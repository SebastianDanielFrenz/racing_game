// rg/fixed_rate_loop.cpp — see rg/fixed_rate_loop.h for the public contract.
#include "rg/fixed_rate_loop.h"

// ps_godot::detail::precise_sleep_until, reused BY PATH from physics_sim's
// own adapter (see fixed_rate_loop.h's top comment) - already on rg_core's
// public include path (core/CMakeLists.txt's PS_ADAPTER_SRC_DIR), Godot-free.
#include "sim_thread.h"

// sim_thread.h unconditionally #includes <windows.h> without NOMINMAX (it is
// physics_sim's own read-only file), which leaves the min/max macros active
// and breaks every std::min/std::max call below (they expand as function-
// like macros instead of being looked up as templates) - undef them right
// after the only include that can define them, same fix physics_sim's own
// code would need if it called std::min/std::max after including this
// header.
#if defined(min)
#undef min
#endif
#if defined(max)
#undef max
#endif

#include <algorithm>

namespace rg {

namespace {

std::chrono::steady_clock::duration hz_to_period(double hz) {
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / hz));
}

} // namespace

FixedRateLoop::FixedRateLoop(double hz, int max_catch_up) : period_(hz_to_period(hz)), max_catch_up_(max_catch_up) {}

FixedRateLoop::~FixedRateLoop() { stop(); }

void FixedRateLoop::start(TryStep try_step) {
    if (running_.exchange(true)) return; // already running

#if defined(_WIN32)
    // Same fix as ps_godot::SimThread::start() (sim_thread.h, already
    // included above): without raising the process's timer resolution from
    // Windows' default (~15.6 ms) to 1 ms, precise_sleep_until's own coarse
    // sleep_for(1ms) chunks actually sleep ~15.6 ms each, so a short period
    // (e.g. 200 Hz = 5 ms) is missed by 2-3x - measured directly here (a
    // freeze/step test relying on ~5 ms cadence saw ~13 ms/tick without
    // this call).
    timeBeginPeriod(1);
#endif

    try_step_ = std::move(try_step);
    {
        std::lock_guard<std::mutex> lk(stats_mutex_);
        start_time_ = std::chrono::steady_clock::now();
        started_ = true;
        stepped_count_ = 0;
        freeze_count_ = 0;
        frozen_ms_total_ = 0.0;
        step_ring_count_ = 0;
        step_ring_next_ = 0;
        step_ms_max_ = 0.0;
    }
    thread_ = std::thread([this] { loop(); });
}

void FixedRateLoop::stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
#if defined(_WIN32)
    timeEndPeriod(1);
#endif
}

void FixedRateLoop::record_stepped_sample(double step_ms) {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    ++stepped_count_;
    step_ms_ring_[step_ring_next_] = step_ms;
    step_ring_next_ = (step_ring_next_ + 1) % kStepSampleCapacity;
    step_ring_count_ = std::min(step_ring_count_ + 1, kStepSampleCapacity);
    step_ms_max_ = std::max(step_ms_max_, step_ms);
}

void FixedRateLoop::record_frozen_sample(double step_ms) {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    ++freeze_count_;
    frozen_ms_total_ += step_ms;
}

void FixedRateLoop::loop() {
    using clock = std::chrono::steady_clock;

    auto next = clock::now() + period_;
    while (running_.load(std::memory_order_relaxed)) {
        ps_godot::detail::precise_sleep_until(next);

        int ran = 0;
        while (running_.load(std::memory_order_relaxed) && clock::now() >= next && ran < max_catch_up_) {
            const auto t0 = clock::now();
            const bool stepped = try_step_();
            const auto t1 = clock::now();
            const double dur_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

            if (stepped) {
                record_stepped_sample(dur_ms);
                next += period_;
                ++ran;
            } else {
                record_frozen_sample(dur_ms);
                // Resync the deadline to "now" (NOT next += period_): a
                // frozen tick must never leave a stale, far-in-the-past
                // deadline behind that would cause a catch-up BURST once
                // try_step_ starts returning true again (R2.2 plan section 2
                // - "no catch-up burst" after a freeze). Also deliberately
                // stop attempting further catch-up this wake-up: one frozen
                // attempt per wake-up, not kMaxCatchUp of them.
                next = clock::now() + period_;
                break;
            }
        }
        if (clock::now() >= next) {
            // Still behind after the bounded catch-up - a genuine stall
            // (mirrors SimThread's own handling exactly, e.g. the process
            // was paused). Resync to "now" rather than an ever-growing
            // backlog; a fixed-rate cadence resuming late is intended, not a
            // catch-up spiral.
            next = clock::now() + period_;
        }
    }
}

FixedRateLoop::LoopStats FixedRateLoop::stats() const {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    LoopStats out;
    out.freeze_count = freeze_count_;
    out.stepped_count = stepped_count_;
    out.frozen_ms = frozen_ms_total_;
    out.step_max_ms = step_ms_max_;

    if (started_ && stepped_count_ > 0) {
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time_).count();
        const double unfrozen_ms = std::max(0.0, elapsed_ms - frozen_ms_total_);
        if (unfrozen_ms > 0.0) {
            out.achieved_hz = static_cast<double>(stepped_count_) / (unfrozen_ms / 1000.0);
        }
    }

    if (step_ring_count_ > 0) {
        std::array<double, kStepSampleCapacity> sorted = step_ms_ring_;
        std::sort(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(step_ring_count_));
        const auto pick = [&](double fraction) {
            std::size_t idx = static_cast<std::size_t>(fraction * static_cast<double>(step_ring_count_ - 1));
            idx = std::min(idx, step_ring_count_ - 1);
            return sorted[idx];
        };
        out.step_p50_ms = pick(0.50);
        out.step_p99_ms = pick(0.99);
    }
    return out;
}

} // namespace rg
