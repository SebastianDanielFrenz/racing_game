// rg/fixed_rate_loop.h — rg::FixedRateLoop: a wall-clock-driven fixed-rate
// scheduler around a "try to step, or report frozen" callback (R2.2 plan
// section 2 - the terrain streaming gate: Session::try_step() will return
// false while it is waiting for required terrain tiles to become resident,
// and no ps::World::step() must run on such a tick, per the "clock freeze is
// a pre-step gate racing_game owns" design). Godot-free.
//
// Mirrors external/physics_sim/adapters/godot/src/sim_thread.h's own
// bounded-catch-up shape almost exactly (both a per-wakeup catch-up cap and
// a "resync the deadline after a stall" fallback) - ps_godot::detail::
// precise_sleep_until is reused BY PATH from there rather than copied, since
// it is already the well-tested fix for std::this_thread::sleep_until's
// measured Windows overshoot (see that header's own comment). The one
// behavioural difference from SimThread: here, a tick where the callback
// itself reports "frozen" resyncs the deadline to `now + period` on the
// spot, rather than only doing so after the bounded catch-up is exhausted -
// so a long freeze never leaves a stale, far-in-the-past deadline behind
// that would otherwise cause a catch-up BURST once the callback starts
// stepping again (R2.2 plan section 2's own "no catch-up burst" requirement).
//
// rg::Session::start() runs its tick attempts on one of these (R2.2 R4);
// the primitive's own coverage is tests/unit/test_fixed_rate_loop.cpp, the
// Session-level freeze/no-burst check tests/unit/test_session_terrain.cpp.
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

namespace rg {

class FixedRateLoop {
public:
    // Returns true if this tick actually advanced the simulation ("stepped"),
    // false if it was held back ("frozen" - e.g. a terrain streaming gate
    // not ready yet). Called synchronously on the loop's own thread, once
    // per tick attempt; must not block for long (same contract as
    // ps_godot::SimThread's pre_step/post_step callbacks).
    using TryStep = std::function<bool()>;

    struct LoopStats {
        // Ticks-per-second actually achieved while STEPPING - stepped_count
        // divided by (wall time since start() minus frozen_ms). 0.0 before
        // the first stepped tick.
        double achieved_hz = 0.0;
        // Wall-clock duration of the TryStep() call itself, for STEPPED
        // ticks only, over a bounded ring of the most recent samples
        // (kStepSampleCapacity) - a frozen tick's own call duration is not
        // stepping work and is excluded from this distribution.
        double step_p50_ms = 0.0;
        double step_p99_ms = 0.0;
        double step_max_ms = 0.0;
        // Total wall-clock time spent on ticks where TryStep() returned
        // false, and how many such ticks there were, since start().
        double frozen_ms = 0.0;
        std::uint64_t freeze_count = 0;
        std::uint64_t stepped_count = 0;
    };

    // hz must be > 0. max_catch_up bounds how many STEPPED ticks a single
    // wake-up may run back-to-back before re-checking the wall clock (same
    // bounded catch-up as SimThread's own kMaxCatchUpTicksPerWakeup; default
    // 8 per the R2.2 plan) - a genuine multi-tick stall still resyncs the
    // deadline to "now" rather than replaying an ever-growing backlog.
    explicit FixedRateLoop(double hz, int max_catch_up = 8);
    ~FixedRateLoop();

    FixedRateLoop(const FixedRateLoop&) = delete;
    FixedRateLoop& operator=(const FixedRateLoop&) = delete;

    // Starts the loop thread running `try_step` once per tick. Not
    // thread-safe to call again while running() (mirrors SimThread's own
    // "already running" no-op via running_.exchange).
    void start(TryStep try_step);

    // Stops and joins the loop thread. Safe to call from any thread, and
    // safe to call when not running() (a no-op then). ~FixedRateLoop() calls
    // this.
    void stop();

    [[nodiscard]] bool running() const { return running_.load(std::memory_order_relaxed); }

    // Safe to call from any thread, including while running() - guarded by
    // stats_mutex_ (see the private section below for why this is a plain
    // mutex rather than a lock-free structure).
    [[nodiscard]] LoopStats stats() const;

private:
    void loop();
    void record_stepped_sample(double step_ms);
    void record_frozen_sample(double step_ms);

    const std::chrono::steady_clock::duration period_;
    const int max_catch_up_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    TryStep try_step_; // set once in start() before the thread is launched; only ever read on the loop thread

    static constexpr std::size_t kStepSampleCapacity = 256;

    // Written only by the loop thread; read by stats() from any thread -
    // guarded by one mutex rather than made lock-free, since stats() is a
    // cold/diagnostic path (e.g. a HUD polling a few times a second), never
    // once per tick itself (recording a sample is a handful of stores under
    // the same lock, cheap next to a 240 Hz tick's own budget).
    mutable std::mutex stats_mutex_;
    std::chrono::steady_clock::time_point start_time_{};
    bool started_ = false;
    std::uint64_t stepped_count_ = 0;
    std::uint64_t freeze_count_ = 0;
    double frozen_ms_total_ = 0.0;
    std::array<double, kStepSampleCapacity> step_ms_ring_{};
    std::size_t step_ring_count_ = 0; // <= kStepSampleCapacity
    std::size_t step_ring_next_ = 0;
    double step_ms_max_ = 0.0;
};

} // namespace rg
