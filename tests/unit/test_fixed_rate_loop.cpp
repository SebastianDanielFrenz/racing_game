// test_fixed_rate_loop.cpp — rg::FixedRateLoop coverage (R2.2 plan section
// 2): a stub TryStep that freezes for N ticks then steps shows no catch-up
// burst afterwards; stop() joins promptly; stats() reports sane values.
//
// TOOL-031 note (vault, physics_sim, also cited by this repo's own
// test_world_terrain.cpp): no Catch2 assertion ever runs on the loop's own
// thread - every TryStep stub below only touches std::atomic state, and
// every CHECK/REQUIRE happens on the TEST's own thread after stop()/join.
//
// Wall-clock robustness: every bound below is deliberately generous (this
// is a real OS-scheduled thread, not a mocked clock) - the intent is to
// distinguish "the resync-on-freeze fix works" from "a bug reintroduces an
// 8x-catch-up burst", not to pin exact tick counts.
#include "rg/fixed_rate_loop.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

TEST_CASE("FixedRateLoop: a freeze period causes no catch-up burst once stepping resumes", "[fixed_rate_loop]") {
    using namespace std::chrono;

    constexpr double kHz = 200.0;         // 5 ms period - fast enough for a short test
    constexpr int kFreezeTicks = 40;       // ~200 ms frozen before the stub starts stepping
    std::atomic<int> call_count{0};
    std::atomic<int> step_count{0};

    rg::FixedRateLoop loop(kHz);
    loop.start([&]() -> bool {
        const int n = call_count.fetch_add(1, std::memory_order_relaxed);
        if (n < kFreezeTicks) {
            return false; // frozen
        }
        step_count.fetch_add(1, std::memory_order_relaxed);
        return true;
    });

    // Run through the freeze period and settle into ordinary stepping.
    std::this_thread::sleep_for(milliseconds(400));
    const int steps_after_settle = step_count.load(std::memory_order_relaxed);

    // A short window right after settling: ~50 ms at a 5 ms period is ~10
    // ticks. If the freeze left a stale deadline behind (the bug this test
    // guards against), the loop would instead try to "catch up" using its
    // bounded-catch-up-per-wakeup path (default cap 8) repeatedly across
    // many wake-ups, producing far more steps in this window than a normal
    // cadence ever could.
    std::this_thread::sleep_for(milliseconds(50));
    const int steps_after_window = step_count.load(std::memory_order_relaxed);

    loop.stop();

    const int steps_in_window = steps_after_window - steps_after_settle;
    CHECK(steps_in_window >= 0);
    CHECK(steps_in_window <= 30); // generous: normal cadence ~10-15, a burst bug would be 60+

    const rg::FixedRateLoop::LoopStats stats = loop.stats();
    CHECK(stats.freeze_count >= static_cast<std::uint64_t>(kFreezeTicks));
    CHECK(stats.stepped_count == static_cast<std::uint64_t>(step_count.load(std::memory_order_relaxed)));
    CHECK(stats.frozen_ms >= 0.0);
}

TEST_CASE("FixedRateLoop: stop() joins promptly", "[fixed_rate_loop]") {
    using namespace std::chrono;

    rg::FixedRateLoop loop(100.0); // 10 ms period
    loop.start([]() -> bool { return true; });

    std::this_thread::sleep_for(milliseconds(50));

    const auto t0 = steady_clock::now();
    loop.stop();
    const auto stop_duration = steady_clock::now() - t0;

    CHECK_FALSE(loop.running());
    // Generous bound: precise_sleep_until's own coarse-sleep granularity is
    // ~1 ms chunks plus a ~1.5 ms spin margin, well under one 10 ms period -
    // 500 ms is many multiples of that, comfortably ruling out "stop() never
    // returns" without being a tight timing assertion.
    CHECK(stop_duration <= milliseconds(500));
}

TEST_CASE("FixedRateLoop: stats() are sane for a plain always-stepping loop", "[fixed_rate_loop]") {
    using namespace std::chrono;

    constexpr double kHz = 200.0;
    rg::FixedRateLoop loop(kHz);
    loop.start([]() -> bool {
        // A little bit of real work so step_ms samples are not all exactly
        // 0.0 - not asserted precisely, just gives stats() something to sort.
        volatile int x = 0;
        for (int i = 0; i < 1000; ++i) x += i;
        (void)x;
        return true;
    });

    std::this_thread::sleep_for(milliseconds(300));
    loop.stop();

    const rg::FixedRateLoop::LoopStats stats = loop.stats();
    CHECK(stats.freeze_count == 0);
    CHECK(stats.frozen_ms == 0.0);
    CHECK(stats.stepped_count > 0);
    // Generous bounds: ~300 ms at 200 Hz is ~60 ticks; scheduler jitter on a
    // shared CI machine could plausibly halve or double that, but never
    // wildly exceed the requested rate.
    REQUIRE(stats.achieved_hz > 0.0);
    CHECK(stats.achieved_hz <= kHz * 2.0);
    CHECK(stats.step_p50_ms >= 0.0);
    CHECK(stats.step_p99_ms >= stats.step_p50_ms);
    CHECK(stats.step_max_ms >= stats.step_p99_ms);
}

TEST_CASE("FixedRateLoop: a synthetic 404-style permanent freeze reports failed-but-frozen sanely",
         "[fixed_rate_loop]") {
    // Not a real streaming test (that is R4) - just confirms stats() stay
    // sane (no NaN/negative values, freeze_count keeps growing, no crash on
    // repeated stats() calls) when TryStep never once returns true.
    using namespace std::chrono;

    rg::FixedRateLoop loop(200.0);
    loop.start([]() -> bool { return false; });

    std::this_thread::sleep_for(milliseconds(100));
    const rg::FixedRateLoop::LoopStats mid_stats = loop.stats();
    loop.stop();
    const rg::FixedRateLoop::LoopStats final_stats = loop.stats();

    CHECK(mid_stats.stepped_count == 0);
    CHECK(mid_stats.freeze_count > 0);
    CHECK(mid_stats.achieved_hz == 0.0); // never stepped once
    CHECK(final_stats.freeze_count >= mid_stats.freeze_count);
    CHECK(final_stats.frozen_ms >= mid_stats.frozen_ms);
}
