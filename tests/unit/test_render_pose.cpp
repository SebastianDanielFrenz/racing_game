// test_render_pose.cpp - render-time pose sampling on the sim's NOMINAL clock
// (physics_sim 2d8f0b8 port, tag [render_pose]):
//  - rg::FixedRateLoop stamps each tick attempt with its scheduled due time
//    (tick_nominal_time) and flags schedule re-anchors (tick_clock_break);
//  - rg::Session pushes the chassis pose of every real-time tick into a
//    ps_godot::PoseHistory with those stamps, and a relocation is rendered as
//    a step, never a smear across the map.
// PoseHistory's own blend math is covered by physics_sim's
// adapters/godot/tests/test_render_interp.cpp; this file covers the wiring.
//
// TOOL-031: no Catch2 assertion runs on the loop thread - the TryStep stub
// only records into a pre-sized vector, every CHECK happens after stop().

#include "rg/fixed_rate_loop.h"
#include "rg/session.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct TickStamp {
    Clock::time_point nominal;
    bool clock_break = false;
    bool stepped = false;
};

rg::SessionConfig make_flat_config() {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json";
    return config;
}

} // namespace

TEST_CASE("FixedRateLoop: nominal tick times are exactly one period apart whatever the step costs", "[fixed_rate_loop][render_pose]") {
    constexpr double kHz = 200.0; // 5 ms period
    rg::FixedRateLoop loop(kHz);
    std::vector<TickStamp> stamps(512);
    std::atomic<int> n{0};
    std::atomic<bool> done{false};
    loop.start([&]() -> bool {
        const int i = n.load(std::memory_order_relaxed);
        if (i >= static_cast<int>(stamps.size())) {
            done.store(true, std::memory_order_relaxed);
            return true;
        }
        stamps[static_cast<std::size_t>(i)] = TickStamp{loop.tick_nominal_time(), loop.tick_clock_break(), true};
        n.store(i + 1, std::memory_order_relaxed);
        // Jittery step cost, up to 80 % of the 5 ms period (a late publish).
        std::this_thread::sleep_for(std::chrono::microseconds(500 * (i % 8)));
        return true;
    });
    const auto deadline = Clock::now() + std::chrono::seconds(5);
    while (n.load() < 60 && Clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    loop.stop();

    const int count = n.load();
    REQUIRE(count >= 40);
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / kHz));
    CHECK(std::abs(loop.period_s() - 1.0 / kHz) < 1e-8);
    CHECK(stamps[0].clock_break); // the first attempt starts a schedule
    int breaks = 0;
    int off_period = 0;
    for (int i = 1; i < count; ++i) {
        const auto& s = stamps[static_cast<std::size_t>(i)];
        const auto& p = stamps[static_cast<std::size_t>(i - 1)];
        if (s.clock_break) {
            ++breaks;
            continue; // a re-anchor: the gap says nothing about motion
        }
        if (s.nominal - p.nominal != period) ++off_period;
    }
    CHECK(off_period == 0); // exact: `next += period_`, however long each step took
    CHECK(breaks <= 1);     // a loaded machine may add one stall resync, never a steady stream
}

TEST_CASE("FixedRateLoop: a freeze resync is flagged as a clock break on the next tick", "[fixed_rate_loop][render_pose]") {
    constexpr double kHz = 200.0;
    rg::FixedRateLoop loop(kHz);
    std::vector<TickStamp> stamps(512);
    std::atomic<int> calls{0};
    std::atomic<int> recorded{0};
    loop.start([&]() -> bool {
        const int call = calls.fetch_add(1, std::memory_order_relaxed);
        const bool frozen = call >= 20 && call < 24; // held back, like a terrain gate
        const int i = recorded.load(std::memory_order_relaxed);
        if (!frozen && i < static_cast<int>(stamps.size())) {
            stamps[static_cast<std::size_t>(i)] = TickStamp{loop.tick_nominal_time(), loop.tick_clock_break(), true};
            recorded.store(i + 1, std::memory_order_relaxed);
        }
        return !frozen;
    });
    const auto deadline = Clock::now() + std::chrono::seconds(5);
    while (recorded.load() < 60 && Clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    loop.stop();

    const int count = recorded.load();
    REQUIRE(count >= 40);
    // Stepped ticks 0..19 came before the freeze; tick 20 is the first one after it.
    CHECK(stamps[0].clock_break);
    CHECK_FALSE(stamps[5].clock_break);
    CHECK_FALSE(stamps[19].clock_break);
    CHECK(stamps[20].clock_break);
    CHECK(stamps[20].nominal > stamps[19].nominal); // strictly increasing across the break
}

TEST_CASE("Session: the render pose history is empty until the real-time loop runs", "[session][render_pose]") {
    rg::Session session(make_flat_config());
    ps_godot::RenderSample sample;
    CHECK_FALSE(session.sample_render_poses(Clock::now(), sample));
    CHECK(session.tick_period_s() == 1.0 / 240.0);
    for (int i = 0; i < 20; ++i) session.step(); // synchronous stepping never fills it
    CHECK_FALSE(session.sample_render_poses(Clock::now(), sample));
}

TEST_CASE("Session: render samples follow the chassis and a relocation is a step, not a smear", "[session][render_pose]") {
    rg::Session session(make_flat_config());
    session.start();
    const auto wait_for_tick = [&](std::uint64_t tick) {
        const auto deadline = Clock::now() + std::chrono::seconds(5);
        while (session.snapshot().tick < tick && Clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(2));
        return session.snapshot().tick >= tick;
    };
    REQUIRE(wait_for_tick(240)); // 1 s: the car has settled on its springs
    const double dt = session.tick_period_s();
    constexpr double kDelayTicks = 2.0;

    // Settled: the sample at now - 2 ticks is valid, inside the history (not late) and where the car is.
    ps_godot::RenderSample sample;
    REQUIRE(session.sample_render_poses(ps_godot::render_time_for(Clock::now(), dt, kDelayTicks), sample));
    REQUIRE(sample.valid);
    REQUIRE(sample.poses.size() == 1);
    CHECK_FALSE(sample.late);
    CHECK_FALSE(sample.early);
    const ps::Pose before = session.snapshot().chassis_pose;
    const ps::Vec3 d0 = sample.poses[0].position - before.position;
    CHECK(std::sqrt(d0.x * d0.x + d0.y * d0.y + d0.z * d0.z) < 0.05);

    // Relocate 60 m away while sampling every ~1 ms: every rendered position is near the old
    // spot or the new one (a blend across the teleport would pass through the 30 m midpoint).
    session.request_relocate(before.position.x + 60.0, before.position.y, 0.0);
    double worst_min_dist = 0.0;
    double closest_to_new = 1e9;
    int samples = 0;
    const auto end = Clock::now() + std::chrono::milliseconds(700);
    while (Clock::now() < end) {
        if (session.sample_render_poses(ps_godot::render_time_for(Clock::now(), dt, kDelayTicks), sample) && sample.valid) {
            const double dx_old = sample.poses[0].position.x - before.position.x;
            const double dy_old = sample.poses[0].position.y - before.position.y;
            const double dx_new = sample.poses[0].position.x - (before.position.x + 60.0);
            const double dy_new = sample.poses[0].position.y - before.position.y;
            const double d_old = std::sqrt(dx_old * dx_old + dy_old * dy_old);
            const double d_new = std::sqrt(dx_new * dx_new + dy_new * dy_new);
            worst_min_dist = std::max(worst_min_dist, std::min(d_old, d_new));
            closest_to_new = std::min(closest_to_new, d_new);
            ++samples;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    session.stop();
    CHECK(session.streaming_status().relocations == 1);
    CHECK(samples > 200);
    CHECK(worst_min_dist < 2.0); // never a smeared pose between the two places
    CHECK(closest_to_new < 1.0); // and the render did reach the new place
}
