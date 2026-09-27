// test_session.cpp — smoke coverage for rg::Session (PLAN.md R0's "unit
// tests for rg_core" scope): constructs a Session against the submodule's
// own car_sedan.json/surfaces.json, steps it synchronously a few seconds'
// worth of ticks, and checks nothing crashes/NaNs and the chassis stays
// upright and roughly where a free vehicle at rest should be.

#include "rg/drive_script.h"
#include "rg/session.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

namespace {

// RG_SOURCE_DIR is baked in at configure time (CMakeLists.txt) - the WSL/
// Linux preset builds outside the source tree entirely, same reasoning as
// physics_sim's own PS_SOURCE_DIR (see e.g.
// external/physics_sim/tests/unit/test_scenario_articulation.cpp).
rg::SessionConfig make_test_config() {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json";
    return config;
}

bool finite(const ps::Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

} // namespace

TEST_CASE("Session constructs and steps without crashing", "[session]") {
    rg::Session session(make_test_config());

    REQUIRE(session.world().vehicle_wheel_count(session.vehicle_id()) > 0);

    // 2 seconds at the session's own 240 Hz default tick rate.
    for (int i = 0; i < 480; ++i) session.step();

    const ps::Pose pose = session.world().get_pose(session.chassis_body());
    REQUIRE(finite(pose.position));
    // A car spawned at rest on flat ground and left alone (no throttle, no
    // steer) should settle close to where it started, not fly off or fall
    // through the ground.
    REQUIRE(pose.position.z > 0.0);
    REQUIRE(pose.position.z < 2.0);
    REQUIRE(std::abs(pose.position.x) < 5.0);
    REQUIRE(std::abs(pose.position.y) < 5.0);
}

TEST_CASE("Session control channels round-trip", "[session]") {
    rg::Session session(make_test_config());

    session.set_control("steer", 0.5);
    REQUIRE(session.get_control("steer") == 0.5);

    // Unknown channel: documented no-op / reads back 0.
    session.set_control("not_a_real_channel", 1.0);
    REQUIRE(session.get_control("not_a_real_channel") == 0.0);
}

TEST_CASE("Session snapshot captures wheel and powertrain state", "[session]") {
    rg::Session session(make_test_config());
    for (int i = 0; i < 10; ++i) session.step();

    // Synchronous mode never publishes into the triple buffer (only
    // start()'s SimThread post_step hook does) - this test exercises the
    // capture path indirectly via the direct world()/vehicle accessors that
    // capture_frame_snapshot() itself uses, since start() spins a real
    // wall-clock thread unsuitable for a fast, deterministic unit test.
    const std::size_t wheel_count = session.world().vehicle_wheel_count(session.vehicle_id());
    REQUIRE(wheel_count == 4);
    for (std::size_t i = 0; i < wheel_count; ++i) {
        const ps::vehicle::WheelState ws = session.world().wheel_state(session.vehicle_id(), i);
        REQUIRE(std::isfinite(ws.load));
    }
}

// Tick-spike diagnostics (rg::Session::drain_tick_spikes): one real-time
// loop attempt made slow on purpose (the drive script sleeps inside it) is
// queued with its time attributed to the controls phase, and formats into
// the key=value line the game prints as RG_TICK_SPIKE. Synchronous step()
// attempts are never judged.
TEST_CASE("Session queues a slow real-time tick as a tick spike", "[session]") {
    rg::Session session(make_test_config());
    for (int i = 0; i < 10; ++i) session.step();
    REQUIRE(session.drain_tick_spikes().empty());

    rg::DriveScript script;
    script.set_controller([](const rg::DriveTickContext& ctx, ps::World&) {
        if (ctx.drive_tick == 30) std::this_thread::sleep_for(std::chrono::milliseconds(15));
    });
    session.set_drive_script(std::move(script));
    session.start();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (session.snapshot().tick < 120 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    session.stop();

    std::uint64_t overflow = 0;
    const std::vector<rg::Session::TickSpike> spikes = session.drain_tick_spikes(&overflow);
    CHECK(overflow == 0);
    bool found = false;
    for (const rg::Session::TickSpike& s : spikes) {
        if (s.kind == rg::Session::TickSpikeKind::Stepped && s.controls_ms >= 14.0) {
            found = true;
            CHECK(s.total_ms >= s.controls_ms);
            CHECK(s.step_ms < s.controls_ms);
            const std::string line = rg::Session::format_tick_spike(s);
            INFO(line);
            CHECK(line.find("kind=stepped") != std::string::npos);
            CHECK(line.find("controls_ms=") != std::string::npos);
        }
    }
    CHECK(found);
    CHECK(session.drain_tick_spikes().empty());
}
