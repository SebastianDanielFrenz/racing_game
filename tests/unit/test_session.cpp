// test_session.cpp — smoke coverage for rg::Session (PLAN.md R0's "unit
// tests for rg_core" scope): constructs a Session against the submodule's
// own car_sedan.json/surfaces.json, steps it synchronously a few seconds'
// worth of ticks, and checks nothing crashes/NaNs and the chassis stays
// upright and roughly where a free vehicle at rest should be.

#include "rg/drive_script.h"
#include "rg/session.h"

#include "ps/math/quat.h"
#include "ps/math/transcendental.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
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

constexpr double kPi = 3.14159265358979323846;

double wrap_pi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

// Composes a pure yaw (about world +Z) with a rotation about a LOCAL axis
// (applied first, in body space) - used below to build a chassis pose
// rolled/pitched onto a face while still facing a known heading. Rolling
// about the chassis's own local forward axis (unit_x) never changes where
// the forward axis points (a vector is invariant under rotation about
// itself), which is exactly why Session::request_flip_upright's primary yaw
// derivation (from the forward axis) recovers the same yaw whether the car
// is upright, on its side or upside down - only a pitch onto the nose/tail
// (rotating about the local left axis, unit_y) makes the forward axis
// vertical and forces the left-axis fallback.
ps::Pose rolled_pose(const ps::Vec3& position, double yaw_rad, const ps::Vec3& local_axis, double angle_rad) {
    ps::Pose pose;
    pose.position = position;
    const ps::Quat q_yaw = ps::Quat::from_axis_angle(ps::Vec3::unit_z(), yaw_rad);
    const ps::Quat q_local = ps::Quat::from_axis_angle(local_axis, angle_rad);
    pose.orientation = q_yaw * q_local;
    return pose;
}

// Steps up to max_ticks times, stopping early once the chassis is close to
// rest - bounded so a test never spins forever if a setup is not actually
// stable (or, post-flip, so it does not wait out the full budget once the
// car has already settled onto its wheels).
void settle(rg::Session& session, int max_ticks) {
    for (int i = 0; i < max_ticks; ++i) {
        session.step();
        const ps::Motion m = session.world().get_motion(session.chassis_body());
        if (i > 30 && m.linear.length() < 0.02 && m.angular.length() < 0.02) return;
    }
}

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

// Session::request_flip_upright (owner request: put a rolled-over car back
// on its wheels WHERE IT IS, keeping its heading - unlike
// request_reset_to_spawn, which sends it back to spawn). Flat mode,
// synchronous step(): request_flip_upright reads the chassis pose on the
// stepping thread inside take_relocate_request and reuses the relocation
// path (five-ray placement, zeroed motion, drivetrain reset), so these tests
// check the same things the relocate tests would plus the yaw derivation.

TEST_CASE("Session flip upright rights an upside-down car in place", "[session]") {
    rg::Session session(make_test_config());
    const std::uint64_t relocations_before = session.streaming_status().relocations;

    // Rolled 180 deg about the chassis's own forward axis: this leaves the
    // forward axis (and so the recoverable yaw) unchanged, but flips up to
    // down - genuinely upside down.
    constexpr double kYaw = 0.7;
    session.world().backend().set_pose(
        session.chassis_body(), rolled_pose(ps::Vec3{5.0, -3.0, 1.0}, kYaw, ps::Vec3::unit_x(), kPi));
    session.world().backend().set_motion(session.chassis_body(), ps::Motion{});

    settle(session, 480);
    const ps::Pose roof_pose = session.world().get_pose(session.chassis_body());
    REQUIRE(finite(roof_pose.position));
    // Confirm it actually settled upside down before testing the recovery.
    REQUIRE(roof_pose.orientation.rotate(ps::Vec3::unit_z()).z < -0.9);

    session.request_flip_upright();
    settle(session, 720);

    CHECK(session.streaming_status().relocations == relocations_before + 1);

    const ps::Pose righted = session.world().get_pose(session.chassis_body());
    CHECK(righted.orientation.rotate(ps::Vec3::unit_z()).z > 0.99);

    const ps::Vec3 fwd = righted.orientation.rotate(ps::Vec3::unit_x());
    const double yaw = ps::math::atan2(fwd.y, fwd.x);
    CHECK(std::abs(wrap_pi(yaw - kYaw)) < 1e-6);

    CHECK(std::abs(righted.position.x - roof_pose.position.x) < 0.5);
    CHECK(std::abs(righted.position.y - roof_pose.position.y) < 0.5);

    const std::size_t wheel_count = session.world().vehicle_wheel_count(session.vehicle_id());
    REQUIRE(wheel_count == 4);
    double total_load = 0.0;
    bool all_positive = true;
    for (std::size_t i = 0; i < wheel_count; ++i) {
        const ps::vehicle::WheelState ws = session.world().wheel_state(session.vehicle_id(), i);
        total_load += ws.load;
        if (!(ws.load > 0.0)) all_positive = false;
    }
    const double expected_weight = 1500.0 * 9.81; // SessionConfig defaults: chassis_mass_kg, |gravity|
    CHECK((all_positive || std::abs(total_load - expected_weight) < 0.10 * expected_weight));
}

TEST_CASE("Session flip upright rights a car on its side in place", "[session]") {
    rg::Session session(make_test_config());
    const std::uint64_t relocations_before = session.streaming_status().relocations;

    // Rolled 90 deg about the chassis's own forward axis: on its side, not
    // upside down, but the same forward-axis yaw derivation applies.
    constexpr double kYaw = -0.4;
    session.world().backend().set_pose(
        session.chassis_body(), rolled_pose(ps::Vec3{-8.0, 4.0, 1.0}, kYaw, ps::Vec3::unit_x(), kPi * 0.5));
    session.world().backend().set_motion(session.chassis_body(), ps::Motion{});

    // Unlike upside-down (a genuinely stable rest on the flat roof, nothing
    // else touching the ground), on-its-side is NOT stable for a full
    // vehicle: the wheels stick out past the chassis's own narrow half-width
    // (0.4 m) and touch down first, levering the car back upright within a
    // couple of ticks - confirmed by this exact CI run settling back to
    // up.z ~1.0 well inside the old 480-tick pre-settle. Read the pose right
    // after setting it (before any step, same pattern as the nose-down case
    // below) so the "on its side" precondition checks the pose
    // request_flip_upright is actually asked to recover from here.
    const ps::Pose side_pose = session.world().get_pose(session.chassis_body());
    REQUIRE(finite(side_pose.position));
    REQUIRE(std::abs(side_pose.orientation.rotate(ps::Vec3::unit_z()).z) < 0.2);

    session.request_flip_upright();
    settle(session, 720);

    CHECK(session.streaming_status().relocations == relocations_before + 1);

    const ps::Pose righted = session.world().get_pose(session.chassis_body());
    CHECK(righted.orientation.rotate(ps::Vec3::unit_z()).z > 0.99);

    const ps::Vec3 fwd = righted.orientation.rotate(ps::Vec3::unit_x());
    const double yaw = ps::math::atan2(fwd.y, fwd.x);
    CHECK(std::abs(wrap_pi(yaw - kYaw)) < 1e-6);

    CHECK(std::abs(righted.position.x - side_pose.position.x) < 0.5);
    CHECK(std::abs(righted.position.y - side_pose.position.y) < 0.5);

    const std::size_t wheel_count = session.world().vehicle_wheel_count(session.vehicle_id());
    REQUIRE(wheel_count == 4);
    double total_load = 0.0;
    bool all_positive = true;
    for (std::size_t i = 0; i < wheel_count; ++i) {
        const ps::vehicle::WheelState ws = session.world().wheel_state(session.vehicle_id(), i);
        total_load += ws.load;
        if (!(ws.load > 0.0)) all_positive = false;
    }
    const double expected_weight = 1500.0 * 9.81;
    CHECK((all_positive || std::abs(total_load - expected_weight) < 0.10 * expected_weight));
}

TEST_CASE("Session flip upright recovers yaw via the left-axis fallback when nose-down", "[session]") {
    rg::Session session(make_test_config());
    const std::uint64_t relocations_before = session.streaming_status().relocations;

    // Pitched -90 deg about the chassis's own left axis: forward now points
    // straight down (nose-down), so the primary (forward-axis) yaw
    // derivation is degenerate (< 0.2 horizontal length) and
    // request_flip_upright must fall back to the left axis instead.
    constexpr double kYaw = 0.3;
    session.world().backend().set_pose(
        session.chassis_body(), rolled_pose(ps::Vec3{12.0, 6.0, 3.0}, kYaw, ps::Vec3::unit_y(), kPi * 0.5));
    session.world().backend().set_motion(session.chassis_body(), ps::Motion{});

    const ps::Pose nose_down = session.world().get_pose(session.chassis_body());
    REQUIRE(nose_down.orientation.rotate(ps::Vec3::unit_x()).z < -0.9);

    session.request_flip_upright();
    settle(session, 720);

    CHECK(session.streaming_status().relocations == relocations_before + 1);

    const ps::Pose righted = session.world().get_pose(session.chassis_body());
    CHECK(righted.orientation.rotate(ps::Vec3::unit_z()).z > 0.99);

    const ps::Vec3 fwd = righted.orientation.rotate(ps::Vec3::unit_x());
    const double yaw = ps::math::atan2(fwd.y, fwd.x);
    CHECK(std::abs(wrap_pi(yaw - kYaw)) < 1e-6);

    CHECK(std::abs(righted.position.x - nose_down.position.x) < 0.5);
    CHECK(std::abs(righted.position.y - nose_down.position.y) < 0.5);
}

TEST_CASE("Session flip upright is superseded by a pending relocate in the same tick", "[session]") {
    rg::Session session(make_test_config());
    for (int i = 0; i < 60; ++i) session.step(); // settle at spawn first
    const std::uint64_t relocations_before = session.streaming_status().relocations;

    // Both requests are pending before the next tick attempt consumes
    // either: the explicit relocate must win outright (it already lands
    // upright, so the flip request is simply dropped - session.h's
    // request_flip_upright doc comment).
    session.request_flip_upright();
    constexpr double kTargetYaw = 1.2;
    session.request_relocate(50.0, 20.0, kTargetYaw);

    settle(session, 720);

    // Exactly one relocation happened - the flip was dropped outright, not
    // queued for a later tick.
    CHECK(session.streaming_status().relocations == relocations_before + 1);

    const ps::Pose pose = session.world().get_pose(session.chassis_body());
    CHECK(std::abs(pose.position.x - 50.0) < 0.5);
    CHECK(std::abs(pose.position.y - 20.0) < 0.5);
    const ps::Vec3 fwd = pose.orientation.rotate(ps::Vec3::unit_x());
    const double yaw = ps::math::atan2(fwd.y, fwd.x);
    CHECK(std::abs(wrap_pi(yaw - kTargetYaw)) < 1e-6);
}

TEST_CASE("Session publishes wheel telemetry for audio at the captured tick", "[session][audio]") {
    auto config = make_test_config();
    config.chassis_initial_velocity = {12.0, 0.0, 0.0};
    rg::Session session(config);
    session.set_control("brake", 0.5);
    session.start();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (session.snapshot().tick < 120 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    session.stop(); // world comparisons are safe only after its worker joins
    bool saw_force = false;
    {
        const auto& frame = session.snapshot();
        REQUIRE(frame.tick >= 120);
        REQUIRE(frame.tick == session.world().tick());
        for (std::size_t wheel = 0; wheel < frame.wheels.size(); ++wheel) {
            const auto expected = session.world().wheel_telemetry(session.vehicle_id(), wheel);
            const auto& actual = frame.wheels[wheel].telemetry;
            REQUIRE(actual.omega == expected.omega);
            REQUIRE(actual.fx == expected.fx);
            REQUIRE(actual.fy == expected.fy);
            REQUIRE(actual.fz == frame.wheels[wheel].state.load);
            REQUIRE(std::isfinite(actual.omega));
            REQUIRE(std::isfinite(actual.fx));
            REQUIRE(std::isfinite(actual.fy));
            saw_force = saw_force || std::abs(actual.fx) > 1.0;
        }
    }
    REQUIRE(saw_force);
}
