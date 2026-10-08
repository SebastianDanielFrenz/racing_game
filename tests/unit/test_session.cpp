// test_session.cpp — smoke coverage for rg::Session (PLAN.md R0's "unit
// tests for rg_core" scope): constructs a Session against the submodule's
// own car_sedan.json/surfaces.json, steps it synchronously a few seconds'
// worth of ticks, and checks nothing crashes/NaNs and the chassis stays
// upright and roughly where a free vehicle at rest should be.

#include "rg/drive_script.h"
#include "rg/session.h"
#include "rg/aero_map_selection.h"
#include "ps/aero/coefficient_map.h"
#include "g2m/core/hash.h"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>

#include "ps/math/quat.h"
#include "ps/math/transcendental.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
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

// The N2O arm switch (gamepad Y toggles it, main.gd): a pre-seeded plain 0/1 level that the
// real-time loop copies into the World like every other channel; the shipped cars have no
// nitrous kit, so for them it is inert and must not disturb stepping.
TEST_CASE("Session nitrous_arm channel is pre-seeded and reaches the World", "[session]") {
    rg::Session session(make_test_config());
    CHECK(session.get_control("nitrous_arm") == 0.0);
    session.set_control("nitrous_arm", 1.0 - session.get_control("nitrous_arm")); // the toggle
    CHECK(session.get_control("nitrous_arm") == 1.0);
    session.start();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (session.snapshot().tick < 30 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    session.stop();
    CHECK(session.snapshot().tick >= 30);
    CHECK(session.world().get_control("nitrous_arm") == 1.0);
    session.set_control("nitrous_arm", 1.0 - session.get_control("nitrous_arm"));
    CHECK(session.get_control("nitrous_arm") == 0.0);
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

TEST_CASE("Session rebuilds from a retained immutable vehicle definition", "[session][audio]") {
    auto config = make_test_config();
    rg::Session original(config);
    config.vehicle_definition = std::make_shared<const ps::vehicle::VehicleDesc>(original.vehicle_desc());
    config.vehicle_json_path = "missing/vehicle.json"; // no second file read
    rg::Session rebuilt(config);
    REQUIRE(rebuilt.vehicle_desc().name == original.vehicle_desc().name);
    REQUIRE(rebuilt.vehicle_desc().wheels.size() == original.vehicle_desc().wheels.size());
    rebuilt.set_control("throttle", 1.0);
    for (int i = 0; i < 60; ++i) rebuilt.step();
    REQUIRE(original.world().tick() == 0);
    REQUIRE(rebuilt.world().tick() == 60);
    REQUIRE(config.vehicle_definition->wheels.size() == original.vehicle_desc().wheels.size());
}

TEST_CASE("Engine audio publishes without render reads and detaches safely", "[session][audio_publisher]") {
    rg::Session session(make_test_config());
    std::atomic<int> calls{0};
    session.set_engine_audio_publisher([&](const ps::drivetrain::EngineSoundState&,double){
        calls.fetch_add(1);
    });
    session.start();
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    // No snapshot/render reader is involved in engine-input delivery.
    while(calls.load()<12 && std::chrono::steady_clock::now()<deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    session.set_engine_audio_publisher({});
    const int detached=calls.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    session.stop();
    REQUIRE(detached>=12);
    REQUIRE(calls.load()==detached);
    // A fresh publisher after restarting is not tied to the prior voice.
    session.set_engine_audio_publisher([&](const ps::drivetrain::EngineSoundState&,double){calls.fetch_add(1);});
    session.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    session.set_engine_audio_publisher({});session.stop();
    REQUIRE(calls.load()>detached);
}

// R9b: following a vehicle (rg::Session::set_followed_vehicle) in flat mode -
// the actor is kept alive while followed, and a follow whose vehicle does not
// exist (despawned, never existed, no truck) is cleared and counted.
TEST_CASE("session follow: a followed actor survives, a lost follow is cleared and counted", "[session][follow]") {
    rg::SessionConfig config = make_test_config();
    config.job_workers = 1;
    rg::Session session(config);
    ps::Pose pose;
    pose.position = ps::Vec3{40.0, 0.0, 0.8};
    const std::uint64_t followed = session.add_test_traffic_actor(pose, 5.0);
    pose.position = ps::Vec3{40.0, 10.0, 0.8};
    const std::uint64_t other = session.add_test_traffic_actor(pose, 5.0);
    CHECK(followed != other);
    CHECK(followed < rg::Session::kNpcTruckVehicleId);
    CHECK(rg::Session::kNpcTruckVehicleId <= static_cast<std::uint64_t>(INT64_MAX)); // a positive Godot int

    session.set_followed_vehicle(followed);
    CHECK(session.followed_vehicle() == std::optional<std::uint64_t>{followed});
    const std::uint64_t lost0 = session.streaming_status().followed_lost;
    // 5 s of sim time: an unfollowed, unseen surplus actor (flat-mode target 0) goes after > 3 s.
    for (int k = 0; k < 1200; ++k) session.step();
    CHECK(session.followed_vehicle() == std::optional<std::uint64_t>{followed}); // survived
    CHECK(session.streaming_status().followed_lost == lost0);
    CHECK(session.streaming_status().followed_id == followed);

    // The unfollowed one is gone by now: following it is a lost follow.
    session.set_followed_vehicle(other);
    session.step();
    CHECK_FALSE(session.followed_vehicle().has_value());
    rg::StreamingStatus st = session.streaming_status();
    CHECK(st.followed_lost == lost0 + 1);
    CHECK(st.followed_lost_id == other);
    CHECK(st.followed_id == 0);

    // An id that never existed, and the truck id while no truck exists.
    session.set_followed_vehicle(std::uint64_t{987654});
    session.step();
    CHECK(session.streaming_status().followed_lost == lost0 + 2);
    CHECK(session.streaming_status().followed_lost_id == 987654);
    session.set_followed_vehicle(rg::Session::kNpcTruckVehicleId);
    session.step();
    CHECK(session.streaming_status().followed_lost == lost0 + 3);
    CHECK(session.streaming_status().followed_lost_id == rg::Session::kNpcTruckVehicleId);

    // Clearing the follow is not a loss; the id 0 means none.
    session.set_followed_vehicle(followed);
    session.set_followed_vehicle(std::nullopt);
    session.step();
    CHECK_FALSE(session.followed_vehicle().has_value());
    session.set_followed_vehicle(std::uint64_t{0});
    CHECK_FALSE(session.followed_vehicle().has_value());
    CHECK(session.streaming_status().followed_lost == lost0 + 3);
    CHECK(session.last_interest_points().empty()); // flat mode has no interest points
}

// Pause (the shell's pause menu): the real-time loop is held back like a
// terrain-gate freeze - the published tick stops advancing, resuming continues
// from there, and the simulation never replays the paused wall time (no
// catch-up burst: the tick count after a 0.6 s pause is a handful of ticks
// beyond the pre-pause count, nowhere near the ~144 a 0.6 s backlog would be).
TEST_CASE("Session pause freezes the real-time loop and resumes without a burst", "[session][pause]") {
    rg::Session session(make_test_config());
    CHECK_FALSE(session.paused());
    session.start();
    const auto wait_for_tick = [&](std::uint64_t tick) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (session.snapshot().tick < tick && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return session.snapshot().tick >= tick;
    };
    REQUIRE(wait_for_tick(60));

    session.set_paused(true);
    CHECK(session.paused());
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // let an in-flight tick land
    const std::uint64_t held = session.snapshot().tick;
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    const std::uint64_t still = session.snapshot().tick;
    CHECK(still == held);

    session.set_paused(false);
    REQUIRE(wait_for_tick(held + 10));
    // 50 ms after resuming: if the 0.6 s had been replayed the tick would be
    // ~144 ahead; real time allows about 12.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const std::uint64_t resumed = session.snapshot().tick;
    CHECK(resumed < held + 60);
    session.stop();
}

// FrameSnapshot::road_ahead is empty and costs nothing unless asked for; in
// the flat world (no road data) it stays empty even when asked.
TEST_CASE("Session road_ahead stays empty in the flat world", "[session][road_ahead]") {
    rg::Session session(make_test_config());
    session.set_road_ahead_wanted(true);
    session.start();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (session.snapshot().tick < 60 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    session.stop();
    CHECK(session.snapshot().tick >= 60);
    CHECK(session.snapshot().road_ahead.empty());
}

TEST_CASE("Game aero selection binds reviewed bytes and rejects stale geometry", "[aero_map_selection]") {
    using nlohmann::json;
    const auto dir=std::filesystem::temp_directory_path()/("rg_aero_map_test_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(dir);
    struct Cleanup { std::filesystem::path path;~Cleanup(){std::error_code error;std::filesystem::remove_all(path,error);} } cleanup{dir};
    auto write=[&](const char* name,const std::string& text){std::ofstream file(dir/name,std::ios::binary);file<<text;};
    auto digest=[](const std::string& text){return g2m::to_hex(g2m::Sha256::of(std::string_view(text)));};
    // Synthetic schema fixture only; never shipped or represented as measured CFD.
    const std::string geometry="synthetic test geometry";write("geometry.bin",geometry);
    const auto geometry_sha=digest(geometry);
    json rows=json::array();for(int i=0;i<6;++i)rows.push_back({-.3,0,0,0,0,0});
    json map={{"schema","physics_sim.aero_coefficients/1"},{"frame","ISO_BODY_X_FORWARD_Y_LEFT_Z_UP"},{"units","SI_DEGREES_DIMENSIONLESS_COEFFICIENTS"},
        {"reference",{{"area_m2",1.9},{"lengths_m",{2.7,2.7,2.7}},{"point_local_m",{0,0,0}}}},
        {"provenance",{{"geometry_sha256",geometry_sha},{"solver","synthetic unit fixture"},{"source","not CFD"}}},
        {"axes",{{"yaw_deg",{-180,0,180}},{"pitch_deg",{-90,90}},{"speed_m_s",{20}}}},{"coefficients",rows}};
    const auto encoded=map.dump();write("map.json",encoded);
    json proof={{"validated",false},{"kind","cfd"},{"map_sha256",digest(encoded)},{"geometry_sha256",geometry_sha},
        {"geometry",json::array({{{"path","geometry.bin"},{"sha256",geometry_sha}}})}};
    write("map.json.provenance.json",proof.dump());
    json selection={{"enabled",false},{"coefficient_map",{{"file","map.json"},{"scope","replace_body"}}}};write("selection.json",selection.dump());
    ps::vehicle::VehicleDesc vehicle;
    rg::apply_aero_map_selection((dir/"selection.json").string(),vehicle);
    CHECK_FALSE(vehicle.aero.coefficient_map.table);
    selection["enabled"]=true;write("selection.json",selection.dump());
    CHECK_THROWS(rg::apply_aero_map_selection((dir/"selection.json").string(),vehicle));
    CHECK_FALSE(vehicle.aero.coefficient_map.table);
    proof["validated"]=true;write("map.json.provenance.json",proof.dump());
    rg::apply_aero_map_selection((dir/"selection.json").string(),vehicle);
    REQUIRE(vehicle.aero.coefficient_map.table);
    const auto retained=vehicle.aero.coefficient_map.table;
    write("geometry.bin","changed geometry");
    CHECK_THROWS(rg::apply_aero_map_selection((dir/"selection.json").string(),vehicle));
    CHECK(vehicle.aero.coefficient_map.table==retained);
    write("geometry.bin",geometry);write("map.json",encoded+" ");
    CHECK_THROWS(rg::apply_aero_map_selection((dir/"selection.json").string(),vehicle));
    selection["enabled"]=false;write("selection.json",selection.dump());
    rg::apply_aero_map_selection((dir/"selection.json").string(),vehicle);
    CHECK_FALSE(vehicle.aero.coefficient_map.table);
    ps::aero::SurfaceDesc wing;
    wing.name="test_wing";wing.max_offset_rad=1;wing.max_lift_m=.28;
    vehicle.aero.surfaces.push_back(wing);
    selection["enabled"]=true;
    selection["coefficient_map"]["scope"]="replace_passive_aero";
    selection["coefficient_map"]["wing_surface"]="test_wing";
    write("selection.json",selection.dump());
    map["axes"]["wing_offset_deg"]={0,55};
    auto publish=[&](int count) {
        map["coefficients"]=json::array();
        for(int i=0;i<count;++i)map["coefficients"].push_back({-.3,0,0,0,0,0});
        const auto bytes=map.dump();write("map.json",bytes);
        proof["map_sha256"]=digest(bytes);write("map.json.provenance.json",proof.dump());
    };
    publish(12);
    CHECK_THROWS_WITH(rg::apply_aero_map_selection((dir/"selection.json").string(),vehicle),
        "full passive aero map for a lifting wing requires wing_lift_m samples and wing_surface binding");
    CHECK_FALSE(vehicle.aero.coefficient_map.table);
    map["axes"]["wing_lift_m"]={0,.28};publish(24);
    rg::apply_aero_map_selection((dir/"selection.json").string(),vehicle);
    REQUIRE(vehicle.aero.coefficient_map.table);
    CHECK(vehicle.aero.coefficient_map.table->data().axes[5].size()==2);
    vehicle.aero.surfaces[0].max_lift_m=0;
    map["axes"].erase("wing_lift_m");publish(12);
    rg::apply_aero_map_selection((dir/"selection.json").string(),vehicle);
    CHECK(vehicle.aero.coefficient_map.table->data().axes[5].empty());
}

TEST_CASE("Session rolling resistance selection survives immutable cached reloads", "[session][rolling_resistance]") {
    auto config=make_test_config();
    rg::Session quadratic(config);
    REQUIRE_FALSE(quadratic.vehicle_desc().wheels.empty());
    for (const auto& wheel : quadratic.vehicle_desc().wheels)
        CHECK(wheel.tyre.rolling_resistance_speed_law==ps::vehicle::RollingResistanceSpeedLaw::Quadratic);
    config.vehicle_definition=std::make_shared<const ps::vehicle::VehicleDesc>(quadratic.vehicle_desc());
    config.vehicle_json_path="missing/cached_vehicle.json";
    config.rolling_resistance_model="fourth_power";
    rg::Session legacy(config);
    for (const auto& wheel : legacy.vehicle_desc().wheels)
        CHECK(wheel.tyre.rolling_resistance_speed_law==ps::vehicle::RollingResistanceSpeedLaw::FourthPower);
    for (const auto& wheel : config.vehicle_definition->wheels)
        CHECK(wheel.tyre.rolling_resistance_speed_law==ps::vehicle::RollingResistanceSpeedLaw::Quadratic);
    config.rolling_resistance_model="invalid";
    std::string error;
    CHECK_FALSE(rg::make_session(config,&error));
    CHECK(error.find("rolling_resistance_model")!=std::string::npos);
}