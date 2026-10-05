// test_player_mode.cpp — rg::PlayerModeMachine / rules_for /
// unattended_controls (R2.2 R9, rg/player_mode.h), plus the Session side of
// the unattended rule (the real-time loop's control copy).
//
// TOOL-031: no Catch2 assertion runs on the loop thread - the World's
// control values are read only after stop().
#include "rg/player_mode.h"
#include "rg/session.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

rg::SessionConfig flat_config() {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json";
    config.job_workers = 1;
    return config;
}

void run_loop_ticks(rg::Session& session, std::uint64_t ticks) {
    const std::uint64_t n0 = session.loop_stats().stepped_count;
    session.start();
    while (session.loop_stats().stepped_count < n0 + ticks) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    session.stop();
}

} // namespace

TEST_CASE("player mode: rules per mode", "[player_mode]") {
    const rg::ModeRules drive = rg::rules_for(rg::PlayerMode::Drive);
    CHECK(drive.implemented);
    CHECK(drive.vehicle_control == rg::VehicleControl::Player);
    CHECK(drive.driving_inputs_live);
    CHECK(drive.camera_rig == rg::CameraRig::Chase);

    const rg::ModeRules free_cam = rg::rules_for(rg::PlayerMode::FreeCam);
    CHECK(free_cam.implemented);
    CHECK(free_cam.vehicle_control == rg::VehicleControl::Unattended);
    CHECK_FALSE(free_cam.driving_inputs_live);
    CHECK(free_cam.camera_inputs_live);
    CHECK(free_cam.camera_rig == rg::CameraRig::Free);

    // Drone follow (R9b): the own car is the default target and keeps driving.
    const rg::ModeRules drone = rg::rules_for(rg::PlayerMode::DroneFollow);
    CHECK(drone.implemented);
    CHECK(drone.camera_rig == rg::CameraRig::Drone);
    CHECK(drone.vehicle_control == rg::VehicleControl::Player);
    CHECK(drone.driving_inputs_live);
    CHECK(drone.camera_inputs_live);
    // Another vehicle: the car is Unattended like in free cam.
    const rg::ModeRules drone_other = rg::rules_for(rg::PlayerMode::DroneFollow, std::uint64_t{7});
    CHECK(drone_other.implemented);
    CHECK(drone_other.camera_rig == rg::CameraRig::Drone);
    CHECK(drone_other.vehicle_control == rg::VehicleControl::Unattended);
    CHECK_FALSE(drone_other.driving_inputs_live);
    CHECK(drone_other.camera_inputs_live);
    // The target means nothing outside DroneFollow.
    CHECK(rg::rules_for(rg::PlayerMode::Drive, std::uint64_t{7}).driving_inputs_live);

    // On foot (R9c): the car is parked (Unattended like in free cam), the
    // driving inputs are off and the walking inputs are live.
    const rg::ModeRules foot = rg::rules_for(rg::PlayerMode::OnFoot);
    CHECK(foot.implemented);
    CHECK(foot.vehicle_control == rg::VehicleControl::Unattended);
    CHECK_FALSE(foot.driving_inputs_live);
    CHECK(foot.walking_inputs_live);
    CHECK(foot.camera_inputs_live);
    CHECK(foot.camera_rig == rg::CameraRig::Walker);
    // Only OnFoot has walking inputs.
    for (rg::PlayerMode m : {rg::PlayerMode::Drive, rg::PlayerMode::FreeCam, rg::PlayerMode::DroneFollow}) {
        CHECK_FALSE(rg::rules_for(m).walking_inputs_live);
    }

    // Reserved: no behaviour (Cockpit is unused - the cockpit is a Drive view toggle).
    CHECK_FALSE(rg::rules_for(rg::PlayerMode::Cockpit).implemented);
    CHECK_FALSE(rg::rules_for(rg::PlayerMode::Cockpit).driving_inputs_live);
}

TEST_CASE("player mode: names round-trip", "[player_mode]") {
    for (int i = 0; i < rg::kPlayerModeCount; ++i) {
        const auto m = static_cast<rg::PlayerMode>(i);
        CHECK(rg::player_mode_from_string(rg::to_string(m)) == m);
    }
    CHECK_FALSE(rg::player_mode_from_string("Drive").has_value());
    CHECK(rg::world_kind_from_string("real_world") == rg::WorldKind::RealWorld);
    CHECK(rg::world_kind_from_string("flat") == rg::WorldKind::Flat);
    CHECK_FALSE(rg::world_kind_from_string("moon").has_value());
}

TEST_CASE("player mode: transitions", "[player_mode]") {
    rg::PlayerModeMachine m(rg::PlayerMode::Drive);
    CHECK(m.mode() == rg::PlayerMode::Drive);
    const std::uint64_t r0 = m.revision();
    CHECK(m.request_mode(rg::PlayerMode::Drive) == rg::PlayerModeMachine::Result::NoChange);
    CHECK(m.revision() == r0);
    CHECK(m.request_mode(rg::PlayerMode::Cockpit) == rg::PlayerModeMachine::Result::NotImplemented);
    CHECK(m.mode() == rg::PlayerMode::Drive);
    CHECK(m.request_mode(rg::PlayerMode::FreeCam) == rg::PlayerModeMachine::Result::Changed);
    CHECK(m.mode() == rg::PlayerMode::FreeCam);
    CHECK(m.revision() > r0);
    // cycle: drive -> free_cam -> drone_follow -> on_foot -> drive, skipping the reserved Cockpit
    CHECK(m.cycle_mode() == rg::PlayerMode::DroneFollow);
    CHECK(m.cycle_mode() == rg::PlayerMode::OnFoot);
    CHECK(m.cycle_mode() == rg::PlayerMode::Drive);
    CHECK(m.cycle_mode() == rg::PlayerMode::FreeCam);
    CHECK(m.cycle_mode() == rg::PlayerMode::DroneFollow);
    CHECK(m.request_mode(rg::PlayerMode::DroneFollow) == rg::PlayerModeMachine::Result::NoChange);

    CHECK_THROWS_AS(rg::PlayerModeMachine(rg::PlayerMode::Cockpit), std::invalid_argument);
}

TEST_CASE("player mode: getting out needs a nearly stopped car", "[player_mode][on_foot]") {
    CHECK(rg::may_get_out(0.0));
    CHECK(rg::may_get_out(1.99));
    CHECK(rg::may_get_out(-1.99)); // reversing slowly
    CHECK_FALSE(rg::may_get_out(rg::kGetOutMaxSpeedMps));
    CHECK_FALSE(rg::may_get_out(30.0));
    CHECK_FALSE(rg::may_get_out(-30.0));
    CHECK_FALSE(rg::may_get_out(std::nan("")));

    rg::PlayerModeMachine m(rg::PlayerMode::Drive);
    // Moving: refused, no state change, counted.
    const std::uint64_t r0 = m.revision();
    CHECK(m.request_mode(rg::PlayerMode::OnFoot, 20.0) == rg::PlayerModeMachine::Result::Refused);
    CHECK(m.mode() == rg::PlayerMode::Drive);
    CHECK(m.revision() == r0);
    CHECK(m.get_out_refusals() == 1);
    // Nearly stopped: allowed.
    CHECK(m.request_mode(rg::PlayerMode::OnFoot, 0.5) == rg::PlayerModeMachine::Result::Changed);
    CHECK(m.mode() == rg::PlayerMode::OnFoot);
    CHECK(m.get_out_refusals() == 1);
    // Already on foot: the speed does not matter (the walker is not the car).
    CHECK(m.request_mode(rg::PlayerMode::OnFoot, 20.0) == rg::PlayerModeMachine::Result::NoChange);
    // Leaving OnFoot is never refused.
    CHECK(m.request_mode(rg::PlayerMode::Drive, 20.0) == rg::PlayerModeMachine::Result::Changed);
    CHECK(m.get_out_refusals() == 1);

    // The cycle skips OnFoot while the car is moving (and counts it): it never gets stuck.
    rg::PlayerModeMachine c(rg::PlayerMode::DroneFollow);
    CHECK(c.cycle_mode(15.0) == rg::PlayerMode::Drive);
    CHECK(c.get_out_refusals() == 1);
    CHECK(c.cycle_mode(15.0) == rg::PlayerMode::FreeCam);
    CHECK(c.cycle_mode(15.0) == rg::PlayerMode::DroneFollow);
    CHECK(c.cycle_mode(0.0) == rg::PlayerMode::OnFoot);
    CHECK(c.cycle_mode(0.0) == rg::PlayerMode::Drive);
    CHECK(c.get_out_refusals() == 1);

    // The drone target resets on entering OnFoot like on any other change.
    rg::PlayerModeMachine d(rg::PlayerMode::DroneFollow);
    CHECK(d.set_drone_target(std::uint64_t{5}));
    CHECK(d.request_mode(rg::PlayerMode::OnFoot, 0.0) == rg::PlayerModeMachine::Result::Changed);
    CHECK_FALSE(d.drone_target().has_value());
}

TEST_CASE("player mode: walking inputs follow the world phase", "[player_mode][on_foot]") {
    rg::PlayerModeMachine m(rg::PlayerMode::Drive);
    REQUIRE(m.request_mode(rg::PlayerMode::OnFoot) == rg::PlayerModeMachine::Result::Changed);
    // No world yet: masked like the driving inputs, the look input stays live.
    CHECK_FALSE(m.effective_rules().walking_inputs_live);
    CHECK(m.effective_rules().camera_inputs_live);
    const std::uint64_t s = m.begin_world_load(rg::WorldKind::Flat);
    CHECK_FALSE(m.effective_rules().walking_inputs_live);
    REQUIRE(m.finish_world_load(s, true));
    CHECK(m.effective_rules().walking_inputs_live);
    CHECK(m.effective_rules().vehicle_control == rg::VehicleControl::Unattended);
    CHECK_FALSE(m.effective_rules().driving_inputs_live);
}

TEST_CASE("player mode: world switch flow", "[player_mode]") {
    rg::PlayerModeMachine m(rg::PlayerMode::Drive);
    CHECK(m.world_phase() == rg::WorldPhase::None);
    // No world: driving inputs masked, the car (if any) unattended.
    CHECK_FALSE(m.effective_rules().driving_inputs_live);
    CHECK(m.effective_rules().vehicle_control == rg::VehicleControl::Unattended);

    const std::uint64_t s1 = m.begin_world_load(rg::WorldKind::RealWorld);
    CHECK(m.world_phase() == rg::WorldPhase::Loading);
    CHECK(m.world_kind() == rg::WorldKind::RealWorld);
    CHECK(m.other_world() == rg::WorldKind::Flat);
    CHECK_FALSE(m.effective_rules().driving_inputs_live);
    CHECK(m.effective_rules().camera_inputs_live);

    // A second switch while loading supersedes the first; the stale finish is ignored.
    const std::uint64_t s2 = m.begin_world_load(rg::WorldKind::Flat);
    CHECK(s2 != s1);
    CHECK_FALSE(m.finish_world_load(s1, true));
    CHECK(m.world_phase() == rg::WorldPhase::Loading);
    CHECK(m.finish_world_load(s2, true));
    CHECK(m.world_phase() == rg::WorldPhase::Ready);
    CHECK(m.world_kind() == rg::WorldKind::Flat);
    CHECK(m.effective_rules().driving_inputs_live);
    CHECK(m.effective_rules().vehicle_control == rg::VehicleControl::Player);
    CHECK_FALSE(m.finish_world_load(s2, false)); // already finished

    // The mode survives a world switch; a failed load stays failed (no fallback).
    m.request_mode(rg::PlayerMode::FreeCam);
    const std::uint64_t s3 = m.begin_world_load(rg::WorldKind::RealWorld);
    CHECK(m.finish_world_load(s3, false));
    CHECK(m.world_phase() == rg::WorldPhase::Failed);
    CHECK(m.world_kind() == rg::WorldKind::RealWorld);
    CHECK(m.mode() == rg::PlayerMode::FreeCam);
    CHECK_FALSE(m.effective_rules().driving_inputs_live);
}

TEST_CASE("player mode: drone follow rules depend on the target", "[player_mode]") {
    rg::PlayerModeMachine m(rg::PlayerMode::Drive);
    m.finish_world_load(m.begin_world_load(rg::WorldKind::Flat), true);

    // Outside DroneFollow the target cannot be set.
    const std::uint64_t r_drive = m.revision();
    CHECK_FALSE(m.set_drone_target(std::uint64_t{5}));
    CHECK_FALSE(m.drone_target().has_value());
    CHECK(m.revision() == r_drive);

    REQUIRE(m.request_mode(rg::PlayerMode::DroneFollow) == rg::PlayerModeMachine::Result::Changed);
    CHECK_FALSE(m.drone_target().has_value()); // own car by default
    rg::ModeRules r = m.effective_rules();
    CHECK(r.camera_rig == rg::CameraRig::Drone);
    CHECK(r.vehicle_control == rg::VehicleControl::Player);
    CHECK(r.driving_inputs_live);
    CHECK(r.camera_inputs_live);

    // Another vehicle: unattended, driving inputs off, camera inputs on; one revision bump.
    std::uint64_t rev = m.revision();
    CHECK(m.set_drone_target(std::uint64_t{42}));
    CHECK(m.drone_target() == std::optional<std::uint64_t>{42});
    CHECK(m.revision() == rev + 1);
    r = m.effective_rules();
    CHECK(r.vehicle_control == rg::VehicleControl::Unattended);
    CHECK_FALSE(r.driving_inputs_live);
    CHECK(r.camera_inputs_live);
    CHECK(r.camera_rig == rg::CameraRig::Drone);

    // Same target again: no bump. Another id: bump.
    rev = m.revision();
    CHECK(m.set_drone_target(std::uint64_t{42}));
    CHECK(m.revision() == rev);
    CHECK(m.set_drone_target(std::uint64_t{43}));
    CHECK(m.revision() == rev + 1);

    // Back to the own car (what the binding does when the followed vehicle disappears).
    rev = m.revision();
    CHECK(m.set_drone_target(std::nullopt));
    CHECK(m.revision() == rev + 1);
    r = m.effective_rules();
    CHECK(r.vehicle_control == rg::VehicleControl::Player);
    CHECK(r.driving_inputs_live);

    // Leaving and re-entering resets the target to the own car.
    CHECK(m.set_drone_target(std::uint64_t{9}));
    m.request_mode(rg::PlayerMode::FreeCam);
    CHECK_FALSE(m.drone_target().has_value());
    CHECK(m.request_mode(rg::PlayerMode::DroneFollow) == rg::PlayerModeMachine::Result::Changed);
    CHECK_FALSE(m.drone_target().has_value());
    CHECK(m.effective_rules().driving_inputs_live);
    CHECK(m.set_drone_target(std::uint64_t{9}));
    CHECK(m.cycle_mode() == rg::PlayerMode::OnFoot); // drone_follow -> on_foot (R9c)
    CHECK_FALSE(m.drone_target().has_value());
}

TEST_CASE("player mode: drone follow masks driving inputs while the world is not ready", "[player_mode]") {
    rg::PlayerModeMachine m(rg::PlayerMode::Drive);
    REQUIRE(m.request_mode(rg::PlayerMode::DroneFollow) == rg::PlayerModeMachine::Result::Changed);
    // No world yet: even the own-car target cannot drive; the camera stays live.
    CHECK_FALSE(m.effective_rules().driving_inputs_live);
    CHECK(m.effective_rules().vehicle_control == rg::VehicleControl::Unattended);
    CHECK(m.effective_rules().camera_inputs_live);

    const std::uint64_t serial = m.begin_world_load(rg::WorldKind::RealWorld);
    CHECK_FALSE(m.effective_rules().driving_inputs_live);
    CHECK(m.set_drone_target(std::uint64_t{3}));
    CHECK_FALSE(m.effective_rules().driving_inputs_live);
    CHECK(m.set_drone_target(std::nullopt));
    CHECK_FALSE(m.effective_rules().driving_inputs_live); // still loading

    REQUIRE(m.finish_world_load(serial, true));
    CHECK(m.effective_rules().driving_inputs_live);
    CHECK(m.effective_rules().vehicle_control == rg::VehicleControl::Player);
    CHECK(m.set_drone_target(std::uint64_t{3}));
    CHECK_FALSE(m.effective_rules().driving_inputs_live);

    // The mode and the target survive a world switch; the loading mask applies again.
    const std::uint64_t s2 = m.begin_world_load(rg::WorldKind::Flat);
    CHECK(m.mode() == rg::PlayerMode::DroneFollow);
    CHECK(m.drone_target() == std::optional<std::uint64_t>{3});
    CHECK_FALSE(m.effective_rules().driving_inputs_live);
    CHECK(m.effective_rules().camera_inputs_live);
    m.finish_world_load(s2, false);
    CHECK_FALSE(m.effective_rules().driving_inputs_live);
}

TEST_CASE("player mode: next_drone_target cycles own car, then nearest first, ties by id", "[player_mode]") {
    using rg::DroneCandidate;
    using Target = std::optional<std::uint64_t>;
    const std::vector<DroneCandidate> c = {
        {10, 30.0, 0.0},   // distance 30
        {4, 0.0, 10.0},    // distance 10
        {7, -10.0, 0.0},   // distance 10 (tie with 4: id 4 first)
        {99, 900.0, 0.0},  // out of range
        {55, 100.0, 0.0},  // distance 100
    };
    const double range = 200.0;

    // own car -> nearest (tie: lower id) -> ... -> farthest in range -> own car
    Target t = std::nullopt;
    const Target expected[] = {Target{4}, Target{7}, Target{10}, Target{55}, std::nullopt};
    for (const Target& e : expected) {
        t = rg::next_drone_target(t, c, 0.0, 0.0, range);
        CHECK(t == e);
    }
    // The out-of-range vehicle is never selected.
    CHECK(rg::next_drone_target(Target{55}, c, 0.0, 0.0, range) == std::nullopt);
    // A larger range lets it in, at the end.
    CHECK(rg::next_drone_target(Target{55}, c, 0.0, 0.0, 1000.0) == Target{99});
    CHECK(rg::next_drone_target(Target{99}, c, 0.0, 0.0, 1000.0) == std::nullopt);

    // The order follows the reference point, not the list order.
    CHECK(rg::next_drone_target(std::nullopt, c, 100.0, 0.0, range) == Target{55});
    CHECK(rg::next_drone_target(Target{55}, c, 100.0, 0.0, range) == Target{10});

    // A current target that is gone (not in the list) or out of range goes to the own car.
    CHECK(rg::next_drone_target(Target{12345}, c, 0.0, 0.0, range) == std::nullopt);
    CHECK(rg::next_drone_target(Target{99}, c, 0.0, 0.0, range) == std::nullopt);

    // No candidates: the own car stays the own car.
    CHECK(rg::next_drone_target(std::nullopt, std::span<const DroneCandidate>{}, 0.0, 0.0, range) == std::nullopt);
    // Non-finite positions are ignored.
    const std::vector<DroneCandidate> bad = {{1, std::nan(""), 0.0}, {2, 5.0, 0.0}};
    CHECK(rg::next_drone_target(std::nullopt, bad, 0.0, 0.0, range) == Target{2});
    CHECK(rg::next_drone_target(Target{2}, bad, 0.0, 0.0, range) == std::nullopt);

    // Equal distance: the lower id first, whatever the list order.
    const std::vector<DroneCandidate> tie_a = {{8, 5.0, 0.0}, {3, 0.0, 5.0}};
    const std::vector<DroneCandidate> tie_b = {{3, 0.0, 5.0}, {8, 5.0, 0.0}};
    CHECK(rg::next_drone_target(std::nullopt, tie_a, 0.0, 0.0, range) == Target{3});
    CHECK(rg::next_drone_target(std::nullopt, tie_b, 0.0, 0.0, range) == Target{3});
    CHECK(rg::next_drone_target(Target{3}, tie_a, 0.0, 0.0, range) == Target{8});
    CHECK(rg::next_drone_target(Target{3}, tie_b, 0.0, 0.0, range) == Target{8});
}

TEST_CASE("player mode: the unattended policy", "[player_mode]") {
    const rg::UnattendedControls moving = rg::unattended_controls(20.0);
    CHECK(moving.throttle == 0.0);
    CHECK(moving.steer == 0.0);
    CHECK(moving.clutch == 1.0);
    CHECK(moving.starter == 0.0);
    CHECK(moving.brake == rg::kUnattendedRollingBrake);
    CHECK(moving.handbrake == 0.0);
    const rg::UnattendedControls parked = rg::unattended_controls(0.5);
    CHECK(parked.brake == 1.0);
    CHECK(parked.handbrake == 1.0);
    CHECK(parked.clutch == 1.0);
    CHECK(rg::unattended_controls(-20.0).handbrake == 0.0); // |speed|
}

TEST_CASE("player mode: an unattended session overrides the driving channels in the loop", "[player_mode]") {
    rg::Session session(flat_config());
    CHECK(session.vehicle_control() == rg::VehicleControl::Player);
    session.set_control("throttle", 1.0);
    session.set_control("steer", 0.5);
    session.set_control("ignition", 1.0);
    session.set_control("assist.auto_clutch", 1.0);

    // Player: the atomics reach the World unchanged.
    run_loop_ticks(session, 5);
    CHECK(session.world().get_control("throttle") == 1.0);
    CHECK(session.world().get_control("steer") == 0.5);

    // Unattended: the driving channels follow the policy; the rest pass through.
    session.set_vehicle_control(rg::VehicleControl::Unattended);
    run_loop_ticks(session, 5);
    CHECK(session.world().get_control("throttle") == 0.0);
    CHECK(session.world().get_control("steer") == 0.0);
    CHECK(session.world().get_control("clutch") == 1.0);
    CHECK(session.world().get_control("brake") == 1.0); // nearly at rest: hold
    CHECK(session.world().get_control("handbrake") == 1.0);
    CHECK(session.world().get_control("ignition") == 1.0);
    CHECK(session.world().get_control("assist.auto_clutch") == 1.0);
    CHECK(session.get_control("throttle") == 1.0); // the player's own channel value is untouched

    // Back to Player: the atomics are live again.
    session.set_vehicle_control(rg::VehicleControl::Player);
    run_loop_ticks(session, 5);
    CHECK(session.world().get_control("throttle") == 1.0);
}

TEST_CASE("player mode: reset to spawn puts the car back at rest (flat)", "[player_mode]") {
    rg::Session session(flat_config());
    const ps::Pose spawn = session.world().get_pose(session.chassis_body());
    ps::Pose away = spawn;
    away.position = ps::Vec3{50.0, 20.0, spawn.position.z};
    session.world().backend().set_pose(session.chassis_body(), away);
    session.world().backend().set_motion(session.chassis_body(), ps::Motion{ps::Vec3{10.0, 0.0, 0.0}, ps::Vec3{}});

    session.request_reset_to_spawn();
    session.step(); // the relocation attempt
    const ps::Pose p = session.world().get_pose(session.chassis_body());
    const ps::Motion m = session.world().get_motion(session.chassis_body());
    CHECK(std::abs(p.position.x - spawn.position.x) < 1e-9);
    CHECK(std::abs(p.position.y - spawn.position.y) < 1e-9);
    CHECK(std::abs(p.position.z - spawn.position.z) < 1e-6); // ground top z = 0, + chassis_z_m
    CHECK(m.linear.length() == 0.0);
    CHECK(session.streaming_status().relocations == 1);
    session.step(); // the next attempt drives again
    CHECK(session.streaming_status().relocations == 1);
}

TEST_CASE("player mode: unload_world clears the world, invalidates the load and returns to drive", "[player_mode]") {
    rg::PlayerModeMachine m(rg::PlayerMode::Drive);
    const std::uint64_t serial = m.begin_world_load(rg::WorldKind::RealWorld);
    REQUIRE(m.finish_world_load(serial, true));
    REQUIRE(m.request_mode(rg::PlayerMode::FreeCam) == rg::PlayerModeMachine::Result::Changed);
    CHECK(m.world_phase() == rg::WorldPhase::Ready);
    const std::uint64_t rev = m.revision();

    m.unload_world();
    CHECK(m.world_phase() == rg::WorldPhase::None);
    CHECK(m.mode() == rg::PlayerMode::Drive);
    CHECK_FALSE(m.drone_target().has_value());
    CHECK(m.revision() > rev);
    CHECK_FALSE(m.effective_rules().driving_inputs_live); // masked until a world is ready again

    // A load that was in flight when the world was unloaded cannot finish.
    const std::uint64_t pending = m.begin_world_load(rg::WorldKind::Flat);
    m.unload_world();
    CHECK_FALSE(m.finish_world_load(pending, true));
    CHECK(m.world_phase() == rg::WorldPhase::None);

    // And a fresh load works as usual.
    const std::uint64_t again = m.begin_world_load(rg::WorldKind::Flat);
    CHECK(m.finish_world_load(again, true));
    CHECK(m.effective_rules().driving_inputs_live);
}
