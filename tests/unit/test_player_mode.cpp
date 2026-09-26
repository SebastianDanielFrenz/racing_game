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
#include <stdexcept>
#include <string>
#include <thread>

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

    // Reserved for R9b/R9c: no behaviour.
    for (rg::PlayerMode m : {rg::PlayerMode::DroneFollow, rg::PlayerMode::Cockpit, rg::PlayerMode::OnFoot}) {
        CHECK_FALSE(rg::rules_for(m).implemented);
        CHECK_FALSE(rg::rules_for(m).driving_inputs_live);
    }
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
    // cycle skips the reserved modes
    CHECK(m.cycle_mode() == rg::PlayerMode::Drive);
    CHECK(m.cycle_mode() == rg::PlayerMode::FreeCam);
    CHECK(m.cycle_mode() == rg::PlayerMode::Drive);

    CHECK_THROWS_AS(rg::PlayerModeMachine(rg::PlayerMode::OnFoot), std::invalid_argument);
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
