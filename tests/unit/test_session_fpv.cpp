// test_session_fpv.cpp - the FPV drone inside rg::Session on the flat world: spawn above the car, fly, ground
// clamp against the real World ray, despawn, worker-count determinism. The terrain-mode interest-point case is in
// test_session_terrain.cpp ([fpv]).
#include "rg/session.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>

namespace {
const rg::FpvDroneConfig& fpv_cfg() {
    static const rg::FpvDroneConfig c;
    return c;
}


using Catch::Approx;

rg::SessionConfig flat_config(unsigned workers = 1) {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json";
    config.job_workers = workers;
    return config;
}

void settle(rg::Session& s, int ticks = 240) {
    for (int i = 0; i < ticks; ++i) s.step();
}

} // namespace

TEST_CASE("fpv session: spawns above the car, flies, lands on the ground, despawns", "[fpv][session]") {
    rg::Session s(flat_config());
    settle(s);
    CHECK_FALSE(s.fpv_active());
    CHECK_FALSE(s.snapshot().fpv.active);
    const ps::Pose car = s.world().get_pose(s.chassis_body());

    s.request_fpv_spawn();
    s.step();
    REQUIRE(s.fpv_active());
    CHECK(s.fpv_counters().spawned == 1);
    CHECK(s.fpv_counters().spawn_failed == 0);
    const rg::FpvDroneState& d = s.fpv_drone().state();
    CHECK(d.position.x == Approx(car.position.x).margin(0.1));
    CHECK(d.position.y == Approx(car.position.y).margin(0.1));
    CHECK(d.position.z == Approx(car.position.z + fpv_cfg().spawn_height_above_car_m).margin(0.1));
    const ps::Vec3 fwd = car.orientation.rotate(ps::Vec3::unit_x());
    CHECK(d.yaw_rad == Approx(std::atan2(fwd.y, fwd.x)).margin(1e-3));
    CHECK(s.fpv_counters().spawned == 1);
    s.request_fpv_spawn(); // already flying: no second drone
    s.step();
    CHECK(s.fpv_counters().spawned == 1);

    // Fly forward 2 s: it moves along its heading, the published snapshot matches.
    rg::FpvDroneInput in;
    in.move_forward = 1.0;
    s.set_fpv_input(in);
    for (int i = 0; i < 480; ++i) s.step();
    CHECK(std::hypot(d.position.x - car.position.x, d.position.y - car.position.y) > 20.0);
    CHECK(d.speed_mps == Approx(fpv_cfg().max_speed_mps).margin(1e-6));

    // Descend away from the car: stops at the ground (z = 0 plane) plus the clearance, never below.
    in = {};
    in.move_up = -1.0;
    s.set_fpv_input(in);
    double min_z = 1e9;
    for (int i = 0; i < 960; ++i) {
        s.step();
        min_z = std::min(min_z, d.position.z);
    }
    CHECK(min_z >= fpv_cfg().ground_clearance_m - 0.02);
    CHECK(d.on_ground);
    CHECK(d.position.z == Approx(fpv_cfg().ground_clearance_m).margin(0.02));

    s.request_fpv_despawn();
    s.step();
    CHECK_FALSE(s.fpv_active());
    CHECK(s.fpv_counters().despawned == 1);
    CHECK_FALSE(s.snapshot().fpv.active);
}

TEST_CASE("fpv session: a flight is worker-count deterministic", "[fpv][session][determinism]") {
    const auto run = [](unsigned workers) {
        rg::Session s(flat_config(workers));
        settle(s, 120);
        s.request_fpv_spawn();
        for (int i = 0; i < 960; ++i) {
            rg::FpvDroneInput in;
            in.move_forward = 1.0;
            in.move_right = std::sin(0.01 * i);
            in.yaw = std::cos(0.02 * i);
            in.move_up = std::sin(0.005 * i) - 0.2;
            s.set_fpv_input(in);
            s.step();
        }
        return std::pair{s.world().state_hash(), s.fpv_drone().state().position};
    };
    const auto a = run(1), b = run(4);
    CHECK(a.first == b.first);
    CHECK(a.second.x == b.second.x);
    CHECK(a.second.y == b.second.y);
    CHECK(a.second.z == b.second.z);
}
