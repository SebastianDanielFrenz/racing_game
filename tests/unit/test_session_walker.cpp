// test_session_walker.cpp — the on-foot player inside rg::Session (R2.2 R9c):
// get out beside the driver's door, walk, get back in, forced removal,
// footprint collision with the own car, worker-count determinism and the
// real-time loop path. Flat world (a static ground box); the terrain-mode
// interest-point tests are in test_session_terrain.cpp ([on_foot]).
//
// TOOL-031: no Catch2 assertion runs on the loop thread.
#include "rg/session.h"
#include "rg/walker.h"

#include "ps/backend/shape_desc.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>

namespace {

using Catch::Approx;

constexpr double kPi = 3.141592653589793;

rg::SessionConfig flat_config(unsigned workers = 1) {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json";
    config.job_workers = workers;
    return config;
}

void settle(rg::Session& s, int ticks = 360) {
    for (int i = 0; i < ticks; ++i) s.step();
}

void add_static_box(rg::Session& s, ps::Vec3 centre, ps::Vec3 half) {
    ps::BodyDesc d;
    d.shape = ps::BoxShape{half};
    d.motion = ps::BodyMotionType::Static;
    d.pose.position = centre;
    s.world().create_body(d);
}

// Chassis-local coordinates of a world XY.
void to_chassis(rg::Session& s, double x, double y, double& lx, double& ly) {
    const ps::Pose p = s.world().get_pose(s.chassis_body());
    const ps::Vec3 f = p.orientation.rotate(ps::Vec3::unit_x());
    const double yaw = std::atan2(f.y, f.x);
    const double dx = x - p.position.x, dy = y - p.position.y;
    lx = std::cos(yaw) * dx + std::sin(yaw) * dy;
    ly = -std::sin(yaw) * dx + std::cos(yaw) * dy;
}

const rg::WalkerState& spawn_walker(rg::Session& s) {
    s.request_walker_spawn();
    s.step();
    REQUIRE(s.walker_active());
    REQUIRE(s.walker_controller() != nullptr);
    return s.walker_controller()->state();
}

} // namespace

TEST_CASE("on foot: the walker spawns beside the driver's door", "[on_foot][session]") {
    rg::Session s(flat_config());
    settle(s);
    CHECK_FALSE(s.walker_active());
    CHECK(s.walker_controller() == nullptr);
    const std::size_t bodies_before = s.world().backend().live_body_ids().size();

    s.request_walker_spawn();
    s.step();
    REQUIRE(s.walker_active());
    CHECK(s.walker_counters().spawned == 1);
    CHECK(s.walker_counters().spawn_failed == 0);
    CHECK(s.world().backend().live_body_ids().size() == bodies_before + 1);

    const rg::WalkerState& w = s.walker_controller()->state();
    double lx = 0.0, ly = 0.0;
    to_chassis(s, w.feet.x, w.feet.y, lx, ly);
    CHECK(lx == Approx(0.3).margin(0.05));            // at the door, not at the nose or the tail
    CHECK(ly > 0.4 + 0.3);                            // left of the chassis, outside the capsule radius
    const rg::OrientedRect car = s.own_car_footprint();
    const double dist = rg::distance_to_rect(car, w.feet.x, w.feet.y);
    CHECK(dist >= 0.3);                               // the capsule does not overlap the footprint
    CHECK(dist == Approx(0.3 + 0.25).margin(0.02));   // radius + clearance
    CHECK(w.feet.z == Approx(0.0).margin(0.05));      // on the ground
    CHECK(w.grounded);

    // The frame snapshot data the HUD reads: the prompt is on at the door.
    // (post_step publishes only from the loop; the controller's own numbers agree.)
    CHECK(rg::within_enter_range(car, w.feet, 1.5, 2.5));

    // A second spawn request does nothing while a walker exists.
    s.request_walker_spawn();
    s.step();
    CHECK(s.walker_counters().spawned == 1);
}

TEST_CASE("on foot: a blocked driver side falls back to the passenger side", "[on_foot][session]") {
    rg::Session s(flat_config());
    settle(s);
    const ps::Vec3 car = s.world().get_pose(s.chassis_body()).position;
    // A thick wall on the left (+y) of the car, 0.2 m from the footprint (0.4 + wheels ~0.95 half width).
    add_static_box(s, {car.x, car.y + 1.2 + 2.0, 1.5}, {6.0, 2.0, 1.5});
    const rg::WalkerState& w = spawn_walker(s);
    double lx = 0.0, ly = 0.0;
    to_chassis(s, w.feet.x, w.feet.y, lx, ly);
    CHECK(ly < -0.4); // right side
    CHECK(rg::distance_to_rect(s.own_car_footprint(), w.feet.x, w.feet.y) >= 0.3);
}

TEST_CASE("on foot: boxed in on every side, the walker is dropped onto the roof line and pushed out", "[on_foot][session]") {
    rg::Session s(flat_config());
    settle(s);
    const ps::Vec3 car = s.world().get_pose(s.chassis_body()).position;
    // Four thick walls leaving a 0.7 m corridor around the footprint: a capsule fits
    // in it, but no standing spot exists (every spawn candidate overlaps a wall).
    const rg::OrientedRect fp = s.own_car_footprint();
    const double gap = 0.7, thick = 3.0;
    const double ox = fp.half_x + gap + thick, oy = fp.half_y + gap + thick;
    add_static_box(s, {car.x, car.y + oy, 1.5}, {fp.half_x + 12.0, thick, 1.5});
    add_static_box(s, {car.x, car.y - oy, 1.5}, {fp.half_x + 12.0, thick, 1.5});
    add_static_box(s, {car.x + ox, car.y, 1.5}, {thick, fp.half_y + 12.0, 1.5});
    add_static_box(s, {car.x - ox, car.y, 1.5}, {thick, fp.half_y + 12.0, 1.5});
    const rg::WalkerState& w0 = spawn_walker(s);
    // The roof fallback: spawned above the car, in the air.
    CHECK(w0.feet.z > 1.0);
    CHECK_FALSE(w0.grounded);
    CHECK(w0.feet.x == Approx(car.x).margin(0.2));
    for (int i = 0; i < 480; ++i) s.step();
    const rg::WalkerState& w = s.walker_controller()->state();
    CHECK(std::isfinite(w.feet.x));
    CHECK(w.feet.z < 0.5); // came down to the ground, not stuck on the roof
    CHECK(w.grounded);
    CHECK(rg::distance_to_rect(s.own_car_footprint(), w.feet.x, w.feet.y) >= 0.3 - 1e-3);
}

TEST_CASE("on foot: the own car blocks the walker and is not disturbed by it", "[on_foot][session]") {
    rg::Session s(flat_config());
    settle(s);
    const ps::Pose car0 = s.world().get_pose(s.chassis_body());
    const rg::WalkerState& w = spawn_walker(s);
    // Walk straight at the car (it stands at -y of the walker: the walker spawned on the left side).
    rg::WalkerInput in;
    in.move_forward = 1.0;
    in.look_yaw_rad = -kPi / 2.0; // along -y
    in.run = true;
    s.set_walker_input(in);
    for (int i = 0; i < 480; ++i) s.step();
    const rg::OrientedRect footprint = s.own_car_footprint();
    CHECK(rg::distance_to_rect(footprint, w.feet.x, w.feet.y) >= 0.3 - 1e-3);
    CHECK(rg::distance_to_rect(footprint, w.feet.x, w.feet.y) < 0.35);
    CHECK(w.blocked);
    // Pressing against a parked car does not move it.
    const ps::Pose car1 = s.world().get_pose(s.chassis_body());
    CHECK((car1.position - car0.position).length() < 2.0e-3);
    // And the walker did not climb onto it.
    CHECK(w.feet.z < 0.2);
}

TEST_CASE("on foot: get in only within range of the own car", "[on_foot][session]") {
    rg::Session s(flat_config());
    settle(s);
    spawn_walker(s);

    // Beside the door: the interact request enters.
    s.request_walker_enter();
    s.step();
    CHECK_FALSE(s.walker_active());
    CHECK(s.walker_counters().entered == 1);
    CHECK(s.walker_counters().enter_refused == 0);
    const std::size_t bodies = s.world().backend().live_body_ids().size();

    // Spawn again, run away 8 m, and try: refused, the walker stays.
    spawn_walker(s);
    CHECK(s.world().backend().live_body_ids().size() == bodies + 1);
    rg::WalkerInput away;
    away.move_forward = 1.0;
    away.look_yaw_rad = kPi / 2.0; // +y, away from the car on its left side
    away.run = true;
    s.set_walker_input(away);
    for (int i = 0; i < 360; ++i) s.step();
    s.request_walker_enter();
    s.step();
    CHECK(s.walker_active());
    CHECK(s.walker_counters().entered == 1);
    CHECK(s.walker_counters().enter_refused == 1);

    // Come back (turn around), get in again.
    rg::WalkerInput back = away;
    back.look_yaw_rad = -kPi / 2.0;
    s.set_walker_input(back);
    for (int i = 0; i < 400; ++i) {
        s.step();
        if (s.walker_controller()->state().blocked) break;
    }
    s.set_walker_input(rg::WalkerInput{});
    for (int i = 0; i < 30; ++i) s.step();
    s.request_walker_enter();
    s.step();
    CHECK_FALSE(s.walker_active());
    CHECK(s.walker_counters().entered == 2);
    // The body is gone; the car is untouched and the World keeps stepping.
    CHECK(s.world().backend().live_body_ids().size() == bodies);
    for (int i = 0; i < 10; ++i) s.step();
}

TEST_CASE("on foot: a forced despawn removes the walker without a range check", "[on_foot][session]") {
    rg::Session s(flat_config());
    settle(s);
    spawn_walker(s);
    rg::WalkerInput away;
    away.move_forward = 1.0;
    away.look_yaw_rad = kPi / 2.0;
    away.run = true;
    s.set_walker_input(away);
    for (int i = 0; i < 480; ++i) s.step();
    s.request_walker_despawn();
    s.step();
    CHECK_FALSE(s.walker_active());
    CHECK(s.walker_counters().despawned == 1);
    CHECK(s.walker_counters().entered == 0);
    // Requests against a missing walker are harmless.
    s.request_walker_enter();
    s.request_walker_despawn();
    s.step();
    CHECK(s.walker_counters().entered == 0);
    CHECK(s.walker_counters().enter_refused == 0);
    // And a new spawn works again, beside the car.
    spawn_walker(s);
    CHECK(s.walker_counters().spawned == 2);
}

TEST_CASE("on foot: input, run and jump reach the controller", "[on_foot][session]") {
    rg::Session s(flat_config());
    settle(s);
    const rg::WalkerState& w = spawn_walker(s);
    rg::WalkerInput in;
    in.move_forward = 1.0;
    in.look_yaw_rad = kPi / 2.0;
    s.set_walker_input(in);
    for (int i = 0; i < 120; ++i) s.step();
    CHECK(std::hypot(w.velocity.x, w.velocity.y) == Approx(1.4).margin(0.02));
    in.run = true;
    s.set_walker_input(in);
    for (int i = 0; i < 120; ++i) s.step();
    CHECK(std::hypot(w.velocity.x, w.velocity.y) == Approx(5.0).margin(0.02));
    s.request_walker_jump();
    double apex = w.feet.z;
    for (int i = 0; i < 200; ++i) {
        s.step();
        apex = std::max(apex, w.feet.z);
    }
    CHECK(apex > 0.7);
    CHECK(apex < 1.1);
}

TEST_CASE("on foot: state hash with a walker is identical at 1 and 4 workers", "[on_foot][session][determinism]") {
    const auto run = [](unsigned workers) {
        rg::Session s(flat_config(workers));
        settle(s, 240);
        s.request_walker_spawn();
        for (int i = 0; i < 1200; ++i) {
            rg::WalkerInput in;
            const double t = i / 240.0;
            in.move_forward = 1.0;
            in.move_right = std::fmod(t, 2.0) < 1.0 ? 0.0 : -0.5;
            in.look_yaw_rad = 0.3 * std::fmod(t, 7.0);
            in.run = std::fmod(t, 3.0) < 1.5;
            s.set_walker_input(in);
            if (i % 300 == 299) s.request_walker_jump();
            s.step();
        }
        REQUIRE(s.walker_active());
        return std::pair{s.world().state_hash(), s.walker_controller()->state().feet};
    };
    const auto a = run(1);
    const auto b = run(4);
    CHECK(a.first == b.first);
    CHECK(a.second == b.second);
    // The walker is part of the hash: a run without one differs.
    rg::Session plain(flat_config(1));
    settle(plain, 240);
    for (int i = 0; i < 1200; ++i) plain.step();
    CHECK(plain.world().state_hash() != a.first);
}

TEST_CASE("on foot: a relocation of the car leaves the walker alone", "[on_foot][session]") {
    rg::Session s(flat_config());
    settle(s);
    const rg::WalkerState& w = spawn_walker(s);
    for (int i = 0; i < 60; ++i) s.step();
    const ps::Vec3 feet = w.feet;
    s.request_relocate(60.0, 40.0, 0.0);
    for (int i = 0; i < 30; ++i) s.step();
    CHECK(s.walker_active());
    CHECK((w.feet - feet).length() < 1.0e-6);
    const ps::Vec3 car = s.world().get_pose(s.chassis_body()).position;
    CHECK(car.x == Approx(60.0).margin(1.0));
    // Out of range now: interact is refused.
    s.request_walker_enter();
    s.step();
    CHECK(s.walker_counters().enter_refused == 1);
}

TEST_CASE("on foot: the real-time loop publishes the walker in the frame snapshot", "[on_foot][session]") {
    rg::Session s(flat_config());
    settle(s, 240);
    s.set_vehicle_control(rg::VehicleControl::Unattended);
    s.request_walker_spawn();
    rg::WalkerInput in;
    in.move_forward = 1.0;
    in.look_yaw_rad = kPi / 2.0;
    s.set_walker_input(in);
    s.start();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    bool seen_moving = false;
    while (std::chrono::steady_clock::now() < deadline) {
        const rg::FrameSnapshot& f = s.snapshot();
        if (f.walker.active && f.walker.velocity.y > 1.0) {
            seen_moving = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const rg::FrameSnapshot frame = s.snapshot();
    s.stop();
    REQUIRE(seen_moving);
    CHECK(frame.walker.active);
    CHECK(frame.walker.grounded);
    CHECK(frame.walker.yaw_rad == Approx(kPi / 2.0).margin(0.1));
    CHECK(frame.walker.enter_distance_m >= 0.3 - 1e-3);
    CHECK(s.walker_counters().spawned == 1);
    // The unattended car braked and held itself while the walker moved around it.
    CHECK(frame.chassis_motion.linear.length() < 0.5);
}
