// test_walker.cpp — rg::WalkerController and the pure walker helpers
// (rg/walker.h, R2.2 R9c). The controller tests run on a plain ps::World with
// hand-built static boxes (flat ground, kerbs, walls, ramps); the Session
// integration (interest points, get out / get in) is in test_session_walker.cpp.
#include "rg/walker.h"

#include "ps/backend/shape_desc.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace {

using Catch::Approx;

constexpr double kPi = 3.141592653589793;

struct Rig {
    std::unique_ptr<ps::World> world;
    std::unique_ptr<rg::WalkerController> walker;
    double dt = 0.0;

    explicit Rig(unsigned workers = 1, bool ground = true) {
        ps::WorldConfig cfg;
        cfg.job_workers = static_cast<int>(workers);
        world = std::make_unique<ps::World>(cfg);
        dt = world->dt();
        if (ground) add_box({0.0, 0.0, -0.5}, {200.0, 200.0, 0.5});
        walker = std::make_unique<rg::WalkerController>(*world, rg::WalkerConfig{});
    }
    ps::BodyId add_box(ps::Vec3 centre, ps::Vec3 half, ps::Quat q = ps::Quat::identity()) {
        ps::BodyDesc d;
        d.shape = ps::BoxShape{half};
        d.motion = ps::BodyMotionType::Static;
        d.pose = ps::Pose{centre, q};
        return world->create_body(d);
    }
    // A ramp rising toward +x at `angle_deg`, its low edge at x = x0 (z = 0), 10 m long.
    void add_ramp(double x0, double angle_deg) {
        const double th = angle_deg * kPi / 180.0;
        const double hz = 0.1, half_len = 5.0;
        const ps::Vec3 n{-std::sin(th), 0.0, std::cos(th)};
        const ps::Vec3 top{x0 + half_len * std::cos(th), 0.0, half_len * std::sin(th)};
        add_box(top - n * hz, {half_len, 5.0, hz}, ps::Quat::from_axis_angle({0, 1, 0}, -th));
    }
    void spawn(double x, double y, double z, double yaw = 0.0) { walker->spawn({x, y, z}, yaw); }
    void run(const rg::WalkerInput& in, double seconds) {
        const int n = static_cast<int>(std::lround(seconds / dt));
        for (int i = 0; i < n; ++i) tick(in);
    }
    void tick(const rg::WalkerInput& in) {
        walker->step(in, dt);
        world->step();
    }
    [[nodiscard]] const rg::WalkerState& st() const { return walker->state(); }
};

rg::WalkerInput forward(double yaw = 0.0, bool run = false) {
    rg::WalkerInput in;
    in.move_forward = 1.0;
    in.look_yaw_rad = yaw;
    in.run = run;
    return in;
}

} // namespace

// ---------------------------------------------------------------------------
// Pure helpers
// ---------------------------------------------------------------------------

TEST_CASE("walker: rectangle distance and push-out", "[on_foot]") {
    const rg::OrientedRect r = rg::make_oriented_rect(10.0, 5.0, 0.0, 2.0, 1.0, 0.0, 2.0);
    CHECK(rg::distance_to_rect(r, 10.0, 5.0) == Approx(0.0));
    CHECK(rg::distance_to_rect(r, 12.0, 5.0) == Approx(0.0)); // on the edge
    CHECK(rg::distance_to_rect(r, 14.0, 5.0) == Approx(2.0));
    CHECK(rg::distance_to_rect(r, 10.0, 8.0) == Approx(2.0));
    CHECK(rg::distance_to_rect(r, 14.0, 8.0) == Approx(std::sqrt(4.0 + 4.0))); // corner

    double x = 12.1, y = 5.0; // 0.1 outside the +x edge, circle radius 0.3 overlaps
    CHECK(rg::push_circle_out_of_rect(r, 0.3, x, y));
    CHECK(x == Approx(12.3));
    CHECK(y == Approx(5.0));
    x = 13.0;
    CHECK_FALSE(rg::push_circle_out_of_rect(r, 0.3, x, y)); // clear
    CHECK(x == 13.0);
    // Centre inside: out through the nearest edge.
    x = 11.5;
    y = 5.2;
    CHECK(rg::push_circle_out_of_rect(r, 0.3, x, y));
    CHECK(x == Approx(12.3));
    // A rotated rectangle (90 deg: its local x runs along world +y).
    const rg::OrientedRect q = rg::make_oriented_rect(0.0, 0.0, kPi / 2.0, 2.0, 1.0, 0.0, 2.0);
    CHECK(rg::distance_to_rect(q, 0.0, 3.0) == Approx(1.0));
    CHECK(rg::distance_to_rect(q, 3.0, 0.0) == Approx(2.0));
}

TEST_CASE("walker: vehicle footprint covers chassis and wheels, offset for an asymmetric wheelbase", "[on_foot]") {
    // car_hyper-like: front axle at +1.566, rear at -1.134, track +-0.85, wheel radius 0.35.
    const std::vector<rg::WheelFootprint> wheels = {
        {1.566, 0.85, 0.347, 0.15}, {1.566, -0.85, 0.347, 0.15}, {-1.134, 0.825, 0.347, 0.15}, {-1.134, -0.825, 0.347, 0.15}};
    const rg::OrientedRect r = rg::vehicle_footprint({100.0, 50.0, 0.5}, {1.0, 0.0, 0.0}, 2.0, 0.4, wheels, 0.1, -0.5, 1.4);
    // x: chassis box +-2.0 dominates the front (1.913), the rear is the box too (-2.0): symmetric here.
    CHECK(r.half_x == Approx(2.1));
    CHECK(r.half_y == Approx(0.85 + 0.15 + 0.1));
    CHECK(r.cx == Approx(100.0));
    CHECK(r.cy == Approx(50.0));
    // A short chassis box: the wheels decide and the rectangle is offset toward the front.
    const rg::OrientedRect s = rg::vehicle_footprint({0.0, 0.0, 0.5}, {1.0, 0.0, 0.0}, 1.0, 0.4, wheels, 0.0, -0.5, 1.4);
    CHECK(s.half_x == Approx(0.5 * ((1.566 + 0.347) - (-1.134 - 0.347))));
    CHECK(s.cx == Approx(0.5 * ((1.566 + 0.347) + (-1.134 - 0.347))));
    CHECK(s.cx > 0.0);
    // Yawed 90 deg: the centre offset follows the heading.
    const rg::OrientedRect t = rg::vehicle_footprint({0.0, 0.0, 0.5}, {0.0, 1.0, 0.0}, 1.0, 0.4, wheels, 0.0, -0.5, 1.4);
    CHECK(t.cx == Approx(0.0).margin(1e-12));
    CHECK(t.cy == Approx(s.cx));
    // A degenerate (vertical) forward falls back to +x.
    const rg::OrientedRect u = rg::vehicle_footprint({0.0, 0.0, 0.5}, {0.0, 0.0, 1.0}, 2.0, 0.4, wheels, 0.0, -0.5, 1.4);
    CHECK(u.cos_yaw == Approx(1.0));
}

TEST_CASE("walker: enter range check", "[on_foot]") {
    // 4 m x 2 m car at the origin on ground z = 0 (z range -0.5 .. 1.4 -> reference height 0).
    const rg::OrientedRect car = rg::make_oriented_rect(0.0, 0.0, 0.0, 2.0, 1.0, -0.5, 1.4);
    // Beside the driver door.
    CHECK(rg::within_enter_range(car, {0.3, 2.2, 0.0}, 1.5, 2.5));   // 1.2 from the left edge
    CHECK(rg::within_enter_range(car, {0.3, -2.4, 0.0}, 1.5, 2.5));  // right side, 1.4
    CHECK_FALSE(rg::within_enter_range(car, {0.3, 2.6, 0.0}, 1.5, 2.5)); // 1.6 away
    // Any side counts: behind and in front.
    CHECK(rg::within_enter_range(car, {-3.2, 0.0, 0.0}, 1.5, 2.5));
    CHECK(rg::within_enter_range(car, {3.4, 0.0, 0.0}, 1.5, 2.5));
    CHECK_FALSE(rg::within_enter_range(car, {-3.6, 0.0, 0.0}, 1.5, 2.5));
    // Corner distance is Euclidean: (1.1, 1.1) off the corner = 1.556 > 1.5.
    CHECK_FALSE(rg::within_enter_range(car, {3.1, 2.1, 0.0}, 1.5, 2.5));
    CHECK(rg::within_enter_range(car, {3.0, 2.0, 0.0}, 1.5, 2.5));
    // Too far above or below (a bridge over the car, a pit).
    CHECK_FALSE(rg::within_enter_range(car, {0.3, 2.2, 3.0}, 1.5, 2.5));
    CHECK_FALSE(rg::within_enter_range(car, {0.3, 2.2, -3.0}, 1.5, 2.5));
    // NaN never enters.
    CHECK_FALSE(rg::within_enter_range(car, {std::nan(""), 2.2, 0.0}, 1.5, 2.5));
}

TEST_CASE("walker: spawn spot prefers the driver side, then the passenger side, rear, front, roof", "[on_foot]") {
    const ps::Vec3 chassis{10.0, 20.0, 0.5};
    const rg::OrientedRect car = rg::make_oriented_rect(10.0, 20.0, 0.0, 2.0, 0.9, -0.5, 1.4);
    const double r = 0.3, clearance = 0.25, door_x = 0.3, roof = 1.5;

    // All open: the driver side (left = +y for yaw 0) at left edge + r + clearance.
    auto open = [](double, double) -> std::optional<double> { return 0.0; };
    rg::SpawnSpot spot = rg::select_spawn_spot(car, chassis, 0.0, r, door_x, clearance, roof, open);
    CHECK(spot.candidate == 0);
    CHECK_FALSE(spot.above_roof);
    CHECK(spot.feet.x == Approx(10.3));
    CHECK(spot.feet.y == Approx(20.0 + 0.9 + 0.3 + 0.25));
    CHECK(spot.yaw_rad == Approx(kPi / 2.0)); // looking away from the car
    // ... and the spot is inside the get-in range.
    CHECK(rg::within_enter_range(car, {spot.feet.x, spot.feet.y, 0.0}, 1.5, 2.5));

    // The driver side blocked (a wall at y > 20.5): the passenger side.
    auto no_left = [](double, double y) -> std::optional<double> {
        if (y > 20.5) return std::nullopt;
        return 0.0;
    };
    spot = rg::select_spawn_spot(car, chassis, 0.0, r, door_x, clearance, roof, no_left);
    CHECK(spot.candidate == 1);
    CHECK(spot.feet.y == Approx(20.0 - 0.9 - 0.3 - 0.25));
    CHECK(spot.yaw_rad == Approx(-kPi / 2.0));

    // Both sides blocked: behind, then in front.
    auto only_ends = [](double x, double y) -> std::optional<double> {
        if (std::fabs(y - 20.0) > 0.5 && std::fabs(x - 10.0) < 3.0) return std::nullopt;
        return 0.0;
    };
    spot = rg::select_spawn_spot(car, chassis, 0.0, r, door_x, clearance, roof, only_ends);
    CHECK(spot.candidate == 2);
    CHECK(spot.feet.x == Approx(10.0 - 2.0 - 0.3 - 0.25));
    auto only_front = [](double x, double y) -> std::optional<double> {
        if (x < 12.2) return std::nullopt;
        (void)y;
        return 0.0;
    };
    spot = rg::select_spawn_spot(car, chassis, 0.0, r, door_x, clearance, roof, only_front);
    CHECK(spot.candidate == 3);
    CHECK(spot.feet.x == Approx(10.0 + 2.0 + 0.3 + 0.25));

    // A candidate further out on the same side is tried before the next side.
    auto far_left = [](double x, double y) -> std::optional<double> {
        (void)x;
        if (y > 21.9 && y < 22.5) return 0.0; // only a patch 0.7 m beyond the first spot
        return std::nullopt;
    };
    spot = rg::select_spawn_spot(car, chassis, 0.0, r, door_x, clearance, roof, far_left);
    CHECK(spot.candidate == 0);
    CHECK(spot.feet.y == Approx(20.0 + 0.9 + 0.3 + 0.25 + 0.7));

    // Everything blocked: above the roof, the probe's ground height is not used.
    auto none = [](double, double) -> std::optional<double> { return std::nullopt; };
    spot = rg::select_spawn_spot(car, chassis, 0.0, r, door_x, clearance, roof, none);
    CHECK(spot.candidate == 4);
    CHECK(spot.above_roof);
    CHECK(spot.feet.x == Approx(10.0));
    CHECK(spot.feet.y == Approx(20.0));
    CHECK(spot.feet.z == Approx(1.5));

    // The probe's ground height becomes the spawn height.
    auto sloped = [](double x, double) -> std::optional<double> { return 0.1 * x; };
    spot = rg::select_spawn_spot(car, chassis, 0.0, r, door_x, clearance, roof, sloped);
    CHECK(spot.feet.z == Approx(0.1 * spot.feet.x));

    // A car yawed 90 deg (nose along +y): its left (driver) side is world -x.
    const rg::OrientedRect car90 = rg::make_oriented_rect(10.0, 20.0, kPi / 2.0, 2.0, 0.9, -0.5, 1.4);
    spot = rg::select_spawn_spot(car90, chassis, kPi / 2.0, r, door_x, clearance, roof, open);
    CHECK(spot.candidate == 0);
    CHECK(spot.feet.x == Approx(10.0 - 0.9 - 0.3 - 0.25));
    CHECK(spot.feet.y == Approx(20.3));
    CHECK(std::fabs(spot.yaw_rad) == Approx(kPi)); // looking along -x
}

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

TEST_CASE("walker: walks at 1.4 m/s and runs at 5 m/s, diagonals are not faster", "[on_foot]") {
    {
        Rig rig;
        rig.spawn(0.0, 0.0, 0.0);
        rig.run(forward(), 3.0);
        CHECK(rig.st().velocity.x == Approx(1.4).margin(0.01));
        CHECK(rig.st().velocity.y == Approx(0.0).margin(0.01));
        CHECK(rig.st().grounded);
        CHECK(rig.st().feet.z == Approx(0.0).margin(1e-3));
        // 3 s at 1.4 m/s minus the ~0.05 s acceleration.
        CHECK(rig.st().feet.x == Approx(4.2).margin(0.1));
        CHECK(rig.st().yaw_rad == Approx(0.0).margin(1e-6));
    }
    {
        Rig rig;
        rig.spawn(0.0, 0.0, 0.0);
        rig.run(forward(0.0, true), 3.0);
        CHECK(rig.st().velocity.x == Approx(5.0).margin(0.01));
        CHECK(rig.st().feet.x == Approx(14.6).margin(0.2)); // minus the 0.17 s acceleration
    }
    {
        // Diagonal: forward + right at once, still 1.4 m/s total; look yaw 90 deg
        // (+y) makes "right" +x and "forward" +y.
        Rig rig;
        rig.spawn(0.0, 0.0, 0.0);
        rg::WalkerInput in;
        in.move_forward = 1.0;
        in.move_right = 1.0;
        in.look_yaw_rad = kPi / 2.0;
        rig.run(in, 2.0);
        const double speed = std::hypot(rig.st().velocity.x, rig.st().velocity.y);
        CHECK(speed == Approx(1.4).margin(0.01));
        CHECK(rig.st().velocity.x == Approx(1.4 / std::sqrt(2.0)).margin(0.01));
        CHECK(rig.st().velocity.y == Approx(1.4 / std::sqrt(2.0)).margin(0.01));
        // Facing the direction of motion (45 deg).
        CHECK(rig.st().yaw_rad == Approx(kPi / 4.0).margin(0.02));
    }
    {
        // Releasing the stick stops the walker within a fraction of a second.
        Rig rig;
        rig.spawn(0.0, 0.0, 0.0);
        rig.run(forward(0.0, true), 1.0);
        rig.run(rg::WalkerInput{}, 0.5);
        CHECK(std::hypot(rig.st().velocity.x, rig.st().velocity.y) < 1e-6);
    }
}

TEST_CASE("walker: falls under gravity, lands, and jumps about 0.9 m", "[on_foot]") {
    Rig rig;
    rig.spawn(0.0, 0.0, 3.0);
    rig.run(rg::WalkerInput{}, 0.3);
    CHECK_FALSE(rig.st().grounded);
    CHECK(rig.st().feet.z < 3.0);
    CHECK(rig.st().velocity.z < 0.0);
    rig.run(rg::WalkerInput{}, 1.5);
    CHECK(rig.st().grounded);
    CHECK(rig.st().feet.z == Approx(0.0).margin(1e-3));
    CHECK(rig.st().velocity.z == 0.0);

    // Jump: one request tick, then watch the apex.
    rg::WalkerInput jump;
    jump.jump = true;
    rig.tick(jump);
    double apex = rig.st().feet.z;
    for (int i = 0; i < 240; ++i) {
        rig.tick(rg::WalkerInput{});
        apex = std::max(apex, rig.st().feet.z);
    }
    CHECK(apex == Approx(0.9).margin(0.05));
    CHECK(rig.st().grounded);
    CHECK(rig.st().feet.z == Approx(0.0).margin(1e-3));

    // A jump request in mid-air is ignored (no double jump).
    rig.tick(jump);
    for (int i = 0; i < 20; ++i) rig.tick(rg::WalkerInput{});
    const double z_mid = rig.st().feet.z;
    REQUIRE_FALSE(rig.st().grounded);
    rig.tick(jump);
    CHECK(rig.st().velocity.z < 4.2 - 9.81 * 20.0 / 240.0 + 0.1); // still falling on the original arc
    (void)z_mid;
}

TEST_CASE("walker: steps up a 12 cm kerb, is stopped by a 25 cm one", "[on_foot]") {
    {
        Rig rig;
        rig.add_box({5.0, 0.0, 0.06}, {1.0, 10.0, 0.06}); // kerb 12 cm high, x 4..6
        rig.spawn(0.0, 0.0, 0.0);
        rig.run(forward(), 5.0);
        CHECK(rig.st().feet.x > 6.0);
        CHECK(rig.st().feet.z == Approx(0.0).margin(1e-3)); // stepped up and back down
        CHECK(rig.st().grounded);
    }
    {
        Rig rig;
        rig.add_box({5.0, 0.0, 0.06}, {1.0, 10.0, 0.06});
        rig.spawn(0.0, 0.0, 0.0);
        rig.run(forward(), 3.6); // on top of it
        CHECK(rig.st().feet.x > 4.1);
        CHECK(rig.st().feet.z == Approx(0.12).margin(0.01));
        CHECK(rig.st().grounded);
    }
    {
        Rig rig;
        rig.add_box({5.0, 0.0, 0.125}, {1.0, 10.0, 0.125}); // 25 cm: a wall
        rig.spawn(0.0, 0.0, 0.0);
        rig.run(forward(), 5.0);
        CHECK(rig.st().feet.x < 4.0);
        CHECK(rig.st().feet.x > 3.5);
        CHECK(rig.st().feet.z == Approx(0.0).margin(1e-3));
        CHECK(rig.st().blocked);
    }
}

TEST_CASE("walker: stops at a wall and slides along it", "[on_foot]") {
    Rig rig;
    rig.add_box({6.0, 0.0, 1.5}, {0.5, 20.0, 1.5}); // wall face at x = 5.5
    rig.spawn(0.0, 0.0, 0.0);
    rig.run(forward(), 6.0);
    CHECK(rig.st().feet.x < 5.5);
    CHECK(rig.st().feet.x > 5.5 - 0.3 - 0.05);
    CHECK(rig.st().blocked);
    CHECK(rig.st().velocity.x == Approx(0.0).margin(0.05));
    // Diagonal into the wall: x stops, y keeps going (slide).
    const double y0 = rig.st().feet.y;
    rg::WalkerInput in;
    in.move_forward = 1.0;
    in.move_right = -1.0; // forward + left = +x +y at look yaw 0
    rig.run(in, 2.0);
    CHECK(rig.st().feet.x < 5.5);
    CHECK(rig.st().feet.y > y0 + 1.0);
    // Jumping against the wall does not climb it.
    rg::WalkerInput jump = forward();
    jump.jump = true;
    rig.tick(jump);
    rig.run(forward(), 1.0);
    CHECK(rig.st().feet.x < 5.5);
}

TEST_CASE("walker: climbs a 30 degree slope and refuses a 60 degree one", "[on_foot]") {
    {
        Rig rig;
        rig.add_ramp(3.0, 30.0);
        rig.spawn(0.0, 0.0, 0.0);
        rig.run(forward(), 4.5);
        const double x = rig.st().feet.x;
        REQUIRE(x > 5.0);
        CHECK(rig.st().grounded);
        // Height follows the slope (the capsule's round bottom floats up to ~5 cm).
        CHECK(rig.st().feet.z == Approx((x - 3.0) * std::tan(30.0 * kPi / 180.0)).margin(0.12));
        CHECK(rig.st().ground_normal.z == Approx(std::cos(30.0 * kPi / 180.0)).margin(0.01));
        // Coming back down stays glued to the ground (no airborne phase).
        rg::WalkerInput back;
        back.move_forward = -1.0;
        bool airborne = false;
        for (int i = 0; i < 600; ++i) {
            rig.tick(back);
            if (!rig.st().grounded) airborne = true;
        }
        CHECK_FALSE(airborne);
        CHECK(rig.st().feet.x < 3.5);
    }
    {
        Rig rig;
        rig.add_ramp(3.0, 60.0);
        rig.spawn(0.0, 0.0, 0.0);
        rig.run(forward(0.0, true), 4.0);
        CHECK(rig.st().feet.z < 0.3);
        CHECK(rig.st().feet.x < 3.6);
        CHECK(rig.st().grounded);
    }
}

TEST_CASE("walker: the own-car footprint blocks and deflects", "[on_foot]") {
    Rig rig;
    const rg::OrientedRect car = rg::make_oriented_rect(8.0, 0.0, 0.0, 2.0, 1.0, -1.0, 1.5);
    rig.walker->set_obstacles(std::span<const rg::OrientedRect>(&car, 1));
    rig.spawn(0.0, 0.0, 0.0);
    rig.run(forward(), 8.0); // straight at the nose: x_edge = 6
    CHECK(rig.st().feet.x < 6.0 - 0.3 + 1e-3);
    CHECK(rig.st().feet.x > 6.0 - 0.3 - 0.05);
    CHECK(rig.st().blocked);
    // Walk around it: sidestep left, forward past it, step back to the line.
    rg::WalkerInput left;
    left.move_right = -1.0;
    rig.run(left, 2.0);
    rig.run(forward(), 6.0);
    CHECK(rig.st().feet.x > 11.0);
    CHECK(rig.st().feet.y > 1.0);
    // A walker above the car's z range is not blocked (a jump over a low wall would be): spawn above.
    Rig rig2;
    rig2.walker->set_obstacles(std::span<const rg::OrientedRect>(&car, 1));
    rig2.spawn(7.0, 0.0, 3.0); // above the roof, falls through the prism and is pushed out sideways
    rig2.run(rg::WalkerInput{}, 1.5);
    CHECK(rig2.st().grounded);
    CHECK(rg::distance_to_rect(car, rig2.st().feet.x, rig2.st().feet.y) >= 0.3 - 1e-3);
}

TEST_CASE("walker: holds its height where no ground is loaded, falls off a real ledge", "[on_foot]") {
    {
        // No ground anywhere: never falls into the void.
        Rig rig(1, false);
        rig.spawn(0.0, 0.0, 5.0);
        rig.run(rg::WalkerInput{}, 2.0);
        CHECK(rig.st().feet.z == Approx(5.0).margin(1e-9));
        CHECK(rig.st().hold);
        CHECK(rig.st().velocity.z == 0.0);
        // Ground appears (the tile streamed in): the walker drops onto it.
        rig.add_box({0.0, 0.0, -0.5}, {50.0, 50.0, 0.5});
        rig.run(rg::WalkerInput{}, 2.0);
        CHECK(rig.st().grounded);
        CHECK(rig.st().feet.z == Approx(0.0).margin(1e-3));
        CHECK_FALSE(rig.st().hold);
    }
    {
        // A 2 m high platform: the walker steps off its edge and falls to the lower ground.
        Rig rig(1, false);
        rig.add_box({0.0, 0.0, -0.5}, {200.0, 200.0, 0.5});  // lower ground, top z = 0
        rig.add_box({-5.0, 0.0, 1.0}, {5.0, 20.0, 1.0});      // platform x -10..0, top z = 2
        rig.spawn(-3.0, 0.0, 2.0);
        rig.run(forward(), 2.4);
        REQUIRE(rig.st().feet.x > 0.0);
        bool fell = false;
        for (int i = 0; i < 120; ++i) {
            rig.tick(forward());
            if (!rig.st().grounded && rig.st().velocity.z < -1.0) fell = true;
        }
        CHECK(fell);
        CHECK(rig.st().grounded);
        CHECK(rig.st().feet.z == Approx(0.0).margin(1e-3));
    }
    {
        // A 20 cm drop (kerb height) is walked down without leaving the ground.
        Rig rig(1, false);
        rig.add_box({0.0, 0.0, -0.5}, {200.0, 200.0, 0.5});
        rig.add_box({-5.0, 0.0, 0.1}, {5.0, 20.0, 0.1}); // top z = 0.2, x -10..0
        rig.spawn(-3.0, 0.0, 0.2);
        bool airborne = false;
        for (int i = 0; i < 720; ++i) {
            rig.tick(forward());
            if (!rig.st().grounded) airborne = true;
        }
        CHECK_FALSE(airborne);
        CHECK(rig.st().feet.x > 0.0);
        CHECK(rig.st().feet.z == Approx(0.0).margin(1e-3));
    }
}

TEST_CASE("walker: the kinematic body follows the controller state", "[on_foot]") {
    Rig rig;
    rig.spawn(1.0, 2.0, 0.0);
    CHECK(rig.walker->active());
    rig.run(forward(0.3, true), 1.0);
    const ps::Pose pose = rig.world->get_pose(rig.walker->body());
    CHECK(pose.position.x == Approx(rig.st().feet.x).margin(1e-9));
    CHECK(pose.position.y == Approx(rig.st().feet.y).margin(1e-9));
    CHECK(pose.position.z == Approx(rig.st().feet.z + 0.5 * 1.75).margin(1e-9));
    const ps::Motion m = rig.world->get_motion(rig.walker->body());
    CHECK(m.linear.x == Approx(rig.st().velocity.x).margin(1e-6));
    // despawn removes the body; the world keeps stepping.
    rig.walker->despawn();
    CHECK_FALSE(rig.walker->active());
    rig.world->step();
}

TEST_CASE("walker: ignored bodies do not block (the own car's thin chassis box)", "[on_foot]") {
    Rig rig;
    const ps::BodyId box = rig.add_box({5.0, 0.0, 1.5}, {0.5, 5.0, 1.5});
    rig.walker->set_ignored_bodies({box});
    rig.spawn(0.0, 0.0, 0.0);
    rig.run(forward(), 5.0);
    CHECK(rig.st().feet.x > 5.5); // walked through it
}

TEST_CASE("walker: state hash is identical at 1 and 4 workers and across runs", "[on_foot][determinism]") {
    auto run = [](unsigned workers) {
        Rig rig(workers);
        rig.add_box({5.0, 0.0, 0.06}, {1.0, 10.0, 0.06});
        rig.add_ramp(12.0, 25.0);
        rig.add_box({14.0, 4.0, 1.5}, {0.5, 3.0, 1.5});
        rig.spawn(0.0, 0.0, 1.0);
        const rg::OrientedRect car = rg::make_oriented_rect(8.0, -4.0, 0.3, 2.0, 1.0, -1.0, 1.5);
        rig.walker->set_obstacles(std::span<const rg::OrientedRect>(&car, 1));
        for (int i = 0; i < 1500; ++i) {
            rg::WalkerInput in;
            const double t = i * rig.dt;
            in.move_forward = 1.0;
            in.move_right = std::fmod(t, 3.0) < 1.5 ? 0.0 : 0.6;
            in.look_yaw_rad = 0.2 * std::fmod(t, 5.0);
            in.run = std::fmod(t, 4.0) < 2.0;
            in.jump = (i % 400) == 399;
            rig.tick(in);
        }
        return std::pair{rig.world->state_hash(), rig.st().feet};
    };
    const auto a = run(1);
    const auto b = run(4);
    const auto c = run(1);
    CHECK(a.first == b.first);
    CHECK(a.first == c.first);
    CHECK(a.second == b.second);
    CHECK(a.second.x > 3.0); // it actually went somewhere
}
