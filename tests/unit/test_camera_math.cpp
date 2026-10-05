// test_camera_math.cpp - rg camera math (PLAN.md R5): the view cycle, the
// bumper eye from a real vehicle file, the orbit controller, roadside shot
// placement, the predicted-path fallback and the cinematic director (cuts,
// determinism, side clearance, source tagging).
#include "rg/camera_math.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace {

using Catch::Matchers::WithinAbs;

constexpr double kPi = 3.14159265358979323846;

std::vector<rg::RoadPoint> straight_road(double length = 400.0, double half_width = 3.5) {
    std::vector<rg::RoadPoint> r;
    for (double x = 0.0; x <= length + 1e-9; x += 10.0) r.push_back({x, 0.0, half_width});
    return r;
}

rg::CarKinematics car_at(double x, double y, double speed, double heading_rad) {
    rg::CarKinematics c;
    c.x = x;
    c.y = y;
    c.heading_x = std::cos(heading_rad);
    c.heading_y = std::sin(heading_rad);
    c.vx = c.heading_x * speed;
    c.vy = c.heading_y * speed;
    return c;
}

} // namespace

TEST_CASE("camera math: the view cycle", "[camera_math]") {
    using rg::DriveView;
    // Cycles through every view once and returns to the start.
    std::set<int> seen;
    DriveView v = DriveView::Chase;
    for (int i = 0; i < rg::kDriveViewCount; ++i) {
        seen.insert(static_cast<int>(v));
        v = rg::next_drive_view(v);
    }
    CHECK(seen.size() == static_cast<std::size_t>(rg::kDriveViewCount));
    CHECK(v == DriveView::Chase);
    CHECK(rg::next_drive_view(DriveView::Chase) == DriveView::Bumper);
    CHECK(rg::next_drive_view(DriveView::Cinematic) == DriveView::Chase);

    // The Tab key.
    CHECK(rg::toggle_cockpit_view(DriveView::Chase) == DriveView::Cockpit);
    CHECK(rg::toggle_cockpit_view(DriveView::Bumper) == DriveView::Cockpit);
    CHECK(rg::toggle_cockpit_view(DriveView::Orbit) == DriveView::Cockpit);
    CHECK(rg::toggle_cockpit_view(DriveView::Cockpit) == DriveView::Chase);

    // Names round trip and are distinct.
    for (int i = 0; i < rg::kDriveViewCount; ++i) {
        const auto d = static_cast<DriveView>(i);
        const auto back = rg::drive_view_from_string(rg::to_string(d));
        REQUIRE(back.has_value());
        CHECK(*back == d);
    }
    CHECK_FALSE(rg::drive_view_from_string("free").has_value());
    CHECK_FALSE(rg::drive_view_from_string("").has_value());
}

TEST_CASE("camera math: the bumper eye follows the wheels of the shipped cars", "[camera_math]") {
    // A literal four-wheel description (the real files' wheel layout is pinned
    // by test_vehicle_data.cpp; this checks the arithmetic).
    ps::vehicle::VehicleDesc desc;
    ps::vehicle::WheelDesc fl, fr, rl, rr;
    fl.attachment_local = {1.35, 0.8, -0.2};
    fr.attachment_local = {1.35, -0.8, -0.2};
    rl.attachment_local = {-1.25, 0.8, -0.2};
    rr.attachment_local = {-1.25, -0.8, -0.2};
    for (auto* w : {&fl, &fr, &rl, &rr}) w->wheel_radius = 0.33;
    desc.wheels = {fl, fr, rl, rr};

    const rg::VehicleBounds b = rg::vehicle_bounds(desc);
    CHECK_THAT(b.front_axle_x, WithinAbs(1.35, 1e-12));
    CHECK_THAT(b.rear_axle_x, WithinAbs(-1.25, 1e-12));
    CHECK_THAT(b.half_track_m, WithinAbs(0.8, 1e-12));
    CHECK_THAT(b.wheel_radius_m, WithinAbs(0.33, 1e-12));
    CHECK_THAT(b.ground_z, WithinAbs(-0.2 - 0.33, 1e-12));

    const rg::BumperParams p;
    const ps::Vec3 eye = rg::bumper_eye_local(b, p);
    CHECK_THAT(static_cast<double>(eye.x), WithinAbs(1.35 + p.forward_of_front_axle_m, 1e-12));
    CHECK_THAT(static_cast<double>(eye.y), WithinAbs(0.0, 1e-12));
    CHECK_THAT(static_cast<double>(eye.z), WithinAbs(-0.53 + p.height_above_ground_m, 1e-12));
    // In front of the front axle, above the ground, below a roof.
    CHECK(eye.x > b.front_axle_x);
    CHECK(eye.z > b.ground_z);
    CHECK(eye.z - b.ground_z < 1.0);

    // A vehicle without wheels gets the documented defaults, never garbage.
    const rg::VehicleBounds d = rg::vehicle_bounds(ps::vehicle::VehicleDesc{});
    CHECK_THAT(d.front_axle_x, WithinAbs(1.3, 1e-12));
}

TEST_CASE("camera math: orbit controller", "[camera_math]") {
    const rg::OrbitParams p;
    rg::OrbitState s;
    // Idle input leaves the pose alone until the idle delay, then drifts.
    rg::OrbitInput none;
    rg::OrbitState a = s;
    for (int i = 0; i < 100; ++i) a = rg::orbit_step(a, none, 0.016, p); // 1.6 s
    CHECK_THAT(a.azimuth_rad, WithinAbs(0.0, 1e-12));
    for (int i = 0; i < 200; ++i) a = rg::orbit_step(a, none, 0.016, p); // 4.8 s total
    CHECK(a.azimuth_rad > 0.0);
    CHECK(a.idle_s > p.idle_delay_s);

    // Input resets the idle timer and stops the drift.
    rg::OrbitInput right;
    right.look_x = 1.0;
    const rg::OrbitState b = rg::orbit_step(a, right, 0.016, p);
    CHECK(b.idle_s == 0.0);
    CHECK_THAT(b.azimuth_rad - a.azimuth_rad, WithinAbs(p.look_rate_rad_s * 0.016, 1e-12));

    // An input with live=false (a panel is open) does nothing and does not
    // reset the idle timer.
    rg::OrbitInput dead = right;
    dead.live = false;
    const rg::OrbitState c = rg::orbit_step(a, dead, 0.016, p);
    CHECK(c.idle_s > a.idle_s);

    // Clamps.
    rg::OrbitInput up;
    up.look_y = 1.0;
    up.zoom_steps = 100.0;
    rg::OrbitState d = s;
    for (int i = 0; i < 400; ++i) d = rg::orbit_step(d, up, 0.016, p);
    CHECK_THAT(d.elevation_rad, WithinAbs(p.max_elevation_rad, 1e-12));
    CHECK_THAT(d.distance_m, WithinAbs(p.min_distance_m, 1e-12));
    rg::OrbitInput down;
    down.look_y = -1.0;
    down.zoom_steps = -100.0;
    for (int i = 0; i < 400; ++i) d = rg::orbit_step(d, down, 0.016, p);
    CHECK_THAT(d.elevation_rad, WithinAbs(p.min_elevation_rad, 1e-12));
    CHECK_THAT(d.distance_m, WithinAbs(p.max_distance_m, 1e-12));

    // The azimuth wraps instead of growing without bound.
    rg::OrbitState w = s;
    for (int i = 0; i < 2000; ++i) w = rg::orbit_step(w, right, 0.016, p);
    CHECK(w.azimuth_rad <= kPi);
    CHECK(w.azimuth_rad > -kPi);

    // The offset: azimuth 0 is straight behind, + is to the right (-y is
    // right in the left-handed ISO frame's y = left), never below the ground
    // plane, at the requested distance.
    rg::OrbitState o;
    o.azimuth_rad = 0.0;
    o.elevation_rad = 0.0;
    o.distance_m = 8.0;
    ps::Vec3 off = rg::orbit_offset(o);
    CHECK_THAT(static_cast<double>(off.x), WithinAbs(-8.0, 1e-12));
    CHECK_THAT(static_cast<double>(off.y), WithinAbs(0.0, 1e-12));
    CHECK_THAT(static_cast<double>(off.z), WithinAbs(0.0, 1e-12));
    o.azimuth_rad = kPi / 2.0; // camera to the right of the car
    off = rg::orbit_offset(o);
    CHECK_THAT(static_cast<double>(off.x), WithinAbs(0.0, 1e-9));
    CHECK_THAT(static_cast<double>(off.y), WithinAbs(-8.0, 1e-12));
    o.elevation_rad = kPi / 4.0;
    off = rg::orbit_offset(o);
    const double len = std::sqrt(off.x * off.x + off.y * off.y + off.z * off.z);
    CHECK_THAT(len, WithinAbs(8.0, 1e-12));
    CHECK(off.z > 0.0);
}

TEST_CASE("camera math: roadside shot placement", "[camera_math]") {
    const rg::CinematicParams p;
    const auto road = straight_road();
    const auto left = rg::place_roadside_shot(road, 100.0, +1, 2.0, p, rg::ShotSource::RoadAhead);
    const auto right = rg::place_roadside_shot(road, 100.0, -1, 2.0, p, rg::ShotSource::RoadAhead);
    REQUIRE(left.has_value());
    REQUIRE(right.has_value());
    CHECK_THAT(left->x, WithinAbs(100.0, 1e-9));
    CHECK_THAT(right->x, WithinAbs(100.0, 1e-9));
    // Left of the travel direction (+x) is +y; clear of the road edge.
    CHECK_THAT(left->y, WithinAbs(3.5 + p.side_clearance_m, 1e-9));
    CHECK_THAT(right->y, WithinAbs(-(3.5 + p.side_clearance_m), 1e-9));
    CHECK(left->side == 1);
    CHECK(right->side == -1);
    CHECK(left->source == rg::ShotSource::RoadAhead);
    CHECK_THAT(left->height_above_ground_m, WithinAbs(2.0, 1e-12));

    // Fewer than two points: no shot. A lead beyond the end clamps to the end.
    CHECK_FALSE(rg::place_roadside_shot({}, 10.0, 1, 2.0, p, rg::ShotSource::RoadAhead).has_value());
    CHECK_FALSE(rg::place_roadside_shot({{0, 0, 3}}, 10.0, 1, 2.0, p, rg::ShotSource::RoadAhead).has_value());
    const auto far = rg::place_roadside_shot(road, 5000.0, 1, 2.0, p, rg::ShotSource::PredictedPath);
    REQUIRE(far.has_value());
    CHECK_THAT(far->x, WithinAbs(400.0, 1e-9));
    CHECK(far->source == rg::ShotSource::PredictedPath);

    // On a road heading north the left side is -x (west).
    std::vector<rg::RoadPoint> north;
    for (double y = 0.0; y <= 200.0; y += 10.0) north.push_back({50.0, y, 4.0});
    const auto n = rg::place_roadside_shot(north, 60.0, 1, 2.0, p, rg::ShotSource::RoadAhead);
    REQUIRE(n.has_value());
    CHECK_THAT(n->x, WithinAbs(50.0 - (4.0 + p.side_clearance_m), 1e-9));
    CHECK_THAT(n->y, WithinAbs(60.0, 1e-9));
}

TEST_CASE("camera math: predicted path fallback", "[camera_math]") {
    const rg::CinematicParams p;
    // Straight history: a straight prediction ahead of the car.
    std::vector<std::pair<double, double>> hist;
    for (int i = 0; i <= 20; ++i) hist.emplace_back(i * 5.0, 0.0);
    const rg::CarKinematics straight = car_at(100.0, 0.0, 20.0, 0.0);
    const auto line = rg::predict_path_ahead(hist, straight, p);
    REQUIRE(line.size() >= 10);
    CHECK_THAT(line.front().x, WithinAbs(100.0, 1e-9));
    for (const auto& pt : line) CHECK_THAT(pt.y, WithinAbs(0.0, 1e-9));
    CHECK_THAT(line.back().x - line.front().x, WithinAbs(p.predict_length_m, p.predict_step_m + 1e-9));

    // A left-hand circle: the prediction keeps curving left.
    std::vector<std::pair<double, double>> arc;
    const double radius = 60.0, speed = 15.0;
    const double omega = speed / radius;
    for (int i = 0; i <= 32; ++i) {
        const double t = i * p.history_step_s;
        arc.emplace_back(radius * std::sin(omega * t), radius * (1.0 - std::cos(omega * t)));
    }
    const double th = omega * 32 * p.history_step_s;
    rg::CarKinematics turning = car_at(arc.back().first, arc.back().second, speed, th);
    const auto curve = rg::predict_path_ahead(arc, turning, p);
    REQUIRE(curve.size() >= 5);
    // Left turn: the final heading has rotated counter-clockwise.
    const double end_heading = std::atan2(curve.back().y - curve[curve.size() - 2].y, curve.back().x - curve[curve.size() - 2].x);
    const double start_heading = std::atan2(curve[1].y - curve[0].y, curve[1].x - curve[0].x);
    double dh = end_heading - start_heading;
    dh = std::atan2(std::sin(dh), std::cos(dh));
    CHECK(dh > 0.3);

    // Standing still: no turn-rate extrapolation, straight along the heading.
    const rg::CarKinematics parked = car_at(0.0, 0.0, 0.0, kPi / 2.0);
    const auto up = rg::predict_path_ahead(arc, parked, p);
    REQUIRE(up.size() >= 2);
    CHECK_THAT(up.back().x, WithinAbs(0.0, 1e-6));
    CHECK(up.back().y > 100.0);
}

TEST_CASE("camera math: cinematic director cuts, clearance and source", "[camera_math]") {
    const rg::CinematicParams p;
    rg::CinematicDirector dir(42);
    const auto road = straight_road(2000.0, 3.5);

    CHECK_FALSE(dir.has_shot());
    double x = 0.0;
    const double speed = 25.0, dt = 1.0 / 60.0;
    std::set<std::uint64_t> serials;
    int cuts = 0;
    double last_cut_t = -1e9;
    double min_gap = 1e9;
    for (int i = 0; i < 60 * 40; ++i) {
        x += speed * dt;
        // The road ahead from the car: session coordinates, from the car on.
        std::vector<rg::RoadPoint> ahead;
        for (const auto& r : road) {
            if (r.x >= x - 5.0 && r.x <= x + 170.0) ahead.push_back(r);
        }
        const auto& shot = dir.update(dt, car_at(x, 0.0, speed, 0.0), ahead);
        serials.insert(shot.serial);
        if (dir.cut_this_update()) {
            ++cuts;
            const double t = i * dt;
            min_gap = std::min(min_gap, t - last_cut_t);
            last_cut_t = t;
            // Every shot is beside the road, clear of its edge, with real road data.
            CHECK(shot.source == rg::ShotSource::RoadAhead);
            CHECK(std::fabs(shot.y) >= 3.5 + p.side_clearance_m - 1e-9);
            CHECK(shot.height_above_ground_m >= p.min_height_m);
            CHECK(shot.height_above_ground_m <= p.max_height_m);
            CHECK(shot.fov_deg >= 14.0);
            CHECK(shot.fov_deg <= p.base_fov_deg);
            CHECK(shot.x > x); // ahead of the car at the cut
        }
    }
    CHECK(cuts >= 4);
    CHECK(cuts == static_cast<int>(dir.shots_started()));
    CHECK(serials.size() == static_cast<std::size_t>(cuts));
    CHECK((min_gap >= p.min_hold_s - 1e-9 - 1.0 / 60.0 || cuts == 1));

    // A shot never outlives max_hold_s.
    rg::CinematicDirector still(7);
    const auto parked = car_at(0.0, 0.0, 0.0, 0.0);
    std::uint64_t first = 0;
    for (int i = 0; i < 60 * 30; ++i) {
        const auto& s = still.update(dt, parked, road);
        if (i == 0) first = s.serial;
    }
    CHECK(first == 1);
    CHECK(still.shots_started() >= 2);
    CHECK(still.shots_started() <= 4);
}

TEST_CASE("camera math: cinematic director is deterministic and sides alternate", "[camera_math]") {
    const auto road = straight_road(3000.0, 3.0);
    const double dt = 1.0 / 60.0;
    auto run = [&](std::uint64_t seed) {
        rg::CinematicDirector d(seed);
        std::vector<rg::CinematicShot> shots;
        double x = 0.0;
        std::uint64_t seen = 0;
        for (int i = 0; i < 60 * 60; ++i) {
            x += 30.0 * dt;
            std::vector<rg::RoadPoint> ahead;
            for (const auto& r : road) {
                if (r.x >= x - 5.0 && r.x <= x + 170.0) ahead.push_back(r);
            }
            const auto& s = d.update(dt, car_at(x, 0.0, 30.0, 0.0), ahead);
            if (s.serial != seen) {
                shots.push_back(s);
                seen = s.serial;
            }
        }
        return shots;
    };
    const auto a = run(5);
    const auto b = run(5);
    const auto c = run(6);
    REQUIRE(a.size() == b.size());
    REQUIRE(a.size() >= 6);
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i].x == b[i].x);
        CHECK(a[i].y == b[i].y);
        CHECK(a[i].height_above_ground_m == b[i].height_above_ground_m);
        CHECK(a[i].fov_deg == b[i].fov_deg);
        CHECK(a[i].side == b[i].side);
    }
    bool differs = a.size() != c.size();
    for (std::size_t i = 0; i < std::min(a.size(), c.size()) && !differs; ++i) {
        differs = a[i].height_above_ground_m != c[i].height_above_ground_m || a[i].side != c[i].side;
    }
    CHECK(differs);

    // Both sides get used.
    int left = 0, right = 0;
    for (const auto& s : a) (s.side > 0 ? left : right)++;
    CHECK(left >= 1);
    CHECK(right >= 1);

    // reset() restores the exact sequence.
    rg::CinematicDirector d(5);
    const auto car = car_at(0.0, 0.0, 30.0, 0.0);
    std::vector<rg::RoadPoint> ahead;
    for (const auto& r : road) {
        if (r.x <= 170.0) ahead.push_back(r);
    }
    const rg::CinematicShot first = d.update(dt, car, ahead);
    d.update(dt, car, ahead);
    d.reset();
    CHECK_FALSE(d.has_shot());
    const rg::CinematicShot again = d.update(dt, car, ahead);
    CHECK(first.x == again.x);
    CHECK(first.y == again.y);
    CHECK(first.serial == 1);
    CHECK(again.serial == 1);
}

TEST_CASE("camera math: without road data the shot comes from the predicted path and says so", "[camera_math]") {
    rg::CinematicDirector dir(3);
    const double dt = 1.0 / 60.0;
    double x = 0.0;
    for (int i = 0; i < 60 * 20; ++i) {
        x += 20.0 * dt;
        const auto& s = dir.update(dt, car_at(x, 0.0, 20.0, 0.0), {});
        CHECK(s.source == rg::ShotSource::PredictedPath);
        if (dir.cut_this_update()) {
            // Beside the straight path, well clear of the car's own lane (3 m
            // half width + clearance), and ahead.
            CHECK(std::fabs(s.y) >= 3.0 + dir.params().side_clearance_m - 1e-9);
            CHECK(s.x > x);
        }
    }
    CHECK(dir.shots_started() >= 3);

    // A short road stub counts as no road data (< 30 m): predicted path again.
    rg::CinematicDirector stub(3);
    const std::vector<rg::RoadPoint> tiny{{0, 0, 3}, {10, 0, 3}};
    const auto& s = stub.update(dt, car_at(0.0, 0.0, 20.0, 0.0), tiny);
    CHECK(s.source == rg::ShotSource::PredictedPath);
}

TEST_CASE("camera math: the shot is cut when the car has passed it or it is too far", "[camera_math]") {
    rg::CinematicParams p;
    p.min_hold_s = 0.0;
    p.max_hold_s = 1000.0;
    rg::CinematicDirector dir(11, p);
    const auto road = straight_road(1000.0);
    const double dt = 1.0 / 60.0;
    dir.update(dt, car_at(0.0, 0.0, 30.0, 0.0), road);
    const auto first = dir.shot();
    // Car well before the camera: same shot.
    dir.update(dt, car_at(first.x - 40.0, 0.0, 30.0, 0.0), road);
    CHECK(dir.shot().serial == first.serial);
    // Car passed it by more than cut_when_passed_m: new shot.
    dir.update(dt, car_at(first.x + p.cut_when_passed_m + 1.0, 0.0, 30.0, 0.0), road);
    CHECK(dir.shot().serial == first.serial + 1);
    CHECK(dir.cut_this_update());
    // Teleported far away: new shot.
    const auto second = dir.shot();
    dir.update(dt, car_at(second.x - p.cut_when_far_m - 10.0, 0.0, 30.0, 0.0), road);
    CHECK(dir.shot().serial == second.serial + 1);
}

// ------------------------------------------------------------------ chase --
// The chase camera must stay at its wanted offset from the car whatever the speed, acceleration or frame rate (owner:
// "the first camera still lags"). The earlier rig smoothed the camera's ABSOLUTE position, which trails a moving car
// by speed / rate metres; rg::chase_follow_offset smooths the car-relative offset instead.
namespace {

struct ChaseRun {
    double max_err_m = 0.0; // worst |camera - car - wanted offset| after the first second
    double final_err_m = 0.0;
};

ps::Vec3 add(const ps::Vec3& a, const ps::Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
ps::Vec3 sub(const ps::Vec3& a, const ps::Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
double len(const ps::Vec3& a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }

enum class FollowKind { Offset, AbsolutePosition /* the earlier rig, kept as the reference */, FixedAlphaPerFrame /* sabotage */ };

// Car on a straight line with speed v(t) = v0 + a*t (turn_rate = 0) or on a circle at constant speed v0 with the given
// yaw rate. The wanted offset is 7 m behind and 2.5 m above, rotated with the car's heading.
ChaseRun run_chase(FollowKind kind, double fps, double v0, double accel, double turn_rate, double seconds = 8.0,
                   double rate = 6.0) {
    const double dt = 1.0 / fps;
    double x = 0.0, y = 0.0, heading = 0.0, v = v0;
    auto wanted = [&](double h) { return ps::Vec3{-7.0 * std::cos(h), -7.0 * std::sin(h), 2.5}; };
    ps::Vec3 offset = wanted(heading);
    ps::Vec3 abs_cam = add(ps::Vec3{x, y, 0.0}, offset);
    ChaseRun r;
    const int frames = static_cast<int>(seconds * fps);
    for (int i = 0; i < frames; ++i) {
        v += accel * dt;
        heading += turn_rate * dt;
        x += std::cos(heading) * v * dt;
        y += std::sin(heading) * v * dt;
        const ps::Vec3 car{x, y, 0.0};
        const ps::Vec3 want = wanted(heading);
        double err = 0.0;
        switch (kind) {
        case FollowKind::Offset:
            offset = rg::chase_follow_offset(offset, want, dt, rate);
            err = len(sub(offset, want));
            break;
        case FollowKind::AbsolutePosition: {
            const double a = 1.0 - std::exp(-rate * dt);
            const ps::Vec3 target = add(car, want);
            abs_cam = add(abs_cam, ps::Vec3{(target.x - abs_cam.x) * a, (target.y - abs_cam.y) * a, (target.z - abs_cam.z) * a});
            err = len(sub(abs_cam, target));
            break;
        }
        case FollowKind::FixedAlphaPerFrame:
            offset = rg::chase_follow_offset(offset, want, 1.0 / 60.0, rate); // ignores the real dt
            err = len(sub(offset, want));
            break;
        }
        if (i >= static_cast<int>(fps)) r.max_err_m = std::max(r.max_err_m, err);
        r.final_err_m = err;
    }
    return r;
}

} // namespace

TEST_CASE("camera math: chase offset smoothing", "[camera_math]") {
    SECTION("straight driving: the camera stays exactly at its offset at any speed, acceleration and frame rate") {
        for (const double fps : {30.0, 60.0, 144.0, 240.0}) {
            for (const double v : {0.0, 10.0, 30.0, 60.0}) {
                const ChaseRun cruise = run_chase(FollowKind::Offset, fps, v, 0.0, 0.0);
                const ChaseRun accel = run_chase(FollowKind::Offset, fps, v, 6.0, 0.0);
                CHECK(cruise.max_err_m < 1e-9);
                CHECK(accel.max_err_m < 1e-9);
            }
        }
    }

    SECTION("before: smoothing the absolute position trailed the car by speed / rate") {
        // The numbers the fix is measured against: 10 / 30 / 60 m/s at 60 fps.
        const double e10 = run_chase(FollowKind::AbsolutePosition, 60.0, 10.0, 0.0, 0.0).max_err_m;
        const double e30 = run_chase(FollowKind::AbsolutePosition, 60.0, 30.0, 0.0, 0.0).max_err_m;
        const double e60 = run_chase(FollowKind::AbsolutePosition, 60.0, 60.0, 0.0, 0.0).max_err_m;
        INFO("absolute-position follow lag at 10/30/60 m/s: " << e10 << " / " << e30 << " / " << e60 << " m");
        CHECK_THAT(e10, WithinAbs(10.0 / 6.0, 0.1 + 0.05 * 10.0 / 6.0));
        CHECK_THAT(e30, WithinAbs(30.0 / 6.0, 0.1 + 0.05 * 30.0 / 6.0));
        CHECK_THAT(e60, WithinAbs(60.0 / 6.0, 0.1 + 0.05 * 60.0 / 6.0));
        // ... which the offset form removes entirely.
        CHECK(run_chase(FollowKind::Offset, 60.0, 60.0, 0.0, 0.0).max_err_m < 1e-9);
    }

    SECTION("cornering: the swing-in error is the same curve at every frame rate and never worse than before") {
        // 15 m/s on a 50 m circle (yaw rate 0.3 rad/s): the wanted offset rotates, the camera eases after it.
        const double ref = run_chase(FollowKind::Offset, 240.0, 15.0, 0.0, 0.3).max_err_m;
        INFO("cornering swing error at 240 fps: " << ref << " m");
        for (const double fps : {30.0, 60.0, 144.0}) {
            const double e = run_chase(FollowKind::Offset, fps, 15.0, 0.0, 0.3).max_err_m;
            CHECK_THAT(e, WithinAbs(ref, (fps < 60.0 ? 0.12 : 0.06) * ref)); // first-order discretisation: O(rate * dt)
        }
        const double before = run_chase(FollowKind::AbsolutePosition, 60.0, 15.0, 0.0, 0.3).max_err_m;
        INFO("absolute-position follow in the same corner: " << before << " m");
        CHECK(run_chase(FollowKind::Offset, 60.0, 15.0, 0.0, 0.3).max_err_m < before);
    }

    SECTION("a changed wanted offset eases in with 1 - exp(-rate * t), whatever the frame rate") {
        for (const double fps : {30.0, 60.0, 240.0}) {
            ps::Vec3 o{-7.0, 0.0, 2.5};
            const ps::Vec3 want{-3.4, 0.0, 1.5};
            const int frames = static_cast<int>(0.5 * fps);
            for (int i = 0; i < frames; ++i) o = rg::chase_follow_offset(o, want, 1.0 / fps, 6.0);
            const double expected = -7.0 + 3.6 * (1.0 - std::exp(-6.0 * 0.5));
            CHECK_THAT(o.x, WithinAbs(expected, 1e-9));
        }
        // rate <= 0 = no smoothing.
        const ps::Vec3 o = rg::chase_follow_offset({-7, 0, 2.5}, {-3, 1, 1}, 0.016, 0.0);
        CHECK_THAT(o.x, WithinAbs(-3.0, 1e-12));
    }

    SECTION("a smoothing step that ignored the real frame time would show up as frame-rate dependence") {
        // The sabotage reference: always easing as if at 60 fps. At 240 fps the camera then swings in 4x faster.
        const double honest = run_chase(FollowKind::Offset, 240.0, 15.0, 0.0, 0.3).max_err_m;
        const double broken = run_chase(FollowKind::FixedAlphaPerFrame, 240.0, 15.0, 0.0, 0.3).max_err_m;
        INFO("honest " << honest << " m vs fixed-alpha " << broken << " m");
        CHECK(std::abs(honest - broken) > 0.1 * honest);
    }
}
