// test_camera_math.cpp - rg camera math (PLAN.md R5): the view cycle, the
// bumper eye from a real vehicle file, the orbit controller, roadside shot
// placement, the predicted-path fallback and the cinematic director (cuts,
// determinism, side clearance, source tagging).
#include "rg/camera_math.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
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

TEST_CASE("cockpit look: mouse and rate input pitch clamps and disabled input is inert", "[camera][cockpit]") {
    rg::CockpitLookConfig c{-60,10,0.15,100,0.2};
    auto s=rg::cockpit_look_step({0,-3},{20,20,0,0,true},0.01,c);
    CHECK_THAT(s.yaw_deg,WithinAbs(3,1e-12));
    CHECK_THAT(s.pitch_deg,WithinAbs(-6,1e-12));
    s=rg::cockpit_look_step(s,{0,1000,0,0,true},0.01,c);
    CHECK(s.pitch_deg==-60);
    s=rg::cockpit_look_step(s,{0,-1000,0,0,true},0.01,c);
    CHECK(s.pitch_deg==10);
    s=rg::cockpit_look_step(s,{100,100,1,1,false},1,c);
    CHECK(s.pitch_deg==10); CHECK_THAT(s.yaw_deg,WithinAbs(3,1e-12));
    const auto a=rg::cockpit_look_step({0,0},{0,0,0.1,0.1,true},1,c);
    CHECK(a.yaw_deg==0); CHECK(a.pitch_deg==0);
}
TEST_CASE("cockpit look: stick rates are frame independent and wheel angle honors lock", "[camera][cockpit]") {
    rg::CockpitLookConfig c{-60,10,0.15,100,0.2};
    rg::CockpitLook a,b;
    for(int i=0;i<60;++i)a=rg::cockpit_look_step(a,{0,0,0.6,-0.4,true},1.0/60,c);
    for(int i=0;i<240;++i)b=rg::cockpit_look_step(b,{0,0,0.6,-0.4,true},1.0/240,c);
    CHECK_THAT(a.yaw_deg,WithinAbs(b.yaw_deg,1e-10));
    CHECK_THAT(a.pitch_deg,WithinAbs(b.pitch_deg,1e-10));
    CHECK_THAT(rg::steering_wheel_angle_rad(0.25,360),WithinAbs(kPi/2,1e-12));
    CHECK_THAT(rg::steering_wheel_angle_rad(-1,360),WithinAbs(-2*kPi,1e-12));
    CHECK_THAT(rg::steering_wheel_angle_rad(2,180),WithinAbs(kPi,1e-12));
}

// ------------------------------------------------------------------ drone ----
// Golden comparison against the former GDScript drone_rig.gd, transcribed line by line below in Godot's own Y-up
// frame (Vector3 maths written out, lerp_angle as in Godot 4). The rg_core version works in a Z-up frame; the
// binding maps Godot (x, y, z) to (x, -z, y), done here by hand.
namespace {

struct G3 {
    double x = 0, y = 0, z = 0;
};
G3 operator+(G3 a, G3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
G3 operator-(G3 a, G3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
G3 operator*(G3 a, double k) { return {a.x * k, a.y * k, a.z * k}; }
double glen(G3 a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
G3 glerp(G3 a, G3 b, double w) { return a + (b - a) * w; }
G3 grotated_up(G3 v, double ang) { // Vector3.rotated(Vector3.UP, ang)
    const double c = std::cos(ang), s = std::sin(ang);
    return {v.x * c + v.z * s, v.y, -v.x * s + v.z * c};
}
double glerp_angle(double from, double to, double w) {
    const double tau = 6.283185307179586;
    const double diff = std::fmod(to - from, tau);
    const double dist = std::fmod(2.0 * diff, tau) - diff;
    return from + dist * w;
}
double gsignf(double v) { return v > 0 ? 1.0 : (v < 0 ? -1.0 : 0.0); }

// The former drone_rig.gd, state and update_rig body (the ground probes are fed in by the caller).
struct GdDrone {
    double follow_distance_m = 12.0, height_per_distance = 0.5, look_height_m = 1.0, terrain_clearance_m = 1.0;
    double position_lag = 4.0, orbit_deadzone = 0.2, orbit_lag = 10.0, zoom_wheel_factor = 1.15, zoom_key_rate = 1.2;
    const double SNAP_M = 60.0, MIN_DISTANCE_M = 4.0, MAX_DISTANCE_M = 60.0;
    double _distance_m = 12.0;
    G3 _smoothed_pos, _smoothed_offset, position, look_at_pos;
    bool _from_absolute = false, _initialized = false, _drain_wheel = false, looked = false;
    double _orbit_yaw = 0.0;
    G3 _heading{0, 0, -1};

    GdDrone() { _distance_m = std::clamp(follow_distance_m, MIN_DISTANCE_M, MAX_DISTANCE_M); }
    void activate(G3 from) {
        _smoothed_pos = from;
        _from_absolute = true;
        _initialized = true;
        _orbit_yaw = 0.0;
        _drain_wheel = true;
    }
    void shift_origin(G3 d) {
        position = position + d;
        _smoothed_pos = _smoothed_pos + d;
    }
    double target_orbit_yaw(double lx, double ly, double sx, double sy) {
        if (std::fabs(lx) > 0.5) return gsignf(lx) * 3.141592653589793 * 0.5;
        if (ly < -0.5) return 3.141592653589793;
        const double mag = std::min(std::sqrt(sx * sx + sy * sy), 1.0);
        if (mag <= orbit_deadzone) return 0.0;
        return std::atan2(sx, sy);
    }
    void update_zoom(double delta, double wheel, double key, bool live) {
        if (_drain_wheel) {
            _drain_wheel = false;
            return;
        }
        if (!live) return;
        _distance_m *= std::pow(zoom_wheel_factor, -wheel) * std::exp(-key * zoom_key_rate * delta);
        _distance_m = std::clamp(_distance_m, MIN_DISTANCE_M, MAX_DISTANCE_M);
    }
    // grounds: one optional height per probe (5), in the script's probe order.
    void update_rig(double delta, G3 target_pos, G3 basis_x, bool live, double lx, double ly, double sx, double sy,
                    const std::vector<std::optional<double>>& grounds) {
        const G3 fwd{basis_x.x, 0.0, basis_x.z};
        if (glen(fwd) > 0.2) _heading = fwd * (1.0 / glen(fwd));
        const G3 behind = _heading * (-_distance_m);
        const G3 desired_offset = behind + G3{0, 1, 0} * (_distance_m * height_per_distance);
        look_at_pos = target_pos + G3{0, 1, 0} * look_height_m;
        if (_from_absolute) {
            _smoothed_offset = _smoothed_pos - target_pos;
            _from_absolute = false;
        }
        if (!_initialized || glen(_smoothed_offset - desired_offset) > SNAP_M) {
            _smoothed_offset = desired_offset;
            _initialized = true;
        }
        _smoothed_offset = glerp(_smoothed_offset, desired_offset, 1.0 - std::exp(-position_lag * delta));
        _smoothed_pos = target_pos + _smoothed_offset;
        double target_yaw = 0.0;
        if (live) target_yaw = target_orbit_yaw(lx, ly, sx, sy);
        if (orbit_lag <= 0.0) _orbit_yaw = target_yaw;
        else _orbit_yaw = glerp_angle(_orbit_yaw, target_yaw, 1.0 - std::exp(-orbit_lag * delta));
        position = target_pos + grotated_up(_smoothed_offset, -_orbit_yaw);
        for (const auto& g : grounds)
            if (g) position.y = std::max(position.y, *g + terrain_clearance_m);
        looked = glen(position - look_at_pos) > 0.01;
    }
};

ps::Vec3 to_core(G3 g) { return {g.x, -g.z, g.y}; }
G3 to_godot(ps::Vec3 c) { return {c.x, c.z, -c.y}; }

} // namespace

TEST_CASE("drone follow matches the former GDScript rig", "[camera_math][drone][golden]") {
    GdDrone ref;
    rg::DroneParams p; // defaults == the script's exports
    rg::DroneState st = rg::drone_initial_state(p);
    CHECK_THAT(st.distance_m, WithinAbs(ref._distance_m, 0.0));

    const auto probes = rg::drone_probe_offsets();
    REQUIRE(probes.size() == 5);
    // The script's order: ZERO, (0.3,0,0), (-0.3,0,0), (0,0,0.3), (0,0,-0.3) in Godot axes.
    const G3 gd_probes[5] = {{0, 0, 0}, {0.3, 0, 0}, {-0.3, 0, 0}, {0, 0, 0.3}, {0, 0, -0.3}};
    for (int i = 0; i < 5; ++i) {
        const G3 g = to_godot(probes[static_cast<size_t>(i)]);
        CHECK_THAT(g.x, WithinAbs(gd_probes[i].x, 1e-12));
        CHECK_THAT(g.y, WithinAbs(gd_probes[i].y, 1e-12));
        CHECK_THAT(g.z, WithinAbs(gd_probes[i].z, 1e-12));
    }

    ref.activate({3, 8, -20});
    rg::drone_activate(st, to_core({3, 8, -20}));

    const double dt = 1.0 / 60.0;
    double worst = 0.0;
    G3 car{0, 0.5, 0};
    double car_yaw = 0.0;
    for (int i = 0; i < 1500; ++i) {
        const double t = i * dt;
        // Car: accelerating drive with turns, a teleport at i == 600 (relocation, > SNAP_M) and a near-vertical
        // forward axis (heading hold) around i == 900.
        car_yaw += 0.4 * std::sin(t * 0.7) * dt;
        const double speed = 5.0 + 25.0 * (0.5 + 0.5 * std::sin(t * 0.2));
        car.x += std::cos(car_yaw) * speed * dt;
        car.z += -std::sin(car_yaw) * speed * dt;
        car.y = 0.5 + 0.3 * std::sin(t);
        if (i == 600) car = car + G3{400, 0, -250};
        G3 fwd{std::cos(car_yaw), 0.1 * std::sin(t), -std::sin(car_yaw)};
        if (i >= 900 && i < 930) fwd = {0.05, 1.0, 0.02}; // horizontal part < 0.2: previous heading is kept
        // Inputs: wheel/key zoom bursts, arrow-key looks, stick sweeps, camera inputs off for a stretch.
        const bool live = !(i >= 300 && i < 360);
        const double wheel = (i % 97 == 0) ? 2.0 : (i % 131 == 0 ? -3.0 : 0.0);
        const double zkey = (i >= 700 && i < 760) ? 1.0 : ((i >= 760 && i < 800) ? -1.0 : 0.0);
        double lx = 0, ly = 0, sx = 0, sy = 0;
        if (i >= 100 && i < 160) lx = 1.0;
        if (i >= 160 && i < 200) lx = -1.0;
        if (i >= 200 && i < 240) ly = -1.0;
        if (i >= 400 && i < 700) {
            sx = 0.9 * std::sin(t);
            sy = 0.9 * std::cos(t);
        }
        if (i >= 1000 && i < 1010) { sx = 0.1; sy = 0.1; } // inside the dead zone
        if (i == 1100) { // floating-origin rebase
            ref.shift_origin({-100, 0, 40});
            rg::DroneState& s = st;
            rg::drone_shift_origin(s, to_core({-100, 0, 40}));
            car = car + G3{-100, 0, 40};
        }
        if (i == 1200) { // activate again from the current camera (a rig switch)
            ref.activate(ref.position);
            rg::drone_activate(st, to_core(ref.position));
        }
        std::vector<std::optional<double>> grounds(5);
        for (int k = 0; k < 5; ++k)
            if ((i + k) % 3 != 0) grounds[static_cast<size_t>(k)] = 4.0 + 6.0 * std::sin(0.01 * i + k); // sometimes above the camera

        ref.update_zoom(dt, wheel, zkey, live);
        ref.update_rig(dt, car, fwd, live, lx, ly, sx, sy, grounds);

        rg::drone_zoom_step(st, wheel, zkey, live, dt, p);
        rg::DroneFollowInput in;
        in.target_pos = to_core(car);
        in.target_forward = to_core(fwd);
        in.key_x = lx; in.key_y = ly; in.stick_x = sx; in.stick_y = sy;
        in.live = live;
        const rg::DroneFrame f = rg::drone_follow_step(st, in, dt, p);
        const double z = rg::drone_clear_terrain(f.position.z, grounds, p.terrain_clearance_m);
        const G3 got = to_godot({f.position.x, f.position.y, z});
        worst = std::max({worst, std::fabs(got.x - ref.position.x), std::fabs(got.y - ref.position.y),
                          std::fabs(got.z - ref.position.z)});
        const G3 la = to_godot(f.look_at);
        worst = std::max({worst, std::fabs(la.x - ref.look_at_pos.x), std::fabs(la.y - ref.look_at_pos.y),
                          std::fabs(la.z - ref.look_at_pos.z), std::fabs(st.distance_m - ref._distance_m),
                          std::fabs(st.orbit_yaw_rad - ref._orbit_yaw)});
        CHECK(f.has_look == ref.looked);
        // The binding stores the position the rig actually sets (after clearance) the same way.
        ref.position = ref.position; // clearance already applied in the reference
    }
    CHECK(worst < 1e-9);
}

TEST_CASE("drone follow: zoom law, snap, orbit target and terrain clearance", "[camera_math][drone]") {
    const rg::DroneParams p;
    rg::DroneState s = rg::drone_initial_state(p);
    CHECK_THAT(s.distance_m, WithinAbs(12.0, 1e-12));
    // The first zoom call after activate only drains.
    rg::drone_activate(s, {0, 0, 5});
    rg::drone_zoom_step(s, 5.0, 0.0, true, 0.016, p);
    CHECK_THAT(s.distance_m, WithinAbs(12.0, 1e-12));
    rg::drone_zoom_step(s, 1.0, 0.0, true, 0.016, p); // one tick in: distance / 1.15
    CHECK_THAT(s.distance_m, WithinAbs(12.0 / 1.15, 1e-9));
    rg::drone_zoom_step(s, 100.0, 0.0, true, 0.016, p);
    CHECK_THAT(s.distance_m, WithinAbs(p.min_distance_m, 1e-12));
    rg::drone_zoom_step(s, -100.0, 0.0, true, 0.016, p);
    CHECK_THAT(s.distance_m, WithinAbs(p.max_distance_m, 1e-12));
    const double d = s.distance_m;
    rg::drone_zoom_step(s, 3.0, 1.0, false, 0.016, p); // inputs off: no zoom
    CHECK(s.distance_m == d);

    CHECK_THAT(rg::drone_target_orbit_yaw(1, 0, 0, 0, 0.2), WithinAbs(kPi / 2, 1e-12));
    CHECK_THAT(rg::drone_target_orbit_yaw(-1, 0, 0, 0, 0.2), WithinAbs(-kPi / 2, 1e-12));
    CHECK_THAT(rg::drone_target_orbit_yaw(0, -1, 0, 0, 0.2), WithinAbs(kPi, 1e-12));
    CHECK(rg::drone_target_orbit_yaw(0, 0, 0.1, 0.1, 0.2) == 0.0); // dead zone
    CHECK_THAT(rg::drone_target_orbit_yaw(0, 0, 1, 0, 0.2), WithinAbs(kPi / 2, 1e-12));

    // Constant speed: zero lag (the camera stays at the same offset behind a moving target).
    rg::DroneState t = rg::drone_initial_state(p);
    rg::DroneFollowInput in;
    in.target_forward = {1, 0, 0};
    in.live = false;
    rg::drone_hold(t);
    ps::Vec3 first{}, last{};
    for (int i = 0; i < 300; ++i) {
        in.target_pos = {30.0 * i / 60.0, 0, 0};
        const auto f = rg::drone_follow_step(t, in, 1.0 / 60.0, p);
        if (i == 0) first = {f.position.x - in.target_pos.x, f.position.y, f.position.z};
        last = {f.position.x - in.target_pos.x, f.position.y, f.position.z};
    }
    CHECK_THAT(last.x, WithinAbs(first.x, 1e-9));
    CHECK_THAT(first.x, WithinAbs(-12.0, 1e-9));
    CHECK_THAT(first.z, WithinAbs(6.0, 1e-9));
    // A far jump snaps to the wanted offset at once.
    in.target_pos = {5000, 0, 0};
    const auto f = rg::drone_follow_step(t, in, 1.0 / 60.0, p);
    CHECK_THAT(f.position.x, WithinAbs(5000.0 - 12.0, 1e-9));

    CHECK(rg::drone_clear_terrain(2.0, {std::nullopt, 5.0, 3.0}, 1.0) == 6.0);
    CHECK(rg::drone_clear_terrain(10.0, {1.0, std::nullopt}, 1.0) == 10.0);
    CHECK(rg::drone_clear_terrain(2.0, {}, 1.0) == 2.0);
}

// ------------------------------------------------------- chase/free/walker/orbit rigs ----
// Golden comparisons against the former GDScript rigs (chase_rig.gd, free_rig.gd, walker_rig.gd, orbit_rig.gd),
// transcribed line by line in Godot's Y-up frame with the helpers of the drone section (docs/unreal_port.md leak #10).
namespace {

G3 gcross(G3 a, G3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
G3 gnorm(G3 a) {
    const double l = glen(a);
    return l > 0.0 ? a * (1.0 / l) : a;
}
double gclamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }
// atan2/asin as the scripts call them, on Godot-frame values.
const G3 kUp{0, 1, 0};

// A chassis orientation in Godot axes: basis.x = forward, basis.y = left (what frame_convert.h hands out).
struct Chassis {
    G3 x, y;
};
Chassis chassis_pose(double yaw, double pitch, double roll) {
    const G3 f{std::cos(pitch) * std::cos(yaw), std::sin(pitch), -std::cos(pitch) * std::sin(yaw)};
    const G3 l0 = gnorm(gcross(kUp, f));
    const G3 l = l0 * std::cos(roll) + gcross(f, l0) * std::sin(roll);
    return {f, l};
}

double target_orbit_yaw_gd(double lx, double ly, double sx, double sy, double deadzone) {
    if (std::fabs(lx) > 0.5) return gsignf(lx) * 3.141592653589793 * 0.5;
    if (ly < -0.5) return 3.141592653589793;
    const double mag = std::min(std::sqrt(sx * sx + sy * sy), 1.0);
    if (mag <= deadzone) return 0.0;
    return std::atan2(sx, sy);
}

// ---- chase_rig.gd
struct GdChase {
    double follow_distance_m = 7.0, follow_height_m = 2.5, look_height_m = 1.0, side_offset_m = 0.0;
    double terrain_clearance_m = 0.6, position_lag = 6.0, orbit_deadzone = 0.2, orbit_lag = 14.0;
    const double SNAP_M = 60.0;
    G3 _smoothed_offset;
    bool _initialized = false;
    double _orbit_yaw = 0.0;
    G3 position, look_at_pos;
    bool looked = false;

    void update_rig(double delta, G3 chassis_pos, const Chassis& c, bool live, double lx, double ly, double sx, double sy,
                    const std::vector<std::optional<double>>& grounds) {
        const G3 behind = gnorm(c.x) * (-follow_distance_m);
        const G3 side = gnorm(c.y) * side_offset_m;
        const G3 desired_offset = behind + side + kUp * follow_height_m;
        look_at_pos = chassis_pos + kUp * look_height_m;
        if (!_initialized || glen(_smoothed_offset - desired_offset) > SNAP_M) {
            _smoothed_offset = desired_offset;
            _initialized = true;
        }
        _smoothed_offset = glerp(_smoothed_offset, desired_offset, 1.0 - std::exp(-position_lag * delta));
        double target_yaw = 0.0;
        if (live) target_yaw = target_orbit_yaw_gd(lx, ly, sx, sy, orbit_deadzone);
        if (orbit_lag <= 0.0) _orbit_yaw = target_yaw;
        else _orbit_yaw = glerp_angle(_orbit_yaw, target_yaw, 1.0 - std::exp(-orbit_lag * delta));
        position = chassis_pos + grotated_up(_smoothed_offset, -_orbit_yaw);
        for (const auto& g : grounds)
            if (g) position.y = std::max(position.y, *g + terrain_clearance_m);
        looked = glen(position - look_at_pos) > 0.01;
    }
};

TEST_CASE("chase rig matches the former GDScript rig", "[camera_math][chase][golden]") {
    for (const double side : {0.0, 1.4}) {
        for (const double orbit_lag : {14.0, 0.0}) {
            GdChase ref;
            ref.side_offset_m = side;
            ref.orbit_lag = orbit_lag;
            rg::ChaseParams p;
            p.side_offset_m = side;
            p.orbit_lag = orbit_lag;
            rg::ChaseState st;
            const double dt = 1.0 / 60.0;
            double worst = 0.0;
            G3 car{0, 0.5, 0};
            double yaw = 0.0;
            for (int i = 0; i < 1500; ++i) {
                const double t = i * dt;
                yaw += 0.5 * std::sin(t * 0.6) * dt;
                const double speed = 3.0 + 40.0 * (0.5 + 0.5 * std::sin(t * 0.25));
                car.x += std::cos(yaw) * speed * dt;
                car.z += -std::sin(yaw) * speed * dt;
                car.y = 0.5 + 0.4 * std::sin(t * 1.3);
                if (i == 700) car = car + G3{500, 0, 200}; // relocation: the offset jump is not the car's (snap on the lag)
                const Chassis c = chassis_pose(yaw, 0.15 * std::sin(t), 0.1 * std::sin(t * 0.7));
                const bool live = !(i >= 400 && i < 460);
                double lx = 0, ly = 0, sx = 0, sy = 0;
                if (i >= 100 && i < 160) lx = 1.0;
                if (i >= 160 && i < 200) lx = -1.0;
                if (i >= 200 && i < 240) ly = -1.0;
                if (i >= 500 && i < 900) {
                    sx = 0.9 * std::sin(t);
                    sy = 0.9 * std::cos(t);
                }
                if (i >= 1000 && i < 1010) { sx = 0.1; sy = 0.1; }
                if (i == 1100) { // a changed follow distance beyond the snap distance
                    ref.follow_distance_m = 90.0;
                    p.follow_distance_m = 90.0;
                }
                if (i == 1200) { // a rig switch re-seats, a frame without a simulation too
                    ref._initialized = false;
                    rg::chase_reset(st);
                }
                std::vector<std::optional<double>> grounds(5);
                for (int k = 0; k < 5; ++k)
                    if ((i + k) % 4 != 0) grounds[static_cast<size_t>(k)] = 2.0 + 8.0 * std::sin(0.01 * i + k);

                ref.update_rig(dt, car, c, live, lx, ly, sx, sy, grounds);

                rg::ChaseInput in;
                in.target_pos = to_core(car);
                in.forward = to_core(c.x);
                in.left = to_core(c.y);
                in.key_x = lx; in.key_y = ly; in.stick_x = sx; in.stick_y = sy;
                in.live = live;
                const rg::ChaseFrame f = rg::chase_follow_step(st, in, dt, p);
                const double z = rg::clear_terrain_height(f.position.z, grounds, p.terrain_clearance_m);
                const G3 got = to_godot({f.position.x, f.position.y, z});
                const G3 la = to_godot(f.look_at);
                worst = std::max({worst, std::fabs(got.x - ref.position.x), std::fabs(got.y - ref.position.y),
                                  std::fabs(got.z - ref.position.z), std::fabs(la.x - ref.look_at_pos.x),
                                  std::fabs(la.y - ref.look_at_pos.y), std::fabs(la.z - ref.look_at_pos.z),
                                  std::fabs(st.orbit_yaw_rad - ref._orbit_yaw)});
                CHECK(rg::look_valid({got.x, -got.z, got.y}, f.look_at) == ref.looked);
            }
            CHECK(worst < 1e-9);
        }
    }
}

TEST_CASE("chase rig: constant speed keeps its offset, the first step sits behind the car", "[camera_math][chase]") {
    const rg::ChaseParams p;
    rg::ChaseState s;
    rg::ChaseInput in;
    in.forward = {1, 0, 0};
    in.left = {0, 1, 0};
    in.live = false;
    const rg::ChaseFrame first = rg::chase_follow_step(s, in, 0.016, p);
    CHECK_THAT(first.position.x, WithinAbs(-7.0, 1e-12));
    CHECK_THAT(first.position.z, WithinAbs(2.5, 1e-12));
    CHECK_THAT(first.look_at.z, WithinAbs(1.0, 1e-12));
    rg::ChaseFrame last = first;
    for (int i = 1; i < 300; ++i) {
        in.target_pos = {30.0 * i / 60.0, 0, 0};
        last = rg::chase_follow_step(s, in, 1.0 / 60.0, p);
    }
    CHECK_THAT(last.position.x - in.target_pos.x, WithinAbs(-7.0, 1e-9)); // zero lag at any constant speed
    // reset re-seats (keeps the orbit yaw)
    s.orbit_yaw_rad = 1.0;
    rg::chase_reset(s);
    CHECK_FALSE(s.initialized);
    CHECK(s.orbit_yaw_rad == 1.0);
}

// ---- free_rig.gd
struct GdFree {
    double move_speed_mps = 40.0, fast_multiplier = 6.0, look_rate_rad_s = 1.8, mouse_sensitivity = 0.0025, pitch_limit_rad = 1.5;
    G3 position;
    double _yaw = 0.0, _pitch = 0.0;
    void activate(G3 origin, G3 fwd) { // fwd = -from.basis.z
        position = origin;
        _yaw = std::atan2(-fwd.x, -fwd.z);
        _pitch = gclamp(std::asin(gclamp(fwd.y, -1.0, 1.0)), -pitch_limit_rad, pitch_limit_rad);
    }
    void place(G3 pos, double yaw, double pitch) {
        position = pos;
        _yaw = yaw;
        _pitch = gclamp(pitch, -pitch_limit_rad, pitch_limit_rad);
    }
    void update_rig(double delta, bool camera_input_live, double mx, double my, double lx, double ly, G3 m, bool fast) {
        if (!camera_input_live) return;
        _yaw -= mx * mouse_sensitivity + lx * look_rate_rad_s * delta;
        _pitch -= my * mouse_sensitivity;
        _pitch += ly * look_rate_rad_s * delta;
        _pitch = gclamp(_pitch, -pitch_limit_rad, pitch_limit_rad);
        G3 move = grotated_up(G3{m.x, 0.0, -m.z}, _yaw) + kUp * m.y;
        if (glen(move) > 1.0) move = gnorm(move);
        const double speed = move_speed_mps * (fast ? fast_multiplier : 1.0);
        position = position + move * speed * delta;
    }
};

TEST_CASE("free rig matches the former GDScript rig", "[camera_math][free][golden]") {
    GdFree ref;
    rg::FreeParams p;
    rg::FreeState st;
    const double dt = 1.0 / 60.0;
    double worst = 0.0;
    const G3 start{10, 5, -30};
    const G3 look{0.3, 0.2, -0.9};
    const G3 fwd = gnorm(look);
    ref.activate(start, fwd);
    rg::free_activate(st, to_core(start), to_core(fwd), p);
    for (int i = 0; i < 1800; ++i) {
        const double t = i * dt;
        if (i == 900) { // place (screenshot tools), clamped pitch
            ref.place({-40, 12, 7}, 2.2, 3.0);
            rg::free_place(st, to_core({-40, 12, 7}), 2.2, 3.0, p);
        }
        if (i == 1200) { // floating-origin rebase
            ref.position = ref.position + G3{-100, 0, 40};
            rg::free_shift_origin(st, to_core({-100, 0, 40}));
        }
        if (i == 1500) { // activate from a camera looking straight up (pitch limit, heading from the horizontal part)
            const G3 up_look = gnorm({0.01, 1.0, -0.02});
            ref.activate({1, 2, 3}, up_look);
            rg::free_activate(st, to_core({1, 2, 3}), to_core(up_look), p);
        }
        const bool live = !(i >= 300 && i < 360);
        const double mx = (i % 50 == 0) ? 40.0 : 0.0, my = (i % 70 == 0) ? -25.0 : 0.0;
        const double lx = (i >= 100 && i < 250) ? 0.8 : 0.0, ly = (i >= 400 && i < 700) ? 1.0 : (i >= 700 && i < 800 ? -1.0 : 0.0);
        const G3 m{std::sin(t), 0.5 * std::sin(t * 0.7), std::cos(t * 1.3) * 1.2}; // z up to 1.2: a long vector normalises
        const bool fast = (i % 400) < 100;
        ref.update_rig(dt, live, mx, my, lx, ly, m, fast);
        rg::FreeInput in;
        in.mouse_x = mx; in.mouse_y = my; in.look_x = lx; in.look_y = ly;
        in.move_right = m.x; in.move_up = m.y; in.move_forward = m.z;
        in.fast = fast; in.live = live;
        rg::free_step(st, in, dt, p);
        const G3 got = to_godot(st.position);
        worst = std::max({worst, std::fabs(got.x - ref.position.x), std::fabs(got.y - ref.position.y),
                          std::fabs(got.z - ref.position.z), std::fabs(st.yaw_rad - ref._yaw),
                          std::fabs(st.pitch_rad - ref._pitch)});
    }
    CHECK(worst < 1e-9);
}

// ---- walker_rig.gd
struct GdWalker {
    double look_rate_rad_s = 2.2, mouse_sensitivity = 0.0025, third_person_distance_m = 3.2, min_distance_m = 1.2,
           max_distance_m = 9.0, head_height_m = 1.55, eye_height_m = 1.62, terrain_clearance_m = 0.3, follow_rate = 28.0,
           snap_m = 12.0, third_pitch_min = -1.1, third_pitch_max = 1.2, first_pitch_limit = 1.5;
    bool first_person = false;
    double _yaw = 0.0, _pitch = -0.15, _distance_m = 3.2;
    G3 _smoothed_feet, position;
    double root_yaw = 0.0, cam_pitch = 0.0;
    bool _have_walker = false, _drain_wheel = false;

    GdWalker() { _distance_m = gclamp(third_person_distance_m, min_distance_m, max_distance_m); }
    void activate(G3 from_origin, G3 fwd) {
        position = from_origin;
        _have_walker = false;
        _drain_wheel = true;
        _yaw = std::atan2(-fwd.x, -fwd.z);
        _pitch = -0.15;
        root_yaw = _yaw;
        cam_pitch = _pitch;
    }
    void shift_origin(G3 d) {
        position = position + d;
        _smoothed_feet = _smoothed_feet + d;
    }
    G3 get_look_forward() const { return grotated_up(G3{0, 0, -1}, _yaw); }
    void apply_rotation() {
        root_yaw = _yaw;
        cam_pitch = _pitch;
    }
    void update_look(double delta, double mx, double my, double wheel, double lx, double ly, double key, bool live) {
        if (_drain_wheel) {
            _drain_wheel = false;
            wheel = 0.0;
        }
        if (!live) return;
        _yaw -= mx * mouse_sensitivity + lx * look_rate_rad_s * delta;
        _pitch -= my * mouse_sensitivity;
        _pitch += ly * look_rate_rad_s * delta;
        if (first_person) {
            _pitch = gclamp(_pitch, -first_pitch_limit, first_pitch_limit);
        } else {
            _pitch = gclamp(_pitch, third_pitch_min, third_pitch_max);
            _distance_m *= std::pow(1.15, -wheel) * std::exp(-key * 1.2 * delta);
            _distance_m = gclamp(_distance_m, min_distance_m, max_distance_m);
        }
    }
    // has_walker false: ws.is_empty(). grounds: one optional per probe.
    void update_rig(double delta, bool has_walker, G3 feet, G3 car_position, double mx, double my, double wheel, double lx,
                    double ly, double key, bool live, const std::vector<std::optional<double>>& grounds) {
        if (!has_walker) {
            update_look(delta, mx, my, wheel, lx, ly, key, live);
            apply_rotation();
            return;
        }
        if (!_have_walker) {
            _have_walker = true;
            _smoothed_feet = feet;
            G3 to_car = car_position - feet;
            to_car.y = 0.0;
            if (glen(to_car) > 0.1) _yaw = std::atan2(-to_car.x, -to_car.z);
            _pitch = -0.15;
        } else if (glen(_smoothed_feet - feet) > snap_m) {
            _smoothed_feet = feet;
        } else {
            _smoothed_feet = glerp(_smoothed_feet, feet, 1.0 - std::exp(-follow_rate * delta));
        }
        update_look(delta, mx, my, wheel, lx, ly, key, live);
        if (first_person) {
            position = _smoothed_feet + kUp * eye_height_m;
            apply_rotation();
            return;
        }
        const G3 pivot = _smoothed_feet + kUp * head_height_m;
        // Basis(UP, yaw) * (Basis(RIGHT, pitch) * (0, 0, -1)); Basis(RIGHT, p) * (x, y, z) = (x, y c - z s, y s + z c)
        const G3 pitched{0.0, 0.0 * std::cos(_pitch) - (-1.0) * std::sin(_pitch), 0.0 * std::sin(_pitch) + (-1.0) * std::cos(_pitch)};
        const G3 look_dir = grotated_up(pitched, _yaw);
        G3 pos = pivot - look_dir * _distance_m;
        for (const auto& g : grounds)
            if (g) pos.y = std::max(pos.y, *g + terrain_clearance_m);
        position = pos;
        const G3 to_pivot = pivot - pos;
        const double flat = std::sqrt(to_pivot.x * to_pivot.x + to_pivot.z * to_pivot.z);
        if (flat > 0.01) {
            root_yaw = std::atan2(-to_pivot.x, -to_pivot.z);
            cam_pitch = gclamp(std::atan2(to_pivot.y, flat), third_pitch_min, third_pitch_max);
        } else {
            apply_rotation();
        }
    }
};

TEST_CASE("walker camera matches the former GDScript rig", "[camera_math][walker_cam][golden]") {
    GdWalker ref;
    rg::WalkerCamParams p;
    rg::WalkerCamState st = rg::walker_cam_initial_state(p);
    CHECK_THAT(st.distance_m, WithinAbs(ref._distance_m, 0.0));
    const double dt = 1.0 / 60.0;
    double worst = 0.0;
    G3 car{20, 0.5, -10};
    G3 feet{21.5, 0.2, -9};
    const G3 from_fwd = gnorm({0.6, 0.0, -0.8});
    ref.activate({0, 3, 0}, from_fwd);
    rg::walker_cam_activate(st, to_core(from_fwd), p);
    for (int i = 0; i < 2400; ++i) {
        const double t = i * dt;
        // The walker spawns after 20 frames, runs, jumps (feet height), teleports at 900 (snap), despawns 1500..1560.
        const bool has = !(i < 20 || (i >= 1500 && i < 1560));
        feet.x += 3.0 * std::cos(0.3 * t) * dt;
        feet.z += 3.0 * std::sin(0.3 * t) * dt;
        feet.y = 0.2 + std::max(0.0, std::sin(t * 2.0)) * 0.8;
        if (i == 900) feet = feet + G3{60, 0, -40};
        if (i == 1100) { // floating-origin rebase
            ref.shift_origin({-50, 0, 10});
            rg::walker_cam_shift_origin(st, to_core({-50, 0, 10}));
            feet = feet + G3{-50, 0, 10};
            car = car + G3{-50, 0, 10};
        }
        if (i == 1300) ref.first_person = true;
        if (i == 1700) ref.first_person = false;
        if (i == 1900) { // a rig switch again, from a new camera heading
            const G3 f2 = gnorm({-0.7, 0.2, 0.5});
            ref.activate(ref.position, f2);
            rg::walker_cam_activate(st, to_core(f2), p);
        }
        const bool live = !(i >= 300 && i < 360);
        const double mx = (i % 60 == 0) ? 30.0 : 0.0, my = (i % 90 == 0) ? 20.0 : 0.0;
        const double wheel = (i % 53 == 0) ? 2.0 : (i % 71 == 0 ? -4.0 : 0.0);
        const double lx = (i >= 100 && i < 250) ? 0.7 : 0.0, ly = (i >= 400 && i < 900) ? 1.0 : (i >= 900 && i < 1000 ? -1.0 : 0.0);
        const double key = (i >= 600 && i < 650) ? 1.0 : ((i >= 650 && i < 700) ? -1.0 : 0.0);
        std::vector<std::optional<double>> grounds(5);
        for (int k = 0; k < 5; ++k)
            if ((i + k) % 3 != 0) grounds[static_cast<size_t>(k)] = -1.0 + 3.0 * std::sin(0.02 * i + k);

        ref.update_rig(dt, has, feet, car, mx, my, wheel, lx, ly, key, live, grounds);

        rg::WalkerCamInput in;
        in.has_walker = has;
        in.feet = to_core(feet);
        in.car_position = to_core(car);
        in.mouse_x = mx; in.mouse_y = my; in.wheel_steps = wheel; in.look_x = lx; in.look_y = ly; in.zoom_key = key;
        in.live = live;
        in.first_person = ref.first_person;
        const rg::WalkerCamFrame f = rg::walker_cam_step(st, in, dt, p);
        worst = std::max({worst, std::fabs(st.yaw_rad - ref._yaw), std::fabs(st.pitch_rad - ref._pitch),
                          std::fabs(st.distance_m - ref._distance_m)});
        const G3 lf = to_godot(rg::walker_cam_look_forward(st));
        const G3 rlf = ref.get_look_forward();
        worst = std::max({worst, std::fabs(lf.x - rlf.x), std::fabs(lf.y - rlf.y), std::fabs(lf.z - rlf.z)});
        double yaw = f.yaw_rad, pitch = f.pitch_rad;
        if (f.placed) {
            ps::Vec3 pos = f.position;
            if (f.third_person) {
                pos.z = rg::clear_terrain_height(pos.z, grounds, p.terrain_clearance_m);
                const rg::WalkerCamAim aim = rg::walker_cam_aim(f.pivot, pos, p);
                if (aim.valid) {
                    yaw = aim.yaw_rad;
                    pitch = aim.pitch_rad;
                }
            }
            const G3 got = to_godot(pos);
            worst = std::max({worst, std::fabs(got.x - ref.position.x), std::fabs(got.y - ref.position.y),
                              std::fabs(got.z - ref.position.z)});
        }
        worst = std::max({worst, std::fabs(yaw - ref.root_yaw), std::fabs(pitch - ref.cam_pitch)});
    }
    CHECK(worst < 1e-9);
}

TEST_CASE("walker camera: first sight, wheel drain, zoom clamp, aim", "[camera_math][walker_cam]") {
    const rg::WalkerCamParams p;
    rg::WalkerCamState s = rg::walker_cam_initial_state(p);
    CHECK_THAT(s.distance_m, WithinAbs(3.2, 1e-12));
    CHECK_THAT(s.pitch_rad, WithinAbs(-0.15, 1e-12));
    rg::walker_cam_activate(s, {0, 1, 0}, p); // looking along +y (Godot -Z): yaw 0
    CHECK_THAT(s.yaw_rad, WithinAbs(0.0, 1e-12));
    rg::WalkerCamInput in;
    in.wheel_steps = 5.0;
    // no walker: the look stays responsive, nothing is placed, the first wheel call is drained
    rg::WalkerCamFrame f = rg::walker_cam_step(s, in, 0.016, p);
    CHECK_FALSE(f.placed);
    CHECK_THAT(s.distance_m, WithinAbs(3.2, 1e-12));
    in.wheel_steps = 100.0;
    rg::walker_cam_step(s, in, 0.016, p);
    CHECK_THAT(s.distance_m, WithinAbs(p.min_distance_m, 1e-12));
    in.wheel_steps = -100.0;
    rg::walker_cam_step(s, in, 0.016, p);
    CHECK_THAT(s.distance_m, WithinAbs(p.max_distance_m, 1e-12));
    // first sight: look at the car (here on +x -> yaw -pi/2: turning right), pitch reset
    in = {};
    in.has_walker = true;
    in.feet = {0, 0, 0};
    in.car_position = {5, 0, 0};
    f = rg::walker_cam_step(s, in, 0.016, p);
    CHECK(f.placed);
    CHECK(f.third_person);
    CHECK_THAT(s.yaw_rad, WithinAbs(-kPi / 2, 1e-12));
    const ps::Vec3 fw = rg::walker_cam_look_forward(s);
    CHECK_THAT(fw.x, WithinAbs(1.0, 1e-12));
    CHECK_THAT(fw.y, WithinAbs(0.0, 1e-12));
    // the camera stands behind the pivot, along the look direction
    CHECK(f.position.x < f.pivot.x);
    CHECK_THAT(f.pivot.z, WithinAbs(1.55, 1e-12));
    // a jump beyond the snap distance snaps the pivot; first person sits at the eye height
    in.feet = {100, 0, 0};
    in.first_person = true;
    f = rg::walker_cam_step(s, in, 0.016, p);
    CHECK_THAT(f.position.x, WithinAbs(100.0, 1e-12));
    CHECK_THAT(f.position.z, WithinAbs(1.62, 1e-12));
    CHECK_FALSE(f.third_person);
    // inputs off: the look does not change
    const double yaw = s.yaw_rad;
    in.live = false;
    in.look_x = 1.0;
    rg::walker_cam_step(s, in, 0.016, p);
    CHECK(s.yaw_rad == yaw);
    // aim: straight above/below is no aim; otherwise toward the pivot with the pitch clamped
    CHECK_FALSE(rg::walker_cam_aim({0, 0, 5}, {0.001, 0, 0}, p).valid);
    const rg::WalkerCamAim a = rg::walker_cam_aim({0, 0, 5}, {0, -1, 0}, p); // pivot ahead (+y) and 5 m up
    CHECK(a.valid);
    CHECK_THAT(a.yaw_rad, WithinAbs(0.0, 1e-12));
    CHECK_THAT(a.pitch_rad, WithinAbs(p.third_pitch_max, 1e-12));
}

// ---- orbit_rig.gd (orbit_step itself is the core function the script already called)
struct GdOrbitRig {
    static constexpr double SNAP_M = 60.0, FOLLOW_LAG = 8.0, CLEARANCE_M = 0.6, MOUSE_LOOK_PER_PX = 0.05;
    rg::OrbitState state;
    G3 _target, _target_rel{0, 1, 0}, position;
    bool _initialized = false, looked = false;
    GdOrbitRig() {
        const rg::OrbitParams op;
        state.elevation_rad = op.default_elevation_rad;
        state.distance_m = op.default_distance_m;
    }
    void activate() {
        *this = GdOrbitRig();
    }
    void update_rig(double delta, G3 chassis_origin, G3 basis_x, double lx, double ly, double mx, double my, double zs,
                    double zk, bool input_live, std::optional<double> ground) {
        const G3 focus_rel = kUp * 1.0;
        if (!_initialized || glen(_target_rel - focus_rel) > SNAP_M) {
            _target_rel = focus_rel;
            _initialized = true;
        }
        _target_rel = glerp(_target_rel, focus_rel, 1.0 - std::exp(-FOLLOW_LAG * delta));
        _target = chassis_origin + _target_rel;
        rg::OrbitInput input;
        input.live = input_live;
        if (input_live) {
            input.look_x = gclamp(lx + mx * MOUSE_LOOK_PER_PX, -1.0, 1.0);
            input.look_y = gclamp(ly + my * MOUSE_LOOK_PER_PX, -1.0, 1.0);
            input.zoom_steps = zs;
            input.zoom_key = zk;
        }
        state = rg::orbit_step(state, input, delta);
        const ps::Vec3 o = rg::orbit_offset(state);
        G3 forward{basis_x.x, 0.0, basis_x.z};
        forward = glen(forward) > 0.001 ? gnorm(forward) : G3{0.0, 0.0, -1.0};
        const G3 left = gcross(kUp, forward);
        position = _target + forward * o.x + left * o.y + kUp * o.z;
        if (ground) position.y = std::max(position.y, *ground + CLEARANCE_M);
        looked = glen(position - _target) > 0.01;
    }
};

TEST_CASE("orbit rig matches the former GDScript rig", "[camera_math][orbit_rig][golden]") {
    GdOrbitRig ref;
    rg::OrbitRigState st = rg::orbit_rig_initial_state();
    const rg::OrbitRigParams p;
    const double dt = 1.0 / 60.0;
    double worst = 0.0;
    G3 car{0, 0.5, 0};
    double yaw = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const double t = i * dt;
        yaw += 0.6 * std::sin(t * 0.5) * dt;
        car.x += std::cos(yaw) * 15.0 * dt;
        car.z += -std::sin(yaw) * 15.0 * dt;
        if (i == 800) car = car + G3{300, 0, 0};
        if (i == 1400) { // a rig switch
            ref.activate();
            rg::orbit_rig_activate(st);
        }
        if (i == 1700) { // a frame without a simulation
            ref._initialized = false;
            rg::orbit_rig_reset(st);
        }
        const Chassis c = chassis_pose(yaw, 0.1 * std::sin(t), 0.0);
        const bool live = !(i >= 500 && i < 560);
        const double lx = (i >= 100 && i < 300) ? 0.8 : 0.0, ly = (i >= 300 && i < 400) ? -0.6 : 0.0;
        const double mx = (i % 40 == 0) ? 12.0 : 0.0, my = (i % 55 == 0) ? -30.0 : 0.0;
        const double zs = (i % 97 == 0) ? 2.0 : 0.0, zk = (i >= 1000 && i < 1100) ? 1.0 : 0.0;
        std::optional<double> ground;
        if (i % 3 != 0) ground = 1.0 + 4.0 * std::sin(0.02 * i);
        ref.update_rig(dt, car, c.x, lx, ly, mx, my, zs, zk, live, ground);

        rg::OrbitRigInput in;
        in.chassis_pos = to_core(car);
        in.forward = to_core(c.x);
        in.look_x = lx; in.look_y = ly; in.mouse_x = mx; in.mouse_y = my; in.zoom_steps = zs; in.zoom_key = zk;
        in.live = live;
        const rg::OrbitRigFrame f = rg::orbit_rig_step(st, in, dt, p);
        std::vector<std::optional<double>> grounds{ground};
        const double z = rg::clear_terrain_height(f.position.z, grounds, p.clearance_m);
        const G3 got = to_godot({f.position.x, f.position.y, z});
        const G3 tg = to_godot(f.target);
        worst = std::max({worst, std::fabs(got.x - ref.position.x), std::fabs(got.y - ref.position.y),
                          std::fabs(got.z - ref.position.z), std::fabs(tg.x - ref._target.x), std::fabs(tg.y - ref._target.y),
                          std::fabs(tg.z - ref._target.z), std::fabs(st.orbit.azimuth_rad - ref.state.azimuth_rad),
                          std::fabs(st.orbit.distance_m - ref.state.distance_m)});
        CHECK(rg::look_valid({got.x, -got.z, got.y}, f.target) == ref.looked);
    }
    CHECK(worst < 1e-9);
}

TEST_CASE("rig helpers: probes, clearance, look validity", "[camera_math][rig_helpers]") {
    const auto probes = rg::ground_probe_offsets();
    REQUIRE(probes.size() == 5);
    CHECK(probes[0].x == 0.0);
    const auto drone = rg::drone_probe_offsets();
    for (size_t i = 0; i < 5; ++i) CHECK(probes[i].y == drone[i].y);
    CHECK(rg::clear_terrain_height(2.0, {}, 1.0) == 2.0);
    CHECK(rg::clear_terrain_height(2.0, {std::nullopt, 5.0, 1.0}, 0.5) == 5.5);
    CHECK_FALSE(rg::look_valid({0, 0, 0}, {0.005, 0, 0}));
    CHECK(rg::look_valid({0, 0, 0}, {0.02, 0, 0}));
}

} // namespace
