// test_cinematic_obstacles.cpp - the cinematic camera must not stand inside buildings or film through them
// (rg/shot_obstacles.h, rg/building_footprints.h, CinematicDirector::set_obstacles). Each case names the sabotage that
// makes it fail (re-measured when the case was written).

#include "rg/building_footprints.h"
#include "rg/camera_math.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <mutex>
#include <vector>

namespace {

using Ring = std::vector<std::pair<double, double>>;

// An L: the 20 x 10 base with a 10 x 10 arm up the left side; the notch is x in [10,20], y in [10,20].
Ring l_shape() { return {{0, 0}, {20, 0}, {20, 10}, {10, 10}, {10, 20}, {0, 20}}; }

struct Box {
    double x0, y0, x1, y1, z0, z1;
};

// Synthetic obstacles: axis aligned boxes, an optional terrain function, an optional "still loading" switch.
class FakeObstacles final : public rg::ShotObstacles {
public:
    std::vector<Box> boxes;
    std::function<double(double, double)> ground;
    std::atomic<bool> loaded{true};

    [[nodiscard]] std::optional<double> ground_height(double x, double y) const override {
        if (!ground) return std::nullopt;
        return ground(x, y);
    }
    [[nodiscard]] bool building_covers(double x, double y, double z, double margin) const override {
        for (const auto& b : boxes)
            if (z >= b.z0 && z <= b.z1 && x >= b.x0 - margin && x <= b.x1 + margin && y >= b.y0 - margin && y <= b.y1 + margin)
                return true;
        return false;
    }
    [[nodiscard]] bool building_blocks(double ax, double ay, double az, double bx, double by, double bz) const override {
        for (const auto& b : boxes) {
            double t0 = 0.0, t1 = 1.0;
            const double a[3] = {ax, ay, az}, d[3] = {bx - ax, by - ay, bz - az};
            const double lo[3] = {b.x0, b.y0, b.z0}, hi[3] = {b.x1, b.y1, b.z1};
            bool hit = true;
            for (int i = 0; i < 3 && hit; ++i) {
                if (std::fabs(d[i]) < 1.0e-12) {
                    hit = a[i] >= lo[i] && a[i] <= hi[i];
                } else {
                    double s0 = (lo[i] - a[i]) / d[i], s1 = (hi[i] - a[i]) / d[i];
                    if (s0 > s1) std::swap(s0, s1);
                    t0 = std::max(t0, s0);
                    t1 = std::min(t1, s1);
                    hit = t0 <= t1;
                }
            }
            if (hit) return true;
        }
        return false;
    }
    [[nodiscard]] bool ready(double, double) const override { return loaded.load(); }
};

// A straight road along +x with the car at the origin doing 20 m/s: the first shot sits ~70 m ahead, ~6 m to a side.
rg::CarKinematics car_at_origin() {
    rg::CarKinematics car;
    car.vx = 20.0;
    return car;
}

std::vector<rg::RoadPoint> straight_road(double x0 = 0.0) {
    std::vector<rg::RoadPoint> road;
    for (int i = 0; i <= 20; ++i) road.push_back({x0 + 10.0 * i, 0.0, 3.0});
    return road;
}

rg::CinematicShot first_shot(const rg::ShotObstacles* obstacles, std::uint64_t seed = 42) {
    rg::CinematicDirector director(seed);
    director.set_obstacles(obstacles);
    return director.update(1.0 / 60.0, car_at_origin(), straight_road());
}

Box around(const rg::CinematicShot& s, double half = 5.0) {
    return {s.x - half, s.y - half, s.x + half, s.y + half, -500.0, 500.0};
}

} // namespace

// ------------------------------------------------------------- geometry ----

TEST_CASE("footprints: point tests honour the outline, the margin and the vertical span", "[cinematic][obstacles]") {
    rg::BuildingFootprints f;
    f.add(l_shape(), 100.0, 130.0);
    CHECK(f.size() == 1);
    CHECK(f.near_point(5.0, 15.0, 110.0, 0.0));     // inside the arm
    CHECK(f.near_point(15.0, 5.0, 110.0, 0.0));     // inside the base
    CHECK_FALSE(f.near_point(15.0, 15.0, 110.0, 0.0)); // in the notch (concave: the bounding box would say yes)
    CHECK_FALSE(f.near_point(15.0, 15.0, 110.0, 4.0)); // 5 m from both notch walls
    CHECK(f.near_point(15.0, 15.0, 110.0, 5.5));       // within the margin of the notch walls
    CHECK(f.near_point(-0.8, 5.0, 110.0, 1.0));         // outside by 0.8 m, margin 1 m
    CHECK_FALSE(f.near_point(-1.5, 5.0, 110.0, 1.0));
    // Vertical span: the roof is at 130 m and the walls start at 100 m (an elevated block is open underneath).
    // Sabotage: ignoring z makes both of these true.
    CHECK_FALSE(f.near_point(5.0, 5.0, 131.0, 1.0));
    CHECK_FALSE(f.near_point(5.0, 5.0, 99.0, 1.0));
}

TEST_CASE("footprints: a segment is blocked only inside the outline and the span", "[cinematic][obstacles]") {
    rg::BuildingFootprints f;
    f.add(l_shape(), 0.0, 30.0);
    CHECK(f.blocks_segment(-10, 5, 2, 30, 5, 2));        // straight through the base
    CHECK_FALSE(f.blocks_segment(-10, 5, 40, 30, 5, 40)); // the same line above the roof (sabotage: no z test)
    CHECK_FALSE(f.blocks_segment(-10, 25, 2, 30, 25, 2)); // beside the building
    CHECK_FALSE(f.blocks_segment(12, 12, 2, 28, 12, 2));  // along the notch (concave: outside the outline)
    CHECK(f.blocks_segment(-5, 15, 2, 15, 15, 2));        // through the arm into the notch
    CHECK(f.blocks_segment(30, 5, 2, 15, 5, 2));          // ends inside the base
    CHECK(f.blocks_segment(15, 5, 20, 15, 5, -5));        // dips from above the roof into the base
    // A long segment crossing many grid cells still finds a small building in the middle.
    rg::BuildingFootprints g;
    g.add({{500, 100}, {504, 100}, {504, 104}, {500, 104}}, 0.0, 10.0);
    CHECK(g.blocks_segment(0, 102, 2, 1000, 102, 2));
    CHECK_FALSE(g.blocks_segment(0, 110, 2, 1000, 110, 2));
}

TEST_CASE("building obstacles: records become session-frame prisms, tiles report readiness", "[cinematic][obstacles]") {
    const double e0 = 1000000.0, n0 = 2000000.0;
    rg::BuildingObstacles obstacles(e0, n0, {}, {});
    rg::Building b;
    b.id = 1;
    b.base = 210.0;   // highest ground under the footprint
    b.bottom = 199.0; // lowest ground - 1
    b.height = 20.0;  // roof at 230
    b.outers.push_back({{e0 + 100, n0 + 50}, {e0 + 120, n0 + 50}, {e0 + 120, n0 + 70}, {e0 + 100, n0 + 70}});
    // Session x 100..120, y 50..70 lies in UTM tile (floor((x + e0) / 1024), floor((y + n0) / 1024)).
    const int tx = static_cast<int>(std::floor((110.0 + e0) / 1024.0)), ty = static_cast<int>(std::floor((60.0 + n0) / 1024.0));
    CHECK_FALSE(obstacles.ready(110.0, 60.0)); // nothing loaded yet
    obstacles.add_tile(tx, ty, {b});
    CHECK(obstacles.ready(110.0, 60.0));
    CHECK(obstacles.footprint_count() == 1);
    CHECK(obstacles.building_covers(110.0, 60.0, 215.0, 1.0));
    CHECK(obstacles.building_covers(98.5, 60.0, 215.0, 2.0));       // within the margin of the west wall
    CHECK_FALSE(obstacles.building_covers(110.0, 60.0, 232.0, 1.0)); // above the roof (absolute heights)
    CHECK_FALSE(obstacles.building_covers(110.0, 60.0, 190.0, 1.0)); // below the lowest ground
    CHECK(obstacles.building_blocks(90, 60, 212, 130, 60, 212));
    CHECK_FALSE(obstacles.building_blocks(90, 80, 212, 130, 80, 212));
    // An elevated block (min_height): bottom = base + min_height, open underneath.
    rg::Building bridge = b;
    bridge.bottom = bridge.base + 8.0;
    obstacles.add_tile(tx, ty, {bridge}); // replaces the tile
    CHECK_FALSE(obstacles.building_covers(110.0, 60.0, 212.0, 1.0));
    CHECK(obstacles.building_covers(110.0, 60.0, 222.0, 1.0));
}

TEST_CASE("building obstacles: a worker loads the 3 x 3 tiles around a request, nearest first, and evicts far ones", "[cinematic][obstacles]") {
    std::mutex mutex;
    std::vector<std::pair<int, int>> fetched;
    rg::BuildingObstacles obstacles(
        0.0, 0.0,
        [&](int tx, int ty) {
            std::lock_guard<std::mutex> lock(mutex);
            fetched.emplace_back(tx, ty);
            return std::vector<rg::Building>{};
        },
        {}, 1000.0);
    obstacles.request_around(1500.0, 2500.0); // tile (1, 2)
    REQUIRE(obstacles.wait_idle(5.0));
    CHECK(obstacles.loaded_tile_count() == 9);
    CHECK(obstacles.ready(1500.0, 2500.0));
    CHECK(obstacles.ready(500.0, 1500.0));    // the corner tile (0, 1)
    CHECK_FALSE(obstacles.ready(4500.0, 2500.0));
    {
        std::lock_guard<std::mutex> lock(mutex);
        REQUIRE(fetched.size() == 9);
        CHECK(fetched.front() == std::pair<int, int>{1, 2}); // the tile under the request first
    }
    obstacles.request_around(1500.0, 2500.0); // nothing new to load
    REQUIRE(obstacles.wait_idle(5.0));
    {
        std::lock_guard<std::mutex> lock(mutex);
        CHECK(fetched.size() == 9);
    }
    // Driving far away: the old tiles are evicted once more than 25 are held.
    for (int step = 1; step <= 4; ++step) {
        obstacles.request_around(1500.0 + 3000.0 * step, 2500.0);
        REQUIRE(obstacles.wait_idle(5.0));
    }
    CHECK(obstacles.loaded_tile_count() <= 25);
    CHECK_FALSE(obstacles.ready(1500.0, 2500.0));
    CHECK(obstacles.ready(13500.0, 2500.0));
}

// ------------------------------------------------------------- director ----

TEST_CASE("cinematic: a footprint over the default shot point moves the shot out of it", "[cinematic][obstacles]") {
    FakeObstacles clear;
    const rg::CinematicShot plain = first_shot(&clear); // no obstacles: the unchanged placement
    CHECK(plain.source == rg::ShotSource::RoadAhead);

    FakeObstacles blocked;
    blocked.boxes.push_back(around(plain));
    rg::CinematicDirector director(42);
    director.set_obstacles(&blocked);
    const rg::CinematicShot shot = director.update(1.0 / 60.0, car_at_origin(), straight_road());
    // Sabotage: skipping the camera-point test keeps `plain` - a camera inside the building.
    CHECK_FALSE(blocked.building_covers(shot.x, shot.y, shot.height_above_ground_m, 1.0));
    CHECK(std::hypot(shot.x - plain.x, shot.y - plain.y) > 5.0);
    CHECK(shot.source == rg::ShotSource::RoadAhead); // a clear candidate was found, no fallback
    CHECK(director.stats().rejected_in_building >= 1);
    CHECK(director.stats().fallbacks == 0);
}

TEST_CASE("cinematic: a footprint within 1 m of the camera point rejects it, 2 m away does not", "[cinematic][obstacles]") {
    FakeObstacles clear;
    const rg::CinematicShot plain = first_shot(&clear);
    // A box whose edge is 0.6 m from the shot point (rejected, margin 1 m) ...
    FakeObstacles near_box;
    near_box.boxes.push_back({plain.x + 0.6, plain.y - 30.0, plain.x + 10.0, plain.y + 30.0, -500.0, 500.0});
    near_box.boxes.push_back({plain.x - 10.0, plain.y - 30.0, plain.x - 0.6, plain.y + 30.0, -500.0, 500.0});
    rg::CinematicDirector rejected(42);
    rejected.set_obstacles(&near_box);
    rejected.update(1.0 / 60.0, car_at_origin(), straight_road());
    CHECK(rejected.stats().rejected_in_building >= 1);
    // ... and one 2 m away (accepted: it is not blocking the sight line either, being behind the camera point).
    FakeObstacles far_box;
    const double y_a = plain.y + 2.0 * plain.side, y_b = plain.y + 10.0 * plain.side;
    far_box.boxes.push_back({plain.x + 2.0, std::min(y_a, y_b), plain.x + 10.0, std::max(y_a, y_b), -500.0, 500.0});
    rg::CinematicDirector accepted(42);
    accepted.set_obstacles(&far_box);
    const auto& shot = accepted.update(1.0 / 60.0, car_at_origin(), straight_road());
    CHECK(shot.x == Catch::Approx(plain.x));
    CHECK(shot.y == Catch::Approx(plain.y));
    CHECK(accepted.stats().rejected_in_building == 0);
}

TEST_CASE("cinematic: a building between the camera and the car is rejected", "[cinematic][obstacles]") {
    FakeObstacles clear;
    const rg::CinematicShot plain = first_shot(&clear);
    // A wall 40 % of the way from the camera to the car, on the camera's side of the road.
    FakeObstacles walled;
    const double wx = plain.x + (0.0 - plain.x) * 0.4, wy = plain.y + (0.0 - plain.y) * 0.4;
    walled.boxes.push_back({wx - 3.0, wy - 3.0, wx + 3.0, wy + 3.0, -500.0, 500.0});
    CHECK(walled.building_blocks(plain.x, plain.y, 2.0, 0.0, 0.0, 1.0));
    rg::CinematicDirector director(42);
    director.set_obstacles(&walled);
    const rg::CinematicShot shot = director.update(1.0 / 60.0, car_at_origin(), straight_road());
    // Sabotage: skipping the sight-line test keeps `plain`, with the wall in front of the lens.
    CHECK(director.stats().rejected_sight >= 1);
    CHECK_FALSE(walled.building_blocks(shot.x, shot.y, 2.0, 0.0, 0.0, 1.0));
    CHECK((std::fabs(shot.x - plain.x) > 1.0 || std::fabs(shot.y - plain.y) > 1.0));
    CHECK(shot.source == rg::ShotSource::RoadAhead);
}

TEST_CASE("cinematic: terrain between the camera and the car is rejected, the other side is tried", "[cinematic][obstacles]") {
    FakeObstacles clear;
    const rg::CinematicShot plain = first_shot(&clear);
    // A 15 m ridge across x in [30, 45] on the camera's side of the road only.
    const double side = plain.side;
    FakeObstacles ridge;
    ridge.ground = [side](double x, double y) { return (x >= 30.0 && x <= 45.0 && y * side > 0.0) ? 15.0 : 0.0; };
    rg::CinematicDirector director(42);
    director.set_obstacles(&ridge);
    const rg::CinematicShot shot = director.update(1.0 / 60.0, car_at_origin(), straight_road());
    // Sabotage: skipping the terrain samples keeps `plain` on the far side of the ridge.
    CHECK(director.stats().rejected_sight >= 1);
    CHECK(shot.source == rg::ShotSource::RoadAhead);
    CHECK(shot.side == -plain.side); // the other road side
}

TEST_CASE("cinematic: no clear shot frames the car from behind, follows it, and finds a roadside shot again", "[cinematic][obstacles]") {
    FakeObstacles wall;
    rg::CarKinematics car = car_at_origin();
    // A ridge across the whole road 10-25 m ahead of the car, wherever the car is.
    wall.ground = [&car](double x, double) { return (x >= car.x + 10.0 && x <= car.x + 25.0) ? 15.0 : 0.0; };
    rg::CinematicDirector director(42);
    director.set_obstacles(&wall);
    const double dt = 1.0 / 60.0;

    const rg::CinematicShot first = director.update(dt, car, straight_road(car.x));
    CHECK(first.source == rg::ShotSource::ChaseFallback);
    CHECK(director.cut_this_update());
    CHECK(first.serial == 1);
    CHECK(first.x == Catch::Approx(-8.0)); // fallback_distance_m behind the car
    CHECK(first.y == Catch::Approx(0.0));
    CHECK(director.stats().fallbacks == 1);

    // The camera rides with the car: no new cut, the position keeps its distance.
    for (int i = 0; i < 30; ++i) {
        car.x += 20.0 * dt;
        const auto& s = director.update(dt, car, straight_road(car.x));
        CHECK_FALSE(director.cut_this_update());
        CHECK(s.serial == 1);
        CHECK(s.x == Catch::Approx(car.x - 8.0));
    }

    // Retrying while the obstruction stays does not cut either (same fallback, same serial).
    for (int i = 0; i < 120; ++i) {
        car.x += 20.0 * dt;
        director.update(dt, car, straight_road(car.x));
        CHECK_FALSE(director.cut_this_update());
    }
    CHECK(director.shot().serial == 1);

    // The ridge goes away (the car drove on, say): the next retry finds a roadside shot and cuts to it.
    wall.ground = nullptr;
    bool cut = false;
    for (int i = 0; i < 120 && !cut; ++i) {
        car.x += 20.0 * dt;
        director.update(dt, car, straight_road(car.x));
        cut = director.cut_this_update();
    }
    CHECK(cut);
    CHECK(director.shot().source == rg::ShotSource::RoadAhead);
    CHECK(director.shot().serial == 2);
}

TEST_CASE("cinematic: while obstacle data is still loading the shot is a fallback, then a roadside shot", "[cinematic][obstacles]") {
    FakeObstacles loading;
    loading.loaded = false;
    rg::CinematicDirector director(42);
    director.set_obstacles(&loading);
    const double dt = 1.0 / 60.0;
    CHECK(director.update(dt, car_at_origin(), straight_road()).source == rg::ShotSource::ChaseFallback);
    CHECK(director.stats().rejected_not_ready >= 1);
    loading.loaded = true;
    bool cut = false;
    for (int i = 0; i < 120 && !cut; ++i) {
        director.update(dt, car_at_origin(), straight_road());
        cut = director.cut_this_update();
    }
    CHECK(cut);
    CHECK(director.shot().source == rg::ShotSource::RoadAhead);
}

TEST_CASE("cinematic: without obstacles the placement is unchanged by the search", "[cinematic][obstacles]") {
    rg::CinematicDirector a(7), b(7);
    FakeObstacles empty; // nothing blocks anything
    b.set_obstacles(&empty);
    rg::CarKinematics car = car_at_origin();
    for (int i = 0; i < 2000; ++i) {
        car.x += 20.0 / 60.0;
        const auto sa = a.update(1.0 / 60.0, car, straight_road());
        const auto sb = b.update(1.0 / 60.0, car, straight_road());
        REQUIRE(sa.serial == sb.serial);
        REQUIRE(sa.x == Catch::Approx(sb.x));
        REQUIRE(sa.y == Catch::Approx(sb.y));
    }
    CHECK(b.stats().fallbacks == 0);
    CHECK(b.stats().rejected_in_building == 0);
    CHECK(b.stats().rejected_sight == 0);
}
