// test_road_ahead.cpp - rg::trace_road_ahead on synthetic road segments:
// straight chains, direction from the heading, snapping, junction choice, dead
// ends, tile boundaries, determinism. Pure (no real data).
#include "rg/road_ahead.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <span>
#include <vector>

namespace {

using Catch::Matchers::WithinAbs;

// UTM-like grid metres; far from zero on purpose (mm rounding of a big origin).
constexpr double kE = 464000.0;
constexpr double kN = 5559000.0;

g2m::RoadSegment seg(double ax, double ay, double bx, double by, std::int64_t way, double half_width_m = 3.0) {
    g2m::RoadSegment s;
    s.a = {static_cast<std::int64_t>(std::llround((kE + ax) * 1000.0)), static_cast<std::int64_t>(std::llround((kN + ay) * 1000.0))};
    s.b = {static_cast<std::int64_t>(std::llround((kE + bx) * 1000.0)), static_cast<std::int64_t>(std::llround((kN + by) * 1000.0))};
    s.half_width_mm = static_cast<std::int64_t>(std::llround(half_width_m * 1000.0));
    s.way_id = way;
    return s;
}

// A straight road along +x from x=-50 to x=300 in 10 m pieces.
std::vector<g2m::RoadSegment> straight_road() {
    std::vector<g2m::RoadSegment> v;
    for (int i = -5; i < 30; ++i) v.push_back(seg(i * 10.0, 0.0, (i + 1) * 10.0, 0.0, 1));
    return v;
}

bool trace(const std::vector<g2m::RoadSegment>& segs, double car_x, double car_y, double hx, double hy,
           std::vector<rg::RoadPoint>& out, double anchor_dx = 0.0, double anchor_dy = 0.0,
           const rg::RoadAheadParams& params = {}) {
    // The output is anchored at the car's own position plus the offset, i.e.
    // with a zero offset the points come back in the same frame as the
    // segments (the car at (car_x, car_y) is where rg::Session anchors them).
    const double anchor_x = car_x + anchor_dx;
    const double anchor_y = car_y + anchor_dy;
    std::vector<std::span<const g2m::RoadSegment>> tiles;
    tiles.emplace_back(segs.data(), segs.size());
    return rg::trace_road_ahead(tiles, kE + car_x, kN + car_y, hx, hy, anchor_x, anchor_y, params, out);
}

double polyline_length(const std::vector<rg::RoadPoint>& p) {
    double l = 0.0;
    for (std::size_t i = 0; i + 1 < p.size(); ++i) l += std::hypot(p[i + 1].x - p[i].x, p[i + 1].y - p[i].y);
    return l;
}

} // namespace

TEST_CASE("road ahead: follows a straight road in the direction of travel", "[road_ahead]") {
    const auto road = straight_road();
    std::vector<rg::RoadPoint> out;
    REQUIRE(trace(road, 5.0, 1.0, 1.0, 0.0, out));
    REQUIRE(out.size() >= 3);
    // First point: the car's projection on the road (y = 0), x = the car's x.
    CHECK_THAT(out.front().x, WithinAbs(5.0, 1e-3));
    CHECK_THAT(out.front().y, WithinAbs(0.0, 1e-3));
    CHECK_THAT(out.front().half_width_m, WithinAbs(3.0, 1e-9));
    // Then each node onwards, x increasing.
    for (std::size_t i = 1; i < out.size(); ++i) {
        CHECK(out[i].x > out[i - 1].x);
        CHECK_THAT(out[i].y, WithinAbs(0.0, 1e-3));
    }
    // Stops once the road is long enough (max 160 m, one hop of slack).
    const double len = polyline_length(out);
    CHECK(len >= 160.0);
    CHECK(len < 160.0 + 10.0 + 1e-6);
}

TEST_CASE("road ahead: the heading picks the direction on a two-way road", "[road_ahead]") {
    const auto road = straight_road();
    std::vector<rg::RoadPoint> out;
    REQUIRE(trace(road, 105.0, 0.5, -1.0, 0.0, out));
    REQUIRE(out.size() >= 3);
    CHECK_THAT(out.front().x, WithinAbs(105.0, 1e-3));
    for (std::size_t i = 1; i < out.size(); ++i) CHECK(out[i].x < out[i - 1].x);
    // Ends at the road's west end (x = -50) at the latest.
    CHECK(out.back().x >= -50.0 - 1e-3);
}

TEST_CASE("road ahead: anchor offsets the output, nothing else", "[road_ahead]") {
    const auto road = straight_road();
    std::vector<rg::RoadPoint> a, b;
    REQUIRE(trace(road, 20.0, 0.0, 1.0, 0.0, a));
    REQUIRE(trace(road, 20.0, 0.0, 1.0, 0.0, b, 1000.0, -2000.0));
    REQUIRE(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK_THAT(b[i].x - a[i].x, WithinAbs(1000.0, 1e-9));
        CHECK_THAT(b[i].y - a[i].y, WithinAbs(-2000.0, 1e-9));
        CHECK(b[i].half_width_m == a[i].half_width_m);
    }
}

TEST_CASE("road ahead: refuses when not on a road or facing across it", "[road_ahead]") {
    const auto road = straight_road();
    std::vector<rg::RoadPoint> out{{1, 2, 3}};
    // 30 m off the road.
    CHECK_FALSE(trace(road, 20.0, 30.0, 1.0, 0.0, out));
    CHECK(out.empty()); // cleared
    // Driving across the road (heading 90 deg from the segment).
    CHECK_FALSE(trace(road, 20.0, 1.0, 0.0, 1.0, out));
    // Zero heading, no segments.
    CHECK_FALSE(trace(road, 20.0, 1.0, 0.0, 0.0, out));
    CHECK_FALSE(trace({}, 20.0, 1.0, 1.0, 0.0, out));
    // Within snapping distance but past the tolerance on either side.
    rg::RoadAheadParams tight;
    tight.max_snap_m = 2.0;
    CHECK_FALSE(trace(road, 20.0, 5.0, 1.0, 0.0, out, 0.0, 0.0, tight));
    CHECK(trace(road, 20.0, 1.5, 1.0, 0.0, out, 0.0, 0.0, tight));
}

TEST_CASE("road ahead: at a junction the straightest continuation wins", "[road_ahead]") {
    // Main road along +x with a side road branching at x = 50 to the left (+y)
    // and a gentle right bend continuing at x = 50.
    std::vector<g2m::RoadSegment> road;
    for (int i = 0; i < 5; ++i) road.push_back(seg(i * 10.0, 0.0, (i + 1) * 10.0, 0.0, 1));
    // Straight on: 50 -> 100 along +x.
    for (int i = 5; i < 10; ++i) road.push_back(seg(i * 10.0, 0.0, (i + 1) * 10.0, 0.0, 1));
    // Side road: 50,0 -> 50,100 (a 90 degree turn).
    for (int i = 0; i < 10; ++i) road.push_back(seg(50.0, i * 10.0, 50.0, (i + 1) * 10.0, 2));
    std::vector<rg::RoadPoint> out;
    REQUIRE(trace(road, 10.0, 0.0, 1.0, 0.0, out));
    for (const auto& p : out) CHECK_THAT(p.y, WithinAbs(0.0, 1e-3));
    CHECK(out.back().x > 90.0);

    // Heading roughly north on the side road, then straight again on it.
    REQUIRE(trace(road, 50.0, 20.0, 0.0, 1.0, out));
    for (const auto& p : out) CHECK_THAT(p.x, WithinAbs(50.0, 1e-3));
    CHECK(out.back().y > 90.0);
}

TEST_CASE("road ahead: a sharp turn at the end is not followed, a dead end ends the trace", "[road_ahead]") {
    std::vector<g2m::RoadSegment> road;
    for (int i = 0; i < 5; ++i) road.push_back(seg(i * 10.0, 0.0, (i + 1) * 10.0, 0.0, 1));
    // A U-turn back along y = 5 would dot < 0 with the heading.
    road.push_back(seg(50.0, 0.0, 50.0, 5.0, 2)); // 90 degrees: cos 0 < 0.2 threshold is only for dot >= 0.2
    std::vector<rg::RoadPoint> out;
    REQUIRE(trace(road, 5.0, 0.0, 1.0, 0.0, out));
    // The 90 degree side stub (cos 0) is below max_turn_cos 0.2: not followed.
    CHECK_THAT(out.back().x, WithinAbs(50.0, 1e-3));
    CHECK_THAT(out.back().y, WithinAbs(0.0, 1e-3));
    CHECK(polyline_length(out) < 60.0);
}

TEST_CASE("road ahead: segments in several tiles join, order does not matter", "[road_ahead]") {
    std::vector<g2m::RoadSegment> west, east;
    for (int i = -5; i < 10; ++i) west.push_back(seg(i * 10.0, 0.0, (i + 1) * 10.0, 0.0, 1));
    for (int i = 10; i < 30; ++i) east.push_back(seg(i * 10.0, 0.0, (i + 1) * 10.0, 0.0, 1));
    std::vector<std::span<const g2m::RoadSegment>> ab{{west.data(), west.size()}, {east.data(), east.size()}};
    std::vector<std::span<const g2m::RoadSegment>> ba{{east.data(), east.size()}, {west.data(), west.size()}};
    std::vector<rg::RoadPoint> o1, o2;
    REQUIRE(rg::trace_road_ahead(ab, kE + 60.0, kN, 1.0, 0.0, 0.0, 0.0, {}, o1));
    REQUIRE(rg::trace_road_ahead(ba, kE + 60.0, kN, 1.0, 0.0, 0.0, 0.0, {}, o2));
    REQUIRE(o1.size() == o2.size());
    CHECK(polyline_length(o1) >= 160.0); // crossed the tile boundary at x = 100
    for (std::size_t i = 0; i < o1.size(); ++i) {
        CHECK(o1[i].x == o2[i].x);
        CHECK(o1[i].y == o2[i].y);
    }
}

TEST_CASE("road ahead: a bend is followed and the half width varies per segment", "[road_ahead]") {
    std::vector<g2m::RoadSegment> road;
    // A quarter circle of radius 100 m turning left, in 10 degree pieces,
    // way width 2 m, then straight with 5 m.
    const double r = 100.0;
    const double pi = 3.14159265358979323846;
    double px = 0.0, py = -r; // centre at origin
    for (int i = 1; i <= 9; ++i) {
        const double ang = -pi / 2.0 + i * pi / 18.0;
        const double nx = r * std::cos(ang), ny = r * std::sin(ang);
        road.push_back(seg(px, py, nx, ny, 1, 2.0));
        px = nx;
        py = ny;
    }
    for (int i = 0; i < 10; ++i) road.push_back(seg(px, py + i * 10.0, px, py + (i + 1) * 10.0, 3, 5.0));
    std::vector<rg::RoadPoint> out;
    REQUIRE(trace(road, 0.0, -r, 1.0, 0.0, out));
    REQUIRE(out.size() > 8);
    CHECK_THAT(out.front().half_width_m, WithinAbs(2.0, 1e-9));
    // It turned left: y increased, and the final points are on the straight with 5 m.
    CHECK(out.back().y > out.front().y + 100.0);
    CHECK_THAT(out.back().half_width_m, WithinAbs(5.0, 1e-9));
}

TEST_CASE("road ahead: the hop cap bounds a loop", "[road_ahead]") {
    // A closed square ring: following it forever must stop at max_length_m.
    std::vector<g2m::RoadSegment> ring;
    const double s = 10.0;
    for (int i = 0; i < 4; ++i) {
        const double x0[4] = {0, 40, 40, 0}, y0[4] = {0, 0, 40, 40};
        const double x1[4] = {40, 40, 0, 0}, y1[4] = {0, 40, 40, 0};
        // 10 m pieces.
        for (int k = 0; k < 4; ++k) {
            const double t0 = k / 4.0, t1 = (k + 1) / 4.0;
            ring.push_back(seg(x0[i] + (x1[i] - x0[i]) * t0, y0[i] + (y1[i] - y0[i]) * t0, x0[i] + (x1[i] - x0[i]) * t1,
                               y0[i] + (y1[i] - y0[i]) * t1, 7));
        }
    }
    (void)s;
    rg::RoadAheadParams p;
    p.max_length_m = 300.0;
    p.max_turn_cos = -0.5; // allow the 90 degree corners
    std::vector<rg::RoadPoint> out;
    REQUIRE(trace(ring, 5.0, 0.0, 1.0, 0.0, out, 0.0, 0.0, p));
    CHECK(polyline_length(out) >= 300.0);
    CHECK(polyline_length(out) < 300.0 + 20.0);
}
