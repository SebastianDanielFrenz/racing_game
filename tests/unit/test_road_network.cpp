// test_road_network.cpp - rg::RoadNetwork / plan_path / smooth_polyline (G3/R3 slice S3), on small
// synthetic networks (no store, no world). Sabotage per test, recorded 2026-10-05 (each was applied
// to the implementation, the test run and seen to FAIL, then reverted):
//   junction detection  : counting degree >= 2 as a junction  -> the straight road reports 1 junction
//   clustering          : kJunctionClusterM compared as 0     -> the 20 m pair stays two clusters
//   oneway / access     : ignoring the oneway tag             -> arc counts and the closed direction fail
//   U-turn ban          : removing the `nb.to == arc.from` skip -> the lollipop path is empty, not 3 arcs
//   dual carriageway    : same ignored oneway                 -> way direction -1 is accepted (no error)
//   bridge runs         : treating bridge=no as a bridge      -> the non-bridge way reports a run
//   corner smoothing    : radius_m = 0 (no fillet)            -> the corner keeps radius 0
#include "rg/road_network.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace {

using Tags = std::vector<std::pair<std::string, std::string>>;

struct Builder {
    rg::RoadNetwork net;
    void node(std::int64_t id, double x, double y) { net.add_node({id, x, y}); }
    void way(std::int64_t id, std::vector<std::int64_t> nodes, Tags tags = {{"highway", "residential"}}) {
        net.add_way({id, std::move(nodes), std::move(tags)});
    }
};

// A plus-shaped crossing: ways 10 (west-east) and 11 (south-north) meet at node 1.
Builder cross() {
    Builder b;
    b.node(1, 0, 0);
    b.node(2, -100, 0);
    b.node(3, 100, 0);
    b.node(4, 0, -100);
    b.node(5, 0, 100);
    b.way(10, {2, 1, 3});
    b.way(11, {4, 1, 5});
    b.net.finish();
    return b;
}

double circle_radius(std::pair<double, double> a, std::pair<double, double> b, std::pair<double, double> c) {
    const double ab = std::hypot(b.first - a.first, b.second - a.second);
    const double bc = std::hypot(c.first - b.first, c.second - b.second);
    const double ca = std::hypot(a.first - c.first, a.second - c.second);
    const double cross2 =
        std::abs((b.first - a.first) * (c.second - a.second) - (b.second - a.second) * (c.first - a.first));
    return cross2 == 0.0 ? 1e30 : ab * bc * ca / (2.0 * cross2);
}

} // namespace

TEST_CASE("road_network: a crossing is one junction, a straight road none", "[road_network]") {
    const Builder b = cross();
    REQUIRE(b.net.junctions().size() == 1);
    CHECK(b.net.junctions()[0].node_ids == std::vector<std::int64_t>{1});
    CHECK(b.net.junction_of_node(b.net.node_index(1)) == 0);
    CHECK(b.net.junction_of_node(b.net.node_index(2)) == -1);

    Builder straight; // two ways joined end to end: degree 2 everywhere
    straight.node(1, 0, 0);
    straight.node(2, 100, 0);
    straight.node(3, 200, 0);
    straight.way(10, {1, 2});
    straight.way(11, {2, 3});
    straight.net.finish();
    CHECK(straight.net.junctions().empty());
}

TEST_CASE("road_network: junction nodes within 30 m form one cluster, 40 m apart two", "[road_network]") {
    auto build = [](double gap) {
        Builder b;
        b.node(1, 0, 0);
        b.node(6, gap, 0);
        b.node(2, -100, 0);
        b.node(3, gap + 100, 0);
        b.node(4, 0, -80);
        b.node(7, gap, -80);
        b.way(10, {2, 1, 6, 3});
        b.way(11, {4, 1});
        b.way(12, {7, 6});
        b.net.finish();
        return b;
    };
    const Builder near = build(20.0);
    REQUIRE(near.net.junctions().size() == 1);
    CHECK(near.net.junctions()[0].node_ids == std::vector<std::int64_t>{1, 6});
    CHECK(near.net.junctions()[0].radius_m == Catch::Approx(10.0));
    CHECK(near.net.junctions()[0].x == Catch::Approx(10.0));
    const Builder far = build(40.0);
    CHECK(far.net.junctions().size() == 2);
    // The cluster identity is the node set: the same data gives the same ids.
    CHECK(build(20.0).net.junctions()[0].node_ids == near.net.junctions()[0].node_ids);
}

TEST_CASE("road_network: oneway, access and class decide the arcs", "[road_network]") {
    Builder b;
    for (int i = 1; i <= 8; ++i) b.node(i, 100.0 * i, 0);
    b.way(10, {1, 2}, {{"highway", "residential"}});                       // two arcs
    b.way(11, {2, 3}, {{"highway", "residential"}, {"oneway", "yes"}});    // forward only
    b.way(12, {3, 4}, {{"highway", "residential"}, {"oneway", "-1"}});     // backward only
    b.way(13, {4, 5}, {{"highway", "motorway"}});                          // implied oneway forward
    b.way(14, {5, 6}, {{"highway", "service"}});                           // not drivable
    b.way(15, {6, 7}, {{"highway", "residential"}, {"access", "private"}}); // not drivable
    b.way(16, {7, 8}, {{"highway", "residential"}, {"junction", "roundabout"}}); // implied oneway forward
    b.net.finish();
    auto count = [&](std::int64_t way, int dir) { return b.net.way_arcs(way, dir).size(); };
    CHECK(count(10, 1) == 1);
    CHECK(count(10, -1) == 1);
    CHECK(count(11, 1) == 1);
    CHECK(count(11, -1) == 0);
    CHECK(count(12, 1) == 0);
    CHECK(count(12, -1) == 1);
    CHECK(count(13, 1) == 1);
    CHECK(count(13, -1) == 0);
    CHECK(b.net.way_index(14) == -1);
    CHECK(b.net.way_index(15) == -1);
    CHECK(count(16, 1) == 1);
    CHECK(count(16, -1) == 0);
    CHECK(b.net.stats().ways == 5);
}

TEST_CASE("road_network: shortest_arc_path never U-turns", "[road_network]") {
    // A two-way road 1-2 whose end 2 carries a triangle loop 2-3-4-2: to come back along the road the
    // route has to drive the loop, three arcs, not reverse on the spot.
    Builder b;
    b.node(1, 0, 0);
    b.node(2, 100, 0);
    b.node(3, 130, 20);
    b.node(4, 130, -20);
    b.way(10, {1, 2});
    b.way(11, {2, 3, 4, 2});
    b.net.finish();
    const auto out = b.net.way_arcs(10, 1);   // 1 -> 2
    const auto back = b.net.way_arcs(10, -1); // 2 -> 1
    REQUIRE(out.size() == 1);
    REQUIRE(back.size() == 1);
    const auto path = b.net.shortest_arc_path(out[0], back[0]);
    REQUIRE(path.has_value());
    CHECK(path->size() == 3);
    for (int a : *path) CHECK(b.net.arcs()[static_cast<std::size_t>(a)].way_id == 11);

    Builder dead; // the same road with a plain dead end: no way back without a U-turn
    dead.node(1, 0, 0);
    dead.node(2, 100, 0);
    dead.way(10, {1, 2});
    dead.net.finish();
    CHECK_FALSE(dead.net.shortest_arc_path(dead.net.way_arcs(10, 1)[0], dead.net.way_arcs(10, -1)[0]).has_value());
}

TEST_CASE("plan_path: a dual carriageway is entered and left through the link roads", "[road_network]") {
    // Lead-in 7-1 (two-way), carriageway A 1->2 east and B 3->4 west (both oneway), link roads
    // 2->5->3 and 4->6->1 (oneway). Planning "traverse A, then traverse B" must go A, link, B.
    Builder b;
    b.node(7, -100, 0);
    b.node(1, 0, 0);
    b.node(2, 100, 0);
    b.node(3, 100, 10);
    b.node(4, 0, 10);
    b.node(5, 120, 5);
    b.node(6, -20, 5);
    b.way(20, {7, 1});
    b.way(21, {1, 2}, {{"highway", "trunk"}, {"oneway", "yes"}, {"bridge", "yes"}});
    b.way(22, {3, 4}, {{"highway", "trunk"}, {"oneway", "yes"}, {"bridge", "yes"}});
    b.way(23, {2, 5, 3}, {{"highway", "trunk_link"}, {"oneway", "yes"}});
    b.way(24, {4, 6, 1}, {{"highway", "trunk_link"}, {"oneway", "yes"}});
    b.net.finish();
    using K = rg::RouteStep::Kind;
    std::string err;
    const auto p = rg::plan_path(b.net, -50.0, 0.4, 1.0, 0.0,
                                 {{K::TraverseWay, 21, 1}, {K::TraverseWay, 22, 1}}, &err);
    INFO(err);
    REQUIRE(p.has_value());
    std::vector<std::int64_t> ways;
    for (int a : p->arcs) ways.push_back(b.net.arcs()[static_cast<std::size_t>(a)].way_id);
    CHECK(ways == std::vector<std::int64_t>{20, 21, 23, 23, 22});
    CHECK(p->start_dist_m == Catch::Approx(0.4));
    REQUIRE(p->step_approach_m.size() == 2);
    CHECK(p->step_approach_m[0] == Catch::Approx(0.0));
    CHECK(p->step_approach_m[1] == Catch::Approx(std::hypot(20, 5) + std::hypot(20, 5)));
    // Both carriageways are bridges of one structure; each is crossed in its own direction.
    const auto runs = b.net.bridge_runs();
    REQUIRE(runs.size() == 2);
    CHECK(runs[0].way_id == 21);
    CHECK(runs[0].enter_node == 1);
    CHECK(runs[0].exit_node == 2);
    CHECK(runs[0].length_m == Catch::Approx(100.0));
    CHECK(runs[1].way_id == 22);
    // A closed direction is an error, not a silent U-turn.
    CHECK_FALSE(rg::plan_path(b.net, -50.0, 0.4, 1.0, 0.0, {{K::TraverseWay, 21, -1}}, &err).has_value());
    CHECK(err.find("no arcs in direction -1") != std::string::npos);
    // Off the network: no start.
    CHECK_FALSE(rg::plan_path(b.net, -50.0, 80.0, 1.0, 0.0, {}, &err).has_value());
    // Via a node: the cheapest arc arriving there.
    const auto v = rg::plan_path(b.net, -50.0, 0.0, 1.0, 0.0, {{K::ViaNode, 5, 0}}, &err);
    REQUIRE(v.has_value());
    CHECK(b.net.nodes()[static_cast<std::size_t>(b.net.arcs()[static_cast<std::size_t>(v->arcs.back())].to)].osm_id == 5);
}

TEST_CASE("road_network: bridge=no is not a bridge, a two-way bridge has two runs", "[road_network]") {
    Builder b;
    b.node(1, 0, 0);
    b.node(2, 30, 0);
    b.node(3, 60, 0);
    b.node(4, 90, 0);
    b.way(10, {1, 2, 3}, {{"highway", "primary"}, {"bridge", "viaduct"}});
    b.way(11, {3, 4}, {{"highway", "primary"}, {"bridge", "no"}});
    b.net.finish();
    const auto runs = b.net.bridge_runs();
    REQUIRE(runs.size() == 2);
    CHECK(runs[0].way_id == 10);
    CHECK(runs[0].direction == 1);
    CHECK(runs[0].length_m == Catch::Approx(60.0));
    CHECK(runs[0].arcs.size() == 2);
    CHECK(runs[1].way_id == 10);
    CHECK(runs[1].direction == -1);
    CHECK(runs[1].enter_node == 3);
    CHECK(runs[1].exit_node == 1);
}

TEST_CASE("road_network: junctions_on_arcs lists each cluster once, in visit order", "[road_network]") {
    const Builder b = cross();
    const auto we = b.net.way_arcs(10, 1);  // 2 -> 1, 1 -> 3
    const auto ew = b.net.way_arcs(10, -1); // 3 -> 1, 1 -> 2
    REQUIRE(we.size() == 2);
    std::vector<int> path = we;
    path.insert(path.end(), ew.begin(), ew.end()); // through the junction twice
    CHECK(b.net.junctions_on_arcs(path) == std::vector<int>{0});
    CHECK(b.net.junctions_on_arcs({}).empty());
}

TEST_CASE("smooth_polyline: a right angle gets a fillet, spacing stays bounded, ends stay put", "[road_network]") {
    std::vector<std::pair<double, double>> l{{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}};
    const auto s = rg::smooth_polyline(l, 10.0, 5.0);
    REQUIRE(s.size() > 40);
    CHECK(s.front() == l.front());
    CHECK(s.back() == l.back());
    double min_radius = 1e30, max_step = 0.0;
    for (std::size_t i = 1; i < s.size(); ++i) {
        max_step = std::max(max_step, std::hypot(s[i].first - s[i - 1].first, s[i].second - s[i - 1].second));
    }
    for (std::size_t i = 1; i + 1 < s.size(); ++i) {
        min_radius = std::min(min_radius, circle_radius(s[i - 1], s[i], s[i + 1]));
    }
    CHECK(max_step <= 5.0 + 1e-9);
    CHECK(min_radius > 4.0); // a quadratic Bezier corner with tangent length 10 m bottoms out at ~7.07 m
    // The curve cuts the corner by about t / (2 sqrt 2) = 3.5 m: nearest approach to (100, 0).
    double nearest = 1e30;
    for (const auto& p : s) nearest = std::min(nearest, std::hypot(p.first - 100.0, p.second));
    CHECK(nearest == Catch::Approx(3.5).margin(0.8));
}

TEST_CASE("smooth_polyline: a straight line is only resampled", "[road_network]") {
    std::vector<std::pair<double, double>> l{{0.0, 0.0}, {50.0, 0.0}, {120.0, 0.0}};
    const auto s = rg::smooth_polyline(l, 10.0, 5.0);
    CHECK(s.size() == 25);
    for (const auto& p : s) CHECK(p.second == 0.0);
}
