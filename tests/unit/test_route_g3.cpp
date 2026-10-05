// test_route_g3.cpp - the G3/R3 junction route (docs/g3_consumer_design.md 6.1, slice S3): the
// committed data/routes/home_g3_junctions.json recounted from its own waypoints (>= 20 distinct
// junction clusters, B8 bridge #4 and the L3014 bridge crossed in both directions), the rg.route/1
// loader's junction/bridge/generator fields, and the grade exemption for listed bridges.
// Everything runs on the committed file or synthetic data - CI never reads cache/. The hidden
// [.][realdata] case checks the file against the real roads.graph store (RG_G2M_HOME).
//
// Sabotage, recorded 2026-10-05 (applied, test run and seen to FAIL, then reverted):
//   delete the waypoints across one junction      -> that id leaves the crossed list, the count drops
//   delete the waypoints across six junctions     -> the count falls below criteria.min_junctions
//   delete the waypoints across one bridge        -> route_crosses_bridge is false for that entry
//   grade exemption with an empty exempt list     -> the 60 m pit fails the 12 % criterion
//   count_junctions_crossed with radius 0         -> the committed route crosses 0 junctions
#include "rg/road_network.h"
#include "rg/route_check.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include "g2m/core/tile_key.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace {

const std::string kRoute = std::string(RG_SOURCE_DIR) + "/data/routes/home_g3_junctions.json";

rg::Route load_committed() {
    std::string err;
    const auto r = rg::load_route(kRoute, &err);
    INFO(err);
    REQUIRE(r.has_value());
    return *r;
}

// The waypoints without those within `radius` of any of the given centres.
std::vector<rg::RoutePoint> without_near(const std::vector<rg::RoutePoint>& wps, const std::vector<std::pair<double, double>>& centres,
                                         double radius) {
    std::vector<rg::RoutePoint> out;
    for (const auto& p : wps) {
        bool near = false;
        for (const auto& c : centres) near = near || std::hypot(p.x - c.first, p.y - c.second) <= radius;
        if (!near) out.push_back(p);
    }
    return out;
}

bool have_env(const char* name) {
#ifdef _WIN32
    char* buf = nullptr;
    std::size_t len = 0;
    const bool ok = _dupenv_s(&buf, &len, name) == 0 && buf != nullptr;
    std::free(buf);
    return ok;
#else
    return std::getenv(name) != nullptr;
#endif
}

std::string write_temp(const std::string& name, const std::string& text) {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream(path, std::ios::binary) << text;
    return path.string();
}

// A minimal valid route file around `extra` (inserted before "waypoints").
std::string route_text(const std::string& extra) {
    return std::string("{\"format\":\"rg.route/1\",\"name\":\"t\",\"session_origin_utm\":{\"zone\":32,\"e0\":464000,\"n0\":5559000},"
                       "\"spawn\":{\"x\":0,\"y\":0,\"yaw_deg\":0},") +
           extra + "\"waypoints\":[[0,0],[10,0],[20,0]]}";
}

std::string load_error(const std::string& extra) {
    std::string err;
    const auto r = rg::load_route(write_temp("rg_route_g3_test.json", route_text(extra)), &err);
    REQUIRE_FALSE(r.has_value());
    return err;
}

} // namespace

TEST_CASE("route_g3: the committed route passes >= 20 distinct junctions, recounted from the waypoints", "[route_g3]") {
    const rg::Route r = load_committed();
    REQUIRE(r.criteria.min_junctions.has_value());
    CHECK(*r.criteria.min_junctions >= 20);
    REQUIRE(r.generator.has_value());
    CHECK(r.generator->tool == "rg_route_gen");
    CHECK(r.generator->version == 1);
    CHECK(r.generator->geo2map_pin == "54d7083");
    CHECK(r.generator->junction_cluster_m == rg::kJunctionClusterM);
    CHECK(r.generator->drivable_classes_version == rg::kDrivableClassesVersion);
    // spawn / waypoints[0]
    CHECK(r.waypoints[0].x == r.spawn.x);
    CHECK(r.waypoints[0].y == r.spawn.y);
    // waypoints are at most 5 m apart (the generator's spacing)
    double length = 0.0;
    for (std::size_t i = 1; i < r.waypoints.size(); ++i) {
        const double d = std::hypot(r.waypoints[i].x - r.waypoints[i - 1].x, r.waypoints[i].y - r.waypoints[i - 1].y);
        CHECK(d <= 5.01);
        length += d;
    }
    CHECK(length > 3000.0);
    // junctions: distinct ids, sorted node sets, ids named after the smallest node, s in visit order
    std::set<std::string> ids;
    double prev_s = -1.0;
    for (const rg::RouteJunction& j : r.junctions) {
        CHECK(ids.insert(j.id).second);
        REQUIRE_FALSE(j.node_ids.empty());
        CHECK(std::is_sorted(j.node_ids.begin(), j.node_ids.end()));
        CHECK(j.id == "n" + std::to_string(j.node_ids.front()));
        CHECK(j.s_m >= prev_s);
        prev_s = j.s_m;
    }
    std::vector<std::string> crossed;
    const int n = rg::count_junctions_crossed(r.waypoints, r.junctions, &crossed);
    CHECK(n >= *r.criteria.min_junctions);
    CHECK(n == static_cast<int>(r.junctions.size())); // every listed junction is really passed
    CHECK(crossed.size() == r.junctions.size());
}

TEST_CASE("route_g3: B8 bridge #4 and the L3014 bridge are crossed in both directions", "[route_g3]") {
    const rg::Route r = load_committed();
    std::map<std::string, std::set<int>> dirs;
    std::map<std::string, std::set<std::int64_t>> ways;
    for (const rg::RouteBridge& b : r.bridges) {
        CHECK(rg::route_crosses_bridge(r.waypoints, b));
        CHECK(b.s_exit_m > b.s_enter_m);
        CHECK(b.length_m > 5.0);
        dirs[b.group].insert(b.direction);
        ways[b.group].insert(b.way_id);
    }
    // B8 bridge #4 is a dual carriageway: two oneway ways, each crossed in its own (only) direction.
    REQUIRE(ways.count("B8 bridge #4") == 1);
    CHECK(ways["B8 bridge #4"] == std::set<std::int64_t>{1096866567, 1096866569});
    // The L3014 bridge is one two-way way: both directions of it.
    REQUIRE(dirs.count("L3014 bridge") == 1);
    CHECK(dirs["L3014 bridge"] == std::set<int>{-1, 1});
    CHECK(ways["L3014 bridge"] == std::set<std::int64_t>{14799333});
}

TEST_CASE("route_g3: sabotage - deleting the waypoints across a junction or a bridge is detected", "[route_g3]") {
    const rg::Route r = load_committed();
    const int total = rg::count_junctions_crossed(r.waypoints, r.junctions);
    // one junction
    const rg::RouteJunction& j = r.junctions[10];
    const auto cut = without_near(r.waypoints, {{j.x, j.y}}, j.radius_m + 6.0);
    std::vector<std::string> crossed;
    const int after = rg::count_junctions_crossed(cut, r.junctions, &crossed);
    CHECK(after < total);
    CHECK(std::find(crossed.begin(), crossed.end(), j.id) == crossed.end());
    // six junctions: below the route's own minimum
    std::vector<std::pair<double, double>> centres;
    for (std::size_t i = 0; i < 6; ++i) centres.push_back({r.junctions[i].x, r.junctions[i].y});
    const auto cut6 = without_near(r.waypoints, centres, 60.0);
    CHECK(rg::count_junctions_crossed(cut6, r.junctions) < *r.criteria.min_junctions);
    // one bridge: the waypoints no longer cross it
    const rg::RouteBridge& b = r.bridges[0];
    const auto cutb = without_near(r.waypoints, {{(b.enter_x + b.exit_x) / 2, (b.enter_y + b.exit_y) / 2}}, b.length_m / 2 + 15.0);
    CHECK_FALSE(rg::route_crosses_bridge(cutb, b));
    // a junction radius of 0 would find none
    std::vector<rg::RouteJunction> zero = r.junctions;
    for (auto& z : zero) z.radius_m = 1e-9;
    CHECK(rg::count_junctions_crossed(r.waypoints, zero) < total);
}

TEST_CASE("route_g3: the loader reads min_junctions, junctions, bridges and the generator block", "[route_g3][io]") {
    const std::string good =
        "\"criteria\":{\"min_junctions\":2},"
        "\"generator\":{\"tool\":\"rg_route_gen\",\"version\":1,\"config\":\"c.json\",\"geo2map_pin\":\"abc\",\"drivable_classes_version\":1,\"junction_cluster_m\":30},"
        "\"junctions\":[{\"id\":\"n1\",\"x\":10,\"y\":0,\"radius_m\":8,\"s_m\":10,\"node_ids\":[1,2]}],"
        "\"bridges\":[{\"group\":\"g\",\"way_id\":5,\"direction\":-1,\"enter_node\":1,\"exit_node\":2,\"enter_x\":0,\"enter_y\":0,\"exit_x\":20,\"exit_y\":0,"
        "\"length_m\":20,\"s_enter_m\":0,\"s_exit_m\":20}],";
    std::string err;
    const auto r = rg::load_route(write_temp("rg_route_g3_good.json", route_text(good)), &err);
    INFO(err);
    REQUIRE(r.has_value());
    CHECK(*r->criteria.min_junctions == 2);
    REQUIRE(r->junctions.size() == 1);
    CHECK(r->junctions[0].node_ids == std::vector<std::int64_t>{1, 2});
    REQUIRE(r->bridges.size() == 1);
    CHECK(r->bridges[0].direction == -1);
    CHECK(r->generator->geo2map_pin == "abc");
    CHECK(rg::count_junctions_crossed(r->waypoints, r->junctions) == 1);
    CHECK(rg::route_crosses_bridge(r->waypoints, r->bridges[0]));

    CHECK(load_error("\"criteria\":{\"min_junctions\":2.5},").find("min_junctions") != std::string::npos);
    CHECK(load_error("\"junctions\":[{\"id\":\"n1\",\"x\":0,\"y\":0,\"s_m\":0,\"node_ids\":[1]}],").find("junctions[0]") != std::string::npos);
    CHECK(load_error("\"junctions\":[{\"id\":\"n1\",\"x\":0,\"y\":0,\"radius_m\":5,\"s_m\":0,\"node_ids\":[]}],").find("node_ids") != std::string::npos);
    CHECK(load_error("\"bridges\":[{\"group\":\"g\",\"way_id\":5,\"direction\":0,\"enter_node\":1,\"exit_node\":2,\"enter_x\":0,\"enter_y\":0,"
                     "\"exit_x\":20,\"exit_y\":0,\"length_m\":20,\"s_enter_m\":0,\"s_exit_m\":20}],").find("direction") != std::string::npos);
    CHECK(load_error("\"criteria\":{\"min_junction\":2},").find("unknown key") != std::string::npos);
}

TEST_CASE("route_g3: grade windows touching a listed bridge are exempt from the grade criterion", "[route_g3]") {
    // A straight 400 m route over a 60 m deep pit between s = 150 and s = 190: 100 % grade there.
    std::vector<rg::RoutePoint> wps;
    for (int i = 0; i <= 400; i += 5) wps.push_back({static_cast<double>(i), 0.0});
    const rg::HeightAtFn pit = [](double x, double) -> std::optional<double> {
        return (x >= 160.0 && x <= 180.0) ? 0.0 : (x > 180.0 && x < 190.0 ? 5.0 : (x > 150.0 && x < 160.0 ? 5.0 : 10.0));
    };
    rg::RouteCheckParams p;
    p.min_length_m = 300.0;
    p.min_seam_crossings = 0;
    p.min_corner_radius_m = 0.0;
    const rg::RouteCheckReport bare = rg::check_route(wps, wps[0], pit, p);
    CHECK_FALSE(bare.ok());
    CHECK(bare.grade_window_exempt_count == 0);
    p.grade_exempt_s.emplace_back(150.0, 190.0);
    const rg::RouteCheckReport exempt = rg::check_route(wps, wps[0], pit, p);
    INFO(rg::format_route_report(exempt));
    CHECK(exempt.ok());
    CHECK(exempt.grade_window_exempt_count > 30);
}

TEST_CASE("route_g3: the committed route lies on the real road graph and its junctions re-resolve",
          "[.][realdata][route_g3]") {
    if (!have_env("RG_G2M_HOME")) {
        SKIP("RG_G2M_HOME not set");
    }
    std::string err;
    const auto world = rg::load_world_config(std::string(RG_SOURCE_DIR) + "/data/world/world_config.json", &err);
    INFO(err);
    REQUIRE(world.has_value());
    auto terrain = rg::WorldTerrain::open(*world, &err);
    REQUIRE(terrain != nullptr);
    const rg::Route r = load_committed();
    rg::RoadNetwork net;
    const double e0 = world->session_origin_utm.e0, n0 = world->session_origin_utm.n0;
    for (double y = -10000.0; y <= 1024.0; y += 1024.0) {
        for (double x = -3500.0; x <= 6500.0; x += 1024.0) {
            const auto key = g2m::tile_key_at(terrain->frame().zone(), 2, e0 + x, n0 + y);
            if (!key) continue;
            const auto t = terrain->road_graph_tile(*key);
            REQUIRE(t != nullptr);
            net.add_road_graph_tile(*t, e0, n0);
        }
    }
    net.finish();
    // Every junction record re-resolves: a cluster with exactly the stored node set exists.
    for (const rg::RouteJunction& j : r.junctions) {
        const int ni = net.node_index(j.node_ids.front());
        REQUIRE(ni >= 0);
        const int c = net.junction_of_node(ni);
        INFO(j.id);
        REQUIRE(c >= 0);
        CHECK(net.junctions()[static_cast<std::size_t>(c)].node_ids == j.node_ids);
    }
    // Every bridge way exists in the stored direction.
    for (const rg::RouteBridge& b : r.bridges) CHECK_FALSE(net.way_arcs(b.way_id, b.direction).empty());
    // Every waypoint is within 6 m of a drivable arc (the route is on roads; corner fillets cut up to ~5 m).
    int off = 0;
    for (std::size_t i = 0; i < r.waypoints.size(); i += 4) {
        if (net.nearest_arc(r.waypoints[i].x, r.waypoints[i].y, 0.0, 0.0, 6.0, -1.0) < 0) ++off;
    }
    CHECK(off == 0);
}
