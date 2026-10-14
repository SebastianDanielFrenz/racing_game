// test_route_planner.cpp — rg::RoutePlanner, rg::route_progress and
// rg::MapView (R4 minimap / route core) on a synthetic road graph, tiled into
// level-2 roads.graph tiles with geo2map's own select_road_graph_tile (the
// same per-tile content the real store serves), so the planner runs its real
// region assembly path. No map store is needed.
#include "rg/lod_settings.h"
#include "rg/map_view.h"
#include "rg/route_planner.h"

#include "g2m/layer/osm_roads.h"
#include "g2m/route/route_search.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

using Catch::Approx;

namespace {

const std::string kRoot = RG_SOURCE_DIR;
const g2m::geo::UtmZone kZone{32, g2m::geo::Hemisphere::North};
// Session origin on a level-2 tile corner (tile 450, 5420).
constexpr double kE0 = 460800.0;
constexpr double kN0 = 5550080.0;
using Tags = std::vector<std::pair<std::string, std::string>>;

struct Builder {
    g2m::RoadGraph g;
    void node(std::int64_t id, double x, double y) {
        g.nodes.push_back({id, {std::llround((kE0 + x) * 1000.0), std::llround((kN0 + y) * 1000.0)}, {}});
    }
    void way(std::int64_t id, std::vector<std::int64_t> refs, Tags tags) {
        std::sort(tags.begin(), tags.end());
        int rank = 0;
        for (const auto& [k, v] : tags)
            if (k == "highway")
                if (const auto* e = g2m::RoadStyle::default_style().find_highway(v)) rank = e->rank;
        g.ways.push_back({id, rank, std::move(refs), std::move(tags)});
    }
    g2m::RoadGraph build() {
        std::sort(g.nodes.begin(), g.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
        std::sort(g.ways.begin(), g.ways.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
        g.edges.clear();
        for (const auto& w : g.ways)
            for (std::size_t i = 1; i < w.node_ids.size(); ++i)
                g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
        return g;
    }
};

// A 9 x 9 street grid, 200 m blocks, from (0, 0) to (1600, 1600): rows are
// residential "Reihe <j>", columns residential "Spalte <i>", except row 4 =
// primary "Hauptstrasse" (B 8). Plus an isolated way far east (unreachable).
std::int64_t grid_id(int i, int j) { return 1000 + j * 9 + i; }
g2m::RoadGraph grid_world() {
    Builder b;
    for (int j = 0; j < 9; ++j)
        for (int i = 0; i < 9; ++i) b.node(grid_id(i, j), 200.0 * i, 200.0 * j);
    for (int j = 0; j < 9; ++j) {
        std::vector<std::int64_t> refs;
        for (int i = 0; i < 9; ++i) refs.push_back(grid_id(i, j));
        if (j == 4) b.way(100 + j, refs, {{"highway", "primary"}, {"name", "Hauptstrasse"}, {"ref", "B 8"}});
        else b.way(100 + j, refs, {{"highway", "residential"}, {"name", "Reihe " + std::to_string(j)}});
    }
    for (int i = 0; i < 9; ++i) {
        std::vector<std::int64_t> refs;
        for (int j = 0; j < 9; ++j) refs.push_back(grid_id(i, j));
        b.way(200 + i, refs, {{"highway", "residential"}, {"name", "Spalte " + std::to_string(i)}});
    }
    b.node(9001, 3000.0, 3000.0);
    b.node(9002, 3200.0, 3000.0);
    b.way(900, {9001, 9002}, {{"highway", "residential"}, {"name", "Insel"}});
    return b.build();
}

// Every level-2 tile of the world, as the store would serve it.
struct TiledWorld {
    g2m::RoadGraph world;
    std::map<g2m::TileKey, std::shared_ptr<const g2m::RoadGraphTile>> tiles;
    std::atomic<int> fetches{0};
    std::set<g2m::TileKey> failing;
    explicit TiledWorld(g2m::RoadGraph g) : world(std::move(g)) {
        for (const auto& key : rg::tiles_in_square(kZone, 2, kE0, kN0, 2000.0, 2000.0, 2100.0)) {
            auto tile = g2m::select_road_graph_tile(world, key);
            REQUIRE(tile);
            tiles[key] = std::make_shared<const g2m::RoadGraphTile>(std::move(tile.value()));
        }
    }
    rg::RoadGraphTileSource source() {
        return [this](const g2m::TileKey& key) -> std::shared_ptr<const g2m::RoadGraphTile> {
            fetches.fetch_add(1);
            if (failing.contains(key)) return nullptr;
            const auto it = tiles.find(key);
            if (it != tiles.end()) return it->second;
            auto empty = std::make_shared<g2m::RoadGraphTile>();
            empty->key = key;
            return empty;
        };
    }
};

rg::RoutePlannerConfig test_config() {
    rg::RoutePlannerConfig c;
    c.region_radius_m = 2100.0;
    c.landmarks = 4;
    return c;
}

std::vector<g2m::TileKey> region(const rg::RoutePlannerConfig& c) {
    return rg::tiles_in_square(kZone, 2, kE0, kN0, 2000.0, 2000.0, c.region_radius_m);
}

rg::PlannedRoute plan(rg::RoutePlanner& p, double sx, double sy, double hx, double hy, double dx, double dy) {
    rg::PlannedRoute r;
    REQUIRE(p.wait(p.request(sx, sy, hx, hy, dx, dy), r));
    return r;
}

} // namespace

TEST_CASE("route planner: the shipped config loads; bad files are refused", "[route_planner]") {
    std::string err;
    const auto c = rg::load_route_planner_config(kRoot + "/data/world/route_planner.json", &err);
    INFO(err);
    REQUIRE(c.has_value());
    CHECK(c->landmarks == 16);
    CHECK(c->snap_max_distance_m == Approx(50.0));
    CHECK_FALSE(rg::parse_route_planner_config("{}", "t", &err));
    CHECK_FALSE(rg::parse_route_planner_config(R"({"format": "rg.route_planner/1"})", "t", &err));
    CHECK(err.find("missing") != std::string::npos);
}

TEST_CASE("route planner: region graph from tiles, a route, guidance and the polyline", "[route_planner]") {
    TiledWorld w(grid_world());
    const rg::RoutePlannerConfig config = test_config();
    rg::RoutePlanner planner(w.source(), region(config), kE0, kN0, config);

    // From the south-west corner heading east, to the north-east corner of the grid.
    const rg::PlannedRoute r = plan(planner, 10.0, 0.0, 1.0, 0.0, 1600.0, 1590.0);
    REQUIRE(planner.graph_state() == rg::RoutePlanner::GraphState::Ready);
    CHECK(planner.graph_stats().nodes == 81 + 2);
    CHECK(planner.graph_stats().landmarks == 4);
    INFO(r.message);
    REQUIRE(r.status == rg::PlannedRoute::Status::Ok);
    // Any monotone path through a grid has the Manhattan length.
    CHECK(r.length_m == Approx(1590.0 + 1590.0).margin(0.01));
    CHECK(r.duration_s > 0.0);
    REQUIRE(r.polyline.size() >= 3);
    CHECK(r.polyline.front()[0] == Approx(10.0).margin(0.001));
    CHECK(r.polyline.front()[1] == Approx(0.0).margin(0.001));
    CHECK(r.polyline.back()[0] == Approx(1600.0).margin(0.001));
    CHECK(r.polyline.back()[1] == Approx(1590.0).margin(0.001));
    CHECK(r.polyline_s.back() == Approx(r.length_m).margin(0.01));
    REQUIRE(r.maneuvers.size() >= 3);
    CHECK(r.maneuvers.front().kind == "depart");
    CHECK(r.maneuvers.back().kind == "arrive");
    CHECK(r.maneuvers.back().distance_from_start_m == Approx(r.length_m).margin(0.01));
    for (std::size_t i = 1; i < r.maneuvers.size(); ++i)
        CHECK(r.maneuvers[i].distance_from_start_m >= r.maneuvers[i - 1].distance_from_start_m);
    // Driving east first then north: the turn onto a column is a left.
    CHECK(r.maneuvers[1].kind == "left");

    // The same cost as geo2map's search on the whole (untiled) world graph.
    auto whole = g2m::route::build_routing_graph(w.world, {});
    REQUIRE(whole);
    g2m::route::RouteSearcher searcher(whole.value());
    g2m::route::SnapOptions so;
    so.heading_dx = 1.0;
    const auto from = whole.value().snap({std::llround((kE0 + 10.0) * 1000.0), std::llround(kN0 * 1000.0)}, so);
    const auto to = whole.value().snap({std::llround((kE0 + 1600.0) * 1000.0), std::llround((kN0 + 1590.0) * 1000.0)});
    REQUIRE(from);
    REQUIRE(to);
    const auto ref = searcher.find(from.value(), to.value());
    REQUIRE(ref);
    CHECK(r.duration_s == Approx(static_cast<double>(ref.value().cost_ms) / 1000.0));
    CHECK(r.length_m == Approx(static_cast<double>(ref.value().length_mm) / 1000.0));
}

TEST_CASE("route planner: snap failures, unreachable, the newest request wins", "[route_planner]") {
    TiledWorld w(grid_world());
    const rg::RoutePlannerConfig config = test_config();
    rg::RoutePlanner planner(w.source(), region(config), kE0, kN0, config);
    CHECK(plan(planner, 700.0, 700.0, 0, 0, 10.0, 0.0).status == rg::PlannedRoute::Status::NoStart); // block centre
    CHECK(plan(planner, 0.0, 0.0, 0, 0, 900.0, 900.0).status == rg::PlannedRoute::Status::NoDestination);
    CHECK(plan(planner, 0.0, 0.0, 0, 0, 3100.0, 3000.0).status == rg::PlannedRoute::Status::Unreachable);

    // Two requests back to back: the result that arrives is the second.
    const std::uint64_t a = planner.request(0.0, 0.0, 0, 0, 400.0, 0.0);
    const std::uint64_t b = planner.request(0.0, 0.0, 0, 0, 0.0, 400.0);
    CHECK(b > a);
    rg::PlannedRoute r;
    REQUIRE(planner.wait(b, r));
    CHECK(r.serial == b);
    CHECK(r.polyline.back()[1] == Approx(400.0).margin(0.001));
}

TEST_CASE("route planner: a failing tile fails the graph, cleanly", "[route_planner]") {
    TiledWorld w(grid_world());
    const rg::RoutePlannerConfig config = test_config();
    w.failing.insert(region(config).front());
    rg::RoutePlanner planner(w.source(), region(config), kE0, kN0, config);
    const rg::PlannedRoute r = plan(planner, 10.0, 0.0, 1.0, 0.0, 1600.0, 1590.0);
    CHECK(planner.graph_state() == rg::RoutePlanner::GraphState::Failed);
    CHECK(r.status == rg::PlannedRoute::Status::NoGraph);
    CHECK(planner.graph_message().find("failed") != std::string::npos);
}

TEST_CASE("route planner: destroying it while the graph builds does not hang", "[route_planner]") {
    TiledWorld w(grid_world());
    const rg::RoutePlannerConfig config = test_config();
    for (int k = 0; k < 20; ++k) {
        rg::RoutePlanner planner(w.source(), region(config), kE0, kN0, config);
    }
    SUCCEED();
}

TEST_CASE("route progress: position along the route and the next manoeuvres", "[route_planner]") {
    TiledWorld w(grid_world());
    const rg::RoutePlannerConfig config = test_config();
    rg::RoutePlanner planner(w.source(), region(config), kE0, kN0, config);
    const rg::PlannedRoute r = plan(planner, 10.0, 0.0, 1.0, 0.0, 1600.0, 1590.0);
    REQUIRE(r.status == rg::PlannedRoute::Status::Ok);
    REQUIRE(r.maneuvers.size() >= 3);

    const rg::RouteProgress start = rg::route_progress(r, 10.0, 2.0, 0.0, 2, config);
    REQUIRE(start.valid);
    CHECK(start.along_m == Approx(0.0).margin(0.01));
    CHECK(start.off_route_m == Approx(2.0).margin(0.01));
    CHECK_FALSE(start.off_route);
    REQUIRE(start.next.size() == 2);
    CHECK(start.next[0].maneuver.kind == r.maneuvers[1].kind);
    CHECK(start.next[0].distance_m == Approx(r.maneuvers[1].distance_from_start_m).margin(0.01));

    // Halfway to the first turn.
    const double turn_s = r.maneuvers[1].distance_from_start_m;
    const double half = turn_s / 2.0;
    const rg::RouteProgress mid = rg::route_progress(r, 10.0 + half, 0.0, 0.0, 3, config);
    CHECK(mid.along_m == Approx(half).margin(0.01));
    CHECK(mid.next[0].distance_m == Approx(turn_s - half).margin(0.01));
    CHECK(mid.remaining_m == Approx(r.length_m - half).margin(0.01));

    // Far off the line.
    const rg::RouteProgress off = rg::route_progress(r, 700.0, 700.0, half, 3, config);
    CHECK(off.off_route);
    // At the destination only "arrive" is left.
    const rg::RouteProgress end = rg::route_progress(r, 1600.0, 1590.0, r.length_m, 3, config);
    REQUIRE(end.next.size() == 1);
    CHECK(end.next[0].maneuver.kind == "arrive");
    CHECK(end.next[0].distance_m == Approx(0.0).margin(0.01));
    // No route: not valid.
    CHECK_FALSE(rg::route_progress(rg::PlannedRoute{}, 0.0, 0.0, 0.0, 3, config).valid);
}

TEST_CASE("map view: tiles around the centre, flattened to session metres", "[route_planner][map_view]") {
    TiledWorld w(grid_world());
    const g2m::route::MapConfig config = g2m::route::default_map_config();
    const rg::MapViewSelection sel = rg::select_map_view(config, kZone, kE0, kN0, 800.0, 800.0, 900.0);
    CHECK(sel.lod == g2m::route::MapLod::Detail);
    CHECK(sel.keys.size() == 9); // 1 km tiles over [-100, 1700] (origin on a tile corner)
    g2m::route::MapTileCache cache(config);
    const rg::MapViewFrame f = rg::build_map_view(cache, w.source(), sel, kE0, kN0);
    CHECK(f.failed_tiles == 0);
    CHECK(f.tiles == 9);
    std::set<std::int64_t> ways;
    for (const auto& l : f.lines) {
        ways.insert(l.way_id);
        CHECK(l.point_count >= 2);
        for (std::uint32_t k = 0; k < l.point_count; ++k) {
            const auto& p = f.points[l.point_first + k];
            CHECK(p[0] >= -0.01f);
            CHECK(p[0] <= 1600.01f);
            CHECK(p[1] >= -0.01f);
            CHECK(p[1] <= 1600.01f);
        }
    }
    CHECK(ways.size() == 18); // every grid way, the island is outside
    const auto main = std::find_if(f.lines.begin(), f.lines.end(), [](const auto& l) { return l.way_id == 104; });
    REQUIRE(main != f.lines.end());
    CHECK(main->name == "Hauptstrasse");
    CHECK(main->ref == "B 8");
    // A larger view switches to a coarser LOD and its level.
    const rg::MapViewSelection far = rg::select_map_view(config, kZone, kE0, kN0, 800.0, 800.0, 20000.0);
    CHECK(far.lod == g2m::route::MapLod::Far);
    CHECK(far.keys.front().level == 6);
    // Map detail x2 doubles the switch distance: 3 km is still Detail.
    rg::LodChoice c;
    c.map_detail = 2.0;
    CHECK(rg::select_map_view(config, kZone, kE0, kN0, 0, 0, 3000.0).lod == g2m::route::MapLod::Mid);
    CHECK(rg::select_map_view(rg::map_config_for(c), kZone, kE0, kN0, 0, 0, 3000.0).lod == g2m::route::MapLod::Detail);
}

TEST_CASE("map view: worker frames, no rebuild for an unchanged view, settings rebuild", "[route_planner][map_view]") {
    TiledWorld w(grid_world());
    rg::MapView view(w.source(), kZone, kE0, kN0);
    view.request(800.0, 800.0, 900.0);
    rg::MapViewFrame f;
    REQUIRE(view.wait(f));
    CHECK(f.revision == 1);
    CHECK(f.lines.size() >= 18);
    const std::size_t builds = view.tile_builds();
    CHECK(builds == 9);
    // Moving inside the same tiles: no new frame, nothing built.
    view.request(810.0, 790.0, 900.0);
    rg::MapViewFrame g;
    CHECK_FALSE(view.wait(g));
    CHECK(view.tile_builds() == builds);
    // An invalid config is refused; a valid map-detail change rebuilds the view.
    g2m::route::MapConfig bad = g2m::route::default_map_config();
    bad.detail_scale = 100.0;
    CHECK_FALSE(view.set_config(bad));
    rg::LodChoice c;
    c.map_detail = 4.0;
    REQUIRE(view.set_config(rg::map_config_for(c)));
    REQUIRE(view.wait(g));
    CHECK(g.revision == 2);
    CHECK(view.tile_builds() == builds + 9);
}
