// Ground ribbons under an overpass (owner report 2026-10-14, utm32n/2/457/5422, E 468936 N 5552913): the motorway
// ribbon beneath an overpass formed metre-scale spikes. Synthetic scene through the real pipeline: OSM -> geo2map
// roads.geom (served options) -> rg::complete_road_profiles -> rg::RoadSurfacePatch on L0 -> g2m::world::build_road_ribbons (geo2map, phase 3a).
#include "rg/road_surface.h"
#include "rg/road_structures.h"
#include "g2m/world/road_ribbons.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"
#include "g2m/core/geo/utm.h"
#include "g2m/layer/osm_data.h"
#include "g2m/layer/road_dem_dependencies.h"
#include "g2m/layer/road_dem_sampler.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_graph.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/world/height_sampler.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace {
using Tags = std::vector<std::pair<std::string, std::string>>;
const g2m::geo::UtmZone kZone{32};
constexpr double kMarkerE = 468936.064, kMarkerN = 5552913.578;
const g2m::TileKey kTile{kZone, 2, 457, 5422};

struct OverpassScene {
    double bridge_half_m = 25;      // OSM bridge way half length (along the overpass road)
    bool bridge_layer = true;       // layer=1 on the bridge way
    bool approach_layer = false;    // layer=1 on the approach ways as well
    double deck_rise_m = 7.5;       // embankment / deck above the motorway
    double heading_deg = 70;        // overpass road direction (motorway runs east-west)
};

struct Built {
    g2m::RoadGeomTile geometry;
    std::map<g2m::TileKey, std::shared_ptr<g2m::HeightTile>> raw, carved;
    std::function<double(double, double)> dem;
};

// Terrain: flat 100 m, the motorway at 100 m, the overpass road on an embankment that rises to deck_rise_m at the
// abutments (bridge_half_m from the crossing) and falls back to grade 150 m further out; 1:1.5 side slopes. Under the
// span the DEM is bare earth (100 m), as in DGM1.
double scene_height(const OverpassScene& s, double e, double n) {
    const double h = s.heading_deg * 3.14159265358979323846 / 180, ux = std::cos(h), uy = std::sin(h);
    const double along = (e - kMarkerE) * ux + (n - kMarkerN) * uy, across = std::abs(-(e - kMarkerE) * uy + (n - kMarkerN) * ux);
    const double a = std::abs(along);
    if (a < s.bridge_half_m) return 100;
    const double crest = s.deck_rise_m * std::clamp(1 - (a - s.bridge_half_m) / 150, 0.0, 1.0);
    const double top_half = 6;
    return 100 + std::max(0.0, crest - std::max(0.0, across - top_half) / 1.5);
}

std::unique_ptr<Built> build(const OverpassScene& s, const rg::RoadSeparation& separation = {}) {
    auto out = std::make_unique<Built>();
    out->dem = [s](double e, double n) { return scene_height(s, e, n); };
    g2m::geo::Utm utm;
    g2m::osm::OsmBuilder osm;
    std::int64_t next_node = 1;
    const auto node = [&](double e, double n) {
        const auto ll = utm.reverse(kZone, e, n);
        osm.add_node(next_node, static_cast<std::int32_t>(std::llround(ll.lat * 1e7)), static_cast<std::int32_t>(std::llround(ll.lon * 1e7)));
        return next_node++;
    };
    // Motorway: two one-way carriageways 7.5 m either side of the axis, 3 lanes, 600 m each way of the marker.
    for (int side : {-1, 1}) {
        std::vector<std::int64_t> refs;
        for (int i = -12; i <= 12; ++i) refs.push_back(node(kMarkerE + i * 50.0 * (side < 0 ? 1 : -1), kMarkerN + side * 7.5));
        osm.add_way(side < 0 ? 100 : 101, refs, {{"highway", "motorway"}, {"lanes", "3"}, {"oneway", "yes"}, {"ref", "A 66"}});
    }
    // Overpass: approach - bridge - approach, nodes every 10 m.
    const double h = s.heading_deg * 3.14159265358979323846 / 180, ux = std::cos(h), uy = std::sin(h);
    const auto at = [&](double t) { return node(kMarkerE + ux * t, kMarkerN + uy * t); };
    std::vector<std::int64_t> south, bridge, north;
    for (double t = -300; t < -s.bridge_half_m; t += 10) south.push_back(at(t));
    const auto a0 = at(-s.bridge_half_m); south.push_back(a0); bridge.push_back(a0);
    bridge.push_back(at(0));
    const auto a1 = at(s.bridge_half_m); bridge.push_back(a1); north.push_back(a1);
    for (double t = s.bridge_half_m + 10; t <= 300; t += 10) north.push_back(at(t));
    Tags road{{"highway", "secondary"}, {"lanes", "2"}};
    Tags approach = road; if (s.approach_layer) approach.push_back({"layer", "1"});
    Tags span = road; span.push_back({"bridge", "yes"}); if (s.bridge_layer) span.push_back({"layer", "1"});
    osm.add_way(200, south, approach);
    osm.add_way(201, bridge, span);
    osm.add_way(202, north, approach);
    auto data = osm.build(); REQUIRE(data);
    auto world = g2m::build_road_graph(*data, kZone); REQUIRE(world);

    // The deriver's inputs (geo2map tests/support/road_fixture.h road_geom_inputs, both discovery stages).
    auto tile = g2m::select_road_graph_tile(*world, kTile); REQUIRE(tile);
    auto dem_deps = g2m::road_dem_dependencies(tile->graph, kZone); REQUIRE(dem_deps);
    std::set<g2m::TileKey> dem_keys; for (const auto& d : *dem_deps) dem_keys.insert(d.key);
    auto deps = g2m::road_context_dependencies(tile->graph, kZone, &kTile); REQUIRE(deps);
    std::set<g2m::TileKey> graph_keys; for (const auto& d : *deps) graph_keys.insert(d.key);
    std::vector<g2m::RoadGraphTile> others;
    const auto merge = [&]() {
        others.clear();
        for (const auto& k : graph_keys) { auto t = g2m::select_road_graph_tile(*world, k); REQUIRE(t); others.push_back(std::move(*t)); }
        std::vector<const g2m::RoadGraph*> graphs{&tile->graph};
        for (const auto& t : others) graphs.push_back(&t.graph);
        auto m = g2m::merge_road_graphs(graphs); REQUIRE(m); return std::move(*m);
    };
    auto context = merge();
    auto bridges = g2m::road_bridge_dependencies(tile->graph, context, kZone, &kTile); REQUIRE(bridges);
    for (const auto& d : *bridges) (d.layer == g2m::kRoadGraphLayer ? graph_keys : dem_keys).insert(d.key);
    context = merge();
    std::vector<std::shared_ptr<g2m::HeightTile>> dem_storage;
    std::vector<const g2m::HeightTile*> views;
    const auto fill = [&](const g2m::TileKey& key) {
        auto t = std::make_shared<g2m::HeightTile>(); t->key = key;
        const double cell = static_cast<double>(key.size_m()) / 256;
        for (int j = 0; j < 256; ++j) for (int i = 0; i < 256; ++i)
            t->h[static_cast<std::size_t>(j) * 256 + i] = static_cast<std::int32_t>(std::llround(
                scene_height(s, key.min_easting() + (i + .5) * cell, key.min_northing() + (j + .5) * cell) * 256));
        return t;
    };
    for (const auto& k : dem_keys) { dem_storage.push_back(fill(k)); views.push_back(dem_storage.back().get()); }
    auto dem = g2m::RoadDemSampler::make(views); REQUIRE(dem);
    g2m::RoadProfileLimits limits; limits.reference.allow_curvature_exceptions = true;
    auto geom = g2m::build_road_geom_tile(*tile, *dem, limits, &context, g2m::builtin_road_rules_de(), g2m::served_road_completion_options());
    REQUIRE(geom);
    out->geometry = std::move(*geom);
    rg::complete_road_profiles(out->geometry, *dem);   // the game's completion step (WorldTerrain::produce_road_surface)
    const rg::RoadSurfacePatch patch(out->geometry, .06, separation);
    // L0 tiles within 400 m of the marker, carved by this L2 tile's patch.
    for (int dy = -2; dy <= 2; ++dy) for (int dx = -2; dx <= 2; ++dx) {
        auto key = *g2m::tile_key_at(kZone, 0, kMarkerE + dx * 256.0, kMarkerN + dy * 256.0);
        if (key.parent().parent() != kTile) continue;
        auto raw = fill(key); auto carved = std::make_shared<g2m::HeightTile>(*raw); patch.apply(*carved);
        out->raw[key] = raw; out->carved[key] = carved;
    }
    return out;
}

// Triangles as "x y z" lines (absolute UTM), three per triangle, for tools/road_mesh_plot.py.
void dump_mesh(const g2m::world::RoadRibbonTile& mesh, const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "w"); if (!f) return;
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
        for (int k = 0; k < 3; ++k) {
            const auto& v = mesh.vertices[static_cast<std::size_t>(mesh.indices[i + static_cast<std::size_t>(k)])];
            std::fprintf(f, "%.3f %.3f %.3f\n", kTile.min_easting() + v.x, kTile.min_northing() + v.y, v.z);
        }
    std::fclose(f);
}
struct SpikeStats { std::size_t triangles = 0, spiky = 0; double worst = 0, worst_e = 0, worst_n = 0; };
SpikeStats spikes(const g2m::world::RoadRibbonTile& mesh, double radius_m, double threshold_m) {
    SpikeStats s;
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const auto& a = mesh.vertices[static_cast<std::size_t>(mesh.indices[i])];
        const auto& b = mesh.vertices[static_cast<std::size_t>(mesh.indices[i + 1])];
        const auto& c = mesh.vertices[static_cast<std::size_t>(mesh.indices[i + 2])];
        const double e = kTile.min_easting() + (a.x + b.x + c.x) / 3.0, n = kTile.min_northing() + (a.y + b.y + c.y) / 3.0;
        if (std::hypot(e - kMarkerE, n - kMarkerN) > radius_m) continue;
        ++s.triangles;
        const double v = std::max({a.z, b.z, c.z}) - std::min({a.z, b.z, c.z});
        if (v > threshold_m) ++s.spiky;
        if (v > s.worst) { s.worst = v; s.worst_e = e; s.worst_n = n; }
    }
    return s;
}
}

// The variants: an OSM bridge way that ends beyond the motorway (25 m) and two that end over its outer carriageway
// (15 m, 10 m), the approaches with and without layer=1.
TEST_CASE("Overpass: the road under a short OSM bridge keeps its own surface", "[road_overpass]") {
    for (double half : {25.0, 15.0, 10.0}) for (bool alayer : {false, true}) {
        CAPTURE(half, alayer);
        OverpassScene s; s.bridge_half_m = half; s.approach_layer = alayer;
        auto b = build(s);
        const auto lookup = [&](const g2m::TileKey& k) -> const g2m::HeightTile* { auto it = b->carved.find(k); return it == b->carved.end() ? nullptr : it->second.get(); };
        const auto mesh = g2m::world::build_road_ribbons(b->geometry, kTile, [&](double e, double n) { return g2m::world::sample_height_bilinear(lookup, kZone, 0, 0, e, n); }, .018);
        const auto st = spikes(mesh, 300, 1.0);
        CAPTURE(st.spiky, st.worst, st.worst_e, st.worst_n);
        CHECK(st.triangles > 1000);
        CHECK(st.spiky == 0);
        CHECK(st.worst < 0.5);
        if (const char* dir = std::getenv("RG_OVERPASS_DUMP")) dump_mesh(mesh, std::string(dir) + "/scene_h" + std::to_string(static_cast<int>(half)) + "_l" + std::to_string(alayer) + ".tri");
        // Collision: the surface physics collides with (the carved L0 tiles, Jolt's triangulation) along lines across
        // each carriageway (centre and +-5.5 m) within 50-60 m of the overpass: smooth, no steps.
        for (int side : {-1, 1}) for (double off : {-5.5, 0.0, 5.5}) {
            double lo = 1e9, hi = -1e9, step = 0;
            std::optional<double> prev;
            for (double x = -60; x <= 50; x += .5) { // the L2 tile ends 56 m east of the marker
                const auto z = g2m::world::sample_height_exact(lookup, kZone, 0, 0, kMarkerE + x, kMarkerN + side * 7.5 + off);
                REQUIRE(z);
                lo = std::min(lo, *z); hi = std::max(hi, *z);
                if (prev) step = std::max(step, std::abs(*z - *prev));
                prev = z;
            }
            CAPTURE(side, off, lo, hi, step);
            CHECK(hi - lo < 1.5);  // the motorway's own fitted profile (a smooth ~1.2 m hump), not a step
            CHECK(step < 0.05); // before the fix: metre steps where the approach embankment met the carriageway
        }
        // The overpass keeps its clearance: the bridge deck stands well above the motorway.
        const auto decks = rg::build_road_decks(b->geometry);
        const auto span = std::find_if(decks.begin(), decks.end(), [](const auto& d) { return d->way_id == 201 && !d->inferred; });
        REQUIRE(span != decks.end());
        const auto top = rg::road_deck_height(**span, ((*span)->start_station + (*span)->end_station) / 2, 0);
        REQUIRE(top);
        const auto ground = g2m::world::sample_height_exact(lookup, kZone, 0, 0, kMarkerE, kMarkerN + 7.5);
        REQUIRE(ground);
        CHECK(*top - *ground > 5.0);
        // The short spans get an inferred deck on the approach that crosses the outer carriageway.
        if (half < 20) CHECK(std::any_of(decks.begin(), decks.end(), [](const auto& d) { return d->inferred && (d->way_id == 200 || d->way_id == 202); }));
    }
}

// Negative control: without grade separation (the behaviour before) the short spans spike the motorway ribbon.
TEST_CASE("Overpass: without grade separation the short span spikes", "[road_overpass]") {
    OverpassScene s; s.bridge_half_m = 10;
    rg::RoadSeparation off; off.clearance_m = 1e9; off.margin_m = 0; off.approach_reach_m = 0;
    auto b = build(s, off);
    const auto lookup = [&](const g2m::TileKey& k) -> const g2m::HeightTile* { auto it = b->carved.find(k); return it == b->carved.end() ? nullptr : it->second.get(); };
    const auto mesh = g2m::world::build_road_ribbons(b->geometry, kTile, [&](double e, double n) { return g2m::world::sample_height_bilinear(lookup, kZone, 0, 0, e, n); }, .018, off);
    const auto st = spikes(mesh, 300, 1.0);
    CAPTURE(st.worst);
    CHECK(st.spiky > 50);
    CHECK(st.worst > 2.0);
    double step = 0;
    for (int side : {-1, 1}) for (double off : {-5.5, 0.0, 5.5}) {
        std::optional<double> prev;
        for (double x = -60; x <= 50; x += .5) {
            const auto z = g2m::world::sample_height_exact(lookup, kZone, 0, 0, kMarkerE + x, kMarkerN + side * 7.5 + off);
            if (z && prev) step = std::max(step, std::abs(*z - *prev));
            prev = z;
        }
    }
    std::printf("no separation: ribbon spiky=%zu worst=%.3f, collision max step %.3f m per 0.5 m\n", st.spiky, st.worst, step);
    CHECK(step > 0.5); // the collision surface has the same steps
}

// The owner's location on real data (corridor world, utm32n/2/457/5422). Hidden; SKIPs without the store.
TEST_CASE("Overpass: real data at utm32n/2/457/5422", "[.][realdata][road_overpass]") {
    std::string err;
    const auto world = rg::load_world_config(std::string(RG_SOURCE_DIR) + "/data/world/world_config_rhein_main_corridor.json", &err);
    REQUIRE(world);
    if (!std::filesystem::exists(world->source_store.dir)) SKIP("no geo2map store at " + world->source_store.dir);
    std::shared_ptr<rg::WorldTerrain> terrain(rg::WorldTerrain::open(*world, &err));
    REQUIRE(terrain);
    const auto mesh = terrain->road_visual_tile(kTile, .018);
    const auto st = spikes(mesh, 300, 1.0);
    std::printf("realdata 457/5422: tris=%zu spiky=%zu worst=%.3f at E %.2f N %.2f\n", st.triangles, st.spiky, st.worst, st.worst_e, st.worst_n);
    if (const char* dir = std::getenv("RG_OVERPASS_DUMP")) dump_mesh(mesh, std::string(dir) + "/real_457_5422.tri");
    CHECK(st.triangles > 1000);
    CHECK(st.spiky == 0);
    // Collision at the reported worst point (E 468922.78, N 5552905.0): no metre step within 3 m.
    std::vector<std::shared_ptr<const g2m::HeightTile>> held;
    const auto lookup = [&](const g2m::TileKey& k) -> const g2m::HeightTile* { held.push_back(terrain->height_tile_shared(k).tile); return held.back().get(); };
    double lo = 1e9, hi = -1e9;
    for (double dx = -1.5; dx <= 1.5; dx += .5) for (double dy = -1.5; dy <= 1.5; dy += .5) {
        const auto z = g2m::world::sample_height_exact(lookup, kZone, 0, 0, 468922.78 + dx, 5552905.0 + dy);
        if (z) { lo = std::min(lo, *z); hi = std::max(hi, *z); }
    }
    CAPTURE(lo, hi);
    CHECK(hi - lo < 1.0);
}
