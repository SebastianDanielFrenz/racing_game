// test_route_road_render_path.cpp — roads_plan.md R-4 owner-review follow-up
// (2026-09-27): R-3's test_route_road_coverage.cpp classifies a point by
// calling rg::WorldTerrain::road_segments() directly, single-threaded, on a
// single 1x1 g2m::Lattice - it never goes through the REAL render path
// (rg::select_view_keys/rg::build_render_chunks, the same functions
// RgTerrainView/TerrainViewStreamer call, at the SAME 8-worker thread count
// production uses) and so cannot catch a render-path-only bug. This test
// does: it opens one rg::WorldTerrain (matching a live Session, which keeps
// ONE WorldTerrain for the whole run across every relocate) and, on that
// SAME instance, repeatedly calls build_render_chunks for the four
// road_shots.gd target points in sequence (s=150/2870/6000/9800 along
// data/routes/home_r1_drive.json - same order road_shots.gd relocates
// through), at the real 8-thread cap, and checks whether the target's own
// L0 render chunk actually got road land_class painted - cross-checked
// against a known-good single-threaded rg::WorldTerrain::road_segments()
// call on a FRESH terrain instance for the same tile (ground truth: is
// there a road there at all, and how far is the target point from the
// nearest segment).
#include "rg/road_classes.h"
#include "rg/route_check.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/mesh/terrain_chunk.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {

std::optional<std::string> safe_getenv(const char* name) {
#ifdef _WIN32
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) {
        return std::nullopt;
    }
    std::string value(buf);
    std::free(buf);
    return value;
#else
    const char* value = std::getenv(name);
    return value != nullptr ? std::optional<std::string>(value) : std::nullopt;
#endif
}

// Same arc-length-walk convention as road_shots.gd's _point_at_s() / R-3's
// own arc-length pass: returns absolute UTM (e, n) at cumulative Euclidean
// arc length target_s along the route.
std::optional<std::pair<double, double>> point_at_s(const rg::Route& route, double target_s) {
    if (route.waypoints.size() < 2 || target_s < 0.0) {
        return std::nullopt;
    }
    double along = 0.0;
    for (std::size_t i = 1; i < route.waypoints.size(); ++i) {
        const double ax = route.waypoints[i - 1].x, ay = route.waypoints[i - 1].y;
        const double bx = route.waypoints[i].x, by = route.waypoints[i].y;
        const double seg = std::sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
        if (along + seg >= target_s || i + 1 == route.waypoints.size()) {
            const double t = seg <= 0.0 ? 0.0 : std::clamp((target_s - along) / seg, 0.0, 1.0);
            const double x = ax + (bx - ax) * t;
            const double y = ay + (by - ay) * t;
            return std::make_pair(route.e0 + x, route.n0 + y);
        }
        along += seg;
    }
    return std::nullopt;
}

// Nearest-segment distance (metres) from (e_m, n_m) to any RoadSegment in
// `segs` - clamped point-to-segment distance, plain double math (diagnostic
// only, not the exact-integer decision rasterize_road_segments itself uses).
double nearest_segment_distance_m(const std::vector<g2m::RoadSegment>& segs, double e_m, double n_m) {
    const double px = e_m * 1000.0, py = n_m * 1000.0; // mm, matching RoadSegment::a/b
    double best_mm = std::numeric_limits<double>::infinity();
    for (const g2m::RoadSegment& s : segs) {
        const double ax = static_cast<double>(s.a.x), ay = static_cast<double>(s.a.y);
        const double bx = static_cast<double>(s.b.x), by = static_cast<double>(s.b.y);
        const double dx = bx - ax, dy = by - ay;
        const double len2 = dx * dx + dy * dy;
        double t = len2 <= 0.0 ? 0.0 : ((px - ax) * dx + (py - ay) * dy) / len2;
        t = std::clamp(t, 0.0, 1.0);
        const double cx = ax + t * dx, cy = ay + t * dy;
        const double d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy);
        best_mm = std::min(best_mm, d2);
    }
    if (!std::isfinite(best_mm)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::sqrt(best_mm) / 1000.0;
}

// Level-0 ChunkKey covering absolute UTM (e_m, n_m) - terrain_chunk.h: chunk
// (0, cx, cy) covers samples [64*cx, 64*cx+64) of the level-0 lattice, i.e.
// [64*cx, 64*cx+64) METRES at level 0 (spacing 1 m/sample).
g2m::mesh::ChunkKey l0_chunk_at(g2m::geo::UtmZone zone, double e_m, double n_m) {
    return g2m::mesh::ChunkKey{zone, /*level=*/0, static_cast<std::int32_t>(std::floor(e_m / 64.0)),
                               static_cast<std::int32_t>(std::floor(n_m / 64.0))};
}

struct TargetResult {
    const char* name;
    double s;
    bool found_chunk = false;
    int road_vertices = 0;
    int total_vertices = 0;
    double distance_to_road_m = -1.0;
    std::uint64_t osm_ok_delta = 0;
    std::uint64_t osm_fail_delta = 0;
    std::uint64_t osm_cache_hits_delta = 0;
    double with_class_ms = 0.0;
    double without_class_ms = -1.0; // -1 = not measured this repeat (repeat > 0)
};

} // namespace

TEST_CASE("route road render path: the four road_shots.gd targets keep road land_class through the real, "
          "8-thread build_render_chunks path on one persistent WorldTerrain",
          "[.][realdata][terrain]") {
    if (!safe_getenv("RG_G2M_HOME").has_value()) {
        SKIP("RG_G2M_HOME not set");
    }
    std::string err;
    const auto world = rg::load_world_config(std::string(RG_SOURCE_DIR) + "/data/world/world_config.json", &err);
    INFO(err);
    REQUIRE(world.has_value());
    const auto route = rg::load_route(std::string(RG_SOURCE_DIR) + "/data/routes/home_r1_drive.json", &err);
    INFO(err);
    REQUIRE(route.has_value());

    const g2m::geo::UtmZone zone{route->zone};

    struct Target {
        const char* name;
        double s;
    };
    const std::vector<Target> targets = {
        {"05_road_b8_junction", 150.0},
        {"06_road_bridge", 2870.0},
        {"07_road_forest", 6000.0},
        {"08_road_konigstein", 9800.0},
    };

    // Ground truth (known-good, single-threaded, matching R-3 exactly): a
    // FRESH terrain per point so this reference never shares cache state
    // with the render-path pass below.
    for (const Target& t : targets) {
        std::unique_ptr<rg::WorldTerrain> ref_terrain = rg::WorldTerrain::open(*world, &err);
        INFO(err);
        REQUIRE(ref_terrain != nullptr);
        const auto point = point_at_s(*route, t.s);
        REQUIRE(point.has_value());
        const auto [e_m, n_m] = *point;
        const std::optional<g2m::TileKey> key = g2m::tile_key_at(zone, /*level=*/2, e_m, n_m);
        REQUIRE(key.has_value());
        const std::shared_ptr<const std::vector<g2m::RoadSegment>> segs = ref_terrain->road_segments(*key);
        REQUIRE(segs != nullptr);
        const double dist_m = nearest_segment_distance_m(*segs, e_m, n_m);
        std::printf("route_road_render_path: ground_truth %-22s s=%.0f segs_in_tile=%zu nearest_road_m=%.2f\n",
                    t.name, t.s, segs->size(), dist_m);
        INFO("ground truth nearest-road distance for " << t.name);
        CHECK(dist_m < 15.0); // rules out "relocate lands off the route/road" - a real B8/L-road is this wide or wider
    }

    // Real render path: ONE WorldTerrain (mirrors a live Session, which keeps
    // exactly one WorldTerrain across every relocate for the whole run),
    // build_render_chunks called in sequence for each target - same order
    // road_shots.gd relocates through - at the same thread cap production
    // uses (auto_initial_threads()/build_static_view's own formula).
    const unsigned hw = std::thread::hardware_concurrency();
    const unsigned threads = hw == 0 ? 4u : std::min(hw, 8u);
    std::printf("route_road_render_path: hardware_concurrency=%u build_threads=%u\n", hw, threads);

    constexpr int kRepeats = 3;
    for (int repeat = 0; repeat < kRepeats; ++repeat) {
        std::unique_ptr<rg::WorldTerrain> terrain = rg::WorldTerrain::open(*world, &err);
        INFO(err);
        REQUIRE(terrain != nullptr);
        const rg::TerrainViewSource source = terrain->view_source();

        // Prime a small initial view at the route's own spawn (waypoints[0]),
        // matching RgTerrainView::load_preview's own first call, before the
        // big relocate-sized jumps below - a cache/origin bug conditioned on
        // "was this the very first build" would show up only without this.
        {
            const double spawn_e = route->e0 + route->waypoints.front().x;
            const double spawn_n = route->n0 + route->waypoints.front().y;
            std::vector<g2m::mesh::ChunkKey> keys;
            rg::select_view_keys(spawn_e - source.e0, spawn_n - source.n0, source.params, source.e0, source.n0, keys);
            std::vector<rg::RenderChunk> chunks;
            REQUIRE(rg::build_render_chunks(keys, source.lookup, source.ctx, source.e0, source.n0, threads, chunks,
                                            nullptr, source.class_lookup));
        }

        std::vector<TargetResult> results;
        for (const Target& t : targets) {
            const auto point = point_at_s(*route, t.s);
            REQUIRE(point.has_value());
            const auto [e_m, n_m] = *point;

            const rg::WorldTerrain::OsmFetchStats before = terrain->osm_fetch_stats();

            std::vector<g2m::mesh::ChunkKey> keys;
            rg::select_view_keys(e_m - source.e0, n_m - source.n0, source.params, source.e0, source.n0, keys);
            std::vector<rg::RenderChunk> chunks;
            const auto t_with_start = std::chrono::steady_clock::now();
            REQUIRE(rg::build_render_chunks(keys, source.lookup, source.ctx, source.e0, source.n0, threads, chunks,
                                            nullptr, source.class_lookup));
            const double with_class_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_with_start).count();

            const rg::WorldTerrain::OsmFetchStats after = terrain->osm_fetch_stats();

            // Coordinator review (2026-09-27) task 2, "measure whether the road
            // raster/fetch runs on the main thread or per frame": it does
            // neither (see rg_terrain_view.cpp/terrain_view_streamer.cpp - road
            // classing only ever runs inside build_render_chunks, called once
            // per focus-change from TerrainViewStreamer's OWN background worker
            // thread, never from RgTerrainView::_process/the main thread, and
            // never every frame). What this DOES cost: a second build of the
            // SAME already-fetched (now cache-warm) keys with class_lookup
            // OMITTED isolates road classing's own steady-state CPU cost -
            // repeat 0 only, to keep this test's own runtime bounded.
            double without_class_ms = -1.0;
            if (repeat == 0) {
                std::vector<rg::RenderChunk> chunks_no_class;
                const auto t_without_start = std::chrono::steady_clock::now();
                REQUIRE(rg::build_render_chunks(keys, source.lookup, source.ctx, source.e0, source.n0, threads,
                                                chunks_no_class, nullptr, rg::ClassLookup{}));
                without_class_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                                              t_without_start)
                                        .count();
            }

            const g2m::mesh::ChunkKey want = l0_chunk_at(zone, e_m, n_m);
            TargetResult r;
            r.name = t.name;
            r.s = t.s;
            r.osm_ok_delta = after.ok - before.ok;
            r.osm_fail_delta = after.fail - before.fail;
            r.osm_cache_hits_delta = after.cache_hits - before.cache_hits;
            r.with_class_ms = with_class_ms;
            r.without_class_ms = without_class_ms;
            for (const rg::RenderChunk& c : chunks) {
                if (c.key == want) {
                    r.found_chunk = true;
                    r.total_vertices = static_cast<int>(c.mesh.land_class.size());
                    for (std::uint8_t lc : c.mesh.land_class) {
                        if (lc != static_cast<std::uint8_t>(g2m::LandClass::Unknown)) {
                            ++r.road_vertices;
                        }
                    }
                    break;
                }
            }
            results.push_back(r);
        }

        for (const TargetResult& r : results) {
            std::printf("route_road_render_path: repeat=%d %-22s s=%.0f found_chunk=%d road_vertices=%d/%d "
                        "osm_ok=+%llu osm_fail=+%llu osm_cache_hits=+%llu with_class_ms=%.1f without_class_ms=%.1f\n",
                        repeat, r.name, r.s, r.found_chunk ? 1 : 0, r.road_vertices, r.total_vertices,
                        static_cast<unsigned long long>(r.osm_ok_delta), static_cast<unsigned long long>(r.osm_fail_delta),
                        static_cast<unsigned long long>(r.osm_cache_hits_delta), r.with_class_ms, r.without_class_ms);
        }

        for (const TargetResult& r : results) {
            INFO("repeat=" << repeat << " target=" << r.name << " s=" << r.s);
            CHECK(r.found_chunk);
            CHECK(r.road_vertices > 0);
        }
    }
}
