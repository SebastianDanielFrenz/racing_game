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
#include "g2m/world/road_ribbons.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
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
                                                chunks_no_class, nullptr, g2m::world::ClassLookup{}));
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

namespace {

struct IncidentProbePoint {
    double e, n, z, heading_deg;
    int marker;
    double sim_time;
};

bool point_in_triangle(double x, double y, const g2m::world::RoadRibbonVertex& a,
                       const g2m::world::RoadRibbonVertex& b, const g2m::world::RoadRibbonVertex& c,
                       double expected_z) {
    const double abx=b.x-a.x, aby=b.y-a.y, acx=c.x-a.x, acy=c.y-a.y;
    const double area=abx*acy-aby*acx;
    if(std::abs(area)<1e-8) return false;
    const double apx=x-a.x, apy=y-a.y;
    const double u=(apx*acy-apy*acx)/area, v=(abx*apy-aby*apx)/area;
    const double w=1.0-u-v;
    if(u < -1e-5 || v < -1e-5 || w < -1e-5) return false;
    const double z=w*a.z+u*b.z+v*c.z;
    return std::abs(z-expected_z)<5.0;
}

bool paved_ribbon_at(const g2m::world::RoadRibbonTile& mesh, double e, double n, double z) {
    const double x=e-mesh.key.min_easting(), y=n-mesh.key.min_northing();
    for(std::size_t i=0;i+2<mesh.indices.size();i+=3) {
        const auto& a=mesh.vertices[static_cast<std::size_t>(mesh.indices[i])];
        const auto& b=mesh.vertices[static_cast<std::size_t>(mesh.indices[i+1])];
        const auto& c=mesh.vertices[static_cast<std::size_t>(mesh.indices[i+2])];
        const int flags=static_cast<int>(std::lround(a.flags));
        // Painted lines/arrows and the gravel verge are drawables too, but do
        // not represent the paved contact surface. Junction asphalt is emitted
        // with flags=0; unpaved pieces carry bit 8192.
        if((flags&16)!=0 || (flags&8192)!=0 || a.verge>0.5f || b.verge>0.5f || c.verge>0.5f) continue;
        if(point_in_triangle(x,y,a,b,c,z)) return true;
    }
    return false;
}

struct PhysicsRoadSample { bool paved=false; double nearest_paved_edge_m=1e9; };

PhysicsRoadSample physics_paved_at(rg::WorldTerrain& terrain, g2m::geo::UtmZone zone, double e, double n) {
    const auto key=g2m::tile_key_at(zone,2,e,n);
    if(!key) return {};
    const auto segs=terrain.road_segments(*key);
    if(!segs || segs->empty()) return {};
    const std::int64_t ec=std::llround(e), nc=std::llround(n); // physics cell centers are integer UTM metres
    const g2m::Lattice lattice{ec*1000,nc*1000,1000,1,1};
    std::uint8_t land=static_cast<std::uint8_t>(g2m::LandClass::Unknown);
    std::uint8_t surface=static_cast<std::uint8_t>(g2m::SurfaceKind::Unknown);
    g2m::rasterize_road_segments(*segs,lattice,g2m::RasterParams{.min_rank=0,.min_half_width_mm=710},
                                 std::span<std::uint8_t>(&land,1),std::span<std::uint8_t>(&surface,1));
    // The incident world's road-mode grip LUT maps every PavedRoad segment
    // to the asphalt surface id (including OSM paving-stone tags).
    PhysicsRoadSample sample;
    sample.paved=land==static_cast<std::uint8_t>(g2m::LandClass::PavedRoad);
    const double px=ec*1000.0,py=nc*1000.0;
    for(const auto& seg:*segs) {
        if(seg.land_class!=g2m::LandClass::PavedRoad) continue;
        const double ax=static_cast<double>(seg.a.x),ay=static_cast<double>(seg.a.y);
        const double bx=static_cast<double>(seg.b.x),by=static_cast<double>(seg.b.y);
        const double dx=bx-ax,dy=by-ay,len2=dx*dx+dy*dy;
        const double t=len2>0?std::clamp(((px-ax)*dx+(py-ay)*dy)/len2,0.0,1.0):0.0;
        const double d=std::hypot(px-(ax+t*dx),py-(ay+t*dy))/1000.0;
        sample.nearest_paved_edge_m=std::min(sample.nearest_paved_edge_m,
            std::max(0.0,d-std::max(710.0,static_cast<double>(seg.half_width_mm))/1000.0));
    }
    return sample;
}

bool served_class_paved_at(rg::WorldTerrain& terrain, g2m::geo::UtmZone zone, double e, double n) {
    const std::int64_t ec=std::llround(e), nc=std::llround(n);
    const auto key=g2m::tile_key_at(zone,0,static_cast<double>(ec),static_cast<double>(nc));
    if(!key) return false;
    const auto tile=terrain.terrain_class_tile(*key);
    if(!tile || tile->key!=*key) return false;
    const int i=static_cast<int>(ec-key->min_easting()),j=static_cast<int>(nc-key->min_northing());
    if(i<0||j<0||i>=g2m::kTerrainClassSamples||j>=g2m::kTerrainClassSamples) return false;
    return tile->land_class[g2m::TerrainClassTile::index(i,j)]==static_cast<std::uint8_t>(g2m::LandClass::PavedRoad);
}

} // namespace

TEST_CASE("corridor incident samples: served geometry classes reduce raw OSM grip/visual disagreement",
          "[.][realdata][terrain][road-parity]") {
    static constexpr IncidentProbePoint points[] = {
        {461770.841,5555456.676,251.235,-33.5,1,72.1}, {461814.334,5555427.904,250.565,-33.5,1,73.1},
        {461865.335,5555394.158,249.696,-33.5,1,74.1}, {461924.909,5555351.560,248.526,-33.5,1,75.1},
        {461983.939,5555304.668,247.229,-33.5,1,76.1},
        {465468.046,5552451.984,131.305,163.7,2,152.2}, {465447.418,5552458.038,131.527,163.7,2,153.2},
        {465420.467,5552465.849,131.842,163.7,2,154.2}, {465384.589,5552476.276,132.277,163.7,2,155.2},
        {465340.639,5552489.031,132.848,163.7,2,156.2},
        {463658.495,5553285.490,175.346,133.3,3,198.3}, {463603.260,5553344.065,177.036,133.3,3,199.3},
        {463552.796,5553407.267,178.742,133.3,3,200.3}, {463506.090,5553476.867,180.409,133.3,3,201.3},
        {463458.800,5553552.557,182.262,133.3,3,202.3},
        {464287.764,5552910.887,159.985,154.2,4,258.4}, {464238.977,5552934.536,161.273,154.2,4,259.4},
        {464182.720,5552962.556,162.708,154.2,4,260.4}, {464117.253,5552996.000,164.339,154.2,4,261.4},
        {464045.598,5553033.447,166.031,154.2,4,262.4},
        // Latest user-marked incident, sampled before the first split-grip
        // wheel state: asphalt at t=62..64, with the owner marker at 64.8667.
        {465269.234,5552517.079,133.976,160.9,5,62.1},
        {465229.483,5552530.844,134.631,160.9,5,63.1},
        {465181.770,5552547.229,135.473,160.9,5,64.1},
        {465147.723,5552559.482,136.119,160.9,5,64.8667},
        // Stationary marker at the Unterliederbach spawn before the user
        // reproduced a right pull; chassis heading is the configured spawn yaw.
        {466758.462,5551413.982,109.571,161.9,6,108.7542},
    };
    std::string err;
    auto config=rg::load_world_config(std::string(RG_SOURCE_DIR)+"/data/world/world_config_rhein_main_corridor.json",&err);
    INFO(err);
    REQUIRE(config.has_value());
    // The served class layer uses the same road geometry/cross-sections as
    // the visible ribbons; test it as a candidate shared source for tire grip.
    config->render_classes_served=true;
    // Avoid writes to the shared baked cache while a player may be using it.
    const auto temp=std::filesystem::temp_directory_path()/(
        "rg_tc_incident_road_parity_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    REQUIRE(std::filesystem::create_directory(temp));
    config->derived_store.dir=temp.string();
    auto terrain=rg::WorldTerrain::open(*config,&err);
    INFO(err);
    REQUIRE(terrain!=nullptr);
    const g2m::geo::UtmZone zone{32,g2m::geo::Hemisphere::North};
    int samples=0,mismatches=0,drawn_but_grass=0,grip_but_not_drawn=0,class_mismatches=0;
    for(const auto& p:points) {
        const auto key=g2m::tile_key_at(zone,2,p.e,p.n);
        REQUIRE(key.has_value());
        const auto ribbon=terrain->road_visual_tile(*key,0.018);
        const double h=p.heading_deg*3.14159265358979323846/180.0;
        const double nx=-std::sin(h),ny=std::cos(h);
        for(const double offset : {-1.4,-0.85,-0.8,0.0,0.8,0.825,1.4}) {
            const double e=p.e+nx*offset,n=p.n+ny*offset;
            // Physics selects its material from the integer-metre cell centre.
            // Compare the visible road at that same quantized contact point.
            const double cell_e=std::round(e),cell_n=std::round(n);
            const bool draw=paved_ribbon_at(ribbon,cell_e,cell_n,p.z-0.5);
            const PhysicsRoadSample physics=physics_paved_at(*terrain,zone,e,n);
            const bool grip=physics.paved;
            const bool class_grip=served_class_paved_at(*terrain,zone,e,n);
            ++samples;
            if(draw!=grip) {
                ++mismatches;
                if(draw) ++drawn_but_grass; else ++grip_but_not_drawn;
                std::printf("TC_ROAD_PARITY mismatch mark=%d t=%.1f offset=%+.1fm UTM=(%.3f,%.3f) drawn_paved=%d physics_paved=%d nearest_raw_paved_edge_m=%.2f ribbon_vertices=%zu\n",
                            p.marker,p.sim_time,offset,cell_e,cell_n,draw?1:0,grip?1:0,physics.nearest_paved_edge_m,ribbon.vertices.size());
            }
            if(draw!=class_grip) {
                ++class_mismatches;
                std::printf("TC_ROAD_CLASS mismatch mark=%d t=%.1f offset=%+.1fm UTM=(%.3f,%.3f) drawn_paved=%d served_class_paved=%d\n",
                            p.marker,p.sim_time,offset,cell_e,cell_n,draw?1:0,class_grip?1:0);
            }
        }
        if(p.marker==6) {
            static constexpr std::array<double,4> local_x{1.566,1.566,-1.134,-1.134};
            static constexpr std::array<double,4> local_y{0.85,-0.85,0.825,-0.825};
            static constexpr std::array<const char*,4> reported{"asphalt","grass","asphalt","asphalt"};
            const double c=std::cos(h),s=std::sin(h);
            for(std::size_t wheel=0;wheel<local_x.size();++wheel) {
                const double wx=p.e+c*local_x[wheel]-s*local_y[wheel];
                const double wy=p.n+s*local_x[wheel]+c*local_y[wheel];
                const double cell_e=std::round(wx),cell_n=std::round(wy);
                const bool draw=paved_ribbon_at(ribbon,cell_e,cell_n,p.z-0.5);
                const PhysicsRoadSample physics=physics_paved_at(*terrain,zone,wx,wy);
                const bool class_grip=served_class_paved_at(*terrain,zone,wx,wy);
                ++samples;
                if(draw!=physics.paved) ++mismatches;
                if(draw!=class_grip) ++class_mismatches;
                std::printf("TC_ROAD_WHEEL mark=6 wheel=%s offset=(%.3f,%.3f) cell=(%.0f,%.0f) reported=%s ribbon_paved=%d raw_physics_paved=%d served_class_paved=%d nearest_raw_paved_edge_m=%.2f\n",
                            std::array<const char*,4>{"FL","FR","RL","RR"}[wheel],local_x[wheel],local_y[wheel],cell_e,cell_n,
                            reported[wheel],draw?1:0,physics.paved?1:0,class_grip?1:0,physics.nearest_paved_edge_m);
            }
        }
    }
    std::printf("TC_ROAD_PARITY summary samples=%d raw_osm_mismatch=%d drawn_paved_physics_grass=%d physics_paved_not_drawn=%d served_class_mismatch=%d\n",
                samples,mismatches,drawn_but_grass,grip_but_not_drawn,class_mismatches);
    // The legacy raster is expected to disagree at the measured road-bed
    // boundaries; the shared geometry classes must materially reduce that
    // disagreement. Remaining class/ribbon differences are edge quantization
    // (mesh vertices sit at +0.5m while class/physics cells use integer nodes).
    CHECK(drawn_but_grass > 0);
    CHECK(grip_but_not_drawn == 0);
    CHECK(class_mismatches * 2 < mismatches);
    terrain.reset();
    std::error_code ec;
    std::filesystem::remove_all(temp,ec);
}
