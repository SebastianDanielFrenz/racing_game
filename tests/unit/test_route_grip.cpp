// test_route_grip.cpp — G2.5a-grip R-c acceptance (plan section 8): on the
// real committed home_r1_drive route, tyre grip (physics.road_surfaces, via
// G2mTerrainSource::fill_tile - the SAME per-cell resolution
// Session::setup_terrain wires up) agrees with what a driver would expect:
// on-route waypoints read paved (asphalt), a forest-stretch control 30 m off
// the route reads off_road (grass), and a real rg::Session relocated onto
// the route reads the same thing through its own wheels.
//
// Mirrors test_route_road_coverage.cpp's own waypoint/forest-control
// machinery (same route file, same forest window s=[5800,6200], same 30 m
// perpendicular-offset control-point construction) but classifies through
// the PHYSICS grip path (SurfaceId via fill_tile) instead of the render
// path's LandClass rasterizer.
//
// Exclusions (vault G2M-010/011): the B8 crosses three underpasses on decks
// that bare-earth DGM1 removes, at route arc length s ~= 2852, 4453 and
// 5261 m (2870 m is test_route_road_render_path.cpp's own "06_road_bridge"
// target - the same crossing). Right under/near one of them the underpassing
// way can read as road a real bridge deck is not, in either direction (an
// off-route control point, or a relocated car) - points within 40 m of any
// of the three (by arc length) are excluded from the off-route control and
// the Session-relocation checks below, and the exclusion count is reported.
// The on-route waypoint coverage check is NOT filtered: a real car on the
// route surface at those arc lengths is still ON a real, driveable surface
// (the underpassing road, not empty air), so it is expected to read paved.
//
// Real data only: hidden ([.]) and SKIPs unless RG_G2M_HOME is set, same
// pattern as every other [.][realdata] case in this suite - CI never reads
// cache/. Numbers are printed unconditionally (std::printf) so a direct run
// reports them even on a pass.
#include "rg/route_check.h"
#include "rg/session.h"
#include "rg/terrain_mode.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include "g2m/core/geo/session_frame.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/phys/height_tile_loader.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/resident_heights.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "ps/io/surface_table.h"
#include "ps/terrain/terrain_source.h"
#include "ps/types.h"
#include "ps/vehicle/wheel_state.h"
#include "ps/world/ids.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

// Portable getenv (std::getenv is deprecated by the Windows UCRT headers and
// rg_warnings is /WX - same pattern as every other realdata test in this suite).
std::optional<std::string> safe_getenv(const char* name) {
#ifdef _WIN32
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) return std::nullopt;
    std::string value(buf);
    std::free(buf);
    return value;
#else
    const char* value = std::getenv(name);
    return value != nullptr ? std::optional<std::string>(value) : std::nullopt;
#endif
}

// Vault G2M-010/011: the B8's three bare-earth-DGM1 underpasses, by route
// arc length (metres). 2870 m (test_route_road_render_path.cpp's own
// "06_road_bridge" target) is the middle one.
constexpr double kUnderpassS[] = {2852.0, 4453.0, 5261.0};
constexpr double kUnderpassExclusionM = 40.0;

bool near_underpass(double s) {
    for (const double u : kUnderpassS) {
        if (std::abs(s - u) < kUnderpassExclusionM) return true;
    }
    return false;
}

// Same arc-length walk as test_route_road_render_path.cpp/test_terrain_mode.cpp's
// own point_at_s().
std::optional<std::pair<double, double>> point_at_s(const rg::Route& route, double target_s) {
    if (route.waypoints.size() < 2 || target_s < 0.0) return std::nullopt;
    double along = 0.0;
    for (std::size_t i = 1; i < route.waypoints.size(); ++i) {
        const double ax = route.waypoints[i - 1].x, ay = route.waypoints[i - 1].y;
        const double bx = route.waypoints[i].x, by = route.waypoints[i].y;
        const double seg = std::sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
        if (along + seg >= target_s || i + 1 == route.waypoints.size()) {
            const double t = seg <= 0.0 ? 0.0 : std::clamp((target_s - along) / seg, 0.0, 1.0);
            return std::make_pair(route.e0 + ax + (bx - ax) * t, route.n0 + ay + (by - ay) * t);
        }
        along += seg;
    }
    return std::nullopt;
}

// Physics grip cell for an absolute UTM point, matching geo2map_engine's own
// include/g2m/phys/fill_physics_surfaces.h cell-centre placement EXACTLY:
// cell k of physics tile ix is centred at LOCAL integer metre
// ts*ix + k + 1, ts = grid.samples() - 1 (255) - the midpoint between DGM1-
// anchored height samples k and k+1 (physics_grid.h's own anchoring
// derivation) - so a real point at local coordinate v falls in cell
// k = floor(v - ts*ix - 0.5). cell_surface_ids is row-major, row stride ts
// (fill_physics_surfaces.h's own "row-major, row_stride == samples()-1").
struct PhysCell {
    g2m::phys::PhysTileIndex tile;
    int x = 0;
    int y = 0;
};

PhysCell physics_cell_for(const g2m::phys::PhysicsTileGrid& grid, double x_local, double y_local) {
    const double ts = grid.tile_size_m(); // 255.0
    const g2m::phys::PhysTileIndex idx = grid.index_for_local(x_local, y_local);
    PhysCell cell;
    cell.tile = idx;
    cell.x = static_cast<int>(std::floor(x_local - ts * static_cast<double>(idx.ix) - 0.5));
    cell.y = static_cast<int>(std::floor(y_local - ts * static_cast<double>(idx.iy) - 0.5));
    return cell;
}

// Caches one filled ps::terrain::TileSample per physics tile (fetch + install
// + G2mTerrainSource::fill_tile once, reused by every point that lands in the
// same tile - a route's waypoints cluster into a modest number of physics
// tiles, so this keeps a multi-thousand-point sweep from re-fetching/
// re-filling the same tile over and over).
class SurfaceGrid {
public:
    SurfaceGrid(g2m::phys::PhysicsTileGrid grid, std::shared_ptr<g2m::phys::IHeightTileFetch> fetch,
                g2m::ps_bridge::RoadSurfaceConfig road)
        : grid_(grid), fetch_(std::move(fetch)), resident_(std::make_shared<g2m::phys::ResidentHeightSet>()),
          source_(grid_, resident_, std::move(road)) {}

    // Absolute UTM (e_m, n_m) -> the physics grip SurfaceId there, or
    // kInvalidSurfaceId (counted) if the covering tile could not be filled.
    ps::SurfaceId surface_at(double e_m, double n_m) {
        const double x_local = e_m - grid_.frame().e0_m();
        const double y_local = n_m - grid_.frame().n0_m();
        const PhysCell cell = physics_cell_for(grid_, x_local, y_local);
        const int ts = grid_.samples() - 1;
        if (cell.x < 0 || cell.x >= ts || cell.y < 0 || cell.y >= ts) {
            ++out_of_range_;
            return ps::kInvalidSurfaceId;
        }
        const std::int64_t packed = g2m::phys::PhysicsTileGrid::pack(cell.tile);
        auto it = tiles_.find(packed);
        if (it == tiles_.end()) {
            std::vector<g2m::TileKey> l0_keys;
            grid_.l0_keys_for_square(cell.tile, 0, l0_keys);
            bool ok = true;
            for (const g2m::TileKey& key : l0_keys) {
                const g2m::phys::FetchResult fr = fetch_->fetch(key);
                if (fr.status != g2m::Status::Ok || fr.tile == nullptr) {
                    ok = false;
                    continue;
                }
                resident_->install(key, fr.tile, fr.roads);
            }
            ps::terrain::TileSample sample;
            sample.heights.assign(256u * 256u, 0.0);
            sample.cell_surface_ids.assign(255u * 255u, ps::kInvalidSurfaceId);
            source_.fill_tile(ps::terrain::TileKey{packed}, sample);
            if (!ok) ++fetch_failures_;
            it = tiles_.emplace(packed, std::move(sample)).first;
        }
        return it->second.cell_surface_ids[static_cast<std::size_t>(cell.y) * static_cast<std::size_t>(ts) +
                                           static_cast<std::size_t>(cell.x)];
    }

    [[nodiscard]] std::uint64_t fill_miss_count() const { return source_.fill_miss_count(); }
    [[nodiscard]] std::uint64_t fetch_failures() const { return fetch_failures_; }
    [[nodiscard]] std::uint64_t out_of_range() const { return out_of_range_; }
    [[nodiscard]] std::size_t tiles_filled() const { return tiles_.size(); }

private:
    g2m::phys::PhysicsTileGrid grid_;
    std::shared_ptr<g2m::phys::IHeightTileFetch> fetch_;
    std::shared_ptr<g2m::phys::ResidentHeightSet> resident_;
    g2m::ps_bridge::G2mTerrainSource source_;
    std::map<std::int64_t, ps::terrain::TileSample> tiles_;
    std::uint64_t fetch_failures_ = 0;
    std::uint64_t out_of_range_ = 0;
};

// Cumulative arc length per waypoint - same as test_route_road_coverage.cpp's
// own `s` array.
std::vector<double> arc_lengths(const rg::Route& route) {
    std::vector<double> s(route.waypoints.size(), 0.0);
    for (std::size_t i = 1; i < route.waypoints.size(); ++i) {
        const double dx = route.waypoints[i].x - route.waypoints[i - 1].x;
        const double dy = route.waypoints[i].y - route.waypoints[i - 1].y;
        s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    return s;
}

} // namespace

TEST_CASE("route grip: home_r1_drive's waypoints are >= 99% asphalt via fill_tile; a forest-stretch control "
          "30 m off stays >= 90% grass",
          "[.][realdata][grip]") {
    if (!safe_getenv("RG_G2M_HOME").has_value()) SKIP("RG_G2M_HOME not set");
    std::string err;
    const auto world = rg::load_world_config(std::string(RG_SOURCE_DIR) + "/data/world/world_config.json", &err);
    INFO(err);
    REQUIRE(world.has_value());
    REQUIRE(world->physics.road_surfaces.enabled);
    const auto route = rg::load_route(std::string(RG_SOURCE_DIR) + "/data/routes/home_r1_drive.json", &err);
    INFO(err);
    REQUIRE(route.has_value());
    std::shared_ptr<rg::WorldTerrain> terrain(rg::WorldTerrain::open(*world, &err));
    INFO(err);
    REQUIRE(terrain != nullptr);

    const rg::TerrainModeConfig tm = rg::make_terrain_mode(*world, terrain);
    REQUIRE(tm.fetch != nullptr);
    REQUIRE(tm.fetch->provides_roads());

    const ps::io::SurfaceTable surfaces(std::string(RG_SOURCE_DIR) +
                                        "/external/physics_sim/data/surfaces/surfaces.json");
    const ps::SurfaceId paved = surfaces.id_for(world->physics.road_surfaces.paved);
    const ps::SurfaceId unpaved = surfaces.id_for(world->physics.road_surfaces.unpaved);
    const ps::SurfaceId off_road = surfaces.id_for(world->physics.road_surfaces.off_road);
    REQUIRE(paved != ps::kInvalidSurfaceId);
    REQUIRE(unpaved != ps::kInvalidSurfaceId);
    REQUIRE(off_road != ps::kInvalidSurfaceId);
    const g2m::RoadValueLut lut = g2m::RoadValueLut::by_land_class(
        off_road, {{g2m::LandClass::PavedRoad, paved}, {g2m::LandClass::UnpavedRoad, unpaved}});

    SurfaceGrid grid(g2m::phys::PhysicsTileGrid(tm.frame), tm.fetch, g2m::ps_bridge::RoadSurfaceConfig{lut, off_road});

    const double e0 = route->e0;
    const double n0 = route->n0;

    // --- (a) every waypoint (waypoints[0] == spawn, included), unfiltered.
    int total = 0;
    int paved_count = 0;
    for (const rg::RoutePoint& wp : route->waypoints) {
        ++total;
        if (grid.surface_at(e0 + wp.x, n0 + wp.y) == paved) ++paved_count;
    }
    REQUIRE(total > 0);
    const double coverage_pct = 100.0 * static_cast<double>(paved_count) / static_cast<double>(total);
    std::printf("route_grip: waypoints=%d paved=%d coverage=%.2f%% (fetch failures %llu, out of range %llu, "
                "tiles filled %zu)\n",
                total, paved_count, coverage_pct, static_cast<unsigned long long>(grid.fetch_failures()),
                static_cast<unsigned long long>(grid.out_of_range()), grid.tiles_filled());

    // --- (b) the forest-stretch control: points 30 m off the route, along
    // waypoints whose arc length falls in s=[5800,6200] (same window as
    // test_route_road_coverage.cpp), perpendicular to the local tangent.
    // None of this window's arc lengths are near an underpass (2852/4453/
    // 5261), but the exclusion is still applied on principle (and reported).
    const std::vector<double> s = arc_lengths(*route);
    constexpr double kForestCentreM = 6000.0;
    constexpr double kForestHalfWindowM = 200.0;
    constexpr double kOffsetM = 30.0;
    int control_total = 0;
    int control_grass_count = 0;
    int control_excluded = 0;
    for (std::size_t i = 1; i + 1 < route->waypoints.size(); ++i) {
        if (s[i] < kForestCentreM - kForestHalfWindowM || s[i] > kForestCentreM + kForestHalfWindowM) continue;
        if (near_underpass(s[i])) {
            ++control_excluded;
            continue;
        }
        const double tx = route->waypoints[i + 1].x - route->waypoints[i - 1].x;
        const double ty = route->waypoints[i + 1].y - route->waypoints[i - 1].y;
        const double len = std::sqrt(tx * tx + ty * ty);
        if (len <= 1e-6) continue; // degenerate (duplicate) waypoint pair
        const double px = -ty / len;
        const double py = tx / len;
        const double ox = route->waypoints[i].x + kOffsetM * px;
        const double oy = route->waypoints[i].y + kOffsetM * py;
        ++control_total;
        if (grid.surface_at(e0 + ox, n0 + oy) == off_road) ++control_grass_count;
    }
    REQUIRE(control_total > 0);
    const double control_pct =
        100.0 * static_cast<double>(control_grass_count) / static_cast<double>(control_total);
    std::printf("route_grip: forest_control_points=%d grass=%d control=%.2f%% (window s=[%.0f, %.0f], excluded "
                "near underpass %d)\n",
                control_total, control_grass_count, control_pct, kForestCentreM - kForestHalfWindowM,
                kForestCentreM + kForestHalfWindowM, control_excluded);

    INFO("coverage_pct=" << coverage_pct << " control_pct=" << control_pct);
    CHECK(grid.fill_miss_count() == 0);
    CHECK(coverage_pct >= 99.0);
    CHECK(control_pct >= 90.0);
}

TEST_CASE("route grip: a Session relocated onto the route reads paved; 30 m off reads off_road",
          "[.][realdata][grip]") {
    if (!safe_getenv("RG_G2M_HOME").has_value()) SKIP("RG_G2M_HOME not set");
    std::string err;
    const auto world = rg::load_world_config(std::string(RG_SOURCE_DIR) + "/data/world/world_config.json", &err);
    INFO(err);
    REQUIRE(world.has_value());
    const auto route = rg::load_route(std::string(RG_SOURCE_DIR) + "/data/routes/home_r1_drive.json", &err);
    INFO(err);
    REQUIRE(route.has_value());
    const std::vector<double> s = arc_lengths(*route);
    const double route_length_m = s.back();

    std::shared_ptr<rg::WorldTerrain> terrain(rg::WorldTerrain::open(*world, &err));
    INFO(err);
    REQUIRE(terrain != nullptr);
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json";
    config.terrain = rg::make_terrain_mode(*world, terrain);
    config.terrain->physics.max_tile_fills_per_tick = 32; // faster streaming between relocations
    rg::Session session(config);
    // No drive script, no set_control: every channel defaults to 0
    // (neutral/no throttle/no brake), so a relocated, settled car just sits
    // there - exactly what reading its wheels' resting surface needs.

    auto wheel_surfaces = [&](double e_m, double n_m, double yaw_rad) {
        const std::uint64_t reloc_before = session.streaming_status().relocations;
        const std::uint64_t fail_before = session.streaming_status().relocate_failures;
        session.request_relocate(e_m - config.terrain->frame.e0_m(), n_m - config.terrain->frame.n0_m(), yaw_rad);
        // session.step() itself blocks (internally retrying the streaming
        // gate every 1 ms) until the tick that completes the relocation
        // steps, or throws after physics.startup_timeout_s - see
        // rg::Session::step()/request_relocate()'s own doc comments. One
        // call is enough to land the car at the new target.
        session.step();
        const rg::StreamingStatus st = session.streaming_status();
        REQUIRE(st.relocations == reloc_before + 1);
        REQUIRE(st.relocate_failures == fail_before); // ground was found under the target
        for (int k = 0; k < 240; ++k) session.step(); // settle (suspension compresses onto the new surface)
        // A settled car on rough natural (DGM1) terrain, unlike a flat paved
        // strip, still has a small residual suspension bounce - a single
        // tick can catch one wheel's contact shape-cast between touches and
        // read kInvalidSurfaceId for that instant alone. Majority-vote each
        // wheel's surface over the next 20 ticks (~80 ms) instead of trusting
        // one snapshot; a wheel that is genuinely airborne the whole window
        // still reports kInvalidSurfaceId (no false positive).
        constexpr int kSampleTicks = 20;
        std::array<std::unordered_map<ps::SurfaceId, int>, 4> counts;
        for (int k = 0; k < kSampleTicks; ++k) {
            session.step();
            for (unsigned w = 0; w < 4; ++w) {
                const ps::vehicle::WheelState ws = session.world().wheel_state(session.vehicle_id(), w);
                ++counts[w][ws.surface];
            }
        }
        std::vector<std::string> names;
        for (unsigned w = 0; w < 4; ++w) {
            ps::SurfaceId best = ps::kInvalidSurfaceId;
            int best_count = -1;
            for (const auto& [id, count] : counts[w]) {
                if (count > best_count) {
                    best_count = count;
                    best = id;
                }
            }
            names.push_back(session.surface_table().name_for(best));
        }
        return names;
    };

    // On-route sample points, spread across the whole route, excluding
    // anything within 40 m (arc length) of the three known underpasses.
    int on_route_checked = 0, on_route_paved = 0, on_route_excluded = 0;
    for (double target_s = 150.0; target_s < route_length_m - 50.0; target_s += 1500.0) {
        if (near_underpass(target_s)) {
            ++on_route_excluded;
            continue;
        }
        const auto point = point_at_s(*route, target_s);
        if (!point.has_value()) continue;
        const auto [e_m, n_m] = *point;
        const std::vector<std::string> names = wheel_surfaces(e_m, n_m, 0.0);
        ++on_route_checked;
        int paved_wheels = 0;
        for (const std::string& n : names) {
            if (n == world->physics.road_surfaces.paved) ++paved_wheels;
        }
        std::printf("route_grip: session on-route s=%.0f wheels=[%s,%s,%s,%s]\n", target_s, names[0].c_str(),
                    names[1].c_str(), names[2].c_str(), names[3].c_str());
        if (paved_wheels == 4) ++on_route_paved;
    }
    REQUIRE(on_route_checked > 0);

    // Off-route sample points, 30 m off the route inside the already-
    // validated forest window s=[5800,6200] (test_route_road_coverage.cpp's
    // own control stretch) - none of these arc lengths are near an
    // underpass, but the exclusion is still applied and reported.
    int off_route_checked = 0, off_route_grass = 0, off_route_excluded = 0;
    for (double target_s = 5850.0; target_s <= 6150.0; target_s += 75.0) {
        if (near_underpass(target_s)) {
            ++off_route_excluded;
            continue;
        }
        const auto a = point_at_s(*route, target_s - 5.0);
        const auto b = point_at_s(*route, target_s + 5.0);
        const auto p = point_at_s(*route, target_s);
        if (!a.has_value() || !b.has_value() || !p.has_value()) continue;
        const double tx = b->first - a->first, ty = b->second - a->second;
        const double len = std::sqrt(tx * tx + ty * ty);
        if (len <= 1e-6) continue;
        const double px = -ty / len, py = tx / len;
        constexpr double kOffsetM = 30.0;
        const double ox = p->first + kOffsetM * px, oy = p->second + kOffsetM * py;
        const std::vector<std::string> names = wheel_surfaces(ox, oy, 0.0);
        ++off_route_checked;
        int grass_wheels = 0;
        for (const std::string& n : names) {
            if (n == world->physics.road_surfaces.off_road) ++grass_wheels;
        }
        std::printf("route_grip: session off-route s=%.0f wheels=[%s,%s,%s,%s]\n", target_s, names[0].c_str(),
                    names[1].c_str(), names[2].c_str(), names[3].c_str());
        // Unlike the flat paved strip, real forest DGM1 terrain has small-
        // scale relief: at s=5925/6000 exactly one of the four wheels
        // (measured, reproducible across a 2.5 s settle and a 20-tick
        // majority vote - not a one-tick blip) sits over a local dip beyond
        // its suspension travel while the other three, and the chassis
        // itself, are squarely on grass. >= 3 of 4 wheels is the right bar
        // here (a genuinely off-road car, not a flicker) - the on-route
        // paved strip above is flat enough to hold every point to all 4.
        if (grass_wheels >= 3) ++off_route_grass;
    }
    REQUIRE(off_route_checked > 0);

    std::printf("route_grip: session on-route %d/%d all-4-wheels-paved (excluded near underpass %d); off-route "
                "%d/%d at-least-3-wheels-grass (excluded near underpass %d)\n",
                on_route_paved, on_route_checked, on_route_excluded, off_route_grass, off_route_checked,
                off_route_excluded);
    CHECK(on_route_paved == on_route_checked);
    CHECK(off_route_grass == off_route_checked);
}
