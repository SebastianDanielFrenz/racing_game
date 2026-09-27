// test_route_road_coverage.cpp — roads_plan.md R-3 acceptance: on the real
// committed home_r1_drive route (2103 waypoints per its own "source" field,
// measured by tools/route_check against the real store), >= 99% of waypoints
// classify as road at the L0 level policy (every road, roads_plan.md R-2:
// road_raster_params_for_level(0) == RasterParams{min_rank=0,
// min_half_width_mm=0}), spawn (waypoints[0]) included. Control: points 30 m
// off the route's forest stretch around s ~ 6000 m (the same reference point
// R-4's forest screenshot uses) are road less than 10% of the time - a sanity
// check that the classifier is not simply saying "yes" everywhere.
//
// Real data only: hidden ([.]) and SKIPs unless RG_G2M_HOME is set, same
// pattern as test_route_check.cpp's own [.][realdata] case - CI never reads
// cache/. Numbers are printed unconditionally (std::printf, not just INFO) so
// a direct run of the test binary reports them even on a pass.
#include "rg/route_check.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_roads.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

// Portable getenv (std::getenv is deprecated by the Windows UCRT headers and
// rg_warnings is /WX - same pattern as test_route_check.cpp/test_world_config.cpp).
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

// Classifies one absolute UTM point (metres) at the L0 level policy (every
// road, roads_plan.md R-2): fetches the overlapping g2m.src.osm level-2 tile
// via WorldTerrain::road_segments (cached, never fatal on a fetch failure -
// "no roads" comes back as an empty segment list, which classifies as not
// road, same as the render path's own null-lookup contract) and rasterizes a
// single-sample lattice at the point.
bool point_is_road(rg::WorldTerrain& terrain, g2m::geo::UtmZone zone, double e_m, double n_m) {
    const std::optional<g2m::TileKey> key = g2m::tile_key_at(zone, /*level=*/2, e_m, n_m);
    if (!key.has_value()) {
        return false;
    }
    const std::shared_ptr<const std::vector<g2m::RoadSegment>> segs = terrain.road_segments(*key);
    if (!segs || segs->empty()) {
        return false;
    }
    const std::int64_t e_mm = static_cast<std::int64_t>(std::llround(e_m * 1000.0));
    const std::int64_t n_mm = static_cast<std::int64_t>(std::llround(n_m * 1000.0));
    const g2m::Lattice lattice{e_mm, n_mm, /*spacing_mm=*/1, /*nx=*/1, /*ny=*/1};
    std::vector<std::uint8_t> land_class(1, static_cast<std::uint8_t>(g2m::LandClass::Unknown));
    std::vector<std::uint8_t> surface(1, static_cast<std::uint8_t>(g2m::SurfaceKind::Unknown));
    const g2m::RasterParams params{/*min_rank=*/0, /*min_half_width_mm=*/0}; // L0: every road, no width floor
    g2m::rasterize_road_segments(*segs, lattice, params, land_class, surface);
    return land_class[0] != static_cast<std::uint8_t>(g2m::LandClass::Unknown);
}

} // namespace

TEST_CASE("route road coverage: >= 99% of home_r1_drive's waypoints are road at L0, spawn included; "
          "a forest-stretch control stays under 10%",
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
    std::unique_ptr<rg::WorldTerrain> terrain = rg::WorldTerrain::open(*world, &err);
    INFO(err);
    REQUIRE(terrain != nullptr);

    const g2m::geo::UtmZone zone{route->zone};
    const double e0 = route->e0;
    const double n0 = route->n0;

    // Coverage over every waypoint (waypoints[0] == route->spawn, included).
    int total = 0;
    int road_count = 0;
    for (const rg::RoutePoint& wp : route->waypoints) {
        ++total;
        if (point_is_road(*terrain, zone, e0 + wp.x, n0 + wp.y)) {
            ++road_count;
        }
    }
    REQUIRE(total > 0);
    const double coverage_pct = 100.0 * static_cast<double>(road_count) / static_cast<double>(total);
    std::printf("route_road_coverage: waypoints=%d road=%d coverage=%.2f%%\n", total, road_count, coverage_pct);

    // Control: points 30 m off the route, sampled along the waypoints whose
    // cumulative arc length falls in the forest stretch around s ~ 6000 m
    // (roads_plan.md R-3/R-4's shared reference point - the same stretch R-4's
    // forest screenshot relocates to). Offset perpendicular to the local
    // tangent (central difference over the neighbouring waypoints).
    std::vector<double> s(route->waypoints.size(), 0.0);
    for (std::size_t i = 1; i < route->waypoints.size(); ++i) {
        const double dx = route->waypoints[i].x - route->waypoints[i - 1].x;
        const double dy = route->waypoints[i].y - route->waypoints[i - 1].y;
        s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    constexpr double kForestCentreM = 6000.0;
    constexpr double kForestHalfWindowM = 200.0;
    constexpr double kOffsetM = 30.0;
    int control_total = 0;
    int control_road_count = 0;
    for (std::size_t i = 1; i + 1 < route->waypoints.size(); ++i) {
        if (s[i] < kForestCentreM - kForestHalfWindowM || s[i] > kForestCentreM + kForestHalfWindowM) {
            continue;
        }
        const double tx = route->waypoints[i + 1].x - route->waypoints[i - 1].x;
        const double ty = route->waypoints[i + 1].y - route->waypoints[i - 1].y;
        const double len = std::sqrt(tx * tx + ty * ty);
        if (len <= 1e-6) {
            continue; // degenerate (duplicate) waypoint pair - skip, not UB
        }
        // Perpendicular (rotate tangent +90 deg): (-ty, tx) / len.
        const double px = -ty / len;
        const double py = tx / len;
        const double ox = route->waypoints[i].x + kOffsetM * px;
        const double oy = route->waypoints[i].y + kOffsetM * py;
        ++control_total;
        if (point_is_road(*terrain, zone, e0 + ox, n0 + oy)) {
            ++control_road_count;
        }
    }
    REQUIRE(control_total > 0);
    const double control_pct = 100.0 * static_cast<double>(control_road_count) / static_cast<double>(control_total);
    std::printf("route_road_coverage: forest_control_points=%d road=%d control=%.2f%% (window s=[%.0f, %.0f])\n",
                control_total, control_road_count, control_pct, kForestCentreM - kForestHalfWindowM,
                kForestCentreM + kForestHalfWindowM);

    INFO("coverage_pct=" << coverage_pct << " control_pct=" << control_pct);
    CHECK(coverage_pct >= 99.0);
    CHECK(control_pct < 10.0);
}
