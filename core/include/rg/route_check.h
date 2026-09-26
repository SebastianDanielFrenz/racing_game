// rg/route_check.h — a drive route on the real terrain ("rg.route/1" JSON,
// data/routes/*.json) and the Godot-free check that it is drivable by R2.2's
// scripted autopilot (R2.2 plan section 3, "tools/terrain_drive": start at
// the spawn, >= 3 km, max grade <= 12 %, >= 12 physics-tile seam crossings,
// no NoData).
//
// Three layers, each usable on its own:
//  - load_route(): strict, exception-free JSON loader (same TOOL-019
//    allow_exceptions=false contract as load_world_config).
//  - sample_l0_height(): bilinear height at a session-local point from
//    g2m L0 HeightTiles (cell-registered, row 0 = south, 1/256 m units),
//    through any tile lookup - WorldTerrain::height_tile_shared on real data,
//    an in-memory map in tests.
//  - check_route(): the criteria over a waypoint polyline and any height
//    function (a plain lambda in tests), returning every measured number plus
//    the list of failed criteria.
//
// tools/route_check is a thin main over these; R5's [realdata] drive test
// calls the same functions. No Godot type anywhere (engine-neutral rule).
#pragma once

#include "rg/world_config.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace rg {

struct RoutePoint {
    double x = 0.0; // session-local metres east of session_origin_utm.e0
    double y = 0.0; // session-local metres north of session_origin_utm.n0
};

// "rg.route/1". Waypoints are in the SESSION frame of the world config named
// by session_origin_utm (which the loader reads so a caller can check it
// matches the WorldConfig in use - see route_matches_world()).
struct Route {
    std::string format; // always "rg.route/1" once loaded
    std::string name;
    int zone = 0;
    double e0 = 0.0;
    double n0 = 0.0;
    RoutePoint spawn;       // session-local; must equal the world config's spawn
    double spawn_yaw_deg = 0.0;
    std::vector<RoutePoint> waypoints; // >= 2; waypoints[0] must equal spawn
};

// Strict loader: returns nullopt and sets *err ("<path>: <problem>") on any
// malformed/missing field. Never throws.
std::optional<Route> load_route(const std::string& path, std::string* err);

// Physics-tile index of one session-local coordinate on the 255 m physics
// grid (g2m::phys::PhysicsTileGrid::index_for_local): floor((v - origin) /
// tile_size). Negative for points west/south of the grid origin.
std::int64_t phys_tile_index(double v, double tile_size_m = 255.0, double origin_m = 0.5);

// Number of physics-tile seam LINES the polyline crosses: per segment,
// |dix| + |diy| of the endpoints' physics-tile indices (exact, independent of
// any sampling step; a segment through a tile corner counts 2).
int count_seam_crossings(const std::vector<RoutePoint>& polyline, double tile_size_m = 255.0,
                         double origin_m = 0.5);

// Returns the L0 tile for `key`, or nullptr when it is missing/out of
// coverage (treated as NoData). The pointer must stay valid until the call
// that received it returns.
using L0TileLookupFn = std::function<const g2m::HeightTile*(const g2m::TileKey&)>;

// Bilinear height (metres) at session-local (x, y) from the four L0 cell
// centres around it (sample (i, j) of tile key sits at
// (min_easting + i + 0.5, min_northing + j + 0.5)); the four may come from up
// to four different tiles. nullopt when any of the four is NoData or its
// tile is missing. (Jolt triangulates the same cell-centre lattice; bilinear
// and the triangle split differ by far less than the grade/elevation
// resolution this is used for.)
std::optional<double> sample_l0_height(const L0TileLookupFn& lookup, g2m::geo::UtmZone zone, std::int64_t e0,
                                       std::int64_t n0, double x, double y);

struct RouteCheckParams {
    double sample_step_m = 1.0;   // arc-length sampling step for heights
    double grade_window_m = 10.0; // grade = |dh| over this much arc length (rounded to whole steps)
    double phys_tile_size_m = 255.0;
    double phys_origin_m = 0.5;
    // Criteria (R2.2 plan section 3, plus the autopilot's corner limit):
    double min_length_m = 3000.0;
    double max_grade = 0.12;
    int min_seam_crossings = 12;
    double min_corner_radius_m = 30.0;
    double start_tolerance_m = 0.01; // waypoints[0] vs the expected start (the spawn)
};

struct RouteCheckReport {
    double length_m = 0.0;
    int waypoint_count = 0;
    int sample_count = 0;
    // Grade over grade_window_m (windows with a NoData end are skipped).
    int grade_window_count = 0;
    double max_grade = 0.0;
    double max_grade_at_m = 0.0; // arc length of the steepest window's start
    double p99_grade = 0.0;      // nearest-rank 99th percentile over all windows
    double mean_abs_grade = 0.0;
    int seam_crossings = 0;
    int seam_crossings_x = 0; // north-south seam lines (ix changes)
    int seam_crossings_y = 0; // east-west seam lines (iy changes)
    double min_elevation_m = 0.0;
    double max_elevation_m = 0.0;
    int nodata_samples = 0;
    double first_nodata_at_m = -1.0; // arc length, -1 if none
    // Tightest corner: at interior waypoint k with turn angle theta,
    // radius = (min(l_in, l_out) / 2) / tan(theta / 2) - the arc tangent to
    // both segments at half the shorter one (R*cos(theta/2) for a regular
    // polyline inscribed in a circle of radius R - 0.4 % short at 10 deg
    // steps). Infinity when the route has no turns.
    double min_corner_radius_m = 0.0;
    int min_corner_waypoint = -1;
    double start_offset_m = 0.0;     // |waypoints[0] - expected start|
    double start_heading_deg = 0.0;  // first segment, math convention (0 = east, 90 = north)
    std::vector<std::string> failures; // one line per failed criterion; empty = pass

    [[nodiscard]] bool ok() const { return failures.empty(); }
};

// Height at a session-local point; nullopt = NoData.
using HeightAtFn = std::function<std::optional<double>(double x, double y)>;

// Samples the polyline every sample_step_m of arc length (plus its exact end
// point), measures everything in RouteCheckReport and evaluates the
// criteria. A polyline with < 2 points or zero length is a failure, not UB.
RouteCheckReport check_route(const std::vector<RoutePoint>& waypoints, const RoutePoint& expected_start,
                             const HeightAtFn& height_at, const RouteCheckParams& params = {});

// Multi-line, human-readable summary ("key=value" lines, then one "FAIL ..."
// line per failure, then "route_check: PASS"/"route_check: FAIL").
std::string format_route_report(const RouteCheckReport& report);

// Checks a loaded route against the world config's frame and spawn
// (zone/e0/n0 equal, route.spawn == (spawn_e - e0, spawn_n - n0) within
// tolerance_m, yaw equal). Returns an empty string when they match, else a
// description of the mismatch.
std::string route_matches_world(const Route& route, int zone, double e0, double n0, double spawn_e, double spawn_n,
                                double spawn_yaw_deg, double tolerance_m = 0.01);

class WorldTerrain;

// The whole check on real terrain, as tools/route_check and the [realdata]
// drive test run it: route_matches_world() against `world` (a mismatch is
// reported as a failure), then check_route() with expected_start =
// route.spawn and heights from sample_l0_height() over
// terrain.height_tile_shared() (level-0 tiles only).
RouteCheckReport check_route_on_world(const Route& route, const WorldConfig& world, WorldTerrain& terrain,
                                      const RouteCheckParams& params = {});

} // namespace rg
