// rg/world_config.h — rg::WorldConfig: strict loader for a "rg.world/1"
// world-config JSON file (data/world/world_config.json, PLAN.md R2.0). Names
// the region, the session's UTM origin, the geo2map_engine source/derived
// stores and the spawn point a real-terrain session (R2+) is built from.
// Engine-neutral (no Godot type), same rule as session.h.
#pragma once
#include "rg/terrain_smoothing.h"

#include <cstdint>
#include <optional>
#include <string>

namespace rg {

struct WorldConfig {
    struct UtmOrigin {
        int zone = 0;    // 1..60 (g2m::geo::UtmZone's own range)
        double e0 = 0.0; // metres
        double n0 = 0.0; // metres
    };

    struct SourceStore {
        std::string dir; // ${VAR}-expanded; NOT resolved against the repo root (an external cache/store path)
        std::string scope;
        bool read_only = false;
    };

    struct DerivedStore {
        std::string dir; // ${VAR}-expanded; NOT resolved against the repo root
        std::string name;
    };

    struct Spawn {
        double e = 0.0;
        double n = 0.0;
        double yaw_deg = 0.0; // heading of the vehicle's forward (+x) axis: 0 = east (+e), counter-clockwise, 90 = north (a rotation about +Z)
    };

    // Optional top-level "lod" object (PLAN.md R2.1: "make max_distance a
    // config value ... not a hard-coded constant"). Absent entirely -> every
    // field keeps g2m::mesh::LodParams's own default (max_distance_m =
    // 6000.0) - a pre-R2.1 world_config.json with no "lod" key still loads
    // unchanged. rg::WorldTerrain::open reads this into its LodParams'
    // max_distance_m; range0_m/max_level are not yet config-exposed (R2.1's
    // own measurement only swept max_distance_m - see the repo CLAUDE.md's
    // LOD-distance table).
    struct Lod {
        double max_distance_m = 6000.0;
    };

    // Optional top-level "physics" object (R2.2 plan section 3): the terrain
    // streaming gate's own tuning knobs - how far around the vehicle tiles
    // must be resident (radius_m), which data/surfaces/surfaces.json entry a
    // real-terrain vehicle's tyres see (terrain_surface), how many tile
    // fills a tick may spend catching up (max_tile_fills_per_tick, mirrors
    // ps::terrain::TerrainConfig::max_tile_fills_per_tick), how many worker
    // threads the (not-yet-built, R3 scope is config-only) tile loader may
    // use (loader_workers), how much extra clearance a spawn-point tile
    // check demands beyond bare contact (spawn_clearance_m), and how long
    // Session may wait for the initial required tiles before giving up
    // (startup_timeout_s). Absent entirely -> every field keeps its default
    // below; not yet consumed by anything (R4 wires FixedRateLoop and the
    // streaming gate itself into Session).
    // Optional nested "road_surfaces" object inside "physics" (G2.5a-grip
    // R-b, roads_plan.md section 7): maps g2m::LandClass::PavedRoad/
    // UnpavedRoad/everything-else to a data/surfaces/surfaces.json name, so
    // a road-mode session's tyres can see per-cell grip (rg::RoadSurfaceMap,
    // rg/road_classes.h) instead of one uniform `terrain_surface` above -
    // `terrain_surface` is unchanged and still applies wherever
    // `road_surfaces.enabled` is false or a chunk has no road classes at
    // all. Absent entirely -> `enabled` stays false and paved/unpaved/
    // off_road keep the defaults below, which also match the render path's
    // own pre-existing hardcoded colours (world_terrain.cpp's
    // chunk_vertex_colors) exactly - a world_config.json with no
    // "road_surfaces" block renders byte-identically to before this struct
    // existed. Physics wiring (reading `enabled`, building a grip LUT from
    // these names) is R-c, not this commit - R-b only parses and renders.
    struct RoadSurfaces {
        bool enabled = false;
        std::string paved = "asphalt";
        std::string unpaved = "dirt";
        std::string off_road = "grass";
    };

    struct PhysicsTerrainConfig {
        bool engine_map_cache_enabled = true;
        std::string rolling_resistance_model = "quadratic";
        double radius_m = 400.0;
        std::string terrain_surface = "asphalt";
        std::uint32_t max_tile_fills_per_tick = 1;
        int loader_workers = 2;
        // Background shape prefetch; both zero preserves the synchronous path.
        std::uint32_t prefetch_margin_tiles = 0;
        std::uint32_t prefetch_max_tiles = 0;
        double spawn_clearance_m = 0.10;
        double startup_timeout_s = 30.0;
        RoadSurfaces road_surfaces;
    };

    std::string format; // always "rg.world/1" once successfully loaded
    std::string region;
    UtmOrigin session_origin_utm;
    SourceStore source_store;
    DerivedStore derived_store;
    Spawn spawn;
    Lod lod;
    TerrainSmoothing terrain_smoothing;
    double road_verge_drop_m=0.06;
    PhysicsTerrainConfig physics;
    std::string surface_map; // resolved to an absolute path (see below)
    std::string palette;     // resolved to an absolute path (see below)
};

// Loads and strictly validates a "rg.world/1" world-config JSON file (see
// data/world/world_config.json for the shape this parses). On success
// returns the parsed/resolved config; on failure returns std::nullopt and,
// if err is non-null, sets *err to a human-readable "<path>: <problem>"
// message (mirrors physics_sim's io loaders' error-string style) - never
// throws, so a malformed file is always reported as an ordinary error
// return, not an exception (deliberate: see this .cpp's own top comment for
// why it avoids catch-and-touch-the-exception entirely, vault TOOL-019).
//
// Path resolution (two conventions, picked deliberately - see the .cpp for
// the reasoning):
//  - "dir" fields (source_store.dir, derived_store.dir) go through ${VAR}
//    placeholder expansion ONLY. They name a location outside the repo (a
//    geo2map_engine store/cache directory) and are never resolved against
//    the repo root, and are used as-is (relative-to-CWD) if not absolute.
//  - "surface_map"/"palette" are resolved against the REPO ROOT, defined as
//    the nearest ancestor directory of the CONFIG FILE ITSELF that contains
//    a top-level CMakeLists.txt (so a config file living in a different
//    checked-out repo resolves against THAT repo's root, not necessarily
//    racing_game's). An already-absolute value is used as-is. Neither file
//    is required to exist - only the path is resolved.
//
// Placeholder expansion: "${NAME}" inside any string field (source_store.dir
// and derived_store.dir; no other field is expanded) is replaced by the
// environment variable NAME. RG_G2M_HOME is special-cased: if unset, it
// defaults to "<repo root>/cache/g2m/home-r1" (repo root = the nearest
// ancestor of THIS CONFIG FILE with a CMakeLists.txt - see the .cpp's
// find_repo_root; that directory is gitignored, PLAN.md R2.1 coordinator
// update 2026-09-26 - a local copy of the source store, never the original
// external geo2map_cache directory) instead of being an error. Any other
// unset/unknown "${NAME}" is a validation error.
//
// RG_G2M_DERIVED (checked directly, not a "${NAME}" placeholder): if set and
// non-empty, overrides the resolved derived_store.dir entirely (used as-is).
// Optional - world_config.json's own "derived_store.dir":
// "${RG_G2M_HOME}/derived" already tracks the RG_G2M_HOME default above.
std::optional<WorldConfig> load_world_config(const std::string& path, std::string* err);

// The settings screen's "Map data folder" (rg::Settings map_data.store_dir):
// points the source store at `dir` and the derived store at "<dir>/baked" (the
// same relation data/world/world_config.json states with ${RG_G2M_HOME}),
// unless RG_G2M_DERIVED is set, which keeps winning exactly as in
// load_world_config. An empty `dir` changes nothing (the config's own
// ${RG_G2M_HOME} / default stays). Applied by RgSimulation to the loaded
// config before the world opens; a pure function so it is unit-testable.
void apply_store_dir_override(WorldConfig& config, const std::string& dir);

} // namespace rg
