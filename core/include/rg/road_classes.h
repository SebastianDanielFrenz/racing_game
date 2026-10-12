// rg/road_classes.h — the game's LandClass -> surface NAME map (RoadSurfaceMap).
// The chunk road-class rasterisation (ClassLookup, road_class_lattice,
// src_osm_tiles_for_chunk, rasterize_chunk_road_classes, the level policy)
// moved to geo2map (g2m/world/chunk_road_classes.h, terrain migration phase 1,
// vault G2M-020). RoadSurfaceMap goes in phase 7 (carve switch).
//
// No Godot type anywhere in this header (PLAN.md 11.1 / repo CLAUDE.md
// "engine-neutral logic").
#pragma once

#include "g2m/layer/osm_roads.h"

#include <string>

namespace rg {

// LandClass -> physics.road_surfaces surface NAME (G2.5a-grip R-b,
// roads_plan.md section 7): resolves g2m::LandClass::PavedRoad/UnpavedRoad/
// everything-else to one of three data/surfaces/surfaces.json names, so both
// the render path (world_terrain.cpp's chunk_vertex_colors, keyed by the
// resolved name rather than LandClass directly) and physics (R-c, not this
// commit - Session::setup_terrain would build a g2m::RoadValueLut from these
// same three names via the session's own SurfaceTable) share one source of
// truth: a name change in rg::WorldConfig::PhysicsTerrainConfig::
// road_surfaces cannot silently desync the two. Defaults ("asphalt"/"dirt"/
// "grass") match this repo's pre-existing hardcoded render colours exactly
// (world_terrain.cpp's own kPavedR.../kUnpavedR... constants), so a
// default-constructed map renders byte-identically to before it existed.
struct RoadSurfaceMap {
    std::string paved = "asphalt";
    std::string unpaved = "dirt";
    std::string off_road = "grass";

    [[nodiscard]] const std::string& name_for(g2m::LandClass land_class) const {
        if (land_class == g2m::LandClass::PavedRoad) return paved;
        if (land_class == g2m::LandClass::UnpavedRoad) return unpaved;
        return off_road;
    }
};

} // namespace rg
