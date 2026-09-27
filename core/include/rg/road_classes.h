// rg/road_classes.h — OSM road classes projected onto render-chunk vertices
// (roads_plan.md R-2, owner request 2026-09-27: "Next time I test drive, I
// at least want to see where roads are"). Fetches g2m.src.osm tiles (via a
// caller-supplied lookup - WorldTerrain::road_segments in production), runs
// geo2map_engine's own g2m::extract_road_segments/rasterize_road_segments,
// and rasterises the result onto one render chunk's L0 cell-centre lattice
// (g2m::mesh::ClassWindow) - the same lattice a level-0 TerrainChunkMesh's
// own vertices sit on (g2m/mesh/terrain_chunk.h's vertex-placement formula:
// vertex (i, j) at absolute E = (64*cx + i) * 2^level + 0.5, same for N), so
// a paved/unpaved road appears exactly where the chunk's own mesh puts its
// vertex.
//
// Render path only (roads_plan.md approach (a')): this file never touches
// physics_sim's cell_surface_ids / tyre grip - that is a separate later
// step, gated on its own owner yes (roads_plan.md "Open for the owner").
//
// No Godot type anywhere in this header or its .cpp (PLAN.md 11.1 / repo
// CLAUDE.md "engine-neutral logic").
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/mesh/terrain_chunk.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

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

// Looks up (fetches/decodes/caches) the OSM road segments for one
// g2m.src.osm tile (level 2, 1024 m + 128 m halo, docs/formats/src_osm.md).
// Returns an empty (never null) vector on any failure - "no roads" is a
// valid, non-fatal output. Mirrors g2m::mesh::TileLookup's own bare
// function-pointer-plus-ctx shape; WorldTerrain::road_segments (via the
// free function world_terrain_road_class_lookup in world_terrain.cpp) is
// the production implementation, tests supply a synthetic one.
using RoadSegmentsFn = std::shared_ptr<const std::vector<g2m::RoadSegment>> (*)(void* ctx, const g2m::TileKey& key);

// Bundles a RoadSegmentsFn with its ctx. A null `fn` means "no road
// classes": TerrainViewSource::class_lookup defaults to this, and
// rasterize_chunk_road_classes leaves the vertex ClassWindow all-Unknown
// without fetching anything - the R-2 "null lookup = byte-identical output"
// requirement (world_terrain.cpp's build_one_chunk calls build_chunk with a
// literal nullptr ClassWindow, the exact pre-R-2 code path, whenever
// class_lookup.fn is null).
struct ClassLookup {
    RoadSegmentsFn fn = nullptr;
    void* ctx = nullptr;
};

// The level policy (roads_plan.md R-2): thin forwarder to
// g2m::raster_params_for_render_level (G2.5a-grip G-c moved the actual
// policy table there so any g2m_layer caller can share it - see that
// function's own doc comment in g2m/layer/osm_roads.h for the exact L0-L3
// rules; L>=4 is std::nullopt, no road classes rasterised at all). Kept here
// (rather than deleted) so existing callers/tests need no signature change.
std::optional<g2m::RasterParams> road_raster_params_for_level(int level);

// The g2m::Lattice matching render chunk `key`'s own 65x65 L0 cell-centre
// vertex grid.
g2m::Lattice road_class_lattice(const g2m::mesh::ChunkKey& key);

// The g2m.src.osm tile keys (level 2, 1024 m) whose footprint overlaps
// render chunk `key`'s own footprint - at most 4 for any chunk level this
// project builds (level <= 3 => chunk side <= 512 m).
std::vector<g2m::TileKey> src_osm_tiles_for_chunk(const g2m::mesh::ChunkKey& key);

// Rasterises this chunk's road classes using g2m::rasterize_road_blocks'
// OWNER RULE (G2.5a-grip R-b, roads_plan.md section 2) instead of the old
// cross-tile merge + stable_sort-by-rank: `lookup` is called once per
// aligned g2m.src.osm (level 2, 1024 m) block this chunk's lattice
// straddles, and each block is painted from ONLY that block's own owner
// tile's segment list - never a concatenation of every overlapping tile's
// list. This means the result at any sample cannot depend on any
// neighbouring tile's own data or on `lookup`'s call order (the old merge's
// own weakness: an equal-rank tie between two tiles' segments was decided by
// which tile's list happened to be concatenated first, i.e. by
// src_osm_tiles_for_chunk's own iteration order - see
// tests/unit/test_road_classes.cpp's "equal-rank overlap" case for a pinned
// example). `out` is always first reset to all-Unknown, then left that way
// (no fetch at all) if `lookup.fn` is null or this chunk's level has no road
// policy (road_raster_params_for_level returns std::nullopt) - both
// "disabled" and "no roads found nearby" therefore look identical to
// build_chunk (LandClass::Unknown everywhere).
void rasterize_chunk_road_classes(const g2m::mesh::ChunkKey& key, const ClassLookup& lookup,
                                   g2m::mesh::ClassWindow& out);

} // namespace rg
