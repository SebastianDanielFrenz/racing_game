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
#include <vector>

namespace rg {

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

// The level policy (roads_plan.md R-2):
//   L0-L1: every drivable road (min_rank 0, no width floor).
//   L2:    tertiary and above (rank >= the style's own "tertiary" rank).
//   L3:    primary and above (rank >= the style's own "primary" rank), and
//          every segment's half-width is floored to 0.75 * this level's
//          lattice spacing so a thin road stays continuous on this coarser
//          grid (roads_plan.md's own number).
//   L>=4:  std::nullopt - no road classes rasterised at all.
// Tertiary/primary ranks are looked up from g2m::RoadStyle::default_style()
// rather than hard-coded, so a future re-numbering of osm_roads.cpp's own
// rank table cannot silently desync this policy from it (falls back to the
// table's current values, 5 and 7, only if a style is ever shipped without
// those two highway kinds at all).
std::optional<g2m::RasterParams> road_raster_params_for_level(int level);

// The g2m::Lattice matching render chunk `key`'s own 65x65 L0 cell-centre
// vertex grid.
g2m::Lattice road_class_lattice(const g2m::mesh::ChunkKey& key);

// The g2m.src.osm tile keys (level 2, 1024 m) whose footprint overlaps
// render chunk `key`'s own footprint - at most 4 for any chunk level this
// project builds (level <= 3 => chunk side <= 512 m).
std::vector<g2m::TileKey> src_osm_tiles_for_chunk(const g2m::mesh::ChunkKey& key);

// Fetches every overlapping g2m.src.osm tile via `lookup`, merges their road
// segments, re-sorts the merge by ascending rank (osm_roads.h's "later
// overwrites earlier" rasterisation rule needs ascending rank; each tile's
// own extract_road_segments output is already sorted that way, but the
// concatenation of several tiles' outputs is not), and rasterises into
// `out`. `out` is always first reset to all-Unknown, then left that way
// (no fetch at all) if `lookup.fn` is null or this chunk's level has no
// road policy (road_raster_params_for_level returns std::nullopt) - both
// "disabled" and "no roads found nearby" therefore look identical to
// build_chunk (LandClass::Unknown everywhere).
void rasterize_chunk_road_classes(const g2m::mesh::ChunkKey& key, const ClassLookup& lookup,
                                   g2m::mesh::ClassWindow& out);

} // namespace rg
