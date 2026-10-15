// rg/terrain_colours.h - the game's terrain chunk colours (terrain migration phase 3b): a colour table keyed by the
// per-vertex class id g2m::world::ViewStreamer chunks carry (mesh.land_class), plus the hypsometric height/slope ramp
// for every class without an entry. geo2map builds the chunks; their colours are the game's, applied through the
// ViewSource colour hook on the build thread. Engine-neutral (no Godot type).
#pragma once

#include "rg/road_classes.h"
#include "rg/terrain_render.h"

#include "g2m/mesh/terrain_chunk.h"
#include "g2m/world/view_streamer.h"

#include <array>
#include <cstdint>
#include <vector>

namespace rg {

// Packed RGBA8 per vertex: (r << 24) | (g << 16) | (b << 8) | a.
// One entry per class id; an id without an entry takes the height/slope ramp.
struct TerrainColourTable {
    std::array<std::uint32_t, 256> rgba{};
    std::array<bool, 256> set{};
};

// Road classes by their resolved surface NAME: PavedRoad/UnpavedRoad whose name is "asphalt" are grey
// (0.30, 0.30, 0.32), "dirt" brown (0.36, 0.28, 0.16); any other name, and every other class, the ramp.
[[nodiscard]] TerrainColourTable make_terrain_colour_table(const RoadSurfaceMap& surfaces);

// One colour per mesh vertex into `rgba` (mesh.land_class ids looked up in `table`; a mesh without class ids is all
// ramp). Valley green at 90 m, ridge brown at 880 m, blended to rock grey from ~14 deg to ~45 deg slope.
void chunk_vertex_colors(const g2m::mesh::TerrainChunkMesh& mesh, const TerrainColourTable& table,
                         std::vector<std::uint32_t>& rgba);

// Debug aid: env RG_TERRAIN_TINT = level | carve | both recolours a chunk by LOD level and/or carve state (L0 red ..
// L6 violet; carved green, Fallback magenta, not carved blue), keeping a hint of the shading. Read once; unset = no-op.
void apply_debug_tint(RenderChunk& chunk);

// The colour hook a ViewSource carries: chunk_vertex_colors with the table of `surfaces`, then apply_debug_tint.
[[nodiscard]] g2m::world::ChunkColourFn chunk_colour_hook(const RoadSurfaceMap& surfaces = {});

} // namespace rg
