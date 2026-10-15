// rg/terrain_render.h - RenderChunk: one built terrain chunk plus its session-local origin and per-vertex colour.
// Since terrain migration phase 3b the type is geo2map's g2m::world::ViewChunk (key, carve state, mesh with class ids,
// origin_session, rgba); the game fills rgba through its colour hook (rg/terrain_colours.h). godot_ext's RgTerrainView
// is the only place these cross into godot:: RenderingServer calls.
#pragma once

#include "g2m/world/view_streamer.h"

namespace rg {

// How the height tile(s) a chunk was built from were carved (NotCarved/Carved/Fallback, WorldTerrain::tile_carve).
using ChunkCarve = g2m::world::ChunkCarve;
using RenderChunk = g2m::world::ViewChunk;

} // namespace rg
