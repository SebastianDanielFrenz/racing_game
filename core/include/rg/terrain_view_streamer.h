// rg/terrain_view_streamer.h - rg::TerrainViewStreamer: render LOD that follows a moving focus point (R2.2 plan
// section 3, R8). Since terrain migration phase 3b it is geo2map's g2m::world::ViewStreamer (g2m/world/view_streamer.h:
// the adapter ordering contract - upload every add, THEN free the removals and commit - and the threading rules are in
// that header). Engine-neutral; godot_ext's RgTerrainView drives it.
#pragma once

#include "rg/terrain_render.h"
#include "rg/world_terrain.h"

#include "g2m/world/view_streamer.h"

namespace rg {

using TerrainViewDiff = g2m::world::ViewDiff;
using TerrainViewStreamer = g2m::world::ViewStreamer;

} // namespace rg
