#pragma once

#include "g2m/world/road_ribbons.h"
#include "g2m/layer/terrain_class.h"

namespace rg {

// Append explicit surface triangles for unpaved roads and cross-section paths/sidewalks that are not part of the
// paved carriageway ribbon. The returned vertices share RoadRibbonTile's renderer contract (tile-local UTM metres,
// absolute height, and the road shader's material flags), so the terrain remains a natural underlay.
void append_route_surface_mesh(g2m::world::RoadRibbonTile& mesh, const g2m::RoadGeomTile& geometry,
                               const g2m::TileKey& key, const g2m::world::RoadRibbonGround& ground, double lift_m,
                               const g2m::world::RoadSeparation& separation = {});

// Stamp the same explicit path/walkway triangles into level-0 physics classes. This keeps the 1 m physics material
// grid aligned with the constructed route surface instead of relying on the coarser landcover classifier.
void stamp_route_surface_classes(g2m::TerrainClassTile& classes, const g2m::world::RoadRibbonTile& mesh,
                                 const g2m::world::RoadRibbonGround& terrain_ground);

} // namespace rg
