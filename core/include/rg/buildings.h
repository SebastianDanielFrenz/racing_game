// rg/buildings.h - OSM building footprints. Since terrain migration phase 3b the extraction is geo2map's
// (g2m/world/buildings.h, BuildingSet); these names stay for the game's callers.
#pragma once
#include "g2m/world/buildings.h"
namespace rg {
using g2m::world::Building;
using g2m::world::BuildingGround;
using g2m::world::BuildingPoint;
using g2m::world::extract_buildings;
} // namespace rg
