#pragma once
#include "g2m/layer/road_geom_tile.h"
#include "g2m/world/road_structures.h"
#include <memory>
namespace rg {
// Structure decks (bridges, tunnel floors, inferred tunnel roofs, approach decks of grade-separated crossings) are built
// by geo2map's g2m_world since terrain migration phase 3a; the names stay available here for the game's callers.
using RoadDeck = g2m::world::RoadDeck;
// Grade separation between ground roads (world_config "road_separation"). Two roads whose surfaces differ by more than
// clearance_m cannot share terrain: one of them stands on a structure there.
//  - RoadSurfacePatch never lets a road lift (or lower) the terrain inside another road's protected footprint
//    (half width + shoulder + margin_m) when it is more than clearance_m above (below) that road;
//  - build_road_decks continues a bridge onto its ground approach (an inferred deck) from the bridge end as far as the
//    approach's footprint still overlaps a road more than clearance_m below it (within approach_reach_m), so the upper
//    road is carried by the structure, never by terrain carved for the road underneath. An OSM bridge way that ends
//    over the lower road's verge is common; without this the approach's embankment was painted onto the lower road.
// (g2m::world::RoadSeparation: {clearance_m, margin_m, approach_reach_m, enabled = true}.)
using RoadSeparation = g2m::world::RoadSeparation;
using g2m::world::road_deck_height;
using g2m::world::build_road_decks;
// Reconstruct rejected ground profiles and bridge abutments from immutable DEM.
// Inferred spans require an actual crossing of an OSM tunnel at a lower layer.
// (Game-side until terrain migration phase 4 serves the completed profiles.)
void complete_road_profiles(g2m::RoadGeomTile& geometry,const g2m::RoadDemSampler& dem);
}
