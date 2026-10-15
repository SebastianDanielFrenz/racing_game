#pragma once
#include "g2m/layer/road_geom_tile.h"
#include "g2m/mesh/terrain_chunk.h"
#include <memory>
namespace rg {
struct RoadDeck {
    std::int64_t way_id=0;
    std::uint32_t start_ref=0; // the stretch of way_id the stations belong to (a way can hold several stretches)
    double start_station=0,end_station=0;
    std::vector<g2m::VerticalCubic> vertical;
    double half_width=0,crown=0;
    bool inferred=false;
    bool tunnel_floor=false;
    double min_easting=0,max_easting=0,min_northing=0,max_northing=0;
    g2m::LandClass land_class=g2m::LandClass::PavedRoad;
    g2m::mesh::TerrainChunkMesh mesh;
};
// Grade separation between ground roads (world_config "road_separation"). Two roads whose surfaces differ by more than
// clearance_m cannot share terrain: one of them stands on a structure there.
//  - RoadSurfacePatch never lets a road lift (or lower) the terrain inside another road's protected footprint
//    (half width + shoulder + margin_m) when it is more than clearance_m above (below) that road;
//  - build_road_decks continues a bridge onto its ground approach (an inferred deck) from the bridge end as far as the
//    approach's footprint still overlaps a road more than clearance_m below it (within approach_reach_m), so the upper
//    road is carried by the structure, never by terrain carved for the road underneath. An OSM bridge way that ends
//    over the lower road's verge is common; without this the approach's embankment was painted onto the lower road.
struct RoadSeparation {
    double clearance_m=2.5;
    double margin_m=3.0;
    double approach_reach_m=60.0;
};
std::optional<double> road_deck_height(const RoadDeck&,double station,double lateral);
// Reconstruct rejected ground profiles and bridge abutments from immutable DEM.
// Inferred spans require an actual crossing of an OSM tunnel at a lower layer.
void complete_road_profiles(g2m::RoadGeomTile& geometry,const g2m::RoadDemSampler& dem);
std::vector<std::shared_ptr<const RoadDeck>> build_road_decks(const g2m::RoadGeomTile& geometry,const RoadSeparation& separation={});
}
