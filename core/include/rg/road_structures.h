#pragma once
#include "g2m/layer/road_geom_tile.h"
#include "g2m/mesh/terrain_chunk.h"
#include <memory>
namespace rg {
struct RoadDeck {
    std::int64_t way_id=0;
    double start_station=0,end_station=0;
    std::vector<g2m::VerticalCubic> vertical;
    double half_width=0,crown=0;
    bool inferred=false;
    bool tunnel_floor=false;
    double min_easting=0,max_easting=0,min_northing=0,max_northing=0;
    g2m::LandClass land_class=g2m::LandClass::PavedRoad;
    g2m::mesh::TerrainChunkMesh mesh;
};
std::optional<double> road_deck_height(const RoadDeck&,double station,double lateral);
// Reconstruct rejected ground profiles and bridge abutments from immutable DEM.
// Inferred spans require an actual crossing of an OSM tunnel at a lower layer.
void complete_road_profiles(g2m::RoadGeomTile& geometry,const g2m::RoadDemSampler& dem);
std::vector<std::shared_ptr<const RoadDeck>> build_road_decks(const g2m::RoadGeomTile& geometry);
}
