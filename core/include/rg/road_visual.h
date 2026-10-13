#pragma once
#include "g2m/layer/road_geom_tile.h"
#include <functional>
namespace rg {
struct RoadVisualVertex { float x,y,z,u,v,width,fade,lanes,forward,flags; };
struct RoadVisualMesh { std::vector<RoadVisualVertex> vertices; std::vector<std::int32_t> indices; };
using RoadVisualGround = std::function<std::optional<double>(double,double)>;
// Ribbons per paved stretch plus (owner tile only) one asphalt patch per served junction road bed ring.
RoadVisualMesh build_road_visual(const g2m::RoadGeomTile&,const g2m::TileKey&,const RoadVisualGround&,double lift_m);
// The junction patches alone (appended to `mesh`), in tile-local metres like build_road_visual.
void add_junction_surfaces(RoadVisualMesh& mesh,const g2m::RoadGeomTile&,const g2m::TileKey&,const RoadVisualGround&,double lift_m);
}
