#pragma once
#include "g2m/layer/road_geom_tile.h"
#include <functional>
namespace rg {
// flags (bits, read by road.gdshader): 1 lane markings, 2 edge lines, 4 solid separator, 8 fast road, 16 solid paint,
// 32 overlay (only the paint is drawn), 64 / 128 left / right edge line broken (a junction mouth), 512 / 1024 left /
// right edge line not drawn, 2048 paving stones (no painted lines), 4096 concrete, 8192 unpaved (a junction patch
// piece at a track's mouth, drawn like the terrain's dirt). The Godot adapter adds 256 for a verge vertex and splits the
// bits over COLOR.b and COLOR.a.
// u: lateral offset from the centre of the counted lanes, v: station, width: the counted lanes' width; verge 1 = the
// gravel strip beside the road bed (no asphalt, no paint).
struct RoadVisualVertex { float x,y,z,u,v,width,fade,lanes,forward,flags,verge=0; };
struct RoadVisualMesh { std::vector<RoadVisualVertex> vertices; std::vector<std::int32_t> indices; };
using RoadVisualGround = std::function<std::optional<double>(double,double)>;
// Ribbons per paved stretch plus (owner tile only) one asphalt patch per served junction road bed ring, the through road's
// markings continued over it, and the stop, give-way and crossing lines (OSM furniture plus implied give-way lines).
RoadVisualMesh build_road_visual(const g2m::RoadGeomTile&,const g2m::TileKey&,const RoadVisualGround&,double lift_m);
// The junction patches alone (appended to `mesh`), in tile-local metres like build_road_visual.
void add_junction_surfaces(RoadVisualMesh& mesh,const g2m::RoadGeomTile&,const g2m::TileKey&,const RoadVisualGround&,double lift_m);
}
