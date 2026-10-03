#pragma once
#include "g2m/layer/src_osm.h"
#include "g2m/core/geo/utm.h"
#include <functional>
#include <optional>
namespace rg {
struct BuildingPoint {double e,n;};
struct Building {
 // OSM identity and geometry survive changes to the rendering/asset strategy.
 std::int64_t id;bool relation=false;double height=0,min_height=0,base=0,bottom=0;
 std::string height_source,colour,type;
 std::vector<std::vector<BuildingPoint>> outers,inners;
};
using BuildingGround=std::function<std::optional<double>(double,double)>;
std::vector<Building> extract_buildings(const g2m::OsmTile&,const g2m::TileKey&,double min_height,double fallback_height,double storey_height,const BuildingGround&);
}
