#pragma once
#include "g2m/layer/height_tile.h"
#include <functional>
#include <memory>
namespace rg {
struct TerrainSmoothing {
    bool enabled=false;
    double radius_m=2.0;
    double strength=.35;
    int passes=1;
};
using RawHeightLookup=std::function<std::shared_ptr<const g2m::HeightTile>(const g2m::TileKey&)>;
// Separable Gaussian on the absolute sample lattice, with a complete halo
// for every pass. NoData centres remain NoData; imported heights are immutable.
std::shared_ptr<const g2m::HeightTile> smooth_terrain(const g2m::HeightTile& input,
    const TerrainSmoothing& parameters,const RawHeightLookup& lookup);
}
