#pragma once
#include "g2m/layer/road_geom_tile.h"
#include "rg/road_structures.h"
#include "g2m/layer/height_tile.h"
#include <map>
#include <utility>
namespace rg {
// Immutable one-metre sampling of accepted ground-road profiles. This is
// applied before the shared tile is published to physics and rendering.
// Bridges/tunnels and explicit declines never carve the ground underneath.
class RoadSurfacePatch {
public:
    explicit RoadSurfacePatch(const g2m::RoadGeomTile& geometry,double verge_drop_m=0.0);
    std::size_t apply(g2m::HeightTile& tile) const;
    std::vector<std::shared_ptr<const RoadDeck>> decks;
    std::size_t accepted = 0, declined = 0, separated = 0;
private:
    struct Segment { double x0,y0,z0,x1,y1,z1,half_width,shoulder,blend,crown;
        std::int64_t way; std::uint32_t start_ref; int layer; bool tunnel; };
    double verge_drop_m_=0;
    std::vector<Segment> segments_;
    std::map<std::pair<int,int>,std::vector<std::size_t>> bins_;
};
}
