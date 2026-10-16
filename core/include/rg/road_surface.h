#pragma once
#include "g2m/layer/road_geom_tile.h"
#include "rg/road_structures.h"
#include "g2m/layer/height_tile.h"
#include <array>
#include <map>
#include <vector>
#include <utility>
namespace rg {
// Immutable one-metre sampling of accepted ground-road profiles. This is
// applied before the shared tile is published to physics and rendering.
// Bridges/tunnels and explicit declines never carve the ground underneath.
class RoadSurfacePatch {
public:
    explicit RoadSurfacePatch(const g2m::RoadGeomTile& geometry,double verge_drop_m=0.0,const RoadSeparation& separation={});
    std::size_t apply(g2m::HeightTile& tile) const;
    // Debug/probe aid: every `stride`-th ground (non-tunnel) road segment start as {easting, northing, bed height}
    // (absolute UTM) appended to `out` - the physics road bed at the road centre line.
    void bed_points(std::vector<std::array<double, 3>>& out, int stride) const;
    std::vector<std::shared_ptr<const RoadDeck>> decks;
    std::size_t accepted = 0, declined = 0, separated = 0;
private:
    // A deck end plane (underpass decks, geo2map D59): the normal points into the deck. A segment of the ground piece
    // next to a served underpass deck does not carve past it: its weight fades to zero over kUnderpassWallFadeM.
    struct DeckPlane { bool on; double x,y,nx,ny; };
    struct Segment { double x0,y0,z0,x1,y1,z1,half_width,shoulder,blend,crown;
        std::int64_t way; std::uint32_t start_ref; int layer; bool tunnel; DeckPlane deck_lo, deck_hi; };
    struct DeckCut {double x0,y0,z0,x1,y1,z1,half_width,crown,from,total;};
    std::vector<DeckCut> deck_cuts_;
    std::map<std::pair<int,int>,std::vector<std::size_t>> cut_bins_;
    double verge_drop_m_=0;
    RoadSeparation separation_;
    std::vector<Segment> segments_;
    std::map<std::pair<int,int>,std::vector<std::size_t>> bins_;
};
}
