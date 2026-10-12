// complete_road_profiles must only complete what geo2map declined (terrain migration plan, phase 0, O6): a ground stretch
// the server already profiled stays exactly as served, even when one of its end nodes is a bridge anchor.
#include "rg/road_structures.h"
#include "g2m/layer/road_dem_sampler.h"
#include "g2m/layer/road_reference.h"
#include "g2m/layer/road_rules.h"
#include "g2m/layer/vertical_profile.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
namespace {
using Tags=std::vector<std::pair<std::string,std::string>>;
const g2m::geo::UtmZone kZone{32};
// Centre of the level-0 height tile (1822, 21686): every node and the 3 m junction footprint stay inside one tile.
constexpr double kOriginE=1822*256.0+128,kOriginN=21686*256.0+128;
constexpr double kDemHeightM=90,kServedHeightM=95,kBridgeHeightM=90;
constexpr std::int64_t kServedGround=10,kBridge=20,kDeclinedGround=30;

struct Scene {
    g2m::RoadGeomTile tile;
    std::unique_ptr<g2m::HeightTile> dem_tile;
    std::optional<g2m::RoadDemSampler> dem;
    std::size_t served=0,bridge=0,declined=0;
};
g2m::RoadGraphWay graph_way(std::int64_t id,std::vector<std::int64_t> nodes,Tags tags) {
    std::sort(tags.begin(),tags.end());
    return {id,3,std::move(nodes),std::move(tags)};
}
// Flat vertical profile of one stretch at a constant height.
g2m::RoadProfile flat_profile(const g2m::RoadReferenceBuilder& builder,const g2m::RoadGraphWay& way,g2m::RoadStretch stretch,double height_m) {
    auto attrs=g2m::resolve_road_attributes(way);REQUIRE(attrs);
    g2m::ReferenceFitLimits limits;limits.allow_curvature_exceptions=true;
    auto ref=builder.fit_stretch(way.osm_id,stretch,limits);REQUIRE(ref);
    const std::vector<g2m::VerticalSample> samples{{0,height_m},{ref->length_m,height_m}};
    g2m::VerticalFitLimits fit;fit.max_grade=.12;fit.min_radius_m=250;fit.max_iterations=16384;
    auto vertical=g2m::fit_vertical_profile(samples,fit);REQUIRE(vertical);
    g2m::RoadProfile p;p.stretch=stretch;p.attributes=*attrs;p.reference=std::move(*ref);p.vertical=std::move(*vertical);
    return p;
}
// node 1 --[served ground]-- node 2 --[accepted bridge]-- node 3 --[declined ground]-- node 4, flat DEM at 90 m.
// Both ground ways end at a bridge anchor (node 2 resp. node 3).
std::unique_ptr<Scene> make_scene() {
    auto scene=std::make_unique<Scene>();
    auto& g=scene->tile.source.graph;
    const double xs[]={-90,-20,20,90};
    for(int i=0;i<4;++i) g.nodes.push_back({i+1,{std::llround((kOriginE+xs[i])*1000),std::llround(kOriginN*1000)},{}});
    const Tags road{{"highway","tertiary"}},bridge{{"highway","tertiary"},{"bridge","yes"}};
    g.ways.push_back(graph_way(kServedGround,{1,2},road));
    g.ways.push_back(graph_way(kBridge,{2,3},bridge));
    g.ways.push_back(graph_way(kDeclinedGround,{3,4},road));
    g.edges={{kServedGround,0,1,2},{kBridge,0,2,3},{kDeclinedGround,0,3,4}};
    scene->tile.source.key={kZone,2,455,5421};
    auto builder=g2m::RoadReferenceBuilder::make(g);REQUIRE(builder);
    const g2m::RoadStretch whole{0,1};
    scene->tile.entries.push_back({kServedGround,whole,flat_profile(*builder,g.ways[0],whole,kServedHeightM),{},{},{}});
    scene->tile.entries.push_back({kBridge,whole,flat_profile(*builder,g.ways[1],whole,kBridgeHeightM),{},{},{}});
    g2m::Error declined;declined.message="road profile: insufficient ground approach support";
    scene->tile.entries.push_back({kDeclinedGround,whole,{},declined,{},{}});
    scene->served=0;scene->bridge=1;scene->declined=2;
    scene->dem_tile=std::make_unique<g2m::HeightTile>();
    scene->dem_tile->key={kZone,0,1822,21686};
    scene->dem_tile->h.fill(static_cast<std::int32_t>(kDemHeightM*256));
    const g2m::HeightTile* views[]={scene->dem_tile.get()};
    auto dem=g2m::RoadDemSampler::make(views);REQUIRE(dem);scene->dem=std::move(*dem);
    return scene;
}
bool same_vertical(const g2m::VerticalProfile& a,const g2m::VerticalProfile& b) {
    if(a.segments.size()!=b.segments.size()) return false;
    for(std::size_t i=0;i<a.segments.size();++i) {
        const auto& x=a.segments[i];const auto& y=b.segments[i];
        if(x.station_m!=y.station_m||x.length_m!=y.length_m||x.height0_m!=y.height0_m||x.height1_m!=y.height1_m||x.grade0!=y.grade0||x.grade1!=y.grade1) return false;
    }
    return true;
}
}
TEST_CASE("A served ground profile with a bridge-anchor end node comes out unchanged", "[road_structures]") {
    auto scene=make_scene();
    const auto served_before=*scene->tile.entries[scene->served].profile;
    const auto bridge_before=*scene->tile.entries[scene->bridge].profile;
    rg::complete_road_profiles(scene->tile,*scene->dem);
    const auto& served=scene->tile.entries[scene->served];
    REQUIRE(served.profile);
    CHECK(same_vertical(served.profile->vertical,served_before.vertical));
    CHECK(served.profile->reference.length_m==served_before.reference.length_m);
    CHECK(served.profile->at(30)->height_m==kServedHeightM);
    // The accepted bridge is not touched either.
    REQUIRE(scene->tile.entries[scene->bridge].profile);
    CHECK(same_vertical(scene->tile.entries[scene->bridge].profile->vertical,bridge_before.vertical));
}
TEST_CASE("A ground stretch geo2map declined is still completed against a bridge anchor", "[road_structures]") {
    auto scene=make_scene();
    REQUIRE_FALSE(scene->tile.entries[scene->declined].profile);
    rg::complete_road_profiles(scene->tile,*scene->dem);
    const auto& entry=scene->tile.entries[scene->declined];
    REQUIRE(entry.profile);
    CHECK_FALSE(entry.decline);
    // Anchor (the bridge end) and DEM are both at 90 m, so the completed ground is flat there.
    CHECK(std::abs(entry.profile->at(0)->height_m-kBridgeHeightM)<.01);
    CHECK(std::abs(entry.profile->at(entry.profile->reference.length_m/2)->height_m-kDemHeightM)<.01);
}
