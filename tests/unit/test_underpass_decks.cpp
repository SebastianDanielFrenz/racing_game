// Underpass decks (geo2map roads.geom encoding 9, design D53-D58): a ground stretch whose served span crosses a lower
// road in a DEM cutting carries an inferred deck over exactly that span. The game treats it like a bridge deck: the
// ground under it is not carved to the (rim-chord) profile, terrain is kept below the slab, and the deck is the collider.
#include "rg/road_surface.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <algorithm>
#include <cmath>
namespace {
// One straight 512 m paved ground road along y = 128.5, 100 m at station 0 rising 5 %.
g2m::RoadGeomTile geometry(bool with_underpass) {
    g2m::RoadGeomTile tile;
    g2m::RoadProfile p;
    p.attributes.width_mm=10000; p.attributes.shoulder_mm=1000;
    p.attributes.blend_mm=5000; p.attributes.crown_per_mille=0;
    p.attributes.land_class=g2m::LandClass::PavedRoad;
    p.reference.length_m=512;
    g2m::RoadReferenceSegment s; s.curve={0,128.5,0,512,0,0};
    p.reference.segments.push_back(s);
    p.vertical.segments.push_back({0,512,100,125.6,.05,.05});
    g2m::RoadGeomEntry entry; entry.way_id=1; entry.stretch={0,1}; entry.profile=std::move(p);
    tile.entries.push_back(std::move(entry));
    if(with_underpass) {
        g2m::RoadUnderpass u; u.entry_index=0; u.start_m=200; u.end_m=240; u.depth_mm=6000; u.lower_ways={99};
        tile.underpasses.push_back(u);
    }
    return tile;
}
std::unique_ptr<g2m::HeightTile> terrain(double height_m) {
    auto t=std::make_unique<g2m::HeightTile>();t->key={{32},0,0,0};
    t->h.fill(static_cast<std::int32_t>(height_m*256));return t;
}
double at(const g2m::HeightTile& t,int x,int y=128) { return t.h[static_cast<std::size_t>(y)*256+x]/256.0; }
}

TEST_CASE("Underpass span becomes one inferred deck over exactly the span; none without the record", "[road_surface][deck][underpass]") {
    const auto decks=rg::build_road_decks(geometry(true));
    REQUIRE(decks.size()==1);
    const auto& d=*decks.front();
    CHECK(d.inferred); CHECK_FALSE(d.tunnel_floor);
    CHECK(d.way_id==1); CHECK(d.start_ref==0);
    CHECK(d.start_station==200); CHECK(d.end_station==240);
    // The deck carries the stretch's own profile: its top is the road height at each station.
    for(double station:{200.0,220.0,240.0}) {
        const auto z=rg::road_deck_height(d,station,0);REQUIRE(z);
        CHECK(*z==Catch::Approx(100+.05*station).margin(1e-6));
    }
    CHECK_FALSE(rg::road_deck_height(d,199,0));CHECK_FALSE(rg::road_deck_height(d,241,0));
    CHECK(rg::build_road_decks(geometry(false)).empty());
}

TEST_CASE("Ground under an underpass deck is not carved to the road; outside it is", "[road_surface][deck][underpass]") {
    const rg::RoadSurfacePatch with(geometry(true)),without(geometry(false));
    REQUIRE(with.decks.size()==1);REQUIRE(without.decks.empty());
    auto a=terrain(90),b=terrain(90);
    REQUIRE(with.apply(*a)>0);REQUIRE(without.apply(*b)>0);
    // Mid-span (beyond the 11 m footprint + blend of the carved road on either side): the lower road's cutting stays,
    // the profile is not drawn.
    for(int x=212;x<=228;++x) {
        INFO("x="<<x);
        CHECK(at(*a,x)==Catch::Approx(90));
        CHECK(at(*b,x)==Catch::Approx(100+.05*(x+.5)).margin(.01));
    }
    // Outside the span both carve the same road surface.
    for(int x:{50,100,180,250}) CHECK(a->h[128*256+x]==b->h[128*256+x]);
    CHECK(at(*a,100)==Catch::Approx(105.025).margin(.01));
}

TEST_CASE("Terrain above an underpass deck is cut below its slab like a bridge", "[road_surface][deck][underpass]") {
    const rg::RoadSurfacePatch patch(geometry(true));
    auto t=terrain(130);REQUIRE(patch.apply(*t)>0);
    for(int x=208;x<=232;++x) {
        INFO("x="<<x);
        CHECK(at(*t,x)<=100+.05*(x+.5)-.45);
    }
    CHECK(at(*t,0,10)==Catch::Approx(130));
}

TEST_CASE("No ground carve reaches into an underpass deck past the 1 m wall fade (D59)", "[road_surface][deck][underpass]") {
    // The 1 m carve segments beside the deck are capsules: without the deck end planes their round ends and blend
    // (5 m + 1 m + 5 m here) painted the road onto the cutting up to 11 m inside the deck.
    const rg::RoadSurfacePatch patch(geometry(true));
    auto t=terrain(90);REQUIRE(patch.apply(*t)>0);
    for(int y=108;y<=148;++y) for(int x=201;x<=238;++x) {   // cell centres x+0.5 in [201.5, 238.5]: past both fades
        INFO("x="<<x<<" y="<<y);
        CHECK(at(*t,x,y)==Catch::Approx(90));
    }
}

TEST_CASE("The carved approach meets the underpass deck without a step (D59)", "[road_surface][deck][underpass]") {
    const rg::RoadSurfacePatch patch(geometry(true));
    REQUIRE(patch.decks.size()==1);
    const auto& d=*patch.decks.front();
    auto t=terrain(90);REQUIRE(patch.apply(*t)>0);
    // The last full-weight cell before each plane carries the road profile; the deck top at the plane continues it.
    // Bound: the profile's own change over one 1 m carve segment (5 % x 1 m; the segment ending on the deck start is
    // covered, so the cell there takes the previous segment's end height) plus the 1/256 m quantisation and 1 cm.
    const double bound=.05*1+1.0/256+.01;
    const auto top_start=rg::road_deck_height(d,200,0),top_end=rg::road_deck_height(d,240,0);
    REQUIRE(top_start);REQUIRE(top_end);
    CHECK(std::abs(at(*t,199)-*top_start)<=bound);
    CHECK(std::abs(at(*t,240)-*top_end)<=bound);
    for(int y=123;y<=133;++y) {   // inside the 6 m core (half width + shoulder) of the road at y = 128.5
        INFO("y="<<y);
        CHECK(std::abs(at(*t,198,y)-(100+.05*198.5))<=.01);
        CHECK(std::abs(at(*t,241,y)-(100+.05*241.5))<=.01);
        // Through the fade the carve only falls from the road to the cutting: no overshoot either way.
        for(int x:{199,200,239,240}) { CHECK(at(*t,x,y)<=100+.05*(x+.5)+.01); CHECK(at(*t,x,y)>=90-.01); }
        CHECK(at(*t,200,y)<=at(*t,199,y)+1e-9);CHECK(at(*t,239,y)<=at(*t,240,y)+1e-9);
    }
}
