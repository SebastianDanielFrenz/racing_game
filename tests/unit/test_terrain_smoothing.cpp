#include "rg/terrain_smoothing.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <map>
#include <cmath>
TEST_CASE("Terrain smoothing retains slope and uses identical halos across tile seams", "[terrain_smoothing]") {
    std::map<g2m::TileKey,std::shared_ptr<g2m::HeightTile>> tiles;
    for(int ty=-1;ty<=1;++ty) for(int tx=-1;tx<=2;++tx) {
        auto tile=std::make_shared<g2m::HeightTile>();tile->key={{32},0,tx,ty};
        for(int y=0;y<256;++y) for(int x=0;x<256;++x) {
            const int gx=tx*256+x,gy=ty*256+y;
            const double h=100+.01*(gx+.5)+.02*(gy+.5)+(gx%2==0?.3:-.3);
            tile->h[static_cast<std::size_t>(y)*256+x]=static_cast<std::int32_t>(std::llround(h*256));
        }
        tiles.emplace(tile->key,std::move(tile));
    }
    rg::TerrainSmoothing settings;settings.enabled=true;settings.radius_m=3;settings.strength=.5;settings.passes=2;
    auto lookup=[&](const auto& key)->std::shared_ptr<const g2m::HeightTile>{auto it=tiles.find(key);return it==tiles.end()?nullptr:it->second;};
    auto a=rg::smooth_terrain(*tiles.at({{32},0,0,0}),settings,lookup);
    auto b=rg::smooth_terrain(*tiles.at({{32},0,1,0}),settings,lookup);
    const auto at=[](const auto& t,int x,int y){return t->h[static_cast<std::size_t>(y)*256+x]/256.0;};
    CHECK(at(a,100,120)-at(a,98,120)==Catch::Approx(.02).margin(.005));
    CHECK(at(a,100,120)-at(a,100,118)==Catch::Approx(.04).margin(.005));
    // Both sides see the same alternating noise, with half its amplitude.
    CHECK(std::abs(at(a,254,128)-(100+.01*254.5+.02*128.5))<.16);
    CHECK(at(b,0,128)-at(a,254,128)==Catch::Approx(.02).margin(.005));
    CHECK(at(a,255,128)-at(a,253,128)==Catch::Approx(.02).margin(.005));
    // Input data remains immutable and disabling smoothing is byte-exact.
    CHECK(std::abs(at(tiles.at({{32},0,0,0}),100,128)-(100+1.005+2.57))>.29);
    settings.enabled=false;CHECK(rg::smooth_terrain(*tiles.at({{32},0,0,0}),settings,lookup)->h==tiles.at({{32},0,0,0})->h);
}
TEST_CASE("Terrain smoothing does not manufacture heights over NoData", "[terrain_smoothing]") {
    auto tile=std::make_shared<g2m::HeightTile>();tile->key={{32},0,0,0};tile->h.fill(100*256);
    tile->h[128*256+128]=g2m::kHeightNoData;tile->has_nodata=true;
    rg::TerrainSmoothing settings;settings.enabled=true;
    auto out=rg::smooth_terrain(*tile,settings,[](const auto&)->std::shared_ptr<const g2m::HeightTile>{return {};});
    CHECK(out->h[128*256+128]==g2m::kHeightNoData);CHECK(out->h[128*256+127]==100*256);CHECK(out->has_nodata);
}
