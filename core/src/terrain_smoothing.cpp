#include "rg/terrain_smoothing.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
namespace rg {
std::shared_ptr<const g2m::HeightTile> smooth_terrain(const g2m::HeightTile& input,
    const TerrainSmoothing& parameters,const RawHeightLookup& lookup) {
    auto output=std::make_shared<g2m::HeightTile>(input);
    if(!parameters.enabled||parameters.strength<=0) return output;
    const double spacing=std::ldexp(1.0,input.key.level);
    const int radius=static_cast<int>(std::floor(parameters.radius_m/spacing+1e-9));
    if(radius<1) return output;
    const int halo=radius*parameters.passes,size=256+2*halo;
    const double missing=std::numeric_limits<double>::quiet_NaN();
    std::vector<double> original(static_cast<std::size_t>(size)*size,missing),work=original,temp=original;
    std::map<g2m::TileKey,std::shared_ptr<const g2m::HeightTile>> tiles;
    for(int y=0;y<size;++y) for(int x=0;x<size;++x) {
        const auto gx=static_cast<std::int64_t>(input.key.x)*256+x-halo,gy=static_cast<std::int64_t>(input.key.y)*256+y-halo;
        const auto tx=g2m::floor_div(gx,256),ty=g2m::floor_div(gy,256);
        const g2m::TileKey key{input.key.zone,input.key.level,static_cast<std::int32_t>(tx),static_cast<std::int32_t>(ty)};
        const g2m::HeightTile* tile=&input;
        if(key!=input.key) {
            auto [it,inserted]=tiles.try_emplace(key);if(inserted) it->second=lookup(key);
            tile=it->second.get();
        }
        if(tile) {
            const auto value=tile->h[static_cast<std::size_t>(gy-ty*256)*256+static_cast<std::size_t>(gx-tx*256)];
            if(value!=g2m::kHeightNoData) original[static_cast<std::size_t>(y)*size+x]=value/256.0;
        }
    }
    work=original;
    std::vector<double> kernel(static_cast<std::size_t>(2*radius+1));
    const double sigma=std::max(.5,radius/2.0);
    for(int i=-radius;i<=radius;++i) kernel[static_cast<std::size_t>(i+radius)]=std::exp(-i*i/(2*sigma*sigma));
    for(int pass=0;pass<parameters.passes;++pass) {
        for(int axis=0;axis<2;++axis) {
            for(int y=0;y<size;++y) for(int x=0;x<size;++x) {
                const auto index=static_cast<std::size_t>(y)*size+x;
                if(!std::isfinite(work[index])) {temp[index]=missing;continue;}
                double sum=0,weights=0;
                for(int d=-radius;d<=radius;++d) {
                    const int xx=x+(axis==0?d:0),yy=y+(axis==1?d:0);
                    if(xx<0||xx>=size||yy<0||yy>=size) continue;
                    const double value=work[static_cast<std::size_t>(yy)*size+xx];if(!std::isfinite(value)) continue;
                    const double weight=kernel[static_cast<std::size_t>(d+radius)];sum+=weight*value;weights+=weight;
                }
                temp[index]=weights>0?sum/weights:missing;
            }
            work.swap(temp);
        }
    }
    for(int y=0;y<256;++y) for(int x=0;x<256;++x) {
        auto& value=output->h[static_cast<std::size_t>(y)*256+x];if(value==g2m::kHeightNoData) continue;
        const auto index=static_cast<std::size_t>(y+halo)*size+x+halo;
        const double height=original[index]+parameters.strength*(work[index]-original[index]);
        if(std::isfinite(height)) value=static_cast<std::int32_t>(std::llround(height*256));
    }
    return output;
}
}
