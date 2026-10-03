#include "rg/road_surface.h"
#include <algorithm>
#include <cmath>
#include <limits>
namespace rg {
namespace {
constexpr double bin_size = 16;
int bin(double value) { return static_cast<int>(std::floor(value / bin_size)); }
double smooth(double t) { t=std::clamp(t,0.0,1.0); return t*t*(3-2*t); }
}
RoadSurfacePatch::RoadSurfacePatch(const g2m::RoadGeomTile& geometry) {
    decks=build_road_decks(geometry);
    for (const auto& entry : geometry.entries) {
        if (!entry.profile) { ++declined; continue; }
        const auto& p = *entry.profile;
        if (p.attributes.bridge) { ++separated; continue; }
        if (p.attributes.land_class != g2m::LandClass::PavedRoad && !p.attributes.tunnel) continue;
        const int count = std::max(1,static_cast<int>(std::ceil(p.reference.length_m)));
        auto a=p.reference.at(0); auto za=p.at(0);
        if (!a.ok() || !za.ok()) continue;
        ++accepted;
        for (int i=1;i<=count;++i) {
            const double station = p.reference.length_m*i/count;
            auto b=p.reference.at(station); auto zb=p.at(station);
            if (!b.ok() || !zb.ok()) break;
            const bool covered=std::any_of(decks.begin(),decks.end(),[&](const auto& d){
                return d->inferred&&d->way_id==entry.way_id&&station>=d->start_station&&station<=d->end_station;
            });
            if(covered) {a=std::move(b);za=std::move(zb);continue;}
            Segment s{a.value().x,a.value().y,za.value().height_m,
                b.value().x,b.value().y,zb.value().height_m,
                p.attributes.width_mm/2000.0,p.attributes.shoulder_mm/1000.0,
                p.attributes.blend_mm/1000.0,p.attributes.crown_per_mille/1000.0,
                entry.way_id,entry.stretch.start_ref};
            const double radius=s.half_width+s.shoulder+s.blend;
            const auto index=segments_.size(); segments_.push_back(s);
            for(int y=bin(std::min(s.y0,s.y1)-radius);y<=bin(std::max(s.y0,s.y1)+radius);++y)
                for(int x=bin(std::min(s.x0,s.x1)-radius);x<=bin(std::max(s.x0,s.x1)+radius);++x)
                    bins_[{x,y}].push_back(index);
            a=std::move(b); za=std::move(zb);
        }
    }
}
std::size_t RoadSurfacePatch::apply(g2m::HeightTile& tile) const {
    std::size_t changed=0;
    const double spacing=std::ldexp(1.0,tile.key.level);
    for(int y=0;y<256;++y) for(int x=0;x<256;++x) {
        auto& raw=tile.h[static_cast<std::size_t>(y)*256+x];
        if(raw==g2m::kHeightNoData) continue;
        const double e=static_cast<double>(tile.key.min_easting())+(x+.5)*spacing;
        const double n=static_cast<double>(tile.key.min_northing())+(y+.5)*spacing;
        const auto candidates=bins_.find({bin(e),bin(n)});
        if(candidates==bins_.end()) continue;
        const Segment* best=nullptr; double distance=std::numeric_limits<double>::infinity(), fraction=0;
        for(auto index:candidates->second) {
            const auto& s=segments_[index]; const double dx=s.x1-s.x0,dy=s.y1-s.y0;
            const double length2=dx*dx+dy*dy; if(length2<1e-12) continue;
            const double t=std::clamp(((e-s.x0)*dx+(n-s.y0)*dy)/length2,0.0,1.0);
            const double d=std::hypot(e-s.x0-t*dx,n-s.y0-t*dy);
            if(d>s.half_width+s.shoulder+s.blend) continue;
            // Normalised distance gives a wider carriageway its proper footprint.
            const double score=d/(s.half_width+s.shoulder);
            if(score<distance || (score==distance && best && std::pair{s.way,s.start_ref}<std::pair{best->way,best->start_ref})) {
                distance=score; best=&s; fraction=t;
            }
        }
        if(!best) continue;
        const auto& s=*best; const double d=distance*(s.half_width+s.shoulder);
        // Rounded crown removes the centreline crease, while retaining camber.
        const double camber=s.crown*(std::sqrt(std::min(d,s.half_width)*std::min(d,s.half_width)+.25)-.5);
        const double road=s.z0+(s.z1-s.z0)*fraction-camber;
        const double core=s.half_width+s.shoulder;
        const double weight=d<=core ? 1 : 1-smooth((d-core)/std::max(.001,s.blend));
        const double height=raw/256.0+(road-raw/256.0)*weight;
        const auto quantized=static_cast<std::int32_t>(std::llround(height*256));
        if(quantized!=raw) { raw=quantized; ++changed; }
    }
    return changed;
}
}
