#include "rg/road_surface.h"
#include <algorithm>
#include <cmath>
#include <charconv>
#include <limits>
namespace rg {
namespace {
constexpr double bin_size = 16;
int bin(double value) { return static_cast<int>(std::floor(value / bin_size)); }
double smooth(double t) { t=std::clamp(t,0.0,1.0); return t*t*(3-2*t); }
// geo2map D59 (terrain_carve.cpp kUnderpassWallFadeM): past an underpass deck's end plane a carve segment's weight
// fades to zero over this distance, so the cutting under the deck keeps steep walls.
constexpr double kUnderpassWallFadeM = 1.0;
}
RoadSurfacePatch::RoadSurfacePatch(const g2m::RoadGeomTile& geometry,double verge_drop,const RoadSeparation& separation) : verge_drop_m_(verge_drop),separation_(separation) {
    decks=build_road_decks(geometry,separation);
    for(const auto& deck:decks) {
        if(deck->tunnel_floor)continue;
        const auto& m=deck->mesh;const auto rows=m.positions.size()/30;
        for(std::size_t i=1;i<rows;++i) {
            const auto a=(i-1)*15+6,b=i*15+6;
            const double total=deck->end_station-deck->start_station;
            DeckCut cut{m.origin[0]+m.positions[a],m.origin[1]+m.positions[a+1],m.positions[a+2],
                m.origin[0]+m.positions[b],m.origin[1]+m.positions[b+1],m.positions[b+2],
                deck->half_width,deck->crown,total*(i-1)/(rows-1),total};
            const auto index=deck_cuts_.size();deck_cuts_.push_back(cut);const double radius=cut.half_width+1;
            for(int y=bin(std::min(cut.y0,cut.y1)-radius);y<=bin(std::max(cut.y0,cut.y1)+radius);++y)
                for(int x=bin(std::min(cut.x0,cut.x1)-radius);x<=bin(std::max(cut.x0,cut.x1)+radius);++x)cut_bins_[{x,y}].push_back(index);
        }
    }
    for (std::size_t entry_index=0;entry_index<geometry.entries.size();++entry_index) {
        const auto& entry=geometry.entries[entry_index];
        if (!entry.profile) { ++declined; continue; }
        const auto& p = *entry.profile;
        int layer=0;
        const auto way=std::lower_bound(geometry.source.graph.ways.begin(),geometry.source.graph.ways.end(),entry.way_id,[](const auto& w,auto id){return w.osm_id<id;});
        if(way!=geometry.source.graph.ways.end()&&way->osm_id==entry.way_id) for(const auto& [key,value]:way->tags) if(key=="layer") {
            int parsed=0;const auto result=std::from_chars(value.data(),value.data()+value.size(),parsed);
            if(result.ec==std::errc{}&&result.ptr==value.data()+value.size()) layer=parsed;
        }
        if (p.attributes.bridge) { ++separated; continue; }
        if (p.attributes.land_class != g2m::LandClass::PavedRoad && !p.attributes.tunnel) continue;
        const int count = std::max(1,static_cast<int>(std::ceil(p.reference.length_m)));
        auto a=p.reference.at(0); auto za=p.at(0);
        if (!a.ok() || !za.ok()) continue;
        ++accepted;
        // End planes of this entry's served underpass decks (D59): {start station, start plane, end station, end plane}.
        struct Span { double start, end; DeckPlane at_start, at_end; };
        std::vector<Span> spans;
        for(const auto& u:geometry.underpasses) {
            if(u.entry_index!=entry_index) continue;
            const auto ps=p.reference.at(u.start_m),pe=p.reference.at(u.end_m);if(!ps.ok()||!pe.ok()) continue;
            spans.push_back({u.start_m,u.end_m,{true,ps.value().x,ps.value().y,std::cos(ps.value().heading),std::sin(ps.value().heading)},
                {true,pe.value().x,pe.value().y,-std::cos(pe.value().heading),-std::sin(pe.value().heading)}});
        }
        for (int i=1;i<=count;++i) {
            const double station = p.reference.length_m*i/count;
            auto b=p.reference.at(station); auto zb=p.at(station);
            if (!b.ok() || !zb.ok()) break;
            const bool covered=std::any_of(decks.begin(),decks.end(),[&](const auto& d){
                return d->inferred&&d->way_id==entry.way_id&&d->start_ref==entry.stretch.start_ref&&station>=d->start_station&&station<=d->end_station;
            });
            if(covered) {a=std::move(b);za=std::move(zb);continue;}
            // The piece's deck planes: the nearest underpass deck ending before this station and the nearest starting after it.
            DeckPlane lo{false,0,0,0,0},hi{false,0,0,0,0};double lo_end=-1,hi_start=std::numeric_limits<double>::infinity();
            for(const auto& span:spans) {
                if(span.end<station&&span.end>lo_end) {lo_end=span.end;lo=span.at_end;}
                if(span.start>station&&span.start<hi_start) {hi_start=span.start;hi=span.at_start;}
            }
            Segment s{a.value().x,a.value().y,za.value().height_m,
                b.value().x,b.value().y,zb.value().height_m,
                p.attributes.width_mm/2000.0,p.attributes.shoulder_mm/1000.0,
                p.attributes.blend_mm/1000.0,p.attributes.crown_per_mille/1000.0,
                entry.way_id,entry.stretch.start_ref,layer,p.attributes.tunnel,lo,hi};
            const double radius=s.half_width+s.shoulder+s.blend;
            const auto index=segments_.size(); segments_.push_back(s);
            for(int y=bin(std::min(s.y0,s.y1)-radius);y<=bin(std::max(s.y0,s.y1)+radius);++y)
                for(int x=bin(std::min(s.x0,s.x1)-radius);x<=bin(std::max(s.x0,s.x1)+radius);++x)
                    bins_[{x,y}].push_back(index);
            a=std::move(b); za=std::move(zb);
        }
    }
}
void RoadSurfacePatch::bed_points(std::vector<std::array<double, 3>>& out, int stride) const {
    const std::size_t step = static_cast<std::size_t>(stride < 1 ? 1 : stride);
    for (std::size_t i = 0; i < segments_.size(); i += step) {
        const Segment& s = segments_[i];
        if (s.tunnel) continue;
        out.push_back({s.x0, s.y0, s.z0});
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
        struct Candidate { const Segment* segment; double score, fraction, wall; };
        std::map<std::pair<std::int64_t,std::uint32_t>,Candidate> roads;
        for(auto index:candidates->second) {
            const auto& s=segments_[index]; const double dx=s.x1-s.x0,dy=s.y1-s.y0;
            const double length2=dx*dx+dy*dy; if(length2<1e-12) continue;
            const double t=std::clamp(((e-s.x0)*dx+(n-s.y0)*dy)/length2,0.0,1.0);
            const double d=std::hypot(e-s.x0-t*dx,n-s.y0-t*dy);
            if(d>s.half_width+s.shoulder+s.blend) continue;
            // D59: no carve past a served underpass deck's end plane (a 1 m fade keeps the carve continuous).
            double past=0;
            for(const auto* plane:{&s.deck_lo,&s.deck_hi}) if(plane->on) past=std::max(past,(e-plane->x)*plane->nx+(n-plane->y)*plane->ny);
            if(past>=kUnderpassWallFadeM) continue;
            const double wall=1-smooth(past/kUnderpassWallFadeM);
            // Normalised distance gives a wider carriageway its proper footprint.
            const double score=d/(s.half_width+s.shoulder);
            const auto id=std::pair{s.way,s.start_ref};
            auto found=roads.find(id);
            if(found==roads.end() || score<found->second.score) roads.insert_or_assign(id,Candidate{&s,score,t,wall});
        }
        if(roads.empty()) continue;
        // Each nearby road's own surface height at this cell (camber, verge drop; no terrain blend yet).
        struct Road { const Segment* segment; double score, fraction, d, z, wall; bool protects, kept; };
        std::vector<Road> near; near.reserve(roads.size());
        for(const auto& [id,c]:roads) {
            (void)id;const auto& r=*c.segment;const double d=c.score*(r.half_width+r.shoulder),lateral=std::min(d,r.half_width);
            const double z=r.z0+(r.z1-r.z0)*c.fraction-r.crown*(std::sqrt(lateral*lateral+.25)-.5)-verge_drop_m_*smooth((d-r.half_width)/std::max(.001,r.shoulder));
            near.push_back({&r,c.score,c.fraction,d,z,c.wall,!r.tunnel&&d<=r.half_width+r.shoulder+separation_.margin_m,true});
        }
        // Grade separation (RoadSeparation): inside a road's protected footprint, a road more than clearance_m above it
        // is on a structure here and must not lift the terrain, and one more than clearance_m below it (that does not
        // protect this cell itself) must not dig into it. Where two such roads both protect the cell, the lower wins:
        // the upper one cannot stand on ground over a road that is driven underneath.
        for(auto& b:near) {
            if(b.segment->tunnel) continue;
            for(const auto& a:near) {
                if(&a==&b||!a.protects) continue;
                if(b.z>a.z+separation_.clearance_m||(b.z<a.z-separation_.clearance_m&&!b.protects)){b.kept=false;break;}
            }
        }
        const Road* best=nullptr;
        for(const auto& r:near) if(r.kept&&(!best||r.score<best->score)) best=&r; // map order: ties go to the smaller id
        if(!best) continue;
        const auto& s=*best->segment; const double distance=best->score, d=best->d;
        const double road=best->z;
        const double core=s.half_width+s.shoulder;
        const double weight=(d<=core ? 1 : 1-smooth((d-core)/std::max(.001,s.blend)))*best->wall;
        double height=raw/256.0+(road-raw/256.0)*weight;
        // Nearest-road ownership is discontinuous where ramps merge. Blend
        // the nearest sample from each road across that ownership boundary,
        // keeping grade-separated layers/floors separate and preserving the
        // original profile away from overlapping road footprints.
        double sum=0,weights=0;
        constexpr double transition=.35;
        for(const auto& candidate:near) {
            if(!candidate.kept) continue;
            const auto& other=*candidate.segment;
            if(other.layer!=s.layer || other.tunnel!=s.tunnel) continue;
            const double proximity=1-smooth((candidate.score-distance)/transition);
            if(proximity<=0) continue;
            const double oc=other.half_width+other.shoulder;
            const double influence=(candidate.d<=oc?1:1-smooth((candidate.d-oc)/std::max(.001,other.blend)))*candidate.wall;
            sum+=proximity*(raw/256.0+(candidate.z-raw/256.0)*influence);weights+=proximity;
        }
        if(weights>0) height=sum/weights;
        const auto quantized=static_cast<std::int32_t>(std::llround(height*256));
        if(quantized!=raw) { raw=quantized; ++changed; }
    }
    // Bare-earth terrain must not protrude through a separately collidable slab.
    // Keep the approach surface flush, then leave clearance below its underside.
    for(int y=0;y<256;++y)for(int x=0;x<256;++x) {
        auto& raw=tile.h[static_cast<std::size_t>(y)*256+x];if(raw==g2m::kHeightNoData)continue;
        const double e=tile.key.min_easting()+(x+.5)*spacing,n=tile.key.min_northing()+(y+.5)*spacing;
        const auto candidates=cut_bins_.find({bin(e),bin(n)});if(candidates==cut_bins_.end())continue;
        double ceiling=raw/256.0;
        for(auto index:candidates->second) {
            const auto& c=deck_cuts_[index];const double dx=c.x1-c.x0,dy=c.y1-c.y0,l2=dx*dx+dy*dy;if(l2<1e-12)continue;
            const double t=((e-c.x0)*dx+(n-c.y0)*dy)/l2;if(t<0||t>1)continue;
            const double lateral=std::abs((e-c.x0)*dy-(n-c.y0)*dx)/std::sqrt(l2);
            if(lateral>c.half_width+1)continue;
            const double station=c.from+t*std::sqrt(l2),edge=std::min(station,c.total-station);
            const double top=c.z0+(c.z1-c.z0)*t-c.crown*(std::sqrt(lateral*lateral+.25)-.5);
            ceiling=std::min(ceiling,top-.55*smooth(edge/4.0));
        }
        const auto quantized=static_cast<std::int32_t>(std::floor(ceiling*256));
        if(quantized<raw){raw=quantized;++changed;}
    }
    return changed;
}
}
