#include "rg/road_structures.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
namespace rg {
namespace {
const g2m::RoadGraphWay* way(const g2m::RoadGraph& g,std::int64_t id) {
    auto it=std::lower_bound(g.ways.begin(),g.ways.end(),id,[](const auto& w,auto value){return w.osm_id<value;});
    return it==g.ways.end()||it->osm_id!=id?nullptr:&*it;
}
g2m::geom::PointMm node(const g2m::RoadGraph& g,std::int64_t id) {
    auto it=std::lower_bound(g.nodes.begin(),g.nodes.end(),id,[](const auto& n,auto value){return n.osm_id<value;});
    return it->position;
}
int layer(const g2m::RoadGraphWay& w) {
    for(const auto& [key,value]:w.tags) if(key=="layer") {
        if(value=="-1") return -1;if(value=="-2") return -2;
        if(value=="1") return 1;if(value=="2") return 2;
    }
    return 0;
}
// Find supported ground away from the bare-earth hole. Degree-two bridges
// can be split over several OSM ways, and a nearby junction is not a reason
// to lose the whole structure: continue the most aligned ground approach.
std::optional<g2m::RoadJunctionPlane> abutment(const g2m::RoadGraph& g,const g2m::RoadDemSampler& dem,
    g2m::geo::UtmZone zone,std::int64_t id,double out_x,double out_y) {
    const auto origin=node(g,id);auto current=id;std::int64_t previous=0;double remaining=32;
    for(int work=0;work<100;++work) {
        const g2m::RoadGraphEdge* selected=nullptr;double score=.25;
        for(const auto& edge:g.edges) {
            if(edge.from_node!=current&&edge.to_node!=current) continue;
            const auto other=edge.from_node==current?edge.to_node:edge.from_node;if(other==previous) continue;
            const auto* w=way(g,edge.way_id);if(!w) continue;
            auto attrs=g2m::resolve_road_attributes(*w);if(!attrs||attrs->tunnel) continue;
            const auto a=node(g,current),b=node(g,other);
            const double dx=(b.x-a.x)/1000.0,dy=(b.y-a.y)/1000.0,len=std::hypot(dx,dy);
            if(len<.001) continue;
            const double dot=(dx*out_x+dy*out_y)/len;
            if(dot>score) {score=dot;selected=&edge;}
        }
        if(!selected) break;
        const auto other=selected->from_node==current?selected->to_node:selected->from_node;
        const auto a=node(g,current),b=node(g,other);
        const double dx=(b.x-a.x)/1000.0,dy=(b.y-a.y)/1000.0,len=std::hypot(dx,dy);
        const auto chosen_attrs=g2m::resolve_road_attributes(*way(g,selected->way_id));
        const bool on_bridge=chosen_attrs&&chosen_attrs->bridge;
        if(!on_bridge&&len>=remaining) {
            const double t=remaining/len;
            const g2m::geom::PointMm support{std::llround(a.x+t*(b.x-a.x)),std::llround(a.y+t*(b.y-a.y))};
            auto plane=dem.junction_plane(zone,support);if(!plane) return {};
            const double h=plane->height/256.0+plane->grade_x*(origin.x-support.x)/1000.0+plane->grade_y*(origin.y-support.y)/1000.0;
            plane->height=static_cast<std::int32_t>(std::llround(h*256));return *plane;
        }
        if(!on_bridge) remaining-=len;out_x=dx/len;out_y=dy/len;previous=current;current=other;
    }
    return {};
}
std::shared_ptr<const RoadDeck> deck(const g2m::RoadProfile& profile,std::int64_t id,
    double from,double to,bool inferred) {
    if(to-from<.5||to-from>1000) return {};
    auto output=std::make_shared<RoadDeck>();output->way_id=id;output->start_station=from;output->end_station=to;
    output->tunnel_floor=profile.attributes.tunnel;
    output->inferred=inferred;output->land_class=profile.attributes.land_class;
    auto first=profile.reference.at(from);if(!first) return {};
    output->mesh.origin[0]=first->x;output->mesh.origin[1]=first->y;
    const int count=std::max(1,static_cast<int>(std::ceil(to-from)));
    const double half=profile.attributes.width_mm/2000.0;
    auto& m=output->mesh;
    for(int i=0;i<=count;++i) {
        const double station=from+(to-from)*i/count;
        auto xy=profile.reference.at(station);auto z=profile.at(station);if(!xy||!z) return {};
        const double nx=-std::sin(xy->heading),ny=std::cos(xy->heading);
        // Five vertices across the carriageway give a rounded crown and
        // identical geometry to collision, rather than a visible-only slab.
        for(int j=0;j<5;++j) {
            const double offset=half*(j-2)/2;
            const double camber=profile.attributes.crown_per_mille/1000.0*(std::sqrt(offset*offset+.25)-.5);
            m.positions.insert(m.positions.end(),{static_cast<float>(xy->x+nx*offset-m.origin[0]),static_cast<float>(xy->y+ny*offset-m.origin[1]),static_cast<float>(z->height_m-camber)});
            const double cross_grade=-profile.attributes.crown_per_mille/1000.0*offset/std::sqrt(offset*offset+.25);
            double gx=z->grade*std::cos(xy->heading)+cross_grade*nx,gy=z->grade*std::sin(xy->heading)+cross_grade*ny;
            const double scale=1/std::sqrt(1+gx*gx+gy*gy);
            m.normals.insert(m.normals.end(),{static_cast<float>(-gx*scale),static_cast<float>(-gy*scale),static_cast<float>(scale)});
            m.land_class.push_back(static_cast<std::uint8_t>(output->land_class));
        }
    }
    for(int i=0;i<count;++i) for(int j=0;j<4;++j) {
        const auto a=static_cast<std::uint16_t>(i*5+j),b=static_cast<std::uint16_t>((i+1)*5+j);
        m.indices.insert(m.indices.end(),{a,b,static_cast<std::uint16_t>(a+1),static_cast<std::uint16_t>(a+1),b,static_cast<std::uint16_t>(b+1)});
    }
    output->min_easting=output->max_easting=m.origin[0]+m.positions[0];
    output->min_northing=output->max_northing=m.origin[1]+m.positions[1];
    for(std::size_t v=0;v<m.positions.size();v+=3) {
        output->min_easting=std::min(output->min_easting,m.origin[0]+m.positions[v]);
        output->max_easting=std::max(output->max_easting,m.origin[0]+m.positions[v]);
        output->min_northing=std::min(output->min_northing,m.origin[1]+m.positions[v+1]);
        output->max_northing=std::max(output->max_northing,m.origin[1]+m.positions[v+1]);
    }
    // A half-metre structural slab supplies visible fascia and a ceiling
    // collider beneath the same road surface. No pillars are invented in
    // the lower roadway's footprint.
    const auto top_vertices=m.positions.size()/3;
    const auto top_indices=m.indices;
    for(std::size_t i=0;i<top_vertices;++i) {
        m.positions.insert(m.positions.end(),{m.positions[i*3],m.positions[i*3+1],m.positions[i*3+2]-.5f});
        m.normals.insert(m.normals.end(),{0,0,-1});m.land_class.push_back(static_cast<std::uint8_t>(output->land_class));
    }
    for(std::size_t i=0;i<top_indices.size();i+=3) m.indices.insert(m.indices.end(),{
        static_cast<std::uint16_t>(top_vertices+top_indices[i]),static_cast<std::uint16_t>(top_vertices+top_indices[i+2]),static_cast<std::uint16_t>(top_vertices+top_indices[i+1])});
    for(int i=0;i<count;++i) for(int side:{0,4}) {
        const auto a=static_cast<std::uint16_t>(i*5+side),b=static_cast<std::uint16_t>((i+1)*5+side);
        const auto c=static_cast<std::uint16_t>(b+top_vertices),d=static_cast<std::uint16_t>(a+top_vertices);
        if(side==0) m.indices.insert(m.indices.end(),{a,c,b,a,d,c});
        else m.indices.insert(m.indices.end(),{a,b,c,a,c,d});
    }
    return output;
}
}
void complete_road_profiles(g2m::RoadGeomTile& geometry,const g2m::RoadDemSampler& dem) {
    const auto& graph=geometry.source.graph;auto builder=g2m::RoadReferenceBuilder::make(graph);if(!builder) return;
    const auto zone=geometry.source.key.zone;
    std::map<std::int64_t,g2m::RoadJunctionPlane> anchors;
    // Existing accepted bridge anchors are authoritative for adjoining ground.
    for(const auto& entry:geometry.entries) if(entry.profile&&entry.profile->attributes.bridge) {
        const auto* w=way(graph,entry.way_id);if(!w) continue;
        const auto& p=*entry.profile;
        for(int end=0;end<2;++end) {
            const double station=end?p.reference.length_m:0;auto xy=p.reference.at(station);auto z=p.at(station);
            if(xy&&z) anchors[w->node_ids[end?entry.stretch.end_ref:entry.stretch.start_ref]]={static_cast<std::int32_t>(std::llround(z->height_m*256)),z->grade*std::cos(xy->heading),z->grade*std::sin(xy->heading)};
        }
    }
    // Tunnel floors use the lower road's DEM, while their roofs are supplied
    // by the upper road below. They never paint their floor onto the highway.
    for(auto& entry:geometry.entries) {
        const auto* w=way(graph,entry.way_id);if(!w||entry.profile) continue;
        auto attrs=g2m::resolve_road_attributes(*w);if(!attrs||!attrs->tunnel) continue;
        g2m::ReferenceFitLimits limits;limits.allow_curvature_exceptions=true;
        auto ref=builder->fit_stretch(entry.way_id,entry.stretch,limits);if(!ref||ref->length_m<.5) continue;
        const int count=std::max(1,static_cast<int>(std::ceil(ref->length_m/5)));if(count>2047) continue;
        std::vector<g2m::VerticalSample> samples;bool complete=true;
        for(int i=0;i<=count;++i) {
            const double station=ref->length_m*i/count;auto xy=ref->at(station);if(!xy){complete=false;break;}
            auto h=dem.sample(zone,{std::llround(xy->x*1000),std::llround(xy->y*1000)});if(!h){complete=false;break;}
            samples.push_back({station,*h/256.0});
        }
        if(!complete) continue;
        g2m::VerticalFitLimits fit;fit.max_grade=.3;fit.min_radius_m=20;fit.max_iterations=16384;
        fit.start_grade=std::clamp((samples[1].height_m-samples[0].height_m)/(samples[1].station_m-samples[0].station_m),-.3,.3);
        fit.end_grade=std::clamp((samples.back().height_m-samples[samples.size()-2].height_m)/(samples.back().station_m-samples[samples.size()-2].station_m),-.3,.3);
        auto vertical=g2m::fit_vertical_profile(samples,fit);if(!vertical) continue;
        g2m::RoadProfile p;p.stretch=entry.stretch;p.attributes=*attrs;p.reference=std::move(*ref);p.vertical=std::move(*vertical);
        entry.profile=std::move(p);entry.decline.reset();
    }
    for(auto& entry:geometry.entries) {
        const auto* w=way(graph,entry.way_id);if(!w||entry.profile) continue;
        auto attrs=g2m::resolve_road_attributes(*w);if(!attrs||!attrs->bridge) continue;
        g2m::ReferenceFitLimits limits;limits.allow_curvature_exceptions=true;
        auto ref=builder->fit_stretch(entry.way_id,entry.stretch,limits);if(!ref) continue;
        auto start=ref->at(0),end=ref->at(ref->length_m);if(!start||!end) continue;
        auto a=abutment(graph,dem,zone,w->node_ids[entry.stretch.start_ref],-std::cos(start->heading),-std::sin(start->heading));
        auto b=abutment(graph,dem,zone,w->node_ids[entry.stretch.end_ref],std::cos(end->heading),std::sin(end->heading));
        if(anchors.contains(w->node_ids[entry.stretch.start_ref])) a=anchors[w->node_ids[entry.stretch.start_ref]];
        if(anchors.contains(w->node_ids[entry.stretch.end_ref])) b=anchors[w->node_ids[entry.stretch.end_ref]];
        if(!a||!b) {std::fprintf(stderr,"RG_STRUCTURE bridge_declined way=%lld support=%d:%d\n",static_cast<long long>(entry.way_id),a.has_value(),b.has_value());continue;}
        g2m::VerticalFitLimits fit;fit.max_grade=std::max(.12,attrs->max_grade_per_mille/1000.0);
        fit.min_radius_m=std::min(250,attrs->min_vertical_radius_m);fit.max_iterations=16384;
        fit.start_grade=a->grade_x*std::cos(start->heading)+a->grade_y*std::sin(start->heading);
        fit.end_grade=b->grade_x*std::cos(end->heading)+b->grade_y*std::sin(end->heading);
        const std::array<g2m::VerticalSample,2> samples{{{0,a->height/256.0},{ref->length_m,b->height/256.0}}};
        auto vertical=g2m::fit_vertical_profile(samples,fit);
        if(!vertical) {
            // Estimated DEM gradients at a short span can disagree. Its
            // abutment rise remains measured; choose the common secant grade.
            fit.start_grade=fit.end_grade=(b->height-a->height)/256.0/ref->length_m;
            vertical=g2m::fit_vertical_profile(samples,fit);if(!vertical) continue;
            a->grade_x=fit.start_grade*std::cos(start->heading);a->grade_y=fit.start_grade*std::sin(start->heading);
            b->grade_x=fit.end_grade*std::cos(end->heading);b->grade_y=fit.end_grade*std::sin(end->heading);
        }
        g2m::RoadProfile p;p.stretch=entry.stretch;p.attributes=*attrs;p.reference=std::move(*ref);p.vertical=std::move(*vertical);
        p.start_anchor_inferred=p.end_anchor_inferred=true;
        anchors[w->node_ids[entry.stretch.start_ref]]=*a;anchors[w->node_ids[entry.stretch.end_ref]]=*b;
        entry.profile=std::move(p);entry.decline.reset();
        std::fprintf(stderr,"RG_STRUCTURE bridge_reconstructed way=%lld\n",static_cast<long long>(entry.way_id));
    }
    for(auto& entry:geometry.entries) {
        const auto* w=way(graph,entry.way_id);if(!w) continue;
        auto attrs=g2m::resolve_road_attributes(*w);if(!attrs||attrs->bridge||attrs->tunnel) continue;
        const auto first_id=w->node_ids[entry.stretch.start_ref],last_id=w->node_ids[entry.stretch.end_ref];
        if(entry.profile&&!anchors.contains(first_id)&&!anchors.contains(last_id)) continue;
        g2m::ReferenceFitLimits limits;limits.allow_curvature_exceptions=true;
        auto ref=builder->fit_stretch(entry.way_id,entry.stretch,limits);if(!ref||ref->length_m<.5) continue;
        auto a=dem.junction_plane(zone,node(graph,first_id)),b=dem.junction_plane(zone,node(graph,last_id));
        if(anchors.contains(first_id)) a=anchors[first_id];if(anchors.contains(last_id)) b=anchors[last_id];
        if(!a||!b) continue;
        auto start=ref->at(0),end=ref->at(ref->length_m);if(!start||!end) continue;
        g2m::VerticalFitLimits fit;fit.max_grade=std::max(.12,attrs->max_grade_per_mille/1000.0);
        fit.min_radius_m=std::min(500,attrs->min_vertical_radius_m);fit.max_iterations=16384;
        fit.start_grade=a->grade_x*std::cos(start->heading)+a->grade_y*std::sin(start->heading);
        fit.end_grade=b->grade_x*std::cos(end->heading)+b->grade_y*std::sin(end->heading);
        fit.start_grade=std::clamp(fit.start_grade,-.10,.10);
        fit.end_grade=std::clamp(fit.end_grade,-.10,.10);
        const int count=std::max(1,static_cast<int>(std::ceil(ref->length_m/40)));
        if(count>2047) continue;
        std::vector<g2m::VerticalSample> samples{{0,a->height/256.0}};bool complete=true;
        for(int i=1;i<count;++i) {
            const double station=ref->length_m*i/count;auto xy=ref->at(station);if(!xy){complete=false;break;}
            auto height=dem.junction_height(zone,{std::llround(xy->x*1000),std::llround(xy->y*1000)});
            if(!height){complete=false;break;}
            double z=*height/256.0;
            // Carry inferred abutments to supported ground, keeping ramps out
            // of the DEM's underpass depression on either side of the span.
            if(anchors.contains(first_id)&&station<32) z=a->height/256.0+fit.start_grade*station;
            if(anchors.contains(last_id)&&ref->length_m-station<32) z=b->height/256.0-fit.end_grade*(ref->length_m-station);
            samples.push_back({station,z});
        }
        if(!complete) continue;samples.push_back({ref->length_m,b->height/256.0});
        auto vertical=g2m::fit_vertical_profile(samples,fit);if(!vertical) {if(w->rank>=8) std::fprintf(stderr,"RG_STRUCTURE ground_declined way=%lld length=%.1f grades=%.4f:%.4f reason=%s\n",static_cast<long long>(entry.way_id),ref->length_m,fit.start_grade,fit.end_grade,vertical.error().message.c_str());continue;}
        g2m::RoadProfile p;p.stretch=entry.stretch;p.attributes=*attrs;p.reference=std::move(*ref);p.vertical=std::move(*vertical);
        entry.profile=std::move(p);entry.decline.reset();
        std::fprintf(stderr,"RG_STRUCTURE ground_reconstructed way=%lld\n",static_cast<long long>(entry.way_id));
    }
}
std::vector<std::shared_ptr<const RoadDeck>> build_road_decks(const g2m::RoadGeomTile& geometry) {
    std::vector<std::shared_ptr<const RoadDeck>> result;
    for(const auto& entry:geometry.entries) if(entry.profile&&(entry.profile->attributes.bridge||entry.profile->attributes.tunnel)) {
        if(auto d=deck(*entry.profile,entry.way_id,0,entry.profile->reference.length_m,false)) result.push_back(std::move(d));
    }
    const auto& graph=geometry.source.graph;
    for(const auto& lower:graph.ways) {
        auto attrs=g2m::resolve_road_attributes(lower);if(!attrs||!attrs->tunnel) continue;
        for(const auto& entry:geometry.entries) {
            if(!entry.profile||entry.profile->attributes.bridge||entry.profile->attributes.tunnel) continue;
            const auto* upper=way(graph,entry.way_id);if(!upper||upper->rank<5||layer(*upper)<=layer(lower)) continue;
            const auto& p=*entry.profile;
            for(std::size_t i=1;i<lower.node_ids.size();++i) {
                const auto a=node(graph,lower.node_ids[i-1]),b=node(graph,lower.node_ids[i]);
                const double ax=a.x/1000.0,ay=a.y/1000.0,dx=(b.x-a.x)/1000.0,dy=(b.y-a.y)/1000.0;
                for(const auto& segment:p.reference.segments) {
                    auto begin=segment.curve.at(0),finish=segment.curve.at(segment.curve.length);if(!begin||!finish) continue;
                    const double ux=finish->x-begin->x,uy=finish->y-begin->y;
                    const double determinant=ux*dy-uy*dx;if(std::abs(determinant)<1e-6) continue;
                    const double vx=ax-begin->x,vy=ay-begin->y;
                    const double t=(vx*dy-vy*dx)/determinant,u=(vx*uy-vy*ux)/determinant;
                    if(t<0||t>1||u<0||u>1) continue;
                    const double station=segment.station_m+t*segment.curve.length;
                    const double sine=std::abs(determinant)/std::max(.001,std::hypot(ux,uy)*std::hypot(dx,dy));
                    const double extent=std::max(20.0,(attrs->width_mm/2000.0+4)/std::max(.2,sine));
                    const double from=std::max(0.0,station-extent),to=std::min(p.reference.length_m,station+extent);
                    // Infer the roof from the upper road's supported ends,
                    // never from the lower tunnel's bare-earth floor.
                    auto z0=p.at(from),z1=p.at(to);if(!z0||!z1) continue;
                    auto upper_profile=p;upper_profile.vertical.segments={{0,p.reference.length_m,
                        z0->height_m-(from)*(z1->height_m-z0->height_m)/(to-from),
                        z1->height_m+(p.reference.length_m-to)*(z1->height_m-z0->height_m)/(to-from),
                        (z1->height_m-z0->height_m)/(to-from),(z1->height_m-z0->height_m)/(to-from)}};
                    if(auto d=deck(upper_profile,entry.way_id,from,to,true)) {
                        result.push_back(std::move(d));std::fprintf(stderr,"RG_STRUCTURE tunnel_roof upper=%lld lower=%lld\n",static_cast<long long>(entry.way_id),static_cast<long long>(lower.osm_id));
                    }
                }
            }
        }
    }
    return result;
}
}
