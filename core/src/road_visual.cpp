#include "rg/road_visual.h"
#include "rg/road_structures.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <array>
#include <map>
#include <optional>
#include <tuple>
#include <vector>
namespace rg {
namespace {
std::string tag(const g2m::RoadGraphWay& way,std::string_view key) {for(const auto& [k,v]:way.tags)if(k==key)return v;return {};}
int integer(const std::string& s) {int n=0;auto r=std::from_chars(s.data(),s.data()+s.size(),n);return r.ec==std::errc{}&&r.ptr==s.data()+s.size()&&n>=0&&n<=30?n:0;}

constexpr double kJunctionLiftM=0.004; // above the ribbons (lift) and the turn arrows (+3 mm)
// ---- junction surfaces (geo2map roads.geom encoding 7, design D44 kerb returns) ----
struct P2 { double x,y; };
double cross(P2 o,P2 a,P2 b) {return (a.x-o.x)*(b.y-o.y)-(a.y-o.y)*(b.x-o.x);}
// Ear clipping of a simple counter-clockwise ring. Collinear and repeated vertices are tolerated; a ring that cannot be
// clipped completely (numerically degenerate) returns the triangles found so far.
std::vector<std::array<P2,3>> triangulate(std::vector<P2> ring) {
 std::vector<std::array<P2,3>> out;
 const auto inside=[](P2 p,P2 a,P2 b,P2 c){return cross(a,b,p)>=0&&cross(b,c,p)>=0&&cross(c,a,p)>=0;};
 const std::size_t limit=ring.size()+16;std::size_t guard=0,start=0; // every pass removes one vertex or stops
 while(ring.size()>3&&guard++<limit) {
  bool clipped=false;
  for(std::size_t step=0;step<ring.size();++step) {
   const std::size_t i=(start+step)%ring.size();
   const std::size_t ia=(i+ring.size()-1)%ring.size(),ic=(i+1)%ring.size();
   const P2 a=ring[ia],b=ring[i],c=ring[ic];const double turn=cross(a,b,c);
   if(turn<0)continue;
   if(turn==0){ring.erase(ring.begin()+static_cast<std::ptrdiff_t>(i));start=i;clipped=true;break;} // collinear: drop the vertex
   bool ear=true;
   for(std::size_t k=0;k<ring.size()&&ear;++k)if(k!=ia&&k!=i&&k!=ic&&inside(ring[k],a,b,c)&&!(ring[k].x==a.x&&ring[k].y==a.y)&&!(ring[k].x==c.x&&ring[k].y==c.y))ear=false;
   if(!ear)continue;
   // Resume next to the clipped ear: one pass per ear on typical rings instead of a rescan from vertex 0.
   out.push_back({a,b,c});ring.erase(ring.begin()+static_cast<std::ptrdiff_t>(i));start=i;clipped=true;break;
  }
  if(!clipped)break;
 }
 if(ring.size()==3&&cross(ring[0],ring[1],ring[2])>0)out.push_back({ring[0],ring[1],ring[2]});
 return out;
}
// Convex polygon clipped to x >= v (keep = +1) or x <= v (keep = -1) on axis 0 (x) or 1 (y).
std::vector<P2> clip(const std::vector<P2>& poly,int axis,double v,int keep) {
 std::vector<P2> out;const auto d=[&](P2 p){return keep*((axis?p.y:p.x)-v);};
 for(std::size_t i=0;i<poly.size();++i) {
  const P2 a=poly[i],b=poly[(i+1)%poly.size()];const double da=d(a),db=d(b);
  if(da>=0)out.push_back(a);
  if((da<0)!=(db<0)){const double t=da/(da-db);P2 q{a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t};if(axis)q.y=v;else q.x=v;out.push_back(q);}
 }
 return out;
}
}
// Every junction this tile owns, as an asphalt patch over the road bed ring: ear clipping, then each triangle cut by the
// terrain lattice lines (DGM1 cell centres, x.5 m) so the patch bends exactly where the terrain does and neighbouring
// pieces share their vertices (no T-junctions). Heights come from `ground`; patches sit `lift` above it, a little above
// the ribbons, so the square ribbon ends under the junction disappear. No markings (flags 0, fade 0).
void add_junction_surfaces(RoadVisualMesh& mesh,const g2m::RoadGeomTile& geometry,const g2m::TileKey& key,const RoadVisualGround& ground,double lift) {
 const double ox=key.min_easting(),oy=key.min_northing();
 for(const auto& j:geometry.junctions) {
  if(j.decline!=g2m::JunctionDecline::None||(j.flags&static_cast<std::uint16_t>(g2m::JunctionFlags::OnStructure)))continue;
  const auto& v=j.rings[static_cast<std::size_t>(g2m::JunctionRingLayer::RoadBed)].vertices;if(v.size()<3)continue;
  std::vector<P2> ring;ring.reserve(v.size());
  for(const auto& q:v)ring.push_back({static_cast<double>(q.x)/1000.0-ox,static_cast<double>(q.y)/1000.0-oy});
  const auto first=mesh.vertices.size();const auto first_index=mesh.indices.size();bool valid=true;
  std::map<std::pair<std::int64_t,std::int64_t>,std::int32_t> shared; // vertex dedup on a 0.1 mm key
  const auto vertex=[&](P2 p)->std::optional<std::int32_t>{
   const std::pair<std::int64_t,std::int64_t> k{std::llround(p.x*1e4),std::llround(p.y*1e4)};
   if(auto it=shared.find(k);it!=shared.end())return it->second;
   const auto h=ground(p.x+ox,p.y+oy);if(!h)return {};
   const auto index=static_cast<std::int32_t>(mesh.vertices.size());shared.emplace(k,index);
   mesh.vertices.push_back({static_cast<float>(p.x),static_cast<float>(p.y),static_cast<float>(*h+lift+kJunctionLiftM),0,0,1000.0f,0,0,0,0});
   return index;
  };
  for(const auto& t:triangulate(ring)) {
   const double x0=std::min({t[0].x,t[1].x,t[2].x}),x1=std::max({t[0].x,t[1].x,t[2].x});
   const double y0=std::min({t[0].y,t[1].y,t[2].y}),y1=std::max({t[0].y,t[1].y,t[2].y});
   for(double gx=std::floor(x0-.5)+.5;gx<x1&&valid;gx+=1)
    for(double gy=std::floor(y0-.5)+.5;gy<y1&&valid;gy+=1) {
     std::vector<P2> cell(t.begin(),t.end());
     cell=clip(cell,0,gx,1);cell=clip(cell,0,gx+1,-1);cell=clip(cell,1,gy,1);cell=clip(cell,1,gy+1,-1);
     if(cell.size()<3)continue;
     std::vector<std::int32_t> ids;
     for(const auto& p:cell){auto id=vertex(p);if(!id){valid=false;break;}if(ids.empty()||ids.back()!=*id)ids.push_back(*id);}
     if(!valid)break;
     if(ids.size()>1&&ids.front()==ids.back())ids.pop_back();
     for(std::size_t i=1;i+1<ids.size();++i)mesh.indices.insert(mesh.indices.end(),{ids[0],ids[i],ids[i+1]});
    }
   if(!valid)break;
  }
  if(!valid){mesh.vertices.resize(first);mesh.indices.resize(first_index);}
 }
}
namespace {
}
RoadVisualMesh build_road_visual(const g2m::RoadGeomTile& geometry,const g2m::TileKey& key,const RoadVisualGround& ground,double lift) {
 RoadVisualMesh mesh;const auto decks=build_road_decks(geometry);
 const auto deck_height=[&](std::int64_t way,double station,double lateral)->std::optional<double>{
  for(const auto& d:decks)if(d->way_id==way)if(auto z=road_deck_height(*d,station,lateral))return z;return {};
 };
 // Trims of the junctions this tile owns, per (way, stretch start_ref, at_end): lane markings end at the trim line.
 std::map<std::tuple<std::int64_t,std::uint32_t,bool>,double> trims;
 for(const auto& j:geometry.junctions)if(j.decline==g2m::JunctionDecline::None)
  for(const auto& a:j.approaches)if(a.status==g2m::JunctionApproachStatus::Used&&a.entry_index<geometry.entries.size()){
   const auto& e=geometry.entries[a.entry_index];trims[{e.way_id,e.stretch.start_ref,a.at_end}]=a.trim_m;}
 std::map<std::int64_t,int> degree;for(const auto& edge:geometry.source.graph.edges){++degree[edge.from_node];++degree[edge.to_node];}
 const double ox=key.min_easting(),oy=key.min_northing();
 for(const auto& entry:geometry.entries) {
  if(!entry.profile)continue;const auto& p=*entry.profile;if(p.attributes.land_class!=g2m::LandClass::PavedRoad)continue;
  auto it=std::lower_bound(geometry.source.graph.ways.begin(),geometry.source.graph.ways.end(),entry.way_id,[](const auto& w,auto id){return w.osm_id<id;});
  if(it==geometry.source.graph.ways.end()||it->osm_id!=entry.way_id)continue;const auto& w=*it;
  const auto highway=tag(w,"highway");if(highway=="footway"||highway=="path"||highway=="cycleway"||highway=="steps"||highway=="pedestrian")continue;
  const auto direction=tag(w,"oneway");const bool one=direction=="yes"||direction=="1"||direction=="true"||direction=="-1"||(direction.empty()&&(highway=="motorway"||tag(w,"junction")=="roundabout"));
  const bool fast=highway=="motorway"||highway=="trunk"||highway=="motorway_link"||highway=="trunk_link";
  const bool edge_lines=fast||highway=="primary"||highway=="secondary"||highway=="tertiary";
  const double width=p.attributes.width_mm/1000.0,half=width*.5;
  int lanes=p.attributes.lanes;const int f=integer(tag(w,"lanes:forward")),b=integer(tag(w,"lanes:backward"));
  if(lanes==0&&f+b>0)lanes=f+b;
  if(lanes==0)lanes=one?std::clamp(static_cast<int>(std::round(width/3.25)),1,fast?4:2):(width>=5.5?2:1);
  lanes=std::clamp(lanes,1,30);int forward=one?(direction=="-1"?0:lanes):(f>0?f:(b>0?lanes-b:(lanes+1)/2));forward=std::clamp(forward,0,lanes);
  const bool marking=tag(w,"lane_markings")!="no"&&width/lanes>=2.4&&highway!="living_street";
  const bool solid=tag(w,"overtaking")=="no";
  const int flags=(marking?1:0)|(edge_lines?2:0)|(solid?4:0)|(fast?8:0);
  const double length=p.reference.length_m;if(length<=.01)continue;
  const bool start_junction=p.stretch.start_ref<w.node_ids.size()&&degree[w.node_ids[p.stretch.start_ref]]>2;
  const bool end_junction=p.stretch.end_ref<w.node_ids.size()&&degree[w.node_ids[p.stretch.end_ref]]>2;
  // A junction served for this stretch end (owner tile only) gives the exact trim; elsewhere the 3-11 m ramp stays.
  const auto trim_of=[&](bool at_end)->std::optional<double>{auto t=trims.find({entry.way_id,entry.stretch.start_ref,at_end});if(t==trims.end())return {};return t->second;};
  const auto start_trim=trim_of(false),end_trim=trim_of(true);
  const int count=std::max(1,static_cast<int>(std::ceil(length))); // metre spacing matches the terrain detail.
  const int across=std::max(2,static_cast<int>(std::ceil(width)));const double shoulder=.30;
  for(int i=0;i<count;++i) {
   const double stations[2]={length*i/count,length*(i+1)/count};auto mid=p.reference.at((stations[0]+stations[1])*.5);if(!mid)continue;
   if(std::floor(mid->x/1024)!=key.x||std::floor(mid->y/1024)!=key.y)continue;
   const auto first=mesh.vertices.size();bool valid=true;
   for(double station:stations) {
    auto xy=p.reference.at(station);auto z=p.at(station);if(!xy||!z){valid=false;break;}
    const double nx=-std::sin(xy->heading),ny=std::cos(xy->heading);
    const bool separated=p.attributes.bridge||std::any_of(decks.begin(),decks.end(),[&](const auto& d){return d->way_id==entry.way_id&&station>=d->start_station&&station<=d->end_station;});
    const double extent=half+(separated?0:shoulder);
    double fade=1;
    if(start_trim)fade=std::min(fade,std::clamp((station-*start_trim)/2.0,0.0,1.0));
    else if(start_junction)fade=std::min(fade,std::clamp((station-3)/8.0,0.0,1.0));
    if(end_trim)fade=std::min(fade,std::clamp((length-station-*end_trim)/2.0,0.0,1.0));
    else if(end_junction)fade=std::min(fade,std::clamp((length-station-3)/8.0,0.0,1.0));
    for(int j=0;j<=across;++j) {
     const double lateral=-extent+2*extent*j/across,e=xy->x+nx*lateral,n=xy->y+ny*lateral;
     const auto height=separated?deck_height(entry.way_id,station,lateral):ground(e,n);
     if(!height){valid=false;break;}
     mesh.vertices.push_back({static_cast<float>(e-ox),static_cast<float>(n-oy),static_cast<float>(*height+lift),static_cast<float>(lateral),static_cast<float>(station),static_cast<float>(width),static_cast<float>(fade),static_cast<float>(lanes),static_cast<float>(forward),static_cast<float>(flags)});
    }
    if(!valid)break;
   }
   if(!valid){mesh.vertices.resize(first);continue;}
   for(int j=0;j<across;++j){const auto a=static_cast<std::int32_t>(first+j),b=a+across+1;mesh.indices.insert(mesh.indices.end(),{a,b,a+1,a+1,b,b+1});}
  }
  // Direction arrows only where OSM explicitly records turn:lanes.
  const auto arrows=[&](std::string turns,int direction,int lane_start,int lane_count) {
   if(turns.empty()||lane_count<=0||length<20)return;
   std::vector<std::string> tokens;std::size_t begin=0;
   while(true){auto end=turns.find('|',begin);tokens.push_back(turns.substr(begin,end-begin));if(end==std::string::npos)break;begin=end+1;}
   if(static_cast<int>(tokens.size())!=lane_count)return;
   const double station=direction>0?length-15:15;
   auto center=p.reference.at(station);if(!center||std::floor(center->x/1024)!=key.x||std::floor(center->y/1024)!=key.y)return;
   const double lane_width=width/lanes;if(lane_width<2.4)return;
   for(int lane=0;lane<lane_count;++lane){
    const double offset=direction>0?-half+(lane_start+lane_count-lane-.5)*lane_width:-half+(lane_start+lane+.5)*lane_width;
    const auto vertex=[&](double side,double along)->std::optional<RoadVisualVertex>{
     const double st=station+direction*along;auto xy=p.reference.at(st);auto z=p.at(st);if(!xy||!z)return {};
     const double lateral=offset+direction*side,e=xy->x-std::sin(xy->heading)*lateral,n=xy->y+std::cos(xy->heading)*lateral;
     const bool separated=p.attributes.bridge||std::any_of(decks.begin(),decks.end(),[&](const auto& d){return d->way_id==entry.way_id&&st>=d->start_station&&st<=d->end_station;});
     auto height=separated?deck_height(entry.way_id,st,lateral):ground(e,n);if(!height)return {};
     return RoadVisualVertex{static_cast<float>(e-ox),static_cast<float>(n-oy),static_cast<float>(*height+lift+.003),0,0,static_cast<float>(width),1,0,0,16};
    };
    const auto triangle=[&](double ax,double ay,double bx,double by,double cx,double cy){
     auto a=vertex(ax,ay),b=vertex(bx,by),c=vertex(cx,cy);if(!a||!b||!c)return;const auto first=static_cast<std::int32_t>(mesh.vertices.size());
     mesh.vertices.insert(mesh.vertices.end(),{*a,*b,*c});mesh.indices.insert(mesh.indices.end(),{first,first+1,first+2});
    };
    const auto stroke=[&](double ax,double ay,double bx,double by){const double dx=bx-ax,dy=by-ay,len=std::hypot(dx,dy);if(len<.01)return;const double nx=-dy/len*.11,ny=dx/len*.11;triangle(ax+nx,ay+ny,ax-nx,ay-ny,bx+nx,by+ny);triangle(ax-nx,ay-ny,bx-nx,by-ny,bx+nx,by+ny);};
    const auto& token=tokens[lane];
    const bool through=token.find("through")!=std::string::npos;
    const bool left=token.find("left")!=std::string::npos,right=token.find("right")!=std::string::npos;
    if(!through&&!left&&!right)continue;
    stroke(0,-2,0,0);
    if(through){stroke(0,0,0,1.1);triangle(-.45,.85,.45,.85,0,1.7);}
    if(left){stroke(0,0,.7,.5);triangle(.65,.15,.65,.85,1.15,.5);}
    if(right){stroke(0,0,-.7,.5);triangle(-.65,.15,-.65,.85,-1.15,.5);}
   }
  };
  if(one)arrows(tag(w,"turn:lanes"),direction=="-1"?-1:1,0,lanes);
  else {arrows(tag(w,"turn:lanes:forward"),1,0,forward);arrows(tag(w,"turn:lanes:backward"),-1,forward,lanes-forward);}
 }
 add_junction_surfaces(mesh,geometry,key,ground,lift);
 return mesh;
}
}
