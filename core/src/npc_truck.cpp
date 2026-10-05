#include "rg/npc_truck.h"
#include "rg/world_terrain.h"
#include "rg/road_structures.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
namespace rg {
namespace {
std::string tag(const g2m::RoadGraphWay& w,const char* key) { for(const auto& [k,v]:w.tags)if(k==key)return v;return {}; }
int count(const std::string& s) { if(s.empty()||s.size()>2)return 0;int n=0;for(char c:s){if(c<'0'||c>'9')return 0;n=n*10+c-'0';}return n<=8?n:0; }
}
TruckLane truck_lane(const g2m::RoadGraphWay& w,double width,int lanes,int dir) {
 if(!std::isfinite(width)||width<2.8)return {};
 auto one=tag(w,"oneway"); if(one.empty()&&(tag(w,"highway")=="motorway"||tag(w,"junction")=="roundabout"))one="yes";
 const bool forward=one=="yes"||one=="1"||one=="true",reverse=one=="-1";
 if((forward&&dir<0)||(reverse&&dir>0))return {};
 const bool single=forward||reverse;
 int n=lanes;
 if(n<=0||n>8||width/n<2.7)n=single?std::max(1,static_cast<int>(width/3.25)):(width>=5.8?2:1);
 const auto direction_tag=tag(w,dir>0?"lanes:forward":"lanes:backward");
 if(direction_tag=="0")return {};
 // One lane in total on a two-way road: each direction still drives in ITS half (centre of the right half),
 // never on the centre line - both directions on the same line made every oncoming pair a head-on deadlock
 // (docs/npc_traffic.md, "Stuck traffic"). A one-way road stays centred.
 if(n==1)return {true,single?0.:-width*.25*dir};
 const int tagged=count(direction_tag);
 const int directional=single?n:(tagged>0&&tagged<n?tagged:std::max(1,n/2));
 double offset=-width*.5+width/(2*n);
 auto turns=tag(w,dir>0?"turn:lanes:forward":"turn:lanes:backward");
 if(turns.empty()&&single)turns=tag(w,"turn:lanes");
 if(!turns.empty()) {
  std::vector<std::string> lane_turns;std::size_t start=0;
  for(;;){auto end=turns.find('|',start);lane_turns.push_back(turns.substr(start,end-start));if(end==std::string::npos)break;start=end+1;}
  if(static_cast<int>(lane_turns.size())==directional) {
   for(int i=directional-1;i>=0;--i)if(lane_turns[static_cast<std::size_t>(i)].find("through")!=std::string::npos||lane_turns[static_cast<std::size_t>(i)].empty()||lane_turns[static_cast<std::size_t>(i)]=="none") {
    offset+=(directional-1-i)*width/n;break;
   }
  }
 }
 return {true,offset*dir};
}
TruckRoute build_truck_route(std::shared_ptr<WorldTerrain> terrain,const ps::Pose& car,double target,const std::atomic<bool>& cancel) {
 TruckRoute route;
 const auto heading=car.orientation.rotate(ps::Vec3::unit_x());
 if(!terrain) {
  for(int i=0;i<=1500;++i)route.points.push_back({{car.position.x+heading.x*(2*i),car.position.y+heading.y*(2*i),0},std::atan2(heading.y,heading.x),0,target,2.0*i,0});
  route.message="Truck: flat-world drafting route";return route;
 }
 struct Candidate { const g2m::RoadProfile* p; const g2m::RoadGraphWay* w; std::int64_t first,last; };
 std::vector<std::shared_ptr<const g2m::RoadGeomTile>> tiles;
 std::vector<Candidate> candidates;
 std::set<std::tuple<std::int64_t,std::uint32_t,std::uint32_t>> seen;
 const auto& frame=terrain->frame();
 for(double ahead:{0.,600.,1200.}) {
  if(cancel.load())return route;
  auto tile=terrain->road_geometry_at(frame.grid_easting(car.position.x+heading.x*ahead),frame.grid_northing(car.position.y+heading.y*ahead));
  if(!tile)continue;tiles.push_back(tile);
  for(const auto& e:tile->entries) {
   if(!e.profile||!seen.emplace(e.way_id,e.stretch.start_ref,e.stretch.end_ref).second)continue;
   const auto& ways=tile->source.graph.ways;
   auto w=std::lower_bound(ways.begin(),ways.end(),e.way_id,[](const auto& a,auto id){return a.osm_id<id;});
   if(w==ways.end()||w->osm_id!=e.way_id||e.stretch.end_ref>=w->node_ids.size())continue;
   candidates.push_back({&*e.profile,&*w,w->node_ids[e.stretch.start_ref],w->node_ids[e.stretch.end_ref]});
  }
 }
 std::size_t selected=candidates.size();int direction=1;double start=0,best=150;
 for(std::size_t i=0;i<candidates.size();++i) {
  const auto& c=candidates[i];
  for(int dir:{1,-1}) {
   if(!truck_lane(*c.w,c.p->attributes.width_mm*.001,c.p->attributes.lanes,dir).allowed)continue;
   for(double s=0;s<=c.p->reference.length_m;s+=3) {
    auto xy=c.p->reference.at(s);auto z=c.p->at(s);if(!xy||!z)continue;
    const double dot=(std::cos(xy->heading)*heading.x+std::sin(xy->heading)*heading.y)*dir;
    if(dot<.5)continue;
    const double x=(xy->x-static_cast<double>(frame.e0_m()))-car.position.x,y=(xy->y-static_cast<double>(frame.n0_m()))-car.position.y;
    const double dz=z->height_m-car.position.z;
    const double score=x*x+y*y+dz*dz*4+(1-dot)*20;
    if(score<best){best=score;selected=i;direction=dir;start=s;}
   }
  }
 }
 if(selected==candidates.size()){route.message="Truck: no usable lane nearby; move onto a wider road";return route;}
 const auto decks=terrain->road_decks();
 std::set<std::size_t> visited;
 double total=0;
 for(int stretches=0;stretches<32&&!cancel.load();++stretches) {
  if(!visited.insert(selected).second)break;
  const auto& c=candidates[selected];const auto& p=*c.p;
  const auto lane=truck_lane(*c.w,p.attributes.width_mm*.001,p.attributes.lanes,direction);
  double speed=target;
  auto limit=g2m::parse_road_speed_limit(tag(*c.w,direction>0?"maxspeed:forward":"maxspeed:backward"));
  if(limit.kind==g2m::SpeedLimitKind::Unknown)limit=g2m::parse_road_speed_limit(tag(*c.w,"maxspeed"));
  if(limit.kind==g2m::SpeedLimitKind::Numeric)speed=std::min(speed,limit.kph/3.6);
  else if(tag(*c.w,"highway")=="residential"||tag(*c.w,"highway")=="living_street")speed=std::min(speed,tag(*c.w,"highway")=="living_street"?2.0:30./3.6);
  const double end=direction>0?p.reference.length_m:0;
  for(double s=start;;s+=direction*2) {
   if((direction>0&&s>end)||(direction<0&&s<end))s=end;
   auto xy=p.reference.at(s);auto z=p.at(s);if(!xy||!z)break;
   double h=z->height_m-p.attributes.crown_per_mille*.001*std::abs(lane.offset_m);
   for(const auto& deck:decks)if(deck->way_id==c.w->osm_id&&xy->x>=deck->min_easting-2&&xy->x<=deck->max_easting+2&&xy->y>=deck->min_northing-2&&xy->y<=deck->max_northing+2)if(auto dh=road_deck_height(*deck,s,lane.offset_m))h=*dh;
   const double x=xy->x-std::sin(xy->heading)*lane.offset_m,y=xy->y+std::cos(xy->heading)*lane.offset_m;
   TruckRoutePoint point{{(x-static_cast<double>(frame.e0_m())),(y-static_cast<double>(frame.n0_m())),h},xy->heading+(direction<0?3.141592653589793:0),z->grade*direction,std::min(speed,std::sqrt(1.2/std::max(.001,std::abs(xy->curvature)))),0,c.w->osm_id};
   if(!route.points.empty()) {
    const double gap=(point.ground-route.points.back().ground).length();
    if(gap>6){route.message="Truck: stopping before a disconnected lane";return route;}
    total+=gap;
   }
   point.station=total;route.points.push_back(point);
   if(total>2500||s==end)break;
  }
  if(total>2500||route.points.empty())break;
  const auto node=direction>0?c.last:c.first;
  const double yaw=route.points.back().yaw;
  std::size_t next=candidates.size();int next_dir=1;double score=.65;
  for(std::size_t i=0;i<candidates.size();++i)if(!visited.contains(i))for(int dir:{1,-1}) {
   const auto& candidate=candidates[i];if((dir>0?candidate.first:candidate.last)!=node)continue;
   if(!truck_lane(*candidate.w,candidate.p->attributes.width_mm*.001,candidate.p->attributes.lanes,dir).allowed)continue;
   auto xy=candidate.p->reference.at(dir>0?0:candidate.p->reference.length_m);if(!xy)continue;
   double aligned=std::cos(xy->heading+(dir<0?3.141592653589793:0)-yaw);
   if(aligned<.65)continue;
   if(candidate.w->osm_id==c.w->osm_id)aligned+=.2;
   if(aligned>score){score=aligned;next=i;next_dir=dir;}
  }
  if(next==candidates.size())break;
  selected=next;direction=next_dir;start=direction>0?0:candidates[next].p->reference.length_m;
 }
 if(route.points.size()<30){route.points.clear();route.message="Truck: not enough continuous road ahead";return route;}
 for(std::size_t i=1;i<route.points.size();++i) {
  const auto delta=route.points[i].ground-route.points[i-1].ground;
  route.points[i-1].grade=delta.z/std::max(.01,std::hypot(delta.x,delta.y));
 }
 route.points.back().grade=route.points[route.points.size()-2].grade;
 // Sampled joins can be less smooth than an individual fitted reference.
 // Bound lateral acceleration there too, then propagate braking upstream.
 for(std::size_t i=1;i+1<route.points.size();++i) {
  const double length=std::max(.5,route.points[i+1].station-route.points[i-1].station);
  const double curvature=std::abs(std::remainder(route.points[i+1].yaw-route.points[i-1].yaw,2*3.141592653589793))/length;
  route.points[i].speed_m_s=std::min(route.points[i].speed_m_s,std::sqrt(1.2/std::max(.001,curvature)));
 }
 route.points.back().speed_m_s=0;
 for(std::size_t i=route.points.size()-1;i>0;--i) {
  const double distance=route.points[i].station-route.points[i-1].station;
  route.points[i-1].speed_m_s=std::min(route.points[i-1].speed_m_s,std::sqrt(route.points[i].speed_m_s*route.points[i].speed_m_s+5*distance));
 }
 route.message="Truck: following rightmost usable lane";return route;
}
}
