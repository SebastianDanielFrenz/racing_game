#include "rg/npc_traffic.h"
#include "rg/world_terrain.h"
#include "rg/road_structures.h"
#include "g2m/layer/osm_roads.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <set>
namespace rg {
namespace {
std::string tag(const g2m::RoadGraphWay& w,const std::string& k){for(const auto& [key,v]:w.tags)if(key==k)return v;return {};}
bool public_access(const std::string& s){return s.empty()||s=="yes"||s=="permissive"||s=="designated";}
}
TrafficConfig sanitize_traffic_config(TrafficConfig c){
 auto bounded=[](double v,double fallback,double lo,double hi){return std::clamp(std::isfinite(v)?v:fallback,lo,hi);};
 c.density_per_km=bounded(c.density_per_km,120,0,1000);c.radius_m=bounded(c.radius_m,1200,200,3000);
 c.min_spawn_m=bounded(c.min_spawn_m,100,30,c.radius_m-30);c.grip_multiplier=bounded(c.grip_multiplier,1,.05,1);
 c.max_vehicles=std::clamp(c.max_vehicles,0,4096);return c;
}
bool traffic_road_allowed(const g2m::RoadGraphWay& w,bool truck,int dir){
 static const std::set<std::string> allowed={"motorway","trunk","primary","secondary","tertiary","unclassified","residential","motorway_link","trunk_link","primary_link","secondary_link","tertiary_link","living_street","road"};
 if(!allowed.contains(tag(w,"highway")))return false;
 // More-specific access overrides the generic class, but private remains nonpublic.
 if(tag(w,"access")=="private")return false;
 std::string access;
 for(const auto& k:std::vector<std::string>{"access","vehicle","motor_vehicle",truck?"hgv":"motorcar"})if(!tag(w,k).empty())access=tag(w,k);
 for(const auto& k:std::vector<std::string>{"vehicle","motor_vehicle",truck?"hgv":"motorcar"})if(!tag(w,k+(dir>0?":forward":":backward")).empty())access=tag(w,k+(dir>0?":forward":":backward"));
 if(!public_access(access))return false;
 for(const auto& [k,v]:w.tags)if(!v.empty()&&k.find("conditional")!=std::string::npos&&(k.starts_with("access")||k.starts_with("vehicle")||k.starts_with("motor_vehicle")||k.starts_with(truck?"hgv":"motorcar")))return false;
 return truck_lane(w,6.5,2,dir).allowed;
}
double traffic_speed_cap(const g2m::RoadGraphWay& w,bool truck,int dir,double desired){
 double cap=truck?120./3.6:250./3.6;bool numeric=false,unresolved=false;
 for(const auto& k:std::vector<std::string>{"maxspeed",dir>0?"maxspeed:forward":"maxspeed:backward",truck?"maxspeed:hgv":"maxspeed:motorcar"}) {
  const auto value=tag(w,k);if(value.empty())continue;auto limit=g2m::parse_road_speed_limit(value);
  if(limit.kind==g2m::SpeedLimitKind::Numeric){cap=std::min(cap,limit.kph/3.6);numeric=true;}
  else if(value!="none")unresolved=true;
 }
 auto lanes=tag(w,dir>0?"maxspeed:lanes:forward":"maxspeed:lanes:backward");if(lanes.empty())lanes=tag(w,"maxspeed:lanes");
 // Conservative minimum until per-lane limit resolution is authoritative.
 if(!lanes.empty()){std::size_t start=0;for(;;){auto end=lanes.find('|',start);auto v=lanes.substr(start,end-start);if(!v.empty()){auto l=g2m::parse_road_speed_limit(v);if(l.kind==g2m::SpeedLimitKind::Numeric){cap=std::min(cap,l.kph/3.6);numeric=true;}else if(v!="none")unresolved=true;}if(end==std::string::npos)break;start=end+1;}}
 for(const auto& [k,v]:w.tags)if(k.starts_with("maxspeed")&&k.find("conditional")!=std::string::npos&&!v.empty())unresolved=true;
 if(unresolved)cap=std::min(cap,tag(w,"highway")=="living_street"?2.:30./3.6);
 if(!numeric)cap=std::min(cap,truck?80./3.6:std::max(0.,desired));
 return cap;
}
std::vector<TrafficDestination> extract_traffic_destinations(const g2m::OsmTile& tile,const g2m::geo::SessionFrame& frame){
 const auto& d=tile.data;g2m::geo::Utm utm;
 auto value=[&](const auto& item,const char* k){auto v=d.tag(d.tags_of(item),k);return v?std::string(*v):std::string{};};
 auto point=[&](const g2m::osm::Node& n){auto p=utm.forward(frame.zone(),n.lat*1e-7,n.lon*1e-7);return ps::Vec3{p.easting-frame.e0_m(),p.northing-frame.n0_m(),0};};
 using Ring=std::vector<ps::Vec3>;
 auto ring=[&](const g2m::osm::Way& w){Ring r;auto refs=d.refs_of(w);if(refs.size()<4||refs.front()!=refs.back())return r;for(auto id:refs){auto n=d.find_node(id);if(!n)return Ring{};r.push_back(point(*n));}return r;};
 auto inside=[](const Ring& r,ps::Vec3 p){bool yes=false;for(std::size_t i=0,j=r.size()-1;i<r.size();j=i++)if((r[i].y>p.y)!=(r[j].y>p.y)&&p.x<(r[j].x-r[i].x)*(p.y-r[i].y)/(r[j].y-r[i].y)+r[i].x)yes=!yes;return yes;};
 auto center=[](const Ring& r){ps::Vec3 p{};for(std::size_t i=0;i+1<r.size();++i)p+=r[i];return p/static_cast<double>(r.size()-1);};
 std::vector<Ring> residential,excluded_residential;
 for(const auto& w:d.ways)if(value(w,"landuse")=="residential"){auto r=ring(w);if(!r.empty())residential.push_back(std::move(r));}
 // Complete closed outer members are usable; fragmented/missing multipolygons are deferred.
 for(const auto& rel:d.relations)if(value(rel,"landuse")=="residential")for(auto m:d.members_of(rel))if(m.type==g2m::osm::MemberType::Way)if(auto w=d.find_way(m.ref)){auto r=ring(*w);if(!r.empty()){
  if(d.str(m.role)=="outer")residential.push_back(std::move(r));
  else if(d.str(m.role)=="inner")excluded_residential.push_back(std::move(r));}}
 std::vector<TrafficDestination> result;
 const auto add=[&](const auto& item,ps::Vec3 p,bool relation,int osm_type=1){
  const bool parking=value(item,"amenity")=="parking";
  const auto building=value(item,"building");
  bool area=false;for(const auto& r:residential)if(inside(r,p)){area=true;break;}for(const auto& r:excluded_residential)if(inside(r,p))area=false;
  if(!parking&&(building.empty()||building=="no"||!area))return;
  if(parking&&!public_access(value(item,"access")))return;
  result.push_back({item.id,relation,parking,p,osm_type});
 };
 for(const auto& w:d.ways){auto r=ring(w);if(!r.empty())add(w,center(r),false);}
 for(const auto& n:d.nodes)if(value(n,"amenity")=="parking")add(n,point(n),false,0);
 for(const auto& rel:d.relations)if(!value(rel,"building").empty()||value(rel,"amenity")=="parking")for(auto m:d.members_of(rel))if(m.type==g2m::osm::MemberType::Way&&d.str(m.role)=="outer")if(auto w=d.find_way(m.ref)){auto r=ring(*w);if(!r.empty()){add(rel,center(r),true,2);break;}}
 return result;
}
TrafficPlan plan_traffic(std::shared_ptr<WorldTerrain> terrain,ps::Vec3 player,TrafficConfig config,std::uint64_t seed,const std::atomic<bool>& cancel,bool cached_geometry_only){
 TrafficPlan plan;if(cancel.load())return plan;config=sanitize_traffic_config(config);if(!terrain){plan.message="Traffic needs OSM roads and destinations (real world)";return plan;}
 const auto& frame=terrain->frame();
 struct Edge{std::int64_t first,last;const g2m::RoadProfile* profile;const g2m::RoadGraphWay* way;int direction;};
 struct TurnRule {std::int64_t from=0,to=0,via=0;bool only=false,unsupported=false;};
 std::vector<TurnRule> turn_rules;
 std::vector<std::shared_ptr<const g2m::RoadGeomTile>> tiles;std::vector<Edge> edges;std::vector<TrafficDestination> destinations;
 std::set<std::tuple<std::int64_t,std::uint32_t,std::uint32_t>> seen;std::set<std::pair<std::int64_t,int>> seen_dest;
 const double e=frame.grid_easting(player.x),n=frame.grid_northing(player.y);
 const int x0=static_cast<int>(std::floor((e-config.radius_m)/1024)),x1=static_cast<int>(std::floor((e+config.radius_m)/1024));
 const int y0=static_cast<int>(std::floor((n-config.radius_m)/1024)),y1=static_cast<int>(std::floor((n+config.radius_m)/1024));
 for(int y=y0;y<=y1&&!cancel.load();++y)for(int x=x0;x<=x1&&!cancel.load();++x){
  g2m::TileKey key{frame.zone(),2,x,y};
  if(cancel.load())return plan;
  auto tile=cached_geometry_only?terrain->cached_road_geometry_at(x*1024.+512,y*1024.+512):terrain->road_geometry_at(x*1024.+512,y*1024.+512);if(!tile)continue;tiles.push_back(tile);
  if(auto osm=terrain->source_osm_tile(key)){
   for(auto d:extract_traffic_destinations(*osm,frame))if(seen_dest.emplace(d.id,d.osm_type).second)destinations.push_back(d);
   const auto& data=osm->data;
   for(const auto& relation:data.relations){auto type=data.tag(data.tags_of(relation),"type");if(!type||*type!="restriction")continue;
    auto restriction=data.tag(data.tags_of(relation),"restriction");if(!restriction)continue;TurnRule rule;rule.only=restriction->starts_with("only_");
    for(auto member:data.members_of(relation)){auto role=data.str(member.role);if(role=="from"&&member.type==g2m::osm::MemberType::Way)rule.from=member.ref;
     if(role=="to"&&member.type==g2m::osm::MemberType::Way)rule.to=member.ref;
     if(role=="via"){if(member.type==g2m::osm::MemberType::Node)rule.via=member.ref;else rule.unsupported=true;}}
    if(rule.from&&rule.to)turn_rules.push_back(rule);
   }
  }
  for(const auto& entry:tile->entries){if(!entry.profile||!seen.emplace(entry.way_id,entry.stretch.start_ref,entry.stretch.end_ref).second)continue;
   const auto& ways=tile->source.graph.ways;auto w=std::lower_bound(ways.begin(),ways.end(),entry.way_id,[](const auto& a,auto id){return a.osm_id<id;});
   if(w==ways.end()||w->osm_id!=entry.way_id||entry.stretch.end_ref>=w->node_ids.size())continue;
   for(int dir:{1,-1})if(traffic_road_allowed(*w,false,dir)&&truck_lane(*w,entry.profile->attributes.width_mm*.001,entry.profile->attributes.lanes,dir).allowed)
    edges.push_back({w->node_ids[dir>0?entry.stretch.start_ref:entry.stretch.end_ref],w->node_ids[dir>0?entry.stretch.end_ref:entry.stretch.start_ref],&*entry.profile,&*w,dir});
  }
 }
 plan.destinations=destinations.size();
 for(const auto& edge:edges){auto xy=edge.profile->reference.at(edge.profile->reference.length_m*.5);if(xy&&std::hypot(xy->x-e,xy->y-n)<config.radius_m)plan.road_length_m+=edge.profile->reference.length_m;}
 const int count=std::min(std::min(512,config.max_vehicles),static_cast<int>(std::ceil(plan.road_length_m*config.density_per_km/1000)));
 std::mt19937_64 random(seed);const auto decks=terrain->road_decks();
 std::map<std::int64_t,std::vector<std::size_t>> outgoing;for(std::size_t i=0;i<edges.size();++i)outgoing[edges[i].first].push_back(i);
 // Destination entrances attach only to nearby ordinary roads, never motorway/grade crossings.
 struct Goal{TrafficDestination destination;std::size_t edge;double station;};std::vector<Goal> goals;
 struct RoadSample{std::size_t edge;double station,x,y;};
 std::map<std::pair<int,int>,std::vector<RoadSample>> sample_grid;
 for(std::size_t i=0;i<edges.size();++i){
  if(cancel.load())return plan;
  const auto& edge=edges[i];const auto highway=tag(*edge.way,"highway");
  if(highway=="motorway"||highway=="trunk"||highway.ends_with("_link")||tag(*edge.way,"bridge")=="yes"||tag(*edge.way,"tunnel")=="yes")continue;
  for(double station=0;station<=edge.profile->reference.length_m;station+=5){
   if(cancel.load())return plan;auto xy=edge.profile->reference.at(station);if(!xy)continue;
   const double x=xy->x-frame.e0_m(),y=xy->y-frame.n0_m();
   sample_grid[{static_cast<int>(std::floor(x/40)),static_cast<int>(std::floor(y/40))}].push_back({i,station,x,y});
  }
 }
 for(auto destination:destinations){
  if(cancel.load())return plan;double best=40;std::size_t selected=edges.size();double station=0;
  const int cx=static_cast<int>(std::floor(destination.point.x/40)),cy=static_cast<int>(std::floor(destination.point.y/40));
  for(int y=cy-1;y<=cy+1;++y)for(int x=cx-1;x<=cx+1;++x){
   auto bin=sample_grid.find({x,y});if(bin==sample_grid.end())continue;
   for(const auto& sample:bin->second){double distance=std::hypot(sample.x-destination.point.x,sample.y-destination.point.y);if(distance<best){best=distance;selected=sample.edge;station=sample.station;}}
  }
  if(selected<edges.size())goals.push_back({destination,selected,station});
 }
 if(edges.empty()||goals.empty()){plan.message="No reachable residential/parking destinations in local OSM network";return plan;}
 std::map<std::pair<int,int>,std::vector<std::size_t>> starts;
 for(int attempt=0;attempt<std::min(12000,count*12)&&static_cast<int>(plan.trips.size())<count&&!cancel.load();++attempt){
  const std::size_t initial=random()%edges.size();const auto& first=edges[initial];const double start=first.profile->reference.length_m*(.15+.7*std::generate_canonical<double,53>(random));
  auto spawn=first.profile->reference.at(start);if(!spawn)continue;double distance=std::hypot(spawn->x-e,spawn->y-n);if(distance<config.min_spawn_m||distance>config.radius_m)continue;
  const bool truck=random()%5==0;if(truck&&!traffic_road_allowed(*first.way,true,first.direction))continue;
  std::vector<double> dist(edges.size(),1e30);std::vector<std::size_t> parent(edges.size(),edges.size());
  using QueueItem=std::pair<double,std::size_t>;std::priority_queue<QueueItem,std::vector<QueueItem>,std::greater<QueueItem>> queue;
  dist[initial]=0;queue.push({0,initial});
  while(!queue.empty()){if(cancel.load())return plan;auto [cost,index]=queue.top();queue.pop();if(cost!=dist[index])continue;const auto& edge=edges[index];
   for(auto next:outgoing[edge.last]){const auto& candidate=edges[next];if(!traffic_road_allowed(*candidate.way,truck,candidate.direction)||candidate.last==edge.first)continue;
    bool forbidden=false;
    for(const auto& rule:turn_rules)if(rule.from==edge.way->osm_id){
     if(rule.unsupported){if(candidate.way->osm_id!=edge.way->osm_id)forbidden=true;}
     else if(rule.via==edge.last&&((rule.only&&candidate.way->osm_id!=rule.to)||(!rule.only&&candidate.way->osm_id==rule.to)))forbidden=true;
    }if(forbidden)continue;
    auto a=edge.profile->reference.at(edge.direction>0?edge.profile->reference.length_m:0),b=candidate.profile->reference.at(candidate.direction>0?0:candidate.profile->reference.length_m);if(!a||!b||std::cos(b->heading+(candidate.direction<0?3.141592653589793:0)-a->heading-(edge.direction<0?3.141592653589793:0))<-.2)continue;
    const double value=cost+candidate.profile->reference.length_m;if(value<dist[next]){dist[next]=value;parent[next]=index;queue.push({value,next});}
   }
  }
  std::vector<std::size_t> reachable;
  for(std::size_t i=0;i<goals.size();++i){const auto& goal=goals[i];if(dist[goal.edge]<1e29&&(goal.edge!=initial||(goal.station-start)*first.direction>40))reachable.push_back(i);}
  if(reachable.empty())continue;const auto& goal=goals[reachable[random()%reachable.size()]];
  std::vector<std::size_t> path;for(auto i=goal.edge;i<edges.size();i=parent[i]){path.push_back(i);if(i==initial)break;}if(path.back()!=initial)continue;std::reverse(path.begin(),path.end());
  TrafficTrip trip;trip.truck=truck;trip.destination=goal.destination;trip.desired_speed=truck?80./3.6:(80.+170.*std::generate_canonical<double,53>(random))/3.6;
  bool valid=true;double total=0;
  for(auto index:path){if(cancel.load())return plan;const auto& edge=edges[index];const auto& p=*edge.profile;const auto lane=truck_lane(*edge.way,p.attributes.width_mm*.001,p.attributes.lanes,edge.direction);
   const double begin=index==initial?start:(edge.direction>0?0:p.reference.length_m),end=index==goal.edge?goal.station:(edge.direction>0?p.reference.length_m:0);
   double mu=tag(*edge.way,"surface")=="gravel"||tag(*edge.way,"surface")=="unpaved"?.45:1.;
   for(double s=begin;;s+=edge.direction*2){if(cancel.load())return plan;if((edge.direction>0&&s>end)||(edge.direction<0&&s<end))s=end;auto xy=p.reference.at(s);auto z=p.at(s);if(!xy||!z){valid=false;break;}
    double height=z->height_m-p.attributes.crown_per_mille*.001*std::abs(lane.offset_m);
    for(const auto& deck:decks)if(deck->way_id==edge.way->osm_id&&xy->x>=deck->min_easting-2&&xy->x<=deck->max_easting+2&&xy->y>=deck->min_northing-2&&xy->y<=deck->max_northing+2)if(auto h=road_deck_height(*deck,s,lane.offset_m))height=*h;
    TruckRoutePoint point{{xy->x-std::sin(xy->heading)*lane.offset_m-frame.e0_m(),xy->y+std::cos(xy->heading)*lane.offset_m-frame.n0_m(),height},xy->heading+(edge.direction<0?3.141592653589793:0),z->grade*edge.direction,traffic_speed_cap(*edge.way,truck,edge.direction,trip.desired_speed),0,edge.way->osm_id};
    point.speed_m_s=std::min(point.speed_m_s,std::sqrt(2.5*mu*config.grip_multiplier/std::max(.001,std::abs(xy->curvature))));
    if(!trip.route.points.empty()){double gap=(point.ground-trip.route.points.back().ground).length();if(gap>12||std::abs(point.ground.z-trip.route.points.back().ground.z)>2){valid=false;break;}total+=gap;}
    point.station=total;trip.route.points.push_back(point);if(s==end)break;
   }if(!valid)break;
  }
  if(!valid||total<40||trip.route.points.size()<3)continue;
  // Reject overlapping initial footprints; physics performs another live check before spawn.
  const auto& proposed=trip.route.points.front();
  const int sx=static_cast<int>(std::floor(proposed.ground.x/32)),sy=static_cast<int>(std::floor(proposed.ground.y/32));
  for(int y=sy-1;y<=sy+1;++y)for(int x=sx-1;x<=sx+1;++x){auto bin=starts.find({x,y});if(bin==starts.end())continue;
   for(auto existing_index:bin->second){const auto& other=plan.trips[existing_index];const auto& previous=other.route.points.front();
    const auto delta=proposed.ground-previous.ground;if(std::abs(delta.z)>4)continue;
    const double along=std::abs(delta.x*std::cos(proposed.yaw)+delta.y*std::sin(proposed.yaw));
    const double side=std::abs(-delta.x*std::sin(proposed.yaw)+delta.y*std::cos(proposed.yaw));
    if(side<2.5&&along<(trip.truck?6.5:2.3)+(other.truck?6.5:2.3)+5)valid=false;
    // Crossing orientations need conservative spacing at the intersection.
    if(std::abs(std::sin(proposed.yaw-previous.yaw))>.4&&delta.length()<12)valid=false;
   }
  }if(!valid)continue;
  // Corner speed from the heading change over a WINDOW of at least 4 m of route each side. The previous 2-neighbour
  // difference divided by max(0.1, ds) hit the 0.1 floor wherever two route points sit (almost) on top of each other
  // - the join of two road edges at a junction, where the heading also jumps - giving a corner "radius" of a few cm
  // and a cap of ~0.4 m/s: the car crawled below the stuck threshold through every junction turn.
  {const auto& pts=trip.route.points;std::vector<double> cap(pts.size(),1e30);
   for(std::size_t i=1;i+1<pts.size();++i){std::size_t j0=i,j1=i;
    while(j0>0&&pts[i].station-pts[j0].station<4)--j0;while(j1+1<pts.size()&&pts[j1].station-pts[i].station<4)++j1;
    const double curvature=std::abs(std::remainder(pts[j1].yaw-pts[j0].yaw,6.283185307179586))/std::max(1.,pts[j1].station-pts[j0].station);
    cap[i]=std::sqrt(2.5*config.grip_multiplier/std::max(.001,curvature));}
   for(std::size_t i=1;i+1<pts.size();++i)trip.route.points[i].speed_m_s=std::min(trip.route.points[i].speed_m_s,cap[i]);}
  trip.route.points.back().speed_m_s=0;
  for(std::size_t i=trip.route.points.size()-1;i>0;--i){double ds=trip.route.points[i].station-trip.route.points[i-1].station;trip.route.points[i-1].speed_m_s=std::min(trip.route.points[i-1].speed_m_s,std::sqrt(trip.route.points[i].speed_m_s*trip.route.points[i].speed_m_s+5*config.grip_multiplier*ds));}
  starts[{sx,sy}].push_back(plan.trips.size());
  plan.trips.push_back(std::move(trip));
 }
 plan.message="Destination traffic: "+std::to_string(plan.trips.size())+" planned trips";return plan;
}
}
