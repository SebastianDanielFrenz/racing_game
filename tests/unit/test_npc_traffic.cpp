#include "rg/npc_traffic.h"
#include "rg/world_terrain.h"
#include "rg/world_config.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <limits>
TEST_CASE("Traffic public access and speed policies", "[npc_traffic]") {
 g2m::RoadGraphWay way;way.tags={{"highway","residential"}};
 CHECK(rg::traffic_road_allowed(way,false,1));
 CHECK(rg::traffic_speed_cap(way,true,1,50)==Catch::Approx(80./3.6));
 way.tags.push_back({"maxspeed","50"});CHECK(rg::traffic_speed_cap(way,false,1,60)==Catch::Approx(50./3.6));
 way.tags.push_back({"maxspeed:lanes","50|30"});CHECK(rg::traffic_speed_cap(way,false,1,60)==Catch::Approx(30./3.6));
 way.tags.push_back({"access","private"});CHECK_FALSE(rg::traffic_road_allowed(way,false,1));
 way.tags={{"highway","service"}};CHECK_FALSE(rg::traffic_road_allowed(way,false,1));
 way.tags={{"highway","motorway"}};CHECK_FALSE(rg::traffic_road_allowed(way,false,-1));
 way.tags={{"highway","residential"},{"access","no"},{"motor_vehicle","yes"}};CHECK(rg::traffic_road_allowed(way,false,1));
 way.tags.push_back({"hgv","no"});CHECK_FALSE(rg::traffic_road_allowed(way,true,1));
 way.tags={{"highway","primary"},{"maxspeed","signals"}};CHECK(rg::traffic_speed_cap(way,false,1,60)<=30./3.6);
 rg::TrafficConfig config;config.radius_m=10;config.min_spawn_m=1000;config.density_per_km=-1;config.grip_multiplier=std::numeric_limits<double>::quiet_NaN();
 config=rg::sanitize_traffic_config(config);CHECK(config.radius_m==200);CHECK(config.min_spawn_m==170);CHECK(config.density_per_km==0);CHECK(config.grip_multiplier==1);
}
TEST_CASE("Traffic destinations use residential landuse and public parking", "[npc_traffic]") {
 g2m::osm::OsmBuilder builder;
 // Closed residential area, one building inside and one outside.
 builder.add_node(1,501000000,85000000);builder.add_node(2,501000000,85100000);builder.add_node(3,501100000,85100000);builder.add_node(4,501100000,85000000);
 builder.add_way(10,{1,2,3,4,1},{{"landuse","residential"}});
 builder.add_node(5,501020000,85020000);builder.add_node(6,501020000,85021000);builder.add_node(7,501021000,85021000);builder.add_node(8,501021000,85020000);
 builder.add_way(20,{5,6,7,8,5},{{"building","yes"}});
 builder.add_node(9,501200000,85020000);builder.add_node(11,501200000,85021000);builder.add_node(12,501201000,85021000);builder.add_node(13,501201000,85020000);
 builder.add_way(30,{9,11,12,13,9},{{"building","house"}});
 builder.add_node(14,501040000,85040000,{{"amenity","parking"}});
 builder.add_node(15,501050000,85050000,{{"amenity","parking"},{"access","private"}});
 auto data=builder.build();REQUIRE(data);g2m::OsmTile tile;tile.data=std::move(*data);
 g2m::geo::SessionFrame frame({32,g2m::geo::Hemisphere::North},460000,5550000);
 auto goals=rg::extract_traffic_destinations(tile,frame);REQUIRE(goals.size()==2);
 CHECK(goals[0].id==20);CHECK_FALSE(goals[0].parking);CHECK(goals[1].id==14);CHECK(goals[1].parking);
}
TEST_CASE("Real traffic plans reach OSM destinations", "[.][npc_traffic_real]") {
 std::string error;auto cfg=rg::load_world_config(std::string(RG_SOURCE_DIR)+"/data/world/world_config.json",&error);REQUIRE(cfg);
 std::shared_ptr<rg::WorldTerrain> terrain(rg::WorldTerrain::open(*cfg,&error));REQUIRE(terrain);
 auto player=terrain->frame().from_geodetic(50.1367,8.4505); // Local residential network, not a motorway destination.
 std::atomic<bool> cancel{false};rg::TrafficConfig config;config.radius_m=600;config.density_per_km=4;config.max_vehicles=24;
 auto plan=rg::plan_traffic(terrain,{player.x,player.y,250},config,51,cancel);
 INFO(plan.message);INFO(plan.destinations);REQUIRE(plan.destinations>0);REQUIRE_FALSE(plan.trips.empty());
 for(const auto& trip:plan.trips){REQUIRE(trip.route.points.size()>2);CHECK(trip.route.points.back().speed_m_s==0);CHECK((trip.route.points.front().ground-ps::Vec3{player.x,player.y,250}).length()>config.min_spawn_m-1);
  const auto last=trip.route.points.back().ground;CHECK(std::hypot(last.x-trip.destination.point.x,last.y-trip.destination.point.y)<45);}
}

TEST_CASE("Cancelled traffic planning exits before map access", "[npc_traffic]") {
 std::atomic<bool> cancel{true};
 auto plan=rg::plan_traffic(nullptr,{},rg::TrafficConfig{},1,cancel,true);
 CHECK(plan.trips.empty());CHECK(plan.message.empty());
}

TEST_CASE("Traffic capacity supports thousands without legacy clamps", "[npc_traffic]") {
 rg::TrafficConfig config;
 CHECK(config.max_vehicles==2048);CHECK(config.density_per_km==120);
 config.max_vehicles=4096;config.density_per_km=1000;config.radius_m=3000;
 auto c=rg::sanitize_traffic_config(config);
 CHECK(c.max_vehicles==4096);CHECK(c.density_per_km==1000);CHECK(c.radius_m==3000);
 config.max_vehicles=999999;CHECK(rg::sanitize_traffic_config(config).max_vehicles==4096);
 config.max_vehicles=0;CHECK(rg::sanitize_traffic_config(config).max_vehicles==0);
}
namespace {
// A lane polyline from (x0, y0) heading `yaw` for `length` m, one point per 2 m (as plan_traffic samples a path edge).
std::vector<rg::TruckRoutePoint> lane(double x0,double y0,double yaw,double length){
 std::vector<rg::TruckRoutePoint> leg;
 for(double s=0;;s+=2){if(s>length)s=length;rg::TruckRoutePoint p;p.ground={x0+std::cos(yaw)*s,y0+std::sin(yaw)*s,0};p.yaw=yaw;p.speed_m_s=14;leg.push_back(p);if(s==length)break;}
 return leg;
}
struct PathCheck{double min_forward=1e30,max_curvature=0;bool monotone=true;};
// min_forward: the smallest cosine between a segment and the heading of its start point (< 0 = the path runs backwards);
// max_curvature: the largest heading change per metre over windows of >= 1 m.
PathCheck check_path(const rg::TruckRoute& r){
 PathCheck c;const auto& p=r.points;
 for(std::size_t i=1;i<p.size();++i){
  const double dx=p[i].ground.x-p[i-1].ground.x,dy=p[i].ground.y-p[i-1].ground.y,d=std::hypot(dx,dy);
  if(d>1e-6)c.min_forward=std::min(c.min_forward,(dx*std::cos(p[i-1].yaw)+dy*std::sin(p[i-1].yaw))/d);
  if(!(p[i].station>p[i-1].station))c.monotone=false;
 }
 for(std::size_t i=0;i<p.size();++i){std::size_t j=i;while(j+1<p.size()&&p[j].station-p[i].station<1)++j;
  if(j>i)c.max_curvature=std::max(c.max_curvature,std::abs(std::remainder(p[j].yaw-p[i].yaw,6.283185307179586))/(p[j].station-p[i].station));}
 return c;
}
constexpr double kHalfPi=1.5707963267948966;
}
TEST_CASE("NPC turn paths: a right turn curves through the junction instead of hopping back across the node", "[npc_traffic][junction_turn]") {
 // Eastbound lane 1.75 m right of the road into the node at the origin, then southbound, 1.75 m right of that road.
 const std::vector<std::vector<rg::TruckRoutePoint>> legs{lane(-60,-1.75,0,60),lane(-1.75,0,-kHalfPi,60)};
 const auto old_join=check_path(rg::join_route_legs(legs,{{0,0}}));
 CHECK(old_join.min_forward<0);                  // the pre-turn-path route: a backwards hop between the lane offsets
 const auto route=rg::join_route_legs(legs,{{8,8}});
 const auto c=check_path(route);
 CHECK(c.monotone);
 CHECK(c.min_forward>0.95);
 CHECK(c.max_curvature<1/5.0);                   // tighter than a 5 m radius nowhere (the corner is 6.25 m from both lane lines)
 // Starts on the approach lane, ends on the exit lane, never leaves the quadrant the kerb return rounds.
 CHECK(route.points.front().ground.y==Catch::Approx(-1.75));CHECK(route.points.back().ground.x==Catch::Approx(-1.75));
 for(const auto& p:route.points)CHECK((p.ground.x<=-1.75+1e-6||p.ground.y<=-1.75+1e-6));
 CHECK(route.points.back().station<120);
}
TEST_CASE("NPC turn paths: a left turn and a straight joint stay smooth", "[npc_traffic][junction_turn]") {
 // Left turn: eastbound into the node, then northbound 1.75 m east of the crossing road's centreline.
 const auto left=rg::join_route_legs({lane(-60,-1.75,0,60),lane(1.75,0,kHalfPi,60)},{{8,8}});
 const auto c=check_path(left);
 CHECK(c.monotone);CHECK(c.min_forward>0.95);CHECK(c.max_curvature<1/7.0);
 // Straight on across a stretch split: one straight line, constant heading.
 const auto straight=rg::join_route_legs({lane(-60,-1.75,0,60),lane(0,-1.75,0,60)},{{2,2}});
 for(const auto& p:straight.points){CHECK(p.ground.y==Catch::Approx(-1.75));CHECK(std::abs(p.yaw)<1e-9);}
 CHECK(straight.points.back().station==Catch::Approx(120));
}
TEST_CASE("NPC turn paths: cuts longer than a leg keep a tenth of it; overlapping cuts fall back to the plain join", "[npc_traffic][junction_turn]") {
 const auto route=rg::join_route_legs({lane(-60,-1.75,0,60),lane(-1.75,0,-kHalfPi,10),lane(-1.75,-10,-kHalfPi,60)},{{8,30},{30,2}});
 const auto c=check_path(route);
 CHECK(c.monotone);CHECK(c.min_forward>0.9);
 bool kept=false;for(const auto& p:route.points)if(std::abs(p.ground.x+1.75)<1e-6&&p.ground.y<-0.5&&p.ground.y>-10)kept=true;
 CHECK(kept);                                    // part of the 10 m middle leg survives the cuts
 // A turn whose cut points lie behind each other (no room) is joined straight, not with a loop.
 const auto tight=rg::join_route_legs({lane(-60,-1.75,0,60),lane(-1.75,0,-kHalfPi,60)},{{0.1,0.1}});
 CHECK(check_path(tight).monotone);
}
TEST_CASE("NPC turn paths: cuts come from the served trims, else the road widths, capped by the turn angle", "[npc_traffic][junction_turn]") {
 CHECK(rg::junction_turn_cuts(0.05,7,7)==std::pair<double,double>{2.,2.});
 const auto fallback=rg::junction_turn_cuts(kHalfPi,6,10);
 CHECK(fallback.first==Catch::Approx(8));CHECK(fallback.second==Catch::Approx(6.5));   // 6 m from the width, raised to the 4 m radius cut
 const auto served=rg::junction_turn_cuts(kHalfPi,6,10,11,9.5);
 CHECK(served.first==Catch::Approx(11));CHECK(served.second==Catch::Approx(9.5));
 // A tiny trim on a narrow crossing: at least the cut of a 4 m radius right angle + 2.5 m.
 CHECK(rg::junction_turn_cuts(kHalfPi,4,4,1.5,1.5).first==Catch::Approx(6.5));
 // A 40 m merge-wedge trim on a 6 degree joint: capped near the cut of a 30 m radius turn (+2 m).
 const auto merge=rg::junction_turn_cuts(0.105,7,7,40,40);
 CHECK(merge.first<=2+30*std::tan(0.105/2)+1e-9);CHECK(merge.first>=3);
 // A right angle never exceeds the 30 m radius cut either.
 CHECK(rg::junction_turn_cuts(kHalfPi,7,7,45,45).first==Catch::Approx(32));
}
TEST_CASE("NPC turn paths: on a bending road the curve starts where the lane still points at the junction", "[npc_traffic][junction_turn]") {
 // Approach on a quarter circle of radius 12 m (centre (0, 12)) ending at the origin heading east, then straight on east.
 std::vector<rg::TruckRoutePoint> arc;
 // arc: from (-12, 12) heading south (yaw -pi/2) to (0, 0) heading east (yaw 0)
 for(int i=0;i<=40;++i){const double phi=kHalfPi*i/40;rg::TruckRoutePoint p;
  p.ground={-12*std::cos(phi),12-12*std::sin(phi),0};p.yaw=phi-kHalfPi;arc.push_back(p);}
 REQUIRE(arc.back().ground.x==Catch::Approx(0).margin(1e-9));REQUIRE(arc.back().ground.y==Catch::Approx(0).margin(1e-9));
 const auto route=rg::join_route_legs({arc,lane(0,0,0,40)},{{30,30}});
 // Every point stays on the road: within 1 m of the circle (radius 12 about (0, 12)) or of the straight y = 0.
 for(const auto& p:route.points){
  const double to_circle=std::abs(std::hypot(p.ground.x,p.ground.y-12)-12),to_line=p.ground.x>=0?std::abs(p.ground.y):1e9;
  CHECK(std::min(to_circle,to_line)<1.0);
 }
}
TEST_CASE("Right of way rules: signs, then road class, then right before left; signal phases never overlap", "[npc_traffic][right_of_way]") {
 rg::TrafficJunction minor;minor.rank=1;rg::TrafficJunction major;major.rank=4;
 CHECK(rg::junction_yields(minor,major,false));CHECK_FALSE(rg::junction_yields(major,minor,true));
 rg::TrafficJunction signed_major=major;signed_major.sign_yield=true;
 CHECK(rg::junction_yields(signed_major,minor,false));CHECK_FALSE(rg::junction_yields(minor,signed_major,true));
 CHECK(rg::junction_yields(minor,minor,true));CHECK_FALSE(rg::junction_yields(minor,minor,false));
 g2m::RoadGraphWay way;way.tags={{"highway","secondary_link"}};CHECK(rg::traffic_road_rank(way)==4);
 way.tags={{"highway","residential"}};CHECK(rg::traffic_road_rank(way)==1);
 way.tags={{"highway","living_street"}};CHECK(rg::traffic_road_rank(way)==0);
 for(std::int64_t node:{1,56,123456789}){int green0=0,green1=0;
  for(int k=0;k<5600;++k){const double t=k*.01;const bool a=rg::traffic_signal_green(node,0,t),b=rg::traffic_signal_green(node,1,t);
   CHECK_FALSE((a&&b));green0+=a;green1+=b;}
  CHECK(green0==Catch::Approx(2500).margin(2));CHECK(green1==Catch::Approx(2500).margin(2));}
 // join_route_legs reports where each turn curve starts and ends.
 std::vector<std::pair<double,double>> joints;
 const auto route=rg::join_route_legs({lane(-60,-1.75,0,60),lane(-1.75,0,-kHalfPi,60)},{{8,8}},&joints);
 REQUIRE(joints.size()==1);
 CHECK(joints[0].first==Catch::Approx(52));CHECK(joints[0].second>joints[0].first+8);CHECK(joints[0].second<route.points.back().station-50);
}
