#include "rg/npc_traffic.h"
#include "rg/world_terrain.h"
#include "rg/world_config.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
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
