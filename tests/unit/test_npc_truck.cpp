#include "rg/npc_truck.h"
#include "rg/session.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"
#include "ps/io/vehicle_io.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <chrono>
#include <thread>
#include <cstdio>
TEST_CASE("Truck lanes honor one-way direction and rightmost through lanes", "[npc_truck]") {
 g2m::RoadGraphWay way;
 auto forward=rg::truck_lane(way,6.5,2,1),reverse=rg::truck_lane(way,6.5,2,-1);
 REQUIRE(forward.allowed);REQUIRE(reverse.allowed);
 CHECK(forward.offset_m==Catch::Approx(-1.625));CHECK(reverse.offset_m==Catch::Approx(1.625));
 way.tags={{"oneway","yes"},{"turn:lanes","left|through|right"}};
 CHECK_FALSE(rg::truck_lane(way,9.75,3,-1).allowed);
 CHECK(rg::truck_lane(way,9.75,3,1).offset_m==Catch::Approx(0));
 way.tags={{"oneway","-1"}};
 CHECK_FALSE(rg::truck_lane(way,6.5,2,1).allowed);CHECK(rg::truck_lane(way,6.5,2,-1).allowed);
 CHECK_FALSE(rg::truck_lane(way,2.5,1,-1).allowed);
 CHECK(rg::truck_lane(way,3.2,6,-1).offset_m==Catch::Approx(0));
}
// A two-way road with a single lane in total: each direction drives in the centre of ITS half. Both on the centre line
// (the old behaviour, offset 0) made every oncoming pair a head-on mutual yield - 153 of 190 deadlock roots measured.
// Sabotage: returning offset 0 for n == 1 two-way makes both CHECKs below fail.
TEST_CASE("Single-lane two-way roads put the directions in opposite halves", "[npc_truck]") {
 g2m::RoadGraphWay way;
 const auto forward=rg::truck_lane(way,3.2,1,1),reverse=rg::truck_lane(way,3.2,1,-1);
 REQUIRE(forward.allowed);REQUIRE(reverse.allowed);
 CHECK(forward.offset_m==Catch::Approx(-0.8));CHECK(reverse.offset_m==Catch::Approx(0.8));
 way.tags={{"oneway","yes"}};
 CHECK(rg::truck_lane(way,3.2,1,1).offset_m==Catch::Approx(0)); // a one-way road stays centred
}
TEST_CASE("NPC truck publishes moving collision body and reduces trailing airspeed", "[npc_truck]") {
 auto desc=ps::io::load_vehicle_json(std::string(RG_SOURCE_DIR)+"/external/physics_sim/data/vehicles/car_sedan.json");
 desc.aero.drag_area_m2=0;desc.aero.body.drag_area_xyz_m2={.6,.1,.1};desc.aero.body.reference_area_m2=2;
 rg::SessionConfig cfg;cfg.vehicle_definition=std::make_shared<const ps::vehicle::VehicleDesc>(desc);
 cfg.surface_table_path=std::string(RG_SOURCE_DIR)+"/external/physics_sim/data/surfaces/surfaces.json";
 cfg.chassis_initial_velocity={20,0,0};
 rg::Session session(cfg);session.request_npc_truck(true,70);session.start();
 bool active=false;
 const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
 while(std::chrono::steady_clock::now()<deadline) {
  const auto& snapshot=session.snapshot();
  active=snapshot.truck.active;
  if(active&&snapshot.tick>80)break;
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
 }
 session.stop();REQUIRE(active);
 const auto frame=session.snapshot();
 CHECK(frame.truck.speed_m_s>5);CHECK(frame.truck.pose.position.x>45);
 const auto air=session.world().vehicle_aero_telemetry(session.vehicle_id()).body_airspeed_m_s;
 const auto speed=session.world().get_motion(session.chassis_body()).linear.length();
 CHECK(air<speed*.98);
 auto hit=session.world().backend().ray_cast(frame.truck.pose.position+ps::Vec3{-15,0,0},ps::Vec3::unit_x(),30);
 REQUIRE(hit.hit);
 // F preserves nearby traffic and its controller resumes instead of restarting a route worker.
 const auto before_flip=session.snapshot().truck.pose.position.x;
 session.request_flip_upright();session.start();
 std::this_thread::sleep_for(std::chrono::milliseconds(120));session.stop();
 REQUIRE(session.snapshot().truck.active);
 CHECK(session.snapshot().truck.pose.position.x>before_flip);
 session.request_npc_truck(false,70);session.start();
 std::this_thread::sleep_for(std::chrono::milliseconds(40));session.stop();
 CHECK_FALSE(session.snapshot().truck.active);
}
TEST_CASE("Truck route prepares the owner real-world spawn off the simulation thread", "[.][npc_truck_real]") {
 std::string error;auto cfg=rg::load_world_config(std::string(RG_SOURCE_DIR)+"/data/world/world_config.json",&error);REQUIRE(cfg);
 std::shared_ptr<rg::WorldTerrain> terrain(rg::WorldTerrain::open(*cfg,&error));REQUIRE(terrain);
 const auto config=rg::make_terrain_mode(*cfg,terrain);
 ps::Pose car;car.position={config.spawn_x,config.spawn_y,251.5};car.orientation=ps::Quat::from_axis_angle(ps::Vec3::unit_z(),config.spawn_yaw_rad);
 std::atomic<bool> cancel{false};auto route=rg::build_truck_route(terrain,car,70./3.6,cancel);
 std::printf("NPC_ROUTE points=%zu message=%s\n",route.points.size(),route.message.c_str());
 INFO(route.message);REQUIRE(route.points.size()>30);
 // Owner-reported B8 section, travelling northwest toward Kelkheim.
 const auto b8=terrain->frame().from_geodetic(50.141900,8.479882);
 ps::Pose highway;highway.position={b8.x,b8.y,0};highway.orientation=ps::Quat::from_axis_angle(ps::Vec3::unit_z(),2.4);
 auto geometry=terrain->road_geometry_at(terrain->frame().grid_easting(b8.x),terrain->frame().grid_northing(b8.y));REQUIRE(geometry);
 double best=1e30;
 for(const auto& entry:geometry->entries)if(entry.profile)for(double station=0;station<=entry.profile->reference.length_m;station+=3) {
  auto xy=entry.profile->reference.at(station);auto height=entry.profile->at(station);if(!xy||!height)continue;
  const double dx=xy->x-terrain->frame().grid_easting(b8.x),dy=xy->y-terrain->frame().grid_northing(b8.y);
  if(dx*dx+dy*dy<best){best=dx*dx+dy*dy;highway.position.z=height->height_m+.6;}
 }
 auto highway_route=rg::build_truck_route(terrain,highway,70./3.6,cancel);
 std::printf("NPC_B8_ROUTE points=%zu message=%s\n",highway_route.points.size(),highway_route.message.c_str());
 INFO(highway_route.message);REQUIRE(highway_route.points.size()>200);
 for(std::size_t i=1;i<route.points.size();++i) {
  CHECK((route.points[i].ground-route.points[i-1].ground).length()<6.01);
  CHECK(route.points[i].station>=route.points[i-1].station);
 }
}
