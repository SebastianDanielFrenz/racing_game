#pragma once
#include "rg/npc_truck.h"
#include "g2m/layer/src_osm.h"
#include "g2m/core/geo/session_frame.h"
#include <random>
namespace rg {
struct TrafficConfig {
 double density_per_km=4,radius_m=600,min_spawn_m=100,grip_multiplier=1;
 int max_vehicles=24;
};
struct TrafficDestination {std::int64_t id=0;bool relation=false,parking=false;ps::Vec3 point{};int osm_type=1;};
bool traffic_road_allowed(const g2m::RoadGraphWay&,bool truck,int direction);
double traffic_speed_cap(const g2m::RoadGraphWay&,bool truck,int direction,double desired_m_s);
TrafficConfig sanitize_traffic_config(TrafficConfig);
std::vector<TrafficDestination> extract_traffic_destinations(const g2m::OsmTile&,const g2m::geo::SessionFrame&);
struct TrafficTrip {TruckRoute route;TrafficDestination destination;bool truck=false;double desired_speed=0;};
struct TrafficPlan {std::vector<TrafficTrip> trips;double road_length_m=0;std::size_t destinations=0;std::string message;};
TrafficPlan plan_traffic(std::shared_ptr<WorldTerrain>,ps::Vec3 player,TrafficConfig,std::uint64_t seed,const std::atomic<bool>& cancel);
struct TrafficActorSnapshot {std::uint64_t id=0;bool truck=false;ps::Pose pose{};double speed_m_s=0;TrafficDestination destination;};
struct TrafficSnapshot {std::vector<TrafficActorSnapshot> actors;TrafficConfig config;bool loading=false;std::string message;};
}
