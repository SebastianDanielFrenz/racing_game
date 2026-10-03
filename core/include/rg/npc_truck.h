#pragma once
#include "ps/math/pose.h"
#include "g2m/layer/road_graph.h"
#include <atomic>
#include <memory>
#include <string>
#include <vector>
namespace rg {
class WorldTerrain;
struct TruckLane { bool allowed=false; double offset_m=0; };
TruckLane truck_lane(const g2m::RoadGraphWay&,double width_m,int lanes,int direction);
struct TruckRoutePoint { ps::Vec3 ground; double yaw=0,grade=0,speed_m_s=0,station=0; std::int64_t way_id=0; };
struct TruckRoute { std::vector<TruckRoutePoint> points; std::string message; };
TruckRoute build_truck_route(std::shared_ptr<WorldTerrain>,const ps::Pose&,double target_speed,const std::atomic<bool>& cancel);
struct TruckSnapshot { bool active=false,loading=false; ps::Pose pose{}; double speed_m_s=0; std::string message; };
}
