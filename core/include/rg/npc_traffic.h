#pragma once
#include "rg/npc_truck.h"
#include "g2m/layer/src_osm.h"
#include "g2m/core/geo/session_frame.h"
#include <random>
namespace rg {
struct TrafficConfig {
 double density_per_km=120,radius_m=1200,min_spawn_m=100,grip_multiplier=1;
 int max_vehicles=2048;
};
struct TrafficDestination {std::int64_t id=0;bool relation=false,parking=false;ps::Vec3 point{};int osm_type=1;};
bool traffic_road_allowed(const g2m::RoadGraphWay&,bool truck,int direction);
double traffic_speed_cap(const g2m::RoadGraphWay&,bool truck,int direction,double desired_m_s);
TrafficConfig sanitize_traffic_config(TrafficConfig);
std::vector<TrafficDestination> extract_traffic_destinations(const g2m::OsmTile&,const g2m::geo::SessionFrame&);
// Joins the per-edge lane polylines of a route into one drivable path. legs[k] is the lane line of path edge k, its
// points in driving order; cuts[k] = {how far before the end of leg k, how far after the start of leg k+1} the turn
// through the junction between them begins and ends (the served junction trims where known). Each joint becomes a
// cubic Bezier between the two cut points, tangent to both lane lines, with handles of a circular arc for the heading
// change, so a turn follows a kerb-return-like curve instead of meeting at the node with a heading jump (and, for a
// right turn, a backwards hop between the two lane offsets). Cuts end where the lane has turned 20 degrees from its
// heading at the junction (a bending road) and are capped so a leg keeps at least 10 % of its
// length. Stations are recomputed from the result.
TruckRoute join_route_legs(const std::vector<std::vector<TruckRoutePoint>>& legs,const std::vector<std::pair<double,double>>& cuts);
// Where the turn path through a junction starts (before the node, on the approach) and ends (after it, on the exit).
// Served trims (< 0 = unknown) set them where known, so the turn runs over the junction surface; without a trim, half
// the width of the road being entered plus 3 m (2 m for an almost straight joint); never less than the cut of a 4 m
// radius turn plus 2.5 m. Either is capped at the cut of a
// 30 m radius turn for this heading change (+ 2 m), so a long merge wedge or a gentle bend never gets a curve that
// leaves a bending road.
std::pair<double,double> junction_turn_cuts(double heading_change_rad,double width_in_m,double width_out_m,double trim_in_m=-1,double trim_out_m=-1);
struct TrafficTrip {TruckRoute route;TrafficDestination destination;bool truck=false;double desired_speed=0;};
struct TrafficPlan {std::vector<TrafficTrip> trips;double road_length_m=0;std::size_t destinations=0;std::string message;};
TrafficPlan plan_traffic(std::shared_ptr<WorldTerrain>,ps::Vec3 player,TrafficConfig,std::uint64_t seed,const std::atomic<bool>& cancel,bool cached_geometry_only=false);
struct TrafficActorSnapshot {std::uint64_t id=0;bool truck=false;ps::Pose pose{};double speed_m_s=0;TrafficDestination destination;};
struct TrafficSnapshot {std::vector<TrafficActorSnapshot> actors;TrafficConfig config;int target=0,queued=0;bool loading=false;std::string message;};
}
