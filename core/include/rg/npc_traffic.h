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
// joints (optional): per joint, the route stations where its turn curve starts and ends.
TruckRoute join_route_legs(const std::vector<std::vector<TruckRoutePoint>>& legs,const std::vector<std::pair<double,double>>& cuts,
 std::vector<std::pair<double,double>>* joints=nullptr);
// Where the turn path through a junction starts (before the node, on the approach) and ends (after it, on the exit).
// Served trims (< 0 = unknown) set them where known, so the turn runs over the junction surface; without a trim, half
// the width of the road being entered plus 3 m (2 m for an almost straight joint); never less than the cut of a 4 m
// radius turn plus 2.5 m. Either is capped at the cut of a
// 30 m radius turn for this heading change (+ 2 m), so a long merge wedge or a gentle bend never gets a curve that
// leaves a bending road.
std::pair<double,double> junction_turn_cuts(double heading_change_rad,double width_in_m,double width_out_m,double trim_in_m=-1,double trim_out_m=-1);
// One junction a trip drives through: the OSM node, the stations where the turn path enters and leaves it, and this
// approach's right of way (StVO 8): its road rank, a give-way or stop line on it, and its signal phase (-1 = no signal).
struct TrafficJunction {std::int64_t node=0;double enter_station=0,exit_station=0;int rank=0;bool sign_yield=false;int signal_phase=-1;};
// Road class rank for right of way: motorway 14, trunk 12 ... residential 2, living_street 0 (even steps); every
// *_link ranks 1, so it yields where it merges or ends (StVO 18(3)). Unknown classes (service, track) rank 0.
int traffic_road_rank(const g2m::RoadGraphWay&);
// Whether approach `a` must let approach `b` (both at the same unsignalled junction) go first: a give-way/stop line
// yields to an approach without one; else the lower road rank yields; equal ranks: right before left (b_from_right).
bool junction_yields(const TrafficJunction& a,const TrafficJunction& b,bool b_from_right);
// Fixed-time signal: two phases of 25 s green with 3 s all-red between them, offset per node so neighbouring signals do
// not switch together. `phase` 0 or 1 (approaches on the same axis share a phase).
bool traffic_signal_green(std::int64_t node,int phase,double time_s);
struct TrafficTrip {TruckRoute route;TrafficDestination destination;bool truck=false;double desired_speed=0;std::vector<TrafficJunction> junctions;};
struct TrafficPlan {std::vector<TrafficTrip> trips;double road_length_m=0;std::size_t destinations=0;std::string message;};
TrafficPlan plan_traffic(std::shared_ptr<WorldTerrain>,ps::Vec3 player,TrafficConfig,std::uint64_t seed,const std::atomic<bool>& cancel,bool cached_geometry_only=false);
struct TrafficActorSnapshot {std::uint64_t id=0;bool truck=false;ps::Pose pose{};double speed_m_s=0;TrafficDestination destination;};
struct TrafficSnapshot {std::vector<TrafficActorSnapshot> actors;TrafficConfig config;int target=0,queued=0;bool loading=false;std::string message;};
}
