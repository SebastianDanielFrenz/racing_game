// rg/route_planner.h — R4 route planning over geo2map's G4 routing library
// (g2m_route, docs/routing_design.md in the geo2map_engine submodule).
//
// Engine-neutral. A RoutePlanner owns one worker thread. At construction the
// worker assembles the region's routing graph from level-2 g2m.roads.graph
// tiles (assemble_region_graph + build_region_routing with ALT landmarks, the
// design's "built on a worker at world load"); after that it serves route
// requests: snap start (with the car's heading) and destination, A* + ALT,
// then the route polyline (session metres) and the guidance manoeuvres.
// Requests coalesce: only the newest one is planned. The game polls.
// route_progress() turns a planned route and the car's position into the
// HUD's "next manoeuvres" (pure, any thread).
//
// Coordinates: session-local metres (x east, y north from the session
// origin e0/n0); geo2map works in absolute UTM millimetres.
// Turn restrictions: roads.graph carries none yet (design slice 3), so the
// graph is built without them.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/road_graph_tile.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace rg {

// Every number is data (data/world/route_planner.json, "rg.route_planner/1").
struct RoutePlannerConfig {
    double region_radius_m = 10000.0;      // routing graph: level-2 tiles within this square half-size of the centre
    std::size_t landmarks = 16;            // ALT landmarks (0 = plain A*)
    double snap_max_distance_m = 50.0;     // start/destination farther than this from a routable road: refused
    double heading_tolerance_deg = 60.0;   // start snap: road directions within this of the car's heading
    std::size_t maneuvers_ahead = 3;       // route_progress' default count
    double rejoin_window_back_m = 100.0;   // route_progress: search the polyline from hint - back ...
    double rejoin_window_ahead_m = 500.0;  // ... to hint + ahead first, the whole line only when off it
    double off_route_m = 40.0;             // farther than this from the line = off route
    double minimap_radius_m = 1000.0;      // the minimap's view radius (map LOD selection)
    std::size_t map_cache_max_tiles = 512; // rg::MapView clears its tile cache beyond this
};
std::optional<RoutePlannerConfig> parse_route_planner_config(const std::string& json_text, const std::string& origin,
                                                             std::string* err);
std::optional<RoutePlannerConfig> load_route_planner_config(const std::string& path, std::string* err);

// A level-2 g2m.roads.graph tile by key: null = fetch failure (aborts the
// graph build); an empty tile = no data there. Must be safe to call from the
// planner's worker (WorldTerrain::road_graph_tile is).
using RoadGraphTileSource = std::function<std::shared_ptr<const g2m::RoadGraphTile>(const g2m::TileKey&)>;

// Tiles of `level` overlapping the square [cx +- half, cy +- half] (session metres), sorted.
std::vector<g2m::TileKey> tiles_in_square(g2m::geo::UtmZone zone, int level, double e0, double n0, double cx,
                                          double cy, double half_m);

struct PlannedManeuver {
    std::string kind;          // g2m::route::to_string(ManeuverKind): "depart", "left", "roundabout_enter", ...
    double distance_from_start_m = 0.0;
    double x = 0.0, y = 0.0;   // session metres
    int angle_deg = 0;         // heading change, positive = left
    int exit_number = 0;       // roundabouts
    std::string name, ref, destination;
};

struct PlannedRoute {
    enum class Status { Ok, Unreachable, NoStart, NoDestination, NoGraph, Failed };
    std::uint64_t serial = 0;  // RoutePlanner::request's return value
    Status status = Status::Failed;
    std::string message;
    std::vector<std::array<double, 2>> polyline; // session metres, travel order
    std::vector<double> polyline_s;              // distance along the line at each point (m)
    double length_m = 0.0;
    double duration_s = 0.0;
    std::vector<PlannedManeuver> maneuvers;      // depart first, arrive last
};
const char* to_string(PlannedRoute::Status status);

struct UpcomingManeuver {
    PlannedManeuver maneuver;
    double distance_m = 0.0; // from the car (along the route)
};
struct RouteProgress {
    bool valid = false;        // a route with a polyline
    double along_m = 0.0;      // the car's position along the route
    double off_route_m = 0.0;  // distance from the line
    bool off_route = false;
    double remaining_m = 0.0;
    std::vector<UpcomingManeuver> next;
};
// The car at (x, y): projected onto the route (first within the rejoin window
// around hint_along_m, the whole line when that is farther than off_route_m),
// and the next `count` manoeuvres ahead of it (arrive included).
RouteProgress route_progress(const PlannedRoute& route, double x, double y, double hint_along_m, std::size_t count,
                             const RoutePlannerConfig& config = {});

class RoutePlanner {
public:
    enum class GraphState { Building, Ready, Failed };
    struct GraphStats {
        std::size_t tiles = 0, nodes = 0, arcs = 0, landmarks = 0;
        double build_ms = 0.0;
    };

    // Starts the worker; it fetches `keys` (level-2) and builds the graph at once.
    RoutePlanner(RoadGraphTileSource source, std::vector<g2m::TileKey> keys, double e0, double n0,
                 RoutePlannerConfig config);
    ~RoutePlanner(); // cancels the graph build (between tile fetches) and joins
    RoutePlanner(const RoutePlanner&) = delete;
    RoutePlanner& operator=(const RoutePlanner&) = delete;

    [[nodiscard]] GraphState graph_state() const;
    [[nodiscard]] std::string graph_message() const;
    [[nodiscard]] GraphStats graph_stats() const;
    [[nodiscard]] const RoutePlannerConfig& config() const { return config_; }

    // Plans from (sx, sy) driving along (hx, hy) (zero = no heading) to (dx, dy).
    // Non-blocking; returns the request's serial. A newer request supersedes an older one not yet started.
    std::uint64_t request(double sx, double sy, double hx, double hy, double dx, double dy);
    // The newest finished route not yet polled.
    bool poll(PlannedRoute& out);
    // Blocks until the request `serial` (or a newer one) has finished or the graph failed; tests and tools.
    bool wait(std::uint64_t serial, PlannedRoute& out);

private:
    struct Impl;
    void worker_main();

    RoadGraphTileSource source_;
    std::vector<g2m::TileKey> keys_;
    double e0_, n0_;
    RoutePlannerConfig config_;
    std::unique_ptr<Impl> impl_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    GraphState state_ = GraphState::Building;
    std::string message_;
    GraphStats stats_;
    bool stop_ = false;
    std::atomic<bool> cancel_{false};
    std::uint64_t next_serial_ = 1;
    bool has_request_ = false;
    std::uint64_t request_serial_ = 0;
    std::array<double, 6> request_{};
    bool has_result_ = false;
    PlannedRoute result_;
    std::uint64_t finished_serial_ = 0;
    std::thread worker_; // last
};

} // namespace rg
