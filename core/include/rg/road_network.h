// rg/road_network.h - a directed, drivable road network built from the g2m
// roads.graph layer (G3/R3 slice S3, docs/g3_consumer_design.md 6.1), plus the
// junction clusters and bridge runs a route generator needs and a planner on top.
//
// Engine-neutral and Godot-free. The builder takes plain ways/nodes (a g2m tile
// is only one source: add_road_graph_tile), so tests build synthetic networks
// without a world. Everything is deterministic: ordered containers only, no
// randomness, no transcendental maths (sqrt and arithmetic only).
//
// Model.
//  - A node is an OSM node (id, session metres). A way is an ordered list of node
//    refs plus tags. Level-2 road graph tiles overlap, so the same way/node can
//    arrive twice: they are merged by id (the first copy wins).
//  - An ARC is one directed segment between two consecutive refs of a drivable
//    way. Oneway ("yes"/"1"/"true" forward, "-1" backward; motorway and
//    junction=roundabout imply forward) and access restrictions decide which
//    directions exist. Drivable classes: motorway, trunk, primary, secondary,
//    tertiary, unclassified, residential and their _link variants. (kDrivableClassesVersion
//    is part of a generated route's provenance.)
//  - A JUNCTION is a cluster (kJunctionClusterM) of OSM nodes with >= 3 distinct
//    drivable neighbours and >= 2 distinct ways. A cluster's identity is its node
//    id set (sorted) - OSM node ids survive a data-pin bump, positions and ids of
//    derived records do not, so this is what a consumer re-resolves against later.
//  - A BRIDGE RUN is a maximal run of consecutive arcs of one way with bridge=*
//    (not "no"), in one direction.
//
// Planner. shortest_arc_path() is Dijkstra over arc states (not nodes) so a
// turn is a transition between two arcs and a U-turn - going back along the arc
// just driven - can be forbidden. It is the building block of the generator
// (tools/rg_route_gen).
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace g2m {
struct RoadGraphTile;
}

namespace rg {

inline constexpr int kDrivableClassesVersion = 1;
inline constexpr double kJunctionClusterM = 30.0;

struct RoadNetNode {
    std::int64_t osm_id = 0;
    double x = 0.0; // session metres
    double y = 0.0;
};

struct RoadNetWay {
    std::int64_t osm_id = 0;
    std::vector<std::int64_t> node_ids;
    std::vector<std::pair<std::string, std::string>> tags;
};

struct RoadArc {
    int from = -1; // node index
    int to = -1;
    std::int64_t way_id = 0;
    int way = -1;          // way index
    int segment = 0;       // ref-pair index within the way (0 = refs 0->1)
    int direction = 1;     // +1: along the way's node order, -1: against
    double length = 0.0;
    bool bridge = false;
};

struct RoadJunction {
    std::vector<std::int64_t> node_ids; // sorted
    double x = 0.0;                     // centroid of the member nodes
    double y = 0.0;
    double radius_m = 0.0;              // max member distance from the centroid
};

struct BridgeRun {
    std::int64_t way_id = 0;
    int direction = 1;
    std::int64_t enter_node = 0;
    std::int64_t exit_node = 0;
    double length_m = 0.0;
    std::vector<int> arcs; // arc indices, in travel order
};

struct RoadNetworkStats {
    std::size_t nodes = 0, ways = 0, arcs = 0, junction_nodes = 0, junctions = 0;
};

class RoadNetwork {
public:
    // Builder phase: add everything, then finish() once.
    void add_node(const RoadNetNode& node);
    void add_way(const RoadNetWay& way);
    // Adds every node and way of a g2m road graph tile; positions are UTM mm in the tile,
    // converted to session metres with the session origin e0/n0.
    void add_road_graph_tile(const g2m::RoadGraphTile& tile, double e0, double n0);
    // Builds arcs, junction nodes and clusters. Drops ways whose class is not drivable and
    // ways with an unknown node. Idempotent only in the sense that it must be called once.
    void finish();

    [[nodiscard]] const std::vector<RoadNetNode>& nodes() const { return nodes_; }
    [[nodiscard]] const std::vector<RoadNetWay>& ways() const { return ways_; }
    [[nodiscard]] const std::vector<RoadArc>& arcs() const { return arcs_; }
    [[nodiscard]] const std::vector<RoadJunction>& junctions() const { return junctions_; }
    [[nodiscard]] RoadNetworkStats stats() const;

    [[nodiscard]] int node_index(std::int64_t osm_id) const; // -1 if unknown
    [[nodiscard]] int way_index(std::int64_t osm_id) const;  // -1 if unknown
    // Junction cluster index of a node (-1 if the node is not in one).
    [[nodiscard]] int junction_of_node(int node) const;
    // Outgoing arc indices of a node, ascending.
    [[nodiscard]] const std::vector<int>& out_arcs(int node) const { return out_[static_cast<std::size_t>(node)]; }
    // The arcs of a way in one direction, in travel order (empty if that direction is closed).
    [[nodiscard]] std::vector<int> way_arcs(std::int64_t way_id, int direction) const;
    // Maximal bridge runs, ascending (way id, direction, first segment).
    [[nodiscard]] std::vector<BridgeRun> bridge_runs() const;
    // Directed arc nearest to (x, y) whose direction is within `max_angle_cos` (dot of unit
    // headings >= it) of the heading (hx, hy); -1 if none within `max_dist_m`.
    // `t_out` is the fraction along the arc of the projection.
    [[nodiscard]] int nearest_arc(double x, double y, double hx, double hy, double max_dist_m, double min_cos,
                                  double* t_out = nullptr, double* dist_out = nullptr) const;

    // Dijkstra over arcs from the END of `from_arc` to the START of `to_arc` (so the path
    // returned excludes both and is empty when to_arc directly follows from_arc). U-turns
    // (the arc whose reverse is the previous arc) are never taken. Returns nullopt if
    // unreachable within `max_length_m`.
    [[nodiscard]] std::optional<std::vector<int>> shortest_arc_path(int from_arc, int to_arc,
                                                                    double max_length_m = 1.0e9) const;

    // Distinct junction clusters visited by a node sequence, in first-visit order.
    [[nodiscard]] std::vector<int> junctions_on_arcs(const std::vector<int>& arcs) const;

private:
    std::vector<RoadNetNode> nodes_;
    std::vector<RoadNetWay> ways_;
    std::map<std::int64_t, std::size_t> node_by_id_, way_by_id_;
    std::vector<RoadArc> arcs_;
    std::vector<std::vector<int>> out_;
    std::vector<RoadJunction> junctions_;
    std::vector<int> node_junction_;
    std::size_t junction_nodes_ = 0;
    bool finished_ = false;

    // Pending input (nodes may arrive after ways and in tile overlap).
    std::map<std::int64_t, RoadNetNode> pending_nodes_;
    std::map<std::int64_t, RoadNetWay> pending_ways_;
};

// One step of a planned route: traverse OSM way `id` in `direction` (+1 along its node order, -1
// against; every arc of it, in travel order), or reach OSM node `id` (the cheapest arc arriving there).
struct RouteStep {
    enum class Kind { TraverseWay, ViaNode };
    Kind kind = Kind::TraverseWay;
    std::int64_t id = 0;
    int direction = 1;
};

struct PlannedPath {
    std::vector<int> arcs;       // arc indices in travel order; arcs[0] is the arc the start lies on
    double start_t = 0.0;        // fraction along arcs[0] of the start point's projection
    double start_dist_m = 0.0;   // distance from the start point to that arc
    std::vector<double> step_approach_m; // per step: path length from the previous position to the step
};

// Plans a drivable path: starts on the directed arc nearest (sx, sy) heading (hx, hy) (within 15 m and
// 60 degrees), then chains the steps with shortest_arc_path (oneway honoured, no U-turns). Returns
// nullopt and sets *err ("step N: ...") when the start is off the network, a way has no arcs in the
// asked direction, or a step is unreachable.
std::optional<PlannedPath> plan_path(const RoadNetwork& net, double sx, double sy, double hx, double hy,
                                     const std::vector<RouteStep>& steps, std::string* err);

// Smooths a polyline for a driver: every vertex whose turn angle exceeds ~3 degrees is replaced by a
// quadratic Bezier corner of tangent length min(radius_m * tan(turn/2), 0.45 * shorter adjacent
// segment), then the result is resampled to a spacing <= max_spacing_m. The first and last points
// are kept exactly. No transcendental maths.
std::vector<std::pair<double, double>> smooth_polyline(const std::vector<std::pair<double, double>>& pts,
                                                       double radius_m, double max_spacing_m);

} // namespace rg
