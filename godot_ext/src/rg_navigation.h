// godot_ext/src/rg_navigation.h - RgNavigation: the thin Godot binding of the
// R4 route core (rg/route_planner.h: region routing graph, route requests,
// route_progress; rg/map_view.h: minimap geometry). All logic is in rg_core;
// this class converts to Variants and owns the two worker-backed objects for
// one world (initialize_shared ... release).
#pragma once

#include "rg/lod_settings.h"
#include "rg/map_view.h"
#include "rg/route_planner.h"

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>

#include <memory>

namespace rg_godot {

class RgSimulation;

class RgNavigation : public godot::Node {
    GDCLASS(RgNavigation, godot::Node)

public:
    ~RgNavigation() override;

    // Reuses the Session's WorldTerrain (real world only); starts the region graph build around
    // (center_x, center_y) (session metres) on a worker. config_path = data/world/route_planner.json.
    bool initialize_shared(RgSimulation* sim, const godot::String& config_path, double center_x, double center_y);
    void release();
    [[nodiscard]] bool is_active() const { return planner_ != nullptr; }
    [[nodiscard]] godot::String get_last_error() const { return last_error_; }
    // {state: building|ready|failed, message, tiles, nodes, arcs, landmarks, build_ms}
    [[nodiscard]] godot::Dictionary get_graph_status() const;
    [[nodiscard]] double get_minimap_radius_m() const { return config_.minimap_radius_m; }

    // In-game LOD settings (RgShell.get_lod_choice()): map_detail -> MapConfig::detail_scale. Kept across worlds.
    void set_view_lod(const godot::Dictionary& choice);

    // Minimap geometry: request every frame (cheap); poll_map() is true when a new frame arrived,
    // get_map() then returns {revision, lod, lines: [{points: PackedVector2Array (session m), rank, flags, name, ref}]}.
    void request_map(double center_x, double center_y, double radius_m);
    bool poll_map();
    [[nodiscard]] godot::Dictionary get_map() const { return map_; }

    // Route: from (sx, sy) driving along (hx, hy) to (dx, dy), session metres. Returns the request serial.
    std::int64_t plan_route(double sx, double sy, double hx, double hy, double dx, double dy);
    void clear_route();
    // True when a planned route arrived; get_route() then returns {serial, status, message, length_m, duration_s,
    // polyline: PackedVector2Array, maneuvers: [{kind, distance_from_start_m, x, y, angle_deg, exit_number, name, ref,
    // destination}]} ({} when none).
    bool poll_route();
    [[nodiscard]] godot::Dictionary get_route() const { return route_dict_; }
    // rg::route_progress for the car at (x, y): {valid, along_m, off_route_m, off_route, remaining_m,
    // next: [{kind, distance_m, name, ref, exit_number, angle_deg}]}.
    godot::Dictionary get_progress(double x, double y);

protected:
    static void _bind_methods();

private:
    rg::RoutePlannerConfig config_;
    rg::LodChoice lod_choice_;
    std::unique_ptr<rg::RoutePlanner> planner_;
    std::unique_ptr<rg::MapView> map_view_;
    godot::Dictionary map_;
    rg::PlannedRoute route_;
    bool has_route_ = false;
    godot::Dictionary route_dict_;
    double along_hint_m_ = 0.0;
    godot::String last_error_;
};

} // namespace rg_godot
