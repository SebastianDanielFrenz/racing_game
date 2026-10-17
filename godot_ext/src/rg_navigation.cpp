// godot_ext/src/rg_navigation.cpp - see rg_navigation.h.
#include "rg_navigation.h"

#include "rg_simulation.h"

#include "rg/world_terrain.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>

namespace rg_godot {

using godot::D_METHOD;

namespace {
godot::String from_std(const std::string& s) { return godot::String::utf8(s.c_str()); }
} // namespace

RgNavigation::~RgNavigation() { release(); }

bool RgNavigation::initialize_shared(RgSimulation* sim, const godot::String& config_path, double center_x,
                                     double center_y) {
    release();
    if (sim == nullptr) {
        last_error_ = "RgNavigation: null RgSimulation";
        return false;
    }
    std::shared_ptr<rg::WorldTerrain> terrain = sim->shared_world_terrain();
    if (terrain == nullptr) {
        last_error_ = "RgNavigation: no terrain-mode Session";
        return false;
    }
    std::string err;
    const auto config = rg::load_route_planner_config(config_path.utf8().get_data(), &err);
    if (!config) {
        last_error_ = from_std(err);
        return false;
    }
    config_ = *config;
    const g2m::geo::SessionFrame& frame = terrain->frame();
    const double e0 = static_cast<double>(frame.e0_m());
    const double n0 = static_cast<double>(frame.n0_m());
    // The planner's region build asks at the lowest priority; the minimap at render priority.
    rg::RoadGraphTileSource region_source = [terrain](const g2m::TileKey& key) {
        return terrain->road_graph_tile(key, g2m::world::Priority::Warm);
    };
    rg::RoadGraphTileSource map_source = [terrain](const g2m::TileKey& key) { return terrain->road_graph_tile(key); };
    planner_ = std::make_unique<rg::RoutePlanner>(
        region_source, rg::tiles_in_square(frame.zone(), 2, e0, n0, center_x, center_y, config_.region_radius_m), e0, n0,
        config_);
    map_view_ = std::make_unique<rg::MapView>(map_source, frame.zone(), e0, n0, rg::map_config_for(lod_choice_),
                                              config_.map_cache_max_tiles);
    last_error_ = godot::String();
    return true;
}

void RgNavigation::release() {
    map_view_.reset(); // joins its worker (it holds the WorldTerrain)
    planner_.reset();
    map_ = godot::Dictionary();
    route_ = rg::PlannedRoute{};
    has_route_ = false;
    route_dict_ = godot::Dictionary();
    along_hint_m_ = 0.0;
}

godot::Dictionary RgNavigation::get_graph_status() const {
    godot::Dictionary d;
    if (!planner_) {
        d["state"] = "none";
        return d;
    }
    switch (planner_->graph_state()) {
        case rg::RoutePlanner::GraphState::Building: d["state"] = "building"; break;
        case rg::RoutePlanner::GraphState::Ready: d["state"] = "ready"; break;
        case rg::RoutePlanner::GraphState::Failed: d["state"] = "failed"; break;
    }
    const auto s = planner_->graph_stats();
    d["message"] = from_std(planner_->graph_message());
    d["tiles"] = static_cast<std::int64_t>(s.tiles);
    d["nodes"] = static_cast<std::int64_t>(s.nodes);
    d["arcs"] = static_cast<std::int64_t>(s.arcs);
    d["landmarks"] = static_cast<std::int64_t>(s.landmarks);
    d["build_ms"] = s.build_ms;
    return d;
}

void RgNavigation::set_view_lod(const godot::Dictionary& choice) {
    lod_choice_.map_detail = static_cast<double>(choice.get("map_detail", 1.0));
    if (map_view_) map_view_->set_config(rg::map_config_for(lod_choice_));
}

void RgNavigation::request_map(double center_x, double center_y, double radius_m) {
    if (map_view_) map_view_->request(center_x, center_y, radius_m);
}

bool RgNavigation::poll_map() {
    rg::MapViewFrame f;
    if (!map_view_ || !map_view_->poll(f)) return false;
    godot::Dictionary d;
    d["revision"] = static_cast<std::int64_t>(f.revision);
    d["lod"] = godot::String(g2m::route::to_string(f.lod));
    d["failed_tiles"] = static_cast<std::int64_t>(f.failed_tiles);
    godot::Array lines;
    for (const rg::MapViewLine& l : f.lines) {
        godot::PackedVector2Array pts;
        pts.resize(l.point_count);
        for (std::uint32_t k = 0; k < l.point_count; ++k) {
            const auto& p = f.points[l.point_first + k];
            pts.set(k, godot::Vector2(p[0], p[1]));
        }
        godot::Dictionary line;
        line["points"] = pts;
        line["rank"] = l.rank;
        line["flags"] = l.flags;
        line["name"] = from_std(l.name);
        line["ref"] = from_std(l.ref);
        lines.append(line);
    }
    d["lines"] = lines;
    map_ = d;
    return true;
}

std::int64_t RgNavigation::plan_route(double sx, double sy, double hx, double hy, double dx, double dy) {
    if (!planner_) return 0;
    return static_cast<std::int64_t>(planner_->request(sx, sy, hx, hy, dx, dy));
}

void RgNavigation::clear_route() {
    route_ = rg::PlannedRoute{};
    has_route_ = false;
    route_dict_ = godot::Dictionary();
    along_hint_m_ = 0.0;
}

bool RgNavigation::poll_route() {
    rg::PlannedRoute r;
    if (!planner_ || !planner_->poll(r)) return false;
    route_ = std::move(r);
    has_route_ = true;
    along_hint_m_ = 0.0;
    godot::Dictionary d;
    d["serial"] = static_cast<std::int64_t>(route_.serial);
    d["status"] = godot::String(rg::to_string(route_.status));
    d["message"] = from_std(route_.message);
    d["length_m"] = route_.length_m;
    d["duration_s"] = route_.duration_s;
    godot::PackedVector2Array pts;
    for (const auto& p : route_.polyline) pts.append(godot::Vector2(static_cast<float>(p[0]), static_cast<float>(p[1])));
    d["polyline"] = pts;
    godot::Array maneuvers;
    for (const rg::PlannedManeuver& m : route_.maneuvers) {
        godot::Dictionary md;
        md["kind"] = from_std(m.kind);
        md["distance_from_start_m"] = m.distance_from_start_m;
        md["x"] = m.x;
        md["y"] = m.y;
        md["angle_deg"] = m.angle_deg;
        md["exit_number"] = m.exit_number;
        md["name"] = from_std(m.name);
        md["ref"] = from_std(m.ref);
        md["destination"] = from_std(m.destination);
        maneuvers.append(md);
    }
    d["maneuvers"] = maneuvers;
    route_dict_ = d;
    return true;
}

godot::Dictionary RgNavigation::get_progress(double x, double y) {
    godot::Dictionary d;
    if (!has_route_) {
        d["valid"] = false;
        return d;
    }
    const rg::RouteProgress p = rg::route_progress(route_, x, y, along_hint_m_, config_.maneuvers_ahead, config_);
    if (p.valid && !p.off_route) along_hint_m_ = p.along_m;
    d["valid"] = p.valid;
    d["along_m"] = p.along_m;
    d["off_route_m"] = p.off_route_m;
    d["off_route"] = p.off_route;
    d["remaining_m"] = p.remaining_m;
    godot::Array next;
    for (const rg::UpcomingManeuver& u : p.next) {
        godot::Dictionary m;
        m["kind"] = from_std(u.maneuver.kind);
        m["distance_m"] = u.distance_m;
        m["name"] = from_std(u.maneuver.name);
        m["ref"] = from_std(u.maneuver.ref);
        m["exit_number"] = u.maneuver.exit_number;
        m["angle_deg"] = u.maneuver.angle_deg;
        next.append(m);
    }
    d["next"] = next;
    return d;
}

void RgNavigation::_bind_methods() {
    godot::ClassDB::bind_method(D_METHOD("initialize_shared", "sim", "config_path", "center_x", "center_y"),
                                &RgNavigation::initialize_shared);
    godot::ClassDB::bind_method(D_METHOD("release"), &RgNavigation::release);
    godot::ClassDB::bind_method(D_METHOD("is_active"), &RgNavigation::is_active);
    godot::ClassDB::bind_method(D_METHOD("get_last_error"), &RgNavigation::get_last_error);
    godot::ClassDB::bind_method(D_METHOD("get_graph_status"), &RgNavigation::get_graph_status);
    godot::ClassDB::bind_method(D_METHOD("get_minimap_radius_m"), &RgNavigation::get_minimap_radius_m);
    godot::ClassDB::bind_method(D_METHOD("set_view_lod", "choice"), &RgNavigation::set_view_lod);
    godot::ClassDB::bind_method(D_METHOD("request_map", "center_x", "center_y", "radius_m"), &RgNavigation::request_map);
    godot::ClassDB::bind_method(D_METHOD("poll_map"), &RgNavigation::poll_map);
    godot::ClassDB::bind_method(D_METHOD("get_map"), &RgNavigation::get_map);
    godot::ClassDB::bind_method(D_METHOD("plan_route", "sx", "sy", "hx", "hy", "dx", "dy"), &RgNavigation::plan_route);
    godot::ClassDB::bind_method(D_METHOD("clear_route"), &RgNavigation::clear_route);
    godot::ClassDB::bind_method(D_METHOD("poll_route"), &RgNavigation::poll_route);
    godot::ClassDB::bind_method(D_METHOD("get_route"), &RgNavigation::get_route);
    godot::ClassDB::bind_method(D_METHOD("get_progress", "x", "y"), &RgNavigation::get_progress);
}

} // namespace rg_godot
