#include "rg_camera_math.h"

#include "rg/camera_math.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <optional>
#include <vector>

namespace rg_godot {

using godot::Dictionary;
using godot::String;

namespace {

rg::DriveView view_from(const String& name) {
    const godot::CharString utf8 = name.utf8();
    const auto parsed = rg::drive_view_from_string(std::string(utf8.get_data()));
    return parsed.value_or(rg::DriveView::Chase);
}

double num(const Dictionary& d, const char* key, double fallback) {
    const godot::Variant v = d.get(String(key), godot::Variant(fallback));
    return static_cast<double>(v);
}

Dictionary state_to_dict(const rg::OrbitState& s) {
    Dictionary d;
    d["azimuth_rad"] = s.azimuth_rad;
    d["elevation_rad"] = s.elevation_rad;
    d["distance_m"] = s.distance_m;
    d["idle_s"] = s.idle_s;
    return d;
}

// ---- Drone follow helpers: Godot (x, y, z) <-> rg_core Z-up (x, -z, y), a proper rotation.
ps::Vec3 to_core(const godot::Vector3& v) { return ps::Vec3{v.x, -v.z, v.y}; }
godot::Vector3 to_godot(const ps::Vec3& v) {
    return godot::Vector3(static_cast<float>(v.x), static_cast<float>(v.z), static_cast<float>(-v.y));
}

rg::DroneParams drone_params_from(const Dictionary& d) {
    rg::DroneParams p;
    p.follow_distance_m = num(d, "follow_distance_m", p.follow_distance_m);
    p.height_per_distance = num(d, "height_per_distance", p.height_per_distance);
    p.look_height_m = num(d, "look_height_m", p.look_height_m);
    p.terrain_clearance_m = num(d, "terrain_clearance_m", p.terrain_clearance_m);
    p.position_lag = num(d, "position_lag", p.position_lag);
    p.orbit_deadzone = num(d, "orbit_deadzone", p.orbit_deadzone);
    p.orbit_lag = num(d, "orbit_lag", p.orbit_lag);
    p.zoom_wheel_factor = num(d, "zoom_wheel_factor", p.zoom_wheel_factor);
    p.zoom_key_rate = num(d, "zoom_key_rate", p.zoom_key_rate);
    return p;
}

// State keys are plain doubles/bools (no float32 round trip through a Vector3 per frame).
rg::DroneState drone_state_from(const Dictionary& d) {
    rg::DroneState s;
    s.distance_m = num(d, "distance_m", s.distance_m);
    s.orbit_yaw_rad = num(d, "orbit_yaw_rad", 0.0);
    s.offset = ps::Vec3{num(d, "offset_x", 0.0), num(d, "offset_y", 0.0), num(d, "offset_z", 0.0)};
    s.smoothed_pos = ps::Vec3{num(d, "pos_x", 0.0), num(d, "pos_y", 0.0), num(d, "pos_z", 0.0)};
    s.heading_x = num(d, "heading_x", s.heading_x);
    s.heading_y = num(d, "heading_y", s.heading_y);
    s.from_absolute = static_cast<bool>(d.get(String("from_absolute"), godot::Variant(false)));
    s.initialized = static_cast<bool>(d.get(String("initialized"), godot::Variant(false)));
    s.drain_wheel = static_cast<bool>(d.get(String("drain_wheel"), godot::Variant(false)));
    return s;
}

Dictionary drone_state_to(const rg::DroneState& s) {
    Dictionary d;
    d["distance_m"] = s.distance_m;
    d["orbit_yaw_rad"] = s.orbit_yaw_rad;
    d["offset_x"] = s.offset.x; d["offset_y"] = s.offset.y; d["offset_z"] = s.offset.z;
    d["pos_x"] = s.smoothed_pos.x; d["pos_y"] = s.smoothed_pos.y; d["pos_z"] = s.smoothed_pos.z;
    d["heading_x"] = s.heading_x;
    d["heading_y"] = s.heading_y;
    d["from_absolute"] = s.from_absolute;
    d["initialized"] = s.initialized;
    d["drain_wheel"] = s.drain_wheel;
    return d;
}

} // namespace

godot::PackedStringArray RgCameraMath::drive_view_names() {
    godot::PackedStringArray names;
    rg::DriveView v = rg::DriveView::Chase;
    for (int i = 0; i < rg::kDriveViewCount; ++i) {
        names.push_back(String(rg::to_string(v)));
        v = rg::next_drive_view(v);
    }
    return names;
}

String RgCameraMath::next_drive_view(const String& view) {
    return String(rg::to_string(rg::next_drive_view(view_from(view))));
}

String RgCameraMath::toggle_cockpit_view(const String& view) {
    return String(rg::to_string(rg::toggle_cockpit_view(view_from(view))));
}

Dictionary RgCameraMath::orbit_default_state() {
    const rg::OrbitParams p;
    rg::OrbitState s;
    s.elevation_rad = p.default_elevation_rad;
    s.distance_m = p.default_distance_m;
    return state_to_dict(s);
}

Dictionary RgCameraMath::orbit_step(const Dictionary& state, const Dictionary& input, double dt) {
    const rg::OrbitParams p;
    rg::OrbitState s;
    s.azimuth_rad = num(state, "azimuth_rad", 0.0);
    s.elevation_rad = num(state, "elevation_rad", p.default_elevation_rad);
    s.distance_m = num(state, "distance_m", p.default_distance_m);
    s.idle_s = num(state, "idle_s", 0.0);
    rg::OrbitInput in;
    in.look_x = num(input, "look_x", 0.0);
    in.look_y = num(input, "look_y", 0.0);
    in.zoom_steps = num(input, "zoom_steps", 0.0);
    in.zoom_key = num(input, "zoom_key", 0.0);
    in.live = static_cast<bool>(input.get(String("live"), godot::Variant(true)));
    const rg::OrbitState next = rg::orbit_step(s, in, dt, p);
    Dictionary d = state_to_dict(next);
    const ps::Vec3 o = rg::orbit_offset(next);
    d["offset"] = godot::Vector3(static_cast<float>(o.x), static_cast<float>(o.y), static_cast<float>(o.z));
    return d;
}

godot::Vector3 RgCameraMath::chase_follow_offset(const godot::Vector3& previous, const godot::Vector3& desired, double dt, double rate) {
    const ps::Vec3 o = rg::chase_follow_offset(ps::Vec3{previous.x, previous.y, previous.z}, ps::Vec3{desired.x, desired.y, desired.z}, dt, rate);
    return godot::Vector3(static_cast<float>(o.x), static_cast<float>(o.y), static_cast<float>(o.z));
}

Dictionary RgCameraMath::cockpit_look_step(const Dictionary& state,const Dictionary& input,double dt,const Dictionary& config) {
    const rg::CockpitLook s{num(state,"yaw_deg",0),num(state,"pitch_deg",0)};
    const rg::CockpitLookInput in{num(input,"mouse_x",0),num(input,"mouse_y",0),num(input,"look_x",0),num(input,"look_y",0),static_cast<bool>(input.get("live",true))};
    const rg::CockpitLookConfig c{num(config,"cockpit_pitch_min_deg",0),num(config,"cockpit_pitch_max_deg",0),num(config,"cockpit_mouse_deg_per_pixel",0),num(config,"cockpit_look_rate_deg_s",0),num(config,"cockpit_look_deadzone",0)};
    const auto next=rg::cockpit_look_step(s,in,dt,c);
    Dictionary out;out["yaw_deg"]=next.yaw_deg;out["pitch_deg"]=next.pitch_deg;return out;
}
double RgCameraMath::steering_wheel_angle_rad(double steer,double lock_deg) { return rg::steering_wheel_angle_rad(steer,lock_deg); }

Dictionary RgCameraMath::drone_initial_state(const Dictionary& params) {
    return drone_state_to(rg::drone_initial_state(drone_params_from(params)));
}
Dictionary RgCameraMath::drone_activate(const Dictionary& state, const godot::Vector3& from_position) {
    rg::DroneState s = drone_state_from(state);
    rg::drone_activate(s, to_core(from_position));
    return drone_state_to(s);
}
Dictionary RgCameraMath::drone_shift_origin(const Dictionary& state, const godot::Vector3& delta) {
    rg::DroneState s = drone_state_from(state);
    rg::drone_shift_origin(s, to_core(delta));
    return drone_state_to(s);
}
Dictionary RgCameraMath::drone_hold(const Dictionary& state) {
    rg::DroneState s = drone_state_from(state);
    rg::drone_hold(s);
    return drone_state_to(s);
}
Dictionary RgCameraMath::drone_zoom_step(const Dictionary& state, double wheel_steps, double zoom_key, bool live, double dt, const Dictionary& params) {
    rg::DroneState s = drone_state_from(state);
    rg::drone_zoom_step(s, wheel_steps, zoom_key, live, dt, drone_params_from(params));
    return drone_state_to(s);
}
Dictionary RgCameraMath::drone_follow_step(const Dictionary& state, const Dictionary& input, double dt, const Dictionary& params) {
    rg::DroneState s = drone_state_from(state);
    rg::DroneFollowInput in;
    in.target_pos = to_core(static_cast<godot::Vector3>(input.get(String("target_position"), godot::Variant(godot::Vector3()))));
    in.target_forward = to_core(static_cast<godot::Vector3>(input.get(String("target_forward"), godot::Variant(godot::Vector3()))));
    const godot::Vector2 key = input.get(String("key_look"), godot::Variant(godot::Vector2()));
    const godot::Vector2 stick = input.get(String("stick"), godot::Variant(godot::Vector2()));
    in.key_x = key.x; in.key_y = key.y;
    in.stick_x = stick.x; in.stick_y = stick.y;
    in.live = static_cast<bool>(input.get(String("live"), godot::Variant(true)));
    const rg::DroneFrame f = rg::drone_follow_step(s, in, dt, drone_params_from(params));
    Dictionary d = drone_state_to(s);
    d["position"] = to_godot(f.position);
    d["look_at"] = to_godot(f.look_at);
    d["has_look"] = f.has_look;
    return d;
}
godot::PackedVector3Array RgCameraMath::drone_probe_points(const godot::Vector3& position) {
    godot::PackedVector3Array out;
    for (const ps::Vec3& o : rg::drone_probe_offsets()) out.push_back(position + to_godot(o));
    return out;
}
godot::Vector3 RgCameraMath::drone_apply_clearance(const godot::Vector3& position, const godot::Array& grounds, const Dictionary& params) {
    std::vector<std::optional<double>> g;
    for (int i = 0; i < grounds.size(); ++i) {
        const godot::Variant v = grounds[i];
        if (v.get_type() == godot::Variant::NIL) g.push_back(std::nullopt);
        else g.push_back(static_cast<double>(v));
    }
    godot::Vector3 out = position;
    out.y = static_cast<float>(rg::drone_clear_terrain(position.y, g, drone_params_from(params).terrain_clearance_m));
    return out;
}
void RgCameraMath::_bind_methods() {
    godot::ClassDB::bind_static_method("RgCameraMath",godot::D_METHOD("cockpit_look_step","state","input","dt","config"),&RgCameraMath::cockpit_look_step);
    godot::ClassDB::bind_static_method("RgCameraMath",godot::D_METHOD("steering_wheel_angle_rad","steer","lock_deg"),&RgCameraMath::steering_wheel_angle_rad);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("drive_view_names"), &RgCameraMath::drive_view_names);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("next_drive_view", "view"), &RgCameraMath::next_drive_view);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("toggle_cockpit_view", "view"), &RgCameraMath::toggle_cockpit_view);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("orbit_default_state"), &RgCameraMath::orbit_default_state);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("orbit_step", "state", "input", "dt"), &RgCameraMath::orbit_step);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("chase_follow_offset", "previous", "desired", "dt", "rate"), &RgCameraMath::chase_follow_offset);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("drone_initial_state", "params"), &RgCameraMath::drone_initial_state);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("drone_activate", "state", "from_position"), &RgCameraMath::drone_activate);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("drone_shift_origin", "state", "delta"), &RgCameraMath::drone_shift_origin);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("drone_hold", "state"), &RgCameraMath::drone_hold);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("drone_zoom_step", "state", "wheel_steps", "zoom_key", "live", "dt", "params"), &RgCameraMath::drone_zoom_step);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("drone_follow_step", "state", "input", "dt", "params"), &RgCameraMath::drone_follow_step);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("drone_probe_points", "position"), &RgCameraMath::drone_probe_points);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("drone_apply_clearance", "position", "grounds", "params"), &RgCameraMath::drone_apply_clearance);
}

} // namespace rg_godot
