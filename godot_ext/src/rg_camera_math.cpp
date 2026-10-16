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

godot::Vector3 vec3_in(const Dictionary& d, const char* key) {
    return static_cast<godot::Vector3>(d.get(String(key), godot::Variant(godot::Vector3())));
}
godot::Vector2 vec2_in(const Dictionary& d, const char* key) {
    return static_cast<godot::Vector2>(d.get(String(key), godot::Variant(godot::Vector2())));
}
bool flag(const Dictionary& d, const char* key, bool fallback) {
    return static_cast<bool>(d.get(String(key), godot::Variant(fallback)));
}
void put_vec(Dictionary& d, const char* x, const char* y, const char* z, const ps::Vec3& v) {
    d[x] = v.x; d[y] = v.y; d[z] = v.z;
}
ps::Vec3 get_vec(const Dictionary& d, const char* x, const char* y, const char* z, const ps::Vec3& fallback) {
    return ps::Vec3{num(d, x, fallback.x), num(d, y, fallback.y), num(d, z, fallback.z)};
}

rg::ChaseParams chase_params_from(const Dictionary& d) {
    rg::ChaseParams p;
    p.follow_distance_m = num(d, "follow_distance_m", p.follow_distance_m);
    p.follow_height_m = num(d, "follow_height_m", p.follow_height_m);
    p.look_height_m = num(d, "look_height_m", p.look_height_m);
    p.side_offset_m = num(d, "side_offset_m", p.side_offset_m);
    p.terrain_clearance_m = num(d, "terrain_clearance_m", p.terrain_clearance_m);
    p.position_lag = num(d, "position_lag", p.position_lag);
    p.orbit_deadzone = num(d, "orbit_deadzone", p.orbit_deadzone);
    p.orbit_lag = num(d, "orbit_lag", p.orbit_lag);
    p.snap_m = num(d, "snap_m", p.snap_m);
    return p;
}
rg::ChaseState chase_state_from(const Dictionary& d) {
    rg::ChaseState s;
    s.offset = get_vec(d, "offset_x", "offset_y", "offset_z", s.offset);
    s.orbit_yaw_rad = num(d, "orbit_yaw_rad", 0.0);
    s.initialized = flag(d, "initialized", false);
    return s;
}
Dictionary chase_state_to(const rg::ChaseState& s) {
    Dictionary d;
    put_vec(d, "offset_x", "offset_y", "offset_z", s.offset);
    d["orbit_yaw_rad"] = s.orbit_yaw_rad;
    d["initialized"] = s.initialized;
    return d;
}

rg::FreeParams free_params_from(const Dictionary& d) {
    rg::FreeParams p;
    p.move_speed_mps = num(d, "move_speed_mps", p.move_speed_mps);
    p.fast_multiplier = num(d, "fast_multiplier", p.fast_multiplier);
    p.look_rate_rad_s = num(d, "look_rate_rad_s", p.look_rate_rad_s);
    p.mouse_sensitivity = num(d, "mouse_sensitivity", p.mouse_sensitivity);
    p.pitch_limit_rad = num(d, "pitch_limit_rad", p.pitch_limit_rad);
    return p;
}
rg::FreeState free_state_from(const Dictionary& d) {
    rg::FreeState s;
    s.position = get_vec(d, "pos_x", "pos_y", "pos_z", s.position);
    s.yaw_rad = num(d, "yaw_rad", 0.0);
    s.pitch_rad = num(d, "pitch_rad", 0.0);
    return s;
}
Dictionary free_state_to(const rg::FreeState& s) {
    Dictionary d;
    put_vec(d, "pos_x", "pos_y", "pos_z", s.position);
    d["yaw_rad"] = s.yaw_rad;
    d["pitch_rad"] = s.pitch_rad;
    d["position"] = to_godot(s.position);
    return d;
}

rg::WalkerCamParams walker_params_from(const Dictionary& d) {
    rg::WalkerCamParams p;
    p.look_rate_rad_s = num(d, "look_rate_rad_s", p.look_rate_rad_s);
    p.mouse_sensitivity = num(d, "mouse_sensitivity", p.mouse_sensitivity);
    p.third_person_distance_m = num(d, "third_person_distance_m", p.third_person_distance_m);
    p.min_distance_m = num(d, "min_distance_m", p.min_distance_m);
    p.max_distance_m = num(d, "max_distance_m", p.max_distance_m);
    p.head_height_m = num(d, "head_height_m", p.head_height_m);
    p.eye_height_m = num(d, "eye_height_m", p.eye_height_m);
    p.terrain_clearance_m = num(d, "terrain_clearance_m", p.terrain_clearance_m);
    p.follow_rate = num(d, "follow_rate", p.follow_rate);
    p.snap_m = num(d, "snap_m", p.snap_m);
    p.third_pitch_min = num(d, "third_pitch_min", p.third_pitch_min);
    p.third_pitch_max = num(d, "third_pitch_max", p.third_pitch_max);
    p.first_pitch_limit = num(d, "first_pitch_limit", p.first_pitch_limit);
    return p;
}
rg::WalkerCamState walker_state_from(const Dictionary& d) {
    rg::WalkerCamState s;
    s.yaw_rad = num(d, "yaw_rad", s.yaw_rad);
    s.pitch_rad = num(d, "pitch_rad", s.pitch_rad);
    s.distance_m = num(d, "distance_m", s.distance_m);
    s.smoothed_feet = get_vec(d, "feet_x", "feet_y", "feet_z", s.smoothed_feet);
    s.have_walker = flag(d, "have_walker", false);
    s.drain_wheel = flag(d, "drain_wheel", false);
    return s;
}
Dictionary walker_state_to(const rg::WalkerCamState& s) {
    Dictionary d;
    d["yaw_rad"] = s.yaw_rad;
    d["pitch_rad"] = s.pitch_rad;
    d["distance_m"] = s.distance_m;
    put_vec(d, "feet_x", "feet_y", "feet_z", s.smoothed_feet);
    d["have_walker"] = s.have_walker;
    d["drain_wheel"] = s.drain_wheel;
    return d;
}

rg::OrbitRigState orbit_rig_state_from(const Dictionary& d) {
    rg::OrbitRigState s = rg::orbit_rig_initial_state();
    s.orbit.azimuth_rad = num(d, "azimuth_rad", s.orbit.azimuth_rad);
    s.orbit.elevation_rad = num(d, "elevation_rad", s.orbit.elevation_rad);
    s.orbit.distance_m = num(d, "distance_m", s.orbit.distance_m);
    s.orbit.idle_s = num(d, "idle_s", s.orbit.idle_s);
    s.target_rel = get_vec(d, "target_rel_x", "target_rel_y", "target_rel_z", s.target_rel);
    s.initialized = flag(d, "initialized", false);
    return s;
}
Dictionary orbit_rig_state_to(const rg::OrbitRigState& s) {
    Dictionary d;
    d["azimuth_rad"] = s.orbit.azimuth_rad;
    d["elevation_rad"] = s.orbit.elevation_rad;
    d["distance_m"] = s.orbit.distance_m;
    d["idle_s"] = s.orbit.idle_s;
    put_vec(d, "target_rel_x", "target_rel_y", "target_rel_z", s.target_rel);
    d["initialized"] = s.initialized;
    return d;
}

std::vector<std::optional<double>> grounds_from(const godot::Array& grounds) {
    std::vector<std::optional<double>> g;
    for (int i = 0; i < grounds.size(); ++i) {
        const godot::Variant v = grounds[i];
        if (v.get_type() == godot::Variant::NIL) g.push_back(std::nullopt);
        else g.push_back(static_cast<double>(v));
    }
    return g;
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
godot::PackedVector3Array RgCameraMath::ground_probe_points(const godot::Vector3& position) {
    godot::PackedVector3Array out;
    for (const ps::Vec3& o : rg::ground_probe_offsets()) out.push_back(position + to_godot(o));
    return out;
}
godot::Vector3 RgCameraMath::apply_clearance(const godot::Vector3& position, const godot::Array& grounds, double clearance_m) {
    godot::Vector3 out = position;
    out.y = static_cast<float>(rg::clear_terrain_height(position.y, grounds_from(grounds), clearance_m));
    return out;
}
bool RgCameraMath::look_valid(const godot::Vector3& position, const godot::Vector3& target) {
    return rg::look_valid(to_core(position), to_core(target));
}

Dictionary RgCameraMath::chase_initial_state() { return chase_state_to(rg::ChaseState{}); }
Dictionary RgCameraMath::chase_reset(const Dictionary& state) {
    rg::ChaseState s = chase_state_from(state);
    rg::chase_reset(s);
    return chase_state_to(s);
}
Dictionary RgCameraMath::chase_follow_step(const Dictionary& state, const Dictionary& input, double dt, const Dictionary& params) {
    rg::ChaseState s = chase_state_from(state);
    rg::ChaseInput in;
    in.target_pos = to_core(vec3_in(input, "target_position"));
    in.forward = to_core(vec3_in(input, "forward"));
    in.left = to_core(vec3_in(input, "left"));
    const godot::Vector2 key = vec2_in(input, "key_look");
    const godot::Vector2 stick = vec2_in(input, "stick");
    in.key_x = key.x; in.key_y = key.y;
    in.stick_x = stick.x; in.stick_y = stick.y;
    in.live = flag(input, "live", true);
    const rg::ChaseFrame f = rg::chase_follow_step(s, in, dt, chase_params_from(params));
    Dictionary d = chase_state_to(s);
    d["position"] = to_godot(f.position);
    d["look_at"] = to_godot(f.look_at);
    return d;
}

Dictionary RgCameraMath::free_initial_state() { return free_state_to(rg::FreeState{}); }
Dictionary RgCameraMath::free_activate(const Dictionary& state, const godot::Vector3& position, const godot::Vector3& forward, const Dictionary& params) {
    rg::FreeState s = free_state_from(state);
    rg::free_activate(s, to_core(position), to_core(forward), free_params_from(params));
    return free_state_to(s);
}
Dictionary RgCameraMath::free_place(const Dictionary& state, const godot::Vector3& position, double yaw, double pitch, const Dictionary& params) {
    rg::FreeState s = free_state_from(state);
    rg::free_place(s, to_core(position), yaw, pitch, free_params_from(params));
    return free_state_to(s);
}
Dictionary RgCameraMath::free_shift_origin(const Dictionary& state, const godot::Vector3& delta) {
    rg::FreeState s = free_state_from(state);
    rg::free_shift_origin(s, to_core(delta));
    return free_state_to(s);
}
Dictionary RgCameraMath::free_step(const Dictionary& state, const Dictionary& input, double dt, const Dictionary& params) {
    rg::FreeState s = free_state_from(state);
    rg::FreeInput in;
    const godot::Vector2 mouse = vec2_in(input, "mouse");
    const godot::Vector2 look = vec2_in(input, "look");
    const godot::Vector3 move = vec3_in(input, "move");
    in.mouse_x = mouse.x; in.mouse_y = mouse.y;
    in.look_x = look.x; in.look_y = look.y;
    in.move_right = move.x; in.move_up = move.y; in.move_forward = move.z;
    in.fast = flag(input, "fast", false);
    in.live = flag(input, "live", true);
    rg::free_step(s, in, dt, free_params_from(params));
    return free_state_to(s);
}

Dictionary RgCameraMath::walker_cam_initial_state(const Dictionary& params) {
    return walker_state_to(rg::walker_cam_initial_state(walker_params_from(params)));
}
Dictionary RgCameraMath::walker_cam_activate(const Dictionary& state, const godot::Vector3& from_forward, const Dictionary& params) {
    rg::WalkerCamState s = walker_state_from(state);
    rg::walker_cam_activate(s, to_core(from_forward), walker_params_from(params));
    return walker_state_to(s);
}
Dictionary RgCameraMath::walker_cam_shift_origin(const Dictionary& state, const godot::Vector3& delta) {
    rg::WalkerCamState s = walker_state_from(state);
    rg::walker_cam_shift_origin(s, to_core(delta));
    return walker_state_to(s);
}
Dictionary RgCameraMath::walker_cam_hold(const Dictionary& state) {
    rg::WalkerCamState s = walker_state_from(state);
    rg::walker_cam_hold(s);
    return walker_state_to(s);
}
godot::Vector3 RgCameraMath::walker_cam_look_forward(const Dictionary& state) {
    return to_godot(rg::walker_cam_look_forward(walker_state_from(state)));
}
Dictionary RgCameraMath::walker_cam_step(const Dictionary& state, const Dictionary& input, double dt, const Dictionary& params) {
    rg::WalkerCamState s = walker_state_from(state);
    rg::WalkerCamInput in;
    in.has_walker = flag(input, "has_walker", false);
    in.feet = to_core(vec3_in(input, "feet"));
    in.car_position = to_core(vec3_in(input, "car_position"));
    const godot::Vector2 mouse = vec2_in(input, "mouse");
    const godot::Vector2 look = vec2_in(input, "look");
    in.mouse_x = mouse.x; in.mouse_y = mouse.y;
    in.look_x = look.x; in.look_y = look.y;
    in.wheel_steps = num(input, "wheel_steps", 0.0);
    in.zoom_key = num(input, "zoom_key", 0.0);
    in.live = flag(input, "live", true);
    in.first_person = flag(input, "first_person", false);
    const rg::WalkerCamFrame f = rg::walker_cam_step(s, in, dt, walker_params_from(params));
    Dictionary d = walker_state_to(s);
    d["placed"] = f.placed;
    d["third_person"] = f.third_person;
    d["position"] = to_godot(f.position);
    d["pivot"] = to_godot(f.pivot);
    d["yaw"] = f.yaw_rad;
    d["pitch"] = f.pitch_rad;
    return d;
}
Dictionary RgCameraMath::walker_cam_aim(const godot::Vector3& pivot, const godot::Vector3& position, const Dictionary& params) {
    const rg::WalkerCamAim a = rg::walker_cam_aim(to_core(pivot), to_core(position), walker_params_from(params));
    Dictionary d;
    d["valid"] = a.valid;
    d["yaw"] = a.yaw_rad;
    d["pitch"] = a.pitch_rad;
    return d;
}

Dictionary RgCameraMath::orbit_rig_initial_state() { return orbit_rig_state_to(rg::orbit_rig_initial_state()); }
Dictionary RgCameraMath::orbit_rig_activate(const Dictionary& state) {
    rg::OrbitRigState s = orbit_rig_state_from(state);
    rg::orbit_rig_activate(s);
    return orbit_rig_state_to(s);
}
Dictionary RgCameraMath::orbit_rig_reset(const Dictionary& state) {
    rg::OrbitRigState s = orbit_rig_state_from(state);
    rg::orbit_rig_reset(s);
    return orbit_rig_state_to(s);
}
Dictionary RgCameraMath::orbit_rig_step(const Dictionary& state, const Dictionary& input, double dt) {
    rg::OrbitRigState s = orbit_rig_state_from(state);
    rg::OrbitRigInput in;
    in.chassis_pos = to_core(vec3_in(input, "chassis_position"));
    in.forward = to_core(vec3_in(input, "chassis_forward"));
    const godot::Vector2 look = vec2_in(input, "look");
    const godot::Vector2 mouse = vec2_in(input, "mouse");
    in.look_x = look.x; in.look_y = look.y;
    in.mouse_x = mouse.x; in.mouse_y = mouse.y;
    in.zoom_steps = num(input, "zoom_steps", 0.0);
    in.zoom_key = num(input, "zoom_key", 0.0);
    in.live = flag(input, "live", true);
    const rg::OrbitRigFrame f = rg::orbit_rig_step(s, in, dt);
    Dictionary d = orbit_rig_state_to(s);
    d["position"] = to_godot(f.position);
    d["target"] = to_godot(f.target);
    return d;
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
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("ground_probe_points","position"), &RgCameraMath::ground_probe_points);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("apply_clearance","position","grounds","clearance_m"), &RgCameraMath::apply_clearance);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("look_valid","position","target"), &RgCameraMath::look_valid);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("chase_initial_state"), &RgCameraMath::chase_initial_state);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("chase_reset","state"), &RgCameraMath::chase_reset);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("chase_follow_step","state","input","dt","params"), &RgCameraMath::chase_follow_step);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("free_initial_state"), &RgCameraMath::free_initial_state);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("free_activate","state","position","forward","params"), &RgCameraMath::free_activate);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("free_place","state","position","yaw","pitch","params"), &RgCameraMath::free_place);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("free_shift_origin","state","delta"), &RgCameraMath::free_shift_origin);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("free_step","state","input","dt","params"), &RgCameraMath::free_step);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("walker_cam_initial_state","params"), &RgCameraMath::walker_cam_initial_state);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("walker_cam_activate","state","from_forward","params"), &RgCameraMath::walker_cam_activate);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("walker_cam_shift_origin","state","delta"), &RgCameraMath::walker_cam_shift_origin);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("walker_cam_hold","state"), &RgCameraMath::walker_cam_hold);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("walker_cam_look_forward","state"), &RgCameraMath::walker_cam_look_forward);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("walker_cam_step","state","input","dt","params"), &RgCameraMath::walker_cam_step);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("walker_cam_aim","pivot","position","params"), &RgCameraMath::walker_cam_aim);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("orbit_rig_initial_state"), &RgCameraMath::orbit_rig_initial_state);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("orbit_rig_activate","state"), &RgCameraMath::orbit_rig_activate);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("orbit_rig_reset","state"), &RgCameraMath::orbit_rig_reset);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("orbit_rig_step","state","input","dt"), &RgCameraMath::orbit_rig_step);
}

} // namespace rg_godot
