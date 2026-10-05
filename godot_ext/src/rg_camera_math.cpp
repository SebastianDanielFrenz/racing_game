#include "rg_camera_math.h"

#include "rg/camera_math.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/vector3.hpp>

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

void RgCameraMath::_bind_methods() {
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("drive_view_names"), &RgCameraMath::drive_view_names);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("next_drive_view", "view"), &RgCameraMath::next_drive_view);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("toggle_cockpit_view", "view"), &RgCameraMath::toggle_cockpit_view);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("orbit_default_state"), &RgCameraMath::orbit_default_state);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("orbit_step", "state", "input", "dt"), &RgCameraMath::orbit_step);
    godot::ClassDB::bind_static_method("RgCameraMath", godot::D_METHOD("chase_follow_offset", "previous", "desired", "dt", "rate"), &RgCameraMath::chase_follow_offset);
}

} // namespace rg_godot
