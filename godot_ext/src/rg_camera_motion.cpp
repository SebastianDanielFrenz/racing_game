#include "rg_camera_motion.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <string>

namespace rg_godot {

using godot::Dictionary;
using godot::String;
using godot::Vector3;

namespace {

ps::Vec3 to_ps(const godot::Variant& v) {
    if (v.get_type() != godot::Variant::VECTOR3) return ps::Vec3{};
    const Vector3 g = v;
    return ps::Vec3{g.x, g.y, g.z};
}

Vector3 to_godot(const ps::Vec3& v) {
    return Vector3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

} // namespace

bool RgCameraMotion::initialize(const String& config_path) {
    std::string err;
    const auto c = rg::load_camera_motion_config(std::string(config_path.utf8().get_data()), &err);
    if (!c) {
        godot::UtilityFunctions::push_warning(String("RgCameraMotion: ") + String(err.c_str()));
        initialized_ = false;
        return false;
    }
    motion_ = rg::CameraMotion(*c);
    initialized_ = true;
    return true;
}

void RgCameraMotion::update(const Dictionary& sample, double delta) {
    if (!initialized_ || sample.is_empty()) return;
    rg::CarMotionSample s;
    s.sim_time = sample.get("sim_time", 0.0);
    s.velocity = to_ps(sample.get("velocity", Vector3()));
    s.forward = to_ps(sample.get("forward", Vector3(1, 0, 0)));
    s.left = to_ps(sample.get("left", Vector3(0, 1, 0)));
    s.up = to_ps(sample.get("up", Vector3(0, 0, 1)));
    const godot::Variant c = sample.get("compression", godot::PackedFloat64Array());
    if (c.get_type() == godot::Variant::PACKED_FLOAT64_ARRAY) {
        const godot::PackedFloat64Array a = c;
        for (int i = 0; i < a.size(); ++i) s.wheel_compression.push_back(a[i]);
    }
    motion_.update(s, delta);
}

Dictionary RgCameraMotion::get_view(const String& view_name, double strength) const {
    const rg::ViewMotion m = motion_.view(std::string(view_name.utf8().get_data()), strength);
    Dictionary d;
    d["offset"] = to_godot(m.offset);
    d["pitch_down_rad"] = m.pitch_down_rad;
    d["roll_right_rad"] = m.roll_right_rad;
    d["fov_extra_deg"] = m.fov_extra_deg;
    return d;
}

Dictionary RgCameraMotion::get_state() const {
    const rg::HeadState& h = motion_.state();
    Dictionary d;
    d["offset"] = to_godot(h.offset);
    d["pitch_down_rad"] = h.pitch_down_rad;
    d["roll_right_rad"] = h.roll_right_rad;
    d["speed_mps"] = h.speed_mps;
    d["roughness_mps"] = h.roughness_mps;
    d["accel_car"] = to_godot(h.accel_car);
    return d;
}

void RgCameraMotion::_bind_methods() {
    godot::ClassDB::bind_method(godot::D_METHOD("initialize", "config_path"), &RgCameraMotion::initialize);
    godot::ClassDB::bind_method(godot::D_METHOD("is_initialized"), &RgCameraMotion::is_initialized);
    godot::ClassDB::bind_method(godot::D_METHOD("reset"), &RgCameraMotion::reset);
    godot::ClassDB::bind_method(godot::D_METHOD("update", "sample", "delta"), &RgCameraMotion::update);
    godot::ClassDB::bind_method(godot::D_METHOD("get_view", "view_name", "strength"), &RgCameraMotion::get_view);
    godot::ClassDB::bind_method(godot::D_METHOD("get_state"), &RgCameraMotion::get_state);
}

} // namespace rg_godot
