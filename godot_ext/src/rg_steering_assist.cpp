#include "rg_steering_assist.h"

#include <godot_cpp/core/class_db.hpp>

#include <string>

namespace rg_godot {

namespace {
std::string to_std(const godot::String& s) { return std::string(s.utf8().get_data()); }
} // namespace

godot::String RgSteeringAssist::load_config(const godot::String& path) {
    std::string err;
    const auto config = rg::load_steering_assist_config(to_std(path), &err);
    if (!config) {
        rg::SteeringAssistConfig off;
        off.enabled = false; // a broken file never shapes the driver's input with guessed numbers
        assist_.set_config(off);
        return godot::String::utf8(err.c_str());
    }
    assist_.set_config(*config);
    return godot::String();
}

bool RgSteeringAssist::configure_vehicle(const godot::String& vehicle_json_path) {
    const auto geometry = rg::steering_geometry_from_vehicle_json(to_std(vehicle_json_path));
    if (!geometry) return false;
    assist_.set_geometry(*geometry);
    return true;
}

double RgSteeringAssist::translate(double raw, bool keyboard, bool wheel, const godot::Dictionary& motion, double delta) {
    rg::SteeringMotion m;
    m.forward_mps = static_cast<double>(motion.get("forward_mps", 0.0));
    m.left_mps = static_cast<double>(motion.get("left_mps", 0.0));
    m.yaw_rad_s = static_cast<double>(motion.get("yaw_rad_s", 0.0));
    m.upright = static_cast<double>(motion.get("upright", 1.0));
    m.relocations = static_cast<std::int64_t>(motion.get("relocations", 0));
    return assist_.translate(raw, keyboard, wheel, m, delta);
}

void RgSteeringAssist::_bind_methods() {
    using godot::D_METHOD;
    godot::ClassDB::bind_method(D_METHOD("load_config", "path"), &RgSteeringAssist::load_config);
    godot::ClassDB::bind_method(D_METHOD("configure_vehicle", "vehicle_json_path"), &RgSteeringAssist::configure_vehicle);
    godot::ClassDB::bind_method(D_METHOD("reset"), &RgSteeringAssist::reset);
    godot::ClassDB::bind_method(D_METHOD("translate", "raw", "keyboard", "wheel", "motion", "delta"),
                                &RgSteeringAssist::translate);
    godot::ClassDB::bind_method(D_METHOD("get_wheelbase_m"), &RgSteeringAssist::get_wheelbase_m);
    godot::ClassDB::bind_method(D_METHOD("get_max_wheel_angle_rad"), &RgSteeringAssist::get_max_wheel_angle_rad);
}

} // namespace rg_godot
