// godot_ext/src/rg_steering_assist.h - RgSteeringAssist: the thin binding of rg/steering_assist.h (adaptive steering
// input translation). game/scripts/adaptive_steering.gd keeps its old method names and forwards here.
#pragma once

#include "rg/steering_assist.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>

namespace rg_godot {

class RgSteeringAssist : public godot::RefCounted {
    GDCLASS(RgSteeringAssist, godot::RefCounted)

public:
    // Loads data/controls/steering.json; "" on success, else the loader message (the assist then passes input through).
    godot::String load_config(const godot::String& path);
    // Reads the car's wheelbase and full-lock wheel angle from its vehicle file; false when the file is unreadable.
    bool configure_vehicle(const godot::String& vehicle_json_path);
    void reset() { assist_.reset(); }
    // motion: RgSimulation.get_steering_kinematics() {forward_mps, left_mps, yaw_rad_s, upright, relocations}.
    double translate(double raw, bool keyboard, bool wheel, const godot::Dictionary& motion, double delta);
    [[nodiscard]] double get_wheelbase_m() const { return assist_.geometry().wheelbase_m; }
    [[nodiscard]] double get_max_wheel_angle_rad() const { return assist_.geometry().max_wheel_angle_rad; }

protected:
    static void _bind_methods();

private:
    rg::SteeringAssist assist_;
};

} // namespace rg_godot
