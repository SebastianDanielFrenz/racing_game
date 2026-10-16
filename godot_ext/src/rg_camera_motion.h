// godot_ext/src/rg_camera_motion.h - RgCameraMotion: thin Godot binding of
// rg/camera_motion.h (the driver's virtual body and the speed effects of the
// drive cameras). Conversion only; the model and its numbers are rg_core's.
#pragma once

#include "rg/camera_motion.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>

namespace rg_godot {

class RgCameraMotion : public godot::RefCounted {
    GDCLASS(RgCameraMotion, godot::RefCounted)

public:
    // Loads data/controls/camera_motion.json; false (and a warning) on error.
    bool initialize(const godot::String& config_path);
    [[nodiscard]] bool is_initialized() const { return initialized_; }
    void reset() { motion_.reset(); }
    // sample = RgSimulation.get_camera_motion_sample(); delta = render frame time.
    void update(const godot::Dictionary& sample, double delta);
    // {offset (car frame, ISO x fwd / y left / z up, m), pitch_down_rad,
    //  roll_right_rad, fov_extra_deg} for one view ("chase", "bumper",
    // "cockpit", "xr_cockpit"); strength = the camera.body_motion setting.
    [[nodiscard]] godot::Dictionary get_view(const godot::String& view_name, double strength) const;
    // Diagnostics: {offset, pitch_down_rad, roll_right_rad, speed_mps, roughness_mps, accel_car}.
    [[nodiscard]] godot::Dictionary get_state() const;

protected:
    static void _bind_methods();

private:
    rg::CameraMotion motion_;
    bool initialized_ = false;
};

} // namespace rg_godot
