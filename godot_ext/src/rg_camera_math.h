// godot_ext/src/rg_camera_math.h - RgCameraMath: static, stateless bindings of
// rg/camera_math.h's drive-view cycle and orbit control (PLAN.md R5). The
// GDScript rigs keep their own state in plain Dictionaries and hand it back
// each frame; every number is computed in rg_core.
#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace rg_godot {

class RgCameraMath : public godot::RefCounted {
    GDCLASS(RgCameraMath, godot::RefCounted)

public:
    // ["chase", "bumper", "cockpit", "orbit", "cinematic"] (cycling order).
    static godot::PackedStringArray drive_view_names();
    // The next view in the cycle; an unknown name returns "chase".
    static godot::String next_drive_view(const godot::String& view);
    // Tab: cockpit -> chase, anything else -> cockpit.
    static godot::String toggle_cockpit_view(const godot::String& view);
    // {azimuth_rad, elevation_rad, distance_m, idle_s}
    static godot::Dictionary orbit_default_state();
    // state: orbit_default_state()'s keys; input: {look_x, look_y, zoom_steps,
    // zoom_key, live}. Returns the new state's keys plus "offset": a Vector3 in
    // the ISO heading frame (x forward, y left, z up) from the look-at point.
    static godot::Dictionary orbit_step(const godot::Dictionary& state, const godot::Dictionary& input, double dt);
    // The chase rig's follow smoothing (rg::chase_follow_offset): the camera's smoothed offset from the car, easing
    // toward `desired` with 1 - exp(-rate * dt). Any frame, isotropic - the rig passes Godot-space offsets.
    static godot::Vector3 chase_follow_offset(const godot::Vector3& previous, const godot::Vector3& desired, double dt, double rate);

    static godot::Dictionary cockpit_look_step(const godot::Dictionary& state, const godot::Dictionary& input, double dt, const godot::Dictionary& config);
    static double steering_wheel_angle_rad(double steer,double lock_deg);

    // ---- Drone follow (rg::drone_*; the drone_rig.gd state lives in a Dictionary the rig hands back each frame).
    // All vectors are Godot-frame (Y up); the binding maps them to rg_core's Z-up frame (x, -z, y) and back.
    // params keys (all optional, defaults = rg::DroneParams): follow_distance_m, height_per_distance, look_height_m,
    // terrain_clearance_m, position_lag, orbit_deadzone, orbit_lag, zoom_wheel_factor, zoom_key_rate.
    static godot::Dictionary drone_initial_state(const godot::Dictionary& params);
    static godot::Dictionary drone_activate(const godot::Dictionary& state, const godot::Vector3& from_position);
    static godot::Dictionary drone_shift_origin(const godot::Dictionary& state, const godot::Vector3& delta);
    static godot::Dictionary drone_hold(const godot::Dictionary& state);
    // wheel_steps, zoom_key, live -> the new state.
    static godot::Dictionary drone_zoom_step(const godot::Dictionary& state, double wheel_steps, double zoom_key, bool live, double dt, const godot::Dictionary& params);
    // input: {target_position, target_forward (Vector3), key_look (Vector2), stick (Vector2), live}. Returns the new state's
    // keys plus position, look_at (Vector3) and has_look (bool); position is before the terrain clearance.
    static godot::Dictionary drone_follow_step(const godot::Dictionary& state, const godot::Dictionary& input, double dt, const godot::Dictionary& params);
    // The ground-probe positions around `position` (5 Vector3) and the camera height after the clearance (grounds: one
    // entry per probe, a float height or null).
    static godot::PackedVector3Array drone_probe_points(const godot::Vector3& position);
    static godot::Vector3 drone_apply_clearance(const godot::Vector3& position, const godot::Array& grounds, const godot::Dictionary& params);

protected:
    static void _bind_methods();
};

} // namespace rg_godot
