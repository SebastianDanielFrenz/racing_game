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

    // ---- Shared rig helpers (chase, orbit rig, walker): ground probes, clearance, look-at validity.
    static godot::PackedVector3Array ground_probe_points(const godot::Vector3& position);
    // grounds: one entry per probe (a float height or null); raises position.y to ground + clearance_m.
    static godot::Vector3 apply_clearance(const godot::Vector3& position, const godot::Array& grounds, double clearance_m);
    static bool look_valid(const godot::Vector3& position, const godot::Vector3& target);

    // ---- Chase rig (rg::chase_*; state Dictionary handed back each frame, vectors Godot-frame).
    // params keys (optional, defaults = rg::ChaseParams): follow_distance_m, follow_height_m, look_height_m, side_offset_m,
    // terrain_clearance_m, position_lag, orbit_deadzone, orbit_lag, snap_m.
    static godot::Dictionary chase_initial_state();
    static godot::Dictionary chase_reset(const godot::Dictionary& state);
    // input: {target_position, forward, left (Vector3: the chassis basis x and y), key_look, stick (Vector2), live}.
    // Returns the new state's keys plus position (before the clearance) and look_at.
    static godot::Dictionary chase_follow_step(const godot::Dictionary& state, const godot::Dictionary& input, double dt, const godot::Dictionary& params);

    // ---- Free cam rig (rg::free_*). params: move_speed_mps, fast_multiplier, look_rate_rad_s, mouse_sensitivity, pitch_limit_rad.
    static godot::Dictionary free_initial_state();
    static godot::Dictionary free_activate(const godot::Dictionary& state, const godot::Vector3& position, const godot::Vector3& forward, const godot::Dictionary& params);
    static godot::Dictionary free_place(const godot::Dictionary& state, const godot::Vector3& position, double yaw, double pitch, const godot::Dictionary& params);
    static godot::Dictionary free_shift_origin(const godot::Dictionary& state, const godot::Vector3& delta);
    // input: {mouse (Vector2 px), look (Vector2 rate), move (Vector3: x right, y up, z forward), fast, live}.
    // Returns the state's keys plus position (Vector3); yaw_rad/pitch_rad are the rotation to apply.
    static godot::Dictionary free_step(const godot::Dictionary& state, const godot::Dictionary& input, double dt, const godot::Dictionary& params);

    // ---- Walker camera (rg::walker_cam_*). params = rg::WalkerCamParams keys (look_rate_rad_s, mouse_sensitivity,
    // third_person_distance_m, min_distance_m, max_distance_m, head_height_m, eye_height_m, terrain_clearance_m,
    // follow_rate, snap_m, third_pitch_min, third_pitch_max, first_pitch_limit).
    static godot::Dictionary walker_cam_initial_state(const godot::Dictionary& params);
    static godot::Dictionary walker_cam_activate(const godot::Dictionary& state, const godot::Vector3& from_forward, const godot::Dictionary& params);
    static godot::Dictionary walker_cam_shift_origin(const godot::Dictionary& state, const godot::Vector3& delta);
    static godot::Dictionary walker_cam_hold(const godot::Dictionary& state);
    static godot::Vector3 walker_cam_look_forward(const godot::Dictionary& state);
    // input: {has_walker, feet, car_position (Vector3), mouse, look (Vector2), wheel_steps, zoom_key, live, first_person}.
    // Returns the state's keys plus placed, third_person, position (before the clearance), pivot, yaw, pitch.
    static godot::Dictionary walker_cam_step(const godot::Dictionary& state, const godot::Dictionary& input, double dt, const godot::Dictionary& params);
    // {valid, yaw, pitch}: aim at the pivot after the clearance moved the camera.
    static godot::Dictionary walker_cam_aim(const godot::Vector3& pivot, const godot::Vector3& position, const godot::Dictionary& params);

    // ---- Orbit rig around orbit_step (rg::orbit_rig_*).
    static godot::Dictionary orbit_rig_initial_state();
    static godot::Dictionary orbit_rig_activate(const godot::Dictionary& state);
    static godot::Dictionary orbit_rig_reset(const godot::Dictionary& state);
    // input: {chassis_position, chassis_forward (Vector3), look, mouse (Vector2), zoom_steps, zoom_key, live}.
    // Returns the state's keys plus position (before the clearance) and target.
    static godot::Dictionary orbit_rig_step(const godot::Dictionary& state, const godot::Dictionary& input, double dt);

protected:
    static void _bind_methods();
};

} // namespace rg_godot
