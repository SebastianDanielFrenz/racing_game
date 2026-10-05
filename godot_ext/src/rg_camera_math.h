// godot_ext/src/rg_camera_math.h - RgCameraMath: static, stateless bindings of
// rg/camera_math.h's drive-view cycle and orbit control (PLAN.md R5). The
// GDScript rigs keep their own state in plain Dictionaries and hand it back
// each frame; every number is computed in rg_core.
#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/string.hpp>

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

protected:
    static void _bind_methods();
};

} // namespace rg_godot
