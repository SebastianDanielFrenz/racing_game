#pragma once
#include "rg/cockpit_hit.h"
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace rg_godot {
// Binding of rg::CockpitHit (mouse interaction with the mounted hyper cockpit controls). All positions and directions are
// in the car MODEL frame in ISO 8855 (x forward, y left, z up); the GDScript adapter converts from the world.
class RgCockpit : public godot::RefCounted {
    GDCLASS(RgCockpit, godot::RefCounted)
public:
    // Empty string on success, else the error text.
    godot::String load(const godot::String& layout_path, const godot::String& car_rig_path);
    int get_control_count() const;
    // Static facts of one control: id, part, wheel_mounted, click (mode), joint_node, has_joint, joint_axis (Vector3, ISO),
    // joint_revolute, joint_max, blank_when_unavailable, hit_half (Vector3, ISO), secondary (has a secondary binding).
    godot::Dictionary control_info(int index) const;
    // Live state: available, travel, pressed, hover.
    godot::Dictionary control_state(int index) const;
    int find(const godot::String& id) const;
    godot::Dictionary mouse_config() const;
    void set_wheel_angle(double rad);
    void set_enabled(bool enabled);
    bool is_enabled() const;
    int pick(const godot::Vector3& origin, const godot::Vector3& direction) const;
    void pointer_move(const godot::Vector3& origin, const godot::Vector3& direction);
    // button 0 = primary (left), 1 = secondary (right)
    int pointer_down(const godot::Vector3& origin, const godot::Vector3& direction, int button);
    void pointer_up(int button);
    int get_hover() const;
    // Returns the events produced: control, control_id, action_id, kind (RgDriverActions.Kind), value, source (COCKPIT_MOUSE), shift.
    godot::Array update(double dt, const godot::Dictionary& snapshot, const godot::Dictionary& channels);
    godot::Array held_actions() const;
    godot::Vector3 hit_centre_model(int index) const;
protected:
    static void _bind_methods();
private:
    rg::CockpitHit hit_;
};
}
