// rg/cockpit_hit.h - engine-neutral mouse interaction with the mounted cockpit controls (P3 flat-screen clicking).
//
// What lives here: the hit volumes of the mounted control parts in the MODEL frame (ISO 8855: x forward, y left, z up,
// the frame of the car's rig.json), ray picking against them, hover, the push travel of a pressed control, the
// thresholds that turn travel into exactly one event per crossing, the debounce, and the mapping from a control to the
// DriverActions event (or shift edge) it submits. No Godot type and no DriverActions instance: update() returns
// ActionEvents (source CockpitMouse) that the adapter submits to the ONE shared DriverActions, exactly like a keyboard
// or pad press does, so a click and a key give the same channel writes.
//
// Data (no literals here):
//  - data/cockpits/car_hyper_common.json  mounts (socket, part, fallback pose) + per-mount "control" (kind, availability
//    state, primary/secondary action) + the "mouse" block (pick margin, travel speed, eye distance).
//  - the parts rig (cockpit_parts.rig.json): hit boxes ("hit_<part>": pivot, half extents), joints ("ctl_<part>": axis,
//    limits, behaviour press_at_m / click_at_rad / apply_at_rad / release_at_m) and interaction_defaults
//    (press_fraction, release_fraction, debounce_s).
//  - the car rig: socket frames (position, normal, up) that override a mount's fallback pose, and the steering_wheel node
//    (pivot + joint axis) that wheel-mounted controls (horn, paddles) follow.
//
// A pressed control "travels": while the pointer is down (or a started press is still being committed) its travel
// fraction rises at mouse.travel_per_s, the press event fires when it crosses the rig's press threshold, and the control
// re-arms when it falls back below the release threshold. A click is therefore exactly one event, a held horn asserts its
// level from the press crossing until the release crossing, and a duplicate pointer-down without an up in between is
// ignored. Controls without momentary travel (the light dial, the stalks) act once on pointer-down.
//
// Moving parts are handled by the caller supplying the steering wheel angle (set_wheel_angle) and rays in the model
// frame; floating-origin rebases and seat movement never enter because picking is done per ray, in the model's own frame.
#pragma once

#include "rg/driver_actions.h"

#include "ps/math/mat3.h"
#include "ps/math/vec3.h"

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace rg {

enum class CockpitButton { Primary = 0, Secondary = 1 };
enum class CockpitControlMode { Push, Click }; // Push: travels to thresholds; Click: acts on pointer-down

// What a control submits. Exactly one of `id` (a DriverActions event) or `shift` (+1 up / -1 down, the shift edge counters
// that keyboard and pad feed through input_map) is meaningful.
struct CockpitBinding {
    std::string id;
    DriverActionKind kind = DriverActionKind::Press;
    double value = 1;
    int shift = 0;
    bool held = false; // a level asserted every frame while pressed (horn), released on the release crossing
    bool valid() const { return !id.empty() || shift != 0; }
};

struct CockpitMouseConfig {
    double max_eye_distance_m = 0; // clicks count only while the camera is this close to the driver eye
    double pick_margin_m = 0;      // each hit box is grown by this much for picking
    double travel_per_s = 0;       // push travel fraction per second (full travel in 1 / travel_per_s)
};

// One mounted control, in the frame of its parent (model root, or the steering wheel node).
struct CockpitControl {
    std::string id;   // mount id: "tc", "horn", "paddle_L"
    std::string part; // part tree: "tc", "drive_mode_sport"
    bool wheel_mounted = false;
    CockpitControlMode mode = CockpitControlMode::Push;
    ps::Vec3 mount_origin; // part frame origin in the parent frame
    ps::Mat3 mount_rot;    // columns = the part's X, Y, Z axes in the parent frame
    ps::Vec3 hit_centre;   // part frame
    ps::Vec3 hit_half;     // part frame, un-grown
    // Joint of the pushed node ("ctl_<part>"), part frame.
    bool has_joint = false;
    bool joint_revolute = false;
    std::string joint_node;
    ps::Vec3 joint_axis;
    double joint_max = 0;    // travel at fraction 1 (m or rad)
    double press_at = 1;     // travel fraction at which the press event fires
    double release_at = 0;   // travel fraction below which the control re-arms
    CockpitBinding primary, secondary;
    std::string state_id; // DriverActions::state id deciding availability ("" = always available)
    int mode_bit = -1;    // >= 0: also needs this bit of DriverSnapshot::drive_modes
    bool blank_when_unavailable = false; // the adapter shows the blank cap instead (an unoffered drive mode, the EPB)
};

// One result of update()/pointer_down(): submit `action` to the shared DriverActions (or count the shift edge).
struct CockpitEvent {
    int control = -1;
    std::string control_id;
    ActionEvent action; // source CockpitMouse; id empty for a shift edge
    int shift = 0;
};

class CockpitHit {
public:
    // Reads the layout, the parts rig it names and (optionally) the car rig. False + *error on any problem.
    bool load(const std::string& layout_path, const std::string& car_rig_path, std::string* error = nullptr);
    std::size_t control_count() const { return controls_.size(); }
    const CockpitControl& control(std::size_t index) const { return controls_[index]; }
    int find(std::string_view id) const; // control index by mount id, -1 if none
    const CockpitMouseConfig& mouse_config() const { return config_; }
    double debounce_s() const { return debounce_s_; }

    // Steering wheel joint value (rad, rig convention), rotating the wheel-mounted controls about the rig's wheel axis.
    void set_wheel_angle(double rad) { wheel_angle_ = rad; }
    double wheel_angle() const { return wheel_angle_; }
    // Pose of a control's part frame in the model frame at the current wheel angle.
    void part_pose(std::size_t index, ps::Vec3& origin, ps::Mat3& rot) const;
    ps::Vec3 hit_centre_model(std::size_t index) const;

    // The control a ray (model frame) hits first, -1 for a miss. A smaller box wins over a larger one it lies inside.
    int pick(const ps::Vec3& origin, const ps::Vec3& direction) const;

    // Disabled (a menu, a text field, the cursor captured for looking): no hover, pointer input ignored, every pressed
    // control is released.
    void set_enabled(bool enabled);
    bool enabled() const { return enabled_; }

    void pointer_move(const ps::Vec3& origin, const ps::Vec3& direction);
    // Returns the control the pointer went down on (-1 miss, also -1 while disabled). An unavailable control is hit but
    // inert: nothing is submitted. A second pointer-down without pointer_up for the same button is ignored.
    int pointer_down(const ps::Vec3& origin, const ps::Vec3& direction, CockpitButton button = CockpitButton::Primary);
    void pointer_up(CockpitButton button = CockpitButton::Primary);
    int hover() const { return hover_; }

    // Availability from the shared projection, push travel, threshold crossings and events. Call once per frame.
    void update(double dt, const DriverSnapshot& snapshot, const ChannelSink& channels);
    std::vector<CockpitEvent> take_events();
    // Levels that must be re-asserted every frame before DriverActions::apply (the horn while pressed).
    std::vector<ActionEvent> held_actions() const;

    bool available(std::size_t index) const { return index < available_.size() && available_[index]; }
    double travel(std::size_t index) const { return index < state_.size() ? state_[index].travel : 0; }
    bool pressed(std::size_t index) const { return index < state_.size() && state_[index].pressed; }
    int debounced_count() const { return debounced_; }

private:
    struct State {
        double travel = 0;
        bool pointer = false;   // the button that started the press is still down
        bool committed = false; // a started press runs to its threshold even after the pointer is released
        bool armed = true;
        bool pressed = false;   // past the press crossing, before the release crossing
        bool fired = false;     // the press crossing produced an event (not debounced, not inert)
        CockpitButton button = CockpitButton::Primary;
        double last_fire_s = -1e9;
    };
    bool fire(std::size_t index, const CockpitBinding& binding, bool release);
    void release_all();
    bool ray_box(std::size_t index, const ps::Vec3& o, const ps::Vec3& d, double& t, double& volume) const;

    std::vector<CockpitControl> controls_;
    std::vector<State> state_;
    std::vector<bool> available_;
    std::vector<CockpitEvent> events_;
    CockpitMouseConfig config_;
    double debounce_s_ = 0;
    ps::Vec3 wheel_pivot_;
    ps::Vec3 wheel_axis_{1, 0, 0};
    double wheel_angle_ = 0;
    double clock_s_ = 0;
    int hover_ = -1;
    bool enabled_ = true;
    int debounced_ = 0;
};

} // namespace rg
