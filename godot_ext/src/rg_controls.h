// godot_ext/src/rg_controls.h - RgControls: the Godot-facing face of rg::Controls (PLAN.md R5b),
// the control configuration model. Every decision lives in rg_core (rg::Controls, rg::BindingCapture,
// rg::AxisCalibrator); this node only converts to and from Godot Variants so input_map.gd (devices ->
// action values) and shell_ui.gd / controls_ui.gd (the controls screen) can use it. It owns no scene
// node and reads no device itself: the GDScript side gathers a snapshot (pressed keys, mouse, pad
// buttons and axes) and passes it in.
//
// Snapshot dictionary (all keys optional):
//   keys: PackedStringArray (Godot key names of the pressed physical keys, "W", "Space", "PageUp"),
//   mouse_buttons: PackedInt32Array, mouse_dx, mouse_dy, wheel_up, wheel_down, wheel_left, wheel_right: float,
//   pads: Array of {slot: int, buttons: PackedInt32Array, axes: PackedFloat32Array}
#pragma once

#include "rg/control_capture.h"
#include "rg/controls.h"

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/string.hpp>

#include <string>
#include <unordered_map>

namespace rg_godot {

class RgControls : public godot::Node {
    GDCLASS(RgControls, godot::Node)

public:
    RgControls() = default;

    // data_dir: racing_game's own data/ (absolute); user_dir: the absolute user:// folder (controls.json lives
    // there). Loads the schema + defaults, then the player's file (a broken one gives defaults and a .bak).
    // {ok, error, path, file_missing, load_ok, message, dropped: PackedStringArray, backup_path}
    godot::Dictionary initialize(const godot::String& data_dir, const godot::String& user_dir);
    [[nodiscard]] bool is_ready() const { return controls_.ready(); }
    // Writes user://controls.json when something changed (or `force`). False on an I/O failure.
    bool save(bool force = false);
    [[nodiscard]] bool is_dirty() const { return controls_.dirty(); }
    [[nodiscard]] godot::String get_file_path() const;
    [[nodiscard]] godot::String get_load_message() const; // "" when the file loaded fine
    [[nodiscard]] godot::String to_json() const;
    // Test hook: replaces the in-memory state with a file's content (a second "instance" reading the same file).
    godot::Dictionary reload();

    // ---- devices ----
    godot::String joypad_connected(int slot, const godot::String& guid, const godot::String& name, int vendor, int product,
                                   bool standard_mapping);
    void joypad_disconnected(int slot);
    // [{key, class, name, label, guid, ordinal, connected, slot, customised, inherits}]
    [[nodiscard]] godot::Array get_devices() const;
    [[nodiscard]] godot::String get_device_key_for_slot(int slot) const;
    [[nodiscard]] int get_revision() const { return static_cast<int>(controls_.revision() & 0x7fffffff); }

    // ---- bindings (by device key) ----
    // The action list rows: [{action, sign, label, help, group, group_title, kind, range, overridden, conflict,
    //   bindings: [binding]}] - see binding_to_dict for a binding.
    [[nodiscard]] godot::Array get_rows(const godot::String& device_key) const;
    // [{action_a, label_a, action_b, label_b, binding_a, binding_b}]
    [[nodiscard]] godot::Array get_conflicts(const godot::String& device_key) const;
    godot::Dictionary bind_captured(const godot::String& device_key, const godot::String& action, int sign, bool replace);
    // A combined pedal axis from the captured axis: it becomes throttle and brake, opposite halves.
    godot::Dictionary bind_captured_combined_pedals(const godot::String& device_key);
    bool remove_binding(const godot::String& device_key, const godot::String& action, int index);
    bool clear_row(const godot::String& device_key, const godot::String& action, int sign);
    bool reset_action(const godot::String& device_key, const godot::String& action);
    bool reset_device(const godot::String& device_key);
    // Programmatic binding (tests, scripts): {type: "key"|"joy_button"|"joy_axis"|"mouse_button"|"mouse_motion", key, index, span,
    // sign, deadzone, saturation, curve, sensitivity, invert, calibrated, cal_min, cal_max}.
    godot::Dictionary bind_binding(const godot::String& device_key, const godot::String& action, int sign,
                                   const godot::Dictionary& binding, bool replace);
    // Tuning of an analogue binding: any of deadzone, saturation, curve, sensitivity, invert (the others untouched).
    godot::Dictionary set_binding_tuning(const godot::String& device_key, const godot::String& action, int index,
                                         const godot::Dictionary& tuning);
    [[nodiscard]] godot::Dictionary get_binding(const godot::String& device_key, const godot::String& action, int index) const;

    // ---- capture ("press the input you want to bind") ----
    // `snapshot` as above; for a pad the selected pad is the only entry of `pads`.
    godot::Dictionary capture_begin(const godot::String& device_key, const godot::String& action, int sign,
                                    const godot::Dictionary& snapshot);
    // "waiting" | "captured" | "cancelled"
    godot::String capture_feed(const godot::Dictionary& snapshot, double dt);
    [[nodiscard]] godot::Dictionary get_capture_result() const; // a binding dict
    [[nodiscard]] double get_capture_hold_progress() const;      // 0..1, B held toward "bind B"
    [[nodiscard]] godot::String get_capture_hint() const;        // what the prompt should ask for

    // ---- calibration (move the axis through its whole travel) ----
    void calibration_begin(double rest);
    void calibration_feed(double value);
    [[nodiscard]] godot::Dictionary get_calibration() const; // {valid, rest, low, high}
    bool calibration_apply(const godot::String& device_key, const godot::String& action, int index);

    // ---- evaluation (every frame) ----
    void evaluate(const godot::Dictionary& snapshot);
    [[nodiscard]] double get_value(const godot::String& action) const;
    [[nodiscard]] double get_raw(const godot::String& action) const;
    [[nodiscard]] double get_pulses(const godot::String& action) const;
    [[nodiscard]] double get_keyboard_value(const godot::String& action) const;
    [[nodiscard]] double get_pad_value(const godot::String& action) const;
    [[nodiscard]] double get_pad_raw(const godot::String& action) const;
    [[nodiscard]] int get_source(const godot::String& action) const; // DeviceClass as int, -1 = none
    [[nodiscard]] godot::PackedStringArray get_polled_keys() const;
    [[nodiscard]] bool is_mouse_button_bound(const godot::String& action, int button) const;
    // [{type: "key"|"joy_button", key, index}] for ui_accept / ui_cancel.
    [[nodiscard]] godot::Array get_menu_bindings(const godot::String& action) const;
    // The live monitor of one device: [{action, label, value, raw}] of every action the snapshot moves.
    [[nodiscard]] godot::Array monitor_device(const godot::String& device_key, const godot::Dictionary& snapshot) const;

protected:
    static void _bind_methods();

private:
    rg::InputSnapshot to_snapshot(const godot::Dictionary& d) const;
    godot::Dictionary binding_to_dict(const rg::Binding& b) const;
    rg::Binding dict_to_binding(const godot::Dictionary& d) const;
    int action_index(const godot::String& action) const;

    rg::Controls controls_;
    rg::BindingCapture capture_;
    rg::CaptureTarget capture_target_;
    rg::AxisCalibrator calibrator_;
    rg::ActionState state_;
    std::unordered_map<std::string, int> index_;
    std::string path_;
    std::string load_message_;
    bool reload_available_ = false;
};

} // namespace rg_godot
