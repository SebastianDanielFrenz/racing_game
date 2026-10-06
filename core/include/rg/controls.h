// rg/controls.h - rg::Controls: the game's input model (PLAN.md R5b). One action
// schema (control_schema.h), per-device binding profiles, the device registry
// with hot-plug, conflict detection, persistence (user://controls.json,
// "rg.controls/1") and the evaluation of a raw input snapshot into action
// values. Engine-neutral: the Godot layer (input_map.gd + RgControls) only
// gathers raw device state into an InputSnapshot, forwards device events and
// draws; it decides nothing about what an input means.
//
// Device identity (owner decision 2026-10-06, see PLAN.md R5b):
//   - the keyboard and the mouse each have one profile ("keyboard", "mouse");
//   - a joypad is keyed by its SDL GUID (bus / vendor / product / version - the same for two
//     units of one model; there is no per-unit serial) plus an ordinal among the devices of
//     that GUID: key "joy:<guid>#<n>". The ordinal is the lowest one not held by a currently
//     connected device of the same GUID, so a lone pad is always #1 and a reconnected pad gets
//     its profile back; two identical pads are #1 and #2 in connection order.
//   - profile resolution: the device's own profile -> the GUID's #1 profile (a second identical
//     pad starts from the first one's setup; its own profile is created, copying that setup,
//     on the first edit) -> the built-in default of the device class. A pad whose SDL mapping
//     is unknown (class Generic) has an empty default: it does nothing until the player binds it.
//   - a device that is unplugged keeps its profile and is listed as "not connected".
//   - several devices act at once: each profile binds actions on its own device only and the
//     larger magnitude wins per action.
//
// A profile stores only its OVERRIDES of the class default (action -> bindings; an empty list
// is a cleared action), so "reset action" erases one entry, "reset device" all of them, and a
// default that changes with a new build reaches every action the player never touched.
//
// File policy (load_text/load_file): never fails the game. A missing file -> defaults; a syntax
// error, a wrong structure or an unknown "rg.controls/N" version -> defaults, reported, and
// load_file keeps the bad file as "<path>.bak"; an individual entry that does not validate (an
// unknown action, a binding of the wrong device) is dropped and listed in the report.
#pragma once

#include "rg/control_binding.h"
#include "rg/control_schema.h"
#include "rg/control_types.h"

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace rg {

inline constexpr int kControlsFormatVersion = 1;
inline constexpr std::size_t kMaxBindingsPerAction = 8;

// Wheel when the name looks like one and the SDL mapping is standard, Gamepad for any other
// standard-mapped pad, Generic when the mapping is unknown.
DeviceClass classify_joypad(const std::string& name, bool standard_mapping);

// ---- raw input -------------------------------------------------------------------------

struct PadSnapshot {
    int slot = -1;
    std::vector<int> buttons; // indices of the pressed buttons
    std::vector<float> axes;  // raw axis values, index = axis number
};

struct InputSnapshot {
    std::vector<std::string> keys;  // pressed physical keys, by name
    std::vector<int> mouse_buttons; // held mouse buttons (not the wheel)
    float mouse_dx = 0.0F;          // relative motion since the previous snapshot, pixels
    float mouse_dy = 0.0F;
    float wheel_up = 0.0F;          // wheel ticks since the previous snapshot (the wheel is a
    float wheel_down = 0.0F;        // pulse, not a held button: it counts as `pulses`)
    float wheel_left = 0.0F;
    float wheel_right = 0.0F;
    std::vector<PadSnapshot> pads;
};

// The result of one evaluation, indexed like ActionSchema::actions().
struct ActionState {
    std::vector<float> value;    // merged over every device; Button: 0 / 1, Axis: -1..1 / 0..1 / delta
    std::vector<float> raw;      // analogue value before deadzone/curve/sensitivity (calibrated, inverted, spanned)
    std::vector<float> pulses;   // wheel ticks bound to the action (zoom steps)
    std::vector<float> keyboard; // the keyboard's own contribution
    std::vector<float> pad;      // the joypads' own contribution (largest magnitude)
    std::vector<float> pad_raw;  // the joypads' own raw channel (largest magnitude): the camera rigs' stick position
    std::vector<int> source;     // DeviceClass (as int) of the device whose value won, -1 = none
};

// ---- devices ---------------------------------------------------------------------------

struct DeviceInfo {
    std::string key;   // "keyboard", "mouse", "joy:<guid>#<n>"
    DeviceClass cls = DeviceClass::Keyboard;
    std::string name;  // "Xbox Controller"
    std::string label; // "Xbox Controller #2"
    std::string guid;
    int vendor = 0;
    int product = 0;
    int ordinal = 0;   // joypads: 1, 2, ...
    bool connected = false;
    int slot = -1;     // the engine's device index while connected
    bool customised = false; // the player changed something (the profile has overrides, possibly none left)
    bool inherits = false;   // not customised and resolved through the GUID's #1 profile
};

struct RowInfo {
    std::string action;
    int sign = 0; // 0 = the whole action / the full axis, -1 / +1 = the negative / positive side of a signed axis
    std::string label;
    ControlGroup group = ControlGroup::Driving;
    std::vector<int> bindings; // indices into effective_bindings(device, action)
    bool overridden = false;
    bool conflict = false;
};

struct Conflict {
    std::string action_a;
    std::string action_b;
    Binding binding_a;
    Binding binding_b;
};

struct ControlsLoadReport {
    bool file_missing = false;
    bool parse_error = false;      // syntax error, wrong structure: defaults
    bool unknown_version = false;  // "rg.controls/N" with N != 1: defaults
    int version_found = 0;
    bool backup_written = false;   // load_file kept the bad file as .bak
    std::string backup_path;
    std::string message;           // one human line for a log / the controls screen, empty when fine
    std::vector<std::string> dropped; // entries that did not validate
    [[nodiscard]] bool ok() const { return !parse_error && !unknown_version; }
};

class Controls {
public:
    Controls();

    // Loads <data_dir>/controls/actions.json and defaults/{keyboard,mouse,gamepad}.json (wheel uses the
    // gamepad's, generic has none). False + *err when a file is missing or invalid.
    bool load_data(const std::string& data_dir, std::string* err = nullptr);
    // Same from text (tests): the three default profiles by class name.
    bool load_data_text(const std::string& actions_json, const std::map<std::string, std::string>& default_profiles_json,
                        std::string* err = nullptr);

    [[nodiscard]] const ActionSchema& schema() const { return schema_; }
    [[nodiscard]] bool ready() const { return schema_.size() > 0; }
    // Bumped by every change of a binding or of the device list: a consumer caching things derived
    // from the bindings (the list of keys to poll) compares it.
    [[nodiscard]] std::uint64_t revision() const { return revision_; }

    // ---- persistence ----
    ControlsLoadReport load_text(const std::string& json_text);
    ControlsLoadReport load_file(const std::string& path);
    [[nodiscard]] std::string to_json() const;
    // Atomic (temp file in the same folder, then rename). False + *err on an I/O failure.
    bool save_file(const std::string& path, std::string* err = nullptr);
    [[nodiscard]] bool dirty() const { return dirty_; }
    void mark_saved() { dirty_ = false; }

    // ---- devices ----
    // A joypad appeared (or was there at start-up). Returns its profile key. `standard_mapping` is
    // true when the engine knows an SDL gamepad mapping for it.
    std::string joypad_connected(int slot, const std::string& guid, const std::string& name, int vendor, int product,
                                 bool standard_mapping);
    void joypad_disconnected(int slot);
    // Keyboard, mouse, the connected pads by slot, then the remembered unplugged ones by key.
    [[nodiscard]] std::vector<DeviceInfo> devices() const;
    [[nodiscard]] std::optional<DeviceInfo> device(const std::string& key) const;
    [[nodiscard]] std::string key_for_slot(int slot) const; // "" when no connected pad has it

    // ---- bindings (all by device key) ----
    // The actions this device can have bindings for (keyboard: no mouse-look; mouse: no menu; pads:
    // no mouse-look), schema order.
    [[nodiscard]] std::vector<std::string> applicable_actions(const std::string& device_key) const;
    // The bindings in force: the profile's override, else the class default.
    [[nodiscard]] std::vector<Binding> effective_bindings(const std::string& device_key, const std::string& action) const;
    [[nodiscard]] std::vector<Binding> default_bindings(const std::string& device_key, const std::string& action) const;
    [[nodiscard]] bool is_overridden(const std::string& device_key, const std::string& action) const;
    // The rows of the device's action list (see RowInfo), in schema order.
    [[nodiscard]] std::vector<RowInfo> rows(const std::string& device_key) const;

    // Replaces all bindings of an action. False + *err when the action is unknown or a binding does not
    // suit the device or the action (a key on a pad, mouse motion on a button, more than
    // kMaxBindingsPerAction). An override equal to the default is dropped.
    bool set_bindings(const std::string& device_key, const std::string& action, const std::vector<Binding>& bindings,
                      std::string* err = nullptr);
    // Binds `binding` to the row (action, sign): replace = the row's current bindings go first. A digital
    // binding on a signed axis takes the row's sign.
    bool bind(const std::string& device_key, const std::string& action, int sign, Binding binding, bool replace,
              std::string* err = nullptr);
    bool remove_binding(const std::string& device_key, const std::string& action, int index, std::string* err = nullptr);
    bool replace_binding(const std::string& device_key, const std::string& action, int index, Binding binding,
                         std::string* err = nullptr);
    bool clear_action(const std::string& device_key, const std::string& action, std::string* err = nullptr);
    bool reset_action(const std::string& device_key, const std::string& action);
    // Back to the class default (a second identical pad stops inheriting the first one's setup too).
    bool reset_device(const std::string& device_key);
    // One axis feeding throttle and brake: `throttle_half` is the half of the (optionally calibrated)
    // axis the throttle pedal pushes toward, the brake gets the other half.
    bool bind_combined_pedals(const std::string& device_key, int axis, AxisSpan throttle_half, const AxisTuning& tuning,
                              std::string* err = nullptr);

    // Same input bound to two actions of one device whose modes overlap (see ActionDef::modes). Different
    // halves of one axis do not conflict.
    [[nodiscard]] std::vector<Conflict> conflicts(const std::string& device_key) const;
    [[nodiscard]] std::vector<Conflict> conflicts_for(const std::string& device_key, const std::string& action) const;

    // ---- evaluation ----
    // Every connected device (keyboard, mouse, each pad found in `in.pads` by slot) through its profile.
    [[nodiscard]] ActionState evaluate(const InputSnapshot& in) const;
    // One device only (the live monitor of the controls screen); `in.pads` holds that pad.
    [[nodiscard]] ActionState evaluate_device(const std::string& device_key, const InputSnapshot& in) const;
    [[nodiscard]] ActionState empty_state() const;
    // The key names, mouse buttons and wheel bindings in force on the active devices: what a consumer has to poll.
    [[nodiscard]] std::vector<std::string> polled_keys() const;
    // True when `button` (a mouse button number) is bound to `action` on the mouse profile (event-driven
    // consumers: the mouse capture on a right click).
    [[nodiscard]] bool mouse_button_bound(const std::string& action, int button) const;
    // The built-in aliases of the menu actions (Enter / Esc / pad A / B) followed by the player's own
    // bindings on the keyboard and the connected pads: what the engine's menu-navigation actions
    // (ui_accept / ui_cancel) are set to. Keys and joy buttons only.
    [[nodiscard]] std::vector<Binding> menu_bindings(const std::string& action) const;

private:
    struct Record {
        std::string key;
        DeviceClass cls = DeviceClass::Keyboard;
        std::string name;
        std::string guid;
        int vendor = 0;
        int product = 0;
        int ordinal = 0;
        bool connected = false;
        int slot = -1;
        bool customised = false;
        std::map<std::string, std::vector<Binding>> overrides;
    };

    [[nodiscard]] Record* find(const std::string& key);
    [[nodiscard]] const Record* find(const std::string& key) const;
    [[nodiscard]] const std::map<std::string, std::vector<Binding>>* base_overrides(const Record& r) const;
    [[nodiscard]] const std::vector<Binding>* default_list(DeviceClass cls, const std::string& action) const;
    [[nodiscard]] std::vector<Binding> effective(const Record& r, const std::string& action) const;
    [[nodiscard]] bool binding_suits(const Record& r, const ActionDef& a, const Binding& b, std::string* err) const;
    void make_custom(Record& r);
    void changed();
    void evaluate_record(const Record& r, const PadSnapshot* pad, const InputSnapshot& in, ActionState& out) const;
    ControlsLoadReport load_document(const std::string& json_text);
    void reset_records();

    [[nodiscard]] const std::vector<std::vector<Binding>>& effective_all(const Record& r) const;

    ActionSchema schema_;
    std::map<DeviceClass, std::map<std::string, std::vector<Binding>>> defaults_;
    std::vector<Record> records_;
    std::uint64_t revision_ = 1;
    bool dirty_ = false;
    // evaluate() runs every frame: the effective bindings per device, rebuilt when revision_ moved.
    // Not thread-safe (the input layer runs on the main thread).
    mutable std::map<std::string, std::vector<std::vector<Binding>>> eff_cache_;
    mutable std::uint64_t eff_cache_revision_ = 0;
};

} // namespace rg
