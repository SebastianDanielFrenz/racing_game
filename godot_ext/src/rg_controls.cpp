#include "rg_controls.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <string>
#include <utility>

using godot::D_METHOD;
using godot::String;

namespace rg_godot {

namespace {

std::string to_std(const String& s) { return std::string(s.utf8().get_data()); }
String from_std(const std::string& s) { return String::utf8(s.c_str()); }

godot::Dictionary result(bool ok, const std::string& error = {}) {
    godot::Dictionary d;
    d["ok"] = ok;
    d["error"] = from_std(error);
    return d;
}

} // namespace

// ---- conversion ---------------------------------------------------------------------------------------

rg::InputSnapshot RgControls::to_snapshot(const godot::Dictionary& d) const {
    rg::InputSnapshot s;
    const godot::PackedStringArray keys = d.get("keys", godot::PackedStringArray());
    for (int i = 0; i < keys.size(); ++i) s.keys.push_back(to_std(keys[i]));
    const godot::PackedInt32Array buttons = d.get("mouse_buttons", godot::PackedInt32Array());
    for (int i = 0; i < buttons.size(); ++i) s.mouse_buttons.push_back(buttons[i]);
    s.mouse_dx = static_cast<float>(static_cast<double>(d.get("mouse_dx", 0.0)));
    s.mouse_dy = static_cast<float>(static_cast<double>(d.get("mouse_dy", 0.0)));
    s.wheel_up = static_cast<float>(static_cast<double>(d.get("wheel_up", 0.0)));
    s.wheel_down = static_cast<float>(static_cast<double>(d.get("wheel_down", 0.0)));
    s.wheel_left = static_cast<float>(static_cast<double>(d.get("wheel_left", 0.0)));
    s.wheel_right = static_cast<float>(static_cast<double>(d.get("wheel_right", 0.0)));
    const godot::Array pads = d.get("pads", godot::Array());
    for (int i = 0; i < pads.size(); ++i) {
        const godot::Dictionary pd = pads[i];
        rg::PadSnapshot p;
        p.slot = static_cast<int>(pd.get("slot", -1));
        const godot::PackedInt32Array pb = pd.get("buttons", godot::PackedInt32Array());
        for (int j = 0; j < pb.size(); ++j) p.buttons.push_back(pb[j]);
        const godot::PackedFloat32Array pa = pd.get("axes", godot::PackedFloat32Array());
        for (int j = 0; j < pa.size(); ++j) p.axes.push_back(pa[j]);
        s.pads.push_back(std::move(p));
    }
    return s;
}

godot::Dictionary RgControls::binding_to_dict(const rg::Binding& b) const {
    godot::Dictionary d;
    d["type"] = String(rg::to_string(b.type));
    d["text"] = from_std(rg::describe(b));
    d["input"] = from_std(rg::input_name(b));
    d["analogue"] = b.is_analogue();
    d["key"] = from_std(b.key);
    d["index"] = b.index;
    d["span"] = String(rg::to_string(b.span));
    d["sign"] = static_cast<double>(b.sign);
    d["deadzone"] = static_cast<double>(b.tuning.deadzone);
    d["saturation"] = static_cast<double>(b.tuning.saturation);
    d["curve"] = static_cast<double>(b.tuning.curve);
    d["sensitivity"] = static_cast<double>(b.tuning.sensitivity);
    d["invert"] = b.tuning.invert;
    d["calibrated"] = b.tuning.calibrated;
    d["cal_min"] = static_cast<double>(b.tuning.cal_min);
    d["cal_max"] = static_cast<double>(b.tuning.cal_max);
    return d;
}

rg::Binding RgControls::dict_to_binding(const godot::Dictionary& d) const {
    rg::Binding b;
    const std::string type = to_std(String(d.get("type", "key")));
    if (type == "mouse_button") b.type = rg::BindingType::MouseButton;
    else if (type == "mouse_motion") b.type = rg::BindingType::MouseMotion;
    else if (type == "joy_button") b.type = rg::BindingType::JoyButton;
    else if (type == "joy_axis") b.type = rg::BindingType::JoyAxis;
    else b.type = rg::BindingType::Key;
    b.key = to_std(String(d.get("key", "")));
    b.index = static_cast<int>(d.get("index", 0));
    const std::string span = to_std(String(d.get("span", "full")));
    b.span = span == "positive" ? rg::AxisSpan::Positive : (span == "negative" ? rg::AxisSpan::Negative : rg::AxisSpan::Full);
    b.sign = static_cast<float>(static_cast<double>(d.get("sign", 1.0)));
    b.tuning.deadzone = static_cast<float>(static_cast<double>(d.get("deadzone", 0.0)));
    b.tuning.saturation = static_cast<float>(static_cast<double>(d.get("saturation", 1.0)));
    b.tuning.curve = static_cast<float>(static_cast<double>(d.get("curve", 1.0)));
    b.tuning.sensitivity = static_cast<float>(static_cast<double>(d.get("sensitivity", 1.0)));
    b.tuning.invert = static_cast<bool>(d.get("invert", false));
    b.tuning.calibrated = static_cast<bool>(d.get("calibrated", false));
    b.tuning.cal_min = static_cast<float>(static_cast<double>(d.get("cal_min", -1.0)));
    b.tuning.cal_max = static_cast<float>(static_cast<double>(d.get("cal_max", 1.0)));
    return b;
}

int RgControls::action_index(const String& action) const {
    const auto it = index_.find(to_std(action));
    return it == index_.end() ? -1 : it->second;
}

// ---- lifecycle ----------------------------------------------------------------------------------------------

godot::Dictionary RgControls::initialize(const String& data_dir, const String& user_dir) {
    godot::Dictionary out;
    std::string err;
    if (!controls_.load_data(to_std(data_dir), &err)) {
        out["ok"] = false;
        out["error"] = from_std(err);
        return out;
    }
    index_.clear();
    for (std::size_t i = 0; i < controls_.schema().size(); ++i) index_[controls_.schema().actions()[i].id] = static_cast<int>(i);
    state_ = controls_.empty_state();
    std::string dir = to_std(user_dir);
    while (!dir.empty() && (dir.back() == '/' || dir.back() == '\\')) dir.pop_back();
    path_ = dir + "/controls.json";
    out["ok"] = true;
    out["error"] = "";
    out["path"] = from_std(path_);
    reload_available_ = true;
    const godot::Dictionary load = reload();
    out["file_missing"] = load["file_missing"];
    out["load_ok"] = load["load_ok"];
    out["message"] = load["message"];
    out["dropped"] = load["dropped"];
    out["backup_path"] = load["backup_path"];
    return out;
}

godot::Dictionary RgControls::reload() {
    godot::Dictionary out;
    const rg::ControlsLoadReport rep = controls_.load_file(path_);
    load_message_ = rep.message;
    out["file_missing"] = rep.file_missing;
    out["load_ok"] = rep.ok();
    out["message"] = from_std(rep.message);
    godot::PackedStringArray dropped;
    for (const std::string& d : rep.dropped) dropped.push_back(from_std(d));
    out["dropped"] = dropped;
    out["backup_path"] = from_std(rep.backup_path);
    return out;
}

bool RgControls::save(bool force) {
    if (!controls_.ready() || path_.empty()) return false;
    if (!force && !controls_.dirty()) return true;
    std::string err;
    if (!controls_.save_file(path_, &err)) {
        godot::UtilityFunctions::push_error(String("RgControls: ") + from_std(err));
        return false;
    }
    return true;
}

String RgControls::get_file_path() const { return from_std(path_); }
String RgControls::get_load_message() const { return from_std(load_message_); }
String RgControls::to_json() const { return from_std(controls_.to_json()); }

// ---- devices -----------------------------------------------------------------------------------------------------

String RgControls::joypad_connected(int slot, const String& guid, const String& name, int vendor, int product, bool standard_mapping) {
    return from_std(controls_.joypad_connected(slot, to_std(guid), to_std(name), vendor, product, standard_mapping));
}

void RgControls::joypad_disconnected(int slot) { controls_.joypad_disconnected(slot); }

godot::Array RgControls::get_devices() const {
    godot::Array out;
    for (const rg::DeviceInfo& d : controls_.devices()) {
        godot::Dictionary e;
        e["key"] = from_std(d.key);
        e["class"] = String(rg::to_string(d.cls));
        e["name"] = from_std(d.name);
        e["label"] = from_std(d.label);
        e["guid"] = from_std(d.guid);
        e["ordinal"] = d.ordinal;
        e["connected"] = d.connected;
        e["slot"] = d.slot;
        e["customised"] = d.customised;
        e["inherits"] = d.inherits;
        out.push_back(e);
    }
    return out;
}

String RgControls::get_device_key_for_slot(int slot) const { return from_std(controls_.key_for_slot(slot)); }

// ---- bindings ---------------------------------------------------------------------------------------------------------

godot::Array RgControls::get_rows(const String& device_key) const {
    godot::Array out;
    const std::string key = to_std(device_key);
    for (const rg::RowInfo& r : controls_.rows(key)) {
        const rg::ActionDef* def = controls_.schema().find(r.action);
        if (def == nullptr) continue;
        godot::Dictionary e;
        e["action"] = from_std(r.action);
        e["sign"] = r.sign;
        e["label"] = from_std(r.label);
        e["help"] = from_std(def->help);
        e["group"] = String(rg::to_string(r.group));
        e["group_title"] = String(rg::group_title(r.group));
        e["kind"] = String(rg::to_string(def->kind));
        e["range"] = def->kind == rg::ActionKind::Button ? String("button")
                                                          : (def->range == rg::AxisRange::Signed ? String("signed")
                                                                                                  : (def->range == rg::AxisRange::Unit ? String("unit") : String("delta")));
        e["overridden"] = r.overridden;
        e["conflict"] = r.conflict;
        const std::vector<rg::Binding> eff = controls_.effective_bindings(key, r.action);
        godot::Array bindings;
        for (const int i : r.bindings) {
            godot::Dictionary b = binding_to_dict(eff[static_cast<std::size_t>(i)]);
            b["list_index"] = i;
            bindings.push_back(b);
        }
        e["bindings"] = bindings;
        out.push_back(e);
    }
    return out;
}

godot::Array RgControls::get_conflicts(const String& device_key) const {
    godot::Array out;
    for (const rg::Conflict& c : controls_.conflicts(to_std(device_key))) {
        const rg::ActionDef* a = controls_.schema().find(c.action_a);
        const rg::ActionDef* b = controls_.schema().find(c.action_b);
        godot::Dictionary e;
        e["action_a"] = from_std(c.action_a);
        e["label_a"] = from_std(a != nullptr ? a->label : c.action_a);
        e["action_b"] = from_std(c.action_b);
        e["label_b"] = from_std(b != nullptr ? b->label : c.action_b);
        e["binding_a"] = binding_to_dict(c.binding_a);
        e["binding_b"] = binding_to_dict(c.binding_b);
        out.push_back(e);
    }
    return out;
}

godot::Dictionary RgControls::bind_captured(const String& device_key, const String& action, int sign, bool replace) {
    std::string err;
    const std::string key = to_std(device_key);
    const std::string act = to_std(action);
    if (!controls_.bind(key, act, sign, capture_.result(), replace, &err)) return result(false, err);
    godot::Dictionary out = result(true);
    godot::Array conflicts;
    for (const rg::Conflict& c : controls_.conflicts_for(key, act)) {
        const std::string& other = c.action_a == act ? c.action_b : c.action_a;
        const rg::ActionDef* def = controls_.schema().find(other);
        conflicts.push_back(from_std(def != nullptr ? def->label : other));
    }
    out["conflicts"] = conflicts;
    return out;
}

godot::Dictionary RgControls::bind_captured_combined_pedals(const String& device_key) {
    const rg::Binding& r = capture_.result();
    if (r.type != rg::BindingType::JoyAxis) return result(false, "move an axis to use it as combined pedals");
    if (r.tuning.calibrated || r.span == rg::AxisSpan::Full) return result(false, "this axis rests at its end, not in the middle: it cannot be combined pedals");
    std::string err;
    if (!controls_.bind_combined_pedals(to_std(device_key), r.index, r.span, rg::AxisTuning{}, &err)) return result(false, err);
    return result(true);
}

// The half-span axis binding `index` of throttle or brake, turned into combined pedals (one axis, both pedals).
godot::Dictionary RgControls::make_combined_pedals(const String& device_key, const String& action, int index) {
    const std::string key = to_std(device_key);
    const std::string act = to_std(action);
    if (act != "throttle" && act != "brake") return result(false, "only the throttle and brake can be combined");
    const std::vector<rg::Binding> eff = controls_.effective_bindings(key, act);
    if (index < 0 || static_cast<std::size_t>(index) >= eff.size()) return result(false, "no such binding");
    const rg::Binding& b = eff[static_cast<std::size_t>(index)];
    if (b.type != rg::BindingType::JoyAxis || b.tuning.calibrated || b.span == rg::AxisSpan::Full)
        return result(false, "this axis rests at its end, not in the middle: it cannot be combined pedals");
    const rg::AxisSpan opposite = b.span == rg::AxisSpan::Positive ? rg::AxisSpan::Negative : rg::AxisSpan::Positive;
    const rg::AxisSpan throttle_half = act == "throttle" ? b.span : opposite;
    std::string err;
    if (!controls_.bind_combined_pedals(key, b.index, throttle_half, rg::AxisTuning{}, &err)) return result(false, err);
    return result(true);
}

bool RgControls::remove_binding(const String& device_key, const String& action, int index) {
    return controls_.remove_binding(to_std(device_key), to_std(action), index);
}

bool RgControls::clear_row(const String& device_key, const String& action, int sign) {
    const std::string key = to_std(device_key);
    const std::string act = to_std(action);
    for (const rg::RowInfo& r : controls_.rows(key)) {
        if (r.action != act || r.sign != sign) continue;
        std::vector<rg::Binding> eff = controls_.effective_bindings(key, act);
        std::vector<rg::Binding> keep;
        for (std::size_t i = 0; i < eff.size(); ++i) {
            bool in_row = false;
            for (const int b : r.bindings) in_row = in_row || static_cast<std::size_t>(b) == i;
            if (!in_row) keep.push_back(eff[i]);
        }
        return controls_.set_bindings(key, act, keep);
    }
    return false;
}

bool RgControls::reset_action(const String& device_key, const String& action) {
    return controls_.reset_action(to_std(device_key), to_std(action));
}

bool RgControls::reset_device(const String& device_key) { return controls_.reset_device(to_std(device_key)); }

godot::Dictionary RgControls::bind_binding(const String& device_key, const String& action, int sign, const godot::Dictionary& binding,
                                           bool replace) {
    std::string err;
    if (!controls_.bind(to_std(device_key), to_std(action), sign, dict_to_binding(binding), replace, &err)) return result(false, err);
    return result(true);
}

godot::Dictionary RgControls::set_binding_tuning(const String& device_key, const String& action, int index, const godot::Dictionary& tuning) {
    const std::string key = to_std(device_key);
    const std::string act = to_std(action);
    std::vector<rg::Binding> eff = controls_.effective_bindings(key, act);
    if (index < 0 || static_cast<std::size_t>(index) >= eff.size()) return result(false, "no such binding");
    rg::Binding b = eff[static_cast<std::size_t>(index)];
    if (!b.is_analogue()) return result(false, "only an axis can be tuned");
    if (tuning.has("deadzone")) b.tuning.deadzone = static_cast<float>(static_cast<double>(tuning["deadzone"]));
    if (tuning.has("saturation")) b.tuning.saturation = static_cast<float>(static_cast<double>(tuning["saturation"]));
    if (tuning.has("curve")) b.tuning.curve = static_cast<float>(static_cast<double>(tuning["curve"]));
    if (tuning.has("sensitivity")) b.tuning.sensitivity = static_cast<float>(static_cast<double>(tuning["sensitivity"]));
    if (tuning.has("invert")) b.tuning.invert = static_cast<bool>(tuning["invert"]);
    if (tuning.has("calibrated") && !static_cast<bool>(tuning["calibrated"])) b.tuning.calibrated = false;
    std::string err;
    if (!controls_.replace_binding(key, act, index, b, &err)) return result(false, err);
    godot::Dictionary out = result(true);
    out["binding"] = binding_to_dict(controls_.effective_bindings(key, act)[static_cast<std::size_t>(index)]);
    return out;
}

godot::Dictionary RgControls::get_binding(const String& device_key, const String& action, int index) const {
    const std::vector<rg::Binding> eff = controls_.effective_bindings(to_std(device_key), to_std(action));
    if (index < 0 || static_cast<std::size_t>(index) >= eff.size()) return godot::Dictionary();
    return binding_to_dict(eff[static_cast<std::size_t>(index)]);
}

// ---- capture / calibration ---------------------------------------------------------------------------------------------------

godot::Dictionary RgControls::capture_begin(const String& device_key, const String& action, int sign, const godot::Dictionary& snapshot) {
    const std::optional<rg::DeviceInfo> dev = controls_.device(to_std(device_key));
    const rg::ActionDef* def = controls_.schema().find(to_std(action));
    if (!dev || def == nullptr) return result(false, "unknown device or action");
    capture_target_ = rg::CaptureTarget{};
    capture_target_.device = dev->cls;
    capture_target_.button_action = def->kind == rg::ActionKind::Button;
    capture_target_.range = def->range;
    capture_target_.sign = sign;
    capture_.begin(capture_target_, to_snapshot(snapshot));
    return result(true);
}

String RgControls::capture_feed(const godot::Dictionary& snapshot, double dt) {
    switch (capture_.feed(to_snapshot(snapshot), dt)) {
        case rg::CaptureState::Waiting: return "waiting";
        case rg::CaptureState::Captured: return "captured";
        case rg::CaptureState::Cancelled: return "cancelled";
    }
    return "cancelled";
}

godot::Dictionary RgControls::get_capture_result() const { return binding_to_dict(capture_.result()); }

double RgControls::get_capture_hold_progress() const { return std::min(1.0, capture_.b_held() / 0.8); }

String RgControls::get_capture_hint() const {
    const rg::CaptureTarget& t = capture_target_;
    switch (t.device) {
        case rg::DeviceClass::Keyboard: return "Press the key";
        case rg::DeviceClass::Mouse:
            if (!t.button_action && t.range == rg::AxisRange::Delta) return "Move the mouse";
            return "Click a mouse button or turn the wheel";
        default: break;
    }
    if (!t.button_action && t.range == rg::AxisRange::Delta) return "";
    if (!t.button_action && t.range == rg::AxisRange::Signed && t.sign == 0) return "Move the stick, wheel or axis";
    if (!t.button_action && t.range == rg::AxisRange::Unit) return "Press the pedal, trigger or button";
    return "Press the button or move the axis";
}

void RgControls::calibration_begin(double rest) { calibrator_.begin(static_cast<float>(rest)); }
void RgControls::calibration_feed(double value) { calibrator_.feed(static_cast<float>(value)); }

godot::Dictionary RgControls::get_calibration() const {
    godot::Dictionary d;
    d["valid"] = calibrator_.valid();
    d["rest"] = static_cast<double>(calibrator_.rest());
    d["low"] = static_cast<double>(calibrator_.lowest());
    d["high"] = static_cast<double>(calibrator_.highest());
    return d;
}

bool RgControls::calibration_apply(const String& device_key, const String& action, int index) {
    const std::string key = to_std(device_key);
    const std::string act = to_std(action);
    const rg::ActionDef* def = controls_.schema().find(act);
    std::vector<rg::Binding> eff = controls_.effective_bindings(key, act);
    if (def == nullptr || index < 0 || static_cast<std::size_t>(index) >= eff.size()) return false;
    rg::Binding b = eff[static_cast<std::size_t>(index)];
    if (b.type != rg::BindingType::JoyAxis) return false;
    const bool unit_pedal = def->kind == rg::ActionKind::Axis && def->range == rg::AxisRange::Unit && b.span == rg::AxisSpan::Full;
    if (!calibrator_.apply(b.tuning, unit_pedal)) return false;
    return controls_.replace_binding(key, act, index, b);
}

// ---- evaluation ---------------------------------------------------------------------------------------------------------------

void RgControls::evaluate(const godot::Dictionary& snapshot) { state_ = controls_.evaluate(to_snapshot(snapshot)); }

#define RG_STATE_GETTER(name, field, type)                                                      \
    type RgControls::name(const String& action) const {                                        \
        const int i = action_index(action);                                                     \
        if (i < 0 || static_cast<std::size_t>(i) >= state_.field.size()) return type(0);        \
        return static_cast<type>(state_.field[static_cast<std::size_t>(i)]);                    \
    }

RG_STATE_GETTER(get_value, value, double)
RG_STATE_GETTER(get_raw, raw, double)
RG_STATE_GETTER(get_pulses, pulses, double)
RG_STATE_GETTER(get_keyboard_value, keyboard, double)
RG_STATE_GETTER(get_pad_value, pad, double)
RG_STATE_GETTER(get_pad_raw, pad_raw, double)

int RgControls::get_source(const String& action) const {
    const int i = action_index(action);
    if (i < 0 || static_cast<std::size_t>(i) >= state_.source.size()) return -1;
    return state_.source[static_cast<std::size_t>(i)];
}

godot::PackedStringArray RgControls::get_polled_keys() const {
    godot::PackedStringArray out;
    for (const std::string& k : controls_.polled_keys()) out.push_back(from_std(k));
    return out;
}

bool RgControls::is_mouse_button_bound(const String& action, int button) const {
    return controls_.mouse_button_bound(to_std(action), button);
}

godot::Array RgControls::get_menu_bindings(const String& action) const {
    godot::Array out;
    for (const rg::Binding& b : controls_.menu_bindings(to_std(action))) {
        godot::Dictionary d;
        d["type"] = String(rg::to_string(b.type));
        d["key"] = from_std(b.key);
        d["index"] = b.index;
        out.push_back(d);
    }
    return out;
}

godot::Array RgControls::monitor_device(const String& device_key, const godot::Dictionary& snapshot) const {
    godot::Array out;
    const rg::ActionState s = controls_.evaluate_device(to_std(device_key), to_snapshot(snapshot));
    for (std::size_t i = 0; i < controls_.schema().size(); ++i) {
        if (s.value[i] == 0.0F && s.pulses[i] == 0.0F) continue;
        const rg::ActionDef& a = controls_.schema().actions()[i];
        godot::Dictionary e;
        e["action"] = from_std(a.id);
        e["label"] = from_std(a.label);
        e["value"] = static_cast<double>(s.value[i]);
        e["raw"] = static_cast<double>(s.raw[i]);
        e["pulses"] = static_cast<double>(s.pulses[i]);
        e["range"] = a.kind == rg::ActionKind::Button ? String("button") : (a.range == rg::AxisRange::Signed ? String("signed") : String("unit"));
        out.push_back(e);
    }
    return out;
}

// ---- binding -----------------------------------------------------------------------------------------------------------------------

void RgControls::_bind_methods() {
    using godot::ClassDB;
    ClassDB::bind_method(D_METHOD("initialize", "data_dir", "user_dir"), &RgControls::initialize);
    ClassDB::bind_method(D_METHOD("is_ready"), &RgControls::is_ready);
    ClassDB::bind_method(D_METHOD("save", "force"), &RgControls::save, DEFVAL(false));
    ClassDB::bind_method(D_METHOD("is_dirty"), &RgControls::is_dirty);
    ClassDB::bind_method(D_METHOD("get_file_path"), &RgControls::get_file_path);
    ClassDB::bind_method(D_METHOD("get_load_message"), &RgControls::get_load_message);
    ClassDB::bind_method(D_METHOD("to_json"), &RgControls::to_json);
    ClassDB::bind_method(D_METHOD("reload"), &RgControls::reload);
    ClassDB::bind_method(D_METHOD("joypad_connected", "slot", "guid", "name", "vendor", "product", "standard_mapping"), &RgControls::joypad_connected);
    ClassDB::bind_method(D_METHOD("joypad_disconnected", "slot"), &RgControls::joypad_disconnected);
    ClassDB::bind_method(D_METHOD("get_devices"), &RgControls::get_devices);
    ClassDB::bind_method(D_METHOD("get_device_key_for_slot", "slot"), &RgControls::get_device_key_for_slot);
    ClassDB::bind_method(D_METHOD("get_revision"), &RgControls::get_revision);
    ClassDB::bind_method(D_METHOD("get_rows", "device_key"), &RgControls::get_rows);
    ClassDB::bind_method(D_METHOD("get_conflicts", "device_key"), &RgControls::get_conflicts);
    ClassDB::bind_method(D_METHOD("bind_captured", "device_key", "action", "sign", "replace"), &RgControls::bind_captured);
    ClassDB::bind_method(D_METHOD("bind_captured_combined_pedals", "device_key"), &RgControls::bind_captured_combined_pedals);
    ClassDB::bind_method(D_METHOD("make_combined_pedals", "device_key", "action", "index"), &RgControls::make_combined_pedals);
    ClassDB::bind_method(D_METHOD("remove_binding", "device_key", "action", "index"), &RgControls::remove_binding);
    ClassDB::bind_method(D_METHOD("clear_row", "device_key", "action", "sign"), &RgControls::clear_row);
    ClassDB::bind_method(D_METHOD("reset_action", "device_key", "action"), &RgControls::reset_action);
    ClassDB::bind_method(D_METHOD("reset_device", "device_key"), &RgControls::reset_device);
    ClassDB::bind_method(D_METHOD("bind_binding", "device_key", "action", "sign", "binding", "replace"), &RgControls::bind_binding);
    ClassDB::bind_method(D_METHOD("set_binding_tuning", "device_key", "action", "index", "tuning"), &RgControls::set_binding_tuning);
    ClassDB::bind_method(D_METHOD("get_binding", "device_key", "action", "index"), &RgControls::get_binding);
    ClassDB::bind_method(D_METHOD("capture_begin", "device_key", "action", "sign", "snapshot"), &RgControls::capture_begin);
    ClassDB::bind_method(D_METHOD("capture_feed", "snapshot", "dt"), &RgControls::capture_feed);
    ClassDB::bind_method(D_METHOD("get_capture_result"), &RgControls::get_capture_result);
    ClassDB::bind_method(D_METHOD("get_capture_hold_progress"), &RgControls::get_capture_hold_progress);
    ClassDB::bind_method(D_METHOD("get_capture_hint"), &RgControls::get_capture_hint);
    ClassDB::bind_method(D_METHOD("calibration_begin", "rest"), &RgControls::calibration_begin);
    ClassDB::bind_method(D_METHOD("calibration_feed", "value"), &RgControls::calibration_feed);
    ClassDB::bind_method(D_METHOD("get_calibration"), &RgControls::get_calibration);
    ClassDB::bind_method(D_METHOD("calibration_apply", "device_key", "action", "index"), &RgControls::calibration_apply);
    ClassDB::bind_method(D_METHOD("evaluate", "snapshot"), &RgControls::evaluate);
    ClassDB::bind_method(D_METHOD("get_value", "action"), &RgControls::get_value);
    ClassDB::bind_method(D_METHOD("get_raw", "action"), &RgControls::get_raw);
    ClassDB::bind_method(D_METHOD("get_pulses", "action"), &RgControls::get_pulses);
    ClassDB::bind_method(D_METHOD("get_keyboard_value", "action"), &RgControls::get_keyboard_value);
    ClassDB::bind_method(D_METHOD("get_pad_value", "action"), &RgControls::get_pad_value);
    ClassDB::bind_method(D_METHOD("get_pad_raw", "action"), &RgControls::get_pad_raw);
    ClassDB::bind_method(D_METHOD("get_source", "action"), &RgControls::get_source);
    ClassDB::bind_method(D_METHOD("get_polled_keys"), &RgControls::get_polled_keys);
    ClassDB::bind_method(D_METHOD("is_mouse_button_bound", "action", "button"), &RgControls::is_mouse_button_bound);
    ClassDB::bind_method(D_METHOD("get_menu_bindings", "action"), &RgControls::get_menu_bindings);
    ClassDB::bind_method(D_METHOD("monitor_device", "device_key", "snapshot"), &RgControls::monitor_device);
}

} // namespace rg_godot
