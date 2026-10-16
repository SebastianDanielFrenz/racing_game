#include "rg_driver_actions.h"
#include <godot_cpp/core/class_db.hpp>

namespace rg_godot {
namespace {
godot::String text(std::string_view value) {
    return godot::String::utf8(std::string(value).c_str());
}
rg::DriverSnapshot convert(const godot::Dictionary& d) {
    rg::DriverSnapshot s;
    s.fitted_bits = static_cast<std::uint32_t>(static_cast<std::int64_t>(d.get("assist_fitted_bits", 0)));
    s.traction_modes = static_cast<std::uint32_t>(static_cast<std::int64_t>(d.get("traction_modes_available_bits", 0)));
    s.traction_effective = static_cast<int>(d.get("traction_mode_effective", 0));
    s.traction_intervening = static_cast<bool>(d.get("traction_control_intervening", false));
    s.traction_active = static_cast<bool>(d.get("assist_traction_control", false));
    s.drive_modes = static_cast<std::uint32_t>(static_cast<std::int64_t>(d.get("drive_modes_available_bits", 0)));
    s.drive_mode_effective = static_cast<int>(d.get("drive_mode_effective", 0));
    s.launch_phase = static_cast<int>(d.get("launch_phase", 0));
    s.launch = static_cast<rg::LaunchLamp>(static_cast<int>(d.get("launch_lamp", 0)));
    s.aero_active = static_cast<bool>(d.get("aero_control_active", false));
    s.aero_target_deg = static_cast<double>(d.get("aero_control_target_deg", 0.0));
    s.nitrous_fitted = static_cast<bool>(d.get("nitrous_fitted", false));
    s.park_fitted = static_cast<bool>(d.get("park_lock_fitted", false));
    s.park_engaged = static_cast<bool>(d.get("park_lock_engaged", false));
    s.epb_fitted = static_cast<bool>(d.get("epb_fitted", false));
    s.epb_state = static_cast<int>(d.get("epb_state", 0));
    s.epb_refused = static_cast<bool>(d.get("epb_refused", false));
    s.abs_fitted = static_cast<bool>(d.get("abs_fitted", false));
    s.abs_on = static_cast<bool>(d.get("abs_on", false));
    s.abs_intervening = static_cast<bool>(d.get("abs_active", false));
    s.esc_fitted = static_cast<bool>(d.get("esc_fitted", false));
    s.esc_effective = static_cast<int>(d.get("esc_mode_effective", 0));
    s.esc_intervening = static_cast<bool>(d.get("esc_active", false));
    s.gear = static_cast<int>(d.get("gear",0));
    s.transmission_kind = static_cast<int>(d.get("transmission_kind", 0));
    s.transmission_selector = static_cast<int>(d.get("transmission_selector", 0));
    s.transmission_manual = static_cast<bool>(d.get("transmission_manual_mode", false));
    s.transmission_sport = static_cast<bool>(d.get("transmission_sport_mode", false));
    s.engine_running = godot::String(d.get("engine_state", "")) == "Running";
    s.speed_mps = static_cast<double>(d.get("speed_mps",0.0));
    s.steering_wheel_deg = static_cast<double>(d.get("steering_wheel_deg",0.0));
    return s;
}
class Sink final : public rg::ChannelSink {
public:
    explicit Sink(godot::Dictionary input) : values(std::move(input)) {}
    double get(std::string_view channel) const override {
        return static_cast<double>(values.get(text(channel), 0.0));
    }
    void set(std::string_view channel, double value) override {
        values[text(channel)] = value;
        godot::Dictionary write;
        write["channel"] = text(channel);
        write["value"] = value;
        writes.push_back(write);
    }
    godot::Dictionary values;
    godot::Array writes;
};
}
bool RgDriverActions::submit(const godot::String& id, int kind, double value, int source) {
    if (kind < PRESS || kind > HOLD || source < KEYBOARD || source > SCRIPT) return false;
    return actions_.submit({id.utf8().get_data(), static_cast<rg::DriverActionKind>(kind), value, static_cast<rg::ActionSource>(source)});
}
godot::Array RgDriverActions::apply(const godot::Dictionary& snapshot, const godot::Dictionary& channels, bool driving_enabled) {
    Sink sink(channels.duplicate());
    actions_.apply({convert(snapshot), driving_enabled}, sink);
    return sink.writes;
}
void RgDriverActions::configure_cabin(const godot::Dictionary& d) {
    rg::CabinConfig c;
    c.crank_timeout_s = d.get("crank_timeout_s",c.crank_timeout_s);
    c.stop_max_speed_mps = d.get("stop_max_speed_mps",c.stop_max_speed_mps);
    c.blink_hz = d.get("blink_hz",c.blink_hz);
    c.indicator_arm_deg = d.get("indicator_arm_deg",c.indicator_arm_deg);
    c.indicator_cancel_deg = d.get("indicator_cancel_deg",c.indicator_cancel_deg);
    c.brake_lamp_threshold = d.get("brake_lamp_threshold",c.brake_lamp_threshold);
    try { actions_.configure_cabin(c); } catch (const std::exception& e) { ERR_PRINT(e.what()); }
}
godot::Array RgDriverActions::advance(const godot::Dictionary& snapshot, const godot::Dictionary& channels, bool enabled, double dt) {
    Sink sink(channels.duplicate());
    actions_.advance({convert(snapshot),enabled},sink,dt);
    return sink.writes;
}
void RgDriverActions::discard_pending() { actions_.discard_pending(); }
void RgDriverActions::configure_auto_shift(bool enabled) { actions_.configure_auto_shift(enabled); }
bool RgDriverActions::get_auto_shift() const { return actions_.cabin().auto_shift; }
godot::Dictionary RgDriverActions::state(const godot::String& id, const godot::Dictionary& snapshot, const godot::Dictionary& channels) {
    const Sink sink(channels);
    const auto s = rg::DriverActions::state(id.utf8().get_data(), convert(snapshot), sink);
    godot::Dictionary d;
    d["fitted"] = s.fitted;
    d["enabled"] = s.enabled;
    d["active"] = s.active;
    d["value"] = s.value;
    d["lamp_segments"] = static_cast<std::int64_t>(s.lamp_segments);
    d["blink"] = s.blink;
    d["lamp_state"] = s.lamp_state;
    d["colour_role"] = text(s.colour_role);
    d["label"] = text(s.label);
    d["aero_target_deg"] = s.aero_target_deg;
    return d;
}
godot::Dictionary RgDriverActions::selector_shift(const godot::Dictionary& snapshot, int up, int down) {
    const rg::SelectorShift r = rg::selector_shift(convert(snapshot), up, down);
    godot::Dictionary d;
    d["gear_request"] = r.gear_request;
    d["change"] = r.gear_request != rg::kGearRequestNoChange;
    d["park_request"] = r.park_request;
    d["refused"] = r.refused;
    d["forward_counts"] = r.forward_counts;
    return d;
}
void RgDriverActions::_bind_methods() {
    using godot::ClassDB;
    using godot::D_METHOD;
    ClassDB::bind_method(D_METHOD("submit", "id", "kind", "value", "source"), &RgDriverActions::submit);
    ClassDB::bind_method(D_METHOD("apply", "snapshot", "channels", "driving_enabled"), &RgDriverActions::apply);
    ClassDB::bind_method(D_METHOD("configure_cabin", "values"), &RgDriverActions::configure_cabin);
    ClassDB::bind_method(D_METHOD("advance", "snapshot", "channels", "enabled", "dt"), &RgDriverActions::advance);
    ClassDB::bind_method(D_METHOD("discard_pending"), &RgDriverActions::discard_pending);
    ClassDB::bind_method(D_METHOD("configure_auto_shift", "enabled"), &RgDriverActions::configure_auto_shift);
    ClassDB::bind_method(D_METHOD("get_auto_shift"), &RgDriverActions::get_auto_shift);
    ClassDB::bind_static_method("RgDriverActions", D_METHOD("state", "id", "snapshot", "channels"), &RgDriverActions::state);
    ClassDB::bind_static_method("RgDriverActions", D_METHOD("selector_shift", "snapshot", "up", "down"), &RgDriverActions::selector_shift);
    BIND_ENUM_CONSTANT(PRESS); BIND_ENUM_CONSTANT(RELEASE); BIND_ENUM_CONSTANT(SET); BIND_ENUM_CONSTANT(HOLD);
    BIND_ENUM_CONSTANT(KEYBOARD); BIND_ENUM_CONSTANT(PAD); BIND_ENUM_CONSTANT(HUD); BIND_ENUM_CONSTANT(COCKPIT_MOUSE);
    BIND_ENUM_CONSTANT(VR_POKE); BIND_ENUM_CONSTANT(VR_GRAB); BIND_ENUM_CONSTANT(VR_RAY); BIND_ENUM_CONSTANT(SCRIPT);
}
}
