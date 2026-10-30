#include "rg/driver_actions.h"

#include <cmath>
#include <utility>
#include <stdexcept>
#include <algorithm>

namespace rg {
namespace {
// Preserve the original forwarding order; per-action events chain through the
// last value written, irrespective of which device submitted them.
constexpr std::array<std::string_view, 20> ids = {
    "auto_shift", "traction_control", "toggle_aero", "toggle_launch_control", "drive_mode", "toggle_nitrous", "park", "epb", "start_stop", "hazard", "indicator_left", "indicator_right", "horn", "lights",
    "transmission_mode", "manual_mode", "toggle_abs", "cycle_esc", "electric_mode", "main_beam"
};
constexpr std::array<std::string_view, 8> channels = {
    "assist.auto_shift", "assist.traction_control", "assist.aero_control", "assist.launch_control", "assist.drive_mode", "nitrous_arm", "park_lock", "epb"
};
int slot(std::string_view id) {
    for (std::size_t i = 0; i < ids.size(); ++i) if (ids[i] == id) return static_cast<int>(i);
    return -1;
}
int mode(double value) {
    if (!std::isfinite(value) || value < -1 || value > 31) return -1;
    return static_cast<int>(std::round(value));
}
// All offered bits participate: future drive modes need no new cycling law.
int next_offered(std::uint32_t bits, int current) {
    if (!bits) return -1;
    if (current < 0 || current >= 32) current = -1;
    for (int step = 1; step <= 32; ++step) {
        const int next = (current + step) % 32;
        if (bits & (std::uint32_t{1} << next)) return next;
    }
    return -1;
}
std::uint32_t segment(int value) {
    return value >= 0 && value < 32 ? (std::uint32_t{1} << value) : 0;
}
} // namespace

bool DriverActions::submit(ActionEvent event) {
    const int index = slot(event.id);
    if (index < 0 || !std::isfinite(event.value)) return false;
    if (index == 12) {
        // Horn is a held level, not a rising-edge toggle.
        event.value = event.kind == DriverActionKind::Release ? 0 : event.value;
        event.kind = DriverActionKind::Set;
        pending_.push_back(std::move(event));
        return true;
    }
    if (event.kind == DriverActionKind::Hold) {
        const bool down = event.value != 0;
        const bool edge = down && !held_[static_cast<std::size_t>(index)];
        held_[static_cast<std::size_t>(index)] = down;
        if (!edge) return true;
        event.kind = DriverActionKind::Press;
    }
    if (event.kind == DriverActionKind::Release) {
        held_[static_cast<std::size_t>(index)] = false;
        return true;
    }
    // Previously input_map kept this toggle even while driving was blocked.
    // Keep that behaviour, including same-car reset/world-switch persistence.
    if (index == 0) {
        cabin_.auto_shift = event.kind == DriverActionKind::Set ? event.value != 0 : !cabin_.auto_shift;
        return true;
    }
    pending_.push_back(std::move(event));
    return true;
}

void DriverActions::discard_pending() {
    pending_.clear(); // held levels survive, so holding a button cannot replay it
}

void DriverActions::apply(const VehicleContext& context, ChannelSink& sink) {
    if (!context.driving_enabled) {
        discard_pending();
        return;
    }
    sink.set(channels[0], cabin_.auto_shift ? 1 : 0);
    const auto& snapshot = context.snapshot;
    for (std::size_t index = 1; index < channels.size(); ++index) {
        for (const auto& event : pending_) {
            if (event.id != ids[index]) continue;
            const double current = sink.get(channels[index]);
            double next = current;
            if (index == 1 || index == 4) {
                const auto bits = index == 1 ? snapshot.traction_modes : snapshot.drive_modes;
                const int target = event.kind == DriverActionKind::Set ? mode(event.value) : next_offered(bits, mode(current));
                if (target < 0 || !(bits & segment(target))) continue;
                next = target;
            } else if (index == 2) {
                if (event.kind == DriverActionKind::Set) {
                    if (!(snapshot.fitted_bits & kAssistBitAeroControl)) continue;
                    next = event.value;
                } else next = toggled_aero_control(snapshot.fitted_bits, current);
            } else if (index == 3) {
                if (event.kind == DriverActionKind::Set) {
                    if (!(snapshot.fitted_bits & kAssistBitLaunchControl)) continue;
                    next = event.value;
                } else next = toggled_launch_control(snapshot.fitted_bits, current);
            } else if (index == 5) {
                // Keep the existing inert channel write on cars without a kit.
                next = event.kind == DriverActionKind::Set ? event.value : 1 - current;
            }
            if (index == 6) {
                if (!snapshot.park_fitted) continue;
                next = event.kind == DriverActionKind::Set ? (event.value != 0 ? 1 : 0) : 1;
                // An automatic's P = N + park lock (physics automatic_tcu).
                if (has_selector(snapshot) && next != 0) sink.set("gear_request", 0);
            } else if (index == 7) {
                if (!snapshot.epb_fitted) continue;
                // Switch LEVEL only; never reconstruct or retry the physics latch.
                next = event.kind == DriverActionKind::Set ? (event.value != 0 ? 1 : 0) : 1 - current;
            }
            sink.set(channels[index], next);
        }
    }
    for (const auto& event : pending_) {
        // ABS switch and ESC mode (assist unit abs / esc blocks): the CURRENT channel value is the base, a car without the
        // block ignores the press silently (no channel exists in its World).
        const int index = slot(event.id);
        if (index == 16) {
            if (!(snapshot.fitted_bits & kAssistBitAbs)) continue;
            const double current = sink.get("assist.abs");
            sink.set("assist.abs", event.kind == DriverActionKind::Set ? (event.value >= 0.5 ? 1 : 0) : toggled_abs(snapshot.fitted_bits, current));
        } else if (index == 17) {
            if (!(snapshot.fitted_bits & kAssistBitEsc)) continue;
            const double current = sink.get("assist.esc");
            const int target = event.kind == DriverActionKind::Set ? mode(event.value) : next_esc_mode(snapshot.esc_modes, mode(current));
            if (target < 0 || target >= kEscModeCodeCount || !(snapshot.esc_modes & segment(target))) continue;
            sink.set("assist.esc", target);
        } else if (index == 18) {
            // Hybrid "E" key: Electric (drive mode 4) on / off. The channel is assist.drive_mode, the same one the mode cycle writes;
            // a car whose drive_modes part does not offer Electric ignores the press. Switching off returns to the mode that was
            // requested before (else Normal, else the first offered mode).
            if (!(snapshot.drive_modes & segment(kDriveModeElectric))) continue;
            const int current = mode(sink.get("assist.drive_mode"));
            const bool want = event.kind == DriverActionKind::Set ? event.value >= 0.5 : current != kDriveModeElectric;
            if (want == (current == kDriveModeElectric)) continue;
            if (want) {
                electric_return_mode_ = current >= 0 && current != kDriveModeElectric ? current : -1;
                sink.set("assist.drive_mode", kDriveModeElectric);
            } else {
                int back = electric_return_mode_ >= 0 ? electric_return_mode_ : kDriveModeNormal;
                if (back == kDriveModeElectric || !(snapshot.drive_modes & segment(back))) {
                    back = -1;
                    for (int code = 0; code < kDriveModeCodeCount && back < 0; ++code)
                        if (code != kDriveModeElectric && (snapshot.drive_modes & segment(code))) back = code;
                }
                if (back >= 0) sink.set("assist.drive_mode", back);
            }
        }
    }
    for (const auto& event : pending_) {
        // Automatic gearbox plan A1: the S/D schedule and manual (tiptronic) mode, only on an automatic or dual-clutch box.
        const int index = slot(event.id);
        if (index < 14 || index > 15 || snapshot.transmission_kind == 0) continue;
        if (index == 14) {
            const double current = sink.get("transmission.mode");
            sink.set("transmission.mode", event.kind == DriverActionKind::Set ? (event.value >= 0.5 ? 1 : 0) : (current >= 0.5 ? 0 : 1));
            continue;
        }
        // index 15, manual mode: only from D. The state is the physics' own (snapshot), not a latch of ours.
        if (snapshot.transmission_selector <= 0) continue;
        const bool want = event.kind == DriverActionKind::Set ? event.value >= 0.5 : !snapshot.transmission_manual;
        if (want == snapshot.transmission_manual) continue;
        if (want) {
            // Physics enters manual mode on a shift count in D; one up plus one down changes no gear.
            sink.set("shift_up_count", sink.get("shift_up_count") + 1);
            sink.set("shift_down_count", sink.get("shift_down_count") + 1);
        } else if (snapshot.hybrid_fitted) {
            // The hybrid supervisor owns the gearbox request (always 99 while it drives): only a selector change ends manual
            // mode, so go through N for manual_exit_hold_s and back to D (advance() sends the D).
            sink.set("gear_request", 0);
            manual_exit_pending_ = true;
            manual_exit_elapsed_s_ = 0;
        } else {
            // Back to D: a changed gear_request ends manual mode (any value >= the gear count is plain D).
            sink.set("gear_request", sink.get("gear_request") == kGearRequestDrive ? kGearRequestDrive - 1 : kGearRequestDrive);
        }
    }
    for (const auto& event : pending_) {
        const int index = slot(event.id);
        if (!cabin_configured_) continue;
        if (index == 8) {
            if((snapshot.transmission_kind==4||snapshot.hybrid_fitted)) {
                if(!snapshot.electric_ready||std::abs(snapshot.speed_mps)<config_.stop_max_speed_mps)
                    sink.set("electric.ready",snapshot.electric_ready?0:1);
                cabin_.auto_cranking=false;sink.set("starter",0);
            } else if (!snapshot.engine_running) {
                cabin_.auto_cranking = true; cabin_.auto_crank_elapsed_s = 0;
                sink.set("ignition",1);
            } else if (std::abs(snapshot.speed_mps) < config_.stop_max_speed_mps) {
                cabin_.auto_cranking = false; sink.set("ignition",0); sink.set("starter",0);
            }
        } else if (index == 9) {
            cabin_.hazard = event.kind == DriverActionKind::Set ? event.value != 0 : !cabin_.hazard;
            cabin_.blink_elapsed_s = 0;
        } else if (index == 10 || index == 11) {
            const int direction = index == 10 ? 1 : -1; // ISO left positive
            cabin_.indicator = event.kind == DriverActionKind::Set ? (event.value != 0 ? direction : 0) : cabin_.indicator == direction ? 0 : direction;
            cabin_.indicator_turned = false; cabin_.blink_elapsed_s = 0;
        } else if (index == 12) cabin_.horn = event.value != 0;
        else if (index == 13) {
            // Press cycles the switch Auto -> On -> Off -> Auto. Set picks a position:
            // < 0 auto, 0 off, 1 on (dipped), 2 on with main beam.
            if (event.kind == DriverActionKind::Set) {
                const long v = std::lround(event.value);
                if (v < 0) cabin_.light_mode = LightMode::Auto;
                else if (v == 0) { cabin_.light_mode = LightMode::Off; cabin_.lights = 0; }
                else { cabin_.light_mode = LightMode::On; cabin_.lights = 1; cabin_.main_beam = v >= 2; }
            } else {
                cabin_.light_mode = cabin_.light_mode == LightMode::Auto ? LightMode::On
                                  : cabin_.light_mode == LightMode::On ? LightMode::Off : LightMode::Auto;
            }
            apply_light_mode();
        } else if (index == 19) {
            // Main beam (stalk push / key): Press toggles, Set picks. Only while the headlamps are on - with the
            // lamps off the request does nothing (no latch that would surprise when the auto lamps come on).
            const bool want = event.kind == DriverActionKind::Set ? event.value != 0 : !cabin_.main_beam;
            cabin_.main_beam = want && cabin_.lights > 0;
            apply_light_mode();
        }
    }
    discard_pending();
}

namespace {
// Selector positions in lever order.
enum Pos { PosP = 0, PosR = 1, PosN = 2, PosD = 3, PosB = 4 };
int selector_pos(const DriverSnapshot& s) {
    if (s.transmission_selector < 0) return PosR;
    if (s.transmission_selector > 0) return s.b_active ? PosB : PosD;
    return s.park_fitted && s.park_engaged ? PosP : PosN;
}
} // namespace

bool has_selector(const DriverSnapshot& snapshot) { return snapshot.transmission_kind != 0 || snapshot.hybrid_fitted; }

SelectorShift selector_shift(const DriverSnapshot& snapshot, int up, int down) {
    SelectorShift out;
    if (!has_selector(snapshot) || (up <= 0 && down <= 0)) return out;
    int pos = selector_pos(snapshot);
    if (pos == PosD && snapshot.transmission_manual) return out; // tiptronic: the counters change gears in physics
    out.forward_counts = false;
    // B above D (EV one-pedal regeneration; a hybrid in Eco or Electric mode). Physics steps D <-> B on the shift counters itself, so ONE
    // press that only moves between D and B is forwarded as counters and writes no gear_request (no request value to lose to an
    // unchanged channel). Presses that go further (down past D) use the absolute request as before (B is the step above D: two presses down from B reach N).
    if (snapshot.b_available && (pos == PosD || pos == PosB) && !(up > 0 && down > 0)) {
        if (pos == PosD && up > 0) { out.forward_counts = true; return out; }
        if (pos == PosB && up > 0) return out; // nothing above B
        if (pos == PosB && down == 1) { out.forward_counts = true; return out; }
    }
    const bool standstill = std::abs(snapshot.speed_mps) <= kSelectorStandstillMps;
    const int lowest = snapshot.park_fitted ? PosP : PosR;
    const int start = pos;
    // One step per press, up first then down; P and R are refused while rolling (physics refuses them too).
    for (int i = 0; i < up && pos < PosD; ++i) ++pos;
    for (int i = 0; i < down; ++i) {
        const int next = pos - 1;
        if (next < lowest) break;
        if (!standstill && (next == PosP || next == PosR)) { out.refused = true; break; }
        pos = next;
    }
    if (pos == start) return out;
    switch (pos) {
    case PosP: out.gear_request = 0.0; out.park_request = 1; break;
    case PosR: out.gear_request = -1.0; out.park_request = 0; break;
    case PosN: out.gear_request = 0.0; out.park_request = 0; break;
    default: out.gear_request = kGearRequestDrive; out.park_request = 0; break;
    }
    return out;
}

std::string selector_label(const DriverSnapshot& snapshot) {
    if (!has_selector(snapshot)) return {};
    if (snapshot.transmission_selector < 0) return "R";
    if (snapshot.transmission_selector == 0) return snapshot.park_engaged ? "P" : "N";
    if (snapshot.b_active) return "B";
    if (snapshot.transmission_manual) return "M" + std::to_string(std::max(1, snapshot.gear));
    return snapshot.transmission_sport ? "S" : "D";
}

std::string selector_display(const DriverSnapshot& snapshot) {
    std::string text = selector_label(snapshot);
    if (snapshot.hybrid_electric && text == "D") return "E"; // Electric mode: the combustion engine is off, no gear number to show
    if (text != "D" && text != "S") return text;
    bool gear_shown = false;
    if (snapshot.hybrid_fitted) gear_shown = snapshot.engine_running && snapshot.gear > 0 && snapshot.clutch_engagement >= 0.5;
    else if (snapshot.transmission_kind != 4) gear_shown = snapshot.gear > 0;
    return gear_shown ? text + std::to_string(snapshot.gear) : text;
}

void DriverActions::configure_cabin(const CabinConfig& c) {
    if (!std::isfinite(c.crank_timeout_s) || c.crank_timeout_s <= 0 ||
        !std::isfinite(c.stop_max_speed_mps) || c.stop_max_speed_mps < 0 ||
        !std::isfinite(c.blink_hz) || c.blink_hz <= 0 ||
        !std::isfinite(c.indicator_arm_deg) || !std::isfinite(c.indicator_cancel_deg) ||
        c.indicator_cancel_deg < 0 || c.indicator_arm_deg <= c.indicator_cancel_deg ||
        !std::isfinite(c.brake_lamp_threshold) || c.brake_lamp_threshold < 0 || c.brake_lamp_threshold > 1 ||
        !std::isfinite(c.manual_exit_hold_s) || c.manual_exit_hold_s <= 0)
        throw std::invalid_argument("invalid cabin control configuration");
    if (c.auto_lights &&
        (!std::isfinite(c.auto_lights_on_lux) || !std::isfinite(c.auto_lights_off_lux) || c.auto_lights_on_lux <= 0 ||
         c.auto_lights_off_lux < c.auto_lights_on_lux || !std::isfinite(c.auto_lights_on_delay_s) ||
         !std::isfinite(c.auto_lights_off_delay_s) || c.auto_lights_on_delay_s < 0 || c.auto_lights_off_delay_s < 0))
        throw std::invalid_argument("invalid automatic headlamp configuration (0 < on_lux <= off_lux, delays >= 0)");
    config_ = c;
    cabin_configured_ = true;
}

void DriverActions::advance(const VehicleContext& context, ChannelSink& sink, double dt) {
    if (!std::isfinite(dt) || dt < 0) dt = 0;
    if (manual_exit_pending_) {
        // Second half of the hybrid manual-mode exit; anything else touching gear_request in the meantime cancels it.
        manual_exit_elapsed_s_ += dt;
        if (manual_exit_elapsed_s_ >= config_.manual_exit_hold_s) {
            manual_exit_pending_ = false;
            if (sink.get("gear_request") == 0) sink.set("gear_request", kGearRequestDrive);
        }
    }
    if (!cabin_configured_) return;
    if (!context.driving_enabled) {
        cabin_.horn = false; cabin_.auto_cranking = false;
        sink.set("starter",0); sink.set("cabin.horn",0);
        discard_pending(); return;
    }
    if (cabin_.auto_cranking) {
        cabin_.auto_crank_elapsed_s += dt;
        if (context.snapshot.engine_running || cabin_.auto_crank_elapsed_s >= config_.crank_timeout_s || sink.get("ignition") == 0) {
            cabin_.auto_cranking = false; sink.set("starter",0);
        } else sink.set("starter",1);
    }
    if (cabin_.indicator != 0 && !cabin_.hazard) {
        const double angle = context.snapshot.steering_wheel_deg * cabin_.indicator;
        if (angle >= config_.indicator_arm_deg) cabin_.indicator_turned = true;
        if (cabin_.indicator_turned && angle <= config_.indicator_cancel_deg) {
            cabin_.indicator = 0; cabin_.indicator_turned = false;
        }
    }
    const double period = 1 / config_.blink_hz;
    const bool lit = cabin_.blink_elapsed_s < period / 2;
    cabin_.blink_elapsed_s = std::fmod(cabin_.blink_elapsed_s + dt,period);
    sink.set("cabin.blink_lit",lit ? 1 : 0);
    sink.set("cabin.hazard",cabin_.hazard ? 1 : 0);
    sink.set("cabin.indicator",cabin_.indicator);
    sink.set("cabin.indicator_left",lit && (cabin_.hazard || cabin_.indicator > 0) ? 1 : 0);
    sink.set("cabin.indicator_right",lit && (cabin_.hazard || cabin_.indicator < 0) ? 1 : 0);
    sink.set("cabin.brake_lamp",sink.get("brake") > config_.brake_lamp_threshold ? 1 : 0);
    sink.set("cabin.horn",cabin_.horn ? 1 : 0);
    update_auto_lights(context.snapshot.ambient_lux, dt);
    apply_light_mode();
    sink.set("cabin.lights",cabin_.lights);
    sink.set("cabin.light_mode",static_cast<int>(cabin_.light_mode));
}

const char* light_mode_name(LightMode mode) {
    switch (mode) {
    case LightMode::Auto: return "auto";
    case LightMode::On: return "on";
    case LightMode::Off: return "off";
    }
    return "auto";
}

bool parse_light_mode(std::string_view name, LightMode& out) {
    for (LightMode m : {LightMode::Auto, LightMode::On, LightMode::Off})
        if (name == light_mode_name(m)) { out = m; return true; }
    return false;
}

void DriverActions::set_light_mode(LightMode mode) {
    cabin_.light_mode = mode;
    apply_light_mode();
}

void DriverActions::apply_light_mode() {
    bool on = cabin_.lights > 0;
    switch (cabin_.light_mode) {
    case LightMode::Off: on = false; break;
    case LightMode::On: on = true; break;
    case LightMode::Auto:
        if (config_.auto_lights && cabin_.light_sensor_valid) on = cabin_.light_sensor_dark;
        break; // no sensor: Auto holds whatever the lamps were
    }
    if (!on) cabin_.main_beam = false; // main beam needs the headlamps; it does not come back on its own
    cabin_.lights = on ? (cabin_.main_beam ? 2 : 1) : 0;
}

void DriverActions::update_auto_lights(double lux, double dt) {
    if (!config_.auto_lights || !std::isfinite(lux)) return;
    auto decide = [&](bool dark) {
        cabin_.light_sensor_dark = dark;
        cabin_.light_sensor_timer_s = 0;
    };
    if (!cabin_.light_sensor_valid) {
        cabin_.light_sensor_valid = true;
        decide(lux < config_.auto_lights_on_lux);
        return;
    }
    const bool flip = cabin_.light_sensor_dark ? lux > config_.auto_lights_off_lux : lux < config_.auto_lights_on_lux;
    if (!flip) { cabin_.light_sensor_timer_s = 0; return; }
    cabin_.light_sensor_timer_s += dt;
    const double delay = cabin_.light_sensor_dark ? config_.auto_lights_off_delay_s : config_.auto_lights_on_delay_s;
    if (cabin_.light_sensor_timer_s >= delay) decide(!cabin_.light_sensor_dark);
}

ControlState DriverActions::state(std::string_view id, const DriverSnapshot& snapshot, const ChannelSink& sink) {
    ControlState out;
    if (id == "brake_lamp" || id == "reverse_lamp") {
        out.fitted = true; out.enabled = true;
        out.active = id == "reverse_lamp" ? snapshot.gear < 0 : sink.get("cabin.brake_lamp") != 0;
        out.value = out.active ? 1 : 0; out.lamp_segments = out.active ? 1 : 0;
    } else if (id == "hazard" || id == "indicator_left" || id == "indicator_right" || id == "horn" || id == "start_stop") {
        out.fitted = true; out.enabled = true;
        const auto channel = id == "hazard" ? "cabin.hazard" : id == "horn" ? "cabin.horn" : id == "start_stop" ? "ignition" : id == "indicator_left" ? "cabin.indicator_left" : "cabin.indicator_right";
        out.value = sink.get(id=="start_stop"&&(snapshot.transmission_kind==4||snapshot.hybrid_fitted)?"electric.ready":channel); out.active = out.value != 0;
        out.lamp_segments = id == "hazard" ? (out.active && sink.get("cabin.indicator_left") != 0 ? 1 : 0) : id == "start_stop" ? (((snapshot.transmission_kind==4||snapshot.hybrid_fitted)?snapshot.electric_ready:snapshot.engine_running) ? 1 : 0) : out.active ? 1 : 0;
        out.blink = id == "hazard" && out.active;
        out.label = id == "start_stop" ? ((snapshot.transmission_kind==4||snapshot.hybrid_fitted)?(snapshot.electric_ready?"READY":"OFF"):(snapshot.engine_running?"RUNNING":"OFF")) : std::string(id);
        out.colour_role = out.active ? "ok" : "off";
    } else if (id == "lights" || id == "head_lamp" || id == "tail_lamp" || id == "main_beam") {
        out.fitted = true; out.enabled = true;
        out.value = sink.get("cabin.lights");
        out.active = id == "main_beam" ? out.value >= 2 : out.value != 0;
        out.lamp_segments = out.active ? 1 : 0;
        out.label = out.value >= 2 ? "MAIN BEAM" : out.value != 0 ? "LIGHTS" : "";
        if (sink.get("cabin.light_mode") == static_cast<int>(LightMode::Auto))
            out.label = out.label.empty() ? "AUTO" : "AUTO " + out.label;
        out.colour_role = !out.active ? "off" : out.value >= 2 ? "info" : "ok";
    } else if (id == "light_mode") {
        out.fitted = true; out.enabled = true;
        out.value = sink.get("cabin.light_mode");
        const auto mode = static_cast<LightMode>(std::clamp(static_cast<int>(std::lround(out.value)), 0, 2));
        out.active = mode == LightMode::Auto;
        out.lamp_state = static_cast<int>(mode);
        out.lamp_segments = mode == LightMode::Auto ? 1 : 0;
        out.label = mode == LightMode::Auto ? "AUTO" : mode == LightMode::On ? "ON" : "OFF";
        out.colour_role = mode == LightMode::Auto ? "info" : mode == LightMode::On ? "ok" : "off";
    } else if (id == "park") {
        out.fitted = snapshot.park_fitted;
        out.enabled = out.fitted;
        out.active = out.fitted && snapshot.park_engaged;
        out.value = out.active ? 1 : 0;
        out.lamp_segments = out.active ? 1 : 0;
        out.label = out.active ? "P" : "";
        out.colour_role = out.active ? "warning" : "off";
    } else if (id == "transmission") {
        // Automatic gearbox plan A1: the selector letter (P R N D S M<n>) of an automatic or dual-clutch box.
        out.fitted = has_selector(snapshot);
        out.enabled = out.fitted;
        out.value = snapshot.transmission_selector;
        out.active = out.fitted && snapshot.transmission_selector != 0;
        out.lamp_segments = out.fitted ? 1 : 0;
        out.label = selector_display(snapshot);
        out.lamp_state = snapshot.transmission_kind;
        out.colour_role = !out.fitted ? "off" : snapshot.transmission_sport ? "sport" : "normal";
    } else if (id == "epb") {
        out.fitted = snapshot.epb_fitted;
        out.enabled = out.fitted;
        out.active = out.fitted && snapshot.epb_state != 0;
        out.value = snapshot.epb_state;
        out.blink = out.fitted && snapshot.epb_refused;
        out.lamp_segments = out.blink ? (sink.get("cabin.blink_lit") != 0 ? 1 : 0) : out.active ? 1 : 0;
        out.label = !out.fitted ? "" : out.blink ? "EPB REFUSED" : out.active ? "EPB" : "";
        out.colour_role = out.lamp_segments ? "warning" : "off";
    } else if (id == "traction_control") {
        out.fitted = (snapshot.fitted_bits & kAssistBitTractionControl) != 0;
        out.enabled = out.fitted && snapshot.traction_modes != 0;
        out.value = snapshot.traction_effective;
        out.active = snapshot.traction_active;
        out.lamp_segments = out.fitted ? segment(snapshot.traction_effective) : 0;
        out.blink = snapshot.traction_intervening;
        out.label = traction_mode_name(snapshot.traction_effective);
        out.colour_role = out.blink ? "warning" : (out.value == 0 ? "off" : "ok");
    } else if (id == "drive_mode") {
        out.fitted = snapshot.drive_modes != 0;
        out.enabled = out.fitted;
        out.value = snapshot.drive_mode_effective;
        out.active = out.fitted;
        out.lamp_segments = out.fitted ? segment(snapshot.drive_mode_effective) : 0;
        out.label = out.fitted ? drive_mode_name(snapshot.drive_mode_effective) : "";
        out.colour_role = out.value == 0 ? "eco" : out.value == 1 ? "normal" : out.value == 2 ? "sport" : out.value == 3 ? "racing" : "electric";
    } else if (id == "abs") {
        out.fitted = snapshot.abs_fitted;
        out.enabled = out.fitted;
        out.active = out.fitted && snapshot.abs_on;
        out.value = out.active ? 1 : 0;
        out.blink = out.active && snapshot.abs_intervening;
        out.lamp_segments = out.active ? 1 : 0;
        out.label = !out.fitted ? "" : !out.active ? "ABS OFF" : out.blink ? "ABS ACTIVE" : "ABS";
        out.colour_role = !out.active ? "warning" : out.blink ? "warning" : "ok";
    } else if (id == "esc") {
        out.fitted = snapshot.esc_fitted;
        out.enabled = out.fitted;
        out.value = snapshot.esc_effective;
        out.active = out.fitted && snapshot.esc_effective != 0;
        out.blink = out.active && snapshot.esc_intervening;
        out.lamp_segments = out.fitted ? segment(snapshot.esc_effective) : 0;
        out.label = !out.fitted ? "" : !out.active ? "ESC OFF" : std::string("ESC ") + (snapshot.esc_effective == 2 ? "SPORT" : "ON") + (out.blink ? " ACTIVE" : "");
        out.colour_role = !out.active || out.blink ? "warning" : "ok";
    } else if (id == "toggle_aero") {
        out.fitted = (snapshot.fitted_bits & kAssistBitAeroControl) != 0;
        out.enabled = out.fitted;
        out.value = snapshot.aero_active ? 1 : 0;
        out.active = snapshot.aero_active;
        out.lamp_segments = out.fitted && snapshot.aero_active ? 1 : 0;
        out.label = snapshot.aero_active ? "AERO AUTO" : "AERO OFF";
        out.colour_role = snapshot.aero_active ? "ok" : "off";
        out.aero_target_deg = snapshot.aero_target_deg;
    } else if (id == "toggle_launch_control") {
        out.fitted = snapshot.launch != LaunchLamp::Hidden;
        out.enabled = out.fitted;
        out.value = sink.get("assist.launch_control");
        out.active = snapshot.launch > LaunchLamp::Off;
        out.lamp_state = static_cast<int>(snapshot.launch);
        out.lamp_segments = snapshot.launch > LaunchLamp::Off ? 1 : 0;
        out.blink = snapshot.launch == LaunchLamp::Armed;
        out.label = launch_lamp_text(snapshot.launch);
        out.colour_role = out.blink ? "warning" : snapshot.launch == LaunchLamp::Launching ? "launch" : out.lamp_segments ? "ok" : "off";
    } else if (id == "auto_shift") {
        out.fitted = true;
        out.enabled = true;
        out.value = sink.get("assist.auto_shift");
        out.active = out.value != 0;
        out.lamp_segments = out.value != 0 ? 1 : 0;
        out.label = out.value != 0 ? "A" : "M";
        out.colour_role = out.value != 0 ? "ok" : "off";
    } else if (id == "toggle_nitrous") {
        out.fitted = snapshot.nitrous_fitted;
        out.enabled = out.fitted;
        out.value = sink.get("nitrous_arm");
        out.active = out.fitted && out.value != 0;
        out.lamp_segments = out.fitted && out.value != 0 ? 1 : 0;
        out.label = out.value != 0 ? "ARMED" : "OFF";
        out.colour_role = out.value != 0 ? "warning" : "off";
    }
    return out;
}
} // namespace rg
