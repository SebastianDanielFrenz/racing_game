#include "rg/driver_actions.h"

#include <cmath>
#include <utility>
#include <stdexcept>
#include <algorithm>

namespace rg {
namespace {
// Preserve the original forwarding order; per-action events chain through the
// last value written, irrespective of which device submitted them.
constexpr std::array<std::string_view, 13> ids = {
    "auto_shift", "traction_control", "toggle_aero", "toggle_launch_control", "drive_mode", "toggle_nitrous", "park", "epb", "start_stop", "hazard", "indicator_left", "indicator_right", "horn"
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
std::string drive_name(int value) {
    return value == 3 ? "RACING" : drive_mode_name(value);
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
            } else if (index == 7) {
                if (!snapshot.epb_fitted) continue;
                // Switch LEVEL only; never reconstruct or retry the physics latch.
                next = event.kind == DriverActionKind::Set ? (event.value != 0 ? 1 : 0) : 1 - current;
            }
            sink.set(channels[index], next);
        }
    }
    for (const auto& event : pending_) {
        const int index = slot(event.id);
        if (!cabin_configured_) continue;
        if (index == 8) {
            if (!snapshot.engine_running) {
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
    }
    discard_pending();
}

void DriverActions::configure_cabin(const CabinConfig& c) {
    if (!std::isfinite(c.crank_timeout_s) || c.crank_timeout_s <= 0 ||
        !std::isfinite(c.stop_max_speed_mps) || c.stop_max_speed_mps < 0 ||
        !std::isfinite(c.blink_hz) || c.blink_hz <= 0 ||
        !std::isfinite(c.indicator_arm_deg) || !std::isfinite(c.indicator_cancel_deg) ||
        c.indicator_cancel_deg < 0 || c.indicator_arm_deg <= c.indicator_cancel_deg ||
        !std::isfinite(c.brake_lamp_threshold) || c.brake_lamp_threshold < 0 || c.brake_lamp_threshold > 1)
        throw std::invalid_argument("invalid cabin control configuration");
    config_ = c;
    cabin_configured_ = true;
}

void DriverActions::advance(const VehicleContext& context, ChannelSink& sink, double dt) {
    if (!cabin_configured_) return;
    if (!std::isfinite(dt) || dt < 0) dt = 0;
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
        out.value = sink.get(channel); out.active = out.value != 0;
        out.lamp_segments = id == "hazard" ? (out.active && sink.get("cabin.indicator_left") != 0 ? 1 : 0) : id == "start_stop" ? (snapshot.engine_running ? 1 : 0) : out.active ? 1 : 0;
        out.blink = id == "hazard" && out.active;
        out.label = id == "start_stop" ? (snapshot.engine_running ? "RUNNING" : "OFF") : std::string(id);
        out.colour_role = out.active ? "ok" : "off";
    } else if (id == "park") {
        out.fitted = snapshot.park_fitted;
        out.enabled = out.fitted;
        out.active = out.fitted && snapshot.park_engaged;
        out.value = out.active ? 1 : 0;
        out.lamp_segments = out.active ? 1 : 0;
        out.label = out.active ? "P" : "";
        out.colour_role = out.active ? "warning" : "off";
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
        out.label = out.fitted ? drive_name(snapshot.drive_mode_effective) : "";
        out.colour_role = out.value == 0 ? "eco" : out.value == 1 ? "normal" : out.value == 2 ? "sport" : "racing";
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
