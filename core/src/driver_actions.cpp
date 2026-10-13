#include "rg/driver_actions.h"

#include <cmath>
#include <utility>

namespace rg {
namespace {
// Preserve the original forwarding order; per-action events chain through the
// last value written, irrespective of which device submitted them.
constexpr std::array<std::string_view, 6> ids = {
    "auto_shift", "traction_control", "toggle_aero", "toggle_launch_control", "drive_mode", "toggle_nitrous"
};
constexpr std::array<std::string_view, 6> channels = {
    "assist.auto_shift", "assist.traction_control", "assist.aero_control", "assist.launch_control", "assist.drive_mode", "nitrous_arm"
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
    for (std::size_t index = 1; index < ids.size(); ++index) {
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
            sink.set(channels[index], next);
        }
    }
    discard_pending();
}

ControlState DriverActions::state(std::string_view id, const DriverSnapshot& snapshot, const ChannelSink& sink) {
    ControlState out;
    if (id == "traction_control") {
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
