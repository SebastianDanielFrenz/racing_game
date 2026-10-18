// rg/regen_info.h - the brake-regeneration readout for the HUD and the digital cluster (physics_sim brake blending,
// docs/electric_drive.md "Brake blending"). Engine-neutral, header-only: the Godot binding forwards these values and
// the GDScript only draws the label. Shown only for a car with electric motors (electric_fitted and motors > 0).
#pragma once

#include "ps/drivetrain/powertrain_state.h"

#include <algorithm>
#include <cstdio>
#include <string>

namespace rg {

inline constexpr double kRegenActiveMinW = 100.0;  // below this the readout counts as "not regenerating"

struct RegenHud {
    bool fitted = false;        // the car has electric motors
    bool active = false;        // regenerating right now (power >= kRegenActiveMinW)
    double power_kw = 0;        // power pushed into the pack (>= 0)
    double regen_share = 0;     // share of the service braking the motors supply, 0..1
    double pad_share = 0;       // share the pads supply, 0..1
    double regen_torque_nm = 0, pad_torque_nm = 0;
};

[[nodiscard]] inline RegenHud regen_hud(const ps::drivetrain::PowertrainSnapshot& p) {
    RegenHud h;
    h.fitted = p.electric_fitted && p.electric_motor_count > 0;
    if (!h.fitted) return h;
    h.power_kw = std::max(0.0, static_cast<double>(p.regen_power_w)) / 1000.0;
    h.active = static_cast<double>(p.regen_power_w) >= kRegenActiveMinW;
    h.regen_share = std::clamp(static_cast<double>(p.regen_brake_share), 0.0, 1.0);
    h.pad_share = std::clamp(static_cast<double>(p.pad_brake_share), 0.0, 1.0);
    h.regen_torque_nm = static_cast<double>(p.regen_brake_torque_nm);
    h.pad_torque_nm = static_cast<double>(p.pad_brake_torque_nm);
    return h;
}

// "REGEN 42 kW / 65%" while regenerating, "REGEN 0 kW" otherwise; empty for a car without electric motors.
[[nodiscard]] inline std::string regen_label(const RegenHud& h) {
    if (!h.fitted) return {};
    char buf[48];
    if (h.active) std::snprintf(buf, sizeof buf, "REGEN %.0f kW / %.0f%%", h.power_kw, 100.0 * h.regen_share);
    else std::snprintf(buf, sizeof buf, "REGEN 0 kW");
    return buf;
}

}  // namespace rg
