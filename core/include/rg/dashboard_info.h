// rg/dashboard_info.h - what the dashboards (HUD tach gauge, digital cluster, cockpit instruments) show for a car with a
// traction battery and electric motors: the battery charge readout and the tachometer layout (owner 2026-10-18).
// Engine-neutral: the Godot binding forwards these values and the GDScript only draws them.
//
//   * battery_hud: SoC in whole percent, a text label and a level (0 ok, 1 low, 2 critical) for the small bar's colour.
//     The two thresholds are config values (data/controls/dashboard.json).
//   * tach_scales: which dials a car gets. An ICE car is unchanged; a pure EV gets one motor-rpm dial; a hybrid gets the
//     ICE dial (0..limiter, the main needle) plus an INNER secondary scale for the traction motor. The inner scale's
//     maximum is the traction motor's max_rpm from the car's own description, rounded up to the config's tick step -
//     nothing is hard-coded (the hyper conversion's 20000 rpm motor gives 0..20 x1000, ticks every 2).
//   * traction_motor_rpm: the traction motor's speed. The snapshot's motor_rpm is the MAX over every machine, so on a
//     hybrid it is the crank generator that follows the engine; the traction motor sits behind a fixed reduction on the
//     gearbox output shaft, so its speed is gearbox_out_omega x that ratio (found in the car's shaft graph).
#pragma once

#include "ps/drivetrain/powertrain_state.h"
#include "ps/vehicle/vehicle_desc.h"

#include <optional>
#include <string>

namespace rg {

struct DashboardConfig {
    double battery_low_soc = 0.20;        // below this the bar turns amber
    double battery_critical_soc = 0.10;   // below this it turns red
    double motor_dial_tick_krpm = 2.0;    // inner scale: one labelled tick every this many x1000 rpm
    double motor_dial_max_krpm = 0.0;     // > 0 overrides the max read from the motor description
};

// Parses data/controls/dashboard.json text. Unknown keys are ignored; a wrong type or out-of-range value is refused
// (nullopt + *err). An empty object gives the defaults.
[[nodiscard]] std::optional<DashboardConfig> parse_dashboard_config(const std::string& text, std::string* err = nullptr);

struct BatteryHud {
    bool fitted = false;     // the car has a traction battery with a capacity
    double soc = 0;          // 0..1
    int percent = 0;         // rounded, 0..100
    int level = 0;           // 0 ok, 1 low (< battery_low_soc), 2 critical (< battery_critical_soc)
    std::string label;       // "SoC 25%", empty when not fitted
};

[[nodiscard]] BatteryHud battery_hud(const ps::drivetrain::PowertrainSnapshot& p, const DashboardConfig& cfg = {});

enum class TachKind { Ice, Ev, Hybrid };

struct TachScales {
    TachKind kind = TachKind::Ice;
    double ice_limiter_rpm = 0;      // the combustion engine's limiter (0 without an engine)
    double motor_limit_rpm = 0;      // the traction motor's max_rpm (0 without a traction motor)
    double motor_dial_max_rpm = 0;   // the inner/EV scale's last tick (rounded up to the tick step)
    double motor_dial_tick_rpm = 0;  // spacing of the inner/EV scale's ticks
    double traction_ratio = 0;       // traction motor shaft rpm per gearbox-output rpm (0 = unknown)
};

[[nodiscard]] TachScales tach_scales(const ps::vehicle::VehicleDesc& desc, const DashboardConfig& cfg = {});

// The traction motor's speed in rpm: gearbox_out_omega x ratio when the ratio is known, else the snapshot's motor_rpm.
[[nodiscard]] double traction_motor_rpm(const ps::drivetrain::PowertrainSnapshot& p, const TachScales& scales);

}  // namespace rg
