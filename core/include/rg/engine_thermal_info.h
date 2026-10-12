// rg/engine_thermal_info.h - what a dashboard needs to know about a car's engine temperature (physics_sim engine
// thermal plan T1, vault PHYS-063).
//
// Engine-neutral and header-only, same pattern as rg/nitrous_info.h: the Godot binding (RgSimulation.
// get_vehicle_thermal) and the GDScript gauges only forward and draw these values. This is the plain state getter
// every dashboard, HUD or future cockpit control reads; there is no shared action/state layer for car functions yet.
//
// A car has a thermal model iff its vehicle file names a cooling_system AND its engine has thermal data
// (PowertrainDesc::thermal, EngineThermalSnapshot::has_thermal). A car without one reports present = false and no
// value: the dashboard hides the gauge instead of showing a fake number.
//
// Colour levels (Cold / Normal / Hot / Overheat) are decided here, from the car's own data where it has any:
//   Cold     coolant below the cooling system's warnings.cold_c        (class_passenger_car.json: 50 C)
//   Normal   cold_c .. fan_on_c
//   Hot      coolant at or above the radiator fan's on_c, i.e. the     (class_passenger_car.json: 98 C)
//            thermostat is fully open and the cooling is working at its limit
//   Overheat coolant at or above warnings.overheat_c, or the            (class_passenger_car.json: 118 C)
//            physics warning bit kThermalWarningOverheat
// A threshold that the data leaves at 0 falls back: hot -> thermostat full-open temperature + 5 K [AS],
// overheat -> hot + 20 K [AS], cold -> none (never Cold). Oil has no limits in the data, so oil is a plain number.
#pragma once

#include "ps/drivetrain/engine_thermal_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/vehicle/vehicle_desc.h"

#include <string_view>

namespace rg {

enum class ThermalLevel : int { Cold = 0, Normal = 1, Hot = 2, Overheat = 3 };

[[nodiscard]] constexpr std::string_view thermal_level_name(ThermalLevel level) {
    switch (level) {
        case ThermalLevel::Cold: return "cold";
        case ThermalLevel::Normal: return "normal";
        case ThermalLevel::Hot: return "hot";
        case ThermalLevel::Overheat: return "overheat";
    }
    return "normal";
}

// The static side: thresholds in degrees C, read once from the vehicle description.
struct EngineThermalInfo {
    bool present = false; // the vehicle opted in to the thermal model
    double cold_c = 0.0;     // below this: Cold (0 = never)
    double hot_c = 0.0;      // at or above: Hot
    double overheat_c = 0.0; // at or above: Overheat
    double thermostat_open_c = 0.0, thermostat_full_open_c = 0.0;
    double fan_on_c = 0.0, fan_off_c = 0.0;
};

// Fallback margins for a threshold the data leaves at zero ([AS]).
constexpr double kThermalHotAboveFullOpenK = 5.0;
constexpr double kThermalOverheatAboveHotK = 20.0;

[[nodiscard]] inline EngineThermalInfo engine_thermal_info(const ps::vehicle::VehicleDesc& desc) {
    EngineThermalInfo info;
    if (!desc.powertrain.thermal.has_value()) return info;
    const ps::drivetrain::EngineThermalConfig& cfg = *desc.powertrain.thermal;
    constexpr double kK = ps::drivetrain::kCelsiusToKelvin;
    info.present = true;
    info.thermostat_open_c = static_cast<double>(cfg.engine.thermostat.open_k) - kK;
    info.thermostat_full_open_c = static_cast<double>(cfg.engine.thermostat.full_open_k) - kK;
    info.fan_on_c = static_cast<double>(cfg.cooling.fan.on_k) - kK;
    info.fan_off_c = static_cast<double>(cfg.cooling.fan.off_k) - kK;
    if (cfg.cooling.warnings.cold_k > 0.0) info.cold_c = static_cast<double>(cfg.cooling.warnings.cold_k) - kK;
    if (cfg.cooling.fan.on_k > 0.0) {
        info.hot_c = info.fan_on_c;
    } else if (cfg.engine.thermostat.full_open_k > 0.0) {
        info.hot_c = info.thermostat_full_open_c + kThermalHotAboveFullOpenK;
    }
    if (cfg.cooling.warnings.overheat_k > 0.0) {
        info.overheat_c = static_cast<double>(cfg.cooling.warnings.overheat_k) - kK;
    } else if (info.hot_c > 0.0) {
        info.overheat_c = info.hot_c + kThermalOverheatAboveHotK;
    }
    return info;
}

// The temperature level for the colour. `overheat_warning` is the physics warning bit (it wins).
[[nodiscard]] constexpr ThermalLevel thermal_level(const EngineThermalInfo& info, double coolant_c,
                                                   bool overheat_warning = false) {
    if (overheat_warning || (info.overheat_c > 0.0 && coolant_c >= info.overheat_c)) return ThermalLevel::Overheat;
    if (info.hot_c > 0.0 && coolant_c >= info.hot_c) return ThermalLevel::Hot;
    if (info.cold_c > 0.0 && coolant_c < info.cold_c) return ThermalLevel::Cold;
    return ThermalLevel::Normal;
}

// Everything a dashboard shows for one frame (snapshot values, no physics).
struct EngineThermalHud {
    bool present = false; // false: hide the gauge, nothing else is meaningful
    double coolant_c = 0.0;
    double oil_c = 0.0;
    double gauge = 0.0;               // 0..1 needle position from the car's gauge curve (buffered like a real needle)
    double thermostat_opening = 0.0;  // 0..1
    bool fan_on = false;
    ThermalLevel level = ThermalLevel::Normal;
};

[[nodiscard]] inline EngineThermalHud engine_thermal_hud(const EngineThermalInfo& info,
                                                         const ps::drivetrain::PowertrainSnapshot& p) {
    EngineThermalHud h;
    const ps::drivetrain::EngineThermalSnapshot& t = p.thermal;
    if (!info.present || !t.has_thermal) return h;
    h.present = true;
    h.coolant_c = static_cast<double>(t.coolant_c);
    h.oil_c = static_cast<double>(t.oil_c);
    h.gauge = static_cast<double>(t.gauge);
    h.thermostat_opening = static_cast<double>(t.thermostat_opening);
    h.fan_on = t.fan_on;
    h.level = thermal_level(info, h.coolant_c, (t.warning_bits & ps::drivetrain::kThermalWarningOverheat) != 0u);
    return h;
}

} // namespace rg
