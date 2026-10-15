// rg/engine_swap.h - what an engine swap changes in a car (engine garage G1, out/engine_garage_design.md 4.4, D1, D2).
// Engine-neutral, plain data in, plain data out: vehicle_setup.cpp applies the result to the working vehicle file.
//
// derive_swap_package() is the one place the swap rules live:
//   * Fuel (D1): the tank's fuel is kept if the new engine lists it, else it becomes the engine's reference fuel
//     (fuels[0]) with a notice; a fuel the player picked must be one of the engine's.
//   * Clutch: kinetic capacity = 1.5 x the new engine's peak torque when the current one is lower (a stronger clutch
//     stays); static_factor is the car's. A turbo kit (SwapChoices::turbo_kit) multiplies the peak torque by its
//     estimated boost torque factor and applies the clutch rule even without an engine swap.
//   * Auto-clutch (TCU): bite = idle + 300, anti-stall = idle - 100, rolling re-engage = idle + 500, full range from
//     bite + 500 to max(that, 0.45 x limiter) [AS] - the physics loader's ordering rules hold by construction.
//   * Auto-shift: up at full throttle = limiter - 300, at zero throttle 0.45 x limiter, down 0.19 / 0.41 x limiter
//     (never below idle + 300 / 500 above that) [AS].
//   * Auto-blip: zeroed on a mechanical-throttle engine.
//   * Engine thermal model: a car with a cooling_system keeps it only if the new engine has thermal data (default_thermal);
//     the physics loader throws otherwise, so without it the cooling_system (and engine_thermal_start) is dropped and the
//     car runs without a thermal model (the dashboard gauge hides).
//   * Slot overrides on the engine component (forced induction, installation, thermal, ecu, cam) belong to the old
//     engine and are dropped; a nitrous kit stays on a simulated engine and is removed (with a notice) on a torque map.
//   * Chassis mass: + (new engine mass - the car's own engine mass), the catalog figures (until physics models the
//     centre of mass, request R5).
#pragma once

#include "rg/engine_store.h"
#include "rg/vehicle_catalog.h"

#include <string>
#include <vector>

namespace rg {

// What the car has now (read from its working vehicle file).
struct SwapCar {
    bool has_tank = false;
    std::string tank_fuel;               // the fuel file's stem ("e5"); "" without a tank
    double clutch_kinetic_nm = 0.0;      // 0 = no clutch component
    bool has_nitrous = false;            // an n2o_bottle / nitrous_kit component is fitted
    bool has_slot_overrides = false;     // the engine component carries forced_induction / installation / thermal / ecu / cam
    bool has_cooling_system = false;     // the vehicle file names a cooling_system (the car has the engine thermal model)
    double stock_engine_mass_kg = 0.0;   // the car's own engine (catalog figure; 0 = unknown)
};

// The engine going in (from the store and the physics loader).
struct SwapEngine {
    std::string name;
    double mass_kg = 0.0;                // 0 = unknown
    double peak_torque_nm = 0.0;         // 0 = unknown
    double idle_rpm = 0.0;
    double limiter_rpm = 0.0;
    bool simulated = false;
    bool mechanical_throttle = false;
    bool has_thermal = true;             // the engine file (with its extends chain) names a default_thermal
    std::vector<std::string> fuels;      // fuel stems in the engine's list order; [0] = reference fuel
};

struct SwapChoices {
    std::string fuel; // the fuel the player picked ("" = the rule above)
    // A garage turbo kit on an engine that had none (rg/turbo_kit.h): its label and the estimated WOT torque ratio
    // boosted / NA. The clutch rule then sizes for peak torque x factor, with or without an engine swap.
    std::string turbo_kit;
    double boost_torque_factor = 1.0;
};

struct SwapAutoClutch {
    double bite_rpm = 0, full_rpm_min = 0, full_rpm_max = 0, anti_stall_rpm = 0, rolling_reengage_rpm = 0;
};
struct SwapAutoShift {
    double up_zero = 0, up_full = 0, down_zero = 0, down_full = 0;
};

struct SwapPackage {
    bool ok = true;
    std::string error;                  // a picked fuel the engine does not list
    bool engine_swapped = false;
    // Fuel
    bool set_fuel = false;              // edit the tank's fuel
    std::string fuel;                   // the stem the tank holds afterwards ("" without a tank)
    bool fuel_changed = false;          // differs from the tank's current fuel
    // Clutch
    bool set_clutch = false;
    double clutch_kinetic_nm = 0.0;
    // TCU
    bool set_tcu = false;
    SwapAutoClutch auto_clutch;
    SwapAutoShift auto_shift;
    bool zero_blip = false;
    // Parts
    bool drop_nitrous = false;
    bool drop_slot_overrides = false;
    bool drop_cooling_system = false;   // the new engine has no thermal data: the car loses cooling_system + engine_thermal_start
    // Mass
    double mass_delta_kg = 0.0;
    // For the player, one line per item that changes (and notices).
    std::vector<std::string> lines;
};

SwapPackage derive_swap_package(const SwapCar& car, const SwapEngine& engine, const SwapChoices& choices, bool engine_swapped);

// "e10" -> "E10", "tier3_e10_cert" -> "Tier3 E10 cert", "gasoline" -> "Gasoline".
std::string fuel_label(const std::string& stem);

// The fit seam (design 4.4, new cars plan Q10): the engine's generated envelope (EngineEntry::envelope_mm, x along
// the crank) against the car's bay (CatalogEntry::engine_bay_envelope), crank along the bay's x or y. A car without a
// measured bay, or an engine without an envelope, fits (nothing to compare). Reserved: mount pattern, driveline.
struct FitResult {
    bool allowed = true;
    std::vector<std::string> reasons; // empty when allowed
};
FitResult check_fit(const CatalogEntry& car, const EngineEntry& engine);

} // namespace rg
