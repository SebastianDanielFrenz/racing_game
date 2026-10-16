// rg/cooling_upgrades.h - cooling system upgrades (owner on PR #21: "More combustion should result in more heat for the
// engine components which should need to be cooled. We need cooling system upgrades (water cooling elements in the engine
// block, bigger radiator, ...)"). Engine-neutral, exception-free, no Godot type.
//
// The physics (physics_sim engine thermal model, docs/data_formats/engine_thermal.md and cooling_system.md) already makes the
// heat follow the fuel burned: the heat into the engine lump is wall_heat_fraction x the RELEASED fuel energy of every
// substep (+ friction), so a turbo kit's extra air and fuel run the coolant hotter at the same rpm (measured in
// tests/unit/test_cooling_upgrades.cpp [.][cooling][long]). What was missing was a way to carry that heat away. The schema's
// levers, and the four parts that use them (data/cooling_systems/cooling_upgrades.json, rg.cooling_upgrades/1):
//   radiator      cooling_system: radiator.frontal_area_m2, ua_ref_w_k, air_mass_flow_ref_kg_s (scaled together, the way
//                 data/cooling_systems/hyper.json scales the class core), ram_coefficient (ducting)
//   fan           cooling_system: fan.face_velocity_m_s, power_w, on_c/off_c (the fan only works below the speed at which
//                 the ram air alone is faster: it helps in a crawl, not at 200 km/h)
//   oil_cooler    cooling_system: oil_cooler.air_w_k_per_m_s (added, per litre of displacement)
//   block_cooling engine_thermal: pump.l_min_per_krpm(_per_l), capacity(_per_l).engine_j_k (more coolant in the block),
//                 thermostat.open_c/full_open_c. The schema has NO water-jacket geometry, passage area or wall heat transfer
//                 coefficient: "water cooling elements in the block" can only be a stronger pump, a larger coolant volume
//                 (a slower rise, no change of the steady state) and a cooler thermostat. wall_heat_fraction is not touched
//                 (it is how much of the fuel energy reaches the coolant, not how well it is carried off).
// A grade edits the car's cooling_system file and/or engine thermal file as a merge patch over the file's EFFECTIVE values
// (the extends chain resolved), so one grade table fits every car; materialise_setup writes the patched copies.
// Mass (owner rule: the car's mass is the sum of its parts): the stock hardware is the "engine.cooling" node of the part-mass
// tree; a grade adds a child node with the mass the NEW part adds over the stock part it replaces ([AS] per grade).
#pragma once

#include "rg/part_mass.h"

#include "ps/drivetrain/engine_thermal_desc.h"

#include <nlohmann/json.hpp>

#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace rg {

struct CoolingGrade {
    std::string id, label, source;
    // radiator
    double area_factor = 1.0;      // frontal area (air flow at the reference point scales with it)
    double core_factor = 1.0;      // UA per area at equal face velocity (a thicker or denser core)
    double ram_factor = 1.0;       // ram_coefficient, capped at 1 (ducting)
    double added_kg_per_m2 = 0.0;  // mass added per m2 of the extra frontal area
    // fan
    double fan_flow_factor = 1.0;
    double fan_power_factor = 1.0;
    double fan_on_offset_k = 0.0;  // switch-on/off temperatures move by this (negative = earlier)
    // oil cooler
    double oil_cooler_w_k_per_m_s_per_l = 0.0; // added to oil_cooler.air_w_k_per_m_s, per litre of displacement
    // block cooling
    double pump_factor = 1.0;
    double extra_coolant_l_per_l = 0.0;  // coolant volume added to the block, litres per litre of displacement
    double thermostat_offset_k = 0.0;    // open_c and full_open_c move by this
    // mass (any part)
    double fixed_kg = 0.0;
    double per_litre_kg = 0.0;           // x displacement_l
};

struct CoolingPart {
    std::string kind;     // "radiator" | "fan" | "oil_cooler" | "block_cooling"
    std::string label;    // "Radiator"
    std::string damage_part; // what a damage model would name when this part is the limit
    std::vector<CoolingGrade> grades; // grades[0] is "stock"
    [[nodiscard]] const CoolingGrade* find(const std::string& id) const;
};

struct CoolingCheckConfig {
    double ambient_c = 30.0;               // the day the check assumes
    double ram_speed_m_s = 30.0;           // ram air when top_speed_m_s_per_cbrt_kw is 0 (about 108 km/h)
    // > 0: the ram air of a rpm point is k x cbrt(power in kW) - the speed that power holds (drag power ~ v^3), i.e. the check
    // asks whether the car can sustain full power at the pace that power gives it, not at one fixed speed.
    double top_speed_m_s_per_cbrt_kw = 0.0;
    double max_rpm_fraction_of_limiter = 0.0; // > 0: sweep points above this share of the rev limiter are not checked
    double power_band = 0.95;              // only sweep points with at least this share of the peak power are checked
    double friction_power_fraction = 0.09; // friction heat as a share of the shaft power
    // The transient (transient_thermal): simulated from the warm spawn state for at most transient_horizon_s at a step of
    // transient_step_s. How soon a limit is reached grades the item (advisor.json thermal_time).
    double transient_horizon_s = 1800.0;
    double transient_step_s = 0.25;
    double oil_limit_c = 135.0;
    std::string source;
};

struct CoolingUpgradeConfig {
    CoolingCheckConfig check;
    std::vector<CoolingPart> parts;
    [[nodiscard]] const CoolingPart* find(const std::string& kind) const;
};

std::optional<CoolingUpgradeConfig> load_cooling_upgrades(const std::string& path, std::string* err);
std::optional<CoolingUpgradeConfig> parse_cooling_upgrades(const std::string& json_text, const std::string& origin, std::string* err);

// A cooling_system / engine thermal file with its "extends" chain merged (parent first, this file last; physics_sim's own rule,
// depth <= 8). nullopt + *err on a missing file, a cycle or non-object.
std::optional<nlohmann::json> load_effective_json(const std::string& path, std::string* err);

// One part as installed on a car.
struct CoolingPartBuild {
    std::string kind, kind_label;
    std::string grade, grade_label;
    std::string damage_part;
    std::string option_id;      // the garage option that swaps it
    double extra_kg = 0.0;      // mass over the stock part
    std::string detail;         // "0.35 -> 0.47 m2 core, UA x1.55 · +1.0 kg"
};

struct CoolingBuild {
    std::vector<CoolingPartBuild> parts; // every kind of the config, in config order (stock where not chosen)
    double displacement_l = 0.0;
    [[nodiscard]] double mass_delta_kg() const;
};

// The effective docs the grades work on (from load_effective_json), and what they produce: merge patches for the
// cooling_system file (`cooling`) and the engine thermal file (`thermal`), each {} when nothing changes.
struct CoolingEdit {
    nlohmann::json cooling = nlohmann::json::object();
    nlohmann::json thermal = nlohmann::json::object();
};

// `grades`: kind -> grade id ("stock" when absent). `options`: kind -> the garage option that swaps it. nullopt + *err on an
// unknown grade or a file the grade cannot work on (no displacement for a per-litre grade, ...). `displacement_hint_l` is used
// when the thermal file carries none.
std::optional<CoolingBuild> build_cooling(const CoolingUpgradeConfig& config, const std::map<std::string, std::string>& grades,
                                          const std::map<std::string, std::string>& options, const nlohmann::json& cooling_effective,
                                          const nlohmann::json& thermal_effective, double displacement_hint_l, CoolingEdit* edit,
                                          std::string* err);

// The card line of ONE grade on this car ("" on an unknown kind/grade).
std::string cooling_grade_detail(const CoolingUpgradeConfig& config, const std::string& kind, const std::string& grade,
                                 const nlohmann::json& cooling_effective, const nlohmann::json& thermal_effective,
                                 double displacement_hint_l);

// The part-mass tree entries: one child of "engine.cooling" per part that differs from stock (own mass = extra_kg).
std::vector<PartAttachment> cooling_parts(const CoolingBuild& build);

// ---- steady-state temperatures (the setup check) -------------------------------------------------------------------------

struct SteadyThermal {
    double coolant_c = 0.0, oil_c = 0.0;
    double radiator_w = 0.0;
    bool fan_on = false;
    double thermostat_opening = 0.0;
};
// The steady state of the physics model (physics_sim core/src/drivetrain/engine_thermal.cpp: two lumps, eps-NTU cross-flow
// radiator, thermostat, fan hysteresis) at a constant rpm, heat input and ram air speed: the temperatures it settles at, not a
// time history. `wall_heat_w` = wall_heat_fraction x the fuel power, `friction_heat_w` the friction power.
SteadyThermal steady_thermal(const ps::drivetrain::EngineThermalConfig& config, double engine_rpm, double wall_heat_w,
                             double friction_heat_w, double ambient_c, double ram_speed_m_s);

// ---- time to the limit (the setup check, owner answer on PR #28: option C) ------------------------------------------------

struct ThermalTransient {
    // Seconds of constant full power until the coolant / oil first reaches its limit; NaN = not within the horizon (or the
    // temperatures settle below it first).
    double coolant_limit_s = std::numeric_limits<double>::quiet_NaN();
    double oil_limit_s = std::numeric_limits<double>::quiet_NaN();
    double end_coolant_c = 0.0, end_oil_c = 0.0; // where the run stopped
};
// The physics model's own time history (physics_sim engine_thermal.cpp step(): the two lumps with their heat capacities,
// backward Euler, thermostat wax lag + play, fan hysteresis, eps-NTU radiator) at a constant rpm, heat input and ram air, started
// from the warm spawn state (coolant a quarter into the thermostat band, oil at its reference temperature, fan off). Stops when
// both limits are reached, when both lumps have settled, or after `horizon_s`.
ThermalTransient transient_thermal(const ps::drivetrain::EngineThermalConfig& config, double engine_rpm, double wall_heat_w,
                                   double friction_heat_w, double ambient_c, double ram_speed_m_s, double coolant_limit_c,
                                   double oil_limit_c, double horizon_s, double step_s);

} // namespace rg
