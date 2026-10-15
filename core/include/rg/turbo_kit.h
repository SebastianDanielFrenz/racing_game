// rg/turbo_kit.h - a turbo system for an engine that has none (garage "Turbo kit", owner request on PR #5:
// "Turbos cannot be fitted yet to many engines").
//
// No per-engine turbo files. One rule sizes the hardware from the engine's own air demand, the same similarity method
// physics_sim uses for its turbo hardware family (docs/data_formats/turbo.md "Hardware family and templates"):
//   * demand: charge density (ambient compressed to the kit's target, compressor heating at the anchor's efficiency,
//     intercooler effectiveness) x swept volume x limiter rpm / 120 x volumetric efficiency, split over the kit's turbos;
//   * required compressor flow per turbo = demand x flow_margin (the compressor's max flow above the engine's peak need,
//     so it neither chokes at the limiter nor spools as late as an oversized unit);
//   * hardware = the anchor (physics_sim data/turbos/hyper_turbo_approx.json, GTX3582R-class) scaled at constant tip
//     speed: s = sqrt(flow / anchor flow) is the wheel-diameter ratio; flow, turbine throat, wastegate port and shaft loss
//     x s^2, reference rpm / s, rotor inertia x s^5, efficiencies anchor + log slope x ln(s) (small wheels lose more);
//     the reference head is unchanged (same tip speed, same stage).
// The conversion around it is what physics_sim's engine.md "Adding or removing forced induction" asks for: the
// manifold-pressure axis becomes ps::drivetrain::boost_grid_p_im(cycle, target) and the compression ratio is lowered to
// the kit's ceiling where it is above it. Knock, lambda and spark are the engine's own ECU and the physics' knock model.
//
// Which engines: simulated engines with a cycle block and no turbo of their own, with 1 to 4 turbos whatever the cylinder
// count (a quad on a 1.0 L three is allowed when it fits the bay: turbo_kit_envelope_mm + check_fit). Torque-curve engines
// have no air path (refusal text from turbo_kit_refusal, shown by the garage instead of hiding the option).
// More turbos on the same engine = each gets 1/N of the air: a smaller wheel (s ~ 1/sqrt(N)), a higher reference speed and
// s^5 less rotor inertia (quicker spool), but a lower efficiency (log rule) and a choke flow per unit only `flow_margin`
// above its share; the hardware is clamped to [min_scale, max_scale] (a quad on a small engine hits min_scale: oversized).
//
// The generated configuration and hardware are written (atomically, content-addressed) to <cache dir>/<cache_subdir>/;
// the same engine and kit always give the same files. Engine-neutral, exception-free.
#pragma once

#include "rg/engine_store.h"
#include "rg/part_mass.h"

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace rg {

struct TurboKitSpec {
    std::string id;          // lower_snake, unique ("sport_single")
    std::string label;       // "Sport turbo kit"
    int count = 1;           // turbos, 1 to 4 on any engine: a shortcut for a count and a starting boost; the boost slider
                             // (turbo_boost_target) then moves the target freely and the kit is re-sized for it
    double target_boost_bar = 0.8; // gauge
};

// Charge piping (compressor -> intercooler -> throttle) material: a separate part the player chooses. The rating is what
// the garage's limit advisor holds the boost against (warning above `rated_bar`, critical above `burst_bar`); nothing is
// simulated to fail yet (docs/garage_ui.md "Damage model data").
struct ChargePipingSpec {
    std::string id;              // "plastic", "aluminium", "steel"
    std::string label;
    double rated_bar = 1.5;      // continuous gauge pressure the piping and its couplers are rated for
    double burst_bar = 3.0;      // gauge pressure at which a pipe or coupler lets go
    double wall_mm = 3.0;
    double density_kg_m3 = 1350.0;
    std::string source;          // [AS] reasoning for the numbers
};

struct TurboKitConfig {
    std::string anchor_hardware;        // absolute path of the physics_sim.turbo/1 file the sizes are scaled from
    double anchor_exducer_mm = 82.0;    // the anchor's compressor exducer
    std::string cache_subdir = "turbo_kits";
    // sizing
    double volumetric_efficiency = 0.9;
    double flow_margin = 1.4;
    double ambient_pressure_kpa = 101.325;
    double ambient_temperature_k = 288.15;
    double efficiency_log_slope = 0.127; // efficiency = anchor + slope * ln(s), clamped to [efficiency_min, anchor]
    double efficiency_min = 0.70;
    double wastegate_to_throat = 3.0;
    double min_scale = 0.3, max_scale = 1.6; // the wheel-diameter ratio range the rule may produce
    int max_count = 4;
    // conversion
    double max_compression_ratio = 10.0;
    double intercooler_effectiveness = 0.75;
    bool compressible_exhaust = true;
    double kp = 1.0, ki_per_s = 1.0, kd_s = 0.05, actuator_tau_s = 0.1;
    // envelope of one turbo unit from its compressor exducer D: length along the shaft = length_per_exducer x D +
    // length_add_mm, outer diameter = diameter_per_exducer x D; units sit in a row along the crank on the exhaust side
    // (a V engine: split over both banks), `gap_mm` apart; the charge pipe (pipe_velocity_m_s at charge density) sets
    // the piping mass.
    double unit_length_per_exducer = 2.6, unit_length_add_mm = 60.0, unit_diameter_per_exducer = 2.2, unit_gap_mm = 20.0;
    // share of a unit's outer size that stands out beside the engine's own envelope (the turbo takes the place of the NA
    // exhaust manifold, which the engine envelope already holds); the charge pipe is routed through the gaps to the
    // front-mounted intercooler and is not part of the box
    double unit_side_protrusion = 0.5;
    double pipe_velocity_m_s = 60.0;
    // masses ([AS], reasoning in the file): turbo unit = fixed + scaled x s^3; exhaust manifold / downpipe per unit;
    // front-mounted intercooler core area per kg/s of total flow x depth x effective density; charge pipe length.
    double unit_mass_fixed_kg = 2.0, unit_mass_scaled_kg = 7.5, hot_side_mass_per_unit_kg = 3.0;
    double intercooler_area_m2_per_kg_s = 0.45, intercooler_depth_m = 0.076, intercooler_density_kg_m3 = 650.0;
    double piping_length_base_m = 1.8, piping_length_per_extra_unit_m = 0.6;
    // damage-model data: the rotor's rated maximum shaft speed = the scaled reference speed x this
    double max_shaft_speed_factor = 1.15;
    std::vector<TurboKitSpec> kits;
    std::vector<ChargePipingSpec> piping;
    std::string default_piping;         // the piping a kit comes with ("plastic")
    [[nodiscard]] const TurboKitSpec* find(const std::string& id) const;
    [[nodiscard]] const ChargePipingSpec* find_piping(const std::string& id) const;
};

// data/engines/turbo_kits.json (rg.turbo_kits/1); `repo_root` resolves "anchor_hardware". Strict, exception-free.
std::optional<TurboKitConfig> load_turbo_kit_config(const std::string& path, const std::string& repo_root, std::string* err);

// "" when a turbo kit can be fitted to this engine, else the reason in the player's words. `name` is the engine's name.
std::string turbo_kit_refusal(const EngineFacts& facts, const std::string& name);

// The rule's result for one engine and kit (pure; no file is read or written).
struct TurboKitSizing {
    bool ok = false;
    std::string error;
    double demand_kg_s = 0.0;          // whole engine at the limiter and the kit's target
    double flow_per_turbo_kg_s = 0.0;  // required compressor max flow per turbo (demand / count x margin)
    double scale = 1.0;                // wheel-diameter ratio to the anchor
    double exducer_mm = 0.0;
    double reference_rpm = 0.0;
    double max_mass_flow_kg_s = 0.0;   // per turbo
    double rotor_inertia_kgm2 = 0.0;   // per turbo
    double turbine_area_m2 = 0.0;
    double wastegate_area_m2 = 0.0;
    double shaft_loss_w = 0.0;
    double compressor_efficiency = 0.0, turbine_efficiency = 0.0;
    double reference_head_bar = 0.0;
    double compression_ratio = 0.0;    // after the conversion (unchanged when already at or under the ceiling)
    double torque_factor = 1.0;        // estimated WOT torque ratio boosted / NA: the charge pressure ratio (A25A: 1.79 at
                                       // 0.8 bar against 1.78-1.81 measured, physics_sim engine.md)
    double max_shaft_rpm = 0.0;        // damage-model data: the rotor's rated maximum (reference rpm x max_shaft_speed_factor)
    // envelope
    double unit_length_mm = 0.0, unit_diameter_mm = 0.0, pipe_diameter_mm = 0.0;
    // masses
    double unit_mass_kg = 0.0;         // one turbo unit
    double hot_side_mass_kg = 0.0;     // all units' manifolds / downpipes
    double intercooler_area_m2 = 0.0, intercooler_mass_kg = 0.0;
};
TurboKitSizing size_turbo_kit(const TurboKitConfig& config, const TurboKitSpec& spec, const EngineFacts& facts, double limiter_rpm);

// One card line: "2 x 36/45 mm compressor · 0.10 kg/s per unit · 1.0 bar".
std::string turbo_kit_detail(const TurboKitConfig& config, const TurboKitSpec& spec, const TurboKitSizing& sizing);

// The engine with the kit fitted, as a box (mm, x along the crank): the units in a row along the exhaust side (a V
// engine: split over both banks), each side wider by unit_side_protrusion x the unit's outer size. `engine_mm` = the engine's own envelope, `v_layout` = a
// V engine (rg.engine_envelope/1 "layout": "v").
std::array<double, 3> turbo_kit_envelope_mm(const TurboKitConfig& config, int count, const TurboKitSizing& sizing,
                                            const std::array<double, 3>& engine_mm, bool v_layout);

// One part of a fitted kit, for the part-mass tree and the future damage model: mass and the limits it is held to.
struct TurboKitPart {
    std::string id;     // "turbo_unit", "hot_side", "intercooler", "charge_piping"
    std::string label;
    int count = 1;
    double mass_kg = 0.0; // each
    std::string mass_source;
    std::map<std::string, double> limits; // "max_shaft_rpm", "rated_bar", "burst_bar" ...
};
// What a fitted kit is: the choice, its sizing and its parts.
struct TurboKitBuild {
    TurboKitSpec spec;            // count and the FINAL target (after the boost slider)
    TurboKitSizing sizing;
    ChargePipingSpec piping;
    double piping_length_m = 0.0;
    std::array<double, 3> envelope_mm{0.0, 0.0, 0.0}; // engine + kit; zeros when the engine has no envelope
    std::vector<TurboKitPart> parts;
    [[nodiscard]] double total_mass_kg() const;
};
TurboKitBuild describe_turbo_kit(const TurboKitConfig& config, const TurboKitSpec& spec, const TurboKitSizing& sizing,
                                 const ChargePipingSpec& piping);

// The kit's parts for the car's part-mass tree: one "turbo_kit" node (own mass 0) under "engine" with one child per
// part of the build (each turbo, each exhaust side, the intercooler, the charge piping), masses and [AS] sources from
// describe_turbo_kit. The drive's chassis mass follows the kit through the tree.
PartAttachment turbo_kit_parts(const TurboKitBuild& build);

// Writes the sized configuration (+ its hardware file) under <cache_dir>/<config.cache_subdir>/ unless the same content
// is there already. Returns the configuration's absolute path, "" + *err on failure.
std::string write_turbo_kit(const TurboKitConfig& config, const TurboKitSpec& spec, const TurboKitSizing& sizing,
                            const std::string& engine_name, const std::string& cache_dir, std::string* err);


// The engine-file edits of the conversion: grid_p_im_kpa for the kit target (kPa), nullopt + *err if the physics helper
// refuses the engine's axis.
std::optional<std::vector<double>> turbo_kit_grid_kpa(const EngineFacts& facts, double target_boost_bar, std::string* err);

} // namespace rg
