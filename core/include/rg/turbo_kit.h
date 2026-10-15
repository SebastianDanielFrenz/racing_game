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
// Which engines: simulated engines with a cycle block and no turbo of their own. Torque-curve engines have no air path
// (refusal text from turbo_kit_refusal, shown by the garage instead of hiding the option).
//
// The generated configuration and hardware are written (atomically, content-addressed) to <cache dir>/<cache_subdir>/;
// the same engine and kit always give the same files. Engine-neutral, exception-free.
#pragma once

#include "rg/engine_store.h"
#include "rg/part_mass.h"

#include <optional>
#include <string>
#include <vector>

namespace rg {

struct TurboKitSpec {
    std::string id;          // lower_snake, unique ("street_single")
    std::string label;       // "Street turbo kit"
    int count = 1;           // turbos (1 or 2)
    double target_boost_bar = 0.8; // gauge
    int min_cylinders = 1;   // a twin needs this many cylinders (one turbo per bank)
};

struct TurboKitConfig {
    std::string anchor_hardware;        // absolute path of the physics_sim.turbo/1 file the sizes are scaled from
    double anchor_exducer_mm = 82.0;    // the anchor's compressor exducer (only for the card text)
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
    // conversion
    double max_compression_ratio = 10.0;
    double intercooler_effectiveness = 0.75;
    bool compressible_exhaust = true;
    double kp = 1.0, ki_per_s = 1.0, kd_s = 0.05, actuator_tau_s = 0.1;
    // part masses (rg/part_mass.h): a kit's turbos, exhaust manifolds and charge-air side, all [AS] in turbo_kits.json
    double turbo_anchor_kg = 11.0;       // one anchor-size turbo with its housings
    double turbo_scale_exponent = 2.0;   // turbo mass = anchor x scale^exponent, at least turbo_min_kg
    double turbo_min_kg = 5.0;
    double manifold_kg_per_turbo = 8.0;  // exhaust manifold, downpipe, wastegate, oil and water lines
    double intercooler_kg = 10.0;        // intercooler and charge piping for a single
    double intercooler_kg_per_extra_turbo = 2.0;
    std::vector<TurboKitSpec> kits;
    [[nodiscard]] const TurboKitSpec* find(const std::string& id) const;
};

// data/engines/turbo_kits.json (rg.turbo_kits/1); `repo_root` resolves "anchor_hardware". Strict, exception-free.
std::optional<TurboKitConfig> load_turbo_kit_config(const std::string& path, const std::string& repo_root, std::string* err);

// "" when a turbo kit can be fitted to this engine, else the reason in the player's words. `name` is the engine's name.
std::string turbo_kit_refusal(const EngineFacts& facts, const std::string& name);
// "" when this kit suits the engine (cylinder count for a twin), else why not.
std::string turbo_kit_spec_refusal(const TurboKitSpec& spec, const EngineFacts& facts);

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
};
TurboKitSizing size_turbo_kit(const TurboKitConfig& config, const TurboKitSpec& spec, const EngineFacts& facts, double limiter_rpm);

// One card line: "1 x 41/51 mm · 0.12 kg/s · 0.8 bar".
std::string turbo_kit_detail(const TurboKitConfig& config, const TurboKitSpec& spec, const TurboKitSizing& sizing);

// Writes the sized configuration (+ its hardware file) under <cache_dir>/<config.cache_subdir>/ unless the same content
// is there already. Returns the configuration's absolute path, "" + *err on failure.
std::string write_turbo_kit(const TurboKitConfig& config, const TurboKitSpec& spec, const TurboKitSizing& sizing,
                            const std::string& engine_name, const std::string& cache_dir, std::string* err);

// The kit's parts for the car's part-mass tree: one "turbo_kit" node (own mass 0) under "engine" with the turbo(s),
// the exhaust side and the charge-air side as children. Turbo mass follows the sized wheel (sizing.scale).
PartAttachment turbo_kit_parts(const TurboKitConfig& config, const TurboKitSpec& spec, const TurboKitSizing& sizing);

// The engine-file edits of the conversion: grid_p_im_kpa for the kit target (kPa), nullopt + *err if the physics helper
// refuses the engine's axis.
std::optional<std::vector<double>> turbo_kit_grid_kpa(const EngineFacts& facts, double target_boost_bar, std::string* err);

} // namespace rg
