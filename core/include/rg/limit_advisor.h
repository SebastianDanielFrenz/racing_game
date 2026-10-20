// rg/limit_advisor.h - the garage limit advisor (owner 2026-10-13: "Allow stupid turbo setups, show
// recommendations/warnings"). Engine-neutral.
//
// The garage never clamps a setup value. Instead the PENDING setup is materialised exactly like a drive would
// (rg::materialise_setup + rg::load_vehicle), its engine is swept on physics_sim's dyno (run_dyno_sweep: the
// real runtime engine, turbo and wastegate included) and the LimitFindings the sweep measured are turned into
// UI-ready advice items:
//
//     *Excessive turbo boost target*          <- title
//     Boost target is set to 15.5 bar.        <- the configured value
//     The current setup achieved              <- what the setup actually achieved ...
//     3.4 bar @ 8000 RPM.                     <- ... and where
//     Bigger turbos or a lower boost target.  <- an optional suggestion
//
// Layers (each testable on its own):
//   * AdvisorConfig (data/garage/advisor.json, rg.garage_advisor/1): everything the GAME decides - the debounce, the
//     sweep grid, the merge gap, the text templates and the few kinds the game hides (RevLimiter, FuelCut, MapGridEdge
//     below Exceeded). Nothing is hard-coded (owner rule: refusal limits, advice thresholds and the like are config
//     values). WHICH findings are worth telling at all (Near / Limiting / Exceeded) and every physics threshold is
//     physics_sim's limit policy (physics_sim.limit_policy/1, L0): data/garage/limit_policy.json extends physics'
//     default, an optional <user dir>/limit_policy.json is merged last; AdvisorConfig::policy.
//   * build_advice(): a PURE function, findings + sweep points -> items (group per kind, merge contiguous rpm
//     ranges, worst severity first, render the templates). No threads, no engine.
//   * run_advice_sweep(): loaded VehicleDesc -> findings (dyno sweep, driveline check, nitrous kit when the car has one).
//   * LimitAdvisor: the background thread (debounced, cancellable, progress, on-demand "Find fix" through
//     physics_sim's solve_remedy). The Godot layer only starts, polls and shows.
#pragma once

#include "rg/cooling_upgrades.h"
#include "rg/turbo_map_extension.h"
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_rules.h"
#include "ps/drivetrain/limit_findings.h"
#include "ps/drivetrain/limit_policy.h"
#include "ps/drivetrain/limit_remedies.h"
#include "ps/vehicle/vehicle_desc.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace rg {

// ---- configuration (data/garage/advisor.json) -------------------------------------------------------------------------

// The texts of one advice kind. Templates use {name} placeholders (known_advice_fact_names()); a template that
// renders to nothing is dropped (an empty suggestion = no suggestion line).
struct AdviceText {
    std::string title;
    std::vector<std::string> lines; // configured value, what was achieved, where
    std::string suggestion;
    std::string remedy;        // the line of a solved (or Closed) remedy
    std::string remedy_failed; // a Simulated remedy that found no value
    // A variant only: the garage option this variant is fixed with when it differs from the kind's (ClutchCapacity on a
    // clutch-to-clutch gearbox: "shift_clutch_capacity" instead of the manual clutch's "clutch_capacity"). "" = the kind's.
    std::string option;
    // Caps the item's severity (the item is never louder than this); nullopt = no cap. Variants only in practice.
    std::optional<ps::drivetrain::LimitSeverity> max_severity;
};

struct AdviceKindConfig {
    ps::drivetrain::LimitKind kind = ps::drivetrain::LimitKind::WastegateSaturated;
    // What the GAME adds on top of the limit policy (AdvisorConfig::policy.warns): `hidden` = the kind is never shown
    // ("min_severity": "never" in the file); `min_severity` = a kind-specific floor above the policy's (absent = the
    // policy alone decides).
    bool hidden = false;
    std::optional<ps::drivetrain::LimitSeverity> min_severity;
    std::string option_id; // the garage option that changes it ("" = none); the UI may point at it
    std::string rule_name; // build_rules entries only: the physics BuildRule name ("TurboStageMapCoverage"); kind is unused there
    AdviceText text;
    std::map<std::string, AdviceText> variants; // fields left empty fall back to `text`
};

struct AdvisorConfig {
    int debounce_ms = 300;
    struct Sweep {
        std::string quality = "sketch"; // sketch | preview | full
        double rpm_step = 250.0;
        double rpm_min = 2000.0;
        double limiter_margin_rpm = 50.0;
        bool nitrous_with_kit = true;
    } sweep;
    double merge_gap_rpm = 300.0;
    std::map<std::string, int> decimals; // quantity name -> decimals (missing = the built-in default)
    std::vector<std::string> solve_points = {"worst", "last"};
    std::vector<AdviceKindConfig> kinds; // the "kinds" key order of the FILE (re-read with ordered_json; plain json sorts keys) = the order of equal-severity items
    // Static build-rule issues (ps::drivetrain::BuildIssue TurboStageMapCoverage, and the game's part limits above) shown after the sweep findings of
    // equal severity; same text/variant/min_severity vocabulary as `kinds`. The variant "beyond_map" is used for Limiting.
    std::vector<AdviceKindConfig> build_rules;
    // Engine-map loader warnings (EngineMapOptions::warnings_out) passed through as lines of ONE advice item
    // (`loader_notice.text.lines` rendered once per warning with {message}); a warning containing any of
    // `loader_ignore` is dropped (the stage-map advisory is already a build rule above).
    std::optional<AdviceKindConfig> loader_notice;
    std::vector<std::string> loader_ignore;
    // The limit policy (physics_sim.limit_policy/1): which severities are worth telling (warns), which a garage would
    // refuse (refuses), and every physics threshold. A default-constructed policy is physics' compiled default.
    // `limit_policy` in the file names the policy file (relative to the advisor file); load_advisor_config loads it.
    std::string policy_file;
    ps::drivetrain::LimitPolicy policy;
    // Part limits (damage-model data, docs/garage_ui.md "Damage model data"): the game-side build rules ChargePipingPressure,
    // TurboShaftSpeed, MeanPistonSpeed and ValveFloat. A value at `part_near_fraction` x its rating or above is Near, above
    // the rating Limiting, at a burst figure (piping) Exceeded. The piping and turbo figures come with the parts
    // (rg/turbo_kit.h), the piston-speed and valve-float figures with the engine internals (rg/engine_internals.h: the
    // weakest installed part). `max_mean_piston_speed_m_s` ("part_limits" in the file) is the engine-wide fallback for an
    // engine without internals data.
    double part_near_fraction = 0.9;
    double max_mean_piston_speed_m_s = 25.0;
    // "thermal_time" (owner answer on PR #28): a coolant / oil limit reached at constant full power within this many seconds
    // of the warm spawn state is critical, later a warning, never = clean.
    double thermal_critical_within_s = 60.0;

    [[nodiscard]] const AdviceKindConfig* find(ps::drivetrain::LimitKind kind) const;
    [[nodiscard]] const AdviceKindConfig* find_rule(const std::string& rule_name) const;
    [[nodiscard]] int decimals_for(const std::string& quantity, int fallback) const;
};

// nullopt + *err (origin-prefixed) on a malformed file, an unknown kind, an unknown {placeholder}, a bad number.
std::optional<AdvisorConfig> parse_advisor_config(const std::string& json_text, const std::string& origin, std::string* err);
// Loads the file; `limit_policy` (when given) is loaded from the path next to it, `user_policy_path` (empty = none) is
// the user layer merged over the policy chain. Either failing is an error (origin-prefixed) and returns nullopt.
std::optional<AdvisorConfig> load_advisor_config(const std::string& path, std::string* err, const std::string& user_policy_path = {});
// <user_dir>/limit_policy.json when it exists (the player's own limit policy layer), else "". user_dir "" = "".
std::string user_limit_policy_path(const std::string& user_dir);

// Every {placeholder} a template may use.
const std::vector<std::string>& known_advice_fact_names();
// {name} -> facts[name]; an unknown name renders "?" and is appended to *missing (when given).
std::string render_advice_template(const std::string& tpl, const std::map<std::string, std::string>& facts,
                                   std::vector<std::string>* missing = nullptr);

// ---- advice items -----------------------------------------------------------------------------------------------------

// The UI vocabulary of ps LimitSeverity: Near = advice (neutral colour), Limiting = warning (amber), Exceeded = critical (red).
enum class AdviceSeverity : int { Advice = 0, Warning = 1, Critical = 2 };
const char* to_string(AdviceSeverity s);
AdviceSeverity advice_severity(ps::drivetrain::LimitSeverity s);

enum class RemedyState : int {
    None = 0,      // no remedy exists
    Closed = 1,    // a closed-form number, shown at once (remedy_text)
    Available = 2, // a simulated remedy; "Find fix" computes it
    Running = 3,
    Solved = 4,    // remedy_text holds the answer
    Failed = 5,    // the search found no value (remedy_text says so)
};
const char* to_string(RemedyState s);

struct AdviceRange {
    double from_rpm = 0.0;
    double to_rpm = 0.0;
};

struct AdviceItem {
    std::string id;        // "<Kind>" or "<Kind>@<field>" (stable across runs)
    ps::drivetrain::LimitKind kind = ps::drivetrain::LimitKind::WastegateSaturated;
    std::string kind_name;  // the LimitKind name, or the BuildRule name / "LoaderNotice" (then `kind` is meaningless)
    std::string rule_name;  // non-empty for build-rule and loader-notice items
    AdviceSeverity severity = AdviceSeverity::Advice;
    std::vector<AdviceRange> ranges; // merged contiguous rpm ranges, ascending
    double rpm_from = 0.0;           // ranges.front().from_rpm
    double rpm_to = 0.0;             // ranges.back().to_rpm
    std::string variant;             // "" = the kind's base text
    std::string title;
    std::vector<std::string> lines;
    std::string suggestion; // "" = none
    std::string option_id;
    // Damage-model hook: the part a part-limit item says would take the damage ("charge_piping", "turbo_unit",
    // "connecting_rods", "pistons", "crankshaft", "valve_springs", "cylinder_block", "engine_internals" for the engine-wide fallback); "" for
    // physics limit kinds and the loader notice. A future damage model wears or breaks exactly this part.
    std::string damage_part;
    RemedyState remedy_state = RemedyState::None;
    std::string remedy_text;
    bool can_solve = false; // a "Find fix" search exists for this item
    // The (filtered) findings behind the item, ascending rpm: what a solve works on.
    std::vector<ps::drivetrain::LimitFinding> findings;
};

// What the pure builder needs from a sweep.
struct AdviceInput {
    std::vector<ps::drivetrain::LimitFinding> findings;
    std::vector<ps::drivetrain::DynoPoint> points; // the swept points (peak boost, sweep extent)
    double target_boost_bar = std::numeric_limits<double>::quiet_NaN(); // configured, gauge; NaN = no turbo
    std::vector<ps::drivetrain::BuildIssue> build_issues; // check_turbo_stage_coverage of the engine (physics, with the policy)
    std::vector<std::string> loader_warnings;             // EngineMapOptions::warnings_out of the load
    // Set when the load extended the turbo stage map (rg/turbo_map_extension.h): a demand still outside the extended map is the
    // compressor's choke, reported as such (variant "choke") instead of "map too narrow"; a fully covered demand shows no item.
    std::optional<TurboChoke> turbo_choke;
    // Part limits of the fitted parts (NaN = no such part): a garage turbo kit's charge piping and rotor (CompiledSetup::
    // turbo_kit), and the engine's stroke for the mean piston speed (run_advice_sweep; 0 = unknown).
    double piping_rated_bar = std::numeric_limits<double>::quiet_NaN();
    double piping_burst_bar = std::numeric_limits<double>::quiet_NaN();
    std::string piping_label;
    double turbo_max_shaft_rpm = std::numeric_limits<double>::quiet_NaN();
    double stroke_m = 0.0;
    // Engine internals (CompiledSetup::internals): the weakest piston-speed part (NaN = use the config's engine-wide
    // figure) and the valve springs' float rpm (NaN = no check). *_kind = the part kind (damage_part), *_option = the
    // garage option that swaps it ("" = the rule's own option_id).
    double piston_speed_limit_m_s = std::numeric_limits<double>::quiet_NaN();
    std::string piston_speed_part, piston_speed_kind, piston_speed_kind_label, piston_speed_option;
    double valve_float_rpm = std::numeric_limits<double>::quiet_NaN();
    std::string valve_part, valve_option;
    // Cooling check (rg/cooling_upgrades.h transient_thermal + steady_thermal, set by set_thermal_limits): for the sweep's
    // full-power points on the check's day and air speed, the SOONEST time from the warm spawn state to the coolant / oil
    // limit (*_limit_time_s, NaN = never reached: clean) with that point's rpm, air speed and steady temperature
    // (*_peak_c, what it would settle at), and the limits (the car's own overheat warning for the coolant, the check's oil
    // limit). coolant_limit_c / oil_limit_c NaN = no check (a car without the thermal model, a torque-curve engine).
    double coolant_limit_time_s = std::numeric_limits<double>::quiet_NaN(), oil_limit_time_s = std::numeric_limits<double>::quiet_NaN();
    double coolant_peak_c = std::numeric_limits<double>::quiet_NaN(), coolant_limit_c = std::numeric_limits<double>::quiet_NaN();
    double coolant_peak_rpm = 0.0;
    double oil_peak_c = std::numeric_limits<double>::quiet_NaN(), oil_limit_c = std::numeric_limits<double>::quiet_NaN();
    double oil_peak_rpm = 0.0;
    double thermal_ambient_c = 0.0, thermal_air_speed_m_s = 0.0, coolant_air_speed_m_s = 0.0, oil_air_speed_m_s = 0.0;
    std::string radiator_part, oil_cooler_part;               // installed grade labels ("stock radiator")
    std::string radiator_option, oil_cooler_option;           // the garage options that swap them
    // A bored block (EngineInternalsBuild::bore): the wall left between two cylinders and the [AS] safe figure (NaN = not
    // bored). Below the safe figure the CylinderWall build rule warns (damage_part "cylinder_block").
    double cylinder_wall_mm = std::numeric_limits<double>::quiet_NaN();
    double cylinder_wall_safe_mm = std::numeric_limits<double>::quiet_NaN();
    std::string bore_label, bore_option;
};
// Fills the part-limit fields of `input` from a compiled setup's turbo kit and engine internals (none = left NaN).
void set_part_limits(AdviceInput& input, const CompiledSetup& compiled);
// Fills the cooling-check fields of `input` from the loaded car (its VehicleDesc carries the engine thermal model with the
// installed cooling parts), the sweep's points (`input.points`: fuel flow and power per rpm) and the check settings; the grade
// labels and options come from `compiled.cooling`. Leaves the fields NaN when the car has no thermal model or the engine no
// fuel model (a torque map without fuel flow).
void set_thermal_limits(AdviceInput& input, const ps::vehicle::VehicleDesc& desc, const CoolingCheckConfig& check, const CompiledSetup& compiled);

// Groups the findings per kind (and part field), drops what the config hides, merges contiguous rpm ranges, renders
// the templates; worst severity first (then the config's kind order, then rpm).
std::vector<AdviceItem> build_advice(const AdviceInput& input, const AdvisorConfig& config);

// Whether solve_remedy has a search for the kind (limit_remedies.h): the wastegate kinds.
bool advice_kind_is_solvable(ps::drivetrain::LimitKind kind);

// Applies solved remedies (one per solved finding, same order as the solve points) to an item: Solved with the
// rendered text, or Failed when any search found no value.
void apply_solved_remedy(AdviceItem& item, const std::vector<ps::drivetrain::Remedy>& solved, const AdvisorConfig& config);

// "7000 RPM", "3500-8200 RPM", "2000-2500, 6000-7000 RPM".
std::string format_rpm_ranges(const std::vector<AdviceRange>& ranges, const AdvisorConfig& config);

// ---- the physics part --------------------------------------------------------------------------------------------------

struct AdviceSweep {
    AdviceInput input;
    ps::drivetrain::RemedyRequest remedy_request; // what solve_remedy needs (engine copy, maps, spec)
    bool has_engine = false;
    double sweep_seconds = 0.0;
    // A car without a combustion engine: the checks not made and why, in the player's words (the engine checks, the motor
    // and battery checks physics has no limit kind for, and the driveline checks that do not apply).
    std::vector<std::string> skipped;
    // ... and, when a traction motor drives the gearbox input, the input rpm / torque the driveline checks were graded on.
    std::vector<ps::drivetrain::DynoPoint> input_torque_points;
};
// The shift clutches of a clutch-to-clutch gearbox (automatic, dual-clutch, clutch-launched CVT: GearboxDesc::shift_clutch)
// against the sweep's peak net torque, graded exactly like physics_sim's manual-clutch check (check_driveline_limits:
// static capacity = capacity x static_factor; Near within policy.driveline_torque_margin, Limiting at the peak, Exceeded more than
// policy.gearbox_rating_over_exceeded over it - the owner's 2026-10-20 reuse of the gearbox band). Such a car has
// no ClutchDesc, so that check never sees them. Findings are ClutchCapacity at part {"gearbox", gearbox_file,
// "/shift_clutch/capacity_nm"} with a Closed remedy (required_min = peak / static_factor, required_margin x margin).
[[nodiscard]] std::vector<ps::drivetrain::LimitFinding> check_shift_clutch_limits(const std::vector<ps::drivetrain::DynoPoint>& points,
                                                                                   const ps::drivetrain::PowertrainDesc& powertrain,
                                                                                   const std::string& gearbox_file,
                                                                                   const ps::drivetrain::LimitPolicy* policy);

// Sweeps the engine of `desc` (simulated or torque-map) over rpm and collects the findings, plus the driveline check
// (clutch and, when its file names a rated_input_torque_nm, the gearbox). `config.policy` goes to the sweep
// (DynoSweepSpec::policy) and the driveline check; remedy_request.spec keeps that pointer, so `config` must outlive
// the returned AdviceSweep. `clutch_file` / `gearbox_file` (absolute paths, "" = unknown) only fill PartRef::file.
// A car without a combustion engine (electric, fuel-cell) gets no engine sweep: the clutch and gearbox-input checks run
// against the traction motors' peak input torque from ps::drivetrain::run_powertrain_dyno when a motor drives the gearbox
// input; everything not checked is listed in AdviceSweep::skipped.
// Throws ps::io::GenerationCancelled when `cancel` is set; std::runtime_error on an engine without a dyno harness.
AdviceSweep run_advice_sweep(const ps::vehicle::VehicleDesc& desc, const AdvisorConfig& config, ps::jobs::JobSystem* pool,
                             const std::atomic<bool>* cancel, const std::string& clutch_file = {}, const std::string& gearbox_file = {});

// ---- the background advisor -------------------------------------------------------------------------------------------

struct AdvisorRequest {
    CatalogEntry entry;
    SetupOptionTable table;
    SetupContext context;
    VehicleSetup setup;
    std::string key; // equal keys are one request
};

struct AdvisorResult {
    std::string vehicle_id, key, error;
    std::uint64_t revision = 0; // counts completed runs for one vehicle
    bool busy = false;
    int progress = 0;      // 0..100
    std::string stage;     // "Waiting", "Building engine maps", "Sweeping the engine", "Solving"
    bool solving = false;  // a Find fix search runs
    int solve_progress = 0;
    bool have_result = false;
    std::vector<AdviceItem> items;
    double maps_seconds = 0.0, sweep_seconds = 0.0, last_solve_seconds = 0.0;
};

// Immutable snapshots only; no Garage, Godot or live-world state on the worker (like GarageDyno).
class LimitAdvisor {
public:
    explicit LimitAdvisor(AdvisorConfig config);
    ~LimitAdvisor();
    LimitAdvisor(const LimitAdvisor&) = delete;
    LimitAdvisor& operator=(const LimitAdvisor&) = delete;

    [[nodiscard]] const AdvisorConfig& config() const { return config_; }
    void request(AdvisorRequest request); // newest wins; the debounce applies
    void cancel();                         // drops pending work, keeps the last result
    void solve(const std::string& item_id); // "Find fix": runs on the worker, the item changes state
    [[nodiscard]] AdvisorResult result() const;

private:
    void run();
    AdvisorConfig config_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::optional<AdvisorRequest> pending_;
    std::vector<std::string> pending_solves_;
    std::shared_ptr<std::atomic<bool>> cancel_;
    AdvisorResult result_;
    std::string requested_key_;
    std::uint64_t generation_ = 0; // bumps on every request / cancel
    bool stopping_ = false;
    std::thread thread_;
};

} // namespace rg
