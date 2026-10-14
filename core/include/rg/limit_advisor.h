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
    std::vector<AdviceKindConfig> kinds; // file order = the order of equal-severity items
    // Static build-rule issues (ps::drivetrain::BuildIssue, today TurboStageMapCoverage) shown after the sweep findings of
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
};

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
};
// Sweeps the engine of `desc` (simulated or torque-map) over rpm and collects the findings, plus the driveline check
// (clutch and, when its file names a rated_input_torque_nm, the gearbox). `config.policy` goes to the sweep
// (DynoSweepSpec::policy) and the driveline check; remedy_request.spec keeps that pointer, so `config` must outlive
// the returned AdviceSweep. `clutch_file` / `gearbox_file` (absolute paths, "" = unknown) only fill PartRef::file.
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
