// rg/damage.h - the car damage model (2026-10-16; owner: radiator / fan damage part, then on PR #33: "pushing far in
// excess of safe values should ruin a part quicker", "an engine without a real working cooling solution should keep
// running, but then other parts will start failing and that might cause the engine to shut off").
//
// Owner rules (binding): damage persists until it is repaired; repairs are free; a repair needs an explicit player action
// (the pause menu's "Repair car", the garage's Repair buttons; later a long pit stop); damage changes how the car drives;
// the whole model is switched by the difficulty setting difficulty.mechanical_damage (separate from any later collision
// damage).
//
// Engine-neutral (no Godot type). Three pieces:
//   * DamageConfig - data/vehicles/damage.json (rg.damage/2): the named parts (id, label, what a worn part does to the
//     car: its health-to-function translation) and the wear rules (a thermal reading past a safe value wears a part at a
//     rate read off a curve over the excess - a small excursion costs little, far past it ruins the part in seconds).
//   * DamageState  - the per-car part health (0 = destroyed .. 1 = as new), saved per vehicle in the user dir
//     (<user dir>/garage/damage/<vehicle id>.json, rg.damage_state/1). A part that is not listed is healthy, and a part
//     id the config does not know is kept (a newer build's part survives an older build's save).
//   * DamageModel  - one tick of accumulation on the Session stepping thread, from the engine thermal snapshot; plain
//     double arithmetic in a fixed order (rules in file order), so a run is deterministic and worker-count invariant.
//
// Health to function (each part has one effect, factor = scale_at_zero + (1 - scale_at_zero) x health; several parts on
// the same effect multiply):
//   radiator_ua - the radiator's heat-exchanger conductance (physics_sim World::set_engine_cooling_capacity)
//   fan_flow    - the radiator fan's air flow (same call)
//   friction    - the engine's friction (physics_sim World::set_engine_wear: the plant's oil-viscosity friction scale;
//                 scale_at_zero > 1: a worn part costs power and puts more heat into the oil)
//   none        - shown only
// A part with seize_below seizes once its health falls to or below that figure: the engine shuts off (the Session holds
// the ignition and the starter at 0) and cannot be restarted until the car is repaired. A car whose parts are all healthy
// never calls the physics, so its state hash is unchanged (the hash_check vehicle_step_steer hash, every pinned test).
//
// The cascade the owner asked for follows from the physics: worn cooling runs the coolant hotter, the coolant heats the
// oil, hot oil wears the bearings and pistons, their friction heats the oil further, and they seize.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rg {

enum class DamageEffect { None, RadiatorUa, FanFlow, Friction };
const char* to_string(DamageEffect effect);

struct DamagePartConfig {
    std::string id;          // lower_snake, the key of the saved state and of AdviceItem::damage_part
    std::string label;       // what the player reads ("Radiator")
    std::string description; // one line: what a worn part does
    DamageEffect effect = DamageEffect::None;
    double scale_at_zero = 1.0; // the effect's factor at health 0 (1 at health 1, linear between)
    double seize_below = -1.0;  // health at or below which the engine seizes (< 0 = never)
    std::string source;         // where the numbers come from ([AS] ...)
    [[nodiscard]] bool can_seize() const { return seize_below >= 0.0; }
};

enum class DamageSignal { CoolantC, OilC };

struct DamageRuleConfig {
    std::string id;
    std::string part;                     // a DamagePartConfig::id
    DamageSignal signal = DamageSignal::CoolantC;
    // Safe value: either absolute (above_c) or relative to the car's own overheat warning (above_overheat_k; the cooling
    // system's warnings.overheat_c as rg::engine_thermal_info reads it). Exactly one is given.
    bool relative_to_overheat = false;
    double above = 0.0;                   // deg C (absolute) or K above the overheat warning
    double after_s = 0.0;                 // dwell: the signal must stay past the safe value this long before wear starts
    // Wear curve: (excess in K past the safe value, health lost per second), excess strictly increasing from 0, rate
    // non-decreasing; linear between the points and, past the last point, on along the last segment's slope (so a value
    // far past the table only wears faster).
    std::vector<std::pair<double, double>> curve;
    bool only_while_fan_on = false;       // the rule only counts while the radiator fan runs
    bool only_while_running = false;      // the rule only counts while the engine runs (rubbing parts need motion)
    std::string source;
};

// Health lost per second at `excess_k` past the rule's safe value (0 at or below it).
double wear_rate_per_s(const DamageRuleConfig& rule, double excess_k);

struct DamageConfig {
    std::vector<DamagePartConfig> parts;   // file order (the HUD and garage list them in it)
    std::vector<DamageRuleConfig> rules;   // file order (the order accumulation applies them in)
    double report_below = 0.995;           // a part below this health counts as damaged (HUD line, garage)
    double save_interval_s = 5.0;          // the binding saves a changed state at most this often while driving
    [[nodiscard]] const DamagePartConfig* find(const std::string& id) const;
};

// Strict, exception-free loaders; std::nullopt + *err (with the origin) on any error.
std::optional<DamageConfig> parse_damage_config(const std::string& text, const std::string& origin, std::string* err);
std::optional<DamageConfig> load_damage_config(const std::string& path, std::string* err);

struct DamageState {
    std::map<std::string, double> health; // part id -> 0..1; absent = 1 (healthy)
    [[nodiscard]] double health_of(const std::string& part) const;
    [[nodiscard]] bool all_healthy() const; // every listed value is exactly 1 (or nothing listed)
};

// The per-vehicle state file: <user_dir>/garage/damage/<vehicle_id>.json.
std::string damage_state_path(const std::string& user_dir, const std::string& vehicle_id);
// A missing file is a healthy car (empty state). A broken file is an error (nullopt + *err): the caller decides
// (the binding drives healthy and says so; it never overwrites the broken file until the next save).
std::optional<DamageState> load_damage_state(const std::string& path, std::string* err);
std::optional<DamageState> parse_damage_state(const std::string& text, const std::string& origin, std::string* err);
std::string damage_state_to_json(const DamageState& state);
// Atomic write (temp file + rename); a healthy state removes the file instead. false + *err on an I/O error.
bool save_damage_state(const std::string& path, const DamageState& state, std::string* err);

// The factors the damaged parts give (1, 1, 1 = healthy).
struct DamageScales {
    double radiator_ua = 1.0;
    double fan_flow = 1.0;
    double friction = 1.0;
    [[nodiscard]] bool cooling_unity() const { return radiator_ua == 1.0 && fan_flow == 1.0; }
    [[nodiscard]] bool unity() const { return cooling_unity() && friction == 1.0; }
    bool operator==(const DamageScales& o) const { return radiator_ua == o.radiator_ua && fan_flow == o.fan_flow && friction == o.friction; }
    bool operator!=(const DamageScales& o) const { return !(*this == o); }
};
DamageScales damage_scales(const DamageConfig& config, const DamageState& state);
// The first part (config order) that has seized, nullptr when the engine runs.
const DamagePartConfig* seized_part(const DamageConfig& config, const DamageState& state);

// One tick's inputs (from the powertrain snapshot's EngineThermalSnapshot).
struct DamageInputs {
    bool has_thermal = false;
    double coolant_c = 0.0;
    double oil_c = 0.0;
    bool fan_on = false;
    bool engine_running = true;
};

class DamageModel {
public:
    // overheat_c: the car's overheat warning (rg::EngineThermalInfo::overheat_c; 0 = none: relative rules never fire).
    DamageModel(DamageConfig config, DamageState state, double overheat_c);
    // One tick of dt seconds. Returns true when any health changed.
    bool step(double dt, const DamageInputs& in);
    // Every part back to health 1 and every dwell timer to 0 (a repair).
    void repair();
    [[nodiscard]] const DamageState& state() const { return state_; }
    [[nodiscard]] const DamageConfig& config() const { return config_; }
    [[nodiscard]] DamageScales scales() const { return damage_scales(config_, state_); }
    [[nodiscard]] const DamagePartConfig* seized() const { return seized_part(config_, state_); }
    [[nodiscard]] double dwell_s(std::size_t rule) const { return dwell_.at(rule); }

private:
    DamageConfig config_;
    DamageState state_;
    double overheat_c_;
    std::vector<double> dwell_; // per rule, seconds past its safe value (reset when it drops back)
};

// What the HUD and the garage show.
struct DamagePartView {
    std::string id, label, description;
    double health = 1.0;
    bool damaged = false;
    bool seized = false;       // this part has seized the engine
    DamageEffect effect = DamageEffect::None;
    double effect_scale = 1.0; // this part's own factor on its effect
};
struct DamageView {
    std::vector<DamagePartView> parts; // config order, then unknown ids from the state (sorted)
    bool any_damaged = false;
    bool seized = false;               // the engine is seized (shut off until repaired)
    std::string seized_label;          // the part that seized it
    DamageScales scales;
};
DamageView damage_view(const DamageConfig& config, const DamageState& state);
// One short HUD line, "" when nothing is damaged:
//   "DAMAGE: Radiator 62 %, Radiator fan 80 % - repair: pause menu or garage"
//   "ENGINE SEIZED (Crank bearings) - repair the car: pause menu or garage"
std::string damage_warning_line(const DamageView& view);

} // namespace rg
