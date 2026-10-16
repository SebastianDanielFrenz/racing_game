// rg/damage.h - the car damage model, first piece (2026-10-16, owner: "yes" to a radiator / fan damage part).
//
// Owner rules (binding): damage persists until it is repaired; repairs are free; a repair needs an explicit player action
// (the garage's Repair button; later a long pit stop); damage changes how the car drives.
//
// Engine-neutral (no Godot type). Three pieces:
//   * DamageConfig - data/vehicles/damage.json (rg.damage/1): the named parts (id, label, what a worn part does to the
//     car) and the accumulation rules (a thermal reading above a threshold for a dwell time wears a part at a rate).
//   * DamageState  - the per-car part health (0 = destroyed .. 1 = as new), saved per vehicle in the user dir
//     (<user dir>/garage/damage/<vehicle id>.json, rg.damage_state/1). A part that is not listed is healthy, and a part
//     id the config does not know is kept (a newer build's part survives an older build's save).
//   * DamageModel  - one tick of accumulation on the Session stepping thread, from the engine thermal snapshot; plain
//     double arithmetic in a fixed order (rules in file order), so a run is deterministic and worker-count invariant.
//
// Effect on driving (today): a part whose effect is "radiator_ua" scales the radiator's heat-exchanger conductance, one
// with "fan_flow" the fan's air flow, each as min_scale + (1 - min_scale) x health; several parts on the same effect
// multiply. The Session hands the product to physics_sim's World::set_engine_cooling_capacity. A car whose parts are all
// healthy never calls it, so its state hash is unchanged (the hash_check vehicle_step_steer hash, every pinned test).
//
// Future sources (knock, piston speed, charge-pipe boost, turbo overspeed, other overheating parts) are more rule sources
// and more effect kinds: the store is keyed by part id, and every limit of the garage setup check already names the part
// it would hurt (AdviceItem::damage_part).
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace rg {

enum class DamageEffect { None, RadiatorUa, FanFlow };
const char* to_string(DamageEffect effect);

struct DamagePartConfig {
    std::string id;          // lower_snake, the key of the saved state and of AdviceItem::damage_part
    std::string label;       // what the player reads ("Radiator")
    std::string description; // one line: what a worn part does
    DamageEffect effect = DamageEffect::None;
    double min_scale = 1.0;  // the effect's factor at health 0 (1 at health 1, linear between)
    std::string source;      // where the numbers come from ([AS] ...)
};

enum class DamageSignal { CoolantC, OilC };

struct DamageRuleConfig {
    std::string id;
    std::string part;                     // a DamagePartConfig::id
    DamageSignal signal = DamageSignal::CoolantC;
    // Threshold: either absolute (above_c) or relative to the car's own overheat warning (above_overheat_k; the cooling
    // system's warnings.overheat_c as rg::engine_thermal_info reads it). Exactly one is given.
    bool relative_to_overheat = false;
    double above = 0.0;                   // deg C (absolute) or K above the overheat warning
    double after_s = 0.0;                 // dwell: the signal must stay above the threshold this long before wear starts
    double rate_per_s = 0.0;              // health lost per second while wearing
    double rate_per_s_per_k = 0.0;        // plus this per second per K above the threshold
    bool only_while_fan_on = false;       // the rule only counts while the radiator fan runs
    std::string source;
};

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

// The cooling factors the damaged parts give (1, 1 = healthy).
struct CoolingScales {
    double radiator_ua = 1.0;
    double fan_flow = 1.0;
    [[nodiscard]] bool unity() const { return radiator_ua == 1.0 && fan_flow == 1.0; }
    bool operator==(const CoolingScales& o) const { return radiator_ua == o.radiator_ua && fan_flow == o.fan_flow; }
    bool operator!=(const CoolingScales& o) const { return !(*this == o); }
};
CoolingScales cooling_scales(const DamageConfig& config, const DamageState& state);

// One tick's inputs (from the powertrain snapshot's EngineThermalSnapshot).
struct DamageInputs {
    bool has_thermal = false;
    double coolant_c = 0.0;
    double oil_c = 0.0;
    bool fan_on = false;
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
    [[nodiscard]] CoolingScales scales() const { return cooling_scales(config_, state_); }
    [[nodiscard]] double dwell_s(std::size_t rule) const { return dwell_.at(rule); }

private:
    DamageConfig config_;
    DamageState state_;
    double overheat_c_;
    std::vector<double> dwell_; // per rule, seconds above its threshold (reset when it drops below)
};

// What the HUD and the garage show.
struct DamagePartView {
    std::string id, label, description;
    double health = 1.0;
    bool damaged = false;
    DamageEffect effect = DamageEffect::None;
    double effect_scale = 1.0; // this part's own factor on its effect
};
struct DamageView {
    std::vector<DamagePartView> parts; // config order, then unknown ids from the state (sorted)
    bool any_damaged = false;
    CoolingScales scales;
};
DamageView damage_view(const DamageConfig& config, const DamageState& state);
// One short HUD line, "" when nothing is damaged: "DAMAGE: Radiator 62 %, Radiator fan 80 % - repair in the garage".
std::string damage_warning_line(const DamageView& view);

} // namespace rg
