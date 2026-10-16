// rg/engine_internals.h - swappable engine internals (owner on PR #21: "The maximum safe piston speed should depend on the
// material used for the relevant parts (connecting rod, ...). Such parts of the engine should be swappable." and "One
// could, for example, slap in a titanium connecting rod (such as used in F1). That one should hold up to higher speeds.").
//
// data/engines/engine_internals.json (rg.engine_internals/1) lists the part kinds (connecting rods, pistons, crankshaft,
// valve springs) with material grades. Each grade carries a limit and a mass factor ([AS], reasoning per entry):
//   * limit_kind "mean_piston_speed": the mean piston speed (2 x stroke x rpm / 60) the part survives. The engine's limit
//     is the WEAKEST installed part's (a titanium rod on a cast crank still breaks the crank);
//   * limit_kind "limiter_factor": valve float, as a factor of the engine's own factory limiter.
// A stock grade is rated for what the factory runs: its limit is at least `stock_margin` x the factory value (the mean
// piston speed at the engine's own limiter, or the limiter itself), so a car as built never shows a warning. An upgrade
// never rates lower than stock (limit = max(grade figure, stock limit)).
// Masses: stock set = stock_mass_per_litre_kg x displacement; a grade's set = stock x mass_factor. The garage's part-mass
// tree gets the DIFFERENCE to stock (a node per changed part under "engine.internals", negative for a lighter part), so the
// drive's chassis mass follows; the engine's rotating inertia in the physics is not changed (no physics input for it).
//
// Damage-model data (docs/garage_ui.md "Damage model data"): nothing fails in the simulation yet; the garage setup check
// (rg/limit_advisor.h MeanPistonSpeed / ValveFloat) warns and names the part (`kind`) that would take the damage.
// Which engines: simulated engines with a cycle block (bore/stroke known), not rotary (no pistons/rods/valves); a torque-curve
// engine is refused with engine_internals_refusal's reason. Engine-neutral, exception-free.
#pragma once

#include "rg/engine_store.h"
#include "rg/part_mass.h"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace rg {

enum class InternalLimitKind { MeanPistonSpeed, LimiterFactor };

struct InternalGrade {
    std::string id;           // "stock", "forged_steel", "titanium" ...; "stock" is always first
    std::string label;
    double limit = 0.0;       // m/s (MeanPistonSpeed) or x factory limiter (LimiterFactor)
    double mass_factor = 1.0; // the set's mass relative to stock
    std::string source;
};

struct InternalPartKind {
    std::string kind;   // "connecting_rods", "pistons", "crankshaft", "valve_springs"
    std::string label;  // "Connecting rods"
    InternalLimitKind limit_kind = InternalLimitKind::MeanPistonSpeed;
    double stock_mass_per_litre_kg = 0.0;
    std::string mass_source;
    std::vector<InternalGrade> grades;
    [[nodiscard]] const InternalGrade* find(const std::string& id) const;
};

struct EngineInternalsConfig {
    double stock_margin = 1.15;
    std::vector<InternalPartKind> parts;
    [[nodiscard]] const InternalPartKind* find(const std::string& kind) const;
};

// Strict, exception-free: nullopt + *err on a malformed file (every part needs a "stock" grade first).
std::optional<EngineInternalsConfig> load_engine_internals(const std::string& path, std::string* err);
std::optional<EngineInternalsConfig> parse_engine_internals(const std::string& json_text, const std::string& origin, std::string* err);

// "" when the engine's internals can be swapped, else the reason in the player's words (`name` = the engine's name).
std::string engine_internals_refusal(const EngineFacts& facts, const std::string& name);

// One installed part as built: grade, limit for THIS engine, mass and its difference to stock.
struct InternalPartBuild {
    std::string kind, kind_label;
    std::string grade, grade_label;
    InternalLimitKind limit_kind = InternalLimitKind::MeanPistonSpeed;
    double limit = 0.0;          // m/s or rpm (LimiterFactor: factory limiter x factor), after the stock rule
    double mass_kg = 0.0;        // the set
    double mass_delta_kg = 0.0;  // to the stock set
    std::string mass_source;
    std::string option_id;       // the garage option that swaps it ("" when no option names it)
};

struct EngineInternalsBuild {
    double stroke_m = 0.0;
    double factory_limiter_rpm = 0.0;
    double factory_piston_speed_m_s = 0.0;   // at the factory limiter
    std::vector<InternalPartBuild> parts;    // every kind of the config, in config order
    // The weakest piston-speed part (the one that would fail first) and its limit; nullptr/NaN without one.
    [[nodiscard]] const InternalPartBuild* weakest_piston_speed() const;
    [[nodiscard]] const InternalPartBuild* valve_springs() const;   // the LimiterFactor part (valve float rpm)
    [[nodiscard]] double mass_delta_kg() const;
};

// `grades`: kind -> grade id ("stock" when absent); `options`: kind -> the option id that swaps it. nullopt + *err when a
// grade id is unknown or the engine is refused.
std::optional<EngineInternalsBuild> describe_engine_internals(const EngineInternalsConfig& config, const EngineFacts& facts,
                                                             const std::map<std::string, std::string>& grades,
                                                             const std::map<std::string, std::string>& options, std::string* err);

// A grade's limit for this engine (the stock rule applied), for the garage card; NaN on an unknown kind/grade.
double internal_grade_limit(const EngineInternalsConfig& config, const EngineFacts& facts, const std::string& kind, const std::string& grade);
// "limit 28.0 m/s mean piston speed (8,900 rpm) · -1.4 kg" style card line for one grade on this engine.
std::string internal_grade_detail(const EngineInternalsConfig& config, const EngineFacts& facts, const std::string& kind, const std::string& grade);

// The part-mass tree entry: an "internals" node under "engine" with one child per part that differs from stock (own
// mass = the difference, may be negative); nullopt when every part is stock.
std::optional<PartAttachment> engine_internals_parts(const EngineInternalsBuild& build);

} // namespace rg
