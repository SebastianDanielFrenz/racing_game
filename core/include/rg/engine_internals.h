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
// never rates lower than stock (limit = max(grade figure, stock limit)). A DOWNGRADE (a grade whose table figure is below
// the stock table figure: cast-steel rods, low-silicon pistons, a lightweight cast crank, weak single springs) keeps its own
// figure, so it may rate below what the factory limiter needs and then the setup check warns (MeanPistonSpeed / ValveFloat,
// naming the part). Stock itself still holds the stock_margin.
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

enum class InternalLimitKind { MeanPistonSpeed, LimiterFactor, RodLoad };

struct InternalGrade {
    std::string id;           // "stock", "forged_steel", "titanium" ...; "stock" is always first
    std::string label;
    double limit = 0.0;       // m/s (MeanPistonSpeed), x factory limiter (LimiterFactor) or rated tensile N per litre of
                              // cylinder volume (RodLoad: one rod's rated load = limit x displacement / cylinders)
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

// Cylinder bore upgrade (owner on PR #21: "I want upgrades that bore out the cylinders; this logically implies new pistons
// (due to diameter difference), among other things."). engine_internals.json "bore": oversize steps, limited per engine by
// the wall left between two bores (bore spacing = pitch_factor x factory bore, [AS]: no engine file carries its spacing).
struct BoreStep {
    std::string id;            // "overbore_0_5", "overbore_1_0", "big_bore"
    std::string label;
    double oversize_mm = 0.0;  // added to the factory bore
    std::string source;
};

struct BoreConfig {
    double pitch_factor = 1.09;               // bore spacing / factory bore
    double min_wall_mm = 4.0;                 // a step leaving less is refused
    double safe_wall_mm = 5.0;                // below it the setup check warns (damage part cylinder_block)
    double piston_thickness_exponent = 1.0;   // piston mass x (bore / factory bore)^(2 + this)
    double liner_length_per_stroke = 1.5;     // bored length = this x stroke (block mass removed)
    double liner_density_kg_m3 = 7200.0;
    std::vector<BoreStep> steps;
    [[nodiscard]] const BoreStep* find(const std::string& id) const;
};

struct EngineInternalsConfig {
    double stock_margin = 1.15;
    // Rod load (limit_kind "rod_load"): peak inertial tension at TDC F = m_recip x (S/2) x w^2 x (1 + lambda), with
    // m_recip = piston set + the small-end share of the rod, lambda = crank radius / rod length (the engine data has no rod
    // length yet, so one [AS] ratio for every engine).
    double rod_crank_ratio = 0.3;
    double rod_small_end_share = 1.0 / 3.0;
    std::vector<InternalPartKind> parts;
    std::optional<BoreConfig> bore;   // absent: no bore steps
    [[nodiscard]] const InternalPartKind* find(const std::string& kind) const;
};

// The grades-map key (describe_engine_internals) and refresh key that carries the bore step id ("stock" = factory bore).
inline constexpr const char* kCylinderBoreKind = "cylinder_bore";

// Strict, exception-free: nullopt + *err on a malformed file (every part needs a "stock" grade first).
std::optional<EngineInternalsConfig> load_engine_internals(const std::string& path, std::string* err);
std::optional<EngineInternalsConfig> parse_engine_internals(const std::string& json_text, const std::string& origin, std::string* err);

// "" when the engine's internals can be swapped, else the reason in the player's words (`name` = the engine's name).
std::string engine_internals_refusal(const EngineFacts& facts, const std::string& name);

// A bore step on one engine, from the FACTORY figures (EngineFacts::factory_*; a bored facts value gives the same answer).
// The compression ratio is computed with the head's clearance volume kept: CR' = 1 + (CR - 1) x (bore' / bore)^2.
struct BoreGeometry {
    std::string step, label;            // "stock" for the factory bore
    double oversize_mm = 0.0;
    double factory_bore_m = 0.0, bore_m = 0.0;
    double spacing_mm = 0.0;            // pitch_factor x factory bore
    double wall_mm = 0.0;               // spacing - bore
    double min_wall_mm = 0.0, safe_wall_mm = 0.0; // the config's figures
    double max_oversize_mm = 0.0;       // spacing - factory bore - min_wall_mm
    double displacement_l = 0.0, factory_displacement_l = 0.0;
    double compression_ratio = 0.0, factory_compression_ratio = 0.0;
    double block_mass_delta_kg = 0.0;   // the liner ring bored away (negative)
    bool fits = true;                   // wall >= min_wall_mm
    bool warns = false;                 // fits but wall < safe_wall_mm
    std::string reason;                 // why it does not fit ("" when it does)
};

// nullopt + *err when the engine's internals are refused (engine_internals_refusal), the config has no bore block or the
// step id is unknown. A step that does not fit still returns a geometry (fits = false, reason set).
std::optional<BoreGeometry> bore_geometry(const EngineInternalsConfig& config, const EngineFacts& facts, const std::string& step, std::string* err);
// "" when the step fits this engine, else the reason in the player's words.
std::string bore_refusal(const EngineInternalsConfig& config, const EngineFacts& facts, const std::string& step, const std::string& name);
// The facts of the bored engine: bore_m, displacement_l and compression_ratio of `g` (factory_* kept). Stock = unchanged.
EngineFacts bored_engine_facts(const EngineFacts& facts, const BoreGeometry& g);
// "87.0 mm · 2.54 L · CR 13.3 · wall 6.9 mm · -0.3 kg block" card line.
std::string bore_step_detail(const BoreGeometry& g);

// One installed part as built: grade, limit for THIS engine, mass and its difference to stock.
struct InternalPartBuild {
    std::string kind, kind_label;
    std::string grade, grade_label;
    InternalLimitKind limit_kind = InternalLimitKind::MeanPistonSpeed;
    double limit = 0.0;          // m/s, rpm (LimiterFactor: factory limiter x factor) or N per rod (RodLoad), after the stock rule
    // The engine speed at which this part reaches its rating (piston speed / rod load / valve float), in rpm. The weakest
    // part is the one with the lowest. For a rod it depends on the installed piston and rod masses.
    double limit_rpm = 0.0;
    double recip_mass_kg = 0.0;  // RodLoad: m_recip per cylinder as installed
    double mass_kg = 0.0;        // the set
    double mass_delta_kg = 0.0;  // to the stock set
    std::string mass_source;
    std::string option_id;       // the garage option that swaps it ("" when no option names it)
};

struct EngineInternalsBuild {
    std::optional<BoreGeometry> bore;   // set when the block is bored (grades[kCylinderBoreKind] != "stock")
    std::string bore_option_id;         // options[kCylinderBoreKind]
    double stroke_m = 0.0;
    double factory_limiter_rpm = 0.0;
    double factory_piston_speed_m_s = 0.0;   // at the factory limiter
    std::vector<InternalPartBuild> parts;    // every kind of the config, in config order
    // The weakest bottom-end part (piston speed or rod load: the lowest limit_rpm; the one that would fail first); nullptr without one.
    [[nodiscard]] const InternalPartBuild* weakest_piston_speed() const;
    [[nodiscard]] const InternalPartBuild* valve_springs() const;   // the LimiterFactor part (valve float rpm)
    [[nodiscard]] double mass_delta_kg() const;   // parts + the block material bored away
};

// `grades`: kind -> grade id ("stock" when absent), plus kCylinderBoreKind -> bore step id; `options`: kind -> the option
// id that swaps it. `facts` may be the factory or the bored facts (the factory_* fields are the baseline either way); with a
// bore step the pistons are the new diameter's: piston mass x (bore / factory bore)^(2 + exponent), which loads the rods,
// while rods, crank and springs keep their factory masses and the rods their factory rating (so the rod-limited rpm
// drops). nullopt + *err when a grade id is unknown, the bore step does not fit or the engine is refused.
std::optional<EngineInternalsBuild> describe_engine_internals(const EngineInternalsConfig& config, const EngineFacts& facts,
                                                             const std::map<std::string, std::string>& grades,
                                                             const std::map<std::string, std::string>& options, std::string* err);

// A grade's limit for this engine (the stock rule applied), for the garage card; NaN on an unknown kind/grade.
double internal_grade_limit(const EngineInternalsConfig& config, const EngineFacts& facts, const std::string& kind, const std::string& grade);
// "limit 28.0 m/s mean piston speed (8,900 rpm) · -1.4 kg" style card line for one grade on this engine.
std::string internal_grade_detail(const EngineInternalsConfig& config, const EngineFacts& facts, const std::string& kind, const std::string& grade);

// The part-mass tree entry: an "internals" node under "engine" with one child per part that differs from stock (own
// mass = the difference, may be negative; bored: the pistons of the new diameter even at the stock grade, and
// "engine.internals.cylinder_block" with the liner material removed); nullopt when every part is stock.
std::optional<PartAttachment> engine_internals_parts(const EngineInternalsBuild& build);

} // namespace rg
