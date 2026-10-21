// rg/vehicle_setup.h - setup overlays for a catalog vehicle (PLAN.md R6, 11.6).
// Engine-neutral; the Godot layer shows what this builds and forwards the
// player's choices, it decides nothing.
//
// A setup (`racing_game.vehicle_setup/1`) is a small overlay on a physics vehicle:
// a set of option values. The option table (data/vehicles/setup_options.json,
// `racing_game.vehicle_setup_options/1`) IS the whitelist: every option names the
// JSON-pointer paths it may change (with `*` for every array element and
// `[key=value]` to pick array elements, e.g. `/wheels/[is_front=true]/...`), the
// kind of change and its range. Nothing else can be changed.
//
// Applying a setup:
//   1. each option edits a working copy of the file its pointers name (the
//      vehicle file, or the gearbox/engine/turbo file it references); a tyre choice edits the wheels' tyre_spec;
//   2. the difference to the original file is written as an RFC 7396 JSON merge
//      patch (arrays replace whole, as that RFC defines) and applied to the
//      original - the merge patch is the mechanism, the edit step only builds it;
//   3. the whitelist is enforced on the result: every changed leaf must be one the
//      enabled options may touch;
//   4. the patched files are materialised into a work directory (a patched file
//      is written there, every relative file reference in it rewritten to an
//      absolute path; an untouched referenced file is referenced in place) and
//      loaded with rg::load_vehicle (ps::io::load_vehicle_json + the tyre injection). Whatever the loader says is the
//      verdict: a setup the loader rejects is shown with the loader's message and
//      cannot be saved.
// The work directory's files are removed again (ScopedWorkDir) - except the one
// materialisation a drive uses, which the Garage owns and removes on cleanup().
//
// v1 options: final drive, per-gear ratios (kept strictly decreasing), brake
// bias (same total) and brake force, spring/damper/anti-roll bar per axle,
// tyre choice per axle (every compound at every size the car's axle fitment allows), the three
// assists (their defaults in the controller block), paint and rim colour
// (visual only, no file edit). ABS/TC/ESC follow once physics has them.
#pragma once

#include "rg/engine_store.h"
#include "rg/driveline_swap.h"
#include "rg/gearbox_catalog.h"
#include "rg/turbo_kit.h"
#include "rg/engine_internals.h"
#include "rg/cooling_upgrades.h"
#include "rg/part_mass.h"
#include "rg/turbo_map_extension.h"
#include "rg/slider_map.h"
#include "rg/vehicle_catalog.h"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace rg {

enum class OptionKind {
    Scale,      // number: factor applied to every target leaf
    ScaleList,  // number[]: one factor per target leaf (a gear each)
    BrakeBias,  // number: the front share of the total brake torque
    FileChoice, // string: an explicitly compatible referenced component file
    TyreChoice, // string: a tyre id (compound_width_aspectrRim) whose size the axle's fitment allows
    Bool,       // bool: a target leaf set to it
    Colour,     // string "#rrggbb": visual only, edits no file
    PartChoice, // string: a part id for one removable / swappable part slot of the part-mass tree (weight reduction);
                // edits no file, only the tree's "trim.<slot>" node ("stock" = the car's own part)
    ScalePreset, // string: a preset id; preset i scales the leaves of pointer pattern k by its factors[k] (a battery pack
                 // sets capacity, power and mass together; "stock" = the car's own values). No free per-leaf factor.
};

const char* to_string(OptionKind k);

struct SetupPart {
    std::string id, label, path, image, detail;
    bool naturally_aspirated = false;
    // A catalog option may group its parts (the gearbox list: "Manual" / "Automatic" / "CVT") and give each a second line.
    std::string section;   // "" = not grouped
    std::string subtitle;  // kind and focus, e.g. "Sequential · Race · straight-cut"; detail holds the numbers
    // PartChoice: the part's mass is part_masses.json `mass_key` (empty = the car's stock part) x `mass_factor`;
    // `removed` takes the part out of the car.
    std::string mass_key;
    double mass_factor = 1.0;
    bool removed = false;
    // The part's mass in kg as the model computed it (the stock part's, or the choice's after mass_factor/removal); < 0 = no figure.
    double mass_kg = -1.0;
    // False when the car cannot take this choice (a turbo kit or engine that does not fit the engine bay): `fit_reason` says why
    // in the player's words. The garage hides such a choice by default ("show all" lists it greyed out with the reason).
    bool fits = true;
    std::string fit_reason;
    // ScalePreset: one factor per pointer pattern of the option (same order), each in (0, 10).
    std::vector<double> factors;
};

// ScalePreset: how the card line shows the value a pointer pattern reaches (summed over its leaves) - "135 kWh".
// An empty unit leaves that pattern out of the line.
struct PresetDisplay {
    std::string unit;
    double multiplier = 1.0;
    int decimals = 0;
};

struct SetupOptionDef {
    std::string id;       // unique, lower_snake
    std::string label;
    std::string group;    // "Drivetrain", "Brakes", "Suspension", "Tyres", "Assists", "Look"
    std::string area;     // garage camera area the option belongs to ("overview", "wheels", "engine", "rear")
    std::string help;
    std::string unit;     // shown after the value ("x", "%", "")
    OptionKind kind = OptionKind::Scale;
    double min = 1.0;     // Scale/ScaleList: factor range
    double max = 1.0;
    double step = 0.01;
    double max_shift = 0.1; // BrakeBias: allowed shift of the front share around the stock one (required in the table)
    double share_min = 0.0; // BrakeBias: lowest front share the garage allows (required in the table)
    double share_max = 1.0; // BrakeBias: highest front share the garage allows (required in the table)
    double max_shift_bound = 0.0; // BrakeBias: upper limit of max_shift, in (0, 0.5] (required in the table)
    std::string file = "vehicle"; // "vehicle" | "gearbox" | "engine" | "turbo"
    std::vector<std::string> pointers; // the whitelist paths (empty for Colour)
    std::string monotonic;             // "decreasing": the resulting list must stay strictly decreasing
    std::vector<SetupPart> parts; // FileChoice: approved files; paths resolved against option-table directory
                                  // ScalePreset: the presets (id, label, factors), "stock" implicit
    std::vector<PresetDisplay> preset_display; // ScalePreset: one per pointer pattern (empty = no numbers on the cards)
    // FileChoice whose parts are not listed in the option table but come from a store at run time (engine garage G1):
    // "engines" = every engine of rg::EngineStore (defaults and user engines; ids "default:<id>" / "user:<uid>", a
    // legacy table id such as "sedan_i6" still resolves), "fuels" = the fuels of the installed engine (ids = fuel stems).
    // "gearboxes" = the rg::GearboxCatalog (data/gearboxes/gearbox_catalog.json), offered when the box's torque rating
    // covers the installed engine's peak torque; ids = the catalogue ids. An engine choice also derives the swap
    // package (engine_swap.h); all of them ignore compatible_engines.
    // "turbo_kits" = the garage turbo kits (rg/turbo_kit.h, data/engines/turbo_kits.json) for the INSTALLED engine when it
    // has no turbo of its own: the kit is sized for that engine and written to the cache, the engine's
    // default_forced_induction, manifold-pressure grid and (when above the kit ceiling) compression ratio are set. The
    // option is unavailable, with OptionView::unavailable_reason, on an engine that cannot take one.
    // "engine_internals" = the grades of one engine part (`internal` names the kind: connecting_rods, pistons, crankshaft,
    // valve_springs; rg/engine_internals.h, data/engines/engine_internals.json) for the INSTALLED engine; the choice edits
    // no file (limits and masses only: the setup check and the part-mass tree). Unavailable, with a reason, on an engine
    // whose internals cannot be swapped (torque curve, rotary). Ids = the grade ids, "stock" = the factory part.
    // Drivetrain swap (rg/driveline_swap.h, data/drivetrains/driveline_catalog.json): "drive_layouts" = the layouts
    // (FWD / RWD / AWD with a centre kind; ids = the catalogue ids), "differentials" = the axle differential units for
    // the axle named by `axle` (front | rear; offered only while that axle is driven). Both transform the vehicle file's
    // driveline in code (never a file reference) and are offered only on a car whose driveline the swap understands.
    std::string catalog;
    std::vector<std::string> compatible_engines; // original engine stems, an explicit game fitment list; "*" = any engine
    std::string part_format; // required referenced file format
    bool no_turbo_override = false; // default engine turbo must not shadow a vehicle override
    std::string internal;              // "engine_internals" catalog: the part kind
    // "cooling_upgrades" catalog (rg/cooling_upgrades.h, data/cooling_systems/cooling_upgrades.json): the cooling part kind
    // (radiator, fan, oil_cooler, block_cooling) whose grades the option offers for the INSTALLED engine and the car's
    // cooling_system file. The choice patches the cooling_system / engine thermal files (written by materialise_setup) and adds
    // a mass node under "engine.cooling". Unavailable, with a reason, on a car whose vehicle file names no cooling_system or
    // whose engine has no thermal data. Ids = the grade ids, "stock" = the car's own part.
    std::string cooling_part;
    std::string axle;                  // TyreChoice and the "differentials" catalog: "front" | "rear"
    std::string slot;                  // PartChoice: the part slot (catalog "interior" key, tree node "trim.<slot>")
    // Scale only (garage numeric inputs): the option is shown and typed in ABSOLUTE units (rpm, bar, ms) on a linear or
    // logarithmic slider, derived from the stock value of its first target leaf; min/max/step are then not authored and the
    // allowed factor range is abs_min/stock .. abs_max/stock (widened to contain 1). See rg/slider_map.h.
    std::optional<SliderSpec> slider;
    // Bool only: a visual part of the car's art (CatalogEntry::model_options), not a file edit - no file, no
    // pointers; offered only on a car whose model carries it, stock = off.
    std::string model_option;
    // FileChoice only: the controller/component entry the pointers name may be ABSENT in a car's stock file (the car
    // has no such part). Stock then means "nothing fitted" (shown as `stock_label`, the file stays untouched); any other
    // choice appends `entry` (a JSON object, e.g. {"id":"assists","type":"electronics"}) to the array at `array`
    // (a JSON pointer, e.g. "/controllers") with its file reference set under `ref_key`.
    struct InsertIfAbsent {
        std::string array;
        std::string entry_json;
        std::string ref_key = "ref";
        std::string stock_label;
    };
    std::optional<InsertIfAbsent> insert_if_absent;
};

struct SetupOptionTable {
    std::string format;
    std::vector<SetupOptionDef> options;
    [[nodiscard]] const SetupOptionDef* find(const std::string& id) const;
};

std::optional<SetupOptionTable> load_setup_options(const std::string& path, std::string* err);
std::optional<SetupOptionTable> parse_setup_options(const std::string& json_text, const std::string& origin,
                                                    std::string* err);

// SetupValue (the variant of one option value) lives in vehicle_catalog.h: a preset entry carries one.

// The overlay: only the options the player changed from the stock value.
struct VehicleSetup {
    std::string vehicle_id;
    std::map<std::string, SetupValue> values;
};

std::string setup_to_json(const VehicleSetup& setup);
// Replaces every pre-universal tyre name (hyper_rear_track_w255_r21, passenger_sport_225_45r17, the physics_sim
// stock files ...) of a tyre_choice value by its tyre id (rg::migrate_tyre_id). Names it does not know stay as
// they are (and then fail with a message naming the reason). Applied when a saved setup is loaded (Garage) and again
// when a setup is compiled.
struct SetupContext;
void migrate_setup_tyres(VehicleSetup& setup, const SetupOptionTable& table, const SetupContext& ctx);
// Parses a saved setup; every option id must exist in `table` and carry the
// value type its kind needs (nullopt + *err otherwise). Range checks happen when
// the setup is applied, not here.
std::optional<VehicleSetup> parse_setup(const std::string& json_text, const SetupOptionTable& table,
                                        const std::string& origin, std::string* err);

// Where the files an option may edit are found.
struct SetupContext {
    std::string tyre_families_path;     // data/tyres/families.json (rg/tyre_spec.h): the compounds TyreChoice offers; "" = no tyre choice
    std::string work_root;              // materialisations and validation temp dirs go under here
    std::string engine_map_cache_dir;   // .psmaps cache for load_vehicle_json ("" = no disk cache)
    // The engine list the "engines"/"fuels" catalog options read (a snapshot: a worker thread may hold a copy of this
    // context while the garage edits user engines). nullptr = those options are unavailable.
    std::shared_ptr<const EngineStore> engines;
    // The gearbox list the "gearboxes" catalog option reads. nullptr = that option is unavailable.
    std::shared_ptr<const GearboxCatalog> gearboxes;
    // The drive layouts and differentials the "drive_layouts"/"differentials" options read. nullptr = unavailable.
    std::shared_ptr<const DrivelineCatalog> drivelines;
    // Turbo stage map extension policy (rg/turbo_map_extension.h): when set, a load that finds the engine's air demand outside
    // the stage map extends and caches the map under engine_map_cache_dir. nullptr = maps are used as they are.
    std::shared_ptr<const TurboMapExtensionConfig> turbo_map_extension;
    // The garage turbo kits the "turbo_kits" catalog option offers (rg/turbo_kit.h). nullptr = that option is unavailable.
    std::shared_ptr<const TurboKitConfig> turbo_kits;
    // The swappable engine internals the "engine_internals" catalog options offer (rg/engine_internals.h). nullptr = those
    // options are unavailable and the setup check rates the engine with AdvisorConfig's one engine-wide piston speed.
    std::shared_ptr<const EngineInternalsConfig> engine_internals;
    // The cooling system upgrades the "cooling_upgrades" catalog options offer (rg/cooling_upgrades.h). nullptr = those options
    // are unavailable and the setup check skips the temperature rules.
    std::shared_ptr<const CoolingUpgradeConfig> cooling_upgrades;
    // Base part masses (data/vehicles/part_masses.json, rg/part_mass.h). With it and the entry's body mass, every
    // model / compiled setup carries the car's part-mass tree and the drive takes its chassis mass from it.
    // nullptr = no tree: the drive uses the catalog chassis mass plus the engine swap's delta.
    std::shared_ptr<const PartMassTable> part_masses;
};

// One option as the configurator shows it, for one vehicle.
struct OptionView {
    SetupOptionDef def;                // pointers already replaced by the entry's override
    bool available = true;             // false: the entry does not offer it, or its pointers match nothing
    std::string unavailable_reason;    // !available on an offered option: why, in the player's words ("" = nothing to say)
    double min = 1.0;                  // effective range (Scale/ScaleList: factor; BrakeBias: front share)
    double max = 1.0;
    SetupValue stock;                  // the value that changes nothing
    std::vector<std::string> choices;  // TyreChoice: candidate tyre ids (compound x allowed sizes)
    std::vector<SetupPart> parts; // available presentation records, including stock
    std::vector<std::string> coming_soon; // a grouped catalog option: the sections that have no entries yet (none since the CVT rows)
    int list_size = 0;                 // ScaleList: number of entries
    std::vector<double> stock_numbers; // ScaleList: stock leaf values (gear ratios) for display
};

// The upgrade lists show only what the car can fit unless the player asks for everything (settings key
// kShowAllUpgradesKey, hidden, default off). The stock choice is never hidden.
inline constexpr const char* kShowAllUpgradesKey = "garage.show_all_upgrades";
// The parts of an option the garage lists: every part, or with show_all == false only those that fit (plus stock).
std::vector<SetupPart> visible_parts(const OptionView& view, bool show_all);
// "" while at least one non-stock part fits (or the option has none); else the reason the first hidden one gives, so a tile
// with an empty list can say why ("<label>: <reason>").
std::string hidden_parts_reason(const OptionView& view);

// How the garage shows and types ONE number of an option (a Scale, a BrakeBias, one gear of a ScaleList) in the units the
// player thinks in. displayed = stored x multiplier; every field comes from the data and the car's stock values.
struct NumberEntry {
    double multiplier = 1.0;
    std::string unit;                    // "rpm", "bar", "ms", "%", "x", "" (a gear ratio)
    int decimals = 2;
    double lo = 0.0, hi = 0.0;           // the allowed range, displayed units
    bool positional = false;             // true: the slider runs over [0, 1] along `mapping`; false: over the stored value
    SliderMapping mapping = SliderMapping::Linear;
};
// Scale / ScaleList (entry `index`) / BrakeBias; every other kind returns the default (multiplier 1, empty range).
NumberEntry number_entry(const OptionView& view, int index = 0);
// The stored value as the player reads it ("8250 rpm", "x1.05", "52.5 %").
std::string format_entry(const NumberEntry& entry, double stored);
double entry_to_displayed(const NumberEntry& entry, double stored);
// Slider position in [0, 1] of a stored value (positional entries) and the stored value at a position (rounded to the
// entry's decimals in displayed units so a slider lands on readable numbers).
double entry_slider_position(const NumberEntry& entry, double stored);
double entry_value_at_position(const NumberEntry& entry, double position);
// A typed displayed number converted to the stored value; !ok + message ("outside the allowed range 2000 to 20000 rpm")
// when it is not finite or outside [lo, hi].
struct EntryConversion {
    bool ok = false;
    double value = 0.0;
    std::string message;
};
EntryConversion entry_from_displayed(const NumberEntry& entry, double displayed);

struct SetupModel {
    std::vector<OptionView> options;
    std::string error; // non-empty: the vehicle's files could not be read
    // With an installed engine/fuel choice (the `installed` setup): what the swap changes, for the player.
    std::vector<std::string> swap_lines;
    double chassis_mass_delta_kg = 0.0;
    std::string engine_title;    // the installed engine's display name ("" = the car's own)
    std::string engine_key;      // the installed engine's EngineStore key (the car's own too); "" = not in the store
    // The part-mass tree of the car with the `installed` part choices (engine, gearbox ...); !parts.ok without a part
    // mass table or a body mass (parts.error says why).
    PartTree parts;
    // The driveline with the installed choices (a drivetrain swap included): what the stats show.
    DrivelineInfo driveline;
};
// Builds the option views of an entry from its files.
// Whether the car offers this weight-reduction choice: the option is in its list AND the car has that part at all
// (catalog "interior" slot). A car without a separate passenger seat (a bench) or without rear seats (a two-seater)
// does not offer the removal; a setup that still holds such a value ignores it.
bool part_choice_offered(const CatalogEntry& entry, const SetupOptionDef& def);

SetupModel build_setup_model(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx,
                             const VehicleSetup* installed = nullptr);

// The result of applying a setup to the entry's files in memory (no disk write).
struct CompiledSetup {
    bool ok = false;
    std::string error;                           // the rule that was broken, in plain words
    std::string vehicle_patch;                   // RFC 7396 merge patch for the vehicle file (JSON text, "{}" if none)
    std::map<std::string, std::string> patches;  // "gearbox"/"engine"/"tyre:<stem>" -> merge patch text
    std::string paint;                           // visual: "" = the catalog default
    std::string rim;
    std::vector<std::string> model_options;      // visual: the model parts switched on (SetupOptionDef::model_option)
    // Engine garage G1: what an engine/fuel choice changed besides the choice itself.
    std::vector<std::string> swap_lines;         // one line each, for the player (fuel kept/changed, clutch, mass ...)
    double chassis_mass_delta_kg = 0.0;          // added to the catalog chassis mass for the drive
    std::string engine_title;                    // the installed engine's display name ("" = the car's own)
    // The part-mass tree of the car as this setup leaves it (rg/part_mass.h). When parts.ok, the drive's chassis mass
    // is parts.chassis_kg (chassis_mass_delta_kg above is then only the engine line of swap_lines).
    PartTree parts;
    // A garage turbo kit (catalog "turbo_kits"), sized for the FINAL boost target (the boost slider re-sizes it), with
    // its charge piping, parts, masses and limits (the part-mass tree and the limit advisor read it); unset = no kit.
    std::optional<TurboKitBuild> turbo_kit;
    // The engine internals as installed (catalog "engine_internals"; every part, stock or swapped) with their limits for
    // this engine and masses; unset without an internals table or on an engine whose internals cannot be swapped.
    std::optional<EngineInternalsBuild> internals;
    // The cooling parts as installed (catalog "cooling_upgrades"; every kind, stock or upgraded) with their masses; unset
    // when the car has no cooling model or the context carries no upgrade table.
    std::optional<CoolingBuild> cooling;
};
CompiledSetup compile_setup(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx,
                            const VehicleSetup& setup);

// Checks `patch` (merge patch text) against the leaves `pointers` may change, for
// the file text `base_json`: every leaf the patch changes must be reachable by one
// of the patterns. "" when fine, else the offending path. Exposed for tests.
std::string whitelist_violation(const std::string& base_json, const std::string& patch_json,
                                const std::vector<std::string>& pointer_patterns);
// The same with subtrees (concrete JSON pointers, e.g. "/controllers/0/auto_clutch") that trusted code - the swap
// package - may change without an option naming each leaf.
std::string whitelist_violation_in(const std::string& base_json, const std::string& patch_json,
                                   const std::vector<std::string>& pointer_patterns,
                                   const std::vector<std::string>& derived_subtrees);
// RFC 7396: applies `patch` to `base`; "" on a syntax error. Exposed for tests.
std::string apply_merge_patch(const std::string& base_json, const std::string& patch_json);

struct MaterialisedSetup {
    bool ok = false;
    std::string error;
    std::string vehicle_path;          // the file to hand to load_vehicle_json
    std::string dir;                   // the directory everything was written into
    std::vector<std::string> files;    // every file written
    CompiledSetup compiled;
};
// Compiles and writes the setup under `out_dir` (created). The caller removes it.
MaterialisedSetup materialise_setup(const CatalogEntry& entry, const SetupOptionTable& table,
                                    const SetupContext& ctx, const VehicleSetup& setup, const std::string& out_dir);

struct ValidationResult {
    bool ok = false;
    std::string message; // "" when ok; else the rule or the loader's own message
};
// compile + materialise into a scratch directory + ps::io::load_vehicle_json; the
// scratch directory is always removed again.
ValidationResult validate_setup(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx,
                                const VehicleSetup& setup, bool generate_maps = true);

// A scratch directory under `root` that removes itself.
class ScopedWorkDir {
public:
    explicit ScopedWorkDir(const std::string& root, const std::string& prefix);
    ~ScopedWorkDir();
    ScopedWorkDir(const ScopedWorkDir&) = delete;
    ScopedWorkDir& operator=(const ScopedWorkDir&) = delete;
    [[nodiscard]] const std::string& path() const { return path_; }
    // Number of regular files below it right now (tests).
    [[nodiscard]] static std::size_t file_count(const std::string& dir);

private:
    std::string path_;
};

// The part-mass tree of the stock car (no setup), !ok without ctx.part_masses or a body mass.
PartTree stock_part_tree(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx);

// Equality of two option values (numbers within 1e-9), used to drop a value set back to stock.
bool setup_values_equal(const SetupValue& a, const SetupValue& b);

// "#rrggbb" check shared by the setup and the UI view-models.
bool is_hex_colour(const std::string& s);

} // namespace rg
