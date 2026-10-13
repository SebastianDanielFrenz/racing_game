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
#include "rg/gearbox_catalog.h"
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
};

const char* to_string(OptionKind k);

struct SetupPart {
    std::string id, label, path, image, detail;
    bool naturally_aspirated = false;
    // A catalog option may group its parts (the gearbox list: "Manual" / "Automatic" / "CVT") and give each a second line.
    std::string section;   // "" = not grouped
    std::string subtitle;  // kind and focus, e.g. "Sequential · Race · straight-cut"; detail holds the numbers
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
    double max_shift = 0.1; // BrakeBias: allowed shift of the front share around the stock one
    std::string file = "vehicle"; // "vehicle" | "gearbox" | "engine" | "turbo"
    std::vector<std::string> pointers; // the whitelist paths (empty for Colour)
    std::string monotonic;             // "decreasing": the resulting list must stay strictly decreasing
    std::vector<SetupPart> parts; // FileChoice: approved files; paths resolved against option-table directory
    // FileChoice whose parts are not listed in the option table but come from a store at run time (engine garage G1):
    // "engines" = every engine of rg::EngineStore (defaults and user engines; ids "default:<id>" / "user:<uid>", a
    // legacy table id such as "sedan_i6" still resolves), "fuels" = the fuels of the installed engine (ids = fuel stems).
    // "gearboxes" = the rg::GearboxCatalog (data/gearboxes/gearbox_catalog.json), offered when the box's torque rating
    // covers the installed engine's peak torque; ids = the catalogue ids. An engine choice also derives the swap
    // package (engine_swap.h); all of them ignore compatible_engines.
    std::string catalog;
    std::vector<std::string> compatible_engines; // original engine stems, an explicit game fitment list; "*" = any engine
    std::string part_format; // required referenced file format
    bool no_turbo_override = false; // default engine turbo must not shadow a vehicle override
    std::string axle;                  // TyreChoice: "front" | "rear"
    // Bool only: a visual part of the car's art (CatalogEntry::model_options), not a file edit - no file, no
    // pointers; offered only on a car whose model carries it, stock = off.
    std::string model_option;
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
};

// One option as the configurator shows it, for one vehicle.
struct OptionView {
    SetupOptionDef def;                // pointers already replaced by the entry's override
    bool available = true;             // false: the entry does not offer it, or its pointers match nothing
    double min = 1.0;                  // effective range (Scale/ScaleList: factor; BrakeBias: front share)
    double max = 1.0;
    SetupValue stock;                  // the value that changes nothing
    std::vector<std::string> choices;  // TyreChoice: candidate tyre ids (compound x allowed sizes)
    std::vector<SetupPart> parts; // available presentation records, including stock
    std::vector<std::string> coming_soon; // a grouped catalog option: the sections that have no entries yet ("Automatic", "CVT")
    int list_size = 0;                 // ScaleList: number of entries
    std::vector<double> stock_numbers; // ScaleList: stock leaf values (gear ratios) for display
};

struct SetupModel {
    std::vector<OptionView> options;
    std::string error; // non-empty: the vehicle's files could not be read
    // With an installed engine/fuel choice (the `installed` setup): what the swap changes, for the player.
    std::vector<std::string> swap_lines;
    double chassis_mass_delta_kg = 0.0;
    std::string engine_title;    // the installed engine's display name ("" = the car's own)
    std::string engine_key;      // the installed engine's EngineStore key (the car's own too); "" = not in the store
};
// Builds the option views of an entry from its files.
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

// Equality of two option values (numbers within 1e-9), used to drop a value set back to stock.
bool setup_values_equal(const SetupValue& a, const SetupValue& b);

// "#rrggbb" check shared by the setup and the UI view-models.
bool is_hex_colour(const std::string& s);

} // namespace rg
