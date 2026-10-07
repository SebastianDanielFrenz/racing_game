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
//      vehicle file, or the gearbox/engine/tyre file it references);
//   2. the difference to the original file is written as an RFC 7396 JSON merge
//      patch (arrays replace whole, as that RFC defines) and applied to the
//      original - the merge patch is the mechanism, the edit step only builds it;
//   3. the whitelist is enforced on the result: every changed leaf must be one the
//      enabled options may touch;
//   4. the patched files are materialised into a work directory (a patched file
//      is written there, every relative file reference in it rewritten to an
//      absolute path; an untouched referenced file is referenced in place) and
//      loaded with ps::io::load_vehicle_json. Whatever the loader says is the
//      verdict: a setup the loader rejects is shown with the loader's message and
//      cannot be saved.
// The work directory's files are removed again (ScopedWorkDir) - except the one
// materialisation a drive uses, which the Garage owns and removes on cleanup().
//
// v1 options: final drive, per-gear ratios (kept strictly decreasing), brake
// bias (same total) and brake force, spring/damper/anti-roll bar per axle,
// tyre choice per axle (tyre files that match the wheel size), the three
// assists (their defaults in the controller block), paint and rim colour
// (visual only, no file edit). ABS/TC/ESC follow once physics has them.
#pragma once

#include "rg/vehicle_catalog.h"

#include <map>
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
    TyreChoice, // string: a tyre file (by file stem) that matches the axle's wheels
    Bool,       // bool: a target leaf set to it
    Colour,     // string "#rrggbb": visual only, edits no file
};

const char* to_string(OptionKind k);

struct SetupPart {
    std::string id, label, path, image, detail;
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
    std::string file = "vehicle"; // "vehicle" | "gearbox" | "engine" | "tyre"
    std::vector<std::string> pointers; // the whitelist paths (empty for Colour)
    std::string monotonic;             // "decreasing": the resulting list must stay strictly decreasing
    std::vector<SetupPart> parts; // FileChoice: approved files; paths resolved against option-table directory
    std::vector<std::string> compatible_engines; // original engine stems, an explicit game fitment list
    std::string part_format; // required referenced file format
    bool no_turbo_override = false; // default engine turbo must not shadow a vehicle override
    std::string axle;                  // TyreChoice: "front" | "rear"
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
// Parses a saved setup; every option id must exist in `table` and carry the
// value type its kind needs (nullopt + *err otherwise). Range checks happen when
// the setup is applied, not here.
std::optional<VehicleSetup> parse_setup(const std::string& json_text, const SetupOptionTable& table,
                                        const std::string& origin, std::string* err);

// Where the files an option may edit are found.
struct SetupContext {
    std::vector<std::string> tyre_dirs; // directories scanned for *.json tyre files (TyreChoice)
    std::string work_root;              // materialisations and validation temp dirs go under here
    std::string engine_map_cache_dir;   // .psmaps cache for load_vehicle_json ("" = no disk cache)
};

// One option as the configurator shows it, for one vehicle.
struct OptionView {
    SetupOptionDef def;                // pointers already replaced by the entry's override
    bool available = true;             // false: the entry does not offer it, or its pointers match nothing
    double min = 1.0;                  // effective range (Scale/ScaleList: factor; BrakeBias: front share)
    double max = 1.0;
    SetupValue stock;                  // the value that changes nothing
    std::vector<std::string> choices;  // TyreChoice: candidate file stems
    std::vector<SetupPart> parts; // available presentation records, including stock
    int list_size = 0;                 // ScaleList: number of entries
    std::vector<double> stock_numbers; // ScaleList: stock leaf values (gear ratios) for display
};

struct SetupModel {
    std::vector<OptionView> options;
    std::string error; // non-empty: the vehicle's files could not be read
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
};
CompiledSetup compile_setup(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx,
                            const VehicleSetup& setup);

// Checks `patch` (merge patch text) against the leaves `pointers` may change, for
// the file text `base_json`: every leaf the patch changes must be reachable by one
// of the patterns. "" when fine, else the offending path. Exposed for tests.
std::string whitelist_violation(const std::string& base_json, const std::string& patch_json,
                                const std::vector<std::string>& pointer_patterns);
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
                                const VehicleSetup& setup);

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
