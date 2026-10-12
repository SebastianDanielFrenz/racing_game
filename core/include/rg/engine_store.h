// rg/engine_store.h - the engine list of the garage (engine garage G1, out/engine_garage_design.md 4.1-4.3).
// Engine-neutral.
//
// One list: the default engines (data/engines/engine_catalog.json, "rg.engine_catalog/1": id, file, name,
// class, family, mass, peaks, legacy aliases) followed, each directly after its default, by the player's
// user engines (<user dir>/garage/engines/<uid>/{meta.json, engine.json}). A user engine is an engine part
// file that "extends" its default (physics_sim's P2 user layer, RFC 7396 merge patch on top of the base file),
// so editing a default means creating a user engine that holds only what the player changed. Defaults are
// read-only. Keys: "default:<id>" and "user:<uid>"; a bare catalog id or a legacy alias ("sedan_i4",
// "sedan_i6": the ids the first engine_install option used) also resolves to a default.
//
// What the engines say about themselves (idle, limiter, fuels, peak torque of a curve engine) is read through
// the physics loader with `generate = false` (milliseconds, no map generation) - never typed into this repo.
// The peak figures of a simulated (cycle) engine have no curve, so the catalog row carries the ones measured
// with the engine dyno (a hidden test re-measures them); a user engine inherits its base's unless it is a
// curve engine, whose figures come from its own WOT curve.
#pragma once

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rg {

enum class EngineClass { Curve, Mapped, Cycle };
const char* to_string(EngineClass c);

struct EnginePeaks {
    bool known = false;
    double torque_nm = 0.0, torque_rpm = 0.0;
    double power_kw = 0.0, power_rpm = 0.0;
    std::string source; // "wot_curve" | "catalog" (measured with the engine dyno)
};

struct EngineEntry {
    std::string key;         // "default:<id>" | "user:<uid>"
    std::string id;          // catalog id, or the user engine's uid
    bool user = false;
    std::string name;        // display name (a user engine: its own, e.g. "Hyper V8 TT (custom 1)")
    std::string file;        // absolute path of the engine file handed to the physics loader
    std::string base_key;    // user engine: the default it extends; "" for a default
    std::string base_name;   // user engine: that default's name
    std::string base_stem;   // file stem of the default's file (the id engine-bound options are gated on)
    EngineClass cls = EngineClass::Curve;
    std::string family;      // "I4", "I6", "V8 twin turbo", "Rotary" ...
    std::string image;       // res:// path of the card image (shared placeholders until art exists)
    std::string detail;      // one short line for the card
    double mass_kg = 0.0;    // catalog figure (the swap's chassis-mass delta); 0 = unknown
    std::string mass_source;
    std::array<double, 3> crank_axis_local{0.0, 1.0, 0.0};
    EnginePeaks catalog_peaks; // measured once for cycle engines (a curve engine's are read from the file)
    std::vector<std::string> aliases; // legacy ids, defaults only
    std::string created, modified;    // user engines (ISO-like text)
};

// What the physics loader says about one engine file (generate = false).
struct FuelFact {
    std::string name;   // FuelDesc::name == the fuel file's stem for every shipped fuel
    double lhv_mj_kg = 0.0;
    double density_kg_m3 = 0.0;
};
struct EngineFacts {
    bool simulated = false;           // SimulatedEngineDesc (else torque map)
    bool mechanical_throttle = false;
    bool spark = true;
    double idle_rpm = 0.0;
    double limiter_rpm = 0.0;
    double mass_kg = 0.0;             // the file's own mass_kg (0 = none)
    bool has_thermal = false;         // the file (with its extends chain) names a default_thermal (the engine thermal model)
    std::vector<FuelFact> fuels;      // in the engine's list order; [0] is the reference fuel
    EnginePeaks curve_peaks;          // torque map: from the WOT curve; else known = false
};
// Loads `engine_file` through ps::io::load_engine_json (generate = false). nullopt + *err on a loader
// rejection. Results are cached per (path, size, mtime), so repeated reads while editing are cheap.
std::optional<EngineFacts> read_engine_facts(const std::string& engine_file, std::string* err);

class EngineStore {
public:
    // `catalog_path`: data/engines/engine_catalog.json; `repo_root` resolves its relative paths;
    // `user_dir`: "" = no user engines (<user_dir>/garage/engines/ otherwise). nullptr + *err on a bad catalog.
    static std::unique_ptr<EngineStore> open(const std::string& catalog_path, const std::string& repo_root,
                                             const std::string& user_dir, std::string* err);

    // Defaults in catalog order, each followed by its user engines (oldest first).
    [[nodiscard]] const std::vector<EngineEntry>& list() const { return entries_; }
    // "default:<id>", "user:<uid>", a bare catalog id or a legacy alias; nullptr when unknown.
    [[nodiscard]] const EngineEntry* find(const std::string& key) const;
    // The entry whose file is `path` (normalised), or nullptr.
    [[nodiscard]] const EngineEntry* find_by_file(const std::string& path) const;
    // The directory the fuel files of the engines are found in (an engine's fuel name + ".json").
    [[nodiscard]] const std::string& fuel_dir() const { return fuel_dir_; }
    [[nodiscard]] std::string fuel_file(const std::string& fuel_name) const;

    // Peak figures to show/derive from: a curve engine reads its own file, a cycle engine uses the catalog
    // figures of its default (a user engine inherits them).
    [[nodiscard]] EnginePeaks peaks(const EngineEntry& e, std::string* err = nullptr) const;

    // ---- user engines ----
    // Creates "user:<uid>" extending `base_key`'s default with the merge patch `patch_json` ("{}" = a plain
    // copy). The result is validated with the physics loader (generate = false) BEFORE anything is written; a
    // rejection is returned in *err and nothing is created. Returns the new key.
    std::optional<std::string> create_user_engine(const std::string& base_key, const std::string& name,
                                                  const std::string& patch_json, std::string* err);
    // Replaces the merge patch of a user engine (validated first).
    bool update_user_engine(const std::string& key, const std::string& patch_json, std::string* err);
    bool rename_user_engine(const std::string& key, const std::string& name, std::string* err);
    bool remove_user_engine(const std::string& key, std::string* err);
    // The patch text a user engine holds (without its "extends"), "" when unknown.
    [[nodiscard]] std::string user_patch(const std::string& key) const;
    // Re-reads the user directory (other RgGarage instances, tests).
    void reload_user_engines();

private:
    EngineStore() = default;
    void rebuild();
    std::string user_root() const;

    std::string repo_root_;
    std::string user_dir_;
    std::string fuel_dir_;
    std::vector<EngineEntry> defaults_;
    std::vector<EngineEntry> users_;
    std::vector<EngineEntry> entries_;
};

} // namespace rg
