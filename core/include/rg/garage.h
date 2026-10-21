// rg/garage.h - rg::Garage: the garage's model (PLAN.md R6, 11.6). Engine-neutral.
// Owns the vehicle catalog, the setup option table and the garage set data, the
// persistence of "which car is selected" and one saved setup per vehicle (JSON
// under <user dir>/garage/), the edit session of the configurator (a working
// copy that is validated with the physics loader on every change and can only
// be saved while valid) and the hand-over to the drive: prepare_drive()
// materialises the saved setup into a work directory and names everything the
// world load needs (vehicle file, model, chassis, assist defaults, colours).
// The Godot layer (RgGarage + garage_*.gd) shows and forwards; it decides nothing.
//
// Temp files: validations use scratch directories that remove themselves; the
// one materialisation a drive uses lives in <work_root>/drive_* and is removed by
// cleanup() / the destructor, and a stale drive_* directory left by a crashed
// run is removed when the garage opens.
#pragma once

#include "rg/damage.h"
#include "rg/car_browser.h"
#include "rg/engine_store.h"
#include "rg/garage_set.h"
#include "rg/strip_visual.h"
#include "rg/vehicle_catalog.h"
#include "rg/vehicle_setup.h"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rg {

struct GarageConfig {
    std::string repo_root;     // the catalog's relative paths are relative to this
    std::string catalog_path;  // data/vehicles/catalog.json
    std::string options_path;  // data/vehicles/setup_options.json
    std::string set_path;      // data/garage/garage_set.json
    std::string engine_catalog_path; // data/engines/engine_catalog.json ("" = <repo_root>/data/engines/engine_catalog.json)
    std::string gearbox_catalog_path; // data/gearboxes/gearbox_catalog.json ("" = <repo_root>/data/gearboxes/gearbox_catalog.json)
    std::string driveline_catalog_path; // data/drivetrains/driveline_catalog.json ("" = <repo_root>/data/drivetrains/driveline_catalog.json)
    std::string tyre_families_path; // data/tyres/families.json ("" = no tyre choice)
    std::string turbo_map_extension_path; // data/engines/turbo_map_extension.json ("" = <repo_root>/data/engines/turbo_map_extension.json if it exists, else no extension)
    std::string turbo_kits_path; // data/engines/turbo_kits.json ("" = <repo_root>/data/engines/turbo_kits.json if it exists, else no turbo kits)
    std::string engine_internals_path; // data/engines/engine_internals.json ("" = <repo_root>/data/engines/engine_internals.json if it exists, else no internals options)
    std::string cooling_upgrades_path; // data/cooling_systems/cooling_upgrades.json ("" = <repo_root>/data/cooling_systems/cooling_upgrades.json if it exists, else no cooling options)
    std::string part_masses_path; // data/vehicles/part_masses.json ("" = <repo_root>/data/vehicles/part_masses.json if it exists, else no part-mass tree)
    std::string damage_config_path; // data/vehicles/damage.json ("" = <repo_root>/data/vehicles/damage.json if it exists, else no damage model)
    std::string strip_visual_path; // data/vehicles/strip_visual.json ("" = <repo_root>/data/vehicles/strip_visual.json if it exists, else nothing is hidden)
    std::string user_dir;      // selection and setups are saved under <user_dir>/garage/
    std::string work_root;     // scratch and drive materialisations
    std::string engine_map_cache_dir; // .psmaps cache for the loader ("" = the per-vehicle default rg::Session uses)
};

// Everything the world load needs for the chosen car.
struct DriveSelection {
    bool ok = false;
    std::string error;
    std::string vehicle_id;
    std::string sim_name;      // the key the simulation reports the vehicle under
    std::string vehicle_path;  // file to hand to the simulation (the original, or the materialised setup)
    std::string model_path;
    VehicleChassis chassis;
    std::string paint;         // "#rrggbb", the setup's or the catalog default
    std::string rim;
    std::vector<std::string> model_options; // visual model parts switched on by the setup (e.g. "roof")
    std::vector<std::string> hidden_nodes;  // model nodes the setup's weight reduction takes off the car (rg/strip_visual.h)
    std::string engine_map_cache_dir; // the .psmaps cache the world load should use (the vehicle's own default)
    std::string turbo_map_extension_path; // the turbo map extension policy the world load should use ("" = none)
    bool modified = false;     // a saved setup is applied
    bool assist_auto_clutch = true;  // the vehicle file's controller defaults (after the setup)
    bool assist_auto_blip = true;
    bool assist_auto_shift = false;
    bool manual_gearbox = true;      // the vehicle has a manual_tcu controller
    std::string engine_title;        // the installed engine's name when the setup swapped it ("" = the car's own)
    std::vector<std::string> swap_lines; // what the engine/fuel choice changed (for the wait screen and the garage)
    PartTree parts;            // the driven car's part-mass tree (rg/part_mass.h); when parts.ok, chassis.mass_kg == parts.chassis_kg
    std::string warning;       // e.g. the saved setup no longer validates and the stock car is used
    // Damage model (rg/damage.h): the rules file and this car's saved health (<user dir>/garage/damage/<id>.json, which
    // may not exist: a healthy car). Both "" = no damage model for this drive.
    std::string damage_config_path;
    std::string damage_state_path;
};

struct EditResult {
    bool accepted = false;       // the option exists, is offered for this car and the value has the right type
    std::string message;         // why not accepted
    std::string notice;          // accepted, but other options were reset because the new engine does not offer them
    ValidationResult validation; // the whole working setup after the change
};

class Garage {
public:
    // nullptr + *err when the catalog, option table or set data cannot be loaded.
    static std::unique_ptr<Garage> open(const GarageConfig& config, std::string* err);
    ~Garage();
    Garage(const Garage&) = delete;
    Garage& operator=(const Garage&) = delete;

    [[nodiscard]] const VehicleCatalog& catalog() const { return catalog_; }
    [[nodiscard]] const SetupOptionTable& options() const { return options_; }
    [[nodiscard]] const GarageSetDesc& set() const { return set_; }
    [[nodiscard]] const GarageConfig& config() const { return config_; }
    [[nodiscard]] const SetupContext& context() const { return ctx_; }
    // ---- the engine list (defaults + user engines, one list) ----
    [[nodiscard]] const EngineStore& engines() const { return *store_; }
    // User engines: the store's validation and file rules; the setup context's snapshot and the edit session follow.
    std::optional<std::string> create_user_engine(const std::string& base_key, const std::string& name,
                                                  const std::string& patch_json, std::string* err);
    bool update_user_engine(const std::string& key, const std::string& patch_json, std::string* err);
    bool rename_user_engine(const std::string& key, const std::string& name, std::string* err);
    // Removes the engine; a car whose working or saved setup used it falls back to its own engine (a notice at the drive).
    bool remove_user_engine(const std::string& key, std::string* err);
    // The setup context for one vehicle (its engine-map cache directory filled in).
    [[nodiscard]] SetupContext context_for(const CatalogEntry& entry) const;

    // ---- selection (persisted) ----
    [[nodiscard]] const std::string& selected_id() const { return selected_; }
    bool select(const std::string& id, std::string* err); // persists
    // ---- per-vehicle data ----
    // mass_kg is the car's part-mass tree total with its setup applied (a preset: its overlay, owner on PR #22), the
    // catalog chassis.mass_kg only when the tree cannot be built.
    // The stats follow the car's drive layout as the setup leaves it (the working copy while that car is edited, else the
    // saved setup on top of the preset): a drivetrain swap changes layout, driven_wheels and driveline.
    [[nodiscard]] std::optional<VehicleStats> stats(const std::string& id, std::string* err) const;
    // The engine the garage's bay view shows for this car: the edit session's installed engine while that car is
    // edited, else the car's own. nullptr when the engine is not in the store.
    [[nodiscard]] const EngineEntry* bay_engine(const std::string& id) const;
    // The car's geometry for the garage camera; `model_min/max` (car frame, from the
    // loaded model's bounds) override the chassis-box based estimate when given.
    [[nodiscard]] std::optional<GarageSubject> subject(const std::string& id, const GVec3* model_min,
                                                       const GVec3* model_max, std::string* err) const;
    // The player's saved setup file of this car (empty values when none).
    [[nodiscard]] VehicleSetup saved_setup(const std::string& id) const;
    // A preset entry's own overlay (empty for a base car) and the setup a drive/edit starts from:
    // the preset's overlay with the player's saved values on top.
    [[nodiscard]] VehicleSetup preset_setup(const std::string& id) const;
    [[nodiscard]] VehicleSetup effective_setup(const std::string& id) const;
    [[nodiscard]] bool has_saved_setup(const std::string& id) const;
    [[nodiscard]] std::string setup_path(const std::string& id) const;

    // ---- the car browser (R6c) ----
    // The model of the browser screen: one BrowserCar per catalog entry (a preset is a car of its own),
    // with the stats of its vehicle file and the paint/rim a drive of it would use (the saved setup on top
    // of the preset overlay). The view state (group/sort/filter) lives in the model; the caller persists it
    // through CarBrowser::apply_settings / store_settings. The stats are those of the entry's vehicle file:
    // a preset's overlay is not applied to them, except the drive layout, which follows a drivetrain swap (stats()).
    [[nodiscard]] CarBrowser& browser() { return browser_; }
    [[nodiscard]] const CarBrowser& browser() const { return browser_; }
    [[nodiscard]] std::vector<BrowserCar> browser_cars() const;
    // Rebuilds the browser's cars (after a setup was saved: the paint may have changed).
    void refresh_browser();

    // ---- the configurator's edit session ----
    bool begin_edit(const std::string& id, std::string* err); // loads the saved setup as the working copy
    [[nodiscard]] bool editing() const { return !edit_id_.empty(); }
    [[nodiscard]] const std::string& edit_id() const { return edit_id_; }
    [[nodiscard]] std::vector<SetupPart> powertrain_engine_choices() const;
    [[nodiscard]] const SetupModel& model() const { return model_; }
    [[nodiscard]] const VehicleSetup& working() const { return working_; }
    // The working value of an option, or its stock value when unchanged.
    [[nodiscard]] SetupValue current_value(const std::string& option_id) const;
    EditResult set_option(const std::string& option_id, const SetupValue& value);
    // What the installed engine/fuel choice changes (the swap package's lines), for the garage screen.
    [[nodiscard]] const std::vector<std::string>& swap_lines() const { return model_.swap_lines; }
    [[nodiscard]] double chassis_mass_delta_kg() const { return model_.chassis_mass_delta_kg; }
    // The part-mass tree of the working copy (the whole setup, compiled; the installed parts only while the working copy
    // does not compile) - what the garage's weight breakdown shows. !ok without a part mass table or a body mass.
    [[nodiscard]] const PartTree& parts() const { return parts_; }
    // The part-mass tree of a car as it would be driven (its preset overlay and saved setup applied), for screens outside
    // the edit session.
    [[nodiscard]] PartTree part_tree(const std::string& id) const;
    // The car's stock part-mass tree (its base vehicle file, no setup; a preset: its base car without the overlay).
    [[nodiscard]] PartTree stock_part_tree_of(const std::string& id) const;
    void reset_option(const std::string& option_id);
    void reset_all();
    // The working setup as it WOULD be after the player made these edits one after the other (set_option's own rules: the
    // options an engine/gearbox/layout change resets go back to stock), without changing the working copy, the validation or
    // anything else: the whole-car candidate of an upgrade hint (rg/upgrade_hints.h). The edits are applied without
    // validating the result (nothing is loaded; a candidate the loader refuses is found out when its figures are computed).
    // nullopt + *err when no car is edited or an edit is not accepted (unknown/unavailable option, wrong value type).
    [[nodiscard]] std::optional<VehicleSetup> candidate_setup(const std::vector<std::pair<std::string, SetupValue>>& edits,
                                                              std::string* err = nullptr);
    // true when every edit selects what is installed NOW (the option's current value): such a click changes nothing, so
    // the tile of the installed part has no upgrade hint (rule of the core, not of the UI). candidate_setup of such edits
    // is the working copy itself, bit for bit.
    [[nodiscard]] bool edits_installed(const std::vector<std::pair<std::string, SetupValue>>& edits) const;
    [[nodiscard]] bool dirty() const; // the working copy differs from the saved one
    [[nodiscard]] const ValidationResult& validation() const { return validation_; }
    // Writes the working copy as the vehicle's saved setup (a working copy equal to the car's own
    // preset overlay - empty for a base car - removes the file). false + *err while the working copy does not validate or the write fails.
    bool save(std::string* err);
    void discard(); // ends the session, keeps nothing
    // Paint/rim of the working copy (visual only).
    [[nodiscard]] std::string working_paint() const;
    [[nodiscard]] std::string working_rim() const;
    // The model parts the working setup switches on (setup options with a model_option, e.g. "roof").
    [[nodiscard]] std::vector<std::string> working_model_options() const;
    // The model nodes the working setup's weight reduction takes off the car (rg/strip_visual.h; empty without the table).
    [[nodiscard]] std::vector<std::string> working_hidden_nodes() const;

    // ---- damage (rg/damage.h; owner: damage persists until repaired, repairs are free and need the player's action) ----
    [[nodiscard]] const DamageConfig* damage_config() const { return damage_ ? &*damage_ : nullptr; }
    // <user dir>/garage/damage/<id>.json ("" without a damage model or a user dir).
    [[nodiscard]] std::string damage_state_path(const std::string& id) const;
    // The car's saved part health (healthy when no file; a broken file reads healthy and sets *err).
    [[nodiscard]] DamageState damage_state(const std::string& id, std::string* err = nullptr) const;
    // Repair (free): every part back to health 1, i.e. the state file is removed. false + *err on an I/O error.
    bool repair_damage(const std::string& id, std::string* err);

    // ---- hand-over to the drive ----
    DriveSelection prepare_drive(const std::string& id);
    // Removes the drive materialisation and any scratch state.
    void cleanup();

private:
    Garage() = default;
    [[nodiscard]] std::string garage_dir() const;
    bool write_selection(std::string* err) const;
    void revalidate();
    void remove_drive_dir();
    void engines_changed();
    // gearbox_change: the probe also installs the chosen gearbox (an automatic drops the clutch and manual-TCU options).
    std::string drop_unavailable_values(bool gearbox_change = false);
    // After a drive layout change: an axle differential choice for an axle the new layout does not drive returns to stock.
    std::string drop_driveline_values();
    std::string drop_driveline_values_after_conversion();
    // The overlay with legacy engine ids resolved to keys; an engine that no longer exists is dropped (named in *notices).
    void normalise_engine_values(VehicleSetup& setup, std::vector<std::string>* notices) const;
    [[nodiscard]] VehicleSetup effective_setup_with_notices(const std::string& id, std::vector<std::string>* notices) const;

    GarageConfig config_;
    VehicleCatalog catalog_;
    SetupOptionTable options_;
    GarageSetDesc set_;
    SetupContext ctx_;
    std::string turbo_map_extension_path_; // the loaded policy file, "" = none
    std::optional<DamageConfig> damage_;    // data/vehicles/damage.json, nullopt = no damage model
    std::string damage_config_path_;
    std::shared_ptr<const StripVisualTable> strip_visual_; // null = nothing is hidden
    [[nodiscard]] std::vector<std::string> hidden_nodes_for(const CatalogEntry& e, const VehicleSetup& setup) const;
    std::shared_ptr<EngineStore> store_;
    std::string selected_;

    std::string edit_id_;
    SetupModel model_;
    VehicleSetup working_;
    VehicleSetup saved_at_begin_;
    ValidationResult validation_;
    PartTree parts_;
    bool probe_only_ = false; // candidate_setup: revalidate() rebuilds the model but loads and validates nothing

    std::string drive_dir_;
    CarBrowser browser_;
    mutable std::map<std::string, VehicleStats> stats_cache_; // by vehicle file + declared displacement (the files do not change while the game runs)
    mutable std::map<std::string, double> mass_cache_; // the part-tree total by car id + setup JSON (stats mass_kg)
};

} // namespace rg
