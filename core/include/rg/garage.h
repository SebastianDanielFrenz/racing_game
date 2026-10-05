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

#include "rg/car_browser.h"
#include "rg/garage_set.h"
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
    std::vector<std::string> tyre_dirs;
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
    std::string engine_map_cache_dir; // the .psmaps cache the world load should use (the vehicle's own default)
    bool modified = false;     // a saved setup is applied
    bool assist_auto_clutch = true;  // the vehicle file's controller defaults (after the setup)
    bool assist_auto_blip = true;
    bool assist_auto_shift = false;
    bool manual_gearbox = true;      // the vehicle has a manual_tcu controller
    std::string warning;       // e.g. the saved setup no longer validates and the stock car is used
};

struct EditResult {
    bool accepted = false;       // the option exists, is offered for this car and the value has the right type
    std::string message;         // why not accepted
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
    // The setup context for one vehicle (its engine-map cache directory filled in).
    [[nodiscard]] SetupContext context_for(const CatalogEntry& entry) const;

    // ---- selection (persisted) ----
    [[nodiscard]] const std::string& selected_id() const { return selected_; }
    bool select(const std::string& id, std::string* err); // persists
    // ---- per-vehicle data ----
    [[nodiscard]] std::optional<VehicleStats> stats(const std::string& id, std::string* err) const;
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
    // a preset's overlay is not applied to them.
    [[nodiscard]] CarBrowser& browser() { return browser_; }
    [[nodiscard]] const CarBrowser& browser() const { return browser_; }
    [[nodiscard]] std::vector<BrowserCar> browser_cars() const;
    // Rebuilds the browser's cars (after a setup was saved: the paint may have changed).
    void refresh_browser();

    // ---- the configurator's edit session ----
    bool begin_edit(const std::string& id, std::string* err); // loads the saved setup as the working copy
    [[nodiscard]] bool editing() const { return !edit_id_.empty(); }
    [[nodiscard]] const std::string& edit_id() const { return edit_id_; }
    [[nodiscard]] const SetupModel& model() const { return model_; }
    [[nodiscard]] const VehicleSetup& working() const { return working_; }
    // The working value of an option, or its stock value when unchanged.
    [[nodiscard]] SetupValue current_value(const std::string& option_id) const;
    EditResult set_option(const std::string& option_id, const SetupValue& value);
    void reset_option(const std::string& option_id);
    void reset_all();
    [[nodiscard]] bool dirty() const; // the working copy differs from the saved one
    [[nodiscard]] const ValidationResult& validation() const { return validation_; }
    // Writes the working copy as the vehicle's saved setup (a working copy equal to the car's own
    // preset overlay - empty for a base car - removes the file). false + *err while the working copy does not validate or the write fails.
    bool save(std::string* err);
    void discard(); // ends the session, keeps nothing
    // Paint/rim of the working copy (visual only).
    [[nodiscard]] std::string working_paint() const;
    [[nodiscard]] std::string working_rim() const;

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

    GarageConfig config_;
    VehicleCatalog catalog_;
    SetupOptionTable options_;
    GarageSetDesc set_;
    SetupContext ctx_;
    std::string selected_;

    std::string edit_id_;
    SetupModel model_;
    VehicleSetup working_;
    VehicleSetup saved_at_begin_;
    ValidationResult validation_;

    std::string drive_dir_;
    CarBrowser browser_;
    mutable std::map<std::string, VehicleStats> stats_cache_; // by vehicle file + declared displacement (the files do not change while the game runs)
};

} // namespace rg
