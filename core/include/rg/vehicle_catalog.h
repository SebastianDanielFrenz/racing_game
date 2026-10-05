// rg/vehicle_catalog.h - the vehicles the game offers (PLAN.md R6, 11.6).
// Engine-neutral. data/vehicles/catalog.json ("rg.vehicle_catalog/1") lists, per
// entry, the physics vehicle file (inside the pinned physics_sim tree or the
// game's own data/), the model, the display text, the chassis data the vehicle
// file does not carry yet (PLAN.md physics request R5: mass, box and spawn
// height live here as the interim) and which setup options apply.
//
// The numbers vehicle select shows are DATA, never typed into the UI: peak
// torque/power from the engine file (a torque-map engine's WOT curve; a
// simulated engine has no curve, so such an entry declares the figures its
// engine file's own source text cites - flagged `declared` in the stats), the
// mass from the chassis block, the drivetrain layout from the vehicle file's
// shaft graph (which wheels are driven), the gear count from the gearbox file.
//
// Paths in the catalog are relative to the repository root and are resolved at
// load time, so the file works from any checkout location.
#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace rg {

// One option value of a setup overlay (vehicle_setup.h): a number (scale, bias), a
// switch, a string (tyre file stem, "#rrggbb") or a list of numbers (per-gear factors).
using SetupValue = std::variant<double, bool, std::string, std::vector<double>>;

struct VehicleChassis {
    double mass_kg = 1500.0;
    std::array<double, 3> half_extents{2.0, 0.4, 0.15}; // chassis collision box, ISO (x fwd, y left, z up)
    double spawn_z_m = 0.6;                              // chassis height above the ground at spawn
    std::string source;                                  // where the numbers come from
};

// What an engine file does not carry as a curve (a simulated engine): the peak
// figures its own source text cites. Shown with a "declared" marker.
struct DeclaredEngineStats {
    double peak_torque_nm = 0.0;
    double peak_torque_rpm = 0.0;
    double peak_power_kw = 0.0;
    double peak_power_rpm = 0.0;
    std::string source;
};

// An entry is what the player sees as ONE car. A base entry names its own physics
// vehicle file; a preset (R6c) names `preset_of` = a base entry plus a setup overlay
// (`preset_setup`, the same option ids and values as a saved setup) and inherits every
// field it does not set itself (vehicle, model, chassis, engine bay, declared engine,
// colours, body type, manufacturer, displacement, offered options). The player's own
// saved setup stays per entry and is applied on top of the preset's overlay.
struct CatalogEntry {
    std::string id;          // unique, lower_snake
    std::string title;
    std::string subtitle;
    std::string description;
    std::string body_type = "other"; // sedan, hatchback, estate, pickup, suv, sports, hypercar, race, utility, ... (open list, lower_snake)
    std::string manufacturer;        // "" = none given (the browser groups those under "Other")
    std::optional<double> displacement_l; // litres, for an engine file that carries no cylinder geometry
    std::string displacement_source;      // where displacement_l comes from (required with it)
    std::string preset_of;                // "" = a base car; else the base entry's id
    std::map<std::string, SetupValue> preset_setup; // option id -> value, applied before the player's own setup
    std::string vehicle_path; // absolute
    std::string model_path;   // absolute .glb
    std::string sim_name;     // the vehicle file's "name": the key the simulation reports the vehicle under
    VehicleChassis chassis;
    std::string engine_bay = "front"; // "front" | "mid" | "rear": where the garage camera looks for the engine
    std::optional<DeclaredEngineStats> declared_engine;
    std::string default_paint = "#8a8f98"; // visual only
    std::string default_rim = "#c4c8cf";
    // Setup: the option ids this vehicle offers (empty list = every option of the
    // table), pointer overrides per option (a vehicle whose final drive is two
    // differentials, say) and range overrides per option.
    std::vector<std::string> setup_options;
    std::map<std::string, std::vector<std::string>> setup_pointers;
    std::map<std::string, std::pair<double, double>> setup_ranges;
};

struct VehicleCatalog {
    std::string default_id;
    std::vector<CatalogEntry> entries;
    [[nodiscard]] const CatalogEntry* find(const std::string& id) const;
};

// `repo_root`: the directory the catalog's relative paths are relative to.
// Every entry's vehicle file must be readable JSON with a "name" and its model
// must exist; nullopt + *err otherwise.
std::optional<VehicleCatalog> load_vehicle_catalog(const std::string& path, const std::string& repo_root,
                                                   std::string* err);
std::optional<VehicleCatalog> parse_vehicle_catalog(const std::string& json_text, const std::string& repo_root,
                                                    const std::string& origin, std::string* err);

// The persistent engine-map cache directory rg::Session uses for a vehicle file:
// <the tree that holds data/vehicles>/out/godot_engine_cache (physics_sim's demo location).
std::string default_engine_cache_dir(const std::string& vehicle_path);

// The stats vehicle select shows, all read from data.
struct VehicleStats {
    std::string engine_name;
    std::string engine_kind;          // "torque_map" | "simulated"
    double peak_torque_nm = 0.0;
    double peak_torque_rpm = 0.0;
    double peak_power_kw = 0.0;
    double peak_power_rpm = 0.0;
    bool engine_figures_declared = false; // true: from the entry's declared block, not a curve
    double mass_kg = 0.0;
    std::string layout;               // "FWD" | "RWD" | "AWD" | "other"
    int driven_wheels = 0;
    int wheel_count = 0;
    int gear_count = 0;               // forward gears of the gearbox file
    // Displacement in litres: computed from the engine file's cylinders x bore x stroke
    // when it has them, else the catalog entry's own figure, else unknown ("-" in the UI).
    bool displacement_known = false;
    double displacement_l = 0.0;
    std::string displacement_source;  // "engine file: 8 x bore 92 x stroke 95.25 mm" / the catalog's source / ""
};

// Cylinders x pi/4 x bore^2 x stroke of an engine file's JSON text ("bore_mm", "stroke_mm",
// "cylinders"); nullopt when the file has no such geometry (a torque-map or rotary engine).
std::optional<double> displacement_from_engine_json(const std::string& engine_json_text);

// Reads the vehicle file and the engine/gearbox files it references (plain JSON,
// no map generation: instant even for the simulated engines).
std::optional<VehicleStats> compute_vehicle_stats(const CatalogEntry& entry, std::string* err);

} // namespace rg
