// rg/vehicle_load.h - loading a racing_game vehicle file with its universal tyres.
//
// The physics loader (ps::io::load_vehicle_json) only knows tyre FILES. A racing_game vehicle file instead carries
// a tyre SPEC per wheel (`"tyre_spec": {"compound": "road", "width_mm": 265, "aspect": 35, "rim_in": 20}`) and a
// top-level `"tyre_families"` path (relative to the vehicle file) to data/tyres/families.json; the wheel has no
// "tyre" key. rg::load_vehicle builds each wheel's tyre with TyreFamilies::make(spec), in memory, nothing written to
// disk, and hands them to the physics loader through ps::io::VehicleLoadOptions::tyres (keyed by the wheel's "name").
// EVERY load of a racing_game vehicle file must go through here: since physics_sim 6c5bb87 a raw
// ps::io::load_vehicle_json of such a file is a load error ("wheel '<name>': no tyre").
// A wheel with a "tyre" key (a physics_sim vehicle file) is left exactly as the physics loader made it.
#pragma once

#include "ps/io/vehicle_io.h"
#include "rg/tyre_spec.h"

#include <string>
#include <vector>

namespace rg {

// What the vehicle file declares for its wheels' tyres. Empty `wheels` entries (compound empty) = no spec.
struct VehicleTyreSpecs {
    std::string families_path;        // absolute/resolved path of families.json ("" = the file has no specs)
    std::vector<TyreSpec> wheels;     // one per wheel, in file order; compound empty = the wheel has no spec
    std::vector<std::string> names;   // the wheel names (physics_sim's WheelDesc::name, "wheel" when absent), parallel to `wheels`
    std::string error;                // non-empty: the file's tyre_spec/tyre_families entries are malformed
};
VehicleTyreSpecs read_vehicle_tyre_specs(const std::string& vehicle_path);

// ps::io::load_vehicle_json + the tyre injection. Throws std::runtime_error with the loader's message.
ps::vehicle::VehicleDesc load_vehicle(const std::string& path);
ps::vehicle::VehicleDesc load_vehicle(const std::string& path, const ps::io::EngineMapOptions& options);
ps::vehicle::VehicleDesc load_vehicle(const std::string& path, const ps::io::VehicleLoadOptions& options);

} // namespace rg
