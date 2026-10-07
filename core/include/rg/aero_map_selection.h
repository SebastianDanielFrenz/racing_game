#pragma once
#include "ps/vehicle/vehicle_desc.h"
#include <string>
namespace rg {
// Startup-only; disabled selection is cheap. Enabled selection requires an
// explicitly reviewed sidecar bound to exact coefficient/geometry bytes.
void apply_aero_map_selection(const std::string& selection_path,ps::vehicle::VehicleDesc& vehicle);
}
