// driveline_swap_json.h - internal: the drivetrain swap on a parsed vehicle document (rg/driveline_swap.h has the
// text API and the rules). Used by vehicle_setup.cpp (the setup's working copy) and vehicle_catalog.cpp (stats).
#pragma once

#include "rg/driveline_swap.h"

#include "json_util.h"

namespace rg::detail {

DrivelineInfo describe_driveline_json(const json& vehicle);
// "" when applied, else the reason. `vehicle` is unchanged on failure.
std::string apply_layout_json(json& vehicle, const DrivelineCatalog& catalog, const DriveLayoutEntry& layout);
// `*index` (optional) = the edited component's index in /powertrain/components.
std::string apply_axle_diff_json(json& vehicle, bool front, const DiffUnit& unit, int* index = nullptr);

} // namespace rg::detail
