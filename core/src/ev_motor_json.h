// ev_motor_json.h - internal: the EV motor layout/unit transforms on a parsed vehicle document (rg/ev_motor.h has the
// text API and the rules). Used by vehicle_setup.cpp (the setup's working copy).
#pragma once

#include "rg/ev_motor.h"

#include "json_util.h"

namespace rg::detail {

EvMotorInfo describe_ev_motors_json(const json& vehicle, const EvMotorCatalog* catalog);
// "" when applied, else the reason. `vehicle` is unchanged on failure. A layout the car already has changes nothing.
std::string apply_ev_layout_json(json& vehicle, const EvLayoutEntry& layout);
// Sets the unit's figures on every motor of one axle and its reduction on every reduction of that axle.
// `touched` (optional) receives the indices in /powertrain/components that were edited.
std::string apply_ev_unit_json(json& vehicle, bool front, const EvMotorUnit& unit, std::vector<int>* touched = nullptr);

// The motors of one axle as a unit (the figures of the first motor of that axle, else of the other axle) with the reduction
// ratio/efficiency of its reduction; nullopt when the car is not swappable. id/label/inspiration/source stay empty.
std::optional<EvMotorUnit> ev_axle_unit_json(const json& vehicle, bool front);

} // namespace rg::detail
