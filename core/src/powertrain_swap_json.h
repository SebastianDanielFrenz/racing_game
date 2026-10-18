#pragma once
#include "json_util.h"
#include <string>
namespace rg::detail {
std::string powertrain_kind(const nlohmann::json& vehicle);
std::string swap_powertrain(nlohmann::json& vehicle,const std::string& choice,const std::string& config_path,const std::string& vehicle_path);
// The assist unit of a hybrid-converted car: `own` (effective json, extends resolved) with the traction_control of `hybrid`, no launch control.
nlohmann::json merge_hybrid_unit(const nlohmann::json& own,const nlohmann::json& hybrid);
}
