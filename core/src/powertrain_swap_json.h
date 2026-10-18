#pragma once
#include "json_util.h"
#include <string>
namespace rg::detail {
std::string powertrain_kind(const nlohmann::json& vehicle);
std::string swap_powertrain(nlohmann::json& vehicle,const std::string& choice,const std::string& config_path);
}
