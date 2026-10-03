#pragma once
#include "ps/math/vec3.h"
#include <optional>
#include <string>
namespace rg {
struct EnvironmentConfig {
 bool enabled=false,altitude_density=true;
 double sea_level_pressure_pa=101325,sea_level_temperature_k=288.15,lapse_k_per_m=.0065;
 ps::Vec3 wind_world_m_s{},gust_amplitude_m_s{};
 double gust_period_s=12;
 bool automatic_rear_wing=true;
 double cruise_wing_offset_deg=4,airbrake_wing_offset_deg=45;
 double airbrake_min_speed_m_s=20,airbrake_threshold=.45;
 double fan_command=0,fan_battery_energy_j=0;
};
struct EnvironmentSample {double pressure_pa=101325,temperature_k=288.15,air_density=1.225;ps::Vec3 wind_world_m_s{};};
EnvironmentSample sample_environment(const EnvironmentConfig&,double altitude_m,double sim_time_s);
std::optional<EnvironmentConfig> load_environment_config(const std::string& path,std::string* error);
}
