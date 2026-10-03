#include "rg/environment.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
namespace rg {
EnvironmentSample sample_environment(const EnvironmentConfig& c,double altitude,double time) {
 if(!c.enabled)return {};
 const double h=c.altitude_density?std::clamp(altitude,-500.0,11000.0):0.0;
 EnvironmentSample s;
 s.temperature_k=c.sea_level_temperature_k-c.lapse_k_per_m*h;
 constexpr double r=287.05287,g=9.80665;
 s.pressure_pa=c.lapse_k_per_m>1e-9?c.sea_level_pressure_pa*std::pow(s.temperature_k/c.sea_level_temperature_k,g/(r*c.lapse_k_per_m)):c.sea_level_pressure_pa*std::exp(-g*h/(r*c.sea_level_temperature_k));
 s.air_density=s.pressure_pa/(r*s.temperature_k);
 const double phase=6.283185307179586*time/c.gust_period_s;
 // Smooth, reproducible wind: simulation time pauses with the terrain gate.
 s.wind_world_m_s=c.wind_world_m_s+ps::Vec3{c.gust_amplitude_m_s.x*std::sin(phase),c.gust_amplitude_m_s.y*std::sin(phase*.731+1.1),c.gust_amplitude_m_s.z*std::sin(phase*.419+2.2)};
 return s;
}
std::optional<EnvironmentConfig> load_environment_config(const std::string& path,std::string* error) {
 try {
  std::ifstream f(path);if(!f)return EnvironmentConfig{};nlohmann::json j;f>>j;
  if(!j.is_object()||j.value("format",std::string{})!="rg.environment/1")throw std::runtime_error("expected rg.environment/1 object");
  EnvironmentConfig c;c.enabled=j.value("enabled",true);c.altitude_density=j.value("altitude_density",true);
  const auto number=[&](const char* key,double fallback,double lo,double hi){double v=j.value(key,fallback);if(!std::isfinite(v)||v<lo||v>hi)throw std::runtime_error(std::string(key)+" outside allowed range");return v;};
  c.sea_level_pressure_pa=number("sea_level_pressure_pa",101325,50000,120000);
  c.sea_level_temperature_k=number("sea_level_temperature_k",288.15,240,330);
  c.lapse_k_per_m=number("lapse_k_per_m",.0065,0,.009);
  c.gust_period_s=number("gust_period_s",12,1,600);
  c.automatic_rear_wing=j.value("automatic_rear_wing",true);
  c.cruise_wing_offset_deg=number("cruise_wing_offset_deg",4,-8,55);
  c.airbrake_wing_offset_deg=number("airbrake_wing_offset_deg",45,0,55);
  c.airbrake_min_speed_m_s=number("airbrake_min_speed_m_s",20,0,150);
  c.airbrake_threshold=number("airbrake_threshold",.45,0,.99);
  c.fan_command=number("fan_command",0,0,1);
  c.fan_battery_energy_j=number("fan_battery_energy_j",0,0,1e9);
  const auto vector=[&](const char* key,bool positive){ps::Vec3 v{};if(!j.contains(key))return v;const auto& a=j.at(key);if(!a.is_array()||a.size()!=3)throw std::runtime_error(std::string(key)+" must contain east, north, up");double values[3];for(int i=0;i<3;++i){values[i]=a[i].get<double>();if(!std::isfinite(values[i])||std::abs(values[i])>100||(positive&&values[i]<0))throw std::runtime_error(std::string(key)+" invalid component");}return ps::Vec3{values[0],values[1],values[2]};};
  c.wind_world_m_s=vector("wind_world_m_s",false);c.gust_amplitude_m_s=vector("gust_amplitude_m_s",true);return c;
 }catch(const std::exception& e){if(error)*error=path+": "+e.what();return {};}
}
}
