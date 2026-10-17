// rg/climate.h - air temperature from date, time and location, and the
// physics weather environment (V8) built from the same file
// (data/world/climate.json, rg.climate/1). Engine-neutral.
//
// The one temperature: climate_temperature_c() is what the weather widget
// shows; make_session_weather()/physics_weather_preset() pin the physics
// environment's station temperature to the same value (temp_override_c), which
// physics lifts to the car's altitude with the same ISA lapse.
#pragma once

#include "rg/sky.h"

#include "ps/environment/climate_desc.h"
#include "ps/environment/environment.h"
#include "ps/environment/weather_desc.h"

#include <array>
#include <map>
#include <optional>
#include <string>

namespace rg {

struct ClimateConfig {
    double station_elevation_m = 0;
    std::array<double, 12> temp_mean_c{}, temp_range_k{}, rh_mean{}, ground_temp_c{};
    double pressure_msl_hpa = 1013.25;
    double lapse_k_per_m = 0.0065;
    double peak_solar_hour = 14.5;
    double cloud_range_reduction = 0.6;
    // Physics environment inputs (V8).
    ps::environment::SurfaceConditionsDesc surface_conditions;
    ps::environment::TyreProfileDesc tyre_profile;
    std::string ground_surface;                 // flat-world ground surface name (tagged so wet grip applies)
    std::uint64_t seed = 1;
    double weather_ramp_s = 60;
    double resync_temperature_k = 0.25, resync_clock_s = 120, resync_clock_real_s = 0.5; // rg::WeatherSync
    std::string physics_blocks_json;            // lags_s/precip_phase/humidity/diurnal/lightning/condition (serialised)
    std::map<std::string, ps::environment::WeatherPresetDesc> weather; // by sky.json weather id
};
std::optional<ClimateConfig> load_climate_config(const std::string& path, std::string* error);

// Daily mean / range at a (fractional, 1-based) day of year, interpolated
// linearly between the monthly midpoints of a non-leap year (cyclic).
double climate_daily_mean_c(const ClimateConfig& c, double day_of_year);
double climate_daily_range_k(const ClimateConfig& c, double day_of_year);

// Air temperature in degC: date_ut/ut_h = the UT instant, longitude for the
// local mean solar time, altitude above sea level, cloud cover 0..1.
double climate_temperature_c(const ClimateConfig& c, const CivilDate& date_ut, double ut_h, double longitude_deg,
                             double altitude_m, double cloud_cover);
// At the station elevation (what the physics preset is pinned to).
inline double climate_station_temperature_c(const ClimateConfig& c, const CivilDate& date_ut, double ut_h,
                                            double longitude_deg, double cloud_cover) {
    return climate_temperature_c(c, date_ut, ut_h, longitude_deg, c.station_elevation_m, cloud_cover);
}

// Physics ClimateDesc: 12 month seasons (real day counts, doy spans, the
// monthly scalars), the location and the physics blocks. nullopt + error on a
// physics validation failure.
std::optional<ps::environment::ClimateDesc> make_physics_climate(const ClimateConfig& c, double latitude_deg,
                                                                 double longitude_deg, std::string* error);

// The physics preset for a sky.json weather id with the temperature pinned;
// nullopt for an id climate.json does not map.
std::optional<ps::environment::WeatherPresetDesc> physics_weather_preset(const ClimateConfig& c, const std::string& weather_id,
                                                                         double station_temp_c);

// Environment time of a UT instant: env seconds since 1 Jan 00:00 local mean
// solar time of the climate calendar's year 0 (one 365-day year; day index =
// day_of_year - 1, 29 Feb shares 28 Feb's index).
struct EnvClockPoint {
    double env_s = 0;
    int day_index = 0;   // [0, 365): the calendar's fixed day
    double solar_tod_s = 0;
};
EnvClockPoint env_clock_point(const CivilDate& date_ut, double ut_h, double longitude_deg);

// What rg::Session installs (SessionConfig::weather): the climate and the
// environment config (calendar locked to the start day, Preset mode). Session
// fills surface_names and altitude_offset_m itself.
struct SessionWeather {
    ps::environment::ClimateDesc climate;
    ps::environment::EnvironmentConfig config;
    std::string ground_surface;
};
std::optional<SessionWeather> make_session_weather(const ClimateConfig& c, double latitude_deg, double longitude_deg,
                                                   double grid_convergence_deg, const CivilDate& date_ut, double ut_h,
                                                   double time_scale, const std::string& weather_id, std::string* error);

} // namespace rg
