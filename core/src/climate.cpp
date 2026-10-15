// climate.cpp - see rg/climate.h.
#include "rg/climate.h"

#include "json_util.h"

#include "ps/environment/climate_io.h"
#include "ps/environment/surface_conditions_io.h"
#include "ps/environment/tyre_profile_io.h"
#include "ps/environment/weather_io.h"

#include <cmath>
#include <exception>
#include <filesystem>

namespace rg {
namespace {

using detail::json;
constexpr double kPi = 3.14159265358979323846;
constexpr std::array<int, 12> kMonthDays{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
constexpr const char* kMonthNames[12] = {"jan", "feb", "mar", "apr", "may", "jun",
                                         "jul", "aug", "sep", "oct", "nov", "dec"};

bool fail(std::string* error, const std::string& msg) {
    if (error) *error = "climate: " + msg;
    return false;
}

bool read_array12(const json& o, const char* key, std::array<double, 12>& out, std::string* error) {
    const auto it = o.find(key);
    if (it == o.end() || !it->is_array() || it->size() != 12) return fail(error, std::string("monthly.") + key + " must be 12 numbers");
    for (std::size_t i = 0; i < 12; ++i) {
        if (!(*it)[i].is_number()) return fail(error, std::string("monthly.") + key + " must be 12 numbers");
        out[i] = (*it)[i].get<double>();
        if (!std::isfinite(out[i])) return fail(error, std::string("monthly.") + key + " must be finite");
    }
    return true;
}

bool read_number(const json& o, const char* key, double& out, std::string* error, const char* ctx) {
    if (!detail::get_number(o, key, out) || !std::isfinite(out)) return fail(error, std::string(ctx) + key + " must be a number");
    return true;
}

// Day-of-year (1-based, non-leap, fractional) of each month's midpoint.
double month_mid(int m) {
    int start = 0;
    for (int i = 0; i < m; ++i) start += kMonthDays[static_cast<std::size_t>(i)];
    return start + 0.5 * kMonthDays[static_cast<std::size_t>(m)] + 0.5;
}

double interp_monthly(const std::array<double, 12>& v, double doy) {
    // Cyclic over 365 days between the 12 midpoints.
    double d = std::fmod(doy - 1.0, 365.0);
    if (d < 0) d += 365.0;
    d += 1.0;
    for (int m = 0; m < 12; ++m) {
        const int n = (m + 1) % 12;
        const double a = month_mid(m);
        double b = month_mid(n);
        double x = d;
        if (n == 0) { b += 365.0; if (x < a) x += 365.0; }
        if (x >= a && x < b) {
            const double t = (x - a) / (b - a);
            return v[static_cast<std::size_t>(m)] + t * (v[static_cast<std::size_t>(n)] - v[static_cast<std::size_t>(m)]);
        }
    }
    return v[11]; // unreachable: every day lies in one span
}

int nonleap_day_of_year(const CivilDate& d) {
    int doy = day_of_year(d);
    const bool leap = days_in_month(d.year, 2) == 29;
    if (leap && doy >= 60) doy -= 1;
    return doy;
}

} // namespace

std::optional<ClimateConfig> load_climate_config(const std::string& path, std::string* error) {
    const auto text = detail::read_text_file(path);
    if (!text) { fail(error, "cannot read " + path); return std::nullopt; }
    const auto root = detail::parse_json(*text);
    if (!root || !root->is_object()) { fail(error, path + " is not a JSON object"); return std::nullopt; }
    std::string format;
    if (!detail::get_string(*root, "format", format) || format != "rg.climate/1") { fail(error, "format must be rg.climate/1"); return std::nullopt; }
    ClimateConfig c;
    const auto st = root->find("station");
    if (st == root->end() || !st->is_object() || !read_number(*st, "elevation_m", c.station_elevation_m, error, "station.")) return std::nullopt;
    const auto mo = root->find("monthly");
    if (mo == root->end() || !mo->is_object()) { fail(error, "monthly missing"); return std::nullopt; }
    if (!read_array12(*mo, "temp_mean_c", c.temp_mean_c, error) || !read_array12(*mo, "temp_range_k", c.temp_range_k, error) ||
        !read_array12(*mo, "rh_mean", c.rh_mean, error) || !read_array12(*mo, "ground_temp_c", c.ground_temp_c, error) ||
        !read_number(*mo, "pressure_msl_hpa", c.pressure_msl_hpa, error, "monthly."))
        return std::nullopt;
    for (int i = 0; i < 12; ++i) {
        if (c.temp_range_k[static_cast<std::size_t>(i)] < 0 || c.temp_range_k[static_cast<std::size_t>(i)] > 40) { fail(error, "monthly.temp_range_k out of [0, 40] K"); return std::nullopt; }
        if (c.rh_mean[static_cast<std::size_t>(i)] < 0 || c.rh_mean[static_cast<std::size_t>(i)] > 1) { fail(error, "monthly.rh_mean out of [0, 1]"); return std::nullopt; }
    }
    const auto te = root->find("temperature");
    if (te == root->end() || !te->is_object() || !read_number(*te, "lapse_k_per_m", c.lapse_k_per_m, error, "temperature.") ||
        !read_number(*te, "peak_solar_hour", c.peak_solar_hour, error, "temperature.") ||
        !read_number(*te, "cloud_range_reduction", c.cloud_range_reduction, error, "temperature."))
        return std::nullopt;
    if (c.lapse_k_per_m < 0 || c.lapse_k_per_m > 0.02) { fail(error, "temperature.lapse_k_per_m out of [0, 0.02]"); return std::nullopt; }
    if (c.peak_solar_hour < 0 || c.peak_solar_hour >= 24) { fail(error, "temperature.peak_solar_hour out of [0, 24)"); return std::nullopt; }
    if (c.cloud_range_reduction < 0 || c.cloud_range_reduction > 1) { fail(error, "temperature.cloud_range_reduction out of [0, 1]"); return std::nullopt; }
    double seed = 1;
    if (!read_number(*root, "seed", seed, error, "") || !read_number(*root, "weather_ramp_s", c.weather_ramp_s, error, "")) return std::nullopt;
    if (seed < 0 || c.weather_ramp_s < 0) { fail(error, "seed and weather_ramp_s must be >= 0"); return std::nullopt; }
    c.seed = static_cast<std::uint64_t>(seed);
    const auto rs = root->find("resync");
    if (rs == root->end() || !rs->is_object() || !read_number(*rs, "temperature_k", c.resync_temperature_k, error, "resync.") ||
        !read_number(*rs, "clock_s", c.resync_clock_s, error, "resync.") ||
        !read_number(*rs, "clock_real_s", c.resync_clock_real_s, error, "resync."))
        return std::nullopt;
    if (!(c.resync_temperature_k > 0) || !(c.resync_clock_s > 0) || !(c.resync_clock_real_s > 0)) { fail(error, "resync values must be > 0"); return std::nullopt; }
    if (!detail::get_string(*root, "ground_surface", c.ground_surface) || c.ground_surface.empty()) { fail(error, "ground_surface must be a surface name"); return std::nullopt; }
    const auto ph = root->find("physics");
    if (ph == root->end() || !ph->is_object()) { fail(error, "physics block missing"); return std::nullopt; }
    c.physics_blocks_json = ph->dump();
    const std::filesystem::path dir = std::filesystem::path(path).parent_path();
    std::string sc_path, tp_path;
    if (!detail::get_string(*root, "surface_conditions", sc_path) || !detail::get_string(*root, "tyre_profile", tp_path)) {
        fail(error, "surface_conditions and tyre_profile paths required");
        return std::nullopt;
    }
    const auto wx = root->find("weather");
    if (wx == root->end() || !wx->is_object() || wx->empty()) { fail(error, "weather presets missing"); return std::nullopt; }
    try {
        c.surface_conditions = ps::io::load_surface_conditions_json(dir / sc_path);
        c.tyre_profile = ps::io::load_tyre_profile_json(dir / tp_path);
        for (auto it = wx->begin(); it != wx->end(); ++it) c.weather[it.key()] = ps::io::parse_weather_preset_json(it.value(), path + " weather." + it.key());
    } catch (const std::exception& e) {
        fail(error, e.what());
        return std::nullopt;
    }
    return c;
}

double climate_daily_mean_c(const ClimateConfig& c, double day_of_year) { return interp_monthly(c.temp_mean_c, day_of_year); }
double climate_daily_range_k(const ClimateConfig& c, double day_of_year) { return interp_monthly(c.temp_range_k, day_of_year); }

EnvClockPoint env_clock_point(const CivilDate& date_ut, double ut_h, double longitude_deg) {
    CivilDate d = date_ut;
    double h = ut_h + longitude_deg / 15.0;
    while (h < 0) { h += 24; d = add_days(d, -1); }
    while (h >= 24) { h -= 24; d = add_days(d, 1); }
    EnvClockPoint p;
    p.day_index = nonleap_day_of_year(d) - 1;
    p.solar_tod_s = h * 3600.0;
    p.env_s = p.day_index * 86400.0 + p.solar_tod_s;
    return p;
}

double climate_temperature_c(const ClimateConfig& c, const CivilDate& date_ut, double ut_h, double longitude_deg,
                             double altitude_m, double cloud_cover) {
    const EnvClockPoint p = env_clock_point(date_ut, ut_h, longitude_deg);
    const double doy = p.day_index + 1 + p.solar_tod_s / 86400.0;
    const double cloud = std::fmin(1.0, std::fmax(0.0, std::isfinite(cloud_cover) ? cloud_cover : 0.0));
    const double amp = 0.5 * climate_daily_range_k(c, doy) * (1.0 - c.cloud_range_reduction * cloud);
    const double solar_h = p.solar_tod_s / 3600.0;
    return climate_daily_mean_c(c, doy) + amp * std::cos(2.0 * kPi * (solar_h - c.peak_solar_hour) / 24.0) -
           c.lapse_k_per_m * (altitude_m - c.station_elevation_m);
}

std::optional<ps::environment::ClimateDesc> make_physics_climate(const ClimateConfig& c, double latitude_deg,
                                                                 double longitude_deg, std::string* error) {
    json j;
    j["format"] = "physics_sim.climate/1";
    j["location"] = {{"latitude_deg", latitude_deg}, {"longitude_deg", longitude_deg}, {"elevation_m", c.station_elevation_m}};
    j["sun"] = {{"mode", "astronomical"}};
    j["calendar"] = {{"epoch", {{"year", 0}, {"season", kMonthNames[0]}, {"day", 0}, {"time_of_day_s", 0}}}, {"cycle_s", 86400}};
    json seasons = json::array();
    int start = 1;
    for (std::size_t m = 0; m < 12; ++m) {
        const int end = start + kMonthDays[m] - 1;
        seasons.push_back({{"name", kMonthNames[m]}, {"days", kMonthDays[m]}, {"doy_span", {start, end}},
                           {"day_night", {{"astronomical", true}}}, {"temp_mean_c", c.temp_mean_c[m]},
                           {"temp_range_k", c.temp_range_k[m]}, {"rh_mean", c.rh_mean[m]},
                           {"pressure_msl_hpa", c.pressure_msl_hpa}, {"ground_temp_c", c.ground_temp_c[m]}});
        start = end + 1;
    }
    j["seasons"] = seasons;
    const auto blocks = detail::parse_json(c.physics_blocks_json);
    if (!blocks || !blocks->is_object()) { fail(error, "physics block is not an object"); return std::nullopt; }
    for (auto it = blocks->begin(); it != blocks->end(); ++it) j[it.key()] = it.value();
    try {
        return ps::io::parse_climate_json(j, "rg.climate (built)");
    } catch (const std::exception& e) {
        fail(error, e.what());
        return std::nullopt;
    }
}

std::optional<ps::environment::WeatherPresetDesc> physics_weather_preset(const ClimateConfig& c, const std::string& weather_id,
                                                                         double station_temp_c) {
    const auto it = c.weather.find(weather_id);
    if (it == c.weather.end() || !std::isfinite(station_temp_c)) return std::nullopt;
    ps::environment::WeatherPresetDesc p = it->second;
    p.temp_is_override = true;
    p.temp_override_c = station_temp_c;
    p.temp_offset_k = 0;
    return p;
}

std::optional<SessionWeather> make_session_weather(const ClimateConfig& c, double latitude_deg, double longitude_deg,
                                                   double grid_convergence_deg, const CivilDate& date_ut, double ut_h,
                                                   double time_scale, const std::string& weather_id, std::string* error) {
    auto climate = make_physics_climate(c, latitude_deg, longitude_deg, error);
    if (!climate) return std::nullopt;
    const auto& wx = c.weather.find(weather_id) != c.weather.end() ? weather_id : c.weather.begin()->first;
    const double cloud = c.weather.at(wx).cloud_cover;
    const auto preset = physics_weather_preset(c, wx, climate_station_temperature_c(c, date_ut, ut_h, longitude_deg, cloud));
    const EnvClockPoint p = env_clock_point(date_ut, ut_h, longitude_deg);
    SessionWeather w;
    w.climate = std::move(*climate);
    w.ground_surface = c.ground_surface;
    auto& cfg = w.config;
    cfg.fixed_day_of_year = p.day_index;
    int month = 0, day = p.day_index;
    while (day >= kMonthDays[static_cast<std::size_t>(month)]) { day -= kMonthDays[static_cast<std::size_t>(month)]; ++month; }
    cfg.start = ps::environment::EnvironmentStart{0, kMonthNames[month], day, p.solar_tod_s};
    cfg.time_scale = std::isfinite(time_scale) && time_scale >= 0 ? time_scale : 1.0;
    cfg.north_yaw_deg = std::isfinite(grid_convergence_deg) ? grid_convergence_deg : 0.0;
    cfg.weather = ps::environment::EnvironmentWeather::Preset;
    cfg.weather_preset = *preset;
    cfg.seed = c.seed;
    cfg.surface_conditions = c.surface_conditions;
    cfg.default_tyre_profile = c.tyre_profile;
    try {
        ps::environment::Environment probe(w.climate, cfg); // validate now, not on the sim thread
    } catch (const std::exception& e) {
        fail(error, e.what());
        return std::nullopt;
    }
    return w;
}

} // namespace rg
