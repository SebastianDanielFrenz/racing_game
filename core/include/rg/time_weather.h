// rg/time_weather.h - time of day and weather state for the visual world
// (sun/moon light, sky colours, the top-right weather/time widget and its
// menu). Engine-neutral: the Godot adapter (RgSky) only converts. Config:
// data/world/sky.json (rg.sky/1). Weather here is VISUAL state only (cloud
// cover -> sky/light); precipitation and thunder are listed but not wired
// (no precipitation visuals, no wet-grip environment installed in Session).
#pragma once

#include "rg/sky.h"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace rg {

struct WorldConfig;

using Rgb = std::array<double, 3>;

enum class Precipitation { None, Rain, Snow, Hail };
enum class WeatherIcon { Sun, SunCloud, Cloud, Moon, MoonCloud, Rain, Snow, Hail, Thunder };
const char* weather_icon_name(WeatherIcon icon);

struct WeatherState {
    double cloud_cover = 0; // 0..1
    Precipitation precipitation = Precipitation::None;
    bool thunder = false;
};

struct IconThresholds {
    double partly_cloudy_min = 0.2;
    double overcast_min = 0.7;
    double night_sun_elevation_deg = -3.0;
};
WeatherIcon weather_icon(const WeatherState& weather, double sun_elevation_deg, const IconThresholds& thresholds);

struct LightingKey {
    double sun_elevation_deg = 0;
    double sun_energy = 0;
    Rgb sun_rgb{1, 1, 1};
    double ambient_energy = 0;
    Rgb sky_top_rgb{0, 0, 0};
    Rgb sky_horizon_rgb{0, 0, 0};
};
struct LightingConfig {
    std::vector<LightingKey> keys; // ascending sun elevation, >= 2
    double moon_energy_full = 0;
    Rgb moon_rgb{0.7, 0.8, 1.0};
    double moon_fade_elevation_deg = 5; // moon light ramps in over 0..this elevation
    double cloud_sun_dim = 0.75;        // sun energy x (1 - dim * cover^exponent)
    double cloud_exponent = 3.4;
    double cloud_ambient_dim = 0.3;     // ambient energy x (1 - dim * cover)
    Rgb overcast_rgb{0.55, 0.57, 0.6};  // sky colour blended in with cloud cover
};
struct SkyLighting {
    double sun_energy = 0;
    Rgb sun_rgb{1, 1, 1};
    double moon_energy = 0;
    Rgb moon_rgb{0.7, 0.8, 1.0};
    double ambient_energy = 0;
    Rgb sky_top_rgb{0, 0, 0};
    Rgb sky_horizon_rgb{0, 0, 0};
};
SkyLighting sky_lighting(const LightingConfig& config, const SkyPosition& sun, const MoonPosition& moon, double cloud_cover);

struct TimePreset {
    std::string id, label;
    double local_time_h = 12;
};
struct WeatherPreset {
    std::string id, label;
    WeatherState state;
};

struct SkyConfig {
    std::optional<double> latitude_deg, longitude_deg; // unset: from the world config's session origin
    TimeZoneRule time_zone;
    std::optional<CivilDate> start_date;               // unset ("today"): the system clock's date
    double start_local_time_h = 12;
    double default_rate = 1;
    std::vector<double> rate_options;                  // game seconds per real second
    std::vector<TimePreset> time_presets;
    std::vector<WeatherPreset> weather_presets;
    std::string default_weather;
    std::string unwired_reason;                        // shown for presets the game cannot apply yet
    IconThresholds icon;
    LightingConfig lighting;
};
std::optional<SkyConfig> load_sky_config(const std::string& path, std::string* error);

struct GeoLocation {
    double latitude_deg = 0, longitude_deg = 0;
    double grid_convergence_deg = 0; // bearing of grid north from true north (session x/y are UTM grid axes)
};
GeoLocation location_from_world_config(const WorldConfig& world);

// Local civil clock, progressive at `rate` game seconds per real second.
class TimeController {
public:
    TimeController() = default;
    TimeController(CivilDate local_date, double local_time_h, double rate, TimeZoneRule zone);
    void advance(double real_dt_s);
    void set_local_time_h(double hours); // same day, wrapped into [0, 24)
    void set_date(CivilDate date) { date_ = date; }
    void set_rate(double rate);          // < 0 or non-finite ignored
    double rate() const { return rate_; }
    const CivilDate& local_date() const { return date_; }
    double local_time_h() const { return seconds_ / 3600.0; }
    double utc_offset_h() const;
    double julian_day_ut() const;

private:
    CivilDate date_;
    double seconds_ = 12 * 3600.0;
    double rate_ = 1;
    TimeZoneRule zone_;
};

struct MenuItem {
    std::string id;    // "time:<preset>", "rate:<index>", "weather:<preset>"
    std::string group; // "time", "rate", "weather"
    std::string label;
    bool enabled = true;
    bool selected = false;
    std::string reason; // why disabled
};

struct TimeWeatherState {
    CivilDate local_date;
    double local_time_h = 0;
    double utc_offset_h = 0;
    std::string clock_text; // "14:05"
    std::string date_text;  // "2026-06-21"
    SkyPosition sun;        // true-north frame
    MoonPosition moon;
    SkyLighting lighting;
    WeatherState weather;
    std::string weather_id, weather_label;
    WeatherIcon icon = WeatherIcon::Sun;
    double rate = 1;
};

// Session-frame (UTM grid: x east, y north, z up) unit direction towards a
// body, rotated by the grid convergence.
std::array<double, 3> grid_direction(const SkyPosition& body, double grid_convergence_deg);

class TimeWeather {
public:
    TimeWeather(SkyConfig config, GeoLocation location);
    void advance(double real_dt_s) { clock_.advance(real_dt_s); }
    TimeController& clock() { return clock_; }
    const SkyConfig& config() const { return config_; }
    const GeoLocation& location() const { return location_; }
    TimeWeatherState state() const;
    std::vector<MenuItem> menu() const;
    // Applies a menu item; false for an unknown or disabled id.
    bool choose(const std::string& id);
    bool weather_wired(const WeatherPreset& preset) const;

private:
    SkyConfig config_;
    GeoLocation location_;
    TimeController clock_;
    std::size_t weather_ = 0;
};

// "15 °C" (rounded) from kelvin; "-- °C" when not finite.
std::string format_temperature_c(double kelvin);

} // namespace rg
