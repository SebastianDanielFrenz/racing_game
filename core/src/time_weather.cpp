#include "rg/time_weather.h"

#include "rg/climate.h"

#include "json_util.h"
#include "rg/world_config.h"

#include "g2m/core/geo/utm.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>

namespace rg {
namespace {
using detail::json;
constexpr double kDeg = 3.14159265358979323846 / 180.0;

bool fail(std::string* error, const std::string& message) {
    if (error) *error = message;
    return false;
}
bool read_rgb(const json& o, const char* key, Rgb& out) {
    const auto it = o.find(key);
    if (it == o.end() || !it->is_array() || it->size() != 3) return false;
    for (std::size_t i = 0; i < 3; ++i) {
        if (!(*it)[i].is_number()) return false;
        out[i] = (*it)[i].get<double>();
    }
    return true;
}
bool parse_date(const std::string& text, CivilDate& out) {
    int y = 0, m = 0, d = 0;
    if (std::sscanf(text.c_str(), "%d-%d-%d", &y, &m, &d) != 3 || m < 1 || m > 12 || d < 1 || d > days_in_month(y, m)) return false;
    out = {y, m, d};
    return true;
}
std::optional<Precipitation> parse_precipitation(const std::string& s) {
    if (s == "none") return Precipitation::None;
    if (s == "rain") return Precipitation::Rain;
    if (s == "snow") return Precipitation::Snow;
    if (s == "hail") return Precipitation::Hail;
    return std::nullopt;
}
double lerp(double a, double b, double t) { return a + (b - a) * t; }
Rgb lerp(const Rgb& a, const Rgb& b, double t) { return {lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t)}; }
double smooth01(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3 - 2 * x);
}
std::string two_digits(int v) {
    char buf[8];
    std::snprintf(buf, sizeof buf, "%02d", v);
    return buf;
}
} // namespace

const char* weather_icon_name(WeatherIcon icon) {
    switch (icon) {
    case WeatherIcon::Sun: return "sun";
    case WeatherIcon::SunCloud: return "sun_cloud";
    case WeatherIcon::Cloud: return "cloud";
    case WeatherIcon::Moon: return "moon";
    case WeatherIcon::MoonCloud: return "moon_cloud";
    case WeatherIcon::Rain: return "rain";
    case WeatherIcon::Snow: return "snow";
    case WeatherIcon::Hail: return "hail";
    case WeatherIcon::Thunder: return "thunder";
    }
    return "sun";
}

WeatherIcon weather_icon(const WeatherState& w, double sun_elevation_deg, const IconThresholds& t) {
    if (w.thunder) return WeatherIcon::Thunder;
    if (w.precipitation == Precipitation::Hail) return WeatherIcon::Hail;
    if (w.precipitation == Precipitation::Snow) return WeatherIcon::Snow;
    if (w.precipitation == Precipitation::Rain) return WeatherIcon::Rain;
    if (w.cloud_cover >= t.overcast_min) return WeatherIcon::Cloud;
    const bool night = sun_elevation_deg < t.night_sun_elevation_deg;
    if (w.cloud_cover >= t.partly_cloudy_min) return night ? WeatherIcon::MoonCloud : WeatherIcon::SunCloud;
    return night ? WeatherIcon::Moon : WeatherIcon::Sun;
}

SkyLighting sky_lighting(const LightingConfig& c, const SkyPosition& sun, const MoonPosition& moon, double cover) {
    SkyLighting out;
    cover = std::clamp(std::isfinite(cover) ? cover : 0.0, 0.0, 1.0);
    if (c.keys.empty()) return out;
    const double el = sun.elevation_deg;
    LightingKey key = c.keys.front();
    if (el >= c.keys.back().sun_elevation_deg) key = c.keys.back();
    else
        for (std::size_t i = 1; i < c.keys.size(); ++i) {
            const auto& a = c.keys[i - 1];
            const auto& b = c.keys[i];
            if (el < b.sun_elevation_deg) {
                const double t = el <= a.sun_elevation_deg ? 0 : (el - a.sun_elevation_deg) / (b.sun_elevation_deg - a.sun_elevation_deg);
                key.sun_energy = lerp(a.sun_energy, b.sun_energy, t);
                key.sun_rgb = lerp(a.sun_rgb, b.sun_rgb, t);
                key.ambient_energy = lerp(a.ambient_energy, b.ambient_energy, t);
                key.sky_top_rgb = lerp(a.sky_top_rgb, b.sky_top_rgb, t);
                key.sky_horizon_rgb = lerp(a.sky_horizon_rgb, b.sky_horizon_rgb, t);
                break;
            }
        }
    const double cloud_light = 1 - c.cloud_sun_dim * std::pow(cover, c.cloud_exponent);
    out.sun_energy = el > 0 ? key.sun_energy * cloud_light : 0;
    out.sun_rgb = key.sun_rgb;
    out.ambient_energy = key.ambient_energy * (1 - c.cloud_ambient_dim * cover);
    // Overcast: the sky colour goes towards grey, scaled to the key's brightness.
    const double top_luma = (key.sky_top_rgb[0] + key.sky_top_rgb[1] + key.sky_top_rgb[2]) / 3;
    const double hor_luma = (key.sky_horizon_rgb[0] + key.sky_horizon_rgb[1] + key.sky_horizon_rgb[2]) / 3;
    const double grey = (c.overcast_rgb[0] + c.overcast_rgb[1] + c.overcast_rgb[2]) / 3;
    auto overcast = [&](double luma) {
        const double s = grey > 0 ? luma / grey : 0;
        return Rgb{c.overcast_rgb[0] * s, c.overcast_rgb[1] * s, c.overcast_rgb[2] * s};
    };
    out.sky_top_rgb = lerp(key.sky_top_rgb, overcast(top_luma), cover);
    out.sky_horizon_rgb = lerp(key.sky_horizon_rgb, overcast(hor_luma), cover);
    // Moon light only where the sun light is gone, faded in with the moon's height.
    const double moon_up = c.moon_fade_elevation_deg > 0 ? smooth01(moon.elevation_deg / c.moon_fade_elevation_deg) : (moon.elevation_deg > 0 ? 1.0 : 0.0);
    const double sun_gone = out.sun_energy > 0 ? 0.0 : 1.0;
    out.moon_energy = c.moon_energy_full * moon.illuminated_fraction * moon_up * sun_gone * cloud_light;
    out.moon_rgb = c.moon_rgb;
    return out;
}

std::optional<SkyConfig> load_sky_config(const std::string& path, std::string* error) {
    const auto text = detail::read_text_file(path);
    if (!text) { fail(error, path + ": cannot read"); return std::nullopt; }
    const auto parsed = detail::parse_json(*text);
    if (!parsed || !parsed->is_object()) { fail(error, path + ": not a JSON object"); return std::nullopt; }
    const json& j = *parsed;
    std::string format;
    if (!detail::get_string(j, "format", format) || format != "rg.sky/1") { fail(error, path + ": format must be \"rg.sky/1\""); return std::nullopt; }
    SkyConfig c;
    auto bad = [&](const std::string& what) { fail(error, path + ": " + what); return std::nullopt; };
    if (const auto loc = j.find("location"); loc != j.end()) {
        if (!loc->is_object()) return bad("location must be an object");
        double v = 0;
        if (detail::get_number(*loc, "latitude_deg", v)) c.latitude_deg = v;
        if (detail::get_number(*loc, "longitude_deg", v)) c.longitude_deg = v;
        if (c.latitude_deg.has_value() != c.longitude_deg.has_value()) return bad("location needs both latitude_deg and longitude_deg");
    }
    const auto tz = j.find("time_zone");
    if (tz == j.end() || !tz->is_object() || !detail::get_number(*tz, "standard_offset_h", c.time_zone.standard_offset_h) ||
        !detail::get_bool(*tz, "eu_summer_time", c.time_zone.eu_summer_time))
        return bad("time_zone needs standard_offset_h and eu_summer_time");
    const auto start = j.find("start");
    std::string date;
    if (start == j.end() || !start->is_object() || !detail::get_string(*start, "date", date) ||
        !detail::get_number(*start, "local_time_h", c.start_local_time_h) || c.start_local_time_h < 0 || c.start_local_time_h >= 24)
        return bad("start needs date (\"today\" or YYYY-MM-DD) and local_time_h in [0, 24)");
    if (date != "today") {
        CivilDate d;
        if (!parse_date(date, d)) return bad("start.date is not a valid YYYY-MM-DD date");
        c.start_date = d;
    }
    const auto rate = j.find("rate");
    if (rate == j.end() || !rate->is_object() || !detail::get_number(*rate, "default", c.default_rate) || c.default_rate < 0)
        return bad("rate needs default >= 0");
    if (const auto opts = rate->find("options"); opts != rate->end() && opts->is_array()) {
        for (const auto& v : *opts) {
            if (!v.is_number() || v.get<double>() < 0) return bad("rate.options must be numbers >= 0");
            c.rate_options.push_back(v.get<double>());
        }
    }
    if (const auto tp = j.find("time_presets"); tp != j.end() && tp->is_array()) {
        for (const auto& p : *tp) {
            TimePreset preset;
            if (!p.is_object() || !detail::get_string(p, "id", preset.id) || !detail::get_string(p, "label", preset.label) ||
                !detail::get_number(p, "local_time_h", preset.local_time_h) || preset.local_time_h < 0 || preset.local_time_h >= 24)
                return bad("each time_presets entry needs id, label, local_time_h in [0, 24)");
            c.time_presets.push_back(preset);
        }
    }
    const auto wp = j.find("weather_presets");
    if (wp == j.end() || !wp->is_array() || wp->empty()) return bad("weather_presets must be a non-empty array");
    for (const auto& p : *wp) {
        WeatherPreset preset;
        std::string precip;
        if (!p.is_object() || !detail::get_string(p, "id", preset.id) || !detail::get_string(p, "label", preset.label) ||
            !detail::get_number(p, "cloud_cover", preset.state.cloud_cover) || preset.state.cloud_cover < 0 || preset.state.cloud_cover > 1 ||
            !detail::get_string(p, "precipitation", precip) || !detail::get_bool(p, "thunder", preset.state.thunder))
            return bad("each weather_presets entry needs id, label, cloud_cover in [0, 1], precipitation, thunder");
        const auto kind = parse_precipitation(precip);
        if (!kind) return bad("precipitation must be none, rain, snow or hail");
        preset.state.precipitation = *kind;
        c.weather_presets.push_back(preset);
    }
    if (!detail::get_string(j, "default_weather", c.default_weather)) return bad("default_weather missing");
    if (std::none_of(c.weather_presets.begin(), c.weather_presets.end(), [&](const auto& p) { return p.id == c.default_weather; }))
        return bad("default_weather names no weather preset");
    detail::get_string(j, "unwired_reason", c.unwired_reason);
    const auto icon = j.find("icon");
    if (icon == j.end() || !icon->is_object() || !detail::get_number(*icon, "partly_cloudy_min", c.icon.partly_cloudy_min) ||
        !detail::get_number(*icon, "overcast_min", c.icon.overcast_min) ||
        !detail::get_number(*icon, "night_sun_elevation_deg", c.icon.night_sun_elevation_deg) || c.icon.partly_cloudy_min > c.icon.overcast_min)
        return bad("icon needs partly_cloudy_min <= overcast_min and night_sun_elevation_deg");
    const auto light = j.find("lighting");
    if (light == j.end() || !light->is_object()) return bad("lighting missing");
    auto& L = c.lighting;
    if (!detail::get_number(*light, "moon_energy_full", L.moon_energy_full) || !read_rgb(*light, "moon_rgb", L.moon_rgb) ||
        !detail::get_number(*light, "moon_fade_elevation_deg", L.moon_fade_elevation_deg) ||
        !detail::get_number(*light, "cloud_sun_dim", L.cloud_sun_dim) || !detail::get_number(*light, "cloud_exponent", L.cloud_exponent) ||
        !detail::get_number(*light, "cloud_ambient_dim", L.cloud_ambient_dim) || !read_rgb(*light, "overcast_rgb", L.overcast_rgb))
        return bad("lighting needs moon_energy_full, moon_rgb, moon_fade_elevation_deg, cloud_sun_dim, cloud_exponent, cloud_ambient_dim, overcast_rgb");
    const auto keys = light->find("keys");
    if (keys == light->end() || !keys->is_array() || keys->size() < 2) return bad("lighting.keys needs at least two entries");
    for (const auto& k : *keys) {
        LightingKey key;
        if (!k.is_object() || !detail::get_number(k, "sun_elevation_deg", key.sun_elevation_deg) || !detail::get_number(k, "sun_energy", key.sun_energy) ||
            !read_rgb(k, "sun_rgb", key.sun_rgb) || !detail::get_number(k, "ambient_energy", key.ambient_energy) ||
            !read_rgb(k, "sky_top_rgb", key.sky_top_rgb) || !read_rgb(k, "sky_horizon_rgb", key.sky_horizon_rgb))
            return bad("each lighting key needs sun_elevation_deg, sun_energy, sun_rgb, ambient_energy, sky_top_rgb, sky_horizon_rgb");
        if (!L.keys.empty() && key.sun_elevation_deg <= L.keys.back().sun_elevation_deg) return bad("lighting.keys must ascend in sun_elevation_deg");
        L.keys.push_back(key);
    }
    return c;
}

GeoLocation location_from_world_config(const WorldConfig& world) {
    const g2m::geo::UtmZone zone{world.session_origin_utm.zone};
    const auto r = g2m::geo::Utm{}.reverse(zone, world.session_origin_utm.e0, world.session_origin_utm.n0);
    return {r.lat, r.lon, r.gamma};
}

TimeController::TimeController(CivilDate date, double hours, double rate, TimeZoneRule zone) : date_(date), rate_(rate), zone_(zone) {
    set_local_time_h(hours);
}

void TimeController::advance(double dt) {
    if (!std::isfinite(dt) || dt <= 0) return;
    seconds_ += dt * rate_;
    const double day = 86400.0;
    if (seconds_ >= day) {
        const int days = static_cast<int>(std::floor(seconds_ / day));
        date_ = add_days(date_, days);
        seconds_ -= days * day;
    }
}

void TimeController::set_local_time_h(double hours) {
    if (!std::isfinite(hours)) return;
    hours = std::fmod(hours, 24.0);
    if (hours < 0) hours += 24.0;
    seconds_ = hours * 3600.0;
    ++revision_;
}

void TimeController::set_rate(double rate) {
    if (std::isfinite(rate) && rate >= 0 && rate != rate_) {
        rate_ = rate;
        ++revision_;
    }
}

void TimeController::ut_instant(CivilDate& date_ut, double& ut_h) const {
    date_ut = date_;
    ut_h = local_time_h() - utc_offset_h();
    if (ut_h < 0) { ut_h += 24; date_ut = add_days(date_ut, -1); }
    if (ut_h >= 24) { ut_h -= 24; date_ut = add_days(date_ut, 1); }
}

double TimeController::utc_offset_h() const {
    // The rule is defined on UT: convert with the standard offset first, then
    // with the offset found there (exact except within the switch hour).
    auto offset_at = [&](double off) {
        double ut = local_time_h() - off;
        CivilDate d = date_;
        if (ut < 0) { ut += 24; d = add_days(d, -1); }
        if (ut >= 24) { ut -= 24; d = add_days(d, 1); }
        return utc_offset_hours(zone_, d, ut);
    };
    return offset_at(offset_at(zone_.standard_offset_h));
}

double TimeController::julian_day_ut() const {
    return julian_day(date_, local_time_h() - utc_offset_h());
}

std::array<double, 3> grid_direction(const SkyPosition& body, double gamma_deg) {
    // Grid north is `gamma` clockwise from true north: grid azimuth = az - gamma.
    const double az = (body.azimuth_deg - gamma_deg) * kDeg;
    const double horiz = std::sqrt(std::max(0.0, 1 - body.up * body.up));
    return {horiz * std::sin(az), horiz * std::cos(az), body.up};
}

TimeWeather::TimeWeather(SkyConfig config, GeoLocation location) : config_(std::move(config)), location_(location) {
    if (config_.latitude_deg) {
        location_.latitude_deg = *config_.latitude_deg;
        location_.longitude_deg = *config_.longitude_deg;
    }
    CivilDate start;
    if (config_.start_date) start = *config_.start_date;
    else {
        const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm tm{};
#ifdef _WIN32
        gmtime_s(&tm, &now);
#else
        gmtime_r(&now, &tm);
#endif
        start = {tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday};
    }
    clock_ = TimeController(start, config_.start_local_time_h, config_.default_rate, config_.time_zone);
    for (std::size_t i = 0; i < config_.weather_presets.size(); ++i)
        if (config_.weather_presets[i].id == config_.default_weather) weather_ = i;
}

bool TimeWeather::weather_wired(const WeatherPreset& p) const {
    if (climate_ && climate_->weather.count(p.id)) return true;
    return p.state.precipitation == Precipitation::None && !p.state.thunder;
}

TimeWeatherState TimeWeather::state() const {
    TimeWeatherState s;
    s.local_date = clock_.local_date();
    s.local_time_h = clock_.local_time_h();
    s.utc_offset_h = clock_.utc_offset_h();
    const int minutes = static_cast<int>(std::floor(s.local_time_h * 60.0)) % 1440;
    s.clock_text = two_digits(minutes / 60) + ":" + two_digits(minutes % 60);
    s.date_text = std::to_string(s.local_date.year) + "-" + two_digits(s.local_date.month) + "-" + two_digits(s.local_date.day);
    const double jd = clock_.julian_day_ut();
    s.sun = sun_position(jd, location_.latitude_deg, location_.longitude_deg);
    s.moon = moon_position(jd, location_.latitude_deg, location_.longitude_deg);
    const auto& preset = config_.weather_presets[weather_];
    s.weather = preset.state;
    s.weather_id = preset.id;
    s.weather_label = preset.label;
    s.lighting = sky_lighting(config_.lighting, s.sun, s.moon, s.weather.cloud_cover);
    s.icon = weather_icon(s.weather, s.sun.elevation_deg, config_.icon);
    s.rate = clock_.rate();
    if (climate_) {
        CivilDate d;
        double ut = 0;
        clock_.ut_instant(d, ut);
        s.air_temp_c = climate_station_temperature_c(*climate_, d, ut, location_.longitude_deg, s.weather.cloud_cover);
    }
    return s;
}

std::vector<SkyMenuItem> TimeWeather::menu() const {
    std::vector<SkyMenuItem> items;
    for (const auto& p : config_.time_presets) items.push_back({"time:" + p.id, "time", p.label, true, false, ""});
    for (std::size_t i = 0; i < config_.rate_options.size(); ++i) {
        const double r = config_.rate_options[i];
        char label[32];
        if (r == 0) std::snprintf(label, sizeof label, "Stopped");
        else std::snprintf(label, sizeof label, "x%g", r);
        items.push_back({"rate:" + std::to_string(i), "rate", label, true, r == clock_.rate(), ""});
    }
    for (std::size_t i = 0; i < config_.weather_presets.size(); ++i) {
        const auto& p = config_.weather_presets[i];
        const bool wired = weather_wired(p);
        items.push_back({"weather:" + p.id, "weather", p.label, wired, i == weather_, wired ? "" : config_.unwired_reason});
    }
    return items;
}

bool TimeWeather::choose(const std::string& id) {
    for (const auto& item : menu()) {
        if (item.id != id) continue;
        if (!item.enabled) return false;
        const std::string key = id.substr(item.group.size() + 1);
        if (item.group == "time") {
            for (const auto& p : config_.time_presets)
                if (p.id == key) clock_.set_local_time_h(p.local_time_h);
        } else if (item.group == "rate") {
            clock_.set_rate(config_.rate_options[std::stoul(key)]);
        } else {
            for (std::size_t i = 0; i < config_.weather_presets.size(); ++i)
                if (config_.weather_presets[i].id == key) weather_ = i;
        }
        return true;
    }
    return false;
}

std::string format_temperature_c(double kelvin) {
    if (!std::isfinite(kelvin)) return "-- °C";
    const long c = std::lround(kelvin - 273.15);
    return std::to_string(c == 0 ? 0 : c) + " °C";
}

} // namespace rg
