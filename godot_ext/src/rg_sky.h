// godot_ext/src/rg_sky.h - RgSky: the thin Godot binding of rg/time_weather.h
// (time of day, sun/moon light, visual weather, the weather/time widget's
// view-model). All numbers are computed in rg_core; this file only converts
// (session ENU -> Godot axes, Rgb -> Color, structs -> Dictionaries).
#pragma once

#include "rg/time_weather.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>

#include <memory>

namespace rg_godot {

class RgSky : public godot::RefCounted {
    GDCLASS(RgSky, godot::RefCounted)

public:
    // data/world/sky.json + the world config (location from its session origin
    // unless sky.json names one). false + get_error() on a load failure.
    bool initialize(const godot::String& sky_json_path, const godot::String& world_config_path);
    [[nodiscard]] godot::String get_error() const { return error_; }
    void advance(double real_dt_s);
    void set_local_time_h(double hours);
    void set_rate(double rate);
    // {clock_text, date_text, local_time_h, rate, sun_direction (Godot, towards
    // the sun), sun_elevation_deg, sun_energy, sun_color, moon_direction,
    // moon_elevation_deg, moon_illuminated, moon_energy, moon_color,
    // ambient_energy, sky_top_color, sky_horizon_color, icon, weather_id,
    // weather_label, cloud_cover, illuminance_lux, exposure (time-adapted
    // display value per cd/m2, rg/exposure.h), adapted_lux, night_exposure}; {}
    // before initialize. Not const: each call advances the exposure adaptation.
    [[nodiscard]] godot::Dictionary get_state();
    // [{id, group, label, enabled, selected, reason}]
    [[nodiscard]] godot::Array get_menu() const;
    bool choose(const godot::String& id);
    static godot::String format_temperature_c(double kelvin);
    // V8: climate.json beside sky.json; empty when it loaded (the physics weather and the derived temperature then
    // follow this sky), else why not (weather stays visual only, precipitation presets greyed out).
    [[nodiscard]] godot::String get_climate_error() const { return climate_error_; }
    // C++ only (RgSimulation reads the clock/weather to feed the physics environment).
    [[nodiscard]] const rg::TimeWeather* time_weather() const { return tw_.get(); }

protected:
    static void _bind_methods();

private:
    std::unique_ptr<rg::TimeWeather> tw_;
    godot::String error_;
    godot::String climate_error_;
};

} // namespace rg_godot
