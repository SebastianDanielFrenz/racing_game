#include "rg_sky.h"

#include "rg/climate.h"

#include "rg/world_config.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace rg_godot {
namespace {
godot::Color to_color(const rg::Rgb& c) { return godot::Color(float(c[0]), float(c[1]), float(c[2])); }
// Session grid ENU (x east, y north, z up) -> Godot (x = -north, y = up, z = -east),
// the same ISO->Godot change as frame_convert_core.h.
godot::Vector3 to_godot(const std::array<double, 3>& enu) { return godot::Vector3(float(-enu[1]), float(enu[2]), float(-enu[0])); }
std::string utf8(const godot::String& s) { return std::string(s.utf8().get_data()); }
} // namespace

bool RgSky::initialize(const godot::String& sky_json_path, const godot::String& world_config_path) {
    std::string err;
    auto config = rg::load_sky_config(utf8(sky_json_path), &err);
    if (!config) { error_ = godot::String::utf8(err.c_str()); return false; }
    rg::GeoLocation location;
    if (!config->latitude_deg) {
        auto world = rg::load_world_config(utf8(world_config_path), &err);
        if (!world) { error_ = godot::String::utf8(err.c_str()); return false; }
        location = rg::location_from_world_config(*world);
    }
    tw_ = std::make_unique<rg::TimeWeather>(std::move(*config), location);
    error_ = "";
    const std::string sky_path = utf8(sky_json_path);
    const auto slash = sky_path.find_last_of("/\\");
    std::string climate_err;
    auto climate = rg::load_climate_config(sky_path.substr(0, slash == std::string::npos ? 0 : slash + 1) + "climate.json", &climate_err);
    if (climate) {
        tw_->set_climate(std::make_shared<const rg::ClimateConfig>(std::move(*climate)));
        climate_error_ = "";
    } else {
        climate_error_ = godot::String::utf8(climate_err.c_str());
    }
    return true;
}

void RgSky::advance(double dt) { if (tw_) tw_->advance(dt); }
void RgSky::set_local_time_h(double h) { if (tw_) tw_->clock().set_local_time_h(h); }
void RgSky::set_rate(double r) { if (tw_) tw_->clock().set_rate(r); }

godot::Dictionary RgSky::get_state() const {
    godot::Dictionary d;
    if (!tw_) return d;
    const auto s = tw_->state();
    const double gamma = tw_->location().grid_convergence_deg;
    d["clock_text"] = godot::String(s.clock_text.c_str());
    d["date_text"] = godot::String(s.date_text.c_str());
    d["local_time_h"] = s.local_time_h;
    d["rate"] = s.rate;
    d["sun_direction"] = to_godot(rg::grid_direction(s.sun, gamma));
    d["sun_elevation_deg"] = s.sun.elevation_deg;
    d["sun_energy"] = s.lighting.sun_energy;
    d["sun_color"] = to_color(s.lighting.sun_rgb);
    d["moon_direction"] = to_godot(rg::grid_direction(s.moon, gamma));
    d["moon_elevation_deg"] = s.moon.elevation_deg;
    d["moon_illuminated"] = s.moon.illuminated_fraction;
    d["moon_energy"] = s.lighting.moon_energy;
    d["moon_color"] = to_color(s.lighting.moon_rgb);
    d["ambient_energy"] = s.lighting.ambient_energy;
    d["sky_top_color"] = to_color(s.lighting.sky_top_rgb);
    d["sky_horizon_color"] = to_color(s.lighting.sky_horizon_rgb);
    d["icon"] = godot::String(rg::weather_icon_name(s.icon));
    d["weather_id"] = godot::String(s.weather_id.c_str());
    d["weather_label"] = godot::String(s.weather_label.c_str());
    d["cloud_cover"] = s.weather.cloud_cover;
    d["air_temp_c"] = s.air_temp_c; // NaN without a climate
    d["physics_weather"] = tw_->climate() != nullptr;
    return d;
}

godot::Array RgSky::get_menu() const {
    godot::Array out;
    if (!tw_) return out;
    for (const auto& item : tw_->menu()) {
        godot::Dictionary d;
        d["id"] = godot::String(item.id.c_str());
        d["group"] = godot::String(item.group.c_str());
        d["label"] = godot::String::utf8(item.label.c_str());
        d["enabled"] = item.enabled;
        d["selected"] = item.selected;
        d["reason"] = godot::String::utf8(item.reason.c_str());
        out.push_back(d);
    }
    return out;
}

bool RgSky::choose(const godot::String& id) { return tw_ && tw_->choose(utf8(id)); }

godot::String RgSky::format_temperature_c(double kelvin) {
    return godot::String::utf8(rg::format_temperature_c(kelvin).c_str());
}

void RgSky::_bind_methods() {
    using godot::ClassDB;
    using godot::D_METHOD;
    ClassDB::bind_method(D_METHOD("initialize", "sky_json_path", "world_config_path"), &RgSky::initialize);
    ClassDB::bind_method(D_METHOD("get_error"), &RgSky::get_error);
    ClassDB::bind_method(D_METHOD("advance", "real_dt_s"), &RgSky::advance);
    ClassDB::bind_method(D_METHOD("set_local_time_h", "hours"), &RgSky::set_local_time_h);
    ClassDB::bind_method(D_METHOD("set_rate", "rate"), &RgSky::set_rate);
    ClassDB::bind_method(D_METHOD("get_state"), &RgSky::get_state);
    ClassDB::bind_method(D_METHOD("get_menu"), &RgSky::get_menu);
    ClassDB::bind_method(D_METHOD("choose", "id"), &RgSky::choose);
    ClassDB::bind_method(D_METHOD("get_climate_error"), &RgSky::get_climate_error);
    ClassDB::bind_static_method("RgSky", D_METHOD("format_temperature_c", "kelvin"), &RgSky::format_temperature_c);
}

} // namespace rg_godot
