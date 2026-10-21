#include "rg_wind_noise.h"

#include <godot_cpp/core/class_db.hpp>

#include <string>

namespace rg_godot {

godot::String RgWindNoise::setup(const godot::String& config_json) {
    rg::WindNoiseConfig cfg;
    std::string error;
    if (!rg::parse_wind_noise_config(config_json.utf8().get_data(), cfg, error)) cfg = rg::WindNoiseConfig{};
    voice_.configure(cfg);
    return godot::String::utf8(error.c_str());
}

godot::PackedFloat32Array RgWindNoise::mix_into(const godot::PackedFloat32Array& interleaved, int channels, double gain) {
    godot::PackedFloat32Array out = interleaved;
    if (channels <= 0 || out.is_empty()) return out;
    voice_.mix_into(out.ptrw(), channels, static_cast<int>(out.size() / channels), gain);
    return out;
}

godot::Dictionary RgWindNoise::get_state() const {
    godot::Dictionary d;
    d["amplitude"] = voice_.current_amplitude();
    d["centre_hz"] = voice_.current_centre_hz();
    d["airspeed_mps"] = voice_.smoothed_airspeed();
    return d;
}

void RgWindNoise::_bind_methods() {
    godot::ClassDB::bind_method(godot::D_METHOD("setup", "config_json"), &RgWindNoise::setup);
    godot::ClassDB::bind_method(godot::D_METHOD("set_airspeed", "mps"), &RgWindNoise::set_airspeed);
    godot::ClassDB::bind_method(godot::D_METHOD("mix_into", "interleaved", "channels", "gain"), &RgWindNoise::mix_into);
    godot::ClassDB::bind_method(godot::D_METHOD("get_state"), &RgWindNoise::get_state);
}

} // namespace rg_godot
