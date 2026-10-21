// rg_wind_noise.h - RgWindNoise: binding of rg::WindNoise (aerodynamic noise from the AIRSPEED). Conversion only; the
// synthesis and the law are in rg_core (rg/wind_noise.h, data/controls/wind_noise.json).
#pragma once

#include "rg/wind_noise.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/string.hpp>

namespace rg_godot {

class RgWindNoise : public godot::RefCounted {
    GDCLASS(RgWindNoise, godot::RefCounted)

public:
    // config_json: the TEXT of data/controls/wind_noise.json. On a parse error the defaults are used; returns the message ("" = ok).
    godot::String setup(const godot::String& config_json);
    // Speed relative to the air in m/s (the sim's aero airspeed, wind and gusts included).
    void set_airspeed(double mps) { voice_.set_airspeed(mps); }
    // Adds `frames` mono wind samples to every channel of the interleaved buffer; returns the sum buffer.
    godot::PackedFloat32Array mix_into(const godot::PackedFloat32Array& interleaved, int channels, double gain);
    // gain, centre_hz, airspeed (smoothed) of the voice.
    godot::Dictionary get_state() const;

protected:
    static void _bind_methods();

private:
    rg::WindNoise voice_;
};

} // namespace rg_godot
