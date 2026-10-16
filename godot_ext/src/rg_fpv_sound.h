// rg_fpv_sound.h - RgFpvSound: binding of rg::FpvRotorVoice (procedural quadcopter rotor buzz). Conversion only.
#pragma once

#include "rg/fpv_drone_sound.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/string.hpp>

namespace rg_godot {

class RgFpvSound : public godot::RefCounted {
    GDCLASS(RgFpvSound, godot::RefCounted)

public:
    // Loads the "audio" block of data/controls/fpv_drone.json; "" or the error.
    godot::String load_config(const godot::String& path);
    // 0..1, FpvDroneState.activity (RgSimulation.get_fpv_state()["activity"]).
    void set_activity(double activity);
    // `frames` mono samples at 48 kHz scaled by gain.
    godot::PackedFloat32Array render(int frames, double gain);
    double get_frequency_hz() const;

protected:
    static void _bind_methods();

private:
    rg::FpvAudioConfig config_{};
    rg::FpvRotorVoice voice_{};
};

} // namespace rg_godot
