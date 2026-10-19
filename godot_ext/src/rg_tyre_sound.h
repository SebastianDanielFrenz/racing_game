// rg_tyre_sound.h - RgTyreSound: binding of rg::TyreSoundBank plus the two sample-buffer helpers the Godot fallback
// audio path needs (interleaved -> one stereo channel, gain). Conversion only; the synthesis is in rg_core.
#pragma once

#include "rg/tyre_sound.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/string.hpp>

#include <memory>

namespace rg_godot {

class RgTyreSound : public godot::RefCounted {
    GDCLASS(RgTyreSound, godot::RefCounted)

public:
    // squeal_pcm: the 16-bit mono recording's data (AudioStreamWAV.data), may be empty.
    // roll_pcm/roll_channels: the owner's rolling recording (16-bit interleaved), config: data/controls/tyre_sound.json
    // "roll" block. Empty roll_pcm = procedural hiss everywhere.
    void setup(int wheels, int base_seed, const godot::PackedByteArray& squeal_pcm, const godot::PackedByteArray& roll_pcm,
               int roll_channels, const godot::Dictionary& roll_config);
    void update_wheel(int wheel, double speed, double tread_speed, double slip_ratio, double slip_angle, double load_n,
                      double fx, double fy, double wetness, bool sealed);
    // `frames` frames, interleaved wheels, scaled by gain.
    godot::PackedFloat32Array render(int frames, double gain);
    // Channel `channel` of interleaved `samples` as a stereo (L = R) buffer.
    static godot::PackedVector2Array channel_to_stereo(const godot::PackedFloat32Array& samples, int channels, int channel);
    static godot::PackedFloat32Array scaled(const godot::PackedFloat32Array& samples, double gain);

protected:
    static void _bind_methods();

private:
    std::unique_ptr<rg::TyreSoundBank> bank_;
};

} // namespace rg_godot
