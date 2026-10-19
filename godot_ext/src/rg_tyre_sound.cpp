#include "rg_tyre_sound.h"

#include <godot_cpp/core/class_db.hpp>

#include <cstring>

namespace rg_godot {

void RgTyreSound::setup(int wheels, int base_seed, const godot::PackedByteArray& squeal_pcm, const godot::PackedByteArray& roll_pcm,
                        int roll_channels, const godot::Dictionary& roll_config) {
    rg::TyreRollConfig cfg;
    cfg.reference_speed_mps = roll_config.get("reference_speed_kmh", cfg.reference_speed_mps * 3.6).operator double() / 3.6;
    cfg.use_recording = godot::String(roll_config.get("roll_source", "recording")) != godot::String("procedural");
    cfg.min_rate = roll_config.get("min_rate", cfg.min_rate);
    cfg.max_rate = roll_config.get("max_rate", cfg.max_rate);
    cfg.silent_below_mps = roll_config.get("silent_below_kmh", cfg.silent_below_mps * 3.6).operator double() / 3.6;
    cfg.full_above_mps = roll_config.get("full_above_kmh", cfg.full_above_mps * 3.6).operator double() / 3.6;
    cfg.gain_at_reference = roll_config.get("gain_at_reference", cfg.gain_at_reference);
    cfg.gain_exponent = roll_config.get("gain_exponent", cfg.gain_exponent);
    cfg.max_gain = roll_config.get("max_gain", cfg.max_gain);
    cfg.loop_crossfade_ms = roll_config.get("loop_crossfade_ms", cfg.loop_crossfade_ms);
    std::shared_ptr<const std::vector<float>> loop;
    if (roll_pcm.size() >= 2 && roll_channels >= 1) {
        std::vector<std::int16_t> raw(static_cast<std::size_t>(roll_pcm.size()) / 2);
        std::memcpy(raw.data(), roll_pcm.ptr(), raw.size() * 2);
        loop = rg::make_roll_loop(raw.data(), raw.size(), roll_channels, cfg.loop_crossfade_ms);
    }
    auto squeal = std::make_shared<std::vector<std::int16_t>>(static_cast<std::size_t>(squeal_pcm.size()) / 2);
    if (!squeal->empty()) std::memcpy(squeal->data(), squeal_pcm.ptr(), squeal->size() * 2); // little-endian host
    bank_ = std::make_unique<rg::TyreSoundBank>(wheels, static_cast<std::uint64_t>(base_seed), squeal, loop, cfg);
}

void RgTyreSound::update_wheel(int wheel, double speed, double tread_speed, double slip_ratio, double slip_angle,
                               double load_n, double fx, double fy, double wetness, bool sealed) {
    if (!bank_) return;
    rg::TyreSoundInput in;
    in.speed = speed;
    in.tread_speed = tread_speed;
    in.slip_ratio = slip_ratio;
    in.slip_angle = slip_angle;
    in.load_n = load_n;
    in.fx = fx;
    in.fy = fy;
    in.wetness = wetness;
    in.sealed = sealed;
    bank_->update(wheel, in);
}

godot::PackedFloat32Array RgTyreSound::render(int frames, double gain) {
    godot::PackedFloat32Array out;
    if (!bank_ || frames <= 0) return out;
    const std::vector<float> pcm = bank_->render(frames, static_cast<float>(gain));
    out.resize(static_cast<int64_t>(pcm.size()));
    if (!pcm.empty()) std::memcpy(out.ptrw(), pcm.data(), pcm.size() * sizeof(float));
    return out;
}

godot::PackedVector2Array RgTyreSound::channel_to_stereo(const godot::PackedFloat32Array& samples, int channels, int channel) {
    godot::PackedVector2Array out;
    if (channels <= 0 || channel < 0 || channel >= channels) return out;
    const int64_t frames = samples.size() / channels;
    out.resize(frames);
    const float* src = samples.ptr();
    godot::Vector2* dst = out.ptrw();
    for (int64_t i = 0; i < frames; ++i) {
        const float s = src[i * channels + channel];
        dst[i] = godot::Vector2(s, s);
    }
    return out;
}

godot::PackedFloat32Array RgTyreSound::scaled(const godot::PackedFloat32Array& samples, double gain) {
    godot::PackedFloat32Array out;
    out.resize(samples.size());
    const float g = static_cast<float>(gain);
    const float* src = samples.ptr();
    float* dst = out.ptrw();
    for (int64_t i = 0; i < samples.size(); ++i) dst[i] = src[i] * g;
    return out;
}

void RgTyreSound::_bind_methods() {
    godot::ClassDB::bind_method(godot::D_METHOD("setup", "wheels", "base_seed", "squeal_pcm", "roll_pcm", "roll_channels", "roll_config"), &RgTyreSound::setup);
    godot::ClassDB::bind_method(godot::D_METHOD("update_wheel", "wheel", "speed", "tread_speed", "slip_ratio", "slip_angle", "load_n", "fx", "fy", "wetness", "sealed"),
                                &RgTyreSound::update_wheel, DEFVAL(true));
    godot::ClassDB::bind_method(godot::D_METHOD("render", "frames", "gain"), &RgTyreSound::render);
    godot::ClassDB::bind_static_method("RgTyreSound", godot::D_METHOD("channel_to_stereo", "samples", "channels", "channel"), &RgTyreSound::channel_to_stereo);
    godot::ClassDB::bind_static_method("RgTyreSound", godot::D_METHOD("scaled", "samples", "gain"), &RgTyreSound::scaled);
}

} // namespace rg_godot
