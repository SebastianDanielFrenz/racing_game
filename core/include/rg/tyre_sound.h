// rg/tyre_sound.h - procedural tyre noise (roll hiss + skid squeal), engine-neutral. Port of the former
// game/scripts/tyre_sound_model.gd plus its per-sample mixing loop in vehicle_audio.gd: the GDScript version cost
// 2-3 ms of main-thread time per frame (24 k samples/s x 4 wheels), the same work here is a few tens of microseconds.
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

namespace rg {

struct TyreSoundInput {
    double speed = 0.0;       // chassis speed, m/s
    double tread_speed = 0.0; // wheel omega * radius, m/s
    double slip_ratio = 0.0;
    double slip_angle = 0.0;  // rad
    double load_n = 0.0;
    double fx = 0.0;
    double fy = 0.0;
    double wetness = 0.0;     // 0..1
};

// One wheel's voice. `squeal` is the 16-bit mono skid recording (may be empty: no squeal).
class TyreVoice {
public:
    TyreVoice(std::uint64_t seed, std::shared_ptr<const std::vector<std::int16_t>> squeal);
    void update(const TyreSoundInput& in);
    float sample(double dt);

private:
    float noise();
    float squeal_sample();

    std::shared_ptr<const std::vector<std::int16_t>> squeal_;
    std::uint64_t rng_;
    std::size_t squeal_pos_ = 0;
    double squeal_level_ = 0.0, roll_ = 0.0, slide_ = 0.0, tone_gain_ = 0.0, frequency_ = 1000.0, roll_cutoff_ = 850.0;
    double low_ = 0.0, phase_ = 0.0, grain_ = 0.0, wetness_ = 0.0;
    bool contact_ = false;
};

// All wheels of a car, rendered together as interleaved frames (frame-major, one value per wheel). The voices run at
// 24 kHz (every second output frame advances them, the value is held in between - the demo's pitch) with the one-pole
// smoothing the GDScript used.
class TyreSoundBank {
public:
    TyreSoundBank(int wheels, std::uint64_t base_seed, std::shared_ptr<const std::vector<std::int16_t>> squeal);
    int wheels() const { return static_cast<int>(voices_.size()); }
    void update(int wheel, const TyreSoundInput& in);
    // `frames` output frames at 48 kHz, interleaved wheels, scaled by `gain`.
    std::vector<float> render(int frames, float gain);

private:
    std::vector<TyreVoice> voices_;
    std::vector<float> values_;
    bool even_ = true;
};

} // namespace rg
