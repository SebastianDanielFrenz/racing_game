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
    bool sealed = true;       // asphalt-like sealed surface: the recorded roll layer is used (if loaded), else the procedural hiss
};

// Recorded rolling layer: ONE car-level loop (owner's PROVISIONAL AI-generated recording, data/controls/tyre_sound.json). All numbers are config.
struct TyreRollConfig {
    double reference_speed_mps = 50.0 / 3.6; // [AS] speed the recording was made at (unknown); rate = speed / reference
    double min_rate = 0.5, max_rate = 2.0;   // playback-rate clamp
    double silent_below_mps = 0.3;           // gain is exactly 0 up to here ...
    double full_above_mps = 2.0;             // ... and fades in (smoothstep) to here
    bool use_recording = true;               // roll_source: "recording" | "procedural" (data/controls/tyre_sound.json)
    double gain_at_reference = 0.25;         // linear gain at the reference speed (recording is about -20 dBFS RMS)
    double gain_exponent = 1.0;              // gain = gain_at_reference * (speed / reference)^exponent
    double max_gain = 1.0;
    double loop_crossfade_ms = 75.0;
};

// Mono, seamlessly looping roll sample at 48 kHz built from a decoded recording (interleaved 16-bit, 1 or 2 channels).
// Returns null when the data is empty/too short.
std::shared_ptr<const std::vector<float>> make_roll_loop(const std::int16_t* pcm, std::size_t samples, int channels,
                                                         double crossfade_ms);

// One wheel's voice. `squeal` is the 16-bit mono skid recording (may be empty: no squeal).
class TyreVoice {
public:
    TyreVoice(std::uint64_t seed, std::shared_ptr<const std::vector<std::int16_t>> squeal);
    // The bank mutes the procedural roll of a wheel while the car-level recording covers it. All procedural code stays.
    void set_roll_muted(bool muted) { roll_muted_ = muted; }
    void update(const TyreSoundInput& in);
    float sample(double dt);

private:
    float noise();
    float squeal_sample();

    std::shared_ptr<const std::vector<std::int16_t>> squeal_;
    bool roll_muted_ = false;
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
    // roll_loop null or roll_cfg.use_recording false: the procedural per-wheel roll everywhere.
    TyreSoundBank(int wheels, std::uint64_t base_seed, std::shared_ptr<const std::vector<std::int16_t>> squeal,
                  std::shared_ptr<const std::vector<float>> roll_loop = nullptr, TyreRollConfig roll_cfg = {});
    int wheels() const { return static_cast<int>(voices_.size()); }
    void update(int wheel, const TyreSoundInput& in);
    // `frames` output frames at 48 kHz, interleaved wheels, scaled by `gain`.
    std::vector<float> render(int frames, float gain);

private:
    std::vector<TyreVoice> voices_;
    std::vector<float> values_;
    bool even_ = true;
    // car-level recorded roll
    std::shared_ptr<const std::vector<float>> roll_loop_;
    TyreRollConfig roll_cfg_;
    std::vector<char> sealed_, contact_;
    double speed_ = 0.0, roll_pos_ = 0.0, roll_gain_ = 0.0;
    float roll_value_ = 0.0f;
    float roll_sample(double dt);
};

} // namespace rg
