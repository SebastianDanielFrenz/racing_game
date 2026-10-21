// rg/tyre_sound.h - procedural tyre noise (roll hiss + skid squeal), engine-neutral. Port of the former
// game/scripts/tyre_sound_model.gd plus its per-sample mixing loop in vehicle_audio.gd: the GDScript version cost
// 2-3 ms of main-thread time per frame (24 k samples/s x 4 wheels), the same work here is a few tens of microseconds.
#pragma once

#include "rg/biquad.h"

#include <cstdint>
#include <memory>
#include <string>
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
    bool sealed = true;       // asphalt-like sealed surface: the car-level roll layer is used (if loaded), else the procedural hiss
    double surface_db = 0.0;  // road-surface level offset (TyreRollConfig::surfaces), dB re dense asphalt
};

// Level law of the rolling noise (tyre/road noise): gain = gain_at_reference * (v / reference_speed)^(level_db_per_decade / 20)
// for v up to max_speed, times a standstill fade.
//
// Why a power law and why 35 dB per decade: CPX/CPB measurements of passenger-car tyre/road noise follow
// L = L_ref + a * log10(v / v_ref) with a = 30 ... 40 dB for 50 ... 120 km/h on dense surfaces (Sandberg & Ejsmont, "Tyre/Road
// Noise Reference Book", Informex 2002, ch. 3 and the speed-dependence tables; the Harmonoise/CNOSSOS-EU rolling-noise
// coefficients are of the same size, 30 for light vehicles). 35 is the middle of the band [AS]. In amplitude: exponent a / 20.
// The pre-2026-10-19 layer used exponent 1 (20 dB/decade, 6 dB per doubling) clamped at 200 km/h and a playback rate clamped
// to [0.5, 2] x the reference speed - flat in pitch above 100 km/h and in level above 200 km/h.
struct TyreRollLaw {
    double reference_speed_mps = 100.0 / 3.6; // [AS]
    double gain_at_reference = 0.22;          // [AS] linear gain of the car-level layer at the reference speed
    double level_db_per_decade = 35.0;        // [AS] Sandberg & Ejsmont: 30-40
    double max_speed_mps = 500.0 / 3.6;       // the law stops growing here, above the hypercar's 446 km/h top speed
    double silent_below_mps = 0.0;            // gain is exactly 0 up to here ...
    double full_above_mps = 1.0;              // ... and fades in (smoothstep) to here; the power law is > 60 dB down by then
};

// One road surface: which wheels use the car-level layer (sealed) and its level offset.
struct TyreSurfaceNoise {
    std::string name;
    bool sealed = false;
    double db = 0.0; // [AS] dB re dense asphalt; Sandberg & Ejsmont ch. 4: tined concrete +2..4, pavers +4..7, setts/cobbles +6..10
};

// Car-level rolling layer: ONE loop (owner's PROVISIONAL AI-generated recording, data/controls/tyre_sound.json) plus a band-pass
// hiss and a tread-pattern tone. All numbers are config.
struct TyreRollConfig {
    TyreRollLaw level;
    // Playback rate of the recording = (v / rate_reference_speed)^rate_exponent: the spectrum of rolling noise shifts up
    // slowly with speed (tread-impact and air-pumping peak about 0.8-1.2 kHz at 100 km/h, rising roughly with v^0.3) [AS].
    double rate_reference_speed_mps = 50.0 / 3.6; // [AS] speed the recording was made at (unknown)
    double rate_exponent = 0.35;                  // [AS]
    double min_rate = 0.25, max_rate = 4.0;       // wide, only a safety bound: 4x is reached at 50 * 4^(1/0.35) km/h
    // Band-pass hiss next to the recording: centre = hiss_centre_hz * (v / level.reference_speed)^hiss_centre_exponent.
    double hiss_db = -6.0;                        // [AS] relative to the recording's RMS
    double hiss_centre_hz = 1000.0;               // [AS] at the reference speed
    double hiss_centre_exponent = 0.3;            // [AS]
    double hiss_q = 1.2;                          // [AS]
    // Tread-block passing tone: f = tread_speed / tread_pitch_m = blocks per revolution * wheel revolutions per second.
    double tread_pitch_m = 0.032;                 // [AS] about 62 pitches round a 2.0 m circumference
    double tone_db = -14.0;                       // [AS] relative to the recording's RMS
    double tone_q = 10.0;                         // [AS] pitch sequencing smears the tone into a narrow noise band
    double tone_sine_share = 0.3;                 // [AS] power share of the pure sinusoid in the tone
    double procedural_gain_at_reference = 0.0145; // the per-wheel procedural hiss (unsealed surfaces, no recording): amplitude at the reference speed
    bool use_recording = true;                    // roll_source: "recording" | "procedural" (data/controls/tyre_sound.json)
    double loop_crossfade_ms = 75.0;
    std::vector<TyreSurfaceNoise> surfaces;       // names missing here are unsealed, 0 dB
    const TyreSurfaceNoise* find_surface(const std::string& name) const;
};

// Pure laws (v in m/s, absolute value used).
double tyre_roll_gain(const TyreRollLaw& law, double v);           // includes the standstill fade
double tyre_roll_rate(const TyreRollConfig& c, double v);          // playback rate of the recording
double tyre_hiss_centre_hz(const TyreRollConfig& c, double v);
double tyre_tread_tone_hz(const TyreRollConfig& c, double tread_speed_mps);
// data/controls/tyre_sound.json ("roll" block). Returns false and fills `error` on a malformed or out-of-range file.
bool parse_tyre_roll_config(const std::string& json_text, TyreRollConfig& out, std::string& error);

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
    // Level law of the procedural per-wheel roll (the bank sets it from its config).
    void set_roll_law(const TyreRollLaw& law) { law_ = law; }
    void update(const TyreSoundInput& in);
    float sample(double dt);

private:
    float noise();
    float squeal_sample();

    std::shared_ptr<const std::vector<std::int16_t>> squeal_;
    bool roll_muted_ = false;
    TyreRollLaw law_{100.0 / 3.6, 0.0145, 35.0, 500.0 / 3.6, 0.0, 1.0};
    std::uint64_t rng_;
    std::size_t squeal_pos_ = 0;
    double squeal_level_ = 0.0, roll_ = 0.0, slide_ = 0.0, tone_gain_ = 0.0, frequency_ = 1000.0, roll_cutoff_ = 850.0;
    double low_ = 0.0, phase_ = 0.0, grain_ = 0.0, wetness_ = 0.0, surface_gain_ = 1.0;
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
    // car-level roll: recording + hiss + tread tone
    std::shared_ptr<const std::vector<float>> roll_loop_;
    TyreRollConfig roll_cfg_;
    std::vector<char> sealed_, contact_;
    std::vector<double> surface_gain_, wheel_speed_, wheel_tread_;
    double layer_amp_ = 0.0;                             // mean surface gain of the sealed wheels in contact
    double speed_s_ = 0.0, tread_s_ = 0.0;               // smoothed
    double roll_pos_ = 0.0, roll_gain_ = 0.0, loop_rms_ = 0.1, tone_phase_ = 0.0;
    float roll_value_ = 0.0f;
    int roll_block_ = 0;
    std::uint64_t roll_rng_ = 0x2545F4914F6CDD1Dull;
    BandPass hiss_band_, tone_band_;
    double hiss_norm_ = 1.0, tone_norm_ = 1.0, tone_hz_ = 1000.0;
    float roll_sample(double dt);
    double roll_noise();
public:
    // Diagnostics of the car-level layer (what the last roll_sample() used): gain after smoothing, recording playback rate,
    // hiss centre, tread tone frequency, smoothed speed.
    struct RollState { double gain = 0, rate = 0, hiss_hz = 0, tone_hz = 0, speed = 0; };
    RollState roll_state() const { return roll_state_; }
    const TyreRollConfig& config() const { return roll_cfg_; }
private:
    RollState roll_state_;
};

} // namespace rg
