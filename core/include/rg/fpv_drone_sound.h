// rg/fpv_drone_sound.h - procedural quadcopter rotor buzz for the FPV drone (engine-neutral, like tyre_sound.h).
// Four rotors, slightly detuned against each other, each a blade-pass hum with a few harmonics, plus a little air
// noise. At idle the hum is low and steady; pitch and level rise with the drone's activity (acceleration,
// deceleration, turning - FpvDroneState::activity) and fall back when it settles. Mono, 48 kHz.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rg {

struct FpvAudioConfig {
    double idle_hz = 150.0;            // blade-pass fundamental when hovering
    double max_hz = 420.0;             // ... at full activity
    double idle_level = 0.22;          // 0..1
    double max_level = 0.8;
    double activity_rise_per_s = 6.0;  // smoothing of the activity going up
    double activity_fall_per_s = 2.5;  // ... and coming down (slower: the rotors spin down)
    double yaw_activity_weight = 0.5;  // documented for the data file; the weight is applied by FpvDrone::activity
};

// Loads the "audio" block of data/controls/fpv_drone.json; nullopt + *err on any problem.
std::optional<FpvAudioConfig> load_fpv_audio_config(const std::string& path, std::string* err);

class FpvRotorVoice {
public:
    explicit FpvRotorVoice(const FpvAudioConfig& config = {}, std::uint64_t seed = 1);
    // The drone's current activity 0..1 (FpvDroneState::activity); call once per game frame.
    void set_activity(double activity) { target_activity_ = activity < 0.0 ? 0.0 : (activity > 1.0 ? 1.0 : activity); }
    // `frames` mono samples at 48 kHz scaled by `gain`; advances the smoothing by frames / 48000 s.
    std::vector<float> render(int frames, float gain);
    [[nodiscard]] double smoothed_activity() const { return activity_; }
    [[nodiscard]] double frequency_hz() const;
    [[nodiscard]] double level() const;

private:
    float noise();

    FpvAudioConfig config_;
    std::uint64_t rng_;
    double target_activity_ = 0.0, activity_ = 0.0;
    double phase_[4] = {0.0, 0.0, 0.0, 0.0};
    double air_ = 0.0;
};

} // namespace rg
