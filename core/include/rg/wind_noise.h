// rg/wind_noise.h - aerodynamic (wind) noise from the car's AIRSPEED, engine-neutral. Config: data/controls/wind_noise.json.
//
// Owner 2026-10-19: there was no wind noise at all (the "wind" the owner heard was the tyre layer). This layer is driven by
// the speed relative to the AIR (the sim's aero airspeed: ground velocity minus the environment wind and gusts), not by the
// ground speed, so a headwind is louder and a tailwind quieter at the same ground speed.
//
// Law (all numbers are config; [AS] = assumed, no published value for this exact car):
//   amplitude  A(U) = rms_at_reference * (U / reference_speed)^(level_db_per_decade / 20)       (U clamped to max_speed)
//   with level_db_per_decade = 40 [AS] (was 50; owner 2026-10-21: the wind scaled too aggressively). The layer is what the
//   driver hears INSIDE the cabin. Outside, aerodynamic sound from turbulent flow over a body grows with a high power of
//   speed - Lighthill (1952, "On sound generated aerodynamically I"): monopole U^4, dipole U^6, quadrupole U^8; Hucho,
//   Aerodynamics of Road Vehicles, 4th ed., aeroacoustics chapter: "about 50 to 60 log10(v)". The glass and seals attenuate
//   more as the spectrum moves up with the airspeed, so the interior level grows less steeply: 40 dB per decade = 12 dB
//   per doubling, close to the total interior level of a passenger car (roughly 65 dBA at 100 km/h, 76 dBA at 200 km/h).
//   The tyre layer grows at 35 dB per decade, so it still dominates in town.
//   centre frequency f_c(U) = centre_hz_at_reference * (U / reference_speed)^centre_exponent, exponent 1 [AS]: a
//   Strouhal-type scaling f = St * U / L (shedding from mirrors/A-pillars/antenna) puts the spectrum's weight at a
//   frequency proportional to U.
//   fade: A is multiplied by smoothstep(0, full_above_mps, U) so the layer is exactly 0 at rest; with the power law the
//   level at walking pace is already > 50 dB below the level at the reference speed, so the fade is not audible as a threshold.
//   gust modulation: a slow (gust_rate_hz) smoothed noise multiplies the level by (1 + gust_depth * g), g ~ unit std.
#pragma once

#include "rg/biquad.h"

#include <cstdint>
#include <string>
#include <vector>

namespace rg {

struct WindNoiseConfig {
    double reference_speed_mps = 100.0 / 3.6; // [AS] speed the levels below are given at
    double rms_at_reference = 0.0172;         // [AS] output RMS (full scale = 1) at the reference speed, before the master gain
    double level_db_per_decade = 40.0;        // [AS] interior slope, see the header comment
    double max_speed_mps = 500.0 / 3.6;       // the law stops growing here - above the hyper's 446 km/h top speed
    double full_above_mps = 1.0;              // smoothstep(0, this, U): exactly 0 at rest
    double centre_hz_at_reference = 700.0;    // [AS]
    double centre_exponent = 1.0;             // [AS] f_c ~ U (Strouhal)
    double min_centre_hz = 120.0, max_centre_hz = 6000.0;
    double q = 0.7;                           // [AS] broad band
    double gust_depth = 0.15, gust_rate_hz = 0.8; // [AS]
    double smoothing_s = 0.05;                // airspeed smoothing in the voice
};

// Pure laws (U in m/s, absolute value used).
double wind_noise_amplitude(const WindNoiseConfig& c, double airspeed_mps); // output RMS before the master gain
double wind_noise_centre_hz(const WindNoiseConfig& c, double airspeed_mps);
// data/controls/wind_noise.json ("wind" block). Returns false and fills `error` on a malformed or out-of-range file.
bool parse_wind_noise_config(const std::string& json_text, WindNoiseConfig& out, std::string& error);

class WindNoise {
public:
    static constexpr int rate = 48000;
    explicit WindNoise(const WindNoiseConfig& cfg = {}, std::uint64_t seed = 1);
    void configure(const WindNoiseConfig& cfg) { cfg_ = cfg; }
    const WindNoiseConfig& config() const { return cfg_; }
    // Target airspeed in m/s (non-finite -> 0). Smoothed inside render().
    void set_airspeed(double mps);
    // Mono, `frames` clamped to [0, 16384].
    std::vector<float> render(int frames, double gain);
    // Adds `frames` mono samples to every channel of an interleaved buffer.
    void mix_into(float* interleaved, int channels, int frames, double gain);
    double smoothed_airspeed() const { return u_; }
    double current_amplitude() const { return amp_; }
    double current_centre_hz() const { return centre_; }

private:
    float next(double gain);
    double noise();
    WindNoiseConfig cfg_;
    std::uint64_t rng_;
    double target_ = 0.0, u_ = 0.0, amp_ = 0.0, centre_ = 700.0, gust_ = 0.0, norm_ = 1.0;
    int block_ = 0;
    BandPass band_;
};

} // namespace rg
