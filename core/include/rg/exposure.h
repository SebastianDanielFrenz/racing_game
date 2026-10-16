// rg/exposure.h - camera exposure from the sky's illuminance (lights/exposure, 2026-10-15).
//
// Photometric frame: scene values are luminances in cd/m^2 ("nits"); a white
// Lambertian surface under an illuminance of E lux has E/pi nits. The display
// value (linear, before tonemapping) is scene * exposure, with
//     exposure = key * pi / A,   A = adapted illuminance, clamped to [min, max]
// i.e. a white surface under the adapted illuminance shows at `key`. The eye/
// camera adapts down to min_adapted_lux; below that (deep dusk, night) the
// exposure stays at its night value, so night looks dark and lamps dominate.
// At noon (A ~ 1e5 lux) the exposure is ~2000x lower than at night, so a lamp
// of constant physical intensity is negligible against daylight.
//
// Engine-neutral: no renderer assumption. How an adapter applies it depends on
// the renderer's range (see docs/visual_world_plan.md "Exposure"): the
// Compatibility renderer keeps the tuned sky at exposure 1 and multiplies only
// the lamps by `exposure` (exact for a linear tonemap); an HDR renderer would
// scale the sky by 1/exposure and set the camera exposure instead.
#pragma once

#include <limits>
#include <string>

namespace rg {

struct ExposureConfig {
    double key = 1.5;                // display value of a white surface under the adapted illuminance
    double min_adapted_lux = 50;     // adaptation stops here: the night exposure is key*pi/min_adapted_lux
    double max_adapted_lux = 150000; // and here (bright snow/noon)
    double adapt_tau_s = 1.5;        // time constant of the adaptation, in log illuminance
};

// Strict check of a config (finite, key > 0, 0 < min <= max, tau >= 0); empty when valid.
std::string validate_exposure_config(const ExposureConfig& config);

double clamp_adapted_lux(const ExposureConfig& config, double lux);
// Display value per nit for an adapted illuminance (clamped first).
double exposure_for_lux(const ExposureConfig& config, double adapted_lux);
// The exposure at full dark adaptation (the most a lamp can show).
inline double night_exposure(const ExposureConfig& config) { return exposure_for_lux(config, config.min_adapted_lux); }

// Geometric blend of the sky-driven exposure with a measured one (for a
// renderer that meters the frame, e.g. Godot Mobile/Forward+ auto exposure):
// weight 0 = sky only, 1 = measured only. A non-finite or non-positive measured
// value returns the sky value.
double combine_exposure(double sky_exposure, double measured_exposure, double measured_weight);

// Smooth adaptation in log illuminance (first order, tau = adapt_tau_s).
class ExposureController {
public:
    explicit ExposureController(ExposureConfig config = {}) : config_(config) {}
    // Jumps straight to `lux` (spawn, time-of-day menu jump).
    void reset(double lux);
    // Moves towards `target_lux`; the first call after construction resets.
    void update(double target_lux, double dt_s);
    bool valid() const { return valid_; }
    double adapted_lux() const { return adapted_lux_; }
    double exposure() const { return exposure_for_lux(config_, adapted_lux_); }
    const ExposureConfig& config() const { return config_; }

private:
    ExposureConfig config_;
    double adapted_lux_ = std::numeric_limits<double>::quiet_NaN();
    bool valid_ = false;
};

} // namespace rg
