#include "rg/exposure.h"

#include <algorithm>
#include <cmath>

namespace rg {
namespace {
constexpr double kPi = 3.14159265358979323846;
}

std::string validate_exposure_config(const ExposureConfig& c) {
    if (!std::isfinite(c.key) || c.key <= 0) return "exposure.key must be > 0";
    if (!std::isfinite(c.min_adapted_lux) || !std::isfinite(c.max_adapted_lux) || c.min_adapted_lux <= 0 ||
        c.max_adapted_lux < c.min_adapted_lux)
        return "exposure needs 0 < min_adapted_lux <= max_adapted_lux";
    if (!std::isfinite(c.adapt_tau_s) || c.adapt_tau_s < 0) return "exposure.adapt_tau_s must be >= 0";
    return {};
}

double clamp_adapted_lux(const ExposureConfig& c, double lux) {
    if (!std::isfinite(lux)) lux = c.min_adapted_lux;
    return std::clamp(lux, c.min_adapted_lux, c.max_adapted_lux);
}

double exposure_for_lux(const ExposureConfig& c, double adapted_lux) {
    return c.key * kPi / clamp_adapted_lux(c, adapted_lux);
}

double combine_exposure(double sky, double measured, double weight) {
    if (!std::isfinite(measured) || measured <= 0 || !std::isfinite(weight)) return sky;
    weight = std::clamp(weight, 0.0, 1.0);
    return std::exp((1 - weight) * std::log(sky) + weight * std::log(measured));
}

void ExposureController::reset(double lux) {
    adapted_lux_ = clamp_adapted_lux(config_, lux);
    valid_ = true;
}

void ExposureController::update(double target_lux, double dt) {
    const double target = clamp_adapted_lux(config_, target_lux);
    if (!valid_) { reset(target); return; }
    if (!std::isfinite(dt) || dt <= 0) return;
    const double a = config_.adapt_tau_s > 0 ? 1 - std::exp(-dt / config_.adapt_tau_s) : 1.0;
    adapted_lux_ = std::exp(std::log(adapted_lux_) + a * (std::log(target) - std::log(adapted_lux_)));
}

} // namespace rg
