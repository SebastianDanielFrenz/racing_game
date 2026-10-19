// rg/biquad.h - tiny second-order band-pass for the procedural noise layers (tyre hiss/tread tone, wind). Header-only,
// engine-neutral. RBJ "Audio EQ Cookbook" band-pass with constant 0 dB peak gain (BPF, constant peak gain = Q).
#pragma once

#include <algorithm>
#include <cmath>

namespace rg {

class BandPass {
public:
    // centre_hz is clamped to [20, 0.45 * sample_rate]; q to [0.3, 50].
    void set(double centre_hz, double q, double sample_rate) {
        const double w0 = 6.283185307179586 * std::clamp(centre_hz, 20.0, 0.45 * sample_rate) / sample_rate;
        const double alpha = std::sin(w0) / (2.0 * std::clamp(q, 0.3, 50.0));
        const double a0 = 1.0 + alpha;
        b0_ = alpha / a0;
        b2_ = -alpha / a0;
        a1_ = -2.0 * std::cos(w0) / a0;
        a2_ = (1.0 - alpha) / a0;
    }
    double process(double x) {
        const double y = b0_ * x + z1_;
        z1_ = /* b1 = 0 */ -a1_ * y + z2_;
        z2_ = b2_ * x - a2_ * y;
        return y;
    }
    void reset() { z1_ = z2_ = 0.0; }

    // Amplitude gain that turns the band-pass output of UNIT-POWER white noise into unit power again: 1 / sqrt(mean of |H|^2
    // over 0 .. Nyquist), integrated numerically (128 midpoints) so it stays right for wide bands near Nyquist. Clamped to 12.
    double unit_power_norm() const {
        double sum = 0.0;
        constexpr int n = 128;
        for (int i = 0; i < n; ++i) {
            const double w = 3.141592653589793 * (i + 0.5) / n;
            const double c1 = std::cos(w), s1 = std::sin(w), c2 = std::cos(2 * w), s2 = std::sin(2 * w);
            const double nr = b0_ + b2_ * c2, ni = -b2_ * s2;
            const double dr = 1.0 + a1_ * c1 + a2_ * c2, di = -a1_ * s1 - a2_ * s2;
            sum += (nr * nr + ni * ni) / (dr * dr + di * di);
        }
        return std::min(1.0 / std::sqrt(std::max(sum / n, 1e-12)), 12.0);
    }

private:
    double b0_ = 0.0, b2_ = 0.0, a1_ = 0.0, a2_ = 0.0, z1_ = 0.0, z2_ = 0.0;
};

} // namespace rg
