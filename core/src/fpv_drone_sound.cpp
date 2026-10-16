#include "rg/fpv_drone_sound.h"

#include "json_util.h"

#include <cmath>

namespace rg {

namespace {
constexpr double kTwoPi = 6.283185307179586;
constexpr double kRate = 48000.0;
// Each rotor runs a little off the others: the beating is what makes four props sound like a drone.
constexpr double kDetune[4] = {1.0, 1.013, 0.987, 1.027};
constexpr double kRotorGain[4] = {0.30, 0.26, 0.26, 0.22};
} // namespace

std::optional<FpvAudioConfig> load_fpv_audio_config(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        if (err) *err = "fpv_drone audio: cannot read " + path;
        return std::nullopt;
    }
    const auto root = detail::parse_json(*text);
    if (!root || !root->is_object()) {
        if (err) *err = "fpv_drone audio: " + path + " is not a JSON object";
        return std::nullopt;
    }
    const auto it = root->find("audio");
    if (it == root->end() || !it->is_object()) {
        if (err) *err = "fpv_drone audio: missing 'audio' object";
        return std::nullopt;
    }
    FpvAudioConfig c;
    const struct { const char* key; double* out; bool positive; } fields[] = {
        {"idle_hz", &c.idle_hz, true},
        {"max_hz", &c.max_hz, true},
        {"idle_level", &c.idle_level, false},
        {"max_level", &c.max_level, false},
        {"activity_rise_per_s", &c.activity_rise_per_s, true},
        {"activity_fall_per_s", &c.activity_fall_per_s, true},
        {"yaw_activity_weight", &c.yaw_activity_weight, false},
    };
    for (const auto& f : fields) {
        if (!detail::get_number(*it, f.key, *f.out) || !std::isfinite(*f.out) || (f.positive ? !(*f.out > 0.0) : *f.out < 0.0)) {
            if (err) *err = std::string("fpv_drone audio: '") + f.key + "' missing or out of range";
            return std::nullopt;
        }
    }
    if (c.max_hz < c.idle_hz || c.max_level < c.idle_level || c.max_level > 1.0) {
        if (err) *err = "fpv_drone audio: max_hz/max_level must not be below idle, max_level at most 1";
        return std::nullopt;
    }
    return c;
}

FpvRotorVoice::FpvRotorVoice(const FpvAudioConfig& config, std::uint64_t seed)
    : config_(config), rng_(seed ? seed : 1) {
    for (int i = 0; i < 4; ++i) phase_[i] = 0.17 * i; // fixed offsets: deterministic start
}

double FpvRotorVoice::frequency_hz() const { return config_.idle_hz + (config_.max_hz - config_.idle_hz) * activity_; }
double FpvRotorVoice::level() const { return config_.idle_level + (config_.max_level - config_.idle_level) * activity_; }

float FpvRotorVoice::noise() {
    rng_ = rng_ * 6364136223846793005ull + 1442695040888963407ull;
    return static_cast<float>(static_cast<std::int64_t>(rng_ >> 11) / 4503599627370496.0 - 1.0); // [-1, 1)
}

std::vector<float> FpvRotorVoice::render(int frames, float gain) {
    std::vector<float> out;
    if (frames <= 0) return out;
    out.resize(static_cast<std::size_t>(frames));
    const double dt = 1.0 / kRate;
    for (int n = 0; n < frames; ++n) {
        const double rate = target_activity_ > activity_ ? config_.activity_rise_per_s : config_.activity_fall_per_s;
        activity_ += (target_activity_ - activity_) * (1.0 - std::exp(-rate * dt));
        const double f = frequency_hz();
        double v = 0.0;
        for (int r = 0; r < 4; ++r) {
            phase_[r] += kTwoPi * f * kDetune[r] * dt;
            if (phase_[r] > kTwoPi) phase_[r] -= kTwoPi;
            const double p = phase_[r];
            // blade-pass fundamental plus 3 harmonics
            v += kRotorGain[r] * (std::sin(p) + 0.55 * std::sin(2.0 * p) + 0.30 * std::sin(3.0 * p) + 0.15 * std::sin(4.0 * p));
        }
        // air noise, low-passed, a little more of it under load
        air_ += (static_cast<double>(noise()) - air_) * 0.25;
        v += air_ * (0.10 + 0.25 * activity_);
        out[static_cast<std::size_t>(n)] = static_cast<float>(v * level() * 0.5) * gain;
    }
    return out;
}

} // namespace rg
