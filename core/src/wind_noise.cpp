#include "rg/wind_noise.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <set>

namespace rg {
namespace {
constexpr double kTau = 6.283185307179586;
double clampd(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }
double smoothstep(double a, double b, double x) {
    if (b <= a) return x < a ? 0.0 : 1.0;
    const double t = clampd((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
} // namespace

double wind_noise_amplitude(const WindNoiseConfig& c, double airspeed_mps) {
    if (!std::isfinite(airspeed_mps)) return 0.0;
    const double u = std::min(std::abs(airspeed_mps), c.max_speed_mps);
    if (u <= 0.0) return 0.0;
    return c.rms_at_reference * std::pow(u / c.reference_speed_mps, c.level_db_per_decade / 20.0) * smoothstep(0.0, c.full_above_mps, u);
}

double wind_noise_centre_hz(const WindNoiseConfig& c, double airspeed_mps) {
    const double u = std::isfinite(airspeed_mps) ? std::min(std::abs(airspeed_mps), c.max_speed_mps) : 0.0;
    const double f = c.centre_hz_at_reference * std::pow(std::max(u, 0.1) / c.reference_speed_mps, c.centre_exponent);
    return clampd(f, c.min_centre_hz, c.max_centre_hz);
}

bool parse_wind_noise_config(const std::string& json_text, WindNoiseConfig& out, std::string& error) {
    nlohmann::json root;
    try {
        root = nlohmann::json::parse(json_text);
    } catch (const nlohmann::json::exception& e) {
        error = std::string("wind_noise: not valid JSON: ") + e.what();
        return false;
    }
    if (!root.is_object() || !root.contains("wind") || !root["wind"].is_object()) {
        error = "wind_noise: top-level object with a \"wind\" block required";
        return false;
    }
    static const std::set<std::string> known{"reference_speed_kmh", "rms_at_reference", "level_db_per_decade", "max_speed_kmh", "full_above_kmh",
                                            "centre_hz_at_reference", "centre_exponent", "min_centre_hz", "max_centre_hz", "q", "gust_depth",
                                            "gust_rate_hz", "smoothing_s"};
    WindNoiseConfig c;
    for (const auto& [key, value] : root["wind"].items()) {
        if (key == "status" || key == "source" || (key.size() > 5 && key.compare(key.size() - 5, 5, "_note") == 0)) continue;
        if (!known.count(key)) { error = "wind_noise: unknown key \"" + key + "\""; return false; }
        if (!value.is_number()) { error = "wind_noise: \"" + key + "\" must be a number"; return false; }
        const double v = value.get<double>();
        if (!std::isfinite(v)) { error = "wind_noise: \"" + key + "\" is not finite"; return false; }
        if (key == "reference_speed_kmh") c.reference_speed_mps = v / 3.6;
        else if (key == "rms_at_reference") c.rms_at_reference = v;
        else if (key == "level_db_per_decade") c.level_db_per_decade = v;
        else if (key == "max_speed_kmh") c.max_speed_mps = v / 3.6;
        else if (key == "full_above_kmh") c.full_above_mps = v / 3.6;
        else if (key == "centre_hz_at_reference") c.centre_hz_at_reference = v;
        else if (key == "centre_exponent") c.centre_exponent = v;
        else if (key == "min_centre_hz") c.min_centre_hz = v;
        else if (key == "max_centre_hz") c.max_centre_hz = v;
        else if (key == "q") c.q = v;
        else if (key == "gust_depth") c.gust_depth = v;
        else if (key == "gust_rate_hz") c.gust_rate_hz = v;
        else if (key == "smoothing_s") c.smoothing_s = v;
    }
    auto bad = [&](const char* what) { error = std::string("wind_noise: ") + what; return false; };
    if (c.reference_speed_mps < 5.0 / 3.6) return bad("reference_speed_kmh must be >= 5");
    if (c.rms_at_reference <= 0.0 || c.rms_at_reference > 1.0) return bad("rms_at_reference must be in (0, 1]");
    if (c.level_db_per_decade < 20.0 || c.level_db_per_decade > 90.0) return bad("level_db_per_decade must be in [20, 90]");
    // The cap must lie well above any road car: a clamp at a walking-pace-to-motorway speed is exactly the "flat above X" defect.
    if (c.max_speed_mps < 400.0 / 3.6) return bad("max_speed_kmh must be >= 400 (the law must not saturate below the hypercar's top speed)");
    if (c.full_above_mps <= 0.0 || c.full_above_mps > 10.0 / 3.6) return bad("full_above_kmh must be in (0, 10]");
    if (c.centre_hz_at_reference < 50.0 || c.centre_hz_at_reference > 8000.0) return bad("centre_hz_at_reference must be in [50, 8000]");
    if (c.centre_exponent < 0.0 || c.centre_exponent > 2.0) return bad("centre_exponent must be in [0, 2]");
    if (c.min_centre_hz < 20.0 || c.max_centre_hz <= c.min_centre_hz || c.max_centre_hz > 0.45 * WindNoise::rate) return bad("min_centre_hz/max_centre_hz out of range");
    if (c.q < 0.3 || c.q > 10.0) return bad("q must be in [0.3, 10]");
    if (c.gust_depth < 0.0 || c.gust_depth > 0.5) return bad("gust_depth must be in [0, 0.5]");
    if (c.gust_rate_hz <= 0.0 || c.gust_rate_hz > 10.0) return bad("gust_rate_hz must be in (0, 10]");
    if (c.smoothing_s < 0.0 || c.smoothing_s > 2.0) return bad("smoothing_s must be in [0, 2]");
    out = c;
    error.clear();
    return true;
}

WindNoise::WindNoise(const WindNoiseConfig& cfg, std::uint64_t seed) : cfg_(cfg), rng_(seed * 0x9E3779B97F4A7C15ull + 0xC0FFEEull) {
    centre_ = wind_noise_centre_hz(cfg_, 0.0);
}

void WindNoise::set_airspeed(double mps) { target_ = std::isfinite(mps) ? std::min(std::abs(mps), 1000.0) : 0.0; }

double WindNoise::noise() { // xorshift64*, uniform [-1, 1)
    rng_ ^= rng_ >> 12;
    rng_ ^= rng_ << 25;
    rng_ ^= rng_ >> 27;
    const std::uint64_t r = rng_ * 0x2545F4914F6CDD1Dull;
    return static_cast<double>(r >> 40) / 8388608.0 - 1.0;
}

float WindNoise::next(double gain) {
    constexpr double dt = 1.0 / rate;
    u_ += (target_ - u_) * (cfg_.smoothing_s > 0.0 ? 1.0 - std::exp(-dt / cfg_.smoothing_s) : 1.0);
    if ((block_++ & 15) == 0) { // filter and law at 3 kHz, the audio runs at 48 kHz
        amp_ = wind_noise_amplitude(cfg_, u_);
        centre_ = wind_noise_centre_hz(cfg_, u_);
        band_.set(centre_, cfg_.q, rate);
        norm_ = band_.unit_power_norm();
    }
    // Slow gust modulation: one-pole low-passed noise, normalised to unit standard deviation.
    const double a = 1.0 - std::exp(-kTau * cfg_.gust_rate_hz * dt);
    gust_ += a * (noise() - gust_);
    const double gust_unit = gust_ * std::sqrt((2.0 - a) / a) / 0.5773502691896258;
    const double mod = std::max(0.2, 1.0 + cfg_.gust_depth * gust_unit);
    const double white = noise() / 0.5773502691896258; // unit power
    const double v = band_.process(white) * norm_ * amp_ * mod * gain;
    return static_cast<float>(v);
}

std::vector<float> WindNoise::render(int frames, double gain) {
    frames = std::clamp(frames, 0, 16384);
    gain = std::isfinite(gain) ? std::clamp(gain, 0.0, 1.0) : 0.0;
    std::vector<float> out(static_cast<std::size_t>(frames));
    for (auto& s : out) s = next(gain);
    return out;
}

void WindNoise::mix_into(float* interleaved, int channels, int frames, double gain) {
    if (!interleaved || channels <= 0 || frames <= 0) return;
    gain = std::isfinite(gain) ? std::clamp(gain, 0.0, 1.0) : 0.0;
    for (int f = 0; f < frames; ++f) {
        const float s = next(gain);
        for (int c = 0; c < channels; ++c) interleaved[static_cast<std::size_t>(f) * static_cast<std::size_t>(channels) + static_cast<std::size_t>(c)] += s;
    }
}

} // namespace rg
