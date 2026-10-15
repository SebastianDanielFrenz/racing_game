#include "rg/tyre_sound.h"

#include <algorithm>
#include <cmath>

namespace rg {
namespace {
constexpr double kTau = 6.283185307179586;
constexpr std::size_t kCrossfade = 1200; // 50 ms overlap at 24 kHz
double clampd(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }
double smoothstep(double a, double b, double x) {
    if (a == b) return x < a ? 0.0 : 1.0;
    const double t = clampd((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
} // namespace

TyreVoice::TyreVoice(std::uint64_t seed, std::shared_ptr<const std::vector<std::int16_t>> squeal)
    : squeal_(std::move(squeal)), rng_(seed * 0x9E3779B97F4A7C15ull + 0x1234567ull) {
    const std::size_t count = squeal_ ? squeal_->size() : 0;
    squeal_pos_ = static_cast<std::size_t>(seed % std::max<std::size_t>(1, count > 1200 ? count - 1200 : 1));
}

float TyreVoice::noise() { // xorshift64*, uniform in [-1, 1)
    rng_ ^= rng_ >> 12;
    rng_ ^= rng_ << 25;
    rng_ ^= rng_ >> 27;
    const std::uint64_t r = rng_ * 0x2545F4914F6CDD1Dull;
    return static_cast<float>(static_cast<double>(r >> 40) / 8388608.0 - 1.0);
}

void TyreVoice::update(const TyreSoundInput& in) {
    wetness_ = clampd(in.wetness, 0.0, 1.0);
    const double v = std::abs(in.speed);
    contact_ = in.load_n > 20.0;
    const double load_scale = contact_ ? std::sqrt(clampd(in.load_n / 4000.0, 0.0, 2.0)) : 0.0;
    const double longitudinal = std::max(v, std::abs(in.tread_speed)) * std::min(std::abs(in.slip_ratio), 1.0);
    const double lateral = v * std::abs(std::sin(in.slip_angle));
    const double work = std::abs(in.fx) * longitudinal + std::abs(in.fy) * lateral;
    double scrub = clampd(std::sqrt(work / 15000.0), 0.0, 1.8);
    if (longitudinal + lateral < 0.05) scrub = 0.0;
    // The shared asphalt profile: roll 1.0, tone 1.0, base frequency 1100 Hz, grain 0.
    roll_ = 0.014 * std::pow(std::min(v / 30.0, 3.0), 0.7) * load_scale;
    const double slip_demand = std::sqrt(std::pow(std::abs(in.slip_ratio) / 0.10, 2.0) + std::pow(std::abs(in.slip_angle) / 0.08, 2.0));
    double skid_gate = smoothstep(1.0, 2.0, slip_demand);
    skid_gate *= smoothstep(0.5, 2.0, longitudinal + lateral);
    slide_ = 0.035 * scrub * load_scale * skid_gate;
    tone_gain_ = 1.0 - wetness_ * 0.7;
    frequency_ = clampd(1100.0 + 10.0 * v + 35.0 * std::sqrt(longitudinal + lateral), 200.0, 5000.0);
    grain_ = 0.0;
    roll_cutoff_ = clampd(250.0 + v * 32.0 + grain_ * 450.0, 200.0, 5000.0);
}

float TyreVoice::sample(double dt) {
    const double n = noise();
    low_ += (1.0 - std::exp(-kTau * roll_cutoff_ * dt)) * (n - low_);
    phase_ = std::fmod(phase_ + frequency_ * dt, 1.0);
    const double rough = low_ + grain_ * (n - low_);
    squeal_level_ += (slide_ * (1.0 - wetness_ * 0.7) - squeal_level_) * (1.0 - std::exp(-dt / 0.04));
    const double skid = squeal_sample();
    const double value = rough * roll_ + skid * squeal_level_;
    return contact_ ? static_cast<float>(clampd(value, -0.4, 0.4)) : 0.0f;
}

float TyreVoice::squeal_sample() {
    const std::size_t count = squeal_ ? squeal_->size() : 0;
    if (count <= kCrossfade) return 0.0f;
    const auto& pcm = *squeal_;
    double value = pcm[squeal_pos_] / 32768.0;
    if (squeal_pos_ >= count - kCrossfade) {
        const std::size_t head = squeal_pos_ - (count - kCrossfade);
        const double blend = static_cast<double>(head) / kCrossfade;
        value += (pcm[head] / 32768.0 - value) * blend;
    }
    if (++squeal_pos_ >= count) squeal_pos_ = kCrossfade;
    return static_cast<float>(value);
}

TyreSoundBank::TyreSoundBank(int wheels, std::uint64_t base_seed, std::shared_ptr<const std::vector<std::int16_t>> squeal) {
    for (int i = 0; i < wheels; ++i) voices_.emplace_back(base_seed + static_cast<std::uint64_t>(i) * 997u, squeal);
    values_.assign(static_cast<std::size_t>(std::max(wheels, 0)), 0.0f);
}

void TyreSoundBank::update(int wheel, const TyreSoundInput& in) {
    if (wheel >= 0 && wheel < wheels()) voices_[static_cast<std::size_t>(wheel)].update(in);
}

std::vector<float> TyreSoundBank::render(int frames, float gain) {
    const std::size_t w = voices_.size();
    std::vector<float> out(static_cast<std::size_t>(std::max(frames, 0)) * w);
    for (int f = 0; f < frames; ++f) {
        for (std::size_t k = 0; k < w; ++k) {
            if (even_) values_[k] += (voices_[k].sample(1.0 / 24000.0) - values_[k]) * 0.65f;
            out[static_cast<std::size_t>(f) * w + k] = values_[k] * gain;
        }
        even_ = !even_;
    }
    return out;
}

} // namespace rg
