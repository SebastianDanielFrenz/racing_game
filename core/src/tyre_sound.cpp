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

std::shared_ptr<const std::vector<float>> make_roll_loop(const std::int16_t* pcm, std::size_t samples, int channels,
                                                         double crossfade_ms) {
    if (!pcm || channels < 1) return nullptr;
    const std::size_t n = samples / static_cast<std::size_t>(channels);
    std::vector<float> mono(n);
    for (std::size_t i = 0; i < n; ++i) {
        double sum = 0.0;
        for (int c = 0; c < channels; ++c) sum += pcm[i * static_cast<std::size_t>(channels) + static_cast<std::size_t>(c)] / 32768.0;
        mono[i] = static_cast<float>(sum / channels);
    }
    const std::size_t x = static_cast<std::size_t>(std::max(crossfade_ms, 1.0) * 48.0);
    if (n < 4 * x) return nullptr;
    // Loop of length m = n - x: the head x samples are blended with the x samples after the loop end, so the sample
    // after the last loop sample (src[m]) is the first loop sample exactly - no discontinuity at the seam.
    const std::size_t m = n - x;
    auto loop = std::make_shared<std::vector<float>>(mono.begin(), mono.begin() + static_cast<std::ptrdiff_t>(m));
    for (std::size_t i = 0; i < x; ++i) {
        const double w = 0.5 - 0.5 * std::cos(3.141592653589793 * static_cast<double>(i) / static_cast<double>(x)); // 0 -> ~1
        (*loop)[i] = static_cast<float>(mono[i] * w + mono[m + i] * (1.0 - w));
    }
    return loop;
}

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
    const double value = (roll_muted_ ? 0.0 : rough * roll_) + skid * squeal_level_;
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

TyreSoundBank::TyreSoundBank(int wheels, std::uint64_t base_seed, std::shared_ptr<const std::vector<std::int16_t>> squeal,
                             std::shared_ptr<const std::vector<float>> roll_loop, TyreRollConfig roll_cfg) {
    roll_loop_ = (roll_cfg.use_recording && roll_loop && !roll_loop->empty()) ? std::move(roll_loop) : nullptr;
    roll_cfg_ = roll_cfg;
    for (int i = 0; i < wheels; ++i) voices_.emplace_back(base_seed + static_cast<std::uint64_t>(i) * 997u, squeal);
    sealed_.assign(static_cast<std::size_t>(std::max(wheels, 0)), 1);
    contact_.assign(sealed_.size(), 0);
    values_.assign(static_cast<std::size_t>(std::max(wheels, 0)), 0.0f);
}

void TyreSoundBank::update(int wheel, const TyreSoundInput& in) {
    if (wheel < 0 || wheel >= wheels()) return;
    const std::size_t k = static_cast<std::size_t>(wheel);
    voices_[k].update(in);
    sealed_[k] = in.sealed;
    contact_[k] = in.load_n > 20.0;
    voices_[k].set_roll_muted(roll_loop_ && in.sealed);
    // One car-level layer: speed = the fastest wheel's chassis/tread speed.
    speed_ = std::max(std::abs(in.speed), std::abs(in.tread_speed));
}

float TyreSoundBank::roll_sample(double dt) {
    if (!roll_loop_) return 0.0f;
    // Share of wheels on a sealed surface with ground contact scales the layer (airborne: silent).
    double share = 0.0;
    for (std::size_t i = 0; i < sealed_.size(); ++i) share += (sealed_[i] && contact_[i]) ? 1.0 : 0.0;
    share /= std::max<std::size_t>(1, sealed_.size());
    const double ref = std::max(roll_cfg_.reference_speed_mps, 0.1);
    const double rate = clampd(speed_ / ref, roll_cfg_.min_rate, roll_cfg_.max_rate);
    const double fade = smoothstep(roll_cfg_.silent_below_mps, roll_cfg_.full_above_mps, speed_);
    const double target = std::min(roll_cfg_.gain_at_reference * std::pow(speed_ / ref, roll_cfg_.gain_exponent), roll_cfg_.max_gain) * fade * share;
    roll_gain_ += (target - roll_gain_) * (1.0 - std::exp(-dt / 0.02));
    if (target == 0.0 && roll_gain_ < 1e-6) roll_gain_ = 0.0;
    const auto& loop = *roll_loop_;
    const std::size_t n = loop.size();
    roll_pos_ += rate * 48000.0 * dt;
    while (roll_pos_ >= static_cast<double>(n)) roll_pos_ -= static_cast<double>(n);
    const std::size_t i0 = static_cast<std::size_t>(roll_pos_);
    const std::size_t i1 = i0 + 1 >= n ? 0 : i0 + 1;
    const double f = roll_pos_ - static_cast<double>(i0);
    return static_cast<float>((loop[i0] * (1.0 - f) + loop[i1] * f) * roll_gain_);
}

std::vector<float> TyreSoundBank::render(int frames, float gain) {
    const std::size_t w = voices_.size();
    std::vector<float> out(static_cast<std::size_t>(std::max(frames, 0)) * w);
    for (int f = 0; f < frames; ++f) {
        if (even_) roll_value_ += (roll_sample(1.0 / 24000.0) - roll_value_) * 0.65f;
        for (std::size_t k = 0; k < w; ++k) {
            if (even_) values_[k] += (voices_[k].sample(1.0 / 24000.0) - values_[k]) * 0.65f;
            out[static_cast<std::size_t>(f) * w + k] = (values_[k] + roll_value_) * gain;
        }
        even_ = !even_;
    }
    return out;
}

} // namespace rg
