#include "rg/tyre_sound.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <set>

namespace rg {
namespace {
constexpr double kTau = 6.283185307179586;
constexpr std::size_t kCrossfade = 1200; // 50 ms overlap at 24 kHz
constexpr double kUniformStd = 0.5773502691896258; // standard deviation of a uniform [-1, 1) variate
constexpr double kLayerRate = 24000.0;
double clampd(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }
double smoothstep(double a, double b, double x) {
    if (a == b) return x < a ? 0.0 : 1.0;
    const double t = clampd((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
// Transparent below 0.6, bounded by 1.0: keeps the loudest speeds from hard-clipping without flattening the law.
double soft_limit(double x) {
    const double a = std::abs(x);
    if (a <= 0.6) return x;
    return std::copysign(0.6 + 0.4 * std::tanh((a - 0.6) / 0.4), x);
}
bool ends_with_note(const std::string& key) { return key.size() > 5 && key.compare(key.size() - 5, 5, "_note") == 0; }
} // namespace

const TyreSurfaceNoise* TyreRollConfig::find_surface(const std::string& name) const {
    for (const auto& s : surfaces)
        if (s.name == name) return &s;
    return nullptr;
}

double tyre_roll_gain(const TyreRollLaw& law, double v) {
    if (!std::isfinite(v)) return 0.0;
    const double s = std::min(std::abs(v), law.max_speed_mps);
    if (s <= 0.0) return 0.0;
    return law.gain_at_reference * std::pow(s / law.reference_speed_mps, law.level_db_per_decade / 20.0) *
           smoothstep(law.silent_below_mps, law.full_above_mps, s);
}

double tyre_roll_rate(const TyreRollConfig& c, double v) {
    const double s = std::isfinite(v) ? std::min(std::abs(v), c.level.max_speed_mps) : 0.0;
    return clampd(std::pow(std::max(s, 0.01) / c.rate_reference_speed_mps, c.rate_exponent), c.min_rate, c.max_rate);
}

double tyre_hiss_centre_hz(const TyreRollConfig& c, double v) {
    const double s = std::isfinite(v) ? std::min(std::abs(v), c.level.max_speed_mps) : 0.0;
    return clampd(c.hiss_centre_hz * std::pow(std::max(s, 0.5) / c.level.reference_speed_mps, c.hiss_centre_exponent), 200.0, 10000.0);
}

double tyre_tread_tone_hz(const TyreRollConfig& c, double tread_speed_mps) {
    const double s = std::isfinite(tread_speed_mps) ? std::min(std::abs(tread_speed_mps), c.level.max_speed_mps) : 0.0;
    return clampd(s / c.tread_pitch_m, 30.0, 10000.0);
}

bool parse_tyre_roll_config(const std::string& json_text, TyreRollConfig& out, std::string& error) {
    nlohmann::json root;
    try {
        root = nlohmann::json::parse(json_text);
    } catch (const nlohmann::json::exception& e) {
        error = std::string("tyre_sound: not valid JSON: ") + e.what();
        return false;
    }
    if (!root.is_object() || !root.contains("roll") || !root["roll"].is_object()) {
        error = "tyre_sound: top-level object with a \"roll\" block required";
        return false;
    }
    static const std::set<std::string> numeric{"reference_speed_kmh", "gain_at_reference", "level_db_per_decade", "max_speed_kmh", "silent_below_kmh",
        "full_above_kmh", "rate_reference_speed_kmh", "rate_exponent", "min_rate", "max_rate", "hiss_db", "hiss_centre_hz", "hiss_centre_exponent",
        "hiss_q", "tread_pitch_m", "tone_db", "tone_q", "tone_sine_share", "procedural_gain_at_reference", "loop_crossfade_ms"};
    TyreRollConfig c;
    for (const auto& [key, value] : root["roll"].items()) {
        if (key == "status" || key == "source" || key == "file" || ends_with_note(key)) continue;
        if (key == "roll_source") {
            if (!value.is_string() || (value.get<std::string>() != "recording" && value.get<std::string>() != "procedural")) {
                error = "tyre_sound: roll_source must be \"recording\" or \"procedural\"";
                return false;
            }
            c.use_recording = value.get<std::string>() == "recording";
            continue;
        }
        if (key == "surfaces") {
            if (!value.is_object()) { error = "tyre_sound: \"surfaces\" must be an object"; return false; }
            for (const auto& [name, entry] : value.items()) {
                if (ends_with_note(name)) continue;
                if (name.empty() || !entry.is_object()) { error = "tyre_sound: surfaces." + name + " must be an object"; return false; }
                TyreSurfaceNoise s;
                s.name = name;
                for (const auto& [k, v] : entry.items()) {
                    if (k == "sealed" && v.is_boolean()) s.sealed = v.get<bool>();
                    else if (k == "db" && v.is_number() && std::isfinite(v.get<double>())) s.db = v.get<double>();
                    else if (k == "source" || ends_with_note(k)) continue;
                    else { error = "tyre_sound: surfaces." + name + ": bad or unknown key \"" + k + "\""; return false; }
                }
                if (s.db < -30.0 || s.db > 20.0) { error = "tyre_sound: surfaces." + name + ".db must be in [-30, 20]"; return false; }
                c.surfaces.push_back(s);
            }
            continue;
        }
        if (!numeric.count(key)) { error = "tyre_sound: unknown key \"" + key + "\""; return false; }
        if (!value.is_number() || !std::isfinite(value.get<double>())) { error = "tyre_sound: \"" + key + "\" must be a finite number"; return false; }
        const double v = value.get<double>();
        if (key == "reference_speed_kmh") c.level.reference_speed_mps = v / 3.6;
        else if (key == "gain_at_reference") c.level.gain_at_reference = v;
        else if (key == "level_db_per_decade") c.level.level_db_per_decade = v;
        else if (key == "max_speed_kmh") c.level.max_speed_mps = v / 3.6;
        else if (key == "silent_below_kmh") c.level.silent_below_mps = v / 3.6;
        else if (key == "full_above_kmh") c.level.full_above_mps = v / 3.6;
        else if (key == "rate_reference_speed_kmh") c.rate_reference_speed_mps = v / 3.6;
        else if (key == "rate_exponent") c.rate_exponent = v;
        else if (key == "min_rate") c.min_rate = v;
        else if (key == "max_rate") c.max_rate = v;
        else if (key == "hiss_db") c.hiss_db = v;
        else if (key == "hiss_centre_hz") c.hiss_centre_hz = v;
        else if (key == "hiss_centre_exponent") c.hiss_centre_exponent = v;
        else if (key == "hiss_q") c.hiss_q = v;
        else if (key == "tread_pitch_m") c.tread_pitch_m = v;
        else if (key == "tone_db") c.tone_db = v;
        else if (key == "tone_q") c.tone_q = v;
        else if (key == "tone_sine_share") c.tone_sine_share = v;
        else if (key == "procedural_gain_at_reference") c.procedural_gain_at_reference = v;
        else if (key == "loop_crossfade_ms") c.loop_crossfade_ms = v;
    }
    auto bad = [&](const char* what) { error = std::string("tyre_sound: ") + what; return false; };
    if (c.level.reference_speed_mps < 5.0 / 3.6) return bad("reference_speed_kmh must be >= 5");
    if (c.level.gain_at_reference <= 0.0 || c.level.gain_at_reference > 2.0) return bad("gain_at_reference must be in (0, 2]");
    if (c.level.level_db_per_decade < 15.0 || c.level.level_db_per_decade > 60.0) return bad("level_db_per_decade must be in [15, 60]");
    // The cap must lie well above any road car: a clamp at 50-200 km/h is exactly the "flat above X" defect this law replaced.
    if (c.level.max_speed_mps < 400.0 / 3.6) return bad("max_speed_kmh must be >= 400 (the law must not saturate below the hypercar's top speed)");
    if (c.level.silent_below_mps < 0.0 || c.level.full_above_mps <= c.level.silent_below_mps || c.level.full_above_mps > 10.0 / 3.6)
        return bad("silent_below_kmh/full_above_kmh must satisfy 0 <= silent_below < full_above <= 10");
    if (c.rate_reference_speed_mps < 5.0 / 3.6) return bad("rate_reference_speed_kmh must be >= 5");
    if (c.rate_exponent < 0.05 || c.rate_exponent > 1.5) return bad("rate_exponent must be in [0.05, 1.5] (0 would freeze the pitch)");
    if (c.min_rate <= 0.0 || c.max_rate <= c.min_rate || c.min_rate > 0.5 || c.max_rate < 3.0) return bad("min_rate must be in (0, 0.5], max_rate >= 3 and > min_rate");
    if (c.hiss_db < -60.0 || c.hiss_db > 12.0 || c.tone_db < -60.0 || c.tone_db > 12.0) return bad("hiss_db/tone_db must be in [-60, 12]");
    if (c.hiss_centre_hz < 200.0 || c.hiss_centre_hz > 8000.0) return bad("hiss_centre_hz must be in [200, 8000]");
    if (c.hiss_centre_exponent < 0.0 || c.hiss_centre_exponent > 1.5) return bad("hiss_centre_exponent must be in [0, 1.5]");
    if (c.hiss_q < 0.3 || c.hiss_q > 20.0 || c.tone_q < 0.3 || c.tone_q > 50.0) return bad("hiss_q/tone_q out of range");
    if (c.tread_pitch_m < 0.005 || c.tread_pitch_m > 0.2) return bad("tread_pitch_m must be in [0.005, 0.2]");
    if (c.tone_sine_share < 0.0 || c.tone_sine_share > 1.0) return bad("tone_sine_share must be in [0, 1]");
    if (c.procedural_gain_at_reference <= 0.0 || c.procedural_gain_at_reference > 1.0) return bad("procedural_gain_at_reference must be in (0, 1]");
    if (c.loop_crossfade_ms < 1.0 || c.loop_crossfade_ms > 1000.0) return bad("loop_crossfade_ms must be in [1, 1000]");
    out = c;
    error.clear();
    return true;
}

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
    surface_gain_ = std::pow(10.0, clampd(in.surface_db, -30.0, 20.0) / 20.0);
    const double v = std::abs(in.speed);
    contact_ = in.load_n > 20.0;
    const double load_scale = contact_ ? std::sqrt(clampd(in.load_n / 4000.0, 0.0, 2.0)) : 0.0;
    const double longitudinal = std::max(v, std::abs(in.tread_speed)) * std::min(std::abs(in.slip_ratio), 1.0);
    const double lateral = v * std::abs(std::sin(in.slip_angle));
    const double work = std::abs(in.fx) * longitudinal + std::abs(in.fy) * lateral;
    double scrub = clampd(std::sqrt(work / 15000.0), 0.0, 1.8);
    if (longitudinal + lateral < 0.05) scrub = 0.0;
    // The shared asphalt profile: roll 1.0, tone 1.0, base frequency 1100 Hz, grain 0. The roll level follows the same
    // power law as the car-level layer (tyre_roll_gain); the old 0.014 * min(v / 30, 3)^0.7 saturated at 324 km/h and grew 4 dB per doubling.
    roll_ = tyre_roll_gain(law_, v) * load_scale * surface_gain_;
    const double slip_demand = std::sqrt(std::pow(std::abs(in.slip_ratio) / 0.10, 2.0) + std::pow(std::abs(in.slip_angle) / 0.08, 2.0));
    double skid_gate = smoothstep(1.0, 2.0, slip_demand);
    skid_gate *= smoothstep(0.5, 2.0, longitudinal + lateral);
    slide_ = 0.035 * scrub * load_scale * skid_gate;
    tone_gain_ = 1.0 - wetness_ * 0.7;
    frequency_ = clampd(1100.0 + 10.0 * v + 35.0 * std::sqrt(longitudinal + lateral), 200.0, 5000.0);
    grain_ = 0.0;
    roll_cutoff_ = clampd(250.0 + std::min(v, law_.max_speed_mps) * 32.0 + grain_ * 450.0, 200.0, 5000.0);
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
    roll_cfg_ = std::move(roll_cfg);
    TyreRollLaw procedural = roll_cfg_.level;
    procedural.gain_at_reference = roll_cfg_.procedural_gain_at_reference;
    for (int i = 0; i < wheels; ++i) {
        voices_.emplace_back(base_seed + static_cast<std::uint64_t>(i) * 997u, squeal);
        voices_.back().set_roll_law(procedural);
    }
    const std::size_t n = static_cast<std::size_t>(std::max(wheels, 0));
    sealed_.assign(n, 1);
    contact_.assign(n, 0);
    surface_gain_.assign(n, 1.0);
    wheel_speed_.assign(n, 0.0);
    wheel_tread_.assign(n, 0.0);
    values_.assign(n, 0.0f);
    if (roll_loop_) { // the recording's own RMS is the unit the hiss and tone levels are given in
        double sum = 0.0;
        for (float x : *roll_loop_) sum += static_cast<double>(x) * x;
        loop_rms_ = std::max(std::sqrt(sum / static_cast<double>(roll_loop_->size())), 1e-4);
    }
}

void TyreSoundBank::update(int wheel, const TyreSoundInput& in) {
    if (wheel < 0 || wheel >= wheels()) return;
    const std::size_t k = static_cast<std::size_t>(wheel);
    voices_[k].update(in);
    sealed_[k] = in.sealed;
    contact_[k] = in.load_n > 20.0;
    surface_gain_[k] = std::pow(10.0, clampd(in.surface_db, -30.0, 20.0) / 20.0);
    voices_[k].set_roll_muted(roll_loop_ && in.sealed);
    wheel_speed_[k] = std::max(std::abs(in.speed), std::abs(in.tread_speed));
    wheel_tread_[k] = std::abs(in.tread_speed);
}

double TyreSoundBank::roll_noise() { // xorshift64*, uniform in [-1, 1)
    roll_rng_ ^= roll_rng_ >> 12;
    roll_rng_ ^= roll_rng_ << 25;
    roll_rng_ ^= roll_rng_ >> 27;
    const std::uint64_t r = roll_rng_ * 0x2545F4914F6CDD1Dull;
    return static_cast<double>(r >> 40) / 8388608.0 - 1.0;
}

float TyreSoundBank::roll_sample(double dt) {
    if (!roll_loop_) return 0.0f;
    // Wheels on a sealed surface with ground contact make up the layer (airborne: silent). Its level is the mean of their
    // surface gains over ALL wheels, its speed the mean road speed / tread speed of those wheels.
    double amp = 0.0, speed = 0.0, tread = 0.0;
    int n = 0;
    for (std::size_t i = 0; i < sealed_.size(); ++i) {
        if (!(sealed_[i] && contact_[i])) continue;
        amp += surface_gain_[i];
        speed += wheel_speed_[i];
        tread += wheel_tread_[i];
        ++n;
    }
    layer_amp_ = amp / static_cast<double>(std::max<std::size_t>(1, sealed_.size()));
    if (n > 0) { speed /= n; tread /= n; }
    const double a = 1.0 - std::exp(-dt / 0.05); // 50 ms: no zipper in rate / pitch, still follows a hard launch
    speed_s_ += (speed - speed_s_) * a;
    tread_s_ += (tread - tread_s_) * a;
    const double v = std::min(speed_s_, roll_cfg_.level.max_speed_mps);
    const double rate = tyre_roll_rate(roll_cfg_, v);
    const double target = tyre_roll_gain(roll_cfg_.level, v) * layer_amp_;
    roll_gain_ += (target - roll_gain_) * (1.0 - std::exp(-dt / 0.02));
    if (target == 0.0 && roll_gain_ < 1e-6) roll_gain_ = 0.0;

    // recording
    const auto& loop = *roll_loop_;
    const std::size_t len = loop.size();
    roll_pos_ += rate * 48000.0 * dt;
    while (roll_pos_ >= static_cast<double>(len)) roll_pos_ -= static_cast<double>(len);
    const std::size_t i0 = static_cast<std::size_t>(roll_pos_);
    const std::size_t i1 = i0 + 1 >= len ? 0 : i0 + 1;
    const double f = roll_pos_ - static_cast<double>(i0);
    double value = loop[i0] * (1.0 - f) + loop[i1] * f;

    // band-pass hiss and tread tone, levels in units of the recording's RMS
    if ((roll_block_++ & 15) == 0) {
        const double hiss_hz = tyre_hiss_centre_hz(roll_cfg_, v);
        hiss_band_.set(hiss_hz, roll_cfg_.hiss_q, kLayerRate);
        hiss_norm_ = hiss_band_.unit_power_norm();
        tone_hz_ = tyre_tread_tone_hz(roll_cfg_, tread_s_);
        tone_band_.set(tone_hz_, roll_cfg_.tone_q, kLayerRate);
        tone_norm_ = tone_band_.unit_power_norm();
        roll_state_.hiss_hz = hiss_hz;
    }
    const double white = roll_noise() / kUniformStd; // unit power
    value += hiss_band_.process(white) * hiss_norm_ * std::pow(10.0, roll_cfg_.hiss_db / 20.0) * loop_rms_;
    tone_phase_ = std::fmod(tone_phase_ + tone_hz_ * dt, 1.0);
    const double tone_noise = tone_band_.process(white) * tone_norm_;
    const double tone_sine = 1.4142135623730951 * std::sin(kTau * tone_phase_);
    const double tone_follow = clampd(tread_s_ / std::max(speed_s_, 0.5), 0.0, 1.0); // a locked wheel has no tread pass
    value += (std::sqrt(roll_cfg_.tone_sine_share) * tone_sine + std::sqrt(1.0 - roll_cfg_.tone_sine_share) * tone_noise) *
             std::pow(10.0, roll_cfg_.tone_db / 20.0) * loop_rms_ * tone_follow;

    roll_state_.gain = roll_gain_;
    roll_state_.rate = rate;
    roll_state_.tone_hz = tone_hz_;
    roll_state_.speed = speed_s_;
    return static_cast<float>(soft_limit(value * roll_gain_));
}

std::vector<float> TyreSoundBank::render(int frames, float gain) {
    const std::size_t w = voices_.size();
    std::vector<float> out(static_cast<std::size_t>(std::max(frames, 0)) * w);
    for (int f = 0; f < frames; ++f) {
        if (even_) roll_value_ += (roll_sample(1.0 / kLayerRate) - roll_value_) * 0.65f;
        for (std::size_t k = 0; k < w; ++k) {
            if (even_) values_[k] += (voices_[k].sample(1.0 / kLayerRate) - values_[k]) * 0.65f;
            out[static_cast<std::size_t>(f) * w + k] = (values_[k] + roll_value_) * gain;
        }
        even_ = !even_;
    }
    return out;
}

} // namespace rg
