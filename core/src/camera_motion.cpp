// camera_motion.cpp - see rg/camera_motion.h.
#include "rg/camera_motion.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rg {

namespace {

using detail::json;

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxSubstepS = 1.0 / 240.0;
constexpr double kMaxSimGapS = 0.5; // a longer gap (pause, load) restarts the estimate

bool fail(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

bool read_num(const json& o, const char* section, const char* key, double lo, double hi, double& out,
              std::string* err) {
    if (!detail::get_number(o, key, out) || !std::isfinite(out))
        return fail(err, std::string(section) + "." + key + ": missing or not a number");
    if (out < lo || out > hi)
        return fail(err, std::string(section) + "." + key + ": out of range");
    return true;
}

bool read_triple(const json& o, const char* section, const char* key, double lo, double hi,
                 std::array<double, 3>& out, std::string* err) {
    const auto it = o.find(key);
    if (it == o.end() || !it->is_array() || it->size() != 3)
        return fail(err, std::string(section) + "." + key + ": expected an array of 3 numbers");
    for (size_t i = 0; i < 3; ++i) {
        if (!(*it)[i].is_number()) return fail(err, std::string(section) + "." + key + ": not a number");
        out[i] = (*it)[i].get<double>();
        if (!std::isfinite(out[i]) || out[i] < lo || out[i] > hi)
            return fail(err, std::string(section) + "." + key + ": out of range");
    }
    return true;
}

const json* object(const json& o, const char* key, std::string* err) {
    const auto it = o.find(key);
    if (it == o.end() || !it->is_object()) {
        fail(err, std::string(key) + ": missing object");
        return nullptr;
    }
    return &*it;
}

double ramp(double v, double start, double full) {
    if (full <= start) return v >= full ? 1.0 : 0.0;
    return std::clamp((v - start) / (full - start), 0.0, 1.0);
}

double lowpass_alpha(double hz, double dt) { return 1.0 - std::exp(-2.0 * kPi * hz * dt); }

} // namespace

std::optional<CameraMotionConfig> load_camera_motion_config(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) { fail(err, "cannot read " + path); return std::nullopt; }
    const auto root = detail::parse_json(*text);
    if (!root || !root->is_object()) { fail(err, path + ": not a JSON object"); return std::nullopt; }
    std::string format;
    if (!detail::get_string(*root, "format", format) || format != "rg.camera_motion/1") {
        fail(err, path + ": format must be \"rg.camera_motion/1\"");
        return std::nullopt;
    }
    CameraMotionConfig c;
    const json* head = object(*root, "head", err);
    const json* shake = head ? object(*root, "speed_shake", err) : nullptr;
    const json* fov = shake ? object(*root, "speed_fov", err) : nullptr;
    const json* views = fov ? object(*root, "views", err) : nullptr;
    if (!views) return std::nullopt;
    bool ok = read_triple(*head, "head", "frequency_hz", 0.1, 20.0, c.frequency_hz, err) &&
              read_triple(*head, "head", "damping_ratio", 0.05, 4.0, c.damping_ratio, err) &&
              read_triple(*head, "head", "deflection_m_per_mps2", 0.0, 0.05, c.deflection_m_per_mps2, err) &&
              read_triple(*head, "head", "limit_m", 0.001, 0.5, c.limit_m, err) &&
              read_num(*head, "head", "pitch_deg_per_m", 0.0, 500.0, c.pitch_deg_per_m, err) &&
              read_num(*head, "head", "roll_deg_per_m", 0.0, 500.0, c.roll_deg_per_m, err) &&
              read_num(*head, "head", "rotation_limit_deg", 0.0, 30.0, c.rotation_limit_deg, err) &&
              read_num(*head, "head", "accel_filter_hz", 0.1, 100.0, c.accel_filter_hz, err) &&
              read_num(*head, "head", "max_accel_mps2", 1.0, 1000.0, c.max_accel_mps2, err) &&
              read_num(*shake, "speed_shake", "start_speed_mps", 0.0, 500.0, c.shake_start_speed_mps, err) &&
              read_num(*shake, "speed_shake", "full_speed_mps", 0.0, 500.0, c.shake_full_speed_mps, err) &&
              read_num(*shake, "speed_shake", "amplitude_m", 0.0, 0.1, c.shake_amplitude_m, err) &&
              read_num(*shake, "speed_shake", "roughness_gain_s_per_m", 0.0, 100.0, c.shake_roughness_gain_s_per_m, err) &&
              read_num(*shake, "speed_shake", "roughness_filter_hz", 0.1, 100.0, c.shake_roughness_filter_hz, err) &&
              read_num(*shake, "speed_shake", "max_amplitude_m", 0.0, 0.1, c.shake_max_amplitude_m, err) &&
              read_num(*fov, "speed_fov", "start_speed_mps", 0.0, 500.0, c.fov_start_speed_mps, err) &&
              read_num(*fov, "speed_fov", "full_speed_mps", 0.0, 500.0, c.fov_full_speed_mps, err) &&
              read_num(*fov, "speed_fov", "max_extra_deg", 0.0, 40.0, c.fov_max_extra_deg, err);
    if (!ok) return std::nullopt;
    if (c.shake_full_speed_mps <= c.shake_start_speed_mps || c.fov_full_speed_mps <= c.fov_start_speed_mps) {
        fail(err, "full_speed_mps must be above start_speed_mps");
        return std::nullopt;
    }
    const auto freqs = shake->find("frequencies_hz");
    if (freqs == shake->end() || !freqs->is_array() || freqs->empty() || freqs->size() > 8) {
        fail(err, "speed_shake.frequencies_hz: expected 1..8 numbers");
        return std::nullopt;
    }
    c.shake_frequencies_hz.clear();
    for (const auto& f : *freqs) {
        if (!f.is_number() || f.get<double>() <= 0.0 || f.get<double>() > 200.0) {
            fail(err, "speed_shake.frequencies_hz: out of range");
            return std::nullopt;
        }
        c.shake_frequencies_hz.push_back(f.get<double>());
    }
    for (const char* name : {"chase", "bumper", "cockpit", "xr_cockpit"}) {
        const json* v = object(*views, name, err);
        if (!v) return std::nullopt;
        CameraViewGains g;
        const std::string sec = std::string("views.") + name;
        if (!read_num(*v, sec.c_str(), "head_translation", 0.0, 2.0, g.head_translation, err) ||
            !read_num(*v, sec.c_str(), "head_rotation", 0.0, 2.0, g.head_rotation, err) ||
            !read_num(*v, sec.c_str(), "shake", 0.0, 2.0, g.shake, err) ||
            !read_num(*v, sec.c_str(), "fov", 0.0, 2.0, g.fov, err))
            return std::nullopt;
        c.views[name] = g;
    }
    const CameraViewGains& xr = c.views["xr_cockpit"];
    if (xr.head_rotation != 0.0 || xr.shake != 0.0 || xr.fov != 0.0) {
        fail(err, "views.xr_cockpit: head_rotation, shake and fov must be 0 (VR comfort)");
        return std::nullopt;
    }
    return c;
}

CameraMotion::CameraMotion(CameraMotionConfig config) : config_(std::move(config)) {}

void CameraMotion::reset() {
    head_ = {};
    x_ = {};
    v_ = {};
    accel_world_ = {};
    prev_velocity_ = {};
    prev_compression_.clear();
    prev_sim_time_ = 0.0;
    have_prev_ = false;
    // clock_s_ keeps running: the shake phase is not part of the head state
}

const HeadState& CameraMotion::update(const CarMotionSample& s, double dt) {
    if (!(dt > 0.0) || !std::isfinite(dt)) return head_;
    dt = std::min(dt, 0.1);
    clock_s_ += dt;
    head_.speed_mps = s.velocity.length();

    // New simulation tick: acceleration and suspension velocity estimates.
    if (have_prev_ && s.sim_time != prev_sim_time_) {
        const double dts = s.sim_time - prev_sim_time_;
        if (dts > 0.0 && dts <= kMaxSimGapS) {
            const ps::Vec3 a = (s.velocity - prev_velocity_) / dts;
            if (a.length() > config_.max_accel_mps2) {
                // a relocation / reset: not a force the driver feels
                accel_world_ = {};
                x_ = {};
                v_ = {};
            } else {
                accel_world_ += (a - accel_world_) * lowpass_alpha(config_.accel_filter_hz, dts);
            }
            double rough = 0.0;
            if (prev_compression_.size() == s.wheel_compression.size() && !s.wheel_compression.empty()) {
                for (size_t i = 0; i < s.wheel_compression.size(); ++i)
                    rough += std::fabs(s.wheel_compression[i] - prev_compression_[i]) / dts;
                rough /= double(s.wheel_compression.size());
            }
            head_.roughness_mps += (rough - head_.roughness_mps) * lowpass_alpha(config_.shake_roughness_filter_hz, dts);
        } else {
            accel_world_ = {};
        }
    }
    if (!have_prev_ || s.sim_time != prev_sim_time_) {
        prev_velocity_ = s.velocity;
        prev_compression_ = s.wheel_compression;
        prev_sim_time_ = s.sim_time;
        have_prev_ = true;
    }

    const ps::Vec3 a_car{ps::dot(accel_world_, s.forward), ps::dot(accel_world_, s.left), ps::dot(accel_world_, s.up)};
    head_.accel_car = a_car;
    const int n = std::max(1, int(std::ceil(dt / kMaxSubstepS)));
    const double h = dt / n;
    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < 3; ++i) {
            const double w = 2.0 * kPi * config_.frequency_hz[i];
            const double acc = -w * w * x_[i] - 2.0 * config_.damping_ratio[i] * w * v_[i] -
                               w * w * config_.deflection_m_per_mps2[i] * a_car[i];
            v_[i] += acc * h;
            x_[i] += v_[i] * h;
        }
    }
    double off[3];
    for (int i = 0; i < 3; ++i) off[i] = config_.limit_m[i] * std::tanh(x_[i] / config_.limit_m[i]);
    head_.offset = ps::Vec3{off[0], off[1], off[2]};
    const double lim = config_.rotation_limit_deg * kPi / 180.0;
    head_.pitch_down_rad = std::clamp(config_.pitch_deg_per_m * off[0] * kPi / 180.0, -lim, lim);
    head_.roll_right_rad = std::clamp(-config_.roll_deg_per_m * off[1] * kPi / 180.0, -lim, lim);
    return head_;
}

ViewMotion CameraMotion::view(const CameraViewGains& g, double strength) const {
    strength = std::clamp(std::isfinite(strength) ? strength : 0.0, 0.0, 1.0);
    ViewMotion m;
    m.offset = head_.offset * (g.head_translation * strength);
    m.pitch_down_rad = head_.pitch_down_rad * g.head_rotation * strength;
    m.roll_right_rad = head_.roll_right_rad * g.head_rotation * strength;
    if (g.shake > 0.0) {
        double amp = config_.shake_amplitude_m *
                     ramp(head_.speed_mps, config_.shake_start_speed_mps, config_.shake_full_speed_mps) *
                     (1.0 + config_.shake_roughness_gain_s_per_m * head_.roughness_mps);
        amp = std::min(amp, config_.shake_max_amplitude_m) * g.shake;
        double sy = 0.0, sz = 0.0;
        const double norm = 1.0 / double(config_.shake_frequencies_hz.size());
        for (size_t i = 0; i < config_.shake_frequencies_hz.size(); ++i) {
            const double ph = 2.0 * kPi * config_.shake_frequencies_hz[i] * clock_s_;
            sy += std::sin(ph + 1.3 * double(i)) * norm;
            sz += std::sin(1.17 * ph + 0.7 * double(i)) * norm;
        }
        m.offset += ps::Vec3{0.0, amp * sy, amp * sz};
    }
    m.fov_extra_deg = config_.fov_max_extra_deg * g.fov *
                      ramp(head_.speed_mps, config_.fov_start_speed_mps, config_.fov_full_speed_mps);
    return m;
}

ViewMotion CameraMotion::view(const std::string& view_name, double strength) const {
    const auto it = config_.views.find(view_name);
    if (it == config_.views.end()) return {};
    return view(it->second, strength);
}

} // namespace rg
