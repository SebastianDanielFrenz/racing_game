#include "rg/fpv_drone.h"

#include "json_util.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>

namespace rg {

namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp1(double v) {
    if (!std::isfinite(v)) return 0.0;
    return std::clamp(v, -1.0, 1.0);
}

bool read_positive(const detail::json& o, const char* key, double& out, std::string* err) {
    if (!detail::get_number(o, key, out) || !std::isfinite(out) || !(out > 0.0)) {
        if (err) *err = std::string("fpv_drone: '") + key + "' must be a positive number";
        return false;
    }
    return true;
}

} // namespace

std::optional<FpvDroneConfig> load_fpv_drone_config(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        if (err) *err = "fpv_drone: cannot read " + path;
        return std::nullopt;
    }
    const auto root = detail::parse_json(*text);
    if (!root || !root->is_object()) {
        if (err) *err = "fpv_drone: " + path + " is not a JSON object";
        return std::nullopt;
    }
    std::string format;
    if (!detail::get_string(*root, "format", format) || format != "rg.fpv_drone/1") {
        if (err) *err = "fpv_drone: format must be \"rg.fpv_drone/1\"";
        return std::nullopt;
    }
    const auto it = root->find("flight");
    if (it == root->end() || !it->is_object()) {
        if (err) *err = "fpv_drone: missing 'flight' object";
        return std::nullopt;
    }
    FpvDroneConfig c;
    if (!read_positive(*it, "max_speed_mps", c.max_speed_mps, err) ||
        !read_positive(*it, "max_vertical_speed_mps", c.max_vertical_speed_mps, err) ||
        !read_positive(*it, "accel_mps2", c.accel_mps2, err) || !read_positive(*it, "decel_mps2", c.decel_mps2, err) ||
        !read_positive(*it, "yaw_rate_rad_s", c.yaw_rate_rad_s, err) ||
        !read_positive(*it, "pitch_rate_rad_s", c.pitch_rate_rad_s, err) ||
        !read_positive(*it, "pitch_limit_deg", c.pitch_limit_deg, err) ||
        !read_positive(*it, "rate_response_per_s", c.rate_response_per_s, err) ||
        !read_positive(*it, "ground_clearance_m", c.ground_clearance_m, err) ||
        !read_positive(*it, "spawn_height_above_car_m", c.spawn_height_above_car_m, err) ||
        !read_positive(*it, "ground_probe_up_m", c.ground_probe_up_m, err)) {
        return std::nullopt;
    }
    if (c.pitch_limit_deg >= 90.0) {
        if (err) *err = "fpv_drone: 'pitch_limit_deg' must be below 90";
        return std::nullopt;
    }
    return c;
}

void FpvDrone::spawn(const ps::Vec3& position, double yaw_rad) {
    state_ = FpvDroneState{};
    state_.active = true;
    state_.position = position;
    state_.yaw_rad = yaw_rad;
    state_.hold = true;
}

void FpvDrone::despawn() { state_ = FpvDroneState{}; }

void FpvDrone::teleport(const ps::Vec3& position) {
    if (!state_.active) return;
    state_.position = position;
    halt();
}

void FpvDrone::halt() {
    state_.velocity = ps::Vec3{};
    state_.yaw_rate_rad_s = 0.0;
    state_.pitch_rate_rad_s = 0.0;
    state_.speed_mps = 0.0;
    state_.accel_mps2 = 0.0;
    state_.activity = 0.0;
}

const FpvDroneState& FpvDrone::step(const FpvDroneInput& in, double dt, const FpvGroundProbe& ground) {
    if (!state_.active || !(dt > 0.0) || !std::isfinite(dt)) return state_;
    const FpvDroneConfig& c = config_;

    // --- turning: yaw/pitch rates follow the input with an exponential response, no roll anywhere.
    const double follow = 1.0 - std::exp(-c.rate_response_per_s * dt);
    state_.yaw_rate_rad_s += (clamp1(in.yaw) * c.yaw_rate_rad_s - state_.yaw_rate_rad_s) * follow;
    state_.pitch_rate_rad_s += (clamp1(in.pitch) * c.pitch_rate_rad_s - state_.pitch_rate_rad_s) * follow;
    state_.yaw_rad += state_.yaw_rate_rad_s * dt;
    state_.yaw_rad = std::remainder(state_.yaw_rad, 2.0 * kPi);
    const double pitch_limit = c.pitch_limit_deg * kPi / 180.0;
    state_.pitch_rad = std::clamp(state_.pitch_rad + state_.pitch_rate_rad_s * dt, -pitch_limit, pitch_limit);
    if (state_.pitch_rad == pitch_limit || state_.pitch_rad == -pitch_limit) state_.pitch_rate_rad_s = 0.0;

    // --- wanted velocity in the yaw frame.
    double s = 0.0, co = 1.0;
    ps::math::sincos(state_.yaw_rad, s, co);
    const ps::Vec3 fwd{co, s, 0.0};
    const ps::Vec3 right{s, -co, 0.0};
    double mr = clamp1(in.move_right), mf = clamp1(in.move_forward);
    const double mh = std::sqrt(mr * mr + mf * mf);
    if (mh > 1.0) {
        mr /= mh;
        mf /= mh;
    }
    ps::Vec3 wanted = right * (mr * c.max_speed_mps) + fwd * (mf * c.max_speed_mps);
    wanted.z = clamp1(in.move_up) * c.max_vertical_speed_mps;

    // --- accelerate toward it with the acceleration / deceleration limit (a per-tick velocity change cap).
    const ps::Vec3 old_velocity = state_.velocity;
    const ps::Vec3 dv = wanted - old_velocity;
    const double dvl = dv.length();
    const double limit = (wanted.length() >= old_velocity.length() ? c.accel_mps2 : c.decel_mps2) * dt;
    ps::Vec3 v = old_velocity;
    if (dvl <= limit) v = wanted;
    else if (dvl > 0.0) v = old_velocity + dv * (limit / dvl);

    // --- hard speed limits (a wanted velocity is within them, so this only matters after a teleport or a bad config).
    const double hs = std::sqrt(v.x * v.x + v.y * v.y);
    if (hs > c.max_speed_mps) {
        v.x *= c.max_speed_mps / hs;
        v.y *= c.max_speed_mps / hs;
    }
    v.z = std::clamp(v.z, -c.max_vertical_speed_mps, c.max_vertical_speed_mps);

    // --- move, then collide with the ground.
    const ps::Vec3 old_position = state_.position;
    ps::Vec3 p = old_position + v * dt;
    state_.on_ground = false;
    const double z_top = std::max(old_position.z, p.z) + c.ground_probe_up_m;
    const std::optional<double> g = ground ? ground(p.x, p.y, z_top) : std::nullopt;
    if (g && std::isfinite(*g)) {
        state_.hold = false;
        state_.ground_z = *g;
        const double min_z = *g + c.ground_clearance_m;
        if (p.z <= min_z) {
            p.z = min_z;
            if (v.z < 0.0) v.z = 0.0;
            state_.on_ground = true;
        }
        state_.height_agl_m = p.z - *g;
    } else {
        // No ground loaded below: keep the old height (and no vertical motion), still move sideways.
        state_.hold = true;
        p.z = old_position.z;
        v.z = 0.0;
        state_.height_agl_m = 0.0;
    }
    state_.position = p;
    state_.velocity = v;
    state_.speed_mps = v.length();

    const ps::Vec3 applied = v - old_velocity;
    state_.accel_mps2 = applied.length() / dt;
    const double peak = std::max(c.accel_mps2, c.decel_mps2);
    const double lin = std::min(state_.accel_mps2 / peak, 1.0);
    const double turn = std::max(std::fabs(state_.yaw_rate_rad_s) / c.yaw_rate_rad_s,
                                 std::fabs(state_.pitch_rate_rad_s) / c.pitch_rate_rad_s);
    state_.activity = std::min(std::max(lin, 0.5 * std::min(turn, 1.0)), 1.0);
    return state_;
}

} // namespace rg
