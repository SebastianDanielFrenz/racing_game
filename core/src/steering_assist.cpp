// steering_assist.cpp - see rg/steering_assist.h. A line-for-line port of the former
// game/scripts/adaptive_steering.gd (same clamps, same Godot smoothstep / move_toward semantics).
#include "rg/steering_assist.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rg {

namespace {

constexpr double kPi = 3.14159265358979323846;

double deg_to_rad(double d) { return d * kPi / 180.0; }

// Godot's smoothstep(from, to, x).
double smoothstep(double from, double to, double x) {
    if (std::fabs(from - to) < 1e-12) return x < from ? 0.0 : 1.0;
    const double s = std::clamp((x - from) / (to - from), 0.0, 1.0);
    return s * s * (3.0 - 2.0 * s);
}

// Godot's move_toward(from, to, delta).
double move_toward(double from, double to, double delta) {
    if (std::fabs(to - from) <= delta) return to;
    return from + (to > from ? delta : -delta);
}

double lerp(double a, double b, double t) { return a + (b - a) * t; }

double sign(double v) { return v > 0.0 ? 1.0 : (v < 0.0 ? -1.0 : 0.0); }

} // namespace

std::optional<SteeringAssistConfig> load_steering_assist_config(const std::string& path, std::string* err) {
    const auto fail = [err](const std::string& m) -> std::optional<SteeringAssistConfig> {
        if (err) *err = m;
        return std::nullopt;
    };
    const auto text = detail::read_text_file(path);
    if (!text) return fail("steering config: cannot read " + path);
    const auto j = detail::parse_json(*text);
    if (!j || !j->is_object()) return fail("steering config: " + path + " is not a JSON object");
    SteeringAssistConfig c;
    if (!detail::get_bool(*j, "enabled", c.enabled)) return fail("steering config: 'enabled' must be a bool");
    if (!detail::get_string(*j, "input_profile", c.input_profile))
        return fail("steering config: 'input_profile' must be a string");
    struct Field {
        const char* key;
        double* out;
    };
    const Field fields[] = {
        {"normal_min_fraction", &c.normal_min_fraction},
        {"normal_lateral_accel_mps2", &c.normal_lateral_accel_mps2},
        {"input_exponent", &c.input_exponent},
        {"keyboard_rise_per_s", &c.keyboard_rise_per_s},
        {"keyboard_return_per_s", &c.keyboard_return_per_s},
        {"recovery_keyboard_per_s", &c.recovery_keyboard_per_s},
        {"slip_start_deg", &c.slip_start_deg},
        {"slip_full_deg", &c.slip_full_deg},
        {"yaw_anticipation_s", &c.yaw_anticipation_s},
        {"recovery_rise_per_s", &c.recovery_rise_per_s},
        {"recovery_fall_per_s", &c.recovery_fall_per_s},
    };
    for (const Field& f : fields) {
        if (!detail::get_number(*j, f.key, *f.out) || !std::isfinite(*f.out))
            return fail(std::string("steering config: '") + f.key + "' must be a finite number");
    }
    return c;
}

std::optional<SteeringGeometry> steering_geometry_from_vehicle_json(const std::string& path) {
    const auto text = detail::read_text_file(path);
    if (!text) return std::nullopt;
    const auto j = detail::parse_json(*text);
    if (!j || !j->is_object()) return std::nullopt;
    SteeringGeometry g;
    const auto steering = j->find("steering");
    if (steering != j->end() && steering->is_object()) {
        double angle = 0.0;
        if (detail::get_number(*steering, "max_wheel_angle", angle) && std::isfinite(angle))
            g.max_wheel_angle_rad = std::max(0.01, angle);
    }
    double front = -std::numeric_limits<double>::infinity();
    double rear = std::numeric_limits<double>::infinity();
    const auto wheels = j->find("wheels");
    if (wheels != j->end() && wheels->is_array()) {
        for (const auto& wheel : *wheels) {
            double x = 0.0;
            if (wheel.is_object()) {
                const auto a = wheel.find("attachment_local");
                if (a != wheel.end() && a->is_array() && !a->empty() && (*a)[0].is_number()) x = (*a)[0].get<double>();
            }
            front = std::max(front, x);
            rear = std::min(rear, x);
        }
    }
    if (std::isfinite(front) && std::isfinite(rear)) g.wheelbase_m = std::max(0.1, front - rear);
    return g;
}

void SteeringAssist::set_geometry(const SteeringGeometry& geometry) {
    geometry_ = geometry;
    geometry_.max_wheel_angle_rad = std::max(0.01, geometry_.max_wheel_angle_rad);
    geometry_.wheelbase_m = std::max(0.1, geometry_.wheelbase_m);
}

void SteeringAssist::reset() {
    keyboard_ = 0.0;
    recovery_ = 0.0;
    relocations_ = -1;
}

double SteeringAssist::translate(double raw, bool keyboard, bool wheel, const SteeringMotion& motion, double dt) {
    raw = std::isfinite(raw) ? std::clamp(raw, -1.0, 1.0) : 0.0;
    const SteeringAssistConfig& c = config_;
    const std::string& profile = c.input_profile;
    if (!c.enabled || (!keyboard && (profile == "wheel" || (profile == "auto" && wheel)))) {
        reset();
        return raw;
    }
    if (relocations_ != motion.relocations) {
        reset();
        relocations_ = motion.relocations;
    }
    const double forward = motion.forward_mps;
    const double slip = std::atan2(motion.left_mps, std::max(std::fabs(forward), 1.0));
    const double correction = slip - motion.yaw_rad_s * std::clamp(c.yaw_anticipation_s, 0.0, 0.5);
    const bool countersteering = raw * correction > 0.0 && forward > 5.0 && motion.upright > 0.5;
    const double start = std::max(0.1, c.slip_start_deg);
    const double full = std::max(start + 0.1, c.slip_full_deg);
    const double demand =
        countersteering ? smoothstep(deg_to_rad(start), deg_to_rad(full), std::fabs(slip)) : 0.0;
    const double step_dt = std::isfinite(dt) ? std::clamp(dt, 0.0, 0.05) : 0.0;
    const double recovery_rate = std::max(0.1, demand > recovery_ ? c.recovery_rise_per_s : c.recovery_fall_per_s);
    recovery_ = move_toward(recovery_, demand, recovery_rate * step_dt);
    // Never retain increased authority for input that steers into the slide.
    double authority = countersteering ? recovery_ : 0.0;
    double input_value = raw;
    if (keyboard) {
        const double rise = std::max(0.1, c.keyboard_rise_per_s);
        const double fast = std::max(rise, c.recovery_keyboard_per_s);
        double rate = lerp(rise, fast, authority);
        if (std::fabs(raw) < 1e-5) rate = std::max(0.1, c.keyboard_return_per_s);
        keyboard_ = move_toward(keyboard_, raw, rate * step_dt);
        input_value = keyboard_;
    } else {
        keyboard_ = 0.0;
    }
    if (input_value * correction <= 0.0) authority = 0.0;
    const double accel = std::max(0.1, c.normal_lateral_accel_mps2);
    const double floor_fraction = std::clamp(c.normal_min_fraction, 0.01, 0.95);
    const double normal_limit = std::clamp(
        std::atan(geometry_.wheelbase_m * accel / std::max(forward * forward, 1.0)) / geometry_.max_wheel_angle_rad,
        floor_fraction, 1.0);
    const double exponent = lerp(std::clamp(c.input_exponent, 1.0, 4.0), 1.0, authority);
    const double shaped = sign(input_value) * std::pow(std::fabs(input_value), exponent);
    return std::clamp(shaped * lerp(normal_limit, 1.0, authority), -1.0, 1.0);
}

} // namespace rg
