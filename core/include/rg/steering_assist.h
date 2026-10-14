// rg/steering_assist.h - the adaptive steering input translation (data/controls/steering.json), moved out of
// game/scripts/adaptive_steering.gd (Unreal port prep, 2026-10-14) so every engine front end shapes the driver's steer
// input the same way.
//
// Input translation only: it never generates steering without driver input. Per frame it takes the raw steer level
// (-1..1, ISO sign: + = left), whether a keyboard / a wheel produced it and the chassis motion, and returns the value
// for the "steer" control channel:
//  - a keyboard level is rate limited (rise / return rates; faster while counter-steering a slide);
//  - the input is shaped by an exponent and limited to the wheel angle that gives `normal_lateral_accel_mps2` at the
//    current speed (never below `normal_min_fraction` of full lock);
//  - while the driver counter-steers a slide (slip angle beyond slip_start_deg, speed > 5 m/s, upright) the limit and
//    the exponent relax toward full authority (recovery_rise / recovery_fall rates).
// A wheel (input_profile "wheel", or "auto" with a wheel connected) and enabled = false pass the raw level through.
#pragma once

#include <optional>
#include <string>

namespace rg {

struct SteeringAssistConfig {
    bool enabled = true;
    std::string input_profile = "auto"; // "auto" | "wheel" | "keyboard" (any other value behaves like "keyboard")
    double normal_min_fraction = 0.14;
    double normal_lateral_accel_mps2 = 18.0;
    double input_exponent = 1.8;
    double keyboard_rise_per_s = 4.0;
    double keyboard_return_per_s = 8.0;
    double recovery_keyboard_per_s = 18.0;
    double slip_start_deg = 3.0;
    double slip_full_deg = 20.0;
    double yaw_anticipation_s = 0.15;
    double recovery_rise_per_s = 12.0;
    double recovery_fall_per_s = 4.0;
};

// Loads data/controls/steering.json. Every field is required (no silent defaults); std::nullopt + *err on a missing
// file, a syntax error, a missing key or a wrong type.
std::optional<SteeringAssistConfig> load_steering_assist_config(const std::string& path, std::string* err);

// The car's steering geometry: wheelbase (front-most minus rear-most wheel attachment x) and the road-wheel angle at
// full lock.
struct SteeringGeometry {
    double wheelbase_m = 2.7;
    double max_wheel_angle_rad = 0.5585053606381855; // 32 deg
};

// Reads a physics_sim.vehicle file (steering.max_wheel_angle, wheels[].attachment_local[0]); fields the file lacks keep
// the SteeringGeometry defaults. std::nullopt when the file cannot be read or parsed.
std::optional<SteeringGeometry> steering_geometry_from_vehicle_json(const std::string& path);

// Chassis motion in the chassis frame (ISO: x forward, y left, z up).
struct SteeringMotion {
    double forward_mps = 0.0;
    double left_mps = 0.0;
    double yaw_rad_s = 0.0;
    double upright = 1.0;           // chassis up . world up
    long long relocations = 0;      // Session relocation counter: a change resets the filter state
};

class SteeringAssist {
public:
    SteeringAssist() = default;
    explicit SteeringAssist(const SteeringAssistConfig& config) : config_(config) {}

    void set_config(const SteeringAssistConfig& config) { config_ = config; }
    [[nodiscard]] const SteeringAssistConfig& config() const { return config_; }
    void set_geometry(const SteeringGeometry& geometry);
    [[nodiscard]] const SteeringGeometry& geometry() const { return geometry_; }

    void reset();
    // raw: the device steer level; keyboard: a keyboard produced it; wheel: a steering wheel is the steer source;
    // dt: frame time in seconds (clamped to 0..0.05). Returns the steer channel value, -1..1.
    double translate(double raw, bool keyboard, bool wheel, const SteeringMotion& motion, double dt);

    [[nodiscard]] double keyboard_level() const { return keyboard_; }
    [[nodiscard]] double recovery() const { return recovery_; }

private:
    SteeringAssistConfig config_;
    SteeringGeometry geometry_;
    double keyboard_ = 0.0;
    double recovery_ = 0.0;
    long long relocations_ = -1;
};

} // namespace rg
