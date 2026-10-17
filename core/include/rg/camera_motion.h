// rg/camera_motion.h - the driver's virtual body and the speed effects of the
// drive cameras. Engine-neutral; the rigs only add the returned offset/angles.
//
// Head: per car axis (ISO x forward, y left, z up) a damped spring-mass on the
// seat, driven by the chassis acceleration in the car frame (no gravity: a
// constant velocity or standing on a slope gives no offset):
//   x'' = -w^2 x - 2 zeta w x' - w^2 d a
// so the static deflection is -d * a: braking (a.x < 0) moves the eye forward
// toward the wheel, a left turn (a.y > 0) moves it right, a bump (a.z > 0)
// pushes it down. The deflection is soft-limited (L tanh(x / L)); pitch/roll
// follow it (nodding forward under braking, leaning out in a corner).
//
// Speed effects (never in VR): a translation-only high-frequency shake that
// rises with speed and road roughness (suspension velocity), and an FOV
// widening with speed.
//
// Numbers: data/controls/camera_motion.json (rg.camera_motion/1), one gain set
// per view; the user setting camera.body_motion scales the head motion.
// Owner rule (PR #5): head motion only in the cockpit views; the VR view never
// gets a rotation it did not make (xr_cockpit rotation, shake and FOV are
// forced to zero by the loader).
#pragma once

#include "ps/math/vec3.h"

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace rg {

struct CameraViewGains {
    double head_translation = 0.0;
    double head_rotation = 0.0;
    double shake = 0.0;
    double fov = 0.0;
};

struct CameraMotionConfig {
    std::array<double, 3> frequency_hz{1.4, 1.2, 2.2};
    std::array<double, 3> damping_ratio{0.55, 0.5, 0.35};
    std::array<double, 3> deflection_m_per_mps2{0.006, 0.005, 0.003};
    std::array<double, 3> limit_m{0.08, 0.07, 0.04};
    double pitch_deg_per_m = 40.0;
    double roll_deg_per_m = 40.0;
    double rotation_limit_deg = 3.0;
    double accel_filter_hz = 8.0;
    double max_accel_mps2 = 80.0; // a larger velocity jump is a relocation, not a force
    double shake_start_speed_mps = 15.0;
    double shake_full_speed_mps = 100.0;
    double shake_amplitude_m = 0.003;
    double shake_roughness_gain_s_per_m = 4.0;
    double shake_roughness_filter_hz = 4.0;
    double shake_max_amplitude_m = 0.012;
    std::vector<double> shake_frequencies_hz{11.3, 17.9, 23.7};
    double fov_start_speed_mps = 20.0;
    double fov_full_speed_mps = 110.0;
    double fov_max_extra_deg = 8.0;
    std::map<std::string, CameraViewGains> views; // "chase", "bumper", "cockpit", "xr_cockpit"
};

// Strict loader; nullopt + *err on any missing/invalid field. Forces the
// xr_cockpit rotation/shake/FOV gains to be zero (a non-zero value is an error).
std::optional<CameraMotionConfig> load_camera_motion_config(const std::string& path, std::string* err);

// One rendered frame's view of the car (session frame, any consistent world frame).
struct CarMotionSample {
    double sim_time = 0.0;  // advances only when the simulation published a new tick
    ps::Vec3 velocity{};    // world, m/s
    ps::Vec3 forward{1.0, 0.0, 0.0}, left{0.0, 1.0, 0.0}, up{0.0, 0.0, 1.0}; // car axes in the world
    std::vector<double> wheel_compression; // m, any count
};

struct HeadState {
    ps::Vec3 offset{};         // eye offset in the car frame (x forward, y left, z up), m
    double pitch_down_rad = 0.0; // + = the view nods down
    double roll_right_rad = 0.0; // + = the head leans to the right
    double speed_mps = 0.0;
    double roughness_mps = 0.0; // filtered mean |suspension velocity|
    ps::Vec3 accel_car{};       // filtered acceleration in the car frame (diagnostic)
};

struct ViewMotion {
    ps::Vec3 offset{};           // car frame: head offset + shake
    double pitch_down_rad = 0.0;
    double roll_right_rad = 0.0;
    double fov_extra_deg = 0.0;
};

class CameraMotion {
public:
    explicit CameraMotion(CameraMotionConfig config = {});
    void reset();
    // dt = render frame time. Deterministic for the same input sequence.
    const HeadState& update(const CarMotionSample& sample, double dt);
    const HeadState& state() const { return head_; }
    // The motion one view shows; strength = the camera.body_motion setting (scales the head only).
    ViewMotion view(const CameraViewGains& gains, double strength) const;
    ViewMotion view(const std::string& view_name, double strength) const;
    const CameraMotionConfig& config() const { return config_; }

private:
    CameraMotionConfig config_;
    HeadState head_;
    std::array<double, 3> x_{}, v_{};
    ps::Vec3 accel_world_{};
    ps::Vec3 prev_velocity_{};
    std::vector<double> prev_compression_;
    double prev_sim_time_ = 0.0;
    bool have_prev_ = false;
    double clock_s_ = 0.0;
};

} // namespace rg
