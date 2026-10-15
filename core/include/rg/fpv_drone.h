// rg/fpv_drone.h - the pilotable FPV drone (PlayerMode::FpvDrone). Engine-neutral and deliberately simple, as the owner
// asked: a KINEMATIC flyer, not a physics body. Six translation axes in the drone's yaw frame (right/left, up/down,
// forward/back), a yaw rate and a pitch rate, a maximum speed, acceleration and deceleration limits, ground
// collision. No roll, no wind, no stabilisation logic (pitch only tilts the camera/body, it does not tilt the thrust).
//
// Frame: the session frame like everything in rg_core - ISO, x east, y north, z up; yaw 0 = east, counter-clockwise
// positive (the vehicle's +x axis rotated about +z, as WorldConfig::spawn); pitch positive = nose up.
//
// Ground: the caller provides FpvGroundProbe (the Session casts a ray down through the World; tests use a function).
// A probe that finds nothing means "no ground loaded": the drone holds its height (like WalkerController::hold)
// but still moves horizontally, so streaming terrain can catch up. Only terrain/bodies below are collided with;
// there is no side collision with buildings.
#pragma once

#include "ps/math/vec3.h"

#include <functional>
#include <optional>
#include <string>

namespace rg {

struct FpvDroneConfig {
    double max_speed_mps = 25.0;           // horizontal speed limit
    double max_vertical_speed_mps = 12.0;
    double accel_mps2 = 16.0;              // speeding up (|wanted velocity| >= |velocity|)
    double decel_mps2 = 22.0;              // slowing down or reversing
    double yaw_rate_rad_s = 2.2;           // at full input
    double pitch_rate_rad_s = 1.5;
    double pitch_limit_deg = 80.0;
    double rate_response_per_s = 14.0;     // yaw/pitch rate follows the input with this exponential rate
    double ground_clearance_m = 0.35;      // the drone's centre never goes below ground + this
    double spawn_height_above_car_m = 2.2; // above the chassis origin
    double ground_probe_up_m = 1.0;        // the probe starts this far above the highest of old/new height
};

// Loads the "flight" block of data/controls/fpv_drone.json (rg.fpv_drone/1). Every field is required; nullopt + *err
// on a missing file, a syntax error, a missing key, a wrong type or an out-of-range value.
std::optional<FpvDroneConfig> load_fpv_drone_config(const std::string& path, std::string* err);

// Per-tick input, every axis in [-1, 1] (clamped; the horizontal magnitude is limited to 1, so a diagonal is not faster).
struct FpvDroneInput {
    double move_right = 0.0;    // + = right of the drone's heading
    double move_up = 0.0;       // + = up
    double move_forward = 0.0;  // + = along the heading (horizontal, whatever the pitch)
    double yaw = 0.0;           // + = turn left (counter-clockwise from above)
    double pitch = 0.0;         // + = nose up
};

struct FpvDroneState {
    bool active = false;
    ps::Vec3 position{};        // the drone's centre, session metres
    ps::Vec3 velocity{};        // world frame
    double yaw_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rate_rad_s = 0.0;
    double pitch_rate_rad_s = 0.0;
    double speed_mps = 0.0;
    double accel_mps2 = 0.0;    // magnitude of this tick's velocity change / dt
    double activity = 0.0;      // 0..1: how hard the rotors work this tick (acceleration, deceleration, turning)
    bool on_ground = false;     // resting on / pressed against the ground this tick
    bool hold = false;          // no ground below: the height is held
    double ground_z = 0.0;      // valid when !hold
    double height_agl_m = 0.0;  // valid when !hold
};

// Ground height below (x, y), looking down from z_top; nullopt = nothing found (no terrain loaded).
using FpvGroundProbe = std::function<std::optional<double>(double x, double y, double z_top)>;

class FpvDrone {
public:
    explicit FpvDrone(const FpvDroneConfig& config = {}) : config_(config) {}

    void spawn(const ps::Vec3& position, double yaw_rad);
    void despawn();
    [[nodiscard]] bool active() const { return state_.active; }
    [[nodiscard]] const FpvDroneState& state() const { return state_; }
    [[nodiscard]] const FpvDroneConfig& config() const { return config_; }

    // Place the drone (a relocation); velocity and rates reset.
    void teleport(const ps::Vec3& position);
    // Zero velocity and turn rates (the World was stepped under the drone, e.g. priming ticks).
    void halt();

    // One tick of dt seconds. Does nothing (returns the state) while not active.
    const FpvDroneState& step(const FpvDroneInput& input, double dt, const FpvGroundProbe& ground);

private:
    FpvDroneConfig config_;
    FpvDroneState state_;
};

} // namespace rg
