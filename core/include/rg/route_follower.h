// rg/route_follower.h — rg::RouteFollower: a deterministic route-following driver
// (G3/R3 slice S2, docs/g3_consumer_design.md sections 6.1 and 8): pure pursuit on a
// route polyline plus a speed PI, engine-neutral and Godot-free. It is the "driver" the
// R3 acceptance harness (S5) puts on the home routes, and a free-standing autopilot for
// anything else that needs a car to follow a polyline.
//
// Pieces:
//  - RouteFollower: the pure controller. step(state, dt) -> command. No World, no wall
//    clock, no randomness, no libm transcendental except ps::math::atan2 (PHYS D7): the
//    same inputs give the same commands on every platform and at any worker count.
//  - make_route_follower_controller(): the rg::DriveScript::Controller adapter. It reads
//    the chassis pose/motion from the World right before World::step() (that is the state
//    the last published snapshot carries - the controller runs on the stepping thread, so
//    it needs no cross-thread hand-off), sets the driving assists once, and writes
//    steer/throttle/brake every tick.
//  - follower_params_for_vehicle(): wheelbase, rear-axle offset and the steering limit
//    read from a ps::vehicle::VehicleDesc, never hand-copied.
//
// Steering. Reference point = the REAR AXLE (pure pursuit's kinematic reference).
// Look-ahead = clamp(lookahead_gain_s * v, lookahead_min_m, lookahead_max_m) metres of
// route arc length ahead of the projection of the rear axle onto the route. The wheel
// angle is atan2(2 L sin(alpha), d) with alpha the bearing of the look-ahead point
// relative to the heading and d the chord to it (the exact bicycle-model circle through
// both); the "steer" control is that angle over the vehicle's maximum wheel angle,
// positive = left (counter-clockwise, ISO 8855).
//
// Speed. target(s) = min(max_speed, sqrt(a_lat / kappa(s))), with kappa the curvature of
// the circle through the route points +-corner_window_m along the route (route_check's
// corner definition), then a backward pass at brake_decel_mps2 so the car slows BEFORE a
// corner, and 0 at the route end. A PI on (target - v) gives throttle; a P on the
// overspeed gives brake. Integrator anti-windup: frozen while the throttle is saturated,
// cleared while braking.
//
// Out-and-back routes (a bridge crossed in both directions) pass the same place twice;
// the projection therefore only searches a window around the previous projection
// (forward-biased) instead of the whole polyline.
#pragma once

#include "rg/drive_script.h"
#include "rg/route_check.h"

#include "ps/vehicle/vehicle_desc.h"

#include <cstddef>
#include <functional>
#include <vector>

namespace rg {

struct RouteFollowerParams {
    // Vehicle geometry in the CHASSIS frame (+x forward). The reference point is the
    // rear axle at x = rear_axle_x_m; wheelbase is rear-to-front axle distance.
    double wheelbase_m = 2.70;
    double rear_axle_x_m = -1.62;
    double max_wheel_angle_rad = 0.5236; // wheel angle at steer = +-1

    // Pure pursuit.
    double lookahead_gain_s = 0.8;
    double lookahead_min_m = 4.0;
    double lookahead_max_m = 25.0;

    // Speed planning.
    double max_speed_mps = 25.0;     // the route's speed cap
    double a_lat_mps2 = 4.0;         // cornering speed = sqrt(a_lat * R)
    double brake_decel_mps2 = 3.5;   // planning deceleration for the backward pass
    double corner_window_m = 10.0;   // curvature window, same as route_check
    double profile_step_m = 1.0;     // resampling step of the route (arc length)

    // Speed PI. throttle = kp * e + ki * integral(e), e = target - v (m/s).
    double kp_throttle = 0.30;
    double ki_throttle = 0.12;
    double integral_limit = 2.0;     // |ki * integral| <= this (throttle units)
    double kp_brake = 0.35;          // brake = kp_brake * (v - target - brake_deadband_mps)
    double brake_deadband_mps = 0.4;
    double max_throttle = 1.0;
    double max_brake = 1.0;

    // Projection search window around the previous projection (metres of arc length).
    double search_back_m = 5.0;
    double search_ahead_m = 12.0;

    // The route is finished once the projection is within finish_distance_m of its end
    // and the car is slower than finish_speed_mps; the follower then holds the brake.
    double finish_distance_m = 1.5;
    double finish_speed_mps = 0.5;
};

// Pose of the chassis body frame and its forward speed.
struct FollowerState {
    double x = 0.0;     // chassis origin, session metres
    double y = 0.0;
    double hx = 1.0;    // unit forward vector of the chassis in the ground plane
    double hy = 0.0;    // (yaw 0 = +x = east, counter-clockwise positive)
    double speed = 0.0; // signed forward speed, m/s
};

struct FollowerCommand {
    double steer = 0.0;    // -1..1, positive = left
    double throttle = 0.0; // 0..1
    double brake = 0.0;    // 0..1
    // Diagnostics (not inputs to anything):
    double target_speed = 0.0;
    double s = 0.0;           // arc length of the projection of the rear axle
    double cross_track = 0.0; // metres, positive = the rear axle is LEFT of the route
    double lookahead_m = 0.0;
    bool finished = false;
};

class RouteFollower {
public:
    // `polyline` needs >= 2 points and a nonzero length (otherwise valid() is false and
    // step() returns an all-zero command with finished = true).
    RouteFollower(const std::vector<RoutePoint>& polyline, const RouteFollowerParams& params);

    [[nodiscard]] bool valid() const { return valid_; }
    [[nodiscard]] double length_m() const { return length_; }
    [[nodiscard]] const RouteFollowerParams& params() const { return params_; }

    // One control step. `dt` is the tick period (the integrator's time step).
    FollowerCommand step(const FollowerState& state, double dt);

    // Back to the start of the route (projection cursor and integrator).
    void reset();

    // Continues from arc length `s` instead of the start (a car placed mid-route): the
    // next projection searches only around it. Clears the integrator.
    void seek(double s);

    // The planned speed at arc length s (clamped to the route), exposed for tests and
    // for harnesses that want to log it.
    [[nodiscard]] double target_speed_at(double s) const;

    // Route curvature (1/m, >= 0) at sample i of the resampled route, and the number
    // of samples; for tests.
    [[nodiscard]] std::size_t sample_count() const { return px_.size(); }
    [[nodiscard]] double curvature_at_sample(std::size_t i) const { return kappa_[i]; }
    [[nodiscard]] double profile_at_sample(std::size_t i) const { return profile_[i]; }

private:
    struct Projection {
        std::size_t index = 0; // lower sample of the closest segment
        double t = 0.0;        // 0..1 along it
        double distance = 0.0; // unsigned
        double side = 0.0;     // +: the query point is left of the segment direction
    };
    Projection project(double qx, double qy) const;
    void point_at(double s, double& x, double& y) const;

    RouteFollowerParams params_;
    bool valid_ = false;
    double length_ = 0.0;
    double step_ = 1.0;           // actual sample spacing (length / (n - 1))
    std::vector<double> px_, py_; // resampled route
    std::vector<double> kappa_;   // curvature per sample
    std::vector<double> profile_; // planned speed per sample

    // State.
    std::size_t cursor_ = 0; // sample index of the previous projection
    bool have_cursor_ = false;
    double integral_ = 0.0;
};

// Reads wheelbase, rear-axle x and the steering limit from the loaded vehicle:
// front axle = mean attachment x of the steered wheels, rear axle = mean of the others.
// Everything else keeps `base`'s value.
RouteFollowerParams follower_params_for_vehicle(const ps::vehicle::VehicleDesc& desc,
                                                const RouteFollowerParams& base = {});

// The DriveScript adapter. Installs: ignition on, auto clutch and auto shift on, handbrake
// off (once, on the first call), then each tick reads the chassis state and writes
// steer/throttle/brake. `tick_dt` is the World tick period (1/tick_rate_hz).
// `on_command`, if set, sees every command (a harness's logging hook); it must be
// deterministic.
DriveScript::Controller make_route_follower_controller(RouteFollower follower, double tick_dt,
                                                       std::function<void(const DriveTickContext&, const FollowerState&,
                                                                          const FollowerCommand&)>
                                                           on_command = {});

} // namespace rg
