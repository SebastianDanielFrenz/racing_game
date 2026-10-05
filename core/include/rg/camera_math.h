// rg/camera_math.h - the numbers behind the drive cameras (PLAN.md R5, 11.3).
// Engine-neutral: the GDScript rigs (bumper_rig.gd, orbit_rig.gd,
// cinematic_rig.gd, ...) only place a Camera3D from what these return and
// forward input; every choice (where the bumper camera sits, how the orbit
// moves, where a cinematic shot is placed and when it cuts) is made here and
// unit-tested.
//
// Frames. Per-vehicle values (bumper eye, orbit offset) are in the CHASSIS
// frame, ISO 8855: x forward, y left, z up - the rig maps them with the
// chassis transform's basis columns exactly like the existing chase/cockpit
// rigs. Cinematic shots are in the SESSION frame (x east, y north, metres,
// like every other rg_core position); the rig converts them to the Godot
// frame every frame with RgSimulation.session_to_godot, so a floating-origin
// rebase can never move a shot (PHYS-008).
#pragma once

#include "ps/math/vec3.h"
#include "ps/vehicle/vehicle_desc.h"

#include "rg/road_ahead.h"
#include "rg/shot_obstacles.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rg {

// ---------------------------------------------------------------- views ----

// The ways of looking at the car while driving. (The sixth camera, Free, is a
// player MODE - rg::PlayerMode::FreeCam - not a drive view.) Cycling order is
// the declaration order.
enum class DriveView { Chase, Bumper, Cockpit, Orbit, Cinematic };

inline constexpr int kDriveViewCount = 5;

const char* to_string(DriveView v); // "chase", "bumper", "cockpit", "orbit", "cinematic"
std::optional<DriveView> drive_view_from_string(const std::string& name);
// chase -> bumper -> cockpit -> orbit -> cinematic -> chase.
DriveView next_drive_view(DriveView v);
// The Tab key: cockpit -> chase, anything else -> cockpit.
DriveView toggle_cockpit_view(DriveView v);

// ------------------------------------------------------------- bumper ------

// Where the vehicle's wheels are, from its VehicleDesc (the model's own mesh
// bounds are not engine-neutral, so the bumper camera is derived from the
// wheels). Chassis frame.
struct VehicleBounds {
    double front_axle_x = 1.3;
    double rear_axle_x = -1.3;
    double half_track_m = 0.8;  // max |y| of a wheel centre
    double wheel_radius_m = 0.3;
    double ground_z = -0.5;     // chassis-frame z of the ground at rest (lowest wheel centre - radius)
};

// Defaults for a vehicle without wheels (never the case for a loaded car).
VehicleBounds vehicle_bounds(const ps::vehicle::VehicleDesc& desc);

struct BumperParams {
    double forward_of_front_axle_m = 0.75; // typical front overhang: the camera sits at the nose
    double height_above_ground_m = 0.55;   // low, bumper height
    double pitch_down_deg = 1.5;           // looks very slightly down the road
    double fov_deg = 80.0;
};

// The bumper camera position in the chassis frame.
ps::Vec3 bumper_eye_local(const VehicleBounds& bounds, const BumperParams& params = {});

// --------------------------------------------------------------- orbit -----

struct OrbitParams {
    double min_distance_m = 3.5;
    double max_distance_m = 25.0;
    double default_distance_m = 7.0;
    double min_elevation_rad = -0.05; // just above the ground plane, never under it
    double max_elevation_rad = 1.45;
    double default_elevation_rad = 0.30;
    double look_rate_rad_s = 1.8;     // at full stick/key deflection
    double zoom_wheel_factor = 1.15;  // distance factor per wheel tick
    double zoom_key_rate = 1.2;       // e-folds per second at full PageUp/PageDown
    double idle_delay_s = 2.5;        // no input for this long: slow turntable drift
    double idle_rate_rad_s = 0.20;
};

// Azimuth: 0 = behind the car, + = to the car's right (clockwise seen from
// above). Elevation: angle above the horizontal. Distance from the car's
// look-at point.
struct OrbitState {
    double azimuth_rad = 0.0;
    double elevation_rad = 0.30;
    double distance_m = 7.0;
    double idle_s = 0.0; // seconds since the last input
};

struct OrbitInput {
    double look_x = 0.0;      // -1..1, + = move the camera right around the car
    double look_y = 0.0;      // -1..1, + = move the camera up
    double zoom_steps = 0.0;  // wheel ticks, + = in
    double zoom_key = 0.0;    // -1..1, + = in
    bool live = true;         // camera inputs enabled (false while a panel is open)
};

// One frame of orbit control; azimuth wraps to (-pi, pi], elevation and
// distance clamp to the params, the idle drift turns the camera slowly once
// nothing has been touched for idle_delay_s.
OrbitState orbit_step(const OrbitState& s, const OrbitInput& in, double dt, const OrbitParams& p = {});

// The camera offset from the look-at point in the HEADING frame (x forward,
// y left, z up).
ps::Vec3 orbit_offset(const OrbitState& s);

// ---------------------------------------------------------------- chase ------

// One frame of the chase camera's follow smoothing. The camera is kept at a smoothed OFFSET from the car, not at a
// smoothed absolute position: the car's own translation then carries the camera rigidly (zero lag at any speed or
// acceleration), and only the change of the wanted offset - the car yawing or pitching, a changed follow distance -
// is eased in. Smoothing the absolute position (the earlier rig) trails the car by speed / rate metres, so the
// camera dropped back 5 m at 30 m/s and 10 m at 60 m/s. The easing is 1 - exp(-rate * dt): the same curve at any frame
// rate. `previous`/`desired` are offsets from the car (any frame, the maths is isotropic); rate <= 0 = no smoothing.
ps::Vec3 chase_follow_offset(const ps::Vec3& previous, const ps::Vec3& desired, double dt, double rate_per_s);

// -------------------------------------------------------------- cinematic --

// ChaseFallback: no clear roadside shot exists (buildings or terrain in the way), so the camera frames the car from
// behind and above like a chase camera; its x/y follow the car every update.
enum class ShotSource { RoadAhead, PredictedPath, ChaseFallback };
const char* to_string(ShotSource s);

struct CarKinematics {
    double x = 0.0; // session metres
    double y = 0.0;
    double vx = 0.0; // m/s
    double vy = 0.0;
    double heading_x = 1.0; // unit horizontal forward
    double heading_y = 0.0;
};

// A roadside camera: a fixed spot, shown until the car has passed it.
struct CinematicShot {
    std::uint64_t serial = 0; // 1, 2, ...; changes on every cut
    double x = 0.0;           // session metres
    double y = 0.0;
    double height_above_ground_m = 2.0;
    ShotSource source = ShotSource::RoadAhead;
    int side = 1;             // +1 = left of the road in the direction of travel, -1 = right
    double fov_deg = 45.0;    // set from the distance when the shot starts
};

struct CinematicParams {
    double base_fov_deg = 60.0;
    double lead_time_s = 3.5;          // place the camera this many seconds of travel ahead
    double min_lead_m = 45.0;
    double max_lead_m = 150.0;
    double side_clearance_m = 3.0;     // beyond the road's edge
    double min_height_m = 1.3;
    double max_height_m = 3.2;
    double min_hold_s = 2.0;           // never cut sooner than this
    double max_hold_s = 12.0;
    double cut_when_passed_m = 12.0;   // the car is this far past the camera along its heading
    double cut_when_far_m = 170.0;     // or this far away
    double min_speed_mps = 1.0;        // below this the heading is used as the lead direction
    double history_span_s = 8.0;       // recent path kept for the predicted-path fallback
    double history_step_s = 0.25;
    double predict_length_m = 160.0;
    double predict_step_m = 8.0;
    double max_turn_rate_rad_s = 0.8;  // clamp on the turn rate extrapolated from the recent path
    // Occlusion (only with CinematicDirector::set_obstacles): a candidate camera is rejected when it lies inside or within
    // building_margin_m of a building footprint whose vertical span covers the camera height, or when the line of sight to
    // the car (now, and to the road a third and two thirds of the way to the camera) passes through a building or
    // terrain. The director then tries the other road side, then other lead distances (lead_factors), and only then
    // frames the car from behind (ShotSource::ChaseFallback).
    double building_margin_m = 1.0;
    double car_target_height_m = 1.0;      // the point above the ground the sight line is checked to
    double sight_ground_clearance_m = 0.3; // the sight line must stay this far above sampled terrain
    double sight_sample_step_m = 5.0;      // terrain sampling interval along a sight line
    int sight_max_samples = 48;
    double lead_factors[4] = {0.7, 1.35, 0.5, 1.7}; // other leads tried after the preferred one (x the preferred lead)
    double min_candidate_lead_m = 25.0;
    double fallback_distance_m = 8.0;      // ChaseFallback: behind the car by this much
    double fallback_height_m = 2.4;        // ... and this high above the ground
    double fallback_retry_s = 1.0;         // look for a clear roadside shot again after this long
};

// How the occlusion search went (cumulative since reset()): tests and diagnostics read it.
struct CinematicStats {
    std::uint64_t candidates = 0;           // roadside candidates examined
    std::uint64_t rejected_in_building = 0; // camera point inside/near a building footprint
    std::uint64_t rejected_sight = 0;       // line of sight through a building or terrain
    std::uint64_t rejected_not_ready = 0;   // obstacle data still loading
    std::uint64_t fallbacks = 0;            // shots that ended as ChaseFallback
};

// Pure shot placement: the point `lead` metres of road/path ahead, pushed to
// one side. `polyline` is in SESSION coordinates (first point near the car);
// the shot is on the road's `side` by half_width + clearance. nullopt when the
// polyline is shorter than 2 points.
std::optional<CinematicShot> place_roadside_shot(const std::vector<RoadPoint>& polyline_session, double lead_m,
                                                 int side, double height_m, const CinematicParams& p,
                                                 ShotSource source);

// The car's recent path extrapolated ahead (constant speed, the turn rate of
// the last history_span_s clamped to max_turn_rate_rad_s) as a polyline in
// session coordinates, with a nominal half width of 3 m (a lane and a verge).
// `history` is oldest-first. Used where there is no road data (the flat world,
// off-road).
std::vector<RoadPoint> predict_path_ahead(const std::vector<std::pair<double, double>>& history,
                                               const CarKinematics& car, const CinematicParams& p);

class CinematicDirector {
public:
    explicit CinematicDirector(std::uint64_t seed = 1, CinematicParams params = {});

    // Forget the shot and the recent path (a world switch, a relocation).
    void reset();

    // One frame. `road_ahead` is the polyline rg::Session traced ahead of the
    // car in session coordinates (empty = no road data); the director falls
    // back to the predicted path. Returns the shot to show now.
    const CinematicShot& update(double dt, const CarKinematics& car, const std::vector<RoadPoint>& road_ahead);

    [[nodiscard]] bool cut_this_update() const { return cut_; }
    [[nodiscard]] std::uint64_t shots_started() const { return serial_; }
    [[nodiscard]] bool has_shot() const { return serial_ > 0; }
    [[nodiscard]] const CinematicShot& shot() const { return shot_; }
    [[nodiscard]] const CinematicParams& params() const { return params_; }

    // Non-owning; nullptr (the default) = nothing is checked, shots are placed purely along the road. Must outlive the
    // director or be cleared first. Not thread safe: call from the thread that calls update().
    void set_obstacles(const ShotObstacles* obstacles) { obstacles_ = obstacles; }
    [[nodiscard]] const CinematicStats& stats() const { return stats_; }

private:
    std::uint64_t next_random();
    double random_unit(); // [0, 1)
    void start_shot(const CarKinematics& car, const std::vector<RoadPoint>& road_ahead);
    [[nodiscard]] bool shot_expired(const CarKinematics& car) const;
    // Camera point and sight lines of a candidate; counts the reason in stats_ when it fails.
    [[nodiscard]] bool candidate_clear(const CinematicShot& shot, const CarKinematics& car,
                                       const std::vector<RoadPoint>& poly, double lead_m);
    [[nodiscard]] bool sight_clear(double ax, double ay, double az, double bx, double by, double bz) const;
    void place_chase_fallback(const CarKinematics& car);

    CinematicParams params_;
    std::uint64_t seed_ = 1;
    std::uint64_t rng_ = 1;
    CinematicShot shot_;
    std::uint64_t serial_ = 0;
    double age_s_ = 0.0;
    bool cut_ = false;
    int next_side_ = 1;
    double history_clock_s_ = 0.0;
    std::vector<std::pair<double, double>> history_; // oldest first
    const ShotObstacles* obstacles_ = nullptr;
    CinematicStats stats_;
};

} // namespace rg
