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

struct CockpitLook { double yaw_deg=0, pitch_deg=0; };
struct CockpitLookInput { double mouse_x=0,mouse_y=0,look_x=0,look_y=0; bool live=true; };
struct CockpitLookConfig { double pitch_min=0,pitch_max=0,mouse_deg_per_pixel=0,rate_deg_s=0,deadzone=0; };
CockpitLook cockpit_look_step(CockpitLook state, const CockpitLookInput& input, double dt, const CockpitLookConfig& config);
double steering_wheel_angle_rad(double steer, double lock_deg);

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
    double nose_margin_m = 0.05;           // model-based eye: this far ahead of the model's nose
};

// The bumper camera position in the chassis frame (wheel-based fallback: used
// only when the car's model bounds are unknown).
ps::Vec3 bumper_eye_local(const VehicleBounds& bounds, const BumperParams& params = {});

// The visual model's extent, read from its rig file (physics_sim.rig/1
// "overall_bounds", model frame: ISO axes, z = 0 the ground) plus an optional
// per-car override, the socket "socket_bumper_camera" (model frame).
struct ModelBounds {
    ps::Vec3 min{}, max{};
    std::optional<ps::Vec3> bumper_socket;
};
std::optional<ModelBounds> load_model_bounds(const std::string& rig_json_path, std::string* err = nullptr);

// The bumper eye from the model: the socket if the rig has one, else just ahead
// of the nose (max x + nose_margin_m), centred, height_above_ground_m above the
// model's ground but at most half the model's height. model_offset = where the
// model's origin sits in the chassis frame (the visual's alignment offset).
// Always outside the body, whatever the wheelbase/overhang.
ps::Vec3 bumper_eye_from_model(const ModelBounds& model, const ps::Vec3& model_offset,
                               const BumperParams& params = {});

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

// ------------------------------------------------------------------ drone ----

// The DroneFollow camera (R9b, docs/unreal_port.md leak #10): trails a target vehicle from above and behind. All the
// decisions - the orbit yaw wanted by the look input, the zoom law, the target-relative lag, the snap after a jump and
// the terrain clearance - are made here; drone_rig.gd only forwards input, places the Camera3D and probes the ground.
//
// Frame: a right-handed Z-up frame (x, y horizontal, z up) in the render/session orientation of the caller. The Godot
// binding maps Godot (x, y, z) to (x, -z, y), a proper rotation, so a rotation about Godot UP is a rotation about +z
// here. Positions are absolute in the caller's (floating-origin) frame; offsets are camera minus target.
struct DroneParams {
    double follow_distance_m = 12.0;   // horizontal distance behind the target (the default zoom)
    double height_per_distance = 0.5;  // follow height = distance * this
    double look_height_m = 1.0;        // the look-at point above the target origin
    double terrain_clearance_m = 1.0;
    double position_lag = 4.0;         // higher = snappier; the lag acts on the target-relative offset
    double orbit_deadzone = 0.2;       // right-stick deflection below this is no orbit
    double orbit_lag = 10.0;           // higher = snappier; <= 0 = no smoothing
    double zoom_wheel_factor = 1.15;   // distance factor per wheel tick
    double zoom_key_rate = 1.2;        // e-folds per second while the zoom key is held
    double snap_m = 60.0;              // a wanted-offset jump beyond this snaps instead of easing
    double min_distance_m = 4.0;
    double max_distance_m = 60.0;
};

struct DroneState {
    double distance_m = 12.0;
    double orbit_yaw_rad = 0.0;   // + = looking to the right of the target's heading
    ps::Vec3 offset{};            // smoothed camera - target
    ps::Vec3 smoothed_pos{};      // absolute; only read right after drone_activate
    double heading_x = 0.0;       // last horizontal heading of the target (unit)
    double heading_y = 1.0;       // initial: Godot FORWARD (0, 0, -1) in this frame
    bool from_absolute = false;
    bool initialized = false;
    bool drain_wheel = false;
};

struct DroneFollowInput {
    ps::Vec3 target_pos{};
    ps::Vec3 target_forward{};  // the target's forward axis (any length; only its horizontal part is used)
    double key_x = 0.0, key_y = 0.0;      // arrow-key look (see drone_target_orbit_yaw)
    double stick_x = 0.0, stick_y = 0.0;  // right-stick position
    bool live = true;                     // camera inputs enabled
};

struct DroneFrame {
    ps::Vec3 position{};  // before the terrain clearance
    ps::Vec3 look_at{};
    bool has_look = false; // false when the camera is within 1 cm of the look-at point
};

DroneState drone_initial_state(const DroneParams& p = {});
// Start from where the previous camera was; the exponential lag flies to the drone position.
void drone_activate(DroneState& s, const ps::Vec3& from_position);
void drone_shift_origin(DroneState& s, const ps::Vec3& delta);
// No simulation this frame: the next follow step starts over.
void drone_hold(DroneState& s);
// Orbit yaw wanted by the look input: arrow left/right = +-90 deg, arrow down = 180 deg, else the stick direction
// (atan2(x, y)) past the dead zone, else 0.
double drone_target_orbit_yaw(double key_x, double key_y, double stick_x, double stick_y, double deadzone);
// Zoom from wheel ticks and the zoom key. The first call after drone_activate only drains (ticks made while another
// rig was active must not zoom this one); not live = no zoom.
void drone_zoom_step(DroneState& s, double wheel_steps, double zoom_key, bool live, double dt, const DroneParams& p = {});
DroneFrame drone_follow_step(DroneState& s, const DroneFollowInput& in, double dt, const DroneParams& p = {});
// Ground probes around the camera (the near-plane footprint): offsets from the position, 5 entries.
std::vector<ps::Vec3> drone_probe_offsets();
// The camera height after the clearance: max(z, ground + clearance) over every probe that found ground (nullopt = none).
double drone_clear_terrain(double z, const std::vector<std::optional<double>>& grounds, double clearance_m);

// ------------------------------------------------- shared rig helpers ----
// Used by the chase, drone and walker rigs (docs/unreal_port.md leak #10). Same Z-up frame as the drone.

// Ground probes around a camera (the near-plane footprint): 5 offsets from the position, the first is zero.
std::vector<ps::Vec3> ground_probe_offsets();
// The camera height after the clearance: max(z, ground + clearance) over every probe that found ground.
double clear_terrain_height(double z, const std::vector<std::optional<double>>& grounds, double clearance_m);
// False when the camera is within 1 cm of the point it should look at (no look-at then).
bool look_valid(const ps::Vec3& position, const ps::Vec3& target);
// Orbit yaw wanted by the look input of the chase and drone rigs (see drone_target_orbit_yaw).
double rig_orbit_yaw(double key_x, double key_y, double stick_x, double stick_y, double deadzone);

// ------------------------------------------------------------------ chase ----
// The Drive chase rig (chase_rig.gd): the camera at a smoothed car-relative offset, the look-around orbit and the
// snap after a relocation. Frame as the drone: Z-up; `forward`/`left` are the chassis axes (basis x and y in Godot).
struct ChaseParams {
    double follow_distance_m = 7.0;
    double follow_height_m = 2.5;
    double look_height_m = 1.0;
    double side_offset_m = 0.0;        // + = the car's left, added to the "behind" position
    double terrain_clearance_m = 0.6;
    double position_lag = 6.0;
    double orbit_deadzone = 0.2;
    double orbit_lag = 14.0;           // <= 0 = no smoothing
    double snap_m = 60.0;
};

struct ChaseState {
    ps::Vec3 offset{};            // smoothed camera - car
    double orbit_yaw_rad = 0.0;   // + = looking to the right of the car's heading
    bool initialized = false;
};

struct ChaseInput {
    ps::Vec3 target_pos{};
    ps::Vec3 forward{1, 0, 0};
    ps::Vec3 left{0, 1, 0};
    double key_x = 0.0, key_y = 0.0;
    double stick_x = 0.0, stick_y = 0.0;
    bool live = true;
};

struct ChaseFrame {
    ps::Vec3 position{};  // before the terrain clearance
    ps::Vec3 look_at{};
};

// Re-seat behind the car on the next step (a rig switch, or no simulation this frame). The orbit yaw is kept.
void chase_reset(ChaseState& s);
ChaseFrame chase_follow_step(ChaseState& s, const ChaseInput& in, double dt, const ChaseParams& p = {});

// ------------------------------------------------------------------- free ----
// The FreeCam rig (free_rig.gd): yaw/pitch from mouse and look rate, horizontal movement relative to the heading.
// yaw 0 looks along +y (Godot -Z), + turns left (counter-clockwise seen from above); pitch + = up.
struct FreeParams {
    double move_speed_mps = 40.0;
    double fast_multiplier = 6.0;
    double look_rate_rad_s = 1.8;
    double mouse_sensitivity = 0.0025;
    double pitch_limit_rad = 1.5;
};

struct FreeState {
    ps::Vec3 position{};
    double yaw_rad = 0.0;
    double pitch_rad = 0.0;
};

struct FreeInput {
    double mouse_x = 0.0, mouse_y = 0.0;   // pixels this frame
    double look_x = 0.0, look_y = 0.0;     // look rate, -1..1
    double move_right = 0.0, move_up = 0.0, move_forward = 0.0;
    bool fast = false;
    bool live = true;                      // false: nothing changes
};

// Start where the previous camera was, looking the same way (`forward` need not be unit length).
void free_activate(FreeState& s, const ps::Vec3& position, const ps::Vec3& forward, const FreeParams& p = {});
void free_place(FreeState& s, const ps::Vec3& position, double yaw_rad, double pitch_rad, const FreeParams& p = {});
void free_shift_origin(FreeState& s, const ps::Vec3& delta);
void free_step(FreeState& s, const FreeInput& in, double dt, const FreeParams& p = {});

// ----------------------------------------------------------------- walker ----
// The OnFoot camera (walker_rig.gd): look yaw/pitch, third-person zoom, pivot smoothing, placement and aim. The
// walking itself is rg/walker.h; this is display only. Yaw convention as FreeState.
struct WalkerCamParams {
    double look_rate_rad_s = 2.2;
    double mouse_sensitivity = 0.0025;
    double third_person_distance_m = 3.2;
    double min_distance_m = 1.2;
    double max_distance_m = 9.0;
    double head_height_m = 1.55;       // third-person pivot above the feet
    double eye_height_m = 1.62;        // first-person camera above the feet
    double terrain_clearance_m = 0.3;
    double follow_rate = 28.0;
    double snap_m = 12.0;
    double third_pitch_min = -1.1;
    double third_pitch_max = 1.2;
    double first_pitch_limit = 1.5;
    double zoom_wheel_factor = 1.15;
    double zoom_key_rate = 1.2;
    double start_pitch_rad = -0.15;
};

struct WalkerCamState {
    double yaw_rad = 0.0;
    double pitch_rad = -0.15;
    double distance_m = 3.2;
    ps::Vec3 smoothed_feet{};
    bool have_walker = false;
    bool drain_wheel = false;
};

struct WalkerCamInput {
    bool has_walker = false;      // false: the walker is not spawned (yet) or already gone
    ps::Vec3 feet{};
    ps::Vec3 car_position{};      // looked at on the first sight of the walker
    double mouse_x = 0.0, mouse_y = 0.0;
    double wheel_steps = 0.0;
    double zoom_key = 0.0;
    double look_x = 0.0, look_y = 0.0;
    bool live = true;
    bool first_person = false;
};

struct WalkerCamFrame {
    bool placed = false;          // false: keep the position, only the look rotates
    bool third_person = false;
    ps::Vec3 position{};          // before the terrain clearance (third person)
    ps::Vec3 pivot{};             // the point the third-person camera aims at
    double yaw_rad = 0.0;         // the look angles (the rotation when not aimed at the pivot)
    double pitch_rad = 0.0;
};

struct WalkerCamAim {
    bool valid = false;           // false: use the look angles of the frame
    double yaw_rad = 0.0;
    double pitch_rad = 0.0;
};

WalkerCamState walker_cam_initial_state(const WalkerCamParams& p = {});
// A rig switch: the pose comes from the previous camera's forward axis, the wheel is drained once.
void walker_cam_activate(WalkerCamState& s, const ps::Vec3& from_forward, const WalkerCamParams& p = {});
void walker_cam_shift_origin(WalkerCamState& s, const ps::Vec3& delta);
void walker_cam_hold(WalkerCamState& s);  // no simulation this frame
// Horizontal forward direction of the look heading, for the walking input.
ps::Vec3 walker_cam_look_forward(const WalkerCamState& s);
WalkerCamFrame walker_cam_step(WalkerCamState& s, const WalkerCamInput& in, double dt, const WalkerCamParams& p = {});
// After the terrain clearance moved the third-person camera: aim at the pivot (pitch clamped to the third-person range).
WalkerCamAim walker_cam_aim(const ps::Vec3& pivot, const ps::Vec3& position, const WalkerCamParams& p = {});

// ------------------------------------------------------------- orbit rig ----
// The orbit view's rig around orbit_step (orbit_rig.gd): the car-relative look-at point with its short lag, the mouse
// scaling, the car's heading frame and the placement. Z-up frame as the drone.
struct OrbitRigParams {
    double snap_m = 60.0;
    double follow_lag = 8.0;
    double clearance_m = 0.6;
    double mouse_look_per_px = 0.05;   // full deflection per 20 px of captured-mouse motion in one frame
    double focus_height_m = 1.0;       // the look-at point above the chassis origin
};

struct OrbitRigState {
    OrbitState orbit{};
    ps::Vec3 target_rel{0, 0, 1};
    bool initialized = false;
};

struct OrbitRigInput {
    ps::Vec3 chassis_pos{};
    ps::Vec3 forward{1, 0, 0};         // the chassis forward axis (any length; only its horizontal part is used)
    double look_x = 0.0, look_y = 0.0; // look rate
    double mouse_x = 0.0, mouse_y = 0.0;
    double zoom_steps = 0.0;
    double zoom_key = 0.0;
    bool live = true;
};

struct OrbitRigFrame {
    ps::Vec3 position{};  // before the terrain clearance
    ps::Vec3 target{};
};

OrbitRigState orbit_rig_initial_state();
void orbit_rig_activate(OrbitRigState& s);   // default orbit, re-seat the look-at point
void orbit_rig_reset(OrbitRigState& s);      // no simulation this frame
OrbitRigFrame orbit_rig_step(OrbitRigState& s, const OrbitRigInput& in, double dt, const OrbitRigParams& p = {});


} // namespace rg
