// rg/walker.h — the on-foot player (R2.2 R9c): a kinematic capsule character
// controller on top of ps::IRigidBackend's PUBLIC query API, plus the pure
// pieces around it (own-car footprint, spawn-spot choice, enter-range check).
// Engine-neutral (no Godot type): godot_ext only forwards input and draws.
//
// Representation (decision, see the R9c report / racing_game CLAUDE.md):
//  - the walker is ONE Jolt body, kinematic, a CapsuleShape (axis rotated
//    upright). The controller moves it exactly like the NPC traffic does:
//    each tick it reads the pose, decides the target and writes only
//    set_motion((target - pos) / dt); the backend integrates it during
//    World::step, so the pose after the step IS the target and the walker is
//    part of ps::World::state_hash (pose + velocity bytes) with no extra
//    hash bookkeeping. A kinematic body never collides with static terrain
//    (no contact work), but it does collide with dynamic bodies, so other
//    physics actors see a person-sized object.
//  - collision is resolved by the controller itself with collide_shape_into /
//    ray_cast only (never shape_cast: it cannot exclude the walker's own
//    body): move -> overlap queries -> push-out along the contact normals,
//    classifying normals into walkable ground (slope <= slope_limit_deg),
//    walls, and ceilings. Obstacles shorter than step_height_m are stepped
//    over by the classic raise / move / drop sequence; the drop ray also
//    snaps the walker to the ground (walking down kerbs and slopes).
//  - the car's PHYSICAL collider is a thin box (wheels are suspension casts,
//    not bodies), so the own car is excluded from the queries and replaced by
//    an oriented footprint rectangle (OrientedRect, vehicle_footprint()),
//    built from the wheel attachments + chassis box + margin. NPC vehicles
//    are kinematic boxes and block through the normal overlap queries.
//
// Never falls through terrain: a vertical sweep that finds no ground holds
// the walker in place (a long ray decides between "ledge, fall" and "no
// ground loaded, hold"), and the Session gate keeps the walker's own terrain
// resident (the walker is physics interest point 0).
//
// Determinism: plain IEEE arithmetic (+ - * / sqrt) and ps::math::sincos, no
// libm transcendental, no unordered iteration; the only inputs are the body
// pose, the input struct and the backend's own (deterministic) queries.
#pragma once

#include "ps/backend/i_rigid_backend.h"
#include "ps/math/vec3.h"
#include "ps/world/ids.h"
#include "ps/world/world.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace rg {

struct WalkerConfig {
    double radius_m = 0.30;
    double height_m = 1.75;            // feet to head
    double walk_speed_mps = 1.4;
    double run_speed_mps = 5.0;
    double ground_accel_mps2 = 30.0;
    double air_accel_mps2 = 4.0;
    double jump_speed_mps = 4.2;       // ~0.9 m apex
    double gravity_mps2 = 9.81;
    double max_fall_speed_mps = 40.0;
    double step_height_m = 0.15;       // kerbs up to this are walked over
    double slope_limit_deg = 45.0;     // steeper surfaces are walls
    double snap_down_m = 0.30;         // walking off a drop up to this keeps the walker grounded
    double turn_rate_rad_s = 14.0;     // facing follows the movement direction
    double enter_range_m = 1.5;        // get-in range from the own car's footprint
    double enter_max_dz_m = 2.5;
};

// Per-tick input. Move axes are the stick/WASD values in [-1, 1] relative to
// the LOOK heading (look_yaw_rad, ISO: 0 = east, counter-clockwise positive;
// move_forward is along it, move_right is clockwise of it). The magnitude is
// clamped to 1 (diagonals are not faster).
struct WalkerInput {
    double move_right = 0.0;
    double move_forward = 0.0;
    double look_yaw_rad = 0.0;
    bool run = false;
    bool jump = false; // true on the tick(s) the jump is requested; honoured only while grounded
};

struct WalkerState {
    ps::Vec3 feet{};            // capsule bottom
    ps::Vec3 velocity{};        // m/s, vertical in z
    double yaw_rad = 0.0;       // facing (display only, not hashed)
    bool grounded = false;
    ps::Vec3 ground_normal{0.0, 0.0, 1.0};
    bool blocked = false;       // the last tick's horizontal move was stopped or deflected by an obstacle
    bool hold = false;          // no ground found below: the walker holds its height (terrain not loaded)
};

// A vertical prism standing on the ground: an oriented rectangle in XY with a
// z range. Used for the own car's footprint. `cos_yaw`/`sin_yaw` are cached by
// make_oriented_rect (ps::math::sincos).
struct OrientedRect {
    double cx = 0.0, cy = 0.0;
    double half_x = 0.0, half_y = 0.0; // along the rect's own x (forward) and y (left)
    double cos_yaw = 1.0, sin_yaw = 0.0;
    double z_min = 0.0, z_max = 0.0;   // absolute
};
[[nodiscard]] OrientedRect make_oriented_rect(double cx, double cy, double yaw_rad, double half_x, double half_y,
                                              double z_min, double z_max);

// Distance from (x, y) to the rectangle's boundary, 0 inside.
[[nodiscard]] double distance_to_rect(const OrientedRect& rect, double x, double y);
// Pushes a circle (centre x/y, radius) out of the rectangle; returns false
// (and leaves x/y) when they do not overlap. Inside the rectangle the centre
// goes to the nearest edge plus the radius.
bool push_circle_out_of_rect(const OrientedRect& rect, double radius, double& x, double& y);

// The own car's footprint: the bounding rectangle, in the chassis' own XY
// axes, of the chassis box and every wheel (attachment +- radius/half width)
// plus `margin_m`, placed in the world with the chassis' yaw (its +X axis
// projected on the ground; callers pass the local +X in world axes). Not
// centred on the chassis: a car whose wheelbase is not symmetric about its
// origin gets the tighter, offset rectangle. z range is absolute (the walker
// is blocked while its body overlaps it vertically).
struct WheelFootprint {
    double x = 0.0, y = 0.0;       // chassis-local attachment XY
    double radius = 0.0;           // along x
    double half_width = 0.0;       // along y
};
[[nodiscard]] OrientedRect vehicle_footprint(const ps::Vec3& chassis_position, const ps::Vec3& chassis_forward_world,
                                             double chassis_half_x, double chassis_half_y,
                                             std::span<const WheelFootprint> wheels, double margin_m, double z_min,
                                             double z_max);

// Get-in range: the walker (feet) is within `range_m` of the footprint
// outline in XY and within `max_dz_m` vertically of the footprint's ground
// (taken as z_min + 0.5).
[[nodiscard]] bool within_enter_range(const OrientedRect& footprint, const ps::Vec3& walker_feet, double range_m,
                                      double max_dz_m);

// Spawn beside the driver's door, in order of preference: driver side (left,
// +local y), passenger side (right), behind the car, in front of it, and last
// above the roof. `probe` answers whether a standing capsule fits at a
// candidate's XY and returns the ground height there (nullopt = blocked or no
// ground). The first candidate with an answer wins; the roof fallback is
// returned (above = true) with z = roof_z when every side is blocked.
struct SpawnSpot {
    ps::Vec3 feet{};
    double yaw_rad = 0.0;   // facing away from the car's door side (the walker looks outward)
    int candidate = 0;      // 0 driver side, 1 passenger side, 2 rear, 3 front, 4 above the roof
    bool above_roof = false;
};
using SpawnProbe = std::function<std::optional<double>(double x, double y)>;
[[nodiscard]] SpawnSpot select_spawn_spot(const OrientedRect& footprint, const ps::Vec3& chassis_position,
                                          double chassis_yaw_rad, double walker_radius_m, double door_x_m,
                                          double clearance_m, double roof_z_m, const SpawnProbe& probe);

class WalkerController {
public:
    // The World must outlive the controller (the destructor destroys the body).
    WalkerController(ps::World& world, const WalkerConfig& config);
    ~WalkerController();
    WalkerController(const WalkerController&) = delete;
    WalkerController& operator=(const WalkerController&) = delete;

    // Creates the kinematic capsule body with its feet at `feet`.
    void spawn(const ps::Vec3& feet, double yaw_rad);
    // Destroys the body. Idempotent.
    void despawn();
    [[nodiscard]] bool active() const { return body_.valid(); }
    [[nodiscard]] ps::BodyId body() const { return body_; }
    [[nodiscard]] const WalkerConfig& config() const { return config_; }

    // Bodies the queries must ignore (the own car's thin physical chassis;
    // the walker's own body is always ignored).
    void set_ignored_bodies(std::vector<ps::BodyId> ids) { ignored_ = std::move(ids); }
    // Extra obstacles (the own car's footprint). Copied.
    void set_obstacles(std::span<const OrientedRect> rects) { obstacles_.assign(rects.begin(), rects.end()); }

    // One tick: reads the body's pose, integrates input/gravity, resolves
    // collisions and writes the body's target velocity (set_motion). The
    // caller then steps the World. Returns the state AFTER the move (feet =
    // where the body will be once the step ran). Requires active().
    const WalkerState& step(const WalkerInput& input, double dt);
    // The state as of the last step() (or spawn()). After a World step it
    // matches the body pose; sync_from_body() re-reads it explicitly.
    [[nodiscard]] const WalkerState& state() const { return state_; }
    void sync_from_body();
    // Zero the body's velocity (the World is about to step without step()).
    void halt();

    // Capsule centre for a feet position / the pose the body has there.
    [[nodiscard]] ps::Vec3 centre_of(const ps::Vec3& feet) const;

private:
    struct Ground {
        double z = 0.0;
        ps::Vec3 normal{0.0, 0.0, 1.0};
    };
    [[nodiscard]] bool ignored(ps::BodyId id) const;
    [[nodiscard]] std::optional<Ground> ray_down(double x, double y, double z_from, double dist) const;
    // Pushes the capsule at `feet` out of overlapping bodies and rects.
    // allow_up: walkable overlaps lift the walker (final placement); false:
    // the raised pass of a ground move, where only walls count and a contact
    // above ref_z + step height is a wall too. Returns the accumulated push.
    ps::Vec3 resolve(ps::Vec3& feet, bool allow_up, double ref_z, const ps::Vec3& move_dir, bool& hit_ceiling) const;
    void apply_rects(ps::Vec3& feet) const;
    struct GroundMove {
        ps::Vec3 feet;
        bool ok = false;
        bool grounded = false;
        bool ledge = false; // no ground within snap range: walked off an edge
        Ground ground;
        ps::Vec3 slide_normal{};
        bool have_slide_normal = false;
    };
    [[nodiscard]] GroundMove try_ground_move(const ps::Vec3& feet, double dx, double dy) const;
    void write_target(const ps::Vec3& target_feet, double dt);

    ps::World& world_;
    WalkerConfig config_;
    double cos_slope_ = 0.7071067811865476;
    ps::BodyId body_{};
    WalkerState state_{};
    std::vector<ps::BodyId> ignored_;
    std::vector<OrientedRect> obstacles_;
    mutable std::vector<ps::ShapeContactHit> hits_; // scratch
};

} // namespace rg
