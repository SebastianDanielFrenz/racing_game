// rg/player_mode.h — the player-mode framework (R2.2 R9): which mode the
// player is in (Drive, FreeCam, and the reserved R9b/R9c modes), the rules
// each mode implies, and the world-switch flow (flat test scene <-> real
// world) - all engine-neutral (no Godot type), so a UE5 port or an XR rig
// reuses the same state machine. godot_ext/ binds it; GDScript only draws
// camera rigs/HUD and maps input according to effective_rules().
//
// Rules per mode (rules_for):
//  - render streaming (RgTerrainView::update_focus) always follows the ACTIVE
//    camera rig - the rig is what the player sees, whatever the mode;
//  - physics interest (rg::Session::physics_interest_points) follows the
//    player's VEHICLE in every implemented mode - sim state, never a camera;
//  - vehicle_control says who drives the car: Player (the input channels are
//    live) or Unattended (rg::Session overrides the driving channels with
//    unattended_controls(), see below);
//  - driving_inputs_live / camera_inputs_live say which input groups the
//    input mapping forwards (the other group is ignored, so a key shared by
//    both groups - W/S, E/Q - means one thing at a time).
//
// World switch flow: begin_world_load(kind) -> Loading (supersedes any load
// in flight: its serial becomes stale and its finish is ignored) ->
// finish_world_load(serial, ok) -> Ready or Failed. The player mode is kept
// across a world switch (no restart, no forced mode change); while the world
// is not Ready, effective_rules() masks the driving inputs and leaves the
// camera inputs live (a free cam can look around the loading screen). No
// automatic fallback to the flat world on failure - that would hide the
// failure; the HUD shows it and the world-switch key still works.
//
// Drone follow (R9b): a camera that trails a TARGET vehicle from above and
// behind. The target is the player's own car (default; the player keeps
// driving, driving inputs live) or another vehicle named by its uint64 id (an
// NPC traffic car or the NPC truck; the player's car is then Unattended like
// in free cam, camera inputs stay live). The target lives in the machine
// (set_drone_target) and resets to the own car whenever the mode is left or
// entered; effective_rules() derives vehicle_control/driving_inputs_live from
// it. next_drone_target() is the pure cycling order (own car first, then the
// candidates in range by distance, ties by id, wrapping to the own car).
//
// On foot (R9c): the player left the car and walks (rg/walker.h; the walker
// itself lives in rg::Session). The car is Unattended like in free cam, the
// driving inputs are off and the walking inputs (move, run, jump, interact)
// are live; look input (camera_inputs_live) steers the third-person or
// first-person walker rig. Getting out is only allowed while the car is
// nearly stopped (kGetOutMaxSpeedMps): the speed-aware request_mode()/
// cycle_mode() overloads refuse OnFoot above it (Result::Refused; cycle_mode
// skips it and counts the refusal so a HUD can say why). Mode cycle: drive ->
// free_cam -> drone_follow -> on_foot -> drive (Cockpit stays unused - the
// cockpit is a Drive view toggle). Leaving OnFoot by the cycle (or any
// request_mode) puts the player back into the car wherever the walker is -
// the range-checked "get in" is the interact action, handled by the Session.
#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace rg {

enum class PlayerMode : std::uint8_t {
    Drive = 0,       // chase camera behind the player's car, driving inputs live
    FreeCam = 1,     // free-flying camera, the car is Unattended
    DroneFollow = 2, // R9b: drone camera trailing a target vehicle (own car: the player keeps driving)
    Cockpit = 3,     // reserved, unused (the cockpit is a Drive view toggle, not a mode)
    OnFoot = 4,      // R9c: walker beside the car, interact gets back in (own car only)
};
inline constexpr int kPlayerModeCount = 5;

enum class WorldKind : std::uint8_t { Flat = 0, RealWorld = 1 };
enum class WorldPhase : std::uint8_t { None = 0, Loading = 1, Ready = 2, Failed = 3 };
enum class VehicleControl : std::uint8_t { Player = 0, Unattended = 1 };
enum class CameraRig : std::uint8_t { Chase = 0, Free = 1, Drone = 2, Seat = 3, Walker = 4 };

struct ModeRules {
    bool implemented = false;
    VehicleControl vehicle_control = VehicleControl::Unattended;
    bool driving_inputs_live = false;
    bool camera_inputs_live = false;
    bool walking_inputs_live = false; // R9c: move/run/jump/interact reach the walker
    CameraRig camera_rig = CameraRig::Free;
};

// `drone_target` only matters for DroneFollow: nullopt = the player's own car
// (driving), an id = another vehicle (the player's car is Unattended).
[[nodiscard]] ModeRules rules_for(PlayerMode mode, std::optional<std::uint64_t> drone_target = std::nullopt);

[[nodiscard]] const char* to_string(PlayerMode mode);
[[nodiscard]] const char* to_string(WorldKind kind);
[[nodiscard]] const char* to_string(WorldPhase phase);
[[nodiscard]] const char* to_string(VehicleControl control);
[[nodiscard]] const char* to_string(CameraRig rig);
// Case-sensitive, the to_string spellings ("drive", "free_cam", ...).
[[nodiscard]] std::optional<PlayerMode> player_mode_from_string(std::string_view name);
[[nodiscard]] std::optional<WorldKind> world_kind_from_string(std::string_view name);

// What rg::Session writes into the driving channels while the car is
// Unattended (the player is not driving it): steer 0, throttle 0, starter 0,
// clutch pedal fully pressed (1 = disengaged, so the engine keeps idling and
// cannot stall whatever gear is selected), and the brakes: service brake
// kUnattendedRollingBrake while the car still moves faster than
// kUnattendedHoldSpeedMps (a firm stop without locking the rear axle with
// the handbrake at speed), then service brake 1 plus handbrake 1 to hold it.
// Every other channel (ignition, assists, shift counters) passes through.
struct UnattendedControls {
    double steer = 0.0;
    double throttle = 0.0;
    double brake = 0.0;
    double handbrake = 0.0;
    double clutch = 1.0;
    double starter = 0.0;
};
inline constexpr double kUnattendedHoldSpeedMps = 2.0;
inline constexpr double kUnattendedRollingBrake = 0.6;
// R9c: the player may only get out while the own car is slower than this.
inline constexpr double kGetOutMaxSpeedMps = 2.0;
[[nodiscard]] constexpr bool may_get_out(double vehicle_speed_mps) {
    // NaN speeds fail the comparison: refuse.
    return vehicle_speed_mps < kGetOutMaxSpeedMps && vehicle_speed_mps > -kGetOutMaxSpeedMps;
}
[[nodiscard]] UnattendedControls unattended_controls(double speed_mps);

// One followable vehicle other than the player's own car, session XY in metres.
struct DroneCandidate {
    std::uint64_t id = 0;
    double x = 0.0;
    double y = 0.0;
};

// The target after `current` in the drone-follow cycle: the own car (nullopt)
// first, then every candidate within max_range_m of (ref_x, ref_y) ordered by
// distance, ties by id, then back to the own car. From the own car it returns
// the nearest candidate (nullopt when none is in range); from the last one it
// wraps to nullopt; a `current` that is not an in-range candidate (gone, out
// of range) goes to the own car. Non-finite candidates are ignored. Pure,
// allocation-free, O(n). The binding passes the PLAYER'S CAR as the reference
// point, not the camera: the camera sits next to the current target, so a
// camera-relative order re-sorts on every step and ping-pongs between the two
// nearest vehicles.
[[nodiscard]] std::optional<std::uint64_t> next_drone_target(std::optional<std::uint64_t> current,
                                                             std::span<const DroneCandidate> candidates,
                                                             double ref_x, double ref_y, double max_range_m);

class PlayerModeMachine {
public:
    enum class Result : std::uint8_t { Changed, NoChange, NotImplemented, Refused };

    explicit PlayerModeMachine(PlayerMode start = PlayerMode::Drive);

    [[nodiscard]] PlayerMode mode() const { return mode_; }
    // The start mode must be implemented (std::invalid_argument otherwise).
    Result request_mode(PlayerMode mode);
    // Next implemented mode after the current one (wrapping); returns it.
    PlayerMode cycle_mode();
    // Speed-aware variants (R9c): entering OnFoot needs may_get_out(speed) of
    // the player's own car. request_mode returns Refused (no state change,
    // get_out_refusals() +1); cycle_mode skips OnFoot (also +1) and goes on
    // to the next implemented mode, so the cycle never gets stuck.
    Result request_mode(PlayerMode mode, double vehicle_speed_mps);
    PlayerMode cycle_mode(double vehicle_speed_mps);
    // Lifetime count of refused get-outs (a HUD compares it with the last one it saw).
    [[nodiscard]] std::uint64_t get_out_refusals() const { return get_out_refusals_; }

    // Drone-follow target: nullopt = the player's own car, an id = another
    // vehicle. Only meaningful in DroneFollow: outside it the call changes
    // nothing and returns false. Returns true when the machine is in the
    // requested state afterwards (also when it already was); only an actual
    // change bumps revision(). Entering or leaving DroneFollow resets it to
    // the own car.
    bool set_drone_target(std::optional<std::uint64_t> target);
    [[nodiscard]] std::optional<std::uint64_t> drone_target() const { return drone_target_; }

    // World switch. Returns the load's serial (never 0).
    std::uint64_t begin_world_load(WorldKind kind);
    // false (and no state change) when `serial` is not the latest load's.
    bool finish_world_load(std::uint64_t serial, bool ok);
    [[nodiscard]] WorldKind world_kind() const { return world_kind_; } // the latest requested world
    [[nodiscard]] WorldPhase world_phase() const { return world_phase_; }
    [[nodiscard]] std::uint64_t world_serial() const { return world_serial_; }
    // The other world (the world-switch key's target).
    [[nodiscard]] WorldKind other_world() const {
        return world_kind_ == WorldKind::Flat ? WorldKind::RealWorld : WorldKind::Flat;
    }

    // rules_for(mode(), drone_target()) with the world phase applied: driving
    // inputs are masked while the world is not Ready.
    [[nodiscard]] ModeRules effective_rules() const;

    // Bumped by every state change (mode or world) - cheap change detection
    // for a binding that pushes state into the Session.
    [[nodiscard]] std::uint64_t revision() const { return revision_; }

private:
    PlayerMode mode_;
    std::optional<std::uint64_t> drone_target_;
    WorldKind world_kind_ = WorldKind::Flat;
    WorldPhase world_phase_ = WorldPhase::None;
    std::uint64_t world_serial_ = 0;
    std::uint64_t revision_ = 0;
    std::uint64_t get_out_refusals_ = 0;
};

} // namespace rg
