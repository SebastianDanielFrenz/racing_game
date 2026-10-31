// rg/trailer.h - towing a trailer (farm attachment plan step A2, farming_sim docs/farm/PLAN.md section 6). Engine-neutral.
//
// physics_sim owns the coupling (ps::machine, World::check_attach / attach / detach, hitch classes, the articulation limits,
// the tow brake). This file is the GAME side of it:
//   * TrailerConfig (data/trailers/trailers.json, rg.trailers/1): which hitch classes the player can couple, the reach and
//     the speeds of a hook-up, the trailer kinds with their load states, the overload rule, the advice texts. Every
//     number there carries a [C]/[D]/[M]/[AS] source tag; nothing here is a literal threshold.
//   * plan_hookup / align_trailer_pose: PURE functions - which of the cars' towing points and the trailer's towed points
//     mate, how far apart they are, whether a hook-up is allowed and where the trailer has to stand for the sockets to meet.
//   * TrailerJack: the trailer's parking leg as a force element; it carries the nose of a standing trailer.
//   * HitchOverload: the pure overload rule (what the 300-730 kN of a jackknife against the articulation limit means).
//   * TrailerRig: the attachment state machine on the stepping thread (None -> Standing -> Coupled -> Standing / None).
//     rg::Session owns one; every call is sim thread, between steps.
//   * TrailerRecord (user dir), CoupledSetAuthority (one authority per coupled set): the save and the server rule.
//
// What the physics needs from the game (external/physics_sim/docs/data_formats/hitch.md) and where it is done:
//   - detach before a reset or a teleport of a coupled vehicle   -> TrailerRig::begin_relocation
//   - vehicles with a channel_scope, World::vehicle_channel      -> TrailerRig::spawn ("trailer"), brake_channel_
//   - attach on the sim thread between steps, check_attach first -> TrailerRig::attach
//   - sockets inside the class's latch limits                    -> align_trailer_pose (the assisted hook-up)
//   - a free-standing tandem trailer drops its nose              -> TrailerJack
//   - retired snapshot entries                                   -> TrailerRig::snapshot reads live ids only
#pragma once

#include "ps/drivetrain/limit_findings.h"
#include "ps/drivetrain/limit_policy.h"
#include "ps/machine/hitch_point.h"
#include "ps/math/pose.h"
#include "ps/vehicle/tyre_model.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/vehicle/wheel_state.h"
#include "ps/world/force_element.h"
#include "ps/world/ids.h"
#include "ps/world/world.h"

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rg {

// ---- configuration (data/trailers/trailers.json) ---------------------------------------------------------------------

struct TrailerLoad {
    std::string id;    // "empty", "half", "full"
    std::string label;
    double mass_kg = 0.0; // the whole trailer, the body mass the game creates the chassis with
};

struct TrailerJackSpec {
    ps::Vec3 foot_local{};          // where the foot touches the ground, chassis frame
    double stiffness_n_m = 0.0;
    double damping_n_s_m = 0.0;
    double max_force_n = 0.0;
};

struct TrailerKind {
    std::string id;
    std::string title;
    std::string vehicle_path; // physics vehicle file (absolute)
    std::string model_path;   // .glb (absolute); the rig json sits next to it
    ps::Vec3 chassis_half_extents{2.5, 0.5, 0.3};
    double chassis_friction = 0.5;
    std::vector<TrailerLoad> loads;
    std::string default_load;
    TrailerJackSpec jack;
    // The model's rig origin in the chassis frame (ISO). The same number vehicle_visual.gd derives for a car from its
    // wheels; a trailer rig has no susp_* nodes, so it comes with the data.
    ps::Vec3 model_offset{};
    double length_m = 7.7; // overall, for the camera framing

    [[nodiscard]] const TrailerLoad* find_load(const std::string& load_id) const;
};

// A hitch class the player can couple. A class that is not listed is never offered a hook-up.
struct HitchClassRule {
    std::string name;  // the physics_sim.hitch_class name: "ball", "drawbar_pin"
    std::string label; // shown to the player
    // The hitch lets go when the force on it stays above break_d_multiple x D for break_hold_s. D is the ISO 7641-1 D value of
    // the pair, g x T x R / (T + R) (T the towing vehicle's mass, R the trailer's).
    double break_d_multiple = 4.0;
    double break_hold_s = 0.05;
};

struct TrailerAdviceText {
    // Placeholders: {mass} {limit_mass} {tongue} {limit_tongue} {severity} {car} {trailer} {force} {break_force} {distance}
    std::string tow_mass;
    std::string tongue_load;
    std::string overload;
    std::string hookup_ready;
    std::string hookup_far;
    std::string coupled;
    std::string standing;
    std::string severity_near, severity_limiting, severity_exceeded;
};

struct TrailerConfig {
    struct Hookup {
        bool assist = true;               // false: the sockets must already be inside the class's latch limits
        double reach_m = 3.0;             // sockets further apart than this are not offered a hook-up
        double max_heading_deg = 60.0;    // angle between the two vehicles' forward axes
        double max_speed_m_s = 0.5;       // the car, the trailer and their sockets
        double max_vertical_m = 0.32;     // the jack's stroke: the nose height the assist may change
        double nose_down_margin = 0.0;    // reserved
    } hookup;
    struct Unhook {
        double max_speed_m_s = 1.0;       // the car's speed above which the trailer is not let go
    } unhook;
    struct Spawn {
        double gap_m = 3.0;               // standing trailer: its eye this far behind the car's tow point
        double clearance_m = 0.03;        // the chassis is placed this far above its rest height
        double ray_height_m = 6.0;        // the ground rays start this far above the tow point
        double ray_range_m = 60.0;
    } spawn;
    struct Abandon {
        double distance_m = 250.0;        // a standing trailer left further behind than this is removed
        double below_m = 50.0;            // ... or one that fell this far below the car
        double relocation_keep_m = 30.0;  // after a relocation a standing trailer within this distance of its old place stays
    } abandon;
    struct Park {
        double offset_x_m = 40.0;         // while the car is parked for a relocation the trailer parks this far beside it
    } park;
    double brake_when_standing = 1.0;     // the trailer's own brake channel while it stands uncoupled (a parking brake)
    std::string channel_scope = "trailer";
    std::string limit_policy_file;        // resolved, "" = physics' compiled default
    ps::drivetrain::LimitPolicy policy;
    TrailerAdviceText advice;
    std::vector<HitchClassRule> classes;
    std::vector<TrailerKind> trailers;
    std::string default_trailer;

    [[nodiscard]] const HitchClassRule* find_class(const std::string& name) const;
    [[nodiscard]] const TrailerKind* find_kind(const std::string& id) const;
};

std::optional<TrailerConfig> parse_trailer_config(const std::string& json_text, const std::string& base_dir, const std::string& origin,
                                                  std::string* err);
std::optional<TrailerConfig> load_trailer_config(const std::string& path, std::string* err);

// {name} -> values[name]; an unknown name stays as written.
std::string render_trailer_text(const std::string& tpl, const std::map<std::string, std::string>& values);

// ---- hook-up planning (pure) -----------------------------------------------------------------------------------------

struct HookupGeometry {
    const std::vector<ps::machine::HitchPointDesc>* towing_points = nullptr; // the car's
    const std::vector<ps::machine::HitchPointDesc>* towed_points = nullptr;  // the trailer's
    ps::Pose car{};
    ps::Pose trailer{};
    ps::Vec3 car_velocity{};
    ps::Vec3 trailer_velocity{};
    ps::Vec3 trailer_pivot_local{}; // the point the assist pitches the trailer about: the middle of its axles
};

enum class HookupVerdict : int { Ok = 0, NoPair = 1, TooFar = 2, HeadingOff = 3, TooFast = 4, OutOfJackRange = 5, NoGeometry = 6 };

struct HookupPlan {
    HookupVerdict verdict = HookupVerdict::NoGeometry;
    std::string towing_point, towed_point, class_name;
    double distance_m = 0.0;     // between the two sockets
    double vertical_m = 0.0;     // socket height minus eye height (the nose has to move by this)
    double heading_deg = 0.0;    // between the forward axes
    double speed_m_s = 0.0;      // the faster of the car and the trailer
    double latch_error_m = 0.0;  // the class's latch limit: inside it no assist is needed
    bool within_latch = false;   // the sockets already meet the class's latch error
    ps::Pose aligned{};          // where the trailer has to stand for the sockets to coincide
    std::string message;
};

// Every pair of one towing and one towed point that mate (same class name) and whose class the config lists, nearest first.
// The plan is for the NEAREST pair. Never throws.
[[nodiscard]] HookupPlan plan_hookup(const TrailerConfig& config, const HookupGeometry& geometry);

// The pose the trailer has to take for its eye (eye_local, chassis frame) to sit at `target`: first a shift in plan, then a
// pitch about the lateral axis through `pivot_local` (the axles, so the wheels keep the ground) for the height. Three
// rounds; exact to well under a millimetre for a nose change of the jack's stroke.
[[nodiscard]] ps::Pose align_trailer_pose(const ps::Pose& trailer, const ps::Vec3& eye_local, const ps::Vec3& pivot_local, const ps::Vec3& target);

// The yaw (about +Z, radians) a pose heads in: its forward axis projected on the ground plane.
[[nodiscard]] double heading_yaw(const ps::Quat& orientation);

// ---- the overload rule (pure) ----------------------------------------------------------------------------------------

// ISO 7641-1 D value in newtons: g x T x R / (T + R).
[[nodiscard]] double iso_d_value_n(double towing_mass_kg, double trailer_mass_kg);

class HitchOverload {
public:
    // True on the update that makes the hitch let go: `force_n` has stayed above `limit_n` for `hold_s`. A force back under
    // the limit starts the count again (a spike shorter than hold_s is a shock the hitch survives).
    bool update(double force_n, double limit_n, double hold_s, double dt_s);
    void reset() { over_s_ = 0.0; }
    [[nodiscard]] double over_s() const { return over_s_; }

private:
    double over_s_ = 0.0;
};

// ---- the parking leg -------------------------------------------------------------------------------------------------

// A unilateral spring-damper at the foot, pushing up while the foot is below the ground. The ground height under the foot is
// set by the owner between steps (a plain double read by evaluate() during the step, written before it).
class TrailerJack final : public ps::IForceElement {
public:
    TrailerJack(ps::Frame frame, TrailerJackSpec spec) : frame_(frame), spec_(spec) {}
    [[nodiscard]] const ps::Frame& frame() const override { return frame_; }
    [[nodiscard]] ps::Wrench evaluate(const ps::ForceElementContext& ctx) const override;
    void set_ground(bool deployed, bool have_ground, double ground_z) {
        deployed_ = deployed;
        have_ground_ = have_ground;
        ground_z_ = ground_z;
    }
    [[nodiscard]] const TrailerJackSpec& spec() const { return spec_; }

private:
    ps::Frame frame_;
    TrailerJackSpec spec_;
    bool deployed_ = false;
    bool have_ground_ = false;
    double ground_z_ = 0.0;
};

// ---- the attachment state machine ------------------------------------------------------------------------------------

enum class TrailerState : int { None = 0, Standing = 1, Coupled = 2 };

struct TrailerResult {
    bool ok = false;
    std::string message;
};

struct TrailerFinding {
    ps::drivetrain::LimitKind kind = ps::drivetrain::LimitKind::TowedMass;
    ps::drivetrain::LimitSeverity severity = ps::drivetrain::LimitSeverity::Near;
    double value = 0.0;
    double limit = 0.0;
    std::string text; // the configured advice line, filled
};

// What the HUD, the camera and the visual read; captured on the stepping thread with the rest of the tick.
struct TrailerSnapshot {
    bool present = false;
    TrailerState state = TrailerState::None;
    std::string kind_id, load_id, vehicle_name;
    std::uint64_t serial = 0;       // +1 for every trailer spawned (the visual rebuilds on a change)
    ps::Pose pose{};
    ps::Motion motion{};
    std::vector<ps::vehicle::WheelState> wheels;
    std::vector<std::string> wheel_names;
    double mass_kg = 0.0;
    // Coupled
    std::string towing_point, towed_point, class_name;
    double hitch_force_n = 0.0;     // last tick's mean force on the towing socket
    double break_force_n = 0.0;
    double yaw_deg = 0.0, pitch_deg = 0.0, roll_deg = 0.0; // articulation angles
    double socket_error_m = 0.0;
    double tow_brake = 0.0;         // the trailer's brake demand through the hitch, 0..1 (the car's brake channel through the trailer file's tow_brake)
    double latch_progress = 1.0;
    double towed_mass_kg = 0.0, tongue_load_n = 0.0;
    std::vector<TrailerFinding> findings;
    // Standing: the nearest mating pair and whether a hook-up would be allowed now
    double hookup_distance_m = 0.0;
    HookupVerdict hookup_verdict = HookupVerdict::NoGeometry;
    // The last action's outcome, and lifetime counters
    std::string last_message;
    std::uint64_t attaches = 0, detaches = 0, overloads = 0, removed = 0;
    // The abandoned/relocation bookkeeping
    bool jack_down = false;
};

class TrailerRig {
public:
    struct Car {
        ps::VehicleId vehicle{};
        ps::BodyId body{};
        const ps::vehicle::VehicleDesc* desc = nullptr;
    };

    explicit TrailerRig(std::shared_ptr<const TrailerConfig> config);
    ~TrailerRig();
    TrailerRig(const TrailerRig&) = delete;
    TrailerRig& operator=(const TrailerRig&) = delete;

    [[nodiscard]] const TrailerConfig& config() const { return *config_; }
    [[nodiscard]] TrailerState state() const { return state_; }
    [[nodiscard]] bool present() const { return state_ != TrailerState::None; }
    [[nodiscard]] ps::BodyId body() const { return body_; }
    [[nodiscard]] ps::VehicleId vehicle() const { return vehicle_; }
    [[nodiscard]] std::uint64_t serial() const { return serial_; }
    // The speed law of the trailer tyres' rolling resistance: the session's own (rolling_resistance_model). Set before the
    // first spawn; a desc already built keeps the law it was built with.
    void set_rolling_resistance_law(ps::vehicle::RollingResistanceSpeedLaw law) { rolling_law_ = law; }
    [[nodiscard]] const std::string& kind_id() const { return kind_id_; }
    [[nodiscard]] const std::string& load_id() const { return load_id_; }

    // --- sim thread, between steps (every one of them throws nothing; a refusal is a TrailerResult with a message) ---
    // Puts a trailer of `kind` (loaded `load`; "" = the kind's default) behind the car: standing `gap_m` behind its tow point,
    // or - coupled - hooked on. A trailer that is already there is removed first.
    TrailerResult spawn(ps::World& world, const Car& car, const std::string& kind, const std::string& load, bool coupled);
    TrailerResult attach(ps::World& world, const Car& car);
    TrailerResult detach(ps::World& world, const Car& car, bool ignore_speed = false);
    TrailerResult remove(ps::World& world);
    // Puts a coupled or standing trailer upright behind the car again (it rolled over, or it is stuck): detach, place, attach.
    TrailerResult recover(ps::World& world, const Car& car);

    // Right before / after World::step.
    void before_step(ps::World& world, const Car& car);
    void after_step(ps::World& world, const Car& car, double dt_s);

    // A reset or a teleport of the car: begin_relocation detaches (the physics refuses a teleport of a coupled vehicle) and
    // remembers whether to hook on again; park() holds the trailer beside the parked car through the terrain priming ticks;
    // finish_relocation places it behind the car again and re-couples, or puts a standing one back where it stood (within
    // abandon.relocation_keep_m of the new place) or removes it; abort_relocation (no ground at the target) removes it.
    void begin_relocation(ps::World& world, const Car& car);
    void park(ps::World& world, const ps::Pose& car_park_pose);
    TrailerResult finish_relocation(ps::World& world, const Car& car);
    void abort_relocation(ps::World& world);
    [[nodiscard]] bool relocating() const { return relocating_; }

    [[nodiscard]] TrailerSnapshot snapshot(const ps::World& world, const Car& car) const;
    // A pose or placement changed discontinuously (spawn, hook-up shift, relocation): the render history must step, not blend.
    [[nodiscard]] std::uint64_t teleports() const { return teleports_; }

private:
    struct KindCache {
        ps::vehicle::VehicleDesc desc;
    };
    const ps::vehicle::VehicleDesc& desc_for(const TrailerKind& kind);
    [[nodiscard]] const TrailerKind* current_kind() const;
    TrailerResult place_standing(ps::World& world, const Car& car, double gap_m);
    bool ground_z(const ps::World& world, double x, double y, double above_z, double* out) const;
    HookupGeometry geometry_for(const ps::World& world, const Car& car) const;
    void set_brake(ps::World& world, double value);
    void note_teleport() { ++teleports_; }
    TrailerResult refuse(std::string message) {
        last_message_ = message;
        return {false, std::move(message)};
    }
    void destroy(ps::World& world);

    std::shared_ptr<const TrailerConfig> config_;
    std::map<std::string, KindCache> descs_;
    TrailerState state_ = TrailerState::None;
    std::string kind_id_, load_id_;
    ps::BodyId body_{};
    ps::VehicleId vehicle_{};
    ps::HitchId hitch_{};
    ps::ForceElementId jack_id_{};
    std::unique_ptr<TrailerJack> jack_;
    const ps::vehicle::VehicleDesc* active_desc_ = nullptr; // into descs_ (a std::map: the address is stable)
    ps::vehicle::RollingResistanceSpeedLaw rolling_law_ = ps::vehicle::RollingResistanceSpeedLaw::FourthPower;
    std::string brake_channel_;
    double mass_kg_ = 0.0;
    std::string towing_point_, towed_point_, class_name_;
    double break_force_n_ = 0.0;
    HitchOverload overload_;
    std::string last_message_;
    std::uint64_t serial_ = 0, attaches_ = 0, detaches_ = 0, overloads_ = 0, removed_ = 0, teleports_ = 0;
    // relocation bookkeeping
    bool relocating_ = false, relocating_was_coupled_ = false;
    ps::Pose saved_pose_{};
};

// ---- the advice lines ------------------------------------------------------------------------------------------------

// The hitch ratings of a live coupling as configured advice lines (World::hitch_limits through the config's policy).
[[nodiscard]] std::vector<TrailerFinding> tow_findings(const TrailerConfig& config, const std::vector<ps::drivetrain::LimitFinding>& raw,
                                                      const std::string& car_name, const std::string& trailer_title);

// ---- persistence and the server rule ---------------------------------------------------------------------------------

// <user dir>/trailer.json (rg.trailer_state/1): the trailer the player had when the world was left. Restored by asking the
// next Session to spawn it (coupled when it was coupled).
struct TrailerRecord {
    bool present = false;
    std::string kind, load;
    bool coupled = false;
};
[[nodiscard]] std::string trailer_record_to_json(const TrailerRecord& record);
[[nodiscard]] std::optional<TrailerRecord> parse_trailer_record(const std::string& json_text, std::string* err);
bool save_trailer_record(const std::string& path, const TrailerRecord& record, std::string* err);
[[nodiscard]] std::optional<TrailerRecord> load_trailer_record(const std::string& path, std::string* err);

// One authority per coupled set. A set is a towing vehicle and everything coupled behind it; its simulation belongs to the
// peer that owns the TOWING vehicle (the trailer has no driver and no state of its own), so a second peer cannot hook on to a
// trailer that is already part of someone's set, and a trailer is released when its set's owner leaves. The server does not
// replicate players yet ("player_replication": false in the hello); this is the rule it will apply.
class CoupledSetAuthority {
public:
    using PeerId = std::uint64_t;
    using VehicleKey = std::uint64_t;
    // Registers a vehicle as owned by `peer`. False when the key is already owned by another peer.
    bool register_vehicle(VehicleKey vehicle, PeerId peer);
    // `peer` asks to couple `trailer` behind `towing`: allowed when the peer owns the towing vehicle and the trailer is
    // unowned or the peer's own and not yet coupled. On success the trailer joins the towing vehicle's set.
    bool couple(VehicleKey towing, VehicleKey trailer, PeerId peer);
    bool uncouple(VehicleKey trailer, PeerId peer);
    // The peer that simulates this vehicle: the owner of its set's root. 0 = unknown.
    [[nodiscard]] PeerId authority(VehicleKey vehicle) const;
    // A peer left: its vehicles and the trailers coupled behind them are released (returned).
    std::vector<VehicleKey> release_peer(PeerId peer);
    [[nodiscard]] std::size_t vehicle_count() const { return owner_.size(); }

private:
    std::map<VehicleKey, PeerId> owner_;
    std::map<VehicleKey, VehicleKey> towed_by_; // trailer -> towing vehicle
    [[nodiscard]] VehicleKey root_of(VehicleKey v) const;
};

} // namespace rg
