// rg/session.h — rg::Session: one ps::World, one ground body, one
// car_sedan-shaped vehicle. Engine-neutral (PLAN.md 11.1 / user memory
// "Engine-neutral logic": no Godot type appears anywhere in this header or
// its .cpp) - godot_ext/ wraps this in a thin godot::Node, the reverse
// direction never happens, and a future UE5 port would wrap the same
// rg_core library instead of touching this file.
//
// Reuses four Godot-free headers from physics_sim's own adapters/godot/src
// BY PATH (see CLAUDE.md's "Reused from physics_sim" note for why: they are
// already engine-neutral, so copying them would just create a second
// definition to keep in sync with a read-only upstream): sim_thread.h
// (SimThread, fixed-rate wall-clock stepping), triple_buffer.h
// (TripleBuffer<T>, lock-free single-writer/reader snapshot hand-off),
// origin_rebase.h (OriginRebase, floating-origin bookkeeping) and
// frame_convert_core.h/.cpp (ISO<->Godot basis conversion math - used by
// godot_ext, not by this header directly, but built into rg_core so
// godot_ext's own thin wrapper has it available). All four live in
// namespace ps_godot (their own, physics_sim-side namespace) - kept as-is
// rather than renamed, since renaming a byte-for-byte reused file would
// only make future diffs against physics_sim harder to read.

#pragma once

#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/surface_table.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/vehicle/wheel_state.h"
#include "ps/world/ids.h"
#include "ps/world/world.h"

#include "origin_rebase.h"
#include "sim_thread.h"
#include "triple_buffer.h"

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rg {

struct SessionConfig {
    double tick_rate_hz = 240.0;
    double substep_rate_hz = 960.0;
    ps::Vec3 gravity{0.0, 0.0, -9.81};
    unsigned job_workers = 0; // 0 = ps::World's own auto (WorldConfig::job_workers)

    // Ground: one large flat static box, mirrors
    // external/physics_sim/data/scenarios/vehicle_step_steer.json's own
    // "ground" body (half_extents [1000,1000,0.5], position z=-0.5, so its
    // top face sits at world z=0) - see CLAUDE.md's hash-check section.
    ps::real ground_half_extent_m = 1000.0;
    ps::real ground_friction = 1.0;

    // Chassis: mirrors the same scenario's "chassis" body exactly (same
    // mass/half_extents/z/friction/allow_sleep) so the SAME construction
    // code below (Session::build_world) can be driven either by the real
    // game (chassis_initial_velocity left at rest) or by tools/hash_check
    // (chassis_initial_velocity set to {20,0,0} to match the scenario) -
    // see CLAUDE.md for exactly what the hash-check tool compares and why.
    ps::real chassis_mass_kg = 1500.0;
    ps::Vec3 chassis_half_extents{2.0, 0.4, 0.15};
    ps::real chassis_z_m = 0.6;
    ps::real chassis_friction = 0.5;
    ps::Vec3 chassis_initial_velocity{0.0, 0.0, 0.0};

    // Resolved (absolute, or relative to the process's CWD) paths - the
    // caller resolves these (godot_ext relative to res://, tools/hash_check
    // relative to the submodule's own data/ directory per its own CLI arg)
    // so this header stays free of any Godot resource-path convention.
    std::string vehicle_json_path;  // e.g. .../data/vehicles/car_sedan.json
    std::string surface_table_path; // e.g. .../data/surfaces/surfaces.json
};

struct WheelSnapshot {
    std::string name;
    ps::vehicle::WheelState state{};
};

// One tick's worth of everything godot_ext/the HUD/the chase cam need,
// captured in one place right after World::step() returns (PLAN.md D3: "the
// adapter runs the sim on its own thread and reads interpolated snapshots
// via a lock-free triple buffer") - published into snapshot_buffer_ by the
// SimThread's post_step hook, read via Session::snapshot() from any thread.
struct FrameSnapshot {
    std::uint64_t tick = 0;
    double sim_time = 0.0;
    ps::Pose chassis_pose{};
    ps::Motion chassis_motion{};
    std::vector<WheelSnapshot> wheels;
    ps::drivetrain::PowertrainSnapshot powertrain{};
};

// The full list of control channels Session pre-seeds at construction (see
// session.cpp) - mirrors physics_sim's adapters/godot/demo/scripts/main.gd's
// own channel set. set_control on any other name is a documented no-op
// (cheaper and safer than silently growing the map from a worker thread -
// see set_control's own doc comment below for the concurrency reason this
// matters).
extern const char* const kControlChannelNames[];
extern const std::size_t kControlChannelCount;

class Session {
public:
    explicit Session(const SessionConfig& config);
    ~Session();
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    // --- Synchronous stepping: no thread, caller drives every tick. Used
    // by tools/hash_check (deterministic, wall-clock-free) and rg_core's
    // own unit tests. Must not be called while running().
    void step();

    // --- Real-time stepping: a ps_godot::SimThread steps world() on its
    // own thread at config.tick_rate_hz; each tick's result is published
    // into a ps_godot::TripleBuffer, read back race-free via snapshot().
    // Mirrors physics_sim's own ps_simulation.h pattern (PLAN.md D3).
    void start();
    void stop();
    [[nodiscard]] bool running() const { return sim_thread_.running(); }

    // Race-free once start() has produced at least one tick; before that,
    // returns a default-constructed FrameSnapshot (tick == 0).
    [[nodiscard]] const FrameSnapshot& snapshot() { return snapshot_buffer_.read(); }

    // --- Named control channels (PLAN.md P2: "input reaches the sim only
    // as named channels, in ISO/SI terms"). Safe to call from any thread
    // while running(): channel keys are pre-seeded once, single-threaded,
    // in the constructor (before start() can run), and each value is a
    // std::atomic<double> - the SAME concurrency pattern physics_sim's own
    // ps_simulation.h::set_control uses and documents (see that file's doc
    // comment for the full reasoning), copied into ps::World::set_control
    // from inside the SimThread's pre_step hook every tick (never called
    // directly from another thread - World::set_control/get_control have no
    // synchronisation of their own).
    void set_control(const std::string& channel, double value);
    [[nodiscard]] double get_control(const std::string& channel) const;

    // --- Direct access: synchronous-mode / test / hash-check-tool only.
    // NOT race-free against a running SimThread - never call these while
    // running() from a thread other than the one driving step().
    [[nodiscard]] ps::World& world() { return *world_; }
    [[nodiscard]] const ps::World& world() const { return *world_; }
    [[nodiscard]] ps::BodyId ground_body() const { return ground_body_; }
    [[nodiscard]] ps::BodyId chassis_body() const { return chassis_body_; }
    [[nodiscard]] ps::VehicleId vehicle_id() const { return vehicle_id_; }

    // The loaded VehicleDesc (kept around after create_vehicle so a caller
    // can read static gauge info - idle/limiter rpm, gear count - without
    // re-parsing config.vehicle_json_path itself; mirrors physics_sim's own
    // ps_simulation.h's vehicle_desc_by_name_ map, trimmed to one vehicle).
    [[nodiscard]] const ps::vehicle::VehicleDesc& vehicle_desc() const { return vehicle_desc_; }
    [[nodiscard]] const ps::io::SurfaceTable& surface_table() const { return *surface_table_; }

    // Owned by Session so godot_ext's chase-camera wrapper has somewhere to
    // keep it (PLAN.md 11.3: "only the active camera rebases") without
    // needing its own copy of the threshold/state logic - the camera script
    // still decides WHEN to call update(), Session just holds the object.
    [[nodiscard]] ps_godot::OriginRebase& origin_rebase() { return origin_rebase_; }

private:
    // Builds just the ps::World object itself (WorldConfig from config) -
    // split out as a static helper because ps_godot::SimThread's
    // constructor takes a ps::World& and must be initialised in THIS
    // class's member-initializer-list (it has no default constructor), so
    // world_ must already exist by the time sim_thread_ is constructed.
    // Everything else build_world() needs to do (ground/chassis bodies, the
    // vehicle, the surface table, the control-channel map) happens in the
    // constructor BODY instead, after every member below is initialised.
    [[nodiscard]] static std::unique_ptr<ps::World> make_world(const SessionConfig& config);
    void build_world_contents(const SessionConfig& config);
    [[nodiscard]] FrameSnapshot capture_frame_snapshot() const;

    SessionConfig config_;
    std::shared_ptr<ps::io::SurfaceTable> surface_table_;
    std::unique_ptr<ps::World> world_;
    ps::BodyId ground_body_{};
    ps::BodyId chassis_body_{};
    ps::VehicleId vehicle_id_{};
    ps::vehicle::VehicleDesc vehicle_desc_;

    // Constructed from *world_, so must be declared (and therefore
    // initialised - C++ member order follows declaration order regardless
    // of initializer-list order) after world_ above.
    ps_godot::SimThread sim_thread_;
    ps_godot::TripleBuffer<FrameSnapshot> snapshot_buffer_;
    ps_godot::OriginRebase origin_rebase_;

    std::unordered_map<std::string, std::atomic<double>> control_channels_;
};

} // namespace rg
