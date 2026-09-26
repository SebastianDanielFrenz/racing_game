// rg/session.h — rg::Session: one ps::World, its ground (a flat box, or
// streamed geo2map terrain), one car_sedan-shaped vehicle. Engine-neutral
// (PLAN.md 11.1 / user memory "Engine-neutral logic": no Godot type appears
// anywhere in this header or its .cpp) - godot_ext/ wraps this in a thin
// godot::Node, the reverse direction never happens, and a future UE5 port
// would wrap the same rg_core library instead of touching this file.
//
// Reuses Godot-free headers from physics_sim's own adapters/godot/src BY
// PATH (see CLAUDE.md's "Reused from physics_sim" note for why: they are
// already engine-neutral, so copying them would just create a second
// definition to keep in sync with a read-only upstream): triple_buffer.h
// (TripleBuffer<T>, lock-free single-writer/reader snapshot hand-off),
// origin_rebase.h (OriginRebase, floating-origin bookkeeping),
// fall_detector.h (FallDetector, terrain mode's fall diagnostic - included by
// session.cpp only) and frame_convert_core.h/.cpp (ISO<->Godot basis
// conversion math - used by godot_ext, not by this header directly, but
// built into rg_core so godot_ext's own thin wrapper has it available).
// sim_thread.h is no longer used by Session itself (R2.2 R4 replaced
// ps_godot::SimThread with rg::FixedRateLoop, which reuses only its
// precise_sleep_until). All live in namespace ps_godot (their own,
// physics_sim-side namespace) - kept as-is rather than renamed, since
// renaming a byte-for-byte reused file would only make future diffs against
// physics_sim harder to read.
//
// Two modes (R2.2 R4):
//  - flat (SessionConfig::terrain empty): one large static ground box, the
//    pre-R4 behaviour, bit-identical;
//  - terrain (SessionConfig::terrain set): no ground box; the World's terrain
//    is a g2m::ps_bridge::G2mTerrainSource over a ResidentHeightSet that a
//    g2m::phys::PhysicsTerrainStreamer fills from TerrainModeConfig::fetch,
//    and every tick attempt passes the clock-freeze gate first (try_step()).
#pragma once

#include "rg/drive_script.h"
#include "rg/fixed_rate_loop.h"
#include "rg/terrain_mode.h"

#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/surface_table.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/vehicle/wheel_state.h"
#include "ps/world/ids.h"
#include "ps/world/world.h"

#include "origin_rebase.h"
#include "triple_buffer.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace g2m::ps_bridge {
class G2mTerrainSource;
}
namespace g2m::phys {
struct InterestPoint;
}

namespace rg {

struct SessionConfig {
    double tick_rate_hz = 240.0;
    double substep_rate_hz = 960.0;
    ps::Vec3 gravity{0.0, 0.0, -9.81};
    unsigned job_workers = 0; // 0 = ps::World's own auto (WorldConfig::job_workers)

    // Ground (flat mode only): one large flat static box, mirrors
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
    // Terrain mode: chassis_z_m is the height above the highest of the five
    // spawn ray hits (plus physics.spawn_clearance_m); x/y/yaw come from
    // TerrainModeConfig.
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

    // Terrain mode (R2.2 R4) when set; flat mode otherwise.
    std::optional<TerrainModeConfig> terrain;
};

struct WheelSnapshot {
    std::string name;
    ps::vehicle::WheelState state{};
};

// One tick's worth of everything godot_ext/the HUD/the chase cam need,
// captured in one place right after World::step() returns (PLAN.md D3: "the
// adapter runs the sim on its own thread and reads interpolated snapshots
// via a lock-free triple buffer") - published into snapshot_buffer_ after
// every stepped tick of the real-time loop, read via Session::snapshot()
// from any thread.
struct FrameSnapshot {
    std::uint64_t tick = 0;
    double sim_time = 0.0;
    ps::Pose chassis_pose{};
    ps::Motion chassis_motion{};
    std::vector<WheelSnapshot> wheels;
    ps::drivetrain::PowertrainSnapshot powertrain{};
};

// Terrain streaming state (R2.2 R4), plain values. Each field is its own
// relaxed atomic inside Session, so a reader on another thread sees every
// field at most one tick attempt stale, but the fields are not one
// consistent cut. All zero/false in flat mode.
struct StreamingStatus {
    bool terrain_mode = false;
    bool ready = false;  // the last gate check passed (every gate L0 key Resident or Absent)
    bool frozen = false; // the last tick attempt was held back by the gate

    // The streamer's GateStatus as of the last gate check.
    std::uint32_t missing_required = 0;
    std::uint32_t inflight = 0;
    std::uint32_t resident_l0 = 0;
    std::uint32_t failed = 0;

    // Since the vehicle was spawned: tick attempts the gate held back, and
    // freeze episodes (maximal runs of held-back attempts).
    std::uint64_t frozen_attempts = 0;
    std::uint64_t freeze_count = 0;

    // G2mTerrainSource counters (fill_misses must stay 0 - the determinism
    // witness) and the FallDetector's fall events.
    std::uint64_t fill_misses = 0;
    std::uint64_t nodata_fills = 0;
    std::uint64_t falls = 0;

    // ps::World's TileManager, after the last stepped tick.
    std::uint64_t resident_tiles = 0;
    std::uint64_t starved_tiles = 0;
    std::uint64_t relief_overflow = 0;

    // Start-up: wall time the constructor waited for the first ready gate,
    // and how many priming ticks it stepped before spawning.
    double startup_ms = 0.0;
    std::uint32_t prime_ticks = 0;
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
    // Terrain mode: blocks until the gate around the spawn point is ready
    // (std::runtime_error after physics.startup_timeout_s), steps the
    // priming ticks with no vehicle (std::runtime_error if the TileManager
    // then holds fewer than its square's tiles, any starved tile or any fill
    // miss), ray-casts the spawn height (std::runtime_error "spawn over
    // NoData" when any of the five rays misses) and only then creates the
    // chassis and vehicle.
    explicit Session(const SessionConfig& config);
    ~Session();
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    // --- Synchronous stepping: no thread, caller drives every tick. Used
    // by tools/hash_check (deterministic, wall-clock-free) and rg_core's
    // own unit tests. Must not be called while running(). Controls come
    // from the drive script if one is set, otherwise from whatever the
    // caller wrote into world() directly - the set_control atomics are NOT
    // copied here (tools/hash_check depends on that). Terrain mode: retries
    // the gate (1 ms sleeps) until the tick steps; std::runtime_error if it
    // stays frozen for physics.startup_timeout_s.
    void step();

    // One tick attempt through the gate, on the calling thread (the loop
    // thread while running()): terrain mode builds the physics interest
    // points (R4: just the chassis pose/velocity - sim state), updates the
    // streamer and TileManager's interest points, and returns false - World untouched -
    // while the gate is not ready. Otherwise applies the drive script (or,
    // without one, the set_control atomics), steps the World, runs the
    // post-step work (snapshot, fall detector, StreamingStatus) and returns
    // true. Flat mode always steps. Must not be called while running().
    bool try_step();

    // --- Real-time stepping: an rg::FixedRateLoop calls try_step()'s body
    // on its own thread at config.tick_rate_hz (a frozen attempt resyncs the
    // deadline - no catch-up burst afterwards); each stepped tick's result
    // is published into a ps_godot::TripleBuffer, read back race-free via
    // snapshot(). Mirrors physics_sim's own ps_simulation.h pattern
    // (PLAN.md D3).
    void start();
    void stop();
    [[nodiscard]] bool running() const { return loop_.running(); }
    [[nodiscard]] FixedRateLoop::LoopStats loop_stats() const { return loop_.stats(); }

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
    // on the loop thread right before each stepped tick when no drive
    // script is set (never called directly from another thread -
    // World::set_control/get_control have no synchronisation of their own).
    void set_control(const std::string& channel, double value);
    [[nodiscard]] double get_control(const std::string& channel) const;

    // Replaces the drive script (rewound: every event already due at the
    // current drive tick is applied on the next stepped tick). While a
    // script is set it is the only control source of try_step()/step() and
    // of the real-time loop. Both throw std::logic_error while running().
    void set_drive_script(DriveScript script);
    void clear_drive_script();

    // Stepped World ticks since the vehicle was spawned (the drive script's
    // tick index), and the World tick the vehicle was spawned at (0 in flat
    // mode, the priming tick count in terrain mode).
    [[nodiscard]] std::uint64_t drive_tick() const { return world_->tick() - spawn_tick_; }
    [[nodiscard]] std::uint64_t spawn_tick() const { return spawn_tick_; }

    // --- Terrain mode (defaults / null in flat mode).
    [[nodiscard]] bool terrain_mode() const { return terrain_ != nullptr; }
    [[nodiscard]] StreamingStatus streaming_status() const;
    // Its counters are atomics (any thread); fill_tile is for the sim
    // thread / tests only.
    [[nodiscard]] const g2m::ps_bridge::G2mTerrainSource* terrain_source() const;
    // Asks the streamer to forget its remembered Failed keys at the next
    // gate check, so they are fetched again (e.g. once a 503 storm is
    // over). Any thread; consumed on the thread that steps.
    void retry_failed_tiles() { retry_failed_requested_.store(true, std::memory_order_relaxed); }

    // --- Direct access: synchronous-mode / test / hash-check-tool only.
    // NOT race-free against a running loop - never call these while
    // running() from a thread other than the one driving step().
    [[nodiscard]] ps::World& world() { return *world_; }
    [[nodiscard]] const ps::World& world() const { return *world_; }
    [[nodiscard]] ps::BodyId ground_body() const { return ground_body_; } // invalid in terrain mode
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
    struct Terrain; // session.cpp: grid, resident set, loader, streamer, source, fall detector

    [[nodiscard]] static std::unique_ptr<ps::World> make_world(const SessionConfig& config);
    void build_world_contents(const SessionConfig& config);
    // Terrain mode start-up (source, start-up block, priming); returns the
    // chassis spawn pose from the five spawn rays.
    [[nodiscard]] ps::Pose setup_terrain(const TerrainModeConfig& terrain);
    // One tick attempt; `from_loop` = called by the real-time loop (copies
    // the atomics when no script is set, publishes the snapshot).
    bool step_once(bool from_loop);
    // Terrain gate check (no stepping): streamer update + TileManager
    // interest point + status atomics; true = ready.
    bool gate_check();
    // The physics interest points the streamer and TileManager keep terrain
    // resident around - a LIST so later actors (walker, parked cars) slot in;
    // for R4 exactly one entry, the chassis. Sim thread only; the span points
    // into Terrain's reused buffer (valid until the next call).
    std::span<const g2m::phys::InterestPoint> physics_interest_points();
    // Retries step_once(false) until it steps; throws after the stall timeout.
    void step_blocking(const char* what);
    void post_step(bool from_loop);
    [[nodiscard]] FrameSnapshot capture_frame_snapshot() const;

    SessionConfig config_;
    std::shared_ptr<ps::io::SurfaceTable> surface_table_;
    std::unique_ptr<ps::World> world_;
    ps::BodyId ground_body_{};
    ps::BodyId chassis_body_{};
    ps::VehicleId vehicle_id_{};
    bool have_vehicle_ = false;
    std::uint64_t spawn_tick_ = 0;
    ps::vehicle::VehicleDesc vehicle_desc_;

    std::unique_ptr<Terrain> terrain_; // null in flat mode
    std::optional<DriveScript> drive_script_;
    std::atomic<bool> retry_failed_requested_{false};

    // StreamingStatus, field by field (see StreamingStatus's comment).
    struct StatusAtomics {
        std::atomic<bool> ready{false};
        std::atomic<bool> frozen{false};
        std::atomic<std::uint32_t> missing_required{0};
        std::atomic<std::uint32_t> inflight{0};
        std::atomic<std::uint32_t> resident_l0{0};
        std::atomic<std::uint32_t> failed{0};
        std::atomic<std::uint64_t> frozen_attempts{0};
        std::atomic<std::uint64_t> freeze_count{0};
        std::atomic<std::uint64_t> fill_misses{0};
        std::atomic<std::uint64_t> nodata_fills{0};
        std::atomic<std::uint64_t> falls{0};
        std::atomic<std::uint64_t> resident_tiles{0};
        std::atomic<std::uint64_t> starved_tiles{0};
        std::atomic<std::uint64_t> relief_overflow{0};
        std::atomic<double> startup_ms{0.0};
        std::atomic<std::uint32_t> prime_ticks{0};
    };
    StatusAtomics status_;
    bool in_freeze_ = false; // stepping thread only

    ps_godot::TripleBuffer<FrameSnapshot> snapshot_buffer_;
    ps_godot::OriginRebase origin_rebase_;

    std::unordered_map<std::string, std::atomic<double>> control_channels_;

    // Its thread runs step_once over every member above; ~Session() stops it
    // before anything is destroyed.
    FixedRateLoop loop_;
};

} // namespace rg
