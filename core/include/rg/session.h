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
#include <set>
#include <tuple>

#include "g2m/layer/osm_roads.h"

#include "rg/deck_installer.h"
#include "rg/drive_script.h"
#include "rg/environment.h"
#include "rg/npc_truck.h"
#include "rg/npc_traffic.h"
#include "rg/traffic_stuck.h"
#include <thread>
#include "rg/fixed_rate_loop.h"
#include "rg/player_mode.h"
#include "rg/road_ahead.h"
#include "rg/terrain_mode.h"
#include "rg/walker.h"

#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/surface_table.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/vehicle/wheel_state.h"
#include "ps/world/ids.h"
#include "ps/world/world.h"

#include "origin_rebase.h"
#include "render_interp.h"
#include "triple_buffer.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <functional>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace g2m::ps_bridge {
class G2mTerrainSource;
}
namespace g2m::phys {
struct InterestPoint;
}

namespace rg {

// Terrain-mode start-up progress and cancellation (R2.2 R9), shared between
// the thread constructing a Session (which may block in setup_terrain for up
// to physics.startup_timeout_s) and any other thread: the constructor
// mirrors the gate's figures into it after every gate check, and polls
// `cancel` in its start-up and priming loops - a set flag makes the
// constructor throw SessionCancelled within about one gate check (1 ms
// sleeps), instead of running to completion or the timeout. Optional
// (SessionConfig::startup null = no progress, no cancellation, the pre-R9
// behaviour); never read by the simulation itself, so it cannot move a
// state hash.
struct StartupProgress {
    enum Stage : int { NotStarted = 0, WaitingForGate = 1, Priming = 2, Spawning = 3, Done = 4 };
    std::atomic<int> stage{NotStarted};
    std::atomic<std::uint32_t> missing_required{0};
    // Largest missing_required seen while waiting for the gate: the gate's
    // full key set at its first update, so (gate_total - missing_required) /
    // gate_total is a real loading fraction (the loading screen's bar).
    std::atomic<std::uint32_t> gate_total{0};
    std::atomic<std::uint32_t> inflight{0};
    std::atomic<std::uint32_t> resident_l0{0};
    std::atomic<std::uint32_t> failed{0};
    std::atomic<std::uint32_t> prime_done{0};
    std::atomic<std::uint32_t> prime_total{0};
    std::atomic<bool> cancel{false};
};

// Thrown by Session's constructor when StartupProgress::cancel was set.
class SessionCancelled : public std::runtime_error {
public:
    SessionCancelled() : std::runtime_error("Session: terrain start-up cancelled") {}
};

struct SessionConfig {
    std::string rolling_resistance_model = "quadratic";
    std::string aero_map_selection_path; // startup-only game-owned CFD map selection

    double tick_rate_hz = 240.0;
    double substep_rate_hz = 960.0;
    ps::Vec3 gravity{0.0, 0.0, -9.81};
    EnvironmentConfig environment;
    unsigned job_workers = 0; // 0 = ps::World's own auto (WorldConfig::job_workers)
    // Reference/measurement switch (S1): install road decks the pre-S1 way - the mesh shape built on the sim thread inside
    // create_body(desc), one deck per tick attempt, only decks in range. false = DeckInstaller (shapes built on own
    // threads, prefetched, strict key-order install).
    bool legacy_deck_install = false;

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
    // Optional immutable, already-loaded definition for rebuilding a world.
    // Engine maps remain shared/const; each world creates fresh runtime state.
    std::shared_ptr<const ps::vehicle::VehicleDesc> vehicle_definition;
    bool engine_map_cache_enabled = true; // Persistent simulated-engine maps, matching the physics demo.
    // Non-empty: the .psmaps cache directory to use (R6: a materialised garage setup lives outside
    // physics_sim's tree, so the path-derived default would put its cache in the work directory).
    std::string engine_map_cache_dir;
    std::string vehicle_json_path;  // e.g. .../data/vehicles/car_sedan.json
    std::string surface_table_path; // e.g. .../data/surfaces/surfaces.json

    // On foot (R9c): the walker's size/speeds (rg/walker.h). Only used once a
    // walker is spawned (request_walker_spawn).
    WalkerConfig walker;

    // Terrain mode (R2.2 R4) when set; flat mode otherwise.
    std::optional<TerrainModeConfig> terrain;

    // Start-up progress/cancellation (R2.2 R9, terrain mode only; see
    // StartupProgress). Null = none.
    std::shared_ptr<StartupProgress> startup;
};

struct WheelSnapshot {
    std::string name;
    ps::vehicle::WheelState state{};
    ps::vehicle::WheelTelemetry telemetry{};
};

// One tick's worth of everything godot_ext/the HUD/the chase cam need,
// captured in one place right after World::step() returns (PLAN.md D3: "the
// adapter runs the sim on its own thread and reads interpolated snapshots
// via a lock-free triple buffer") - published into snapshot_buffer_ after
// every stepped tick of the real-time loop, read via Session::snapshot()
// from any thread.
struct AeroSurfaceSnapshot {
    std::string name;
    double alpha_rad=0,beta_rad=0,cl=0,cd=0,clearance_m=-1,ground_multiplier=1,dynamic_pressure_pa=0;
    ps::Vec3 force_world{};
};
struct AeroSnapshot {
    bool coefficient_map_active=false,coefficient_map_ground_missing=false;
    std::uint32_t coefficient_map_clamped_axes=0;
    std::array<double,5> coefficient_map_inputs{};
    std::array<double,6> coefficient_map_coefficients{};
    bool enabled=false;
    double airspeed_m_s=0,drag_n=0,downforce_n=0,side_force_n=0;
    double front_balance=0,wing_pitch_offset_deg=0,wing_lift_m=0,fan_power_w=0,wake_factor=1;
    ps::Vec3 force_world{},moment_world{};
    double fan_energy_remaining_j=0;
    std::vector<AeroSurfaceSnapshot> surfaces;
    EnvironmentSample environment;
};

// The on-foot player (R9c), captured with the rest of the tick. `active` false
// = no walker (every other field is then zero/false). Positions are session
// metres (ISO, z up); feet = the capsule bottom.
struct WalkerSnapshot {
    bool active = false;
    ps::Vec3 feet{};
    ps::Vec3 velocity{};
    double yaw_rad = 0.0;      // facing
    bool grounded = false;
    bool blocked = false;      // pressing against an obstacle
    bool hold = false;         // no ground loaded below: holding the height
    bool can_enter = false;    // within get-in range of the own car (the HUD prompt)
    double enter_distance_m = 0.0; // to the car's footprint outline
    // The own car's footprint rectangle the walker is blocked by (display/debug).
    double car_x = 0.0, car_y = 0.0, car_half_x = 0.0, car_half_y = 0.0, car_yaw_rad = 0.0;
};

struct FrameSnapshot {
    std::uint64_t tick = 0;
    double sim_time = 0.0;
    ps::Pose chassis_pose{};
    ps::Motion chassis_motion{};
    std::vector<WheelSnapshot> wheels;
    ps::drivetrain::PowertrainSnapshot powertrain{};
    AeroSnapshot aero;
    TruckSnapshot truck;
    TrafficSnapshot traffic;
    g2m::RoadSpeedMatch road_speed_limit{};
    WalkerSnapshot walker;
    // The road under and ahead of the car (session coordinates, first point
    // = the car's projection onto its road), refreshed ~20 Hz, only while
    // set_road_ahead_wanted(true). Empty when not wanted, in the flat world,
    // or when the car is not on a road. Feeds the cinematic camera.
    std::vector<RoadPoint> road_ahead;
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
    ps::terrain::PrefetchStats prefetch; // lifetime counters after last stepped tick

    // Start-up: wall time the constructor waited for the first ready gate,
    // and how many priming ticks it stepped before spawning.
    double startup_ms = 0.0;
    std::uint32_t prime_ticks = 0;
    std::uint64_t relocations = 0;       // request_relocate placements done (R9)
    std::uint64_t relocate_failures = 0; // relocations with no ground at the target

    // Drone follow (R9b, Session::set_followed_vehicle): the vehicle currently
    // followed (0 = none), how many times the Session LOST a followed vehicle
    // (despawned, truck removed, traffic cleared - lifetime counter, check
    // `after > before`) and the id of the latest one lost. The Session clears
    // the follow itself when it loses the vehicle; the binding reacts by
    // returning to the player's own car. Valid in flat mode too.
    std::uint64_t followed_id = 0;
    std::uint64_t followed_lost = 0;
    std::uint64_t followed_lost_id = 0;
    // Physics interest points of the last gate check (terrain mode; 1 = the
    // player's car, 2 = plus the followed vehicle).
    std::uint32_t interest_points = 0;

    // G2.5a-grip R-c (plan section 8): whether tyre grip comes from per-cell
    // OSM road classes (physics.road_surfaces.enabled) rather than one
    // uniform terrain_surface. False in flat mode and whenever road_surfaces
    // is disabled. osm_ok/osm_fail mirror WorldTerrain::OsmFetchStats (0
    // when this Session's TerrainModeConfig has no WorldTerrain, e.g. a
    // synthetic test session).
    bool road_surfaces = false;
    std::uint64_t osm_ok = 0;
    std::uint64_t osm_fail = 0;
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
    // Pause (the shell's pause menu): while paused every loop attempt is held
    // back exactly like a terrain-gate freeze - World untouched, the deadline
    // resynced to now, so resuming never replays a backlog (no catch-up
    // burst). The published snapshot stays the last stepped tick. Only the
    // real-time loop honours it; step()/try_step() (hash_check, tests) ignore
    // it. Any thread. Counted by the loop as frozen attempts, not by
    // StreamingStatus::freeze_count (that counts terrain-gate episodes only).
    void set_paused(bool paused) { paused_.store(paused, std::memory_order_relaxed); }
    [[nodiscard]] bool paused() const { return paused_.load(std::memory_order_relaxed); }
    // The cinematic camera wants FrameSnapshot::road_ahead. Off by default so a
    // session that never asks does no road tracing and allocates nothing.
    void set_road_ahead_wanted(bool wanted) { road_ahead_wanted_.store(wanted, std::memory_order_relaxed); }
    [[nodiscard]] FixedRateLoop::LoopStats loop_stats() const { return loop_.stats(); }

    // --- Tick-spike diagnostics (owner drive 2026-09-27: ~170 ms sim-thread
    // stalls every ~255 m at 150-185 km/h that a headless replay does not
    // reproduce). Every real-time loop attempt is timed per phase; one that
    // takes longer than kTickSpikeAttemptMs, or starts more than
    // kTickSpikeGapMs after the previous attempt started, is queued here
    // (bounded - overflow is counted, never blocks the loop). gap_ms minus
    // the previous attempt's own total is time the loop thread spent NOT in
    // step_once (sleep overshoot, descheduling). Drained from any thread.
    static constexpr double kTickSpikeAttemptMs = 8.0;
    static constexpr double kTickSpikeGapMs = 20.0;
    static constexpr std::size_t kTickSpikeCapacity = 256;
    enum class TickSpikeKind : int { Stepped = 0, Frozen = 1, Relocation = 2 };
    struct TickSpike {
        std::uint64_t tick = 0;       // World tick after the attempt
        double wall_s = 0.0;          // since start()
        TickSpikeKind kind = TickSpikeKind::Stepped;
        double gap_ms = 0.0;          // previous attempt start -> this attempt start
        double prev_total_ms = 0.0;   // previous attempt's own duration
        double total_ms = 0.0;        // this attempt
        double take_relocate_ms = 0.0;
        double gate_update_ms = 0.0;  // PhysicsStreamer::update
        double gate_interest_ms = 0.0; // World::set_terrain_interest_point
        double controls_ms = 0.0;     // drive script / live controls
        double step_ms = 0.0;         // World::step (or the relocation's priming ticks)
        double post_ms = 0.0;         // snapshot publish, fall detector, status
        double x = 0.0, y = 0.0, speed_mps = 0.0;
        std::uint64_t resident_tiles = 0;
        ps::terrain::PrefetchStats prefetch;
    };
    // Returns (and clears) the queued spikes, oldest first; *overflow gets
    // the number dropped since the previous drain.
    [[nodiscard]] std::vector<TickSpike> drain_tick_spikes(std::uint64_t* overflow = nullptr);
    // Road-deck installer counters (S1): sim-thread install time, worker build time, waits. Stepping thread, or any
    // thread while the session is not stepping. Zeros without a deck installer (flat mode, no road decks).
    [[nodiscard]] DeckInstallStats deck_install_stats() const { return deck_installer_ ? deck_installer_->stats() : DeckInstallStats{}; }
    // One log line (no prefix, no newline), key=value tokens.
    [[nodiscard]] static std::string format_tick_spike(const TickSpike& spike);

    // Race-free once start() has produced at least one tick; before that,
    // returns a default-constructed FrameSnapshot (tick == 0).
    [[nodiscard]] const FrameSnapshot& snapshot() { return snapshot_buffer_.read(); }
    // Render-time pose sampling on the sim's NOMINAL clock (physics_sim 2d8f0b8,
    // render_interp.h): while the real-time loop runs, every stepped tick pushes
    // its chassis pose into an 8-frame history stamped with the tick's scheduled
    // time (FixedRateLoop::tick_nominal_time), not the wall time the step
    // finished, so a render frame samples a piecewise-linear function of the
    // SCHEDULE (the follow-camera shake came from publish-time jitter). Pose
    // layout of RenderSample::poses: [0] = the chassis. A relocation is flagged
    // per frame and rendered as a step, never a smear. Render thread; false (and
    // nothing valid in `out`) when the history is empty - not running, or no tick
    // yet - the caller then falls back to snapshot().chassis_pose. Sample at
    // ps_godot::render_time_for(now, tick_period_s(), delay_ticks) with a delay
    // of 2 ticks (covers the publish lateness).
    bool sample_render_poses(ps_godot::SimClock::time_point render_time, ps_godot::RenderSample& out) const { return pose_history_.sample(render_time, out); }
    [[nodiscard]] const ps_godot::PoseHistory& render_pose_history() const { return pose_history_; }
    [[nodiscard]] double tick_period_s() const { return 1.0 / config_.tick_rate_hz; }
    // Single engine-input publisher on the physics thread. Clearing waits for
    // an in-flight callback, so its owner can safely destroy the audio voice.
    using EngineAudioPublisher = std::function<void(const ps::drivetrain::EngineSoundState&, double)>;
    void set_engine_audio_publisher(EngineAudioPublisher publisher);


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

    // Who drives the car (R2.2 R9, rg/player_mode.h): Player (default) -
    // the real-time loop copies the set_control atomics as before;
    // Unattended - the loop overrides steer/throttle/brake/handbrake/clutch/
    // starter with rg::unattended_controls(chassis speed) and copies every
    // other channel. Only the real-time loop's control copy reads it (a
    // drive script or the synchronous step() are unaffected), so a Player
    // session's behaviour and hashes are exactly the pre-R9 ones. Any thread.
    void set_vehicle_control(VehicleControl control) {
        vehicle_control_.store(control, std::memory_order_relaxed);
    }
    [[nodiscard]] VehicleControl vehicle_control() const { return vehicle_control_.load(std::memory_order_relaxed); }

    // Relocation (R2.2 R9): move the car to session XY (x, y) facing yaw_rad
    // (about +Z, 0 = east), at rest in neutral. Any thread; consumed by the
    // next tick attempt on the stepping thread (a newer request replaces one
    // not yet consumed or still waiting for its terrain). Terrain mode: the
    // physics interest point moves to the target first, so the clock freezes
    // through the gate (a real freeze, counted in freeze_count) until the
    // target's tiles are resident; then one attempt steps the TileManager
    // priming ticks with the chassis parked at kRelocateParkZ, places it with
    // the spawn's five rays (chassis_z_m + spawn_clearance_m above the
    // highest hit) and resets the drivetrain (World::reset_vehicle: gear 0,
    // hubs stopped, engine left running unless off). A target with no ground
    // under it counts relocate_failures and leaves the chassis at the park
    // height over the target (it then falls: pick targets inside the data).
    // Flat mode: the same placement, no gate or priming. The game's "reset
    // car" key (request_reset_to_spawn) and the smoke test's forced freeze
    // use it; nothing calls it on its own, so hashes are unaffected.
    void request_relocate(double x, double y, double yaw_rad);
    void request_reset_to_spawn();
    void configure_traffic(TrafficConfig);
    void set_visible_traffic(std::vector<std::uint64_t> ids);

    // Drone follow (R9b): keep a vehicle other than the player's car alive and
    // its terrain resident while the camera trails it. `id` is a
    // FrameSnapshot::traffic.actors[].id or kNpcTruckVehicleId; nullopt (or 0)
    // ends the follow. Any thread, consumed on the stepping thread.
    //  - Terrain mode: physics_interest_points() appends a SECOND point (id
    //    kFollowedInterestId) at that actor's position/velocity with the same
    //    radius; the player's car stays point 0. The TileManager pool is sized
    //    for two points (setup_terrain doubles make_terrain_config's value).
    //  - The actor is not despawned for distance/surplus while followed (it
    //    still ends with its route), and the truck no longer stops at 200 m
    //    from the player's car.
    //  - If the actor disappears anyway (route end, truck removed, traffic
    //    cleared by a relocation, an id that never existed), the Session
    //    clears the follow and bumps StreamingStatus::followed_lost.
    // Never called by the Session itself, so a session without a follow behaves
    // (and hashes) exactly as before.
    void set_followed_vehicle(std::optional<std::uint64_t> id);
    [[nodiscard]] std::optional<std::uint64_t> followed_vehicle() const;
    // The same two counters as StreamingStatus::followed_lost/_lost_id, cheap
    // enough to poll every frame (two atomic loads, count first).
    struct FollowLoss {
        std::uint64_t count = 0;
        std::uint64_t id = 0;
    };
    [[nodiscard]] FollowLoss followed_loss() const;
    // The NPC truck's followable id (traffic actor ids count up from 1 and can
    // never reach 2^62; 2^62 also stays a positive int64 for Godot).
    static constexpr std::uint64_t kNpcTruckVehicleId = std::uint64_t{1} << 62;
    // TileManager/streamer interest-point ids: the player's car, the followed vehicle.
    static constexpr std::uint32_t kPlayerInterestId = 0;
    static constexpr std::uint32_t kFollowedInterestId = 1;
    // On foot: the second point is the parked car (never together with a followed vehicle).
    static constexpr std::uint32_t kParkedCarInterestId = 1;
    void request_npc_truck(bool enabled,double speed_kph); // the config's spawn (flat mode: the origin, yaw 0)

    // --- On foot (R9c, rg/walker.h). The walker is one kinematic capsule body
    // in the World, driven by a WalkerController on the stepping thread; all
    // requests below are any-thread and consumed by the next tick attempt that
    // passes the terrain gate (so a spawn waits for residency like the car).
    //  - request_walker_spawn(): get out - the walker appears beside the
    //    driver's door (select_spawn_spot; other sides, then above the roof, when
    //    blocked), facing away from the car. Terrain mode: the walker becomes
    //    physics interest point kPlayerInterestId and the PARKED CAR keeps its
    //    terrain resident as the second point (kParkedCarInterestId). Does
    //    nothing when a walker exists or there is no vehicle yet.
    //  - request_walker_enter(): get in (interact) - succeeds only within
    //    WalkerConfig::enter_range_m of the own car's footprint; the walker is
    //    removed (walker_counters().entered +1) or the attempt is counted
    //    (enter_refused +1, the walker stays).
    //  - request_walker_despawn(): remove the walker without any range check (the
    //    player left the mode some other way; counted as despawned).
    //  Newest request wins when several are pending. The car's driving stays
    //  Unattended (set_vehicle_control) for as long as the caller wants; the
    //  Session itself never changes it.
    void request_walker_spawn();
    void request_walker_enter();
    void request_walker_despawn();
    // Input for the NEXT ticks: move axes in [-1, 1] relative to the look
    // heading (WalkerInput), plus a one-shot jump (consumed by the next walker
    // step; honoured only while grounded). The synchronous step() applies it
    // too (unlike the driving channels).
    void set_walker_input(const WalkerInput& input);
    void request_walker_jump() { walker_jump_.fetch_add(1, std::memory_order_relaxed); }
    // Lifetime counters (any thread, a poller compares with its last values).
    struct WalkerCounters {
        std::uint64_t spawned = 0;       // walkers created
        std::uint64_t entered = 0;       // successful get-ins
        std::uint64_t enter_refused = 0; // interact pressed out of range
        std::uint64_t despawned = 0;     // forced removals
        std::uint64_t spawn_failed = 0;  // spawn requests that could not place a walker
    };
    [[nodiscard]] WalkerCounters walker_counters() const;
    [[nodiscard]] bool walker_active() const { return walker_active_.load(std::memory_order_acquire); }
    // Stepping thread / tests only (not while running()): the controller, null before the first spawn.
    [[nodiscard]] const WalkerController* walker_controller() const { return walker_.get(); }
    // The own car's footprint as of the latest walker update (stepping thread / tests only).
    [[nodiscard]] OrientedRect own_car_footprint() const;
    static constexpr double kRelocateParkZ = 4000.0;

    // Flip the car upright IN PLACE, keeping its current position and
    // heading (owner request: a car that rolled onto its roof/side/nose
    // should be put back on its wheels where it is, not sent back to spawn -
    // that is what request_reset_to_spawn is for). Any thread; consumed by
    // the next tick attempt on the stepping thread, which reads the
    // chassis's CURRENT pose there (never off this thread) and builds a
    // RelocateTarget from it: yaw comes from the chassis's local +X
    // (forward) axis rotated to world and projected onto the horizontal
    // plane, or - when the car is standing on its nose or tail and that
    // projection is too short (< 0.2) to trust - from the local +Y (left)
    // axis projected instead (yaw = atan2(left.y, left.x) - pi/2; both
    // degenerate cannot happen for a unit rotation, kept as a defensive
    // fallback to yaw 0). The target then goes through EXACTLY the same path
    // as request_relocate (relocation_ -> the terrain gate ->
    // finish_relocation): five-ray placement at the target x/y/yaw, zeroed
    // motion, World::reset_vehicle (gear to neutral, engine kept running
    // unless it was off) and status_.relocations counted. If a real
    // request_relocate is ALSO pending for the same tick, the explicit
    // relocate wins outright - it already lands upright, so the flip request
    // is simply dropped rather than carried into a later tick against a pose
    // the relocate has already moved past. Nothing calls this on its own, so
    // hashes are unaffected.
    void request_flip_upright();

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

    // The WorldTerrain this Session's terrain mode was built from (R2.2 R7:
    // "render and physics share decoded tiles since R4" - RgTerrainView's
    // initialize_shared() reuses this instead of opening/decoding a second
    // copy). Null in flat mode, or if this Session's TerrainModeConfig was
    // built directly (tests, tools/hash_check) rather than via the real
    // make_terrain_mode(WorldConfig, shared_ptr<WorldTerrain>) overload.
    [[nodiscard]] std::shared_ptr<WorldTerrain> world_terrain() const;

    // physics.terrain_surface's configured name (G2.5a-grip R-c HUD: "grip:
    // uniform <name>" when StreamingStatus::road_surfaces is false). Fixed at
    // construction, never mutated afterwards, so safe from any thread like
    // world_terrain() above. Empty in flat mode.
    [[nodiscard]] std::string terrain_surface_name() const;

    // --- Test seams (not running()): a kinematic traffic actor on a straight
    // 2 km route from `pose` along its own heading at `speed_mps` (no geo2map
    // data needed, flat or terrain mode), returned id is followable like any
    // traffic actor. Without a follow it despawns after a few unseen seconds
    // (the flat-mode population target is 0), like any surplus actor.
    // route_length_m / stop_at_end: a shorter route whose last two points reproduce a planned trip end (the second-last
    // point 0.7 m before the end at 1 m/s, the last at 0 m/s) for the arrival-despawn test.
    std::uint64_t add_test_traffic_actor(const ps::Pose& pose, double speed_mps, double route_length_m = 2000.0, bool stop_at_end = false);
    // The interest points the latest gate check built (terrain mode; empty in
    // flat mode or before the first one). Stepping thread / tests only.
    [[nodiscard]] std::span<const g2m::phys::InterestPoint> last_interest_points() const;
    // Where the latest NPC traffic scan was planned around (the chassis
    // position on the tick it started); nullopt before the first scan. A scan
    // needs the chassis, so none starts during terrain start-up priming.
    // Stepping thread / tests only.
    [[nodiscard]] std::optional<ps::Vec3> last_traffic_scan_origin() const { return traffic_scan_origin_; }
    // NPC stuck detector totals and the bounded event list (docs/npc_traffic.md). Stepping thread / tests only.
    [[nodiscard]] const TrafficStuckStats& traffic_stuck_stats() const { return traffic_stuck_; }
    [[nodiscard]] std::size_t traffic_actor_count() const { return traffic_actors_.size(); }
    // World position / controller speed of one traffic actor (nullopt when it is gone). Stepping thread / tests only.
    [[nodiscard]] std::optional<ps::Vec3> traffic_actor_position(std::uint64_t id) const {
        for (const auto& a : traffic_actors_) if (a.id == id) return world_->get_pose(a.body).position;
        return std::nullopt;
    }
    [[nodiscard]] std::optional<double> traffic_actor_speed(std::uint64_t id) const {
        for (const auto& a : traffic_actors_) if (a.id == id) return a.speed;
        return std::nullopt;
    }
    // True while a background traffic scan has not finished (tools/traffic_probe waits on it so a headless run is
    // deterministic: the plan is consumed at the same tick every time). Stepping thread / tests only.
    [[nodiscard]] bool traffic_scan_in_flight() const { return traffic_worker_.joinable() && !traffic_done_.load(); }

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
    // resident around - a LIST so later actors (walker, parked cars) slot in:
    // entry 0 is the chassis (or the spawn / a relocation target), entry 1 the
    // followed vehicle while one is followed (set_followed_vehicle). The
    // TileManager pool is sized for TWO points (setup_terrain); a third needs
    // another re-size. Sim thread only; the span points into Terrain's reused
    // buffer (valid until the next call).
    std::span<const g2m::phys::InterestPoint> physics_interest_points();
    // Where the followed actor is (traffic actor or the NPC truck); false when it does not exist.
    bool locate_followed(std::uint64_t id, ps::Vec3& position, ps::Vec3& velocity) const;
    // Stepping thread: clear a follow whose vehicle no longer exists and count it.
    void check_followed_alive();
    // Retries step_once(false) until it steps; throws after the stall
    // timeout, or SessionCancelled once config_.startup->cancel is set.
    void step_blocking(const char* what);
    void throw_if_cancelled() const;
    // The real-time loop's control copy (atomics, or the unattended override).
    void apply_live_controls();
    // The five spawn rays at (x, y, yaw); false (+ the first missing ray's
    // XY) when one misses.
    bool ray_spawn_pose(double x, double y, double yaw_rad, double clearance_m, ps::Pose& out, double& miss_x,
                        double& miss_y) const;
    // R9c, stepping thread. process: consume the pending spawn/enter/despawn
    // request (after the gate and any relocation); update: one controller tick
    // before World::step; spawn_walker places the body.
    void process_walker_request();
    void update_walker();
    void spawn_walker();
    void remove_walker();
    [[nodiscard]] ps::Vec3 traffic_anchor() const; // the walker while one exists, else the chassis
    void take_relocate_request(); // stepping thread
    void finish_relocation();     // stepping thread, gate ready at the target
    void post_step(bool from_loop);
    [[nodiscard]] FrameSnapshot capture_frame_snapshot() const;

    void update_traffic(bool clear=false);
    // follow_id/obstacle_* record WHAT the cached caps came from (stuck detector, traffic_stuck.h); route_cap is the
    // legal/corner/end target of the latest tick; stuck is the per-actor tracker.
    struct TrafficActor {std::uint64_t id;ps::BodyId body;TrafficTrip trip;double station=0,speed=0,unseen=0,obstacle_cap=1e30,follow_cap=1e30;
     std::uint64_t follow_id=0;ps::BodyId obstacle_body{};bool obstacle_hit=false;double obstacle_normal_z=0,route_cap=1e30,stuck_since=0;StuckTrack stuck;};
    static constexpr std::uint64_t kFollowPlayerId=~std::uint64_t{0};
    TrafficStuckStats traffic_stuck_;
    double traffic_stuck_summary_time_=0;
    void note_stuck_events(const std::vector<std::uint64_t>& declared,double now);
    std::vector<TrafficActor> traffic_actors_;
    std::thread traffic_worker_;
    std::atomic<bool> traffic_cancel_{false},traffic_done_{false};
    std::mutex traffic_mutex_;
    TrafficConfig traffic_requested_,traffic_config_;
    bool traffic_config_changed_=false,traffic_loading_=false,traffic_scan_needed_=true;
    std::vector<std::uint64_t> traffic_visible_;
    TrafficPlan traffic_pending_,traffic_ready_;
    double traffic_scan_time_=0;
    std::uint64_t traffic_next_id_=1,traffic_seed_=91731;
    int traffic_population_target_=0;
    struct TrafficNeighbor {std::uint64_t id;ps::Vec3 position;double speed,half_length,fx,fy;};
    std::map<std::pair<int,int>,std::vector<TrafficNeighbor>> traffic_neighbor_grid_;
    std::unordered_set<std::uint64_t> traffic_body_keys_; // bodies of the traffic actors (20 Hz): the obstacle probe leaves NPCs to the follow rule
    static std::uint64_t body_key(ps::BodyId b){return (static_cast<std::uint64_t>(b.index)<<32)^b.generation;}
    std::string traffic_message_;
    std::optional<ps::Vec3> traffic_scan_origin_;
    void update_npc_truck();
    std::thread truck_worker_;
    std::atomic<bool> truck_cancel_{false},truck_done_{false};
    std::atomic<int> truck_request_{0};
    std::atomic<double> truck_target_{70.0/3.6};
    std::mutex truck_mutex_;
    TruckRoute truck_pending_,truck_route_;
    TruckSnapshot truck_state_;
    ps::BodyId truck_body_{};
    double truck_station_=0,truck_speed_=0;
    SessionConfig config_;
    EnvironmentSample environment_sample_;
    std::shared_ptr<ps::io::SurfaceTable> surface_table_;
    std::unique_ptr<ps::World> world_;
    // Built lazily by sync_road_decks; declared after world_ so it is destroyed first (its workers call into the World).
    std::unique_ptr<DeckInstaller> deck_installer_;
    ps::BodyId ground_body_{};
    ps::BodyId chassis_body_{};
    ps::VehicleId vehicle_id_{};
    bool have_vehicle_ = false;
    std::uint64_t spawn_tick_ = 0;
    ps::vehicle::VehicleDesc vehicle_desc_;

    // On foot (R9c). walker_ is created at the first spawn and kept (its body
    // exists only while active()).
    std::unique_ptr<WalkerController> walker_;
    std::vector<WheelFootprint> walker_wheels_; // the own car's wheels, chassis-local, filled at construction
    std::mutex walker_input_mutex_;
    WalkerInput walker_input_;
    std::atomic<int> walker_jump_{0};
    std::atomic<int> walker_request_{0}; // 0 none, 1 spawn, 2 enter, 3 despawn
    std::atomic<bool> walker_active_{false};
    std::atomic<std::uint64_t> walker_spawned_{0}, walker_entered_{0}, walker_enter_refused_{0}, walker_despawned_{0},
        walker_spawn_failed_{0};
    OrientedRect walker_footprint_{};      // latest own-car footprint (stepping thread)
    bool walker_can_enter_ = false;
    double walker_enter_distance_m_ = 0.0;

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
        std::atomic<std::uint64_t> prefetch_enqueued{0};
        std::atomic<std::uint64_t> prefetch_installed{0};
        std::atomic<std::uint64_t> prefetch_late_sync{0};
        std::atomic<std::uint64_t> prefetch_late_wait{0};
        std::atomic<std::uint64_t> prefetch_late_wait_ns_total{0};
        std::atomic<std::uint64_t> prefetch_late_wait_ns_max{0};
        std::atomic<std::uint64_t> prefetch_stale_inputs{0};
        std::atomic<std::uint64_t> prefetch_cancelled{0};
        std::atomic<std::uint64_t> prefetch_skipped_suppression{0};
        std::atomic<double> startup_ms{0.0};
        std::atomic<std::uint32_t> prime_ticks{0};
        std::atomic<std::uint64_t> relocations{0};
        std::atomic<std::uint64_t> relocate_failures{0};
        std::atomic<std::uint64_t> followed_lost{0};
        std::atomic<std::uint64_t> followed_lost_id{0};
        std::atomic<std::uint32_t> interest_points{0};
    };
    std::atomic<std::uint64_t> followed_id_{0}; // 0 = none (traffic ids start at 1)
    bool follow_point_active_ = false;          // stepping thread: the TileManager holds kFollowedInterestId
    StatusAtomics status_;
    // Road decks (S1): create_body(desc, handle) is O(1) in the shape size (6.4 us for a 48k-triangle deck, was 22 ms), so
    // many decks install per tick; the shape builds run on kDeckBuildWorkers own threads, decks within
    // kDeckPrefetchMarginM beyond the required radius are built ahead.
    static constexpr int kDeckInstallBudget = 16;
    static constexpr unsigned kDeckBuildWorkers = 2;
    static constexpr double kDeckPrefetchMarginM = 600.0;
    bool sync_road_decks(int budget=0);
    bool in_freeze_ = false; // stepping thread only

    ps_godot::TripleBuffer<FrameSnapshot> snapshot_buffer_;
    ps_godot::PoseHistory pose_history_; // pushed by the loop thread (post_step), sampled by the render thread
    std::vector<ps::Pose> pose_history_scratch_{1};           // loop thread only
    std::vector<std::uint8_t> pose_history_teleport_{0};     // loop thread only
    std::uint64_t pose_history_relocations_ = 0;             // loop thread only: status_.relocations at the last push
    std::atomic<bool> paused_{false};
    std::atomic<bool> road_ahead_wanted_{false};
    // Stepping thread only (capture_frame_snapshot): the last traced road and
    // the tick it was traced at, reused between refreshes.
    mutable std::vector<RoadPoint> road_ahead_cache_;
    mutable std::uint64_t road_ahead_tick_ = 0;
    mutable bool road_ahead_cache_valid_ = false;
    std::mutex engine_audio_mutex_;
    EngineAudioPublisher engine_audio_publisher_;
    ps_godot::OriginRebase origin_rebase_;

    std::unordered_map<std::string, std::atomic<double>> control_channels_;
    std::atomic<VehicleControl> vehicle_control_{VehicleControl::Player};

    struct RelocateTarget {
        double x = 0.0;
        double y = 0.0;
        double yaw_rad = 0.0;
        bool clear_traffic = true;
    };
    double spawn_x_ = 0.0, spawn_y_ = 0.0, spawn_yaw_rad_ = 0.0;
    std::mutex relocate_mutex_;         // guards relocate_request_
    RelocateTarget relocate_request_;   // latest request (any thread)
    std::atomic<bool> relocate_pending_{false};
    std::atomic<bool> flip_upright_pending_{false}; // request_flip_upright (any thread)
    std::optional<RelocateTarget> relocation_; // stepping thread: accepted, waiting for its gate

    // Its thread runs step_once over every member above; ~Session() stops it
    // before anything is destroyed.
    // Tick-spike diagnostics (drain_tick_spikes): the timing members are
    // stepping-thread only; spikes_/spike_overflow_ are guarded by
    // spike_mutex_, taken only when an attempt actually is a spike.
    std::mutex spike_mutex_;
    std::vector<TickSpike> spikes_;
    std::uint64_t spike_overflow_ = 0;
    std::chrono::steady_clock::time_point loop_started_{};
    std::chrono::steady_clock::time_point last_attempt_start_{};
    bool have_last_attempt_ = false;
    double last_attempt_total_ms_ = 0.0;
    double last_gate_update_ms_ = 0.0;
    double last_gate_interest_ms_ = 0.0;

    FixedRateLoop loop_;
};

// Why constructing a Session failed (make_session below).
enum class SessionFailure : int { None = 0, Cancelled = 1, Error = 2 };

// Constructs a Session and turns ANY exception its constructor throws into
// nullptr plus *error (e.what(), or a fixed text for a non-std exception) and
// *failure (Cancelled for SessionCancelled). For callers that must not - or
// cannot - catch rg_core's exceptions themselves: a TU built with godot-cpp's
// default GODOTCPP_DISABLE_EXCEPTIONS (-D_HAS_EXCEPTIONS=0) sees MSVC STL's
// `std::exception` as stdext::exception, a DIFFERENT type from the one
// rg_core throws, so its `catch (const std::exception&)` never matches and
// the exception ends in std::terminate. Both out-pointers may be null.
std::unique_ptr<Session> make_session(const SessionConfig& config, std::string* error = nullptr,
                                      SessionFailure* failure = nullptr);

} // namespace rg
