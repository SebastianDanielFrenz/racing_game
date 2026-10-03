// godot_ext/src/rg_simulation.h — RgSimulation: the thin godot::Node
// wrapper around rg::Session (core/include/rg/session.h). Mirrors
// physics_sim's own adapters/godot/src/ps_simulation.h in SHAPE (a Node
// exposing simulation state as Godot-native types/Dictionaries to
// GDScript) but is deliberately much smaller - R0 scope is one ground body,
// one car_sedan-shaped vehicle, no terrain streaming, no articulations, no
// debug draw, no render-time interpolation (PLAN.md's judder fix is a
// later-milestone concern - see this file's own get_body_transform comment
// for exactly what is skipped and why).
//
// Every ps::/rg:: type crossing into a godot:: type happens in THIS file
// (plus frame_convert.h) - the engine-neutral/Godot boundary PLAN.md 11.1
// draws, mirrored from physics_sim's own P2 report.

#pragma once

#include "rg/player_mode.h"
#include "rg/session.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include "vehicle_voice.h"
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace rg_godot {

class RgTerrainView; // shared_world_terrain() friend access, see below

class RgSimulation : public godot::Node {
    GDCLASS(RgSimulation, godot::Node)

public:
    RgSimulation() = default;
    // Audio uses published snapshots only; no audio worker reads the world.
    godot::Dictionary start_engine_audio();
    void stop_engine_audio();
    void update_engine_audio();
    godot::PackedFloat32Array read_engine_audio(int frames);
    godot::PackedVector3Array get_engine_audio_positions() const;
    ~RgSimulation() override;

    // Builds a FLAT-mode rg::Session (ground + car_sedan-shaped chassis +
    // vehicle - see rg::SessionConfig's own doc comment for the exact
    // geometry). Paths are absolute, globalized (ProjectSettings.
    // globalize_path) by the GDScript caller - this class does no res://
    // resolution itself, same "no Godot resource-path convention leaks into
    // rg_core" reasoning as SessionConfig's own doc comment. Returns false
    // and sets get_last_error() on failure (e.g. a malformed vehicle JSON)
    // instead of letting rg::io::load_vehicle_json's std::runtime_error
    // cross the GDExtension boundary uncaught. Synchronous (flat start-up is
    // fast - no worker thread, unlike initialize_terrain() below).
    //
    // Re-entrant (R2.2 R7 "runtime world switch"): may be called again on an
    // object that already holds a Session (flat or terrain, loading or
    // running) - the old Session/init worker is torn down first (see
    // teardown_current()), never leaked, never left running alongside the
    // new one.
    bool initialize(const godot::String& vehicle_json_absolute_path, const godot::String& surface_table_absolute_path);

    // Builds a TERRAIN-mode rg::Session (rg::make_terrain_mode +
    // rg::Session) - opens the WorldTerrain named by world_config_absolute_path
    // itself (RgTerrainView::initialize_shared() then reuses it, no second
    // decode). R4's start-up can block for up to
    // WorldConfig::PhysicsTerrainConfig::startup_timeout_s (30 s) waiting for
    // real tile data, so the ENTIRE build - world_config parse, WorldTerrain::
    // open, make_terrain_mode, and the rg::Session constructor's own blocking
    // start-up/priming - runs on a background worker thread
    // (run_terrain_init_worker). Returns true once that thread has been
    // STARTED, not once it has succeeded; poll get_init_status() every frame
    // for a loading screen and call start() only once its state is "ready".
    // Re-entrant exactly like initialize() above - teardown_current()
    // CANCELS (R2.2 R9: rg::StartupProgress::cancel) and then joins any init
    // still in flight before starting the new one, so a world switch during
    // loading returns within a few milliseconds.
    bool initialize_terrain(const godot::String& world_config_absolute_path,
                            const godot::String& vehicle_json_absolute_path,
                            const godot::String& surface_table_absolute_path);

    // {state: "idle"|"loading"|"ready"|"error", message: String,
    //  resident_l0: int, missing_required: int, inflight: int, failed: int,
    //  stage: "opening"|"waiting_for_gate"|"priming"|"spawning"|"done",
    //  prime_done: int, prime_total: int}. Non-blocking - safe to poll every
    // frame from GDScript for a loading screen (see reap_init_thread()'s doc
    // comment for exactly why this never blocks). While "loading" the
    // figures come from the worker's rg::StartupProgress (R2.2 R9, live gate
    // figures while the Session constructor blocks); once "ready" from
    // session_->streaming_status().
    [[nodiscard]] godot::Dictionary get_init_status();

    // Test/smoke only (smoke_test.ps1 -DriveDelayMs): every height-tile fetch
    // of the NEXT initialize_terrain() sleeps D..2D ms first
    // (g2m::phys::DelayedFetch). 0 = off (default).
    void set_fetch_delay_ms(std::int64_t ms) { fetch_delay_ms_ = ms > 0 ? ms : 0; }

    // --- Player modes (R2.2 R9, rg/player_mode.h). The mode machine lives
    // here, not in the Session: it outlives world switches (the mode is kept
    // across initialize()/initialize_terrain()). Every change pushes the
    // effective rules' vehicle control into the Session. Mode names are
    // rg::to_string's ("drive", "free_cam", ...). ---
    // "changed" | "no_change" | "not_implemented" | "unknown_mode"
    godot::String set_player_mode(const godot::String& mode_name);
    godot::String cycle_player_mode(); // returns the new mode's name
    [[nodiscard]] godot::String get_player_mode() const;
    // {mode, implemented, vehicle_control, driving_inputs_live,
    //  camera_inputs_live, camera_rig, world_kind, world_phase, other_world,
    //  revision} - the EFFECTIVE rules (driving inputs masked while the world
    //  is not ready).
    [[nodiscard]] godot::Dictionary get_mode_state();

    void start();
    void stop();
    [[nodiscard]] bool is_running() const;

    // --- Terrain mode (R2.2 R7; all zero/false/empty in flat mode or before
    // a successful initialize_terrain()) ---
    [[nodiscard]] bool is_terrain_mode() const;
    [[nodiscard]] godot::Dictionary get_streaming_status() const; // every rg::StreamingStatus field, snake_case
    // Tick-spike diagnostics (rg::Session::drain_tick_spikes): the queued
    // spikes as preformatted lines, oldest first, plus one final
    // "overflow=N" line when spikes were dropped since the last call.
    [[nodiscard]] godot::PackedStringArray drain_tick_spikes();
    // rg::FixedRateLoop::LoopStats, snake_case (empty before start()).
    [[nodiscard]] godot::Dictionary get_loop_stats() const;
    // physics.terrain_surface's configured name (G2.5a-grip R-c HUD); empty
    // in flat mode. See rg::Session::terrain_surface_name().
    [[nodiscard]] godot::String get_terrain_surface_name() const;
    [[nodiscard]] godot::Vector3 get_render_origin_session() const; // RAW session-frame (east, north, up)
    void retry_failed_tiles();
    // rg::Session::request_relocate / request_reset_to_spawn (R2.2 R9):
    // move the car (session XY, yaw in degrees about +Z, 0 = east) - it
    // freezes through the terrain gate until the target is resident. Both
    // modes; no-op without a Session.
    void relocate_vehicle(double session_x, double session_y, double yaw_deg);
    void reset_vehicle_to_spawn();
    // rg::Session::request_flip_upright: put the car back on its wheels
    // WHERE IT IS, keeping its heading - unlike reset_vehicle_to_spawn, never
    // moves it back to spawn. No-op without a Session.
    void flip_vehicle_upright();

    [[nodiscard]] std::int64_t get_step_count() const;
    [[nodiscard]] double get_sim_time() const;
    [[nodiscard]] double get_tick_rate_hz() const;
    [[nodiscard]] std::int64_t get_body_count() const; // always 2 in R0 (ground, chassis)
    [[nodiscard]] godot::String get_last_error() const { return last_error_; }

    // Reports whether THIS DLL was built optimised, e.g. "optimized=yes
    // build_type=RelWithDebInfo" - a debug build makes the physics ~10x
    // slower (tick stalls while driving), and game/bin/librg_godot.dll can
    // be silently left over from a different preset's build (see
    // tools/run.ps1's stale-DLL guard) - main.gd prints this once so the
    // build type is always visible in the log, not just inferable from
    // which build dir was last touched. No Session needed - reports the
    // DLL's own compile-time config, not anything session state.
    [[nodiscard]] godot::String get_build_info() const;

    // --- Floating origin (PLAN.md 11.3: "only the active camera rebases") ---
    [[nodiscard]] godot::Vector3 get_world_origin_godot_position() const;
    godot::Vector3 rebase_focus(const godot::Vector3& focus_godot_position);

    // --- Adapter frame-time accounting (chase_cam.gd's own HUD counter -
    // plain accumulation, no core involvement, see .cpp) ---
    void add_adapter_time_us(std::int64_t us);
    [[nodiscard]] std::int64_t consume_adapter_frame_time_us();

    // --- Body transforms. "ground"/"chassis" are the only two body names
    // in R0 (rg::Session::build_world_contents). NOT render-time
    // interpolated (unlike physics_sim's own get_body_transform) - R0 reads
    // the session's latest published FrameSnapshot as-is; a visible
    // 240 Hz-vs-render-rate judder fix is deferred (see repo CLAUDE.md's
    // deviations list). ---
    [[nodiscard]] godot::Transform3D get_body_transform(const godot::String& body_name) const;
    [[nodiscard]] float get_body_speed_mps(const godot::String& body_name) const;

    // --- Named control channels (PLAN.md P2) ---
    void set_control(const godot::String& channel, double value);
    [[nodiscard]] double get_control(const godot::String& channel) const;

    // --- Vehicle / wheel telemetry. vehicle_name is unused beyond
    // validating it equals the one vehicle R0 ever creates ("car",
    // game/scripts/main.gd) - kept as a parameter for API parity with
    // physics_sim's own multi-vehicle-shaped methods, so a later milestone
    // adding a second vehicle needs no signature change here. ---
    [[nodiscard]] godot::PackedStringArray get_vehicle_names() const;
    [[nodiscard]] std::int64_t get_vehicle_wheel_count(const godot::String& vehicle_name) const;
    [[nodiscard]] godot::String get_wheel_name(const godot::String& vehicle_name, std::int64_t wheel_index) const;
    [[nodiscard]] float get_wheel_load_n(const godot::String& vehicle_name, std::int64_t wheel_index) const;
    [[nodiscard]] float get_wheel_slip_ratio(const godot::String& vehicle_name, std::int64_t wheel_index) const;
    [[nodiscard]] float get_wheel_slip_angle(const godot::String& vehicle_name, std::int64_t wheel_index) const;
    [[nodiscard]] godot::String get_wheel_surface_name(const godot::String& vehicle_name, std::int64_t wheel_index) const;

    [[nodiscard]] float get_wheel_omega(const godot::String& vehicle_name, std::int64_t wheel_index) const;

    [[nodiscard]] float get_wheel_fx(const godot::String& vehicle_name, std::int64_t wheel_index) const;

    [[nodiscard]] float get_wheel_fy(const godot::String& vehicle_name, std::int64_t wheel_index) const;

    [[nodiscard]] float get_wheel_radius(const godot::String& vehicle_name, std::int64_t wheel_index) const;

    // --- Vehicle visual (carvis brief, 2026-09-26): everything
    // game/scripts/vehicle_visual.gd (ported from physics_sim's own
    // adapters/godot/demo/scripts/vehicle_visual.gd) needs to place and
    // animate the real car_sedan.glb art instead of the red placeholder box.
    // Static geometry (attachment_local/steered/is_front) comes straight
    // from rg::Session::vehicle_desc() (cached at spawn, mirrors
    // physics_sim's own ps_simulation.h wheel_desc() pattern); the per-frame
    // fields (compression/spin_angle/steer_angle) come from
    // rg::Session::snapshot().wheels[i].state - the SAME race-free
    // FrameSnapshot get_wheel_load_n/slip_ratio/slip_angle above already
    // read (rg::Session::capture_frame_snapshot() fills it, on the sim
    // thread, from ps::World::wheel_state - never called live from here). ---
    [[nodiscard]] godot::Vector3 get_wheel_attachment_local(const godot::String& vehicle_name, std::int64_t wheel_index) const;
    [[nodiscard]] bool get_wheel_steered(const godot::String& vehicle_name, std::int64_t wheel_index) const;
    [[nodiscard]] bool get_wheel_is_front(const godot::String& vehicle_name, std::int64_t wheel_index) const;
    [[nodiscard]] float get_wheel_compression(const godot::String& vehicle_name, std::int64_t wheel_index) const; // m, suspension_travel: 0 = full extension
    [[nodiscard]] float get_wheel_spin_angle(const godot::String& vehicle_name, std::int64_t wheel_index) const; // rad, integrated, unbounded
    [[nodiscard]] float get_wheel_steer_angle(const godot::String& vehicle_name, std::int64_t wheel_index) const; // rad, this wheel's own post-Ackermann angle

    [[nodiscard]] godot::Dictionary get_vehicle_gauge_info(const godot::String& vehicle_name) const;
    [[nodiscard]] godot::Dictionary get_vehicle_speed_limit(const godot::String& vehicle_name) const;
    [[nodiscard]] godot::Dictionary get_vehicle_powertrain(const godot::String& vehicle_name) const;
    [[nodiscard]] float get_vehicle_ground_speed_mps(const godot::String& vehicle_name) const;

protected:
    static void _bind_methods();

private:
    friend class RgTerrainView; // shared_world_terrain() below

    [[nodiscard]] bool has_vehicle(const godot::String& vehicle_name) const;

    // The Session's shared WorldTerrain (null in flat mode or before a
    // successful initialize_terrain()) - RgTerrainView::initialize_shared()
    // is the one caller, reached across the GDExtension boundary via a
    // friend rather than a bound (GDScript-visible) method, since a raw
    // rg::WorldTerrain is not a Variant-compatible type and has no business
    // being exposed to script (R2.2 R7's "adapter stays thin" rule: this is
    // plumbing between two adapter classes, not game logic).
    [[nodiscard]] std::shared_ptr<rg::WorldTerrain> shared_world_terrain() const {
        return session_ ? session_->world_terrain() : nullptr;
    }

    enum class InitPhase : int { Idle, Loading, Ready, Error };

    // Pushes modes_.effective_rules().vehicle_control into session_.
    void apply_mode_to_session();

    // Tears down whatever this object currently holds - an in-flight init
    // worker (cancelled, then joined - R2.2 R9) and/or a
    // Session (stopped then destroyed) - leaving the object equivalent to a
    // freshly constructed one. Called at the start of initialize() and
    // initialize_terrain() (the "runtime world switch": R2.2 R7 task 1) and
    // from the destructor; never leaves a thread un-joined or a Session
    // running.
    void teardown_current();

    // reap_init_thread(wait=false): the per-frame poll path
    // (get_init_status()/start()) - peeks init_phase_ first and returns
    // immediately WITHOUT joining while it is still Loading, so a GDScript
    // per-frame poll never blocks the render thread for up to 30 s. Once the
    // worker has reached Ready/Error, joining is a formality (the thread has
    // already returned) - this call then joins and, on Ready, adopts
    // pending_session_ into session_.
    //
    // reap_init_thread(wait=true): the lifecycle points (stop/re-init/
    // destruction) - always joins. Since R2.2 R9 re-init and destruction
    // first set the in-flight start-up's rg::StartupProgress::cancel
    // (cancel_init()), so the join returns within a few gate checks; stop()
    // does not cancel (it waits for the start-up, as in R7).
    void reap_init_thread(bool wait);
    void cancel_init();

    // Runs entirely on init_thread_: parses world_config_absolute_path,
    // opens the WorldTerrain, builds a terrain-mode SessionConfig via
    // rg::make_terrain_mode, and constructs rg::Session (the up-to-30 s
    // blocking start-up, via rg::make_session) - every failure (a bad path,
    // a malformed config, a Session exception incl. SessionCancelled) is
    // reported through init_message_/InitPhase::Error. No exception is thrown
    // or caught in this TU: godot-cpp's -D_HAS_EXCEPTIONS=0 makes its
    // `std::exception` a different type (stdext::exception) from rg_core's,
    // so only rg_core itself can catch rg_core's exceptions.
    void run_terrain_init_worker(std::string world_config_path, std::string vehicle_json_path,
                                 std::string surface_table_path, std::shared_ptr<rg::StartupProgress> progress,
                                 std::int64_t fetch_delay_ms);

    // Access is serialized by init-worker join or Ready acquire. A running
    // world never writes this cache. Different paths invalidate its identity.
    void reuse_vehicle_definition(rg::SessionConfig& config) const;
    void remember_vehicle_definition(const rg::SessionConfig& config, const rg::Session& session);
    std::string cached_vehicle_path_;
    std::shared_ptr<const ps::vehicle::VehicleDesc> cached_vehicle_definition_;
    std::unique_ptr<rg::Session> session_;
    std::unique_ptr<ps_godot::VehicleVoice> engine_voice_;
    std::uint64_t audio_tick_ = ~std::uint64_t{0};
    godot::String last_error_;
    ps_godot::OriginRebase* origin_rebase_ = nullptr; // points at session_->origin_rebase(), valid once session_ exists
    std::atomic<std::int64_t> adapter_time_us_accum_{0}; // main-thread-only in practice (Godot single main thread)

    // Background terrain-init worker (R2.2 R7). init_message_/pending_session_
    // are plain (non-atomic) fields the worker writes and the main thread
    // reads - safe ONLY because every write happens-before the worker's
    // release-store into init_phase_, and the main thread never reads them
    // until its OWN acquire-load of init_phase_ has observed Ready/Error
    // (the same release/acquire hand-off pattern as
    // rg::TerrainViewStreamer). Never detached - see reap_init_thread().
    std::thread init_thread_;
    std::atomic<InitPhase> init_phase_{InitPhase::Idle};
    std::string init_message_;
    std::unique_ptr<rg::Session> pending_session_;
    std::shared_ptr<rg::StartupProgress> init_progress_; // the in-flight terrain init's (main thread owns the pointer)
    std::int64_t fetch_delay_ms_ = 0;

    rg::PlayerModeMachine modes_{rg::PlayerMode::Drive};
    std::uint64_t world_load_serial_ = 0; // modes_' serial of the load this object is running
};

} // namespace rg_godot
