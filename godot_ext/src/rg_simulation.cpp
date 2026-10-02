#include "rg_simulation.h"

#include "frame_convert.h"

#include "g2m/phys/height_tile_loader.h"

#include "ps/drivetrain/powertrain_desc.h"

#include <godot_cpp/core/class_db.hpp>

#include <chrono>
#include <optional>
#include <string>
#include <utility>
#include <variant>

using godot::D_METHOD;
using godot::String;

// godot_ext/CMakeLists.txt sets this via a $<CONFIG> generator expression
// (RgSimulation::get_build_info() below); fall back rather than fail to
// compile if this file is ever built outside that target.
#ifndef RG_BUILD_CONFIG
#define RG_BUILD_CONFIG "unknown"
#endif

namespace rg_godot {

namespace {

std::string to_std_string(const String& s) { return std::string(s.utf8().get_data()); }

// Mirrors physics_sim's own ps_simulation.cpp::wheel_desc() - the static
// (never changes per tick) wheel geometry a vehicle_visual.gd-style script
// needs, read from rg::Session::vehicle_desc() rather than the per-tick
// FrameSnapshot (see rg_simulation.h's "Vehicle visual" doc comment).
const ps::vehicle::WheelDesc* wheel_desc(const rg::Session& session, std::int64_t wheel_index) {
    const auto& wheels = session.vehicle_desc().wheels;
    if (wheel_index < 0 || static_cast<std::size_t>(wheel_index) >= wheels.size()) return nullptr;
    return &wheels[static_cast<std::size_t>(wheel_index)];
}

const char* shift_phase_name(ps::drivetrain::ShiftPhase p) {
    using ps::drivetrain::ShiftPhase;
    switch (p) {
        case ShiftPhase::Idle: return "Idle";
        case ShiftPhase::ClutchOpening: return "ClutchOpening";
        case ShiftPhase::Releasing: return "Releasing";
        case ShiftPhase::Selecting: return "Selecting";
        case ShiftPhase::Synchronising: return "Synchronising";
        case ShiftPhase::ClutchClosing: return "ClutchClosing";
    }
    return "Unknown";
}

const char* engine_state_name(ps::drivetrain::EngineState s) {
    using ps::drivetrain::EngineState;
    switch (s) {
        case EngineState::Off: return "Off";
        case EngineState::Stalled: return "Stalled";
        case EngineState::Cranking: return "Cranking";
        case EngineState::Running: return "Running";
    }
    return "Unknown";
}

} // namespace

RgSimulation::~RgSimulation() {
    cancel_init();
    reap_init_thread(/*wait=*/true);
    session_.reset(); // Session's own destructor stops its loop
    origin_rebase_ = nullptr;
}

void RgSimulation::cancel_init() {
    if (init_progress_) init_progress_->cancel.store(true, std::memory_order_relaxed);
}

void RgSimulation::teardown_current() {
    cancel_init();                   // R2.2 R9: cancel-then-join
    reap_init_thread(/*wait=*/true); // joins any in-flight worker first (see its own doc comment)
    if (session_) session_->stop();
    session_.reset();
    origin_rebase_ = nullptr;
    init_phase_.store(InitPhase::Idle, std::memory_order_relaxed);
    init_message_.clear();
}

void RgSimulation::reap_init_thread(bool wait) {
    if (!init_thread_.joinable()) return;
    if (!wait) {
        // Non-blocking peek: never join while the worker might still be
        // inside rg::Session's blocking terrain start-up (up to 30 s) - a
        // per-frame GDScript poll (get_init_status()/start()) must never
        // stall the render thread.
        if (init_phase_.load(std::memory_order_acquire) == InitPhase::Loading) return;
    }
    init_thread_.join(); // wait==true: blocks until the worker returns; wait==false: already finished, returns at once
    const bool ready = init_phase_.load(std::memory_order_acquire) == InitPhase::Ready;
    if (ready) {
        session_ = std::move(pending_session_);
        origin_rebase_ = &session_->origin_rebase();
    }
    pending_session_.reset(); // no-op on the Ready path (already moved out); frees nothing on Error (never set)
    init_progress_.reset();
    modes_.finish_world_load(world_load_serial_, ready); // ignored when a newer load superseded it
    apply_mode_to_session();
}

void RgSimulation::apply_mode_to_session() {
    if (session_) session_->set_vehicle_control(modes_.effective_rules().vehicle_control);
}

void RgSimulation::run_terrain_init_worker(std::string world_config_path, std::string vehicle_json_path,
                                           std::string surface_table_path,
                                           std::shared_ptr<rg::StartupProgress> progress,
                                           std::int64_t fetch_delay_ms) {
    // No C++ exception may be thrown or caught here: this TU is built with
    // godot-cpp's -D_HAS_EXCEPTIONS=0, where `std::exception` names
    // stdext::exception, so a `catch (const std::exception&)` here would never
    // match what rg_core throws (the R9 cancel-during-load crash:
    // SessionCancelled ended in std::terminate, exit 0xC0000409).
    // rg::make_session catches inside rg_core instead.
    const auto fail = [this](std::string message) {
        pending_session_.reset();
        init_message_ = std::move(message);
        init_phase_.store(InitPhase::Error, std::memory_order_release);
    };
    std::string err;
    std::optional<rg::WorldConfig> world_config = rg::load_world_config(world_config_path, &err);
    if (!world_config.has_value()) {
        fail("load_world_config: " + err);
        return;
    }

    const auto cancelled = [&progress] { return progress && progress->cancel.load(std::memory_order_relaxed); };
    if (cancelled()) {
        fail("Session: terrain start-up cancelled");
        return;
    }
    std::string open_err;
    std::shared_ptr<rg::WorldTerrain> terrain(rg::WorldTerrain::open(*world_config, &open_err));
    if (terrain == nullptr) {
        fail("WorldTerrain::open: " + open_err);
        return;
    }

    if (cancelled()) { // opening the WorldTerrain cannot be interrupted; check right after it
        fail("Session: terrain start-up cancelled");
        return;
    }

    rg::SessionConfig config;
    config.vehicle_json_path = std::move(vehicle_json_path);
    config.surface_table_path = std::move(surface_table_path);
    config.terrain = rg::make_terrain_mode(*world_config, terrain); // start-up blocks inside make_session below
    config.startup = progress;
    if (fetch_delay_ms > 0) {
        config.terrain->fetch = std::make_shared<g2m::phys::DelayedFetch>(
            config.terrain->fetch, std::chrono::milliseconds(fetch_delay_ms), 7u);
    }

    std::string session_err;
    std::unique_ptr<rg::Session> session = rg::make_session(config, &session_err);
    if (session == nullptr) {
        fail(std::move(session_err)); // includes "Session: terrain start-up cancelled"
        return;
    }
    pending_session_ = std::move(session);
    init_message_ = "ready";
    init_phase_.store(InitPhase::Ready, std::memory_order_release);
}

bool RgSimulation::initialize(const String& vehicle_json_absolute_path, const String& surface_table_absolute_path) {
    teardown_current();
    world_load_serial_ = modes_.begin_world_load(rg::WorldKind::Flat);
    rg::SessionConfig config;
    config.vehicle_json_path = to_std_string(vehicle_json_absolute_path);
    config.surface_table_path = to_std_string(surface_table_absolute_path);
    std::string err;
    session_ = rg::make_session(config, &err); // never throws (see run_terrain_init_worker)
    if (session_ == nullptr) {
        last_error_ = String(err.c_str());
        modes_.finish_world_load(world_load_serial_, false);
        return false;
    }
    origin_rebase_ = &session_->origin_rebase();
    last_error_ = String();
    modes_.finish_world_load(world_load_serial_, true);
    apply_mode_to_session();
    return true;
}

bool RgSimulation::initialize_terrain(const String& world_config_absolute_path,
                                      const String& vehicle_json_absolute_path,
                                      const String& surface_table_absolute_path) {
    teardown_current();
    last_error_ = String();
    init_message_.clear();
    world_load_serial_ = modes_.begin_world_load(rg::WorldKind::RealWorld);
    init_progress_ = std::make_shared<rg::StartupProgress>();
    init_phase_.store(InitPhase::Loading, std::memory_order_release);

    std::string world_config_path = to_std_string(world_config_absolute_path);
    std::string vehicle_json_path = to_std_string(vehicle_json_absolute_path);
    std::string surface_table_path = to_std_string(surface_table_absolute_path);
    init_thread_ = std::thread([this, world_config_path = std::move(world_config_path),
                               vehicle_json_path = std::move(vehicle_json_path),
                               surface_table_path = std::move(surface_table_path), progress = init_progress_,
                               delay = fetch_delay_ms_]() mutable {
        run_terrain_init_worker(std::move(world_config_path), std::move(vehicle_json_path),
                                std::move(surface_table_path), std::move(progress), delay);
    });
    return true; // started, not necessarily succeeded - see get_init_status()
}

godot::Dictionary RgSimulation::get_init_status() {
    reap_init_thread(/*wait=*/false);
    godot::Dictionary d;
    const InitPhase phase = init_phase_.load(std::memory_order_acquire);
    const char* state = "idle";
    switch (phase) {
        case InitPhase::Idle: state = "idle"; break;
        case InitPhase::Loading: state = "loading"; break;
        case InitPhase::Ready: state = "ready"; break;
        case InitPhase::Error: state = "error"; break;
    }
    d["state"] = String(state);
    d["message"] = String(init_message_.c_str());
    std::int64_t resident = 0, missing = 0, inflight = 0, failed = 0, prime_done = 0, prime_total = 0;
    const char* stage = "done";
    if (phase == InitPhase::Ready && session_) {
        const rg::StreamingStatus s = session_->streaming_status();
        resident = s.resident_l0;
        missing = s.missing_required;
        inflight = s.inflight;
        failed = s.failed;
        prime_done = prime_total = s.prime_ticks;
    } else if (phase == InitPhase::Loading && init_progress_) {
        const rg::StartupProgress& p = *init_progress_;
        resident = p.resident_l0.load(std::memory_order_relaxed);
        missing = p.missing_required.load(std::memory_order_relaxed);
        inflight = p.inflight.load(std::memory_order_relaxed);
        failed = p.failed.load(std::memory_order_relaxed);
        prime_done = p.prime_done.load(std::memory_order_relaxed);
        prime_total = p.prime_total.load(std::memory_order_relaxed);
        switch (p.stage.load(std::memory_order_relaxed)) {
            case rg::StartupProgress::NotStarted: stage = "opening"; break;
            case rg::StartupProgress::WaitingForGate: stage = "waiting_for_gate"; break;
            case rg::StartupProgress::Priming: stage = "priming"; break;
            case rg::StartupProgress::Spawning: stage = "spawning"; break;
            default: stage = "done"; break;
        }
    }
    d["resident_l0"] = resident;
    d["missing_required"] = missing;
    d["inflight"] = inflight;
    d["failed"] = failed;
    d["stage"] = String(stage);
    d["prime_done"] = prime_done;
    d["prime_total"] = prime_total;
    return d;
}

void RgSimulation::start() {
    reap_init_thread(/*wait=*/false); // opportunistically adopt a just-finished terrain init
    if (session_) session_->start();
}

void RgSimulation::stop() {
    reap_init_thread(/*wait=*/true); // per brief: join the init thread on stop
    if (session_) session_->stop();
}

bool RgSimulation::is_running() const { return session_ && session_->running(); }

bool RgSimulation::is_terrain_mode() const { return session_ && session_->terrain_mode(); }

godot::Dictionary RgSimulation::get_streaming_status() const {
    godot::Dictionary d;
    rg::StreamingStatus s; // default-constructed = every field zero/false, terrain_mode = false
    if (session_) s = session_->streaming_status();
    d["terrain_mode"] = s.terrain_mode;
    d["ready"] = s.ready;
    d["frozen"] = s.frozen;
    d["missing_required"] = static_cast<std::int64_t>(s.missing_required);
    d["inflight"] = static_cast<std::int64_t>(s.inflight);
    d["resident_l0"] = static_cast<std::int64_t>(s.resident_l0);
    d["failed"] = static_cast<std::int64_t>(s.failed);
    d["frozen_attempts"] = static_cast<std::int64_t>(s.frozen_attempts);
    d["freeze_count"] = static_cast<std::int64_t>(s.freeze_count);
    d["fill_misses"] = static_cast<std::int64_t>(s.fill_misses);
    d["nodata_fills"] = static_cast<std::int64_t>(s.nodata_fills);
    d["falls"] = static_cast<std::int64_t>(s.falls);
    d["resident_tiles"] = static_cast<std::int64_t>(s.resident_tiles);
    d["starved_tiles"] = static_cast<std::int64_t>(s.starved_tiles);
    d["relief_overflow"] = static_cast<std::int64_t>(s.relief_overflow);
    d["startup_ms"] = s.startup_ms;
    d["prime_ticks"] = static_cast<std::int64_t>(s.prime_ticks);
    d["relocations"] = static_cast<std::int64_t>(s.relocations);
    d["relocate_failures"] = static_cast<std::int64_t>(s.relocate_failures);
    d["prefetch_enqueued"] = static_cast<std::int64_t>(s.prefetch.enqueued);
    d["prefetch_installed"] = static_cast<std::int64_t>(s.prefetch.installed);
    d["prefetch_late_sync"] = static_cast<std::int64_t>(s.prefetch.late_sync);
    d["prefetch_late_wait"] = static_cast<std::int64_t>(s.prefetch.late_wait);
    d["prefetch_late_wait_ns_total"] = static_cast<std::int64_t>(s.prefetch.late_wait_ns_total);
    d["prefetch_late_wait_ns_max"] = static_cast<std::int64_t>(s.prefetch.late_wait_ns_max);
    d["prefetch_stale_inputs"] = static_cast<std::int64_t>(s.prefetch.stale_inputs);
    d["prefetch_cancelled"] = static_cast<std::int64_t>(s.prefetch.cancelled);
    d["prefetch_skipped_suppression"] = static_cast<std::int64_t>(s.prefetch.skipped_suppression);
    d["road_surfaces"] = s.road_surfaces;
    d["osm_ok"] = static_cast<std::int64_t>(s.osm_ok);
    d["osm_fail"] = static_cast<std::int64_t>(s.osm_fail);
    return d;
}

godot::PackedStringArray RgSimulation::drain_tick_spikes() {
    godot::PackedStringArray out;
    if (!session_) return out;
    std::uint64_t overflow = 0;
    for (const rg::Session::TickSpike& s : session_->drain_tick_spikes(&overflow)) {
        out.push_back(String(rg::Session::format_tick_spike(s).c_str()));
    }
    if (overflow > 0) out.push_back(String(("overflow=" + std::to_string(overflow)).c_str()));
    return out;
}

godot::Dictionary RgSimulation::get_loop_stats() const {
    godot::Dictionary d;
    if (!session_ || !session_->running()) return d;
    const rg::FixedRateLoop::LoopStats s = session_->loop_stats();
    d["achieved_hz"] = s.achieved_hz;
    d["step_p50_ms"] = s.step_p50_ms;
    d["step_p99_ms"] = s.step_p99_ms;
    d["step_max_ms"] = s.step_max_ms;
    d["frozen_ms"] = s.frozen_ms;
    d["freeze_count"] = static_cast<std::int64_t>(s.freeze_count);
    d["stepped_count"] = static_cast<std::int64_t>(s.stepped_count);
    d["dropped_ticks"] = static_cast<std::int64_t>(s.dropped_ticks);
    return d;
}

String RgSimulation::get_terrain_surface_name() const {
    return session_ ? String(session_->terrain_surface_name().c_str()) : String();
}

godot::Vector3 RgSimulation::get_render_origin_session() const {
    if (!origin_rebase_) return godot::Vector3();
    const ps::Vec3 o = origin_rebase_->origin(); // RAW session-frame (east, north, up) - no basis conversion
    return godot::Vector3(static_cast<float>(o.x), static_cast<float>(o.y), static_cast<float>(o.z));
}

void RgSimulation::retry_failed_tiles() {
    if (session_) session_->retry_failed_tiles();
}

void RgSimulation::relocate_vehicle(double session_x, double session_y, double yaw_deg) {
    if (session_) session_->request_relocate(session_x, session_y, yaw_deg * (3.14159265358979323846 / 180.0));
}

void RgSimulation::reset_vehicle_to_spawn() {
    if (session_) session_->request_reset_to_spawn();
}

void RgSimulation::flip_vehicle_upright() {
    if (session_) session_->request_flip_upright();
}

std::int64_t RgSimulation::get_step_count() const {
    return session_ ? static_cast<std::int64_t>(session_->snapshot().tick) : 0;
}

double RgSimulation::get_sim_time() const { return session_ ? session_->snapshot().sim_time : 0.0; }

double RgSimulation::get_tick_rate_hz() const { return session_ ? session_->world().dt() > 0.0 ? 1.0 / session_->world().dt() : 0.0 : 0.0; }

std::int64_t RgSimulation::get_body_count() const { return session_ ? 2 : 0; }

godot::String RgSimulation::get_build_info() const {
    // NDEBUG matches CMake's own Release/RelWithDebInfo default flags (both
    // define it, Debug does not) - the same signal an optimised-vs-debug
    // build reports itself with everywhere else in this codebase.
#ifdef NDEBUG
    const bool optimized = true;
#else
    const bool optimized = false;
#endif
    std::string info = "optimized=";
    info += optimized ? "yes" : "no";
    info += " build_type=";
    info += RG_BUILD_CONFIG;
    return String(info.c_str());
}

godot::Vector3 RgSimulation::get_world_origin_godot_position() const {
    if (!origin_rebase_) return godot::Vector3();
    return iso_to_godot(ps::Vec3{0.0, 0.0, 0.0}, origin_rebase_->origin());
}

godot::Vector3 RgSimulation::rebase_focus(const godot::Vector3& focus_godot_position) {
    if (!origin_rebase_) return focus_godot_position;
    const ps::Vec3 absolute = godot_to_iso(focus_godot_position, origin_rebase_->origin());
    origin_rebase_->update(absolute);
    return iso_to_godot(absolute, origin_rebase_->origin());
}

void RgSimulation::add_adapter_time_us(std::int64_t us) {
    adapter_time_us_accum_.fetch_add(us, std::memory_order_relaxed);
}

std::int64_t RgSimulation::consume_adapter_frame_time_us() {
    return adapter_time_us_accum_.exchange(0, std::memory_order_relaxed);
}

godot::Transform3D RgSimulation::get_body_transform(const String& body_name) const {
    if (!session_) return godot::Transform3D();
    const std::string name = to_std_string(body_name);
    ps::Pose pose;
    if (name == "chassis") {
        pose = session_->snapshot().chassis_pose;
    } else if (name == "ground") {
        pose = ps::Pose::identity(); // static, built at the origin (session.cpp)
    } else {
        return godot::Transform3D();
    }
    return iso_to_godot_transform(pose, origin_rebase_ ? origin_rebase_->origin() : ps::Vec3{});
}

float RgSimulation::get_body_speed_mps(const String& body_name) const {
    if (!session_ || to_std_string(body_name) != "chassis") return 0.0f;
    return static_cast<float>(session_->snapshot().chassis_motion.linear.length());
}

void RgSimulation::set_control(const String& channel, double value) {
    if (session_) session_->set_control(to_std_string(channel), value);
}

double RgSimulation::get_control(const String& channel) const {
    return session_ ? session_->get_control(to_std_string(channel)) : 0.0;
}

bool RgSimulation::has_vehicle(const String& vehicle_name) const {
    return session_ && to_std_string(vehicle_name) == session_->vehicle_desc().name;
}

godot::PackedStringArray RgSimulation::get_vehicle_names() const {
    godot::PackedStringArray out;
    if (session_) out.push_back(String(session_->vehicle_desc().name.c_str()));
    return out;
}

std::int64_t RgSimulation::get_vehicle_wheel_count(const String& vehicle_name) const {
    if (!has_vehicle(vehicle_name)) return 0;
    return static_cast<std::int64_t>(session_->snapshot().wheels.size());
}

String RgSimulation::get_wheel_name(const String& vehicle_name, std::int64_t wheel_index) const {
    if (!has_vehicle(vehicle_name)) return String();
    const auto& wheels = session_->snapshot().wheels;
    if (wheel_index < 0 || static_cast<std::size_t>(wheel_index) >= wheels.size()) return String();
    return String(wheels[static_cast<std::size_t>(wheel_index)].name.c_str());
}

float RgSimulation::get_wheel_load_n(const String& vehicle_name, std::int64_t wheel_index) const {
    if (!has_vehicle(vehicle_name)) return 0.0f;
    const auto& wheels = session_->snapshot().wheels;
    if (wheel_index < 0 || static_cast<std::size_t>(wheel_index) >= wheels.size()) return 0.0f;
    return static_cast<float>(wheels[static_cast<std::size_t>(wheel_index)].state.load);
}

float RgSimulation::get_wheel_slip_ratio(const String& vehicle_name, std::int64_t wheel_index) const {
    if (!has_vehicle(vehicle_name)) return 0.0f;
    const auto& wheels = session_->snapshot().wheels;
    if (wheel_index < 0 || static_cast<std::size_t>(wheel_index) >= wheels.size()) return 0.0f;
    return static_cast<float>(wheels[static_cast<std::size_t>(wheel_index)].state.slip_ratio);
}

float RgSimulation::get_wheel_slip_angle(const String& vehicle_name, std::int64_t wheel_index) const {
    if (!has_vehicle(vehicle_name)) return 0.0f;
    const auto& wheels = session_->snapshot().wheels;
    if (wheel_index < 0 || static_cast<std::size_t>(wheel_index) >= wheels.size()) return 0.0f;
    return static_cast<float>(wheels[static_cast<std::size_t>(wheel_index)].state.slip_angle);
}

String RgSimulation::get_wheel_surface_name(const String& vehicle_name, std::int64_t wheel_index) const {
    if (!has_vehicle(vehicle_name)) return String();
    const auto& wheels = session_->snapshot().wheels;
    if (wheel_index < 0 || static_cast<std::size_t>(wheel_index) >= wheels.size()) return String();
    const ps::SurfaceId surface = wheels[static_cast<std::size_t>(wheel_index)].state.surface;
    return String(session_->surface_table().name_for(surface).c_str());
}

godot::Vector3 RgSimulation::get_wheel_attachment_local(const String& vehicle_name, std::int64_t wheel_index) const {
    if (!has_vehicle(vehicle_name)) return godot::Vector3();
    const ps::vehicle::WheelDesc* w = wheel_desc(*session_, wheel_index);
    if (w == nullptr) return godot::Vector3();
    return godot::Vector3(static_cast<float>(w->attachment_local.x), static_cast<float>(w->attachment_local.y),
                          static_cast<float>(w->attachment_local.z));
}

bool RgSimulation::get_wheel_steered(const String& vehicle_name, std::int64_t wheel_index) const {
    if (!has_vehicle(vehicle_name)) return false;
    const ps::vehicle::WheelDesc* w = wheel_desc(*session_, wheel_index);
    return w != nullptr && w->steered;
}

bool RgSimulation::get_wheel_is_front(const String& vehicle_name, std::int64_t wheel_index) const {
    if (!has_vehicle(vehicle_name)) return false;
    const ps::vehicle::WheelDesc* w = wheel_desc(*session_, wheel_index);
    return w != nullptr && w->is_front;
}

float RgSimulation::get_wheel_compression(const String& vehicle_name, std::int64_t wheel_index) const {
    if (!has_vehicle(vehicle_name)) return 0.0f;
    const auto& wheels = session_->snapshot().wheels;
    if (wheel_index < 0 || static_cast<std::size_t>(wheel_index) >= wheels.size()) return 0.0f;
    return static_cast<float>(wheels[static_cast<std::size_t>(wheel_index)].state.suspension_travel);
}

float RgSimulation::get_wheel_spin_angle(const String& vehicle_name, std::int64_t wheel_index) const {
    if (!has_vehicle(vehicle_name)) return 0.0f;
    const auto& wheels = session_->snapshot().wheels;
    if (wheel_index < 0 || static_cast<std::size_t>(wheel_index) >= wheels.size()) return 0.0f;
    return static_cast<float>(wheels[static_cast<std::size_t>(wheel_index)].state.spin_angle);
}

float RgSimulation::get_wheel_steer_angle(const String& vehicle_name, std::int64_t wheel_index) const {
    if (!has_vehicle(vehicle_name)) return 0.0f;
    const auto& wheels = session_->snapshot().wheels;
    if (wheel_index < 0 || static_cast<std::size_t>(wheel_index) >= wheels.size()) return 0.0f;
    return static_cast<float>(wheels[static_cast<std::size_t>(wheel_index)].state.steer_angle);
}

godot::Dictionary RgSimulation::get_vehicle_gauge_info(const String& vehicle_name) const {
    godot::Dictionary d;
    if (!has_vehicle(vehicle_name)) return d;

    ps::real idle_rpm = 0.0, limiter_rpm = 0.0;
    std::int64_t gear_count = 0;
    for (const auto& c : session_->vehicle_desc().powertrain.components) {
        if (const auto* engine = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&c.params)) {
            idle_rpm = engine->idle_rpm;
            limiter_rpm = engine->limiter.rpm;
        } else if (const auto* gearbox = std::get_if<ps::drivetrain::GearboxDesc>(&c.params)) {
            gear_count = static_cast<std::int64_t>(gearbox->forward_ratios.size());
        }
    }
    d["idle_rpm"] = static_cast<float>(idle_rpm);
    d["limiter_rpm"] = static_cast<float>(limiter_rpm);
    d["gear_count"] = gear_count;
    d["fitted_assists"] = godot::PackedStringArray();
    return d;
}

godot::Dictionary RgSimulation::get_vehicle_powertrain(const String& vehicle_name) const {
    godot::Dictionary d;
    if (!has_vehicle(vehicle_name)) return d;
    const ps::drivetrain::PowertrainSnapshot& p = session_->snapshot().powertrain;

    d["gear"] = static_cast<int>(p.gear);
    d["gear_target"] = static_cast<int>(p.gear_target);
    d["shift_phase"] = String(shift_phase_name(p.shift_phase));
    d["rpm"] = static_cast<float>(p.engine_rpm);
    d["engine_state"] = String(engine_state_name(p.engine_state));
    d["speedo_kmh"] = static_cast<float>(p.speedo_mps * 3.6);
    d["clutch_engagement"] = static_cast<float>(p.clutch_engagement);
    d["limiter"] = !p.engines.empty() && p.engines[0].limiter_active;
    d["starter_active"] = !p.engines.empty() && p.engines[0].starter_active;
    d["dfco"] = !p.engines.empty() && p.engines[0].dfco_active;
    d["grind_active"] = p.driveline.grind_active;
    d["assist_auto_clutch"] = (p.assist_active_bits & 1u) != 0;
    d["assist_auto_blip"] = (p.assist_active_bits & 2u) != 0;
    d["assist_auto_shift"] = (p.assist_active_bits & 4u) != 0;
    return d;
}

float RgSimulation::get_vehicle_ground_speed_mps(const String& vehicle_name) const {
    if (!has_vehicle(vehicle_name)) return 0.0f;
    return static_cast<float>(session_->snapshot().chassis_motion.linear.length());
}

godot::String RgSimulation::set_player_mode(const String& mode_name) {
    const std::optional<rg::PlayerMode> mode = rg::player_mode_from_string(to_std_string(mode_name));
    if (!mode.has_value()) return String("unknown_mode");
    const rg::PlayerModeMachine::Result r = modes_.request_mode(*mode);
    apply_mode_to_session();
    switch (r) {
        case rg::PlayerModeMachine::Result::Changed: return String("changed");
        case rg::PlayerModeMachine::Result::NoChange: return String("no_change");
        case rg::PlayerModeMachine::Result::NotImplemented: return String("not_implemented");
    }
    return String("unknown_mode");
}

godot::String RgSimulation::cycle_player_mode() {
    const rg::PlayerMode m = modes_.cycle_mode();
    apply_mode_to_session();
    return String(rg::to_string(m));
}

godot::String RgSimulation::get_player_mode() const { return String(rg::to_string(modes_.mode())); }

godot::Dictionary RgSimulation::get_mode_state() {
    reap_init_thread(/*wait=*/false); // a just-finished load updates the world phase
    const rg::ModeRules r = modes_.effective_rules();
    godot::Dictionary d;
    d["mode"] = String(rg::to_string(modes_.mode()));
    d["implemented"] = r.implemented;
    d["vehicle_control"] = String(rg::to_string(r.vehicle_control));
    d["driving_inputs_live"] = r.driving_inputs_live;
    d["camera_inputs_live"] = r.camera_inputs_live;
    d["camera_rig"] = String(rg::to_string(r.camera_rig));
    d["world_kind"] = String(rg::to_string(modes_.world_kind()));
    d["world_phase"] = String(rg::to_string(modes_.world_phase()));
    d["other_world"] = String(rg::to_string(modes_.other_world()));
    d["revision"] = static_cast<std::int64_t>(modes_.revision());
    return d;
}

void RgSimulation::_bind_methods() {
    godot::ClassDB::bind_method(D_METHOD("initialize", "vehicle_json_absolute_path", "surface_table_absolute_path"), &RgSimulation::initialize);
    godot::ClassDB::bind_method(D_METHOD("initialize_terrain", "world_config_absolute_path", "vehicle_json_absolute_path", "surface_table_absolute_path"), &RgSimulation::initialize_terrain);
    godot::ClassDB::bind_method(D_METHOD("get_init_status"), &RgSimulation::get_init_status);
    godot::ClassDB::bind_method(D_METHOD("set_fetch_delay_ms", "ms"), &RgSimulation::set_fetch_delay_ms);
    godot::ClassDB::bind_method(D_METHOD("set_player_mode", "mode_name"), &RgSimulation::set_player_mode);
    godot::ClassDB::bind_method(D_METHOD("cycle_player_mode"), &RgSimulation::cycle_player_mode);
    godot::ClassDB::bind_method(D_METHOD("get_player_mode"), &RgSimulation::get_player_mode);
    godot::ClassDB::bind_method(D_METHOD("get_mode_state"), &RgSimulation::get_mode_state);
    godot::ClassDB::bind_method(D_METHOD("start"), &RgSimulation::start);
    godot::ClassDB::bind_method(D_METHOD("stop"), &RgSimulation::stop);
    godot::ClassDB::bind_method(D_METHOD("is_running"), &RgSimulation::is_running);
    godot::ClassDB::bind_method(D_METHOD("is_terrain_mode"), &RgSimulation::is_terrain_mode);
    godot::ClassDB::bind_method(D_METHOD("get_streaming_status"), &RgSimulation::get_streaming_status);
    godot::ClassDB::bind_method(D_METHOD("drain_tick_spikes"), &RgSimulation::drain_tick_spikes);
    godot::ClassDB::bind_method(D_METHOD("get_loop_stats"), &RgSimulation::get_loop_stats);
    godot::ClassDB::bind_method(D_METHOD("get_terrain_surface_name"), &RgSimulation::get_terrain_surface_name);
    godot::ClassDB::bind_method(D_METHOD("get_render_origin_session"), &RgSimulation::get_render_origin_session);
    godot::ClassDB::bind_method(D_METHOD("retry_failed_tiles"), &RgSimulation::retry_failed_tiles);
    godot::ClassDB::bind_method(D_METHOD("relocate_vehicle", "session_x", "session_y", "yaw_deg"),
                                &RgSimulation::relocate_vehicle);
    godot::ClassDB::bind_method(D_METHOD("reset_vehicle_to_spawn"), &RgSimulation::reset_vehicle_to_spawn);
    godot::ClassDB::bind_method(D_METHOD("flip_vehicle_upright"), &RgSimulation::flip_vehicle_upright);
    godot::ClassDB::bind_method(D_METHOD("get_step_count"), &RgSimulation::get_step_count);
    godot::ClassDB::bind_method(D_METHOD("get_sim_time"), &RgSimulation::get_sim_time);
    godot::ClassDB::bind_method(D_METHOD("get_tick_rate_hz"), &RgSimulation::get_tick_rate_hz);
    godot::ClassDB::bind_method(D_METHOD("get_body_count"), &RgSimulation::get_body_count);
    godot::ClassDB::bind_method(D_METHOD("get_last_error"), &RgSimulation::get_last_error);
    godot::ClassDB::bind_method(D_METHOD("get_build_info"), &RgSimulation::get_build_info);

    godot::ClassDB::bind_method(D_METHOD("get_world_origin_godot_position"), &RgSimulation::get_world_origin_godot_position);
    godot::ClassDB::bind_method(D_METHOD("rebase_focus", "focus_godot_position"), &RgSimulation::rebase_focus);
    godot::ClassDB::bind_method(D_METHOD("add_adapter_time_us", "us"), &RgSimulation::add_adapter_time_us);
    godot::ClassDB::bind_method(D_METHOD("consume_adapter_frame_time_us"), &RgSimulation::consume_adapter_frame_time_us);

    godot::ClassDB::bind_method(D_METHOD("get_body_transform", "body_name"), &RgSimulation::get_body_transform);
    godot::ClassDB::bind_method(D_METHOD("get_body_speed_mps", "body_name"), &RgSimulation::get_body_speed_mps);

    godot::ClassDB::bind_method(D_METHOD("set_control", "channel", "value"), &RgSimulation::set_control);
    godot::ClassDB::bind_method(D_METHOD("get_control", "channel"), &RgSimulation::get_control);

    godot::ClassDB::bind_method(D_METHOD("get_vehicle_names"), &RgSimulation::get_vehicle_names);
    godot::ClassDB::bind_method(D_METHOD("get_vehicle_wheel_count", "vehicle_name"), &RgSimulation::get_vehicle_wheel_count);
    godot::ClassDB::bind_method(D_METHOD("get_wheel_name", "vehicle_name", "wheel_index"), &RgSimulation::get_wheel_name);
    godot::ClassDB::bind_method(D_METHOD("get_wheel_load_n", "vehicle_name", "wheel_index"), &RgSimulation::get_wheel_load_n);
    godot::ClassDB::bind_method(D_METHOD("get_wheel_slip_ratio", "vehicle_name", "wheel_index"), &RgSimulation::get_wheel_slip_ratio);
    godot::ClassDB::bind_method(D_METHOD("get_wheel_slip_angle", "vehicle_name", "wheel_index"), &RgSimulation::get_wheel_slip_angle);
    godot::ClassDB::bind_method(D_METHOD("get_wheel_surface_name", "vehicle_name", "wheel_index"), &RgSimulation::get_wheel_surface_name);

    godot::ClassDB::bind_method(D_METHOD("get_wheel_attachment_local", "vehicle_name", "wheel_index"), &RgSimulation::get_wheel_attachment_local);
    godot::ClassDB::bind_method(D_METHOD("get_wheel_steered", "vehicle_name", "wheel_index"), &RgSimulation::get_wheel_steered);
    godot::ClassDB::bind_method(D_METHOD("get_wheel_is_front", "vehicle_name", "wheel_index"), &RgSimulation::get_wheel_is_front);
    godot::ClassDB::bind_method(D_METHOD("get_wheel_compression", "vehicle_name", "wheel_index"), &RgSimulation::get_wheel_compression);
    godot::ClassDB::bind_method(D_METHOD("get_wheel_spin_angle", "vehicle_name", "wheel_index"), &RgSimulation::get_wheel_spin_angle);
    godot::ClassDB::bind_method(D_METHOD("get_wheel_steer_angle", "vehicle_name", "wheel_index"), &RgSimulation::get_wheel_steer_angle);

    godot::ClassDB::bind_method(D_METHOD("get_vehicle_gauge_info", "vehicle_name"), &RgSimulation::get_vehicle_gauge_info);
    godot::ClassDB::bind_method(D_METHOD("get_vehicle_powertrain", "vehicle_name"), &RgSimulation::get_vehicle_powertrain);
    godot::ClassDB::bind_method(D_METHOD("get_vehicle_ground_speed_mps", "vehicle_name"), &RgSimulation::get_vehicle_ground_speed_mps);
}

} // namespace rg_godot
