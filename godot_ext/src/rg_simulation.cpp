#include "rg_simulation.h"

#include "frame_convert.h"

#include "ps/drivetrain/powertrain_desc.h"

#include <godot_cpp/core/class_db.hpp>

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

using godot::D_METHOD;
using godot::String;

namespace rg_godot {

namespace {

std::string to_std_string(const String& s) { return std::string(s.utf8().get_data()); }

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
    reap_init_thread(/*wait=*/true);
    session_.reset(); // Session's own destructor stops its loop
    origin_rebase_ = nullptr;
}

void RgSimulation::teardown_current() {
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
    if (init_phase_.load(std::memory_order_acquire) == InitPhase::Ready) {
        session_ = std::move(pending_session_);
        origin_rebase_ = &session_->origin_rebase();
    }
    pending_session_.reset(); // no-op on the Ready path (already moved out); frees nothing on Error (never set)
}

void RgSimulation::run_terrain_init_worker(std::string world_config_path, std::string vehicle_json_path,
                                           std::string surface_table_path) {
    try {
        std::string err;
        std::optional<rg::WorldConfig> world_config = rg::load_world_config(world_config_path, &err);
        if (!world_config.has_value()) {
            throw std::runtime_error("load_world_config: " + err);
        }

        std::string open_err;
        std::shared_ptr<rg::WorldTerrain> terrain(rg::WorldTerrain::open(*world_config, &open_err));
        if (terrain == nullptr) {
            throw std::runtime_error("WorldTerrain::open: " + open_err);
        }

        rg::SessionConfig config;
        config.vehicle_json_path = std::move(vehicle_json_path);
        config.surface_table_path = std::move(surface_table_path);
        config.terrain = rg::make_terrain_mode(*world_config, terrain); // start-up blocks inside the Session ctor below

        auto session = std::make_unique<rg::Session>(config);
        pending_session_ = std::move(session);
        init_message_ = "ready";
        init_phase_.store(InitPhase::Ready, std::memory_order_release);
    } catch (const std::exception& e) {
        pending_session_.reset();
        init_message_ = e.what();
        init_phase_.store(InitPhase::Error, std::memory_order_release);
    }
}

bool RgSimulation::initialize(const String& vehicle_json_absolute_path, const String& surface_table_absolute_path) {
    teardown_current();
    rg::SessionConfig config;
    config.vehicle_json_path = to_std_string(vehicle_json_absolute_path);
    config.surface_table_path = to_std_string(surface_table_absolute_path);
    try {
        session_ = std::make_unique<rg::Session>(config);
    } catch (const std::exception& e) {
        last_error_ = String(e.what());
        session_.reset();
        return false;
    }
    origin_rebase_ = &session_->origin_rebase();
    last_error_ = String();
    return true;
}

bool RgSimulation::initialize_terrain(const String& world_config_absolute_path,
                                      const String& vehicle_json_absolute_path,
                                      const String& surface_table_absolute_path) {
    teardown_current();
    last_error_ = String();
    init_message_.clear();
    init_phase_.store(InitPhase::Loading, std::memory_order_release);

    std::string world_config_path = to_std_string(world_config_absolute_path);
    std::string vehicle_json_path = to_std_string(vehicle_json_absolute_path);
    std::string surface_table_path = to_std_string(surface_table_absolute_path);
    init_thread_ = std::thread([this, world_config_path = std::move(world_config_path),
                               vehicle_json_path = std::move(vehicle_json_path),
                               surface_table_path = std::move(surface_table_path)]() mutable {
        run_terrain_init_worker(std::move(world_config_path), std::move(vehicle_json_path),
                                std::move(surface_table_path));
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
    if (phase == InitPhase::Ready && session_) {
        const rg::StreamingStatus s = session_->streaming_status();
        d["resident_l0"] = static_cast<std::int64_t>(s.resident_l0);
        d["missing_required"] = static_cast<std::int64_t>(s.missing_required);
    } else {
        // rg::Session's constructor is one blocking call with no progress
        // callback (see this method's own header doc comment) - no
        // finer-grained figure exists to report while "loading"/"error".
        d["resident_l0"] = static_cast<std::int64_t>(0);
        d["missing_required"] = static_cast<std::int64_t>(0);
    }
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
    return d;
}

godot::Vector3 RgSimulation::get_render_origin_session() const {
    if (!origin_rebase_) return godot::Vector3();
    const ps::Vec3 o = origin_rebase_->origin(); // RAW session-frame (east, north, up) - no basis conversion
    return godot::Vector3(static_cast<float>(o.x), static_cast<float>(o.y), static_cast<float>(o.z));
}

void RgSimulation::retry_failed_tiles() {
    if (session_) session_->retry_failed_tiles();
}

std::int64_t RgSimulation::get_step_count() const {
    return session_ ? static_cast<std::int64_t>(session_->snapshot().tick) : 0;
}

double RgSimulation::get_sim_time() const { return session_ ? session_->snapshot().sim_time : 0.0; }

double RgSimulation::get_tick_rate_hz() const { return session_ ? session_->world().dt() > 0.0 ? 1.0 / session_->world().dt() : 0.0 : 0.0; }

std::int64_t RgSimulation::get_body_count() const { return session_ ? 2 : 0; }

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

void RgSimulation::_bind_methods() {
    godot::ClassDB::bind_method(D_METHOD("initialize", "vehicle_json_absolute_path", "surface_table_absolute_path"), &RgSimulation::initialize);
    godot::ClassDB::bind_method(D_METHOD("initialize_terrain", "world_config_absolute_path", "vehicle_json_absolute_path", "surface_table_absolute_path"), &RgSimulation::initialize_terrain);
    godot::ClassDB::bind_method(D_METHOD("get_init_status"), &RgSimulation::get_init_status);
    godot::ClassDB::bind_method(D_METHOD("start"), &RgSimulation::start);
    godot::ClassDB::bind_method(D_METHOD("stop"), &RgSimulation::stop);
    godot::ClassDB::bind_method(D_METHOD("is_running"), &RgSimulation::is_running);
    godot::ClassDB::bind_method(D_METHOD("is_terrain_mode"), &RgSimulation::is_terrain_mode);
    godot::ClassDB::bind_method(D_METHOD("get_streaming_status"), &RgSimulation::get_streaming_status);
    godot::ClassDB::bind_method(D_METHOD("get_render_origin_session"), &RgSimulation::get_render_origin_session);
    godot::ClassDB::bind_method(D_METHOD("retry_failed_tiles"), &RgSimulation::retry_failed_tiles);
    godot::ClassDB::bind_method(D_METHOD("get_step_count"), &RgSimulation::get_step_count);
    godot::ClassDB::bind_method(D_METHOD("get_sim_time"), &RgSimulation::get_sim_time);
    godot::ClassDB::bind_method(D_METHOD("get_tick_rate_hz"), &RgSimulation::get_tick_rate_hz);
    godot::ClassDB::bind_method(D_METHOD("get_body_count"), &RgSimulation::get_body_count);
    godot::ClassDB::bind_method(D_METHOD("get_last_error"), &RgSimulation::get_last_error);

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

    godot::ClassDB::bind_method(D_METHOD("get_vehicle_gauge_info", "vehicle_name"), &RgSimulation::get_vehicle_gauge_info);
    godot::ClassDB::bind_method(D_METHOD("get_vehicle_powertrain", "vehicle_name"), &RgSimulation::get_vehicle_powertrain);
    godot::ClassDB::bind_method(D_METHOD("get_vehicle_ground_speed_mps", "vehicle_name"), &RgSimulation::get_vehicle_ground_speed_mps);
}

} // namespace rg_godot
