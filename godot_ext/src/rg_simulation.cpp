#include "rg_simulation.h"

#include "frame_convert.h"

#include "ps/drivetrain/powertrain_desc.h"

#include <godot_cpp/core/class_db.hpp>

#include <string>
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

bool RgSimulation::initialize(const String& vehicle_json_absolute_path, const String& surface_table_absolute_path) {
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

void RgSimulation::start() {
    if (session_) session_->start();
}

void RgSimulation::stop() {
    if (session_) session_->stop();
}

bool RgSimulation::is_running() const { return session_ && session_->running(); }

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
    godot::ClassDB::bind_method(D_METHOD("start"), &RgSimulation::start);
    godot::ClassDB::bind_method(D_METHOD("stop"), &RgSimulation::stop);
    godot::ClassDB::bind_method(D_METHOD("is_running"), &RgSimulation::is_running);
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
