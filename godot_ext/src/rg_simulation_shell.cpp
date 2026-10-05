// godot_ext/src/rg_simulation_shell.cpp - the RgSimulation methods the game
// shell (PLAN.md R5) needs: tearing a world down inside a running process,
// pause, a picked spawn, the cinematic/bumper camera numbers and the
// session-frame -> Godot-frame conversion. Every decision (what the shot is,
// where the bumper sits) is rg_core's (rg/camera_math.h); this file only moves
// values between the Session snapshot and Godot Variants.
//
// No C++ exception is thrown or caught here (godot-cpp's -D_HAS_EXCEPTIONS=0).

#include "rg_simulation.h"

#include "frame_convert.h"

#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <cmath>
#include <string>

namespace rg_godot {

using godot::Dictionary;
using godot::String;
using godot::Vector3;

namespace {
double deg_from_rad(double r) { return r * 180.0 / 3.14159265358979323846; }
} // namespace

void RgSimulation::unload() {
    // teardown_current: cancel + join an init in flight, stop the engine
    // audio, stop the Session's sim thread and destroy it (streaming stops with
    // it), forget the drone target / walker bookkeeping.
    teardown_current();
    modes_.unload_world();
    cinematic_.reset();
    spawn_override_.reset();
    paused_ = false;
    road_ahead_wanted_ = false;
    last_error_ = String();
}

void RgSimulation::apply_shell_flags_to_session() {
    if (!session_) return;
    session_->set_paused(paused_);
    session_->set_road_ahead_wanted(road_ahead_wanted_);
}

void RgSimulation::set_paused(bool paused) {
    paused_ = paused;
    if (session_) session_->set_paused(paused);
}

void RgSimulation::set_spawn_override(double session_x, double session_y, double yaw_deg) {
    spawn_override_ = SpawnOverride{session_x, session_y, yaw_deg};
}

void RgSimulation::clear_spawn_override() { spawn_override_.reset(); }

void RgSimulation::set_store_dir_override(const String& dir) { store_dir_override_ = std::string(dir.utf8().get_data()); }

void RgSimulation::set_road_ahead_wanted(bool wanted) {
    road_ahead_wanted_ = wanted;
    if (session_) session_->set_road_ahead_wanted(wanted);
}

Dictionary RgSimulation::get_chassis_session_pose() const {
    Dictionary d;
    if (!session_) return d;
    const rg::FrameSnapshot& f = frame_snapshot();
    const ps::Vec3 forward = f.chassis_pose.orientation.rotate(ps::Vec3{1.0, 0.0, 0.0});
    d["x"] = f.chassis_pose.position.x;
    d["y"] = f.chassis_pose.position.y;
    d["z"] = f.chassis_pose.position.z;
    d["yaw_deg"] = deg_from_rad(std::atan2(forward.y, forward.x));
    d["speed_mps"] = f.chassis_motion.linear.length();
    return d;
}

Vector3 RgSimulation::session_to_godot(const Vector3& session_position) const {
    if (!origin_rebase_) return session_position;
    return iso_to_godot(ps::Vec3{session_position.x, session_position.y, session_position.z},
                        origin_rebase_->origin());
}

Dictionary RgSimulation::get_bumper_camera() const {
    Dictionary d;
    if (!session_) return d;
    const rg::BumperParams params;
    const ps::Vec3 eye = rg::bumper_eye_local(rg::vehicle_bounds(session_->vehicle_desc()), params);
    d["eye_local"] = Vector3(static_cast<float>(eye.x), static_cast<float>(eye.y), static_cast<float>(eye.z));
    d["pitch_down_deg"] = params.pitch_down_deg;
    d["fov_deg"] = params.fov_deg;
    return d;
}

Dictionary RgSimulation::update_cinematic(double delta) {
    Dictionary d;
    if (!session_) return d;
    const rg::FrameSnapshot& f = frame_snapshot();
    rg::CarKinematics car;
    car.x = f.chassis_pose.position.x;
    car.y = f.chassis_pose.position.y;
    car.vx = f.chassis_motion.linear.x;
    car.vy = f.chassis_motion.linear.y;
    const ps::Vec3 forward = f.chassis_pose.orientation.rotate(ps::Vec3{1.0, 0.0, 0.0});
    const double flat = std::sqrt(forward.x * forward.x + forward.y * forward.y);
    if (flat > 1e-6) {
        car.heading_x = forward.x / flat;
        car.heading_y = forward.y / flat;
    }
    const rg::CinematicShot& shot = cinematic_.update(delta, car, f.road_ahead);
    d["serial"] = static_cast<std::int64_t>(shot.serial);
    d["cut"] = cinematic_.cut_this_update();
    d["source"] = String(rg::to_string(shot.source));
    d["side"] = shot.side;
    d["session_x"] = shot.x;
    d["session_y"] = shot.y;
    d["height_above_ground_m"] = shot.height_above_ground_m;
    d["fov_deg"] = shot.fov_deg;
    return d;
}

void RgSimulation::reset_cinematic() { cinematic_.reset(); }

} // namespace rg_godot
