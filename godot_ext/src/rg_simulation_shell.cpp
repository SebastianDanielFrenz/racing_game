// godot_ext/src/rg_simulation_shell.cpp - the RgSimulation methods the game
// shell (PLAN.md R5) needs: tearing a world down inside a running process,
// pause, a picked spawn, the cinematic/bumper camera numbers and the
// session-frame -> Godot-frame conversion. Every decision (what the shot is,
// where the bumper sits) is rg_core's (rg/camera_math.h); this file only moves
// values between the Session snapshot and Godot Variants.
//
// No C++ exception is thrown or caught here (godot-cpp's -D_HAS_EXCEPTIONS=0).

#include "rg_simulation.h"
#include "rg_damage_convert.h"

#include "frame_convert.h"

#include "rg/route_check.h"

#include "g2m/world/height_sampler.h"

#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
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
    release_cinematic_obstacles();
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

void RgSimulation::set_vehicle_overrides(const godot::Dictionary& overrides) {
    VehicleOverrides o;
    if (overrides.has("half_extents") && overrides.has("z_m")) {
        const godot::Vector3 h = overrides["half_extents"];
        o.has_chassis = true;
        o.chassis_half_extents = ps::Vec3{h.x, h.y, h.z};
        o.chassis_z_m = static_cast<double>(overrides["z_m"]);
        if (overrides.has("mass_kg")) o.chassis_mass_kg = static_cast<double>(overrides["mass_kg"]);
    }
    if (overrides.has("engine_map_cache_dir"))
        o.engine_map_cache_dir = std::string(godot::String(overrides["engine_map_cache_dir"]).utf8().get_data());
    if (overrides.has("turbo_map_extension_path"))
        o.turbo_map_extension_path = std::string(godot::String(overrides["turbo_map_extension_path"]).utf8().get_data());
    if (overrides.has("damage_config_path"))
        o.damage_config_path = std::string(godot::String(overrides["damage_config_path"]).utf8().get_data());
    if (overrides.has("damage_state_path"))
        o.damage_state_path = std::string(godot::String(overrides["damage_state_path"]).utf8().get_data());
    o.damage_enabled = vehicle_overrides_.damage_enabled; // a setting, not part of the garage selection
    vehicle_overrides_ = std::move(o);
}

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
    d["vx"] = f.chassis_motion.linear.x;
    d["vy"] = f.chassis_motion.linear.y;
    d["vz"] = f.chassis_motion.linear.z;
    return d;
}

Vector3 RgSimulation::session_to_godot(const Vector3& session_position) const {
    if (!origin_rebase_) return session_position;
    return iso_to_godot(ps::Vec3{session_position.x, session_position.y, session_position.z},
                        origin_rebase_->origin());
}

Dictionary RgSimulation::get_bumper_camera(const String& rig_json_path, const Vector3& model_offset) const {
    Dictionary d;
    if (!session_) return d;
    const rg::BumperParams params;
    ps::Vec3 eye = rg::bumper_eye_local(rg::vehicle_bounds(session_->vehicle_desc()), params);
    String source = "wheels";
    if (!rig_json_path.is_empty()) {
        std::string err;
        const auto model = rg::load_model_bounds(std::string(rig_json_path.utf8().get_data()), &err);
        if (model) {
            eye = rg::bumper_eye_from_model(*model, ps::Vec3{model_offset.x, model_offset.y, model_offset.z}, params);
            source = model->bumper_socket ? "socket" : "model";
        } else {
            godot::UtilityFunctions::push_warning(String("bumper camera: ") + String(err.c_str()));
        }
    }
    d["source"] = source;
    d["eye_local"] = Vector3(static_cast<float>(eye.x), static_cast<float>(eye.y), static_cast<float>(eye.z));
    d["pitch_down_deg"] = params.pitch_down_deg;
    d["fov_deg"] = params.fov_deg;
    return d;
}

Dictionary RgSimulation::get_camera_motion_sample() const {
    Dictionary d;
    if (!session_) return d;
    const rg::FrameSnapshot& f = frame_snapshot();
    const ps::Pose pose = render_chassis_pose();
    auto v3 = [](const ps::Vec3& v) {
        return Vector3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
    };
    d["sim_time"] = f.sim_time;
    d["velocity"] = v3(f.chassis_motion.linear);
    d["forward"] = v3(pose.orientation.rotate(ps::Vec3{1.0, 0.0, 0.0}));
    d["left"] = v3(pose.orientation.rotate(ps::Vec3{0.0, 1.0, 0.0}));
    d["up"] = v3(pose.orientation.rotate(ps::Vec3{0.0, 0.0, 1.0}));
    godot::PackedFloat64Array c;
    for (const auto& w : f.wheels) c.push_back(static_cast<double>(w.state.suspension_travel));
    d["compression"] = c;
    return d;
}

Dictionary RgSimulation::update_cinematic(double delta) {
    Dictionary d;
    if (!session_) return d;
    const rg::FrameSnapshot& f = frame_snapshot();
    rg::CarKinematics car;
    const ps::Pose render_pose = render_chassis_pose(); // nominal-clock sample, the same pose the car is drawn at
    car.x = render_pose.position.x;
    car.y = render_pose.position.y;
    car.vx = f.chassis_motion.linear.x;
    car.vy = f.chassis_motion.linear.y;
    const ps::Vec3 forward = render_pose.orientation.rotate(ps::Vec3{1.0, 0.0, 0.0});
    const double flat = std::sqrt(forward.x * forward.x + forward.y * forward.y);
    if (flat > 1e-6) {
        car.heading_x = forward.x / flat;
        car.heading_y = forward.y / flat;
    }
    sync_cinematic_obstacles(car.x, car.y);
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

void RgSimulation::release_cinematic_obstacles() {
    cinematic_.set_obstacles(nullptr);
    cinematic_obstacles_.reset(); // joins the worker before the terrain it reads from can go
    cinematic_terrain_.reset();
}

// Building footprints and terrain for the cinematic director's occlusion checks (rg_core does the geometry; this only
// feeds it the world's own data). The 1 km OSM tiles load on a worker thread; until a tile has arrived the director
// frames from behind the car instead of trusting missing data. Flat world: no terrain, no obstacles.
void RgSimulation::sync_cinematic_obstacles(double car_x, double car_y) {
    const std::shared_ptr<rg::WorldTerrain> terrain = session_ ? session_->world_terrain() : nullptr;
    if (terrain != cinematic_terrain_) {
        release_cinematic_obstacles();
        cinematic_terrain_ = terrain;
        if (terrain) {
            const auto zone = terrain->frame().zone();
            const double e0 = terrain->frame().e0_m();
            const double n0 = terrain->frame().n0_m();
            rg::WorldTerrain* t = terrain.get();
            cinematic_obstacles_ = std::make_unique<rg::BuildingObstacles>(
                e0, n0,
                [t, zone](int tx, int ty) { return t->buildings_tile(g2m::TileKey{zone, 2, tx, ty}, 0.0, 8.0, 3.0); },
                [t, zone, e0, n0](double x, double y) {
                    return g2m::world::sample_height_bilinear([t](const auto& key) { return t->cached_height_tile(key); }, zone, e0, n0,
                                                x, y);
                });
        }
        cinematic_.set_obstacles(cinematic_obstacles_.get());
    }
    if (cinematic_obstacles_) cinematic_obstacles_->request_around(car_x, car_y);
}


// ---- damage model persistence (rg/damage.h) ----------------------------------------------------------------------

void RgSimulation::save_damage() {
    if (!session_) return;
    const std::string path = session_->damage_state_path();
    if (path.empty()) return;
    const rg::DamageReport r = session_->damage_report();
    if (!r.present || r.revision == damage_saved_revision_) return; // nothing changed since the load or the last save
    const auto now = std::chrono::steady_clock::now();
    std::string err;
    if (!rg::save_damage_state(path, r.state, &err)) {
        std::fprintf(stderr, "RG_DAMAGE state not saved: %s\n", err.c_str());
        return;
    }
    damage_saved_revision_ = r.revision;
    damage_saved_at_ = now;
}

godot::Dictionary RgSimulation::get_damage_state() {
    if (!session_ || session_->damage_config() == nullptr) {
        godot::Dictionary d = rg_godot::damage_view_to_dictionary(rg::DamageView{}, false);
        d["state_path"] = godot::String();
        d["thermal"] = false;
        return d;
    }
    const rg::DamageReport r = session_->damage_report();
    const rg::DamageConfig& cfg = *session_->damage_config();
    // Periodic save while driving, so a crash loses at most save_interval_s of wear.
    if (r.revision != damage_saved_revision_ &&
        std::chrono::steady_clock::now() - damage_saved_at_ >= std::chrono::duration<double>(cfg.save_interval_s)) {
        save_damage();
    }
    godot::Dictionary d = rg_godot::damage_view_to_dictionary(rg::damage_view(cfg, r.state), true);
    d["state_path"] = godot::String::utf8(session_->damage_state_path().c_str());
    d["thermal"] = r.thermal;
    d["enabled"] = r.enabled;
    // What the Session holds (the view's own seized follows the state alone; with the setting off nothing is held).
    d["seized"] = r.seized;
    d["seized_part"] = godot::String::utf8(r.seized_part.c_str());
    if (!r.enabled) d["warning"] = godot::String(); // stored damage has no effect: no HUD line
    return d;
}

bool RgSimulation::repair_damage(const godot::String& state_path) {
    if (!session_ || session_->damage_config() == nullptr) return false;
    const std::string path(state_path.utf8().get_data());
    if (!path.empty() && path != session_->damage_state_path()) return false;
    session_->request_damage_repair();
    save_damage(); // the file stays removed (healthy)
    return true;
}

void RgSimulation::set_damage_enabled(bool enabled) {
    vehicle_overrides_.damage_enabled = enabled;
    if (session_) session_->set_damage_enabled(enabled);
}


// ---- towing (rg/trailer.h) -------------------------------------------------------------------------------------------

namespace {
const char* trailer_state_name(rg::TrailerState s) {
    switch (s) {
        case rg::TrailerState::None: return "none";
        case rg::TrailerState::Standing: return "standing";
        case rg::TrailerState::Coupled: return "coupled";
    }
    return "none";
}
const char* hookup_verdict_name(rg::HookupVerdict v) {
    switch (v) {
        case rg::HookupVerdict::Ok: return "ok";
        case rg::HookupVerdict::NoPair: return "no_pair";
        case rg::HookupVerdict::TooFar: return "too_far";
        case rg::HookupVerdict::HeadingOff: return "heading_off";
        case rg::HookupVerdict::TooFast: return "too_fast";
        case rg::HookupVerdict::OutOfJackRange: return "out_of_jack_range";
        case rg::HookupVerdict::NoGeometry: return "no_geometry";
    }
    return "no_geometry";
}
godot::String utf8(const std::string& s) { return godot::String::utf8(s.c_str()); }
std::string std_utf8(const godot::String& s) { return std::string(s.utf8().get_data()); }
} // namespace

godot::String RgSimulation::set_trailer_config(const godot::String& config_path, const godot::String& state_path) {
    trailer_config_.reset();
    trailer_restore_ = rg::TrailerRecord{};
    trailer_state_path_ = std_utf8(state_path);
    const std::string path = std_utf8(config_path);
    if (path.empty()) return godot::String();
    std::string err;
    auto cfg = rg::load_trailer_config(path, &err);
    if (!cfg) return utf8(err.empty() ? std::string("trailer config not loaded") : err);
    trailer_config_ = std::make_shared<const rg::TrailerConfig>(std::move(*cfg));
    if (!trailer_state_path_.empty()) {
        std::string rerr;
        if (auto rec = rg::load_trailer_record(trailer_state_path_, &rerr)) trailer_restore_ = *rec; // a missing file = none
    }
    return godot::String();
}

void RgSimulation::capture_trailer_record() {
    if (!session_ || trailer_config_ == nullptr) return;
    const rg::FrameSnapshot& snap = session_->snapshot();
    if (snap.tick == 0) return; // never stepped: the stored record (restore pending) stays as it is
    const rg::TrailerSnapshot& t = snap.trailer;
    rg::TrailerRecord rec;
    rec.present = t.present;
    rec.kind = t.kind_id;
    rec.load = t.load_id;
    rec.coupled = t.state == rg::TrailerState::Coupled;
    trailer_restore_ = rec;
    if (trailer_state_path_.empty()) return;
    std::string err;
    if (!rg::save_trailer_record(trailer_state_path_, rec, &err)) std::fprintf(stderr, "RG_TRAILER state not saved: %s\n", err.c_str());
}

void RgSimulation::request_trailer_spawn(const godot::String& kind, const godot::String& load, bool coupled) {
    if (session_) session_->request_trailer_spawn(std_utf8(kind), std_utf8(load), coupled);
}
void RgSimulation::request_trailer_attach() { if (session_) session_->request_trailer_attach(); }
void RgSimulation::request_trailer_detach() { if (session_) session_->request_trailer_detach(); }
void RgSimulation::request_trailer_remove() { if (session_) session_->request_trailer_remove(); }
void RgSimulation::request_trailer_recover() { if (session_) session_->request_trailer_recover(); }

godot::Dictionary RgSimulation::get_trailer_state() const {
    Dictionary d;
    d["enabled"] = trailer_config_ != nullptr && session_ != nullptr && session_->trailer_config() != nullptr;
    d["present"] = false;
    d["state"] = godot::String("none");
    if (!session_) return d;
    const rg::TrailerSnapshot& t = frame_snapshot().trailer;
    d["message"] = utf8(t.last_message);
    if (!t.present) return d;
    d["present"] = true;
    d["state"] = godot::String(trailer_state_name(t.state));
    d["kind"] = utf8(t.kind_id);
    d["load"] = utf8(t.load_id);
    d["name"] = utf8(t.vehicle_name);
    d["serial"] = static_cast<std::int64_t>(t.serial);
    d["mass_kg"] = t.mass_kg;
    d["towed_mass_kg"] = t.towed_mass_kg;
    d["tongue_load_n"] = t.tongue_load_n;
    d["hitch_force_n"] = t.hitch_force_n;
    d["break_force_n"] = t.break_force_n;
    d["yaw_deg"] = t.yaw_deg;
    d["pitch_deg"] = t.pitch_deg;
    d["roll_deg"] = t.roll_deg;
    d["socket_error_m"] = t.socket_error_m;
    d["tow_brake"] = t.tow_brake;
    d["latch_progress"] = t.latch_progress;
    d["class"] = utf8(t.class_name);
    d["hookup_distance_m"] = t.hookup_distance_m;
    d["hookup_ok"] = t.hookup_verdict == rg::HookupVerdict::Ok;
    d["hookup_verdict"] = godot::String(hookup_verdict_name(t.hookup_verdict));
    d["jack_down"] = t.jack_down;
    d["attaches"] = static_cast<std::int64_t>(t.attaches);
    d["detaches"] = static_cast<std::int64_t>(t.detaches);
    d["overloads"] = static_cast<std::int64_t>(t.overloads);
    godot::Array findings;
    for (const rg::TrailerFinding& f : t.findings) findings.push_back(utf8(f.text));
    d["findings"] = findings;
    godot::Array wheels;
    for (std::size_t i = 0; i < t.wheels.size(); ++i) {
        Dictionary w;
        w["name"] = i < t.wheel_names.size() ? utf8(t.wheel_names[i]) : godot::String();
        w["spin_angle"] = t.wheels[i].spin_angle;
        w["compression"] = t.wheels[i].suspension_travel;
        w["load"] = t.wheels[i].load;
        w["steer_angle"] = t.wheels[i].steer_angle;
        wheels.push_back(w);
    }
    d["wheels"] = wheels;
    if (const rg::TrailerConfig* cfg = session_->trailer_config()) {
        if (const rg::TrailerKind* kind = cfg->find_kind(t.kind_id)) {
            d["model_offset"] = godot::Vector3(static_cast<float>(kind->model_offset.x), static_cast<float>(kind->model_offset.y),
                                               static_cast<float>(kind->model_offset.z));
            d["length_m"] = kind->length_m;
        }
    }
    return d;
}

godot::Transform3D RgSimulation::get_trailer_transform() const {
    if (!session_) return godot::Transform3D();
    const rg::TrailerSnapshot& t = frame_snapshot().trailer;
    if (!t.present) return godot::Transform3D();
    ps::Pose pose = t.pose;
    (void)render_chassis_pose(); // samples the frame's interpolated poses once (render_sample_)
    if (render_sample_fresh_ && render_sample_.poses.size() >= 2) pose = render_sample_.poses[1];
    return iso_to_godot_transform(pose, origin_rebase_ ? origin_rebase_->origin() : ps::Vec3{});
}

} // namespace rg_godot
