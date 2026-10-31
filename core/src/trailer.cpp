// rg/trailer.cpp - see trailer.h.
#include "rg/trailer.h"

#include "rg/vehicle_load.h"

#include "json_util.h"

#include "ps/backend/i_rigid_backend.h"
#include "ps/io/limit_policy_io.h"
#include "ps/machine/hitch_limits.h"
#include "ps/math/transcendental.h"
#include "ps/vehicle/tyre_desc.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

namespace rg {

namespace {

namespace fs = std::filesystem;
using detail::json;

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;

std::string resolve_path(const std::string& base_dir, const std::string& ref) {
    const fs::path p(ref);
    return (p.is_absolute() ? p : fs::path(base_dir) / p).lexically_normal().generic_string();
}

bool fail(std::string* err, const std::string& origin, const std::string& what) {
    if (err != nullptr) *err = origin + ": " + what;
    return false;
}

bool read_vec3(const json& o, const char* key, ps::Vec3& out) {
    const auto it = o.find(key);
    if (it == o.end() || !it->is_array() || it->size() != 3) return false;
    for (const json& e : *it)
        if (!e.is_number()) return false;
    out = ps::Vec3{(*it)[0].get<double>(), (*it)[1].get<double>(), (*it)[2].get<double>()};
    return true;
}

std::string number_text(double v, int decimals) {
    char buf[48];
    std::snprintf(buf, sizeof buf, "%.*f", decimals, v);
    return buf;
}

} // namespace

// ---- config ----------------------------------------------------------------------------------------------------------

const TrailerLoad* TrailerKind::find_load(const std::string& load_id) const {
    for (const TrailerLoad& l : loads)
        if (l.id == load_id) return &l;
    return nullptr;
}

const HitchClassRule* TrailerConfig::find_class(const std::string& name) const {
    for (const HitchClassRule& c : classes)
        if (c.name == name) return &c;
    return nullptr;
}

const TrailerKind* TrailerConfig::find_kind(const std::string& id) const {
    for (const TrailerKind& k : trailers)
        if (k.id == id) return &k;
    return nullptr;
}

std::optional<TrailerConfig> parse_trailer_config(const std::string& json_text, const std::string& base_dir, const std::string& origin,
                                                  std::string* err) {
    const auto parsed = detail::parse_json(json_text);
    if (!parsed || !parsed->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    const json& j = *parsed;
    std::string format;
    if (!detail::get_string(j, "format", format) || format != "rg.trailers/1") {
        fail(err, origin, "\"format\" must be \"rg.trailers/1\"");
        return std::nullopt;
    }
    TrailerConfig c;
    const auto number = [&](const json& o, const char* key, double& out, const char* where, double lo, bool lo_open) -> bool {
        if (!detail::get_number(o, key, out) || !std::isfinite(out) || out < lo || (lo_open && out <= lo)) {
            fail(err, origin, std::string(where) + "." + key + " must be a number " + (lo_open ? "above " : "of at least ") + number_text(lo, 3));
            return false;
        }
        return true;
    };
    const auto object = [&](const char* key) -> const json* {
        const auto it = j.find(key);
        return it != j.end() && it->is_object() ? &*it : nullptr;
    };

    if (!detail::get_string(j, "default_trailer", c.default_trailer) || c.default_trailer.empty()) {
        fail(err, origin, "\"default_trailer\" is required");
        return std::nullopt;
    }
    if (detail::get_string(j, "channel_scope", c.channel_scope) == false || c.channel_scope.empty() || c.channel_scope.find('.') != std::string::npos) {
        fail(err, origin, "\"channel_scope\" must be a non-empty text without dots");
        return std::nullopt;
    }
    if (!number(j, "brake_when_standing", c.brake_when_standing, "", 0.0, false) || c.brake_when_standing > 1.0) {
        if (err != nullptr && err->empty()) fail(err, origin, "brake_when_standing must be in [0, 1]");
        return std::nullopt;
    }

    const json* hookup = object("hookup");
    if (hookup == nullptr || !detail::get_bool(*hookup, "assist", c.hookup.assist) || !number(*hookup, "reach_m", c.hookup.reach_m, "hookup", 0.0, true) ||
        !number(*hookup, "max_heading_deg", c.hookup.max_heading_deg, "hookup", 0.0, true) ||
        !number(*hookup, "max_speed_m_s", c.hookup.max_speed_m_s, "hookup", 0.0, true) ||
        !number(*hookup, "max_vertical_m", c.hookup.max_vertical_m, "hookup", 0.0, false)) {
        if (err != nullptr && err->empty()) fail(err, origin, "\"hookup\" needs assist, reach_m, max_heading_deg, max_speed_m_s, max_vertical_m");
        return std::nullopt;
    }
    const json* unhook = object("unhook");
    if (unhook == nullptr || !number(*unhook, "max_speed_m_s", c.unhook.max_speed_m_s, "unhook", 0.0, false)) {
        if (err != nullptr && err->empty()) fail(err, origin, "\"unhook\" needs max_speed_m_s");
        return std::nullopt;
    }
    const json* spawn = object("spawn");
    if (spawn == nullptr || !number(*spawn, "gap_m", c.spawn.gap_m, "spawn", 0.0, false) || !number(*spawn, "clearance_m", c.spawn.clearance_m, "spawn", 0.0, false) ||
        !number(*spawn, "ray_height_m", c.spawn.ray_height_m, "spawn", 0.0, true) || !number(*spawn, "ray_range_m", c.spawn.ray_range_m, "spawn", 0.0, true)) {
        if (err != nullptr && err->empty()) fail(err, origin, "\"spawn\" needs gap_m, clearance_m, ray_height_m, ray_range_m");
        return std::nullopt;
    }
    const json* abandon = object("abandon");
    if (abandon == nullptr || !number(*abandon, "distance_m", c.abandon.distance_m, "abandon", 0.0, true) ||
        !number(*abandon, "below_m", c.abandon.below_m, "abandon", 0.0, true) ||
        !number(*abandon, "relocation_keep_m", c.abandon.relocation_keep_m, "abandon", 0.0, false)) {
        if (err != nullptr && err->empty()) fail(err, origin, "\"abandon\" needs distance_m, below_m, relocation_keep_m");
        return std::nullopt;
    }
    const json* park = object("park");
    if (park == nullptr || !number(*park, "offset_x_m", c.park.offset_x_m, "park", 0.0, false)) {
        if (err != nullptr && err->empty()) fail(err, origin, "\"park\" needs offset_x_m");
        return std::nullopt;
    }

    // Advice texts.
    const json* advice = object("advice");
    if (advice == nullptr) {
        fail(err, origin, "\"advice\" is required");
        return std::nullopt;
    }
    TrailerAdviceText& a = c.advice;
    for (const auto& field : std::initializer_list<std::pair<const char*, std::string*>>{
             {"tow_mass", &a.tow_mass}, {"tongue_load", &a.tongue_load}, {"overload", &a.overload}, {"hookup_ready", &a.hookup_ready},
             {"hookup_far", &a.hookup_far}, {"coupled", &a.coupled}, {"standing", &a.standing}, {"severity_near", &a.severity_near},
             {"severity_limiting", &a.severity_limiting}, {"severity_exceeded", &a.severity_exceeded}}) {
        if (!detail::get_string(*advice, field.first, *field.second)) {
            fail(err, origin, std::string("advice.") + field.first + " must be a text");
            return std::nullopt;
        }
    }

    // Hitch classes.
    const auto classes = j.find("classes");
    if (classes == j.end() || !classes->is_array() || classes->empty()) {
        fail(err, origin, "\"classes\" must list at least one hitch class");
        return std::nullopt;
    }
    for (const json& e : *classes) {
        HitchClassRule rule;
        if (!e.is_object() || !detail::get_string(e, "name", rule.name) || rule.name.empty() || !detail::get_string(e, "label", rule.label) ||
            !number(e, "break_d_multiple", rule.break_d_multiple, "classes[]", 0.0, true) || !number(e, "break_hold_s", rule.break_hold_s, "classes[]", 0.0, false)) {
            if (err != nullptr && err->empty()) fail(err, origin, "a class needs name, label, break_d_multiple, break_hold_s");
            return std::nullopt;
        }
        if (c.find_class(rule.name) != nullptr) {
            fail(err, origin, "class \"" + rule.name + "\" is listed twice");
            return std::nullopt;
        }
        c.classes.push_back(std::move(rule));
    }

    // Trailer kinds.
    const auto trailers = j.find("trailers");
    if (trailers == j.end() || !trailers->is_array() || trailers->empty()) {
        fail(err, origin, "\"trailers\" must list at least one trailer");
        return std::nullopt;
    }
    for (const json& e : *trailers) {
        TrailerKind k;
        std::string vehicle, model;
        if (!e.is_object() || !detail::get_string(e, "id", k.id) || k.id.empty() || !detail::get_string(e, "title", k.title) ||
            !detail::get_string(e, "vehicle", vehicle) || !detail::get_string(e, "model", model) || !read_vec3(e, "chassis_half_extents", k.chassis_half_extents) ||
            !detail::get_number(e, "chassis_friction", k.chassis_friction) || !read_vec3(e, "model_offset", k.model_offset) ||
            !detail::get_number(e, "length_m", k.length_m) || !detail::get_string(e, "default_load", k.default_load)) {
            fail(err, origin, "a trailer needs id, title, vehicle, model, chassis_half_extents, chassis_friction, model_offset, length_m, default_load, loads, jack");
            return std::nullopt;
        }
        if (c.find_kind(k.id) != nullptr) {
            fail(err, origin, "trailer \"" + k.id + "\" is listed twice");
            return std::nullopt;
        }
        k.vehicle_path = resolve_path(base_dir, vehicle);
        k.model_path = resolve_path(base_dir, model);
        const auto loads = e.find("loads");
        if (loads == e.end() || !loads->is_array() || loads->empty()) {
            fail(err, origin, "trailer \"" + k.id + "\": \"loads\" must list at least one load state");
            return std::nullopt;
        }
        for (const json& l : *loads) {
            TrailerLoad load;
            if (!l.is_object() || !detail::get_string(l, "id", load.id) || load.id.empty() || !detail::get_string(l, "label", load.label) ||
                !detail::get_number(l, "mass_kg", load.mass_kg) || !(load.mass_kg > 0.0) || !std::isfinite(load.mass_kg)) {
                fail(err, origin, "trailer \"" + k.id + "\": a load needs id, label and a positive mass_kg");
                return std::nullopt;
            }
            k.loads.push_back(std::move(load));
        }
        if (k.find_load(k.default_load) == nullptr) {
            fail(err, origin, "trailer \"" + k.id + "\": default_load \"" + k.default_load + "\" is not a listed load");
            return std::nullopt;
        }
        const auto jack = e.find("jack");
        if (jack == e.end() || !jack->is_object() || !read_vec3(*jack, "foot_local", k.jack.foot_local) ||
            !detail::get_number(*jack, "stiffness_n_m", k.jack.stiffness_n_m) || !detail::get_number(*jack, "damping_n_s_m", k.jack.damping_n_s_m) ||
            !detail::get_number(*jack, "max_force_n", k.jack.max_force_n) || !(k.jack.stiffness_n_m > 0.0) || k.jack.damping_n_s_m < 0.0 || !(k.jack.max_force_n > 0.0)) {
            fail(err, origin, "trailer \"" + k.id + "\": \"jack\" needs foot_local, a positive stiffness_n_m and max_force_n, damping_n_s_m >= 0");
            return std::nullopt;
        }
        c.trailers.push_back(std::move(k));
    }
    if (c.find_kind(c.default_trailer) == nullptr) {
        fail(err, origin, "default_trailer \"" + c.default_trailer + "\" is not a listed trailer");
        return std::nullopt;
    }

    // The limit policy that grades the hitch ratings.
    std::string policy_file;
    if (detail::get_string(j, "limit_policy", policy_file) && !policy_file.empty()) {
        c.limit_policy_file = resolve_path(base_dir, policy_file);
        try {
            c.policy = ps::io::load_limit_policy_json(c.limit_policy_file);
        } catch (const std::exception& ex) {
            fail(err, origin, std::string("limit policy: ") + ex.what());
            return std::nullopt;
        }
    }
    return c;
}

std::optional<TrailerConfig> load_trailer_config(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot read the file");
        return std::nullopt;
    }
    return parse_trailer_config(*text, fs::path(path).parent_path().generic_string(), path, err);
}

std::string render_trailer_text(const std::string& tpl, const std::map<std::string, std::string>& values) {
    std::string out;
    for (std::size_t i = 0; i < tpl.size(); ++i) {
        if (tpl[i] == '{') {
            const std::size_t close = tpl.find('}', i);
            if (close != std::string::npos) {
                const auto it = values.find(tpl.substr(i + 1, close - i - 1));
                if (it != values.end()) {
                    out += it->second;
                    i = close;
                    continue;
                }
            }
        }
        out += tpl[i];
    }
    return out;
}

// ---- geometry --------------------------------------------------------------------------------------------------------

double heading_yaw(const ps::Quat& orientation) {
    const ps::Vec3 fwd = orientation.rotate(ps::Vec3::unit_x());
    if (std::sqrt(fwd.x * fwd.x + fwd.y * fwd.y) >= 0.2) return ps::math::atan2(fwd.y, fwd.x);
    const ps::Vec3 left = orientation.rotate(ps::Vec3::unit_y());
    if (std::sqrt(left.x * left.x + left.y * left.y) >= 0.2) return ps::math::atan2(left.y, left.x) - 0.5 * kPi;
    return 0.0;
}

namespace {

// The first pair of towing/towed points that mate and whose class the config lists.
struct PairIndex {
    const ps::machine::HitchPointDesc* towing = nullptr;
    const ps::machine::HitchPointDesc* towed = nullptr;
};

}

ps::Pose align_trailer_pose(const ps::Pose& trailer, const ps::Vec3& eye_local, const ps::Vec3& pivot_local, const ps::Vec3& target) {
    ps::Pose p = trailer;
    for (int round = 0; round < 3; ++round) {
        ps::Vec3 eye = p.transform_point(eye_local);
        const ps::Vec3 pivot = p.transform_point(pivot_local);
        const ps::Vec3 lateral = p.orientation.rotate(ps::Vec3::unit_y());
        const ps::Vec3 arm = eye - pivot;
        // Height of the eye per radian of rotation about the lateral axis through the pivot (positive = nose down).
        const double rate = ps::cross(lateral, arm).z;
        const double dz = target.z - eye.z;
        if (std::fabs(rate) > 1.0e-6) {
            const ps::Quat rot = ps::Quat::from_axis_angle(lateral, dz / rate);
            p.orientation = (rot * p.orientation).normalized();
            p.position = pivot + rot.rotate(p.position - pivot);
        }
        eye = p.transform_point(eye_local);
        p.position.x += target.x - eye.x;
        p.position.y += target.y - eye.y;
    }
    const ps::Vec3 eye = p.transform_point(eye_local);
    p.position.z += target.z - eye.z; // sub-millimetre residue
    return p;
}

HookupPlan plan_hookup(const TrailerConfig& config, const HookupGeometry& g) {
    HookupPlan plan;
    if (g.towing_points == nullptr || g.towed_points == nullptr) return plan;
    double best = 1.0e300;
    const ps::machine::HitchPointDesc* towing = nullptr;
    const ps::machine::HitchPointDesc* towed = nullptr;
    for (const ps::machine::HitchPointDesc& tp : *g.towing_points) {
        if (tp.role != ps::machine::HitchRole::Towing || config.find_class(tp.hitch.name) == nullptr) continue;
        for (const ps::machine::HitchPointDesc& wp : *g.towed_points) {
            if (wp.role != ps::machine::HitchRole::Towed || wp.hitch.name != tp.hitch.name) continue;
            const double d = ps::distance(g.car.transform_point(tp.position_local), g.trailer.transform_point(wp.position_local));
            if (d < best) {
                best = d;
                towing = &tp;
                towed = &wp;
            }
        }
    }
    if (towing == nullptr) {
        plan.verdict = HookupVerdict::NoPair;
        plan.message = "This car and this trailer have no hitch that fits.";
        return plan;
    }
    const ps::Vec3 socket = g.car.transform_point(towing->position_local);
    const ps::Vec3 eye = g.trailer.transform_point(towed->position_local);
    plan.towing_point = towing->name;
    plan.towed_point = towed->name;
    plan.class_name = towing->hitch.name;
    plan.distance_m = best;
    plan.vertical_m = socket.z - eye.z;
    plan.latch_error_m = towing->hitch.max_latch_error_m;
    plan.within_latch = best <= towing->hitch.max_latch_error_m;
    {
        const double a = heading_yaw(g.car.orientation), b = heading_yaw(g.trailer.orientation);
        double diff = std::fabs(a - b);
        while (diff > 2.0 * kPi) diff -= 2.0 * kPi;
        if (diff > kPi) diff = 2.0 * kPi - diff;
        plan.heading_deg = diff * 180.0 / kPi;
    }
    plan.speed_m_s = std::max({g.car_velocity.length(), g.trailer_velocity.length(), (g.car_velocity - g.trailer_velocity).length()});
    plan.aligned = align_trailer_pose(g.trailer, towed->position_local, g.trailer_pivot_local, socket);

    const TrailerConfig::Hookup& h = config.hookup;
    if (plan.speed_m_s > h.max_speed_m_s) {
        plan.verdict = HookupVerdict::TooFast;
        plan.message = "Stop first: the car and the trailer must stand still to hook up.";
    } else if (plan.distance_m > h.reach_m) {
        plan.verdict = HookupVerdict::TooFar;
        plan.message = "The trailer is too far from the tow point (" + number_text(plan.distance_m, 1) + " m).";
    } else if (plan.heading_deg > h.max_heading_deg) {
        plan.verdict = HookupVerdict::HeadingOff;
        plan.message = "The trailer does not point the way the car does.";
    } else if (!plan.within_latch && std::fabs(plan.vertical_m) > h.max_vertical_m) {
        plan.verdict = HookupVerdict::OutOfJackRange;
        plan.message = "The hitch is " + number_text(std::fabs(plan.vertical_m) * 100.0, 0) + " cm off in height: more than the jack can wind.";
    } else {
        plan.verdict = HookupVerdict::Ok;
    }
    return plan;
}

double iso_d_value_n(double towing_mass_kg, double trailer_mass_kg) {
    const double sum = towing_mass_kg + trailer_mass_kg;
    return sum > 0.0 ? kGravity * towing_mass_kg * trailer_mass_kg / sum : 0.0;
}

bool HitchOverload::update(double force_n, double limit_n, double hold_s, double dt_s) {
    if (!(force_n > limit_n)) {
        over_s_ = 0.0;
        return false;
    }
    over_s_ += dt_s;
    if (over_s_ + 1.0e-9 >= hold_s) { // the tolerance: 12 ticks of 1/240 s sum to 0.04999999999999999
        over_s_ = 0.0;
        return true;
    }
    return false;
}

ps::Wrench TrailerJack::evaluate(const ps::ForceElementContext& ctx) const {
    if (!deployed_ || !have_ground_) return ps::Wrench{};
    const ps::Vec3 foot = ctx.frame_pose.transform_point(spec_.foot_local);
    const double depth = ground_z_ - foot.z;
    if (!(depth > 0.0)) return ps::Wrench{};
    const ps::Vec3 arm = foot - ctx.frame_pose.position;
    const ps::Vec3 velocity = ctx.frame_motion.linear + ps::cross(ctx.frame_motion.angular, arm);
    const double f = std::clamp(spec_.stiffness_n_m * depth - spec_.damping_n_s_m * velocity.z, 0.0, spec_.max_force_n);
    const ps::Vec3 force{0.0, 0.0, f};
    return ps::Wrench{force, ps::cross(arm, force)};
}

// ---- the rig ---------------------------------------------------------------------------------------------------------

TrailerRig::TrailerRig(std::shared_ptr<const TrailerConfig> config) : config_(std::move(config)) {}

TrailerRig::~TrailerRig() = default;

const TrailerKind* TrailerRig::current_kind() const { return config_->find_kind(kind_id_); }

const ps::vehicle::VehicleDesc& TrailerRig::desc_for(const TrailerKind& kind) {
    auto it = descs_.find(kind.id);
    if (it != descs_.end()) return it->second.desc;
    KindCache cache;
    cache.desc = rg::load_vehicle(kind.vehicle_path);
    for (auto& wheel : cache.desc.wheels)
        ps::vehicle::select_rolling_resistance_speed_law(wheel.tyre, rolling_law_);
    return descs_.emplace(kind.id, std::move(cache)).first->second.desc;
}

bool TrailerRig::ground_z(const ps::World& world, double x, double y, double above_z, double* out) const {
    const ps::RayCastHit hit = world.backend().ray_cast(ps::Vec3{x, y, above_z}, ps::Vec3{0.0, 0.0, -1.0}, config_->spawn.ray_range_m);
    if (!hit.hit) return false;
    *out = hit.point.z;
    return true;
}

void TrailerRig::set_brake(ps::World& world, double value) {
    if (!brake_channel_.empty()) world.set_control(brake_channel_, value);
}

HookupGeometry TrailerRig::geometry_for(const ps::World& world, const Car& car) const {
    HookupGeometry g;
    if (car.desc == nullptr || active_desc_ == nullptr) return g;
    g.towing_points = &car.desc->hitch_points;
    g.towed_points = &active_desc_->hitch_points;
    g.car = world.get_pose(car.body);
    g.trailer = world.get_pose(body_);
    g.car_velocity = world.get_motion(car.body).linear;
    g.trailer_velocity = world.get_motion(body_).linear;
    // The middle of the axles: x and z of the wheel attachments, on the centre line.
    ps::Vec3 mean{};
    for (const auto& w : active_desc_->wheels) mean += w.attachment_local;
    if (!active_desc_->wheels.empty()) mean = mean / static_cast<double>(active_desc_->wheels.size());
    g.trailer_pivot_local = ps::Vec3{mean.x, 0.0, mean.z};
    return g;
}

TrailerResult TrailerRig::place_standing(ps::World& world, const Car& car, double gap_m) {
    const TrailerKind* kind = current_kind();
    if (kind == nullptr || active_desc_ == nullptr || car.desc == nullptr) return {false, "No trailer."};
    // The first pair of a car tow point and a trailer eye that mate.
    const ps::machine::HitchPointDesc* tp = nullptr;
    const ps::machine::HitchPointDesc* wp = nullptr;
    for (const auto& a : car.desc->hitch_points) {
        if (a.role != ps::machine::HitchRole::Towing || config_->find_class(a.hitch.name) == nullptr) continue;
        for (const auto& b : active_desc_->hitch_points) {
            if (b.role == ps::machine::HitchRole::Towed && b.hitch.name == a.hitch.name) {
                tp = &a;
                wp = &b;
                break;
            }
        }
        if (tp != nullptr) break;
    }
    if (tp == nullptr) return {false, "This car has no tow bar that fits the trailer."};

    const ps::Pose carp = world.get_pose(car.body);
    const double yaw = heading_yaw(carp.orientation);
    const ps::Quat q = ps::Quat::from_axis_angle(ps::Vec3::unit_z(), yaw);
    const ps::Vec3 socket = carp.transform_point(tp->position_local);
    const ps::Vec3 fwd = q.rotate(ps::Vec3::unit_x());
    const ps::Vec3 eye_off = q.rotate(wp->position_local);
    const double ox = socket.x - eye_off.x - fwd.x * gap_m;
    const double oy = socket.y - eye_off.y - fwd.y * gap_m;
    double ground_max = -1.0e300, rest = 0.0;
    for (const auto& w : active_desc_->wheels) {
        const ps::Vec3 off = q.rotate(ps::Vec3{w.attachment_local.x, w.attachment_local.y, 0.0});
        double gz = 0.0;
        if (!ground_z(world, ox + off.x, oy + off.y, socket.z + config_->spawn.ray_height_m, &gz))
            return {false, "There is no ground behind the car to put a trailer on."};
        ground_max = std::max(ground_max, gz);
        rest += w.wheel_radius - w.attachment_local.z;
    }
    if (!active_desc_->wheels.empty()) rest /= static_cast<double>(active_desc_->wheels.size());
    ps::Pose pose;
    pose.position = ps::Vec3{ox, oy, ground_max + rest + config_->spawn.clearance_m};
    pose.orientation = q;
    world.backend().set_pose(body_, pose);
    world.backend().set_motion(body_, ps::Motion{});
    note_teleport();
    return {true, {}};
}

TrailerResult TrailerRig::spawn(ps::World& world, const Car& car, const std::string& kind_id, const std::string& load_id, bool coupled) {
    const TrailerKind* kind = config_->find_kind(kind_id.empty() ? config_->default_trailer : kind_id);
    if (kind == nullptr) return refuse("Unknown trailer \"" + kind_id + "\".");
    const TrailerLoad* load = kind->find_load(load_id.empty() ? kind->default_load : load_id);
    if (load == nullptr) return refuse("Unknown load \"" + load_id + "\" for the " + kind->title + ".");
    if (car.desc == nullptr) return refuse("No car.");
    if (state_ != TrailerState::None) remove(world);
    const ps::vehicle::VehicleDesc* desc = nullptr;
    try {
        desc = &desc_for(*kind);
    } catch (const std::exception& e) {
        last_message_ = std::string("The trailer file failed to load: ") + e.what();
        return {false, last_message_};
    }
    ps::BodyDesc body;
    body.shape = ps::BoxShape{kind->chassis_half_extents};
    body.motion = ps::BodyMotionType::Dynamic;
    body.mass = load->mass_kg;
    body.friction = kind->chassis_friction;
    body.allow_sleep = false;
    body.pose.position = world.get_pose(car.body).position + ps::Vec3{0.0, 0.0, 30.0}; // replaced right below, away from the ground
    body_ = world.create_body(body);
    // The physics keeps a retired vehicle's channel scope reserved for good, so every trailer gets its own: trailer1, trailer2...
    ++serial_;
    vehicle_ = world.create_vehicle(*desc, body_, {}, config_->channel_scope + std::to_string(serial_));
    brake_channel_ = world.vehicle_channel(vehicle_, "brake");
    active_desc_ = desc;
    kind_id_ = kind->id;
    load_id_ = load->id;
    mass_kg_ = load->mass_kg;
    jack_ = std::make_unique<TrailerJack>(ps::Frame(body_), kind->jack);
    jack_id_ = world.add_force_element(jack_.get());
    state_ = TrailerState::Standing;
    hitch_ = ps::HitchId{};
    overload_.reset();
    set_brake(world, config_->brake_when_standing);

    TrailerResult placed = place_standing(world, car, coupled ? 0.0 : config_->spawn.gap_m);
    if (!placed.ok) {
        remove(world);
        last_message_ = placed.message;
        return placed;
    }
    last_message_ = kind->title + " (" + load->label + ") put behind the car.";
    if (coupled) {
        TrailerResult attached = attach(world, car);
        if (!attached.ok) return attached;
    }
    return {true, last_message_};
}

TrailerResult TrailerRig::attach(ps::World& world, const Car& car) {
    if (state_ == TrailerState::None) return refuse("There is no trailer to hook up.");
    if (state_ == TrailerState::Coupled) return refuse("The trailer is already hooked up.");
    const HookupGeometry g = geometry_for(world, car);
    const HookupPlan plan = plan_hookup(*config_, g);
    // A trailer that was just put at the tow point (spawn coupled, recover, relocation) has its speed and distance right by
    // construction; the checks below are for a player's hook-up.
    if (plan.verdict != HookupVerdict::Ok) {
        last_message_ = plan.message;
        return {false, plan.message};
    }
    const bool needs_move = !plan.within_latch;
    if (needs_move) {
        if (!config_->hookup.assist) {
            last_message_ = "The tow ball is " + number_text(plan.distance_m * 100.0, 0) + " cm from the coupling; back up to within " +
                            number_text(plan.latch_error_m * 100.0, 0) + " cm.";
            return {false, last_message_};
        }
        world.backend().set_pose(body_, plan.aligned);
        world.backend().set_motion(body_, ps::Motion{});
        note_teleport();
    }
    const ps::machine::AttachCheck check = world.check_attach(car.vehicle, plan.towing_point, vehicle_, plan.towed_point);
    if (!check.ok) {
        last_message_ = check.message.empty() ? std::string("The hitch would not latch.") : check.message;
        return {false, last_message_};
    }
    hitch_ = world.attach(car.vehicle, plan.towing_point, vehicle_, plan.towed_point);
    state_ = TrailerState::Coupled;
    towing_point_ = plan.towing_point;
    towed_point_ = plan.towed_point;
    class_name_ = plan.class_name;
    const HitchClassRule* rule = config_->find_class(plan.class_name);
    const double d = iso_d_value_n(world.backend().get_mass(car.body), mass_kg_);
    break_force_n_ = rule != nullptr ? rule->break_d_multiple * d : 0.0;
    overload_.reset();
    jack_->set_ground(false, false, 0.0);
    set_brake(world, 0.0);
    ++attaches_;
    const TrailerKind* kind = current_kind();
    last_message_ = "Hooked up the " + (kind != nullptr ? kind->title : std::string("trailer")) + ".";
    return {true, last_message_};
}

TrailerResult TrailerRig::detach(ps::World& world, const Car& car, bool ignore_speed) {
    if (state_ != TrailerState::Coupled) return refuse("The trailer is not hooked up.");
    const double speed = world.get_motion(car.body).linear.length();
    if (!ignore_speed && speed > config_->unhook.max_speed_m_s) {
        last_message_ = "Stop first: the trailer is let go standing still.";
        return {false, last_message_};
    }
    if (world.hitch_alive(hitch_)) world.detach(hitch_);
    hitch_ = ps::HitchId{};
    state_ = TrailerState::Standing;
    set_brake(world, config_->brake_when_standing);
    ++detaches_;
    last_message_ = "Trailer let go.";
    return {true, last_message_};
}

void TrailerRig::destroy(ps::World& world) {
    if (state_ == TrailerState::None) return;
    if (state_ == TrailerState::Coupled && world.hitch_alive(hitch_)) world.detach(hitch_);
    world.destroy_vehicle(vehicle_);
    world.remove_force_element(jack_id_);
    world.destroy_body(body_);
    jack_.reset();
    state_ = TrailerState::None;
    hitch_ = ps::HitchId{};
    vehicle_ = ps::VehicleId{};
    body_ = ps::BodyId{};
    jack_id_ = ps::ForceElementId{};
    active_desc_ = nullptr;
    brake_channel_.clear();
    ++removed_;
}

TrailerResult TrailerRig::remove(ps::World& world) {
    if (state_ == TrailerState::None) return {false, "There is no trailer."}; // not remembered: remove() runs in the destructor too
    destroy(world);
    last_message_ = "Trailer removed.";
    return {true, last_message_};
}

TrailerResult TrailerRig::recover(ps::World& world, const Car& car) {
    if (state_ == TrailerState::None) return refuse("There is no trailer.");
    if (state_ == TrailerState::Coupled) detach(world, car, true);
    TrailerResult placed = place_standing(world, car, 0.0);
    if (!placed.ok) return placed;
    // The trailer was put at the tow point: every check of a player's hook-up is satisfied by construction.
    TrailerResult attached = attach(world, car);
    if (attached.ok) last_message_ = "Trailer put back behind the car.";
    return attached.ok ? TrailerResult{true, last_message_} : attached;
}

void TrailerRig::before_step(ps::World& world, const Car& car) {
    if (state_ == TrailerState::None) return;
    if (state_ == TrailerState::Coupled) {
        set_brake(world, 0.0);
        return;
    }
    // Standing: the parking leg and the parking brake.
    const ps::Pose pose = world.get_pose(body_);
    const ps::Vec3 foot = pose.transform_point(jack_->spec().foot_local);
    double gz = 0.0;
    const bool have = ground_z(world, foot.x, foot.y, foot.z + 2.0, &gz);
    jack_->set_ground(true, have, gz);
    set_brake(world, config_->brake_when_standing);
    // Left behind, or fallen through: nothing keeps its ground loaded any more.
    const ps::Vec3 cp = world.get_pose(car.body).position;
    const double dx = pose.position.x - cp.x, dy = pose.position.y - cp.y;
    if (!relocating_ && (std::sqrt(dx * dx + dy * dy) > config_->abandon.distance_m || pose.position.z < cp.z - config_->abandon.below_m)) {
        destroy(world);
        last_message_ = "The trailer was left behind.";
    }
}

void TrailerRig::after_step(ps::World& world, const Car& car, double dt_s) {
    if (state_ != TrailerState::Coupled) return;
    if (!world.hitch_alive(hitch_)) {
        state_ = TrailerState::Standing;
        set_brake(world, config_->brake_when_standing);
        last_message_ = "The hitch is gone.";
        return;
    }
    const ps::machine::CouplingState& cs = world.coupling_state(hitch_);
    if (cs.latch_progress < 1.0) return; // the ramp that brings the sockets together is not a load
    const HitchClassRule* rule = config_->find_class(class_name_);
    if (rule == nullptr || break_force_n_ <= 0.0) return;
    if (overload_.update(cs.mean_force_on_towing_world.length(), break_force_n_, rule->break_hold_s, dt_s)) {
        const double force = cs.mean_force_on_towing_world.length();
        std::map<std::string, std::string> values{{"force", number_text(force / 1000.0, 0) + " kN"}, {"break_force", number_text(break_force_n_ / 1000.0, 0) + " kN"}};
        detach(world, car, true);
        ++overloads_;
        last_message_ = render_trailer_text(config_->advice.overload, values);
    }
}

void TrailerRig::begin_relocation(ps::World& world, const Car& car) {
    if (state_ == TrailerState::None) return;
    relocating_ = true;
    relocating_was_coupled_ = state_ == TrailerState::Coupled;
    saved_pose_ = world.get_pose(body_);
    if (relocating_was_coupled_) {
        if (world.hitch_alive(hitch_)) world.detach(hitch_);
        hitch_ = ps::HitchId{};
        state_ = TrailerState::Standing;
    }
    (void)car;
}

void TrailerRig::park(ps::World& world, const ps::Pose& car_park_pose) {
    if (!relocating_ || state_ == TrailerState::None) return;
    ps::Pose p = car_park_pose;
    p.position.x += config_->park.offset_x_m;
    world.backend().set_pose(body_, p);
    world.backend().set_motion(body_, ps::Motion{});
}

TrailerResult TrailerRig::finish_relocation(ps::World& world, const Car& car) {
    if (!relocating_ || state_ == TrailerState::None) {
        relocating_ = false;
        return {true, {}};
    }
    relocating_ = false;
    if (relocating_was_coupled_) {
        TrailerResult placed = place_standing(world, car, 0.0);
        if (!placed.ok) {
            destroy(world);
            last_message_ = placed.message;
            return placed;
        }
        TrailerResult attached = attach(world, car);
        if (!attached.ok) last_message_ = attached.message;
        return attached;
    }
    const ps::Vec3 cp = world.get_pose(car.body).position;
    const double dx = saved_pose_.position.x - cp.x, dy = saved_pose_.position.y - cp.y;
    if (std::sqrt(dx * dx + dy * dy) <= config_->abandon.relocation_keep_m) {
        world.backend().set_pose(body_, saved_pose_);
        world.backend().set_motion(body_, ps::Motion{});
        note_teleport();
        return {true, {}};
    }
    destroy(world);
    last_message_ = "The trailer was left behind.";
    return {true, last_message_};
}

void TrailerRig::abort_relocation(ps::World& world) {
    relocating_ = false;
    if (state_ != TrailerState::None) {
        destroy(world);
        last_message_ = "The trailer was left behind.";
    }
}

std::vector<TrailerFinding> tow_findings(const TrailerConfig& config, const std::vector<ps::drivetrain::LimitFinding>& raw, const std::string& car_name,
                                        const std::string& trailer_title) {
    std::vector<TrailerFinding> out;
    for (const ps::drivetrain::LimitFinding& f : raw) {
        if (f.kind != ps::drivetrain::LimitKind::TowedMass && f.kind != ps::drivetrain::LimitKind::TongueLoad) continue;
        if (!config.policy.warns(f.severity)) continue;
        TrailerFinding t;
        t.kind = f.kind;
        t.severity = f.severity;
        t.value = f.value;
        t.limit = f.limit;
        const std::string& severity = f.severity == ps::drivetrain::LimitSeverity::Near       ? config.advice.severity_near
                                      : f.severity == ps::drivetrain::LimitSeverity::Limiting ? config.advice.severity_limiting
                                                                                             : config.advice.severity_exceeded;
        std::map<std::string, std::string> values{{"severity", severity}, {"car", car_name}, {"trailer", trailer_title}};
        if (f.kind == ps::drivetrain::LimitKind::TowedMass) {
            values["mass"] = number_text(f.value, 0) + " kg";
            values["limit_mass"] = number_text(f.limit, 0) + " kg";
            t.text = render_trailer_text(config.advice.tow_mass, values);
        } else {
            values["tongue"] = number_text(f.value, 0) + " N";
            values["limit_tongue"] = number_text(f.limit, 0) + " N";
            t.text = render_trailer_text(config.advice.tongue_load, values);
        }
        out.push_back(std::move(t));
    }
    return out;
}

TrailerSnapshot TrailerRig::snapshot(const ps::World& world, const Car& car) const {
    TrailerSnapshot s;
    s.last_message = last_message_;
    s.attaches = attaches_;
    s.detaches = detaches_;
    s.overloads = overloads_;
    s.removed = removed_;
    s.serial = serial_;
    if (state_ == TrailerState::None || active_desc_ == nullptr) return s;
    s.present = true;
    s.state = state_;
    s.kind_id = kind_id_;
    s.load_id = load_id_;
    s.vehicle_name = active_desc_->name;
    s.pose = world.get_pose(body_);
    s.motion = world.get_motion(body_);
    s.mass_kg = mass_kg_;
    s.jack_down = state_ == TrailerState::Standing;
    const std::size_t wheels = world.vehicle_wheel_count(vehicle_);
    s.wheels.reserve(wheels);
    s.wheel_names.reserve(wheels);
    for (std::size_t i = 0; i < wheels; ++i) {
        s.wheels.push_back(world.wheel_state(vehicle_, i));
        s.wheel_names.push_back(world.wheel_name(vehicle_, i));
    }
    const TrailerKind* kind = current_kind();
    const std::string title = kind != nullptr ? kind->title : std::string("trailer");
    if (state_ == TrailerState::Coupled && world.hitch_alive(hitch_)) {
        s.towing_point = towing_point_;
        s.towed_point = towed_point_;
        s.class_name = class_name_;
        const ps::machine::CouplingState& cs = world.coupling_state(hitch_);
        s.hitch_force_n = cs.mean_force_on_towing_world.length();
        s.break_force_n = break_force_n_;
        s.yaw_deg = cs.yaw_rad * 180.0 / kPi;
        s.pitch_deg = cs.pitch_rad * 180.0 / kPi;
        s.roll_deg = cs.roll_rad * 180.0 / kPi;
        s.socket_error_m = cs.max_error_m;
        s.tow_brake = cs.tow_brake;
        s.latch_progress = cs.latch_progress;
        const ps::machine::HitchLoads loads = world.hitch_loads(hitch_);
        s.towed_mass_kg = loads.towed_mass_kg;
        s.tongue_load_n = loads.tongue_load_n;
        s.findings = tow_findings(*config_, world.hitch_limits(hitch_, &config_->policy), car.desc != nullptr ? car.desc->name : std::string("car"), title);
    } else if (state_ == TrailerState::Standing && !relocating_) {
        const HookupGeometry g = geometry_for(world, car);
        const HookupPlan plan = plan_hookup(*config_, g);
        s.hookup_distance_m = plan.distance_m;
        s.hookup_verdict = plan.verdict;
        s.towing_point = plan.towing_point;
        s.towed_point = plan.towed_point;
        s.class_name = plan.class_name;
    }
    return s;
}

// ---- persistence and the server rule ---------------------------------------------------------------------------------

std::string trailer_record_to_json(const TrailerRecord& record) {
    json j;
    j["format"] = "rg.trailer_state/1";
    j["present"] = record.present;
    j["kind"] = record.kind;
    j["load"] = record.load;
    j["coupled"] = record.coupled;
    return j.dump(2) + "\n";
}

std::optional<TrailerRecord> parse_trailer_record(const std::string& json_text, std::string* err) {
    const auto parsed = detail::parse_json(json_text);
    if (!parsed || !parsed->is_object()) {
        if (err != nullptr) *err = "trailer state: not a JSON object";
        return std::nullopt;
    }
    std::string format;
    TrailerRecord r;
    if (!detail::get_string(*parsed, "format", format) || format != "rg.trailer_state/1" || !detail::get_bool(*parsed, "present", r.present) ||
        !detail::get_string(*parsed, "kind", r.kind) || !detail::get_string(*parsed, "load", r.load) || !detail::get_bool(*parsed, "coupled", r.coupled)) {
        if (err != nullptr) *err = "trailer state: needs format rg.trailer_state/1, present, kind, load, coupled";
        return std::nullopt;
    }
    return r;
}

bool save_trailer_record(const std::string& path, const TrailerRecord& record, std::string* err) {
    std::error_code ec;
    fs::create_directories(fs::path(path).parent_path(), ec);
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        if (err != nullptr) *err = path + ": cannot write";
        return false;
    }
    out << trailer_record_to_json(record);
    out.flush();
    if (!out) {
        if (err != nullptr) *err = path + ": write failed";
        return false;
    }
    return true;
}

std::optional<TrailerRecord> load_trailer_record(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        if (err != nullptr) *err = path + ": cannot read";
        return std::nullopt;
    }
    return parse_trailer_record(*text, err);
}

CoupledSetAuthority::VehicleKey CoupledSetAuthority::root_of(VehicleKey v) const {
    for (int guard = 0; guard < 16; ++guard) {
        const auto it = towed_by_.find(v);
        if (it == towed_by_.end()) return v;
        v = it->second;
    }
    return v;
}

bool CoupledSetAuthority::register_vehicle(VehicleKey vehicle, PeerId peer) {
    const auto it = owner_.find(vehicle);
    if (it != owner_.end()) return it->second == peer;
    owner_[vehicle] = peer;
    return true;
}

bool CoupledSetAuthority::couple(VehicleKey towing, VehicleKey trailer, PeerId peer) {
    if (towing == trailer) return false;
    const auto t = owner_.find(towing);
    if (t == owner_.end() || t->second != peer) return false; // only the towing vehicle's owner builds the set
    if (towed_by_.count(trailer) != 0) return false;          // already someone's trailer
    const auto r = owner_.find(trailer);
    if (r != owner_.end() && r->second != peer) return false; // another peer's vehicle
    if (root_of(towing) == trailer) return false;             // a loop
    owner_[trailer] = peer;
    towed_by_[trailer] = towing;
    return true;
}

bool CoupledSetAuthority::uncouple(VehicleKey trailer, PeerId peer) {
    const auto it = towed_by_.find(trailer);
    if (it == towed_by_.end()) return false;
    if (authority(trailer) != peer) return false;
    towed_by_.erase(it);
    return true;
}

CoupledSetAuthority::PeerId CoupledSetAuthority::authority(VehicleKey vehicle) const {
    const auto it = owner_.find(root_of(vehicle));
    return it == owner_.end() ? 0 : it->second;
}

std::vector<CoupledSetAuthority::VehicleKey> CoupledSetAuthority::release_peer(PeerId peer) {
    std::vector<VehicleKey> released;
    for (auto it = owner_.begin(); it != owner_.end();) {
        if (it->second == peer) {
            released.push_back(it->first);
            towed_by_.erase(it->first);
            it = owner_.erase(it);
        } else {
            ++it;
        }
    }
    // A trailer whose towing vehicle was released goes with it (the set has one authority).
    for (auto it = towed_by_.begin(); it != towed_by_.end();) {
        if (std::find(released.begin(), released.end(), it->second) != released.end()) {
            released.push_back(it->first);
            owner_.erase(it->first);
            it = towed_by_.erase(it);
        } else {
            ++it;
        }
    }
    return released;
}

} // namespace rg
