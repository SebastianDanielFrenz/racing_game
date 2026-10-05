// rg/garage_set.cpp - see garage_set.h.
#include "rg/garage_set.h"

#include "rg/vehicle_setup.h" // is_hex_colour

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace rg {

namespace {

using detail::json;

constexpr double kPi = 3.14159265358979323846;
double rad(double deg) { return deg * kPi / 180.0; }

bool fail(std::string* err, const std::string& origin, const std::string& message) {
    if (err != nullptr) *err = origin + ": " + message;
    return false;
}

bool read_vec3(const json& o, const char* key, GVec3& out) {
    const auto it = o.find(key);
    if (it == o.end() || !it->is_array() || it->size() != 3) return false;
    double v[3];
    for (std::size_t i = 0; i < 3; ++i) {
        if (!(*it)[i].is_number()) return false;
        v[i] = (*it)[i].get<double>();
        if (!std::isfinite(v[i])) return false;
    }
    out = GVec3{v[0], v[1], v[2]};
    return true;
}

bool read_colour(const json& o, const char* key, std::string& out, const std::string& where, const std::string& origin,
                 std::string* err) {
    const auto it = o.find(key);
    if (it == o.end()) return true;
    if (!it->is_string() || !is_hex_colour(it->get<std::string>())) {
        return fail(err, origin, where + "." + key + " must be a #rrggbb colour");
    }
    out = it->get<std::string>();
    return true;
}

double lerp(double a, double b, double t) { return a + (b - a) * t; }
GVec3 lerp(const GVec3& a, const GVec3& b, double t) { return {lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.z, b.z, t)}; }

GVec3 rotate_z(const GVec3& v, double yaw_deg) {
    const double c = std::cos(rad(yaw_deg));
    const double s = std::sin(rad(yaw_deg));
    return {v.x * c - v.y * s, v.x * s + v.y * c, v.z};
}

double smoothstep(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double wrap180(double deg) {
    double d = std::fmod(deg + 180.0, 360.0);
    if (d < 0.0) d += 360.0;
    return d - 180.0;
}

} // namespace

const GarageAreaDef* GarageSetDesc::find_area(const std::string& id) const {
    for (const GarageAreaDef& a : areas) {
        if (a.id == id) return &a;
    }
    return nullptr;
}

std::optional<GarageSetDesc> parse_garage_set(const std::string& json_text, const std::string& origin, std::string* err) {
    const std::optional<json> parsed = detail::parse_json(json_text);
    if (!parsed || !parsed->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    std::string format;
    if (!detail::get_string(*parsed, "format", format) || format != "rg.garage_set/1") {
        fail(err, origin, "\"format\" must be \"rg.garage_set/1\"");
        return std::nullopt;
    }
    GarageSetDesc set;

    if (const auto room = parsed->find("room"); room != parsed->end() && room->is_object()) {
        detail::get_number(*room, "radius_m", set.room.radius_m);
        detail::get_number(*room, "height_m", set.room.height_m);
        if (!read_colour(*room, "floor_colour", set.room.floor_colour, "room", origin, err) ||
            !read_colour(*room, "wall_colour", set.room.wall_colour, "room", origin, err) ||
            !read_colour(*room, "accent_colour", set.room.accent_colour, "room", origin, err)) {
            return std::nullopt;
        }
        if (!(set.room.radius_m >= 6.0 && set.room.radius_m <= 60.0 && set.room.height_m >= 3.0 && set.room.height_m <= 30.0)) {
            fail(err, origin, "room.radius_m must be 6..60 and room.height_m 3..30");
            return std::nullopt;
        }
    }
    if (const auto tt = parsed->find("turntable"); tt != parsed->end() && tt->is_object()) {
        detail::get_number(*tt, "radius_m", set.turntable.radius_m);
        detail::get_number(*tt, "height_m", set.turntable.height_m);
        detail::get_number(*tt, "spin_deg_s", set.turntable.spin_deg_s);
        if (!read_colour(*tt, "colour", set.turntable.colour, "turntable", origin, err) ||
            !read_colour(*tt, "rim_colour", set.turntable.rim_colour, "turntable", origin, err)) {
            return std::nullopt;
        }
        if (!(set.turntable.radius_m >= 2.0 && set.turntable.radius_m <= set.room.radius_m * 0.6 &&
              set.turntable.height_m >= 0.0 && set.turntable.height_m <= 1.0 && std::abs(set.turntable.spin_deg_s) <= 90.0)) {
            fail(err, origin, "turntable: radius_m 2..0.6*room, height_m 0..1, spin_deg_s within +-90");
            return std::nullopt;
        }
    }
    if (const auto env = parsed->find("environment"); env != parsed->end() && env->is_object()) {
        detail::get_number(*env, "exposure", set.environment.exposure);
        detail::get_number(*env, "ambient_energy", set.environment.ambient_energy);
        detail::get_number(*env, "reflection_energy", set.environment.reflection_energy);
        detail::get_number(*env, "floor_reflectivity", set.environment.floor_reflectivity);
        if (!read_colour(*env, "ambient_colour", set.environment.ambient_colour, "environment", origin, err)) return std::nullopt;
        if (!(set.environment.exposure > 0.0 && set.environment.floor_reflectivity >= 0.0 && set.environment.floor_reflectivity <= 1.0)) {
            fail(err, origin, "environment: exposure > 0 and floor_reflectivity within 0..1");
            return std::nullopt;
        }
    }
    if (const auto lights = parsed->find("lights"); lights != parsed->end()) {
        if (!lights->is_array()) {
            fail(err, origin, "\"lights\" must be an array");
            return std::nullopt;
        }
        std::set<std::string> ids;
        for (const json& l : *lights) {
            GarageLight d;
            if (!l.is_object() || !detail::get_string(l, "id", d.id) || d.id.empty() || !ids.insert(d.id).second) {
                fail(err, origin, "every light needs a unique \"id\"");
                return std::nullopt;
            }
            detail::get_string(l, "kind", d.kind);
            if (d.kind != "spot" && d.kind != "omni") {
                fail(err, origin, "light " + d.id + ": kind must be spot or omni");
                return std::nullopt;
            }
            if (!read_vec3(l, "position", d.position)) {
                fail(err, origin, "light " + d.id + ": position must be three numbers");
                return std::nullopt;
            }
            if (l.contains("target") && !read_vec3(l, "target", d.target)) {
                fail(err, origin, "light " + d.id + ": target must be three numbers");
                return std::nullopt;
            }
            if (!read_colour(l, "colour", d.colour, "light " + d.id, origin, err)) return std::nullopt;
            detail::get_number(l, "energy", d.energy);
            detail::get_number(l, "range_m", d.range_m);
            detail::get_number(l, "angle_deg", d.angle_deg);
            detail::get_bool(l, "shadow", d.shadow);
            if (!(d.energy >= 0.0 && d.range_m > 0.0 && d.angle_deg > 0.0 && d.angle_deg <= 89.0)) {
                fail(err, origin, "light " + d.id + ": energy >= 0, range_m > 0, angle_deg within 0..89");
                return std::nullopt;
            }
            set.lights.push_back(std::move(d));
        }
    }
    if (const auto boxes = parsed->find("softboxes"); boxes != parsed->end()) {
        if (!boxes->is_array()) {
            fail(err, origin, "\"softboxes\" must be an array");
            return std::nullopt;
        }
        for (const json& b : *boxes) {
            GarageSoftbox d;
            if (!b.is_object() || !detail::get_string(b, "id", d.id) || d.id.empty() || !read_vec3(b, "position", d.position) ||
                !read_vec3(b, "target", d.target)) {
                fail(err, origin, "every softbox needs id, position and target");
                return std::nullopt;
            }
            detail::get_number(b, "width_m", d.width_m);
            detail::get_number(b, "height_m", d.height_m);
            detail::get_number(b, "energy", d.energy);
            if (!read_colour(b, "colour", d.colour, "softbox " + d.id, origin, err)) return std::nullopt;
            if (!(d.width_m > 0.0 && d.height_m > 0.0 && d.energy >= 0.0)) {
                fail(err, origin, "softbox " + d.id + ": width_m, height_m > 0 and energy >= 0");
                return std::nullopt;
            }
            set.softboxes.push_back(std::move(d));
        }
    }
    const auto cameras = parsed->find("cameras");
    if (cameras == parsed->end() || !cameras->is_object()) {
        fail(err, origin, "\"cameras\" block missing");
        return std::nullopt;
    }
    detail::get_number(*cameras, "transition_s", set.transition_s);
    if (!(set.transition_s >= 0.05 && set.transition_s <= 5.0)) {
        fail(err, origin, "cameras.transition_s must be 0.05..5");
        return std::nullopt;
    }
    const auto areas = cameras->find("areas");
    if (areas == cameras->end() || !areas->is_array() || areas->empty()) {
        fail(err, origin, "cameras.areas must be a non-empty array");
        return std::nullopt;
    }
    static const std::set<std::string> kAnchors = {"centre", "front_wheel", "rear_wheel", "engine_bay", "rear"};
    std::set<std::string> ids;
    for (const json& a : *areas) {
        GarageAreaDef d;
        if (!a.is_object() || !detail::get_string(a, "id", d.id) || d.id.empty() || !ids.insert(d.id).second) {
            fail(err, origin, "every camera area needs a unique \"id\"");
            return std::nullopt;
        }
        detail::get_string(a, "label", d.label);
        if (d.label.empty()) d.label = d.id;
        detail::get_string(a, "anchor", d.anchor);
        if (kAnchors.count(d.anchor) == 0) {
            fail(err, origin, "area " + d.id + ": unknown anchor \"" + d.anchor + "\"");
            return std::nullopt;
        }
        if (a.contains("anchor_offset") && !read_vec3(a, "anchor_offset", d.anchor_offset)) {
            fail(err, origin, "area " + d.id + ": anchor_offset must be three numbers");
            return std::nullopt;
        }
        detail::get_number(a, "azimuth_deg", d.azimuth_deg);
        detail::get_number(a, "elevation_deg", d.elevation_deg);
        detail::get_number(a, "distance_m", d.distance_m);
        detail::get_number(a, "distance_scale", d.distance_scale);
        detail::get_number(a, "fov_deg", d.fov_deg);
        detail::get_bool(a, "spin", d.spin);
        detail::get_number(a, "car_yaw_deg", d.car_yaw_deg);
        if (!(d.elevation_deg > -10.0 && d.elevation_deg < 80.0 && d.fov_deg >= 15.0 && d.fov_deg <= 100.0 && d.distance_m >= 0.0 &&
              d.distance_scale > 0.0)) {
            fail(err, origin, "area " + d.id + ": elevation -10..80, fov 15..100, distance_m >= 0, distance_scale > 0");
            return std::nullopt;
        }
        set.areas.push_back(std::move(d));
    }
    if (set.find_area("overview") == nullptr) {
        fail(err, origin, "cameras.areas must contain an \"overview\" area");
        return std::nullopt;
    }
    return set;
}

std::optional<GarageSetDesc> load_garage_set(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot open");
        return std::nullopt;
    }
    return parse_garage_set(*text, path, err);
}

GVec3 garage_anchor(const GarageAreaDef& area, const GarageSubject& s) {
    const double height = std::max(0.1, s.bounds_max.z - s.bounds_min.z);
    const double cy = 0.5 * (s.bounds_min.y + s.bounds_max.y);
    GVec3 p;
    if (area.anchor == "front_wheel") {
        p = {s.front_axle_x, cy + s.half_track, s.wheel_radius};
    } else if (area.anchor == "rear_wheel") {
        p = {s.rear_axle_x, cy + s.half_track, s.wheel_radius};
    } else if (area.anchor == "engine_bay") {
        if (s.engine_bay == "mid") {
            p = {s.rear_axle_x + 0.18 * (s.front_axle_x - s.rear_axle_x), cy, s.bounds_min.z + 0.70 * height};
        } else if (s.engine_bay == "rear") {
            p = {lerp(s.rear_axle_x, s.bounds_min.x, 0.35), cy, s.bounds_min.z + 0.65 * height};
        } else {
            p = {lerp(s.front_axle_x, s.bounds_max.x, 0.50), cy, s.bounds_min.z + 0.62 * height};
        }
    } else if (area.anchor == "rear") {
        p = {0.5 * (s.rear_axle_x + s.bounds_min.x), cy, s.bounds_min.z + 0.45 * height};
    } else {
        p = {0.5 * (s.bounds_min.x + s.bounds_max.x), cy, 0.5 * (s.bounds_min.z + s.bounds_max.z)};
    }
    return {p.x + area.anchor_offset.x, p.y + area.anchor_offset.y, p.z + area.anchor_offset.z};
}

GarageCameraPose garage_area_pose(const GarageAreaDef& area, const GarageSubject& s, double car_yaw_deg) {
    const GVec3 anchor = garage_anchor(area, s);
    const double length = std::max(0.5, s.bounds_max.x - s.bounds_min.x);
    const double dist = area.distance_m > 0.0 ? area.distance_m : area.distance_scale * length;
    // An engine bay behind the cabin (mid/rear engine) is looked at from behind: the
    // same view mirrored about the car's lateral axis.
    const bool from_behind = area.anchor == "engine_bay" && s.engine_bay != "front";
    const double az = rad(from_behind ? 180.0 - area.azimuth_deg : area.azimuth_deg);
    const double el = rad(area.elevation_deg);
    const GVec3 dir{std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
    const GVec3 cam_car{anchor.x + dist * dir.x, anchor.y + dist * dir.y, anchor.z + dist * dir.z};
    GarageCameraPose pose;
    pose.position = rotate_z(cam_car, car_yaw_deg);
    pose.look_at = rotate_z(anchor, car_yaw_deg);
    pose.fov_deg = area.fov_deg;
    return pose;
}

GarageCamera::GarageCamera(const GarageSetDesc& set, const GarageSubject& subject) : set_(set), subject_(subject) {
    area_ = "overview";
    pose_ = garage_area_pose(*set_.find_area(area_), subject_, 0.0);
    from_ = pose_;
}

GarageCameraPose GarageCamera::target_pose() const {
    const GarageAreaDef* a = set_.find_area(area_);
    return garage_area_pose(*a, subject_, a->spin ? 0.0 : a->car_yaw_deg);
}

bool GarageCamera::go_to(const std::string& area_id, bool instant) {
    const GarageAreaDef* a = set_.find_area(area_id);
    if (a == nullptr) return false;
    area_ = area_id;
    if (a->spin) {
        yaw_target_deg_ = yaw_deg_;
    } else {
        // the equivalent of the area's yaw nearest to where the car stands now
        yaw_target_deg_ = yaw_deg_ + wrap180(a->car_yaw_deg - yaw_deg_);
    }
    from_ = pose_;
    if (instant) {
        pose_ = target_pose();
        from_ = pose_;
        t_ = 1.0;
        if (!a->spin) yaw_deg_ = yaw_target_deg_;
    } else {
        t_ = 0.0;
    }
    return true;
}

void GarageCamera::set_subject(const GarageSubject& subject) {
    subject_ = subject;
    if (t_ >= 1.0) {
        pose_ = target_pose();
        from_ = pose_;
    }
}

void GarageCamera::update(double dt_s) {
    if (!(dt_s > 0.0) || !std::isfinite(dt_s)) return;
    dt_s = std::min(dt_s, 0.25);
    const GarageAreaDef* a = set_.find_area(area_);
    if (t_ < 1.0) {
        t_ = std::min(1.0, t_ + dt_s / set_.transition_s);
        const GarageCameraPose to = target_pose();
        const double k = smoothstep(t_);
        pose_.position = lerp(from_.position, to.position, k);
        pose_.look_at = lerp(from_.look_at, to.look_at, k);
        pose_.fov_deg = lerp(from_.fov_deg, to.fov_deg, k);
    } else {
        pose_ = target_pose();
    }
    if (a->spin) {
        yaw_deg_ = std::fmod(yaw_deg_ + set_.turntable.spin_deg_s * dt_s, 360.0);
        yaw_target_deg_ = yaw_deg_;
    } else {
        const double diff = yaw_target_deg_ - yaw_deg_;
        double step = diff * (1.0 - std::exp(-dt_s / 0.22));
        const double max_step = 150.0 * dt_s;
        step = std::clamp(step, -max_step, max_step);
        if (std::abs(diff) < 0.05) step = diff;
        yaw_deg_ += step;
    }
}

bool GarageCamera::settled() const {
    const GarageAreaDef* a = set_.find_area(area_);
    if (t_ < 1.0) return false;
    return a->spin || std::abs(yaw_target_deg_ - yaw_deg_) < 0.5;
}

} // namespace rg
