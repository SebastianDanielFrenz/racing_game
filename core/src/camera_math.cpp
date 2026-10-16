// rg/camera_math.cpp - see camera_math.h.
#include "rg/camera_math.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>

namespace rg {

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrap_pi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

double hypot2(double x, double y) { return std::sqrt(x * x + y * y); }

} // namespace

// ----------------------------------------------------------------- views ---

const char* to_string(DriveView v) {
    switch (v) {
        case DriveView::Chase: return "chase";
        case DriveView::Bumper: return "bumper";
        case DriveView::Cockpit: return "cockpit";
        case DriveView::Orbit: return "orbit";
        case DriveView::Cinematic: return "cinematic";
    }
    return "chase";
}

std::optional<DriveView> drive_view_from_string(const std::string& name) {
    for (int i = 0; i < kDriveViewCount; ++i) {
        const auto v = static_cast<DriveView>(i);
        if (name == to_string(v)) return v;
    }
    return std::nullopt;
}

DriveView next_drive_view(DriveView v) {
    return static_cast<DriveView>((static_cast<int>(v) + 1) % kDriveViewCount);
}

DriveView toggle_cockpit_view(DriveView v) { return v == DriveView::Cockpit ? DriveView::Chase : DriveView::Cockpit; }

// ---------------------------------------------------------------- bumper ---

VehicleBounds vehicle_bounds(const ps::vehicle::VehicleDesc& desc) {
    VehicleBounds b;
    if (desc.wheels.empty()) return b;
    double front = -1.0e30, rear = 1.0e30, track = 0.0, radius = 0.0, low = 1.0e30;
    for (const auto& w : desc.wheels) {
        front = std::max(front, static_cast<double>(w.attachment_local.x));
        rear = std::min(rear, static_cast<double>(w.attachment_local.x));
        track = std::max(track, std::fabs(static_cast<double>(w.attachment_local.y)));
        radius = std::max(radius, static_cast<double>(w.wheel_radius));
        low = std::min(low, static_cast<double>(w.attachment_local.z) - static_cast<double>(w.wheel_radius));
    }
    b.front_axle_x = front;
    b.rear_axle_x = rear;
    b.half_track_m = track;
    b.wheel_radius_m = radius;
    b.ground_z = low;
    return b;
}

ps::Vec3 bumper_eye_local(const VehicleBounds& bounds, const BumperParams& params) {
    return ps::Vec3{bounds.front_axle_x + params.forward_of_front_axle_m, 0.0,
                    bounds.ground_z + params.height_above_ground_m};
}

std::optional<ModelBounds> load_model_bounds(const std::string& rig_json_path, std::string* err) {
    auto fail = [&](const std::string& m) -> std::optional<ModelBounds> {
        if (err) *err = m;
        return std::nullopt;
    };
    const auto text = detail::read_text_file(rig_json_path);
    if (!text) return fail("cannot read " + rig_json_path);
    const auto root = detail::parse_json(*text);
    if (!root || !root->is_object()) return fail(rig_json_path + ": not a JSON object");
    auto vec = [](const detail::json& a, ps::Vec3& out) {
        if (!a.is_array() || a.size() != 3) return false;
        for (const auto& v : a)
            if (!v.is_number()) return false;
        out = ps::Vec3{a[0].get<double>(), a[1].get<double>(), a[2].get<double>()};
        return true;
    };
    ModelBounds b;
    const auto ob = root->find("overall_bounds");
    if (ob == root->end() || !ob->is_object() || !ob->contains("min") || !ob->contains("max") ||
        !vec((*ob)["min"], b.min) || !vec((*ob)["max"], b.max) || !(b.max.x > b.min.x) || !(b.max.z > b.min.z))
        return fail(rig_json_path + ": overall_bounds {min, max} missing or invalid");
    const auto sockets = root->find("sockets");
    if (sockets != root->end() && sockets->is_array()) {
        for (const auto& s : *sockets) {
            if (!s.is_object()) continue;
            std::string name;
            if (!detail::get_string(s, "name", name) || name != "socket_bumper_camera") continue;
            ps::Vec3 p;
            if (!s.contains("position") || !vec(s["position"], p))
                return fail(rig_json_path + ": socket_bumper_camera has no valid position");
            b.bumper_socket = p;
        }
    }
    return b;
}

ps::Vec3 bumper_eye_from_model(const ModelBounds& model, const ps::Vec3& model_offset, const BumperParams& params) {
    if (model.bumper_socket) return model_offset + *model.bumper_socket;
    const double height = std::min(params.height_above_ground_m, 0.5 * (model.max.z - model.min.z));
    const double centre_y = 0.5 * (model.min.y + model.max.y);
    return model_offset + ps::Vec3{model.max.x + params.nose_margin_m, centre_y, model.min.z + height};
}

// ----------------------------------------------------------------- orbit ---

OrbitState orbit_step(const OrbitState& s, const OrbitInput& in, double dt, const OrbitParams& p) {
    OrbitState n = s;
    const bool touched = in.live && (std::fabs(in.look_x) > 0.05 || std::fabs(in.look_y) > 0.05 ||
                                     std::fabs(in.zoom_steps) > 0.0 || std::fabs(in.zoom_key) > 0.05);
    if (in.live) {
        n.azimuth_rad += in.look_x * p.look_rate_rad_s * dt;
        n.elevation_rad += in.look_y * p.look_rate_rad_s * dt;
        n.distance_m *= std::pow(p.zoom_wheel_factor, -in.zoom_steps) * std::exp(-in.zoom_key * p.zoom_key_rate * dt);
    }
    if (touched) n.idle_s = 0.0;
    else n.idle_s += dt;
    if (n.idle_s >= p.idle_delay_s) n.azimuth_rad += p.idle_rate_rad_s * dt;
    n.azimuth_rad = wrap_pi(n.azimuth_rad);
    n.elevation_rad = std::clamp(n.elevation_rad, p.min_elevation_rad, p.max_elevation_rad);
    n.distance_m = std::clamp(n.distance_m, p.min_distance_m, p.max_distance_m);
    return n;
}

ps::Vec3 orbit_offset(const OrbitState& s) {
    const double r = s.distance_m * std::cos(s.elevation_rad);
    return ps::Vec3{-r * std::cos(s.azimuth_rad), -r * std::sin(s.azimuth_rad), s.distance_m * std::sin(s.elevation_rad)};
}

// -------------------------------------------------------------- cinematic ---

const char* to_string(ShotSource s) {
    switch (s) {
        case ShotSource::RoadAhead: return "road_ahead";
        case ShotSource::PredictedPath: return "predicted_path";
        case ShotSource::ChaseFallback: return "chase_fallback";
    }
    return "road_ahead";
}

std::optional<CinematicShot> place_roadside_shot(const std::vector<RoadPoint>& poly, double lead_m, int side,
                                                 double height_m, const CinematicParams& p, ShotSource source) {
    if (poly.size() < 2) return std::nullopt;
    double remaining = std::max(0.0, lead_m);
    for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
        const double dx = poly[i + 1].x - poly[i].x;
        const double dy = poly[i + 1].y - poly[i].y;
        const double l = hypot2(dx, dy);
        const bool last = i + 2 == poly.size();
        if (l < 1.0e-6 && !last) continue;
        if (remaining <= l || last) {
            const double t = l < 1.0e-6 ? 0.0 : std::min(1.0, remaining / l);
            const double ux = l < 1.0e-6 ? 1.0 : dx / l;
            const double uy = l < 1.0e-6 ? 0.0 : dy / l;
            const double hw = poly[i].half_width_m + (poly[i + 1].half_width_m - poly[i].half_width_m) * t;
            const double off = hw + p.side_clearance_m;
            const int sd = side >= 0 ? 1 : -1;
            CinematicShot shot;
            shot.x = poly[i].x + dx * t + (-uy) * off * sd;
            shot.y = poly[i].y + dy * t + ux * off * sd;
            shot.height_above_ground_m = height_m;
            shot.source = source;
            shot.side = sd;
            shot.fov_deg = p.base_fov_deg;
            return shot;
        }
        remaining -= l;
    }
    return std::nullopt;
}

std::vector<RoadPoint> predict_path_ahead(const std::vector<std::pair<double, double>>& history, const CarKinematics& car,
                                          const CinematicParams& p) {
    const double speed = hypot2(car.vx, car.vy);
    double omega = 0.0; // rad/s, + = turning left (counter-clockwise)
    if (history.size() >= 3 && speed >= p.min_speed_mps) {
        const auto& a = history.front();
        const auto& m = history[history.size() / 2];
        const auto& b = history.back();
        const double d1x = m.first - a.first, d1y = m.second - a.second;
        const double d2x = b.first - m.first, d2y = b.second - m.second;
        if (hypot2(d1x, d1y) > 2.0 && hypot2(d2x, d2y) > 2.0) {
            const double turn = wrap_pi(std::atan2(d2y, d2x) - std::atan2(d1y, d1x));
            const double dt_mid = (static_cast<double>(history.size() - 1) * p.history_step_s) * 0.5;
            if (dt_mid > 1.0e-6) omega = std::clamp(turn / dt_mid, -p.max_turn_rate_rad_s, p.max_turn_rate_rad_s);
        }
    }
    std::vector<RoadPoint> out;
    double x = car.x, y = car.y;
    double heading = std::atan2(car.heading_y, car.heading_x);
    const double v = std::max(speed, p.min_speed_mps);
    out.push_back({x, y, 3.0});
    double travelled = 0.0;
    while (travelled < p.predict_length_m) {
        heading += omega * (p.predict_step_m / v);
        x += std::cos(heading) * p.predict_step_m;
        y += std::sin(heading) * p.predict_step_m;
        travelled += p.predict_step_m;
        out.push_back({x, y, 3.0});
    }
    return out;
}

CinematicDirector::CinematicDirector(std::uint64_t seed, CinematicParams params)
    : params_(params), seed_(seed), rng_(seed) {}

void CinematicDirector::reset() {
    rng_ = seed_;
    shot_ = CinematicShot{};
    serial_ = 0;
    age_s_ = 0.0;
    cut_ = false;
    next_side_ = 1;
    history_clock_s_ = 0.0;
    history_.clear();
    stats_ = CinematicStats{};
}

std::uint64_t CinematicDirector::next_random() {
    // SplitMix64: tiny, well mixed, and the same sequence on every platform.
    rng_ += 0x9E3779B97F4A7C15ull;
    std::uint64_t z = rng_;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

double CinematicDirector::random_unit() {
    return static_cast<double>(next_random() >> 11) * (1.0 / 9007199254740992.0);
}

bool CinematicDirector::shot_expired(const CarKinematics& car) const {
    const double rx = car.x - shot_.x;
    const double ry = car.y - shot_.y;
    if (hypot2(rx, ry) > params_.cut_when_far_m) return true;
    // The car has gone past the camera: its position relative to the camera,
    // along its own heading, is positive.
    return rx * car.heading_x + ry * car.heading_y > params_.cut_when_passed_m;
}

namespace {

// The point `s` metres along a polyline (clamped to its ends), horizontal position only.
std::pair<double, double> point_along(const std::vector<RoadPoint>& poly, double s) {
    if (poly.empty()) return {0.0, 0.0};
    double remaining = std::max(0.0, s);
    for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
        const double dx = poly[i + 1].x - poly[i].x;
        const double dy = poly[i + 1].y - poly[i].y;
        const double l = hypot2(dx, dy);
        if (remaining <= l && l > 1.0e-9) {
            const double t = remaining / l;
            return {poly[i].x + dx * t, poly[i].y + dy * t};
        }
        remaining -= l;
    }
    return {poly.back().x, poly.back().y};
}

double polyline_length(const std::vector<RoadPoint>& poly) {
    double len = 0.0;
    for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
        len += hypot2(poly[i + 1].x - poly[i].x, poly[i + 1].y - poly[i].y);
    }
    return len;
}

} // namespace

bool CinematicDirector::sight_clear(double ax, double ay, double az, double bx, double by, double bz) const {
    if (!obstacles_) return true;
    if (obstacles_->building_blocks(ax, ay, az, bx, by, bz)) return false;
    const double dx = bx - ax, dy = by - ay, dz = bz - az;
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    const int n = std::clamp(static_cast<int>(std::ceil(len / std::max(0.5, params_.sight_sample_step_m))), 1,
                             std::max(1, params_.sight_max_samples));
    for (int i = 1; i < n; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(n);
        const double x = ax + dx * t, y = ay + dy * t, z = az + dz * t;
        const auto ground = obstacles_->ground_height(x, y);
        if (ground && z < *ground + params_.sight_ground_clearance_m) return false;
    }
    return true;
}

bool CinematicDirector::candidate_clear(const CinematicShot& shot, const CarKinematics& car,
                                        const std::vector<RoadPoint>& poly, double lead_m) {
    if (!obstacles_) return true;
    ++stats_.candidates;
    if (!obstacles_->ready(shot.x, shot.y) || !obstacles_->ready(car.x, car.y)) {
        ++stats_.rejected_not_ready;
        return false;
    }
    const double cam_ground = obstacles_->ground_height(shot.x, shot.y).value_or(0.0);
    const double cam_z = cam_ground + shot.height_above_ground_m;
    if (obstacles_->building_covers(shot.x, shot.y, cam_z, params_.building_margin_m)) {
        ++stats_.rejected_in_building;
        return false;
    }
    // The car now, then the road a third and two thirds of the way to the camera (the stretch the shot films).
    const double car_ground = obstacles_->ground_height(car.x, car.y).value_or(0.0);
    if (!sight_clear(shot.x, shot.y, cam_z, car.x, car.y, car_ground + params_.car_target_height_m)) {
        ++stats_.rejected_sight;
        return false;
    }
    for (const double fraction : {1.0 / 3.0, 2.0 / 3.0}) {
        const auto p = point_along(poly, lead_m * fraction);
        const double g = obstacles_->ground_height(p.first, p.second).value_or(0.0);
        if (!sight_clear(shot.x, shot.y, cam_z, p.first, p.second, g + params_.car_target_height_m)) {
            ++stats_.rejected_sight;
            return false;
        }
    }
    return true;
}

void CinematicDirector::place_chase_fallback(const CarKinematics& car) {
    // Behind the car on its own path (the road under it is clear by construction); shorten the distance if even that
    // lands inside a building (a narrow street corner).
    double dist = params_.fallback_distance_m;
    double x = car.x - car.heading_x * dist, y = car.y - car.heading_y * dist;
    if (obstacles_) {
        const double ground_car = obstacles_->ground_height(car.x, car.y).value_or(0.0);
        for (const double factor : {1.0, 0.6, 0.35}) {
            dist = params_.fallback_distance_m * factor;
            x = car.x - car.heading_x * dist;
            y = car.y - car.heading_y * dist;
            const double z = obstacles_->ground_height(x, y).value_or(ground_car) + params_.fallback_height_m;
            if (!obstacles_->building_covers(x, y, z, params_.building_margin_m)) break;
        }
    }
    shot_.x = x;
    shot_.y = y;
    shot_.height_above_ground_m = params_.fallback_height_m;
    shot_.source = ShotSource::ChaseFallback;
    shot_.side = 1;
    shot_.fov_deg = params_.base_fov_deg;
}

void CinematicDirector::start_shot(const CarKinematics& car, const std::vector<RoadPoint>& road_ahead) {
    const double speed = hypot2(car.vx, car.vy);
    const double lead = std::clamp(speed * params_.lead_time_s, params_.min_lead_m, params_.max_lead_m);

    // Prefer real road data with a useful stretch of road.
    std::vector<RoadPoint> poly;
    ShotSource source = ShotSource::RoadAhead;
    double road_len = 0.0;
    for (std::size_t i = 0; i + 1 < road_ahead.size(); ++i) {
        road_len += hypot2(road_ahead[i + 1].x - road_ahead[i].x, road_ahead[i + 1].y - road_ahead[i].y);
    }
    if (road_ahead.size() >= 2 && road_len >= 30.0) {
        poly = road_ahead;
    } else {
        poly = predict_path_ahead(history_, car, params_);
        source = ShotSource::PredictedPath;
    }

    // Alternate sides, with a seeded chance of staying on the same side.
    int side = next_side_;
    if (random_unit() < 0.25) side = -side;
    next_side_ = -side;
    const double height = params_.min_height_m + (params_.max_height_m - params_.min_height_m) * random_unit();

    const double base_lead = std::min(lead, road_len > 0.0 ? road_len : lead);
    std::optional<CinematicShot> shot;
    double shot_lead = base_lead;
    if (!obstacles_) {
        shot = place_roadside_shot(poly, base_lead, side, height, params_, source);
    } else {
        // Preferred lead on the preferred side, then the other side, then the other leads (shorter and longer) on both.
        const double poly_len = polyline_length(poly);
        double leads[5];
        int lead_count = 0;
        leads[lead_count++] = base_lead;
        for (const double factor : params_.lead_factors) {
            const double l = std::clamp(base_lead * factor, std::min(params_.min_candidate_lead_m, poly_len), poly_len);
            bool duplicate = false;
            for (int i = 0; i < lead_count; ++i) duplicate = duplicate || std::fabs(leads[i] - l) < 2.0;
            if (!duplicate && lead_count < 5) leads[lead_count++] = l;
        }
        for (int i = 0; i < lead_count && !shot; ++i) {
            for (const int s : {side, -side}) {
                auto candidate = place_roadside_shot(poly, leads[i], s, height, params_, source);
                if (candidate && candidate_clear(*candidate, car, poly, leads[i])) {
                    shot = candidate;
                    shot_lead = leads[i];
                    break;
                }
            }
        }
        (void)shot_lead;
    }
    if (!shot && !obstacles_) {
        // Cannot happen with a >= 2 point polyline; keep a sane shot anyway.
        CinematicShot s;
        s.x = car.x + car.heading_x * lead - car.heading_y * 6.0 * side;
        s.y = car.y + car.heading_y * lead + car.heading_x * 6.0 * side;
        s.height_above_ground_m = height;
        s.source = ShotSource::PredictedPath;
        s.side = side;
        shot = s;
    }
    if (!shot) {
        // Every candidate was blocked (or the data is still loading): film from behind the car and look again soon.
        const bool already = serial_ > 0 && shot_.source == ShotSource::ChaseFallback;
        place_chase_fallback(car);
        age_s_ = 0.0;
        if (!already) {
            ++serial_;
            shot_.serial = serial_;
            cut_ = true;
        }
        ++stats_.fallbacks;
        return;
    }
    const double dist = std::max(1.0, hypot2(shot->x - car.x, shot->y - car.y));
    shot->fov_deg = std::clamp(2.0 * std::atan(15.0 / dist) * 180.0 / kPi, 14.0, params_.base_fov_deg);
    ++serial_;
    shot->serial = serial_;
    shot_ = *shot;
    age_s_ = 0.0;
    cut_ = true;
}

const CinematicShot& CinematicDirector::update(double dt, const CarKinematics& car,
                                               const std::vector<RoadPoint>& road_ahead) {
    cut_ = false;
    history_clock_s_ += dt;
    if (history_.empty() || history_clock_s_ >= params_.history_step_s) {
        history_clock_s_ = 0.0;
        history_.emplace_back(car.x, car.y);
        const auto max_points = static_cast<std::size_t>(params_.history_span_s / params_.history_step_s) + 1;
        while (history_.size() > max_points) history_.erase(history_.begin());
    }
    age_s_ += dt;
    if (serial_ == 0) {
        start_shot(car, road_ahead);
    } else if (shot_.source == ShotSource::ChaseFallback) {
        // The fallback rides with the car (allocation free); every fallback_retry_s look for a clear roadside shot again.
        if (age_s_ >= params_.fallback_retry_s) start_shot(car, road_ahead);
        else place_chase_fallback(car);
    } else if (age_s_ >= params_.max_hold_s || (age_s_ >= params_.min_hold_s && shot_expired(car))) {
        start_shot(car, road_ahead);
    }
    return shot_;
}

} // namespace rg

namespace rg {

ps::Vec3 chase_follow_offset(const ps::Vec3& previous, const ps::Vec3& desired, double dt, double rate_per_s) {
    if (!(rate_per_s > 0.0) || !(dt > 0.0)) return rate_per_s > 0.0 ? previous : desired;
    const double a = 1.0 - std::exp(-rate_per_s * dt);
    return ps::Vec3{previous.x + (desired.x - previous.x) * a, previous.y + (desired.y - previous.y) * a,
                    previous.z + (desired.z - previous.z) * a};
}

} // namespace rg

namespace rg {
CockpitLook cockpit_look_step(CockpitLook state, const CockpitLookInput& in, double dt, const CockpitLookConfig& c) {
    if (!in.live || !std::isfinite(dt) || dt<0 || !std::isfinite(c.pitch_min) ||
        !std::isfinite(c.pitch_max) || c.pitch_min>c.pitch_max || !std::isfinite(c.deadzone) ||
        c.deadzone<0 || c.deadzone>=1 || !std::isfinite(c.rate_deg_s) || c.rate_deg_s<0 ||
        !std::isfinite(c.mouse_deg_per_pixel) || c.mouse_deg_per_pixel<0) return state;
    const auto clean=[](double v){return std::isfinite(v)?v:0;};
    const auto axis=[&](double v){v=std::clamp(clean(v),-1.0,1.0);return std::abs(v)<=c.deadzone?0:std::copysign((std::abs(v)-c.deadzone)/(1-c.deadzone),v);};
    state.yaw_deg=std::remainder(clean(state.yaw_deg)+clean(in.mouse_x)*c.mouse_deg_per_pixel+axis(in.look_x)*c.rate_deg_s*dt,360.0);
    state.pitch_deg=std::clamp(clean(state.pitch_deg)-clean(in.mouse_y)*c.mouse_deg_per_pixel+axis(in.look_y)*c.rate_deg_s*dt,c.pitch_min,c.pitch_max);
    return state;
}
double steering_wheel_angle_rad(double steer,double lock_deg) {
    return std::isfinite(steer)&&std::isfinite(lock_deg)&&lock_deg>=0?std::clamp(steer,-1.0,1.0)*lock_deg*kPi/180:0;
}
}

// ------------------------------------------------------------------ drone ----

namespace rg {

namespace {
constexpr double kTau = 6.283185307179586476925286766559;
constexpr double kHalfPi = 1.5707963267948966192313216916398;
// Godot's lerp_angle (core/math/math_funcs.h), the former GDScript call.
double lerp_angle_godot(double from, double to, double w) {
    const double difference = std::fmod(to - from, kTau);
    const double distance = std::fmod(2.0 * difference, kTau) - difference;
    return from + distance * w;
}
double len3(const ps::Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
} // namespace

DroneState drone_initial_state(const DroneParams& p) {
    DroneState s;
    s.distance_m = std::clamp(p.follow_distance_m, p.min_distance_m, p.max_distance_m);
    return s;
}

void drone_activate(DroneState& s, const ps::Vec3& from_position) {
    s.smoothed_pos = from_position;
    s.from_absolute = true;
    s.initialized = true;
    s.orbit_yaw_rad = 0.0;
    s.drain_wheel = true;
}

void drone_shift_origin(DroneState& s, const ps::Vec3& d) {
    s.smoothed_pos = ps::Vec3{s.smoothed_pos.x + d.x, s.smoothed_pos.y + d.y, s.smoothed_pos.z + d.z};
}

void drone_hold(DroneState& s) { s.initialized = false; }

double drone_target_orbit_yaw(double key_x, double key_y, double stick_x, double stick_y, double deadzone) {
    if (std::fabs(key_x) > 0.5) return (key_x > 0.0 ? 1.0 : (key_x < 0.0 ? -1.0 : 0.0)) * kHalfPi;
    if (key_y < -0.5) return kPi;
    const double mag = std::min(std::sqrt(stick_x * stick_x + stick_y * stick_y), 1.0);
    if (mag <= deadzone) return 0.0;
    return std::atan2(stick_x, stick_y);
}

void drone_zoom_step(DroneState& s, double wheel, double key, bool live, double dt, const DroneParams& p) {
    if (s.drain_wheel) {
        s.drain_wheel = false;
        return;
    }
    if (!live) return;
    s.distance_m *= std::pow(p.zoom_wheel_factor, -wheel) * std::exp(-key * p.zoom_key_rate * dt);
    s.distance_m = std::clamp(s.distance_m, p.min_distance_m, p.max_distance_m);
}

DroneFrame drone_follow_step(DroneState& s, const DroneFollowInput& in, double dt, const DroneParams& p) {
    // The target's forward axis kept horizontal, so a pitching car does not swing the drone up and down.
    const double fx = in.target_forward.x, fy = in.target_forward.y;
    const double fl = std::sqrt(fx * fx + fy * fy);
    if (fl > 0.2) {
        s.heading_x = fx / fl;
        s.heading_y = fy / fl;
    }
    const ps::Vec3 desired{-s.heading_x * s.distance_m, -s.heading_y * s.distance_m, s.distance_m * p.height_per_distance};
    const ps::Vec3 look_at{in.target_pos.x, in.target_pos.y, in.target_pos.z + p.look_height_m};

    if (s.from_absolute) {
        s.offset = ps::Vec3{s.smoothed_pos.x - in.target_pos.x, s.smoothed_pos.y - in.target_pos.y,
                            s.smoothed_pos.z - in.target_pos.z};
        s.from_absolute = false;
    }
    const ps::Vec3 jump{s.offset.x - desired.x, s.offset.y - desired.y, s.offset.z - desired.z};
    if (!s.initialized || len3(jump) > p.snap_m) {
        s.offset = desired;
        s.initialized = true;
    }
    const double a = 1.0 - std::exp(-p.position_lag * dt);
    s.offset = ps::Vec3{s.offset.x + (desired.x - s.offset.x) * a, s.offset.y + (desired.y - s.offset.y) * a,
                        s.offset.z + (desired.z - s.offset.z) * a};
    s.smoothed_pos = ps::Vec3{in.target_pos.x + s.offset.x, in.target_pos.y + s.offset.y, in.target_pos.z + s.offset.z};

    double target_yaw = 0.0;
    if (in.live) target_yaw = drone_target_orbit_yaw(in.key_x, in.key_y, in.stick_x, in.stick_y, p.orbit_deadzone);
    if (p.orbit_lag <= 0.0) s.orbit_yaw_rad = target_yaw;
    else s.orbit_yaw_rad = lerp_angle_godot(s.orbit_yaw_rad, target_yaw, 1.0 - std::exp(-p.orbit_lag * dt));

    // Rotation about +z by -yaw (Godot: offset.rotated(UP, -yaw)).
    const double c = std::cos(-s.orbit_yaw_rad), sn = std::sin(-s.orbit_yaw_rad);
    DroneFrame f;
    f.position = ps::Vec3{in.target_pos.x + s.offset.x * c - s.offset.y * sn,
                          in.target_pos.y + s.offset.x * sn + s.offset.y * c, in.target_pos.z + s.offset.z};
    f.look_at = look_at;
    const ps::Vec3 d{f.position.x - look_at.x, f.position.y - look_at.y, f.position.z - look_at.z};
    f.has_look = len3(d) > 0.01;
    return f;
}

std::vector<ps::Vec3> drone_probe_offsets() { return ground_probe_offsets(); }

double drone_clear_terrain(double z, const std::vector<std::optional<double>>& grounds, double clearance_m) {
    return clear_terrain_height(z, grounds, clearance_m);
}

// ------------------------------------------------- shared rig helpers ----

std::vector<ps::Vec3> ground_probe_offsets() {
    return {ps::Vec3{0, 0, 0}, ps::Vec3{0.3, 0, 0}, ps::Vec3{-0.3, 0, 0}, ps::Vec3{0, -0.3, 0}, ps::Vec3{0, 0.3, 0}};
}

double clear_terrain_height(double z, const std::vector<std::optional<double>>& grounds, double clearance_m) {
    for (const auto& g : grounds)
        if (g) z = std::max(z, *g + clearance_m);
    return z;
}

bool look_valid(const ps::Vec3& position, const ps::Vec3& target) {
    return len3(ps::Vec3{position.x - target.x, position.y - target.y, position.z - target.z}) > 0.01;
}

double rig_orbit_yaw(double key_x, double key_y, double stick_x, double stick_y, double deadzone) {
    return drone_target_orbit_yaw(key_x, key_y, stick_x, stick_y, deadzone);
}

namespace {
ps::Vec3 add3(const ps::Vec3& a, const ps::Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
ps::Vec3 sub3(const ps::Vec3& a, const ps::Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
ps::Vec3 mul3(const ps::Vec3& a, double k) { return {a.x * k, a.y * k, a.z * k}; }
// Godot's Vector3.normalized(): a zero vector stays zero.
ps::Vec3 unit3(const ps::Vec3& a) {
    const double l = len3(a);
    return l > 0.0 ? mul3(a, 1.0 / l) : a;
}
// Rotation about +z (Godot: Vector3.rotated(UP, angle)).
ps::Vec3 rotz(const ps::Vec3& v, double angle) {
    const double c = std::cos(angle), sn = std::sin(angle);
    return {v.x * c - v.y * sn, v.x * sn + v.y * c, v.z};
}
} // namespace

// ------------------------------------------------------------------ chase ----

void chase_reset(ChaseState& s) { s.initialized = false; }

ChaseFrame chase_follow_step(ChaseState& s, const ChaseInput& in, double dt, const ChaseParams& p) {
    const ps::Vec3 behind = mul3(unit3(in.forward), -p.follow_distance_m);
    const ps::Vec3 side = mul3(unit3(in.left), p.side_offset_m);
    const ps::Vec3 desired = add3(add3(behind, side), ps::Vec3{0, 0, p.follow_height_m});
    const ps::Vec3 look_at = add3(in.target_pos, ps::Vec3{0, 0, p.look_height_m});

    if (!s.initialized || len3(sub3(s.offset, desired)) > p.snap_m) {
        s.offset = desired;
        s.initialized = true;
    }
    s.offset = chase_follow_offset(s.offset, desired, dt, p.position_lag);

    double target_yaw = 0.0;
    if (in.live) target_yaw = rig_orbit_yaw(in.key_x, in.key_y, in.stick_x, in.stick_y, p.orbit_deadzone);
    if (p.orbit_lag <= 0.0) s.orbit_yaw_rad = target_yaw;
    else s.orbit_yaw_rad = lerp_angle_godot(s.orbit_yaw_rad, target_yaw, 1.0 - std::exp(-p.orbit_lag * dt));

    ChaseFrame f;
    f.position = add3(in.target_pos, rotz(s.offset, -s.orbit_yaw_rad));
    f.look_at = look_at;
    return f;
}

// ------------------------------------------------------------------- free ----

namespace {
void free_apply_limits(FreeState& s, const FreeParams& p) {
    s.pitch_rad = std::clamp(s.pitch_rad, -p.pitch_limit_rad, p.pitch_limit_rad);
}
} // namespace

void free_activate(FreeState& s, const ps::Vec3& position, const ps::Vec3& forward, const FreeParams& p) {
    const ps::Vec3 f = unit3(forward);
    s.position = position;
    s.yaw_rad = std::atan2(-f.x, f.y);
    s.pitch_rad = std::asin(std::clamp(f.z, -1.0, 1.0));
    free_apply_limits(s, p);
}

void free_place(FreeState& s, const ps::Vec3& position, double yaw_rad, double pitch_rad, const FreeParams& p) {
    s.position = position;
    s.yaw_rad = yaw_rad;
    s.pitch_rad = pitch_rad;
    free_apply_limits(s, p);
}

void free_shift_origin(FreeState& s, const ps::Vec3& delta) { s.position = add3(s.position, delta); }

void free_step(FreeState& s, const FreeInput& in, double dt, const FreeParams& p) {
    if (!in.live) return;
    s.yaw_rad -= in.mouse_x * p.mouse_sensitivity + in.look_x * p.look_rate_rad_s * dt;
    s.pitch_rad -= in.mouse_y * p.mouse_sensitivity;
    s.pitch_rad += in.look_y * p.look_rate_rad_s * dt;
    free_apply_limits(s, p);

    // Horizontal move relative to the heading (straight runs never dive), plus the vertical axis.
    ps::Vec3 move = add3(rotz(ps::Vec3{in.move_right, in.move_forward, 0.0}, s.yaw_rad), ps::Vec3{0, 0, in.move_up});
    if (len3(move) > 1.0) move = unit3(move);
    const double speed = p.move_speed_mps * (in.fast ? p.fast_multiplier : 1.0);
    s.position = add3(s.position, mul3(move, speed * dt));
}

// ----------------------------------------------------------------- walker ----

WalkerCamState walker_cam_initial_state(const WalkerCamParams& p) {
    WalkerCamState s;
    s.pitch_rad = p.start_pitch_rad;
    s.distance_m = std::clamp(p.third_person_distance_m, p.min_distance_m, p.max_distance_m);
    return s;
}

void walker_cam_activate(WalkerCamState& s, const ps::Vec3& from_forward, const WalkerCamParams& p) {
    s.have_walker = false;
    s.drain_wheel = true;
    const ps::Vec3 f = unit3(from_forward);
    s.yaw_rad = std::atan2(-f.x, f.y);
    s.pitch_rad = p.start_pitch_rad;
}

void walker_cam_shift_origin(WalkerCamState& s, const ps::Vec3& delta) { s.smoothed_feet = add3(s.smoothed_feet, delta); }

void walker_cam_hold(WalkerCamState& s) { s.have_walker = false; }

ps::Vec3 walker_cam_look_forward(const WalkerCamState& s) { return {-std::sin(s.yaw_rad), std::cos(s.yaw_rad), 0.0}; }

WalkerCamFrame walker_cam_step(WalkerCamState& s, const WalkerCamInput& in, double dt, const WalkerCamParams& p) {
    WalkerCamFrame f;
    if (in.has_walker) {
        if (!s.have_walker) {
            s.have_walker = true;
            s.smoothed_feet = in.feet;
            // First sight of the walker: look at the car it just got out of.
            ps::Vec3 to_car = sub3(in.car_position, in.feet);
            to_car.z = 0.0;
            if (len3(to_car) > 0.1) s.yaw_rad = std::atan2(-to_car.x, to_car.y);
            s.pitch_rad = p.start_pitch_rad;
        } else if (len3(sub3(s.smoothed_feet, in.feet)) > p.snap_m) {
            s.smoothed_feet = in.feet;
        } else {
            const double a = 1.0 - std::exp(-p.follow_rate * dt);
            s.smoothed_feet = add3(s.smoothed_feet, mul3(sub3(in.feet, s.smoothed_feet), a));
        }
    }

    // The wheel is drained once after an activation even while the inputs are off.
    double wheel = in.wheel_steps;
    if (s.drain_wheel) {
        s.drain_wheel = false;
        wheel = 0.0;
    }
    if (in.live) {
        s.yaw_rad -= in.mouse_x * p.mouse_sensitivity + in.look_x * p.look_rate_rad_s * dt;
        s.pitch_rad -= in.mouse_y * p.mouse_sensitivity;
        s.pitch_rad += in.look_y * p.look_rate_rad_s * dt;
        if (in.first_person) {
            s.pitch_rad = std::clamp(s.pitch_rad, -p.first_pitch_limit, p.first_pitch_limit);
        } else {
            s.pitch_rad = std::clamp(s.pitch_rad, p.third_pitch_min, p.third_pitch_max);
            s.distance_m *= std::pow(p.zoom_wheel_factor, -wheel) * std::exp(-in.zoom_key * p.zoom_key_rate * dt);
            s.distance_m = std::clamp(s.distance_m, p.min_distance_m, p.max_distance_m);
        }
    }

    f.yaw_rad = s.yaw_rad;
    f.pitch_rad = s.pitch_rad;
    if (!in.has_walker) return f; // not spawned yet / already despawned: hold still, the look stays responsive
    f.placed = true;
    if (in.first_person) {
        f.position = add3(s.smoothed_feet, ps::Vec3{0, 0, p.eye_height_m});
        return f;
    }
    f.third_person = true;
    f.pivot = add3(s.smoothed_feet, ps::Vec3{0, 0, p.head_height_m});
    // The look direction: pitch about the right axis, then yaw about up (Godot: yaw * (pitch * forward)).
    const double cp = std::cos(s.pitch_rad), sp = std::sin(s.pitch_rad);
    const ps::Vec3 look_dir = rotz(ps::Vec3{0.0, cp, sp}, s.yaw_rad);
    f.position = sub3(f.pivot, mul3(look_dir, s.distance_m));
    return f;
}

WalkerCamAim walker_cam_aim(const ps::Vec3& pivot, const ps::Vec3& position, const WalkerCamParams& p) {
    WalkerCamAim a;
    const ps::Vec3 d = sub3(pivot, position);
    const double flat = std::sqrt(d.x * d.x + d.y * d.y);
    if (flat > 0.01) {
        a.valid = true;
        a.yaw_rad = std::atan2(-d.x, d.y);
        a.pitch_rad = std::clamp(std::atan2(d.z, flat), p.third_pitch_min, p.third_pitch_max);
    }
    return a;
}

// ------------------------------------------------------------- orbit rig ----

OrbitRigState orbit_rig_initial_state() {
    OrbitRigState s;
    const OrbitParams op;
    s.orbit.elevation_rad = op.default_elevation_rad;
    s.orbit.distance_m = op.default_distance_m;
    return s;
}

void orbit_rig_activate(OrbitRigState& s) {
    s = orbit_rig_initial_state();
}

void orbit_rig_reset(OrbitRigState& s) { s.initialized = false; }

OrbitRigFrame orbit_rig_step(OrbitRigState& s, const OrbitRigInput& in, double dt, const OrbitRigParams& p) {
    const ps::Vec3 focus_rel{0.0, 0.0, p.focus_height_m};
    if (!s.initialized || len3(sub3(s.target_rel, focus_rel)) > p.snap_m) {
        s.target_rel = focus_rel;
        s.initialized = true;
    }
    const double a = 1.0 - std::exp(-p.follow_lag * dt);
    s.target_rel = add3(s.target_rel, mul3(sub3(focus_rel, s.target_rel), a));
    OrbitRigFrame f;
    f.target = add3(in.chassis_pos, s.target_rel);

    OrbitInput oi;
    oi.live = in.live;
    if (in.live) {
        // Right/up moves the camera right/up; the captured mouse drags it the same way.
        oi.look_x = std::clamp(in.look_x + in.mouse_x * p.mouse_look_per_px, -1.0, 1.0);
        oi.look_y = std::clamp(in.look_y + in.mouse_y * p.mouse_look_per_px, -1.0, 1.0);
        oi.zoom_steps = in.zoom_steps;
        oi.zoom_key = in.zoom_key;
    }
    s.orbit = orbit_step(s.orbit, oi, dt);

    // The offset is in the heading frame (x forward, y left, z up).
    const ps::Vec3 o = orbit_offset(s.orbit);
    ps::Vec3 fwd{in.forward.x, in.forward.y, 0.0};
    fwd = len3(fwd) > 0.001 ? unit3(fwd) : ps::Vec3{0.0, 1.0, 0.0}; // Godot FORWARD (0, 0, -1)
    const ps::Vec3 left{-fwd.y, fwd.x, 0.0};
    f.position = add3(add3(add3(f.target, mul3(fwd, o.x)), mul3(left, o.y)), ps::Vec3{0, 0, o.z});
    return f;
}

} // namespace rg
