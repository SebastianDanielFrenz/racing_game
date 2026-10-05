// rg/camera_math.cpp - see camera_math.h.
#include "rg/camera_math.h"

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

const char* to_string(ShotSource s) { return s == ShotSource::RoadAhead ? "road_ahead" : "predicted_path"; }

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

    std::optional<CinematicShot> shot = place_roadside_shot(poly, std::min(lead, road_len > 0.0 ? road_len : lead), side,
                                                           height, params_, source);
    if (!shot) {
        // Cannot happen with a >= 2 point polyline; keep a sane shot anyway.
        CinematicShot s;
        s.x = car.x + car.heading_x * lead - car.heading_y * 6.0 * side;
        s.y = car.y + car.heading_y * lead + car.heading_x * 6.0 * side;
        s.height_above_ground_m = height;
        s.source = ShotSource::PredictedPath;
        s.side = side;
        shot = s;
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
    if (serial_ == 0 || age_s_ >= params_.max_hold_s || (age_s_ >= params_.min_hold_s && shot_expired(car))) {
        start_shot(car, road_ahead);
    }
    return shot_;
}

} // namespace rg
