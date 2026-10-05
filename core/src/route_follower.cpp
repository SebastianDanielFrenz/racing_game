// rg/route_follower.cpp — see route_follower.h for the contract.
#include "rg/route_follower.h"

#include "ps/math/transcendental.h"
#include "ps/world/world.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rg {

namespace {

double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

} // namespace

RouteFollower::RouteFollower(const std::vector<RoutePoint>& polyline, const RouteFollowerParams& params)
    : params_(params) {
    if (polyline.size() < 2) return;
    // Cumulative arc length of the input polyline.
    std::vector<double> cum(polyline.size(), 0.0);
    for (std::size_t i = 1; i < polyline.size(); ++i) {
        const double dx = polyline[i].x - polyline[i - 1].x;
        const double dy = polyline[i].y - polyline[i - 1].y;
        cum[i] = cum[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    const double total = cum.back();
    if (!(total > 1.0e-6) || !std::isfinite(total)) return;

    // Uniform resampling.
    const double want_step = std::max(0.1, params_.profile_step_m);
    const std::size_t n = std::max<std::size_t>(2, static_cast<std::size_t>(std::ceil(total / want_step)) + 1);
    step_ = total / static_cast<double>(n - 1);
    length_ = total;
    px_.resize(n);
    py_.resize(n);
    std::size_t seg = 1;
    for (std::size_t k = 0; k < n; ++k) {
        const double s = std::min(total, step_ * static_cast<double>(k));
        while (seg < polyline.size() - 1 && cum[seg] < s) ++seg;
        const double seg_len = cum[seg] - cum[seg - 1];
        const double t = seg_len > 0.0 ? (s - cum[seg - 1]) / seg_len : 0.0;
        px_[k] = polyline[seg - 1].x + t * (polyline[seg].x - polyline[seg - 1].x);
        py_[k] = polyline[seg - 1].y + t * (polyline[seg].y - polyline[seg - 1].y);
    }

    // Curvature: circle through the samples +-w around i (window shrinks at the ends).
    kappa_.assign(n, 0.0);
    const std::size_t w = std::max<std::size_t>(2, static_cast<std::size_t>(std::lround(params_.corner_window_m / step_)));
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t we = std::min({w, i, n - 1 - i});
        if (we < 2) continue;
        const double ax = px_[i - we], ay = py_[i - we];
        const double bx = px_[i], by = py_[i];
        const double cx = px_[i + we], cy = py_[i + we];
        const double abx = bx - ax, aby = by - ay;
        const double bcx = cx - bx, bcy = cy - by;
        const double acx = cx - ax, acy = cy - ay;
        const double cross = abx * bcy - aby * bcx;
        const double lab = std::sqrt(abx * abx + aby * aby);
        const double lbc = std::sqrt(bcx * bcx + bcy * bcy);
        const double lac = std::sqrt(acx * acx + acy * acy);
        const double denom = lab * lbc * lac;
        kappa_[i] = denom > 1.0e-12 ? 2.0 * std::fabs(cross) / denom : 0.0;
    }

    // Planned speed: cornering limit, then a backward braking pass, 0 at the end.
    profile_.assign(n, params_.max_speed_mps);
    const double a_lat = std::max(0.1, params_.a_lat_mps2);
    for (std::size_t i = 0; i < n; ++i) {
        if (kappa_[i] > 1.0e-9) profile_[i] = std::min(profile_[i], std::sqrt(a_lat / kappa_[i]));
    }
    profile_[n - 1] = 0.0;
    const double decel = std::max(0.1, params_.brake_decel_mps2);
    for (std::size_t i = n - 1; i-- > 0;) {
        profile_[i] = std::min(profile_[i], std::sqrt(profile_[i + 1] * profile_[i + 1] + 2.0 * decel * step_));
    }
    valid_ = true;
}

void RouteFollower::seek(double s) {
    cursor_ = valid_ ? std::min(static_cast<std::size_t>(clamp(s, 0.0, length_) / step_), px_.size() - 2) : 0;
    have_cursor_ = valid_;
    integral_ = 0.0;
}

void RouteFollower::reset() {
    cursor_ = 0;
    have_cursor_ = false;
    integral_ = 0.0;
}

double RouteFollower::target_speed_at(double s) const {
    if (!valid_) return 0.0;
    const double u = clamp(s, 0.0, length_) / step_;
    const std::size_t i = std::min(static_cast<std::size_t>(u), profile_.size() - 2);
    const double t = u - static_cast<double>(i);
    return profile_[i] + t * (profile_[i + 1] - profile_[i]);
}

RouteFollower::Projection RouteFollower::project(double qx, double qy) const {
    const std::size_t nseg = px_.size() - 1;
    auto eval = [&](std::size_t k) {
        Projection p;
        p.index = k;
        const double dx = px_[k + 1] - px_[k], dy = py_[k + 1] - py_[k];
        const double len2 = dx * dx + dy * dy;
        const double rx = qx - px_[k], ry = qy - py_[k];
        double t = len2 > 0.0 ? (rx * dx + ry * dy) / len2 : 0.0;
        t = clamp(t, 0.0, 1.0);
        const double ex = rx - t * dx, ey = ry - t * dy;
        p.t = t;
        p.distance = std::sqrt(ex * ex + ey * ey);
        p.side = dx * ry - dy * rx; // cross(dir, r): > 0 = left of the route
        return p;
    };
    auto search = [&](std::size_t lo, std::size_t hi) {
        Projection best = eval(lo);
        for (std::size_t k = lo + 1; k <= hi; ++k) {
            const Projection p = eval(k);
            if (p.distance < best.distance - 1.0e-9) best = p; // ties: the lowest index
        }
        return best;
    };
    if (!have_cursor_) {
        // First fix: the car starts at the beginning of the route (use seek() otherwise). A
        // route that passes its own start again (a loop) must not tie-break onto a later lap.
        const std::size_t first = std::min(nseg - 1, static_cast<std::size_t>(std::ceil(60.0 / step_)));
        Projection b = search(0, first);
        return b.distance > 25.0 ? search(0, nseg - 1) : b;
    }
    const std::size_t back = static_cast<std::size_t>(std::ceil(params_.search_back_m / step_));
    const std::size_t ahead = static_cast<std::size_t>(std::ceil(params_.search_ahead_m / step_));
    const std::size_t lo = cursor_ > back ? cursor_ - back : 0;
    const std::size_t hi = std::min(nseg - 1, cursor_ + ahead);
    Projection best = search(lo, hi);
    if (best.distance > 25.0) best = search(0, nseg - 1); // lost: re-acquire globally
    return best;
}

void RouteFollower::point_at(double s, double& x, double& y) const {
    const double u = clamp(s, 0.0, length_) / step_;
    const std::size_t i = std::min(static_cast<std::size_t>(u), px_.size() - 2);
    const double t = u - static_cast<double>(i);
    x = px_[i] + t * (px_[i + 1] - px_[i]);
    y = py_[i] + t * (py_[i + 1] - py_[i]);
}

FollowerCommand RouteFollower::step(const FollowerState& st, double dt) {
    FollowerCommand cmd;
    if (!valid_) {
        cmd.finished = true;
        cmd.brake = params_.max_brake;
        return cmd;
    }
    // Rear-axle reference point.
    const double rx = st.x + st.hx * params_.rear_axle_x_m;
    const double ry = st.y + st.hy * params_.rear_axle_x_m;

    const Projection pr = project(rx, ry);
    cursor_ = pr.index;
    have_cursor_ = true;
    const double s = (static_cast<double>(pr.index) + pr.t) * step_;
    cmd.s = s;
    cmd.cross_track = pr.side >= 0.0 ? pr.distance : -pr.distance;

    // Pure pursuit.
    const double v = st.speed;
    const double ld = clamp(params_.lookahead_gain_s * std::fabs(v), params_.lookahead_min_m, params_.lookahead_max_m);
    cmd.lookahead_m = ld;
    double tx = 0.0, ty = 0.0;
    point_at(s + ld, tx, ty);
    const double vx = tx - rx, vy = ty - ry;
    const double d = std::sqrt(vx * vx + vy * vy);
    if (d > 0.25) {
        const double sin_alpha_d = (st.hx * vy - st.hy * vx) / d; // sin(alpha), alpha = bearing - heading
        const double delta = ps::math::atan2(2.0 * params_.wheelbase_m * sin_alpha_d, d);
        cmd.steer = clamp(delta / params_.max_wheel_angle_rad, -1.0, 1.0);
    }

    // Speed.
    cmd.target_speed = target_speed_at(s);
    const bool at_end = (length_ - s) <= params_.finish_distance_m;
    if (at_end && std::fabs(v) <= params_.finish_speed_mps) cmd.finished = true;
    if (cmd.finished) {
        cmd.throttle = 0.0;
        cmd.brake = params_.max_brake;
        integral_ = 0.0;
        return cmd;
    }
    const double e = cmd.target_speed - v;
    if (e < -params_.brake_deadband_mps) {
        cmd.throttle = 0.0;
        cmd.brake = clamp(params_.kp_brake * (-e - params_.brake_deadband_mps), 0.0, params_.max_brake);
        integral_ = 0.0;
    } else {
        const double i_term = clamp(params_.ki_throttle * integral_, -params_.integral_limit, params_.integral_limit);
        const double u = params_.kp_throttle * e + i_term;
        cmd.throttle = clamp(u, 0.0, params_.max_throttle);
        if (cmd.target_speed < 0.3) cmd.throttle = 0.0;
        const bool saturated_high = u >= params_.max_throttle && e > 0.0;
        const bool saturated_low = u <= 0.0 && e < 0.0;
        if (!saturated_high && !saturated_low) {
            integral_ += e * dt;
            const double lim = params_.integral_limit / std::max(1.0e-9, params_.ki_throttle);
            integral_ = clamp(integral_, -lim, lim);
        }
    }
    return cmd;
}

RouteFollowerParams follower_params_for_vehicle(const ps::vehicle::VehicleDesc& desc, const RouteFollowerParams& base) {
    RouteFollowerParams p = base;
    double front = 0.0, rear = 0.0;
    int nf = 0, nr = 0;
    for (const auto& w : desc.wheels) {
        if (w.steered) {
            front += w.attachment_local.x;
            ++nf;
        } else {
            rear += w.attachment_local.x;
            ++nr;
        }
    }
    if (nf > 0 && nr > 0) {
        front /= nf;
        rear /= nr;
        p.wheelbase_m = front - rear;
        p.rear_axle_x_m = rear;
    }
    if (desc.steering.max_wheel_angle > 0.0) p.max_wheel_angle_rad = desc.steering.max_wheel_angle;
    return p;
}

DriveScript::Controller make_route_follower_controller(
    RouteFollower follower, double tick_dt,
    std::function<void(const DriveTickContext&, const FollowerState&, const FollowerCommand&)> on_command) {
    bool initialised = false;
    return [follower = std::move(follower), tick_dt, on_command = std::move(on_command), initialised](
               const DriveTickContext& ctx, ps::World& world) mutable {
        if (!initialised) {
            world.set_control("ignition", 1.0);
            world.set_control("assist.auto_clutch", 1.0);
            world.set_control("assist.auto_shift", 1.0);
            world.set_control("handbrake", 0.0);
            initialised = true;
        }
        const ps::Pose pose = world.get_pose(ctx.chassis);
        const ps::Motion motion = world.get_motion(ctx.chassis);
        const ps::Vec3 fwd = pose.orientation.rotate(ps::Vec3::unit_x());
        const double hlen = std::sqrt(fwd.x * fwd.x + fwd.y * fwd.y);
        FollowerState st;
        st.x = pose.position.x;
        st.y = pose.position.y;
        if (hlen > 1.0e-6) {
            st.hx = fwd.x / hlen;
            st.hy = fwd.y / hlen;
        }
        st.speed = motion.linear.x * fwd.x + motion.linear.y * fwd.y + motion.linear.z * fwd.z;
        const FollowerCommand cmd = follower.step(st, tick_dt);
        world.set_control("steer", cmd.steer);
        world.set_control("throttle", cmd.throttle);
        world.set_control("brake", cmd.brake);
        if (on_command) on_command(ctx, st, cmd);
    };
}

} // namespace rg
