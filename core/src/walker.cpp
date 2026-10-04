#include "rg/walker.h"

#include "ps/backend/shape_desc.h"
#include "ps/math/quat.h"
#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>

namespace rg {

namespace {

constexpr double kPi = 3.141592653589793;
constexpr double kPenEps = 1.0e-4;       // overlaps below this are ignored
constexpr double kSkin = 0.002;          // keeps the capsule just outside walls
constexpr double kStepTol = 0.01;

// Capsule axis is local Y; +90 deg about X stands it up.
const ps::Quat kUpright{0.7071067811865476, 0.0, 0.0, 0.7071067811865476};

double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

double wrap_pi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

// Rect-local coordinates of (x, y).
void to_local(const OrientedRect& r, double x, double y, double& lx, double& ly) {
    const double dx = x - r.cx, dy = y - r.cy;
    lx = r.cos_yaw * dx + r.sin_yaw * dy;
    ly = -r.sin_yaw * dx + r.cos_yaw * dy;
}

void from_local(const OrientedRect& r, double lx, double ly, double& x, double& y) {
    x = r.cx + r.cos_yaw * lx - r.sin_yaw * ly;
    y = r.cy + r.sin_yaw * lx + r.cos_yaw * ly;
}

} // namespace

OrientedRect make_oriented_rect(double cx, double cy, double yaw_rad, double half_x, double half_y, double z_min,
                                double z_max) {
    OrientedRect r;
    r.cx = cx;
    r.cy = cy;
    r.half_x = half_x;
    r.half_y = half_y;
    ps::math::sincos(yaw_rad, r.sin_yaw, r.cos_yaw);
    r.z_min = z_min;
    r.z_max = z_max;
    return r;
}

double distance_to_rect(const OrientedRect& rect, double x, double y) {
    double lx = 0.0, ly = 0.0;
    to_local(rect, x, y, lx, ly);
    const double ddx = lx - clampd(lx, -rect.half_x, rect.half_x);
    const double ddy = ly - clampd(ly, -rect.half_y, rect.half_y);
    return std::sqrt(ddx * ddx + ddy * ddy);
}

bool push_circle_out_of_rect(const OrientedRect& rect, double radius, double& x, double& y) {
    double lx = 0.0, ly = 0.0;
    to_local(rect, x, y, lx, ly);
    const double qx = clampd(lx, -rect.half_x, rect.half_x);
    const double qy = clampd(ly, -rect.half_y, rect.half_y);
    const double ddx = lx - qx, ddy = ly - qy;
    const double d2 = ddx * ddx + ddy * ddy;
    double nx = lx, ny = ly;
    if (d2 > 0.0) {
        const double d = std::sqrt(d2);
        if (d >= radius) return false;
        const double k = radius / d; // move to distance `radius` along the same direction from the closest point
        nx = qx + ddx * k;
        ny = qy + ddy * k;
    } else {
        // Centre inside the rectangle: out through the nearest edge.
        const double ex = rect.half_x - std::fabs(lx);
        const double ey = rect.half_y - std::fabs(ly);
        if (ex < ey) {
            nx = (lx >= 0.0 ? 1.0 : -1.0) * (rect.half_x + radius);
        } else {
            ny = (ly >= 0.0 ? 1.0 : -1.0) * (rect.half_y + radius);
        }
    }
    from_local(rect, nx, ny, x, y);
    return true;
}

OrientedRect vehicle_footprint(const ps::Vec3& chassis_position, const ps::Vec3& chassis_forward_world,
                               double chassis_half_x, double chassis_half_y, std::span<const WheelFootprint> wheels,
                               double margin_m, double z_min, double z_max) {
    double fx = chassis_forward_world.x, fy = chassis_forward_world.y;
    const double fl = std::sqrt(fx * fx + fy * fy);
    if (fl > 1.0e-9) {
        fx /= fl;
        fy /= fl;
    } else {
        fx = 1.0;
        fy = 0.0;
    }
    double x_lo = -chassis_half_x, x_hi = chassis_half_x, y_lo = -chassis_half_y, y_hi = chassis_half_y;
    for (const WheelFootprint& w : wheels) {
        x_lo = std::min(x_lo, w.x - w.radius);
        x_hi = std::max(x_hi, w.x + w.radius);
        y_lo = std::min(y_lo, w.y - w.half_width);
        y_hi = std::max(y_hi, w.y + w.half_width);
    }
    x_lo -= margin_m;
    x_hi += margin_m;
    y_lo -= margin_m;
    y_hi += margin_m;
    const double mid_x = 0.5 * (x_lo + x_hi), mid_y = 0.5 * (y_lo + y_hi);
    OrientedRect r;
    r.cos_yaw = fx;
    r.sin_yaw = fy;
    r.half_x = 0.5 * (x_hi - x_lo);
    r.half_y = 0.5 * (y_hi - y_lo);
    r.cx = chassis_position.x + fx * mid_x - fy * mid_y;
    r.cy = chassis_position.y + fy * mid_x + fx * mid_y;
    r.z_min = z_min;
    r.z_max = z_max;
    return r;
}

bool within_enter_range(const OrientedRect& footprint, const ps::Vec3& walker_feet, double range_m, double max_dz_m) {
    if (!(distance_to_rect(footprint, walker_feet.x, walker_feet.y) <= range_m)) return false;
    return std::fabs(walker_feet.z - (footprint.z_min + 0.5)) <= max_dz_m;
}

SpawnSpot select_spawn_spot(const OrientedRect& footprint, const ps::Vec3& chassis_position, double chassis_yaw_rad,
                            double walker_radius_m, double door_x_m, double clearance_m, double roof_z_m,
                            const SpawnProbe& probe) {
    double s = 0.0, c = 1.0;
    ps::math::sincos(chassis_yaw_rad, s, c);
    // The footprint's extent in the chassis' own axes.
    const double rx = footprint.cx - chassis_position.x, ry = footprint.cy - chassis_position.y;
    const double lcx = c * rx + s * ry;
    const double lcy = -s * rx + c * ry;
    const double rear = lcx - footprint.half_x, front = lcx + footprint.half_x;
    const double right = lcy - footprint.half_y, left = lcy + footprint.half_y;
    const double off = walker_radius_m + clearance_m;

    struct Candidate {
        double lx, ly, yaw_off;
    };
    // Per side: the base spot, then two steps further out.
    static constexpr double kExtra[3] = {0.0, 0.7, 1.4};
    for (int side = 0; side < 4; ++side) {
        for (double extra : kExtra) {
            Candidate cd{};
            switch (side) {
                case 0: cd = {door_x_m, left + off + extra, 0.5 * kPi}; break;
                case 1: cd = {door_x_m, right - off - extra, -0.5 * kPi}; break;
                case 2: cd = {rear - off - extra, lcy, kPi}; break;
                default: cd = {front + off + extra, lcy, 0.0}; break;
            }
            const double wx = chassis_position.x + c * cd.lx - s * cd.ly;
            const double wy = chassis_position.y + s * cd.lx + c * cd.ly;
            if (const std::optional<double> z = probe(wx, wy)) {
                SpawnSpot spot;
                spot.feet = ps::Vec3{wx, wy, *z};
                spot.yaw_rad = wrap_pi(chassis_yaw_rad + cd.yaw_off);
                spot.candidate = side;
                return spot;
            }
        }
    }
    SpawnSpot spot;
    spot.feet = ps::Vec3{chassis_position.x, chassis_position.y, roof_z_m};
    spot.yaw_rad = chassis_yaw_rad;
    spot.candidate = 4;
    spot.above_roof = true;
    return spot;
}

// ---------------------------------------------------------------------------

WalkerController::WalkerController(ps::World& world, const WalkerConfig& config) : world_(world), config_(config) {
    double s = 0.0, c = 1.0;
    ps::math::sincos(config.slope_limit_deg * kPi / 180.0, s, c);
    cos_slope_ = c;
}

WalkerController::~WalkerController() { despawn(); }

ps::Vec3 WalkerController::centre_of(const ps::Vec3& feet) const {
    return ps::Vec3{feet.x, feet.y, feet.z + 0.5 * config_.height_m};
}

void WalkerController::spawn(const ps::Vec3& feet, double yaw_rad) {
    despawn();
    ps::BodyDesc desc;
    desc.shape = ps::CapsuleShape{0.5 * config_.height_m - config_.radius_m, config_.radius_m};
    desc.motion = ps::BodyMotionType::Kinematic;
    desc.mass = 80.0;
    desc.pose = ps::Pose{centre_of(feet), kUpright};
    desc.friction = 0.5;
    desc.gravity_enabled = false;
    desc.allow_sleep = false;
    body_ = world_.create_body(desc);
    state_ = WalkerState{};
    state_.feet = feet;
    state_.yaw_rad = yaw_rad;
    state_.grounded = true;
}

void WalkerController::despawn() {
    if (!body_.valid()) return;
    world_.destroy_body(body_);
    body_ = ps::BodyId{};
}

void WalkerController::sync_from_body() {
    if (!active()) return;
    const ps::Pose p = world_.backend().get_pose(body_);
    state_.feet = ps::Vec3{p.position.x, p.position.y, p.position.z - 0.5 * config_.height_m};
}

void WalkerController::halt() {
    if (active()) world_.backend().set_motion(body_, ps::Motion{});
}

bool WalkerController::ignored(ps::BodyId id) const {
    if (id == body_) return true;
    return std::find(ignored_.begin(), ignored_.end(), id) != ignored_.end();
}

std::optional<WalkerController::Ground> WalkerController::ray_down(double x, double y, double z_from,
                                                                   double dist) const {
    // The walker's own body is excluded by the backend's own filter (a ray that
    // starts inside a convex shape would otherwise report it at fraction 0).
    // The other ignored bodies (the own car's thin chassis box - only reachable
    // from the roof-spawn fallback, the footprint keeps the walker out of it
    // otherwise) are skipped by re-casting from below their thickness.
    constexpr double kIgnoredBodySkip = 0.35;
    double z = z_from;
    double remaining = dist;
    for (int attempt = 0; attempt < 3 && remaining > 0.0; ++attempt) {
        const ps::RayCastHit hit =
            world_.backend().ray_cast_excluding(ps::Vec3{x, y, z}, ps::Vec3{0.0, 0.0, -1.0}, remaining, body_);
        if (!hit.hit) return std::nullopt;
        if (std::find(ignored_.begin(), ignored_.end(), hit.body) != ignored_.end()) {
            const double used = (z - hit.point.z) + kIgnoredBodySkip;
            z -= used;
            remaining -= used;
            continue;
        }
        Ground g;
        g.z = hit.point.z;
        g.normal = hit.normal.length_squared() > 0.5 ? hit.normal.normalized() : ps::Vec3{0.0, 0.0, 1.0};
        if (g.normal.z < 0.0) g.normal = -g.normal; // a ray from above sees an upward face
        return g;
    }
    return std::nullopt;
}

ps::Vec3 WalkerController::resolve(ps::Vec3& feet, bool allow_up, double ref_z, const ps::Vec3& move_dir,
                                   bool& hit_ceiling) const {
    const ps::Vec3 start = feet;
    const ps::CapsuleShape shape{0.5 * config_.height_m - config_.radius_m, config_.radius_m};
    for (int iter = 0; iter < 4; ++iter) {
        const ps::Vec3 centre = centre_of(feet);
        world_.backend().collide_shape_into(shape, ps::Pose{centre, kUpright}, hits_);
        bool found = false;
        double best_pen = 0.0;
        ps::Vec3 best_push{};
        bool best_ceiling = false;
        for (const ps::ShapeContactHit& h : hits_) {
            if (ignored(h.body) || !(h.penetration > kPenEps)) continue;
            const ps::Vec3 n = h.normal;
            ps::Vec3 push{};
            bool ceiling = false;
            // A contact within step height of the reference feet is a kerb, a
            // slope or a step edge: the raise/drop logic and the ground ray own
            // it (an edge contact has an in-between normal that would otherwise
            // read as a wall and stop every step).
            const bool low = h.point.z <= ref_z + config_.step_height_m + kStepTol;
            if (n.z >= cos_slope_) {
                if (allow_up) {
                    push = ps::Vec3{0.0, 0.0, std::min(h.penetration / n.z, 0.2)};
                } else if (!low) {
                    // The top of something taller than a step: a wall.
                    double dx = centre.x - h.point.x, dy = centre.y - h.point.y;
                    double len = std::sqrt(dx * dx + dy * dy);
                    if (len < 1.0e-4) {
                        dx = -move_dir.x;
                        dy = -move_dir.y;
                        len = std::sqrt(dx * dx + dy * dy);
                    }
                    if (len < 1.0e-9) continue;
                    const double k = std::min(h.penetration, 0.1) / len;
                    push = ps::Vec3{dx * k, dy * k, 0.0};
                } else {
                    continue; // a floor within step range: handled by the drop
                }
            } else if (n.z < -0.3) {
                push = n * std::min(h.penetration, 0.2);
                ceiling = true;
            } else {
                if (low) continue;
                const double len = std::sqrt(n.x * n.x + n.y * n.y);
                if (len < 1.0e-6) continue;
                const double k = std::min(h.penetration / len, 0.5) + kSkin;
                push = ps::Vec3{n.x / len * k, n.y / len * k, 0.0};
            }
            if (!found || h.penetration > best_pen) {
                found = true;
                best_pen = h.penetration;
                best_push = push;
                best_ceiling = ceiling;
            }
        }
        if (!found) break;
        feet += best_push;
        if (best_ceiling) hit_ceiling = true;
    }
    return feet - start;
}

void WalkerController::apply_rects(ps::Vec3& feet) const {
    for (const OrientedRect& r : obstacles_) {
        if (!(feet.z < r.z_max - 0.02 && feet.z + config_.height_m > r.z_min)) continue;
        push_circle_out_of_rect(r, config_.radius_m + kSkin, feet.x, feet.y);
    }
}

WalkerController::GroundMove WalkerController::try_ground_move(const ps::Vec3& feet, double dx, double dy) const {
    GroundMove out;
    out.feet = feet;
    const double step = config_.step_height_m;
    const double dl = std::sqrt(dx * dx + dy * dy);
    const ps::Vec3 dir = dl > 1.0e-12 ? ps::Vec3{dx / dl, dy / dl, 0.0} : ps::Vec3{};

    // Raise, move, push out of walls (everything taller than a step), drop.
    ps::Vec3 q{feet.x + dx, feet.y + dy, feet.z + step};
    bool ceiling = false;
    resolve(q, false, feet.z, dir, ceiling);
    ps::Vec3 qr = q;
    qr.z = feet.z;
    apply_rects(qr);
    q.x = qr.x;
    q.y = qr.y;
    resolve(q, false, feet.z, dir, ceiling);

    const double above = 0.05;
    const std::optional<Ground> g = ray_down(q.x, q.y, feet.z + step + above, step + above + config_.snap_down_m);
    if (!g) {
        // Nothing within snap range below: an edge, the walker steps off it.
        out.ok = true;
        out.ledge = true;
        out.feet = ps::Vec3{q.x, q.y, feet.z};
        return out;
    }
    const bool walkable = g->normal.z >= cos_slope_;
    const double rise = g->z - feet.z;
    if (!walkable && rise > 1.0e-3) {
        // Steep ground going up: a wall; slide along its horizontal normal.
        out.slide_normal = g->normal;
        out.have_slide_normal = true;
        return out;
    }
    if (rise > step + kStepTol) {
        out.slide_normal = ps::Vec3{-dir.x, -dir.y, 0.0};
        out.have_slide_normal = dl > 1.0e-12;
        return out;
    }
    ps::Vec3 f{q.x, q.y, g->z};
    resolve(f, true, f.z, dir, ceiling);
    apply_rects(f);
    if (f.z - feet.z > step + 0.05) {
        out.slide_normal = ps::Vec3{-dir.x, -dir.y, 0.0};
        out.have_slide_normal = dl > 1.0e-12;
        return out;
    }
    out.ok = true;
    out.grounded = true;
    out.ground = Ground{g->z, g->normal};
    out.feet = f;
    return out;
}

void WalkerController::write_target(const ps::Vec3& target_feet, double dt) {
    const ps::Pose cur = world_.backend().get_pose(body_);
    const ps::Vec3 target_centre = centre_of(target_feet);
    ps::Motion m;
    m.linear = (target_centre - cur.position) / dt;
    world_.backend().set_motion(body_, m);
}

const WalkerState& WalkerController::step(const WalkerInput& input, double dt) {
    sync_from_body();
    WalkerState& st = state_;
    const ps::Vec3 feet0 = st.feet;

    // --- horizontal wish velocity, relative to the look heading
    double ml = std::sqrt(input.move_right * input.move_right + input.move_forward * input.move_forward);
    const double scale = ml > 1.0 ? 1.0 / ml : 1.0;
    double sy = 0.0, cy = 1.0;
    ps::math::sincos(input.look_yaw_rad, sy, cy);
    const double speed = input.run ? config_.run_speed_mps : config_.walk_speed_mps;
    const double wx = (cy * input.move_forward + sy * input.move_right) * scale * speed;
    const double wy = (sy * input.move_forward - cy * input.move_right) * scale * speed;
    double vx = st.velocity.x, vy = st.velocity.y;
    {
        double ddx = wx - vx, ddy = wy - vy;
        const double dl = std::sqrt(ddx * ddx + ddy * ddy);
        const double max_d = (st.grounded ? config_.ground_accel_mps2 : config_.air_accel_mps2) * dt;
        if (dl > max_d && dl > 0.0) {
            const double k = max_d / dl;
            ddx *= k;
            ddy *= k;
        }
        vx += ddx;
        vy += ddy;
    }

    // --- vertical
    double vz = st.velocity.z;
    if (st.grounded && input.jump) {
        vz = config_.jump_speed_mps;
        st.grounded = false;
    }
    if (st.grounded) {
        vz = 0.0;
    } else {
        vz = std::max(vz - config_.gravity_mps2 * dt, -config_.max_fall_speed_mps);
    }

    ps::Vec3 feet = feet0;
    bool grounded = st.grounded;
    ps::Vec3 normal = st.ground_normal;
    bool hold = false;
    bool blocked = false;
    double dx = vx * dt, dy = vy * dt;
    // The horizontal push the obstacles applied against the intended move
    // (zero when nothing was in the way); the velocity loses its component
    // along it so a walker pressed against a wall does not keep "running".
    double push_x = 0.0, push_y = 0.0;

    if (grounded) {
        GroundMove m = try_ground_move(feet, dx, dy);
        double ux = dx, uy = dy;
        if (!m.ok && m.have_slide_normal) {
            const double nl = std::sqrt(m.slide_normal.x * m.slide_normal.x + m.slide_normal.y * m.slide_normal.y);
            if (nl > 1.0e-9) {
                const double nx = m.slide_normal.x / nl, ny = m.slide_normal.y / nl;
                const double into = dx * nx + dy * ny;
                if (into < 0.0) {
                    const double sx = dx - nx * into, sy = dy - ny * into;
                    const GroundMove s = try_ground_move(feet, sx, sy);
                    // The slide itself is the obstacle's doing: the push is what
                    // it took away from the intended move.
                    if (s.ok) {
                        m = s;
                        ux = sx;
                        uy = sy;
                        push_x = -nx * into; // away from the wall (into < 0)
                        push_y = -ny * into;
                        blocked = true;
                    }
                }
            }
        }
        if (m.ok) {
            feet = m.feet;
            push_x += feet.x - (feet0.x + ux);
            push_y += feet.y - (feet0.y + uy);
            if (m.ledge) {
                grounded = false;
                vz = 0.0;
            } else {
                grounded = true;
                normal = m.ground.normal;
            }
        } else {
            // Fully blocked: stay put (still on the ground).
            vx = 0.0;
            vy = 0.0;
            blocked = true;
        }
    } else {
        ps::Vec3 q{feet.x + dx, feet.y + dy, feet.z + vz * dt};
        const ps::Vec3 intended = q;
        bool ceiling = false;
        const ps::Vec3 dir{dx, dy, 0.0};
        resolve(q, true, feet.z, dir, ceiling);
        apply_rects(q);
        push_x = q.x - intended.x;
        push_y = q.y - intended.y;
        if (ceiling && vz > 0.0) vz = 0.0;
        if (vz <= 0.0) {
            // Landing: never below the ground, and never through it. The ray
            // starts a little above the old feet so a surface rising under the
            // walker is still seen.
            const double top = std::max(feet.z, q.z) + 0.3;
            std::optional<Ground> g = ray_down(q.x, q.y, top, top - q.z + 0.25);
            if (!g) {
                // Nothing within the sweep. A long ray tells a ledge/fall from a
                // missing (not loaded) ground: hold the height in that case.
                const std::optional<Ground> far = ray_down(q.x, q.y, top, 3000.0);
                if (!far) {
                    q.z = feet.z;
                    vz = 0.0;
                    hold = true;
                }
            } else if (q.z <= g->z + 1.0e-3) {
                q.z = g->z;
                vz = 0.0;
                if (g->normal.z >= cos_slope_) {
                    grounded = true;
                    normal = g->normal;
                }
            }
        }
        feet = q;
    }

    const double pl = std::sqrt(push_x * push_x + push_y * push_y);
    if (pl > 1.0e-6) {
        blocked = true;
        const double px = push_x / pl, py = push_y / pl;
        const double into = vx * px + vy * py;
        if (into < 0.0) {
            vx -= px * into;
            vy -= py * into;
        }
    }

    // --- bookkeeping
    st.blocked = blocked;
    st.feet = feet;
    st.velocity = ps::Vec3{vx, vy, grounded ? 0.0 : vz};
    st.grounded = grounded;
    st.ground_normal = normal;
    st.hold = hold;

    // Facing follows the movement direction (display only).
    const double hs2 = vx * vx + vy * vy;
    if (hs2 > 0.3 * 0.3) {
        const double target = ps::math::atan2(vy, vx);
        const double diff = wrap_pi(target - st.yaw_rad);
        const double max_turn = config_.turn_rate_rad_s * dt;
        st.yaw_rad = wrap_pi(st.yaw_rad + clampd(diff, -max_turn, max_turn));
    }

    write_target(feet, dt);
    return st;
}

} // namespace rg
