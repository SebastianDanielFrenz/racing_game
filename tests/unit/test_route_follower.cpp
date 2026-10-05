// test_route_follower.cpp — rg::RouteFollower (G3/R3 slice S2, docs/g3_consumer_design.md
// sections 6.1 and 8): pure pursuit + speed PI, driven through the real rg::Session in
// the flat world (the car is the game's own physics_sim car_sedan.json) on synthetic
// routes. Tag [route_follower].
//
// Recorded sabotages (the design names "flip the steering sign -> circle test fails"),
// each applied to core/src/route_follower.cpp alone, release build, then reverted:
//   1. steering sign flipped (delta -> -delta): the circle test fails (max |r - R| 349 m,
//      speed 5.7 m/s), the S-bend test fails (172.6 m off the route), and the unit sign
//      case and "stops at the route end" fail too (4 of 8 cases);
//   2. projection searches the whole polyline instead of the window around the previous
//      projection: the out-and-back case fails (the projection jumps onto the return leg,
//      s = 193 at x = 10 vs the outbound leg's s <= 100) and so does the circle (the first
//      fix tie-breaks onto a later lap); 36 assertions;
//   3. the speed plan's backward braking pass removed: the corner-plan case fails (the plan
//      stays at the 25 m/s cap 50 m before a 7 m corner), the S-bend leaves the route by
//      7.7 m and the stop-at-end case overshoots to x = 155 (3 of 8 cases).

#include "rg/route_follower.h"

#include "rg/drive_script.h"
#include "rg/route_check.h"
#include "rg/session.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

rg::SessionConfig flat_config(unsigned workers = 0) {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json";
    config.job_workers = workers;
    return config;
}

struct Sample {
    double t = 0.0;
    double x = 0.0, y = 0.0; // rear axle
    double speed = 0.0;
    double target = 0.0;
    double cross_track = 0.0;
    double s = 0.0;
    double steer = 0.0;
};

struct Run {
    std::vector<Sample> samples;
    std::uint64_t command_hash = 0xcbf29ce484222325ull;
    std::uint64_t world_hash = 0;
    bool finished = false;
};

void fold(std::uint64_t& h, double v) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof bits);
    for (int i = 0; i < 8; ++i) {
        h ^= (bits >> (8 * i)) & 0xffu;
        h *= 0x100000001b3ull;
    }
}

Run follow(const std::vector<rg::RoutePoint>& route, rg::RouteFollowerParams params, double seconds,
           unsigned workers = 0) {
    rg::Session session(flat_config(workers));
    params = rg::follower_params_for_vehicle(session.vehicle_desc(), params);
    rg::RouteFollower follower(route, params);
    REQUIRE(follower.valid());
    const double dt = 1.0 / 240.0;
    Run run;
    rg::DriveScript script;
    script.set_controller(rg::make_route_follower_controller(
        follower, dt,
        [&](const rg::DriveTickContext& ctx, const rg::FollowerState& st, const rg::FollowerCommand& cmd) {
            fold(run.command_hash, cmd.steer);
            fold(run.command_hash, cmd.throttle);
            fold(run.command_hash, cmd.brake);
            if (cmd.finished) run.finished = true;
            if (ctx.drive_tick % 12 == 0) {
                Sample s;
                s.t = static_cast<double>(ctx.drive_tick) * dt;
                s.x = st.x + st.hx * params.rear_axle_x_m;
                s.y = st.y + st.hy * params.rear_axle_x_m;
                s.speed = st.speed;
                s.target = cmd.target_speed;
                s.cross_track = cmd.cross_track;
                s.s = cmd.s;
                s.steer = cmd.steer;
                run.samples.push_back(s);
            }
        }));
    session.set_drive_script(std::move(script));
    const std::uint64_t ticks = static_cast<std::uint64_t>(seconds * 240.0);
    for (std::uint64_t i = 0; i < ticks; ++i) session.step();
    run.world_hash = session.world().state_hash();
    return run;
}

// CCW circle of radius r through the origin, tangent to +x there (centre (0, r)), `laps` laps.
std::vector<rg::RoutePoint> circle_route(double r, double laps, double step_m = 2.0) {
    std::vector<rg::RoutePoint> pts;
    const double total = 2.0 * kPi * r * laps;
    const int n = static_cast<int>(total / step_m) + 1;
    for (int i = 0; i <= n; ++i) {
        const double th = total * i / n / r;
        pts.push_back({r * std::sin(th), r * (1.0 - std::cos(th))});
    }
    return pts;
}

// Straight 40 m, left arc (R, deg), right arc (R, deg), straight 40 m, starting at the origin along +x.
std::vector<rg::RoutePoint> s_bend_route(double r, double deg) {
    std::vector<rg::RoutePoint> pts;
    double x = 0.0, y = 0.0, h = 0.0;
    auto line = [&](double len) {
        const int n = static_cast<int>(len);
        for (int i = 0; i < n; ++i) {
            pts.push_back({x, y});
            x += std::cos(h);
            y += std::sin(h);
        }
    };
    auto arc = [&](double radius, double sweep_deg) { // + = left
        const double sweep = sweep_deg * kPi / 180.0;
        const int n = std::max(2, static_cast<int>(std::fabs(sweep) * radius));
        const double dh = sweep / n;
        const double ds = std::fabs(sweep) * radius / n;
        for (int i = 0; i < n; ++i) {
            pts.push_back({x, y});
            x += ds * std::cos(h + dh * 0.5);
            y += ds * std::sin(h + dh * 0.5);
            h += dh;
        }
    };
    line(40.0);
    arc(r, deg);
    arc(r, -deg);
    line(40.0);
    pts.push_back({x, y});
    return pts;
}

double dist_to_polyline(const std::vector<rg::RoutePoint>& pl, double x, double y) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < pl.size(); ++i) {
        const double dx = pl[i + 1].x - pl[i].x, dy = pl[i + 1].y - pl[i].y;
        const double l2 = dx * dx + dy * dy;
        double t = l2 > 0 ? ((x - pl[i].x) * dx + (y - pl[i].y) * dy) / l2 : 0.0;
        t = std::clamp(t, 0.0, 1.0);
        const double ex = x - (pl[i].x + t * dx), ey = y - (pl[i].y + t * dy);
        best = std::min(best, std::sqrt(ex * ex + ey * ey));
    }
    return best;
}

} // namespace

TEST_CASE("route follower: look-ahead grows with speed and the speed plan brakes before a corner",
          "[route_follower]") {
    // A straight 300 m, then a 90 degree corner (kink), then 300 m: the planned speed at the
    // corner is sqrt(a_lat * R) with R = w / (2 sin(theta/2)) = 7.07 m (route_check's reading
    // of a kink), and the plan must already be low well before it.
    std::vector<rg::RoutePoint> pl;
    for (int i = 0; i <= 300; ++i) pl.push_back({static_cast<double>(i), 0.0});
    for (int i = 1; i <= 300; ++i) pl.push_back({300.0, static_cast<double>(i)});
    rg::RouteFollowerParams p;
    p.max_speed_mps = 25.0;
    rg::RouteFollower f(pl, p);
    REQUIRE(f.valid());
    CHECK(f.length_m() == Catch::Approx(600.0).margin(0.01));
    const double v_corner = f.target_speed_at(300.0);
    CHECK(v_corner == Catch::Approx(std::sqrt(p.a_lat_mps2 * 7.07)).margin(0.6));
    // Braking reaches back until v^2 = v_c^2 + 2 b d meets the cap: 50 m out it is ~19 m/s,
    // 100 m out (beyond sqrt(28 + 7 * 86) = 25) it is the cap again.
    const double v_50 = f.target_speed_at(250.0);
    CHECK(v_50 <= std::sqrt(v_corner * v_corner + 2.0 * p.brake_decel_mps2 * 50.0) + 1e-6);
    CHECK(v_50 < p.max_speed_mps - 3.0);
    CHECK(f.target_speed_at(200.0) == Catch::Approx(25.0));
    CHECK(f.target_speed_at(299.0) < f.target_speed_at(250.0));
    CHECK(f.target_speed_at(600.0) == Catch::Approx(0.0)); // stops at the route end

    // Look-ahead: a car at the start, moving at 3 and at 20 m/s, on a straight route.
    std::vector<rg::RoutePoint> line;
    for (int i = 0; i <= 200; ++i) line.push_back({static_cast<double>(i), 0.0});
    rg::RouteFollower g(line, rg::RouteFollowerParams{});
    rg::FollowerState st;
    st.x = 10.0 - g.params().rear_axle_x_m; // rear axle at x = 10
    st.speed = 3.0;
    const rg::FollowerCommand slow = g.step(st, 1.0 / 240.0);
    st.speed = 20.0;
    const rg::FollowerCommand fast = g.step(st, 1.0 / 240.0);
    CHECK(slow.lookahead_m == Catch::Approx(4.0)); // 0.8 * 3 = 2.4 -> the 4 m minimum
    CHECK(fast.lookahead_m == Catch::Approx(16.0));
    CHECK(slow.steer == Catch::Approx(0.0).margin(1e-12)); // on the line, heading along it
}

TEST_CASE("route follower: sign convention on a unit pose (a route to the left steers left)", "[route_follower]") {
    std::vector<rg::RoutePoint> pl;
    for (int i = 0; i <= 100; ++i) pl.push_back({static_cast<double>(i), 0.0});
    rg::RouteFollower f(pl, rg::RouteFollowerParams{});
    rg::FollowerState st;
    st.x = 10.0 - f.params().rear_axle_x_m;
    st.y = -2.0; // the car is 2 m RIGHT of the route (route is at y = 0, to its left)
    st.speed = 8.0;
    const rg::FollowerCommand c = f.step(st, 1.0 / 240.0);
    CHECK(c.steer > 0.0);        // steer left, towards the route
    CHECK(c.cross_track < 0.0);  // the rear axle is right of the route
    f.reset();
    st.y = 2.0;
    const rg::FollowerCommand d = f.step(st, 1.0 / 240.0);
    CHECK(d.steer < 0.0);
    CHECK(d.cross_track > 0.0);
}

TEST_CASE("route follower: 50 m circle at 15 m/s holds the lateral error under 0.5 m", "[route_follower][session]") {
    rg::RouteFollowerParams p;
    p.max_speed_mps = 15.0;
    p.a_lat_mps2 = 4.5; // 15^2 / 50: the plan asks for the full 15 m/s on this circle
    const double radius = 50.0;
    const std::vector<rg::RoutePoint> route = circle_route(radius, 3.0);
    const Run run = follow(route, p, 50.0);
    REQUIRE(run.samples.size() > 100);
    // Steady state: after the car has settled onto the circle (t >= 15 s).
    double max_err = 0.0;
    double min_speed = 1e9, max_speed = 0.0;
    int n = 0;
    for (const Sample& s : run.samples) {
        if (s.t < 15.0) continue;
        const double d = std::sqrt(s.x * s.x + (s.y - radius) * (s.y - radius));
        max_err = std::max(max_err, std::fabs(d - radius));
        min_speed = std::min(min_speed, s.speed);
        max_speed = std::max(max_speed, s.speed);
        ++n;
    }
    REQUIRE(n > 50);
    INFO("max |r - R| = " << max_err << " m, speed " << min_speed << " .. " << max_speed);
    CHECK(max_err <= 0.5);
    CHECK(min_speed > 14.0);
    CHECK(max_speed < 16.0);
}

TEST_CASE("route follower: S-bend stays within 1 m of the route", "[route_follower][session]") {
    rg::RouteFollowerParams p;
    p.max_speed_mps = 12.0;
    const std::vector<rg::RoutePoint> route = s_bend_route(60.0, 50.0);
    const Run run = follow(route, p, 40.0);
    REQUIRE(run.samples.size() > 100);
    double max_err = 0.0;
    for (const Sample& s : run.samples) {
        if (s.x < 0.0) continue; // the rear axle starts 1.62 m behind the route start
        max_err = std::max(max_err, dist_to_polyline(route, s.x, s.y));
    }
    INFO("max route distance " << max_err << " m");
    CHECK(max_err <= 1.0);
    // It drove the whole thing: the rear axle ended near the end of the route.
    const Sample& last = run.samples.back();
    CHECK(std::sqrt((last.x - route.back().x) * (last.x - route.back().x) +
                    (last.y - route.back().y) * (last.y - route.back().y)) < 3.0);
}

TEST_CASE("route follower: speed within 1 m/s of the plan after 5 s", "[route_follower][session]") {
    rg::RouteFollowerParams p;
    p.max_speed_mps = 12.0;
    std::vector<rg::RoutePoint> route;
    for (int i = 0; i <= 600; ++i) route.push_back({static_cast<double>(i), 0.0});
    const Run run = follow(route, p, 40.0);
    REQUIRE(run.samples.size() > 100);
    double worst = 0.0;
    int n = 0;
    for (const Sample& s : run.samples) {
        // The plan brakes for the route end (600 m); judge the cruise part only.
        if (s.t < 5.0 || s.target < 11.9) continue;
        worst = std::max(worst, std::fabs(s.speed - s.target));
        ++n;
    }
    REQUIRE(n > 50);
    INFO("worst |v - target| after 5 s = " << worst << " m/s over " << n << " samples");
    CHECK(worst <= 1.0);
}

TEST_CASE("route follower: stops at the route end", "[route_follower][session]") {
    rg::RouteFollowerParams p;
    p.max_speed_mps = 10.0;
    std::vector<rg::RoutePoint> route;
    for (int i = 0; i <= 150; ++i) route.push_back({static_cast<double>(i), 0.0});
    const Run run = follow(route, p, 40.0);
    CHECK(run.finished);
    const Sample& last = run.samples.back();
    CHECK(std::fabs(last.speed) < 0.6);
    CHECK(last.x > 146.0); // rear axle within a few metres of the end, never beyond it by much
    CHECK(last.x < 152.0);
}

TEST_CASE("route follower: identical commands and world state at 1 and 10 workers", "[route_follower][session][determinism]") {
    rg::RouteFollowerParams p;
    p.max_speed_mps = 12.0;
    const std::vector<rg::RoutePoint> route = s_bend_route(60.0, 40.0);
    const Run a = follow(route, p, 15.0, 1);
    const Run b = follow(route, p, 15.0, 10);
    CHECK(a.command_hash == b.command_hash);
    CHECK(a.world_hash == b.world_hash);
    CHECK(a.world_hash != 0);
    // And repeatable at the same worker count.
    const Run c = follow(route, p, 15.0, 1);
    CHECK(c.command_hash == a.command_hash);
}
TEST_CASE("route follower: a route that doubles back is followed in order (out-and-back)", "[route_follower]") {
    // Out along +x 100 m and back along y = 3. After the first fix the car drifts to y = 1.9,
    // nearer to the RETURN leg (1.1 m) than to the outbound one (1.9 m): a global
    // nearest-point search would jump to the return leg (s >= 103); the windowed search
    // must stay on the outbound leg.
    std::vector<rg::RoutePoint> pl;
    for (int i = 0; i <= 100; ++i) pl.push_back({static_cast<double>(i), 0.0});
    for (int i = 100; i >= 0; --i) pl.push_back({static_cast<double>(i), 3.0});
    rg::RouteFollower f(pl, rg::RouteFollowerParams{});
    rg::FollowerState st;
    st.speed = 8.0;
    st.x = 5.0 - f.params().rear_axle_x_m;
    st.y = 0.0;
    double prev_s = f.step(st, 1.0 / 240.0).s;
    CHECK(prev_s == Catch::Approx(5.0).margin(1.1));
    st.y = 1.9;
    for (int x = 10; x <= 95; x += 5) {
        st.x = static_cast<double>(x) - f.params().rear_axle_x_m;
        const rg::FollowerCommand c = f.step(st, 1.0 / 240.0);
        CHECK(c.s >= prev_s - 1e-9);
        CHECK(c.s < 100.5); // never the return leg
        prev_s = c.s;
    }
}

TEST_CASE("route follower: diagnostic dump", "[.][diag][route_follower]") {
    rg::RouteFollowerParams p;
    p.max_speed_mps = 15.0;
    p.a_lat_mps2 = 4.5;
    const std::vector<rg::RoutePoint> route = circle_route(50.0, 3.0);
    const Run run = follow(route, p, 50.0);
    for (const Sample& s : run.samples) {
        if (s.t < 8.0 || (static_cast<int>(s.t * 20.0) % 20 == 0 && std::fabs(s.t * 20.0 - std::round(s.t * 20.0)) < 1e-6))
            std::printf("t=%.2f x=%.2f y=%.2f v=%.2f tgt=%.2f ct=%.3f steer=%.3f s=%.1f\n", s.t, s.x, s.y, s.speed, s.target,
                        s.cross_track, s.steer, s.s);
    }
    const std::vector<rg::RoutePoint> sb = s_bend_route(60.0, 50.0);
    const Run r2 = follow(sb, rg::RouteFollowerParams{}, 40.0);
    for (const Sample& s : r2.samples) {
        if (std::fabs(s.t * 2.0 - std::round(s.t * 2.0)) < 1e-6)
            std::printf("S t=%.2f x=%.2f y=%.2f v=%.2f tgt=%.2f ct=%.3f steer=%.3f s=%.1f\n", s.t, s.x, s.y, s.speed, s.target,
                        s.cross_track, s.steer, s.s);
    }
}
