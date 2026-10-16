// test_fpv_drone.cpp - rg::FpvDrone (kinematic FPV flyer), its config loader and the rotor sound (tags [fpv]).
#include "rg/fpv_drone.h"
#include "rg/fpv_drone_sound.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace {

using Catch::Approx;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 1.0 / 240.0;

// A flat floor at z = 0.
const rg::FpvGroundProbe flat_ground = [](double, double, double) -> std::optional<double> { return 0.0; };
const rg::FpvGroundProbe no_ground = [](double, double, double) -> std::optional<double> { return std::nullopt; };

rg::FpvDrone flying(double z = 50.0, double yaw = 0.0) {
    rg::FpvDrone d;
    d.spawn({0.0, 0.0, z}, yaw);
    return d;
}

void run(rg::FpvDrone& d, const rg::FpvDroneInput& in, double seconds, const rg::FpvGroundProbe& g = flat_ground) {
    const int n = static_cast<int>(seconds / kDt);
    for (int i = 0; i < n; ++i) d.step(in, kDt, g);
}

} // namespace

TEST_CASE("fpv: speed never exceeds the maximum, whatever the input", "[fpv]") {
    rg::FpvDrone d = flying();
    const rg::FpvDroneConfig& c = d.config();
    rg::FpvDroneInput in;
    in.move_forward = 5.0; // out of range: clamped
    in.move_right = 5.0;
    in.move_up = 5.0;
    double max_h = 0.0, max_v = 0.0;
    for (int i = 0; i < 240 * 6; ++i) {
        const rg::FpvDroneState& s = d.step(in, kDt, flat_ground);
        max_h = std::max(max_h, std::hypot(s.velocity.x, s.velocity.y));
        max_v = std::max(max_v, std::fabs(s.velocity.z));
    }
    CHECK(max_h <= c.max_speed_mps + 1e-9);
    CHECK(max_h == Approx(c.max_speed_mps).margin(1e-6)); // and it does get there (a diagonal is not faster)
    CHECK(max_v <= c.max_vertical_speed_mps + 1e-9);
    CHECK(max_v == Approx(c.max_vertical_speed_mps).margin(1e-6));
}

TEST_CASE("fpv: acceleration and deceleration limits", "[fpv]") {
    rg::FpvDrone d = flying();
    const rg::FpvDroneConfig& c = d.config();
    rg::FpvDroneInput in;
    in.move_forward = 1.0;
    double worst_up = 0.0;
    for (int i = 0; i < 240; ++i) worst_up = std::max(worst_up, d.step(in, kDt, flat_ground).accel_mps2);
    CHECK(worst_up <= c.accel_mps2 + 1e-9);
    CHECK(worst_up == Approx(c.accel_mps2).margin(1e-6));
    // After 1 s at accel 16 m/s^2 the speed is 16 m/s (still below the 25 m/s maximum).
    CHECK(d.state().speed_mps == Approx(c.accel_mps2).margin(1e-6));

    run(d, in, 3.0); // at the maximum
    REQUIRE(d.state().speed_mps == Approx(c.max_speed_mps).margin(1e-6));
    in = rg::FpvDroneInput{}; // let go: decelerate, harder than accelerating
    double worst_down = 0.0;
    int ticks = 0;
    while (d.state().speed_mps > 1e-9 && ticks < 240 * 10) {
        worst_down = std::max(worst_down, d.step(in, kDt, flat_ground).accel_mps2);
        ++ticks;
    }
    CHECK(worst_down <= c.decel_mps2 + 1e-9);
    CHECK(worst_down == Approx(c.decel_mps2).margin(1e-6));
    CHECK(ticks * kDt == Approx(c.max_speed_mps / c.decel_mps2).margin(0.02));
    CHECK(d.state().speed_mps == 0.0);
}

TEST_CASE("fpv: movement is in the yaw frame and pitch never tilts it", "[fpv]") {
    // Yaw 90 deg = north: forward is +y, right is +x.
    rg::FpvDrone d = flying(50.0, kPi / 2);
    rg::FpvDroneInput in;
    in.move_forward = 1.0;
    run(d, in, 2.0);
    CHECK(d.state().velocity.x == Approx(0.0).margin(1e-6));
    CHECK(d.state().velocity.y == Approx(d.config().max_speed_mps).margin(1e-6));
    CHECK(d.state().velocity.z == Approx(0.0).margin(1e-9));
    d = flying(50.0, kPi / 2);
    in = {};
    in.move_right = 1.0;
    run(d, in, 2.0);
    CHECK(d.state().velocity.x == Approx(d.config().max_speed_mps).margin(1e-6));
    CHECK(std::fabs(d.state().velocity.y) < 1e-6);

    // Pitch the nose up to its limit, then fly forward: still horizontal.
    d = flying(50.0, 0.0);
    in = {};
    in.pitch = 1.0;
    run(d, in, 3.0);
    const double limit = d.config().pitch_limit_deg * kPi / 180.0;
    CHECK(d.state().pitch_rad == Approx(limit).margin(1e-9));
    in = {};
    in.move_forward = 1.0;
    in.pitch = 1.0;
    run(d, in, 2.0);
    CHECK(d.state().pitch_rad <= limit + 1e-12);
    CHECK(std::fabs(d.state().velocity.z) < 1e-9);
    CHECK(d.state().velocity.x == Approx(d.config().max_speed_mps).margin(1e-6));
}

TEST_CASE("fpv: yaw and pitch rates, limits and no roll", "[fpv]") {
    rg::FpvDrone d = flying(50.0, 0.0);
    rg::FpvDroneInput in;
    in.yaw = 1.0; // turn left
    run(d, in, 1.0);
    CHECK(d.state().yaw_rate_rad_s == Approx(d.config().yaw_rate_rad_s).margin(1e-3));
    CHECK(d.state().yaw_rad > 1.5); // ~2.2 rad/s for 1 s, minus the response ramp
    CHECK(d.state().yaw_rad < d.config().yaw_rate_rad_s + 1e-9);
    // Yaw wraps into (-pi, pi].
    run(d, in, 10.0);
    CHECK(std::fabs(d.state().yaw_rad) <= kPi + 1e-12);
    // The rate settles to zero without input.
    in = {};
    run(d, in, 2.0);
    CHECK(std::fabs(d.state().yaw_rate_rad_s) < 1e-6);
    // Pitch down to the negative limit.
    in.pitch = -1.0;
    run(d, in, 4.0);
    CHECK(d.state().pitch_rad == Approx(-d.config().pitch_limit_deg * kPi / 180.0).margin(1e-9));
    // The state has no roll field at all: the only attitude is yaw and pitch (compile-time documentation).
    static_assert(sizeof(rg::FpvDroneState::yaw_rad) == sizeof(double));
}

TEST_CASE("fpv: ground clamp and hold without ground", "[fpv]") {
    // Hills: ground z = 5 for x > 100, else 0.
    const rg::FpvGroundProbe hill = [](double x, double, double) -> std::optional<double> { return x > 100.0 ? 5.0 : 0.0; };
    rg::FpvDrone d = flying(3.0);
    const double clearance = d.config().ground_clearance_m;
    rg::FpvDroneInput in;
    in.move_up = -1.0;
    run(d, in, 3.0, hill);
    CHECK(d.state().position.z == Approx(clearance).margin(1e-9));
    CHECK(d.state().on_ground);
    CHECK(d.state().velocity.z == 0.0);
    CHECK(d.state().height_agl_m == Approx(clearance).margin(1e-9));
    // Fly into the hill side while pressing down: lifted onto it, never below.
    in = {};
    in.move_forward = 1.0;
    in.move_up = -1.0;
    double min_agl = 1e9;
    for (int i = 0; i < 240 * 10; ++i) {
        const auto& s = d.step(in, kDt, hill);
        min_agl = std::min(min_agl, s.position.z - (s.position.x > 100.0 ? 5.0 : 0.0));
    }
    CHECK(d.state().position.x > 100.0);
    CHECK(min_agl >= clearance - 1e-9);
    CHECK(d.state().position.z == Approx(5.0 + clearance).margin(1e-9));

    // No ground loaded: height held (no falling into the void, no climbing), still flies sideways.
    rg::FpvDrone h = flying(40.0);
    in = {};
    in.move_forward = 1.0;
    in.move_up = -1.0;
    run(h, in, 2.0, no_ground);
    CHECK(h.state().hold);
    CHECK(h.state().position.z == Approx(40.0));
    CHECK(h.state().position.x > 10.0);
}

TEST_CASE("fpv: fast descent does not tunnel through the ground", "[fpv]") {
    rg::FpvDrone d = flying(2.0);
    rg::FpvDroneInput in;
    in.move_up = -1.0;
    for (int i = 0; i < 240 * 3; ++i) d.step(in, kDt, flat_ground);
    CHECK(d.state().position.z >= d.config().ground_clearance_m - 1e-9);
}

TEST_CASE("fpv: stepping is deterministic", "[fpv][determinism]") {
    const auto drive = [] {
        rg::FpvDrone d = flying(30.0, 0.3);
        for (int i = 0; i < 4000; ++i) {
            rg::FpvDroneInput in;
            const double t = i * kDt;
            in.move_forward = std::sin(t * 0.7);
            in.move_right = std::cos(t * 1.1);
            in.move_up = std::sin(t * 0.3);
            in.yaw = std::sin(t * 0.5);
            in.pitch = std::cos(t * 0.9);
            d.step(in, kDt, [](double x, double y, double) -> std::optional<double> { return 3.0 * std::sin(0.05 * x) + 0.02 * y; });
        }
        return d.state();
    };
    const rg::FpvDroneState a = drive(), b = drive();
    CHECK(a.position.x == b.position.x);
    CHECK(a.position.y == b.position.y);
    CHECK(a.position.z == b.position.z);
    CHECK(a.yaw_rad == b.yaw_rad);
    CHECK(a.pitch_rad == b.pitch_rad);
    CHECK(a.activity == b.activity);
}

TEST_CASE("fpv: activity rises with acceleration and turning, spawn/despawn/halt", "[fpv]") {
    rg::FpvDrone d = flying();
    d.step({}, kDt, flat_ground);
    CHECK(d.state().activity == 0.0); // hovering: idle
    rg::FpvDroneInput in;
    in.move_forward = 1.0;
    d.step(in, kDt, flat_ground);
    CHECK(d.state().activity > 0.7); // full acceleration
    run(d, in, 4.0);
    CHECK(d.state().activity < 0.05); // cruising at the maximum speed: back to idle
    in = {};
    d.step(in, kDt, flat_ground);
    CHECK(d.state().activity > 0.9); // braking
    d.halt();
    CHECK(d.state().speed_mps == 0.0);
    CHECK(d.state().activity == 0.0);
    d.despawn();
    CHECK_FALSE(d.active());
    const ps::Vec3 before = d.state().position;
    d.step(in, kDt, flat_ground); // inactive: no-op
    CHECK(d.state().position.x == before.x);
}

TEST_CASE("fpv: shipped config files load, bad ones are refused", "[fpv]") {
    const std::string path = std::string(RG_SOURCE_DIR) + "/data/controls/fpv_drone.json";
    std::string err;
    const auto c = rg::load_fpv_drone_config(path, &err);
    REQUIRE(c);
    CHECK(c->max_speed_mps == Approx(25.0));
    const auto a = rg::load_fpv_audio_config(path, &err);
    REQUIRE(a);
    CHECK(a->idle_hz < a->max_hz);
    CHECK_FALSE(rg::load_fpv_drone_config(path + ".missing", &err));
    CHECK(err.find("cannot read") != std::string::npos);
}

TEST_CASE("fpv sound: idle hum, pitch and level rise with activity", "[fpv][sound]") {
    rg::FpvAudioConfig cfg;
    rg::FpvRotorVoice v(cfg, 7);
    const auto rms = [](const std::vector<float>& s) {
        double e = 0.0;
        for (float x : s) e += static_cast<double>(x) * x;
        return std::sqrt(e / static_cast<double>(s.size()));
    };
    const auto idle = v.render(48000, 1.0f);
    REQUIRE(idle.size() == 48000);
    CHECK(rms(idle) > 0.01); // a hover is audible
    CHECK(v.frequency_hz() == Approx(cfg.idle_hz).margin(1e-6));
    v.set_activity(1.0);
    v.render(48000, 1.0f); // let the rotors spin up
    const auto busy = v.render(48000, 1.0f);
    CHECK(v.frequency_hz() > 0.95 * cfg.max_hz);
    CHECK(rms(busy) > 1.5 * rms(idle));
    for (float x : busy) REQUIRE(std::fabs(x) <= 1.0f);
    // Back to idle.
    v.set_activity(0.0);
    v.render(48000 * 3, 1.0f);
    CHECK(v.frequency_hz() < cfg.idle_hz * 1.05);
    // Deterministic for a seed; gain scales.
    rg::FpvRotorVoice a(cfg, 3), b(cfg, 3);
    const auto sa = a.render(1000, 1.0f), sb = b.render(1000, 1.0f), sc = b.render(1000, 0.0f);
    CHECK(sa == sb);
    bool silent = true;
    for (float x : sc) silent = silent && x == 0.0f;
    CHECK(silent);
}
