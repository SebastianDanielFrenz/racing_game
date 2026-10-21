// test_tc_owner_repro.cpp - the owner's PR #5 report "TC is still broken (low speed / standstill, and not enough
// wheelspin prevention at ~180 km/h)". The game's car_hyper (catalog entry, rg::Session on flat tessellated ground,
// auto-clutch + auto-shift on) through full/part throttle launches and launch-control launches per drive mode and a WOT
// pull through 150-210 km/h, TC default vs off. True slip = (faster rear peripheral speed - chassis vx at the contact
// height) / max(2, |vx|). The [tc_repro] cases are hidden diagnostics: rg_unit_tests "[tc_repro]" -s (RG_TC_REPRO_TRACE=1
// for a per-0.05 s trace); the last case is the regression test with the bounds.
#include "rg/session.h"
#include "rg/vehicle_catalog.h"

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>

namespace {
const std::string kRepo = RG_SOURCE_DIR;

struct Stats {
    double t30 = -1, t60 = -1, t100 = -1, v_end = 0, slip_peak = 0, slip_mean = 0, frac_min = 1, cut = 0, crawl_cut = 0;
    int n = 0;
    double band_peak = 0, band_mean = 0, band_over = 0, band_frac_min = 1;
    double lc_peak = 0, post_peak = 0, over30 = 0; // slip peak while launch control owns the car / after; time above 0.3
    int band_n = 0, g_lo = 99, g_hi = -99;
};

Stats run(const char* name, int drive_mode, double tc, double seconds, const std::function<double(double, double)>& thr_fn, double lc_hold_s = 0) {
    std::string err;
    const auto catalog = rg::load_vehicle_catalog(kRepo + "/data/vehicles/catalog.json", kRepo, &err);
    REQUIRE(catalog.has_value());
    const rg::CatalogEntry* e = catalog->find("car_hyper");
    REQUIRE(e != nullptr);
    rg::SessionConfig c;
    c.vehicle_json_path = e->vehicle_path;
    c.surface_table_path = kRepo + "/data/surfaces/surfaces.json";
    c.chassis_mass_kg = e->chassis.mass_kg;
    c.chassis_half_extents = ps::Vec3{e->chassis.half_extents[0], e->chassis.half_extents[1], e->chassis.half_extents[2]};
    c.chassis_z_m = e->chassis.spawn_z_m;
    c.ground_tessellated = true;
    c.ground_half_extent_m = 6000;
    c.ground_half_extent_y_m = 200;
    rg::Session s(c);
    auto& w = s.world();
    const auto& desc = s.vehicle_desc();
    const ps::VehicleId vid = s.vehicle_id();
    w.set_control("ignition", 1.0);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_shift", 1.0);
    w.set_control("handbrake", 1.0);
    for (int i = 0; i < 96; ++i) s.step();
    w.set_control("assist.drive_mode", drive_mode); // a CHANGE applies the mode's TC default
    for (int i = 0; i < 24; ++i) s.step();
    if (tc >= 0) w.set_control("assist.traction_control", tc);
    w.set_control("handbrake", 0.0);
    if (lc_hold_s > 0) { // launch control: brake + full throttle, then release the brake
        w.set_control("brake", 1.0);
        w.set_control("throttle", 1.0);
        for (int i = 0; i < static_cast<int>(lc_hold_s * 240); ++i) s.step();
        std::printf("  %s LC hold: phase=%d rpm=%.0f boost=%.2f\n", name, (int)w.powertrain_state(s.vehicle_id()).launch_phase,
                    (double)w.powertrain_state(s.vehicle_id()).engine_rpm,
                    w.powertrain_state(s.vehicle_id()).engines.empty() ? 0.0 : (double)w.powertrain_state(s.vehicle_id()).engines[0].boost_bar);
        w.set_control("brake", 0.0);
    }
    const double cz = desc.wheels[0].attachment_local.z - desc.wheels[0].wheel_radius;
    const bool trace = std::getenv("RG_TC_REPRO_TRACE") != nullptr;
    Stats r;
    int tcm = -1;
    auto vx_of = [&] {
        const ps::Pose p = w.get_pose(s.chassis_body());
        return p.orientation.inverse().rotate(w.get_motion(s.chassis_body()).linear).x;
    };
    const int ticks = static_cast<int>(seconds * 240);
    for (int k = 0; k < ticks; ++k) {
        const double t = (k + 1) / 240.0;
        const double thr = thr_fn(t, vx_of());
        w.set_control("throttle", thr);
        s.step();
        const auto pt = w.powertrain_state(vid);
        tcm = pt.traction_mode_effective;
        const ps::Pose p = w.get_pose(s.chassis_body());
        const ps::Vec3 om = p.orientation.inverse().rotate(w.get_motion(s.chassis_body()).angular);
        const double vx = vx_of() + om.y * cz;
        const double rl = w.wheel_omega(vid, 2) * desc.wheels[2].wheel_radius, rr = w.wheel_omega(vid, 3) * desc.wheels[3].wheel_radius;
        const double slip = (std::max(rl, rr) - vx) / std::max(2.0, std::abs(vx));
        const double frac = pt.traction_torque_fraction;
        if (r.t30 < 0 && vx >= 30 / 3.6) r.t30 = t;
        if (r.t60 < 0 && vx >= 60 / 3.6) r.t60 = t;
        if (r.t100 < 0 && vx >= 100 / 3.6) r.t100 = t;
        if (t >= 0.3 && thr > 0.05) {
            r.slip_peak = std::max(r.slip_peak, slip); r.slip_mean += slip; r.frac_min = std::min(r.frac_min, frac);
            if (frac < 0.5) r.cut += 1.0 / 240;
            if (pt.launch_phase != 0) r.lc_peak = std::max(r.lc_peak, slip); else r.post_peak = std::max(r.post_peak, slip);
            if (slip > 0.3) r.over30 += 1.0 / 240;
            ++r.n;
        }
        if (thr > 0.05 && vx < 2 && frac < 0.2) r.crawl_cut += 1.0 / 240;
        if (thr > 0.95 && vx >= 150 / 3.6 && vx <= 210 / 3.6) {
            r.band_peak = std::max(r.band_peak, slip); r.band_mean += slip; ++r.band_n;
            r.band_frac_min = std::min(r.band_frac_min, frac);
            if (slip > 0.15) r.band_over += 1.0 / 240;
            r.g_lo = std::min(r.g_lo, static_cast<int>(pt.gear)); r.g_hi = std::max(r.g_hi, static_cast<int>(pt.gear));
        }
        if (trace && k % 12 == 0)
            std::printf("  %s t=%.2f vx=%.2f rl=%.2f rr=%.2f est=%.2f tcslip=%.3f slip=%.3f frac=%.3f g=%d rpm=%.0f cl=%.2f boost=%.2f lc=%d\n",
                        name, t, vx, rl, rr, (double)pt.traction_estimated_speed_m_s, (double)pt.traction_positive_slip, slip, frac,
                        (int)pt.gear, (double)pt.engine_rpm, (double)pt.clutch_engagement,
                        pt.engines.empty() ? 0.0 : (double)pt.engines[0].boost_bar, (int)pt.launch_phase);
        r.v_end = vx;
    }
    if (r.n) r.slip_mean /= r.n;
    if (r.band_n) {
        r.band_mean /= r.band_n;
        std::printf("RGBAND  %-24s tc=%d 150-210 km/h gears %d-%d: slip peak=%.3f mean=%.3f t(slip>0.15)=%.2fs frac min=%.3f\n", name, tcm,
                    r.g_lo, r.g_hi, r.band_peak, r.band_mean, r.band_over, r.band_frac_min);
    }
    std::printf("RGREPRO %-24s tc=%d t30=%.2f t60=%.2f t100=%.2f v_end=%.1f km/h | slip peak=%.3f mean=%.3f | frac min=%.3f cut<0.5=%.2fs | crawl_cut=%.2fs | lc_peak=%.3f post_peak=%.3f t(slip>0.3)=%.2fs\n",
                name, tcm, r.t30, r.t60, r.t100, r.v_end * 3.6, r.slip_peak, r.slip_mean, r.frac_min, r.cut, r.crawl_cut, r.lc_peak, r.post_peak, r.over30);
    return r;
}
} // namespace

TEST_CASE("TC owner repro (game car): launches", "[.][tc_repro][launch]") {
    const char* modes[] = {"eco", "normal", "sport", "racing"};
    for (int m = 0; m < 4; ++m) {
        char n[64];
        std::snprintf(n, sizeof n, "full_%s_default", modes[m]); run(n, m, -1, 5, [](double, double) { return 1.0; });
        std::snprintf(n, sizeof n, "full_%s_tcoff", modes[m]); run(n, m, 0, 5, [](double, double) { return 1.0; });
        std::snprintf(n, sizeof n, "part30_%s_default", modes[m]); run(n, m, -1, 5, [](double, double) { return 0.3; });
        std::snprintf(n, sizeof n, "crawl15_%s_default", modes[m]); run(n, m, -1, 6, [](double, double) { return 0.15; });
    }
}

TEST_CASE("TC owner repro (game car): launch control", "[.][tc_repro][lc]") {
    const char* modes[] = {"eco", "normal", "sport", "racing"};
    for (int m = 0; m < 4; ++m) {
        char n[64];
        std::snprintf(n, sizeof n, "lc_%s_default", modes[m]); run(n, m, -1, 5, [](double, double) { return 1.0; }, 1.5);
    }
}

TEST_CASE("TC owner repro (game car): WOT pull through 180 km/h", "[.][tc_repro][pull]") {
    const char* modes[] = {"eco", "normal", "sport", "racing"};
    for (int m = 0; m < 4; ++m) {
        char n[64];
        std::snprintf(n, sizeof n, "pull_%s_default", modes[m]); run(n, m, -1, 14, [](double, double) { return 1.0; });
        std::snprintf(n, sizeof n, "pull_%s_tcoff", modes[m]); run(n, m, 0, 14, [](double, double) { return 1.0; });
    }
}

// Regression (not hidden) for the PR #5 report, physics_sim d11626d (closed-loop launch-control hand-over; the Racing
// assist unit's TC Sport retards the spark at the current boost, Sport/Relaxed gains up). Measured on this setup
// (before = physics_sim 4fe9a1f):
//   Racing drive mode (TC Sport, target 0.18), launch control: slip peak 2.149 -> 0.403 (a 0.01 s spike at the
//   release), after the hand-over 0.242; 0-100 km/h 4.71 -> 4.04 s.
//   Racing drive mode, WOT pull, 150-210 km/h: peak 0.340 -> 0.199, mean 0.096.
//   Normal drive mode (TC On, target 0.08): pull band peak 0.116, standstill WOT launch peak 0.166 (unchanged).
TEST_CASE("TC regression (game car): standstill launches and the 180 km/h pull keep the wheelspin bounded", "[tc_regression][assist][long]") {
    const auto wot = [](double, double) { return 1.0; };
    const Stats lc_racing = run("reg_lc_racing", 3, -1, 5, wot, 1.5);
    CHECK(lc_racing.slip_peak < 0.6);
    CHECK(lc_racing.post_peak < 0.3);
    CHECK(lc_racing.over30 < 0.1);
    CHECK(lc_racing.t100 > 0);
    CHECK(lc_racing.t100 < 4.5);
    const Stats launch_normal = run("reg_launch_normal", 1, -1, 5, wot);
    CHECK(launch_normal.slip_peak < 0.25);
    CHECK(launch_normal.crawl_cut == 0.0);
    CHECK(launch_normal.t100 > 0);
    CHECK(launch_normal.t100 < 5.0);
    const Stats pull_racing = run("reg_pull_racing", 3, -1, 14, wot);
    REQUIRE(pull_racing.band_n > 0);
    CHECK(pull_racing.band_peak < 0.25);
    CHECK(pull_racing.band_mean < 0.18);
    const Stats pull_normal = run("reg_pull_normal", 1, -1, 14, wot);
    REQUIRE(pull_normal.band_n > 0);
    CHECK(pull_normal.band_peak < 0.15);
}
