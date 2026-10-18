// test_abs_repro.cpp - owner report "ABS has never been observed working". Loads car_sedan and car_hyper through the SAME
// path the garage uses for a drive (rg::materialise_setup of a saved setup with the "assist_unit" option set, then
// rg::Session on that vehicle file), accelerates to 100 km/h and brakes with brake = 1.0 as an instant step (keyboard)
// and as a 0.5 s ramp, with assist.abs on / off and on dry asphalt (the flat world's untagged ground), wet asphalt
// (weather "rain", one hour soaked) and a low-grip surface (surface table "low_mu", lambda_mu 0.1). Per wheel slip ratio,
// powertrain abs_active, the assist.abs channel as the World reads it, and the stopping distance are printed (RGABS lines).
//   rg_unit_tests "[abs_repro]" -s       hidden measurement table  ([.][abs_repro][table])
//   rg_unit_tests "[abs_regression]"     the asserted cases
#include "rg/climate.h"
#include "rg/session.h"
#include "rg/vehicle_catalog.h"
#include "rg/vehicle_setup.h"
#include "rg/vehicle_load.h"
#include "rg/world_config.h"

#include "ps/io/vehicle_io.h"
#include "ps/vehicle/assist_unit_desc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

namespace {
namespace fs = std::filesystem;
const std::string kRepo = RG_SOURCE_DIR;
constexpr double kLat = 50.1109, kLon = 8.6821;

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_abs_repro"};
    Fixture() {
        std::string err;
        auto c = rg::load_vehicle_catalog(kRepo + "/data/vehicles/catalog.json", kRepo, &err);
        REQUIRE(c.has_value());
        catalog = *c;
        auto t = rg::load_setup_options(kRepo + "/data/vehicles/setup_options.json", &err);
        REQUIRE(t.has_value());
        table = *t;
        ctx.tyre_families_path = kRepo + "/data/tyres/families.json";
        ctx.work_root = root.path();
        ctx.engine_map_cache_dir = kRepo + "/out/godot_engine_cache";
        auto store = rg::EngineStore::open(kRepo + "/data/engines/engine_catalog.json", kRepo, "", &err);
        REQUIRE(store != nullptr);
        ctx.engines = std::shared_ptr<const rg::EngineStore>(std::move(store));
    }
};

struct Result {
    bool ran = false;
    bool fitted = false;       // assist_fitted_bits bit 8
    double channel_before = -1; // World::get_control("assist.abs") when the brake is pressed (-1 = channel not registered)
    double v0_kmh = 0;
    double distance_m = 0, stop_s = 0;
    double lock_s = 0;          // time any wheel is locked (slip < -0.8) while the car is above 3 m/s
    double peak_abs_slip = 0;   // max |kappa| after the first 0.6 s of braking while above 3 m/s
    double mean_abs_slip = 0;   // mean of the per-tick mean |kappa| of the four wheels, same window
    double slip_min = 0;        // most negative per-wheel slip ratio after 0.6 s (above 3 m/s)
    double active_frac = 0;     // fraction of ticks abs_active
    int toggles = 0;            // abs_active edges
    double decel_mean = 0;      // v0 / stop time
};

// surface: "dry" = flat untagged ground, "wet" = rain on asphalt, "low_mu" = surface table low_mu, "dirt" = surface table dirt
Result run(const Fixture& f, const char* car, const char* unit, const char* surface, double abs_channel, double ramp_s, bool trace = false,
           bool via_loop = false) {
    Result r;
    const rg::CatalogEntry* e = f.catalog.find(car);
    REQUIRE(e != nullptr);
    rg::VehicleSetup setup;
    setup.vehicle_id = car;
    setup.values["assist_unit"] = std::string(unit);
    rg::ScopedWorkDir dir(f.ctx.work_root, std::string("t_abs_") + car);
    const rg::MaterialisedSetup m = rg::materialise_setup(*e, f.table, f.ctx, setup, dir.path());
    INFO(m.error);
    REQUIRE(m.ok);
    r.ran = true;

    rg::SessionConfig c;
    c.vehicle_json_path = std::string(unit) == "stock" ? e->vehicle_path : m.vehicle_path;
    c.surface_table_path = kRepo + "/data/surfaces/surfaces.json";
    c.engine_map_cache_dir = f.ctx.engine_map_cache_dir;
    c.chassis_mass_kg = e->chassis.mass_kg;
    c.chassis_half_extents = ps::Vec3{e->chassis.half_extents[0], e->chassis.half_extents[1], e->chassis.half_extents[2]};
    c.chassis_z_m = e->chassis.spawn_z_m;
    c.ground_tessellated = true;
    c.ground_half_extent_m = 6000;
    c.ground_half_extent_y_m = 200;
    const std::string s = surface;
    std::string cerr;
    static const auto climate = rg::load_climate_config(kRepo + "/data/world/climate.json", &cerr);
    REQUIRE(climate.has_value());
    if (s != "dry") {
        std::string err;
        auto w = rg::make_session_weather(*climate, kLat, kLon, 0.0, {2026, 10, 14}, 12.0, 1.0, s == "wet" ? "rain" : "clear", &err);
        INFO(err);
        REQUIRE(w.has_value());
        w->ground_surface = s == "wet" ? "asphalt" : s;
        c.weather = std::make_shared<const rg::SessionWeather>(std::move(*w));
    }
    rg::Session ses(c);
    auto& w = ses.world();
    const ps::VehicleId vid = ses.vehicle_id();
    const std::size_t nw = ses.vehicle_desc().wheels.size();
    // via_loop: the path the Godot adapter takes - Session::set_control (the atomic store) + try_step (which pushes the
    // WHOLE store into the World before every tick and adopts physics writes after it); else World::set_control + step().
    const auto ctl = [&](const char* name, double v) { if (via_loop) ses.set_control(name, v); else w.set_control(name, v); };
    const auto get = [&](const char* name) { return via_loop ? ses.get_control(name) : w.get_control(name); };
    const auto tick = [&] { if (via_loop) { while (!ses.try_step()) {} } else ses.step(); };
    if (s == "wet") {
        const auto p = rg::env_clock_point({2026, 10, 14}, 13.0, kLon);
        ses.set_environment_clock(p.env_s, p.day_index, 1.0);
        for (int i = 0; i < 5; ++i) tick();
    }
    ctl("ignition", 1.0);
    ctl("assist.auto_clutch", 1.0);
    ctl("assist.auto_shift", 1.0);
    ctl("starter", 1.0);
    for (int i = 0; i < 480; ++i) {
        tick();
        if (i == 120) ctl("starter", 0.0);
    }
    // Accelerate to ~100 km/h.
    auto vx = [&] {
        const ps::Pose p = w.get_pose(ses.chassis_body());
        return p.orientation.inverse().rotate(w.get_motion(ses.chassis_body()).linear).x;
    };
    ctl("throttle", 1.0);
    for (int i = 0; i < 240 * 40 && vx() < 100.0 / 3.6; ++i) tick();
    ctl("throttle", 0.0);
    for (int i = 0; i < 24; ++i) tick(); // 0.1 s of lift so the brake press is a driver's lift-then-brake
    r.channel_before = get("assist.abs");
    if (abs_channel >= 0) {
        ctl("assist.abs", abs_channel);
        tick();
        r.channel_before = get("assist.abs");
    }
    r.fitted = (w.powertrain_state(vid).assist_fitted_bits & (1u << 8)) != 0;
    r.v0_kmh = vx() * 3.6;
    const double x0 = w.get_pose(ses.chassis_body()).position.x;
    const double v0 = vx();
    const int max_ticks = 240 * 14;
    int n_tick = 0, n_slip = 0, active_ticks = 0;
    bool prev_active = false;
    for (int k = 0; k < max_ticks; ++k) {
        const double t = (k + 1) / 240.0;
        ctl("brake", ramp_s > 0 ? std::min(1.0, t / ramp_s) : 1.0);
        tick();
        const double v = vx();
        const auto& pt = w.powertrain_state(vid);
        ++n_tick;
        if (pt.abs_active) ++active_ticks;
        if (pt.abs_active != prev_active) ++r.toggles;
        prev_active = pt.abs_active;
        double sum = 0, mn = 0;
        for (std::size_t i = 0; i < nw; ++i) {
            const double kap = w.wheel_telemetry(vid, i).slip_ratio;
            sum += std::abs(kap);
            mn = std::min(mn, kap);
            if (v > 3.0 && kap < -0.8) { /* counted once per tick below */ }
        }
        if (v > 3.0 && mn < -0.8) r.lock_s += 1.0 / 240;
        if (t > 0.6 && v > 3.0) {
            r.peak_abs_slip = std::max(r.peak_abs_slip, -mn);
            r.slip_min = std::min(r.slip_min, mn);
            r.mean_abs_slip += sum / static_cast<double>(nw);
            ++n_slip;
        }
        if (trace && k % 12 == 0) {
            std::printf("  t=%.2f v=%.2f abs=%d ref=%.2f scale=[", t, v, (int)pt.abs_active, (double)pt.abs_reference_speed_m_s);
            for (std::size_t i = 0; i < nw; ++i) std::printf("%.2f ", (double)w.powertrain_debug(vid).abs_wheel_scale[i]);
            std::printf("] kappa=[");
            for (std::size_t i = 0; i < nw; ++i) std::printf("%.2f ", (double)w.wheel_telemetry(vid, i).slip_ratio);
            std::printf("]\n");
        }
        if (v < 0.3) {
            r.stop_s = t;
            break;
        }
    }
    r.distance_m = w.get_pose(ses.chassis_body()).position.x - x0;
    if (n_slip) r.mean_abs_slip /= n_slip;
    r.active_frac = n_tick ? static_cast<double>(active_ticks) / n_tick : 0;
    if (r.stop_s <= 0) r.stop_s = n_tick / 240.0;
    r.decel_mean = v0 / r.stop_s;
    std::printf("RGABS %-9s %-22s %-6s abs_ch=%-4.1f(fit=%d) %s v0=%5.1f km/h dist=%6.1f m stop=%5.2f s a=%5.2f | lock=%.2f s slip_min=%.2f peak=%.2f mean|k|=%.3f | abs_active=%.2f toggles=%d\n",
                car, unit, surface, r.channel_before, (int)r.fitted, ramp_s > 0 ? "ramp" : "step", r.v0_kmh, r.distance_m, r.stop_s, r.decel_mean,
                r.lock_s, r.slip_min, r.peak_abs_slip, r.mean_abs_slip, r.active_frac, r.toggles);
    return r;
}
} // namespace

TEST_CASE("ABS repro (game cars): measurement table", "[.][abs_repro][table]") {
    Fixture f;
    for (const char* car : {"car_sedan", "car_hyper"}) {
        for (const char* surf : {"dry", "wet", "dirt", "low_mu"}) {
            run(f, car, "stock", surf, -1, 0);                                   // as shipped
            for (const char* unit : {"assists_street_abs_esc", "assists_racing_abs_esc"}) {
                run(f, car, unit, surf, -1, 0);                                  // ABS on (unit default), keyboard step
                run(f, car, unit, surf, -1, 0.5);                                // ABS on, ramp
                run(f, car, unit, surf, 0.0, 0);                                 // assist.abs = 0
            }
        }
    }
}

TEST_CASE("ABS repro (game cars): the Godot adapter's channel path", "[.][abs_repro][loop]") {
    Fixture f;
    for (const char* car : {"car_sedan", "car_hyper"}) {
        run(f, car, "assists_street_abs_esc", "dry", -1, 0, false, true);   // untouched assist.abs
        run(f, car, "assists_street_abs_esc", "dry", 0.0, 0, false, true);  // the driver switches it off
    }
}

TEST_CASE("ABS repro (game cars): trace", "[.][abs_repro][trace]") {
    Fixture f;
    const char* car = std::getenv("RG_ABS_CAR") ? std::getenv("RG_ABS_CAR") : "car_sedan";
    const char* surf = std::getenv("RG_ABS_SURF") ? std::getenv("RG_ABS_SURF") : "dry";
    run(f, car, "assists_street_abs_esc", surf, -1, 0, true);
}

// Does the garage's ABS choice survive the other swaps? (material: materialise + physics loader, no map generation)
TEST_CASE("ABS repro (game cars): ABS unit next to other garage swaps", "[.][abs_repro][swaps]") {
    Fixture f;
    struct Case { const char* car; const char* option; const char* value; };
    const Case cases[] = {
        {"car_sedan", "", ""},
        {"car_sedan", "gearbox_install", "auto_8at"}, {"car_sedan", "gearbox_install", "dct_7"}, {"car_sedan", "gearbox_install", "cvt_converter"},
        {"car_sedan", "engine_install", "ventura_v8_5l0_na"},
        {"car_sedan", "powertrain_install", "hybrid_battery"}, {"car_sedan", "powertrain_install", "battery_ev"},
        {"car_hyper", "", ""}, {"car_hyper", "powertrain_install", "hybrid_battery"}, {"car_hyper", "powertrain_install", "battery_ev"},
    };
    for (const Case& c : cases) {
        const rg::CatalogEntry* e = f.catalog.find(c.car);
        REQUIRE(e != nullptr);
        rg::VehicleSetup setup;
        setup.vehicle_id = c.car;
        setup.values["assist_unit"] = std::string("assists_street_abs_esc");
        if (std::string(c.option) != "") setup.values[c.option] = std::string(c.value);
        const auto v = rg::validate_setup(*e, f.table, f.ctx, setup, false);
        if (!v.ok) { std::printf("RGSWAP %-10s %-18s %-20s validate: %s\n", c.car, c.option, c.value, v.message.c_str()); continue; }
        rg::ScopedWorkDir dir(f.ctx.work_root, "t_swap");
        const rg::MaterialisedSetup m = rg::materialise_setup(*e, f.table, f.ctx, setup, dir.path());
        if (!m.ok) { std::printf("RGSWAP %-10s %-18s %-20s materialise: %s\n", c.car, c.option, c.value, m.error.c_str()); continue; }
        ps::io::EngineMapOptions options;
        options.generate = false;
        const ps::vehicle::VehicleDesc desc = rg::load_vehicle(m.vehicle_path, options);
        std::printf("RGSWAP %-10s %-18s %-20s assist_unit=%d abs=%d\n", c.car,c.option, c.value, (int)desc.assist_unit.has_value(),
                    (int)(desc.assist_unit.has_value() && desc.assist_unit->abs.has_value()));
    }
}

// Regression: a car that HAS the ABS block (the garage's "+ ABS/ESC" assist units) holds the wheels near the target slip
// (0.12) instead of locking them, through the garage materialisation path and through the Godot adapter's channel path
// (Session::set_control + try_step). Measured (physics_sim 5c24933, RelWithDebInfo):
//   car_sedan dry, 100 km/h, brake step: ABS on  37.9 m, 2.74 s, locked 0.00 s, |kappa| peak 0.13, abs_active 93 %
//                                         ABS off 44.0 m, 3.15 s, locked 2.78 s (kappa -1.00)
//   car_hyper dry via the loop path:      ABS on  32.5 m, locked 0.00 s;  assist.abs = 0: 37.2 m, locked 2.37 s
//   car_sedan wet (rain, soaked):         ABS on  59.1 m vs 69.7 m locked (a unit without ABS)
TEST_CASE("ABS regression (game cars): a fitted ABS keeps the wheels from locking, assist.abs = 0 does not", "[abs_regression][assist]") {
    Fixture f;
    const Result on = run(f, "car_sedan", "assists_street_abs_esc", "dry", -1, 0);
    const Result off = run(f, "car_sedan", "assists_street_abs_esc", "dry", 0.0, 0);
    REQUIRE(on.fitted);
    CHECK(on.channel_before == 1.0);
    CHECK(on.lock_s < 0.05);
    CHECK(on.peak_abs_slip < 0.25);
    CHECK(on.active_frac > 0.5);
    CHECK(on.toggles >= 2);
    CHECK(on.distance_m < 0.92 * off.distance_m);
    CHECK(off.channel_before == 0.0);
    CHECK(off.lock_s > 1.5);
    CHECK(off.peak_abs_slip > 0.95);
    CHECK(off.toggles == 0);
    const Result wet = run(f, "car_sedan", "assists_street_abs_esc", "wet", -1, 0);
    const Result wet_stock = run(f, "car_sedan", "assists_street", "wet", -1, 0); // a unit without ABS: what the car did before it got one
    CHECK(wet.lock_s < 0.05);
    CHECK(wet_stock.lock_s > 1.5); // locks on a keyboard brake press
    CHECK(wet.distance_m < 0.92 * wet_stock.distance_m);
    // The car as shipped (model-year rule, test_abs_fitment.cpp) carries ABS and ESC and does not lock either.
    const Result shipped = run(f, "car_sedan", "stock", "wet", -1, 0);
    REQUIRE(shipped.fitted);
    CHECK(shipped.channel_before == 1.0);
    CHECK(shipped.lock_s < 0.05);
    CHECK(shipped.distance_m < 0.92 * wet_stock.distance_m);
}

TEST_CASE("ABS regression (game cars): the Godot adapter's channel path keeps assist.abs on", "[abs_regression][assist][long]") {
    Fixture f;
    const Result on = run(f, "car_hyper", "assists_street_abs_esc", "dry", -1, 0, false, true);
    REQUIRE(on.fitted);
    CHECK(on.channel_before == 1.0);
    CHECK(on.lock_s < 0.05);
    CHECK(on.peak_abs_slip < 0.25);
    const Result off = run(f, "car_hyper", "assists_street_abs_esc", "dry", 0.0, 0, false, true);
    CHECK(off.lock_s > 1.5);
    CHECK(on.distance_m < 0.92 * off.distance_m);
}
