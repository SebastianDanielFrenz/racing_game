// test_game_car_thermal.cpp - every shipped game car (data/vehicles/car_*.json) has the physics engine thermal model, so
// the coolant/oil gauge on the HUD and the hypercar cockpit panel (rg::engine_thermal_hud) shows real values.
// Owner decision 2026-10-12. The fast case loads each vehicle file with the physics loader in validate-only mode (no
// engine-map generation, milliseconds) and checks the desc the dashboard reads (rg::engine_thermal_info); the engine swap
// case checks that a car loses its cooling_system when the new engine has no thermal data (the loader would throw).
//
// Hidden diagnostics [.][diag][thermal_run] (minutes, not registered as normal tests) measure the hypercars' coolant and
// oil temperature over time: RG_DIAG_CAR (default car_hyper), RG_DIAG_AMBIENT_C (default 15 = ISA), RG_DIAG_MODE =
// "wot" (full throttle until the speed plateau, then RG_DIAG_HOLD_S more seconds, default 90) or "cruise"
// (RG_DIAG_CRUISE_KMH, default 130, held RG_DIAG_HOLD_S seconds, default 600). Output: one DIAG line per 5 s.
#include "rg/engine_swap.h"
#include "rg/engine_thermal_info.h"
#include "rg/session.h"
#include "rg/vehicle_catalog.h"

#include "ps/io/vehicle_io.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
const std::string kRepo = RG_SOURCE_DIR;

std::string env_or(const char* name, const std::string& fallback) {
#ifdef _WIN32
    char* buffer = nullptr;
    std::size_t length = 0;
    if (_dupenv_s(&buffer, &length, name) != 0 || buffer == nullptr) return fallback;
    const std::string value(buffer);
    std::free(buffer);
    return value.empty() ? fallback : value;
#else
    const char* v = std::getenv(name);
    return (v != nullptr && *v != '\0') ? std::string(v) : fallback;
#endif
}

} // namespace

TEST_CASE("game cars: every shipped car has the engine thermal model with sane thresholds", "[thermal][vehicle_data]") {
    std::vector<std::string> files;
    for (const auto& e : fs::directory_iterator(kRepo + "/data/vehicles")) {
        const std::string name = e.path().filename().string();
        if (name.rfind("car_", 0) == 0 && e.path().extension() == ".json") files.push_back(e.path().string());
    }
    std::sort(files.begin(), files.end());
    CHECK(files.size() >= 6); // car_hyper, car_hyper_n2o, car_sedan, car_sedan_awd, car_sedan_gen_n2o, car_sedan_rwd
    for (const std::string& path : files) {
        INFO(path);
        ps::io::EngineMapOptions options;
        options.generate = false; // validate-only: the thermal block is resolved, no engine maps are generated
        const ps::vehicle::VehicleDesc desc = ps::io::load_vehicle_json(path, options);
        const rg::EngineThermalInfo info = rg::engine_thermal_info(desc);
        REQUIRE(info.present);
        REQUIRE(desc.powertrain.thermal.has_value());
        CHECK(desc.powertrain.thermal->start == ps::drivetrain::ThermalStartKind::Warm);
        // The colour levels come from the car's data: cold < hot (fan on) < overheat, all in a physical range.
        CHECK(info.cold_c > 20.0);
        CHECK(info.hot_c > info.cold_c);
        CHECK(info.overheat_c > info.hot_c);
        CHECK(info.overheat_c < 140.0);
        CHECK(info.fan_off_c < info.fan_on_c);
    }
}

TEST_CASE("engine swap: a car loses its cooling system when the new engine has no thermal data", "[thermal][engine_swap]") {
    rg::SwapCar car;
    car.has_cooling_system = true;
    rg::SwapEngine engine;
    engine.name = "No thermal";
    engine.idle_rpm = 800;
    engine.limiter_rpm = 6500;
    engine.peak_torque_nm = 200;
    engine.fuels = {"e5"};

    engine.has_thermal = false;
    rg::SwapPackage p = rg::derive_swap_package(car, engine, {}, true);
    CHECK(p.drop_cooling_system);
    const bool noted = std::any_of(p.lines.begin(), p.lines.end(), [](const std::string& l) { return l.find("temperature gauge") != std::string::npos; });
    CHECK(noted);

    engine.has_thermal = true; // thermal data on the new engine: the cooling system stays
    CHECK_FALSE(rg::derive_swap_package(car, engine, {}, true).drop_cooling_system);

    engine.has_thermal = false; // no swap (only a fuel choice): nothing is dropped
    CHECK_FALSE(rg::derive_swap_package(car, engine, {}, false).drop_cooling_system);

    car.has_cooling_system = false; // a car without a thermal model has nothing to drop
    CHECK_FALSE(rg::derive_swap_package(car, engine, {}, true).drop_cooling_system);
}

TEST_CASE("diag: hypercar engine temperature over a full-throttle run or a cruise", "[.][diag][thermal_run]") {
    const std::string car = env_or("RG_DIAG_CAR", "car_hyper");
    const std::string mode = env_or("RG_DIAG_MODE", "wot");
    const double ambient_c = std::atof(env_or("RG_DIAG_AMBIENT_C", "15").c_str());
    const double cruise_kmh = std::atof(env_or("RG_DIAG_CRUISE_KMH", "130").c_str());
    const double hold_s = std::atof(env_or("RG_DIAG_HOLD_S", mode == "wot" ? "90" : "600").c_str());
    const bool arm = env_or("RG_DIAG_NITROUS", "0") == "1";

    std::string err;
    auto catalog = rg::load_vehicle_catalog(kRepo + "/data/vehicles/catalog.json", kRepo, &err);
    REQUIRE(catalog.has_value());
    const rg::CatalogEntry* entry = catalog->find(car);
    REQUIRE(entry != nullptr);

    rg::SessionConfig config;
    config.vehicle_json_path = entry->vehicle_path;
    config.surface_table_path = kRepo + "/data/surfaces/surfaces.json";
    config.chassis_mass_kg = entry->chassis.mass_kg;
    config.chassis_half_extents = ps::Vec3{entry->chassis.half_extents[0], entry->chassis.half_extents[1], entry->chassis.half_extents[2]};
    config.chassis_z_m = entry->chassis.spawn_z_m;
    config.ground_half_extent_m = 20000.0; // 40 km tessellated strip (PHYS-047), as test_hyper_rear_travel
    config.ground_half_extent_y_m = 500.0;
    config.ground_tessellated = true;
    config.engine_map_cache_dir = kRepo + "/out/godot_engine_cache";
    rg::Session session(config);
    auto& world = session.world();
    const auto vid = session.vehicle_id();
    const rg::EngineThermalInfo info = rg::engine_thermal_info(session.vehicle_desc());
    REQUIRE(info.present);
    world.set_ambient(ps::AmbientSample{101325.0, 273.15 + ambient_c, 0.0});
    std::printf("DIAG car=%s mode=%s ambient=%.1f C hot=%.0f overheat=%.0f\n", car.c_str(), mode.c_str(), ambient_c, info.hot_c, info.overheat_c);

    for (int i = 0; i < 480; ++i) session.step();
    world.set_control("ignition", 1.0);
    world.set_control("starter", 1.0);
    world.set_control("assist.auto_shift", 1.0);
    world.set_control("assist.auto_clutch", 1.0);
    world.set_control("assist.auto_blip", 1.0);
    world.set_control("clutch", 0.0);
    world.set_control("brake", 0.0);
    world.set_control("handbrake", 0.0);
    if (arm) world.set_control("nitrous_arm", 1.0);

    const double dt = 1.0 / 240.0;
    double t = 0.0, peak_coolant = 0.0, peak_oil = 0.0, t_overheat = -1.0, plateau_kmh = 0.0, plateau_t = 0.0;
    double hold_start = -1.0, hold_peak_coolant = 0.0, hold_peak_oil = 0.0;
    bool started = false;
    double throttle_i = 0.0;
    const long max_steps = 240L * 900L;
    std::printf("DIAG t_s   kmh  gear  rpm | coolant oil  fan thermo | rad_kW wall_kW fric_kW\n");
    for (long step = 0; step < max_steps; ++step) {
        if (!started && world.powertrain_state(vid).engine_state == ps::drivetrain::EngineState::Running) {
            world.set_control("starter", 0.0);
            started = true;
        }
        if (!started && step > 240 * 6) world.set_control("starter", 0.0), started = true;
        const ps::Motion mo = world.get_motion(session.chassis_body());
        const double kmh = std::sqrt(mo.linear.x * mo.linear.x + mo.linear.y * mo.linear.y) * 3.6;
        if (started) {
            if (mode == "wot") {
                world.set_control("throttle", 1.0);
            } else { // PI cruise on the throttle, launch at 0.6 until 30 km/h
                const double e = (cruise_kmh - kmh) / 20.0;
                throttle_i = std::clamp(throttle_i + e * dt * 0.5, 0.0, 1.0);
                const double thr = kmh < 30.0 ? 0.6 : std::clamp(throttle_i + e, 0.0, 1.0);
                world.set_control("throttle", thr);
            }
        }
        session.step();
        t += dt;
        const auto pt = world.powertrain_state(vid);
        const rg::EngineThermalHud h = rg::engine_thermal_hud(info, pt);
        if (h.present && started) {
            peak_coolant = std::max(peak_coolant, h.coolant_c);
            peak_oil = std::max(peak_oil, h.oil_c);
            if (h.level == rg::ThermalLevel::Overheat && t_overheat < 0.0) t_overheat = t;
        }
        if ((step % (240 * 5)) == 0 && h.present) {
            std::printf("DIAG %5.0f %5.0f %3d %5.0f | %6.1f %6.1f %d %.2f | %6.0f %6.0f %6.0f\n", t, kmh, static_cast<int>(pt.gear), pt.engine_rpm,
                        h.coolant_c, h.oil_c, h.fan_on ? 1 : 0, h.thermostat_opening, pt.thermal.radiator_heat_w / 1000.0,
                        pt.thermal.wall_heat_w / 1000.0, pt.thermal.friction_heat_w / 1000.0);
        }
        if (mode == "wot") {
            if (hold_start < 0.0 && kmh > 200.0 && t - plateau_t >= 10.0) {
                if (kmh - plateau_kmh < 0.5) {
                    hold_start = t;
                    std::printf("DIAG top speed plateau %.1f km/h at t=%.1f s; holding %.0f s\n", kmh, t, hold_s);
                }
                plateau_kmh = kmh;
                plateau_t = t;
            } else if (hold_start < 0.0 && kmh <= 200.0 && t - plateau_t >= 10.0) {
                plateau_kmh = kmh;
                plateau_t = t;
            }
            if (hold_start >= 0.0) {
                hold_peak_coolant = std::max(hold_peak_coolant, h.coolant_c);
                hold_peak_oil = std::max(hold_peak_oil, h.oil_c);
                if (t - hold_start >= hold_s) break;
            }
            if (world.get_pose(session.chassis_body()).position.x > 19000.0) { std::printf("DIAG ran out of ground\n"); break; }
        } else if (t > 20.0 + hold_s + 120.0) {
            break;
        }
    }
    std::printf("DIAG finished t=%.1f s; peak coolant %.1f C, peak oil %.1f C, first overheat at %.1f s; during the hold: coolant %.1f oil %.1f\n",
                t, peak_coolant, peak_oil, t_overheat, hold_peak_coolant, hold_peak_oil);
    SUCCEED();
}
