// test_hyper_power_jumps.cpp - hidden diagnostics [.][diag][power_jumps]: owner complaint 2026-10-13 "the engine power
// constantly jumps around" while driving car_hyper_n2o with the saved garage setup (8.6 x boost target = 15.5 bar
// gauge, hyper_twin_large_n2o, final drive x0.8, gears [1,1,1,1,0.96,0.93,0.8], clutch x8, track tyres).
//
// Case 1 "diag: hypercar power trace": builds the car (RG_DIAG_CAR, default car_hyper_n2o) with a saved garage setup
// (RG_DIAG_SETUP = a racing_game.vehicle_setup/1 file, default the owner's saved one under %APPDATA%; "none" = stock
// car) on a flat tessellated Session strip, starts the engine and drives with assist.auto_shift on. Throttle profile
// RG_DIAG_PROFILE: "wot" (default; RG_DIAG_LAUNCH_THROTTLE until RG_DIAG_LAUNCH_KMH, default 0.6 / 40), "hold" (WOT up
// to RG_DIAG_HOLD_KMH, then a PI speed hold on the pedal), "const" (RG_DIAG_THROTTLE all the time). Other switches:
// RG_DIAG_SECONDS (default 90), RG_DIAG_TC=0 (traction control off), RG_DIAG_NITROUS=1 (arm), RG_DIAG_DRIVE_MODE
// (0 eco / 1 normal / 2 sport), RG_DIAG_CSV (per-tick CSV, every tick at 240 Hz).
// Two power signals: eng_kw = the tick-mean engine power from the energy ledger (engine_out difference / dt, the
// physical crank work), hud_kw = what the HUD / digital cluster shows as "DRIVE kW" (godot_ext rg_simulation.cpp
// get_vehicle_powertrain: -driveline.clutch_torque * engine omega, i.e. the clutch friction row's multiplier of the
// tick's LAST substep times omega).
//
// Case 2 "diag: hypercar engine dyno limits": run_dyno_sweep + limit findings on the engine the setup materialises to.
//
// Not registered as a normal test: it runs minutes, depends on a user file and only measures.
#include "rg/session.h"
#include "rg/vehicle_catalog.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/limit_findings.h"
#include "ps/jobs/job_system.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
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

struct Built {
    rg::CatalogEntry entry;
    std::string vehicle_path;
    std::string cache_dir;
};

// Materialises the setup (or the stock car) into `work`; returns the vehicle path to load.
Built build_car(const std::string& car, const std::string& setup_path, rg::ScopedWorkDir& root,
                std::unique_ptr<rg::ScopedWorkDir>& work) {
    std::string err;
    auto catalog = rg::load_vehicle_catalog(kRepo + "/data/vehicles/catalog.json", kRepo, &err);
    REQUIRE(catalog.has_value());
    auto table = rg::load_setup_options(kRepo + "/data/vehicles/setup_options.json", &err);
    REQUIRE(table.has_value());
    const rg::CatalogEntry* found = catalog->find(car);
    REQUIRE(found != nullptr);
    Built b;
    b.entry = *found;
    b.vehicle_path = b.entry.vehicle_path;
    b.cache_dir = kRepo + "/out/godot_engine_cache";
    rg::SetupContext ctx;
    ctx.tyre_families_path = kRepo + "/data/tyres/families.json";
    ctx.work_root = root.path();
    ctx.engine_map_cache_dir = b.cache_dir;
    auto store = rg::EngineStore::open(kRepo + "/data/engines/engine_catalog.json", kRepo, "", &err);
    REQUIRE(store != nullptr);
    ctx.engines = std::shared_ptr<const rg::EngineStore>(std::move(store));
    work = std::make_unique<rg::ScopedWorkDir>(ctx.work_root, "setup_" + car);
    if (setup_path != "none") {
        std::ifstream in(setup_path);
        REQUIRE(in.good());
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        auto setup = rg::parse_setup(text, *table, setup_path, &err);
        INFO(err);
        REQUIRE(setup.has_value());
        const rg::MaterialisedSetup m = rg::materialise_setup(b.entry, *table, ctx, *setup, work->path());
        INFO(m.error);
        REQUIRE(m.ok);
        b.vehicle_path = m.vehicle_path;
    }
    return b;
}

std::string default_setup(const std::string& car) {
    return env_or("RG_DIAG_SETUP", env_or("APPDATA", "") + "/Godot/app_userdata/racing_game R0/garage/setups/" + car + ".json");
}
} // namespace

TEST_CASE("diag: hypercar power trace", "[.][diag][power_jumps]") {
    const std::string car = env_or("RG_DIAG_CAR", "car_hyper_n2o");
    const std::string setup_path = default_setup(car);
    rg::ScopedWorkDir root(fs::temp_directory_path().generic_string(), "rg_power_jumps");
    std::unique_ptr<rg::ScopedWorkDir> work;
    const Built built = build_car(car, setup_path, root, work);
    std::printf("DIAG car=%s setup=%s vehicle=%s\n", car.c_str(), setup_path.c_str(), built.vehicle_path.c_str());

    rg::SessionConfig config;
    config.vehicle_json_path = built.vehicle_path;
    config.surface_table_path = kRepo + "/data/surfaces/surfaces.json";
    config.chassis_mass_kg = built.entry.chassis.mass_kg;
    config.chassis_half_extents = ps::Vec3{built.entry.chassis.half_extents[0], built.entry.chassis.half_extents[1],
                                           built.entry.chassis.half_extents[2]};
    config.chassis_z_m = built.entry.chassis.spawn_z_m;
    config.ground_half_extent_m = 20000.0;
    config.ground_half_extent_y_m = 500.0;
    config.ground_tessellated = true;
    config.engine_map_cache_dir = built.cache_dir;
    rg::Session session(config);
    auto& world = session.world();
    const auto vid = session.vehicle_id();

    for (int i = 0; i < 480; ++i) session.step();

    const std::string profile = env_or("RG_DIAG_PROFILE", "wot");
    const double seconds = std::atof(env_or("RG_DIAG_SECONDS", "90").c_str());
    const double launch_throttle = std::atof(env_or("RG_DIAG_LAUNCH_THROTTLE", "0.6").c_str());
    const double launch_kmh = std::atof(env_or("RG_DIAG_LAUNCH_KMH", "40").c_str());
    const double hold_kmh = std::atof(env_or("RG_DIAG_HOLD_KMH", "540").c_str());
    const double const_throttle = std::atof(env_or("RG_DIAG_THROTTLE", "0.3").c_str());
    // "hold": after RG_DIAG_FREEZE_S seconds of PI hold the pedal is frozen (open loop) - separates a powertrain limit
    // cycle from one the speed-hold loop sustains. 0 = never freeze.
    const double freeze_s = std::atof(env_or("RG_DIAG_FREEZE_S", "0").c_str());
    double hold_t = 0.0;
    bool frozen = false;

    world.set_control("ignition", 1.0);
    world.set_control("starter", 1.0);
    world.set_control("assist.auto_shift", 1.0);
    world.set_control("assist.auto_clutch", 1.0);
    world.set_control("assist.auto_blip", 1.0);
    world.set_control("clutch", 0.0);
    world.set_control("brake", 0.0);
    world.set_control("handbrake", 0.0);
    const std::string mode = env_or("RG_DIAG_DRIVE_MODE", "");
    if (!mode.empty()) world.set_control("assist.drive_mode", std::atof(mode.c_str()));
    if (env_or("RG_DIAG_NITROUS", "0") == "1") world.set_control("nitrous_arm", 1.0);
    if (env_or("RG_DIAG_TC", "1") == "0") world.set_control("assist.traction_control", 0.0);
    std::printf("DIAG profile=%s TC channel %.0f drive_mode channel %.0f aero channel %.0f\n", profile.c_str(),
                world.get_control("assist.traction_control"), world.get_control("assist.drive_mode"),
                world.get_control("assist.aero_control"));

    std::ofstream csv;
    const std::string csv_path = env_or("RG_DIAG_CSV", "");
    if (!csv_path.empty()) {
        csv.open(csv_path);
        csv << "t,kmh,gear,gear_target,shift_phase,rpm,pedal,throttle_cmd,throttle_pos,theta_c,throttle_eff,throttle_ceiling,"
               "ignition_cut,torque_demand,eng_kw,eng_nm,hud_kw,clutch_torque,clutch_slip,clutch_stuck,clutch_eng,"
               "clutch_cap,boost_bar,charge_boost_bar,p_intake_kpa,wastegate,turbo_rpm,comp_flow,comp_pr,surge,surge_events,"
               "bov,bov_events,limiter,dfco,launch,tc_active,tc_mode,tc_frac,tc_pos_slip,tc_via_clutch,n2o_sol,n2o_frac,"
               "n2o_gs,drive_mode,aero_active,aero_target_deg,downforce_n,slip_rl,slip_rr,fx_rl,fx_rr,fz_rl,fz_rr,spark_deg,"
               "lambda,fuel_gs,comp_stage,turb_stage,stage_fallback,p_exhaust_kpa,turbine_inlet_k\n";
    }

    const double dt = 1.0 / 240.0;
    double t = 0.0, integ = 0.0, pedal = 0.0;
    bool started = false, holding = false;
    double prev_engine_out = world.powertrain_energy(vid).engine_out;
    const long max_steps = static_cast<long>(seconds * 240.0);
    for (long step = 0; step < max_steps; ++step) {
        const ps::Motion mo0 = world.get_motion(session.chassis_body());
        const double kmh0 = std::sqrt(mo0.linear.x * mo0.linear.x + mo0.linear.y * mo0.linear.y) * 3.6;
        if (!started && (world.powertrain_state(vid).engine_state == ps::drivetrain::EngineState::Running || step > 240 * 6)) {
            world.set_control("starter", 0.0);
            started = true;
        }
        if (started) {
            if (profile == "const") {
                pedal = const_throttle;
            } else if (profile == "hold" && (holding || kmh0 >= hold_kmh)) {
                if (!holding) { holding = true; integ = 0.6; }
                hold_t += dt;
                if (freeze_s > 0.0 && hold_t >= freeze_s && !frozen) {
                    frozen = true;
                    std::printf("DIAG pedal frozen at %.4f (t=%.1f)\n", pedal, t);
                }
                if (!frozen) {
                    const double e = (hold_kmh - kmh0) / 3.6;
                    integ = std::clamp(integ + 0.05 * e * dt, 0.0, 1.0);
                    pedal = std::clamp(0.08 * e + integ, 0.0, 1.0);
                }
            } else {
                pedal = kmh0 < launch_kmh ? launch_throttle : 1.0;
            }
            world.set_control("throttle", pedal);
        }
        session.step();
        t += dt;
        const ps::Motion mo = world.get_motion(session.chassis_body());
        const double kmh = std::sqrt(mo.linear.x * mo.linear.x + mo.linear.y * mo.linear.y) * 3.6;
        const auto& pt = world.powertrain_state(vid);
        const auto dbg = world.powertrain_debug(vid);
        const auto& en = world.powertrain_energy(vid);
        const double eng_kw = (en.engine_out - prev_engine_out) / dt / 1000.0;
        prev_engine_out = en.engine_out;
        ps::drivetrain::EngineSoundState es{};
        if (!pt.engines.empty()) es = pt.engines[0];
        const double omega = es.omega;
        const double hud_kw = -pt.driveline.clutch_torque * omega / 1000.0;
        const auto& aero = world.vehicle_aero_telemetry(vid);
        const auto& rl = world.wheel_telemetry(vid, 2);
        const auto& rr = world.wheel_telemetry(vid, 3);
        if (csv.is_open()) {
            csv << t << ',' << kmh << ',' << int(pt.gear) << ',' << int(pt.gear_target) << ',' << int(pt.shift_phase) << ','
                << pt.engine_rpm << ',' << pedal << ',' << es.throttle_command << ',' << es.throttle_position << ',' << es.load
                << ',' << dbg.throttle_eff << ',' << dbg.throttle_ceiling << ',' << (dbg.ignition_cut ? 1 : 0) << ','
                << dbg.engine_torque_demand << ',' << eng_kw << ',' << (omega > 1.0 ? eng_kw * 1000.0 / omega : 0.0) << ','
                << hud_kw << ',' << pt.driveline.clutch_torque << ',' << dbg.clutch_slip << ',' << (dbg.clutch_stuck ? 1 : 0)
                << ',' << pt.clutch_engagement << ',' << dbg.clutch_cap_kinetic << ',' << es.boost_bar << ','
                << (es.p_charge_pa > 0 ? (es.p_charge_pa - 101325.0) / 1e5 : 0.0) << ',' << es.p_intake_pa / 1000.0 << ','
                << es.wastegate_opening << ',' << es.turbo_rpm << ',' << es.compressor_flow_kg_s << ','
                << es.compressor_pressure_ratio << ',' << int(es.surge_active) << ',' << es.surge_events << ','
                << es.bov_opening << ',' << es.bov_events << ',' << (es.limiter_active ? 1 : 0) << ','
                << (es.dfco_active ? 1 : 0) << ',' << (es.launch_active ? 1 : 0) << ','
                << (pt.traction_control_intervening ? 1 : 0) << ',' << int(pt.traction_mode_effective) << ','
                << pt.traction_torque_fraction << ',' << pt.traction_positive_slip << ',' << (dbg.tc_via_clutch ? 1 : 0)
                << ',' << int(es.n2o_solenoid_open) << ',' << es.n2o_flow_fraction << ',' << es.n2o_flow_g_s << ','
                << int(pt.drive_mode_effective) << ',' << (pt.aero_control_active ? 1 : 0) << ','
                << pt.aero_control_target_deg << ',' << aero.downforce_n << ',' << rl.slip_ratio << ',' << rr.slip_ratio
                << ',' << rl.fx << ',' << rr.fx << ',' << rl.fz << ',' << rr.fz << ',' << es.spark_advance_deg << ','
                << es.lambda << ',' << es.fuel_flow_g_s << ',' << (es.turbo_compressor_stage_used ? 1 : 0) << ','
                << (es.turbo_turbine_stage_used ? 1 : 0) << ',' << (es.turbo_stage_map_fallback ? 1 : 0) << ','
                << es.p_exhaust_pa / 1000.0 << ',' << es.turbine_inlet_k << '\n';
        }
        if (step % 240 == 0) {
            std::printf("DIAG t=%5.1f kmh=%6.1f gear=%d rpm=%5.0f eng_kw=%7.1f hud_kw=%7.1f boost=%5.2f wg=%.2f turbo=%6.0f "
                        "tc=%d frac=%.2f lim=%d slipRL=%.3f\n",
                        t, kmh, int(pt.gear), pt.engine_rpm, eng_kw, hud_kw, es.boost_bar, es.wastegate_opening,
                        es.turbo_rpm, pt.traction_control_intervening ? 1 : 0, pt.traction_torque_fraction,
                        es.limiter_active ? 1 : 0, rl.slip_ratio);
        }
        if (mo.linear.x > 0 && world.get_pose(session.chassis_body()).position.x > 19000.0) {
            std::printf("DIAG ran out of ground at t=%.1f\n", t);
            break;
        }
    }
    SUCCEED();
}

TEST_CASE("diag: hypercar engine dyno limits", "[.][diag][power_jumps]") {
    const std::string car = env_or("RG_DIAG_CAR", "car_hyper_n2o");
    const std::string setup_path = default_setup(car);
    rg::ScopedWorkDir root(fs::temp_directory_path().generic_string(), "rg_power_dyno");
    std::unique_ptr<rg::ScopedWorkDir> work;
    const Built built = build_car(car, setup_path, root, work);
    ps::jobs::JobSystem pool(static_cast<int>(std::max(1u, std::thread::hardware_concurrency() - 2)));
    ps::io::EngineMapOptions maps;
    maps.cache_dir = built.cache_dir;
    maps.pool = &pool;
    const auto desc = rg::load_vehicle(built.vehicle_path, maps);
    const ps::drivetrain::ComponentParams* engine = nullptr;
    const ps::drivetrain::SimulatedEngineDesc* sim = nullptr;
    for (const auto& c : desc.powertrain.components) {
        if (const auto* s = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) { engine = &c.params; sim = s; }
    }
    REQUIRE(sim != nullptr);
    ps::drivetrain::DynoSweepSpec spec;
    for (double rpm = 2000.0; rpm <= sim->limiter.rpm - 1.0; rpm += 500.0) spec.rpm.push_back(rpm);
    spec.rpm.push_back(sim->limiter.rpm - 50.0);
    spec.turbo_count = sim->turbo_pair ? 2 : 1;
    if (env_or("RG_DIAG_NITROUS", "0") == "1") spec.nitrous = ps::drivetrain::nitrous_from_powertrain(desc.powertrain);
    std::printf("DIAG dyno setup=%s vehicle=%s turbo=%d limiter=%.0f\n", setup_path.c_str(), built.vehicle_path.c_str(),
                sim->turbo_pair ? 1 : 0, sim->limiter.rpm);
    const auto r = ps::drivetrain::run_dyno_sweep(*engine, nullptr, spec, &pool);
    std::printf("DIAG rpm  torque  power  boost  charge  turbo_rpm  p_im_kPa  conv settle_s\n");
    for (const auto& p : r.points) {
        std::printf("DIAG %5.0f %7.1f %6.1f %6.2f %6.2f %9.0f %8.1f %d %5.2f\n", p.rpm, p.torque_nm, p.power_kw, p.boost_bar,
                    p.charge_boost_bar, p.turbo_rpm, p.p_intake_kpa, p.converged ? 1 : 0, p.settle_s);
    }
    for (const auto& f : r.limits) {
        std::printf("DIAG LIMIT rpm=%5.0f %s %s value=%.4g limit=%.4g %s part=%s%s\n", f.rpm, ps::drivetrain::to_string(f.kind),
                    ps::drivetrain::to_string(f.severity), f.value, f.limit, ps::drivetrain::to_string(f.unit),
                    f.part.file.c_str(), f.part.field.c_str());
    }
    for (const auto& f : ps::drivetrain::check_driveline_limits(r, desc.powertrain)) {
        std::printf("DIAG DRIVELINE %s %s value=%.4g limit=%.4g\n", ps::drivetrain::to_string(f.kind),
                    ps::drivetrain::to_string(f.severity), f.value, f.limit);
    }
    // Part-throttle dyno (the pedal is the ECU torque request; the boost target follows the plate): steady torque per
    // pedal at RG_DIAG_DYNO_RPM (default 5000), and how long each point needs to settle.
    ps::drivetrain::DynoSweepSpec part = spec;
    part.rpm = {std::atof(env_or("RG_DIAG_DYNO_RPM", "5000").c_str())};
    std::printf("DIAG part-throttle rpm=%.0f: pedal torque power boost charge turbo_rpm p_im conv settle_s\n", part.rpm[0]);
    for (double pedal : {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0}) {
        part.throttle = pedal;
        const auto pr = ps::drivetrain::run_dyno_sweep(*engine, nullptr, part, &pool);
        for (const auto& p : pr.points) {
            std::printf("DIAG PT %.1f %7.1f %7.1f %6.2f %6.2f %9.0f %8.1f %d %5.2f\n", pedal, p.torque_nm, p.power_kw, p.boost_bar,
                        p.charge_boost_bar, p.turbo_rpm, p.p_intake_kpa, p.converged ? 1 : 0, p.settle_s);
        }
    }
    SUCCEED();
}
