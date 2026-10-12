// test_hyper_rear_travel.cpp - hidden diagnostic [.][diag][hyper_travel]: does the hypercar's rear suspension reach its
// bump stop at speed? (owner report 2026-10-12: the Hyper One N2O gets bouncy and light at the front above 390 km/h.)
//
// Builds car_hyper_n2o (or RG_DIAG_CAR) with a saved garage setup (RG_DIAG_SETUP = a racing_game.vehicle_setup/1 file,
// default = the owner's saved one under %APPDATA%), accelerates in a flat Session (tessellated mesh ground, 40 km x 1 km, cells <= 20 m: a big box or
// big triangles give phantom wheel-cast hits, PHYS-047 / rg::flat_ground_mesh), ignition + starter, assist.auto_shift 1, throttle 1, and bins per 20 km/h: rear max
// compression, RL/RR fz high-pass amplitude, front fz minimum, traction-control activity. The table goes to stdout; the
// per-tick CSV to RG_DIAG_CSV when set. Other env switches: RG_DIAG_STOP_KMH (default 480), RG_DIAG_NITROUS=1 (arm the kit),
// RG_DIAG_TC=0 (assist.traction_control off), RG_DIAG_LAUNCH_THROTTLE / RG_DIAG_LAUNCH_KMH (throttle below that speed; the
// owner's tall-geared setup stalls or hops on a full-throttle launch with TC on, 0.6 until 40 km/h is reliable),
// RG_DIAG_VEHICLE (repo-relative vehicle file replacing the catalog entry's, e.g. a copy of the pre-change file for a
// before/after run; keep it next to the other vehicle files so its relative refs resolve), RG_DIAG_SETUP=none (stock car).
// Columns: hp-* = fz minus a 0.05 s low-pass (the bounce band), hp-pp = its peak-to-peak per bin; the bump stop engages at
// 94.5 mm (0.9 x travel_max) and "overStop%" is the share of ticks a rear wheel was above it.
// 2026-10-12 result for the owner's car_hyper_n2o setup, TC on: rear max travel 94.8 mm at 380-400 km/h and 99 mm above
// 440 before the 1.3x spring change; 73.9 / 77.4 / 85.1 / 89.3 / 93.9 mm at 380-400 / 400-420 / 440-460 / 460-480 / 480-500
// after it (never above the stop). Flat road shows no fz oscillation either way (hp-pp < 10 N from 320 km/h).
//
// Not registered as a normal test: it runs minutes, depends on a user file and only measures.
#include "rg/session.h"
#include "rg/vehicle_catalog.h"
#include "rg/vehicle_setup.h"

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
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

struct Bin {
    long n = 0;
    double max_comp[2] = {0, 0};   // RL, RR [m]
    double max_comp_front[2] = {0, 0};
    long over_stop = 0;            // ticks with a rear wheel above 94.5 mm
    double hp_sq[4] = {0, 0, 0, 0}; // fz high-pass energy: RL, RR, FL, FR
    double hp_pp_min[4] = {1e30, 1e30, 1e30, 1e30};
    double hp_pp_max[4] = {-1e30, -1e30, -1e30, -1e30};
    double min_fz[4] = {1e30, 1e30, 1e30, 1e30};
    double sum_fz[4] = {0, 0, 0, 0};
    long tc_ticks = 0;
    double tc_min_fraction = 1.0;
    double pitch_max = -1e30;
};
} // namespace

TEST_CASE("diag: hypercar rear suspension travel vs speed", "[.][diag][hyper_travel]") {
    const std::string car = env_or("RG_DIAG_CAR", "car_hyper_n2o");
    const std::string setup_path = env_or(
        "RG_DIAG_SETUP", env_or("APPDATA", "") + "/Godot/app_userdata/racing_game R0/garage/setups/" + car + ".json");
    const double stop_kmh = std::atof(env_or("RG_DIAG_STOP_KMH", "480").c_str());
    const bool arm = env_or("RG_DIAG_NITROUS", "0") == "1";

    std::string err;
    auto catalog = rg::load_vehicle_catalog(kRepo + "/data/vehicles/catalog.json", kRepo, &err);
    REQUIRE(catalog.has_value());
    auto table = rg::load_setup_options(kRepo + "/data/vehicles/setup_options.json", &err);
    REQUIRE(table.has_value());
    const rg::CatalogEntry* found = catalog->find(car);
    REQUIRE(found != nullptr);
    rg::CatalogEntry entry_copy = *found;
    const std::string vehicle_override = env_or("RG_DIAG_VEHICLE", ""); // e.g. a copy of the pre-change file for a before/after run
    if (!vehicle_override.empty()) entry_copy.vehicle_path = kRepo + "/" + vehicle_override;
    const rg::CatalogEntry* entry = &entry_copy;

    rg::SetupContext ctx;
    ctx.tyre_families_path = kRepo + "/data/tyres/families.json";
    rg::ScopedWorkDir root(fs::temp_directory_path().generic_string(), "rg_hyper_travel");
    ctx.work_root = root.path();
    ctx.engine_map_cache_dir = kRepo + "/out/godot_engine_cache";
    auto store = rg::EngineStore::open(kRepo + "/data/engines/engine_catalog.json", kRepo, "", &err);
    REQUIRE(store != nullptr);
    ctx.engines = std::shared_ptr<const rg::EngineStore>(std::move(store));

    std::string vehicle_path = entry->vehicle_path;
    rg::ScopedWorkDir work(ctx.work_root, "setup_" + car);
    if (setup_path != "none") {
        std::ifstream in(setup_path);
        REQUIRE(in.good());
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        auto setup = rg::parse_setup(text, *table, setup_path, &err);
        INFO(err);
        REQUIRE(setup.has_value());
        const rg::MaterialisedSetup m = rg::materialise_setup(*entry, *table, ctx, *setup, work.path());
        INFO(m.error);
        REQUIRE(m.ok);
        vehicle_path = m.vehicle_path;
    }
    std::printf("DIAG car=%s setup=%s vehicle=%s\n", car.c_str(), setup_path.c_str(), vehicle_path.c_str());

    rg::SessionConfig config;
    config.vehicle_json_path = vehicle_path;
    config.surface_table_path = kRepo + "/data/surfaces/surfaces.json";
    config.chassis_mass_kg = entry->chassis.mass_kg;
    config.chassis_half_extents = ps::Vec3{entry->chassis.half_extents[0], entry->chassis.half_extents[1], entry->chassis.half_extents[2]};
    config.chassis_z_m = entry->chassis.spawn_z_m;
    config.ground_half_extent_m = 20000.0; // 40 km long strip, tessellated (a 20 km box gives the PHYS-047 launch hop)
    config.ground_half_extent_y_m = 500.0;
    config.ground_tessellated = true;
    config.engine_map_cache_dir = ctx.engine_map_cache_dir;
    rg::Session session(config);
    auto& world = session.world();
    const auto vid = session.vehicle_id();
    const auto& desc = session.vehicle_desc();
    REQUIRE(desc.wheels.size() == 4);
    const double travel_max = desc.wheels[2].suspension.travel_max;
    const double stop_engage = 0.0945; // bump stop engagement (from the vehicle file: 90 % of travel_max)
    std::printf("DIAG rear spring %.0f N/m bump %.0f rebound %.0f travel_max %.4f; front spring %.0f; TC default %s\n",
                desc.wheels[2].suspension.spring_rate, desc.wheels[2].suspension.damper_bump_rate,
                desc.wheels[2].suspension.damper_rebound_rate, travel_max, desc.wheels[0].suspension.spring_rate,
                desc.traction_control.enabled ? "on" : "off");

    // Static ride height after settling.
    for (int i = 0; i < 480; ++i) session.step();
    std::printf("DIAG static chassis z %.4f m; compression FL %.4f FR %.4f RL %.4f RR %.4f\n",
                world.get_pose(session.chassis_body()).position.z, world.wheel_telemetry(vid, 0).compression,
                world.wheel_telemetry(vid, 1).compression, world.wheel_telemetry(vid, 2).compression,
                world.wheel_telemetry(vid, 3).compression);

    world.set_control("ignition", 1.0);
    world.set_control("starter", 1.0);
    world.set_control("assist.auto_shift", 1.0);
    world.set_control("assist.auto_clutch", 1.0);
    world.set_control("assist.auto_blip", 1.0);
    world.set_control("clutch", 0.0);
    world.set_control("brake", 0.0);
    world.set_control("handbrake", 0.0);
    if (arm) world.set_control("nitrous_arm", 1.0);
    if (env_or("RG_DIAG_TC", "1") == "0") world.set_control("assist.traction_control", 0.0);

    std::ofstream csv;
    const std::string csv_path = env_or("RG_DIAG_CSV", "");
    if (!csv_path.empty()) {
        csv.open(csv_path);
        csv << "t,kmh,gear,rpm,comp_fl,comp_fr,comp_rl,comp_rr,fz_fl,fz_fr,fz_rl,fz_rr,tc,tcfrac,pitch\n";
    }

    constexpr double kBinKmh = 20.0;
    std::vector<Bin> bins(40);
    const double dt = 1.0 / 240.0;
    const double tau = 0.05; // high-pass corner ~3 Hz: removes the acceleration ramp, keeps the bounce band
    const double alpha = dt / tau;
    double ema[4] = {0, 0, 0, 0};
    bool ema_init = false;
    double t = 0.0, top_kmh = 0.0, plateau_kmh = 0.0, plateau_t = 0.0;
    bool started = false;
    const double launch_throttle = std::atof(env_or("RG_DIAG_LAUNCH_THROTTLE", "1").c_str());
    const double launch_kmh = std::atof(env_or("RG_DIAG_LAUNCH_KMH", "0").c_str());
    long max_steps = 240L * 300L;
    for (long step = 0; step < max_steps; ++step) {
        if (!started && world.powertrain_state(vid).engine_state == ps::drivetrain::EngineState::Running) {
            world.set_control("starter", 0.0);
            world.set_control("throttle", 1.0);
            started = true;
        }
        if (!started && step > 240 * 6) world.set_control("starter", 0.0), world.set_control("throttle", 1.0);
        session.step();
        t += dt;
        const ps::Motion mo = world.get_motion(session.chassis_body());
        const double kmh = std::sqrt(mo.linear.x * mo.linear.x + mo.linear.y * mo.linear.y) * 3.6;
        if (started) world.set_control("throttle", kmh < launch_kmh ? launch_throttle : 1.0);
        const auto pt = world.powertrain_state(vid);
        double comp[4], fz[4];
        for (int w = 0; w < 4; ++w) {
            const auto tel = world.wheel_telemetry(vid, w);
            comp[w] = tel.compression;
            fz[w] = tel.fz;
        }
        const int order[4] = {2, 3, 0, 1}; // RL, RR, FL, FR
        if (!ema_init) {
            for (int k = 0; k < 4; ++k) ema[k] = fz[order[k]];
            ema_init = true;
        }
        const ps::Pose pose = world.get_pose(session.chassis_body());
        const double pitch = std::asin(std::clamp(-pose.orientation.rotate(ps::Vec3::unit_x()).z, -1.0, 1.0));
        if (kmh > 20.0) { // bins only once moving
            const std::size_t bi = std::min<std::size_t>(static_cast<std::size_t>(kmh / kBinKmh), bins.size() - 1);
            Bin& b = bins[bi];
            ++b.n;
            b.max_comp[0] = std::max(b.max_comp[0], comp[2]);
            b.max_comp[1] = std::max(b.max_comp[1], comp[3]);
            b.max_comp_front[0] = std::max(b.max_comp_front[0], comp[0]);
            b.max_comp_front[1] = std::max(b.max_comp_front[1], comp[1]);
            if (comp[2] > stop_engage || comp[3] > stop_engage) ++b.over_stop;
            for (int k = 0; k < 4; ++k) {
                const double hp = fz[order[k]] - ema[k];
                b.hp_sq[k] += hp * hp;
                b.hp_pp_min[k] = std::min(b.hp_pp_min[k], hp);
                b.hp_pp_max[k] = std::max(b.hp_pp_max[k], hp);
                b.min_fz[k] = std::min(b.min_fz[k], fz[order[k]]);
                b.sum_fz[k] += fz[order[k]];
            }
            if (pt.traction_control_intervening) ++b.tc_ticks;
            b.tc_min_fraction = std::min(b.tc_min_fraction, static_cast<double>(pt.traction_torque_fraction));
            b.pitch_max = std::max(b.pitch_max, pitch);
        }
        for (int k = 0; k < 4; ++k) ema[k] += alpha * (fz[order[k]] - ema[k]);
        if (csv.is_open() && (step % 4) == 0) {
            csv << t << ',' << kmh << ',' << static_cast<int>(pt.gear) << ',' << pt.engine_rpm << ',' << comp[0] << ',' << comp[1] << ',' << comp[2] << ','
                << comp[3] << ',' << fz[0] << ',' << fz[1] << ',' << fz[2] << ',' << fz[3] << ','
                << (pt.traction_control_intervening ? 1 : 0) << ',' << pt.traction_torque_fraction << ',' << pitch << '\n';
        }
        top_kmh = std::max(top_kmh, kmh);
        if (kmh > 200.0 && t - plateau_t >= 10.0) {
            if (kmh - plateau_kmh < 0.5) { std::printf("DIAG top speed plateau %.1f km/h at t=%.1f s\n", kmh, t); break; }
            plateau_kmh = kmh;
            plateau_t = t;
        } else if (kmh <= 200.0 && t - plateau_t >= 10.0) {
            plateau_kmh = kmh;
            plateau_t = t;
        }
        if (kmh >= stop_kmh) break;
        if (mo.linear.x * 3.6 > 0 && pose.position.x > 19000.0) { std::printf("DIAG ran out of ground\n"); break; }
    }
    std::printf("DIAG finished t=%.1f s top %.1f km/h started=%d\n", t, top_kmh, started ? 1 : 0);
    std::printf("DIAG bin(km/h)  n   maxRL  maxRR  maxFL maxFR [mm] | overStop%% | hp-std RL RR FL FR [N] | hp-pp RL RR [N] | minFz RL RR FL FR [N] | TC%% tcMin | pitchMax[deg]\n");
    for (std::size_t i = 0; i < bins.size(); ++i) {
        const Bin& b = bins[i];
        if (b.n == 0) continue;
        std::printf("DIAG %3zu-%3zu %6ld %6.1f %6.1f %6.1f %6.1f | %5.1f | %6.0f %6.0f %6.0f %6.0f | %6.0f %6.0f | %6.0f %6.0f %6.0f %6.0f | %5.1f %.2f | %.2f\n",
                    i * 20, i * 20 + 20, b.n, b.max_comp[0] * 1000, b.max_comp[1] * 1000, b.max_comp_front[0] * 1000,
                    b.max_comp_front[1] * 1000, 100.0 * b.over_stop / b.n, std::sqrt(b.hp_sq[0] / b.n), std::sqrt(b.hp_sq[1] / b.n),
                    std::sqrt(b.hp_sq[2] / b.n), std::sqrt(b.hp_sq[3] / b.n), b.hp_pp_max[0] - b.hp_pp_min[0], b.hp_pp_max[1] - b.hp_pp_min[1],
                    b.min_fz[0], b.min_fz[1], b.min_fz[2], b.min_fz[3], 100.0 * b.tc_ticks / b.n, b.tc_min_fraction,
                    b.pitch_max * 57.29578);
    }
    SUCCEED();
}
