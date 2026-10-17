// test_nitrous_gain.cpp - S1 open item: the armed-vs-unarmed gain of the two N2O catalog cars.
// Rolling start in a held gear at a fixed rpm on the flat world, wide-open throttle, once with the kit unarmed and
// once armed. Reports dv over a fixed window, the time from the start rpm to 6000 rpm and the maximum boost.
// Hidden: run with "[.][nitrous][gain]". Asserts only sanity (the armed run sprayed and gained speed).
#include "rg/nitrous_info.h"
#include "rg/session.h"
#include "rg/vehicle_catalog.h"

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

const std::string kRepo = RG_SOURCE_DIR;

constexpr double kStartRpm = 2500.0;
constexpr double kTargetRpm = 6000.0;
constexpr double kWindowS = 2.0; // dv window
constexpr double kRunS = 8.0;    // total run (time-to-6000 search)

struct Run {
    double dv_window = 0.0, v0 = 0.0, start_rpm = 0.0;
    double t_to_target = -1.0; // -1 = not reached
    double max_boost_bar = 0.0, boost_at_window = 0.0;
    double spray_s = 0.0, first_spray_s = -1.0;
    double max_rpm = 0.0;
    int max_cut = 0;
    double bottle_start = 0.0, bottle_end = 0.0;
};

Run measure(const std::string& id, int gear, bool armed, int drive_mode = -1) {
    std::string err;
    const auto catalog = rg::load_vehicle_catalog(kRepo + "/data/vehicles/catalog.json", kRepo, &err);
    INFO(err);
    REQUIRE(catalog.has_value());
    const rg::CatalogEntry* entry = catalog->find(id);
    REQUIRE(entry != nullptr);

    rg::SessionConfig config;
    config.vehicle_json_path = entry->vehicle_path;
    config.surface_table_path = kRepo + "/data/surfaces/surfaces.json";
    config.chassis_mass_kg = entry->chassis.mass_kg;
    config.chassis_half_extents = ps::Vec3{entry->chassis.half_extents[0], entry->chassis.half_extents[1],
                                           entry->chassis.half_extents[2]};
    config.chassis_z_m = entry->chassis.spawn_z_m;
    rg::Session session(config);
    auto& world = session.world();
    const rg::NitrousInfo info = rg::nitrous_info(session.vehicle_desc());
    REQUIRE(info.present);

    world.set_control("ignition", 1.0);
    if (drive_mode >= 0) world.set_control("assist.drive_mode", drive_mode); // before launch control: Sport defaults it on
    world.set_control("assist.auto_shift", 0.0);
    world.set_control("assist.auto_clutch", 0.0);
    world.set_control("assist.launch_control", 0.0);
    world.set_control("clutch", 0.0);
    world.set_control("throttle", 0.0);
    world.set_control("steer", 0.0);
    world.set_control("nitrous_arm", 0.0);
    for (int i = 0; i < 240; ++i) session.step();

    const double dt = static_cast<double>(world.dt());
    // Put the car at speed in the gear: speed scaled so the engine sits at kStartRpm (the rpm is G * hub speed).
    auto roll_at = [&](double v) {
        ps::Motion m;
        m.linear = ps::Vec3{v, 0, 0}; // flat spawn: yaw 0, chassis +x is forward
        world.backend().set_motion(session.chassis_body(), m);
        ps::vehicle::VehicleResetOptions opts;
        opts.gear = gear;
        opts.engine_state = ps::drivetrain::EngineState::Running;
        opts.hubs_roll_with_chassis = true;
        opts.clutch_locked = true;
        world.reset_vehicle(session.vehicle_id(), opts);
    };
    double v = 15.0;
    for (int k = 0; k < 4; ++k) {
        roll_at(v);
        session.step();
        const double rpm = static_cast<double>(world.powertrain_state(session.vehicle_id()).engine_rpm);
        REQUIRE(rpm > 100.0);
        v *= kStartRpm / rpm;
    }
    roll_at(v);
    world.set_control("gear_request", gear);
    world.set_control("nitrous_arm", armed ? 1.0 : 0.0);
    world.set_control("throttle", 1.0);

    auto speed = [&] { return static_cast<double>(world.backend().get_motion(session.chassis_body()).linear.length()); };
    Run r;
    r.v0 = speed();
    r.start_rpm = static_cast<double>(world.powertrain_state(session.vehicle_id()).engine_rpm);
    r.bottle_start = static_cast<double>(world.powertrain_state(session.vehicle_id()).n2o_bottle_kg);
    const int n = static_cast<int>(kRunS / dt);
    const int nw = static_cast<int>(kWindowS / dt);
    for (int i = 1; i <= n; ++i) {
        session.step();
        const auto& p = world.powertrain_state(session.vehicle_id());
        const rg::NitrousHud h = rg::nitrous_hud(info, p, world.get_control("nitrous_arm"));
        const double t = i * dt;
        if (std::getenv("RG_N2O_DEBUG") && armed && i % 60 == 0 && !p.engines.empty())
            std::printf("dbg t=%.2f gear=%d rpm=%.0f sol=%d act=%d bar=%.1f cut=%d thr=%.2f tc=%d boost=%.2f\n", t, (int)p.gear, (double)p.engine_rpm,
                        (int)h.solenoid_open, (int)h.spraying, h.bottle_bar, h.safety_cut, (double)world.get_control("throttle"),
                        (int)p.traction_control_intervening, (double)p.engines[0].boost_bar);
        const double rpm = static_cast<double>(p.engine_rpm);
        r.max_rpm = std::max(r.max_rpm, rpm);
        r.max_cut = std::max(r.max_cut, h.safety_cut);
        if (h.spraying) {
            r.spray_s += dt;
            if (r.first_spray_s < 0) r.first_spray_s = t;
        }
        if (!p.engines.empty()) r.max_boost_bar = std::max(r.max_boost_bar, static_cast<double>(p.engines[0].boost_bar));
        if (r.t_to_target < 0 && rpm >= kTargetRpm) r.t_to_target = t;
        if (i == nw) {
            r.dv_window = speed() - r.v0;
            if (!p.engines.empty()) r.boost_at_window = static_cast<double>(p.engines[0].boost_bar);
        }
        r.bottle_end = static_cast<double>(p.n2o_bottle_kg);
    }
    return r;
}

void print_table(const std::string& id, int gear, const char* label, const Run& off, const Run& on) {
    std::printf("\nN2O gain, %s, gear %d, %s, WOT from %.0f rpm (v0 %.1f m/s), flat world\n", id.c_str(), gear, label, kStartRpm, on.v0);
    std::printf("                      unarmed     armed\n");
    std::printf("dv over %.1f s (m/s)   %8.2f  %8.2f   (%+.1f %%)\n", kWindowS, off.dv_window, on.dv_window,
                100.0 * (on.dv_window / off.dv_window - 1.0));
    std::printf("t to %.0f rpm (s)     %8.2f  %8.2f   (-1 = not reached in %.0f s)\n", kTargetRpm, off.t_to_target, on.t_to_target, kRunS);
    std::printf("max boost (bar)       %8.2f  %8.2f   (at window end %.2f / %.2f)\n", off.max_boost_bar, on.max_boost_bar,
                off.boost_at_window, on.boost_at_window);
    std::printf("max rpm               %8.0f  %8.0f\n", off.max_rpm, on.max_rpm);
    std::printf("spray time (s)        %8.2f  %8.2f   first spray at %.2f s, safety cut max %d\n", off.spray_s, on.spray_s,
                on.first_spray_s, on.max_cut);
    std::printf("bottle kg start/end   %.2f/%.2f  %.2f/%.2f\n", off.bottle_start, off.bottle_end, on.bottle_start, on.bottle_end);
}

void report(const std::string& id, int gear, int drive_mode, const char* label) {
    const Run off = measure(id, gear, false, drive_mode);
    const Run on = measure(id, gear, true, drive_mode);
    print_table(id, gear, label, off, on);
    CHECK(off.spray_s == 0.0);
    REQUIRE(on.spray_s > 0.0); // the armed run must actually reach SPRAYING
    CHECK(on.dv_window > 0.0);
    CHECK(on.dv_window > off.dv_window);
}

} // namespace

// The hypercar's drive modes (data/electronics/drive_modes_car_hyper_n2o.json) allow nitrous only in Sport (2): in the
// default Normal mode the kit arms but never sprays. The measured run is therefore Sport for both columns.
TEST_CASE("nitrous gain: hypercar 3rd gear, armed vs unarmed (Sport)", "[.][nitrous][gain]") { report("car_hyper_n2o", 3, 2, "Sport mode"); }
TEST_CASE("nitrous gain: hypercar 3rd gear in Normal mode never sprays", "[.][nitrous][gain][mode]") {
    const Run on = measure("car_hyper_n2o", 3, true, 1);
    std::printf("\nhyper Normal mode, armed: spray %.2f s, dv %.2f m/s\n", on.spray_s, on.dv_window);
    CHECK(on.spray_s == 0.0);
}
TEST_CASE("nitrous gain: sedan 2nd gear, armed vs unarmed", "[.][nitrous][gain]") { report("car_sedan_gen_n2o", 2, -1, "default"); }
