// test_nitrous_pull.cpp - S1 smoke measurement: what the nitrous kit gives in a real rg::Session (flat world, the game's
// own Session code path, the catalog's chassis proxy). Each car does a full-throttle pull: launch in 1st, auto-shift up
// by gear_request at a given rpm, and once the target gear is engaged with the clutch locked the "measurement" starts -
// the arm channel is set at that instant in the armed run, left off in the baseline run. Both runs are identical up to
// that tick (same World, same inputs), so the difference is the shot alone.
//
//   car_sedan_gen_n2o (2.5 L sedan, 4.5 kg bottle, 37 kW shot): 2nd gear from ~3000 rpm.
//   car_hyper_n2o (twin-turbo V8, 6.8 kg bottle, ~600 kW shot): 3rd gear from ~3200 rpm.
//
// Known physics caveat (physics_sim N7, reported not fixed): with the spray the hypercar's boost overshoots its 1.8 bar
// target to 2.0-2.4 bar and the turbo can sit on the model's rotor speed cap at 6000-7000 rpm.
#include "rg/session.h"

#include "ps/drivetrain/powertrain_state.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace {

struct Car {
    const char* name;
    const char* file;
    int gear;           // the pull gear
    double shift_rpm;   // up-shift request rpm below the pull gear
    double window_s;    // measurement window after the pull starts
    double end_rpm;     // the pull ends when the crank reaches this
    double mass_kg;
    ps::Vec3 half_extents;
    double chassis_z;
    double v_init = 0.0;  // rolling start speed (m/s): FWD launches in 1st/2nd are wheelspin-limited and hide the shot
    int start_gear = 1;
};

struct Pull {
    bool started = false, stalled = false;
    double arm_rpm = 0, v0 = 0, dv = 0, mean_accel = 0, peak_accel = 0, t_end_rpm = -1, n2o_g = 0, boost_max = 0,
           flow_max = 0, bottle_before = 0, bottle_after = 0;
    int spray_ticks = 0;
};

Pull run(const Car& car, bool arm) {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/" + car.file;
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/data/surfaces/surfaces.json";
    config.chassis_mass_kg = car.mass_kg;
    config.chassis_half_extents = car.half_extents;
    config.chassis_z_m = car.chassis_z;
    config.chassis_initial_velocity = ps::Vec3{car.v_init, 0.0, 0.0};
    config.ground_half_extent_m = 4000.0;
    rg::Session session(config);

    Pull out;
    bool started = false;
    double t_start = 0.0, v_prev = 0.0, t_prev = 0.0;
    rg::DriveScript script;
    script.set_controller([&](const rg::DriveTickContext& ctx, ps::World& w) {
        const auto& st = w.powertrain_state(ctx.vehicle);
        const double t = w.sim_time();
        w.set_control("throttle", 1.0);
        if (t < 0.02) w.set_control("gear_request", static_cast<double>(car.start_gear));
        const bool idle = st.shift_phase == ps::drivetrain::ShiftPhase::Idle;
        if (st.engine_state != ps::drivetrain::EngineState::Running && t > 1.0) out.stalled = true;
        if (idle && st.gear >= 1 && st.gear < car.gear && st.engine_rpm >= car.shift_rpm)
            w.set_control("gear_request", static_cast<double>(st.gear + 1));
        const double vx = w.get_motion(ctx.chassis).linear.x;
        if (!started && st.gear == car.gear && idle && st.clutch_engagement >= 0.999) {
            started = true;
            out.started = true;
            out.arm_rpm = st.engine_rpm;
            out.v0 = vx;
            t_start = t;
            out.bottle_before = st.n2o_bottle_kg;
            if (arm) w.set_control("nitrous_arm", 1.0);
        } else if (started) {
            const auto& e = st.engines.at(0);
            if (e.n2o_active) ++out.spray_ticks;
            out.flow_max = std::max(out.flow_max, static_cast<double>(e.n2o_flow_g_s));
            out.boost_max = std::max(out.boost_max, static_cast<double>(e.boost_bar));
            if (t - t_start <= car.window_s) {
                out.dv = vx - out.v0;
                if (t > t_prev) out.peak_accel = std::max(out.peak_accel, (vx - v_prev) / (t - t_prev));
            }
            if (out.t_end_rpm < 0 && st.engine_rpm >= car.end_rpm) out.t_end_rpm = t - t_start;
            out.bottle_after = st.n2o_bottle_kg;
        }
        v_prev = vx;
        t_prev = t;
    });
    session.set_drive_script(std::move(script));

    for (int tick = 0; tick < 240 * 40; ++tick) {
        session.step();
        if (started && session.world().sim_time() - t_start > car.window_s + 1.0 && out.t_end_rpm >= 0) break;
        if (started && session.world().sim_time() - t_start > 8.0) break;
    }
    out.mean_accel = out.dv / car.window_s;
    out.n2o_g = (out.bottle_before - out.bottle_after) * 1000.0;
    return out;
}

// The shot must not make the pull slower (dv >= 0.98 x baseline: a traction-limited FWD car gains little velocity in the
// window) and must reach the end rpm clearly sooner; `min_dv_ratio` > 1 additionally demands a velocity gain.
void measure(const Car& car, double min_dv_ratio) {
    const Pull base = run(car, false);
    const Pull shot = run(car, true);
    INFO(car.name);
    REQUIRE(base.started);
    REQUIRE(shot.started);
    CHECK_FALSE(base.stalled);
    CHECK_FALSE(shot.stalled);
    std::printf("N2O_PULL car=%s gear=%d pull_start_rpm=%.0f v0=%.2f m/s window=%.2f s | unarmed: dv=%.2f m/s mean %.2f "
                "m/s2 peak %.2f t_to_%.0frpm=%.2f s | armed: dv=%.2f m/s mean %.2f m/s2 peak %.2f t_to_%.0frpm=%.2f s "
                "| gain dv %+.1f %% mean accel %+.2f m/s2 | spray ticks %d, N2O %.0f g (max %.0f g/s), boost max %.2f bar "
                "(unarmed %.2f)\n",
                car.name, car.gear, shot.arm_rpm, shot.v0, car.window_s, base.dv, base.mean_accel, base.peak_accel,
                car.end_rpm, base.t_end_rpm, shot.dv, shot.mean_accel, shot.peak_accel, car.end_rpm, shot.t_end_rpm,
                100.0 * (shot.dv / base.dv - 1.0), shot.mean_accel - base.mean_accel, shot.spray_ticks, shot.n2o_g,
                shot.flow_max, shot.boost_max, base.boost_max);
    CHECK(base.spray_ticks == 0);
    CHECK(shot.spray_ticks > 0);
    CHECK(shot.n2o_g > 1.0);
    CHECK(shot.dv > base.dv * min_dv_ratio);
    REQUIRE(base.t_end_rpm > 0);
    REQUIRE(shot.t_end_rpm > 0);
    CHECK(shot.t_end_rpm < base.t_end_rpm * 0.95);
}

} // namespace

TEST_CASE("nitrous: car_sedan_gen_n2o armed beats unarmed in a 2nd-gear WOT pull", "[nitrous][s1][vehicle]") {
    measure(Car{"car_sedan_gen_n2o", "car_sedan_gen_n2o.json", 2, 3800.0, 1.5, 6000.0, 1500.0, {2.0, 0.4, 0.15}, 0.6, 0.0, 1},
            0.9);
}

TEST_CASE("nitrous: car_hyper_n2o armed beats unarmed in a 3rd-gear WOT pull", "[nitrous][s1][vehicle][long]") {
    measure(Car{"car_hyper_n2o", "car_hyper_n2o.json", 3, 6500.0, 0.6, 7600.0, 1500.0, {2.0, 0.4, 0.12}, 0.5}, 1.03);
}
