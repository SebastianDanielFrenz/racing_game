// test_dashboard_info.cpp - rg/dashboard_info.h: the battery readout and the tachometer layout of EV / hybrid cars.
// rg_unit_tests "[dashboard_info]"
#include "rg/dashboard_info.h"

#include "ps/drivetrain/powertrain_desc.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <string>

namespace pd = ps::drivetrain;

namespace {
ps::vehicle::VehicleDesc hybrid_desc(double motor_max_rpm, double reduction) {
    ps::vehicle::VehicleDesc d;
    pd::SimulatedEngineDesc engine;
    engine.limiter.rpm = 8250;
    pd::ElectricMotorDesc motor;
    motor.max_rpm = motor_max_rpm;
    pd::ElectricMotorDesc generator;
    generator.max_rpm = 12000;
    generator.crank_assist = true;
    pd::GearPairDesc gear;
    gear.ratio = reduction;
    d.powertrain.components.push_back({"engine", engine});
    d.powertrain.components.push_back({"conversion_motor", motor});
    d.powertrain.components.push_back({"conversion_generator", generator});
    d.powertrain.components.push_back({"conversion_reduction", gear});
    pd::ShaftDesc s;
    s.ports = {{"conversion_motor", "shaft", false}, {"conversion_reduction", "in", false}};
    d.powertrain.shafts.push_back(s);
    return d;
}
}  // namespace

TEST_CASE("dashboard config: defaults, overrides and refusals", "[dashboard_info]") {
    std::string err;
    auto c = rg::parse_dashboard_config("{}", &err);
    REQUIRE(c.has_value());
    CHECK(c->battery_low_soc == 0.20);
    CHECK(c->motor_dial_tick_krpm == 2.0);
    c = rg::parse_dashboard_config(R"({"battery_low_soc":0.3,"battery_critical_soc":0.15,"motor_dial_max_krpm":16})", &err);
    REQUIRE(c.has_value());
    CHECK(c->battery_low_soc == 0.3);
    CHECK(c->motor_dial_max_krpm == 16);
    CHECK_FALSE(rg::parse_dashboard_config(R"({"battery_low_soc":"x"})", &err).has_value());
    CHECK_FALSE(rg::parse_dashboard_config(R"({"battery_low_soc":0.1,"battery_critical_soc":0.2})", &err).has_value());
    CHECK_FALSE(rg::parse_dashboard_config(R"({"motor_dial_tick_krpm":0})", &err).has_value());
    CHECK_FALSE(rg::parse_dashboard_config("[1]", &err).has_value());
}

TEST_CASE("battery_hud: percent, label and level; nothing without a battery", "[dashboard_info]") {
    pd::PowertrainSnapshot p;
    CHECK_FALSE(rg::battery_hud(p).fitted);
    CHECK(rg::battery_hud(p).label.empty());
    p.battery_capacity_kwh = 20;
    p.battery_soc = 0.254;
    auto h = rg::battery_hud(p);
    CHECK(h.fitted);
    CHECK(h.percent == 25);
    CHECK(h.label == "SoC 25%");
    CHECK(h.level == 0);
    p.battery_soc = 0.19;
    CHECK(rg::battery_hud(p).level == 1);
    p.battery_soc = 0.05;
    CHECK(rg::battery_hud(p).level == 2);
    p.battery_soc = 1.7; // clamped
    CHECK(rg::battery_hud(p).percent == 100);
    rg::DashboardConfig cfg;
    cfg.battery_low_soc = 0.5;
    p.battery_soc = 0.4;
    CHECK(rg::battery_hud(p, cfg).level == 1); // the threshold is a config value
}

TEST_CASE("tach_scales: ICE unchanged, hybrid gets an inner motor scale from the motor's own max rpm", "[dashboard_info]") {
    ps::vehicle::VehicleDesc ice;
    pd::SimulatedEngineDesc engine;
    engine.limiter.rpm = 8250;
    ice.powertrain.components.push_back({"engine", engine});
    CHECK(rg::tach_scales(ice).kind == rg::TachKind::Ice);

    const auto hybrid = rg::tach_scales(hybrid_desc(20000, 2.2));
    CHECK(hybrid.kind == rg::TachKind::Hybrid);
    CHECK(hybrid.ice_limiter_rpm == 8250);
    CHECK(hybrid.motor_limit_rpm == 20000);
    CHECK(hybrid.motor_dial_max_rpm == 20000);
    CHECK(hybrid.motor_dial_tick_rpm == 2000);
    CHECK(hybrid.traction_ratio == 2.2);

    // Not hard-coded to 20k: a 15 500 rpm motor rounds up to the next tick (16), and the config can override it.
    CHECK(rg::tach_scales(hybrid_desc(15500, 2.2)).motor_dial_max_rpm == 16000);
    rg::DashboardConfig cfg;
    cfg.motor_dial_max_krpm = 12;
    CHECK(rg::tach_scales(hybrid_desc(20000, 2.2), cfg).motor_dial_max_rpm == 12000);

    ps::vehicle::VehicleDesc ev;
    pd::ElectricMotorDesc motor;
    motor.max_rpm = 18000;
    ev.powertrain.components.push_back({"m", motor});
    const auto e = rg::tach_scales(ev);
    CHECK(e.kind == rg::TachKind::Ev);
    CHECK(e.ice_limiter_rpm == 0);
    CHECK(e.motor_dial_max_rpm == 18000);
}

TEST_CASE("traction_motor_rpm: gearbox output times the reduction, not the max over all machines", "[dashboard_info]") {
    const auto s = rg::tach_scales(hybrid_desc(20000, 2.2));
    pd::PowertrainSnapshot p;
    p.motor_rpm = 3300; // the crank generator following the engine
    p.driveline.gearbox_out_omega = 100.0; // rad/s
    CHECK_THAT(rg::traction_motor_rpm(p, s), Catch::Matchers::WithinRel(100.0 * 2.2 * 30.0 / 3.14159265358979, 1e-9));
    p.driveline.gearbox_out_omega = -100.0; // reverse
    CHECK(rg::traction_motor_rpm(p, s) > 2000.0);
    rg::TachScales unknown = s;
    unknown.traction_ratio = 0;
    CHECK(rg::traction_motor_rpm(p, unknown) == 3300.0); // no ratio known: the snapshot's value
}
