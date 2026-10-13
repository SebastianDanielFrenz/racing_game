// test_engine_thermal_info.cpp - the engine temperature dashboard state (rg::engine_thermal_info / engine_thermal_hud,
// what RgSimulation.get_vehicle_thermal forwards). A thermal car (the game's own data/vehicles/car_sedan.json: every game
// car has the thermal model; physics_sim no longer ships a thermal reference vehicle in data/vehicles, its
// car_sedan_thermal.json moved to its test fixtures) reports present with coolant/oil/gauge and the normal / hot /
// overheat / cold level; a car without a thermal model (physics_sim's data/vehicles/car_sedan.json, the only shipped car
// without a cooling system) reports present = false and no value. The game cars as a set are covered by
// test_game_car_thermal.cpp.
#include "rg/engine_thermal_info.h"
#include "rg/session.h"

#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>

namespace {

const std::string kRepo = RG_SOURCE_DIR;

rg::SessionConfig config_for(const std::string& vehicle_path) {
    rg::SessionConfig config;
    config.vehicle_json_path = vehicle_path;
    config.surface_table_path = kRepo + "/external/physics_sim/data/surfaces/surfaces.json";
    return config;
}

} // namespace

TEST_CASE("engine thermal info: a thermal car reports coolant, oil, gauge and a level", "[thermal][dashboard][vehicle]") {
    rg::Session session(config_for(kRepo + "/data/vehicles/car_sedan.json"));
    const rg::EngineThermalInfo info = rg::engine_thermal_info(session.vehicle_desc());
    REQUIRE(info.present);
    // The thresholds come from the car's cooling system (cooling_systems/sedan.json -> class_passenger_car.json): fan on 98 C, overheat 118 C, cold 50 C.
    CHECK(info.hot_c == Catch::Approx(98.0).margin(1e-6));
    CHECK(info.overheat_c == Catch::Approx(118.0).margin(1e-6));
    CHECK(info.cold_c == Catch::Approx(50.0).margin(1e-6));
    CHECK(info.fan_off_c < info.fan_on_c);

    for (int i = 0; i < 240; ++i) session.step();
    const auto hud = [&] {
        return rg::engine_thermal_hud(info, session.world().powertrain_state(session.vehicle_id()));
    };
    const rg::EngineThermalHud warm = hud();
    REQUIRE(warm.present);
    CHECK(warm.coolant_c > 80.0); // starts warm: coolant 91 C, oil 95 C
    CHECK(warm.coolant_c < 98.0);
    CHECK(warm.oil_c > 85.0);
    CHECK(warm.gauge >= 0.0);
    CHECK(warm.gauge <= 1.0);
    CHECK(warm.level == rg::ThermalLevel::Normal);

    const auto level_at = [&](double coolant_c) {
        session.world().set_engine_thermal_state(session.vehicle_id(), {coolant_c, coolant_c});
        for (int i = 0; i < 3; ++i) session.step();
        return hud();
    };
    CHECK(level_at(105.0).level == rg::ThermalLevel::Hot);
    const rg::EngineThermalHud over = level_at(121.0);
    CHECK(over.level == rg::ThermalLevel::Overheat);
    CHECK(over.coolant_c == Catch::Approx(121.0).margin(2.0));
    CHECK(level_at(30.0).level == rg::ThermalLevel::Cold);
}

TEST_CASE("engine thermal info: a car without a thermal model hides the gauge", "[thermal][dashboard][vehicle]") {
    rg::Session session(config_for(kRepo + "/external/physics_sim/data/vehicles/car_sedan.json"));
    const rg::EngineThermalInfo info = rg::engine_thermal_info(session.vehicle_desc());
    CHECK_FALSE(info.present);
    for (int i = 0; i < 120; ++i) session.step();
    const rg::EngineThermalHud h = rg::engine_thermal_hud(info, session.world().powertrain_state(session.vehicle_id()));
    CHECK_FALSE(h.present);
    CHECK(h.coolant_c == 0.0);
    CHECK(h.oil_c == 0.0);
}

TEST_CASE("engine thermal info: the level thresholds and the data fallbacks", "[thermal][dashboard]") {
    rg::EngineThermalInfo info;
    info.present = true;
    info.cold_c = 50.0;
    info.hot_c = 98.0;
    info.overheat_c = 118.0;
    CHECK(rg::thermal_level(info, 49.9) == rg::ThermalLevel::Cold);
    CHECK(rg::thermal_level(info, 50.0) == rg::ThermalLevel::Normal);
    CHECK(rg::thermal_level(info, 97.9) == rg::ThermalLevel::Normal);
    CHECK(rg::thermal_level(info, 98.0) == rg::ThermalLevel::Hot);
    CHECK(rg::thermal_level(info, 118.0) == rg::ThermalLevel::Overheat);
    CHECK(rg::thermal_level(info, 90.0, true) == rg::ThermalLevel::Overheat); // the physics warning bit wins
    info.cold_c = 0.0; // no cold threshold in the data: never Cold
    CHECK(rg::thermal_level(info, -10.0) == rg::ThermalLevel::Normal);
    CHECK(rg::thermal_level_name(rg::ThermalLevel::Hot) == "hot");
}
