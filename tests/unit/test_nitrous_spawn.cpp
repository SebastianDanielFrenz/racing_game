// test_nitrous_spawn.cpp - S1: both nitrous catalog cars (car_sedan_gen_n2o, car_hyper_n2o) are catalog entries that load,
// spawn in a real rg::Session (the catalog's chassis proxy, the vehicle file the entry names), carry a kit, and the
// HUD data (rg::nitrous_hud, what RgSimulation.get_vehicle_nitrous forwards) shows the nitrous state: "off" until the
// arm channel is set, "ARMED" after, with the bottle full. No power measurement here (the owner reads that in game).
#include "rg/nitrous_info.h"
#include "rg/session.h"
#include "rg/vehicle_catalog.h"

#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>

namespace {

const std::string kRepo = RG_SOURCE_DIR;

void check_nitrous_car(const std::string& id) {
    INFO("catalog id " << id);
    std::string err;
    const auto catalog = rg::load_vehicle_catalog(kRepo + "/data/vehicles/catalog.json", kRepo, &err);
    INFO(err);
    REQUIRE(catalog.has_value());
    const rg::CatalogEntry* entry = catalog->find(id);
    REQUIRE(entry != nullptr);
    CHECK(entry->preset_of.empty());

    rg::SessionConfig config;
    config.vehicle_json_path = entry->vehicle_path;
    config.surface_table_path = kRepo + "/data/surfaces/surfaces.json";
    config.chassis_mass_kg = entry->chassis.mass_kg;
    config.chassis_half_extents = ps::Vec3{entry->chassis.half_extents[0], entry->chassis.half_extents[1],
                                           entry->chassis.half_extents[2]};
    config.chassis_z_m = entry->chassis.spawn_z_m;
    rg::Session session(config);

    const rg::NitrousInfo info = rg::nitrous_info(session.vehicle_desc());
    REQUIRE(info.present);
    CHECK(info.bottle_capacity_kg > 1.0);

    for (int i = 0; i < 240; ++i) session.step(); // spawned and stepping
    const auto hud = [&] {
        return rg::nitrous_hud(info, session.world().powertrain_state(session.vehicle_id()),
                               session.world().get_control("nitrous_arm"));
    };
    const rg::NitrousHud off = hud();
    CHECK(off.present);
    CHECK_FALSE(off.armed);
    CHECK_FALSE(off.spraying);
    CHECK(off.bottle_kg == Catch::Approx(info.bottle_initial_kg).epsilon(0.02));
    CHECK(rg::nitrous_state_label(off.present, off.armed, off.spraying, off.safety_cut, off.purging) == "off");

    session.world().set_control("nitrous_arm", 1.0); // pad Y / key N toggles this channel
    for (int i = 0; i < 240; ++i) session.step();
    const rg::NitrousHud armed = hud();
    CHECK(armed.armed);
    CHECK_FALSE(armed.spraying); // idle, no throttle: the controller does not spray
    CHECK(rg::nitrous_state_label(armed.present, armed.armed, armed.spraying, armed.safety_cut, armed.purging) == "ARMED");
}

} // namespace

TEST_CASE("nitrous: the sedan N2O catalog car spawns and the HUD shows off then ARMED", "[nitrous][s1][vehicle]") {
    check_nitrous_car("car_sedan_gen_n2o");
}

TEST_CASE("nitrous: the hypercar N2O catalog car spawns and the HUD shows off then ARMED", "[nitrous][s1][vehicle]") {
    check_nitrous_car("car_hyper_n2o");
}
