// test_vehicle_catalog.cpp - the R6 vehicle catalog (data/vehicles/catalog.json):
// every entry's files exist, the stats vehicle select shows come from data, and
// every entry loads with ps::io::load_vehicle_json.
#include "rg/vehicle_catalog.h"

#include "ps/io/vehicle_io.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <string>

namespace {
const std::string kRoot = RG_SOURCE_DIR;

rg::VehicleCatalog load_catalog() {
    std::string err;
    auto catalog = rg::load_vehicle_catalog(kRoot + "/data/vehicles/catalog.json", kRoot, &err);
    INFO(err);
    REQUIRE(catalog.has_value());
    return *catalog;
}
} // namespace

TEST_CASE("catalog: the shipped catalog loads and names its default", "[catalog][vehicle_select]") {
    const rg::VehicleCatalog c = load_catalog();
    REQUIRE(c.entries.size() >= 4);
    REQUIRE(c.find(c.default_id) != nullptr);
    REQUIRE(c.find("car_hyper") != nullptr);
    REQUIRE(c.find("car_sedan") != nullptr);
    REQUIRE(c.find("nope") == nullptr);
    for (const auto& e : c.entries) {
        INFO(e.id);
        CHECK(!e.title.empty());
        CHECK(e.chassis.mass_kg > 100.0);
        CHECK(!e.sim_name.empty());
    }
}

TEST_CASE("catalog: stats come from the data files", "[catalog][vehicle_select]") {
    const rg::VehicleCatalog c = load_catalog();
    std::string err;

    const auto sedan = rg::compute_vehicle_stats(*c.find("car_sedan"), &err);
    INFO(err);
    REQUIRE(sedan.has_value());
    CHECK(sedan->layout == "FWD");
    CHECK(sedan->driven_wheels == 2);
    CHECK(sedan->wheel_count == 4);
    CHECK(sedan->gear_count == 6);
    CHECK(sedan->mass_kg == 1500.0);
    CHECK(!sedan->engine_figures_declared); // a torque-map engine: read from its WOT curve
    CHECK(sedan->peak_torque_nm > 250.0);
    CHECK(sedan->peak_torque_nm < 400.0);
    CHECK(sedan->peak_power_kw > 100.0);
    // power is torque * speed of the same curve point: never above peak torque * the highest rpm
    CHECK(sedan->peak_power_kw < sedan->peak_torque_nm * 8000.0 * 3.14159265 / 30.0 / 1000.0);

    CHECK(rg::compute_vehicle_stats(*c.find("car_sedan_rwd"), &err)->layout == "RWD");
    CHECK(rg::compute_vehicle_stats(*c.find("car_sedan_awd"), &err)->layout == "AWD");
    CHECK(rg::compute_vehicle_stats(*c.find("car_sedan_awd"), &err)->driven_wheels == 4);

    const auto hyper = rg::compute_vehicle_stats(*c.find("car_hyper"), &err);
    REQUIRE(hyper.has_value());
    CHECK(hyper->layout == "RWD");
    CHECK(hyper->gear_count == 7);
    CHECK(hyper->engine_figures_declared); // a simulated engine: the entry declares the cited figures
    CHECK(hyper->peak_torque_nm == 1371.0);
}

TEST_CASE("catalog: every entry loads with the physics vehicle loader", "[catalog][vehicle_select]") {
    const rg::VehicleCatalog c = load_catalog();
    ps::io::EngineMapOptions options;
    options.cache_dir = kRoot + "/out/godot_engine_cache";
    for (const auto& e : c.entries) {
        INFO(e.id);
        const ps::vehicle::VehicleDesc desc = ps::io::load_vehicle_json(e.vehicle_path, options);
        CHECK(desc.name == e.sim_name);
        CHECK(desc.wheels.size() == 4);
    }
}

TEST_CASE("catalog: malformed catalogs are rejected with a reason", "[catalog][vehicle_select]") {
    std::string err;
    CHECK(!rg::parse_vehicle_catalog("not json", kRoot, "t", &err).has_value());
    CHECK(!rg::parse_vehicle_catalog(R"({"format":"x"})", kRoot, "t", &err).has_value());
    CHECK_THAT(err, Catch::Matchers::ContainsSubstring("format"));
    CHECK(!rg::parse_vehicle_catalog(R"({"format":"rg.vehicle_catalog/1","vehicles":[]})", kRoot, "t", &err).has_value());
    const char* missing_chassis = R"({"format":"rg.vehicle_catalog/1","vehicles":[
        {"id":"a","title":"A","vehicle":"data/vehicles/car_sedan.json","model":"external/physics_sim/data/models/car_sedan/car_sedan.glb"}]})";
    CHECK(!rg::parse_vehicle_catalog(missing_chassis, kRoot, "t", &err).has_value());
    CHECK_THAT(err, Catch::Matchers::ContainsSubstring("chassis"));
    const char* missing_file = R"({"format":"rg.vehicle_catalog/1","vehicles":[
        {"id":"a","title":"A","vehicle":"data/vehicles/none.json","model":"x.glb",
         "chassis":{"mass_kg":1000,"half_extents":[2,0.4,0.15],"spawn_z_m":0.6}}]})";
    CHECK(!rg::parse_vehicle_catalog(missing_file, kRoot, "t", &err).has_value());
}
