// test_ev_part_upgrades.cpp - garage upgrades for electric drives (owner 2026-10-21: "Allow for upgrading of electric motors
// and battery size and battery power output on EVs"): the three scale options ev_motor_scale, ev_battery_capacity and
// ev_battery_power in data/vehicles/setup_options.json.
//
// Checks: the options are available on car_electric and not on a combustion car; a setup scales exactly the documented
// leaves of every motor and the battery (mass with torque, mass with capacity - [AS], see the options' help) and nothing
// else; the part-mass tree follows the new component masses; the stock files are untouched.
#include "rg/engine_store.h"
#include "rg/part_mass.h"
#include "rg/vehicle_setup.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
const std::string kRoot = RG_SOURCE_DIR;

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    std::shared_ptr<rg::EngineStore> store;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_ev_part_tests"};
    Fixture() {
        std::string err;
        auto c = rg::load_vehicle_catalog(kRoot + "/data/vehicles/catalog.json", kRoot, &err);
        INFO(err);
        REQUIRE(c.has_value());
        catalog = *c;
        auto t = rg::load_setup_options(kRoot + "/data/vehicles/setup_options.json", &err);
        INFO(err);
        REQUIRE(t.has_value());
        table = *t;
        ctx.tyre_families_path = kRoot + "/data/tyres/families.json";
        ctx.work_root = root.path();
        ctx.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
        store = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, "", &err);
        INFO(err);
        REQUIRE(store != nullptr);
        ctx.engines = std::make_shared<const rg::EngineStore>(*store);
        auto masses = rg::load_part_mass_table(kRoot + "/data/vehicles/part_masses.json", &err);
        INFO(err);
        REQUIRE(masses.has_value());
        ctx.part_masses = std::make_shared<const rg::PartMassTable>(*masses);
    }
    const rg::CatalogEntry& entry(const std::string& id) const {
        const rg::CatalogEntry* e = catalog.find(id);
        REQUIRE(e != nullptr);
        return *e;
    }
};

rg::VehicleSetup setup_of(const std::string& vehicle, std::vector<std::pair<std::string, rg::SetupValue>> values) {
    rg::VehicleSetup s;
    s.vehicle_id = vehicle;
    for (auto& kv : values) s.values[kv.first] = kv.second;
    return s;
}

const rg::OptionView* view_of(const rg::SetupModel& m, const std::string& id) {
    for (const rg::OptionView& v : m.options) if (v.def.id == id) return &v;
    return nullptr;
}

nlohmann::json read_json(const std::string& path) {
    std::ifstream in(path);
    REQUIRE(in.good());
    return nlohmann::json::parse(in);
}

const nlohmann::json& component(const nlohmann::json& vehicle, const std::string& id) {
    for (const auto& c : vehicle.at("powertrain").at("components")) if (c.value("id", "") == id) return c;
    FAIL("no component " << id);
    static const nlohmann::json none;
    return none;
}

const char* const kEvOptions[] = {"ev_motor_scale", "ev_battery_capacity", "ev_battery_power"};

} // namespace

TEST_CASE("ev parts: the options are available on the electric car only", "[setup][garage][ev]") {
    Fixture f;
    const rg::SetupModel ev = rg::build_setup_model(f.entry("car_electric"), f.table, f.ctx);
    const rg::SetupModel sedan = rg::build_setup_model(f.entry("car_sedan"), f.table, f.ctx);
    REQUIRE(ev.error.empty());
    REQUIRE(sedan.error.empty());
    for (const char* id : kEvOptions) {
        INFO(id);
        const rg::OptionView* v = view_of(ev, id);
        REQUIRE(v != nullptr);
        CHECK(v->available);
        CHECK(v->def.group == "Electric drive");
        const rg::OptionView* s = view_of(sedan, id);
        if (s != nullptr) CHECK_FALSE(s->available); // no motor, no battery: the pointers match nothing
    }
}

TEST_CASE("ev parts: a setup scales the motors and the battery, and the mass follows", "[setup][garage][ev][part_mass]") {
    Fixture f;
    const rg::CatalogEntry& e = f.entry("car_electric");
    rg::ScopedWorkDir d0(f.ctx.work_root, "stock"), d1(f.ctx.work_root, "up");
    const rg::MaterialisedSetup m0 = rg::materialise_setup(e, f.table, f.ctx, setup_of("car_electric", {}), d0.path());
    const rg::MaterialisedSetup m1 = rg::materialise_setup(
        e, f.table, f.ctx,
        setup_of("car_electric", {{"ev_motor_scale", 1.5}, {"ev_battery_capacity", 1.2}, {"ev_battery_power", 2.0}}),
        d1.path());
    INFO(m0.error);
    INFO(m1.error);
    REQUIRE(m0.ok);
    REQUIRE(m1.ok);
    REQUIRE(m1.vehicle_path != m0.vehicle_path); // the setup writes its own vehicle file; the stock one stays
    const nlohmann::json v0 = read_json(m0.vehicle_path);
    const nlohmann::json v1 = read_json(m1.vehicle_path);
    CHECK(read_json(e.vehicle_path) == v0);

    for (const char* motor : {"front_motor", "rear_motor"}) {
        INFO(motor);
        const nlohmann::json& a = component(v0, motor);
        const nlohmann::json& b = component(v1, motor);
        for (const char* key : {"peak_torque_nm", "max_power_w", "regen_torque_nm", "mass_kg"}) {
            INFO(key);
            CHECK(b.at(key).get<double>() == Approx(a.at(key).get<double>() * 1.5).epsilon(1e-9));
        }
        // speed range, inertia and efficiency are the same machine design: unchanged
        for (const char* key : {"max_rpm", "inertia", "efficiency"}) {
            INFO(key);
            if (a.contains(key)) CHECK(b.at(key) == a.at(key));
        }
    }
    const nlohmann::json& b0 = component(v0, "battery");
    const nlohmann::json& b1 = component(v1, "battery");
    CHECK(b1.at("capacity_kwh").get<double>() == Approx(b0.at("capacity_kwh").get<double>() * 1.2).epsilon(1e-9));
    CHECK(b1.at("mass_kg").get<double>() == Approx(b0.at("mass_kg").get<double>() * 1.2).epsilon(1e-9));
    CHECK(b1.at("max_discharge_w").get<double>() == Approx(b0.at("max_discharge_w").get<double>() * 2.0).epsilon(1e-9));
    CHECK(b1.at("max_charge_w") == b0.at("max_charge_w"));
    CHECK(b1.at("initial_soc") == b0.at("initial_soc"));

    // part masses: the battery and motor nodes carry the new masses, the car total grows by exactly the difference
    REQUIRE(m0.compiled.parts.ok);
    REQUIRE(m1.compiled.parts.ok);
    double added = (b1.at("mass_kg").get<double>() - b0.at("mass_kg").get<double>());
    for (const char* motor : {"front_motor", "rear_motor"})
        added += component(v1, motor).at("mass_kg").get<double>() - component(v0, motor).at("mass_kg").get<double>();
    CHECK(added > 100.0);
    const rg::PartNode* bat0 = m0.compiled.parts.root.find("drivetrain.battery");
    const rg::PartNode* bat1 = m1.compiled.parts.root.find("drivetrain.battery");
    REQUIRE(bat0 != nullptr);
    REQUIRE(bat1 != nullptr);
    CHECK(bat1->total_kg() - bat0->total_kg() == Approx(b1.at("mass_kg").get<double>() - b0.at("mass_kg").get<double>()));
    CHECK(m1.compiled.parts.root.total_kg() - m0.compiled.parts.root.total_kg() == Approx(added).epsilon(1e-9));
}
