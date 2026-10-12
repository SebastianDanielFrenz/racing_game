// test_engine_garage.cpp - engine garage G1 (out/engine_garage_design.md 4, 15.2): the engine catalog and store,
// the swap derivation, the fit seam, the fuel choice and the chassis-mass delta, all through the real physics loader.
#include "rg/vehicle_load.h"
#include "rg/engine_store.h"
#include "rg/engine_swap.h"
#include "rg/garage.h"
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
const std::string kRoot = RG_SOURCE_DIR;

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    std::shared_ptr<rg::EngineStore> store;
    rg::ScopedWorkDir isolated_root{fs::temp_directory_path().generic_string(), "rg_g1_unit_tests"};
    explicit Fixture(const std::string& user_dir = "") {
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
        ctx.work_root = isolated_root.path();
        ctx.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
        store = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, user_dir, &err);
        INFO(err);
        REQUIRE(store != nullptr);
        refresh();
    }
    void refresh() { ctx.engines = std::make_shared<const rg::EngineStore>(*store); }
    const rg::CatalogEntry& entry(const std::string& id) const {
        const rg::CatalogEntry* e = catalog.find(id);
        REQUIRE(e != nullptr);
        return *e;
    }
    std::vector<const rg::CatalogEntry*> base_cars() const {
        std::vector<const rg::CatalogEntry*> out;
        for (const auto& e : catalog.entries) if (e.preset_of.empty()) out.push_back(&e);
        return out;
    }
};

rg::VehicleSetup setup_of(const std::string& vehicle, std::initializer_list<std::pair<const char*, rg::SetupValue>> values) {
    rg::VehicleSetup s;
    s.vehicle_id = vehicle;
    for (const auto& kv : values) s.values[kv.first] = kv.second;
    return s;
}

// Materialise and load with the physics loader WITHOUT generating engine maps (validate-only, milliseconds).
struct Loaded {
    bool ok = false;
    std::string error;
    ps::vehicle::VehicleDesc desc;
    rg::CompiledSetup compiled;
};
Loaded load_setup(const Fixture& f, const rg::CatalogEntry& e, const rg::VehicleSetup& s) {
    Loaded out;
    rg::ScopedWorkDir dir(f.ctx.work_root, "t_" + e.id);
    const rg::MaterialisedSetup m = rg::materialise_setup(e, f.table, f.ctx, s, dir.path());
    out.compiled = m.compiled;
    if (!m.ok) {
        out.error = m.error;
        return out;
    }
    ps::io::VehicleLoadOptions options;
    options.maps.generate = false;
    try {
        out.desc = rg::load_vehicle(m.vehicle_path, options);
        out.ok = true;
    } catch (const std::exception& ex) {
        out.error = ex.what();
    }
    return out;
}

struct EngineView {
    bool found = false;
    bool simulated = false;
    double idle = 0.0, limiter = 0.0;
    std::vector<std::string> fuels;
};
EngineView engine_of(const ps::vehicle::VehicleDesc& d) {
    EngineView v;
    for (const auto& c : d.powertrain.components) {
        if (const auto* p = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&c.params)) {
            v.found = true; v.idle = p->idle_rpm; v.limiter = p->limiter.rpm;
            for (const auto& fu : p->fuels) v.fuels.push_back(fu.name);
        }
        if (const auto* p = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) {
            v.found = true; v.simulated = true; v.idle = p->idle_rpm; v.limiter = p->limiter.rpm;
            for (const auto& fu : p->fuels) v.fuels.push_back(fu.name);
        }
    }
    return v;
}
std::string tank_fuel_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* t = std::get_if<ps::drivetrain::FuelTankDesc>(&c.params)) return t->fuel.name;
    }
    return {};
}
double clutch_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* t = std::get_if<ps::drivetrain::ClutchDesc>(&c.params)) return t->capacity_nm;
    }
    return 0.0;
}
const ps::drivetrain::ManualTcuDesc* tcu_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.controllers) {
        if (const auto* t = std::get_if<ps::drivetrain::ManualTcuDesc>(&c.params)) return t;
    }
    return nullptr;
}

rg::SwapEngine swap_engine_of(const rg::EngineStore& store, const rg::EngineEntry& e) {
    std::string err;
    const auto facts = rg::read_engine_facts(e.file, &err);
    INFO(err);
    REQUIRE(facts.has_value());
    rg::SwapEngine s;
    s.name = e.name;
    s.mass_kg = e.mass_kg;
    s.peak_torque_nm = store.peaks(e).torque_nm;
    s.idle_rpm = facts->idle_rpm;
    s.limiter_rpm = facts->limiter_rpm;
    s.simulated = facts->simulated;
    s.mechanical_throttle = facts->mechanical_throttle;
    for (const auto& fu : facts->fuels) s.fuels.push_back(fu.name);
    return s;
}

struct TempDir {
    std::string path;
    TempDir() {
        static int counter = 0;
        const int n = ++counter;
        path = (fs::temp_directory_path() / ("rg_g1_user_" + std::to_string(n))).generic_string();
        std::error_code ec;
        fs::remove_all(path, ec);
        fs::create_directories(path, ec);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
};

} // namespace

TEST_CASE("engine catalog: every row matches the file it names", "[engine_garage][engine_store]") {
    Fixture f;
    const auto& list = f.store->list();
    REQUIRE(list.size() >= 6);
    for (const rg::EngineEntry& e : list) {
        INFO(e.key);
        REQUIRE_FALSE(e.user);
        std::string err;
        const auto facts = rg::read_engine_facts(e.file, &err);
        INFO(err);
        REQUIRE(facts.has_value());
        // class <-> file kind: a curve engine is a torque map, mapped/cycle engines are simulated
        CHECK((e.cls == rg::EngineClass::Curve) == !facts->simulated);
        // the catalog mass is the file's own when the file carries one
        if (facts->mass_kg > 0.0) CHECK(e.mass_kg == Approx(facts->mass_kg));
        CHECK(e.mass_kg > 0.0);
        CHECK_FALSE(e.mass_source.empty());
        // every fuel the engine lists has a fuel file in the fuel directory
        REQUIRE_FALSE(facts->fuels.empty());
        for (const auto& fu : facts->fuels) CHECK(fs::is_regular_file(fs::path(f.store->fuel_file(fu.name))));
        // peak figures exist for every class
        const rg::EnginePeaks p = f.store->peaks(e);
        CHECK(p.known);
        CHECK(p.torque_nm > 0.0);
        CHECK(p.power_kw > 0.0);
        if (e.cls == rg::EngineClass::Curve) CHECK(p.source == "wot_curve");
        else CHECK(p.source == "catalog");
        CHECK(facts->limiter_rpm > facts->idle_rpm);
    }
    // The legacy ids of the first engine_install option resolve to their defaults.
    REQUIRE(f.store->find("sedan_i4") != nullptr);
    CHECK(f.store->find("sedan_i4")->key == "default:i4_2l_na");
    REQUIRE(f.store->find("sedan_i6") != nullptr);
    CHECK(f.store->find("sedan_i6")->key == "default:n52b30_3l_na");
    CHECK(f.store->find("hyper_v8_one1_audio") == f.store->find("default:hyper_v8_one1_audio"));
    CHECK(f.store->find("no_such_engine") == nullptr);
    CHECK(f.store->find("user:e9999") == nullptr);
}

TEST_CASE("engine catalog: the loader rejects a malformed table", "[engine_garage][engine_store]") {
    TempDir t;
    const auto write = [&](const std::string& name, const std::string& text) {
        std::ofstream(t.path + "/" + name, std::ios::binary) << text;
        return t.path + "/" + name;
    };
    std::string err;
    CHECK(rg::EngineStore::open(write("a.json", "{\"format\":\"rg.engine_catalog/2\",\"engines\":[]}"), kRoot, "", &err) == nullptr);
    CHECK_THAT(err, ContainsSubstring("rg.engine_catalog/1"));
    // a cycle engine without measured peaks
    const std::string row = R"({"id":"x","name":"X","file":"data/engines/hyper_v8_one1_audio.json","class":"cycle","mass_kg":100,"mass_source":"s"})";
    CHECK(rg::EngineStore::open(write("b.json", std::string("{\"format\":\"rg.engine_catalog/1\",\"fuel_dir\":\"external/physics_sim/data/fuels\",\"engines\":[") + row + "]}"),
                                kRoot, "", &err) == nullptr);
    CHECK_THAT(err, ContainsSubstring("peaks"));
    // a missing mass
    const std::string no_mass = R"({"id":"x","name":"X","file":"data/engines/n52b30_3l_na.json","class":"curve"})";
    CHECK(rg::EngineStore::open(write("c.json", std::string("{\"format\":\"rg.engine_catalog/1\",\"fuel_dir\":\"external/physics_sim/data/fuels\",\"engines\":[") + no_mass + "]}"),
                                kRoot, "", &err) == nullptr);
    CHECK_THAT(err, ContainsSubstring("mass_kg"));
}

TEST_CASE("engine swap: every catalog engine loads on every base car", "[engine_garage][engine_swap]") {
    Fixture f;
    const auto cars = f.base_cars();
    REQUIRE(cars.size() >= 6);
    int combinations = 0;
    for (const rg::CatalogEntry* car : cars) {
        for (const rg::EngineEntry& engine : f.store->list()) {
            INFO(car->id << " + " << engine.key);
            const auto setup = setup_of(car->id, {{"engine_install", engine.key}});
            const Loaded l = load_setup(f, *car, setup);
            INFO(l.error);
            REQUIRE(l.ok);
            ++combinations;
            const EngineView v = engine_of(l.desc);
            REQUIRE(v.found);
            std::string err;
            const auto facts = rg::read_engine_facts(engine.file, &err);
            REQUIRE(facts.has_value());
            CHECK(v.limiter == Approx(facts->limiter_rpm));
            // the tank holds a fuel the engine lists
            const std::string fuel = tank_fuel_of(l.desc);
            REQUIRE_FALSE(fuel.empty());
            CHECK(std::find(v.fuels.begin(), v.fuels.end(), fuel) != v.fuels.end());
            // the clutch holds the engine (1.5 x peak, rounded up to 10 N*m) unless the car's own is stronger
            const double peak = f.store->peaks(engine).torque_nm;
            if (!l.compiled.swap_lines.empty()) { // the car's own engine changes nothing, its clutch stays
                CHECK(clutch_of(l.desc) >= std::ceil(1.5 * peak / 10.0) * 10.0 - 1e-9);
            }
            // a nitrous kit only stays on a simulated engine
            bool has_kit = false;
            for (const auto& c : l.desc.powertrain.components) {
                if (std::holds_alternative<ps::drivetrain::NitrousKitDesc>(c.params)) has_kit = true;
            }
            if (!v.simulated) CHECK_FALSE(has_kit);
        }
    }
    CHECK(combinations == static_cast<int>(cars.size() * f.store->list().size()));
}

TEST_CASE("engine swap: the car's own engine changes nothing", "[engine_garage][engine_swap]") {
    Fixture f;
    const auto& car = f.entry("car_sedan");
    const Loaded stock = load_setup(f, car, setup_of("car_sedan", {}));
    const Loaded own = load_setup(f, car, setup_of("car_sedan", {{"engine_install", std::string("default:n52b30_3l_na")}}));
    REQUIRE(stock.ok);
    REQUIRE(own.ok);
    CHECK(own.compiled.chassis_mass_delta_kg == 0.0);
    CHECK(own.compiled.swap_lines.empty());
    CHECK(own.compiled.vehicle_patch == "{}");
    CHECK(clutch_of(own.desc) == clutch_of(stock.desc));
    // a legacy id of the car's own engine is the same thing
    const Loaded legacy = load_setup(f, car, setup_of("car_sedan", {{"engine_install", std::string("sedan_i6")}}));
    REQUIRE(legacy.ok);
    CHECK(legacy.compiled.vehicle_patch == "{}");
}

TEST_CASE("engine swap: the engine thermal model follows the engine (a thermal car loses its cooling system with a no-thermal engine)",
          "[engine_garage][engine_swap][thermal]") {
    Fixture f;
    int with_thermal = 0, without_thermal = 0;
    for (const rg::CatalogEntry* car : f.base_cars()) {
        const Loaded stock = load_setup(f, *car, setup_of(car->id, {}));
        REQUIRE(stock.ok);
        REQUIRE(stock.desc.powertrain.thermal.has_value()); // every game car ships with the thermal model
        for (const rg::EngineEntry& engine : f.store->list()) {
            INFO(car->id << " + " << engine.key);
            std::string err;
            const auto facts = rg::read_engine_facts(engine.file, &err);
            INFO(err);
            REQUIRE(facts.has_value());
            const Loaded l = load_setup(f, *car, setup_of(car->id, {{"engine_install", engine.key}}));
            INFO(l.error);
            REQUIRE(l.ok); // without the drop the loader throws: a cooling_system needs the engine's default_thermal
            CHECK(l.desc.powertrain.thermal.has_value() == facts->has_thermal);
            const bool noted = std::any_of(l.compiled.swap_lines.begin(), l.compiled.swap_lines.end(),
                                           [](const std::string& s) { return s.find("temperature gauge") != std::string::npos; });
            const bool own = l.compiled.swap_lines.empty(); // the car's own engine changes nothing
            CHECK(noted == (!facts->has_thermal && !own));
            (facts->has_thermal ? with_thermal : without_thermal) += 1;
        }
    }
    // The catalog holds engines on both sides (demo_i4_pipes and r26b_4rotor have no thermal data at the time of writing).
    CHECK(with_thermal > 0);
    CHECK(without_thermal > 0);
}

TEST_CASE("engine swap: the fuel is kept when the new engine lists it, else it changes with a notice (D1)", "[engine_garage][engine_swap][fuel]") {
    Fixture f;
    const auto& sedan = f.entry("car_sedan");
    REQUIRE(tank_fuel_of(load_setup(f, sedan, setup_of("car_sedan", {})).desc) == "e5");
    {   // the 2.0 L I4 lists e5: kept, no fuel line
        const Loaded l = load_setup(f, sedan, setup_of("car_sedan", {{"engine_install", std::string("sedan_i4")}}));
        REQUIRE(l.ok);
        CHECK(tank_fuel_of(l.desc) == "e5");
        for (const auto& line : l.compiled.swap_lines) CHECK_THAT(line, !ContainsSubstring("Fuel changed"));
    }
    {   // the A25A lists tier3_e10_cert and e10 only: the reference fuel, with a notice
        const Loaded l = load_setup(f, sedan, setup_of("car_sedan", {{"engine_install", std::string("default:toyota_a25a_fks_gen")}}));
        REQUIRE(l.ok);
        CHECK(tank_fuel_of(l.desc) == "tier3_e10_cert");
        bool notice = false;
        for (const auto& line : l.compiled.swap_lines) if (line.find("Fuel changed") != std::string::npos) notice = true;
        CHECK(notice);
    }
    {   // a fuel the player picked (and the engine lists) wins
        const Loaded l = load_setup(f, sedan, setup_of("car_sedan", {{"engine_install", std::string("sedan_i4")}, {"fuel", std::string("e10")}}));
        REQUIRE(l.ok);
        CHECK(tank_fuel_of(l.desc) == "e10");
    }
    {   // ... and on the car's own engine too
        const Loaded l = load_setup(f, sedan, setup_of("car_sedan", {{"fuel", std::string("e10")}}));
        REQUIRE(l.ok);
        CHECK(tank_fuel_of(l.desc) == "e10");
    }
    {   // a fuel the engine does not list is refused
        const Loaded l = load_setup(f, sedan, setup_of("car_sedan", {{"fuel", std::string("diesel")}}));
        CHECK_FALSE(l.ok);
        CHECK_THAT(l.error, ContainsSubstring("not one of this engine's fuels"));
    }
    {   // the hyper's E5 map is reused when its own engine keeps its fuel (no regeneration): the tank fuel is e5
        const Loaded l = load_setup(f, f.entry("car_hyper"), setup_of("car_hyper", {}));
        REQUIRE(l.ok);
        CHECK(tank_fuel_of(l.desc) == "e5");
    }
}

TEST_CASE("engine swap: the TCU derivation satisfies the loader's ordering for every engine", "[engine_garage][engine_swap][tcu]") {
    Fixture f;
    for (const rg::EngineEntry& engine : f.store->list()) {
        INFO(engine.key);
        const rg::SwapEngine se = swap_engine_of(*f.store, engine);
        rg::SwapCar car;
        car.has_tank = true;
        car.tank_fuel = "e5";
        car.clutch_kinetic_nm = 300.0;
        car.stock_engine_mass_kg = 165.0;
        const rg::SwapPackage p = rg::derive_swap_package(car, se, {}, true);
        REQUIRE(p.ok);
        REQUIRE(p.set_tcu);
        const auto& ac = p.auto_clutch;
        CHECK(ac.anti_stall_rpm < ac.bite_rpm);
        CHECK(ac.bite_rpm < ac.full_rpm_min);
        CHECK(ac.full_rpm_min <= ac.full_rpm_max);
        CHECK(ac.rolling_reengage_rpm > ac.anti_stall_rpm);
        CHECK(ac.full_rpm_max <= se.limiter_rpm);
        const auto& as = p.auto_shift;
        CHECK(as.up_zero < as.up_full);
        CHECK(as.down_zero < as.down_full);
        CHECK(as.down_full < as.up_zero);
        CHECK(as.up_full < se.limiter_rpm);
        CHECK(as.down_zero > se.idle_rpm);
    }
    // golden numbers: the hyper V8 (idle 950, limiter 8250)
    rg::SwapEngine hyper = swap_engine_of(*f.store, *f.store->find("default:hyper_v8_one1_audio"));
    CHECK(hyper.idle_rpm == Approx(950.0));
    CHECK(hyper.limiter_rpm == Approx(8250.0));
    rg::SwapCar car;
    car.has_tank = true;
    car.tank_fuel = "e5";
    car.clutch_kinetic_nm = 300.0;
    const rg::SwapPackage p = rg::derive_swap_package(car, hyper, {}, true);
    CHECK(p.auto_clutch.bite_rpm == Approx(1250.0));
    CHECK(p.auto_clutch.anti_stall_rpm == Approx(850.0));
    CHECK(p.auto_clutch.rolling_reengage_rpm == Approx(1450.0));
    CHECK(p.auto_clutch.full_rpm_min == Approx(1750.0));
    CHECK(p.auto_clutch.full_rpm_max == Approx(3700.0));
    CHECK(p.auto_shift.up_full == Approx(7950.0));
    CHECK(p.clutch_kinetic_nm == Approx(2090.0)); // 1.5 x 1387.3 N*m rounded up to 10
    CHECK(p.set_clutch);
    // a stronger clutch of the car stays
    car.clutch_kinetic_nm = 3650.0;
    CHECK_FALSE(rg::derive_swap_package(car, hyper, {}, true).set_clutch);
}

TEST_CASE("engine swap: the derived controller values reach the loaded car", "[engine_garage][engine_swap][tcu]") {
    Fixture f;
    const Loaded l = load_setup(f, f.entry("car_sedan"), setup_of("car_sedan", {{"engine_install", std::string("default:hyper_v8_one1_audio")}}));
    REQUIRE(l.ok);
    const auto* tcu = tcu_of(l.desc);
    REQUIRE(tcu != nullptr);
    CHECK(tcu->auto_clutch.bite_rpm == Approx(1250.0));
    CHECK(tcu->auto_clutch.anti_stall_rpm == Approx(850.0));
    CHECK(tcu->auto_clutch.rolling_reengage_rpm == Approx(1450.0));
    CHECK(tcu->auto_clutch.full_rpm_max == Approx(3700.0));
    CHECK(tcu->auto_shift.up_rpm_at_full_throttle == Approx(7950.0));
    CHECK(clutch_of(l.desc) == Approx(2090.0));
    CHECK(engine_of(l.desc).limiter == Approx(8250.0));
}

TEST_CASE("engine swap: a mechanical-throttle engine zeroes the blip, a nitrous kit leaves with a torque-map engine", "[engine_garage][engine_swap]") {
    rg::SwapCar car;
    car.has_nitrous = true;
    car.has_tank = true;
    car.tank_fuel = "e5";
    car.has_slot_overrides = true;
    car.clutch_kinetic_nm = 100.0;
    rg::SwapEngine e;
    e.name = "Mech";
    e.idle_rpm = 700;
    e.limiter_rpm = 6000;
    e.peak_torque_nm = 200;
    e.mechanical_throttle = true;
    e.fuels = {"e5"};
    rg::SwapPackage p = rg::derive_swap_package(car, e, {}, true);
    CHECK(p.zero_blip);
    CHECK(p.drop_nitrous); // not simulated
    CHECK(p.drop_slot_overrides);
    e.simulated = true;
    e.mechanical_throttle = false;
    p = rg::derive_swap_package(car, e, {}, true);
    CHECK_FALSE(p.zero_blip);
    CHECK_FALSE(p.drop_nitrous);
    // a fuel the player picked must be the engine's
    rg::SwapChoices choices;
    choices.fuel = "diesel";
    p = rg::derive_swap_package(car, e, choices, true);
    CHECK_FALSE(p.ok);
    // no swap, no derivation (only the fuel choice applies)
    p = rg::derive_swap_package(car, e, {}, false);
    CHECK_FALSE(p.set_tcu);
    CHECK_FALSE(p.set_clutch);
    CHECK_FALSE(p.engine_swapped);
}

TEST_CASE("engine swap: engine-bound options follow the installed engine", "[engine_garage][engine_swap]") {
    Fixture f;
    const auto& hyper = f.entry("car_hyper");
    auto model_of = [&](const rg::VehicleSetup& s) { return rg::build_setup_model(hyper, f.table, f.ctx, &s); };
    const auto availability = [&](const rg::SetupModel& m, const char* id) {
        for (const auto& o : m.options) if (o.def.id == id) return o.available;
        return false;
    };
    const auto stock = model_of(setup_of("car_hyper", {}));
    REQUIRE(stock.error.empty());
    CHECK(availability(stock, "turbo_install"));
    const auto swapped = model_of(setup_of("car_hyper", {{"engine_install", std::string("sedan_i6")}}));
    REQUIRE(swapped.error.empty());
    CHECK_FALSE(availability(swapped, "turbo_install")); // the I6 has no turbo to install
    CHECK(swapped.chassis_mass_delta_kg == Approx(165.0 - 230.0));
    CHECK_FALSE(swapped.swap_lines.empty());
    CHECK(swapped.engine_title == "Inline-six 3.0");

    // Availability reads the INSTALLED engine, not the car's own: the A25A car offers its sedan nitrous kit, an I4 swapped in
    // (torque map) loses the kit and a simulated V8 keeps it, but the kit is not on the V8's engine list.
    const auto& n2o_car = f.entry("car_sedan_gen_n2o");
    const auto n2o_model = [&](const rg::VehicleSetup& s) { return rg::build_setup_model(n2o_car, f.table, f.ctx, &s); };
    const auto n2o_stock = n2o_model(setup_of("car_sedan_gen_n2o", {}));
    REQUIRE(n2o_stock.error.empty());
    CHECK(availability(n2o_stock, "nitrous_install_sedan"));
    const auto n2o_v8 = n2o_model(setup_of("car_sedan_gen_n2o", {{"engine_install", std::string("default:hyper_v8_one1_audio")}}));
    REQUIRE(n2o_v8.error.empty());
    CHECK_FALSE(availability(n2o_v8, "nitrous_install_sedan"));
}

TEST_CASE("engine swap: the chassis-mass delta is the catalog figure difference", "[engine_garage][engine_swap][mass]") {
    Fixture f;
    const auto delta = [&](const char* car, const char* engine) {
        const Loaded l = load_setup(f, f.entry(car), setup_of(car, {{"engine_install", std::string(engine)}}));
        REQUIRE(l.ok);
        return l.compiled.chassis_mass_delta_kg;
    };
    CHECK(delta("car_sedan", "default:i4_2l_na") == Approx(105.0 - 165.0));
    CHECK(delta("car_sedan", "default:hyper_v8_one1_audio") == Approx(230.0 - 165.0));
    CHECK(delta("car_sedan_rwd", "default:n52b30_3l_na") == Approx(165.0 - 105.0));
    CHECK(delta("car_hyper", "default:i4_2l_na") == Approx(105.0 - 230.0));
    CHECK(delta("car_sedan_rwd", "default:i4_2l_na") == 0.0);
}

TEST_CASE("engine swap: the fit seam allows every engine on every car today", "[engine_garage][engine_swap]") {
    Fixture f;
    for (const rg::CatalogEntry* car : f.base_cars()) {
        for (const rg::EngineEntry& e : f.store->list()) {
            const rg::FitResult r = rg::check_fit(*car, e);
            CHECK(r.allowed);
            CHECK(r.reasons.empty());
        }
    }
}

TEST_CASE("engine setup: a /1 setup with the first version's engine ids reads unchanged and resolves", "[engine_garage][setup]") {
    Fixture f;
    const std::string golden =
        "{\n"
        "  \"format\": \"racing_game.vehicle_setup/1\",\n"
        "  \"options\": {\n"
        "    \"engine_install\": \"sedan_i6\",\n"
        "    \"engine_rev_limit\": 0.9\n"
        "  },\n"
        "  \"vehicle\": \"car_sedan_rwd\"\n"
        "}\n";
    std::string err;
    auto parsed = rg::parse_setup(golden, f.table, "golden", &err);
    INFO(err);
    REQUIRE(parsed.has_value());
    CHECK(parsed->vehicle_id == "car_sedan_rwd");
    REQUIRE(parsed->values.count("engine_install") == 1);
    CHECK(std::get<std::string>(parsed->values.at("engine_install")) == "sedan_i6"); // not rewritten on read
    CHECK(std::get<double>(parsed->values.at("engine_rev_limit")) == Approx(0.9));
    CHECK(rg::setup_to_json(*parsed) == golden); // and written back identically
    // the legacy id resolves to the same car as its key
    const auto& car = f.entry("car_sedan_rwd");
    const Loaded legacy = load_setup(f, car, *parsed);
    REQUIRE(legacy.ok);
    auto keyed = *parsed;
    keyed.values["engine_install"] = std::string("default:n52b30_3l_na");
    const Loaded by_key = load_setup(f, car, keyed);
    REQUIRE(by_key.ok);
    CHECK(legacy.compiled.vehicle_patch == by_key.compiled.vehicle_patch);
    CHECK(legacy.compiled.patches == by_key.compiled.patches);
    CHECK(engine_of(legacy.desc).limiter == Approx(6900.0 * 0.9));
}

TEST_CASE("user engines: an edited default is listed next to its default and drives with the change", "[engine_garage][engine_store][user]") {
    TempDir user;
    Fixture f(user.path);
    const std::size_t defaults = f.store->list().size();
    std::string err;
    // a bad patch creates nothing
    CHECK_FALSE(f.store->create_user_engine("default:i4_2l_na", "Broken", R"({"limiter":{"rpm":-5}})", &err).has_value());
    CHECK_FALSE(err.empty());
    CHECK(f.store->list().size() == defaults);
    // edited once
    const auto key = f.store->create_user_engine("default:i4_2l_na", "Revvy I4", R"({"limiter":{"type":"fuel_cut","rpm":7300,"hysteresis_rpm":150}})", &err);
    INFO(err);
    REQUIRE(key.has_value());
    CHECK(f.store->list().size() == defaults + 1);
    // listed directly after its default
    const auto& list = f.store->list();
    std::size_t at = 0;
    for (std::size_t i = 0; i < list.size(); ++i) if (list[i].key == "default:i4_2l_na") at = i;
    REQUIRE(at + 1 < list.size());
    CHECK(list[at + 1].key == *key);
    CHECK(list[at + 1].user);
    CHECK(list[at + 1].name == "Revvy I4");
    CHECK(list[at + 1].base_key == "default:i4_2l_na");
    CHECK(list[at + 1].mass_kg == Approx(105.0));
    CHECK(f.store->peaks(list[at + 1]).torque_nm == Approx(f.store->peaks(list[at]).torque_nm));
    // the physics loader sees the change
    f.refresh();
    const Loaded l = load_setup(f, f.entry("car_sedan_rwd"), setup_of("car_sedan_rwd", {{"engine_install", *key}}));
    INFO(l.error);
    REQUIRE(l.ok);
    CHECK(engine_of(l.desc).limiter == Approx(7300.0));
    // the default is untouched
    const Loaded stock = load_setup(f, f.entry("car_sedan_rwd"), setup_of("car_sedan_rwd", {{"engine_install", std::string("default:i4_2l_na")}}));
    REQUIRE(stock.ok);
    CHECK(engine_of(stock.desc).limiter == Approx(6800.0));
    // survives a restart
    auto reopened = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, user.path, &err);
    REQUIRE(reopened != nullptr);
    REQUIRE(reopened->find(*key) != nullptr);
    CHECK(reopened->find(*key)->name == "Revvy I4");
    // update, rename, remove
    REQUIRE(f.store->update_user_engine(*key, R"({"limiter":{"type":"fuel_cut","rpm":7000,"hysteresis_rpm":150}})", &err));
    CHECK(f.store->user_patch(*key).find("7000") != std::string::npos);
    REQUIRE(f.store->rename_user_engine(*key, "Calmer I4", &err));
    CHECK(f.store->find(*key)->name == "Calmer I4");
    CHECK_FALSE(f.store->remove_user_engine("default:i4_2l_na", &err));
    REQUIRE(f.store->remove_user_engine(*key, &err));
    CHECK(f.store->list().size() == defaults);
    CHECK(f.store->find(*key) == nullptr);
}

TEST_CASE("garage: an engine swap reaches the prepared drive, the legacy id is stored as a key, a removed engine falls back", "[engine_garage][garage]") {
    TempDir user, work;
    rg::GarageConfig cfg;
    cfg.repo_root = kRoot;
    cfg.catalog_path = kRoot + "/data/vehicles/catalog.json";
    cfg.options_path = kRoot + "/data/vehicles/setup_options.json";
    cfg.set_path = kRoot + "/data/garage/garage_set.json";
    cfg.tyre_families_path = kRoot + "/data/tyres/families.json";
    cfg.user_dir = user.path;
    cfg.work_root = work.path;
    cfg.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
    std::string err;
    auto g = rg::Garage::open(cfg, &err);
    INFO(err);
    REQUIRE(g != nullptr);
    REQUIRE(g->begin_edit("car_sedan_rwd", &err));
    const auto r = g->set_option("engine_install", std::string("sedan_i6"));
    INFO(r.message);
    REQUIRE(r.accepted);
    CHECK(std::get<std::string>(g->current_value("engine_install")) == "default:n52b30_3l_na");
    CHECK(g->swap_lines().empty() == false);
    CHECK(g->chassis_mass_delta_kg() == Approx(165.0 - 105.0));
    REQUIRE(g->save(&err));
    const auto drive = g->prepare_drive("car_sedan_rwd");
    INFO(drive.error);
    REQUIRE(drive.ok);
    CHECK(drive.engine_title == "Inline-six 3.0");
    CHECK(drive.chassis.mass_kg == Approx(g->catalog().find("car_sedan_rwd")->chassis.mass_kg + 60.0));
    CHECK_FALSE(drive.swap_lines.empty());
    // an engine that disappears: the car keeps its own engine and the drive says so
    const auto key = g->create_user_engine("default:i4_2l_na", "Mine", "{}", &err);
    REQUIRE(key.has_value());
    REQUIRE(g->begin_edit("car_sedan", &err));
    REQUIRE(g->set_option("engine_install", *key).accepted);
    REQUIRE(g->save(&err));
    REQUIRE(g->remove_user_engine(*key, &err));
    const auto after = g->prepare_drive("car_sedan");
    REQUIRE(after.ok);
    CHECK_THAT(after.warning, ContainsSubstring("no longer in the garage list"));
    CHECK(after.engine_title.empty());
}

TEST_CASE("garage: changing the engine resets options the new engine does not offer", "[engine_garage][garage]") {
    TempDir user, work;
    rg::GarageConfig cfg;
    cfg.repo_root = kRoot;
    cfg.catalog_path = kRoot + "/data/vehicles/catalog.json";
    cfg.options_path = kRoot + "/data/vehicles/setup_options.json";
    cfg.set_path = kRoot + "/data/garage/garage_set.json";
    cfg.tyre_families_path = kRoot + "/data/tyres/families.json";
    cfg.user_dir = user.path;
    cfg.work_root = work.path;
    cfg.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
    std::string err;
    auto g = rg::Garage::open(cfg, &err);
    REQUIRE(g != nullptr);
    REQUIRE(g->begin_edit("car_hyper", &err));
    REQUIRE(g->set_option("turbo_install", std::string("hyper_balanced")).accepted);
    const auto r = g->set_option("engine_install", std::string("sedan_i4"));
    REQUIRE(r.accepted);
    CHECK_FALSE(r.notice.empty());
    CHECK(std::get<std::string>(g->current_value("turbo_install")) == "stock");
    CHECK(g->validation().ok);
}

TEST_CASE("engine catalog: the cycle engines' peak figures match the engine dyno", "[engine_garage][engine_store][long][.]") {
    Fixture f;
    ps::jobs::JobSystem pool(8);
    for (const rg::EngineEntry& e : f.store->list()) {
        if (e.cls == rg::EngineClass::Curve) continue;
        INFO(e.key);
        ps::io::EngineMapOptions options;
        options.cache_dir = f.ctx.engine_map_cache_dir;
        options.pool = &pool;
        const auto params = ps::io::load_engine_json(e.file, options);
        ps::drivetrain::DynoSweepSpec spec;
        const auto result = ps::drivetrain::run_dyno_sweep(params, nullptr, spec, &pool);
        double tq = 0, tq_rpm = 0, pw = 0, pw_rpm = 0;
        for (const auto& p : result.points) {
            if (p.torque_nm > tq) { tq = p.torque_nm; tq_rpm = p.rpm; }
            if (p.power_kw > pw) { pw = p.power_kw; pw_rpm = p.rpm; }
        }
        WARN(e.key << ": torque " << tq << " N*m at " << tq_rpm << " rpm, power " << pw << " kW at " << pw_rpm << " rpm");
        const rg::EnginePeaks c = e.catalog_peaks;
        CHECK(c.torque_nm == Approx(tq).epsilon(0.03));
        CHECK(c.power_kw == Approx(pw).epsilon(0.03));
        CHECK(std::abs(c.torque_rpm - tq_rpm) <= 500.0);
        CHECK(std::abs(c.power_rpm - pw_rpm) <= 500.0);
    }
}
