// test_limit_advisor_electric.cpp - the setup check on cars without a combustion engine (2026-10-20). It used to throw "The
// vehicle has no engine the dyno can sweep", so electric and fuel-cell cars showed "check failed". Now the engine checks are
// skipped with a reason, and the clutch / gearbox-input checks (physics check_driveline_limits) run against the traction
// motors' input torque from physics run_powertrain_dyno whenever a motor drives the gearbox input.
#include "rg/engine_store.h"
#include "rg/garage.h"
#include "rg/gearbox_catalog.h"
#include "rg/limit_advisor.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"
#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/electric_drive_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/jobs/job_system.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <thread>
#include <variant>
namespace {
namespace fs = std::filesystem;
namespace pd = ps::drivetrain;
using Catch::Approx;
const std::string kRoot = RG_SOURCE_DIR;

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    rg::AdvisorConfig config;
    rg::ScopedWorkDir isolated_root{fs::temp_directory_path().generic_string(), "rg_advisor_electric_tests"};
    std::unique_ptr<rg::Garage> garage; // after the work dir: closed before it is removed
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
        // The setup context exactly as the garage builds it (every catalogue: engines, gearboxes, drivelines, kits, internals).
        rg::GarageConfig gc;
        gc.repo_root = kRoot;
        gc.catalog_path = kRoot + "/data/vehicles/catalog.json";
        gc.options_path = kRoot + "/data/vehicles/setup_options.json";
        gc.set_path = kRoot + "/data/garage/garage_set.json";
        gc.tyre_families_path = kRoot + "/data/tyres/families.json";
        gc.user_dir = isolated_root.path() + "/user";
        gc.work_root = isolated_root.path() + "/work";
        gc.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
        fs::create_directories(gc.user_dir);
        fs::create_directories(gc.work_root);
        garage = rg::Garage::open(gc, &err);
        INFO(err);
        REQUIRE(garage != nullptr);
        ctx = garage->context();
        auto a = rg::load_advisor_config(kRoot + "/data/garage/advisor.json", &err);
        INFO(err);
        REQUIRE(a.has_value());
        config = *a;
        config.debounce_ms = 5;
    }
    const rg::CatalogEntry& entry(const std::string& id) const {
        const rg::CatalogEntry* e = catalog.find(id);
        REQUIRE(e != nullptr);
        return *e;
    }
    // The setup check exactly as the garage runs it (background advisor).
    rg::AdvisorResult check(const std::string& car, std::map<std::string, rg::SetupValue> values) {
        rg::LimitAdvisor advisor(config);
        rg::AdvisorRequest r;
        r.entry = entry(car);
        r.table = table;
        r.context = ctx;
        r.setup.vehicle_id = car;
        r.setup.values = std::move(values);
        r.key = car;
        advisor.request(r);
        rg::AdvisorResult res;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
        for (;;) {
            res = advisor.result();
            if (!res.busy && !res.solving && res.have_result) break;
            if (!res.busy && !res.solving && !res.error.empty()) break;
            REQUIRE(std::chrono::steady_clock::now() < deadline);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return res;
    }
};
bool contains(const std::vector<std::string>& list, const std::string& part) {
    return std::any_of(list.begin(), list.end(), [&](const std::string& s) { return s.find(part) != std::string::npos; });
}
} // namespace

TEST_CASE("advisor: electric and fuel-cell cars get a setup check, not 'check failed'", "[limit_advisor][electric]") {
    Fixture f;
    for (const auto& [car, install] : std::vector<std::pair<std::string, std::string>>{{"car_electric", ""}, {"car_sedan", "fuel_cell_ev"},
                                                                                       {"car_sedan", "battery_ev"}}) {
        INFO(car << " + " << install);
        std::map<std::string, rg::SetupValue> values;
        if (!install.empty()) values["powertrain_install"] = install;
        const rg::AdvisorResult res = f.check(car, values);
        INFO(res.error);
        CHECK(res.error.empty());
        CHECK(res.have_result);
        // Nothing engine-shaped can be reported for a car without an engine; anything else would be a real driveline finding.
        for (const rg::AdviceItem& item : res.items) {
            INFO(item.id);
            CHECK((item.kind_name == "ClutchCapacity" || item.kind_name == "GearboxInputTorque" || !item.rule_name.empty()));
        }
    }
}

TEST_CASE("advisor sweep: a car without an engine names what it skips", "[limit_advisor][electric]") {
    Fixture f;
    ps::jobs::JobSystem pool(4);
    ps::io::EngineMapOptions maps;
    const auto ev = rg::load_vehicle(f.entry("car_electric").vehicle_path, maps);
    const rg::AdviceSweep sweep = rg::run_advice_sweep(ev, f.config, &pool, nullptr);
    CHECK_FALSE(sweep.has_engine);
    CHECK(sweep.input.findings.empty());
    CHECK(sweep.input.points.empty());
    CHECK(contains(sweep.skipped, "no combustion engine"));
    CHECK(contains(sweep.skipped, "no limit kind for electric machines"));
    CHECK(contains(sweep.skipped, "the car has neither")); // axle motors, no clutch or gearbox
    CHECK(rg::build_advice(sweep.input, f.config).empty());
}

// No shipped electric car drives through a clutch and gearbox, so this one is built in code: car_sedan with its engine replaced
// by a 600 N*m / 200 kW motor on the crank shaft (same component id) and a battery.
namespace {
ps::vehicle::VehicleDesc motor_sedan(Fixture& f, double clutch_scale, double gearbox_rating_nm) {
    ps::io::EngineMapOptions maps;
    auto desc = rg::load_vehicle(f.entry("car_sedan").vehicle_path, maps);
    pd::ElectricMotorDesc motor;
    motor.battery = "battery";
    motor.peak_torque_nm = 600;
    motor.max_power_w = 200000;
    motor.max_rpm = 12000;
    pd::BatteryDesc battery;
    battery.max_discharge_w = 400000;
    auto& comps = desc.powertrain.components;
    std::string tank;
    for (auto& c : comps) {
        if (c.id == "engine") c.params = motor;
        if (std::holds_alternative<pd::FuelTankDesc>(c.params)) tank = c.id;
        if (auto* cl = std::get_if<pd::ClutchDesc>(&c.params)) cl->capacity_nm *= clutch_scale;
        if (auto* box = std::get_if<pd::GearboxDesc>(&c.params)) box->rated_input_torque_nm = gearbox_rating_nm;
    }
    comps.erase(std::remove_if(comps.begin(), comps.end(), [&](const pd::ComponentDesc& c) { return c.id == tank; }), comps.end());
    comps.insert(comps.begin(), pd::ComponentDesc{"battery", battery});
    desc.powertrain.fluid.clear();
    // The manual TCU drives an engine (auto-clutch, blips); without one the clutch is the pedal's (closed) and the dyno holds the gear.
    desc.controllers.erase(std::remove_if(desc.controllers.begin(), desc.controllers.end(),
                                          [](const pd::ControllerDesc& c) { return std::holds_alternative<pd::ManualTcuDesc>(c.params); }),
                           desc.controllers.end());
    for (auto& shaft : desc.powertrain.shafts)
        for (auto& port : shaft.ports)
            if (port.component == "engine" && port.port == "crank") port.port = "shaft";
    return desc;
}
double clutch_static_nm(const ps::vehicle::VehicleDesc& desc) {
    for (const auto& c : desc.powertrain.components)
        if (const auto* cl = std::get_if<pd::ClutchDesc>(&c.params)) return cl->capacity_nm * cl->static_factor;
    return 0;
}
const pd::LimitFinding* find_kind(const rg::AdviceSweep& sweep, pd::LimitKind kind) {
    for (const auto& x : sweep.input.findings)
        if (x.kind == kind) return &x;
    return nullptr;
}
double peak_input_nm(const rg::AdviceSweep& sweep) {
    double peak = 0;
    for (const auto& p : sweep.input_torque_points) peak = std::max(peak, p.torque_nm);
    return peak;
}
} // namespace

TEST_CASE("advisor sweep: a motor driving the gearbox input is graded by the powertrain dyno", "[limit_advisor][electric]") {
    Fixture f;
    ps::jobs::JobSystem pool(4);
    SECTION("a strong clutch: the gearbox rating is graded on the motor's full torque") {
        const auto desc = motor_sedan(f, 5.0, 400.0);
        REQUIRE(clutch_static_nm(desc) > 700);
        const rg::AdviceSweep sweep = rg::run_advice_sweep(desc, f.config, &pool, nullptr, "clutch.json", "gearbox.json");
        CHECK_FALSE(sweep.has_engine);
        REQUIRE_FALSE(sweep.input_torque_points.empty());
        const double peak = peak_input_nm(sweep);
        CHECK(peak == Approx(600).epsilon(0.01)); // the motor's rating at the input (it sits on the crank shaft, clutch locked)
        const pd::LimitFinding* box = find_kind(sweep, pd::LimitKind::GearboxInputTorque);
        REQUIRE(box != nullptr);
        CHECK(box->severity == pd::LimitSeverity::Exceeded); // 50 % over the rating
        CHECK(box->value == Approx(peak));
        CHECK(box->limit == Approx(400));
        CHECK(box->part.file == "gearbox.json");
        CHECK(find_kind(sweep, pd::LimitKind::ClutchCapacity) == nullptr);
        CHECK_FALSE(contains(sweep.skipped, "Clutch and gearbox checks"));
    }
    SECTION("the stock clutch: overloaded by the motor") {
        const auto desc = motor_sedan(f, 1.0, 0.0);
        const double clutch_static = clutch_static_nm(desc);
        REQUIRE(clutch_static > 0);
        REQUIRE(clutch_static < 600);
        const rg::AdviceSweep sweep = rg::run_advice_sweep(desc, f.config, &pool, nullptr, "clutch.json", "gearbox.json");
        const pd::LimitFinding* clutch = find_kind(sweep, pd::LimitKind::ClutchCapacity);
        REQUIRE(clutch != nullptr);
        INFO("graded " << clutch->value << " vs static " << clutch->limit);
        CHECK(clutch->value == Approx(peak_input_nm(sweep)));
        // The clutch slips on the dyno, so the motor runs above its base speed and the graded torque (506 of 600 N*m against a
        // 517.5 N*m static capacity) understates the locked demand; physics still reports the finding.
        CHECK(clutch->value > 0.9 * clutch_static);
        CHECK(clutch->limit == Approx(clutch_static));
        CHECK(clutch->part.file == "clutch.json");
    }
}

// The owner's garage setup of 2026-10-19 (garage/setups/car_hyper_n2o.json): hyper_7dct, big bore, forged pistons, titanium
// rods, 4.08 bar boost on hyper_twin_large_n2o, hybrid_battery. Its shift clutches (1900 N*m kinetic, 2185 static) slipped
// ~1300 rpm in 6th/7th on the drive. The setup check must name them with the uprated-clutch option and a solved size.
namespace {
std::map<std::string, rg::SetupValue> owner_hyper_n2o_setup() {
    return {{"drive_layout", std::string("awd_open")},
            {"engine_bore", std::string("big_bore")},
            {"engine_pistons", std::string("forged")},
            {"engine_rods", std::string("titanium")},
            {"gearbox_install", std::string("hyper_7dct")},
            {"powertrain_install", std::string("hybrid_battery")},
            {"turbo_boost_target", 2.2666666666666666},
            {"turbo_install_n2o", std::string("hyper_twin_large_n2o")},
            {"tyre_front", std::string("slick_305_30r20")},
            {"tyre_rear", std::string("slick_305_32r21")}};
}
const rg::AdviceItem* find_item(const rg::AdvisorResult& res, const std::string& kind) {
    for (const rg::AdviceItem& item : res.items)
        if (item.kind_name == kind) return &item;
    return nullptr;
}
} // namespace

TEST_CASE("advisor: the owner's hyper n2o DCT hybrid setup names the slipping shift clutches", "[limit_advisor][shift_clutch]") {
    Fixture f;
    const rg::AdvisorResult res = f.check("car_hyper_n2o", owner_hyper_n2o_setup());
    INFO(res.error);
    REQUIRE(res.error.empty());
    REQUIRE(res.have_result);
    const rg::AdviceItem* clutch = find_item(res, "ClutchCapacity");
    REQUIRE(clutch != nullptr);
    CHECK(clutch->variant == "shift_clutch");
    CHECK(clutch->option_id == "shift_clutch_capacity");
    CHECK(clutch->severity == rg::AdviceSeverity::Warning); // physics' clutch grade: Limiting once the peak reaches the static capacity
    CHECK(clutch->remedy_state == rg::RemedyState::Closed); // "Gearbox clutches rated for about ... N*m hold it with margin"
    REQUIRE(clutch->findings.size() == 1);
    const pd::LimitFinding& x = clutch->findings.front();
    CHECK(x.part.field == "/shift_clutch/capacity_nm");
    CHECK(x.limit == Approx(2185.0)); // 1900 N*m x static factor 1.15
    CHECK(x.value > 2185.0);          // the advisor's sweep peak (the N2O shot included: 4361 N*m @ 3000 rpm)
    REQUIRE(x.remedy.has_value());
    CHECK(x.remedy->current == Approx(1900.0));
    CHECK(x.remedy->required_min > 1900.0);
}

TEST_CASE("advisor diag: the owner's hyper n2o DCT hybrid setup", "[.][diag][limit_advisor][shift_clutch]") {
    Fixture f;
    const auto values = owner_hyper_n2o_setup();
    for (const bool n2o : {true, false}) {
        f.config.sweep.nitrous_with_kit = n2o;
        const rg::AdvisorResult res = f.check("car_hyper_n2o", values);
        std::printf("n2o %d: error '%s' have %d sweep %.2f s\n", n2o ? 1 : 0, res.error.c_str(), res.have_result ? 1 : 0, res.sweep_seconds);
        for (const rg::AdviceItem& item : res.items) {
            if (item.kind_name != "ClutchCapacity" && item.kind_name != "GearboxInputTorque") continue;
            std::printf("item %s variant '%s' sev %d option '%s' suggestion '%s' remedy %d '%s' solve %d\n", item.id.c_str(), item.variant.c_str(),
                        static_cast<int>(item.severity), item.option_id.c_str(), item.suggestion.c_str(), static_cast<int>(item.remedy_state),
                        item.remedy_text.c_str(), item.can_solve ? 1 : 0);
            for (const auto& x : item.findings)
                std::printf("   rpm %.0f value %.1f limit %.1f field %s remedy %d min %.1f\n", x.rpm, x.value, x.limit, x.part.field.c_str(),
                            x.remedy ? 1 : 0, x.remedy ? x.remedy->required_min : 0.0);
        }
    }
}
