// test_nitrous_tuning.cpp - the N2O controller's rpm window as Tuning sliders (owner 2026-10-21: "I do not see a tuning
// option for N2O controller rpm range. It belong in Tuning."). nitrous_rpm_min / nitrous_rpm_max are scale options on the
// file kind "nitrous_controller": the controller the car's kit runs (the kit component's "controller", else the kit file's
// default_controller). A tuned controller is materialised next to the vehicle and the vehicle's kit component names it;
// the shared kit file is never copied.
#include "rg/engine_store.h"
#include "rg/vehicle_catalog.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/nitrous_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <variant>

namespace {
namespace fs = std::filesystem;
using Catch::Matchers::ContainsSubstring;
const std::string kRoot = RG_SOURCE_DIR;

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    rg::ScopedWorkDir isolated_root{fs::temp_directory_path().generic_string(), "rg_n2o_tuning_tests"};
    Fixture() {
        std::string err;
        auto c = rg::load_vehicle_catalog(kRoot + "/data/vehicles/catalog.json", kRoot, &err);
        REQUIRE(c.has_value());
        catalog = *c;
        auto t = rg::load_setup_options(kRoot + "/data/vehicles/setup_options.json", &err);
        INFO(err);
        REQUIRE(t.has_value());
        table = *t;
        ctx.tyre_families_path = kRoot + "/data/tyres/families.json";
        ctx.work_root = isolated_root.path();
        ctx.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
        auto store = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, "", &err);
        INFO(err);
        REQUIRE(store != nullptr);
        ctx.engines = std::shared_ptr<const rg::EngineStore>(std::move(store));
    }
    const rg::CatalogEntry& entry(const char* id) const {
        const rg::CatalogEntry* e = catalog.find(id);
        REQUIRE(e != nullptr);
        return *e;
    }
};

const rg::OptionView* option(const rg::SetupModel& m, const char* id) {
    for (const rg::OptionView& v : m.options) {
        if (v.def.id == id) return &v;
    }
    return nullptr;
}

const ps::drivetrain::NitrousKitDesc* kit_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* k = std::get_if<ps::drivetrain::NitrousKitDesc>(&c.params)) return k;
    }
    return nullptr;
}

struct Loaded {
    rg::MaterialisedSetup m;
    ps::vehicle::VehicleDesc desc;
};

Loaded materialise_and_load(const Fixture& f, const rg::CatalogEntry& e, const rg::VehicleSetup& s, const rg::ScopedWorkDir& dir) {
    Loaded out;
    out.m = rg::materialise_setup(e, f.table, f.ctx, s, dir.path());
    INFO(out.m.error);
    REQUIRE(out.m.ok);
    ps::io::EngineMapOptions options;
    options.cache_dir = f.ctx.engine_map_cache_dir;
    ps::jobs::JobSystem workers(4);
    options.pool = &workers;
    out.desc = rg::load_vehicle(out.m.vehicle_path, options);
    return out;
}

bool wrote_controller(const rg::MaterialisedSetup& m) {
    for (const std::string& p : m.files) {
        if (fs::path(p).filename().string().rfind("nitrous_controller_", 0) == 0) return true;
    }
    return false;
}

} // namespace

TEST_CASE("n2o tuning: the rpm window is a Tuning slider on cars with a kit only", "[setup][nitrous]") {
    Fixture f;
    const rg::SetupModel hyper = rg::build_setup_model(f.entry("car_hyper_n2o"), f.table, f.ctx);
    REQUIRE(hyper.error.empty());
    const rg::OptionView* lo = option(hyper, "nitrous_rpm_min");
    const rg::OptionView* hi = option(hyper, "nitrous_rpm_max");
    REQUIRE(lo != nullptr);
    REQUIRE(hi != nullptr);
    CHECK(lo->available);
    CHECK(hi->available);
    CHECK(lo->def.area == "engine");
    REQUIRE(lo->stock_numbers.size() == 1);
    REQUIRE(hi->stock_numbers.size() == 1);
    CHECK(lo->stock_numbers[0] == 3000.0); // data/nitrous_controllers/custom_wet_800.json
    CHECK(hi->stock_numbers[0] == 8100.0);

    const rg::SetupModel sedan = rg::build_setup_model(f.entry("car_sedan_gen_n2o"), f.table, f.ctx);
    REQUIRE(sedan.error.empty());
    REQUIRE(option(sedan, "nitrous_rpm_max") != nullptr);
    CHECK(option(sedan, "nitrous_rpm_max")->available);

    const rg::SetupModel plain = rg::build_setup_model(f.entry("car_hyper"), f.table, f.ctx);
    REQUIRE(plain.error.empty());
    REQUIRE(option(plain, "nitrous_rpm_max") != nullptr);
    CHECK_FALSE(option(plain, "nitrous_rpm_max")->available);
}

TEST_CASE("n2o tuning: a raised window reaches the physics controller, stock writes no copy", "[setup][nitrous]") {
    Fixture f;
    const rg::CatalogEntry& e = f.entry("car_hyper_n2o");
    {
        rg::ScopedWorkDir dir(f.ctx.work_root, "n2o_stock");
        rg::VehicleSetup stock;
        stock.vehicle_id = e.id;
        const Loaded l = materialise_and_load(f, e, stock, dir);
        CHECK_FALSE(wrote_controller(l.m));
        const auto* kit = kit_of(l.desc);
        REQUIRE(kit != nullptr);
        CHECK(kit->controller.min_rpm == 3000.0);
        CHECK(kit->controller.max_rpm == 8100.0);
    }
    // The owner case: rev limit raised to ~9300 rpm, spray window raised with it to 9200 rpm, start at 2500 rpm.
    rg::ScopedWorkDir dir(f.ctx.work_root, "n2o_raised");
    rg::VehicleSetup s;
    s.vehicle_id = e.id;
    s.values["engine_rev_limit"] = 9300.0 / 8250.0;
    s.values["nitrous_rpm_min"] = 2500.0 / 3000.0;
    s.values["nitrous_rpm_max"] = 9200.0 / 8100.0;
    const Loaded l = materialise_and_load(f, e, s, dir);
    CHECK(wrote_controller(l.m));
    const auto* kit = kit_of(l.desc);
    REQUIRE(kit != nullptr);
    CHECK(std::abs(kit->controller.min_rpm - 2500.0) < 1e-6);
    CHECK(std::abs(kit->controller.max_rpm - 9200.0) < 1e-6);
    // Everything else in the controller is the shipped one.
    CHECK(kit->controller.max_rpm_fade_rpm == 300.0);
    CHECK(kit->controller.min_gear == 2);
    CHECK(kit->controller.arm_channel == "nitrous_arm");
}

TEST_CASE("n2o tuning: a swapped kit tunes its own default controller", "[setup][nitrous]") {
    Fixture f;
    const rg::CatalogEntry& e = f.entry("car_hyper_n2o");
    rg::ScopedWorkDir dir(f.ctx.work_root, "n2o_half");
    rg::VehicleSetup s;
    s.vehicle_id = e.id;
    s.values["nitrous_install_hyper"] = std::string("nitrous_hyper_half");
    s.values["nitrous_rpm_max"] = 7000.0 / 8100.0;
    const Loaded l = materialise_and_load(f, e, s, dir);
    const auto* kit = kit_of(l.desc);
    REQUIRE(kit != nullptr);
    CHECK(std::abs(kit->controller.max_rpm - 7000.0) < 1e-6);
    CHECK(kit->nitrous_jet_diameter_mm < 3.0); // the half-flow kit is the one installed
}

TEST_CASE("n2o tuning: an inverted window is refused with the loader's reason", "[setup][nitrous]") {
    Fixture f;
    const rg::CatalogEntry& e = f.entry("car_hyper_n2o");
    rg::VehicleSetup s;
    s.vehicle_id = e.id;
    s.values["nitrous_rpm_min"] = 9000.0 / 3000.0;
    s.values["nitrous_rpm_max"] = 5000.0 / 8100.0;
    const rg::ValidationResult v = rg::validate_setup(e, f.table, f.ctx, s, true);
    CHECK_FALSE(v.ok);
    CHECK_THAT(v.message, ContainsSubstring("max_rpm"));
}

TEST_CASE("n2o tuning: a car without a kit ignores a saved window", "[setup][nitrous]") {
    Fixture f;
    const rg::CatalogEntry& e = f.entry("car_hyper");
    rg::ScopedWorkDir dir(f.ctx.work_root, "n2o_none");
    rg::VehicleSetup s;
    s.vehicle_id = e.id;
    s.values["nitrous_rpm_max"] = 1.1;
    const rg::MaterialisedSetup m = rg::materialise_setup(e, f.table, f.ctx, s, dir.path());
    INFO(m.error);
    CHECK(m.ok);
    CHECK_FALSE(wrote_controller(m));
}
