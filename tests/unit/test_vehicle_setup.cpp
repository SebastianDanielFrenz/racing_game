// test_vehicle_setup.cpp - R6 setup overlays (racing_game.vehicle_setup/1):
// every whitelisted change round-trips through ps::io::load_vehicle_json, an
// out-of-range or invalid overlay is rejected with the loader's/rule's message,
// the whitelist is enforced on the merge patch, temp files are removed.
#include "rg/tyre_spec.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/assist_unit_desc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <initializer_list>
#include <set>
#include <string>
#include <utility>
#include <variant>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
const std::string kRoot = RG_SOURCE_DIR;

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    rg::ScopedWorkDir isolated_root{fs::temp_directory_path().generic_string(), "rg_r6_unit_tests"};
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

rg::VehicleSetup setup_of(const char* vehicle, std::initializer_list<std::pair<const char*, rg::SetupValue>> values) {
    rg::VehicleSetup s;
    s.vehicle_id = vehicle;
    for (const auto& kv : values) s.values[kv.first] = kv.second;
    return s;
}

// Materialise, load with the physics loader, return the desc (empty on a materialise failure).
ps::vehicle::VehicleDesc load_materialised(const Fixture& f, const rg::CatalogEntry& e, const rg::VehicleSetup& s,
                                           std::string* error = nullptr) {
    rg::ScopedWorkDir dir(f.ctx.work_root, "t_" + e.id);
    const rg::MaterialisedSetup m = rg::materialise_setup(e, f.table, f.ctx, s, dir.path());
    if (!m.ok) {
        if (error != nullptr) *error = m.error;
        return {};
    }
    ps::io::EngineMapOptions options;
    options.cache_dir = f.ctx.engine_map_cache_dir;
    ps::jobs::JobSystem workers(4);
    options.pool = &workers;
    return rg::load_vehicle(m.vehicle_path, options);
}

const ps::drivetrain::GearboxDesc* gearbox_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* g = std::get_if<ps::drivetrain::GearboxDesc>(&c.params)) return g;
    }
    return nullptr;
}
std::vector<const ps::drivetrain::DifferentialDesc*> diffs_of(const ps::vehicle::VehicleDesc& d) {
    std::vector<const ps::drivetrain::DifferentialDesc*> out;
    for (const auto& c : d.powertrain.components) {
        if (const auto* g = std::get_if<ps::drivetrain::DifferentialDesc>(&c.params)) out.push_back(g);
    }
    return out;
}
const ps::drivetrain::ManualTcuDesc* tcu_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.controllers) {
        if (const auto* g = std::get_if<ps::drivetrain::ManualTcuDesc>(&c.params)) return g;
    }
    return nullptr;
}
} // namespace

TEST_CASE("setup: the option table parses and rejects a bad one", "[setup][garage]") {
    Fixture f;
    CHECK(f.table.options.size() >= 16);
    CHECK(f.table.find("final_drive") != nullptr);
    std::string err;
    CHECK(!rg::parse_setup_options("{}", "t", &err).has_value());
    const char* no_pointers = R"({"format":"racing_game.vehicle_setup_options/1","options":[
        {"id":"a","kind":"scale","min":0.8,"max":1.2,"step":0.01,"file":"vehicle"}]})";
    CHECK(!rg::parse_setup_options(no_pointers, "t", &err).has_value());
    CHECK_THAT(err, ContainsSubstring("whitelist"));
    const char* bad_range = R"({"format":"racing_game.vehicle_setup_options/1","options":[
        {"id":"a","kind":"scale","min":0,"max":1.2,"step":0.01,"file":"vehicle","pointers":["/x"]}]})";
    CHECK(!rg::parse_setup_options(bad_range, "t", &err).has_value());
}

TEST_CASE("setup: an empty setup changes nothing and writes no file", "[setup][garage]") {
    Fixture f;
    const auto& e = f.entry("car_sedan");
    rg::ScopedWorkDir dir(f.ctx.work_root, "t_empty");
    const rg::MaterialisedSetup m = rg::materialise_setup(e, f.table, f.ctx, setup_of("car_sedan", {}), dir.path());
    REQUIRE(m.ok);
    CHECK(m.files.empty());
    CHECK(m.vehicle_path == fs::path(e.vehicle_path).lexically_normal().generic_string());
    CHECK(rg::ScopedWorkDir::file_count(dir.path()) == 0);
    // a value equal to the stock one is no change either
    const rg::MaterialisedSetup same =
        rg::materialise_setup(e, f.table, f.ctx, setup_of("car_sedan", {{"final_drive", 1.0}}), dir.path());
    REQUIRE(same.ok);
    CHECK(same.files.empty());
}

TEST_CASE("setup: final drive, brakes, suspension round-trip through load_vehicle_json", "[setup][garage]") {
    Fixture f;
    const auto& e = f.entry("car_sedan");
    const ps::vehicle::VehicleDesc stock = rg::load_vehicle(e.vehicle_path);
    const auto s = setup_of("car_sedan", {{"final_drive", 1.15}, {"spring_front", 0.8}, {"damper_rear", 1.25},
                                          {"arb_front", 1.3}, {"brake_force", 0.9}});
    const ps::vehicle::VehicleDesc d = load_materialised(f, e, s);
    REQUIRE(d.wheels.size() == 4);
    CHECK(diffs_of(d).front()->ratio == Approx(diffs_of(stock).front()->ratio * 1.15));
    CHECK(d.wheels[0].suspension.spring_rate == Approx(stock.wheels[0].suspension.spring_rate * 0.8));
    CHECK(d.wheels[1].suspension.spring_rate == Approx(stock.wheels[1].suspension.spring_rate * 0.8));
    CHECK(d.wheels[2].suspension.spring_rate == Approx(stock.wheels[2].suspension.spring_rate)); // rear untouched
    CHECK(d.wheels[2].suspension.damper_bump_rate == Approx(stock.wheels[2].suspension.damper_bump_rate * 1.25));
    CHECK(d.wheels[3].suspension.damper_rebound_rate == Approx(stock.wheels[3].suspension.damper_rebound_rate * 1.25));
    CHECK(d.wheels[0].suspension.arb_rate == Approx(stock.wheels[0].suspension.arb_rate * 1.3));
    CHECK(d.brake_system.corners[0].max_torque_nm == Approx(stock.brake_system.corners[0].max_torque_nm * 0.9));
    CHECK(d.brake_system.corners[2].max_torque_nm == Approx(stock.brake_system.corners[2].max_torque_nm * 0.9));
}

TEST_CASE("setup: gear ratios are scaled per gear and must stay decreasing", "[setup][garage]") {
    Fixture f;
    const auto& e = f.entry("car_sedan");
    const ps::vehicle::VehicleDesc stock = rg::load_vehicle(e.vehicle_path);
    const auto* sg = gearbox_of(stock);
    REQUIRE(sg != nullptr);
    const auto ok = setup_of("car_sedan", {{"gear_ratios", std::vector<double>{1.1, 1.0, 1.0, 0.95, 1.0, 0.9}}});
    const ps::vehicle::VehicleDesc d = load_materialised(f, e, ok);
    const auto* g = gearbox_of(d);
    REQUIRE(g != nullptr);
    REQUIRE(g->forward_ratios.size() == 6);
    CHECK(g->forward_ratios[0] == Approx(sg->forward_ratios[0] * 1.1));
    CHECK(g->forward_ratios[3] == Approx(sg->forward_ratios[3] * 0.95));
    CHECK(g->forward_ratios[5] == Approx(sg->forward_ratios[5] * 0.9));

    // second gear pushed above first: rejected, naming the rule
    std::string err;
    load_materialised(f, e, setup_of("car_sedan", {{"gear_ratios", std::vector<double>{1.0, 1.0, 1.0, 0.8, 1.2, 1.0}}}), &err);
    CHECK_THAT(err, ContainsSubstring("keep getting lower"));
    // wrong number of values
    load_materialised(f, e, setup_of("car_sedan", {{"gear_ratios", std::vector<double>{1.0, 1.0}}}), &err);
    CHECK_THAT(err, ContainsSubstring("needs 6 values"));
}

TEST_CASE("setup: brake bias moves the front share and keeps the total", "[setup][garage]") {
    Fixture f;
    const auto& e = f.entry("car_sedan");
    const ps::vehicle::VehicleDesc stock = rg::load_vehicle(e.vehicle_path);
    double total = 0.0, front = 0.0;
    for (const auto& c : stock.brake_system.corners) {
        total += c.max_torque_nm;
        if (c.wheel[0] == 'F') front += c.max_torque_nm;
    }
    const double stock_share = front / total;
    const ps::vehicle::VehicleDesc d = load_materialised(f, e, setup_of("car_sedan", {{"brake_bias", stock_share - 0.08}}));
    double t2 = 0.0, f2 = 0.0;
    for (const auto& c : d.brake_system.corners) {
        t2 += c.max_torque_nm;
        if (c.wheel[0] == 'F') f2 += c.max_torque_nm;
    }
    CHECK(t2 == Approx(total));
    CHECK(f2 / t2 == Approx(stock_share - 0.08));
    // brake force first, then the bias keeps the (scaled) total
    const ps::vehicle::VehicleDesc d2 =
        load_materialised(f, e, setup_of("car_sedan", {{"brake_force", 1.2}, {"brake_bias", stock_share + 0.05}}));
    double t3 = 0.0, f3 = 0.0;
    for (const auto& c : d2.brake_system.corners) {
        t3 += c.max_torque_nm;
        if (c.wheel[0] == 'F') f3 += c.max_torque_nm;
    }
    CHECK(t3 == Approx(total * 1.2));
    CHECK(f3 / t3 == Approx(stock_share + 0.05));
    // beyond the allowed shift
    std::string err;
    load_materialised(f, e, setup_of("car_sedan", {{"brake_bias", stock_share + 0.3}}), &err);
    CHECK_THAT(err, ContainsSubstring("outside the allowed range"));
}

TEST_CASE("setup: brake bias limits are read from the option table and a missing one is rejected", "[setup][garage][brake_bias]") {
    Fixture f;
    const rg::SetupOptionDef* shipped = f.table.find("brake_bias");
    REQUIRE(shipped != nullptr);
    CHECK(shipped->share_min == Approx(0.2));
    CHECK(shipped->share_max == Approx(0.85));
    CHECK(shipped->max_shift == Approx(0.1));
    CHECK(shipped->max_shift_bound == Approx(0.5));
    const auto table_with = [](const std::string& fields) {
        return std::string(R"({"format":"racing_game.vehicle_setup_options/1","options":[{"id":"brake_bias","kind":"brake_bias",)") +
               R"("file":"vehicle","pointers":["/brake_system/corners/*/max_torque_nm"],)" + fields + "}]}";
    };
    struct Bad { const char* fields; const char* expect; };
    const Bad bad[] = {
        {R"("max_shift_bound":0.5,"share_min":0.2,"share_max":0.85)", "\"max_shift\""},
        {R"("max_shift":0.1,"share_min":0.2,"share_max":0.85)", "\"max_shift_bound\""},
        {R"("max_shift":0.1,"max_shift_bound":0.5,"share_max":0.85)", "\"share_min\""},
        {R"("max_shift":0.1,"max_shift_bound":0.5,"share_min":0.2)", "\"share_max\""},
        {R"("max_shift":0.1,"max_shift_bound":0.5,"share_min":0.9,"share_max":0.85)", "share_min and share_max"},
        {R"("max_shift":0.5,"max_shift_bound":0.5,"share_min":0.2,"share_max":0.85)", "max_shift must be"},
        {R"("max_shift":0.1,"max_shift_bound":0.6,"share_min":0.2,"share_max":0.85)", "max_shift_bound must be"},
    };
    for (const Bad& b : bad) {
        INFO(b.fields);
        std::string err;
        CHECK(!rg::parse_setup_options(table_with(b.fields), "t", &err).has_value());
        CHECK_THAT(err, ContainsSubstring(b.expect));
    }
    // The limits are what the garage uses: a table with its own range moves the allowed shares and the check.
    std::string err;
    auto custom = rg::parse_setup_options(table_with(R"("max_shift":0.45,"max_shift_bound":0.5,"share_min":0.5,"share_max":0.6)"), "t", &err);
    REQUIRE(custom.has_value());
    f.table = *custom;
    const auto& e = f.entry("car_sedan");
    const rg::SetupModel model = rg::build_setup_model(e, f.table, f.ctx);
    const rg::OptionView* view = nullptr;
    for (const auto& o : model.options) if (o.def.id == "brake_bias") view = &o;
    REQUIRE(view != nullptr);
    CHECK(view->min == Approx(0.5));
    CHECK(view->max == Approx(0.6));
    CHECK(rg::validate_setup(e, f.table, f.ctx, setup_of("car_sedan", {{"brake_bias", 0.55}})).ok);
    const auto low = rg::validate_setup(e, f.table, f.ctx, setup_of("car_sedan", {{"brake_bias", 0.4}}));
    CHECK(!low.ok);
    CHECK_THAT(low.message, ContainsSubstring("outside the allowed range"));
}

TEST_CASE("setup: the AWD final drive scales both axle differentials, not the centre", "[setup][garage]") {
    Fixture f;
    const auto& e = f.entry("car_sedan_awd");
    const ps::vehicle::VehicleDesc stock = rg::load_vehicle(e.vehicle_path);
    const ps::vehicle::VehicleDesc d = load_materialised(f, e, setup_of("car_sedan_awd", {{"final_drive", 0.9}}));
    const auto ds = diffs_of(stock);
    const auto dd = diffs_of(d);
    REQUIRE(ds.size() == 3);
    REQUIRE(dd.size() == 3);
    int changed = 0, same = 0;
    for (std::size_t i = 0; i < 3; ++i) {
        if (dd[i]->ratio == Approx(ds[i]->ratio * 0.9)) ++changed;
        else if (dd[i]->ratio == Approx(ds[i]->ratio)) ++same;
    }
    CHECK(changed == 2);
    CHECK(same == 1);
}

std::shared_ptr<const rg::TyreFamilies> families_for(const Fixture& f) {
    std::string err;
    auto fam = rg::TyreFamilies::load(f.ctx.tyre_families_path, &err);
    INFO(err);
    REQUIRE(fam != nullptr);
    return fam;
}

// The id the garage uses for `compound` at `width` on a wheel of `diameter_mm` (the aspect keeps the diameter).
std::string size_id(const char* compound, int width, int rim, double diameter_mm) {
    rg::TyreSpec s;
    s.compound = compound;
    s.width_mm = width;
    s.rim_in = rim;
    s.aspect = rg::tyre_aspect_for(rim, width, diameter_mm);
    return rg::tyre_id(s);
}

TEST_CASE("setup: tyre choice offers every compound at every size the axle fitment allows", "[setup][garage][tyre_spec]") {
    Fixture f;
    const auto& e = f.entry("car_sedan");
    const auto families = families_for(f);
    const rg::SetupModel model = rg::build_setup_model(e, f.table, f.ctx);
    REQUIRE(model.error.empty());
    const rg::OptionView* tf = nullptr;
    for (const auto& o : model.options) {
        if (o.def.id == "tyre_front") tf = &o;
    }
    REQUIRE(tf != nullptr);
    CHECK(tf->available);
    CHECK(std::get<std::string>(tf->stock) == "road_225_45r17");
    CHECK(tf->choices.size() == families->compounds().size() * 7); // 195..255 mm in 10 mm steps
    for (const auto& c : tf->choices) CHECK(c.find("r20") == std::string::npos); // another rim is never offered

    const ps::vehicle::VehicleDesc d =
        load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", std::string("track_225_45r17")}}));
    const ps::vehicle::VehicleDesc stock = rg::load_vehicle(e.vehicle_path);
    const rg::TyreSpec track{"track", 225, 45, 17};
    CHECK(d.wheels[0].tyre.name == "track_225_45r17");
    CHECK(d.wheels[0].tyre.lambda_mux == families->make(track).lambda_mux);
    CHECK(d.wheels[0].tyre.lambda_mux > stock.wheels[0].tyre.lambda_mux);
    CHECK(d.wheels[2].tyre.lambda_mux == stock.wheels[2].tyre.lambda_mux); // rear untouched

    std::string err;
    load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", std::string("road_265_35r20")}}), &err);
    CHECK_THAT(err, ContainsSubstring("tyre_front: rim R20 does not fit this axle's R17 wheels"));
    err.clear();
    load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", std::string("not_a_tyre")}}), &err);
    CHECK_THAT(err, ContainsSubstring("unknown tyre \"not_a_tyre\""));
    err.clear();
    load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", std::string("mud_225_45r17")}}), &err);
    CHECK_THAT(err, ContainsSubstring("unknown tyre compound \"mud\""));
}

TEST_CASE("setup: assists change the controller defaults", "[setup][garage]") {
    Fixture f;
    const auto& e = f.entry("car_sedan");
    const ps::vehicle::VehicleDesc d =
        load_materialised(f, e, setup_of("car_sedan", {{"assist_auto_shift", true}, {"assist_auto_clutch", false}}));
    const auto* tcu = tcu_of(d);
    REQUIRE(tcu != nullptr);
    CHECK(tcu->auto_shift.enabled);
    CHECK(!tcu->auto_clutch.enabled);
}

TEST_CASE("setup: out-of-range and invalid values are rejected before anything is written", "[setup][garage]") {
    Fixture f;
    const auto& e = f.entry("car_sedan");
    for (const double bad : {0.79, 1.21, 0.0, -1.0, 100.0}) {
        const auto v = rg::validate_setup(e, f.table, f.ctx, setup_of("car_sedan", {{"final_drive", bad}}));
        INFO(bad);
        CHECK(!v.ok);
        CHECK_THAT(v.message, ContainsSubstring("outside the allowed range"));
    }
    CHECK(!rg::validate_setup(e, f.table, f.ctx, setup_of("car_sedan", {{"final_drive", true}})).ok); // wrong type
    CHECK(!rg::validate_setup(e, f.table, f.ctx, setup_of("car_sedan", {{"paint", std::string("red")}})).ok);
    CHECK(!rg::validate_setup(e, f.table, f.ctx, setup_of("car_sedan_rwd", {})).ok); // wrong vehicle
    CHECK(rg::validate_setup(e, f.table, f.ctx, setup_of("car_sedan", {{"final_drive", 1.2}, {"paint", std::string("#ff0000")}})).ok);
    // validation never leaves files behind
    CHECK(rg::ScopedWorkDir::file_count(f.ctx.work_root) == 0);
}

TEST_CASE("setup: a setup the loader rejects is reported with the loader's message", "[setup][garage]") {
    Fixture f;
    // A table whose range lets a factor through that breaks the physics file: the
    // differential efficiency scaled above 1. Our range check passes; the LOADER must say no.
    const char* json = R"({"format":"racing_game.vehicle_setup_options/1","options":[
        {"id":"diff_efficiency","kind":"scale","min":1.0,"max":2.0,"step":0.01,"file":"vehicle",
         "pointers":["/powertrain/components/[id=diff]/efficiency"]}]})";
    std::string err;
    auto table = rg::parse_setup_options(json, "t", &err);
    REQUIRE(table.has_value());
    f.table = *table;
    const auto& e = f.entry("car_sedan");
    const auto v = rg::validate_setup(e, f.table, f.ctx, setup_of("car_sedan", {{"diff_efficiency", 1.5}}));
    CHECK(!v.ok);
    CHECK(!v.message.empty());
    INFO(v.message);
    CHECK_THAT(v.message, ContainsSubstring("efficiency")); // the loader's own text, not ours
    CHECK(v.message.find("outside the allowed range") == std::string::npos);
    CHECK(rg::ScopedWorkDir::file_count(f.ctx.work_root) == 0);
}

TEST_CASE("setup: the whitelist is enforced on a merge patch", "[setup][garage]") {
    const std::string base = R"({"a":{"x":1,"y":2},"list":[{"k":1,"v":10},{"k":2,"v":20}],"keep":true})";
    const std::vector<std::string> patterns = {"/a/x", "/list/[k=2]/v"};
    CHECK(rg::whitelist_violation(base, R"({"a":{"x":5}})", patterns).empty());
    CHECK(rg::whitelist_violation(base, R"({"a":{"y":5}})", patterns) == "/a/y");
    CHECK(rg::whitelist_violation(base, R"({"keep":false})", patterns) == "/keep");
    CHECK(!rg::whitelist_violation(base, R"({"a":null})", patterns).empty()); // deleting the object removes /a/y too
    // arrays replace whole (RFC 7396): the changed element is the one the pattern names
    CHECK(rg::whitelist_violation(base, R"({"list":[{"k":1,"v":10},{"k":2,"v":99}]})", patterns).empty());
    CHECK(rg::whitelist_violation(base, R"({"list":[{"k":1,"v":11},{"k":2,"v":20}]})", patterns) == "/list/0/v");
    // RFC 7396 itself
    CHECK(rg::apply_merge_patch(R"({"a":{"b":1,"c":2}})", R"({"a":{"b":null,"d":3}})") == R"({"a":{"c":2,"d":3}})");
    CHECK(rg::apply_merge_patch(R"({"a":[1,2]})", R"({"a":[9]})") == R"({"a":[9]})");
}

TEST_CASE("setup: the saved document round-trips", "[setup][garage]") {
    Fixture f;
    rg::VehicleSetup s = setup_of("car_sedan", {{"final_drive", 1.1}, {"assist_auto_shift", true},
                                                  {"tyre_front", std::string("sport_225_45r17")},
                                                  {"gear_ratios", std::vector<double>{1, 1, 1, 1, 1, 0.9}},
                                                  {"paint", std::string("#112233")}});
    const std::string text = rg::setup_to_json(s);
    std::string err;
    const auto back = rg::parse_setup(text, f.table, "t", &err);
    INFO(err);
    REQUIRE(back.has_value());
    CHECK(back->vehicle_id == "car_sedan");
    CHECK(back->values.size() == 5);
    CHECK(std::get<double>(back->values.at("final_drive")) == 1.1);
    CHECK(std::get<bool>(back->values.at("assist_auto_shift")));
    CHECK(std::get<std::vector<double>>(back->values.at("gear_ratios")).size() == 6);
    CHECK(!rg::parse_setup(R"({"format":"racing_game.vehicle_setup/1","vehicle":"a","options":{"nope":1}})", f.table, "t", &err).has_value());
    CHECK_THAT(err, ContainsSubstring("unknown option"));
}

TEST_CASE("setup: every catalog entry accepts a setup touching each available option", "[setup][garage]") {
    Fixture f;
    for (const auto& e : f.catalog.entries) {
        INFO(e.id);
        const rg::SetupModel model = rg::build_setup_model(e, f.table, f.ctx);
        REQUIRE(model.error.empty());
        rg::VehicleSetup s;
        s.vehicle_id = e.id;
        int touched = 0;
        for (const auto& o : model.options) {
            if (!o.available) continue;
            // The catalog choices (engine, fuel) change which other options exist; they are tested on their own
            // (test_engine_swap.cpp: every engine on every car, fuel kept or changed).
            if (!o.def.catalog.empty()) continue;
            switch (o.def.kind) {
                case rg::OptionKind::Scale: s.values[o.def.id] = o.max; ++touched; break;
                case rg::OptionKind::ScaleList:
                    s.values[o.def.id] = std::vector<double>(static_cast<std::size_t>(o.list_size), 1.05);
                    ++touched;
                    break;
                case rg::OptionKind::BrakeBias: s.values[o.def.id] = std::get<double>(o.stock) + 0.02; ++touched; break;
                case rg::OptionKind::FileChoice:
                case rg::OptionKind::TyreChoice:
                case rg::OptionKind::PartChoice: s.values[o.def.id] = o.choices.back(); ++touched; break;
                case rg::OptionKind::Bool: s.values[o.def.id] = !std::get<bool>(o.stock); ++touched; break;
                case rg::OptionKind::Colour: s.values[o.def.id] = std::string("#336699"); break;
            }
        }
        if (e.id == "car_electric") CHECK(touched == 2); // Its catalogue explicitly offers the two axle tyre choices.
        else CHECK(touched >= 10);
        const auto v = rg::validate_setup(e, f.table, f.ctx, s);
        INFO(v.message);
        CHECK(v.ok);
    }
}

TEST_CASE("setup: tyre width changes geometry, grip and resistance together", "[setup][tyre_width][tyre_spec]") {
    Fixture f;
    const auto& e = f.entry("car_sedan");
    const double diameter = 634.0; // car_sedan's wheels
    const std::string narrow_id = size_id("road", 195, 17, diameter), wide_id = size_id("road", 255, 17, diameter);
    const auto narrow = load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", narrow_id}}));
    const auto wide = load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", wide_id}}));
    const auto stock = rg::load_vehicle(e.vehicle_path);
    REQUIRE(narrow.wheels.size() == 4);
    REQUIRE(wide.wheels.size() == 4);
    for (int i : {0, 1}) {
        CHECK(narrow.wheels[i].wheel_width == Approx(.195));
        CHECK(wide.wheels[i].wheel_width == Approx(.255));
        CHECK(wide.wheels[i].tyre.lambda_mux > narrow.wheels[i].tyre.lambda_mux);
        CHECK(wide.wheels[i].tyre.lambda_muy > narrow.wheels[i].tyre.lambda_muy);
        CHECK(wide.wheels[i].tyre.qsy1 > narrow.wheels[i].tyre.qsy1);
        CHECK(wide.wheels[i].wheel_inertia == Approx(stock.wheels[i].wheel_inertia * .255/.225));
        CHECK(wide.wheels[i].wheel_radius == Approx(stock.wheels[i].wheel_radius));
    }
    std::string saved_error;
    const auto saved = rg::parse_setup(rg::setup_to_json(setup_of("car_sedan", {{"tyre_front", wide_id}})), f.table, "width save test", &saved_error);
    REQUIRE(saved.has_value());
    const auto reloaded = load_materialised(f, e, *saved);
    CHECK(reloaded.wheels[0].wheel_width == Approx(.255));
    CHECK(reloaded.wheels[0].tyre.lambda_mux == wide.wheels[0].tyre.lambda_mux);
    CHECK(wide.wheels[2].wheel_width == Approx(stock.wheels[2].wheel_width));
    CHECK(wide.wheels[2].tyre.lambda_mux == stock.wheels[2].tyre.lambda_mux);
    // Too wide / too narrow: the message names the real reason, with the axle's limit.
    std::string error;
    load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", size_id("road", 265, 17, diameter)}}), &error);
    CHECK_THAT(error, ContainsSubstring("tyre_front: 265 mm is wider than this axle's 255 mm maximum"));
    error.clear();
    load_materialised(f, e, setup_of("car_sedan", {{"tyre_rear", size_id("road", 185, 17, diameter)}}), &error);
    CHECK_THAT(error, ContainsSubstring("tyre_rear: 185 mm is narrower than this axle's 195 mm minimum"));
    error.clear();
    load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", "road_225_30r17"}}), &error);
    CHECK_THAT(error, ContainsSubstring("tyre_front: overall diameter 567 mm differs from this axle's 634 mm by more than 12 mm"));
    error.clear();
    load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", "road_231_44r17"}}), &error);
    CHECK_THAT(error, ContainsSubstring("231 mm is not a standard width for this axle (steps of 10 mm from 195 mm)"));
    // The catalog's fitment changes the limit without changing compiler code.
    auto enlarged = e;
    enlarged.tyre_fitment["front"].max_width_mm = 275;
    const auto expanded = load_materialised(f, enlarged, setup_of("car_sedan", {{"tyre_front", size_id("road", 275, 17, diameter)}}));
    REQUIRE(expanded.wheels.size() == 4);
    CHECK(expanded.wheels[0].wheel_width == Approx(.275));
}

TEST_CASE("setup: the garage lists exactly the sizes the fitment allows", "[setup][tyre_width][tyre_spec]") {
    Fixture f;
    const auto families = families_for(f);
    for (const char* car : {"car_hyper", "car_hyper_n2o", "car_sedan", "car_sedan_awd"}) {
        const auto& e = f.entry(car);
        const auto stock = rg::load_vehicle(e.vehicle_path);
        const auto model = rg::build_setup_model(e, f.table, f.ctx);
        REQUIRE(model.error.empty());
        for (const auto& opt : model.options) {
            if (opt.def.id != "tyre_front" && opt.def.id != "tyre_rear") continue;
            const bool front = opt.def.id == "tyre_front";
            INFO(car << " " << opt.def.id);
            const rg::TyreAxleFitment& fit = e.tyre_fitment.at(front ? "front" : "rear");
            const double diameter = 2000.0 * stock.wheels[front ? 0 : 2].wheel_radius;
            // Independent brute force over a much wider range than any fitment: every compound x rim x width x aspect.
            std::set<std::string> expected;
            for (const auto& compound : families->compounds()) {
                for (int rim = 12; rim <= 26; ++rim) {
                    for (int width = 100; width <= 500; ++width) {
                        for (int aspect = 10; aspect <= 90; ++aspect) {
                            const rg::TyreSpec s{compound.id, width, aspect, rim};
                            if (rg::tyre_fit_error(s, fit, diameter).empty() && aspect == rg::tyre_aspect_for(rim, width, diameter)) expected.insert(rg::tyre_id(s));
                        }
                    }
                }
            }
            expected.insert(std::get<std::string>(opt.stock));
            const std::set<std::string> offered(opt.choices.begin(), opt.choices.end());
            CHECK(offered.size() == opt.choices.size()); // no duplicates
            CHECK(offered == expected);
            const int widths = (fit.max_width_mm - fit.min_width_mm) / fit.width_step_mm + 1;
            CHECK(opt.choices.size() == families->compounds().size() * static_cast<std::size_t>(widths) * fit.rims_in.size());
            // Each width sits on the fitment's step grid, between the declared limits.
            for (const auto& id : opt.choices) {
                const auto spec = rg::parse_tyre_id(id);
                REQUIRE(spec.has_value());
                CHECK(spec->width_mm >= fit.min_width_mm);
                CHECK(spec->width_mm <= fit.max_width_mm);
                CHECK((spec->width_mm - fit.min_width_mm) % fit.width_step_mm == 0);
            }
        }
    }
}

TEST_CASE("setup: road tyre technology levels retain their physical tradeoffs", "[setup][tyre_width][tyre_spec]") {
    Fixture f;
    const auto& e = f.entry("car_sedan");
    std::vector<ps::vehicle::VehicleDesc> levels;
    for (const char* family : {"cord", "belted", "touring", "economy", "sport"}) {
        const std::string key = std::string(family) + "_225_45r17";
        levels.push_back(load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", key}})));
        REQUIRE(levels.back().wheels.size() == 4);
        CHECK(levels.back().wheels[0].wheel_width == Approx(.225));
    }
    for (std::size_t i = 1; i < levels.size(); ++i) {
        CHECK(levels[i].wheels[0].tyre.lambda_mux > levels[i-1].wheels[0].tyre.lambda_mux);
        CHECK(levels[i].wheels[0].tyre.lambda_muy > levels[i-1].wheels[0].tyre.lambda_muy);
    }
    CHECK(levels[3].wheels[0].tyre.qsy1 < levels[2].wheels[0].tyre.qsy1); // eco reduces resistance
    CHECK(levels[3].wheels[0].tyre.qsy1 < levels[4].wheels[0].tyre.qsy1); // sport favors grip
    CHECK(levels[0].wheels[0].tyre.qsy4 > levels[4].wheels[0].tyre.qsy4); // low-speed fit penalizes overspeed
}

TEST_CASE("engine calibration changes real parameters and rejects excessive limits", "[engine_setup][garage]") {
    Fixture f;
    const auto& e = f.entry("car_hyper");
    auto setup = setup_of("car_hyper", {{"engine_rev_limit", 0.9}, {"engine_throttle_response", 0.5}});
    auto result = rg::compile_setup(e, f.table, f.ctx, setup);
    INFO(result.error);
    REQUIRE(result.ok);
    REQUIRE(result.patches.count("engine:hyper_v8_one1_audio") == 1);
    const auto& patch = result.patches.at("engine:hyper_v8_one1_audio");
    REQUIRE_THAT(patch, ContainsSubstring("limiter"));
    REQUIRE_THAT(patch, ContainsSubstring("7425"));
    REQUIRE_THAT(patch, ContainsSubstring("throttle_actuator_tau_s"));
    REQUIRE_THAT(patch, ContainsSubstring("0.02"));
    auto validation = rg::validate_setup(e, f.table, f.ctx, setup);
    INFO(validation.message);
    REQUIRE(validation.ok);
    REQUIRE_FALSE(rg::compile_setup(e, f.table, f.ctx,
        setup_of("car_hyper", {{"engine_rev_limit", 2.5}})).ok);  // 20625 rpm, above the 20000 rpm slider end
    REQUIRE_FALSE(rg::compile_setup(e, f.table, f.ctx,
        setup_of("car_hyper", {{"engine_rev_limit", 0.2}})).ok);  // 1650 rpm, below the 2000 rpm slider start
    // the data allows far more than the stock limit now (the setup check warns, it does not refuse)
    REQUIRE(rg::compile_setup(e, f.table, f.ctx, setup_of("car_hyper", {{"engine_rev_limit", 1.1}})).ok);
    REQUIRE_FALSE(rg::compile_setup(e, f.table, f.ctx,
        setup_of("car_hyper", {{"engine_throttle_response", 0.0}})).ok);
}


TEST_CASE("engine parts: swaps tune the newly installed engine and keep other references", "[engine_parts][garage]") {
    Fixture f;
    for (const auto& selection : {std::pair{"car_sedan", "sedan_i4"}, std::pair{"car_sedan_rwd", "sedan_i6"}}) {
        const auto& e = f.entry(selection.first);
        auto setup = setup_of(selection.first, {{"engine_install", std::string(selection.second)}, {"engine_rev_limit", 0.9}});
        std::string err;
        const auto d = load_materialised(f, e, setup, &err);
        INFO(err);
        const ps::drivetrain::TorqueMapEngineDesc* engine = nullptr;
        for (const auto& c : d.powertrain.components) if (auto p = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&c.params)) engine = p;
        REQUIRE(engine != nullptr);
        const bool i4 = std::string(selection.second) == "sedan_i4";
        CHECK(engine->limiter.rpm == Approx((i4 ? 6800.0 : 6900.0) * 0.9));
        CHECK(d.wheels.size() == 4);
        REQUIRE(gearbox_of(d) != nullptr);
        auto roundtrip = rg::parse_setup(rg::setup_to_json(setup), f.table, "roundtrip", &err);
        REQUIRE(roundtrip.has_value());
        CHECK(std::get<std::string>(roundtrip->values.at("engine_install")) == selection.second);
        const auto model = rg::build_setup_model(e, f.table, f.ctx, &setup);
        CHECK(model.error.empty());
        bool found = false;
        for (const auto& opt : model.options) if (opt.def.id == "engine_install") {
            found = true;
            REQUIRE(opt.available);
            REQUIRE(opt.parts.size() == f.ctx.engines->list().size()); // the car's own engine is the stock part
            CHECK(opt.parts.size() >= 6);
            CHECK(opt.parts.front().id == "stock");
            CHECK(!opt.parts.front().image.empty());
        }
        CHECK(found);
    }
    {   // G1: any catalog engine that fits the bay goes into any catalog car (no per-car compatible_engines list)
        const auto hyper = rg::compile_setup(f.entry("car_hyper"), f.table, f.ctx,
            setup_of("car_hyper", {{"engine_install", std::string("sedan_i4")}}));
        INFO(hyper.error);
        CHECK(hyper.ok);
        const auto too_long = rg::compile_setup(f.entry("car_hyper"), f.table, f.ctx,
            setup_of("car_hyper", {{"engine_install", std::string("sedan_i6")}}));
        CHECK_FALSE(too_long.ok);
        CHECK_THAT(too_long.error, Catch::Matchers::ContainsSubstring("801 mm long, the bay takes 700 mm"));
    }
    CHECK_FALSE(rg::compile_setup(f.entry("car_sedan"), f.table, f.ctx,
        setup_of("car_sedan", {{"engine_install", std::string("../../anything.json")}})).ok);
}

TEST_CASE("engine parts: turbo count and boost survive materialization, nitrous keeps its gate", "[engine_parts][garage]") {
    Fixture f;
    for (const auto& car : {"car_hyper", "car_hyper_n2o"}) {
        const auto& e = f.entry(car);
        const bool nitrous = std::string(car) == "car_hyper_n2o";
        const auto setup = setup_of(car, {{nitrous ? "turbo_install_n2o" : "turbo_install", std::string(nitrous ? "hyper_balanced_n2o" : "hyper_balanced")}});
        const auto tuned = load_materialised(f, e, setup);
        const auto stock = load_materialised(f, e, setup_of(car, {}));
        const ps::drivetrain::SimulatedEngineDesc *a=nullptr, *b=nullptr;
        for (const auto& c : tuned.powertrain.components) if (auto p=std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) a=p;
        for (const auto& c : stock.powertrain.components) if (auto p=std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) b=p;
        REQUIRE(a != nullptr);
        REQUIRE(b != nullptr);
        REQUIRE(a->turbo_pair.has_value());
        REQUIRE(b->turbo_pair.has_value());
        CHECK(a->turbo_pair->target_boost_pa == Approx(180000.0));
        CHECK(a->turbo_pair->wastegate_area() == Approx(b->turbo_pair->wastegate_area() * 1.44));
        CHECK(a->turbo_pair->rotor_inertia_kgm2 == Approx(b->turbo_pair->rotor_inertia_kgm2 * std::pow(1.2, 5)));
    }
    const auto twin = ps::io::load_turbo_configuration_json(kRoot + "/external/physics_sim/data/turbo_configurations/hyper_twin_default.json");
    const auto single = ps::io::load_turbo_configuration_json(kRoot + "/data/turbo_configurations/garage_hyper_single.json");
    CHECK(single.rotor_inertia_kgm2 == Approx(twin.rotor_inertia_kgm2 * 0.5));
    CHECK(single.max_mass_flow_kg_s == Approx(twin.max_mass_flow_kg_s * 0.5));
    CHECK_FALSE(rg::compile_setup(f.entry("car_hyper_n2o"), f.table, f.ctx,
        setup_of("car_hyper_n2o", {{"turbo_install", std::string("hyper_single")}})).ok);
    CHECK_FALSE(rg::compile_setup(f.entry("car_sedan"), f.table, f.ctx,
        setup_of("car_sedan", {{"turbo_install", std::string("hyper_single")}})).ok);
}

TEST_CASE("engine parts: NA removes both turbo sources and single nitrous retains gate", "[engine_parts][induction]") {
    Fixture f;
    for (const auto& car : {"car_hyper", "car_hyper_n2o"}) {
        const bool nitrous = std::string(car) == "car_hyper_n2o";
        const auto& e = f.entry(car);
        const auto na = load_materialised(f, e, setup_of(car, {{nitrous ? "turbo_install_n2o" : "turbo_install", std::string("hyper_na")}}));
        const ps::drivetrain::SimulatedEngineDesc* engine = nullptr;
        for (const auto& c : na.powertrain.components)
            if (auto p = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) engine = p;
        REQUIRE(engine != nullptr);
        CHECK_FALSE(engine->turbo_pair.has_value());
    }
    const auto twin = ps::io::load_turbo_configuration_json(kRoot + "/external/physics_sim/data/turbo_configurations/hyper_twin_n2o.json");
    const auto single = ps::io::load_turbo_configuration_json(kRoot + "/data/turbo_configurations/garage_hyper_single_n2o.json");
    CHECK(single.wastegate_area() == Approx(twin.wastegate_area() * 0.5));
    CHECK(single.rotor_inertia_kgm2 == Approx(twin.rotor_inertia_kgm2 * 0.5));
    CHECK(single.max_mass_flow_kg_s == Approx(twin.max_mass_flow_kg_s * 0.5));
}
TEST_CASE("garage turbo size and independent boost calibration reach native physics", "[garage][hardware]") {
    Fixture f;
    for (const auto* car : {"car_hyper", "car_hyper_n2o"}) {
        const bool n2o = std::string(car) == "car_hyper_n2o";
        const auto setup = setup_of(car, {{n2o ? "turbo_install_n2o" : "turbo_install", std::string(n2o ? "hyper_twin_compact_n2o" : "hyper_twin_compact")}, {"turbo_boost_target", 0.5}});
        std::string err;
        const auto d = load_materialised(f, f.entry(car), setup, &err);
        INFO(err);
        const ps::drivetrain::SimulatedEngineDesc* engine = nullptr;
        for (const auto& c : d.powertrain.components) if (auto p=std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) engine=p;
        REQUIRE(engine != nullptr);
        REQUIRE(engine->turbo_pair.has_value());
        CHECK(engine->turbo_pair->target_boost_pa == Approx(90000.0));
        CHECK(engine->turbo_pair->rotor_inertia_kgm2 == Approx(2 * 0.00023 * std::pow(0.8,5)));
        CHECK(engine->turbo_pair->max_mass_flow_kg_s == Approx(2 * 0.6 * 0.64));
        CHECK(rg::compile_setup(f.entry(car), f.table, f.ctx, setup_of(car, {{n2o ? "turbo_install_n2o" : "turbo_install", std::string("hyper_na")}, {"turbo_boost_target", 0.5}})).ok);
    }
    CHECK_FALSE(rg::compile_setup(f.entry("car_hyper"), f.table, f.ctx, setup_of("car_hyper", {{"turbo_boost_target", 20.0}})).ok);
}

TEST_CASE("N2O upgrades retain existing plumbing and reject unequipped vehicles", "[garage][hardware]") {
    Fixture f;
    for (const auto* car : {"car_hyper_n2o", "car_sedan_gen_n2o"}) {
        const bool hyper = std::string(car) == "car_hyper_n2o";
        const auto setup = setup_of(car, {{hyper ? "nitrous_install_hyper" : "nitrous_install_sedan", std::string(hyper ? "nitrous_hyper_half" : "nitrous_sedan_half")}});
        const auto validation = rg::validate_setup(f.entry(car), f.table, f.ctx, setup, false);
        INFO(validation.message);
        REQUIRE(validation.ok);
        const auto patch = rg::compile_setup(f.entry(car), f.table, f.ctx, setup);
        CHECK_THAT(patch.vehicle_patch, ContainsSubstring("garage_"));
        CHECK(rg::parse_setup(rg::setup_to_json(setup), f.table, "roundtrip", nullptr).has_value());
    }
    CHECK_FALSE(rg::compile_setup(f.entry("car_hyper"), f.table, f.ctx, setup_of("car_hyper", {{"nitrous_install_hyper", std::string("nitrous_hyper_half")}})).ok);
}

TEST_CASE("garage: the assist unit is a selectable part on the hypercars", "[garage][assist]") {
    Fixture f;
    // part id -> traction_modes_available_bits of the unit (Racing is the stock part of both hypers)
    const std::pair<const char*, std::uint32_t> grades[] = {{"assists_street", 0b00011}, {"assists_sport", 0b01111}, {"stock", 0b11111},
                                                            {"assists_street_abs_esc", 0b00011}, {"assists_sport_abs_esc", 0b01111}, {"assists_racing_abs_esc", 0b11111}};
    for (const char* car : {"car_hyper", "car_hyper_n2o"}) {
        INFO(car);
        const auto& e = f.entry(car);
        const rg::SetupModel model = rg::build_setup_model(e, f.table, f.ctx);
        REQUIRE(model.error.empty());
        const rg::OptionView* view = nullptr;
        for (const auto& o : model.options) if (o.def.id == "assist_unit") view = &o;
        REQUIRE(view != nullptr);
        CHECK(view->available);
        REQUIRE(view->choices.size() == 6);
        CHECK(view->choices.front() == "stock");
        for (const auto& [part, bits] : grades) {
            INFO(part);
            const auto setup = setup_of(car, {{"assist_unit", std::string(part)}});
            const auto v = rg::validate_setup(e, f.table, f.ctx, setup, false);
            INFO(v.message);
            REQUIRE(v.ok);
            // Materialise and load validate-only (no engine-map generation): the unit, not the maps, is under test.
            rg::ScopedWorkDir dir(f.ctx.work_root, std::string("t_assist_") + car);
            const rg::MaterialisedSetup m = rg::materialise_setup(e, f.table, f.ctx, setup, dir.path());
            INFO(m.error);
            REQUIRE(m.ok);
            ps::io::EngineMapOptions options;
            options.generate = false;
            const ps::vehicle::VehicleDesc desc = rg::load_vehicle(m.vehicle_path, options);
            REQUIRE(desc.assist_unit.has_value());
            CHECK(ps::vehicle::traction_modes_available_bits(desc.assist_unit, desc.traction_control.fitted) == bits);
            // Only the Racing units carry the wing automation.
            const std::string id = part;
            CHECK(ps::vehicle::find_aero_control_surface(desc.aero, desc.assist_unit).has_value() == (id == "stock" || id == "assists_racing_abs_esc"));
            // ABS and ESC only come with the + ABS/ESC variants (A5).
            const bool with_abs_esc = id.find("_abs_esc") != std::string::npos;
            CHECK(desc.assist_unit->abs.has_value() == with_abs_esc);
            CHECK(desc.assist_unit->esc.has_value() == with_abs_esc);
        }
    }
}

TEST_CASE("garage: the sedans offer an assist unit that is added to the stock file", "[garage][assist][setup]") {
    Fixture f;
    const std::pair<const char*, std::uint32_t> grades[] = {{"assists_street", 0b00011}, {"assists_sport", 0b01111}, {"assists_racing", 0b11111},
                                                            {"assists_street_abs_esc", 0b00011}, {"assists_racing_abs_esc", 0b11111}};
    for (const char* car : {"car_sedan", "car_sedan_gen_n2o"}) {
        INFO(car);
        const auto& e = f.entry(car);
        const rg::SetupModel model = rg::build_setup_model(e, f.table, f.ctx);
        REQUIRE(model.error.empty());
        const rg::OptionView* view = nullptr;
        for (const auto& o : model.options) if (o.def.id == "assist_unit") view = &o;
        REQUIRE(view != nullptr);
        CHECK(view->available);
        REQUIRE(view->choices.size() == 7);
        CHECK(view->choices.front() == "stock");
        CHECK(view->parts.front().label == "No assist unit");
        // Stock materialises without an assists entry.
        {
            rg::ScopedWorkDir dir(f.ctx.work_root, std::string("t_assist_stock_") + car);
            const auto m = rg::materialise_setup(e, f.table, f.ctx, setup_of(car, {{"assist_unit", std::string("stock")}}), dir.path());
            INFO(m.error);
            REQUIRE(m.ok);
            ps::io::EngineMapOptions options;
            options.generate = false;
            const ps::vehicle::VehicleDesc desc = rg::load_vehicle(m.vehicle_path, options);
            CHECK_FALSE(desc.assist_unit.has_value());
            const auto compiled = rg::compile_setup(e, f.table, f.ctx, setup_of(car, {{"assist_unit", std::string("stock")}}));
            CHECK(compiled.ok);
            CHECK(compiled.vehicle_patch.find("assists") == std::string::npos);
        }
        for (const auto& [part, bits] : grades) {
            INFO(part);
            const auto setup = setup_of(car, {{"assist_unit", std::string(part)}});
            const auto v = rg::validate_setup(e, f.table, f.ctx, setup, false);
            INFO(v.message);
            REQUIRE(v.ok);
            rg::ScopedWorkDir dir(f.ctx.work_root, std::string("t_assist_") + car);
            const rg::MaterialisedSetup m = rg::materialise_setup(e, f.table, f.ctx, setup, dir.path());
            INFO(m.error);
            REQUIRE(m.ok);
            ps::io::EngineMapOptions options;
            options.generate = false;
            const ps::vehicle::VehicleDesc desc = rg::load_vehicle(m.vehicle_path, options);
            REQUIRE(desc.assist_unit.has_value());
            CHECK(ps::vehicle::traction_modes_available_bits(desc.assist_unit, desc.traction_control.fitted) == bits);
        }
        CHECK(rg::parse_setup(rg::setup_to_json(setup_of(car, {{"assist_unit", std::string("assists_sport")}})), f.table, "roundtrip", nullptr).has_value());
    }
}

TEST_CASE("garage pressure domain follows the configured engine and absolute boost", "[adaptive_setup]") {
    Fixture f;
    for (const double boost : {0.1, 3.36, 30.0}) {
        auto setup=setup_of("car_hyper", {{"turbo_boost_target", boost/1.8}});
        const auto compiled=rg::compile_setup(f.entry("car_hyper"),f.table,f.ctx,setup);
        INFO(compiled.error);
        REQUIRE(compiled.ok);
        bool found=false;
        for(const auto& [name,patch]:compiled.patches) if(name.find("engine:")==0) {
            found=true;
            CHECK_THAT(patch,ContainsSubstring("pressure_sampling"));
            CHECK_THAT(patch,ContainsSubstring("max_kpa"));
        }
        CHECK(found);
        const auto validated=rg::validate_setup(f.entry("car_hyper"),f.table,f.ctx,setup,false);
        INFO(validated.message);
        CHECK(validated.ok);
    }
}

TEST_CASE("setup: clutch upgrade changes transmitted torque capacity and respects limits", "[setup][garage][clutch_upgrade]") {
    Fixture f;
    for(const char* id:{"car_hyper","car_hyper_n2o","car_sedan"}) {
        const auto& entry=f.entry(id);
        const auto model=rg::build_setup_model(entry,f.table,f.ctx);
        const auto view=std::find_if(model.options.begin(),model.options.end(),[](const auto& v){return v.def.id=="clutch_capacity";});
        REQUIRE(view!=model.options.end());REQUIRE(view->available);REQUIRE(view->stock_numbers.size()==1);
        for(double factor:{1.,1.5,2.,3.,5.,8.}) {
            auto setup=setup_of(id,{{"clutch_capacity",factor}});
            const auto desc=load_materialised(f,entry,setup);
            const auto clutch=std::find_if(desc.powertrain.components.begin(),desc.powertrain.components.end(),[](const auto& c){return c.id=="clutch";});
            REQUIRE(clutch!=desc.powertrain.components.end());
            const auto* values=std::get_if<ps::drivetrain::ClutchDesc>(&clutch->params);
            REQUIRE(values!=nullptr);
            CHECK(values->capacity_nm==Approx(view->stock_numbers[0]*factor));
        }
        for(double invalid:{.5,8.5}) {
            const auto compiled=rg::compile_setup(entry,f.table,f.ctx,setup_of(id,{{"clutch_capacity",invalid}}));
            CHECK_FALSE(compiled.ok);
        }
    }
}

TEST_CASE("setup: the Runabout's folding top is a visual model option, offered only where the model has it", "[setup][garage][model_option]") {
    Fixture f;
    const auto offered = [&](const char* car) {
        const rg::SetupModel m = rg::build_setup_model(f.entry(car), f.table, f.ctx);
        REQUIRE(m.error.empty());
        for (const auto& o : m.options) {
            if (o.def.id == "folding_top") return o.available && std::get<bool>(o.stock) == false;
        }
        return false;
    };
    CHECK(offered("car_runabout"));
    CHECK_FALSE(offered("car_sedan")); // no such part on its model
    CHECK_FALSE(offered("car_gt70"));

    rg::VehicleSetup s;
    s.vehicle_id = "car_runabout";
    s.values["folding_top"] = true;
    const rg::CompiledSetup on = rg::compile_setup(f.entry("car_runabout"), f.table, f.ctx, s);
    INFO(on.error);
    REQUIRE(on.ok);
    CHECK(on.model_options == std::vector<std::string>{"roof"});
    CHECK(on.vehicle_patch == "{}"); // visual only: no physics file changes
    CHECK(on.patches.empty());

    s.vehicle_id = "car_sedan";
    const rg::CompiledSetup refused = rg::compile_setup(f.entry("car_sedan"), f.table, f.ctx, s);
    CHECK_FALSE(refused.ok);
}
