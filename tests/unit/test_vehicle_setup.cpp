// test_vehicle_setup.cpp - R6 setup overlays (racing_game.vehicle_setup/1):
// every whitelisted change round-trips through ps::io::load_vehicle_json, an
// out-of-range or invalid overlay is rejected with the loader's/rule's message,
// the whitelist is enforced on the merge patch, temp files are removed.
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <initializer_list>
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
        ctx.tyre_dirs = {kRoot + "/external/physics_sim/data/tyres", kRoot + "/data/tyres"};
        ctx.work_root = isolated_root.path();
        ctx.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
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
    return ps::io::load_vehicle_json(m.vehicle_path, options);
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
    const ps::vehicle::VehicleDesc stock = ps::io::load_vehicle_json(e.vehicle_path);
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
    const ps::vehicle::VehicleDesc stock = ps::io::load_vehicle_json(e.vehicle_path);
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
    const ps::vehicle::VehicleDesc stock = ps::io::load_vehicle_json(e.vehicle_path);
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

TEST_CASE("setup: the AWD final drive scales both axle differentials, not the centre", "[setup][garage]") {
    Fixture f;
    const auto& e = f.entry("car_sedan_awd");
    const ps::vehicle::VehicleDesc stock = ps::io::load_vehicle_json(e.vehicle_path);
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

TEST_CASE("setup: tyre choice offers files that match the wheel size", "[setup][garage]") {
    Fixture f;
    const auto& e = f.entry("car_sedan");
    const rg::SetupModel model = rg::build_setup_model(e, f.table, f.ctx);
    REQUIRE(model.error.empty());
    const rg::OptionView* tf = nullptr;
    for (const auto& o : model.options) {
        if (o.def.id == "tyre_front") tf = &o;
    }
    REQUIRE(tf != nullptr);
    CHECK(tf->available);
    CHECK(std::get<std::string>(tf->stock) == "passenger_car_225_45r17");
    CHECK(tf->choices.size() >= 3);
    for (const auto& c : tf->choices) CHECK(c.find("hyper") == std::string::npos); // wrong size never offered

    const ps::vehicle::VehicleDesc d =
        load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", std::string("passenger_sport_225_45r17")}}));
    const ps::vehicle::VehicleDesc stock = ps::io::load_vehicle_json(e.vehicle_path);
    CHECK(d.wheels[0].tyre.lambda_mux == Approx(stock.wheels[0].tyre.lambda_mux * 1.08));
    CHECK(d.wheels[2].tyre.lambda_mux == Approx(stock.wheels[2].tyre.lambda_mux)); // rear untouched

    std::string err;
    load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", std::string("hyper_front_265_35r20")}}), &err);
    CHECK_THAT(err, ContainsSubstring("not a tyre that fits"));
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
                                                  {"tyre_front", std::string("passenger_sport_225_45r17")},
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
            switch (o.def.kind) {
                case rg::OptionKind::Scale: s.values[o.def.id] = o.max; ++touched; break;
                case rg::OptionKind::ScaleList:
                    s.values[o.def.id] = std::vector<double>(static_cast<std::size_t>(o.list_size), 1.05);
                    ++touched;
                    break;
                case rg::OptionKind::BrakeBias: s.values[o.def.id] = std::get<double>(o.stock) + 0.02; ++touched; break;
                case rg::OptionKind::FileChoice:
                case rg::OptionKind::TyreChoice: s.values[o.def.id] = o.choices.back(); ++touched; break;
                case rg::OptionKind::Bool: s.values[o.def.id] = !std::get<bool>(o.stock); ++touched; break;
                case rg::OptionKind::Colour: s.values[o.def.id] = std::string("#336699"); break;
            }
        }
        CHECK(touched >= 10);
        const auto v = rg::validate_setup(e, f.table, f.ctx, s);
        INFO(v.message);
        CHECK(v.ok);
    }
}

TEST_CASE("setup: tyre width changes geometry, grip and resistance together", "[setup][tyre_width]") {
    Fixture f;
    const auto& e = f.entry("car_sedan");
    const auto narrow = load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", std::string("passenger_road_w195_r17")}}));
    const auto wide = load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", std::string("passenger_road_w255_r17")}}));
    const auto stock = ps::io::load_vehicle_json(e.vehicle_path);
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
    const auto saved = rg::parse_setup(rg::setup_to_json(setup_of("car_sedan", {{"tyre_front", std::string("passenger_road_w255_r17")}})), f.table, "width save test", &saved_error);
    REQUIRE(saved.has_value());
    const auto reloaded = load_materialised(f, e, *saved);
    CHECK(reloaded.wheels[0].wheel_width == Approx(.255));
    CHECK(reloaded.wheels[0].tyre.lambda_mux == Approx(wide.wheels[0].tyre.lambda_mux));
    CHECK(wide.wheels[2].wheel_width == Approx(stock.wheels[2].wheel_width));
    CHECK(wide.wheels[2].tyre.lambda_mux == Approx(stock.wheels[2].tyre.lambda_mux));
    std::string error;
    load_materialised(f, e, setup_of("car_sedan", {{"tyre_front", std::string("passenger_road_w265_r17")}}), &error);
    CHECK_THAT(error, ContainsSubstring("not a tyre that fits"));
    // Catalog range changes the limit without changing compiler code.
    auto enlarged = e;
    enlarged.setup_ranges["tyre_front"] = {195, 275};
    const auto expanded = load_materialised(f, enlarged, setup_of("car_sedan", {{"tyre_front", std::string("passenger_road_w275_r17")}}));
    REQUIRE(expanded.wheels.size() == 4);
    CHECK(expanded.wheels[0].wheel_width == Approx(.275));
}

TEST_CASE("setup: each tyre family offers width variants within its axle body envelope", "[setup][tyre_width]") {
    Fixture f;
    const auto& e = f.entry("car_hyper");
    const auto model = rg::build_setup_model(e, f.table, f.ctx);
    REQUIRE(model.error.empty());
    for (const auto& opt : model.options) {
        if (opt.def.id != "tyre_front" && opt.def.id != "tyre_rear") continue;
        const bool front = opt.def.id == "tyre_front";
        const std::string prefix = front ? "hyper_front_" : "hyper_rear_";
        const std::string rim = front ? "_r20" : "_r21";
        const int minimum = front ? 235 : 275;
        const int maximum = front ? 305 : 365;
        for (const char* family : {"cord", "belted", "touring", "economy", "sport", "road", "track", "slick", "drag"}) {
            for (int width = minimum; width <= maximum; width += 10) {
                const auto key = prefix + family + "_w" + std::to_string(width) + rim;
                CHECK(std::find(opt.choices.begin(), opt.choices.end(), key) != opt.choices.end());
            }
            const auto too_wide = prefix + family + "_w" + std::to_string(maximum + 10) + rim;
            CHECK(std::find(opt.choices.begin(), opt.choices.end(), too_wide) == opt.choices.end());
        }
    }
}

TEST_CASE("setup: road tyre technology levels retain their physical tradeoffs", "[setup][tyre_width]") {
    Fixture f;
    const auto& e = f.entry("car_sedan");
    std::vector<ps::vehicle::VehicleDesc> levels;
    for (const char* family : {"cord", "belted", "touring", "economy", "sport"}) {
        const std::string key = std::string("passenger_") + family + "_w225_r17";
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
        setup_of("car_hyper", {{"engine_rev_limit", 1.1}})).ok);
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
            REQUIRE(opt.parts.size() == 2);
            CHECK(opt.parts.front().id == "stock");
            CHECK(!opt.parts.front().image.empty());
        }
        CHECK(found);
    }
    CHECK_FALSE(rg::compile_setup(f.entry("car_hyper"), f.table, f.ctx,
        setup_of("car_hyper", {{"engine_install", std::string("sedan_i6")}})).ok);
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
