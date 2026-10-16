// test_engine_internals.cpp - swappable engine internals (rg/engine_internals.h; owner on PR #21: the safe piston speed
// depends on the parts' material, a titanium rod as in F1 holds higher speeds, and such parts should be swappable).
#include "rg/engine_internals.h"
#include "rg/engine_store.h"
#include "rg/limit_advisor.h"
#include "rg/part_mass.h"
#include "rg/settings.h"
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/limit_findings.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
const std::string kRoot = RG_SOURCE_DIR;
const std::string kFile = kRoot + "/data/engines/engine_internals.json";

rg::EngineInternalsConfig internals() {
    std::string err;
    auto c = rg::load_engine_internals(kFile, &err);
    INFO(err);
    REQUIRE(c.has_value());
    return *c;
}

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    std::shared_ptr<rg::EngineStore> store;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_engine_internals_tests"};
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
        ctx.engine_internals = std::make_shared<const rg::EngineInternalsConfig>(internals());
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

rg::AdvisorConfig advisor() {
    std::string err;
    auto adv = rg::load_advisor_config(kRoot + "/data/garage/advisor.json", &err);
    INFO(err);
    REQUIRE(adv.has_value());
    return *adv;
}

const rg::AdviceItem* find_item(const std::vector<rg::AdviceItem>& items, const std::string& name) {
    for (const rg::AdviceItem& i : items) if (i.rule_name == name || i.kind_name == name) return &i;
    return nullptr;
}

ps::drivetrain::DynoPoint point_at(double rpm) {
    ps::drivetrain::DynoPoint p;
    p.rpm = static_cast<float>(rpm);
    return p;
}

} // namespace

TEST_CASE("engine internals: the shipped table loads, every grade has an [AS] source, stock first", "[engine_internals]") {
    const rg::EngineInternalsConfig c = internals();
    CHECK(c.stock_margin > 1.0);
    for (const char* kind : {"connecting_rods", "pistons", "crankshaft", "valve_springs"}) {
        INFO(kind);
        const rg::InternalPartKind* k = c.find(kind);
        REQUIRE(k != nullptr);
        REQUIRE(k->grades.size() >= 2);
        CHECK(k->grades.front().id == "stock");
        CHECK_THAT(k->mass_source, ContainsSubstring("[AS]"));
        for (const rg::InternalGrade& g : k->grades) {
            INFO(g.id);
            CHECK_THAT(g.source, ContainsSubstring("[AS]"));
            // an upgrade is rated above the table's stock part; a downgrade (below stock) is allowed and has its own id
            if (g.id != "stock") CHECK(g.limit != k->grades.front().limit);
        }
    }
    CHECK(c.find("valve_springs")->limit_kind == rg::InternalLimitKind::LimiterFactor);
    CHECK(c.find("connecting_rods")->find("titanium") != nullptr);
    CHECK(c.find("connecting_rods")->find("titanium")->mass_factor < 1.0);
    CHECK(c.find("crankshaft")->find("billet_steel") != nullptr);

    // Strict loader: the stock grade must come first; one limiter-factor part at most.
    std::ifstream in(kFile, std::ios::binary);
    nlohmann::json doc = nlohmann::json::parse(in);
    nlohmann::json swapped = doc;
    auto& grades = swapped["parts"][0]["grades"];
    std::swap(grades[0], grades[1]);
    std::string err;
    CHECK_FALSE(rg::parse_engine_internals(swapped.dump(), "t", &err).has_value());
    CHECK_THAT(err, ContainsSubstring("stock"));
    nlohmann::json two = doc;
    two["parts"][0]["limit_kind"] = "limiter_factor";
    two["parts"][0]["grades"] = two["parts"][3]["grades"];
    CHECK_FALSE(rg::parse_engine_internals(two.dump(), "t", &err).has_value());
}

TEST_CASE("engine internals: stock parts hold the factory limiter on every engine, upgrades never rate lower",
          "[engine_internals]") {
    Fixture f;
    const rg::EngineInternalsConfig& c = *f.ctx.engine_internals;
    const rg::AdvisorConfig adv = advisor();
    int rated = 0;
    for (const rg::EngineEntry& e : f.store->list()) {
        if (e.user) continue;
        const auto facts = rg::read_engine_facts(e.file, nullptr);
        REQUIRE(facts.has_value());
        INFO(e.key);
        if (!rg::engine_internals_refusal(*facts, e.name).empty()) continue;
        ++rated;
        std::string err;
        const auto stock = rg::describe_engine_internals(c, *facts, {}, {}, &err);
        INFO(err);
        REQUIRE(stock.has_value());
        const double factory = 2.0 * facts->stroke_m * facts->limiter_rpm / 60.0;
        CHECK(stock->factory_piston_speed_m_s == Approx(factory));
        const rg::InternalPartBuild* weak = stock->weakest_piston_speed();
        REQUIRE(weak != nullptr);
        CHECK(weak->limit_rpm >= c.stock_margin * facts->limiter_rpm - 1e-6);
        REQUIRE(stock->valve_springs() != nullptr);
        CHECK(stock->valve_springs()->limit >= c.stock_margin * facts->limiter_rpm - 1e-6);
        CHECK(stock->mass_delta_kg() == Approx(0.0));
        // As built, the setup check says nothing at the factory limiter.
        rg::CompiledSetup cs;
        cs.internals = *stock;
        rg::AdviceInput in;
        rg::set_part_limits(in, cs);
        in.stroke_m = facts->stroke_m;
        in.points.push_back(point_at(facts->limiter_rpm));
        const auto items = rg::build_advice(in, adv);
        CHECK(find_item(items, "MeanPistonSpeed") == nullptr);
        CHECK(find_item(items, "ValveFloat") == nullptr);
        // Every grade on this engine rates at or above stock.
        for (const rg::InternalPartKind& k : c.parts) {
            const double s = rg::internal_grade_limit(c, *facts, k.kind, "stock");
            for (const rg::InternalGrade& g : k.grades) {
                if (g.limit >= k.grades.front().limit) CHECK(rg::internal_grade_limit(c, *facts, k.kind, g.id) >= s);
                else CHECK(rg::internal_grade_limit(c, *facts, k.kind, g.id) < s); // a downgrade rates below stock
            }
        }
    }
    CHECK(rated >= 3);
}

TEST_CASE("engine internals: the weakest installed part sets the piston-speed limit", "[engine_internals]") {
    Fixture f;
    const rg::EngineInternalsConfig& c = *f.ctx.engine_internals;
    const auto facts = rg::read_engine_facts(f.store->find("default:pico_i3_1l0_na")->file, nullptr);
    REQUIRE(facts.has_value());
    std::string err;
    const auto stock = rg::describe_engine_internals(c, *facts, {}, {}, &err);
    REQUIRE(stock.has_value());
    // Titanium rods alone: the rods are no longer the weak point of a stock engine's bottom end unless nothing else is lower.
    const auto ti = rg::describe_engine_internals(c, *facts, {{"connecting_rods", "titanium"}}, {}, &err);
    REQUIRE(ti.has_value());
    CHECK(ti->weakest_piston_speed()->limit_rpm >= stock->weakest_piston_speed()->limit_rpm);
    CHECK(ti->weakest_piston_speed()->kind != "connecting_rods");
    // Titanium rods, forged pistons and a billet crank raise the engine's safe rpm over stock.
    const auto all = rg::describe_engine_internals(
        c, *facts, {{"connecting_rods", "titanium"}, {"pistons", "forged"}, {"crankshaft", "billet_steel"}}, {}, &err);
    REQUIRE(all.has_value());
    CHECK(all->weakest_piston_speed()->limit_rpm > stock->weakest_piston_speed()->limit_rpm);
    // Masses: the difference to stock per part, by displacement.
    const double rods_stock = c.find("connecting_rods")->stock_mass_per_litre_kg * facts->displacement_l;
    const rg::InternalPartBuild& rods = all->parts.front();
    CHECK(rods.kind == "connecting_rods");
    CHECK(rods.mass_delta_kg == Approx(rods_stock * (c.find("connecting_rods")->find("titanium")->mass_factor - 1.0)));
    CHECK(rods.mass_delta_kg < 0.0);
    // Unknown grade.
    CHECK_FALSE(rg::describe_engine_internals(c, *facts, {{"connecting_rods", "unobtainium"}}, {}, &err).has_value());
    CHECK_THAT(err, ContainsSubstring("unobtainium"));
}

TEST_CASE("engine internals: garage options, compiled limits and the part-mass tree", "[engine_internals][part_mass]") {
    Fixture f;
    const rg::CatalogEntry& gt = f.entry("car_gt70");
    // The option views: every grade of the kind, the stock card named after the stock material.
    const rg::SetupModel model = rg::build_setup_model(gt, f.table, f.ctx);
    INFO(model.error);
    REQUIRE(model.error.empty());
    int seen = 0;
    for (const rg::OptionView& v : model.options) {
        if (v.def.catalog != "engine_internals") continue;
        INFO(v.def.id);
        ++seen;
        CHECK(v.available);
        const rg::InternalPartKind* k = f.ctx.engine_internals->find(v.def.internal);
        REQUIRE(k != nullptr);
        REQUIRE(v.parts.size() == k->grades.size());
        CHECK(v.parts.front().id == "stock");
        CHECK(v.parts.front().label == k->grades.front().label);
        for (const rg::SetupPart& p : v.parts) CHECK_FALSE(p.detail.empty());
    }
    CHECK(seen == 4);

    const rg::CompiledSetup stock = rg::compile_setup(gt, f.table, f.ctx, setup_of("car_gt70", {}));
    INFO(stock.error);
    REQUIRE(stock.ok);
    REQUIRE(stock.parts.ok);
    REQUIRE(stock.internals.has_value());
    CHECK(stock.parts.root.find("engine.internals") == nullptr);
    CHECK(stock.internals->weakest_piston_speed()->option_id == "engine_rods"); // ties: the first part in the table

    const rg::CompiledSetup ti = rg::compile_setup(gt, f.table, f.ctx, setup_of("car_gt70", {{"engine_rods", std::string("titanium")}}));
    INFO(ti.error);
    REQUIRE(ti.ok);
    REQUIRE(ti.parts.ok);
    // No engine data changes: limits and masses only.
    const auto patch = ti.patches.find("engine");
    CHECK((patch == ti.patches.end() || patch->second == "{}" || patch->second == "null"));
    const rg::PartNode* node = ti.parts.root.find("engine.internals.connecting_rods");
    REQUIRE(node != nullptr);
    CHECK(node->own_kg < 0.0);
    CHECK_THAT(node->label, ContainsSubstring("Titanium"));
    CHECK(ti.parts.total_kg == Approx(stock.parts.total_kg + node->own_kg));
    CHECK(ti.parts.chassis_kg == Approx(stock.parts.chassis_kg + node->own_kg));
    REQUIRE(ti.internals.has_value());
    CHECK(ti.internals->parts.front().grade == "titanium");

    // The model's tree follows the installed choice too.
    const rg::VehicleSetup installed = setup_of("car_gt70", {{"engine_rods", std::string("titanium")}});
    const rg::SetupModel m2 = rg::build_setup_model(gt, f.table, f.ctx, &installed);
    REQUIRE(m2.parts.ok);
    CHECK(m2.parts.root.find("engine.internals.connecting_rods") != nullptr);

    // A torque-curve engine has no modelled internals: unavailable with the reason, a saved choice is refused.
    const rg::CatalogEntry& sedan = f.entry("car_sedan");
    const rg::SetupModel sm = rg::build_setup_model(sedan, f.table, f.ctx);
    for (const rg::OptionView& v : sm.options) {
        if (v.def.catalog != "engine_internals") continue;
        INFO(v.def.id);
        CHECK_FALSE(v.available);
        CHECK_THAT(v.unavailable_reason, ContainsSubstring("torque-curve"));
    }
    const rg::CompiledSetup bad = rg::compile_setup(sedan, f.table, f.ctx, setup_of("car_sedan", {{"engine_rods", std::string("titanium")}}));
    CHECK_FALSE(bad.ok);
    CHECK_THAT(bad.error, ContainsSubstring("torque-curve"));
    // Without an internals table the options are simply not offered.
    rg::SetupContext bare = f.ctx;
    bare.engine_internals.reset();
    const rg::CompiledSetup none = rg::compile_setup(gt, f.table, bare, setup_of("car_gt70", {}));
    REQUIRE(none.ok);
    CHECK_FALSE(none.internals.has_value());
}

TEST_CASE("engine internals: the setup check names the part that would fail", "[engine_internals][limit_advisor]") {
    Fixture f;
    const rg::AdvisorConfig adv = advisor();
    const rg::CatalogEntry& gt = f.entry("car_gt70");
    // The Ventura V8: 76.2 mm stroke, factory limiter 6300 rpm (16 m/s): stock parts hold 21 m/s (8268 rpm), stock springs
    // 1.15 x 6300 = 7245 rpm, race springs 1.45 x 6300 = 9135 rpm.
    const auto items_at = [&](const rg::VehicleSetup& s, double rpm, bool physics_finding) {
        const rg::CompiledSetup c = rg::compile_setup(gt, f.table, f.ctx, s);
        INFO(c.error);
        REQUIRE(c.ok);
        REQUIRE(c.internals.has_value());
        rg::AdviceInput in;
        rg::set_part_limits(in, c);
        in.stroke_m = c.internals->stroke_m;
        in.points.push_back(point_at(rpm));
        if (physics_finding) {
            ps::drivetrain::LimitFinding fnd;
            fnd.kind = ps::drivetrain::LimitKind::MeanPistonSpeed;
            fnd.severity = ps::drivetrain::LimitSeverity::Limiting;
            fnd.rpm = static_cast<float>(rpm);
            fnd.value = 26.0f;
            fnd.limit = 25.0f;
            fnd.unit = ps::drivetrain::LimitUnit::MetrePerSecond;
            fnd.part.field = "/limiter/rpm";
            in.findings.push_back(fnd);
        }
        return rg::build_advice(in, adv);
    };
    // Stock parts at 9000 rpm (22.9 m/s): the rods fail first (ties: first in the table), valves float.
    const auto stock = items_at(setup_of("car_gt70", {}), 9000.0, true);
    const rg::AdviceItem* piston = find_item(stock, "MeanPistonSpeed");
    REQUIRE(piston != nullptr);
    CHECK(piston->rule_name == "MeanPistonSpeed"); // the game's part rule; physics' table finding is dropped
    int piston_items = 0;
    for (const rg::AdviceItem& i : stock) piston_items += i.kind_name == "MeanPistonSpeed" ? 1 : 0;
    CHECK(piston_items == 1);
    CHECK(piston->severity == rg::AdviceSeverity::Warning);
    CHECK(piston->damage_part == "connecting_rods");
    CHECK(piston->option_id == "engine_rods");
    REQUIRE_FALSE(piston->lines.empty());
    CHECK_THAT(piston->lines.front(), ContainsSubstring("connecting rods"));
    CHECK_THAT(piston->suggestion, ContainsSubstring("connecting rods"));
    const rg::AdviceItem* valves = find_item(stock, "ValveFloat");
    REQUIRE(valves != nullptr);
    CHECK(valves->damage_part == "valve_springs");
    CHECK(valves->option_id == "engine_valve_springs");
    CHECK_THAT(valves->lines.front(), ContainsSubstring("valve springs"));

    // Titanium rods alone move the weak point to the pistons; still a warning.
    const auto ti = items_at(setup_of("car_gt70", {{"engine_rods", std::string("titanium")}}), 9000.0, false);
    const rg::AdviceItem* p2 = find_item(ti, "MeanPistonSpeed");
    REQUIRE(p2 != nullptr);
    CHECK(p2->damage_part == "pistons");
    CHECK(p2->option_id == "engine_pistons");

    // The full race set and race springs at 8000 rpm: no piston-speed or valve item (physics' finding dropped).
    const auto race = items_at(setup_of("car_gt70", {{"engine_rods", std::string("titanium")}, {"engine_pistons", std::string("forged")},
                                                     {"engine_crankshaft", std::string("billet_steel")}, {"engine_valve_springs", std::string("race")}}),
                               8000.0, true);
    CHECK(find_item(race, "MeanPistonSpeed") == nullptr);
    CHECK(find_item(race, "ValveFloat") == nullptr);

    // Part-limit items of the turbo kit carry their part too (the damage-model hook).
    rg::AdviceInput fallback;
    fallback.stroke_m = 0.09;
    fallback.points.push_back(point_at(9000.0)); // 27 m/s, no internals data: the engine-wide 25 m/s figure
    const auto fb = rg::build_advice(fallback, adv);
    const rg::AdviceItem* p3 = find_item(fb, "MeanPistonSpeed");
    REQUIRE(p3 != nullptr);
    CHECK(p3->damage_part == "engine_internals");
    CHECK(p3->option_id == "engine_rev_limit");
}

TEST_CASE("engine internals: downgrades rate below stock, warn at the stock limiter and weigh differently", "[engine_internals]") {
    Fixture f;
    const rg::EngineInternalsConfig& c = *f.ctx.engine_internals;
    const rg::AdvisorConfig adv = advisor();
    // Every part has at least one grade below stock, with a [AS] source and a rating under the stock table figure.
    for (const char* kind : {"connecting_rods", "pistons", "crankshaft", "valve_springs"}) {
        const rg::InternalPartKind* k = c.find(kind);
        REQUIRE(k != nullptr);
        int down = 0;
        for (const rg::InternalGrade& g : k->grades) {
            if (g.limit < k->grades.front().limit) {
                ++down;
                CHECK_THAT(g.source, ContainsSubstring("[AS]"));
            }
        }
        INFO(kind);
        CHECK(down >= 1);
    }
    const rg::InternalPartKind* rods = c.find("connecting_rods");
    REQUIRE(rods->find("cast_steel") != nullptr);
    CHECK(rods->find("cast_steel")->limit < rods->grades.front().limit);

    const auto facts = rg::read_engine_facts(f.store->find("default:pico_i3_1l0_na")->file, nullptr);
    REQUIRE(facts.has_value());
    std::string err;
    for (const char* spec : {"connecting_rods:cast_steel", "pistons:low_silicon_cast", "crankshaft:lightweight_cast", "valve_springs:weak_single"}) {
        const std::string s = spec;
        const std::string kind = s.substr(0, s.find(':')), grade = s.substr(s.find(':') + 1);
        INFO(spec);
        const double stock_lim = rg::internal_grade_limit(c, *facts, kind, "stock");
        const double lim = rg::internal_grade_limit(c, *facts, kind, grade);
        CHECK(lim < stock_lim);
        const auto b = rg::describe_engine_internals(c, *facts, {{kind, grade}}, {}, &err);
        REQUIRE(b.has_value());
        CHECK_THAT(rg::internal_grade_detail(c, *facts, kind, grade), ContainsSubstring("below stock"));
        // Mass: the mass_factor sign decides the delta; cast steel and low-silicon pistons are heavier, the other two lighter.
        const rg::InternalPartKind* k = c.find(kind);
        const double factor = k->find(grade)->mass_factor;
        for (const rg::InternalPartBuild& p : b->parts) if (p.kind == kind) CHECK((p.mass_delta_kg < 0.0) == (factor < 1.0));
    }
    // Valve springs: the weak single springs float below the factory limiter itself.
    const auto weak = rg::describe_engine_internals(c, *facts, {{"valve_springs", "weak_single"}}, {}, &err);
    REQUIRE(weak.has_value());
    CHECK(weak->valve_springs()->limit < facts->limiter_rpm);
    
    // Setup check on the Pico at its own limiter: stock says nothing, each downgrade that the limiter exceeds warns and
    // names its part.
    const auto warns = [&](const std::map<std::string, std::string>& grades, const char*) {
        const auto b = rg::describe_engine_internals(c, *facts, grades, {{"connecting_rods", "engine_rods"}, {"valve_springs", "engine_valve_springs"}}, &err);
        REQUIRE(b.has_value());
        rg::CompiledSetup cs;
        cs.internals = *b;
        rg::AdviceInput in;
        rg::set_part_limits(in, cs);
        in.stroke_m = facts->stroke_m;
        in.points.push_back(point_at(facts->limiter_rpm));
        return rg::build_advice(in, adv);
    };
    const auto none = warns({}, "");
    CHECK(find_item(none, "MeanPistonSpeed") == nullptr);
    CHECK(find_item(none, "ValveFloat") == nullptr);
    const auto cast = rg::describe_engine_internals(c, *facts, {{"connecting_rods", "cast_steel"}}, {}, &err);
    REQUIRE(cast.has_value());
    REQUIRE(cast->parts.front().limit_rpm < facts->limiter_rpm); // the cast-steel rods give out below the stock limiter
    if (cast->parts.front().limit_rpm < facts->limiter_rpm) {
        const auto items = warns({{"connecting_rods", "cast_steel"}}, "MeanPistonSpeed");
        const rg::AdviceItem* it = find_item(items, "MeanPistonSpeed");
        REQUIRE(it != nullptr);
        CHECK(it->damage_part == "connecting_rods");
        CHECK(it->option_id == "engine_rods");
    }
    const auto sp = warns({{"valve_springs", "weak_single"}}, "ValveFloat");
    const rg::AdviceItem* vf = find_item(sp, "ValveFloat");
    REQUIRE(vf != nullptr);
    CHECK(vf->damage_part == "valve_springs");
    CHECK(vf->option_id == "engine_valve_springs");
}

TEST_CASE("engine internals: downgrades are garage options, warn at the stock limiter and have a negative/positive mass node",
          "[engine_internals][part_mass][limit_advisor]") {
    Fixture f;
    const rg::CatalogEntry& gt = f.entry("car_gt70");
    const rg::SetupModel model = rg::build_setup_model(gt, f.table, f.ctx);
    REQUIRE(model.error.empty());
    bool saw = false;
    for (const rg::OptionView& v : model.options) {
        if (v.def.id != "engine_rods") continue;
        for (const rg::SetupPart& p : v.parts) if (p.id == "cast_steel") saw = true;
    }
    CHECK(saw);
    const rg::CompiledSetup stock = rg::compile_setup(gt, f.table, f.ctx, setup_of("car_gt70", {}));
    REQUIRE(stock.ok);
    // A light crank: its mass node is negative, the tree total follows; cast-steel rods are heavier (positive node).
    const rg::CompiledSetup lc = rg::compile_setup(gt, f.table, f.ctx, setup_of("car_gt70", {{"engine_crankshaft", std::string("lightweight_cast")}}));
    REQUIRE(lc.ok);
    const rg::PartNode* n = lc.parts.root.find("engine.internals.crankshaft");
    REQUIRE(n != nullptr);
    CHECK(n->own_kg < 0.0);
    CHECK(lc.parts.total_kg == Approx(stock.parts.total_kg + n->own_kg));
    const rg::CompiledSetup cr = rg::compile_setup(gt, f.table, f.ctx, setup_of("car_gt70", {{"engine_rods", std::string("cast_steel")}}));
    REQUIRE(cr.ok);
    CHECK(cr.parts.root.find("engine.internals.connecting_rods")->own_kg > 0.0);
    // The GT70's Ventura-class limiter against cast-steel rods: warn at the stock limiter when the piston speed there
    // exceeds the rod rating, naming the rods.
    // Cast-steel rods against the GT70's engine: the setup check warns once the piston speed passes the rod rating, naming the rods.
    const rg::CompiledSetup low = rg::compile_setup(gt, f.table, f.ctx, setup_of("car_gt70", {{"engine_rods", std::string("cast_steel")}}));
    const double rod_rpm = low.internals->parts.front().limit * 60.0 / (2.0 * low.internals->stroke_m);
    rg::AdviceInput in2;
    rg::set_part_limits(in2, low);
    in2.stroke_m = low.internals->stroke_m;
    in2.points.push_back(point_at(rod_rpm * 1.05));
    const rg::AdvisorConfig adv2 = advisor();
    const auto items = rg::build_advice(in2, adv2);
    const rg::AdviceItem* it = find_item(items, "MeanPistonSpeed");
    REQUIRE(it != nullptr);
    CHECK(it->damage_part == "connecting_rods");
}

TEST_CASE("upgrade lists: choices the car cannot fit are hidden unless show all is on", "[engine_internals][upgrade_fit]") {
    rg::OptionView v;
    rg::SetupPart stock, ok, bad;
    stock.id = "stock";
    ok.id = "single";
    bad.id = "quad";
    bad.fits = false;
    bad.fit_reason = "Quad-turbo does not fit Pico (556 mm long, the bay takes 500 mm)";
    v.parts = {stock, ok, bad};
    CHECK(rg::visible_parts(v, false).size() == 2);
    CHECK(rg::visible_parts(v, true).size() == 3);
    CHECK(rg::hidden_parts_reason(v).empty()); // one choice still fits
    v.parts = {stock, bad};
    CHECK(rg::visible_parts(v, false).size() == 1); // stock only
    CHECK_THAT(rg::hidden_parts_reason(v), ContainsSubstring("does not fit"));
    // The setting exists, hidden, default off.
    bool found = false;
    for (const rg::SettingDef& d : rg::settings_schema()) {
        if (d.key != rg::kShowAllUpgradesKey) continue;
        found = true;
        CHECK_FALSE(d.shown);
        CHECK(std::get<bool>(d.default_value) == false);
    }
    CHECK(found);
}

TEST_CASE("engine internals: the rod load follows the piston mass", "[engine_internals]") {
    Fixture f;
    const rg::EngineInternalsConfig& c = *f.ctx.engine_internals;
    const auto facts = rg::read_engine_facts(f.store->find("default:ventura_v8_5l0_na")->file, nullptr);
    REQUIRE(facts.has_value());
    std::string err;
    const auto rod_rpm = [&](const char* piston) {
        const auto b = rg::describe_engine_internals(c, *facts, {{"pistons", piston}}, {}, &err);
        REQUIRE(b.has_value());
        for (const rg::InternalPartBuild& p : b->parts) {
            if (p.limit_kind == rg::InternalLimitKind::RodLoad) return p.limit_rpm;
        }
        FAIL("no rod part");
        return 0.0;
    };
    // The same rod: a lighter forged piston raises the rpm where the rod reaches its rating, a heavier one lowers it.
    const double stock = rod_rpm("stock");
    CHECK(rod_rpm("forged") > stock);           // x0.92
    CHECK(rod_rpm("hypereutectic") > stock);    // x0.97
    CHECK(rod_rpm("low_silicon_cast") < stock); // x1.05
    // The formula: F = m_recip (S/2) w^2 (1 + lambda) equals the rating at that rpm.
    const auto b = rg::describe_engine_internals(c, *facts, {{"pistons", "forged"}}, {}, &err);
    REQUIRE(b.has_value());
    for (const rg::InternalPartBuild& p : b->parts) {
        if (p.limit_kind != rg::InternalLimitKind::RodLoad) continue;
        const double w = p.limit_rpm * 2.0 * 3.14159265358979 / 60.0;
        CHECK(p.recip_mass_kg * (facts->stroke_m / 2.0) * w * w * (1.0 + c.rod_crank_ratio) == Approx(p.limit).epsilon(1e-9));
    }
    // A lighter rod set lowers the load too (the small-end share), so titanium rods hold a higher rpm than stock rods.
    const auto ti = rg::describe_engine_internals(c, *facts, {{"connecting_rods", "titanium"}}, {}, &err);
    REQUIRE(ti.has_value());
    CHECK(ti->parts.front().limit_rpm > b->parts.front().limit_rpm * 0.0 + stock);
    // Stock rods on stock pistons keep the margin at the factory limiter.
    const auto st = rg::describe_engine_internals(c, *facts, {}, {}, &err);
    REQUIRE(st.has_value());
    CHECK(st->parts.front().limit_rpm >= c.stock_margin * facts->limiter_rpm - 1e-6);
}
