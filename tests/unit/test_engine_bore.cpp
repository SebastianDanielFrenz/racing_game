// test_engine_bore.cpp - the cylinder bore upgrade (rg/engine_internals.h BoreConfig; owner on PR #21: "I want upgrades
// that bore out the cylinders; this logically implies new pistons (due to diameter difference), among other things.").
// The step goes into the engine file's bore_mm through the merge patch, so displacement and the generated maps follow;
// the compression ratio is computed (head volume kept); the pistons are the new diameter's and heavier; a turbo kit sizes
// for the larger displacement; a thin wall warns in the setup check.
#include "rg/engine_internals.h"
#include "rg/engine_store.h"
#include "rg/garage.h"
#include "rg/limit_advisor.h"
#include "rg/part_mass.h"
#include "rg/turbo_kit.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
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
constexpr double kPi = 3.14159265358979323846;

rg::EngineInternalsConfig internals() {
    std::string err;
    auto c = rg::load_engine_internals(kRoot + "/data/engines/engine_internals.json", &err);
    INFO(err);
    REQUIRE(c.has_value());
    return *c;
}

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    std::shared_ptr<rg::EngineStore> store;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_engine_bore_tests"};
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
        auto kits = rg::load_turbo_kit_config(kRoot + "/data/engines/turbo_kits.json", kRoot, &err);
        INFO(err);
        REQUIRE(kits.has_value());
        ctx.turbo_kits = std::make_shared<const rg::TurboKitConfig>(*kits);
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
    rg::EngineFacts facts(const std::string& id) const {
        const rg::EngineEntry* e = store->find("default:" + id);
        REQUIRE(e != nullptr);
        auto f = rg::read_engine_facts(e->file, nullptr);
        REQUIRE(f.has_value());
        return *f;
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
const rg::SetupPart* part_of(const rg::OptionView& v, const std::string& id) {
    for (const rg::SetupPart& p : v.parts) if (p.id == id) return &p;
    return nullptr;
}

struct Loaded {
    bool ok = false;
    std::string error;
    ps::vehicle::VehicleDesc desc;
    rg::CompiledSetup compiled;
    nlohmann::json engine_file; // the materialised engine file
};
// generate = false: validate-only. Otherwise Sketch maps through the shared cache.
Loaded load_setup(const Fixture& f, const rg::CatalogEntry& e, const rg::VehicleSetup& s, ps::jobs::JobSystem* pool = nullptr) {
    Loaded out;
    rg::ScopedWorkDir dir(f.ctx.work_root, "b_" + e.id);
    const rg::MaterialisedSetup m = rg::materialise_setup(e, f.table, f.ctx, s, dir.path());
    out.compiled = m.compiled;
    if (!m.ok) {
        out.error = m.error;
        return out;
    }
    ps::io::VehicleLoadOptions options;
    if (pool == nullptr) {
        options.maps.generate = false;
    } else {
        options.maps.quality = ps::drivetrain::MapQuality::Sketch;
        options.maps.pool = pool;
        options.maps.cache_dir = f.ctx.engine_map_cache_dir;
    }
    try {
        out.desc = rg::load_vehicle(m.vehicle_path, options);
        out.ok = true;
    } catch (const std::exception& ex) {
        out.error = ex.what();
    }
    // The engine file the vehicle names (the materialised copy, as the loader read it).
    for (const std::string& file : m.files) {
        if (file.find("engine") == std::string::npos || file.find(".json") == std::string::npos) continue;
        std::ifstream in(file, std::ios::binary);
        const nlohmann::json j = nlohmann::json::parse(in, nullptr, false);
        if (j.is_object() && j.contains("bore_mm") && j.contains("cycle")) out.engine_file = j;
    }
    return out;
}

const ps::drivetrain::SimulatedEngineDesc* sim_engine(const ps::vehicle::VehicleDesc& d, const ps::drivetrain::ComponentParams** params = nullptr) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* p = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) {
            if (params != nullptr) *params = &c.params;
            return p;
        }
    }
    return nullptr;
}

double litres(const ps::drivetrain::SimulatedEngineDesc& e) {
    return e.piston.cylinders * kPi / 4.0 * e.piston.bore_m * e.piston.bore_m * e.piston.stroke_m * 1000.0;
}

} // namespace

TEST_CASE("bore: the shipped steps load, the loader is strict", "[engine_internals][bore]") {
    const rg::EngineInternalsConfig c = internals();
    REQUIRE(c.bore.has_value());
    REQUIRE(c.bore->steps.size() == 3);
    CHECK(c.bore->steps[0].oversize_mm == Approx(0.5));
    CHECK(c.bore->steps[1].oversize_mm == Approx(1.0));
    CHECK(c.bore->steps[2].id == "big_bore");
    for (const rg::BoreStep& s : c.bore->steps) CHECK_THAT(s.source, ContainsSubstring("[AS]"));
    CHECK(c.bore->min_wall_mm < c.bore->safe_wall_mm);

    std::ifstream in(kRoot + "/data/engines/engine_internals.json", std::ios::binary);
    const nlohmann::json doc = nlohmann::json::parse(in);
    std::string err;
    {
        nlohmann::json bad = doc;
        bad["bore"]["steps"][1]["oversize_mm"] = 0.2; // must grow
        CHECK_FALSE(rg::parse_engine_internals(bad.dump(), "t", &err).has_value());
        CHECK_THAT(err, ContainsSubstring("oversize_mm"));
    }
    {
        nlohmann::json bad = doc;
        bad["bore"]["compression_rule"] = "kept";
        CHECK_FALSE(rg::parse_engine_internals(bad.dump(), "t", &err).has_value());
        CHECK_THAT(err, ContainsSubstring("compression_rule"));
    }
    {
        nlohmann::json bad = doc;
        bad["bore"]["safe_wall_mm"] = 3.0; // below min_wall_mm
        CHECK_FALSE(rg::parse_engine_internals(bad.dump(), "t", &err).has_value());
    }
    {
        nlohmann::json none = doc;
        none.erase("bore");
        const auto ok = rg::parse_engine_internals(none.dump(), "t", &err);
        REQUIRE(ok.has_value());
        CHECK_FALSE(ok->bore.has_value());
    }
}

TEST_CASE("bore: geometry, compression ratio and the per-engine limit", "[engine_internals][bore]") {
    Fixture f;
    const rg::EngineInternalsConfig& c = *f.ctx.engine_internals;
    // Every simulated piston engine: displacement grows with bore^2, CR by the head-volume rule, wall = spacing - bore.
    for (const char* id : {"hyper_v8_one1_audio", "demo_i4_pipes", "toyota_a25a_fks_gen", "pico_i3_1l0_na", "ventura_v8_5l0_na", "runabout_i4_2l9_sv"}) {
        INFO(id);
        const rg::EngineFacts e = f.facts(id);
        REQUIRE(e.bore_m > 0.0);
        CHECK(e.factory_bore_m == e.bore_m);
        for (const rg::BoreStep& s : c.bore->steps) {
            INFO(s.id);
            const auto g = rg::bore_geometry(c, e, s.id, nullptr);
            REQUIRE(g.has_value());
            const double r = (e.bore_m + s.oversize_mm / 1000.0) / e.bore_m;
            CHECK(g->bore_m == Approx(e.bore_m + s.oversize_mm / 1000.0));
            CHECK(g->displacement_l == Approx(e.displacement_l * r * r));
            CHECK(g->compression_ratio == Approx(1.0 + (e.compression_ratio - 1.0) * r * r).margin(0.001));
            CHECK(g->compression_ratio > e.compression_ratio);
            CHECK(g->wall_mm == Approx(c.bore->pitch_factor * e.bore_m * 1000.0 - g->bore_m * 1000.0));
            CHECK(g->block_mass_delta_kg < 0.0);
            CHECK(g->fits == (g->wall_mm >= c.bore->min_wall_mm - 1e-6));
            CHECK(g->fits == rg::bore_refusal(c, e, s.id, "X").empty());
        }
        // The two overbores fit every engine.
        CHECK(rg::bore_refusal(c, e, "overbore_0_5", "").empty());
        CHECK(rg::bore_refusal(c, e, "overbore_1_0", "").empty());
    }
    // The 71 mm three has the narrowest spacing (77.4 mm at pitch 1.09): the 3 mm big bore would leave 3.4 mm.
    {
        const rg::EngineFacts pico = f.facts("pico_i3_1l0_na");
        const auto g = rg::bore_geometry(c, pico, "big_bore", nullptr);
        REQUIRE(g.has_value());
        CHECK_FALSE(g->fits);
        CHECK(g->max_oversize_mm == Approx(2.39).margin(0.01));
        const std::string why = rg::bore_refusal(c, pico, "big_bore", "Pico");
        CHECK_THAT(why, ContainsSubstring("wall"));
        CHECK_THAT(why, ContainsSubstring("+2.4 mm"));
    }
    // The A25A's big bore fits (4.9 mm wall) but warns (below 5 mm); the Ventura's does not warn.
    {
        const auto g = rg::bore_geometry(c, f.facts("toyota_a25a_fks_gen"), "big_bore", nullptr);
        REQUIRE(g.has_value());
        CHECK(g->fits);
        CHECK(g->warns);
        const auto v = rg::bore_geometry(c, f.facts("ventura_v8_5l0_na"), "big_bore", nullptr);
        REQUIRE(v.has_value());
        CHECK(v->fits);
        CHECK_FALSE(v->warns);
    }
    // bored_engine_facts keeps the factory figures.
    const rg::EngineFacts v8 = f.facts("ventura_v8_5l0_na");
    const auto g = rg::bore_geometry(c, v8, "big_bore", nullptr);
    const rg::EngineFacts bored = rg::bored_engine_facts(v8, *g);
    CHECK(bored.factory_bore_m == v8.bore_m);
    CHECK(bored.factory_displacement_l == v8.displacement_l);
    CHECK(bored.displacement_l == Approx(g->displacement_l));
    // Same answer from bored facts (the factory figures are the baseline).
    const auto again = rg::bore_geometry(c, bored, "big_bore", nullptr);
    REQUIRE(again.has_value());
    CHECK(again->bore_m == Approx(g->bore_m));
}

TEST_CASE("bore: torque-curve and rotary engines are refused with the reason", "[engine_internals][bore]") {
    Fixture f;
    const rg::EngineInternalsConfig& c = *f.ctx.engine_internals;
    CHECK_THAT(rg::bore_refusal(c, f.facts("n52b30_3l_na"), "overbore_0_5", "N52"), ContainsSubstring("torque-curve"));
    CHECK_THAT(rg::bore_refusal(c, f.facts("r26b_4rotor"), "overbore_0_5", "R26B"), ContainsSubstring("R26B")); // no cycle pistons
    // The garage: the torque-curve sedan shows the option unavailable with the reason; a saved choice is refused.
    const rg::CatalogEntry& sedan = f.entry("car_sedan");
    const rg::SetupModel m = rg::build_setup_model(sedan, f.table, f.ctx);
    REQUIRE(m.error.empty());
    const rg::OptionView* v = view_of(m, "engine_bore");
    REQUIRE(v != nullptr);
    CHECK_FALSE(v->available);
    CHECK_THAT(v->unavailable_reason, ContainsSubstring("torque-curve"));
    const rg::CompiledSetup bad = rg::compile_setup(sedan, f.table, f.ctx, setup_of("car_sedan", {{"engine_bore", std::string("overbore_0_5")}}));
    CHECK_FALSE(bad.ok);
    CHECK_THAT(bad.error, ContainsSubstring("torque-curve"));
}

TEST_CASE("bore: the garage lists the steps per engine, an unfit one hidden with its reason", "[engine_internals][bore][garage]") {
    Fixture f;
    // Pico (car_city): both overbores, the big bore listed with fits = false and the wall reason.
    {
        const rg::SetupModel m = rg::build_setup_model(f.entry("car_city"), f.table, f.ctx);
        REQUIRE(m.error.empty());
        const rg::OptionView* v = view_of(m, "engine_bore");
        REQUIRE(v != nullptr);
        CHECK(v->available);
        REQUIRE(v->parts.size() == 4);
        CHECK(v->parts.front().id == "stock");
        CHECK_THAT(v->parts.front().detail, ContainsSubstring("71.0 mm"));
        CHECK(part_of(*v, "overbore_0_5")->fits);
        CHECK(part_of(*v, "overbore_1_0")->fits);
        const rg::SetupPart* big = part_of(*v, "big_bore");
        REQUIRE(big != nullptr);
        CHECK_FALSE(big->fits);
        CHECK_THAT(big->fit_reason, ContainsSubstring("wall"));
        CHECK(rg::hidden_parts_reason(*v).empty()); // two steps still show: only the big bore is hidden
        CHECK(rg::visible_parts(*v, false).size() == 3); // stock + two overbores; the big bore hidden by default
        CHECK(rg::visible_parts(*v, true).size() == 4);
        const rg::CompiledSetup c = rg::compile_setup(f.entry("car_city"), f.table, f.ctx, setup_of("car_city", {{"engine_bore", std::string("big_bore")}}));
        CHECK_FALSE(c.ok);
        CHECK_THAT(c.error, ContainsSubstring("wall"));
    }
    // Ventura V8 (car_gt70): every step fits.
    {
        const rg::SetupModel m = rg::build_setup_model(f.entry("car_gt70"), f.table, f.ctx);
        REQUIRE(m.error.empty());
        const rg::OptionView* v = view_of(m, "engine_bore");
        REQUIRE(v != nullptr);
        for (const rg::SetupPart& p : v->parts) {
            INFO(p.id);
            CHECK(p.fits);
            CHECK_FALSE(p.detail.empty());
        }
        CHECK_THAT(part_of(*v, "big_bore")->detail, ContainsSubstring("104.6 mm"));
    }
}

TEST_CASE("bore: an engine swap to a block that cannot take the step drops it", "[engine_internals][bore][garage]") {
    rg::ScopedWorkDir user(fs::temp_directory_path().generic_string(), "rg_engine_bore_user");
    rg::ScopedWorkDir work(fs::temp_directory_path().generic_string(), "rg_engine_bore_work");
    rg::GarageConfig cfg;
    cfg.repo_root = kRoot;
    cfg.catalog_path = kRoot + "/data/vehicles/catalog.json";
    cfg.options_path = kRoot + "/data/vehicles/setup_options.json";
    cfg.set_path = kRoot + "/data/garage/garage_set.json";
    cfg.tyre_families_path = kRoot + "/data/tyres/families.json";
    cfg.user_dir = user.path();
    cfg.work_root = work.path();
    cfg.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
    std::string err;
    auto g = rg::Garage::open(cfg, &err);
    INFO(err);
    REQUIRE(g != nullptr);
    REQUIRE(g->begin_edit("car_hyper", &err));
    REQUIRE(g->set_option("engine_install", std::string("default:ventura_v8_5l0_na")).accepted);
    REQUIRE(g->set_option("engine_bore", std::string("big_bore")).accepted);
    INFO(g->validation().message);
    CHECK(g->validation().ok);
    // The 71 mm three cannot take +3 mm: back to the stock bore, with a notice.
    const auto r = g->set_option("engine_install", std::string("default:pico_i3_1l0_na"));
    INFO(r.notice << " / " << g->validation().message);
    REQUIRE(r.accepted);
    CHECK(g->validation().ok);
    CHECK_THAT(r.notice, ContainsSubstring("bore"));
    CHECK(std::get<std::string>(g->current_value("engine_bore")) == "stock");
}

TEST_CASE("bore: the materialised engine has the new bore, displacement and compression ratio", "[engine_internals][bore]") {
    Fixture f;
    const rg::CatalogEntry& gt = f.entry("car_gt70");
    const Loaded stock = load_setup(f, gt, setup_of("car_gt70", {}));
    INFO(stock.error);
    REQUIRE(stock.ok);
    const Loaded big = load_setup(f, gt, setup_of("car_gt70", {{"engine_bore", std::string("big_bore")}}));
    INFO(big.error);
    REQUIRE(big.ok);
    const auto* s0 = sim_engine(stock.desc);
    const auto* s1 = sim_engine(big.desc);
    REQUIRE(s0 != nullptr);
    REQUIRE(s1 != nullptr);
    CHECK(s0->piston.bore_m == Approx(0.1016));
    CHECK(s1->piston.bore_m == Approx(0.1046));
    CHECK(s1->piston.stroke_m == Approx(s0->piston.stroke_m));
    CHECK(litres(*s1) > litres(*s0));
    CHECK(litres(*s1) / litres(*s0) == Approx((104.6 / 101.6) * (104.6 / 101.6)));
    // The patch carries bore_mm and the computed compression ratio.
    auto patch = big.compiled.patches.begin();
    while (patch != big.compiled.patches.end() && patch->first.rfind("engine:", 0) != 0) ++patch;
    REQUIRE(patch != big.compiled.patches.end());
    CHECK_THAT(patch->second, ContainsSubstring("bore_mm"));
    CHECK_THAT(patch->second, ContainsSubstring("compression_ratio"));
    REQUIRE(big.engine_file.is_object());
    CHECK(big.engine_file["bore_mm"].get<double>() == Approx(104.6));
    const double cr = 1.0 + (9.0 - 1.0) * (104.6 / 101.6) * (104.6 / 101.6);
    CHECK(big.engine_file["cycle"]["compression_ratio"].get<double>() == Approx(cr).margin(0.001));
    // Internals: bored, pistons of the new size, block material removed.
    REQUIRE(big.compiled.internals.has_value());
    REQUIRE(big.compiled.internals->bore.has_value());
    CHECK(big.compiled.internals->bore_option_id == "engine_bore");
}

TEST_CASE("bore: heavier pistons of the new size, a lower rod-limited rpm, block mass removed", "[engine_internals][bore][part_mass]") {
    Fixture f;
    const rg::CatalogEntry& gt = f.entry("car_gt70");
    const rg::CompiledSetup stock = rg::compile_setup(gt, f.table, f.ctx, setup_of("car_gt70", {}));
    REQUIRE(stock.ok);
    REQUIRE(stock.parts.ok);
    const rg::CompiledSetup big = rg::compile_setup(gt, f.table, f.ctx, setup_of("car_gt70", {{"engine_bore", std::string("big_bore")}}));
    INFO(big.error);
    REQUIRE(big.ok);
    REQUIRE(big.parts.ok);
    REQUIRE(stock.internals.has_value());
    REQUIRE(big.internals.has_value());
    const auto part = [](const rg::EngineInternalsBuild& b, const std::string& kind) -> const rg::InternalPartBuild& {
        for (const rg::InternalPartBuild& p : b.parts) if (p.kind == kind) return p;
        FAIL("no " << kind);
        return b.parts.front();
    };
    const double r = 104.6 / 101.6;
    const rg::InternalPartBuild& p0 = part(*stock.internals, "pistons");
    const rg::InternalPartBuild& p1 = part(*big.internals, "pistons");
    CHECK(p1.mass_kg == Approx(p0.mass_kg * std::pow(r, 3.0)));
    CHECK(p1.mass_delta_kg > 0.0);
    CHECK_THAT(p1.grade_label, ContainsSubstring("104.6 mm"));
    const rg::InternalPartBuild& r0 = part(*stock.internals, "connecting_rods");
    const rg::InternalPartBuild& r1 = part(*big.internals, "connecting_rods");
    CHECK(r1.limit == Approx(r0.limit));        // the same rod, the same rating
    CHECK(r1.recip_mass_kg > r0.recip_mass_kg); // the heavier piston loads it more
    CHECK(r1.limit_rpm < r0.limit_rpm);
    CHECK(r1.mass_kg == Approx(r0.mass_kg));    // rods, crank and springs keep their mass
    CHECK(part(*big.internals, "crankshaft").mass_kg == Approx(part(*stock.internals, "crankshaft").mass_kg));
    WARN("Ventura big bore: pistons " << p0.mass_kg << " -> " << p1.mass_kg << " kg, rod-limited " << r0.limit_rpm << " -> "
                                      << r1.limit_rpm << " rpm, block " << big.internals->bore->block_mass_delta_kg << " kg");
    // Part-mass tree: a piston node (even at the stock grade) and the bored block.
    const rg::PartNode* pn = big.parts.root.find("engine.internals.pistons");
    REQUIRE(pn != nullptr);
    CHECK(pn->own_kg == Approx(p1.mass_delta_kg));
    const rg::PartNode* block = big.parts.root.find("engine.internals.cylinder_block");
    REQUIRE(block != nullptr);
    CHECK(block->own_kg < 0.0);
    CHECK(block->own_kg == Approx(big.internals->bore->block_mass_delta_kg));
    CHECK(big.parts.total_kg == Approx(stock.parts.total_kg + pn->own_kg + block->own_kg));

    // Every piston grade is offered at the chosen bore: the cards name the size and weigh more than at the stock bore.
    const rg::VehicleSetup bored = setup_of("car_gt70", {{"engine_bore", std::string("big_bore")}});
    const rg::SetupModel m0 = rg::build_setup_model(gt, f.table, f.ctx);
    const rg::SetupModel m1 = rg::build_setup_model(gt, f.table, f.ctx, &bored);
    REQUIRE(m1.error.empty());
    const rg::OptionView* v0 = view_of(m0, "engine_pistons");
    const rg::OptionView* v1 = view_of(m1, "engine_pistons");
    REQUIRE(v0 != nullptr);
    REQUIRE(v1 != nullptr);
    REQUIRE(v1->parts.size() == v0->parts.size());
    for (std::size_t i = 0; i < v1->parts.size(); ++i) {
        INFO(v1->parts[i].id);
        CHECK_THAT(v1->parts[i].detail, ContainsSubstring("104.6 mm bore"));
        CHECK_FALSE(v0->parts[i].detail.find("mm bore") != std::string::npos);
    }
    // A forged piston at the big bore is still a piston of the big bore (mass x r^3 x grade factor).
    const rg::CompiledSetup forged = rg::compile_setup(gt, f.table, f.ctx, setup_of("car_gt70", {{"engine_bore", std::string("big_bore")}, {"engine_pistons", std::string("forged")}}));
    REQUIRE(forged.ok);
    CHECK(part(*forged.internals, "pistons").mass_kg == Approx(p0.mass_kg * std::pow(r, 3.0) * 0.92));
}

TEST_CASE("bore: a thin wall warns in the setup check, naming the cylinder block", "[engine_internals][bore][limit_advisor]") {
    Fixture f;
    std::string err;
    auto adv = rg::load_advisor_config(kRoot + "/data/garage/advisor.json", &err);
    INFO(err);
    REQUIRE(adv.has_value());
    // A25A in the hypercar: the big bore leaves 4.9 mm (fits, below the 5 mm safe figure).
    const rg::CatalogEntry& hyper = f.entry("car_hyper");
    const rg::CompiledSetup c = rg::compile_setup(hyper, f.table, f.ctx,
                                                  setup_of("car_hyper", {{"engine_install", std::string("default:toyota_a25a_fks_gen")}, {"engine_bore", std::string("big_bore")}}));
    INFO(c.error);
    REQUIRE(c.ok);
    rg::AdviceInput in;
    rg::set_part_limits(in, c);
    CHECK(in.cylinder_wall_mm == Approx(4.875).margin(0.01));
    const auto items = rg::build_advice(in, *adv);
    const rg::AdviceItem* wall = nullptr;
    for (const rg::AdviceItem& i : items) if (i.rule_name == "CylinderWall") wall = &i;
    REQUIRE(wall != nullptr);
    CHECK(wall->damage_part == "cylinder_block");
    CHECK(wall->option_id == "engine_bore");
    CHECK(wall->severity == rg::AdviceSeverity::Warning);
    CHECK_THAT(wall->lines.front(), ContainsSubstring("4.9 mm"));
    // The +1.0 mm overbore leaves plenty: no item.
    const rg::CompiledSetup ok = rg::compile_setup(hyper, f.table, f.ctx,
                                                   setup_of("car_hyper", {{"engine_install", std::string("default:toyota_a25a_fks_gen")}, {"engine_bore", std::string("overbore_1_0")}}));
    REQUIRE(ok.ok);
    rg::AdviceInput in2;
    rg::set_part_limits(in2, ok);
    for (const rg::AdviceItem& i : rg::build_advice(in2, *adv)) CHECK(i.rule_name != "CylinderWall");
}

TEST_CASE("bore: a turbo kit sizes for the bored displacement", "[engine_internals][bore][turbo_kit]") {
    Fixture f;
    const rg::CatalogEntry& gt = f.entry("car_gt70");
    const rg::CompiledSetup k0 = rg::compile_setup(gt, f.table, f.ctx, setup_of("car_gt70", {{"turbo_kit", std::string("street_single")}}));
    INFO(k0.error);
    REQUIRE(k0.ok);
    const rg::CompiledSetup k1 = rg::compile_setup(gt, f.table, f.ctx,
                                                   setup_of("car_gt70", {{"engine_bore", std::string("big_bore")}, {"turbo_kit", std::string("street_single")}}));
    INFO(k1.error);
    REQUIRE(k1.ok);
    REQUIRE(k0.turbo_kit.has_value());
    REQUIRE(k1.turbo_kit.has_value());
    const double r2 = (104.6 / 101.6) * (104.6 / 101.6);
    CHECK(k1.turbo_kit->sizing.demand_kg_s > k0.turbo_kit->sizing.demand_kg_s);
    CHECK(k1.turbo_kit->sizing.demand_kg_s == Approx(k0.turbo_kit->sizing.demand_kg_s * r2).epsilon(0.02));
    CHECK(k1.turbo_kit->sizing.scale > k0.turbo_kit->sizing.scale);
    WARN("Ventura + street kit: demand " << k0.turbo_kit->sizing.demand_kg_s << " -> " << k1.turbo_kit->sizing.demand_kg_s << " kg/s, wheel x"
                                         << k0.turbo_kit->sizing.scale << " -> x" << k1.turbo_kit->sizing.scale);
    // The kit still lowers the (now higher) compression ratio to its ceiling.
    const Loaded l = load_setup(f, gt, setup_of("car_gt70", {{"engine_bore", std::string("big_bore")}, {"turbo_kit", std::string("street_single")}}));
    INFO(l.error);
    REQUIRE(l.ok);
    REQUIRE(l.engine_file.is_object());
    CHECK(l.engine_file["bore_mm"].get<double>() == Approx(104.6));
    CHECK(l.engine_file["cycle"]["compression_ratio"].get<double>() <= k1.turbo_kit->sizing.compression_ratio + 1e-9);
}

TEST_CASE("bore: more torque and power at the same rpm", "[engine_internals][bore][dyno][long]") {
    Fixture f;
    ps::jobs::JobSystem pool(2);
    struct Case { const char* car; const char* engine; const char* step; };
    for (const Case cs : {Case{"car_gt70", "", "big_bore"}, Case{"car_hyper", "default:toyota_a25a_fks_gen", "overbore_1_0"}}) {
        INFO(cs.car << " " << cs.engine << " " << cs.step);
        std::vector<std::pair<std::string, rg::SetupValue>> base;
        if (*cs.engine != '\0') base.push_back({"engine_install", std::string(cs.engine)});
        auto bored = base;
        bored.push_back({"engine_bore", std::string(cs.step)});
        const Loaded a = load_setup(f, f.entry(cs.car), setup_of(cs.car, base), &pool);
        INFO(a.error);
        REQUIRE(a.ok);
        const Loaded b = load_setup(f, f.entry(cs.car), setup_of(cs.car, bored), &pool);
        INFO(b.error);
        REQUIRE(b.ok);
        const ps::drivetrain::ComponentParams* pa = nullptr;
        const ps::drivetrain::ComponentParams* pb = nullptr;
        const auto* sa = sim_engine(a.desc, &pa);
        const auto* sb = sim_engine(b.desc, &pb);
        REQUIRE(sa != nullptr);
        REQUIRE(sb != nullptr);
        CHECK(sb->piston.bore_m > sa->piston.bore_m);
        ps::drivetrain::DynoSweepSpec spec;
        const double lim = sa->limiter.rpm;
        for (double frac : {0.35, 0.55, 0.75, 0.9}) spec.rpm.push_back(std::round(frac * lim / 50.0) * 50.0);
        spec.max_settle_s = 8.0;
        const auto ra = ps::drivetrain::run_dyno_sweep(*pa, nullptr, spec, &pool);
        const auto rb = ps::drivetrain::run_dyno_sweep(*pb, nullptr, spec, &pool);
        REQUIRE(ra.points.size() == spec.rpm.size());
        REQUIRE(rb.points.size() == spec.rpm.size());
        for (std::size_t i = 0; i < spec.rpm.size(); ++i) {
            WARN(cs.car << " " << cs.step << " @ " << spec.rpm[i] << " rpm: stock " << ra.points[i].torque_nm << " N*m / " << ra.points[i].power_kw
                        << " kW, bored " << rb.points[i].torque_nm << " N*m / " << rb.points[i].power_kw << " kW (x"
                        << rb.points[i].power_kw / ra.points[i].power_kw << ")");
            CHECK(rb.points[i].torque_nm > ra.points[i].torque_nm);
            CHECK(rb.points[i].power_kw > ra.points[i].power_kw);
        }
    }
}
