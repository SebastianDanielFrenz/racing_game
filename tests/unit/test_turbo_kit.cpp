// test_turbo_kit.cpp - garage turbo kits (rg/turbo_kit.h; owner on PR #5: "Turbos cannot be fitted yet to many engines").
// One rule sizes a turbo system for any simulated engine without a turbo; the setup option "turbo_kit" fits it and
// converts the engine (boost grid, compression ratio). Torque-curve engines are refused with a reason.
#include "rg/engine_store.h"
#include "rg/engine_swap.h"
#include "rg/garage.h"
#include "rg/turbo_kit.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include "rg/part_mass.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
const std::string kRoot = RG_SOURCE_DIR;

rg::TurboKitConfig kits() {
    std::string err;
    auto c = rg::load_turbo_kit_config(kRoot + "/data/engines/turbo_kits.json", kRoot, &err);
    INFO(err);
    REQUIRE(c.has_value());
    return *c;
}

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    std::shared_ptr<rg::EngineStore> store;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_turbo_kit_tests"};
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
        ctx.turbo_kits = std::make_shared<const rg::TurboKitConfig>(kits());
    }
    const rg::CatalogEntry& entry(const std::string& id) const {
        const rg::CatalogEntry* e = catalog.find(id);
        REQUIRE(e != nullptr);
        return *e;
    }
    // The default engines a kit must fit: simulated, a cycle block, no turbo of their own.
    std::vector<const rg::EngineEntry*> kit_engines() const {
        std::vector<const rg::EngineEntry*> out;
        for (const rg::EngineEntry& e : store->list()) {
            if (e.user) continue;
            const auto facts = rg::read_engine_facts(e.file, nullptr);
            REQUIRE(facts.has_value());
            if (facts->simulated && facts->has_cycle && !facts->has_turbo) out.push_back(&e);
        }
        return out;
    }
};

rg::VehicleSetup setup_of(const std::string& vehicle, std::vector<std::pair<std::string, rg::SetupValue>> values) {
    rg::VehicleSetup s;
    s.vehicle_id = vehicle;
    for (auto& kv : values) s.values[kv.first] = kv.second;
    return s;
}

struct Loaded {
    bool ok = false;
    std::string error;
    ps::vehicle::VehicleDesc desc;
    rg::CompiledSetup compiled;
};
// generate = false: validate-only (milliseconds). Otherwise Sketch maps through the shared cache.
Loaded load_setup(const Fixture& f, const rg::CatalogEntry& e, const rg::VehicleSetup& s, ps::jobs::JobSystem* pool = nullptr) {
    Loaded out;
    rg::ScopedWorkDir dir(f.ctx.work_root, "t_" + e.id);
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
double clutch_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* t = std::get_if<ps::drivetrain::ClutchDesc>(&c.params)) return t->capacity_nm;
    }
    return 0.0;
}
const rg::OptionView* view_of(const rg::SetupModel& m, const std::string& id) {
    for (const rg::OptionView& v : m.options) if (v.def.id == id) return &v;
    return nullptr;
}

} // namespace

TEST_CASE("turbo kits: the shipped config loads and the loader is strict", "[turbo_kit]") {
    const rg::TurboKitConfig c = kits();
    REQUIRE(c.kits.size() >= 2);
    CHECK(c.find("street_single") != nullptr);
    CHECK(c.max_compression_ratio <= 10.5); // physics check_buildable's forced-induction ceiling
    const fs::path dir = fs::temp_directory_path() / "rg_turbo_kit_cfg";
    fs::create_directories(dir);
    const auto write = [&](const std::string& text) {
        const std::string p = (dir / "kits.json").generic_string();
        FILE* fp = std::fopen(p.c_str(), "wb");
        REQUIRE(fp != nullptr);
        std::fputs(text.c_str(), fp);
        std::fclose(fp);
        return p;
    };
    std::string err;
    CHECK_FALSE(rg::load_turbo_kit_config(write("{\"format\": \"rg.turbo_kits/2\"}"), kRoot, &err).has_value());
    CHECK_THAT(err, ContainsSubstring("rg.turbo_kits/1"));
    CHECK_FALSE(rg::load_turbo_kit_config(write("{\"format\": \"rg.turbo_kits/1\", \"anchor_hardware\": \"nope.json\"}"), kRoot, &err).has_value());
    CHECK_THAT(err, ContainsSubstring("anchor_hardware"));
    fs::remove_all(dir);
}

TEST_CASE("turbo kits: which catalog engines can take one, and why the others cannot", "[turbo_kit]") {
    Fixture f;
    int fit = 0;
    for (const rg::EngineEntry& e : f.store->list()) {
        if (e.user) continue;
        INFO(e.key);
        const auto facts = rg::read_engine_facts(e.file, nullptr);
        REQUIRE(facts.has_value());
        const std::string why = rg::turbo_kit_refusal(*facts, e.name);
        if (e.cls == rg::EngineClass::Curve) {
            CHECK_THAT(why, ContainsSubstring("torque-curve engine"));
        } else if (facts->has_turbo) {
            CHECK_THAT(why, ContainsSubstring("turbocharged already"));
        } else {
            CHECK(why.empty());
            ++fit;
        }
    }
    // pico, ventura, runabout, the demo pipe I4 and the A25A
    CHECK(fit >= 5);
}

TEST_CASE("turbo kits: the sizing rule scales the anchor with the engine's air demand", "[turbo_kit]") {
    Fixture f;
    const rg::TurboKitConfig& c = *f.ctx.turbo_kits;
    const rg::TurboKitSpec& street = *c.find("street_single");
    const rg::TurboKitSpec& sport = *c.find("sport_single");
    double last_flow = 0.0;
    // Larger engines need larger turbos: order the kit engines by displacement x limiter.
    for (const rg::EngineEntry* e : f.kit_engines()) {
        INFO(e->key);
        const auto facts = rg::read_engine_facts(e->file, nullptr);
        REQUIRE(facts.has_value());
        const rg::TurboKitSizing a = rg::size_turbo_kit(c, street, *facts, facts->limiter_rpm);
        const rg::TurboKitSizing b = rg::size_turbo_kit(c, sport, *facts, facts->limiter_rpm);
        REQUIRE(a.ok);
        REQUIRE(b.ok);
        CHECK(a.scale > c.min_scale);
        CHECK(a.scale < c.max_scale);
        CHECK(b.demand_kg_s > a.demand_kg_s); // more boost, denser charge
        CHECK(b.max_mass_flow_kg_s > a.max_mass_flow_kg_s);
        CHECK(a.max_mass_flow_kg_s >= a.demand_kg_s); // the compressor flows what the engine needs at the limiter
        CHECK(a.reference_rpm > 0.0);
        CHECK(a.compression_ratio <= c.max_compression_ratio + 1e-9);
        CHECK(a.compression_ratio <= facts->compression_ratio + 1e-9);
        CHECK(a.torque_factor > 1.4);
        CHECK(a.torque_factor < 1.6);
        WARN(e->key << ": " << facts->displacement_l << " L, limiter " << facts->limiter_rpm << " rpm -> street "
                    << a.flow_per_turbo_kg_s << " kg/s, s = " << a.scale << ", " << a.exducer_mm << " mm, "
                    << a.reference_rpm << " rpm; CR " << facts->compression_ratio << " -> " << a.compression_ratio);
        last_flow = a.max_mass_flow_kg_s;
    }
    CHECK(last_flow > 0.0);
    // The twin needs six cylinders; a 3-cylinder is refused with the reason.
    rg::EngineFacts three;
    three.cylinders = 3;
    CHECK_THAT(rg::turbo_kit_spec_refusal(*c.find("sport_twin"), three), ContainsSubstring("6 cylinders"));
}

TEST_CASE("turbo kits: every kit engine loads validate-only with a kit fitted (own car and via an engine swap)", "[turbo_kit]") {
    Fixture f;
    // On the cars that ship with these engines.
    for (const auto& [car, kit] : std::vector<std::pair<std::string, std::string>>{
             {"car_city", "street_single"}, {"car_gt70", "sport_twin"}, {"car_runabout", "street_single"}}) {
        INFO(car << " + " << kit);
        const Loaded l = load_setup(f, f.entry(car), setup_of(car, {{"turbo_kit", kit}}));
        INFO(l.error);
        REQUIRE(l.ok);
        const auto* sim = sim_engine(l.desc);
        REQUIRE(sim != nullptr);
        CHECK(sim->turbo_pair.has_value());
        REQUIRE(sim->cycle.has_value());
        CHECK(sim->cycle->compression_ratio <= f.ctx.turbo_kits->max_compression_ratio + 1e-9);
        bool kit_line = false;
        for (const auto& line : l.compiled.swap_lines) kit_line = kit_line || line.rfind("Turbo kit:", 0) == 0;
        CHECK(kit_line);
    }
    // Every kit engine swapped into the hypercar, with the street kit.
    for (const rg::EngineEntry* e : f.kit_engines()) {
        INFO(e->key);
        const Loaded l = load_setup(f, f.entry("car_hyper"), setup_of("car_hyper", {{"engine_install", e->key}, {"turbo_kit", std::string("street_single")}}));
        INFO(l.error);
        REQUIRE(l.ok);
        const auto* sim = sim_engine(l.desc);
        REQUIRE(sim != nullptr);
        CHECK(sim->turbo_pair.has_value());
    }
}

TEST_CASE("turbo kits: the garage shows why a kit cannot be fitted", "[turbo_kit]") {
    Fixture f;
    {
        const rg::SetupModel m = rg::build_setup_model(f.entry("car_runabout"), f.table, f.ctx);
        INFO(m.error);
        const rg::OptionView* v = view_of(m, "turbo_kit");
        REQUIRE(v != nullptr);
        CHECK(v->available);
        CHECK(v->unavailable_reason.empty());
        CHECK(v->choices.size() >= 2); // stock + the single kits (a 4-cylinder gets no twin)
        for (const auto& c : v->choices) CHECK(c != "sport_twin");
    }
    {
        // A torque-curve engine: unavailable, with the reason.
        const rg::SetupModel m = rg::build_setup_model(f.entry("car_sedan"), f.table, f.ctx);
        const rg::OptionView* v = view_of(m, "turbo_kit");
        REQUIRE(v != nullptr);
        CHECK_FALSE(v->available);
        CHECK_THAT(v->unavailable_reason, ContainsSubstring("torque-curve engine"));
        const rg::CompiledSetup c = rg::compile_setup(f.entry("car_sedan"), f.table, f.ctx, setup_of("car_sedan", {{"turbo_kit", std::string("street_single")}}));
        CHECK_FALSE(c.ok);
        CHECK_THAT(c.error, ContainsSubstring("torque-curve engine"));
    }
    {
        const rg::SetupModel m = rg::build_setup_model(f.entry("car_hyper"), f.table, f.ctx);
        const rg::OptionView* v = view_of(m, "turbo_kit");
        REQUIRE(v != nullptr);
        CHECK_FALSE(v->available);
        CHECK_THAT(v->unavailable_reason, ContainsSubstring("turbocharged already"));
    }
}

TEST_CASE("turbo kits: the swap package sizes the clutch for the boosted torque", "[turbo_kit][engine_swap]") {
    rg::SwapCar car;
    car.clutch_kinetic_nm = 200.0;
    rg::SwapEngine eng;
    eng.name = "Test I4";
    eng.peak_torque_nm = 150.0;
    eng.idle_rpm = 800.0;
    eng.limiter_rpm = 6500.0;
    eng.simulated = true;
    rg::SwapChoices ch;
    ch.turbo_kit = "Sport turbo kit";
    ch.boost_torque_factor = 1.99;
    const rg::SwapPackage p = rg::derive_swap_package(car, eng, ch, false);
    REQUIRE(p.ok);
    CHECK(p.set_clutch);
    CHECK(p.clutch_kinetic_nm == std::ceil(1.5 * 150.0 * 1.99 / 10.0) * 10.0);
    CHECK_FALSE(p.set_tcu); // no engine swap: the shift points stay
    // Without a kit and without a swap nothing changes.
    const rg::SwapPackage none = rg::derive_swap_package(car, eng, rg::SwapChoices{}, false);
    CHECK_FALSE(none.set_clutch);
    CHECK(none.lines.empty());

    // Through the real files: the city car's clutch after the sport kit.
    Fixture f;
    const Loaded l = load_setup(f, f.entry("car_city"), setup_of("car_city", {{"turbo_kit", std::string("sport_single")}}));
    INFO(l.error);
    REQUIRE(l.ok);
    const rg::EngineEntry* pico = f.store->find("default:pico_i3_1l0_na");
    REQUIRE(pico != nullptr);
    const double peak = f.store->peaks(*pico).torque_nm;
    REQUIRE(peak > 0.0);
    CHECK(clutch_of(l.desc) >= 1.5 * peak * 1.9 - 10.0);
}

TEST_CASE("turbo kits: the garage drops a kit when an engine that cannot take one goes in", "[turbo_kit][garage]") {
    rg::ScopedWorkDir user(fs::temp_directory_path().generic_string(), "rg_turbo_kit_user");
    rg::ScopedWorkDir work(fs::temp_directory_path().generic_string(), "rg_turbo_kit_work");
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
    REQUIRE(g->set_option("engine_install", std::string("default:pico_i3_1l0_na")).accepted);
    REQUIRE(g->set_option("turbo_kit", std::string("street_single")).accepted);
    CHECK(g->validation().ok);
    // A torque-curve engine (the 2.0 L I4) cannot take a kit.
    const auto r = g->set_option("engine_install", std::string("default:i4_2l_na"));
    INFO(r.notice << " / " << g->validation().message);
    REQUIRE(r.accepted);
    CHECK(g->validation().ok);
    CHECK(std::get<std::string>(g->current_value("turbo_kit")) == "stock");
}

// Each kit engine makes more power with the street kit than without it, at the same rpm (engine dyno, Sketch maps).
TEST_CASE("turbo kits: a kit's turbos, exhaust and intercooler join the car's part-mass tree", "[turbo_kit][part_mass]") {
    Fixture f;
    std::string err;
    auto table = rg::load_part_mass_table(kRoot + "/data/vehicles/part_masses.json", &err);
    INFO(err);
    REQUIRE(table.has_value());
    f.ctx.part_masses = std::make_shared<const rg::PartMassTable>(*table);
    const rg::TurboKitConfig& cfg = *f.ctx.turbo_kits;

    // The parts of one kit: turbo mass follows the sized wheel, the plumbing is per turbo / per kit.
    rg::TurboKitSizing sizing;
    sizing.scale = 1.0;
    const rg::TurboKitSpec* twin = cfg.find("sport_twin");
    REQUIRE(twin != nullptr);
    const rg::PartAttachment a = rg::turbo_kit_parts(cfg, *twin, sizing);
    CHECK(a.parent_id == "engine");
    CHECK(a.node.children.size() == 5); // 2 turbos, 2 exhaust sides, 1 intercooler
    CHECK(a.node.total_kg() == Approx(2 * cfg.turbo_anchor_kg + 2 * cfg.manifold_kg_per_turbo + cfg.intercooler_kg +
                                      cfg.intercooler_kg_per_extra_turbo));
    sizing.scale = 0.1; // a tiny wheel: held at the minimum
    CHECK(rg::turbo_kit_parts(cfg, *twin, sizing).node.children.front().own_kg == Approx(cfg.turbo_min_kg));

    // Through the garage files: the city car with each single kit.
    const rg::CatalogEntry& city = f.entry("car_city");
    const rg::PartTree stock = rg::stock_part_tree(city, f.table, f.ctx);
    INFO(stock.error);
    REQUIRE(stock.ok);
    CHECK(stock.root.find("engine.turbo_kit") == nullptr);
    double street_kg = 0.0;
    for (const char* kit : {"street_single", "sport_single"}) {
        const rg::CompiledSetup c = rg::compile_setup(city, f.table, f.ctx, setup_of("car_city", {{"turbo_kit", std::string(kit)}}));
        INFO(std::string(kit) + ": " + c.error + " " + c.parts.error);
        REQUIRE(c.ok);
        REQUIRE(c.parts.ok);
        const rg::PartNode* node = c.parts.root.find("engine.turbo_kit");
        REQUIRE(node != nullptr);
        CHECK(node->label == cfg.find(kit)->label);
        CHECK(node->children.size() == 3);
        CHECK(node->total_kg() >= cfg.turbo_min_kg + cfg.manifold_kg_per_turbo + cfg.intercooler_kg);
        CHECK(c.parts.total_kg == Approx(stock.total_kg + node->total_kg()));
        CHECK(c.parts.chassis_kg == Approx(c.parts.total_kg).margin(0.001));
        if (std::string(kit) == "street_single") street_kg = node->total_kg();
        else CHECK(node->total_kg() >= street_kg); // more boost: a larger wheel, never lighter
    }

    // Loader: the masses block is required.
    std::ifstream in(kRoot + "/data/engines/turbo_kits.json", std::ios::binary);
    nlohmann::json doc = nlohmann::json::parse(in);
    doc.erase("masses");
    const fs::path dir = fs::temp_directory_path() / "rg_turbo_kit_masses";
    fs::create_directories(dir);
    const std::string p = (dir / "kits.json").generic_string();
    std::ofstream(p, std::ios::binary) << doc.dump();
    CHECK_FALSE(rg::load_turbo_kit_config(p, kRoot, &err).has_value());
    CHECK_THAT(err, ContainsSubstring("masses"));
    fs::remove_all(dir);
}

TEST_CASE("turbo kits: a kit engine makes more power at the same rpm", "[turbo_kit][dyno][long]") {
    Fixture f;
    ps::jobs::JobSystem pool(2);
    for (const rg::EngineEntry* e : f.kit_engines()) {
        INFO(e->key);
        const Loaded na = load_setup(f, f.entry("car_hyper"), setup_of("car_hyper", {{"engine_install", e->key}}), &pool);
        INFO(na.error);
        REQUIRE(na.ok);
        const Loaded tk = load_setup(f, f.entry("car_hyper"), setup_of("car_hyper", {{"engine_install", e->key}, {"turbo_kit", std::string("street_single")}}), &pool);
        INFO(tk.error);
        REQUIRE(tk.ok);
        const ps::drivetrain::ComponentParams* pna = nullptr;
        const ps::drivetrain::ComponentParams* ptk = nullptr;
        const auto* sna = sim_engine(na.desc, &pna);
        const auto* stk = sim_engine(tk.desc, &ptk);
        REQUIRE(sna != nullptr);
        REQUIRE(stk != nullptr);
        REQUIRE(stk->turbo_pair.has_value());
        ps::drivetrain::DynoSweepSpec spec;
        const double lim = sna->limiter.rpm;
        for (double frac : {0.55, 0.75, 0.9}) spec.rpm.push_back(std::round(frac * lim / 50.0) * 50.0);
        spec.max_settle_s = 8.0;
        const auto rn = ps::drivetrain::run_dyno_sweep(*pna, nullptr, spec, &pool);
        const auto rt = ps::drivetrain::run_dyno_sweep(*ptk, nullptr, spec, &pool);
        REQUIRE(rn.points.size() == spec.rpm.size());
        REQUIRE(rt.points.size() == spec.rpm.size());
        for (std::size_t i = 0; i < spec.rpm.size(); ++i) {
            WARN(e->key << " @ " << spec.rpm[i] << " rpm: NA " << rn.points[i].power_kw << " kW, kit " << rt.points[i].power_kw
                        << " kW (x" << rt.points[i].power_kw / rn.points[i].power_kw << "), boost " << rt.points[i].boost_bar << " bar");
            CHECK(rt.points[i].power_kw > rn.points[i].power_kw);
        }
        // At the top of the band the kit is at full boost: clearly more.
        CHECK(rt.points.back().power_kw > 1.15 * rn.points.back().power_kw);
    }
}
