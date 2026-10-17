// test_turbo_kit.cpp - garage turbo kits (rg/turbo_kit.h; owner on PR #5: "Turbos cannot be fitted yet to many engines").
// One rule sizes a turbo system for any simulated engine without a turbo; the setup option "turbo_kit" fits it and
// converts the engine (boost grid, compression ratio). Torque-curve engines are refused with a reason.
#include "rg/engine_store.h"
#include "rg/engine_swap.h"
#include "rg/garage.h"
#include "rg/limit_advisor.h"
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
    // Any count on any eligible engine: a quad on the 1.0 L three splits the air four ways - each unit gets a quarter of
    // the flow, a smaller wheel (s ~ 1/sqrt(4), down to min_scale), a higher shaft speed and a lighter rotor.
    const auto pico = rg::read_engine_facts(f.store->find("default:pico_i3_1l0_na")->file, nullptr);
    REQUIRE(pico.has_value());
    const rg::TurboKitSizing one = rg::size_turbo_kit(c, sport, *pico, pico->limiter_rpm);
    const rg::TurboKitSizing four = rg::size_turbo_kit(c, *c.find("quad"), *pico, pico->limiter_rpm);
    REQUIRE(one.ok);
    REQUIRE(four.ok);
    CHECK(four.demand_kg_s == Approx(one.demand_kg_s)); // same engine, same target: the same air
    CHECK(four.flow_per_turbo_kg_s == Approx(one.flow_per_turbo_kg_s / 4.0));
    CHECK(four.scale < one.scale);
    CHECK(four.scale >= c.min_scale);
    CHECK(four.reference_rpm > one.reference_rpm);
    CHECK(four.rotor_inertia_kgm2 < one.rotor_inertia_kgm2);
    CHECK(four.max_shaft_rpm == Approx(four.reference_rpm * c.max_shaft_speed_factor));
    CHECK(four.unit_mass_kg < one.unit_mass_kg);
    CHECK(four.unit_diameter_mm < one.unit_diameter_mm);
    WARN("pico single: " << one.exducer_mm << " mm, " << one.unit_mass_kg << " kg; quad: 4 x " << four.exducer_mm << " mm, "
                         << four.unit_mass_kg << " kg each, " << four.reference_rpm << " rpm");
}

TEST_CASE("turbo kits: the engine with its turbos must fit the engine bay", "[turbo_kit]") {
    Fixture f;
    const rg::TurboKitConfig& c = *f.ctx.turbo_kits;
    const rg::EngineEntry* pico = f.store->find("default:pico_i3_1l0_na");
    REQUIRE(pico != nullptr);
    REQUIRE(pico->envelope_mm.has_value());
    const auto facts = rg::read_engine_facts(pico->file, nullptr);
    REQUIRE(facts.has_value());
    // The envelope rule: units in a row along the crank on the exhaust side, the charge pipe beside them.
    const rg::TurboKitSizing quad = rg::size_turbo_kit(c, *c.find("quad"), *facts, facts->limiter_rpm);
    const auto box = rg::turbo_kit_envelope_mm(c, 4, quad, *pico->envelope_mm, false);
    CHECK(box[0] == Approx(std::max((*pico->envelope_mm)[0], 4 * quad.unit_length_mm + 3 * c.unit_gap_mm)));
    CHECK(box[1] == Approx((*pico->envelope_mm)[1] + c.unit_side_protrusion * quad.unit_diameter_mm));
    // A V engine splits its turbos over both banks: a shorter row, wider overall.
    const auto vbox = rg::turbo_kit_envelope_mm(c, 4, quad, *pico->envelope_mm, true);
    CHECK(vbox[0] <= box[0]);
    CHECK(vbox[1] > box[1]);
    for (const auto& [car, kit] : std::vector<std::pair<std::string, std::string>>{
             {"car_city", "street_single"}, {"car_city", "quad"}, {"car_gt70", "quad"}, {"car_runabout", "quad"}}) {
        const rg::TurboKitSizing z = rg::size_turbo_kit(c, *c.find(kit), *facts, facts->limiter_rpm);
        const auto b = rg::turbo_kit_envelope_mm(c, c.find(kit)->count, z, *pico->envelope_mm, false);
        const rg::FitResult fit = rg::check_fit_size(f.entry(car), b);
        WARN(car << " + pico + " << kit << ": " << b[0] << " x " << b[1] << " x " << b[2] << " mm -> "
                 << (fit.allowed ? std::string("fits") : fit.reasons.front()));
    }
    // A quad on the pico fits the GT's long bay. Its row of four is longer than the engine: a bay with room for the engine
    // but not for the row refuses it, naming the dimension; the single still fits there.
    const rg::CompiledSetup gt = rg::compile_setup(f.entry("car_gt70"), f.table, f.ctx,
                                                   setup_of("car_gt70", {{"engine_install", std::string("default:pico_i3_1l0_na")}, {"turbo_kit", std::string("quad")}}));
    INFO(gt.error);
    CHECK(gt.ok);
    REQUIRE(gt.turbo_kit.has_value());
    CHECK(gt.turbo_kit->spec.count == 4);
    CHECK(gt.turbo_kit->envelope_mm[0] == Approx(box[0]));
    rg::CatalogEntry tight = f.entry("car_city");
    REQUIRE(tight.engine_bay_envelope.has_value());
    REQUIRE(tight.engine_bay_envelope->crank_axis == 'y');
    tight.engine_bay_envelope->size_mm[1] = (*pico->envelope_mm)[0] + 20.0; // room along the crank: the engine + 20 mm
    REQUIRE(box[0] > tight.engine_bay_envelope->size_mm[1]);
    const rg::CompiledSetup city = rg::compile_setup(tight, f.table, f.ctx, setup_of("car_city", {{"turbo_kit", std::string("quad")}}));
    CHECK_FALSE(city.ok);
    CHECK_THAT(city.error, ContainsSubstring("does not fit"));
    CHECK_THAT(city.error, ContainsSubstring("mm long"));
    const rg::CompiledSetup single = rg::compile_setup(tight, f.table, f.ctx, setup_of("car_city", {{"turbo_kit", std::string("street_single")}}));
    INFO(single.error);
    CHECK(single.ok);
    // The garage card of the quad says why before the player picks it.
    const rg::SetupModel m = rg::build_setup_model(tight, f.table, f.ctx);
    const rg::OptionView* v = view_of(m, "turbo_kit");
    REQUIRE(v != nullptr);
    bool quad_card = false;
    for (const rg::SetupPart& p : v->parts) {
        if (p.id != "quad") continue;
        quad_card = true;
        CHECK_THAT(p.detail, ContainsSubstring("does not fit the engine bay"));
    }
    CHECK(quad_card);
}

TEST_CASE("turbo kits: the boost slider re-sizes the kit for the target actually set", "[turbo_kit]") {
    Fixture f;
    const rg::CompiledSetup start = rg::compile_setup(f.entry("car_gt70"), f.table, f.ctx, setup_of("car_gt70", {{"turbo_kit", std::string("sport_single")}}));
    INFO(start.error);
    REQUIRE(start.ok);
    REQUIRE(start.turbo_kit.has_value());
    const rg::CompiledSetup more = rg::compile_setup(f.entry("car_gt70"), f.table, f.ctx,
                                                     setup_of("car_gt70", {{"turbo_kit", std::string("sport_single")}, {"turbo_boost_target", 1.8}}));
    INFO(more.error);
    REQUIRE(more.ok);
    REQUIRE(more.turbo_kit.has_value());
    CHECK(more.turbo_kit->spec.target_boost_bar == Approx(1.8 * start.turbo_kit->spec.target_boost_bar));
    CHECK(more.turbo_kit->sizing.demand_kg_s > start.turbo_kit->sizing.demand_kg_s);
    CHECK(more.turbo_kit->sizing.exducer_mm > start.turbo_kit->sizing.exducer_mm);
    CHECK(more.turbo_kit->sizing.torque_factor > start.turbo_kit->sizing.torque_factor);
    // The regenerated configuration carries the new target, so no turbo-file patch is left over.
    CHECK(more.patches.count("turbo") == 0);
    // It loads.
    const Loaded l = load_setup(f, f.entry("car_gt70"), setup_of("car_gt70", {{"turbo_kit", std::string("sport_single")}, {"turbo_boost_target", 1.8}}));
    INFO(l.error);
    REQUIRE(l.ok);
    const auto* sim = sim_engine(l.desc);
    REQUIRE(sim != nullptr);
    REQUIRE(sim->turbo_pair.has_value());
    CHECK(sim->turbo_pair->target_boost_pa / 1.0e5 == Approx(1.8 * start.turbo_kit->spec.target_boost_bar).margin(1e-6));
}

TEST_CASE("turbo kits: charge piping is a part, its rating is advisor data", "[turbo_kit][limit_advisor]") {
    Fixture f;
    const rg::TurboKitConfig& c = *f.ctx.turbo_kits;
    REQUIRE(c.find_piping("plastic") != nullptr);
    REQUIRE(c.find_piping("aluminium") != nullptr);
    REQUIRE(c.find_piping("steel") != nullptr);
    CHECK(c.default_piping == "plastic");
    for (const rg::ChargePipingSpec& p : c.piping) {
        INFO(p.id);
        CHECK(p.burst_bar > p.rated_bar);
        CHECK_THAT(p.source, ContainsSubstring("[AS]"));
    }
    CHECK(c.find_piping("plastic")->rated_bar < c.find_piping("aluminium")->rated_bar);
    CHECK(c.find_piping("aluminium")->rated_bar < c.find_piping("steel")->rated_bar);
    // The option: offered with a kit-capable engine, stock = the kit's plastic.
    const rg::SetupModel m = rg::build_setup_model(f.entry("car_gt70"), f.table, f.ctx);
    const rg::OptionView* v = view_of(m, "charge_piping");
    REQUIRE(v != nullptr);
    CHECK(v->available);
    CHECK_THAT(v->parts.front().label, ContainsSubstring("Plastic"));
    CHECK(v->choices.size() == 3);
    const rg::SetupModel hyper_model = rg::build_setup_model(f.entry("car_hyper"), f.table, f.ctx);
    const rg::OptionView* hv = view_of(hyper_model, "charge_piping");
    REQUIRE(hv != nullptr);
    CHECK_FALSE(hv->available); // its own twin turbo: no kit, no kit piping
    // Compiled: the choice reaches the build; metal piping is heavier.
    const auto kit_with = [&](const char* piping) {
        const rg::CompiledSetup s = rg::compile_setup(f.entry("car_gt70"), f.table, f.ctx,
                                                      setup_of("car_gt70", {{"turbo_kit", std::string("sport_single")}, {"charge_piping", std::string(piping)}}));
        INFO(s.error);
        REQUIRE(s.ok);
        REQUIRE(s.turbo_kit.has_value());
        return *s.turbo_kit;
    };
    const rg::TurboKitBuild plastic = kit_with("stock");
    const rg::TurboKitBuild steel = kit_with("steel");
    CHECK(plastic.piping.id == "plastic");
    CHECK(steel.piping.id == "steel");
    const auto piping_kg = [](const rg::TurboKitBuild& b) {
        for (const rg::TurboKitPart& p : b.parts) if (p.id == "charge_piping") return p.mass_kg;
        return 0.0;
    };
    CHECK(piping_kg(steel) > piping_kg(plastic));
    CHECK(steel.total_mass_kg() > plastic.total_mass_kg());

    // The advisor: the piping's rating against the boost (no sweep needed: the target alone).
    std::string err;
    const auto adv = rg::load_advisor_config(kRoot + "/data/garage/advisor.json", &err);
    INFO(err);
    REQUIRE(adv.has_value());
    const auto items_for = [&](const rg::TurboKitBuild& b, double target) {
        rg::CompiledSetup cs;
        cs.turbo_kit = b;
        rg::AdviceInput in;
        rg::set_part_limits(in, cs);
        in.target_boost_bar = target;
        return rg::build_advice(in, *adv);
    };
    const auto find = [](const std::vector<rg::AdviceItem>& items, const std::string& name) -> const rg::AdviceItem* {
        for (const rg::AdviceItem& i : items) if (i.rule_name == name) return &i;
        return nullptr;
    };
    CHECK(find(items_for(plastic, 1.0), "ChargePipingPressure") == nullptr); // well under 1.5 bar
    const auto over = items_for(plastic, 2.0);
    const rg::AdviceItem* warn = find(over, "ChargePipingPressure");
    REQUIRE(warn != nullptr);
    CHECK(warn->severity == rg::AdviceSeverity::Warning);
    CHECK(warn->option_id == "charge_piping");
    REQUIRE_FALSE(warn->lines.empty());
    CHECK_THAT(warn->lines.front(), ContainsSubstring("2.0 bar"));
    CHECK_THAT(warn->lines.front(), ContainsSubstring("Plastic charge piping"));
    const auto burst_items = items_for(plastic, 3.5);
    const rg::AdviceItem* burst = find(burst_items, "ChargePipingPressure");
    REQUIRE(burst != nullptr);
    CHECK(burst->variant == "burst");
    CHECK(burst->severity == rg::AdviceSeverity::Critical);
    CHECK(find(items_for(steel, 2.0), "ChargePipingPressure") == nullptr);
    // ... and the setup still compiles and saves: the advisor only warns.
    const rg::CompiledSetup high = rg::compile_setup(f.entry("car_gt70"), f.table, f.ctx,
                                                     setup_of("car_gt70", {{"turbo_kit", std::string("sport_single")}, {"turbo_boost_target", 2.5}}));
    INFO(high.error);
    CHECK(high.ok);

    // Rotor speed and mean piston speed from the sweep points.
    rg::AdviceInput in;
    rg::CompiledSetup cs;
    cs.turbo_kit = plastic;
    rg::set_part_limits(in, cs);
    in.target_boost_bar = 1.0;
    in.stroke_m = 0.09;
    ps::drivetrain::DynoPoint p;
    p.rpm = 9000.0; // 27 m/s with a 90 mm stroke
    p.turbo_rpm = 1.2 * plastic.sizing.max_shaft_rpm;
    p.charge_boost_bar = 1.0;
    in.points.push_back(p);
    const auto items = rg::build_advice(in, *adv);
    const rg::AdviceItem* shaft = find(items, "TurboShaftSpeed");
    REQUIRE(shaft != nullptr);
    CHECK(shaft->severity == rg::AdviceSeverity::Warning);
    const rg::AdviceItem* piston = find(items, "MeanPistonSpeed");
    REQUIRE(piston != nullptr);
    CHECK(piston->severity == rg::AdviceSeverity::Warning);
    CHECK_THAT(piston->lines.front(), ContainsSubstring("27.0 m/s"));
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
        CHECK(v->choices.size() >= 2); // stock + every kit (any count on any eligible engine; fit is checked on install)
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

    // The parts of one kit: one per turbo (mass follows the sized wheel), one exhaust side per turbo, the intercooler
    // (sized by the air flow) and the charge piping (length by the turbo count, mass by the material).
    const auto facts = rg::read_engine_facts(f.store->find("default:ventura_v8_5l0_na")->file, nullptr);
    REQUIRE(facts.has_value());
    const rg::TurboKitSpec* twin = cfg.find("sport_twin");
    REQUIRE(twin != nullptr);
    const rg::TurboKitSizing sizing = rg::size_turbo_kit(cfg, *twin, *facts, facts->limiter_rpm);
    REQUIRE(sizing.ok);
    const rg::TurboKitBuild build = rg::describe_turbo_kit(cfg, *twin, sizing, *cfg.find_piping("plastic"));
    const rg::PartAttachment a = rg::turbo_kit_parts(build);
    CHECK(a.parent_id == "engine");
    CHECK(a.node.children.size() == 6); // 2 turbos, 2 exhaust sides, 1 intercooler, 1 charge piping
    CHECK(a.node.total_kg() == Approx(build.total_mass_kg()));
    CHECK(build.total_mass_kg() == Approx(2 * sizing.unit_mass_kg + 2 * cfg.hot_side_mass_per_unit_kg + sizing.intercooler_mass_kg +
                                          build.parts.back().mass_kg));
    for (const rg::PartNode& n : a.node.children) {
        INFO(n.id);
        CHECK(n.own_kg > 0.0);
        CHECK_THAT(n.source, ContainsSubstring("[AS]"));
    }
    CHECK(sizing.unit_mass_kg == Approx(cfg.unit_mass_fixed_kg + cfg.unit_mass_scaled_kg * std::pow(sizing.scale, 3.0)));

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
        CHECK(node->children.size() == 4);
        CHECK(node->total_kg() > cfg.unit_mass_fixed_kg + cfg.hot_side_mass_per_unit_kg);
        REQUIRE(c.turbo_kit.has_value());
        CHECK(node->total_kg() == Approx(c.turbo_kit->total_mass_kg()));
        CHECK(c.parts.total_kg == Approx(stock.total_kg + node->total_kg()));
        CHECK(c.parts.chassis_kg == Approx(c.parts.total_kg - c.parts.physics_carried_kg).margin(0.001)); // the physics carries some parts itself (#22)
        if (std::string(kit) == "street_single") street_kg = node->total_kg();
        else CHECK(node->total_kg() >= street_kg); // more boost: a larger wheel, never lighter
    }

    // Loader: the mass block is required.
    std::ifstream in(kRoot + "/data/engines/turbo_kits.json", std::ios::binary);
    nlohmann::json doc = nlohmann::json::parse(in);
    doc.erase("mass");
    const fs::path dir = fs::temp_directory_path() / "rg_turbo_kit_masses";
    fs::create_directories(dir);
    const std::string p = (dir / "kits.json").generic_string();
    std::ofstream(p, std::ios::binary) << doc.dump();
    CHECK_FALSE(rg::load_turbo_kit_config(p, kRoot, &err).has_value());
    CHECK_THAT(err, ContainsSubstring("mass"));
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

// Owner on PR #21: "slap a quad turbo on a 1.0l 3 cylinder engine, if that would fit in the engine bay". The pico with the
// quad kit in the GT's bay: four tiny units (min_scale) on 1 L. It loads, makes boost and more power than without.
TEST_CASE("turbo kits: a quad on the 1.0 L three makes boost on the dyno", "[turbo_kit][dyno][long]") {
    Fixture f;
    ps::jobs::JobSystem pool(2);
    const auto setup = [](bool kit) {
        std::vector<std::pair<std::string, rg::SetupValue>> v{{"engine_install", std::string("default:pico_i3_1l0_na")}};
        if (kit) v.push_back({"turbo_kit", std::string("quad")});
        return setup_of("car_gt70", v);
    };
    const Loaded na = load_setup(f, f.entry("car_gt70"), setup(false), &pool);
    INFO(na.error);
    REQUIRE(na.ok);
    const Loaded q = load_setup(f, f.entry("car_gt70"), setup(true), &pool);
    INFO(q.error);
    REQUIRE(q.ok);
    const ps::drivetrain::ComponentParams* pna = nullptr;
    const ps::drivetrain::ComponentParams* pq = nullptr;
    const auto* sna = sim_engine(na.desc, &pna);
    const auto* sq = sim_engine(q.desc, &pq);
    REQUIRE(sna != nullptr);
    REQUIRE(sq != nullptr);
    REQUIRE(sq->turbo_pair.has_value());
    ps::drivetrain::DynoSweepSpec spec;
    const double lim = sna->limiter.rpm;
    for (double frac : {0.55, 0.75, 0.9}) spec.rpm.push_back(std::round(frac * lim / 50.0) * 50.0);
    spec.max_settle_s = 8.0;
    spec.turbo_count = 4;
    const auto rn = ps::drivetrain::run_dyno_sweep(*pna, nullptr, spec, &pool);
    const auto rq = ps::drivetrain::run_dyno_sweep(*pq, nullptr, spec, &pool);
    REQUIRE(rn.points.size() == spec.rpm.size());
    REQUIRE(rq.points.size() == spec.rpm.size());
    for (std::size_t i = 0; i < spec.rpm.size(); ++i) {
        WARN("pico quad @ " << spec.rpm[i] << " rpm: NA " << rn.points[i].power_kw << " kW, quad " << rq.points[i].power_kw << " kW (x"
                            << rq.points[i].power_kw / rn.points[i].power_kw << "), boost " << rq.points[i].boost_bar << " bar, rotor "
                            << rq.points[i].turbo_rpm << " rpm (rated " << q.compiled.turbo_kit->sizing.max_shaft_rpm << ")");
        CHECK(std::isfinite(rq.points[i].power_kw));
    }
    CHECK(rq.points.back().boost_bar > 0.5);
    CHECK(rq.points.back().power_kw > 1.3 * rn.points.back().power_kw);
}
