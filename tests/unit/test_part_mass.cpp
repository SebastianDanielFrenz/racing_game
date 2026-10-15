// test_part_mass.cpp - the per-part mass tree (rg/part_mass.h; owner on PR #19): the table loader, cumulative masses,
// the driveline derived from the vehicle file, physics_carries, every stock catalog car summing to its previous chassis
// mass, and setups/swaps changing the tree and the drive's chassis mass.
#include "rg/garage.h"
#include "rg/part_mass.h"
#include "rg/vehicle_catalog.h"
#include "rg/vehicle_setup.h"

#include <cmath>
#include <nlohmann/json.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
using json = nlohmann::json;
const std::string kRoot = RG_SOURCE_DIR;

rg::PartMassTable shipped_table() {
    std::string err;
    auto t = rg::load_part_mass_table(kRoot + "/data/vehicles/part_masses.json", &err);
    INFO(err);
    REQUIRE(t.has_value());
    return *t;
}

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    rg::ScopedWorkDir work{fs::temp_directory_path().generic_string(), "rg_part_mass_tests"};
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
        ctx.work_root = work.path();
        ctx.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
        auto store = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, "", &err);
        INFO(err);
        REQUIRE(store != nullptr);
        ctx.engines = std::shared_ptr<const rg::EngineStore>(std::move(store));
        auto boxes = rg::GearboxCatalog::open(kRoot + "/data/gearboxes/gearbox_catalog.json", kRoot, &err);
        INFO(err);
        REQUIRE(boxes != nullptr);
        ctx.gearboxes = std::shared_ptr<const rg::GearboxCatalog>(std::move(boxes));
        ctx.part_masses = std::make_shared<const rg::PartMassTable>(shipped_table());
    }
    const rg::CatalogEntry& entry(const std::string& id) const {
        const rg::CatalogEntry* e = catalog.find(id);
        REQUIRE(e != nullptr);
        return *e;
    }
};

std::string table_text() {
    std::ifstream in(kRoot + "/data/vehicles/part_masses.json", std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

std::string parse_error(const json& doc) {
    std::string err;
    const auto t = rg::parse_part_mass_table(doc.dump(), "t.json", &err);
    return t ? std::string() : err;
}

// A plain two-axle car: engine -> clutch -> gearbox -> one axle differential driving the named wheels.
json synthetic_vehicle(const std::string& layout) {
    json v;
    v["name"] = "synthetic";
    v["wheels"] = json::array();
    for (const char* n : {"FL", "FR", "RL", "RR"}) {
        const bool front = n[0] == 'F';
        v["wheels"].push_back({{"name", n}, {"is_front", front}, {"wheel_mass", front ? 16.0 : 20.0}});
    }
    v["brake_system"] = {{"corners", json::array({{{"wheel", "FL"}}, {{"wheel", "FR"}}, {{"wheel", "RL"}}, {{"wheel", "RR"}}})}};
    json comps = json::array({{{"id", "engine"}, {"type", "engine"}, {"ref", "e.json"}},
                              {{"id", "clutch"}, {"type", "clutch"}},
                              {{"id", "gearbox"}, {"type", "gearbox"}, {"ref", "g.json"}},
                              {{"id", "tank"}, {"type", "fuel_tank"}, {"capacity_kg", 50.0}}});
    json shafts = json::array({{{"ports", {"engine.crank", "clutch.in"}}}, {{"ports", {"clutch.out", "gearbox.in"}}}});
    if (layout == "AWD") {
        comps.push_back({{"id", "centre"}, {"type", "differential"}, {"kind", "viscous"}});
        comps.push_back({{"id", "front"}, {"type", "differential"}, {"kind", "open"}});
        comps.push_back({{"id", "rear"}, {"type", "differential"}, {"kind", "torsen"}});
        shafts.push_back({{"ports", {"gearbox.out", "centre.carrier"}}});
        shafts.push_back({{"ports", {"centre.left", "front.carrier"}}});
        shafts.push_back({{"ports", {"centre.right", "rear.carrier"}}});
        for (const char* w : {"FL", "FR"}) shafts.push_back({{"ports", {std::string("front.") + (w[1] == 'L' ? "left" : "right"), std::string("wheel:") + w}}});
        for (const char* w : {"RL", "RR"}) shafts.push_back({{"ports", {std::string("rear.") + (w[1] == 'L' ? "left" : "right"), std::string("wheel:") + w}}});
    } else {
        const bool front = layout == "FWD";
        comps.push_back({{"id", "diff"}, {"type", "differential"}, {"kind", "open"}});
        shafts.push_back({{"ports", {"gearbox.out", "diff.carrier"}}});
        shafts.push_back({{"ports", {"diff.left", front ? "wheel:FL" : "wheel:RL"}}});
        shafts.push_back({{"ports", {"diff.right", front ? "wheel:FR" : "wheel:RR"}}});
    }
    v["powertrain"] = {{"components", comps}, {"shafts", shafts}};
    return v;
}

rg::PartTreeInput synthetic_input(const std::string& layout, const std::string& bay = "front") {
    rg::PartTreeInput in;
    in.body_kg = 900.0;
    in.vehicle_json = synthetic_vehicle(layout).dump();
    in.engine_bay = bay;
    in.engine = rg::PartRef{"Test engine", 150.0, "test"};
    in.gearbox = rg::PartRef{"Test box", 40.0, "test"};
    return in;
}

double sum_leaves_and_own(const rg::PartNode& n) {
    double t = n.own_kg;
    for (const auto& c : n.children) t += sum_leaves_and_own(c);
    return t;
}

void print_tree(const rg::PartNode& n, int depth) {
    std::printf("%*s%-40s own %8.2f  total %8.2f  (%s)\n", depth * 2, "", (n.label + " [" + n.id + "]").c_str(), n.own_kg,
                n.total_kg(), n.source.c_str());
    for (const auto& c : n.children) print_tree(c, depth + 1);
}

} // namespace

TEST_CASE("part mass table: the shipped file loads and the loader is strict", "[part_mass]") {
    const rg::PartMassTable t = shipped_table();
    CHECK(t.physics_carries.empty());
    for (const auto& p : t.parts) {
        INFO(p.key);
        CHECK(p.mass_kg >= 0.0);
        // Every figure names where it comes from; every invented figure is marked [AS].
        CHECK_FALSE(p.source.empty());
        if (p.key != "wheel_default") CHECK_THAT(p.source, ContainsSubstring("[AS]"));
    }
    const json good = json::parse(table_text());
    CHECK(parse_error(good).empty());
    json no_clutch = good;
    no_clutch["parts"].erase("clutch");
    CHECK_THAT(parse_error(no_clutch), ContainsSubstring("parts.clutch missing"));
    json bad_carries = good;
    bad_carries["physics_carries"] = {"flux_capacitor"};
    CHECK_THAT(parse_error(bad_carries), ContainsSubstring("not a part key"));
    json body_carried = good;
    body_carried["physics_carries"] = {"body"};
    CHECK_THAT(parse_error(body_carried), ContainsSubstring("not a part key"));
    json no_source = good;
    no_source["parts"]["clutch"].erase("source");
    CHECK_THAT(parse_error(no_source), ContainsSubstring("source"));
    json negative = good;
    negative["parts"]["clutch"]["mass_kg"] = -1.0;
    CHECK_THAT(parse_error(negative), ContainsSubstring("mass_kg"));
    json fmt = good;
    fmt["format"] = "x";
    CHECK_THAT(parse_error(fmt), ContainsSubstring("rg.part_masses/1"));
}

TEST_CASE("part mass tree: cumulative masses, groups and the driveline read from the vehicle file", "[part_mass]") {
    const rg::PartMassTable t = shipped_table();
    auto kg = [&](const char* key) { return t.find(key)->mass_kg; };

    const rg::PartTree fwd = rg::build_part_tree(t, synthetic_input("FWD"));
    INFO(fwd.error);
    REQUIRE(fwd.ok);
    // A parent's cumulative mass is its own plus its children's; the total is every node's own mass once.
    CHECK(fwd.total_kg == Approx(sum_leaves_and_own(fwd.root)));
    const rg::PartNode* axle = fwd.root.find("drivetrain.front_axle");
    REQUIRE(axle != nullptr);
    CHECK(axle->own_kg == Approx(kg("diff_axle_open")));
    CHECK(axle->children.size() == 2); // one half shaft per driven wheel
    CHECK(axle->total_kg() == Approx(kg("diff_axle_open") + 2 * kg("halfshaft")));
    CHECK(fwd.root.find("drivetrain.rear_axle") == nullptr);
    CHECK(fwd.root.find("drivetrain.propshaft") == nullptr); // front engine, front axle
    CHECK(fwd.root.find("drivetrain.transfer_case") == nullptr);
    // wheels from the vehicle file's wheel_mass, a brake only where the brake system has a corner
    CHECK(fwd.root.find("running_gear.fl.wheel")->own_kg == 16.0);
    CHECK(fwd.root.find("running_gear.rr.wheel")->own_kg == 20.0);
    CHECK(fwd.root.find("running_gear.rl.brake")->own_kg == kg("brake_rear"));
    // the sized tank: fixed share + per kg of capacity
    CHECK(fwd.root.find("fuel_system.tank")->own_kg == Approx(kg("fuel_tank") + 50.0 * t.find("fuel_tank")->per_capacity_kg));
    // engine and gearbox: the catalogue figures handed in
    CHECK(fwd.root.find("engine")->own_kg == 150.0);
    CHECK(fwd.root.find("drivetrain.gearbox")->own_kg == 40.0);
    // groups: the body, then every child of the root with its cumulative mass; they add up to the total
    const auto groups = rg::mass_groups(fwd);
    REQUIRE(groups.size() >= 4);
    CHECK(groups[0].id == "body");
    CHECK(groups[0].kg == 900.0);
    double sum = 0.0;
    for (const auto& g : groups) sum += g.kg;
    CHECK(sum == Approx(fwd.total_kg));
    // nothing is carried by physics today: the chassis body is the whole car
    CHECK(fwd.chassis_kg == Approx(fwd.total_kg));

    // RWD with a front engine: a propshaft; mid engine: none
    const rg::PartTree rwd = rg::build_part_tree(t, synthetic_input("RWD"));
    REQUIRE(rwd.ok);
    CHECK(rwd.root.find("drivetrain.rear_axle") != nullptr);
    CHECK(rwd.root.find("drivetrain.propshaft") != nullptr);
    CHECK(rwd.total_kg == Approx(fwd.total_kg + kg("propshaft")));
    const rg::PartTree mid = rg::build_part_tree(t, synthetic_input("RWD", "mid"));
    REQUIRE(mid.ok);
    CHECK(mid.root.find("drivetrain.propshaft") == nullptr);

    // AWD: a transfer case (the centre's kind), both axles with their own kinds, a propshaft and four half shafts.
    const rg::PartTree awd = rg::build_part_tree(t, synthetic_input("AWD"));
    INFO(awd.error);
    REQUIRE(awd.ok);
    CHECK(awd.root.find("drivetrain.transfer_case")->own_kg == kg("diff_centre_viscous"));
    CHECK(awd.root.find("drivetrain.front_axle")->own_kg == kg("diff_axle_open"));
    CHECK(awd.root.find("drivetrain.rear_axle")->own_kg == kg("diff_axle_torsen"));
    CHECK(awd.root.find("drivetrain.propshaft") != nullptr);
    CHECK(awd.total_kg == Approx(fwd.total_kg + kg("diff_centre_viscous") + kg("diff_axle_torsen") + 2 * kg("halfshaft") +
                                 kg("propshaft")));
}

TEST_CASE("part mass tree: a layout other than the stock one carries the conversion penalty", "[part_mass][layout]") {
    const rg::PartMassTable t = shipped_table();
    auto kg = [&](const char* key) { return t.find(key)->mass_kg; };
    // stock layout `from`, converted to `to` (front engine unless `bay` says otherwise)
    auto convert = [&](const std::string& from, const std::string& to, const std::string& bay = "front") {
        rg::PartTreeInput in = synthetic_input(to, bay);
        in.stock_vehicle_json = synthetic_vehicle(from).dump();
        return rg::build_part_tree(t, in);
    };
    auto plain = [&](const std::string& layout, const std::string& bay = "front") {
        return rg::build_part_tree(t, synthetic_input(layout, bay));
    };
    const double fwd = plain("FWD").total_kg, rwd = plain("RWD").total_kg, awd = plain("AWD").total_kg;

    // the stock layout itself: no penalty, layouts reported
    const rg::PartTree same = convert("RWD", "RWD");
    REQUIRE(same.ok);
    CHECK(same.layout == "RWD");
    CHECK(same.stock_layout == "RWD");
    CHECK(same.root.find("drivetrain.layout_conversion") == nullptr);
    CHECK(same.total_kg == Approx(rwd));

    // FWD -> RWD: the rear axle and propshaft come from the file; the conversion adds its hardware and a transfer
    // drive (no driveline: the layout rule already gives the car one more propshaft than stock).
    const rg::PartTree f2r = convert("FWD", "RWD");
    INFO(f2r.error);
    REQUIRE(f2r.ok);
    const rg::PartNode* conv = f2r.root.find("drivetrain.layout_conversion");
    REQUIRE(conv != nullptr);
    CHECK(conv->label == "Layout conversion (FWD to RWD)");
    CHECK(conv->find("drivetrain.layout_conversion.transfer_drive") != nullptr);
    CHECK(conv->find("drivetrain.layout_conversion.driveline") == nullptr);
    CHECK(f2r.total_kg == Approx(rwd + kg("layout_conversion") + kg("conversion_transfer")));
    // shown as part of the drivetrain group of the breakdown
    double dt = 0.0;
    for (const auto& g : rg::mass_groups(f2r)) dt += g.id == "drivetrain" ? g.kg : 0.0;
    CHECK(dt == Approx(f2r.root.find("drivetrain")->total_kg()));

    // RWD -> FWD: the drive still has to run along the car: a driveline on top
    const rg::PartTree r2f = convert("RWD", "FWD");
    REQUIRE(r2f.ok);
    CHECK(r2f.root.find("drivetrain.layout_conversion.driveline") != nullptr);
    CHECK(r2f.total_kg == Approx(fwd + kg("layout_conversion") + kg("conversion_transfer") + kg("conversion_driveline")));

    // AWD conversions: the transfer case, the extra axle and the propshaft come from the file; FWD -> AWD adds the
    // hardware only, RWD -> AWD also the second shaft to the front axle.
    const rg::PartTree f2a = convert("FWD", "AWD");
    REQUIRE(f2a.ok);
    CHECK(f2a.root.find("drivetrain.layout_conversion.transfer_drive") == nullptr);
    CHECK(f2a.total_kg == Approx(awd + kg("layout_conversion")));
    const rg::PartTree r2a = convert("RWD", "AWD");
    REQUIRE(r2a.ok);
    CHECK(r2a.total_kg == Approx(awd + kg("layout_conversion") + kg("conversion_driveline")));

    // the owner's comparison: a 2WD swap costs about what an AWD conversion costs minus the axle no longer driven
    const double awd_conversion = f2a.total_kg - fwd;
    const double axle = kg("diff_axle_open") + 2 * kg("halfshaft");
    // (between 60 % and 100 % of it: the 2WD swap's transfer drive is lighter than an AWD transfer case, and
    // RWD -> FWD loses the stock propshaft the driveline replaces)
    auto comparable = [](double swap, double awd_minus_axle) { return swap >= 0.6 * awd_minus_axle && swap <= awd_minus_axle; };
    CHECK(comparable(f2r.total_kg - fwd, awd_conversion - axle));
    CHECK(comparable(r2f.total_kg - rwd, r2a.total_kg - rwd - (kg("diff_axle_torsen") + 2 * kg("halfshaft"))));
    // every swap direction is heavier than the same layout built that way from the factory
    CHECK(convert("AWD", "RWD").total_kg > rwd);
    CHECK(convert("AWD", "FWD").total_kg > fwd);
    CHECK(convert("RWD", "FWD", "mid").total_kg > plain("FWD", "mid").total_kg);
}

TEST_CASE("part mass tree: physics_carries leaves a part out of the chassis body, extras attach under a parent", "[part_mass]") {
    rg::PartMassTable t = shipped_table();
    const rg::PartTree plain = rg::build_part_tree(t, synthetic_input("FWD"));
    REQUIRE(plain.ok);
    // Sabotage the "physics carries nothing" fact: if physics simulated the wheels as masses of their own, the chassis
    // body would have to leave them out - the total stays.
    t.physics_carries.push_back("wheel");
    const rg::PartTree carried = rg::build_part_tree(t, synthetic_input("FWD"));
    REQUIRE(carried.ok);
    CHECK(carried.total_kg == Approx(plain.total_kg));
    CHECK(carried.physics_carried_kg == Approx(2 * 16.0 + 2 * 20.0));
    CHECK(carried.chassis_kg == Approx(plain.chassis_kg - 72.0));

    t.physics_carries.clear();
    rg::PartTreeInput in = synthetic_input("FWD");
    rg::PartNode turbo;
    turbo.id = "turbo";
    turbo.label = "Turbocharger";
    turbo.own_kg = 12.5;
    rg::PartNode ic;
    ic.id = "intercooler";
    ic.label = "Intercooler";
    ic.own_kg = 7.0;
    turbo.children.push_back(ic);
    in.extras.push_back({"engine", turbo});
    const rg::PartTree with = rg::build_part_tree(t, in);
    INFO(with.error);
    REQUIRE(with.ok);
    REQUIRE(with.root.find("engine.turbo") != nullptr);
    CHECK(with.root.find("engine")->total_kg() == Approx(plain.root.find("engine")->total_kg() + 19.5));
    CHECK(with.total_kg == Approx(plain.total_kg + 19.5));
    in.extras[0].parent_id = "no_such_part";
    const rg::PartTree orphan = rg::build_part_tree(t, in);
    CHECK_FALSE(orphan.ok);
    CHECK_THAT(orphan.error, ContainsSubstring("no_such_part"));

    // an unknown component type is an error, not a silent zero
    json v = synthetic_vehicle("FWD");
    v["powertrain"]["components"].push_back({{"id", "x"}, {"type", "flux_capacitor"}});
    rg::PartTreeInput bad = synthetic_input("FWD");
    bad.vehicle_json = v.dump();
    const rg::PartTree unknown = rg::build_part_tree(t, bad);
    CHECK_FALSE(unknown.ok);
    CHECK_THAT(unknown.error, ContainsSubstring("flux_capacitor"));
    // no body mass, no tree
    rg::PartTreeInput nobody = synthetic_input("FWD");
    nobody.body_kg = 0.0;
    CHECK_FALSE(rg::build_part_tree(t, nobody).ok);
}

TEST_CASE("part mass tree: every stock catalog car sums to its previous chassis mass", "[part_mass][garage]") {
    Fixture f;
    int base = 0;
    for (const rg::CatalogEntry& e : f.catalog.entries) {
        INFO(e.id);
        REQUIRE(e.chassis.body_mass_kg.has_value());
        CHECK_FALSE(e.chassis.body_mass_source.empty());
        const rg::PartTree tree = rg::stock_part_tree(e, f.table, f.ctx);
        INFO(tree.error);
        REQUIRE(tree.ok);
        CHECK(tree.root.own_kg == Approx(*e.chassis.body_mass_kg));
        if (!e.preset_of.empty()) continue; // a preset's overlay may change parts; its base is checked
        ++base;
        CHECK(tree.total_kg == Approx(e.chassis.mass_kg).margin(1e-6));
        CHECK(tree.chassis_kg == Approx(e.chassis.mass_kg).margin(1e-6));
        // every stock engine and gearbox is a catalogue part (no fallback figure)
        CHECK(tree.root.find("engine")->source == "engine catalogue");
        const rg::PartNode* box = tree.root.find("drivetrain.gearbox");
        REQUIRE(box != nullptr);
        CHECK(box->source == "gearbox catalogue");
    }
    CHECK(base >= 12);
}

TEST_CASE("part mass tree: setups and swaps change the tree and the drive's chassis mass", "[part_mass][garage]") {
    Fixture f;
    const rg::CatalogEntry& rwd = f.entry("car_sedan_rwd");
    auto compile = [&](const rg::CatalogEntry& e, std::initializer_list<std::pair<const char*, rg::SetupValue>> values) {
        rg::VehicleSetup s;
        s.vehicle_id = e.id;
        for (const auto& kv : values) s.values[kv.first] = kv.second;
        rg::CompiledSetup c = rg::compile_setup(e, f.table, f.ctx, s);
        INFO(c.error);
        REQUIRE(c.ok);
        INFO(c.parts.error);
        REQUIRE(c.parts.ok);
        return c;
    };
    const rg::PartTree stock = rg::stock_part_tree(rwd, f.table, f.ctx);
    REQUIRE(stock.ok);
    // engine swap: the engine node takes the new engine's catalogue figure, the rest stays
    const auto i6 = compile(rwd, {{"engine_install", std::string("default:n52b30_3l_na")}});
    CHECK(i6.parts.root.find("engine")->own_kg == Approx(165.0));
    CHECK(i6.parts.total_kg == Approx(stock.total_kg + 165.0 - 105.0));
    CHECK(i6.chassis_mass_delta_kg == Approx(60.0)); // the swap line keeps the engine difference
    // gearbox swap (no mass before the tree): the gearbox node takes the new box's catalogue figure
    const auto box = compile(rwd, {{"gearbox_install", std::string("heavy_duty_6mt")}});
    const rg::GearboxEntry* hd = f.ctx.gearboxes->find("heavy_duty_6mt");
    const rg::GearboxEntry* own = f.ctx.gearboxes->find("touring_6mt");
    REQUIRE(hd != nullptr);
    REQUIRE(own != nullptr);
    CHECK(box.parts.root.find("drivetrain.gearbox")->own_kg == Approx(hd->mass_kg));
    CHECK(box.parts.total_kg == Approx(stock.total_kg + hd->mass_kg - own->mass_kg));
    // an assist unit the sedan has none of: a new electronics part
    const auto assists = compile(rwd, {{"assist_unit", std::string("assists_street")}});
    CHECK(assists.parts.total_kg == Approx(stock.total_kg + f.ctx.part_masses->find("electronics_assist_unit")->mass_kg));
    // a tuning value changes no part
    const auto spring = compile(rwd, {{"spring_front", 1.2}});
    CHECK(spring.parts.total_kg == Approx(stock.total_kg));
}

TEST_CASE("part mass tree: the garage view and the prepared drive use the tree", "[part_mass][garage]") {
    rg::ScopedWorkDir user{fs::temp_directory_path().generic_string(), "rg_part_mass_user"};
    rg::ScopedWorkDir work{fs::temp_directory_path().generic_string(), "rg_part_mass_work"};
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
    const double stock_mass = g->catalog().find("car_sedan_rwd")->chassis.mass_kg;
    // stock drive: the tree's chassis mass is the catalog figure
    const auto stock_drive = g->prepare_drive("car_sedan_rwd");
    REQUIRE(stock_drive.ok);
    REQUIRE(stock_drive.parts.ok);
    CHECK(stock_drive.chassis.mass_kg == Approx(stock_mass));
    // edit: the view follows every change
    REQUIRE(g->begin_edit("car_sedan_rwd", &err));
    REQUIRE(g->parts().ok);
    CHECK(g->parts().total_kg == Approx(stock_mass));
    REQUIRE(g->set_option("gearbox_install", std::string("heavy_duty_6mt")).accepted);
    const double box_delta = g->context().gearboxes->find("heavy_duty_6mt")->mass_kg - g->context().gearboxes->find("touring_6mt")->mass_kg;
    CHECK(g->parts().total_kg == Approx(stock_mass + box_delta));
    REQUIRE(g->set_option("engine_install", std::string("default:n52b30_3l_na")).accepted);
    CHECK(g->parts().total_kg == Approx(stock_mass + box_delta + 60.0));
    REQUIRE(g->save(&err));
    CHECK(g->part_tree("car_sedan_rwd").total_kg == Approx(stock_mass + box_delta + 60.0));
    const auto drive = g->prepare_drive("car_sedan_rwd");
    INFO(drive.error);
    REQUIRE(drive.ok);
    REQUIRE(drive.parts.ok);
    CHECK(drive.chassis.mass_kg == Approx(stock_mass + box_delta + 60.0));
    g->cleanup();
}

TEST_CASE("part mass tree: print every stock car's tree", "[.][diag][part_mass]") {
    Fixture f;
    for (const rg::CatalogEntry& e : f.catalog.entries) {
        if (!e.preset_of.empty()) continue;
        rg::CatalogEntry probe = e;
        probe.chassis.body_mass_kg = 100.0;
        const rg::PartTree tree = rg::stock_part_tree(probe, f.table, f.ctx);
        std::printf("== %s  catalog %.2f  parts without body %.6f  -> body %.6f  %s\n", e.id.c_str(), e.chassis.mass_kg,
                    tree.total_kg - 100.0, e.chassis.mass_kg - (tree.total_kg - 100.0), tree.error.c_str());
        if (tree.ok) print_tree(tree.root, 1);
    }
}
