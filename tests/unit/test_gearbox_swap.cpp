// test_gearbox_swap.cpp - the gearbox swap (owner request 2026-10-13): the catalogue (data/gearboxes/gearbox_catalog.json),
// the rating check against the engine's peak torque, the garage offering exactly the compatible boxes, a setup round trip
// through the real physics loader, the auto-shift points against each box's gear steps, and short drives on auto-shift.
#include "rg/engine_store.h"
#include "rg/garage.h"
#include "rg/gearbox_catalog.h"
#include "rg/session.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <set>
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
    std::shared_ptr<const rg::EngineStore> store;
    std::shared_ptr<const rg::GearboxCatalog> boxes;
    rg::ScopedWorkDir isolated_root{fs::temp_directory_path().generic_string(), "rg_gearbox_unit_tests"};
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
        ctx.work_root = isolated_root.path();
        ctx.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
        auto s = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, "", &err);
        INFO(err);
        REQUIRE(s != nullptr);
        store = std::shared_ptr<const rg::EngineStore>(std::move(s));
        auto g = rg::GearboxCatalog::open(kRoot + "/data/gearboxes/gearbox_catalog.json", kRoot, &err);
        INFO(err);
        REQUIRE(g != nullptr);
        boxes = std::shared_ptr<const rg::GearboxCatalog>(std::move(g));
        ctx.engines = store;
        ctx.gearboxes = boxes;
    }
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

// The file a component of the car's vehicle JSON refers to ("engine" / "gearbox"), normalised.
std::string component_ref(const rg::CatalogEntry& e, const std::string& type) {
    std::ifstream in(e.vehicle_path);
    const nlohmann::json j = nlohmann::json::parse(in);
    for (const auto& c : j.at("powertrain").at("components")) {
        if (c.value("type", "") == type && c.contains("ref")) {
            return (fs::path(e.vehicle_path).parent_path() / c.at("ref").get<std::string>()).lexically_normal().generic_string();
        }
    }
    return {};
}

double engine_peak_of(const Fixture& f, const rg::CatalogEntry& e) {
    const rg::EngineEntry* eng = f.store->find_by_file(component_ref(e, "engine"));
    REQUIRE(eng != nullptr);
    const rg::EnginePeaks p = f.store->peaks(*eng);
    REQUIRE(p.known);
    return p.torque_nm;
}

struct Loaded {
    bool ok = false;
    std::string error;
    ps::vehicle::VehicleDesc desc;
    rg::CompiledSetup compiled;
    std::string vehicle_path;
};
// Materialise and load with the physics loader without generating engine maps (validate-only).
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

const ps::drivetrain::GearboxDesc* gearbox_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* g = std::get_if<ps::drivetrain::GearboxDesc>(&c.params)) return g;
    }
    return nullptr;
}
const ps::drivetrain::ManualTcuDesc* tcu_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.controllers) {
        if (const auto* g = std::get_if<ps::drivetrain::ManualTcuDesc>(&c.params)) return g;
    }
    return nullptr;
}

// The ids the garage lists for a car (every part of the gearbox option), in list order.
std::vector<std::string> offered_ids(const Fixture& f, const rg::CatalogEntry& e) {
    const rg::SetupModel m = rg::build_setup_model(e, f.table, f.ctx);
    REQUIRE(m.error.empty());
    std::vector<std::string> out;
    for (const rg::OptionView& v : m.options) {
        if (v.def.id != "gearbox_install") continue;
        REQUIRE(v.available);
        for (const rg::SetupPart& p : v.parts) out.push_back(p.id);
    }
    return out;
}

// The largest ratio between two neighbouring forward gears.
double max_step(const ps::drivetrain::GearboxDesc& g) {
    double step = 1.0;
    for (std::size_t i = 0; i + 1 < g.forward_ratios.size(); ++i) step = std::max(step, g.forward_ratios[i] / g.forward_ratios[i + 1]);
    return step;
}
} // namespace

TEST_CASE("gearbox catalogue: the shipped rows load with their kinds, ratings and sections", "[gearbox_swap][garage]") {
    Fixture f;
    const auto& list = f.boxes->list();
    CHECK(list.size() == 7);
    std::set<std::string> ids;
    for (const auto& g : list) {
        INFO(g.id);
        ids.insert(g.id);
        CHECK(g.rating_nm > 0.0);
        CHECK(g.gears >= 5);
        CHECK(g.first_ratio > g.top_ratio);
        CHECK(g.first_ratio > 1.0);
        CHECK(fs::exists(g.file));
    }
    CHECK(ids.size() == list.size());
    const rg::GearboxEntry* straight = f.boxes->find("straight_cut_6sq");
    REQUIRE(straight != nullptr);
    CHECK(straight->kind == rg::GearboxKind::Sequential);
    CHECK(straight->gear_type == "straight_cut");
    CHECK(straight->focus == "race");
    CHECK(rg::gearbox_section(*straight) == "Manual");
    // Automatic and CVT have no entries until physics models them: the sections are known and empty.
    const auto sections = f.boxes->sections();
    REQUIRE(sections.size() == 3);
    CHECK(sections[0].label == "Manual");
    CHECK(sections[0].has_entries);
    CHECK(sections[1].label == "Automatic");
    CHECK_FALSE(sections[1].has_entries);
    CHECK(sections[2].label == "CVT");
    CHECK_FALSE(sections[2].has_entries);
    // the stock boxes are the catalogue's too (found by file)
    CHECK(f.boxes->find_by_file(component_ref(f.entry("car_hyper"), "gearbox")) == f.boxes->find("hyper_7mt"));
    CHECK(f.boxes->find_by_file(component_ref(f.entry("car_sedan"), "gearbox")) == f.boxes->find("touring_6mt"));
}

TEST_CASE("gearbox catalogue: a bad catalogue is rejected with the reason", "[gearbox_swap][garage]") {
    Fixture f;
    const fs::path dir = fs::path(f.ctx.work_root) / "bad_gearbox_catalogue";
    fs::create_directories(dir);
    const auto write = [&](const std::string& text) {
        std::ofstream(dir / "c.json") << text;
        std::string err;
        const auto cat = rg::GearboxCatalog::open((dir / "c.json").generic_string(), kRoot, &err);
        return cat ? std::string() : err;
    };
    CHECK_THAT(write(R"({"format":"x","gearboxes":[]})"), ContainsSubstring("format must be rg.gearbox_catalog/1"));
    const std::string row = R"({"id":"a","name":"A","kind":"manual","focus":"touring","gear_type":"helical","synchro":"cone",
        "rating_nm":300,"file":"external/physics_sim/data/gearboxes/sedan_6mt.json")";
    CHECK(write(R"({"format":"rg.gearbox_catalog/1","gearboxes":[)" + row + "}]}").empty());
    CHECK_THAT(write(R"({"format":"rg.gearbox_catalog/1","gearboxes":[)" + row + "}," + row + "}]}"), ContainsSubstring("duplicate"));
    std::string no_rating = row;
    no_rating.replace(no_rating.find("\"rating_nm\":300"), 15, "\"rating_nm\":0");
    CHECK_THAT(write(R"({"format":"rg.gearbox_catalog/1","gearboxes":[)" + no_rating + "}]}"), ContainsSubstring("rating_nm"));
    std::string wrong_kind = row;
    wrong_kind.replace(wrong_kind.find("\"manual\""), 8, "\"sequential\"");
    CHECK_THAT(write(R"({"format":"rg.gearbox_catalog/1","gearboxes":[)" + wrong_kind + "}]}"), ContainsSubstring("file's type is manual"));
}

TEST_CASE("gearbox swap: every catalogue box loads in each car it is compatible with", "[gearbox_swap][garage]") {
    Fixture f;
    int loads = 0;
    for (const rg::CatalogEntry* car : f.base_cars()) {
        const double peak = engine_peak_of(f, *car);
        for (const rg::GearboxEntry& box : f.boxes->list()) {
            if (!rg::check_gearbox_rating(box, peak).empty()) continue;
            INFO(car->id << " + " << box.id);
            const Loaded l = load_setup(f, *car, setup_of(car->id, {{"gearbox_install", box.id}}));
            INFO(l.error);
            REQUIRE(l.ok);
            const auto* gb = gearbox_of(l.desc);
            REQUIRE(gb != nullptr);
            REQUIRE(static_cast<int>(gb->forward_ratios.size()) == box.gears);
            CHECK(gb->forward_ratios.front() == Approx(box.first_ratio));
            CHECK(gb->forward_ratios.back() == Approx(box.top_ratio));
            CHECK((box.kind == rg::GearboxKind::Sequential) == (gb->type == ps::drivetrain::GearboxType::Sequential));
            // the controller still drives the (new) gearbox component and the car keeps its auto-shift
            REQUIRE(tcu_of(l.desc) != nullptr);
            ++loads;
        }
    }
    CHECK(loads >= 20);
}

TEST_CASE("gearbox swap: the garage offers exactly the boxes the engine's torque allows, plus the car's own", "[gearbox_swap][garage]") {
    Fixture f;
    for (const rg::CatalogEntry* car : f.base_cars()) {
        INFO(car->id);
        const double peak = engine_peak_of(f, *car);
        const rg::GearboxEntry* own = f.boxes->find_by_file(component_ref(*car, "gearbox"));
        REQUIRE(own != nullptr);
        // The car's own box is listed under the id "stock" (choosing it changes nothing).
        std::set<std::string> expected;
        for (const rg::GearboxEntry& box : f.boxes->list()) {
            if (&box == own) expected.insert("stock");
            else if (rg::check_gearbox_rating(box, peak).empty()) expected.insert(box.id);
        }
        const std::vector<std::string> got = offered_ids(f, *car);
        CHECK(std::set<std::string>(got.begin(), got.end()) == expected);
        CHECK(got.size() == expected.size());
    }
    // the two hypercars: the stock 7-speed and the straight-cut race box, nothing else takes 1387 N*m
    for (const char* id : {"car_hyper", "car_hyper_n2o"}) {
        const std::vector<std::string> got = offered_ids(f, f.entry(id));
        CHECK(std::set<std::string>(got.begin(), got.end()) == std::set<std::string>{"stock", "straight_cut_6sq"});
    }
    // an economy sedan (about 150 N*m) is offered the whole catalogue
    const std::vector<std::string> sedan = offered_ids(f, f.entry("car_sedan_rwd"));
    CHECK(sedan.size() == f.boxes->list().size());
}

TEST_CASE("gearbox swap: a box rated below the engine's torque is turned away with a plain message", "[gearbox_swap][garage]") {
    Fixture f;
    const auto& hyper = f.entry("car_hyper");
    const double peak = engine_peak_of(f, hyper);
    const rg::GearboxEntry* weak = f.boxes->find("heavy_duty_6mt");
    REQUIRE(weak != nullptr);
    REQUIRE(weak->rating_nm < peak);
    const rg::CompiledSetup c = rg::compile_setup(hyper, f.table, f.ctx, setup_of("car_hyper", {{"gearbox_install", std::string("heavy_duty_6mt")}}));
    CHECK_FALSE(c.ok);
    CHECK_THAT(c.error, ContainsSubstring("gearbox: rated 700 N*m, engine makes "));
    CHECK_THAT(c.error, ContainsSubstring(rg::check_gearbox_rating(*weak, peak)));
    // the same sentence from the pure function, with an unknown torque letting every box through
    CHECK(rg::check_gearbox_rating(*weak, 0.0).empty());
    CHECK_THAT(rg::check_gearbox_rating(*f.boxes->find("close_ratio_6mt"), 1371.0),
               ContainsSubstring("gearbox: rated 550 N*m, engine makes 1371 N*m"));
    // an engine swapped in is the one the rating is checked against: the hypercar's box does not take the V8 once
    // the sedan's I4 is installed (it does - 1600 - and the sedan's own does not take the V8 either)
    const rg::CompiledSetup v8_in_sedan = rg::compile_setup(
        f.entry("car_sedan_rwd"), f.table, f.ctx,
        setup_of("car_sedan_rwd", {{"engine_install", std::string("default:hyper_v8_one1_audio")}, {"gearbox_install", std::string("close_ratio_6mt")}}));
    CHECK_FALSE(v8_in_sedan.ok);
    CHECK_THAT(v8_in_sedan.error, ContainsSubstring("gearbox: rated 550 N*m"));
    const rg::CompiledSetup v8_ok = rg::compile_setup(
        f.entry("car_sedan_rwd"), f.table, f.ctx,
        setup_of("car_sedan_rwd", {{"engine_install", std::string("default:hyper_v8_one1_audio")}, {"gearbox_install", std::string("hyper_7mt")}}));
    INFO(v8_ok.error);
    CHECK(v8_ok.ok);
    // an unknown id and the car's own box chosen explicitly
    const rg::CompiledSetup unknown = rg::compile_setup(f.entry("car_sedan"), f.table, f.ctx, setup_of("car_sedan", {{"gearbox_install", std::string("nope")}}));
    CHECK_FALSE(unknown.ok);
    const Loaded own = load_setup(f, f.entry("car_sedan"), setup_of("car_sedan", {{"gearbox_install", std::string("touring_6mt")}}));
    REQUIRE(own.ok);
    CHECK(own.compiled.patches.count("gearbox") == 0);
}

TEST_CASE("gearbox swap: a swapped box round-trips through the saved setup and the materialised car", "[gearbox_swap][garage]") {
    Fixture f;
    const auto& car = f.entry("car_sedan_rwd");
    rg::VehicleSetup s = setup_of("car_sedan_rwd", {{"gearbox_install", std::string("econ_5mt")}});
    const std::string text = rg::setup_to_json(s);
    std::string err;
    const auto parsed = rg::parse_setup(text, f.table, "test", &err);
    INFO(err);
    REQUIRE(parsed.has_value());
    CHECK(std::get<std::string>(parsed->values.at("gearbox_install")) == "econ_5mt");
    const Loaded plain = load_setup(f, car, *parsed);
    INFO(plain.error);
    REQUIRE(plain.ok);
    const auto* gb = gearbox_of(plain.desc);
    REQUIRE(gb != nullptr);
    REQUIRE(gb->forward_ratios.size() == 5);
    CHECK(gb->forward_ratios[0] == Approx(3.45));
    CHECK(gb->forward_ratios[4] == Approx(0.70));
    // the per-gear scaling applies to the NEW box: five factors for five gears
    rg::VehicleSetup scaled = *parsed;
    scaled.values["gear_ratios"] = std::vector<double>{1.1, 1.0, 1.0, 1.0, 0.9};
    const Loaded sc = load_setup(f, car, scaled);
    INFO(sc.error);
    REQUIRE(sc.ok);
    const auto* sg = gearbox_of(sc.desc);
    REQUIRE(sg != nullptr);
    CHECK(sg->forward_ratios[0] == Approx(3.45 * 1.1));
    CHECK(sg->forward_ratios[4] == Approx(0.70 * 0.9));
    // six factors for the 5-speed are a length error, not a silent truncation
    rg::VehicleSetup wrong = *parsed;
    wrong.values["gear_ratios"] = std::vector<double>{1.1, 1.0, 1.0, 1.0, 1.0, 1.0};
    const Loaded bad = load_setup(f, car, wrong);
    CHECK_FALSE(bad.ok);
    CHECK_THAT(bad.error, ContainsSubstring("needs 5 values, got 6"));
    // the stock car is untouched by all this
    const Loaded stock = load_setup(f, car, setup_of("car_sedan_rwd", {}));
    REQUIRE(stock.ok);
    CHECK(gearbox_of(stock.desc)->forward_ratios.size() == 6);
}

TEST_CASE("gearbox swap: garage edits reset the ratios on a new box and drop an unfit box after an engine change", "[gearbox_swap][garage]") {
    const fs::path user = fs::temp_directory_path() / "rg_gearbox_garage_user";
    const fs::path work = fs::temp_directory_path() / "rg_gearbox_garage_work";
    std::error_code ec;
    fs::remove_all(user, ec);
    fs::remove_all(work, ec);
    fs::create_directories(user, ec);
    fs::create_directories(work, ec);
    rg::GarageConfig cfg;
    cfg.repo_root = kRoot;
    cfg.catalog_path = kRoot + "/data/vehicles/catalog.json";
    cfg.options_path = kRoot + "/data/vehicles/setup_options.json";
    cfg.set_path = kRoot + "/data/garage/garage_set.json";
    cfg.tyre_families_path = kRoot + "/data/tyres/families.json";
    cfg.user_dir = user.generic_string();
    cfg.work_root = work.generic_string();
    cfg.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
    std::string err;
    auto g = rg::Garage::open(cfg, &err);
    INFO(err);
    REQUIRE(g != nullptr);
    REQUIRE(g->begin_edit("car_sedan_rwd", &err));
    // ratios edited on the stock box are dropped (with a notice) when another box comes in
    REQUIRE(g->set_option("gear_ratios", std::vector<double>{1.05, 1.0, 1.0, 1.0, 1.0, 1.0}).accepted);
    const auto r = g->set_option("gearbox_install", std::string("close_ratio_6mt"));
    INFO(r.message);
    REQUIRE(r.accepted);
    CHECK_THAT(r.notice, ContainsSubstring("gear ratios were reset"));
    CHECK(g->working().values.count("gear_ratios") == 0);
    CHECK(std::get<std::string>(g->current_value("gearbox_install")) == "close_ratio_6mt");
    CHECK(g->validation().ok);
    // a box the engine does not fit is refused by the edit itself
    REQUIRE(g->begin_edit("car_hyper", &err));
    const auto weak = g->set_option("gearbox_install", std::string("heavy_duty_6mt"));
    CHECK_FALSE(weak.validation.ok); // the edit is taken, the setup does not validate and says why
    CHECK_THAT(weak.validation.message, ContainsSubstring("gearbox: rated 700 N*m"));
    CHECK_FALSE(g->validation().ok);
    const auto strong = g->set_option("gearbox_install", std::string("straight_cut_6sq"));
    CHECK(strong.validation.ok);
    CHECK(g->validation().ok);
    // a stronger engine change later leaves a too-weak box with a notice, but the swap stays editable
    REQUIRE(g->begin_edit("car_sedan_rwd", &err));
    REQUIRE(g->set_option("gearbox_install", std::string("econ_5mt")).accepted);
    const auto engine = g->set_option("engine_install", std::string("default:hyper_v8_one1_audio"));
    INFO(engine.message);
    REQUIRE(engine.accepted);
    CHECK_FALSE(engine.notice.empty());
    CHECK(std::get<std::string>(g->current_value("gearbox_install")) == "stock"); // the weak box was dropped
    CHECK_THAT(engine.notice, ContainsSubstring("Gearbox"));
    g->cleanup();
    fs::remove_all(user, ec);
    fs::remove_all(work, ec);
}

TEST_CASE("gearbox swap: the auto-shift points leave room for every box's largest gear step (no shift hunting)", "[gearbox_swap][tcu]") {
    Fixture f;
    // After an upshift at `up` rpm the engine falls to up / step; it must stay above the downshift point, so
    // step < up / down at zero and full throttle, for the car's base points and every drive mode's overrides.
    int checked = 0;
    for (const rg::CatalogEntry* car : f.base_cars()) {
        const double peak = engine_peak_of(f, *car);
        for (const rg::GearboxEntry& box : f.boxes->list()) {
            if (!rg::check_gearbox_rating(box, peak).empty()) continue;
            INFO(car->id << " + " << box.id);
            const Loaded l = load_setup(f, *car, setup_of(car->id, {{"gearbox_install", box.id}}));
            REQUIRE(l.ok);
            const auto* gb = gearbox_of(l.desc);
            const auto* tcu = tcu_of(l.desc);
            REQUIRE(gb != nullptr);
            REQUIRE(tcu != nullptr);
            const double step = max_step(*gb);
            const auto& a = tcu->auto_shift;
            CHECK(step < a.up_rpm_at_zero_throttle / a.down_rpm_at_zero_throttle);
            CHECK(step < a.up_rpm_at_full_throttle / a.down_rpm_at_full_throttle);
            if (l.desc.drive_modes) {
                const auto& dm = *l.desc.drive_modes;
                for (int m = 0; m < ps::vehicle::kDriveModeCount; ++m) {
                    if ((dm.modes_bits & (1u << m)) == 0) continue;
                    const auto& p = dm.params[static_cast<std::size_t>(m)];
                    const double uz = p.up_rpm_at_zero_throttle.value_or(a.up_rpm_at_zero_throttle);
                    const double uf = p.up_rpm_at_full_throttle.value_or(a.up_rpm_at_full_throttle);
                    const double dz = p.down_rpm_at_zero_throttle.value_or(a.down_rpm_at_zero_throttle);
                    const double df = p.down_rpm_at_full_throttle.value_or(a.down_rpm_at_full_throttle);
                    INFO("drive mode " << ps::vehicle::kDriveModeNames[m]);
                    CHECK(step < uz / dz);
                    CHECK(step < uf / df);
                }
            }
            // the point must also lie inside the engine's range at the top of every gear's use: the highest upshift
            // point cannot exceed the limiter, or the car would never upshift before the cut
            ++checked;
        }
    }
    CHECK(checked >= 20);
}

namespace {
// A short WOT drive on a flat tessellated ground with the assists on; returns the grind-event count, the top gear
// reached and the final speed.
struct Drive {
    unsigned grinds = 0;
    int top_gear = 0;
    double kmh = 0.0;
    bool started = false;
};
Drive drive_with(const Fixture& f, const rg::CatalogEntry& car, const rg::VehicleSetup& setup, double seconds) {
    rg::ScopedWorkDir dir(f.ctx.work_root, "drive_" + car.id);
    const rg::MaterialisedSetup m = rg::materialise_setup(car, f.table, f.ctx, setup, dir.path());
    REQUIRE(m.ok);
    rg::SessionConfig config;
    config.vehicle_json_path = m.vehicle_path;
    config.surface_table_path = kRoot + "/data/surfaces/surfaces.json";
    config.chassis_mass_kg = car.chassis.mass_kg;
    config.chassis_half_extents = ps::Vec3{car.chassis.half_extents[0], car.chassis.half_extents[1], car.chassis.half_extents[2]};
    config.chassis_z_m = car.chassis.spawn_z_m;
    config.ground_half_extent_m = 2000.0;
    config.ground_half_extent_y_m = 100.0;
    config.ground_tessellated = true;
    config.engine_map_cache_dir = f.ctx.engine_map_cache_dir;
    rg::Session session(config);
    auto& world = session.world();
    const auto vid = session.vehicle_id();
    for (int i = 0; i < 240; ++i) session.step();
    world.set_control("ignition", 1.0);
    world.set_control("starter", 1.0);
    world.set_control("assist.auto_shift", 1.0);
    world.set_control("assist.auto_clutch", 1.0);
    world.set_control("assist.auto_blip", 1.0);
    world.set_control("clutch", 0.0);
    world.set_control("brake", 0.0);
    world.set_control("handbrake", 0.0);
    Drive out;
    const long steps = static_cast<long>(seconds * 240.0);
    for (long i = 0; i < steps; ++i) {
        if (!out.started && world.powertrain_state(vid).engine_state == ps::drivetrain::EngineState::Running) {
            world.set_control("starter", 0.0);
            out.started = true;
        }
        if (!out.started && i > 240 * 6) {
            world.set_control("starter", 0.0);
            out.started = true;
        }
        if (out.started) world.set_control("throttle", 1.0);
        session.step();
        const auto pt = world.powertrain_state(vid);
        out.top_gear = std::max(out.top_gear, static_cast<int>(pt.gear));
        out.grinds = std::max(out.grinds, pt.driveline.grind_events);
    }
    const ps::Motion mo = world.get_motion(session.chassis_body());
    out.kmh = std::sqrt(mo.linear.x * mo.linear.x + mo.linear.y * mo.linear.y) * 3.6;
    return out;
}
} // namespace

TEST_CASE("gearbox swap: the sedan shifts up through every box on auto-shift without grinding", "[gearbox_swap][drive][long]") {
    Fixture f;
    const auto& car = f.entry("car_sedan_rwd");
    for (const rg::GearboxEntry& box : f.boxes->list()) {
        INFO(box.id);
        const Drive d = drive_with(f, car, setup_of(car.id, {{"gearbox_install", box.id}}), 16.0);
        CHECK(d.started);
        CHECK(d.grinds == 0u);
        CHECK(d.kmh > 60.0);
        CHECK(d.top_gear >= 3);
    }
}

TEST_CASE("gearbox swap: the hypercar with the straight-cut box accelerates through its gears without grinding", "[gearbox_swap][drive][long]") {
    Fixture f;
    const auto& car = f.entry("car_hyper");
    const Drive d = drive_with(f, car, setup_of(car.id, {{"gearbox_install", std::string("straight_cut_6sq")}}), 16.0);
    CHECK(d.started);
    CHECK(d.grinds == 0u);
    CHECK(d.top_gear >= 4);
    CHECK(d.kmh > 150.0);
}
