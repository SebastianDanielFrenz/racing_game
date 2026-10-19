// test_drivetrain_swap.cpp - the drivetrain swap (owner on PR #5, 2026-10-15): the catalogue
// (data/drivetrains/driveline_catalog.json), the generic driveline transform (rg/driveline_swap.h), every catalog car in
// every layout through the real physics loader, the setup round trip, the garage's edit rules and stats, and short drives
// on a flat Session showing the drive torque reaches exactly the wheels the layout names.
#include "rg/driveline_swap.h"
#include "rg/engine_store.h"
#include "rg/garage.h"
#include "rg/gearbox_catalog.h"
#include "rg/session.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

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
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
using DK = ps::drivetrain::DifferentialKind;
const std::string kRoot = RG_SOURCE_DIR;

std::string read_text(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    std::shared_ptr<const rg::DrivelineCatalog> lines;
    rg::ScopedWorkDir isolated_root{fs::temp_directory_path().generic_string(), "rg_drivetrain_unit_tests"};
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
        ctx.engines = std::shared_ptr<const rg::EngineStore>(std::move(s));
        auto g = rg::GearboxCatalog::open(kRoot + "/data/gearboxes/gearbox_catalog.json", kRoot, &err);
        INFO(err);
        REQUIRE(g != nullptr);
        ctx.gearboxes = std::shared_ptr<const rg::GearboxCatalog>(std::move(g));
        auto d = rg::DrivelineCatalog::open(kRoot + "/data/drivetrains/driveline_catalog.json", &err);
        INFO(err);
        REQUIRE(d != nullptr);
        lines = std::shared_ptr<const rg::DrivelineCatalog>(std::move(d));
        ctx.drivelines = lines;
    }
    const rg::CatalogEntry& entry(const std::string& id) const {
        const rg::CatalogEntry* e = catalog.find(id);
        REQUIRE(e != nullptr);
        return *e;
    }
    std::vector<const rg::CatalogEntry*> base_cars() const {
        std::vector<const rg::CatalogEntry*> out;
        for (const auto& e : catalog.entries) if (e.preset_of.empty() && e.id != "car_electric") out.push_back(&e); // ICE transmission packages; EV axle reductions are covered separately.
        return out;
    }
};

rg::VehicleSetup setup_of(const std::string& vehicle, std::initializer_list<std::pair<const char*, rg::SetupValue>> values) {
    rg::VehicleSetup s;
    s.vehicle_id = vehicle;
    for (const auto& kv : values) s.values[kv.first] = kv.second;
    return s;
}

struct Loaded {
    bool ok = false;
    std::string error;
    ps::vehicle::VehicleDesc desc;
    rg::DrivelineInfo line; // of the materialised vehicle file
};
// Materialise and load with the physics loader without generating engine maps (validate-only).
Loaded load_setup(const Fixture& f, const rg::CatalogEntry& e, const rg::VehicleSetup& s) {
    Loaded out;
    rg::ScopedWorkDir dir(f.ctx.work_root, "t_" + e.id);
    const rg::MaterialisedSetup m = rg::materialise_setup(e, f.table, f.ctx, s, dir.path());
    if (!m.ok) {
        out.error = m.error;
        return out;
    }
    out.line = rg::describe_driveline(read_text(m.vehicle_path));
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

std::map<std::string, ps::drivetrain::DifferentialDesc> diffs_of(const ps::vehicle::VehicleDesc& d) {
    std::map<std::string, ps::drivetrain::DifferentialDesc> out;
    for (const auto& c : d.powertrain.components) {
        if (const auto* g = std::get_if<ps::drivetrain::DifferentialDesc>(&c.params)) out[c.id] = *g;
    }
    return out;
}

// The wheels the loaded powertrain drives: those named by a "wheel:<name>" shaft port, as front/rear counts.
std::pair<int, int> driven_axles(const ps::vehicle::VehicleDesc& d) {
    int front = 0, rear = 0;
    for (const auto& s : d.powertrain.shafts) {
        for (const ps::drivetrain::PortRef& p : s.ports) {
            if (!p.is_wheel) continue;
            for (const auto& w : d.wheels) if (w.name == p.component) (w.is_front ? front : rear) += 1;
        }
    }
    return {front, rear};
}

const rg::OptionView* view_of(const rg::SetupModel& m, const std::string& id) {
    for (const rg::OptionView& v : m.options) if (v.def.id == id) return &v;
    return nullptr;
}

std::string layout_of(const rg::CatalogEntry& e) { return rg::describe_driveline(read_text(e.vehicle_path)).layout; }
} // namespace

TEST_CASE("driveline catalogue: the shipped layouts and differentials load", "[drivetrain_swap][garage]") {
    Fixture f;
    REQUIRE(f.lines->layouts().size() == 5);
    REQUIRE(f.lines->differentials().size() == 5);
    for (const char* id : {"fwd", "rwd", "awd_open", "awd_viscous", "awd_locked"}) CHECK(f.lines->find_layout(id) != nullptr);
    for (const char* kind : {"open", "clutch_lsd", "torsen", "viscous", "locked"}) CHECK(f.lines->differential_of_kind(kind) != nullptr);
    CHECK_FALSE(f.lines->find_layout("fwd")->centre.has_value());
    REQUIRE(f.lines->find_layout("awd_viscous")->centre.has_value());
    CHECK(f.lines->find_layout("awd_viscous")->centre->kind == "viscous");
    CHECK(f.lines->find_layout("awd_torsen") == nullptr); // physics_sim has no Torsen/LSD centre
    CHECK(f.lines->awd_layout_of_centre("viscous")->id == "awd_viscous");
}

TEST_CASE("driveline catalogue: a bad catalogue is rejected with the reason", "[drivetrain_swap][garage]") {
    const std::string centre = R"("centre_unit": {"ratio": 1, "efficiency": 1, "carrier_inertia_kgm2": 0.01, "carrier_axis_local": [1,0,0]})";
    const std::string diffs = R"("differentials": [{"id": "open", "name": "Open", "kind": "open"}])";
    auto parse = [](const std::string& text) {
        std::string err;
        auto c = rg::DrivelineCatalog::parse(text, "t.json", &err);
        return std::make_pair(c != nullptr, err);
    };
    CHECK(parse("{\"format\": \"rg.driveline_catalog/1\", " + centre + ", " + diffs + R"(, "layouts": [{"id": "fwd", "name": "F", "layout": "FWD"}]})").first);
    auto r = parse("{\"format\": \"rg.driveline_catalog/2\", " + centre + ", " + diffs + R"(, "layouts": [{"id": "fwd", "name": "F", "layout": "FWD"}]})");
    CHECK_FALSE(r.first);
    CHECK_THAT(r.second, ContainsSubstring("format"));
    r = parse("{\"format\": \"rg.driveline_catalog/1\", " + centre + ", " + diffs + R"(, "layouts": [{"id": "awd", "name": "A", "layout": "AWD"}]})");
    CHECK_FALSE(r.first);
    CHECK_THAT(r.second, ContainsSubstring("needs a \"centre\""));
    r = parse("{\"format\": \"rg.driveline_catalog/1\", " + centre +
              R"(, "differentials": [{"id": "t", "name": "T", "kind": "open", "torque_bias_ratio": 2}], "layouts": [{"id": "fwd", "name": "F", "layout": "FWD"}]})");
    CHECK_FALSE(r.first);
    CHECK_THAT(r.second, ContainsSubstring("does not belong to kind \"open\""));
    r = parse("{\"format\": \"rg.driveline_catalog/1\", " + centre +
              R"(, "differentials": [{"id": "t", "name": "T", "kind": "torsen", "torque_bias_ratio": 0.5}], "layouts": [{"id": "fwd", "name": "F", "layout": "FWD"}]})");
    CHECK_FALSE(r.first);
    CHECK_THAT(r.second, ContainsSubstring("torque_bias_ratio > 1"));
    r = parse("{\"format\": \"rg.driveline_catalog/1\", " + centre + ", " + diffs +
              R"(, "layouts": [{"id": "awd", "name": "A", "layout": "AWD", "centre": {"kind": "torsen", "torque_bias_ratio": 3}}]})");
    CHECK_FALSE(r.first);
    CHECK_THAT(r.second, ContainsSubstring("centre differential must be open, viscous or locked"));
}

TEST_CASE("driveline transform: reads every catalog car and refuses a driveline it does not understand", "[drivetrain_swap]") {
    Fixture f;
    for (const rg::CatalogEntry* e : f.base_cars()) {
        INFO(e->id);
        const rg::DrivelineInfo info = rg::describe_driveline(read_text(e->vehicle_path));
        CHECK(info.swappable);
        CHECK(info.wheel_count == 4);
        CHECK(info.layout != "other");
    }
    const rg::DrivelineInfo hyper = rg::describe_driveline(read_text(f.entry("car_hyper").vehicle_path));
    CHECK(hyper.layout == "RWD");
    CHECK(hyper.rear_kind == "torsen");
    CHECK(hyper.front_kind.empty());
    CHECK(rg::driveline_summary(hyper) == "RWD · Torsen rear");
    // a transfer case is not a topology the swap reads
    nlohmann::json v = nlohmann::json::parse(read_text(f.entry("car_sedan_rwd").vehicle_path));
    v["powertrain"]["components"].push_back({{"id", "tc"}, {"type", "transfer_case"}});
    const rg::DrivelineInfo tc = rg::describe_driveline(v.dump());
    CHECK_FALSE(tc.swappable);
    CHECK_THAT(tc.reason, ContainsSubstring("transfer_case"));
    std::string out;
    rg::DrivelineChoice choice;
    choice.layout_id = "awd_open";
    CHECK_THAT(rg::apply_driveline(v.dump(), *f.lines, choice, &out), ContainsSubstring("cannot be swapped"));
    // a differential for an axle that is not driven
    rg::DrivelineChoice rear;
    rear.rear_diff_id = "torsen";
    CHECK(rg::apply_driveline(read_text(f.entry("car_sedan").vehicle_path), *f.lines, rear, &out) == "the rear axle is not driven");
}

TEST_CASE("driveline transform: the hypercar keeps its park pawl and gearing in every layout", "[drivetrain_swap]") {
    Fixture f;
    const std::string text = read_text(f.entry("car_hyper").vehicle_path);
    const nlohmann::json stock = nlohmann::json::parse(text);
    double stock_ratio = 0.0;
    for (const auto& c : stock["powertrain"]["components"]) if (c.value("id", "") == "diff") stock_ratio = c.at("ratio").get<double>();
    REQUIRE(stock_ratio > 0.0);
    for (const rg::DriveLayoutEntry& l : f.lines->layouts()) {
        INFO(l.id);
        rg::DrivelineChoice choice;
        choice.layout_id = l.id;
        std::string out;
        REQUIRE(rg::apply_driveline(text, *f.lines, choice, &out).empty());
        const nlohmann::json v = nlohmann::json::parse(out);
        bool park = false;
        for (const auto& s : v["powertrain"]["shafts"]) {
            const auto ports = s["ports"].get<std::vector<std::string>>();
            if (std::find(ports.begin(), ports.end(), "gearbox.out") == ports.end()) continue;
            park = std::find(ports.begin(), ports.end(), "park.shaft") != ports.end();
            CHECK(std::find(ports.begin(), ports.end(), (l.layout == "AWD" ? "centre" : "diff") + std::string(".carrier")) != ports.end());
        }
        CHECK(park);
        for (const auto& c : v["powertrain"]["components"]) {
            if (c.value("type", "") != "differential") continue;
            const std::string id = c.value("id", "");
            if (id == "centre") {
                CHECK(c.at("ratio").get<double>() == 1.0);
                CHECK(c.value("kind", "open") == l.centre->kind);
            } else {
                CHECK(c.at("ratio").get<double>() == stock_ratio); // the axle unit is a copy of the car's own
                CHECK(c.value("kind", "") == "torsen");
            }
        }
        const rg::DrivelineInfo info = rg::describe_driveline(out);
        CHECK(info.layout == l.layout);
        CHECK(info.swappable);
        // applying the same layout again changes nothing
        std::string again;
        REQUIRE(rg::apply_driveline(out, *f.lines, choice, &again).empty());
        CHECK(nlohmann::json::parse(again) == v);
    }
}

TEST_CASE("drivetrain swap: every catalog car loads in every layout and with every differential", "[drivetrain_swap][garage]") {
    Fixture f;
    int loaded = 0;
    for (const rg::CatalogEntry* e : f.base_cars()) {
        for (const rg::DriveLayoutEntry& l : f.lines->layouts()) {
            INFO(e->id << " " << l.id);
            const Loaded r = load_setup(f, *e, setup_of(e->id, {{"drive_layout", l.id}}));
            INFO(r.error);
            REQUIRE(r.ok);
            ++loaded;
            CHECK(r.line.layout == l.layout);
            const auto [front, rear] = driven_axles(r.desc);
            CHECK(front == (l.layout == "RWD" ? 0 : 2));
            CHECK(rear == (l.layout == "FWD" ? 0 : 2));
            const auto diffs = diffs_of(r.desc);
            CHECK(diffs.size() == (l.layout == "AWD" ? 3u : 1u));
            if (l.layout == "AWD") {
                REQUIRE(diffs.count("centre") == 1);
                const DK want = l.centre->kind == "open" ? DK::Open : l.centre->kind == "viscous" ? DK::Viscous : DK::Locked;
                CHECK(diffs.at("centre").kind == want);
                if (want != DK::Open) CHECK(diffs.at("centre").efficiency == 1.0); // physics_sim: a non-open centre is lossless
                CHECK(diffs.at("front").ratio == Approx(diffs.at("rear").ratio));
            }
        }
        // each differential on the axle(s) the car drives as delivered
        const std::string layout = layout_of(*e);
        for (const rg::DiffUnit& u : f.lines->differentials()) {
            rg::VehicleSetup s = setup_of(e->id, {});
            if (layout != "RWD") s.values["diff_front"] = u.id;
            if (layout != "FWD") s.values["diff_rear"] = u.id;
            INFO(e->id << " " << u.id);
            const Loaded r = load_setup(f, *e, s);
            INFO(r.error);
            REQUIRE(r.ok);
            ++loaded;
            if (layout != "RWD") CHECK(r.line.front_kind == u.kind);
            if (layout != "FWD") CHECK(r.line.rear_kind == u.kind);
        }
    }
    CHECK(loaded >= 12 * 10);
}

TEST_CASE("drivetrain swap: a swapped driveline round-trips through the saved setup", "[drivetrain_swap][garage]") {
    Fixture f;
    const auto& car = f.entry("car_sedan"); // FWD, open
    const rg::VehicleSetup s = setup_of("car_sedan", {{"drive_layout", std::string("awd_viscous")},
                                                      {"diff_front", std::string("clutch_lsd")},
                                                      {"diff_rear", std::string("torsen")},
                                                      {"final_drive", 1.1}});
    std::string err;
    const auto parsed = rg::parse_setup(rg::setup_to_json(s), f.table, "test", &err);
    INFO(err);
    REQUIRE(parsed.has_value());
    CHECK(std::get<std::string>(parsed->values.at("drive_layout")) == "awd_viscous");
    CHECK(std::get<std::string>(parsed->values.at("diff_rear")) == "torsen");
    const Loaded r = load_setup(f, car, *parsed);
    INFO(r.error);
    REQUIRE(r.ok);
    const auto diffs = diffs_of(r.desc);
    REQUIRE(diffs.size() == 3);
    CHECK(diffs.at("centre").kind == DK::Viscous);
    CHECK(diffs.at("centre").viscous_nm_s_rad == Approx(40.0));
    CHECK(diffs.at("front").kind == DK::ClutchLsd);
    CHECK(diffs.at("front").preload_nm == Approx(60.0));
    CHECK(diffs.at("front").drive_factor == Approx(0.35));
    CHECK(diffs.at("rear").kind == DK::Torsen);
    CHECK(diffs.at("rear").torque_bias_ratio == Approx(2.5));
    // the final drive scale reaches both new axle units, the centre stays 1:1
    const Loaded stock = load_setup(f, car, setup_of("car_sedan", {}));
    REQUIRE(stock.ok);
    const double stock_ratio = diffs_of(stock.desc).at("diff").ratio;
    CHECK(diffs.at("front").ratio == Approx(stock_ratio * 1.1));
    CHECK(diffs.at("rear").ratio == Approx(stock_ratio * 1.1));
    CHECK(diffs.at("centre").ratio == Approx(1.0));
    // a rear differential on the car as delivered (FWD) is refused in plain words
    const Loaded bad = load_setup(f, car, setup_of("car_sedan", {{"diff_rear", std::string("torsen")}}));
    CHECK_FALSE(bad.ok);
    CHECK_THAT(bad.error, ContainsSubstring("rear axle is not driven"));
    // the car's own layout chosen explicitly changes nothing
    const rg::CompiledSetup same = rg::compile_setup(car, f.table, f.ctx, setup_of("car_sedan", {{"drive_layout", std::string("fwd")}}));
    INFO(same.error);
    REQUIRE(same.ok);
    CHECK((same.vehicle_patch.empty() || same.vehicle_patch == "{}"));
}

TEST_CASE("drivetrain swap: the option views list what fits and the model reports the new layout", "[drivetrain_swap][garage]") {
    Fixture f;
    const auto& car = f.entry("car_sedan");
    const rg::SetupModel m = rg::build_setup_model(car, f.table, f.ctx);
    REQUIRE(m.error.empty());
    CHECK(m.driveline.layout == "FWD");
    const rg::OptionView* layout = view_of(m, "drive_layout");
    REQUIRE(layout != nullptr);
    CHECK(layout->available);
    REQUIRE(layout->parts.size() == 5); // stock (= fwd) + 4 others
    CHECK(layout->parts.front().id == "stock");
    CHECK(layout->parts.front().section == "FWD");
    CHECK(std::find(layout->choices.begin(), layout->choices.end(), "fwd") == layout->choices.end());
    const rg::OptionView* rear = view_of(m, "diff_rear");
    REQUIRE(rear != nullptr);
    CHECK_FALSE(rear->available);
    const rg::OptionView* front = view_of(m, "diff_front");
    REQUIRE(front != nullptr);
    CHECK(front->available);
    CHECK(front->parts.front().label == "Open differential"); // the car's own kind is the stock card
    CHECK(std::find(front->choices.begin(), front->choices.end(), "open") == front->choices.end());

    const rg::VehicleSetup awd = setup_of("car_sedan", {{"drive_layout", std::string("awd_viscous")}});
    const rg::SetupModel a = rg::build_setup_model(car, f.table, f.ctx, &awd);
    REQUIRE(a.error.empty());
    CHECK(a.driveline.layout == "AWD");
    CHECK(a.driveline.driven_wheels == 4);
    CHECK(a.driveline.centre_kind == "viscous");
    CHECK(rg::driveline_summary(a.driveline) == "AWD · Viscous centre · Open front · Open rear");
    REQUIRE(view_of(a, "diff_rear") != nullptr);
    CHECK(view_of(a, "diff_rear")->available);
}

TEST_CASE("drivetrain swap: garage edits drop a differential of an axle the new layout does not drive; stats follow", "[drivetrain_swap][garage]") {
    const fs::path user = fs::temp_directory_path() / "rg_drivetrain_garage_user";
    const fs::path work = fs::temp_directory_path() / "rg_drivetrain_garage_work";
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
    auto st = g->stats("car_sedan_awd", &err);
    REQUIRE(st.has_value());
    CHECK(st->layout == "AWD");
    CHECK(st->driveline == "AWD · Open centre · Open front · Open rear");

    REQUIRE(g->begin_edit("car_sedan_awd", &err));
    REQUIRE(g->set_option("diff_rear", std::string("torsen")).accepted);
    REQUIRE(g->set_option("diff_front", std::string("viscous")).accepted);
    CHECK(g->validation().ok);
    const auto r = g->set_option("drive_layout", std::string("fwd"));
    INFO(r.message);
    REQUIRE(r.accepted);
    CHECK_THAT(r.notice, ContainsSubstring("Rear differential"));
    CHECK(g->working().values.count("diff_rear") == 0);
    CHECK(std::get<std::string>(g->current_value("diff_front")) == "viscous"); // the front axle is still driven
    INFO(g->validation().message);
    CHECK(g->validation().ok);
    // while edited, the stats follow the working copy
    st = g->stats("car_sedan_awd", &err);
    REQUIRE(st.has_value());
    CHECK(st->layout == "FWD");
    CHECK(st->driven_wheels == 2);
    CHECK(st->driveline == "FWD · Viscous front");
    REQUIRE(g->save(&err));
    g->discard();
    // saved: the stats and the browser show the swapped layout
    st = g->stats("car_sedan_awd", &err);
    REQUIRE(st.has_value());
    CHECK(st->layout == "FWD");
    const rg::BrowserCar* bc = g->browser().find("car_sedan_awd");
    REQUIRE(bc != nullptr);
    CHECK(bc->layout == "FWD");
    // a layout that already fits the chosen kind turns the choice into stock silently
    REQUIRE(g->begin_edit("car_sedan_rwd", &err));
    REQUIRE(g->set_option("diff_rear", std::string("torsen")).accepted);
    const auto rwd = g->set_option("drive_layout", std::string("awd_open"));
    REQUIRE(rwd.accepted);
    CHECK(rwd.notice.empty());
    CHECK(std::get<std::string>(g->current_value("diff_rear")) == "torsen");
    CHECK(g->validation().ok);
    g->discard();
    g->cleanup();
    fs::remove_all(user, ec);
    fs::remove_all(work, ec);
}

namespace {
// A short full-throttle run on a flat tessellated ground with the auto-shift assists on. Per wheel: the mean
// longitudinal tyre force (N) over the second half of the run and whether the physics counts the wheel as driven.
struct DriveResult {
    bool started = false;
    double kmh = 0.0;
    std::vector<double> mean_fx;
    std::vector<bool> driven;
    std::vector<bool> front;
};
rg::SessionConfig session_config(const Fixture& f, const rg::CatalogEntry& car, const std::string& vehicle_path) {
    rg::SessionConfig config;
    config.vehicle_json_path = vehicle_path;
    config.surface_table_path = kRoot + "/data/surfaces/surfaces.json";
    config.chassis_mass_kg = car.chassis.mass_kg;
    config.chassis_half_extents = ps::Vec3{car.chassis.half_extents[0], car.chassis.half_extents[1], car.chassis.half_extents[2]};
    config.chassis_z_m = car.chassis.spawn_z_m;
    config.ground_half_extent_m = 2000.0;
    config.ground_half_extent_y_m = 100.0;
    config.ground_tessellated = true;
    config.engine_map_cache_dir = f.ctx.engine_map_cache_dir;
    return config;
}

DriveResult drive_with(const Fixture& f, const rg::CatalogEntry& car, const rg::VehicleSetup& setup, double seconds) {
    rg::ScopedWorkDir dir(f.ctx.work_root, "drive_" + car.id);
    const rg::MaterialisedSetup m = rg::materialise_setup(car, f.table, f.ctx, setup, dir.path());
    INFO(m.error);
    REQUIRE(m.ok);
    rg::SessionConfig config = session_config(f, car, m.vehicle_path);
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
    DriveResult out;
    const std::size_t n = world.vehicle_wheel_count(vid);
    out.mean_fx.assign(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        out.driven.push_back(world.wheel_is_driven(vid, i));
        out.front.push_back(session.vehicle_desc().wheels[i].is_front);
    }
    const long steps = static_cast<long>(seconds * 240.0);
    long sampled = 0;
    for (long i = 0; i < steps; ++i) {
        if (!out.started && (world.powertrain_state(vid).engine_state == ps::drivetrain::EngineState::Running || i > 240 * 6)) {
            world.set_control("starter", 0.0);
            out.started = true;
        }
        if (out.started) world.set_control("throttle", 1.0);
        session.step();
        if (i >= steps / 2) {
            for (std::size_t w = 0; w < n; ++w) out.mean_fx[w] += world.wheel_telemetry(vid, w).fx;
            ++sampled;
        }
    }
    for (double& v : out.mean_fx) v /= static_cast<double>(std::max(1L, sampled));
    const ps::Motion mo = world.get_motion(session.chassis_body());
    out.kmh = std::sqrt(mo.linear.x * mo.linear.x + mo.linear.y * mo.linear.y) * 3.6;
    return out;
}
} // namespace

TEST_CASE("drivetrain swap: on a flat road the drive torque reaches exactly the wheels each layout names", "[drivetrain_swap][drive]") {
    Fixture f;
    const auto& car = f.entry("car_sedan"); // FWD as delivered
    for (const rg::DriveLayoutEntry& l : f.lines->layouts()) {
        INFO(l.id);
        const DriveResult d = drive_with(f, car, setup_of(car.id, {{"drive_layout", l.id}}), 6.0);
        CHECK(d.started);
        CHECK(d.kmh > 30.0);
        REQUIRE(d.mean_fx.size() == 4);
        for (std::size_t w = 0; w < 4; ++w) {
            const bool want = l.layout == "AWD" || (l.layout == "FWD") == d.front[w];
            INFO("wheel " << w << (d.front[w] ? " front" : " rear") << " mean fx " << d.mean_fx[w] << " N");
            CHECK(d.driven[w] == want);
            if (want) {
                CHECK(d.mean_fx[w] > 300.0);   // pushes the car
            } else {
                CHECK(d.mean_fx[w] < 100.0);   // only rolls (rolling resistance pulls back)
                CHECK(d.mean_fx[w] > -400.0);
            }
        }
    }
}

TEST_CASE("drivetrain swap: the hypercar on all four wheels with an assist unit drives on auto-shift", "[drivetrain_swap][drive][long]") {
    Fixture f;
    const auto& car = f.entry("car_hyper");
    const DriveResult d = drive_with(f, car, setup_of(car.id, {{"drive_layout", std::string("awd_viscous")},
                                                               {"assist_unit", std::string("assists_racing")}}), 8.0);
    CHECK(d.started);
    CHECK(d.kmh > 80.0);
    for (std::size_t w = 0; w < d.mean_fx.size(); ++w) {
        INFO("wheel " << w << " mean fx " << d.mean_fx[w]);
        CHECK(d.driven[w]);
        CHECK(d.mean_fx[w] > 300.0);
    }
}

TEST_CASE("drivetrain swap: every catalog car builds a physics world in every layout", "[drivetrain_swap][drive]") {
    // The validate-only load does not run the powertrain builder's topology rules (a Torsen centre passes it and is
    // refused only when the World builds the vehicle), so every car x layout also builds a flat Session.
    Fixture f;
    for (const rg::CatalogEntry* e : f.base_cars()) {
        for (const rg::DriveLayoutEntry& l : f.lines->layouts()) {
            INFO(e->id << " " << l.id);
            rg::ScopedWorkDir dir(f.ctx.work_root, "world_" + e->id);
            const rg::MaterialisedSetup m = rg::materialise_setup(*e, f.table, f.ctx, setup_of(e->id, {{"drive_layout", l.id}}), dir.path());
            INFO(m.error);
            REQUIRE(m.ok);
            std::unique_ptr<rg::Session> session;
            std::string err;
            try {
                session = std::make_unique<rg::Session>(session_config(f, *e, m.vehicle_path));
            } catch (const std::exception& ex) {
                err = ex.what();
            }
            INFO(err);
            REQUIRE(session != nullptr);
            int driven = 0;
            for (std::size_t w = 0; w < session->world().vehicle_wheel_count(session->vehicle_id()); ++w) {
                driven += session->world().wheel_is_driven(session->vehicle_id(), w) ? 1 : 0;
            }
            CHECK(driven == (l.layout == "AWD" ? 4 : 2));
        }
    }
}

// ---- hybrid conversions (battery hybrid: the traction motor's reduction rides on the gearbox output shaft) ----------

namespace {
std::unique_ptr<rg::Garage> open_hybrid_garage(const char* tag) {
    const fs::path user = fs::temp_directory_path() / (std::string("rg_hybrid_swap_user_") + tag);
    const fs::path work = fs::temp_directory_path() / (std::string("rg_hybrid_swap_work_") + tag);
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
    return g;
}

// The hybrid conversion's motor reduction and the gearbox output share one shaft with the (single) differential carrier(s)
// the loaded desc has; true when the reduction's output port is on a shaft that also holds the root differential's carrier.
bool motor_rides_on_driveline(const ps::vehicle::VehicleDesc& d) {
    bool reduction = false, carrier = false;
    for (const auto& s : d.powertrain.shafts) {
        bool has_red = false, has_out = false;
        for (const ps::drivetrain::PortRef& p : s.ports) {
            if (p.component == "conversion_reduction" && p.port == "out") has_red = true;
            if (p.component != "conversion_reduction" && p.port == "carrier") carrier = true;
            if (p.port == "out" && p.component != "conversion_reduction") has_out = true;
        }
        if (has_red && has_out) reduction = true;
    }
    return reduction && carrier;
}
} // namespace

TEST_CASE("drivetrain swap: a hybrid conversion keeps its motor and takes every layout and differential", "[drivetrain_swap][hybrid]") {
    Fixture f;
    const auto& car = f.entry("car_hyper");
    const rg::VehicleSetup hybrid = setup_of("car_hyper", {{"powertrain_install", std::string("hybrid_battery")}});
    const rg::SetupModel m = rg::build_setup_model(car, f.table, f.ctx, &hybrid);
    REQUIRE(m.error.empty());
    CHECK(m.driveline.swappable);
    CHECK(m.driveline.layout == "RWD");
    const rg::OptionView* layout = view_of(m, "drive_layout");
    REQUIRE(layout != nullptr);
    CHECK(layout->available);
    CHECK(layout->unavailable_reason.empty());

    const std::pair<const char*, std::pair<int, int>> layouts[] = {{"fwd", {2, 0}}, {"rwd", {0, 2}}, {"awd_open", {2, 2}}, {"awd_viscous", {2, 2}}};
    for (const auto& [id, axles] : layouts) {
        INFO(id);
        const auto r = load_setup(f, car, setup_of("car_hyper", {{"powertrain_install", std::string("hybrid_battery")}, {"drive_layout", std::string(id)}}));
        INFO(r.error);
        REQUIRE(r.ok);
        CHECK(driven_axles(r.desc) == axles);
        CHECK(motor_rides_on_driveline(r.desc));
    }
    // a differential swap on the hybrid, alone and with a layout
    const auto torsen_front = load_setup(f, car, setup_of("car_hyper", {{"powertrain_install", std::string("hybrid_battery")},
                                                                         {"drive_layout", std::string("awd_open")},
                                                                         {"diff_front", std::string("torsen")}}));
    INFO(torsen_front.error);
    REQUIRE(torsen_front.ok);
    CHECK(torsen_front.line.front_kind == "torsen");
    const auto lsd_rear = load_setup(f, car, setup_of("car_hyper", {{"powertrain_install", std::string("hybrid_battery")},
                                                                     {"diff_rear", std::string("clutch_lsd")}}));
    INFO(lsd_rear.error);
    REQUIRE(lsd_rear.ok);
    CHECK(lsd_rear.line.rear_kind == "clutch_lsd");
    CHECK(motor_rides_on_driveline(lsd_rear.desc));

    // The fuel option follows the engine's own fuel list, not the driveline: the hybrid conversion neither adds nor removes it.
    // (Integration 2026-10-19: the Hyper One's engine lists E85 and E5 since knock v4, so the option is offered; before that it was
    // greyed out with "one fuel" while the engine listed E5 alone.)
    const rg::SetupModel plain_model = rg::build_setup_model(car, f.table, f.ctx);
    const rg::OptionView* plain_fuel = view_of(plain_model, "fuel");
    const rg::OptionView* hybrid_fuel = view_of(m, "fuel");
    REQUIRE(plain_fuel != nullptr);
    REQUIRE(hybrid_fuel != nullptr);
    CHECK(plain_fuel->available);
    CHECK(plain_fuel->unavailable_reason.empty());
    CHECK(hybrid_fuel->available == plain_fuel->available);
    CHECK(hybrid_fuel->unavailable_reason == plain_fuel->unavailable_reason);
    // a car without a hybrid is unchanged: same description, no reduction
    const rg::DrivelineInfo plain = rg::describe_driveline(read_text(car.vehicle_path));
    CHECK(plain.swappable);
    CHECK(rg::driveline_summary(plain) == "RWD · Torsen rear");
    // a pure electric conversion (its own reduction, no gearbox) still refuses, with the reason
    const rg::VehicleSetup ev_setup = setup_of("car_hyper", {{"powertrain_install", std::string("battery_ev")}});
    const rg::SetupModel ev = rg::build_setup_model(car, f.table, f.ctx, &ev_setup);
    if (ev.error.empty()) {
        const rg::OptionView* ev_layout = view_of(ev, "drive_layout");
        REQUIRE(ev_layout != nullptr);
        CHECK_FALSE(ev_layout->available);
        CHECK_FALSE(ev_layout->unavailable_reason.empty());
    }
}

TEST_CASE("drivetrain swap: swapping the layout before or after a hybrid conversion keeps it", "[drivetrain_swap][hybrid][garage]") {
    for (const bool layout_first : {true, false}) {
        INFO((layout_first ? "layout, then convert" : "convert, then layout"));
        auto g = open_hybrid_garage(layout_first ? "a" : "b");
        std::string err;
        REQUIRE(g->begin_edit("car_hyper", &err));
        if (layout_first) {
            REQUIRE(g->set_option("drive_layout", std::string("awd_open")).accepted);
            const auto r = g->set_option("powertrain_install", std::string("hybrid_battery"));
            INFO(r.message << " / " << r.notice);
            REQUIRE(r.accepted);
        } else {
            REQUIRE(g->set_option("powertrain_install", std::string("hybrid_battery")).accepted);
            const auto r = g->set_option("drive_layout", std::string("awd_open"));
            INFO(r.message << " / " << r.notice);
            REQUIRE(r.accepted);
        }
        REQUIRE(g->working().values.count("drive_layout") == 1);
        CHECK(std::get<std::string>(g->current_value("drive_layout")) == "awd_open");
        INFO(g->validation().message);
        CHECK(g->validation().ok);
        const auto st = g->stats("car_hyper", &err);
        REQUIRE(st.has_value());
        CHECK(st->layout == "AWD");
        CHECK(st->driven_wheels == 4);
        g->discard();
    }
    // a differential chosen before the conversion stays on its (still driven) axle
    auto g = open_hybrid_garage("c");
    std::string err;
    REQUIRE(g->begin_edit("car_hyper", &err));
    REQUIRE(g->set_option("diff_rear", std::string("clutch_lsd")).accepted);
    REQUIRE(g->set_option("powertrain_install", std::string("hybrid_battery")).accepted);
    CHECK(g->working().values.count("diff_rear") == 1);
    g->discard();
}
