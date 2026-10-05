// test_car_browser_data.cpp - R6c data side of the car browser: body types, displacement
// and presets in the vehicle catalog, preset overlays in rg::Garage, and the browser model
// the garage builds from the catalog. Each test names its sabotage ("sabotage:").
#include "rg/garage.h"
#include "rg/vehicle_catalog.h"

#include "ps/io/vehicle_io.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <filesystem>
#include <string>
#include <variant>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
const std::string kRoot = RG_SOURCE_DIR;

rg::VehicleCatalog load_catalog() {
    std::string err;
    auto catalog = rg::load_vehicle_catalog(kRoot + "/data/vehicles/catalog.json", kRoot, &err);
    INFO(err);
    REQUIRE(catalog.has_value());
    return *catalog;
}

struct TempDirs {
    std::string base;
    TempDirs() {
        base = (fs::temp_directory_path() /
                ("rg_browser_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count() % 1000000000)))
                   .generic_string();
        fs::create_directories(base + "/user");
        fs::create_directories(base + "/work");
    }
    ~TempDirs() {
        std::error_code ec;
        fs::remove_all(fs::path(base), ec);
    }
};

std::unique_ptr<rg::Garage> open_garage(const TempDirs& t) {
    rg::GarageConfig c;
    c.repo_root = kRoot;
    c.catalog_path = kRoot + "/data/vehicles/catalog.json";
    c.options_path = kRoot + "/data/vehicles/setup_options.json";
    c.set_path = kRoot + "/data/garage/garage_set.json";
    c.tyre_dirs = {kRoot + "/external/physics_sim/data/tyres", kRoot + "/data/tyres"};
    c.user_dir = t.base + "/user";
    c.work_root = t.base + "/work";
    c.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
    std::string err;
    auto g = rg::Garage::open(c, &err);
    INFO(err);
    REQUIRE(g != nullptr);
    return g;
}

double final_drive_of(const ps::vehicle::VehicleDesc& desc) {
    for (const auto& c : desc.powertrain.components) {
        if (const auto* diff = std::get_if<ps::drivetrain::DifferentialDesc>(&c.params)) return diff->ratio;
    }
    return 0.0;
}
} // namespace

TEST_CASE("catalog: body types and displacement are data", "[catalog][car_browser]") {
    const rg::VehicleCatalog c = load_catalog();
    std::string err;
    for (const auto& e : c.entries) {
        INFO(e.id);
        CHECK(!e.body_type.empty());
        CHECK(e.body_type != "other"); // every shipped car names its body type
    }
    CHECK(c.find("car_hyper")->body_type == "hypercar");
    CHECK(c.find("car_sedan")->body_type == "sedan");

    // computed from the engine file: the hypercar's simulated V8, 8 x pi/4 x 92^2 x 95.25 mm = 5.07 L
    const auto hyper = rg::compute_vehicle_stats(*c.find("car_hyper"), &err);
    REQUIRE(hyper.has_value());
    CHECK(hyper->displacement_known);
    CHECK(hyper->displacement_l > 5.0);
    CHECK(hyper->displacement_l < 5.15);
    CHECK_THAT(hyper->displacement_source, ContainsSubstring("engine file"));
    // a torque-map engine carries no geometry: the catalog's own figure with its source
    const auto sedan = rg::compute_vehicle_stats(*c.find("car_sedan"), &err);
    REQUIRE(sedan.has_value());
    CHECK(sedan->displacement_known);
    CHECK(sedan->displacement_l == 3.0);
    CHECK(!sedan->displacement_source.empty());
}

TEST_CASE("catalog: displacement from engine geometry", "[catalog][car_browser]") {
    // sabotage: bore^2 without pi/4, or a stroke in the wrong unit, fails the literal
    const auto v = rg::displacement_from_engine_json(R"({"cylinders": 8, "bore_mm": 92.0, "stroke_mm": 95.25})");
    REQUIRE(v.has_value());
    CHECK(*v == Approx(8.0 * 3.14159265358979 / 4.0 * 92.0 * 92.0 * 95.25 / 1.0e6).epsilon(1e-9));
    CHECK(!rg::displacement_from_engine_json(R"({"name": "torque map"})").has_value());
    CHECK(!rg::displacement_from_engine_json(R"({"cylinders": 4, "bore_mm": 0, "stroke_mm": 90})").has_value());
    CHECK(!rg::displacement_from_engine_json("not json").has_value());
}

TEST_CASE("catalog: a preset inherits its base and carries its own overlay", "[catalog][car_browser]") {
    // sabotage: a preset that does not copy its base has no vehicle file / model / chassis
    const rg::VehicleCatalog c = load_catalog();
    const auto* base = c.find("car_sedan");
    const auto* preset = c.find("car_sedan_sport");
    REQUIRE(base != nullptr);
    REQUIRE(preset != nullptr);
    CHECK(preset->preset_of == "car_sedan");
    CHECK(base->preset_of.empty());
    CHECK(preset->vehicle_path == base->vehicle_path);
    CHECK(preset->model_path == base->model_path);
    CHECK(preset->sim_name == base->sim_name);
    CHECK(preset->chassis.mass_kg == base->chassis.mass_kg);
    CHECK(preset->title != base->title);
    CHECK(preset->body_type == "sports");
    CHECK(base->preset_setup.empty());
    CHECK(preset->preset_setup.size() > 2);
    CHECK(std::holds_alternative<double>(preset->preset_setup.at("spring_front")));
    CHECK(std::holds_alternative<std::vector<double>>(preset->preset_setup.at("gear_ratios")));
    CHECK(c.entries.size() >= 12);
}

TEST_CASE("catalog: malformed presets are rejected with a reason", "[catalog][car_browser]") {
    std::string err;
    const std::string base =
        R"({"id":"b","title":"B","vehicle":"data/vehicles/car_sedan.json","model":"external/physics_sim/data/models/car_sedan/car_sedan.glb",
        "chassis":{"mass_kg":1000,"half_extents":[2,0.4,0.15],"spawn_z_m":0.6}})";
    const auto doc = [&](const std::string& preset) {
        return std::string(R"({"format":"rg.vehicle_catalog/1","vehicles":[)") + base + "," + preset + "]}";
    };
    CHECK(rg::parse_vehicle_catalog(doc(R"({"id":"p","title":"P","preset_of":"b","preset_setup":{"spring_front":1.1}})"), kRoot, "t", &err)
              .has_value());
    // unknown base
    CHECK(!rg::parse_vehicle_catalog(doc(R"({"id":"p","title":"P","preset_of":"zz","preset_setup":{"spring_front":1.1}})"), kRoot, "t", &err)
               .has_value());
    CHECK_THAT(err, ContainsSubstring("preset_of"));
    // a preset of a preset
    const std::string two = std::string(R"({"format":"rg.vehicle_catalog/1","vehicles":[)") + base +
                            R"(,{"id":"p","title":"P","preset_of":"b","preset_setup":{"spring_front":1.1}},
                              {"id":"q","title":"Q","preset_of":"p","preset_setup":{"spring_front":1.1}}]})";
    CHECK(!rg::parse_vehicle_catalog(two, kRoot, "t", &err).has_value());
    // an empty overlay
    CHECK(!rg::parse_vehicle_catalog(doc(R"({"id":"p","title":"P","preset_of":"b","preset_setup":{}})"), kRoot, "t", &err).has_value());
    // a preset must not redefine what it inherits
    CHECK(!rg::parse_vehicle_catalog(doc(R"({"id":"p","title":"P","preset_of":"b","vehicle":"x.json","preset_setup":{"spring_front":1.1}})"),
                                     kRoot, "t", &err)
               .has_value());
    // displacement_l needs its source
    const std::string no_source =
        R"({"format":"rg.vehicle_catalog/1","vehicles":[{"id":"b","title":"B","displacement_l":2.0,"vehicle":"data/vehicles/car_sedan.json",
        "model":"external/physics_sim/data/models/car_sedan/car_sedan.glb","chassis":{"mass_kg":1000,"half_extents":[2,0.4,0.15],"spawn_z_m":0.6}}]})";
    CHECK(!rg::parse_vehicle_catalog(no_source, kRoot, "t", &err).has_value());
    CHECK_THAT(err, ContainsSubstring("displacement"));
}

TEST_CASE("garage: a preset is a car whose setup starts from its overlay", "[garage][car_browser]") {
    // sabotage: effective_setup ignoring the overlay leaves the final drive at stock (the Approx check fails)
    TempDirs t;
    auto g = open_garage(t);
    std::string err;
    CHECK(g->preset_setup("car_sedan_sport").values.count("spring_front") == 1);
    CHECK(g->preset_setup("car_sedan").values.empty());

    const rg::DriveSelection d = g->prepare_drive("car_sedan_sport");
    REQUIRE(d.ok);
    CHECK(d.vehicle_id == "car_sedan_sport");
    CHECK(d.modified); // the overlay is applied although the player saved nothing
    CHECK(!g->has_saved_setup("car_sedan_sport"));

    const double stock = final_drive_of(ps::io::load_vehicle_json(g->catalog().find("car_sedan")->vehicle_path));
    CHECK(final_drive_of(ps::io::load_vehicle_json(d.vehicle_path)) == Approx(stock * 1.08));
    // a base car of the same family stays stock (this drive materialisation replaces the previous one)
    const rg::DriveSelection base = g->prepare_drive("car_sedan");
    REQUIRE(base.ok);
    CHECK(!base.modified);

    // editing: the working copy starts at the overlay; a change is saved on top; a reset returns to the overlay
    REQUIRE(g->begin_edit("car_sedan_sport", &err));
    CHECK(g->validation().ok);
    CHECK(!g->dirty());
    CHECK(std::get<double>(g->current_value("spring_front")) == Approx(1.2));
    REQUIRE(g->set_option("spring_front", 1.25).accepted);
    CHECK(g->dirty());
    REQUIRE(g->save(&err));
    CHECK(g->has_saved_setup("car_sedan_sport"));
    g->reset_option("spring_front");
    CHECK(std::get<double>(g->current_value("spring_front")) == Approx(1.2)); // the preset's value, not the stock 1.0
    g->reset_all();
    REQUIRE(g->save(&err));
    CHECK(!g->has_saved_setup("car_sedan_sport")); // equal to the overlay: no player file
    g->discard();
}

TEST_CASE("garage: every shipped preset validates with the physics loader", "[garage][car_browser]") {
    TempDirs t;
    auto g = open_garage(t);
    int presets = 0;
    for (const auto& e : g->catalog().entries) {
        if (e.preset_of.empty()) continue;
        ++presets;
        INFO(e.id);
        std::string err;
        REQUIRE(g->begin_edit(e.id, &err));
        INFO(g->validation().message);
        CHECK(g->validation().ok);
        g->discard();
    }
    CHECK(presets >= 8);
}

TEST_CASE("garage: the browser lists every catalog entry with stats and the effective paint", "[garage][car_browser]") {
    TempDirs t;
    auto g = open_garage(t);
    const rg::CarBrowser& b = g->browser();
    CHECK(b.cars().size() == g->catalog().entries.size());
    const rg::BrowserCar* hyper = b.find("car_hyper");
    const rg::BrowserCar* awd = b.find("car_sedan_awd");
    REQUIRE(hyper != nullptr);
    REQUIRE(awd != nullptr);
    CHECK(hyper->layout == "RWD");
    CHECK(awd->layout == "AWD");
    CHECK(hyper->power_kw > 900.0);
    CHECK(hyper->displacement_known);
    CHECK(!hyper->preset);
    CHECK(b.find("car_hyper_stealth")->preset);
    CHECK(b.find("car_hyper_stealth")->paint == "#1b1d22"); // the preset's paint
    CHECK(b.find("car_hyper")->paint == g->catalog().find("car_hyper")->default_paint);

    // a saved paint shows in the browser after the save, and only on that entry
    std::string err;
    REQUIRE(g->begin_edit("car_hyper", &err));
    REQUIRE(g->set_option("paint", std::string("#123456")).accepted);
    REQUIRE(g->save(&err));
    g->discard();
    CHECK(g->browser().find("car_hyper")->paint == "#123456");
    CHECK(g->browser().find("car_hyper_stealth")->paint == "#1b1d22");
}
