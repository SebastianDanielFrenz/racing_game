// test_garage.cpp - rg::Garage (selection, saved setups, edit session, drive
// hand-over, cleanup) and the garage set / camera (rg/garage_set.h).
#include "rg/garage.h"
#include "rg/garage_set.h"

#include "ps/io/vehicle_io.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
const std::string kRoot = RG_SOURCE_DIR;

struct TempDirs {
    std::string base;
    TempDirs() {
        base = (fs::temp_directory_path() /
                ("rg_garage_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count() % 1000000000)))
                   .generic_string();
        fs::create_directories(base + "/user");
        fs::create_directories(base + "/work");
    }
    ~TempDirs() {
        std::error_code ec;
        fs::remove_all(fs::path(base), ec);
    }
};

rg::GarageConfig config_for(const TempDirs& t) {
    rg::GarageConfig c;
    c.repo_root = kRoot;
    c.catalog_path = kRoot + "/data/vehicles/catalog.json";
    c.options_path = kRoot + "/data/vehicles/setup_options.json";
    c.set_path = kRoot + "/data/garage/garage_set.json";
    c.tyre_dirs = {kRoot + "/external/physics_sim/data/tyres", kRoot + "/data/tyres"};
    c.user_dir = t.base + "/user";
    c.work_root = t.base + "/work";
    c.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
    return c;
}

std::unique_ptr<rg::Garage> open_garage(const TempDirs& t) {
    std::string err;
    auto g = rg::Garage::open(config_for(t), &err);
    INFO(err);
    REQUIRE(g != nullptr);
    return g;
}
} // namespace

TEST_CASE("garage: selection persists across a restart", "[garage]") {
    TempDirs t;
    {
        auto g = open_garage(t);
        CHECK(g->selected_id() == g->catalog().default_id);
        std::string err;
        CHECK(g->select("car_sedan_awd", &err));
        CHECK(!g->select("nope", &err));
    }
    auto g2 = open_garage(t);
    CHECK(g2->selected_id() == "car_sedan_awd");
}

TEST_CASE("garage: an edit is validated, saved, persisted and applied to the drive", "[garage]") {
    TempDirs t;
    auto g = open_garage(t);
    std::string err;
    REQUIRE(g->begin_edit("car_sedan", &err));
    CHECK(g->validation().ok);
    CHECK(!g->dirty());

    const rg::EditResult r = g->set_option("final_drive", 1.12);
    CHECK(r.accepted);
    CHECK(r.validation.ok);
    CHECK(g->dirty());
    REQUIRE(g->save(&err));
    CHECK(!g->dirty());
    CHECK(g->has_saved_setup("car_sedan"));
    CHECK(fs::exists(fs::path(g->setup_path("car_sedan"))));
    g->discard();

    // a new Garage (restart) reads it back
    auto g2 = open_garage(t);
    const rg::VehicleSetup saved = g2->saved_setup("car_sedan");
    REQUIRE(saved.values.count("final_drive") == 1);
    CHECK(std::get<double>(saved.values.at("final_drive")) == Approx(1.12));

    // the drive hand-over names a materialised vehicle file carrying the change
    const rg::DriveSelection d = g2->prepare_drive("car_sedan");
    REQUIRE(d.ok);
    CHECK(d.modified);
    CHECK(d.vehicle_path.find("/work/drive_car_sedan_") != std::string::npos);
    const ps::vehicle::VehicleDesc driven = ps::io::load_vehicle_json(d.vehicle_path);
    const ps::vehicle::VehicleDesc stock = ps::io::load_vehicle_json(g2->catalog().find("car_sedan")->vehicle_path);
    double driven_ratio = 0.0, stock_ratio = 0.0;
    for (const auto& c : driven.powertrain.components) {
        if (const auto* diff = std::get_if<ps::drivetrain::DifferentialDesc>(&c.params)) driven_ratio = diff->ratio;
    }
    for (const auto& c : stock.powertrain.components) {
        if (const auto* diff = std::get_if<ps::drivetrain::DifferentialDesc>(&c.params)) stock_ratio = diff->ratio;
    }
    CHECK(driven_ratio == Approx(stock_ratio * 1.12));
    CHECK(d.sim_name == "car_sedan");
    CHECK(d.chassis.mass_kg == 1500.0);

    // an unmodified car drives from its original file, no copy
    const rg::DriveSelection plain = g2->prepare_drive("car_sedan_rwd");
    REQUIRE(plain.ok);
    CHECK(!plain.modified);
    CHECK(plain.vehicle_path == g2->catalog().find("car_sedan_rwd")->vehicle_path);
}

TEST_CASE("garage: an invalid change is reported with the loader's words and cannot be saved", "[garage]") {
    TempDirs t;
    auto g = open_garage(t);
    std::string err;
    REQUIRE(g->begin_edit("car_sedan", &err));
    const rg::EditResult r = g->set_option("final_drive", 1.5);
    CHECK(r.accepted); // stored so the slider shows it, but...
    CHECK(!r.validation.ok);
    CHECK_THAT(r.validation.message, ContainsSubstring("outside the allowed range"));
    CHECK(!g->save(&err));
    CHECK_THAT(err, ContainsSubstring("outside the allowed range"));
    CHECK(!g->has_saved_setup("car_sedan"));
    // fixing it makes it savable again
    CHECK(g->set_option("final_drive", 1.05).validation.ok);
    CHECK(g->save(&err));

    // a wrong type / unknown option is refused outright
    CHECK(!g->set_option("final_drive", true).accepted);
    CHECK(!g->set_option("nope", 1.0).accepted);
    // setting back to stock removes it; saving an empty setup removes the file
    CHECK(g->set_option("final_drive", 1.0).accepted);
    CHECK(g->working().values.empty());
    CHECK(g->save(&err));
    CHECK(!g->has_saved_setup("car_sedan"));
}

TEST_CASE("garage: setups are kept per vehicle", "[garage]") {
    TempDirs t;
    auto g = open_garage(t);
    std::string err;
    REQUIRE(g->begin_edit("car_sedan", &err));
    REQUIRE(g->set_option("spring_front", 0.8).accepted);
    REQUIRE(g->save(&err));
    REQUIRE(g->begin_edit("car_sedan_rwd", &err));
    CHECK(g->working().values.empty());
    REQUIRE(g->set_option("paint", std::string("#aa3300")).accepted);
    REQUIRE(g->save(&err));
    CHECK(g->working_paint() == "#aa3300");
    g->discard();
    CHECK(g->has_saved_setup("car_sedan"));
    CHECK(g->has_saved_setup("car_sedan_rwd"));
    CHECK(!g->has_saved_setup("car_sedan_awd"));
    // colour-only setup: the drive keeps the original file but carries the colour
    const rg::DriveSelection d = g->prepare_drive("car_sedan_rwd");
    REQUIRE(d.ok);
    CHECK(d.paint == "#aa3300");
    CHECK(d.vehicle_path == g->catalog().find("car_sedan_rwd")->vehicle_path);
}

TEST_CASE("garage: assist defaults of the driven file follow the setup", "[garage]") {
    TempDirs t;
    auto g = open_garage(t);
    std::string err;
    const rg::DriveSelection stock = g->prepare_drive("car_sedan");
    REQUIRE(stock.ok);
    CHECK(stock.assist_auto_clutch);
    CHECK(stock.assist_auto_blip);
    CHECK(!stock.assist_auto_shift);
    CHECK(stock.manual_gearbox);
    REQUIRE(g->begin_edit("car_sedan", &err));
    REQUIRE(g->set_option("assist_auto_shift", true).accepted);
    REQUIRE(g->set_option("assist_auto_blip", false).accepted);
    REQUIRE(g->save(&err));
    const rg::DriveSelection d = g->prepare_drive("car_sedan");
    CHECK(d.assist_auto_shift);
    CHECK(!d.assist_auto_blip);
    CHECK(d.assist_auto_clutch);
}

TEST_CASE("garage: no temp files are left behind", "[garage]") {
    TempDirs t;
    {
        auto g = open_garage(t);
        std::string err;
        REQUIRE(g->begin_edit("car_sedan", &err));
        REQUIRE(g->set_option("final_drive", 1.1).accepted);
        REQUIRE(g->set_option("spring_rear", 1.2).accepted);
        // validation scratch directories are gone after every change
        CHECK(rg::ScopedWorkDir::file_count(t.base + "/work") == 0);
        REQUIRE(g->save(&err));
        const rg::DriveSelection d = g->prepare_drive("car_sedan");
        REQUIRE(d.ok);
        CHECK(rg::ScopedWorkDir::file_count(t.base + "/work") > 0); // the drive materialisation, while it is in use
        // a second prepare replaces it (no pile-up)
        const std::size_t before = rg::ScopedWorkDir::file_count(t.base + "/work");
        (void)g->prepare_drive("car_sedan");
        CHECK(rg::ScopedWorkDir::file_count(t.base + "/work") == before);
    }
    CHECK(rg::ScopedWorkDir::file_count(t.base + "/work") == 0); // destructor cleaned up

    // a stale drive directory of a crashed run is swept at open
    fs::create_directories(t.base + "/work/drive_stale_x");
    { std::ofstream(t.base + "/work/drive_stale_x/car.json") << "{}"; }
    CHECK(rg::ScopedWorkDir::file_count(t.base + "/work") == 1);
    auto g = open_garage(t);
    CHECK(rg::ScopedWorkDir::file_count(t.base + "/work") == 0);
}

TEST_CASE("garage: a corrupt or foreign saved setup is ignored, never fatal", "[garage]") {
    TempDirs t;
    auto g = open_garage(t);
    fs::create_directories(t.base + "/user/garage/setups");
    { std::ofstream(g->setup_path("car_sedan")) << "not json"; }
    CHECK(g->saved_setup("car_sedan").values.empty());
    { std::ofstream(g->setup_path("car_sedan"), std::ios::trunc)
          << R"({"format":"racing_game.vehicle_setup/1","vehicle":"car_sedan","options":{"gone_option":1}})"; }
    CHECK(g->saved_setup("car_sedan").values.empty());
    const rg::DriveSelection d = g->prepare_drive("car_sedan");
    CHECK(d.ok);
    CHECK(!d.modified);
}

// ---- the set and its camera ---------------------------------------------------------

TEST_CASE("garage set: the shipped data loads", "[garage][garage_set]") {
    std::string err;
    const auto set = rg::load_garage_set(kRoot + "/data/garage/garage_set.json", &err);
    INFO(err);
    REQUIRE(set.has_value());
    CHECK(set->areas.size() >= 4);
    for (const char* id : {"overview", "wheels", "engine", "rear"}) CHECK(set->find_area(id) != nullptr);
    CHECK(set->lights.size() >= 3);
    CHECK(!set->softboxes.empty());
    CHECK(!rg::parse_garage_set("{}", "t", &err).has_value());
    CHECK(!rg::parse_garage_set(R"({"format":"rg.garage_set/1","cameras":{"areas":[{"id":"wheels","anchor":"nope"}]}})", "t", &err).has_value());
    CHECK(!rg::parse_garage_set(R"({"format":"rg.garage_set/1","cameras":{"areas":[{"id":"wheels"}]}})", "t", &err).has_value()); // no overview
}

TEST_CASE("garage camera: moves smoothly between areas and settles on each", "[garage][garage_set]") {
    std::string err;
    const auto set = rg::load_garage_set(kRoot + "/data/garage/garage_set.json", &err);
    REQUIRE(set.has_value());
    rg::GarageSubject subject;
    rg::GarageCamera cam(*set, subject);
    CHECK(cam.area() == "overview");
    CHECK(!cam.go_to("nope"));
    CHECK(cam.area() == "overview");

    for (const char* id : {"wheels", "engine", "rear", "overview"}) {
        const rg::GarageCameraPose start = cam.pose();
        REQUIRE(cam.go_to(id));
        CHECK(cam.in_transition());
        rg::GarageCameraPose prev = start;
        double max_step = 0.0;
        int frames = 0;
        while (!cam.settled() && frames < 600) {
            cam.update(1.0 / 60.0);
            const rg::GarageCameraPose p = cam.pose();
            max_step = std::max(max_step, std::sqrt(std::pow(p.position.x - prev.position.x, 2) + std::pow(p.position.y - prev.position.y, 2) +
                                                    std::pow(p.position.z - prev.position.z, 2)));
            prev = p;
            ++frames;
        }
        INFO(id);
        CHECK(cam.settled());
        CHECK(frames < 300);
        CHECK(max_step < 0.35); // no jump larger than 35 cm in one 60 Hz frame
        // settles exactly on the area's pose
        const rg::GarageAreaDef* a = set->find_area(id);
        const rg::GarageCameraPose want = rg::garage_area_pose(*a, subject, a->spin ? 0.0 : a->car_yaw_deg);
        CHECK(cam.pose().position.x == Approx(want.position.x).margin(1e-6));
        CHECK(cam.pose().fov_deg == Approx(a->fov_deg));
    }
}

TEST_CASE("garage camera: the turntable spins in overview and settles elsewhere", "[garage][garage_set]") {
    std::string err;
    const auto set = rg::load_garage_set(kRoot + "/data/garage/garage_set.json", &err);
    REQUIRE(set.has_value());
    rg::GarageCamera cam(*set, rg::GarageSubject{});
    const double y0 = cam.car_yaw_deg();
    for (int i = 0; i < 60; ++i) cam.update(1.0 / 60.0);
    CHECK(cam.car_yaw_deg() == Approx(y0 + set->turntable.spin_deg_s).margin(0.2));
    REQUIRE(cam.go_to("wheels"));
    for (int i = 0; i < 240; ++i) cam.update(1.0 / 60.0);
    CHECK(cam.settled());
    CHECK(std::abs(std::remainder(cam.car_yaw_deg(), 360.0)) < 0.5); // turned to the area's yaw (0)
    const double stopped = cam.car_yaw_deg();
    for (int i = 0; i < 60; ++i) cam.update(1.0 / 60.0);
    CHECK(cam.car_yaw_deg() == stopped);
}

TEST_CASE("garage camera: the engine bay view follows the engine position", "[garage][garage_set]") {
    std::string err;
    const auto set = rg::load_garage_set(kRoot + "/data/garage/garage_set.json", &err);
    REQUIRE(set.has_value());
    rg::GarageSubject front;
    front.engine_bay = "front";
    rg::GarageSubject mid = front;
    mid.engine_bay = "mid";
    const rg::GarageAreaDef* engine = set->find_area("engine");
    const rg::GVec3 a_front = rg::garage_anchor(*engine, front);
    const rg::GVec3 a_mid = rg::garage_anchor(*engine, mid);
    CHECK(a_front.x > a_mid.x); // front engine ahead of the mid engine
    const rg::GarageCameraPose p_front = rg::garage_area_pose(*engine, front, 0.0);
    const rg::GarageCameraPose p_mid = rg::garage_area_pose(*engine, mid, 0.0);
    CHECK(p_front.position.x > a_front.x); // looking at a front engine from ahead
    CHECK(p_mid.position.x < a_mid.x);     // looking at a mid engine from behind
}


TEST_CASE("garage engine install persists and reaches the prepared drive", "[engine_parts][garage]") {
    TempDirs t;
    std::string err;
    {
        auto g = open_garage(t);
        REQUIRE(g->begin_edit("car_sedan", &err));
        REQUIRE(g->set_option("engine_install", std::string("sedan_i4")).validation.ok);
        REQUIRE(g->set_option("engine_rev_limit", 0.9).validation.ok);
        REQUIRE(g->save(&err));
    }
    auto g = open_garage(t);
    REQUIRE(g->begin_edit("car_sedan", &err));
    REQUIRE(std::get<std::string>(g->current_value("engine_install")) == "sedan_i4");
    const auto drive = g->prepare_drive("car_sedan");
    INFO(drive.error);
    REQUIRE(drive.ok);
    const auto d = ps::io::load_vehicle_json(drive.vehicle_path);
    bool found = false;
    for (const auto& c : d.powertrain.components) {
        if (auto e = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&c.params)) {
            found = true;
            CHECK(e->limiter.rpm == Approx(6120.0));
        }
    }
    REQUIRE(found);
    g->reset_all();
    REQUIRE(g->validation().ok);
    REQUIRE(std::get<std::string>(g->current_value("engine_install")) == "stock");
}
