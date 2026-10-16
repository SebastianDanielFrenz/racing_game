// test_automatic_gearbox_swap.cpp - automatic and dual-clutch gearboxes in the garage (automatic gearbox plan A1, owner
// request on racing_game #5): the catalogue rows, the derived automatic_tcu points, the swap package (clutch and manual
// TCU replaced by the physics automatic_tcu, park lock fitted, drive modes dropped), the garage dropping the options a
// clutch-to-clutch box no longer has, the selector helpers, and drives: a car with an automatic creeps at idle and drives
// from rest through several gears on the selector alone.
#include <cstdio>
#include "rg/driver_actions.h"
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

#include <algorithm>
#include <cmath>
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

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    std::shared_ptr<const rg::EngineStore> store;
    std::shared_ptr<const rg::GearboxCatalog> boxes;
    rg::ScopedWorkDir isolated_root{fs::temp_directory_path().generic_string(), "rg_autobox_unit_tests"};
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
};

rg::VehicleSetup setup_of(const std::string& vehicle, const std::string& box) {
    rg::VehicleSetup s;
    s.vehicle_id = vehicle;
    s.values["gearbox_install"] = box;
    return s;
}

struct Loaded {
    bool ok = false;
    std::string error;
    ps::vehicle::VehicleDesc desc;
    rg::CompiledSetup compiled;
};
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

template <class T> int count_components(const ps::vehicle::VehicleDesc& d) {
    int n = 0;
    for (const auto& c : d.powertrain.components) n += std::holds_alternative<T>(c.params) ? 1 : 0;
    return n;
}
const ps::drivetrain::AutomaticTcuDesc* auto_tcu_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.controllers) {
        if (const auto* g = std::get_if<ps::drivetrain::AutomaticTcuDesc>(&c.params)) return g;
    }
    return nullptr;
}
bool has_manual_tcu(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.controllers) {
        if (std::holds_alternative<ps::drivetrain::ManualTcuDesc>(c.params)) return true;
    }
    return false;
}
bool has_line(const std::vector<std::string>& lines, const std::string& part) {
    return std::any_of(lines.begin(), lines.end(), [&](const std::string& l) { return l.find(part) != std::string::npos; });
}
} // namespace

TEST_CASE("automatic gearbox: the catalogue rows and their sections", "[gearbox_swap][automatic]") {
    Fixture f;
    for (const char* id : {"auto_8at", "heavy_8at"}) {
        const rg::GearboxEntry* g = f.boxes->find(id);
        REQUIRE(g != nullptr);
        CHECK(g->kind == rg::GearboxKind::Automatic);
        CHECK(rg::is_clutch_to_clutch(g->kind));
        CHECK(g->gears == 8);
        CHECK(rg::gearbox_section(*g) == "Automatic");
        CHECK(g->synchro == "none");
    }
    for (const char* id : {"dct_7", "hyper_7dct"}) {
        const rg::GearboxEntry* g = f.boxes->find(id);
        REQUIRE(g != nullptr);
        CHECK(g->kind == rg::GearboxKind::DualClutch);
        CHECK(rg::is_clutch_to_clutch(g->kind));
        CHECK(g->gears == 7);
        CHECK(rg::gearbox_section(*g) == "Dual-clutch");
    }
    CHECK_FALSE(rg::is_clutch_to_clutch(rg::GearboxKind::Manual));
    CHECK_FALSE(rg::is_clutch_to_clutch(rg::GearboxKind::Sequential));
    CHECK(f.boxes->find("hyper_7dct")->rating_nm >= 1400.0); // takes the hypercar's 1387 N*m
}

TEST_CASE("automatic gearbox: the derived automatic_tcu points sit inside the engine's band", "[gearbox_swap][automatic]") {
    for (const auto& [idle, limiter] : std::vector<std::pair<double, double>>{{800, 6500}, {900, 8250}, {550, 2200}, {1000, 7000}}) {
        INFO(idle << " .. " << limiter);
        const rg::AutomaticTcuPoints p = rg::derive_automatic_tcu_points(idle, limiter);
        REQUIRE(p.ok);
        CHECK(p.down_zero > idle);
        CHECK(p.down_zero < p.up_zero);
        CHECK(p.down_full < p.up_full);
        CHECK(p.up_zero < p.up_full);
        CHECK(p.up_full < limiter);
        CHECK(p.sport_up_zero > p.up_zero);
        CHECK(p.sport_down_zero > p.down_zero);
        CHECK(p.sport_up_full <= limiter);
        CHECK(p.lockup_release_rpm < p.lockup_engage_rpm);
        CHECK(p.lockup_release_rpm > idle);
        CHECK(p.launch_bite_rpm > idle);
        CHECK(p.launch_full_rpm_min > p.launch_bite_rpm);
        CHECK(p.launch_full_rpm_max >= p.launch_full_rpm_min);
        CHECK(p.launch_anti_stall_rpm < idle);
        CHECK(p.kickdown_max_rpm < limiter);
    }
    CHECK(rg::derive_automatic_tcu_points(800, 6500).up_full == Approx(6100.0));
    CHECK_FALSE(rg::derive_automatic_tcu_points(800, 1500).ok); // no band to place a schedule in
}

TEST_CASE("automatic gearbox: installing one replaces the clutch and the manual TCU", "[gearbox_swap][automatic]") {
    Fixture f;
    for (const char* box : {"auto_8at", "dct_7"}) {
        INFO(box);
        const Loaded l = load_setup(f, f.entry("car_sedan"), setup_of("car_sedan", box));
        INFO(l.error);
        REQUIRE(l.ok);
        CHECK(count_components<ps::drivetrain::ClutchDesc>(l.desc) == 0);
        CHECK(count_components<ps::drivetrain::ParkLockDesc>(l.desc) == 1);
        CHECK_FALSE(has_manual_tcu(l.desc));
        const auto* tcu = auto_tcu_of(l.desc);
        REQUIRE(tcu != nullptr);
        const rg::AutomaticTcuPoints p = rg::derive_automatic_tcu_points(800.0, 7000.0);
        (void)p;
        CHECK(tcu->schedule.up_rpm_at_full_throttle > tcu->schedule.up_rpm_at_zero_throttle);
        CHECK(tcu->sport_schedule.up_rpm_at_zero_throttle > tcu->schedule.up_rpm_at_zero_throttle);
        CHECK(has_line(l.compiled.swap_lines, "Clutch removed"));
        CHECK(has_line(l.compiled.swap_lines, "Park lock fitted"));
        CHECK_FALSE(has_line(l.compiled.swap_lines, "Clutch assist:"));
        const auto* gb = [&]() -> const ps::drivetrain::GearboxDesc* {
            for (const auto& c : l.desc.powertrain.components) {
                if (const auto* g = std::get_if<ps::drivetrain::GearboxDesc>(&c.params)) return g;
            }
            return nullptr;
        }();
        REQUIRE(gb != nullptr);
        CHECK(gb->type == (std::string(box) == "auto_8at" ? ps::drivetrain::GearboxType::Automatic : ps::drivetrain::GearboxType::DualClutch));
    }
    // the hypercar takes the hypercar dual-clutch: its drive modes (which set manual shift points) go
    const Loaded h = load_setup(f, f.entry("car_hyper"), setup_of("car_hyper", "hyper_7dct"));
    INFO(h.error);
    REQUIRE(h.ok);
    CHECK(auto_tcu_of(h.desc) != nullptr);
    CHECK_FALSE(h.desc.drive_modes.has_value());
    CHECK(count_components<ps::drivetrain::ParkLockDesc>(h.desc) == 1); // its own park lock, not a second one
    // an automatic too weak for the engine is refused like a manual box
    const rg::CompiledSetup weak = rg::compile_setup(f.entry("car_hyper"), f.table, f.ctx, setup_of("car_hyper", "auto_8at"));
    CHECK_FALSE(weak.ok);
    CHECK_THAT(weak.error, ContainsSubstring("gearbox: rated 400 N*m"));
}

TEST_CASE("automatic gearbox: an engine swap with an automatic installs both", "[gearbox_swap][automatic]") {
    Fixture f;
    rg::VehicleSetup s = setup_of("car_sedan_rwd", "hyper_7dct");
    s.values["engine_install"] = std::string("default:hyper_v8_one1_audio");
    const Loaded l = load_setup(f, f.entry("car_sedan_rwd"), s);
    INFO(l.error);
    REQUIRE(l.ok);
    const auto* tcu = auto_tcu_of(l.desc);
    REQUIRE(tcu != nullptr);
    CHECK(count_components<ps::drivetrain::ClutchDesc>(l.desc) == 0);
    CHECK_FALSE(has_line(l.compiled.swap_lines, "Clutch capacity"));
    CHECK_FALSE(has_line(l.compiled.swap_lines, "Auto shift:"));
}

TEST_CASE("automatic gearbox: the garage drops the clutch and shift-assist options with the box", "[gearbox_swap][automatic][garage]") {
    const fs::path user = fs::temp_directory_path() / "rg_autobox_garage_user";
    const fs::path work = fs::temp_directory_path() / "rg_autobox_garage_work";
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
    REQUIRE(g->set_option("clutch_capacity", 1.2).accepted);
    REQUIRE(g->set_option("assist_auto_shift", false).accepted);
    REQUIRE(g->validation().ok);
    const auto r = g->set_option("gearbox_install", std::string("auto_8at"));
    INFO(r.message);
    REQUIRE(r.accepted);
    CHECK_THAT(r.notice, ContainsSubstring("The gearbox does not offer"));
    CHECK(g->working().values.count("clutch_capacity") == 0);
    CHECK(g->working().values.count("assist_auto_shift") == 0);
    INFO(g->validation().message);
    CHECK(g->validation().ok);
    // and the options are no longer offered for this setup
    for (const rg::OptionView& v : g->model().options) {
        if (v.def.id == "clutch_capacity" || v.def.id == "assist_auto_clutch") CHECK_FALSE(v.available);
    }
    g->cleanup();
    fs::remove_all(user, ec);
    fs::remove_all(work, ec);
}

TEST_CASE("automatic gearbox: the shift keys pick the selector from P/N/R and are paddles in D", "[automatic][driver_actions]") {
    rg::DriverSnapshot s;
    // a manual car: nothing changes, the counters are forwarded
    rg::SelectorShift m = rg::selector_shift(s, 1, 0);
    CHECK(m.gear_request == rg::kGearRequestNoChange);
    CHECK(m.forward_counts);
    CHECK(rg::selector_label(s).empty());
    s.transmission_kind = 1;
    s.park_fitted = true;
    s.park_engaged = true;
    CHECK(rg::selector_label(s) == "P");
    rg::SelectorShift up = rg::selector_shift(s, 1, 0);
    CHECK(up.gear_request == rg::kGearRequestDrive);
    CHECK(up.release_park);
    CHECK_FALSE(up.forward_counts);
    rg::SelectorShift down = rg::selector_shift(s, 0, 1);
    CHECK(down.gear_request == -1.0);
    s.park_engaged = false;
    CHECK(rg::selector_label(s) == "N");
    s.transmission_selector = -1;
    CHECK(rg::selector_label(s) == "R");
    CHECK(rg::selector_shift(s, 1, 0).gear_request == 0.0); // R -> N
    CHECK(rg::selector_shift(s, 0, 1).gear_request == rg::kGearRequestNoChange);
    s.speed_mps = 5.0; // rolling in R: the keys do nothing
    CHECK(rg::selector_shift(s, 1, 0).gear_request == rg::kGearRequestNoChange);
    s.transmission_selector = 99;
    s.gear = 3;
    CHECK(rg::selector_label(s) == "D");
    const rg::SelectorShift paddles = rg::selector_shift(s, 1, 0);
    CHECK(paddles.forward_counts);
    CHECK(paddles.gear_request == rg::kGearRequestNoChange);
    s.transmission_sport = true;
    CHECK(rg::selector_label(s) == "S");
    s.transmission_manual = true;
    CHECK(rg::selector_label(s) == "M3");
}

TEST_CASE("automatic gearbox: the selector actions write gear_request, park and the D/S mode", "[automatic][driver_actions]") {
    struct MapSink final : rg::ChannelSink {
        std::vector<std::pair<std::string, double>> v;
        double get(std::string_view c) const override {
            for (const auto& kv : v) if (kv.first == c) return kv.second;
            return 0.0;
        }
        void set(std::string_view c, double x) override {
            for (auto& kv : v) if (kv.first == c) { kv.second = x; return; }
            v.emplace_back(std::string(c), x);
        }
    };
    rg::DriverActions a;
    MapSink sink;
    rg::VehicleContext ctx;
    ctx.snapshot.transmission_kind = 2;
    ctx.snapshot.park_fitted = true;
    REQUIRE(a.submit({"select_drive"}));
    a.apply(ctx, sink);
    CHECK(sink.get("gear_request") == rg::kGearRequestDrive);
    REQUIRE(a.submit({"park"}));
    a.apply(ctx, sink);
    CHECK(sink.get("park_lock") == 1.0);
    CHECK(sink.get("gear_request") == 0.0);
    REQUIRE(a.submit({"select_reverse"}));
    a.apply(ctx, sink);
    CHECK(sink.get("gear_request") == -1.0);
    CHECK(sink.get("park_lock") == 0.0);
    REQUIRE(a.submit({"transmission_mode"}));
    a.apply(ctx, sink);
    CHECK(sink.get("transmission.mode") == 1.0);
    // a manual car ignores the selector actions
    MapSink manual;
    rg::VehicleContext mctx;
    REQUIRE(a.submit({"select_drive"}));
    a.apply(mctx, manual);
    CHECK(manual.get("gear_request") == 0.0);
}

namespace {
struct AutoDrive {
    bool started = false;
    int top_gear = 0;
    double creep_mps = 0.0;
    double end_kmh = 0.0;
    bool stalled = false;
};
// The car_sedan with an automatic on a flat tessellated ground: start the engine, select D, release the brake and creep
// with no throttle for creep_s, then full throttle for drive_s.
AutoDrive drive_auto(const Fixture& f, const std::string& box, double creep_s, double drive_s) {
    const rg::CatalogEntry& car = f.entry("car_sedan");
    rg::ScopedWorkDir dir(f.ctx.work_root, "drive_" + box);
    const rg::MaterialisedSetup m = rg::materialise_setup(car, f.table, f.ctx, setup_of(car.id, box), dir.path());
    INFO(m.error);
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
    world.set_control("brake", 1.0);
    for (int i = 0; i < 240; ++i) session.step();
    world.set_control("ignition", 1.0);
    world.set_control("starter", 1.0);
    AutoDrive out;
    for (int i = 0; i < 240 * 4 && !out.started; ++i) {
        session.step();
        if (world.powertrain_state(vid).engine_state == ps::drivetrain::EngineState::Running) out.started = true;
    }
    world.set_control("starter", 0.0);
    world.set_control("gear_request", 99.0); // D
    for (int i = 0; i < 120; ++i) session.step();
    world.set_control("brake", 0.0);
    world.set_control("handbrake", 0.0);
    world.set_control("throttle", 0.0);
    auto speed = [&]() {
        const ps::Motion mo = world.get_motion(session.chassis_body());
        return std::sqrt(mo.linear.x * mo.linear.x + mo.linear.y * mo.linear.y);
    };
    for (long i = 0; i < static_cast<long>(creep_s * 240.0); ++i) {
        session.step();
        if (world.powertrain_state(vid).engine_state != ps::drivetrain::EngineState::Running) out.stalled = true;
    }
    out.creep_mps = speed();
    world.set_control("throttle", 1.0);
    for (long i = 0; i < static_cast<long>(drive_s * 240.0); ++i) {
        session.step();
        const auto pt = world.powertrain_state(vid);
        out.top_gear = std::max(out.top_gear, static_cast<int>(pt.gear));
        if (pt.engine_state != ps::drivetrain::EngineState::Running) out.stalled = true;
    }
    out.end_kmh = speed() * 3.6;
    return out;
}
} // namespace

TEST_CASE("automatic gearbox: the sedan with an automatic creeps at idle and drives from rest through its gears", "[gearbox_swap][automatic][drive]") {
    Fixture f;
    for (const char* box : {"auto_8at", "dct_7"}) {
        INFO(box);
        const AutoDrive d = drive_auto(f, box, 6.0, 14.0);
        CHECK(d.started);
        CHECK_FALSE(d.stalled);
        CHECK(d.creep_mps > 0.5); // creep in D with the brake released and no throttle
        CHECK(d.creep_mps < 5.0);
        CHECK(d.top_gear >= 4);
        CHECK(d.end_kmh > 100.0);
        std::printf("RG_AUTO_DRIVE %s creep=%.3f m/s top_gear=%d end=%.1f km/h\n", box, d.creep_mps, d.top_gear,
                    d.end_kmh);
    }
}
