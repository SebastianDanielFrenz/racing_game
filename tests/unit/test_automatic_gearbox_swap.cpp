// test_automatic_gearbox_swap.cpp - automatic and dual-clutch gearboxes in the garage (automatic gearbox plan A1, owner
// request on racing_game #5): the catalogue rows, the derived automatic_tcu points, the swap package (clutch and manual
// TCU replaced by the physics automatic_tcu, park lock fitted, drive modes dropped), the garage dropping the options a
// clutch-to-clutch box no longer has, the selector helpers, and drives: a car with an automatic creeps at idle and drives
// from rest through several gears on the selector alone.
#include <fstream>
#include <nlohmann/json.hpp>
#include "rg/driver_actions.h"
#include "rg/driveline_swap.h"
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
    REQUIRE(h.desc.drive_modes.has_value());
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

TEST_CASE("automatic gearbox: the shift keys step the selector P-R-N-D", "[automatic][driver_actions]") {
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
    // up from P: R
    rg::SelectorShift st = rg::selector_shift(s, 1, 0);
    CHECK(st.gear_request == -1.0);
    CHECK(st.park_request == 0);
    CHECK_FALSE(st.forward_counts);
    // down from P: nothing
    CHECK(rg::selector_shift(s, 0, 1).gear_request == rg::kGearRequestNoChange);
    // two presses in a frame: P -> R -> N
    st = rg::selector_shift(s, 2, 0);
    CHECK(st.gear_request == 0.0);
    CHECK(st.park_request == 0);
    CHECK(rg::selector_shift(s, 3, 0).gear_request == rg::kGearRequestDrive);
    // R
    s.park_engaged = false;
    s.transmission_selector = -1;
    CHECK(rg::selector_label(s) == "R");
    CHECK(rg::selector_shift(s, 1, 0).gear_request == 0.0); // R -> N
    st = rg::selector_shift(s, 0, 1); // R -> P
    CHECK(st.gear_request == 0.0);
    CHECK(st.park_request == 1);
    // N
    s.transmission_selector = 0;
    CHECK(rg::selector_label(s) == "N");
    CHECK(rg::selector_shift(s, 1, 0).gear_request == rg::kGearRequestDrive);
    CHECK(rg::selector_shift(s, 0, 1).gear_request == -1.0);
    // D: down goes N, then (two presses) R, then P at a standstill
    s.transmission_selector = 99;
    s.gear = 3;
    CHECK(rg::selector_label(s) == "D");
    CHECK(rg::selector_shift(s, 0, 1).gear_request == 0.0);
    CHECK(rg::selector_shift(s, 0, 2).gear_request == -1.0);
    st = rg::selector_shift(s, 0, 3);
    CHECK(st.gear_request == 0.0);
    CHECK(st.park_request == 1);
    // D up: nothing, and the counters are not forwarded (no paddles without manual mode)
    st = rg::selector_shift(s, 1, 0);
    CHECK(st.gear_request == rg::kGearRequestNoChange);
    CHECK_FALSE(st.forward_counts);
    // rolling: D -> N is allowed, N -> R and R -> P are refused
    s.speed_mps = 5.0;
    st = rg::selector_shift(s, 0, 1);
    CHECK(st.gear_request == 0.0);
    CHECK_FALSE(st.refused);
    st = rg::selector_shift(s, 0, 2);
    CHECK(st.gear_request == 0.0); // stops at N
    CHECK(st.refused);
    s.transmission_selector = 0;
    st = rg::selector_shift(s, 0, 1);
    CHECK(st.gear_request == rg::kGearRequestNoChange);
    CHECK(st.refused);
    CHECK(rg::selector_shift(s, 1, 0).gear_request == rg::kGearRequestDrive); // N -> D while rolling is fine
    s.transmission_selector = -1;
    s.speed_mps = -3.0;
    st = rg::selector_shift(s, 0, 1);
    CHECK(st.refused);
    CHECK(st.park_request == -1);
    // a car without a park lock: R is the lowest position
    rg::DriverSnapshot np;
    np.transmission_kind = 1;
    np.transmission_selector = -1;
    CHECK(rg::selector_shift(np, 0, 1).gear_request == rg::kGearRequestNoChange);
    // manual mode in D: the counters go to physics
    s.speed_mps = 20.0;
    s.transmission_selector = 99;
    s.transmission_manual = true;
    const rg::SelectorShift paddles = rg::selector_shift(s, 1, 0);
    CHECK(paddles.forward_counts);
    CHECK(paddles.gear_request == rg::kGearRequestNoChange);
    CHECK(rg::selector_label(s) == "M3");
    s.transmission_manual = false;
    s.transmission_sport = true;
    CHECK(rg::selector_label(s) == "S");
}

TEST_CASE("automatic gearbox: Y toggles manual mode, the D/S action and park", "[automatic][driver_actions]") {
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
    // the removed selector actions no longer exist
    CHECK_FALSE(a.submit({"select_drive"}));
    // manual mode only from D
    ctx.snapshot.transmission_selector = 0;
    REQUIRE(a.submit({"manual_mode"}));
    a.apply(ctx, sink);
    CHECK(sink.get("shift_up_count") == 0.0);
    // from D: one up and one down count (physics enters manual mode, no gear change)
    ctx.snapshot.transmission_selector = 99;
    REQUIRE(a.submit({"manual_mode"}));
    a.apply(ctx, sink);
    CHECK(sink.get("shift_up_count") == 1.0);
    CHECK(sink.get("shift_down_count") == 1.0);
    // in manual mode the same key returns to D by changing gear_request
    ctx.snapshot.transmission_manual = true;
    sink.set("gear_request", rg::kGearRequestDrive);
    REQUIRE(a.submit({"manual_mode"}));
    a.apply(ctx, sink);
    CHECK(sink.get("gear_request") == rg::kGearRequestDrive - 1);
    CHECK(sink.get("shift_up_count") == 1.0);
    sink.set("gear_request", rg::kGearRequestDrive - 1);
    REQUIRE(a.submit({"manual_mode"}));
    a.apply(ctx, sink);
    CHECK(sink.get("gear_request") == rg::kGearRequestDrive);
    // D / S schedule and park keep working
    REQUIRE(a.submit({"transmission_mode"}));
    a.apply(ctx, sink);
    CHECK(sink.get("transmission.mode") == 1.0);
    REQUIRE(a.submit({"park"}));
    a.apply(ctx, sink);
    CHECK(sink.get("park_lock") == 1.0);
    CHECK(sink.get("gear_request") == 0.0);
    // a manual car ignores both
    MapSink manual;
    rg::VehicleContext mctx;
    mctx.snapshot.transmission_selector = 5;
    REQUIRE(a.submit({"manual_mode"}));
    a.apply(mctx, manual);
    CHECK(manual.get("shift_up_count") == 0.0);
    CHECK(manual.get("gear_request") == 0.0);
}

namespace {
struct AutoDrive {
    bool started = false;
    int top_gear = 0;
    int transmission_kind = -1;
    double max_cvt_ratio = 0.0, min_cvt_ratio = 1e9; // a cvt under throttle
    double max_rpm = 0.0;
    double creep_mps = 0.0;
    double end_kmh = 0.0, max_kmh = 0.0;
    bool stalled = false;
    int tc_interventions = 0;
    double max_false_cut_s = 0.0;
    double t50 = -1.0;
};
// The car_sedan with an automatic on a flat tessellated ground: start the engine, select D, release the brake and creep
// with no throttle for creep_s, then full throttle for drive_s.
AutoDrive drive_auto(const Fixture& f, const std::string& box, double creep_s, double drive_s,
                     const std::string& car_id = "car_sedan", int stop_cycles = 0, int corner_mode = -1, bool respawn_check = false, int launch_mode = -1, bool with_lc = false) {
    const rg::CatalogEntry& car = f.entry(car_id);
    rg::ScopedWorkDir dir(f.ctx.work_root, "drive_" + box);
    auto setup = setup_of(car.id, box);
    auto context = f.ctx;
    if (corner_mode >= 0 || launch_mode >= 0) {
        std::string err;
        auto drivelines = rg::DrivelineCatalog::open(kRoot + "/data/drivetrains/driveline_catalog.json", &err);
        INFO(err);
        REQUIRE(drivelines != nullptr);
        context.drivelines = std::shared_ptr<const rg::DrivelineCatalog>(std::move(drivelines));
        auto cooling = rg::load_cooling_upgrades(kRoot + "/data/cooling_systems/cooling_upgrades.json", &err);
        INFO(err);
        REQUIRE(cooling.has_value());
        context.cooling_upgrades = std::make_shared<const rg::CoolingUpgradeConfig>(*cooling);
        setup.values["drive_layout"] = std::string("awd_open");
        setup.values["final_drive"] = 0.8;
        setup.values["engine_rev_limit"] = 0.9939393939393939;
        setup.values["tyre_front"] = std::string("drag_305_30r20");
        setup.values["tyre_rear"] = std::string("drag_305_32r21");
        setup.values["cooling_oil_cooler"] = std::string("race");
        setup.values["cooling_radiator"] = std::string("race_ducted");
    }
    const rg::MaterialisedSetup m = rg::materialise_setup(car, f.table, context, setup, dir.path());
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
    if (corner_mode >= 0) world.set_control("assist.drive_mode", corner_mode);
    if (launch_mode >= 0) world.set_control("assist.drive_mode", launch_mode);
    world.set_control("gear_request", 99.0); // D
    for (int i = 0; i < 120; ++i) session.step();
    if (with_lc) {
        world.set_control("assist.launch_control", 1.0);
        world.set_control("throttle", 1.0);
        for (int i=0;i<720;++i) session.step();
        CHECK(world.powertrain_state(vid).launch_phase == 1);
    }
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
    double false_cut_s = 0.0;
    world.set_control("throttle", 1.0);
    for (long i = 0; i < static_cast<long>(drive_s * 240.0); ++i) {
        if (corner_mode >= 0) {
            const double target = i < 240 * 5 ? 6.0 : 2.78;
            world.set_control("throttle", std::clamp(0.02 + 0.20 * (target - speed()), 0.0, 0.30));
            world.set_control("brake", speed() > target + 0.3 ? 0.1 : 0.0);
            const double turn = i > 240 * 8 ? (i / (240 * 5) % 2 ? 0.9 : -0.9) : 0.0;
            world.set_control("steer", turn);
        }
        session.step();
        const auto pt = world.powertrain_state(vid);
        if (out.t50 < 0 && speed()*3.6 >= 50) out.t50=(i+1)/240.0;
        out.top_gear = std::max(out.top_gear, static_cast<int>(pt.gear));
        out.max_kmh = std::max(out.max_kmh, speed() * 3.6);
        if (pt.traction_torque_fraction < 0.95) ++out.tc_interventions;
        false_cut_s = pt.traction_torque_fraction < 0.95 && pt.traction_positive_slip < 0.02 ? false_cut_s + 1.0 / 240.0 : 0.0;
        out.max_false_cut_s = std::max(out.max_false_cut_s, false_cut_s);
        out.transmission_kind = static_cast<int>(pt.transmission_kind);
        out.max_rpm = std::max(out.max_rpm, static_cast<double>(pt.engine_rpm));
        if (pt.transmission_kind == 3) {
            out.max_cvt_ratio = std::max(out.max_cvt_ratio, static_cast<double>(pt.cvt_ratio));
            out.min_cvt_ratio = std::min(out.min_cvt_ratio, static_cast<double>(pt.cvt_ratio));
        }
        if (pt.engine_state != ps::drivetrain::EngineState::Running) {
            if (corner_mode >= 0 && !out.stalled) {
                std::printf("RG_CORNER_STALL mode=%d t=%.3f speed=%.2f rpm=%.1f gear=%d "
                            "clutch=%.4f steer=%.2f throttle=%.4f\n",
                            corner_mode, i / 240.0, speed() * 3.6, double(pt.engine_rpm), int(pt.gear),
                            double(pt.clutch_engagement), world.get_control("steer"), world.get_control("throttle"));
            }
            out.stalled = true;
        }
    }
    if (respawn_check) {
        world.set_control("throttle", 0.0);
        world.set_control("brake", 1.0);
        session.request_reset_to_spawn();
        for (int tick = 0; tick < 240; ++tick) session.step();
        auto reset = world.powertrain_state(vid);
        INFO("reset rpm=" << reset.engine_rpm << " gear=" << int(reset.gear));
        CHECK(reset.engine_state == ps::drivetrain::EngineState::Running);
        CHECK(reset.engine_rpm > 500.0);
        world.set_control("ignition", 0.0);
        for (int tick = 0; tick < 480; ++tick) session.step();
        world.set_control("ignition", 1.0);
        world.set_control("starter", 1.0);
        for (int tick = 0; tick < 960 && world.powertrain_state(vid).engine_state != ps::drivetrain::EngineState::Running; ++tick) session.step();
        CHECK(world.powertrain_state(vid).engine_state == ps::drivetrain::EngineState::Running);
        world.set_control("starter", 0.0);
    }
    for (int cycle = 0; cycle < stop_cycles; ++cycle) {
        for (int phase = 0; phase < 2; ++phase) {
            world.set_control("throttle", phase == 0 ? 0.0 : 0.3);
            world.set_control("brake", phase == 0 ? 1.0 : 0.0);
            for (int i = 0; i < 240 * (phase == 0 ? 8 : 5); ++i) {
                session.step();
                const auto pt = world.powertrain_state(vid);
                if (pt.engine_state != ps::drivetrain::EngineState::Running) {
                    if (!out.stalled) {
                        std::printf("RG_DCT_STALL car=%s box=%s cycle=%d phase=%d t=%.3f rpm=%.1f "
                                    "gear=%d target=%d clutch=%.4f speed=%.3f\n",
                                    car_id.c_str(), box.c_str(), cycle, phase, i / 240.0, double(pt.engine_rpm),
                                    int(pt.gear), int(pt.gear_target), double(pt.clutch_engagement), speed());
                    }
                    out.stalled = true;
                }
            }
            INFO("stop/start cycle " << cycle << " phase " << phase << " in " << car_id);
            if (phase == 0) CHECK(speed() < 0.25);
            else CHECK(speed() > 1.0);
        }
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

// ---- CVT (physics_sim CVT plan C1) ----

TEST_CASE("cvt gearbox: the catalogue rows and the CVT section", "[gearbox_swap][automatic][cvt]") {
    Fixture f;
    for (const char* id : {"cvt_converter", "cvt_clutch"}) {
        INFO(id);
        const rg::GearboxEntry* g = f.boxes->find(id);
        REQUIRE(g != nullptr);
        CHECK(g->kind == rg::GearboxKind::Cvt);
        CHECK(rg::is_clutch_to_clutch(g->kind));
        CHECK(g->synchro == "none");
        CHECK(rg::gearbox_section(*g) == "CVT");
        CHECK(g->gears == 7);                      // the virtual steps of S
        CHECK(g->first_ratio == Approx(2.63));     // ratio_low
        CHECK(g->top_ratio == Approx(0.38));       // ratio_high
        CHECK(g->converter == (std::string(id) == "cvt_converter"));
        CHECK_THAT(rg::gearbox_detail_line(*g), ContainsSubstring("stepless 2.63 to 0.38"));
        CHECK_THAT(rg::gearbox_summary_line(*g), ContainsSubstring("CVT"));
    }
    bool cvt_listed = false;
    for (const rg::GearboxSection& sec : f.boxes->sections()) {
        if (sec.label == "CVT") cvt_listed = sec.has_entries;
    }
    CHECK(cvt_listed); // no longer "coming soon"
}

TEST_CASE("cvt gearbox: the derived ratio law rises with the throttle inside the engine's band", "[gearbox_swap][automatic][cvt]") {
    for (const auto& [idle, limiter] : std::vector<std::pair<double, double>>{{800, 6500}, {900, 8250}, {550, 2200}, {1000, 7000}}) {
        INFO(idle << " .. " << limiter);
        const rg::AutomaticTcuPoints p = rg::derive_automatic_tcu_points(idle, limiter);
        REQUIRE(p.ok);
        CHECK(p.cvt_target_rpm[0] > idle);
        for (int i = 0; i + 1 < 5; ++i) CHECK(p.cvt_target_rpm[i] < p.cvt_target_rpm[i + 1]);
        CHECK(p.cvt_target_rpm[4] < limiter);
        CHECK(p.cvt_min_rpm[0] > idle * 1.0 - 1.0);
        for (int i = 0; i + 1 < 3; ++i) CHECK(p.cvt_min_rpm[i] < p.cvt_min_rpm[i + 1]);
        CHECK(p.cvt_min_rpm[2] < p.cvt_target_rpm[4]);
    }
    // an idle 800 / limiter 6500 engine gets physics_sim's sedan CVT fixture curve (its full-throttle point 5600)
    const rg::AutomaticTcuPoints p = rg::derive_automatic_tcu_points(800, 6500);
    CHECK(p.cvt_target_rpm[0] == Approx(1300.0));
    CHECK(p.cvt_target_rpm[1] == Approx(1700.0));
    CHECK(p.cvt_target_rpm[2] == Approx(2600.0));
    CHECK(p.cvt_target_rpm[3] == Approx(4200.0));
    CHECK(p.cvt_target_rpm[4] == Approx(5650.0));
    CHECK(p.cvt_min_rpm[0] == Approx(1200.0));
    CHECK(p.cvt_min_rpm[1] == Approx(1500.0));
    CHECK(p.cvt_min_rpm[2] == Approx(2250.0));   // 800 + 0.25 * 5700 = 2225, to the nearest 50
}

TEST_CASE("cvt gearbox: installing one replaces the clutch and the manual TCU with a cvt ratio law", "[gearbox_swap][automatic][cvt]") {
    Fixture f;
    for (const char* box : {"cvt_converter", "cvt_clutch"}) {
        INFO(box);
        const Loaded l = load_setup(f, f.entry("car_sedan_rwd"), setup_of("car_sedan_rwd", box));
        INFO(l.error);
        REQUIRE(l.ok);
        CHECK(count_components<ps::drivetrain::ClutchDesc>(l.desc) == 0);
        CHECK(count_components<ps::drivetrain::ParkLockDesc>(l.desc) == 1);
        CHECK_FALSE(has_manual_tcu(l.desc));
        const auto* tcu = auto_tcu_of(l.desc);
        REQUIRE(tcu != nullptr);
        REQUIRE(tcu->cvt.has_value());
        CHECK(tcu->cvt->stepped_sport);
        const auto& tgt = tcu->cvt->target_rpm_vs_throttle;
        REQUIRE(tgt.y.size() == 5);
        for (std::size_t i = 0; i + 1 < tgt.y.size(); ++i) CHECK(tgt.y[i] < tgt.y[i + 1]);
        CHECK_FALSE(tcu->cvt->min_rpm_vs_output_rpm.x.empty());
        const ps::drivetrain::GearboxDesc* gb = nullptr;
        for (const auto& c : l.desc.powertrain.components) {
            if (const auto* g = std::get_if<ps::drivetrain::GearboxDesc>(&c.params)) gb = g;
        }
        REQUIRE(gb != nullptr);
        CHECK(gb->type == ps::drivetrain::GearboxType::Cvt);
        CHECK(gb->converter.has_value() == (std::string(box) == "cvt_converter"));
        CHECK(has_line(l.compiled.swap_lines, "CVT: holds"));
        CHECK(has_line(l.compiled.swap_lines, "Clutch removed"));
    }
    // the hypercar's torque is far above the CVT's rating
    const rg::CompiledSetup weak = rg::compile_setup(f.entry("car_hyper"), f.table, f.ctx, setup_of("car_hyper", "cvt_clutch"));
    CHECK_FALSE(weak.ok);
    CHECK_THAT(weak.error, ContainsSubstring("gearbox: rated 250 N*m"));
}

TEST_CASE("cvt gearbox: the sedan with a CVT creeps and drives from rest on the variator", "[gearbox_swap][automatic][cvt][drive]") {
    Fixture f;
    for (const char* box : {"cvt_converter", "cvt_clutch"}) {
        INFO(box);
        const AutoDrive d = drive_auto(f, box, 6.0, 14.0, "car_sedan_rwd");
        CHECK(d.started);
        CHECK_FALSE(d.stalled);
        CHECK(d.transmission_kind == 3);
        CHECK(d.creep_mps > 0.3);
        CHECK(d.creep_mps < 5.0);
        CHECK(d.top_gear == 1);                          // D is one "gear": the variator
        CHECK(d.max_cvt_ratio > 2.0);                    // launches near ratio_low (2.63)
        CHECK(d.max_cvt_ratio - d.min_cvt_ratio > 1.0);  // and sweeps up the ratio range (2.63 -> ~1.36) holding the WOT target
        CHECK(d.max_rpm < 6400.0);                       // the WOT target rpm, held below the 6800 fuel cut
        CHECK(d.end_kmh > 100.0);
        std::printf("RG_CVT_DRIVE %s creep=%.3f m/s ratio %.3f..%.3f max_rpm=%.0f end=%.1f km/h\n", box, d.creep_mps,
                    d.max_cvt_ratio, d.min_cvt_ratio, d.max_rpm, d.end_kmh);
    }
}

TEST_CASE("dual clutch: repeated braking and part-throttle launches do not stall", "[automatic][dct_stop_start]") {
    Fixture f;
    for (const auto& car_box : {std::pair{"car_sedan", "dct_7"}, std::pair{"car_hyper", "hyper_7dct"}}) {
        INFO(car_box.first);
        const auto d = drive_auto(f, car_box.second, 2.0, 8.0, car_box.first, 3);
        REQUIRE(d.started);
        CHECK_FALSE(d.stalled);
    }
}

TEST_CASE("automatic gearbox: swaps preserve N2O hypercar driving modes", "[automatic][drive_modes_auto]") {
    Fixture f;
    for (const char* box : {"stock", "hyper_7dct"}) {
        INFO(box);
        const auto h = load_setup(f, f.entry("car_hyper_n2o"), setup_of("car_hyper_n2o", box));
        INFO(h.error);
        REQUIRE(h.ok);
        REQUIRE(h.desc.drive_modes.has_value());
        CHECK(h.desc.drive_modes->modes_bits == 15);
        CHECK(h.desc.drive_modes->default_mode == ps::vehicle::DriveMode::Normal);
    }
}

TEST_CASE("N2O hypercar modes scale the DCT pedal and preserve the manual pedal policy", "[automatic][drive_modes_auto][pedal]") {
    Fixture f;
    const auto& car = f.entry("car_hyper_n2o");
    for (const char* box : {"stock", "hyper_7dct"}) {
        INFO(box);
        rg::ScopedWorkDir dir(f.ctx.work_root, std::string("mode_pedal_") + box);
        const auto m = rg::materialise_setup(car, f.table, f.ctx, setup_of(car.id, box), dir.path());
        INFO(m.error);
        REQUIRE(m.ok);
        rg::SessionConfig config;
        config.vehicle_json_path = m.vehicle_path;
        config.surface_table_path = kRoot + "/data/surfaces/surfaces.json";
        config.chassis_mass_kg = car.chassis.mass_kg;
        config.chassis_half_extents = ps::Vec3{car.chassis.half_extents[0], car.chassis.half_extents[1], car.chassis.half_extents[2]};
        config.chassis_z_m = car.chassis.spawn_z_m;
        config.engine_map_cache_dir = f.ctx.engine_map_cache_dir;
        rg::Session session(config);
        auto& world = session.world();
        world.set_control("brake", 1.0);
        world.set_control("gear_request", 0.0);
        world.set_control("ignition", 1.0);
        world.set_control("starter", 1.0);
        for (int i = 0; i < 240 * 4; ++i) session.step();
        REQUIRE(world.powertrain_state(session.vehicle_id()).engine_state == ps::drivetrain::EngineState::Running);
        world.set_control("starter", 0.0);
        world.set_control("throttle", 0.6);
        const double expected[] = {0.3, 0.6, 0.8, 0.8};
        for (int mode = 0; mode < 4; ++mode) {
            INFO(mode);
            world.set_control("assist.drive_mode", mode);
            for (int i = 0; i < 240; ++i) session.step();
            const auto pt = world.powertrain_state(session.vehicle_id());
            REQUIRE_FALSE(pt.engines.empty());
            CHECK(int(pt.drive_mode_effective) == mode);
            // Pedal plan P1 deliberately keeps manual/sequential gearboxes unbent.
            const double command = std::string(box) == "stock" ? 0.6 : expected[mode];
            CHECK(double(pt.engines[0].throttle_command) == Approx(command).margin(1e-6));
            std::printf("RG_MODE_PEDAL box=%s mode=%d pedal=0.6 command=%.6f\n", box, mode, double(pt.engines[0].throttle_command));
        }
    }
}

TEST_CASE("hypercar DCT stays running through 10 kmh corners", "[automatic][dct_corner]") {
    Fixture f;
    for (int mode = 0; mode < 4; ++mode) {
        INFO(mode);
        const auto d = drive_auto(f, "hyper_7dct", 0.0, 35.0, "car_hyper_n2o", 0, mode);
        CHECK(d.started);
        CHECK(d.max_kmh > 7.0);
        CHECK_FALSE(d.stalled);
        CHECK(d.max_false_cut_s < 0.2); // no sustained cut when the tyres are rolling without spin
        std::printf("RG_DCT_CORNER mode=%d max_kmh=%.2f end_kmh=%.2f tc_ticks=%d\n", mode, d.max_kmh, d.end_kmh, d.tc_interventions);
    }
}

TEST_CASE("DCT respawn preserves idle and allows the starter after ignition cycling", "[automatic][dct_respawn]") {
    Fixture f;
    const auto result = drive_auto(f, "hyper_7dct", 0.0, 15.0, "car_hyper_n2o", 0, -1, true);
    CHECK(result.started);
}

TEST_CASE("hypercar DCT TC still intervenes during a full-throttle launch", "[automatic][traction_control]") {
    Fixture f;
    const auto d = drive_auto(f, "hyper_7dct", 0.0, 10.0, "car_hyper_n2o");
    CHECK(d.started);
    CHECK_FALSE(d.stalled);
    CHECK(d.max_kmh > 30.0);
    CHECK(d.tc_interventions > 0);
}
TEST_CASE("Racing hypercar DCT launch control reaches 50 kmh promptly", "[automatic][racing_lc]") {
    Fixture f;
    const auto plain=drive_auto(f,"hyper_7dct",0,12,"car_hyper_n2o",0,-1,false,3,false);
    const auto lc=drive_auto(f,"hyper_7dct",0,12,"car_hyper_n2o",0,-1,false,3,true);
    std::printf("RG_LC_RESULT plain_t50=%.3f lc_t50=%.3f\n",plain.t50,lc.t50);
    CHECK(plain.started);CHECK(lc.started);
    CHECK_FALSE(plain.stalled);CHECK_FALSE(lc.stalled);
    REQUIRE(plain.t50>0);REQUIRE(lc.t50>0);
    CHECK(lc.t50<3.0);
    CHECK(lc.t50<=plain.t50);
}

TEST_CASE("Selectable electric sedan materialises and drives without a combustion engine", "[ev][automatic]") {
    Fixture f;const auto& car=f.entry("car_electric");
    rg::VehicleSetup setup;setup.vehicle_id=car.id;
    rg::ScopedWorkDir dir(f.ctx.work_root,"drive_ev");
    const auto material=rg::materialise_setup(car,f.table,f.ctx,setup,dir.path());
    INFO(material.error);REQUIRE(material.ok);
    rg::SessionConfig cfg;cfg.vehicle_json_path=material.vehicle_path;
    cfg.surface_table_path=kRoot+"/data/surfaces/surfaces.json";
    cfg.chassis_mass_kg=car.chassis.mass_kg;cfg.chassis_half_extents={2,.4,.15};cfg.chassis_z_m=.6;
    rg::Session session(cfg);auto& w=session.world();const auto id=session.vehicle_id();
    w.set_control("gear_request",99);w.set_control("throttle",1);
    for(int i=0;i<720;++i)session.step();
    const auto pt=w.powertrain_state(id);
    CHECK(pt.transmission_kind==4);CHECK(pt.electric_ready);CHECK(pt.electric_motor_count==2);
    CHECK(pt.engine_state==ps::drivetrain::EngineState::Off);CHECK(pt.battery_soc<.9);CHECK(pt.drive_power_w>0);
    CHECK(w.get_motion(session.chassis_body()).linear.x>10);    w.set_control("throttle",0);w.set_control("electric.ready",0);
    for(int i=0;i<24;++i)session.step();
    CHECK_FALSE(w.powertrain_state(id).electric_ready);
    session.request_reset_to_spawn();
    for(int i=0;i<24;++i)session.step();
    w.set_control("electric.ready",1);w.set_control("gear_request",99);w.set_control("throttle",1);
    for(int i=0;i<720;++i)session.step();
    CHECK(w.powertrain_state(id).electric_ready);
    CHECK(w.powertrain_state(id).engine_state==ps::drivetrain::EngineState::Off);
    CHECK(w.get_motion(session.chassis_body()).linear.x>10);
}
TEST_CASE("Powertrain swaps preserve the body and materialise all propulsion categories", "[powertrain_swap]") {
 Fixture f;
 for(const auto& car_id:{"car_sedan","car_electric"})for(const auto& choice:{"ice","battery_ev","fuel_cell_ev","hybrid_battery","hybrid_dual_hydrogen"}) {
  INFO(car_id<<" / "<<choice);
  rg::VehicleSetup setup;setup.vehicle_id=car_id;setup.values["powertrain_install"]=std::string(choice);
  const auto result=load_setup(f,f.entry(car_id),setup);INFO(result.error);REQUIRE(result.ok);
  CHECK(result.desc.wheels.size()==4);
  const bool ev=std::string(choice)=="battery_ev"||std::string(choice)=="fuel_cell_ev";
  CHECK(count_components<ps::drivetrain::TorqueMapEngineDesc>(result.desc)==(ev?0:1));
  CHECK(count_components<ps::drivetrain::HybridControlDesc>(result.desc)==(std::string(choice).starts_with("hybrid")?1:0));
  CHECK(count_components<ps::drivetrain::FuelCellDesc>(result.desc)==((std::string(choice)=="fuel_cell_ev"||std::string(choice)=="hybrid_dual_hydrogen")?1:0));
  if(std::string(choice).starts_with("hybrid")){REQUIRE(result.desc.drive_modes.has_value());CHECK(result.desc.drive_modes->modes_bits==15);}
 }
}
TEST_CASE("Hybrid conversions retain modes through AT DCT and CVT installation and launch electrically", "[powertrain_swap][automatic]") {
 Fixture f;
 for(const auto& box:{"auto_8at","dct_7","cvt_clutch"}) {
  INFO(box);rg::VehicleSetup setup;setup.vehicle_id="car_sedan";
  setup.values["powertrain_install"]=std::string("hybrid_dual_hydrogen");setup.values["gearbox_install"]=std::string(box);
  rg::ScopedWorkDir dir(f.ctx.work_root,"hybrid_"+std::string(box));
  const auto m=rg::materialise_setup(f.entry("car_sedan"),f.table,f.ctx,setup,dir.path());INFO(m.error);REQUIRE(m.ok);
  rg::SessionConfig cfg;cfg.vehicle_json_path=m.vehicle_path;cfg.surface_table_path=kRoot+"/data/surfaces/surfaces.json";cfg.chassis_mass_kg=1800;cfg.chassis_half_extents={2,.4,.15};cfg.chassis_z_m=.6;
  rg::Session session(cfg);auto& w=session.world();w.set_control("assist.drive_mode",0);w.set_control("gear_request",99);w.set_control("throttle",.25);
  for(int i=0;i<1200;++i)session.step();auto p=w.powertrain_state(session.vehicle_id());
  CHECK(p.hybrid_fitted);CHECK(p.drive_modes_available_bits==15);CHECK(p.engine_state==ps::drivetrain::EngineState::Off);CHECK(w.get_motion(session.chassis_body()).linear.x>2);
  w.set_control("assist.drive_mode",2);for(int i=0;i<1200;++i)session.step();p=w.powertrain_state(session.vehicle_id());
  CHECK(p.engine_state==ps::drivetrain::EngineState::Running);CHECK(p.hybrid_ice_requested);
 }
}
TEST_CASE("Hybrid supported gearboxes charge the battery in mechanical neutral", "[powertrain_swap][automatic]") {
 Fixture f;
 for(const auto& box:{"auto_8at","dct_7","cvt_clutch"}) {
  INFO(box);rg::VehicleSetup setup;setup.vehicle_id="car_sedan";
  setup.values["powertrain_install"]=std::string("hybrid_dual_hydrogen");setup.values["gearbox_install"]=std::string(box);
  rg::ScopedWorkDir dir(f.ctx.work_root,"hybrid_charge_"+std::string(box));
  const auto m=rg::materialise_setup(f.entry("car_sedan"),f.table,f.ctx,setup,dir.path());INFO(m.error);REQUIRE(m.ok);
  nlohmann::json vehicle;{std::ifstream input(m.vehicle_path);input>>vehicle;}
  for(auto& c:vehicle["powertrain"]["components"])if(c.value("type",std::string{})=="battery") {c["capacity_kwh"]=.01;c["initial_soc"]=.19;}
  for(auto& c:vehicle["powertrain"]["components"])if(c.value("type",std::string{})=="fuel_cell")c["initial_hydrogen_kg"]=0;
  {std::ofstream output(m.vehicle_path);output<<vehicle.dump(2);}
  rg::SessionConfig cfg;cfg.vehicle_json_path=m.vehicle_path;cfg.surface_table_path=kRoot+"/data/surfaces/surfaces.json";cfg.chassis_mass_kg=1800;cfg.chassis_half_extents={2,.4,.15};cfg.chassis_z_m=.6;
  rg::Session session(cfg);auto& w=session.world();w.set_control("assist.drive_mode",0);w.set_control("gear_request",0);w.set_control("throttle",0);
  bool charged=false;
  for(int i=0;i<7200;++i) {
   session.step();const auto p=w.powertrain_state(session.vehicle_id());
   REQUIRE(p.gear==0);
   if(p.battery_soc>.3)charged=true;
  }
  const auto p=w.powertrain_state(session.vehicle_id());CHECK(charged);CHECK(p.battery_soc>=.799);CHECK_FALSE(p.hybrid_charging);CHECK(p.engine_state==ps::drivetrain::EngineState::Off);
  const auto motion=w.get_motion(session.chassis_body());CHECK(std::hypot(motion.linear.x,motion.linear.y)<.1);
 }
}
