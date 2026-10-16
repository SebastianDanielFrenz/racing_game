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
    double end_kmh = 0.0;
    bool stalled = false;
};
// The car_sedan with an automatic on a flat tessellated ground: start the engine, select D, release the brake and creep
// with no throttle for creep_s, then full throttle for drive_s.
AutoDrive drive_auto(const Fixture& f, const std::string& box, double creep_s, double drive_s, const std::string& car_id = "car_sedan") {
    const rg::CatalogEntry& car = f.entry(car_id);
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
        out.transmission_kind = static_cast<int>(pt.transmission_kind);
        out.max_rpm = std::max(out.max_rpm, static_cast<double>(pt.engine_rpm));
        if (pt.transmission_kind == 3) {
            out.max_cvt_ratio = std::max(out.max_cvt_ratio, static_cast<double>(pt.cvt_ratio));
            out.min_cvt_ratio = std::min(out.min_cvt_ratio, static_cast<double>(pt.cvt_ratio));
        }
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
