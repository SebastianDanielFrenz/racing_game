// test_hybrid_selector.cpp - every hybrid has the P R N D selector, whatever its gearbox (owner decision 2026-10-18, PR #42).
// The physics hybrid supervisor owns the gearbox (forced auto clutch and auto shift) and takes its D/N/R from gear_request;
// a manual-gearbox hybrid (car_hyper + hybrid_battery, hyper_7mt) started in neutral and could not be put in drive because
// rg::selector_shift ignored cars with transmission_kind 0. Here the shift keys step the selector like on an automatic, and
// the gear indicator shows the letter plus the gear only while the combustion engine is coupled.
//   rg_unit_tests "[hybrid_selector]"
#include "rg/driver_actions.h"
#include "rg/engine_store.h"
#include "rg/session.h"
#include "rg/vehicle_catalog.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <variant>

namespace {
namespace fs = std::filesystem;
const std::string kRoot = RG_SOURCE_DIR;

TEST_CASE("the gear indicator of a selector car: letter, plus the gear only when it drives the wheels", "[hybrid_selector][driver_actions]") {
    rg::DriverSnapshot s;
    CHECK(rg::selector_display(s).empty()); // an ICE with a manual box: the caller shows the gear number
    CHECK_FALSE(rg::has_selector(s));
    // pure EV (single-speed drive: kind 4, gear = direction): just the letter
    rg::DriverSnapshot ev;
    ev.transmission_kind = 4;
    ev.transmission_selector = 1;
    ev.gear = 1;
    CHECK(rg::selector_display(ev) == "D");
    ev.transmission_selector = -1;
    ev.gear = -1;
    CHECK(rg::selector_display(ev) == "R");
    // a parallel hybrid with a manual box (transmission_kind 0): the selector exists all the same
    rg::DriverSnapshot h;
    h.hybrid_fitted = true;
    h.park_fitted = true;
    CHECK(rg::has_selector(h));
    CHECK(rg::selector_display(h) == "N");
    h.park_engaged = true;
    CHECK(rg::selector_display(h) == "P");
    h.park_engaged = false;
    h.transmission_selector = -1;
    CHECK(rg::selector_display(h) == "R");
    h.transmission_selector = 1;
    CHECK(rg::selector_display(h) == "D"); // electric only: engine off, no gear in
    h.gear = 3;
    CHECK(rg::selector_display(h) == "D"); // gear in but the engine is not running
    h.engine_running = true;
    h.clutch_engagement = 0.2;
    CHECK(rg::selector_display(h) == "D"); // running, clutch still slipping: not coupled
    h.clutch_engagement = 1.0;
    CHECK(rg::selector_display(h) == "D3"); // coupled
    h.gear = 0;
    CHECK(rg::selector_display(h) == "D"); // supervisor disconnected the gearbox
    // an automatic (not a hybrid) keeps showing the running gear
    rg::DriverSnapshot a;
    a.transmission_kind = 1;
    a.transmission_selector = 1;
    a.gear = 4;
    CHECK(rg::selector_display(a) == "D4");
    struct Empty final : rg::ChannelSink {
        double get(std::string_view) const override { return 0; }
        void set(std::string_view, double) override {}
    } sink;
    const rg::ControlState st = rg::DriverActions::state("transmission", h, sink);
    CHECK(st.fitted);
    CHECK(st.label == "D");
}

TEST_CASE("selector_shift steps the selector of a manual-gearbox hybrid like an automatic's", "[hybrid_selector][driver_actions]") {
    rg::DriverSnapshot s;
    s.hybrid_fitted = true;
    s.park_fitted = false;
    // no park lock: N, up = D, down = N, down again = R
    rg::SelectorShift up = rg::selector_shift(s, 1, 0);
    CHECK(up.gear_request == rg::kGearRequestDrive);
    CHECK_FALSE(up.forward_counts);
    s.transmission_selector = 1;
    rg::SelectorShift down = rg::selector_shift(s, 0, 1);
    CHECK(down.gear_request == 0.0);
    s.transmission_selector = 0;
    CHECK(rg::selector_shift(s, 0, 1).gear_request == -1.0);
    s.transmission_selector = -1;
    CHECK(rg::selector_shift(s, 0, 1).gear_request == rg::kGearRequestNoChange); // no park lock: R is the lowest position
    // rolling: R and P are refused
    s.transmission_selector = 1;
    s.park_fitted = true;
    s.speed_mps = 10;
    CHECK(rg::selector_shift(s, 0, 2).refused);
}

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_hybrid_selector"};
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
        ctx.engine_map_cache_dir = (fs::temp_directory_path() / "rg_hybrid_electronics_maps").generic_string();
        auto store = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, "", &err);
        REQUIRE(store != nullptr);
        ctx.engines = std::shared_ptr<const rg::EngineStore>(std::move(store));
    }
};

// What the game does each frame: the published snapshot becomes a DriverSnapshot, a shift key press goes through selector_shift
// and its gear_request / park_lock go into the sim channels (game/scripts/main.gd).
struct Drive {
    rg::Session ses;
    bool park_fitted = false;
    explicit Drive(const rg::SessionConfig& c) : ses(c) {
        for (const auto& component : ses.vehicle_desc().powertrain.components)
            park_fitted = park_fitted || std::holds_alternative<ps::drivetrain::ParkLockDesc>(component.params);
    }
    rg::DriverSnapshot snapshot() {
        const auto& p = ses.world().powertrain_state(ses.vehicle_id());
        rg::DriverSnapshot s;
        s.transmission_kind = p.transmission_kind;
        s.transmission_selector = p.transmission_selector;
        s.transmission_manual = p.transmission_manual_mode;
        s.hybrid_fitted = p.hybrid_fitted;
        s.park_fitted = park_fitted;
        s.park_engaged = p.park_lock_engaged != 0;
        s.engine_running = p.engine_state == ps::drivetrain::EngineState::Running;
        s.gear = p.gear;
        s.clutch_engagement = p.clutch_engagement;
        s.electric_ready = p.electric_ready;
        s.speed_mps = std::abs(forward_speed());
        return s;
    }
    double forward_speed() {
        auto& w = ses.world();
        const ps::Pose pose = w.get_pose(ses.chassis_body());
        return pose.orientation.inverse().rotate(w.get_motion(ses.chassis_body()).linear).x;
    }
    rg::SelectorShift press(int up, int down) {
        const rg::SelectorShift r = rg::selector_shift(snapshot(), up, down);
        if (r.gear_request != rg::kGearRequestNoChange) ses.world().set_control("gear_request", r.gear_request);
        if (r.park_request >= 0) ses.world().set_control("park_lock", r.park_request);
        return r;
    }
    void run(double seconds) {
        for (int i = 0; i < static_cast<int>(seconds * 240); ++i) ses.step();
    }
    void stop() {
        auto& w = ses.world();
        w.set_control("throttle", 0);
        w.set_control("brake", 1);
        for (int i = 0; i < 240 * 20 && std::abs(forward_speed()) > 0.2; ++i) ses.step();
        run(0.5);
        w.set_control("brake", 0);
    }
};
} // namespace

TEST_CASE("car_hyper + hybrid battery (manual hyper_7mt) starts in N, the shift keys select D and R and park locks it", "[hybrid_selector][powertrain_swap]") {
    Fixture f;
    const rg::CatalogEntry* e = f.catalog.find("car_hyper");
    REQUIRE(e != nullptr);
    rg::VehicleSetup setup;
    setup.vehicle_id = "car_hyper";
    setup.values["powertrain_install"] = std::string("hybrid_battery");
    rg::ScopedWorkDir dir(f.ctx.work_root, "t_selector");
    const rg::MaterialisedSetup m = rg::materialise_setup(*e, f.table, f.ctx, setup, dir.path());
    INFO(m.error);
    REQUIRE(m.ok);
    rg::SessionConfig c;
    c.vehicle_json_path = m.vehicle_path;
    c.surface_table_path = kRoot + "/data/surfaces/surfaces.json";
    c.engine_map_cache_dir = f.ctx.engine_map_cache_dir;
    c.chassis_mass_kg = e->chassis.mass_kg + m.compiled.chassis_mass_delta_kg;
    c.chassis_half_extents = ps::Vec3{e->chassis.half_extents[0], e->chassis.half_extents[1], e->chassis.half_extents[2]};
    c.chassis_z_m = e->chassis.spawn_z_m;
    c.ground_tessellated = true;
    c.ground_half_extent_m = 3000;
    c.ground_half_extent_y_m = 200;
    Drive d(c);
    auto& w = d.ses.world();
    REQUIRE(d.park_fitted);
    w.set_control("assist.drive_mode", 0); // Eco: electric
    d.run(2);
    {
        const rg::DriverSnapshot s = d.snapshot();
        REQUIRE(s.hybrid_fitted);
        CHECK(s.transmission_kind == 0); // the manual hyper_7mt with a park lock: not an automatic
        CHECK(s.transmission_selector == 0);
        CHECK(rg::selector_display(s) == "N");
        w.set_control("throttle", 0.5);
        d.run(2);
        CHECK(std::abs(d.forward_speed()) < 0.5); // in neutral the car never drives
        w.set_control("throttle", 0);
    }
    // shift up from N: D, the pedal moves the car forward
    const rg::SelectorShift up = d.press(1, 0);
    CHECK_FALSE(up.forward_counts);
    d.run(0.3);
    CHECK(d.snapshot().transmission_selector == 1);
    CHECK(rg::selector_display(d.snapshot()).substr(0, 1) == "D");
    w.set_control("throttle", 0.6);
    d.run(5);
    CHECK(d.forward_speed() > 5.0);
    // stop, shift down twice: N then R, and the car reverses
    d.stop();
    d.press(0, 1);
    d.run(0.3);
    CHECK(d.snapshot().transmission_selector == 0);
    CHECK(rg::selector_display(d.snapshot()) == "N");
    d.press(0, 1);
    d.run(0.3);
    CHECK(d.snapshot().transmission_selector == -1);
    CHECK(rg::selector_display(d.snapshot()) == "R");
    w.set_control("throttle", 0.5);
    d.run(4);
    CHECK(d.forward_speed() < -1.0);
    // stop, shift down once more at standstill: P engages the park lock
    d.stop();
    const rg::SelectorShift park = d.press(0, 1);
    CHECK_FALSE(park.refused);
    CHECK(park.park_request == 1);
    d.run(1.0);
    CHECK(w.powertrain_state(d.ses.vehicle_id()).park_lock_engaged != 0);
    CHECK(rg::selector_display(d.snapshot()) == "P");
    // back up: R, N, D release the lock again
    d.press(1, 0);
    d.run(0.3);
    CHECK(w.powertrain_state(d.ses.vehicle_id()).park_lock_engaged == 0);
}
