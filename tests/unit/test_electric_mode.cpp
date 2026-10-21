// test_electric_mode.cpp - EV one-pedal "B" and the hybrid "Electric" mode on the game side (owner request 2026-10-21).
// Physics (external/physics_sim, docs/electric_drive.md "B mode" / "Hybrid Electric mode") owns every consequence: the B law, the
// selector latch, the Electric supervisor, the wheel-slip guard and the auto hold. The game only
//   * cycles the drive mode through the offered modes (Electric is code 4) and has the "electric_mode" action (key U) toggle it,
//   * steps D <-> B with the shift keys (rg::selector_shift forwards the counters, physics does the step),
//   * shows B / E on the gear display.
//   rg_unit_tests "[b_mode]"
#include "rg/driver_actions.h"
#include "rg/engine_store.h"
#include "rg/gearbox_catalog.h"
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
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <variant>

#include <nlohmann/json.hpp>

namespace {
namespace fs = std::filesystem;
const std::string kRoot = RG_SOURCE_DIR;

nlohmann::json read_json(const std::string& rel) {
    std::ifstream in(kRoot + "/" + rel, std::ios::binary);
    REQUIRE(in.good());
    return nlohmann::json::parse(in);
}

// ---- pure game logic (no physics) -------------------------------------------------------------------------------------

rg::DriverSnapshot ev_snapshot(bool b_available, bool b_active) {
    rg::DriverSnapshot s;
    s.transmission_kind = 4;       // single-speed electric drive
    s.transmission_selector = 1;   // D
    s.gear = 1;
    s.b_available = b_available;
    s.b_active = b_active;
    return s;
}

struct Sink final : rg::ChannelSink {
    std::map<std::string, double, std::less<>> values;
    double get(std::string_view id) const override {
        const auto it = values.find(id);
        return it == values.end() ? 0 : it->second;
    }
    void set(std::string_view id, double value) override { values[std::string(id)] = value; }
};
} // namespace

TEST_CASE("B selector: shift up from D steps to B through the counters, shift down returns to D", "[b_mode][driver_actions]") {
    // D -> up -> B: the counters are forwarded (physics steps D <-> B on them), no gear_request is written.
    auto r = rg::selector_shift(ev_snapshot(true, false), 1, 0);
    CHECK(r.forward_counts);
    CHECK(r.gear_request == rg::kGearRequestNoChange);
    CHECK_FALSE(r.refused);
    // B -> down -> D likewise
    r = rg::selector_shift(ev_snapshot(true, true), 0, 1);
    CHECK(r.forward_counts);
    CHECK(r.gear_request == rg::kGearRequestNoChange);
    // up again in B: nothing above B, nothing forwarded
    r = rg::selector_shift(ev_snapshot(true, true), 1, 0);
    CHECK_FALSE(r.forward_counts);
    CHECK(r.gear_request == rg::kGearRequestNoChange);
    // two presses down from B go past D to N by the absolute request (B counts as D); N is not B
    r = rg::selector_shift(ev_snapshot(true, true), 0, 2);
    CHECK_FALSE(r.forward_counts);
    CHECK(r.gear_request == 0.0);
    // from N up reaches D (an absolute request) and not B in one step
    auto n = ev_snapshot(true, false);
    n.transmission_selector = 0;
    n.gear = 0;
    r = rg::selector_shift(n, 1, 0);
    CHECK(r.gear_request == rg::kGearRequestDrive);
    CHECK_FALSE(r.forward_counts);
    // a car that cannot have B behaves exactly as before: up from D does nothing and the counters are swallowed
    r = rg::selector_shift(ev_snapshot(false, false), 1, 0);
    CHECK_FALSE(r.forward_counts);
    CHECK(r.gear_request == rg::kGearRequestNoChange);
    // the letter on the dash
    CHECK(rg::selector_display(ev_snapshot(true, false)) == "D");
    CHECK(rg::selector_display(ev_snapshot(true, true)) == "B");
    CHECK(rg::selector_label(ev_snapshot(true, true)) == "B");
    auto rev = ev_snapshot(true, false);
    rev.transmission_selector = -1;
    CHECK(rg::selector_display(rev) == "R");
}

TEST_CASE("hybrid in Electric mode: the gear display shows E in D and B in B", "[b_mode][driver_actions]") {
    rg::DriverSnapshot h;
    h.hybrid_fitted = true;
    h.transmission_selector = 1;
    h.hybrid_electric = true;
    h.b_available = true;
    CHECK(rg::selector_display(h) == "E");
    h.b_active = true;
    CHECK(rg::selector_display(h) == "B");
    // the shift keys step D <-> B there as well, whatever the gearbox (physics consumes the counters in Electric mode)
    h.b_active = false;
    CHECK(rg::selector_shift(h, 1, 0).forward_counts);
    // outside Electric mode a hybrid has no B: up from D forwards nothing
    h.hybrid_electric = false;
    h.b_available = false;
    CHECK_FALSE(rg::selector_shift(h, 1, 0).forward_counts);
}

TEST_CASE("electric_mode action toggles Electric (drive mode 4) and returns to the previous mode", "[b_mode][driver_actions]") {
    rg::DriverSnapshot s;
    s.hybrid_fitted = true;
    s.drive_modes = 0b11111; // eco normal sport racing electric
    rg::DriverActions actions;
    Sink sink;
    sink.values["assist.drive_mode"] = 2; // Sport requested
    actions.submit({"electric_mode"});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.drive_mode") == 4);
    actions.submit({"electric_mode"});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.drive_mode") == 2); // back to Sport
    // Set: 1 on, 0 off, idempotent
    actions.submit({"electric_mode", rg::DriverActionKind::Set, 1});
    actions.apply({s, true}, sink);
    actions.submit({"electric_mode", rg::DriverActionKind::Set, 1});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.drive_mode") == 4);
    actions.submit({"electric_mode", rg::DriverActionKind::Set, 0});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.drive_mode") == 2);
    // physics left Electric on its own (low SOC wrote the default back): the next press tries Electric again
    sink.values["assist.drive_mode"] = 1;
    actions.submit({"electric_mode"});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.drive_mode") == 4);
    // a car whose drive-modes part does not offer Electric ignores the key
    s.drive_modes = 0b01111;
    sink.values["assist.drive_mode"] = 1;
    actions.submit({"electric_mode"});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.drive_mode") == 1);
    // the drive_mode cycle reaches Electric when offered, then wraps to Eco
    s.drive_modes = 0b11111;
    sink.values["assist.drive_mode"] = 3;
    actions.submit({"drive_mode"});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.drive_mode") == 4);
    actions.submit({"drive_mode"});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.drive_mode") == 0);
    // the lamp of the cycle action
    s.drive_mode_effective = 4;
    const auto state = rg::DriverActions::state("drive_mode", s, sink);
    CHECK(state.label == "ELECTRIC");
    CHECK(state.colour_role == "electric");
}

TEST_CASE("electric mode data: the action, the key, the hybrid mode list and the derived power", "[b_mode][controls]") {
    // data/controls/actions.json lists the action with the owner's label, the keyboard default is U, no pad default
    bool found = false;
    const auto actions_json = read_json("data/controls/actions.json");
    for (const auto& a : actions_json.at("actions")) {
        if (a.at("id") == "electric_mode") {
            found = true;
            CHECK(a.at("label") == "Electric (E) mode on / off");
        }
    }
    CHECK(found);
    const auto keyboard = read_json("data/controls/defaults/keyboard.json").at("bindings");
    REQUIRE(keyboard.contains("electric_mode"));
    CHECK(keyboard.at("electric_mode")[0].at("key") == "U");
    CHECK_FALSE(read_json("data/controls/defaults/gamepad.json").at("bindings").contains("electric_mode"));
    // no other keyboard binding uses U
    for (const auto& [action, list] : keyboard.items())
        for (const auto& b : list)
            if (action != "electric_mode") CHECK(b.value("key", std::string{}) != "U");
    // the hybrid drive-mode list offers electric
    CHECK(read_json("data/electronics/drive_modes_hybrid.json").at("modes").contains("electric"));
    // the conversion's full-pedal demand is derived by physics now: swaps.json must not author it
    CHECK_FALSE(read_json("data/powertrains/swaps.json").at("hybrid_control").contains("requested_power_w"));
}

// ---- the real hyper hybrid conversion through physics ---------------------------------------------------------------------

namespace {
struct Fx {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_electric_mode"};
    Fx() {
        std::string err;
        auto c = rg::load_vehicle_catalog(kRoot + "/data/vehicles/catalog.json", kRoot, &err);
        REQUIRE(c.has_value());
        catalog = *c;
        auto t = rg::load_setup_options(kRoot + "/data/vehicles/setup_options.json", &err);
        REQUIRE(t.has_value());
        table = *t;
        ctx.tyre_families_path = kRoot + "/data/tyres/families.json";
        ctx.work_root = root.path();
        ctx.engine_map_cache_dir = (fs::temp_directory_path() / "rg_hybrid_electronics_maps").generic_string();
        auto store = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, "", &err);
        REQUIRE(store != nullptr);
        ctx.engines = std::shared_ptr<const rg::EngineStore>(std::move(store));
        auto g = rg::GearboxCatalog::open(kRoot + "/data/gearboxes/gearbox_catalog.json", kRoot, &err);
        REQUIRE(g != nullptr);
        ctx.gearboxes = std::shared_ptr<const rg::GearboxCatalog>(std::move(g));
    }
};

struct Rig {
    rg::MaterialisedSetup m;
    std::unique_ptr<rg::ScopedWorkDir> dir;
    std::unique_ptr<rg::Session> ses;
    bool park_fitted = false;
    struct SessionSink final : rg::ChannelSink {
        rg::Session& s;
        explicit SessionSink(rg::Session& session) : s(session) {}
        double get(std::string_view id) const override { return s.get_control(std::string(id)); }
        void set(std::string_view id, double v) override { s.set_control(std::string(id), v); }
    };
    std::unique_ptr<SessionSink> sink;
    Rig(Fx& f, const std::string& car, const std::string& box, const std::string& dir_name) {
        const rg::CatalogEntry* e = f.catalog.find(car);
        REQUIRE(e != nullptr);
        rg::VehicleSetup setup;
        setup.vehicle_id = car;
        setup.values["powertrain_install"] = std::string("hybrid_battery");
        if (!box.empty()) setup.values["gearbox_install"] = box;
        dir = std::make_unique<rg::ScopedWorkDir>(f.ctx.work_root, dir_name);
        m = rg::materialise_setup(*e, f.table, f.ctx, setup, dir->path());
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
        ses = std::make_unique<rg::Session>(c);
        sink = std::make_unique<SessionSink>(*ses);
        ses->set_control("ignition", 1);
        ses->set_control("electric.ready", 1);
        ses->set_control("assist.auto_shift", 1);
        ses->set_control("assist.auto_clutch", 1);
        ses->set_control("assist.auto_blip", 1);
        for (const auto& component : ses->vehicle_desc().powertrain.components)
            park_fitted = park_fitted || std::holds_alternative<ps::drivetrain::ParkLockDesc>(component.params);
    }
    ps::World& world() { return ses->world(); }
    const ps::drivetrain::PowertrainSnapshot& pt() { return world().powertrain_state(ses->vehicle_id()); }
    double speed() {
        const ps::Pose pose = world().get_pose(ses->chassis_body());
        return pose.orientation.inverse().rotate(world().get_motion(ses->chassis_body()).linear).x;
    }
    // The same projection the Godot adapter hands rg::DriverActions (godot_ext/src/rg_driver_actions.cpp convert()).
    rg::DriverSnapshot snapshot() {
        const auto& p = pt();
        rg::DriverSnapshot s;
        s.drive_modes = static_cast<std::uint32_t>(p.drive_modes_available_bits);
        s.drive_mode_effective = static_cast<int>(p.drive_mode_effective);
        s.transmission_kind = p.transmission_kind;
        s.transmission_selector = p.transmission_selector;
        s.hybrid_fitted = p.hybrid_fitted;
        s.park_fitted = park_fitted;
        s.park_engaged = p.park_lock_engaged != 0;
        s.engine_running = p.engine_state == ps::drivetrain::EngineState::Running;
        s.gear = p.gear;
        s.clutch_engagement = p.clutch_engagement;
        s.electric_ready = p.electric_ready;
        s.b_available = p.b_mode_available;
        s.b_active = p.b_mode_active;
        s.hybrid_electric = p.hybrid_electric_mode;
        s.speed_mps = std::abs(speed());
        return s;
    }
    void tick() { while (!ses->try_step()) {} }
    void run(double seconds) {
        for (int i = 0; i < static_cast<int>(seconds * 240); ++i) tick();
    }
    // What main.gd does for the shift keys: selector_shift, then the forwarded counters.
    void shift_keys(int up, int down) {
        const rg::SelectorShift r = rg::selector_shift(snapshot(), up, down);
        if (r.gear_request != rg::kGearRequestNoChange) ses->set_control("gear_request", r.gear_request);
        if (r.park_request >= 0) ses->set_control("park_lock", r.park_request);
        if (!r.forward_counts) { up = 0; down = 0; }
        if (up > 0) ses->set_control("shift_up_count", ses->get_control("shift_up_count") + up);
        if (down > 0) ses->set_control("shift_down_count", ses->get_control("shift_down_count") + down);
    }
    void press(rg::DriverActions& a, const std::string& action) {
        REQUIRE(a.submit({action}));
        a.apply({snapshot(), true}, *sink);
    }
};
} // namespace

TEST_CASE("hyper hybrid: E key enters Electric, shift keys step D and B, E key returns", "[b_mode][hybrid][powertrain_swap][long]") {
    Fx f;
    Rig r(f, "car_hyper_n2o", "hyper_7dct", "t_em_keys");
    r.ses->set_control("assist.drive_mode", 2); // Sport
    r.run(1);
    r.ses->set_control("gear_request", rg::kGearRequestDrive);
    // the conversion's drive-modes part offers Electric (bit 4)
    CHECK((static_cast<unsigned>(r.pt().drive_modes_available_bits) & 0b10000u) != 0);
    rg::DriverActions actions;
    r.press(actions, "electric_mode");
    CHECK(r.ses->get_control("assist.drive_mode") == 4);
    r.ses->set_control("throttle", 0.3);
    r.run(3);
    const auto& p = r.pt();
    CHECK(p.hybrid_electric_mode);
    CHECK(p.drive_mode_effective == 4);
    CHECK(p.engine_state != ps::drivetrain::EngineState::Running); // ICE off and disconnected
    CHECK(p.b_mode_available);
    CHECK_FALSE(p.b_mode_active);
    CHECK(rg::selector_display(r.snapshot()) == "E");
    // shift up: D -> B (physics steps on the counters)
    r.shift_keys(1, 0);
    r.run(0.5);
    CHECK(r.pt().b_mode_active);
    CHECK(rg::selector_display(r.snapshot()) == "B");
    // pedal released in B: regeneration (negative electric power) while rolling
    r.ses->set_control("throttle", 0.0);
    r.run(2);
    CHECK(r.pt().b_power_command_w < 0);
    CHECK(r.pt().b_zero_power_pedal > 0);
    // shift down: B -> D
    r.shift_keys(0, 1);
    r.run(0.5);
    CHECK_FALSE(r.pt().b_mode_active);
    // E key again: back to Sport, B ends, the combustion engine may run again
    r.shift_keys(1, 0);
    r.run(0.5);
    REQUIRE(r.pt().b_mode_active);
    r.press(actions, "electric_mode");
    CHECK(r.ses->get_control("assist.drive_mode") == 2);
    r.run(1);
    CHECK_FALSE(r.pt().hybrid_electric_mode);
    CHECK_FALSE(r.pt().b_mode_active); // leaving Electric ends B
}

TEST_CASE("hyper hybrid: the full-pedal demand is derived, Sport at WOT asks for far more than the electric potential", "[b_mode][hybrid][powertrain_swap][long]") {
    Fx f;
    Rig r(f, "car_hyper_n2o", "hyper_7dct", "t_em_derived");
    r.ses->set_control("assist.drive_mode", 2); // Sport: the combustion engine is always coupled
    r.run(2);
    r.ses->set_control("gear_request", rg::kGearRequestDrive);
    r.ses->set_control("throttle", 1.0);
    r.run(6);
    const auto& p = r.pt();
    INFO("requested " << p.hybrid_requested_power_w / 1000 << " kW, available " << p.hybrid_available_power_w / 1000 << " kW");
    CHECK(p.hybrid_requested_power_w > 2.0 * p.hybrid_available_power_w);
    CHECK(p.engine_state == ps::drivetrain::EngineState::Running);
}
