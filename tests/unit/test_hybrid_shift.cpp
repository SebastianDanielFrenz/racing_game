// test_hybrid_shift.cpp - a hybrid in D shifts by itself and the manual-mode key works on it (owner 2026-10-18, the engine
// never shifts into 2nd). Findings:
//  * hybrid + DUAL-CLUTCH box (hyper_7dct): D shifts through the gears on the box's own schedule; Y / up / down / Y work.
//  * hybrid + MANUAL box (hyper_7mt, ManualTcu): D never leaves 1st. The physics hybrid supervisor (external/physics_sim
//    core/src/drivetrain/powertrain.cpp, Powertrain::pre_solve_components, the non-automatic branch of the gear_request
//    override: gear_request = box at position 0 ? 1 : none) asks for gear 1 whenever the box is at position 0, which it
//    passes through in EVERY shift, so each upshift the auto-shift starts is reverted to 1st. Not fixable from the game; the
//    case is [!mayfail] until physics_sim changes.
// rg_unit_tests "[hybrid_shift]"
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
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <algorithm>
#include <string>
#include <variant>
#include <vector>

namespace {
namespace fs = std::filesystem;
const std::string kRoot = RG_SOURCE_DIR;
std::string env_or(const char* name, const char* fallback) {
    char* buf = nullptr;
    std::size_t n = 0;
    std::string out = fallback;
    if (_dupenv_s(&buf, &n, name) == 0 && buf) { out = buf; std::free(buf); }
    return out;
}

struct Fx {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_hybrid_shift"};
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
    struct Sink final : rg::ChannelSink {
        rg::Session& s;
        explicit Sink(rg::Session& session) : s(session) {}
        double get(std::string_view id) const override { return s.get_control(std::string(id)); }
        void set(std::string_view id, double v) override { s.set_control(std::string(id), v); }
    };
    std::unique_ptr<Sink> sink;
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
        // The GDScript readers look the car up by the catalog's sim_name (RgSimulation::has_vehicle compares it with the
        // loaded desc's name): a materialised setup must keep it or every dashboard reads an empty powertrain (N, 0, 0).
        CHECK(ses->vehicle_desc().name == e->sim_name);
        sink = std::make_unique<Sink>(*ses);
        // The game's startup state (main.gd): ignition on, electric drive ready, the shift assists as the car defaults them.
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
    rg::DriverSnapshot snapshot() {
        const auto& p = pt();
        rg::DriverSnapshot s;
        s.transmission_kind = p.transmission_kind;
        s.transmission_selector = p.transmission_selector;
        s.transmission_manual = p.transmission_manual_mode;
        s.transmission_sport = p.transmission_sport_mode;
        s.hybrid_fitted = p.hybrid_fitted;
        s.park_fitted = park_fitted;
        s.park_engaged = p.park_lock_engaged != 0;
        s.engine_running = p.engine_state == ps::drivetrain::EngineState::Running;
        s.gear = p.gear;
        s.clutch_engagement = p.clutch_engagement;
        s.electric_ready = p.electric_ready;
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

// WOT from rest in D: the gears seen in order and the ICE rpm at each upshift request.
struct Pull {
    std::vector<int> gears;
    int max_gear = 0;
    double max_rpm = 0, final_kmh = 0;
};
Pull wot_pull(Rig& r, double seconds) {
    Pull out;
    r.ses->set_control("assist.drive_mode", 1);
    r.run(2);
    r.ses->set_control("gear_request", rg::kGearRequestDrive);
    r.run(0.3);
    r.ses->set_control("throttle", 1.0);
    int last_gear = -9;
    for (int i = 0; i < static_cast<int>(seconds * 240); ++i) {
        r.tick();
        const auto& p = r.pt();
        if (p.gear != last_gear) { out.gears.push_back(p.gear); last_gear = p.gear; }
        out.max_gear = std::max<int>(out.max_gear, p.gear);
        out.max_rpm = std::max<double>(out.max_rpm, p.engine_rpm);
    }
    out.final_kmh = r.speed() * 3.6;
    return out;
}
} // namespace

TEST_CASE("hybrid + dual-clutch box: D shifts through the gears by itself", "[hybrid_shift][powertrain_swap]") {
    Fx f;
    Rig r(f, "car_hyper_n2o", "hyper_7dct", "t_hs_dct");
    const Pull p = wot_pull(r, 9.0);
    INFO("max gear " << p.max_gear << " max rpm " << p.max_rpm << " final " << p.final_kmh);
    REQUIRE(p.gears.size() >= 3);
    CHECK(p.max_gear >= 4);
    CHECK(p.final_kmh > 150.0);
    int prev = 0;
    for (int g : p.gears) {
        if (g == 0) continue;
        CHECK(g >= prev); // no 1 -> 2 -> 1 hunting in a WOT pull
        prev = g;
    }
}

TEST_CASE("hybrid + manual ManualTcu box: D shifts 1 -> 2 -> 3 on the schedule", "[hybrid_shift][powertrain_swap][!mayfail]") {
    // Physics defect (see the file header): Powertrain::pre_solve_components reverts every upshift to 1st.
    Fx f;
    Rig r(f, "car_hyper", "", "t_hs_mt");
    const Pull p = wot_pull(r, 6.0);
    INFO("max gear " << p.max_gear << " max rpm " << p.max_rpm);
    CHECK(p.max_gear >= 3);
}

TEST_CASE("hybrid + dual-clutch box: Y enters manual mode, up/up/down change gears, Y returns to D", "[hybrid_shift][driver_actions]") {
    Fx f;
    Rig r(f, "car_hyper_n2o", "hyper_7dct", "t_hs_manual");
    r.ses->set_control("assist.drive_mode", 2); // Sport: the combustion engine is coupled
    r.run(2);
    r.ses->set_control("gear_request", rg::kGearRequestDrive);
    r.ses->set_control("throttle", 0.6);
    r.run(8);
    REQUIRE(r.pt().engine_state == ps::drivetrain::EngineState::Running);
    REQUIRE(r.pt().gear >= 2);
    CHECK_FALSE(r.pt().transmission_manual_mode);
    rg::DriverActions actions;
    r.press(actions, "manual_mode");
    r.run(0.5);
    CHECK(r.pt().transmission_manual_mode);
    const int g0 = r.pt().gear;
    CHECK(rg::selector_label(r.snapshot()) == "M" + std::to_string(std::max(1, g0)));
    r.shift_keys(1, 0);
    r.run(1.5);
    r.shift_keys(1, 0);
    r.run(1.5);
    CHECK(r.pt().gear == g0 + 2);
    r.shift_keys(0, 1);
    r.run(1.5);
    CHECK(r.pt().gear == g0 + 1);
    CHECK(r.pt().transmission_manual_mode);
    r.press(actions, "manual_mode"); // hybrid: N for manual_exit_hold_s, then D from advance()
    r.run(0.1);
    actions.advance({r.snapshot(), true}, *r.sink, 0.1);
    r.run(1.0);
    CHECK_FALSE(r.pt().transmission_manual_mode);
    CHECK(r.pt().transmission_selector > 0);
    CHECK(r.pt().gear > 0);
    CHECK(rg::selector_label(r.snapshot()).substr(0, 1) == "D");
}

TEST_CASE("hybrid dashboard values while driving in D", "[.][diag][hybrid_shift]") {
    Fx f;
    Rig r(f, env_or("HS_CAR", "car_hyper_n2o"), env_or("HS_GB", "hyper_7dct"), "t_hs_dash");
    r.ses->set_control("assist.drive_mode", std::atoi(env_or("HS_MODE", "1").c_str()));
    r.run(2);
    r.ses->set_control("gear_request", rg::kGearRequestDrive);
    r.ses->set_control("throttle", 0.3);
    for (int i = 0; i < 240 * 20; ++i) {
        r.tick();
        if (i % 12 == 0) {
            const auto& p = r.pt();
            std::printf("t=%5.2f v=%6.1f speedo=%6.1f gear=%d rpm=%6.0f motor=%6.0f st=%d sel=%d ice=%d soc=%.3f\n", i / 240.0, r.speed() * 3.6,
                        p.speedo_mps * 3.6, p.gear, p.engine_rpm, p.motor_rpm, static_cast<int>(p.engine_state), p.transmission_selector,
                        p.hybrid_ice_requested, p.battery_soc);
        }
    }
}

TEST_CASE("hybrid shift diag", "[.][diag][hybrid_shift]") {
    Fx f;
    const std::string vid = env_or("HS_CAR", "car_hyper");
    const std::string gb = env_or("HS_GB", "");
    const rg::CatalogEntry* e = f.catalog.find(vid);
    REQUIRE(e != nullptr);
    rg::VehicleSetup setup;
    setup.vehicle_id = vid;
    setup.values["powertrain_install"] = std::string("hybrid_battery");
    if (!gb.empty()) setup.values["gearbox_install"] = gb;
    {
        // HS_OPTS="key=value,key=value" (strings) / numbers for final_drive
        std::string o = env_or("HS_OPTS", "");
        std::size_t pos = 0;
        while (pos < o.size()) {
            std::size_t comma = o.find(',', pos);
            if (comma == std::string::npos) comma = o.size();
            const std::string kv = o.substr(pos, comma - pos);
            const std::size_t eq = kv.find('=');
            if (eq != std::string::npos) {
                const std::string k = kv.substr(0, eq), v = kv.substr(eq + 1);
                if (k == "final_drive") setup.values[k] = std::atof(v.c_str());
                else setup.values[k] = v;
            }
            pos = comma + 1;
        }
    }
    rg::ScopedWorkDir dir(f.ctx.work_root, "t_shift");
    const rg::MaterialisedSetup m = rg::materialise_setup(*e, f.table, f.ctx, setup, dir.path());
    INFO(m.error);
    REQUIRE(m.ok);
    std::printf("vehicle %s\n", m.vehicle_path.c_str());
    { const std::string dump = env_or("HS_DUMP", ""); if (!dump.empty()) fs::copy_file(m.vehicle_path, dump, fs::copy_options::overwrite_existing); }
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
    rg::Session ses(c);
    auto& w = ses.world();
    const int mode = std::atoi(env_or("HS_MODE", "1").c_str());
    w.set_control("assist.drive_mode", mode);
    for (int i = 0; i < 480; ++i) ses.step();
    w.set_control("gear_request", rg::kGearRequestDrive);
    for (int i = 0; i < 72; ++i) ses.step();
    w.set_control("throttle", 1.0);
    for (int i = 0; i < 240 * 14; ++i) {
        ses.step();
        if (i % 24 == 0) {
            const auto& p = w.powertrain_state(ses.vehicle_id());
            const ps::Pose pose = w.get_pose(ses.chassis_body());
            const double v = pose.orientation.inverse().rotate(w.get_motion(ses.chassis_body()).linear).x;
            std::printf("t=%5.2f v=%6.1f gear=%d tgt=%d ph=%d rpm=%6.0f motor=%6.0f st=%d clutch=%.2f sel=%d kind=%d auto=%d bits=0x%x elec=%.0fkW req=%.0f avail=%.0f ice=%d\n",
                        i / 240.0, v * 3.6, p.gear, p.gear_target, static_cast<int>(p.shift_phase), p.engine_rpm, p.motor_rpm,
                        static_cast<int>(p.engine_state), p.clutch_engagement, p.transmission_selector, p.transmission_kind,
                        static_cast<int>(w.get_control("assist.auto_shift")), p.assist_active_bits, p.electric_power_w / 1000,
                        p.hybrid_requested_power_w / 1000, p.hybrid_available_power_w / 1000, p.hybrid_ice_requested);
        }
    }
}
