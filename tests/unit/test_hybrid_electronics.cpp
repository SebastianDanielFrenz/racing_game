// test_hybrid_electronics.cpp - a hybrid conversion keeps the car's own electronics (PR #42 ready-gate). The conversion
// used to drop every electronics controller but the parking brake and add the hybrid assist unit; with an ABS/ESC unit
// on the car that left two units ("at most one assist_unit per vehicle") or no ABS at all. Now exactly one unit remains:
// the car's own with the hybrid unit's traction control, fitted capabilities and hybrid-tuned offered TC modes.
//   rg_unit_tests "[hybrid_electronics]"
#include "rg/engine_store.h"
#include "rg/gearbox_catalog.h"
#include "../../external/physics_sim/core/src/drivetrain/electric_drive.h"
#include <catch2/catch_approx.hpp>
#include "rg/garage.h"
#include "rg/session.h"
#include "rg/vehicle_catalog.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/io/vehicle_io.h"
#include "ps/vehicle/assist_unit_desc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

namespace {
namespace fs = std::filesystem;
const std::string kRoot = RG_SOURCE_DIR;

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_hybrid_electronics"};
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
        // Its own map cache: the owner's running game uses out/.
        ctx.engine_map_cache_dir = (fs::temp_directory_path() / "rg_hybrid_electronics_maps").generic_string();
        auto store = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, "", &err);
        REQUIRE(store != nullptr);
        ctx.engines = std::shared_ptr<const rg::EngineStore>(std::move(store));
        auto boxes = rg::GearboxCatalog::open(kRoot + "/data/gearboxes/gearbox_catalog.json", kRoot, &err);
        REQUIRE(boxes);
        ctx.gearboxes = std::shared_ptr<const rg::GearboxCatalog>(std::move(boxes));
    }
    const rg::CatalogEntry& entry(const std::string& id) const {
        const rg::CatalogEntry* e = catalog.find(id);
        REQUIRE(e != nullptr);
        return *e;
    }
};

rg::VehicleSetup setup_of(const char* car, const char* choice) {
    rg::VehicleSetup s;
    s.vehicle_id = car;
    s.values["powertrain_install"] = std::string(choice);
    return s;
}

ps::vehicle::VehicleDesc load_desc(const std::string& path) {
    ps::io::VehicleLoadOptions options;
    options.maps.generate = false;
    return rg::load_vehicle(path, options);
}

ps::vehicle::AssistUnitDesc hybrid_unit() {
    static const ps::vehicle::VehicleDesc d = [] {
        ps::io::VehicleLoadOptions options;
        options.maps.generate = false;
        options.electronics["assists"] = kRoot + "/data/electronics/assist_unit_hybrid.json";
        return rg::load_vehicle(kRoot + "/data/vehicles/car_sedan.json", options);
    }();
    REQUIRE(d.assist_unit.has_value());
    return *d.assist_unit;
}
} // namespace

TEST_CASE("hybrid conversions keep the car's ABS, ESC, aero and launch capabilities", "[hybrid_electronics][powertrain_swap]") {
    Fixture f;
    const auto hybrid = hybrid_unit();
    REQUIRE(hybrid.has_traction_control);
    for (const char* car : {"car_hyper", "car_sedan"}) {
        const rg::CatalogEntry& e = f.entry(car);
        const auto own = load_desc(e.vehicle_path);
        REQUIRE(own.assist_unit.has_value());
        for (const char* choice : {"hybrid_battery", "hybrid_dual_hydrogen"}) {
            INFO(car << " / " << choice);
            const auto setup = setup_of(car, choice);
            const auto v = rg::validate_setup(e, f.table, f.ctx, setup, false);
            INFO(v.message);
            REQUIRE(v.ok);
            rg::ScopedWorkDir dir(f.ctx.work_root, std::string("t_") + car);
            const auto m = rg::materialise_setup(e, f.table, f.ctx, setup, dir.path());
            INFO(m.error);
            REQUIRE(m.ok);
            const auto d = load_desc(m.vehicle_path);
            REQUIRE(d.assist_unit.has_value());
            const auto& u = *d.assist_unit;
            CHECK(u.abs.has_value());
            CHECK(u.esc.has_value());
            CHECK(u.aero_control.has_value() == own.assist_unit->aero_control.has_value());
            CHECK(u.launch_control.has_value() == own.assist_unit->launch_control.has_value());
            CHECK(u.has_traction_control == own.assist_unit->has_traction_control);
            CHECK(u.traction_control.modes_bits == own.assist_unit->traction_control.modes_bits);
            REQUIRE(d.drive_modes.has_value());
            CHECK(d.drive_modes->modes_bits == 31); // eco normal sport racing electric; the hybrid modes replace the car's own
        }
    }
}

TEST_CASE("a hybrid conversion of a car without an assist unit still gets the hybrid unit", "[hybrid_electronics][powertrain_swap]") {
    Fixture f;
    const rg::CatalogEntry& e = f.entry("car_runabout");
    const auto setup = setup_of("car_runabout", "hybrid_battery");
    rg::ScopedWorkDir dir(f.ctx.work_root, "t_runabout");
    const auto m = rg::materialise_setup(e, f.table, f.ctx, setup, dir.path());
    INFO(m.error);
    REQUIRE(m.ok);
    const auto d = load_desc(m.vehicle_path);
    REQUIRE(d.assist_unit.has_value());
    CHECK_FALSE(d.assist_unit->abs.has_value());
    CHECK(d.assist_unit->has_traction_control);
}

TEST_CASE("a hybrid setup survives save and reload and returning to ICE restores the car's own unit", "[hybrid_electronics][powertrain_swap]") {
    Fixture f;
    for (const char* car : {"car_hyper", "car_sedan"}) {
        INFO(car);
        const rg::CatalogEntry& e = f.entry(car);
        const auto own = load_desc(e.vehicle_path);
        REQUIRE(own.assist_unit.has_value());
        const auto saved = rg::setup_to_json(setup_of(car, "hybrid_battery"));
        std::string err;
        const auto back = rg::parse_setup(saved, f.table, "test", &err);
        INFO(err);
        REQUIRE(back.has_value());
        rg::ScopedWorkDir dir_a(f.ctx.work_root, "t_reload");
        const auto a = rg::materialise_setup(e, f.table, f.ctx, *back, dir_a.path());
        INFO(a.error);
        REQUIRE(a.ok);
        const auto hybrid = load_desc(a.vehicle_path);
        REQUIRE(hybrid.assist_unit.has_value());
        CHECK(hybrid.assist_unit->abs.has_value());
        CHECK(hybrid.assist_unit->launch_control.has_value() == own.assist_unit->launch_control.has_value());
        // back to ICE
        rg::ScopedWorkDir dir_b(f.ctx.work_root, "t_ice");
        const auto b = rg::materialise_setup(e, f.table, f.ctx, setup_of(car, "ice"), dir_b.path());
        INFO(b.error);
        REQUIRE(b.ok);
        const auto ice = load_desc(b.vehicle_path);
        REQUIRE(ice.assist_unit.has_value());
        const auto& u = *ice.assist_unit;
        const auto& o = *own.assist_unit;
        CHECK(u.has_traction_control == o.has_traction_control);
        CHECK(u.launch_control.has_value() == o.launch_control.has_value());
        CHECK(u.aero_control.has_value() == o.aero_control.has_value());
        CHECK(u.abs.has_value());
        CHECK(u.esc.has_value());
        CHECK(ice.drive_modes.has_value() == own.drive_modes.has_value());
    }
}

namespace {
struct Stop {
    double lock_s = 0;
    double peak_slip = 0;
    double distance_m = 0;
    double abs_channel = -1;
    bool fitted_abs = false, fitted_esc = false, fitted_aero = false, fitted_launch = false;
};

// A hybrid-converted car (electric drive, drive mode Eco) accelerates to ~`kmh` and brakes with a pedal step.
Stop hybrid_stop(Fixture& f, const char* car, const char* choice, double abs_channel, double kmh) {
    Stop r;
    const rg::CatalogEntry& e = f.entry(car);
    const auto setup = setup_of(car, choice);
    rg::ScopedWorkDir dir(f.ctx.work_root, std::string("t_stop_") + car);
    const auto m = rg::materialise_setup(e, f.table, f.ctx, setup, dir.path());
    INFO(m.error);
    REQUIRE(m.ok);
    rg::SessionConfig c;
    c.vehicle_json_path = m.vehicle_path;
    c.surface_table_path = kRoot + "/data/surfaces/surfaces.json";
    c.engine_map_cache_dir = f.ctx.engine_map_cache_dir;
    c.chassis_mass_kg = e.chassis.mass_kg + m.compiled.chassis_mass_delta_kg;
    c.chassis_half_extents = ps::Vec3{e.chassis.half_extents[0], e.chassis.half_extents[1], e.chassis.half_extents[2]};
    c.chassis_z_m = e.chassis.spawn_z_m;
    c.ground_tessellated = true;
    c.ground_half_extent_m = 3000;
    c.ground_half_extent_y_m = 200;
    rg::Session ses(c);
    auto& w = ses.world();
    const ps::VehicleId vid = ses.vehicle_id();
    const std::size_t nw = ses.vehicle_desc().wheels.size();
    w.set_control("assist.drive_mode", 0);
    w.set_control("gear_request", 99);
    for (int i = 0; i < 480; ++i) ses.step();
    const auto vx = [&] {
        const ps::Pose p = w.get_pose(ses.chassis_body());
        return p.orientation.inverse().rotate(w.get_motion(ses.chassis_body()).linear).x;
    };
    w.set_control("throttle", 1.0);
    for (int i = 0; i < 240 * 40 && vx() < kmh / 3.6; ++i) ses.step();
    w.set_control("throttle", 0.0);
    for (int i = 0; i < 24; ++i) ses.step();
    const auto& pt0 = w.powertrain_state(vid);
    r.fitted_abs = (pt0.assist_fitted_bits & (1u << 8)) != 0;
    r.fitted_esc = (pt0.assist_fitted_bits & (1u << 9)) != 0;
    r.fitted_aero = (pt0.assist_fitted_bits & (1u << 6)) != 0;
    r.fitted_launch = (pt0.assist_fitted_bits & (1u << 4)) != 0;
    if (abs_channel >= 0) w.set_control("assist.abs", abs_channel);
    ses.step();
    r.abs_channel = w.get_control("assist.abs");
    const double x0 = w.get_pose(ses.chassis_body()).position.x;
    for (int k = 0; k < 240 * 12; ++k) {
        w.set_control("brake", 1.0);
        ses.step();
        const double v = vx();
        double mn = 0;
        for (std::size_t i = 0; i < nw; ++i) mn = std::min(mn, w.wheel_telemetry(vid, i).slip_ratio);
        if (v > 3.0 && k > 144) {
            if (mn < -0.8) r.lock_s += 1.0 / 240;
            r.peak_slip = std::max(r.peak_slip, -mn);
        }
        if (v < 0.3) break;
    }
    r.distance_m = w.get_pose(ses.chassis_body()).position.x - x0;
    return r;
}
} // namespace

TEST_CASE("a hybrid-converted car's ABS keeps the wheels from locking and assist.abs = 0 does not", "[hybrid_electronics][abs_regression][long]") {
    Fixture f;
    for (const char* car : {"car_sedan", "car_hyper"}) {
        INFO(car);
        const Stop on = hybrid_stop(f, car, "hybrid_battery", -1, 100);
        REQUIRE(on.fitted_abs);
        CHECK(on.fitted_esc);
        CHECK(on.fitted_launch == load_desc(f.entry(car).vehicle_path).assist_unit->launch_control.has_value());
        CHECK(on.fitted_aero == (std::string(car) == "car_hyper"));
        CHECK(on.abs_channel == 1.0);
        CHECK(on.lock_s < 0.05);
        CHECK(on.peak_slip < 0.3);
        const Stop off = hybrid_stop(f, car, "hybrid_battery", 0.0, 100);
        CHECK(off.lock_s > 1.0);
        CHECK(on.distance_m < 0.95 * off.distance_m);
    }
}

TEST_CASE("hybrid assist upgrades preserve the chosen grade and work without factory electronics", "[hybrid_electronics][driver_assist_upgrades]") {
    Fixture f;
    for (const char* car : {"car_hyper", "car_runabout"}) {
        for (const char* unit : {"assists_street_abs_esc", "assists_racing_abs_esc", "assists_abs"}) {
            INFO(car << " / " << unit);
            auto setup = setup_of(car, "hybrid_battery");
            setup.values["assist_unit"] = std::string(unit);
            rg::ScopedWorkDir dir(f.ctx.work_root, "assist_choices");
            const auto m = rg::materialise_setup(f.entry(car), f.table, f.ctx, setup, dir.path());
            INFO(m.error); REQUIRE(m.ok);
            const auto d = load_desc(m.vehicle_path);
            REQUIRE(d.assist_unit);
            CHECK(d.assist_unit->launch_control.has_value() == (std::string(unit) == "assists_racing_abs_esc"));
            CHECK(d.assist_unit->has_traction_control == (std::string(unit) != "assists_abs"));
            CHECK(d.assist_unit->abs.has_value());
            if (std::string(unit) == "assists_street_abs_esc")
                CHECK(d.assist_unit->traction_control.modes_bits == 3);
        }
    }
}

TEST_CASE("launch control holds electric traction and caps torque after the power corner", "[hybrid_electronics][driver_assist_upgrades]") {
    using namespace ps::drivetrain;
    NetworkBuilder builder;
    const auto shaft = builder.add_node(0.25);
    auto net = builder.build("hybrid-launch-motor");
    BatteryDesc bd;
    Battery battery(bd);
    ElectricMotorDesc md;
    md.peak_torque_nm = 1200;
    md.max_power_w = 600000;
    md.max_rpm = 20000;
    md.inertia_kgm2 = 0.25;
    ElectricMotor motor(shaft, md, battery);
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960;
    ctx.electric_throttle = 1;
    ctx.electric_direction = 1;
    for (double omega : {200.0, 1500.0}) {
        net.set_omega(shaft, omega);
        ctx.launch_hold_clutch = false;
        ctx.launch_throttle_ceiling = 1;
        const double full = motor.drive_claim_j(ctx, net);
        REQUIRE(full > 0);
        ctx.launch_throttle_ceiling = 0.25;
        CHECK(motor.drive_claim_j(ctx, net) == Catch::Approx(full * 0.25).epsilon(0.02));
        ctx.launch_hold_clutch = true;
        CHECK(motor.drive_claim_j(ctx, net) == 0);
    }
    md.crank_assist = true;
    ElectricMotor starter(shaft, md, battery);
    ctx.electric_charge = -1;
    ctx.launch_throttle_ceiling = 1;
    CHECK(starter.drive_claim_j(ctx, net) > 0); // launch staging cannot prevent ICE start
}

TEST_CASE("hybrid hypercar Launch Control arms and launches with manual and DCT gearboxes", "[hybrid_electronics][driver_assist_launch][long]") {
    Fixture f;
    for (const std::string& box : {std::string{}, std::string{"hyper_7dct"}}) {
        INFO(box);
        auto setup = setup_of("car_hyper", "hybrid_battery");
        if (!box.empty()) setup.values["gearbox_install"] = box;
        rg::ScopedWorkDir dir(f.ctx.work_root, "hybrid_launch");
        const auto m = rg::materialise_setup(f.entry("car_hyper"), f.table, f.ctx, setup, dir.path());
        INFO(m.error); REQUIRE(m.ok);
        const auto& e = f.entry("car_hyper");
        rg::SessionConfig c;
        c.vehicle_json_path = m.vehicle_path;
        c.surface_table_path = kRoot + "/data/surfaces/surfaces.json";
        c.engine_map_cache_dir = f.ctx.engine_map_cache_dir;
        c.chassis_mass_kg = e.chassis.mass_kg + m.compiled.chassis_mass_delta_kg;
        c.chassis_half_extents = ps::Vec3{e.chassis.half_extents[0], e.chassis.half_extents[1], e.chassis.half_extents[2]};
        c.chassis_z_m = e.chassis.spawn_z_m;
        rg::Session ses(c);
        ses.set_control("electric.ready", 1);
        ses.set_control("ignition", 1);
        ses.set_control("assist.auto_clutch", 1);
        ses.set_control("assist.auto_shift", 1);
        ses.set_control("gear_request", 99);
        ses.set_control("assist.drive_mode", 2);
        ses.set_control("brake", 1);
        auto tick = [&] { while (!ses.try_step()) {} };
        auto pt = [&]() -> const ps::drivetrain::PowertrainSnapshot& { return ses.world().powertrain_state(ses.vehicle_id()); };
        for (int i = 0; i < 720; ++i) tick();
        CHECK((pt().assist_fitted_bits & (1u << 4)) != 0);
        INFO("lc policy=" << ses.vehicle_desc().drive_modes->params[2].launch_control_allowed
             << " default=" << ses.vehicle_desc().drive_modes->params[2].launch_control_default.value_or(false));
        ses.set_control("assist.drive_mode", 1);
        tick();
        ses.set_control("assist.drive_mode", 2);
        for (int i = 0; i < 24; ++i) tick();
        CHECK(ses.get_control("assist.launch_control") == 1);
        ses.set_control("throttle", 1);
        bool armed = false;
        for (int i = 0; i < 960; ++i) { tick(); armed |= pt().launch_phase == 1; }
        INFO("mode=" << pt().drive_mode_effective << " lc=" << ses.get_control("assist.launch_control")
             << " auto=" << ses.get_control("assist.auto_clutch") << " engine=" << int(pt().engine_state)
             << " rpm=" << pt().engine_rpm << " phase=" << int(pt().launch_phase)
             << " speed=" << ses.world().get_motion(ses.chassis_body()).linear.length());
        REQUIRE(armed);
        ses.set_control("brake", 0);
        bool launching = false;
        for (int i = 0; i < 480; ++i) { tick(); launching |= pt().launch_phase == 2 || pt().launch_phase == 3; }
        CHECK(launching);
        CHECK(ses.world().get_motion(ses.chassis_body()).linear.length() > 1);
        ses.set_control("assist.drive_mode", 0);
        for (int i = 0; i < 24; ++i) tick();
        CHECK((pt().assist_active_bits & (1u << 4)) == 0);
    }
}
