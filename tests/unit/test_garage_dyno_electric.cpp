// test_garage_dyno_electric.cpp - the garage dyno for electric, fuel-cell and hybrid cars (2026-10-19, owner: "start the
// hybrid dyno"). The chart is physics_sim's run_powertrain_dyno (the car's own Powertrain stepped on a chassis dyno); these
// tests hold it against an INDEPENDENT computation from the loaded physics descriptors: the closed-form motor law
// T = min(T_peak, P_max / (omega + h T_peak / J)), the pack's discharge limit split over the motors that want it, the
// motor efficiency, and the engine's own WOT curve - plus physics' pure envelope functions (motor_full_load_point,
// battery_discharge_budget_w, fuel_cell_supply_w), which are a separate code path from the stepped dyno.
// Every topology the garage offers: the catalogue EV (car_electric), the fuel-cell conversion, the battery hybrid (sedan
// and hypercar), automatic / dual-clutch / CVT hybrids (held gear, 2026-10-21), and combustion cars unchanged.
#include "rg/garage_dyno.h"
#include "rg/engine_store.h"
#include "rg/ev_motor.h"
#include "rg/gearbox_catalog.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"
#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/electric_envelope.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <thread>
#include <variant>
namespace {
namespace fs = std::filesystem;
namespace pd = ps::drivetrain;
using Catch::Approx;
const std::string kRoot = RG_SOURCE_DIR;
constexpr double kPi = 3.14159265358979323846;
constexpr double kH = 1.0 / 960.0; // the car's substep

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    rg::DynoReference reference;
    rg::ScopedWorkDir isolated_root{fs::temp_directory_path().generic_string(), "rg_dyno_electric_tests"};
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
        auto m = rg::EvMotorCatalog::open(kRoot + "/data/electric_motors/motor_catalog.json", &err);
        INFO(err);
        REQUIRE(m != nullptr);
        ctx.ev_motors = std::shared_ptr<const rg::EvMotorCatalog>(std::move(m));
        auto r = rg::load_dyno_reference(kRoot + "/data/garage/dyno_reference.json", &err);
        INFO(err);
        REQUIRE(r.has_value());
        reference = *r;
    }
    const rg::CatalogEntry& entry(const std::string& id) const {
        const rg::CatalogEntry* e = catalog.find(id);
        REQUIRE(e != nullptr);
        return *e;
    }
    // The vehicle exactly as the dyno worker loads it.
    ps::vehicle::VehicleDesc load(const std::string& car, const std::map<std::string, rg::SetupValue>& values, ps::jobs::JobSystem* pool) {
        rg::VehicleSetup setup;
        setup.vehicle_id = car;
        setup.values = values;
        rg::ScopedWorkDir dir(ctx.work_root, "load_" + car);
        const auto m = rg::materialise_setup(entry(car), table, ctx, setup, dir.path());
        INFO(m.error);
        REQUIRE(m.ok);
        ps::io::EngineMapOptions maps;
        maps.cache_dir = ctx.engine_map_cache_dir;
        maps.pool = pool;
        return rg::load_vehicle(m.vehicle_path, maps);
    }
};

template <class T> std::vector<std::pair<std::string, T>> components(const ps::vehicle::VehicleDesc& v) {
    std::vector<std::pair<std::string, T>> out;
    for (const auto& c : v.powertrain.components)
        if (const auto* p = std::get_if<T>(&c.params)) out.emplace_back(c.id, *p);
    return out;
}
// The motor law at full pedal, written out from the descriptor (not physics code): the torque rating, the power corner at
// the rotor-inertia-predicted speed, and the electrical grant `budget_w` (power) scaled by the efficiency.
double closed_form_torque(const pd::ElectricMotorDesc& m, double rpm, double budget_w) {
    if (rpm >= m.max_rpm) return 0.0;
    const double omega = rpm * kPi / 30.0;
    double t = m.peak_torque_nm;
    const double projected = std::max(1.0, omega + kH * t / m.inertia_kgm2);
    t = std::min(t, m.max_power_w / projected);
    const double wanted_w = t * projected / m.efficiency;
    if (wanted_w > budget_w) t *= budget_w / wanted_w;
    return t;
}
std::string ordinal_of(int n) { return std::to_string(n) + (n % 10 == 1 && n != 11 ? "st" : n % 10 == 2 && n != 12 ? "nd" : n % 10 == 3 && n != 13 ? "rd" : "th"); }
rg::DynoResult wait(rg::GarageDyno& dyno) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
    while (dyno.result().busy && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto r = dyno.result();
    REQUIRE_FALSE(r.busy);
    return r;
}
std::string write_temp(const std::string& name, const std::string& text) {
    const auto path = (fs::temp_directory_path() / ("rg_dyno_ref_" + name + ".json")).generic_string();
    std::ofstream(path, std::ios::binary) << text;
    return path;
}
} // namespace

TEST_CASE("dyno reference: the shipped file and every rejection", "[garage_dyno][electric][io]") {
    std::string err;
    const auto r = rg::load_dyno_reference(kRoot + "/data/garage/dyno_reference.json", &err);
    INFO(err);
    REQUIRE(r.has_value());
    CHECK(r->battery_soc == 0.8);
    CHECK(r->drive_mode == "normal");
    CHECK(r->drive_mode_index == 1);
    CHECK(r->points == 33);
    CHECK(r->coarse_points == 9);
    CHECK(r->ambient_temperature_k == 288.15);
    CHECK(std::find(r->electrified_setup_options.begin(), r->electrified_setup_options.end(), "gear_ratios") != r->electrified_setup_options.end());
    // Each case breaks exactly one thing of the valid text.
    CHECK(r->fuel_cell_soc_below_target == 0.05);
    const std::string valid = R"({"format":"rg.garage_dyno_reference/1","source":"t","battery_soc":0.8,"fuel_cell_soc_below_target":0.05,"hydrogen_fraction":1,
        "drive_mode":"normal","pedal":1,"reference_gear_ratio":1,"points":33,"coarse_points":9,"motor_rpm_min":250,
        "motor_rpm_top_fraction":0.99,"hybrid_clutch_margin_rpm":25,"ambient":{"pressure_pa":101325,"temperature_k":288.15},"electrified_setup_options":[]})";
    CHECK(rg::load_dyno_reference(write_temp("valid", valid), &err).has_value());
    const auto rejects = [&](const std::string& name, const std::string& from, const std::string& to, const std::string& message) {
        std::string text = valid;
        const auto at = text.find(from);
        REQUIRE(at != std::string::npos);
        text.replace(at, from.size(), to);
        std::string e;
        INFO(name);
        CHECK_FALSE(rg::load_dyno_reference(write_temp(name, text), &e).has_value());
        CHECK_THAT(e, Catch::Matchers::ContainsSubstring(message));
    };
    rejects("format", "rg.garage_dyno_reference/1", "rg.garage_dyno_reference/2", "\"format\"");
    rejects("unknown", "\"pedal\":1,", "\"pedal\":1,\"stack_power_kw\":5,", "unknown key \"stack_power_kw\"");
    rejects("soc", "\"battery_soc\":0.8", "\"battery_soc\":1.5", "battery_soc");
    rejects("soc_missing", "\"battery_soc\":0.8,", "", "battery_soc");
    rejects("fc_soc", "\"fuel_cell_soc_below_target\":0.05", "\"fuel_cell_soc_below_target\":0.7", "fuel_cell_soc_below_target");
    rejects("points", "\"points\":33", "\"points\":32", "\"points\"");
    rejects("coarse", "\"coarse_points\":9", "\"coarse_points\":65", "coarse_points");
    rejects("mode", "\"drive_mode\":\"normal\"", "\"drive_mode\":\"turbo\"", "drive_mode");
    rejects("ambient_key", "\"temperature_k\":288.15", "\"temperature_k\":288.15,\"humidity\":0.5", "ambient.humidity");
    rejects("ambient_missing", "\"ambient\":{\"pressure_pa\":101325,\"temperature_k\":288.15},", "", "ambient");
    rejects("options", "\"electrified_setup_options\":[]", "\"electrified_setup_options\":\"gear_ratios\"", "electrified_setup_options");
    rejects("source", "\"source\":\"t\",", "", "source");
    CHECK_FALSE(rg::load_dyno_reference(kRoot + "/data/garage/no_such_file.json", &err).has_value());
}

TEST_CASE("dyno setup: combustion cars keep the engine-only key, electrified cars add the gearing", "[garage_dyno][electric]") {
    Fixture f;
    const auto& sedan = f.entry("car_sedan");
    rg::VehicleSetup working;
    working.vehicle_id = "car_sedan";
    working.values["gear_ratios"] = std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.1};
    working.values["final_drive"] = 1.1;
    working.values["engine_rev_limit"] = 0.95;
    working.values["brake_force"] = 1.2;
    // The filter the binding used before the electric dyno: engine and turbo options only.
    rg::VehicleSetup old_filter;
    old_filter.vehicle_id = "car_sedan";
    for (const auto& option : f.table.options)
        if (option.file == "engine" || option.file == "turbo" || option.part_format == "physics_sim.engine/1" ||
            option.part_format == "physics_sim.turbo_configuration/1")
            if (auto v = working.values.find(option.id); v != working.values.end()) old_filter.values.emplace(*v);
    CHECK(rg::setup_to_json(rg::dyno_setup(f.table, sedan, working, &f.reference)) == rg::setup_to_json(old_filter));
    CHECK(rg::setup_to_json(rg::dyno_setup(f.table, sedan, working, nullptr)) == rg::setup_to_json(old_filter));
    auto hybrid = working;
    hybrid.values["powertrain_install"] = std::string("hybrid_battery");
    const auto with = rg::dyno_setup(f.table, sedan, hybrid, &f.reference);
    CHECK(with.values.count("powertrain_install") == 1);
    CHECK(with.values.count("gear_ratios") == 1);
    CHECK(with.values.count("final_drive") == 1);
    CHECK(with.values.count("engine_rev_limit") == 1);
    CHECK(with.values.count("brake_force") == 0);
    // The catalogue EV is electrified without any conversion chosen.
    rg::VehicleSetup ev;
    ev.vehicle_id = "car_electric";
    ev.values["final_drive"] = 0.9;
    CHECK(rg::dyno_setup(f.table, f.entry("car_electric"), ev, &f.reference).values.count("final_drive") == 1);
    // A car converted back to combustion keeps the conversion (it decides what is materialised) but not the gearing.
    auto back = working;
    back.values["powertrain_install"] = std::string("ice");
    const auto ice = rg::dyno_setup(f.table, sedan, back, &f.reference);
    CHECK(ice.values.count("powertrain_install") == 1);
    CHECK(ice.values.count("gear_ratios") == 0);
}

TEST_CASE("dyno EV: car_electric per-motor shares against the motor law and the pack", "[garage_dyno][electric]") {
    Fixture f;
    ps::jobs::JobSystem pool(4);
    const auto vehicle = f.load("car_electric", {}, &pool);
    const auto plan = rg::plan_electrified_dyno(vehicle, f.reference);
    REQUIRE(plan.has_value());
    CHECK(plan->topology == rg::DynoTopology::Electric);
    CHECK(plan->axis == rg::DynoAxis::MotorRpm);
    CHECK(plan->unsupported_reason.empty());
    REQUIRE(plan->labels == std::vector<std::string>{"front_motor", "rear_motor"});
    CHECK_THAT(plan->caption, Catch::Matchers::ContainsSubstring("battery 80 %"));
    const auto motors = components<pd::ElectricMotorDesc>(vehicle);
    const auto battery = components<pd::BatteryDesc>(vehicle).at(0).second;
    REQUIRE(motors.size() == 2);
    REQUIRE(plan->axis_points.front() == 250.0);
    REQUIRE(plan->axis_points.back() == Approx(motors[0].second.max_rpm * 0.99));
    const auto points = rg::measure_electrified_dyno(vehicle, f.reference, *plan, &pool, nullptr, [](const auto&, int) {});
    REQUIRE(points.size() == 33);
    // The pack's limit split over the two motors that want the same power (physics' proportional allocation).
    const double pack_w = pd::battery_discharge_budget_w(battery, 0.8, kH);
    CHECK(pack_w == Approx(battery.max_discharge_w));
    double peak = 0;
    for (const auto& p : points) {
        INFO("rpm " << p.rpm);
        REQUIRE(p.motors.size() == 2);
        for (std::size_t i = 0; i < 2; ++i) {
            const auto& m = motors[i].second;
            const double expected = closed_form_torque(m, p.rpm, pack_w / 2);
            CHECK(p.motors[i].torque_nm == Approx(expected).epsilon(0.005).margin(0.5));
            CHECK(p.motors[i].torque_nm == Approx(pd::motor_full_load_point(m, p.rpm, pack_w / 2, kH).torque_nm).epsilon(0.005).margin(0.5));
            CHECK_FALSE(p.motors[i].battery_limited); // 2 x 300 kW / 0.95 = 631.6 kW < 700 kW
        }
        CHECK(p.electric_power_kw == Approx(p.motors[0].power_kw + p.motors[1].power_kw));
        CHECK(p.electric_torque_nm == Approx(p.motors[0].torque_nm + p.motors[1].torque_nm).epsilon(0.005)); // equal 9:1 reductions
        // At the hubs: through the 9:1 diffs, each mesh P_out = P_in / (2 - eta), eta 0.98.
        CHECK(p.power_kw == Approx(p.electric_power_kw / (2 - 0.98)).epsilon(0.01).margin(0.5));
        CHECK(p.engine_power_kw == 0);
        CHECK(p.fuel_cell_power_kw == 0);
        peak = std::max(peak, p.power_kw);
    }
    CHECK(peak > 570);
    CHECK(peak < 600);
}

TEST_CASE("dyno fuel-cell EV: the stack's share follows the reference state of charge", "[garage_dyno][electric]") {
    Fixture f;
    ps::jobs::JobSystem pool(4);
    const auto vehicle = f.load("car_sedan", {{"powertrain_install", std::string("fuel_cell_ev")}}, &pool);
    const auto battery = components<pd::BatteryDesc>(vehicle).at(0).second;
    const auto cell = components<pd::FuelCellDesc>(vehicle).at(0).second;
    const auto motors = components<pd::ElectricMotorDesc>(vehicle);
    REQUIRE(motors.size() == 2);
    // Owner 2026-10-20: the fuel-cell reference holds the pack at the stack's target_soc minus fuel_cell_soc_below_target
    // (shipped 0.05: target 0.8 -> 0.75, full stack power), not at battery_soc 0.8 (the target itself: 0 kW). Recorded
    // sabotage: planning fuel-cell cars at reference.battery_soc again fails the shipped-file checks (soc 0.8, caption 80 %).
    {
        const auto plan = rg::plan_electrified_dyno(vehicle, f.reference);
        REQUIRE(plan.has_value());
        CHECK(plan->battery_soc == Approx(cell.target_soc - 0.05));
        CHECK_THAT(plan->caption, Catch::Matchers::ContainsSubstring("battery 75 %"));
        CHECK(pd::fuel_cell_supply_w(cell, battery, plan->battery_soc, cell.hydrogen_capacity_kg, kH) == Approx(cell.max_power_w));
        // a battery EV keeps battery_soc
        const auto ev = rg::plan_electrified_dyno(f.load("car_sedan", {{"powertrain_install", std::string("battery_ev")}}, &pool), f.reference);
        REQUIRE(ev.has_value());
        CHECK(ev->battery_soc == f.reference.battery_soc);
    }
    for (const double below : {0.0, 0.2}) {
        const double soc = cell.target_soc - below;
        INFO("soc " << soc);
        auto reference = f.reference;
        reference.fuel_cell_soc_below_target = below;
        reference.battery_soc = 0.3; // ignored for a fuel-cell car
        const auto plan = rg::plan_electrified_dyno(vehicle, reference);
        REQUIRE(plan.has_value());
        CHECK(plan->battery_soc == Approx(soc));
        CHECK(plan->topology == rg::DynoTopology::FuelCellElectric);
        CHECK_THAT(plan->caption, Catch::Matchers::ContainsSubstring("H₂"));
        const auto points = rg::measure_electrified_dyno(vehicle, reference, *plan, &pool, nullptr, [](const auto&, int) {});
        const double stack_w = pd::fuel_cell_supply_w(cell, battery, soc, cell.hydrogen_capacity_kg * reference.hydrogen_fraction, kH);
        // Independent: the stack runs only below its target SOC (fraction (target - soc) / 0.05), capped by its rating.
        const double expected_stack = cell.max_power_w * std::clamp((cell.target_soc - soc) / 0.05, 0.0, 1.0);
        CHECK(stack_w == Approx(expected_stack).margin(1));
        const double supply_w = battery.max_discharge_w + stack_w;
        for (const auto& p : points) {
            INFO("rpm " << p.rpm);
            CHECK(p.fuel_cell_power_kw == Approx(stack_w / 1000).margin(0.5));
            double expected = 0;
            for (const auto& [id, m] : motors) expected += closed_form_torque(m, p.rpm, supply_w / 2) * p.rpm * kPi / 30.0;
            CHECK(p.electric_power_kw == Approx(expected / 1000).epsilon(0.01).margin(0.5));
        }
        const auto& top = points.back();
        CHECK(top.battery_limited); // 2 x 300 kW wanted, 150 (+ stack) kW supplied
        CHECK(top.electric_power_kw == Approx(supply_w * motors[0].second.efficiency / 1000).epsilon(0.01));
    }
}

TEST_CASE("dyno hybrid sedan: engine at its WOT curve plus the pack-limited motor in the reference gear", "[garage_dyno][electric][hybrid]") {
    Fixture f;
    ps::jobs::JobSystem pool(4);
    const auto vehicle = f.load("car_sedan", {{"powertrain_install", std::string("hybrid_battery")}}, &pool);
    const auto plan = rg::plan_electrified_dyno(vehicle, f.reference);
    REQUIRE(plan.has_value());
    CHECK(plan->topology == rg::DynoTopology::Hybrid);
    CHECK(plan->axis == rg::DynoAxis::EngineRpm);
    const auto gearbox = components<pd::GearboxDesc>(vehicle).at(0).second;
    // The gear nearest 1:1 on the sedan 6MT (3.54 2.05 1.40 1.08 0.87 0.72) is 4th.
    CHECK(plan->gear == 4);
    CHECK_THAT(plan->caption, Catch::Matchers::ContainsSubstring("Normal mode"));
    CHECK_THAT(plan->caption, Catch::Matchers::ContainsSubstring("4th"));
    REQUIRE(plan->labels == std::vector<std::string>{"conversion_motor"});
    const auto engine = components<pd::TorqueMapEngineDesc>(vehicle).at(0).second;
    const auto battery = components<pd::BatteryDesc>(vehicle).at(0).second;
    pd::ElectricMotorDesc motor;
    for (const auto& [id, m] : components<pd::ElectricMotorDesc>(vehicle)) if (id == "conversion_motor") motor = m;
    double reduction = 0;
    for (const auto& [id, g] : components<pd::GearPairDesc>(vehicle)) if (id == "conversion_reduction") reduction = g.ratio;
    REQUIRE(reduction == 2.2);
    // From the auto-clutch's lock-up speed (+ the reference margin) to the fuel cut's restore speed.
    pd::ManualTcuDesc tcu;
    for (const auto& c : vehicle.controllers) if (const auto* t = std::get_if<pd::ManualTcuDesc>(&c.params)) tcu = *t;
    CHECK(plan->axis_points.front() == std::max(engine.idle_rpm, tcu.auto_clutch.rolling_reengage_rpm + f.reference.hybrid_clutch_margin_rpm));
    CHECK(plan->axis_points.back() == engine.limiter.rpm - engine.limiter.hysteresis_rpm - 1);
    std::vector<rg::DynoPoint> provisional;
    const auto points = rg::measure_electrified_dyno(vehicle, f.reference, *plan, &pool, nullptr,
        [&](const auto& partial, int done) { if (done == 9) provisional = partial; });
    REQUIRE(points.size() == 33);
    REQUIRE(provisional.size() == 9);
    double peak = 0, peak_engine = 0, peak_electric = 0;
    for (const auto& p : points) {
        INFO("rpm " << p.rpm);
        CHECK(p.engine_running);
        CHECK(p.gear == 4);
        CHECK(p.engine_rpm == Approx(p.rpm).epsilon(0.001)); // the clutch is locked at every point
        CHECK(p.engine_torque_nm == Approx(engine.wot_torque_nm_vs_rpm.evaluate(p.rpm)).epsilon(0.01));
        // The motor turns at the gearbox output speed times its reduction; only it draws (the crank generator idles).
        const double motor_rpm = p.rpm / gearbox.forward_ratios[3] * reduction;
        REQUIRE(p.motors.size() == 1);
        CHECK(p.motors[0].rpm == Approx(motor_rpm).epsilon(0.002));
        CHECK(p.motors[0].torque_nm == Approx(closed_form_torque(motor, motor_rpm, battery.max_discharge_w)).epsilon(0.01).margin(1));
        // Shares are referred to the axis (crank) shaft: power / crank omega.
        CHECK(p.electric_torque_nm == Approx(p.electric_power_kw * 30000 / (kPi * p.rpm)).epsilon(1e-9));
        // System at the hubs: below the sum of the sources (gear losses), above 90 % of it.
        CHECK(p.power_kw < p.engine_power_kw + p.electric_power_kw);
        CHECK(p.power_kw > 0.9 * (p.engine_power_kw + p.electric_power_kw));
        peak = std::max(peak, p.power_kw); peak_engine = std::max(peak_engine, p.engine_power_kw);
        peak_electric = std::max(peak_electric, p.electric_power_kw);
    }
    // 150 kW pack x 0.95 = 142.5 kW at the motor shaft, pack-limited above ~3000 motor rpm.
    CHECK(peak_electric == Approx(142.5).epsilon(0.01));
    CHECK(points.back().battery_limited);
    CHECK(peak_engine == Approx(190.07).epsilon(0.01)); // N52 WOT 288 N*m at 6600 rpm
    CHECK(peak > 310);
    // Eco: the supervisor never couples the engine; the curve is the motor alone.
    auto eco = f.reference;
    eco.drive_mode = "eco";
    eco.drive_mode_index = 0;
    const auto eco_plan = rg::plan_electrified_dyno(vehicle, eco);
    const auto eco_points = rg::measure_electrified_dyno(vehicle, eco, *eco_plan, &pool, nullptr, [](const auto&, int) {});
    for (const auto& p : eco_points) {
        CHECK_FALSE(p.engine_running);
        CHECK(p.engine_power_kw == Approx(0).margin(0.5));
    }
}

TEST_CASE("dyno determinism: 1 and 4 workers give the same points", "[garage_dyno][electric][determinism]") {
    Fixture f;
    ps::jobs::JobSystem one(1), four(4);
    const auto vehicle = f.load("car_sedan", {{"powertrain_install", std::string("hybrid_battery")}}, &four);
    const auto plan = rg::plan_electrified_dyno(vehicle, f.reference);
    const auto a = rg::measure_electrified_dyno(vehicle, f.reference, *plan, &one, nullptr, [](const auto&, int) {});
    const auto b = rg::measure_electrified_dyno(vehicle, f.reference, *plan, &four, nullptr, [](const auto&, int) {});
    REQUIRE(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i].rpm == b[i].rpm);
        CHECK(a[i].power_kw == b[i].power_kw);
        CHECK(a[i].torque_nm == b[i].torque_nm);
        CHECK(a[i].engine_power_kw == b[i].engine_power_kw);
        CHECK(a[i].electric_power_kw == b[i].electric_power_kw);
    }
}

TEST_CASE("dyno hybrid sedan: automatic, dual-clutch and CVT gearboxes are measured in a held gear", "[garage_dyno][electric][hybrid]") {
    // physics_sim holds the gear (AutoTransmission::set_dyno_hold: TCU frozen, starting clutch / converter locked, a CVT's
    // variator held at the virtual step's ratio). Sabotage: dropping the physics hold (set_dyno_gear_hold on an auto box) lets
    // the TCU pick the gear, and the crank / motor speeds below leave the axis relation (verified in physics_sim A1-A4).
    Fixture f;
    ps::jobs::JobSystem pool(4);
    for (const char* box : {"auto_8at", "dct_7", "cvt_converter", "cvt_clutch"}) {
        INFO(box);
        std::map<std::string, rg::SetupValue> values = {{"powertrain_install", std::string("hybrid_battery")}, {"gearbox_install", std::string(box)}};
        // A CVT is rated 250 N*m, below the stock hybrid's N52 (the garage refuses that pairing): it gets the 2.0 L four.
        if (std::string(box).rfind("cvt", 0) == 0) values["engine_install"] = std::string("default:i4_2l_na");
        const auto vehicle = f.load("car_sedan", values, &pool);
        const auto plan = rg::plan_electrified_dyno(vehicle, f.reference);
        REQUIRE(plan.has_value());
        INFO(plan->unsupported_reason);
        CHECK(plan->unsupported_reason.empty());
        CHECK(plan->topology == rg::DynoTopology::Hybrid);
        CHECK(plan->axis == rg::DynoAxis::EngineRpm);
        const auto gearbox = components<pd::GearboxDesc>(vehicle).at(0).second;
        const bool cvt = gearbox.type == pd::GearboxType::Cvt;
        REQUIRE(plan->gear >= 1);
        REQUIRE(plan->gear <= static_cast<int>(gearbox.forward_ratios.size()));
        // The gear whose ratio is nearest the reference ratio.
        for (int g = 1; g <= static_cast<int>(gearbox.forward_ratios.size()); ++g)
            CHECK(std::abs(gearbox.forward_ratios[static_cast<std::size_t>(plan->gear - 1)] - f.reference.reference_gear_ratio) <=
                  std::abs(gearbox.forward_ratios[static_cast<std::size_t>(g - 1)] - f.reference.reference_gear_ratio) + 1e-12);
        CHECK_THAT(plan->caption, Catch::Matchers::ContainsSubstring(cvt ? "at ratio" : ordinal_of(plan->gear)));
        const auto engine = components<pd::TorqueMapEngineDesc>(vehicle).at(0).second;
        const auto battery = components<pd::BatteryDesc>(vehicle).at(0).second;
        pd::ElectricMotorDesc motor;
        for (const auto& [id, m] : components<pd::ElectricMotorDesc>(vehicle)) if (id == "conversion_motor") motor = m;
        double reduction = 0;
        for (const auto& [id, g] : components<pd::GearPairDesc>(vehicle)) if (id == "conversion_reduction") reduction = g.ratio;
        REQUIRE(reduction == 2.2);
        // A starting-clutch box starts above its anti-stall rpm (the clutch opens below it), the others at idle.
        CHECK(plan->axis_points.front() >= engine.idle_rpm);
        CHECK(plan->axis_points.back() == engine.limiter.rpm - engine.limiter.hysteresis_rpm - 1);
        const auto points = rg::measure_electrified_dyno(vehicle, f.reference, *plan, &pool, nullptr, [](const auto&, int) {});
        REQUIRE(points.size() == 33);
        for (const auto& p : points) {
            INFO("rpm " << p.rpm);
            CHECK(p.converged);
            CHECK(p.engine_running);
            CHECK(p.gear == (cvt ? 1 : plan->gear));
            CHECK(p.engine_rpm == Approx(p.rpm).epsilon(0.001)); // clutch / converter locked at every point
            CHECK(p.engine_torque_nm == Approx(engine.wot_torque_nm_vs_rpm.evaluate(p.rpm)).epsilon(0.01));
            const double motor_rpm = p.rpm / gearbox.forward_ratios[static_cast<std::size_t>(plan->gear - 1)] * reduction;
            REQUIRE(p.motors.size() == 1);
            CHECK(p.motors[0].rpm == Approx(motor_rpm).epsilon(0.002));
            CHECK(p.motors[0].torque_nm == Approx(closed_form_torque(motor, motor_rpm, battery.max_discharge_w)).epsilon(0.01).margin(1));
            CHECK(p.power_kw < p.engine_power_kw + p.electric_power_kw);
            CHECK(p.power_kw > 0.9 * (p.engine_power_kw + p.electric_power_kw));
        }
    }
    CHECK_FALSE(rg::plan_electrified_dyno(f.load("car_sedan", {}, &pool), f.reference).has_value()); // combustion: not electrified
}

TEST_CASE("dyno worker: EV, DCT hybrid and the unchanged combustion sweep", "[garage_dyno][electric]") {
    Fixture f;
    auto reference = std::make_shared<const rg::DynoReference>(f.reference);
    rg::GarageDyno dyno;
    const auto request = [&](const std::string& car, std::map<std::string, rg::SetupValue> values, const std::string& key) {
        rg::DynoRequest r;
        r.entry = f.entry(car);
        r.table = f.table;
        r.context = f.ctx;
        r.setup.vehicle_id = car;
        r.setup.values = std::move(values);
        r.key = key;
        r.reference = reference;
        return r;
    };
    dyno.request(request("car_electric", {}, "ev"));
    auto ev = wait(dyno);
    INFO(ev.error);
    REQUIRE(ev.error.empty());
    CHECK(ev.topology == rg::DynoTopology::Electric);
    CHECK(ev.points.size() == 33);
    CHECK(ev.motor_labels.size() == 2);
    CHECK_FALSE(ev.caption.empty());
    dyno.request(request("car_sedan", {{"powertrain_install", std::string("hybrid_battery")}, {"gearbox_install", std::string("dct_7")}}, "dct"));
    auto dct = wait(dyno);
    CHECK(dct.error.empty());
    CHECK(dct.unsupported_reason.empty()); // dual-clutch boxes are measured since 2026-10-21 (held gear)
    CHECK(dct.topology == rg::DynoTopology::Hybrid);
    CHECK(dct.points.size() == 33);
    CHECK(dct.reference_gear > 0);
    CHECK(dct.key == "dct");
    // Combustion: the classic crank sweep, the WOT curve exactly at the 33-point grid (as before the electric dyno).
    dyno.request(request("car_sedan", {}, "ice"));
    auto ice = wait(dyno);
    REQUIRE(ice.error.empty());
    CHECK(ice.topology == rg::DynoTopology::Combustion);
    CHECK(ice.caption.empty());
    CHECK(ice.unsupported_reason.empty());
    ps::io::EngineMapOptions maps;
    const auto sedan = rg::load_vehicle(f.entry("car_sedan").vehicle_path, maps);
    const auto engine = components<pd::TorqueMapEngineDesc>(sedan).at(0).second;
    REQUIRE(ice.points.size() == 33);
    for (int i = 0; i < 33; ++i) {
        const double rpm = engine.idle_rpm + (engine.limiter.rpm - 1.0 - engine.idle_rpm) * i / 32.0;
        CHECK(ice.points[static_cast<std::size_t>(i)].rpm == rpm);
        CHECK(ice.points[static_cast<std::size_t>(i)].torque_nm == engine.wot_torque_nm_vs_rpm.evaluate(rpm));
        CHECK(ice.points[static_cast<std::size_t>(i)].motors.empty());
    }
    // Without the reference file an electrified car is an error naming the file; combustion cars do not need it.
    auto no_ref = request("car_electric", {}, "ev_no_ref");
    no_ref.reference = nullptr;
    dyno.request(no_ref);
    auto missing = wait(dyno);
    CHECK_THAT(missing.error, Catch::Matchers::ContainsSubstring("dyno_reference.json"));
}

TEST_CASE("dyno hybrid hypercar: twin-turbo V8 plus the pack-limited motor", "[garage_dyno][electric][hybrid][long]") {
    Fixture f;
    ps::jobs::JobSystem pool(std::max(1u, std::thread::hardware_concurrency() - 2));
    const auto vehicle = f.load("car_hyper", {{"powertrain_install", std::string("hybrid_battery")}}, &pool);
    const auto plan = rg::plan_electrified_dyno(vehicle, f.reference);
    REQUIRE(plan.has_value());
    CHECK(plan->topology == rg::DynoTopology::Hybrid);
    const auto t0 = std::chrono::steady_clock::now();
    const auto points = rg::measure_electrified_dyno(vehicle, f.reference, *plan, &pool, nullptr, [](const auto&, int) {});
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    double peak = 0, peak_engine = 0, peak_electric = 0;
    for (const auto& p : points) {
        peak = std::max(peak, p.power_kw); peak_engine = std::max(peak_engine, p.engine_power_kw);
        peak_electric = std::max(peak_electric, p.electric_power_kw);
    }
    WARN("hyper hybrid gear " << plan->gear << ": system " << peak << " kW, engine " << peak_engine << " kW, electric "
         << peak_electric << " kW, " << seconds << " s");
    CHECK(peak_engine > 900);
    CHECK(peak_electric == Approx(142.5).epsilon(0.01));
    CHECK(peak > peak_engine);
}

// Hidden: the per-topology numbers the chart shows (peak system / engine / electric / stack power and the time per sweep).
TEST_CASE("dyno electric report", "[.][diag][garage_dyno][electric]") {
    Fixture f;
    ps::jobs::JobSystem pool(10);
    const std::vector<std::pair<std::string, std::string>> cases = {
        {"car_electric", ""}, {"car_sedan", "battery_ev"}, {"car_sedan", "fuel_cell_ev"}, {"car_sedan", "hybrid_battery"},
        {"car_sedan", "hybrid_dual_hydrogen"}, {"car_hyper", "hybrid_battery"}};
    for (const auto& [car, swap] : cases) {
        std::map<std::string, rg::SetupValue> values;
        if (!swap.empty()) values["powertrain_install"] = swap;
        const auto vehicle = f.load(car, values, &pool);
        const auto plan = rg::plan_electrified_dyno(vehicle, f.reference);
        REQUIRE(plan.has_value());
        const auto t0 = std::chrono::steady_clock::now();
        const auto points = rg::measure_electrified_dyno(vehicle, f.reference, *plan, &pool, nullptr, [](const auto&, int) {});
        const double ms = 1000 * std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const rg::DynoPoint* best = &points.front();
        double pe = 0, pel = 0, pfc = 0;
        for (const auto& p : points) {
            if (p.power_kw > best->power_kw) best = &p;
            pe = std::max(pe, p.engine_power_kw); pel = std::max(pel, p.electric_power_kw); pfc = std::max(pfc, p.fuel_cell_power_kw);
        }
        WARN(car << " + " << (swap.empty() ? "stock" : swap) << " [" << rg::to_string(plan->topology) << "] " << plan->caption
             << ": peak system " << best->power_kw << " kW at " << best->rpm << " rpm (engine " << best->engine_power_kw
             << ", electric " << best->electric_power_kw << ", stack " << best->fuel_cell_power_kw << "); max engine " << pe
             << ", max electric " << pel << ", max stack " << pfc << "; low end " << points.front().rpm << " rpm "
             << points.front().power_kw << " kW / " << points.front().torque_nm << " N*m; " << points.size() << " points " << ms << " ms");
    }
}

// Owner 2026-10-21: "the garage dyno did not react to doubling engine power" (pure EV). dyno_setup kept only the engine,
// turbo, conversion and gearing options, so Upgrades -> Electric drive (the motor scale, ev_battery_pack) never reached the
// dyno's setup (nor its cache key): the chart measured the stock motors and pack while the drive had the new ones.
namespace {
// The garage binding's path (RgGarage::refresh_dyno): the filtered setup, its JSON as the key, the worker, the result.
rg::DynoResult dyno_like_the_garage(Fixture& f, rg::GarageDyno& dyno, const std::string& car, const rg::VehicleSetup& working,
                                    const std::shared_ptr<const rg::DynoReference>& reference) {
    rg::DynoRequest r;
    r.entry = f.entry(car);
    r.table = f.table;
    r.context = f.ctx;
    r.setup = rg::dyno_setup(f.table, r.entry, working, reference.get());
    r.key = rg::setup_to_json(r.setup);
    r.reference = reference;
    dyno.request(std::move(r));
    auto result = wait(dyno);
    INFO(result.error);
    REQUIRE(result.error.empty());
    REQUIRE(result.points.size() == 33);
    return result;
}
rg::VehicleSetup ev_working(const std::map<std::string, rg::SetupValue>& values) {
    rg::VehicleSetup working;
    working.vehicle_id = "car_electric";
    working.values = values;
    return working;
}
} // namespace

TEST_CASE("dyno setup: electric motor and battery options reach an electrified car's dyno", "[garage_dyno][electric]") {
    Fixture f;
    const auto& ev = f.entry("car_electric");
    const auto working = ev_working({{"ev_motor_output", 0.5}, {"ev_layout", std::string("quad")}, {"ev_battery_pack", std::string("race")},
                                     {"brake_force", 1.2}});
    const auto filtered = rg::dyno_setup(f.table, ev, working, &f.reference);
    CHECK(filtered.values.count("ev_motor_output") == 1);
    CHECK(filtered.values.count("ev_layout") == 1);
    CHECK(filtered.values.count("ev_battery_pack") == 1);
    CHECK(filtered.values.count("brake_force") == 0); // still only what changes the curve
    // The cache key follows, so a new motor output is a new request.
    CHECK(rg::setup_to_json(filtered) != rg::setup_to_json(rg::dyno_setup(f.table, ev, ev_working({}), &f.reference)));
    // A combustion car never takes electric hardware into its dyno setup.
    rg::VehicleSetup sedan;
    sedan.vehicle_id = "car_sedan";
    sedan.values["ev_motor_output"] = 0.5;
    CHECK(rg::dyno_setup(f.table, f.entry("car_sedan"), sedan, &f.reference).values.count("ev_motor_output") == 0);
}

TEST_CASE("dyno worker: motor output scales the low-rpm torque, the race pack lifts the battery cap", "[garage_dyno][electric]") {
    Fixture f;
    auto reference = std::make_shared<const rg::DynoReference>(f.reference);
    rg::GarageDyno dyno;
    const auto stock = dyno_like_the_garage(f, dyno, "car_electric", ev_working({}), reference);
    const auto half = dyno_like_the_garage(f, dyno, "car_electric", ev_working({{"ev_motor_output", 0.5}}), reference);
    const auto quad = dyno_like_the_garage(f, dyno, "car_electric", ev_working({{"ev_layout", std::string("quad")}}), reference);
    const auto race = dyno_like_the_garage(f, dyno, "car_electric", ev_working({{"ev_layout", std::string("quad")}, {"ev_battery_pack", std::string("race")}}), reference);
    const auto peak_kw = [](const rg::DynoResult& r) {
        double best = 0;
        for (const auto& p : r.points) best = std::max(best, p.power_kw);
        return best;
    };
    // Lowest axis points: torque limited by the motor rating, not the pack: output x0.5, system torque x0.5.
    for (int i = 0; i < 4; ++i) {
        const auto k = static_cast<std::size_t>(i);
        INFO("axis point " << i << " at " << stock.points[k].rpm << " rpm");
        CHECK(half.points[k].torque_nm == Approx(0.5 * stock.points[k].torque_nm).epsilon(0.05));
    }
    // Four stock motors (1200 kW) are capped by the stock pack near its 700 kW; the race pack (x2.2 discharge) raises the cap.
    CHECK(peak_kw(quad) > peak_kw(stock));
    CHECK(peak_kw(quad) < 750.0);
    CHECK(peak_kw(race) > 1.15 * peak_kw(quad));
    WARN("peak system kW: stock " << peak_kw(stock) << ", quad " << peak_kw(quad) << ", quad + race pack " << peak_kw(race)
         << "; torque at " << stock.points[0].rpm << " rpm: " << stock.points[0].torque_nm << " -> " << half.points[0].torque_nm << " at output x0.5");
}
