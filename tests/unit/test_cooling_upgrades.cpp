// test_cooling_upgrades.cpp - garage cooling system upgrades (rg/cooling_upgrades.h; owner on PR #21: "More combustion should
// result in more heat for the engine components which should need to be cooled. We need cooling system upgrades (water cooling
// elements in the engine block, bigger radiator, ...)").
//
// Fast cases: the shipped table, the options on every car, the materialised files, the part masses, the setup check.
// Hidden [.][cooling][long]: full-throttle runs in a Session at 35 C (minutes): (1) the heat into the engine follows the fuel
// burned (a turbo kit makes more than the same engine without), (2) a bigger radiator runs the coolant cooler at the same time
// on the same run. Output lines start with "COOLING".
#include "rg/cooling_upgrades.h"
#include "rg/engine_store.h"
#include "rg/limit_advisor.h"
#include "rg/part_mass.h"
#include "rg/session.h"
#include "rg/turbo_kit.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/jobs/job_system.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
const std::string kRoot = RG_SOURCE_DIR;
const std::string kFile = kRoot + "/data/cooling_systems/cooling_upgrades.json";

rg::CoolingUpgradeConfig upgrades() {
    std::string err;
    auto c = rg::load_cooling_upgrades(kFile, &err);
    INFO(err);
    REQUIRE(c.has_value());
    return *c;
}

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    std::shared_ptr<rg::EngineStore> store;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_cooling_tests"};
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
        ctx.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
        store = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, "", &err);
        INFO(err);
        REQUIRE(store != nullptr);
        ctx.engines = std::make_shared<const rg::EngineStore>(*store);
        auto kits = rg::load_turbo_kit_config(kRoot + "/data/engines/turbo_kits.json", kRoot, &err);
        INFO(err);
        REQUIRE(kits.has_value());
        ctx.turbo_kits = std::make_shared<const rg::TurboKitConfig>(*kits);
        auto masses = rg::load_part_mass_table(kRoot + "/data/vehicles/part_masses.json", &err);
        INFO(err);
        REQUIRE(masses.has_value());
        ctx.part_masses = std::make_shared<const rg::PartMassTable>(*masses);
        ctx.cooling_upgrades = std::make_shared<const rg::CoolingUpgradeConfig>(upgrades());
    }
    const rg::CatalogEntry& entry(const std::string& id) const {
        const rg::CatalogEntry* e = catalog.find(id);
        REQUIRE(e != nullptr);
        return *e;
    }
};

rg::VehicleSetup setup_of(const std::string& vehicle, std::vector<std::pair<std::string, rg::SetupValue>> values) {
    rg::VehicleSetup s;
    s.vehicle_id = vehicle;
    for (auto& kv : values) s.values[kv.first] = kv.second;
    return s;
}

ps::vehicle::VehicleDesc load_materialised(const rg::MaterialisedSetup& m) {
    REQUIRE(m.ok);
    ps::io::VehicleLoadOptions options;
    options.maps.generate = false;
    return rg::load_vehicle(m.vehicle_path, options);
}

const rg::OptionView* view_of(const rg::SetupModel& m, const std::string& id) {
    for (const rg::OptionView& v : m.options) if (v.def.id == id) return &v;
    return nullptr;
}

} // namespace

TEST_CASE("cooling: the shipped table loads, every grade has an [AS] source, stock first", "[cooling]") {
    const rg::CoolingUpgradeConfig c = upgrades();
    CHECK_THAT(c.check.source, ContainsSubstring("[AS]"));
    for (const char* kind : {"radiator", "fan", "oil_cooler", "block_cooling"}) {
        INFO(kind);
        const rg::CoolingPart* p = c.find(kind);
        REQUIRE(p != nullptr);
        REQUIRE(p->grades.size() >= 2);
        CHECK(p->grades.front().id == "stock");
        CHECK_FALSE(p->damage_part.empty());
        for (const rg::CoolingGrade& g : p->grades) {
            INFO(g.id);
            if (g.id != "stock") {
                CHECK_THAT(g.source, ContainsSubstring("[AS]"));
                CHECK_THAT(g.source, ContainsSubstring("Mass"));
                CHECK((g.fixed_kg > 0.0 || g.per_litre_kg > 0.0 || g.added_kg_per_m2 > 0.0));
            }
        }
    }
    std::string err;
    CHECK_FALSE(rg::parse_cooling_upgrades("{\"format\":\"nope\"}", "t", &err).has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("cooling: the options are on every car, with a reason where they do not fit", "[cooling][setup]") {
    Fixture f;
    for (const rg::CatalogEntry& e : f.catalog.entries) {
        INFO(e.id);
        const rg::SetupModel m = rg::build_setup_model(e, f.table, f.ctx);
        REQUIRE(m.error.empty());
        const rg::OptionView* rad = view_of(m, "cooling_radiator");
        if (rad == nullptr) continue; // a catalog entry that lists no cooling options
        REQUIRE(rad->parts.size() >= 3);
        CHECK(rad->parts.front().id == "stock");
        for (const rg::SetupPart& p : rad->parts) {
            CHECK_FALSE(p.detail.empty());
            if (!p.fits) CHECK_FALSE(p.fit_reason.empty());
        }
        CHECK(rad->available);
    }
}

TEST_CASE("cooling: a radiator grade changes the cooling file and the mass nodes sum", "[cooling][setup][part_mass]") {
    Fixture f;
    const rg::CatalogEntry& gt = f.entry("car_gt70");
    const rg::VehicleSetup stock_setup = setup_of("car_gt70", {});
    const rg::VehicleSetup up = setup_of("car_gt70", {{"cooling_radiator", std::string("aluminium_wide")},
                                                      {"cooling_oil_cooler", std::string("street")},
                                                      {"cooling_block_cooling", std::string("enlarged_jackets")}});
    rg::ScopedWorkDir d0(f.ctx.work_root, "stock"), d1(f.ctx.work_root, "up");
    const rg::MaterialisedSetup m0 = rg::materialise_setup(gt, f.table, f.ctx, stock_setup, d0.path());
    const rg::MaterialisedSetup m1 = rg::materialise_setup(gt, f.table, f.ctx, up, d1.path());
    INFO(m0.error);
    INFO(m1.error);
    const ps::vehicle::VehicleDesc v0 = load_materialised(m0);
    const ps::vehicle::VehicleDesc v1 = load_materialised(m1);
    REQUIRE(v0.powertrain.thermal.has_value());
    REQUIRE(v1.powertrain.thermal.has_value());
    const auto& r0 = v0.powertrain.thermal->cooling.radiator;
    const auto& r1 = v1.powertrain.thermal->cooling.radiator;
    CHECK(r1.frontal_area_m2 == Approx(r0.frontal_area_m2 * 1.35).epsilon(1e-6));
    CHECK(r1.ua_ref_w_k == Approx(r0.ua_ref_w_k * 1.35 * 1.15).epsilon(1e-6));
    CHECK(v1.powertrain.thermal->cooling.oil_cooler_air_w_k_per_m_s > v0.powertrain.thermal->cooling.oil_cooler_air_w_k_per_m_s);
    // block cooling: a bigger coolant volume (engine lump), a cooler thermostat
    CHECK(v1.powertrain.thermal->engine.engine_capacity_j_k > v0.powertrain.thermal->engine.engine_capacity_j_k);
    CHECK(v1.powertrain.thermal->engine.thermostat.open_k < v0.powertrain.thermal->engine.thermostat.open_k);
    // the stock files are not touched
    CHECK(m0.compiled.cooling.has_value());
    CHECK(m0.compiled.cooling->mass_delta_kg() == Approx(0.0));

    // mass: the car's mass is the sum of its parts; the cooling node = stock hardware + the grades' added parts
    REQUIRE(m0.compiled.parts.ok);
    REQUIRE(m1.compiled.parts.ok);
    REQUIRE(m1.compiled.cooling.has_value());
    const double added = m1.compiled.cooling->mass_delta_kg();
    CHECK(added > 3.0);
    const rg::PartNode* c0 = m0.compiled.parts.root.find("engine.cooling");
    const rg::PartNode* c1 = m1.compiled.parts.root.find("engine.cooling");
    REQUIRE(c0 != nullptr);
    REQUIRE(c1 != nullptr);
    CHECK(c1->total_kg() == Approx(c0->total_kg() + added).margin(1e-6));
    CHECK(m1.compiled.parts.total_kg == Approx(m0.compiled.parts.total_kg + added).margin(1e-6));
    double children = 0.0;
    for (const rg::PartNode& ch : c1->children) children += ch.total_kg();
    CHECK(children == Approx(added).margin(1e-6));
    // and the stats' tree: the chassis mass the drive uses moves by the same amount
    CHECK(m1.compiled.parts.chassis_kg == Approx(m0.compiled.parts.chassis_kg + added).margin(1e-6));
}

TEST_CASE("cooling: a bigger radiator and an oil cooler move the setup check's steady temperatures", "[cooling][limit_advisor]") {
    Fixture f;
    const rg::CatalogEntry& hyper = f.entry("car_hyper");
    const auto steady_of = [&](const rg::VehicleSetup& s, double* oil) {
        rg::ScopedWorkDir d(f.ctx.work_root, "steady");
        const rg::MaterialisedSetup m = rg::materialise_setup(hyper, f.table, f.ctx, s, d.path());
        const ps::vehicle::VehicleDesc v = load_materialised(m);
        REQUIRE(v.powertrain.thermal.has_value());
        const rg::SteadyThermal t = rg::steady_thermal(*v.powertrain.thermal, 7000.0, 600e3, 90e3, 30.0, 45.0);
        if (oil != nullptr) *oil = t.oil_c;
        return t.coolant_c;
    };
    double oil0 = 0.0, oil1 = 0.0;
    const double c0 = steady_of(setup_of("car_hyper", {}), &oil0);
    const double c1 = steady_of(setup_of("car_hyper", {{"cooling_radiator", std::string("race_ducted")}, {"cooling_oil_cooler", std::string("race")}}), &oil1);
    std::printf("COOLING steady (7000 rpm, 600+90 kW heat, 30 C, 45 m/s): stock coolant %.1f oil %.1f -> race radiator + oil cooler coolant %.1f oil %.1f\n", c0, oil0, c1, oil1);
    CHECK(c1 < c0 - 3.0);
    CHECK(oil1 < oil0 - 2.0);
}

TEST_CASE("cooling: the time to the limit follows the physics model's lumps and ends at its steady state", "[cooling][limit_advisor]") {
    Fixture f;
    const rg::CatalogEntry& hyper = f.entry("car_hyper");
    rg::ScopedWorkDir d(f.ctx.work_root, "transient");
    const rg::MaterialisedSetup m = rg::materialise_setup(hyper, f.table, f.ctx, setup_of("car_hyper", {}), d.path());
    const ps::vehicle::VehicleDesc v = load_materialised(m);
    REQUIRE(v.powertrain.thermal.has_value());
    const auto& th = *v.powertrain.thermal;
    // Far above the limits: both reached, the coolant with more heat sooner; a finer step agrees within a second.
    const rg::ThermalTransient hot = rg::transient_thermal(th, 7000.0, 1400e3, 100e3, 30.0, 110.0, 118.0, 135.0, 1800.0, 0.25);
    const rg::ThermalTransient hotter = rg::transient_thermal(th, 7000.0, 1800e3, 100e3, 30.0, 110.0, 118.0, 135.0, 1800.0, 0.25);
    const rg::ThermalTransient fine = rg::transient_thermal(th, 7000.0, 1400e3, 100e3, 30.0, 110.0, 118.0, 135.0, 1800.0, 1.0 / 240.0);
    std::printf("COOLING transient (car_hyper stock, 1400 kW wall + 100 kW friction, 30 C, 110 m/s): coolant 118 C after %.1f s (1/240 s step %.1f s), oil 135 C after %.1f s\n",
                hot.coolant_limit_s, fine.coolant_limit_s, hot.oil_limit_s);
    REQUIRE(std::isfinite(hot.coolant_limit_s));
    REQUIRE(std::isfinite(hotter.coolant_limit_s));
    CHECK(hotter.coolant_limit_s < hot.coolant_limit_s);
    CHECK(std::abs(fine.coolant_limit_s - hot.coolant_limit_s) < 1.0);
    // The lumps set the pace: from the warm start (coolant ~91 C) the engine lump needs at least C x 27 K / heat in.
    const double warm_c = th.engine.thermostat.open_k + 0.25 * (th.engine.thermostat.full_open_k - th.engine.thermostat.open_k) - 273.15;
    CHECK(hot.coolant_limit_s > th.engine.engine_capacity_j_k * (118.0 - warm_c) / 1500e3);
    // Below the limits: never reached, and the run ends where steady_thermal says it settles.
    const rg::ThermalTransient cool = rg::transient_thermal(th, 5000.0, 300e3, 40e3, 30.0, 60.0, 118.0, 135.0, 7200.0, 0.25);
    const rg::SteadyThermal st = rg::steady_thermal(th, 5000.0, 300e3, 40e3, 30.0, 60.0);
    CHECK_FALSE(std::isfinite(cool.coolant_limit_s));
    CHECK_FALSE(std::isfinite(cool.oil_limit_s));
    CHECK(std::abs(cool.end_coolant_c - st.coolant_c) < 0.2);
    CHECK(std::abs(cool.end_oil_c - st.oil_c) < 0.5);
}

TEST_CASE("cooling: the setup check warns about a coolant or oil temperature above the limit and names the part", "[cooling][limit_advisor]") {
    std::string err;
    auto adv = rg::load_advisor_config(kRoot + "/data/garage/advisor.json", &err);
    INFO(err);
    REQUIRE(adv.has_value());
    rg::AdviceInput in;
    in.coolant_limit_c = 118.0;
    in.oil_limit_c = 135.0;
    in.thermal_ambient_c = 30.0;
    in.thermal_air_speed_m_s = in.coolant_air_speed_m_s = in.oil_air_speed_m_s = 45.0;
    in.radiator_part = "stock radiator";
    in.radiator_option = "cooling_radiator";
    in.oil_cooler_part = "stock oil cooler";
    in.oil_cooler_option = "cooling_oil_cooler";
    const auto find = [](const std::vector<rg::AdviceItem>& v, const std::string& name) -> const rg::AdviceItem* {
        for (const rg::AdviceItem& i : v) if (i.rule_name == name) return &i;
        return nullptr;
    };
    // never reaches the limit: nothing (whatever the steady figure says)
    in.coolant_peak_c = 100.0;
    in.oil_peak_c = 110.0;
    CHECK(find(rg::build_advice(in, *adv), "CoolantTemperature") == nullptr);
    CHECK(find(rg::build_advice(in, *adv), "OilTemperature") == nullptr);
    // reached after 150 s: a warning; within 60 s (advisor.json thermal_time.critical_within_s): critical
    REQUIRE(adv->thermal_critical_within_s == 60.0);
    in.coolant_peak_c = 122.0;
    in.coolant_limit_time_s = 150.0;
    in.oil_peak_c = 150.0;
    in.oil_limit_time_s = 42.0;
    const auto items = rg::build_advice(in, *adv);
    const rg::AdviceItem* hot = find(items, "CoolantTemperature");
    REQUIRE(hot != nullptr);
    CHECK(hot->severity == rg::AdviceSeverity::Warning);
    CHECK(hot->damage_part == "head_gasket");
    CHECK(hot->option_id == "cooling_radiator");
    REQUIRE_FALSE(hot->lines.empty());
    CHECK_THAT(hot->lines.front(), ContainsSubstring("after about 150 s"));
    CHECK_THAT(hot->lines.front(), ContainsSubstring("118"));
    CHECK_THAT(hot->lines.front(), ContainsSubstring("122"));
    const rg::AdviceItem* oil = find(items, "OilTemperature");
    REQUIRE(oil != nullptr);
    CHECK(oil->severity == rg::AdviceSeverity::Critical);
    CHECK_THAT(oil->lines.front(), ContainsSubstring("40 s"));
    CHECK(oil->option_id == "cooling_oil_cooler");
    // exactly at the threshold is still critical, just past it a warning
    in.oil_limit_time_s = 60.0;
    CHECK(find(rg::build_advice(in, *adv), "OilTemperature")->severity == rg::AdviceSeverity::Critical);
    in.oil_limit_time_s = 61.0;
    CHECK(find(rg::build_advice(in, *adv), "OilTemperature")->severity == rg::AdviceSeverity::Warning);
    CHECK_FALSE(oil->damage_part.empty());
}

// ---- hidden measurements (minutes) ----------------------------------------------------------------------------------------

namespace {
struct HotRun {
    double coolant_c = 0.0, oil_c = 0.0, peak_wall_kw = 0.0, peak_fuel_kw = 0.0, kmh = 0.0;
    double t_s = 0.0;
    double rpm = 0.0, wall_w = 0.0, fric_w = 0.0, rad_w = 0.0, coolant_10s_ago = 0.0, steady_c = 0.0, steady_oil_c = 0.0;
};
// Full throttle from a standstill on a 40 km strip, `seconds` of simulated time (or until the strip ends), ambient 35 C.
HotRun hot_run(const rg::CatalogEntry& entry, const std::string& vehicle_path, double ambient_c, double seconds) {
    rg::SessionConfig config;
    config.vehicle_json_path = vehicle_path;
    config.surface_table_path = kRoot + "/data/surfaces/surfaces.json";
    config.chassis_mass_kg = entry.chassis.mass_kg;
    config.chassis_half_extents = ps::Vec3{entry.chassis.half_extents[0], entry.chassis.half_extents[1], entry.chassis.half_extents[2]};
    config.chassis_z_m = entry.chassis.spawn_z_m;
    config.ground_half_extent_m = 20000.0;
    config.ground_half_extent_y_m = 500.0;
    config.ground_tessellated = true;
    config.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
    rg::Session session(config);
    auto& world = session.world();
    const auto vid = session.vehicle_id();
    const ps::drivetrain::EngineThermalConfig* thermal = session.vehicle_desc().powertrain.thermal ? &*session.vehicle_desc().powertrain.thermal : nullptr;
    world.set_ambient(ps::AmbientSample{101325.0, 273.15 + ambient_c, 0.0});
    for (int i = 0; i < 480; ++i) session.step();
    world.set_control("ignition", 1.0);
    world.set_control("starter", 1.0);
    world.set_control("assist.auto_shift", 1.0);
    world.set_control("assist.auto_clutch", 1.0);
    world.set_control("assist.auto_blip", 1.0);
    world.set_control("clutch", 0.0);
    world.set_control("brake", 0.0);
    world.set_control("handbrake", 0.0);
    HotRun r;
    bool started = false;
    const long steps = static_cast<long>(seconds * 240.0);
    for (long s = 0; s < steps; ++s) {
        if (!started && (world.powertrain_state(vid).engine_state == ps::drivetrain::EngineState::Running || s > 240 * 6)) {
            world.set_control("starter", 0.0);
            started = true;
        }
        if (started) world.set_control("throttle", 1.0);
        session.step();
        r.t_s += 1.0 / 240.0;
        const auto pt = world.powertrain_state(vid);
        r.peak_wall_kw = std::max(r.peak_wall_kw, static_cast<double>(pt.thermal.wall_heat_w) / 1000.0);
        const ps::Motion mo = world.get_motion(session.chassis_body());
        r.kmh = std::sqrt(mo.linear.x * mo.linear.x + mo.linear.y * mo.linear.y) * 3.6;
        r.coolant_c = pt.thermal.coolant_c;
        r.oil_c = pt.thermal.oil_c;
        r.rpm = pt.engine_rpm;
        r.wall_w = pt.thermal.wall_heat_w;
        r.fric_w = pt.thermal.friction_heat_w;
        r.rad_w = pt.thermal.radiator_heat_w;
        if (s == steps - 241 * 10) r.coolant_10s_ago = r.coolant_c;
        if (ambient_c > -100.0 && thermal != nullptr) {
            const rg::SteadyThermal st = rg::steady_thermal(*thermal, r.rpm, r.wall_w, r.fric_w, ambient_c, r.kmh / 3.6);
            r.steady_c = st.coolant_c;
            r.steady_oil_c = st.oil_c;
        }
        if (world.get_pose(session.chassis_body()).position.x > 19000.0) break;
    }
    return r;
}
} // namespace

TEST_CASE("cooling: more fuel burned means more heat into the engine (turbo kit vs none)", "[.][cooling][long]") {
    Fixture f;
    const rg::CatalogEntry& gt = f.entry("car_gt70");
    const auto run = [&](const rg::VehicleSetup& s) {
        rg::ScopedWorkDir d(f.ctx.work_root, "heat");
        rg::SetupContext ctx = f.ctx;
        const rg::MaterialisedSetup m = rg::materialise_setup(gt, f.table, ctx, s, d.path());
        INFO(m.error);
        REQUIRE(m.ok);
        return hot_run(gt, m.vehicle_path, 35.0, 25.0);
    };
    const HotRun na = run(setup_of("car_gt70", {}));
    const HotRun turbo = run(setup_of("car_gt70", {{"turbo_kit", std::string("sport_single")}}));
    std::printf("COOLING heat: NA peak wall heat %.0f kW, turbo kit %.0f kW (x%.2f); coolant after 25 s %.1f vs %.1f C\n", na.peak_wall_kw,
                turbo.peak_wall_kw, turbo.peak_wall_kw / std::max(1.0, na.peak_wall_kw), na.coolant_c, turbo.coolant_c);
    CHECK(turbo.peak_wall_kw > na.peak_wall_kw * 1.1);
    CHECK(turbo.coolant_c >= na.coolant_c);
}

TEST_CASE("cooling: a bigger radiator and oil cooler run the coolant cooler on the same hot full-throttle run", "[.][cooling][long]") {
    Fixture f;
    const rg::CatalogEntry& hyper = f.entry("car_hyper");
    const auto run = [&](const rg::VehicleSetup& s) {
        rg::ScopedWorkDir d(f.ctx.work_root, "hot");
        const rg::MaterialisedSetup m = rg::materialise_setup(hyper, f.table, f.ctx, s, d.path());
        INFO(m.error);
        REQUIRE(m.ok);
        return hot_run(hyper, m.vehicle_path, 35.0, 170.0);
    };
    const HotRun stock = run(setup_of("car_hyper", {}));
    const HotRun race = run(setup_of("car_hyper", {{"cooling_radiator", std::string("race_ducted")}, {"cooling_oil_cooler", std::string("race")}}));
    std::printf("COOLING hot run (car_hyper, 35 C, full throttle %.0f s, %.0f km/h): stock coolant %.1f oil %.1f C; race radiator + oil cooler coolant %.1f oil %.1f C\n",
                stock.t_s, stock.kmh, stock.coolant_c, stock.oil_c, race.coolant_c, race.oil_c);
    for (const HotRun* h : {&stock, &race})
        std::printf("COOLING   end of run: %.0f rpm, wall %.0f kW + friction %.0f kW, radiator takes %.0f kW, coolant %.1f (10 s earlier %.1f); steady_thermal() at this instant predicts coolant %.1f oil %.1f\n",
                    h->rpm, h->wall_w / 1000.0, h->fric_w / 1000.0, h->rad_w / 1000.0, h->coolant_c, h->coolant_10s_ago, h->steady_c, h->steady_oil_c);
    CHECK(race.coolant_c < stock.coolant_c - 2.0);
    CHECK(race.oil_c < stock.oil_c - 1.0);
}

TEST_CASE("cooling: the setup check on every stock car; the stock hypercar holds full power, only the nitrous one runs out", "[.][cooling][long][limit_advisor]") {
    Fixture f;
    std::string err;
    auto adv = rg::load_advisor_config(kRoot + "/data/garage/advisor.json", &err);
    INFO(err);
    REQUIRE(adv.has_value());
    adv->debounce_ms = 5;
    const auto check_of = [&](const rg::CatalogEntry& e, const rg::VehicleSetup& setup) {
        rg::LimitAdvisor advisor(*adv);
        rg::AdvisorRequest r;
        r.entry = e;
        r.table = f.table;
        r.context = f.ctx;
        r.setup = setup;
        r.key = e.id;
        advisor.request(r);
        rg::AdvisorResult res;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(300);
        for (;;) {
            res = advisor.result();
            if (!res.busy && !res.solving) break;
            REQUIRE(std::chrono::steady_clock::now() < deadline);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        INFO(res.error);
        REQUIRE(res.error.empty());
        std::vector<rg::AdviceItem> out;
        for (const rg::AdviceItem& i : res.items)
            if (i.rule_name == "CoolantTemperature" || i.rule_name == "OilTemperature") out.push_back(i);
        return out;
    };
    for (const rg::CatalogEntry& e : f.catalog.entries) {
        if (!e.preset_of.empty()) continue;
        INFO(e.id);
        const auto items = check_of(e, setup_of(e.id, {}));
        for (const rg::AdviceItem& i : items)
            std::printf("COOLING check %s: [%s] %s\n", e.id.c_str(), rg::to_string(i.severity), i.lines.empty() ? "" : i.lines.front().c_str());
        if (items.empty()) std::printf("COOLING check %s: clean\n", e.id.c_str());
        // Owner on PR #28: the stock hypercar is clean or at most a plain warning (data/cooling_systems/hyper.json is sized for
        // it). The nitrous hypercar shares that cooling system and is not sized for an endless shot. Every other car is clean.
        if (e.id == "car_hyper") {
            for (const rg::AdviceItem& i : items) CHECK(i.severity != rg::AdviceSeverity::Critical);
        } else if (e.id != "car_hyper_n2o") {
            CHECK(items.empty());
        }
    }
    // The figures behind the hypercar items (also when clean): soonest time to each limit and the steady temperature.
    const auto figures = [&](const rg::CatalogEntry& e, const rg::VehicleSetup& setup, const char* label) {
        rg::ScopedWorkDir d(f.ctx.work_root, "figures");
        const rg::MaterialisedSetup m = rg::materialise_setup(e, f.table, f.ctx, setup, d.path());
        REQUIRE(m.ok);
        ps::io::EngineMapOptions maps;
        maps.quality = ps::drivetrain::MapQuality::Sketch;
        maps.cache_dir = f.ctx.engine_map_cache_dir;
        const ps::vehicle::VehicleDesc desc = rg::load_vehicle(m.vehicle_path, maps);
        ps::jobs::JobSystem pool(2);
        rg::AdviceSweep sweep = rg::run_advice_sweep(desc, *adv, &pool, nullptr);
        rg::set_thermal_limits(sweep.input, desc, f.ctx.cooling_upgrades->check, m.compiled);
        const auto when = [](double t) { return std::isfinite(t) ? std::to_string(static_cast<int>(std::lround(t))) + " s" : std::string("never"); };
        std::printf("COOLING figures %s: coolant settles %.1f C (limit %.0f C) at %.0f rpm, reached %s; oil settles %.1f C (limit %.0f C) at %.0f rpm, reached %s\n",
                    label, sweep.input.coolant_peak_c, sweep.input.coolant_limit_c, sweep.input.coolant_peak_rpm, when(sweep.input.coolant_limit_time_s).c_str(),
                    sweep.input.oil_peak_c, sweep.input.oil_limit_c, sweep.input.oil_peak_rpm, when(sweep.input.oil_limit_time_s).c_str());
    };
    REQUIRE(f.ctx.cooling_upgrades != nullptr);
    const rg::VehicleSetup race_cooling_hyper = setup_of("car_hyper", {{"cooling_radiator", std::string("race_ducted")}, {"cooling_oil_cooler", std::string("race")}});
    figures(f.entry("car_hyper"), setup_of("car_hyper", {}), "car_hyper stock");
    figures(f.entry("car_hyper"), race_cooling_hyper, "car_hyper race radiator + oil cooler");
    figures(f.entry("car_hyper_n2o"), setup_of("car_hyper_n2o", {}), "car_hyper_n2o stock");
    // The race radiator and oil cooler take the hypercar's coolant item away.
    const rg::CatalogEntry& hyper = f.entry("car_hyper");
    const auto fixed = check_of(hyper, setup_of("car_hyper", {{"cooling_radiator", std::string("race_ducted")}, {"cooling_oil_cooler", std::string("race")}}));
    for (const rg::AdviceItem& i : fixed)
        std::printf("COOLING check car_hyper + race cooling: [%s] %s\n", rg::to_string(i.severity), i.lines.empty() ? "" : i.lines.front().c_str());
    if (fixed.empty()) std::printf("COOLING check car_hyper + race cooling: clean\n");
    CHECK(fixed.empty());
}
