#ifdef _MSC_VER
// Calibration-only environment reads do not modify the environment.
#define _CRT_SECURE_NO_WARNINGS
#endif
// test_damage_engine.cpp - the engine internals of the damage model (rg/damage.h; owner on PR #33: "getting oil up to 170 C
// within 3 minutes should not be possible because the engine breaks first", "an engine without a real working cooling
// solution should keep running, but then other parts will start failing and that might cause the engine to shut off").
//
// Fast cases (the game's own thermal sedan, a torque-map engine): worn bearings cost power through physics_sim's
// World::set_engine_wear; the cascade (broken cooling -> hot oil -> bearing / piston wear -> more friction -> seized);
// a seized engine shuts off and does not restart until a repair; difficulty.mechanical_damage off = no wear, no effect;
// 1 vs 4 workers. Hidden [.][damage][long]: the owner's hypercar scenario (about 2 MW, 11 bar) driven through the garage
// setup, the timeline printed as "DAMAGE_RUN" lines (docs/damage_model.md and the PR #33 text quote it).
#include "rg/cooling_upgrades.h"
#include "rg/damage.h"
#include "rg/engine_store.h"
#include "rg/part_mass.h"
#include "rg/session.h"
#include "rg/turbo_kit.h"
#include "rg/vehicle_catalog.h"
#include "rg/vehicle_setup.h"

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
const std::string kRoot = RG_SOURCE_DIR;
using ps::drivetrain::EngineState;

rg::DamageConfig shipped() {
    std::string err;
    auto c = rg::load_damage_config(kRoot + "/data/vehicles/damage.json", &err);
    INFO(err);
    REQUIRE(c.has_value());
    return *c;
}

std::shared_ptr<const rg::SessionConfig::Damage> damage_of(const rg::DamageState& state, bool enabled = true) {
    auto d = std::make_shared<rg::SessionConfig::Damage>();
    d->config = shipped();
    d->state = state;
    d->enabled = enabled;
    return d;
}

rg::SessionConfig sedan(unsigned workers = 2) {
    rg::SessionConfig config;
    config.vehicle_json_path = kRoot + "/data/vehicles/car_sedan.json";
    config.surface_table_path = kRoot + "/data/surfaces/surfaces.json";
    config.job_workers = workers;
    return config;
}

// Start the engine (2 s of cranking), then hold it in neutral at `throttle`.
void start_engine(rg::Session& s, double throttle) {
    ps::World& w = s.world();
    w.set_control("ignition", 1.0);
    w.set_control("starter", 1.0);
    w.set_control("clutch", 1.0);
    w.set_control("brake", 1.0);
    for (int i = 0; i < 240 * 2; ++i) s.step();
    w.set_control("starter", 0.0);
    w.set_control("throttle", throttle);
}

struct Rev {
    double rpm = 0.0, oil_c = 0.0, coolant_c = 0.0, friction_kw = 0.0, wear_scale = 1.0;
    EngineState state = EngineState::Off;
    std::uint64_t hash = 0;
    rg::DamageReport report;
    double seized_at_s = -1.0;
    double peak_oil_c = 0.0;          // the whole run (after a seizure the hot coolant still soaks into the oil)
    double peak_oil_running_c = 0.0;  // while the engine ran
};

// Engine started, oil / coolant forced to the given temperatures, then `seconds` at `throttle` in neutral.
Rev rev_hot(rg::SessionConfig config, double coolant_c, double oil_c, double throttle, double seconds, bool print = false) {
    rg::Session s(config);
    start_engine(s, throttle);
    ps::World& w = s.world();
    const auto vid = s.vehicle_id();
    w.set_engine_thermal_state(vid, ps::drivetrain::EngineThermalState{coolant_c, oil_c});
    Rev r;
    const long steps = static_cast<long>(seconds * 240.0);
    for (long i = 0; i < steps; ++i) {
        s.step();
        const auto pt = w.powertrain_state(vid);
        r.peak_oil_c = std::max(r.peak_oil_c, static_cast<double>(pt.thermal.oil_c));
        if (pt.engine_state == EngineState::Running) r.peak_oil_running_c = std::max(r.peak_oil_running_c, static_cast<double>(pt.thermal.oil_c));
        if (r.seized_at_s < 0.0 && s.damage_report().seized) r.seized_at_s = static_cast<double>(i + 1) / 240.0;
        if (print && i % (240 * 5) == 0) {
            const rg::DamageReport d = s.damage_report();
            std::printf("DAMAGE_CASCADE t %5.1f s  rpm %5.0f  coolant %6.1f  oil %6.1f  friction %5.1f kW x%.2f  radiator %.2f fan %.2f gasket %.2f crank %.3f pistons %.3f\n",
                        static_cast<double>(i) / 240.0, static_cast<double>(pt.engine_rpm), static_cast<double>(pt.thermal.coolant_c),
                        static_cast<double>(pt.thermal.oil_c), static_cast<double>(pt.thermal.friction_heat_w) / 1000.0, d.applied.friction,
                        d.state.health_of("radiator"), d.state.health_of("radiator_fan"), d.state.health_of("head_gasket"),
                        d.state.health_of("crankshaft"), d.state.health_of("pistons"));
        }
    }
    const auto pt = w.powertrain_state(vid);
    r.rpm = pt.engine_rpm;
    r.oil_c = pt.thermal.oil_c;
    r.coolant_c = pt.thermal.coolant_c;
    r.friction_kw = pt.thermal.friction_heat_w / 1000.0;
    r.wear_scale = pt.thermal.wear_friction_scale;
    r.state = pt.engine_state;
    r.hash = w.state_hash();
    r.report = s.damage_report();
    return r;
}

} // namespace

TEST_CASE("damage engine: the shipped file has the oil-driven internals, with absolute oil limits above the setup check's", "[damage][engine]") {
    const rg::DamageConfig c = shipped();
    for (const char* id : {"crankshaft", "pistons"}) {
        INFO(id);
        const rg::DamagePartConfig* p = c.find(id);
        REQUIRE(p != nullptr);
        CHECK(p->effect == rg::DamageEffect::Friction);
        CHECK(p->scale_at_zero > 1.0);
        CHECK(p->can_seize());
    }
    for (const rg::DamageRuleConfig& r : c.rules) {
        if (r.signal != rg::DamageSignal::OilC) continue;
        CHECK_FALSE(r.relative_to_overheat);
        CHECK(r.above >= 135.0); // the setup check's oil limit: a car the check passes never wears its internals
    }
}

TEST_CASE("damage engine: worn bearings raise the friction and cost rpm, through World::set_engine_wear", "[damage][engine][session]") {
    rg::SessionConfig healthy = sedan(), worn = sedan();
    healthy.damage = damage_of(rg::DamageState{});
    rg::DamageState s;
    s.health["crankshaft"] = 0.5; // friction x (3 + (1 - 3) x 0.5) = x2
    worn.damage = damage_of(s);
    const double thr = std::getenv("RG_WEAR_THR") ? std::atof(std::getenv("RG_WEAR_THR")) : 0.3;
    const Rev h = rev_hot(healthy, 95.0, 100.0, thr, 6.0);
    const Rev w = rev_hot(worn, 95.0, 100.0, thr, 6.0);
    std::printf("DAMAGE_ENGINE bearings at 50 %%: friction %.2f -> %.2f kW (scale %.2f), rpm %.0f -> %.0f, oil %.2f -> %.2f C\n", h.friction_kw,
                w.friction_kw, w.wear_scale, h.rpm, w.rpm, h.oil_c, w.oil_c);
    CHECK(h.report.applied.friction == 1.0);
    CHECK(w.report.applied.friction == 2.0);
    CHECK(w.wear_scale == 2.0);
    CHECK(w.rpm < 0.7 * h.rpm); // the same throttle in neutral: the doubled friction eats most of the free-revving torque
    CHECK(w.report.applied.cooling_unity()); // a friction-only damage never touches the cooling seam
    CHECK(w.state == EngineState::Running);  // worn, not seized: it keeps running
}

TEST_CASE("damage engine: the cascade - broken cooling keeps the oil hot, the internals wear, their friction heats the oil, the engine seizes",
          "[damage][engine][session][long]") {
    // The same hot start (coolant 125 C, oil 140 C, high revs in neutral). Working cooling pulls the oil back under the
    // bearings' 135 C within seconds; with a dead radiator and fan it stays hot, the bearings and pistons wear, the
    // friction they add heats the oil further, and the engine seizes and stops.
    rg::DamageState broken;
    broken.health["radiator"] = 0.0;
    broken.health["radiator_fan"] = 0.0;
    rg::SessionConfig good = sedan(1), bad1 = sedan(1), bad4 = sedan(4);
    good.damage = damage_of(rg::DamageState{});
    bad1.damage = damage_of(broken);
    bad4.damage = damage_of(broken);
    const double thr = std::getenv("RG_CASCADE_THR") ? std::atof(std::getenv("RG_CASCADE_THR")) : 0.2;
    const double c0 = std::getenv("RG_CASCADE_C0") ? std::atof(std::getenv("RG_CASCADE_C0")) : 95.0;
    const double o0 = std::getenv("RG_CASCADE_O0") ? std::atof(std::getenv("RG_CASCADE_O0")) : 105.0;
    const double secs = std::getenv("RG_CASCADE_S") ? std::atof(std::getenv("RG_CASCADE_S")) : 320.0;
    const Rev g = rev_hot(good, c0, o0, thr, secs, /*print=*/true);
    const Rev b = rev_hot(bad1, c0, o0, thr, secs, /*print=*/true);
    const Rev b4 = rev_hot(bad4, c0, o0, thr, secs);
    std::printf("DAMAGE_CASCADE working cooling: oil %.1f C at the end, crank %.3f; broken cooling: peak oil while running %.1f C (%.1f C after the stop), crank %.3f, pistons %.3f, seized at %.1f s (%s)\n",
                g.oil_c, g.report.state.health_of("crankshaft"), b.peak_oil_running_c, b.peak_oil_c, b.report.state.health_of("crankshaft"),
                b.report.state.health_of("pistons"), b.seized_at_s, b.report.seized_part.c_str());
    CHECK(g.report.state.health_of("crankshaft") > 0.95);
    CHECK_FALSE(g.report.seized);
    CHECK(g.state == EngineState::Running);
    CHECK(b.report.seized);
    CHECK(b.seized_at_s > 0.0);
    CHECK(b.state != EngineState::Running);
    CHECK(b.rpm < 50.0);
    CHECK(b.peak_oil_running_c < 170.0); // the engine broke before the oil reached 170 C
    // Deterministic: 1 vs 4 workers, bit for bit.
    CHECK(b.hash == b4.hash);
    CHECK(b.report.state.health == b4.report.state.health);
    CHECK(b.seized_at_s == b4.seized_at_s);
}

TEST_CASE("damage engine: a seized engine does not restart until it is repaired", "[damage][engine][session]") {
    rg::SessionConfig c = sedan();
    rg::DamageState seized;
    seized.health["crankshaft"] = 0.02; // below seize_below 0.05
    c.damage = damage_of(seized);
    rg::Session s(c);
    ps::World& w = s.world();
    const auto vid = s.vehicle_id();
    CHECK(s.damage_report().seized);
    CHECK(s.damage_report().seized_part == "crankshaft");
    // The player cranks for 3 s: nothing.
    start_engine(s, 0.2);
    w.set_control("starter", 1.0);
    for (int i = 0; i < 240; ++i) s.step();
    CHECK(w.powertrain_state(vid).engine_state != EngineState::Running);
    CHECK(w.powertrain_state(vid).engine_rpm < 50.0);
    CHECK(w.get_control("ignition") == 0.0); // held at 0 for the step
    // Repair: the same key turn starts it.
    s.request_damage_repair();
    CHECK_FALSE(s.damage_report().seized);
    bool ran = false;
    for (int i = 0; i < 240 * 3 && !ran; ++i) {
        w.set_control("ignition", 1.0); // the first tick after the request is still held (the repair lands after its step)
        w.set_control("starter", 1.0);
        s.step();
        ran = w.powertrain_state(vid).engine_state == EngineState::Running;
    }
    CHECK(ran);
    CHECK(s.damage_report().state.all_healthy());
    CHECK(s.damage_report().applied.unity());

    // Seizing while running: the engine stops.
    rg::SessionConfig c2 = sedan();
    rg::DamageState nearly;
    nearly.health["crankshaft"] = 0.06;
    c2.damage = damage_of(nearly);
    const Rev r = rev_hot(c2, 100.0, 170.0, 0.5, 5.0);
    CHECK(r.report.seized);
    CHECK(r.state != EngineState::Running);
}

TEST_CASE("damage engine: difficulty.mechanical_damage off - nothing wears and stored damage has no effect, but is kept",
          "[damage][engine][session]") {
    rg::DamageState worn;
    worn.health["crankshaft"] = 0.02; // seized
    worn.health["radiator"] = 0.3;
    rg::SessionConfig none = sedan(), off = sedan();
    off.damage = damage_of(worn, /*enabled=*/false);
    // Hot oil far past both oil rules: no wear with the toggle off; the run equals a car with no damage model at all.
    const Rev a = rev_hot(none, 100.0, 160.0, 0.5, 5.0);
    const Rev b = rev_hot(off, 100.0, 160.0, 0.5, 5.0);
    CHECK(a.hash == b.hash);
    CHECK(b.state == EngineState::Running);
    CHECK_FALSE(b.report.enabled);
    CHECK_FALSE(b.report.seized);
    CHECK(b.report.applied.unity());
    CHECK(b.report.revision == 0);
    CHECK(b.report.state.health_of("crankshaft") == 0.02); // kept as stored
    CHECK(b.report.state.health_of("pistons") == 1.0);     // no wear

    // Switched on and off while driving.
    rg::SessionConfig on = sedan();
    on.damage = damage_of(worn);
    rg::Session s(on);
    ps::World& w = s.world();
    const auto vid = s.vehicle_id();
    s.step();
    CHECK(s.damage_report().seized);
    s.set_damage_enabled(false);
    s.step();
    CHECK_FALSE(s.damage_report().seized);
    CHECK(s.damage_report().applied.unity());
    start_engine(s, 0.2);
    CHECK(w.powertrain_state(vid).engine_state == EngineState::Running);
    s.set_damage_enabled(true);
    for (int i = 0; i < 240; ++i) s.step();
    CHECK(s.damage_report().seized);
    CHECK(w.powertrain_state(vid).engine_state != EngineState::Running);
}

TEST_CASE("damage engine: oil climbing toward 190 C in 3 minutes (the owner's report) seizes the engine before 170 C", "[damage][engine]") {
    // The damage model alone on a prescribed oil temperature ramp from 110 C: the owner's 190 C after 3 minutes is
    // 0.44 K/s; faster ramps to 3 K/s too. No friction feedback here (that only makes it seize sooner, see the cascade).
    for (const double rise_k_per_s : {0.44, 1.0, 3.0}) {
        rg::DamageModel m(shipped(), rg::DamageState{}, 118.0);
        const double dt = 1.0 / 240.0;
        double oil = 110.0, t = 0.0;
        while (m.seized() == nullptr && oil < 200.0) {
            m.step(dt, rg::DamageInputs{true, 100.0, oil, false, true});
            oil += rise_k_per_s * dt;
            t += dt;
        }
        std::printf("DAMAGE_RAMP oil +%.2f K/s from 110 C: seized (%s) after %.1f s at %.1f C; crank %.3f, pistons %.3f\n", rise_k_per_s,
                    m.seized() != nullptr ? m.seized()->id.c_str() : "-", t, oil, m.state().health_of("crankshaft"), m.state().health_of("pistons"));
        CHECK(m.seized() != nullptr);
        CHECK(oil < 170.0);
        // Cooled down again at once (the engine still running): nothing more wears.
        const rg::DamageState before = m.state();
        rg::DamageModel cool(shipped(), before, 118.0);
        CHECK_FALSE(cool.step(1.0, rg::DamageInputs{true, 100.0, 130.0, false, true}));
    }
    // A stopped engine does not wear its bearings, however hot the oil.
    rg::DamageModel off(shipped(), rg::DamageState{}, 118.0);
    CHECK_FALSE(off.step(1.0, rg::DamageInputs{true, 100.0, 180.0, false, false}));
}

// ---- the owner's scenario ---------------------------------------------------------------------------------------------

namespace {

struct Garage {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    std::shared_ptr<rg::EngineStore> store;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_damage_engine"};
    Garage() {
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
        auto up = rg::load_cooling_upgrades(kRoot + "/data/cooling_systems/cooling_upgrades.json", &err);
        INFO(err);
        REQUIRE(up.has_value());
        ctx.cooling_upgrades = std::make_shared<const rg::CoolingUpgradeConfig>(*up);
    }
    const rg::CatalogEntry& entry(const std::string& id) const {
        const rg::CatalogEntry* e = catalog.find(id);
        REQUIRE(e != nullptr);
        return *e;
    }
};

struct OwnerRun {
    double end_t_s = 0.0, seized_at_s = -1.0, oil_at_seize_c = 0.0, peak_oil_c = 0.0, peak_oil_running_c = 0.0, peak_kw = 0.0, peak_boost_bar = 0.0;
    double t_oil_150 = -1.0, t_oil_170 = -1.0;
    bool restarted_while_seized = false;
    std::string seized_part;
};

// Full throttle from standstill on a 40 km strip, auto-shift, `ambient_c`, at most `seconds`; damage optional. After a
// seizure: 10 s of coasting, then 3 s of cranking (must not start).
OwnerRun owner_run(const char* label, const rg::CatalogEntry& entry, const std::string& vehicle_path,
                   std::shared_ptr<const rg::SessionConfig::Damage> damage, double ambient_c, double seconds, bool sport_nitrous) {
    rg::SessionConfig config;
    config.vehicle_json_path = vehicle_path;
    config.surface_table_path = kRoot + "/data/surfaces/surfaces.json";
    config.chassis_mass_kg = entry.chassis.mass_kg;
    config.chassis_half_extents = ps::Vec3{entry.chassis.half_extents[0], entry.chassis.half_extents[1], entry.chassis.half_extents[2]};
    config.chassis_z_m = entry.chassis.spawn_z_m;
    config.ground_half_extent_m = 40000.0;
    config.ground_half_extent_y_m = 500.0;
    config.ground_tessellated = true;
    config.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
    config.damage = std::move(damage);
    rg::Session session(config);
    auto& world = session.world();
    const auto vid = session.vehicle_id();
    world.set_ambient(ps::AmbientSample{101325.0, 273.15 + ambient_c, 0.0});
    for (int i = 0; i < 480; ++i) session.step();
    world.set_control("ignition", 1.0);
    world.set_control("starter", 1.0);
    world.set_control("assist.auto_shift", 1.0);
    world.set_control("assist.auto_clutch", 1.0);
    world.set_control("assist.auto_blip", 1.0);
    if (sport_nitrous) {
        world.set_control("assist.drive_mode", 2.0);
        world.set_control("nitrous_arm", 1.0);
    }
    world.set_control("clutch", 0.0);
    world.set_control("brake", 0.0);
    world.set_control("handbrake", 0.0);
    OwnerRun r;
    bool started = false;
    double t = 0.0, after_seize = 0.0;
    const long steps = static_cast<long>(seconds * 240.0);
    for (long s = 0; s < steps; ++s) {
        if (!started && (world.powertrain_state(vid).engine_state == EngineState::Running || s > 240 * 6)) {
            world.set_control("starter", 0.0);
            started = true;
        }
        if (started && r.seized_at_s < 0.0) world.set_control("throttle", 1.0);
        session.step();
        t += 1.0 / 240.0;
        const auto pt = world.powertrain_state(vid);
        const auto dbg = world.powertrain_debug(vid);
        const double boost = pt.engines.empty() ? 0.0 : pt.engines.front().boost_bar;
        const double omega = pt.engines.empty() ? 0.0 : pt.engines.front().omega;
        const double kw = std::max(0.0, static_cast<double>(dbg.engine_torque_demand) * omega / 1000.0);
        r.peak_kw = std::max(r.peak_kw, kw);
        r.peak_boost_bar = std::max(r.peak_boost_bar, boost);
        r.peak_oil_c = std::max(r.peak_oil_c, static_cast<double>(pt.thermal.oil_c));
        if (r.seized_at_s < 0.0) r.peak_oil_running_c = std::max(r.peak_oil_running_c, static_cast<double>(pt.thermal.oil_c));
        if (r.t_oil_150 < 0.0 && pt.thermal.oil_c >= 150.0) r.t_oil_150 = t;
        if (r.t_oil_170 < 0.0 && pt.thermal.oil_c >= 170.0) r.t_oil_170 = t;
        const rg::DamageReport d = session.damage_report();
        const ps::Motion mo = world.get_motion(session.chassis_body());
        const double kmh = std::sqrt(mo.linear.x * mo.linear.x + mo.linear.y * mo.linear.y) * 3.6;
        const bool tick5 = s % (240 * 5) == 0;
        const bool seized_now = d.seized && r.seized_at_s < 0.0;
        if (tick5 || seized_now)
            std::printf("DAMAGE_RUN %s t %6.1f s  %5.0f km/h  %5.0f rpm  crank %5.0f kW  boost %5.2f bar  coolant %6.1f  oil %6.1f  friction %6.1f kW x%.2f  "
                        "radiator %.3f fan %.3f gasket %.3f crank %.3f pistons %.3f%s\n",
                        label, t, kmh, static_cast<double>(pt.engine_rpm), kw, boost, static_cast<double>(pt.thermal.coolant_c),
                        static_cast<double>(pt.thermal.oil_c), static_cast<double>(pt.thermal.friction_heat_w) / 1000.0, d.applied.friction,
                        d.state.health_of("radiator"), d.state.health_of("radiator_fan"), d.state.health_of("head_gasket"),
                        d.state.health_of("crankshaft"), d.state.health_of("pistons"), seized_now ? "  SEIZED" : "");
        if (seized_now) {
            r.seized_at_s = t;
            r.oil_at_seize_c = pt.thermal.oil_c;
            r.seized_part = d.seized_part;
            world.set_control("throttle", 0.0);
        }
        if (r.seized_at_s >= 0.0) {
            after_seize += 1.0 / 240.0;
            if (after_seize > 10.0) world.set_control("starter", 1.0);
            if (after_seize > 10.0 && pt.engine_state == EngineState::Running) r.restarted_while_seized = true;
            if (after_seize > 13.0) break;
        }
        if (mo.linear.x > 0.0 && world.get_pose(session.chassis_body()).position.x > 39000.0) break;
    }
    r.end_t_s = t;
    std::printf("DAMAGE_RUN %s end %.1f s: peak crank %.0f kW, peak boost %.2f bar, peak oil %.1f C while running, %.1f C in all (150 C at %.1f s, 170 C at %.1f s); seized %s at %.1f s with oil %.1f C; restart while seized %s\n",
                label, r.end_t_s, r.peak_kw, r.peak_boost_bar, r.peak_oil_running_c, r.peak_oil_c, r.t_oil_150, r.t_oil_170,
                r.seized_part.empty() ? "-" : r.seized_part.c_str(), r.seized_at_s, r.oil_at_seize_c, r.restarted_while_seized ? "YES" : "no");
    return r;
}

rg::VehicleSetup setup_of(const std::string& vehicle, std::vector<std::pair<std::string, rg::SetupValue>> values) {
    rg::VehicleSetup s;
    s.vehicle_id = vehicle;
    for (auto& kv : values) s.values[kv.first] = kv.second;
    return s;
}

} // namespace

TEST_CASE("damage engine: the owner's 2 MW / 11 bar hypercar breaks its engine before the oil reaches 170 C", "[.][damage][long]") {
    // The owner's report: car_hyper at about 2 MW and 11 bar reached about 190 C oil within 3 minutes. Reproduced as far
    // as the garage allows: turbo_install hyper_twin_large (or the 120 mm single) and turbo_boost_target 11 / 1.8 (the
    // option scales the stock 1.8 bar target), full throttle from standstill on a 40 km strip, 30 C. Each case runs twice:
    // "frozen" (the start state applied, no wear rules: what the oil does without the model) and live.
    Garage g;
    const double ambient = 30.0;
    const double boost_factor = 11.0 / 1.8;
    rg::DamageState dead_cooling;
    dead_cooling.health["radiator"] = 0.0;
    dead_cooling.health["radiator_fan"] = 0.0;
    struct Case {
        const char* label;
        const char* vehicle;
        std::vector<std::pair<std::string, rg::SetupValue>> setup;
        bool sport_nitrous;
        rg::DamageState start;
    };
    const std::vector<Case> cases = {
        {"hyper_11bar", "car_hyper", {{"turbo_install", std::string("hyper_twin_large")}, {"turbo_boost_target", boost_factor}}, false, {}},
        {"hyper_single120_11bar", "car_hyper", {{"turbo_install", std::string("hyper_single_120mm")}, {"turbo_boost_target", boost_factor}}, false, {}},
        {"hyper_n2o_11bar", "car_hyper_n2o", {{"turbo_install_n2o", std::string("hyper_twin_large_n2o")}, {"turbo_boost_target", boost_factor}}, true, {}},
        {"hyper_11bar_dead_radiator", "car_hyper", {{"turbo_install", std::string("hyper_twin_large")}, {"turbo_boost_target", boost_factor}}, false, dead_cooling},
    };
    const char* only = std::getenv("RG_DAMAGE_CASE");
    const bool with_frozen = std::getenv("RG_DAMAGE_NO_PLAIN") == nullptr;
    for (const Case& c : cases) {
        if (only != nullptr && std::string(only) != c.label) continue;
        const rg::CatalogEntry& e = g.entry(c.vehicle);
        rg::ScopedWorkDir d(g.ctx.work_root, c.label);
        const rg::MaterialisedSetup m = rg::materialise_setup(e, g.table, g.ctx, setup_of(c.vehicle, c.setup), d.path());
        INFO(m.error);
        REQUIRE(m.ok);
        OwnerRun frozen;
        frozen.peak_oil_running_c = 999.0;
        if (with_frozen) {
            auto fz = std::make_shared<rg::SessionConfig::Damage>();
            fz->config = shipped();
            fz->config.rules.clear(); // the start state acts, nothing wears
            fz->state = c.start;
            const std::string label = std::string(c.label) + "_frozen";
            frozen = owner_run(label.c_str(), e, m.vehicle_path, fz, ambient, 180.0, c.sport_nitrous);
        }
        if (std::getenv("RG_DAMAGE_PLAIN_ONLY") != nullptr) continue;
        const OwnerRun live = owner_run(c.label, e, m.vehicle_path, damage_of(c.start), ambient, 180.0, c.sport_nitrous);
        CHECK(live.peak_oil_running_c < 170.0);
        CHECK_FALSE(live.restarted_while_seized);
        // Only a run whose oil passes the bearings' limit without the model tests the owner's claim.
        if (frozen.peak_oil_running_c >= 150.0) CHECK(live.seized_at_s > 0.0);
    }
}
