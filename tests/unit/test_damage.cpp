// test_damage.cpp - the damage model, first piece (rg/damage.h; owner: "yes" to a radiator / fan damage part).
//
// Owner rules under test: damage persists until repaired (state file round trip, a Session started with a damaged state
// drives damaged), repairs are free and explicit (repair() / Session::request_damage_repair / Garage::repair_damage),
// damage changes how the car drives (a worn radiator runs the coolant hotter through physics_sim's
// World::set_engine_cooling_capacity). Plus: deterministic (1 vs 4 workers), and a healthy car is bit-identical to no
// damage model at all.
#include "rg/damage.h"
#include "rg/engine_thermal_info.h"
#include "rg/garage.h"
#include "rg/session.h"

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

using Catch::Matchers::ContainsSubstring;
namespace fs = std::filesystem;

namespace {

const std::string kRoot = RG_SOURCE_DIR;

rg::DamageConfig shipped() {
    std::string err;
    auto c = rg::load_damage_config(kRoot + "/data/vehicles/damage.json", &err);
    INFO(err);
    REQUIRE(c.has_value());
    return *c;
}

struct TempDir {
    fs::path path;
    explicit TempDir(const std::string& name) : path(fs::temp_directory_path() / ("rg_damage_" + name)) {
        std::error_code ec;
        fs::remove_all(path, ec);
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
};

// A minimal config: one part per effect, one absolute rule each.
const char* kMiniConfig = R"({
  "format": "rg.damage/1", "source": "test",
  "parts": [
    {"id": "radiator", "label": "Radiator", "effect": "radiator_ua", "min_scale": 0.5, "source": "t"},
    {"id": "radiator_fan", "label": "Fan", "effect": "fan_flow", "min_scale": 0.0, "source": "t"}
  ],
  "rules": [
    {"id": "boil", "part": "radiator", "signal": "coolant_c", "above_c": 120.0, "after_s": 2.0, "rate_per_s": 0.1, "rate_per_s_per_k": 0.01, "source": "t"},
    {"id": "fan", "part": "radiator_fan", "signal": "coolant_c", "above_overheat_k": 0.0, "after_s": 0.0, "rate_per_s": 0.05, "only_while_fan_on": true, "source": "t"}
  ]
})";

rg::DamageConfig mini() {
    std::string err;
    auto c = rg::parse_damage_config(kMiniConfig, "mini", &err);
    INFO(err);
    REQUIRE(c.has_value());
    return *c;
}

// A flat Session of the game's own thermal car (data/vehicles/car_sedan.json, a torque-map engine: no maps to generate).
rg::SessionConfig thermal_car(unsigned workers = 2) {
    rg::SessionConfig config;
    config.vehicle_json_path = kRoot + "/data/vehicles/car_sedan.json";
    config.surface_table_path = kRoot + "/data/surfaces/surfaces.json";
    config.job_workers = workers;
    return config;
}

std::shared_ptr<const rg::SessionConfig::Damage> damage_of(const rg::DamageState& state) {
    auto d = std::make_shared<rg::SessionConfig::Damage>();
    d->config = shipped();
    d->state = state;
    return d;
}

struct HotIdle {
    std::uint64_t hash = 0;
    double coolant_c = 0.0;
    rg::DamageReport report;
};
// Engine running at part throttle in neutral, coolant started hot (thermostat open, fan on), `seconds` later.
HotIdle hot_idle(rg::SessionConfig config, double start_coolant_c, double seconds) {
    rg::Session session(config);
    ps::World& w = session.world();
    const auto vid = session.vehicle_id();
    w.set_control("ignition", 1.0);
    w.set_control("starter", 1.0);
    w.set_control("clutch", 1.0);
    w.set_control("brake", 1.0);
    for (int i = 0; i < 240 * 2; ++i) session.step();
    w.set_control("starter", 0.0);
    w.set_control("throttle", 0.35);
    w.set_engine_thermal_state(vid, ps::drivetrain::EngineThermalState{start_coolant_c, 100.0});
    const long steps = static_cast<long>(seconds * 240.0);
    for (long i = 0; i < steps; ++i) session.step();
    HotIdle r;
    r.hash = w.state_hash();
    r.coolant_c = w.powertrain_state(vid).thermal.coolant_c;
    r.report = session.damage_report();
    return r;
}

} // namespace

// ---- config + state files -----------------------------------------------------------------------------------------------

TEST_CASE("damage: the shipped damage.json loads and names the cooling parts", "[damage]") {
    const rg::DamageConfig c = shipped();
    REQUIRE(c.find("radiator") != nullptr);
    REQUIRE(c.find("radiator_fan") != nullptr);
    REQUIRE(c.find("head_gasket") != nullptr);
    CHECK(c.find("radiator")->effect == rg::DamageEffect::RadiatorUa);
    CHECK(c.find("radiator_fan")->effect == rg::DamageEffect::FanFlow);
    CHECK_FALSE(c.rules.empty());
    for (const rg::DamageRuleConfig& r : c.rules) CHECK(c.find(r.part) != nullptr);
}

TEST_CASE("damage: the loader rejects malformed configs with a reason", "[damage]") {
    const auto bad = [](const std::string& text) {
        std::string err;
        const bool ok = rg::parse_damage_config(text, "x", &err).has_value();
        CHECK_FALSE(ok);
        return err;
    };
    CHECK_THAT(bad("[]"), ContainsSubstring("not a JSON object"));
    CHECK_THAT(bad(R"({"format":"rg.damage/2","parts":[]})"), ContainsSubstring("rg.damage/1"));
    std::string t = kMiniConfig;
    CHECK_THAT(bad(std::string(t).replace(t.find("\"radiator_ua\""), 13, "\"boost\"")), ContainsSubstring("effect"));
    CHECK_THAT(bad(std::string(t).replace(t.find("\"part\": \"radiator\""), 18, "\"part\": \"turbo\"")), ContainsSubstring("part"));
    CHECK_THAT(bad(std::string(t).replace(t.find("\"above_c\": 120.0"), 16, "\"above_c\": 120.0, \"above_overheat_k\": 1")),
               ContainsSubstring("exactly one"));
    CHECK_THAT(bad(std::string(t).replace(t.find("\"source\": \"test\""), 16, "\"source\": \"test\", \"colour\": 1")), ContainsSubstring("unknown key"));
}

TEST_CASE("damage: the state file round-trips, keeps unknown parts, and a healthy state removes it", "[damage]") {
    TempDir dir("state");
    const std::string path = rg::damage_state_path(dir.path.generic_string(), "car_x");
    CHECK_THAT(path, ContainsSubstring("garage/damage/car_x.json"));
    std::string err;
    // Missing file = a healthy car.
    auto none = rg::load_damage_state(path, &err);
    REQUIRE(none.has_value());
    CHECK(none->all_healthy());

    rg::DamageState s;
    s.health["radiator"] = 0.625;
    s.health["future_turbo"] = 0.5; // a part a newer build knows: kept
    REQUIRE(rg::save_damage_state(path, s, &err));
    REQUIRE(fs::exists(path));
    auto back = rg::load_damage_state(path, &err);
    INFO(err);
    REQUIRE(back.has_value());
    CHECK(back->health_of("radiator") == 0.625);
    CHECK(back->health_of("future_turbo") == 0.5);
    CHECK(back->health_of("radiator_fan") == 1.0);

    // A repair (healthy state) removes the file.
    REQUIRE(rg::save_damage_state(path, rg::DamageState{}, &err));
    CHECK_FALSE(fs::exists(path));

    // A broken file is an error, not silently healthy.
    fs::create_directories(fs::path(path).parent_path());
    std::ofstream(path) << "{\"format\":\"rg.damage_state/1\",\"parts\":{\"radiator\":1.5}}";
    CHECK_FALSE(rg::load_damage_state(path, &err).has_value());
    CHECK_THAT(err, ContainsSubstring("[0, 1]"));
}

TEST_CASE("damage: cooling scales are exactly 1 while healthy and follow min_scale when worn", "[damage]") {
    const rg::DamageConfig c = mini();
    rg::DamageState s;
    CHECK(rg::cooling_scales(c, s).unity());
    s.health["radiator"] = 1.0;
    CHECK(rg::cooling_scales(c, s).unity());
    s.health["radiator"] = 0.5;      // 0.5 + 0.5 x 0.5
    s.health["radiator_fan"] = 0.25; // 0 + 1 x 0.25
    const rg::CoolingScales k = rg::cooling_scales(c, s);
    CHECK(k.radiator_ua == 0.75);
    CHECK(k.fan_flow == 0.25);
    const rg::DamageView v = rg::damage_view(c, s);
    CHECK(v.any_damaged);
    CHECK_THAT(rg::damage_warning_line(v), ContainsSubstring("Radiator 50 %"));
    CHECK_THAT(rg::damage_warning_line(v), ContainsSubstring("repair in the garage"));
    CHECK(rg::damage_warning_line(rg::damage_view(c, rg::DamageState{})).empty());
}

// ---- accumulation ------------------------------------------------------------------------------------------------------

TEST_CASE("damage: wear starts after the dwell, grows with the excess, stops below the threshold and persists", "[damage]") {
    rg::DamageModel m(mini(), rg::DamageState{}, /*overheat_c=*/118.0);
    const double dt = 0.5;
    rg::DamageInputs hot{true, 125.0, 110.0, false}; // 5 K above the boil rule; fan off: the fan rule waits
    // Dwell 2 s: the first four half-second ticks (dwell 0.5 .. 2.0) - wear begins on the tick the dwell reaches 2 s.
    CHECK_FALSE(m.step(dt, hot));
    CHECK_FALSE(m.step(dt, hot));
    CHECK_FALSE(m.step(dt, hot));
    CHECK(m.step(dt, hot));
    const double h1 = m.state().health_of("radiator");
    CHECK(std::abs(h1 - (1.0 - (0.1 + 0.01 * 5.0) * dt)) < 1e-12);
    CHECK(m.state().health_of("radiator_fan") == 1.0);
    // Below the threshold: no wear, the dwell resets, and the damage stays (no self-healing).
    rg::DamageInputs cool{true, 100.0, 100.0, true};
    CHECK_FALSE(m.step(dt, cool));
    CHECK(m.dwell_s(0) == 0.0);
    CHECK(m.state().health_of("radiator") == h1);
    // Hotter wears faster.
    rg::DamageModel a(mini(), rg::DamageState{}, 118.0), b(mini(), rg::DamageState{}, 118.0);
    for (int i = 0; i < 8; ++i) {
        a.step(dt, rg::DamageInputs{true, 122.0, 100.0, false});
        b.step(dt, rg::DamageInputs{true, 140.0, 100.0, false});
    }
    CHECK(b.state().health_of("radiator") > 0.0);
    CHECK(b.state().health_of("radiator") < a.state().health_of("radiator"));
    // The relative fan rule needs the fan on; health never goes below 0.
    rg::DamageModel f(mini(), rg::DamageState{}, 118.0);
    for (int i = 0; i < 100; ++i) f.step(dt, rg::DamageInputs{true, 119.0, 100.0, true});
    CHECK(f.state().health_of("radiator_fan") == 0.0);
    CHECK(f.scales().fan_flow == 0.0);
    // No thermal model: nothing happens.
    rg::DamageModel n(mini(), rg::DamageState{}, 118.0);
    CHECK_FALSE(n.step(dt, rg::DamageInputs{false, 200.0, 200.0, true}));
    // Repair: everything back to 1, timers cleared.
    m.repair();
    CHECK(m.state().all_healthy());
    CHECK(m.scales().unity());
    CHECK(m.dwell_s(0) == 0.0);
}

// ---- Session: no-op, driving effect, determinism, repair ------------------------------------------------------------

TEST_CASE("damage: a healthy car with the damage model is bit-identical to no damage model", "[damage][session]") {
    // Thermal car: the model runs every tick but never calls the physics while every part is healthy (no overheat here).
    rg::SessionConfig base = thermal_car();
    rg::SessionConfig with = thermal_car();
    with.damage = damage_of(rg::DamageState{});
    const HotIdle a = hot_idle(base, 95.0, 5.0);
    const HotIdle b = hot_idle(with, 95.0, 5.0);
    CHECK(a.hash == b.hash);
    CHECK(a.coolant_c == b.coolant_c);
    CHECK(b.report.present);
    CHECK(b.report.thermal);
    CHECK(b.report.applied.unity());
    CHECK(b.report.revision == 0);
    // The hash_check car (physics_sim's car_sedan, no cooling_system): the model is present, nothing wears or is applied.
    rg::SessionConfig plain;
    plain.vehicle_json_path = kRoot + "/external/physics_sim/data/vehicles/car_sedan.json";
    plain.surface_table_path = kRoot + "/external/physics_sim/data/surfaces/surfaces.json";
    rg::SessionConfig plain_d = plain;
    rg::DamageState worn;
    worn.health["radiator"] = 0.2;
    plain_d.damage = damage_of(worn);
    rg::Session s0(plain), s1(plain_d);
    for (int i = 0; i < 480; ++i) {
        s0.step();
        s1.step();
    }
    CHECK(s0.world().state_hash() == s1.world().state_hash());
    CHECK_FALSE(s1.damage_report().thermal);
    CHECK(s1.damage_report().applied.unity());
    CHECK(s1.damage_report().state.health_of("radiator") == 0.2); // kept as loaded, not lost
}

TEST_CASE("damage: a worn radiator and fan run the coolant hotter, the same with 1 and 4 workers", "[damage][session]") {
    rg::DamageState worn;
    worn.health["radiator"] = 0.1;
    worn.health["radiator_fan"] = 0.1;
    rg::SessionConfig healthy = thermal_car(1);
    healthy.damage = damage_of(rg::DamageState{});
    rg::SessionConfig damaged1 = thermal_car(1), damaged4 = thermal_car(4);
    damaged1.damage = damage_of(worn);
    damaged4.damage = damage_of(worn);
    const HotIdle h = hot_idle(healthy, 104.0, 20.0);
    const HotIdle d1 = hot_idle(damaged1, 104.0, 20.0);
    const HotIdle d4 = hot_idle(damaged4, 104.0, 20.0);
    std::printf("DAMAGE coolant after 20 s at part throttle: healthy %.2f C, radiator + fan at 10 %% %.2f C (applied UA x%.3f, fan x%.3f)\n",
                h.coolant_c, d1.coolant_c, d1.report.applied.radiator_ua, d1.report.applied.fan_flow);
    CHECK(d1.report.applied.radiator_ua < 0.5);
    CHECK(d1.report.applied.fan_flow < 0.2);
    CHECK(d1.coolant_c > h.coolant_c + 0.5);
    CHECK(d1.hash != h.hash);
    CHECK(d1.hash == d4.hash);
    CHECK(d1.coolant_c == d4.coolant_c);
}

TEST_CASE("damage: an overheating engine wears its parts in the Session, deterministically, and a repair resets them", "[damage][session]") {
    // Coolant forced far past the overheat warning: the head-gasket rule (overheat + 12 K for 5 s) and the radiator rule
    // (overheat + 4 K for 15 s) wear while it stays there.
    const auto run = [](unsigned workers) {
        rg::SessionConfig c = thermal_car(workers);
        rg::DamageState start;
        start.health["radiator"] = 0.3; // already worn: it stays hot for long enough
        start.health["radiator_fan"] = 0.0;
        c.damage = damage_of(start);
        return hot_idle(c, 150.0, 20.0);
    };
    const HotIdle a = run(1), b = run(4);
    std::printf("DAMAGE forced overheat (150 C start): coolant after 20 s %.1f C, head gasket %.4f, radiator %.4f, revision %llu\n",
                a.coolant_c, a.report.state.health_of("head_gasket"), a.report.state.health_of("radiator"),
                static_cast<unsigned long long>(a.report.revision));
    CHECK(a.report.revision > 0);
    CHECK(a.report.state.health_of("head_gasket") < 1.0);
    CHECK(a.report.state.health_of("radiator") <= 0.3);
    CHECK(a.hash == b.hash);
    CHECK(a.report.state.health == b.report.state.health);

    // Repair while the world runs: healthy at once in the report, the physics back to (1, 1) after the next tick.
    rg::SessionConfig c = thermal_car();
    rg::DamageState worn;
    worn.health["radiator"] = 0.4;
    c.damage = damage_of(worn);
    rg::Session s(c);
    s.step();
    REQUIRE_FALSE(s.damage_report().applied.unity());
    const std::uint64_t rev = s.damage_report().revision;
    s.request_damage_repair();
    CHECK(s.damage_report().state.all_healthy());
    CHECK(s.damage_report().revision > rev);
    s.step(); // the repair is processed after this tick's World::step and queued for the next tick boundary
    CHECK(s.damage_report().applied.unity());
    CHECK(s.damage_report().state.all_healthy());
    s.step();
    CHECK(s.world().powertrain_state(s.vehicle_id()).thermal.radiator_ua_scale == 1.0);
}

TEST_CASE("damage: the garage hands the paths to the drive, reads the saved state and repairs for free", "[damage][garage]") {
    TempDir user("garage_user");
    TempDir work("garage_work");
    rg::GarageConfig gc;
    gc.repo_root = kRoot;
    gc.catalog_path = kRoot + "/data/vehicles/catalog.json";
    gc.options_path = kRoot + "/data/vehicles/setup_options.json";
    gc.set_path = kRoot + "/data/garage/garage_set.json";
    gc.tyre_families_path = kRoot + "/data/tyres/families.json";
    gc.user_dir = user.path.generic_string();
    gc.work_root = work.path.generic_string();
    std::string err;
    auto g = rg::Garage::open(gc, &err);
    INFO(err);
    REQUIRE(g != nullptr);
    REQUIRE(g->damage_config() != nullptr);
    const std::string id = "car_sedan";
    const std::string path = g->damage_state_path(id);
    CHECK(path == rg::damage_state_path(gc.user_dir, id));
    CHECK(g->damage_state(id).all_healthy());

    rg::DamageState s;
    s.health["radiator_fan"] = 0.4;
    REQUIRE(rg::save_damage_state(path, s, &err));
    CHECK(g->damage_state(id).health_of("radiator_fan") == 0.4);
    const rg::DriveSelection d = g->prepare_drive(id);
    CHECK(d.damage_state_path == path);
    CHECK_THAT(d.damage_config_path, ContainsSubstring("damage.json"));

    // The drive loads it (what RgSimulation does with the two paths) and drives damaged.
    auto cfg = rg::load_damage_config(d.damage_config_path, &err);
    auto st = rg::load_damage_state(d.damage_state_path, &err);
    REQUIRE(cfg.has_value());
    REQUIRE(st.has_value());
    CHECK(rg::cooling_scales(*cfg, *st).fan_flow < 1.0);

    // Repair: free, the file is gone, the car is healthy.
    REQUIRE(g->repair_damage(id, &err));
    CHECK_FALSE(fs::exists(path));
    CHECK(g->damage_state(id).all_healthy());
    CHECK_FALSE(g->repair_damage("no_such_car", &err));
    g->cleanup();
}
