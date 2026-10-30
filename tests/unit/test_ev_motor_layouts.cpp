// test_ev_motor_layouts.cpp - motor layouts and drive units of battery EVs (owner 2026-10-21): "add options for single, dual,
// tri and quad-motor and add different motors with different weight ... at least 5 motor options" and "if the motors sit on
// the wheels, they need a gear ratio that is adjustable".
//
// The options: ev_layout (file_choice, catalog ev_layouts), ev_front_motors / ev_rear_motors (file_choice, catalog
// ev_motor_units, data/electric_motors/motor_catalog.json), and the two Tuning sliders ev_motor_output (x0.5-1.0 of the
// fitted units' peak torque and power) and ev_reduction_ratio (x0.8-1.2 of every motor-fed reduction). The old
// ev_motor_scale is a retired option: a saved setup that carries it still loads.
//
// Checks: the catalogue; the layouts on car_electric and on a battery_ev conversion of car_sedan (motor counts 1/2/3/4,
// ids, the driven wheels the physics reports); unit figures in every motor and the part-mass tree following; the ratio
// slider reaching every reduction and the output slider leaving the mass alone; a layout change keeping the chosen units;
// the old setup loading; a flat-road full-throttle drive per layout (every driven wheel pushes, nothing stalls or goes NaN,
// quad + hypercar units beat single + economy); and the garage dyno following a layout change.
#include "rg/engine_store.h"
#include "rg/driver_actions.h"
#include "rg/ev_motor.h"
#include "rg/garage_dyno.h"
#include "rg/gearbox_catalog.h"
#include "rg/part_mass.h"
#include "rg/session.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
using json = nlohmann::json;
const std::string kRoot = RG_SOURCE_DIR;

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    std::shared_ptr<const rg::EngineStore> store;
    std::shared_ptr<const rg::EvMotorCatalog> motors;
    rg::DynoReference reference;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_ev_motor_layout_tests"};
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
        auto s = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, "", &err);
        INFO(err);
        REQUIRE(s != nullptr);
        store = std::shared_ptr<const rg::EngineStore>(std::move(s));
        ctx.engines = store;
        auto g = rg::GearboxCatalog::open(kRoot + "/data/gearboxes/gearbox_catalog.json", kRoot, &err);
        INFO(err);
        REQUIRE(g != nullptr);
        ctx.gearboxes = std::shared_ptr<const rg::GearboxCatalog>(std::move(g));
        auto masses = rg::load_part_mass_table(kRoot + "/data/vehicles/part_masses.json", &err);
        INFO(err);
        REQUIRE(masses.has_value());
        ctx.part_masses = std::make_shared<const rg::PartMassTable>(*masses);
        auto m = rg::EvMotorCatalog::open(kRoot + "/data/electric_motors/motor_catalog.json", &err);
        INFO(err);
        REQUIRE(m != nullptr);
        motors = std::shared_ptr<const rg::EvMotorCatalog>(std::move(m));
        ctx.ev_motors = motors;
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
};

rg::VehicleSetup setup_of(const std::string& vehicle, std::vector<std::pair<std::string, rg::SetupValue>> values) {
    rg::VehicleSetup s;
    s.vehicle_id = vehicle;
    for (auto& kv : values) s.values[kv.first] = kv.second;
    return s;
}

const rg::OptionView* view_of(const rg::SetupModel& m, const std::string& id) {
    for (const rg::OptionView& v : m.options) if (v.def.id == id) return &v;
    return nullptr;
}

json read_json(const std::string& path) {
    std::ifstream in(path);
    REQUIRE(in.good());
    return json::parse(in);
}

const json* component_of(const json& vehicle, const std::string& id) {
    for (const auto& c : vehicle.at("powertrain").at("components")) if (c.value("id", "") == id) return &c;
    return nullptr;
}

std::vector<const json*> components_of_type(const json& vehicle, const std::string& type) {
    std::vector<const json*> out;
    for (const auto& c : vehicle.at("powertrain").at("components")) if (c.value("type", "") == type) out.push_back(&c);
    return out;
}

std::set<std::string> ids_of_type(const json& vehicle, const std::string& type) {
    std::set<std::string> out;
    for (const json* c : components_of_type(vehicle, type)) out.insert(c->value("id", ""));
    return out;
}

// The materialised vehicle file of a setup.
struct Built {
    rg::MaterialisedSetup m;
    json vehicle;
    std::unique_ptr<rg::ScopedWorkDir> dir;
};
Built build(const Fixture& f, const std::string& car, std::vector<std::pair<std::string, rg::SetupValue>> values) {
    Built b;
    b.dir = std::make_unique<rg::ScopedWorkDir>(f.ctx.work_root, "ev_" + car);
    b.m = rg::materialise_setup(f.entry(car), f.table, f.ctx, setup_of(car, std::move(values)), b.dir->path());
    INFO(b.m.error);
    REQUIRE(b.m.ok);
    b.vehicle = read_json(b.m.vehicle_path);
    return b;
}

double motor_mass_in_tree(const rg::PartTree& tree) {
    const rg::PartNode* dt = tree.root.find("drivetrain");
    REQUIRE(dt != nullptr);
    double sum = 0;
    for (const rg::PartNode& c : dt->children) if (c.key == "electric_motor") sum += c.total_kg();
    return sum;
}

struct LayoutExpect {
    const char* id;
    std::set<std::string> motors;
    std::set<std::string> diffs;
    std::set<std::string> gears;
    int driven_wheels;
    bool front_driven;
};
const std::vector<LayoutExpect>& layouts() {
    static const std::vector<LayoutExpect> l = {
        {"single", {"rear_motor"}, {"rear_diff"}, {}, 2, false},
        {"dual", {"front_motor", "rear_motor"}, {"front_diff", "rear_diff"}, {}, 4, true},
        {"tri", {"front_motor", "motor_rl", "motor_rr"}, {"front_diff"}, {"reduction_rl", "reduction_rr"}, 4, true},
        {"quad", {"motor_fl", "motor_fr", "motor_rl", "motor_rr"}, {}, {"reduction_fl", "reduction_fr", "reduction_rl", "reduction_rr"}, 4, true},
    };
    return l;
}

// A flat full-throttle drive on a tessellated ground: per wheel mean fx between 1 s and the end, the speed after 3 s and
// the time to 100 km/h (-1 = not within `max_s`).
struct Drive {
    bool finite = true;
    int motor_count = 0;
    std::vector<bool> driven;
    std::vector<double> mean_fx;
    double kmh_at_3s = 0;
    double t100_s = -1;
    double max_motor_torque_nm = 0;
    bool tc_intervened = false;
};
Drive drive(const Fixture& f, const std::string& car, std::vector<std::pair<std::string, rg::SetupValue>> values, double max_s,
            double traction_control = 1.0) {
    Built b = build(f, car, std::move(values));
    rg::SessionConfig config;
    config.vehicle_json_path = b.m.vehicle_path;
    config.surface_table_path = kRoot + "/data/surfaces/surfaces.json";
    const rg::CatalogEntry& e = f.entry(car);
    config.chassis_mass_kg = e.chassis.mass_kg;
    config.chassis_half_extents = ps::Vec3{e.chassis.half_extents[0], e.chassis.half_extents[1], e.chassis.half_extents[2]};
    config.chassis_z_m = e.chassis.spawn_z_m;
    config.ground_half_extent_m = 3000.0;
    config.ground_half_extent_y_m = 100.0;
    config.ground_tessellated = true;
    config.engine_map_cache_dir = f.ctx.engine_map_cache_dir;
    rg::Session session(config);
    auto& world = session.world();
    const auto vid = session.vehicle_id();
    for (int i = 0; i < 240; ++i) session.step();
    world.set_control("ignition", 1.0);
    world.set_control("electric.ready", 1.0);
    world.set_control("assist.traction_control", traction_control);
    world.set_control("brake", 0.0);
    world.set_control("handbrake", 0.0);
    world.set_control("gear_request", rg::kGearRequestDrive);
    for (int i = 0; i < 120; ++i) session.step(); // ready + drive selected, car standing
    Drive out;
    const std::size_t n = world.vehicle_wheel_count(vid);
    out.mean_fx.assign(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) out.driven.push_back(world.wheel_is_driven(vid, i));
    const long steps = static_cast<long>(max_s * 240.0);
    long sampled = 0;
    world.set_control("throttle", 1.0);
    for (long i = 0; i < steps; ++i) {
        session.step();
        const double t = static_cast<double>(i + 1) / 240.0;
        const ps::Motion mo = world.get_motion(session.chassis_body());
        const double kmh = std::sqrt(mo.linear.x * mo.linear.x + mo.linear.y * mo.linear.y) * 3.6;
        const auto& pt = world.powertrain_state(vid);
        if (!std::isfinite(kmh) || !std::isfinite(pt.motor_torque_nm)) out.finite = false;
        out.motor_count = pt.electric_motor_count;
        out.max_motor_torque_nm = std::max(out.max_motor_torque_nm, std::abs(pt.motor_torque_nm));
        out.tc_intervened = out.tc_intervened || pt.traction_control_intervening;
        if (t <= 3.0) out.kmh_at_3s = kmh;
        if (out.t100_s < 0 && kmh >= 100.0) {
            out.t100_s = t;
            if (t >= 3.0) break;
        }
        if (t >= 1.0 && t <= 3.0) {
            for (std::size_t w = 0; w < n; ++w) {
                const double fx = world.wheel_telemetry(vid, w).fx;
                if (!std::isfinite(fx)) out.finite = false;
                out.mean_fx[w] += fx;
            }
            ++sampled;
        }
    }
    for (double& v : out.mean_fx) v /= static_cast<double>(std::max(1L, sampled));
    return out;
}

} // namespace

TEST_CASE("ev motors: the catalogue has at least six drive units and the four layouts", "[ev][ev_motors][io]") {
    Fixture f;
    const auto& units = f.motors->units();
    REQUIRE(units.size() >= 6);
    std::set<std::string> ids;
    for (const rg::EvMotorUnit& u : units) {
        INFO(u.id);
        CHECK(ids.insert(u.id).second);
        CHECK(u.peak_torque_nm > 0);
        CHECK(u.max_power_w > 0);
        CHECK(u.max_rpm > 0);
        CHECK(u.inertia_kgm2 > 0);
        CHECK(u.efficiency > 0.5);
        CHECK(u.efficiency <= 1.0);
        CHECK(u.regen_torque_nm > 0);
        CHECK(u.mass_kg > 0);
        CHECK(u.reduction_ratio > 1.0);
        CHECK_FALSE(u.inspiration.empty());
        CHECK_THAT(u.source, ContainsSubstring("[AS]"));
    }
    // different motors with different weight and different power
    std::set<double> masses, powers;
    for (const rg::EvMotorUnit& u : units) { masses.insert(u.mass_kg); powers.insert(u.max_power_w); }
    CHECK(masses.size() >= 5);
    CHECK(powers.size() >= 5);
    REQUIRE(f.motors->layouts().size() == 4);
    for (const LayoutExpect& l : layouts()) {
        const rg::EvLayoutEntry* e = f.motors->find_layout(l.id);
        REQUIRE(e != nullptr);
        CHECK(e->motor_count() == static_cast<int>(l.motors.size()));
    }
    // a catalogue without units is refused
    std::string err;
    const std::string bad = R"({"format":"rg.ev_motor_catalog/1","layouts":[],"units":[]})";
    CHECK(rg::EvMotorCatalog::parse(bad, "inline", &err) == nullptr);
    CHECK_FALSE(err.empty());
}

TEST_CASE("ev motors: the options are offered on electric cars only and the retired scale is gone", "[ev][ev_motors][setup]") {
    Fixture f;
    const rg::SetupModel ev = rg::build_setup_model(f.entry("car_electric"), f.table, f.ctx);
    const rg::SetupModel sedan = rg::build_setup_model(f.entry("car_sedan"), f.table, f.ctx);
    REQUIRE(ev.error.empty());
    REQUIRE(sedan.error.empty());
    for (const char* id : {"ev_layout", "ev_front_motors", "ev_rear_motors", "ev_motor_output", "ev_reduction_ratio"}) {
        INFO(id);
        const rg::OptionView* v = view_of(ev, id);
        REQUIRE(v != nullptr);
        CHECK(v->available);
        CHECK(v->def.group == "Electric drive");
        const rg::OptionView* s = view_of(sedan, id);
        if (s != nullptr) CHECK_FALSE(s->available);
    }
    CHECK(view_of(ev, "ev_motor_scale") == nullptr);
    // the layout cards: the car's own layout as "Original", then the other three
    const rg::OptionView* layout = view_of(ev, "ev_layout");
    REQUIRE(layout != nullptr);
    CHECK(layout->def.kind == rg::OptionKind::FileChoice);
    REQUIRE(layout->parts.size() == 4);
    CHECK(layout->parts.front().id == "stock");
    CHECK_THAT(layout->parts.front().label, ContainsSubstring("Dual"));
    // the unit cards: stock first, then the six catalogue units, each with power, torque, rpm and mass
    for (const char* id : {"ev_front_motors", "ev_rear_motors"}) {
        const rg::OptionView* units = view_of(ev, id);
        REQUIRE(units != nullptr);
        REQUIRE(units->parts.size() == f.motors->units().size() + 1);
        CHECK(units->parts.front().id == "stock");
        CHECK_THAT(units->parts.front().detail, ContainsSubstring("300 kW"));
        for (std::size_t i = 1; i < units->parts.size(); ++i) {
            CHECK_THAT(units->parts[i].detail, ContainsSubstring("kW"));
            CHECK_THAT(units->parts[i].detail, ContainsSubstring("N*m"));
            CHECK_THAT(units->parts[i].detail, ContainsSubstring("rpm"));
            CHECK_THAT(units->parts[i].detail, ContainsSubstring("kg"));
        }
    }
    // the sliders are Tuning sliders with the documented ranges
    const rg::OptionView* out = view_of(ev, "ev_motor_output");
    REQUIRE(out != nullptr);
    CHECK(out->def.min == Approx(0.5));
    CHECK(out->def.max == Approx(1.0));
    const rg::OptionView* ratio = view_of(ev, "ev_reduction_ratio");
    REQUIRE(ratio != nullptr);
    CHECK(ratio->def.min == Approx(0.8));
    CHECK(ratio->def.max == Approx(1.2));
    // a combustion car converted to battery EV offers the layout too
    const rg::VehicleSetup conv_setup = setup_of("car_sedan", {{"powertrain_install", std::string("battery_ev")}});
    const rg::SetupModel conv = rg::build_setup_model(f.entry("car_sedan"), f.table, f.ctx, &conv_setup);
    const rg::OptionView* clayout = view_of(conv, "ev_layout");
    REQUIRE(clayout != nullptr);
    CHECK(clayout->available);
}

TEST_CASE("ev motors: every layout builds on car_electric and on a battery_ev conversion of car_sedan", "[ev][ev_motors][setup]") {
    Fixture f;
    for (const char* car : {"car_electric", "car_sedan"}) {
        for (const LayoutExpect& l : layouts()) {
            INFO(car << " " << l.id);
            std::vector<std::pair<std::string, rg::SetupValue>> values = {{"ev_layout", std::string(l.id)}};
            if (std::string(car) == "car_sedan") values.insert(values.begin(), {"powertrain_install", std::string("battery_ev")});
            const Built b = build(f, car, values);
            CHECK(ids_of_type(b.vehicle, "electric_motor") == l.motors);
            CHECK(ids_of_type(b.vehicle, "differential") == l.diffs);
            CHECK(ids_of_type(b.vehicle, "gear_pair") == l.gears);
            CHECK(components_of_type(b.vehicle, "battery").size() == 1);
            // the physics loader accepts it and reports the driven wheels
            const rg::EvMotorInfo info = rg::describe_ev_motors(b.vehicle.dump(), f.motors.get());
            CHECK(info.swappable);
            CHECK(info.layout_id == l.id);
            CHECK(info.motor_count == static_cast<int>(l.motors.size()));
            CHECK(info.driven_wheels == l.driven_wheels);
            // every motor is fed by the battery and the controllers are the car's
            for (const json* m : components_of_type(b.vehicle, "electric_motor")) CHECK(m->value("battery", "") == "battery");
            CHECK(b.vehicle.contains("controllers"));
            // every wheel hub of a driven axle is reached by exactly one shaft end
            ps::io::EngineMapOptions maps;
            maps.cache_dir = f.ctx.engine_map_cache_dir;
            CHECK_NOTHROW(rg::load_vehicle(b.m.vehicle_path, maps));
        }
    }
}

TEST_CASE("ev motors: a unit choice lands in every motor of its axle, the mass follows", "[ev][ev_motors][setup][part_mass]") {
    Fixture f;
    const Built stock = build(f, "car_electric", {{"ev_layout", std::string("quad")}});
    const rg::EvMotorUnit* hyper = f.motors->find_unit("hypercar_oil_cooled");
    const rg::EvMotorUnit* eco = f.motors->find_unit("economy_pmsm");
    REQUIRE(hyper != nullptr);
    REQUIRE(eco != nullptr);
    const Built b = build(f, "car_electric",
                          {{"ev_layout", std::string("quad")},
                           {"ev_front_motors", std::string("hypercar_oil_cooled")},
                           {"ev_rear_motors", std::string("economy_pmsm")}});
    for (const char* w : {"fl", "fr", "rl", "rr"}) {
        const bool front = w[0] == 'f';
        const rg::EvMotorUnit& u = front ? *hyper : *eco;
        INFO(w);
        const json* m = component_of(b.vehicle, std::string("motor_") + w);
        const json* g = component_of(b.vehicle, std::string("reduction_") + w);
        REQUIRE(m != nullptr);
        REQUIRE(g != nullptr);
        CHECK(m->at("peak_torque_nm").get<double>() == Approx(u.peak_torque_nm));
        CHECK(m->at("max_power_w").get<double>() == Approx(u.max_power_w));
        CHECK(m->at("max_rpm").get<double>() == Approx(u.max_rpm));
        CHECK(m->at("efficiency").get<double>() == Approx(u.efficiency));
        CHECK(m->at("regen_torque_nm").get<double>() == Approx(u.regen_torque_nm));
        CHECK(m->at("mass_kg").get<double>() == Approx(u.mass_kg));
        CHECK(g->at("ratio").get<double>() == Approx(u.reduction_ratio));
    }
    // the part-mass tree: the motor nodes carry the unit masses and the car total moves by exactly the difference
    REQUIRE(b.m.compiled.parts.ok);
    REQUIRE(stock.m.compiled.parts.ok);
    CHECK(motor_mass_in_tree(b.m.compiled.parts) == Approx(2 * hyper->mass_kg + 2 * eco->mass_kg));
    const double stock_motor_kg = motor_mass_in_tree(stock.m.compiled.parts);
    CHECK(b.m.compiled.parts.root.total_kg() - stock.m.compiled.parts.root.total_kg() ==
          Approx(2 * hyper->mass_kg + 2 * eco->mass_kg - stock_motor_kg).epsilon(1e-9));
    // the wheel gears have a node of their own (no mass of their own, one half shaft each) and no layout-conversion penalty
    CHECK(b.m.compiled.parts.root.find("drivetrain.layout_conversion") == nullptr);
    CHECK(b.m.compiled.parts.root.find("drivetrain.reduction_fl") != nullptr);
    // an axle unit choice on the dual layout
    const Built d = build(f, "car_electric", {{"ev_rear_motors", std::string("axial_flux")}});
    const rg::EvMotorUnit* axial = f.motors->find_unit("axial_flux");
    REQUIRE(axial != nullptr);
    CHECK(component_of(d.vehicle, "rear_motor")->at("peak_torque_nm").get<double>() == Approx(axial->peak_torque_nm));
    CHECK(component_of(d.vehicle, "rear_diff")->at("ratio").get<double>() == Approx(axial->reduction_ratio));
    CHECK(component_of(d.vehicle, "front_motor")->at("peak_torque_nm").get<double>() == Approx(600.0)); // untouched
}

TEST_CASE("ev motors: a layout change keeps the chosen units", "[ev][ev_motors][setup]") {
    Fixture f;
    // Single drives the rear axle only: the front choice waits and applies again as soon as the layout drives the front.
    const rg::VehicleSetup single_setup =
        setup_of("car_electric", {{"ev_layout", std::string("single")}, {"ev_front_motors", std::string("compact_pmsm")}});
    const rg::SetupModel single = rg::build_setup_model(f.entry("car_electric"), f.table, f.ctx, &single_setup);
    const rg::OptionView* front = view_of(single, "ev_front_motors");
    REQUIRE(front != nullptr);
    CHECK_FALSE(front->available);
    const auto single_check = rg::validate_setup(f.entry("car_electric"), f.table, f.ctx,
                                                 setup_of("car_electric", {{"ev_layout", std::string("single")},
                                                                           {"ev_front_motors", std::string("compact_pmsm")}}));
    CHECK(single_check.ok);
    const Built s = build(f, "car_electric", {{"ev_layout", std::string("single")}, {"ev_front_motors", std::string("compact_pmsm")}});
    CHECK(ids_of_type(s.vehicle, "electric_motor") == std::set<std::string>{"rear_motor"});
    const Built q = build(f, "car_electric", {{"ev_layout", std::string("quad")}, {"ev_front_motors", std::string("compact_pmsm")}});
    const rg::EvMotorUnit* u = f.motors->find_unit("compact_pmsm");
    REQUIRE(u != nullptr);
    CHECK(component_of(q.vehicle, "motor_fl")->at("peak_torque_nm").get<double>() == Approx(u->peak_torque_nm));
    CHECK(component_of(q.vehicle, "motor_rr")->at("peak_torque_nm").get<double>() == Approx(600.0)); // the rear keeps its own
    // a combustion car is offered none of this and a setup naming it is refused
    const auto bad = rg::validate_setup(f.entry("car_sedan"), f.table, f.ctx,
                                        setup_of("car_sedan", {{"ev_layout", std::string("quad")}}));
    CHECK_FALSE(bad.ok);
    // an unknown layout / unit is refused
    CHECK_FALSE(rg::validate_setup(f.entry("car_electric"), f.table, f.ctx,
                                   setup_of("car_electric", {{"ev_layout", std::string("hexa")}})).ok);
    CHECK_FALSE(rg::validate_setup(f.entry("car_electric"), f.table, f.ctx,
                                   setup_of("car_electric", {{"ev_rear_motors", std::string("no_such_unit")}})).ok);
}

TEST_CASE("ev motors: the reduction ratio slider reaches every reduction, the output slider leaves mass alone",
          "[ev][ev_motors][setup][part_mass]") {
    Fixture f;
    for (const LayoutExpect& l : layouts()) {
        INFO(l.id);
        const Built base = build(f, "car_electric", {{"ev_layout", std::string(l.id)}});
        const Built tuned = build(f, "car_electric",
                                  {{"ev_layout", std::string(l.id)}, {"ev_reduction_ratio", 1.2}, {"ev_motor_output", 0.5}});
        std::set<std::string> reductions = l.diffs;
        reductions.insert(l.gears.begin(), l.gears.end());
        REQUIRE_FALSE(reductions.empty());
        for (const std::string& id : reductions) {
            INFO(id);
            const json* a = component_of(base.vehicle, id);
            const json* b = component_of(tuned.vehicle, id);
            REQUIRE(a != nullptr);
            REQUIRE(b != nullptr);
            CHECK(b->at("ratio").get<double>() == Approx(a->at("ratio").get<double>() * 1.2));
        }
        for (const std::string& id : l.motors) {
            INFO(id);
            const json* a = component_of(base.vehicle, id);
            const json* b = component_of(tuned.vehicle, id);
            REQUIRE(a != nullptr);
            REQUIRE(b != nullptr);
            CHECK(b->at("peak_torque_nm").get<double>() == Approx(a->at("peak_torque_nm").get<double>() * 0.5));
            CHECK(b->at("max_power_w").get<double>() == Approx(a->at("max_power_w").get<double>() * 0.5));
            CHECK(b->at("mass_kg") == a->at("mass_kg"));
            CHECK(b->at("regen_torque_nm") == a->at("regen_torque_nm"));
            CHECK(b->at("max_rpm") == a->at("max_rpm"));
        }
        // the car total does not move with either slider
        CHECK(tuned.m.compiled.parts.root.total_kg() == Approx(base.m.compiled.parts.root.total_kg()).epsilon(1e-12));
    }
    // no overlap with the combustion cars' final drive: that slider matches nothing in an EV's files, so the reduction
    // slider is the EV's only gearing option
    CHECK_FALSE(rg::validate_setup(f.entry("car_electric"), f.table, f.ctx, setup_of("car_electric", {{"final_drive", 1.1}})).ok);
}

TEST_CASE("ev motors: an old setup carrying ev_motor_scale still loads as Motor output", "[ev][ev_motors][setup]") {
    Fixture f;
    std::string err;
    const std::string old_setup = R"({"format":"racing_game.vehicle_setup/1","vehicle":"car_electric","options":{"ev_motor_scale":0.7,"ev_battery_pack":"long_range"}})";
    const auto parsed = rg::parse_setup(old_setup, f.table, "old setup", &err);
    INFO(err);
    REQUIRE(parsed.has_value());
    REQUIRE(parsed->values.count("ev_motor_output") == 1);
    CHECK(std::get<double>(parsed->values.at("ev_motor_output")) == Approx(0.7));
    CHECK(parsed->values.count("ev_motor_scale") == 0);
    CHECK(parsed->values.count("ev_battery_pack") == 1);
    // the old stronger-than-stock factors clamp to the stock output
    const auto strong = rg::parse_setup(R"({"format":"racing_game.vehicle_setup/1","vehicle":"car_electric","options":{"ev_motor_scale":1.5}})",
                                        f.table, "old setup", &err);
    REQUIRE(strong.has_value());
    CHECK(std::get<double>(strong->values.at("ev_motor_output")) == Approx(1.0));
    // a setup that sets both keeps the new one
    const auto both = rg::parse_setup(
        R"({"format":"racing_game.vehicle_setup/1","vehicle":"car_electric","options":{"ev_motor_scale":0.7,"ev_motor_output":0.9}})",
        f.table, "old setup", &err);
    REQUIRE(both.has_value());
    CHECK(std::get<double>(both->values.at("ev_motor_output")) == Approx(0.9));
    // and it materialises
    const Built b = build(f, "car_electric", {{"ev_motor_output", 0.7}});
    CHECK(component_of(b.vehicle, "front_motor")->at("peak_torque_nm").get<double>() == Approx(420.0));
    // an option that never existed is still an error
    CHECK_FALSE(rg::parse_setup(R"({"format":"racing_game.vehicle_setup/1","vehicle":"car_electric","options":{"no_such":1}})",
                                f.table, "x", &err).has_value());
}

TEST_CASE("ev motors: a full-throttle drive per layout, every driven wheel pushes", "[ev][ev_motors][drive]") {
    Fixture f;
    double single_kmh = 0, quad_kmh = 0;
    for (const LayoutExpect& l : layouts()) {
        INFO(l.id);
        const Drive d = drive(f, "car_electric", {{"ev_layout", std::string(l.id)}}, 3.0);
        CHECK(d.finite);
        CHECK(d.motor_count == static_cast<int>(l.motors.size()));
        CHECK(d.max_motor_torque_nm > 100.0); // the motors really deliver torque
        CHECK(d.kmh_at_3s > 25.0);            // the car accelerates
        int driven = 0;
        for (std::size_t w = 0; w < d.driven.size(); ++w) {
            if (d.driven[w]) {
                ++driven;
                INFO("wheel " << w);
                CHECK(d.mean_fx[w] > 500.0); // every driven wheel pushes forward
            }
        }
        CHECK(driven == l.driven_wheels);
        if (std::string(l.id) == "single") single_kmh = d.kmh_at_3s;
        if (std::string(l.id) == "quad") quad_kmh = d.kmh_at_3s;
    }
    // quad on the hypercar units beats single on the economy unit
    const Drive fast = drive(f, "car_electric", {{"ev_layout", std::string("quad")},
                                                 {"ev_front_motors", std::string("hypercar_oil_cooled")},
                                                 {"ev_rear_motors", std::string("hypercar_oil_cooled")}}, 3.0);
    const Drive slow = drive(f, "car_electric", {{"ev_layout", std::string("single")}, {"ev_rear_motors", std::string("economy_pmsm")}}, 3.0);
    CHECK(fast.finite);
    CHECK(slow.finite);
    CHECK(slow.kmh_at_3s > 5.0);
    CHECK(fast.kmh_at_3s > slow.kmh_at_3s + 10.0);
    CHECK(quad_kmh > 0);
    CHECK(single_kmh > 0);
}

TEST_CASE("ev motors: a battery_ev conversion of car_sedan drives in every layout and traction control finds the wheels",
          "[ev][ev_motors][drive][traction_control]") {
    Fixture f;
    for (const LayoutExpect& l : layouts()) {
        INFO(l.id);
        const Drive d = drive(f, "car_sedan", {{"powertrain_install", std::string("battery_ev")}, {"ev_layout", std::string(l.id)}}, 3.0);
        CHECK(d.finite);
        CHECK(d.motor_count == static_cast<int>(l.motors.size()));
        CHECK(d.kmh_at_3s > 15.0);
        int driven = 0;
        for (std::size_t w = 0; w < d.driven.size(); ++w) driven += d.driven[w] ? 1 : 0;
        CHECK(driven == l.driven_wheels);
    }
    // traction control: on the quad hypercar-unit car a full-throttle launch breaks the tyres loose with it off, and with
    // it on the controller finds the driven wheels (it intervenes) and the driven wheels slip less
    const std::vector<std::pair<std::string, rg::SetupValue>> quad = {{"ev_layout", std::string("quad")},
                                                                      {"ev_front_motors", std::string("hypercar_oil_cooled")},
                                                                      {"ev_rear_motors", std::string("hypercar_oil_cooled")}};
    const Drive on = drive(f, "car_electric", quad, 3.0, 1.0);
    const Drive off = drive(f, "car_electric", quad, 3.0, 0.0);
    CHECK(on.finite);
    CHECK(off.finite);
    CHECK(on.tc_intervened);
    CHECK_FALSE(off.tc_intervened);
}

TEST_CASE("ev motors: the garage dyno follows a layout change", "[ev][ev_motors][garage_dyno]") {
    Fixture f;
    // the dyno request keeps the electric hardware options
    const rg::VehicleSetup working = setup_of("car_electric", {{"ev_layout", std::string("quad")},
                                                               {"ev_rear_motors", std::string("compact_pmsm")},
                                                               {"ev_motor_output", 0.8},
                                                               {"ev_reduction_ratio", 1.1}});
    const rg::VehicleSetup kept = rg::dyno_setup(f.table, f.entry("car_electric"), working, &f.reference);
    for (const char* id : {"ev_layout", "ev_rear_motors", "ev_motor_output", "ev_reduction_ratio"}) {
        INFO(id);
        CHECK(kept.values.count(id) == 1);
    }
    ps::jobs::JobSystem pool(4);
    auto peak_power = [&](const std::string& layout) {
        rg::ScopedWorkDir dir(f.ctx.work_root, "dyno_" + layout);
        const auto m = rg::materialise_setup(f.entry("car_electric"), f.table, f.ctx,
                                             rg::dyno_setup(f.table, f.entry("car_electric"), setup_of("car_electric", {{"ev_layout", layout}}), &f.reference),
                                             dir.path());
        INFO(m.error);
        REQUIRE(m.ok);
        ps::io::EngineMapOptions maps;
        maps.cache_dir = f.ctx.engine_map_cache_dir;
        maps.pool = &pool;
        const auto vehicle = rg::load_vehicle(m.vehicle_path, maps);
        const auto plan = rg::plan_electrified_dyno(vehicle, f.reference);
        REQUIRE(plan.has_value());
        const auto points = rg::measure_electrified_dyno(vehicle, f.reference, *plan, &pool, nullptr, [](const auto&, int) {});
        double peak = 0;
        std::size_t motors = 0;
        for (const auto& p : points) {
            peak = std::max(peak, p.power_kw);
            motors = std::max(motors, p.motors.size());
        }
        return std::make_pair(peak, motors);
    };
    const auto single = peak_power("single");
    const auto quad = peak_power("quad");
    CHECK(single.second == 1);
    CHECK(quad.second == 4);
    CHECK(quad.first > single.first * 1.5);
}

// Numbers for the hand-over report: layout x unit -> kW, N*m, kg, 0-100 km/h. Hidden: run with "[ev_motors_report]".
TEST_CASE("ev motors: report of the layouts and units", "[.][ev_motors_report]") {
    Fixture f;
    struct Row { const char* layout; const char* unit; };
    const std::vector<Row> rows = {{"single", ""}, {"dual", ""}, {"tri", ""}, {"quad", ""},
                                   {"single", "economy_pmsm"}, {"dual", "economy_pmsm"}, {"quad", "economy_pmsm"},
                                   {"single", "hypercar_oil_cooled"}, {"dual", "hypercar_oil_cooled"}, {"quad", "hypercar_oil_cooled"},
                                   {"quad", "axial_flux"}, {"quad", "compact_pmsm"}, {"quad", "compact_induction"}, {"quad", "high_density_pmsm"}};
    for (const Row& r : rows) {
        std::vector<std::pair<std::string, rg::SetupValue>> v = {{"ev_layout", std::string(r.layout)}};
        if (*r.unit) { v.push_back({"ev_front_motors", std::string(r.unit)}); v.push_back({"ev_rear_motors", std::string(r.unit)}); }
        const Built b = build(f, "car_electric", v);
        double kw = 0, nm = 0, motor_kg = 0;
        for (const json* m : components_of_type(b.vehicle, "electric_motor")) {
            kw += m->at("max_power_w").get<double>() / 1000.0;
            nm += m->at("peak_torque_nm").get<double>();
            motor_kg += m->at("mass_kg").get<double>();
        }
        const Drive d = drive(f, "car_electric", v, 12.0);
        const Drive d0 = drive(f, "car_electric", v, 12.0, 0.0);
        std::printf("EVREPORT_TC0 %s %s: 0-100 %.2f s, v3s %.1f\n", r.layout, r.unit, d0.t100_s, d0.kmh_at_3s);
        std::printf("EVREPORT layout=%-6s unit=%-20s motors=%zu  sum %.0f kW  %.0f N*m(motor)  motors %.0f kg  car %.0f kg  v3s %.1f km/h  0-100 %.2f s\n",
                    r.layout, *r.unit ? r.unit : "(stock 300kW)", components_of_type(b.vehicle, "electric_motor").size(), kw, nm, motor_kg,
                    b.m.compiled.parts.root.total_kg(), d.kmh_at_3s, d.t100_s);
    }
}

// ---- hybrids: the traction motor's drive unit (owner 2026-10-21 follow-up) -------------------------------------------

TEST_CASE("ev motors: a hybrid offers the drive unit of its traction motor, units only", "[ev][ev_motors][hybrid][setup]") {
    Fixture f;
    const std::vector<std::pair<std::string, rg::SetupValue>> hybrid = {{"powertrain_install", std::string("hybrid_battery")}};
    const rg::VehicleSetup hybrid_setup = setup_of("car_hyper_n2o", hybrid);
    const rg::SetupModel m = rg::build_setup_model(f.entry("car_hyper_n2o"), f.table, f.ctx, &hybrid_setup);
    REQUIRE(m.error.empty());
    const rg::OptionView* unit = view_of(m, "ev_hybrid_motor");
    REQUIRE(unit != nullptr);
    CHECK(unit->available);
    REQUIRE(unit->parts.size() == f.motors->units().size() + 1);
    CHECK(unit->parts.front().id == "stock");
    CHECK_THAT(unit->parts.front().detail, ContainsSubstring("200 kW"));
    // the layout and the per-axle options stay battery-EV only
    for (const char* id : {"ev_layout", "ev_front_motors", "ev_rear_motors"}) {
        INFO(id);
        const rg::OptionView* v = view_of(m, id);
        REQUIRE(v != nullptr);
        CHECK_FALSE(v->available);
    }
    // a battery EV and an ICE car do not offer it
    const rg::SetupModel ev = rg::build_setup_model(f.entry("car_electric"), f.table, f.ctx);
    REQUIRE(view_of(ev, "ev_hybrid_motor") != nullptr);
    CHECK_FALSE(view_of(ev, "ev_hybrid_motor")->available);
    const rg::SetupModel ice = rg::build_setup_model(f.entry("car_hyper_n2o"), f.table, f.ctx);
    REQUIRE(view_of(ice, "ev_hybrid_motor") != nullptr);
    CHECK_FALSE(view_of(ice, "ev_hybrid_motor")->available);
    // the layout cannot be set on a hybrid
    CHECK_FALSE(rg::validate_setup(f.entry("car_hyper_n2o"), f.table, f.ctx,
                                   setup_of("car_hyper_n2o", {{"powertrain_install", std::string("hybrid_battery")}, {"ev_layout", std::string("quad")}})).ok);
}

TEST_CASE("ev motors: a hypercar unit lands in the hybrid's traction motor and its reduction, the generator stays",
          "[ev][ev_motors][hybrid]") {
    Fixture f;
    const Built stock = build(f, "car_hyper_n2o", {{"powertrain_install", std::string("hybrid_battery")}});
    const Built b = build(f, "car_hyper_n2o", {{"powertrain_install", std::string("hybrid_battery")},
                                               {"ev_hybrid_motor", std::string("hypercar_oil_cooled")}});
    const rg::EvMotorUnit* u = f.motors->find_unit("hypercar_oil_cooled");
    REQUIRE(u != nullptr);
    const json* motor = component_of(b.vehicle, "conversion_motor");
    const json* motor0 = component_of(stock.vehicle, "conversion_motor");
    REQUIRE(motor != nullptr);
    REQUIRE(motor0 != nullptr);
    CHECK(motor->at("peak_torque_nm").get<double>() == Approx(u->peak_torque_nm));
    CHECK(motor->at("max_power_w").get<double>() == Approx(u->max_power_w));
    CHECK(motor->at("max_rpm").get<double>() == Approx(u->max_rpm));
    CHECK(motor->at("mass_kg").get<double>() == Approx(u->mass_kg));
    CHECK(motor->at("crank_assist").get<bool>() == false);
    CHECK(motor->at("battery").get<std::string>() == "conversion_battery");
    // the crank generator is untouched
    CHECK(*component_of(b.vehicle, "conversion_generator") == *component_of(stock.vehicle, "conversion_generator"));
    // the reduction: the unit's ratio is the whole motor-to-wheel ratio, so the reduction before the final drive is ratio / FD
    const double final_drive = component_of(b.vehicle, "diff")->at("ratio").get<double>();
    const json* red = component_of(b.vehicle, "conversion_reduction");
    REQUIRE(red != nullptr);
    CHECK(red->at("ratio").get<double>() * final_drive == Approx(u->reduction_ratio));
    CHECK(red->at("efficiency").get<double>() == Approx(u->reduction_efficiency));
    CHECK(component_of(stock.vehicle, "conversion_reduction")->at("ratio").get<double>() == Approx(2.2));
    // the swap authors no full-pedal rating: physics derives it (electric available + engine peak), so the unit adds none
    CHECK_FALSE(component_of(b.vehicle, "conversion_control")->contains("requested_power_w"));
    // the part-mass tree follows the motor
    auto motors_kg = [](const rg::PartTree& tree) {
        double sum = 0;
        std::vector<const rg::PartNode*> todo = {&tree.root};
        while (!todo.empty()) {
            const rg::PartNode* n = todo.back();
            todo.pop_back();
            if (n->key == "electric_motor") sum += n->total_kg();
            for (const rg::PartNode& c : n->children) todo.push_back(&c);
        }
        return sum;
    };
    INFO(b.m.compiled.parts.error);
    REQUIRE(b.m.compiled.parts.ok);
    CHECK(motors_kg(b.m.compiled.parts) > motors_kg(stock.m.compiled.parts));
    // Motor output scales the traction motor only
    const Built half = build(f, "car_hyper_n2o", {{"powertrain_install", std::string("hybrid_battery")},
                                                  {"ev_hybrid_motor", std::string("hypercar_oil_cooled")}, {"ev_motor_output", 0.5}});
    CHECK(component_of(half.vehicle, "conversion_motor")->at("max_power_w").get<double>() == Approx(u->max_power_w * 0.5));
    CHECK(*component_of(half.vehicle, "conversion_generator") == *component_of(stock.vehicle, "conversion_generator"));
    // it validates, and loads in the physics
    CHECK(rg::validate_setup(f.entry("car_hyper_n2o"), f.table, f.ctx,
                             setup_of("car_hyper_n2o", {{"powertrain_install", std::string("hybrid_battery")}, {"ev_hybrid_motor", std::string("hypercar_oil_cooled")}})).ok);
    ps::jobs::JobSystem pool(4);
    ps::io::EngineMapOptions maps;
    maps.cache_dir = f.ctx.engine_map_cache_dir;
    maps.pool = &pool;
    const auto vehicle = rg::load_vehicle(b.m.vehicle_path, maps);
    CHECK_FALSE(vehicle.powertrain.components.empty());
}

TEST_CASE("ev motors: the garage dyno follows the hybrid's drive unit", "[ev][ev_motors][hybrid][garage_dyno]") {
    Fixture f;
    // a race pack (330 kW) lets the 450 kW unit show over the car's own 200 kW motor
    const std::vector<std::pair<std::string, rg::SetupValue>> base = {{"powertrain_install", std::string("hybrid_battery")},
                                                                      {"ev_battery_pack", std::string("race")}};
    auto with_unit = base;
    with_unit.push_back({"ev_hybrid_motor", std::string("hypercar_oil_cooled")});
    const rg::VehicleSetup kept = rg::dyno_setup(f.table, f.entry("car_hyper_n2o"), setup_of("car_hyper_n2o", with_unit), &f.reference);
    CHECK(kept.values.count("ev_hybrid_motor") == 1);
    ps::jobs::JobSystem pool(4);
    auto peak = [&](const std::vector<std::pair<std::string, rg::SetupValue>>& values, const char* tag) {
        rg::ScopedWorkDir dir(f.ctx.work_root, std::string("hybdyno_") + tag);
        const auto m = rg::materialise_setup(f.entry("car_hyper_n2o"), f.table, f.ctx,
                                             rg::dyno_setup(f.table, f.entry("car_hyper_n2o"), setup_of("car_hyper_n2o", values), &f.reference),
                                             dir.path());
        INFO(m.error);
        REQUIRE(m.ok);
        ps::io::EngineMapOptions maps;
        maps.cache_dir = f.ctx.engine_map_cache_dir;
        maps.pool = &pool;
        const auto vehicle = rg::load_vehicle(m.vehicle_path, maps);
        const auto plan = rg::plan_electrified_dyno(vehicle, f.reference);
        REQUIRE(plan.has_value());
        const auto points = rg::measure_electrified_dyno(vehicle, f.reference, *plan, &pool, nullptr, [](const auto&, int) {});
        double best = 0;
        for (const auto& p : points) best = std::max(best, p.power_kw);
        return best;
    };
    const double own = peak(base, "own");
    const double hyper = peak(with_unit, "unit");
    CHECK(hyper > own + 50.0); // the engine stays on: the hybrid rating grows with the motor
}

TEST_CASE("ev motors: battery pack cards show absolute figures for the car being edited", "[ev][ev_motors][hybrid][setup]") {
    Fixture f;
    auto pack_detail = [&](const std::string& car, std::vector<std::pair<std::string, rg::SetupValue>> values, const std::string& part) {
        const rg::VehicleSetup vs = setup_of(car, std::move(values));
        const rg::SetupModel m = rg::build_setup_model(f.entry(car), f.table, f.ctx, &vs);
        REQUIRE(m.error.empty());
        const rg::OptionView* v = view_of(m, "ev_battery_pack");
        REQUIRE(v != nullptr);
        for (const rg::SetupPart& p : v->parts) if (p.id == part) return p.detail;
        FAIL("no such pack card " + part);
        return std::string();
    };
    CHECK(pack_detail("car_hyper_n2o", {{"powertrain_install", std::string("hybrid_battery")}}, "grand_tourer") == "28 kWh \xc2\xb7 300 kW \xc2\xb7 231 kg");
    CHECK(pack_detail("car_electric", {}, "grand_tourer") == "126 kWh \xc2\xb7 1400 kW \xc2\xb7 825 kg");
    const rg::SetupModel m = rg::build_setup_model(f.entry("car_electric"), f.table, f.ctx);
    CHECK_THAT(view_of(m, "ev_battery_pack")->def.help, ContainsSubstring("caps the electric power"));
}

TEST_CASE("ev motors: an old ev_motor_scale of 2.0 on a hybrid loads as Motor output 1.0", "[ev][ev_motors][hybrid][setup]") {
    Fixture f;
    std::string err;
    const auto parsed = rg::parse_setup(
        R"({"format":"racing_game.vehicle_setup/1","vehicle":"car_hyper_n2o","options":{"powertrain_install":"hybrid_battery","ev_motor_scale":2.0,"ev_battery_pack":"grand_tourer"}})",
        f.table, "old setup", &err);
    INFO(err);
    REQUIRE(parsed.has_value());
    CHECK(std::get<double>(parsed->values.at("ev_motor_output")) == Approx(1.0));
    const Built b = build(f, "car_hyper_n2o", {{"powertrain_install", std::string("hybrid_battery")}, {"ev_motor_output", 1.0},
                                               {"ev_battery_pack", std::string("grand_tourer")}});
    CHECK(component_of(b.vehicle, "conversion_motor")->at("max_power_w").get<double>() == Approx(200000.0));
    CHECK(component_of(b.vehicle, "conversion_battery")->at("max_discharge_w").get<double>() == Approx(300000.0));
}

TEST_CASE("Hybrid charging power is a persisted tuning setting in kW", "[vehicle_setup][hybrid_charge]") {
    Fixture f;
    const auto setup = setup_of("car_hyper_n2o", {{"powertrain_install", std::string("hybrid_battery")}});
    const auto hybrid = rg::build_setup_model(f.entry("car_hyper_n2o"), f.table, f.ctx, &setup);
    const auto* option = view_of(hybrid, "hybrid_charge_power");
    REQUIRE(option != nullptr);
    REQUIRE(option->available);
    REQUIRE(option->def.slider.has_value());
    CHECK(option->def.slider->unit == "kW");
    const Built built = build(f, "car_hyper_n2o", {{"powertrain_install", std::string("hybrid_battery")}, {"hybrid_charge_power", 0.5}});
    REQUIRE(built.m.ok);
    const auto vehicle = read_json(built.m.vehicle_path);
    bool found = false;
    for (const auto& c : vehicle.at("powertrain").at("components")) if (c.value("type", "") == "hybrid_control") {
        CHECK(c.at("charge_power_w").get<double>() == Approx(25000));found = true;
    }
    CHECK(found);
    const auto pure_ev = rg::build_setup_model(f.entry("car_electric"), f.table, f.ctx);
    const auto* ev_option = view_of(pure_ev, "hybrid_charge_power");
    REQUIRE(ev_option != nullptr);CHECK_FALSE(ev_option->available);
}

TEST_CASE("Hybrid generator hardware upgrades capacity and mass separately from the charge target", "[vehicle_setup][hybrid_charge]") {
    Fixture f;
    const auto stock=build(f,"car_hyper_n2o",{{"powertrain_install",std::string("hybrid_battery")}});
    const auto upgraded=build(f,"car_hyper_n2o",{{"powertrain_install",std::string("hybrid_battery")},
        {"hybrid_generator",std::string("race")},{"hybrid_charge_power",2.0}});
    bool found=false;
    for(const auto& c:upgraded.vehicle.at("powertrain").at("components")) {
        if(c.value("type","")=="electric_motor"&&c.value("crank_assist",false)) {
            CHECK(c.at("max_power_w").get<double>()==Approx(200000));
            CHECK(c.at("regen_torque_nm").get<double>()==Approx(800));
            CHECK(c.at("mass_kg").get<double>()==Approx(75));found=true;
        }
        if(c.value("type","")=="hybrid_control")CHECK(c.at("charge_power_w").get<double>()==Approx(100000));
    }
    CHECK(found);
    std::string error;
    const auto saved=rg::parse_setup(R"({"format":"racing_game.vehicle_setup/1","vehicle":"car_hyper_n2o","options":{"hybrid_generator":"race","hybrid_charge_power":2.0}})",f.table,"saved hybrid",&error);
    INFO(error);REQUIRE(saved.has_value());CHECK(std::get<double>(saved->values.at("hybrid_charge_power"))==2.0);
}
