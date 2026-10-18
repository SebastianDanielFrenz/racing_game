// test_abs_fitment.cpp - ABS and ESC are fitted by model year (owner decision 2026-10-18: "depends on the car, not on
// very old ones"). Every catalogue car is loaded through the garage's stock path and compared with the rule:
//   none:       car_runabout (1910s), car_gt70 (1970s), car_phoenix_455 ('71), car_fairheaven_lt80 ('80), car_kiri_86 ('86)
//   ABS only:   car_shvan_92, car_lct_3000 ('95)
//   ABS + ESC:  everything newer (both on by default)
// Traction control must not move: the fitted modes, the channel default and the unit's TC block equal the car as it was
// before the change (the same car loaded with its former unit, or with no unit).
#include "rg/vehicle_catalog.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/io/vehicle_io.h"
#include "ps/vehicle/assist_unit_desc.h"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <map>
#include <memory>
#include <string>

namespace {
namespace fs = std::filesystem;
const std::string kRoot = RG_SOURCE_DIR;

enum class Fit { None, AbsOnly, AbsEsc };

struct Rule {
    Fit fit;
    const char* former_unit; // what the car carried before ("none" = no unit)
    const char* stock_label; // the garage's label of the stock "Driver assists" choice
};

const std::map<std::string, Rule>& rules() {
    static const std::string phys = "../../external/physics_sim/data/electronics/";
    static const std::map<std::string, Rule> r = {
        {"car_runabout", {Fit::None, "none", "No assist unit"}},
        {"car_gt70", {Fit::None, "none", "No assist unit"}},
        {"car_phoenix_455", {Fit::None, "none", "No assist unit"}},
        {"car_fairheaven_lt80", {Fit::None, "none", "No assist unit"}},
        {"car_kiri_86", {Fit::None, "none", "No assist unit"}},
        {"car_shvan_92", {Fit::AbsOnly, "none", "ABS only"}},
        {"car_lct_3000", {Fit::AbsOnly, "none", "ABS only"}},
        {"car_ace_11", {Fit::AbsEsc, "none", "ABS + ESC"}},
        {"car_city", {Fit::AbsEsc, "none", "ABS + ESC"}},
        {"car_sedan", {Fit::AbsEsc, "none", "ABS + ESC"}},
        {"car_sedan_rwd", {Fit::AbsEsc, "none", "ABS + ESC"}},
        {"car_sedan_awd", {Fit::AbsEsc, "none", "ABS + ESC"}},
        {"car_sedan_gen_n2o", {Fit::AbsEsc, "none", "ABS + ESC"}},
        {"car_toyama_xr", {Fit::AbsEsc, "none", "ABS + ESC"}},
        {"car_generic_suv", {Fit::AbsEsc, "none", "ABS + ESC"}},
        {"car_electric", {Fit::AbsEsc, "../electronics/assist_unit_electric.json", "Electric Assists + ABS/ESC"}},
        {"car_hyper", {Fit::AbsEsc, "../../external/physics_sim/data/electronics/assist_unit_racing.json", "Racing Assists + ABS/ESC"}},
        {"car_hyper_n2o", {Fit::AbsEsc, "../../external/physics_sim/data/electronics/assist_unit_racing.json", "Racing Assists + ABS/ESC"}},
    };
    return r;
}

struct Fixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    rg::ScopedWorkDir root{fs::temp_directory_path().generic_string(), "rg_abs_fitment"};
    Fixture() {
        std::string err;
        auto c = rg::load_vehicle_catalog(kRoot + "/data/vehicles/catalog.json", kRoot, &err);
        REQUIRE(c.has_value());
        catalog = *c;
        auto t = rg::load_setup_options(kRoot + "/data/vehicles/setup_options.json", &err);
        REQUIRE(t.has_value());
        table = *t;
        ctx.tyre_families_path = kRoot + "/data/tyres/families.json";
        ctx.work_root = root.path();
        ctx.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
        auto store = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, "", &err);
        REQUIRE(store != nullptr);
        ctx.engines = std::shared_ptr<const rg::EngineStore>(std::move(store));
    }
};

ps::vehicle::VehicleDesc load(const std::string& path, const std::map<std::string, std::string>& electronics = {}) {
    ps::io::VehicleLoadOptions options;
    options.maps.generate = false;
    options.electronics = electronics;
    return rg::load_vehicle(path, options);
}

bool same_traction(const ps::vehicle::VehicleDesc& a, const ps::vehicle::VehicleDesc& b) {
    namespace pv = ps::vehicle;
    if (pv::traction_modes_available_bits(a.assist_unit, a.traction_control.fitted) != pv::traction_modes_available_bits(b.assist_unit, b.traction_control.fitted)) return false;
    if (pv::traction_default_channel_value(a.traction_control, a.assist_unit) != pv::traction_default_channel_value(b.traction_control, b.assist_unit)) return false;
    if (a.traction_control.enabled != b.traction_control.enabled || a.traction_control.fitted != b.traction_control.fitted) return false;
    const bool ua = a.assist_unit.has_value(), ub = b.assist_unit.has_value();
    if (ua != ub) return ua ? !a.assist_unit->has_traction_control : !b.assist_unit->has_traction_control; // an ABS-only unit adds no TC block
    if (!ua) return true;
    return a.assist_unit->grade == b.assist_unit->grade && a.assist_unit->has_traction_control == b.assist_unit->has_traction_control &&
           a.assist_unit->launch_control.has_value() == b.assist_unit->launch_control.has_value() &&
           a.assist_unit->aero_control.has_value() == b.assist_unit->aero_control.has_value();
}
} // namespace

TEST_CASE("ABS fitment: every catalogue car matches the model-year rule and keeps its traction control", "[abs_fitment][assist][garage]") {
    Fixture f;
    REQUIRE(!f.catalog.entries.empty());
    for (const rg::CatalogEntry& e : f.catalog.entries) {
        const std::string stem = fs::path(e.vehicle_path).stem().string();
        INFO(e.id << " (" << stem << ")");
        const auto it = rules().find(stem);
        REQUIRE(it != rules().end()); // a new car must be classified here
        const Rule& rule = it->second;

        const ps::vehicle::VehicleDesc desc = load(e.vehicle_path);
        const bool abs = desc.assist_unit.has_value() && desc.assist_unit->abs.has_value();
        const bool esc = desc.assist_unit.has_value() && desc.assist_unit->esc.has_value();
        CHECK(abs == (rule.fit != Fit::None));       // assist_fitted_bits bit 8
        CHECK(esc == (rule.fit == Fit::AbsEsc));     // bit 9
        if (abs) CHECK(desc.assist_unit->abs->default_enabled);
        if (esc) CHECK(desc.assist_unit->esc->default_mode == ps::vehicle::EscMode::On);

        // Traction control as before: the same car with its former unit (or none).
        const std::string former = rule.former_unit;
        const ps::vehicle::VehicleDesc before = (former == "none" && rule.fit == Fit::None) ? desc : load(e.vehicle_path, {{"assists", former}});
        CHECK(same_traction(desc, before));
        if (former == "none") CHECK_FALSE(before.assist_unit.has_value());

        // The garage shows the stock choice as what the car carries.
        const rg::SetupModel model = rg::build_setup_model(e, f.table, f.ctx);
        REQUIRE(model.error.empty());
        const rg::OptionView* view = nullptr;
        for (const auto& o : model.options) if (o.def.id == "assist_unit") view = &o;
        REQUIRE(view != nullptr);
        REQUIRE(!view->parts.empty());
        CHECK(view->choices.front() == "stock");
        CHECK(view->parts.front().label == rule.stock_label);
    }
}

TEST_CASE("ABS fitment: a garage unit without ABS removes it, the existing units keep working", "[abs_fitment][assist][garage]") {
    Fixture f;
    const rg::CatalogEntry* e = f.catalog.find("car_sedan");
    REQUIRE(e != nullptr);
    struct Case { const char* part; bool abs, esc; std::uint32_t tc_bits; };
    const Case cases[] = {{"stock", true, true, 0}, {"assists_street", false, false, 0b00011}, {"assists_racing", false, false, 0b11111},
                          {"assists_street_abs_esc", true, true, 0b00011}, {"assists_abs", true, false, 0}, {"assists_abs_esc", true, true, 0}};
    for (const Case& c : cases) {
        INFO(c.part);
        rg::VehicleSetup setup;
        setup.vehicle_id = e->id;
        setup.values["assist_unit"] = std::string(c.part);
        const auto v = rg::validate_setup(*e, f.table, f.ctx, setup, false);
        INFO(v.message);
        REQUIRE(v.ok);
        rg::ScopedWorkDir dir(f.ctx.work_root, "t_abs_fit");
        const rg::MaterialisedSetup m = rg::materialise_setup(*e, f.table, f.ctx, setup, dir.path());
        INFO(m.error);
        REQUIRE(m.ok);
        const ps::vehicle::VehicleDesc desc = load(m.vehicle_path);
        REQUIRE(desc.assist_unit.has_value());
        CHECK(desc.assist_unit->abs.has_value() == c.abs);
        CHECK(desc.assist_unit->esc.has_value() == c.esc);
        if (c.tc_bits != 0) CHECK(ps::vehicle::traction_modes_available_bits(desc.assist_unit, desc.traction_control.fitted) == c.tc_bits);
    }
}

TEST_CASE("ABS fitment: the ABS/ESC blocks of every shipped unit equal the street unit's", "[abs_fitment][assist][io]") {
    namespace pv = ps::vehicle;
    const pv::VehicleDesc base = load(kRoot + "/data/vehicles/car_city.json"); // physics' assist_unit_abs_esc.json
    const pv::VehicleDesc hyper = load(kRoot + "/data/vehicles/car_hyper.json");
    const pv::VehicleDesc electric = load(kRoot + "/data/vehicles/car_electric.json");
    const pv::VehicleDesc abs_only = load(kRoot + "/data/vehicles/car_lct_3000.json");
    REQUIRE(base.assist_unit.has_value());
    REQUIRE(hyper.assist_unit.has_value());
    REQUIRE(electric.assist_unit.has_value());
    REQUIRE(abs_only.assist_unit.has_value());
    const auto& ref = *base.assist_unit;
    for (const auto* d : {&hyper, &electric, &abs_only}) {
        REQUIRE(d->assist_unit->abs.has_value());
        CHECK(d->assist_unit->abs->target_slip == ref.abs->target_slip);
        CHECK(d->assist_unit->abs->kp == ref.abs->kp);
        CHECK(d->assist_unit->abs->ki_per_s == ref.abs->ki_per_s);
        CHECK(d->assist_unit->abs->front_split_max == ref.abs->front_split_max);
    }
    for (const auto* d : {&hyper, &electric}) {
        REQUIRE(d->assist_unit->esc.has_value());
        CHECK(d->assist_unit->esc->max_lateral_accel_mps2 == ref.esc->max_lateral_accel_mps2);
        CHECK(d->assist_unit->esc->params[2].max_brake == ref.esc->params[2].max_brake);
    }
    CHECK_FALSE(abs_only.assist_unit->esc.has_value());
    // The hypers keep the racing unit's launch control and wing automation.
    CHECK(hyper.assist_unit->launch_control.has_value());
    CHECK(hyper.assist_unit->aero_control.has_value());
}

TEST_CASE("ABS fitment: every game vehicle file (also the cars outside the catalogue) matches the rule", "[abs_fitment][assist][io]") {
    int seen = 0;
    for (const auto& de : fs::directory_iterator(kRoot + "/data/vehicles")) {
        const std::string stem = de.path().stem().string();
        if (de.path().extension() != ".json" || stem.rfind("car_", 0) != 0) continue;
        INFO(stem);
        const auto it = rules().find(stem);
        REQUIRE(it != rules().end());
        const ps::vehicle::VehicleDesc desc = load(de.path().generic_string());
        const bool abs = desc.assist_unit.has_value() && desc.assist_unit->abs.has_value();
        const bool esc = desc.assist_unit.has_value() && desc.assist_unit->esc.has_value();
        CHECK(abs == (it->second.fit != Fit::None));
        CHECK(esc == (it->second.fit == Fit::AbsEsc));
        ++seen;
    }
    CHECK(seen == static_cast<int>(rules().size()));
}

// The hypers' unit "extends" the physics Racing unit and so inherits its launch block: an engine swap that leaves the 5000 rpm
// launch rpm outside the new engine's band must still move it (the patch goes into the unit's own launch_control block).
TEST_CASE("ABS fitment: an engine swap still moves the inherited launch rpm of an extending unit", "[abs_fitment][assist][garage]") {
    Fixture f;
    const rg::CatalogEntry* e = f.catalog.find("car_hyper");
    REQUIRE(e != nullptr);
    rg::VehicleSetup setup;
    setup.vehicle_id = e->id;
    setup.values["engine_install"] = std::string("runabout_i4_2l9_sv");
    rg::ScopedWorkDir dir(f.ctx.work_root, "t_abs_launch");
    const rg::MaterialisedSetup m = rg::materialise_setup(*e, f.table, f.ctx, setup, dir.path());
    INFO(m.error);
    REQUIRE(m.ok);
    bool moved = false;
    for (const std::string& l : m.compiled.swap_lines) if (l.rfind("Launch control: launch rpm moved", 0) == 0) moved = true;
    CHECK(moved);
    const ps::vehicle::VehicleDesc desc = load(m.vehicle_path);
    REQUIRE(desc.assist_unit.has_value());
    REQUIRE(desc.assist_unit->launch_control.has_value());
    CHECK(desc.assist_unit->launch_control->launch_rpm < 5000.0);
    CHECK(desc.assist_unit->abs.has_value());
}
