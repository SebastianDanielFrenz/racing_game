// test_tyre_spec.cpp - universal tyres: a compound plus a size, built on the fly (rg/tyre_spec.h, rg/vehicle_load.h).
//
// The tyre rule (data/tyres/families.json), the id format, per-axle fitment and its error messages, migration of
// the pre-universal tyre names, and that every racing_game vehicle file loads with spec-built tyres.
#include "rg/garage.h"
#include "rg/tyre_spec.h"
#include "rg/vehicle_catalog.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/io/vehicle_io.h"
#include "ps/vehicle/tyre_model.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
const std::string kRoot = RG_SOURCE_DIR;

std::shared_ptr<const rg::TyreFamilies> families() {
    std::string err;
    auto f = rg::TyreFamilies::load(kRoot + "/data/tyres/families.json", &err);
    INFO(err);
    REQUIRE(f != nullptr);
    return f;
}

// Every numeric member of a TyreDesc as raw bits, so "bit-identical" is literal.
std::vector<double> numbers_of(const ps::vehicle::TyreDesc& d) {
    return {d.unloaded_radius, d.width, d.rim_radius, d.fz0, d.vertical_stiffness, d.v0, d.qsy1, d.qsy2, d.qsy3, d.qsy4,
            d.rolling_resistance_v_eps, d.relaxation_length_x, d.relaxation_length_y, d.lambda_mux, d.lambda_muy,
            d.low_speed_damping_ratio, d.low_speed_damping_vx,
            d.pcx1, d.pdx1, d.pdx2, d.pdx3, d.pex1, d.pex2, d.pex3, d.pex4, d.pkx1, d.pkx2, d.pkx3, d.phx1, d.phx2, d.pvx1, d.pvx2,
            d.pcy1, d.pdy1, d.pdy2, d.pdy3, d.pey1, d.pey2, d.pey3, d.pey4, d.pky1, d.pky2, d.phy1, d.phy2, d.phy3, d.pvy1, d.pvy2, d.pvy3, d.pvy4,
            d.rbx1, d.rbx2, d.rcx1, d.rhx1, d.rby1, d.rby2, d.rby3, d.rcy1, d.rhy1, d.rhy2, d.rvy1, d.rvy2, d.rvy3, d.rvy4, d.rvy5, d.rvy6,
            d.qbz1, d.qbz2, d.qbz3, d.qcz1, d.qdz1, d.qdz2, d.qez1, d.qez2, d.qhz1, d.qhz2, d.qhz3, d.qsx1, d.qsx2, d.qsx3,
            d.rolling_resistance_anchors.rated_speed_kmh, d.rolling_resistance_anchors.crr_base, d.rolling_resistance_anchors.fraction,
            d.rolling_resistance_anchors.factor, d.rolling_resistance_anchors.factor_at_rated};
}

bool bit_identical(const ps::vehicle::TyreDesc& a, const ps::vehicle::TyreDesc& b) {
    const auto x = numbers_of(a), y = numbers_of(b);
    return a.name == b.name && a.source == b.source && x.size() == y.size() && std::memcmp(x.data(), y.data(), x.size() * sizeof(double)) == 0;
}

struct TempDirs {
    std::string base;
    TempDirs() {
        base = (fs::temp_directory_path() /
                ("rg_tyre_spec_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count() % 1000000000)))
                   .generic_string();
        fs::create_directories(base + "/user");
        fs::create_directories(base + "/work");
    }
    ~TempDirs() {
        std::error_code ec;
        fs::remove_all(fs::path(base), ec);
    }
};

rg::GarageConfig config_for(const TempDirs& t) {
    rg::GarageConfig c;
    c.repo_root = kRoot;
    c.catalog_path = kRoot + "/data/vehicles/catalog.json";
    c.options_path = kRoot + "/data/vehicles/setup_options.json";
    c.set_path = kRoot + "/data/garage/garage_set.json";
    c.tyre_families_path = kRoot + "/data/tyres/families.json";
    c.user_dir = t.base + "/user";
    c.work_root = t.base + "/work";
    c.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
    return c;
}

struct SetupFixture {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    TempDirs dirs;
    SetupFixture() {
        std::string err;
        auto c = rg::load_vehicle_catalog(kRoot + "/data/vehicles/catalog.json", kRoot, &err);
        REQUIRE(c.has_value());
        catalog = *c;
        auto t = rg::load_setup_options(kRoot + "/data/vehicles/setup_options.json", &err);
        REQUIRE(t.has_value());
        table = *t;
        ctx.tyre_families_path = kRoot + "/data/tyres/families.json";
        ctx.work_root = dirs.base + "/work";
        ctx.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
    }
    rg::VehicleSetup setup(const char* vehicle, const char* option, const std::string& value) const {
        rg::VehicleSetup s;
        s.vehicle_id = vehicle;
        s.values[option] = value;
        return s;
    }
    std::string compile_error(const char* vehicle, const char* option, const std::string& value) const {
        const rg::CatalogEntry* e = catalog.find(vehicle);
        REQUIRE(e != nullptr);
        const rg::CompiledSetup r = rg::compile_setup(*e, table, ctx, setup(vehicle, option, value));
        return r.ok ? std::string() : r.error;
    }
};
} // namespace

TEST_CASE("tyre spec: the same spec gives a bit-identical TyreDesc, a different spec does not", "[tyre_spec][tyre]") {
    const auto f = families();
    std::set<std::string> distinct;
    for (const auto& c : f->compounds()) {
        for (int width = 195; width <= 365; width += 10) {
            for (int rim : {17, 20, 21}) {
                const rg::TyreSpec s{c.id, width, rg::tyre_aspect_for(rim, width, rim == 17 ? 634.0 : 700.0), rim};
                const ps::vehicle::TyreDesc a = f->make(s), b = f->make(s);
                CHECK(bit_identical(a, b));
                CHECK(a.name == rg::tyre_id(s));
                distinct.insert(rg::tyre_id(s));
            }
        }
    }
    CHECK(distinct.size() == f->compounds().size() * 18 * 3);
    // A reload of the families file (a fresh process in effect) gives the same bits.
    std::string err;
    const auto again = rg::TyreFamilies::load(kRoot + "/data/tyres/families.json", &err);
    REQUIRE(again != nullptr);
    const rg::TyreSpec s{"track", 325, 30, 21};
    CHECK(bit_identical(f->make(s), again->make(s)));
    CHECK_FALSE(bit_identical(f->make(s), f->make({"track", 335, 30, 21})));
    CHECK_FALSE(bit_identical(f->make(s), f->make({"road", 325, 30, 21})));
    CHECK_THROWS_AS(f->make({"mud", 225, 45, 17}), std::invalid_argument);
    CHECK_THROWS_AS(f->make({"road", 0, 45, 17}), std::invalid_argument);
}

TEST_CASE("tyre spec: one rule derives every coefficient from compound and size", "[tyre_spec][tyre]") {
    const auto f = families();
    const auto* road = f->find("road");
    const auto* track = f->find("track");
    REQUIRE(road != nullptr);
    REQUIRE(track != nullptr);

    // Reference width: the compound's own grip, no width term.
    const ps::vehicle::TyreDesc ref = f->make({"road", 225, 45, 17});
    CHECK(ref.lambda_mux == Approx(road->longitudinal_grip));
    CHECK(ref.lambda_muy == Approx(road->lateral_grip));
    CHECK(ref.rolling_resistance_anchors.crr_base == Approx(0.01 * road->rolling_resistance));
    CHECK(ref.unloaded_radius == Approx((17 * 25.4 + 2 * 0.45 * 225) / 2000.0));
    CHECK(ref.rim_radius == Approx(17 * 25.4 / 2000.0));
    CHECK(ref.width == Approx(0.225));

    // Width response: grip ~ w^0.35, rolling resistance ~ w^0.65, same compound.
    const ps::vehicle::TyreDesc wide = f->make({"road", 315, 45, 17});
    CHECK(wide.lambda_mux / ref.lambda_mux == Approx(std::pow(315.0 / 225.0, 0.35)));
    CHECK(wide.rolling_resistance_anchors.crr_base / ref.rolling_resistance_anchors.crr_base == Approx(std::pow(315.0 / 225.0, 0.65)));
    CHECK(wide.fz0 > ref.fz0);
    CHECK(wide.vertical_stiffness > ref.vertical_stiffness);
    // fz0 and the vertical stiffness follow the same section volume.
    CHECK(wide.fz0 / ref.fz0 == Approx(wide.vertical_stiffness / ref.vertical_stiffness));

    // Compound: grip, rolling resistance and stiffness scale from the family entry; the rest is the base tyre.
    const ps::vehicle::TyreDesc t = f->make({"track", 225, 45, 17});
    CHECK(t.lambda_mux / ref.lambda_mux == Approx(track->longitudinal_grip / road->longitudinal_grip));
    CHECK(t.pkx1 / ref.pkx1 == Approx(track->stiffness_scale / road->stiffness_scale));
    CHECK(t.pky1 / ref.pky1 == Approx(track->stiffness_scale / road->stiffness_scale));
    CHECK(t.pcx1 == ref.pcx1);
    CHECK(t.pdy1 == ref.pdy1);
    CHECK(t.qdz1 == ref.qdz1);
    CHECK(t.relaxation_length_y == ref.relaxation_length_y);

    // Rolling resistance anchors derive qsy1/qsy3/qsy4 exactly as the physics library does for a tyre file.
    const auto derived = ps::vehicle::derive_rolling_resistance(t.rolling_resistance_anchors, t.v0);
    CHECK(t.qsy1 == derived.qsy1);
    CHECK(t.qsy3 == derived.qsy3);
    CHECK(t.qsy4 == derived.qsy4);

    // Rated speed: the compound's, capped at 270 km/h for rims up to 17 in.
    CHECK(f->rated_speed_kmh({"track", 225, 45, 17}) == 270.0);
    CHECK(f->rated_speed_kmh({"track", 265, 35, 20}) == track->rated_speed_kmh);
    CHECK(f->rated_speed_kmh({"cord", 225, 45, 17}) == f->find("cord")->rated_speed_kmh);
    CHECK(t.rolling_resistance_anchors.rated_speed_kmh == 270.0);

    // No anchoring: every compound x size comes from the rule, so grip is monotone in width for every compound.
    for (const auto& c : f->compounds()) {
        double previous = 0.0;
        for (int w = 175; w <= 395; w += 10) {
            const double mu = f->make({c.id, w, 40, 18}).lambda_mux;
            CHECK(mu > previous);
            previous = mu;
        }
    }
}

TEST_CASE("tyre spec: ids parse and format back, anything else is rejected", "[tyre_spec][tyre]") {
    const auto s = rg::parse_tyre_id("track_325_30r21");
    REQUIRE(s.has_value());
    CHECK(s->compound == "track");
    CHECK(s->width_mm == 325);
    CHECK(s->aspect == 30);
    CHECK(s->rim_in == 21);
    CHECK(rg::tyre_id(*s) == "track_325_30r21");
    CHECK(rg::tyre_size_text(*s) == "325/30 R21");
    CHECK(rg::tyre_diameter_mm(*s) == Approx(21 * 25.4 + 2 * 0.30 * 325));
    for (const char* bad : {"", "track", "track_325", "track_325_30", "track_325_30r", "_325_30r21", "track_x_30r21", "Track_325_30r21",
                            "track_325_30r2x", "hyper_rear_track_w255_r21", "passenger_car_225_45r17", "track_0_30r21", "track_325_0r21"}) {
        INFO(bad);
        CHECK_FALSE(rg::parse_tyre_id(bad).has_value());
    }
}

TEST_CASE("tyre spec: fitment errors name the real reason", "[tyre_spec][tyre]") {
    rg::TyreAxleFitment rear;
    rear.rims_in = {21};
    rear.min_width_mm = 275;
    rear.max_width_mm = 365;
    const double rear_d = 728.0;
    CHECK(rg::tyre_fit_error({"road", 325, 30, 21}, rear, rear_d).empty());
    CHECK(rg::tyre_fit_error({"road", 255, 36, 21}, rear, rear_d) == "255 mm is narrower than this axle's 275 mm minimum");
    CHECK(rg::tyre_fit_error({"road", 375, 25, 21}, rear, rear_d) == "375 mm is wider than this axle's 365 mm maximum");
    CHECK(rg::tyre_fit_error({"road", 325, 30, 20}, rear, rear_d) == "rim R20 does not fit this axle's R21 wheels");
    CHECK(rg::tyre_fit_error({"road", 325, 29, 21}, rear, rear_d).empty()); // inside the diameter tolerance
    CHECK(rg::tyre_fit_error({"road", 325, 40, 21}, rear, rear_d) == "overall diameter 793 mm differs from this axle's 728 mm by more than 12 mm");
    CHECK(rg::tyre_fit_error({"road", 285 + 3, 33, 21}, rear, rear_d) == "288 mm is not a standard width for this axle (steps of 10 mm from 275 mm)");
    rear.rims_in = {20, 21};
    CHECK(rg::tyre_fit_error({"road", 325, 30, 19}, rear, rear_d) == "rim R19 does not fit this axle's R20 or R21 wheels");
    // Every size the fitment lists fits; a size one step outside does not.
    rear.rims_in = {21};
    const auto sizes = rg::tyre_sizes_for(rear, rear_d);
    CHECK(sizes.size() == 10);
    for (const auto& size : sizes) CHECK(rg::tyre_fit_error(size, rear, rear_d).empty());
    CHECK(sizes.front().width_mm == 275);
    CHECK(sizes.back().width_mm == 365);
}

TEST_CASE("tyre spec: the owner's N2O case - 255 mm on the rear is rejected by width, not by a vague range", "[tyre_spec][setup][garage]") {
    const SetupFixture f;
    const rg::TyreSpec own{"road", 255, rg::tyre_aspect_for(21, 255, 728.0), 21};
    CHECK(f.compile_error("car_hyper_n2o", "tyre_rear", rg::tyre_id(own)) == "tyre_rear: 255 mm is narrower than this axle's 275 mm minimum");
    CHECK(f.compile_error("car_hyper_n2o", "tyre_front", "road_265_35r21") == "tyre_front: rim R21 does not fit this axle's R20 wheels");
    CHECK(f.compile_error("car_hyper_n2o", "tyre_front", "road_315_33r20") == "tyre_front: 315 mm is wider than this axle's 305 mm maximum");
    CHECK_THAT(f.compile_error("car_hyper_n2o", "tyre_front", "wet_265_35r20"), ContainsSubstring("unknown tyre compound \"wet\""));
    CHECK_THAT(f.compile_error("car_hyper_n2o", "tyre_front", "mystery_tyre"), ContainsSubstring("unknown tyre \"mystery_tyre\""));
    // A fitting size is accepted on either hypercar.
    CHECK(f.compile_error("car_hyper_n2o", "tyre_rear", "track_285_35r21").empty());
    CHECK(f.compile_error("car_hyper", "tyre_rear", "slick_365_28r21").empty());
}

TEST_CASE("tyre spec: pre-universal tyre names migrate to specs", "[tyre_spec][tyre][migration]") {
    const auto f = families();
    // The hand copies and the physics_sim stock files.
    CHECK(rg::migrate_tyre_id(*f, "passenger_car_225_45r17") == "road_225_45r17");
    CHECK(rg::migrate_tyre_id(*f, "passenger_sport_225_45r17") == "sport_225_45r17");
    CHECK(rg::migrate_tyre_id(*f, "passenger_comfort_225_45r17") == "economy_225_45r17");
    CHECK(rg::migrate_tyre_id(*f, "hyper_front_265_35r20") == "road_265_35r20");
    CHECK(rg::migrate_tyre_id(*f, "hyper_rear_325_30r21") == "road_325_30r21");
    CHECK(rg::migrate_tyre_id(*f, "hyper_front_track_265_35r20") == "track_265_35r20");
    CHECK(rg::migrate_tyre_id(*f, "hyper_rear_track_325_30r21") == "track_325_30r21");
    // The per-car/per-axle generated stems carry the width; the aspect follows from the rim's wheel.
    CHECK(rg::migrate_tyre_id(*f, "hyper_rear_track_w255_r21") == "track_255_38r21");
    CHECK(rg::migrate_tyre_id(*f, "hyper_front_road_w295_r20") == "road_295_32r20");
    CHECK(rg::migrate_tyre_id(*f, "passenger_sport_w195_r17") == "sport_195_52r17");
    CHECK(rg::migrate_tyre_id(*f, "passenger_road_w255_r17") == "road_255_40r17");
    // Stems and paths alike; names that are not legacy stay unmapped.
    CHECK(rg::migrate_tyre_id(*f, "../../external/physics_sim/data/tyres/hyper_front_265_35r20.json") == "road_265_35r20");
    CHECK(rg::migrate_tyre_id(*f, "road_225_45r17").empty());
    CHECK(rg::migrate_tyre_id(*f, "passenger_mud_w225_r17").empty());
    CHECK(rg::migrate_tyre_id(*f, "passenger_road_w225_r16").empty()); // no legacy rim 16
    CHECK(rg::migrate_tyre_id(*f, "nonsense").empty());
    // Every compound migrates at every legacy rim.
    for (const auto& c : f->compounds()) {
        for (const auto& legacy : {std::make_pair("passenger_", "w225_r17"), std::make_pair("hyper_front_", "w265_r20"), std::make_pair("hyper_rear_", "w325_r21")}) {
            const std::string old = std::string(legacy.first) + c.id + "_" + legacy.second;
            const std::string id = rg::migrate_tyre_id(*f, old);
            INFO(old);
            REQUIRE_FALSE(id.empty());
            const auto spec = rg::parse_tyre_id(id);
            REQUIRE(spec.has_value());
            CHECK(spec->compound == c.id);
        }
    }
}

TEST_CASE("tyre spec: a saved setup with old tyre names migrates; a size that no longer fits fails with the new message", "[tyre_spec][garage][migration]") {
    TempDirs dirs;
    // A setup saved before the universal tyres: old generated stems on car_hyper_n2o.
    fs::create_directories(dirs.base + "/user/garage/setups");
    {
        std::ofstream out(dirs.base + "/user/garage/setups/car_hyper_n2o.json", std::ios::binary);
        out << R"({"format":"racing_game.vehicle_setup/1","vehicle":"car_hyper_n2o","options":{
            "tyre_front":"hyper_front_track_w285_r20","tyre_rear":"hyper_rear_road_w255_r21","spring_front":1.1}})";
    }
    std::string err;
    auto g = rg::Garage::open(config_for(dirs), &err);
    INFO(err);
    REQUIRE(g != nullptr);
    const rg::VehicleSetup saved = g->saved_setup("car_hyper_n2o");
    REQUIRE(saved.values.count("tyre_front") == 1);
    REQUIRE(saved.values.count("tyre_rear") == 1);
    CHECK(std::get<std::string>(saved.values.at("tyre_front")) == "track_285_33r20");
    CHECK(std::get<std::string>(saved.values.at("tyre_rear")) == "road_255_38r21");
    CHECK(std::get<double>(saved.values.at("spring_front")) == Approx(1.1));

    // The front size still fits and drives; the rear 255 mm is narrower than the N2O rear fitment: the new message.
    rg::VehicleSetup front_only = saved;
    front_only.values.erase("tyre_rear");
    const rg::CatalogEntry* e = g->catalog().find("car_hyper_n2o");
    REQUIRE(e != nullptr);
    SetupFixture f;
    CHECK(rg::compile_setup(*e, f.table, f.ctx, front_only).ok);
    const rg::CompiledSetup rejected = rg::compile_setup(*e, f.table, f.ctx, saved);
    CHECK_FALSE(rejected.ok);
    CHECK(rejected.error == "tyre_rear: 255 mm is narrower than this axle's 275 mm minimum");

    // Compiling migrates too: an unmigrated setup handed straight to the compiler gives the same verdict.
    rg::VehicleSetup raw;
    raw.vehicle_id = "car_hyper_n2o";
    raw.values["tyre_rear"] = std::string("hyper_rear_road_w285_r21");
    CHECK(rg::compile_setup(*e, f.table, f.ctx, raw).ok);
}

TEST_CASE("tyre spec: every racing_game vehicle file loads with spec-built tyres", "[tyre_spec][tyre][vehicle_data]") {
    const auto fam = families();
    ps::io::EngineMapOptions options;
    options.cache_dir = kRoot + "/out/godot_engine_cache";
    options.generate = false; // validate-only: the tyres do not depend on the engine maps
    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(kRoot + "/data/vehicles")) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("car_", 0) == 0 && entry.path().extension() == ".json") files.push_back(entry.path());
    }
    REQUIRE(files.size() >= 6);
    for (const auto& file : files) {
        INFO(file.string());
        const rg::VehicleTyreSpecs specs = rg::read_vehicle_tyre_specs(file.generic_string());
        REQUIRE(specs.error.empty());
        REQUIRE_FALSE(specs.families_path.empty());
        const ps::vehicle::VehicleDesc desc = rg::load_vehicle(file.generic_string(), options);
        REQUIRE(desc.wheels.size() == specs.wheels.size());
        for (std::size_t i = 0; i < desc.wheels.size(); ++i) {
            REQUIRE_FALSE(specs.wheels[i].compound.empty());
            CHECK(bit_identical(desc.wheels[i].tyre, fam->make(specs.wheels[i])));
            CHECK(desc.wheels[i].tyre.name == rg::tyre_id(specs.wheels[i]));
            // The wheel and its tyre agree on the width; the car's wheel diameter is inside the tyre's tolerance.
            CHECK(desc.wheels[i].wheel_width == Approx(specs.wheels[i].width_mm / 1000.0));
            CHECK(std::abs(rg::tyre_diameter_mm(specs.wheels[i]) - 2000.0 * desc.wheels[i].wheel_radius) <= 12.0);
        }
        // The raw physics loader no longer defaults a tyre (physics_sim 6c5bb87: a wheel without one is a load error):
        // only rg::load_vehicle, which injects the spec-built descs, loads a game vehicle file.
        CHECK_THROWS_WITH(ps::io::load_vehicle_json(file.generic_string(), options), ContainsSubstring("no tyre"));
    }
}

TEST_CASE("tyre injection: two loads of a game car give bit-identical tyres, per wheel name", "[tyre_spec][tyre][vehicle_data]") {
    ps::io::EngineMapOptions options;
    options.cache_dir = kRoot + "/out/godot_engine_cache";
    options.generate = false;
    const std::string path = kRoot + "/data/vehicles/car_hyper_n2o.json";
    const ps::vehicle::VehicleDesc a = rg::load_vehicle(path, options);
    const ps::vehicle::VehicleDesc b = rg::load_vehicle(path, options);
    REQUIRE(a.wheels.size() == b.wheels.size());
    for (std::size_t i = 0; i < a.wheels.size(); ++i) CHECK(bit_identical(a.wheels[i].tyre, b.wheels[i].tyre));
    // The injected descs are what physics_sim's validate_tyre_desc accepts (a load that got this far passed it), and
    // they carry the tyre id as name and a non-empty source.
    for (const auto& w : a.wheels) {
        CHECK_FALSE(w.tyre.name.empty());
        CHECK_FALSE(w.tyre.source.empty());
        CHECK(w.tyre.rolling_resistance_v_eps > 0.0);
        CHECK(w.tyre.low_speed_damping_ratio >= 0.0);
        CHECK(w.tyre.low_speed_damping_ratio <= 2.0);
        CHECK(w.tyre.low_speed_damping_vx > 0.0);
    }
    // A caller-supplied injected tyre for a wheel wins over the file's spec.
    ps::io::VehicleLoadOptions lo;
    lo.maps = options;
    ps::vehicle::TyreDesc mine = a.wheels[0].tyre;
    mine.name = "caller_tyre";
    lo.tyres[a.wheels[0].name] = mine;
    const ps::vehicle::VehicleDesc c = rg::load_vehicle(path, lo);
    CHECK(c.wheels[0].tyre.name == "caller_tyre");
    CHECK(c.wheels[1].tyre.name == a.wheels[1].tyre.name);
}

namespace {
// car_sedan.json as a temp file with every relative reference made absolute (so it loads from anywhere), after
// `mutate` edited the text. A game vehicle with a bad tyre spec is built this way.
std::string car_variant(const TempDirs& dirs, const std::function<void(std::string&)>& mutate) {
    std::ifstream in(kRoot + "/data/vehicles/car_sedan.json", std::ios::binary);
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    mutate(text);
    const std::string prefix = "\"" + kRoot + "/data/vehicles/../";
    for (std::size_t p = text.find("\"../"); p != std::string::npos; p = text.find("\"../", p + prefix.size())) text.replace(p, 4, prefix);
    const std::string path = dirs.base + "/car_variant.json";
    std::ofstream(path, std::ios::binary) << text;
    return path;
}
void replace_first(std::string& text, const std::string& from, const std::string& to) {
    const std::size_t p = text.find(from);
    REQUIRE(p != std::string::npos);
    text.replace(p, from.size(), to);
}
} // namespace

TEST_CASE("tyre injection: a game vehicle with a bad tyre spec reports a clear error", "[tyre_spec][tyre][vehicle_data]") {
    ps::io::EngineMapOptions options;
    options.generate = false;
    TempDirs dirs;

    // The unmodified variant loads (the harness itself is sound).
    CHECK_NOTHROW(rg::load_vehicle(car_variant(dirs, [](std::string&) {}), options));

    // An unknown compound names the file, the wheel and the compound.
    try {
        rg::load_vehicle(car_variant(dirs, [](std::string& t) { replace_first(t, "\"compound\": \"road\"", "\"compound\": \"mud_pie\""); }), options);
        FAIL("an unknown compound must not load");
    } catch (const std::exception& e) {
        const std::string m = e.what();
        CHECK_THAT(m, ContainsSubstring("car_variant.json"));
        CHECK_THAT(m, ContainsSubstring("wheel 'FL'"));
        CHECK_THAT(m, ContainsSubstring("unknown tyre compound \"mud_pie\""));
    }

    // A size the rule cannot build (aspect 0 is rejected by the spec reader) names the problem.
    CHECK_THROWS_WITH(rg::load_vehicle(car_variant(dirs, [](std::string& t) { replace_first(t, "\"aspect\": 45", "\"aspect\": 0"); }), options),
                      ContainsSubstring("tyre_spec"));

    // A wheel without any tyre: physics_sim's own message, naming the wheel.
    CHECK_THROWS_WITH(rg::load_vehicle(car_variant(dirs, [](std::string& t) {
                          replace_first(t, "\"arb_partner\": 1, \"tyre_spec\": { \"compound\": \"road\", \"width_mm\": 225, \"aspect\": 45, \"rim_in\": 17 }, ", "\"arb_partner\": 1, ");
                      }), options),
                      ContainsSubstring("wheel 'FL': no tyre"));

    // Two wheels with one name but different specs cannot share one injected tyre.
    CHECK_THROWS_WITH(rg::load_vehicle(car_variant(dirs, [](std::string& t) {
                          replace_first(t, "\"name\": \"RR\"", "\"name\": \"FL\"");
                          replace_first(t, "\"arb_partner\": 2, \"tyre_spec\": { \"compound\": \"road\", \"width_mm\": 225", "\"arb_partner\": 2, \"tyre_spec\": { \"compound\": \"road\", \"width_mm\": 235");
                      }), options),
                      ContainsSubstring("unique \"name\""));

    // A families file whose base tyre damping is out of physics_sim's range fails at the families file, not deep inside
    // the physics loader.
    {
        std::ifstream in(kRoot + "/data/tyres/families.json", std::ios::binary);
        std::string fam((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        replace_first(fam, "\"low_speed_damping_ratio\": 1.5", "\"low_speed_damping_ratio\": 3.5");
        const std::string fam_path = dirs.base + "/bad_families.json";
        std::ofstream(fam_path, std::ios::binary) << fam;
        CHECK_THROWS_WITH(rg::load_vehicle(car_variant(dirs, [&](std::string& t) { replace_first(t, "\"../tyres/families.json\"", "\"" + fam_path + "\""); }), options),
                          ContainsSubstring("low_speed_damping_ratio"));
    }
}

TEST_CASE("tyre spec: malformed tyre specs in a vehicle file are reported", "[tyre_spec][tyre]") {
    TempDirs dirs;
    const auto write = [&](const char* name, const char* text) {
        const std::string path = dirs.base + "/" + name;
        std::ofstream(path, std::ios::binary) << text;
        return path;
    };
    // Reader only: it looks at the wheels and tyre_families, the physics loader judges the rest.
    auto s = rg::read_vehicle_tyre_specs(write("a.json", R"({"tyre_families":"f.json","wheels":[{"tyre_spec":{"compound":"road","width_mm":225,"aspect":45,"rim_in":17}},{}]})"));
    CHECK(s.error.empty());
    REQUIRE(s.wheels.size() == 2);
    CHECK(s.wheels[0].compound == "road");
    CHECK(s.wheels[1].compound.empty());
    CHECK(s.families_path == dirs.base + "/f.json");
    s = rg::read_vehicle_tyre_specs(write("b.json", R"({"wheels":[{"tyre_spec":{"compound":"road","width_mm":225,"aspect":45,"rim_in":17}}]})"));
    CHECK_THAT(s.error, ContainsSubstring("tyre_families"));
    s = rg::read_vehicle_tyre_specs(write("c.json", R"({"tyre_families":"f.json","wheels":[{"tyre":"x.json","tyre_spec":{"compound":"road","width_mm":225,"aspect":45,"rim_in":17}}]})"));
    CHECK_THAT(s.error, ContainsSubstring("both"));
    s = rg::read_vehicle_tyre_specs(write("d.json", R"({"tyre_families":"f.json","wheels":[{"tyre_spec":{"compound":"road","width_mm":225}}]})"));
    CHECK_THAT(s.error, ContainsSubstring("tyre_spec"));
    s = rg::read_vehicle_tyre_specs(write("e.json", R"({"wheels":[{"tyre":"x.json"}]})")); // a physics_sim style file: nothing to do
    CHECK(s.error.empty());
    CHECK(s.families_path.empty());
}

// Old (file based) vs new (rule built) values of the three stock tyres, for the refactor report.
TEST_CASE("tyre spec: stock tyre values (report)", "[.][diag][tyre_spec]") {
    const auto f = families();
    for (const rg::TyreSpec& s : {rg::TyreSpec{"road", 225, 45, 17}, rg::TyreSpec{"road", 265, 35, 20}, rg::TyreSpec{"road", 325, 30, 21}}) {
        const auto d = f->make(s);
        std::printf("STOCK %s lambda_mux=%.6f lambda_muy=%.6f crr_base=%.6f qsy1=%.6f qsy3=%.6f qsy4=%.9f fz0=%.2f vstiff=%.0f pkx1=%.4f pky1=%.4f rated=%.0f\n",
                    d.name.c_str(), d.lambda_mux, d.lambda_muy, d.rolling_resistance_anchors.crr_base, d.qsy1, d.qsy3, d.qsy4, d.fz0,
                    d.vertical_stiffness, d.pkx1, d.pky1, d.rolling_resistance_anchors.rated_speed_kmh);
    }
}
