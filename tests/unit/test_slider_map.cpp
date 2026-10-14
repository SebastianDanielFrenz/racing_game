// test_slider_map.cpp - garage numeric inputs (2026-10-13): the data-driven slider mapping of a numeric setup option
// (rg/slider_map.h), its parsing and validation in the option table, the stored-factor <-> absolute-unit conversions the
// garage boxes use (rg::NumberEntry) and the 2000..20000 rpm rev limiter / 0.1..30 bar boost ranges that come from data.
#include "rg/slider_map.h"
#include "rg/vehicle_load.h"
#include "rg/vehicle_setup.h"

#include "ps/drivetrain/powertrain_desc.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <filesystem>
#include <string>
#include <variant>

namespace {
namespace fs = std::filesystem;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
const std::string kRoot = RG_SOURCE_DIR;

rg::SliderSpec log_spec(double lo, double hi) {
    rg::SliderSpec s;
    s.mapping = rg::SliderMapping::Log;
    s.abs_min = lo;
    s.abs_max = hi;
    return s;
}

std::string options_with(const std::string& slider_json, const std::string& extra = "") {
    return R"({"format":"racing_game.vehicle_setup_options/1","options":[
        {"id":"a","kind":"scale","file":"engine","pointers":["/x"],"unit":"x")" + extra + R"(,"slider":)" + slider_json + "}]}";
}

struct Setup {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    rg::ScopedWorkDir work{fs::temp_directory_path().generic_string(), "rg_slider_tests"};
    Setup() {
        std::string err;
        auto c = rg::load_vehicle_catalog(kRoot + "/data/vehicles/catalog.json", kRoot, &err);
        REQUIRE(c.has_value());
        catalog = *c;
        auto t = rg::load_setup_options(kRoot + "/data/vehicles/setup_options.json", &err);
        INFO(err);
        REQUIRE(t.has_value());
        table = *t;
        ctx.tyre_families_path = kRoot + "/data/tyres/families.json";
        ctx.work_root = work.path();
        ctx.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
        auto store = rg::EngineStore::open(kRoot + "/data/engines/engine_catalog.json", kRoot, "", &err);
        INFO(err);
        REQUIRE(store != nullptr);
        ctx.engines = std::shared_ptr<const rg::EngineStore>(std::move(store));
    }
    rg::VehicleSetup setup_of(const std::string& car, const std::string& id, rg::SetupValue v) const {
        rg::VehicleSetup s;
        s.vehicle_id = car;
        s.values[id] = std::move(v);
        return s;
    }
    const rg::OptionView* view_of(const rg::SetupModel& m, const std::string& id) const {
        for (const rg::OptionView& v : m.options) if (v.def.id == id) return &v;
        return nullptr;
    }
};
} // namespace

TEST_CASE("slider maths: the log mapping round-trips at both ends and in the middle", "[slider][garage]") {
    const double lo = 2000.0, hi = 20000.0;
    CHECK(rg::slider_position(rg::SliderMapping::Log, lo, hi, lo) == Approx(0.0).margin(1e-12));
    CHECK(rg::slider_position(rg::SliderMapping::Log, lo, hi, hi) == Approx(1.0));
    CHECK(rg::slider_value(rg::SliderMapping::Log, lo, hi, 0.0) == Approx(lo));
    CHECK(rg::slider_value(rg::SliderMapping::Log, lo, hi, 1.0) == Approx(hi));
    // equal ratios per equal distance: the middle is the geometric mean, a decade is the whole span here
    CHECK(rg::slider_value(rg::SliderMapping::Log, lo, hi, 0.5) == Approx(std::sqrt(lo * hi)));
    CHECK(rg::slider_position(rg::SliderMapping::Log, lo, hi, 6000.0) == Approx(std::log10(3.0)));
    for (double v : {2000.0, 2500.0, 4472.0, 8250.0, 12345.0, 19999.0, 20000.0}) {
        const double p = rg::slider_position(rg::SliderMapping::Log, lo, hi, v);
        CHECK(rg::slider_value(rg::SliderMapping::Log, lo, hi, p) == Approx(v).epsilon(1e-12));
    }
    // outside the range clamps instead of extrapolating
    CHECK(rg::slider_position(rg::SliderMapping::Log, lo, hi, 100.0) == 0.0);
    CHECK(rg::slider_position(rg::SliderMapping::Log, lo, hi, 1e9) == 1.0);
    CHECK(rg::slider_position(rg::SliderMapping::Log, lo, hi, -5.0) == 0.0);
    CHECK(rg::slider_value(rg::SliderMapping::Log, lo, hi, -1.0) == Approx(lo));
    CHECK(rg::slider_value(rg::SliderMapping::Log, lo, hi, 7.0) == Approx(hi));
}

TEST_CASE("slider maths: the linear mapping and rounding", "[slider][garage]") {
    CHECK(rg::slider_position(rg::SliderMapping::Linear, 20.0, 100.0, 60.0) == Approx(0.5));
    CHECK(rg::slider_value(rg::SliderMapping::Linear, 20.0, 100.0, 0.25) == Approx(40.0));
    CHECK(rg::slider_position(rg::SliderMapping::Linear, 20.0, 100.0, 0.0) == 0.0);
    CHECK(rg::slider_position(rg::SliderMapping::Linear, 5.0, 5.0, 5.0) == 0.0); // a degenerate range does not divide by zero
    CHECK(rg::round_to_decimals(4472.1356, 0) == 4472.0);
    CHECK(rg::round_to_decimals(1.2345, 2) == Approx(1.23));
}

TEST_CASE("slider spec: validation and the stock value always being allowed", "[slider][garage]") {
    CHECK(rg::validate_slider_spec(log_spec(2000, 20000)).empty());
    CHECK_THAT(rg::validate_slider_spec(log_spec(0, 20000)), ContainsSubstring("abs_min > 0"));
    CHECK_THAT(rg::validate_slider_spec(log_spec(5, 5)), ContainsSubstring("below"));
    CHECK_THAT(rg::validate_slider_spec(log_spec(9, 5)), ContainsSubstring("below"));
    rg::SliderSpec s = log_spec(1, 10);
    s.factor = 0.0;
    CHECK_THAT(rg::validate_slider_spec(s), ContainsSubstring("factor"));
    s = log_spec(1, 10);
    s.decimals = 7;
    CHECK_THAT(rg::validate_slider_spec(s), ContainsSubstring("decimals"));
    s = log_spec(1, std::nan(""));
    CHECK_THAT(rg::validate_slider_spec(s), ContainsSubstring("finite"));
    rg::SliderSpec lin;
    lin.abs_min = -1.0;
    lin.abs_max = 1.0;
    CHECK_FALSE(rg::validate_slider_spec(lin).empty());

    // a stock value inside the range changes nothing, one outside widens the range so the stock setup stays valid
    const auto inside = rg::slider_bounds(log_spec(2000, 20000), 8250.0);
    CHECK(inside.first == 2000.0);
    CHECK(inside.second == 20000.0);
    const auto above = rg::slider_bounds(log_spec(2000, 20000), 25000.0);
    CHECK(above.second == 25000.0);
    const auto below = rg::slider_bounds(log_spec(2000, 20000), 1500.0);
    CHECK(below.first == 1500.0);
}

TEST_CASE("option table: the slider block is parsed, validated and exclusive with min/max", "[slider][setup][garage]") {
    std::string err;
    const auto good = rg::parse_setup_options(
        options_with(R"({"mapping":"log","abs_min":2000,"abs_max":20000,"unit":"rpm","decimals":0})"), "t", &err);
    INFO(err);
    REQUIRE(good.has_value());
    const rg::SetupOptionDef* a = good->find("a");
    REQUIRE(a != nullptr);
    REQUIRE(a->slider.has_value());
    CHECK(a->slider->mapping == rg::SliderMapping::Log);
    CHECK(a->slider->abs_min == 2000.0);
    CHECK(a->slider->abs_max == 20000.0);
    CHECK(a->slider->unit == "rpm");
    CHECK(a->slider->decimals == 0);
    CHECK(a->slider->factor == 1.0);

    // the unit falls back to the option's own
    const auto fallback = rg::parse_setup_options(options_with(R"({"mapping":"linear","abs_min":1,"abs_max":2})"), "t", &err);
    REQUIRE(fallback.has_value());
    CHECK(fallback->find("a")->slider->unit == "x");

    const auto rejected = [&](const std::string& slider, const std::string& extra, const char* reason) {
        std::string e;
        CHECK_FALSE(rg::parse_setup_options(options_with(slider, extra), "t", &e).has_value());
        INFO(slider);
        CHECK_THAT(e, ContainsSubstring(reason));
    };
    rejected(R"({"mapping":"cubic","abs_min":1,"abs_max":2})", "", "mapping");
    rejected(R"({"mapping":"log","abs_min":1})", "", "abs_min and abs_max");
    rejected(R"({"mapping":"log","abs_min":0,"abs_max":2})", "", "abs_min > 0");
    rejected(R"({"mapping":"log","abs_min":3,"abs_max":2})", "", "below");
    rejected(R"({"mapping":"log","abs_min":1,"abs_max":2,"decimals":1.5})", "", "whole number");
    rejected(R"({"mapping":"log","abs_min":1,"abs_max":2,"factor":-1})", "", "factor");
    rejected(R"("log")", "", "object");
    rejected(R"({"mapping":"log","abs_min":1,"abs_max":2})", R"(,"min":0.5,"max":1.5,"step":0.01)", "min");
    // only a scale option has a slider
    std::string e;
    CHECK_FALSE(rg::parse_setup_options(R"({"format":"racing_game.vehicle_setup_options/1","options":[
        {"id":"a","kind":"bool","file":"vehicle","pointers":["/x"],"slider":{"mapping":"log","abs_min":1,"abs_max":2}}]})", "t", &e).has_value());
    CHECK_THAT(e, ContainsSubstring("scale option only"));
}

TEST_CASE("shipped options: rev limiter, boost target and throttle response carry their slider in the data", "[slider][setup][garage]") {
    Setup f;
    const rg::SetupOptionDef* rev = f.table.find("engine_rev_limit");
    REQUIRE(rev != nullptr);
    REQUIRE(rev->slider.has_value());
    CHECK(rev->slider->mapping == rg::SliderMapping::Log);
    CHECK(rev->slider->abs_min == 2000.0);
    CHECK(rev->slider->abs_max == 20000.0);
    CHECK(rev->slider->unit == "rpm");
    const rg::SetupOptionDef* boost = f.table.find("turbo_boost_target");
    REQUIRE(boost != nullptr);
    REQUIRE(boost->slider.has_value());
    CHECK(boost->slider->mapping == rg::SliderMapping::Log);
    CHECK(boost->slider->abs_min == Approx(0.1));
    CHECK(boost->slider->abs_max == Approx(30.0));
    CHECK(boost->slider->unit == "bar");
    const rg::SetupOptionDef* tau = f.table.find("engine_throttle_response");
    REQUIRE(tau != nullptr);
    REQUIRE(tau->slider.has_value());
    CHECK(tau->slider->unit == "ms");
    CHECK(tau->slider->factor == 1000.0);
}

TEST_CASE("rev limiter: 2000 to 20000 rpm from the data, stored as rpm / stock, old saved setups still valid", "[slider][setup][garage]") {
    Setup f;
    const rg::CatalogEntry* hyper = f.catalog.find("car_hyper");
    REQUIRE(hyper != nullptr);
    const rg::SetupModel model = rg::build_setup_model(*hyper, f.table, f.ctx);
    INFO(model.error);
    REQUIRE(model.error.empty());
    const rg::OptionView* view = f.view_of(model, "engine_rev_limit");
    REQUIRE(view != nullptr);
    REQUIRE(view->available);
    REQUIRE(view->stock_numbers.size() == 1);
    const double stock = view->stock_numbers[0];
    CHECK(stock == Approx(8250.0)); // data/engines/hyper_v8_one1_audio.json
    CHECK(view->min == Approx(2000.0 / stock));
    CHECK(view->max == Approx(20000.0 / stock));

    const rg::NumberEntry e = rg::number_entry(*view);
    CHECK(e.positional);
    CHECK(e.mapping == rg::SliderMapping::Log);
    CHECK(e.unit == "rpm");
    CHECK(e.decimals == 0);
    CHECK(e.multiplier == Approx(stock));
    CHECK(e.lo == Approx(2000.0));
    CHECK(e.hi == Approx(20000.0));
    CHECK(rg::format_entry(e, 1.0) == "8250 rpm");
    // both ends of the slider and back
    CHECK(rg::entry_slider_position(e, 2000.0 / stock) == Approx(0.0).margin(1e-12));
    CHECK(rg::entry_slider_position(e, 20000.0 / stock) == Approx(1.0));
    CHECK(rg::entry_value_at_position(e, 0.0) * stock == Approx(2000.0));
    CHECK(rg::entry_value_at_position(e, 1.0) * stock == Approx(20000.0));
    for (double rpm : {2000.0, 3000.0, 6500.0, 8250.0, 14000.0, 20000.0}) {
        const double p = rg::entry_slider_position(e, rpm / stock);
        CHECK(rg::entry_value_at_position(e, p) * stock == Approx(rpm).margin(0.5)); // rounded to whole rpm
    }
    // typed numbers: the bounds are inclusive, anything outside is refused with the range in the message
    CHECK(rg::entry_from_displayed(e, 2000.0).ok);
    CHECK(rg::entry_from_displayed(e, 20000.0).ok);
    CHECK(rg::entry_from_displayed(e, 20000.0).value == Approx(20000.0 / stock));
    const rg::EntryConversion low = rg::entry_from_displayed(e, 1999.0);
    CHECK_FALSE(low.ok);
    CHECK_THAT(low.message, ContainsSubstring("2000 to 20000 rpm"));
    CHECK_FALSE(rg::entry_from_displayed(e, 20001.0).ok);
    CHECK_FALSE(rg::entry_from_displayed(e, std::nan("")).ok);
    CHECK_FALSE(rg::entry_from_displayed(e, INFINITY).ok);

    // compile_setup takes exactly the range of the data
    for (double rpm : {2000.0, 20000.0}) {
        const auto compiled = rg::compile_setup(*hyper, f.table, f.ctx, f.setup_of("car_hyper", "engine_rev_limit", rpm / stock));
        INFO(rpm << ": " << compiled.error);
        CHECK(compiled.ok);
    }
    const auto too_low = rg::compile_setup(*hyper, f.table, f.ctx, f.setup_of("car_hyper", "engine_rev_limit", 1999.0 / stock));
    CHECK_FALSE(too_low.ok);
    CHECK_THAT(too_low.error, ContainsSubstring("outside the allowed range 2000 to 20000 rpm"));
    CHECK_FALSE(rg::compile_setup(*hyper, f.table, f.ctx, f.setup_of("car_hyper", "engine_rev_limit", 20001.0 / stock)).ok);
    // a setup saved before this change (the old 0.75..1.0 scale) means the same and still compiles
    for (double scale : {0.75, 0.9, 1.0}) {
        CHECK(rg::compile_setup(*hyper, f.table, f.ctx, f.setup_of("car_hyper", "engine_rev_limit", scale)).ok);
    }
}

TEST_CASE("boost target: 0.1 to 30 bar from the data", "[slider][setup][garage]") {
    Setup f;
    const rg::CatalogEntry* hyper = f.catalog.find("car_hyper");
    REQUIRE(hyper != nullptr);
    const rg::SetupModel model = rg::build_setup_model(*hyper, f.table, f.ctx);
    REQUIRE(model.error.empty());
    const rg::OptionView* view = f.view_of(model, "turbo_boost_target");
    REQUIRE(view != nullptr);
    REQUIRE(view->available);
    const double stock = view->stock_numbers.at(0); // bar
    CHECK(view->min == Approx(0.1 / stock));
    CHECK(view->max == Approx(30.0 / stock));
    const rg::NumberEntry e = rg::number_entry(*view);
    CHECK(e.unit == "bar");
    CHECK(rg::entry_slider_position(e, 0.1 / stock) == Approx(0.0).margin(1e-12));
    CHECK(rg::entry_slider_position(e, 30.0 / stock) == Approx(1.0));
    CHECK(rg::entry_slider_position(e, 1.0) == Approx(std::log(stock / 0.1) / std::log(300.0)));
    CHECK(rg::entry_from_displayed(e, 0.1).ok);
    CHECK(rg::entry_from_displayed(e, 30.0).ok);
    CHECK_FALSE(rg::entry_from_displayed(e, 0.09).ok);
    CHECK_FALSE(rg::entry_from_displayed(e, 30.5).ok);
    CHECK(rg::compile_setup(*hyper, f.table, f.ctx, f.setup_of("car_hyper", "turbo_boost_target", 30.0 / stock)).ok);
    const auto over = rg::compile_setup(*hyper, f.table, f.ctx, f.setup_of("car_hyper", "turbo_boost_target", 31.0 / stock));
    CHECK_FALSE(over.ok);
    CHECK_THAT(over.error, ContainsSubstring("bar is outside the allowed range 0.10 to 30.00 bar"));
}

TEST_CASE("number entries: brake bias in percent, a gear in its absolute ratio, a plain scale as a factor", "[slider][setup][garage]") {
    Setup f;
    const rg::CatalogEntry* hyper = f.catalog.find("car_hyper");
    REQUIRE(hyper != nullptr);
    const rg::SetupModel model = rg::build_setup_model(*hyper, f.table, f.ctx);
    REQUIRE(model.error.empty());

    const rg::OptionView* bias = f.view_of(model, "brake_bias");
    REQUIRE(bias != nullptr);
    const rg::NumberEntry be = rg::number_entry(*bias);
    CHECK(be.unit == "%");
    CHECK(be.multiplier == 100.0);
    CHECK_FALSE(be.positional);
    CHECK(be.lo == Approx(bias->min * 100.0));
    CHECK(rg::entry_from_displayed(be, 50.0).value == Approx(0.5));
    CHECK_FALSE(rg::entry_from_displayed(be, 99.0).ok);
    CHECK(rg::format_entry(be, 0.5) == "50.0 %");

    const rg::OptionView* gears = f.view_of(model, "gear_ratios");
    REQUIRE(gears != nullptr);
    REQUIRE(gears->list_size >= 5);
    const rg::NumberEntry g0 = rg::number_entry(*gears, 0);
    const rg::NumberEntry g1 = rg::number_entry(*gears, 1);
    CHECK(g0.multiplier == Approx(gears->stock_numbers[0]));
    CHECK(g1.multiplier == Approx(gears->stock_numbers[1]));
    CHECK(g0.lo == Approx(gears->min * gears->stock_numbers[0]));
    CHECK(g0.hi == Approx(gears->max * gears->stock_numbers[0]));
    // typing the ratio the gear already has stores factor 1; the limits are per gear, in ratio units
    CHECK(rg::entry_from_displayed(g1, gears->stock_numbers[1]).value == Approx(1.0));
    CHECK_FALSE(rg::entry_from_displayed(g1, gears->stock_numbers[1] * (gears->max + 0.5)).ok);

    const rg::OptionView* spring = f.view_of(model, "spring_front");
    REQUIRE(spring != nullptr);
    const rg::NumberEntry se = rg::number_entry(*spring);
    CHECK(se.unit == "x");
    CHECK(se.multiplier == 1.0);
    CHECK(rg::format_entry(se, 1.25) == "x1.25");
    CHECK(rg::entry_from_displayed(se, 1.3).ok);
    CHECK_FALSE(rg::entry_from_displayed(se, 1.31).ok);
}
