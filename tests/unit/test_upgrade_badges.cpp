// Upgrade badges (rg/upgrade_badges.h, data/garage/upgrade_badges.json rg.upgrade_badges/2): the direction table (every
// quantity declares one), relative quantities, the ordered metrics and the badge cap, the units, the option-id/group category
// choice, the placeholder for pending figures, the part-to-part weight fallback and the data-file rejections.
#include "rg/upgrade_badges.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>

using namespace rg;

namespace {

const UpgradeBadgeConfig& config() {
    static const UpgradeBadgeConfig cfg = [] {
        std::string err;
        auto loaded = load_upgrade_badge_config(std::string(RG_SOURCE_DIR) + "/data/garage/upgrade_badges.json", &err);
        if (!loaded) FAIL(err);
        return *loaded;
    }();
    return cfg;
}

PartFigures weight(double kg) { return PartFigures{{UpgradeQuantity::Weight, kg}}; }

SetupPart part(const std::string& id, double mass_kg) {
    SetupPart p;
    p.id = id;
    p.label = id;
    p.mass_kg = mass_kg;
    return p;
}

} // namespace

TEST_CASE("badges: every quantity of the shipped file declares its direction and unit", "[upgrade_badges][garage]") {
    // Lower is better: weight, rolling resistance, the temperatures. Everything else is higher-is-better.
    const std::set<UpgradeQuantity> lower = {UpgradeQuantity::Weight, UpgradeQuantity::RollingResistance, UpgradeQuantity::CoolantTemperature,
                                             UpgradeQuantity::OilTemperature};
    for (UpgradeQuantity q : all_upgrade_quantities()) {
        INFO(upgrade_quantity_name(q));
        const UpgradeQuantityRule* rule = upgrade_quantity_rule(config(), q);
        REQUIRE(rule != nullptr); // the file lists every quantity the format knows
        CHECK(rule->higher_is_better == (lower.count(q) == 0));
        CHECK_FALSE(rule->unit.empty());
    }
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::Weight)->unit == "kg");
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::Power)->unit == "kW");
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::GripLateral)->unit == "g");
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::RollingResistance)->unit == "%");
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::RollingResistance)->relative);
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::CoolantTemperature)->unit == "K");
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::BatteryKwh)->unit == "kWh");
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::Torque)->unit == "N*m");
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::Downforce)->unit == "N");
    CHECK(config().max_badges == 3);
}

TEST_CASE("badges: a heavier part is red, a lighter one green, with a signed number", "[upgrade_badges][garage]") {
    auto heavier = upgrade_badges(config(), "Weight reduction", weight(12.0), weight(6.5));
    REQUIRE(heavier.size() == 1);
    CHECK(heavier[0].text == "+5.5 kg");
    CHECK(heavier[0].worse);
    CHECK(heavier[0].delta == Catch::Approx(5.5));

    auto lighter = upgrade_badges(config(), "Weight reduction", weight(0.0), weight(6.5));
    REQUIRE(lighter.size() == 1);
    CHECK(lighter[0].text == "-6.5 kg");
    CHECK_FALSE(lighter[0].worse);

    // Trailing zeros are dropped, values from 100 up lose their decimals.
    CHECK(upgrade_badges(config(), "Weight reduction", weight(1005.0), weight(1000.0))[0].text == "+5 kg");
    CHECK(upgrade_badges(config(), "Weight reduction", weight(1105.4), weight(1000.0))[0].text == "+105 kg");
}

TEST_CASE("badges: the installed part, a change under the threshold, or a missing figure gives no badge",
          "[upgrade_badges][garage]") {
    CHECK(upgrade_badges(config(), "Weight reduction", weight(6.5), weight(6.5)).empty());
    CHECK(upgrade_badges(config(), "Weight reduction", weight(6.52), weight(6.5)).empty()); // 0.02 kg < min_change 0.05
    CHECK(upgrade_badges(config(), "Weight reduction", PartFigures{}, weight(6.5)).empty());
    CHECK(upgrade_badges(config(), "Weight reduction", weight(6.5), PartFigures{}).empty());
    CHECK(upgrade_badges(config(), "No such category", weight(1.0), weight(6.5)).empty());
}

TEST_CASE("badges: power is higher-is-better, a drop is red", "[upgrade_badges][garage]") {
    const PartFigures installed{{UpgradeQuantity::Power, 300.0}};
    auto up = upgrade_badges(config(), "Turbo system", PartFigures{{UpgradeQuantity::Power, 342.4}}, installed);
    REQUIRE(up.size() == 1);
    CHECK(up[0].text == "+42.4 kW");
    CHECK_FALSE(up[0].worse);
    auto down = upgrade_badges(config(), "Turbo system", PartFigures{{UpgradeQuantity::Power, 290.0}}, installed);
    REQUIRE(down.size() == 1);
    CHECK(down[0].text == "-10 kW");
    CHECK(down[0].worse);
    CHECK(upgrade_badges(config(), "Turbo system", PartFigures{{UpgradeQuantity::Power, 300.2}}, installed).empty()); // < 0.5 kW
}

TEST_CASE("badges: rolling resistance is a relative change, lower is better", "[upgrade_badges][garage]") {
    const PartFigures installed{{UpgradeQuantity::RollingResistance, 0.0100}};
    auto more = upgrade_badges(config(), "Tyres", PartFigures{{UpgradeQuantity::RollingResistance, 0.0104}}, installed);
    REQUIRE(more.size() == 1);
    CHECK(more[0].text == "+4 %");
    CHECK(more[0].worse);
    CHECK(more[0].delta == Catch::Approx(4.0));
    auto less = upgrade_badges(config(), "Tyres", PartFigures{{UpgradeQuantity::RollingResistance, 0.0095}}, installed);
    REQUIRE(less.size() == 1);
    CHECK(less[0].text == "-5 %");
    CHECK_FALSE(less[0].worse);
    // Against a zero baseline a relative change does not exist: no badge, not a division by zero.
    CHECK(upgrade_badges(config(), "Tyres", PartFigures{{UpgradeQuantity::RollingResistance, 0.01}},
                         PartFigures{{UpgradeQuantity::RollingResistance, 0.0}})
              .empty());
}

TEST_CASE("badges: grip is an offset in g with the magnitude-following decimals", "[upgrade_badges][garage]") {
    const PartFigures installed{{UpgradeQuantity::GripLongitudinal, 1.100}, {UpgradeQuantity::GripLateral, 1.200}};
    const PartFigures candidate{{UpgradeQuantity::GripLongitudinal, 1.1523}, {UpgradeQuantity::GripLateral, 1.1999}};
    auto b = upgrade_badges(config(), "Tyres", candidate, installed);
    REQUIRE(b.size() == 1); // lateral changed by 0.0001 g < 0.0005
    CHECK(b[0].quantity == UpgradeQuantity::GripLongitudinal);
    CHECK(b[0].text == "+0.052 g");
    CHECK_FALSE(b[0].worse);
}

TEST_CASE("badges: the category lists ordered metrics and at most max_badges are shown", "[upgrade_badges][garage]") {
    const UpgradeCategoryMetrics* tyres = upgrade_category_metrics(config(), "Tyres");
    REQUIRE(tyres != nullptr);
    CHECK(tyres->metrics == std::vector<UpgradeQuantity>{UpgradeQuantity::GripLongitudinal, UpgradeQuantity::GripLateral,
                                                         UpgradeQuantity::RollingResistance});
    const UpgradeCategoryMetrics* bore = upgrade_category_metrics(config(), "engine_bore");
    REQUIRE(bore != nullptr);
    CHECK(bore->metrics.size() == 4); // more metrics than badges

    const PartFigures installed{{UpgradeQuantity::Power, 300.0},
                                {UpgradeQuantity::RpmTolerance, 8000.0},
                                {UpgradeQuantity::CoolantTemperature, 100.0},
                                {UpgradeQuantity::OilTemperature, 110.0}};
    const PartFigures candidate{{UpgradeQuantity::Power, 320.0},
                                {UpgradeQuantity::RpmTolerance, 7800.0},
                                {UpgradeQuantity::CoolantTemperature, 103.0},
                                {UpgradeQuantity::OilTemperature, 112.0}};
    auto b = upgrade_badges(config(), "engine_bore", candidate, installed);
    REQUIRE(b.size() == 3);
    CHECK(b[0].quantity == UpgradeQuantity::Power);
    CHECK(b[1].quantity == UpgradeQuantity::RpmTolerance);
    CHECK(b[2].quantity == UpgradeQuantity::CoolantTemperature);
    CHECK(b[1].text == "-200 rpm");
    CHECK(b[1].worse);
    CHECK(b[2].text == "+3 K");
    CHECK(b[2].worse);

    // A metric under its threshold is skipped and the next one takes its place.
    PartFigures same_power = candidate;
    same_power[UpgradeQuantity::Power] = 300.1;
    auto skipped = upgrade_badges(config(), "engine_bore", same_power, installed);
    REQUIRE(skipped.size() == 3);
    CHECK(skipped[0].quantity == UpgradeQuantity::RpmTolerance);
    CHECK(skipped[2].quantity == UpgradeQuantity::OilTemperature);
}

TEST_CASE("badges: the option id names the category before its group does", "[upgrade_badges][garage]") {
    std::string key;
    CHECK(upgrade_category_for(config(), "engine_install", "Swaps", &key) != nullptr);
    CHECK(key == "engine_install");
    CHECK(upgrade_category_for(config(), "no_such_option", "Tyres", &key) != nullptr);
    CHECK(key == "Tyres");
    CHECK(upgrade_category_for(config(), "no_such_option", "no_such_group", &key) == nullptr);
    CHECK(upgrade_category_for(config(), "n2o_kit", "N2O", &key)->nitrous_armed);
    CHECK_FALSE(upgrade_category_for(config(), "turbo_install", "Turbo system", &key)->nitrous_armed);
}

TEST_CASE("badges: a pending figure shows a placeholder, a settled one its number", "[upgrade_badges][garage]") {
    const PartFigures installed{{UpgradeQuantity::Power, 300.0}};
    const std::set<UpgradeQuantity> pending{UpgradeQuantity::Power};
    auto b = upgrade_badges(config(), "Turbo system", PartFigures{}, installed, &pending);
    REQUIRE(b.size() == 1);
    CHECK(b[0].pending);
    CHECK(b[0].text == kPendingBadgeText);
    auto settled = upgrade_badges(config(), "Turbo system", PartFigures{{UpgradeQuantity::Power, 350.0}}, installed, nullptr);
    REQUIRE(settled.size() == 1);
    CHECK_FALSE(settled[0].pending);
}

TEST_CASE("badges: option_part_badges compares the parts of one option against the installed one",
          "[upgrade_badges][garage]") {
    OptionView view;
    view.def.group = "Weight reduction";
    view.parts = {part("stock", 6.5), part("sport", 12.0), part("removed", 0.0), part("light", 6.5)};

    auto to_sport = option_part_badges(config(), view, "stock", "sport");
    REQUIRE(to_sport.size() == 1);
    CHECK(to_sport[0].text == "+5.5 kg");

    auto to_removed = option_part_badges(config(), view, "stock", "removed");
    REQUIRE(to_removed.size() == 1);
    CHECK(to_removed[0].text == "-6.5 kg");

    CHECK(option_part_badges(config(), view, "stock", "stock").empty());
    CHECK(option_part_badges(config(), view, "stock", "light").empty()); // same mass: no badge
    CHECK(option_part_badges(config(), view, "stock", "unknown").empty());

    view.def.group = "Engine";
    CHECK(option_part_badges(config(), view, "stock", "sport").empty()); // a category without figures
}

namespace {

const char* kRefs = R"({"tyres": {"surface_table": "s.json", "surface": "asphalt", "rolling_resistance_speed_kmh": 100},
  "dyno": {"quality": "sketch", "rpm_step": 250, "rpm_min": 2000, "limiter_margin_rpm": 50, "debounce_ms": 200},
  "thermal": {"ambient_c": 30, "air_speed_kmh": 100, "friction_power_fraction": 0.09}})";

// A minimal valid /2 file; the loader rejections each change exactly one piece.
std::string file_text(const std::string& quantities, const std::string& categories, const std::string& format = "rg.upgrade_badges/2",
                      const std::string& refs = kRefs) {
    return R"({"format": ")" + format + R"(", "max_badges": 3, "references": )" + refs + R"(, "quantities": {)" + quantities +
           R"(}, "categories": {)" + categories + "}}";
}
const char* kWeight = R"("weight": {"unit": "kg", "higher_is_better": false, "max_decimals": 1, "min_change": 0.05})";
const char* kPower = R"("power": {"unit": "kW", "higher_is_better": true, "max_decimals": 1, "min_change": 0.5})";
const char* kCat = R"("Engine": {"metrics": ["power", "weight"]})";

// weight with a replaced body next to the ordinary power
std::string with_weight(const std::string& weight_body) { return "\"weight\": " + weight_body + "," + kPower; }

struct Loaded {
    bool ok = false;
    std::string err;
};

} // namespace

TEST_CASE("badges: the config loader accepts a minimal /2 file and refuses each kind of bad one", "[upgrade_badges][garage]") {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "rg_upgrade_badges_test";
    fs::create_directories(dir);
    int n = 0;
    const auto load = [&](const std::string& text) {
        const fs::path p = dir / ("f" + std::to_string(n++) + ".json");
        std::ofstream(p) << text;
        Loaded l;
        l.ok = load_upgrade_badge_config(p.string(), &l.err).has_value();
        return l;
    };
    const std::string both = std::string(kWeight) + "," + kPower;

    SECTION("a minimal file loads") { CHECK(load(file_text(both, kCat)).ok); }
    SECTION("the old /1 format is refused with its name") {
        const auto l = load(file_text(both, kCat, "rg.upgrade_badges/1"));
        CHECK_FALSE(l.ok);
        CHECK(l.err.find("rg.upgrade_badges/1") != std::string::npos);
    }
    SECTION("an unknown format") { CHECK_FALSE(load(file_text(both, kCat, "x")).ok); }
    SECTION("a quantity without a direction is refused: there is no default") {
        const auto l = load(file_text(with_weight(R"({"unit": "kg", "max_decimals": 1, "min_change": 0.05})"), kCat));
        CHECK_FALSE(l.ok);
        CHECK(l.err.find("higher_is_better") != std::string::npos);
    }
    SECTION("a direction that is not a boolean is refused") {
        CHECK_FALSE(load(file_text(with_weight(R"({"unit": "kg", "higher_is_better": "no", "max_decimals": 1, "min_change": 0.05})"), kCat)).ok);
    }
    SECTION("the old key decimals is refused with the rename") {
        const auto l = load(file_text(with_weight(R"({"unit": "kg", "higher_is_better": false, "decimals": 1, "min_change": 0.05})"), kCat));
        CHECK_FALSE(l.ok);
        CHECK(l.err.find("max_decimals") != std::string::npos);
    }
    SECTION("max_decimals out of range or fractional") {
        CHECK_FALSE(load(file_text(with_weight(R"({"unit": "kg", "higher_is_better": false, "max_decimals": 7, "min_change": 0.05})"), kCat)).ok);
        CHECK_FALSE(load(file_text(with_weight(R"({"unit": "kg", "higher_is_better": false, "max_decimals": 1.5, "min_change": 0.05})"), kCat)).ok);
    }
    SECTION("a negative min_change, a missing unit, an unknown key") {
        CHECK_FALSE(load(file_text(with_weight(R"({"unit": "kg", "higher_is_better": false, "max_decimals": 1, "min_change": -1})"), kCat)).ok);
        CHECK_FALSE(load(file_text(with_weight(R"({"higher_is_better": false, "max_decimals": 1, "min_change": 0.05})"), kCat)).ok);
        CHECK_FALSE(load(file_text(with_weight(R"({"unit": "kg", "higher_is_better": false, "max_decimals": 1, "min_change": 0.05, "colour": "red"})"), kCat)).ok);
    }
    SECTION("an unknown quantity name") { CHECK_FALSE(load(file_text(both + R"(, "speed": {})", kCat)).ok); }
    SECTION("primary/secondary are refused in favour of metrics") {
        const auto l = load(file_text(both, R"("Engine": {"primary": "power", "secondary": "weight"})"));
        CHECK_FALSE(l.ok);
        CHECK(l.err.find("metrics") != std::string::npos);
    }
    SECTION("a category metric must name a listed quantity, only once, and the list may not be empty") {
        CHECK_FALSE(load(file_text(both, R"("Engine": {"metrics": ["torque"]})")).ok);
        CHECK_FALSE(load(file_text(both, R"("Engine": {"metrics": ["power", "power"]})")).ok);
        CHECK_FALSE(load(file_text(both, R"("Engine": {"metrics": []})")).ok);
        CHECK_FALSE(load(file_text(both, R"("Engine": {"metrics": ["power"], "colour": 1})")).ok);
    }
    SECTION("references are required and validated") {
        CHECK_FALSE(load(file_text(both, kCat, "rg.upgrade_badges/2", R"({"tyres": {}, "dyno": {}, "thermal": {}})")).ok);
        CHECK_FALSE(load(file_text(both, kCat, "rg.upgrade_badges/2",
                                   R"({"tyres": {"surface_table": "s.json", "surface": "asphalt", "rolling_resistance_speed_kmh": 100},
        "dyno": {"quality": "ultra", "rpm_step": 250, "rpm_min": 2000, "limiter_margin_rpm": 50, "debounce_ms": 200},
        "thermal": {"ambient_c": 30, "air_speed_kmh": 100, "friction_power_fraction": 0.09}})"))
                          .ok);
        CHECK_FALSE(load(file_text(both, kCat, "rg.upgrade_badges/2",
                                   R"({"tyres": {"surface_table": "s.json", "surface": "asphalt", "rolling_resistance_speed_kmh": 100},
        "dyno": {"quality": "sketch", "rpm_step": 250, "rpm_min": 2000, "limiter_margin_rpm": 50, "debounce_ms": 200},
        "thermal": {"ambient_c": 300, "air_speed_kmh": 100, "friction_power_fraction": 0.09}})"))
                          .ok);
    }
    SECTION("a missing file") {
        std::string err;
        CHECK_FALSE(load_upgrade_badge_config((dir / "nope.json").string(), &err));
        CHECK_FALSE(err.empty());
    }
    fs::remove_all(dir);
}

TEST_CASE("badges: the shipped file's references resolve and every category metric exists", "[upgrade_badges][garage]") {
    const UpgradeReferences& r = config().references;
    CHECK(std::filesystem::exists(r.surface_table));
    CHECK(r.surface == "asphalt");
    CHECK(r.rolling_resistance_speed_kmh == Catch::Approx(100.0));
    CHECK(r.dyno_quality == "sketch");
    for (const auto& [name, cat] : config().categories) {
        INFO(name);
        CHECK_FALSE(cat.metrics.empty());
        for (UpgradeQuantity q : cat.metrics) CHECK(upgrade_quantity_rule(config(), q) != nullptr);
    }
}
