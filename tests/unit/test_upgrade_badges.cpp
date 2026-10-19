// Upgrade badges (rg/upgrade_badges.h): the direction table, the category choice, the two-badge cap and the
// part-to-part weight deltas on the garage's own option view.
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

TEST_CASE("badges: the quantity table says which way is better", "[upgrade_badges][garage]") {
    // Lower is better for weight only; everything else listed here is higher-is-better.
    CHECK_FALSE(upgrade_quantity_rule(config(), UpgradeQuantity::Weight)->higher_is_better);
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::Power)->higher_is_better);
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::Torque)->higher_is_better);
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::Grip)->higher_is_better);
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::Downforce)->higher_is_better);
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::BrakingTorque)->higher_is_better);
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::BatteryKwh)->higher_is_better);
    CHECK(upgrade_quantity_rule(config(), UpgradeQuantity::Weight)->unit == "kg");
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
}

TEST_CASE("badges: the installed part, a change under the threshold, or a missing figure gives no badge",
          "[upgrade_badges][garage]") {
    CHECK(upgrade_badges(config(), "Weight reduction", weight(6.5), weight(6.5)).empty());
    CHECK(upgrade_badges(config(), "Weight reduction", weight(6.8), weight(6.5)).empty()); // 0.3 kg < 0.5 kg
    CHECK(upgrade_badges(config(), "Weight reduction", PartFigures{}, weight(6.5)).empty());
    CHECK(upgrade_badges(config(), "No such category", weight(1.0), weight(6.5)).empty());
}

TEST_CASE("badges: the category table names the primary metric", "[upgrade_badges][garage]") {
    const UpgradeCategoryMetrics* weight_page = upgrade_category_metrics(config(), "Weight reduction");
    REQUIRE(weight_page != nullptr);
    CHECK(weight_page->primary == UpgradeQuantity::Weight);
    CHECK_FALSE(weight_page->has_secondary);
    CHECK(upgrade_category_metrics(config(), "Engine") == nullptr); // not in the config: no badges
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

TEST_CASE("badges: the config loader refuses bad files and applies a secondary metric", "[upgrade_badges][garage]") {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "rg_upgrade_badges_test";
    fs::create_directories(dir);
    const auto write = [&](const char* name, const std::string& text) {
        const fs::path p = dir / name;
        std::ofstream(p) << text;
        return p.string();
    };
    const std::string quantities =
        R"("quantities": {"weight": {"unit": "kg", "higher_is_better": false, "decimals": 1, "min_change": 0.5},)"
        R"( "power": {"unit": "kW", "higher_is_better": true, "decimals": 0, "min_change": 10}})";
    std::string err;
    CHECK_FALSE(load_upgrade_badge_config(write("a.json", R"({"format": "x", "quantities": {}, "categories": {}})"), &err));
    CHECK_FALSE(load_upgrade_badge_config(write("b.json", R"({"format": "rg.upgrade_badges/1", "quantities": {"speed": {}}, "categories": {}})"), &err));
    CHECK_FALSE(load_upgrade_badge_config(
        write("c.json", R"({"format": "rg.upgrade_badges/1", )" + quantities + R"(, "categories": {"Engine": {"primary": "torque"}}})"), &err));
    CHECK_FALSE(load_upgrade_badge_config(
        write("d.json", R"({"format": "rg.upgrade_badges/1", )" + quantities + R"(, "categories": {"Engine": {"primary": "power", "secondary": "power"}}})"), &err));

    auto cfg = load_upgrade_badge_config(
        write("e.json", R"({"format": "rg.upgrade_badges/1", )" + quantities + R"(, "categories": {"Engine": {"primary": "power", "secondary": "weight"}}})"), &err);
    REQUIRE(cfg.has_value());
    PartFigures installed{{UpgradeQuantity::Power, 100.0}, {UpgradeQuantity::Weight, 150.0}};
    // Primary +50 kW (green), secondary +20 kg (red): two badges, primary first.
    auto both = upgrade_badges(*cfg, "Engine", PartFigures{{UpgradeQuantity::Power, 150.0}, {UpgradeQuantity::Weight, 170.0}}, installed);
    REQUIRE(both.size() == 2);
    CHECK(both[0].text == "+50 kW");
    CHECK_FALSE(both[0].worse);
    CHECK(both[1].text == "+20.0 kg");
    CHECK(both[1].worse);
    // Primary below its threshold (5 kW < 10): hidden; the secondary alone remains.
    auto only_secondary = upgrade_badges(*cfg, "Engine", PartFigures{{UpgradeQuantity::Power, 105.0}, {UpgradeQuantity::Weight, 120.0}}, installed);
    REQUIRE(only_secondary.size() == 1);
    CHECK(only_secondary[0].text == "-30.0 kg");
    CHECK_FALSE(only_secondary[0].worse);
    // Higher-is-better: a power DROP is red.
    auto drop = upgrade_badges(*cfg, "Engine", PartFigures{{UpgradeQuantity::Power, 60.0}, {UpgradeQuantity::Weight, 150.0}}, installed);
    REQUIRE(drop.size() == 1);
    CHECK(drop[0].worse);
    fs::remove_all(dir);
}
