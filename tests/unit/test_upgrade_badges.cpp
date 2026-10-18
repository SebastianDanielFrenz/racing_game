// Upgrade badges (rg/upgrade_badges.h): the direction table, the category choice, the two-badge cap and the
// part-to-part weight deltas on the garage's own option view.
#include "rg/upgrade_badges.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace rg;

namespace {

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
    CHECK_FALSE(upgrade_quantity_rule(UpgradeQuantity::Weight).higher_is_better);
    CHECK(upgrade_quantity_rule(UpgradeQuantity::Power).higher_is_better);
    CHECK(upgrade_quantity_rule(UpgradeQuantity::Torque).higher_is_better);
    CHECK(upgrade_quantity_rule(UpgradeQuantity::Grip).higher_is_better);
    CHECK(upgrade_quantity_rule(UpgradeQuantity::Downforce).higher_is_better);
    CHECK(upgrade_quantity_rule(UpgradeQuantity::BrakingTorque).higher_is_better);
    CHECK(upgrade_quantity_rule(UpgradeQuantity::BatteryKwh).higher_is_better);
    CHECK(std::string(upgrade_quantity_rule(UpgradeQuantity::Weight).unit) == "kg");
}

TEST_CASE("badges: a heavier part is red, a lighter one green, with a signed number", "[upgrade_badges][garage]") {
    auto heavier = upgrade_badges("Weight reduction", weight(12.0), weight(6.5));
    REQUIRE(heavier.size() == 1);
    CHECK(heavier[0].text == "+5.5 kg");
    CHECK(heavier[0].worse);
    CHECK(heavier[0].delta == Catch::Approx(5.5));

    auto lighter = upgrade_badges("Weight reduction", weight(0.0), weight(6.5));
    REQUIRE(lighter.size() == 1);
    CHECK(lighter[0].text == "-6.5 kg");
    CHECK_FALSE(lighter[0].worse);
}

TEST_CASE("badges: the installed part, a change under the threshold, or a missing figure gives no badge",
          "[upgrade_badges][garage]") {
    CHECK(upgrade_badges("Weight reduction", weight(6.5), weight(6.5)).empty());
    CHECK(upgrade_badges("Weight reduction", weight(6.8), weight(6.5)).empty()); // 0.3 kg < 0.5 kg
    CHECK(upgrade_badges("Weight reduction", PartFigures{}, weight(6.5)).empty());
    CHECK(upgrade_badges("No such category", weight(1.0), weight(6.5)).empty());
}

TEST_CASE("badges: the category table names the primary metric", "[upgrade_badges][garage]") {
    const UpgradeCategoryMetrics* weight_page = upgrade_category_metrics("Weight reduction");
    REQUIRE(weight_page != nullptr);
    CHECK(weight_page->primary == UpgradeQuantity::Weight);
    CHECK_FALSE(weight_page->has_secondary);
    CHECK(upgrade_category_metrics("Engine") == nullptr); // no figures in the model: no badges
}

TEST_CASE("badges: option_part_badges compares the parts of one option against the installed one",
          "[upgrade_badges][garage]") {
    OptionView view;
    view.def.group = "Weight reduction";
    view.parts = {part("stock", 6.5), part("sport", 12.0), part("removed", 0.0), part("light", 6.5)};

    auto to_sport = option_part_badges(view, "stock", "sport");
    REQUIRE(to_sport.size() == 1);
    CHECK(to_sport[0].text == "+5.5 kg");

    auto to_removed = option_part_badges(view, "stock", "removed");
    REQUIRE(to_removed.size() == 1);
    CHECK(to_removed[0].text == "-6.5 kg");

    CHECK(option_part_badges(view, "stock", "stock").empty());
    CHECK(option_part_badges(view, "stock", "light").empty()); // same mass: no badge
    CHECK(option_part_badges(view, "stock", "unknown").empty());

    view.def.group = "Engine";
    CHECK(option_part_badges(view, "stock", "sport").empty()); // a category without figures
}
