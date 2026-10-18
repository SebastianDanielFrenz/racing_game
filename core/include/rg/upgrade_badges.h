// Upgrade badges (garage Upgrades page, owner 2026-10-18): the numbers a highlighted part tile shows instead of prose.
//
// A part's badge is its change against the INSTALLED part of the same option: "+5.5 kg" when it is heavier, "-6.5 kg"
// when it is lighter. Each upgrade category names one primary quantity and at most one secondary one that is shown only
// when it changes meaningfully. Which direction is better per quantity, the units, the precision and the "meaningful"
// threshold are one table here (upgrade_badges.cpp), unit-tested. The GDScript only draws what this returns.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "rg/vehicle_setup.h"

namespace rg {

// What an upgrade can change. Append-only: the values are not stored anywhere yet, but the table is keyed by them.
enum class UpgradeQuantity : std::uint8_t {
    Weight = 0,        // kg, lower is better
    Power = 1,         // kW, higher is better
    Torque = 2,        // N*m, higher is better
    Grip = 3,          // grip coefficient x100 (a percentage), higher is better
    Downforce = 4,     // N, higher is better
    BrakingTorque = 5, // N*m, higher is better
    BatteryKwh = 6,    // kWh, higher is better
};

// One row of the quantity table.
struct UpgradeQuantityRule {
    UpgradeQuantity quantity;
    const char* unit;   // printed after the number
    bool higher_is_better;
    int decimals;       // digits after the point in a badge
    double min_change;  // a change smaller than this is "no change" (no badge, and never a secondary)
};

// The per-category choice: one primary quantity, and optionally one secondary one.
struct UpgradeCategoryMetrics {
    const char* category; // the option's group ("Weight reduction")
    UpgradeQuantity primary;
    bool has_secondary;
    UpgradeQuantity secondary;
};

// The figures one part carries. A missing quantity means the garage model has no number for it.
using PartFigures = std::map<UpgradeQuantity, double>;

// One badge on a highlighted tile.
struct UpgradeBadge {
    UpgradeQuantity quantity = UpgradeQuantity::Weight;
    double delta = 0.0;   // candidate minus installed, in the quantity's unit
    bool worse = false;   // red when true, green when false
    std::string text;     // "+5.5 kg", "-6.5 kg"
};

const UpgradeQuantityRule& upgrade_quantity_rule(UpgradeQuantity q);
// nullptr when the category has no metric (its tiles get no badges).
const UpgradeCategoryMetrics* upgrade_category_metrics(const std::string& category);

// The badges for `candidate` against `installed`: the primary quantity first, then the secondary one when it changes by
// at least its min_change. At most two. Empty when the candidate is the installed part or nothing changes.
std::vector<UpgradeBadge> upgrade_badges(const std::string& category, const PartFigures& candidate,
                                         const PartFigures& installed);

// The badges of one option part (a SetupOptionDef's part) against the installed part of that option. The figures come from
// the option's own parts (SetupPart::mass_kg today); a category without figures returns nothing.
std::vector<UpgradeBadge> option_part_badges(const OptionView& view, const std::string& installed_id,
                                             const std::string& candidate_id);

} // namespace rg
