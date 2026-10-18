// Upgrade badges (garage Upgrades page, owner 2026-10-18): the numbers a highlighted part tile shows instead of prose.
//
// A part's badge is its change against the INSTALLED part of the same option: "+5.5 kg" when it is heavier, "-6.5 kg"
// when it is lighter. Each upgrade category names one primary quantity and at most one secondary one that is shown only
// when it changes meaningfully. Which direction is better per quantity, the units, the precision and the "meaningful"
// threshold are one data file (data/garage/upgrade_badges.json), unit-tested. The GDScript only draws what this returns.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
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

// One row of the quantity table (data/garage/upgrade_badges.json "quantities").
struct UpgradeQuantityRule {
    UpgradeQuantity quantity = UpgradeQuantity::Weight;
    std::string unit;          // printed after the number
    bool higher_is_better = true;
    int decimals = 1;          // digits after the point in a badge
    double min_change = 0.0;   // a change smaller than this is "no change" (no badge, and never a secondary)
};

// The per-category choice: one primary quantity, and optionally one secondary one.
struct UpgradeCategoryMetrics {
    UpgradeQuantity primary = UpgradeQuantity::Weight;
    bool has_secondary = false;
    UpgradeQuantity secondary = UpgradeQuantity::Weight;
};

// data/garage/upgrade_badges.json (rg.upgrade_badges/1). Everything the badge rules decide lives in that file.
struct UpgradeBadgeConfig {
    std::map<UpgradeQuantity, UpgradeQuantityRule> quantities;
    std::map<std::string, UpgradeCategoryMetrics> categories; // keyed by the option group ("Weight reduction")
};

// Loads and validates the file (unknown quantity names, a category naming an unlisted quantity, primary == secondary,
// negative decimals/min_change are refused). std::nullopt with `err` set on failure.
std::optional<UpgradeBadgeConfig> load_upgrade_badge_config(const std::string& path, std::string* err);

// The figures one part carries. A missing quantity means the garage model has no number for it.
using PartFigures = std::map<UpgradeQuantity, double>;

// One badge on a highlighted tile.
struct UpgradeBadge {
    UpgradeQuantity quantity = UpgradeQuantity::Weight;
    double delta = 0.0;   // candidate minus installed, in the quantity's unit
    bool worse = false;   // red when true, green when false
    std::string text;     // "+5.5 kg", "-6.5 kg"
};

// nullptr when the quantity / category is not in the config (its tiles get no badges).
const UpgradeQuantityRule* upgrade_quantity_rule(const UpgradeBadgeConfig& cfg, UpgradeQuantity q);
const UpgradeCategoryMetrics* upgrade_category_metrics(const UpgradeBadgeConfig& cfg, const std::string& category);

// The badges for `candidate` against `installed`: the primary quantity first, then the secondary one when it changes by
// at least its min_change. At most two. Empty when the candidate is the installed part or nothing changes.
std::vector<UpgradeBadge> upgrade_badges(const UpgradeBadgeConfig& cfg, const std::string& category,
                                         const PartFigures& candidate, const PartFigures& installed);

// The badges of one option part (a SetupOptionDef's part) against the installed part of that option. The figures come from
// the option's own parts (SetupPart::mass_kg today); a category without figures returns nothing.
std::vector<UpgradeBadge> option_part_badges(const UpgradeBadgeConfig& cfg, const OptionView& view,
                                             const std::string& installed_id, const std::string& candidate_id);

} // namespace rg
