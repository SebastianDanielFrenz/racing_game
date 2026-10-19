// Upgrade badges (garage Upgrades page, owner 2026-10-18/19): the numbers a highlighted part tile shows instead of prose.
//
// A part's badge is a HINT: the change of one number when the part is swapped into the car AS IT IS CONFIGURED NOW, i.e.
// (the whole car with the candidate installed) minus (the whole car as currently configured). Never against stock and never
// part-against-part in isolation (rg/upgrade_hints.h computes the whole-car figures). Totals such as lap times or top speed
// are not hints: they belong in the left pane that shows the current configuration's simulation results.
//
// Each upgrade category names an ordered list of metrics (at most `max_badges` shown, each only when it changes by at least
// its `min_change`). Which direction is better per quantity, the unit, the precision cap, the "meaningful" threshold and the
// reference conditions (surface, speeds, ambient, dyno settings) are ONE data file (data/garage/upgrade_badges.json,
// rg.upgrade_badges/2), unit-tested. The GDScript only draws what this returns.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "rg/vehicle_setup.h"

namespace rg {

// What an upgrade can change. Append-only; value 3 was the /1 "grip" percentage (replaced by the two g figures below).
enum class UpgradeQuantity : std::uint8_t {
    Weight = 0,              // kg, lower is better
    Power = 1,               // kW, higher is better
    Torque = 2,              // N*m, higher is better
    Downforce = 4,           // N at a reference speed, higher is better
    BrakingTorque = 5,       // N*m, higher is better
    BatteryKwh = 6,          // kWh, higher is better
    GripLongitudinal = 7,    // peak Fx of the car's tyres at the static corner loads / (m g), shown as an offset in g
    GripLateral = 8,         // peak Fy likewise
    RollingResistance = 9,   // rolling resistance force at the reference speed / (m g); shown as a RELATIVE change in %
    RpmTolerance = 10,       // the rpm the weakest engine internal allows, rpm
    CoolantTemperature = 11, // steady-state coolant temperature at sustained full power, shown as a difference in K
    OilTemperature = 12,     // steady-state oil temperature likewise
};

// One row of the quantity table (data/garage/upgrade_badges.json "quantities"). Every field is required: there is no default
// direction, a quantity without `higher_is_better` is refused by the loader.
struct UpgradeQuantityRule {
    UpgradeQuantity quantity = UpgradeQuantity::Weight;
    std::string unit;          // printed after the number
    bool higher_is_better = true;
    int max_decimals = 1;      // the MOST digits after the point; the printed count follows the magnitude (rg/number_format.h)
    double min_change = 0.0;   // a change smaller than this (in the SHOWN unit) is "no change": no badge
    bool relative = false;     // true: the shown change is (candidate - installed) / installed in percent
};

// The per-category choice: the metrics in display order.
struct UpgradeCategoryMetrics {
    std::vector<UpgradeQuantity> metrics;
    // Power figures of this category are measured with the car's N2O kit armed (the kit's gain is the hint); every other
    // category measures the engine without nitrous.
    bool nitrous_armed = false;
};

// The reference conditions every hint figure is measured under. All of them are data (upgrade_badges.json "references").
struct UpgradeReferences {
    // Tyres: the static grip figures are measured on this surface (a name of the surface table), dry.
    std::string surface_table;            // absolute once loaded (the file's path is relative to the badge file)
    std::string surface = "asphalt";
    double rolling_resistance_speed_kmh = 100.0;
    // Engine sweeps (power, torque, temperatures): the map quality and the rpm grid.
    std::string dyno_quality = "sketch";  // sketch | preview | full
    double dyno_rpm_step = 250.0;
    double dyno_rpm_min = 2000.0;
    double dyno_limiter_margin_rpm = 50.0;
    int dyno_debounce_ms = 200;           // a hover must rest this long before a dyno starts (a flick over the tiles starts none)
    // Steady-state temperatures at full power: the day and the cooling air speed.
    double thermal_ambient_c = 30.0;
    double thermal_air_speed_kmh = 100.0;
    double thermal_friction_power_fraction = 0.09; // friction heat as a share of the shaft power (the setup check's figure)
};

// data/garage/upgrade_badges.json (rg.upgrade_badges/2). Everything the badge rules decide lives in that file.
struct UpgradeBadgeConfig {
    std::map<UpgradeQuantity, UpgradeQuantityRule> quantities;
    // Keyed by an option id ("engine_install") or an option group ("Weight reduction"); the option id wins (see
    // upgrade_category_for).
    std::map<std::string, UpgradeCategoryMetrics> categories;
    int max_badges = 3;
    UpgradeReferences references;
};

// Loads and validates the file. Refused (std::nullopt with `err` set): the old /1 format (named as such), an unknown or
// duplicated quantity name, a quantity without unit / higher_is_better / max_decimals / min_change, a category naming an
// unlisted quantity, an empty or duplicated metric list, negative or fractional decimals, an unknown key.
std::optional<UpgradeBadgeConfig> load_upgrade_badge_config(const std::string& path, std::string* err);

// The quantity names used in the file ("weight", "grip_longitudinal" ...).
const char* upgrade_quantity_name(UpgradeQuantity q);
bool upgrade_quantity_from_name(const std::string& name, UpgradeQuantity& out);
// Every quantity the format knows, in enum order.
const std::vector<UpgradeQuantity>& all_upgrade_quantities();

// The figures one configuration (or one part) carries. A missing quantity means the garage model has no number for it.
using PartFigures = std::map<UpgradeQuantity, double>;

// One badge on a highlighted tile.
struct UpgradeBadge {
    UpgradeQuantity quantity = UpgradeQuantity::Weight;
    double delta = 0.0;   // candidate minus installed in the SHOWN unit (percent for a relative quantity)
    bool worse = false;   // red when true, green when false
    bool pending = false; // the figure is still being computed: `text` is the placeholder, `worse`/`delta` mean nothing
    std::string text;     // "+5.5 kg", "-6.5 kg", "+4.2 %", "+0.05 g"; the placeholder while pending
};

// nullptr when the quantity / category is not in the config (its tiles get no badges).
const UpgradeQuantityRule* upgrade_quantity_rule(const UpgradeBadgeConfig& cfg, UpgradeQuantity q);
const UpgradeCategoryMetrics* upgrade_category_metrics(const UpgradeBadgeConfig& cfg, const std::string& category);
// The category of an option: the entry named by the option id, else the one named by its group; nullptr for none.
const UpgradeCategoryMetrics* upgrade_category_for(const UpgradeBadgeConfig& cfg, const std::string& option_id,
                                                   const std::string& group, std::string* key_out = nullptr);

// The text of one signed change in a quantity's unit: "+5.5 kg" (rg/number_format.h rules).
std::string upgrade_badge_text(const UpgradeQuantityRule& rule, double delta);

// The placeholder shown while a figure is computed.
inline constexpr const char* kPendingBadgeText = "...";

// The badges for `candidate` against `installed`, in the category's metric order, at most `max_badges`. A metric appears
// when both figures exist and the change reaches its min_change; a metric in `pending` (when given) appears as a placeholder
// instead (its figures are being computed). Empty when the candidate is the installed part or nothing changes.
std::vector<UpgradeBadge> upgrade_badges(const UpgradeBadgeConfig& cfg, const std::string& category,
                                         const PartFigures& candidate, const PartFigures& installed,
                                         const std::set<UpgradeQuantity>* pending = nullptr);

// The badges of one option part (a SetupOptionDef's part) against the installed part of that option, from the parts' own
// mass (SetupPart::mass_kg). A cheap part-to-part fallback for options with a mass figure only; the garage's hints use the
// whole-car figures of rg/upgrade_hints.h instead.
std::vector<UpgradeBadge> option_part_badges(const UpgradeBadgeConfig& cfg, const OptionView& view,
                                             const std::string& installed_id, const std::string& candidate_id);

} // namespace rg
