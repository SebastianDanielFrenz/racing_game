// The quantity table and the badge rules of the garage Upgrades page. See rg/upgrade_badges.h.
#include "rg/upgrade_badges.h"

#include <cmath>
#include <cstdio>

namespace rg {

namespace {

// One row per quantity. min_change is the smallest change that counts ("no change" below it): 0.5 kg is the garage's
// own existing rule for the mass totals.
constexpr UpgradeQuantityRule kQuantities[] = {
    {UpgradeQuantity::Weight, "kg", false, 1, 0.5},
    {UpgradeQuantity::Power, "kW", true, 0, 1.0},
    {UpgradeQuantity::Torque, "N*m", true, 0, 1.0},
    {UpgradeQuantity::Grip, "%", true, 1, 0.5},
    {UpgradeQuantity::Downforce, "N", true, 0, 5.0},
    {UpgradeQuantity::BrakingTorque, "N*m", true, 0, 5.0},
    {UpgradeQuantity::BatteryKwh, "kWh", true, 1, 0.1},
};

// One row per category that has numbers. A category missing here gets no badges. Only "Weight reduction" has figures
// in the garage model today (SetupPart::mass_kg); the other quantities are listed so a category can name them once the
// model carries the figure.
constexpr UpgradeCategoryMetrics kCategories[] = {
    {"Weight reduction", UpgradeQuantity::Weight, false, UpgradeQuantity::Weight},
};

// The text of one signed change: "+5.5 kg" / "-6.5 kg".
std::string badge_text(const UpgradeQuantityRule& rule, double delta) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%+.*f %s", rule.decimals, delta, rule.unit);
    return buf;
}

// The badge of one quantity, or nothing when either part lacks the figure or the change is too small to show.
bool make_badge(UpgradeQuantity q, const PartFigures& candidate, const PartFigures& installed, UpgradeBadge& out) {
    const auto c = candidate.find(q);
    const auto i = installed.find(q);
    if (c == candidate.end() || i == installed.end()) return false;
    const UpgradeQuantityRule& rule = upgrade_quantity_rule(q);
    const double delta = c->second - i->second;
    if (std::fabs(delta) < rule.min_change) return false;
    out.quantity = q;
    out.delta = delta;
    // A rise is worse for lower-is-better quantities (weight), and for higher-is-better ones a fall is worse.
    out.worse = rule.higher_is_better ? delta < 0.0 : delta > 0.0;
    out.text = badge_text(rule, delta);
    return true;
}

} // namespace

const UpgradeQuantityRule& upgrade_quantity_rule(UpgradeQuantity q) {
    for (const UpgradeQuantityRule& rule : kQuantities) {
        if (rule.quantity == q) return rule;
    }
    return kQuantities[0];
}

const UpgradeCategoryMetrics* upgrade_category_metrics(const std::string& category) {
    for (const UpgradeCategoryMetrics& m : kCategories) {
        if (category == m.category) return &m;
    }
    return nullptr;
}

std::vector<UpgradeBadge> upgrade_badges(const std::string& category, const PartFigures& candidate,
                                         const PartFigures& installed) {
    std::vector<UpgradeBadge> out;
    const UpgradeCategoryMetrics* metrics = upgrade_category_metrics(category);
    if (metrics == nullptr) return out;
    UpgradeBadge badge;
    if (make_badge(metrics->primary, candidate, installed, badge)) out.push_back(badge);
    if (metrics->has_secondary && make_badge(metrics->secondary, candidate, installed, badge)) out.push_back(badge);
    return out;
}

std::vector<UpgradeBadge> option_part_badges(const OptionView& view, const std::string& installed_id,
                                             const std::string& candidate_id) {
    const SetupPart* candidate = nullptr;
    const SetupPart* installed = nullptr;
    for (const SetupPart& p : view.parts) {
        if (p.id == candidate_id) candidate = &p;
        if (p.id == installed_id) installed = &p;
    }
    if (candidate == nullptr || installed == nullptr || candidate->id == installed->id) return {};
    PartFigures cand;
    PartFigures inst;
    if (candidate->mass_kg >= 0.0) cand[UpgradeQuantity::Weight] = candidate->mass_kg;
    if (installed->mass_kg >= 0.0) inst[UpgradeQuantity::Weight] = installed->mass_kg;
    return upgrade_badges(view.def.group, cand, inst);
}

} // namespace rg
