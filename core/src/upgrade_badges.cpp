// The badge rules of the garage Upgrades page. See rg/upgrade_badges.h. The tables are data/garage/upgrade_badges.json.
#include "rg/upgrade_badges.h"

#include "json_util.h"

#include <cmath>
#include <cstdio>

namespace rg {

namespace {

struct QuantityName {
    UpgradeQuantity quantity;
    const char* name;
};
constexpr QuantityName kNames[] = {
    {UpgradeQuantity::Weight, "weight"},       {UpgradeQuantity::Power, "power"},
    {UpgradeQuantity::Torque, "torque"},       {UpgradeQuantity::Grip, "grip"},
    {UpgradeQuantity::Downforce, "downforce"}, {UpgradeQuantity::BrakingTorque, "braking_torque"},
    {UpgradeQuantity::BatteryKwh, "battery_kwh"},
};

bool quantity_from_name(const std::string& name, UpgradeQuantity& out) {
    for (const QuantityName& n : kNames) {
        if (name == n.name) {
            out = n.quantity;
            return true;
        }
    }
    return false;
}

void set_err(std::string* err, const std::string& msg) {
    if (err != nullptr) *err = msg;
}

// The text of one signed change: "+5.5 kg" / "-6.5 kg".
std::string badge_text(const UpgradeQuantityRule& rule, double delta) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%+.*f %s", rule.decimals, delta, rule.unit.c_str());
    return buf;
}

// The badge of one quantity, or nothing when either part lacks the figure or the change is too small to show.
bool make_badge(const UpgradeBadgeConfig& cfg, UpgradeQuantity q, const PartFigures& candidate,
                const PartFigures& installed, UpgradeBadge& out) {
    const UpgradeQuantityRule* rule = upgrade_quantity_rule(cfg, q);
    const auto c = candidate.find(q);
    const auto i = installed.find(q);
    if (rule == nullptr || c == candidate.end() || i == installed.end()) return false;
    const double delta = c->second - i->second;
    if (std::fabs(delta) < rule->min_change) return false;
    out.quantity = q;
    out.delta = delta;
    // A rise is worse for lower-is-better quantities (weight), and for higher-is-better ones a fall is worse.
    out.worse = rule->higher_is_better ? delta < 0.0 : delta > 0.0;
    out.text = badge_text(*rule, delta);
    return true;
}

} // namespace

std::optional<UpgradeBadgeConfig> load_upgrade_badge_config(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        set_err(err, path + ": cannot read the file");
        return std::nullopt;
    }
    const auto doc = detail::parse_json(*text);
    if (!doc || !doc->is_object()) {
        set_err(err, path + ": not a JSON object");
        return std::nullopt;
    }
    std::string format;
    if (!detail::get_string(*doc, "format", format) || format != "rg.upgrade_badges/1") {
        set_err(err, path + ": \"format\" must be \"rg.upgrade_badges/1\"");
        return std::nullopt;
    }
    UpgradeBadgeConfig cfg;
    const auto quantities = doc->find("quantities");
    if (quantities == doc->end() || !quantities->is_object()) {
        set_err(err, path + ": \"quantities\" must be an object");
        return std::nullopt;
    }
    for (auto it = quantities->begin(); it != quantities->end(); ++it) {
        UpgradeQuantityRule rule;
        if (!quantity_from_name(it.key(), rule.quantity)) {
            set_err(err, path + ": unknown quantity \"" + it.key() + "\"");
            return std::nullopt;
        }
        double decimals = 0.0;
        if (!it->is_object() || !detail::get_string(*it, "unit", rule.unit) ||
            !detail::get_bool(*it, "higher_is_better", rule.higher_is_better) ||
            !detail::get_number(*it, "decimals", decimals) || !detail::get_number(*it, "min_change", rule.min_change)) {
            set_err(err, path + ": quantity \"" + it.key() + "\" needs unit, higher_is_better, decimals and min_change");
            return std::nullopt;
        }
        rule.decimals = static_cast<int>(decimals);
        if (decimals < 0.0 || decimals > 6.0 || decimals != static_cast<double>(rule.decimals) || !(rule.min_change >= 0.0)) {
            set_err(err, path + ": quantity \"" + it.key() + "\": decimals must be an integer 0..6 and min_change >= 0");
            return std::nullopt;
        }
        cfg.quantities[rule.quantity] = rule;
    }
    const auto categories = doc->find("categories");
    if (categories == doc->end() || !categories->is_object()) {
        set_err(err, path + ": \"categories\" must be an object");
        return std::nullopt;
    }
    for (auto it = categories->begin(); it != categories->end(); ++it) {
        UpgradeCategoryMetrics m;
        std::string primary;
        std::string secondary;
        if (!it->is_object() || !detail::get_string(*it, "primary", primary) || !quantity_from_name(primary, m.primary) ||
            cfg.quantities.count(m.primary) == 0) {
            set_err(err, path + ": category \"" + it.key() + "\": \"primary\" must name a listed quantity");
            return std::nullopt;
        }
        if (it->contains("secondary")) {
            if (!detail::get_string(*it, "secondary", secondary) || !quantity_from_name(secondary, m.secondary) ||
                cfg.quantities.count(m.secondary) == 0 || m.secondary == m.primary) {
                set_err(err, path + ": category \"" + it.key() + "\": \"secondary\" must name another listed quantity");
                return std::nullopt;
            }
            m.has_secondary = true;
        }
        for (auto k = it->begin(); k != it->end(); ++k) {
            if (k.key() != "primary" && k.key() != "secondary") {
                set_err(err, path + ": category \"" + it.key() + "\": unknown key \"" + k.key() + "\"");
                return std::nullopt;
            }
        }
        cfg.categories[it.key()] = m;
    }
    return cfg;
}

const UpgradeQuantityRule* upgrade_quantity_rule(const UpgradeBadgeConfig& cfg, UpgradeQuantity q) {
    const auto it = cfg.quantities.find(q);
    return it == cfg.quantities.end() ? nullptr : &it->second;
}

const UpgradeCategoryMetrics* upgrade_category_metrics(const UpgradeBadgeConfig& cfg, const std::string& category) {
    const auto it = cfg.categories.find(category);
    return it == cfg.categories.end() ? nullptr : &it->second;
}

std::vector<UpgradeBadge> upgrade_badges(const UpgradeBadgeConfig& cfg, const std::string& category,
                                         const PartFigures& candidate, const PartFigures& installed) {
    std::vector<UpgradeBadge> out;
    const UpgradeCategoryMetrics* metrics = upgrade_category_metrics(cfg, category);
    if (metrics == nullptr) return out;
    UpgradeBadge badge;
    if (make_badge(cfg, metrics->primary, candidate, installed, badge)) out.push_back(badge);
    if (metrics->has_secondary && make_badge(cfg, metrics->secondary, candidate, installed, badge)) out.push_back(badge);
    return out;
}

std::vector<UpgradeBadge> option_part_badges(const UpgradeBadgeConfig& cfg, const OptionView& view,
                                             const std::string& installed_id, const std::string& candidate_id) {
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
    return upgrade_badges(cfg, view.def.group, cand, inst);
}

} // namespace rg
