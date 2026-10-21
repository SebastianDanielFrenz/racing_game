// The badge rules of the garage Upgrades page. See rg/upgrade_badges.h. The tables are data/garage/upgrade_badges.json.
#include "rg/upgrade_badges.h"

#include "json_util.h"
#include "rg/number_format.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <initializer_list>

namespace rg {

namespace {

struct QuantityName {
    UpgradeQuantity quantity;
    const char* name;
};
constexpr QuantityName kNames[] = {
    {UpgradeQuantity::Weight, "weight"},
    {UpgradeQuantity::Power, "power"},
    {UpgradeQuantity::Torque, "torque"},
    {UpgradeQuantity::Downforce, "downforce"},
    {UpgradeQuantity::BrakingTorque, "braking_torque"},
    {UpgradeQuantity::BatteryKwh, "battery_kwh"},
    {UpgradeQuantity::GripLongitudinal, "grip_longitudinal"},
    {UpgradeQuantity::GripLateral, "grip_lateral"},
    {UpgradeQuantity::RollingResistance, "rolling_resistance"},
    {UpgradeQuantity::RpmTolerance, "rpm_tolerance"},
    {UpgradeQuantity::CoolantTemperature, "coolant_temperature"},
    {UpgradeQuantity::OilTemperature, "oil_temperature"},
};

void set_err(std::string* err, const std::string& msg) {
    if (err != nullptr) *err = msg;
}

// The shown change of one quantity; false when it cannot be formed (a relative change against zero).
bool shown_delta(const UpgradeQuantityRule& rule, double candidate, double installed, double& out) {
    if (!std::isfinite(candidate) || !std::isfinite(installed)) return false;
    if (rule.relative) {
        if (!(std::fabs(installed) > 0.0)) return false;
        out = (candidate - installed) / std::fabs(installed) * 100.0;
    } else {
        out = candidate - installed;
    }
    return true;
}

// The badge of one quantity, or nothing when either part lacks the figure or the change is too small to show.
bool make_badge(const UpgradeBadgeConfig& cfg, UpgradeQuantity q, const PartFigures& candidate, const PartFigures& installed,
                UpgradeBadge& out) {
    const UpgradeQuantityRule* rule = upgrade_quantity_rule(cfg, q);
    const auto c = candidate.find(q);
    const auto i = installed.find(q);
    if (rule == nullptr || c == candidate.end() || i == installed.end()) return false;
    double delta = 0.0;
    if (!shown_delta(*rule, c->second, i->second, delta)) return false;
    // A change that rounds to nothing in the printed number is no change either.
    if (std::fabs(delta) < rule->min_change || format_number(delta, rule->max_decimals) == "0") return false;
    out.quantity = q;
    out.delta = delta;
    out.pending = false;
    // A rise is worse for lower-is-better quantities (weight, temperatures, rolling resistance); for higher-is-better ones a
    // fall is worse. The same rule serves every current and future metric: better = green, worse = red.
    out.worse = rule->higher_is_better ? delta < 0.0 : delta > 0.0;
    out.text = upgrade_badge_text(*rule, delta);
    return true;
}

bool object_with_keys(const std::string& where, const detail::json& o, std::initializer_list<const char*> allowed, std::string* err) {
    if (!o.is_object()) {
        set_err(err, where + " must be an object");
        return false;
    }
    for (auto k = o.begin(); k != o.end(); ++k) {
        bool known = false;
        for (const char* a : allowed) known = known || k.key() == a;
        if (!known) {
            set_err(err, where + ": unknown key \"" + k.key() + "\"");
            return false;
        }
    }
    return true;
}

bool read_quantity_rule(const std::string& path, const std::string& key, const detail::json& body, UpgradeQuantityRule& rule,
                        std::string* err) {
    const std::string where = path + ": quantity \"" + key + "\"";
    if (!body.is_object()) {
        set_err(err, where + " must be an object");
        return false;
    }
    if (body.contains("decimals")) {
        set_err(err, where + ": \"decimals\" was renamed \"max_decimals\" in rg.upgrade_badges/2 (it is now only an upper limit)");
        return false;
    }
    if (!object_with_keys(where, body, {"unit", "higher_is_better", "max_decimals", "min_change", "relative", "source"}, err)) return false;
    if (!detail::get_string(body, "unit", rule.unit)) {
        set_err(err, where + " needs a \"unit\" string");
        return false;
    }
    if (!detail::get_bool(body, "higher_is_better", rule.higher_is_better)) {
        // There is no default direction: the colour of a badge must never be a guess.
        set_err(err, where + " must declare \"higher_is_better\" (true or false); there is no default direction");
        return false;
    }
    double max_decimals = 0.0;
    if (!detail::get_number(body, "max_decimals", max_decimals) || !detail::get_number(body, "min_change", rule.min_change)) {
        set_err(err, where + " needs \"max_decimals\" and \"min_change\" numbers");
        return false;
    }
    rule.max_decimals = static_cast<int>(max_decimals);
    if (max_decimals < 0.0 || max_decimals > 6.0 || max_decimals != static_cast<double>(rule.max_decimals) || !(rule.min_change >= 0.0)) {
        set_err(err, where + ": max_decimals must be an integer 0..6 and min_change >= 0");
        return false;
    }
    if (body.contains("relative") && !detail::get_bool(body, "relative", rule.relative)) {
        set_err(err, where + ": \"relative\" must be true or false");
        return false;
    }
    return true;
}

bool read_references(const std::string& path, const detail::json& doc, UpgradeReferences& r, std::string* err) {
    const auto refs = doc.find("references");
    if (refs == doc.end()) {
        set_err(err, path + ": \"references\" must be an object");
        return false;
    }
    if (!object_with_keys(path + ": \"references\"", *refs, {"source", "tyres", "dyno", "thermal"}, err)) return false;
    const auto tyres = refs->find("tyres");
    const auto dyno = refs->find("dyno");
    const auto thermal = refs->find("thermal");
    if (tyres == refs->end() || dyno == refs->end() || thermal == refs->end()) {
        set_err(err, path + ": \"references\" needs \"tyres\", \"dyno\" and \"thermal\"");
        return false;
    }
    if (!object_with_keys(path + ": references.tyres", *tyres, {"source", "surface_table", "surface", "rolling_resistance_speed_kmh"}, err) ||
        !object_with_keys(path + ": references.dyno", *dyno, {"source", "quality", "rpm_step", "rpm_min", "limiter_margin_rpm", "debounce_ms"}, err) ||
        !object_with_keys(path + ": references.thermal", *thermal, {"source", "ambient_c", "air_speed_kmh", "friction_power_fraction"}, err)) {
        return false;
    }
    std::string table;
    if (!detail::get_string(*tyres, "surface_table", table) || table.empty() || !detail::get_string(*tyres, "surface", r.surface) ||
        r.surface.empty() || !detail::get_number(*tyres, "rolling_resistance_speed_kmh", r.rolling_resistance_speed_kmh) ||
        !(r.rolling_resistance_speed_kmh > 0.0) || r.rolling_resistance_speed_kmh > 1000.0) {
        set_err(err, path + ": references.tyres needs surface_table, surface and rolling_resistance_speed_kmh (> 0)");
        return false;
    }
    namespace fs = std::filesystem;
    const fs::path p(table);
    r.surface_table = (p.is_absolute() ? p : fs::path(path).parent_path() / p).lexically_normal().generic_string();
    if (!detail::get_string(*dyno, "quality", r.dyno_quality) ||
        (r.dyno_quality != "sketch" && r.dyno_quality != "preview" && r.dyno_quality != "full")) {
        set_err(err, path + ": references.dyno.quality must be \"sketch\", \"preview\" or \"full\"");
        return false;
    }
    if (!detail::get_number(*dyno, "rpm_step", r.dyno_rpm_step) || !detail::get_number(*dyno, "rpm_min", r.dyno_rpm_min) ||
        !detail::get_number(*dyno, "limiter_margin_rpm", r.dyno_limiter_margin_rpm) || !(r.dyno_rpm_step >= 10.0) ||
        !(r.dyno_rpm_min > 0.0) || !(r.dyno_limiter_margin_rpm >= 0.0)) {
        set_err(err, path + ": references.dyno needs rpm_step (>= 10), rpm_min (> 0) and limiter_margin_rpm (>= 0)");
        return false;
    }
    double debounce = 0.0;
    if (!detail::get_number(*dyno, "debounce_ms", debounce) || debounce < 0.0 || debounce > 5000.0) {
        set_err(err, path + ": references.dyno.debounce_ms must be 0..5000");
        return false;
    }
    r.dyno_debounce_ms = static_cast<int>(debounce);
    if (!detail::get_number(*thermal, "ambient_c", r.thermal_ambient_c) || !detail::get_number(*thermal, "air_speed_kmh", r.thermal_air_speed_kmh) ||
        !detail::get_number(*thermal, "friction_power_fraction", r.thermal_friction_power_fraction) ||
        !(r.thermal_air_speed_kmh >= 0.0) || !(r.thermal_ambient_c > -60.0 && r.thermal_ambient_c < 70.0) ||
        !(r.thermal_friction_power_fraction >= 0.0 && r.thermal_friction_power_fraction < 1.0)) {
        set_err(err, path + ": references.thermal needs ambient_c (-60..70), air_speed_kmh (>= 0) and friction_power_fraction (0..1)");
        return false;
    }
    return true;
}

} // namespace

const char* upgrade_quantity_name(UpgradeQuantity q) {
    for (const QuantityName& n : kNames) {
        if (n.quantity == q) return n.name;
    }
    return "?";
}

bool upgrade_quantity_from_name(const std::string& name, UpgradeQuantity& out) {
    for (const QuantityName& n : kNames) {
        if (name == n.name) {
            out = n.quantity;
            return true;
        }
    }
    return false;
}

const std::vector<UpgradeQuantity>& all_upgrade_quantities() {
    static const std::vector<UpgradeQuantity> all = [] {
        std::vector<UpgradeQuantity> v;
        for (const QuantityName& n : kNames) v.push_back(n.quantity);
        return v;
    }();
    return all;
}

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
    detail::get_string(*doc, "format", format);
    if (format == "rg.upgrade_badges/1") {
        set_err(err, path + ": rg.upgrade_badges/1 is no longer supported (the schema changed: \"decimals\" became \"max_decimals\", "
                           "categories list \"metrics\" instead of \"primary\"/\"secondary\", and \"references\" is required); use rg.upgrade_badges/2");
        return std::nullopt;
    }
    if (format != "rg.upgrade_badges/2") {
        set_err(err, path + ": \"format\" must be \"rg.upgrade_badges/2\"");
        return std::nullopt;
    }
    if (!object_with_keys(path, *doc, {"format", "source", "max_badges", "references", "quantities", "categories"}, err)) return std::nullopt;
    UpgradeBadgeConfig cfg;
    double max_badges = 0.0;
    if (!detail::get_number(*doc, "max_badges", max_badges) || max_badges < 1.0 || max_badges > 6.0 ||
        max_badges != std::floor(max_badges)) {
        set_err(err, path + ": \"max_badges\" must be an integer 1..6");
        return std::nullopt;
    }
    cfg.max_badges = static_cast<int>(max_badges);
    if (!read_references(path, *doc, cfg.references, err)) return std::nullopt;

    const auto quantities = doc->find("quantities");
    if (quantities == doc->end() || !quantities->is_object()) {
        set_err(err, path + ": \"quantities\" must be an object");
        return std::nullopt;
    }
    for (auto it = quantities->begin(); it != quantities->end(); ++it) {
        UpgradeQuantityRule rule;
        if (!upgrade_quantity_from_name(it.key(), rule.quantity)) {
            set_err(err, path + ": unknown quantity \"" + it.key() + "\"");
            return std::nullopt;
        }
        if (!read_quantity_rule(path, it.key(), *it, rule, err)) return std::nullopt;
        cfg.quantities[rule.quantity] = rule;
    }
    const auto categories = doc->find("categories");
    if (categories == doc->end() || !categories->is_object()) {
        set_err(err, path + ": \"categories\" must be an object");
        return std::nullopt;
    }
    for (auto it = categories->begin(); it != categories->end(); ++it) {
        const std::string where = path + ": category \"" + it.key() + "\"";
        if (!it->is_object()) {
            set_err(err, where + " must be an object");
            return std::nullopt;
        }
        if (it->contains("primary") || it->contains("secondary")) {
            set_err(err, where + ": \"primary\"/\"secondary\" were replaced by the ordered list \"metrics\" in rg.upgrade_badges/2");
            return std::nullopt;
        }
        if (!object_with_keys(where, *it, {"metrics", "nitrous_armed", "source"}, err)) return std::nullopt;
        UpgradeCategoryMetrics m;
        const auto list = it->find("metrics");
        if (list == it->end() || !list->is_array() || list->empty()) {
            set_err(err, where + ": \"metrics\" must be a non-empty array of quantity names");
            return std::nullopt;
        }
        for (const auto& name : *list) {
            UpgradeQuantity q = UpgradeQuantity::Weight;
            if (!name.is_string() || !upgrade_quantity_from_name(name.get<std::string>(), q) || cfg.quantities.count(q) == 0) {
                set_err(err, where + ": every metric must name a listed quantity");
                return std::nullopt;
            }
            if (std::find(m.metrics.begin(), m.metrics.end(), q) != m.metrics.end()) {
                set_err(err, where + ": metric \"" + name.get<std::string>() + "\" is listed twice");
                return std::nullopt;
            }
            m.metrics.push_back(q);
        }
        if (it->contains("nitrous_armed") && !detail::get_bool(*it, "nitrous_armed", m.nitrous_armed)) {
            set_err(err, where + ": \"nitrous_armed\" must be true or false");
            return std::nullopt;
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

const UpgradeCategoryMetrics* upgrade_category_for(const UpgradeBadgeConfig& cfg, const std::string& option_id,
                                                   const std::string& group, std::string* key_out) {
    for (const std::string* key : {&option_id, &group}) {
        if (const UpgradeCategoryMetrics* m = upgrade_category_metrics(cfg, *key)) {
            if (key_out != nullptr) *key_out = *key;
            return m;
        }
    }
    return nullptr;
}

std::string upgrade_badge_text(const UpgradeQuantityRule& rule, double delta) {
    std::string out = format_signed_number(delta, rule.max_decimals);
    if (!rule.unit.empty()) out += " " + rule.unit;
    return out;
}

std::vector<UpgradeBadge> upgrade_badges(const UpgradeBadgeConfig& cfg, const std::string& category, const PartFigures& candidate,
                                         const PartFigures& installed, const std::set<UpgradeQuantity>* pending) {
    std::vector<UpgradeBadge> out;
    const UpgradeCategoryMetrics* metrics = upgrade_category_metrics(cfg, category);
    if (metrics == nullptr) return out;
    for (UpgradeQuantity q : metrics->metrics) {
        if (static_cast<int>(out.size()) >= cfg.max_badges) break;
        UpgradeBadge badge;
        if (pending != nullptr && pending->count(q) != 0) {
            badge.quantity = q;
            badge.pending = true;
            badge.text = kPendingBadgeText;
            out.push_back(badge);
        } else if (make_badge(cfg, q, candidate, installed, badge)) {
            out.push_back(badge);
        }
    }
    return out;
}

std::vector<UpgradeBadge> option_part_badges(const UpgradeBadgeConfig& cfg, const OptionView& view, const std::string& installed_id,
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
    return upgrade_badges(cfg, view.def.group, cand, inst);
}

} // namespace rg
