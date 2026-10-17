// rg/damage.h implementation.
#include "rg/damage.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <system_error>

namespace rg {

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

bool fail(std::string* err, const std::string& origin, const std::string& msg) {
    if (err != nullptr) *err = origin + ": " + msg;
    return false;
}

bool is_snake(const std::string& s) {
    return !s.empty() && std::all_of(s.begin(), s.end(), [](char c) { return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'; });
}

bool known_keys(const json& o, const std::set<std::string>& allowed, const std::string& where, const std::string& origin, std::string* err) {
    for (auto it = o.begin(); it != o.end(); ++it) {
        if (allowed.count(it.key()) == 0) return fail(err, origin, where + ": unknown key \"" + it.key() + "\"");
    }
    return true;
}

bool number_in(const json& o, const char* key, double lo, double hi, double& out, const std::string& where, const std::string& origin,
               std::string* err, bool required = true) {
    const auto it = o.find(key);
    if (it == o.end()) return required ? fail(err, origin, where + ": \"" + key + "\" is required") : true;
    if (!it->is_number()) return fail(err, origin, where + ": \"" + key + "\" must be a number");
    const double v = it->get<double>();
    if (!std::isfinite(v) || v < lo || v > hi) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), ": \"%s\" must be in [%g, %g]", key, lo, hi);
        return fail(err, origin, where + buf);
    }
    out = v;
    return true;
}

double lerp_scale(double scale_at_zero, double health) { return scale_at_zero + (1.0 - scale_at_zero) * health; }

} // namespace

const char* to_string(DamageEffect effect) {
    switch (effect) {
        case DamageEffect::None: return "none";
        case DamageEffect::RadiatorUa: return "radiator_ua";
        case DamageEffect::FanFlow: return "fan_flow";
        case DamageEffect::Friction: return "friction";
    }
    return "none";
}

const DamagePartConfig* DamageConfig::find(const std::string& id) const {
    for (const DamagePartConfig& p : parts)
        if (p.id == id) return &p;
    return nullptr;
}

std::optional<DamageConfig> parse_damage_config(const std::string& text, const std::string& origin, std::string* err) {
    const std::optional<json> doc = detail::parse_json(text);
    if (!doc || !doc->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    const json& j = *doc;
    if (!known_keys(j, {"format", "source", "report_below", "save_interval_s", "parts", "rules"}, "top level", origin, err)) return std::nullopt;
    std::string format;
    if (!detail::get_string(j, "format", format) || format != "rg.damage/2") {
        fail(err, origin, "\"format\" must be \"rg.damage/2\"");
        return std::nullopt;
    }
    DamageConfig c;
    if (!number_in(j, "report_below", 0.0, 1.0, c.report_below, "top level", origin, err, false)) return std::nullopt;
    if (!number_in(j, "save_interval_s", 0.1, 3600.0, c.save_interval_s, "top level", origin, err, false)) return std::nullopt;

    const auto parts = j.find("parts");
    if (parts == j.end() || !parts->is_array() || parts->empty()) {
        fail(err, origin, "\"parts\" must be a non-empty array");
        return std::nullopt;
    }
    for (std::size_t i = 0; i < parts->size(); ++i) {
        const json& po = (*parts)[i];
        const std::string where = "parts[" + std::to_string(i) + "]";
        if (!po.is_object()) { fail(err, origin, where + ": not an object"); return std::nullopt; }
        if (!known_keys(po, {"id", "label", "description", "effect", "scale_at_zero", "seize_below", "source"}, where, origin, err)) return std::nullopt;
        DamagePartConfig p;
        if (!detail::get_string(po, "id", p.id) || !is_snake(p.id)) { fail(err, origin, where + ": \"id\" must be a lower_snake name"); return std::nullopt; }
        if (c.find(p.id) != nullptr) { fail(err, origin, where + ": duplicate part id \"" + p.id + "\""); return std::nullopt; }
        if (!detail::get_string(po, "label", p.label) || p.label.empty()) { fail(err, origin, where + ": \"label\" is required"); return std::nullopt; }
        detail::get_string(po, "description", p.description);
        if (!detail::get_string(po, "source", p.source) || p.source.empty()) { fail(err, origin, where + ": \"source\" is required"); return std::nullopt; }
        std::string effect;
        if (!detail::get_string(po, "effect", effect)) { fail(err, origin, where + ": \"effect\" is required"); return std::nullopt; }
        if (effect == "radiator_ua") p.effect = DamageEffect::RadiatorUa;
        else if (effect == "fan_flow") p.effect = DamageEffect::FanFlow;
        else if (effect == "friction") p.effect = DamageEffect::Friction;
        else if (effect == "none") p.effect = DamageEffect::None;
        else { fail(err, origin, where + ": \"effect\" must be radiator_ua, fan_flow, friction or none"); return std::nullopt; }
        if (p.effect == DamageEffect::Friction) {
            // A worn bearing or ring rubs harder: the factor rises from 1 (healthy) to scale_at_zero.
            if (!number_in(po, "scale_at_zero", 1.0, 50.0, p.scale_at_zero, where, origin, err)) return std::nullopt;
        } else if (p.effect != DamageEffect::None) {
            if (!number_in(po, "scale_at_zero", 0.0, 1.0, p.scale_at_zero, where, origin, err)) return std::nullopt;
        } else if (po.contains("scale_at_zero")) {
            fail(err, origin, where + ": \"scale_at_zero\" needs an effect");
            return std::nullopt;
        }
        if (po.contains("seize_below") && !number_in(po, "seize_below", 0.0, 0.99, p.seize_below, where, origin, err)) return std::nullopt;
        c.parts.push_back(std::move(p));
    }

    const auto rules = j.find("rules");
    if (rules == j.end() || !rules->is_array()) {
        fail(err, origin, "\"rules\" must be an array");
        return std::nullopt;
    }
    std::set<std::string> rule_ids;
    for (std::size_t i = 0; i < rules->size(); ++i) {
        const json& ro = (*rules)[i];
        const std::string where = "rules[" + std::to_string(i) + "]";
        if (!ro.is_object()) { fail(err, origin, where + ": not an object"); return std::nullopt; }
        if (!known_keys(ro, {"id", "part", "signal", "above_c", "above_overheat_k", "after_s", "curve", "only_while_fan_on", "only_while_running", "source"}, where, origin, err))
            return std::nullopt;
        DamageRuleConfig r;
        if (!detail::get_string(ro, "id", r.id) || !is_snake(r.id) || !rule_ids.insert(r.id).second) {
            fail(err, origin, where + ": \"id\" must be a unique lower_snake name");
            return std::nullopt;
        }
        if (!detail::get_string(ro, "part", r.part) || c.find(r.part) == nullptr) {
            fail(err, origin, where + ": \"part\" must name one of the parts");
            return std::nullopt;
        }
        std::string signal;
        if (!detail::get_string(ro, "signal", signal) || (signal != "coolant_c" && signal != "oil_c")) {
            fail(err, origin, where + ": \"signal\" must be coolant_c or oil_c");
            return std::nullopt;
        }
        r.signal = signal == "oil_c" ? DamageSignal::OilC : DamageSignal::CoolantC;
        const bool abs = ro.contains("above_c"), rel = ro.contains("above_overheat_k");
        if (abs == rel) { fail(err, origin, where + ": give exactly one of \"above_c\" and \"above_overheat_k\""); return std::nullopt; }
        r.relative_to_overheat = rel;
        if (!number_in(ro, rel ? "above_overheat_k" : "above_c", rel ? -100.0 : -50.0, rel ? 200.0 : 400.0, r.above, where, origin, err)) return std::nullopt;
        if (!number_in(ro, "after_s", 0.0, 3600.0, r.after_s, where, origin, err, false)) return std::nullopt;
        const auto curve = ro.find("curve");
        if (curve == ro.end() || !curve->is_array() || curve->size() < 2) {
            fail(err, origin, where + ": \"curve\" must be an array of at least two [excess_k, health_per_s] points");
            return std::nullopt;
        }
        bool any_positive = false;
        for (std::size_t k = 0; k < curve->size(); ++k) {
            const json& pt = (*curve)[k];
            const std::string at = where + ".curve[" + std::to_string(k) + "]";
            if (!pt.is_array() || pt.size() != 2 || !pt[0].is_number() || !pt[1].is_number()) {
                fail(err, origin, at + ": a point is [excess_k, health_per_s]");
                return std::nullopt;
            }
            const double x = pt[0].get<double>(), y = pt[1].get<double>();
            if (!std::isfinite(x) || !std::isfinite(y) || x < 0.0 || x > 500.0 || y < 0.0 || y > 10.0) {
                fail(err, origin, at + ": excess in [0, 500] K, rate in [0, 10] per s");
                return std::nullopt;
            }
            if (k == 0 && x != 0.0) { fail(err, origin, at + ": the curve starts at excess 0"); return std::nullopt; }
            if (k > 0 && !(x > r.curve.back().first)) { fail(err, origin, at + ": excess must increase strictly"); return std::nullopt; }
            if (k > 0 && y < r.curve.back().second) { fail(err, origin, at + ": the rate must not fall as the excess grows"); return std::nullopt; }
            any_positive = any_positive || y > 0.0;
            r.curve.emplace_back(x, y);
        }
        if (!any_positive) { fail(err, origin, where + ": a curve needs a positive rate"); return std::nullopt; }
        if (ro.contains("only_while_fan_on") && !detail::get_bool(ro, "only_while_fan_on", r.only_while_fan_on)) {
            fail(err, origin, where + ": \"only_while_fan_on\" must be a boolean");
            return std::nullopt;
        }
        if (ro.contains("only_while_running") && !detail::get_bool(ro, "only_while_running", r.only_while_running)) {
            fail(err, origin, where + ": \"only_while_running\" must be a boolean");
            return std::nullopt;
        }
        if (!detail::get_string(ro, "source", r.source) || r.source.empty()) { fail(err, origin, where + ": \"source\" is required"); return std::nullopt; }
        c.rules.push_back(std::move(r));
    }
    return c;
}

std::optional<DamageConfig> load_damage_config(const std::string& path, std::string* err) {
    const std::optional<std::string> text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot read the file");
        return std::nullopt;
    }
    return parse_damage_config(*text, path, err);
}

double DamageState::health_of(const std::string& part) const {
    const auto it = health.find(part);
    return it == health.end() ? 1.0 : it->second;
}

bool DamageState::all_healthy() const {
    return std::all_of(health.begin(), health.end(), [](const auto& kv) { return kv.second == 1.0; });
}

std::string damage_state_path(const std::string& user_dir, const std::string& vehicle_id) {
    return (fs::path(user_dir) / "garage" / "damage" / (vehicle_id + ".json")).string();
}

std::optional<DamageState> parse_damage_state(const std::string& text, const std::string& origin, std::string* err) {
    const std::optional<json> doc = detail::parse_json(text);
    if (!doc || !doc->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    std::string format;
    if (!detail::get_string(*doc, "format", format) || format != "rg.damage_state/1") {
        fail(err, origin, "\"format\" must be \"rg.damage_state/1\"");
        return std::nullopt;
    }
    DamageState s;
    const auto parts = doc->find("parts");
    if (parts == doc->end()) return s;
    if (!parts->is_object()) {
        fail(err, origin, "\"parts\" must be an object (part id -> health)");
        return std::nullopt;
    }
    for (auto it = parts->begin(); it != parts->end(); ++it) {
        if (!is_snake(it.key()) || !it->is_number()) {
            fail(err, origin, "parts." + it.key() + ": a lower_snake part id with a number");
            return std::nullopt;
        }
        const double h = it->get<double>();
        if (!std::isfinite(h) || h < 0.0 || h > 1.0) {
            fail(err, origin, "parts." + it.key() + ": health must be in [0, 1]");
            return std::nullopt;
        }
        s.health[it.key()] = h;
    }
    return s;
}

std::optional<DamageState> load_damage_state(const std::string& path, std::string* err) {
    std::error_code ec;
    if (!fs::exists(fs::path(path), ec)) return DamageState{};
    const std::optional<std::string> text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot read the file");
        return std::nullopt;
    }
    return parse_damage_state(*text, path, err);
}

std::string damage_state_to_json(const DamageState& state) {
    json parts = json::object();
    for (const auto& [id, h] : state.health)
        if (h != 1.0) parts[id] = h;
    json j = {{"format", "rg.damage_state/1"}, {"parts", parts}};
    return j.dump(2) + "\n";
}

bool save_damage_state(const std::string& path, const DamageState& state, std::string* err) {
    std::error_code ec;
    const fs::path p(path);
    if (state.all_healthy()) {
        fs::remove(p, ec);
        if (ec) return fail(err, path, "cannot remove the file: " + ec.message());
        return true;
    }
    fs::create_directories(p.parent_path(), ec);
    if (ec) return fail(err, path, "cannot create the folder: " + ec.message());
    const fs::path tmp = p.string() + ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) return fail(err, tmp.string(), "cannot write the file");
        out << damage_state_to_json(state);
        if (!out) return fail(err, tmp.string(), "write failed");
    }
    fs::rename(tmp, p, ec);
    if (ec) {
        fs::remove(tmp, ec);
        return fail(err, path, "cannot replace the file");
    }
    return true;
}

double wear_rate_per_s(const DamageRuleConfig& rule, double excess_k) {
    const auto& c = rule.curve;
    if (!(excess_k > 0.0) || c.size() < 2) return 0.0;
    std::size_t i = 1;
    while (i + 1 < c.size() && excess_k > c[i].first) ++i; // segment [i-1, i]; past the last point: the last segment
    const double x0 = c[i - 1].first, y0 = c[i - 1].second, x1 = c[i].first, y1 = c[i].second;
    const double rate = y0 + (y1 - y0) * (excess_k - x0) / (x1 - x0);
    return std::max(0.0, rate);
}

DamageScales damage_scales(const DamageConfig& config, const DamageState& state) {
    DamageScales s;
    for (const DamagePartConfig& p : config.parts) {
        const double h = state.health_of(p.id);
        if (h == 1.0) continue; // exactly unity: no multiplication at all
        const double f = lerp_scale(p.scale_at_zero, h);
        if (p.effect == DamageEffect::RadiatorUa) s.radiator_ua *= f;
        else if (p.effect == DamageEffect::FanFlow) s.fan_flow *= f;
        else if (p.effect == DamageEffect::Friction) s.friction *= f;
    }
    return s;
}

const DamagePartConfig* seized_part(const DamageConfig& config, const DamageState& state) {
    for (const DamagePartConfig& p : config.parts)
        if (p.can_seize() && state.health_of(p.id) <= p.seize_below) return &p;
    return nullptr;
}

DamageModel::DamageModel(DamageConfig config, DamageState state, double overheat_c)
    : config_(std::move(config)), state_(std::move(state)), overheat_c_(overheat_c), dwell_(config_.rules.size(), 0.0) {}

bool DamageModel::step(double dt, const DamageInputs& in) {
    if (!in.has_thermal || !(dt > 0.0)) return false;
    bool changed = false;
    for (std::size_t i = 0; i < config_.rules.size(); ++i) {
        const DamageRuleConfig& r = config_.rules[i];
        if (r.relative_to_overheat && !(overheat_c_ > 0.0)) continue; // the car has no overheat warning
        const double threshold = r.relative_to_overheat ? overheat_c_ + r.above : r.above;
        const double value = r.signal == DamageSignal::OilC ? in.oil_c : in.coolant_c;
        const bool active = value > threshold && (!r.only_while_fan_on || in.fan_on) && (!r.only_while_running || in.engine_running);
        if (!active) {
            dwell_[i] = 0.0;
            continue;
        }
        dwell_[i] += dt;
        if (dwell_[i] < r.after_s) continue;
        const auto it = state_.health.find(r.part);
        const double before = it == state_.health.end() ? 1.0 : it->second;
        if (before <= 0.0) continue;
        const double loss = wear_rate_per_s(r, value - threshold) * dt;
        const double after = std::max(0.0, before - loss);
        if (after != before) {
            state_.health[r.part] = after;
            changed = true;
        }
    }
    return changed;
}

void DamageModel::repair() {
    state_.health.clear();
    std::fill(dwell_.begin(), dwell_.end(), 0.0);
}

DamageView damage_view(const DamageConfig& config, const DamageState& state) {
    DamageView v;
    std::set<std::string> listed;
    for (const DamagePartConfig& p : config.parts) {
        DamagePartView pv;
        pv.id = p.id;
        pv.label = p.label;
        pv.description = p.description;
        pv.health = state.health_of(p.id);
        pv.damaged = pv.health < config.report_below;
        pv.effect = p.effect;
        pv.effect_scale = p.effect == DamageEffect::None ? 1.0 : lerp_scale(p.scale_at_zero, pv.health);
        pv.seized = p.can_seize() && pv.health <= p.seize_below;
        if (pv.seized && !v.seized) {
            v.seized = true;
            v.seized_label = p.label;
        }
        v.any_damaged = v.any_damaged || pv.damaged;
        listed.insert(p.id);
        v.parts.push_back(std::move(pv));
    }
    for (const auto& [id, h] : state.health) { // a part a newer build saved: shown, no effect here
        if (listed.count(id) != 0) continue;
        DamagePartView pv;
        pv.id = id;
        pv.label = id;
        pv.health = h;
        pv.damaged = h < config.report_below;
        v.any_damaged = v.any_damaged || pv.damaged;
        v.parts.push_back(std::move(pv));
    }
    v.scales = damage_scales(config, state);
    return v;
}

std::string damage_warning_line(const DamageView& view) {
    if (view.seized) return "ENGINE SEIZED (" + view.seized_label + ") - repair the car: pause menu or garage";
    if (!view.any_damaged) return "";
    std::string line = "DAMAGE:";
    bool first = true;
    for (const DamagePartView& p : view.parts) {
        if (!p.damaged) continue;
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s %s %.0f %%", first ? "" : ",", p.label.c_str(), std::floor(p.health * 100.0));
        line += buf;
        first = false;
    }
    return line + " - repair: pause menu or garage";
}

} // namespace rg
