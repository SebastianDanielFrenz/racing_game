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

double lerp_scale(double min_scale, double health) { return min_scale + (1.0 - min_scale) * health; }

} // namespace

const char* to_string(DamageEffect effect) {
    switch (effect) {
        case DamageEffect::None: return "none";
        case DamageEffect::RadiatorUa: return "radiator_ua";
        case DamageEffect::FanFlow: return "fan_flow";
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
    if (!detail::get_string(j, "format", format) || format != "rg.damage/1") {
        fail(err, origin, "\"format\" must be \"rg.damage/1\"");
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
        if (!known_keys(po, {"id", "label", "description", "effect", "min_scale", "source"}, where, origin, err)) return std::nullopt;
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
        else if (effect == "none") p.effect = DamageEffect::None;
        else { fail(err, origin, where + ": \"effect\" must be radiator_ua, fan_flow or none"); return std::nullopt; }
        if (p.effect != DamageEffect::None) {
            if (!number_in(po, "min_scale", 0.0, 1.0, p.min_scale, where, origin, err)) return std::nullopt;
        } else if (po.contains("min_scale")) {
            fail(err, origin, where + ": \"min_scale\" needs an effect");
            return std::nullopt;
        }
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
        if (!known_keys(ro, {"id", "part", "signal", "above_c", "above_overheat_k", "after_s", "rate_per_s", "rate_per_s_per_k", "only_while_fan_on", "source"},
                        where, origin, err))
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
        if (!number_in(ro, "after_s", 0.0, 3600.0, r.after_s, where, origin, err)) return std::nullopt;
        if (!number_in(ro, "rate_per_s", 0.0, 1.0, r.rate_per_s, where, origin, err)) return std::nullopt;
        if (!number_in(ro, "rate_per_s_per_k", 0.0, 1.0, r.rate_per_s_per_k, where, origin, err, false)) return std::nullopt;
        if (!(r.rate_per_s > 0.0) && !(r.rate_per_s_per_k > 0.0)) { fail(err, origin, where + ": a rule needs a positive rate"); return std::nullopt; }
        if (ro.contains("only_while_fan_on") && !detail::get_bool(ro, "only_while_fan_on", r.only_while_fan_on)) {
            fail(err, origin, where + ": \"only_while_fan_on\" must be a boolean");
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

CoolingScales cooling_scales(const DamageConfig& config, const DamageState& state) {
    CoolingScales s;
    for (const DamagePartConfig& p : config.parts) {
        const double h = state.health_of(p.id);
        if (h == 1.0) continue; // exactly unity: no multiplication at all
        const double f = lerp_scale(p.min_scale, h);
        if (p.effect == DamageEffect::RadiatorUa) s.radiator_ua *= f;
        else if (p.effect == DamageEffect::FanFlow) s.fan_flow *= f;
    }
    return s;
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
        const bool active = value > threshold && (!r.only_while_fan_on || in.fan_on);
        if (!active) {
            dwell_[i] = 0.0;
            continue;
        }
        dwell_[i] += dt;
        if (dwell_[i] < r.after_s) continue;
        const auto it = state_.health.find(r.part);
        const double before = it == state_.health.end() ? 1.0 : it->second;
        if (before <= 0.0) continue;
        const double loss = (r.rate_per_s + r.rate_per_s_per_k * (value - threshold)) * dt;
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
        pv.effect_scale = p.effect == DamageEffect::None ? 1.0 : lerp_scale(p.min_scale, pv.health);
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
    v.scales = cooling_scales(config, state);
    return v;
}

std::string damage_warning_line(const DamageView& view) {
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
    return line + " - repair in the garage";
}

} // namespace rg
