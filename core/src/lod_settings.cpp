// rg/lod_settings.cpp — see lod_settings.h.
#include "rg/lod_settings.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <mutex>

namespace rg {

namespace {

using detail::json;

bool fail(std::string* err, const std::string& origin, const std::string& message) {
    if (err != nullptr) *err = origin + ": " + message;
    return false;
}

bool parse_def(const json& j, const std::string& section, const std::string& origin, SettingDef& d, std::string* err) {
    if (!j.is_object()) return fail(err, origin, "a \"settings\" entry is not an object");
    std::string type;
    if (!detail::get_string(j, "key", d.key) || !detail::get_string(j, "type", type) ||
        !detail::get_string(j, "label", d.label)) {
        return fail(err, origin, "a setting needs \"key\", \"type\" and \"label\" strings");
    }
    detail::get_string(j, "help", d.help);
    if (d.key.rfind(section + ".", 0) != 0 || d.key.size() <= section.size() + 1) {
        return fail(err, origin, d.key + ": key must start with \"" + section + ".\"");
    }
    d.section = section;
    if (type == "float" || type == "int") {
        double def = 0.0;
        if (!detail::get_number(j, "default", def) || !detail::get_number(j, "min", d.min) ||
            !detail::get_number(j, "max", d.max) || !detail::get_number(j, "step", d.step)) {
            return fail(err, origin, d.key + ": a number needs \"default\", \"min\", \"max\" and \"step\"");
        }
        if (!(d.min < d.max) || def < d.min || def > d.max || !(d.step > 0.0)) {
            return fail(err, origin, d.key + ": needs min < max, min <= default <= max and step > 0");
        }
        if (type == "float") {
            d.type = SettingType::Float;
            d.default_value = def;
            double decimals = -1.0;
            if (detail::get_number(j, "decimals", decimals)) d.decimals = static_cast<int>(decimals);
        } else {
            d.type = SettingType::Int;
            d.default_value = static_cast<std::int64_t>(def);
        }
        return true;
    }
    if (type == "choice") {
        std::string def;
        const auto it = j.find("choices");
        if (!detail::get_string(j, "default", def) || it == j.end() || !it->is_array() || it->empty()) {
            return fail(err, origin, d.key + ": a choice needs \"default\" and a non-empty \"choices\" array");
        }
        for (const json& c : *it) {
            if (!c.is_string()) return fail(err, origin, d.key + ": choices must be strings");
            d.choices.push_back(c.get<std::string>());
        }
        if (std::find(d.choices.begin(), d.choices.end(), def) == d.choices.end()) {
            return fail(err, origin, d.key + ": default is not one of the choices");
        }
        d.type = SettingType::Choice;
        d.default_value = def;
        return true;
    }
    return fail(err, origin, d.key + ": type must be float, int or choice");
}

bool parse_preset(const json& j, const std::string& name, const std::string& origin, g2m::RenderRoadLod& out,
                  std::string* err) {
    if (!j.is_array()) return fail(err, origin, "road preset \"" + name + "\" is not an array");
    for (const json& level : j) {
        g2m::RenderRoadLodLevel l;
        if (level.is_null()) {
            l.enabled = false;
            out.levels.push_back(l);
            continue;
        }
        std::string cls;
        if (!level.is_object() || !detail::get_string(level, "min_class", cls)) {
            return fail(err, origin, "road preset \"" + name + "\": a level is null or {\"min_class\": ...}");
        }
        if (cls != "all") {
            const g2m::HighwayStyleEntry* e = g2m::RoadStyle::default_style().find_highway(cls);
            if (e == nullptr) return fail(err, origin, "road preset \"" + name + "\": unknown road class \"" + cls + "\"");
            l.min_rank = e->rank;
        }
        double half_width_m = 0.0;
        if (detail::get_number(level, "min_half_width_m", half_width_m)) {
            if (!(half_width_m >= 0.0) || half_width_m > 1000.0) {
                return fail(err, origin, "road preset \"" + name + "\": min_half_width_m must be 0..1000");
            }
            l.min_half_width_mm = std::llround(half_width_m * 1000.0);
        }
        out.levels.push_back(l);
    }
    return true;
}

std::mutex& active_mutex() {
    static std::mutex m;
    return m;
}
std::shared_ptr<const LodSettingsSpec>& active_slot() {
    static std::shared_ptr<const LodSettingsSpec> spec;
    return spec;
}

} // namespace

const g2m::RenderRoadLod* LodSettingsSpec::road_preset(const std::string& name) const {
    for (const auto& [n, lod] : road_presets) {
        if (n == name) return &lod;
    }
    return nullptr;
}

std::optional<LodSettingsSpec> parse_lod_settings_spec(const std::string& json_text, const std::string& origin,
                                                       std::string* err) {
    const std::optional<json> root = detail::parse_json(json_text);
    if (!root || !root->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    std::string format;
    if (!detail::get_string(*root, "format", format) || format != "rg.lod_settings/1") {
        fail(err, origin, "\"format\" must be \"rg.lod_settings/1\"");
        return std::nullopt;
    }
    LodSettingsSpec spec;
    const auto section = root->find("section");
    if (section == root->end() || !section->is_object() || !detail::get_string(*section, "id", spec.section) ||
        !detail::get_string(*section, "title", spec.section_title) || spec.section.empty()) {
        fail(err, origin, "\"section\" needs \"id\" and \"title\"");
        return std::nullopt;
    }
    const auto settings = root->find("settings");
    if (settings == root->end() || !settings->is_array()) {
        fail(err, origin, "\"settings\" must be an array");
        return std::nullopt;
    }
    for (const json& j : *settings) {
        SettingDef d;
        if (!parse_def(j, spec.section, origin, d, err)) return std::nullopt;
        spec.defs.push_back(std::move(d));
    }
    const auto presets = root->find("road_presets");
    if (presets == root->end() || !presets->is_object()) {
        fail(err, origin, "\"road_presets\" must be an object");
        return std::nullopt;
    }
    for (auto it = presets->begin(); it != presets->end(); ++it) {
        g2m::RenderRoadLod lod;
        if (!parse_preset(*it, it.key(), origin, lod, err)) return std::nullopt;
        spec.road_presets.emplace_back(it.key(), std::move(lod));
    }
    // The consumers read exactly these four keys.
    const auto find = [&](const char* key) -> const SettingDef* {
        for (const SettingDef& d : spec.defs) {
            if (d.key == key) return &d;
        }
        return nullptr;
    };
    for (const char* key : {kLodViewDistanceKey, kLodTerrainDetailKey, kLodMapDetailKey}) {
        const SettingDef* d = find(key);
        if (d == nullptr || d->type != SettingType::Float) {
            fail(err, origin, std::string(key) + ": missing or not a float setting");
            return std::nullopt;
        }
        if (!(d->min > 0.0)) {
            fail(err, origin, std::string(key) + ": min must be > 0");
            return std::nullopt;
        }
    }
    if (const SettingDef* m = find(kLodMapDetailKey); m->min < 0.05 || m->max > 20.0) {
        fail(err, origin, std::string(kLodMapDetailKey) + ": range must lie within 0.05..20 (geo2map detail_scale)");
        return std::nullopt;
    }
    const SettingDef* road = find(kLodRoadDetailKey);
    if (road == nullptr || road->type != SettingType::Choice) {
        fail(err, origin, std::string(kLodRoadDetailKey) + ": missing or not a choice setting");
        return std::nullopt;
    }
    for (const std::string& c : road->choices) {
        if (spec.road_preset(c) == nullptr) {
            fail(err, origin, std::string(kLodRoadDetailKey) + ": choice \"" + c + "\" has no road preset");
            return std::nullopt;
        }
    }
    return spec;
}

std::optional<LodSettingsSpec> load_lod_settings_spec(const std::string& path, std::string* err) {
    const std::optional<std::string> text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot read");
        return std::nullopt;
    }
    return parse_lod_settings_spec(*text, path, err);
}

void install_lod_settings(const LodSettingsSpec& spec) {
    extend_settings_schema(spec.defs, spec.section, spec.section_title);
    std::lock_guard<std::mutex> lk(active_mutex());
    active_slot() = std::make_shared<const LodSettingsSpec>(spec);
}

std::shared_ptr<const LodSettingsSpec> active_lod_settings() {
    std::lock_guard<std::mutex> lk(active_mutex());
    return active_slot();
}

LodChoice lod_choice_from(const Settings& settings) {
    LodChoice c;
    if (find_setting(kLodViewDistanceKey) != nullptr) c.view_distance_m = settings.get_float(kLodViewDistanceKey);
    if (find_setting(kLodTerrainDetailKey) != nullptr) c.terrain_detail = settings.get_float(kLodTerrainDetailKey);
    if (find_setting(kLodRoadDetailKey) != nullptr) c.road_detail = settings.get_string(kLodRoadDetailKey);
    if (find_setting(kLodMapDetailKey) != nullptr) c.map_detail = settings.get_float(kLodMapDetailKey);
    return c;
}

g2m::mesh::LodParams terrain_lod_params(const g2m::mesh::LodParams& world, const LodChoice& choice) {
    g2m::mesh::LodParams p = world;
    if (std::isfinite(choice.terrain_detail) && choice.terrain_detail > 0.0) p.range0_m = world.range0_m * choice.terrain_detail;
    if (std::isfinite(choice.view_distance_m) && choice.view_distance_m > 0.0) p.max_distance_m = choice.view_distance_m;
    return p;
}

std::shared_ptr<const g2m::RenderRoadLod> road_lod_for(const LodSettingsSpec* spec, const std::string& preset) {
    if (spec == nullptr) return nullptr;
    const g2m::RenderRoadLod* lod = spec->road_preset(preset);
    if (lod == nullptr) return nullptr;
    return std::make_shared<const g2m::RenderRoadLod>(*lod);
}

g2m::route::MapConfig map_config_for(const LodChoice& choice) {
    g2m::route::MapConfig config = g2m::route::default_map_config();
    if (std::isfinite(choice.map_detail)) config.detail_scale = std::clamp(choice.map_detail, 0.05, 20.0);
    return config;
}

} // namespace rg
