// rg/settings.cpp — see settings.h.
#include "rg/settings.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <system_error>

namespace rg {

namespace {

using detail::json;

SettingDef make_bool(const char* section, const char* name, const char* label, const char* help, bool def) {
    SettingDef d;
    d.section = section;
    d.key = std::string(section) + "." + name;
    d.label = label;
    d.help = help;
    d.type = SettingType::Bool;
    d.default_value = def;
    return d;
}

SettingDef make_int(const char* section, const char* name, const char* label, const char* help, std::int64_t def,
                    double lo, double hi, double step) {
    SettingDef d;
    d.section = section;
    d.key = std::string(section) + "." + name;
    d.label = label;
    d.help = help;
    d.type = SettingType::Int;
    d.default_value = def;
    d.min = lo;
    d.max = hi;
    d.step = step;
    return d;
}

SettingDef make_float(const char* section, const char* name, const char* label, const char* help, double def,
                      double lo, double hi, double step) {
    SettingDef d;
    d.section = section;
    d.key = std::string(section) + "." + name;
    d.label = label;
    d.help = help;
    d.type = SettingType::Float;
    d.default_value = def;
    d.min = lo;
    d.max = hi;
    d.step = step;
    return d;
}

SettingDef make_choice(const char* section, const char* name, const char* label, const char* help, const char* def,
                       std::vector<std::string> choices) {
    SettingDef d;
    d.section = section;
    d.key = std::string(section) + "." + name;
    d.label = label;
    d.help = help;
    d.type = SettingType::Choice;
    d.default_value = std::string(def);
    d.choices = std::move(choices);
    return d;
}

std::vector<SettingDef> build_schema() {
    std::vector<SettingDef> s;
    s.push_back(make_choice("graphics", "window_mode", "Window mode",
                            "Windowed, borderless full screen, or exclusive full screen.", "windowed",
                            {"windowed", "fullscreen", "exclusive_fullscreen"}));
    s.push_back(make_bool("graphics", "vsync", "Vertical sync", "Wait for the display refresh (no tearing).", true));
    s.push_back(make_int("graphics", "max_fps", "Maximum frame rate",
                         "Caps the render rate. 0 = unlimited. The simulation runs at its own fixed rate either way.", 0,
                         0.0, 360.0, 10.0));
    s.push_back(make_float("camera", "fov_deg", "Field of view",
                           "Vertical field of view of the driving cameras, in degrees.", 70.0, 50.0, 110.0, 1.0));
    s.push_back(make_float("audio", "master_volume", "Master volume", "Scales every sound.", 1.0, 0.0, 1.0, 0.05));
    s.push_back(make_float("audio", "engine_volume", "Engine volume", "Engine sound level, before the master volume.",
                           1.0, 0.0, 1.0, 0.05));
    s.push_back(make_float("audio", "tyre_volume", "Tyre volume", "Tyre noise level, before the master volume.", 1.0,
                           0.0, 1.0, 0.05));
    SettingDef store;
    store.section = "map_data";
    store.key = "map_data.store_dir";
    store.label = "Map data folder";
    store.help = "Folder holding the geo2map tile store (tiles.sqlite3). Empty = RG_G2M_HOME, else cache/g2m/home-r1 in "
                 "the game folder. Takes effect when the next world loads.";
    store.type = SettingType::Path;
    store.default_value = std::string();
    store.applies_live = false;
    s.push_back(std::move(store));
    return s;
}

bool is_integral(double v) { return std::isfinite(v) && v == std::floor(v) && std::fabs(v) < 9.0e15; }

bool valid_path(const std::string& p) {
    if (p.size() > 1024) return false;
    return std::none_of(p.begin(), p.end(), [](unsigned char c) { return c < 0x20 || c == 0x7f; });
}

double clamp_to(double v, double lo, double hi, bool& clamped) {
    if (v < lo) {
        clamped = true;
        return lo;
    }
    if (v > hi) {
        clamped = true;
        return hi;
    }
    return v;
}

// Converts `in` to the def's type, validating. On success sets `out` and, for
// numbers, *clamped. On failure sets *err.
bool coerce(const SettingDef& def, const SettingValue& in, SettingValue& out, bool* clamped, std::string* err) {
    bool was_clamped = false;
    const auto set_err = [&](const std::string& m) {
        if (err != nullptr) *err = def.key + ": " + m;
        return false;
    };
    switch (def.type) {
        case SettingType::Bool:
            if (const bool* b = std::get_if<bool>(&in)) {
                out = *b;
                return true;
            }
            return set_err("expected a boolean");
        case SettingType::Int: {
            double v = 0.0;
            if (const auto* i = std::get_if<std::int64_t>(&in)) v = static_cast<double>(*i);
            else if (const auto* d = std::get_if<double>(&in)) {
                if (!is_integral(*d)) return set_err("expected a whole number");
                v = *d;
            } else return set_err("expected a whole number");
            out = static_cast<std::int64_t>(clamp_to(v, def.min, def.max, was_clamped));
            break;
        }
        case SettingType::Float: {
            double v = 0.0;
            if (const auto* i = std::get_if<std::int64_t>(&in)) v = static_cast<double>(*i);
            else if (const auto* d = std::get_if<double>(&in)) v = *d;
            else return set_err("expected a number");
            if (!std::isfinite(v)) return set_err("expected a finite number");
            out = clamp_to(v, def.min, def.max, was_clamped);
            break;
        }
        case SettingType::Choice: {
            const std::string* s = std::get_if<std::string>(&in);
            if (s == nullptr) return set_err("expected a string");
            if (std::find(def.choices.begin(), def.choices.end(), *s) == def.choices.end()) {
                return set_err("\"" + *s + "\" is not one of the choices");
            }
            out = *s;
            return true;
        }
        case SettingType::Path: {
            const std::string* s = std::get_if<std::string>(&in);
            if (s == nullptr) return set_err("expected a string");
            if (!valid_path(*s)) return set_err("not a valid path");
            out = *s;
            return true;
        }
    }
    if (clamped != nullptr) *clamped = was_clamped;
    return true;
}

SettingValue neutral_for(SettingType t) {
    switch (t) {
        case SettingType::Bool: return false;
        case SettingType::Int: return std::int64_t{0};
        case SettingType::Float: return 0.0;
        case SettingType::Choice:
        case SettingType::Path: return std::string();
    }
    return false;
}

bool scalar_from_json(const json& j, SettingValue& out) {
    if (j.is_boolean()) out = j.get<bool>();
    else if (j.is_number_integer()) out = j.get<std::int64_t>();
    else if (j.is_number_float()) out = j.get<double>();
    else if (j.is_string()) out = j.get<std::string>();
    else return false;
    return true;
}

json json_from_value(const SettingValue& v) {
    if (const bool* b = std::get_if<bool>(&v)) return *b;
    if (const auto* i = std::get_if<std::int64_t>(&v)) return *i;
    if (const double* d = std::get_if<double>(&v)) return *d;
    return std::get<std::string>(v);
}

} // namespace

const std::vector<SettingDef>& settings_schema() {
    static const std::vector<SettingDef> schema = build_schema();
    return schema;
}

const SettingDef* find_setting(const std::string& key) {
    for (const SettingDef& d : settings_schema()) {
        if (d.key == key) return &d;
    }
    return nullptr;
}

std::vector<std::string> settings_sections() {
    std::vector<std::string> out;
    for (const SettingDef& d : settings_schema()) {
        if (std::find(out.begin(), out.end(), d.section) == out.end()) out.push_back(d.section);
    }
    return out;
}

std::string settings_section_title(const std::string& section) {
    if (section == "graphics") return "Graphics";
    if (section == "camera") return "Camera";
    if (section == "audio") return "Audio";
    if (section == "map_data") return "Map data";
    return section;
}

const std::vector<SettingsMigration>& default_settings_migrations() {
    static const std::vector<SettingsMigration> none;
    return none;
}

Settings::Settings() { reset_all(); dirty_ = false; }

void Settings::reset_all() {
    values_.clear();
    for (const SettingDef& d : settings_schema()) values_[d.key] = d.default_value;
    dirty_ = true;
}

void Settings::reset(const std::string& key) {
    if (const SettingDef* d = find_setting(key)) {
        values_[key] = d->default_value;
        dirty_ = true;
    }
}

SettingValue Settings::get(const std::string& key) const {
    const auto it = values_.find(key);
    if (it != values_.end()) return it->second;
    const SettingDef* d = find_setting(key);
    return d != nullptr ? d->default_value : SettingValue(false);
}

bool Settings::get_bool(const std::string& key) const {
    const SettingValue v = get(key);
    const bool* b = std::get_if<bool>(&v);
    return b != nullptr && *b;
}

std::int64_t Settings::get_int(const std::string& key) const {
    const SettingValue v = get(key);
    if (const auto* i = std::get_if<std::int64_t>(&v)) return *i;
    if (const double* d = std::get_if<double>(&v)) return static_cast<std::int64_t>(*d);
    return 0;
}

double Settings::get_float(const std::string& key) const {
    const SettingValue v = get(key);
    if (const double* d = std::get_if<double>(&v)) return *d;
    if (const auto* i = std::get_if<std::int64_t>(&v)) return static_cast<double>(*i);
    return 0.0;
}

std::string Settings::get_string(const std::string& key) const {
    const SettingValue v = get(key);
    const std::string* s = std::get_if<std::string>(&v);
    return s != nullptr ? *s : std::string();
}

bool Settings::set(const std::string& key, const SettingValue& value, std::string* err, bool* clamped) {
    if (clamped != nullptr) *clamped = false;
    const SettingDef* def = find_setting(key);
    if (def == nullptr) {
        if (err != nullptr) *err = key + ": unknown setting";
        return false;
    }
    SettingValue coerced;
    if (!coerce(*def, value, coerced, clamped, err)) return false;
    if (values_[key] != coerced) {
        values_[key] = coerced;
        dirty_ = true;
    }
    return true;
}

std::string Settings::display_value(const std::string& key) const {
    const SettingDef* def = find_setting(key);
    if (def == nullptr) return {};
    char buf[64];
    switch (def->type) {
        case SettingType::Bool: return get_bool(key) ? "on" : "off";
        case SettingType::Int:
            if (key == "graphics.max_fps" && get_int(key) == 0) return "unlimited";
            return std::to_string(get_int(key));
        case SettingType::Float:
            std::snprintf(buf, sizeof(buf), def->max <= 1.0 ? "%.2f" : "%.0f", get_float(key));
            return buf;
        case SettingType::Choice: return get_string(key);
        case SettingType::Path: return get_string(key).empty() ? "(default)" : get_string(key);
    }
    return {};
}

SettingsLoadReport Settings::load_text(const std::string& json_text, const std::vector<SettingsMigration>& migrations) {
    SettingsLoadReport report;
    reset_all();
    extras_.clear();
    dirty_ = false;

    const std::optional<json> parsed = detail::parse_json(json_text);
    if (!parsed || !parsed->is_object()) {
        report.parse_error = true;
        return report;
    }

    // Version.
    std::string format;
    if (detail::get_string(*parsed, "format", format) && format.rfind("rg.settings/", 0) == 0) {
        const std::string tail = format.substr(12);
        if (!tail.empty() && std::all_of(tail.begin(), tail.end(), [](unsigned char c) { return c >= '0' && c <= '9'; }) &&
            tail.size() < 6) {
            report.version_found = std::stoi(tail);
        }
    }
    report.from_newer = report.version_found > kSettingsFormatVersion;

    // Flatten: scalar members of the section objects (and scalars at top level)
    // into a SettingsDoc, everything else into extras as raw JSON.
    SettingsDoc doc;
    for (auto it = parsed->begin(); it != parsed->end(); ++it) {
        const std::string& name = it.key();
        if (name == "format") continue;
        if (it->is_object()) {
            for (auto m = it->begin(); m != it->end(); ++m) {
                const std::string key = name + "." + m.key();
                SettingValue v;
                if (scalar_from_json(*m, v)) doc[key] = v;
                else extras_[key] = m->dump();
            }
        } else {
            SettingValue v;
            if (scalar_from_json(*it, v)) doc[name] = v;
            else extras_[name] = it->dump();
        }
    }

    // Migrate older documents.
    if (report.version_found >= 1 && report.version_found < kSettingsFormatVersion) {
        for (int v = report.version_found; v < kSettingsFormatVersion; ++v) {
            for (const SettingsMigration& m : migrations) {
                if (m.from == v && m.apply) {
                    m.apply(doc);
                    report.migrated = true;
                }
            }
        }
    } else if (report.version_found == 0 && !migrations.empty()) {
        // A document with no readable format string is treated as version 0.
        for (int v = 0; v < kSettingsFormatVersion; ++v) {
            for (const SettingsMigration& m : migrations) {
                if (m.from == v && m.apply) {
                    m.apply(doc);
                    report.migrated = true;
                }
            }
        }
    }

    // Apply to the schema.
    for (const auto& [key, value] : doc) {
        const SettingDef* def = find_setting(key);
        if (def == nullptr) {
            extras_[key] = json_from_value(value).dump();
            report.unknown_keys.push_back(key);
            continue;
        }
        SettingValue coerced;
        bool clamped = false;
        if (!coerce(*def, value, coerced, &clamped, nullptr)) {
            report.invalid_keys.push_back(key);
            continue; // keeps the default
        }
        if (clamped) report.clamped_keys.push_back(key);
        values_[key] = coerced;
    }
    for (const auto& [key, raw] : extras_) {
        if (std::find(report.unknown_keys.begin(), report.unknown_keys.end(), key) == report.unknown_keys.end()) {
            report.unknown_keys.push_back(key);
        }
    }
    std::sort(report.unknown_keys.begin(), report.unknown_keys.end());
    dirty_ = !report.invalid_keys.empty() || !report.clamped_keys.empty() || report.migrated;
    return report;
}

SettingsLoadReport Settings::load_file(const std::string& path, const std::vector<SettingsMigration>& migrations) {
    const std::optional<std::string> text = detail::read_text_file(path);
    if (!text) {
        SettingsLoadReport r;
        reset_all();
        extras_.clear();
        dirty_ = false;
        r.file_missing = true;
        return r;
    }
    return load_text(*text, migrations);
}

std::string Settings::to_json() const {
    json root = json::object();
    root["format"] = "rg.settings/" + std::to_string(kSettingsFormatVersion);
    for (const SettingDef& d : settings_schema()) {
        root[d.section][d.key.substr(d.section.size() + 1)] = json_from_value(get(d.key));
    }
    for (const auto& [key, raw] : extras_) {
        const json value = json::parse(raw, nullptr, false);
        if (value.is_discarded()) continue;
        const std::size_t dot = key.find('.');
        if (dot == std::string::npos) root[key] = value;
        else root[key.substr(0, dot)][key.substr(dot + 1)] = value;
    }
    return root.dump(2) + "\n";
}

bool Settings::save_file(const std::string& path, std::string* err) {
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path target(path);
    if (target.has_parent_path()) fs::create_directories(target.parent_path(), ec);
    const fs::path tmp = fs::path(path + ".tmp");
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) {
            if (err != nullptr) *err = path + ": cannot write";
            return false;
        }
        out << to_json();
        out.flush();
        if (!out) {
            if (err != nullptr) *err = path + ": write failed";
            return false;
        }
    }
    ec.clear();
    fs::rename(tmp, target, ec);
    if (ec) {
        if (err != nullptr) *err = path + ": rename failed: " + ec.message();
        fs::remove(tmp, ec);
        return false;
    }
    dirty_ = false;
    return true;
}

} // namespace rg
