// rg_ui_theme.cpp — see rg_ui_theme.h.
#include "rg_ui_theme.h"

#include "rg/ui_theme.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>

using godot::Array;
using godot::Dictionary;
using godot::PackedStringArray;
using godot::String;

namespace {

String from_std(const std::string& text) {
    return String::utf8(text.c_str());
}

std::string to_std(const String& text) {
    return text.utf8().get_data();
}

PackedStringArray strings(const std::vector<std::string>& values) {
    PackedStringArray out;
    for (const std::string& v : values) out.push_back(from_std(v));
    return out;
}

godot::Color colour(const rg::UiColour& c) {
    return godot::Color(static_cast<float>(c.r), static_cast<float>(c.g), static_cast<float>(c.b), static_cast<float>(c.a));
}

} // namespace

namespace rg_godot {

Dictionary RgUiTheme::load_theme(const String& data_dir) {
    Dictionary out;
    std::string err;
    const std::optional<rg::UiTheme> theme = rg::load_ui_theme(to_std(data_dir), &err);
    out["ok"] = theme.has_value();
    out["error"] = from_std(err);
    if (!theme) return out;

    Dictionary palettes;
    PackedStringArray ids;
    for (const rg::UiPalette& p : theme->palettes) {
        Dictionary colours;
        for (const auto& [role, c] : p.colours) colours[godot::StringName(role.c_str())] = colour(c);
        Dictionary d;
        d["label"] = from_std(p.label);
        d["claims"] = strings(p.claims);
        d["colours"] = colours;
        palettes[from_std(p.id)] = d;
        ids.push_back(from_std(p.id));
    }
    Array refused;
    for (const rg::UiRefusedPalette& r : theme->refused) {
        Dictionary d;
        d["id"] = from_std(r.id);
        d["problems"] = strings(r.problems);
        refused.push_back(d);
    }
    Array roles;
    for (const rg::UiRoleDef& r : theme->contract.roles) {
        Dictionary d;
        d["id"] = from_std(r.id);
        d["group"] = from_std(r.group);
        d["meaning"] = from_std(r.meaning);
        roles.push_back(d);
    }
    Array groups;
    for (const rg::UiRoleGroup& g : theme->contract.groups) {
        Dictionary d;
        d["id"] = from_std(g.id);
        d["title"] = from_std(g.title);
        groups.push_back(d);
    }
    Dictionary tokens;
    for (const auto& [name, value] : theme->tokens.values) tokens[godot::StringName(name.c_str())] = value;

    out["palettes"] = palettes;
    out["ids"] = ids;
    out["refused"] = refused;
    out["default_palette"] = from_std(theme->default_palette);
    out["roles"] = roles;
    out["groups"] = groups;
    out["tokens"] = tokens;
    out["reference_height_px"] = theme->tokens.reference_height_px;
    out["min_scale"] = theme->tokens.min_scale;
    out["palette_setting_key"] = from_std(theme->game.palette_setting.key);
    return out;
}

PackedStringArray RgUiTheme::validate_palette_text(const String& data_dir, const String& palette_json) {
    std::string err;
    const std::optional<rg::UiTheme> theme = rg::load_ui_theme(to_std(data_dir), &err);
    if (!theme) return strings({err});
    const std::optional<rg::UiPalette> palette = rg::parse_ui_palette(to_std(palette_json), "palette", &err);
    if (!palette) return strings({err});
    return strings(rg::validate_ui_palette(theme->contract, *palette));
}

void RgUiTheme::_bind_methods() {
    const godot::StringName cls = get_class_static();
    godot::ClassDB::bind_static_method(cls, godot::D_METHOD("load_theme", "data_dir"), &RgUiTheme::load_theme);
    godot::ClassDB::bind_static_method(cls, godot::D_METHOD("validate_palette_text", "data_dir", "palette_json"), &RgUiTheme::validate_palette_text);
}

} // namespace rg_godot
