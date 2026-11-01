// rg_ui_theme.h — thin Godot face of rg/ui_theme.h: hands the loaded design
// language to game/scripts/ui/ui_theme.gd, which builds the Godot Theme and
// answers role lookups. The rules live in rg_core.
#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/string.hpp>

namespace rg_godot {

class RgUiTheme : public godot::RefCounted {
    GDCLASS(RgUiTheme, godot::RefCounted)

public:
    // {ok, error, palettes: {id: {label, claims, colours: {role: Color}}}, ids (accepted, in menu order),
    //  refused: [{id, problems}], default_palette, roles: [{id, group, meaning}], groups: [{id, title}],
    //  tokens: {dotted name: float}, reference_height_px, min_scale, palette_setting_key}
    static godot::Dictionary load_theme(const godot::String& data_dir);
    // The problems of a palette file against the contract in data_dir; empty = accepted.
    static godot::PackedStringArray validate_palette_text(const godot::String& data_dir, const godot::String& palette_json);

protected:
    static void _bind_methods();
};

} // namespace rg_godot
