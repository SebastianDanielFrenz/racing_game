// rg_damage_convert.h - rg::DamageView -> godot::Dictionary, shared by RgGarage and RgSimulation (conversion only).
#pragma once

#include "rg/damage.h"

#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>

namespace rg_godot {

// {present, any_damaged, warning (the HUD line, "" when healthy), radiator_ua_scale, fan_flow_scale,
//  parts: [{id, label, description, health (0..1), damaged, effect ("radiator_ua" | "fan_flow" | "none"), effect_scale}]}
inline godot::Dictionary damage_view_to_dictionary(const rg::DamageView& view, bool present) {
    const auto str = [](const std::string& s) { return godot::String::utf8(s.c_str()); };
    godot::Dictionary d;
    d["present"] = present;
    d["any_damaged"] = view.any_damaged;
    d["warning"] = str(rg::damage_warning_line(view));
    d["radiator_ua_scale"] = view.scales.radiator_ua;
    d["fan_flow_scale"] = view.scales.fan_flow;
    godot::Array parts;
    for (const rg::DamagePartView& p : view.parts) {
        godot::Dictionary e;
        e["id"] = str(p.id);
        e["label"] = str(p.label);
        e["description"] = str(p.description);
        e["health"] = p.health;
        e["damaged"] = p.damaged;
        e["effect"] = str(rg::to_string(p.effect));
        e["effect_scale"] = p.effect_scale;
        parts.push_back(e);
    }
    d["parts"] = parts;
    return d;
}

} // namespace rg_godot
