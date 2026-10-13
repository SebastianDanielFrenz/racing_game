#include "rg_render_focus.h"

#include "rg/terrain_stream.h"

#include <godot_cpp/core/class_db.hpp>

#include <array>
#include <string>
#include <vector>

namespace rg_godot {

using godot::Dictionary;
using godot::String;

void RgRenderFocus::set_lead_s(double lead_s) {
    rg::RenderFocusParams p = tracker_.params();
    p.lead_s = lead_s > 0.0 ? lead_s : 0.0;
    tracker_.set_params(p);
}

void RgRenderFocus::set_cap_m(double cap_m) {
    rg::RenderFocusParams p = tracker_.params();
    p.cap_m = cap_m > 0.0 ? cap_m : 0.0;
    tracker_.set_params(p);
}

void RgRenderFocus::set_smooth_tau_s(double tau_s) {
    rg::RenderFocusParams p = tracker_.params();
    p.smooth_tau_s = tau_s > 0.0 ? tau_s : 0.0;
    tracker_.set_params(p);
}

Dictionary RgRenderFocus::update(double px, double py, double vx, double vy, double dt, bool leads) {
    const rg::RenderFocus f = tracker_.update(px, py, vx, vy, dt, leads);
    Dictionary d;
    d["x"] = f.x;
    d["y"] = f.y;
    d["lead_x"] = f.lead_x;
    d["lead_y"] = f.lead_y;
    return d;
}

bool RgRenderFocus::rig_leads_focus(const String& rig_name) {
    const godot::CharString utf8 = rig_name.utf8();
    return rg::rig_leads_focus(std::string(utf8.get_data()));
}

double RgRenderFocus::grid_coverage_ahead(const godot::PackedInt32Array& tiles, double tile_m, double px, double py,
                                          double dx, double dy, double max_m, double step_m) {
    std::vector<std::array<int, 2>> keys;
    keys.reserve(static_cast<std::size_t>(tiles.size() / 2));
    for (int i = 0; i + 1 < tiles.size(); i += 2) keys.push_back({tiles[i], tiles[i + 1]});
    return rg::grid_coverage_ahead(keys, tile_m, px, py, dx, dy, max_m, step_m);
}

void RgRenderFocus::_bind_methods() {
    godot::ClassDB::bind_method(godot::D_METHOD("set_lead_s", "lead_s"), &RgRenderFocus::set_lead_s);
    godot::ClassDB::bind_method(godot::D_METHOD("get_lead_s"), &RgRenderFocus::get_lead_s);
    godot::ClassDB::bind_method(godot::D_METHOD("set_cap_m", "cap_m"), &RgRenderFocus::set_cap_m);
    godot::ClassDB::bind_method(godot::D_METHOD("get_cap_m"), &RgRenderFocus::get_cap_m);
    godot::ClassDB::bind_method(godot::D_METHOD("set_smooth_tau_s", "tau_s"), &RgRenderFocus::set_smooth_tau_s);
    godot::ClassDB::bind_method(godot::D_METHOD("get_smooth_tau_s"), &RgRenderFocus::get_smooth_tau_s);
    godot::ClassDB::bind_method(godot::D_METHOD("update", "px", "py", "vx", "vy", "dt", "leads"), &RgRenderFocus::update);
    godot::ClassDB::bind_method(godot::D_METHOD("reset"), &RgRenderFocus::reset);
    godot::ClassDB::bind_method(godot::D_METHOD("get_lead_length_m"), &RgRenderFocus::get_lead_length_m);
    godot::ClassDB::bind_static_method("RgRenderFocus", godot::D_METHOD("rig_leads_focus", "rig_name"),
                                       &RgRenderFocus::rig_leads_focus);
    godot::ClassDB::bind_static_method("RgRenderFocus",
                                       godot::D_METHOD("grid_coverage_ahead", "tiles", "tile_m", "px", "py", "dx", "dy", "max_m", "step_m"),
                                       &RgRenderFocus::grid_coverage_ahead);
}

} // namespace rg_godot
