// godot_ext/src/rg_render_focus.h - RgRenderFocus: the thin Godot binding of
// rg/render_focus.h (velocity-led render focus for the terrain view, the road
// ribbons and the buildings) and of the road-ribbon coverage helper
// rg::grid_coverage_ahead. All numbers are computed in rg_core.
#pragma once

#include "rg/render_focus.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/string.hpp>

namespace rg_godot {

class RgRenderFocus : public godot::RefCounted {
    GDCLASS(RgRenderFocus, godot::RefCounted)

public:
    // Tunables (defaults: 1 s, 150 m, tau 0.35 s).
    void set_lead_s(double lead_s);
    void set_cap_m(double cap_m);
    void set_smooth_tau_s(double tau_s);
    [[nodiscard]] double get_lead_s() const { return tracker_.params().lead_s; }
    [[nodiscard]] double get_cap_m() const { return tracker_.params().cap_m; }
    [[nodiscard]] double get_smooth_tau_s() const { return tracker_.params().smooth_tau_s; }

    // {x, y, lead_x, lead_y}: the focus for the car at (px, py) moving with
    // (vx, vy), all session-local; `leads` = the active rig follows the car.
    [[nodiscard]] godot::Dictionary update(double px, double py, double vx, double vy, double dt, bool leads);
    void reset() { tracker_.reset(); }
    [[nodiscard]] double get_lead_length_m() const { return tracker_.lead_length_m(); }

    // rg::rig_leads_focus.
    static bool rig_leads_focus(const godot::String& rig_name);
    // rg::grid_coverage_ahead: `tiles` is a flat (ix, iy, ix, iy, ...) array.
    static double grid_coverage_ahead(const godot::PackedInt32Array& tiles, double tile_m, double px, double py,
                                      double dx, double dy, double max_m, double step_m);

protected:
    static void _bind_methods();

private:
    rg::RenderFocusTracker tracker_;
};

} // namespace rg_godot
