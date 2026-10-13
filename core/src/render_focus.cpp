// rg/render_focus.cpp - see rg/render_focus.h.
#include "rg/render_focus.h"

#include <algorithm>
#include <cmath>

namespace rg {

void render_focus_target_lead(const RenderFocusParams& params, double vx, double vy, double& lead_x, double& lead_y) {
    lead_x = 0.0;
    lead_y = 0.0;
    if (!std::isfinite(vx) || !std::isfinite(vy)) {
        return;
    }
    const double speed = std::hypot(vx, vy);
    if (speed < 1e-6) {
        return;
    }
    const double length = std::min(speed * params.lead_s, params.cap_m);
    lead_x = vx / speed * length;
    lead_y = vy / speed * length;
}

RenderFocus RenderFocusTracker::update(double px, double py, double vx, double vy, double dt, bool leads) {
    double tx = 0.0;
    double ty = 0.0;
    if (leads) {
        render_focus_target_lead(params_, vx, vy, tx, ty);
    }
    const double step = std::clamp(std::isfinite(dt) ? dt : 0.0, 0.0, params_.max_dt_s);
    const double alpha = params_.smooth_tau_s <= 0.0 ? 1.0 : 1.0 - std::exp(-step / params_.smooth_tau_s);
    lead_x_ += (tx - lead_x_) * alpha;
    lead_y_ += (ty - lead_y_) * alpha;
    RenderFocus out;
    out.lead_x = lead_x_;
    out.lead_y = lead_y_;
    out.x = px + lead_x_;
    out.y = py + lead_y_;
    return out;
}

void RenderFocusTracker::reset() {
    lead_x_ = 0.0;
    lead_y_ = 0.0;
}

double RenderFocusTracker::lead_length_m() const { return std::hypot(lead_x_, lead_y_); }

bool rig_leads_focus(std::string_view rig_name) {
    return rig_name == "chase" || rig_name == "bumper" || rig_name == "cockpit" || rig_name == "orbit" ||
           rig_name == "xr_cockpit";
}

} // namespace rg
