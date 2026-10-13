// rg/render_focus.h - the render-side LOD focus for fast driving (owner
// complaint 2026-10-13: at 400-580 km/h the terrain does not resolve in time).
//
// The terrain view, the road ribbons and the buildings stream around ONE
// "render focus" point. For a drive camera that point must not be the camera
// itself but a point AHEAD of it along the car's velocity, otherwise the car
// outruns the finest level: L0 (1 m grid) only reaches ~250 m around the
// focus, i.e. 1.5 s of driving at 162 m/s, and the streamer needs 0.5-2 s per
// diff.
//
//   focus = p + v_hat * min(|v| * lead_s, cap_m)
//
// Engine-neutral (no Godot type): the velocity is the SIM CHASSIS' velocity in
// the session frame, never a camera delta (a camera delta spikes on a
// floating-origin rebase or a camera switch). The lead VECTOR is smoothed with
// a first-order filter (tau smooth_tau_s) so the focus does not jump when the
// car brakes, reverses or the rig changes; it decays to 0 when the active rig
// does not follow the car (free cam, drone, on foot).
#pragma once

#include <string_view>

namespace rg {

struct RenderFocusParams {
    double lead_s = 1.0;        // look-ahead time at |v|
    double cap_m = 150.0;       // longest lead (keeps the car >= ~90 m inside L0)
    double smooth_tau_s = 0.35; // lead vector low-pass time constant
    double max_dt_s = 0.25;     // a longer frame is treated as this long
};

struct RenderFocus {
    double x = 0.0; // session-local focus point
    double y = 0.0;
    double lead_x = 0.0; // the (smoothed) lead vector that was added to p
    double lead_y = 0.0;
};

// The un-smoothed lead vector for a velocity (0 for a non-finite velocity).
void render_focus_target_lead(const RenderFocusParams& params, double vx, double vy, double& lead_x, double& lead_y);

class RenderFocusTracker {
public:
    RenderFocusTracker() = default;
    explicit RenderFocusTracker(const RenderFocusParams& params) : params_(params) {}

    // (px, py): the camera / car position, (vx, vy): the chassis velocity,
    // both session-local; dt in seconds; `leads` = the active rig follows the
    // player car. Returns the focus to feed the streamers.
    RenderFocus update(double px, double py, double vx, double vy, double dt, bool leads);
    // Drops the smoothed lead (world switch, relocate, rig teleport).
    void reset();
    [[nodiscard]] double lead_length_m() const;
    [[nodiscard]] const RenderFocusParams& params() const { return params_; }
    void set_params(const RenderFocusParams& params) { params_ = params; }

private:
    RenderFocusParams params_{};
    double lead_x_ = 0.0;
    double lead_y_ = 0.0;
};

// True for the camera rigs that sit on / around the player car and move with
// it: "chase", "bumper", "cockpit", "orbit" (and the VR cockpit). False for
// free cam, drone, walker, cinematic and anything unknown.
[[nodiscard]] bool rig_leads_focus(std::string_view rig_name);

} // namespace rg
