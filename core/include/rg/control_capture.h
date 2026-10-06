// rg/control_capture.h - the "press the input you want to bind" logic of the controls
// screen (PLAN.md R5b), engine-neutral: BindingCapture turns a stream of raw input
// snapshots into one Binding (or a cancel), AxisCalibrator learns the travel of an axis.
//
// BindingCapture rules:
//   - inputs already held when the capture starts do not count until released and pressed again
//     (the click on the "Bind" button itself must not bind the left mouse button);
//   - Esc always cancels; on a pad B cancels on a short press and binds on a hold of
//     `hold_to_bind_s` (B is a legal binding, the menu back action must still work while capturing);
//   - an analogue axis binds when it moved more than `axis_threshold` away from its resting value
//     at the start (noise and the small drift of a worn stick never reach it). The direction of
//     the motion decides the half of the axis (Positive/Negative); a pedal that rests at an
//     extreme (|rest| >= 0.8) is bound calibrated, rest -> opposite end, over the full span;
//   - which inputs qualify depends on the target: Delta (mouse look) takes mouse motion only,
//     the full-axis row of a signed action (sign 0) takes analogue axes only, everything else
//     a key / button / wheel / axis of the device being captured.
#pragma once

#include "rg/control_binding.h"
#include "rg/controls.h"

#include <set>
#include <string>
#include <vector>

namespace rg {

enum class CaptureState { Waiting, Captured, Cancelled };

struct CaptureTarget {
    DeviceClass device = DeviceClass::Keyboard;
    bool button_action = true;          // the action is a button (any input qualifies)
    AxisRange range = AxisRange::Unit;  // axis actions
    int sign = 0;                       // signed-axis row: -1 / +1, 0 = the full axis
};

struct CaptureSettings {
    float axis_threshold = 0.6F;
    float mouse_motion_threshold = 60.0F; // pixels of accumulated motion for a mouse-look axis
    double hold_to_bind_s = 0.8;          // pad B
};

class BindingCapture {
public:
    explicit BindingCapture(CaptureSettings s = {}) : settings_(s) {}

    // `now` is the current snapshot (the pad being captured is in.pads.front() when the target is a pad).
    void begin(const CaptureTarget& target, const InputSnapshot& now);
    CaptureState feed(const InputSnapshot& now, double dt);
    [[nodiscard]] CaptureState state() const { return state_; }
    [[nodiscard]] const Binding& result() const { return result_; }
    [[nodiscard]] const CaptureTarget& target() const { return target_; }
    // Seconds B has been held (0 when not), for a "hold B to bind it" progress hint.
    [[nodiscard]] double b_held() const { return b_held_; }

private:
    CaptureSettings settings_;
    CaptureTarget target_;
    CaptureState state_ = CaptureState::Cancelled;
    Binding result_;
    std::set<std::string> prev_keys_;
    std::set<int> prev_buttons_;
    std::set<int> prev_pad_buttons_;
    std::vector<float> rest_axes_;
    float motion_x_ = 0.0F;
    float motion_y_ = 0.0F;
    double b_held_ = 0.0;
    bool b_consumed_ = false;
};

// Learns the physical travel of one axis while the player moves it through its whole range.
class AxisCalibrator {
public:
    // `rest` is the value of the axis at rest (a pedal untouched, a stick centred).
    void begin(float rest);
    void feed(float value);
    [[nodiscard]] float rest() const { return rest_; }
    [[nodiscard]] float lowest() const { return lo_; }
    [[nodiscard]] float highest() const { return hi_; }
    // True once the axis moved far enough (travel >= 0.3) for a calibration to mean anything.
    [[nodiscard]] bool valid() const { return hi_ - lo_ >= 0.3F; }
    // Writes the calibration into `t`. `unit_pedal`: cal_min is the rest end and cal_max the far end (the
    // output runs 0..1 from rest to the end of the travel); otherwise cal_min / cal_max are the lowest /
    // highest raw value seen (a centred stick or wheel).
    bool apply(AxisTuning& t, bool unit_pedal) const;

private:
    float rest_ = 0.0F;
    float lo_ = 0.0F;
    float hi_ = 0.0F;
};

} // namespace rg
