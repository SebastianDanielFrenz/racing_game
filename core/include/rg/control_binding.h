// rg/control_binding.h - one binding of a physical input to an action and the
// axis pipeline that shapes an analogue input (PLAN.md R5b). Engine-neutral:
// no Godot type, keys are named by string ("W", "Space", "Left", "F5"), buttons
// and axes by the SDL / Godot standard numbering (A = 0, B = 1, X = 2, Y = 3,
// Back = 4, Guide = 5, Start = 6, left stick = 7, right stick = 8, LB = 9,
// RB = 10, D-pad up/down/left/right = 11/12/13/14; axes left X/Y = 0/1, right
// X/Y = 2/3, left trigger = 4, right trigger = 5; mouse buttons left/right/
// middle = 1/2/3, wheel up/down = 4/5).
//
// The axis pipeline (evaluate_axis_binding), in this order:
//   1. calibration: the raw value is mapped from [cal_min, cal_max] (the values the
//      device reports at the two ends of its physical travel - cal_min may be larger
//      than cal_max, which inverts it) to -1..1; without calibration the raw value is
//      used as it is. A pedal that rests at +1 and goes to -1 is cal_min = +1, cal_max = -1.
//   2. invert flips the sign.
//   3. span selects the part of the axis that feeds the action: Full (the whole -1..1),
//      Positive (only above zero) or Negative (only below zero, as a positive magnitude).
//      Two bindings of one axis with opposite halves split a combined throttle/brake
//      pedal axis into two actions.
//   4. deadzone and saturation: the output rises from 0 at |v| = deadzone to 1 at
//      |v| = saturation (so saturation < 1 reaches full output before the end of the travel).
//   5. curve is the response exponent (1 = linear, 2 = soft around the centre) and
//      sensitivity a final gain (clamped to 1).
// A Full span on a Unit action (a pedal axis -1..1) is first mapped to 0..1, and the
// deadzone then applies at the rest end instead of at the centre.
#pragma once

#include "rg/control_types.h"

#include <string>

namespace rg {

enum class BindingType { Key, MouseButton, MouseMotion, JoyButton, JoyAxis };
enum class AxisSpan { Full, Positive, Negative };

struct AxisTuning {
    float deadzone = 0.0F;
    float saturation = 1.0F;
    float curve = 1.0F;
    float sensitivity = 1.0F;
    bool invert = false;
    bool calibrated = false;
    float cal_min = -1.0F; // raw value at one end of the travel (output -1 / 0)
    float cal_max = 1.0F;  // raw value at the other end (output +1 / 1)

    bool operator==(const AxisTuning& o) const;
};

struct Binding {
    BindingType type = BindingType::Key;
    std::string key; // Key: "W", "Space", "Left" (physical key names)
    int index = 0;   // MouseButton: button; MouseMotion: 0 = x, 1 = y; JoyButton: button; JoyAxis: axis
    AxisSpan span = AxisSpan::Full; // JoyAxis / MouseMotion
    // Digital sources (key, button) on a Signed action push it to this side (+1 / -1); a
    // half-span axis likewise. Ignored for a Unit action and for a Full-span axis.
    float sign = 1.0F;
    AxisTuning tuning; // JoyAxis / MouseMotion

    bool operator==(const Binding& o) const;
    [[nodiscard]] bool is_analogue() const { return type == BindingType::JoyAxis || type == BindingType::MouseMotion; }
};

// Clamps a tuning into its valid ranges (deadzone 0..0.95, saturation deadzone+0.05..1, curve
// 0.2..5, sensitivity 0.1..5). A degenerate calibration (|max - min| < 0.1) is dropped.
AxisTuning sanitize_tuning(AxisTuning t);

// The pipeline above. `target` is the action's range: Signed -> -1..1, Unit -> 0..1; Delta (mouse
// motion) -> raw * sensitivity (invert flips), unbounded. `pre_shape`, when given, receives the
// value after steps 1-3 (calibrated, inverted, spanned; before deadzone/curve/sensitivity), in
// the action's own scale - the "raw" the camera rigs read (they apply their own deadzone).
float evaluate_axis_binding(const Binding& b, float raw, AxisRange target, float* pre_shape = nullptr);

// What a pressed digital source contributes to an action of range `target`: sign for Signed, 1 for
// Unit/Delta-less targets.
float digital_contribution(const Binding& b, AxisRange target);

// True when the two bindings are the same physical input (a key is the same key, an axis half
// overlaps another half of the same axis, ...). Two different halves of one axis are not the same input.
bool same_input(const Binding& a, const Binding& b);

// "W", "Space (-)", "Pad A", "Pad left stick X (+)", "Mouse wheel up", "Axis 6 (full, inverted)".
std::string describe(const Binding& b);
// "Pad A" / "Pad left stick X" / "Pad axis 6" without direction annotations.
std::string input_name(const Binding& b);

const char* to_string(BindingType t);
const char* to_string(AxisSpan s);

} // namespace rg
