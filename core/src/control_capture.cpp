// rg/control_capture.cpp - see control_capture.h.
#include "rg/control_capture.h"

#include <algorithm>
#include <cmath>

namespace rg {

namespace {

constexpr int kPadB = 1;

const PadSnapshot* first_pad(const InputSnapshot& in) { return in.pads.empty() ? nullptr : &in.pads.front(); }

float side_sign(const CaptureTarget& t) { return t.sign != 0 ? static_cast<float>(t.sign) : 1.0F; }

} // namespace

void BindingCapture::begin(const CaptureTarget& target, const InputSnapshot& now) {
    target_ = target;
    state_ = CaptureState::Waiting;
    result_ = Binding{};
    prev_keys_ = std::set<std::string>(now.keys.begin(), now.keys.end());
    prev_buttons_ = std::set<int>(now.mouse_buttons.begin(), now.mouse_buttons.end());
    prev_pad_buttons_.clear();
    rest_axes_.clear();
    if (const PadSnapshot* p = first_pad(now)) {
        prev_pad_buttons_ = std::set<int>(p->buttons.begin(), p->buttons.end());
        rest_axes_ = p->axes;
    }
    motion_x_ = motion_y_ = 0.0F;
    b_held_ = 0.0;
    // B held while the capture starts is the confirm press that opened it: it must be released first.
    b_consumed_ = prev_pad_buttons_.count(kPadB) != 0;
}

CaptureState BindingCapture::feed(const InputSnapshot& now, double dt) {
    if (state_ != CaptureState::Waiting) return state_;
    const bool pad = target_.device == DeviceClass::Gamepad || target_.device == DeviceClass::Wheel || target_.device == DeviceClass::Generic;
    const bool delta = !target_.button_action && target_.range == AxisRange::Delta;
    const bool axis_row = !target_.button_action && target_.range == AxisRange::Signed && target_.sign == 0;
    const bool digital_ok = target_.button_action || !(delta || axis_row);

    const std::set<std::string> keys(now.keys.begin(), now.keys.end());
    const std::set<int> mouse(now.mouse_buttons.begin(), now.mouse_buttons.end());
    std::set<int> pad_buttons;
    const PadSnapshot* p = first_pad(now);
    if (p != nullptr) pad_buttons = std::set<int>(p->buttons.begin(), p->buttons.end());

    // Esc cancels whatever device is being captured.
    if (keys.count("Escape") != 0 && prev_keys_.count("Escape") == 0) {
        state_ = CaptureState::Cancelled;
        prev_keys_ = keys;
        return state_;
    }

    if (target_.device == DeviceClass::Keyboard) {
        if (digital_ok) {
            for (const std::string& k : keys) {
                if (prev_keys_.count(k) != 0 || k == "Escape") continue;
                result_ = Binding{};
                result_.type = BindingType::Key;
                result_.key = k;
                result_.sign = side_sign(target_);
                state_ = CaptureState::Captured;
                break;
            }
        }
    } else if (target_.device == DeviceClass::Mouse) {
        if (delta) {
            motion_x_ += std::fabs(now.mouse_dx);
            motion_y_ += std::fabs(now.mouse_dy);
            if (std::max(motion_x_, motion_y_) >= settings_.mouse_motion_threshold) {
                result_ = Binding{};
                result_.type = BindingType::MouseMotion;
                result_.index = motion_x_ >= motion_y_ ? 0 : 1;
                state_ = CaptureState::Captured;
            }
        } else if (digital_ok) {
            for (const int b : mouse) {
                if (prev_buttons_.count(b) != 0) continue;
                result_ = Binding{};
                result_.type = BindingType::MouseButton;
                result_.index = b;
                result_.sign = side_sign(target_);
                state_ = CaptureState::Captured;
                break;
            }
            if (state_ == CaptureState::Waiting) {
                const float wheel[4] = {now.wheel_up, now.wheel_down, now.wheel_left, now.wheel_right};
                for (int i = 0; i < 4; ++i) {
                    if (wheel[i] > 0.0F) {
                        result_ = Binding{};
                        result_.type = BindingType::MouseButton;
                        result_.index = 4 + i;
                        result_.sign = side_sign(target_);
                        state_ = CaptureState::Captured;
                        break;
                    }
                }
            }
        }
    } else if (pad && p != nullptr) {
        // B: a short press cancels, a hold binds.
        const bool b_down = pad_buttons.count(kPadB) != 0;
        if (b_down && !b_consumed_) {
            b_held_ += dt;
            if (b_held_ >= settings_.hold_to_bind_s && digital_ok) {
                result_ = Binding{};
                result_.type = BindingType::JoyButton;
                result_.index = kPadB;
                result_.sign = side_sign(target_);
                state_ = CaptureState::Captured;
                b_consumed_ = true;
            }
        } else if (!b_down) {
            if (b_held_ > 0.0 && !b_consumed_) state_ = CaptureState::Cancelled; // released before the hold time
            b_held_ = 0.0;
            b_consumed_ = false;
        }
        if (state_ == CaptureState::Waiting && digital_ok) {
            for (const int b : pad_buttons) {
                if (b == kPadB || prev_pad_buttons_.count(b) != 0) continue;
                result_ = Binding{};
                result_.type = BindingType::JoyButton;
                result_.index = b;
                result_.sign = side_sign(target_);
                state_ = CaptureState::Captured;
                break;
            }
        }
        if (state_ == CaptureState::Waiting && !delta) {
            int best = -1;
            float best_dev = settings_.axis_threshold;
            for (std::size_t a = 0; a < p->axes.size(); ++a) {
                const float rest = a < rest_axes_.size() ? rest_axes_[a] : 0.0F;
                const float dev = std::fabs(p->axes[a] - rest);
                if (dev >= best_dev) {
                    best_dev = dev;
                    best = static_cast<int>(a);
                }
            }
            if (best >= 0) {
                const std::size_t bi = static_cast<std::size_t>(best);
                const float rest = bi < rest_axes_.size() ? rest_axes_[bi] : 0.0F;
                const float value = p->axes[bi];
                result_ = Binding{};
                result_.type = BindingType::JoyAxis;
                result_.index = best;
                if (std::fabs(rest) >= 0.8F && target_.range != AxisRange::Signed) {
                    // A pedal resting at an extreme: calibrated from rest to the opposite end.
                    result_.span = AxisSpan::Full;
                    result_.tuning.calibrated = true;
                    result_.tuning.cal_min = rest;
                    result_.tuning.cal_max = -rest;
                } else if (axis_row) {
                    result_.span = AxisSpan::Full;
                } else {
                    result_.span = (value - rest) >= 0.0F ? AxisSpan::Positive : AxisSpan::Negative;
                    result_.sign = side_sign(target_);
                }
                state_ = CaptureState::Captured;
            }
        }
    }

    prev_keys_ = keys;
    prev_buttons_ = mouse;
    prev_pad_buttons_ = pad_buttons;
    return state_;
}

void AxisCalibrator::begin(float rest) {
    rest_ = rest;
    lo_ = hi_ = rest;
}

void AxisCalibrator::feed(float value) {
    lo_ = std::min(lo_, value);
    hi_ = std::max(hi_, value);
}

bool AxisCalibrator::apply(AxisTuning& t, bool unit_pedal) const {
    if (!valid()) return false;
    t.calibrated = true;
    if (unit_pedal) {
        t.cal_min = rest_;
        t.cal_max = (hi_ - rest_) >= (rest_ - lo_) ? hi_ : lo_;
    } else {
        t.cal_min = lo_;
        t.cal_max = hi_;
    }
    return true;
}

} // namespace rg
