// rg/control_binding.cpp - see control_binding.h.
#include "rg/control_binding.h"

#include "control_json.h"

#include <utility>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace rg {

namespace {

float clampf(float v, float lo, float hi) { return std::min(std::max(v, lo), hi); }

// 0..1 magnitude -> shaped 0..1 (steps 4 and 5 of the pipeline).
float shape_magnitude(float v, const AxisTuning& t) {
    if (v <= t.deadzone) return 0.0F;
    float m = clampf((v - t.deadzone) / (t.saturation - t.deadzone), 0.0F, 1.0F);
    if (t.curve != 1.0F) m = std::pow(m, t.curve);
    return std::min(1.0F, m * t.sensitivity);
}

const char* pad_button_name(int b) {
    switch (b) {
        case 0: return "A";
        case 1: return "B";
        case 2: return "X";
        case 3: return "Y";
        case 4: return "Back";
        case 5: return "Guide";
        case 6: return "Start";
        case 7: return "left stick click";
        case 8: return "right stick click";
        case 9: return "LB";
        case 10: return "RB";
        case 11: return "D-pad up";
        case 12: return "D-pad down";
        case 13: return "D-pad left";
        case 14: return "D-pad right";
        default: return nullptr;
    }
}

const char* pad_axis_name(int a) {
    switch (a) {
        case 0: return "left stick X";
        case 1: return "left stick Y";
        case 2: return "right stick X";
        case 3: return "right stick Y";
        case 4: return "left trigger";
        case 5: return "right trigger";
        default: return nullptr;
    }
}

const char* mouse_button_name(int b) {
    switch (b) {
        case 1: return "left button";
        case 2: return "right button";
        case 3: return "middle button";
        case 4: return "wheel up";
        case 5: return "wheel down";
        case 6: return "wheel left";
        case 7: return "wheel right";
        case 8: return "side button 1";
        case 9: return "side button 2";
        default: return nullptr;
    }
}

bool spans_overlap(AxisSpan a, AxisSpan b) { return a == AxisSpan::Full || b == AxisSpan::Full || a == b; }

} // namespace

const char* to_string(BindingType t) {
    switch (t) {
        case BindingType::Key: return "key";
        case BindingType::MouseButton: return "mouse_button";
        case BindingType::MouseMotion: return "mouse_motion";
        case BindingType::JoyButton: return "joy_button";
        case BindingType::JoyAxis: return "joy_axis";
    }
    return "key";
}

const char* to_string(AxisSpan s) {
    switch (s) {
        case AxisSpan::Full: return "full";
        case AxisSpan::Positive: return "positive";
        case AxisSpan::Negative: return "negative";
    }
    return "full";
}

bool AxisTuning::operator==(const AxisTuning& o) const {
    return deadzone == o.deadzone && saturation == o.saturation && curve == o.curve && sensitivity == o.sensitivity &&
           invert == o.invert && calibrated == o.calibrated && (!calibrated || (cal_min == o.cal_min && cal_max == o.cal_max));
}

bool Binding::operator==(const Binding& o) const {
    if (type != o.type || key != o.key || index != o.index || sign != o.sign) return false;
    if (is_analogue()) return span == o.span && tuning == o.tuning;
    return true;
}

AxisTuning sanitize_tuning(AxisTuning t) {
    if (!std::isfinite(t.deadzone)) t.deadzone = 0.0F;
    if (!std::isfinite(t.saturation)) t.saturation = 1.0F;
    if (!std::isfinite(t.curve)) t.curve = 1.0F;
    if (!std::isfinite(t.sensitivity)) t.sensitivity = 1.0F;
    t.deadzone = clampf(t.deadzone, 0.0F, 0.95F);
    t.saturation = clampf(t.saturation, t.deadzone + 0.05F, 1.0F);
    t.curve = clampf(t.curve, 0.2F, 5.0F);
    t.sensitivity = clampf(t.sensitivity, 0.1F, 5.0F);
    if (t.calibrated) {
        if (!std::isfinite(t.cal_min) || !std::isfinite(t.cal_max) || std::fabs(t.cal_max - t.cal_min) < 0.1F) {
            t.calibrated = false;
            t.cal_min = -1.0F;
            t.cal_max = 1.0F;
        }
    }
    return t;
}

float evaluate_axis_binding(const Binding& b, float raw, AxisRange target, float* pre_shape) {
    if (!std::isfinite(raw)) raw = 0.0F;
    const AxisTuning t = sanitize_tuning(b.tuning);
    if (target == AxisRange::Delta) {
        const float v = raw * (t.invert ? -1.0F : 1.0F);
        if (pre_shape != nullptr) *pre_shape = v;
        return v * t.sensitivity;
    }
    float s = clampf(raw, -1.0F, 1.0F);
    if (t.calibrated) s = clampf(2.0F * (raw - t.cal_min) / (t.cal_max - t.cal_min) - 1.0F, -1.0F, 1.0F);
    if (t.invert) s = -s;
    if (b.span == AxisSpan::Full) {
        if (target == AxisRange::Unit) {
            const float u = (s + 1.0F) * 0.5F;
            if (pre_shape != nullptr) *pre_shape = u;
            return shape_magnitude(u, t);
        }
        if (pre_shape != nullptr) *pre_shape = s;
        const float m = shape_magnitude(std::fabs(s), t);
        return s < 0.0F ? -m : m;
    }
    const float half = b.span == AxisSpan::Positive ? std::max(s, 0.0F) : std::max(-s, 0.0F);
    const float m = shape_magnitude(half, t);
    if (target == AxisRange::Signed) {
        if (pre_shape != nullptr) *pre_shape = b.sign * half;
        return b.sign * m;
    }
    if (pre_shape != nullptr) *pre_shape = half;
    return m;
}

float digital_contribution(const Binding& b, AxisRange target) { return target == AxisRange::Signed ? b.sign : 1.0F; }

bool same_input(const Binding& a, const Binding& b) {
    if (a.type != b.type) return false;
    switch (a.type) {
        case BindingType::Key: return a.key == b.key;
        case BindingType::MouseButton:
        case BindingType::JoyButton: return a.index == b.index;
        case BindingType::MouseMotion:
        case BindingType::JoyAxis: return a.index == b.index && spans_overlap(a.span, b.span);
    }
    return false;
}

std::string input_name(const Binding& b) {
    switch (b.type) {
        case BindingType::Key: return b.key;
        case BindingType::MouseButton: {
            const char* n = mouse_button_name(b.index);
            return n != nullptr ? std::string("Mouse ") + n : "Mouse button " + std::to_string(b.index);
        }
        case BindingType::MouseMotion: return b.index == 0 ? "Mouse X" : "Mouse Y";
        case BindingType::JoyButton: {
            const char* n = pad_button_name(b.index);
            return n != nullptr ? std::string("Pad ") + n : "Pad button " + std::to_string(b.index);
        }
        case BindingType::JoyAxis: {
            const char* n = pad_axis_name(b.index);
            return n != nullptr ? std::string("Pad ") + n : "Axis " + std::to_string(b.index);
        }
    }
    return "?";
}

std::string describe(const Binding& b) {
    std::string out = input_name(b);
    if (b.is_analogue()) {
        std::string note;
        if (b.span == AxisSpan::Positive) note = "+ half";
        else if (b.span == AxisSpan::Negative) note = "- half";
        if (b.tuning.invert) note += note.empty() ? "inverted" : ", inverted";
        if (b.tuning.calibrated) note += note.empty() ? "calibrated" : ", calibrated";
        if (b.span != AxisSpan::Full && b.sign < 0.0F) note += note.empty() ? "pushes -" : ", pushes -";
        if (!note.empty()) out += " (" + note + ")";
    } else if (b.sign < 0.0F) {
        out += " (-)";
    }
    return out;
}

namespace detail {

using nlohmann::json;

namespace {

bool read_int(const json& o, const char* key, int lo, int hi, int& out, std::string* err) {
    const auto it = o.find(key);
    if (it == o.end()) {
        if (err != nullptr) *err = std::string("\"") + key + "\" missing";
        return false;
    }
    if (!it->is_number_integer()) {
        if (err != nullptr) *err = std::string("\"") + key + "\" must be an integer";
        return false;
    }
    const long long v = it->get<long long>();
    if (v < lo || v > hi) {
        if (err != nullptr) *err = std::string("\"") + key + "\" out of range";
        return false;
    }
    out = static_cast<int>(v);
    return true;
}

// Optional finite number; false only for a present but non-numeric / non-finite value.
bool read_optional_number(const json& o, const char* key, float& out, bool& present, std::string* err) {
    present = false;
    const auto it = o.find(key);
    if (it == o.end()) return true;
    if (!it->is_number()) {
        if (err != nullptr) *err = std::string("\"") + key + "\" must be a number";
        return false;
    }
    const double v = it->get<double>();
    if (!std::isfinite(v)) {
        if (err != nullptr) *err = std::string("\"") + key + "\" must be finite";
        return false;
    }
    out = static_cast<float>(v);
    present = true;
    return true;
}

} // namespace

json binding_to_json(const Binding& b) {
    json j = json::object();
    j["type"] = to_string(b.type);
    switch (b.type) {
        case BindingType::Key: j["key"] = b.key; break;
        case BindingType::MouseButton:
        case BindingType::JoyButton: j["button"] = b.index; break;
        case BindingType::MouseMotion:
        case BindingType::JoyAxis: j["axis"] = b.index; break;
    }
    if (b.sign != 1.0F) j["sign"] = b.sign;
    if (b.is_analogue()) {
        if (b.span != AxisSpan::Full) j["span"] = to_string(b.span);
        const AxisTuning& t = b.tuning;
        if (t.deadzone != 0.0F) j["deadzone"] = t.deadzone;
        if (t.saturation != 1.0F) j["saturation"] = t.saturation;
        if (t.curve != 1.0F) j["curve"] = t.curve;
        if (t.sensitivity != 1.0F) j["sensitivity"] = t.sensitivity;
        if (t.invert) j["invert"] = true;
        if (t.calibrated) {
            j["cal_min"] = t.cal_min;
            j["cal_max"] = t.cal_max;
        }
    }
    return j;
}

bool binding_from_json(const json& j, Binding& out, std::string* err) {
    if (!j.is_object()) {
        if (err != nullptr) *err = "not an object";
        return false;
    }
    Binding b;
    const auto type_it = j.find("type");
    if (type_it == j.end() || !type_it->is_string()) {
        if (err != nullptr) *err = "\"type\" missing";
        return false;
    }
    const std::string type = type_it->get<std::string>();
    if (type == "key") {
        b.type = BindingType::Key;
        const auto it = j.find("key");
        if (it == j.end() || !it->is_string() || it->get<std::string>().empty() || it->get<std::string>().size() > 32) {
            if (err != nullptr) *err = "\"key\" must be a name of 1-32 characters";
            return false;
        }
        b.key = it->get<std::string>();
    } else if (type == "mouse_button") {
        b.type = BindingType::MouseButton;
        if (!read_int(j, "button", 1, 16, b.index, err)) return false;
    } else if (type == "joy_button") {
        b.type = BindingType::JoyButton;
        if (!read_int(j, "button", 0, 63, b.index, err)) return false;
    } else if (type == "mouse_motion") {
        b.type = BindingType::MouseMotion;
        if (!read_int(j, "axis", 0, 1, b.index, err)) return false;
    } else if (type == "joy_axis") {
        b.type = BindingType::JoyAxis;
        if (!read_int(j, "axis", 0, 15, b.index, err)) return false;
    } else {
        if (err != nullptr) *err = "unknown binding type \"" + type + "\"";
        return false;
    }
    bool present = false;
    float sign = 1.0F;
    if (!read_optional_number(j, "sign", sign, present, err)) return false;
    if (present) {
        if (sign != 1.0F && sign != -1.0F) {
            if (err != nullptr) *err = "\"sign\" must be 1 or -1";
            return false;
        }
        b.sign = sign;
    }
    if (b.is_analogue()) {
        const auto span_it = j.find("span");
        if (span_it != j.end()) {
            if (!span_it->is_string()) {
                if (err != nullptr) *err = "\"span\" must be a string";
                return false;
            }
            const std::string span = span_it->get<std::string>();
            if (span == "full") b.span = AxisSpan::Full;
            else if (span == "positive") b.span = AxisSpan::Positive;
            else if (span == "negative") b.span = AxisSpan::Negative;
            else {
                if (err != nullptr) *err = "unknown span \"" + span + "\"";
                return false;
            }
        }
        AxisTuning t;
        float v = 0.0F;
        if (!read_optional_number(j, "deadzone", v, present, err)) return false;
        if (present) t.deadzone = v;
        if (!read_optional_number(j, "saturation", v, present, err)) return false;
        if (present) t.saturation = v;
        if (!read_optional_number(j, "curve", v, present, err)) return false;
        if (present) t.curve = v;
        if (!read_optional_number(j, "sensitivity", v, present, err)) return false;
        if (present) t.sensitivity = v;
        const auto inv = j.find("invert");
        if (inv != j.end()) {
            if (!inv->is_boolean()) {
                if (err != nullptr) *err = "\"invert\" must be a boolean";
                return false;
            }
            t.invert = inv->get<bool>();
        }
        bool has_min = false;
        bool has_max = false;
        float lo = -1.0F;
        float hi = 1.0F;
        if (!read_optional_number(j, "cal_min", lo, has_min, err)) return false;
        if (!read_optional_number(j, "cal_max", hi, has_max, err)) return false;
        if (has_min != has_max) {
            if (err != nullptr) *err = "\"cal_min\" and \"cal_max\" go together";
            return false;
        }
        if (has_min) {
            t.calibrated = true;
            t.cal_min = lo;
            t.cal_max = hi;
        }
        b.tuning = sanitize_tuning(t);
    }
    out = std::move(b);
    return true;
}

} // namespace detail

} // namespace rg
