// control_json.h - internal: JSON (de)serialisation of one binding, shared by the
// action schema (built-in alias bindings), the device defaults and the user file
// (controls.json). See control_binding.h for the meaning of the fields.
//
//   {"type": "key", "key": "W", "sign": -1}
//   {"type": "mouse_button", "button": 2}
//   {"type": "mouse_motion", "axis": 0, "sensitivity": 1.0, "invert": false}
//   {"type": "joy_button", "button": 0}
//   {"type": "joy_axis", "axis": 0, "span": "full", "deadzone": 0.08, "saturation": 1.0,
//    "curve": 1.0, "sensitivity": 1.0, "invert": false, "cal_min": -1.0, "cal_max": 1.0}
// Defaults are not written. A binding that does not parse is rejected as a whole.
#pragma once

#include "rg/control_binding.h"

#include <nlohmann/json.hpp>

#include <string>

namespace rg::detail {

nlohmann::json binding_to_json(const Binding& b);
// False and *err set (a short reason) when the object is not a valid binding.
bool binding_from_json(const nlohmann::json& j, Binding& out, std::string* err);

} // namespace rg::detail
