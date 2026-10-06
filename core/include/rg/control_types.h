// rg/control_types.h - the small enums shared by the control schema, the
// bindings and the device profiles (PLAN.md R5b). Engine-neutral.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace rg {

enum class ActionKind { Button, Axis };
enum class ButtonMode { Edge, Held };
enum class AxisRange { Signed, Unit, Delta };
enum class ControlGroup { Driving, Camera, OnFoot, Menu, Global };

// Bit set of the modes an action is live in.
enum ControlMode : std::uint32_t {
    kModeDrive = 1u << 0,
    kModeFreeCam = 1u << 1,
    kModeDroneFollow = 1u << 2,
    kModeOnFoot = 1u << 3,
    kModeMenu = 1u << 4,
};
inline constexpr std::uint32_t kModeInWorld = kModeDrive | kModeFreeCam | kModeDroneFollow | kModeOnFoot;

enum class DeviceClass { Keyboard, Mouse, Gamepad, Wheel, Generic };

const char* to_string(ActionKind k);
const char* to_string(ControlGroup g);
const char* to_string(DeviceClass c);
// The section title shown in the controls screen ("Driving", "Camera", "On foot", "Menus", "Global").
const char* group_title(ControlGroup g);
std::optional<DeviceClass> device_class_from_name(const std::string& name);

} // namespace rg
