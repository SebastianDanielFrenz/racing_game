// rg/control_schema.cpp - see control_schema.h.
#include "rg/control_schema.h"

#include "control_json.h"
#include "json_util.h"

#include <set>
#include <utility>

namespace rg {

namespace {

using detail::json;

bool fail(std::string* err, const std::string& origin, const std::string& message) {
    if (err != nullptr) *err = origin + ": " + message;
    return false;
}

bool group_from_name(const std::string& s, ControlGroup& out) {
    if (s == "driving") out = ControlGroup::Driving;
    else if (s == "camera") out = ControlGroup::Camera;
    else if (s == "on_foot") out = ControlGroup::OnFoot;
    else if (s == "menu") out = ControlGroup::Menu;
    else if (s == "global") out = ControlGroup::Global;
    else return false;
    return true;
}

bool mode_from_name(const std::string& s, std::uint32_t& out) {
    if (s == "drive") out = kModeDrive;
    else if (s == "free_cam") out = kModeFreeCam;
    else if (s == "drone_follow") out = kModeDroneFollow;
    else if (s == "on_foot") out = kModeOnFoot;
    else if (s == "menu") out = kModeMenu;
    else if (s == "in_world") out = kModeInWorld;
    else return false;
    return true;
}

} // namespace

const char* to_string(ActionKind k) { return k == ActionKind::Button ? "button" : "axis"; }

const char* to_string(ControlGroup g) {
    switch (g) {
        case ControlGroup::Driving: return "driving";
        case ControlGroup::Camera: return "camera";
        case ControlGroup::OnFoot: return "on_foot";
        case ControlGroup::Menu: return "menu";
        case ControlGroup::Global: return "global";
    }
    return "driving";
}

const char* group_title(ControlGroup g) {
    switch (g) {
        case ControlGroup::Driving: return "Driving";
        case ControlGroup::Camera: return "Camera and flying";
        case ControlGroup::OnFoot: return "On foot";
        case ControlGroup::Menu: return "Menus";
        case ControlGroup::Global: return "Global";
    }
    return "";
}

const char* to_string(DeviceClass c) {
    switch (c) {
        case DeviceClass::Keyboard: return "keyboard";
        case DeviceClass::Mouse: return "mouse";
        case DeviceClass::Gamepad: return "gamepad";
        case DeviceClass::Wheel: return "wheel";
        case DeviceClass::Generic: return "generic";
    }
    return "generic";
}

std::optional<DeviceClass> device_class_from_name(const std::string& name) {
    if (name == "keyboard") return DeviceClass::Keyboard;
    if (name == "mouse") return DeviceClass::Mouse;
    if (name == "gamepad") return DeviceClass::Gamepad;
    if (name == "wheel") return DeviceClass::Wheel;
    if (name == "generic") return DeviceClass::Generic;
    return std::nullopt;
}

int ActionSchema::index_of(const std::string& id) const {
    for (std::size_t i = 0; i < actions_.size(); ++i) {
        if (actions_[i].id == id) return static_cast<int>(i);
    }
    return -1;
}

const ActionDef* ActionSchema::find(const std::string& id) const {
    const int i = index_of(id);
    return i < 0 ? nullptr : &actions_[static_cast<std::size_t>(i)];
}

std::optional<ActionSchema> ActionSchema::parse(const std::string& json_text, const std::string& origin, std::string* err) {
    const std::optional<json> parsed = detail::parse_json(json_text);
    if (!parsed || !parsed->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    std::string format;
    if (!detail::get_string(*parsed, "format", format) || format != "rg.control_actions/1") {
        fail(err, origin, "\"format\" must be \"rg.control_actions/1\"");
        return std::nullopt;
    }
    const auto list = parsed->find("actions");
    if (list == parsed->end() || !list->is_array() || list->empty()) {
        fail(err, origin, "\"actions\" must be a non-empty array");
        return std::nullopt;
    }
    ActionSchema schema;
    std::set<std::string> ids;
    std::size_t index = 0;
    for (const json& e : *list) {
        const std::string where = "actions[" + std::to_string(index++) + "]";
        if (!e.is_object()) {
            fail(err, origin, where + " is not an object");
            return std::nullopt;
        }
        ActionDef a;
        if (!detail::get_string(e, "id", a.id) || a.id.empty()) {
            fail(err, origin, where + ": \"id\" missing");
            return std::nullopt;
        }
        if (!ids.insert(a.id).second) {
            fail(err, origin, where + ": duplicate id \"" + a.id + "\"");
            return std::nullopt;
        }
        if (!detail::get_string(e, "label", a.label) || a.label.empty()) {
            fail(err, origin, where + " (" + a.id + "): \"label\" missing");
            return std::nullopt;
        }
        detail::get_string(e, "help", a.help);
        std::string group;
        if (!detail::get_string(e, "group", group) || !group_from_name(group, a.group)) {
            fail(err, origin, where + " (" + a.id + "): unknown \"group\"");
            return std::nullopt;
        }
        std::string kind;
        if (!detail::get_string(e, "kind", kind) || (kind != "button" && kind != "axis")) {
            fail(err, origin, where + " (" + a.id + "): \"kind\" must be \"button\" or \"axis\"");
            return std::nullopt;
        }
        a.kind = kind == "button" ? ActionKind::Button : ActionKind::Axis;
        if (a.kind == ActionKind::Button) {
            std::string mode = "held";
            detail::get_string(e, "mode", mode);
            if (mode != "edge" && mode != "held") {
                fail(err, origin, where + " (" + a.id + "): \"mode\" must be \"edge\" or \"held\"");
                return std::nullopt;
            }
            a.button_mode = mode == "edge" ? ButtonMode::Edge : ButtonMode::Held;
        } else {
            std::string range;
            if (!detail::get_string(e, "range", range) || (range != "signed" && range != "unit" && range != "delta")) {
                fail(err, origin, where + " (" + a.id + "): \"range\" must be \"signed\", \"unit\" or \"delta\"");
                return std::nullopt;
            }
            a.range = range == "signed" ? AxisRange::Signed : (range == "unit" ? AxisRange::Unit : AxisRange::Delta);
            detail::get_string(e, "negative_label", a.negative_label);
            detail::get_string(e, "positive_label", a.positive_label);
            if (a.range == AxisRange::Signed && (a.negative_label.empty() || a.positive_label.empty())) {
                fail(err, origin, where + " (" + a.id + "): a signed axis needs \"negative_label\" and \"positive_label\"");
                return std::nullopt;
            }
        }
        const auto modes = e.find("modes");
        if (modes == e.end() || !modes->is_array() || modes->empty()) {
            fail(err, origin, where + " (" + a.id + "): \"modes\" must be a non-empty array");
            return std::nullopt;
        }
        for (const json& m : *modes) {
            std::uint32_t bit = 0;
            if (!m.is_string() || !mode_from_name(m.get<std::string>(), bit)) {
                fail(err, origin, where + " (" + a.id + "): unknown mode in \"modes\"");
                return std::nullopt;
            }
            a.modes |= bit;
        }
        const auto always = e.find("always");
        if (always != e.end()) {
            if (!always->is_array()) {
                fail(err, origin, where + " (" + a.id + "): \"always\" must be an array");
                return std::nullopt;
            }
            for (const json& b : *always) {
                Binding binding;
                std::string why;
                if (!detail::binding_from_json(b, binding, &why)) {
                    fail(err, origin, where + " (" + a.id + "): bad \"always\" binding: " + why);
                    return std::nullopt;
                }
                a.always.push_back(std::move(binding));
            }
        }
        schema.actions_.push_back(std::move(a));
    }
    return schema;
}

std::optional<ActionSchema> ActionSchema::load_file(const std::string& path, std::string* err) {
    const std::optional<std::string> text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot read file");
        return std::nullopt;
    }
    return parse(*text, path, err);
}

} // namespace rg
