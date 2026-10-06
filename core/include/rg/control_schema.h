// rg/control_schema.h - the one action schema of the game's input layer
// (PLAN.md R5b, data/controls/actions.json). Engine-neutral: every action the
// game reads from a device is listed here with its kind, the section it is
// shown under and the player modes it is live in. Nothing in the game may read
// an input that is not in this schema (input_map.gd asks rg::Controls for
// exactly these ids and for nothing else).
//
// Kinds:
//   Button  - a level the consumer reads either as a held state (ButtonMode::Held:
//             starter, camera fast, ...) or as rising edges it counts (ButtonMode::Edge:
//             shift up, ignition toggle, reset car, ...). The schema only records which;
//             the edge detection stays with the consumer.
//   Axis    - AxisRange::Signed  -1..1 (steer, camera move/look, positive_label/negative_label
//                                   name the two directions: "Left"/"Right"),
//             AxisRange::Unit     0..1 (throttle, brake, clutch, handbrake),
//             AxisRange::Delta    an unbounded relative amount per frame (mouse look, pixels).
//
// `modes` is the set of player modes the action is live in. Two actions on one
// device can only conflict when their modes overlap - that is what "bound to
// two actions that are active at the same time" means (W is throttle while
// driving and free-cam forward while flying; the same pad button may be a drive
// action and an on-foot action).
//
// The schema file is validated strictly: a duplicate id, an unknown group,
// kind or mode name, or an empty label fails the load (the game then refuses
// to start the controls layer rather than guess).
#pragma once

#include "rg/control_binding.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rg {

struct ActionDef {
    std::string id;    // "throttle"
    std::string label; // "Throttle"
    std::string help;  // optional one-liner
    ControlGroup group = ControlGroup::Driving;
    ActionKind kind = ActionKind::Button;
    ButtonMode button_mode = ButtonMode::Held; // Button
    AxisRange range = AxisRange::Unit;         // Axis
    std::string negative_label;                // signed axes: "Left"
    std::string positive_label;                // signed axes: "Right"
    std::uint32_t modes = 0;
    // Built-in aliases that can never be removed or rebound: the menu actions keep Enter /
    // Esc and pad A / B on top of whatever the player binds, so the menu can never be
    // rebound into a dead end. Empty for every other action.
    std::vector<Binding> always;
};

class ActionSchema {
public:
    [[nodiscard]] const std::vector<ActionDef>& actions() const { return actions_; }
    [[nodiscard]] std::size_t size() const { return actions_.size(); }
    // -1 when unknown.
    [[nodiscard]] int index_of(const std::string& id) const;
    [[nodiscard]] const ActionDef* find(const std::string& id) const;

    // Parses data/controls/actions.json text (format "rg.control_actions/1").
    static std::optional<ActionSchema> parse(const std::string& json_text, const std::string& origin, std::string* err);
    static std::optional<ActionSchema> load_file(const std::string& path, std::string* err);

private:
    std::vector<ActionDef> actions_;
};

} // namespace rg
