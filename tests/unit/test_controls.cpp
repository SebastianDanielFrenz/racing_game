// test_controls.cpp - rg::Controls (PLAN.md R5b): the action schema, the shipped default profiles
// (frozen against the bindings the game had before the controls menu), device identity and
// profile resolution (SDL GUID + ordinal), persistence (round trip, broken / unknown-version
// file -> defaults + .bak), conflict detection, the axis pipeline incl. combined pedals and the
// evaluation of raw input. Each test names the sabotage that makes it fail ("sabotage:").
#include "rg/controls.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

using Catch::Approx;
using rg::AxisSpan;
using rg::Binding;
using rg::BindingType;
using rg::Controls;
using rg::InputSnapshot;
using rg::PadSnapshot;

constexpr const char* kGuidA = "030000005e040000ea02000000007801"; // an Xbox-style pad
constexpr const char* kGuidB = "03000000c82d00000160000000007801"; // another model

std::string data_dir() { return std::string(RG_SOURCE_DIR) + "/data"; }

Controls make_controls() {
    Controls c;
    std::string err;
    INFO(err);
    REQUIRE(c.load_data(data_dir(), &err));
    return c;
}

std::string temp_path(const std::string& name) {
    return (std::filesystem::temp_directory_path() / ("rg_controls_test_" + name)).string();
}

std::string read_all(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void write_all(const std::string& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << text;
}

// A compact, exact text form of a binding for the frozen tables below.
std::string fmt(const Binding& b) {
    std::ostringstream o;
    switch (b.type) {
        case BindingType::Key: o << "key:" << b.key; if (b.sign != 1.0F) o << ":" << static_cast<int>(b.sign); break;
        case BindingType::MouseButton: o << "mouse:" << b.index; break;
        case BindingType::MouseMotion: o << "motion:" << b.index; break;
        case BindingType::JoyButton: o << "btn:" << b.index; if (b.sign != 1.0F) o << ":" << static_cast<int>(b.sign); break;
        case BindingType::JoyAxis:
            o << "axis:" << b.index << ":" << rg::to_string(b.span);
            if (b.tuning.deadzone > 0.0F) o << ":dz" << b.tuning.deadzone;
            if (b.tuning.invert) o << ":inv";
            if (b.span != AxisSpan::Full && b.sign != 1.0F) o << ":s" << static_cast<int>(b.sign);
            break;
    }
    return o.str();
}

std::string fmt_list(const std::vector<Binding>& v) {
    std::string out;
    for (const Binding& b : v) out += (out.empty() ? "" : ", ") + fmt(b);
    return out;
}

Binding key(const std::string& name, float sign = 1.0F) {
    Binding b;
    b.type = BindingType::Key;
    b.key = name;
    b.sign = sign;
    return b;
}

Binding joy_button(int i) {
    Binding b;
    b.type = BindingType::JoyButton;
    b.index = i;
    return b;
}

Binding joy_axis(int i, AxisSpan span = AxisSpan::Full) {
    Binding b;
    b.type = BindingType::JoyAxis;
    b.index = i;
    b.span = span;
    return b;
}

InputSnapshot snap_keys(std::vector<std::string> keys) {
    InputSnapshot s;
    s.keys = std::move(keys);
    return s;
}

PadSnapshot pad(int slot, std::vector<int> buttons, std::vector<float> axes) {
    PadSnapshot p;
    p.slot = slot;
    p.buttons = std::move(buttons);
    p.axes = std::move(axes);
    return p;
}

float value(const Controls& c, const rg::ActionState& s, const std::string& action) {
    const int i = c.schema().index_of(action);
    REQUIRE(i >= 0);
    return s.value[static_cast<std::size_t>(i)];
}

} // namespace

// ---- schema and defaults ------------------------------------------------------------------------

TEST_CASE("controls: the shipped schema and default profiles load and are consistent", "[controls]") {
    Controls c = make_controls();
    CHECK(c.ready());
    CHECK(c.schema().size() == 35);
    // Every action has a label, every signed axis names both directions.
    for (const rg::ActionDef& a : c.schema().actions()) {
        CAPTURE(a.id);
        CHECK_FALSE(a.label.empty());
        CHECK(a.modes != 0);
        if (a.kind == rg::ActionKind::Axis && a.range == rg::AxisRange::Signed) {
            CHECK_FALSE(a.negative_label.empty());
            CHECK_FALSE(a.positive_label.empty());
        }
    }
    // The shipped defaults conflict with nothing (sharing across modes - RB, Space, G, A - is legal).
    c.joypad_connected(0, kGuidA, "Xbox Controller", 0x045e, 0x02ea, true);
    for (const rg::DeviceInfo& d : c.devices()) {
        CAPTURE(d.key);
        CHECK(c.conflicts(d.key).empty());
    }
    // sabotage: giving cycle_view the key of throttle ("W") in defaults/keyboard.json adds a conflict here.
}

TEST_CASE("controls: the schema parser rejects what it cannot use", "[controls]") {
    const std::string ok = R"({"format":"rg.control_actions/1","actions":[
        {"id":"a","label":"A","group":"driving","kind":"button","mode":"edge","modes":["drive"]}]})";
    std::string err;
    CHECK(rg::ActionSchema::parse(ok, "t", &err).has_value());
    const auto bad = [&](const std::string& text) {
        std::string e;
        const bool parsed = rg::ActionSchema::parse(text, "t", &e).has_value();
        INFO(text);
        INFO(e);
        CHECK_FALSE(parsed);
        CHECK_FALSE(e.empty());
    };
    bad("not json");
    bad(R"({"format":"rg.control_actions/2","actions":[]})");
    bad(R"({"format":"rg.control_actions/1","actions":[]})");
    bad(R"({"format":"rg.control_actions/1","actions":[{"id":"a","label":"A","group":"nope","kind":"button","modes":["drive"]}]})");
    bad(R"({"format":"rg.control_actions/1","actions":[{"id":"a","label":"A","group":"driving","kind":"axis","range":"signed","modes":["drive"]}]})"); // signed axis without labels
    bad(R"({"format":"rg.control_actions/1","actions":[{"id":"a","label":"A","group":"driving","kind":"button","modes":["bogus"]}]})");
    bad(R"({"format":"rg.control_actions/1","actions":[
        {"id":"a","label":"A","group":"driving","kind":"button","modes":["drive"]},
        {"id":"a","label":"B","group":"driving","kind":"button","modes":["drive"]}]})"); // duplicate id
    // sabotage: dropping the duplicate-id check in ActionSchema::parse fails the last case.
}

TEST_CASE("controls: the default profiles equal the bindings the game had before the menu", "[controls]") {
    Controls c = make_controls();
    c.joypad_connected(0, kGuidA, "Xbox Controller", 0x045e, 0x02ea, true);
    const std::string pad_key = c.key_for_slot(0);

    // Keyboard: the keys of the old project.godot [input] section and input_map.gd (plus N = nitrous arm, S1 2026-10-06: B is taken by the driving view, N is the drone-follow target key only in another mode).
    const std::map<std::string, std::string> keyboard = {
        {"steer", "key:A:-1, key:D"}, {"throttle", "key:W"}, {"brake", "key:S"}, {"handbrake", "key:Space"}, {"clutch", "key:C"},
        {"shift_up", "key:E"}, {"shift_down", "key:Q"}, {"ignition", "key:I"}, {"starter", "key:K"}, {"auto_shift", "key:F5"},
        {"toggle_nitrous", "key:N"}, {"flip_upright", "key:F"}, {"npc_truck", "key:T"}, {"get_out", "key:G"}, {"cycle_view", "key:B"},
        {"cycle_camera", "key:Tab"}, {"cycle_drone_target", "key:N"}, {"cam_zoom_in", "key:PageUp"}, {"cam_zoom_out", "key:PageDown"},
        {"cam_move_x", "key:A:-1, key:D"}, {"cam_move_z", "key:S:-1, key:W"},
        {"cam_move_y", "key:Q:-1, key:Ctrl:-1, key:E, key:Space"}, {"cam_fast", "key:Shift"},
        {"cam_look_x", "key:Left:-1, key:Right"}, {"cam_look_y", "key:Down:-1, key:Up"},
        {"jump", "key:Space"}, {"interact", "key:G"}, {"cycle_mode", "key:V"}, {"switch_world", "key:F8"}, {"reset_car", "key:R"},
        {"menu_confirm", "key:Enter, key:Kp Enter, key:Space"}, {"menu_back", "key:Escape"},
    };
    for (const auto& [action, expected] : keyboard) {
        CAPTURE(action);
        CHECK(fmt_list(c.effective_bindings("keyboard", action)) == expected);
    }
    // The keyboard binds nothing else.
    for (const std::string& id : c.applicable_actions("keyboard")) {
        if (keyboard.count(id) == 0) CHECK(c.effective_bindings("keyboard", id).empty());
    }

    // Mouse.
    CHECK(fmt_list(c.effective_bindings("mouse", "mouse_capture")) == "mouse:2");
    CHECK(fmt_list(c.effective_bindings("mouse", "cam_zoom_in")) == "mouse:4");
    CHECK(fmt_list(c.effective_bindings("mouse", "cam_zoom_out")) == "mouse:5");
    CHECK(fmt_list(c.effective_bindings("mouse", "mouse_look_x")) == "motion:0");
    CHECK(fmt_list(c.effective_bindings("mouse", "mouse_look_y")) == "motion:1");

    // Gamepad (XInput / standard).
    const std::map<std::string, std::string> gamepad = {
        {"steer", "axis:0:full:dz0.08"}, {"throttle", "axis:5:positive:dz0.02"}, {"brake", "axis:4:positive:dz0.02"},
        {"handbrake", "btn:0"}, {"clutch", "btn:9"}, {"shift_up", "btn:1"}, {"shift_down", "btn:2"}, {"ignition", "btn:11"},
        {"starter", "btn:6"}, {"auto_shift", "btn:13"}, {"toggle_nitrous", "btn:3"}, {"flip_upright", "btn:12"},
        {"cycle_view", "btn:10"}, {"cycle_camera", "btn:8"}, {"cycle_drone_target", "btn:14"}, {"cycle_mode", "btn:4"},
        {"cam_move_x", "axis:0:full:dz0.15"}, {"cam_move_z", "axis:1:full:dz0.15:inv"},
        {"cam_move_y", "btn:9:-1, btn:10, axis:4:positive:dz0.02:s-1, axis:5:positive:dz0.02"}, {"cam_fast", "btn:7"},
        {"cam_look_x", "axis:2:full:dz0.15"}, {"cam_look_y", "axis:3:full:dz0.15:inv"},
        {"jump", "btn:0"}, {"interact", "btn:2"}, {"menu_confirm", "btn:0"}, {"menu_back", "btn:1"},
    };
    for (const auto& [action, expected] : gamepad) {
        CAPTURE(action);
        CHECK(fmt_list(c.effective_bindings(pad_key, action)) == expected);
    }
    for (const std::string& id : c.applicable_actions(pad_key)) {
        if (gamepad.count(id) == 0) CHECK(c.effective_bindings(pad_key, id).empty());
    }
    // Reset car is keyboard only (owner 2026-10-05).
    CHECK(c.effective_bindings(pad_key, "reset_car").empty());
    // sabotage: changing any key or button in data/controls/defaults/*.json fails the matching line.
}

// ---- device identity and profile resolution -----------------------------------------------------------

TEST_CASE("controls: a pad is keyed by GUID and ordinal, a reconnected pad gets its profile back", "[controls][devices]") {
    Controls c = make_controls();
    const std::string a = c.joypad_connected(3, kGuidA, "Xbox Controller", 0x045e, 0x02ea, true);
    CHECK(a == std::string("joy:") + kGuidA + "#1");
    REQUIRE(c.bind(a, "ignition", 0, joy_button(7), true));
    CHECK(c.is_overridden(a, "ignition"));

    c.joypad_disconnected(3);
    // The unplugged device stays listed, marked not connected.
    bool listed = false;
    for (const rg::DeviceInfo& d : c.devices()) {
        if (d.key == a) {
            listed = true;
            CHECK_FALSE(d.connected);
        }
    }
    CHECK(listed);
    // It comes back on another slot with the same profile.
    const std::string again = c.joypad_connected(5, kGuidA, "Xbox Controller", 0x045e, 0x02ea, true);
    CHECK(again == a);
    CHECK(fmt_list(c.effective_bindings(again, "ignition")) == "btn:7");
    // sabotage: keying the profile by slot instead of GUID/ordinal gives a different key and the override is lost.
}

TEST_CASE("controls: two pads of one GUID are #1 and #2, #2 starts from #1 and gets its own profile on edit", "[controls][devices]") {
    Controls c = make_controls();
    const std::string one = c.joypad_connected(0, kGuidA, "Xbox Controller", 0x045e, 0x02ea, true);
    REQUIRE(c.bind(one, "ignition", 0, joy_button(7), true));
    const std::string two = c.joypad_connected(1, kGuidA, "Xbox Controller", 0x045e, 0x02ea, true);
    CHECK(two == std::string("joy:") + kGuidA + "#2");
    CHECK(one != two);
    // #2 has no profile of its own yet: it resolves through #1 ...
    CHECK(fmt_list(c.effective_bindings(two, "ignition")) == "btn:7");
    CHECK(c.device(two)->inherits);
    // ... until it is edited: then it owns a copy and #1 is not touched by it.
    REQUIRE(c.bind(two, "starter", 0, joy_button(8), true));
    CHECK_FALSE(c.device(two)->inherits);
    CHECK(fmt_list(c.effective_bindings(two, "ignition")) == "btn:7");
    CHECK(fmt_list(c.effective_bindings(two, "starter")) == "btn:8");
    CHECK(fmt_list(c.effective_bindings(one, "starter")) == "btn:6");
    // Changing #1 afterwards no longer reaches #2.
    REQUIRE(c.bind(one, "ignition", 0, joy_button(9), true));
    CHECK(fmt_list(c.effective_bindings(two, "ignition")) == "btn:7");
    // Reset device #2: back to the class default, not to #1's setup.
    CHECK(c.reset_device(two));
    CHECK(fmt_list(c.effective_bindings(two, "ignition")) == "btn:11");
    // sabotage: resolving #2 straight to the class default (skipping #1) fails the first inherits check.
}

TEST_CASE("controls: a pad of another GUID does not inherit a customised pad", "[controls][devices]") {
    Controls c = make_controls();
    const std::string one = c.joypad_connected(0, kGuidA, "Xbox Controller", 0x045e, 0x02ea, true);
    REQUIRE(c.bind(one, "ignition", 0, joy_button(7), true));
    const std::string other = c.joypad_connected(1, kGuidB, "8BitDo Pad", 0x2dc8, 0x6001, true);
    CHECK(other == std::string("joy:") + kGuidB + "#1");
    CHECK(fmt_list(c.effective_bindings(other, "ignition")) == "btn:11"); // class default
    CHECK_FALSE(c.device(other)->inherits);
    // sabotage: falling back to "the first customised pad" instead of "#1 of the same GUID" fails here.
}

TEST_CASE("controls: the ordinal is the lowest one free among the connected pads of a GUID", "[controls][devices]") {
    Controls c = make_controls();
    const std::string one = c.joypad_connected(0, kGuidA, "Pad", 1, 2, true);
    const std::string two = c.joypad_connected(1, kGuidA, "Pad", 1, 2, true);
    c.joypad_disconnected(0); // #1 leaves ...
    const std::string three = c.joypad_connected(2, kGuidA, "Pad", 1, 2, true); // ... the next one takes #1 again
    CHECK(three == one);
    CHECK(two.back() == '2');
    CHECK(c.key_for_slot(2) == one);
    CHECK(c.key_for_slot(1) == two);
    CHECK(c.key_for_slot(0).empty());
}

TEST_CASE("controls: device list order and classes", "[controls][devices]") {
    Controls c = make_controls();
    c.joypad_connected(0, kGuidA, "Xbox Controller", 1, 2, true);
    c.joypad_connected(1, kGuidB, "Logitech G29 Driving Force Racing Wheel", 3, 4, true);
    c.joypad_connected(2, "ff", "Unknown stick", 5, 6, false);
    c.joypad_disconnected(0);
    const std::vector<rg::DeviceInfo> d = c.devices();
    REQUIRE(d.size() == 5);
    CHECK(d[0].key == "keyboard");
    CHECK(d[1].key == "mouse");
    CHECK(d[2].connected);
    CHECK(d[2].cls == rg::DeviceClass::Wheel);
    CHECK(d[3].connected);
    CHECK(d[3].cls == rg::DeviceClass::Generic);
    CHECK_FALSE(d[4].connected); // remembered, after the connected ones
    CHECK(d[4].label == "Xbox Controller #1");
    // A pad without an SDL mapping does nothing until bound (safe default).
    CHECK(c.effective_bindings(d[3].key, "steer").empty());
    CHECK(c.effective_bindings(d[3].key, "handbrake").empty());
    // A wheel starts from the gamepad defaults.
    CHECK(fmt_list(c.effective_bindings(d[2].key, "handbrake")) == "btn:0");
}

// ---- editing ------------------------------------------------------------------------------------------------

TEST_CASE("controls: binding, second binding, clear, reset action and reset device", "[controls][edit]") {
    Controls c = make_controls();
    CHECK_FALSE(c.dirty());
    REQUIRE(c.bind("keyboard", "shift_up", 0, key("X"), true));
    CHECK(c.dirty());
    CHECK(fmt_list(c.effective_bindings("keyboard", "shift_up")) == "key:X");
    REQUIRE(c.bind("keyboard", "shift_up", 0, key("Z"), false)); // a second binding
    CHECK(fmt_list(c.effective_bindings("keyboard", "shift_up")) == "key:X, key:Z");
    REQUIRE(c.bind("keyboard", "shift_up", 0, key("Z"), false)); // the same input twice is one binding
    CHECK(c.effective_bindings("keyboard", "shift_up").size() == 2);
    REQUIRE(c.remove_binding("keyboard", "shift_up", 0));
    CHECK(fmt_list(c.effective_bindings("keyboard", "shift_up")) == "key:Z");
    REQUIRE(c.clear_action("keyboard", "shift_up"));
    CHECK(c.effective_bindings("keyboard", "shift_up").empty());
    CHECK(c.is_overridden("keyboard", "shift_up"));
    REQUIRE(c.reset_action("keyboard", "shift_up"));
    CHECK(fmt_list(c.effective_bindings("keyboard", "shift_up")) == "key:E");
    CHECK_FALSE(c.is_overridden("keyboard", "shift_up"));

    REQUIRE(c.bind("keyboard", "shift_down", 0, key("Y"), true));
    REQUIRE(c.bind("keyboard", "ignition", 0, key("U"), true));
    CHECK(c.reset_device("keyboard"));
    CHECK(fmt_list(c.effective_bindings("keyboard", "shift_down")) == "key:Q");
    CHECK(fmt_list(c.effective_bindings("keyboard", "ignition")) == "key:I");
    // Setting an action back to its default drops the override (so a later default change reaches it).
    REQUIRE(c.set_bindings("keyboard", "ignition", {key("U")}));
    REQUIRE(c.set_bindings("keyboard", "ignition", {key("I")}));
    CHECK_FALSE(c.is_overridden("keyboard", "ignition"));
}

TEST_CASE("controls: a binding that does not suit the device or the action is refused", "[controls][edit]") {
    Controls c = make_controls();
    const std::string p = c.joypad_connected(0, kGuidA, "Xbox Controller", 1, 2, true);
    std::string err;
    CHECK_FALSE(c.bind("keyboard", "shift_up", 0, joy_button(2), true, &err)); // a button on the keyboard
    CHECK_FALSE(err.empty());
    CHECK_FALSE(c.bind(p, "shift_up", 0, key("X"), true, &err)); // a key on a pad
    CHECK_FALSE(c.bind("keyboard", "shift_up", 0, key("Escape"), true, &err)); // Esc is reserved
    CHECK(c.bind("keyboard", "menu_back", 0, key("Escape"), true, &err));
    Binding motion;
    motion.type = BindingType::MouseMotion;
    CHECK_FALSE(c.bind("mouse", "shift_up", 0, motion, true, &err)); // motion on a button
    CHECK(c.bind("mouse", "mouse_look_x", 0, motion, true, &err));
    CHECK_FALSE(c.bind("keyboard", "mouse_look_x", 0, key("X"), true, &err)); // mouse look takes motion only
    CHECK_FALSE(c.bind("mouse", "menu_confirm", 0, joy_button(0), true, &err));
    CHECK_FALSE(c.bind("nobody", "shift_up", 0, key("X"), true, &err));
    CHECK_FALSE(c.bind("keyboard", "nothing", 0, key("X"), true, &err));
    std::vector<Binding> many(rg::kMaxBindingsPerAction + 1, key("X"));
    CHECK_FALSE(c.set_bindings("keyboard", "shift_up", many, &err));
    // Nothing of the refused edits stuck.
    CHECK(fmt_list(c.effective_bindings("keyboard", "shift_up")) == "key:E");
}

TEST_CASE("controls: a digital binding on a signed axis takes the row's side", "[controls][edit]") {
    Controls c = make_controls();
    REQUIRE(c.bind("keyboard", "steer", -1, key("J"), true));
    REQUIRE(c.bind("keyboard", "steer", 1, key("L"), true));
    CHECK(fmt_list(c.effective_bindings("keyboard", "steer")) == "key:J:-1, key:L");
    const rg::ActionState s = c.evaluate(snap_keys({"J"}));
    CHECK(value(c, s, "steer") == Approx(-1.0F));
    // Replacing only the right row keeps the left one.
    REQUIRE(c.bind("keyboard", "steer", 1, key("K"), true));
    CHECK(fmt_list(c.effective_bindings("keyboard", "steer")) == "key:J:-1, key:K");
    // sabotage: bind() ignoring `sign` binds J as +1 and the steer check above reads +1.
}

TEST_CASE("controls: rows follow the action list - signed axes split by side, pads add the axis row", "[controls][edit]") {
    Controls c = make_controls();
    const std::string p = c.joypad_connected(0, kGuidA, "Xbox Controller", 1, 2, true);
    int kb_steer = 0;
    for (const rg::RowInfo& r : c.rows("keyboard")) kb_steer += r.action == "steer" ? 1 : 0;
    CHECK(kb_steer == 2);
    int pad_steer = 0;
    for (const rg::RowInfo& r : c.rows(p)) {
        if (r.action != "steer") continue;
        ++pad_steer;
        if (r.sign == 0) CHECK(r.bindings.size() == 1); // the stick axis lives on the full-axis row
        else CHECK(r.bindings.empty());
    }
    CHECK(pad_steer == 3);
    // Keyboard rows: A is on the left row, D on the right one.
    for (const rg::RowInfo& r : c.rows("keyboard")) {
        if (r.action == "steer" && r.sign == -1) {
            REQUIRE(r.bindings.size() == 1);
            CHECK(c.effective_bindings("keyboard", "steer")[static_cast<std::size_t>(r.bindings[0])].key == "A");
        }
    }
}

// ---- conflicts ------------------------------------------------------------------------------------------------

TEST_CASE("controls: the same input on two actions live in one mode is a conflict, across modes it is not", "[controls][conflicts]") {
    Controls c = make_controls();
    // Space = handbrake (drive) and jump (on foot): legal, never active together.
    CHECK(c.conflicts("keyboard").empty());
    REQUIRE(c.bind("keyboard", "shift_up", 0, key("W"), false)); // W is throttle, both driving
    const std::vector<rg::Conflict> found = c.conflicts("keyboard");
    REQUIRE(found.size() == 1);
    CHECK(((found[0].action_a == "throttle" && found[0].action_b == "shift_up") || (found[0].action_a == "shift_up" && found[0].action_b == "throttle")));
    CHECK(c.conflicts_for("keyboard", "shift_up").size() == 1);
    CHECK(c.conflicts_for("keyboard", "brake").empty());
    bool flagged = false;
    for (const rg::RowInfo& r : c.rows("keyboard")) {
        if (r.action == "shift_up" && r.conflict) flagged = true;
    }
    CHECK(flagged);
    REQUIRE(c.reset_action("keyboard", "shift_up"));
    CHECK(c.conflicts("keyboard").empty());
    // sabotage: ignoring the mode overlap (every shared input is a conflict) makes the first CHECK fail on Space/G/A/RB.
}

TEST_CASE("controls: the two halves of one axis do not conflict, the full axis and a half do", "[controls][conflicts]") {
    Controls c = make_controls();
    const std::string p = c.joypad_connected(0, kGuidA, "Xbox Controller", 1, 2, true);
    REQUIRE(c.bind_combined_pedals(p, 5, AxisSpan::Positive, {}));
    CHECK(c.conflicts(p).empty());
    REQUIRE(c.bind(p, "clutch", 0, joy_axis(5), true)); // the full axis 5 overlaps both halves
    CHECK(c.conflicts(p).size() == 2);
    // sabotage: spans_overlap() treating two different halves as overlapping reports a conflict for the pedals.
}

// ---- axis pipeline --------------------------------------------------------------------------------------------------

TEST_CASE("axis pipeline: deadzone, saturation, curve, sensitivity and invert", "[controls][axis]") {
    Binding b = joy_axis(0);
    b.tuning.deadzone = 0.2F;
    const auto eval = [&](const Binding& x, float raw) { return rg::evaluate_axis_binding(x, raw, rg::AxisRange::Signed); };
    CHECK(eval(b, 0.1F) == 0.0F);
    CHECK(eval(b, -0.2F) == 0.0F);
    CHECK(eval(b, 1.0F) == Approx(1.0F));
    CHECK(eval(b, 0.6F) == Approx(0.5F)); // (0.6 - 0.2) / 0.8
    CHECK(eval(b, -0.6F) == Approx(-0.5F));
    b.tuning.saturation = 0.8F;
    CHECK(eval(b, 0.8F) == Approx(1.0F));
    CHECK(eval(b, 0.5F) == Approx(0.5F)); // (0.5 - 0.2) / 0.6
    b.tuning.curve = 2.0F;
    CHECK(eval(b, 0.5F) == Approx(0.25F));
    b.tuning.curve = 1.0F;
    b.tuning.sensitivity = 2.0F;
    CHECK(eval(b, 0.5F) == Approx(1.0F)); // clamped to 1
    b.tuning.sensitivity = 1.0F;
    b.tuning.invert = true;
    CHECK(eval(b, 0.5F) == Approx(-0.5F));
    // Nan / inf never leaks.
    CHECK(eval(joy_axis(0), std::nanf("")) == 0.0F);
    // sabotage: applying the deadzone after the curve (or not rescaling) moves the 0.6 -> 0.5 line.
}

TEST_CASE("axis pipeline: a pedal that rests at +1 (calibration) and a half-axis trigger", "[controls][axis]") {
    // A pedal reporting +1 at rest and -1 fully pressed, bound to a 0..1 action.
    Binding pedal = joy_axis(2);
    pedal.tuning.calibrated = true;
    pedal.tuning.cal_min = 1.0F;
    pedal.tuning.cal_max = -1.0F;
    const auto unit = [&](const Binding& x, float raw) { return rg::evaluate_axis_binding(x, raw, rg::AxisRange::Unit); };
    CHECK(unit(pedal, 1.0F) == Approx(0.0F));
    CHECK(unit(pedal, 0.0F) == Approx(0.5F));
    CHECK(unit(pedal, -1.0F) == Approx(1.0F));
    // Calibrated narrower travel (the pedal only reports 0.8 .. -0.6).
    pedal.tuning.cal_min = 0.8F;
    pedal.tuning.cal_max = -0.6F;
    CHECK(unit(pedal, 0.8F) == Approx(0.0F));
    CHECK(unit(pedal, -0.6F) == Approx(1.0F));
    CHECK(unit(pedal, -1.0F) == Approx(1.0F)); // beyond the calibrated end stays clamped
    // A trigger that rests at 0: the positive half.
    Binding trigger = joy_axis(5, AxisSpan::Positive);
    CHECK(unit(trigger, 0.0F) == 0.0F);
    CHECK(unit(trigger, 0.5F) == Approx(0.5F));
    CHECK(unit(trigger, -0.5F) == 0.0F);
    // The same axis fed through a Full span on a unit action is mapped -1..1 -> 0..1.
    CHECK(unit(joy_axis(2), -1.0F) == Approx(0.0F));
    CHECK(unit(joy_axis(2), 1.0F) == Approx(1.0F));
    // sabotage: not honouring cal_min > cal_max (a reversed pedal) reads 1.0 at rest.
}

TEST_CASE("axis pipeline: combined pedals split one axis into throttle and brake", "[controls][axis]") {
    Controls c = make_controls();
    const std::string p = c.joypad_connected(0, kGuidA, "Xbox Controller", 1, 2, true);
    rg::AxisTuning t;
    t.deadzone = 0.1F;
    REQUIRE(c.bind_combined_pedals(p, 1, AxisSpan::Negative, t)); // pushing the axis up (negative) = throttle
    CHECK(c.effective_bindings(p, "throttle").size() == 1);
    CHECK(c.effective_bindings(p, "brake").size() == 1);
    const auto eval = [&](float axis1) {
        InputSnapshot in;
        in.pads.push_back(pad(0, {}, {0.0F, axis1, 0.0F, 0.0F, 0.0F, 0.0F}));
        return c.evaluate(in);
    };
    rg::ActionState s = eval(-1.0F);
    CHECK(value(c, s, "throttle") == Approx(1.0F));
    CHECK(value(c, s, "brake") == 0.0F);
    s = eval(1.0F);
    CHECK(value(c, s, "throttle") == 0.0F);
    CHECK(value(c, s, "brake") == Approx(1.0F));
    s = eval(0.0F);
    CHECK(value(c, s, "throttle") == 0.0F);
    CHECK(value(c, s, "brake") == 0.0F);
    s = eval(-0.55F);
    CHECK(value(c, s, "throttle") == Approx(0.5F)); // (0.55 - 0.1) / 0.9
    CHECK_FALSE(c.bind_combined_pedals(p, 1, AxisSpan::Full, t));
    // sabotage: giving brake the same half as throttle makes both read 1.0 at -1.
}

TEST_CASE("axis pipeline: mouse motion is a delta scaled by sensitivity", "[controls][axis]") {
    Controls c = make_controls();
    InputSnapshot in;
    in.mouse_dx = 12.0F;
    in.mouse_dy = -3.0F;
    rg::ActionState s = c.evaluate(in);
    CHECK(value(c, s, "mouse_look_x") == Approx(12.0F));
    CHECK(value(c, s, "mouse_look_y") == Approx(-3.0F));
    Binding m;
    m.type = BindingType::MouseMotion;
    m.index = 0;
    m.tuning.sensitivity = 2.0F;
    m.tuning.invert = true;
    REQUIRE(c.bind("mouse", "mouse_look_x", 0, m, true));
    s = c.evaluate(in);
    CHECK(value(c, s, "mouse_look_x") == Approx(-24.0F));
}

// ---- evaluation -------------------------------------------------------------------------------------------------------

TEST_CASE("evaluate: keys, opposite keys cancel, the pad stick and the keyboard merge by magnitude", "[controls][evaluate]") {
    Controls c = make_controls();
    const std::string p = c.joypad_connected(0, kGuidA, "Xbox Controller", 1, 2, true);
    (void)p;
    rg::ActionState s = c.evaluate(snap_keys({"A"}));
    CHECK(value(c, s, "steer") == Approx(-1.0F));
    s = c.evaluate(snap_keys({"A", "D"}));
    CHECK(value(c, s, "steer") == Approx(0.0F)); // like Input.get_axis
    CHECK(value(c, s, "throttle") == 0.0F);
    s = c.evaluate(snap_keys({"W"}));
    CHECK(value(c, s, "throttle") == Approx(1.0F));
    CHECK(s.keyboard[static_cast<std::size_t>(c.schema().index_of("throttle"))] == Approx(1.0F));

    InputSnapshot in = snap_keys({"A"});
    in.pads.push_back(pad(0, {}, {0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}));
    s = c.evaluate(in);
    CHECK(value(c, s, "steer") == Approx(-1.0F)); // the larger magnitude wins: key -1 over stick +0.45
    CHECK(s.pad[static_cast<std::size_t>(c.schema().index_of("steer"))] > 0.3F);
    CHECK(s.source[static_cast<std::size_t>(c.schema().index_of("steer"))] == static_cast<int>(rg::DeviceClass::Keyboard));
    in = snap_keys({});
    in.pads.push_back(pad(0, {}, {0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}));
    s = c.evaluate(in);
    CHECK(value(c, s, "steer") == Approx((0.5F - 0.08F) / (1.0F - 0.08F)));
    CHECK(s.source[static_cast<std::size_t>(c.schema().index_of("steer"))] == static_cast<int>(rg::DeviceClass::Gamepad));
    // The raw channel is the value before the deadzone (what the camera rigs apply their own to).
    CHECK(s.raw[static_cast<std::size_t>(c.schema().index_of("steer"))] == Approx(0.5F));
}

TEST_CASE("evaluate: buttons, triggers as pedals, a trigger bound to a button, the wheel as pulses", "[controls][evaluate]") {
    Controls c = make_controls();
    c.joypad_connected(0, kGuidA, "Xbox Controller", 1, 2, true);
    InputSnapshot in;
    in.pads.push_back(pad(0, {1, 9}, {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.75F}));
    rg::ActionState s = c.evaluate(in);
    CHECK(value(c, s, "shift_up") == 1.0F);
    CHECK(value(c, s, "clutch") == 1.0F);
    CHECK(value(c, s, "shift_down") == 0.0F);
    CHECK(value(c, s, "throttle") == Approx((0.75F - 0.02F) / 0.98F));
    // An analogue half bound to a button fires past half travel.
    REQUIRE(c.bind(c.key_for_slot(0), "flip_upright", 0, joy_axis(4, AxisSpan::Positive), true));
    in.pads[0].axes[4] = 0.4F;
    CHECK(value(c, c.evaluate(in), "flip_upright") == 0.0F);
    in.pads[0].axes[4] = 0.6F;
    CHECK(value(c, c.evaluate(in), "flip_upright") == 1.0F);
    // The wheel: pulses on the action bound to the wheel button.
    InputSnapshot w;
    w.wheel_up = 2.0F;
    s = c.evaluate(w);
    CHECK(s.pulses[static_cast<std::size_t>(c.schema().index_of("cam_zoom_in"))] == Approx(2.0F));
    CHECK(s.pulses[static_cast<std::size_t>(c.schema().index_of("cam_zoom_out"))] == 0.0F);
    // Mouse buttons.
    InputSnapshot m;
    m.mouse_buttons = {2};
    CHECK(value(c, c.evaluate(m), "mouse_capture") == 1.0F);
    // sabotage: counting a wheel tick as a held button leaves pulses at 0.
}

TEST_CASE("evaluate: an unplugged pad is ignored, two pads act at once", "[controls][evaluate]") {
    Controls c = make_controls();
    c.joypad_connected(0, kGuidA, "Xbox Controller", 1, 2, true);
    c.joypad_connected(1, kGuidB, "8BitDo Pad", 3, 4, true);
    InputSnapshot in;
    in.pads.push_back(pad(0, {}, {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.3F}));
    in.pads.push_back(pad(1, {}, {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.9F}));
    CHECK(value(c, c.evaluate(in), "throttle") == Approx((0.9F - 0.02F) / 0.98F)); // the larger one
    c.joypad_disconnected(1);
    CHECK(value(c, c.evaluate(in), "throttle") == Approx((0.3F - 0.02F) / 0.98F));
    // evaluate_device: only the named device.
    InputSnapshot only;
    only.pads.push_back(pad(0, {}, {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F}));
    const rg::ActionState d = c.evaluate_device(c.key_for_slot(0), only);
    CHECK(value(c, d, "throttle") == Approx(1.0F));
    CHECK(value(c, c.evaluate_device("keyboard", snap_keys({"W"})), "throttle") == Approx(1.0F));
    CHECK(value(c, c.evaluate_device("keyboard", only), "throttle") == 0.0F);
}

TEST_CASE("evaluate: an edit shows up at once (the evaluation cache follows the revision)", "[controls][evaluate]") {
    Controls c = make_controls();
    CHECK(value(c, c.evaluate(snap_keys({"X"})), "starter") == 0.0F);
    CHECK(value(c, c.evaluate(snap_keys({"K"})), "starter") == 1.0F);
    const std::uint64_t before = c.revision();
    REQUIRE(c.bind("keyboard", "starter", 0, key("X"), true));
    CHECK(c.revision() > before);
    CHECK(value(c, c.evaluate(snap_keys({"X"})), "starter") == 1.0F);
    CHECK(value(c, c.evaluate(snap_keys({"K"})), "starter") == 0.0F);
    const std::vector<std::string> polled = c.polled_keys();
    CHECK(std::find(polled.begin(), polled.end(), "X") != polled.end());
    CHECK(std::find(polled.begin(), polled.end(), "K") == polled.end());
    // sabotage: not invalidating the cache in changed() leaves "K" live after the rebind.
}

TEST_CASE("menu bindings keep the built-in aliases and add the player's own", "[controls][menu]") {
    Controls c = make_controls();
    std::vector<Binding> confirm = c.menu_bindings("menu_confirm");
    const auto has = [&](const std::vector<Binding>& v, const std::string& f) {
        for (const Binding& b : v) {
            if (fmt(b) == f) return true;
        }
        return false;
    };
    CHECK(has(confirm, "key:Enter"));
    CHECK(has(confirm, "key:Kp Enter"));
    CHECK(has(confirm, "btn:0"));
    REQUIRE(c.set_bindings("keyboard", "menu_confirm", {key("J")}));
    confirm = c.menu_bindings("menu_confirm");
    CHECK(has(confirm, "key:J"));
    CHECK(has(confirm, "key:Enter")); // Enter always works, however the player rebinds
    CHECK_FALSE(has(confirm, "key:Space"));
    REQUIRE(c.clear_action("keyboard", "menu_back"));
    CHECK(has(c.menu_bindings("menu_back"), "key:Escape"));
    CHECK(has(c.menu_bindings("menu_back"), "btn:1"));
    // sabotage: building the list from the profile only would lose Enter / Esc after a rebind or clear.
}

// ---- persistence ------------------------------------------------------------------------------------------------------

TEST_CASE("controls file: round trip keeps every device, binding and tuning", "[controls][file]") {
    Controls a = make_controls();
    const std::string p1 = a.joypad_connected(0, kGuidA, "Xbox Controller", 0x045e, 0x02ea, true);
    const std::string p2 = a.joypad_connected(1, kGuidB, "Logitech G29 Driving Force Racing Wheel", 0x046d, 0xc24f, true);
    REQUIRE(a.bind("keyboard", "shift_up", 0, key("X"), true));
    REQUIRE(a.bind("keyboard", "shift_up", 0, key("Z"), false));
    REQUIRE(a.clear_action("keyboard", "starter"));
    rg::AxisTuning t;
    t.deadzone = 0.12F;
    t.saturation = 0.9F;
    t.curve = 1.5F;
    t.sensitivity = 1.2F;
    t.invert = true;
    t.calibrated = true;
    t.cal_min = 0.9F;
    t.cal_max = -0.8F;
    REQUIRE(a.bind_combined_pedals(p2, 2, AxisSpan::Positive, t));
    Binding steer = joy_axis(0);
    steer.tuning.deadzone = 0.05F;
    REQUIRE(a.bind(p2, "steer", 0, steer, true));
    REQUIRE(a.bind(p1, "ignition", 0, joy_button(7), true));
    Binding m;
    m.type = BindingType::MouseMotion;
    m.index = 1;
    m.tuning.sensitivity = 0.5F;
    REQUIRE(a.bind("mouse", "mouse_look_x", 0, m, true));

    const std::string json = a.to_json();
    CHECK(json.find("rg.controls/1") != std::string::npos);
    CHECK(a.to_json() == json); // byte-stable

    Controls b = make_controls();
    const rg::ControlsLoadReport rep = b.load_text(json);
    CHECK(rep.ok());
    CHECK(rep.dropped.empty());
    CHECK(rep.message.empty());
    CHECK_FALSE(b.dirty());
    // Same devices (all remembered, not connected), same effective bindings everywhere.
    CHECK(b.devices().size() == a.devices().size());
    const std::vector<std::string> keys = {"keyboard", "mouse", p1, p2};
    for (const std::string& k : keys) {
        for (const std::string& action : a.applicable_actions(k)) {
            CAPTURE(k, action);
            CHECK(a.effective_bindings(k, action) == b.effective_bindings(k, action));
        }
    }
    CHECK(b.to_json() == json); // a loaded file writes back identically
    // The remembered pad comes back connected with its profile and name.
    CHECK(b.joypad_connected(7, kGuidB, "Logitech G29 Driving Force Racing Wheel", 0x046d, 0xc24f, true) == p2);
    CHECK(b.effective_bindings(p2, "throttle")[0].tuning.cal_min == Approx(0.9F));
    CHECK(b.device(p2)->cls == rg::DeviceClass::Wheel);
    // sabotage: not writing "calibrated" / cal_min leaves a default tuning and fails the == compare.
}

TEST_CASE("controls file: a pristine setup writes no overrides, defaults travel with the build", "[controls][file]") {
    Controls a = make_controls();
    const std::string json = a.to_json();
    CHECK(json.find("overrides") == std::string::npos);
    // A pad that was only plugged in is remembered by name, without overrides.
    a.joypad_connected(0, kGuidA, "Xbox Controller", 1, 2, true);
    const std::string with_pad = a.to_json();
    CHECK(with_pad.find(kGuidA) != std::string::npos);
    CHECK(with_pad.find("overrides") == std::string::npos);
}

TEST_CASE("controls file: save_file is atomic and load_file reads it back", "[controls][file]") {
    const std::string path = temp_path("save.json");
    std::filesystem::remove(path);
    std::filesystem::remove(path + ".tmp");
    Controls a = make_controls();
    REQUIRE(a.bind("keyboard", "shift_up", 0, key("X"), true));
    std::string err;
    REQUIRE(a.save_file(path, &err));
    INFO(err);
    CHECK_FALSE(a.dirty());
    CHECK(std::filesystem::exists(path));
    CHECK_FALSE(std::filesystem::exists(path + ".tmp")); // no temp file left behind
    Controls b = make_controls();
    const rg::ControlsLoadReport rep = b.load_file(path);
    CHECK(rep.ok());
    CHECK_FALSE(rep.file_missing);
    CHECK(fmt_list(b.effective_bindings("keyboard", "shift_up")) == "key:X");
    // Saving into a folder that does not exist yet creates it.
    const std::string nested = temp_path("nested") + "/deeper/controls.json";
    std::filesystem::remove_all(temp_path("nested"));
    CHECK(a.save_file(nested, &err));
    CHECK(std::filesystem::exists(nested));
    std::filesystem::remove_all(temp_path("nested"));
    std::filesystem::remove(path);
}

TEST_CASE("controls file: missing file gives the defaults without a backup", "[controls][file]") {
    const std::string path = temp_path("missing.json");
    std::filesystem::remove(path);
    std::filesystem::remove(path + ".bak");
    Controls c = make_controls();
    REQUIRE(c.bind("keyboard", "shift_up", 0, key("X"), true));
    const rg::ControlsLoadReport rep = c.load_file(path);
    CHECK(rep.file_missing);
    CHECK(rep.ok());
    CHECK_FALSE(rep.backup_written);
    CHECK_FALSE(std::filesystem::exists(path + ".bak"));
    CHECK(fmt_list(c.effective_bindings("keyboard", "shift_up")) == "key:E"); // the in-memory edit was replaced
}

TEST_CASE("controls file: a broken file gives defaults and is kept as .bak", "[controls][file]") {
    const std::string path = temp_path("broken.json");
    std::filesystem::remove(path + ".bak");
    const std::string garbage = "{ this is not json at all";
    for (const std::string& text : {garbage, std::string("[1,2,3]"), std::string(R"({"format":"something else","devices":[]})"),
                                    std::string(R"({"format":"rg.controls/1","devices":"nope"})")}) {
        std::filesystem::remove(path + ".bak");
        write_all(path, text);
        Controls c = make_controls();
        REQUIRE(c.bind("keyboard", "shift_up", 0, key("X"), true));
        const rg::ControlsLoadReport rep = c.load_file(path);
        INFO(text);
        CHECK_FALSE(rep.ok());
        CHECK(rep.parse_error);
        CHECK(rep.backup_written);
        CHECK_FALSE(rep.message.empty());
        CHECK(read_all(path + ".bak") == text);
        CHECK(fmt_list(c.effective_bindings("keyboard", "shift_up")) == "key:E");
        CHECK(c.devices().size() == 2);
    }
    std::filesystem::remove(path);
    std::filesystem::remove(path + ".bak");
    // sabotage: not copying the file before the caller overwrites it loses the .bak.
}

TEST_CASE("controls file: an unknown version gives defaults and is kept as .bak, never half-read", "[controls][file]") {
    const std::string path = temp_path("future.json");
    std::filesystem::remove(path + ".bak");
    const std::string future = R"({"format":"rg.controls/2","devices":[{"key":"keyboard","class":"keyboard","overrides":{"shift_up":[{"type":"key","key":"X"}]}}]})";
    write_all(path, future);
    Controls c = make_controls();
    const rg::ControlsLoadReport rep = c.load_file(path);
    CHECK_FALSE(rep.ok());
    CHECK(rep.unknown_version);
    CHECK(rep.version_found == 2);
    CHECK(rep.backup_written);
    CHECK(read_all(path + ".bak") == future);
    CHECK(fmt_list(c.effective_bindings("keyboard", "shift_up")) == "key:E");
    std::filesystem::remove(path);
    std::filesystem::remove(path + ".bak");
}

TEST_CASE("controls file: an entry that does not validate is dropped and reported, the rest loads", "[controls][file]") {
    const std::string text = R"({"format":"rg.controls/1","devices":[
        {"key":"keyboard","class":"keyboard","overrides":{
            "shift_up":[{"type":"key","key":"X"}],
            "no_such_action":[{"type":"key","key":"Q"}],
            "shift_down":[{"type":"joy_button","button":3}],
            "ignition":[{"type":"key"}],
            "starter":[{"type":"key","key":"Escape"}]}},
        {"key":"mystery","class":"keyboard"},
        {"key":"joy:aa#0","class":"gamepad"}]})";
    Controls c = make_controls();
    const rg::ControlsLoadReport rep = c.load_text(text);
    CHECK(rep.ok());
    CHECK(rep.dropped.size() == 6);
    CHECK_FALSE(rep.message.empty());
    CHECK(fmt_list(c.effective_bindings("keyboard", "shift_up")) == "key:X");
    CHECK(fmt_list(c.effective_bindings("keyboard", "shift_down")) == "key:Q"); // wrong-device binding dropped -> default
    CHECK(fmt_list(c.effective_bindings("keyboard", "ignition")) == "key:I");
    CHECK(fmt_list(c.effective_bindings("keyboard", "starter")) == "key:K"); // Esc is reserved
    CHECK(c.devices().size() == 2);
}

TEST_CASE("controls file: a pad profile written for GUID#2 stays with #2", "[controls][file]") {
    Controls a = make_controls();
    a.joypad_connected(0, kGuidA, "Xbox Controller", 1, 2, true);
    const std::string two = a.joypad_connected(1, kGuidA, "Xbox Controller", 1, 2, true);
    REQUIRE(a.bind(two, "ignition", 0, joy_button(7), true));
    Controls b = make_controls();
    REQUIRE(b.load_text(a.to_json()).ok());
    // Plugging the pads in again in the same order restores both.
    const std::string one_b = b.joypad_connected(4, kGuidA, "Xbox Controller", 1, 2, true);
    const std::string two_b = b.joypad_connected(5, kGuidA, "Xbox Controller", 1, 2, true);
    CHECK(two_b == two);
    CHECK(fmt_list(b.effective_bindings(two_b, "ignition")) == "btn:7");
    CHECK(fmt_list(b.effective_bindings(one_b, "ignition")) == "btn:11");
}

TEST_CASE("controls data: bad default profiles are rejected at load", "[controls]") {
    const std::string actions = R"({"format":"rg.control_actions/1","actions":[
        {"id":"a","label":"A","group":"driving","kind":"button","mode":"edge","modes":["drive"]}]})";
    const auto load = [&](const std::string& kb) {
        Controls c;
        std::string err;
        const bool ok = c.load_data_text(actions, {{"keyboard", kb}}, &err);
        INFO(kb);
        INFO(err);
        return ok;
    };
    CHECK(load(R"({"format":"rg.control_profile/1","class":"keyboard","bindings":{"a":[{"type":"key","key":"W"}]}})"));
    CHECK_FALSE(load(R"({"format":"rg.control_profile/1","class":"keyboard","bindings":{"zzz":[{"type":"key","key":"W"}]}})"));
    CHECK_FALSE(load(R"({"format":"rg.control_profile/1","class":"keyboard","bindings":{"a":[{"type":"joy_button","button":0}]}})"));
    CHECK_FALSE(load(R"({"format":"rg.control_profile/2","class":"keyboard","bindings":{}})"));
    CHECK_FALSE(load("garbage"));
}
