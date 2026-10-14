#pragma once
#include "rg/driver_actions.h"
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>

namespace rg_godot {
class RgDriverActions : public godot::RefCounted {
    GDCLASS(RgDriverActions, godot::RefCounted)
public:
    enum Kind { PRESS, RELEASE, SET, HOLD };
    enum Source { KEYBOARD, PAD, HUD, COCKPIT_MOUSE, VR_POKE, VR_GRAB, VR_RAY, SCRIPT };
    bool submit(const godot::String& id, int kind, double value, int source);
    godot::Array apply(const godot::Dictionary& snapshot, const godot::Dictionary& channels, bool driving_enabled);
    void discard_pending();
    void configure_cabin(const godot::Dictionary& values);
    godot::Array advance(const godot::Dictionary& snapshot, const godot::Dictionary& channels, bool enabled, double dt);
    void configure_auto_shift(bool enabled);
    bool get_auto_shift() const;
    static godot::Dictionary state(const godot::String& id, const godot::Dictionary& snapshot, const godot::Dictionary& channels);
protected:
    static void _bind_methods();
private:
    rg::DriverActions actions_;
};
}
VARIANT_ENUM_CAST(rg_godot::RgDriverActions::Kind)
VARIANT_ENUM_CAST(rg_godot::RgDriverActions::Source)
