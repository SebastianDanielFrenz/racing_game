// godot_ext/src/rg_assists.h - RgAssists: static, stateless bindings of rg/assist_info.h (physics_sim assist unit).
// The GDScript keeps the key edges only; which traction-control mode comes next (offered modes only) and what the
// aero-automation toggle does are decided in rg_core.
#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/string.hpp>

namespace rg_godot {

class RgAssists : public godot::RefCounted {
    GDCLASS(RgAssists, godot::RefCounted)

public:
    // The next OFFERED traction-control mode code after `current` (0 Off, 1 On, 2 Relaxed, 3 Sport, 4 Drift), cycling
    // through `available_bits` only (PowertrainSnapshot::traction_modes_available_bits). -1 when none is offered.
    static int next_traction_mode(int available_bits, int current);
    // "Off", "On", "Relaxed", "Sport", "Drift" ("?" for an unknown code).
    static godot::String traction_mode_name(int code);
    // The new assist.aero_control channel value after a toggle press: flipped when assist_fitted_bits has bit 6.
    static double toggled_aero_control(int fitted_bits, double current_value);
    // The next OFFERED drive mode code after `current` (0 eco, 1 normal, 2 sport), cycling through `available_bits`
    // only (PowertrainSnapshot::drive_modes_available_bits). -1 when the car offers none: the key does nothing.
    static int next_drive_mode(int available_bits, int current);
    // "ECO", "NORMAL", "SPORT" ("?" for an unknown code).
    static godot::String drive_mode_name(int code);

protected:
    static void _bind_methods();
};

} // namespace rg_godot
