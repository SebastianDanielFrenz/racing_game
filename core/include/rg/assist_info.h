// rg/assist_info.h - the driver-assist state the HUD and the controls need (physics_sim assist unit, assists I1/I2/I5).
//
// Engine-neutral and header-only: the Godot binding (RgSimulation.get_vehicle_powertrain, RgAssists) only forwards
// these values and the GDScript keeps no cycle logic of its own. physics_sim owns every assist decision: which
// traction-control modes a car offers (PowertrainSnapshot::traction_modes_available_bits, bit n = mode code n), which
// mode actually runs (traction_mode_effective) and whether the rear-wing automation is fitted/active. The game only
//  - cycles the requested mode through the OFFERED modes (a mode the car does not offer is never selectable), and
//  - flips the plain 0/1 channel assist.aero_control when the car has the automation.
#pragma once

#include "ps/drivetrain/powertrain_state.h"

#include <cstdint>

namespace rg {

// Mode CODES of the channel assist.traction_control (physics_sim ps::vehicle::TractionMode, append-only).
inline constexpr int kTractionModeCodeCount = 5;
// assist_fitted_bits / assist_active_bits layout (powertrain_state.h).
inline constexpr std::uint32_t kAssistBitTractionControl = 1u << 3;
inline constexpr std::uint32_t kAssistBitAeroControl = 1u << 6;

[[nodiscard]] inline const char* traction_mode_name(int code) {
    static const char* const names[kTractionModeCodeCount] = {"Off", "On", "Relaxed", "Sport", "Drift"};
    return (code >= 0 && code < kTractionModeCodeCount) ? names[code] : "?";
}

[[nodiscard]] inline bool traction_mode_offered(std::uint32_t available_bits, int code) {
    return code >= 0 && code < kTractionModeCodeCount && ((available_bits >> code) & 1u) != 0;
}

// The next OFFERED mode after `current` in code order (Off, On, Relaxed, Sport, Drift, then Off again), skipping every
// mode the car does not offer. `current` need not be offered itself (it is only the starting point). Returns -1 when
// the car offers no mode at all (traction control not fitted); with a single offered mode that mode is returned.
[[nodiscard]] inline int next_traction_mode(std::uint32_t available_bits, int current) {
    if ((available_bits & ((1u << kTractionModeCodeCount) - 1u)) == 0) return -1;
    if (current < 0 || current >= kTractionModeCodeCount) current = -1;
    for (int step = 1; step <= kTractionModeCodeCount; ++step) {
        const int code = (current + step) % kTractionModeCodeCount;
        if (traction_mode_offered(available_bits, code)) return code;
    }
    return -1;
}

// The new value of the channel assist.aero_control after a toggle key press: flipped when the car has the automation
// (bit 6 of assist_fitted_bits), unchanged otherwise (an unfitted car has no such channel; the press does nothing).
[[nodiscard]] inline double toggled_aero_control(std::uint32_t fitted_bits, double current_value) {
    if ((fitted_bits & kAssistBitAeroControl) == 0) return current_value;
    return current_value >= 0.5 ? 0.0 : 1.0;
}

struct AssistHud {
    bool tc_fitted = false;
    int tc_requested = 0;
    int tc_effective = 0; // the mode that actually runs (physics_sim resolves a requested-but-unoffered mode)
    std::uint32_t tc_available_bits = 0;
    bool tc_intervening = false; // the controller is cutting torque right now
    bool aero_fitted = false;
    bool aero_active = false;
    double aero_target_deg = 0.0; // the wing offset the automation asks for (0 when not active)
};

[[nodiscard]] inline AssistHud assist_hud(const ps::drivetrain::PowertrainSnapshot& p) {
    AssistHud h;
    h.tc_fitted = (p.assist_fitted_bits & kAssistBitTractionControl) != 0;
    h.tc_requested = static_cast<int>(p.traction_mode_requested);
    h.tc_effective = static_cast<int>(p.traction_mode_effective);
    h.tc_available_bits = static_cast<std::uint32_t>(p.traction_modes_available_bits);
    h.tc_intervening = p.traction_control_intervening;
    h.aero_fitted = (p.assist_fitted_bits & kAssistBitAeroControl) != 0;
    h.aero_active = p.aero_control_active;
    h.aero_target_deg = static_cast<double>(p.aero_control_target_deg);
    return h;
}

} // namespace rg
