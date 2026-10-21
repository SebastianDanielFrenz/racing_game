// rg/assist_info.h - the driver-assist state the HUD and the controls need (physics_sim assist unit, assists I1/I2/I5,
// launch control I4 and drive modes I7).
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
inline constexpr std::uint32_t kAssistBitEpb = 1u << 7;  // electronic parking brake (Q3 / I8)
inline constexpr std::uint32_t kAssistBitAbs = 1u << 8;  // assist unit abs block (I5 part 2)
inline constexpr std::uint32_t kAssistBitEsc = 1u << 9;  // assist unit esc block (I6)

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

// --- Drive modes (physics_sim assists I7, electronics kind drive_modes; car-specific part) ---------------------------
// Channel assist.drive_mode, append-only codes: 0 eco, 1 normal, 2 sport, 3 racing (Q3 / I8), 4 electric (the hybrid's pure-electric
// mode, physics_sim docs/electric_drive.md "Hybrid Electric mode"). PowertrainSnapshot::drive_modes_available_bits
// has bit n = code n for every mode the car's part lists (0 on a car without the part, which also has no channel).
// physics_sim owns every consequence of a mode (pedal bend, shift points, clutch close time, the traction-control
// default of the mode, the launch-control and N2O gates): the game only cycles the REQUEST through the OFFERED modes
// and shows the mode that runs. The game must NOT write assist.traction_control on a mode change - physics sets the TC
// request to the mode's default itself.
inline constexpr int kDriveModeCodeCount = 5;
inline constexpr int kDriveModeEco = 0, kDriveModeNormal = 1, kDriveModeSport = 2, kDriveModeRacing = 3, kDriveModeElectric = 4;

[[nodiscard]] inline const char* drive_mode_name(int code) {
    static const char* const names[kDriveModeCodeCount] = {"ECO", "NORMAL", "SPORT", "RACING", "ELECTRIC"};
    return (code >= 0 && code < kDriveModeCodeCount) ? names[code] : "?";
}

[[nodiscard]] inline bool drive_mode_offered(std::uint32_t available_bits, int code) {
    return code >= 0 && code < kDriveModeCodeCount && ((available_bits >> code) & 1u) != 0;
}

// The next OFFERED drive mode after `current` in code order (eco, normal, sport, then eco again), skipping every mode
// the car does not offer: an unoffered mode is never returned. `current` need not be offered itself (it is only the
// starting point). -1 when the car offers no mode at all (no drive_modes part: the key does nothing); with a single
// offered mode that mode is returned.
[[nodiscard]] inline int next_drive_mode(std::uint32_t available_bits, int current) {
    if ((available_bits & ((1u << kDriveModeCodeCount) - 1u)) == 0) return -1;
    if (current < 0 || current >= kDriveModeCodeCount) current = -1;
    for (int step = 1; step <= kDriveModeCodeCount; ++step) {
        const int code = (current + step) % kDriveModeCodeCount;
        if (drive_mode_offered(available_bits, code)) return code;
    }
    return -1;
}

// --- ESC (I6) -------------------------------------------------------------------------------------------------------
// Channel assist.esc codes: 0 off, 1 on, 2 sport. A car offers the modes in PowertrainSnapshot::esc_modes_available_bits.
inline constexpr int kEscModeCodeCount = 3;

[[nodiscard]] inline const char* esc_mode_name(int code) {
    static const char* const names[kEscModeCodeCount] = {"Off", "On", "Sport"};
    return (code >= 0 && code < kEscModeCodeCount) ? names[code] : "?";
}

// The new value of the channel assist.abs after a toggle press: flipped when the car has the ABS block (bit 8 of
// assist_fitted_bits), unchanged otherwise. physics_sim reads "on" as >= 0.5.
[[nodiscard]] inline double toggled_abs(std::uint32_t fitted_bits, double current_value) {
    if ((fitted_bits & kAssistBitAbs) == 0) return current_value;
    return current_value >= 0.5 ? 0.0 : 1.0;
}

// The next OFFERED ESC mode after `current` (off, on, sport, then off again); -1 when ESC is not fitted.
[[nodiscard]] inline int next_esc_mode(std::uint32_t available_bits, int current) {
    if ((available_bits & ((1u << kEscModeCodeCount) - 1u)) == 0) return -1;
    if (current < 0 || current >= kEscModeCodeCount) current = -1;
    for (int step = 1; step <= kEscModeCodeCount; ++step) {
        const int code = (current + step) % kEscModeCodeCount;
        if (((available_bits >> code) & 1u) != 0) return code;
    }
    return -1;
}

// --- Launch control lamp (assists I4 unit variant or the P8 two-step; assist bit 4) -------------------------------------
inline constexpr std::uint32_t kAssistBitLaunchControl = 1u << 4;

// The new value of the channel assist.launch_control after a toggle key press, from the channel's CURRENT value (physics_sim
// can write it itself on a drive-mode change, assists I7c): physics_sim reads "on" as strictly > 0.5. A car without launch
// control (bit 4 of assist_fitted_bits) has no such channel; the press does nothing.
[[nodiscard]] inline double toggled_launch_control(std::uint32_t fitted_bits, double current_value) {
    if ((fitted_bits & kAssistBitLaunchControl) == 0) return current_value;
    return current_value > 0.5 ? 0.0 : 1.0;
}

// PowertrainSnapshot::launch_phase codes.
inline constexpr int kLaunchPhaseIdle = 0, kLaunchPhaseArmed = 1, kLaunchPhaseLaunch = 2, kLaunchPhaseHandover = 3;

enum class LaunchLamp : int {
    Hidden = 0,    // the car has no launch control
    Off = 1,       // fitted, switched off
    On = 2,        // fitted and switched on, idle (waiting for brake + throttle at a standstill)
    Armed = 3,     // brake + throttle held at a standstill: the launch rpm is held
    Launching = 4, // the clutch is regulating the launch (also the hand-over to traction control)
};

[[nodiscard]] inline LaunchLamp launch_lamp(std::uint32_t fitted_bits, std::uint32_t active_bits, int launch_phase) {
    if ((fitted_bits & kAssistBitLaunchControl) == 0) return LaunchLamp::Hidden;
    if (launch_phase == kLaunchPhaseArmed) return LaunchLamp::Armed;
    if (launch_phase == kLaunchPhaseLaunch || launch_phase == kLaunchPhaseHandover) return LaunchLamp::Launching;
    return (active_bits & kAssistBitLaunchControl) != 0 ? LaunchLamp::On : LaunchLamp::Off;
}

[[nodiscard]] inline const char* launch_lamp_text(LaunchLamp l) {
    switch (l) {
    case LaunchLamp::Off: return "LC OFF";
    case LaunchLamp::On: return "LC";
    case LaunchLamp::Armed: return "LC ARMED";
    case LaunchLamp::Launching: return "LC LAUNCH";
    default: return "";
    }
}

struct AssistHud {
    bool dm_fitted = false; // the car has a drive_modes part
    int dm_requested = 0;
    int dm_effective = 0;   // the mode that actually runs (an unoffered request maps to the part's default mode)
    std::uint32_t dm_available_bits = 0;
    LaunchLamp lc = LaunchLamp::Hidden;
    int lc_phase = kLaunchPhaseIdle;
    bool tc_fitted = false;
    int tc_requested = 0;
    int tc_effective = 0; // the mode that actually runs (physics_sim resolves a requested-but-unoffered mode)
    std::uint32_t tc_available_bits = 0;
    bool tc_intervening = false; // the controller is cutting torque right now
    bool aero_fitted = false;
    bool aero_active = false;
    double aero_target_deg = 0.0; // the wing offset the automation asks for (0 when not active)
    bool abs_fitted = false;
    bool abs_on = false;           // switched on (assist.abs)
    bool abs_intervening = false;  // a service brake is scaled down right now
    bool esc_fitted = false;
    int esc_requested = 0;
    int esc_effective = 0;         // the mode that runs (0 off, 1 on, 2 sport)
    std::uint32_t esc_available_bits = 0;
    bool esc_intervening = false;  // ESC brakes a wheel or cuts torque right now
    bool epb_fitted = false;
    int epb_state = 0;             // 0 released, 1 latched, 2 handbrake
    bool epb_refused = false;
};

[[nodiscard]] inline AssistHud assist_hud(const ps::drivetrain::PowertrainSnapshot& p) {
    AssistHud h;
    h.dm_available_bits = static_cast<std::uint32_t>(p.drive_modes_available_bits) & ((1u << kDriveModeCodeCount) - 1u);
    h.dm_fitted = h.dm_available_bits != 0;
    h.dm_requested = static_cast<int>(p.drive_mode_requested);
    h.dm_effective = static_cast<int>(p.drive_mode_effective);
    h.lc_phase = static_cast<int>(p.launch_phase);
    h.lc = launch_lamp(p.assist_fitted_bits, p.assist_active_bits, h.lc_phase);
    h.tc_fitted = (p.assist_fitted_bits & kAssistBitTractionControl) != 0;
    h.tc_requested = static_cast<int>(p.traction_mode_requested);
    h.tc_effective = static_cast<int>(p.traction_mode_effective);
    h.tc_available_bits = static_cast<std::uint32_t>(p.traction_modes_available_bits);
    h.tc_intervening = p.traction_control_intervening;
    h.aero_fitted = (p.assist_fitted_bits & kAssistBitAeroControl) != 0;
    h.aero_active = p.aero_control_active;
    h.aero_target_deg = static_cast<double>(p.aero_control_target_deg);
    h.abs_fitted = (p.assist_fitted_bits & kAssistBitAbs) != 0;
    h.abs_on = (p.assist_active_bits & kAssistBitAbs) != 0;
    h.abs_intervening = h.abs_fitted && p.abs_active;
    h.esc_fitted = (p.assist_fitted_bits & kAssistBitEsc) != 0;
    h.esc_requested = static_cast<int>(p.esc_mode_requested);
    h.esc_effective = static_cast<int>(p.esc_mode_effective);
    h.esc_available_bits = static_cast<std::uint32_t>(p.esc_modes_available_bits) & ((1u << kEscModeCodeCount) - 1u);
    h.esc_intervening = h.esc_fitted && p.esc_active;
    h.epb_fitted = (p.assist_fitted_bits & kAssistBitEpb) != 0;
    h.epb_state = static_cast<int>(p.epb_state);
    h.epb_refused = h.epb_fitted && p.epb_refused;
    return h;
}

} // namespace rg
