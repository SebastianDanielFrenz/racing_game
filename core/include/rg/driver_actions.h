#pragma once

#include "rg/assist_info.h"

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace rg {

enum class DriverActionKind { Press, Release, Set, Hold };
enum class ActionSource { Keyboard, Pad, Hud, CockpitMouse, VrPoke, VrGrab, VrRay, Script };

struct ActionEvent {
    std::string id;
    DriverActionKind kind = DriverActionKind::Press;
    double value = 1;
    ActionSource source = ActionSource::Keyboard;
};

// C1 extends this game-owned state; the EPB latch (and its refusal at speed) belongs to physics, never here.
struct CabinState {
    bool auto_shift = false;
    bool horn = false;
    bool auto_cranking = false;
    bool indicator_turned = false;
    double blink_elapsed_s = 0;
    bool hazard = false;
    int indicator = 0;
    int lights = 0; // 0 off, 1 dipped, 2 main beam ("lights" action; channel cabin.lights)
    int wipers = 0;
    double auto_crank_elapsed_s = 0;
};

// Adapter-neutral view of the published snapshot; requested channels are read
// separately because physics can write TC/LC defaults back into them.
struct DriverSnapshot {
    std::uint32_t fitted_bits = 0;
    std::uint32_t traction_modes = 0;
    int traction_effective = 0;
    bool traction_intervening = false;
    bool traction_active = false;
    std::uint32_t drive_modes = 0;
    int drive_mode_effective = 0;
    int launch_phase = 0;
    LaunchLamp launch = LaunchLamp::Hidden;
    bool aero_active = false;
    double aero_target_deg = 0;
    bool nitrous_fitted = false;
    bool park_fitted = false;
    bool park_engaged = false;
    bool epb_fitted = false;
    int epb_state = 0; // released / latched / handbrake
    bool epb_refused = false;
    bool abs_fitted = false;
    bool abs_on = false;
    bool abs_intervening = false;
    bool esc_fitted = false;
    int esc_effective = 0; // 0 off, 1 on, 2 sport
    bool esc_intervening = false;
    int gear = 0;
    // Automatic gearbox plan A1 (physics automatic_tcu): 0 manual, 1 torque-converter automatic, 2 dual-clutch.
    int transmission_kind = 0;
    int transmission_selector = 0; // -1 R, 0 N (P when park_engaged), k > 0 D limited to gear k
    bool transmission_manual = false; // paddles took over in D
    bool transmission_sport = false;
    bool engine_running = false;
    double speed_mps = 0;
    double steering_wheel_deg = 0;
};

// Automatic gearbox plan A1: the shift keys on an automatic. From P/N/R at (near) standstill, shift up selects D and
// shift down selects R; in D the counters go to physics (paddles = manual mode). Returns the gear_request to write
// (kGearRequestNoChange = leave the channel), and whether the shift counters should still be forwarded.
constexpr double kGearRequestNoChange = -100.0;
constexpr double kGearRequestDrive = 99.0;
constexpr double kSelectorStandstillMps = 1.0;
struct SelectorShift {
    double gear_request = kGearRequestNoChange;
    bool release_park = false;
    bool forward_counts = true;
};
SelectorShift selector_shift(const DriverSnapshot& snapshot, int up, int down);
// The selector letter for HUD and cluster: "P", "R", "N", "D", "S" (sport) or "M<gear>" (paddles); "" on a manual car.
std::string selector_label(const DriverSnapshot& snapshot);

struct VehicleContext {
    DriverSnapshot snapshot;
    bool driving_enabled = true;
};

class ChannelSink {
public:
    virtual ~ChannelSink() = default;
    virtual double get(std::string_view channel) const = 0;
    virtual void set(std::string_view channel, double value) = 0;
};

struct ControlState {
    bool fitted = false;
    bool enabled = false;
    bool active = false;
    double value = 0;
    std::uint32_t lamp_segments = 0;
    bool blink = false;
    int lamp_state = 0;
    std::string colour_role = "off";
    std::string label;
    double aero_target_deg = 0;
};

struct CabinConfig {
    double crank_timeout_s = 0;
    double stop_max_speed_mps = 0;
    double blink_hz = 0;
    double indicator_arm_deg = 0;
    double indicator_cancel_deg = 0;
    double brake_lamp_threshold = 0;
};

class DriverActions {
public:
    // Called on the adapter/main thread only. Hold values are sampled levels;
    // edge detection lives here, not in a particular input adapter.
    bool submit(ActionEvent event);
    void apply(const VehicleContext& context, ChannelSink& channels);
    void discard_pending();
    void configure_cabin(const CabinConfig& config);
    void advance(const VehicleContext& context, ChannelSink& channels, double dt);
    void configure_auto_shift(bool enabled) { cabin_ = CabinState{}; cabin_.auto_shift = enabled; }
    const CabinState& cabin() const { return cabin_; }
    std::size_t pending_count() const { return pending_.size(); }

    // The one display projection, shared by every engine adapter/display.
    static ControlState state(std::string_view id, const DriverSnapshot& snapshot, const ChannelSink& channels);

private:
    CabinState cabin_;
    CabinConfig config_;
    bool cabin_configured_ = false;
    std::array<bool, 18> held_{};
    std::vector<ActionEvent> pending_;
};

} // namespace rg
