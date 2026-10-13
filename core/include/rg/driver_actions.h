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

// C1 extends this game-owned state; the EPB latch belongs to physics, never here.
struct CabinState {
    bool auto_shift = false;
    bool hazard = false;
    int indicator = 0;
    int lights = 0;
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
};

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

class DriverActions {
public:
    // Called on the adapter/main thread only. Hold values are sampled levels;
    // edge detection lives here, not in a particular input adapter.
    bool submit(ActionEvent event);
    void apply(const VehicleContext& context, ChannelSink& channels);
    void discard_pending();
    void configure_auto_shift(bool enabled) { cabin_.auto_shift = enabled; }
    const CabinState& cabin() const { return cabin_; }
    std::size_t pending_count() const { return pending_.size(); }

    // The one display projection, shared by every engine adapter/display.
    static ControlState state(std::string_view id, const DriverSnapshot& snapshot, const ChannelSink& channels);

private:
    CabinState cabin_;
    std::array<bool, 8> held_{};
    std::vector<ActionEvent> pending_;
};

} // namespace rg
