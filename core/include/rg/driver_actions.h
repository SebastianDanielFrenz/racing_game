#pragma once

#include "rg/assist_info.h"

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace rg {

enum class DriverActionKind { Press, Release, Set, Hold };
enum class ActionSource { Keyboard, Pad, Hud, CockpitMouse, VrPoke, VrGrab, VrRay, Script };

// Headlamp switch position (the "lights" action cycles Auto -> On -> Off -> Auto).
// Auto: dipped beam follows the light sensor (CabinConfig auto_lights_*); On: dipped
// beam (main beam stays its own choice, Set 2); Off: no lamps. Channel cabin.light_mode
// carries the numeric value (0 auto, 1 on, 2 off).
enum class LightMode { Auto = 0, On = 1, Off = 2 };
const char* light_mode_name(LightMode mode); // "auto", "on", "off"
bool parse_light_mode(std::string_view name, LightMode& out);

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
    LightMode light_mode = LightMode::Auto; // the switch position; survives a vehicle change (configure_auto_shift)
    int wipers = 0;
    double auto_crank_elapsed_s = 0;
    // Automatic headlamps (light sensor): the sensor's last decision and how
    // long the opposite condition has held. Not valid until the first reading.
    bool light_sensor_valid = false;
    bool light_sensor_dark = false;
    double light_sensor_timer_s = 0;
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
    std::uint32_t esc_modes = 0; // bit n = ESC mode code n offered (0 off, 1 on, 2 sport)
    int esc_effective = 0; // 0 off, 1 on, 2 sport
    bool esc_intervening = false;
    int gear = 0;
    // Automatic gearbox plan A1 (physics automatic_tcu): 0 manual, 1 torque-converter automatic, 2 dual-clutch.
    int transmission_kind = 0;
    int transmission_selector = 0; // -1 R, 0 N (P when park_engaged), k > 0 D limited to gear k
    bool transmission_manual = false; // paddles took over in D
    bool transmission_sport = false;
    bool engine_running = false;
    double clutch_engagement = 1.0; // effective engagement of the engine clutch (1 on a car without one)
    bool hybrid_fitted=false;
    bool electric_ready = false;
    double speed_mps = 0;
    double steering_wheel_deg = 0;
    // Light sensor reading (horizontal illuminance, lux; rg::SkyLighting::illuminance_lux).
    // NaN = no reading: the automatic headlamps hold their state.
    double ambient_lux = std::numeric_limits<double>::quiet_NaN();
};

// Automatic gearbox: on an automatic or dual-clutch car the shift up / down keys step the selector along
// P <-> R <-> N <-> D, one position per press (shift up from P goes R, N, D; shift down from D goes N, R, P). Stepping
// into P or R is refused while the car rolls faster than kSelectorStandstillMps (`refused`, nothing changes). In D with
// manual mode on (the physics' own flag) the counters are forwarded and change gears (tiptronic). Returns the
// gear_request and park_lock to write (kGearRequestNoChange / -1 = leave the channel), and whether the shift counters
// should still be forwarded. A manual-gearbox car gets no change and forward_counts true.
constexpr double kGearRequestNoChange = -100.0;
constexpr double kGearRequestDrive = 99.0;
constexpr double kSelectorStandstillMps = 1.0;
struct SelectorShift {
    double gear_request = kGearRequestNoChange;
    int park_request = -1; // -1 no change, 0 release the park lock, 1 engage it
    bool forward_counts = true;
    bool refused = false;
};
SelectorShift selector_shift(const DriverSnapshot& snapshot, int up, int down);
// A car with the P R N D selector: an automatic or dual-clutch box, a single-speed electric drive, or ANY hybrid whatever its
// gearbox (the physics hybrid supervisor owns the gearbox and takes its D/N/R from gear_request; owner decision 2026-10-18).
bool has_selector(const DriverSnapshot& snapshot);
// The selector letter for HUD and cluster: "P", "R", "N", "D", "S" (sport) or "M<gear>" (manual mode); "" on a manual car.
std::string selector_label(const DriverSnapshot& snapshot);
// The text of the gear indicator of a selector car, "" on a manual car: the selector letter, plus the gearbox gear after D/S
// when it drives the wheels. An automatic shows it whenever a gear is in; a hybrid only while the combustion engine is
// actually coupled (engine running, gear in, clutch engaged: "D3"); an electric-only hybrid or a pure EV shows just "D".
std::string selector_display(const DriverSnapshot& snapshot);

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
    // Hybrid manual mode (tiptronic): the physics hybrid supervisor re-sends gear_request 99 whatever the game writes, so the usual
    // "change gear_request" exit never reaches the gearbox. Leaving manual mode therefore requests N for this long (the box drops
    // manual mode on the selector change) and then D again.
    double manual_exit_hold_s = 0.05;
    // Automatic headlamps (LightMode::Auto, the default switch position): dipped
    // beam on once the sensor reads below on_lux for on_delay_s, off once it reads
    // above off_lux for off_delay_s (hysteresis: on_lux <= off_lux). The first
    // reading decides at once (a night spawn starts with lights on). Universal:
    // one pair of thresholds for every car and every weather (weather only
    // changes the illuminance the sensor reads). Main beam is kept while dark.
    // auto_lights false: no sensor, Auto holds whatever the lamps were.
    bool auto_lights = false;
    double auto_lights_on_lux = 0;
    double auto_lights_off_lux = 0;
    double auto_lights_on_delay_s = 0;
    double auto_lights_off_delay_s = 0;
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
    // A vehicle change resets the cabin; the headlamp switch position is the player's and is kept.
    void configure_auto_shift(bool enabled) {
        const LightMode mode = cabin_.light_mode;
        cabin_ = CabinState{};
        cabin_.auto_shift = enabled;
        cabin_.light_mode = mode;
    }
    // The persisted switch position (settings cabin.light_mode, restored at start-up).
    void set_light_mode(LightMode mode);
    LightMode light_mode() const { return cabin_.light_mode; }
    const CabinState& cabin() const { return cabin_; }
    std::size_t pending_count() const { return pending_.size(); }

    // The one display projection, shared by every engine adapter/display.
    static ControlState state(std::string_view id, const DriverSnapshot& snapshot, const ChannelSink& channels);

private:
    CabinState cabin_;
    CabinConfig config_;
    bool cabin_configured_ = false;
    bool manual_exit_pending_ = false;
    double manual_exit_elapsed_s_ = 0;
    void update_auto_lights(double lux, double dt);
    void apply_light_mode();
    std::array<bool, 18> held_{};
    std::vector<ActionEvent> pending_;
};

} // namespace rg
