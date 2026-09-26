#include "rg/player_mode.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace rg {

ModeRules rules_for(PlayerMode mode) {
    ModeRules r;
    switch (mode) {
        case PlayerMode::Drive:
            r.implemented = true;
            r.vehicle_control = VehicleControl::Player;
            r.driving_inputs_live = true;
            r.camera_inputs_live = true; // chase-cam orbit (right stick / mouse)
            r.camera_rig = CameraRig::Chase;
            break;
        case PlayerMode::FreeCam:
            r.implemented = true;
            r.vehicle_control = VehicleControl::Unattended;
            r.driving_inputs_live = false;
            r.camera_inputs_live = true;
            r.camera_rig = CameraRig::Free;
            break;
        case PlayerMode::DroneFollow: r.camera_rig = CameraRig::Drone; break; // R9b
        case PlayerMode::Cockpit: r.camera_rig = CameraRig::Seat; break;      // R9b
        case PlayerMode::OnFoot: r.camera_rig = CameraRig::Walker; break;     // R9c
    }
    return r;
}

const char* to_string(PlayerMode mode) {
    switch (mode) {
        case PlayerMode::Drive: return "drive";
        case PlayerMode::FreeCam: return "free_cam";
        case PlayerMode::DroneFollow: return "drone_follow";
        case PlayerMode::Cockpit: return "cockpit";
        case PlayerMode::OnFoot: return "on_foot";
    }
    return "unknown";
}

const char* to_string(WorldKind kind) {
    switch (kind) {
        case WorldKind::Flat: return "flat";
        case WorldKind::RealWorld: return "real_world";
    }
    return "unknown";
}

const char* to_string(WorldPhase phase) {
    switch (phase) {
        case WorldPhase::None: return "none";
        case WorldPhase::Loading: return "loading";
        case WorldPhase::Ready: return "ready";
        case WorldPhase::Failed: return "failed";
    }
    return "unknown";
}

const char* to_string(VehicleControl control) {
    switch (control) {
        case VehicleControl::Player: return "player";
        case VehicleControl::Unattended: return "unattended";
    }
    return "unknown";
}

const char* to_string(CameraRig rig) {
    switch (rig) {
        case CameraRig::Chase: return "chase";
        case CameraRig::Free: return "free";
        case CameraRig::Drone: return "drone";
        case CameraRig::Seat: return "seat";
        case CameraRig::Walker: return "walker";
    }
    return "unknown";
}

std::optional<PlayerMode> player_mode_from_string(std::string_view name) {
    for (int i = 0; i < kPlayerModeCount; ++i) {
        const auto m = static_cast<PlayerMode>(i);
        if (name == to_string(m)) return m;
    }
    return std::nullopt;
}

std::optional<WorldKind> world_kind_from_string(std::string_view name) {
    if (name == to_string(WorldKind::Flat)) return WorldKind::Flat;
    if (name == to_string(WorldKind::RealWorld)) return WorldKind::RealWorld;
    return std::nullopt;
}

UnattendedControls unattended_controls(double speed_mps) {
    UnattendedControls c;
    if (std::fabs(speed_mps) > kUnattendedHoldSpeedMps) {
        c.brake = kUnattendedRollingBrake;
        c.handbrake = 0.0;
    } else {
        c.brake = 1.0;
        c.handbrake = 1.0;
    }
    return c;
}

PlayerModeMachine::PlayerModeMachine(PlayerMode start) : mode_(start) {
    if (!rules_for(start).implemented) {
        throw std::invalid_argument(std::string("PlayerModeMachine: start mode '") + to_string(start) +
                                    "' is not implemented");
    }
}

PlayerModeMachine::Result PlayerModeMachine::request_mode(PlayerMode mode) {
    if (!rules_for(mode).implemented) return Result::NotImplemented;
    if (mode == mode_) return Result::NoChange;
    mode_ = mode;
    ++revision_;
    return Result::Changed;
}

PlayerMode PlayerModeMachine::cycle_mode() {
    int i = static_cast<int>(mode_);
    for (int step = 1; step < kPlayerModeCount; ++step) {
        const auto candidate = static_cast<PlayerMode>((i + step) % kPlayerModeCount);
        if (rules_for(candidate).implemented) {
            request_mode(candidate);
            break;
        }
    }
    return mode_;
}

std::uint64_t PlayerModeMachine::begin_world_load(WorldKind kind) {
    world_kind_ = kind;
    world_phase_ = WorldPhase::Loading;
    ++world_serial_;
    ++revision_;
    return world_serial_;
}

bool PlayerModeMachine::finish_world_load(std::uint64_t serial, bool ok) {
    if (serial == 0 || serial != world_serial_ || world_phase_ != WorldPhase::Loading) return false;
    world_phase_ = ok ? WorldPhase::Ready : WorldPhase::Failed;
    ++revision_;
    return true;
}

ModeRules PlayerModeMachine::effective_rules() const {
    ModeRules r = rules_for(mode_);
    if (world_phase_ != WorldPhase::Ready) {
        r.driving_inputs_live = false;
        r.vehicle_control = VehicleControl::Unattended;
    }
    return r;
}

} // namespace rg
