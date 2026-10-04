#include "rg/player_mode.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace rg {

ModeRules rules_for(PlayerMode mode, std::optional<std::uint64_t> drone_target) {
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
        case PlayerMode::DroneFollow:
            r.implemented = true;
            r.camera_rig = CameraRig::Drone;
            r.camera_inputs_live = true; // orbit / zoom
            // Own car: the player keeps driving. Another vehicle: the car is
            // Unattended like in free cam.
            r.vehicle_control = drone_target ? VehicleControl::Unattended : VehicleControl::Player;
            r.driving_inputs_live = !drone_target.has_value();
            break;
        case PlayerMode::Cockpit: r.camera_rig = CameraRig::Seat; break;      // reserved, unused
        case PlayerMode::OnFoot:
            r.implemented = true;
            r.vehicle_control = VehicleControl::Unattended; // parked, like in free cam
            r.driving_inputs_live = false;
            r.camera_inputs_live = true;  // look
            r.walking_inputs_live = true; // move / run / jump / interact
            r.camera_rig = CameraRig::Walker;
            break;
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

std::optional<std::uint64_t> next_drone_target(std::optional<std::uint64_t> current,
                                               std::span<const DroneCandidate> candidates, double ref_x,
                                               double ref_y, double max_range_m) {
    struct Key {
        double d2;
        std::uint64_t id;
    };
    const auto less = [](const Key& a, const Key& b) { return a.d2 < b.d2 || (a.d2 == b.d2 && a.id < b.id); };
    const double range2 = max_range_m * max_range_m;
    const auto key_of = [&](const DroneCandidate& c, Key& out) {
        const double dx = c.x - ref_x, dy = c.y - ref_y;
        const double d2 = dx * dx + dy * dy;
        if (!std::isfinite(d2) || !(d2 <= range2)) return false;
        out = Key{d2, c.id};
        return true;
    };

    Key after{0.0, 0};
    bool have_after = false; // false = "from the own car": every in-range candidate qualifies
    if (current) {
        for (const DroneCandidate& c : candidates) {
            Key k{};
            if (c.id == *current && key_of(c, k)) {
                after = k;
                have_after = true;
                break;
            }
        }
        if (!have_after) return std::nullopt; // gone or out of range: back to the own car
    }

    bool found = false;
    Key best{0.0, 0};
    for (const DroneCandidate& c : candidates) {
        Key k{};
        if (!key_of(c, k)) continue;
        if (have_after && !less(after, k)) continue;
        if (!found || less(k, best)) {
            best = k;
            found = true;
        }
    }
    if (!found) return std::nullopt;
    return best.id;
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
    drone_target_.reset(); // entering or leaving DroneFollow starts on the own car
    ++revision_;
    return Result::Changed;
}

PlayerMode PlayerModeMachine::cycle_mode() { return cycle_mode(0.0); }

PlayerModeMachine::Result PlayerModeMachine::request_mode(PlayerMode mode, double vehicle_speed_mps) {
    if (mode == PlayerMode::OnFoot && mode_ != PlayerMode::OnFoot && rules_for(mode).implemented &&
        !may_get_out(vehicle_speed_mps)) {
        ++get_out_refusals_;
        return Result::Refused;
    }
    return request_mode(mode);
}

PlayerMode PlayerModeMachine::cycle_mode(double vehicle_speed_mps) {
    int i = static_cast<int>(mode_);
    for (int step = 1; step < kPlayerModeCount; ++step) {
        const auto candidate = static_cast<PlayerMode>((i + step) % kPlayerModeCount);
        if (!rules_for(candidate).implemented) continue;
        if (request_mode(candidate, vehicle_speed_mps) == Result::Refused) continue; // too fast to get out: skip it
        break;
    }
    return mode_;
}

bool PlayerModeMachine::set_drone_target(std::optional<std::uint64_t> target) {
    if (mode_ != PlayerMode::DroneFollow) return false;
    if (drone_target_ == target) return true;
    drone_target_ = target;
    ++revision_;
    return true;
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
    ModeRules r = rules_for(mode_, drone_target_);
    if (world_phase_ != WorldPhase::Ready) {
        r.driving_inputs_live = false;
        r.walking_inputs_live = false;
        r.vehicle_control = VehicleControl::Unattended;
    }
    return r;
}

} // namespace rg
