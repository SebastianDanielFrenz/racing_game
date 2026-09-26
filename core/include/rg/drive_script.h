// rg/drive_script.h — rg::DriveScript: tick-indexed, open-loop vehicle
// controls for rg::Session (R2.2 R4, Session::set_drive_script). Godot-free
// and wall-clock-free: a script is a function of the drive tick only, so a
// scripted run is as deterministic as the World itself.
//
// Ticks are DRIVE ticks: stepped World ticks since the vehicle was spawned
// (Session::drive_tick() - in terrain mode the World has already stepped its
// priming ticks by then, so World::tick() would not start at 0; frozen tick
// attempts never count).
//
// Events are sample-and-hold: apply(ctx, world) calls World::set_control for
// every event with tick <= ctx.drive_tick that has not been applied yet, in
// (tick, insertion) order - World keeps the value until a later event
// changes it. Applying from any drive tick therefore leaves World's controls
// exactly as if the script had run from tick 0 (a script installed mid-run
// catches up at once).
//
// Extension point (R5, the waypoint autopilot): set_controller() installs a
// closed-loop per-tick hook that runs AFTER the tick's events and may read
// sim state through the World (chassis pose, wheel state) and set any
// control. It must be deterministic (no wall clock, no randomness).
#pragma once

#include "ps/world/ids.h"
#include "ps/world/world.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace rg {

struct DriveEvent {
    std::uint64_t tick = 0; // drive tick at (the start of) which the value takes effect
    std::string channel;    // a World control channel ("throttle", "steer", "assist.auto_shift", ...)
    double value = 0.0;
};

// What Session hands the script once per stepped tick, right before
// World::step().
struct DriveTickContext {
    std::uint64_t drive_tick = 0;
    ps::BodyId chassis{};
    ps::VehicleId vehicle{};
};

class DriveScript {
public:
    using Controller = std::function<void(const DriveTickContext&, ps::World&)>;

    // Adds an event; returns *this so a script reads as a chain of at(...)
    // calls. Events may be added in any order (stable-sorted by tick).
    DriveScript& at(std::uint64_t tick, std::string channel, double value) {
        events_.push_back(DriveEvent{tick, std::move(channel), value});
        std::stable_sort(events_.begin(), events_.end(),
                         [](const DriveEvent& a, const DriveEvent& b) { return a.tick < b.tick; });
        cursor_ = 0;
        return *this;
    }

    DriveScript& set_controller(Controller controller) {
        controller_ = std::move(controller);
        return *this;
    }

    // Session calls this once per stepped tick, before World::step().
    void apply(const DriveTickContext& ctx, ps::World& world) {
        while (cursor_ < events_.size() && events_[cursor_].tick <= ctx.drive_tick) {
            world.set_control(events_[cursor_].channel, events_[cursor_].value);
            ++cursor_;
        }
        if (controller_) controller_(ctx, world);
    }

    // Forgets which events were applied (the next apply() replays from the
    // first event). Session::set_drive_script calls this.
    void rewind() { cursor_ = 0; }

    [[nodiscard]] const std::vector<DriveEvent>& events() const { return events_; }
    [[nodiscard]] bool empty() const { return events_.empty() && !controller_; }

private:
    std::vector<DriveEvent> events_; // stable-sorted by tick
    std::size_t cursor_ = 0;         // events_[0, cursor_) have been applied
    Controller controller_;
};

} // namespace rg
