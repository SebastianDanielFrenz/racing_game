// channel_mirror.h - control-channel mirroring between the session's own channel store and ps::World (physics_sim
// assists I7c, vault PHYS-074). Engine-neutral, header-only; the double-valued twin of physics_sim's
// adapters/godot/src/channel_mirror.h (which is bound to atomic<float>).
//
// The Session re-sends EVERY channel from its atomic store to the World before each tick (push_channels). Since assists
// I7c the physics can WRITE a channel itself: a drive-mode change writes the new mode's traction_control_default /
// launch_control_default into assist.traction_control / assist.launch_control, and World::step applies that write after
// the tick. Without a mirror back, the next push puts the store's stale value over the physics' write. After the step,
// adopt_world_channel_writes() therefore compares each channel's World value with what the World held right before the
// step: a difference is a physics write and is adopted into the store with compare_exchange from the PUSHED value. A UI
// write (Session::set_control, main thread) made between the push and the step changed the store away from the pushed
// value, so the exchange fails and the UI value wins (the next push then sends it on).
//
// Both functions must see the SAME map with no insertion in between (the Session creates every key in its constructor),
// which makes the iteration order identical; the scratch vectors are index-aligned with it and sized once, so after the
// first tick nothing allocates.
#pragma once

#include <atomic>
#include <cstddef>
#include <string>
#include <vector>

namespace rg {

struct ChannelMirror {
    std::vector<double> pushed;       // the store's value that was pushed (the CAS expectation)
    std::vector<double> world_before; // the World's value right before the step (after any policy override)
};

// Channels: map-like, .first = name, .second = std::atomic<double>. set_world(name, value) pushes one channel.
template <class Channels, class SetWorld>
void push_channels(Channels& channels, ChannelMirror& mirror, SetWorld&& set_world) {
    if (mirror.pushed.size() != channels.size()) { // first tick only
        mirror.pushed.resize(channels.size());
        mirror.world_before.resize(channels.size());
    }
    std::size_t i = 0;
    for (auto& kv : channels) {
        const double v = kv.second.load(std::memory_order_relaxed);
        mirror.pushed[i] = v;
        mirror.world_before[i] = v;
        ++i;
        set_world(kv.first, v);
    }
}

// A policy (the unattended override) wrote `value` into the World itself after the push: that is not a physics write.
template <class Channels>
void note_world_override(Channels& channels, ChannelMirror& mirror, const std::string& name, double value) {
    std::size_t i = 0;
    for (auto& kv : channels) {
        if (i >= mirror.world_before.size()) return;
        if (kv.first == name) {
            mirror.world_before[i] = value;
            return;
        }
        ++i;
    }
}

// get_world(name) -> the World's value after the step. Returns the number of adopted channels.
template <class Channels, class GetWorld>
std::size_t adopt_world_channel_writes(Channels& channels, const ChannelMirror& mirror, GetWorld&& get_world) {
    std::size_t adopted = 0, i = 0;
    for (auto& kv : channels) {
        if (i >= mirror.pushed.size()) break;
        const double before = mirror.world_before[i];
        const double pushed = mirror.pushed[i];
        ++i;
        const double now = get_world(kv.first);
        if (now == before) continue;
        double expected = pushed;
        if (kv.second.compare_exchange_strong(expected, now, std::memory_order_relaxed)) ++adopted;
    }
    return adopted;
}

} // namespace rg
