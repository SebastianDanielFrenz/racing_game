// test_channel_mirror.cpp - rg/channel_mirror.h (physics_sim assists I7c): a channel the PHYSICS wrote during World::step
// is adopted into the session's store, but only while the store still holds the pushed value - a UI write made between
// the push and the step wins. Engine-neutral; the World is a plain map here.
#include "rg/channel_mirror.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <map>
#include <string>
#include <unordered_map>

namespace {

struct Rig {
    std::unordered_map<std::string, std::atomic<double>> store;
    std::map<std::string, double> world;
    rg::ChannelMirror mirror;
    Rig() {
        for (const char* n : {"steer", "assist.traction_control", "assist.launch_control"}) store[n].store(0.0);
    }
    void push() {
        rg::push_channels(store, mirror, [this](const std::string& n, double v) { world[n] = v; });
    }
    std::size_t adopt() {
        return rg::adopt_world_channel_writes(store, mirror, [this](const std::string& n) { return world[n]; });
    }
};

} // namespace

TEST_CASE("channel mirror: a physics write is adopted and the next push keeps it", "[channel_mirror][controls]") {
    Rig r;
    r.push();
    r.world["assist.launch_control"] = 1.0; // World::step applied a drive-mode default
    r.world["assist.traction_control"] = 2.0;
    CHECK(r.adopt() == 2);
    CHECK(r.store["assist.launch_control"].load() == 1.0);
    CHECK(r.store["assist.traction_control"].load() == 2.0);
    r.push(); // without the adoption this would put 0 over the physics' value
    CHECK(r.world["assist.launch_control"] == 1.0);
    CHECK(r.adopt() == 0);
}

TEST_CASE("channel mirror: a UI write between the push and the step wins over the physics write", "[channel_mirror][controls]") {
    Rig r;
    r.push();
    r.store["assist.traction_control"].store(3.0); // the player wrote this channel after the push
    r.world["assist.launch_control"] = 1.0;
    r.world["assist.traction_control"] = 2.0;
    CHECK(r.adopt() == 1); // launch control: the store still held the pushed 0, so the physics' 1 is adopted
    CHECK(r.store["assist.launch_control"].load() == 1.0);
    CHECK(r.store["assist.traction_control"].load() == 3.0); // changed since the push: the UI value stays
    r.push();
    CHECK(r.world["assist.traction_control"] == 3.0);
}

TEST_CASE("channel mirror: a policy override after the push is not a physics write", "[channel_mirror][controls]") {
    Rig r;
    r.store["steer"].store(0.25);
    r.push();
    r.world["steer"] = 0.0; // the unattended policy set the World value
    rg::note_world_override(r.store, r.mirror, "steer", 0.0);
    CHECK(r.adopt() == 0);
    CHECK(r.store["steer"].load() == 0.25);
}
