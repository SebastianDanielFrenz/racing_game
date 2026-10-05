// test_traffic_stuck.cpp - the NPC stuck detector (rg/traffic_stuck.h) and the controller rules the stuck-traffic
// investigation changed (docs/npc_traffic.md, "Stuck traffic"). Every Session case names the sabotage that makes it
// fail (re-measured when the case was written).

#include "rg/session.h"
#include "rg/traffic_stuck.h"

#include "ps/math/quat.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <map>
#include <string>

namespace {

rg::SessionConfig make_test_config() {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json";
    config.job_workers = 1;
    return config;
}

ps::Pose pose_at(double x, double y, double yaw_rad) {
    ps::Pose pose;
    pose.position = ps::Vec3{x, y, 0.8};
    pose.orientation = ps::Quat::from_axis_angle(ps::Vec3::unit_z(), yaw_rad);
    return pose;
}

constexpr double kPi = 3.14159265358979323846;

// Steps `seconds` of sim time keeping the given actors "seen" (a flat-mode surplus actor is removed after 3 unseen s).
void run_seen(rg::Session& session, const std::vector<std::uint64_t>& ids, double seconds) {
    const int steps = static_cast<int>(seconds * 240.0);
    for (int k = 0; k < steps; ++k) {
        if (k % 120 == 0) session.set_visible_traffic(ids);
        session.step();
    }
}

} // namespace

TEST_CASE("stuck track declares once after 5 s below 0.5 m/s, recovers at 1 m/s, never while arriving", "[traffic_stuck]") {
    rg::StuckTrack t;
    int declared = 0;
    for (int k = 0; k < 1300; ++k)
        if (rg::advance_stuck_track(t, 0.2, false, 1.0 / 240.0) == rg::StuckTrackEvent::Declared) ++declared;
    CHECK(declared == 1); // 5 s sim time, declared once, not every tick afterwards
    CHECK(t.declared);
    CHECK(rg::advance_stuck_track(t, 0.7, false, 0.1) == rg::StuckTrackEvent::None); // creeping: same episode
    CHECK(rg::advance_stuck_track(t, 1.2, false, 0.1) == rg::StuckTrackEvent::Recovered);
    CHECK_FALSE(t.declared);

    rg::StuckTrack arriving;
    for (int k = 0; k < 2400; ++k) CHECK(rg::advance_stuck_track(arriving, 0.0, true, 1.0 / 240.0) == rg::StuckTrackEvent::None);

    rg::StuckTrack blip; // a 0.4 s pause now and then never accumulates
    for (int round = 0; round < 20; ++round) {
        for (int k = 0; k < 96; ++k) CHECK(rg::advance_stuck_track(blip, 0.1, false, 1.0 / 240.0) == rg::StuckTrackEvent::None);
        CHECK(rg::advance_stuck_track(blip, 3.0, false, 1.0 / 240.0) == rg::StuckTrackEvent::None);
    }
}

TEST_CASE("stuck chain: cycle root is the smallest id, a coasting follower does not end the chain", "[traffic_stuck]") {
    std::map<std::uint64_t, rg::StuckNode> nodes;
    const auto follow = [](std::uint64_t other, double speed) {
        rg::StuckNode n;
        n.cause = rg::StuckCause::FollowNpc;
        n.other_id = other;
        n.speed_mps = speed;
        return n;
    };
    const rg::StuckLookup lookup = [&](std::uint64_t id) -> std::optional<rg::StuckNode> {
        const auto it = nodes.find(id);
        if (it == nodes.end()) return std::nullopt;
        return it->second;
    };

    // 9 -> 4 -> 7 -> 4: the cycle is {4, 7}; 9 is a member of the queue behind it.
    nodes[9] = follow(4, 0.0);
    nodes[4] = follow(7, 0.0);
    nodes[7] = follow(4, 0.0);
    auto c = rg::walk_stuck_chain(9, lookup);
    CHECK(c.end == rg::StuckChainEnd::Cycle);
    CHECK(c.cycle_len == 2);
    CHECK(c.root_id == 4);

    // 20 -> 21 (still coasting at 0.6 m/s, bound by an NPC) -> 22 (route cap 0): the chain must reach 22.
    // Sabotage: ending the chain at any actor above 0.5 m/s reports Moving here (that mislabelled 678 of 743 events).
    nodes.clear();
    nodes[20] = follow(21, 0.0);
    nodes[21] = follow(22, 0.6);
    rg::StuckNode cap;
    cap.cause = rg::StuckCause::RouteCap;
    nodes[22] = cap;
    c = rg::walk_stuck_chain(20, lookup);
    CHECK(c.end == rg::StuckChainEnd::RouteCap);
    CHECK(c.root_id == 22);
    CHECK(c.length == 3);

    // A free-driving head above the threshold is a Moving end (a queue discharging, not a deadlock).
    rg::StuckNode free_head;
    free_head.cause = rg::StuckCause::RouteCap;
    free_head.speed_mps = 4.0;
    nodes[22] = free_head;
    CHECK(rg::walk_stuck_chain(20, lookup).end == rg::StuckChainEnd::Moving);

    // A lost actor ends the chain as Lost.
    nodes.erase(22);
    CHECK(rg::walk_stuck_chain(20, lookup).end == rg::StuckChainEnd::Lost);
}

TEST_CASE("stuck stats count causes, distinct roots and recoveries", "[traffic_stuck]") {
    rg::TrafficStuckStats stats;
    rg::StuckEvent e;
    e.cause = rg::StuckCause::FollowNpc;
    e.chain.end = rg::StuckChainEnd::Cycle;
    e.chain.root_id = 4;
    e.chain.cycle_len = 2;
    stats.record(e);
    e.chain.root_id = 4; // the same deadlock reported by a queued member: one root
    stats.record(e);
    e.chain.root_id = 5;
    stats.record(e);
    stats.note_recovered(12.0);
    const auto& s = stats.summary();
    CHECK(s.events == 3);
    CHECK(s.roots == 2);
    CHECK(s.by_cause[static_cast<int>(rg::StuckCause::FollowNpc)] == 3);
    CHECK(s.recovered == 1);
    CHECK(s.longest_recovered_s == Catch::Approx(12.0));
    const auto text = stats.format_summary(30.0);
    CHECK(text.find("RG_TRAFFIC_STUCK_SUMMARY") == 0);
    CHECK(text.find("follow_npc=3") != std::string::npos);
}

// Two NPCs facing each other in the same lane used to obey each other as obstacles (and the probe ray hit the other
// body): a permanent mutual stop (153 of 190 distinct deadlock roots in the real-world measurement).
// Sabotage: removing the `other_cos < -0.5` skip in the follow rule AND the traffic-body filter on the probe makes
// both actors stop 12-20 m apart and be declared stuck.
TEST_CASE("oncoming NPCs on one line pass each other instead of deadlocking", "[traffic_stuck][session]") {
    rg::Session session(make_test_config());
    const auto a = session.add_test_traffic_actor(pose_at(0.0, 400.0, 0.0), 8.0);
    const auto b = session.add_test_traffic_actor(pose_at(70.0, 400.0, kPi), 8.0);
    run_seen(session, {a, b}, 14.0);
    REQUIRE(session.traffic_actor_position(a).has_value());
    REQUIRE(session.traffic_actor_position(b).has_value());
    CHECK(session.traffic_actor_position(a)->x > 90.0);  // through the other one and beyond
    CHECK(session.traffic_actor_position(b)->x < -20.0);
    CHECK(session.traffic_stuck_stats().summary().events == 0);
}

// A crossing conflict resolves by id: the lower id goes, the higher yields. Both start inside each other's corridor
// at walking pace (a faster pair simply drives through the overlap before it can stop).
// Sabotage: dropping the id test stops both dead at the crossing until the 8 s patience timeout.
TEST_CASE("crossing NPCs: the lower id goes first, nobody waits for the patience timeout", "[traffic_stuck][session]") {
    rg::Session session(make_test_config());
    const auto a = session.add_test_traffic_actor(pose_at(-2.0, 400.0, 0.0), 1.0);           // east
    const auto b = session.add_test_traffic_actor(pose_at(0.0, 398.0, kPi / 2.0), 1.0);      // north, across a's nose
    REQUIRE(a < b);
    run_seen(session, {a, b}, 10.0);
    REQUIRE(session.traffic_actor_position(a).has_value());
    REQUIRE(session.traffic_actor_position(b).has_value());
    CHECK(session.traffic_actor_position(a)->x > 2.0);   // a went at once
    CHECK(session.traffic_actor_position(b)->y > 401.0); // b followed once a had cleared its path
    CHECK(session.traffic_stuck_stats().summary().crossing_overrides == 0);
}

// Two NPCs side by side (lateral 1.6 m, headings 20 deg apart: merging lanes at a junction mouth) each saw the
// other "ahead" by a few centimetres and both stopped for ever (the 483/607 cycle, 165 queued behind it).
// Sabotage: removing the `clearance <= 0 && side > 1.2 && other_id > a.id` tie-break stops both.
TEST_CASE("overlapping side-by-side NPCs do not yield to each other mutually", "[traffic_stuck][session]") {
    rg::Session session(make_test_config());
    const double lean = 10.0 * kPi / 180.0;
    const auto a = session.add_test_traffic_actor(pose_at(0.0, 400.0, lean), 1.0);
    const auto b = session.add_test_traffic_actor(pose_at(0.2, 401.6, -lean), 1.0);
    run_seen(session, {a, b}, 6.0);
    REQUIRE(session.traffic_actor_position(a).has_value());
    CHECK(session.traffic_actor_position(a)->x > 3.0); // the lower id keeps going (mutual yield: stays at ~0)
}

// A planned trip ends with a speed-0 point; the point before it can be up to one spacing earlier, so the actor stops
// there. A fixed 0.5 m arrival margin left such an actor (0.7 m short) parked on the road for ever.
// Sabotage: arrive_margin = 0.5 (the old constant) leaves the actor alive at the end of this test.
TEST_CASE("an actor that stops one route spacing short of the end still despawns on arrival", "[traffic_stuck][session]") {
    rg::Session session(make_test_config());
    const auto a = session.add_test_traffic_actor(pose_at(0.0, 400.0, 0.0), 6.0, 60.0, true);
    CHECK(session.traffic_actor_count() == 1);
    for (int k = 0; k < 240 * 20 && session.traffic_actor_count() > 0; ++k) {
        if (k % 120 == 0) session.set_visible_traffic({a});
        session.step();
    }
    CHECK(session.traffic_actor_count() == 0);
}
