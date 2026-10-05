#include "rg/traffic_stuck.h"
#include <algorithm>
#include <cstdio>

namespace rg {

const char* stuck_cause_name(StuckCause c) {
    switch (c) {
    case StuckCause::FollowPlayer: return "follow_player";
    case StuckCause::FollowNpc: return "follow_npc";
    case StuckCause::FollowTruck: return "follow_truck";
    case StuckCause::ProbePlayer: return "probe_player";
    case StuckCause::ProbeNpc: return "probe_npc";
    case StuckCause::ProbeTruck: return "probe_truck";
    case StuckCause::ProbeTerrain: return "probe_terrain";
    case StuckCause::ProbeDeck: return "probe_deck";
    case StuckCause::ProbeStatic: return "probe_static";
    case StuckCause::ProbeOther: return "probe_other";
    case StuckCause::RouteCap: return "route_cap";
    case StuckCause::Other: return "other";
    case StuckCause::Count: break;
    }
    return "?";
}

const char* stuck_chain_end_name(StuckChainEnd e) {
    switch (e) {
    case StuckChainEnd::None: return "none";
    case StuckChainEnd::Moving: return "moving";
    case StuckChainEnd::Player: return "player";
    case StuckChainEnd::Truck: return "truck";
    case StuckChainEnd::Terrain: return "terrain";
    case StuckChainEnd::Deck: return "deck";
    case StuckChainEnd::Static: return "static";
    case StuckChainEnd::RouteCap: return "route_cap";
    case StuckChainEnd::Cycle: return "cycle";
    case StuckChainEnd::Lost: return "lost";
    case StuckChainEnd::Other: return "other";
    case StuckChainEnd::Count: break;
    }
    return "?";
}

bool stuck_cause_waits_for_npc(StuckCause c) { return c == StuckCause::FollowNpc || c == StuckCause::ProbeNpc; }

StuckTrackEvent advance_stuck_track(StuckTrack& t, double speed, bool arriving, double dt) {
    if (arriving) {
        t.below_s = 0.0;
        t.declared = false;
        return StuckTrackEvent::None;
    }
    if (speed < kStuckSpeedMps) {
        t.below_s += dt;
        if (!t.declared && t.below_s > kStuckTimeS) {
            t.declared = true;
            return StuckTrackEvent::Declared;
        }
        return StuckTrackEvent::None;
    }
    if (t.declared) {
        if (speed >= 2.0 * kStuckSpeedMps) {
            t.declared = false;
            t.below_s = 0.0;
            return StuckTrackEvent::Recovered;
        }
        return StuckTrackEvent::None; // creeping: still the same episode
    }
    t.below_s = 0.0;
    return StuckTrackEvent::None;
}

StuckChain walk_stuck_chain(std::uint64_t id, const StuckLookup& lookup) {
    StuckChain chain;
    chain.last_id = id;
    chain.root_id = id;
    auto node = lookup(id);
    if (!node) {
        chain.end = StuckChainEnd::Lost;
        return chain;
    }
    if (!stuck_cause_waits_for_npc(node->cause)) return chain; // length 1, end None
    std::uint64_t visited[64];
    StuckNode visited_nodes[64];
    int count = 0;
    visited_nodes[count] = *node;
    visited[count++] = id;
    for (;;) {
        const std::uint64_t next = node->other_id;
        const auto at = std::find(visited, visited + count, next);
        if (at != visited + count) {
            chain.end = StuckChainEnd::Cycle;
            chain.last_id = next;
            const int first = static_cast<int>(at - visited);
            chain.cycle_len = count - first;
            int root = first;
            for (int i = first; i < count; ++i)
                if (visited[i] < visited[root]) root = i;
            chain.root_id = visited[root];
            chain.root_heading_cos = visited_nodes[root].heading_cos;
            chain.root_has_heading = visited_nodes[root].has_heading;
            return chain;
        }
        auto n = lookup(next);
        ++chain.length;
        chain.last_id = next;
        chain.root_id = next;
        if (!n) {
            chain.end = StuckChainEnd::Lost;
            return chain;
        }
        // An actor whose binding cap is an NPC keeps the chain going whatever its instantaneous speed (a car still
        // coasting down at 0.6 m/s behind a stopped one is not a moving head); only an actor that is free to
        // drive (no NPC cap binding) and above the stuck threshold ends it as Moving.
        if (!stuck_cause_waits_for_npc(n->cause) && n->speed_mps >= kStuckSpeedMps) {
            chain.end = StuckChainEnd::Moving;
            return chain;
        }
        if (stuck_cause_waits_for_npc(n->cause)) {
            if (count >= 64) {
                chain.end = StuckChainEnd::Other;
                return chain;
            }
            visited_nodes[count] = *n;
            visited[count++] = next;
            node = n;
            continue;
        }
        switch (n->cause) {
        case StuckCause::FollowPlayer:
        case StuckCause::ProbePlayer: chain.end = StuckChainEnd::Player; break;
        case StuckCause::FollowTruck:
        case StuckCause::ProbeTruck: chain.end = StuckChainEnd::Truck; break;
        case StuckCause::ProbeTerrain: chain.end = StuckChainEnd::Terrain; break;
        case StuckCause::ProbeDeck: chain.end = StuckChainEnd::Deck; break;
        case StuckCause::ProbeStatic: chain.end = StuckChainEnd::Static; break;
        case StuckCause::RouteCap: chain.end = StuckChainEnd::RouteCap; break;
        default: chain.end = StuckChainEnd::Other; break;
        }
        return chain;
    }
}

void TrafficStuckStats::clear() {
    summary_ = StuckSummary{};
    events_.clear();
    roots_seen_.clear();
}

void TrafficStuckStats::record(const StuckEvent& e) {
    ++summary_.events;
    ++summary_.by_cause[static_cast<int>(e.cause)];
    if (stuck_cause_waits_for_npc(e.cause)) {
        ++summary_.by_chain_end[static_cast<int>(e.chain.end)];
        if (e.has_other) ++summary_.by_relation[e.other_heading_cos < -0.5 ? 0 : e.other_heading_cos > 0.5 ? 2 : 1];
    }
    if (roots_seen_.insert(e.chain.root_id).second) {
        ++summary_.roots;
        ++summary_.roots_by_end[static_cast<int>(e.chain.end)];
        if (e.chain.end == StuckChainEnd::Cycle) {
            ++summary_.root_cycle_len[e.chain.cycle_len <= 2 ? 0 : e.chain.cycle_len == 3 ? 1 : 2];
            if (e.chain.root_has_heading)
                ++summary_.root_cycle_relation[e.chain.root_heading_cos < -0.5 ? 0 : e.chain.root_heading_cos > 0.5 ? 2 : 1];
        }
    }
    if (events_.size() < kMaxEvents) events_.push_back(e);
}

void TrafficStuckStats::note_recovered(double stuck_s) {
    ++summary_.recovered;
    summary_.longest_recovered_s = std::max(summary_.longest_recovered_s, stuck_s);
}

void TrafficStuckStats::note_backstop() { ++summary_.backstop_despawns; }

void TrafficStuckStats::set_active(std::uint64_t n) {
    summary_.active = n;
    summary_.peak_active = std::max(summary_.peak_active, n);
}

std::string TrafficStuckStats::format_event(const StuckEvent& e) {
    char buf[640];
    const auto cap = [](double v) { return v >= 1e29 ? -1.0 : v; };
    std::snprintf(buf, sizeof(buf),
                  "RG_TRAFFIC_STUCK id=%llu t=%.1f truck=%d cause=%s other=%llu chain=%d chain_end=%s last=%llu "
                  "pos=(%.1f,%.1f) station=%.1f/%.1f way=%lld caps(route=%.2f obstacle=%.2f follow=%.2f) other(cos=%.2f lat=%.1f gap=%.1f)",
                  static_cast<unsigned long long>(e.id), e.time_s, e.truck ? 1 : 0, stuck_cause_name(e.cause),
                  static_cast<unsigned long long>(e.other_id), e.chain.length, stuck_chain_end_name(e.chain.end),
                  static_cast<unsigned long long>(e.chain.last_id), e.x, e.y, e.station_m, e.route_length_m,
                  static_cast<long long>(e.way_id), cap(e.route_cap), cap(e.obstacle_cap), cap(e.follow_cap), e.other_heading_cos, e.other_lateral_m, e.other_gap_m);
    return buf;
}

std::string TrafficStuckStats::format_summary(double time_s) const {
    std::string out;
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "RG_TRAFFIC_STUCK_SUMMARY t=%.0f events=%llu active=%llu peak_active=%llu recovered=%llu backstop=%llu overrides=%llu",
                  time_s, static_cast<unsigned long long>(summary_.events),
                  static_cast<unsigned long long>(summary_.active),
                  static_cast<unsigned long long>(summary_.peak_active),
                  static_cast<unsigned long long>(summary_.recovered),
                  static_cast<unsigned long long>(summary_.backstop_despawns),
                  static_cast<unsigned long long>(summary_.crossing_overrides));
    out = buf;
    for (int i = 0; i < static_cast<int>(StuckCause::Count); ++i) {
        if (summary_.by_cause[i] == 0) continue;
        std::snprintf(buf, sizeof(buf), " %s=%llu", stuck_cause_name(static_cast<StuckCause>(i)),
                      static_cast<unsigned long long>(summary_.by_cause[i]));
        out += buf;
    }
    for (int i = 1; i < static_cast<int>(StuckChainEnd::Count); ++i) {
        if (summary_.by_chain_end[i] == 0) continue;
        std::snprintf(buf, sizeof(buf), " chain_%s=%llu", stuck_chain_end_name(static_cast<StuckChainEnd>(i)),
                      static_cast<unsigned long long>(summary_.by_chain_end[i]));
        out += buf;
    }
    std::snprintf(buf, sizeof(buf), " roots=%llu root_cycle=%llu root_none=%llu root_cycle_len(2/3/4+)=%llu/%llu/%llu",
                  static_cast<unsigned long long>(summary_.roots),
                  static_cast<unsigned long long>(summary_.roots_by_end[static_cast<int>(StuckChainEnd::Cycle)]),
                  static_cast<unsigned long long>(summary_.roots_by_end[static_cast<int>(StuckChainEnd::None)]),
                  static_cast<unsigned long long>(summary_.root_cycle_len[0]),
                  static_cast<unsigned long long>(summary_.root_cycle_len[1]),
                  static_cast<unsigned long long>(summary_.root_cycle_len[2]));
    out += buf;
    static const char* const kRelation[3] = {"oncoming", "crossing", "same_dir"};
    for (int i = 0; i < 3; ++i) {
        if (summary_.root_cycle_relation[i] == 0) continue;
        std::snprintf(buf, sizeof(buf), " rootrel_%s=%llu", kRelation[i],
                      static_cast<unsigned long long>(summary_.root_cycle_relation[i]));
        out += buf;
    }
    for (int i = 0; i < 3; ++i) {
        if (summary_.by_relation[i] == 0) continue;
        std::snprintf(buf, sizeof(buf), " rel_%s=%llu", kRelation[i], static_cast<unsigned long long>(summary_.by_relation[i]));
        out += buf;
    }
    return out;
}

} // namespace rg
