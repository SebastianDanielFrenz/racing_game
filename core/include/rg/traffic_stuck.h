#pragma once
// NPC traffic stuck detector (engine-neutral, no Godot/Jolt types).
//
// An actor is STUCK when it is not arriving and its speed stays below
// kStuckSpeedMps for more than kStuckTimeS. The Session feeds one StuckTrack per
// actor each tick (advance_stuck_track, a few flops), and when a track crosses
// the threshold classifies WHAT the controller is obeying (StuckCause), walks the
// "who am I waiting for" chain (walk_stuck_chain: a chain, a cycle, a stuck root)
// and records the event in TrafficStuckStats (counters, a bounded event list and
// the RG_TRAFFIC_STUCK log text). Cheap counters, kept in the build on purpose:
// docs/npc_traffic.md "Stuck traffic (2026-10-05)".
#include <cstdint>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace rg {

inline constexpr double kStuckSpeedMps = 0.5;
inline constexpr double kStuckTimeS = 5.0;
// Actors this close to the end of their route are arriving, never stuck.
inline constexpr double kStuckArrivingM = 5.0;
// An actor that has been below kStuckSpeedMps for this long stops yielding to crossing NPCs (see Session::update_traffic):
// the id tie-break alone cannot break a wait cycle that mixes same-lane following with crossing yields.
inline constexpr double kCrossingPatienceS = 8.0;

// What the controller's binding (smallest) speed cap came from.
enum class StuckCause : int {
    FollowPlayer = 0, // the car-following rule, target = the player's car
    FollowNpc,        // ... another NPC (see the chain)
    FollowTruck,      // ... the T truck
    ProbePlayer,      // the forward obstacle probe hit the player's chassis
    ProbeNpc,         // ... another NPC's body (see the chain)
    ProbeTruck,       // ... the T truck
    ProbeTerrain,     // ... a terrain heightfield tile (ground ahead on a crest/ramp)
    ProbeDeck,        // ... a road-deck mesh (bridge / underpass)
    ProbeStatic,      // ... any other static body (feature meshes)
    ProbeOther,       // ... anything else (walker, unknown body)
    RouteCap,         // the legal/corner/end cap itself is ~0 (route geometry)
    Other,            // nothing below the stuck speed binds (acceleration limit, bug)
    Count
};

// How a chain of "waiting for another NPC" ends.
enum class StuckChainEnd : int {
    None = 0,  // not an NPC-waiting cause
    Moving,    // the NPC at the end of the chain is moving again (a slow queue)
    Player,
    Truck,
    Terrain,
    Deck,
    Static,
    RouteCap,  // the chain's root is stopped by its own route cap
    Cycle,     // A waits for B waits for ... A: a deadlock
    Lost,      // the NPC at the end of the chain no longer exists
    Other,
    Count
};

[[nodiscard]] const char* stuck_cause_name(StuckCause c);
[[nodiscard]] const char* stuck_chain_end_name(StuckChainEnd e);
[[nodiscard]] bool stuck_cause_waits_for_npc(StuckCause c);

// Per-actor tracker. below_s accumulates while the actor is slow and not
// arriving; one crossing of kStuckTimeS is reported once per episode.
struct StuckTrack {
    double below_s = 0.0;
    bool declared = false;
};
enum class StuckTrackEvent : int { None = 0, Declared, Recovered };
// Returns Declared exactly once when below_s first exceeds kStuckTimeS and
// Recovered once when a declared actor is moving again (speed >= 2 * the stuck
// speed, so a creeping jitter does not flap).
StuckTrackEvent advance_stuck_track(StuckTrack& track, double speed_mps, bool arriving, double dt_s);

// The facts a chain walk needs about one actor.
struct StuckNode {
    double speed_mps = 0.0;
    StuckCause cause = StuckCause::Other;
    std::uint64_t other_id = 0; // the NPC waited for (FollowNpc / ProbeNpc)
    double heading_cos = 1.0;   // cos of the heading difference to that NPC
    bool has_heading = false;
};
struct StuckChain {
    int length = 1;              // actors in the chain including the first
    StuckChainEnd end = StuckChainEnd::None;
    std::uint64_t last_id = 0;   // the last actor looked at
    // The root of the chain: the smallest id among the members of a Cycle, else the last actor. Distinct roots are the
    // real number of deadlocks/blockers; the other events are the queues behind them.
    std::uint64_t root_id = 0;
    int cycle_len = 0;
    double root_heading_cos = 1.0; // of the root's own wait (a cycle's root waits for another cycle member)
    bool root_has_heading = false;
};
using StuckLookup = std::function<std::optional<StuckNode>(std::uint64_t)>;
// Walks from `id` through every "waits for another NPC" link (bounded to 64).
[[nodiscard]] StuckChain walk_stuck_chain(std::uint64_t id, const StuckLookup& lookup);

// One declared stuck episode.
struct StuckEvent {
    std::uint64_t id = 0;
    double time_s = 0.0;
    StuckCause cause = StuckCause::Other;
    std::uint64_t other_id = 0;
    StuckChain chain;
    double x = 0.0, y = 0.0;
    double station_m = 0.0, route_length_m = 0.0;
    std::int64_t way_id = 0;
    double route_cap = 0.0, obstacle_cap = 0.0, follow_cap = 0.0; // m/s (1e30 = none)
    bool truck = false;
    // Geometry of the waited-for NPC (FollowNpc / ProbeNpc only): cos of the heading difference (1 = same direction,
    // -1 = oncoming), lateral offset from this actor's forward axis and the distance along it, metres.
    bool has_other = false;
    double other_heading_cos = 0.0, other_lateral_m = 0.0, other_gap_m = 0.0;
};

// Totals for the whole session (reset by clear()).
struct StuckSummary {
    std::uint64_t events = 0;                 // declared episodes
    std::uint64_t by_cause[static_cast<int>(StuckCause::Count)] = {};
    std::uint64_t by_chain_end[static_cast<int>(StuckChainEnd::Count)] = {};
    // NPC-waiting events by the geometry of the waited-for NPC: [0] oncoming (heading cos < -0.5), [1] crossing, [2] same direction.
    std::uint64_t by_relation[3] = {};
    // DISTINCT chain roots (StuckChain::root_id): the real number of blockers/deadlocks, the rest of `events` are the
    // actors queued behind them. roots_by_end is indexed like by_chain_end (None = stopped by itself, Cycle = deadlock).
    std::uint64_t roots = 0;
    std::uint64_t roots_by_end[static_cast<int>(StuckChainEnd::Count)] = {};
    std::uint64_t root_cycle_len[3] = {};      // deadlock cycles of length 2, 3, 4+
    std::uint64_t root_cycle_relation[3] = {}; // the root's own wait in a cycle: oncoming / crossing / same direction
    std::uint64_t recovered = 0;             // declared actors that moved again on their own
    std::uint64_t backstop_despawns = 0;      // last-resort unsticks (out of view), counted apart
    std::uint64_t crossing_overrides = 0;     // follow decisions that ignored a crossing yield after kCrossingPatienceS
    std::uint64_t active = 0;                 // stuck right now
    std::uint64_t peak_active = 0;
    double longest_recovered_s = 0.0;
};

class TrafficStuckStats {
public:
    void clear();
    void record(const StuckEvent& e);
    void note_recovered(double stuck_s);
    void note_backstop();
    void note_crossing_override() { ++summary_.crossing_overrides; }
    void set_active(std::uint64_t n);
    [[nodiscard]] const StuckSummary& summary() const { return summary_; }
    [[nodiscard]] const std::vector<StuckEvent>& events() const { return events_; } // bounded (kMaxEvents)
    static constexpr std::size_t kMaxEvents = 4096;
    // "RG_TRAFFIC_STUCK id=.. t=.." one line, no newline.
    [[nodiscard]] static std::string format_event(const StuckEvent& e);
    // "RG_TRAFFIC_STUCK_SUMMARY t=.. events=.. active=.. cause_a=n ..." one line, no newline.
    [[nodiscard]] std::string format_summary(double time_s) const;
private:
    StuckSummary summary_;
    std::vector<StuckEvent> events_;
    std::set<std::uint64_t> roots_seen_;
};

} // namespace rg
