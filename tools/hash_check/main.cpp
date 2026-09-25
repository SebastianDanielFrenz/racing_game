// tools/hash_check — the R0 acceptance check (see repo CLAUDE.md's
// "Hash-comparison acceptance check" section for the full explanation of
// what this compares and why): builds an rg::Session exactly the way the
// real game does (rg::Session::build_world_contents, core/src/session.cpp)
// but configured to numerically mirror
// external/physics_sim/data/scenarios/vehicle_step_steer.json (same ground/
// chassis geometry+mass+friction, same initial chassis velocity, same
// car_sedan.json vehicle, same surfaces.json, same control_events, same
// tick count at the same 240 Hz), steps it synchronously (no SimThread, no
// wall clock - Session::step() drives ps::World::step() directly), and
// prints world.state_hash() in the exact format ps_run itself prints so the
// two can be diffed by eye or by a script:
//
//   ps_run:     scenario 'vehicle_step_steer' -> ... (960 ticks, N workers, state_hash=0x...)
//   hash_check: session  'vehicle_step_steer' -> N/A (960 ticks, N workers, state_hash=0x...)
//
// This is a genuine regression test of rg::Session's own world-construction
// code (core/src/session.cpp's build_world_contents), not a tautological
// reload of the same JSON on both sides - hash_check never parses
// vehicle_step_steer.json itself, it only reads car_sedan.json/
// surfaces.json (the same two files the JSON scenario also references) and
// reconstructs the ground/chassis/control-events by hand from
// SessionConfig, matching the scenario file's own literal numbers (see
// SessionConfig's defaults in core/include/rg/session.h, which already
// equal the scenario's ground/chassis fields - only chassis_initial_velocity
// needs overriding here, since the game itself starts a vehicle at rest).

#include "rg/session.h"

#include <cstdio>
#include <exception>
#include <filesystem>
#include <string>

namespace {

struct ControlEvent {
    double time_s;
    std::string channel;
    double value;
};

} // namespace

int main(int argc, char** argv) {
    // Default: assume this executable runs from racing_game's own build
    // tree (out/build/<preset>/tools/hash_check/) and the submodule sits at
    // the repo's fixed external/physics_sim/data location - a caller
    // driving a different layout (e.g. a CI script) passes the data dir
    // explicitly.
    std::string data_dir = "external/physics_sim/data";
    if (argc > 1) data_dir = argv[1];

    try {
        const std::filesystem::path data_path(data_dir);

        rg::SessionConfig config;
        config.tick_rate_hz = 240.0;
        config.substep_rate_hz = 960.0;
        config.gravity = ps::Vec3{0.0, 0.0, -9.81};
        config.job_workers = 0;
        // Ground/chassis geometry fields all already default to
        // vehicle_step_steer.json's own values (session.h) - only the
        // initial velocity needs overriding: the scenario starts the
        // chassis moving at 20 m/s (its own "why": a step-steer input needs
        // forward speed to produce a meaningful yaw-rate response), while
        // rg::Session's own default is at-rest (the real game's spawn
        // condition).
        config.chassis_initial_velocity = ps::Vec3{20.0, 0.0, 0.0};
        config.vehicle_json_path = (data_path / "vehicles" / "car_sedan.json").string();
        config.surface_table_path = (data_path / "surfaces" / "surfaces.json").string();

        rg::Session session(config);

        // vehicle_step_steer.json's own control_events, applied with the
        // exact same gating loop as tools/ps_run/main.cpp (sim_time() <=
        // event time, in time order) - see that file for the pattern this
        // mirrors.
        const std::vector<ControlEvent> control_events = {
            {0.0, "steer", 0.0},
            {1.0, "steer", 0.25},
        };

        constexpr double duration_s = 4.0;
        const std::uint64_t num_ticks =
            static_cast<std::uint64_t>(duration_s * config.tick_rate_hz + 0.5);

        std::size_t next_event = 0;
        for (std::uint64_t i = 0; i < num_ticks; ++i) {
            while (next_event < control_events.size() &&
                   control_events[next_event].time_s <= session.world().sim_time()) {
                const auto& ev = control_events[next_event];
                session.world().set_control(ev.channel, ev.value);
                ++next_event;
            }
            session.step();
        }

        std::printf("hash_check: session 'vehicle_step_steer' -> N/A (%llu ticks, %d workers, state_hash=0x%016llx)\n",
                    static_cast<unsigned long long>(num_ticks),
                    session.world().job_system().worker_count(),
                    static_cast<unsigned long long>(session.world().state_hash()));
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "hash_check: error: %s\n", e.what());
        return 1;
    }
}
