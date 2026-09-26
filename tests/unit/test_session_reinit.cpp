// test_session_reinit.cpp — R2.2 R7 task 4: build/destroy/rebuild cycles
// across rg::Session's two modes (flat, terrain) in a single process,
// proving no state leaks from one Session's construction/destruction into
// the next - the rg_core-side backing for RgSimulation's own re-entrant
// initialize()/initialize_terrain() (godot_ext/src/rg_simulation.h/.cpp).
// No Godot involved (mirrors test_session_terrain.cpp's own "no Godot"
// scope). Reuses that file's own SyntheticFetch/base_config/terrain_config
// helper PATTERN (duplicated here, simplified to a flat/constant-height
// fetch - this test only needs a Session that spawns and steps cleanly, not
// the sine-hill terrain/NoData/coverage-edge machinery test_session_terrain.cpp
// exercises; no tests/support/ header exists yet in this repo for a shared
// helper, see that file's own header comment) and tools/hash_check/main.cpp's
// own flat-mode determinism-check scenario (chassis_initial_velocity
// {20,0,0}, two "steer" control events, 960 ticks @ 240 Hz) as the "flat
// hash" this test compares before vs. after a terrain Session build/destroy
// cycle runs in between.
//
// "Equals the flat hash from a fresh process run": verified as a same-process
// before/after comparison (the reference hash is computed before this test
// ever touches a terrain Session, then recomputed after two terrain Sessions
// and a flat one have been built and destroyed) rather than by shelling out
// to tools/hash_check as a second OS process - see this file's own hand-back
// note for why (path/preset resolution across debug/release builds would
// make a subprocess check more fragile than what this already proves: no
// global/static state a terrain Session touches changes a LATER flat
// Session's own determinism).
#include "rg/session.h"
#include "rg/terrain_mode.h"

#include "g2m/core/geo/session_frame.h"
#include "g2m/layer/height_tile.h"
#include "g2m/phys/height_tile_loader.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr g2m::geo::UtmZone k32N{32, g2m::geo::Hemisphere::North};
constexpr std::int64_t kE0 = 474000;
constexpr std::int64_t kN0 = 5560000;

// A flat, constant-height (300 m), never-NoData, always-Ok synthetic fetch -
// this test only needs terrain mode to spawn/step without error, not any of
// test_session_terrain.cpp's sine-hill/NoData/coverage-edge/503-storm
// scenarios.
class FlatSyntheticFetch final : public g2m::phys::IHeightTileFetch {
public:
    g2m::phys::FetchResult fetch(const g2m::TileKey& key) override {
        if (key.level != 0) return g2m::phys::FetchResult{g2m::Status::NotFound, nullptr};
        auto t = std::make_shared<g2m::HeightTile>(); // heap (TOOL-039: 256 KiB, never on the stack)
        t->key = key;
        t->has_nodata = false;
        const std::int32_t h = static_cast<std::int32_t>(300.0 * 256.0); // flat 300 m everywhere
        for (int j = 0; j < g2m::kTerrainTileSamples; ++j) {
            for (int i = 0; i < g2m::kTerrainTileSamples; ++i) {
                t->h[static_cast<std::size_t>(j) * g2m::kTerrainTileSamples + static_cast<std::size_t>(i)] = h;
            }
        }
        return g2m::phys::FetchResult{g2m::Status::Ok, std::move(t)};
    }
};

rg::SessionConfig base_config(unsigned workers) {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json";
    config.job_workers = workers;
    return config;
}

rg::SessionConfig terrain_config(unsigned workers) {
    rg::SessionConfig config = base_config(workers);
    rg::TerrainModeConfig tm(g2m::geo::SessionFrame(k32N, kE0, kN0), std::make_shared<FlatSyntheticFetch>());
    tm.spawn_x = 100.0;
    tm.spawn_y = 50.0;
    tm.spawn_yaw_rad = 0.0;
    config.terrain = tm;
    return config;
}

struct ControlEvent {
    double time_s;
    const char* channel;
    double value;
};

// tools/hash_check/main.cpp's own scenario and gating loop, verbatim
// (chassis moving at 20 m/s, a step-steer input, 960 ticks @ 240 Hz) - see
// that file's header comment for why these particular numbers.
std::uint64_t flat_reference_hash(unsigned workers) {
    rg::SessionConfig config = base_config(workers);
    config.chassis_initial_velocity = ps::Vec3{20.0, 0.0, 0.0};
    rg::Session session(config);

    const std::vector<ControlEvent> control_events = {
        {0.0, "steer", 0.0},
        {1.0, "steer", 0.25},
    };
    constexpr double duration_s = 4.0;
    const std::uint64_t num_ticks = static_cast<std::uint64_t>(duration_s * config.tick_rate_hz + 0.5);

    std::size_t next_event = 0;
    for (std::uint64_t i = 0; i < num_ticks; ++i) {
        while (next_event < control_events.size() && control_events[next_event].time_s <= session.world().sim_time()) {
            session.world().set_control(control_events[next_event].channel, control_events[next_event].value);
            ++next_event;
        }
        session.step();
    }
    return session.world().state_hash();
}

} // namespace

TEST_CASE("session reinit: terrain -> destroy -> flat -> terrain again, in one process, leaves no leaked state",
          "[session_terrain][session_reinit]") {
    // Reference hash, computed BEFORE this test ever touches a terrain
    // Session - stands in for "a fresh process run" (see this file's own
    // header comment for why a same-process before/after comparison is used
    // instead of a subprocess).
    const std::uint64_t hash_before = flat_reference_hash(1);

    // 1. Build a terrain Session on a synthetic fetch, step it a little, destroy it.
    {
        rg::Session terrain_session(terrain_config(1));
        REQUIRE(terrain_session.terrain_mode());
        CHECK(terrain_session.streaming_status().ready);
        for (int k = 0; k < 60; ++k) terrain_session.step();
        CHECK(terrain_session.drive_tick() == 60);
        // terrain_session destructs here (Session::~Session -> stop(), then
        // Terrain's own streamer/loader/resident set tear down).
    }

    // 2. Build a flat Session, step it, destroy it.
    {
        rg::Session flat_session(base_config(1));
        CHECK_FALSE(flat_session.terrain_mode());
        for (int k = 0; k < 60; ++k) flat_session.step();
        CHECK(flat_session.drive_tick() == 60);
    }

    // 3. Build a terrain Session again (a second, independent instance in
    // the same process), step it, destroy it.
    {
        rg::Session terrain_session2(terrain_config(1));
        REQUIRE(terrain_session2.terrain_mode());
        for (int k = 0; k < 60; ++k) terrain_session2.step();
        CHECK(terrain_session2.drive_tick() == 60);
    }

    // No leaks/hangs: reaching here at all (Catch2's own process exit) is
    // part of what this test asserts - a leaked thread (a HeightTileLoader
    // worker not joined, a FixedRateLoop left running) would hang the test
    // binary's shutdown; a leaked global/static side effect would still let
    // the process exit, which is why the hash comparison below is the other
    // half of this test's acceptance.
    const std::uint64_t hash_after = flat_reference_hash(1);
    CHECK(hash_before == hash_after);
}
