// test_session_terrain.cpp — rg::Session terrain mode (R2.2 R4) on SYNTHETIC
// terrain only: an in-memory g2m::phys::IHeightTileFetch serving smooth
// sine hills (rounded to 1/256 m integers, like real DGM1 tiles), one NoData
// patch, 404 for every L0 tile entirely outside +-3 km of the session
// origin, and a switchable 503 storm. CI never reads cache/.
//
// Covered: spawn lands on the ground (ride height vs. the same car settled
// in flat mode, <= 0.15 m); a 30 s scripted drive with 0 falls and 0 fill
// misses; state_hash at drive tick 7200 identical at 1 vs 4 workers and at
// fetch delay 0 vs 20 ms (the gate only changes WHEN a tick runs, never WHAT
// it computes); spawn over NoData is a hard error; the coverage edge (404 =
// Absent) never freezes; a 503 storm freezes the real-time loop (failed > 0,
// no step at all) and it resumes without a catch-up burst once the storm is
// over (the same check as test_fixed_rate_loop.cpp's "a freeze period
// causes no catch-up burst", here through the real Session gate).
//
// TOOL-031: no Catch2 assertion runs on the loop thread - every CHECK below
// is on the test thread, reading atomics (streaming_status, loop_stats).
#include "rg/session.h"
#include "rg/terrain_mode.h"

#include "g2m/core/geo/session_frame.h"
#include "g2m/phys/height_tile_loader.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

constexpr g2m::geo::UtmZone k32N{32, g2m::geo::Hemisphere::North};
constexpr std::int64_t kE0 = 474000;
constexpr std::int64_t kN0 = 5560000;
constexpr double kPi = 3.14159265358979323846;
constexpr std::int64_t kCoverageM = 3000; // 404 for L0 tiles entirely outside +-3 km

// Height (1/256 m) of the DGM1 cell whose SW corner is (kE0 + dx, kN0 + dy);
// its sample sits at session-local (dx + 0.5, dy + 0.5) on both paths.
std::int32_t synthetic_height_raw(std::int64_t dx, std::int64_t dy) {
    if (dx >= -400 && dx < -380 && dy >= 100 && dy < 120) return g2m::kHeightNoData; // the NoData patch
    const double x = static_cast<double>(dx);
    const double y = static_cast<double>(dy);
    const double h = 300.0 + 2.0 * std::sin(2.0 * kPi * x / 310.0) * std::cos(2.0 * kPi * y / 270.0) +
                     std::sin(2.0 * kPi * (x + y) / 190.0);
    return static_cast<std::int32_t>(std::llround(h * 256.0));
}

double synthetic_height_m(double x, double y) {
    return static_cast<double>(synthetic_height_raw(static_cast<std::int64_t>(std::floor(x)),
                                                    static_cast<std::int64_t>(std::floor(y)))) /
           256.0;
}

// The synthetic server. Tiles are built once and cached for the process
// (every test reuses them); storm/counters are per instance.
class SyntheticFetch final : public g2m::phys::IHeightTileFetch {
public:
    g2m::phys::FetchResult fetch(const g2m::TileKey& key) override {
        calls.fetch_add(1, std::memory_order_relaxed);
        if (storm.load(std::memory_order_relaxed)) {
            unavailable.fetch_add(1, std::memory_order_relaxed);
            return g2m::phys::FetchResult{g2m::Status::Unavailable, nullptr};
        }
        const std::int64_t dx0 = key.min_easting() - kE0;
        const std::int64_t dy0 = key.min_northing() - kN0;
        if (key.level != 0 || dx0 + 256 <= -kCoverageM || dx0 >= kCoverageM || dy0 + 256 <= -kCoverageM ||
            dy0 >= kCoverageM) {
            not_found.fetch_add(1, std::memory_order_relaxed);
            return g2m::phys::FetchResult{g2m::Status::NotFound, nullptr};
        }
        return g2m::phys::FetchResult{g2m::Status::Ok, tile(key)};
    }

    std::atomic<bool> storm{false};
    std::atomic<std::uint64_t> calls{0};
    std::atomic<std::uint64_t> not_found{0};
    std::atomic<std::uint64_t> unavailable{0};

private:
    static std::shared_ptr<const g2m::HeightTile> tile(const g2m::TileKey& key) {
        static std::mutex mutex;
        static std::map<std::uint64_t, std::shared_ptr<const g2m::HeightTile>> cache;
        {
            std::lock_guard<std::mutex> lock(mutex);
            const auto it = cache.find(key.packed());
            if (it != cache.end()) return it->second;
        }
        auto t = std::make_shared<g2m::HeightTile>(); // heap (TOOL-039)
        t->key = key;
        t->has_nodata = false;
        for (int j = 0; j < g2m::kTerrainTileSamples; ++j) {
            for (int i = 0; i < g2m::kTerrainTileSamples; ++i) {
                const std::int32_t h =
                    synthetic_height_raw(key.min_easting() - kE0 + i, key.min_northing() - kN0 + j);
                t->h[static_cast<std::size_t>(j) * g2m::kTerrainTileSamples + static_cast<std::size_t>(i)] = h;
                if (h == g2m::kHeightNoData) t->has_nodata = true;
            }
        }
        std::lock_guard<std::mutex> lock(mutex);
        return cache.emplace(key.packed(), std::move(t)).first->second;
    }
};

rg::SessionConfig base_config(unsigned workers) {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json";
    config.job_workers = workers;
    return config;
}

rg::SessionConfig terrain_config(unsigned workers, std::shared_ptr<g2m::phys::IHeightTileFetch> fetch, double x,
                                 double y, std::uint32_t fills_per_tick = 1) {
    rg::SessionConfig config = base_config(workers);
    rg::TerrainModeConfig tm(g2m::geo::SessionFrame(k32N, kE0, kN0), std::move(fetch));
    tm.spawn_x = x;
    tm.spawn_y = y;
    tm.spawn_yaw_rad = 0.0;
    tm.physics.max_tile_fills_per_tick = fills_per_tick;
    config.terrain = tm;
    return config;
}

// Ground height straight below the chassis centre (the terrain heightfield:
// the ray starts below the chassis box, which is the only other body).
double ground_below_chassis(rg::Session& s) {
    const ps::Vec3 p = s.world().get_pose(s.chassis_body()).position;
    const ps::RayCastHit hit =
        s.world().backend().ray_cast(ps::Vec3{p.x, p.y, p.z - 0.3}, ps::Vec3{0.0, 0.0, -1.0}, 50.0);
    REQUIRE(hit.hit);
    return hit.point.z;
}

rg::DriveScript hold_still_script() {
    rg::DriveScript script;
    script.at(0, "handbrake", 1.0).at(0, "throttle", 0.0);
    return script;
}

// The 30 s drive: auto clutch/shift, 1st gear, throttle 0.6, straight ahead.
rg::DriveScript drive_script() {
    rg::DriveScript script;
    script.at(0, "ignition", 1.0)
        .at(0, "assist.auto_clutch", 1.0)
        .at(0, "assist.auto_shift", 1.0)
        .at(0, "steer", 0.0)
        .at(48, "shift_up_count", 1.0)
        .at(96, "throttle", 0.6);
    return script;
}

struct DriveRun {
    std::vector<std::uint64_t> hashes; // every 240 drive ticks, the last at 7200
    rg::StreamingStatus status;
    double distance_m = 0.0;
    int seams_crossed = 0;
    double wall_s = 0.0;
    std::uint64_t fetch_calls = 0;
    std::uint64_t held_503 = 0;
    bool hold_lifted = false;
};

// Answers 503 for every L0 tile starting at or east of dx = +1100 m while
// `hold` is set: those keys end up Failed (prefetched at start-up, outside
// the start-up gate, which reaches +1020.5 m), and they enter the gate once
// the car crosses into physics tile 1 (x > 255.5 m) - a freeze that happens
// at the same drive tick in every build type, unlike a fetch delay.
class HoldFarFetch final : public g2m::phys::IHeightTileFetch {
public:
    explicit HoldFarFetch(std::shared_ptr<g2m::phys::IHeightTileFetch> inner) : inner_(std::move(inner)) {}
    g2m::phys::FetchResult fetch(const g2m::TileKey& key) override {
        if (hold.load(std::memory_order_relaxed) && key.min_easting() - kE0 >= 1100) {
            held_503.fetch_add(1, std::memory_order_relaxed);
            return g2m::phys::FetchResult{g2m::Status::Unavailable, nullptr};
        }
        return inner_->fetch(key);
    }
    std::atomic<bool> hold{true};
    std::atomic<std::uint64_t> held_503{0};

private:
    std::shared_ptr<g2m::phys::IHeightTileFetch> inner_;
};

DriveRun run_drive(unsigned workers, int delay_ms, bool hold_far_tiles = false) {
    auto synthetic = std::make_shared<SyntheticFetch>();
    std::shared_ptr<g2m::phys::IHeightTileFetch> fetch = synthetic;
    if (delay_ms > 0) {
        fetch = std::make_shared<g2m::phys::DelayedFetch>(fetch, std::chrono::milliseconds(delay_ms), 7u);
    }
    std::shared_ptr<HoldFarFetch> hold;
    if (hold_far_tiles) {
        hold = std::make_shared<HoldFarFetch>(fetch);
        fetch = hold;
    }
    const auto t0 = std::chrono::steady_clock::now();
    rg::Session session(terrain_config(workers, fetch, 100.0, 50.0));
    session.set_drive_script(drive_script());

    // The "storm is over" operator: once step() is frozen on Failed keys,
    // lift the hold and ask for a retry. Touches only atomics (streaming
    // status, the fetch's flag, retry_failed_tiles) - no World access.
    std::atomic<bool> done{false};
    std::thread watcher;
    if (hold) {
        watcher = std::thread([&] {
            while (!done.load()) {
                const rg::StreamingStatus st = session.streaming_status();
                if (hold->hold.load() && st.frozen && st.failed > 0) {
                    hold->hold.store(false);
                    session.retry_failed_tiles();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    }

    DriveRun run;
    const ps::Vec3 start = session.world().get_pose(session.chassis_body()).position;
    auto phys_tile = [](double v) { return static_cast<long>(std::floor((v - 0.5) / 255.0)); };
    long last_tx = phys_tile(start.x), last_ty = phys_tile(start.y);
    for (int k = 1; k <= 7200; ++k) {
        session.step();
        const ps::Vec3 p = session.world().get_pose(session.chassis_body()).position;
        const long tx = phys_tile(p.x), ty = phys_tile(p.y);
        if (tx != last_tx || ty != last_ty) ++run.seams_crossed;
        last_tx = tx;
        last_ty = ty;
        if (k % 240 == 0) run.hashes.push_back(session.world().state_hash());
    }
    done.store(true);
    if (watcher.joinable()) watcher.join();
    if (hold) {
        run.held_503 = hold->held_503.load();
        run.hold_lifted = !hold->hold.load();
    }
    REQUIRE(session.drive_tick() == 7200);
    const ps::Vec3 end = session.world().get_pose(session.chassis_body()).position;
    run.distance_m = std::hypot(end.x - start.x, end.y - start.y);
    run.status = session.streaming_status();
    run.wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    run.fetch_calls = synthetic->calls.load();
    return run;
}

template <class Pred>
bool wait_for(Pred pred, double timeout_s) {
    const auto t0 = std::chrono::steady_clock::now();
    while (!pred()) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_s) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
}

} // namespace

TEST_CASE("session terrain: the spawn lands on the ground", "[session_terrain]") {
    // Reference ride height: the same car settled on flat mode's ground (z = 0).
    double ride_flat = 0.0;
    {
        rg::Session flat(base_config(1));
        flat.set_drive_script(hold_still_script());
        for (int k = 0; k < 480; ++k) flat.step();
        ride_flat = flat.world().get_pose(flat.chassis_body()).position.z;
    }

    auto fetch = std::make_shared<SyntheticFetch>();
    rg::Session session(terrain_config(1, fetch, 100.0, 50.0));
    REQUIRE(session.terrain_mode());
    const rg::StreamingStatus st0 = session.streaming_status();
    CHECK(st0.ready);
    CHECK(st0.prime_ticks == 26); // (2*ceil(400/255)+1)^2 = 25 tiles at 1 fill/tick, + 1
    CHECK(session.spawn_tick() == 26);
    CHECK(session.ground_body() == ps::BodyId{});
    CHECK(session.terrain_source() != nullptr);

    const ps::Vec3 spawn = session.world().get_pose(session.chassis_body()).position;
    const double ground0 = ground_below_chassis(session);
    session.set_drive_script(hold_still_script());
    for (int k = 0; k < 480; ++k) session.step();
    const ps::Vec3 settled = session.world().get_pose(session.chassis_body()).position;
    const double ground = ground_below_chassis(session);
    const double ride = settled.z - ground;
    const double err = ride - ride_flat;
    const rg::StreamingStatus st = session.streaming_status();
    std::printf("[session_terrain] spawn: ground %.4f m (formula %.4f), spawn clearance above ground %.4f m, "
                "settled ride %.4f vs flat %.4f -> error %+.4f m, xy drift %.4f m, settle drop %.4f m, "
                "startup %.1f ms, prime ticks %u, resident tiles %llu, L0 resident %u\n",
                ground0, synthetic_height_m(spawn.x, spawn.y), spawn.z - ground0, ride, ride_flat, err,
                std::hypot(settled.x - spawn.x, settled.y - spawn.y), spawn.z - settled.z, st.startup_ms,
                st.prime_ticks, static_cast<unsigned long long>(st.resident_tiles), st.resident_l0);
    CHECK(std::abs(ground0 - synthetic_height_m(spawn.x, spawn.y)) < 0.1);
    CHECK(std::abs(err) <= 0.15);
    CHECK(std::abs(spawn.z - settled.z) <= 0.15 + 0.10); // settles within tolerance + spawn_clearance_m
    CHECK(st.falls == 0);
    CHECK(st.fill_misses == 0);
    CHECK(st.starved_tiles == 0);
    CHECK(st.frozen_attempts == 0);
}

TEST_CASE("session terrain: a 30 s drive is deterministic across workers and fetch delay, no falls, no misses",
          "[session_terrain]") {
    const DriveRun a = run_drive(1, 0);
    const DriveRun b = run_drive(4, 0);
    const DriveRun c = run_drive(4, 20);
    // The prefetch ring absorbs 20 ms fetches without a freeze, so d forces
    // one: 503s east of +1100 m until step() is frozen on them (then retried).
    const DriveRun d = run_drive(4, 20, true);

    for (const DriveRun* r : {&a, &b, &c, &d}) {
        std::printf("[session_terrain] drive: hash@7200 0x%016llx, distance %.1f m, seams %d, falls %llu, "
                    "fill misses %llu, nodata fills %llu, starved %llu, frozen attempts %llu, freeze episodes %llu, "
                    "fetches %llu, held 503s %llu, wall %.1f s\n",
                    static_cast<unsigned long long>(r->hashes.back()), r->distance_m, r->seams_crossed,
                    static_cast<unsigned long long>(r->status.falls),
                    static_cast<unsigned long long>(r->status.fill_misses),
                    static_cast<unsigned long long>(r->status.nodata_fills),
                    static_cast<unsigned long long>(r->status.starved_tiles),
                    static_cast<unsigned long long>(r->status.frozen_attempts),
                    static_cast<unsigned long long>(r->status.freeze_count),
                    static_cast<unsigned long long>(r->fetch_calls),
                    static_cast<unsigned long long>(r->held_503), r->wall_s);
        CHECK(r->hashes.size() == 30);
        CHECK(r->status.falls == 0);
        CHECK(r->status.fill_misses == 0);
        CHECK(r->status.starved_tiles == 0);
        CHECK(r->status.relief_overflow == 0);
        CHECK(r->seams_crossed >= 2);
        CHECK(r->distance_m > 300.0);
    }
    CHECK(a.hashes == b.hashes); // 1 vs 4 workers
    CHECK(b.hashes == c.hashes); // fetch delay 0 vs 20 ms
    CHECK(d.held_503 > 0);
    CHECK(d.hold_lifted);
    CHECK(d.status.freeze_count >= 1);
    CHECK(d.status.frozen_attempts > 0);
    CHECK(c.hashes == d.hashes); // ... and with a mid-drive freeze on Failed keys
}

TEST_CASE("session terrain: spawning over NoData is a hard error", "[session_terrain]") {
    auto fetch = std::make_shared<SyntheticFetch>();
    bool threw = false;
    std::string what;
    try {
        rg::Session session(terrain_config(1, fetch, -390.0, 110.0));
    } catch (const std::runtime_error& e) {
        threw = true;
        what = e.what();
    }
    CHECK(threw);
    CHECK(what.find("spawn over NoData") != std::string::npos);
}

TEST_CASE("session terrain: invalid terrain configs are rejected", "[session_terrain]") {
    auto fetch = std::make_shared<SyntheticFetch>();
    rg::SessionConfig config = terrain_config(1, fetch, 0.0, 0.0);
    config.terrain->physics.terrain_surface = "no_such_surface";
    CHECK_THROWS_AS(rg::Session(config), std::invalid_argument);

    config = terrain_config(1, nullptr, 0.0, 0.0);
    CHECK_THROWS_AS(rg::Session(config), std::invalid_argument);
}

TEST_CASE("session terrain: the coverage edge (404 = Absent) never freezes", "[session_terrain]") {
    auto fetch = std::make_shared<SyntheticFetch>();
    // L0 tiles sit on the absolute 256 m UTM grid, not on the frame origin:
    // the last served column ends at dx = +3184 m (the tile starting at
    // +2928 still reaches inside +-3 km). From x = 2900 TileManager's square
    // (to +3315.5 m) reaches past it, so its tiles carry 404 holes.
    rg::Session session(terrain_config(1, fetch, 2900.0, 0.0));
    session.set_drive_script(hold_still_script());
    for (int k = 0; k < 240; ++k) session.step();
    const rg::StreamingStatus st = session.streaming_status();
    std::printf("[session_terrain] coverage edge: 404s served %llu, nodata fills %llu, frozen attempts %llu\n",
                static_cast<unsigned long long>(fetch->not_found.load()),
                static_cast<unsigned long long>(st.nodata_fills),
                static_cast<unsigned long long>(st.frozen_attempts));
    CHECK(fetch->not_found.load() > 0);
    // TileManager's tiles reaching past the last served column carry holes
    // there (Absent is not a NoData fill - the bridge counts only NoData
    // samples of resident tiles), and ground up to it.
    auto ground_at = [&](double x) {
        return session.world()
            .backend()
            .ray_cast(ps::Vec3{x, 0.0, 3000.0}, ps::Vec3{0.0, 0.0, -1.0}, 6000.0)
            .hit;
    };
    CHECK(ground_at(3150.0));
    CHECK_FALSE(ground_at(3250.0));
    CHECK(st.frozen_attempts == 0);
    CHECK(st.failed == 0);
    CHECK(st.fill_misses == 0);
    CHECK(st.falls == 0);
}

TEST_CASE("session terrain: a 503 storm freezes the loop without stepping; it resumes without a burst",
          "[session_terrain]") {
    auto fetch = std::make_shared<SyntheticFetch>();
    // 8 fills/tick: the teleport below moves the TileManager's whole square.
    rg::Session session(terrain_config(1, fetch, 100.0, 50.0, 8));
    const std::uint64_t tick0 = session.world().tick();

    // Storm on, then teleport the car ~1.9 km east - far outside everything
    // already resident, so the gate cannot be met until the storm is over.
    fetch->storm.store(true);
    ps::Pose pose = session.world().get_pose(session.chassis_body());
    pose.position = ps::Vec3{2000.0, 50.0, synthetic_height_m(2000.0, 50.0) + 0.7};
    session.world().backend().set_pose(session.chassis_body(), pose);
    session.world().backend().set_motion(session.chassis_body(), ps::Motion{});

    session.start();
    const bool failed_seen = wait_for([&] { return session.streaming_status().failed > 0; }, 20.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const rg::FixedRateLoop::LoopStats frozen_stats = session.loop_stats();
    const rg::StreamingStatus frozen_st = session.streaming_status();
    const std::uint64_t snapshot_tick_frozen = session.snapshot().tick;

    // Storm over: forget the Failed keys; stepping resumes once the gate is met.
    fetch->storm.store(false);
    session.retry_failed_tiles();
    const bool resumed = wait_for([&] { return session.loop_stats().stepped_count > 0; }, 20.0);
    const std::uint64_t s1 = session.loop_stats().stepped_count;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    const std::uint64_t s2 = session.loop_stats().stepped_count;
    session.stop();

    const rg::FixedRateLoop::LoopStats end_stats = session.loop_stats();
    const rg::StreamingStatus st = session.streaming_status();
    std::printf("[session_terrain] 503 storm: failed %u, 503s served %llu, frozen ticks %llu (%.0f ms), "
                "stepped while frozen %llu; resumed: %llu steps in the 250 ms window (240 Hz -> 60), "
                "freeze episodes %llu, frozen attempts %llu, total stepped %llu, achieved %.1f Hz\n",
                frozen_st.failed, static_cast<unsigned long long>(fetch->unavailable.load()),
                static_cast<unsigned long long>(frozen_stats.freeze_count), frozen_stats.frozen_ms,
                static_cast<unsigned long long>(frozen_stats.stepped_count),
                static_cast<unsigned long long>(s2 - s1), static_cast<unsigned long long>(st.freeze_count),
                static_cast<unsigned long long>(st.frozen_attempts),
                static_cast<unsigned long long>(end_stats.stepped_count), end_stats.achieved_hz);

    REQUIRE(failed_seen);
    CHECK(frozen_stats.stepped_count == 0); // no World::step during the storm
    CHECK(frozen_stats.freeze_count > 0);
    CHECK(frozen_st.frozen);
    CHECK_FALSE(frozen_st.ready);
    CHECK(snapshot_tick_frozen == 0); // nothing published
    REQUIRE(resumed);
    CHECK(s2 - s1 <= 90); // ~60 at 240 Hz; a catch-up burst after seconds frozen would be hundreds
    CHECK(st.freeze_count >= 1);
    CHECK(session.world().tick() == tick0 + end_stats.stepped_count);
    CHECK(st.fill_misses == 0);
    CHECK(st.falls == 0);
}

// --- R2.2 R9: start-up progress and cancellation (rg::StartupProgress) ---

TEST_CASE("session terrain: start-up progress is published while the constructor blocks", "[session_terrain]") {
    auto fetch = std::make_shared<SyntheticFetch>();
    rg::SessionConfig config = terrain_config(1, fetch, 100.0, 50.0);
    auto progress = std::make_shared<rg::StartupProgress>();
    config.startup = progress;

    rg::Session session(config);
    const rg::StreamingStatus st = session.streaming_status();
    std::printf("[session_terrain] progress: stage %d, prime %u/%u, resident L0 %u, missing %u\n",
                progress->stage.load(), progress->prime_done.load(), progress->prime_total.load(),
                progress->resident_l0.load(), progress->missing_required.load());
    CHECK(progress->stage.load() == rg::StartupProgress::Done);
    CHECK(progress->prime_total.load() == st.prime_ticks);
    CHECK(progress->prime_done.load() == st.prime_ticks);
    CHECK(progress->resident_l0.load() > 0);
    CHECK(progress->missing_required.load() == 0);
    CHECK(session.spawn_tick() == 26); // the progress mirror changes nothing it computes
}

TEST_CASE("session terrain: a set cancel flag makes the constructor throw SessionCancelled", "[session_terrain]") {
    auto fetch = std::make_shared<SyntheticFetch>();
    rg::SessionConfig config = terrain_config(1, fetch, 100.0, 50.0);
    config.startup = std::make_shared<rg::StartupProgress>();
    config.startup->cancel.store(true);
    CHECK_THROWS_AS(rg::Session(config), rg::SessionCancelled);
}

TEST_CASE("session terrain: cancelling a start-up blocked by a 503 storm returns promptly", "[session_terrain]") {
    auto fetch = std::make_shared<SyntheticFetch>();
    fetch->storm.store(true); // the gate can never be met
    rg::SessionConfig config = terrain_config(1, fetch, 100.0, 50.0);
    auto progress = std::make_shared<rg::StartupProgress>();
    config.startup = progress;

    std::atomic<int> outcome{0}; // 0 running, 1 cancelled, 2 other exception, 3 constructed
    std::thread worker([&] {
        try {
            rg::Session session(config);
            outcome.store(3);
        } catch (const rg::SessionCancelled&) {
            outcome.store(1);
        } catch (...) {
            outcome.store(2);
        }
    });
    // Progress is live while the constructor blocks: the gate's figures.
    const bool seen = wait_for(
        [&] {
            return progress->stage.load() == rg::StartupProgress::WaitingForGate &&
                   progress->missing_required.load() > 0 && progress->failed.load() > 0;
        },
        20.0);
    const std::uint32_t missing = progress->missing_required.load();
    const std::uint32_t failed = progress->failed.load();
    const auto t0 = std::chrono::steady_clock::now();
    progress->cancel.store(true);
    worker.join();
    const double cancel_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("[session_terrain] cancel: missing %u, failed %u while blocked; constructor returned %.1f ms after "
                "cancel\n",
                missing, failed, cancel_ms);
    CHECK(seen);
    CHECK(outcome.load() == 1);
    CHECK(cancel_ms < 2000.0); // the start-up timeout is 30 s; a cancel returns within a few gate checks
}
