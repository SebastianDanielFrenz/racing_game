// test_deck_install_perf.cpp - S1 measurement (hidden, real data): what moving the road deck BVH build off the sim
// thread buys on a drive over the A66/B8 bridges west of Frankfurt (the seven structure points of the "Inspect reported
// B8 structures" test, ~4.5 km).
//
// The same real rg::Session is driven twice along that polyline, kinematically at RG_DECK_SPEED_MPS (default 45 m/s)
// and paced to real time like the game's FixedRateLoop: once with the pre-S1 reference path (SessionConfig::
// legacy_deck_install: shape built on the sim thread inside create_body(desc), one deck per attempt, only decks in
// range) and once with the DeckInstaller (shapes built on own threads, prefetched 600 m ahead, install is O(1)).
// Prints one DECK_PERF line per mode: sim-thread install time, worst stepped tick, clock-freeze time and count.
// Report only, asserts nothing about timing. SKIPs unless RG_G2M_HOME is set.
#include "rg/drive_script.h"
#include "rg/session.h"
#include "rg/terrain_mode.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include "g2m/core/geo/utm.h"

#include "ps/math/pose.h"
#include "ps/math/quat.h"
#include "ps/math/vec3.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

bool have_env(const char* name) {
    char* buf = nullptr;
    std::size_t len = 0;
#ifdef _WIN32
    const bool ok = _dupenv_s(&buf, &len, name) == 0 && buf != nullptr;
    std::free(buf);
    return ok;
#else
    return std::getenv(name) != nullptr;
#endif
}

double env_double(const char* name, double fallback) {
    char* buf = nullptr;
    std::size_t len = 0;
#ifdef _WIN32
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) return fallback;
    const double v = std::atof(buf);
    std::free(buf);
    return v;
#else
    const char* v = std::getenv(name);
    return v != nullptr ? std::atof(v) : fallback;
#endif
}

struct Pt {
    double x, y;
};

struct PerfResult {
    std::uint64_t ticks = 0, frozen_attempts = 0, freeze_episodes = 0;
    double frozen_wall_ms = 0, worst_freeze_ms = 0, worst_step_ms = 0, p99_step_ms = 0, startup_ms = 0;
    rg::DeckInstallStats decks;
};

PerfResult drive(bool legacy, const std::shared_ptr<rg::WorldTerrain>& terrain, const rg::WorldConfig& world_cfg,
                 const std::vector<Pt>& path, double speed_mps) {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/data/surfaces/surfaces.json";
    config.terrain = rg::make_terrain_mode(world_cfg, terrain);
    config.legacy_deck_install = legacy;
    rg::Session session(config);

    PerfResult out;
    out.startup_ms = session.streaming_status().startup_ms;

    // Cumulative arc length of the polyline.
    std::vector<double> cum{0.0};
    for (std::size_t i = 1; i < path.size(); ++i)
        cum.push_back(cum.back() + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y));
    const double total = cum.back();
    const auto at = [&](double s, double& hx, double& hy) {
        std::size_t i = 1;
        while (i + 1 < path.size() && cum[i] < s) ++i;
        const double seg = std::max(1e-9, cum[i] - cum[i - 1]);
        const double t = std::clamp((s - cum[i - 1]) / seg, 0.0, 1.0);
        hx = (path[i].x - path[i - 1].x) / seg;
        hy = (path[i].y - path[i - 1].y) / seg;
        return Pt{path[i - 1].x + (path[i].x - path[i - 1].x) * t, path[i - 1].y + (path[i].y - path[i - 1].y) * t};
    };

    const double dt = 1.0 / config.tick_rate_hz;
    std::uint64_t driven = 0;
    bool driving = false;
    rg::DriveScript script;
    script.set_controller([&](const rg::DriveTickContext& ctx, ps::World& w) {
        if (!driving) return;
        const double s = std::min(total, speed_mps * dt * static_cast<double>(driven));
        double hx = 1, hy = 0;
        const Pt p = at(s, hx, hy);
        const ps::Pose cur = w.get_pose(ctx.chassis);
        const ps::Motion mot = w.get_motion(ctx.chassis);
        ps::Pose pose = cur;
        pose.position = ps::Vec3{p.x, p.y, cur.position.z};
        pose.orientation = ps::Quat::from_axis_angle(ps::Vec3{0.0, 0.0, 1.0}, std::atan2(hy, hx));
        w.backend().set_pose(ctx.chassis, pose);
        ps::Motion m;
        m.linear = ps::Vec3{hx * speed_mps, hy * speed_mps, mot.linear.z};
        w.backend().set_motion(ctx.chassis, m);
    });
    session.set_drive_script(std::move(script));

    // Relocate to the first point (the relocation gate installs the decks around it), then settle a little.
    double hx = 1, hy = 0;
    const Pt first = at(0.0, hx, hy);
    session.request_relocate(first.x, first.y, std::atan2(hy, hx));
    for (int i = 0; i < 480; ++i) session.step();

    // Measured part starts here (the relocation's own cost is reported separately by the stats below).
    const rg::DeckInstallStats after_reloc = session.deck_install_stats();
    driving = true;
    driven = 0;
    const std::uint64_t ticks = static_cast<std::uint64_t>(total / (speed_mps * dt));
    std::vector<double> step_ms;
    step_ms.reserve(ticks);
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(dt));
    auto next = Clock::now();
    bool in_freeze = false;
    Clock::time_point freeze_start;
    const std::uint64_t freezes_before = session.streaming_status().freeze_count;
    while (driven < ticks) {
        while (Clock::now() < next) std::this_thread::yield();
        const auto t0 = Clock::now();
        const bool stepped = session.try_step();
        const auto t1 = Clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (!stepped) {
            ++out.frozen_attempts;
            if (!in_freeze) {
                in_freeze = true;
                freeze_start = t0;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            next = Clock::now();
            continue;
        }
        if (in_freeze) {
            in_freeze = false;
            const double f = std::chrono::duration<double, std::milli>(t1 - freeze_start).count();
            out.frozen_wall_ms += f;
            out.worst_freeze_ms = std::max(out.worst_freeze_ms, f);
        }
        ++driven;
        step_ms.push_back(ms);
        next += period;
        if (t1 > next) next = t1;
    }
    out.ticks = driven;
    out.freeze_episodes = session.streaming_status().freeze_count - freezes_before;
    std::vector<double> sorted = step_ms;
    std::sort(sorted.begin(), sorted.end());
    out.worst_step_ms = sorted.back();
    out.p99_step_ms = sorted[static_cast<std::size_t>(0.99 * static_cast<double>(sorted.size() - 1))];
    out.decks = session.deck_install_stats();
    out.decks.install_ns_total -= after_reloc.install_ns_total;
    out.decks.installed -= after_reloc.installed;
    return out;
}

} // namespace

TEST_CASE("deck install perf: A66/B8 bridge drive, sim-thread build vs prefetched shapes", "[.][realdata][perf][s1]") {
    if (!have_env("RG_G2M_HOME")) SKIP("RG_G2M_HOME not set");
    std::string err;
    const auto world_cfg = rg::load_world_config(std::string(RG_SOURCE_DIR) + "/data/world/world_config.json", &err);
    INFO(err);
    REQUIRE(world_cfg.has_value());
    std::shared_ptr<rg::WorldTerrain> terrain(rg::WorldTerrain::open(*world_cfg, &err));
    INFO(err);
    REQUIRE(terrain != nullptr);

    const double wgs[][2] = {{50.121806, 8.524495}, {50.123512, 8.515598}, {50.126719, 8.502332}, {50.130483, 8.491895},
                             {50.135940, 8.486279}, {50.141900, 8.479882}, {50.149487, 8.466567}};
    g2m::geo::Utm utm;
    const rg::TerrainModeConfig probe = rg::make_terrain_mode(*world_cfg, terrain);
    const double e0 = probe.frame.e0_m(), n0 = probe.frame.n0_m();
    std::vector<Pt> path;
    for (const auto& w : wgs) {
        const auto p = utm.forward({32}, w[0], w[1]);
        path.push_back({p.easting - e0, p.northing - n0});
    }
    const double speed = env_double("RG_DECK_SPEED_MPS", 45.0);

    for (const bool legacy : {true, false}) {
        const PerfResult r = drive(legacy, terrain, *world_cfg, path, speed);
        const double n = std::max<std::uint64_t>(1, r.decks.installed);
        std::printf("DECK_PERF mode=%s speed=%.0f ticks=%llu startup_ms=%.0f | decks installed=%llu "
                    "sim_install_ms_total=%.2f mean=%.3f max=%.3f | worker_build_ms_total=%.1f max=%.1f waits=%llu | "
                    "freeze episodes=%llu attempts=%llu wall_ms=%.1f worst_ms=%.1f | step p99=%.3f max=%.3f ms\n",
                    legacy ? "legacy" : "prefetched", speed, static_cast<unsigned long long>(r.ticks), r.startup_ms,
                    static_cast<unsigned long long>(r.decks.installed), static_cast<double>(r.decks.install_ns_total) / 1e6,
                    static_cast<double>(r.decks.install_ns_total) / 1e6 / n, static_cast<double>(r.decks.install_ns_max) / 1e6,
                    static_cast<double>(r.decks.build_ns_total) / 1e6, static_cast<double>(r.decks.build_ns_max) / 1e6,
                    static_cast<unsigned long long>(r.decks.waits), static_cast<unsigned long long>(r.freeze_episodes),
                    static_cast<unsigned long long>(r.frozen_attempts), r.frozen_wall_ms, r.worst_freeze_ms, r.p99_step_ms,
                    r.worst_step_ms);
    }
    SUCCEED();
}
