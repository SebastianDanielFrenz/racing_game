// test_tick_spikes.cpp - diagnosis harness for periodic sim-tick stalls while
// driving fast on real terrain (owner drive 2026-09-27: ~170 ms lost every
// ~250 m at 150-185 km/h). A real rg::Session on the committed home route is
// pushed KINEMATICALLY along data/routes/home_r1_drive.json at a fixed speed
// (the drive-script controller sets the chassis pose/velocity every tick), so
// terrain streaming, the gate and World::step run exactly as in the game,
// paced to real time like FixedRateLoop. Writes two CSVs next to the build:
// tick_spikes_ticks.csv (per tick: s, wall ms of Session::step) and
// tick_spikes_world.csv (ps::World's own telemetry, incl. stage.<name>.ms).
//
// Real data only: hidden ([.]) and SKIPs unless RG_G2M_HOME is set. Report
// only - it asserts nothing about timing.
#include "rg/drive_script.h"
#include "rg/route_check.h"
#include "rg/session.h"
#include "rg/terrain_mode.h"
#include "rg/terrain_view_streamer.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include "ps/math/pose.h"
#include "ps/math/quat.h"
#include "ps/math/vec3.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

std::optional<std::string> safe_getenv(const char* name) {
#ifdef _WIN32
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) return std::nullopt;
    std::string value(buf);
    std::free(buf);
    return value;
#else
    const char* value = std::getenv(name);
    return value != nullptr ? std::optional<std::string>(value) : std::nullopt;
#endif
}

double env_double(const char* name, double fallback) {
    const std::optional<std::string> v = safe_getenv(name);
    return v.has_value() ? std::atof(v->c_str()) : fallback;
}

// Session-local point and unit heading at arc length s.
struct RouteSample {
    double x = 0.0, y = 0.0, hx = 1.0, hy = 0.0;
};

std::optional<RouteSample> sample_at_s(const rg::Route& route, double target_s, double e0, double n0) {
    double along = 0.0;
    for (std::size_t i = 1; i < route.waypoints.size(); ++i) {
        const double ax = route.waypoints[i - 1].x, ay = route.waypoints[i - 1].y;
        const double bx = route.waypoints[i].x, by = route.waypoints[i].y;
        const double seg = std::sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
        if (seg > 1e-6 && along + seg >= target_s) {
            const double t = std::clamp((target_s - along) / seg, 0.0, 1.0);
            RouteSample r;
            r.x = route.e0 + ax + (bx - ax) * t - e0;
            r.y = route.n0 + ay + (by - ay) * t - n0;
            r.hx = (bx - ax) / seg;
            r.hy = (by - ay) / seg;
            return r;
        }
        along += seg;
    }
    return std::nullopt;
}

} // namespace

TEST_CASE("tick spikes: kinematic fast drive along home_r1_drive through the real Session", "[.][realdata][perf]") {
    if (!safe_getenv("RG_G2M_HOME").has_value()) SKIP("RG_G2M_HOME not set");
    const double speed_mps = env_double("RG_SPIKE_SPEED_MPS", 48.0);
    const double s_begin = env_double("RG_SPIKE_S0", 0.0);
    const double s_end = env_double("RG_SPIKE_S1", 6000.0);
    const bool with_render = env_double("RG_SPIKE_RENDER", 0.0) != 0.0;
    const unsigned render_threads = static_cast<unsigned>(env_double("RG_SPIKE_RENDER_THREADS", 2.0));

    std::string err;
    const auto world_cfg = rg::load_world_config(std::string(RG_SOURCE_DIR) + "/data/world/world_config.json", &err);
    INFO(err);
    REQUIRE(world_cfg.has_value());
    const auto route = rg::load_route(std::string(RG_SOURCE_DIR) + "/data/routes/home_r1_drive.json", &err);
    INFO(err);
    REQUIRE(route.has_value());

    std::shared_ptr<rg::WorldTerrain> terrain(rg::WorldTerrain::open(*world_cfg, &err));
    INFO(err);
    REQUIRE(terrain != nullptr);
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json";
    config.terrain = rg::make_terrain_mode(*world_cfg, terrain);
    const double e0 = config.terrain->frame.e0_m();
    const double n0 = config.terrain->frame.n0_m();
    rg::Session session(config);

    const double dt = 1.0 / config.tick_rate_hz;
    std::uint64_t driven = 0;
    double current_s = s_begin;
    rg::DriveScript script;
    script.set_controller([&](const rg::DriveTickContext& ctx, ps::World& w) {
        current_s = s_begin + speed_mps * dt * static_cast<double>(driven);
        const std::optional<RouteSample> r = sample_at_s(*route, current_s, e0, n0);
        if (!r) return;
        const ps::Pose cur = w.get_pose(ctx.chassis);
        const ps::Motion mot = w.get_motion(ctx.chassis);
        ps::Pose p = cur;
        p.position = ps::Vec3{r->x, r->y, cur.position.z};
        p.orientation = ps::Quat::from_axis_angle(ps::Vec3{0.0, 0.0, 1.0}, std::atan2(r->hy, r->hx));
        w.backend().set_pose(ctx.chassis, p);
        ps::Motion m;
        m.linear = ps::Vec3{r->hx * speed_mps, r->hy * speed_mps, mot.linear.z};
        w.backend().set_motion(ctx.chassis, m);
    });
    session.set_drive_script(std::move(script));

    // Variant B: the render TerrainViewStreamer on the SAME WorldTerrain the
    // Session uses (the game shares it), fed the chassis position from a
    // ~60 Hz "main thread" that polls and commits each diff at once.
    std::atomic<double> car_x{0.0}, car_y{0.0};
    std::atomic<bool> render_stop{false};
    std::unique_ptr<rg::TerrainViewStreamer> view;
    std::thread render_thread;
    if (with_render) {
        rg::TerrainViewStreamer::Options opt;
        opt.build_threads = render_threads;
        view = std::make_unique<rg::TerrainViewStreamer>(terrain->view_source(), opt);
        const ps::Vec3 p0 = session.world().get_pose(session.chassis_body()).position;
        car_x = p0.x;
        car_y = p0.y;
        std::vector<rg::RenderChunk> initial;
        view->build_initial(p0.x, p0.y, initial);
        render_thread = std::thread([&] {
            rg::TerrainViewDiff diff;
            while (!render_stop.load()) {
                view->update_focus(car_x.load(), car_y.load());
                if (view->poll(diff)) view->commit(diff.serial);
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
            }
        });
    }

    const std::string world_csv = "tick_spikes_world.csv";
    session.world().telemetry().start_csv(world_csv);

    const std::uint64_t ticks = static_cast<std::uint64_t>((s_end - s_begin) / (speed_mps * dt));
    std::vector<double> wall_ms;
    std::vector<double> s_at;
    wall_ms.reserve(ticks);
    s_at.reserve(ticks);
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(dt));
    auto next = clock::now();
    std::uint64_t late_ticks = 0;
    for (std::uint64_t i = 0; i < ticks; ++i) {
        while (clock::now() < next) std::this_thread::yield();
        const auto t0 = clock::now();
        session.step();
        const auto t1 = clock::now();
        ++driven;
        if (with_render) {
            const ps::Vec3 pc = session.world().get_pose(session.chassis_body()).position;
            car_x = pc.x;
            car_y = pc.y;
        }
        wall_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        s_at.push_back(current_s);
        next += period;
        if (t1 > next) { // behind: resync like FixedRateLoop instead of bursting
            late_ticks += static_cast<std::uint64_t>((t1 - next) / period) + 1;
            next = t1;
        }
    }
    session.world().telemetry().stop();
    if (with_render) {
        render_stop = true;
        render_thread.join();
        const rg::TerrainViewStreamer::Stats vs = view->stats();
        std::printf("tick_spikes: render build_threads=%u selections=%llu chunks_built=%llu last_build_ms=%.1f\n",
                    render_threads, static_cast<unsigned long long>(vs.selections_started),
                    static_cast<unsigned long long>(vs.chunks_built), vs.last_build_ms);
    }

    {
        std::ofstream f("tick_spikes_ticks.csv");
        f << "i,s,wall_ms\n";
        for (std::size_t i = 0; i < wall_ms.size(); ++i) f << i << ',' << s_at[i] << ',' << wall_ms[i] << '\n';
    }
    std::vector<double> sorted = wall_ms;
    std::sort(sorted.begin(), sorted.end());
    auto pct = [&](double p) { return sorted[static_cast<std::size_t>(p * static_cast<double>(sorted.size() - 1))]; };
    std::size_t over8 = 0;
    for (double v : wall_ms) over8 += v > 8.0 ? 1 : 0;
    const rg::StreamingStatus st = session.streaming_status();
    std::printf("tick_spikes: speed=%.1f m/s s=[%.0f,%.0f] ticks=%zu p50=%.3f p99=%.3f max=%.3f ms over8ms=%zu "
                "late_ticks=%llu freezes=%llu falls=%llu\n",
                speed_mps, s_begin, s_end, wall_ms.size(), pct(0.5), pct(0.99), sorted.back(), over8,
                static_cast<unsigned long long>(late_ticks), static_cast<unsigned long long>(st.freeze_count),
                static_cast<unsigned long long>(st.falls));
    SUCCEED();
}
