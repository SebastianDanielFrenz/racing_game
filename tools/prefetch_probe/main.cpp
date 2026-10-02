// Replays a terrain streaming focus along the real home route, without a
// vehicle or renderer. Timings isolate shape preparation/install, not driving.
#include "rg/route_check.h"
#include "rg/terrain_mode.h"
#include "g2m/phys/physics_streamer.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"
#include "ps/world/world.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <thread>

using Clock = std::chrono::steady_clock;
struct Run { std::vector<std::uint64_t> hashes; };

Run replay(const rg::WorldConfig& cfg, const rg::Route& route, bool enabled,
           const std::string& output, unsigned workers) {
    std::string err;
    auto terrain = std::shared_ptr<rg::WorldTerrain>(rg::WorldTerrain::open(cfg, &err));
    if (!terrain) throw std::runtime_error(err);
    auto tm = rg::make_terrain_mode(cfg, terrain);
    g2m::phys::PhysicsTileGrid grid(tm.frame);
    auto resident = std::make_shared<g2m::phys::ResidentHeightSet>();
    g2m::phys::LoaderConfig lc;
    lc.workers = cfg.physics.loader_workers;
    lc.require_roads = cfg.physics.road_surfaces.enabled;
    g2m::phys::HeightTileLoader loader(tm.fetch, lc);
    g2m::phys::PhysicsTerrainStreamer streamer(grid, *resident, loader, {});
    auto table = std::make_shared<ps::io::SurfaceTable>(std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json");
    const auto& names = cfg.physics.road_surfaces;
    const auto resolve = [&](const std::string& name) {
        const auto id = table->id_for(name);
        if (id == ps::kInvalidSurfaceId || id >= 255) throw std::runtime_error("invalid surface " + name);
        return id;
    };
    std::shared_ptr<g2m::ps_bridge::G2mTerrainSource> source;
    if (names.enabled) {
        auto grass = resolve(names.off_road);
        source = std::make_shared<g2m::ps_bridge::G2mTerrainSource>(grid, resident,
            g2m::ps_bridge::RoadSurfaceConfig{g2m::RoadValueLut::by_land_class(grass,
                {{g2m::LandClass::PavedRoad, resolve(names.paved)},
                 {g2m::LandClass::UnpavedRoad, resolve(names.unpaved)}}), grass});
    } else source = std::make_shared<g2m::ps_bridge::G2mTerrainSource>(grid, resident, resolve(cfg.physics.terrain_surface));
    ps::WorldConfig wc;
    wc.job_workers = workers;
    ps::World world(wc); // destroyed before source/resident/loader
    world.set_surface_table(table);
    world.set_terrain_source(source, g2m::ps_bridge::make_terrain_config(cfg.physics.radius_m,
        cfg.physics.max_tile_fills_per_tick, enabled ? 1 : 0, enabled ? 24 : 0));
    g2m::phys::InterestPoint point{0, tm.spawn_x, tm.spawn_y, 0, 0, cfg.physics.radius_m};
    std::uint64_t freezes = 0;
    const auto gate = [&] {
        const auto begin = Clock::now();
        bool waited = false;
        while (!streamer.update(std::span(&point, 1)).ready) {
            waited = true;
            if (Clock::now() - begin > std::chrono::seconds(30)) throw std::runtime_error("gate timeout");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (waited) ++freezes;
    };
    gate();
    world.set_terrain_interest_point(0, {point.x, point.y, 0}, point.radius_m);
    const int side = 2 * static_cast<int>(std::ceil(point.radius_m / 255.0)) + 1;
    for (int i = 0; i < side * side + 1; ++i) { gate(); world.step(); }
    // Let the ring finish before measurement; startup waits are reported separately.
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    const auto initial = world.terrain_prefetch_stats();
    const auto initial_freezes = freezes;
    std::ofstream csv(output);
    if (!csv) throw std::runtime_error("cannot open " + output);
    csv << "tick,x,y,crossing_window,terrain_ms,step_ms,gate_ms,hash,installed,late_sync,late_wait,stale\n";
    std::vector<double> stages, crossings, steps;
    Run result;
    std::size_t segment = 1;
    double segment_start = 0;
    int crossing_count = 0, crossing_until = -1;
    auto old = grid.index_for_local(point.x, point.y);
    auto deadline = Clock::now();
    constexpr double speed = 100;
    constexpr int ticks = 7200;
    for (int i = 0; i < ticks; ++i) {
        const double distance = speed * (i + 1) / 240.0;
        double len = 0;
        while (segment < route.waypoints.size()) {
            const auto a = route.waypoints[segment - 1], b = route.waypoints[segment];
            len = std::hypot(b.x - a.x, b.y - a.y);
            if (distance <= segment_start + len) break;
            segment_start += len;
            ++segment;
        }
        if (segment == route.waypoints.size() || len <= 0) throw std::runtime_error("route shorter than 3 km");
        const auto a = route.waypoints[segment - 1], b = route.waypoints[segment];
        const double f = (distance - segment_start) / len;
        point.x = a.x + f * (b.x - a.x); point.y = a.y + f * (b.y - a.y);
        point.vx = speed * (b.x - a.x) / len; point.vy = speed * (b.y - a.y) / len;
        const auto index = grid.index_for_local(point.x, point.y);
        if (index.ix != old.ix || index.iy != old.iy) { ++crossing_count; crossing_until = i + side + 1; }
        old = index;
        const auto before_gate = Clock::now(); gate();
        const double gate_ms = std::chrono::duration<double, std::milli>(Clock::now() - before_gate).count();
        world.set_terrain_interest_point(0, {point.x, point.y, 0}, point.radius_m);
        const auto before_step = Clock::now(); world.step();
        const double step_ms = std::chrono::duration<double, std::milli>(Clock::now() - before_step).count();
        const double ms = world.last_stage_ms("terrain_tiles");
        stages.push_back(ms); steps.push_back(step_ms);
        if (i <= crossing_until) crossings.push_back(ms);
        const auto hash = world.state_hash(); result.hashes.push_back(hash);
        const auto stats = world.terrain_prefetch_stats();
        csv << i << ',' << point.x << ',' << point.y << ',' << (i <= crossing_until) << ','
            << ms << ',' << step_ms << ',' << gate_ms << ',' << hash << ',' << stats.installed << ','
            << stats.late_sync << ',' << stats.late_wait << ',' << stats.stale_inputs << '\n';
        deadline += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / 240));
        // Do not accelerate through a freeze or a scheduler stall.
        deadline = std::max(deadline, Clock::now());
        std::this_thread::sleep_until(deadline);
    }
    const auto percentile = [](std::vector<double> values, double p) {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        return values[static_cast<std::size_t>(std::ceil(p * values.size())) - 1];
    };
    const auto stats = world.terrain_prefetch_stats();
    std::printf("prefetch=%s workers=%u crossings=%d terrain_p99_ms=%.3f terrain_max_ms=%.3f crossing_p99_ms=%.3f crossing_max_ms=%.3f step_max_ms=%.3f over_budget=%zu installed=%llu late_sync=%llu late_wait=%llu lifetime_wait_max_ms=%.3f stale=%llu freezes=%llu misses=%llu starved=%zu startup_waits=%llu csv=%s\n",
        enabled ? "on" : "off", workers, crossing_count, percentile(stages, .99), percentile(stages, 1),
        percentile(crossings, .99), percentile(crossings, 1), percentile(steps, 1),
        static_cast<std::size_t>(std::count_if(stages.begin(), stages.end(), [](double ms) { return ms > 1000.0 / 240; })),
        static_cast<unsigned long long>(stats.installed - initial.installed),
        static_cast<unsigned long long>(stats.late_sync - initial.late_sync),
        static_cast<unsigned long long>(stats.late_wait - initial.late_wait), stats.late_wait_ns_max / 1e6,
        static_cast<unsigned long long>(stats.stale_inputs - initial.stale_inputs),
        static_cast<unsigned long long>(freezes - initial_freezes),
        static_cast<unsigned long long>(source->fill_miss_count()), world.terrain_starved_tile_count(),
        static_cast<unsigned long long>(initial.late_wait), output.c_str());
    std::fflush(stdout);
    if (source->fill_miss_count() || world.terrain_starved_tile_count() || crossing_count < 2)
        throw std::runtime_error("invalid crossing run");
    return result;
}

int main(int argc, char** argv) {
    try {
        const unsigned workers = argc > 1 ? static_cast<unsigned>(std::stoul(argv[1])) : 4;
        if (workers < 1 || workers > 24) throw std::runtime_error("workers must be 1..24");
        const std::string root = RG_SOURCE_DIR;
        std::string err;
        const auto cfg = rg::load_world_config(root + "/data/world/world_config.json", &err);
        if (!cfg) throw std::runtime_error(err);
        const auto route = rg::load_route(root + "/data/routes/home_r1_drive.json", &err);
        if (!route) throw std::runtime_error(err);
        const auto& origin = cfg->session_origin_utm;
        err = rg::route_matches_world(*route, origin.zone, origin.e0, origin.n0,
            cfg->spawn.e, cfg->spawn.n, cfg->spawn.yaw_deg);
        if (!err.empty()) throw std::runtime_error(err);
        const auto off = replay(*cfg, *route, false, root + "/out/prefetch_off_" + std::to_string(workers) + ".csv", workers);
        const auto on = replay(*cfg, *route, true, root + "/out/prefetch_on_" + std::to_string(workers) + ".csv", workers);
        if (off.hashes != on.hashes) throw std::runtime_error("per-tick hashes differ");
        std::puts("PASS: all 7200 per-tick hashes match");
        return 0;
    } catch (const std::exception& e) { std::fprintf(stderr, "prefetch_probe: %s\n", e.what()); return 1; }
}
