// tools/traffic_probe - headless NPC traffic harness on the REAL home world (no Godot, no renderer).
//
// Opens rg::WorldTerrain on the world config's store, builds a terrain-mode rg::Session with the player's car
// parked at the spawn, applies a traffic config and steps the simulation as fast as it runs for a given amount of
// simulated time. The Session's own stuck detector (rg/traffic_stuck.h: not arriving and below 0.5 m/s for more
// than 5 s) counts stuck episodes by cause; this tool prints those counts and optionally the event list as CSV.
//
// Determinism: the traffic planner runs on a background thread. After every step the tool waits for a scan that is
// in flight (Session::traffic_scan_in_flight), so the plan is consumed at the same tick on every run, and it warms
// the road geometry cache first (cached-only planning would otherwise see a cold cache). Same flags -> same counts.
//
// Usage: traffic_probe [--seconds S] [--world-config PATH] [--vehicle NAME] [--density D] [--radius R]
//                      [--max N] [--min-spawn M] [--workers N] [--events CSV] [--label TEXT]
// Needs RG_G2M_HOME (or the config's default store). Exit 0; 2 = load failure.
#include "rg/session.h"
#include "rg/terrain_mode.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"
#include "rg/environment.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

namespace {

double flag_number(int argc, char** argv, const char* name, double fallback) {
    for (int i = 1; i + 1 < argc; ++i)
        if (std::strcmp(argv[i], name) == 0) return std::strtod(argv[i + 1], nullptr);
    return fallback;
}
std::string flag_string(int argc, char** argv, const char* name, const std::string& fallback) {
    for (int i = 1; i + 1 < argc; ++i)
        if (std::strcmp(argv[i], name) == 0) return argv[i + 1];
    return fallback;
}

} // namespace

int main(int argc, char** argv) {
    const std::string source = RG_SOURCE_DIR;
    const double seconds = flag_number(argc, argv, "--seconds", 180.0);
    const std::string world_config_path = flag_string(argc, argv, "--world-config", source + "/data/world/world_config.json");
    const std::string vehicle = flag_string(argc, argv, "--vehicle", "car_sedan");
    const std::string events_csv = flag_string(argc, argv, "--events", "");
    const std::string label = flag_string(argc, argv, "--label", "run");
    std::string err;
    auto world_config = rg::load_world_config(world_config_path, &err);
    if (!world_config) { std::fprintf(stderr, "load_world_config: %s\n", err.c_str()); return 2; }
    std::shared_ptr<rg::WorldTerrain> terrain(rg::WorldTerrain::open(*world_config, &err));
    if (!terrain) { std::fprintf(stderr, "WorldTerrain::open: %s\n", err.c_str()); return 2; }

    rg::TrafficConfig traffic;
    traffic.density_per_km = flag_number(argc, argv, "--density", traffic.density_per_km);
    traffic.radius_m = flag_number(argc, argv, "--radius", traffic.radius_m);
    traffic.max_vehicles = static_cast<int>(flag_number(argc, argv, "--max", traffic.max_vehicles));
    traffic.min_spawn_m = flag_number(argc, argv, "--min-spawn", traffic.min_spawn_m);

    // Warm the road geometry of every L2 tile the planner can touch (it plans cached-only at runtime).
    {
        const auto& frame = terrain->frame();
        const double sx = world_config->spawn.e - world_config->session_origin_utm.e0;
        const double sy = world_config->spawn.n - world_config->session_origin_utm.n0;
        const double e = frame.grid_easting(sx), n = frame.grid_northing(sy);
        const double r = traffic.radius_m + 1100.0;
        const auto t0 = std::chrono::steady_clock::now();
        int tiles = 0;
        for (double y = n - r; y <= n + r; y += 1024.0)
            for (double x = e - r; x <= e + r; x += 1024.0) {
                if (terrain->road_geometry_at(x, y)) ++tiles;
            }
        std::printf("traffic_probe: road geometry warmed, %d tiles, %.1f s\n", tiles,
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    }

    rg::SessionConfig config;
    config.job_workers = static_cast<unsigned>(flag_number(argc, argv, "--workers", 2));
    auto environment = rg::load_environment_config(source + "/data/world/environment.json", &err);
    if (environment) config.environment = *environment;
    config.vehicle_json_path = source + "/external/physics_sim/data/vehicles/" + vehicle + ".json";
    config.surface_table_path = source + "/data/surfaces/surfaces.json";
    config.terrain = rg::make_terrain_mode(*world_config, terrain);
    std::string session_err;
    std::unique_ptr<rg::Session> session = rg::make_session(config, &session_err);
    if (!session) { std::fprintf(stderr, "make_session: %s\n", session_err.c_str()); return 2; }
    session->configure_traffic(traffic);

    const double tick_hz = config.tick_rate_hz;
    const auto ticks = static_cast<long long>(seconds * tick_hz);
    const auto wall0 = std::chrono::steady_clock::now();
    std::size_t peak_actors = 0;
    double actor_seconds = 0.0;
    for (long long k = 0; k < ticks; ++k) {
        session->step();
        while (session->traffic_scan_in_flight()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        peak_actors = std::max(peak_actors, session->traffic_actor_count());
        actor_seconds += static_cast<double>(session->traffic_actor_count()) / tick_hz;
        if (k % (static_cast<long long>(tick_hz) * 30) == 0)
            std::printf("t=%.0f actors=%zu stuck_active=%llu events=%llu wall=%.0fs\n", static_cast<double>(k) / tick_hz,
                        session->traffic_actor_count(),
                        static_cast<unsigned long long>(session->traffic_stuck_stats().summary().active),
                        static_cast<unsigned long long>(session->traffic_stuck_stats().summary().events),
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count());
    }
    const auto& stats = session->traffic_stuck_stats();
    std::printf("RESULT %s %s\n", label.c_str(), stats.format_summary(seconds).c_str());
    std::printf("RESULT %s peak_actors=%zu mean_actors=%.1f actor_hours=%.3f events_per_actor_hour=%.1f\n", label.c_str(),
                peak_actors, actor_seconds / seconds, actor_seconds / 3600.0,
                actor_seconds > 0 ? static_cast<double>(stats.summary().events) / (actor_seconds / 3600.0) : 0.0);
    if (!events_csv.empty()) {
        std::ofstream f(events_csv);
        if (f) {
            f << "id,t,truck,cause,other,chain,chain_end,last,x,y,station,route_len,way,route_cap,obstacle_cap,follow_cap,cos,lat,gap\n";
            char line[400];
            for (const auto& e : stats.events()) {
                std::snprintf(line, sizeof(line), "%llu,%.1f,%d,%s,%llu,%d,%s,%llu,%.1f,%.1f,%.1f,%.1f,%lld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                              static_cast<unsigned long long>(e.id), e.time_s, e.truck ? 1 : 0, rg::stuck_cause_name(e.cause),
                              static_cast<unsigned long long>(e.other_id), e.chain.length, rg::stuck_chain_end_name(e.chain.end),
                              static_cast<unsigned long long>(e.chain.last_id), e.x, e.y, e.station_m, e.route_length_m,
                              static_cast<long long>(e.way_id), e.route_cap >= 1e29 ? -1.0 : e.route_cap,
                              e.obstacle_cap >= 1e29 ? -1.0 : e.obstacle_cap, e.follow_cap >= 1e29 ? -1.0 : e.follow_cap,
                              e.other_heading_cos, e.other_lateral_m, e.other_gap_m);
                f << line;
            }
        }
    }
    session.reset();
    return 0;
}
