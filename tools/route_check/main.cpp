// tools/route_check — checks an "rg.route/1" drive route against the real
// terrain store (R2.2 plan section 3, "tools/terrain_drive" route criteria):
// loads the world config and the route, opens rg::WorldTerrain on the
// config's stores, samples L0 heights along the route and prints length,
// grade (max/p99 over a 10 m window), physics-tile seam crossings, elevation
// range, NoData and the tightest corner. Thin main over
// rg::check_route_on_world (core/include/rg/route_check.h) - R5's
// [realdata] drive test runs the same check.
//
// Usage: route_check [--world-config PATH] [--route PATH]
//   defaults: <repo>/data/world/world_config.json,
//             <repo>/data/routes/home_r1_drive.json
// Exit: 0 = every criterion passes, 1 = a criterion failed,
//       2 = usage error / config, route or store failed to load.
// Godot-free; reads the store only through WorldTerrain (the config's
// source store is opened read-only).
#include "rg/route_check.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

int main(int argc, char** argv) {
    std::string world_config_path = std::string(RG_SOURCE_DIR) + "/data/world/world_config.json";
    std::string route_path = std::string(RG_SOURCE_DIR) + "/data/routes/home_r1_drive.json";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--world-config") == 0 && i + 1 < argc) {
            world_config_path = argv[++i];
        } else if (std::strcmp(argv[i], "--route") == 0 && i + 1 < argc) {
            route_path = argv[++i];
        } else {
            std::fprintf(stderr, "usage: route_check [--world-config PATH] [--route PATH]\n");
            return 2;
        }
    }

    std::string err;
    const auto world = rg::load_world_config(world_config_path, &err);
    if (!world.has_value()) {
        std::fprintf(stderr, "route_check: %s\n", err.c_str());
        return 2;
    }
    const auto route = rg::load_route(route_path, &err);
    if (!route.has_value()) {
        std::fprintf(stderr, "route_check: %s\n", err.c_str());
        return 2;
    }
    std::unique_ptr<rg::WorldTerrain> terrain = rg::WorldTerrain::open(*world, &err);
    if (terrain == nullptr) {
        std::fprintf(stderr, "route_check: %s\n", err.c_str());
        return 2;
    }

    std::printf("route_check: world_config=%s\n", world_config_path.c_str());
    std::printf("route_check: route=%s (%s)\n", route_path.c_str(), route->name.c_str());
    const rg::RouteCheckReport report = rg::check_route_on_world(*route, *world, *terrain);
    std::fputs(rg::format_route_report(report).c_str(), stdout);
    const rg::WorldTerrain::FetchStats stats = terrain->fetch_stats();
    std::printf("tiles: server_ok=%llu server_miss=%llu cache_hits=%llu\n",
                static_cast<unsigned long long>(stats.server_ok), static_cast<unsigned long long>(stats.server_miss),
                static_cast<unsigned long long>(stats.cache_hits));
    return report.ok() ? 0 : 1;
}
