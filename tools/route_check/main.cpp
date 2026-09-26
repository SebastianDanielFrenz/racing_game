// tools/route_check — checks an "rg.route/1" drive route against the real
// terrain store (R2.2 plan section 3, "tools/terrain_drive" route criteria):
// loads the world config and the route, opens rg::WorldTerrain on the
// config's stores, samples L0 heights along the route and prints length,
// grade (max/p99 over a 10 m window), physics-tile seam crossings, elevation
// range, NoData, the tightest corner and every stretch tighter than 30 m.
// Thin main over rg::check_route_on_world (core/include/rg/route_check.h) -
// R5's [realdata] drive test runs the same check.
//
// Usage: route_check [--world-config PATH] [--route PATH]
//                    [--min-length-m M] [--max-grade-pct P]
//                    [--min-seams N] [--min-corner-radius-m R]
//   defaults: <repo>/data/world/world_config.json,
//             <repo>/data/routes/home_r1_drive.json
//   Criteria: RouteCheckParams' defaults, then the route file's optional
//   "criteria" object, then these flags (last one wins).
// Exit: 0 = every criterion passes, 1 = a criterion failed,
//       2 = usage error / config, route or store failed to load.
// Godot-free; reads the store only through WorldTerrain (the config's
// source store is opened read-only).
#include "rg/route_check.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

namespace {

// Strict number parse for a flag value: the whole string, finite.
std::optional<double> parse_number(const char* text) {
    char* end = nullptr;
    const double v = std::strtod(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(v)) {
        return std::nullopt;
    }
    return v;
}

constexpr const char* kUsage =
    "usage: route_check [--world-config PATH] [--route PATH] [--min-length-m M] [--max-grade-pct P]\n"
    "                   [--min-seams N] [--min-corner-radius-m R]\n";

} // namespace

int main(int argc, char** argv) {
    std::string world_config_path = std::string(RG_SOURCE_DIR) + "/data/world/world_config.json";
    std::string route_path = std::string(RG_SOURCE_DIR) + "/data/routes/home_r1_drive.json";
    rg::RouteCriteria cli; // command-line overrides, applied after the route file's
    for (int i = 1; i < argc; ++i) {
        const char* flag = argv[i];
        if (std::strcmp(flag, "--world-config") == 0 && i + 1 < argc) {
            world_config_path = argv[++i];
            continue;
        }
        if (std::strcmp(flag, "--route") == 0 && i + 1 < argc) {
            route_path = argv[++i];
            continue;
        }
        const bool numeric = std::strcmp(flag, "--min-length-m") == 0 || std::strcmp(flag, "--max-grade-pct") == 0 ||
                             std::strcmp(flag, "--min-seams") == 0 ||
                             std::strcmp(flag, "--min-corner-radius-m") == 0;
        const std::optional<double> v = (numeric && i + 1 < argc) ? parse_number(argv[i + 1]) : std::nullopt;
        if (!v.has_value()) {
            std::fputs(kUsage, stderr);
            return 2;
        }
        ++i;
        if (std::strcmp(flag, "--min-length-m") == 0 && *v >= 0.0) {
            cli.min_length_m = *v;
        } else if (std::strcmp(flag, "--max-grade-pct") == 0 && *v > 0.0) {
            cli.max_grade = *v / 100.0;
        } else if (std::strcmp(flag, "--min-seams") == 0 && *v >= 0.0 && *v == std::floor(*v) && *v <= 1.0e6) {
            cli.min_seam_crossings = static_cast<int>(*v);
        } else if (std::strcmp(flag, "--min-corner-radius-m") == 0 && *v >= 0.0) {
            cli.min_corner_radius_m = *v;
        } else {
            std::fprintf(stderr, "route_check: invalid value for %s\n", flag);
            std::fputs(kUsage, stderr);
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
    rg::RouteCheckParams params;
    rg::apply_route_criteria(route->criteria, params);
    rg::apply_route_criteria(cli, params);
    std::printf("criteria: min_length_m=%.1f max_grade_pct=%.2f min_seam_crossings=%d min_corner_radius_m=%.1f\n",
                params.min_length_m, params.max_grade * 100.0, params.min_seam_crossings,
                params.min_corner_radius_m);
    const rg::RouteCheckReport report = rg::check_route_on_world(*route, *world, *terrain, params);
    std::fputs(rg::format_route_report(report).c_str(), stdout);
    const rg::WorldTerrain::FetchStats stats = terrain->fetch_stats();
    std::printf("tiles: server_ok=%llu server_miss=%llu cache_hits=%llu\n",
                static_cast<unsigned long long>(stats.server_ok), static_cast<unsigned long long>(stats.server_miss),
                static_cast<unsigned long long>(stats.cache_hits));
    return report.ok() ? 0 : 1;
}
