// tools/lod_measure — PLAN.md R2.1's own "measure, do not guess" LOD-distance
// sweep: opens rg::WorldTerrain against the real committed
// data/world/world_config.json (the actual home-r1 store, not a synthetic
// one) and, for each of max_distance_m in {6000, 12000, 20000} (max_level
// fixed at 6, matching the pyramid's own L0-L6 range - PLAN.md R2.1's spec),
// runs rg::WorldTerrain::build_static_view around the configured spawn point
// twice: once "cold" (this WorldTerrain's height_tile cache and, unless a
// derived store already exists on disk, the on-demand Cache->Derive->
// Upstream path too) and once "warm" (same WorldTerrain instance, so its
// in-process height_cache_ is now fully populated - the second call touches
// no derivation/decode work at all, only already-cached HeightTiles).
//
// Baking cache/g2m/home-r1/derived via g2m_tiler.exe was denied by the
// session's own permission classifier and not retried (see the repo's R2.1
// handback report) - this tool's own on-demand derivation is therefore the
// only measurement available; "cold" here means "this process's first
// touch of each tile", not "against a pre-baked store". RG_G2M_DERIVED
// defaults (if unset) to an out-of-tree, gitignored directory
// (out/g2m_derived/home-r1) so repeated runs of this tool build up their own
// on-disk derived-tile cache across process invocations without ever
// touching cache/g2m/home-r1/derived or the read-only source store.
//
// Prints one line per max_distance_m: chunk count, total vertex count, cold
// build_static_view wall time (ms), warm build_static_view wall time (ms).
// PLAN.md R2.1's target: warm preview load (this tool's "warm" number, which
// is what a second run of the real game against an already-populated
// RG_G2M_DERIVED store would see) stays under 3000 ms.

#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

// Mirrors rg::WorldConfig's own RG_G2M_DERIVED-empty-string convention
// (world_config.cpp: only a non-empty value overrides) - see that file's
// safe_getenv for why this is _dupenv_s on Windows rather than plain
// std::getenv (deprecation warning -> hard failure under rg_warnings' /WX).
std::string default_derived_dir(const std::string& repo_root) {
    return repo_root + "/out/g2m_derived/home-r1";
}

} // namespace

int main(int argc, char** argv) {
    // Optional single-value override (argv[1], a max_distance_m in metres):
    // used to take a genuinely independent "first ever run" cold measurement
    // per distance (wipe RG_G2M_DERIVED, run once) - the default no-arg sweep
    // runs all three values back-to-back in ONE process against the SAME
    // on-disk derived store, so only the very first value's "cold" number is
    // a true from-scratch measurement; later values benefit from tiles the
    // earlier values already derived onto disk (see the report/CLAUDE.md
    // table's own footnote).
    std::optional<double> single_distance_m;
    if (argc > 1) {
        single_distance_m = std::atof(argv[1]);
    }

#ifdef _WIN32
    char* existing = nullptr;
    std::size_t existing_len = 0;
    const bool has_existing = _dupenv_s(&existing, &existing_len, "RG_G2M_DERIVED") == 0 && existing != nullptr &&
                              std::strlen(existing) > 0;
    if (existing != nullptr) {
        std::free(existing);
    }
    if (!has_existing) {
        _putenv_s("RG_G2M_DERIVED", default_derived_dir(RG_SOURCE_DIR).c_str());
    }
#else
    if (std::getenv("RG_G2M_DERIVED") == nullptr) {
        setenv("RG_G2M_DERIVED", default_derived_dir(RG_SOURCE_DIR).c_str(), /*overwrite=*/0);
    }
#endif

    const std::string world_config_path = std::string(RG_SOURCE_DIR) + "/data/world/world_config.json";
    std::string err;
    auto config = rg::load_world_config(world_config_path, &err);
    if (!config.has_value()) {
        std::fprintf(stderr, "lod_measure: load_world_config failed: %s\n", err.c_str());
        return 1;
    }

    std::printf("lod_measure: world_config = %s\n", world_config_path.c_str());
    std::printf("lod_measure: RG_G2M_DERIVED = %s\n", default_derived_dir(RG_SOURCE_DIR).c_str());
    std::printf("%-14s %10s %14s %14s %14s\n", "max_dist_m", "chunks", "total_verts", "cold_ms", "warm_ms");

    const std::vector<double> sweep =
        single_distance_m.has_value() ? std::vector<double>{*single_distance_m} : std::vector<double>{6000.0, 12000.0, 20000.0};
    int exit_code = 0;
    for (double max_distance_m : sweep) {
        rg::WorldConfig cfg = *config;
        cfg.lod.max_distance_m = max_distance_m;

        std::string open_err;
        std::unique_ptr<rg::WorldTerrain> terrain = rg::WorldTerrain::open(cfg, &open_err);
        if (terrain == nullptr) {
            std::fprintf(stderr, "lod_measure: WorldTerrain::open failed for max_distance_m=%.0f: %s\n",
                         max_distance_m, open_err.c_str());
            exit_code = 1;
            continue;
        }

        const double cam_x = cfg.spawn.e - cfg.session_origin_utm.e0;
        const double cam_y = cfg.spawn.n - cfg.session_origin_utm.n0;

        std::vector<rg::RenderChunk> chunks;
        const auto cold_start = std::chrono::steady_clock::now();
        terrain->build_static_view(cam_x, cam_y, chunks);
        const auto cold_end = std::chrono::steady_clock::now();
        const double cold_ms = std::chrono::duration<double, std::milli>(cold_end - cold_start).count();

        std::size_t total_vertices = 0;
        for (const rg::RenderChunk& chunk : chunks) {
            total_vertices += chunk.mesh.positions.size() / 3;
        }

        std::vector<rg::RenderChunk> chunks_warm;
        const auto warm_start = std::chrono::steady_clock::now();
        terrain->build_static_view(cam_x, cam_y, chunks_warm);
        const auto warm_end = std::chrono::steady_clock::now();
        const double warm_ms = std::chrono::duration<double, std::milli>(warm_end - warm_start).count();

        const rg::WorldTerrain::FetchStats stats = terrain->fetch_stats();
        std::printf("%-14.0f %10zu %14zu %14.2f %14.2f\n", max_distance_m, chunks.size(), total_vertices, cold_ms,
                    warm_ms);
        std::printf("  fetch_stats: cache_hits=%llu server_ok=%llu server_miss=%llu\n",
                    static_cast<unsigned long long>(stats.cache_hits),
                    static_cast<unsigned long long>(stats.server_ok),
                    static_cast<unsigned long long>(stats.server_miss));
    }

    return exit_code;
}
