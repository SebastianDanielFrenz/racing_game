// test_stutter_perf.cpp - hidden micro-benchmarks for per-frame / per-second main-thread diagnostics
// (render stutter investigation 2026-10-14). Run: rg_unit_tests "[.][perf][stutter]"
#include "rg/terrain_render.h"
#include "rg/terrain_stream.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {
const g2m::geo::UtmZone kZone{32, g2m::geo::Hemisphere::North};

rg::RenderChunk flat(int level, int cx, int cy) {
    rg::RenderChunk c;
    const g2m::mesh::ChunkKey key{kZone, static_cast<std::uint8_t>(level), cx, cy};
    c.key = key;
    c.mesh.key = key;
    const double spacing = std::ldexp(1.0, level);
    c.mesh.origin[0] = 64.0 * spacing * cx;
    c.mesh.origin[1] = 64.0 * spacing * cy;
    c.mesh.origin[2] = 100.0;
    constexpr int verts = g2m::mesh::kChunkVerts;
    c.mesh.positions.assign(static_cast<std::size_t>(verts) * verts * 3, 0.0f);
    return c;
}
} // namespace

TEST_CASE("rendered_vs_bed cost at game scale (RG_TERRAIN_STREAM bed probe, once per second on the main thread)",
          "[.][perf][stutter]") {
    // ~800 resident chunks: a 24x24 L0 block plus coarser rings.
    std::vector<rg::RenderChunk> chunks;
    for (int y = 0; y < 16; ++y) for (int x = 0; x < 16; ++x) chunks.push_back(flat(0, x, y));
    for (int y = 0; y < 24; ++y) for (int x = 0; x < 24; ++x) chunks.push_back(flat(1, 8 + x, 8 + y));
    for (int y = 0; y < 24; ++y) for (int x = 0; x < 24; ++x) chunks.push_back(flat(2, 4 + x, 4 + y));
    for (int y = 0; y < 8; ++y) for (int x = 0; x < 8; ++x) chunks.push_back(flat(3, x, y));
    for (const int bed_count : {2000, 10000, 30000}) {
        std::vector<std::array<double, 3>> beds;
        for (int i = 0; i < bed_count; ++i) beds.push_back({16.0 + std::fmod(i * 37.7, 1000.0), 16.0 + std::fmod(i * 91.3, 1000.0), 100.0});
        const auto t0 = std::chrono::steady_clock::now();
        const rg::BedCheck c = rg::rendered_vs_bed(chunks, chunks.size(), beds);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("RG_PERF rendered_vs_bed chunks=%zu beds=%d samples=%d ms=%.2f\n", chunks.size(), bed_count, static_cast<int>(c.samples), ms);
    }
    SUCCEED();
}
