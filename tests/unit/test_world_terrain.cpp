// test_world_terrain.cpp — rg::build_static_view_from_lookup coverage
// (PLAN.md R2.1 acceptance criteria): a synthetic in-memory
// TileKey -> HeightTile store (no g2m::TileStore/Server/geo2map decode
// machinery at all - this project links geo2map_engine with
// G2M_BUILD_IMPORT/TESTS/APPS/FUZZERS all OFF, so none of that is available
// here anyway) gives a sorted, 2:1-balanced chunk set, exact origin_session
// values, and byte-identical output for 1 vs N worker threads.
//
// TOOL-030/031 note (vault, physics_sim): SyntheticStore's cache is a
// std::mutex-guarded std::unordered_map, never a vector<bool> (TOOL-030,
// irrelevant here anyway - no bool storage), and every Catch2 assertion runs
// on the TEST's own thread only, never inside a worker lambda (TOOL-031) -
// build_static_view_from_lookup's worker threads only call SyntheticStore's
// plain C++ methods and geo2map_engine's own mesh-building free functions.
#include "rg/world_terrain.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <mutex>
#include <unordered_map>

namespace {

// A flat, always-valid (no NoData) synthetic terrain: every requested tile,
// at any level/x/y, is synthesized on first request and cached forever -
// gather_window/build_chunk therefore always see real data, whatever chunk
// keys select_chunks happens to produce for the test's camera/params. Held
// as *ctx (opaque to build_static_view_from_lookup), looked up through the
// plain g2m::mesh::TileLookup function pointer contract (not std::function -
// see terrain_chunk.h).
class SyntheticStore {
public:
    const g2m::HeightTile* get_or_create(const g2m::TileKey& key) {
        const std::uint64_t packed = key.packed();
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = tiles_.find(packed);
        if (it != tiles_.end()) {
            return &it->second;
        }
        g2m::HeightTile tile;
        tile.key = key;
        // 100 m flat, exact in 1/256 m fixed point (100 * 256 = 25600).
        tile.h.fill(100 * 256);
        tile.has_nodata = false;
        auto [inserted_it, inserted] = tiles_.emplace(packed, std::move(tile));
        (void)inserted; // a racing insert of the identical key is fine - same bytes either way
        return &inserted_it->second;
    }

private:
    std::mutex mutex_;
    std::unordered_map<std::uint64_t, g2m::HeightTile> tiles_;
};

const g2m::HeightTile* synthetic_lookup(void* ctx, const g2m::TileKey& key) {
    return static_cast<SyntheticStore*>(ctx)->get_or_create(key);
}

// A level-L chunk's world-space footprint: 64 samples at 2^L m spacing per
// side (terrain_chunk.h: kChunkQuads=64 samples per chunk; terrain_layers.h:
// level L's own cell spacing is 2^L m), from mesh.origin (already the
// absolute UTM SW corner, per TerrainChunkMesh::origin's own doc comment).
struct Footprint {
    double x0, y0, x1, y1;
};

Footprint footprint_of(const g2m::mesh::TerrainChunkMesh& mesh) {
    const double size_m = 64.0 * static_cast<double>(std::int64_t{1} << mesh.key.level);
    return Footprint{mesh.origin[0], mesh.origin[1], mesh.origin[0] + size_m, mesh.origin[1] + size_m};
}

// Touching or overlapping in 2D (shared edge counts - that is exactly the
// case select_chunks' own restricted-quadtree balance pass must resolve).
bool footprints_adjacent_or_overlapping(const Footprint& a, const Footprint& b) {
    constexpr double kEps = 1e-6;
    const bool separated = a.x1 < b.x0 - kEps || b.x1 < a.x0 - kEps || a.y1 < b.y0 - kEps || b.y1 < a.y0 - kEps;
    return !separated;
}

} // namespace

TEST_CASE("build_static_view_from_lookup: sorted, 2:1-balanced, non-empty chunk set", "[world_terrain]") {
    SyntheticStore store;
    g2m::mesh::LodParams params;
    params.zone = g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North};
    params.range0_m = 64.0;
    params.max_level = 2;
    params.max_distance_m = 300.0;

    const double e0 = 500000.0;
    const double n0 = 5500000.0;

    std::vector<rg::RenderChunk> out;
    rg::build_static_view_from_lookup(/*cam_x=*/0.0, /*cam_y=*/0.0, params, e0, n0, &synthetic_lookup, &store,
                                      /*thread_count=*/1, out);

    REQUIRE_FALSE(out.empty());

    // Sorted by (level desc, cy, cx) - select_chunks' own documented order,
    // which build_static_view_from_lookup must preserve into `out` (PLAN.md
    // R2.1: "deterministic output order = select_chunks order").
    for (std::size_t i = 1; i < out.size(); ++i) {
        const g2m::mesh::ChunkKey& prev = out[i - 1].mesh.key;
        const g2m::mesh::ChunkKey& cur = out[i].mesh.key;
        const auto tie = [](const g2m::mesh::ChunkKey& k) { return std::tuple(-k.level, k.cy, k.cx); };
        CHECK(tie(prev) <= tie(cur));
    }

    // No duplicate chunk keys.
    for (std::size_t i = 0; i < out.size(); ++i) {
        for (std::size_t j = i + 1; j < out.size(); ++j) {
            CHECK_FALSE(out[i].mesh.key == out[j].mesh.key);
        }
    }

    // 2:1 restricted-quadtree balance: any two spatially adjacent/overlapping
    // chunks differ by at most one level.
    for (std::size_t i = 0; i < out.size(); ++i) {
        const Footprint fi = footprint_of(out[i].mesh);
        for (std::size_t j = i + 1; j < out.size(); ++j) {
            const Footprint fj = footprint_of(out[j].mesh);
            if (footprints_adjacent_or_overlapping(fi, fj)) {
                const int level_diff = std::abs(out[i].mesh.key.level - out[j].mesh.key.level);
                CHECK(level_diff <= 1);
            }
        }
    }

    // origin_session = mesh.origin - (e0, n0, 0), exact (plain double
    // subtraction of exactly-representable values).
    for (const rg::RenderChunk& chunk : out) {
        CHECK(chunk.origin_session[0] == chunk.mesh.origin[0] - e0);
        CHECK(chunk.origin_session[1] == chunk.mesh.origin[1] - n0);
        CHECK(chunk.origin_session[2] == chunk.mesh.origin[2]);
    }

    // Every vertex got a colour (chunk_vertex_colors ran).
    for (const rg::RenderChunk& chunk : out) {
        CHECK(chunk.rgba.size() == chunk.mesh.positions.size() / 3);
    }
}

TEST_CASE("build_static_view_from_lookup: LOD-transition chunk footprints are offset by half the finer level's cell",
         "[world_terrain]") {
    // R2.1 crack-lines investigation (2026-09-26): g2m_mesh gives every
    // chunk's footprint origin as (64*cx + 0.5) * 2^level (height_tile.h's
    // cell-registered "sample i's centre = min + (i+0.5)*step" convention,
    // terrain_chunk.h/lod_select.h both use the identical formula), applied
    // independently PER LEVEL with no cross-level nesting. A level-(L+1)
    // chunk at cx=C therefore does NOT cover the same footprint as its own 4
    // children combined: parent origin_e = (64*C+0.5)*2*step_L, first child
    // (cx=2*C) origin_e = (128*C+0.5)*step_L - the parent's whole footprint
    // sits exactly 0.5*step_L further east/north than its children's, purely
    // from the chunk-footprint formula, independent of any height data.
    //
    // At an ACTUAL selected 2:1 LOD boundary this is a systematic mismatch
    // between a coarse leaf's edge and its real finer neighbour's edge: a
    // GAP of exactly 0.5*step_finer on the coarse chunk's west/south border
    // (it starts 0.5 fine-cells short of where the finer neighbour ends) and
    // an OVERLAP of exactly 0.5*step_finer on its east/north border (it
    // extends 0.5 fine-cells past where the finer neighbour begins) - matching
    // both crack appearances seen in the R2.1 screenshots (a solid dark line
    // where the gap exposes whatever is behind the terrain, a dotted/moire
    // line where the overlapping near-coplanar surfaces z-fight).
    //
    // This is pinned here against the REAL select_chunks/build_chunk
    // pipeline (not reimplemented) with flat synthetic height data, because
    // the mismatch is pure chunk-footprint geometry - no height/pyramid data
    // is needed to reproduce it. It is g2m_mesh's own chunk-footprint
    // convention (terrain_chunk.h + lod_select.h), not a racing_game adapter
    // bug: see the R2.1 crack-lines report for the geo2map_engine proposal.
    // If a future g2m_mesh version makes LOD levels nest exactly, this test
    // should start failing (lod_boundary_edges > 0 with gap/overlap == 0) -
    // that is the intended trip-wire, update the assertions then.
    SyntheticStore store;
    g2m::mesh::LodParams params;
    params.zone = g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North};
    params.range0_m = 64.0;
    params.max_level = 2;
    params.max_distance_m = 300.0;

    std::vector<rg::RenderChunk> out;
    rg::build_static_view_from_lookup(0.0, 0.0, params, /*e0=*/0.0, /*n0=*/0.0, &synthetic_lookup, &store,
                                      /*thread_count=*/1, out);
    REQUIRE_FALSE(out.empty());

    int lod_boundary_edges = 0;
    double max_gap = 0.0;
    double max_overlap = 0.0;
    constexpr double kExact = 1e-9; // every quantity here is exact in double (powers of two, half-integers)

    for (const rg::RenderChunk& ci : out) {
        const g2m::mesh::ChunkKey& ki = ci.mesh.key;
        const Footprint fi = footprint_of(ci.mesh);
        for (const rg::RenderChunk& cj : out) {
            const g2m::mesh::ChunkKey& kj = cj.mesh.key;
            if (kj.level + 1 != ki.level) continue; // fi = coarse, fj = its finer neighbour; each edge seen once
            const Footprint fj = footprint_of(cj.mesh);
            const double step_fine = static_cast<double>(std::int64_t{1} << kj.level);
            const bool x_overlaps = fi.x0 < fj.x1 - 1e-6 && fj.x0 < fi.x1 - 1e-6;
            const bool y_overlaps = fi.y0 < fj.y1 - 1e-6 && fj.y0 < fi.y1 - 1e-6;

            if (y_overlaps && std::abs(fi.x0 - fj.x1) < step_fine) { // coarse west border
                const double gap = fi.x0 - fj.x1;
                CHECK(std::abs(gap - 0.5 * step_fine) < kExact);
                max_gap = std::max(max_gap, gap);
                ++lod_boundary_edges;
            }
            if (y_overlaps && std::abs(fi.x1 - fj.x0) < step_fine) { // coarse east border
                const double overlap = fi.x1 - fj.x0;
                CHECK(std::abs(overlap - 0.5 * step_fine) < kExact);
                max_overlap = std::max(max_overlap, overlap);
                ++lod_boundary_edges;
            }
            if (x_overlaps && std::abs(fi.y0 - fj.y1) < step_fine) { // coarse south border
                const double gap = fi.y0 - fj.y1;
                CHECK(std::abs(gap - 0.5 * step_fine) < kExact);
                max_gap = std::max(max_gap, gap);
                ++lod_boundary_edges;
            }
            if (x_overlaps && std::abs(fi.y1 - fj.y0) < step_fine) { // coarse north border
                const double overlap = fi.y1 - fj.y0;
                CHECK(std::abs(overlap - 0.5 * step_fine) < kExact);
                max_overlap = std::max(max_overlap, overlap);
                ++lod_boundary_edges;
            }
        }
    }

    WARN("LOD-boundary edges checked: " << lod_boundary_edges << ", max gap: " << max_gap
                                        << " m, max overlap: " << max_overlap << " m");
    // The test's own params must actually produce at least one 2:1 transition,
    // or the checks above never ran.
    CHECK(lod_boundary_edges > 0);
}

TEST_CASE("build_static_view_from_lookup: identical output for 1 vs N worker threads", "[world_terrain]") {
    SyntheticStore store;
    g2m::mesh::LodParams params;
    params.zone = g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North};
    params.range0_m = 64.0;
    params.max_level = 3;
    params.max_distance_m = 500.0;

    const double e0 = 464000.0;
    const double n0 = 5559000.0;

    std::vector<rg::RenderChunk> single_threaded;
    rg::build_static_view_from_lookup(120.0, -45.0, params, e0, n0, &synthetic_lookup, &store, /*thread_count=*/1,
                                      single_threaded);

    std::vector<rg::RenderChunk> multi_threaded;
    rg::build_static_view_from_lookup(120.0, -45.0, params, e0, n0, &synthetic_lookup, &store, /*thread_count=*/8,
                                      multi_threaded);

    REQUIRE(single_threaded.size() == multi_threaded.size());
    REQUIRE_FALSE(single_threaded.empty());

    for (std::size_t i = 0; i < single_threaded.size(); ++i) {
        const rg::RenderChunk& a = single_threaded[i];
        const rg::RenderChunk& b = multi_threaded[i];
        CHECK(a.mesh.key == b.mesh.key);
        CHECK(a.mesh.origin[0] == b.mesh.origin[0]);
        CHECK(a.mesh.origin[1] == b.mesh.origin[1]);
        CHECK(a.mesh.origin[2] == b.mesh.origin[2]);
        CHECK(a.mesh.positions == b.mesh.positions);
        CHECK(a.mesh.normals == b.mesh.normals);
        CHECK(a.mesh.land_class == b.mesh.land_class);
        CHECK(a.mesh.indices == b.mesh.indices);
        CHECK(a.mesh.skirt_first_vertex == b.mesh.skirt_first_vertex);
        CHECK(a.mesh.skirt_depth_m == b.mesh.skirt_depth_m);
        CHECK(a.mesh.content_hash == b.mesh.content_hash);
        CHECK(a.origin_session[0] == b.origin_session[0]);
        CHECK(a.origin_session[1] == b.origin_session[1]);
        CHECK(a.origin_session[2] == b.origin_session[2]);
        CHECK(a.rgba == b.rgba);
    }
}

TEST_CASE("build_static_view_from_lookup: degenerate max_distance still covers the camera's own chunk, no crash",
         "[world_terrain]") {
    // max_distance_m = 0 does NOT mean "select nothing" - the root chunk the
    // camera itself sits in is always at distance 0 <= 0, so select_chunks
    // still returns exactly that one chunk. This pins that (mildly
    // surprising) boundary behaviour and exercises thread_count > 1 with a
    // single-chunk result (the "n_threads <= 1 || keys.size() < 2" early-out
    // path in build_static_view_from_lookup).
    SyntheticStore store;
    g2m::mesh::LodParams params;
    params.zone = g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North};
    params.range0_m = 64.0;
    params.max_level = 0;
    params.max_distance_m = 0.0;

    std::vector<rg::RenderChunk> out;
    rg::build_static_view_from_lookup(0.0, 0.0, params, 0.0, 0.0, &synthetic_lookup, &store, /*thread_count=*/4, out);
    CHECK(out.size() == 1);
    CHECK(out[0].mesh.key.level == 0);
}
