// test_world_terrain.cpp — rg::build_static_view_from_lookup coverage
// (PLAN.md R2.1 acceptance criteria): a synthetic in-memory
// TileKey -> HeightTile store (no g2m::TileStore/Server/geo2map decode
// machinery at all - this project links geo2map_engine with
// G2M_BUILD_IMPORT/TESTS/APPS/FUZZERS all OFF, so none of that is available
// here anyway) gives a sorted, 2:1-balanced chunk set, exact origin_session
// values, and byte-identical output for 1 vs N worker threads. The
// decode_height_tile_container cases at the end build real containers with
// geo2map_engine's own encode_height_tile/encode_body/assemble_container
// (all in g2m_core) to pin the header height_offset handling.
//
// TOOL-030/031 note (vault, physics_sim): SyntheticStore's cache is a
// std::mutex-guarded std::unordered_map, never a vector<bool> (TOOL-030,
// irrelevant here anyway - no bool storage), and every Catch2 assertion runs
// on the TEST's own thread only, never inside a worker lambda (TOOL-031) -
// build_static_view_from_lookup's worker threads only call SyntheticStore's
// plain C++ methods and geo2map_engine's own mesh-building free functions.
#include "rg/world_terrain.h"

#include "g2m/layer/src_osm.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/layer/tile_container.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

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
            return it->second.get();
        }
        // Heap, never the stack or by value in the map (vault TOOL-039:
        // g2m::HeightTile is 256 KiB).
        auto tile = std::make_unique<g2m::HeightTile>();
        tile->key = key;
        // 100 m flat, exact in 1/256 m fixed point (100 * 256 = 25600).
        tile->h.fill(100 * 256);
        tile->has_nodata = false;
        auto [inserted_it, inserted] = tiles_.emplace(packed, std::move(tile));
        (void)inserted; // a racing insert of the identical key is fine - same bytes either way
        return inserted_it->second.get();
    }

private:
    std::mutex mutex_;
    std::unordered_map<std::uint64_t, std::unique_ptr<g2m::HeightTile>> tiles_;
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

// A chunk's footprint as it is actually RENDERED through the Godot adapter,
// not raw double geometry: godot_ext/src/rg_terrain_view.cpp's
// chunk_instance_transform feeds (origin_session - render_origin_session),
// both ps::real/double, through external/physics_sim/adapters/godot/src/
// frame_convert_core.h's iso_to_godot_position, which casts to float32 only
// AFTER that double subtraction (Godot's Transform3D::origin is
// real_t=float). TerrainChunkMesh::positions are already float, local to
// that origin (terrain_chunk.h's own doc comment), so a chunk's rendered far
// edge is float(origin) + float(64*step) - each term exact in float32 here
// (mesh.origin components are small sums of powers of two plus exact
// halves, well inside float32's 24-bit mantissa; 64*step is an exact power
// of two for every level this file's tests use), which is why the
// zero-gap/zero-overlap test above measures kExact = 0.0 rather than
// picking a tolerance. No set_render_origin() call happens in these tests,
// so render_origin_session is the zero it defaults to (rg_terrain_view.h).
Footprint float_footprint_of(const g2m::mesh::TerrainChunkMesh& mesh) {
    const float step = static_cast<float>(std::int64_t{1} << mesh.key.level);
    const float size_m = 64.0f * step;
    const float x0 = static_cast<float>(mesh.origin[0]);
    const float y0 = static_cast<float>(mesh.origin[1]);
    return Footprint{x0, y0, x0 + size_m, y0 + size_m};
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

TEST_CASE("build_static_view_from_lookup: LOD-transition chunk footprints share exact zero-gap/zero-overlap borders",
         "[world_terrain]") {
    // R2.1 crack-lines FIX (geo2map_engine 87188a6, "fix(mesh): PLAN.md
    // G2.3a - nest LOD chunk footprints on the L0 sample lattice", bumped
    // into this submodule pin 2026-09-26): level k>=1 vertex i now sits at
    // (64*cx+i)*2^k + 0.5 (the SAME L0 sample-centre lattice every level
    // nests onto), not the old per-level sample centre (64*cx+i+0.5)*2^k
    // that left a systematic 0.5*step_finer gap/overlap at every 2:1 LOD
    // boundary. TerrainChunkMesh::origin/lod_select.cpp's chunk_rect() use
    // the same nested formula, so a coarse leaf's footprint now shares an
    // EXACT border with its finer neighbour's - no gap exposing whatever is
    // behind the terrain, no overlap z-fighting.
    //
    // This is the trip-wire's inverse (the ORIGINAL form of this test,
    // added 2026-09-26 alongside the crack-lines investigation, pinned
    // exactly the 0.5*step_finer defect on 52/52 edges before this fix
    // landed - see this file's own prior revision). It is pinned against the
    // REAL select_chunks/build_chunk pipeline (not reimplemented) with flat
    // synthetic height data, same as before - the nesting is pure
    // chunk-footprint geometry, independent of height/pyramid data. If a
    // future geo2map_engine change reopens the gap/overlap, this test
    // should start failing again.
    //
    // Footprint edges are computed with the SAME float arithmetic the Godot
    // adapter applies to a chunk's world position (godot_ext/src/
    // rg_terrain_view.cpp's chunk_instance_transform ->
    // external/physics_sim/adapters/godot/src/frame_convert_core.h's
    // iso_to_godot_position: the session-local origin minus the render
    // origin is computed in ps::real/double, THEN cast to float32 - Godot's
    // Transform3D::origin is real_t=float; TerrainChunkMesh::positions are
    // already float, local to that origin, so a chunk's rendered far edge is
    // float(origin) + float(64*step), not raw double arithmetic). No
    // set_render_origin() call happens in this test, so the render origin is
    // zero (rg_terrain_view.h: "0.0 before load_preview/set_render_origin"),
    // leaving only the origin's own float32 cast as a possible rounding
    // source. Measured: exactly 0.0 at every LOD boundary this test selects
    // (mesh.origin/chunk sizes here are small sums of powers of two plus
    // exact halves - all exactly representable in float32's 24-bit mantissa,
    // per this file's float_footprint_of comment) - kExact below is that
    // measured value, not a loosely chosen tolerance.
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
    constexpr double kExact = 0.0; // measured (see header comment): no float32-cast rounding at this test's scale

    for (const rg::RenderChunk& ci : out) {
        const g2m::mesh::ChunkKey& ki = ci.mesh.key;
        const Footprint fi = float_footprint_of(ci.mesh);
        for (const rg::RenderChunk& cj : out) {
            const g2m::mesh::ChunkKey& kj = cj.mesh.key;
            if (kj.level + 1 != ki.level) continue; // fi = coarse, fj = its finer neighbour; each edge seen once
            const Footprint fj = float_footprint_of(cj.mesh);
            const double step_fine = static_cast<double>(std::int64_t{1} << kj.level);
            const bool x_overlaps = fi.x0 < fj.x1 - 1e-6 && fj.x0 < fi.x1 - 1e-6;
            const bool y_overlaps = fi.y0 < fj.y1 - 1e-6 && fj.y0 < fi.y1 - 1e-6;

            if (y_overlaps && std::abs(fi.x0 - fj.x1) < step_fine) { // coarse west border
                const double gap = std::abs(fi.x0 - fj.x1);
                CHECK(gap <= kExact);
                max_gap = std::max(max_gap, gap);
                ++lod_boundary_edges;
            }
            if (y_overlaps && std::abs(fi.x1 - fj.x0) < step_fine) { // coarse east border
                const double overlap = std::abs(fi.x1 - fj.x0);
                CHECK(overlap <= kExact);
                max_overlap = std::max(max_overlap, overlap);
                ++lod_boundary_edges;
            }
            if (x_overlaps && std::abs(fi.y0 - fj.y1) < step_fine) { // coarse south border
                const double gap = std::abs(fi.y0 - fj.y1);
                CHECK(gap <= kExact);
                max_gap = std::max(max_gap, gap);
                ++lod_boundary_edges;
            }
            if (x_overlaps && std::abs(fi.y1 - fj.y0) < step_fine) { // coarse north border
                const double overlap = std::abs(fi.y1 - fj.y0);
                CHECK(overlap <= kExact);
                max_overlap = std::max(max_overlap, overlap);
                ++lod_boundary_edges;
            }
        }
    }

    WARN("LOD-boundary edges checked: " << lod_boundary_edges << ", max gap: " << max_gap
                                        << " m, max overlap: " << max_overlap << " m");
    // The test's own params must actually produce at least one 2:1 transition,
    // or the checks above never ran (no vacuous pass).
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
        CHECK(a.key == b.key);         // R8: RenderChunk.key
        CHECK(a.key == a.mesh.key);
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

// --- fetch_height_tile_cached (R2.2 plan section 3: the double-checked-
// insert cache behind WorldTerrain::height_tile_shared) ---

TEST_CASE("fetch_height_tile_cached: 8 threads hammering overlapping keys observe one identical tile per key",
         "[world_terrain]") {
    // Concurrency coverage for the double-checked-insert cache, exercised
    // directly via a synthetic fetch_fn - no g2m::Server/TileStore involved
    // at all (same "testable seam" precedent as this file's other tests,
    // e.g. SyntheticStore above). Under a release (non-sanitizer) build this
    // is a smoke test only - a genuine data race is not guaranteed to be
    // caught without TSan - see world_terrain.h's own fetch_height_tile_cached
    // doc comment for the locking scheme this pins.
    //
    // TOOL-031: no Catch2 assertion runs on a worker thread. Each thread
    // writes only into its OWN row of a preallocated per-thread result
    // vector (never another thread's row) - every CHECK/REQUIRE below runs
    // on the test's own thread, after every worker has been joined.
    constexpr int kThreadCount = 8;
    constexpr int kIterationsPerThread = 200;
    constexpr int kKeyCount = 4;

    const g2m::geo::UtmZone zone{32, g2m::geo::Hemisphere::North};
    std::array<g2m::TileKey, kKeyCount> keys;
    for (int i = 0; i < kKeyCount; ++i) {
        keys[static_cast<std::size_t>(i)] = g2m::TileKey{zone, /*level=*/3, /*x=*/i, /*y=*/i * 2};
    }

    std::atomic<int> fetch_call_count{0};
    auto fetch_fn = [&](const g2m::TileKey& key) -> rg::HeightTileFetchResult {
        fetch_call_count.fetch_add(1, std::memory_order_relaxed);
        // A brief sleep widens the race window between the cache-miss check
        // and the eventual insert, so overlapping threads are much more
        // likely to actually race on the SAME key rather than serialising by
        // accident.
        std::this_thread::sleep_for(std::chrono::microseconds(200));
        auto tile = std::make_shared<g2m::HeightTile>(); // heap: TOOL-039
        tile->key = key;
        tile->h.fill(key.x * 1000 + key.y);
        tile->has_nodata = false;
        return rg::HeightTileFetchResult{g2m::Status::Ok, std::move(tile)};
    };

    std::mutex cache_mutex;
    std::map<g2m::TileKey, std::shared_ptr<const g2m::HeightTile>> cache;

    // Each thread's own row: one result (tile + which key it asked for) per
    // iteration, written only by that thread.
    std::vector<std::vector<std::shared_ptr<const g2m::HeightTile>>> per_thread_tile(
        kThreadCount, std::vector<std::shared_ptr<const g2m::HeightTile>>(kIterationsPerThread));
    std::vector<std::vector<int>> per_thread_key_index(kThreadCount,
                                                        std::vector<int>(kIterationsPerThread));

    std::vector<std::thread> workers;
    workers.reserve(kThreadCount);
    for (int t = 0; t < kThreadCount; ++t) {
        workers.emplace_back([&, t]() {
            for (int i = 0; i < kIterationsPerThread; ++i) {
                const int key_index = (t + i) % kKeyCount;
                rg::HeightTileFetchResult result = rg::fetch_height_tile_cached(
                    cache_mutex, cache, keys[static_cast<std::size_t>(key_index)], fetch_fn);
                per_thread_tile[static_cast<std::size_t>(t)][static_cast<std::size_t>(i)] = result.tile;
                per_thread_key_index[static_cast<std::size_t>(t)][static_cast<std::size_t>(i)] = key_index;
            }
        });
    }
    for (std::thread& w : workers) {
        w.join();
    }

    // Every call must have succeeded and returned CORRECT content, and every
    // caller asking for the same key must have received the exact same
    // shared_ptr (the winning insert) - not merely equal content.
    std::array<const g2m::HeightTile*, kKeyCount> winning_ptr{};
    winning_ptr.fill(nullptr);

    for (int t = 0; t < kThreadCount; ++t) {
        for (int i = 0; i < kIterationsPerThread; ++i) {
            const int key_index = per_thread_key_index[static_cast<std::size_t>(t)][static_cast<std::size_t>(i)];
            const std::shared_ptr<const g2m::HeightTile>& tile =
                per_thread_tile[static_cast<std::size_t>(t)][static_cast<std::size_t>(i)];
            const g2m::TileKey& key = keys[static_cast<std::size_t>(key_index)];

            REQUIRE(tile != nullptr);
            CHECK(tile->key == key);
            CHECK(tile->h[0] == key.x * 1000 + key.y);

            const std::size_t ki = static_cast<std::size_t>(key_index);
            if (winning_ptr[ki] == nullptr) {
                winning_ptr[ki] = tile.get();
            } else {
                CHECK(winning_ptr[ki] == tile.get());
            }
        }
    }

    // The cache map itself ended up with exactly one entry per key, each
    // holding the same winning pointer every caller observed.
    {
        std::lock_guard<std::mutex> lk(cache_mutex);
        CHECK(cache.size() == static_cast<std::size_t>(kKeyCount));
        for (int i = 0; i < kKeyCount; ++i) {
            auto it = cache.find(keys[static_cast<std::size_t>(i)]);
            REQUIRE(it != cache.end());
            CHECK(it->second.get() == winning_ptr[static_cast<std::size_t>(i)]);
        }
    }

    // Loose sanity bound (not a tight assertion): fetch_fn was almost
    // certainly called more than kKeyCount times (racing threads before the
    // cache absorbed each key) but never more than the total call count -
    // redundant concurrent fetches for the same missing key are expected and
    // bounded, not eliminated (see world_terrain.h's own doc comment).
    const int calls = fetch_call_count.load();
    CHECK(calls >= kKeyCount);
    CHECK(calls <= kThreadCount * kIterationsPerThread);
}

// --- decode_height_tile_container (the header's height_offset, applied the
// way geo2map's TransportHeightTileFetch applies it) ---

namespace {

const g2m::TileKey kOffsetTestKey{g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North}, /*level=*/0, /*x=*/1813,
                                  /*y=*/21714};

// A heap tile (TOOL-039) with a per-sample pattern, 1/256 m.
std::unique_ptr<g2m::HeightTile> make_pattern_tile() {
    auto tile = std::make_unique<g2m::HeightTile>();
    tile->key = kOffsetTestKey;
    for (std::size_t i = 0; i < tile->h.size(); ++i) {
        tile->h[i] = 100 * 256 + static_cast<std::int32_t>(i % 997) - 400;
    }
    tile->has_nodata = false;
    return tile;
}

// HEADER || BODY for `tile`, the header carrying `height_offset`, `layer` and
// `header_key` - the exact bytes a g2m::TileResponse would carry.
std::vector<std::uint8_t> make_container(const g2m::HeightTile& tile, std::int32_t height_offset,
                                         std::string layer = std::string(g2m::kTerrainHeightLayer),
                                         const g2m::TileKey& header_key = kOffsetTestKey) {
    g2m::Result<g2m::TileBody> body = g2m::encode_height_tile(tile);
    REQUIRE(body.ok());
    g2m::Result<std::vector<std::uint8_t>> body_bytes = g2m::encode_body(body.value());
    REQUIRE(body_bytes.ok());
    g2m::TileHeader header;
    header.layer = std::move(layer);
    header.key = header_key;
    header.height_offset = height_offset;
    g2m::Result<std::vector<std::uint8_t>> container =
        g2m::assemble_container(header, std::span<const std::uint8_t>(body_bytes.value()));
    REQUIRE(container.ok());
    return std::move(container).value();
}

rg::HeightTileFetchResult decode_for_test_key(const std::vector<std::uint8_t>& container) {
    return rg::decode_height_tile_container(container, g2m::kTerrainHeightLayer, kOffsetTestKey);
}

} // namespace

TEST_CASE("decode_height_tile_container: offset 0 returns the stored heights unchanged", "[world_terrain]") {
    const std::unique_ptr<g2m::HeightTile> raw = make_pattern_tile();
    const rg::HeightTileFetchResult result = decode_for_test_key(make_container(*raw, 0));
    REQUIRE(result.status == g2m::Status::Ok);
    REQUIRE(result.tile != nullptr);
    CHECK(result.tile->key == kOffsetTestKey);
    CHECK(result.tile->h == raw->h);
    CHECK_FALSE(result.tile->has_nodata);
}

TEST_CASE("decode_height_tile_container: a non-zero height_offset is added to every sample except NoData",
         "[world_terrain]") {
    std::unique_ptr<g2m::HeightTile> raw = make_pattern_tile();
    // NoData at a corner, an edge and the middle.
    const std::array<std::size_t, 3> nodata_at{0, 255, 128 * 256 + 77};
    for (std::size_t i : nodata_at) {
        raw->h[i] = g2m::kHeightNoData;
    }
    raw->has_nodata = true;

    // +10 m and -150.5 m: both signs, and a non-whole-metre value.
    for (const std::int32_t offset : {10 * 256, -(150 * 256 + 128)}) {
        const rg::HeightTileFetchResult result = decode_for_test_key(make_container(*raw, offset));
        REQUIRE(result.status == g2m::Status::Ok);
        REQUIRE(result.tile != nullptr);
        CHECK(result.tile->has_nodata);
        std::size_t mismatches = 0;
        std::size_t nodata_seen = 0;
        for (std::size_t i = 0; i < raw->h.size(); ++i) {
            if (raw->h[i] == g2m::kHeightNoData) {
                ++nodata_seen;
                if (result.tile->h[i] != g2m::kHeightNoData) {
                    ++mismatches;
                }
            } else if (result.tile->h[i] != raw->h[i] + offset) {
                ++mismatches;
            }
        }
        CHECK(nodata_seen == nodata_at.size());
        CHECK(mismatches == 0);
    }
}

TEST_CASE("decode_height_tile_container: an offset that overflows int32 or lands on NoData rejects the tile",
         "[world_terrain]") {
    std::unique_ptr<g2m::HeightTile> raw = make_pattern_tile();
    const std::int32_t int_max = std::numeric_limits<std::int32_t>::max();

    SECTION("a sum above INT32_MAX") {
        raw->h[1000] = int_max - 10;
        const rg::HeightTileFetchResult result = decode_for_test_key(make_container(*raw, 11));
        CHECK(result.status == g2m::Status::Internal);
        CHECK(result.tile == nullptr);
    }
    SECTION("a sum of exactly INT32_MAX is still accepted") {
        raw->h[1000] = int_max - 10;
        const rg::HeightTileFetchResult result = decode_for_test_key(make_container(*raw, 10));
        REQUIRE(result.status == g2m::Status::Ok);
        REQUIRE(result.tile != nullptr);
        CHECK(result.tile->h[1000] == int_max);
    }
    SECTION("a sum equal to kHeightNoData") {
        raw->h[1000] = g2m::kHeightNoData + 5;
        const rg::HeightTileFetchResult result = decode_for_test_key(make_container(*raw, -5));
        CHECK(result.status == g2m::Status::Internal);
        CHECK(result.tile == nullptr);
    }
}

TEST_CASE("decode_height_tile_container: a header layer or key other than the requested one rejects the tile",
         "[world_terrain]") {
    const std::unique_ptr<g2m::HeightTile> raw = make_pattern_tile();

    SECTION("layer") {
        const rg::HeightTileFetchResult result = decode_for_test_key(make_container(*raw, 256, "g2m.elev.base"));
        CHECK(result.status == g2m::Status::Internal);
        CHECK(result.tile == nullptr);
    }
    SECTION("key") {
        g2m::TileKey other = kOffsetTestKey;
        other.x += 1;
        const rg::HeightTileFetchResult result =
            decode_for_test_key(make_container(*raw, 256, std::string(g2m::kTerrainHeightLayer), other));
        CHECK(result.status == g2m::Status::Internal);
        CHECK(result.tile == nullptr);
    }
    SECTION("a truncated container") {
        std::vector<std::uint8_t> container = make_container(*raw, 256);
        container.resize(container.size() - 1);
        const rg::HeightTileFetchResult result = decode_for_test_key(container);
        CHECK(result.status == g2m::Status::Internal);
        CHECK(result.tile == nullptr);
    }
}

// --- decode_road_segments_response / fetch_road_segments_cached (G2.5a-grip
// R-b: WorldTerrain::road_segments_shared's own pure-decode and cache seams,
// factored out exactly like decode_height_tile_container/
// fetch_height_tile_cached above so the fake-transport cases below need no
// g2m::Server/TileStore/transport at all - just a synthetic g2m::Response or
// a synthetic RoadSegmentsFetchFn lambda) ---

namespace {

const g2m::TileKey kOsmTestKey{g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North}, /*level=*/2, /*x=*/7, /*y=*/3};

// HEADER || BODY for an OsmTile (default: empty OsmData - a validly decoded
// tile with zero ways), the exact bytes a g2m::TileResponse would carry for
// g2m.src.osm.
std::vector<std::uint8_t> make_osm_container(const g2m::OsmTile& tile = g2m::OsmTile{},
                                             std::string layer = std::string(g2m::kSrcOsmLayer),
                                             const g2m::TileKey& header_key = kOsmTestKey) {
    g2m::Result<g2m::TileBody> body = g2m::encode_src_osm(tile);
    REQUIRE(body.ok());
    g2m::Result<std::vector<std::uint8_t>> body_bytes = g2m::encode_body(body.value());
    REQUIRE(body_bytes.ok());
    g2m::TileHeader header;
    header.layer = std::move(layer);
    header.key = header_key;
    g2m::Result<std::vector<std::uint8_t>> container =
        g2m::assemble_container(header, std::span<const std::uint8_t>(body_bytes.value()));
    REQUIRE(container.ok());
    return std::move(container).value();
}

g2m::Response make_tile_response(g2m::Status status, std::vector<std::uint8_t> container = {},
                                 std::string message = {}) {
    g2m::TileResponse response;
    response.meta.status = status;
    response.meta.message = std::move(message);
    response.container = std::move(container);
    return g2m::Response{std::move(response)};
}

} // namespace

TEST_CASE("decode_road_segments_response: classifies a fake transport response into Ok/Absent/Failed",
         "[world_terrain]") {
    const g2m::geo::UtmZone zone{32, g2m::geo::Hemisphere::North};

    SECTION("a non-TileResponse variant (e.g. the wrong endpoint) is Failed") {
        g2m::ManifestResponse manifest_response;
        const rg::RoadSegmentsResult result =
            rg::decode_road_segments_response(g2m::Response{manifest_response}, kOsmTestKey, zone);
        CHECK(result.status == rg::RoadFetchStatus::Failed);
        CHECK(result.segments == nullptr);
    }
    SECTION("a fake 503 (Unavailable) is Failed") {
        const rg::RoadSegmentsResult result =
            rg::decode_road_segments_response(make_tile_response(g2m::Status::Unavailable), kOsmTestKey, zone);
        CHECK(result.status == rg::RoadFetchStatus::Failed);
        CHECK(result.segments == nullptr);
    }
    SECTION("a fake 404 with message 'outside coverage' is Absent, with a non-null empty segment list") {
        const rg::RoadSegmentsResult result = rg::decode_road_segments_response(
            make_tile_response(g2m::Status::NotFound, {}, "outside coverage"), kOsmTestKey, zone);
        CHECK(result.status == rg::RoadFetchStatus::Absent);
        REQUIRE(result.segments != nullptr);
        CHECK(result.segments->empty());
    }
    SECTION("a fake 404 with a DIFFERENT message (e.g. 'unknown layer') is Failed, not Absent") {
        // has_road_layer() is the caller's own guard for "this release has no
        // g2m.src.osm layer at all" - server.cpp's check_tile() only ever
        // uses the literal "outside coverage" message for a genuine
        // coverage-boundary NotFound.
        const rg::RoadSegmentsResult result = rg::decode_road_segments_response(
            make_tile_response(g2m::Status::NotFound, {}, "unknown layer"), kOsmTestKey, zone);
        CHECK(result.status == rg::RoadFetchStatus::Failed);
        CHECK(result.segments == nullptr);
    }
    SECTION("Ok with a malformed container is Failed (a decode error)") {
        std::vector<std::uint8_t> container = make_osm_container();
        container.resize(container.size() - 1); // truncated: parse_container must reject it
        const rg::RoadSegmentsResult result =
            rg::decode_road_segments_response(make_tile_response(g2m::Status::Ok, container), kOsmTestKey, zone);
        CHECK(result.status == rg::RoadFetchStatus::Failed);
        CHECK(result.segments == nullptr);
    }
    SECTION("Ok with a header layer or key other than the requested one is Failed") {
        std::vector<std::uint8_t> wrong_layer = make_osm_container(g2m::OsmTile{}, "g2m.elev.base");
        CHECK(rg::decode_road_segments_response(make_tile_response(g2m::Status::Ok, wrong_layer), kOsmTestKey, zone)
                  .status == rg::RoadFetchStatus::Failed);

        g2m::TileKey other_key = kOsmTestKey;
        other_key.x += 1;
        std::vector<std::uint8_t> wrong_key =
            make_osm_container(g2m::OsmTile{}, std::string(g2m::kSrcOsmLayer), other_key);
        CHECK(rg::decode_road_segments_response(make_tile_response(g2m::Status::Ok, wrong_key), kOsmTestKey, zone)
                  .status == rg::RoadFetchStatus::Failed);
    }
    SECTION("Ok with a valid, empty-data container decodes to Ok with an empty (non-null) segment list") {
        const rg::RoadSegmentsResult result =
            rg::decode_road_segments_response(make_tile_response(g2m::Status::Ok, make_osm_container()), kOsmTestKey,
                                              zone);
        CHECK(result.status == rg::RoadFetchStatus::Ok);
        REQUIRE(result.segments != nullptr);
        CHECK(result.segments->empty());
    }
}

TEST_CASE("fetch_road_segments_cached: a Failed fetch (e.g. a fake 503) is not cached - a second call refetches",
         "[world_terrain]") {
    std::mutex cache_mutex;
    std::map<g2m::TileKey, rg::RoadSegmentsResult> cache;
    int fetch_calls = 0;
    rg::RoadSegmentsFetchFn fetch_fn = [&](const g2m::TileKey&) -> rg::RoadSegmentsResult {
        ++fetch_calls;
        return rg::RoadSegmentsResult{rg::RoadFetchStatus::Failed, nullptr};
    };

    bool was_cache_hit = true;
    rg::RoadSegmentsResult first = rg::fetch_road_segments_cached(cache_mutex, cache, kOsmTestKey, fetch_fn, &was_cache_hit);
    CHECK(first.status == rg::RoadFetchStatus::Failed);
    CHECK_FALSE(was_cache_hit);
    CHECK(cache.empty());

    rg::RoadSegmentsResult second =
        rg::fetch_road_segments_cached(cache_mutex, cache, kOsmTestKey, fetch_fn, &was_cache_hit);
    CHECK(second.status == rg::RoadFetchStatus::Failed);
    CHECK_FALSE(was_cache_hit);
    CHECK(cache.empty());
    CHECK(fetch_calls == 2); // never cached, so both calls actually refetched
}

TEST_CASE("fetch_road_segments_cached: an Ok or Absent result (e.g. a fake 404 'outside coverage') is cached - "
          "a second call does not refetch",
          "[world_terrain]") {
    SECTION("Absent") {
        std::mutex cache_mutex;
        std::map<g2m::TileKey, rg::RoadSegmentsResult> cache;
        int fetch_calls = 0;
        rg::RoadSegmentsFetchFn fetch_fn = [&](const g2m::TileKey&) -> rg::RoadSegmentsResult {
            ++fetch_calls;
            return rg::RoadSegmentsResult{rg::RoadFetchStatus::Absent,
                                          std::make_shared<const std::vector<g2m::RoadSegment>>()};
        };

        bool was_cache_hit = true;
        rg::RoadSegmentsResult first =
            rg::fetch_road_segments_cached(cache_mutex, cache, kOsmTestKey, fetch_fn, &was_cache_hit);
        CHECK(first.status == rg::RoadFetchStatus::Absent);
        CHECK_FALSE(was_cache_hit);
        REQUIRE(cache.count(kOsmTestKey) == 1);

        rg::RoadSegmentsResult second =
            rg::fetch_road_segments_cached(cache_mutex, cache, kOsmTestKey, fetch_fn, &was_cache_hit);
        CHECK(second.status == rg::RoadFetchStatus::Absent);
        CHECK(second.segments == first.segments); // same shared_ptr, served from the cache
        CHECK(was_cache_hit);
        CHECK(fetch_calls == 1); // the second call never invoked fetch_fn again
    }
    SECTION("Ok") {
        std::mutex cache_mutex;
        std::map<g2m::TileKey, rg::RoadSegmentsResult> cache;
        int fetch_calls = 0;
        auto segments = std::make_shared<const std::vector<g2m::RoadSegment>>(std::vector<g2m::RoadSegment>{
            g2m::RoadSegment{{0, 0}, {1000, 0}, 2000, g2m::LandClass::PavedRoad, g2m::SurfaceKind::Asphalt, 9},
        });
        rg::RoadSegmentsFetchFn fetch_fn = [&](const g2m::TileKey&) -> rg::RoadSegmentsResult {
            ++fetch_calls;
            return rg::RoadSegmentsResult{rg::RoadFetchStatus::Ok, segments};
        };

        bool was_cache_hit = true;
        rg::RoadSegmentsResult first =
            rg::fetch_road_segments_cached(cache_mutex, cache, kOsmTestKey, fetch_fn, &was_cache_hit);
        CHECK(first.status == rg::RoadFetchStatus::Ok);
        CHECK_FALSE(was_cache_hit);

        rg::RoadSegmentsResult second =
            rg::fetch_road_segments_cached(cache_mutex, cache, kOsmTestKey, fetch_fn, &was_cache_hit);
        CHECK(second.status == rg::RoadFetchStatus::Ok);
        CHECK(second.segments == first.segments);
        CHECK(was_cache_hit);
        CHECK(fetch_calls == 1);
    }
}
