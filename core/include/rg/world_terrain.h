// rg/world_terrain.h — rg::WorldTerrain: the engine-neutral seam between
// geo2map_engine's offline in-process server/mesh stack and the game
// (PLAN.md R2.1). Owns one local geo2map_engine release built from a
// WorldConfig's source/derived stores and serves g2m::mesh::TerrainChunkMesh
// chunks around a camera point, with LOD (g2m::mesh::select_chunks) and a
// blocking, cached height-tile fetch path.
//
// No Godot type anywhere in this header or its .cpp (PLAN.md 11.1) -
// godot_ext/src/rg_terrain_view.h/.cpp is the only place a RenderChunk's
// data crosses into godot:: RenderingServer calls.
#pragma once

#include "rg/terrain_render.h"
#include "rg/world_config.h"

#include "g2m/core/geo/session_frame.h"
#include "g2m/layer/height_tile.h"
#include "g2m/mesh/lod_select.h"
#include "g2m/mesh/terrain_chunk.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/server.h"
#include "g2m/server/tile_store.h"
#include "g2m/server/upstream.h"
#include "g2m/client/in_process_transport.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace rg {

// Testable seam, independent of the Server/TileStore stack entirely (PLAN.md
// R2.1 acceptance: "build_static_view over a synthetic in-memory store" -
// tests/unit/test_world_terrain.cpp uses a plain std::map<TileKey,
// HeightTile> as `ctx`/`lookup` here, no TileStore/Server/geo2map decode
// machinery needed at all). WorldTerrain::build_static_view (below) is a
// thin wrapper calling this with its own height_tile() as the lookup.
//
// (cam_x, cam_y) are SESSION-LOCAL metres (rg::WorldConfig::spawn's own
// frame: x east of e0, y north of n0); e0/n0 are the session origin's own
// UTM easting/northing (rg::WorldConfig::session_origin_utm). Builds every
// selected chunk (g2m::mesh::select_chunks, then gather_window + build_chunk
// per chunk) using up to `thread_count` std::thread workers (1 = no threads
// spawned, runs on the caller's thread); `out` is resized to
// `select_chunks`'s own key count and each worker writes directly into its
// assigned index, so the result is byte-identical for any thread_count >= 1
// (PLAN.md R2.1 acceptance: "same output for 1 vs N threads") - select_chunks'
// own sort order is therefore also `out`'s final order.
void build_static_view_from_lookup(double cam_x, double cam_y, const g2m::mesh::LodParams& params, double e0,
                                   double n0, g2m::mesh::TileLookup lookup, void* ctx, unsigned thread_count,
                                   std::vector<RenderChunk>& out);

// The two halves build_static_view_from_lookup is made of (R2.2 R8 split
// them out so rg::TerrainViewStreamer can select, diff by key, and then build
// ONLY the chunks that are new):
//
// select_view_keys: g2m::mesh::select_chunks around the session-local point
// (cam_x, cam_y) - `out` in select_chunks' own order (level desc, cy, cx).
void select_view_keys(double cam_x, double cam_y, const g2m::mesh::LodParams& params, double e0, double n0,
                      std::vector<g2m::mesh::ChunkKey>& out);

// build_render_chunks: builds one RenderChunk per key into `out` (resized to
// keys.size(), out[i] <-> keys[i]) with up to `thread_count` workers, each
// writing only its own indices - byte-identical for any thread_count >= 1.
// `cancel` (optional): checked before each chunk; once it reads true the
// remaining chunks are skipped and the function returns false (`out` is then
// partially built and must be discarded). Returns true when every chunk was
// built.
bool build_render_chunks(const std::vector<g2m::mesh::ChunkKey>& keys, g2m::mesh::TileLookup lookup, void* ctx,
                         double e0, double n0, unsigned thread_count, std::vector<RenderChunk>& out,
                         const std::atomic<bool>* cancel = nullptr);

// Everything a chunk selection + build needs, bundled: the LOD params, the
// session origin (e0, n0) and the tile lookup. `lookup(ctx, key)` must be
// safe to call from several threads at once (WorldTerrain::height_tile is;
// so is the tests' synthetic store) and `ctx` must outlive whoever holds this
// struct. WorldTerrain::view_source() returns one over itself.
struct TerrainViewSource {
    g2m::mesh::LodParams params;
    double e0 = 0.0;
    double n0 = 0.0;
    g2m::mesh::TileLookup lookup = nullptr;
    void* ctx = nullptr;
};

// R2.2 plan section 3 / [AMEND] "R3 decoupling": racing_game's own small
// result struct for a cached height-tile lookup, built from EXISTING g2m
// types only (g2m::phys::IHeightTileFetch does not exist yet in the pinned
// geo2map_engine submodule - R4 will add the adapter to it once it lands).
// `tile` is non-null iff `status == g2m::Status::Ok`.
struct HeightTileFetchResult {
    g2m::Status status = g2m::Status::Internal;
    std::shared_ptr<const g2m::HeightTile> tile;
};

// Performs the actual (uncached) fetch + decode for one tile - called by
// fetch_height_tile_cached OUTSIDE any lock. WorldTerrain::height_tile_shared
// wraps its own private fetch_and_decode() into one of these;
// tests/unit/test_world_terrain.cpp's concurrency test instead supplies a
// synthetic lambda, with no Server/TileStore involved at all (same
// "testable seam" precedent as build_static_view_from_lookup's own
// TileLookup above).
using HeightTileFetchFn = std::function<HeightTileFetchResult(const g2m::TileKey&)>;

// Double-checked-insert cache: WorldTerrain::height_tile_shared's own
// implementation, factored out so it can be exercised directly (a synthetic
// fetch_fn, no real store) by the concurrency test. Locking scheme (see also
// the header's file-level comment): `cache_mutex` guards ONLY the `cache`
// map itself (the find and the eventual emplace), never `fetch_fn`'s own
// call, which always runs unlocked; two threads racing on the same missing
// key therefore both call `fetch_fn` concurrently (redundant but safe - the
// underlying store/decode work is independently idempotent, "both write
// identical bytes" per g2m::Server's own documented contract), and whichever
// thread's `cache.emplace` runs first wins the map slot - std::map::emplace
// never overwrites an existing key, so every later racer's own freshly
// fetched tile is simply discarded and every caller (winner and losers
// alike) returns the SAME shared_ptr, read back from the map after the
// emplace. A failed fetch (status != Ok) is never inserted, so a transient
// failure can be retried on the next call rather than being pinned forever.
// `was_cache_hit`, if non-null, is set to whether this call's initial lookup
// already found the key (WorldTerrain uses this for FetchStats::cache_hits;
// a fetch that then fails is not counted as a hit).
HeightTileFetchResult fetch_height_tile_cached(std::mutex& cache_mutex,
                                               std::map<g2m::TileKey, std::shared_ptr<const g2m::HeightTile>>& cache,
                                               const g2m::TileKey& key, const HeightTileFetchFn& fetch_fn,
                                               bool* was_cache_hit = nullptr);

class WorldTerrain {
public:
    ~WorldTerrain();
    WorldTerrain(const WorldTerrain&) = delete;
    WorldTerrain& operator=(const WorldTerrain&) = delete;

    // Opens the source store (read-only) and derived store (read-write) from
    // `config`, builds the one local release over
    // config.source_store.scope's raw tiles (g2m::builtin_local_release_params
    // + g2m::make_local_release - the SAME wiring g2m_tiler's own `bake`
    // subcommand and geo2map_engine's own golden tests use, PLAN.md decision
    // 10: "prebake and on-demand share the same resolver"), and adds it to an
    // offline g2m::Server (config.offline = true: the only IUpstream is a
    // g2m::LocalSourceUpstream over the source store, which never touches
    // the network - see g2m/server/upstream.h). On any failure (a store that
    // fails to open, a malformed release) returns nullptr and, if err is
    // non-null, sets *err to a human-readable message (the g2m::Error's own
    // ErrorCode/message - never throws, matching this repo's other loaders,
    // e.g. world_config.cpp's own TOOL-019 reasoning: geo2map_engine's own
    // Result<T> is exception-free throughout, so nothing here needs to
    // catch anything).
    static std::unique_ptr<WorldTerrain> open(const WorldConfig& config, std::string* err);

    // Blocking fetch + decode of one terrain.height tile (any level), cached
    // (thread-safe double-checked insert - see fetch_height_tile_cached's
    // own doc comment above for the exact locking scheme this wraps).
    // Returns nullptr if the tile is out of coverage or the request
    // otherwise fails (a missing tile in a LOD hole is expected at the edge
    // of the imported region - callers must handle it, see
    // g2m::mesh::gather_window's own NoData contract). Thin wrapper around
    // height_tile_shared() below with unchanged behaviour - the returned raw
    // pointer stays valid for WorldTerrain's own lifetime (the cache never
    // erases an entry, so the shared_ptr it holds never expires).
    const g2m::HeightTile* height_tile(const g2m::TileKey& key);

    // Same lookup as height_tile(), but returns the full
    // HeightTileFetchResult (status plus a shared_ptr the caller can hold
    // onto beyond WorldTerrain's own lifetime, e.g. to hand a tile to
    // another thread) instead of a bare raw pointer. [AMEND] "R3
    // decoupling": this is racing_game's own result type, built from
    // existing g2m types - R4 adapts it to g2m::phys::IHeightTileFetch once
    // that interface exists in the pinned geo2map_engine submodule.
    HeightTileFetchResult height_tile_shared(const g2m::TileKey& key);

    // Builds every LOD-selected chunk around (cam_x, cam_y) (session-local
    // metres) using this WorldTerrain's own store/server and
    // lod_params() - see build_static_view_from_lookup's doc comment above
    // for the determinism/threading contract this wraps.
    void build_static_view(double cam_x, double cam_y, std::vector<RenderChunk>& out);

    // A TerrainViewSource over this WorldTerrain (lod_params() as of THIS
    // call, the session origin, height_tile() as the lookup) for
    // rg::TerrainViewStreamer. The returned struct points at `this`: the
    // streamer holding it must be destroyed before this WorldTerrain.
    [[nodiscard]] TerrainViewSource view_source();

    [[nodiscard]] const g2m::mesh::LodParams& lod_params() const { return lod_params_; }
    void set_lod_params(const g2m::mesh::LodParams& params) { lod_params_ = params; }

    [[nodiscard]] const g2m::geo::SessionFrame& frame() const { return *frame_; }
    [[nodiscard]] const g2m::ReleaseId& release_id() const { return manifest_rid_; }

    // Diagnostics (PLAN.md R2.1's own "log clearly whether a tile came from
    // cache or was derived" requirement) - counts since open().
    struct FetchStats {
        std::uint64_t cache_hits = 0;   // height_tile() calls served from height_cache_
        std::uint64_t server_ok = 0;    // Server::tile() calls that returned 200
        std::uint64_t server_miss = 0;  // Server::tile() calls that returned anything else
    };
    [[nodiscard]] FetchStats fetch_stats() const;

private:
    WorldTerrain() = default;

    // Server::tile() + container decode for one terrain.height tile (any
    // level); status/tile are as they sound (tile is null on any failure:
    // out of coverage, a non-Ok status, a decode error). The tile is
    // heap-allocated (vault TOOL-039: g2m::HeightTile is 256 KiB, never by
    // value or on the stack, std::optional included). Updates
    // fetch_stats_ (server_ok/server_miss). Does NOT touch height_cache_ -
    // height_tile_shared() (the cached, public entry point) does that, via
    // fetch_height_tile_cached.
    struct FetchDecodeResult {
        g2m::Status status = g2m::Status::Internal;
        std::shared_ptr<const g2m::HeightTile> tile;
    };
    FetchDecodeResult fetch_and_decode(const g2m::TileKey& key);

    // Declaration order is the destruction-order contract this class relies
    // on (members destruct in REVERSE declaration order): local_upstream_
    // and transport_ both hold references into earlier members and must be
    // torn down first; server_'s ServerConfig holds pointers into derivers_
    // and local_upstream_, so it must go before them but after transport_.
    std::unique_ptr<g2m::TileStore> source_store_;
    std::unique_ptr<g2m::TileStore> derived_store_;
    g2m::BuiltinTerrainDerivers derivers_;
    std::optional<g2m::LocalSourceUpstream> local_upstream_;
    std::unique_ptr<g2m::Server> server_;
    std::optional<g2m::InProcessTransport> transport_;
    std::unique_ptr<g2m::geo::SessionFrame> frame_;

    g2m::ReleaseId manifest_rid_{};
    std::string terrain_height_layer_;
    g2m::mesh::LodParams lod_params_;

    std::mutex cache_mutex_;
    std::map<g2m::TileKey, std::shared_ptr<const g2m::HeightTile>> height_cache_;
    mutable std::mutex stats_mutex_;
    FetchStats stats_;
};

} // namespace rg
