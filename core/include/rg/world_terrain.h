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

#include "rg/road_classes.h"
#include "rg/buildings.h"
#include "g2m/world/road_ribbons.h"
#include "rg/road_surface.h"
#include "rg/terrain_render.h"
#include "rg/world_config.h"

#include "g2m/core/geo/session_frame.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/mesh/lod_select.h"
#include "g2m/mesh/terrain_chunk.h"
#include "g2m/world/chunk_road_classes.h"
#include "g2m/world/terrain_service.h"
#include "g2m/world/tile_fetch.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rg {

// The carve state of one height tile (a ChunkCarve value as uint8_t); `ctx` is the WorldTerrain.
using TileCarveFn = std::uint8_t (*)(void* ctx, const g2m::TileKey&);

// The worst carve state over the height tiles a chunk's halo window touches (what RenderChunk::carve is built from).
ChunkCarve chunk_window_carve(const g2m::mesh::ChunkKey& key, void* ctx, TileCarveFn fn);

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
// `class_lookup` (R-2, roads_plan.md): a null (default) g2m::world::ClassLookup builds
// every chunk exactly as before this parameter existed (byte-identical
// output - see g2m/world/chunk_road_classes.h's ClassLookup comment); a real one
// rasterises OSM road classes onto each chunk's vertices.
// `surfaces` (G2.5a-grip R-b): resolves a rasterised LandClass to a surface
// NAME for chunk_vertex_colors - a default-constructed RoadSurfaceMap
// matches this repo's pre-existing hardcoded paved/unpaved colours exactly
// (byte-identical output to before this parameter existed).
void build_static_view_from_lookup(double cam_x, double cam_y, const g2m::mesh::LodParams& params, double e0,
                                   double n0, g2m::mesh::TileLookup lookup, void* ctx, unsigned thread_count,
                                   std::vector<RenderChunk>& out, const g2m::world::ClassLookup& class_lookup = {},
                                   const RoadSurfaceMap& surfaces = {}, TileCarveFn carve_state = nullptr);

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
// `class_lookup` (R-2, roads_plan.md): see build_static_view_from_lookup's
// own comment above - null (default) is byte-identical to before this
// parameter existed. `surfaces` (G2.5a-grip R-b): ditto, see that function's
// own comment.
bool build_render_chunks(const std::vector<g2m::mesh::ChunkKey>& keys, g2m::mesh::TileLookup lookup, void* ctx,
                         double e0, double n0, unsigned thread_count, std::vector<RenderChunk>& out,
                         const std::atomic<bool>* cancel = nullptr, const g2m::world::ClassLookup& class_lookup = {},
                         const RoadSurfaceMap& surfaces = {}, TileCarveFn carve_state = nullptr);

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
    // R-2 (roads_plan.md): null = every chunk built exactly as before this
    // field existed (byte-identical output). WorldTerrain::view_source()
    // populates a real one over the same WorldTerrain.
    g2m::world::ClassLookup class_lookup{};
    // G2.5a-grip R-b: default-constructed = byte-identical to before this
    // field existed. WorldTerrain::view_source() populates this from the
    // WorldConfig::PhysicsTerrainConfig::road_surfaces this WorldTerrain was
    // opened with.
    RoadSurfaceMap surfaces{};
    // How a height tile was carved (a ChunkCarve value); null = every chunk reports NotCarved. A chunk's RenderChunk::carve
    // is the worst state over the tiles its height window touches. WorldTerrain::view_source() populates it.
    TileCarveFn carve_state = nullptr;
    // The render road LOD policy (in-game setting, rg/lod_settings.h) that class_lookup.lod points at; holding it
    // here keeps that pointer valid for every copy of this struct. Null = g2m::default_render_road_lod().
    std::shared_ptr<const g2m::RenderRoadLod> road_lod;
};

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
    std::shared_ptr<const g2m::OsmTile> source_osm_tile(const g2m::TileKey& key); // background consumers only
    g2m::world::RoadRibbonTile road_visual_tile(const g2m::TileKey& key,double lift_m);
    std::vector<Building> buildings_tile(const g2m::TileKey& key,double min_height,double fallback_height,double storey_height);
    // `priority` is what a still-missing tile is requested at (a tile already cached returns at once):
    // Render for presentation callers, the physics priority (physics_priority()) for the physics fetch.
    g2m::world::HeightTileFetchResult height_tile_shared(const g2m::TileKey& key,
                                                         g2m::world::Priority priority = g2m::world::Priority::Render);
    // The priority the physics fetch asks at: Gate until the start-up gate has opened, then Physics
    // (Session::setup_terrain flips it). Any thread.
    void set_physics_priority(g2m::world::Priority priority) noexcept {
        physics_priority_.store(static_cast<std::uint8_t>(priority), std::memory_order_relaxed);
    }
    [[nodiscard]] g2m::world::Priority physics_priority() const noexcept {
        return static_cast<g2m::world::Priority>(physics_priority_.load(std::memory_order_relaxed));
    }
    // The geo2map terrain service this WorldTerrain runs on (one worker pool, priorities, dedup).
    [[nodiscard]] g2m::world::TerrainService& service() { return *service_; }
    // Presentation-only lookup: never fetches or waits for derivation.
    const g2m::HeightTile* cached_height_tile(const g2m::TileKey& key);
    std::vector<std::shared_ptr<const RoadDeck>> road_decks();
    std::shared_ptr<const g2m::RoadGeomTile> cached_road_geometry_at(double easting,double northing);
    std::shared_ptr<const g2m::RoadGeomTile> road_geometry_at(double easting, double northing);

    // Blocking fetch + decode of one g2m.roads.graph tile (level 2, 1024 m; the OSM road topology with
    // way ids, node ids and ALL tags - no profile fitting, no DEM), cached. A key outside the release's
    // coverage yields an empty tile (like road_surface_patch); a fetch/decode failure yields nullptr and
    // is not cached. Used by the route generator (tools/rg_route_gen) and the route planner tests.
    // `priority`: what a missing tile is requested at (the route planner's region build asks at Warm).
    std::shared_ptr<const g2m::RoadGraphTile> road_graph_tile(const g2m::TileKey& key,
                                                              g2m::world::Priority priority = g2m::world::Priority::Render);

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

    // The coarsest pyramid level whose height tiles get the road bed carved in (default 2). Level 0 is what
    // physics collides with; L1 (2 m) and L2 (4 m) are carved with the SAME L2 road surface so the visible
    // ground within ~1 km matches the road bed instead of hiding it (owner 2026-10-13: "I drove through the
    // visible ground" - uncarved L1+ DEM sits above the carved road). Changing it only affects tiles produced
    // afterwards (call it right after open(), before any render chunk is built).
    // How the height tile `key` was produced (see ChunkCarve): Carved with the road surface, Fallback (uncarved because
    // the road surface could not be derived at that moment) or NotCarved (level above carve_max_level(), no road layer).
    // A key never produced reads NotCarved. Thread-safe.
    [[nodiscard]] ChunkCarve tile_carve(const g2m::TileKey& key) const;
    // Height tiles currently in the Fallback state (produced uncarved, not yet healed).
    [[nodiscard]] std::size_t fallback_tile_count() const;
    // Re-derives every Fallback tile whose road surface is available now (blocking per tile, on the caller thread);
    // returns how many became Carved. Call from a background thread.
    std::size_t heal_fallback_tiles();
    // Probe aid: the physics road bed (centre-line heights of the ground road segments) inside the UTM box
    // [e0,e1]x[n0,n1], from the road surfaces already derived (never waits). {easting, northing, z}.
    void road_bed_points(double e0, double e1, double n0, double n1, int stride, std::vector<std::array<double, 3>>& out);
    [[nodiscard]] int carve_max_level() const { return carve_max_level_; }
    void set_carve_max_level(int level) { carve_max_level_ = level < 0 ? 0 : (level > 2 ? 2 : level); }

    [[nodiscard]] const g2m::mesh::LodParams& lod_params() const { return lod_params_; }
    void set_lod_params(const g2m::mesh::LodParams& params) { lod_params_ = params; }
    // The LodParams as open() made them from the world config (before any in-game LOD setting).
    [[nodiscard]] const g2m::mesh::LodParams& world_lod_params() const { return world_lod_params_; }
    // The render road LOD policy (in-game setting; null = g2m::default_render_road_lod()) that view_source() and
    // build_static_view() build road classes with from now on. Thread-safe.
    void set_render_road_lod(std::shared_ptr<const g2m::RenderRoadLod> lod);
    [[nodiscard]] std::shared_ptr<const g2m::RenderRoadLod> render_road_lod() const;

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

    // Thin wrapper around road_segments_shared (G2.5a-grip R-b) for the
    // render path, which only ever needs "does this tile have roads", never
    // the Ok/Absent/Failed distinction: returns road_segments_shared's own
    // `segments` (Ok or Absent) or, on Failed, a non-null, empty vector -
    // "no roads" is a valid output, never fatal (roads_plan.md R-2). Both
    // this and road_segments_shared share the SAME cache.
    std::shared_ptr<const std::vector<g2m::RoadSegment>> road_segments(const g2m::TileKey& key);

    // Blocking fetch + decode of one g2m.src.osm tile's road segments
    // (roads_plan.md R-2, tri-state added G2.5a-grip R-b plan section 7),
    // cached (same double-checked-insert scheme as height_tile_shared, its
    // own dedicated mutex/map - see RoadFetchStatus's own doc comment for
    // exactly which statuses are cached). Fetches over the same
    // manifest_rid_/transport_ the height-tile path uses. Only a genuine
    // failure (RoadFetchStatus::Failed) is reported to stderr - Absent is
    // the release's own coverage edge, not a problem, and is never logged.
    g2m::world::RoadSegmentsResult road_segments_shared(const g2m::TileKey& key,
                                                        g2m::world::Priority priority = g2m::world::Priority::Render);

    // Whether this release's manifest lists the g2m.src.osm layer at all
    // (G2.5a-grip R-b plan section 7 R-b) - read once, at open(), from
    // g2m::ReleaseManifest::find_layer. Distinct from a per-tile
    // RoadFetchStatus::Absent (a covered release's own coverage edge): a
    // release with no g2m.src.osm layer at all would report every tile
    // NotFound "unknown layer" or "level not in layer" instead (server.cpp's
    // check_tile), which road_segments_shared treats as Failed, not Absent -
    // a caller that wants to tell "no road layer in this release" apart from
    // "transient fetch failure" should check this first.
    [[nodiscard]] bool has_road_layer() const { return has_road_layer_; }

    // Diagnostics (R-4 owner-review follow-up, 2026-09-27): counts since
    // open(), across every calling thread (plain atomics, relaxed - counters
    // only, no other state depends on their ordering). `ok`/`fail` are
    // mutually exclusive per road_segments_shared() call that actually
    // reached the transport (a cache hit increments only `cache_hits`, not
    // `ok`). G2.5a-grip R-b: `ok` also counts RoadFetchStatus::Absent (a
    // release's own coverage edge - a normal, cacheable, non-failure answer,
    // never logged to stderr), not only RoadFetchStatus::Ok.
    struct OsmFetchStats {
        std::uint64_t ok = 0;          // road_segments_shared() calls that resolved Ok or Absent
        std::uint64_t fail = 0;        // road_segments_shared() calls that resolved Failed (see stderr)
        std::uint64_t cache_hits = 0;  // road_segments_shared()/road_segments() calls served from osm_cache_
    };
    [[nodiscard]] OsmFetchStats osm_fetch_stats() const;

private:
    WorldTerrain() = default;

    // The game's height products, run on the service's pool (terrain migration phase 2). The carved
    // L0 tile (raw DEM -> smoothing -> road patch) and the road surface of one L2 tile are caller-defined
    // products of g2m::world::TerrainService; until phase 7 the carve stays here.
    struct RoadSurface {
        std::shared_ptr<const g2m::RoadGeomTile> geometry; // completed profiles, what road_geometry_at serves
        std::shared_ptr<const RoadSurfacePatch> patch;
    };
    g2m::world::Outcome<g2m::HeightTile> produce_height(g2m::world::ProductContext& ctx, const g2m::TileKey& key);
    g2m::world::Outcome<RoadSurface> produce_road_surface(g2m::world::ProductContext& ctx, const g2m::TileKey& l2);
    // Joins (or runs) the road-surface product of the L2 tile owning `key` from inside a producer.
    g2m::world::Outcome<RoadSurface> join_road_surface(g2m::world::ProductContext& ctx, const g2m::TileKey& l2);
    // Blocking wait on a ticket from a non-pool thread; false on cancellation (service shutdown).
    template <class T>
    bool wait_ticket(g2m::world::Ticket<T>& ticket);
    // Blocks for the road surface of the L2 tile owning `key`; null when the fetch failed. Empty
    // geometry/patch for a tile the release does not cover.
    std::shared_ptr<const RoadSurface> road_surface(const g2m::TileKey& key, g2m::world::Priority priority);
    TerrainSmoothing smoothing_;
    double verge_drop_m_=0.06;
    RoadSeparation separation_;
    std::mutex decks_mutex_;
    std::map<std::tuple<std::int64_t,int,int>,std::shared_ptr<const RoadDeck>> published_decks_;
    std::mutex graph_mutex_;
    std::map<g2m::TileKey, std::shared_ptr<const g2m::RoadGraphTile>> graph_cache_;
    // Brief lock around the two maps only (the old geometry_mutex_ also serialised the derivation, which
    // the service's per-key dedup now does). The maps keep every published surface alive for the process,
    // like before, so an LRU eviction in the service can never make a 16 s derivation run twice.
    std::mutex geometry_cache_mutex_;
    std::map<g2m::TileKey,std::shared_ptr<const g2m::RoadGeomTile>> road_geometry_cache_;
    std::map<g2m::TileKey, std::shared_ptr<const RoadSurface>> geometry_cache_;

    std::unique_ptr<g2m::geo::SessionFrame> frame_;

    g2m::ReleaseId manifest_rid_{};
    std::string terrain_height_layer_;
    g2m::mesh::LodParams lod_params_;
    g2m::mesh::LodParams world_lod_params_;
    mutable std::mutex road_lod_mutex_;
    std::shared_ptr<const g2m::RenderRoadLod> road_lod_;

    std::mutex cache_mutex_;
    std::map<g2m::TileKey, std::shared_ptr<const g2m::HeightTile>> height_cache_;
    std::vector<std::shared_ptr<const g2m::HeightTile>> retired_tiles_; // replaced by heal_fallback_tiles, kept alive (cache_mutex_)
    mutable std::mutex stats_mutex_;
    FetchStats stats_;

    std::mutex osm_cache_mutex_;
    std::map<g2m::TileKey, g2m::world::RoadSegmentsResult> osm_cache_; // only Ok/Absent are ever inserted
    std::atomic<std::uint64_t> osm_ok_{0};
    std::atomic<std::uint64_t> osm_fail_{0};
    std::atomic<std::uint64_t> osm_cache_hits_{0};
    bool road_profiles_enabled_ = true;
    int carve_max_level_ = 2;
    mutable std::mutex carve_mutex_;
    std::map<g2m::TileKey, ChunkCarve> carve_; // per produced height tile
    std::map<g2m::TileKey, std::shared_ptr<const g2m::HeightTile>> fallback_tiles_; // the uncarved tiles of the Fallback keys
    bool has_road_layer_ = false; // set once, at open(), from the manifest
    RoadSurfaceMap road_surfaces_; // from config.physics.road_surfaces (G2.5a-grip R-b)
    std::atomic<std::uint8_t> physics_priority_{static_cast<std::uint8_t>(g2m::world::Priority::Physics)};
    g2m::world::ProductKind height_kind_ = 0;  // rg.height.carved
    g2m::world::ProductKind surface_kind_ = 0; // rg.road.surface

    // Declared LAST: members destruct in reverse order, and the service's pool runs producers that use
    // every member above, so it must be joined first.
    std::unique_ptr<g2m::world::TerrainService> service_;
};

} // namespace rg
