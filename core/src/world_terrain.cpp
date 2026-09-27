// rg/world_terrain.cpp — see rg/world_terrain.h for the public contract.
//
// Exception-free throughout (vault TOOL-019, same reasoning as
// world_config.cpp's own top comment): geo2map_engine's Result<T> is
// exception-free, and every g2m call here is checked via .ok()/.error()
// rather than caught. The one geo2map_engine API that CAN throw -
// g2m::geo::SessionFrame's constructor, on a non-multiple-of-1000 e0/n0 or an
// invalid zone - is guarded by an explicit check in WorldTerrain::open()
// before it is ever called, so this file never needs a try/catch.
#include "rg/world_terrain.h"

#include "g2m/layer/layer_id.h"
#include "g2m/layer/src_osm.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <span>
#include <thread>
#include <utility>

namespace rg {

namespace {

std::uint32_t pack_rgba(float r, float g, float b, float a) {
    auto to_u8 = [](float v) {
        return static_cast<std::uint8_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f);
    };
    return (static_cast<std::uint32_t>(to_u8(r)) << 24) | (static_cast<std::uint32_t>(to_u8(g)) << 16) |
           (static_cast<std::uint32_t>(to_u8(b)) << 8) | static_cast<std::uint32_t>(to_u8(a));
}

// Hypsometric ramp (PLAN.md R2.1: "valley green, ridge brown/grey"), blended
// toward a grey rock colour by slope, since no LandClass exists before G2.5
// (every ClassWindow is Unknown). Height bounds match physics_sim's own
// documented range for this project's home-region terrain (physics_sim's
// CLAUDE.md, "Terrain tile system": "real-world 90-880 m ASL terrain") - the
// Taunus ridge / Main valley home region spans roughly this range.
constexpr float kValleyHeightM = 90.0f;
constexpr float kRidgeHeightM = 880.0f;

// Road colours (R-2, roads_plan.md: "paved grey (0.30, 0.30, 0.32), unpaved
// brown (pick a sensible value and say which)"). Chosen: a mid dirt-track
// brown, (0.36, 0.28, 0.16) - readably distinct from both the paved grey
// above and the hypsometric ramp's own ridge-brown (0.45, 0.34, 0.22) below
// (darker and more saturated, so an unpaved road doesn't blend into bare
// rock/ridge terrain at a glance).
constexpr float kPavedR = 0.30f, kPavedG = 0.30f, kPavedB = 0.32f;
constexpr float kUnpavedR = 0.36f, kUnpavedG = 0.28f, kUnpavedB = 0.16f;

void chunk_vertex_colors(const g2m::mesh::TerrainChunkMesh& mesh, std::vector<std::uint32_t>& rgba) {
    const std::size_t vertex_count = mesh.positions.size() / 3;
    const bool has_land_class = mesh.land_class.size() == vertex_count;
    rgba.assign(vertex_count, 0);
    for (std::size_t i = 0; i < vertex_count; ++i) {
        if (has_land_class) {
            const auto land_class = static_cast<g2m::LandClass>(mesh.land_class[i]);
            if (land_class == g2m::LandClass::PavedRoad) {
                rgba[i] = pack_rgba(kPavedR, kPavedG, kPavedB, 1.0f);
                continue;
            }
            if (land_class == g2m::LandClass::UnpavedRoad) {
                rgba[i] = pack_rgba(kUnpavedR, kUnpavedG, kUnpavedB, 1.0f);
                continue;
            }
        }

        const float height_m = mesh.positions[3 * i + 2];
        const float nz = mesh.normals[3 * i + 2];

        const float t_height =
            std::clamp((height_m - kValleyHeightM) / (kRidgeHeightM - kValleyHeightM), 0.0f, 1.0f);
        constexpr float kValleyR = 0.20f, kValleyG = 0.42f, kValleyB = 0.18f; // valley green
        constexpr float kRidgeR = 0.45f, kRidgeG = 0.34f, kRidgeB = 0.22f;    // ridge brown
        float r = kValleyR + (kRidgeR - kValleyR) * t_height;
        float g = kValleyG + (kRidgeG - kValleyG) * t_height;
        float b = kValleyB + (kRidgeB - kValleyB) * t_height;

        // 0 = flat (normal straight up), 1 = vertical face.
        const float slope = std::clamp(1.0f - nz, 0.0f, 1.0f);
        // Blend to rock/grey above ~14 degrees, fully rock by ~45 degrees.
        const float rock = std::clamp((slope - 0.25f) / 0.5f, 0.0f, 1.0f);
        constexpr float kRockR = 0.42f, kRockG = 0.40f, kRockB = 0.38f;
        r += (kRockR - r) * rock;
        g += (kRockG - g) * rock;
        b += (kRockB - b) * rock;

        rgba[i] = pack_rgba(r, g, b, 1.0f);
    }
}

void build_one_chunk(const g2m::mesh::ChunkKey& key, g2m::mesh::TileLookup lookup, void* ctx, double e0, double n0,
                     const ClassLookup& class_lookup, RenderChunk& out_chunk) {
    g2m::mesh::HeightWindow window;
    g2m::mesh::gather_window(key, lookup, ctx, window);
    if (class_lookup.fn == nullptr) {
        // Byte-identical to the pre-R-2 code path (roads_plan.md R-2: "null
        // lookup = byte-identical output") - the literal same call, not a
        // zero-initialized ClassWindow passed by address.
        g2m::mesh::build_chunk(key, window, /*classes=*/nullptr, out_chunk.mesh);
    } else {
        g2m::mesh::ClassWindow classes;
        rasterize_chunk_road_classes(key, class_lookup, classes);
        g2m::mesh::build_chunk(key, window, &classes, out_chunk.mesh);
    }
    out_chunk.key = key;
    out_chunk.origin_session[0] = out_chunk.mesh.origin[0] - e0;
    out_chunk.origin_session[1] = out_chunk.mesh.origin[1] - n0;
    out_chunk.origin_session[2] = out_chunk.mesh.origin[2];
    chunk_vertex_colors(out_chunk.mesh, out_chunk.rgba);
}

const g2m::HeightTile* world_terrain_lookup(void* ctx, const g2m::TileKey& key) {
    return static_cast<WorldTerrain*>(ctx)->height_tile(key);
}

std::shared_ptr<const std::vector<g2m::RoadSegment>> world_terrain_road_class_lookup(void* ctx,
                                                                                      const g2m::TileKey& key) {
    return static_cast<WorldTerrain*>(ctx)->road_segments(key);
}

} // namespace

void select_view_keys(double cam_x, double cam_y, const g2m::mesh::LodParams& params, double e0, double n0,
                      std::vector<g2m::mesh::ChunkKey>& out) {
    g2m::mesh::select_chunks(cam_x + e0, cam_y + n0, params, out);
}

bool build_render_chunks(const std::vector<g2m::mesh::ChunkKey>& keys, g2m::mesh::TileLookup lookup, void* ctx,
                         double e0, double n0, unsigned thread_count, std::vector<RenderChunk>& out,
                         const std::atomic<bool>* cancel, const ClassLookup& class_lookup) {
    out.clear();
    out.resize(keys.size());
    if (keys.empty()) {
        return true;
    }
    auto cancelled = [cancel]() { return cancel != nullptr && cancel->load(std::memory_order_relaxed); };

    const unsigned n_threads = std::max(1u, std::min(thread_count, static_cast<unsigned>(keys.size())));
    if (n_threads <= 1) {
        for (std::size_t i = 0; i < keys.size(); ++i) {
            if (cancelled()) {
                return false;
            }
            build_one_chunk(keys[i], lookup, ctx, e0, n0, class_lookup, out[i]);
        }
        return !cancelled();
    }

    const std::size_t total = keys.size();
    const std::size_t per_thread = (total + n_threads - 1) / n_threads;
    std::vector<std::thread> workers;
    workers.reserve(n_threads);
    for (unsigned t = 0; t < n_threads; ++t) {
        const std::size_t begin = static_cast<std::size_t>(t) * per_thread;
        if (begin >= total) {
            break;
        }
        const std::size_t end = std::min(total, begin + per_thread);
        workers.emplace_back([&keys, &out, &cancelled, lookup, ctx, e0, n0, &class_lookup, begin, end]() {
            for (std::size_t i = begin; i < end; ++i) {
                if (cancelled()) {
                    return;
                }
                build_one_chunk(keys[i], lookup, ctx, e0, n0, class_lookup, out[i]);
            }
        });
    }
    for (std::thread& w : workers) {
        w.join();
    }
    return !cancelled();
}

void build_static_view_from_lookup(double cam_x, double cam_y, const g2m::mesh::LodParams& params, double e0,
                                   double n0, g2m::mesh::TileLookup lookup, void* ctx, unsigned thread_count,
                                   std::vector<RenderChunk>& out, const ClassLookup& class_lookup) {
    std::vector<g2m::mesh::ChunkKey> keys;
    select_view_keys(cam_x, cam_y, params, e0, n0, keys);
    build_render_chunks(keys, lookup, ctx, e0, n0, thread_count, out, /*cancel=*/nullptr, class_lookup);
}

WorldTerrain::~WorldTerrain() = default;

std::unique_ptr<WorldTerrain> WorldTerrain::open(const WorldConfig& config, std::string* err) {
    auto fail = [&](const std::string& msg) -> std::unique_ptr<WorldTerrain> {
        if (err != nullptr) {
            *err = msg;
        }
        return nullptr;
    };

    // Validate everything g2m::geo::SessionFrame's constructor would
    // otherwise throw std::invalid_argument for (this project's loaders are
    // exception-free throughout, vault TOOL-019 - see this file's own top
    // comment) BEFORE doing any store I/O.
    if (!g2m::geo::UtmZone{config.session_origin_utm.zone}.valid()) {
        return fail("WorldTerrain::open: session_origin_utm.zone " +
                     std::to_string(config.session_origin_utm.zone) + " is out of range (must be 1..60)");
    }
    const double e0 = config.session_origin_utm.e0;
    const double n0 = config.session_origin_utm.n0;
    const double e0_rounded = std::round(e0 / 1000.0) * 1000.0;
    const double n0_rounded = std::round(n0 / 1000.0) * 1000.0;
    if (std::abs(e0 - e0_rounded) > 1e-6 || std::abs(n0 - n0_rounded) > 1e-6) {
        return fail("WorldTerrain::open: session_origin_utm.e0/n0 must be multiples of 1000 m (got e0=" +
                     std::to_string(e0) + ", n0=" + std::to_string(n0) + ")");
    }

    std::unique_ptr<WorldTerrain> wt(new WorldTerrain());

    // --- source store (read-only: this project never writes into it) ---
    g2m::TileStoreConfig source_cfg;
    source_cfg.dir = config.source_store.dir;
    source_cfg.read_only = config.source_store.read_only;
    g2m::Result<std::unique_ptr<g2m::TileStore>> source_result = g2m::TileStore::open(source_cfg);
    if (!source_result.ok()) {
        return fail("WorldTerrain::open: source store \"" + config.source_store.dir +
                     "\": " + source_result.error().message);
    }
    wt->source_store_ = std::move(source_result.value());

    // --- derived store (read-write: Cache/Derive results land here) ---
    g2m::TileStoreConfig derived_cfg;
    derived_cfg.dir = config.derived_store.dir;
    if (!config.derived_store.name.empty()) {
        derived_cfg.name = config.derived_store.name;
    }
    g2m::Result<std::unique_ptr<g2m::TileStore>> derived_result = g2m::TileStore::open(derived_cfg);
    if (!derived_result.ok()) {
        return fail("WorldTerrain::open: derived store \"" + config.derived_store.dir +
                     "\": " + derived_result.error().message);
    }
    wt->derived_store_ = std::move(derived_result.value());

    // --- offline local release, mirroring geo2map_engine's own golden test
    // (tests/golden/golden_g2.cpp) and g2m_tiler's own `bake` wiring ---
    wt->local_upstream_.emplace(*wt->source_store_);

    std::vector<g2m::ReleaseSource> sources_attr = {
        g2m::ReleaseSource{"dgm1_hessen", "2026-09", "dl-de/zero-2-0",
                           "Hessisches Landesamt fuer Bodenmanagement und Geoinformation (HVBG), dl-de/zero-2-0",
                           false}};
    g2m::LocalReleaseParams params =
        g2m::builtin_local_release_params(config.source_store.scope, wt->derivers_, std::move(sources_attr));
    g2m::Result<g2m::ReleaseManifest> manifest_result = g2m::make_local_release(*wt->source_store_, params);
    if (!manifest_result.ok()) {
        return fail("WorldTerrain::open: make_local_release: " + manifest_result.error().message);
    }
    g2m::ReleaseManifest manifest = std::move(manifest_result.value());
    wt->manifest_rid_ = manifest.rid;

    g2m::ServerConfig server_cfg;
    server_cfg.server_id = "racing_game.terrain";
    server_cfg.offline = true; // the only IUpstream is a LocalSourceUpstream (needs_network() == false)
    server_cfg.derived_store = wt->derived_store_.get();
    server_cfg.derivers = wt->derivers_.ordered();
    server_cfg.upstreams = {&*wt->local_upstream_};
    wt->server_ = std::make_unique<g2m::Server>(server_cfg);

    g2m::Result<void> add_result = wt->server_->add_release(manifest, config.source_store.scope);
    if (!add_result.ok()) {
        return fail("WorldTerrain::open: add_release: " + add_result.error().message);
    }

    wt->transport_.emplace(*wt->server_);

    wt->frame_ = std::make_unique<g2m::geo::SessionFrame>(g2m::geo::UtmZone{config.session_origin_utm.zone},
                                                          static_cast<std::int64_t>(e0_rounded),
                                                          static_cast<std::int64_t>(n0_rounded));

    wt->terrain_height_layer_ = std::string(g2m::kTerrainHeightLayer);
    wt->lod_params_.zone = wt->frame_->zone();
    // max_distance_m comes from config (WorldConfig::Lod, PLAN.md R2.1: "make
    // max_distance a config value ... not a hard-coded constant") - defaults
    // to LodParams's own 6000.0 when world_config.json has no "lod" section.
    // range0_m/max_level keep LodParams's own defaults (not yet
    // config-exposed - only max_distance_m was swept/measured for R2.1).
    // WorldTerrain::set_lod_params still overrides all of it if a caller
    // wants to.
    wt->lod_params_.max_distance_m = config.lod.max_distance_m;

    return wt;
}

WorldTerrain::FetchDecodeResult WorldTerrain::fetch_and_decode(const g2m::TileKey& key) {
    g2m::TileRequest request{manifest_rid_, terrain_height_layer_, key, std::nullopt};
    g2m::Response response = transport_->send(g2m::Request{request});

    const auto* tile_response = std::get_if<g2m::TileResponse>(&response);
    const g2m::Status response_status = tile_response != nullptr ? tile_response->meta.status : g2m::Status::Internal;
    {
        std::lock_guard<std::mutex> lk(stats_mutex_);
        if (tile_response != nullptr && tile_response->meta.status == g2m::Status::Ok) {
            ++stats_.server_ok;
        } else {
            ++stats_.server_miss;
        }
    }
    if (tile_response == nullptr || tile_response->meta.status != g2m::Status::Ok) {
        return FetchDecodeResult{response_status, nullptr};
    }

    HeightTileFetchResult decoded = decode_height_tile_container(tile_response->container, terrain_height_layer_, key);
    return FetchDecodeResult{decoded.status, std::move(decoded.tile)};
}

HeightTileFetchResult decode_height_tile_container(std::span<const std::uint8_t> container,
                                                   std::string_view expect_layer, const g2m::TileKey& expect_key) {
    const g2m::Result<g2m::ContainerView> view = g2m::parse_container(container);
    if (!view.ok()) {
        return HeightTileFetchResult{g2m::Status::Internal, nullptr};
    }
    if (view.value().header.layer != expect_layer || !(view.value().header.key == expect_key)) {
        return HeightTileFetchResult{g2m::Status::Internal, nullptr};
    }
    const g2m::Result<g2m::TileBody> body = g2m::decode_body(view.value().body);
    if (!body.ok()) {
        return HeightTileFetchResult{g2m::Status::Internal, nullptr};
    }

    // Heap, never the stack (vault TOOL-039: g2m::HeightTile is 256 KiB).
    auto tile = std::make_shared<g2m::HeightTile>();
    if (!g2m::decode_height_tile(body.value(), expect_key, *tile).ok()) {
        return HeightTileFetchResult{g2m::Status::Internal, nullptr};
    }

    // Same arithmetic as geo2map's TransportHeightTileFetch: NoData stays
    // NoData, everything else gets the offset in int64, and a sum that would
    // collide with NoData or overflow int32 rejects the tile.
    const std::int32_t offset = view.value().header.height_offset;
    if (offset != 0) {
        for (std::int32_t& h : tile->h) {
            if (h == g2m::kHeightNoData) {
                continue;
            }
            const std::int64_t v = static_cast<std::int64_t>(h) + offset;
            if (v <= static_cast<std::int64_t>(g2m::kHeightNoData) ||
                v > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
                return HeightTileFetchResult{g2m::Status::Internal, nullptr};
            }
            h = static_cast<std::int32_t>(v);
        }
    }
    return HeightTileFetchResult{g2m::Status::Ok, std::move(tile)};
}

HeightTileFetchResult fetch_height_tile_cached(std::mutex& cache_mutex,
                                               std::map<g2m::TileKey, std::shared_ptr<const g2m::HeightTile>>& cache,
                                               const g2m::TileKey& key, const HeightTileFetchFn& fetch_fn,
                                               bool* was_cache_hit) {
    if (was_cache_hit != nullptr) {
        *was_cache_hit = false;
    }
    {
        std::lock_guard<std::mutex> lk(cache_mutex);
        auto it = cache.find(key);
        if (it != cache.end()) {
            if (was_cache_hit != nullptr) {
                *was_cache_hit = true;
            }
            return HeightTileFetchResult{g2m::Status::Ok, it->second};
        }
    }

    // Deliberately outside the lock: two threads racing to fetch the same
    // missing tile just both call fetch_fn (see this function's own doc
    // comment in world_terrain.h for why that is safe/bounded).
    HeightTileFetchResult fetched = fetch_fn(key);
    if (fetched.status != g2m::Status::Ok || !fetched.tile) {
        return fetched; // not cached: let a transient failure be retried later
    }

    std::lock_guard<std::mutex> lk(cache_mutex);
    auto [it, inserted] = cache.emplace(key, std::move(fetched.tile));
    (void)inserted; // first inserted value wins; a later racer's own fetch is discarded here
    return HeightTileFetchResult{g2m::Status::Ok, it->second};
}

HeightTileFetchResult WorldTerrain::height_tile_shared(const g2m::TileKey& key) {
    HeightTileFetchFn fetch_fn = [this](const g2m::TileKey& k) -> HeightTileFetchResult {
        FetchDecodeResult result = fetch_and_decode(k);
        if (!result.tile) {
            return HeightTileFetchResult{result.status, nullptr};
        }
        return HeightTileFetchResult{g2m::Status::Ok, std::move(result.tile)};
    };

    bool was_cache_hit = false;
    HeightTileFetchResult out = fetch_height_tile_cached(cache_mutex_, height_cache_, key, fetch_fn, &was_cache_hit);
    if (was_cache_hit) {
        std::lock_guard<std::mutex> lk(stats_mutex_);
        ++stats_.cache_hits;
    }
    return out;
}

const g2m::HeightTile* WorldTerrain::height_tile(const g2m::TileKey& key) {
    HeightTileFetchResult result = height_tile_shared(key);
    return result.tile.get();
}

void WorldTerrain::build_static_view(double cam_x, double cam_y, std::vector<RenderChunk>& out) {
    const unsigned hw = std::thread::hardware_concurrency();
    const unsigned threads = hw == 0 ? 4u : std::min(hw, 8u);
    build_static_view_from_lookup(cam_x, cam_y, lod_params_, static_cast<double>(frame_->e0_m()),
                                  static_cast<double>(frame_->n0_m()), &world_terrain_lookup, this, threads, out,
                                  ClassLookup{&world_terrain_road_class_lookup, this});
}

TerrainViewSource WorldTerrain::view_source() {
    TerrainViewSource source;
    source.params = lod_params_;
    source.e0 = static_cast<double>(frame_->e0_m());
    source.n0 = static_cast<double>(frame_->n0_m());
    source.lookup = &world_terrain_lookup;
    source.ctx = this;
    source.class_lookup = ClassLookup{&world_terrain_road_class_lookup, this};
    return source;
}

std::shared_ptr<const std::vector<g2m::RoadSegment>> WorldTerrain::road_segments(const g2m::TileKey& key) {
    {
        std::lock_guard<std::mutex> lk(osm_cache_mutex_);
        auto it = osm_cache_.find(key);
        if (it != osm_cache_.end()) {
            osm_cache_hits_.fetch_add(1, std::memory_order_relaxed);
            return it->second;
        }
    }

    auto fail = [&](const char* what) -> std::shared_ptr<const std::vector<g2m::RoadSegment>> {
        osm_fail_.fetch_add(1, std::memory_order_relaxed);
        std::fprintf(stderr, "WorldTerrain::road_segments: %s (tile %s) - reporting no roads for this tile\n", what,
                     g2m::to_string(key).c_str());
        return std::make_shared<const std::vector<g2m::RoadSegment>>();
    };

    g2m::TileRequest request{manifest_rid_, std::string(g2m::kSrcOsmLayer), key, std::nullopt};
    g2m::Response response = transport_->send(g2m::Request{request});
    const auto* tile_response = std::get_if<g2m::TileResponse>(&response);
    if (tile_response == nullptr || tile_response->meta.status != g2m::Status::Ok) {
        return fail("g2m.src.osm fetch failed");
    }

    const g2m::Result<g2m::ContainerView> view = g2m::parse_container(tile_response->container);
    if (!view.ok()) {
        return fail("g2m.src.osm container parse failed");
    }
    if (view.value().header.layer != g2m::kSrcOsmLayer || !(view.value().header.key == key)) {
        return fail("g2m.src.osm container header mismatch");
    }
    const g2m::Result<g2m::TileBody> body = g2m::decode_body(view.value().body);
    if (!body.ok()) {
        return fail("g2m.src.osm body decode failed");
    }
    const g2m::Result<g2m::OsmTile> osm_tile = g2m::decode_src_osm(body.value());
    if (!osm_tile.ok()) {
        return fail("g2m.src.osm decode_src_osm failed");
    }

    std::vector<g2m::RoadSegment> segments =
        g2m::extract_road_segments(osm_tile.value().data, frame_->zone(), g2m::RoadStyle::default_style());
    auto shared_segments = std::make_shared<const std::vector<g2m::RoadSegment>>(std::move(segments));

    osm_ok_.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lk(osm_cache_mutex_);
    auto [it, inserted] = osm_cache_.emplace(key, std::move(shared_segments));
    (void)inserted; // first inserted value wins, same policy as height_cache_
    return it->second;
}

WorldTerrain::FetchStats WorldTerrain::fetch_stats() const {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    return stats_;
}

WorldTerrain::OsmFetchStats WorldTerrain::osm_fetch_stats() const {
    return OsmFetchStats{osm_ok_.load(std::memory_order_relaxed), osm_fail_.load(std::memory_order_relaxed),
                          osm_cache_hits_.load(std::memory_order_relaxed)};
}

} // namespace rg
