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

#include <algorithm>
#include <cmath>
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

void chunk_vertex_colors(const g2m::mesh::TerrainChunkMesh& mesh, std::vector<std::uint32_t>& rgba) {
    const std::size_t vertex_count = mesh.positions.size() / 3;
    rgba.assign(vertex_count, 0);
    for (std::size_t i = 0; i < vertex_count; ++i) {
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
                     RenderChunk& out_chunk) {
    g2m::mesh::HeightWindow window;
    g2m::mesh::gather_window(key, lookup, ctx, window);
    g2m::mesh::build_chunk(key, window, /*classes=*/nullptr, out_chunk.mesh);
    out_chunk.origin_session[0] = out_chunk.mesh.origin[0] - e0;
    out_chunk.origin_session[1] = out_chunk.mesh.origin[1] - n0;
    out_chunk.origin_session[2] = out_chunk.mesh.origin[2];
    chunk_vertex_colors(out_chunk.mesh, out_chunk.rgba);
}

const g2m::HeightTile* world_terrain_lookup(void* ctx, const g2m::TileKey& key) {
    return static_cast<WorldTerrain*>(ctx)->height_tile(key);
}

} // namespace

void build_static_view_from_lookup(double cam_x, double cam_y, const g2m::mesh::LodParams& params, double e0,
                                   double n0, g2m::mesh::TileLookup lookup, void* ctx, unsigned thread_count,
                                   std::vector<RenderChunk>& out) {
    const double cam_e = cam_x + e0;
    const double cam_n = cam_y + n0;

    std::vector<g2m::mesh::ChunkKey> keys;
    g2m::mesh::select_chunks(cam_e, cam_n, params, keys);

    out.clear();
    out.resize(keys.size());
    if (keys.empty()) {
        return;
    }

    const unsigned n_threads = std::max(1u, std::min(thread_count, static_cast<unsigned>(keys.size())));
    if (n_threads <= 1) {
        for (std::size_t i = 0; i < keys.size(); ++i) {
            build_one_chunk(keys[i], lookup, ctx, e0, n0, out[i]);
        }
        return;
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
        workers.emplace_back([&keys, &out, lookup, ctx, e0, n0, begin, end]() {
            for (std::size_t i = begin; i < end; ++i) {
                build_one_chunk(keys[i], lookup, ctx, e0, n0, out[i]);
            }
        });
    }
    for (std::thread& w : workers) {
        w.join();
    }
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
        return FetchDecodeResult{response_status, std::nullopt};
    }

    std::size_t consumed = 0;
    g2m::Result<g2m::TileHeader> header_result = g2m::decode_header(tile_response->container, &consumed);
    if (!header_result.ok()) {
        return FetchDecodeResult{g2m::Status::Internal, std::nullopt};
    }
    const std::span<const std::uint8_t> body_bytes(tile_response->container.data() + consumed,
                                                    tile_response->container.size() - consumed);
    g2m::Result<g2m::TileBody> body_result = g2m::decode_body(body_bytes);
    if (!body_result.ok()) {
        return FetchDecodeResult{g2m::Status::Internal, std::nullopt};
    }

    g2m::HeightTile tile;
    g2m::Result<void> decode_result = g2m::decode_height_tile(body_result.value(), key, tile);
    if (!decode_result.ok()) {
        return FetchDecodeResult{g2m::Status::Internal, std::nullopt};
    }
    return FetchDecodeResult{g2m::Status::Ok, std::move(tile)};
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
        if (!result.tile.has_value()) {
            return HeightTileFetchResult{result.status, nullptr};
        }
        return HeightTileFetchResult{g2m::Status::Ok,
                                     std::make_shared<const g2m::HeightTile>(std::move(*result.tile))};
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
                                  static_cast<double>(frame_->n0_m()), &world_terrain_lookup, this, threads, out);
}

WorldTerrain::FetchStats WorldTerrain::fetch_stats() const {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    return stats_;
}

} // namespace rg
