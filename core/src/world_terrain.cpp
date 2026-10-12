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
#include "rg/route_check.h"

#include "g2m/layer/layer_id.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/road_dem_dependencies.h"
#include "g2m/world/chunk_road_classes.h"
#include "g2m/world/height_sampler.h"
#include "g2m/world/tile_fetch.h"

#include <algorithm>
#include <chrono>
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
//
// G2.5a-grip R-b: these two colours are now keyed by the RESOLVED SURFACE
// NAME (via `surfaces`, an rg::RoadSurfaceMap), not directly by LandClass -
// "asphalt" gets the paved colour, "dirt" the unpaved one, and any other
// configured name (a future data/surfaces/surfaces.json addition this map
// has not yet been taught to render specially) falls through to the height
// ramp below, same as an unpaved/paved-but-unrecognised name always did.
// RoadSurfaceMap's own defaults ("asphalt"/"dirt") mean a WorldConfig with no
// "physics.road_surfaces" block still renders these exact bytes.
constexpr float kPavedR = 0.30f, kPavedG = 0.30f, kPavedB = 0.32f;
constexpr float kUnpavedR = 0.36f, kUnpavedG = 0.28f, kUnpavedB = 0.16f;
constexpr std::string_view kAsphaltName = "asphalt";
constexpr std::string_view kDirtName = "dirt";

void chunk_vertex_colors(const g2m::mesh::TerrainChunkMesh& mesh, const RoadSurfaceMap& surfaces,
                         std::vector<std::uint32_t>& rgba) {
    const std::size_t vertex_count = mesh.positions.size() / 3;
    const bool has_land_class = mesh.land_class.size() == vertex_count;
    rgba.assign(vertex_count, 0);
    for (std::size_t i = 0; i < vertex_count; ++i) {
        if (has_land_class) {
            const auto land_class = static_cast<g2m::LandClass>(mesh.land_class[i]);
            if (land_class == g2m::LandClass::PavedRoad || land_class == g2m::LandClass::UnpavedRoad) {
                const std::string& name = surfaces.name_for(land_class);
                if (name == kAsphaltName) {
                    rgba[i] = pack_rgba(kPavedR, kPavedG, kPavedB, 1.0f);
                    continue;
                }
                if (name == kDirtName) {
                    rgba[i] = pack_rgba(kUnpavedR, kUnpavedG, kUnpavedB, 1.0f);
                    continue;
                }
                // An unrecognised name falls through to the height ramp
                // below - no colour table exists for an arbitrary surface
                // name yet.
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
                     const g2m::world::ClassLookup& class_lookup, const RoadSurfaceMap& surfaces, RenderChunk& out_chunk) {
    g2m::mesh::HeightWindow window;
    g2m::mesh::gather_window(key, lookup, ctx, window);
    if (class_lookup.fn == nullptr) {
        // Byte-identical to the pre-R-2 code path (roads_plan.md R-2: "null
        // lookup = byte-identical output") - the literal same call, not a
        // zero-initialized ClassWindow passed by address.
        g2m::mesh::build_chunk(key, window, /*classes=*/nullptr, out_chunk.mesh);
    } else {
        g2m::mesh::ClassWindow classes;
        g2m::world::rasterize_chunk_road_classes(key, class_lookup, classes);
        g2m::mesh::build_chunk(key, window, &classes, out_chunk.mesh);
    }
    out_chunk.key = key;
    out_chunk.origin_session[0] = out_chunk.mesh.origin[0] - e0;
    out_chunk.origin_session[1] = out_chunk.mesh.origin[1] - n0;
    out_chunk.origin_session[2] = out_chunk.mesh.origin[2];
    chunk_vertex_colors(out_chunk.mesh, surfaces, out_chunk.rgba);
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
                         const std::atomic<bool>* cancel, const g2m::world::ClassLookup& class_lookup,
                         const RoadSurfaceMap& surfaces) {
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
            build_one_chunk(keys[i], lookup, ctx, e0, n0, class_lookup, surfaces, out[i]);
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
        workers.emplace_back([&keys, &out, &cancelled, lookup, ctx, e0, n0, &class_lookup, &surfaces, begin, end]() {
            for (std::size_t i = begin; i < end; ++i) {
                if (cancelled()) {
                    return;
                }
                build_one_chunk(keys[i], lookup, ctx, e0, n0, class_lookup, surfaces, out[i]);
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
                                   std::vector<RenderChunk>& out, const g2m::world::ClassLookup& class_lookup,
                                   const RoadSurfaceMap& surfaces) {
    std::vector<g2m::mesh::ChunkKey> keys;
    select_view_keys(cam_x, cam_y, params, e0, n0, keys);
    build_render_chunks(keys, lookup, ctx, e0, n0, thread_count, out, /*cancel=*/nullptr, class_lookup, surfaces);
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
    wt->road_profiles_enabled_ = config.physics.road_surfaces.enabled;
    wt->smoothing_ = config.terrain_smoothing;
    wt->verge_drop_m_ = config.road_verge_drop_m;

    // --- the geo2map terrain service: stores, release, in-process server and ONE worker pool ---
    // (terrain migration plan phase 2). The layer opt-ins and the attribution are what the game's own
    // builtin_local_release_params call used before, so the release id is unchanged.
    g2m::world::TerrainServiceConfig service_cfg;
    service_cfg.stores.source_dir = config.source_store.dir;
    service_cfg.stores.source_scope = config.source_store.scope;
    service_cfg.stores.source_read_only = config.source_store.read_only;
    service_cfg.stores.derived_dir = config.derived_store.dir;
    if (!config.derived_store.name.empty()) {
        service_cfg.stores.derived_name = config.derived_store.name;
    }
    service_cfg.layers.roads = wt->road_profiles_enabled_;
    service_cfg.layers.geometry = wt->road_profiles_enabled_;
    service_cfg.zone = g2m::geo::UtmZone{config.session_origin_utm.zone};
    service_cfg.e0_m = static_cast<std::int64_t>(e0_rounded);
    service_cfg.n0_m = static_cast<std::int64_t>(n0_rounded);
    service_cfg.attribution = {
        g2m::ReleaseSource{"dgm1_hessen", "2026-09", "dl-de/zero-2-0",
                           "Hessisches Landesamt fuer Bodenmanagement und Geoinformation (HVBG), dl-de/zero-2-0",
                           false}};
    service_cfg.derive_workers = 0; // auto
    auto service_result = g2m::world::TerrainService::open(service_cfg);
    if (!service_result.ok()) {
        return fail("WorldTerrain::open: " + service_result.error().message);
    }
    wt->service_ = std::move(service_result.value());
    wt->manifest_rid_ = wt->service_->release_id();
    // G2.5a-grip R-b plan section 7: read once, here, rather than re-querying
    // the server per call - has_road_layer() is a plain bool return.
    wt->has_road_layer_ = wt->service_->has_layer(g2m::kSrcOsmLayer);
    wt->height_kind_ = g2m::world::TerrainService::register_product_kind("rg.height.carved");
    wt->surface_kind_ = g2m::world::TerrainService::register_product_kind("rg.road.surface");

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

    // G2.5a-grip R-b: render colour naming only (see chunk_vertex_colors'
    // own comment) - `enabled` is read by nothing yet (R-c wires physics).
    wt->road_surfaces_.paved = config.physics.road_surfaces.paved;
    wt->road_surfaces_.unpaved = config.physics.road_surfaces.unpaved;
    wt->road_surfaces_.off_road = config.physics.road_surfaces.off_road;

    return wt;
}

namespace {

using g2m::world::Outcome;
using g2m::world::Priority;
using g2m::world::ProductContext;
using g2m::world::TileState;

// A non-pool caller waits on its ticket in short slices so a service shutdown cannot hang it.
template <class T>
bool wait_for_ticket(g2m::world::Ticket<T>& ticket, const g2m::world::TerrainService& service) {
    while (!ticket.wait_for(std::chrono::milliseconds(50))) {
        if (service.stopping()) return false;
    }
    return true;
}

} // namespace

template <class T>
bool WorldTerrain::wait_ticket(g2m::world::Ticket<T>& ticket) {
    return wait_for_ticket(ticket, *service_);
}

Outcome<g2m::HeightTile> WorldTerrain::produce_height(ProductContext& ctx, const g2m::TileKey& key) {
    // Raw DEM tile of this key (g2m.terrain.height): joined if running, derived here otherwise.
    const Outcome<g2m::HeightTile> raw = ctx.height(key);
    {
        std::lock_guard<std::mutex> lk(stats_mutex_);
        if (raw.state == TileState::Ready) {
            ++stats_.server_ok;
        } else {
            ++stats_.server_miss;
        }
    }
    if (raw.state == TileState::Absent) return Outcome<g2m::HeightTile>::absent(raw.message);
    if (raw.state != TileState::Ready || !raw.value) {
        std::fprintf(stderr, "RG_TERRAIN_FETCH response key=%s message=%s\n", g2m::to_string(key).c_str(), raw.message.c_str());
        return Outcome<g2m::HeightTile>::failed(raw.message);
    }
    std::shared_ptr<const g2m::HeightTile> tile = raw.value;
    if (smoothing_.enabled) {
        tile = smooth_terrain(*tile, smoothing_, [&ctx](const g2m::TileKey& k) { return ctx.height(k).value; });
    }
    if (tile && road_profiles_enabled_ && has_road_layer_ && key.level == 0) {
        g2m::TileKey l2 = key;
        while (l2.level < 2) l2 = l2.parent();
        const Outcome<RoadSurface> surface = join_road_surface(ctx, l2);
        if (surface.state != TileState::Ready || !surface.value) {
            return Outcome<g2m::HeightTile>::failed("road surface " + g2m::to_string(l2) + ": " + surface.message);
        }
        auto carved = std::make_shared<g2m::HeightTile>(*tile);
        surface.value->patch->apply(*carved);
        tile = std::move(carved);
    }
    return Outcome<g2m::HeightTile>::ready(std::move(tile), sizeof(g2m::HeightTile));
}

Outcome<WorldTerrain::RoadSurface> WorldTerrain::join_road_surface(ProductContext& ctx, const g2m::TileKey& l2) {
    // A nested product: joined if another worker is deriving it, run inline otherwise (never waits for
    // queued work), so the 25-odd L0 tiles of one L2 tile derive their roads once.
    g2m::world::detail::ErasedOutcome e = ctx.service().join_erased(
        {surface_kind_, l2.packed(), 0}, typeid(RoadSurface), [this, &ctx, l2]() {
            return g2m::world::detail::erase<RoadSurface>(produce_road_surface(ctx, l2));
        });
    Outcome<RoadSurface> out;
    out.state = e.state;
    out.value = std::static_pointer_cast<const RoadSurface>(e.value);
    out.message = std::move(e.message);
    out.bytes = e.bytes;
    return out;
}

std::shared_ptr<const WorldTerrain::RoadSurface> WorldTerrain::road_surface(const g2m::TileKey& key, Priority priority) {
    g2m::TileKey l2 = key;
    while (l2.level < 2) l2 = l2.parent();
    {
        std::lock_guard<std::mutex> lock(geometry_cache_mutex_);
        if (auto it = geometry_cache_.find(l2); it != geometry_cache_.end()) return it->second;
    }
    auto ticket = service_->product<RoadSurface>(
        surface_kind_, l2, priority, 0,
        [this](ProductContext& ctx, const g2m::TileKey& k) { return produce_road_surface(ctx, k); });
    if (!wait_ticket(ticket)) return nullptr;
    return ticket.value(); // null on Failed/Cancelled (produce_road_surface never reports Absent)
}

Outcome<WorldTerrain::RoadSurface> WorldTerrain::produce_road_surface(ProductContext& ctx, const g2m::TileKey& key) {
    {
        std::lock_guard<std::mutex> lock(geometry_cache_mutex_);
        if (auto it = geometry_cache_.find(key); it != geometry_cache_.end()) {
            return Outcome<RoadSurface>::ready(it->second, 4096);
        }
    }
    const auto publish_empty = [&]() {
        auto empty = std::make_shared<RoadSurface>();
        empty->patch = std::make_shared<RoadSurfacePatch>(g2m::RoadGeomTile{});
        std::lock_guard<std::mutex> lock(geometry_cache_mutex_);
        const auto it = geometry_cache_.emplace(key, std::move(empty)).first;
        return Outcome<RoadSurface>::ready(it->second, 4096);
    };

    const Outcome<g2m::RoadGeomTile> fetched = ctx.road_geom(key);
    if (fetched.state == TileState::Absent) {
        // Outside coverage, or "required dependency absent": complete-way profile fitting can reach outside
        // the local DEM. Missing optional road geometry must not discard valid terrain.
        if (fetched.message != "outside coverage") {
            std::fprintf(stderr, "RG_ROAD_SURFACE unavailable key=%s reason=required_dependency_absent fallback=smoothed_terrain\n", g2m::to_string(key).c_str());
        }
        return publish_empty();
    }
    if (fetched.state != TileState::Ready || !fetched.value) {
        if (fetched.message.find("deriver g2m.roads.geom: roads.geom: ") != std::string::npos) {
            // The deriver's own validation rejected this tile (deterministic, e.g. utm32n/2/452/5428
            // "reference endpoints" near Koenigstein, 2026-10-06). Retrying cannot succeed and blocked the
            // start-up gate for 30 s; degrade to smoothed terrain like a missing dependency.
            std::fprintf(stderr, "RG_ROAD_SURFACE unavailable key=%s reason=deriver_rejected message=%s fallback=smoothed_terrain\n", g2m::to_string(key).c_str(), fetched.message.c_str());
            return publish_empty();
        }
        std::fprintf(stderr, "RG_ROAD_SURFACE fetch_failed key=%s message=%s\n", g2m::to_string(key).c_str(), fetched.message.c_str());
        return Outcome<RoadSurface>::failed(fetched.message);
    }
    auto geometry = std::make_shared<g2m::RoadGeomTile>(*fetched.value); // completion below edits the profiles

    // The library's five-metre fit can exhaust its bounded projection solver
    // on kilometre-long noisy DEM stretches. Retry explicit numerical declines
    // at twenty metres with the SAME certified grade/curvature constraints.
    auto builder = g2m::RoadReferenceBuilder::make(geometry->source.graph);
    auto dependencies = g2m::road_dem_dependencies(geometry->source.graph, key.zone);
    std::vector<std::shared_ptr<const g2m::HeightTile>> dem_storage;
    std::vector<const g2m::HeightTile*> dem_tiles;
    if (builder.ok() && dependencies.ok()) {
        bool complete = true;
        for (const auto& dependency : *dependencies) {
            const Outcome<g2m::world::RawTile> raw = ctx.raw_tile(dependency.layer, dependency.key);
            if (raw.state != TileState::Ready || !raw.value) { complete = false; break; }
            auto decoded = g2m::world::decode_height_tile_container(raw.value->container, dependency.layer, dependency.key);
            if (!decoded.tile) { complete = false; break; }
            dem_tiles.push_back(decoded.tile.get());
            dem_storage.push_back(std::move(decoded.tile));
        }
        if (complete) {
            auto dem = g2m::RoadDemSampler::make(dem_tiles);
            if (dem.ok()) complete_road_profiles(*geometry, *dem);
            if (dem.ok()) for (auto& entry : geometry->entries) {
                if (entry.profile || !entry.decline) continue;
                if (entry.decline->message.find("bounded solver") == std::string::npos &&
                    entry.decline->message.find("endpoint curvature") == std::string::npos) continue;
                g2m::RoadProfileLimits limits; limits.sample_spacing_m = 20; limits.max_iterations = 16384;
                limits.reference.allow_curvature_exceptions = true;
                auto profile = g2m::build_road_profile(*builder, *dem, key.zone, entry.way_id, entry.stretch, limits);
                if (profile.ok()) {
                    std::fprintf(stderr, "RG_ROAD_SURFACE retry_accepted way=%lld refs=%u:%u\n", static_cast<long long>(entry.way_id), entry.stretch.start_ref, entry.stretch.end_ref);
                    entry.profile = std::move(*profile); entry.decline.reset();
                }
            }
        }
    }
    auto surface = std::make_shared<RoadSurface>();
    surface->geometry = std::make_shared<const g2m::RoadGeomTile>(*geometry);
    surface->patch = std::make_shared<RoadSurfacePatch>(*geometry, verge_drop_m_);
    std::fprintf(stderr, "RG_ROAD_SURFACE key=%s accepted=%zu declined=%zu separated=%zu\n", g2m::to_string(key).c_str(), surface->patch->accepted, surface->patch->declined, surface->patch->separated);
    {
        std::lock_guard<std::mutex> lock(decks_mutex_);
        for (const auto& d : surface->patch->decks) published_decks_.try_emplace(std::tuple{d->way_id, static_cast<int>(std::round(d->start_station)), static_cast<int>(std::round(d->end_station))}, d);
    }
    std::lock_guard<std::mutex> lock(geometry_cache_mutex_);
    road_geometry_cache_.emplace(key, surface->geometry);
    const auto it = geometry_cache_.emplace(key, std::move(surface)).first;
    return Outcome<RoadSurface>::ready(it->second, 1 << 20);
}

std::shared_ptr<const g2m::RoadGraphTile> WorldTerrain::road_graph_tile(const g2m::TileKey& key) {
    {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        if (auto it = graph_cache_.find(key); it != graph_cache_.end()) return it->second;
    }
    auto ticket = service_->raw_tile(std::string(g2m::kRoadGraphLayer), key, g2m::world::Priority::Render);
    if (!wait_ticket(ticket)) return nullptr;
    std::shared_ptr<const g2m::RoadGraphTile> result;
    if (ticket.state() == g2m::world::TileState::Absent) {
        // The release has no such tile (outside coverage): an empty graph, like road_surface.
        auto empty = std::make_shared<g2m::RoadGraphTile>();
        empty->key = key;
        result = std::move(empty);
    } else if (ticket.state() == g2m::world::TileState::Ready) {
        const auto raw = ticket.value();
        auto container = g2m::parse_container(raw->container);
        if (!container.ok() || container.value().header.key != key || container.value().header.layer != g2m::kRoadGraphLayer) {
            std::fprintf(stderr, "RG_ROAD_GRAPH invalid_container key=%s\n", g2m::to_string(key).c_str());
            return nullptr;
        }
        auto body = g2m::decode_body(container.value().body);
        if (!body.ok()) return nullptr;
        auto graph = g2m::decode_road_graph(body.value());
        if (!graph.ok()) {
            std::fprintf(stderr, "RG_ROAD_GRAPH invalid_graph key=%s message=%s\n", g2m::to_string(key).c_str(), graph.error().message.c_str());
            return nullptr;
        }
        result = std::make_shared<g2m::RoadGraphTile>(std::move(graph.value()));
    } else {
        std::fprintf(stderr, "RG_ROAD_GRAPH fetch_failed key=%s message=%s\n", g2m::to_string(key).c_str(), ticket.message().c_str());
        return nullptr;
    }
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return graph_cache_.emplace(key, std::move(result)).first->second;
}

std::vector<std::shared_ptr<const RoadDeck>> WorldTerrain::road_decks() {
    // Publication has its own brief lock: a physics tick must never wait for
    // seconds of background DEM/profile derivation on the service pool.
    std::lock_guard<std::mutex> lock(decks_mutex_);
    std::vector<std::shared_ptr<const RoadDeck>> result;
    for(const auto& [id,d]:published_decks_) {(void)id;result.push_back(d);}
    return result;
}

std::shared_ptr<const g2m::RoadGeomTile> WorldTerrain::cached_road_geometry_at(double easting,double northing) {
    auto key=g2m::tile_key_at(frame_->zone(),2,easting,northing);if(!key)return nullptr;
    // Opportunistic traffic: only what has been derived already, never a wait for a running derivation
    // (the cache lock is brief now - the derivation runs on the service pool).
    std::lock_guard<std::mutex> lock(geometry_cache_mutex_);
    auto it=road_geometry_cache_.find(*key);return it==road_geometry_cache_.end()?nullptr:it->second;
}

std::shared_ptr<const g2m::RoadGeomTile> WorldTerrain::road_geometry_at(double easting,double northing) {
    auto key=g2m::tile_key_at(frame_->zone(),2,easting,northing);if(!key) return nullptr;
    const auto surface=road_surface(*key,g2m::world::Priority::Render);
    return surface?surface->geometry:nullptr; // null geometry: no road tile there (uncovered / rejected)
}

RoadVisualMesh WorldTerrain::road_visual_tile(const g2m::TileKey& key,double lift) {
    const auto geometry=road_geometry_at(key.min_easting()+512,key.min_northing()+512);
    if(!geometry)return {};
    return build_road_visual(*geometry,key,[this,key](double e,double n){return g2m::world::sample_height_bilinear([this](const auto& k){return height_tile_shared(k).tile.get();},key.zone,0,0,e,n);},lift);
}

std::shared_ptr<const g2m::OsmTile> WorldTerrain::source_osm_tile(const g2m::TileKey& key) {
    auto ticket=service_->source_osm(key,g2m::world::Priority::Render);
    if(!wait_ticket(ticket))return {};
    return ticket.value(); // null unless Ready
}

std::vector<Building> WorldTerrain::buildings_tile(const g2m::TileKey& key,double min_height,double fallback,double storey) {
    const auto osm=source_osm_tile(key);if(!osm)return {};
    // The raw DEM (not the carved one): buildings stand on the terrain, not on a road bed. The sampler hands
    // out raw pointers, so every tile it asks for is held here until the call ends.
    std::vector<std::shared_ptr<const g2m::HeightTile>> keep;
    return extract_buildings(*osm,key,min_height,fallback,storey,[this,key,&keep](double e,double n){
        return g2m::world::sample_height_bilinear([this,&keep](const auto& k)->const g2m::HeightTile*{
            auto ticket=service_->height(k,g2m::world::Priority::Render);
            if(!wait_ticket(ticket))return nullptr;
            auto tile=ticket.value();
            if(!tile)return nullptr;
            keep.push_back(std::move(tile));
            return keep.back().get();
        },key.zone,0,0,e,n);
    });
}

const g2m::HeightTile* WorldTerrain::cached_height_tile(const g2m::TileKey& key) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    const auto it=height_cache_.find(key);
    return it==height_cache_.end()?nullptr:it->second.get();
}

g2m::world::HeightTileFetchResult WorldTerrain::height_tile_shared(const g2m::TileKey& key, g2m::world::Priority priority) {
    g2m::world::HeightTileFetchFn fetch_fn = [this, priority](const g2m::TileKey& k) -> g2m::world::HeightTileFetchResult {
        // The carved tile is a service product: the raw DEM, the smoothing and the road patch run on the
        // service's pool, this caller (a physics loader or render worker) only waits on the ticket.
        auto ticket = service_->product<g2m::HeightTile>(
            height_kind_, k, priority, 0,
            [this](g2m::world::ProductContext& ctx, const g2m::TileKey& tk) { return produce_height(ctx, tk); });
        if (!wait_ticket(ticket)) return g2m::world::HeightTileFetchResult{g2m::Status::Internal, nullptr};
        switch (ticket.state()) {
            case g2m::world::TileState::Ready: {
                auto tile = ticket.value();
                if (tile) return g2m::world::HeightTileFetchResult{g2m::Status::Ok, std::move(tile)};
                return g2m::world::HeightTileFetchResult{g2m::Status::Internal, nullptr};
            }
            case g2m::world::TileState::Absent:
                return g2m::world::HeightTileFetchResult{g2m::Status::NotFound, nullptr};
            default:
                std::fprintf(stderr, "RG_TERRAIN_FETCH failed key=%s message=%s\n", g2m::to_string(k).c_str(), ticket.message().c_str());
                return g2m::world::HeightTileFetchResult{g2m::Status::Internal, nullptr};
        }
    };

    bool was_cache_hit = false;
    g2m::world::HeightTileFetchResult out = g2m::world::fetch_height_tile_cached(cache_mutex_, height_cache_, key, fetch_fn, &was_cache_hit);
    if (was_cache_hit) {
        std::lock_guard<std::mutex> lk(stats_mutex_);
        ++stats_.cache_hits;
    }
    return out;
}

const g2m::HeightTile* WorldTerrain::height_tile(const g2m::TileKey& key) {
    g2m::world::HeightTileFetchResult result = height_tile_shared(key);
    return result.tile.get();
}

void WorldTerrain::build_static_view(double cam_x, double cam_y, std::vector<RenderChunk>& out) {
    const unsigned hw = std::thread::hardware_concurrency();
    const unsigned threads = hw == 0 ? 4u : std::min(hw, 8u);
    build_static_view_from_lookup(cam_x, cam_y, lod_params_, static_cast<double>(frame_->e0_m()),
                                  static_cast<double>(frame_->n0_m()), &world_terrain_lookup, this, threads, out,
                                  g2m::world::ClassLookup{&world_terrain_road_class_lookup, this}, road_surfaces_);
}

TerrainViewSource WorldTerrain::view_source() {
    TerrainViewSource source;
    source.params = lod_params_;
    source.e0 = static_cast<double>(frame_->e0_m());
    source.n0 = static_cast<double>(frame_->n0_m());
    source.lookup = &world_terrain_lookup;
    source.ctx = this;
    source.class_lookup = g2m::world::ClassLookup{&world_terrain_road_class_lookup, this};
    source.surfaces = road_surfaces_;
    return source;
}

g2m::world::RoadSegmentsResult WorldTerrain::road_segments_shared(const g2m::TileKey& key, g2m::world::Priority priority) {
    g2m::world::RoadSegmentsFetchFn fetch_fn = [this, priority](const g2m::TileKey& k) -> g2m::world::RoadSegmentsResult {
        auto ticket = service_->road_segments(k, priority);
        if (!wait_ticket(ticket)) return g2m::world::RoadSegmentsResult{g2m::world::RoadFetchStatus::Failed, nullptr};
        switch (ticket.state()) {
            case g2m::world::TileState::Ready: {
                const auto tile = ticket.value();
                // Aliases the tile: the segment list lives as long as any holder of the result.
                return g2m::world::RoadSegmentsResult{g2m::world::RoadFetchStatus::Ok,
                    std::shared_ptr<const std::vector<g2m::RoadSegment>>(tile, &tile->segments)};
            }
            case g2m::world::TileState::Absent:
                return g2m::world::RoadSegmentsResult{g2m::world::RoadFetchStatus::Absent,
                    std::make_shared<const std::vector<g2m::RoadSegment>>()};
            default:
                std::fprintf(stderr,
                             "WorldTerrain::road_segments_shared: g2m.src.osm fetch/decode failed (tile %s) - "
                             "reporting no roads for this tile\n",
                             g2m::to_string(k).c_str());
                return g2m::world::RoadSegmentsResult{g2m::world::RoadFetchStatus::Failed, nullptr};
        }
    };

    bool was_cache_hit = false;
    g2m::world::RoadSegmentsResult out = g2m::world::fetch_road_segments_cached(osm_cache_mutex_, osm_cache_, key, fetch_fn, &was_cache_hit);
    if (was_cache_hit) {
        osm_cache_hits_.fetch_add(1, std::memory_order_relaxed);
    } else if (out.status == g2m::world::RoadFetchStatus::Failed) {
        osm_fail_.fetch_add(1, std::memory_order_relaxed);
    } else {
        osm_ok_.fetch_add(1, std::memory_order_relaxed);
    }
    return out;
}

std::shared_ptr<const std::vector<g2m::RoadSegment>> WorldTerrain::road_segments(const g2m::TileKey& key) {
    g2m::world::RoadSegmentsResult result = road_segments_shared(key);
    if (result.segments) return result.segments;
    static const auto kNoRoads = std::make_shared<const std::vector<g2m::RoadSegment>>();
    return kNoRoads; // Failed: "no roads" is a valid, non-fatal answer for the render path
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
