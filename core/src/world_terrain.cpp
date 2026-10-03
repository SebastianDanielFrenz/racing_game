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
                     const ClassLookup& class_lookup, const RoadSurfaceMap& surfaces, RenderChunk& out_chunk) {
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
                         const std::atomic<bool>* cancel, const ClassLookup& class_lookup,
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
                                   std::vector<RenderChunk>& out, const ClassLookup& class_lookup,
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
        g2m::builtin_local_release_params(config.source_store.scope, wt->derivers_, std::move(sources_attr), wt->road_profiles_enabled_, wt->road_profiles_enabled_);
    g2m::Result<g2m::ReleaseManifest> manifest_result = g2m::make_local_release(*wt->source_store_, params);
    if (!manifest_result.ok()) {
        return fail("WorldTerrain::open: make_local_release: " + manifest_result.error().message);
    }
    g2m::ReleaseManifest manifest = std::move(manifest_result.value());
    wt->manifest_rid_ = manifest.rid;
    // G2.5a-grip R-b plan section 7: read once, here, rather than re-querying
    // the server per call - has_road_layer() is a plain bool return.
    wt->has_road_layer_ = manifest.find_layer(g2m::kSrcOsmLayer) != nullptr;

    g2m::ServerConfig server_cfg;
    server_cfg.server_id = "racing_game.terrain";
    server_cfg.offline = true; // the only IUpstream is a LocalSourceUpstream (needs_network() == false)
    server_cfg.derived_store = wt->derived_store_.get();
    server_cfg.derivers = wt->derivers_.ordered(wt->road_profiles_enabled_, wt->road_profiles_enabled_);
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

    // G2.5a-grip R-b: render colour naming only (see chunk_vertex_colors'
    // own comment) - `enabled` is read by nothing yet (R-c wires physics).
    wt->road_surfaces_.paved = config.physics.road_surfaces.paved;
    wt->road_surfaces_.unpaved = config.physics.road_surfaces.unpaved;
    wt->road_surfaces_.off_road = config.physics.road_surfaces.off_road;

    return wt;
}

WorldTerrain::FetchDecodeResult WorldTerrain::fetch_raw_decode(const g2m::TileKey& key) {
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
        if(response_status != g2m::Status::NotFound) std::fprintf(stderr,"RG_TERRAIN_FETCH response key=%s message=%s\n",g2m::to_string(key).c_str(),tile_response?tile_response->meta.message.c_str():"not a tile response");
        return FetchDecodeResult{response_status, nullptr};
    }

    HeightTileFetchResult decoded = decode_height_tile_container(tile_response->container, terrain_height_layer_, key);
    if(decoded.status != g2m::Status::Ok) std::fprintf(stderr,"RG_TERRAIN_FETCH decode key=%s\n",g2m::to_string(key).c_str());
    return FetchDecodeResult{decoded.status,std::move(decoded.tile)};
}

HeightTileFetchResult WorldTerrain::raw_height_tile_shared(const g2m::TileKey& key) {
    return fetch_height_tile_cached(raw_cache_mutex_,raw_height_cache_,key,[this](const auto& k){
        auto result=fetch_raw_decode(k);return HeightTileFetchResult{result.status,std::move(result.tile)};
    });
}

WorldTerrain::FetchDecodeResult WorldTerrain::fetch_and_decode(const g2m::TileKey& key) {
    auto decoded=raw_height_tile_shared(key);
    if(decoded.tile&&smoothing_.enabled) decoded.tile=smooth_terrain(*decoded.tile,smoothing_,[this](const auto& k){return raw_height_tile_shared(k).tile;});
    if (decoded.tile && road_profiles_enabled_ && has_road_layer_ && key.level == 0) {
        auto geometry_key=key;
        while(geometry_key.level<2) geometry_key=geometry_key.parent();
        const auto patch=road_surface_patch(geometry_key);
        if (!patch) return FetchDecodeResult{g2m::Status::Internal,nullptr};
        auto carved=std::make_shared<g2m::HeightTile>(*decoded.tile);
        patch->apply(*carved);
        decoded.tile=std::move(carved);
    }
    return FetchDecodeResult{decoded.status, std::move(decoded.tile)};
}

std::shared_ptr<const RoadSurfacePatch> WorldTerrain::road_surface_patch(const g2m::TileKey& key) {
    // Serialise geometry derivation: expensive complete-way DEM dependencies
    // should be staged once, never once per racing height/render worker.
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    if (auto it=geometry_cache_.find(key);it!=geometry_cache_.end()) return it->second;
    auto response=transport_->send(g2m::Request{g2m::TileRequest{manifest_rid_,std::string(g2m::kRoadGeomLayer),key,std::nullopt}});
    auto* tile=std::get_if<g2m::TileResponse>(&response);
    if(!tile) return nullptr;
    if(tile->meta.status==g2m::Status::NotFound && tile->meta.message=="outside coverage") {
        auto empty=std::make_shared<RoadSurfacePatch>(g2m::RoadGeomTile{});
        geometry_cache_.emplace(key,empty); return empty;
    }
    if(tile->meta.status==g2m::Status::NotFound &&
       tile->meta.message=="deriver g2m.roads.geom: roads.geom: required dependency absent") {
        // Complete-way profile fitting can reach outside the local DEM.
        // Missing optional road geometry must not discard valid terrain.
        std::fprintf(stderr,"RG_ROAD_SURFACE unavailable key=%s reason=required_dependency_absent fallback=smoothed_terrain\n",g2m::to_string(key).c_str());
        auto empty=std::make_shared<RoadSurfacePatch>(g2m::RoadGeomTile{});
        geometry_cache_.emplace(key,empty);return empty;
    }
    if(tile->meta.status!=g2m::Status::Ok) {
        std::fprintf(stderr,"RG_ROAD_SURFACE fetch_failed key=%s message=%s\n",g2m::to_string(key).c_str(),tile->meta.message.c_str());
        return nullptr;
    }
    auto container=g2m::parse_container(tile->container);
    if(!container.ok() || container.value().header.key!=key || container.value().header.layer!=g2m::kRoadGeomLayer) {std::fprintf(stderr,"RG_ROAD_SURFACE invalid_container key=%s\n",g2m::to_string(key).c_str());return nullptr;}
    auto body=g2m::decode_body(container.value().body); if(!body.ok()) return nullptr;
    auto geometry=g2m::decode_road_geom(body.value()); if(!geometry.ok()) {std::fprintf(stderr,"RG_ROAD_SURFACE invalid_geometry key=%s message=%s\n",g2m::to_string(key).c_str(),geometry.error().message.c_str());return nullptr;}
    // The library's five-metre fit can exhaust its bounded projection solver
    // on kilometre-long noisy DEM stretches. Retry explicit numerical declines
    // at twenty metres with the SAME certified grade/curvature constraints.
    auto builder=g2m::RoadReferenceBuilder::make(geometry->source.graph);
    auto dependencies=g2m::road_dem_dependencies(geometry->source.graph,key.zone);
    std::vector<std::shared_ptr<const g2m::HeightTile>> dem_storage;
    std::vector<const g2m::HeightTile*> dem_tiles;
    if(builder.ok() && dependencies.ok()) {
        bool complete=true;
        for(const auto& dependency:*dependencies) {
            auto res=transport_->send(g2m::Request{g2m::TileRequest{manifest_rid_,dependency.layer,dependency.key,std::nullopt}});
            const auto* tr=std::get_if<g2m::TileResponse>(&res);
            if(!tr || tr->meta.status!=g2m::Status::Ok) {complete=false;break;}
            auto decoded=decode_height_tile_container(tr->container,dependency.layer,dependency.key);
            if(!decoded.tile) {complete=false;break;}
            dem_tiles.push_back(decoded.tile.get()); dem_storage.push_back(std::move(decoded.tile));
        }
        if(complete) {
            auto dem=g2m::RoadDemSampler::make(dem_tiles);
            if(dem.ok()) complete_road_profiles(*geometry,*dem);
            if(dem.ok()) for(auto& entry:geometry->entries) {
                if(entry.profile || !entry.decline) continue;
                if(entry.decline->message.find("bounded solver")==std::string::npos &&
                   entry.decline->message.find("endpoint curvature")==std::string::npos) continue;
                g2m::RoadProfileLimits limits; limits.sample_spacing_m=20;limits.max_iterations=16384;
                limits.reference.allow_curvature_exceptions=true;
                auto profile=g2m::build_road_profile(*builder,*dem,key.zone,entry.way_id,entry.stretch,limits);
                if(profile.ok()) {
                    std::fprintf(stderr,"RG_ROAD_SURFACE retry_accepted way=%lld refs=%u:%u\n",static_cast<long long>(entry.way_id),entry.stretch.start_ref,entry.stretch.end_ref);
                    entry.profile=std::move(*profile);entry.decline.reset();
                }
            }
        }
    }
    road_geometry_cache_.emplace(key,std::make_shared<g2m::RoadGeomTile>(*geometry));
    auto patch=std::make_shared<RoadSurfacePatch>(geometry.value(),verge_drop_m_);
    std::fprintf(stderr,"RG_ROAD_SURFACE key=%s accepted=%zu declined=%zu separated=%zu\n",g2m::to_string(key).c_str(),patch->accepted,patch->declined,patch->separated);
    {
        std::lock_guard<std::mutex> lock(decks_mutex_);
        for(const auto& d:patch->decks) published_decks_.try_emplace(std::tuple{d->way_id,static_cast<int>(std::round(d->start_station)),static_cast<int>(std::round(d->end_station))},d);
    }
    geometry_cache_.emplace(key,patch); return patch;
}

std::vector<std::shared_ptr<const RoadDeck>> WorldTerrain::road_decks() {
    // Publication has its own brief lock: a physics tick must never wait for
    // seconds of background DEM/profile derivation under geometry_mutex_.
    std::lock_guard<std::mutex> lock(decks_mutex_);
    std::vector<std::shared_ptr<const RoadDeck>> result;
    for(const auto& [id,d]:published_decks_) {(void)id;result.push_back(d);}
    return result;
}

std::shared_ptr<const g2m::RoadGeomTile> WorldTerrain::road_geometry_at(double easting,double northing) {
    auto key=g2m::tile_key_at(frame_->zone(),2,easting,northing);if(!key) return nullptr;
    road_surface_patch(*key);
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    auto it=road_geometry_cache_.find(*key);return it==road_geometry_cache_.end()?nullptr:it->second;
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

RoadVisualMesh WorldTerrain::road_visual_tile(const g2m::TileKey& key,double lift) {
    const auto geometry=road_geometry_at(key.min_easting()+512,key.min_northing()+512);
    if(!geometry)return {};
    return build_road_visual(*geometry,key,[this,key](double e,double n){return sample_l0_height([this](const auto& k){return height_tile_shared(k).tile.get();},key.zone,0,0,e,n);},lift);
}

std::shared_ptr<const g2m::OsmTile> WorldTerrain::source_osm_tile(const g2m::TileKey& key) {
    auto response=transport_->send(g2m::Request{g2m::TileRequest{manifest_rid_,std::string(g2m::kSrcOsmLayer),key,std::nullopt}});
    const auto* tile=std::get_if<g2m::TileResponse>(&response);if(!tile||tile->meta.status!=g2m::Status::Ok)return {};
    auto container=g2m::parse_container(tile->container);if(!container||container->header.key!=key||container->header.layer!=g2m::kSrcOsmLayer)return {};
    auto body=g2m::decode_body(container->body);if(!body)return {};auto osm=g2m::decode_src_osm(*body);if(!osm)return {};
    return std::make_shared<const g2m::OsmTile>(std::move(*osm));
}

std::vector<Building> WorldTerrain::buildings_tile(const g2m::TileKey& key,double min_height,double fallback,double storey) {
    auto response=transport_->send(g2m::Request{g2m::TileRequest{manifest_rid_,std::string(g2m::kSrcOsmLayer),key,std::nullopt}});
    const auto* tile=std::get_if<g2m::TileResponse>(&response);if(!tile||tile->meta.status!=g2m::Status::Ok)return {};
    auto container=g2m::parse_container(tile->container);if(!container||container->header.key!=key||container->header.layer!=g2m::kSrcOsmLayer)return {};
    auto body=g2m::decode_body(container->body);if(!body)return {};auto osm=g2m::decode_src_osm(*body);if(!osm)return {};
    return extract_buildings(*osm,key,min_height,fallback,storey,[this,key](double e,double n){
        return sample_l0_height([this](const auto& k){return raw_height_tile_shared(k).tile.get();},key.zone,0,0,e,n);
    });
}

const g2m::HeightTile* WorldTerrain::cached_height_tile(const g2m::TileKey& key) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    const auto it=height_cache_.find(key);
    return it==height_cache_.end()?nullptr:it->second.get();
}

HeightTileFetchResult WorldTerrain::height_tile_shared(const g2m::TileKey& key) {
    HeightTileFetchFn fetch_fn = [this](const g2m::TileKey& k) -> HeightTileFetchResult {
        FetchDecodeResult result;
        try {
            result = fetch_and_decode(k);
        } catch (const std::exception& error) {
            std::fprintf(stderr,"RG_TERRAIN_FETCH exception key=%s message=%s\n",g2m::to_string(k).c_str(),error.what());
            throw;
        }
        if (result.status != g2m::Status::Ok && result.status != g2m::Status::NotFound) {
            std::fprintf(stderr,"RG_TERRAIN_FETCH failed key=%s status=%d\n",g2m::to_string(k).c_str(),static_cast<int>(result.status));
        }
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
                                  ClassLookup{&world_terrain_road_class_lookup, this}, road_surfaces_);
}

TerrainViewSource WorldTerrain::view_source() {
    TerrainViewSource source;
    source.params = lod_params_;
    source.e0 = static_cast<double>(frame_->e0_m());
    source.n0 = static_cast<double>(frame_->n0_m());
    source.lookup = &world_terrain_lookup;
    source.ctx = this;
    source.class_lookup = ClassLookup{&world_terrain_road_class_lookup, this};
    source.surfaces = road_surfaces_;
    return source;
}

namespace {
const std::shared_ptr<const std::vector<g2m::RoadSegment>>& empty_road_segments() {
    static const auto kEmpty = std::make_shared<const std::vector<g2m::RoadSegment>>();
    return kEmpty;
}
} // namespace

RoadSegmentsResult decode_road_segments_response(const g2m::Response& response, const g2m::TileKey& expect_key,
                                                 g2m::geo::UtmZone zone) {
    const auto* tile_response = std::get_if<g2m::TileResponse>(&response);
    if (tile_response == nullptr) {
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    }
    if (tile_response->meta.status == g2m::Status::NotFound && tile_response->meta.message == "outside coverage") {
        // Absent: a definitive, cacheable "not in this release" answer, not
        // a failure - server.cpp's check_tile() gives this exact message
        // only for "outside coverage", never for "unknown layer"/"level not
        // in layer" (both of those mean this release has no g2m.src.osm
        // layer at all, and fall through to the generic NotFound Failed
        // case below - has_road_layer() is the caller's own guard for that).
        return RoadSegmentsResult{RoadFetchStatus::Absent, empty_road_segments()};
    }
    if (tile_response->meta.status != g2m::Status::Ok) {
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    }

    const g2m::Result<g2m::ContainerView> view = g2m::parse_container(tile_response->container);
    if (!view.ok()) {
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    }
    if (view.value().header.layer != g2m::kSrcOsmLayer || !(view.value().header.key == expect_key)) {
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    }
    const g2m::Result<g2m::TileBody> body = g2m::decode_body(view.value().body);
    if (!body.ok()) {
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    }
    const g2m::Result<g2m::OsmTile> osm_tile = g2m::decode_src_osm(body.value());
    if (!osm_tile.ok()) {
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    }

    std::vector<g2m::RoadSegment> segments =
        g2m::extract_road_segments(osm_tile.value().data, zone, g2m::RoadStyle::default_style());
    return RoadSegmentsResult{RoadFetchStatus::Ok,
                              std::make_shared<const std::vector<g2m::RoadSegment>>(std::move(segments))};
}

RoadSegmentsResult fetch_road_segments_cached(std::mutex& cache_mutex,
                                              std::map<g2m::TileKey, RoadSegmentsResult>& cache,
                                              const g2m::TileKey& key, const RoadSegmentsFetchFn& fetch_fn,
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
            return it->second;
        }
    }

    // Deliberately outside the lock - same rationale as
    // fetch_height_tile_cached's own comment.
    RoadSegmentsResult fetched = fetch_fn(key);
    if (fetched.status == RoadFetchStatus::Failed) {
        return fetched; // not cached: let a transient failure be retried later
    }

    std::lock_guard<std::mutex> lk(cache_mutex);
    auto [it, inserted] = cache.emplace(key, std::move(fetched));
    (void)inserted; // first inserted value wins, same policy as height_cache_
    return it->second;
}

RoadSegmentsResult WorldTerrain::road_segments_shared(const g2m::TileKey& key) {
    RoadSegmentsFetchFn fetch_fn = [this](const g2m::TileKey& k) -> RoadSegmentsResult {
        g2m::TileRequest request{manifest_rid_, std::string(g2m::kSrcOsmLayer), k, std::nullopt};
        g2m::Response response = transport_->send(g2m::Request{request});
        RoadSegmentsResult result = decode_road_segments_response(response, k, frame_->zone());
        if (result.status == RoadFetchStatus::Failed) {
            std::fprintf(stderr,
                         "WorldTerrain::road_segments_shared: g2m.src.osm fetch/decode failed (tile %s) - "
                         "reporting no roads for this tile\n",
                         g2m::to_string(k).c_str());
        }
        return result;
    };

    bool was_cache_hit = false;
    RoadSegmentsResult out = fetch_road_segments_cached(osm_cache_mutex_, osm_cache_, key, fetch_fn, &was_cache_hit);
    if (was_cache_hit) {
        osm_cache_hits_.fetch_add(1, std::memory_order_relaxed);
    } else if (out.status == RoadFetchStatus::Failed) {
        osm_fail_.fetch_add(1, std::memory_order_relaxed);
    } else {
        osm_ok_.fetch_add(1, std::memory_order_relaxed);
    }
    return out;
}

std::shared_ptr<const std::vector<g2m::RoadSegment>> WorldTerrain::road_segments(const g2m::TileKey& key) {
    RoadSegmentsResult result = road_segments_shared(key);
    return result.segments ? result.segments : empty_road_segments();
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
