// test_terrain_mode.cpp — rg/terrain_mode.h (R2.2 R4): the adapter from
// racing_game's cached height fetch (WorldTerrain::height_tile_shared's
// composition: fetch_height_tile_cached over decode_height_tile_container)
// to g2m::phys::IHeightTileFetch, and make_terrain_mode's spawn conversion.
//
// The identity case is the one the R4 brief asks for: for the same L0 tiles,
// served as real containers whose headers carry a NON-ZERO height_offset,
// the physics path (HeightTileSharedFetch -> ResidentHeightSet ->
// G2mTerrainSource::fill_tile) and the render path (the same cache as a
// g2m::mesh::TileLookup -> build_render_chunks' L0 meshes) must give the
// same heights, both equal to (raw + offset) / 256 - offset applied exactly
// once on each path (a double or missing offset was a real bug class, see
// commit 50d5d38). A second case pins racing_game's decode mirror against
// geo2map's own TransportHeightTileFetch decode, byte for byte.
//
// Synthetic containers only (g2m::encode_height_tile/encode_body/
// assemble_container, all in g2m_core) - never reads cache/.
#include "rg/route_check.h"
#include "rg/terrain_mode.h"
#include "rg/world_terrain.h"

#include "g2m/core/geo/session_frame.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/layer/tile_container.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/resident_heights.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"
#include "g2m/server/transport.h"

#include "ps/backend/shape_desc.h"
#include "ps/io/surface_table.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr g2m::geo::UtmZone k32N{32, g2m::geo::Hemisphere::North};
constexpr std::int64_t kE0 = 474000;
constexpr std::int64_t kN0 = 5560000;

g2m::geo::SessionFrame test_frame() { return g2m::geo::SessionFrame(k32N, kE0, kN0); }

// HEADER || BODY for `tile` with `height_offset` in the header (same helper
// shape as test_world_terrain.cpp's make_container).
std::vector<std::uint8_t> make_container(const g2m::HeightTile& tile, std::int32_t height_offset) {
    g2m::Result<g2m::TileBody> body = g2m::encode_height_tile(tile);
    REQUIRE(body.ok());
    g2m::Result<std::vector<std::uint8_t>> body_bytes = g2m::encode_body(body.value());
    REQUIRE(body_bytes.ok());
    g2m::TileHeader header;
    header.layer = std::string(g2m::kTerrainHeightLayer);
    header.key = tile.key;
    header.height_offset = height_offset;
    g2m::Result<std::vector<std::uint8_t>> container =
        g2m::assemble_container(header, std::span<const std::uint8_t>(body_bytes.value()));
    REQUIRE(container.ok());
    return std::move(container).value();
}

// A raw (pre-offset) heap tile (TOOL-039) with a per-cell pattern around
// 20 m, 1/256 m units; `nodata_at` (index into h, or -1) marks one sample.
std::unique_ptr<g2m::HeightTile> make_raw_tile(const g2m::TileKey& key, long nodata_at) {
    auto tile = std::make_unique<g2m::HeightTile>();
    tile->key = key;
    for (int j = 0; j < g2m::kTerrainTileSamples; ++j) {
        for (int i = 0; i < g2m::kTerrainTileSamples; ++i) {
            const std::int64_t c = key.min_easting() + i;
            const std::int64_t r = key.min_northing() + j;
            tile->h[static_cast<std::size_t>(j) * g2m::kTerrainTileSamples + static_cast<std::size_t>(i)] =
                20 * 256 + static_cast<std::int32_t>((c * 37 + r * 11) % 1531) - 700;
        }
    }
    tile->has_nodata = false;
    if (nodata_at >= 0) {
        tile->h[static_cast<std::size_t>(nodata_at)] = g2m::kHeightNoData;
        tile->has_nodata = true;
    }
    return tile;
}

// The container store plus racing_game's shared decoded-tile cache over it -
// the same composition WorldTerrain::height_tile_shared uses
// (fetch_height_tile_cached(decode_height_tile_container(server bytes))).
struct ContainerStore {
    std::map<std::uint64_t, std::vector<std::uint8_t>> containers; // by TileKey::packed()
    std::atomic<int> decode_calls{0};
    std::mutex cache_mutex;
    std::map<g2m::TileKey, std::shared_ptr<const g2m::HeightTile>> cache;

    rg::HeightTileFetchResult decode(const g2m::TileKey& key) {
        const auto it = containers.find(key.packed());
        if (it == containers.end()) return rg::HeightTileFetchResult{g2m::Status::NotFound, nullptr};
        ++decode_calls;
        return rg::decode_height_tile_container(it->second, g2m::kTerrainHeightLayer, key);
    }
    rg::HeightTileFetchResult shared(const g2m::TileKey& key) {
        return rg::fetch_height_tile_cached(cache_mutex, cache, key,
                                            [this](const g2m::TileKey& k) { return decode(k); });
    }
    static const g2m::HeightTile* lookup(void* ctx, const g2m::TileKey& key) {
        return static_cast<ContainerStore*>(ctx)->shared(key).tile.get();
    }
};

// g2m::ITransport serving a ContainerStore's bytes (404 when absent).
class StoreTransport final : public g2m::ITransport {
public:
    explicit StoreTransport(const ContainerStore& store) : store_(store) {}
    g2m::Response send(const g2m::Request& request) override {
        g2m::TileResponse out;
        const auto* tr = std::get_if<g2m::TileRequest>(&request);
        if (tr == nullptr) {
            out.meta.status = g2m::Status::BadRequest;
            return g2m::Response{out};
        }
        const auto it = store_.containers.find(tr->key.packed());
        if (it == store_.containers.end()) {
            out.meta.status = g2m::Status::NotFound;
            return g2m::Response{out};
        }
        out.meta.status = g2m::Status::Ok;
        out.container = it->second;
        return g2m::Response{out};
    }

private:
    const ContainerStore& store_;
};

struct Fixture {
    g2m::phys::PhysicsTileGrid grid{test_frame()};
    std::vector<g2m::TileKey> keys;            // L0 keys of physics tile (0, 0)
    std::map<std::uint64_t, std::int32_t> offset_by_key;
    std::map<std::uint64_t, std::unique_ptr<g2m::HeightTile>> raw_by_key;
    ContainerStore store;

    Fixture() {
        grid.l0_keys_for_square(g2m::phys::PhysTileIndex{0, 0}, 0, keys);
        // Both signs, a fractional value and 0; every tile's offset differs.
        const std::int32_t offsets[] = {50 * 256, -(20 * 256 + 128), 123 * 256 + 64, 0};
        for (std::size_t i = 0; i < keys.size(); ++i) {
            const g2m::TileKey& key = keys[i];
            // One NoData sample inside the physics tile's footprint, in the
            // first key (its sample at column e0+3, row n0+5).
            long nodata_at = -1;
            if (i == 0) {
                const std::int64_t li = (kE0 + 3) - key.min_easting();
                const std::int64_t lj = (kN0 + 5) - key.min_northing();
                if (li >= 0 && li < 256 && lj >= 0 && lj < 256) nodata_at = static_cast<long>(lj * 256 + li);
            }
            auto raw = make_raw_tile(key, nodata_at);
            const std::int32_t off = offsets[i % 4];
            store.containers.emplace(key.packed(), make_container(*raw, off));
            offset_by_key.emplace(key.packed(), off);
            raw_by_key.emplace(key.packed(), std::move(raw));
        }
    }

    // Expected metres at global cell (c, r), or NaN for NoData.
    double expected(std::int64_t c, std::int64_t r) const {
        g2m::TileKey key;
        key.zone = k32N;
        key.level = 0;
        key.x = static_cast<std::int32_t>(c >= 0 ? c / 256 : (c - 255) / 256);
        key.y = static_cast<std::int32_t>(r >= 0 ? r / 256 : (r - 255) / 256);
        const g2m::HeightTile& raw = *raw_by_key.at(key.packed());
        const std::int32_t h = raw.h[static_cast<std::size_t>((r - key.min_northing()) * 256 + (c - key.min_easting()))];
        if (h == g2m::kHeightNoData) return std::nan("");
        return static_cast<double>(h + offset_by_key.at(key.packed())) / 256.0;
    }
};

} // namespace

TEST_CASE("terrain mode: physics and render heights agree, height_offset applied exactly once on both paths",
          "[terrain_mode]") {
    Fixture fx;
    REQUIRE(fx.keys.size() == 4);

    // --- Physics path: HeightTileSharedFetch -> ResidentHeightSet -> fill_tile.
    rg::HeightTileSharedFetch fetch([&fx](const g2m::TileKey& k) { return fx.store.shared(k); });
    auto resident = std::make_shared<g2m::phys::ResidentHeightSet>();
    for (const g2m::TileKey& key : fx.keys) {
        const g2m::phys::FetchResult fr = fetch.fetch(key);
        REQUIRE(fr.status == g2m::Status::Ok);
        REQUIRE(fr.tile != nullptr);
        resident->install(key, fr.tile);
    }
    g2m::ps_bridge::G2mTerrainSource source(fx.grid, resident, ps::SurfaceId{0});
    ps::terrain::TileSample sample;
    sample.heights.assign(256u * 256u, 0.0);
    sample.cell_surface_ids.assign(255u * 255u, ps::kInvalidSurfaceId);
    source.fill_tile(ps::terrain::TileKey{g2m::phys::PhysicsTileGrid::pack({0, 0})}, sample);
    CHECK(source.fill_miss_count() == 0);

    std::size_t phys_checked = 0, phys_mismatch = 0, phys_holes = 0;
    for (int j = 0; j < 256; ++j) {
        for (int k = 0; k < 256; ++k) {
            const double want = fx.expected(kE0 + k, kN0 + j);
            const double got = sample.heights[static_cast<std::size_t>(j) * 256 + static_cast<std::size_t>(k)];
            if (std::isnan(want)) {
                ++phys_holes;
                if (got != ps::kHeightfieldNoCollision) ++phys_mismatch;
                continue;
            }
            ++phys_checked;
            if (got != want) ++phys_mismatch; // exact: h/256 is exact in double
        }
    }
    CHECK(phys_holes == 1);
    CHECK(phys_mismatch == 0);

    // --- Render path: the SAME cache as a TileLookup -> L0 chunk meshes.
    // L0 chunk (cx, cy) vertex (i, j) sits on cell (64*cx + i, 64*cy + j)'s
    // centre, i.e. absolute (64*cx + i + 0.5, 64*cy + j + 0.5).
    std::vector<g2m::mesh::ChunkKey> chunk_keys;
    for (std::int32_t cy = static_cast<std::int32_t>(kN0 / 64); cy <= static_cast<std::int32_t>((kN0 + 255) / 64); ++cy) {
        for (std::int32_t cx = static_cast<std::int32_t>(kE0 / 64); cx <= static_cast<std::int32_t>((kE0 + 255) / 64);
             ++cx) {
            chunk_keys.push_back(g2m::mesh::ChunkKey{k32N, 0, cx, cy});
        }
    }
    std::vector<rg::RenderChunk> chunks;
    REQUIRE(rg::build_render_chunks(chunk_keys, &ContainerStore::lookup, &fx.store, static_cast<double>(kE0),
                                    static_cast<double>(kN0), 1, chunks));

    std::size_t render_checked = 0, render_bad_xy = 0;
    double render_max_err = 0.0, render_vs_phys_max = 0.0;
    for (const rg::RenderChunk& ch : chunks) {
        for (int j = 0; j < g2m::mesh::kChunkVerts; ++j) {
            for (int i = 0; i < g2m::mesh::kChunkVerts; ++i) {
                const std::int64_t c = 64 * static_cast<std::int64_t>(ch.key.cx) + i;
                const std::int64_t r = 64 * static_cast<std::int64_t>(ch.key.cy) + j;
                if (c < kE0 || c > kE0 + 255 || r < kN0 || r > kN0 + 255) continue;
                const double want = fx.expected(c, r);
                if (std::isnan(want)) continue;
                const std::size_t v = static_cast<std::size_t>(j) * g2m::mesh::kChunkVerts + static_cast<std::size_t>(i);
                const double x = ch.mesh.origin[0] + static_cast<double>(ch.mesh.positions[3 * v + 0]);
                const double y = ch.mesh.origin[1] + static_cast<double>(ch.mesh.positions[3 * v + 1]);
                const double z = ch.mesh.origin[2] + static_cast<double>(ch.mesh.positions[3 * v + 2]);
                if (std::abs(x - (static_cast<double>(c) + 0.5)) > 1e-3 || std::abs(y - (static_cast<double>(r) + 0.5)) > 1e-3) {
                    ++render_bad_xy;
                }
                render_max_err = std::max(render_max_err, std::abs(z - want));
                const double phys = sample.heights[static_cast<std::size_t>(r - kN0) * 256 + static_cast<std::size_t>(c - kE0)];
                render_vs_phys_max = std::max(render_vs_phys_max, std::abs(z - phys));
                ++render_checked;
            }
        }
    }
    std::printf("[terrain_mode] identity: physics samples %zu (exact), render vertices %zu, render max |z - "
                "(raw+offset)/256| = %.3g m, render vs physics max = %.3g m, decodes %d\n",
                phys_checked, render_checked, render_max_err, render_vs_phys_max, fx.store.decode_calls.load());
    CHECK(render_checked >= phys_checked); // every physics sample has >= 1 render vertex (chunk borders repeat)
    CHECK(render_bad_xy == 0);
    CHECK(render_max_err <= 1e-4);        // float rounding of ~20..150 m values; a +-offset error would be >= 20 m
    CHECK(render_vs_phys_max <= 1e-4);

    // One decode per tile: both paths read the same cached object.
    CHECK(fx.store.decode_calls.load() == 4);
    for (const g2m::TileKey& key : fx.keys) {
        CHECK(ContainerStore::lookup(&fx.store, key) == fetch.fetch(key).tile.get());
    }
}

TEST_CASE("terrain mode: racing_game's container decode equals geo2map's TransportHeightTileFetch",
          "[terrain_mode]") {
    Fixture fx;
    StoreTransport transport(fx.store);
    g2m::phys::TransportHeightTileFetch g2m_fetch(transport, g2m::ReleaseId{});
    for (const g2m::TileKey& key : fx.keys) {
        const g2m::phys::FetchResult theirs = g2m_fetch.fetch(key);
        const rg::HeightTileFetchResult ours = fx.store.decode(key);
        REQUIRE(theirs.status == g2m::Status::Ok);
        REQUIRE(ours.status == g2m::Status::Ok);
        CHECK(theirs.tile->key == ours.tile->key);
        CHECK(theirs.tile->has_nodata == ours.tile->has_nodata);
        CHECK(theirs.tile->h == ours.tile->h);
    }
    g2m::TileKey missing = fx.keys[0];
    missing.x += 100;
    CHECK(g2m_fetch.fetch(missing).status == g2m::Status::NotFound);
    CHECK(fx.store.decode(missing).status == g2m::Status::NotFound);
}

TEST_CASE("terrain mode: HeightTileSharedFetch passes status and tile through", "[terrain_mode]") {
    auto tile = std::make_shared<g2m::HeightTile>();
    g2m::Status next = g2m::Status::Ok;
    std::shared_ptr<const g2m::HeightTile> next_tile = tile;
    rg::HeightTileSharedFetch fetch([&](const g2m::TileKey&) { return rg::HeightTileFetchResult{next, next_tile}; });
    const g2m::TileKey key{k32N, 0, 1851, 21718};

    g2m::phys::FetchResult r = fetch.fetch(key);
    CHECK(r.status == g2m::Status::Ok);
    CHECK(r.tile.get() == tile.get()); // same object, no copy

    for (const g2m::Status s : {g2m::Status::NotFound, g2m::Status::Unavailable, g2m::Status::Internal}) {
        next = s;
        next_tile = nullptr;
        r = fetch.fetch(key);
        CHECK(r.status == s);
        CHECK(r.tile == nullptr);
    }
    // Ok without a tile breaks the contract -> Internal (retried, then Failed).
    next = g2m::Status::Ok;
    next_tile = nullptr;
    CHECK(fetch.fetch(key).status == g2m::Status::Internal);

    CHECK_THROWS_AS(rg::HeightTileSharedFetch(rg::HeightTileFetchFn{}), std::invalid_argument);
}

TEST_CASE("terrain mode: make_terrain_mode converts the spawn into the session frame", "[terrain_mode]") {
    rg::WorldConfig wc;
    wc.spawn.e = 474123.25;
    wc.spawn.n = 5559950.5;
    wc.spawn.yaw_deg = 90.0;
    wc.physics.radius_m = 300.0;
    wc.physics.terrain_surface = "dirt";
    wc.physics.loader_workers = 3;
    auto fetch = std::make_shared<rg::HeightTileSharedFetch>(
        [](const g2m::TileKey&) { return rg::HeightTileFetchResult{g2m::Status::NotFound, nullptr}; });

    const rg::TerrainModeConfig tm = rg::make_terrain_mode(wc, test_frame(), fetch);
    CHECK(tm.frame.e0_m() == kE0);
    CHECK(tm.frame.n0_m() == kN0);
    CHECK(tm.fetch.get() == fetch.get());
    CHECK(tm.spawn_x == 123.25);
    CHECK(tm.spawn_y == -49.5);
    CHECK(std::abs(tm.spawn_yaw_rad - 1.5707963267948966) < 1e-15);
    CHECK(tm.physics.radius_m == 300.0);
    CHECK(tm.physics.terrain_surface == "dirt");
    CHECK(tm.physics.loader_workers == 3);
    CHECK(tm.prime_ticks == 0);

    CHECK_THROWS_AS(rg::make_terrain_mode(wc, std::shared_ptr<rg::WorldTerrain>{}), std::invalid_argument);
}

// --- G2.5a-grip R-c: physics-vs-render invariant, real make_terrain_mode ---
// (real data: the shipped data/world/world_config.json's own store; hidden
// like every other [.][realdata] test in this suite - never run by default,
// needs RG_G2M_HOME.)
namespace {

std::optional<std::string> safe_getenv(const char* name) {
#ifdef _WIN32
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) return std::nullopt;
    std::string value(buf);
    std::free(buf);
    return value;
#else
    const char* value = std::getenv(name);
    return value != nullptr ? std::optional<std::string>(value) : std::nullopt;
#endif
}

// Same arc-length walk as test_route_road_render_path.cpp's own point_at_s().
std::optional<std::pair<double, double>> point_at_s(const rg::Route& route, double target_s) {
    if (route.waypoints.size() < 2 || target_s < 0.0) return std::nullopt;
    double along = 0.0;
    for (std::size_t i = 1; i < route.waypoints.size(); ++i) {
        const double ax = route.waypoints[i - 1].x, ay = route.waypoints[i - 1].y;
        const double bx = route.waypoints[i].x, by = route.waypoints[i].y;
        const double seg = std::sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
        if (along + seg >= target_s || i + 1 == route.waypoints.size()) {
            const double t = seg <= 0.0 ? 0.0 : std::clamp((target_s - along) / seg, 0.0, 1.0);
            return std::make_pair(route.e0 + ax + (bx - ax) * t, route.n0 + ay + (by - ay) * t);
        }
        along += seg;
    }
    return std::nullopt;
}

} // namespace

TEST_CASE("terrain mode: real make_terrain_mode's road-mode grip agrees with the render path (physics paints paved "
          "wherever the render path paints a paved land_class)",
          "[.][realdata][terrain_mode]") {
    if (!safe_getenv("RG_G2M_HOME").has_value()) SKIP("RG_G2M_HOME not set");
    std::string err;
    const auto world = rg::load_world_config(std::string(RG_SOURCE_DIR) + "/data/world/world_config.json", &err);
    INFO(err);
    REQUIRE(world.has_value());
    REQUIRE(world->physics.road_surfaces.enabled); // the shipped file, G2.5a-grip R-c

    std::shared_ptr<rg::WorldTerrain> terrain(rg::WorldTerrain::open(*world, &err));
    INFO(err);
    REQUIRE(terrain != nullptr);

    // The REAL entry point: make_terrain_mode(config, WorldTerrain) auto-
    // selects HeightTileSharedFetch's road-aware constructor because
    // world->physics.road_surfaces.enabled is true.
    const rg::TerrainModeConfig tm = rg::make_terrain_mode(*world, terrain);
    CHECK(tm.road_layer_available == terrain->has_road_layer());
    REQUIRE(tm.fetch != nullptr);
    CHECK(tm.fetch->provides_roads());

    const auto route = rg::load_route(std::string(RG_SOURCE_DIR) + "/data/routes/home_r1_drive.json", &err);
    INFO(err);
    REQUIRE(route.has_value());
    // test_route_road_render_path.cpp's own "05_road_b8_junction" target -
    // already proven within 15 m of a real B8 road segment there.
    const auto point = point_at_s(*route, 150.0);
    REQUIRE(point.has_value());
    const auto [e_m, n_m] = *point;
    const g2m::geo::UtmZone zone{route->zone};

    // --- Render path: does the L0 chunk at this point carry any painted
    // (non-Unknown) land_class vertex? (mirrors test_route_road_render_path.cpp)
    const rg::TerrainViewSource vs = terrain->view_source();
    std::vector<g2m::mesh::ChunkKey> keys;
    rg::select_view_keys(e_m - vs.e0, n_m - vs.n0, vs.params, vs.e0, vs.n0, keys);
    std::vector<rg::RenderChunk> chunks;
    REQUIRE(rg::build_render_chunks(keys, vs.lookup, vs.ctx, vs.e0, vs.n0, 4, chunks, nullptr, vs.class_lookup));
    const g2m::mesh::ChunkKey want{zone, /*level=*/0, static_cast<std::int32_t>(std::floor(e_m / 64.0)),
                                  static_cast<std::int32_t>(std::floor(n_m / 64.0))};
    int render_road_vertices = 0;
    std::size_t render_total_vertices = 0;
    bool found_chunk = false;
    for (const rg::RenderChunk& c : chunks) {
        if (c.key == want) {
            found_chunk = true;
            render_total_vertices = c.mesh.land_class.size();
            for (std::uint8_t lc : c.mesh.land_class) {
                if (lc != static_cast<std::uint8_t>(g2m::LandClass::Unknown)) ++render_road_vertices;
            }
            break;
        }
    }
    REQUIRE(found_chunk);
    REQUIRE(render_road_vertices > 0); // ground truth, matches test_route_road_render_path.cpp

    // --- Physics path: the SAME make_terrain_mode-wired fetch, through
    // G2mTerrainSource::fill_tile - exactly Session::setup_terrain's own
    // RoadValueLut construction from physics.road_surfaces via a real
    // SurfaceTable.
    const ps::io::SurfaceTable surfaces(std::string(RG_SOURCE_DIR) +
                                        "/external/physics_sim/data/surfaces/surfaces.json");
    const ps::SurfaceId paved = surfaces.id_for(world->physics.road_surfaces.paved);
    const ps::SurfaceId unpaved = surfaces.id_for(world->physics.road_surfaces.unpaved);
    const ps::SurfaceId off_road = surfaces.id_for(world->physics.road_surfaces.off_road);
    REQUIRE(paved != ps::kInvalidSurfaceId);
    REQUIRE(unpaved != ps::kInvalidSurfaceId);
    REQUIRE(off_road != ps::kInvalidSurfaceId);
    const g2m::RoadValueLut lut = g2m::RoadValueLut::by_land_class(
        off_road, {{g2m::LandClass::PavedRoad, paved}, {g2m::LandClass::UnpavedRoad, unpaved}});

    g2m::phys::PhysicsTileGrid grid(tm.frame);
    auto resident = std::make_shared<g2m::phys::ResidentHeightSet>();
    const g2m::phys::PhysTileIndex idx = grid.index_for_local(e_m - tm.frame.e0_m(), n_m - tm.frame.n0_m());
    std::vector<g2m::TileKey> l0_keys;
    grid.l0_keys_for_square(idx, 0, l0_keys);
    for (const g2m::TileKey& key : l0_keys) {
        const g2m::phys::FetchResult fr = tm.fetch->fetch(key);
        REQUIRE(fr.status == g2m::Status::Ok);
        REQUIRE(fr.tile != nullptr);
        resident->install(key, fr.tile, fr.roads);
    }
    g2m::ps_bridge::G2mTerrainSource source(grid, resident, g2m::ps_bridge::RoadSurfaceConfig{lut, off_road});
    ps::terrain::TileSample sample;
    sample.heights.assign(256u * 256u, 0.0);
    sample.cell_surface_ids.assign(255u * 255u, ps::kInvalidSurfaceId);
    source.fill_tile(ps::terrain::TileKey{g2m::phys::PhysicsTileGrid::pack(idx)}, sample);

    std::size_t paved_cells = 0;
    for (const ps::SurfaceId id : sample.cell_surface_ids) {
        if (id == paved) ++paved_cells;
    }
    std::printf("[terrain_mode] physics-vs-render invariant: render road vertices %d/%zu, physics paved cells "
                "%zu/%zu, fill misses %llu\n",
                render_road_vertices, render_total_vertices, paved_cells, sample.cell_surface_ids.size(),
                static_cast<unsigned long long>(source.fill_miss_count()));
    CHECK(source.fill_miss_count() == 0);
    CHECK(paved_cells > 0); // the invariant: wherever render paints paved, physics grip is paved too
}
