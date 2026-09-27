// rg/terrain_mode.h — everything rg::Session needs to run on streamed
// geo2map terrain instead of its flat ground box (R2.2 R4): TerrainModeConfig
// (SessionConfig::terrain), the adapter from racing_game's own cached height
// fetch (WorldTerrain::height_tile_shared) to geo2map's
// g2m::phys::IHeightTileFetch, and make_terrain_mode(), which builds a
// TerrainModeConfig from a loaded WorldConfig plus an open WorldTerrain.
// Engine-neutral (no Godot type), same rule as session.h.
#pragma once

#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include "g2m/core/geo/session_frame.h"
#include "g2m/core/tile_key.h"
#include "g2m/phys/height_tile_loader.h"

#include <memory>

namespace rg {

// g2m::phys::IHeightTileFetch over a racing_game HeightTileFetchFn. The
// function's HeightTileFetchResult is passed through unchanged (status and
// the SAME shared_ptr - no copy, no second decode), so:
//  - 404 (NotFound) reaches the HeightTileLoader as 404 -> Absent (holes);
//  - 5xx and a decode failure (Status::Internal) are retried by the loader,
//    then reported as Failed;
//  - the physics path holds the very tile object the render path's
//    TileLookup returns (WorldTerrain's height cache).
// Thread-safe iff `fn` is (HeightTileLoader calls fetch() from its workers);
// WorldTerrain::height_tile_shared and fetch_height_tile_cached are.
//
// Road mode (G2.5a-grip R-c, plan section 7): the two-argument constructor
// additionally takes a RoadSegmentsFetchFn (rg::WorldTerrain::
// road_segments_shared's own signature, roads ride on the SAME residency
// entry as heights) - fetch() then also resolves the owning L2
// g2m.src.osm tile (key.parent().parent(), a height fetch's L0 key ->
// its L2 road tile) and calls it, ONLY when the height side itself
// succeeded (Ok with a tile) - a failed height fetch is retried on its own
// terms and never needs roads. RoadFetchStatus::Failed becomes a null
// FetchResult::roads (so LoaderConfig::require_roads retries/fails the
// whole tile exactly like a height-side 500); Ok or Absent passes the
// (possibly empty) segment list straight through - "no roads here" is a
// valid, non-fatal answer (roads_plan.md). provides_roads() reports whether
// this instance was built with a road fetch function at all.
class HeightTileSharedFetch final : public g2m::phys::IHeightTileFetch {
public:
    // Throws std::invalid_argument on an empty `fn`.
    explicit HeightTileSharedFetch(HeightTileFetchFn fn);
    // Throws std::invalid_argument on an empty `fn` or `road_fn`.
    HeightTileSharedFetch(HeightTileFetchFn fn, RoadSegmentsFetchFn road_fn);
    g2m::phys::FetchResult fetch(const g2m::TileKey& key) override;
    bool provides_roads() const override { return static_cast<bool>(road_fn_); }

private:
    HeightTileFetchFn fn_;
    RoadSegmentsFetchFn road_fn_; // empty = heights-only (provides_roads() false)
};

// SessionConfig::terrain. Positions are session-local metres (x east of the
// frame's e0, y north of its n0 - physics_sim's world XY).
struct TerrainModeConfig {
    // g2m::geo::SessionFrame has no default constructor, so neither has this.
    TerrainModeConfig(g2m::geo::SessionFrame frame_in, std::shared_ptr<g2m::phys::IHeightTileFetch> fetch_in)
        : frame(frame_in), fetch(std::move(fetch_in)) {}

    g2m::geo::SessionFrame frame;
    std::shared_ptr<g2m::phys::IHeightTileFetch> fetch; // required (Session throws on null)

    // The WorldTerrain the real game's make_terrain_mode(WorldConfig,
    // shared_ptr<WorldTerrain>) overload was built from, kept alongside
    // `fetch` (which only captures it inside a closure) so Session can expose
    // it again via Session::world_terrain() (R2.2 R7: RgTerrainView reuses
    // it instead of opening/decoding a second copy). Null when a
    // TerrainModeConfig is built directly (tests, tools/hash_check) via the
    // "pure" make_terrain_mode(WorldConfig, SessionFrame, fetch) overload, or
    // hand-assembled with a synthetic fetch.
    std::shared_ptr<WorldTerrain> world_terrain;

    double spawn_x = 0.0;
    double spawn_y = 0.0;
    // Heading of the chassis' forward (+x) axis in RADIANS: 0 = +x (east),
    // counter-clockwise about +Z (WorldConfig::Spawn::yaw_deg's convention,
    // converted by make_terrain_mode).
    double spawn_yaw_rad = 0.0;

    WorldConfig::PhysicsTerrainConfig physics;

    // Whether this release's manifest lists the g2m.src.osm layer at all
    // (WorldTerrain::has_road_layer(), G2.5a-grip R-c) - checked by
    // Session::setup_terrain before it builds a road-mode grip LUT, so
    // enabling physics.road_surfaces against a release with no road layer
    // fails fast instead of silently reading "no roads anywhere". Default
    // true (a synthetic TerrainModeConfig built directly, e.g. by tests or
    // tools/hash_check, has no release to ask - tests that need to exercise
    // this hard error set it to false directly). The real
    // make_terrain_mode(WorldConfig, shared_ptr<WorldTerrain>) overload
    // always sets it from the real WorldTerrain.
    bool road_layer_available = true;

    // Priming ticks stepped with no vehicle before the spawn ray casts.
    // 0 = auto: ceil(side^2 / max_tile_fills_per_tick) + 1, side =
    // 2*ceil(radius_m / 255) + 1 (TileManager's own needed square) - 26 at
    // the defaults (radius 400, F = 1).
    int prime_ticks = 0;
};

// The pure part of make_terrain_mode (testable without a WorldTerrain):
// spawn = (spawn.e - frame.e0_m(), spawn.n - frame.n0_m()), yaw converted
// from degrees to radians, physics copied from config.physics.
TerrainModeConfig make_terrain_mode(const WorldConfig& config, const g2m::geo::SessionFrame& frame,
                                    std::shared_ptr<g2m::phys::IHeightTileFetch> fetch);

// The real game's entry point: frame = terrain->frame(), fetch = a
// HeightTileSharedFetch over terrain->height_tile_shared (the fetch keeps
// `terrain` alive). Throws std::invalid_argument on a null `terrain`.
TerrainModeConfig make_terrain_mode(const WorldConfig& config, std::shared_ptr<WorldTerrain> terrain);

} // namespace rg
