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
class HeightTileSharedFetch final : public g2m::phys::IHeightTileFetch {
public:
    // Throws std::invalid_argument on an empty `fn`.
    explicit HeightTileSharedFetch(HeightTileFetchFn fn);
    g2m::phys::FetchResult fetch(const g2m::TileKey& key) override;

private:
    HeightTileFetchFn fn_;
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
