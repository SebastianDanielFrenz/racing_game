#include "rg/terrain_mode.h"

#include <stdexcept>
#include <utility>

namespace rg {

HeightTileSharedFetch::HeightTileSharedFetch(HeightTileFetchFn fn) : fn_(std::move(fn)) {
    if (!fn_) throw std::invalid_argument("HeightTileSharedFetch: empty fetch function");
}

g2m::phys::FetchResult HeightTileSharedFetch::fetch(const g2m::TileKey& key) {
    HeightTileFetchResult r = fn_(key);
    // Keep the contract "tile non-null iff Ok" on the physics side too.
    if (r.status != g2m::Status::Ok) return g2m::phys::FetchResult{r.status, nullptr};
    if (r.tile == nullptr) return g2m::phys::FetchResult{g2m::Status::Internal, nullptr};
    return g2m::phys::FetchResult{r.status, std::move(r.tile)};
}

TerrainModeConfig make_terrain_mode(const WorldConfig& config, const g2m::geo::SessionFrame& frame,
                                    std::shared_ptr<g2m::phys::IHeightTileFetch> fetch) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    TerrainModeConfig out(frame, std::move(fetch));
    out.spawn_x = config.spawn.e - static_cast<double>(frame.e0_m());
    out.spawn_y = config.spawn.n - static_cast<double>(frame.n0_m());
    out.spawn_yaw_rad = config.spawn.yaw_deg * kDegToRad;
    out.physics = config.physics;
    return out;
}

TerrainModeConfig make_terrain_mode(const WorldConfig& config, std::shared_ptr<WorldTerrain> terrain) {
    if (terrain == nullptr) throw std::invalid_argument("make_terrain_mode: null WorldTerrain");
    const g2m::geo::SessionFrame frame = terrain->frame();
    auto fetch = std::make_shared<HeightTileSharedFetch>(
        [terrain](const g2m::TileKey& key) { return terrain->height_tile_shared(key); });
    TerrainModeConfig out = make_terrain_mode(config, frame, std::move(fetch));
    out.world_terrain = std::move(terrain);
    return out;
}

} // namespace rg
