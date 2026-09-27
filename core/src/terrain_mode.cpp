#include "rg/terrain_mode.h"

#include <stdexcept>
#include <utility>

namespace rg {

HeightTileSharedFetch::HeightTileSharedFetch(HeightTileFetchFn fn) : fn_(std::move(fn)) {
    if (!fn_) throw std::invalid_argument("HeightTileSharedFetch: empty fetch function");
}

HeightTileSharedFetch::HeightTileSharedFetch(HeightTileFetchFn fn, RoadSegmentsFetchFn road_fn)
    : fn_(std::move(fn)), road_fn_(std::move(road_fn)) {
    if (!fn_) throw std::invalid_argument("HeightTileSharedFetch: empty fetch function");
    if (!road_fn_) throw std::invalid_argument("HeightTileSharedFetch: empty road fetch function");
}

g2m::phys::FetchResult HeightTileSharedFetch::fetch(const g2m::TileKey& key) {
    HeightTileFetchResult r = fn_(key);
    // Keep the contract "tile non-null iff Ok" on the physics side too.
    if (r.status != g2m::Status::Ok) return g2m::phys::FetchResult{r.status, nullptr, nullptr};
    if (r.tile == nullptr) return g2m::phys::FetchResult{g2m::Status::Internal, nullptr, nullptr};

    std::shared_ptr<const std::vector<g2m::RoadSegment>> roads;
    if (road_fn_) {
        // Roads ride on the SAME residency entry as heights (G2.5a-grip
        // plan section 3): an L0 height key's owning g2m.src.osm tile is
        // its L2 ancestor. A Failed road fetch leaves `roads` null, which
        // (with LoaderConfig::require_roads) makes the whole tile retry/
        // fail exactly like a height-side 500 - never silently missing.
        const g2m::TileKey l2 = key.parent().parent();
        const RoadSegmentsResult rr = road_fn_(l2);
        if (rr.status != RoadFetchStatus::Failed) roads = rr.segments;
    }
    return g2m::phys::FetchResult{r.status, std::move(r.tile), std::move(roads)};
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
    std::shared_ptr<HeightTileSharedFetch> fetch;
    if (config.physics.road_surfaces.enabled) {
        fetch = std::make_shared<HeightTileSharedFetch>(
            [terrain](const g2m::TileKey& key) { return terrain->height_tile_shared(key); },
            [terrain](const g2m::TileKey& l2) { return terrain->road_segments_shared(l2); });
    } else {
        fetch = std::make_shared<HeightTileSharedFetch>(
            [terrain](const g2m::TileKey& key) { return terrain->height_tile_shared(key); });
    }
    const bool road_layer_available = terrain->has_road_layer();
    TerrainModeConfig out = make_terrain_mode(config, frame, std::move(fetch));
    out.road_layer_available = road_layer_available;
    out.world_terrain = std::move(terrain);
    return out;
}

} // namespace rg
