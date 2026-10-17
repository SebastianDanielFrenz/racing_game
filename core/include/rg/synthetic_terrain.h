// core/include/rg/synthetic_terrain.h — a deterministic in-memory terrain (no map store) for headless render-streaming
// benchmarks: rolling hills h(e, n) sampled on every level's height tile lattice on demand, behind a thread-safe
// TileLookup, with the game's colour hook. Used by RgTerrainView.load_synthetic_preview (game/scripts/
// terrain_upload_bench.gd) so the upload path can be timed where no geo2map store exists (C9, 2026-10-15).
#pragma once

#include "rg/world_terrain.h"

#include "g2m/core/geo/utm.h"
#include "g2m/layer/height_tile.h"

#include <map>
#include <memory>
#include <mutex>

namespace rg {

class SyntheticTerrain {
public:
    // e0/n0: the session origin (absolute UTM metres, zone 32N).
    SyntheticTerrain(double e0, double n0) : e0_(e0), n0_(n0) {}
    SyntheticTerrain(const SyntheticTerrain&) = delete;
    SyntheticTerrain& operator=(const SyntheticTerrain&) = delete;

    // A view source over this terrain (params.zone is set to 32N). `this` must outlive every holder.
    [[nodiscard]] TerrainViewSource view_source(g2m::mesh::LodParams params);
    // Thread-safe; tiles are built once and kept.
    const g2m::HeightTile* tile(const g2m::TileKey& key);
    // The terrain height (m) at an absolute UTM point.
    [[nodiscard]] static double height_m(double e, double n);

private:
    double e0_, n0_;
    std::mutex mutex_;
    std::map<g2m::TileKey, std::unique_ptr<g2m::HeightTile>> tiles_;
};

} // namespace rg
