#include "rg/synthetic_terrain.h"

#include "rg/terrain_colours.h"

#include <cmath>

namespace rg {

namespace {
const g2m::HeightTile* synthetic_lookup(void* ctx, const g2m::TileKey& key) {
    return static_cast<SyntheticTerrain*>(ctx)->tile(key);
}
} // namespace

double SyntheticTerrain::height_m(double e, double n) {
    return 150.0 + 40.0 * std::sin(e / 900.0) * std::cos(n / 1300.0) + 6.0 * std::sin(e / 70.0 + n / 110.0);
}

const g2m::HeightTile* SyntheticTerrain::tile(const g2m::TileKey& key) {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        const auto it = tiles_.find(key);
        if (it != tiles_.end()) return it->second.get();
    }
    // Built outside the lock (heap: HeightTile is 256 KiB); the first insert wins.
    auto t = std::make_unique<g2m::HeightTile>();
    t->key = key;
    const double spacing = static_cast<double>(std::int64_t{1} << key.level);
    const double size = 256.0 * spacing;
    const double x0 = static_cast<double>(key.x) * size;
    const double y0 = static_cast<double>(key.y) * size;
    for (int j = 0; j < 256; ++j)
        for (int i = 0; i < 256; ++i)
            t->h[static_cast<std::size_t>(j) * 256 + static_cast<std::size_t>(i)] = static_cast<std::int32_t>(
                std::lround(height_m(x0 + (i + 0.5) * spacing, y0 + (j + 0.5) * spacing) * 256.0));
    std::lock_guard<std::mutex> lk(mutex_);
    return tiles_.emplace(key, std::move(t)).first->second.get();
}

TerrainViewSource SyntheticTerrain::view_source(g2m::mesh::LodParams params) {
    params.zone = g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North};
    TerrainViewSource source;
    source.params = params;
    source.e0 = e0_;
    source.n0 = n0_;
    source.lookup = &synthetic_lookup;
    source.ctx = this;
    source.colour = chunk_colour_hook();
    return source;
}

} // namespace rg
