// rg/road_classes.cpp — see rg/road_classes.h for the public contract.
#include "rg/road_classes.h"

#include "g2m/core/tile_key.h"

#include <algorithm>
#include <cstddef>

namespace rg {

namespace {

// osm_roads.cpp's own kDefaultHighways rank table, current values (track=0
// .. motorway=9): fallback only, used if a style is ever shipped without
// "tertiary"/"primary" at all (road_raster_params_for_level's own comment).
constexpr int kFallbackTertiaryRank = 5;
constexpr int kFallbackPrimaryRank = 7;

} // namespace

std::optional<g2m::RasterParams> road_raster_params_for_level(int level) {
    if (level <= 1) {
        return g2m::RasterParams{/*min_rank=*/0, /*min_half_width_mm=*/0};
    }

    const g2m::RoadStyle& style = g2m::RoadStyle::default_style();

    if (level == 2) {
        const g2m::HighwayStyleEntry* tertiary = style.find_highway("tertiary");
        const int min_rank = tertiary != nullptr ? tertiary->rank : kFallbackTertiaryRank;
        return g2m::RasterParams{min_rank, /*min_half_width_mm=*/0};
    }

    if (level == 3) {
        const g2m::HighwayStyleEntry* primary = style.find_highway("primary");
        const int min_rank = primary != nullptr ? primary->rank : kFallbackPrimaryRank;
        // spacing_mm = (1 << level) * 1000 is exactly divisible by 4 for
        // level >= 2, so this integer arithmetic is exact (0.75 * spacing).
        const std::int64_t spacing_mm = (std::int64_t{1} << level) * 1000;
        const std::int64_t min_half_width_mm = (spacing_mm * 3) / 4;
        return g2m::RasterParams{min_rank, min_half_width_mm};
    }

    return std::nullopt; // L>=4: no road classes
}

g2m::Lattice road_class_lattice(const g2m::mesh::ChunkKey& key) {
    const std::int64_t spacing_mm = (std::int64_t{1} << key.level) * 1000;
    // terrain_chunk.h's vertex-placement formula: vertex (i, j) sits at
    // absolute E = (64*cx + i) * 2^level + 0.5 (same for N) - a FIXED
    // 500 mm offset, not scaled by 2^level.
    const std::int64_t origin_e_mm = static_cast<std::int64_t>(key.cx) * 64 * spacing_mm + 500;
    const std::int64_t origin_n_mm = static_cast<std::int64_t>(key.cy) * 64 * spacing_mm + 500;
    return g2m::Lattice{origin_e_mm, origin_n_mm, spacing_mm, g2m::mesh::kChunkVerts, g2m::mesh::kChunkVerts};
}

std::vector<g2m::TileKey> src_osm_tiles_for_chunk(const g2m::mesh::ChunkKey& key) {
    const std::int64_t size_m = 64 * (std::int64_t{1} << key.level);
    const std::int64_t e_min = static_cast<std::int64_t>(key.cx) * size_m;
    const std::int64_t n_min = static_cast<std::int64_t>(key.cy) * size_m;
    const std::int64_t e_max = e_min + size_m;
    const std::int64_t n_max = n_min + size_m;

    constexpr int kOsmLevel = 2; // g2m.src.osm tiles are level 2 (1024 m), docs/formats/src_osm.md
    constexpr std::int64_t kOsmTileSizeM = g2m::TileKey::kLevel0SizeM << kOsmLevel;

    const std::int64_t x0 = g2m::floor_div(e_min, kOsmTileSizeM);
    const std::int64_t x1 = g2m::floor_div(e_max, kOsmTileSizeM);
    const std::int64_t y0 = g2m::floor_div(n_min, kOsmTileSizeM);
    const std::int64_t y1 = g2m::floor_div(n_max, kOsmTileSizeM);

    std::vector<g2m::TileKey> out;
    out.reserve(static_cast<std::size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));
    for (std::int64_t ty = y0; ty <= y1; ++ty) {
        for (std::int64_t tx = x0; tx <= x1; ++tx) {
            out.push_back(g2m::TileKey{key.zone, kOsmLevel, static_cast<std::int32_t>(tx), static_cast<std::int32_t>(ty)});
        }
    }
    return out;
}

void rasterize_chunk_road_classes(const g2m::mesh::ChunkKey& key, const ClassLookup& lookup,
                                   g2m::mesh::ClassWindow& out) {
    out = g2m::mesh::ClassWindow{}; // all-Unknown (LandClass::Unknown == 0)
    if (lookup.fn == nullptr) {
        return;
    }
    const std::optional<g2m::RasterParams> params = road_raster_params_for_level(key.level);
    if (!params.has_value()) {
        return;
    }

    std::vector<g2m::RoadSegment> merged;
    for (const g2m::TileKey& tile_key : src_osm_tiles_for_chunk(key)) {
        std::shared_ptr<const std::vector<g2m::RoadSegment>> segs = lookup.fn(lookup.ctx, tile_key);
        if (segs && !segs->empty()) {
            merged.insert(merged.end(), segs->begin(), segs->end());
        }
    }
    if (merged.empty()) {
        return;
    }
    // Each tile's own extract_road_segments() output is sorted by
    // (rank, way id, segment index); the concatenation across tiles is not
    // sorted by rank overall, and rasterize_road_segments trusts array
    // order for its own "later overwrites earlier" rule.
    std::stable_sort(merged.begin(), merged.end(),
                      [](const g2m::RoadSegment& a, const g2m::RoadSegment& b) { return a.rank < b.rank; });

    const g2m::Lattice lattice = road_class_lattice(key);
    const std::size_t sample_count = static_cast<std::size_t>(lattice.nx) * static_cast<std::size_t>(lattice.ny);
    std::vector<std::uint8_t> land_class(sample_count, static_cast<std::uint8_t>(g2m::LandClass::Unknown));
    std::vector<std::uint8_t> surface(sample_count, static_cast<std::uint8_t>(g2m::SurfaceKind::Unknown));
    g2m::rasterize_road_segments(merged, lattice, *params, land_class, surface);

    // land_class is row-major j*nx+i (Lattice's own doc comment), matching
    // ClassWindow's row-major layout exactly (both row 0 = south, both
    // nx == ny == kChunkVerts) - a straight copy. surface (SurfaceKind) has
    // no ClassWindow home yet (render path only, roads_plan.md) and is
    // discarded here.
    std::copy(land_class.begin(), land_class.end(), out.c.begin());
}

} // namespace rg
