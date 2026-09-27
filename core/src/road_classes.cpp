// rg/road_classes.cpp — see rg/road_classes.h for the public contract.
#include "rg/road_classes.h"

#include "g2m/core/tile_key.h"

#include <algorithm>
#include <cstddef>

namespace rg {

namespace {

// Allocation-free RoadBlockLookupFn adapter over a ClassLookup (G2.5a-grip
// R-b): ctx is the ClassLookup itself (its own fn/ctx call the production
// WorldTerrain::road_segments or a test's synthetic lookup, same as the
// pre-owner-rule code did per src_osm_tiles_for_chunk tile). An empty or
// null result is RoadBlockStatus::None (osm_roads.h's own "that tile
// genuinely has no drivable ways" case, not a failure) rather than Missing -
// this repo's ClassLookup contract never distinguishes "fetched, empty" from
// "fetch failed" (rg/road_classes.h's own ClassLookup comment: "empty on any
// failure"), so None is the only status this adapter can ever report short
// of Segments.
g2m::RoadBlockLookupResult class_lookup_block_fn(void* ctx, const g2m::TileKey& l2_key) {
    const auto* lookup = static_cast<const ClassLookup*>(ctx);
    std::shared_ptr<const std::vector<g2m::RoadSegment>> segs = lookup->fn(lookup->ctx, l2_key);
    if (!segs || segs->empty()) {
        return g2m::RoadBlockLookupResult{g2m::RoadBlockStatus::None, {}};
    }
    return g2m::RoadBlockLookupResult{g2m::RoadBlockStatus::Segments, std::span<const g2m::RoadSegment>(*segs)};
}

} // namespace

std::optional<g2m::RasterParams> road_raster_params_for_level(int level) {
    return g2m::raster_params_for_render_level(level);
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

    const g2m::Lattice lattice = road_class_lattice(key);
    const std::size_t sample_count = static_cast<std::size_t>(lattice.nx) * static_cast<std::size_t>(lattice.ny);
    std::vector<std::uint8_t> land_class(sample_count, static_cast<std::uint8_t>(g2m::LandClass::Unknown));
    std::vector<std::uint8_t> surface(sample_count, static_cast<std::uint8_t>(g2m::SurfaceKind::Unknown));

    // Owner rule (roads_plan.md section 2 / G2.5a-grip R-b): partition the
    // chunk's own lattice into aligned level-2 (1024 m) g2m.src.osm blocks
    // and paint each one from only its own owner tile's segments - see
    // rasterize_chunk_road_classes' own header comment. `ctx` is `&lookup`
    // itself, consumed synchronously (class_lookup_block_fn's call to
    // lookup.fn and rasterize_road_blocks' own paint both return before this
    // call does), so no lifetime issue despite the const_cast to the
    // non-const void* RoadBlockLookupFn requires.
    constexpr int kOsmBlockLevel = 2; // g2m.src.osm tiles are level 2 (1024 m), docs/formats/src_osm.md
    g2m::rasterize_road_blocks(lattice, kOsmBlockLevel, key.zone, *params, &class_lookup_block_fn,
                               const_cast<void*>(static_cast<const void*>(&lookup)), land_class, surface);

    // land_class is row-major j*nx+i (Lattice's own doc comment), matching
    // ClassWindow's row-major layout exactly (both row 0 = south, both
    // nx == ny == kChunkVerts) - a straight copy. surface (SurfaceKind) has
    // no ClassWindow home yet (render path only, roads_plan.md) and is
    // discarded here.
    std::copy(land_class.begin(), land_class.end(), out.c.begin());
}

} // namespace rg
