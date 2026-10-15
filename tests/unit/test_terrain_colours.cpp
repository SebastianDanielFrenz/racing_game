// test_terrain_colours.cpp - the game's terrain chunk colours (world_terrain.cpp chunk_vertex_colors: the hypsometric
// height/slope ramp plus the asphalt/dirt road colours keyed by the resolved surface name). Pinned before terrain
// migration phase 3b moved the chunk build into geo2map (g2m::world::build_view_chunks with a colour hook), so the
// colours the game paints through the hook are byte-identical to the ones it painted itself. The pins were recorded
// at 5a3baed (rg::build_static_view_from_lookup); the same bytes come out of the wrapper and of geo2map's
// build_static_view with rg::chunk_colour_hook. Re-pinned when the road colours' alpha became the vertex kind (0 =
// road) for terrain.gdshader's detail textures (PR #7); RGB is unchanged.
#include "rg/world_terrain.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace {

// Ridged terrain from 100 to ~900 m: slopes up to 0.8 east-west (the rock blend) plus 0.1 north-south.
class RidgeStore {
public:
    const g2m::HeightTile* get(const g2m::TileKey& key) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = tiles_.find(key);
        if (it != tiles_.end()) return it->second.get();
        auto tile = std::make_unique<g2m::HeightTile>();
        tile->key = key;
        const std::int64_t step = std::int64_t{1} << key.level;
        for (int row = 0; row < 256; ++row)
            for (int col = 0; col < 256; ++col) {
                const std::int64_t e = (std::int64_t{key.x} * 256 + col) * step, n = (std::int64_t{key.y} * 256 + row) * step;
                const std::int64_t m = ((e % 2000) + 2000) % 2000;
                tile->h[static_cast<std::size_t>(row) * 256 + static_cast<std::size_t>(col)] =
                    static_cast<std::int32_t>(256 * 100 + std::llabs(m - 1000) * 205 + (((n % 4000) + 4000) % 4000) * 26);
            }
        return tiles_.emplace(key, std::move(tile)).first->second.get();
    }

private:
    std::mutex mutex_;
    std::map<g2m::TileKey, std::unique_ptr<g2m::HeightTile>> tiles_;
};

const g2m::HeightTile* ridge_lookup(void* ctx, const g2m::TileKey& key) { return static_cast<RidgeStore*>(ctx)->get(key); }

constexpr double kE0 = 464000.0;
constexpr double kN0 = 5559000.0;

std::shared_ptr<const std::vector<g2m::RoadSegment>> roads_fn(void*, const g2m::TileKey&) {
    static const auto segs = [] {
        std::vector<g2m::RoadSegment> v(2);
        v[0].a = {464000000 - 300000, 5559000000};
        v[0].b = {464000000 + 300000, 5559000000 + 100000};
        v[0].half_width_mm = 4000;
        v[0].land_class = g2m::LandClass::PavedRoad;
        v[0].rank = 7;
        v[0].way_id = 1;
        v[1].a = {464000000 - 50000, 5559000000 - 250000};
        v[1].b = {464000000 + 80000, 5559000000 + 300000};
        v[1].half_width_mm = 2500;
        v[1].land_class = g2m::LandClass::UnpavedRoad;
        v[1].rank = 1;
        v[1].way_id = 2;
        return std::make_shared<const std::vector<g2m::RoadSegment>>(std::move(v));
    }();
    return segs;
}

std::uint64_t fnv(std::uint64_t h, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        h ^= (v >> (8 * i)) & 0xff;
        h *= 0x100000001b3ull;
    }
    return h;
}

struct Hashes {
    std::size_t chunks = 0;
    std::uint64_t rgba = 0xcbf29ce484222325ull;
    std::size_t road_vertices = 0;
};

Hashes colours(const rg::RoadSurfaceMap& surfaces, bool through_hook = false) {
    RidgeStore store;
    g2m::mesh::LodParams params;
    params.zone = g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North};
    params.range0_m = 64.0;
    params.max_level = 3;
    params.max_distance_m = 700.0;
    g2m::world::ClassLookup classes;
    classes.fn = &roads_fn;
    std::vector<rg::RenderChunk> out;
    if (through_hook) {
        // geo2map's build with the game's colour hook, the way WorldTerrain::view_source() sets it up.
        g2m::world::ViewSource source;
        source.params = params;
        source.e0 = kE0;
        source.n0 = kN0;
        source.lookup = &ridge_lookup;
        source.ctx = &store;
        source.class_lookup = classes;
        source.colour = rg::chunk_colour_hook(surfaces);
        g2m::world::build_static_view(0.0, 0.0, source, 2, out);
    } else {
        rg::build_static_view_from_lookup(0.0, 0.0, params, kE0, kN0, &ridge_lookup, &store, 3, out, classes, surfaces);
    }
    Hashes h;
    h.chunks = out.size();
    for (const auto& c : out) {
        h.rgba = fnv(h.rgba, c.rgba.size());
        for (auto px : c.rgba) h.rgba = fnv(h.rgba, px);
        for (auto cls : c.mesh.land_class)
            h.road_vertices += cls == static_cast<std::uint8_t>(g2m::LandClass::PavedRoad) ||
                               cls == static_cast<std::uint8_t>(g2m::LandClass::UnpavedRoad);
    }
    return h;
}

} // namespace

TEST_CASE("terrain chunk colours are pinned", "[world_terrain][terrain_colours]") {
    const Hashes standard = colours(rg::RoadSurfaceMap{});
    CHECK(standard.chunks == 62);
    CHECK(standard.road_vertices > 0);
    CHECK(standard.rgba == 0xec6138b77a54c13full);
    rg::RoadSurfaceMap other;
    other.paved = "concrete"; // an unknown name falls through to the height ramp
    other.unpaved = "asphalt";
    const Hashes renamed = colours(other);
    CHECK(renamed.chunks == standard.chunks);
    CHECK(renamed.rgba == 0x227269903ab9a087ull);
    CHECK(colours(rg::RoadSurfaceMap{}, true).rgba == standard.rgba);
    CHECK(colours(other, true).rgba == renamed.rgba);
}

TEST_CASE("terrain colour table: road classes by surface name, everything else the ramp", "[terrain_colours]") {
    const auto paved = static_cast<std::size_t>(g2m::LandClass::PavedRoad);
    const auto unpaved = static_cast<std::size_t>(g2m::LandClass::UnpavedRoad);
    const rg::TerrainColourTable table = rg::make_terrain_colour_table(rg::RoadSurfaceMap{});
    CHECK(table.set[paved]);
    CHECK(table.set[unpaved]);
    CHECK(table.rgba[paved] != table.rgba[unpaved]);
    std::size_t entries = 0;
    for (const bool set : table.set) entries += set ? 1 : 0;
    CHECK(entries == 2);
    rg::RoadSurfaceMap odd;
    odd.paved = "concrete";
    CHECK_FALSE(rg::make_terrain_colour_table(odd).set[paved]);
}
