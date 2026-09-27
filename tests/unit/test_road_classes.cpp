// test_road_classes.cpp — rg::road_classes coverage (roads_plan.md R-2
// acceptance: "roads appear at L0, the level filter works, and a null
// lookup is byte-identical"). Synthetic RoadSegment lists only - no
// g2m::TileStore/Server/OSM-decode machinery (this project links
// geo2map_engine with G2M_BUILD_IMPORT/TESTS/APPS/FUZZERS all OFF, same
// "testable seam" precedent as test_world_terrain.cpp's own SyntheticStore).
#include "rg/road_classes.h"

#include "g2m/layer/classifier.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace {

using g2m::geo::Hemisphere;
using g2m::geo::UtmZone;

constexpr UtmZone kZone{32, Hemisphere::North};

// Ignores `key` entirely and always returns the same segment list - every
// test here places its segment(s) so that whichever g2m.src.osm tile(s)
// src_osm_tiles_for_chunk asks for, the answer is the same synthetic world.
std::shared_ptr<const std::vector<g2m::RoadSegment>> synthetic_road_lookup(void* ctx, const g2m::TileKey&) {
    return *static_cast<const std::shared_ptr<const std::vector<g2m::RoadSegment>>*>(ctx);
}

rg::ClassLookup make_lookup(const std::shared_ptr<const std::vector<g2m::RoadSegment>>& segments) {
    return rg::ClassLookup{&synthetic_road_lookup, const_cast<void*>(static_cast<const void*>(&segments))};
}

std::uint8_t at(const g2m::mesh::ClassWindow& w, int i, int j) {
    return w.c[static_cast<std::size_t>(j) * static_cast<std::size_t>(g2m::mesh::kChunkVerts) +
                static_cast<std::size_t>(i)];
}

constexpr std::uint8_t kUnknown = static_cast<std::uint8_t>(g2m::LandClass::Unknown);
constexpr std::uint8_t kPaved = static_cast<std::uint8_t>(g2m::LandClass::PavedRoad);

} // namespace

TEST_CASE("road_raster_params_for_level: matches roads_plan.md's level policy", "[terrain]") {
    const g2m::RoadStyle& style = g2m::RoadStyle::default_style();
    const g2m::HighwayStyleEntry* tertiary = style.find_highway("tertiary");
    const g2m::HighwayStyleEntry* primary = style.find_highway("primary");
    REQUIRE(tertiary != nullptr);
    REQUIRE(primary != nullptr);

    const auto l0 = rg::road_raster_params_for_level(0);
    REQUIRE(l0.has_value());
    CHECK(l0->min_rank == 0);
    CHECK(l0->min_half_width_mm == 0);

    const auto l1 = rg::road_raster_params_for_level(1);
    REQUIRE(l1.has_value());
    CHECK(l1->min_rank == 0);
    CHECK(l1->min_half_width_mm == 0);

    const auto l2 = rg::road_raster_params_for_level(2);
    REQUIRE(l2.has_value());
    CHECK(l2->min_rank == tertiary->rank);
    CHECK(l2->min_half_width_mm == 0);

    const auto l3 = rg::road_raster_params_for_level(3);
    REQUIRE(l3.has_value());
    CHECK(l3->min_rank == primary->rank);
    CHECK(l3->min_half_width_mm == 6000); // 0.75 * (2^3 * 1000 mm) = 6000 mm

    CHECK_FALSE(rg::road_raster_params_for_level(4).has_value());
    CHECK_FALSE(rg::road_raster_params_for_level(6).has_value());
}

TEST_CASE("road_class_lattice: matches terrain_chunk.h's vertex-placement formula", "[terrain]") {
    {
        const g2m::mesh::ChunkKey key{kZone, 0, 0, 0};
        const g2m::Lattice lattice = rg::road_class_lattice(key);
        CHECK(lattice.origin_e_mm == 500);
        CHECK(lattice.origin_n_mm == 500);
        CHECK(lattice.spacing_mm == 1000);
        CHECK(lattice.nx == g2m::mesh::kChunkVerts);
        CHECK(lattice.ny == g2m::mesh::kChunkVerts);
    }
    {
        // Level 2, cx=3, cy=-1: spacing = 4000 mm; origin = (64*cx*spacing + 500, 64*cy*spacing + 500).
        const g2m::mesh::ChunkKey key{kZone, 2, 3, -1};
        const g2m::Lattice lattice = rg::road_class_lattice(key);
        CHECK(lattice.spacing_mm == 4000);
        CHECK(lattice.origin_e_mm == 64 * 3 * 4000 + 500);
        CHECK(lattice.origin_n_mm == 64 * (-1) * 4000 + 500);
    }
}

TEST_CASE("src_osm_tiles_for_chunk: one tile fully inside, several when straddling a boundary", "[terrain]") {
    // Level 0 chunk (0,0): footprint [0,64) x [0,64) m, well inside g2m.src.osm
    // tile (level 2, x=0, y=0) = [0, 1024) x [0, 1024) m.
    {
        const g2m::mesh::ChunkKey key{kZone, 0, 0, 0};
        const std::vector<g2m::TileKey> tiles = rg::src_osm_tiles_for_chunk(key);
        REQUIRE(tiles.size() == 1);
        CHECK(tiles[0].level == 2);
        CHECK(tiles[0].x == 0);
        CHECK(tiles[0].y == 0);
    }
    // Level 3 chunk (cx=1, cy=0): 512 m side, footprint [512, 1024) x [0, 512),
    // whose east edge sits exactly on the osm tile x=1024 boundary - the
    // inclusive floor_div(e_max, 1024) range therefore covers both x=0 and
    // x=1, y stays a single row.
    {
        const g2m::mesh::ChunkKey key{kZone, 3, 1, 0};
        const std::vector<g2m::TileKey> tiles = rg::src_osm_tiles_for_chunk(key);
        CHECK(tiles.size() == 2);
        for (const g2m::TileKey& t : tiles) {
            CHECK(t.level == 2);
            CHECK(t.y == 0);
        }
    }
}

TEST_CASE("rasterize_chunk_road_classes: a road segment appears at level 0", "[terrain]") {
    // A short paved road along N = 32.5 m, E in [10, 54] m - covers the
    // level-0 chunk (0,0)'s vertex (32, 32) (absolute (32.5, 32.5) m, this
    // chunk's own vertex-placement formula) and stays well clear of the
    // chunk's far corners.
    auto segments = std::make_shared<const std::vector<g2m::RoadSegment>>(std::vector<g2m::RoadSegment>{
        g2m::RoadSegment{{10000, 32500}, {54000, 32500}, /*half_width_mm=*/2000, g2m::LandClass::PavedRoad,
                         g2m::SurfaceKind::Asphalt, /*rank=*/9},
    });
    const rg::ClassLookup lookup = make_lookup(segments);

    const g2m::mesh::ChunkKey key{kZone, 0, 0, 0};
    g2m::mesh::ClassWindow classes;
    rg::rasterize_chunk_road_classes(key, lookup, classes);

    CHECK(at(classes, 32, 32) == kPaved);
    CHECK(at(classes, 0, 0) == kUnknown);
    CHECK(at(classes, 64, 64) == kUnknown);
}

TEST_CASE("rasterize_chunk_road_classes: the level filter works", "[terrain]") {
    const g2m::RoadStyle& style = g2m::RoadStyle::default_style();
    const int residential_rank = style.find_highway("residential")->rank;
    const int tertiary_rank = style.find_highway("tertiary")->rank;
    const int primary_rank = style.find_highway("primary")->rank;
    REQUIRE(residential_rank < tertiary_rank);
    REQUIRE(tertiary_rank < primary_rank);

    // Every chunk below is (cx=0, cy=0) at its own level, and every level's
    // own vertex-placement formula puts SOME integer vertex exactly at
    // absolute (32.5, 32.5) m (level 0 -> (32,32), level 1 -> (16,16),
    // level 2 -> (8,8), level 3 -> (4,4)) - the same physical point the
    // segment below sits on, so every level's "does this road show up
    // here" check lands on an exact vertex, not a tolerance.
    auto make_segments_at_rank = [](int rank) {
        return std::make_shared<const std::vector<g2m::RoadSegment>>(std::vector<g2m::RoadSegment>{
            g2m::RoadSegment{{10000, 32500}, {54000, 32500}, 2000, g2m::LandClass::PavedRoad, g2m::SurfaceKind::Asphalt,
                             rank},
        });
    };

    auto vertex_for_level = [](int level) -> int {
        switch (level) {
            case 0: return 32;
            case 1: return 16;
            case 2: return 8;
            case 3: return 4;
            default: return 0;
        }
    };

    // Residential-rank road: visible at L0/L1, filtered out at L2/L3/L4.
    {
        auto segments = make_segments_at_rank(residential_rank);
        const rg::ClassLookup lookup = make_lookup(segments);
        for (int level : {0, 1}) {
            g2m::mesh::ClassWindow classes;
            rg::rasterize_chunk_road_classes(g2m::mesh::ChunkKey{kZone, static_cast<std::uint8_t>(level), 0, 0}, lookup,
                                             classes);
            const int v = vertex_for_level(level);
            CHECK(at(classes, v, v) == kPaved);
        }
        for (int level : {2, 3, 4}) {
            g2m::mesh::ClassWindow classes;
            rg::rasterize_chunk_road_classes(g2m::mesh::ChunkKey{kZone, static_cast<std::uint8_t>(level), 0, 0}, lookup,
                                             classes);
            const int v = vertex_for_level(level);
            CHECK(at(classes, v, v) == kUnknown);
        }
    }

    // Tertiary-rank road: visible at L0/L1/L2, filtered out at L3/L4 (below
    // L3's own "primary and above" floor).
    {
        auto segments = make_segments_at_rank(tertiary_rank);
        const rg::ClassLookup lookup = make_lookup(segments);
        for (int level : {0, 1, 2}) {
            g2m::mesh::ClassWindow classes;
            rg::rasterize_chunk_road_classes(g2m::mesh::ChunkKey{kZone, static_cast<std::uint8_t>(level), 0, 0}, lookup,
                                             classes);
            const int v = vertex_for_level(level);
            CHECK(at(classes, v, v) == kPaved);
        }
        g2m::mesh::ClassWindow classes3;
        rg::rasterize_chunk_road_classes(g2m::mesh::ChunkKey{kZone, 3, 0, 0}, lookup, classes3);
        CHECK(at(classes3, vertex_for_level(3), vertex_for_level(3)) == kUnknown);
    }

    // Primary-rank road: visible everywhere including L3.
    {
        auto segments = make_segments_at_rank(primary_rank);
        const rg::ClassLookup lookup = make_lookup(segments);
        for (int level : {0, 1, 2, 3}) {
            g2m::mesh::ClassWindow classes;
            rg::rasterize_chunk_road_classes(g2m::mesh::ChunkKey{kZone, static_cast<std::uint8_t>(level), 0, 0}, lookup,
                                             classes);
            const int v = vertex_for_level(level);
            CHECK(at(classes, v, v) == kPaved);
        }
        // L4 is disabled outright, whatever the rank.
        g2m::mesh::ClassWindow classes4;
        rg::rasterize_chunk_road_classes(g2m::mesh::ChunkKey{kZone, 4, 0, 0}, lookup, classes4);
        CHECK(at(classes4, 0, 0) == kUnknown);
    }
}

TEST_CASE("rasterize_chunk_road_classes: a null lookup is byte-identical to all-Unknown", "[terrain]") {
    auto segments = std::make_shared<const std::vector<g2m::RoadSegment>>(std::vector<g2m::RoadSegment>{
        g2m::RoadSegment{{10000, 32500}, {54000, 32500}, 2000, g2m::LandClass::PavedRoad, g2m::SurfaceKind::Asphalt, 9},
    });
    (void)segments; // deliberately unused: lookup.fn is null below, so nothing ever calls it

    const rg::ClassLookup null_lookup{}; // fn == nullptr
    const g2m::mesh::ChunkKey key{kZone, 0, 0, 0};

    g2m::mesh::ClassWindow classes;
    // Pre-fill with a non-zero pattern so the test cannot pass by accident
    // (a default-constructed ClassWindow is already all-zero).
    classes.c.fill(static_cast<std::uint8_t>(g2m::LandClass::TreeCover));

    rg::rasterize_chunk_road_classes(key, null_lookup, classes);

    CHECK(classes.c == g2m::mesh::ClassWindow{}.c);
}
