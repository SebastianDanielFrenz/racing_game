// test_g2m_link_smoke.cpp — proves rg_core actually LINKS a geo2map_engine
// symbol (PLAN.md R2.0), not just compiles against its headers.
// g2m::TileKey::packed()/TileKey::unpack() are declared in
// external/geo2map_engine/include/g2m/core/tile_key.h but DEFINED out of
// line in its own tile_key.cpp (part of the g2m_core static library, pulled
// in transitively via rg_core's PUBLIC link of g2m_layers -> g2m_core) - a
// header-only mistake (e.g. linking against the wrong/no g2m target) would
// fail at LINK time here with an unresolved external symbol, not at compile
// time, which is the whole point of this test existing separately from
// test_world_config.cpp (that one only exercises g2m::geo::UtmZone::valid(),
// a header-only inline function that proves nothing about linking).
#include "g2m/core/tile_key.h"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("g2m::TileKey packs and unpacks (rg_core links g2m_core)", "[g2m_link_smoke]") {
    g2m::geo::UtmZone zone{32, g2m::geo::Hemisphere::North};
    g2m::TileKey key{zone, /*level=*/0, /*x=*/1953, /*y=*/21582};

    REQUIRE(key.valid());

    const std::uint64_t packed = key.packed();
    const std::optional<g2m::TileKey> round_tripped = g2m::TileKey::unpack(packed);

    REQUIRE(round_tripped.has_value());
    CHECK(round_tripped->zone == key.zone);
    CHECK(round_tripped->level == key.level);
    CHECK(round_tripped->x == key.x);
    CHECK(round_tripped->y == key.y);
}
