// test_g2m_ps_bridge_link_smoke.cpp — proves rg_core actually LINKS
// g2m_phys and g2m_ps_bridge (R2.2 R1), not just compiles against their
// headers, and pins the F=1 / pool=49 decision g2m_ps_bridge's own header
// comment (bridges/physics_sim/include/g2m/ps_bridge/g2m_terrain_source.h)
// records for make_terrain_config(400.0, 1) - mirrors
// test_g2m_link_smoke.cpp's own "prove the link, not just the compile"
// precedent for g2m_core.
//
// No real-data access: PhysicsTileGrid needs only a SessionFrame (a bare
// UTM zone/origin), never a TileStore/Server/on-disk store.
#include "g2m/phys/physics_grid.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "g2m/core/geo/session_frame.h"

#include <catch2/catch_test_macros.hpp>

using g2m::geo::Hemisphere;
using g2m::geo::SessionFrame;
using g2m::geo::UtmZone;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;

TEST_CASE("make_terrain_config: pool 49 / 1 fill per tick at r=400m (rg_core links g2m_ps_bridge)",
          "[g2m_link_smoke]") {
    const ps::terrain::TerrainConfig cfg = g2m::ps_bridge::make_terrain_config(/*interest_radius_m=*/400.0,
                                                                                /*fills_per_tick=*/1);

    // (2*ceil(400/255)+3)^2 = (2*2+3)^2 = 49 - the plan's own worked
    // example (racing_game/PLAN.md-adjacent r22_plan.md section 1b),
    // confirmed as-built in geo2map_engine PLAN.md 8.5.
    CHECK(cfg.max_resident_tiles == 49);
    CHECK(cfg.max_tile_fills_per_tick == 1);
    // R2.2 has no static geometry in terrain mode (section 1b).
    CHECK_FALSE(cfg.suppress_terrain_under_static_geometry);
    CHECK_FALSE(cfg.precompute_suppression_at_load);
}

TEST_CASE("PhysicsTileGrid: pack/unpack key round-trip (rg_core links g2m_phys)", "[g2m_link_smoke]") {
    constexpr UtmZone k32N{32, Hemisphere::North};
    SessionFrame frame(k32N, /*e0_m=*/474000, /*n0_m=*/5560000);
    PhysicsTileGrid grid(frame); // default kPhysSamples = 256

    CHECK(grid.samples() == 256);
    CHECK(grid.tile_size_m() == 255.0);

    const PhysTileIndex index{-3, 7};
    const std::int64_t packed = PhysicsTileGrid::pack(index);
    const PhysTileIndex round_tripped = PhysicsTileGrid::unpack(packed);

    CHECK(round_tripped.ix == index.ix);
    CHECK(round_tripped.iy == index.iy);
    CHECK(round_tripped == index);
}
