// rg/surface_kinds.h - geo2map SurfaceKind -> data/surfaces/surfaces.json surface NAME (owner O-2, S1).
//
// The game owns its surface table (data/surfaces/surfaces.json). Every drivable geo2map surface kind has its OWN
// named entry there (no mapping onto asphalt or dirt), and the names equal the class names of physics_sim's
// data/surface_conditions/default.json, so the wet-grip model (weather W6) applies to every one of them.
//
// Nothing consumes this table in physics yet: the bridge's road mode only knows paved/unpaved/off_road
// (RoadSurfaceMap). The carve switch (S12, class mode, request G-3) builds its SurfaceKind -> SurfaceId lookup from
// surface_name_for_kind() through the session's SurfaceTable.
#pragma once

#include "g2m/layer/classifier.h"

#include <array>
#include <string_view>

namespace rg {

inline constexpr std::array<g2m::SurfaceKind, 12> kAllSurfaceKinds{
    g2m::SurfaceKind::Unknown, g2m::SurfaceKind::Asphalt, g2m::SurfaceKind::Concrete, g2m::SurfaceKind::PavingStones,
    g2m::SurfaceKind::Sett,    g2m::SurfaceKind::Cobblestone, g2m::SurfaceKind::Gravel, g2m::SurfaceKind::Compacted,
    g2m::SurfaceKind::Dirt,    g2m::SurfaceKind::Grass,   g2m::SurfaceKind::Sand,     g2m::SurfaceKind::Water};

// Total over every SurfaceKind value (an out-of-range value reads as Unknown).
//   Unknown -> "asphalt": a classifier with no facts on a road cell. An unclassified OFF-road cell is decided by its
//     land class (RoadSurfaceMap::off_road), which S12 consults before this table.
//   Water -> "low_mu": PLACEHOLDER. Water is not drivable in v1; the slipperiest entry keeps a car that somehow
//     reaches it from gripping like dry land. S12 may replace it.
[[nodiscard]] constexpr std::string_view surface_name_for_kind(g2m::SurfaceKind kind) {
    switch (kind) {
        case g2m::SurfaceKind::Asphalt: return "asphalt";
        case g2m::SurfaceKind::Concrete: return "concrete";
        case g2m::SurfaceKind::PavingStones: return "paving_stones";
        case g2m::SurfaceKind::Sett: return "sett";
        case g2m::SurfaceKind::Cobblestone: return "cobblestone";
        case g2m::SurfaceKind::Gravel: return "gravel";
        case g2m::SurfaceKind::Compacted: return "compacted";
        case g2m::SurfaceKind::Dirt: return "dirt";
        case g2m::SurfaceKind::Grass: return "grass";
        case g2m::SurfaceKind::Sand: return "sand";
        case g2m::SurfaceKind::Water: return "low_mu";
        case g2m::SurfaceKind::Unknown: break;
    }
    return "asphalt";
}

} // namespace rg
