#pragma once

#include "g2m/world/road_ribbons.h"

namespace rg {

// Layering of the draped road-visual surfaces of one tile (z-fighting at the Engelsruhe spawn, owner 2026-10-19).
//
// Every layer of a RoadRibbonTile is draped on the same ground by its own vertex sampling, so two layers that cover the
// same spot differ by whatever the two triangulations interpolate differently (measured 2..24 mm on the real tile, more
// than the 3-7 mm the layers are lifted apart): wherever they overlap, the depth test flips between them triangle by
// triangle. The rule that replaces "lift them a few millimetres apart and hope":
//
//   * Surface layers never overlap. In rank order verge (0, the gravel shoulder beside the road bed) < walkway/path
//     (1, cross-section sidewalk and cycle bands, standalone footways) < unpaved route strip (2, earth/gravel tracks)
//     < carriageway (3, the paved road ribbon) < junction patch (4, drawn over the ribbons it replaces; its overlay
//     paints the markings). A triangle of a lower rank is cut against every higher-ranked triangle that lies at
//     (nearly) the same height and keeps only the part nobody outranks. Higher always wins on the carriageway: a
//     footway crossing a road, a verge or a cycle band running into a junction patch is trimmed, not drawn on top.
//     Triangles at different heights (a footway under a bridge deck) never cover each other.
//   * Two carriageway ribbons that lie over each other at the same height (an on-ramp whose last metres end on the axis
//     of the through road it merges into, site 4 at the B8 / Main-Taunus-Zentrum) are ordered by lane count: the ribbon
//     with more lanes keeps the surface, the narrower one is cut. Equal lane counts are left alone.
//   * Decals (lane paint, turn arrows, junction overlay: flags 16 / 32) cannot be cut - they lie on the surface by
//     design - so they are lifted by kDecalLiftM, more than the interpolation mismatch above.
//
// Engine-neutral: pure mesh arithmetic, the renderer draws what comes out. Call once per freshly built tile.
void resolve_surface_layers(g2m::world::RoadRibbonTile& mesh);

// Metres a decal vertex is raised above the surface it is painted on (twice the worst decal-vs-surface drape mismatch
// measured on the real spawn tile: 11 mm on the current base, 9 mm on the older one).
inline constexpr double kDecalLiftM = 0.025;
// Triangles of different layers count as one surface when their mean heights differ by less than this (m).
inline constexpr double kSameSurfaceHeightM = 0.25;

} // namespace rg
