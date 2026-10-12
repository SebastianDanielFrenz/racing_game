// flat_ground.h - the one place racing_game builds a flat static test/flat-mode ground.
//
// Why a helper: a wheel CylinderShape cast (WheelInstance::measure_contact) against a very large UNTAGGED static box or a
// mesh with huge triangles gives wrong results near the shape's edges (vault PHYS-047): one-substep phantom fraction-0 hits
// that jump a wheel to full bump (hyper rear compression 0.031 -> 0.105 m, fz ~550 kN - the "launch hop"). A box with
// half-extent <= 1000 m is clean (the SessionConfig default); anything larger must be a mesh whose triangle edges stay
// at tens of metres. flat_ground_mesh() builds exactly that: a tessellated z = 0 plane, cells <= kMaxFlatGroundCellM,
// untagged (kInvalidSurfaceId, like the box), body-local z = 0 so the body pose is the origin.
#pragma once

#include "ps/backend/shape_desc.h"

namespace rg {

// Triangle-edge limit the probe in tests/unit/test_flat_ground.cpp checks: cells are squares of this side or smaller
// (the diagonal is then sqrt(2) times it, ~28 m, which PHYS-047 measured clean; its 20 m-cell mesh gave 0 bad casts).
inline constexpr double kMaxFlatGroundCellM = 20.0;
// A static box ground at or below this half-extent needs no tessellation.
inline constexpr double kMaxFlatGroundBoxHalfExtentM = 1000.0;

// Rectangle [-half_x, half_x] x [-half_y, half_y] at z = 0, split into square-ish cells of side <= cell_m (clamped to
// kMaxFlatGroundCellM), two triangles each, counter-clockwise seen from +z (upward normals). Throws std::invalid_argument
// for a non-positive extent or cell size.
[[nodiscard]] ps::MeshShape flat_ground_mesh(double half_x, double half_y, double cell_m = kMaxFlatGroundCellM);

} // namespace rg
