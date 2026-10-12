#include "rg/flat_ground.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace rg {

ps::MeshShape flat_ground_mesh(double half_x, double half_y, double cell_m) {
    if (!(half_x > 0.0) || !(half_y > 0.0) || !(cell_m > 0.0)) throw std::invalid_argument("flat_ground_mesh: extents and cell size must be positive");
    const double cell = std::min(cell_m, kMaxFlatGroundCellM);
    const auto nx = static_cast<std::uint32_t>(std::max(1.0, std::ceil(2.0 * half_x / cell)));
    const auto ny = static_cast<std::uint32_t>(std::max(1.0, std::ceil(2.0 * half_y / cell)));
    ps::MeshShape shape;
    shape.vertices.reserve(static_cast<std::size_t>(nx + 1) * (ny + 1));
    for (std::uint32_t j = 0; j <= ny; ++j) {
        const double y = -half_y + 2.0 * half_y * static_cast<double>(j) / static_cast<double>(ny);
        for (std::uint32_t i = 0; i <= nx; ++i) {
            const double x = -half_x + 2.0 * half_x * static_cast<double>(i) / static_cast<double>(nx);
            shape.vertices.push_back(ps::Vec3{x, y, 0.0});
        }
    }
    shape.indices.reserve(static_cast<std::size_t>(nx) * ny * 6);
    const auto vid = [nx](std::uint32_t i, std::uint32_t j) { return j * (nx + 1) + i; };
    for (std::uint32_t j = 0; j < ny; ++j) {
        for (std::uint32_t i = 0; i < nx; ++i) {
            const std::uint32_t a = vid(i, j), b = vid(i + 1, j), c = vid(i + 1, j + 1), d = vid(i, j + 1);
            shape.indices.insert(shape.indices.end(), {a, b, c, a, c, d}); // CCW seen from +z
        }
    }
    return shape; // triangle_surface_ids empty: kInvalidSurfaceId everywhere, like the untagged box
}

} // namespace rg
