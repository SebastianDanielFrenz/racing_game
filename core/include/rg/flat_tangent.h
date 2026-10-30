#pragma once
// Tangents for an unindexed triangle mesh, where every triangle has its own three vertices (the building tiles).
// One triangle has one texture gradient, so its tangent comes straight from its edges and UVs, with no vertex welding
// and no reads from the renderer. On such a mesh this matches SurfaceTool.generate_tangents() (MikkTSpace), which
// game/scripts/terrain_port_test.gd checks on a real tile.
//
// sign is the fourth tangent component: binormal = cross(normal, tangent) * sign. The binormal points against the
// direction of growing v, the convention of Godot's own primitives (a plane in the XZ plane is (1, 0, 0, +1)).
#include <array>
#include <cmath>

namespace rg {

struct UvGradient {
    std::array<double, 3> dp_du{};
    std::array<double, 3> dp_dv{};
    bool valid = false; // false: the triangle's UVs have no area, so there is no gradient
};

struct VertexTangent {
    std::array<float, 3> tangent{};
    float sign = 1.0f;
};

namespace flat_tangent_detail {
inline double dot(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
} // namespace flat_tangent_detail

inline UvGradient triangle_uv_gradient(const std::array<float, 3>& p0, const std::array<float, 3>& p1,
                                       const std::array<float, 3>& p2, const std::array<float, 2>& uv0,
                                       const std::array<float, 2>& uv1, const std::array<float, 2>& uv2) {
    UvGradient out;
    const double du1 = static_cast<double>(uv1[0]) - uv0[0], dv1 = static_cast<double>(uv1[1]) - uv0[1];
    const double du2 = static_cast<double>(uv2[0]) - uv0[0], dv2 = static_cast<double>(uv2[1]) - uv0[1];
    const double area = du1 * dv2 - du2 * dv1;
    if (std::abs(area) < 1e-12) return out;
    for (int i = 0; i < 3; ++i) {
        const double e1 = static_cast<double>(p1[i]) - p0[i], e2 = static_cast<double>(p2[i]) - p0[i];
        out.dp_du[i] = (dv2 * e1 - dv1 * e2) / area;
        out.dp_dv[i] = (du1 * e2 - du2 * e1) / area;
    }
    out.valid = true;
    return out;
}

inline VertexTangent vertex_tangent(const UvGradient& gradient, const std::array<float, 3>& normal) {
    using flat_tangent_detail::cross;
    using flat_tangent_detail::dot;
    const std::array<double, 3> n{normal[0], normal[1], normal[2]};
    std::array<double, 3> t{};
    double length = 0.0;
    if (gradient.valid) {
        const double along = dot(gradient.dp_du, n);
        for (int i = 0; i < 3; ++i) t[i] = gradient.dp_du[i] - n[i] * along;
        length = std::sqrt(dot(t, t));
    }
    if (length < 1e-12) {
        // No usable texture direction: any unit vector across the normal keeps the shader's frame finite.
        const std::array<double, 3> axis = std::abs(n[0]) < 0.9 ? std::array<double, 3>{1.0, 0.0, 0.0}
                                                                : std::array<double, 3>{0.0, 1.0, 0.0};
        const double along = dot(axis, n);
        for (int i = 0; i < 3; ++i) t[i] = axis[i] - n[i] * along;
        length = std::sqrt(dot(t, t));
    }
    VertexTangent out;
    for (int i = 0; i < 3; ++i) out.tangent[i] = static_cast<float>(t[i] / length);
    if (gradient.valid && dot(gradient.dp_dv, cross(n, t)) > 0.0) out.sign = -1.0f;
    return out;
}

} // namespace rg
