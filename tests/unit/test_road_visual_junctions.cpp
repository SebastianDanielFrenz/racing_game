// Junction surfaces in the road visual (geo2map roads.geom encoding 7 rings, design D44 kerb returns):
// rg::add_junction_surfaces drapes each served road bed ring over the ground, cut on the terrain lattice lines.
#include "rg/road_visual.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <map>
#include <utility>

namespace {
constexpr double kPi = 3.14159265358979323846;
const g2m::TileKey kKey = *g2m::tile_key_at(g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North}, 2, 466100.0, 5560100.0);

// An X junction's road bed: a 7 m wide cross with 6 m kerb returns in all four block corners (mm, absolute UTM).
g2m::RoadJunction kerb_cross(double cx, double cy) {
    g2m::RoadJunction j;
    j.nodes = {1};
    const double w = 3.5, r = 6.0, arm = w + r + 2.0;
    std::vector<std::pair<double, double>> pts;
    for (int q = 0; q < 4; ++q) {
        const double a = q * kPi / 2, ca = std::cos(a), sa = std::sin(a);
        const auto rot = [&](double x, double y) { pts.push_back({cx + x * ca - y * sa, cy + x * sa + y * ca}); };
        rot(arm, -w); rot(arm, w);                                        // trim of the arm along +x
        for (int i = 0; i <= 12; ++i) {                                     // kerb return into the arm along +y
            const double ang = 1.5 * kPi - (kPi / 2) * i / 12.0;            // centre (w + r, w + r), from (w + r, w) to (w, w + r)
            rot(w + r + r * std::cos(ang), w + r + r * std::sin(ang));
        }
    }
    for (const auto& [x, y] : pts) j.rings[0].vertices.push_back({std::llround(x * 1000), std::llround(y * 1000)});
    j.rings[1] = j.rings[0]; j.rings[2] = j.rings[0];
    return j;
}
double ring_area(const g2m::JunctionRing& r) {
    double a = 0;
    for (std::size_t i = 0; i < r.vertices.size(); ++i) {
        const auto& p = r.vertices[i]; const auto& q = r.vertices[(i + 1) % r.vertices.size()];
        a += static_cast<double>(p.x) * static_cast<double>(q.y) - static_cast<double>(q.x) * static_cast<double>(p.y);
    }
    return a / 2e6;
}
double mesh_area(const rg::RoadVisualMesh& m) {
    double a = 0;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const auto& p = m.vertices[static_cast<std::size_t>(m.indices[i])], &q = m.vertices[static_cast<std::size_t>(m.indices[i + 1])],
                    &r = m.vertices[static_cast<std::size_t>(m.indices[i + 2])];
        a += ((q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)) / 2.0;
    }
    return a;
}
}

TEST_CASE("Junction surface covers the kerb-return ring exactly, on the terrain lattice, watertight", "[road_visual][junction]") {
    g2m::RoadGeomTile tile;
    const double cx = kKey.min_easting() + 300.3, cy = kKey.min_northing() + 400.7;
    tile.junctions.push_back(kerb_cross(cx, cy));
    const auto ground = [](double e, double n) -> std::optional<double> { return 100.0 + 0.01 * e - 0.02 * n; };
    rg::RoadVisualMesh mesh;
    rg::add_junction_surfaces(mesh, tile, kKey, ground, 0.018);
    REQUIRE_FALSE(mesh.indices.empty());
    // Area: every triangle counter-clockwise and the sum equals the ring's area.
    CHECK(mesh_area(mesh) == Catch::Approx(ring_area(tile.junctions[0].rings[0])).epsilon(1e-4));
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3) {
        const auto& p = mesh.vertices[static_cast<std::size_t>(mesh.indices[i])], &q = mesh.vertices[static_cast<std::size_t>(mesh.indices[i + 1])],
                    &r = mesh.vertices[static_cast<std::size_t>(mesh.indices[i + 2])];
        CHECK((q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x) > -1e-6f);
        // No triangle crosses a lattice line (x.5 m): all three corners lie in one cell (closed).
        for (int axis = 0; axis < 2; ++axis) {
            const double o = axis ? kKey.min_northing() : kKey.min_easting();
            const auto c = [&](const rg::RoadVisualVertex& v) { return (axis ? v.y : v.x) + o; };
            const double lo = std::min({c(p), c(q), c(r)}), hi = std::max({c(p), c(q), c(r)});
            CHECK(std::floor(lo - 0.5 + 1e-6) >= std::floor(hi - 0.5 - 1e-6));
        }
    }
    // Heights: the ground plus lift plus the junction offset, at every vertex.
    for (const auto& v : mesh.vertices) {
        const double e = v.x + kKey.min_easting(), n = v.y + kKey.min_northing();
        CHECK(v.z == Catch::Approx(100.0 + 0.01 * e - 0.02 * n + 0.018 + 0.004).margin(2e-3));
        CHECK(v.fade == 0.0f); CHECK(v.flags == 0.0f);
    }
    // Watertight: every edge is used by two triangles, except the boundary, whose length is the ring's perimeter.
    std::map<std::pair<std::int32_t, std::int32_t>, int> uses;
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
        for (int k = 0; k < 3; ++k) {
            const auto a = mesh.indices[i + static_cast<std::size_t>(k)], b = mesh.indices[i + static_cast<std::size_t>((k + 1) % 3)];
            ++uses[{std::min(a, b), std::max(a, b)}];
        }
    double boundary = 0, perimeter = 0;
    for (const auto& [e, n] : uses) {
        CHECK(n <= 2);
        if (n == 1) boundary += std::hypot(mesh.vertices[static_cast<std::size_t>(e.first)].x - mesh.vertices[static_cast<std::size_t>(e.second)].x,
                                           mesh.vertices[static_cast<std::size_t>(e.first)].y - mesh.vertices[static_cast<std::size_t>(e.second)].y);
    }
    const auto& rv = tile.junctions[0].rings[0].vertices;
    for (std::size_t i = 0; i < rv.size(); ++i)
        perimeter += std::hypot(static_cast<double>(rv[i].x - rv[(i + 1) % rv.size()].x), static_cast<double>(rv[i].y - rv[(i + 1) % rv.size()].y)) / 1000.0;
    CHECK(boundary == Catch::Approx(perimeter).epsilon(1e-3));
}

TEST_CASE("Junction surface skips declined junctions, bridge junctions and rings without ground", "[road_visual][junction]") {
    const double cx = kKey.min_easting() + 500, cy = kKey.min_northing() + 500;
    const auto flat = [](double, double) -> std::optional<double> { return 50.0; };
    g2m::RoadGeomTile tile;
    auto declined = kerb_cross(cx, cy); declined.decline = g2m::JunctionDecline::ClusterTooLarge;
    auto bridge = kerb_cross(cx, cy); bridge.flags = static_cast<std::uint16_t>(g2m::JunctionFlags::OnStructure);
    tile.junctions = {declined, bridge};
    rg::RoadVisualMesh mesh;
    rg::add_junction_surfaces(mesh, tile, kKey, flat, 0.0);
    CHECK(mesh.vertices.empty());
    // A hole in the ground (NoData) drops the whole patch, never a part of it.
    tile.junctions = {kerb_cross(cx, cy)};
    const auto holed = [&](double e, double n) -> std::optional<double> { if (e > cx + 5 && n > cy + 5) return {}; return 50.0; };
    rg::add_junction_surfaces(mesh, tile, kKey, holed, 0.0);
    CHECK(mesh.vertices.empty()); CHECK(mesh.indices.empty());
    rg::add_junction_surfaces(mesh, tile, kKey, flat, 0.0);
    CHECK_FALSE(mesh.indices.empty());
}
