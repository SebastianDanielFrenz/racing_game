// test_flat_ground.cpp - rg::flat_ground_mesh and SessionConfig::ground_tessellated (vault PHYS-047: huge flat boxes and
// huge triangles give phantom wheel-cast hits).
#include "rg/flat_ground.h"
#include "rg/session.h"

#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace {
rg::SessionConfig make_config() {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/surfaces/surfaces.json";
    return config;
}
} // namespace

TEST_CASE("flat_ground_mesh: small cells, upward normals, full coverage", "[session][flat_ground]") {
    const ps::MeshShape m = rg::flat_ground_mesh(1000.0, 130.0);
    REQUIRE(m.indices.size() % 3 == 0);
    CHECK(m.triangle_surface_ids.empty()); // untagged, like the box
    double longest = 0.0, area = 0.0, min_x = 1e30, max_x = -1e30, min_y = 1e30, max_y = -1e30;
    for (const auto& v : m.vertices) {
        CHECK(v.z == 0.0);
        min_x = std::min(min_x, v.x), max_x = std::max(max_x, v.x), min_y = std::min(min_y, v.y), max_y = std::max(max_y, v.y);
    }
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const ps::Vec3 a = m.vertices[m.indices[t]], b = m.vertices[m.indices[t + 1]], c = m.vertices[m.indices[t + 2]];
        longest = std::max({longest, (b - a).length(), (c - b).length(), (a - c).length()});
        const ps::Vec3 n = ps::cross(b - a, c - a);
        CHECK(n.z > 0.0); // CCW from +z
        area += 0.5 * n.z;
    }
    CHECK(longest <= std::sqrt(2.0) * rg::kMaxFlatGroundCellM + 1e-9);
    CHECK(min_x == -1000.0);
    CHECK(max_x == 1000.0);
    CHECK(min_y == -130.0);
    CHECK(max_y == 130.0);
    CHECK(area == Catch::Approx(2000.0 * 260.0));
    CHECK_THROWS_AS(rg::flat_ground_mesh(0.0, 10.0), std::invalid_argument);
    // a cell size above the limit is clamped
    CHECK(rg::flat_ground_mesh(100.0, 100.0, 500.0).indices.size() == rg::flat_ground_mesh(100.0, 100.0, 20.0).indices.size());
}

TEST_CASE("Session: a tessellated ground carries the car like the box and a huge box is refused", "[session][flat_ground]") {
    auto settle_z = [](rg::SessionConfig config) {
        rg::Session session(config);
        for (int i = 0; i < 480; ++i) session.step();
        const auto& w = session.world();
        return std::pair{w.get_pose(session.chassis_body()).position.z, w.wheel_telemetry(session.vehicle_id(), 2).compression};
    };
    const auto box = settle_z(make_config());
    rg::SessionConfig tess = make_config();
    tess.ground_tessellated = true;
    const auto mesh = settle_z(tess);
    CHECK(mesh.first == Catch::Approx(box.first).margin(2e-3));
    CHECK(mesh.second == Catch::Approx(box.second).margin(2e-3));

    rg::SessionConfig huge = make_config();
    huge.ground_half_extent_m = 5000.0;
    CHECK_THROWS_AS(rg::Session(huge), std::invalid_argument);
    huge.ground_tessellated = true;
    CHECK_NOTHROW(rg::Session(huge));
}
