#include "rg/flat_tangent.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>

using Catch::Approx;
using P = std::array<float, 3>;
using U = std::array<float, 2>;

static void check_tangent(const rg::VertexTangent& got, double x, double y, double z, float sign) {
    CHECK(got.tangent[0] == Approx(x).margin(1e-6));
    CHECK(got.tangent[1] == Approx(y).margin(1e-6));
    CHECK(got.tangent[2] == Approx(z).margin(1e-6));
    CHECK(got.sign == sign);
}

TEST_CASE("flat tangent: a plane in XZ matches Godot's primitive convention", "[buildings]") {
    // PlaneMesh: normal +Y, u along +X, v along +Z, tangent (1, 0, 0, +1).
    const auto gradient = rg::triangle_uv_gradient(P{0, 0, 0}, P{2, 0, 0}, P{0, 0, 3}, U{0, 0}, U{2, 0}, U{0, 3});
    REQUIRE(gradient.valid);
    check_tangent(rg::vertex_tangent(gradient, P{0, 1, 0}), 1, 0, 0, 1.0f);
}

TEST_CASE("flat tangent: vertex order does not change the result", "[buildings]") {
    const auto a = rg::triangle_uv_gradient(P{1, 2, 3}, P{4, 2, 3}, P{1, 2, 7}, U{0.1f, 0.2f}, U{1.1f, 0.2f}, U{0.1f, 1.2f});
    const auto b = rg::triangle_uv_gradient(P{1, 2, 3}, P{1, 2, 7}, P{4, 2, 3}, U{0.1f, 0.2f}, U{0.1f, 1.2f}, U{1.1f, 0.2f});
    const auto ta = rg::vertex_tangent(a, P{0, 1, 0}), tb = rg::vertex_tangent(b, P{0, 1, 0});
    check_tangent(ta, tb.tangent[0], tb.tangent[1], tb.tangent[2], tb.sign);
}

TEST_CASE("flat tangent: a mirrored texture flips the tangent, not the frame", "[buildings]") {
    const auto gradient = rg::triangle_uv_gradient(P{0, 0, 0}, P{2, 0, 0}, P{0, 0, 3}, U{0, 0}, U{-2, 0}, U{0, 3});
    REQUIRE(gradient.valid);
    check_tangent(rg::vertex_tangent(gradient, P{0, 1, 0}), -1, 0, 0, -1.0f);
}

TEST_CASE("flat tangent: flipped v flips the sign only", "[buildings]") {
    const auto gradient = rg::triangle_uv_gradient(P{0, 0, 0}, P{2, 0, 0}, P{0, 0, 3}, U{0, 0}, U{2, 0}, U{0, -3});
    REQUIRE(gradient.valid);
    check_tangent(rg::vertex_tangent(gradient, P{0, 1, 0}), 1, 0, 0, -1.0f);
}

TEST_CASE("flat tangent: the tangent is square to a smoothed vertex normal", "[buildings]") {
    const auto gradient = rg::triangle_uv_gradient(P{0, 0, 0}, P{2, 0, 0}, P{0, 0, 3}, U{0, 0}, U{2, 0}, U{0, 3});
    const float c = static_cast<float>(std::sqrt(0.5));
    const auto got = rg::vertex_tangent(gradient, P{c, c, 0});
    CHECK((got.tangent[0] * c + got.tangent[1] * c) == Approx(0.0).margin(1e-6));
    CHECK(std::hypot(got.tangent[0], got.tangent[1], got.tangent[2]) == Approx(1.0).margin(1e-6));
    CHECK(got.tangent[0] > 0.0f);
}

TEST_CASE("flat tangent: degenerate UVs or a gradient along the normal stay finite", "[buildings]") {
    const auto no_area = rg::triangle_uv_gradient(P{0, 0, 0}, P{2, 0, 0}, P{0, 0, 3}, U{1, 1}, U{1, 1}, U{1, 1});
    CHECK_FALSE(no_area.valid);
    for (const P normal : {P{0, 1, 0}, P{1, 0, 0}}) {
        const auto got = rg::vertex_tangent(no_area, normal);
        CHECK(std::hypot(got.tangent[0], got.tangent[1], got.tangent[2]) == Approx(1.0).margin(1e-6));
        CHECK((got.tangent[0] * normal[0] + got.tangent[1] * normal[1] + got.tangent[2] * normal[2]) ==
              Approx(0.0).margin(1e-6));
    }
    const auto gradient = rg::triangle_uv_gradient(P{0, 0, 0}, P{2, 0, 0}, P{0, 0, 3}, U{0, 0}, U{2, 0}, U{0, 3});
    const auto along_normal = rg::vertex_tangent(gradient, P{1, 0, 0});
    CHECK(std::hypot(along_normal.tangent[0], along_normal.tangent[1], along_normal.tangent[2]) == Approx(1.0).margin(1e-6));
}
