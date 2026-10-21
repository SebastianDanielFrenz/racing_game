// resolve_surface_layers (core/include/rg/surface_layers.h): the road-visual layers of a tile never overlap, decals
// are lifted off the surface. Owner 2026-10-19, Engelsruhe spawn: gravel verge triangles flickered through the 2.5 m
// cycle/sidewalk bands beside the carriageway (and paint dashes through the junction patch) because layers draped on the
// same ground by separate vertex sampling differ by up to 24 mm, more than their 3-7 mm lifts.
//
// Recorded sabotage: resolve_surface_layers returning at once (nothing cut, nothing lifted) fails SL1-SL4, SL8, SL9 and
// SL10; SL5-SL7 and SL11 are the guards that the cut does not over-reach (shared edges, different heights, equal lane
// counts, attribute interpolation) and pass either way. Single-rule sabotages: the unpaved-strip rank removed -> SL9
// fails; the lane count dropped from the carriageway order -> SL10 fails.
#include "rg/surface_layers.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include "g2m/core/tile_key.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

using Catch::Approx;

namespace {

using g2m::world::RoadRibbonTile;
using g2m::world::RoadRibbonVertex;

constexpr int kWalk = 32768;
constexpr int kEarth = 8192;

RoadRibbonVertex vtx(float x, float y, float z, float flags, float verge = 0, float width = 5.5f, float lanes = 1) {
    return RoadRibbonVertex{x, y, z, x, y, width, 1, lanes, 0, flags, verge};
}

// One quad (two triangles) of a layer over [x0,x1] x [y0,y1] at height z.
void quad(RoadRibbonTile& m, float x0, float y0, float x1, float y1, float z, float flags, float verge = 0, float width = 5.5f,
          float lanes = 1) {
    const auto base = static_cast<std::int32_t>(m.vertices.size());
    m.vertices.push_back(vtx(x0, y0, z, flags, verge, width, lanes));
    m.vertices.push_back(vtx(x1, y0, z, flags, verge, width, lanes));
    m.vertices.push_back(vtx(x1, y1, z, flags, verge, width, lanes));
    m.vertices.push_back(vtx(x0, y1, z, flags, verge, width, lanes));
    for (const int i : {0, 1, 2, 0, 2, 3}) m.indices.push_back(base + i);
}

struct Hit { int rank; double z; };

int rank_for(const RoadRibbonVertex& v) {
    const int f = static_cast<int>(std::lround(v.flags));
    if (f & (16 | 32)) return -1;
    if (v.verge > 0.5f) return 0;
    if (f & kWalk) return 1;
    if (v.width > 500.0f) return 4;
    return (f & 8192) ? 2 : 3;
}

bool inside(double px, double py, const RoadRibbonVertex& a, const RoadRibbonVertex& b, const RoadRibbonVertex& c, double& z) {
    const double d = (double(b.y) - c.y) * (double(a.x) - c.x) + (double(c.x) - b.x) * (double(a.y) - c.y);
    if (std::abs(d) < 1e-12) return false;
    const double w0 = ((double(b.y) - c.y) * (px - c.x) + (double(c.x) - b.x) * (py - c.y)) / d;
    const double w1 = ((double(c.y) - a.y) * (px - c.x) + (double(a.x) - c.x) * (py - c.y)) / d;
    const double w2 = 1 - w0 - w1;
    if (w0 < 1e-6 || w1 < 1e-6 || w2 < 1e-6) return false; // strictly inside: shared edges are not an overlap
    z = w0 * a.z + w1 * b.z + w2 * c.z;
    return true;
}

std::vector<Hit> hits_at(const RoadRibbonTile& m, double px, double py) {
    std::vector<Hit> out;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const auto& a = m.vertices[static_cast<std::size_t>(m.indices[i])];
        const auto& b = m.vertices[static_cast<std::size_t>(m.indices[i + 1])];
        const auto& c = m.vertices[static_cast<std::size_t>(m.indices[i + 2])];
        double z = 0;
        if (inside(px, py, a, b, c, z)) out.push_back({rank_for(a), z});
    }
    return out;
}

// Samples of a 5 cm grid where two non-decal layers of different rank are both present.
int overlapping_samples(const RoadRibbonTile& m, double x0, double y0, double x1, double y1, double step = 0.05) {
    int n = 0;
    for (double x = x0; x <= x1; x += step)
        for (double y = y0; y <= y1; y += step) {
            int lo = 99, hi = -99;
            for (const Hit& h : hits_at(m, x + 0.0123, y + 0.0071)) {
                if (h.rank < 0) continue;
                lo = std::min(lo, h.rank);
                hi = std::max(hi, h.rank);
            }
            if (lo != hi && lo != 99) ++n;
        }
    return n;
}

double covered_area(const RoadRibbonTile& m, int rank, double x0, double y0, double x1, double y1) {
    double n = 0;
    for (double x = x0; x <= x1; x += 0.05)
        for (double y = y0; y <= y1; y += 0.05)
            for (const Hit& h : hits_at(m, x + 0.0123, y + 0.0071))
                if (h.rank == rank) { n += 0.0025; break; }
    return n;
}

} // namespace

TEST_CASE("SL1 a verge strip under a walkway band is removed, the band stays", "[surface_layers][world]") {
    RoadRibbonTile m;
    quad(m, 0, 0, 10, 0.5f, 100.00f, 1, 1.0f);  // gravel verge shoulder
    quad(m, 0, 0, 10, 2.5f, 100.01f, kWalk);     // cycle band over it
    REQUIRE(overlapping_samples(m, 0, 0, 10, 2.5) > 1000);
    const double band_before = covered_area(m, 1, 0, 0, 10, 2.5);
    rg::resolve_surface_layers(m);
    CHECK(overlapping_samples(m, 0, 0, 10, 2.5) == 0);
    CHECK(covered_area(m, 0, 0, 0, 10, 2.5) < 0.05);
    CHECK(covered_area(m, 1, 0, 0, 10, 2.5) == band_before);
}

TEST_CASE("SL2 a footway crossing the carriageway is cut to the part beside it", "[surface_layers][world]") {
    RoadRibbonTile m;
    quad(m, 0, 0, 20, 6, 50.0f, 1);               // carriageway ribbon
    quad(m, 8, -4, 10, 10, 50.01f, kWalk | 2048); // footway across it (+10 mm lift on the road)
    const double footway_before = covered_area(m, 1, 8, -4, 10, 10);
    REQUIRE(overlapping_samples(m, 8, -4, 10, 10) > 1000);
    rg::resolve_surface_layers(m);
    CHECK(overlapping_samples(m, 0, -4, 20, 10) == 0);
    const double footway_after = covered_area(m, 1, 8, -4, 10, 10);
    CHECK(footway_after == Approx(footway_before - 2.0 * 6.0).margin(0.15)); // 2 m wide strip across 6 m of road
    CHECK(covered_area(m, 3, 0, 0, 20, 6) == Approx(20.0 * 6.0).margin(0.2)); // the road itself is untouched
}

TEST_CASE("SL3 the junction patch outranks the ribbon under it", "[surface_layers][world]") {
    RoadRibbonTile m;
    quad(m, 0, 0, 10, 10, 20.0f, 1);                       // ribbon rows
    quad(m, 3, 3, 8, 8, 20.004f, 0, 0, 1000.0f);           // junction patch (+4 mm)
    rg::resolve_surface_layers(m);
    CHECK(overlapping_samples(m, 0, 0, 10, 10) == 0);
    CHECK(covered_area(m, 4, 3, 3, 8, 8) == Approx(25.0).margin(0.1));
    CHECK(covered_area(m, 3, 0, 0, 10, 10) == Approx(100.0 - 25.0).margin(0.2));
}

TEST_CASE("SL9 an earth track crossing a dual carriageway never shows on the asphalt", "[surface_layers][world]") {
    // Owner 2026-10-19, site 2 (B8 near Main-Taunus-Zentrum): a highway=track strip (flags 8192) lay over both
    // carriageways and the median; the two ribbons are at the same height, so the dirt flickered through the asphalt.
    RoadRibbonTile m;
    quad(m, 0, 0, 40, 6.5f, 130.0f, 1);            // carriageway one
    quad(m, 0, 10.5f, 40, 17, 130.0f, 1);          // carriageway two, 4 m median between
    quad(m, 15, -3, 18, 21, 130.012f, 1 | kEarth); // 3 m wide earth track across both
    REQUIRE(overlapping_samples(m, 15, -3, 18, 21) > 1000);
    const double track_before = covered_area(m, 2, 15, -3, 18, 21);
    rg::resolve_surface_layers(m);
    CHECK(overlapping_samples(m, 0, -3, 40, 21) == 0);
    CHECK(covered_area(m, 2, 15, 0, 18, 6.4) < 0.05);    // gone from carriageway one
    CHECK(covered_area(m, 2, 15, 10.6, 18, 17) < 0.05);  // and from carriageway two
    CHECK(covered_area(m, 2, 15, 6.5, 18, 10.5) == Approx(3.0 * 4.0).margin(0.1)); // the median part stays
    CHECK(covered_area(m, 2, 15, -3, 18, 21) == Approx(track_before - 2.0 * 3.0 * 6.5).margin(0.3));
    CHECK(covered_area(m, 3, 0, 0, 40, 6.5) == Approx(40.0 * 6.5).margin(0.3)); // the asphalt is untouched
}

namespace {
// Samples of a 5 cm grid covered by two or more non-decal triangles (any layer) at (nearly) the same height.
int stacked_samples(const RoadRibbonTile& m, double x0, double y0, double x1, double y1) {
    int n = 0;
    for (double x = x0; x <= x1; x += 0.05)
        for (double y = y0; y <= y1; y += 0.05) {
            const auto hits = hits_at(m, x + 0.0123, y + 0.0071);
            int solid = 0;
            for (const Hit& h : hits) solid += h.rank >= 0 ? 1 : 0;
            if (solid >= 2) ++n;
        }
    return n;
}
} // namespace

TEST_CASE("SL10 an on-ramp ribbon never lies over the carriageway it merges into", "[surface_layers][world]") {
    // Owner 2026-10-19, site 4 (B8 on-ramp from Main-Taunus-Zentrum): the one-lane link ends on the axis of the two-lane
    // B8 node, so its last 40 m lie over the B8's right lane at the same height; both ribbons paint their own
    // markings there and the depth test flips between them. The ribbon with more lanes keeps the surface.
    RoadRibbonTile m;
    quad(m, 0, 0, 40, 7, 130.0f, 1, 0, 7.0f, 2);        // the B8: two lanes
    quad(m, 10, 3.5f, 50, 7.5f, 130.002f, 1, 0, 3.5f, 1); // the link: one lane, its end over the B8's right half
    REQUIRE(stacked_samples(m, 0, 0, 50, 8) > 1000);
    const double link_before = covered_area(m, 3, 0, 0, 60, 8);
    rg::resolve_surface_layers(m);
    CHECK(stacked_samples(m, 0, 0, 50, 8) == 0);
    // the B8 keeps all 280 m2, the link keeps only what lies beside it (x > 40, plus the strip above y = 7)
    // (the 5 cm sample grid counts its end rows too, hence the margin)
    CHECK(covered_area(m, 3, 0, 0, 40, 7) == Approx(40.0 * 7.0).margin(2.5));
    // together 280 (B8) + 55 (link left over: x 40..50 at 4 m, and the 0.5 m strip above the B8 over x 10..40)
    CHECK(covered_area(m, 3, 0, 0, 60, 8) == Approx(280.0 + 40.0 + 15.0).margin(4.0));
    CHECK(covered_area(m, 3, 0, 0, 60, 8) == Approx(link_before).margin(0.5)); // no hole: the union is unchanged
}

TEST_CASE("SL11 ribbons with the same lane count, or at different heights, are left alone", "[surface_layers][world]") {
    {   // two one-lane ribbons across each other: no rule can pick a winner, nothing is cut
        RoadRibbonTile m;
        quad(m, 0, 0, 20, 4, 50.0f, 1, 0, 4.0f, 1);
        quad(m, 8, -6, 12, 10, 50.0f, 1, 0, 4.0f, 1);
        const std::size_t before = m.indices.size();
        rg::resolve_surface_layers(m);
        CHECK(m.indices.size() == before);
    }
    {   // a one-lane ramp over a two-lane road 6 m below it (a bridge): different surfaces
        RoadRibbonTile m;
        quad(m, 0, 0, 40, 7, 50.0f, 1, 0, 7.0f, 2);
        quad(m, 10, 2, 30, 6, 56.0f, 1, 0, 4.0f, 1);
        const std::size_t before = m.indices.size();
        rg::resolve_surface_layers(m);
        CHECK(m.indices.size() == before);
    }
}

TEST_CASE("SL4 decals are lifted, nothing else moves", "[surface_layers][world]") {
    RoadRibbonTile m;
    quad(m, 0, 0, 4, 4, 10.0f, 1);
    quad(m, 1, 1, 2, 2, 10.0f, 16);
    quad(m, 2, 2, 3, 3, 10.0f, 1 | 32);
    rg::resolve_surface_layers(m);
    int lifted = 0, still = 0;
    for (const auto& v : m.vertices) {
        const int f = static_cast<int>(std::lround(v.flags));
        if (f & (16 | 32)) { CHECK(v.z == Approx(10.0 + rg::kDecalLiftM).margin(1e-5)); ++lifted; }
        else { CHECK(v.z == 10.0f); ++still; }
    }
    CHECK(lifted == 8);
    CHECK(still == 4);
}

TEST_CASE("SL5 coplanar neighbours sharing an edge are left untouched", "[surface_layers][world]") {
    RoadRibbonTile m;
    quad(m, 0, 0, 10, 5, 30.0f, 1);
    quad(m, 0, 5, 10, 7.5f, 30.0f, kWalk);
    quad(m, 0, 7.5f, 10, 8.0f, 30.0f, 1, 1.0f);
    const auto verts = m.vertices.size();
    const auto indices = m.indices;
    rg::resolve_surface_layers(m);
    CHECK(m.vertices.size() == verts);
    CHECK(m.indices == indices);
}

TEST_CASE("SL6 a footway under a bridge deck is not cut by it", "[surface_layers][world]") {
    RoadRibbonTile m;
    quad(m, 0, 0, 20, 8, 112.0f, 1);               // bridge deck ribbon
    quad(m, 8, -2, 10, 10, 100.01f, kWalk | 2048); // footway 12 m below
    const auto indices = m.indices.size();
    rg::resolve_surface_layers(m);
    CHECK(m.indices.size() == indices);
}

TEST_CASE("SL7 cut pieces keep the surface: height and uv follow the original triangle", "[surface_layers][world]") {
    RoadRibbonTile m;
    quad(m, 0, 0, 10, 10, 5.0f, 1);
    quad(m, 4, -1, 6, 11, 5.0f, kWalk); // walkway across it
    for (auto& v : m.vertices) v.z = 5.0f + 0.1f * v.x; // a slope: the piece heights must follow it
    rg::resolve_surface_layers(m);
    for (const auto& v : m.vertices) {
        CHECK(v.z == Approx(5.0 + 0.1 * v.x).margin(1e-4));
        CHECK(v.u == Approx(v.x).margin(1e-4)); // vtx() stores u = x
    }
}

TEST_CASE("SL8 real tile: the Engelsruhe spawn tile has no overlapping layers and lifted decals",
          "[.][realdata][surface_layers][world]") {
    const std::string cfg = std::string(RG_SOURCE_DIR) + "/data/world/world_config_rhein_main_hessen.json";
    std::string err;
    const auto world = rg::load_world_config(cfg, &err);
    INFO(err);
    REQUIRE(world.has_value());
    auto terrain = rg::WorldTerrain::open(*world, &err);
    INFO(err);
    REQUIRE(terrain != nullptr);
    const double sx = 2767.79, sy = -7577.48; // data/world/spawn_presets.json engelsruhe
    const double e = terrain->frame().e0_m() + sx, n = terrain->frame().n0_m() + sy;
    const auto key = g2m::tile_key_at(terrain->frame().zone(), 2, e, n);
    REQUIRE(key.has_value());
    const auto mesh = terrain->road_visual_tile_shared(*key, 0.0);
    REQUIRE(mesh != nullptr);
    {   // Cost of the pass on a full-size tile (a second pass finds nothing left to cut: it measures the scan and the grid).
        RoadRibbonTile again = *mesh;
        const auto t0 = std::chrono::steady_clock::now();
        rg::resolve_surface_layers(again);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        WARN("resolve_surface_layers on " << mesh->vertices.size() << " vertices / " << mesh->indices.size() / 3 << " triangles: " << ms << " ms");
        CHECK(ms < 3000.0);
    }
    const double lx = e - key->min_easting(), ly = n - key->min_northing();
    // Only the triangles touching the window: the probes below test every sample against every triangle.
    RoadRibbonTile window;
    for (std::size_t i = 0; i + 2 < mesh->indices.size(); i += 3) {
        const RoadRibbonVertex* t[3] = {&mesh->vertices[static_cast<std::size_t>(mesh->indices[i])],
                                        &mesh->vertices[static_cast<std::size_t>(mesh->indices[i + 1])],
                                        &mesh->vertices[static_cast<std::size_t>(mesh->indices[i + 2])]};
        const double lo_x = std::min({t[0]->x, t[1]->x, t[2]->x}), hi_x = std::max({t[0]->x, t[1]->x, t[2]->x});
        const double lo_y = std::min({t[0]->y, t[1]->y, t[2]->y}), hi_y = std::max({t[0]->y, t[1]->y, t[2]->y});
        if (hi_x < lx - 31 || lo_x > lx + 31 || hi_y < ly - 31 || lo_y > ly + 31) continue;
        const auto base = static_cast<std::int32_t>(window.vertices.size());
        for (const auto* v : t) window.vertices.push_back(*v);
        for (const int k : {0, 1, 2}) window.indices.push_back(base + k);
    }
    // 60 m x 60 m around the spawn: the cycle bands, the junction ahead and the gravel verge slivers all lie in it.
    CHECK(overlapping_samples(window, lx - 30, ly - 30, lx + 30, ly + 30, 0.15) == 0);
    // Decals sit above whatever surface lies under them (before the lift the difference was -8..+9 mm).
    double worst = 1e9;
    for (double x = lx - 30; x <= lx + 30; x += 0.2)
        for (double y = ly - 30; y <= ly + 30; y += 0.2) {
            double decal = -1e9, surface = -1e9;
            for (const Hit& h : hits_at(window, x + 0.0123, y + 0.0071)) {
                if (h.rank < 0) decal = std::max(decal, h.z);
                else surface = std::max(surface, h.z);
            }
            if (decal > -1e8 && surface > -1e8) worst = std::min(worst, decal - surface);
        }
    CHECK(worst >= 0.005);
}
