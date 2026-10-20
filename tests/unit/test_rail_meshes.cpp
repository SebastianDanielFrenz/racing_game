// rg/rail_meshes.h (RACE-028): rail bridge, ballast bed and rail geometry from a g2m.rail.geom tile, and the collision
// decks that never block a road. Every property has a recorded sabotage (a naive variant that must fail the same
// check; its numbers are printed with INFO so a regression shows what the check is worth).
#include "rg/rail_meshes.h"
#include "rg/world_config.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <string>

using Catch::Approx;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kTileX = 456, kTileY = 5420; // the owner spot (E 467577, N 5550232) lies in this level-2 tile
constexpr double kCornerE = kTileX * 1024.0, kCornerN = kTileY * 1024.0;

rg::RailConfig shipped() {
    std::string err;
    auto c = rg::load_rail_config(std::string(RG_SOURCE_DIR) + "/data/world/rail.json", &err);
    INFO(err);
    REQUIRE(c);
    return *c;
}

g2m::TileKey tile_key() { return g2m::TileKey{g2m::geo::UtmZone{32}, 2, kTileX, kTileY}; }

// A piece along `pts` (local metres east/north of the tile corner, height m).
struct P3 {
    double e, n, z;
};
g2m::RailPiece make_piece(std::int64_t way, std::uint32_t index, const std::vector<P3>& pts, std::uint16_t flags = 0,
                          std::uint8_t cls = 0, std::uint8_t tracks = 1, std::uint16_t gauge = 1435) {
    g2m::RailPiece p;
    p.way_id = way;
    p.piece = index;
    p.cls = cls;
    p.flags = flags;
    p.tracks = tracks;
    p.gauge_mm = gauge;
    for (const auto& q : pts) {
        p.xy.push_back({std::llround((kCornerE + q.e) * 1000.0), std::llround((kCornerN + q.n) * 1000.0)});
        p.z.push_back(static_cast<std::int32_t>(std::llround(q.z * 256.0)));
    }
    return p;
}

g2m::RailGeomTile tile_of(std::vector<g2m::RailPiece> pieces) {
    g2m::RailGeomTile t;
    t.key = tile_key();
    t.pieces = std::move(pieces);
    return t;
}

struct V3 {
    double x, y, z;
};
V3 vertex(const std::vector<float>& p, std::uint32_t i) { return {p[3 * i], p[3 * i + 1], p[3 * i + 2]}; }
double signed_volume(const std::vector<float>& pos, const std::vector<std::uint32_t>& idx) {
    double v = 0;
    for (std::size_t t = 0; t + 2 < idx.size(); t += 3) {
        const V3 a = vertex(pos, idx[t]), b = vertex(pos, idx[t + 1]), c = vertex(pos, idx[t + 2]);
        v += (a.x * (b.y * c.z - b.z * c.y) - a.y * (b.x * c.z - b.z * c.x) + a.z * (b.x * c.y - b.y * c.x)) / 6.0;
    }
    return v;
}

// A straight 100 m piece going east from local (200, 300).
std::vector<P3> straight(double z0, double z1, double length = 100.0) {
    return {{200.0, 300.0, z0}, {200.0 + length, 300.0, z1}};
}

double trapezoid_area(double top_half, double bottom_half, double height) { return (top_half + bottom_half) * height; }

} // namespace

TEST_CASE("rail: the shipped config loads and every rejection names its key", "[rail]") {
    const auto c = shipped();
    CHECK(c.draw_class[0]);
    CHECK(c.draw_class[1]);
    CHECK_FALSE(c.draw_class[2]);
    CHECK_FALSE(c.draw_class[3]);
    CHECK(c.collision_min_clearance_m == Approx(5.0));
    std::string err;
    CHECK(rg::validate_rail_config(c, &err));

    const std::string dir = std::string(RG_SOURCE_DIR) + "/out/test_tmp";
    std::filesystem::create_directories(dir);
    const std::string path = dir + "/rail_bad.json";
    std::string text;
    {
        std::ifstream in(std::string(RG_SOURCE_DIR) + "/data/world/rail.json", std::ios::binary);
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    const auto replaced = [&](const std::string& from, const std::string& to) {
        std::string t = text;
        const auto at = t.find(from);
        REQUIRE(at != std::string::npos);
        t.replace(at, from.size(), to);
        std::ofstream(path, std::ios::binary) << t;
        err.clear();
        return rg::load_rail_config(path, &err);
    };
    CHECK_FALSE(replaced("rg.rail/1", "rg.rail/2"));
    CHECK(err.find("format") != std::string::npos);
    CHECK_FALSE(replaced("\"ballast_top_width_m\": 3.6", "\"ballast_top_width_m\": 5.3"));
    CHECK(err.find("fit in bridge.deck_width_m") != std::string::npos);
    CHECK_FALSE(replaced("\"track_spacing_m\": 4.5", "\"track_spacing_m\": 0"));
    CHECK(err.find("track_spacing_m") != std::string::npos);
    CHECK_FALSE(replaced("\"min_clearance_m\": 5.0", "\"min_clearance_m\": -1"));
    CHECK(err.find("min_clearance_m") != std::string::npos);
    CHECK_FALSE(replaced("\"construction\": true", "\"construction\": 1"));
    CHECK(err.find("draw.construction") != std::string::npos);
    CHECK_FALSE(replaced("\"deck\": [0.52, 0.52, 0.5]", "\"deck\": [0.52, 0.52, 7]"));
    CHECK(err.find("colours.deck") != std::string::npos);
    CHECK_FALSE(rg::load_rail_config(path + ".missing", &err));
}

TEST_CASE("rail: a ground piece is a closed ballast bed with two rails, volume and extents exact", "[rail]") {
    const auto c = shipped();
    const auto tile = tile_of({make_piece(10, 0, straight(120.0, 120.0))});
    const auto out = rg::build_rail_tile_mesh(tile, c);
    REQUIRE_FALSE(out.mesh.empty());
    CHECK(out.stats.drawn == 1);
    CHECK(out.stats.ground == 1);
    CHECK(out.origin_e == Approx(kCornerE));
    CHECK(out.origin_n == Approx(kCornerN));
    const double top_half = c.ballast_top_width_m * 0.5;
    const double bottom_half = top_half + c.ballast_depth_m * c.ballast_side_slope;
    const double rails = 2.0 * c.rail_head_width_m * c.rail_height_m;
    const double expected = (trapezoid_area(top_half, bottom_half, c.ballast_depth_m) + rails) * 100.0;
    const double volume = signed_volume(out.mesh.positions, out.mesh.indices);
    INFO("volume " << volume << " expected " << expected);
    CHECK(volume == Approx(expected).epsilon(1e-4));
    // Extents: local east 200..300, lateral (north) +- bottom_half around 300, height from -depth to +rail height.
    double lo[3] = {1e9, 1e9, 1e9}, hi[3] = {-1e9, -1e9, -1e9};
    for (std::uint32_t i = 0; i < out.mesh.vertex_count(); ++i) {
        const V3 v = vertex(out.mesh.positions, i);
        lo[0] = std::min(lo[0], v.x); hi[0] = std::max(hi[0], v.x);
        lo[1] = std::min(lo[1], v.y); hi[1] = std::max(hi[1], v.y);
        lo[2] = std::min(lo[2], v.z); hi[2] = std::max(hi[2], v.z);
    }
    CHECK(lo[0] == Approx(200.0).margin(1e-3));
    CHECK(hi[0] == Approx(300.0).margin(1e-3));
    CHECK(hi[1] - 300.0 == Approx(bottom_half).margin(1e-3));
    CHECK(300.0 - lo[1] == Approx(bottom_half).margin(1e-3));
    CHECK(lo[2] == Approx(120.0 - c.ballast_depth_m).margin(1e-3));
    CHECK(hi[2] == Approx(120.0 + c.rail_height_m).margin(1e-3));
    // Index and normal sanity: every index valid, every stored normal is the triangle's own CCW normal.
    for (std::size_t t = 0; t + 2 < out.mesh.indices.size(); t += 3) {
        const V3 a = vertex(out.mesh.positions, out.mesh.indices[t]), b = vertex(out.mesh.positions, out.mesh.indices[t + 1]),
                 d = vertex(out.mesh.positions, out.mesh.indices[t + 2]);
        const V3 u{b.x - a.x, b.y - a.y, b.z - a.z}, w{d.x - a.x, d.y - a.y, d.z - a.z};
        V3 n{u.y * w.z - u.z * w.y, u.z * w.x - u.x * w.z, u.x * w.y - u.y * w.x};
        const double len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
        REQUIRE(len > 0.0);
        const float* stored = &out.mesh.normals[3 * out.mesh.indices[t]];
        CHECK((n.x / len * stored[0] + n.y / len * stored[1] + n.z / len * stored[2]) > 0.999);
    }
    // Sabotage (recorded): the same mesh with every triangle reversed reads as inside-out.
    auto reversed = out.mesh.indices;
    for (std::size_t t = 0; t + 2 < reversed.size(); t += 3) std::swap(reversed[t + 1], reversed[t + 2]);
    const double inside_out = signed_volume(out.mesh.positions, reversed);
    INFO("sabotage reversed winding: volume " << inside_out);
    CHECK(inside_out < 0.0);
    // Sabotage (recorded): without the two end caps the open mesh no longer closes to the analytic volume.
    std::vector<std::uint32_t> uncapped;
    for (std::size_t t = 0; t + 2 < out.mesh.indices.size(); t += 3) {
        const V3 a = vertex(out.mesh.positions, out.mesh.indices[t]), b = vertex(out.mesh.positions, out.mesh.indices[t + 1]),
                 d = vertex(out.mesh.positions, out.mesh.indices[t + 2]);
        if (std::abs(a.x - b.x) < 1e-4 && std::abs(a.x - d.x) < 1e-4) continue; // an x = const face: a cap
        uncapped.insert(uncapped.end(), {out.mesh.indices[t], out.mesh.indices[t + 1], out.mesh.indices[t + 2]});
    }
    const double open_volume = signed_volume(out.mesh.positions, uncapped);
    INFO("sabotage no caps: volume " << open_volume << " vs " << expected);
    CHECK(std::abs(open_volume - expected) > 0.02 * expected);
}

TEST_CASE("rail: a bridge piece keeps the profile height of the line: deck underside, ballast trough, parapets", "[rail]") {
    const auto c = shipped();
    // A 200 m span climbing from 130 m to 134 m (the line's own profile; nothing here knows a DEM).
    const auto tile = tile_of({make_piece(20, 0, straight(130.0, 134.0, 200.0), g2m::kRailBridge)});
    const auto out = rg::build_rail_tile_mesh(tile, c);
    REQUIRE_FALSE(out.mesh.empty());
    CHECK(out.stats.bridges == 1);
    CHECK(out.stats.ground == 0);
    const double hw = c.deck_width_m * 0.5, bh = c.ballast_height_m;
    const double top_half = c.ballast_top_width_m * 0.5;
    const double bottom_half = std::min(top_half + bh * c.ballast_side_slope, hw - c.parapet_width_m);
    const double length = std::sqrt(200.0 * 200.0 + 4.0 * 4.0); // volume is taken along the sloped line? no: x is horizontal
    (void)length;
    // Extrusion is per station along the horizontal polyline with the profile offsets vertical, so each prism's volume
    // is area * horizontal length exactly (the slope shears it).
    const double area = 2.0 * hw * c.deck_thickness_m                                  // deck slab
                        + trapezoid_area(top_half, bottom_half, bh)                    // ballast trough
                        + 2.0 * c.parapet_width_m * (c.parapet_height_m + bh)          // two parapets
                        + 2.0 * c.rail_head_width_m * c.rail_height_m;                 // two rails
    const double volume = signed_volume(out.mesh.positions, out.mesh.indices);
    INFO("volume " << volume << " expected " << area * 200.0);
    CHECK(volume == Approx(area * 200.0).epsilon(1e-4));
    // Underside of the deck at both ends follows the profile exactly.
    double lowest_at_start = 1e9, lowest_at_end = 1e9;
    for (std::uint32_t i = 0; i < out.mesh.vertex_count(); ++i) {
        const V3 v = vertex(out.mesh.positions, i);
        if (std::abs(v.x - 200.0) < 1e-3) lowest_at_start = std::min(lowest_at_start, v.z);
        if (std::abs(v.x - 400.0) < 1e-3) lowest_at_end = std::min(lowest_at_end, v.z);
    }
    CHECK(lowest_at_start == Approx(130.0 - bh - c.deck_thickness_m).margin(1e-3));
    CHECK(lowest_at_end == Approx(134.0 - bh - c.deck_thickness_m).margin(1e-3));
    // Sabotage (recorded): the same piece built as a ground bed (no flag) has no parapets: its highest point is the rail
    // top, not the parapet top, and its volume is the bed trapezoid, not the deck prism.
    auto ground_variant = tile;
    ground_variant.pieces[0].flags = 0;
    const auto ground = rg::build_rail_tile_mesh(ground_variant, c);
    double ground_highest = -1e9, bridge_highest = -1e9;
    for (std::uint32_t i = 0; i < ground.mesh.vertex_count(); ++i) ground_highest = std::max(ground_highest, vertex(ground.mesh.positions, i).z);
    for (std::uint32_t i = 0; i < out.mesh.vertex_count(); ++i) bridge_highest = std::max(bridge_highest, vertex(out.mesh.positions, i).z);
    INFO("sabotage ground bed: highest " << ground_highest << " vs bridge " << bridge_highest << ", volume "
         << signed_volume(ground.mesh.positions, ground.mesh.indices) << " vs " << volume);
    CHECK(bridge_highest == Approx(134.0 + c.parapet_height_m).margin(1e-3));
    CHECK(ground_highest == Approx(134.0 + c.rail_height_m).margin(1e-3));
    CHECK(std::abs(signed_volume(ground.mesh.positions, ground.mesh.indices) - volume) > 0.05 * volume);
}

TEST_CASE("rail: tunnels, switched-off classes, declined pieces and construction rails follow the config", "[rail]") {
    auto c = shipped();
    const auto tile = tile_of({make_piece(1, 0, straight(100, 100), 0, 0),                          // rail, ground
                               make_piece(2, 0, straight(100, 100), g2m::kRailTunnel, 0),           // tunnel
                               make_piece(3, 0, straight(100, 100), 0, 2),                          // subway
                               make_piece(4, 0, straight(100, 100), g2m::kRailDeclined, 0),         // declined
                               make_piece(5, 0, straight(100, 100), g2m::kRailBridge, 0),           // bridge
                               make_piece(6, 0, straight(100, 100), 0, 3)});                        // tram
    auto construction = tile;
    construction.pieces.push_back(make_piece(7, 0, straight(100, 100), g2m::kRailBridge, 0));
    construction.pieces.back().condition = g2m::RailCondition::Construction;
    const auto out = rg::build_rail_tile_mesh(construction, c);
    CHECK(out.stats.pieces == 7);
    CHECK(out.stats.drawn == 3); // the ground piece, the bridge, the construction bridge
    CHECK(out.stats.skipped_tunnel == 1);
    CHECK(out.stats.skipped_class == 2);
    CHECK(out.stats.skipped_declined == 1);
    // Rails are the dark steel colour: count their vertices with and without the construction piece.
    const auto rail_vertices = [&](const rg::RailTileMesh& m) {
        std::size_t n = 0;
        for (std::size_t i = 0; i < m.mesh.vertex_count(); ++i) {
            if (std::abs(m.mesh.colours[3 * i] - c.rail_colour.r) < 1e-6f && std::abs(m.mesh.colours[3 * i + 2] - c.rail_colour.b) < 1e-6f) ++n;
        }
        return n;
    };
    const auto with_construction = rail_vertices(out);
    auto plain = construction;
    plain.pieces.pop_back();
    const auto without = rail_vertices(rg::build_rail_tile_mesh(plain, c));
    INFO("rail vertices with the construction piece " << with_construction << ", without " << without);
    CHECK(with_construction == without); // a construction piece draws no rails
    // construction_rails on: the construction piece adds its rails.
    c.construction_rails = true;
    CHECK(rail_vertices(rg::build_rail_tile_mesh(construction, c)) > without);
    // Sabotage (recorded): classes ignored would draw the subway and the tram too.
    c.construction_rails = false;
    c.draw_class[2] = c.draw_class[3] = true;
    CHECK(rg::build_rail_tile_mesh(construction, c).stats.drawn == 5);
    // Declined drawn on request.
    c.draw_declined = true;
    CHECK(rg::build_rail_tile_mesh(construction, c).stats.drawn == 6);
}

TEST_CASE("rail: a curve keeps its width, extra tracks widen the bed, the mesh is deterministic", "[rail]") {
    const auto c = shipped();
    // A quarter circle of radius 400 m sampled every 20 degrees... 9 vertices.
    std::vector<P3> arc;
    for (int i = 0; i <= 9; ++i) {
        const double a = (kPi / 2.0) * i / 9.0;
        arc.push_back({200.0 + 400.0 * std::sin(a), 100.0 + 400.0 * (1.0 - std::cos(a)), 110.0});
    }
    const auto tile = tile_of({make_piece(30, 0, arc)});
    const auto first = rg::build_rail_tile_mesh(tile, c);
    const auto second = rg::build_rail_tile_mesh(tile, c);
    CHECK(first.mesh.positions == second.mesh.positions);
    CHECK(first.mesh.indices == second.mesh.indices);
    const double top_half = c.ballast_top_width_m * 0.5;
    const double bottom_half = top_half + c.ballast_depth_m * c.ballast_side_slope;
    const double area = trapezoid_area(top_half, bottom_half, c.ballast_depth_m) + 2.0 * c.rail_head_width_m * c.rail_height_m;
    const double arc_length = 400.0 * kPi / 2.0;
    const double volume = signed_volume(first.mesh.positions, first.mesh.indices);
    INFO("curve volume " << volume << " expected " << area * arc_length);
    CHECK(volume == Approx(area * arc_length).epsilon(0.03)); // a 9-gon chord is 0.3 % short, the mitre adds a little
    for (const float v : first.mesh.positions) CHECK(std::isfinite(v));
    // Two tracks: the ballast widens by track_spacing_m and there are four rails.
    auto two = tile;
    two.pieces[0].tracks = 2;
    const auto wide = rg::build_rail_tile_mesh(two, c);
    double lo = 1e9, hi = -1e9;
    const auto lateral_extent = [&](const rg::RailTileMesh& m) {
        // Straight piece variant for a clean lateral measurement.
        (void)m;
    };
    (void)lateral_extent;
    auto straight_one = tile_of({make_piece(31, 0, straight(100, 100))});
    auto straight_two = straight_one;
    straight_two.pieces[0].tracks = 2;
    const auto extent = [&](const rg::RailTileMesh& m) {
        double a = 1e9, b = -1e9;
        for (std::uint32_t i = 0; i < m.mesh.vertex_count(); ++i) {
            a = std::min(a, vertex(m.mesh.positions, i).y);
            b = std::max(b, vertex(m.mesh.positions, i).y);
        }
        return b - a;
    };
    (void)lo; (void)hi; (void)wide;
    const double one_track = extent(rg::build_rail_tile_mesh(straight_one, c));
    const double two_tracks = extent(rg::build_rail_tile_mesh(straight_two, c));
    CHECK(two_tracks - one_track == Approx(c.track_spacing_m).margin(1e-3));
    // More tracks than the config draws are capped.
    auto many = straight_one;
    many.pieces[0].tracks = 9;
    CHECK(extent(rg::build_rail_tile_mesh(many, c)) - one_track == Approx((c.max_tracks_drawn - 1) * c.track_spacing_m).margin(1e-3));
}

TEST_CASE("rail: a deck has collision only where it clears the ground below (never blocks a road)", "[rail][collision]") {
    const auto c = shipped();
    // 200 m bridge at 130 m. The ground below: a road at 100 m everywhere, but a hump (a ramp up to a road crossing
    // under the bridge at its middle) to 127 m between local E 280 and 320: 130 - 1.25 - 127 = 1.75 m of clearance.
    const auto tile = tile_of({make_piece(40, 0, straight(130.0, 130.0, 200.0), g2m::kRailBridge)});
    const rg::RailGroundFn ground = [&](double e, double) -> std::optional<double> {
        const double local = e - kCornerE;
        return (local >= 280.0 && local <= 320.0) ? 127.0 : 100.0;
    };
    int unknown = 0;
    const auto decks = rg::build_rail_deck_meshes(tile, c, ground, &unknown);
    REQUIRE(decks.size() == 1);
    CHECK(unknown == 0);
    const auto& deck = decks[0];
    CHECK(deck.spans > 0);
    CHECK(deck.spans_dropped >= 5); // the 40 m hump at 8 m steps, plus the span on each side that touches it
    const double bottom_level = 130.0 - c.ballast_height_m - c.deck_thickness_m;
    // Every vertex of every kept span has the clearance the config demands above the ground at its position.
    double worst = 1e9;
    for (std::size_t i = 0; i + 2 < deck.positions.size(); i += 3) {
        const double e = kCornerE + deck.positions[i];
        if (deck.positions[i + 2] > bottom_level + 1e-3) continue; // top face
        worst = std::min(worst, deck.positions[i + 2] - *ground(e, 0.0));
    }
    INFO("worst clearance of a kept underside vertex: " << worst);
    CHECK(worst >= c.collision_min_clearance_m - 1e-6);
    // The kept part is closed: positive volume, indices valid, 16 bit.
    std::vector<std::uint32_t> wide(deck.indices.begin(), deck.indices.end());
    CHECK(signed_volume(deck.positions, wide) > 0.0);
    for (const auto i : deck.indices) CHECK(i < deck.positions.size() / 3);
    // The hump itself has no collision at all: no vertex between E 284 and 316 (the dropped span plus its neighbours).
    for (std::size_t i = 0; i + 2 < deck.positions.size(); i += 3) {
        const double local = deck.positions[i];
        CHECK_FALSE((local > 285.0 && local < 315.0));
    }
    // Sabotage (recorded): a collision deck built without the clearance test blocks the road: its underside is only
    // 1.75 m above the hump, lower than any truck.
    auto naive_config = c;
    naive_config.collision_min_clearance_m = 0.001;
    const auto naive = rg::build_rail_deck_meshes(tile, naive_config, ground);
    REQUIRE(naive.size() == 1);
    double naive_worst = 1e9;
    for (std::size_t i = 0; i + 2 < naive[0].positions.size(); i += 3) {
        const double e = kCornerE + naive[0].positions[i];
        if (naive[0].positions[i + 2] > bottom_level + 1e-3) continue;
        naive_worst = std::min(naive_worst, naive[0].positions[i + 2] - *ground(e, 0.0));
    }
    INFO("sabotage no clearance test: worst clearance " << naive_worst << " m (a 4 m truck does not fit)");
    CHECK(naive_worst < 4.0);
    CHECK(naive[0].spans > deck.spans);
    // No ground callback = every span kept (tests and the flat world).
    const auto flat = rg::build_rail_deck_meshes(tile, c, nullptr);
    REQUIRE(flat.size() == 1);
    CHECK(flat[0].spans_dropped == 0);
}

TEST_CASE("rail: unknown ground gives no collision unless the config says otherwise; low bridges give none", "[rail][collision]") {
    auto c = shipped();
    const auto tile = tile_of({make_piece(50, 0, straight(130.0, 130.0, 100.0), g2m::kRailBridge)});
    const rg::RailGroundFn unknown = [](double, double) -> std::optional<double> { return std::nullopt; };
    int count = 0;
    CHECK(rg::build_rail_deck_meshes(tile, c, unknown, &count).empty());
    CHECK(count > 0); // the caller learns that ground tiles were missing and asks again later
    c.collision_unknown_ground_blocks = true;
    CHECK(rg::build_rail_deck_meshes(tile, c, unknown).size() == 1);
    c = shipped();
    // A low bridge over a road: 2.5 m of clearance everywhere -> no deck.
    const rg::RailGroundFn road = [](double, double) -> std::optional<double> { return 126.25; };
    CHECK(rg::build_rail_deck_meshes(tile, c, road).empty());
    // Exactly the required clearance keeps it.
    const rg::RailGroundFn exact = [&](double, double) -> std::optional<double> {
        return 130.0 - c.ballast_height_m - c.deck_thickness_m - c.collision_min_clearance_m;
    };
    CHECK(rg::build_rail_deck_meshes(tile, c, exact).size() == 1);
    // Switched off, tunnels, declined and ground pieces: no deck.
    auto off = c;
    off.collision_enabled = false;
    CHECK(rg::build_rail_deck_meshes(tile, off, nullptr).empty());
    auto other = tile;
    other.pieces[0].flags = g2m::kRailTunnel;
    CHECK(rg::build_rail_deck_meshes(other, c, nullptr).empty());
    other.pieces[0].flags = g2m::kRailBridge | g2m::kRailDeclined;
    CHECK(rg::build_rail_deck_meshes(other, c, nullptr).empty());
    other.pieces[0].flags = 0;
    CHECK(rg::build_rail_deck_meshes(other, c, nullptr).empty());
}

TEST_CASE("rail: the deck collision slab is the visual deck and ballast, bounds in absolute UTM", "[rail][collision]") {
    const auto c = shipped();
    const auto tile = tile_of({make_piece(60, 3, straight(140.0, 142.0, 100.0), g2m::kRailBridge)});
    const auto decks = rg::build_rail_deck_meshes(tile, c, nullptr);
    REQUIRE(decks.size() == 1);
    const auto& d = decks[0];
    CHECK(d.way_id == 60);
    CHECK(d.piece == 3u);
    const double hw = c.deck_width_m * 0.5;
    std::vector<std::uint32_t> wide(d.indices.begin(), d.indices.end());
    CHECK(signed_volume(d.positions, wide) == Approx(2.0 * hw * (c.ballast_height_m + c.deck_thickness_m) * 100.0).epsilon(1e-4));
    CHECK(d.min_easting == Approx(kCornerE + 200.0).margin(1e-3));
    CHECK(d.max_easting == Approx(kCornerE + 300.0).margin(1e-3));
    CHECK(d.min_northing == Approx(kCornerN + 300.0 - hw).margin(1e-3));
    CHECK(d.max_northing == Approx(kCornerN + 300.0 + hw).margin(1e-3));
    CHECK(d.origin_e == Approx(kCornerE));
    CHECK(d.origin_n == Approx(kCornerN));
}

TEST_CASE("rail: only the _rail world config opts in; its rail.json sits beside it", "[rail][world_config]") {
    const std::string dir = std::string(RG_SOURCE_DIR) + "/data/world/";
    std::string err;
    const auto plain = rg::load_world_config(dir + "world_config_rhein_main_hessen.json", &err);
    REQUIRE(plain.has_value());
    CHECK_FALSE(plain->rail_served); // the shipped configs keep their release id: no re-derive of the owner's store
    CHECK(plain->rail_config_path.empty());
    const auto corridor = rg::load_world_config(dir + "world_config_rhein_main_corridor.json", &err);
    REQUIRE(corridor.has_value());
    CHECK_FALSE(corridor->rail_served);
    const auto rail = rg::load_world_config(dir + "world_config_rhein_main_hessen_rail.json", &err);
    INFO(err);
    REQUIRE(rail.has_value());
    CHECK(rail->rail_served);
    CHECK(std::filesystem::path(rail->rail_config_path).lexically_normal() == std::filesystem::path(dir + "rail.json").lexically_normal());
    CHECK(rg::load_rail_config(rail->rail_config_path, &err).has_value());
    // The rail derived store is its own folder: a rail release never shares tiles.sqlite3 with the plain one.
    CHECK(rail->derived_store.dir != plain->derived_store.dir);
}
