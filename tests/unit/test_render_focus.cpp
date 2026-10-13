// test_render_focus.cpp - rg/render_focus.h (the velocity-led render focus) and
// rg/terrain_stream.h (adaptive upload budget, diff urgency order, LOD coverage
// ahead, visible surface height, sample window). Engine-neutral, no Godot.
#include "rg/render_focus.h"
#include "rg/terrain_stream.h"
#include "rg/terrain_view_streamer.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using Catch::Approx;
using g2m::mesh::ChunkKey;

namespace {

constexpr double kE0 = 464000.0;
constexpr double kN0 = 5559000.0;
const g2m::geo::UtmZone kZone{32, g2m::geo::Hemisphere::North};

ChunkKey key_at(int level, int cx, int cy) {
    return ChunkKey{kZone, static_cast<std::uint8_t>(level), cx, cy};
}

// Chunk (level, cx, cy) whose centre is at session-local (x, y).
ChunkKey key_around(int level, double x, double y) {
    const double size = 64.0 * std::ldexp(1.0, level);
    return key_at(level, static_cast<int>(std::floor((x + kE0) / size)), static_cast<int>(std::floor((y + kN0) / size)));
}

rg::RenderChunk chunk_of(const ChunkKey& key) {
    rg::RenderChunk c;
    c.key = key;
    c.mesh.key = key;
    return c;
}

} // namespace

TEST_CASE("render focus: lead = v_hat * min(|v| * lead_s, cap)", "[render_focus]") {
    rg::RenderFocusParams p;
    double lx = 0.0, ly = 0.0;
    rg::render_focus_target_lead(p, 30.0, 40.0, lx, ly); // 50 m/s -> 50 m
    CHECK(lx == Approx(30.0));
    CHECK(ly == Approx(40.0));
    rg::render_focus_target_lead(p, 160.0, 0.0, lx, ly); // 160 m/s -> capped at 150 m
    CHECK(lx == Approx(150.0));
    CHECK(ly == Approx(0.0).margin(1e-12));
    rg::render_focus_target_lead(p, 0.0, -400.0, lx, ly);
    CHECK(lx == Approx(0.0).margin(1e-12));
    CHECK(ly == Approx(-150.0));
    rg::render_focus_target_lead(p, 0.0, 0.0, lx, ly);
    CHECK(lx == 0.0);
    CHECK(ly == 0.0);
    rg::render_focus_target_lead(p, std::nan(""), 5.0, lx, ly); // non-finite velocity: no lead
    CHECK(lx == 0.0);
    CHECK(ly == 0.0);
}

TEST_CASE("render focus: the tracker converges to the lead and decays without it", "[render_focus]") {
    rg::RenderFocusTracker t;
    rg::RenderFocus f;
    for (int i = 0; i < 600; ++i) f = t.update(100.0, 200.0, 160.0, 0.0, 1.0 / 60.0, true);
    CHECK(f.x == Approx(250.0).margin(0.05));
    CHECK(f.y == Approx(200.0).margin(1e-9));
    CHECK(t.lead_length_m() == Approx(150.0).margin(0.05));
    // The rig stops following the car (free cam): the lead decays to the camera.
    for (int i = 0; i < 600; ++i) f = t.update(100.0, 200.0, 160.0, 0.0, 1.0 / 60.0, false);
    CHECK(f.x == Approx(100.0).margin(0.05));
    CHECK(t.lead_length_m() < 0.05);
    // reset() drops it at once.
    for (int i = 0; i < 600; ++i) t.update(0.0, 0.0, 100.0, 0.0, 1.0 / 60.0, true);
    t.reset();
    CHECK(t.lead_length_m() == 0.0);
}

TEST_CASE("render focus: smoothing - a braking car does not make the focus jump", "[render_focus]") {
    rg::RenderFocusTracker t;
    for (int i = 0; i < 600; ++i) t.update(0.0, 0.0, 160.0, 0.0, 1.0 / 60.0, true);
    const double before = t.lead_length_m();
    // Velocity flips to the opposite direction in one frame: the lead moves by at most
    // alpha * (target - lead) = 1 - exp(-dt/tau) of the way, not all of it.
    const rg::RenderFocus f = t.update(0.0, 0.0, -160.0, 0.0, 1.0 / 60.0, true);
    const double step = std::abs(f.lead_x - before);
    CHECK(step < 0.06 * 300.0); // 1 - exp(-0.01667/0.35) = 4.65 % of the 300 m swing
    CHECK(step > 0.0);
    // A huge dt (stall) counts as max_dt: the filter cannot overshoot.
    const rg::RenderFocus g = t.update(0.0, 0.0, -160.0, 0.0, 30.0, true);
    CHECK(g.lead_x >= -150.0 - 1e-9);
    CHECK(g.lead_x <= before);
}

TEST_CASE("render focus: only the rigs on the player car lead", "[render_focus]") {
    CHECK(rg::rig_leads_focus("chase"));
    CHECK(rg::rig_leads_focus("bumper"));
    CHECK(rg::rig_leads_focus("cockpit"));
    CHECK(rg::rig_leads_focus("orbit"));
    CHECK_FALSE(rg::rig_leads_focus("free"));
    CHECK_FALSE(rg::rig_leads_focus("drone"));
    CHECK_FALSE(rg::rig_leads_focus("walker"));
    CHECK_FALSE(rg::rig_leads_focus("cinematic"));
    CHECK_FALSE(rg::rig_leads_focus(""));
}

TEST_CASE("upload budget: base when idle, rises with backlog and with speed, capped", "[terrain_stream]") {
    rg::UploadBudgetParams p;
    CHECK(rg::adaptive_upload_budget_ms(0, 200.0, p) == Approx(0.8));
    CHECK(rg::adaptive_upload_budget_ms(4, 0.0, p) == Approx(0.8));  // small backlog, slow
    CHECK(rg::adaptive_upload_budget_ms(64, 0.0, p) == Approx(3.0)); // full backlog
    CHECK(rg::adaptive_upload_budget_ms(1000, 0.0, p) == Approx(3.0));
    CHECK(rg::adaptive_upload_budget_ms(4, 120.0, p) == Approx(3.0)); // fast with anything pending
    const double mid = rg::adaptive_upload_budget_ms(36, 0.0, p);     // half way up the backlog ramp
    CHECK(mid == Approx(0.8 + 2.2 * 0.5));
    CHECK(rg::adaptive_upload_budget_ms(36, 75.0, p) >= mid);
    // Monotonic in both inputs.
    double last = 0.0;
    for (std::size_t n = 0; n <= 100; n += 5) {
        const double b = rg::adaptive_upload_budget_ms(n, 10.0, p);
        CHECK(b >= last - 1e-12);
        last = b;
    }
    // A max below base never lowers the budget.
    p.max_ms = 0.5;
    CHECK(rg::adaptive_upload_budget_ms(500, 200.0, p) == Approx(0.8));
}

TEST_CASE("urgency: what the car reaches first comes first, behind last", "[terrain_stream]") {
    // The car at the origin driving east at 160 m/s.
    const double vx = 160.0, vy = 0.0;
    const ChunkKey ahead_l0 = key_around(0, 100.0, 0.0);
    const ChunkKey ahead_l1 = key_around(1, 140.0, 0.0);
    const ChunkKey far_ahead_l0 = key_around(0, 600.0, 0.0);
    const ChunkKey behind_l0 = key_around(0, -100.0, 0.0);
    const ChunkKey ahead_l3 = key_around(3, 1500.0, 0.0);

    CHECK(rg::chunk_time_to_reach_s(ahead_l0, kE0, kN0, 0.0, 0.0, vx, vy) <
          rg::chunk_time_to_reach_s(far_ahead_l0, kE0, kN0, 0.0, 0.0, vx, vy));
    CHECK(rg::chunk_time_to_reach_s(behind_l0, kE0, kN0, 0.0, 0.0, vx, vy) >= 1000.0);
    CHECK(rg::chunk_time_to_reach_s(ahead_l3, kE0, kN0, 0.0, 0.0, vx, vy) <
          rg::chunk_time_to_reach_s(behind_l0, kE0, kN0, 0.0, 0.0, vx, vy));

    std::vector<rg::RenderChunk> adds;
    adds.push_back(chunk_of(behind_l0));
    adds.push_back(chunk_of(ahead_l3));
    adds.push_back(chunk_of(far_ahead_l0));
    adds.push_back(chunk_of(ahead_l1));
    adds.push_back(chunk_of(ahead_l0));
    rg::sort_adds_by_urgency(adds, kE0, kN0, 0.0, 0.0, vx, vy);
    REQUIRE(adds.size() == 5);
    // Nearest first: the L0 chunk 64 m ahead, then the L1 chunk, then the far L0, then the L3.
    CHECK(adds[0].key == ahead_l0);
    CHECK(adds[1].key == ahead_l1);
    CHECK(adds[2].key == far_ahead_l0);
    CHECK(adds[3].key == ahead_l3);
    CHECK(adds.back().key == behind_l0); // behind the car: after everything ahead
    // Deterministic: the same input in another order gives the same output.
    std::vector<rg::RenderChunk> shuffled;
    shuffled.push_back(chunk_of(ahead_l0));
    shuffled.push_back(chunk_of(ahead_l1));
    shuffled.push_back(chunk_of(behind_l0));
    shuffled.push_back(chunk_of(far_ahead_l0));
    shuffled.push_back(chunk_of(ahead_l3));
    rg::sort_adds_by_urgency(shuffled, kE0, kN0, 0.0, 0.0, vx, vy);
    for (std::size_t i = 0; i < adds.size(); ++i) CHECK(adds[i].key == shuffled[i].key);
}

TEST_CASE("urgency: without a direction the order is plain distance", "[terrain_stream]") {
    const ChunkKey near = key_around(0, 30.0, 40.0);
    const ChunkKey far = key_around(0, -500.0, 0.0);
    CHECK(rg::chunk_time_to_reach_s(near, kE0, kN0, 0.0, 0.0, 0.0, 0.0) <
          rg::chunk_time_to_reach_s(far, kE0, kN0, 0.0, 0.0, 0.0, 0.0));
}

TEST_CASE("lod_coverage_ahead: distance to the first sample the level does not cover", "[terrain_stream]") {
    // e0 = n0 = 0 so session-local == absolute. Along y = 10: L0 chunks (64 m) cover x in [0, 256),
    // L1 chunks (128 m) [256, 512), L2 chunks (256 m) [512, 2048).
    std::vector<ChunkKey> keys;
    for (int cx = 0; cx < 4; ++cx) keys.push_back(key_at(0, cx, 0));
    for (int cx = 2; cx < 4; ++cx) keys.push_back(key_at(1, cx, 0));
    for (int cx = 2; cx < 8; ++cx) keys.push_back(key_at(2, cx, 0));
    const rg::LodCoverage c = rg::lod_coverage_ahead(keys, 0.0, 0.0, 2.0, 10.0, 1.0, 0.0, 3000.0, 4.0);
    CHECK(c.ahead_m[0] == Approx(254.0).margin(4.0));  // the first sample past x = 256
    CHECK(c.ahead_m[1] == Approx(510.0).margin(4.0));  // L0 or L1 up to x = 512
    CHECK(c.ahead_m[2] == Approx(2046.0).margin(4.0)); // everything up to x = 2048
    CHECK(c.ahead_m[3] == c.ahead_m[2]);               // no coarser chunk: same end
    // The reverse direction leaves the covered area at once.
    const rg::LodCoverage back = rg::lod_coverage_ahead(keys, 0.0, 0.0, 2.0, 10.0, -1.0, 0.0, 3000.0, 4.0);
    CHECK(back.ahead_m[0] <= 4.0);
    // A zero-length direction yields nothing; so does an empty set.
    CHECK(rg::lod_coverage_ahead(keys, 0.0, 0.0, 2.0, 10.0, 0.0, 0.0).ahead_m[0] == 0.0);
    CHECK(rg::lod_coverage_ahead({}, 0.0, 0.0, 2.0, 10.0, 1.0, 0.0).ahead_m[0] == 0.0);
    // A start outside every chunk: zero everywhere.
    const rg::LodCoverage out = rg::lod_coverage_ahead(keys, 0.0, 0.0, -5000.0, 10.0, 1.0, 0.0, 3000.0, 4.0);
    CHECK(out.ahead_m[0] == 0.0);
    CHECK(out.ahead_m[2] == 0.0);
}

TEST_CASE("lod_coverage_ahead: a fully covered ray reports max_m", "[terrain_stream]") {
    std::vector<ChunkKey> keys;
    for (int cx = 0; cx < 40; ++cx) keys.push_back(key_at(0, cx, 0));
    const rg::LodCoverage c = rg::lod_coverage_ahead(keys, 0.0, 0.0, 5.0, 10.0, 1.0, 0.0, 1000.0, 4.0);
    CHECK(c.ahead_m[0] == Approx(1000.0));
    CHECK(c.ahead_m[7] == Approx(1000.0));
}

TEST_CASE("grid_coverage_ahead: distance to the first missing tile", "[terrain_stream]") {
    const std::vector<std::array<int, 2>> tiles = {{0, 0}, {1, 0}, {2, 0}, {4, 0}}; // (3, 0) missing
    CHECK(rg::grid_coverage_ahead(tiles, 1024.0, 100.0, 100.0, 1.0, 0.0, 6000.0, 64.0) == Approx(3072.0 - 100.0).margin(64.0));
    CHECK(rg::grid_coverage_ahead(tiles, 1024.0, 100.0, 100.0, 0.0, 1.0, 6000.0, 64.0) <= 1024.0);
    CHECK(rg::grid_coverage_ahead(tiles, 1024.0, 100.0, 100.0, 0.0, 0.0, 6000.0, 64.0) == 0.0);
    CHECK(rg::grid_coverage_ahead(tiles, 1024.0, -2000.0, 100.0, 1.0, 0.0, 6000.0, 64.0) == 0.0); // start missing
    const std::vector<std::array<int, 2>> row = {{0, 0}, {1, 0}, {2, 0}};
    CHECK(rg::grid_coverage_ahead(row, 1024.0, 10.0, 10.0, 1.0, 0.0, 1000.0, 64.0) == Approx(1000.0));
}

TEST_CASE("chunk_surface_height: matches the mesh triangles, rejects outside and NoData", "[terrain_stream]") {
    rg::RenderChunk c;
    c.key = key_at(1, 3, 5);
    c.mesh.key = c.key;
    constexpr int verts = g2m::mesh::kChunkVerts;
    c.mesh.origin[0] = c.key.cx * 128.0 + 0.5;
    c.mesh.origin[1] = c.key.cy * 128.0 + 0.5;
    c.mesh.origin[2] = 100.0;
    c.mesh.positions.assign(static_cast<std::size_t>(verts) * verts * 3, 0.0f);
    for (int j = 0; j < verts; ++j) {
        for (int i = 0; i < verts; ++i) {
            const std::size_t k = 3 * static_cast<std::size_t>(j * verts + i);
            c.mesh.positions[k + 0] = static_cast<float>(i * 2);
            c.mesh.positions[k + 1] = static_cast<float>(j * 2);
            c.mesh.positions[k + 2] = static_cast<float>(0.5 * i + 0.25 * j); // a plane: z = 0.5 i + 0.25 j
        }
    }
    double z = 0.0;
    // Point at vertex (10, 4) + (1 m, 0.5 m): in the plane z = 100 + 0.5 u + 0.25 v with u, v in vertex units.
    REQUIRE(rg::chunk_surface_height(c, c.mesh.origin[0] + 21.0, c.mesh.origin[1] + 8.5, z));
    CHECK(z == Approx(100.0 + 0.5 * 10.5 + 0.25 * 4.25));
    CHECK_FALSE(rg::chunk_surface_height(c, c.mesh.origin[0] - 1.0, c.mesh.origin[1] + 8.5, z));
    CHECK_FALSE(rg::chunk_surface_height(c, c.mesh.origin[0] + 129.0, c.mesh.origin[1] + 8.5, z));
    // The far edge belongs to the chunk.
    CHECK(rg::chunk_surface_height(c, c.mesh.origin[0] + 128.0, c.mesh.origin[1] + 128.0, z));
    // A NoData vertex under the sample rejects it.
    c.mesh.positions[3 * static_cast<std::size_t>(4 * verts + 10) + 2] = -8388608.0f;
    CHECK_FALSE(rg::chunk_surface_height(c, c.mesh.origin[0] + 21.0, c.mesh.origin[1] + 8.5, z));
}

TEST_CASE("SampleWindow: mean, max, nearest-rank percentile, ring behaviour", "[terrain_stream]") {
    rg::SampleWindow w(4);
    CHECK(w.count() == 0);
    CHECK(w.mean() == 0.0);
    CHECK(w.percentile(0.95) == 0.0);
    for (double v : {1.0, 2.0, 3.0, 4.0}) w.add(v);
    CHECK(w.mean() == Approx(2.5));
    CHECK(w.max() == 4.0);
    CHECK(w.percentile(0.5) == 2.0);
    CHECK(w.percentile(0.95) == 4.0);
    CHECK(w.percentile(0.0) == 1.0);
    w.add(10.0); // overwrites the oldest (1.0)
    CHECK(w.count() == 4);
    CHECK(w.max() == 10.0);
    CHECK(w.mean() == Approx((2.0 + 3.0 + 4.0 + 10.0) / 4.0));
    w.clear();
    CHECK(w.count() == 0);
}

TEST_CASE("urgency: within one time bin the finest level goes first", "[terrain_stream]") {
    // The car stands inside an L2 chunk and the L0 chunk it contains: both are reached at 0 s.
    std::vector<rg::RenderChunk> adds;
    adds.push_back(chunk_of(key_around(2, 5.0, 5.0)));
    adds.push_back(chunk_of(key_around(0, 5.0, 5.0)));
    rg::sort_adds_by_urgency(adds, kE0, kN0, 5.0, 5.0, 28.0, 28.0);
    CHECK(adds[0].key.level == 0);
    CHECK(adds[1].key.level == 2);
}

TEST_CASE("reselect distance: the base when slow or unknown, halved at high speed", "[terrain_stream]") {
    CHECK(rg::reselect_distance_for_speed(128.0, 0.0) == 128.0);
    CHECK(rg::reselect_distance_for_speed(128.0, 40.0) == 128.0);
    CHECK(rg::reselect_distance_for_speed(128.0, 70.0) == Approx(96.0));
    CHECK(rg::reselect_distance_for_speed(128.0, 100.0) == Approx(64.0));
    CHECK(rg::reselect_distance_for_speed(128.0, 500.0) == Approx(64.0));
    CHECK(rg::reselect_distance_for_speed(128.0, 160.0, 0.25) == Approx(32.0));
}
