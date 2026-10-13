// test_terrain_carve_state.cpp - the carve state of a rendered terrain chunk and the invalidation of chunks that were
// built while the road surface was missing (gray-patch diagnosis 2026-10-13). Engine-neutral, no Godot.
//  * rg::chunk_window_carve: the worst carve state over the height tiles a chunk's halo window touches;
//  * rg::rendered_vs_bed: rendered ground against the physics road bed (also on coarse levels);
//  * rg::stale_fallback_chunks: which resident chunks were built as Fallback and must be rebuilt once the road surface
//    is ready (the streamer never rebuilds a key it already holds, so without this a fallback chunk stays forever).
#include "rg/terrain_render.h"
#include "rg/terrain_stream.h"
#include "rg/world_terrain.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <map>
#include <vector>

using Catch::Approx;
using g2m::mesh::ChunkKey;

namespace {

const g2m::geo::UtmZone kZone{32, g2m::geo::Hemisphere::North};

ChunkKey key_at(int level, int cx, int cy) { return ChunkKey{kZone, static_cast<std::uint8_t>(level), cx, cy}; }

// A flat chunk at absolute height z whose south-west corner is at the chunk's own grid origin.
rg::RenderChunk flat_chunk(const ChunkKey& key, double z, rg::ChunkCarve carve) {
    rg::RenderChunk c;
    c.key = key;
    c.carve = carve;
    c.mesh.key = key;
    const double spacing = std::ldexp(1.0, key.level);
    c.mesh.origin[0] = 64.0 * spacing * key.cx;
    c.mesh.origin[1] = 64.0 * spacing * key.cy;
    c.mesh.origin[2] = z;
    constexpr int verts = g2m::mesh::kChunkVerts;
    c.mesh.positions.assign(static_cast<std::size_t>(verts) * verts * 3, 0.0f); // z relative to origin[2] = 0
    return c;
}

// Records which tiles the window asked about; answers from a map (missing = NotCarved).
struct CarveMap {
    std::map<std::array<int, 4>, rg::ChunkCarve> states;
    mutable int queries = 0;
    static std::uint8_t fn(void* ctx, const g2m::TileKey& t) {
        const auto* self = static_cast<const CarveMap*>(ctx);
        ++self->queries;
        const auto it = self->states.find({t.level, t.x, t.y, 0});
        return static_cast<std::uint8_t>(it == self->states.end() ? rg::ChunkCarve::NotCarved : it->second);
    }
};

} // namespace

TEST_CASE("chunk carve: the worst state over the tiles of the chunk's halo window", "[terrain_stream][terrain]") {
    // Chunk (L1, 4, 4): samples 255..321 on both axes -> height tiles 0 and 1 in x and y (256 samples per tile).
    const ChunkKey key = key_at(1, 4, 4);
    CarveMap m;
    CHECK(rg::chunk_window_carve(key, &m, &CarveMap::fn) == rg::ChunkCarve::NotCarved);
    CHECK(m.queries == 4); // the halo reaches into the neighbour tiles: 2 x 2

    m.states[{1, 1, 1, 0}] = rg::ChunkCarve::Carved;
    CHECK(rg::chunk_window_carve(key, &m, &CarveMap::fn) == rg::ChunkCarve::Carved);

    // One Fallback tile among carved ones makes the chunk Fallback - and so does a tile only the halo touches.
    m.states[{1, 0, 0, 0}] = rg::ChunkCarve::Fallback;
    CHECK(rg::chunk_window_carve(key, &m, &CarveMap::fn) == rg::ChunkCarve::Fallback);

    // A chunk that is clear of that tile is not affected.
    CHECK(rg::chunk_window_carve(key_at(1, 12, 12), &m, &CarveMap::fn) == rg::ChunkCarve::NotCarved);

    // No state function: NotCarved (levels above carve_max_level, worlds without a road layer).
    CHECK(rg::chunk_window_carve(key, nullptr, nullptr) == rg::ChunkCarve::NotCarved);
}

TEST_CASE("rendered vs bed: the highest rendered chunk above each bed point, per level", "[terrain_stream][terrain]") {
    std::vector<rg::RenderChunk> chunks;
    chunks.push_back(flat_chunk(key_at(0, 0, 0), 100.0, rg::ChunkCarve::Carved));    // E,N 0..64, level 0
    chunks.push_back(flat_chunk(key_at(2, 1, 0), 103.0, rg::ChunkCarve::Fallback));  // E 256..512, N 0..256, level 2
    const std::vector<std::array<double, 3>> beds = {
        {30.0, 30.0, 100.0},   // under the carved L0 chunk, on the bed
        {300.0, 100.0, 100.0}, // under the fallback L2 chunk, 3 m below it
        {5000.0, 5000.0, 50.0} // no chunk: not counted
    };

    const rg::BedCheck c = rg::rendered_vs_bed(chunks, chunks.size(), beds, 0.25);
    CHECK(c.samples == 2);
    CHECK(c.above_tolerance == 1);
    CHECK(c.above_max_m == Approx(3.0));
    CHECK(c.worst_level == 2);
    CHECK(c.worst_carve == rg::ChunkCarve::Fallback);
    CHECK(c.worst_e == Approx(300.0));
    CHECK(c.above_max_by_level[0] == Approx(0.0).margin(1e-6));
    CHECK(c.above_max_by_level[2] == Approx(3.0));
    CHECK(c.samples_by_level[0] == 1);
    CHECK(c.samples_by_level[2] == 1);
    CHECK(c.samples_by_level[1] == 0);

    // Only the first `count` chunks count (the not yet uploaded tail is invisible).
    const rg::BedCheck first_only = rg::rendered_vs_bed(chunks, 1, beds, 0.25);
    CHECK(first_only.samples == 1);
    CHECK(first_only.above_tolerance == 0);

    // Nothing to compare: -1000, not 0.
    const rg::BedCheck none = rg::rendered_vs_bed(chunks, chunks.size(), {}, 0.25);
    CHECK(none.samples == 0);
    CHECK(none.above_max_m == -1000.0);
}

TEST_CASE("stale fallback chunks: built as Fallback, carved by now -> rebuild", "[terrain_stream][terrain]") {
    std::vector<rg::RenderChunk> chunks;
    chunks.push_back(flat_chunk(key_at(1, 0, 0), 10.0, rg::ChunkCarve::Fallback)); // road surface ready since
    chunks.push_back(flat_chunk(key_at(1, 1, 0), 10.0, rg::ChunkCarve::Fallback)); // still waiting
    chunks.push_back(flat_chunk(key_at(1, 2, 0), 10.0, rg::ChunkCarve::Carved));   // fine
    chunks.push_back(flat_chunk(key_at(3, 0, 0), 10.0, rg::ChunkCarve::NotCarved)); // never carved
    chunks.push_back(flat_chunk(key_at(1, 3, 0), 10.0, rg::ChunkCarve::Fallback)); // not uploaded yet (beyond count)

    const auto state_now = [](const ChunkKey& k) {
        if (k.level == 1 && k.cx == 1) return rg::ChunkCarve::Fallback; // still no road surface
        return rg::ChunkCarve::Carved;
    };
    const std::vector<std::size_t> stale = rg::stale_fallback_chunks(chunks, 4, state_now);
    REQUIRE(stale.size() == 1);
    CHECK(stale[0] == 0);

    // Everything healed: the one still waiting is stale now too; Carved / NotCarved chunks never are.
    const auto all_ready = [](const ChunkKey&) { return rg::ChunkCarve::Carved; };
    const std::vector<std::size_t> all = rg::stale_fallback_chunks(chunks, 4, all_ready);
    CHECK(all == std::vector<std::size_t>{0, 1});

    // Nothing healed: nothing to rebuild (a rebuild would just produce the same fallback again).
    const auto none_ready = [](const ChunkKey&) { return rg::ChunkCarve::Fallback; };
    CHECK(rg::stale_fallback_chunks(chunks, 4, none_ready).empty());
}
