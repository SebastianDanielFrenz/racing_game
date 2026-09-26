// test_terrain_view_streamer.cpp — rg::TerrainViewStreamer (R2.2 plan
// section 3/4c, commit R8) over a synthetic in-memory tile store (same
// approach as test_world_terrain.cpp: every tile is synthesised on first
// request, no TileStore/Server involved).
//
// TOOL-031: no Catch2 assertion ever runs on a worker thread. The streamer's
// worker and build threads only call SyntheticStore's plain C++ methods; the
// test thread polls, sleeps and asserts.
#include "rg/terrain_view_streamer.h"
#include "rg/world_terrain.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace {

using g2m::mesh::ChunkKey;

// Flat 100 m terrain, synthesised on demand and cached (std::map - ordered,
// lookup only). Optional gate: while closed, every lookup blocks (lets a test
// hold the streamer's worker in Building deterministically). Counts calls.
class SyntheticStore {
public:
    const g2m::HeightTile* get(const g2m::TileKey& key) {
        calls_.fetch_add(1, std::memory_order_relaxed);
        {
            std::unique_lock<std::mutex> lk(gate_mutex_);
            if (!gate_open_) {
                ++waiting_;
                gate_cv_.wait(lk, [this]() { return gate_open_; });
                --waiting_;
            }
        }
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = tiles_.find(key);
        if (it == tiles_.end()) {
            // Heap, never the stack or by value in the map (vault TOOL-039:
            // g2m::HeightTile is 256 KiB).
            auto tile = std::make_unique<g2m::HeightTile>();
            tile->key = key;
            // A gentle, key-dependent height so different tiles are not all
            // byte-identical (keeps the determinism comparison meaningful).
            tile->h.fill(100 * 256 + static_cast<std::int32_t>((key.x * 7 + key.y * 13 + key.level) % 64));
            tile->has_nodata = false;
            it = tiles_.emplace(key, std::move(tile)).first;
        }
        return it->second.get();
    }

    void set_gate(bool open) {
        {
            std::lock_guard<std::mutex> lk(gate_mutex_);
            gate_open_ = open;
        }
        gate_cv_.notify_all();
    }
    int waiting() {
        std::lock_guard<std::mutex> lk(gate_mutex_);
        return waiting_;
    }
    std::uint64_t calls() const { return calls_.load(); }

private:
    std::mutex mutex_;
    std::map<g2m::TileKey, std::unique_ptr<g2m::HeightTile>> tiles_;
    std::mutex gate_mutex_;
    std::condition_variable gate_cv_;
    bool gate_open_ = true;
    int waiting_ = 0;
    std::atomic<std::uint64_t> calls_{0};
};

const g2m::HeightTile* synthetic_lookup(void* ctx, const g2m::TileKey& key) {
    return static_cast<SyntheticStore*>(ctx)->get(key);
}

constexpr double kE0 = 464000.0;
constexpr double kN0 = 5559000.0;

rg::TerrainViewSource make_source(SyntheticStore& store) {
    rg::TerrainViewSource source;
    source.params.zone = g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North};
    source.params.range0_m = 64.0;
    source.params.max_level = 3; // 64 / 128 / 256 / 512 m chunks
    source.params.max_distance_m = 700.0;
    source.e0 = kE0;
    source.n0 = kN0;
    source.lookup = &synthetic_lookup;
    source.ctx = &store;
    return source;
}

rg::TerrainViewStreamer::Options make_options(unsigned build_threads) {
    rg::TerrainViewStreamer::Options options;
    options.reselect_distance_m = 128.0;
    options.build_threads = build_threads;
    options.initial_build_threads = 2;
    return options;
}

std::vector<ChunkKey> sorted_selection(const rg::TerrainViewSource& source, double x, double y) {
    std::vector<ChunkKey> keys;
    rg::select_view_keys(x, y, source.params, source.e0, source.n0, keys);
    std::sort(keys.begin(), keys.end());
    return keys;
}

std::vector<ChunkKey> minus(const std::vector<ChunkKey>& a, const std::vector<ChunkKey>& b) {
    std::vector<ChunkKey> out;
    std::set_difference(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(out));
    return out;
}

std::vector<ChunkKey> keys_of(const std::vector<rg::RenderChunk>& chunks) {
    std::vector<ChunkKey> out;
    out.reserve(chunks.size());
    for (const rg::RenderChunk& c : chunks) out.push_back(c.key);
    return out;
}

// Polls (test thread) until a diff is ready; false on timeout.
bool wait_for_diff(rg::TerrainViewStreamer& streamer, rg::TerrainViewDiff& out,
                   std::chrono::seconds timeout = std::chrono::seconds(60)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (streamer.poll(out)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

// Session-local footprint of a chunk, from its key alone (terrain_chunk.h:
// origin E = 64*cx*2^level + 0.5, side 64*2^level).
struct Rect {
    double x0, y0, x1, y1;
};
Rect footprint(const ChunkKey& k) {
    const double size = 64.0 * static_cast<double>(std::int64_t{1} << k.level);
    const double x0 = static_cast<double>(k.cx) * size + 0.5 - kE0;
    const double y0 = static_cast<double>(k.cy) * size + 0.5 - kN0;
    return Rect{x0, y0, x0 + size, y0 + size};
}

// A grid of sample points with a per-point coverage count (how many resident
// chunk footprints contain the point). Points sit off every chunk edge
// (edges are at k*64 + 0.5; points at 16*i + 7.3), so containment is
// unambiguous.
class CoverageGrid {
public:
    CoverageGrid(double x0, double y0, int nx, int ny) : x0_(x0), y0_(y0), nx_(nx), ny_(ny), count_(nx * ny, 0) {}

    double px(int i) const { return x0_ + kStep * i; }
    double py(int j) const { return y0_ + kStep * j; }
    int size() const { return nx_ * ny_; }

    void add(const ChunkKey& k, int delta) {
        const Rect r = footprint(k);
        const int i0 = std::max(0, static_cast<int>(std::ceil((r.x0 - x0_) / kStep)));
        const int i1 = std::min(nx_ - 1, static_cast<int>(std::floor((r.x1 - x0_) / kStep)));
        const int j0 = std::max(0, static_cast<int>(std::ceil((r.y0 - y0_) / kStep)));
        const int j1 = std::min(ny_ - 1, static_cast<int>(std::floor((r.y1 - y0_) / kStep)));
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i)
                if (px(i) >= r.x0 && px(i) < r.x1 && py(j) >= r.y0 && py(j) < r.y1) count_[j * nx_ + i] += delta;
    }
    int count(int idx) const { return count_[static_cast<std::size_t>(idx)]; }

    // Mask of points covered by the union of `keys`' footprints.
    std::vector<bool> mask_of(const std::vector<ChunkKey>& keys) const {
        CoverageGrid tmp(x0_, y0_, nx_, ny_);
        for (const ChunkKey& k : keys) tmp.add(k, +1);
        std::vector<bool> mask(static_cast<std::size_t>(size()));
        for (int idx = 0; idx < size(); ++idx) mask[static_cast<std::size_t>(idx)] = tmp.count(idx) > 0;
        return mask;
    }
    int index_of(double x, double y) const {
        const int i = static_cast<int>(std::lround((x - x0_) / kStep));
        const int j = static_cast<int>(std::lround((y - y0_) / kStep));
        return j * nx_ + i;
    }
    // Number of points in `required` with zero coverage.
    int holes(const std::vector<bool>& required) const {
        int n = 0;
        for (int idx = 0; idx < size(); ++idx)
            if (required[static_cast<std::size_t>(idx)] && count_[static_cast<std::size_t>(idx)] == 0) ++n;
        return n;
    }

    static constexpr double kStep = 16.0;

private:
    double x0_, y0_;
    int nx_, ny_;
    std::vector<int> count_;
};

} // namespace

TEST_CASE("TerrainViewStreamer: RenderChunk.key matches the selection and the mesh", "[terrain_view_streamer]") {
    SyntheticStore store;
    const rg::TerrainViewSource source = make_source(store);
    std::vector<rg::RenderChunk> chunks;
    rg::build_static_view_from_lookup(10.0, 20.0, source.params, source.e0, source.n0, source.lookup, source.ctx, 3,
                                      chunks);
    std::vector<ChunkKey> selected;
    rg::select_view_keys(10.0, 20.0, source.params, source.e0, source.n0, selected);
    REQUIRE(chunks.size() == selected.size());
    for (std::size_t i = 0; i < chunks.size(); ++i) {
        CHECK(chunks[i].key == selected[i]);
        CHECK(chunks[i].key == chunks[i].mesh.key);
    }
}

TEST_CASE("TerrainViewStreamer: diff after a move is exactly new-minus-old and old-minus-new, sorted",
          "[terrain_view_streamer]") {
    SyntheticStore store;
    const rg::TerrainViewSource source = make_source(store);
    rg::TerrainViewStreamer streamer(source, make_options(2));

    std::vector<rg::RenderChunk> initial;
    REQUIRE(streamer.build_initial(0.0, 0.0, initial));
    std::vector<ChunkKey> old_keys = keys_of(initial);
    std::sort(old_keys.begin(), old_keys.end());
    CHECK(streamer.resident_keys() == old_keys);

    // build_initial == build_static_view_from_lookup for the same point.
    std::vector<rg::RenderChunk> reference;
    rg::build_static_view_from_lookup(0.0, 0.0, source.params, source.e0, source.n0, source.lookup, source.ctx, 1,
                                      reference);
    REQUIRE(reference.size() == initial.size());
    for (std::size_t i = 0; i < reference.size(); ++i) {
        CHECK(reference[i].key == initial[i].key);
        CHECK(reference[i].mesh.content_hash == initial[i].mesh.content_hash);
    }

    const double fx = 300.0, fy = -170.0;
    streamer.update_focus(fx, fy);
    rg::TerrainViewDiff diff;
    REQUIRE(wait_for_diff(streamer, diff));
    CHECK(diff.serial == 1);
    CHECK(diff.focus_x == fx);
    CHECK(diff.focus_y == fy);

    const std::vector<ChunkKey> new_keys = sorted_selection(source, fx, fy);
    const std::vector<ChunkKey> added = keys_of(diff.added);
    CHECK(added == minus(new_keys, old_keys));
    CHECK(diff.removed == minus(old_keys, new_keys));
    CHECK(std::is_sorted(added.begin(), added.end()));
    CHECK(std::is_sorted(diff.removed.begin(), diff.removed.end()));
    CHECK_FALSE(added.empty());
    CHECK_FALSE(diff.removed.empty());
    // Unchanged chunks were reused, not rebuilt.
    const rg::TerrainViewStreamer::Stats stats = streamer.stats();
    CHECK(stats.chunks_built == added.size());
    CHECK(stats.chunks_reused == new_keys.size() - added.size());
    CHECK(stats.chunks_reused > 0);

    // Every added chunk is exactly what a fresh build of that key gives.
    std::vector<rg::RenderChunk> fresh;
    rg::build_render_chunks(added, source.lookup, source.ctx, source.e0, source.n0, 1, fresh);
    REQUIRE(fresh.size() == diff.added.size());
    for (std::size_t i = 0; i < fresh.size(); ++i) {
        CHECK(fresh[i].mesh.content_hash == diff.added[i].mesh.content_hash);
        CHECK(fresh[i].mesh.positions == diff.added[i].mesh.positions);
        CHECK(fresh[i].origin_session[0] == diff.added[i].origin_session[0]);
        CHECK(fresh[i].origin_session[1] == diff.added[i].origin_session[1]);
    }

    CHECK(streamer.phase() == rg::TerrainViewStreamer::Phase::Applying);
    CHECK_FALSE(streamer.commit(diff.serial + 1)); // wrong serial: refused
    REQUIRE(streamer.commit(diff.serial));
    CHECK(streamer.resident_keys() == new_keys);
    CHECK(streamer.phase() == rg::TerrainViewStreamer::Phase::Idle);
}

TEST_CASE("TerrainViewStreamer: adds-then-removals never opens a hole, at every intermediate step",
          "[terrain_view_streamer]") {
    SyntheticStore store;
    const rg::TerrainViewSource source = make_source(store);
    rg::TerrainViewStreamer streamer(source, make_options(2));

    // Grid spanning every selection the walk below can produce (focus within
    // +-600 m, max_distance 700 m, root chunks up to 512 m beyond that).
    // Every step stays well inside the previous selection, so both focus
    // points lie in the area both selections cover.
    const double half = 2200.0;
    CoverageGrid grid(-half + 7.3, -half + 7.3, static_cast<int>(2 * half / CoverageGrid::kStep),
                      static_cast<int>(2 * half / CoverageGrid::kStep));

    std::vector<rg::RenderChunk> initial;
    REQUIRE(streamer.build_initial(0.0, 0.0, initial));
    std::vector<ChunkKey> resident = keys_of(initial);
    std::sort(resident.begin(), resident.end());
    for (const ChunkKey& k : resident) grid.add(k, +1);

    double old_x = 0.0, old_y = 0.0;
    const double walk[][2] = {{200.0, 0.0}, {420.0, 130.0}, {420.0, 400.0}, {-150.0, 600.0}, {-450.0, 200.0}};
    int total_adds = 0, total_removals = 0;
    for (const auto& step : walk) {
        const double nx = step[0], ny = step[1];
        streamer.update_focus(nx, ny);
        rg::TerrainViewDiff diff;
        REQUIRE(wait_for_diff(streamer, diff));
        REQUIRE(diff.focus_x == nx);

        const std::vector<ChunkKey> new_keys = sorted_selection(source, nx, ny);
        const std::vector<bool> in_old = grid.mask_of(resident);
        const std::vector<bool> in_new = grid.mask_of(new_keys);
        std::vector<bool> in_both(in_old.size());
        std::vector<bool> in_either(in_old.size());
        for (std::size_t i = 0; i < in_old.size(); ++i) {
            in_both[i] = in_old[i] && in_new[i];
            in_either[i] = in_old[i] || in_new[i];
        }
        // Non-vacuous: both focus points lie in the area both selections cover.
        REQUIRE(in_both[static_cast<std::size_t>(grid.index_of(old_x + 0.7, old_y + 0.7))]);
        REQUIRE(in_both[static_cast<std::size_t>(grid.index_of(nx + 0.7, ny + 0.7))]);
        REQUIRE(grid.holes(in_old) == 0);

        // Adds, one at a time: the old selection's whole area stays covered.
        int hole_steps = 0;
        for (const rg::RenderChunk& c : diff.added) {
            grid.add(c.key, +1);
            ++total_adds;
            if (grid.holes(in_old) != 0) ++hole_steps;
        }
        CHECK(hole_steps == 0);
        // All adds in: both selections' areas are covered.
        CHECK(grid.holes(in_either) == 0);
        // Removals, one at a time: the new selection's whole area stays covered
        // (and so does the part of the old area the new selection also claims).
        for (const ChunkKey& k : diff.removed) {
            grid.add(k, -1);
            ++total_removals;
            if (grid.holes(in_new) != 0 || grid.holes(in_both) != 0) ++hole_steps;
        }
        CHECK(hole_steps == 0);

        // Negative control: the WRONG order (removals first) does open holes in
        // the area both selections cover - proves the checks above can fail.
        {
            for (const rg::RenderChunk& c : diff.added) grid.add(c.key, -1);
            for (const ChunkKey& k : diff.removed) grid.add(k, +1);
            // grid is back to the old resident set; now remove first:
            for (const ChunkKey& k : diff.removed) grid.add(k, -1);
            CHECK(grid.holes(in_both) > 0);
            for (const rg::RenderChunk& c : diff.added) grid.add(c.key, +1); // restore: new set
        }

        REQUIRE(streamer.commit(diff.serial));
        resident = new_keys;
        CHECK(streamer.resident_keys() == resident);
        CHECK(grid.holes(in_new) == 0);
        old_x = nx;
        old_y = ny;
    }
    WARN("walk: " << total_adds << " adds, " << total_removals << " removals checked step by step");
    CHECK(total_adds > 0);
    CHECK(total_removals > 0);
}

TEST_CASE("TerrainViewStreamer: no work while stationary or within the reselect distance", "[terrain_view_streamer]") {
    SyntheticStore store;
    const rg::TerrainViewSource source = make_source(store);
    rg::TerrainViewStreamer streamer(source, make_options(2));

    std::vector<rg::RenderChunk> initial;
    REQUIRE(streamer.build_initial(50.0, 50.0, initial));
    const std::uint64_t calls_after_initial = store.calls();

    for (int i = 0; i < 200; ++i) {
        streamer.update_focus(50.0, 50.0);
    }
    // Moves up to (but not beyond) 128 m from the selection focus.
    streamer.update_focus(50.0 + 128.0, 50.0);
    streamer.update_focus(50.0 + 100.0, 50.0 - 78.0); // 126.8 m
    streamer.update_focus(50.0 - 90.0, 50.0 + 90.0);  // 127.3 m
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    rg::TerrainViewDiff diff;
    CHECK_FALSE(streamer.poll(diff));
    CHECK(streamer.phase() == rg::TerrainViewStreamer::Phase::Idle);
    CHECK(streamer.stats().selections_started == 0);
    CHECK(store.calls() == calls_after_initial);

    // Positive control: just beyond the distance starts exactly one selection.
    streamer.update_focus(50.0 + 128.5, 50.0);
    CHECK(streamer.stats().selections_started == 1);
    REQUIRE(wait_for_diff(streamer, diff));
    REQUIRE(streamer.commit(diff.serial));
}

TEST_CASE("TerrainViewStreamer: without build_initial the first update delivers the full selection",
          "[terrain_view_streamer]") {
    SyntheticStore store;
    const rg::TerrainViewSource source = make_source(store);
    rg::TerrainViewStreamer streamer(source, make_options(2));
    streamer.update_focus(-40.0, 75.0);
    rg::TerrainViewDiff diff;
    REQUIRE(wait_for_diff(streamer, diff));
    CHECK(keys_of(diff.added) == sorted_selection(source, -40.0, 75.0));
    CHECK(diff.removed.empty());
    REQUIRE(streamer.commit(diff.serial));
}

TEST_CASE("TerrainViewStreamer: diffs are identical for 1 vs 8 build threads", "[terrain_view_streamer]") {
    SyntheticStore store_a;
    SyntheticStore store_b;
    rg::TerrainViewStreamer a(make_source(store_a), make_options(1));
    rg::TerrainViewStreamer b(make_source(store_b), make_options(8));

    std::vector<rg::RenderChunk> ia, ib;
    REQUIRE(a.build_initial(0.0, 0.0, ia));
    REQUIRE(b.build_initial(0.0, 0.0, ib));

    const double walk[][2] = {{260.0, 40.0}, {260.0, 400.0}, {-300.0, 380.0}};
    for (const auto& step : walk) {
        a.update_focus(step[0], step[1]);
        b.update_focus(step[0], step[1]);
        rg::TerrainViewDiff da, db;
        REQUIRE(wait_for_diff(a, da));
        REQUIRE(wait_for_diff(b, db));
        CHECK(da.serial == db.serial);
        CHECK(da.removed == db.removed);
        REQUIRE(da.added.size() == db.added.size());
        for (std::size_t i = 0; i < da.added.size(); ++i) {
            const rg::RenderChunk& x = da.added[i];
            const rg::RenderChunk& y = db.added[i];
            CHECK(x.key == y.key);
            CHECK(x.mesh.positions == y.mesh.positions);
            CHECK(x.mesh.normals == y.mesh.normals);
            CHECK(x.mesh.indices == y.mesh.indices);
            CHECK(x.mesh.content_hash == y.mesh.content_hash);
            CHECK(x.rgba == y.rgba);
            CHECK(x.origin_session[0] == y.origin_session[0]);
            CHECK(x.origin_session[1] == y.origin_session[1]);
            CHECK(x.origin_session[2] == y.origin_session[2]);
        }
        REQUIRE(a.commit(da.serial));
        REQUIRE(b.commit(db.serial));
        CHECK(a.resident_keys() == b.resident_keys());
    }
}

TEST_CASE("TerrainViewStreamer: many moves while busy coalesce into one diff to the latest focus",
          "[terrain_view_streamer]") {
    SyntheticStore store;
    const rg::TerrainViewSource source = make_source(store);
    rg::TerrainViewStreamer streamer(source, make_options(2));

    std::vector<rg::RenderChunk> initial;
    REQUIRE(streamer.build_initial(0.0, 0.0, initial));
    const std::vector<ChunkKey> keys_0 = streamer.resident_keys();

    store.set_gate(false); // hold the worker inside its first lookup
    streamer.update_focus(300.0, 0.0); // A
    REQUIRE(streamer.phase() == rg::TerrainViewStreamer::Phase::Building);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (store.waiting() == 0 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    REQUIRE(store.waiting() > 0);

    // B, C, D while Building - each far beyond 128 m from A and from each other.
    streamer.update_focus(600.0, 200.0);
    streamer.update_focus(900.0, 400.0);
    streamer.update_focus(1200.0, -500.0); // D (latest)
    CHECK(streamer.stats().selections_started == 1);

    store.set_gate(true);
    rg::TerrainViewDiff first;
    REQUIRE(wait_for_diff(streamer, first));
    CHECK(first.focus_x == 300.0);
    CHECK(first.focus_y == 0.0);

    // Ready -> Applying: still no second selection, even with a new move.
    streamer.update_focus(1200.0, -500.0);
    CHECK(streamer.stats().selections_started == 1);
    const std::vector<ChunkKey> keys_a = sorted_selection(source, 300.0, 0.0);
    CHECK(keys_of(first.added) == minus(keys_a, keys_0));

    // commit() starts the follow-up for the LATEST focus immediately.
    REQUIRE(streamer.commit(first.serial));
    CHECK(streamer.stats().selections_started == 2);
    rg::TerrainViewDiff second;
    REQUIRE(wait_for_diff(streamer, second));
    CHECK(second.focus_x == 1200.0);
    CHECK(second.focus_y == -500.0);
    const std::vector<ChunkKey> keys_d = sorted_selection(source, 1200.0, -500.0);
    CHECK(keys_of(second.added) == minus(keys_d, keys_a));
    CHECK(second.removed == minus(keys_a, keys_d));
    REQUIRE(streamer.commit(second.serial));

    // Nothing else pending: the focus sits exactly on the last selection.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    rg::TerrainViewDiff none;
    CHECK_FALSE(streamer.poll(none));
    CHECK(streamer.stats().selections_started == 2);
    CHECK(streamer.stats().diffs_committed == 2);
    CHECK(streamer.phase() == rg::TerrainViewStreamer::Phase::Idle);
}

TEST_CASE("TerrainViewStreamer: destruction mid-build cancels and joins the worker", "[terrain_view_streamer]") {
    SyntheticStore store;
    const rg::TerrainViewSource source = make_source(store);
    auto streamer = std::make_unique<rg::TerrainViewStreamer>(source, make_options(1));
    store.set_gate(false); // hold the worker inside the first chunk's first lookup
    streamer->update_focus(0.0, 0.0); // first update: the full selection is due
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (store.waiting() == 0 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    REQUIRE(store.waiting() > 0);

    // The destructor raises its cancel flag and blocks in join(); the gate
    // opens 20 ms later, the chunk in flight finishes, the build sees the
    // flag before the next chunk and the worker exits.
    std::thread opener([&store]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        store.set_gate(true);
    });
    streamer.reset();
    opener.join();
    const std::uint64_t calls_at_destroy = store.calls();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    std::vector<ChunkKey> full;
    rg::select_view_keys(0.0, 0.0, source.params, source.e0, source.n0, full);
    // One chunk's gather_window touches at most 9 tiles; a full build needs at
    // least one lookup per chunk.
    CHECK(calls_at_destroy <= 9);
    CHECK(full.size() > 9);
    CHECK(store.calls() == calls_at_destroy); // nothing runs after the join
}
