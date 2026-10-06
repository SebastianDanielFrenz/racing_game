// test_deck_installer.cpp - S1 / design 5.2 option B: road deck collision shapes are built on own threads and the
// bodies installed on the sim thread in a FIXED (way_id, start, end) order, whatever order the workers finish in.
//
// Contract checked here (the design's determinism test):
//   * the BodyId sequence and World::state_hash after installing through the DeckInstaller are identical to the
//     legacy path (create_body(desc with the mesh) in key order), for 1/2/4 workers and for builds whose completion
//     order is shuffled by a per-deck sleep;
//   * the tick path is strict: a deck whose shape is not ready holds back every later deck, never installs out of
//     order, and update() reports "not done" until all required decks are installed;
//   * a prefetched deck installs without waiting; a deck the backend rejects fails on its worker and does not block.
//
// Recorded sabotage (run once by hand, see the S1 report): replacing the strict `return false` in
// DeckInstaller::update by `continue` (install any ready deck) makes the "shuffled completion" and "strict order"
// cases fail.

#include "rg/deck_installer.h"

#include "ps/backend/shape_desc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <numeric>
#include <thread>
#include <vector>

namespace {

// A flat strip of `quads` quads along +x with a small zig-zag in z so the BVH is not trivial. Positions are
// relative to the deck's own origin (the Static body carries the world position).
std::shared_ptr<rg::RoadDeck> make_deck(std::int64_t way, int start, int quads) {
    auto deck = std::make_shared<rg::RoadDeck>();
    deck->way_id = way;
    deck->start_station = start;
    deck->end_station = start + quads;
    auto& mesh = deck->mesh;
    for (int i = 0; i <= quads; ++i) {
        const float z = (i % 2 == 0) ? 0.0f : 0.01f;
        mesh.positions.insert(mesh.positions.end(), {static_cast<float>(i), -2.0f, z});
        mesh.positions.insert(mesh.positions.end(), {static_cast<float>(i), 2.0f, z});
    }
    for (int i = 0; i < quads; ++i) {
        const std::uint16_t a = static_cast<std::uint16_t>(2 * i), b = static_cast<std::uint16_t>(2 * i + 1),
                            c = static_cast<std::uint16_t>(2 * i + 2), d = static_cast<std::uint16_t>(2 * i + 3);
        mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
    }
    return deck;
}

// Candidates in ascending key order: 3 ways x 4 stretches, each at its own x/y.
std::vector<rg::DeckCandidate> make_candidates(int quads = 400) {
    std::vector<rg::DeckCandidate> out;
    for (int way = 0; way < 3; ++way) {
        for (int s = 0; s < 4; ++s) {
            rg::DeckCandidate c;
            c.deck = make_deck(1000 + way, s * 500, quads);
            c.key = rg::deck_key(*c.deck);
            c.x = 40.0 * s;
            c.y = 30.0 * way;
            out.push_back(std::move(c));
        }
    }
    return out;
}

std::unique_ptr<ps::World> make_world() {
    ps::WorldConfig wc;
    wc.job_workers = 1;
    return std::make_unique<ps::World>(wc);
}

// A ball dropped on the decks, then 120 ticks, so the hash covers real contact against every installed mesh.
std::uint64_t settle_hash(ps::World& world, const std::vector<rg::DeckCandidate>& cands) {
    for (const auto& c : cands) {
        ps::BodyDesc ball;
        ball.shape = ps::SphereShape{0.3};
        ball.pose.position = {c.x + 20.0, c.y, 1.5};
        world.create_body(ball);
    }
    for (int i = 0; i < 120; ++i) world.step();
    return world.state_hash();
}

struct Legacy {
    std::vector<ps::BodyId> ids;
    std::uint64_t hash = 0;
};

Legacy legacy_install(const std::vector<rg::DeckCandidate>& cands) {
    auto world = make_world();
    Legacy l;
    for (const auto& c : cands) {
        ps::BodyDesc body;
        body.motion = ps::BodyMotionType::Static;
        body.shape = rg::make_deck_mesh_shape(*c.deck, c.surface);
        body.pose.position = {c.x, c.y, 0.0};
        l.ids.push_back(world->create_body(body));
    }
    l.hash = settle_hash(*world, cands);
    return l;
}

bool same_ids(const std::vector<ps::BodyId>& a, const std::vector<ps::BodyId>& b) {
    return a.size() == b.size() &&
           std::equal(a.begin(), a.end(), b.begin(), [](ps::BodyId x, ps::BodyId y) { return x == y; });
}

} // namespace

TEST_CASE("deck installer: install order and state_hash do not depend on build completion order",
          "[deck_installer][s1][determinism]") {
    const auto cands = make_candidates();
    const Legacy legacy = legacy_install(cands);

    for (const unsigned workers : {1u, 2u, 4u}) {
        for (const unsigned seed : {1u, 7u, 23u}) {
            INFO("workers " << workers << " seed " << seed);
            auto world = make_world();
            rg::DeckInstaller::Options options;
            options.workers = workers;
            // Per-deck sleep from a hash of (key, seed): decks finish in a different order for every seed; earlier
            // decks regularly finish AFTER later ones.
            options.before_build = [seed](const rg::DeckKey& k) {
                const unsigned h = static_cast<unsigned>(k.way_id * 31 + k.start * 7 + seed * 13) % 6;
                std::this_thread::sleep_for(std::chrono::milliseconds(h * 4));
            };
            std::uint64_t hash = 0;
            std::vector<ps::BodyId> ids;
            {
                rg::DeckInstaller installer(*world, std::move(options));
                const std::size_t n = installer.wait_until_installed(*world, cands);
                REQUIRE(n == cands.size());
                ids = installer.bodies();
                const auto stats = installer.stats();
                CHECK(stats.built == cands.size());
                CHECK(stats.failed == 0);
                CHECK(stats.installed == cands.size());
                hash = settle_hash(*world, cands);
            }
            CHECK(same_ids(ids, legacy.ids));
            CHECK(hash == legacy.hash);
        }
    }
}

TEST_CASE("deck installer: the reference (sim-thread) path gives the same ids and state_hash",
          "[deck_installer][s1][determinism]") {
    const auto cands = make_candidates();
    const Legacy legacy = legacy_install(cands);
    auto world = make_world();
    rg::DeckInstaller installer(*world, rg::DeckInstaller::Options{});
    // One deck per call, like the old per-attempt pacing.
    int calls = 0;
    while (!installer.update_legacy(*world, cands, 1)) ++calls;
    CHECK(calls == static_cast<int>(cands.size())); // each call installs one and reports "not idle"; the next finds nothing
    CHECK(same_ids(installer.bodies(), legacy.ids));
    CHECK(settle_hash(*world, cands) == legacy.hash);
    CHECK(installer.stats().install_ns_total > 0);
}

TEST_CASE("deck installer: the tick path installs in strict key order and holds back behind an unready deck",
          "[deck_installer][s1]") {
    const auto cands = make_candidates(200);
    auto world = make_world();
    rg::DeckInstaller::Options options;
    options.workers = 2;
    options.before_build = [&](const rg::DeckKey& k) {
        // The FIRST key is slow, every later one fast: they are Ready long before it.
        if (k == cands.front().key) std::this_thread::sleep_for(std::chrono::milliseconds(250));
    };
    rg::DeckInstaller installer(*world, std::move(options));

    // Let the fast decks finish, the first still building.
    CHECK_FALSE(installer.update(*world, cands, {}, 4));
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    CHECK_FALSE(installer.update(*world, cands, {}, 4));
    CHECK(installer.bodies().empty()); // nothing installed out of order
    CHECK(installer.stats().waits >= 2);

    // Now drive to completion with a small budget; installed decks always form a key-order prefix.
    std::size_t last = 0;
    for (int guard = 0; guard < 2000; ++guard) {
        const bool done = installer.update(*world, cands, {}, 4);
        const std::size_t n = installer.bodies().size();
        REQUIRE(n >= last);
        REQUIRE(n - last <= 4); // the budget holds
        for (std::size_t i = 0; i < n; ++i) CHECK(installer.installed(cands[i].key));
        for (std::size_t i = n; i < cands.size(); ++i) REQUIRE_FALSE(installer.installed(cands[i].key));
        last = n;
        if (done) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    CHECK(installer.bodies().size() == cands.size());
    CHECK(installer.update(*world, cands, {}, 4)); // idle: done, nothing new
}

TEST_CASE("deck installer: a prefetched deck installs without waiting for a build", "[deck_installer][s1]") {
    const auto cands = make_candidates(200);
    auto world = make_world();
    rg::DeckInstaller installer(*world, rg::DeckInstaller::Options{});

    // First only prefetch everything, give the workers time, then require it all: nothing is left to wait for.
    CHECK(installer.update(*world, {}, cands, 4)); // nothing required: done
    for (int guard = 0; guard < 4000 && installer.stats().built < cands.size(); ++guard)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    REQUIRE(installer.stats().built == cands.size());
    const std::uint64_t waits_before = installer.stats().waits;
    CHECK(installer.update(*world, cands, {}, 0)); // installs all in one call and is done at once
    CHECK(installer.bodies().size() == cands.size());
    CHECK(installer.stats().waits == waits_before);
    CHECK(installer.update(*world, cands, {}, 0));
}

TEST_CASE("deck installer: a deck the backend rejects fails on its worker and does not block the rest",
          "[deck_installer][s1]") {
    auto cands = make_candidates(50);
    // A mesh with no triangles: create_shape throws std::runtime_error on the worker.
    auto bad = make_deck(1001, 0, 0);
    bad->mesh.indices.clear();
    cands[5].deck = bad;
    auto world = make_world();
    rg::DeckInstaller installer(*world, rg::DeckInstaller::Options{});
    const std::size_t n = installer.wait_until_installed(*world, cands);
    CHECK(n == cands.size() - 1);
    CHECK(installer.stats().failed == 1);
    CHECK_FALSE(installer.installed(cands[5].key));
    CHECK(installer.bodies().size() == cands.size() - 1);
}
