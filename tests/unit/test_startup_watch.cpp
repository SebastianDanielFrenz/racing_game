// test_startup_watch.cpp - the start-up timeout rules (rg::StartupWatch) and the first terrain-view build's
// progress reporting (rg::ViewBuildProgress), the two pieces behind the loading screen's "never frozen" guarantee.
#include "rg/startup_watch.h"
#include "rg/synthetic_terrain.h"
#include "rg/terrain_view_streamer.h"
#include "rg/view_build_progress.h"

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <vector>

TEST_CASE("StartupWatch: a slow but steadily progressing start never times out", "[startup_watch]") {
    rg::StartupWatch watch(rg::StartupWatchConfig{30.0, 0.0}); // no absolute cap
    // 10 minutes of derive, one tile finishing every 10 s: far beyond the old 30 s wall-clock limit.
    for (int t = 0; t <= 600; t += 5) {
        rg::StartupSignature sig;
        sig.missing_required = static_cast<std::uint32_t>(100 - t / 10);
        sig.resident = static_cast<std::uint32_t>(t / 10);
        CHECK(watch.observe(t, sig) == rg::StartupVerdict::Waiting);
    }
}

TEST_CASE("StartupWatch: no change for the no-progress window fails, any change resets it", "[startup_watch]") {
    rg::StartupWatch watch(rg::StartupWatchConfig{30.0, 0.0});
    rg::StartupSignature sig;
    sig.missing_required = 42;
    sig.inflight = 78;
    CHECK(watch.observe(0.0, sig) == rg::StartupVerdict::Waiting);
    CHECK(watch.observe(29.0, sig) == rg::StartupVerdict::Waiting);
    sig.inflight = 77; // something finished: the window restarts at t = 29.5
    CHECK(watch.observe(29.5, sig) == rg::StartupVerdict::Waiting);
    CHECK(watch.observe(59.0, sig) == rg::StartupVerdict::Waiting);
    CHECK(watch.observe(59.6, sig) == rg::StartupVerdict::NoProgress);
}

TEST_CASE("StartupWatch: the absolute cap fires even while progressing", "[startup_watch]") {
    rg::StartupWatch watch(rg::StartupWatchConfig{30.0, 100.0});
    rg::StartupSignature sig;
    for (int t = 0; t <= 100; t += 10) {
        sig.resident = static_cast<std::uint32_t>(t);
        CHECK(watch.observe(t, sig) == rg::StartupVerdict::Waiting);
    }
    sig.resident = 999;
    CHECK(watch.observe(100.5, sig) == rg::StartupVerdict::CapExceeded);
}

TEST_CASE("ViewBuildProgress: total is announced up front and done reaches it when the build ends",
          "[startup_watch][terrain_view_streamer]") {
    rg::SyntheticTerrain terrain(464000.0, 5559000.0);
    g2m::mesh::LodParams params;
    params.max_distance_m = 3000.0;
    rg::TerrainViewSource source = terrain.view_source(params);

    auto progress = std::make_shared<rg::ViewBuildProgress>();
    const std::uint32_t announced = rg::announce_view_build_total(source, 0.0, 0.0, *progress);
    CHECK(announced > 0);
    CHECK(progress->total.load() == announced);
    CHECK(progress->done.load() == 0);

    rg::attach_view_build_progress(source, progress);
    rg::TerrainViewStreamer streamer(source, rg::TerrainViewStreamer::Options{});
    std::vector<rg::RenderChunk> chunks;
    REQUIRE(streamer.build_initial(0.0, 0.0, chunks));
    CHECK(chunks.size() == announced);
    CHECK(progress->done.load() == announced);
}
