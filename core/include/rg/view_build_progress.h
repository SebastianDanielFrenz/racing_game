// rg/view_build_progress.h - progress of the first terrain-view build (the render chunks around the spawn).
//
// The initial build (TerrainViewStreamer::build_initial) derives every coarse LOD tile the view needs and, on a cold
// store, takes tens of seconds to minutes. It runs on a worker; this counter is what the loading screen shows.
// `total` = the number of chunks the first selection holds, `done` = chunks built so far (bumped from the build
// threads through ViewSource::colour, which runs once per built chunk).
#pragma once

#include "rg/world_terrain.h"

#include <atomic>
#include <cstdint>
#include <memory>

namespace rg {

struct ViewBuildProgress {
    std::atomic<std::uint32_t> done{0};
    std::atomic<std::uint32_t> total{0};
};

// Wraps source.colour so every built chunk increments progress->done (the original hook still runs first).
void attach_view_build_progress(TerrainViewSource& source, const std::shared_ptr<ViewBuildProgress>& progress);

// Number of chunks the first selection around the session-local point holds; stores it in progress->total.
std::uint32_t announce_view_build_total(const TerrainViewSource& source, double x, double y,
                                        ViewBuildProgress& progress);

} // namespace rg
