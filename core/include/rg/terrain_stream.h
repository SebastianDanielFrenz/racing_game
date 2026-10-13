// rg/terrain_stream.h - engine-neutral helpers of the render terrain stream
// that matter at high speed (owner complaint 2026-10-13: at 400-580 km/h the
// visible terrain does not resolve in time, the road is not visible far ahead).
// No Godot type; godot_ext/RgTerrainView is a thin consumer.
//
//   adaptive_upload_budget_ms   per-frame upload time budget: base when idle,
//                               rising with the backlog and with the speed.
//   chunk_time_to_reach_s /     order of a diff's added chunks: what the car
//   sort_adds_by_urgency        reaches first is uploaded first (finest level
//                               first within a time bin).
//   lod_coverage_ahead          how far ahead of the car the ground is covered
//                               by chunks of level <= k (the L0/L1 "ahead"
//                               numbers of the RG_TERRAIN_STREAM log line).
//   grid_coverage_ahead         the same for a square tile grid (road ribbons).
//   chunk_surface_height        the exact height of a render chunk's mesh at an
//                               absolute UTM point (visible ground vs physics).
//   SampleWindow                a per-report sample window (avg / max / p95).
#pragma once

#include "rg/terrain_render.h"

#include "g2m/mesh/terrain_chunk.h"

#include <array>
#include <cstddef>
#include <vector>

namespace rg {

// ---------------------------------------------------------------------------
// Upload budget
// ---------------------------------------------------------------------------
struct UploadBudgetParams {
    double base_ms = 0.8;       // idle / small backlog
    double max_ms = 3.0;        // the most a frame may spend uploading
    double backlog_lo = 8.0;    // pending chunks at which the budget starts to rise
    double backlog_hi = 64.0;   // ... and reaches max_ms
    double speed_lo_mps = 30.0; // speed at which the budget starts to rise (only with pending work)
    double speed_hi_mps = 120.0;
};

// base_ms with nothing pending; otherwise base_ms + (max_ms - base_ms) * the
// larger of the backlog factor and the speed factor, both linear in [0, 1].
[[nodiscard]] double adaptive_upload_budget_ms(std::size_t pending, double speed_mps,
                                               const UploadBudgetParams& params = {});

// ---------------------------------------------------------------------------
// Reselect distance
// ---------------------------------------------------------------------------
// How far the focus must move before the streamer starts the next selection.
// The finest level reaches range0 around the focus, so the L0 coverage ahead of
// a car oscillates between (lead + range0) right after a diff and
// (lead + range0 - reselect) just before the next one: at speed the base distance
// is shrunk linearly to `min_fraction` of it between `speed_lo_mps` and
// `speed_hi_mps` (more, smaller diffs; each costs ~150 ms on the build threads).
// Speed 0 (no velocity given) is the base distance, so callers without a
// velocity are unchanged.
[[nodiscard]] double reselect_distance_for_speed(double base_m, double speed_mps, double min_fraction = 0.5,
                                                 double speed_lo_mps = 40.0, double speed_hi_mps = 100.0);

// ---------------------------------------------------------------------------
// Diff order
// ---------------------------------------------------------------------------
// Seconds until the car at (px, py) (session-local) moving with (vx, vy)
// reaches the chunk's footprint: distance ahead along the velocity (lateral
// gap folded in) over speed. A chunk BEHIND the car gets 1000 s plus its
// distance (uploaded after everything ahead). Below 2 m/s of speed there is no
// direction: the plain distance to the footprint over 2 m/s.
[[nodiscard]] double chunk_time_to_reach_s(const g2m::mesh::ChunkKey& key, double e0, double n0, double px, double py,
                                           double vx, double vy);

// Order for uploading `added`: time bins of 0.25 s, then finer level first,
// then time, then key (deterministic). Callers without a velocity simply do
// not call it and keep the key order.
void sort_adds_by_urgency(std::vector<RenderChunk>& added, double e0, double n0, double px, double py, double vx,
                          double vy);

// ---------------------------------------------------------------------------
// Coverage ahead
// ---------------------------------------------------------------------------
inline constexpr int kLodCoverageLevels = 8;

struct LodCoverage {
    // ahead_m[k]: distance from the start point along the direction up to
    // which EVERY sample point (step_m apart) is covered by a chunk of level
    // <= k; max_m if the whole ray is covered, 0 if the start itself is not.
    std::array<double, kLodCoverageLevels> ahead_m{};
};

// `keys`: the chunks that are currently VISIBLE (uploaded), any order.
// (px, py) session-local start, (dx, dy) direction (normalised inside; a zero
// vector yields all-zero coverage), e0/n0 turn session-local into absolute UTM.
[[nodiscard]] LodCoverage lod_coverage_ahead(const std::vector<g2m::mesh::ChunkKey>& keys, double e0, double n0,
                                             double px, double py, double dx, double dy, double max_m = 3000.0,
                                             double step_m = 4.0);

// Distance ahead until the first square tile (tile_m side, indexed
// floor(absolute / tile_m)) of the ray that is not in `tiles` ((ix, iy) pairs).
// (px, py) is absolute here (the caller adds its UTM origin). max_m if all are
// present, 0 if the first is missing. step_m should be well below tile_m.
[[nodiscard]] double grid_coverage_ahead(const std::vector<std::array<int, 2>>& tiles, double tile_m, double px,
                                         double py, double dx, double dy, double max_m, double step_m);

// ---------------------------------------------------------------------------
// Visible surface height
// ---------------------------------------------------------------------------
// The height of the chunk's rendered surface (the same two triangles per quad
// build_chunk emits) at the absolute UTM point (e_abs, n_abs). False if the
// point is outside the chunk or touches a NoData vertex.
[[nodiscard]] bool chunk_surface_height(const RenderChunk& chunk, double e_abs, double n_abs, double& z_out);

// ---------------------------------------------------------------------------
// Sample window
// ---------------------------------------------------------------------------
class SampleWindow {
public:
    explicit SampleWindow(std::size_t capacity = 4096) : capacity_(capacity == 0 ? 1 : capacity) {}
    void add(double value);
    void clear();
    [[nodiscard]] std::size_t count() const { return samples_.size(); }
    [[nodiscard]] double mean() const;
    [[nodiscard]] double max() const;
    // Nearest-rank percentile, q in [0, 1]; 0 for an empty window.
    [[nodiscard]] double percentile(double q) const;

private:
    std::size_t capacity_;
    std::size_t next_ = 0; // ring cursor once full
    std::vector<double> samples_;
};

} // namespace rg
