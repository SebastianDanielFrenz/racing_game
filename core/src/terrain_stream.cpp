// rg/terrain_stream.cpp - see rg/terrain_stream.h.
#include <chrono>
#include "rg/terrain_stream.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>

namespace rg {

namespace {

constexpr double kNoDataZ = -1.0e6;

double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }

} // namespace

double adaptive_upload_budget_ms(std::size_t pending, double speed_mps, const UploadBudgetParams& p) {
    if (pending == 0) {
        return p.base_ms;
    }
    const double backlog =
        clamp01((static_cast<double>(pending) - p.backlog_lo) / std::max(1e-9, p.backlog_hi - p.backlog_lo));
    const double speed = clamp01((speed_mps - p.speed_lo_mps) / std::max(1e-9, p.speed_hi_mps - p.speed_lo_mps));
    return p.base_ms + (std::max(p.max_ms, p.base_ms) - p.base_ms) * std::max(backlog, speed);
}

LodCoverage lod_coverage_ahead(const std::vector<g2m::mesh::ChunkKey>& keys, double e0, double n0, double px,
                               double py, double dx, double dy, double max_m, double step_m) {
    LodCoverage out;
    const double len = std::hypot(dx, dy);
    if (keys.empty() || !(len > 1e-9) || !(step_m > 0.0)) {
        return out;
    }
    const double ux = dx / len;
    const double uy = dy / len;
    std::vector<g2m::mesh::ChunkKey> sorted = keys;
    std::sort(sorted.begin(), sorted.end());
    const auto zone = sorted.front().zone;

    std::array<bool, kLodCoverageLevels> open;
    open.fill(true);
    out.ahead_m.fill(max_m);
    const int steps = static_cast<int>(std::floor(max_m / step_m));
    for (int s = 0; s <= steps; ++s) {
        const double t = s * step_m;
        const double e = px + e0 + ux * t;
        const double n = py + n0 + uy * t;
        int best = kLodCoverageLevels; // finest covering level at this sample (kLodCoverageLevels = none)
        for (int k = 0; k < kLodCoverageLevels; ++k) {
            const double size = static_cast<double>(g2m::mesh::kChunkQuads) * std::ldexp(1.0, k);
            const g2m::mesh::ChunkKey key{zone, static_cast<std::uint8_t>(k),
                                          static_cast<std::int32_t>(std::floor(e / size)),
                                          static_cast<std::int32_t>(std::floor(n / size))};
            if (std::binary_search(sorted.begin(), sorted.end(), key)) {
                best = k;
                break;
            }
        }
        for (int k = 0; k < kLodCoverageLevels; ++k) {
            if (open[static_cast<std::size_t>(k)] && best > k) {
                open[static_cast<std::size_t>(k)] = false;
                out.ahead_m[static_cast<std::size_t>(k)] = t;
            }
        }
    }
    return out;
}

double grid_coverage_ahead(const std::vector<std::array<int, 2>>& tiles, double tile_m, double px, double py,
                           double dx, double dy, double max_m, double step_m) {
    const double len = std::hypot(dx, dy);
    if (!(len > 1e-9) || !(step_m > 0.0) || !(tile_m > 0.0)) {
        return 0.0;
    }
    std::vector<std::array<int, 2>> sorted = tiles;
    std::sort(sorted.begin(), sorted.end());
    const double ux = dx / len;
    const double uy = dy / len;
    const int steps = static_cast<int>(std::floor(max_m / step_m));
    for (int s = 0; s <= steps; ++s) {
        const double t = s * step_m;
        const std::array<int, 2> key{static_cast<int>(std::floor((px + ux * t) / tile_m)),
                                     static_cast<int>(std::floor((py + uy * t) / tile_m))};
        if (!std::binary_search(sorted.begin(), sorted.end(), key)) {
            return t;
        }
    }
    return max_m;
}

bool chunk_surface_height(const RenderChunk& chunk, double e_abs, double n_abs, double& z_out) {
    const auto& mesh = chunk.mesh;
    constexpr int quads = g2m::mesh::kChunkQuads;
    constexpr int verts = g2m::mesh::kChunkVerts;
    if (mesh.positions.size() < static_cast<std::size_t>(verts) * verts * 3) {
        return false;
    }
    const double spacing = std::ldexp(1.0, mesh.key.level);
    const double u = (e_abs - mesh.origin[0]) / spacing;
    const double v = (n_abs - mesh.origin[1]) / spacing;
    if (!(u >= 0.0 && u <= quads && v >= 0.0 && v <= quads)) {
        return false;
    }
    const int i = std::min(static_cast<int>(std::floor(u)), quads - 1);
    const int j = std::min(static_cast<int>(std::floor(v)), quads - 1);
    const double tx = u - i;
    const double ty = v - j;
    const auto z_at = [&mesh](int a, int b) {
        return static_cast<double>(mesh.positions[3 * static_cast<std::size_t>(b * verts + a) + 2]) + mesh.origin[2];
    };
    const double h00 = z_at(i, j);
    const double h10 = z_at(i + 1, j);
    const double h01 = z_at(i, j + 1);
    const double h11 = z_at(i + 1, j + 1);
    if (tx + ty <= 1.0) { // SW triangle (i,j) (i+1,j) (i,j+1)
        if (h00 < kNoDataZ || h10 < kNoDataZ || h01 < kNoDataZ) {
            return false;
        }
        z_out = h00 + tx * (h10 - h00) + ty * (h01 - h00);
    } else { // NE triangle (i+1,j) (i+1,j+1) (i,j+1)
        if (h10 < kNoDataZ || h11 < kNoDataZ || h01 < kNoDataZ) {
            return false;
        }
        z_out = h11 + (1.0 - tx) * (h01 - h11) + (1.0 - ty) * (h10 - h11);
    }
    return true;
}

BedCheck rendered_vs_bed(const std::vector<RenderChunk>& chunks, std::size_t count,
                       const std::vector<std::array<double, 3>>& beds, double tolerance_m) {
    BedCheck r;
    r.above_max_by_level.fill(-1000.0);
    std::vector<double> above;
    above.reserve(beds.size());
    const std::size_t n = std::min(count, chunks.size());
    // Contiguous chunk footprints: the plain scan touched every RenderChunk (~8 us per bed point at 1500 chunks, so a
    // few thousand bed points cost tens of ms on the main thread once per second); the bounds prefilter is ~1 us.
    struct Bounds { double e0, e1, n0, n1; };
    std::vector<Bounds> bounds(n);
    for (std::size_t i = 0; i < n; ++i) {
        const auto& mesh = chunks[i].mesh;
        const double extent = std::ldexp(1.0, mesh.key.level) * g2m::mesh::kChunkQuads;
        bounds[i] = {mesh.origin[0], mesh.origin[0] + extent, mesh.origin[1], mesh.origin[1] + extent};
    }
    for (const auto& bed : beds) {
        double visible = -1.0e9;
        std::size_t best = n;
        for (std::size_t i = 0; i < n; ++i) {
            const Bounds& b = bounds[i];
            if (bed[0] < b.e0 || bed[0] > b.e1 || bed[1] < b.n0 || bed[1] > b.n1) continue;
            double z = 0.0;
            if (chunk_surface_height(chunks[i], bed[0], bed[1], z) && z > visible) {
                visible = z;
                best = i;
            }
        }
        if (best == n) continue;
        const double d = visible - bed[2];
        const int level = std::min<int>(chunks[best].key.level, kLodCoverageLevels - 1);
        ++r.samples;
        ++r.samples_by_level[static_cast<std::size_t>(level)];
        above.push_back(d);
        r.above_max_by_level[static_cast<std::size_t>(level)] = std::max(r.above_max_by_level[static_cast<std::size_t>(level)], d);
        if (d > tolerance_m) ++r.above_tolerance;
        if (d > r.above_max_m) {
            r.above_max_m = d;
            r.worst_level = level;
            r.worst_carve = chunks[best].carve;
            r.worst_e = bed[0];
            r.worst_n = bed[1];
        }
    }
    if (!above.empty()) {
        std::sort(above.begin(), above.end());
        r.above_p95_m = above[std::min(above.size() - 1, static_cast<std::size_t>(std::ceil(0.95 * above.size())) - 1)];
    }
    return r;
}

std::vector<std::size_t> stale_fallback_chunks(const std::vector<RenderChunk>& chunks, std::size_t count,
                                               const std::function<ChunkCarve(const g2m::mesh::ChunkKey&)>& state_now) {
    std::vector<std::size_t> stale;
    const std::size_t n = std::min(count, chunks.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (chunks[i].carve == ChunkCarve::Fallback && state_now(chunks[i].key) != ChunkCarve::Fallback) stale.push_back(i);
    }
    return stale;
}

bool ChunkRebuildJob::start(std::vector<g2m::mesh::ChunkKey> keys, BuildFn build) {
    if (future_.valid() || keys.empty() || !build) return false;
    cancel_.store(false, std::memory_order_relaxed);
    future_ = std::async(std::launch::async, [this, keys = std::move(keys), build = std::move(build)]() {
        std::vector<RenderChunk> built;
        if (!build(keys, built, &cancel_) || built.size() != keys.size()) built.clear();
        return built;
    });
    return true;
}

bool ChunkRebuildJob::take(std::vector<RenderChunk>& out) {
    if (!future_.valid() || future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return false;
    std::vector<RenderChunk> built = future_.get();
    if (built.empty()) return false;
    out = std::move(built);
    return true;
}

void ChunkRebuildJob::cancel_and_join() {
    if (!future_.valid()) return;
    cancel_.store(true, std::memory_order_relaxed);
    future_.wait();
    future_ = {};
}

void SampleWindow::add(double value) {
    if (samples_.size() < capacity_) {
        samples_.push_back(value);
        return;
    }
    samples_[next_] = value;
    next_ = (next_ + 1) % capacity_;
}

void SampleWindow::clear() {
    samples_.clear();
    next_ = 0;
}

double SampleWindow::mean() const {
    if (samples_.empty()) {
        return 0.0;
    }
    return std::accumulate(samples_.begin(), samples_.end(), 0.0) / static_cast<double>(samples_.size());
}

double SampleWindow::max() const {
    if (samples_.empty()) {
        return 0.0;
    }
    return *std::max_element(samples_.begin(), samples_.end());
}

double SampleWindow::percentile(double q) const {
    if (samples_.empty()) {
        return 0.0;
    }
    std::vector<double> sorted = samples_;
    std::sort(sorted.begin(), sorted.end());
    const double rank = std::ceil(std::clamp(q, 0.0, 1.0) * static_cast<double>(sorted.size()));
    const std::size_t index = rank <= 1.0 ? 0 : static_cast<std::size_t>(rank) - 1;
    return sorted[std::min(index, sorted.size() - 1)];
}

} // namespace rg
