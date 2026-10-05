// rg/building_footprints.cpp - see building_footprints.h.
#include "rg/building_footprints.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace rg {

// ------------------------------------------------------- BuildingFootprints --

void BuildingFootprints::clear() {
    prisms_.clear();
    grid_.clear();
}

void BuildingFootprints::add(const std::vector<std::pair<double, double>>& ring, double z_min, double z_max) {
    if (ring.size() < 3 || !(z_max > z_min)) return;
    Prism p;
    p.xs.reserve(ring.size());
    p.ys.reserve(ring.size());
    p.min_x = p.min_y = 1.0e300;
    p.max_x = p.max_y = -1.0e300;
    for (const auto& v : ring) {
        p.xs.push_back(v.first);
        p.ys.push_back(v.second);
        p.min_x = std::min(p.min_x, v.first);
        p.max_x = std::max(p.max_x, v.first);
        p.min_y = std::min(p.min_y, v.second);
        p.max_y = std::max(p.max_y, v.second);
    }
    p.z_min = z_min;
    p.z_max = z_max;
    const auto index = static_cast<std::uint32_t>(prisms_.size());
    const int cx0 = static_cast<int>(std::floor(p.min_x / kCellM)), cx1 = static_cast<int>(std::floor(p.max_x / kCellM));
    const int cy0 = static_cast<int>(std::floor(p.min_y / kCellM)), cy1 = static_cast<int>(std::floor(p.max_y / kCellM));
    // A pathological footprint (a huge multipolygon) is indexed on a coarse cap of cells; queries still test its bbox.
    if (static_cast<long long>(cx1 - cx0 + 1) * (cy1 - cy0 + 1) > 4096) {
        grid_[cell_key(cx0, cy0)].push_back(index);
    } else {
        for (int cy = cy0; cy <= cy1; ++cy)
            for (int cx = cx0; cx <= cx1; ++cx) grid_[cell_key(cx, cy)].push_back(index);
    }
    prisms_.push_back(std::move(p));
}

bool BuildingFootprints::inside(const Prism& p, double x, double y) {
    bool in = false;
    const std::size_t n = p.xs.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = p.xs[i], yi = p.ys[i], xj = p.xs[j], yj = p.ys[j];
        if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) in = !in;
    }
    return in;
}

bool BuildingFootprints::near_point(double x, double y, double z, double margin_m) const {
    if (prisms_.empty()) return false;
    const double m = std::max(0.0, margin_m);
    const int cx0 = static_cast<int>(std::floor((x - m) / kCellM)), cx1 = static_cast<int>(std::floor((x + m) / kCellM));
    const int cy0 = static_cast<int>(std::floor((y - m) / kCellM)), cy1 = static_cast<int>(std::floor((y + m) / kCellM));
    for (int cy = cy0; cy <= cy1; ++cy) {
        for (int cx = cx0; cx <= cx1; ++cx) {
            const auto cell = grid_.find(cell_key(cx, cy));
            if (cell == grid_.end()) continue;
            for (const std::uint32_t index : cell->second) {
                const Prism& p = prisms_[index];
                if (z < p.z_min || z > p.z_max) continue;
                if (x < p.min_x - m || x > p.max_x + m || y < p.min_y - m || y > p.max_y + m) continue;
                if (inside(p, x, y)) return true;
                const std::size_t n = p.xs.size();
                for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
                    const double ex = p.xs[i] - p.xs[j], ey = p.ys[i] - p.ys[j];
                    const double len2 = ex * ex + ey * ey;
                    double t = len2 > 1.0e-12 ? ((x - p.xs[j]) * ex + (y - p.ys[j]) * ey) / len2 : 0.0;
                    t = std::clamp(t, 0.0, 1.0);
                    const double dx = x - (p.xs[j] + ex * t), dy = y - (p.ys[j] + ey * t);
                    if (dx * dx + dy * dy <= m * m) return true;
                }
            }
        }
    }
    return false;
}

bool BuildingFootprints::segment_hits(const Prism& p, double ax, double ay, double az, double bx, double by, double bz) {
    const double dx = bx - ax, dy = by - ay, dz = bz - az;
    // Parameters where the segment crosses the outline, plus the two ends; every interval between consecutive ones lies
    // entirely inside or outside, so its midpoint decides.
    double ts[66];
    int count = 0;
    ts[count++] = 0.0;
    ts[count++] = 1.0;
    const std::size_t n = p.xs.size();
    for (std::size_t i = 0, j = n - 1; i < n && count < 64; j = i++) {
        const double ex = p.xs[i] - p.xs[j], ey = p.ys[i] - p.ys[j];
        const double denom = dx * ey - dy * ex;
        if (std::fabs(denom) < 1.0e-12) continue;
        const double qx = p.xs[j] - ax, qy = p.ys[j] - ay;
        const double t = (qx * ey - qy * ex) / denom;
        const double u = (qx * dy - qy * dx) / denom;
        if (t > 0.0 && t < 1.0 && u >= 0.0 && u <= 1.0) ts[count++] = t;
    }
    std::sort(ts, ts + count);
    for (int k = 0; k + 1 < count; ++k) {
        if (ts[k + 1] - ts[k] < 1.0e-9) continue;
        const double tm = 0.5 * (ts[k] + ts[k + 1]);
        if (!inside(p, ax + dx * tm, ay + dy * tm)) continue;
        const double z0 = az + dz * ts[k], z1 = az + dz * ts[k + 1];
        if (std::max(z0, z1) >= p.z_min && std::min(z0, z1) <= p.z_max) return true;
    }
    return false;
}

bool BuildingFootprints::blocks_segment(double ax, double ay, double az, double bx, double by, double bz) const {
    if (prisms_.empty()) return false;
    const double len = std::sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
    const int samples = std::clamp(static_cast<int>(std::ceil(len / 4.0)), 1, 128);
    std::int64_t keys[640];
    int key_count = 0;
    const double min_x = std::min(ax, bx), max_x = std::max(ax, bx), min_y = std::min(ay, by), max_y = std::max(ay, by);
    for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        const double x = ax + (bx - ax) * t, y = ay + (by - ay) * t;
        // The sample cell and the cells 2 m around it: a short chord through a cell corner is never missed.
        for (const double ox : {-2.0, 2.0}) {
            for (const double oy : {-2.0, 2.0}) {
                const std::int64_t key =
                    cell_key(static_cast<int>(std::floor((x + ox) / kCellM)), static_cast<int>(std::floor((y + oy) / kCellM)));
                bool seen = false;
                for (int k = 0; k < key_count && !seen; ++k) seen = keys[k] == key;
                if (!seen && key_count < 640) keys[key_count++] = key;
            }
        }
    }
    for (int k = 0; k < key_count; ++k) {
        const auto cell = grid_.find(keys[k]);
        if (cell == grid_.end()) continue;
        for (const std::uint32_t index : cell->second) {
            const Prism& p = prisms_[index];
            if (p.max_x < min_x || p.min_x > max_x || p.max_y < min_y || p.min_y > max_y) continue;
            if (std::max(az, bz) < p.z_min || std::min(az, bz) > p.z_max) continue;
            if (segment_hits(p, ax, ay, az, bx, by, bz)) return true;
        }
    }
    return false;
}

// -------------------------------------------------------- BuildingObstacles --

namespace {
constexpr std::size_t kMaxLoadedTiles = 25;
}

BuildingObstacles::BuildingObstacles(double e0, double n0, TileFetch fetch, GroundFn ground, double tile_m)
    : e0_(e0), n0_(n0), tile_m_(tile_m), fetch_(std::move(fetch)), ground_(std::move(ground)) {}

BuildingObstacles::~BuildingObstacles() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    work_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

std::pair<int, int> BuildingObstacles::tile_of(double x, double y) const {
    return {static_cast<int>(std::floor((x + e0_) / tile_m_)), static_cast<int>(std::floor((y + n0_) / tile_m_))};
}

void BuildingObstacles::request_around(double x, double y) {
    const auto center = tile_of(x, y);
    std::lock_guard<std::mutex> lock(mutex_);
    center_ = center;
    if (!fetch_) return;
    bool queued = false;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const std::pair<int, int> key{center.first + dx, center.second + dy};
            if (tiles_.count(key)) continue;
            if (std::find(queue_.begin(), queue_.end(), key) != queue_.end()) continue;
            queue_.push_back(key);
            queued = true;
        }
    }
    if (!queued) return;
    // Nearest tile first: the one under the car is the one the first shot needs.
    std::sort(queue_.begin(), queue_.end(), [&](const auto& a, const auto& b) {
        const auto d = [&](const auto& k) {
            return (k.first - center.first) * (k.first - center.first) + (k.second - center.second) * (k.second - center.second);
        };
        return d(a) < d(b);
    });
    if (!worker_.joinable()) worker_ = std::thread([this] { worker_loop(); });
    work_cv_.notify_one();
}

void BuildingObstacles::worker_loop() {
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
        work_cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
        if (stop_) return;
        const auto key = queue_.front();
        queue_.erase(queue_.begin());
        busy_ = true;
        lock.unlock();
        std::vector<Building> buildings;
        try {
            buildings = fetch_(key.first, key.second);
        } catch (...) {
            buildings.clear(); // a failed tile counts as "no buildings": the director then behaves as before
        }
        add_tile(key.first, key.second, buildings);
        lock.lock();
        busy_ = false;
        idle_cv_.notify_all();
    }
}

void BuildingObstacles::add_tile(int tile_x, int tile_y, const std::vector<Building>& buildings) {
    auto footprints = std::make_unique<BuildingFootprints>();
    for (const auto& b : buildings) {
        const double z_max = b.base + b.height;
        for (const auto& ring : b.outers) {
            std::vector<std::pair<double, double>> pts;
            pts.reserve(ring.size());
            for (const auto& p : ring) pts.emplace_back(p.e - e0_, p.n - n0_);
            footprints->add(pts, b.bottom, z_max);
        }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    tiles_[{tile_x, tile_y}] = std::move(footprints);
    evict_locked();
}

void BuildingObstacles::evict_locked() {
    while (tiles_.size() > kMaxLoadedTiles) {
        auto farthest = tiles_.begin();
        long long best = -1;
        for (auto it = tiles_.begin(); it != tiles_.end(); ++it) {
            const long long dx = it->first.first - center_.first, dy = it->first.second - center_.second;
            if (dx * dx + dy * dy > best) {
                best = dx * dx + dy * dy;
                farthest = it;
            }
        }
        tiles_.erase(farthest);
    }
}

std::size_t BuildingObstacles::loaded_tile_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return tiles_.size();
}

std::size_t BuildingObstacles::footprint_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t n = 0;
    for (const auto& t : tiles_) n += t.second->size();
    return n;
}

bool BuildingObstacles::wait_idle(double timeout_s) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return idle_cv_.wait_for(lock, std::chrono::duration<double>(timeout_s), [this] { return queue_.empty() && !busy_; });
}

std::optional<double> BuildingObstacles::ground_height(double x, double y) const {
    return ground_ ? ground_(x, y) : std::nullopt;
}

bool BuildingObstacles::building_covers(double x, double y, double z, double margin_m) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& t : tiles_)
        if (t.second->near_point(x, y, z, margin_m)) return true;
    return false;
}

bool BuildingObstacles::building_blocks(double ax, double ay, double az, double bx, double by, double bz) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& t : tiles_)
        if (t.second->blocks_segment(ax, ay, az, bx, by, bz)) return true;
    return false;
}

bool BuildingObstacles::ready(double x, double y) const {
    const auto key = tile_of(x, y);
    std::lock_guard<std::mutex> lock(mutex_);
    return tiles_.count(key) != 0;
}

} // namespace rg
