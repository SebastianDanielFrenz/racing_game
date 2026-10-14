// rg/map_view.cpp — see map_view.h.
#include "rg/map_view.h"

#include <memory>
#include <utility>

namespace rg {

MapViewSelection select_map_view(const g2m::route::MapConfig& config, g2m::geo::UtmZone zone, double e0, double n0,
                                 double cx, double cy, double radius_m) {
    MapViewSelection s;
    s.lod = g2m::route::select_map_lod(config, radius_m);
    const int level = g2m::route::effective_lod(config, s.lod).level;
    s.keys = tiles_in_square(zone, level, e0, n0, cx, cy, radius_m);
    return s;
}

MapViewFrame build_map_view(g2m::route::MapTileCache& cache, const RoadGraphTileSource& source,
                            const MapViewSelection& selection, double e0, double n0) {
    MapViewFrame frame;
    frame.lod = selection.lod;
    for (const g2m::TileKey& key : selection.keys) {
        const g2m::route::MapTile* tile = cache.find(key, selection.lod);
        if (tile == nullptr) {
            // The level-2 roads.graph tiles inside this map tile.
            const int n = key.level >= 2 ? 1 << (key.level - 2) : 1;
            std::vector<std::shared_ptr<const g2m::RoadGraphTile>> held;
            std::vector<const g2m::RoadGraphTile*> ptrs;
            bool failed = false;
            for (int dy = 0; dy < n && !failed; ++dy) {
                for (int dx = 0; dx < n; ++dx) {
                    const g2m::TileKey l2{key.zone, 2, key.x * n + dx, key.y * n + dy};
                    std::shared_ptr<const g2m::RoadGraphTile> t = source(l2);
                    if (t == nullptr) {
                        failed = true;
                        break;
                    }
                    if (t->graph.ways.empty() && t->graph.nodes.empty()) continue; // no data there
                    held.push_back(std::move(t));
                    ptrs.push_back(held.back().get());
                }
            }
            if (failed) {
                ++frame.failed_tiles;
                continue;
            }
            auto built = cache.get(key, selection.lod, ptrs);
            if (!built) {
                ++frame.failed_tiles;
                continue;
            }
            tile = built.value();
        }
        ++frame.tiles;
        const double ox = static_cast<double>(tile->key.min_easting()) - e0;
        const double oy = static_cast<double>(tile->key.min_northing()) - n0;
        for (const g2m::route::MapLine& l : tile->lines) {
            MapViewLine v;
            v.way_id = l.way_id;
            v.rank = l.rank;
            v.flags = l.flags;
            if (l.name != g2m::route::kMapNoString) v.name = tile->strings[l.name];
            if (l.ref != g2m::route::kMapNoString) v.ref = tile->strings[l.ref];
            v.point_first = static_cast<std::uint32_t>(frame.points.size());
            v.point_count = l.point_count;
            for (std::uint32_t k = 0; k < l.point_count; ++k) {
                const g2m::route::MapPoint& p = tile->points[l.point_first + k];
                frame.points.push_back({static_cast<float>(ox + p.x * 0.1), static_cast<float>(oy + p.y * 0.1)});
            }
            frame.lines.push_back(std::move(v));
        }
    }
    return frame;
}

MapView::MapView(RoadGraphTileSource source, g2m::geo::UtmZone zone, double e0, double n0,
                 g2m::route::MapConfig config, std::size_t max_cached_tiles)
    : source_(std::move(source)), zone_(zone), e0_(e0), n0_(n0), max_cached_tiles_(max_cached_tiles),
      config_(std::move(config)) {
    worker_ = std::thread([this]() { worker_main(); });
}

MapView::~MapView() {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

bool MapView::set_config(const g2m::route::MapConfig& config) {
    if (!g2m::route::validate_map_config(config)) return false;
    std::lock_guard<std::mutex> lk(mutex_);
    config_ = config;
    config_dirty_ = true;
    if (has_request_ || !request_done_ || frame_.revision > 0) request_done_ = false; // rebuild the current view
    cv_.notify_all();
    return true;
}

void MapView::request(double cx, double cy, double radius_m) {
    std::lock_guard<std::mutex> lk(mutex_);
    request_ = {cx, cy, radius_m};
    has_request_ = true;
    request_done_ = false;
    cv_.notify_all();
}

bool MapView::poll(MapViewFrame& out) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!has_frame_) return false;
    out = std::move(frame_);
    frame_ = MapViewFrame{};
    frame_.revision = out.revision;
    has_frame_ = false;
    return true;
}

bool MapView::wait(MapViewFrame& out) {
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait(lk, [this]() { return stop_ || (request_done_ && has_request_); });
    if (!has_frame_) return false;
    out = std::move(frame_);
    frame_ = MapViewFrame{};
    frame_.revision = out.revision;
    has_frame_ = false;
    return true;
}

void MapView::worker_main() {
    g2m::route::MapTileCache cache(config_);
    MapViewSelection last;
    bool has_last = false;
    std::uint64_t revision = 0;
    for (;;) {
        std::array<double, 3> req{};
        g2m::route::MapConfig config;
        bool config_changed = false;
        {
            std::unique_lock<std::mutex> lk(mutex_);
            cv_.wait(lk, [this]() { return stop_ || (has_request_ && !request_done_); });
            if (stop_) return;
            req = request_;
            config_changed = config_dirty_;
            config_dirty_ = false;
            if (config_changed) config = config_;
        }
        if (config_changed && !cache.set_config(config)) config_changed = false;
        const MapViewSelection selection =
            select_map_view(cache.config(), zone_, e0_, n0_, req[0], req[1], req[2]);
        if (has_last && !config_changed && selection == last) {
            std::lock_guard<std::mutex> lk(mutex_);
            if (request_ == req) request_done_ = true;
            cv_.notify_all();
            continue;
        }
        if (cache.size() > max_cached_tiles_) cache.clear();
        const std::size_t builds_before = cache.builds();
        MapViewFrame frame = build_map_view(cache, source_, selection, e0_, n0_);
        builds_.fetch_add(cache.builds() - builds_before, std::memory_order_relaxed);
        frame.revision = ++revision;
        frame.center_x = req[0];
        frame.center_y = req[1];
        frame.radius_m = req[2];
        last = selection;
        has_last = frame.failed_tiles == 0; // retry failed tiles with the next request
        {
            std::lock_guard<std::mutex> lk(mutex_);
            frame_ = std::move(frame);
            has_frame_ = true;
            if (request_ == req) request_done_ = true;
        }
        cv_.notify_all();
    }
}

} // namespace rg
