// rg/map_view.h — R4 minimap / map geometry: g2m::route map tiles
// (build_map_tile through MapTileCache, geo2map docs/routing_design.md 6)
// around a view centre, flattened into one view-model of polylines in
// session metres that an engine adapter only has to draw.
//
// Engine-neutral. One worker thread owns the MapTileCache (not thread-safe)
// and does every tile fetch and build; the adapter calls request() every
// frame (cheap; a request whose tile set, LOD and config did not change is a
// no-op) and poll()s for a new frame. set_config() is the in-game "map
// detail" setting (MapConfig::detail_scale, rg/lod_settings.h map_config_for):
// the cache drops only the LODs whose map_lod_key changed and the current
// view is rebuilt.
#pragma once

#include "rg/route_planner.h"

#include "g2m/route/route_map.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rg {

struct MapViewLine {
    std::int64_t way_id = 0;
    int rank = 0;              // RoadStyle rank (track 0 .. motorway 9)
    std::uint8_t flags = 0;    // g2m::route::MapLineFlags
    std::string name, ref;
    std::uint32_t point_first = 0, point_count = 0; // into MapViewFrame::points
};

struct MapViewFrame {
    std::uint64_t revision = 0; // increases with every published frame
    g2m::route::MapLod lod = g2m::route::MapLod::Detail;
    double center_x = 0.0, center_y = 0.0, radius_m = 0.0; // the request it answers
    std::size_t tiles = 0;      // map tiles drawn
    std::size_t failed_tiles = 0;
    std::vector<MapViewLine> lines;
    std::vector<std::array<float, 2>> points; // session metres
};

// The map tiles (of the LOD's level) a view of `radius_m` around (cx, cy) needs, and its LOD.
struct MapViewSelection {
    g2m::route::MapLod lod = g2m::route::MapLod::Detail;
    std::vector<g2m::TileKey> keys;
    friend bool operator==(const MapViewSelection&, const MapViewSelection&) = default;
};
MapViewSelection select_map_view(const g2m::route::MapConfig& config, g2m::geo::UtmZone zone, double e0, double n0,
                                 double cx, double cy, double radius_m);

// Builds (through `cache`) and flattens the tiles of `selection`; each map tile is assembled from the level-2
// roads.graph tiles inside it, fetched from `source` (a null tile is skipped and counted as failed). Synchronous.
MapViewFrame build_map_view(g2m::route::MapTileCache& cache, const RoadGraphTileSource& source,
                            const MapViewSelection& selection, double e0, double n0);

class MapView {
public:
    MapView(RoadGraphTileSource source, g2m::geo::UtmZone zone, double e0, double n0,
            g2m::route::MapConfig config = g2m::route::default_map_config(), std::size_t max_cached_tiles = 512);
    ~MapView();
    MapView(const MapView&) = delete;
    MapView& operator=(const MapView&) = delete;

    // False (old config kept) when geo2map's validate_map_config refuses it.
    bool set_config(const g2m::route::MapConfig& config);
    // Non-blocking: the latest request wins.
    void request(double cx, double cy, double radius_m);
    // The newest frame not yet polled.
    bool poll(MapViewFrame& out);
    // Blocks until a frame for the latest request (or a stop) exists; tests.
    bool wait(MapViewFrame& out);
    [[nodiscard]] std::size_t tile_builds() const { return builds_.load(std::memory_order_relaxed); }

private:
    void worker_main();

    RoadGraphTileSource source_;
    g2m::geo::UtmZone zone_;
    double e0_, n0_;
    std::size_t max_cached_tiles_; // the cache is cleared before a build once it holds more

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool stop_ = false;
    g2m::route::MapConfig config_;
    bool config_dirty_ = false;
    bool has_request_ = false;
    bool request_done_ = true;
    std::array<double, 3> request_{};
    bool has_frame_ = false;
    MapViewFrame frame_;
    std::atomic<std::size_t> builds_{0};
    std::thread worker_; // last
};

} // namespace rg
