// rg/building_footprints.h - building footprints as camera obstacles (PLAN.md R5 cinematic view, docs/buildings.md).
// Engine-neutral: no Godot type. The native building records (rg::Building, UTM easting/northing, absolute heights)
// become footprint prisms in SESSION coordinates; BuildingObstacles answers the questions the cinematic director asks
// (rg::ShotObstacles): is this camera point inside or beside a building, does this sight line pass through one.
#pragma once

#include "rg/buildings.h"
#include "rg/shot_obstacles.h"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

namespace rg {

// Footprint prisms (a polygon ring in session x/y, a vertical span in session z) indexed on a 32 m grid. One ring is
// one prism: a multipolygon building adds one per outer ring; inner courtyards are ignored (a camera in a courtyard is
// enclosed by the building anyway, so treating it as solid errs on the safe side). Not thread safe.
class BuildingFootprints {
public:
    void clear();
    // `ring` is closed implicitly; < 3 points or an empty span is ignored.
    void add(const std::vector<std::pair<double, double>>& ring, double z_min, double z_max);
    [[nodiscard]] std::size_t size() const { return prisms_.size(); }

    // True when (x, y) is inside a prism, or within margin_m of one's outline, and z lies in that prism's span.
    [[nodiscard]] bool near_point(double x, double y, double z, double margin_m) const;
    // True when the segment a -> b passes through a prism (inside its outline at a height within its span).
    [[nodiscard]] bool blocks_segment(double ax, double ay, double az, double bx, double by, double bz) const;

private:
    struct Prism {
        std::vector<double> xs, ys;
        double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
        double z_min = 0, z_max = 0;
    };
    static constexpr double kCellM = 32.0;
    static std::int64_t cell_key(int cx, int cy) {
        return (static_cast<std::int64_t>(cx) << 32) ^ static_cast<std::int64_t>(static_cast<std::uint32_t>(cy));
    }
    static bool inside(const Prism& p, double x, double y);
    static bool segment_hits(const Prism& p, double ax, double ay, double az, double bx, double by, double bz);
    std::vector<Prism> prisms_;
    std::map<std::int64_t, std::vector<std::uint32_t>> grid_;
};

// The ShotObstacles the game uses. Footprints are loaded per 1 km tile by a worker thread (the OSM tile decode is far
// too slow for a frame): request_around() queues the 3 x 3 tiles around a point; ready(x, y) is false until the tile
// under (x, y) has arrived, so the director frames from behind the car in the meantime instead of trusting missing
// data. Tiles far from the latest request are evicted. Thread safety: every method may be called from any thread.
class BuildingObstacles final : public ShotObstacles {
public:
    // Loads one tile's buildings (tile_x/tile_y = floor(UTM easting / tile_m), floor(northing / tile_m)). Runs on the
    // worker thread; may take long. A failed tile returns {} (treated as "no buildings").
    using TileFetch = std::function<std::vector<Building>(int tile_x, int tile_y)>;
    using GroundFn = std::function<std::optional<double>(double x, double y)>; // session x/y -> session z, main thread

    // e0/n0: the session frame's UTM origin (session x = easting - e0, y = northing - n0).
    BuildingObstacles(double e0, double n0, TileFetch fetch, GroundFn ground, double tile_m = 1024.0);
    ~BuildingObstacles() override;
    BuildingObstacles(const BuildingObstacles&) = delete;
    BuildingObstacles& operator=(const BuildingObstacles&) = delete;

    void request_around(double session_x, double session_y);
    // Synchronous insertion (what the worker calls; tests use it directly). Replaces a loaded tile.
    void add_tile(int tile_x, int tile_y, const std::vector<Building>& buildings);
    [[nodiscard]] std::size_t loaded_tile_count() const;
    [[nodiscard]] std::size_t footprint_count() const;
    // Blocks until the request queue is empty and the worker idle, or the timeout passes (tests).
    bool wait_idle(double timeout_s) const;

    [[nodiscard]] std::optional<double> ground_height(double x, double y) const override;
    [[nodiscard]] bool building_covers(double x, double y, double z, double margin_m) const override;
    [[nodiscard]] bool building_blocks(double ax, double ay, double az, double bx, double by, double bz) const override;
    [[nodiscard]] bool ready(double x, double y) const override;

private:
    [[nodiscard]] std::pair<int, int> tile_of(double session_x, double session_y) const;
    void worker_loop();
    void evict_locked();

    const double e0_, n0_, tile_m_;
    TileFetch fetch_;
    GroundFn ground_;

    mutable std::mutex mutex_;
    mutable std::condition_variable idle_cv_;
    std::condition_variable work_cv_;
    std::map<std::pair<int, int>, std::unique_ptr<BuildingFootprints>> tiles_;
    std::vector<std::pair<int, int>> queue_;
    std::pair<int, int> center_{0, 0};
    bool busy_ = false;
    bool stop_ = false;
    std::thread worker_;
};

} // namespace rg
