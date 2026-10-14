// rg/route_planner.cpp — see route_planner.h.
#include "rg/route_planner.h"

#include "json_util.h"

#include "g2m/route/region_graph.h"
#include "g2m/route/route_guidance.h"
#include "g2m/route/route_map.h"
#include "g2m/route/route_search.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace rg {

namespace {

using detail::json;

std::int64_t to_mm(double session_m, double origin_m) { return std::llround((session_m + origin_m) * 1000.0); }

} // namespace

std::optional<RoutePlannerConfig> parse_route_planner_config(const std::string& json_text, const std::string& origin,
                                                             std::string* err) {
    const auto fail = [&](const std::string& m) -> std::optional<RoutePlannerConfig> {
        if (err != nullptr) *err = origin + ": " + m;
        return std::nullopt;
    };
    const std::optional<json> root = detail::parse_json(json_text);
    if (!root || !root->is_object()) return fail("not a JSON object");
    std::string format;
    if (!detail::get_string(*root, "format", format) || format != "rg.route_planner/1") {
        return fail("\"format\" must be \"rg.route_planner/1\"");
    }
    RoutePlannerConfig c;
    struct Field {
        const char* name;
        double* value;
        double lo, hi;
    };
    double landmarks = static_cast<double>(c.landmarks);
    double ahead = static_cast<double>(c.maneuvers_ahead);
    double cache_max = static_cast<double>(c.map_cache_max_tiles);
    const Field fields[] = {
        {"region_radius_m", &c.region_radius_m, 500.0, 200000.0},
        {"landmarks", &landmarks, 0.0, 64.0},
        {"snap_max_distance_m", &c.snap_max_distance_m, 1.0, 1000.0},
        {"heading_tolerance_deg", &c.heading_tolerance_deg, 1.0, 180.0},
        {"maneuvers_ahead", &ahead, 1.0, 20.0},
        {"rejoin_window_back_m", &c.rejoin_window_back_m, 0.0, 10000.0},
        {"rejoin_window_ahead_m", &c.rejoin_window_ahead_m, 0.0, 10000.0},
        {"off_route_m", &c.off_route_m, 1.0, 1000.0},
        {"minimap_radius_m", &c.minimap_radius_m, 50.0, 20000.0},
        {"map_cache_max_tiles", &cache_max, 16.0, 100000.0},
    };
    for (const Field& f : fields) {
        if (!detail::get_number(*root, f.name, *f.value)) return fail(std::string("missing number \"") + f.name + "\"");
        if (!(*f.value >= f.lo && *f.value <= f.hi)) {
            return fail(std::string("\"") + f.name + "\" out of range " + std::to_string(f.lo) + ".." + std::to_string(f.hi));
        }
    }
    c.landmarks = static_cast<std::size_t>(landmarks);
    c.maneuvers_ahead = static_cast<std::size_t>(ahead);
    c.map_cache_max_tiles = static_cast<std::size_t>(cache_max);
    return c;
}

std::optional<RoutePlannerConfig> load_route_planner_config(const std::string& path, std::string* err) {
    const std::optional<std::string> text = detail::read_text_file(path);
    if (!text) {
        if (err != nullptr) *err = path + ": cannot read";
        return std::nullopt;
    }
    return parse_route_planner_config(*text, path, err);
}

std::vector<g2m::TileKey> tiles_in_square(g2m::geo::UtmZone zone, int level, double e0, double n0, double cx,
                                          double cy, double half_m) {
    std::vector<g2m::TileKey> out;
    const auto lo = g2m::tile_key_at(zone, level, e0 + cx - half_m, n0 + cy - half_m);
    const auto hi = g2m::tile_key_at(zone, level, e0 + cx + half_m, n0 + cy + half_m);
    if (!lo || !hi) return out;
    for (std::int32_t y = lo->y; y <= hi->y; ++y) {
        for (std::int32_t x = lo->x; x <= hi->x; ++x) out.push_back(g2m::TileKey{zone, level, x, y});
    }
    std::sort(out.begin(), out.end());
    return out;
}

const char* to_string(PlannedRoute::Status status) {
    switch (status) {
        case PlannedRoute::Status::Ok: return "ok";
        case PlannedRoute::Status::Unreachable: return "unreachable";
        case PlannedRoute::Status::NoStart: return "no_start";
        case PlannedRoute::Status::NoDestination: return "no_destination";
        case PlannedRoute::Status::NoGraph: return "no_graph";
        case PlannedRoute::Status::Failed: return "failed";
    }
    return "failed";
}

RouteProgress route_progress(const PlannedRoute& route, double x, double y, double hint_along_m, std::size_t count,
                             const RoutePlannerConfig& config) {
    RouteProgress p;
    const auto& line = route.polyline;
    if (route.status != PlannedRoute::Status::Ok || line.empty() || route.polyline_s.size() != line.size()) return p;
    p.valid = true;
    const auto project = [&](double s_lo, double s_hi, double& best_d2, double& best_s) {
        for (std::size_t i = 0; i + 1 < line.size(); ++i) {
            if (route.polyline_s[i + 1] < s_lo || route.polyline_s[i] > s_hi) continue;
            const double ax = line[i][0], ay = line[i][1];
            const double vx = line[i + 1][0] - ax, vy = line[i + 1][1] - ay;
            const double len2 = vx * vx + vy * vy;
            double t = len2 > 0.0 ? ((x - ax) * vx + (y - ay) * vy) / len2 : 0.0;
            t = std::clamp(t, 0.0, 1.0);
            const double dx = ax + t * vx - x, dy = ay + t * vy - y;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best_s = route.polyline_s[i] + t * (route.polyline_s[i + 1] - route.polyline_s[i]);
            }
        }
        if (line.size() == 1) {
            const double dx = line[0][0] - x, dy = line[0][1] - y;
            best_d2 = dx * dx + dy * dy;
            best_s = 0.0;
        }
    };
    double d2 = std::numeric_limits<double>::infinity();
    double s = 0.0;
    project(hint_along_m - config.rejoin_window_back_m, hint_along_m + config.rejoin_window_ahead_m, d2, s);
    if (!(std::sqrt(d2) <= config.off_route_m)) {
        double d2_all = std::numeric_limits<double>::infinity();
        double s_all = 0.0;
        project(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), d2_all, s_all);
        if (d2_all < d2) {
            d2 = d2_all;
            s = s_all;
        }
    }
    p.along_m = s;
    p.off_route_m = std::sqrt(d2);
    p.off_route = p.off_route_m > config.off_route_m;
    p.remaining_m = std::max(0.0, route.length_m - s);
    for (const PlannedManeuver& m : route.maneuvers) {
        if (p.next.size() >= count) break;
        if (m.distance_from_start_m <= s && m.kind != "arrive") continue;
        p.next.push_back(UpcomingManeuver{m, std::max(0.0, m.distance_from_start_m - s)});
    }
    return p;
}

struct RoutePlanner::Impl {
    g2m::RoadGraph roads;
    g2m::route::RegionRouting routing;
    std::unique_ptr<g2m::route::RouteSearcher> searcher;
};

RoutePlanner::RoutePlanner(RoadGraphTileSource source, std::vector<g2m::TileKey> keys, double e0, double n0,
                           RoutePlannerConfig config)
    : source_(std::move(source)), keys_(std::move(keys)), e0_(e0), n0_(n0), config_(config),
      impl_(std::make_unique<Impl>()) {
    worker_ = std::thread([this]() { worker_main(); });
}

RoutePlanner::~RoutePlanner() {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        stop_ = true;
    }
    cancel_.store(true, std::memory_order_relaxed);
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

RoutePlanner::GraphState RoutePlanner::graph_state() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return state_;
}

std::string RoutePlanner::graph_message() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return message_;
}

RoutePlanner::GraphStats RoutePlanner::graph_stats() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return stats_;
}

std::uint64_t RoutePlanner::request(double sx, double sy, double hx, double hy, double dx, double dy) {
    std::lock_guard<std::mutex> lk(mutex_);
    request_ = {sx, sy, hx, hy, dx, dy};
    request_serial_ = next_serial_++;
    has_request_ = true;
    cv_.notify_all();
    return request_serial_;
}

bool RoutePlanner::poll(PlannedRoute& out) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!has_result_) return false;
    out = std::move(result_);
    result_ = PlannedRoute{};
    has_result_ = false;
    return true;
}

bool RoutePlanner::wait(std::uint64_t serial, PlannedRoute& out) {
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait(lk, [&]() { return stop_ || finished_serial_ >= serial; });
    if (!has_result_) return false;
    out = std::move(result_);
    result_ = PlannedRoute{};
    has_result_ = false;
    return true;
}

void RoutePlanner::worker_main() {
    const auto start = std::chrono::steady_clock::now();
    // 1. The region graph.
    std::string fail_message;
    g2m::route::RegionGraphStats region_stats;
    const g2m::route::RoadGraphTileFetch fetch =
        [this](const g2m::TileKey& key) -> g2m::Result<std::optional<g2m::RoadGraphTile>> {
        if (cancel_.load(std::memory_order_relaxed)) return g2m::make_error(g2m::ErrorCode::InvalidArgument, "cancelled");
        std::shared_ptr<const g2m::RoadGraphTile> tile = source_(key);
        if (tile == nullptr) return g2m::make_error(g2m::ErrorCode::NotFound, "roads.graph tile " + g2m::to_string(key) + " failed");
        if (tile->graph.ways.empty() && tile->graph.nodes.empty()) return std::optional<g2m::RoadGraphTile>{};
        g2m::RoadGraphTile copy = *tile;
        copy.key = key;
        return std::optional<g2m::RoadGraphTile>{std::move(copy)};
    };
    auto roads = g2m::route::assemble_region_graph(keys_, fetch, &region_stats);
    if (!roads) {
        fail_message = "region graph: " + roads.error().message;
    } else {
        impl_->roads = std::move(roads.value());
        auto routing = g2m::route::build_region_routing(impl_->roads, {}, g2m::route::default_route_config(),
                                                        config_.landmarks);
        if (!routing) {
            fail_message = "routing graph: " + routing.error().message;
        } else {
            impl_->routing = std::move(routing.value());
            impl_->searcher = std::make_unique<g2m::route::RouteSearcher>(*impl_->routing.graph,
                                                                          impl_->routing.landmarks.get());
        }
    }
    {
        std::lock_guard<std::mutex> lk(mutex_);
        stats_.tiles = region_stats.tiles_present;
        stats_.build_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        if (impl_->searcher) {
            state_ = GraphState::Ready;
            stats_.nodes = impl_->routing.graph->nodes().size();
            stats_.arcs = impl_->routing.graph->arcs().size();
            stats_.landmarks = impl_->routing.landmarks ? impl_->routing.landmarks->count() : 0;
        } else {
            state_ = GraphState::Failed;
            message_ = fail_message;
        }
    }
    cv_.notify_all();

    // 2. Requests (the newest only).
    for (;;) {
        std::array<double, 6> req{};
        std::uint64_t serial = 0;
        {
            std::unique_lock<std::mutex> lk(mutex_);
            cv_.wait(lk, [this]() { return stop_ || has_request_; });
            if (stop_) return;
            req = request_;
            serial = request_serial_;
            has_request_ = false;
        }
        PlannedRoute route;
        route.serial = serial;
        if (!impl_->searcher) {
            route.status = PlannedRoute::Status::NoGraph;
            route.message = fail_message;
        } else {
            const g2m::route::RoutingGraph& graph = *impl_->routing.graph;
            g2m::route::SnapOptions start_opts;
            start_opts.max_distance_mm = static_cast<std::uint32_t>(std::llround(config_.snap_max_distance_m * 1000.0));
            start_opts.heading_dx = req[2];
            start_opts.heading_dy = req[3];
            start_opts.heading_tolerance_deg = config_.heading_tolerance_deg;
            g2m::route::SnapOptions dest_opts;
            dest_opts.max_distance_mm = start_opts.max_distance_mm;
            const auto from = graph.snap(g2m::geom::PointMm{to_mm(req[0], e0_), to_mm(req[1], n0_)}, start_opts);
            const auto to = graph.snap(g2m::geom::PointMm{to_mm(req[4], e0_), to_mm(req[5], n0_)}, dest_opts);
            if (!from) {
                route.status = PlannedRoute::Status::NoStart;
                route.message = from.error().message;
            } else if (!to) {
                route.status = PlannedRoute::Status::NoDestination;
                route.message = to.error().message;
            } else if (auto found = impl_->searcher->find(from.value(), to.value()); !found) {
                route.status = PlannedRoute::Status::Failed;
                route.message = found.error().message;
            } else if (!found.value().reachable) {
                route.status = PlannedRoute::Status::Unreachable;
            } else {
                const g2m::route::Route& r = found.value();
                route.status = PlannedRoute::Status::Ok;
                route.length_m = static_cast<double>(r.length_mm) / 1000.0;
                route.duration_s = static_cast<double>(r.cost_ms) / 1000.0;
                double s = 0.0;
                for (const g2m::geom::PointMm& pt : g2m::route::route_polyline(graph, r)) {
                    const std::array<double, 2> q{static_cast<double>(pt.x) / 1000.0 - e0_,
                                                  static_cast<double>(pt.y) / 1000.0 - n0_};
                    if (!route.polyline.empty()) {
                        s += std::hypot(q[0] - route.polyline.back()[0], q[1] - route.polyline.back()[1]);
                    }
                    route.polyline.push_back(q);
                    route.polyline_s.push_back(s);
                }
                if (!r.arcs.empty()) {
                    auto guidance = g2m::route::build_guidance(graph, impl_->roads, r);
                    if (guidance) {
                        for (const g2m::route::Maneuver& m : guidance.value()) {
                            PlannedManeuver pm;
                            pm.kind = g2m::route::to_string(m.kind);
                            pm.distance_from_start_m = static_cast<double>(m.distance_from_start_mm) / 1000.0;
                            pm.x = static_cast<double>(m.position.x) / 1000.0 - e0_;
                            pm.y = static_cast<double>(m.position.y) / 1000.0 - n0_;
                            pm.angle_deg = m.angle_deg;
                            pm.exit_number = m.exit_number;
                            pm.name = m.name;
                            pm.ref = m.ref;
                            pm.destination = m.destination;
                            route.maneuvers.push_back(std::move(pm));
                        }
                    } else {
                        route.message = "guidance: " + guidance.error().message;
                    }
                }
            }
        }
        {
            std::lock_guard<std::mutex> lk(mutex_);
            result_ = std::move(route);
            has_result_ = true;
            finished_serial_ = serial;
        }
        cv_.notify_all();
    }
}

} // namespace rg
