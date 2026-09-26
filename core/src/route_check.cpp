// rg/route_check.cpp — see route_check.h for the public contract.
//
// Exception-free like world_config.cpp (vault TOOL-019): nlohmann::json is
// parsed with allow_exceptions=false and every field is type-checked before
// any get<T>().
#include "rg/route_check.h"

#include "rg/world_terrain.h"

#include "g2m/core/raster.h"
#include "g2m/layer/terrain_layers.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>

namespace rg {

namespace {

using json = nlohmann::json;

bool fail(std::string* err, const std::string& path, const std::string& message) {
    if (err != nullptr) {
        *err = path + ": " + message;
    }
    return false;
}

bool get_number(const json& obj, const char* key, const std::string& path, const std::string& where, double* out,
                std::string* err) {
    if (!obj.contains(key)) {
        return fail(err, path, "missing required field \"" + std::string(key) + "\" in " + where);
    }
    const json& v = obj.at(key);
    if (!v.is_number()) {
        return fail(err, path, "\"" + std::string(key) + "\" in " + where + " must be a number");
    }
    *out = v.get<double>();
    if (!std::isfinite(*out)) {
        return fail(err, path, "\"" + std::string(key) + "\" in " + where + " must be finite");
    }
    return true;
}

bool get_string(const json& obj, const char* key, const std::string& path, const std::string& where,
                std::string* out, std::string* err) {
    if (!obj.contains(key)) {
        return fail(err, path, "missing required field \"" + std::string(key) + "\" in " + where);
    }
    const json& v = obj.at(key);
    if (!v.is_string()) {
        return fail(err, path, "\"" + std::string(key) + "\" in " + where + " must be a string");
    }
    *out = v.get<std::string>();
    return true;
}

bool require_object(const json& obj, const char* key, const std::string& path, std::string* err) {
    if (!obj.contains(key)) {
        return fail(err, path, "missing required field \"" + std::string(key) + "\"");
    }
    if (!obj.at(key).is_object()) {
        return fail(err, path, "\"" + std::string(key) + "\" must be an object");
    }
    return true;
}

double dist(const RoutePoint& a, const RoutePoint& b) { return std::hypot(b.x - a.x, b.y - a.y); }

std::string fmt(const char* f, double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), f, v);
    return buf;
}

// One arc-length sample of the polyline.
struct Sample {
    double s = 0.0;
    RoutePoint p;
};

// Samples every `step` metres of arc length (s = 0, step, 2*step, ...) plus
// the exact end point if it does not fall on the lattice.
void resample(const std::vector<RoutePoint>& pts, double step, double total, std::vector<Sample>& out) {
    out.clear();
    const auto n_steps = static_cast<std::int64_t>(std::floor(total / step + 1e-9));
    out.reserve(static_cast<std::size_t>(n_steps) + 2);
    std::size_t seg = 0;
    double seg_start = 0.0;
    double seg_len = dist(pts[0], pts[1]);
    for (std::int64_t k = 0; k <= n_steps; ++k) {
        const double s = static_cast<double>(k) * step;
        while (seg + 2 < pts.size() && s > seg_start + seg_len) {
            seg_start += seg_len;
            ++seg;
            seg_len = dist(pts[seg], pts[seg + 1]);
        }
        const double t = seg_len > 0.0 ? std::clamp((s - seg_start) / seg_len, 0.0, 1.0) : 0.0;
        const RoutePoint& a = pts[seg];
        const RoutePoint& b = pts[seg + 1];
        out.push_back(Sample{s, RoutePoint{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)}});
    }
    if (total - out.back().s > 1e-6) {
        out.push_back(Sample{total, pts.back()});
    }
}

// The polyline point at arc length s (clamped to [0, total]); `cum` holds
// the cumulative arc length at every waypoint (cum[0] = 0).
RoutePoint point_at(const std::vector<RoutePoint>& pts, const std::vector<double>& cum, double s) {
    const auto it = std::upper_bound(cum.begin(), cum.end(), s);
    const auto idx = static_cast<std::size_t>(it - cum.begin());
    const std::size_t seg = std::min(idx == 0 ? std::size_t{0} : idx - 1, pts.size() - 2);
    const double len = cum[seg + 1] - cum[seg];
    const double t = len > 0.0 ? std::clamp((s - cum[seg]) / len, 0.0, 1.0) : 0.0;
    const RoutePoint& a = pts[seg];
    const RoutePoint& b = pts[seg + 1];
    return RoutePoint{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}

// Radius of the circle through a, b, c (b between them along the route).
// Collinear: infinity going straight on, 0 for a full reversal at b.
double circumradius(const RoutePoint& a, const RoutePoint& b, const RoutePoint& c) {
    const double ab = dist(a, b);
    const double bc = dist(b, c);
    const double ca = dist(c, a);
    const double cross2 = std::abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)); // 2 * area
    if (cross2 == 0.0) {
        const double dot = (b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y);
        return dot < 0.0 ? 0.0 : std::numeric_limits<double>::infinity();
    }
    return ab * bc * ca / (2.0 * cross2);
}

// Optional "criteria" object of a route file; unknown keys are an error (a
// misspelt limit must not silently fall back to the default).
bool load_criteria(const json& root, const std::string& path, RouteCriteria* out, std::string* err) {
    if (!root.contains("criteria")) {
        return true;
    }
    const json& c = root.at("criteria");
    if (!c.is_object()) {
        return fail(err, path, "\"criteria\" must be an object");
    }
    const std::string where = "\"criteria\"";
    for (auto it = c.begin(); it != c.end(); ++it) {
        const std::string& key = it.key();
        if (key != "min_length_m" && key != "max_grade_pct" && key != "min_seam_crossings" &&
            key != "min_corner_radius_m") {
            return fail(err, path, "unknown key \"" + key + "\" in " + where);
        }
        double v = 0.0;
        if (!get_number(c, key.c_str(), path, where, &v, err)) {
            return false;
        }
        if (key == "min_length_m") {
            if (v < 0.0) {
                return fail(err, path, "\"criteria.min_length_m\" must be >= 0");
            }
            out->min_length_m = v;
        } else if (key == "max_grade_pct") {
            if (!(v > 0.0)) {
                return fail(err, path, "\"criteria.max_grade_pct\" must be > 0");
            }
            out->max_grade = v / 100.0;
        } else if (key == "min_seam_crossings") {
            if (v != std::floor(v) || v < 0.0 || v > 1.0e6) {
                return fail(err, path, "\"criteria.min_seam_crossings\" must be an integer >= 0");
            }
            out->min_seam_crossings = static_cast<int>(v);
        } else {
            if (v < 0.0) {
                return fail(err, path, "\"criteria.min_corner_radius_m\" must be >= 0");
            }
            out->min_corner_radius_m = v;
        }
    }
    return true;
}

} // namespace

std::optional<Route> load_route(const std::string& path, std::string* err) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        fail(err, path, "cannot open file");
        return std::nullopt;
    }
    std::stringstream buf;
    buf << in.rdbuf();
    const json root = json::parse(buf.str(), /*cb=*/nullptr, /*allow_exceptions=*/false);
    if (root.is_discarded()) {
        fail(err, path, "malformed JSON");
        return std::nullopt;
    }
    if (!root.is_object()) {
        fail(err, path, "top level must be an object");
        return std::nullopt;
    }

    Route route;
    if (!get_string(root, "format", path, "top level", &route.format, err)) {
        return std::nullopt;
    }
    if (route.format != "rg.route/1") {
        fail(err, path, "unsupported \"format\" (expected \"rg.route/1\", got \"" + route.format + "\")");
        return std::nullopt;
    }
    if (!get_string(root, "name", path, "top level", &route.name, err)) {
        return std::nullopt;
    }

    if (!require_object(root, "session_origin_utm", path, err)) {
        return std::nullopt;
    }
    const json& origin = root.at("session_origin_utm");
    double zone = 0.0;
    if (!get_number(origin, "zone", path, "\"session_origin_utm\"", &zone, err) ||
        !get_number(origin, "e0", path, "\"session_origin_utm\"", &route.e0, err) ||
        !get_number(origin, "n0", path, "\"session_origin_utm\"", &route.n0, err)) {
        return std::nullopt;
    }
    if (zone != std::floor(zone) || zone < 1.0 || zone > 60.0) {
        fail(err, path, "\"session_origin_utm.zone\" must be an integer in 1..60");
        return std::nullopt;
    }
    route.zone = static_cast<int>(zone);

    if (!require_object(root, "spawn", path, err)) {
        return std::nullopt;
    }
    const json& spawn = root.at("spawn");
    if (!get_number(spawn, "x", path, "\"spawn\"", &route.spawn.x, err) ||
        !get_number(spawn, "y", path, "\"spawn\"", &route.spawn.y, err) ||
        !get_number(spawn, "yaw_deg", path, "\"spawn\"", &route.spawn_yaw_deg, err)) {
        return std::nullopt;
    }

    if (!root.contains("waypoints") || !root.at("waypoints").is_array()) {
        fail(err, path, "\"waypoints\" must be an array of [x, y] pairs");
        return std::nullopt;
    }
    const json& wps = root.at("waypoints");
    for (std::size_t i = 0; i < wps.size(); ++i) {
        const json& wp = wps.at(i);
        if (!wp.is_array() || wp.size() != 2 || !wp.at(0).is_number() || !wp.at(1).is_number()) {
            fail(err, path, "waypoint " + std::to_string(i) + " must be an [x, y] pair of numbers");
            return std::nullopt;
        }
        const RoutePoint p{wp.at(0).get<double>(), wp.at(1).get<double>()};
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            fail(err, path, "waypoint " + std::to_string(i) + " is not finite");
            return std::nullopt;
        }
        route.waypoints.push_back(p);
    }
    if (route.waypoints.size() < 2) {
        fail(err, path, "\"waypoints\" needs at least 2 points");
        return std::nullopt;
    }
    if (!load_criteria(root, path, &route.criteria, err)) {
        return std::nullopt;
    }
    return route;
}

void apply_route_criteria(const RouteCriteria& criteria, RouteCheckParams& params) {
    if (criteria.min_length_m.has_value()) {
        params.min_length_m = *criteria.min_length_m;
    }
    if (criteria.max_grade.has_value()) {
        params.max_grade = *criteria.max_grade;
    }
    if (criteria.min_seam_crossings.has_value()) {
        params.min_seam_crossings = *criteria.min_seam_crossings;
    }
    if (criteria.min_corner_radius_m.has_value()) {
        params.min_corner_radius_m = *criteria.min_corner_radius_m;
    }
}

std::int64_t phys_tile_index(double v, double tile_size_m, double origin_m) {
    return static_cast<std::int64_t>(std::floor((v - origin_m) / tile_size_m));
}

int count_seam_crossings(const std::vector<RoutePoint>& polyline, double tile_size_m, double origin_m) {
    std::int64_t total = 0;
    for (std::size_t i = 1; i < polyline.size(); ++i) {
        const std::int64_t dx = phys_tile_index(polyline[i].x, tile_size_m, origin_m) -
                                phys_tile_index(polyline[i - 1].x, tile_size_m, origin_m);
        const std::int64_t dy = phys_tile_index(polyline[i].y, tile_size_m, origin_m) -
                                phys_tile_index(polyline[i - 1].y, tile_size_m, origin_m);
        total += (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);
    }
    return static_cast<int>(total);
}

std::optional<double> sample_l0_height(const L0TileLookupFn& lookup, g2m::geo::UtmZone zone, std::int64_t e0,
                                       std::int64_t n0, double x, double y) {
    // Global cell-centre lattice: column c's centre is at easting c + 0.5.
    const double u = static_cast<double>(e0) + x - 0.5;
    const double v = static_cast<double>(n0) + y - 0.5;
    if (!std::isfinite(u) || !std::isfinite(v)) {
        return std::nullopt;
    }
    const double fu = std::floor(u);
    const double fv = std::floor(v);
    const auto c0 = static_cast<std::int64_t>(fu);
    const auto r0 = static_cast<std::int64_t>(fv);
    const double tx = u - fu;
    const double ty = v - fv;

    constexpr std::int64_t kN = g2m::kTerrainTileSamples;
    const g2m::HeightTile* cached = nullptr; // the 4 corners usually share one tile
    double h[2][2] = {};
    for (int dj = 0; dj < 2; ++dj) {
        for (int di = 0; di < 2; ++di) {
            const std::int64_t c = c0 + di;
            const std::int64_t r = r0 + dj;
            g2m::TileKey key;
            key.zone = zone;
            key.level = 0;
            key.x = static_cast<std::int32_t>(g2m::floor_div(c, kN));
            key.y = static_cast<std::int32_t>(g2m::floor_div(r, kN));
            if (!key.valid()) {
                return std::nullopt;
            }
            const g2m::HeightTile* tile = (cached != nullptr && cached->key == key) ? cached : lookup(key);
            if (tile == nullptr) {
                return std::nullopt;
            }
            cached = tile;
            const std::int64_t i = c - static_cast<std::int64_t>(key.x) * kN;
            const std::int64_t j = r - static_cast<std::int64_t>(key.y) * kN;
            const std::int32_t raw = tile->h[static_cast<std::size_t>(j * kN + i)];
            if (raw == g2m::kHeightNoData) {
                return std::nullopt;
            }
            h[dj][di] = static_cast<double>(raw) / 256.0;
        }
    }
    const double south = h[0][0] + tx * (h[0][1] - h[0][0]);
    const double north = h[1][0] + tx * (h[1][1] - h[1][0]);
    return south + ty * (north - south);
}

RouteCheckReport check_route(const std::vector<RoutePoint>& waypoints, const RoutePoint& expected_start,
                             const HeightAtFn& height_at, const RouteCheckParams& params) {
    RouteCheckReport rep;
    rep.waypoint_count = static_cast<int>(waypoints.size());
    if (waypoints.size() < 2) {
        rep.failures.push_back("route has fewer than 2 waypoints");
        return rep;
    }
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        rep.length_m += dist(waypoints[i - 1], waypoints[i]);
    }
    if (!(rep.length_m > 0.0) || !(params.sample_step_m > 0.0)) {
        rep.failures.push_back("route has zero length (or sample_step_m <= 0)");
        return rep;
    }

    rep.start_offset_m = dist(waypoints.front(), expected_start);
    rep.start_heading_deg =
        std::atan2(waypoints[1].y - waypoints[0].y, waypoints[1].x - waypoints[0].x) * 180.0 / 3.14159265358979323846;

    // --- heights along the polyline ---
    std::vector<Sample> samples;
    resample(waypoints, params.sample_step_m, rep.length_m, samples);
    rep.sample_count = static_cast<int>(samples.size());
    std::vector<std::optional<double>> heights(samples.size());
    rep.min_elevation_m = std::numeric_limits<double>::infinity();
    rep.max_elevation_m = -std::numeric_limits<double>::infinity();
    for (std::size_t k = 0; k < samples.size(); ++k) {
        heights[k] = height_at(samples[k].p.x, samples[k].p.y);
        if (!heights[k].has_value()) {
            if (rep.nodata_samples == 0) {
                rep.first_nodata_at_m = samples[k].s;
            }
            ++rep.nodata_samples;
            continue;
        }
        rep.min_elevation_m = std::min(rep.min_elevation_m, *heights[k]);
        rep.max_elevation_m = std::max(rep.max_elevation_m, *heights[k]);
    }
    if (rep.nodata_samples == rep.sample_count) {
        rep.min_elevation_m = 0.0;
        rep.max_elevation_m = 0.0;
    }

    // --- grade over the window (on the regular lattice only; the extra end
    // sample is not part of any window) ---
    const auto window_steps =
        std::max<std::int64_t>(1, static_cast<std::int64_t>(std::llround(params.grade_window_m / params.sample_step_m)));
    const double window_len = static_cast<double>(window_steps) * params.sample_step_m;
    std::vector<double> grades;
    double sum_abs = 0.0;
    rep.grade_report_threshold = params.grade_report_threshold;
    bool in_steep = false;
    RouteSteepStretch steep;
    for (std::size_t k = 0; k + static_cast<std::size_t>(window_steps) < samples.size(); ++k) {
        const std::size_t k2 = k + static_cast<std::size_t>(window_steps);
        if (samples[k2].s - samples[k].s < window_len - 1e-6) {
            break; // k2 is the off-lattice end sample
        }
        if (!heights[k].has_value() || !heights[k2].has_value()) {
            continue;
        }
        const double g = std::abs(*heights[k2] - *heights[k]) / window_len;
        if (g > rep.max_grade) {
            rep.max_grade = g;
            rep.max_grade_at_m = samples[k].s;
        }
        sum_abs += g;
        grades.push_back(g);
        if (g > params.grade_report_threshold) {
            if (!in_steep) {
                in_steep = true;
                steep = RouteSteepStretch{g, samples[k].s, samples[k].p.x, samples[k].p.y, samples[k].s, 0.0};
            } else if (g > steep.max_grade) {
                steep.max_grade = g;
                steep.at_m = samples[k].s;
                steep.x = samples[k].p.x;
                steep.y = samples[k].p.y;
            }
            steep.end_m = samples[k2].s;
        } else if (in_steep) {
            rep.steep_stretches.push_back(steep);
            in_steep = false;
        }
    }
    if (in_steep) {
        rep.steep_stretches.push_back(steep);
    }
    rep.grade_window_count = static_cast<int>(grades.size());
    if (!grades.empty()) {
        rep.mean_abs_grade = sum_abs / static_cast<double>(grades.size());
        std::sort(grades.begin(), grades.end());
        const auto rank = static_cast<std::size_t>(std::ceil(0.99 * static_cast<double>(grades.size())));
        rep.p99_grade = grades[std::max<std::size_t>(rank, 1) - 1];
    }

    // --- seams ---
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        const std::int64_t dx = phys_tile_index(waypoints[i].x, params.phys_tile_size_m, params.phys_origin_m) -
                                phys_tile_index(waypoints[i - 1].x, params.phys_tile_size_m, params.phys_origin_m);
        const std::int64_t dy = phys_tile_index(waypoints[i].y, params.phys_tile_size_m, params.phys_origin_m) -
                                phys_tile_index(waypoints[i - 1].y, params.phys_tile_size_m, params.phys_origin_m);
        rep.seam_crossings_x += static_cast<int>(dx < 0 ? -dx : dx);
        rep.seam_crossings_y += static_cast<int>(dy < 0 ? -dy : dy);
    }
    rep.seam_crossings = rep.seam_crossings_x + rep.seam_crossings_y;

    // --- corners: circle through s - w, s, s + w at every sample with a
    // full window; consecutive samples below the report radius form one
    // listed stretch ---
    rep.min_corner_radius_m = std::numeric_limits<double>::infinity();
    rep.corner_report_radius_m = params.corner_report_radius_m;
    {
        std::vector<double> cum(waypoints.size(), 0.0);
        for (std::size_t i = 1; i < waypoints.size(); ++i) {
            cum[i] = cum[i - 1] + dist(waypoints[i - 1], waypoints[i]);
        }
        const double w = params.corner_window_m;
        bool in_run = false;
        RouteCorner run;
        for (const Sample& smp : samples) {
            if (!(w > 0.0) || smp.s - w < -1e-9 || smp.s + w > rep.length_m + 1e-9) {
                continue;
            }
            const double radius =
                circumradius(point_at(waypoints, cum, smp.s - w), smp.p, point_at(waypoints, cum, smp.s + w));
            if (radius < rep.min_corner_radius_m) {
                rep.min_corner_radius_m = radius;
                rep.min_corner_at_m = smp.s;
                rep.min_corner_x = smp.p.x;
                rep.min_corner_y = smp.p.y;
            }
            if (radius < params.corner_report_radius_m) {
                if (!in_run) {
                    in_run = true;
                    run = RouteCorner{radius, smp.s, smp.p.x, smp.p.y, smp.s, smp.s};
                } else if (radius < run.radius_m) {
                    run.radius_m = radius;
                    run.at_m = smp.s;
                    run.x = smp.p.x;
                    run.y = smp.p.y;
                }
                run.end_m = smp.s;
            } else if (in_run) {
                rep.tight_corners.push_back(run);
                in_run = false;
            }
        }
        if (in_run) {
            rep.tight_corners.push_back(run);
        }
    }

    // --- criteria ---
    if (rep.start_offset_m > params.start_tolerance_m) {
        rep.failures.push_back("start: first waypoint is " + fmt("%.3f", rep.start_offset_m) +
                               " m from the spawn (tolerance " + fmt("%.3f", params.start_tolerance_m) + " m)");
    }
    if (rep.length_m < params.min_length_m) {
        rep.failures.push_back("length: " + fmt("%.1f", rep.length_m) + " m < " + fmt("%.1f", params.min_length_m) +
                               " m");
    }
    if (rep.nodata_samples > 0) {
        rep.failures.push_back("nodata: " + std::to_string(rep.nodata_samples) + " samples, first at s=" +
                               fmt("%.1f", rep.first_nodata_at_m) + " m");
    }
    if (rep.grade_window_count == 0) {
        rep.failures.push_back("grade: no measurable grade window");
    } else if (rep.max_grade > params.max_grade) {
        rep.failures.push_back("grade: max " + fmt("%.2f", rep.max_grade * 100.0) + " % at s=" +
                               fmt("%.1f", rep.max_grade_at_m) + " m > " + fmt("%.2f", params.max_grade * 100.0) +
                               " %");
    }
    if (rep.seam_crossings < params.min_seam_crossings) {
        rep.failures.push_back("seams: " + std::to_string(rep.seam_crossings) + " crossings < " +
                               std::to_string(params.min_seam_crossings));
    }
    if (rep.min_corner_radius_m < params.min_corner_radius_m) {
        rep.failures.push_back("corner: radius " + fmt("%.1f", rep.min_corner_radius_m) + " m at s=" +
                               fmt("%.1f", rep.min_corner_at_m) + " m (" + fmt("%.1f", rep.min_corner_x) + ", " +
                               fmt("%.1f", rep.min_corner_y) + ") < " + fmt("%.1f", params.min_corner_radius_m) +
                               " m");
    }
    return rep;
}

std::string format_route_report(const RouteCheckReport& r) {
    std::ostringstream o;
    char buf[256];
    std::snprintf(buf, sizeof(buf), "waypoints=%d length_m=%.1f samples=%d\n", r.waypoint_count, r.length_m,
                  r.sample_count);
    o << buf;
    std::snprintf(buf, sizeof(buf), "start_offset_m=%.3f start_heading_deg=%.1f\n", r.start_offset_m,
                  r.start_heading_deg);
    o << buf;
    std::snprintf(buf, sizeof(buf),
                  "grade_windows=%d max_grade_pct=%.2f (at s=%.1f m) p99_grade_pct=%.2f mean_grade_pct=%.2f\n",
                  r.grade_window_count, r.max_grade * 100.0, r.max_grade_at_m, r.p99_grade * 100.0,
                  r.mean_abs_grade * 100.0);
    o << buf;
    std::snprintf(buf, sizeof(buf), "steep_stretches=%zu (grade > %.2f %%)\n", r.steep_stretches.size(),
                  r.grade_report_threshold * 100.0);
    o << buf;
    for (const RouteSteepStretch& st : r.steep_stretches) {
        std::snprintf(buf, sizeof(buf),
                      "  steep max_grade_pct=%.2f at s=%.1f m (x=%.1f y=%.1f), s=%.1f..%.1f m\n",
                      st.max_grade * 100.0, st.at_m, st.x, st.y, st.begin_m, st.end_m);
        o << buf;
    }
    std::snprintf(buf, sizeof(buf), "seam_crossings=%d (x=%d y=%d)\n", r.seam_crossings, r.seam_crossings_x,
                  r.seam_crossings_y);
    o << buf;
    std::snprintf(buf, sizeof(buf), "elevation_m min=%.2f max=%.2f\n", r.min_elevation_m, r.max_elevation_m);
    o << buf;
    std::snprintf(buf, sizeof(buf), "nodata_samples=%d first_nodata_at_m=%.1f\n", r.nodata_samples,
                  r.first_nodata_at_m);
    o << buf;
    std::snprintf(buf, sizeof(buf), "min_corner_radius_m=%.1f (at s=%.1f m, x=%.1f y=%.1f)\n",
                  r.min_corner_radius_m, r.min_corner_at_m, r.min_corner_x, r.min_corner_y);
    o << buf;
    std::snprintf(buf, sizeof(buf), "tight_corners=%zu (radius < %.1f m)\n", r.tight_corners.size(),
                  r.corner_report_radius_m);
    o << buf;
    for (const RouteCorner& c : r.tight_corners) {
        std::snprintf(buf, sizeof(buf), "  corner radius_m=%.1f at s=%.1f m (x=%.1f y=%.1f), s=%.1f..%.1f m\n",
                      c.radius_m, c.at_m, c.x, c.y, c.begin_m, c.end_m);
        o << buf;
    }
    for (const std::string& f : r.failures) {
        o << "FAIL " << f << "\n";
    }
    o << (r.ok() ? "route_check: PASS\n" : "route_check: FAIL\n");
    return o.str();
}

std::string route_matches_world(const Route& route, int zone, double e0, double n0, double spawn_e, double spawn_n,
                                double spawn_yaw_deg, double tolerance_m) {
    std::ostringstream o;
    if (route.zone != zone || route.e0 != e0 || route.n0 != n0) {
        o << "route session_origin_utm (" << route.zone << ", " << route.e0 << ", " << route.n0
          << ") != world config (" << zone << ", " << e0 << ", " << n0 << "); ";
    }
    const double sx = spawn_e - e0;
    const double sy = spawn_n - n0;
    if (std::hypot(route.spawn.x - sx, route.spawn.y - sy) > tolerance_m) {
        o << "route spawn (" << route.spawn.x << ", " << route.spawn.y << ") != world config spawn (" << sx << ", "
          << sy << "); ";
    }
    if (route.spawn_yaw_deg != spawn_yaw_deg) {
        o << "route spawn yaw " << route.spawn_yaw_deg << " != world config " << spawn_yaw_deg << "; ";
    }
    return o.str();
}

RouteCheckReport check_route_on_world(const Route& route, const WorldConfig& world, WorldTerrain& terrain,
                                      const RouteCheckParams& params) {
    const g2m::geo::UtmZone zone{world.session_origin_utm.zone};
    const auto e0 = static_cast<std::int64_t>(std::llround(world.session_origin_utm.e0));
    const auto n0 = static_cast<std::int64_t>(std::llround(world.session_origin_utm.n0));
    // height_tile_shared() hands out shared_ptrs; the lookup keeps the most
    // recent one alive for the duration of the sample_l0_height call.
    std::shared_ptr<const g2m::HeightTile> hold;
    const L0TileLookupFn lookup = [&terrain, &hold](const g2m::TileKey& key) -> const g2m::HeightTile* {
        HeightTileFetchResult r = terrain.height_tile_shared(key);
        if (r.status != g2m::Status::Ok || r.tile == nullptr) {
            return nullptr;
        }
        hold = std::move(r.tile);
        return hold.get();
    };
    const HeightAtFn height_at = [&](double x, double y) { return sample_l0_height(lookup, zone, e0, n0, x, y); };

    RouteCheckReport rep = check_route(route.waypoints, route.spawn, height_at, params);
    const std::string mismatch =
        route_matches_world(route, world.session_origin_utm.zone, world.session_origin_utm.e0,
                            world.session_origin_utm.n0, world.spawn.e, world.spawn.n, world.spawn.yaw_deg);
    if (!mismatch.empty()) {
        rep.failures.insert(rep.failures.begin(), "world: " + mismatch);
    }
    return rep;
}

} // namespace rg
