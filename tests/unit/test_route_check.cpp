// test_route_check.cpp — rg::route_check coverage (R2.2 plan R5 route
// criteria): physics-tile seam counting (incl. negative indices and corner
// crossings), the 10 m grade window (max, nearest-rank p99, the steep-stretch
// list), NoData detection, the +-10 m corner radius and its tight-corner
// list, the start/length criteria, sample_l0_height's bilinear read across
// L0 tile borders, load_route (incl. the optional "criteria" object) and its
// error paths, apply_route_criteria and route_matches_world. Everything runs on synthetic terrain - CI never reads
// cache/. The one real-data case is hidden ([.][realdata]) and SKIPs unless
// RG_G2M_HOME is set.
//
// HeightTile is 256 KiB (vault TOOL-039): the synthetic tiles below are
// always heap-allocated (std::make_unique), never locals or by-value copies.
#include "rg/route_check.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include "g2m/core/raster.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace {

using Catch::Approx;
using rg::RoutePoint;

bool has_failure(const rg::RouteCheckReport& r, const std::string& prefix) {
    for (const std::string& f : r.failures) {
        if (f.rfind(prefix, 0) == 0) {
            return true;
        }
    }
    return false;
}

// A straight 3.5 km route east from (0, 0): 14 seam crossings (x index -1
// at x = 0 up to 13 at x = 3500), no turns.
std::vector<RoutePoint> straight_east(double length_m) { return {{0.0, 0.0}, {length_m, 0.0}}; }

// Loose params so a test can exercise one criterion at a time.
rg::RouteCheckParams loose() {
    rg::RouteCheckParams p;
    p.min_length_m = 0.0;
    p.min_seam_crossings = 0;
    p.max_grade = 1.0;
    p.min_corner_radius_m = 0.0;
    return p;
}

// Portable getenv (std::getenv is deprecated by the Windows UCRT headers and
// rg_warnings is /WX - same pattern as test_world_config.cpp).
std::optional<std::string> safe_getenv(const char* name) {
#ifdef _WIN32
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) {
        return std::nullopt;
    }
    std::string value(buf);
    std::free(buf);
    return value;
#else
    const char* value = std::getenv(name);
    return value != nullptr ? std::optional<std::string>(value) : std::nullopt;
#endif
}

class TempFile {
public:
    explicit TempFile(const std::string& contents) {
        // ctest runs test cases as parallel processes: a per-process random
        // tag plus a per-process counter keeps the names apart.
        static const unsigned tag = std::random_device{}();
        static int counter = 0;
        path_ = std::filesystem::temp_directory_path() /
                ("rg_test_route_" + std::to_string(tag) + "_" + std::to_string(counter++) + ".json");
        std::ofstream out(path_, std::ios::binary);
        out << contents;
    }
    ~TempFile() {
        std::error_code ec;
        std::filesystem::remove(path_, ec);
    }
    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;
    std::string path() const { return path_.string(); }

private:
    std::filesystem::path path_;
};

const char* kValidRoute = R"({
  "format": "rg.route/1",
  "name": "t",
  "source": "unknown keys are ignored",
  "session_origin_utm": {"zone": 32, "e0": 464000, "n0": 5559000},
  "spawn": {"x": -876.0, "y": 208.0, "yaw_deg": 45},
  "waypoints": [[-876.0, 208.0], [-800.5, 250.25], [0, 0]]
})";

std::string replace(std::string s, const std::string& from, const std::string& to) {
    const std::size_t at = s.find(from);
    REQUIRE(at != std::string::npos);
    s.replace(at, from.size(), to);
    return s;
}

} // namespace

TEST_CASE("route_check: phys_tile_index on the 255 m grid with origin 0.5, incl. negative indices", "[route_check]") {
    CHECK(rg::phys_tile_index(0.5) == 0);
    CHECK(rg::phys_tile_index(255.49) == 0);
    CHECK(rg::phys_tile_index(255.5) == 1);
    CHECK(rg::phys_tile_index(0.49) == -1);
    CHECK(rg::phys_tile_index(0.0) == -1);
    CHECK(rg::phys_tile_index(-254.5) == -1);
    CHECK(rg::phys_tile_index(-254.51) == -2);
    CHECK(rg::phys_tile_index(-1500.0) == -6); // floor(-1500.5 / 255) = floor(-5.88)
}

TEST_CASE("route_check: count_seam_crossings counts seam lines, both signs, corners count twice", "[route_check]") {
    // East 0 -> 1000: index -1 -> 3.
    CHECK(rg::count_seam_crossings({{0.0, 0.0}, {1000.0, 0.0}}) == 4);
    // West, entirely at negative indices: 100 (0) -> -600 (-3).
    CHECK(rg::count_seam_crossings({{100.0, 10.0}, {-600.0, 10.0}}) == 3);
    // Out and back across the same seam: 2.
    CHECK(rg::count_seam_crossings({{250.0, 10.0}, {260.0, 10.0}, {250.0, 10.0}}) == 2);
    // Diagonal through the (255.5, 255.5) corner: one x seam + one y seam.
    CHECK(rg::count_seam_crossings({{250.0, 250.0}, {260.0, 260.0}}) == 2);
    // Across the negative-side seam at -254.5 in y only.
    CHECK(rg::count_seam_crossings({{10.0, -250.0}, {10.0, -260.0}}) == 1);
    // Staying inside one tile: 0; a single point: 0.
    CHECK(rg::count_seam_crossings({{1.0, 1.0}, {200.0, 200.0}}) == 0);
    CHECK(rg::count_seam_crossings({{1.0, 1.0}}) == 0);
    // Non-default grid.
    CHECK(rg::count_seam_crossings({{0.0, 0.0}, {100.0, 0.0}}, 10.0, 0.0) == 10);
}

TEST_CASE("route_check: check_route passes a gentle 3.5 km straight route", "[route_check]") {
    const auto ramp = [](double x, double) -> std::optional<double> { return 100.0 + 0.05 * x; };
    const rg::RouteCheckReport r = rg::check_route(straight_east(3500.0), {0.0, 0.0}, ramp);
    INFO(rg::format_route_report(r));
    CHECK(r.ok());
    CHECK(r.length_m == Approx(3500.0));
    CHECK(r.sample_count == 3501);
    CHECK(r.grade_window_count == 3491);
    CHECK(r.max_grade == Approx(0.05));
    CHECK(r.p99_grade == Approx(0.05));
    CHECK(r.mean_abs_grade == Approx(0.05));
    CHECK(r.seam_crossings == 14);
    CHECK(r.seam_crossings_x == 14);
    CHECK(r.seam_crossings_y == 0);
    CHECK(r.min_elevation_m == Approx(100.0));
    CHECK(r.max_elevation_m == Approx(275.0));
    CHECK(r.nodata_samples == 0);
    CHECK(r.first_nodata_at_m == -1.0);
    CHECK(std::isinf(r.min_corner_radius_m));
    CHECK(r.start_offset_m == 0.0);
    CHECK(r.start_heading_deg == Approx(0.0));
    CHECK(rg::format_route_report(r).find("route_check: PASS") != std::string::npos);
}

TEST_CASE("route_check: the 10 m grade window - max, its position and the nearest-rank p99", "[route_check]") {
    // Flat, with one 2 m rise over x in [500, 510]: the window starting at
    // s = 500 sees all of it (0.2); the 9 windows either side see i/10 of it.
    const auto bump = [](double x, double) -> std::optional<double> {
        return 0.2 * std::min(std::max(x - 500.0, 0.0), 10.0);
    };
    const rg::RouteCheckReport r = rg::check_route(straight_east(1000.0), {0.0, 0.0}, bump, loose());
    INFO(rg::format_route_report(r));
    CHECK(r.grade_window_count == 991);
    CHECK(r.max_grade == Approx(0.2));
    CHECK(r.max_grade_at_m == Approx(500.0));
    // 972 zero windows + 19 non-zero {0.02, 0.02, 0.04, 0.04, ..., 0.18, 0.18, 0.2};
    // rank ceil(0.99 * 991) = 982 -> the 10th non-zero value ascending = 0.10.
    CHECK(r.p99_grade == Approx(0.10));
    CHECK(r.ok()); // loose params
    // Steep-stretch listing (default threshold 12 %). At 13 % (clear of the
    // 0.12 windows' rounding) the windows starting at s = 497 .. 503
    // (0.14 .. 0.2 .. 0.14) form one stretch ending at 503 + 10.
    CHECK(r.grade_report_threshold == Approx(0.12));
    CHECK(r.steep_stretches.size() == 1);
    rg::RouteCheckParams listing = loose();
    listing.grade_report_threshold = 0.13;
    const rg::RouteCheckReport l = rg::check_route(straight_east(1000.0), {0.0, 0.0}, bump, listing);
    REQUIRE(l.steep_stretches.size() == 1);
    CHECK(l.steep_stretches[0].max_grade == Approx(0.2));
    CHECK(l.steep_stretches[0].at_m == Approx(500.0));
    CHECK(l.steep_stretches[0].x == Approx(500.0));
    CHECK(l.steep_stretches[0].begin_m == Approx(497.0));
    CHECK(l.steep_stretches[0].end_m == Approx(513.0));
    CHECK(l.ok()); // listing only, never a failure

    rg::RouteCheckParams strict = loose();
    strict.max_grade = 0.12;
    const rg::RouteCheckReport s = rg::check_route(straight_east(1000.0), {0.0, 0.0}, bump, strict);
    CHECK_FALSE(s.ok());
    CHECK(has_failure(s, "grade:"));

    // Heading along the contour of a steep side slope: no grade at all.
    const auto side = [](double, double y) -> std::optional<double> { return 0.3 * y; };
    const rg::RouteCheckReport c = rg::check_route(straight_east(500.0), {0.0, 0.0}, side, loose());
    CHECK(c.max_grade == Approx(0.0).margin(1e-12));

    // A 20 m window over a 1 m step: grade 0.05.
    rg::RouteCheckParams w20 = loose();
    w20.grade_window_m = 20.0;
    const auto step = [](double x, double) -> std::optional<double> { return x < 300.0 ? 0.0 : 1.0; };
    const rg::RouteCheckReport ww = rg::check_route(straight_east(600.0), {0.0, 0.0}, step, w20);
    CHECK(ww.max_grade == Approx(0.05));
}

TEST_CASE("route_check: NoData along the route is counted, located and fails the check", "[route_check]") {
    const auto holey = [](double x, double) -> std::optional<double> {
        if (x >= 300.0 && x <= 305.0) {
            return std::nullopt;
        }
        return 10.0;
    };
    const rg::RouteCheckReport r = rg::check_route(straight_east(1000.0), {0.0, 0.0}, holey, loose());
    INFO(rg::format_route_report(r));
    CHECK(r.nodata_samples == 6); // s = 300 .. 305
    CHECK(r.first_nodata_at_m == Approx(300.0));
    CHECK(has_failure(r, "nodata:"));
    // Windows touching a NoData sample are skipped, the rest still measured.
    CHECK(r.grade_window_count == 991 - 12); // k in 290..295 and 300..305
    CHECK(r.max_grade == 0.0);

    // All NoData: no grade window at all -> its own failure, elevation 0/0.
    const auto none = [](double, double) -> std::optional<double> { return std::nullopt; };
    const rg::RouteCheckReport n = rg::check_route(straight_east(100.0), {0.0, 0.0}, none, loose());
    CHECK(n.nodata_samples == n.sample_count);
    CHECK(has_failure(n, "grade:"));
    CHECK(n.min_elevation_m == 0.0);
    CHECK(n.max_elevation_m == 0.0);
}

TEST_CASE("route_check: start, length, seam and corner criteria", "[route_check]") {
    const auto flat = [](double, double) -> std::optional<double> { return 0.0; };
    const rg::RouteCheckReport ok = rg::check_route(straight_east(3500.0), {0.0, 0.0}, flat);
    CHECK(ok.ok());

    // Start: the first waypoint 1 m off the expected spawn.
    const rg::RouteCheckReport off = rg::check_route(straight_east(3500.0), {0.0, 1.0}, flat);
    CHECK(off.start_offset_m == Approx(1.0));
    CHECK(has_failure(off, "start:"));

    // Length.
    const rg::RouteCheckReport short_r = rg::check_route(straight_east(2000.0), {0.0, 0.0}, flat);
    CHECK(has_failure(short_r, "length:"));

    // Seams: a 3486 m shuttle that never leaves physics tile (0, 0) - x
    // ~100, y 1 <-> 250, 14 legs.
    std::vector<RoutePoint> shuttle;
    for (int i = 0; i < 15; ++i) {
        shuttle.push_back({100.0 + 0.01 * i, (i % 2 == 0) ? 1.0 : 250.0});
    }
    const rg::RouteCheckReport s = rg::check_route(shuttle, shuttle.front(), flat);
    CHECK(s.seam_crossings == 0);
    CHECK(has_failure(s, "seams:"));
    CHECK(has_failure(s, "corner:")); // and every reversal is a ~0 m radius corner

    // Corner: a sharp 90 deg kink reads as the circle through the points
    // 10 m either side of it, (90, 0), (100, 0), (100, 10): 10 / sqrt(2),
    // at the kink itself, whatever the leg length.
    const std::vector<RoutePoint> right_angle = {{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}};
    rg::RouteCheckParams p = loose();
    p.min_corner_radius_m = 30.0;
    const rg::RouteCheckReport ra = rg::check_route(right_angle, {0.0, 0.0}, flat, p);
    CHECK(ra.min_corner_radius_m == Approx(10.0 / std::sqrt(2.0)));
    CHECK(ra.min_corner_at_m == Approx(100.0));
    CHECK(ra.min_corner_x == Approx(100.0));
    CHECK(ra.min_corner_y == Approx(0.0));
    CHECK(has_failure(ra, "corner:"));
    // Listed as one stretch around the kink (the radius is < 30 m from
    // s = 90 to 110 at the most, since the window must straddle the kink).
    CHECK(ra.corner_report_radius_m == Approx(30.0));
    REQUIRE(ra.tight_corners.size() == 1);
    CHECK(ra.tight_corners[0].radius_m == Approx(10.0 / std::sqrt(2.0)));
    CHECK(ra.tight_corners[0].at_m == Approx(100.0));
    CHECK(ra.tight_corners[0].begin_m > 90.0);
    CHECK(ra.tight_corners[0].begin_m < 100.0);
    CHECK(ra.tight_corners[0].end_m > 100.0);
    CHECK(ra.tight_corners[0].end_m < 110.0);
}

TEST_CASE("route_check: the corner radius is the circle through s - 10 m, s, s + 10 m", "[route_check]") {
    const auto flat = [](double, double) -> std::optional<double> { return 0.0; };
    const double pi = 3.14159265358979323846;
    // Straight, then a circular arc of radius r over `sweep_deg` at `step_deg`
    // polyline spacing, then straight again.
    const auto arc_route = [pi](double r, double sweep_deg, double step_deg) {
        std::vector<RoutePoint> pts = {{-100.0, 0.0}};
        const int n = static_cast<int>(std::lround(sweep_deg / step_deg));
        for (int k = 0; k <= n; ++k) {
            const double a = k * step_deg * pi / 180.0;
            pts.push_back({r * std::sin(a), r - r * std::cos(a)});
        }
        const double a_end = sweep_deg * pi / 180.0;
        pts.push_back({pts.back().x + 100.0 * std::cos(a_end), pts.back().y + 100.0 * std::sin(a_end)});
        return pts;
    };

    // A polyline arc approximates the circle: within 1 % at 1 and 2 deg steps.
    for (const double step_deg : {1.0, 2.0}) {
        const std::vector<RoutePoint> pts = arc_route(40.0, 120.0, step_deg);
        const rg::RouteCheckReport r = rg::check_route(pts, pts.front(), flat, loose());
        INFO("step_deg " << step_deg);
        CHECK(r.min_corner_radius_m == Approx(40.0).epsilon(0.01));
        CHECK(r.tight_corners.empty()); // never below 30 m
    }

    // Density independence: splitting every segment into 5 collinear pieces
    // (as the ~5 m route densification does) leaves the measured curve, and
    // so the radius, unchanged. The retired tangent-arc-per-vertex measure
    // shrank with the vertex spacing instead.
    const std::vector<RoutePoint> coarse = arc_route(40.0, 120.0, 10.0);
    std::vector<RoutePoint> dense = {coarse.front()};
    for (std::size_t i = 1; i < coarse.size(); ++i) {
        for (int k = 1; k <= 5; ++k) {
            const double t = k / 5.0;
            dense.push_back({coarse[i - 1].x + t * (coarse[i].x - coarse[i - 1].x),
                             coarse[i - 1].y + t * (coarse[i].y - coarse[i - 1].y)});
        }
    }
    const rg::RouteCheckReport rc = rg::check_route(coarse, coarse.front(), flat, loose());
    const rg::RouteCheckReport rd = rg::check_route(dense, dense.front(), flat, loose());
    CHECK(rd.min_corner_radius_m == Approx(rc.min_corner_radius_m).epsilon(1e-9));
    CHECK(rc.min_corner_radius_m > 30.0); // a 40 m bend at 10 deg steps is still no tight corner

    // 20 m radius: below the 30 m default -> one listed stretch, a failure.
    const std::vector<RoutePoint> tight = arc_route(20.0, 90.0, 2.0);
    const rg::RouteCheckReport tr = rg::check_route(tight, tight.front(), flat);
    INFO(rg::format_route_report(tr));
    CHECK(tr.min_corner_radius_m == Approx(20.0).epsilon(0.01));
    CHECK(has_failure(tr, "corner:"));
    REQUIRE(tr.tight_corners.size() == 1);
    // The arc starts at s = 100 and is 20 * pi / 2 = 31.4 m long.
    CHECK(tr.tight_corners[0].begin_m > 90.0);
    CHECK(tr.tight_corners[0].end_m < 100.0 + 31.5 + 10.0);

    // Two separate tight bends make two listed stretches, in route order.
    std::vector<RoutePoint> two = {{0.0, 0.0}, {200.0, 0.0}, {200.0, 200.0}, {400.0, 200.0}};
    const rg::RouteCheckReport tw = rg::check_route(two, two.front(), flat, loose());
    REQUIRE(tw.tight_corners.size() == 2);
    CHECK(tw.tight_corners[0].at_m == Approx(200.0));
    CHECK(tw.tight_corners[1].at_m == Approx(400.0));

    // A full reversal is radius 0; a route shorter than two windows has no
    // measurable corner.
    const rg::RouteCheckReport rv = rg::check_route({{0.0, 0.0}, {50.0, 0.0}, {0.0, 0.0}}, {0.0, 0.0}, flat, loose());
    CHECK(rv.min_corner_radius_m == 0.0);
    const rg::RouteCheckReport sh = rg::check_route({{0.0, 0.0}, {10.0, 0.0}, {10.0, 5.0}}, {0.0, 0.0}, flat, loose());
    CHECK(std::isinf(sh.min_corner_radius_m));
    CHECK(sh.min_corner_at_m == -1.0);

    // Degenerate input is a failure, not UB.
    CHECK_FALSE(rg::check_route({{0.0, 0.0}}, {0.0, 0.0}, flat).ok());
    CHECK_FALSE(rg::check_route({{0.0, 0.0}, {0.0, 0.0}}, {0.0, 0.0}, flat).ok());
}

TEST_CASE("route_check: sample_l0_height is bilinear on the cell-centre lattice across L0 tile borders",
          "[route_check]") {
    // Session origin on a tile corner; tiles for x/y indices (e0/256 - 1 ..
    // e0/256 + 1), synthesized from a linear field h(c, r) = 0.25*(c - E0) +
    // 0.5*(r - N0) metres of the global column/row - exact in 1/256 m, and
    // bilinear interpolation reproduces a linear field exactly.
    constexpr std::int64_t kE0 = 256 * 1000;
    constexpr std::int64_t kN0 = 256 * 20000;
    const g2m::geo::UtmZone zone{32};
    std::map<std::pair<std::int32_t, std::int32_t>, std::unique_ptr<g2m::HeightTile>> tiles;
    for (std::int32_t ty = 19999; ty <= 20001; ++ty) {
        for (std::int32_t tx = 999; tx <= 1001; ++tx) {
            auto t = std::make_unique<g2m::HeightTile>();
            t->key.zone = zone;
            t->key.level = 0;
            t->key.x = tx;
            t->key.y = ty;
            for (std::int64_t j = 0; j < 256; ++j) {
                for (std::int64_t i = 0; i < 256; ++i) {
                    const std::int64_t c = std::int64_t{tx} * 256 + i;
                    const std::int64_t r = std::int64_t{ty} * 256 + j;
                    t->h[static_cast<std::size_t>(j * 256 + i)] =
                        static_cast<std::int32_t>(64 * (c - kE0) + 128 * (r - kN0));
                }
            }
            tiles.emplace(std::make_pair(tx, ty), std::move(t));
        }
    }
    int lookups = 0;
    std::pair<std::int32_t, std::int32_t> missing{-1, -1};
    const rg::L0TileLookupFn lookup = [&](const g2m::TileKey& k) -> const g2m::HeightTile* {
        ++lookups;
        const auto key = std::make_pair(k.x, k.y);
        if (k.level != 0 || key == missing) {
            return nullptr;
        }
        const auto it = tiles.find(key);
        return it == tiles.end() ? nullptr : it->second.get();
    };
    const auto expected = [](double x, double y) { return 0.25 * (x - 0.5) + 0.5 * (y - 0.5); };

    // Inside one tile, on the tile corner (4 tiles), on a west/south border,
    // and at negative session coordinates.
    for (const auto& p : std::vector<RoutePoint>{{100.25, 37.75}, {0.0, 0.0}, {0.3, 128.0}, {-0.2, -0.4},
                                                 {-100.9, -3.3}, {255.9, 256.2}, {511.4, 300.0}}) {
        const std::optional<double> h = rg::sample_l0_height(lookup, zone, kE0, kN0, p.x, p.y);
        INFO("x=" << p.x << " y=" << p.y);
        REQUIRE(h.has_value());
        CHECK(*h == Approx(expected(p.x, p.y)).margin(1e-9));
    }

    // Exactly on a cell centre: that sample, no interpolation.
    const std::optional<double> centre = rg::sample_l0_height(lookup, zone, kE0, kN0, 10.5, 20.5);
    REQUIRE(centre.has_value());
    CHECK(*centre == Approx(0.25 * 10 + 0.5 * 20));

    // A NoData sample poisons exactly the four cells around it.
    tiles.at({1000, 20000})->h[static_cast<std::size_t>(5 * 256 + 5)] = g2m::kHeightNoData;
    CHECK_FALSE(rg::sample_l0_height(lookup, zone, kE0, kN0, 5.6, 5.6).has_value());
    CHECK_FALSE(rg::sample_l0_height(lookup, zone, kE0, kN0, 5.4, 5.4).has_value());
    CHECK(rg::sample_l0_height(lookup, zone, kE0, kN0, 6.6, 6.6).has_value());

    // A missing tile (lookup returns nullptr) is NoData too - also for a
    // point whose other three corners are fine.
    missing = {999, 20000};
    CHECK_FALSE(rg::sample_l0_height(lookup, zone, kE0, kN0, -50.0, 50.0).has_value());
    CHECK_FALSE(rg::sample_l0_height(lookup, zone, kE0, kN0, 0.2, 50.0).has_value());
    CHECK(rg::sample_l0_height(lookup, zone, kE0, kN0, 0.6, 50.0).has_value());

    // And the whole check through it: a route across the missing tile.
    const rg::HeightAtFn height_at = [&](double x, double y) {
        return rg::sample_l0_height(lookup, zone, kE0, kN0, x, y);
    };
    const rg::RouteCheckReport r =
        rg::check_route({{-200.0, 50.0}, {200.0, 50.0}}, {-200.0, 50.0}, height_at, loose());
    CHECK(r.nodata_samples == 201); // s = 0 .. 200 (x = -200 .. 0.0; x = 0 still needs column -1)
    CHECK(r.first_nodata_at_m == 0.0);
    CHECK(lookups > 0);
}

TEST_CASE("route_check: load_route reads rg.route/1 and ignores unknown keys", "[route_check]") {
    const TempFile f(kValidRoute);
    std::string err;
    const std::optional<rg::Route> r = rg::load_route(f.path(), &err);
    INFO(err);
    REQUIRE(r.has_value());
    CHECK(err.empty());
    CHECK(r->format == "rg.route/1");
    CHECK(r->name == "t");
    CHECK(r->zone == 32);
    CHECK(r->e0 == 464000.0);
    CHECK(r->n0 == 5559000.0);
    CHECK(r->spawn.x == -876.0);
    CHECK(r->spawn.y == 208.0);
    CHECK(r->spawn_yaw_deg == 45.0);
    REQUIRE(r->waypoints.size() == 3);
    CHECK(r->waypoints[1].x == -800.5);
    CHECK(r->waypoints[1].y == 250.25);
    // No "criteria" object: nothing overridden.
    CHECK_FALSE(r->criteria.min_length_m.has_value());
    CHECK_FALSE(r->criteria.max_grade.has_value());
    CHECK_FALSE(r->criteria.min_seam_crossings.has_value());
    CHECK_FALSE(r->criteria.min_corner_radius_m.has_value());
}

TEST_CASE("route_check: a route file's criteria override the defaults, field by field", "[route_check]") {
    const std::string text = replace(kValidRoute, "\"waypoints\"",
                                     "\"criteria\": {\"max_grade_pct\": 31.5, \"min_corner_radius_m\": 7}, \"waypoints\"");
    const TempFile f(text);
    std::string err;
    const std::optional<rg::Route> r = rg::load_route(f.path(), &err);
    INFO(err);
    REQUIRE(r.has_value());
    REQUIRE(r->criteria.max_grade.has_value());
    CHECK(*r->criteria.max_grade == Approx(0.315));
    REQUIRE(r->criteria.min_corner_radius_m.has_value());
    CHECK(*r->criteria.min_corner_radius_m == 7.0);
    CHECK_FALSE(r->criteria.min_length_m.has_value());
    CHECK_FALSE(r->criteria.min_seam_crossings.has_value());

    rg::RouteCheckParams params;
    rg::apply_route_criteria(r->criteria, params);
    CHECK(params.max_grade == Approx(0.315));
    CHECK(params.min_corner_radius_m == 7.0);
    CHECK(params.min_length_m == 3000.0); // defaults kept
    CHECK(params.min_seam_crossings == 12);

    // A later layer (the tool's command line) wins over the route file.
    rg::RouteCriteria cli;
    cli.min_seam_crossings = 3;
    cli.max_grade = 0.2;
    rg::apply_route_criteria(cli, params);
    CHECK(params.max_grade == Approx(0.2));
    CHECK(params.min_seam_crossings == 3);
    CHECK(params.min_corner_radius_m == 7.0);

    // All four keys.
    const TempFile all(replace(kValidRoute, "\"waypoints\"",
                               "\"criteria\": {\"min_length_m\": 9000, \"max_grade_pct\": 8, "
                               "\"min_seam_crossings\": 40, \"min_corner_radius_m\": 12.5}, \"waypoints\""));
    const std::optional<rg::Route> ra = rg::load_route(all.path(), &err);
    REQUIRE(ra.has_value());
    rg::RouteCheckParams pa;
    rg::apply_route_criteria(ra->criteria, pa);
    CHECK(pa.min_length_m == 9000.0);
    CHECK(pa.max_grade == Approx(0.08));
    CHECK(pa.min_seam_crossings == 40);
    CHECK(pa.min_corner_radius_m == 12.5);
}

TEST_CASE("route_check: load_route error paths", "[route_check]") {
    const std::string base = kValidRoute;
    const std::vector<std::pair<std::string, std::string>> cases = {
        {"{ not json", "malformed JSON"},
        {"[1, 2]", "top level must be an object"},
        {replace(base, "\"rg.route/1\"", "\"rg.route/2\""), "unsupported \"format\""},
        {replace(base, "\"name\": \"t\",", ""), "missing required field \"name\""},
        {replace(base, "\"zone\": 32", "\"zone\": 32.5"), "must be an integer in 1..60"},
        {replace(base, "\"zone\": 32", "\"zone\": 61"), "must be an integer in 1..60"},
        {replace(base, "\"e0\": 464000", "\"e0\": \"464000\""), "\"e0\""},
        {replace(base, "\"spawn\": {\"x\": -876.0, \"y\": 208.0, \"yaw_deg\": 45},", ""),
         "missing required field \"spawn\""},
        {replace(base, ", \"yaw_deg\": 45", ""), "missing required field \"yaw_deg\""},
        {replace(base, "[[-876.0, 208.0], [-800.5, 250.25], [0, 0]]", "{}"), "\"waypoints\" must be an array"},
        {replace(base, "[[-876.0, 208.0], [-800.5, 250.25], [0, 0]]", "[[-876.0, 208.0]]"), "at least 2 points"},
        {replace(base, "[-800.5, 250.25]", "[-800.5]"), "waypoint 1 must be an [x, y] pair"},
        {replace(base, "[-800.5, 250.25]", "[-800.5, \"y\"]"), "waypoint 1 must be an [x, y] pair"},
        {replace(base, "[0, 0]]", "[0, 0, 0]]"), "waypoint 2 must be an [x, y] pair"},
        {replace(base, "\"waypoints\"", "\"criteria\": [], \"waypoints\""), "\"criteria\" must be an object"},
        {replace(base, "\"waypoints\"", "\"criteria\": {\"max_grade\": 0.3}, \"waypoints\""),
         "unknown key \"max_grade\" in \"criteria\""},
        {replace(base, "\"waypoints\"", "\"criteria\": {\"max_grade_pct\": \"31\"}, \"waypoints\""),
         "\"max_grade_pct\" in \"criteria\" must be a number"},
        {replace(base, "\"waypoints\"", "\"criteria\": {\"max_grade_pct\": 0}, \"waypoints\""),
         "\"criteria.max_grade_pct\" must be > 0"},
        {replace(base, "\"waypoints\"", "\"criteria\": {\"min_seam_crossings\": 2.5}, \"waypoints\""),
         "\"criteria.min_seam_crossings\" must be an integer >= 0"},
        {replace(base, "\"waypoints\"", "\"criteria\": {\"min_corner_radius_m\": -1}, \"waypoints\""),
         "\"criteria.min_corner_radius_m\" must be >= 0"},
        {replace(base, "\"waypoints\"", "\"criteria\": {\"min_length_m\": -1}, \"waypoints\""),
         "\"criteria.min_length_m\" must be >= 0"},
    };
    for (const auto& [text, needle] : cases) {
        const TempFile f(text);
        std::string err;
        INFO("expecting: " << needle);
        CHECK_FALSE(rg::load_route(f.path(), &err).has_value());
        INFO("got: " << err);
        CHECK(err.find(needle) != std::string::npos);
        CHECK(err.rfind(f.path(), 0) == 0);
    }
    std::string err;
    CHECK_FALSE(rg::load_route("this/file/does/not/exist.json", &err).has_value());
    CHECK(err.find("cannot open file") != std::string::npos);
}

TEST_CASE("route_check: route_matches_world compares frame, spawn and yaw", "[route_check]") {
    rg::Route r;
    r.zone = 32;
    r.e0 = 464000.0;
    r.n0 = 5559000.0;
    r.spawn = {-876.0, 208.0};
    r.spawn_yaw_deg = 45.0;
    CHECK(rg::route_matches_world(r, 32, 464000.0, 5559000.0, 463124.0, 5559208.0, 45.0).empty());
    CHECK(rg::route_matches_world(r, 32, 464000.0, 5559000.0, 463124.005, 5559208.0, 45.0).empty()); // tolerance
    CHECK(rg::route_matches_world(r, 32, 464000.0, 5559000.0, 462500.0, 5559500.0, 45.0).find("spawn") !=
          std::string::npos);
    CHECK(rg::route_matches_world(r, 32, 464000.0, 5559000.0, 463124.0, 5559208.0, 0.0).find("yaw") !=
          std::string::npos);
    CHECK(rg::route_matches_world(r, 33, 464000.0, 5559000.0, 463124.0, 5559208.0, 45.0)
              .find("session_origin_utm") != std::string::npos);
}

// Real data: the committed route against the real home-r1 store. Hidden
// ([.]) and SKIPped unless RG_G2M_HOME is set - CI never reads cache/.
TEST_CASE("route_check: the committed home_r1_drive route passes on the real store", "[.][realdata][route_check]") {
    if (!safe_getenv("RG_G2M_HOME").has_value()) {
        SKIP("RG_G2M_HOME not set");
    }
    std::string err;
    const auto world = rg::load_world_config(std::string(RG_SOURCE_DIR) + "/data/world/world_config.json", &err);
    INFO(err);
    REQUIRE(world.has_value());
    const auto route = rg::load_route(std::string(RG_SOURCE_DIR) + "/data/routes/home_r1_drive.json", &err);
    REQUIRE(route.has_value());
    std::unique_ptr<rg::WorldTerrain> terrain = rg::WorldTerrain::open(*world, &err);
    REQUIRE(terrain != nullptr);
    rg::RouteCheckParams params;
    rg::apply_route_criteria(route->criteria, params); // the route's own limits, as tools/route_check applies them
    const rg::RouteCheckReport r = rg::check_route_on_world(*route, *world, *terrain, params);
    INFO(rg::format_route_report(r));
    CHECK(r.ok());
}
