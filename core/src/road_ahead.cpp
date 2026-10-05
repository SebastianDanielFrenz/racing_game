// rg/road_ahead.cpp - see road_ahead.h.
#include "rg/road_ahead.h"

#include <cmath>
#include <cstdint>

namespace rg {

namespace {

struct V2 {
    double x = 0.0;
    double y = 0.0;
};

V2 rel_m(const g2m::geom::PointMm& p, std::int64_t ox_mm, std::int64_t oy_mm) {
    return {static_cast<double>(p.x - ox_mm) * 1.0e-3, static_cast<double>(p.y - oy_mm) * 1.0e-3};
}

double dot(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }

double len(V2 a) { return std::sqrt(a.x * a.x + a.y * a.y); }

struct SegRef {
    const g2m::RoadSegment* seg = nullptr;
    V2 a; // relative to the car, metres
    V2 b;
};

} // namespace

bool trace_road_ahead(const std::vector<std::span<const g2m::RoadSegment>>& tiles, double car_e, double car_n, double hx,
                      double hy, double anchor_x, double anchor_y, const RoadAheadParams& params,
                      std::vector<RoadPoint>& out) {
    out.clear();
    const double hlen = std::sqrt(hx * hx + hy * hy);
    if (!(hlen > 1.0e-9) || !std::isfinite(car_e) || !std::isfinite(car_n)) return false;
    const V2 heading{hx / hlen, hy / hlen};
    const std::int64_t ox = std::llround(car_e * 1000.0);
    const std::int64_t oy = std::llround(car_n * 1000.0);

    // Start segment: closest one roughly along the heading.
    const g2m::RoadSegment* start = nullptr;
    double best_dist = 1.0e300;
    V2 start_a{}, start_b{};
    for (const auto& span : tiles) {
        for (const g2m::RoadSegment& s : span) {
            const V2 a = rel_m(s.a, ox, oy);
            const V2 b = rel_m(s.b, ox, oy);
            const V2 ab{b.x - a.x, b.y - a.y};
            const double l = len(ab);
            if (l < 1.0e-6) continue;
            const V2 dir{ab.x / l, ab.y / l};
            if (std::fabs(dot(dir, heading)) < params.min_heading_cos) continue;
            // Distance from the origin (the car) to the segment.
            const double t = std::fmax(0.0, std::fmin(1.0, -dot(a, ab) / (l * l)));
            const V2 q{a.x + ab.x * t, a.y + ab.y * t};
            const double d = len(q);
            if (d > params.max_snap_m) continue;
            if (d < best_dist || (d == best_dist && start != nullptr && s.way_id < start->way_id)) {
                best_dist = d;
                start = &s;
                start_a = a;
                start_b = b;
            }
        }
    }
    if (start == nullptr) return false;

    // Direction of travel along the start segment.
    V2 ab{start_b.x - start_a.x, start_b.y - start_a.y};
    double l = len(ab);
    V2 dir{ab.x / l, ab.y / l};
    bool forward = dot(dir, heading) >= 0.0;
    V2 from = forward ? start_a : start_b;
    V2 to = forward ? start_b : start_a;
    g2m::geom::PointMm to_mm = forward ? start->b : start->a;
    const double half_start = static_cast<double>(start->half_width_mm) * 1.0e-3;

    // First point: the car's projection onto the start segment.
    {
        const V2 f2t{to.x - from.x, to.y - from.y};
        const double fl = len(f2t);
        const double t = std::fmax(0.0, std::fmin(1.0, -dot(from, f2t) / (fl * fl)));
        out.push_back({anchor_x + from.x + f2t.x * t, anchor_y + from.y + f2t.y * t, half_start});
    }
    out.push_back({anchor_x + to.x, anchor_y + to.y, half_start});
    double length_m = len({to.x - (out[0].x - anchor_x), to.y - (out[0].y - anchor_y)});

    const double tol_mm = params.joint_tolerance_m * 1000.0;
    const auto near_mm = [tol_mm](const g2m::geom::PointMm& p, const g2m::geom::PointMm& q) {
        return std::fabs(static_cast<double>(p.x - q.x)) <= tol_mm && std::fabs(static_cast<double>(p.y - q.y)) <= tol_mm;
    };

    const g2m::RoadSegment* current = start;
    V2 cur_dir = dir;
    if (!forward) cur_dir = {-dir.x, -dir.y};
    for (int hop = 0; hop < params.max_hops && length_m < params.max_length_m; ++hop) {
        const g2m::RoadSegment* next = nullptr;
        double best_cos = params.max_turn_cos;
        V2 next_to{};
        g2m::geom::PointMm next_to_mm{};
        V2 next_dir{};
        for (const auto& span : tiles) {
            for (const g2m::RoadSegment& s : span) {
                if (&s == current) continue;
                const bool at_a = near_mm(s.a, to_mm);
                const bool at_b = !at_a && near_mm(s.b, to_mm);
                if (!at_a && !at_b) continue;
                const V2 far_pt = at_a ? rel_m(s.b, ox, oy) : rel_m(s.a, ox, oy);
                const V2 near_pt = at_a ? rel_m(s.a, ox, oy) : rel_m(s.b, ox, oy);
                const V2 d{far_pt.x - near_pt.x, far_pt.y - near_pt.y};
                const double dl = len(d);
                if (dl < 1.0e-6) continue;
                const V2 dn{d.x / dl, d.y / dl};
                const double c = dot(dn, cur_dir);
                if (c > best_cos || (c == best_cos && next != nullptr && s.way_id < next->way_id)) {
                    best_cos = c;
                    next = &s;
                    next_to = far_pt;
                    next_to_mm = at_a ? s.b : s.a;
                    next_dir = dn;
                }
            }
        }
        if (next == nullptr) break;
        const V2 prev = {out.back().x - anchor_x, out.back().y - anchor_y};
        length_m += len({next_to.x - prev.x, next_to.y - prev.y});
        out.push_back({anchor_x + next_to.x, anchor_y + next_to.y, static_cast<double>(next->half_width_mm) * 1.0e-3});
        current = next;
        to_mm = next_to_mm;
        cur_dir = next_dir;
    }
    return true;
}

} // namespace rg
