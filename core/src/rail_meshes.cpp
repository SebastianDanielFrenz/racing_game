#include "rg/rail_meshes.h"

#include "json_util.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace rg {

namespace {

bool fail(std::string* err, const std::string& message) {
    if (err) *err = message;
    return false;
}

struct Vec2 {
    double x = 0, y = 0;
};

// One station of a piece's centreline, local to the tile corner.
struct Station {
    double e = 0, n = 0, z = 0;
    double rx = 0, ry = 0; // unit lateral direction to the right of travel
    double scale = 1.0;    // mitre factor on lateral offsets
};

constexpr double kEps = 1e-9;

std::vector<Station> sample_piece(const g2m::RailPiece& piece, double tile_e, double tile_n, double spacing) {
    struct P {
        double e, n, z;
    };
    std::vector<P> pts;
    pts.reserve(piece.xy.size() * 2);
    for (std::size_t i = 0; i < piece.xy.size(); ++i) {
        const P p{piece.xy[i].x / 1000.0 - tile_e, piece.xy[i].y / 1000.0 - tile_n, piece.z[i] / 256.0};
        if (!pts.empty()) {
            const P& q = pts.back();
            const double len = std::hypot(p.e - q.e, p.n - q.n);
            if (len < 1e-4) continue;
            const int parts = std::max(1, static_cast<int>(std::ceil(len / spacing)));
            for (int k = 1; k < parts; ++k) {
                const double t = static_cast<double>(k) / parts;
                pts.push_back({q.e + t * (p.e - q.e), q.n + t * (p.n - q.n), q.z + t * (p.z - q.z)});
            }
        }
        pts.push_back(p);
    }
    std::vector<Station> out;
    if (pts.size() < 2) return out;
    // Unit direction of each segment.
    std::vector<Vec2> dir(pts.size() - 1);
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const double dx = pts[i + 1].e - pts[i].e, dy = pts[i + 1].n - pts[i].n, len = std::hypot(dx, dy);
        dir[i] = {dx / len, dy / len};
    }
    out.resize(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
        Vec2 in = dir[i == 0 ? 0 : i - 1], outd = dir[i == pts.size() - 1 ? i - 1 : i];
        Vec2 avg{in.x + outd.x, in.y + outd.y};
        double len = std::hypot(avg.x, avg.y);
        if (len < 1e-6) { avg = outd; len = 1.0; }
        avg.x /= len;
        avg.y /= len;
        // Lateral offsets are measured along the average normal and widened by the mitre so the strip keeps its
        // width through a bend (bounded: a sharp turn does not spike).
        const double cos_half = std::max(0.5, avg.x * outd.x + avg.y * outd.y);
        Station s;
        s.e = pts[i].e;
        s.n = pts[i].n;
        s.z = pts[i].z;
        s.rx = avg.y;
        s.ry = -avg.x;
        s.scale = 1.0 / cos_half;
        out[i] = s;
    }
    return out;
}

struct Pt {
    double u, v;
};
using Poly = std::vector<Pt>;

Poly rect(double u0, double u1, double v0, double v1) { return {{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}}; }

struct V3 {
    double x, y, z;
};
V3 sub(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(V3 a) { return std::sqrt(dot(a, a)); }

V3 at(const Station& s, double u, double v) {
    return {s.e + s.rx * u * s.scale, s.n + s.ry * u * s.scale, s.z + v};
}

void push_vertex(RailMesh& m, V3 p, V3 n, const RailColour& c) {
    m.positions.push_back(static_cast<float>(p.x));
    m.positions.push_back(static_cast<float>(p.y));
    m.positions.push_back(static_cast<float>(p.z));
    m.normals.push_back(static_cast<float>(n.x));
    m.normals.push_back(static_cast<float>(n.y));
    m.normals.push_back(static_cast<float>(n.z));
    m.colours.push_back(c.r);
    m.colours.push_back(c.g);
    m.colours.push_back(c.b);
}

// A triangle with the winding that makes its normal point along `outward` (counter-clockwise seen from outside).
void push_triangle(RailMesh& m, V3 a, V3 b, V3 c, V3 outward, const RailColour& colour) {
    V3 n = cross(sub(b, a), sub(c, a));
    const double len = norm(n);
    if (len < kEps) return;
    if (dot(n, outward) < 0.0) {
        std::swap(b, c);
        n = {-n.x, -n.y, -n.z};
    }
    n = {n.x / len, n.y / len, n.z / len};
    const auto base = static_cast<std::uint32_t>(m.vertex_count());
    push_vertex(m, a, n, colour);
    push_vertex(m, b, n, colour);
    push_vertex(m, c, n, colour);
    m.indices.push_back(base);
    m.indices.push_back(base + 1);
    m.indices.push_back(base + 2);
}

// Sweeps the convex polygon `poly` along the stations: flat-shaded side quads, end caps.
void extrude(RailMesh& m, const std::vector<Station>& st, const Poly& poly, const RailColour& colour) {
    if (st.size() < 2 || poly.size() < 3) return;
    double cu = 0, cv = 0;
    for (const auto& p : poly) { cu += p.u; cv += p.v; }
    cu /= static_cast<double>(poly.size());
    cv /= static_cast<double>(poly.size());
    const std::size_t count = poly.size();
    for (std::size_t i = 0; i + 1 < st.size(); ++i) {
        for (std::size_t k = 0; k < count; ++k) {
            const Pt& a = poly[k];
            const Pt& b = poly[(k + 1) % count];
            const V3 A = at(st[i], a.u, a.v), B = at(st[i], b.u, b.v), C = at(st[i + 1], b.u, b.v), D = at(st[i + 1], a.u, a.v);
            const double ou = (a.u + b.u) * 0.5 - cu, ov = (a.v + b.v) * 0.5 - cv;
            const V3 outward{st[i].rx * ou, st[i].ry * ou, ov};
            push_triangle(m, A, B, C, outward, colour);
            push_triangle(m, A, C, D, outward, colour);
        }
    }
    for (int end = 0; end < 2; ++end) {
        const Station& s = end == 0 ? st.front() : st.back();
        const Station& next = end == 0 ? st[1] : st[st.size() - 2];
        // Out of the strip at this end: away from the neighbouring station.
        const V3 outward{s.e - next.e, s.n - next.n, s.z - next.z};
        for (std::size_t k = 1; k + 1 < count; ++k) {
            push_triangle(m, at(s, poly[0].u, poly[0].v), at(s, poly[k].u, poly[k].v), at(s, poly[k + 1].u, poly[k + 1].v), outward, colour);
        }
    }
}

bool class_drawn(const RailConfig& c, std::uint8_t cls) { return cls < 4 && c.draw_class[cls]; }

} // namespace

bool validate_rail_config(const RailConfig& c, std::string* err) {
    const auto positive = [&](double v, const char* name) {
        return std::isfinite(v) && v > 0.0 ? true : fail(err, std::string(name) + " must be > 0");
    };
    if (!positive(c.sample_spacing_m, "sample_spacing_m") || !positive(c.track_spacing_m, "track_spacing_m") ||
        !positive(c.default_gauge_mm, "default_gauge_mm") || !positive(c.deck_width_m, "bridge.deck_width_m") ||
        !positive(c.deck_thickness_m, "bridge.deck_thickness_m") || !positive(c.parapet_height_m, "bridge.parapet_height_m") ||
        !positive(c.parapet_width_m, "bridge.parapet_width_m") || !positive(c.ballast_top_width_m, "bed.ballast_top_width_m") ||
        !positive(c.ballast_height_m, "bed.ballast_height_m") || !positive(c.ballast_side_slope, "bed.ballast_side_slope") ||
        !positive(c.ballast_depth_m, "bed.ballast_depth_m") || !positive(c.rail_head_width_m, "rail.head_width_m") ||
        !positive(c.rail_height_m, "rail.height_m") || !positive(c.collision_min_clearance_m, "collision.min_clearance_m") ||
        !positive(c.view_distance_m, "view_distance_m"))
        return false;
    if (c.collision_edge_margin_m < 0.0 || !std::isfinite(c.collision_edge_margin_m)) return fail(err, "collision.edge_margin_m must be >= 0");
    if (c.sample_spacing_m < 1.0) return fail(err, "sample_spacing_m must be >= 1 (a piece is up to 512 m)");
    if (c.max_tracks_drawn < 1 || c.max_tracks_drawn > 16) return fail(err, "max_tracks_drawn must be 1..16");
    if (c.ballast_top_width_m + 2.0 * c.parapet_width_m > c.deck_width_m)
        return fail(err, "bed.ballast_top_width_m + 2 * bridge.parapet_width_m must fit in bridge.deck_width_m");
    if (c.rail_head_width_m * 4.0 > c.ballast_top_width_m) return fail(err, "rails do not fit on the ballast");
    return true;
}

std::optional<RailConfig> load_rail_config(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) { fail(err, path + ": cannot read"); return std::nullopt; }
    const auto doc = detail::parse_json(*text);
    if (!doc || !doc->is_object()) { fail(err, path + ": not a JSON object"); return std::nullopt; }
    std::string format;
    if (!detail::get_string(*doc, "format", format) || format != "rg.rail/1") {
        fail(err, path + ": format must be \"rg.rail/1\"");
        return std::nullopt;
    }
    RailConfig c;
    bool ok = true;
    const auto number = [&](const detail::json& o, const char* section, const char* key, double& out) {
        if (!detail::get_number(o, key, out) || !std::isfinite(out)) {
            fail(err, path + ": \"" + section + (section[0] ? "." : "") + key + "\" (number) is required");
            ok = false;
        }
    };
    const auto flag = [&](const detail::json& o, const char* section, const char* key, bool& out) {
        if (!detail::get_bool(o, key, out)) {
            fail(err, path + ": \"" + section + (section[0] ? "." : "") + key + "\" (boolean) is required");
            ok = false;
        }
    };
    const auto object = [&](const detail::json& o, const char* key) -> const detail::json* {
        const auto it = o.find(key);
        if (it == o.end() || !it->is_object()) {
            fail(err, path + ": \"" + key + "\" (object) is required");
            ok = false;
            return nullptr;
        }
        return &*it;
    };
    double tracks = 0;
    number(*doc, "", "sample_spacing_m", c.sample_spacing_m);
    number(*doc, "", "max_tracks_drawn", tracks);
    number(*doc, "", "track_spacing_m", c.track_spacing_m);
    number(*doc, "", "default_gauge_mm", c.default_gauge_mm);
    number(*doc, "", "view_distance_m", c.view_distance_m);
    if (ok) c.max_tracks_drawn = static_cast<int>(tracks);
    if (const auto* d = object(*doc, "draw")) {
        const char* names[4] = {"rail", "light_rail", "subway", "tram"};
        if (const auto* classes = object(*d, "classes")) {
            for (int i = 0; i < 4; ++i) flag(*classes, "draw.classes", names[i], c.draw_class[i]);
        }
        flag(*d, "draw", "construction", c.draw_construction);
        flag(*d, "draw", "construction_rails", c.construction_rails);
        flag(*d, "draw", "declined", c.draw_declined);
    }
    if (const auto* b = object(*doc, "bridge")) {
        number(*b, "bridge", "deck_width_m", c.deck_width_m);
        number(*b, "bridge", "deck_thickness_m", c.deck_thickness_m);
        number(*b, "bridge", "parapet_height_m", c.parapet_height_m);
        number(*b, "bridge", "parapet_width_m", c.parapet_width_m);
    }
    if (const auto* b = object(*doc, "bed")) {
        number(*b, "bed", "ballast_top_width_m", c.ballast_top_width_m);
        number(*b, "bed", "ballast_height_m", c.ballast_height_m);
        number(*b, "bed", "ballast_side_slope", c.ballast_side_slope);
        number(*b, "bed", "ballast_depth_m", c.ballast_depth_m);
    }
    if (const auto* r = object(*doc, "rail")) {
        number(*r, "rail", "head_width_m", c.rail_head_width_m);
        number(*r, "rail", "height_m", c.rail_height_m);
    }
    if (const auto* col = object(*doc, "colours")) {
        const auto colour = [&](const char* key, RailColour& out) {
            const auto it = col->find(key);
            if (it == col->end() || !it->is_array() || it->size() != 3) {
                fail(err, path + ": \"colours." + key + "\" must be [r, g, b]");
                ok = false;
                return;
            }
            double v[3];
            for (int i = 0; i < 3; ++i) {
                if (!(*it)[i].is_number()) { fail(err, path + ": \"colours." + key + "\" must be numbers"); ok = false; return; }
                v[i] = (*it)[i].get<double>();
                if (!(v[i] >= 0.0 && v[i] <= 1.0)) { fail(err, path + ": \"colours." + key + "\" must be in 0..1"); ok = false; return; }
            }
            out = {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
        };
        colour("deck", c.deck_colour);
        colour("parapet", c.parapet_colour);
        colour("ballast", c.ballast_colour);
        colour("rail", c.rail_colour);
    }
    if (const auto* col = object(*doc, "collision")) {
        flag(*col, "collision", "enabled", c.collision_enabled);
        flag(*col, "collision", "unknown_ground_blocks", c.collision_unknown_ground_blocks);
        number(*col, "collision", "min_clearance_m", c.collision_min_clearance_m);
        number(*col, "collision", "edge_margin_m", c.collision_edge_margin_m);
    }
    if (!ok) return std::nullopt;
    std::string verr;
    if (!validate_rail_config(c, &verr)) { fail(err, path + ": " + verr); return std::nullopt; }
    return c;
}

RailTileMesh build_rail_tile_mesh(const g2m::RailGeomTile& tile, const RailConfig& c) {
    RailTileMesh out;
    const double size = static_cast<double>(tile.key.size_m());
    out.origin_e = static_cast<double>(tile.key.x) * size;
    out.origin_n = static_cast<double>(tile.key.y) * size;
    for (const auto& piece : tile.pieces) {
        ++out.stats.pieces;
        if (piece.flags & g2m::kRailTunnel) { ++out.stats.skipped_tunnel; continue; }
        if (!class_drawn(c, piece.cls)) { ++out.stats.skipped_class; continue; }
        const bool construction = piece.condition == g2m::RailCondition::Construction;
        if (construction && !c.draw_construction) { ++out.stats.skipped_construction; continue; }
        if ((piece.flags & g2m::kRailDeclined) && !c.draw_declined) { ++out.stats.skipped_declined; continue; }
        const auto stations = sample_piece(piece, out.origin_e, out.origin_n, c.sample_spacing_m);
        if (stations.size() < 2) continue;
        const int tracks = std::clamp<int>(piece.tracks, 1, c.max_tracks_drawn);
        const double extra = (tracks - 1) * c.track_spacing_m;
        const double ballast_half = (c.ballast_top_width_m + extra) * 0.5;
        const bool bridge = (piece.flags & g2m::kRailBridge) != 0;
        if (bridge) {
            const double hw = (c.deck_width_m + extra) * 0.5;
            const double bh = c.ballast_height_m;
            extrude(out.mesh, stations, rect(-hw, hw, -bh - c.deck_thickness_m, -bh), c.deck_colour);
            const double bottom_half = std::min(ballast_half + bh * c.ballast_side_slope, hw - c.parapet_width_m);
            extrude(out.mesh, stations, {{-bottom_half, -bh}, {bottom_half, -bh}, {ballast_half, 0.0}, {-ballast_half, 0.0}}, c.ballast_colour);
            extrude(out.mesh, stations, rect(-hw, -hw + c.parapet_width_m, -bh, c.parapet_height_m), c.parapet_colour);
            extrude(out.mesh, stations, rect(hw - c.parapet_width_m, hw, -bh, c.parapet_height_m), c.parapet_colour);
            ++out.stats.bridges;
        } else {
            const double bottom_half = ballast_half + c.ballast_depth_m * c.ballast_side_slope;
            extrude(out.mesh, stations, {{-bottom_half, -c.ballast_depth_m}, {bottom_half, -c.ballast_depth_m}, {ballast_half, 0.0}, {-ballast_half, 0.0}}, c.ballast_colour);
            ++out.stats.ground;
        }
        if (!construction || c.construction_rails) {
            const double gauge = (piece.gauge_mm > 0 ? static_cast<double>(piece.gauge_mm) : c.default_gauge_mm) / 1000.0;
            const double head = c.rail_head_width_m;
            for (int t = 0; t < tracks; ++t) {
                const double centre = (t - (tracks - 1) * 0.5) * c.track_spacing_m;
                for (int side = -1; side <= 1; side += 2) {
                    const double u = centre + side * (gauge * 0.5 + head * 0.5);
                    extrude(out.mesh, stations, rect(u - head * 0.5, u + head * 0.5, 0.0, c.rail_height_m), c.rail_colour);
                }
            }
        }
        ++out.stats.drawn;
    }
    return out;
}

std::vector<RailDeckMesh> build_rail_deck_meshes(const g2m::RailGeomTile& tile, const RailConfig& c, const RailGroundFn& ground, int* unknown_ground_spans) {
    std::vector<RailDeckMesh> decks;
    if (unknown_ground_spans) *unknown_ground_spans = 0;
    if (!c.collision_enabled) return decks;
    const double size = static_cast<double>(tile.key.size_m());
    const double origin_e = static_cast<double>(tile.key.x) * size, origin_n = static_cast<double>(tile.key.y) * size;
    for (const auto& piece : tile.pieces) {
        if (!(piece.flags & g2m::kRailBridge) || (piece.flags & (g2m::kRailTunnel | g2m::kRailDeclined))) continue;
        if (!class_drawn(c, piece.cls)) continue;
        if (piece.condition == g2m::RailCondition::Construction && !c.draw_construction) continue;
        const auto st = sample_piece(piece, origin_e, origin_n, c.sample_spacing_m);
        if (st.size() < 2) continue;
        const int tracks = std::clamp<int>(piece.tracks, 1, c.max_tracks_drawn);
        const double hw = (c.deck_width_m + (tracks - 1) * c.track_spacing_m) * 0.5;
        const double top = 0.0, bottom = -(c.ballast_height_m + c.deck_thickness_m);
        // Which stations leave the road below free.
        std::vector<char> blocked(st.size(), 0);
        if (ground) {
            for (std::size_t i = 0; i < st.size(); ++i) {
                const double edge = std::max(0.0, hw - c.collision_edge_margin_m);
                for (const double u : {0.0, -edge, edge}) {
                    const V3 p = at(st[i], u, 0.0);
                    const auto g = ground(p.x + origin_e, p.y + origin_n);
                    if (!g) {
                        if (unknown_ground_spans && !blocked[i]) ++*unknown_ground_spans;
                        if (!c.collision_unknown_ground_blocks) blocked[i] = 1;
                    } else if (p.z + bottom - *g < c.collision_min_clearance_m) {
                        blocked[i] = 1;
                    }
                }
            }
        }
        RailDeckMesh deck;
        deck.way_id = piece.way_id;
        deck.piece = piece.piece;
        deck.origin_e = origin_e;
        deck.origin_n = origin_n;
        deck.min_easting = deck.min_northing = 1e300;
        deck.max_easting = deck.max_northing = -1e300;
        std::vector<V3> verts;
        std::vector<std::uint32_t> tris;
        const auto add_vertex = [&](V3 p) {
            verts.push_back(p);
            return static_cast<std::uint32_t>(verts.size() - 1);
        };
        const auto add_tri = [&](std::uint32_t a, std::uint32_t b, std::uint32_t d, V3 outward) {
            const V3 n = cross(sub(verts[b], verts[a]), sub(verts[d], verts[a]));
            if (norm(n) < kEps) return;
            if (dot(n, outward) < 0.0) std::swap(b, d);
            tris.push_back(a);
            tris.push_back(b);
            tris.push_back(d);
        };
        std::size_t i = 0;
        while (i + 1 < st.size()) {
            if (blocked[i] || blocked[i + 1]) {
                ++deck.spans_dropped;
                ++i;
                continue;
            }
            // A run of kept spans [i, j].
            std::size_t j = i + 1;
            while (j + 1 < st.size() && !blocked[j + 1]) ++j;
            // Corner order per station: bottom-left, bottom-right, top-right, top-left.
            std::vector<std::array<std::uint32_t, 4>> ring;
            for (std::size_t k = i; k <= j; ++k) {
                ring.push_back({add_vertex(at(st[k], -hw, bottom)), add_vertex(at(st[k], hw, bottom)),
                                add_vertex(at(st[k], hw, top)), add_vertex(at(st[k], -hw, top))});
            }
            for (std::size_t k = 0; k + 1 < ring.size(); ++k) {
                const Station& s = st[i + k];
                const V3 out_of[4] = {{0, 0, -1}, {s.rx, s.ry, 0}, {0, 0, 1}, {-s.rx, -s.ry, 0}};
                for (int f = 0; f < 4; ++f) {
                    const int g = (f + 1) % 4;
                    add_tri(ring[k][f], ring[k][g], ring[k + 1][g], out_of[f]);
                    add_tri(ring[k][f], ring[k + 1][g], ring[k + 1][f], out_of[f]);
                }
                ++deck.spans;
            }
            for (int end = 0; end < 2; ++end) {
                const auto& r = end == 0 ? ring.front() : ring.back();
                const Station& s = end == 0 ? st[i] : st[j];
                const Station& nb = end == 0 ? st[i + 1] : st[j - 1];
                const V3 outward{s.e - nb.e, s.n - nb.n, s.z - nb.z};
                add_tri(r[0], r[1], r[2], outward);
                add_tri(r[0], r[2], r[3], outward);
            }
            i = j;
        }
        if (tris.empty()) continue;
        if (verts.size() > 65535) continue;
        for (const V3& v : verts) {
            deck.positions.push_back(static_cast<float>(v.x));
            deck.positions.push_back(static_cast<float>(v.y));
            deck.positions.push_back(static_cast<float>(v.z));
            deck.min_easting = std::min(deck.min_easting, v.x + origin_e);
            deck.max_easting = std::max(deck.max_easting, v.x + origin_e);
            deck.min_northing = std::min(deck.min_northing, v.y + origin_n);
            deck.max_northing = std::max(deck.max_northing, v.y + origin_n);
        }
        for (auto t : tris) deck.indices.push_back(static_cast<std::uint16_t>(t));
        decks.push_back(std::move(deck));
    }
    return decks;
}

} // namespace rg
