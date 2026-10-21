#include "rg/surface_layers.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rg {
namespace {

constexpr int kDecalFlags = 16 | 32; // g2m kPaint | kOverlay
constexpr int kWalkwayFlag = 32768;
constexpr int kUnpavedFlag = 8192;
constexpr double kCellM = 8.0;
constexpr float kPatchWidthM = 500.0f;
constexpr double kMinIntersectionM2 = 1e-6; // shared edges and vertices are not an overlap
constexpr double kMinPieceM2 = 2e-4;        // slivers left over from a cut are dropped

enum Rank : int { kDecal = -1, kVerge = 0, kWalkway = 1, kUnpavedStrip = 2, kCarriageway = 3, kJunctionPatch = 4 };

using Vertex = g2m::world::RoadRibbonVertex;

int rank_of(const Vertex& v) {
    const int flags = static_cast<int>(v.flags + 0.5f); // flags are small non-negative integers
    if ((flags & kDecalFlags) != 0) return kDecal;
    if (v.verge > 0.5f) return kVerge;
    if ((flags & kWalkwayFlag) != 0) return kWalkway;
    if (v.width > kPatchWidthM) return kJunctionPatch; // add_junction_surfaces marks its patches with width 1000
    if ((flags & kUnpavedFlag) != 0) return kUnpavedStrip; // earth/gravel route strip (append_route_surface_mesh)
    return kCarriageway;
}

// A polygon corner: position plus barycentric weights on the triangle it was cut from.
struct Corner { double x, y, b0, b1, b2; };
using Polygon = std::vector<Corner>;

double area(const Polygon& p) {
    double a = 0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Corner& c = p[i];
        const Corner& d = p[(i + 1) % p.size()];
        a += c.x * d.y - d.x * c.y;
    }
    return 0.5 * std::abs(a);
}

// Keep one side of the directed line (px,py)->(qx,qy): inside = left of it (counter-clockwise triangles).
Polygon clip(const Polygon& in, double px, double py, double qx, double qy, bool keep_inside) {
    Polygon out;
    const auto side = [&](const Corner& c) {
        const double s = (qx - px) * (c.y - py) - (qy - py) * (c.x - px);
        return keep_inside ? s : -s;
    };
    for (std::size_t i = 0; i < in.size(); ++i) {
        const Corner& a = in[i];
        const Corner& b = in[(i + 1) % in.size()];
        const double sa = side(a), sb = side(b);
        if (sa >= 0) out.push_back(a);
        if ((sa >= 0) != (sb >= 0)) {
            const double t = sa / (sa - sb);
            out.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.b0 + (b.b0 - a.b0) * t,
                           a.b1 + (b.b1 - a.b1) * t, a.b2 + (b.b2 - a.b2) * t});
        }
    }
    return out;
}

struct Tri {
    float min_x = 0, min_y = 0, max_x = 0, max_y = 0, mean_z = 0;
    std::int8_t rank = kCarriageway;
    int order = 0; // rank * 16 + (carriageway: lane count, at most 15): the higher order covers the lower one
};

} // namespace

void resolve_surface_layers(g2m::world::RoadRibbonTile& mesh) {
    auto& verts = mesh.vertices;
    std::vector<std::int8_t> vertex_rank(verts.size());
    for (std::size_t i = 0; i < verts.size(); ++i) {
        auto& v = verts[i];
        vertex_rank[i] = static_cast<std::int8_t>(rank_of(v));
        if (vertex_rank[i] == kDecal) v.z += static_cast<float>(kDecalLiftM);
    }

    const std::size_t tri_count = mesh.indices.size() / 3;
    std::vector<Tri> tris(tri_count);
    struct Cell { std::array<std::vector<std::int32_t>, 4> by_rank; }; // covers of rank 1, 2, 3, 4
    std::unordered_map<std::int64_t, Cell> grid;
    const auto cell_key = [](int ix, int iy) {
        return (static_cast<std::int64_t>(ix) << 32) ^ static_cast<std::int64_t>(static_cast<std::uint32_t>(iy));
    };
    const auto cell_lo = [](float v) { return static_cast<int>(std::floor(v / static_cast<float>(kCellM))); };
    for (std::size_t t = 0; t < tri_count; ++t) {
        Tri& tr = tris[t];
        const auto i0 = static_cast<std::size_t>(mesh.indices[t * 3]), i1 = static_cast<std::size_t>(mesh.indices[t * 3 + 1]),
                   i2 = static_cast<std::size_t>(mesh.indices[t * 3 + 2]);
        const Vertex& a = verts[i0];
        const Vertex& b = verts[i1];
        const Vertex& c = verts[i2];
        const int ra = vertex_rank[i0], rb = vertex_rank[i1], rc = vertex_rank[i2];
        tr.rank = static_cast<std::int8_t>((ra == rb && rb == rc) ? ra : kCarriageway); // a mixed triangle is never cut
        tr.order = tr.rank * 16;
        if (tr.rank == kCarriageway) tr.order += std::clamp(static_cast<int>(std::lround(a.lanes)), 0, 15);
        tr.min_x = std::min({a.x, b.x, c.x});
        tr.max_x = std::max({a.x, b.x, c.x});
        tr.min_y = std::min({a.y, b.y, c.y});
        tr.max_y = std::max({a.y, b.y, c.y});
        tr.mean_z = (a.z + b.z + c.z) / 3.0f;
    }

    // Only cells a cuttable triangle touches can need a cover: a verge, walkway, unpaved-strip or carriageway triangle, or
    // a ribbon under a junction patch (so the cells of the patches). Index the higher layers in those cells only.
    std::unordered_set<std::int64_t> needed;
    for (std::size_t t = 0; t < tri_count; ++t) {
        const Tri& tr = tris[t];
        if (tr.rank == kDecal) continue;
        for (int ix = cell_lo(tr.min_x); ix <= cell_lo(tr.max_x); ++ix)
            for (int iy = cell_lo(tr.min_y); iy <= cell_lo(tr.max_y); ++iy) needed.insert(cell_key(ix, iy));
    }
    if (needed.empty()) return;
    for (std::size_t t = 0; t < tri_count; ++t) {
        const Tri& tr = tris[t];
        if (tr.rank < kWalkway) continue;
        for (int ix = cell_lo(tr.min_x); ix <= cell_lo(tr.max_x); ++ix)
            for (int iy = cell_lo(tr.min_y); iy <= cell_lo(tr.max_y); ++iy) {
                const auto key = cell_key(ix, iy);
                if (needed.count(key) != 0) grid[key].by_rank[static_cast<std::size_t>(tr.rank - 1)].push_back(static_cast<std::int32_t>(t));
            }
    }

    std::vector<std::int32_t> kept;
    kept.reserve(mesh.indices.size());
    std::vector<std::int32_t> candidates;
    for (std::size_t t = 0; t < tri_count; ++t) {
        const Tri& tr = tris[t];
        const auto keep_whole = [&] { for (std::size_t k = 0; k < 3; ++k) kept.push_back(mesh.indices[t * 3 + k]); };
        if (tr.rank == kDecal || tr.rank == kJunctionPatch) { keep_whole(); continue; }
        candidates.clear();
        for (int ix = cell_lo(tr.min_x); ix <= cell_lo(tr.max_x); ++ix)
            for (int iy = cell_lo(tr.min_y); iy <= cell_lo(tr.max_y); ++iy) {
                const auto it = grid.find(cell_key(ix, iy));
                if (it == grid.end()) continue;
                for (int r = std::max<int>(tr.rank, 1); r <= kJunctionPatch; ++r)
                  for (const std::int32_t o : it->second.by_rank[static_cast<std::size_t>(r - 1)]) {
                    const Tri& other = tris[static_cast<std::size_t>(o)];
                    if (other.order <= tr.order) continue;
                    if (other.max_x < tr.min_x || other.min_x > tr.max_x || other.max_y < tr.min_y || other.min_y > tr.max_y) continue;
                    if (std::abs(other.mean_z - tr.mean_z) > static_cast<float>(kSameSurfaceHeightM)) continue;
                    candidates.push_back(o);
                }
            }
        if (candidates.empty()) { keep_whole(); continue; }
        std::sort(candidates.begin(), candidates.end());
        candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());

        const Vertex va = verts[static_cast<std::size_t>(mesh.indices[t * 3])];
        const Vertex vb = verts[static_cast<std::size_t>(mesh.indices[t * 3 + 1])];
        const Vertex vc = verts[static_cast<std::size_t>(mesh.indices[t * 3 + 2])];
        std::vector<Polygon> pieces{Polygon{{va.x, va.y, 1, 0, 0}, {vb.x, vb.y, 0, 1, 0}, {vc.x, vc.y, 0, 0, 1}}};
        bool cut = false;
        for (const std::int32_t o : candidates) {
            std::array<double, 6> q{};
            {   // the cover's corners, counter-clockwise
                const auto base = static_cast<std::size_t>(o) * 3;
                const Vertex& a = verts[static_cast<std::size_t>(mesh.indices[base])];
                const Vertex& b = verts[static_cast<std::size_t>(mesh.indices[base + 1])];
                const Vertex& c = verts[static_cast<std::size_t>(mesh.indices[base + 2])];
                q = {a.x, a.y, b.x, b.y, c.x, c.y};
                if ((q[2] - q[0]) * (q[5] - q[1]) - (q[4] - q[0]) * (q[3] - q[1]) < 0) { std::swap(q[2], q[4]); std::swap(q[3], q[5]); }
            }
            std::vector<Polygon> next;
            for (Polygon& piece : pieces) {
                // Separating-axis early out (no allocation): every corner on the outer side of one cover edge.
                bool apart = false;
                for (std::size_t e = 0; e < 3 && !apart; ++e) {
                    const std::size_t f = (e + 1) % 3;
                    apart = true;
                    for (const Corner& c : piece)
                        if ((q[f * 2] - q[e * 2]) * (c.y - q[e * 2 + 1]) - (q[f * 2 + 1] - q[e * 2 + 1]) * (c.x - q[e * 2]) > 1e-9) { apart = false; break; }
                }
                if (apart) { next.push_back(std::move(piece)); continue; }
                Polygon inside = piece;
                std::vector<Polygon> outside;
                for (std::size_t e = 0; e < 3 && !inside.empty(); ++e) {
                    const std::size_t f = (e + 1) % 3;
                    Polygon out = clip(inside, q[e * 2], q[e * 2 + 1], q[f * 2], q[f * 2 + 1], false);
                    if (out.size() >= 3) outside.push_back(std::move(out));
                    inside = clip(inside, q[e * 2], q[e * 2 + 1], q[f * 2], q[f * 2 + 1], true);
                }
                if (inside.size() < 3 || area(inside) < kMinIntersectionM2) { next.push_back(std::move(piece)); continue; }
                cut = true;
                for (Polygon& out : outside)
                    if (area(out) >= kMinPieceM2) next.push_back(std::move(out));
            }
            pieces = std::move(next);
            if (pieces.empty()) break;
        }
        if (!cut) { keep_whole(); continue; }
        const auto blend = [&](const Corner& c) {
            Vertex r = va; // lanes, forward, flags and verge are constant over the triangle (rank_of checked the flags)
            const auto mix = [&](float Vertex::*m) { r.*m = static_cast<float>(c.b0 * (va.*m) + c.b1 * (vb.*m) + c.b2 * (vc.*m)); };
            mix(&Vertex::x); mix(&Vertex::y); mix(&Vertex::z); mix(&Vertex::u); mix(&Vertex::v);
            mix(&Vertex::width); mix(&Vertex::fade);
            return r;
        };
        for (const Polygon& piece : pieces) {
            if (piece.size() < 3) continue;
            const auto base = static_cast<std::int32_t>(verts.size());
            for (const Corner& c : piece) verts.push_back(blend(c));
            for (std::size_t k = 1; k + 1 < piece.size(); ++k) {
                kept.push_back(base);
                kept.push_back(base + static_cast<std::int32_t>(k));
                kept.push_back(base + static_cast<std::int32_t>(k + 1));
            }
        }
    }
    mesh.indices = std::move(kept);
}

} // namespace rg
