#include "rg/route_surface_mesh.h"

#include "g2m/layer/road_geom_tile.h"
#include "g2m/world/road_structures.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace rg {
namespace {

constexpr int kStone = 2048;
constexpr int kConcrete = 4096;
constexpr int kUnpaved = 8192;
constexpr int kSett = 16384;
constexpr int kWalkway = 32768; // consumer-neutral tag for tests/physics extraction; ignored by the visual shader.

int material_flags(g2m::SurfaceKind surface, bool paved, g2m::LaneKind kind) {
    if (!paved) return kUnpaved;
    switch (surface) {
        case g2m::SurfaceKind::PavingStones: return kStone;
        case g2m::SurfaceKind::Sett:
        case g2m::SurfaceKind::Cobblestone: return kStone | kSett;
        case g2m::SurfaceKind::Concrete: return kConcrete;
        case g2m::SurfaceKind::Unknown:
            return kind == g2m::LaneKind::Sidewalk ? kStone : 0;
        default: return 0;
    }
}

bool owned(const g2m::TileKey& key, double e, double n) {
    return std::floor(e / static_cast<double>(key.size_m())) == key.x &&
           std::floor(n / static_cast<double>(key.size_m())) == key.y;
}

} // namespace

void append_route_surface_mesh(g2m::world::RoadRibbonTile& mesh, const g2m::RoadGeomTile& geometry,
                               const g2m::TileKey& key, const g2m::world::RoadRibbonGround& ground, double lift_m,
                               const g2m::world::RoadSeparation& separation) {
    if (!ground) return;
    const double ox = static_cast<double>(key.min_easting());
    const double oy = static_cast<double>(key.min_northing());
    const auto decks = g2m::world::build_road_decks(geometry, separation);

    // Keep route strips out of the junction box. The existing junction mesh owns that footprint and carries the
    // appropriate arm material, including mixed paved/unpaved crossings.
    std::vector<double> trim_start(geometry.entries.size(), 0.0);
    std::vector<double> trim_end(geometry.entries.size(), 0.0);
    for (const auto& junction : geometry.junctions) {
        if (junction.decline != g2m::JunctionDecline::None) continue;
        for (const auto& approach : junction.approaches) {
            if (approach.status != g2m::JunctionApproachStatus::Used || approach.entry_index >= geometry.entries.size())
                continue;
            auto& trim = approach.at_end ? trim_end[approach.entry_index] : trim_start[approach.entry_index];
            trim = std::max(trim, approach.trim_m);
        }
    }

    for (std::size_t ei = 0; ei < geometry.entries.size(); ++ei) {
        const auto& entry = geometry.entries[ei];
        if (!entry.profile) continue;
        const auto& profile = *entry.profile;
        const bool unpaved_road = profile.attributes.land_class == g2m::LandClass::UnpavedRoad;
        const auto* xs = entry.cross_section ? &*entry.cross_section : nullptr;
        if (!unpaved_road && xs == nullptr) continue;

        struct Strip { std::size_t band; int flags; };
        std::vector<Strip> strips;
        if (xs != nullptr) {
            for (std::size_t bi = 0; bi < xs->bands.size(); ++bi) {
                const auto& band = xs->bands[bi];
                if (band.kind == g2m::LaneKind::Verge) continue;
                const bool explicit_walkway = band.kind == g2m::LaneKind::Sidewalk ||
                                              (band.kind == g2m::LaneKind::Cycle && !band.counted);
                // Paved-road carriageways already have their own high-detail ribbon. Only add independent walkway
                // bands there; unpaved-road cross-sections are entirely represented by this surface mesh.
                if (!unpaved_road && !explicit_walkway) continue;
                if (!unpaved_road && band.counted) continue;
                const int route_tag = (band.kind == g2m::LaneKind::Sidewalk || band.kind == g2m::LaneKind::Cycle)
                    ? kWalkway : 0;
                strips.push_back({bi, material_flags(profile.attributes.surface, band.paved, band.kind) | route_tag});
            }
        } else if (unpaved_road) {
            strips.push_back({std::numeric_limits<std::size_t>::max(), kUnpaved});
        }
        if (strips.empty()) continue;

        const double length = profile.reference.length_m;
        const double from = std::clamp(trim_start[ei], 0.0, length);
        const double to = std::clamp(length - trim_end[ei], from, length);
        if (to - from <= 0.05) continue;
        const int rows = std::max(1, static_cast<int>(std::ceil(to - from)));

        for (int ri = 0; ri < rows; ++ri) {
            const double s0 = from + (to - from) * static_cast<double>(ri) / rows;
            const double s1 = ri + 1 == rows ? to : from + (to - from) * static_cast<double>(ri + 1) / rows;
            const double sm = (s0 + s1) * 0.5;
            const auto mid = profile.reference.at(sm);
            if (!mid.ok() || !owned(key, mid.value().x, mid.value().y)) continue;

            std::array<double, 33> edges0{}, edges1{};
            if (xs != nullptr &&
                (!xs->boundaries_at(s0, std::span<double>(edges0.data(), xs->bands.size() + 1)) ||
                 !xs->boundaries_at(s1, std::span<double>(edges1.data(), xs->bands.size() + 1))))
                continue;

            const auto separated = [&](double station) {
                if (profile.attributes.bridge || profile.attributes.tunnel) return true;
                return std::any_of(decks.begin(), decks.end(), [&](const auto& deck) {
                    return deck->way_id == entry.way_id && deck->start_ref == entry.stretch.start_ref &&
                           station >= deck->start_station && station <= deck->end_station;
                });
            };
            const auto height = [&](double station, double lateral, double e, double n) -> std::optional<double> {
                for (const auto& deck : decks) {
                    if (deck->way_id != entry.way_id || deck->start_ref != entry.stretch.start_ref ||
                        station < deck->start_station - .05 || station > deck->end_station + .05)
                        continue;
                    if (auto z = g2m::world::road_deck_height(*deck,
                            std::clamp(station, deck->start_station, deck->end_station), lateral)) return z;
                }
                if (separated(station)) {
                    const auto pose = profile.at(std::clamp(station, 0.0, length));
                    if (!pose.ok()) return std::nullopt;
                    return pose.value().height_m;
                }
                return ground(e, n);
            };

            for (const auto& strip : strips) {
                double left0 = 0.0, right0 = 0.0, left1 = 0.0, right1 = 0.0;
                if (strip.band == std::numeric_limits<std::size_t>::max()) {
                    const double half = profile.attributes.width_mm / 2000.0;
                    left0 = left1 = half;
                    right0 = right1 = -half;
                } else {
                    const auto bi = strip.band;
                    left0 = edges0[bi]; right0 = edges0[bi + 1];
                    left1 = edges1[bi]; right1 = edges1[bi + 1];
                }
                if (!(left0 > right0) || !(left1 > right1)) continue;
                const int across = std::clamp(static_cast<int>(std::ceil(std::max(left0 - right0, left1 - right1))), 1, 64);
                const auto first = static_cast<std::int32_t>(mesh.vertices.size());
                bool valid = true;
                for (const double station : {s0, s1}) {
                    const auto ref = profile.reference.at(station);
                    if (!ref.ok()) { valid = false; break; }
                    const auto& edges = station == s0 ? edges0 : edges1;
                    const auto band = strip.band;
                    const double left = band == std::numeric_limits<std::size_t>::max()
                        ? profile.attributes.width_mm / 2000.0 : edges[band];
                    const double right = band == std::numeric_limits<std::size_t>::max()
                        ? -profile.attributes.width_mm / 2000.0 : edges[band + 1];
                    const double nx = -std::sin(ref.value().heading), ny = std::cos(ref.value().heading);
                    for (int j = 0; j <= across; ++j) {
                        const double lateral = left + (right - left) * static_cast<double>(j) / across;
                        const double e = ref.value().x + nx * lateral, n = ref.value().y + ny * lateral;
                        const auto z = height(station, lateral, e, n);
                        if (!z) { valid = false; break; }
                        mesh.vertices.push_back({static_cast<float>(e - ox), static_cast<float>(n - oy),
                            static_cast<float>(*z + lift_m), static_cast<float>(lateral), static_cast<float>(station),
                            static_cast<float>(std::max(left - right, 0.1)), 1.0f,
                            static_cast<float>(profile.attributes.surface), 0.0f,
                            static_cast<float>(strip.flags), 0.0f});
                    }
                    if (!valid) break;
                }
                if (!valid) {
                    mesh.vertices.resize(static_cast<std::size_t>(first));
                    continue;
                }
                for (int j = 0; j < across; ++j) {
                    const auto a = first + j, b = a + across + 1;
                    mesh.indices.insert(mesh.indices.end(), {a, b, a + 1, a + 1, b, b + 1});
                }
            }
        }
    }
}

void stamp_route_surface_classes(g2m::TerrainClassTile& classes, const g2m::world::RoadRibbonTile& mesh,
                                 const g2m::world::RoadRibbonGround& terrain_ground) {
    if (classes.key.level != 0) return;
    constexpr int kUnpaved = 8192;
    constexpr int kWalkway = 32768;
    const double class_e0 = static_cast<double>(classes.key.min_easting());
    const double class_n0 = static_cast<double>(classes.key.min_northing());
    const double class_e1 = class_e0 + 256.0;
    const double class_n1 = class_n0 + 256.0;
    const double mesh_e0 = static_cast<double>(mesh.key.min_easting());
    const double mesh_n0 = static_cast<double>(mesh.key.min_northing());
    std::vector<float> stamped_height(g2m::kTerrainClassNodes, -std::numeric_limits<float>::infinity());
    std::vector<std::uint8_t> stamped_priority(g2m::kTerrainClassNodes, 0);
    const auto edge = [](double ax, double ay, double bx, double by, double px, double py) {
        return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
    };
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const auto& a = mesh.vertices[static_cast<std::size_t>(mesh.indices[i])];
        const auto& b = mesh.vertices[static_cast<std::size_t>(mesh.indices[i + 1])];
        const auto& c = mesh.vertices[static_cast<std::size_t>(mesh.indices[i + 2])];
        const int flags = static_cast<int>(std::lround(a.flags));
        const bool unpaved = (flags & kUnpaved) != 0;
        const bool walkway = (flags & kWalkway) != 0;
        const bool paved_bed = !unpaved && !walkway && (flags & (16 | 32)) == 0 &&
                               a.verge <= 0.5f && b.verge <= 0.5f && c.verge <= 0.5f;
        if (!unpaved && !walkway && !paved_bed) continue;

        const double ax = mesh_e0 + a.x, ay = mesh_n0 + a.y;
        const double bx = mesh_e0 + b.x, by = mesh_n0 + b.y;
        const double cx = mesh_e0 + c.x, cy = mesh_n0 + c.y;
        const double area = edge(ax, ay, bx, by, cx, cy);
        if (std::abs(area) < 1e-10) continue;
        const int ix0 = std::max(0, static_cast<int>(std::ceil(std::min({ax,bx,cx}) - class_e0)));
        const int iy0 = std::max(0, static_cast<int>(std::ceil(std::min({ay,by,cy}) - class_n0)));
        const int ix1 = std::min(256, static_cast<int>(std::floor(std::max({ax,bx,cx}) - class_e0)));
        const int iy1 = std::min(256, static_cast<int>(std::floor(std::max({ay,by,cy}) - class_n0)));
        if (ix0 > ix1 || iy0 > iy1) continue;
        const auto visual_surface = static_cast<g2m::SurfaceKind>(std::clamp(
            static_cast<int>(std::lround(a.lanes)), 0, static_cast<int>(g2m::SurfaceKind::Water)));
        auto physics_surface = visual_surface;
        if (paved_bed) {
            physics_surface = (flags & 2048) != 0 ? g2m::SurfaceKind::PavingStones :
                (flags & 4096) != 0 ? g2m::SurfaceKind::Concrete : g2m::SurfaceKind::Asphalt;
        } else if (physics_surface == g2m::SurfaceKind::Unknown) {
            if (unpaved) physics_surface = g2m::SurfaceKind::Dirt;
            else if ((flags & 2048) != 0) physics_surface = g2m::SurfaceKind::PavingStones;
            else physics_surface = g2m::SurfaceKind::Asphalt;
        }
        const auto land = static_cast<std::uint8_t>(unpaved ? g2m::LandClass::UnpavedRoad : g2m::LandClass::PavedRoad);
        const auto role = static_cast<std::uint8_t>(walkway ? g2m::TerrainRole::Sidewalk : g2m::TerrainRole::RoadBed);
        for (int j = iy0; j <= iy1; ++j) {
            const double y = class_n0 + static_cast<double>(j);
            for (int x = ix0; x <= ix1; ++x) {
                const double px = class_e0 + static_cast<double>(x);
                const double e0 = edge(ax,ay,bx,by,px,y);
                const double e1 = edge(bx,by,cx,cy,px,y);
                const double e2 = edge(cx,cy,ax,ay,px,y);
                if (!(area > 0 ? (e0 >= -1e-6 && e1 >= -1e-6 && e2 >= -1e-6)
                               : (e0 <= 1e-6 && e1 <= 1e-6 && e2 <= 1e-6))) continue;
                const double w0 = edge(bx,by,cx,cy,px,y) / area;
                const double w1 = edge(cx,cy,ax,ay,px,y) / area;
                const double w2 = edge(ax,ay,bx,by,px,y) / area;
                const double mesh_z = w0*a.z + w1*b.z + w2*c.z;
                if (terrain_ground) {
                    const auto ground = terrain_ground(px,y);
                    // Elevated bridge/tunnel surfaces stay in the separate deck path. Compare at the physics node
                    // so an upper route cannot stamp material onto a lower road or its ground.
                    if (!ground || std::abs(*ground - mesh_z) > 0.45) continue;
                }
                const auto node = g2m::TerrainClassTile::index(x,j);
                const float height = static_cast<float>(mesh_z);
                // Multiple routes can share a junction or cross at the same map coordinate. The mesh order is not a
                // visibility order; prefer the upper surface, then a paved carriageway over a path at equal height.
                const std::uint8_t priority = paved_bed ? 3 : (walkway ? 2 : 1);
                if (height < stamped_height[node] - 0.12f ||
                    (std::abs(height - stamped_height[node]) <= 0.12f && priority < stamped_priority[node])) continue;
                stamped_height[node] = height;
                stamped_priority[node] = priority;
                classes.land_class[node] = land;
                classes.surface[node] = static_cast<std::uint8_t>(physics_surface);
                classes.role[node] = role;
                classes.rank[node] = std::max<std::uint8_t>(1, classes.rank[node]);
            }
        }
    }
}

} // namespace rg
