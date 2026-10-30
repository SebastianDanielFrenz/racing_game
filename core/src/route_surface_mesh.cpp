#include "rg/route_surface_mesh.h"

#include "g2m/layer/road_geom_tile.h"
#include "g2m/core/geom/project.h"
#include "g2m/world/road_structures.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

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

struct PathMaterial {
    g2m::SurfaceKind surface = g2m::SurfaceKind::Dirt;
    bool paved = false;
    double width_m = 1.5;
};

std::optional<double> clean_positive_width(std::optional<std::string_view> value) {
    if (!value) return std::nullopt;
    const std::string text(*value);
    char* end = nullptr;
    const double width = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || !std::isfinite(width) || width < 0.6 || width > 12.0)
        return std::nullopt;
    return width;
}

std::optional<PathMaterial> path_material(const g2m::osm::OsmData& data, const g2m::osm::Way& way) {
    const auto tags = data.tags_of(way);
    const auto highway = data.tag(tags, "highway");
    if (!highway) return std::nullopt;
    if (data.tag(tags, "area").value_or(std::string_view{}) == "yes") return std::nullopt;
    // These pedestrian/cycle ways are deliberately separate from the drivable road graph. Carriageways, tracks,
    // service roads and junctions continue to come from the served RoadGeomTile path above.
    double default_width = 1.5;
    if (*highway == "cycleway") default_width = 2.0;
    else if (*highway == "pedestrian") default_width = 3.0;
    else if (*highway == "steps") default_width = 1.2;
    else if (*highway == "bridleway") default_width = 2.0;
    else if (*highway != "footway" && *highway != "path") return std::nullopt;

    const auto footway = data.tag(tags, "footway");
    const auto surface_tag = data.tag(tags, "surface");
    PathMaterial result;
    result.width_m = clean_positive_width(data.tag(tags, "width")).value_or(default_width);
    if (surface_tag) {
        const std::string_view s = *surface_tag;
        if (s == "asphalt") { result.surface = g2m::SurfaceKind::Asphalt; result.paved = true; }
        else if (s == "concrete") { result.surface = g2m::SurfaceKind::Concrete; result.paved = true; }
        else if (s == "paving_stones") { result.surface = g2m::SurfaceKind::PavingStones; result.paved = true; }
        else if (s == "sett" || s == "cobblestone") { result.surface = g2m::SurfaceKind::Sett; result.paved = true; }
        else if (s == "gravel" || s == "fine_gravel") result.surface = g2m::SurfaceKind::Gravel;
        else if (s == "compacted" || s == "unpaved") result.surface = g2m::SurfaceKind::Compacted;
        else if (s == "grass") result.surface = g2m::SurfaceKind::Grass;
        else if (s == "sand") result.surface = g2m::SurfaceKind::Sand;
        else result.surface = g2m::SurfaceKind::Dirt;
    } else if ((*highway == "footway" && footway && *footway == "sidewalk") || *highway == "pedestrian") {
        result.surface = g2m::SurfaceKind::PavingStones;
        result.paved = true;
    } else if (*highway == "cycleway") {
        result.surface = g2m::SurfaceKind::Asphalt;
        result.paved = true;
    }
    return result;
}

struct Point2 { double e = 0, n = 0; };

Point2 add(Point2 a, Point2 b) { return {a.e + b.e, a.n + b.n}; }
Point2 sub(Point2 a, Point2 b) { return {a.e - b.e, a.n - b.n}; }
Point2 scale(Point2 a, double s) { return {a.e * s, a.n * s}; }
double dot(Point2 a, Point2 b) { return a.e * b.e + a.n * b.n; }
double length(Point2 a) { return std::hypot(a.e, a.n); }
Point2 unit(Point2 a) { const double d = length(a); return d > 1e-8 ? scale(a, 1.0 / d) : Point2{}; }

std::vector<Point2> round_path_corners(const std::vector<Point2>& source) {
    if (source.size() < 2) return source;
    std::vector<Point2> rounded;
    rounded.reserve(source.size() * 3);
    rounded.push_back(source.front());
    for (std::size_t i = 1; i + 1 < source.size(); ++i) {
        const Point2 in = unit(sub(source[i], source[i - 1]));
        const Point2 out = unit(sub(source[i + 1], source[i]));
        const double before = length(sub(source[i], source[i - 1]));
        const double after = length(sub(source[i + 1], source[i]));
        const double turn = dot(in, out);
        if (before < 0.1 || after < 0.1 || turn > 0.985 || turn < -0.8) {
            rounded.push_back(source[i]);
            continue;
        }
        const double trim = std::min({2.0, before * 0.25, after * 0.25});
        if (trim < 0.25) { rounded.push_back(source[i]); continue; }
        const Point2 entry = sub(source[i], scale(in, trim));
        const Point2 exit = add(source[i], scale(out, trim));
        rounded.push_back(entry);
        const int steps = std::clamp(static_cast<int>(std::ceil(trim * 1.5)), 2, 8);
        for (int j = 1; j < steps; ++j) {
            const double t = static_cast<double>(j) / steps, one = 1.0 - t;
            rounded.push_back(add(add(scale(entry, one * one), scale(source[i], 2.0 * one * t)),
                                  scale(exit, t * t)));
        }
        rounded.push_back(exit);
    }
    rounded.push_back(source.back());

    // Two-node OSM ways need the same terrain sampling as paths with corners.
    // Returning their endpoints directly makes long floating triangles.
    std::vector<Point2> dense;
    dense.push_back(rounded.front());
    for (std::size_t i = 0; i + 1 < rounded.size(); ++i) {
        const Point2 a = rounded[i], b = rounded[i + 1];
        const int segments = std::max(1, static_cast<int>(std::ceil(length(sub(b, a)) / 1.0)));
        for (int j = 1; j <= segments; ++j)
            dense.push_back(add(a, scale(sub(b, a), static_cast<double>(j) / segments)));
    }
    return dense;
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

void append_standalone_path_mesh(g2m::world::RoadRibbonTile& mesh, const g2m::OsmTile& osm,
                                const g2m::TileKey& key, const g2m::world::RoadRibbonGround& ground,
                                double lift_m) {
    if (!ground) return;
    mesh.key = key;
    const double ox = static_cast<double>(key.min_easting()), oy = static_cast<double>(key.min_northing());
    const auto zone = key.zone;
    const g2m::geo::Utm utm;
    const auto& data = osm.data;
    constexpr int kUnpaved = 8192;
    constexpr int kWalkway = 32768;

    for (const auto& way : data.ways) {
        const auto material = path_material(data, way);
        if (!material || way.refs_count < 2) continue;
        const auto tags = data.tags_of(way);
        if (data.tag(tags, "bridge").value_or(std::string_view{}) == "yes" ||
            data.tag(tags, "tunnel").value_or(std::string_view{}) == "yes") continue;
        std::vector<Point2> original;
        original.reserve(way.refs_count);
        bool valid = true;
        for (const auto id : data.refs_of(way)) {
            const auto* node = data.find_node(id);
            if (!node) { valid = false; break; }
            const auto projected = g2m::geom::project_e7_to_mm(utm, zone, node->lat, node->lon);
            if (!projected) { valid = false; break; }
            const Point2 p{static_cast<double>(projected->x) / 1000.0,
                           static_cast<double>(projected->y) / 1000.0};
            if (original.empty() || length(sub(p, original.back())) > 0.05) original.push_back(p);
        }
        if (!valid || original.size() < 2) continue;
        const auto path = round_path_corners(original);
        if (path.size() < 2) continue;
        const double half = material->width_m * 0.5;
        int route_flags = (material->paved
            ? material_flags(material->surface, true, g2m::LaneKind::Sidewalk)
            : material_flags(material->surface, false, g2m::LaneKind::Sidewalk)) | kWalkway;
        if (!material->paved) route_flags |= kUnpaved;
        const float flags = static_cast<float>(route_flags);
        const float surface = static_cast<float>(material->surface);
        double station = 0.0;
        for (std::size_t i = 0; i + 1 < path.size(); ++i) {
            const double station0 = station;
            station += length(sub(path[i + 1], path[i]));
            const double station1 = station;
            const Point2 midpoint = scale(add(path[i], path[i + 1]), 0.5);
            if (!owned(key, midpoint.e, midpoint.n)) continue;
            const Point2 tangent = unit(sub(path[i + 1], path[i]));
            if (length(tangent) < 0.5) continue;
            const auto cross_section = [&](std::size_t at) {
                const Point2 before = at == 0 ? tangent : unit(sub(path[at], path[at - 1]));
                const Point2 after = at + 1 == path.size() ? tangent : unit(sub(path[at + 1], path[at]));
                const Point2 n0{-before.n, before.e}, n1{-after.n, after.e};
                Point2 normal = unit(add(n0, n1));
                if (length(normal) < 0.5) normal = n1;
                const double denom = std::max(0.5, std::abs(dot(normal, n1)));
                const double offset = std::min(half / denom, half * 2.0);
                return std::pair<Point2, Point2>{add(path[at], scale(normal, offset)),
                                                  sub(path[at], scale(normal, offset))};
            };
            const auto [left0, right0] = cross_section(i);
            const auto [left1, right1] = cross_section(i + 1);
            const std::array<Point2,4> corners{left0,right0,left1,right1};
            std::array<double,4> heights{};
            bool has_ground = true;
            for (std::size_t j = 0; j < corners.size(); ++j) {
                const auto h = ground(corners[j].e, corners[j].n);
                if (!h) { has_ground = false; break; }
                heights[j] = *h + lift_m + 0.01;
            }
            if (!has_ground) continue;
            const auto first = static_cast<std::int32_t>(mesh.vertices.size());
            const auto vertex = [&](Point2 p, double z, float lateral, double station) {
                mesh.vertices.push_back({static_cast<float>(p.e - ox), static_cast<float>(p.n - oy),
                    static_cast<float>(z), lateral, static_cast<float>(station),
                    static_cast<float>(material->width_m), 1.0f, surface, 0.0f,
                    static_cast<float>(route_flags), 0.0f});
            };
            vertex(left0, heights[0], static_cast<float>(half), station0); vertex(right0, heights[1], static_cast<float>(-half), station0);
            vertex(left1, heights[2], static_cast<float>(half), station1); vertex(right1, heights[3], static_cast<float>(-half), station1);
            mesh.indices.insert(mesh.indices.end(), {first, first+1, first+2, first+1, first+3, first+2});
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
