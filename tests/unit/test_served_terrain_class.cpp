// test_served_terrain_class.cpp - render class window from the served g2m.terrain.class layer (ViewSource::
// terrain_classes) against the game's own OSM road raster (ClassLookup), on one synthetic road scene, plus the
// default-off guarantee (an empty TerrainClassLookup changes no byte).
//
// The served class tile is built here from the road geometry itself (a node is road when it lies within the
// half-width of the centre line; the higher rank wins where roads cross), which is what the server's ownership
// reduces to for a straight road without junction or blend. Where the two paths can differ is the vertex-to-node
// rule (chunk_terrain_classes.h): a render vertex sits at a cell centre (N + 0.5 m) but reads the class node at the
// floor of its position, its south-west node. The OSM raster classifies the cell centre itself. A road edge that lies
// between a node and the cell centre half a metre away is therefore classified differently: the first case places
// the edges so both agree, the second shows the one-row difference.
#include "rg/world_terrain.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

namespace {

using g2m::LandClass;

const g2m::geo::UtmZone kZone{32, g2m::geo::Hemisphere::North};
constexpr double kE0 = 464000.0;
constexpr double kN0 = 5559000.0;
// Chunk (7250, 86860) of level 0 spans E 464000..464064, N 5559040..5559104 (64 m); cx % 4 == 2, cy % 4 == 0, so it
// reads columns 128..192 of class tile (1812, 21715), whose SW corner is (463872, 5559040).
constexpr std::int32_t kCx = 7250, kCy = 86860;
constexpr double kChunkE = 64.0 * kCx, kChunkN = 64.0 * kCy;
constexpr std::uint8_t kPaved = static_cast<std::uint8_t>(LandClass::PavedRoad);
constexpr std::uint8_t kUnpaved = static_cast<std::uint8_t>(LandClass::UnpavedRoad);
constexpr std::uint8_t kForest = static_cast<std::uint8_t>(LandClass::TreeCover);

struct Road {
    bool horizontal;      // along E at northing `centre`, else along N at easting `centre` (chunk-local metres)
    double centre_m;      // chunk-local
    double half_width_m;
    LandClass cls;
    int rank;
};

// A flat 100 m height tile for every key (the window gather needs the neighbours too).
const g2m::HeightTile* flat_lookup(void*, const g2m::TileKey& key) {
    static std::vector<std::unique_ptr<g2m::HeightTile>> keep;
    static std::mutex m;
    std::lock_guard<std::mutex> lk(m);
    for (const auto& t : keep)
        if (t->key == key) return t.get();
    auto t = std::make_unique<g2m::HeightTile>();
    t->key = key;
    t->h.fill(256 * 100);
    keep.push_back(std::move(t));
    return keep.back().get();
}

struct Scene {
    std::vector<Road> roads;
    std::shared_ptr<const std::vector<g2m::RoadSegment>> segments;
    std::shared_ptr<const g2m::TerrainClassTile> tile;
};

Scene make_scene(const std::vector<Road>& roads) {
    Scene s;
    s.roads = roads;
    std::vector<g2m::RoadSegment> segs;
    std::int64_t way = 1;
    for (const Road& r : roads) {
        g2m::RoadSegment seg;
        const auto mm = [](double m) { return static_cast<std::int64_t>(std::llround(m * 1000.0)); };
        const std::int64_t base_e = mm(kChunkE), base_n = mm(kChunkN);
        if (r.horizontal) {
            seg.a = {base_e - 200000, base_n + mm(r.centre_m)};
            seg.b = {base_e + 300000, base_n + mm(r.centre_m)};
        } else {
            seg.a = {base_e + mm(r.centre_m), base_n - 200000};
            seg.b = {base_e + mm(r.centre_m), base_n + 300000};
        }
        seg.half_width_mm = mm(r.half_width_m);
        seg.land_class = r.cls;
        seg.rank = r.rank;
        seg.way_id = way++;
        segs.push_back(seg);
    }
    // extract_road_segments hands the rasteriser its segments sorted by rank (later paints over earlier).
    std::stable_sort(segs.begin(), segs.end(), [](const g2m::RoadSegment& x, const g2m::RoadSegment& y) { return x.rank < y.rank; });
    s.segments = std::make_shared<const std::vector<g2m::RoadSegment>>(std::move(segs));

    auto tile = std::make_shared<g2m::TerrainClassTile>();
    const g2m::TileKey key = g2m::world::terrain_class_tile_for_chunk({kZone, 0, kCx, kCy});
    tile->reset(key);
    const double tile_e = 256.0 * key.x, tile_n = 256.0 * key.y;
    for (int j = 0; j < g2m::kTerrainClassSamples; ++j)
        for (int i = 0; i < g2m::kTerrainClassSamples; ++i) {
            const std::size_t idx = g2m::TerrainClassTile::index(i, j);
            tile->land_class[idx] = kForest;
            int best = 0;
            for (const Road& r : roads) {
                const double e = tile_e + i - kChunkE, n = tile_n + j - kChunkN; // chunk-local node position
                const double d = std::abs((r.horizontal ? n : e) - r.centre_m);
                if (d <= r.half_width_m && r.rank > best) {
                    best = r.rank;
                    tile->land_class[idx] = static_cast<std::uint8_t>(r.cls);
                    tile->rank[idx] = static_cast<std::uint8_t>(r.rank);
                }
            }
        }
    s.tile = tile;
    return s;
}

std::shared_ptr<const std::vector<g2m::RoadSegment>> osm_lookup(void* ctx, const g2m::TileKey&) {
    return static_cast<Scene*>(ctx)->segments;
}
std::shared_ptr<const g2m::TerrainClassTile> served_lookup(void* ctx, const g2m::TileKey& key) {
    const Scene* s = static_cast<Scene*>(ctx);
    return key == s->tile->key ? s->tile : nullptr;
}

enum class Path { Osm, Served, None };

std::vector<rg::RenderChunk> build(Scene& scene, Path path, const g2m::RenderRoadLod* lod = nullptr,
                                   const rg::RoadSurfaceMap& surfaces = {}) {
    const std::vector<g2m::mesh::ChunkKey> keys{{kZone, 0, kCx, kCy}};
    g2m::world::ClassLookup osm;
    g2m::world::TerrainClassLookup served;
    if (path == Path::Osm) osm = g2m::world::ClassLookup{&osm_lookup, &scene, lod};
    if (path == Path::Served) served = g2m::world::TerrainClassLookup{&served_lookup, &scene, lod};
    std::vector<rg::RenderChunk> out;
    REQUIRE(rg::build_render_chunks(keys, &flat_lookup, nullptr, kE0, kN0, 1, out, nullptr, osm, surfaces, nullptr,
                                    served));
    REQUIRE(out.size() == 1);
    return out;
}

std::set<std::size_t> vertices_of(const rg::RenderChunk& c, std::uint8_t cls) {
    std::set<std::size_t> v;
    for (std::size_t i = 0; i < static_cast<std::size_t>(g2m::mesh::kBaseVertexCount); ++i) // base lattice, not the skirts
        if (c.mesh.land_class[i] == cls) v.insert(i);
    return v;
}

constexpr std::size_t kVerts = g2m::mesh::kChunkVerts;

} // namespace

TEST_CASE("served terrain class: same paved/unpaved vertices as the OSM raster when edges avoid the half-cell band",
          "[served_class][terrain_class]") {
    // Horizontal paved road, edges at 7.6 and 12.9 m: nodes 8..12 and cell centres 8.5..12.5 agree. Vertical unpaved
    // track, edges at 18.6 and 21.9 m: columns 19..21 either way. Paved outranks unpaved where they cross.
    Scene scene = make_scene({{true, 10.25, 2.65, LandClass::PavedRoad, 7}, {false, 20.25, 1.65, LandClass::UnpavedRoad, 1}});
    const auto osm = build(scene, Path::Osm);
    const auto served = build(scene, Path::Served);

    const auto osm_paved = vertices_of(osm[0], kPaved), served_paved = vertices_of(served[0], kPaved);
    const auto osm_unpaved = vertices_of(osm[0], kUnpaved), served_unpaved = vertices_of(served[0], kUnpaved);
    REQUIRE_FALSE(osm_paved.empty());
    REQUIRE_FALSE(osm_unpaved.empty());
    CHECK(osm_paved == served_paved);
    CHECK(osm_unpaved == served_unpaved);

    // The expected geometry itself: rows 8..12 of the paved road (all 65 columns), columns 19..21 of the track.
    CHECK(osm_paved.size() == 5 * kVerts);
    for (std::size_t j = 8; j <= 12; ++j) CHECK(osm_paved.count(j * kVerts + 40) == 1);
    CHECK(osm_unpaved.size() == 3 * (kVerts - 5));
    CHECK(osm_unpaved.count(30 * kVerts + 19) == 1);
    CHECK(osm_unpaved.count(10 * kVerts + 20) == 0); // crossing: the paved road wins in both paths

    // Colours: the road vertices resolve through the same RoadSurfaceMap, so the whole rgba plane is equal (the
    // served tile's other class is forest, which falls through to the height ramp like Unknown does).
    const auto osm_c = build(scene, Path::Osm), served_c = build(scene, Path::Served);
    CHECK(osm_c[0].rgba == served_c[0].rgba);
    rg::RoadSurfaceMap renamed;
    renamed.paved = "concrete"; // unknown colour name: road vertices fall back to the ramp in both paths
    CHECK(build(scene, Path::Osm, nullptr, renamed)[0].rgba == build(scene, Path::Served, nullptr, renamed)[0].rgba);
}

TEST_CASE("served terrain class: a road edge inside the half-cell band differs by exactly one row", "[served_class][terrain_class]") {
    // Edges at 7.35 and 12.65 m. Nodes (served): 8..12. Cell centres (OSM): row 7 as well (7.5 lies inside), rows
    // 8..12; row 12.5 is inside, row 13.5 is not. So OSM paints rows 7..12, served rows 8..12.
    Scene scene = make_scene({{true, 10.0, 2.65, LandClass::PavedRoad, 7}});
    const auto osm = build(scene, Path::Osm), served = build(scene, Path::Served);
    const auto a = vertices_of(osm[0], kPaved), b = vertices_of(served[0], kPaved);
    CHECK(b.size() == 5 * kVerts);
    CHECK(a.size() == 6 * kVerts);
    std::set<std::size_t> only_osm;
    for (std::size_t v : a)
        if (b.count(v) == 0) only_osm.insert(v);
    for (std::size_t v : b) CHECK(a.count(v) == 1); // served is a subset
    REQUIRE(only_osm.size() == kVerts);
    for (std::size_t v : only_osm) CHECK(v / kVerts == 7); // the southern edge row only
}

TEST_CASE("served terrain class: the road LOD policy reaches the served path", "[served_class][terrain_class]") {
    Scene scene = make_scene({{true, 10.25, 2.65, LandClass::PavedRoad, 7}});
    g2m::RenderRoadLod off = g2m::default_render_road_lod();
    off.levels[0].enabled = false;
    CHECK(vertices_of(build(scene, Path::Served, &off)[0], kPaved).empty());
    CHECK(vertices_of(build(scene, Path::Osm, &off)[0], kPaved).empty());
    CHECK_FALSE(vertices_of(build(scene, Path::Served, nullptr)[0], kPaved).empty());
}

TEST_CASE("served terrain class: no tile means no class data, and an unset lookup changes no byte", "[served_class][terrain_class]") {
    Scene scene = make_scene({{true, 10.25, 2.65, LandClass::PavedRoad, 7}});
    // Default arguments vs an explicit empty TerrainClassLookup: identical meshes and colours (the OSM path).
    const std::vector<g2m::mesh::ChunkKey> keys{{kZone, 0, kCx, kCy}};
    g2m::world::ClassLookup osm{&osm_lookup, &scene, nullptr};
    std::vector<rg::RenderChunk> before, after;
    REQUIRE(rg::build_render_chunks(keys, &flat_lookup, nullptr, kE0, kN0, 1, before, nullptr, osm));
    REQUIRE(rg::build_render_chunks(keys, &flat_lookup, nullptr, kE0, kN0, 1, after, nullptr, osm, {}, nullptr,
                                    g2m::world::TerrainClassLookup{}));
    REQUIRE(before.size() == 1);
    CHECK(before[0].mesh.content_hash == after[0].mesh.content_hash);
    CHECK(before[0].rgba == after[0].rgba);
    CHECK(before[0].mesh.land_class == after[0].mesh.land_class);
    CHECK_FALSE(vertices_of(before[0], kPaved).empty());

    // A served lookup that finds no tile: all-Unknown, no road vertices (the same as an uncovered OSM chunk).
    Scene empty = scene;
    empty.tile = std::make_shared<g2m::TerrainClassTile>(); // key does not match: served_lookup returns null
    const auto none = build(empty, Path::Served);
    CHECK(vertices_of(none[0], kPaved).empty());
    CHECK(vertices_of(none[0], kForest).empty());
}
