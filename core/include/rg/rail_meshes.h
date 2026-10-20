// rg/rail_meshes.h - railways (decision RACE-028): the geometry of the served g2m.rail.geom layer.
//
// Engine-neutral, no Godot type. A g2m::RailGeomTile (centreline + a height profile taken from the rail line itself,
// docs/formats/rail_geom.md in geo2map_engine) becomes
//   * a visual mesh per tile: on a bridge piece a deck slab, a ballast trough and two parapets; on a ground piece a
//     ballast bed (a trapezoid sunk into the terrain); rails on top of both. Declined pieces (no DEM under a needed
//     sample), tunnels (v1 draws no portals), and classes switched off in the config are not drawn;
//   * a collision deck per bridge piece (a closed slab prism), with every span dropped whose underside is not at
//     least `collision.min_clearance_m` above the ground there (the road under a rail bridge must never be blocked
//     by it: a span with too little clearance keeps its look, but a vehicle drives through it).
//
// Every number is data: data/world/rail.json (rg.rail/1), loaded by load_rail_config(). The same approach as
// rg/road_structures.h (geo2map builds, the game consumes), but the geometry lives here because the layer is
// the game's own opt-in.
//
// Coordinates: vertices are metres east / north of the tile corner (key.x * 1024, key.y * 1024) and height as
// served (the elevation datum), like g2m::world::RoadRibbonTile. Triangles are counter-clockwise seen from outside
// in that right-handed (east, north, up) frame; a renderer with another handedness reverses them.
#pragma once

#include "g2m/layer/rail_geom_tile.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace rg {

struct RailColour {
    float r = 0.5f, g = 0.5f, b = 0.5f;
};

struct RailConfig {
    // Sampling and drawing.
    double sample_spacing_m = 8.0;          // longest step along the centreline
    int max_tracks_drawn = 4;               // a piece tagged with more tracks draws this many
    double track_spacing_m = 4.5;           // between adjacent tracks of one piece
    double default_gauge_mm = 1435.0;       // when the piece carries none
    bool draw_class[4] = {true, true, false, false}; // by RailClassRules::id: rail, light_rail, subway, tram
    bool draw_construction = true;          // railway=construction pieces
    bool construction_rails = false;        // rails on a construction piece (deck and ballast are drawn either way)
    bool draw_declined = false;             // pieces without a usable profile
    // Bridge: deck slab under a ballast trough, parapets at both edges.
    double deck_width_m = 5.4;              // single track; + track_spacing_m per extra track
    double deck_thickness_m = 0.9;
    double parapet_height_m = 1.1;          // above the track bed level
    double parapet_width_m = 0.3;
    // Track bed: ballast trapezoid.
    double ballast_top_width_m = 3.6;       // single track; + track_spacing_m per extra track
    double ballast_height_m = 0.35;         // on a deck (the profile height is the top of the ballast)
    double ballast_side_slope = 1.5;        // horizontal metres per vertical metre
    double ballast_depth_m = 1.2;           // how far a ground bed reaches below the profile height
    // Rails.
    double rail_head_width_m = 0.07;
    double rail_height_m = 0.17;
    RailColour deck_colour{0.52f, 0.52f, 0.50f};
    RailColour parapet_colour{0.62f, 0.62f, 0.60f};
    RailColour ballast_colour{0.38f, 0.35f, 0.32f};
    RailColour rail_colour{0.22f, 0.22f, 0.24f};
    // Collision decks.
    bool collision_enabled = true;
    double collision_min_clearance_m = 5.0;  // underside above the ground below, else the span has no collision
    bool collision_unknown_ground_blocks = false; // no ground height known: false drops the span (never block)
    double collision_edge_margin_m = 0.5;   // the ground is probed at the centre and this far inside both edges
    // Streaming: tiles around the viewer (game side, rail_stream.gd).
    double view_distance_m = 2500.0;
};

[[nodiscard]] std::optional<RailConfig> load_rail_config(const std::string& path, std::string* err);
// The same validation for a parsed config (tests build configs in code).
[[nodiscard]] bool validate_rail_config(const RailConfig& config, std::string* err);

// One mesh, flat shaded (a vertex belongs to one face).
struct RailMesh {
    std::vector<float> positions; // 3 per vertex
    std::vector<float> normals;   // 3 per vertex
    std::vector<float> colours;   // 3 per vertex (r, g, b)
    std::vector<std::uint32_t> indices;
    [[nodiscard]] std::size_t vertex_count() const { return positions.size() / 3; }
    [[nodiscard]] std::size_t triangle_count() const { return indices.size() / 3; }
    [[nodiscard]] bool empty() const { return indices.empty(); }
};

struct RailMeshStats {
    int pieces = 0;
    int drawn = 0;
    int bridges = 0;            // bridge pieces drawn
    int ground = 0;             // ground pieces drawn
    int skipped_tunnel = 0;
    int skipped_declined = 0;
    int skipped_class = 0;
    int skipped_construction = 0;
};

struct RailTileMesh {
    double origin_e = 0.0, origin_n = 0.0; // absolute UTM of the tile corner
    RailMesh mesh;
    RailMeshStats stats;
};

[[nodiscard]] RailTileMesh build_rail_tile_mesh(const g2m::RailGeomTile& tile, const RailConfig& config);

// The ground height under an absolute UTM point, or nullopt when not known.
using RailGroundFn = std::function<std::optional<double>(double easting, double northing)>;

// One collision deck: a closed slab prism over the spans of a bridge piece that clear the ground, local to
// (origin_e, origin_n); indices 16 bit (a piece is at most 4096 points, a deck far less).
struct RailDeckMesh {
    std::int64_t way_id = 0;
    std::uint32_t piece = 0;
    double origin_e = 0.0, origin_n = 0.0;
    std::vector<float> positions; // 3 per vertex, x east, y north, z up
    std::vector<std::uint16_t> indices;
    double min_easting = 0, max_easting = 0, min_northing = 0, max_northing = 0;
    int spans = 0;           // spans kept
    int spans_dropped = 0;   // spans without collision (too little clearance or unknown ground)
};

// Bridge pieces only. A bridge whose every span is dropped gives no deck. `ground` null = no clearance test (every
// span kept; tests and the flat world).
[[nodiscard]] std::vector<RailDeckMesh> build_rail_deck_meshes(const g2m::RailGeomTile& tile, const RailConfig& config,
                                                               const RailGroundFn& ground, int* unknown_ground_spans = nullptr);

} // namespace rg
