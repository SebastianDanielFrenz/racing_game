// rg/terrain_colours.cpp - see rg/terrain_colours.h. Moved from world_terrain.cpp (5a3baed) with the per-vertex
// branch on the surface name turned into a table lookup by class id; tests/unit/test_terrain_colours.cpp pins the bytes.
#include "rg/terrain_colours.h"

#include "g2m/layer/osm_roads.h"

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>
#include <string_view>

namespace rg {

namespace {

std::uint32_t pack_rgba(float r, float g, float b, float a) {
    auto to_u8 = [](float v) { return static_cast<std::uint8_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f); };
    return (static_cast<std::uint32_t>(to_u8(r)) << 24) | (static_cast<std::uint32_t>(to_u8(g)) << 16) |
           (static_cast<std::uint32_t>(to_u8(b)) << 8) | static_cast<std::uint32_t>(to_u8(a));
}

// Height bounds of the home region's terrain (physics_sim CLAUDE.md "Terrain tile system": 90-880 m ASL).
constexpr float kValleyHeightM = 90.0f;
constexpr float kRidgeHeightM = 880.0f;
constexpr float kPavedR = 0.30f, kPavedG = 0.30f, kPavedB = 0.32f;
constexpr float kUnpavedR = 0.36f, kUnpavedG = 0.28f, kUnpavedB = 0.16f;
constexpr std::string_view kAsphaltName = "asphalt";
constexpr std::string_view kDirtName = "dirt";

std::uint32_t ramp_colour(float height_m, float nz) {
    const float t_height = std::clamp((height_m - kValleyHeightM) / (kRidgeHeightM - kValleyHeightM), 0.0f, 1.0f);
    constexpr float kValleyR = 0.20f, kValleyG = 0.42f, kValleyB = 0.18f; // valley green
    constexpr float kRidgeR = 0.45f, kRidgeG = 0.34f, kRidgeB = 0.22f;    // ridge brown
    float r = kValleyR + (kRidgeR - kValleyR) * t_height;
    float g = kValleyG + (kRidgeG - kValleyG) * t_height;
    float b = kValleyB + (kRidgeB - kValleyB) * t_height;
    // 0 = flat (normal straight up), 1 = vertical face; rock from ~14 deg, fully rock by ~45 deg.
    const float slope = std::clamp(1.0f - nz, 0.0f, 1.0f);
    const float rock = std::clamp((slope - 0.25f) / 0.5f, 0.0f, 1.0f);
    constexpr float kRockR = 0.42f, kRockG = 0.40f, kRockB = 0.38f;
    r += (kRockR - r) * rock;
    g += (kRockG - g) * rock;
    b += (kRockB - b) * rock;
    return pack_rgba(r, g, b, 1.0f);
}

int terrain_tint_mode() {
    static const int mode = []() {
        const char* v = std::getenv("RG_TERRAIN_TINT");
        if (v == nullptr) return 0;
        const std::string s(v);
        if (s == "level") return 1;
        if (s == "carve") return 2;
        if (s == "both") return 3;
        return 0;
    }();
    return mode;
}

} // namespace

TerrainColourTable make_terrain_colour_table(const RoadSurfaceMap& surfaces) {
    TerrainColourTable table;
    for (const g2m::LandClass land_class : {g2m::LandClass::PavedRoad, g2m::LandClass::UnpavedRoad}) {
        const auto id = static_cast<std::uint8_t>(land_class);
        const std::string& name = surfaces.name_for(land_class);
        if (name == kAsphaltName) {
            table.rgba[id] = pack_rgba(kPavedR, kPavedG, kPavedB, 1.0f);
            table.set[id] = true;
        } else if (name == kDirtName) {
            table.rgba[id] = pack_rgba(kUnpavedR, kUnpavedG, kUnpavedB, 1.0f);
            table.set[id] = true;
        }
    }
    return table;
}

void chunk_vertex_colors(const g2m::mesh::TerrainChunkMesh& mesh, const TerrainColourTable& table,
                         std::vector<std::uint32_t>& rgba) {
    const std::size_t vertex_count = mesh.positions.size() / 3;
    const bool has_land_class = mesh.land_class.size() == vertex_count;
    rgba.assign(vertex_count, 0);
    for (std::size_t i = 0; i < vertex_count; ++i) {
        if (has_land_class) {
            const std::uint8_t id = static_cast<std::uint8_t>(mesh.land_class[i]);
            if (table.set[id]) {
                rgba[i] = table.rgba[id];
                continue;
            }
        }
        rgba[i] = ramp_colour(mesh.positions[3 * i + 2], mesh.normals[3 * i + 2]);
    }
}

void apply_debug_tint(RenderChunk& chunk) {
    const int mode = terrain_tint_mode();
    if (mode == 0) return;
    static const float kLevel[7][3] = {{1.0f, 0.1f, 0.1f}, {1.0f, 0.55f, 0.1f}, {1.0f, 0.95f, 0.1f}, {0.2f, 0.9f, 0.2f},
                                       {0.1f, 0.9f, 0.9f}, {0.2f, 0.3f, 1.0f}, {0.7f, 0.2f, 0.9f}};
    const float* lv = kLevel[std::min<int>(chunk.key.level, 6)];
    float cv[3] = {0.2f, 0.3f, 1.0f};
    if (chunk.carve == ChunkCarve::Carved) { cv[0] = 0.1f; cv[1] = 0.8f; cv[2] = 0.2f; }
    if (chunk.carve == ChunkCarve::Fallback) { cv[0] = 1.0f; cv[1] = 0.0f; cv[2] = 1.0f; }
    float c[3];
    for (int k = 0; k < 3; ++k) c[k] = mode == 1 ? lv[k] : mode == 2 ? cv[k] : 0.5f * (lv[k] + cv[k]);
    for (auto& px : chunk.rgba) {
        const float r = static_cast<float>((px >> 24) & 0xff) / 255.0f, g = static_cast<float>((px >> 16) & 0xff) / 255.0f,
                    b = static_cast<float>((px >> 8) & 0xff) / 255.0f;
        const float shade = 0.5f + 0.5f * (0.299f * r + 0.587f * g + 0.114f * b);
        px = pack_rgba(c[0] * shade, c[1] * shade, c[2] * shade, 1.0f);
    }
}

g2m::world::ChunkColourFn chunk_colour_hook(const RoadSurfaceMap& surfaces) {
    auto table = std::make_shared<const TerrainColourTable>(make_terrain_colour_table(surfaces));
    return [table](RenderChunk& chunk) {
        chunk_vertex_colors(chunk.mesh, *table, chunk.rgba);
        apply_debug_tint(chunk);
    };
}

} // namespace rg
