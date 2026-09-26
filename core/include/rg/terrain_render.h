// rg/terrain_render.h — RenderChunk: one built terrain::mesh chunk plus the
// session-local origin and per-vertex colour a renderer needs to upload it
// (PLAN.md R2.1). Engine-neutral (no Godot type) - godot_ext's RgTerrainView
// is the only place these cross into godot:: RenderingServer calls.
#pragma once

#include "g2m/mesh/terrain_chunk.h"

#include <cstdint>
#include <vector>

namespace rg {

// One chunk ready to upload. `mesh` is geo2map_engine's own built chunk
// (positions/normals/land_class/indices, Z-up, x east / y north, local to
// mesh.origin - an ABSOLUTE UTM point). `origin_session = mesh.origin -
// (E0, N0, 0)` (the session frame's own origin, PLAN.md R2.0/4.2) - the
// local-space origin every vertex position is relative to, still in
// session-local metres (double precision; a renderer's floating-origin
// rebase, godot_ext/src/rg_terrain_view.cpp's set_render_origin, subtracts
// its own render origin from this AFTER this struct is built, per PLAN.md
// R2.1's own spec: "origin = float(origin_session - render_origin)").
//
// `rgba` is one packed RGBA8 colour per mesh vertex (mesh.positions.size()/3
// entries, same order/count as mesh.positions/mesh.normals): byte layout
// (r << 24) | (g << 16) | (b << 8) | a, each channel 0..255. Before G2.5
// lands terrain.class (every LandClass is Unknown), so build_static_view
// shades by height/slope (a hypsometric ramp) instead of a LandClass
// palette - see world_terrain.cpp's chunk_vertex_color().
//
// `key` (R2.2 plan section 3, R8) is the chunk's own g2m ChunkKey (zone,
// level, cx, cy) - the identity rg::TerrainViewStreamer diffs a new LOD
// selection against the resident set by. Always equal to mesh.key (both are
// set from the same select_chunks key by build_one_chunk); it is a separate
// field so a renderer tracks chunks by a RenderChunk-level identity and never
// has to know that TerrainChunkMesh happens to carry one too.
struct RenderChunk {
    g2m::mesh::ChunkKey key{};
    g2m::mesh::TerrainChunkMesh mesh;
    double origin_session[3] = {0.0, 0.0, 0.0};
    std::vector<std::uint32_t> rgba;
};

} // namespace rg
