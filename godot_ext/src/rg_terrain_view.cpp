#include "rg_terrain_view.h"

#include "frame_convert.h"

#include "ps/math/pose.h"
#include "ps/math/quat.h"
#include "ps/math/vec3.h"
#include "ps/types.h"

#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>

#include <algorithm>
#include <chrono>
#include <string>

using godot::D_METHOD;
using godot::String;

namespace rg_godot {

namespace {

std::string to_std_string(const String& s) { return std::string(s.utf8().get_data()); }

godot::Color unpack_rgba(std::uint32_t packed) {
    // rg::chunk_vertex_colors' own pack_rgba layout (core/src/world_terrain.cpp):
    // r << 24 | g << 16 | b << 8 | a, each an 8-bit channel.
    const float r = static_cast<float>((packed >> 24) & 0xFFu) / 255.0f;
    const float g = static_cast<float>((packed >> 16) & 0xFFu) / 255.0f;
    const float b = static_cast<float>((packed >> 8) & 0xFFu) / 255.0f;
    const float a = static_cast<float>(packed & 0xFFu) / 255.0f;
    return godot::Color(r, g, b, a);
}

} // namespace

RgTerrainView::~RgTerrainView() { free_all_uploaded(); }

bool RgTerrainView::initialize(const String& world_config_absolute_path) {
    free_all_uploaded();
    chunks_.clear();
    next_upload_index_ = 0;
    terrain_.reset();
    spawn_x_session_ = 0.0;
    spawn_y_session_ = 0.0;

    std::string err;
    auto config = rg::load_world_config(to_std_string(world_config_absolute_path), &err);
    if (!config.has_value()) {
        last_error_ = String(err.c_str());
        return false;
    }

    std::string open_err;
    terrain_ = rg::WorldTerrain::open(*config, &open_err);
    if (terrain_ == nullptr) {
        last_error_ = String(open_err.c_str());
        return false;
    }

    spawn_x_session_ = config->spawn.e - config->session_origin_utm.e0;
    spawn_y_session_ = config->spawn.n - config->session_origin_utm.n0;

    last_error_ = String();
    return true;
}

bool RgTerrainView::load_preview(float spawn_x, float spawn_y) {
    if (terrain_ == nullptr) {
        last_error_ = String("RgTerrainView::load_preview: initialize() was not called successfully first");
        return false;
    }

    free_all_uploaded();
    chunks_.clear();
    next_upload_index_ = 0;
    total_upload_time_ms_ = 0.0;

    const auto start = std::chrono::steady_clock::now();
    terrain_->build_static_view(static_cast<double>(spawn_x), static_cast<double>(spawn_y), chunks_);
    const auto end = std::chrono::steady_clock::now();
    last_build_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();

    mesh_rids_.reserve(chunks_.size());
    instance_rids_.reserve(chunks_.size());
    last_error_ = String();
    return true;
}

void RgTerrainView::set_render_origin(godot::Vector3 session_origin) {
    render_origin_session_ = session_origin;
    // Re-transform every ALREADY UPLOADED instance in place (a chunk not yet
    // uploaded picks up the new render_origin_session_ naturally when
    // upload_one_chunk eventually builds its transform - no separate
    // bookkeeping needed there).
    godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
    for (std::size_t i = 0; i < instance_rids_.size(); ++i) {
        rs->instance_set_transform(instance_rids_[i], chunk_instance_transform(chunks_[i]));
    }
}

void RgTerrainView::set_material(const godot::RID& material_rid) {
    material_rid_ = material_rid;
    // Re-tag every ALREADY UPLOADED mesh's surface 0 in place - a chunk not
    // yet uploaded picks up material_rid_ naturally when upload_one_chunk
    // eventually builds it (same "now and later" contract as
    // set_render_origin above).
    if (!material_rid_.is_valid()) {
        return;
    }
    godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
    for (const godot::RID& mesh_rid : mesh_rids_) {
        rs->mesh_surface_set_material(mesh_rid, 0, material_rid_);
    }
}

std::int64_t RgTerrainView::get_total_vertex_count() const {
    std::int64_t total = 0;
    for (const rg::RenderChunk& chunk : chunks_) {
        total += static_cast<std::int64_t>(chunk.mesh.positions.size() / 3);
    }
    return total;
}

void RgTerrainView::set_upload_budget_per_frame(std::int64_t chunks_per_frame) {
    upload_budget_per_frame_ = chunks_per_frame > 0 ? chunks_per_frame : 1;
}

godot::Transform3D RgTerrainView::chunk_instance_transform(const rg::RenderChunk& chunk) const {
    // Both sides of this subtraction are session-local (east, north, up)
    // triples reinterpreted, unchanged, as a ps::Vec3 (Z-up, matching
    // TerrainChunkMesh::origin's own documented axes exactly) - iso_to_godot_
    // transform then applies the SAME fixed Z-up -> Y-up permutation this
    // adapter already uses for the chassis/ground (rg_simulation.cpp), so a
    // future milestone rendering both terrain and a vehicle in the same
    // scene sees them in one consistent, mutually-correct rotated frame
    // (PLAN.md R2.1: "Instance transform = the Z-up to Y-up basis from
    // frame_convert.h (exact permutation)").
    const ps::Vec3 chunk_origin_session{chunk.origin_session[0], chunk.origin_session[1], chunk.origin_session[2]};
    const ps::Vec3 render_origin_session{static_cast<ps::real>(render_origin_session_.x),
                                         static_cast<ps::real>(render_origin_session_.y),
                                         static_cast<ps::real>(render_origin_session_.z)};
    return iso_to_godot_transform(ps::Pose{chunk_origin_session}, render_origin_session);
}

void RgTerrainView::upload_one_chunk(const rg::RenderChunk& chunk) {
    const auto start = std::chrono::steady_clock::now();

    const std::size_t vertex_count = chunk.mesh.positions.size() / 3;

    godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
    const godot::RID mesh_rid = rs->mesh_create();

    // A chunk at the imported region's coverage edge can have real vertices
    // but zero triangles (g2m::mesh::build_chunk leaves `indices` empty
    // where gather_window found nothing to triangulate - confirmed present
    // in the real home-r1 data: 24 of 488 chunks at max_distance_m=20000).
    // Assigning an EMPTY PackedInt32Array to ARRAY_INDEX still sets
    // Godot's own ARRAY_FORMAT_INDEX bit (the slot is non-NIL), but the
    // resulting index count of 0 then fails RenderingServer's own
    // mesh_create_surface_data_from_arrays validation ("index_array_len==
    // NO_INDEX_ARRAY") - so skip the surface entirely for such a chunk
    // rather than upload a surface with no triangles. The mesh/instance RID
    // pair is still created below either way - upload_one_chunk always
    // appends exactly one entry to mesh_rids_/instance_rids_ per call, and
    // set_render_origin()/chunk_instance_transform() index chunks_ and
    // instance_rids_ in lockstep by position, so skipping the RID pair here
    // would desync that 1:1 correspondence for every later chunk.
    if (vertex_count > 0 && !chunk.mesh.indices.empty()) {
        godot::PackedVector3Array positions;
        godot::PackedVector3Array normals;
        godot::PackedColorArray colors;
        positions.resize(static_cast<std::int64_t>(vertex_count));
        normals.resize(static_cast<std::int64_t>(vertex_count));
        colors.resize(static_cast<std::int64_t>(vertex_count));
        for (std::size_t i = 0; i < vertex_count; ++i) {
            // Raw chunk-local floats, UNCHANGED (Z-up, matching
            // TerrainChunkMesh's own documented local axes) - the instance
            // transform's basis (chunk_instance_transform, a fixed orthonormal
            // permutation) rotates every vertex AND every normal identically
            // when Godot renders this mesh, so no per-vertex basis-change work
            // is needed here (PLAN.md R2.1: "RgTerrainView only uploads arrays").
            positions[static_cast<std::int64_t>(i)] = godot::Vector3(
                chunk.mesh.positions[3 * i], chunk.mesh.positions[3 * i + 1], chunk.mesh.positions[3 * i + 2]);
            normals[static_cast<std::int64_t>(i)] = godot::Vector3(
                chunk.mesh.normals[3 * i], chunk.mesh.normals[3 * i + 1], chunk.mesh.normals[3 * i + 2]);
            colors[static_cast<std::int64_t>(i)] = unpack_rgba(chunk.rgba[i]);
        }

        godot::PackedInt32Array indices;
        indices.resize(static_cast<std::int64_t>(chunk.mesh.indices.size()));
        // g2m::mesh emits counter-clockwise triangles (seen from above, +Z);
        // Godot treats CLOCKWISE as front-facing, so swap each triangle's
        // last two indices or cull_back removes every top face (the preview
        // then shows only the sky's ground colour plus the backs of steep
        // slopes). The basis change itself is a proper rotation and keeps
        // winding, so this is purely Godot's front-face convention.
        for (std::size_t t = 0; t + 2 < chunk.mesh.indices.size(); t += 3) {
            indices[static_cast<std::int64_t>(t)] = static_cast<std::int32_t>(chunk.mesh.indices[t]);
            indices[static_cast<std::int64_t>(t + 1)] = static_cast<std::int32_t>(chunk.mesh.indices[t + 2]);
            indices[static_cast<std::int64_t>(t + 2)] = static_cast<std::int32_t>(chunk.mesh.indices[t + 1]);
        }

        godot::Array arrays;
        arrays.resize(godot::RenderingServer::ARRAY_MAX);
        arrays[godot::RenderingServer::ARRAY_VERTEX] = positions;
        arrays[godot::RenderingServer::ARRAY_NORMAL] = normals;
        arrays[godot::RenderingServer::ARRAY_COLOR] = colors;
        arrays[godot::RenderingServer::ARRAY_INDEX] = indices;

        rs->mesh_add_surface_from_arrays(mesh_rid, godot::RenderingServer::PRIMITIVE_TRIANGLES, arrays);
        if (material_rid_.is_valid()) {
            rs->mesh_surface_set_material(mesh_rid, 0, material_rid_);
        }
    }

    const godot::RID scenario = get_world_3d().is_valid() ? get_world_3d()->get_scenario() : godot::RID();
    const godot::RID instance_rid = rs->instance_create2(mesh_rid, scenario);
    rs->instance_set_transform(instance_rid, chunk_instance_transform(chunk));

    mesh_rids_.push_back(mesh_rid);
    instance_rids_.push_back(instance_rid);

    const auto end = std::chrono::steady_clock::now();
    total_upload_time_ms_ += std::chrono::duration<double, std::milli>(end - start).count();
}

void RgTerrainView::free_all_uploaded() {
    godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
    if (rs != nullptr) {
        for (const godot::RID& rid : instance_rids_) {
            if (rid.is_valid()) rs->free_rid(rid);
        }
        for (const godot::RID& rid : mesh_rids_) {
            if (rid.is_valid()) rs->free_rid(rid);
        }
    }
    instance_rids_.clear();
    mesh_rids_.clear();
}

void RgTerrainView::_process(double /*delta*/) {
    if (next_upload_index_ >= chunks_.size()) {
        return;
    }
    const std::size_t end_index = std::min(chunks_.size(), next_upload_index_ + static_cast<std::size_t>(upload_budget_per_frame_));
    for (; next_upload_index_ < end_index; ++next_upload_index_) {
        upload_one_chunk(chunks_[next_upload_index_]);
    }
}

void RgTerrainView::_bind_methods() {
    godot::ClassDB::bind_method(D_METHOD("initialize", "world_config_absolute_path"), &RgTerrainView::initialize);
    godot::ClassDB::bind_method(D_METHOD("get_spawn_x"), &RgTerrainView::get_spawn_x);
    godot::ClassDB::bind_method(D_METHOD("get_spawn_y"), &RgTerrainView::get_spawn_y);
    godot::ClassDB::bind_method(D_METHOD("load_preview", "spawn_x", "spawn_y"), &RgTerrainView::load_preview);
    godot::ClassDB::bind_method(D_METHOD("set_render_origin", "session_origin"), &RgTerrainView::set_render_origin);
    godot::ClassDB::bind_method(D_METHOD("set_material", "material_rid"), &RgTerrainView::set_material);
    godot::ClassDB::bind_method(D_METHOD("get_last_error"), &RgTerrainView::get_last_error);
    godot::ClassDB::bind_method(D_METHOD("get_chunk_count"), &RgTerrainView::get_chunk_count);
    godot::ClassDB::bind_method(D_METHOD("get_uploaded_chunk_count"), &RgTerrainView::get_uploaded_chunk_count);
    godot::ClassDB::bind_method(D_METHOD("is_fully_uploaded"), &RgTerrainView::is_fully_uploaded);
    godot::ClassDB::bind_method(D_METHOD("get_total_vertex_count"), &RgTerrainView::get_total_vertex_count);
    godot::ClassDB::bind_method(D_METHOD("get_last_build_time_ms"), &RgTerrainView::get_last_build_time_ms);
    godot::ClassDB::bind_method(D_METHOD("get_total_upload_time_ms"), &RgTerrainView::get_total_upload_time_ms);
    godot::ClassDB::bind_method(D_METHOD("set_upload_budget_per_frame", "chunks_per_frame"),
                                &RgTerrainView::set_upload_budget_per_frame);
    godot::ClassDB::bind_method(D_METHOD("get_upload_budget_per_frame"), &RgTerrainView::get_upload_budget_per_frame);
}

} // namespace rg_godot
