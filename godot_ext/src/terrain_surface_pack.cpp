#include "terrain_surface_pack.h"

#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <chrono>
#include <cstring>

namespace rg_godot {

static_assert(sizeof(godot::Vector3) == 3 * sizeof(float), "positions/normals are copied as float triples");

godot::Array pack_chunk_surface(const rg::RenderChunk& chunk) {
    const std::size_t vertex_count = chunk.mesh.positions.size() / 3;
    if (vertex_count == 0 || chunk.mesh.indices.empty()) return godot::Array();
    const auto n = static_cast<std::int64_t>(vertex_count);

    // Raw chunk-local floats, unchanged (Z-up, TerrainChunkMesh's own axes): the instance transform's basis rotates
    // every vertex and normal (chunk_instance_transform), so this is a plain copy.
    godot::PackedVector3Array positions;
    positions.resize(n);
    std::memcpy(positions.ptrw(), chunk.mesh.positions.data(), vertex_count * 3 * sizeof(float));
    godot::PackedVector3Array normals;
    normals.resize(n);
    std::memcpy(normals.ptrw(), chunk.mesh.normals.data(), vertex_count * 3 * sizeof(float));

    // rg::chunk_vertex_colors' pack layout: r << 24 | g << 16 | b << 8 | a, 8 bits each.
    godot::PackedColorArray colors;
    colors.resize(n);
    godot::Color* c = colors.ptrw();
    for (std::size_t i = 0; i < vertex_count; ++i) {
        const std::uint32_t p = chunk.rgba[i];
        c[i] = godot::Color(static_cast<float>((p >> 24) & 0xFFu) / 255.0f, static_cast<float>((p >> 16) & 0xFFu) / 255.0f,
                            static_cast<float>((p >> 8) & 0xFFu) / 255.0f, static_cast<float>(p & 0xFFu) / 255.0f);
    }

    // g2m::mesh emits counter-clockwise triangles (seen from above, +Z); Godot treats clockwise as front-facing, so the
    // last two indices of each triangle swap (the basis change is a proper rotation and keeps winding).
    const std::size_t index_count = chunk.mesh.indices.size();
    godot::PackedInt32Array indices;
    indices.resize(static_cast<std::int64_t>(index_count));
    std::int32_t* idx = indices.ptrw();
    for (std::size_t t = 0; t + 2 < index_count; t += 3) {
        idx[t] = static_cast<std::int32_t>(chunk.mesh.indices[t]);
        idx[t + 1] = static_cast<std::int32_t>(chunk.mesh.indices[t + 2]);
        idx[t + 2] = static_cast<std::int32_t>(chunk.mesh.indices[t + 1]);
    }

    godot::Array arrays;
    arrays.resize(godot::RenderingServer::ARRAY_MAX);
    arrays[godot::RenderingServer::ARRAY_VERTEX] = positions;
    arrays[godot::RenderingServer::ARRAY_NORMAL] = normals;
    arrays[godot::RenderingServer::ARRAY_COLOR] = colors;
    arrays[godot::RenderingServer::ARRAY_INDEX] = indices;
    return arrays;
}

void SurfacePackJob::start(const rg::RenderChunk* first, std::size_t count) {
    cancel_and_join();
    results_.clear();
    results_.resize(count);
    done_.store(0, std::memory_order_release);
    cancel_.store(false, std::memory_order_relaxed);
    if (count == 0) return;
    thread_ = std::thread([this, first, count]() {
        const auto t0 = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < count; ++i) {
            if (cancel_.load(std::memory_order_relaxed)) return;
            results_[i] = pack_chunk_surface(first[i]);
            done_.store(i + 1, std::memory_order_release);
        }
        pack_ms_.store(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(),
                       std::memory_order_release);
    });
}

void SurfacePackJob::cancel_and_join() {
    cancel_.store(true, std::memory_order_relaxed);
    if (thread_.joinable()) thread_.join();
}

} // namespace rg_godot
