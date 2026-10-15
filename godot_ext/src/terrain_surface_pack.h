// godot_ext/src/terrain_surface_pack.h — the RenderingServer surface arrays of a terrain chunk
// (ARRAY_VERTEX/NORMAL/COLOR/INDEX), built off the main thread (C9, 2026-10-15).
//
// RgTerrainView used to fill these Packed arrays element by element on the main thread in upload_one_chunk (one
// GDExtension call per element). pack_chunk_surface fills them through ptrw() (a copy), and SurfacePackJob runs it on
// its own thread over a range of chunks while the main thread uploads the ones already packed - so the main thread only
// calls mesh_add_surface_from_arrays and creates the instance. The bytes are exactly what upload_one_chunk built.
#pragma once

#include "rg/terrain_render.h"

#include <godot_cpp/variant/array.hpp>

#include <atomic>
#include <cstddef>
#include <thread>
#include <vector>

namespace rg_godot {

// The surface arrays of `chunk` (Array of RenderingServer::ARRAY_MAX), or an EMPTY Array when the chunk has no
// triangles (coverage edge: vertices but no indices - such a chunk gets no surface, see upload_one_chunk).
// Triangle winding is flipped (g2m counter-clockwise -> Godot clockwise front faces). Thread-safe (pure).
godot::Array pack_chunk_surface(const rg::RenderChunk& chunk);

// Packs chunks [first, first + count) of a caller-owned array on one worker thread, in order. The caller must keep
// the chunks alive and unmoved until join (RgTerrainView: the diff's adds, appended to chunks_ after a reserve, are not
// touched until every one is uploaded, which needs every one packed).
class SurfacePackJob {
public:
    SurfacePackJob() = default;
    SurfacePackJob(const SurfacePackJob&) = delete;
    SurfacePackJob& operator=(const SurfacePackJob&) = delete;
    ~SurfacePackJob() { cancel_and_join(); }

    // Joins any previous job first.
    void start(const rg::RenderChunk* first, std::size_t count);
    // Chunks packed so far (results [0, packed()) are ready to take).
    [[nodiscard]] std::size_t packed() const { return done_.load(std::memory_order_acquire); }
    [[nodiscard]] std::size_t count() const { return results_.size(); }
    // Moves result i out (i < packed()).
    godot::Array take(std::size_t i) { return std::move(results_[i]); }
    // Stops after the chunk being packed and joins (the destructor too).
    void cancel_and_join();
    // The worker's total packing time of the last finished job (ms; stats).
    [[nodiscard]] double last_pack_ms() const { return pack_ms_.load(std::memory_order_acquire); }

private:
    std::thread thread_;
    std::vector<godot::Array> results_;
    std::atomic<std::size_t> done_{0};
    std::atomic<bool> cancel_{false};
    std::atomic<double> pack_ms_{0.0};
};

} // namespace rg_godot
