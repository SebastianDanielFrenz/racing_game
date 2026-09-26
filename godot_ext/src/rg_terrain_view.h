// godot_ext/src/rg_terrain_view.h — RgTerrainView: the Godot-facing seam for
// rg::WorldTerrain (PLAN.md R2.1). Mirrors RgSimulation's own shape (a Node
// exposing engine-neutral state to GDScript, all rg::/g2m:: types confined
// to this .h/.cpp pair) but for STATIC preview terrain rather than a
// running physics sim: load_preview() opens a WorldTerrain from a
// world_config.json path, runs one LOD chunk selection
// (rg::WorldTerrain::build_static_view) and queues every resulting
// rg::RenderChunk for upload; _process() drains that queue a few chunks per
// frame via the low-level RenderingServer (mesh_create/
// mesh_add_surface_from_arrays/instance_create2/instance_set_transform) so
// a large chunk set (hundreds of meshes, PLAN.md R2.1's own LOD-distance
// measurement) never stalls a single frame.
//
// Engine-neutral rule (PLAN.md 11.1, repo CLAUDE.md): every RenderChunk's
// mesh/LOD-selection LOGIC lives in rg_core (rg::WorldTerrain,
// build_static_view_from_lookup) - this class only reads the already-built
// RenderChunk arrays and hands them to Godot's RenderingServer. No Jolt/g2m
// decode/LOD-selection code runs anywhere in this file.
//
// RID lifetime: every RID this class creates (one mesh + one instance per
// chunk) is tracked in mesh_rids_/instance_rids_ and freed in
// free_all_uploaded() - called from both the destructor and the start of
// load_preview() (a second load_preview() call replaces the previous
// preview instead of leaking it), so a full editor-quit/headless-exit run
// leaves zero RenderingServer RIDs alive (PLAN.md R2.1 smoke-test
// acceptance: "0 RID leaks").
//
// Streaming (R2.2 R8): load_preview() also creates an rg::TerrainViewStreamer
// (Godot-free, rg_core) seeded with the preview's chunk set. update_focus()
// (called by GDScript every frame with the car's / camera's session-local XY)
// forwards to it; apply_diff() (run from _process) takes a ready diff,
// appends its adds to the upload queue, uploads within upload_budget_ms per
// frame (steady clock), and only once the LAST add is uploaded starts freeing
// the removed chunks' RIDs - under the same per-frame budget (a diff's
// ~100-200 frees cost 4-9 ms in one frame otherwise, measured) - and commits
// the diff back to the streamer after the last one - the streamer header's
// ordering contract, so a chunk swap never opens a hole (overlap only).
// chunks_/mesh_rids_/instance_rids_ stay index-aligned: adds are appended
// (uploaded prefix == RID arrays), removals swap-remove all three at once
// (only ever with the whole queue uploaded and no new diff taken until the
// last removal), index_of_ maps key -> index.
#pragma once

#include "rg/terrain_render.h"
#include "rg/terrain_view_streamer.h"
#include "rg/world_terrain.h"

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

namespace rg_godot {

class RgTerrainView : public godot::Node3D {
    GDCLASS(RgTerrainView, godot::Node3D)

public:
    RgTerrainView() = default;
    ~RgTerrainView() override;

    // Opens rg::WorldTerrain from a world_config.json ABSOLUTE path (the
    // GDScript caller resolves res:// -> filesystem path itself, same
    // "no Godot resource-path convention leaks into rg_core" rule as
    // RgSimulation::initialize's own vehicle/surface JSON paths). Returns
    // false and sets get_last_error() on failure (a malformed config, a
    // store that fails to open) instead of letting rg::WorldTerrain::open's
    // std::string* err cross the GDExtension boundary as anything but a
    // godot::String.
    bool initialize(const godot::String& world_config_absolute_path);

    // The world_config.json's own spawn point, expressed session-local
    // (WorldConfig::Spawn::e/n minus WorldConfig::UtmOrigin::e0/n0 -
    // exactly load_preview()'s own (spawn_x, spawn_y) convention) - cached
    // at initialize() time so GDScript callers (main.gd's --terrain-preview
    // branch) don't have to re-parse world_config.json themselves just to
    // find a reasonable load_preview()/set_render_origin() point. 0.0 before
    // a successful initialize().
    [[nodiscard]] double get_spawn_x() const { return spawn_x_session_; }
    [[nodiscard]] double get_spawn_y() const { return spawn_y_session_; }

    // Selects the LOD chunk set around (spawn_x, spawn_y) - SESSION-LOCAL
    // metres (x east of e0, y north of n0), i.e. WorldConfig::Spawn::e/n
    // minus WorldConfig::UtmOrigin::e0/n0 - via
    // rg::WorldTerrain::build_static_view, and queues every chunk for
    // per-frame upload (_process). Frees any previously uploaded chunks
    // first (a second call replaces the preview, never leaks the old one).
    // Returns false + sets get_last_error() if initialize() was not called
    // successfully first. Records get_last_build_time_ms() (the
    // build_static_view wall time alone, not upload). Since R8 the selection
    // runs through a fresh rg::TerrainViewStreamer's build_initial (the same
    // chunk set and order), which then streams from update_focus() on.
    bool load_preview(float spawn_x, float spawn_y);

    // The session-local (E - E0, N - N0, U) point every queued/uploaded
    // chunk's instance transform is currently expressed relative to (PLAN.md
    // 11.3 floating origin / PLAN.md R2.1: "render_origin = spawn snapped to
    // integer metres"). `session_origin`'s x/y/z fields hold the RAW
    // session-frame components (east, north, up) - NOT yet permuted into
    // Godot's own x/y/z axes; the permutation happens once, uniformly, in
    // chunk_instance_transform() below (see .cpp), the same fixed Z-up ->
    // Y-up basis frame_convert.h's iso_to_godot_transform already applies
    // for the chassis/ground in rg_simulation.cpp. Re-transforms every
    // already-uploaded instance immediately (so calling this after
    // load_preview() still moves the whole preview consistently); does NOT
    // rebuild meshes or re-run LOD selection.
    void set_render_origin(godot::Vector3 session_origin);

    [[nodiscard]] godot::String get_last_error() const { return last_error_; }
    [[nodiscard]] std::int64_t get_chunk_count() const { return static_cast<std::int64_t>(chunks_.size()); }
    [[nodiscard]] std::int64_t get_uploaded_chunk_count() const { return static_cast<std::int64_t>(next_upload_index_); }
    // Every queued chunk is uploaded AND no streaming diff is half-applied.
    [[nodiscard]] bool is_fully_uploaded() const { return next_upload_index_ >= chunks_.size() && !diff_in_progress_; }
    [[nodiscard]] std::int64_t get_total_vertex_count() const;
    [[nodiscard]] double get_last_build_time_ms() const { return last_build_time_ms_; }
    // Wall time spent inside RenderingServer upload calls across every chunk
    // uploaded so far this preview (sum of per-chunk times, not wall-clock
    // elapsed across frames - PLAN.md R2.1's own "per-chunk upload
    // microseconds" measurement, accumulated).
    [[nodiscard]] double get_total_upload_time_ms() const { return total_upload_time_ms_; }

    // Count budget for the INITIAL preview load only (a loading phase: 8
    // chunks/frame drains the ~488-chunk preview in ~60 frames).
    void set_upload_budget_per_frame(std::int64_t chunks_per_frame);
    [[nodiscard]] std::int64_t get_upload_budget_per_frame() const { return upload_budget_per_frame_; }
    // Time budget for streamed diffs (R8): per frame, uploads and then
    // removals continue while less than this many milliseconds (steady
    // clock) have been spent in this frame's apply_diff - always at least one
    // upload or removal, so a diff always makes progress. Default 0.8 ms.
    void set_upload_budget_ms(double budget_ms);
    [[nodiscard]] double get_upload_budget_ms() const { return upload_budget_ms_; }

    // R8: the render LOD focus (session-local metres - the car, or the fly
    // camera via godot_to_session). Call every frame; cheap and non-blocking
    // (rg::TerrainViewStreamer::update_focus). No-op before load_preview().
    void update_focus(double session_x, double session_y);
    // A Godot-space position (this node's parent frame = render frame) back
    // to raw session-local (east, north, up) - the inverse of
    // chunk_instance_transform's mapping, using the current render origin.
    [[nodiscard]] godot::Vector3 godot_to_session(godot::Vector3 godot_pos) const;
    // No diff in flight (streamer Idle) and nothing queued for upload.
    [[nodiscard]] bool is_stream_idle() const;
    // {pending_adds, pending_removals, last_diff_added, last_diff_removed,
    //  upload_ms_this_frame (all of this frame's apply_diff), last_remove_ms
    //  (the current/last diff's RID frees summed over frames), diffs_applied,
    //  selections_started, last_build_ms, streamer_phase (0 Idle, 1 Building,
    //  2 Ready, 3 Applying), resident_chunks, diff_errors}
    [[nodiscard]] godot::Dictionary get_stream_stats() const;

    // The material RenderingServer attaches to every uploaded chunk's mesh
    // surface (mesh_surface_set_material) - GDScript builds this from
    // game/shaders/terrain.gdshader (a ShaderMaterial's RID via
    // Material.get_rid()) and passes it in before/after load_preview(); an
    // already-uploaded chunk is re-tagged immediately (not just chunks
    // uploaded afterward), same "apply to everything now, and to whatever
    // comes later" contract as set_render_origin. Godot's default material
    // does NOT read the mesh's own vertex COLOR array as albedo, so without
    // this call the hypsometric per-vertex colours upload_one_chunk sets
    // would never actually be visible - terrain.gdshader's `ALBEDO =
    // COLOR.rgb` is what makes them show up.
    void set_material(const godot::RID& material_rid);

    void _process(double delta) override;

protected:
    static void _bind_methods();

private:
    void free_all_uploaded();
    void reset_chunks();
    // One frame of streaming (see the file comment): take a ready diff, upload
    // within budget, then (all adds in) free removals within budget, then
    // (all removals done) commit.
    void apply_diff(std::chrono::steady_clock::time_point frame_start);
    void remove_chunk_at(std::size_t index);
    void upload_one_chunk(const rg::RenderChunk& chunk);
    [[nodiscard]] godot::Transform3D chunk_instance_transform(const rg::RenderChunk& chunk) const;

    std::unique_ptr<rg::WorldTerrain> terrain_;
    // Declared AFTER terrain_ so it is destroyed (worker cancelled + joined)
    // BEFORE the WorldTerrain its TerrainViewSource points into.
    std::unique_ptr<rg::TerrainViewStreamer> streamer_;
    std::vector<rg::RenderChunk> chunks_;
    std::vector<godot::RID> mesh_rids_;      // [i] <-> chunks_[i], for i < next_upload_index_
    std::vector<godot::RID> instance_rids_;  // same
    std::map<g2m::mesh::ChunkKey, std::size_t> index_of_; // key -> index into chunks_ (lookup only)
    std::size_t next_upload_index_ = 0;
    std::int64_t upload_budget_per_frame_ = 8;
    double upload_budget_ms_ = 0.8;
    bool bulk_loading_ = false; // the initial preview set is still uploading (count budget)

    bool diff_in_progress_ = false;
    std::uint64_t diff_serial_ = 0;
    std::vector<g2m::mesh::ChunkKey> diff_removed_;
    std::size_t diff_remove_next_ = 0; // cursor into diff_removed_
    std::int64_t last_diff_added_ = 0;
    std::int64_t last_diff_removed_ = 0;
    std::int64_t diffs_applied_ = 0;
    std::int64_t diff_errors_ = 0; // a removed key that was not resident (never expected)
    double upload_ms_this_frame_ = 0.0;
    double last_remove_ms_ = 0.0; // the current/last diff's RID frees, summed over frames

    godot::Vector3 render_origin_session_{}; // raw (east, north, up), see set_render_origin's doc comment
    godot::RID material_rid_; // invalid until set_material() is called - see its doc comment
    double spawn_x_session_ = 0.0;
    double spawn_y_session_ = 0.0;
    godot::String last_error_;
    double last_build_time_ms_ = 0.0;
    double total_upload_time_ms_ = 0.0;
};

} // namespace rg_godot
