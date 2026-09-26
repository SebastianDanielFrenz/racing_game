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
#pragma once

#include "rg/terrain_render.h"
#include "rg/world_terrain.h"

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <cstdint>
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
    // build_static_view wall time alone, not upload).
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
    [[nodiscard]] bool is_fully_uploaded() const { return next_upload_index_ >= chunks_.size(); }
    [[nodiscard]] std::int64_t get_total_vertex_count() const;
    [[nodiscard]] double get_last_build_time_ms() const { return last_build_time_ms_; }
    // Wall time spent inside RenderingServer upload calls across every chunk
    // uploaded so far this preview (sum of per-chunk times, not wall-clock
    // elapsed across frames - PLAN.md R2.1's own "per-chunk upload
    // microseconds" measurement, accumulated).
    [[nodiscard]] double get_total_upload_time_ms() const { return total_upload_time_ms_; }

    void set_upload_budget_per_frame(std::int64_t chunks_per_frame);
    [[nodiscard]] std::int64_t get_upload_budget_per_frame() const { return upload_budget_per_frame_; }

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
    void upload_one_chunk(const rg::RenderChunk& chunk);
    [[nodiscard]] godot::Transform3D chunk_instance_transform(const rg::RenderChunk& chunk) const;

    std::unique_ptr<rg::WorldTerrain> terrain_;
    std::vector<rg::RenderChunk> chunks_;
    std::vector<godot::RID> mesh_rids_;
    std::vector<godot::RID> instance_rids_;
    std::size_t next_upload_index_ = 0;
    std::int64_t upload_budget_per_frame_ = 8;

    godot::Vector3 render_origin_session_{}; // raw (east, north, up), see set_render_origin's doc comment
    godot::RID material_rid_; // invalid until set_material() is called - see its doc comment
    double spawn_x_session_ = 0.0;
    double spawn_y_session_ = 0.0;
    godot::String last_error_;
    double last_build_time_ms_ = 0.0;
    double total_upload_time_ms_ = 0.0;
};

} // namespace rg_godot
