#include "rg_terrain_view.h"

#include "frame_convert.h"
#include "rg_simulation.h"

#include "ps/math/pose.h"
#include "ps/math/quat.h"
#include "ps/math/vec3.h"
#include "ps/types.h"

#include "g2m/world/height_sampler.h"
#include "g2m/world/building_mesh.h"
#include "g2m/world/road_paint.h"
#include "rg/flat_tangent.h"

#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using godot::D_METHOD;
using godot::String;

namespace rg_godot {

namespace {

std::string to_std_string(const String& s) { return std::string(s.utf8().get_data()); }

// Streamer options. initial_build_threads: the cold initial build is IO/derive bound (every chunk blocks on
// gather_window), so it is not capped at 8 like the geo2map default; RG_VIEW_INITIAL_THREADS overrides for A/B runs.
rg::TerrainViewStreamer::Options view_streamer_options() {
    rg::TerrainViewStreamer::Options o{};
    const char* env = std::getenv("RG_VIEW_INITIAL_THREADS");
    const long v = env != nullptr ? std::strtol(env, nullptr, 10) : 0;
    if (v > 0) o.initial_build_threads = static_cast<unsigned>(v);
    return o;
}

} // namespace

RgTerrainView::~RgTerrainView() {
    join_preview();
    reset_packing(); // pack_job_ is declared before chunks_, which its worker reads
    cancel_heal_rebuild();
    streamer_.reset(); // cancel + join the worker before anything else goes
    free_all_uploaded();
}

void RgTerrainView::reset_chunks() {
    reset_packing(); // the worker reads chunks_
    free_all_uploaded();
    chunks_.clear();
    index_of_.clear();
    next_upload_index_ = 0;
    bulk_loading_ = false;
    diff_in_progress_ = false;
    diff_serial_ = 0;
    diff_removed_.clear();
    diff_replaced_.clear();
    diff_remove_next_ = 0;
    last_diff_added_ = 0;
    last_diff_removed_ = 0;
    last_remove_ms_ = 0.0;
    diffs_applied_ = 0;
    diff_errors_ = 0;
}

void RgTerrainView::join_preview() {
    // build_initial cannot be cancelled: wait for it (it reads terrain_ and owns streamer_'s build).
    if (preview_future_.valid()) preview_future_.wait();
    preview_future_ = std::future<void>();
    preview_chunks_.clear();
    preview_state_ = PreviewState::Idle;
}

void RgTerrainView::teardown_terrain() {
    join_preview();
    cancel_heal_rebuild();
    if (heal_future_.valid()) heal_future_.wait(); // the re-carve thread reads terrain_
    streamer_.reset(); // before terrain_: its TerrainViewSource points into it
    reset_chunks();
    terrain_.reset();
    synthetic_.reset();
    spawn_x_session_ = 0.0;
    spawn_y_session_ = 0.0;
}

bool RgTerrainView::initialize(const String& world_config_absolute_path) {
    teardown_terrain();

    std::string err;
    auto config = rg::load_world_config(to_std_string(world_config_absolute_path), &err);
    if (!config.has_value()) {
        last_error_ = String(err.c_str());
        return false;
    }

    std::string open_err;
    terrain_ = rg::WorldTerrain::open(*config, &open_err); // unique_ptr -> shared_ptr (refcount 1)
    if (terrain_ == nullptr) {
        last_error_ = String(open_err.c_str());
        return false;
    }

    spawn_x_session_ = config->spawn.e - config->session_origin_utm.e0;
    spawn_y_session_ = config->spawn.n - config->session_origin_utm.n0;

    last_error_ = String();
    return true;
}

bool RgTerrainView::initialize_shared(RgSimulation* sim) {
    teardown_terrain();

    if (sim == nullptr) {
        last_error_ = String("RgTerrainView::initialize_shared: null RgSimulation");
        return false;
    }
    std::shared_ptr<rg::WorldTerrain> shared = sim->shared_world_terrain();
    if (shared == nullptr) {
        last_error_ = String("RgTerrainView::initialize_shared: RgSimulation has no terrain-mode Session");
        return false;
    }
    terrain_ = std::move(shared); // shares ownership with the Session's TerrainModeConfig - no second decode

    last_error_ = String();
    return true;
}

void RgTerrainView::release() {
    teardown_terrain();
    last_error_ = String();
}

bool RgTerrainView::load_preview(float spawn_x, float spawn_y) {
    if (terrain_ == nullptr) {
        last_error_ = String("RgTerrainView::load_preview: initialize() was not called successfully first");
        return false;
    }

    join_preview();

    cancel_heal_rebuild();
    streamer_.reset();
    reset_chunks();
    total_upload_time_ms_ = 0.0;

    apply_view_lod_to_terrain();
    // R8: the preview's own chunk set is the streamer's initial selection
    // (TerrainViewStreamer::build_initial == build_static_view for the same
    // point, same chunk order), so update_focus() can stream from here on.
    streamer_ = std::make_unique<rg::TerrainViewStreamer>(terrain_->view_source(),
                                                          view_streamer_options());
    const auto start = std::chrono::steady_clock::now();
    streamer_->build_initial(static_cast<double>(spawn_x), static_cast<double>(spawn_y), chunks_);
    const auto end = std::chrono::steady_clock::now();
    last_build_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();
    for (std::size_t i = 0; i < chunks_.size(); ++i) {
        index_of_[chunks_[i].key] = i;
    }
    bulk_loading_ = true;
    start_packing(0);

    mesh_rids_.reserve(chunks_.size());
    instance_rids_.reserve(chunks_.size());
    last_error_ = String();
    return true;
}

bool RgTerrainView::begin_preview(float spawn_x, float spawn_y) {
    if (terrain_ == nullptr) {
        last_error_ = String("RgTerrainView::begin_preview: initialize() was not called successfully first");
        return false;
    }
    join_preview();
    cancel_heal_rebuild();
    streamer_.reset();
    reset_chunks();
    total_upload_time_ms_ = 0.0;

    apply_view_lod_to_terrain();
    preview_progress_ = std::make_shared<rg::ViewBuildProgress>();
    rg::TerrainViewSource source = terrain_->view_source();
    rg::announce_view_build_total(source, static_cast<double>(spawn_x), static_cast<double>(spawn_y), *preview_progress_);
    rg::attach_view_build_progress(source, preview_progress_);
    streamer_ = std::make_unique<rg::TerrainViewStreamer>(std::move(source), view_streamer_options());
    preview_started_ = std::chrono::steady_clock::now();
    preview_state_ = PreviewState::Building;
    rg::TerrainViewStreamer* streamer = streamer_.get();
    const double x = static_cast<double>(spawn_x), y = static_cast<double>(spawn_y);
    preview_future_ = std::async(std::launch::async, [this, streamer, x, y]() {
        streamer->build_initial(x, y, preview_chunks_);
    });
    last_error_ = String();
    return true;
}

godot::Dictionary RgTerrainView::poll_preview() {
    godot::Dictionary d;
    if (preview_state_ == PreviewState::Building && preview_future_.valid() &&
        preview_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try {
            preview_future_.get();
            last_build_time_ms_ =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - preview_started_).count();
            chunks_ = std::move(preview_chunks_);
            preview_chunks_.clear();
            for (std::size_t i = 0; i < chunks_.size(); ++i) index_of_[chunks_[i].key] = i;
            bulk_loading_ = true;
            start_packing(0);
            mesh_rids_.reserve(chunks_.size());
            instance_rids_.reserve(chunks_.size());
            preview_state_ = PreviewState::Ready;
        } catch (const std::exception& e) {
            last_error_ = String(e.what());
            preview_state_ = PreviewState::Failed;
        }
        preview_future_ = std::future<void>();
    }
    const char* name = "idle";
    switch (preview_state_) {
        case PreviewState::Building: name = "building"; break;
        case PreviewState::Ready: name = "ready"; break;
        case PreviewState::Failed: name = "error"; break;
        case PreviewState::Idle: break;
    }
    d["state"] = String(name);
    d["done"] = preview_progress_ ? static_cast<int64_t>(preview_progress_->done.load(std::memory_order_relaxed)) : 0;
    d["total"] = preview_progress_ ? static_cast<int64_t>(preview_progress_->total.load(std::memory_order_relaxed)) : 0;
    d["message"] = last_error_;
    return d;
}

bool RgTerrainView::load_synthetic_preview(double spawn_x, double spawn_y, double max_distance_m) {
    teardown_terrain();
    total_upload_time_ms_ = 0.0;
    synthetic_ = std::make_unique<rg::SyntheticTerrain>(464000.0, 5559000.0);
    g2m::mesh::LodParams params;
    if (max_distance_m > 0.0) params.max_distance_m = max_distance_m;
    streamer_ = std::make_unique<rg::TerrainViewStreamer>(synthetic_->view_source(params), view_streamer_options());
    const auto start = std::chrono::steady_clock::now();
    streamer_->build_initial(spawn_x, spawn_y, chunks_);
    last_build_time_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    for (std::size_t i = 0; i < chunks_.size(); ++i) index_of_[chunks_[i].key] = i;
    bulk_loading_ = true;
    start_packing(0);
    last_error_ = String();
    return true;
}

godot::Dictionary RgTerrainView::get_road_visual_tile(int x,int y,double lift,double line_width_m) {
    godot::Dictionary out;if(!terrain_)return out;
    const g2m::TileKey key{terrain_->frame().zone(),2,x,y};const auto shared=terrain_->road_visual_tile_shared(key,0.0);
    if(!shared)return out;
    // Render-only explicit paint; do not enlarge the physics/cache route mesh.
    g2m::world::RoadPaintOptions paint_options;paint_options.line_width_m=line_width_m;
    const auto paint=g2m::world::build_road_paint(*shared,paint_options);
    const auto& mesh=paint.mesh;
    out["native_paint_complete"]=paint.complete;out["paint_triangles"]=static_cast<int64_t>(paint.paint_triangles);
    godot::PackedVector3Array points,normals;godot::PackedVector2Array uv,uv2;godot::PackedColorArray colours;godot::PackedInt32Array indices;
    for(const auto& v:mesh.vertices){points.append({-v.y,v.z+static_cast<float>(lift),-v.x});normals.append({0,1,0});uv.append({v.u,v.v});uv2.append({v.width,v.fade});const int flags=static_cast<int>(v.flags)|(v.verge>.5f?256:0);colours.append({v.lanes/32,v.forward/32,static_cast<float>(flags&255)/255,static_cast<float>(flags>>8)/255});}
    for(auto i:mesh.indices)indices.append(i);
    out["vertices"]=points;out["normals"]=normals;out["uv"]=uv;out["uv2"]=uv2;out["colours"]=colours;out["indices"]=indices;
    out["origin_session"]=godot::Vector3(static_cast<double>(x)*1024-terrain_->frame().e0_m(),static_cast<double>(y)*1024-terrain_->frame().n0_m(),0);return out;
}

godot::Dictionary RgTerrainView::get_rail_info() const {
    godot::Dictionary out;
    const auto config=terrain_?terrain_->rail_config():nullptr;
    out["enabled"]=terrain_&&terrain_->has_rail_layer()&&config;
    out["view_distance_m"]=config?config->view_distance_m:0.0;
    return out;
}

godot::Dictionary RgTerrainView::get_rail_tile(int x,int y) {
    godot::Dictionary out;if(!terrain_||!terrain_->has_rail_layer())return out;
    const auto config=terrain_->rail_config();if(!config)return out;
    const g2m::TileKey key{terrain_->frame().zone(),2,x,y};
    const auto tile=terrain_->rail_tile(key,g2m::world::Priority::Render);
    if(!tile||tile->pieces.empty())return out;
    const rg::RailTileMesh built=rg::build_rail_tile_mesh(*tile,*config);
    const auto& mesh=built.mesh;if(mesh.empty())return out;
    // (east, north, up) -> Godot (-north, up, -east) is a proper rotation, so the builder's counter-clockwise triangles
    // stay counter-clockwise; Godot's front faces are clockwise, so the indices are reversed here.
    godot::PackedVector3Array points,normals;godot::PackedColorArray colours;godot::PackedInt32Array indices;
    const std::size_t count=mesh.vertex_count();points.resize(static_cast<int64_t>(count));normals.resize(static_cast<int64_t>(count));colours.resize(static_cast<int64_t>(count));
    for(std::size_t i=0;i<count;++i){
        points.set(static_cast<int64_t>(i),{-mesh.positions[3*i+1],mesh.positions[3*i+2],-mesh.positions[3*i]});
        normals.set(static_cast<int64_t>(i),{-mesh.normals[3*i+1],mesh.normals[3*i+2],-mesh.normals[3*i]});
        colours.set(static_cast<int64_t>(i),godot::Color(mesh.colours[3*i],mesh.colours[3*i+1],mesh.colours[3*i+2]));
    }
    indices.resize(static_cast<int64_t>(mesh.indices.size()));
    for(std::size_t t=0;t+2<mesh.indices.size();t+=3){
        indices.set(static_cast<int64_t>(t),static_cast<int32_t>(mesh.indices[t]));
        indices.set(static_cast<int64_t>(t+1),static_cast<int32_t>(mesh.indices[t+2]));
        indices.set(static_cast<int64_t>(t+2),static_cast<int32_t>(mesh.indices[t+1]));
    }
    out["vertices"]=points;out["normals"]=normals;out["colours"]=colours;out["indices"]=indices;
    out["origin_session"]=godot::Vector3(static_cast<double>(x)*1024-terrain_->frame().e0_m(),static_cast<double>(y)*1024-terrain_->frame().n0_m(),0);
    return out;
}

godot::Dictionary RgTerrainView::get_building_tile(int x,int y,double min_height,double fallback,double storey) {
    godot::Dictionary output;godot::Array buildings;if(!terrain_)return output;
    const g2m::TileKey key{terrain_->frame().zone(),2,x,y};
    for(const auto& b:terrain_->buildings_tile(key,min_height,fallback,storey)) {
        godot::Dictionary item;item["id"]=b.id;item["relation"]=b.relation;item["height"]=b.height;item["base"]=b.base;item["bottom"]=b.bottom;
        item["type"]=godot::String(b.type.c_str());item["min_height"]=b.min_height;
        item["height_source"]=godot::String(b.height_source.c_str());item["colour"]=godot::String(b.colour.c_str());
        item["roof_shape"]=godot::String(b.roof_shape.c_str());item["roof_direction"]=godot::String(b.roof_direction.c_str());item["roof_orientation"]=godot::String(b.roof_orientation.c_str());
        godot::Array outer,inner;
        const auto rings=[&](const auto& source,godot::Array& target){for(const auto& ring:source){godot::PackedVector2Array points;for(auto p:ring)points.append({static_cast<float>(p.e-x*1024),static_cast<float>(p.n-y*1024)});target.append(points);}};
        rings(b.outers,outer);rings(b.inners,inner);item["outers"]=outer;item["inners"]=inner;buildings.append(item);
    }
    output["buildings"]=buildings;output["origin_session"]=godot::Vector3(static_cast<double>(x)*1024-terrain_->frame().e0_m(),static_cast<double>(y)*1024-terrain_->frame().n0_m(),0);
    return output;
}

godot::Dictionary RgTerrainView::get_building_mesh_tile(int x,int y,double min_height,double fallback,double storey,double skyline) {
    godot::Dictionary out;if(!terrain_)return out;
    const g2m::TileKey key{terrain_->frame().zone(),2,x,y};
    godot::PackedVector3Array points[2][8],normals[2][8];godot::PackedColorArray colours[2][8];godot::PackedVector2Array uv[2][8];std::vector<float> tangents[2][8];
    std::int64_t unsupported=0,window_count=0,building_count=0,doors=0,chimneys=0,pitched=0,directed=0,directed_supported=0;
    g2m::world::BuildingMeshOptions options;options.origin_e=static_cast<double>(x)*1024;options.origin_n=static_cast<double>(y)*1024;options.storey_height_m=storey;
    for(const auto& b:terrain_->buildings_tile(key,min_height,fallback,storey)) {
        const int tier=b.height>=skyline?1:0;options.facade_detail=tier==0;
        const auto mesh=g2m::world::build_building_mesh(b,options);unsupported+=mesh.unsupported_roof;window_count+=mesh.windows;doors+=mesh.doors;chimneys+=mesh.chimneys;pitched+=mesh.roof!=g2m::world::RoofForm::Flat;++building_count;
        if(!b.roof_direction.empty()){++directed;if(!mesh.unsupported_roof&&mesh.roof!=g2m::world::RoofForm::Flat)++directed_supported;}
        // Native CCW -> Godot clockwise; E/N/up -> -N/up/-E.
        const auto to_godot=[](const std::array<float,3>& p){return std::array<float,3>{-p[1],p[2],-p[0]};};
        for(std::size_t i=0;i+2<mesh.vertices.size();i+=3){
            const auto& a=mesh.vertices[i];const auto& b=mesh.vertices[i+2];const auto& c=mesh.vertices[i+1];
            const rg::UvGradient gradient=rg::triangle_uv_gradient(to_godot(a.position),to_godot(b.position),to_godot(c.position),a.uv,b.uv,c.uv);
            for(const auto* vertex:{&a,&b,&c}){
                const auto& v=*vertex;const auto material=static_cast<unsigned>(v.material);
                const auto p=to_godot(v.position);const auto n=to_godot(v.normal);
                points[tier][material].append({p[0],p[1],p[2]});
                normals[tier][material].append({n[0],n[1],n[2]});colours[tier][material].append({v.colour[0],v.colour[1],v.colour[2]});
                uv[tier][material].append({v.uv[0],v.uv[1]});
                const rg::VertexTangent tangent=rg::vertex_tangent(gradient,n);
                tangents[tier][material].insert(tangents[tier][material].end(),{tangent.tangent[0],tangent.tangent[1],tangent.tangent[2],tangent.sign});
            }
        }
    }
    for(int tier=0;tier<2;++tier){godot::Array surfaces;
        for(int material=0;material<8;++material){if(points[tier][material].is_empty())continue;
            godot::PackedFloat32Array packed_tangents;packed_tangents.resize(static_cast<std::int64_t>(tangents[tier][material].size()));
            std::memcpy(packed_tangents.ptrw(),tangents[tier][material].data(),tangents[tier][material].size()*sizeof(float));
            godot::Array arrays;arrays.append(points[tier][material]);arrays.append(normals[tier][material]);arrays.append(colours[tier][material]);arrays.append(uv[tier][material]);arrays.append(packed_tangents);
            godot::Dictionary surface;surface["material"]=material;surface["arrays"]=arrays;surfaces.append(surface);}
        out[tier==0?"near":"tall"]=surfaces;}
    out["origin_session"]=godot::Vector3(options.origin_e-terrain_->frame().e0_m(),options.origin_n-terrain_->frame().n0_m(),0);
    out["doors"]=doors;out["chimneys"]=chimneys;out["pitched_roofs"]=pitched;
    out["mapped_roof_directions"]=directed;out["supported_roof_directions"]=directed_supported;
    out["unsupported_roofs"]=unsupported;out["windows"]=window_count;out["building_count"]=building_count;return out;
}

void RgTerrainView::set_render_origin(godot::Vector3 session_origin) {
    render_origin_session_ = session_origin;
    for(const auto& [id,deck]:deck_instances_) {
        (void)id;godot::RenderingServer::get_singleton()->instance_set_transform(deck.instance,chunk_instance_transform(deck.chunk));
    }
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

void RgTerrainView::set_upload_budget_ms(double budget_ms) { upload_budget_ms_ = budget_ms > 0.0 ? budget_ms : 0.0; }

void RgTerrainView::update_focus(double session_x, double session_y) {
    focus_vx_ = 0.0;
    focus_vy_ = 0.0;
    if (streamer_ != nullptr && preview_state_ != PreviewState::Building) {
        streamer_->update_focus(session_x, session_y);
    }
}

void RgTerrainView::update_focus_moving(double session_x, double session_y, double vx, double vy, double ref_x,
                                        double ref_y) {
    focus_vx_ = vx;
    focus_vy_ = vy;
    if (streamer_ != nullptr && preview_state_ != PreviewState::Building) {
        streamer_->update_focus(session_x, session_y, vx, vy, ref_x, ref_y);
    }
}

void RgTerrainView::set_view_lod(const godot::Dictionary& choice) {
    rg::LodChoice c;
    c.view_distance_m = static_cast<double>(choice.get("view_distance_m", 0.0));
    c.terrain_detail = static_cast<double>(choice.get("terrain_detail", 1.0));
    c.road_detail = godot::String(choice.get("road_detail", godot::String())).utf8().get_data();
    c.map_detail = static_cast<double>(choice.get("map_detail", 1.0));
    const bool changed = !has_lod_choice_ || !(c == lod_choice_);
    lod_choice_ = c;
    has_lod_choice_ = true;
    if (!changed || terrain_ == nullptr) return;
    apply_view_lod_to_terrain();
    if (streamer_ != nullptr) {
        cancel_heal_rebuild(); // a heal batch built with the old road policy must not be swapped in
        streamer_->set_source(terrain_->view_source());
    }
}

void RgTerrainView::apply_view_lod_to_terrain() {
    if (terrain_ == nullptr || !has_lod_choice_) return;
    terrain_->set_lod_params(rg::terrain_lod_params(terrain_->world_lod_params(), lod_choice_));
    const std::shared_ptr<const rg::LodSettingsSpec> spec = rg::active_lod_settings();
    terrain_->set_render_road_lod(rg::road_lod_for(spec.get(), lod_choice_.road_detail));
}

void RgTerrainView::set_carve_max_level(std::int64_t level) {
    if (terrain_) terrain_->set_carve_max_level(static_cast<int>(level));
}

std::int64_t RgTerrainView::get_carve_max_level() const { return terrain_ ? terrain_->carve_max_level() : 0; }

void RgTerrainView::set_upload_budget_max_ms(double budget_ms) {
    upload_budget_max_ms_ = budget_ms > 0.0 ? budget_ms : 0.0;
}

godot::Dictionary RgTerrainView::get_lod_coverage(double px, double py, double dx, double dy) const {
    godot::Dictionary d;
    if (!terrain_) return d;
    std::vector<g2m::mesh::ChunkKey> visible;
    visible.reserve(next_upload_index_);
    for (std::size_t i = 0; i < next_upload_index_ && i < chunks_.size(); ++i) visible.push_back(chunks_[i].key);
    const rg::LodCoverage c = rg::lod_coverage_ahead(visible, static_cast<double>(terrain_->frame().e0_m()),
                                                     static_cast<double>(terrain_->frame().n0_m()), px, py, dx, dy);
    for (int k = 0; k < 4; ++k) {
        d[godot::String("l") + godot::String::num_int64(k) + "_ahead_m"] = c.ahead_m[static_cast<std::size_t>(k)];
    }
    return d;
}

godot::Dictionary RgTerrainView::get_ground_check(double px, double py, double dx, double dy) {
    godot::Dictionary d;
    d["visible_above_m"] = -1000.0;
    d["at_offset_m"] = 0.0;
    d["visible_level"] = static_cast<std::int64_t>(-1);
    d["samples"] = static_cast<std::int64_t>(0);
    d["far_above_m"] = -1000.0;
    d["far_samples"] = static_cast<std::int64_t>(0);
    d["max_since_reset_m"] = ground_check_max_m_;
    if (!terrain_) return d;
    const double e0 = static_cast<double>(terrain_->frame().e0_m());
    const double n0 = static_cast<double>(terrain_->frame().n0_m());
    const double len = std::hypot(dx, dy);
    const double ux = len > 1e-9 ? dx / len : 0.0;
    const double uy = len > 1e-9 ? dy / len : 0.0;
    const auto zone = terrain_->frame().zone();
    const auto lookup = [this](const g2m::TileKey& key) { return terrain_->cached_height_tile(key); };
    double best = -1000.0;
    double best_offset = 0.0;
    int best_level = -1;
    std::int64_t samples = 0;
    double far_best = -1000.0;
    std::int64_t far_samples = 0;
    // 0/5/20/60 m: where the wheels are now and in the next ~1 s at 60 m/s; 150/300 m: the ground the car reaches
    // in ~1-2 s at 160 m/s (physics resident only as far as the sim's interest set reaches - missing tiles skip).
    for (const double offset : {0.0, 5.0, 20.0, 60.0, 150.0, 300.0}) {
        if (offset > 0.0 && len <= 1e-9) break;
        const double x = px + ux * offset;
        const double y = py + uy * offset;
        double visible = -1.0e9;
        int level = -1;
        for (std::size_t i = 0; i < next_upload_index_ && i < chunks_.size(); ++i) {
            double z = 0.0;
            if (rg::chunk_surface_height(chunks_[i], x + e0, y + n0, z) && z > visible) {
                visible = z;
                level = chunks_[i].key.level;
            }
        }
        if (level < 0) continue;
        const auto physics = g2m::world::sample_height_exact(lookup, zone, terrain_->frame().e0_m(),
                                                              terrain_->frame().n0_m(), x, y);
        if (!physics) continue;
        const double above = visible - *physics;
        if (offset >= 100.0) {
            ++far_samples;
            far_best = std::max(far_best, above);
            continue;
        }
        ++samples;
        if (above > best) {
            best = above;
            best_offset = offset;
            best_level = level;
        }
    }
    if (samples > 0) {
        ground_check_max_m_ = std::max(ground_check_max_m_, best);
        d["visible_above_m"] = best;
        d["at_offset_m"] = best_offset;
        d["visible_level"] = static_cast<std::int64_t>(best_level);
        d["samples"] = samples;
        d["max_since_reset_m"] = ground_check_max_m_;
    }
    d["far_above_m"] = far_best;
    d["far_samples"] = far_samples;
    return d;
}

godot::Dictionary RgTerrainView::get_bed_check(double px, double py, double dx, double dy, double ahead_m, std::int64_t max_samples) {
    godot::Dictionary d;
    d["samples"] = static_cast<std::int64_t>(0);
    d["above_max_m"] = -1000.0;
    d["above_p95_m"] = -1000.0;
    d["above_tolerance"] = static_cast<std::int64_t>(0);
    d["worst_level"] = static_cast<std::int64_t>(-1);
    d["worst_carve"] = static_cast<std::int64_t>(0);
    if (!terrain_) return d;
    const double e0 = static_cast<double>(terrain_->frame().e0_m());
    const double n0 = static_cast<double>(terrain_->frame().n0_m());
    const double len = std::hypot(dx, dy);
    const double ux = len > 1e-9 ? dx / len : 0.0;
    const double uy = len > 1e-9 ? dy / len : 0.0;
    const double r = std::max(ahead_m, 150.0);
    std::vector<std::array<double, 3>> all;
    terrain_->road_bed_points(px + e0 - r, px + e0 + r, py + n0 - r, py + n0 + r, 6, all);
    std::vector<std::array<double, 3>> beds;
    beds.reserve(all.size());
    for (const auto& p : all) {
        const double along = (p[0] - e0 - px) * ux + (p[1] - n0 - py) * uy;
        if (along >= -100.0 && along <= ahead_m) beds.push_back(p);
    }
    // Diagnostic probe on the main thread: keep a uniform subsample (statistics over a few hundred points are as good
    // as over thousands, and the cost stays at about a millisecond).
    if (max_samples > 0 && beds.size() > static_cast<std::size_t>(max_samples)) {
        std::vector<std::array<double, 3>> thinned;
        thinned.reserve(static_cast<std::size_t>(max_samples));
        const double step = static_cast<double>(beds.size()) / static_cast<double>(max_samples);
        for (std::int64_t k = 0; k < max_samples; ++k) thinned.push_back(beds[static_cast<std::size_t>(static_cast<double>(k) * step)]);
        beds.swap(thinned);
    }
    const rg::BedCheck c = rg::rendered_vs_bed(chunks_, next_upload_index_, beds);
    d["samples"] = static_cast<std::int64_t>(c.samples);
    d["above_max_m"] = c.above_max_m;
    d["above_p95_m"] = c.above_p95_m;
    d["above_tolerance"] = static_cast<std::int64_t>(c.above_tolerance);
    d["worst_level"] = static_cast<std::int64_t>(c.worst_level);
    d["worst_carve"] = static_cast<std::int64_t>(c.worst_carve);
    d["worst_x"] = c.worst_e - e0;
    d["worst_y"] = c.worst_n - n0;
    for (int k = 0; k < 4; ++k) {
        d[godot::String("l") + godot::String::num_int64(k) + "_above_max_m"] = c.above_max_by_level[static_cast<std::size_t>(k)];
        d[godot::String("l") + godot::String::num_int64(k) + "_samples"] = static_cast<std::int64_t>(c.samples_by_level[static_cast<std::size_t>(k)]);
    }
    return d;
}

void RgTerrainView::set_terrain_visible(bool visible) {
    terrain_visible_ = visible;
    godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
    if (rs == nullptr) return;
    for (const godot::RID& rid : instance_rids_) {
        if (rid.is_valid()) rs->instance_set_visible(rid, visible);
    }
}

void RgTerrainView::set_cast_shadows(bool on) {
    cast_shadows_ = on;
    godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
    if (rs == nullptr) return;
    const auto setting = on ? godot::RenderingServer::SHADOW_CASTING_SETTING_ON : godot::RenderingServer::SHADOW_CASTING_SETTING_OFF;
    for (const godot::RID& rid : instance_rids_) {
        if (rid.is_valid()) rs->instance_geometry_set_cast_shadows_setting(rid, setting);
    }
}

// Main-thread cost of this node's last _process, split into its three parts (RG_FRAME_SPIKE line in main.gd).
godot::Dictionary RgTerrainView::get_frame_costs() const {
    godot::Dictionary d;
    d["upload_ms"] = upload_ms_this_frame_;
    d["heal_ms"] = heal_ms_this_frame_;
    d["decks_ms"] = decks_ms_this_frame_;
    d["uploads_total"] = uploads_total_;
    d["upload_pack_ms_total"] = upload_pack_ms_total_;       // main thread: building the surface arrays
    d["upload_surface_ms_total"] = upload_surface_ms_total_; // main thread: mesh_add_surface_from_arrays + material
    d["upload_instance_ms_total"] = upload_instance_ms_total_; // main thread: instance create + transform
    d["pending_work"] = static_cast<std::int64_t>((chunks_.size() - std::min(next_upload_index_, chunks_.size())) +
                                                  (diff_removed_.size() - std::min(diff_remove_next_, diff_removed_.size())));
    return d;
}

godot::Dictionary RgTerrainView::take_report_stats() {
    godot::Dictionary d;
    d["upload_ms_avg"] = upload_ms_window_.mean();
    d["upload_ms_max"] = upload_ms_window_.max();
    d["frames"] = static_cast<std::int64_t>(upload_ms_window_.count());
    d["chunks_uploaded"] = chunks_uploaded_window_;
    d["chunks_uploaded_total"] = chunks_uploaded_total_;
    d["chunk_latency_p95_ms"] = latency_ms_window_.percentile(0.95);
    d["chunk_latency_max_ms"] = latency_ms_window_.max();
    d["budget_ms_avg"] = budget_ms_window_.mean();
    upload_ms_window_.clear();
    latency_ms_window_.clear();
    budget_ms_window_.clear();
    chunks_uploaded_window_ = 0;
    return d;
}

godot::Vector3 RgTerrainView::godot_to_session(godot::Vector3 godot_pos) const {
    const ps::Vec3 render_origin_session{static_cast<ps::real>(render_origin_session_.x),
                                         static_cast<ps::real>(render_origin_session_.y),
                                         static_cast<ps::real>(render_origin_session_.z)};
    const ps::Vec3 session = godot_to_iso(godot_pos, render_origin_session);
    return godot::Vector3(static_cast<float>(session.x), static_cast<float>(session.y), static_cast<float>(session.z));
}

bool RgTerrainView::is_stream_idle() const {
    return is_fully_uploaded() &&
           (streamer_ == nullptr || streamer_->phase() == rg::TerrainViewStreamer::Phase::Idle);
}

godot::Dictionary RgTerrainView::get_stream_stats() const {
    godot::Dictionary d;
    d["pending_adds"] = static_cast<std::int64_t>(chunks_.size() - next_upload_index_);
    d["pending_removals"] = static_cast<std::int64_t>(diff_removed_.size() - diff_remove_next_);
    d["last_diff_added"] = last_diff_added_;
    d["last_diff_removed"] = last_diff_removed_;
    d["upload_ms_this_frame"] = upload_ms_this_frame_;
    d["last_remove_ms"] = last_remove_ms_;
    d["diffs_applied"] = diffs_applied_;
    d["diff_errors"] = diff_errors_;
    d["resident_chunks"] = static_cast<std::int64_t>(chunks_.size());
    {
        std::int64_t fallback_chunks = 0;
        for (std::size_t i = 0; i < next_upload_index_ && i < chunks_.size(); ++i) {
            if (chunks_[i].carve == rg::ChunkCarve::Fallback) ++fallback_chunks;
        }
        d["fallback_chunks"] = fallback_chunks;
        d["healed_tiles_total"] = healed_tiles_total_;
        d["rebuilt_chunks_total"] = rebuilt_chunks_total_;
        d["lod_rebuilt_chunks_total"] = lod_rebuilt_chunks_total_;
        d["pending_replacements"] = static_cast<std::int64_t>(diff_replaced_.size());
        d["fallback_tiles"] = terrain_ ? static_cast<std::int64_t>(terrain_->fallback_tile_count()) : 0;
    }
    d["upload_budget_ms"] = current_budget_ms_;
    d["focus_speed_mps"] = std::hypot(focus_vx_, focus_vy_);
    if (streamer_ != nullptr) {
        const rg::TerrainViewStreamer::Stats st = streamer_->stats();
        d["selections_started"] = static_cast<std::int64_t>(st.selections_started);
        d["last_build_ms"] = st.last_build_ms;
        d["streamer_phase"] = static_cast<std::int64_t>(streamer_->phase());
    } else {
        d["selections_started"] = static_cast<std::int64_t>(0);
        d["last_build_ms"] = 0.0;
        d["streamer_phase"] = static_cast<std::int64_t>(0);
    }
    return d;
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

void RgTerrainView::upload_one_chunk(const rg::RenderChunk& chunk, godot::Array* prepacked) {
    const auto start = std::chrono::steady_clock::now();

    godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
    const godot::RID mesh_rid = rs->mesh_create();

    // The surface arrays (terrain_surface_pack.h): normally built on the pack worker (pack_job_), else here. A chunk at
    // the imported region's coverage edge can have vertices but zero triangles (g2m::mesh::build_chunk leaves `indices`
    // empty where gather_window found nothing to triangulate - 24 of 488 chunks of the real home-r1 data at 20 km):
    // its arrays are empty and it gets no surface, because an EMPTY ARRAY_INDEX still sets Godot's ARRAY_FORMAT_INDEX
    // bit and then fails RenderingServer's surface validation ("index_array_len==NO_INDEX_ARRAY"). The mesh/instance RID
    // pair is still created either way - upload_one_chunk always appends exactly one entry to mesh_rids_/instance_rids_
    // per call, and set_render_origin()/chunk_instance_transform() index chunks_ and instance_rids_ in lockstep.
    godot::Array arrays = prepacked != nullptr ? std::move(*prepacked) : pack_chunk_surface(chunk);
    const auto packed_at = std::chrono::steady_clock::now();
    if (!arrays.is_empty()) {
        rs->mesh_add_surface_from_arrays(mesh_rid, godot::RenderingServer::PRIMITIVE_TRIANGLES, arrays);
        if (material_rid_.is_valid()) {
            rs->mesh_surface_set_material(mesh_rid, 0, material_rid_);
        }
    }
    const auto surface_at = std::chrono::steady_clock::now();

    const godot::RID scenario = get_world_3d().is_valid() ? get_world_3d()->get_scenario() : godot::RID();
    const godot::RID instance_rid = rs->instance_create2(mesh_rid, scenario);
    if (!terrain_visible_) rs->instance_set_visible(instance_rid, false);
    if (!cast_shadows_) rs->instance_geometry_set_cast_shadows_setting(instance_rid, godot::RenderingServer::SHADOW_CASTING_SETTING_OFF);
    rs->instance_set_transform(instance_rid, chunk_instance_transform(chunk));

    mesh_rids_.push_back(mesh_rid);
    instance_rids_.push_back(instance_rid);

    const auto end = std::chrono::steady_clock::now();
    total_upload_time_ms_ += std::chrono::duration<double, std::milli>(end - start).count();
    const auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
    upload_pack_ms_total_ += ms(start, packed_at);
    upload_surface_ms_total_ += ms(packed_at, surface_at);
    upload_instance_ms_total_ += ms(surface_at, end);
    ++uploads_total_;
}

// Moves the surfaces the pack worker has finished into surfaces_ (main thread). Once a job's last chunk is taken its
// thread is joined (it only has its timing store left), so chunks_ may move again.
void RgTerrainView::drain_packed() {
    while (pack_taken_ < pack_job_.count() && pack_taken_ < pack_job_.packed()) {
        surfaces_.push_back(pack_job_.take(pack_taken_++));
    }
    if (pack_taken_ == pack_job_.count()) pack_job_.cancel_and_join();
}

bool RgTerrainView::pack_idle() {
    drain_packed();
    return pack_taken_ == pack_job_.count();
}

// Packs chunks_[first, chunks_.size()) on the worker. Every chunk ever appended to chunks_ goes through here, so the
// chunks waiting for upload are exactly surfaces_ (in order) followed by the job's range.
void RgTerrainView::start_packing(std::size_t first) {
    pack_job_.start(chunks_.data() + first, chunks_.size() - first);
    pack_taken_ = 0;
}

void RgTerrainView::reset_packing() {
    pack_job_.cancel_and_join();
    pack_job_.start(nullptr, 0);
    pack_taken_ = 0;
    surfaces_.clear();
}

// Uploads chunks_[next_upload_index_] if its surface is packed; false (nothing done) while the worker is still on it.
bool RgTerrainView::upload_next_chunk() {
    drain_packed();
    if (surfaces_.empty()) return false;
    godot::Array arrays = std::move(surfaces_.front());
    surfaces_.pop_front();
    upload_one_chunk(chunks_[next_upload_index_], &arrays);
    ++next_upload_index_;
    return true;
}

void RgTerrainView::sync_road_decks() {
    if(!terrain_) return;
    int uploaded=0;
    for(const auto& deck:terrain_->road_decks()) {
        const auto id=std::tuple{deck->way_id,static_cast<int>(std::round(deck->start_station)),static_cast<int>(std::round(deck->end_station))};
        if(deck_instances_.contains(id)) continue;
        DeckInstance instance;instance.chunk.mesh=deck->mesh;
        instance.chunk.origin_session[0]=deck->mesh.origin[0]-terrain_->frame().e0_m();
        instance.chunk.origin_session[1]=deck->mesh.origin[1]-terrain_->frame().n0_m();
        instance.chunk.rgba.assign(deck->mesh.positions.size()/3,deck->land_class==g2m::LandClass::PavedRoad?0x4c4c5200u:0x5c472900u);
        // Appearance is generated on geo2map's structure worker. Only the
        // rendering copy is extended; physics still consumes deck->mesh alone.
        const auto& visual=deck->appearance.mesh;
        const auto base=instance.chunk.mesh.positions.size()/3;
        if(!visual.positions.empty() && base+visual.positions.size()/3<65536){
            auto& mesh=instance.chunk.mesh;
            mesh.positions.insert(mesh.positions.end(),visual.positions.begin(),visual.positions.end());
            mesh.normals.insert(mesh.normals.end(),visual.normals.begin(),visual.normals.end());
            mesh.land_class.insert(mesh.land_class.end(),visual.land_class.begin(),visual.land_class.end());
            for(auto i:visual.indices)mesh.indices.push_back(static_cast<std::uint16_t>(base+i));
            instance.chunk.rgba.insert(instance.chunk.rgba.end(),visual.positions.size()/3,deck->appearance.rgba);
        }
        upload_one_chunk(instance.chunk);
        instance.mesh=mesh_rids_.back();instance.instance=instance_rids_.back();
        mesh_rids_.pop_back();instance_rids_.pop_back();
        deck_instances_.emplace(id,std::move(instance));
        if(++uploaded>=2) break;
    }
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
    if(rs) for(const auto& [id,deck]:deck_instances_) {
        (void)id;rs->free_rid(deck.instance);rs->free_rid(deck.mesh);
    }
    deck_instances_.clear();
    instance_rids_.clear();
    mesh_rids_.clear();
}

namespace {
std::uint8_t carve_of_tile(void* ctx, const g2m::TileKey& key) {
    return static_cast<std::uint8_t>(static_cast<rg::WorldTerrain*>(ctx)->tile_carve(key));
}
} // namespace

// Replaces the uploaded chunk at `index` by `fresh` (same key) without a hole: the new mesh is uploaded first, then the
// old RIDs are freed. mesh_rids_/instance_rids_/chunks_ stay index-aligned; next_upload_index_ and index_of_ are unchanged.
void RgTerrainView::replace_chunk_at(std::size_t index, rg::RenderChunk&& fresh) {
    godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
    upload_one_chunk(fresh); // appends one RID pair at the back
    if (instance_rids_[index].is_valid()) rs->free_rid(instance_rids_[index]);
    if (mesh_rids_[index].is_valid()) rs->free_rid(mesh_rids_[index]);
    mesh_rids_[index] = mesh_rids_.back();
    instance_rids_[index] = instance_rids_.back();
    mesh_rids_.pop_back();
    instance_rids_.pop_back();
    chunks_[index] = std::move(fresh);
}

// Gray-patch fix (2026-10-13): a coarse (L1/L2) chunk built while the road surface was missing is an uncarved Fallback
// chunk - ground above the physics road bed, hiding the ribbons - and the streamer never rebuilds a key it already
// holds. Every 2 s, while the terrain reports Fallback tiles, they are re-carved on a background thread
// (WorldTerrain::heal_fallback_tiles); the resident chunks built from them (stale_fallback_chunks) are then rebuilt, at
// most 2 per frame, and swapped in place. A deterministic failure stays a Fallback and is retried only every 2 s.
void RgTerrainView::cancel_heal_rebuild() {
    heal_job_.cancel_and_join();
    heal_ready_.clear();
}

void RgTerrainView::heal_tick() {
    heal_swap_ms_ = 0.0;
    if (!terrain_) return;
    static const bool no_heal = std::getenv("RG_TERRAIN_NO_HEAL") != nullptr; // A/B aid for the gray-patch diagnosis
    if (no_heal) return;
    if (heal_future_.valid()) {
        if (heal_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) healed_tiles_total_ += static_cast<std::int64_t>(heal_future_.get());
    } else {
        const double now_s = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()) / 1000.0;
        if (now_s >= heal_next_check_s_ && terrain_->fallback_tile_count() > 0) {
            heal_next_check_s_ = now_s + 2.0;
            rg::WorldTerrain* wt = terrain_.get();
            heal_future_ = std::async(std::launch::async, [wt]() { return wt->heal_fallback_tiles(); });
        }
    }
    // Rebuilds run on a worker (the tile lookups can block on a derivation); the frame only swaps finished chunks in.
    const auto swap_start = std::chrono::steady_clock::now();
    if (heal_ready_.empty()) heal_job_.take(heal_ready_);
    std::size_t swapped = 0;
    while (!heal_ready_.empty() && swapped < kHealSwapsPerFrame) {
        rg::RenderChunk fresh = std::move(heal_ready_.back());
        heal_ready_.pop_back();
        const auto it = index_of_.find(fresh.key);
        // Gone since the batch started, not uploaded yet, or already healed some other way: drop it.
        if (it == index_of_.end() || it->second >= next_upload_index_ || chunks_[it->second].carve != rg::ChunkCarve::Fallback) continue;
        replace_chunk_at(it->second, std::move(fresh));
        ++rebuilt_chunks_total_;
        ++swapped;
    }
    heal_swap_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - swap_start).count();
    if (!heal_ready_.empty() || heal_job_.busy()) return;

    rg::WorldTerrain* wt = terrain_.get();
    const auto stale = rg::stale_fallback_chunks(chunks_, next_upload_index_, [wt](const g2m::mesh::ChunkKey& key) {
        return rg::chunk_window_carve(key, wt, &carve_of_tile);
    });
    if (stale.empty()) return;
    std::vector<g2m::mesh::ChunkKey> keys;
    for (const std::size_t index : stale) {
        if (keys.size() >= kHealBatch) break;
        keys.push_back(chunks_[index].key);
    }
    const rg::TerrainViewSource source = terrain_->view_source();
    heal_job_.start(std::move(keys), [source](const std::vector<g2m::mesh::ChunkKey>& k, std::vector<rg::RenderChunk>& out,
                                              const std::atomic<bool>* cancel) {
        return g2m::world::build_view_chunks(k, source, 1, out, cancel);
    });
}

void RgTerrainView::remove_chunk_at(std::size_t index) {
    // Only called with the whole queue uploaded, so all three arrays have the
    // same length and swap-remove keeps them index-aligned.
    godot::RenderingServer* rs = godot::RenderingServer::get_singleton();
    if (instance_rids_[index].is_valid()) rs->free_rid(instance_rids_[index]);
    if (mesh_rids_[index].is_valid()) rs->free_rid(mesh_rids_[index]);
    index_of_.erase(chunks_[index].key);
    const std::size_t last = chunks_.size() - 1;
    if (index != last) {
        chunks_[index] = std::move(chunks_[last]);
        mesh_rids_[index] = mesh_rids_[last];
        instance_rids_[index] = instance_rids_[last];
        index_of_[chunks_[index].key] = index;
    }
    chunks_.pop_back();
    mesh_rids_.pop_back();
    instance_rids_.pop_back();
    next_upload_index_ = chunks_.size();
}

void RgTerrainView::apply_diff(std::chrono::steady_clock::time_point frame_start) {
    // 1. Take a ready diff (only one in progress at a time - the streamer
    //    itself never has more than one). Its adds join the upload queue.
    // A new diff is taken only while the pack worker is idle: appending its adds may move chunks_, which the worker reads.
    if (!diff_in_progress_ && streamer_ != nullptr && preview_state_ != PreviewState::Building && pack_idle()) {
        rg::TerrainViewDiff diff;
        if (streamer_->poll(diff)) {
            diff_in_progress_ = true;
            diff_serial_ = diff.serial;
            diff_selected_at_ = diff.selected_at;
            diff_removed_ = std::move(diff.removed);
            diff_replaced_ = std::move(diff.replaced);
            diff_remove_next_ = 0;
            last_remove_ms_ = 0.0;
            last_diff_added_ = static_cast<std::int64_t>(diff.added.size());
            last_diff_removed_ = static_cast<std::int64_t>(diff_removed_.size());
            const std::size_t first_add = chunks_.size();
            chunks_.reserve(chunks_.size() + diff.added.size());
            for (rg::RenderChunk& chunk : diff.added) {
                index_of_[chunk.key] = chunks_.size();
                chunks_.push_back(std::move(chunk));
            }
            start_packing(first_add); // the surface arrays are built on the pack worker, uploaded below as they finish
        }
    }

    // 2. Upload. The initial preview load is a loading phase (count budget);
    //    streamed adds get the steady-clock time budget. `work` counts this
    //    frame's uploads + removals: the first one always runs, so a diff
    //    always makes progress.
    int work = 0;
    const std::size_t pending_work = (chunks_.size() - next_upload_index_) + (diff_removed_.size() - diff_remove_next_);
    double budget_ms = upload_budget_ms_;
    if (upload_adaptive_ && !bulk_loading_) {
        rg::UploadBudgetParams budget_params;
        budget_params.base_ms = upload_budget_ms_;
        budget_params.max_ms = std::max(upload_budget_ms_, upload_budget_max_ms_);
        budget_ms = rg::adaptive_upload_budget_ms(pending_work, std::hypot(focus_vx_, focus_vy_), budget_params);
    }
    current_budget_ms_ = budget_ms;
    const auto over_budget = [&]() {
        return work > 0 &&
               std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - frame_start).count() >=
                   budget_ms;
    };
    if (bulk_loading_) {
        const std::size_t end_index =
            std::min(chunks_.size(), next_upload_index_ + static_cast<std::size_t>(upload_budget_per_frame_));
        while (next_upload_index_ < end_index && upload_next_chunk()) {
            ++work;
        }
        if (next_upload_index_ >= chunks_.size()) {
            bulk_loading_ = false;
        }
    } else {
        while (next_upload_index_ < chunks_.size() && !over_budget()) {
            if (!upload_next_chunk()) break; // the next surface is still being packed: next frame
            ++work;
            ++chunks_uploaded_window_;
            ++chunks_uploaded_total_;
            if (diff_in_progress_) {
                latency_ms_window_.add(
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - diff_selected_at_).count());
            }
        }
    }

    // 3. Every add of the diff is uploaded: only now free the removed chunks
    //    (no holes - their replacements are already up), within the same
    //    frame budget, possibly over several frames (overlap, never a hole).
    if (diff_in_progress_ && next_upload_index_ >= chunks_.size()) {
        // Road LOD rebuilds: each replaces the resident chunk with its key in place (upload, then free the old).
        while (!diff_replaced_.empty() && !over_budget()) {
            rg::RenderChunk fresh = std::move(diff_replaced_.back());
            diff_replaced_.pop_back();
            ++work;
            const auto it = index_of_.find(fresh.key);
            if (it == index_of_.end()) continue; // not resident any more (cannot happen: replaced keys stay selected)
            replace_chunk_at(it->second, std::move(fresh));
            ++lod_rebuilt_chunks_total_;
        }
        const auto remove_start = std::chrono::steady_clock::now();
        while (diff_replaced_.empty() && diff_remove_next_ < diff_removed_.size() && !over_budget()) {
            const g2m::mesh::ChunkKey key = diff_removed_[diff_remove_next_++];
            ++work;
            auto it = index_of_.find(key);
            if (it == index_of_.end()) {
                ++diff_errors_;
                godot::UtilityFunctions::push_error("RgTerrainView: streamed diff removes a chunk that is not resident");
                continue;
            }
            remove_chunk_at(it->second);
        }
        last_remove_ms_ +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - remove_start).count();
    }

    // 4. Every add uploaded and every removal freed: commit.
    if (diff_in_progress_ && next_upload_index_ >= chunks_.size() && diff_replaced_.empty() &&
        diff_remove_next_ >= diff_removed_.size()) {
        diff_removed_.clear();
        diff_remove_next_ = 0;
        diff_in_progress_ = false;
        ++diffs_applied_;
        streamer_->commit(diff_serial_); // may start the next selection at once (coalesced focus)
    }
}

void RgTerrainView::_process(double /*delta*/) {
    const auto ms_since = [](std::chrono::steady_clock::time_point t) {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
    };
    const auto decks_start = std::chrono::steady_clock::now();
    sync_road_decks();
    decks_ms_this_frame_ = ms_since(decks_start);
    const auto heal_start = std::chrono::steady_clock::now();
    heal_tick();
    heal_ms_this_frame_ = heal_swap_ms_; // main-thread swap time only (the rebuild is on a worker)
    (void)heal_start;
    const auto frame_start = std::chrono::steady_clock::now();
    apply_diff(frame_start);
    upload_ms_this_frame_ =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - frame_start).count();
    upload_ms_window_.add(upload_ms_this_frame_);
    budget_ms_window_.add(current_budget_ms_);
}

void RgTerrainView::_bind_methods() {
    godot::ClassDB::bind_method(D_METHOD("initialize", "world_config_absolute_path"), &RgTerrainView::initialize);
    godot::ClassDB::bind_method(D_METHOD("get_road_visual_tile", "x", "y", "lift_m", "line_width_m"), &RgTerrainView::get_road_visual_tile, DEFVAL(0.15));
    godot::ClassDB::bind_method(D_METHOD("get_rail_tile", "x", "y"), &RgTerrainView::get_rail_tile);
    godot::ClassDB::bind_method(D_METHOD("get_rail_info"), &RgTerrainView::get_rail_info);
    godot::ClassDB::bind_method(D_METHOD("get_building_tile", "x", "y", "min_height", "fallback_height", "storey_height"), &RgTerrainView::get_building_tile);
    godot::ClassDB::bind_method(D_METHOD("get_building_mesh_tile", "x", "y", "min_height", "fallback_height", "storey_height", "skyline_height"), &RgTerrainView::get_building_mesh_tile);
    godot::ClassDB::bind_method(D_METHOD("initialize_shared", "sim"), &RgTerrainView::initialize_shared);
    godot::ClassDB::bind_method(D_METHOD("release"), &RgTerrainView::release);
    godot::ClassDB::bind_method(D_METHOD("load_synthetic_preview", "spawn_x", "spawn_y", "max_distance_m"), &RgTerrainView::load_synthetic_preview);
    godot::ClassDB::bind_method(D_METHOD("get_spawn_x"), &RgTerrainView::get_spawn_x);
    godot::ClassDB::bind_method(D_METHOD("get_spawn_y"), &RgTerrainView::get_spawn_y);
    godot::ClassDB::bind_method(D_METHOD("load_preview", "spawn_x", "spawn_y"), &RgTerrainView::load_preview);
    godot::ClassDB::bind_method(D_METHOD("begin_preview", "spawn_x", "spawn_y"), &RgTerrainView::begin_preview);
    godot::ClassDB::bind_method(D_METHOD("poll_preview"), &RgTerrainView::poll_preview);
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
    godot::ClassDB::bind_method(D_METHOD("set_upload_budget_ms", "budget_ms"), &RgTerrainView::set_upload_budget_ms);
    godot::ClassDB::bind_method(D_METHOD("get_upload_budget_ms"), &RgTerrainView::get_upload_budget_ms);
    godot::ClassDB::bind_method(D_METHOD("update_focus", "session_x", "session_y"), &RgTerrainView::update_focus);
    godot::ClassDB::bind_method(D_METHOD("update_focus_moving", "session_x", "session_y", "vx", "vy", "ref_x", "ref_y"),
                                &RgTerrainView::update_focus_moving);
    godot::ClassDB::bind_method(D_METHOD("set_view_lod", "choice"), &RgTerrainView::set_view_lod);
    godot::ClassDB::bind_method(D_METHOD("set_upload_adaptive", "enabled"), &RgTerrainView::set_upload_adaptive);
    godot::ClassDB::bind_method(D_METHOD("get_upload_adaptive"), &RgTerrainView::get_upload_adaptive);
    godot::ClassDB::bind_method(D_METHOD("set_upload_budget_max_ms", "budget_ms"), &RgTerrainView::set_upload_budget_max_ms);
    godot::ClassDB::bind_method(D_METHOD("get_upload_budget_max_ms"), &RgTerrainView::get_upload_budget_max_ms);
    godot::ClassDB::bind_method(D_METHOD("set_carve_max_level", "level"), &RgTerrainView::set_carve_max_level);
    godot::ClassDB::bind_method(D_METHOD("get_carve_max_level"), &RgTerrainView::get_carve_max_level);
    godot::ClassDB::bind_method(D_METHOD("get_lod_coverage", "px", "py", "dx", "dy"), &RgTerrainView::get_lod_coverage);
    godot::ClassDB::bind_method(D_METHOD("get_ground_check", "px", "py", "dx", "dy"), &RgTerrainView::get_ground_check);
    godot::ClassDB::bind_method(D_METHOD("get_bed_check", "px", "py", "dx", "dy", "ahead_m", "max_samples"), &RgTerrainView::get_bed_check, DEFVAL(800));
    godot::ClassDB::bind_method(D_METHOD("set_terrain_visible", "visible"), &RgTerrainView::set_terrain_visible);
    godot::ClassDB::bind_method(D_METHOD("get_terrain_visible"), &RgTerrainView::get_terrain_visible);
    godot::ClassDB::bind_method(D_METHOD("set_cast_shadows", "on"), &RgTerrainView::set_cast_shadows);
    godot::ClassDB::bind_method(D_METHOD("get_cast_shadows"), &RgTerrainView::get_cast_shadows);
    godot::ClassDB::bind_method(D_METHOD("take_report_stats"), &RgTerrainView::take_report_stats);
    godot::ClassDB::bind_method(D_METHOD("get_frame_costs"), &RgTerrainView::get_frame_costs);
    godot::ClassDB::bind_method(D_METHOD("godot_to_session", "godot_pos"), &RgTerrainView::godot_to_session);
    godot::ClassDB::bind_method(D_METHOD("is_stream_idle"), &RgTerrainView::is_stream_idle);
    godot::ClassDB::bind_method(D_METHOD("get_stream_stats"), &RgTerrainView::get_stream_stats);
}

} // namespace rg_godot
