// godot_ext/src/rg_simulation.h — RgSimulation: the thin godot::Node
// wrapper around rg::Session (core/include/rg/session.h). Mirrors
// physics_sim's own adapters/godot/src/ps_simulation.h in SHAPE (a Node
// exposing simulation state as Godot-native types/Dictionaries to
// GDScript) but is deliberately much smaller - R0 scope is one ground body,
// one car_sedan-shaped vehicle, no terrain streaming, no articulations, no
// debug draw, no render-time interpolation (PLAN.md's judder fix is a
// later-milestone concern - see this file's own get_body_transform comment
// for exactly what is skipped and why).
//
// Every ps::/rg:: type crossing into a godot:: type happens in THIS file
// (plus frame_convert.h) - the engine-neutral/Godot boundary PLAN.md 11.1
// draws, mirrored from physics_sim's own P2 report.

#pragma once

#include "rg/session.h"

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <atomic>
#include <memory>

namespace rg_godot {

class RgSimulation : public godot::Node {
    GDCLASS(RgSimulation, godot::Node)

public:
    RgSimulation() = default;
    ~RgSimulation() override = default;

    // Builds the rg::Session (ground + car_sedan-shaped chassis + vehicle -
    // see rg::SessionConfig's own doc comment for the exact geometry).
    // Paths are absolute, globalized (ProjectSettings.globalize_path) by
    // the GDScript caller - this class does no res:// resolution itself,
    // same "no Godot resource-path convention leaks into rg_core" reasoning
    // as SessionConfig's own doc comment. Returns false and sets
    // get_last_error() on failure (e.g. a malformed vehicle JSON) instead
    // of letting rg::io::load_vehicle_json's std::runtime_error cross the
    // GDExtension boundary uncaught.
    bool initialize(const godot::String& vehicle_json_absolute_path, const godot::String& surface_table_absolute_path);

    void start();
    void stop();
    [[nodiscard]] bool is_running() const;

    [[nodiscard]] std::int64_t get_step_count() const;
    [[nodiscard]] double get_sim_time() const;
    [[nodiscard]] double get_tick_rate_hz() const;
    [[nodiscard]] std::int64_t get_body_count() const; // always 2 in R0 (ground, chassis)
    [[nodiscard]] godot::String get_last_error() const { return last_error_; }

    // --- Floating origin (PLAN.md 11.3: "only the active camera rebases") ---
    [[nodiscard]] godot::Vector3 get_world_origin_godot_position() const;
    godot::Vector3 rebase_focus(const godot::Vector3& focus_godot_position);

    // --- Adapter frame-time accounting (chase_cam.gd's own HUD counter -
    // plain accumulation, no core involvement, see .cpp) ---
    void add_adapter_time_us(std::int64_t us);
    [[nodiscard]] std::int64_t consume_adapter_frame_time_us();

    // --- Body transforms. "ground"/"chassis" are the only two body names
    // in R0 (rg::Session::build_world_contents). NOT render-time
    // interpolated (unlike physics_sim's own get_body_transform) - R0 reads
    // the session's latest published FrameSnapshot as-is; a visible
    // 240 Hz-vs-render-rate judder fix is deferred (see repo CLAUDE.md's
    // deviations list). ---
    [[nodiscard]] godot::Transform3D get_body_transform(const godot::String& body_name) const;
    [[nodiscard]] float get_body_speed_mps(const godot::String& body_name) const;

    // --- Named control channels (PLAN.md P2) ---
    void set_control(const godot::String& channel, double value);
    [[nodiscard]] double get_control(const godot::String& channel) const;

    // --- Vehicle / wheel telemetry. vehicle_name is unused beyond
    // validating it equals the one vehicle R0 ever creates ("car",
    // game/scripts/main.gd) - kept as a parameter for API parity with
    // physics_sim's own multi-vehicle-shaped methods, so a later milestone
    // adding a second vehicle needs no signature change here. ---
    [[nodiscard]] godot::PackedStringArray get_vehicle_names() const;
    [[nodiscard]] std::int64_t get_vehicle_wheel_count(const godot::String& vehicle_name) const;
    [[nodiscard]] godot::String get_wheel_name(const godot::String& vehicle_name, std::int64_t wheel_index) const;
    [[nodiscard]] float get_wheel_load_n(const godot::String& vehicle_name, std::int64_t wheel_index) const;
    [[nodiscard]] float get_wheel_slip_ratio(const godot::String& vehicle_name, std::int64_t wheel_index) const;
    [[nodiscard]] float get_wheel_slip_angle(const godot::String& vehicle_name, std::int64_t wheel_index) const;
    [[nodiscard]] godot::String get_wheel_surface_name(const godot::String& vehicle_name, std::int64_t wheel_index) const;

    [[nodiscard]] godot::Dictionary get_vehicle_gauge_info(const godot::String& vehicle_name) const;
    [[nodiscard]] godot::Dictionary get_vehicle_powertrain(const godot::String& vehicle_name) const;
    [[nodiscard]] float get_vehicle_ground_speed_mps(const godot::String& vehicle_name) const;

protected:
    static void _bind_methods();

private:
    [[nodiscard]] bool has_vehicle(const godot::String& vehicle_name) const;

    std::unique_ptr<rg::Session> session_;
    godot::String last_error_;
    ps_godot::OriginRebase* origin_rebase_ = nullptr; // points at session_->origin_rebase(), valid once session_ exists
    std::atomic<std::int64_t> adapter_time_us_accum_{0}; // main-thread-only in practice (Godot single main thread)
};

} // namespace rg_godot
