// godot_ext/src/rg_garage.h - RgGarage: the Godot-facing face of rg::Garage and
// rg::GarageCamera (PLAN.md R6). Every decision (which options exist, whether a
// value is allowed, what the loader says, which file the drive uses, where the
// camera goes) lives in rg_core; this node converts to and from Godot Variants
// so shell_ui.gd / garage_set.gd / main.gd can draw the view-models and forward
// the player's choices. It owns no scene node and touches no Session.
//
// Frames: positions handed to GDScript are converted from the garage's ISO frame
// (x forward, y left, z up) to Godot's (x right, y up, z back) in ONE place here:
// godot = (-iso.y, iso.z, -iso.x). A turntable angle about ISO z is the same angle
// about Godot's +Y.
#pragma once

#include "rg/garage.h"
#include "rg/garage_set.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <memory>
#include <optional>
#include <string>

namespace rg_godot {

class RgGarage : public godot::Node {
    GDCLASS(RgGarage, godot::Node)

public:
    RgGarage() = default;
    ~RgGarage() override;

    // repo_root: racing_game's root (absolute; the catalog's relative paths are
    // relative to it), user_dir: the globalized user:// folder, work_root: where
    // validations and the drive materialisation go (a folder under the user dir).
    // {ok, error}
    godot::Dictionary initialize(const godot::String& repo_root, const godot::String& user_dir,
                                 const godot::String& work_root);
    [[nodiscard]] bool is_ready() const { return garage_ != nullptr; }

    // ---- vehicles ----
    // [{id, title, subtitle, description, default, selected, has_setup, model_path,
    //   paint, rim, stats: {ok, error, engine_name, engine_kind, peak_torque_nm,
    //   peak_torque_rpm, peak_power_kw, peak_power_rpm, figures_declared, mass_kg,
    //   layout, driven_wheels, wheel_count, gear_count}}]
    [[nodiscard]] godot::Array get_vehicles();
    [[nodiscard]] godot::String get_selected_id() const;
    godot::Dictionary select(const godot::String& id); // {ok, error}; persisted
    [[nodiscard]] godot::Dictionary get_vehicle(const godot::String& id);

    // ---- the set (garage_set.json as a Dictionary, positions in Godot axes) ----
    [[nodiscard]] godot::Dictionary get_set() const;

    // ---- the configurator ----
    // {ok, error}: starts the edit session of `id` (loads its saved setup).
    godot::Dictionary begin_edit(const godot::String& id);
    [[nodiscard]] godot::String get_edit_id() const;
    // [{id, label, group, area, help, unit, kind, min, max, step, available, stock, value,
    //   modified, choices, list_size, stock_numbers}]
    [[nodiscard]] godot::Array get_options() const;
    // {accepted, message, ok, validation_message, value, dirty}
    godot::Dictionary set_option(const godot::String& option_id, const godot::Variant& value);
    godot::Dictionary reset_option(const godot::String& option_id);
    godot::Dictionary reset_all();
    [[nodiscard]] bool is_dirty() const;
    // {ok, message}: the loader's verdict on the working copy
    [[nodiscard]] godot::Dictionary get_validation() const;
    // {ok, error}
    godot::Dictionary save();
    void discard();
    // {paint, rim}: the working colours (visual only)
    [[nodiscard]] godot::Dictionary get_working_colours() const;

    // ---- hand-over to the drive ----
    // {ok, error, warning, vehicle_id, sim_name, vehicle_path, model_path, paint, rim,
    //  engine_map_cache_dir, modified, chassis: {mass_kg, half_extents, z_m},
    //  assist_auto_clutch, assist_auto_blip, assist_auto_shift, manual_gearbox}
    godot::Dictionary prepare_drive(const godot::String& id);
    void cleanup();
    // Number of regular files below the garage's work root (tests: nothing is left behind).
    [[nodiscard]] int get_work_file_count() const;

    // ---- camera ----
    // The subject comes from the loaded model's bounds in the CAR (ISO) frame, or an
    // empty/zero box to use the chassis-box estimate. {ok, error}
    godot::Dictionary setup_camera(const godot::String& id, const godot::Vector3& model_min_iso,
                                   const godot::Vector3& model_max_iso, bool has_model_bounds);
    bool camera_go_to(const godot::String& area_id, bool instant);
    void camera_update(double dt_s);
    // {position, look_at, fov_deg, car_yaw_rad (about Godot +Y), area, settled, in_transition, progress}
    [[nodiscard]] godot::Dictionary get_camera() const;
    // The default area id of an option's group ("overview" when unknown).
    [[nodiscard]] godot::String get_area_for_option(const godot::String& option_id) const;

protected:
    static void _bind_methods();

private:
    [[nodiscard]] godot::Dictionary options_to_array_result() const;
    godot::Dictionary edit_reply(const rg::EditResult& r) const;

    std::unique_ptr<rg::Garage> garage_;
    std::optional<rg::GarageCamera> camera_;
    std::string work_root_;
};

} // namespace rg_godot
