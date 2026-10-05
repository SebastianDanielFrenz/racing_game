// godot_ext/src/rg_shell.h - RgShell: the Godot-facing face of the game shell's
// engine-neutral models (PLAN.md R5): rg::ShellFlow (screen state machine),
// rg::Settings, rg::Credits, rg::SpawnPresets. Every decision lives in rg_core;
// this node only converts to and from Godot Variants (Dictionary / Array) so
// game/scripts/shell_ui.gd and main.gd can draw the view-models and forward
// events. It owns no session and touches no scene node.
#pragma once

#include "rg/credits.h"
#include "rg/settings.h"
#include "rg/shell_flow.h"
#include "rg/spawn_presets.h"

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/variant.hpp>

#include <optional>
#include <string>
#include <vector>

namespace rg_godot {

class RgShell : public godot::Node {
    GDCLASS(RgShell, godot::Node)

public:
    RgShell() = default;

    // data_dir: racing_game's own data/ (absolute); user_dir: the absolute,
    // globalized user:// folder (settings.json and last_drive.json live there);
    // world_config_path: data/world/world_config.json (absolute). Loads
    // credits, spawn presets and settings; every failure is non-fatal (the
    // returned Dictionary lists it under "problems") except that a missing
    // credits file leaves an empty attribution line.
    godot::Dictionary initialize(const godot::String& data_dir, const godot::String& user_dir,
                                 const godot::String& world_config_path);

    // ---- screen flow (rg::ShellFlow). Every event returns the transition:
    // {accepted, from, to, actions: [{kind, screen, world, flag}]} with kind in
    // "show_screen" | "load_world" | "unload_world" | "set_paused" |
    // "reset_car" | "save_settings" | "quit" | "open_garage" | "close_garage"; world = {kind: "flat"|"real_world",
    // has_spawn, x, y, yaw_deg, label, open_address_search}. ----
    [[nodiscard]] godot::String get_screen() const;
    [[nodiscard]] godot::String get_settings_return() const;
    [[nodiscard]] godot::Dictionary get_world_request() const;
    [[nodiscard]] godot::String get_last_error() const;
    void clear_last_error();
    godot::Dictionary boot_finished();
    godot::Dictionary direct_start(const godot::Dictionary& world);
    godot::Dictionary menu_item(const godot::String& id);
    godot::Dictionary spawn_picked(const godot::String& choice_id);
    godot::Dictionary back();
    godot::Dictionary load_ready();
    godot::Dictionary load_failed(const godot::String& message);
    godot::Dictionary load_cancelled();
    godot::Dictionary pause_toggle();
    // R6 garage flow: a car chosen in vehicle select; "Drive" in the configurator.
    godot::Dictionary vehicle_chosen(const godot::String& id);
    godot::Dictionary garage_drive();
    [[nodiscard]] godot::String get_garage_return() const;  // "main_menu" | "pause"
    [[nodiscard]] godot::String get_garage_vehicle() const; // the configurator's car
    [[nodiscard]] godot::Array get_main_menu_items() const; // [{id, label}]
    [[nodiscard]] godot::Array get_pause_menu_items() const;

    // ---- spawn picker. Re-reads user://last_drive.json each call.
    // [{id, kind, label, detail, available, unavailable_reason, world,
    //   has_position, x, y, yaw_deg}] ----
    godot::Array get_spawn_choices();

    // ---- settings ----
    // [{id, title, settings: [{key, label, help, type, default, min, max, step,
    //    choices, applies_live}]}]
    [[nodiscard]] godot::Array get_settings_schema() const;
    [[nodiscard]] godot::Variant get_setting(const godot::String& key) const;
    [[nodiscard]] godot::String get_setting_display(const godot::String& key) const;
    // {ok, error, clamped, value}
    godot::Dictionary set_setting(const godot::String& key, const godot::Variant& value);
    void reset_setting(const godot::String& key);
    void reset_all_settings();
    bool save_settings();
    [[nodiscard]] bool settings_dirty() const { return settings_.dirty(); }
    [[nodiscard]] godot::Dictionary get_settings_report() const;
    [[nodiscard]] godot::String get_settings_path() const { return godot::String(settings_path_.c_str()); }

    // ---- credits ----
    [[nodiscard]] godot::String get_attribution_line() const;
    // [{title, entries: [{id, name, kind, provider, licence, licence_url,
    //    attribution_text, attribution_required, note}]}]
    [[nodiscard]] godot::Array get_credits_sections() const;
    [[nodiscard]] int get_credits_entry_count() const { return static_cast<int>(credits_.entries.size()); }

protected:
    static void _bind_methods();

private:
    godot::Dictionary transition_to_dict(const rg::ShellTransition& t) const;

    rg::ShellFlow flow_;
    rg::Settings settings_;
    rg::SettingsLoadReport settings_report_;
    rg::Credits credits_;
    std::optional<rg::SpawnPresetFile> presets_;
    std::string presets_problem_;
    std::string settings_path_;
    std::string last_drive_path_;
    int zone_ = 0;
    double e0_ = 0.0;
    double n0_ = 0.0;
    std::vector<rg::SpawnChoice> choices_; // the picker's last list (spawn_picked resolves ids against it)
};

} // namespace rg_godot
