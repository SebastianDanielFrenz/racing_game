#include "rg_shell.h"

#include "rg/world_config.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <cstdint>
#include <string>
#include <utility>
#include <variant>

using godot::D_METHOD;
using godot::String;

namespace rg_godot {

namespace {

std::string to_std(const String& s) { return std::string(s.utf8().get_data()); }
String from_std(const std::string& s) { return String::utf8(s.c_str()); }

const char* world_kind_name(rg::WorldKind k) { return k == rg::WorldKind::Flat ? "flat" : "real_world"; }

godot::Dictionary world_request_to_dict(const rg::WorldRequest& w) {
    godot::Dictionary d;
    d["kind"] = String(world_kind_name(w.kind));
    d["has_spawn"] = w.has_spawn;
    d["x"] = w.x;
    d["y"] = w.y;
    d["yaw_deg"] = w.yaw_deg;
    d["label"] = from_std(w.label);
    d["open_address_search"] = w.open_address_search;
    return d;
}

rg::WorldRequest dict_to_world_request(const godot::Dictionary& d) {
    rg::WorldRequest w;
    const std::string kind = to_std(String(d.get("kind", "flat")));
    w.kind = kind == "real_world" ? rg::WorldKind::RealWorld : rg::WorldKind::Flat;
    w.has_spawn = static_cast<bool>(d.get("has_spawn", false));
    w.x = static_cast<double>(d.get("x", 0.0));
    w.y = static_cast<double>(d.get("y", 0.0));
    w.yaw_deg = static_cast<double>(d.get("yaw_deg", 0.0));
    w.label = to_std(String(d.get("label", "")));
    w.open_address_search = static_cast<bool>(d.get("open_address_search", false));
    return w;
}

const char* action_kind_name(rg::ShellActionKind k) {
    switch (k) {
        case rg::ShellActionKind::ShowScreen: return "show_screen";
        case rg::ShellActionKind::LoadWorld: return "load_world";
        case rg::ShellActionKind::UnloadWorld: return "unload_world";
        case rg::ShellActionKind::SetPaused: return "set_paused";
        case rg::ShellActionKind::ResetCar: return "reset_car";
        case rg::ShellActionKind::SaveSettings: return "save_settings";
        case rg::ShellActionKind::SaveControls: return "save_controls";
        case rg::ShellActionKind::Quit: return "quit";
        case rg::ShellActionKind::OpenGarage: return "open_garage";
        case rg::ShellActionKind::CloseGarage: return "close_garage";
    }
    return "unknown";
}

godot::Array menu_to_array(const std::vector<rg::MenuItem>& items) {
    godot::Array out;
    for (const rg::MenuItem& m : items) {
        godot::Dictionary d;
        d["id"] = from_std(m.id);
        d["label"] = from_std(m.label);
        out.push_back(d);
    }
    return out;
}

godot::Variant setting_value_to_variant(const rg::SettingValue& v) {
    if (std::holds_alternative<bool>(v)) return godot::Variant(std::get<bool>(v));
    if (std::holds_alternative<std::int64_t>(v)) return godot::Variant(static_cast<int64_t>(std::get<std::int64_t>(v)));
    if (std::holds_alternative<double>(v)) return godot::Variant(std::get<double>(v));
    return godot::Variant(from_std(std::get<std::string>(v)));
}

const char* setting_type_name(rg::SettingType t) {
    switch (t) {
        case rg::SettingType::Bool: return "bool";
        case rg::SettingType::Int: return "int";
        case rg::SettingType::Float: return "float";
        case rg::SettingType::Choice: return "choice";
        case rg::SettingType::Path: return "path";
    }
    return "bool";
}

const char* spawn_kind_name(rg::SpawnChoiceKind k) {
    switch (k) {
        case rg::SpawnChoiceKind::Preset: return "preset";
        case rg::SpawnChoiceKind::LastPosition: return "last_position";
        case rg::SpawnChoiceKind::Flat: return "flat";
        case rg::SpawnChoiceKind::Address: return "address";
    }
    return "preset";
}

} // namespace

godot::Dictionary RgShell::initialize(const String& data_dir, const String& user_dir, const String& world_config_path) {
    godot::Dictionary out;
    godot::PackedStringArray problems;
    const std::string data = to_std(data_dir);
    const std::string user = to_std(user_dir);
    settings_path_ = user + "/settings.json";
    last_drive_path_ = user + "/last_drive.json";

    std::string err;
    if (auto credits = rg::load_credits(data + "/credits.json", &err)) {
        credits_ = std::move(*credits);
    } else {
        credits_ = rg::Credits{};
        problems.push_back(from_std(err));
    }

    presets_.reset();
    presets_problem_.clear();
    if (auto presets = rg::load_spawn_presets(data + "/world/spawn_presets.json", &err)) {
        presets_ = std::move(*presets);
        zone_ = presets_->zone;
        e0_ = presets_->e0;
        n0_ = presets_->n0;
    } else {
        presets_problem_ = err;
        problems.push_back(from_std(err));
    }
    // The world config is the authority on the session origin; a preset file
    // from another origin is then offered as "Default spawn" only.
    if (auto world = rg::load_world_config(to_std(world_config_path), &err)) {
        zone_ = world->session_origin_utm.zone;
        e0_ = world->session_origin_utm.e0;
        n0_ = world->session_origin_utm.n0;
    } else {
        problems.push_back(from_std(err));
    }

    settings_ = rg::Settings{};
    settings_report_ = settings_.load_file(settings_path_);
    out["problems"] = problems;
    out["settings_path"] = from_std(settings_path_);
    out["settings_file_missing"] = settings_report_.file_missing;
    out["attribution_line"] = from_std(rg::attribution_line(credits_));
    out["credits_entries"] = static_cast<int64_t>(credits_.entries.size());
    out["preset_count"] = static_cast<int64_t>(presets_ ? presets_->presets.size() : 0);
    return out;
}

// ---- flow --------------------------------------------------------------------

godot::Dictionary RgShell::transition_to_dict(const rg::ShellTransition& t) const {
    godot::Dictionary d;
    d["accepted"] = t.accepted;
    d["from"] = String(rg::to_string(t.from));
    d["to"] = String(rg::to_string(t.to));
    godot::Array actions;
    for (const rg::ShellAction& a : t.actions) {
        godot::Dictionary ad;
        ad["kind"] = String(action_kind_name(a.kind));
        ad["screen"] = String(rg::to_string(a.screen));
        ad["world"] = world_request_to_dict(a.world);
        ad["flag"] = a.flag;
        actions.push_back(ad);
    }
    d["actions"] = actions;
    return d;
}

String RgShell::get_screen() const { return String(rg::to_string(flow_.screen())); }
String RgShell::get_settings_return() const { return String(rg::to_string(flow_.settings_return())); }
String RgShell::get_controls_return() const { return String(rg::to_string(flow_.controls_return())); }
godot::Dictionary RgShell::get_world_request() const { return world_request_to_dict(flow_.world()); }
String RgShell::get_last_error() const { return from_std(flow_.last_error()); }
void RgShell::clear_last_error() { flow_.clear_error(); }

godot::Dictionary RgShell::boot_finished() { return transition_to_dict(flow_.handle(rg::ShellFlow::boot_finished())); }
godot::Dictionary RgShell::direct_start(const godot::Dictionary& world) {
    return transition_to_dict(flow_.handle(rg::ShellFlow::direct_start(dict_to_world_request(world))));
}
godot::Dictionary RgShell::menu_item(const String& id) { return transition_to_dict(flow_.handle(rg::ShellFlow::menu_item(to_std(id)))); }
godot::Dictionary RgShell::back() { return transition_to_dict(flow_.handle(rg::ShellFlow::back())); }
godot::Dictionary RgShell::load_ready() { return transition_to_dict(flow_.handle(rg::ShellFlow::load_ready())); }
godot::Dictionary RgShell::load_failed(const String& message) {
    return transition_to_dict(flow_.handle(rg::ShellFlow::load_failed(to_std(message))));
}
godot::Dictionary RgShell::load_cancelled() { return transition_to_dict(flow_.handle(rg::ShellFlow::load_cancelled())); }
godot::Dictionary RgShell::vehicle_chosen(const String& id) {
    return transition_to_dict(flow_.handle(rg::ShellFlow::vehicle_chosen(to_std(id))));
}
godot::Dictionary RgShell::garage_drive() { return transition_to_dict(flow_.handle(rg::ShellFlow::garage_drive())); }
String RgShell::get_garage_return() const { return String(rg::to_string(flow_.garage_return())); }
String RgShell::get_garage_vehicle() const { return from_std(flow_.garage_vehicle()); }
godot::Dictionary RgShell::pause_toggle() { return transition_to_dict(flow_.handle(rg::ShellFlow::pause_toggle())); }

godot::Dictionary RgShell::spawn_picked(const String& choice_id) {
    const std::string id = to_std(choice_id);
    for (const rg::SpawnChoice& c : choices_) {
        if (c.id != id) continue;
        if (!c.available) return transition_to_dict(rg::ShellTransition{}); // refused, nothing moves
        rg::WorldRequest w;
        w.kind = c.world;
        w.has_spawn = c.has_position;
        w.x = c.x;
        w.y = c.y;
        w.yaw_deg = c.yaw_deg;
        w.label = c.label;
        w.open_address_search = c.kind == rg::SpawnChoiceKind::Address;
        return transition_to_dict(flow_.handle(rg::ShellFlow::spawn_picked(std::move(w))));
    }
    return transition_to_dict(rg::ShellTransition{});
}

godot::Array RgShell::get_main_menu_items() const { return menu_to_array(rg::main_menu_items()); }
godot::Array RgShell::get_pause_menu_items() const { return menu_to_array(rg::pause_menu_items()); }

// ---- spawn picker ------------------------------------------------------------

godot::Array RgShell::get_spawn_choices() {
    std::string err;
    const std::optional<rg::LastDrive> last = rg::load_last_drive(last_drive_path_, &err);
    choices_ = rg::build_spawn_choices(presets_ ? &*presets_ : nullptr, last, zone_, e0_, n0_);
    godot::Array out;
    for (const rg::SpawnChoice& c : choices_) {
        godot::Dictionary d;
        d["id"] = from_std(c.id);
        d["kind"] = String(spawn_kind_name(c.kind));
        d["label"] = from_std(c.label);
        d["detail"] = from_std(c.detail);
        d["available"] = c.available;
        d["unavailable_reason"] = from_std(c.unavailable_reason);
        d["world"] = String(world_kind_name(c.world));
        d["has_position"] = c.has_position;
        d["x"] = c.x;
        d["y"] = c.y;
        d["yaw_deg"] = c.yaw_deg;
        out.push_back(d);
    }
    return out;
}

// ---- settings ----------------------------------------------------------------

godot::Array RgShell::get_settings_schema() const {
    godot::Array sections;
    for (const std::string& section : rg::settings_sections()) {
        godot::Dictionary sd;
        sd["id"] = from_std(section);
        sd["title"] = from_std(rg::settings_section_title(section));
        godot::Array list;
        for (const rg::SettingDef& def : rg::settings_schema()) {
            if (def.section != section || !def.shown) continue;
            godot::Dictionary d;
            d["key"] = from_std(def.key);
            d["label"] = from_std(def.label);
            d["help"] = from_std(def.help);
            d["type"] = String(setting_type_name(def.type));
            d["default"] = setting_value_to_variant(def.default_value);
            d["min"] = def.min;
            d["max"] = def.max;
            d["step"] = def.step;
            godot::PackedStringArray choices;
            for (const std::string& c : def.choices) choices.push_back(from_std(c));
            d["choices"] = choices;
            d["applies_live"] = def.applies_live;
            list.push_back(d);
        }
        sd["settings"] = list;
        sections.push_back(sd);
    }
    return sections;
}

godot::Variant RgShell::get_setting(const String& key) const { return setting_value_to_variant(settings_.get(to_std(key))); }
String RgShell::get_setting_display(const String& key) const { return from_std(settings_.display_value(to_std(key))); }

godot::Dictionary RgShell::set_setting(const String& key, const godot::Variant& value) {
    godot::Dictionary out;
    rg::SettingValue v;
    switch (value.get_type()) {
        case godot::Variant::BOOL: v = static_cast<bool>(value); break;
        case godot::Variant::INT: v = static_cast<std::int64_t>(static_cast<int64_t>(value)); break;
        case godot::Variant::FLOAT: v = static_cast<double>(value); break;
        case godot::Variant::STRING:
        case godot::Variant::STRING_NAME: v = to_std(String(value)); break;
        default:
            out["ok"] = false;
            out["error"] = String("unsupported value type");
            out["clamped"] = false;
            out["value"] = get_setting(key);
            return out;
    }
    std::string err;
    bool clamped = false;
    const bool ok = settings_.set(to_std(key), v, &err, &clamped);
    out["ok"] = ok;
    out["error"] = from_std(err);
    out["clamped"] = clamped;
    out["value"] = get_setting(key);
    return out;
}

void RgShell::reset_setting(const String& key) { settings_.reset(to_std(key)); }
void RgShell::reset_all_settings() { settings_.reset_all(); }

bool RgShell::save_settings() {
    if (settings_path_.empty()) return false;
    if (!settings_.dirty()) return true;
    std::string err;
    const bool ok = settings_.save_file(settings_path_, &err);
    if (!ok) godot::UtilityFunctions::push_error(String("settings: could not save: ") + from_std(err));
    return ok;
}

godot::Dictionary RgShell::get_settings_report() const {
    godot::Dictionary d;
    d["file_missing"] = settings_report_.file_missing;
    d["parse_error"] = settings_report_.parse_error;
    d["version_found"] = static_cast<int64_t>(settings_report_.version_found);
    d["migrated"] = settings_report_.migrated;
    d["from_newer"] = settings_report_.from_newer;
    godot::PackedStringArray unknown, invalid, clamped;
    for (const auto& k : settings_report_.unknown_keys) unknown.push_back(from_std(k));
    for (const auto& k : settings_report_.invalid_keys) invalid.push_back(from_std(k));
    for (const auto& k : settings_report_.clamped_keys) clamped.push_back(from_std(k));
    d["unknown_keys"] = unknown;
    d["invalid_keys"] = invalid;
    d["clamped_keys"] = clamped;
    return d;
}

// ---- credits -----------------------------------------------------------------

String RgShell::get_attribution_line() const { return from_std(rg::attribution_line(credits_)); }

godot::Array RgShell::get_credits_sections() const {
    godot::Array out;
    for (const rg::CreditsSection& s : rg::credits_sections(credits_)) {
        godot::Dictionary sd;
        sd["title"] = from_std(s.title);
        godot::Array entries;
        for (const rg::CreditEntry* e : s.entries) {
            godot::Dictionary d;
            d["id"] = from_std(e->id);
            d["name"] = from_std(e->name);
            d["kind"] = String(rg::credit_kind_name(e->kind));
            d["provider"] = from_std(e->provider);
            d["licence"] = from_std(e->licence);
            d["licence_url"] = from_std(e->licence_url);
            d["attribution_text"] = from_std(e->attribution_text);
            d["attribution_required"] = e->attribution_required;
            d["note"] = from_std(e->note);
            entries.push_back(d);
        }
        sd["entries"] = entries;
        out.push_back(sd);
    }
    return out;
}

void RgShell::_bind_methods() {
    using godot::ClassDB;
    ClassDB::bind_method(D_METHOD("initialize", "data_dir", "user_dir", "world_config_path"), &RgShell::initialize);
    ClassDB::bind_method(D_METHOD("get_screen"), &RgShell::get_screen);
    ClassDB::bind_method(D_METHOD("get_settings_return"), &RgShell::get_settings_return);
    ClassDB::bind_method(D_METHOD("get_controls_return"), &RgShell::get_controls_return);
    ClassDB::bind_method(D_METHOD("get_world_request"), &RgShell::get_world_request);
    ClassDB::bind_method(D_METHOD("get_last_error"), &RgShell::get_last_error);
    ClassDB::bind_method(D_METHOD("clear_last_error"), &RgShell::clear_last_error);
    ClassDB::bind_method(D_METHOD("boot_finished"), &RgShell::boot_finished);
    ClassDB::bind_method(D_METHOD("direct_start", "world"), &RgShell::direct_start);
    ClassDB::bind_method(D_METHOD("menu_item", "id"), &RgShell::menu_item);
    ClassDB::bind_method(D_METHOD("spawn_picked", "choice_id"), &RgShell::spawn_picked);
    ClassDB::bind_method(D_METHOD("back"), &RgShell::back);
    ClassDB::bind_method(D_METHOD("load_ready"), &RgShell::load_ready);
    ClassDB::bind_method(D_METHOD("load_failed", "message"), &RgShell::load_failed);
    ClassDB::bind_method(D_METHOD("load_cancelled"), &RgShell::load_cancelled);
    ClassDB::bind_method(D_METHOD("pause_toggle"), &RgShell::pause_toggle);
    ClassDB::bind_method(D_METHOD("vehicle_chosen", "id"), &RgShell::vehicle_chosen);
    ClassDB::bind_method(D_METHOD("garage_drive"), &RgShell::garage_drive);
    ClassDB::bind_method(D_METHOD("get_garage_return"), &RgShell::get_garage_return);
    ClassDB::bind_method(D_METHOD("get_garage_vehicle"), &RgShell::get_garage_vehicle);
    ClassDB::bind_method(D_METHOD("get_main_menu_items"), &RgShell::get_main_menu_items);
    ClassDB::bind_method(D_METHOD("get_pause_menu_items"), &RgShell::get_pause_menu_items);
    ClassDB::bind_method(D_METHOD("get_spawn_choices"), &RgShell::get_spawn_choices);
    ClassDB::bind_method(D_METHOD("get_settings_schema"), &RgShell::get_settings_schema);
    ClassDB::bind_method(D_METHOD("get_setting", "key"), &RgShell::get_setting);
    ClassDB::bind_method(D_METHOD("get_setting_display", "key"), &RgShell::get_setting_display);
    ClassDB::bind_method(D_METHOD("set_setting", "key", "value"), &RgShell::set_setting);
    ClassDB::bind_method(D_METHOD("reset_setting", "key"), &RgShell::reset_setting);
    ClassDB::bind_method(D_METHOD("reset_all_settings"), &RgShell::reset_all_settings);
    ClassDB::bind_method(D_METHOD("save_settings"), &RgShell::save_settings);
    ClassDB::bind_method(D_METHOD("settings_dirty"), &RgShell::settings_dirty);
    ClassDB::bind_method(D_METHOD("get_settings_report"), &RgShell::get_settings_report);
    ClassDB::bind_method(D_METHOD("get_settings_path"), &RgShell::get_settings_path);
    ClassDB::bind_method(D_METHOD("get_attribution_line"), &RgShell::get_attribution_line);
    ClassDB::bind_method(D_METHOD("get_credits_sections"), &RgShell::get_credits_sections);
    ClassDB::bind_method(D_METHOD("get_credits_entry_count"), &RgShell::get_credits_entry_count);
}

} // namespace rg_godot
