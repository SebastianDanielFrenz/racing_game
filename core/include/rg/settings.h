// rg/settings.h — rg::Settings: the game's settings model (PLAN.md 11.1:
// "settings ... in rg_core"). Engine-neutral: a schema (key, type, range,
// default, label) plus a value store, strict validation, and a versioned JSON
// file ("rg.settings/1") with migration. GDScript builds its controls from
// settings_schema() and applies the values (window mode, fps cap, volumes,
// field of view); it decides nothing.
//
// Rule (owner, R5): no placebo controls. A key is in the schema only when the
// game acts on it. Today: graphics.window_mode / vsync / max_fps,
// camera.fov_deg, audio.master_volume / engine_volume / tyre_volume,
// map_data.store_dir. (Not here, because nothing consumes them yet: map-server
// endpoint, pin region, cache size - g2m_server does not exist; resolution
// scale / FSR - the project renders with the Compatibility renderer, which has
// no 3D scaling; traffic - the F7 panel and user://traffic.cfg stay the one
// traffic settings model.)
//
// File format (UTF-8 JSON):
//   {"format": "rg.settings/1", "graphics": {"vsync": true, ...}, "audio": {...}}
// Load policy (load_text): never fails the game.
//   - missing/unreadable file or a syntax error -> every default, reported;
//   - a key with a wrong type or a bad choice -> default, listed in
//     LoadReport::invalid_keys; a number out of range -> clamped, listed in
//     LoadReport::clamped_keys;
//   - an older "rg.settings/N" -> the document runs through the migration chain
//     (SettingsMigration table) first;
//   - a NEWER format than this build knows -> known keys are read, the rest is
//     kept and written back unchanged by to_json() (an older build must not
//     destroy a newer build's settings), LoadReport::from_newer = true;
//   - keys this build does not know (any version) are kept the same way and
//     listed in LoadReport::unknown_keys.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace rg {

inline constexpr int kSettingsFormatVersion = 1;

using SettingValue = std::variant<bool, std::int64_t, double, std::string>;

enum class SettingType { Bool, Int, Float, Choice, Path };

struct SettingDef {
    std::string key;     // "section.name"
    std::string section; // "graphics"
    std::string label;
    std::string help;
    SettingType type = SettingType::Bool;
    SettingValue default_value = false;
    double min = 0.0; // Int / Float
    double max = 0.0;
    double step = 0.0;
    std::vector<std::string> choices; // Choice
    // false: takes effect when the next world loads (store_dir), not live.
    bool applies_live = true;
};

// A loaded document before it meets the schema: dotted key ("section.name")
// -> scalar. Non-scalar members are kept aside as raw JSON text and are
// invisible to migrations.
using SettingsDoc = std::map<std::string, SettingValue>;

// Converts a document of version `from` into version `from + 1` in place.
struct SettingsMigration {
    int from = 0;
    std::function<void(SettingsDoc&)> apply;
};

// The migration chain of this build: empty, because "rg.settings/1" is the
// first format. A format change adds an entry here AND bumps
// kSettingsFormatVersion.
const std::vector<SettingsMigration>& default_settings_migrations();

struct SettingsLoadReport {
    bool file_missing = false;  // load_file only
    bool parse_error = false;   // syntax error or not an object: all defaults
    int version_found = 0;      // 0 = no/invalid "format"
    bool migrated = false;
    bool from_newer = false;
    std::vector<std::string> unknown_keys; // kept verbatim, written back by to_json()
    std::vector<std::string> invalid_keys; // replaced by their default
    std::vector<std::string> clamped_keys; // numeric, moved into range
};

// The schema, in display order (grouped by section).
const std::vector<SettingDef>& settings_schema();
const SettingDef* find_setting(const std::string& key);
// Section ids in schema order ("graphics", "camera", "audio", "map_data").
std::vector<std::string> settings_sections();
// Human title of a section.
std::string settings_section_title(const std::string& section);

class Settings {
public:
    Settings(); // every default

    // Typed reads. A key not in the schema returns the neutral value (false/0/"").
    [[nodiscard]] SettingValue get(const std::string& key) const;
    [[nodiscard]] bool get_bool(const std::string& key) const;
    [[nodiscard]] std::int64_t get_int(const std::string& key) const;
    [[nodiscard]] double get_float(const std::string& key) const;
    [[nodiscard]] std::string get_string(const std::string& key) const;

    // Validated write. False (and *err set) for an unknown key, a wrong type,
    // a Choice not in choices, or a Path with control characters / > 1024
    // bytes. A number outside [min, max] is clamped, not rejected (*clamped
    // set); Int accepts a double with an integral value. Marks the store dirty.
    bool set(const std::string& key, const SettingValue& value, std::string* err = nullptr, bool* clamped = nullptr);
    void reset(const std::string& key);
    void reset_all();

    // True when something changed since the last load/mark_saved.
    [[nodiscard]] bool dirty() const { return dirty_; }
    void mark_saved() { dirty_ = false; }

    // Persistence.
    SettingsLoadReport load_text(const std::string& json_text,
                                 const std::vector<SettingsMigration>& migrations = default_settings_migrations());
    SettingsLoadReport load_file(const std::string& path,
                                 const std::vector<SettingsMigration>& migrations = default_settings_migrations());
    // Pretty-printed rg.settings/1 document: schema values then preserved
    // unknown keys.
    [[nodiscard]] std::string to_json() const;
    // Writes to_json() atomically (temp file in the same directory, then
    // rename) and clears dirty. False + *err on an I/O failure.
    bool save_file(const std::string& path, std::string* err = nullptr);

    // "value as shown to the player" ("1.00", "on", "windowed").
    [[nodiscard]] std::string display_value(const std::string& key) const;

private:
    std::map<std::string, SettingValue> values_;
    std::map<std::string, std::string> extras_; // dotted key (or top-level key) -> raw JSON text
    bool dirty_ = false;
};

} // namespace rg
