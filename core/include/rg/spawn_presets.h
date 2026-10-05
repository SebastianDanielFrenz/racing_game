// rg/spawn_presets.h - the spawn picker's data and view-model (PLAN.md R5,
// 11.2 "spawn picker"). Engine-neutral; GDScript draws the choices this builds
// and passes the chosen one back to RgSimulation, it decides nothing.
//
// Three sources feed the picker:
//  - data/world/spawn_presets.json ("rg.spawn_presets/1"): named points in the
//    SESSION frame of the world config (x east / y north of session_origin_utm,
//    yaw 0 = east, counter-clockwise), each with the steepest 60 m grade ahead
//    that tools/route_check measured on the real store (recorded in the file,
//    re-measured by the hidden [.][realdata] test). The loader refuses a preset
//    steeper than kMaxSpawnGradePct and a file whose session origin differs
//    from the world config in use (spawn_presets_match_world).
//  - user://last_drive.json: the file main.gd already writes once a second
//    while driving the real world ("session_m", "utm_zone", "utm_m", and since
//    R5 "yaw_deg"); parse_last_drive() reads it back. An older file without
//    yaw_deg is still usable (the picker faces it east and says so).
//  - the flat test world (always available; no position).
// The address search is not a position source of its own: it needs a running
// world, so the picker offers it as "start at the default spawn, then open the
// address search" (SpawnChoiceKind::Address).
#pragma once

#include "rg/player_mode.h"

#include <optional>
#include <string>
#include <vector>

namespace rg {

// A preset steeper than this over the 60 m ahead is refused at load. Same
// 12 % as route_check's drive-route criterion.
inline constexpr double kMaxSpawnGradePct = 12.0;

struct SpawnPreset {
    std::string id;          // unique, lower_snake
    std::string name;        // shown in the picker
    std::string description; // one line under the name
    double x = 0.0;          // session-local metres east
    double y = 0.0;          // session-local metres north
    double yaw_deg = 0.0;    // 0 = east, counter-clockwise
    // Steepest grade (percent) over a 10 m window along the 60 m ahead of the
    // spawn, as measured by rg::check_route_on_world on the real store.
    double measured_max_grade_pct = 0.0;
    std::string source; // how the point was chosen / measured
};

struct SpawnPresetFile {
    std::string format; // "rg.spawn_presets/1"
    int zone = 0;
    double e0 = 0.0;
    double n0 = 0.0;
    std::vector<SpawnPreset> presets;
};

std::optional<SpawnPresetFile> load_spawn_presets(const std::string& path, std::string* err);
std::optional<SpawnPresetFile> parse_spawn_presets(const std::string& json_text, const std::string& origin,
                                                   std::string* err);

// "" when the file's session origin equals (zone, e0, n0), else a description.
std::string spawn_presets_match_world(const SpawnPresetFile& file, int zone, double e0, double n0);

// user://last_drive.json, parsed.
struct LastDrive {
    double x = 0.0; // session-local
    double y = 0.0;
    double yaw_deg = 0.0;
    bool has_yaw = false; // false: an older file; the picker faces east
    int zone = 0;         // 0 when the file has no UTM data
    double e0 = 0.0;      // utm_m - session_m: the origin the position was saved in
    double n0 = 0.0;
    std::string saved_at;
};
// nullopt + *err for unreadable/malformed text or a flat-world save (no real
// position to return to).
std::optional<LastDrive> parse_last_drive(const std::string& json_text, std::string* err);
std::optional<LastDrive> load_last_drive(const std::string& path, std::string* err);

enum class SpawnChoiceKind { Preset, LastPosition, Flat, Address };

struct SpawnChoice {
    SpawnChoiceKind kind = SpawnChoiceKind::Preset;
    std::string id;     // preset id, "last_position", "flat", "address"
    std::string label;
    std::string detail;
    bool available = true;
    std::string unavailable_reason; // when !available
    WorldKind world = WorldKind::RealWorld;
    bool has_position = false; // false: the world's own default spawn
    double x = 0.0;
    double y = 0.0;
    double yaw_deg = 0.0;
};

// The picker's list, in display order: the presets (file order), the last
// position, the address search, the flat world. `presets` may be null (file
// missing: the list then holds only the always-present entries plus the
// world's default spawn as "Default spawn"). `last` may be empty. A last drive
// saved in a different session origin than (zone, e0, n0) is listed
// unavailable with the reason instead of being silently shifted.
std::vector<SpawnChoice> build_spawn_choices(const SpawnPresetFile* presets, const std::optional<LastDrive>& last,
                                             int zone, double e0, double n0);

} // namespace rg
