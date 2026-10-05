// rg/spawn_presets.cpp - see spawn_presets.h.
#include "rg/spawn_presets.h"

#include "json_util.h"

#include <cmath>
#include <cstdio>
#include <set>

namespace rg {

namespace {

using detail::json;

bool fail(std::string* err, const std::string& origin, const std::string& message) {
    if (err != nullptr) *err = origin + ": " + message;
    return false;
}

bool valid_id(const std::string& s) {
    if (s.empty() || s.size() > 48) return false;
    for (const char c : s) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
        if (!ok) return false;
    }
    return true;
}

// Largest plausible |session coordinate|: the home world spans a few tens of
// km; 200 km catches unit mistakes (mm, degrees) without being tight.
constexpr double kMaxSessionCoord = 200000.0;

std::string fmt(const char* f, double a, double b = 0.0) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), f, a, b);
    return buf;
}

} // namespace

std::optional<SpawnPresetFile> parse_spawn_presets(const std::string& json_text, const std::string& origin,
                                                   std::string* err) {
    const std::optional<json> parsed = detail::parse_json(json_text);
    if (!parsed || !parsed->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    SpawnPresetFile out;
    if (!detail::get_string(*parsed, "format", out.format) || out.format != "rg.spawn_presets/1") {
        fail(err, origin, "\"format\" must be \"rg.spawn_presets/1\"");
        return std::nullopt;
    }
    const auto so = parsed->find("session_origin_utm");
    if (so == parsed->end() || !so->is_object()) {
        fail(err, origin, "\"session_origin_utm\" object missing");
        return std::nullopt;
    }
    double zone = 0.0;
    if (!detail::get_number(*so, "zone", zone) || zone < 1.0 || zone > 60.0 || zone != std::floor(zone) ||
        !detail::get_number(*so, "e0", out.e0) || !detail::get_number(*so, "n0", out.n0) || !std::isfinite(out.e0) ||
        !std::isfinite(out.n0)) {
        fail(err, origin, "\"session_origin_utm\" needs zone (1..60), e0, n0");
        return std::nullopt;
    }
    out.zone = static_cast<int>(zone);
    const auto presets = parsed->find("presets");
    if (presets == parsed->end() || !presets->is_array() || presets->empty()) {
        fail(err, origin, "\"presets\" must be a non-empty array");
        return std::nullopt;
    }
    std::set<std::string> ids;
    std::size_t index = 0;
    for (const json& p : *presets) {
        const std::string where = "presets[" + std::to_string(index++) + "]";
        if (!p.is_object()) {
            fail(err, origin, where + " is not an object");
            return std::nullopt;
        }
        SpawnPreset s;
        if (!detail::get_string(p, "id", s.id) || !valid_id(s.id)) {
            fail(err, origin, where + ": \"id\" must be lower_snake (a-z, 0-9, _)");
            return std::nullopt;
        }
        if (!ids.insert(s.id).second) {
            fail(err, origin, where + ": duplicate id \"" + s.id + "\"");
            return std::nullopt;
        }
        if (!detail::get_string(p, "name", s.name) || s.name.empty() || !detail::get_string(p, "description", s.description) ||
            !detail::get_string(p, "source", s.source) || s.source.empty()) {
            fail(err, origin, where + " (" + s.id + "): \"name\", \"description\" and \"source\" are required strings");
            return std::nullopt;
        }
        if (!detail::get_number(p, "x", s.x) || !detail::get_number(p, "y", s.y) ||
            !detail::get_number(p, "yaw_deg", s.yaw_deg) || !std::isfinite(s.x) || !std::isfinite(s.y) ||
            !std::isfinite(s.yaw_deg)) {
            fail(err, origin, where + " (" + s.id + "): finite \"x\", \"y\", \"yaw_deg\" required");
            return std::nullopt;
        }
        if (std::fabs(s.x) > kMaxSessionCoord || std::fabs(s.y) > kMaxSessionCoord) {
            fail(err, origin, where + " (" + s.id + "): x/y beyond +-200 km of the session origin (wrong unit?)");
            return std::nullopt;
        }
        if (s.yaw_deg <= -360.0 || s.yaw_deg > 360.0) {
            fail(err, origin, where + " (" + s.id + "): yaw_deg must be in (-360, 360]");
            return std::nullopt;
        }
        if (!detail::get_number(p, "measured_max_grade_pct", s.measured_max_grade_pct) ||
            !std::isfinite(s.measured_max_grade_pct) || s.measured_max_grade_pct < 0.0) {
            fail(err, origin, where + " (" + s.id + "): \"measured_max_grade_pct\" (>= 0) is required - every preset is slope-checked");
            return std::nullopt;
        }
        if (s.measured_max_grade_pct > kMaxSpawnGradePct) {
            fail(err, origin,
                 where + " (" + s.id + "): measured grade " + fmt("%.2f", s.measured_max_grade_pct) +
                     " % exceeds the " + fmt("%.0f", kMaxSpawnGradePct) + " % spawn limit");
            return std::nullopt;
        }
        out.presets.push_back(std::move(s));
    }
    return out;
}

std::optional<SpawnPresetFile> load_spawn_presets(const std::string& path, std::string* err) {
    const std::optional<std::string> text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot read file");
        return std::nullopt;
    }
    return parse_spawn_presets(*text, path, err);
}

std::string spawn_presets_match_world(const SpawnPresetFile& file, int zone, double e0, double n0) {
    if (file.zone == zone && std::fabs(file.e0 - e0) < 0.5 && std::fabs(file.n0 - n0) < 0.5) return {};
    return "spawn presets are in session origin zone " + std::to_string(file.zone) + " e0 " + fmt("%.0f", file.e0) +
           " n0 " + fmt("%.0f", file.n0) + ", the world config uses zone " + std::to_string(zone) + " e0 " +
           fmt("%.0f", e0) + " n0 " + fmt("%.0f", n0);
}

std::optional<LastDrive> parse_last_drive(const std::string& json_text, std::string* err) {
    const std::optional<json> parsed = detail::parse_json(json_text);
    if (!parsed || !parsed->is_object()) {
        fail(err, "last_drive", "not a JSON object");
        return std::nullopt;
    }
    std::string world;
    if (detail::get_string(*parsed, "world", world) && world != "real_world") {
        fail(err, "last_drive", "saved in the " + world + " world (no position to return to)");
        return std::nullopt;
    }
    const auto sm = parsed->find("session_m");
    if (sm == parsed->end() || !sm->is_array() || sm->size() < 2 || !(*sm)[0].is_number() || !(*sm)[1].is_number()) {
        fail(err, "last_drive", "\"session_m\" [x, y, z] missing");
        return std::nullopt;
    }
    LastDrive out;
    out.x = (*sm)[0].get<double>();
    out.y = (*sm)[1].get<double>();
    if (!std::isfinite(out.x) || !std::isfinite(out.y) || std::fabs(out.x) > kMaxSessionCoord ||
        std::fabs(out.y) > kMaxSessionCoord) {
        fail(err, "last_drive", "\"session_m\" is not a plausible session position");
        return std::nullopt;
    }
    double yaw = 0.0;
    if (detail::get_number(*parsed, "yaw_deg", yaw) && std::isfinite(yaw)) {
        out.yaw_deg = yaw;
        out.has_yaw = true;
    }
    double zone = 0.0;
    const auto um = parsed->find("utm_m");
    if (detail::get_number(*parsed, "utm_zone", zone) && um != parsed->end() && um->is_array() && um->size() >= 2 &&
        (*um)[0].is_number() && (*um)[1].is_number()) {
        out.zone = static_cast<int>(zone);
        out.e0 = (*um)[0].get<double>() - out.x;
        out.n0 = (*um)[1].get<double>() - out.y;
    }
    detail::get_string(*parsed, "saved_at", out.saved_at);
    return out;
}

std::optional<LastDrive> load_last_drive(const std::string& path, std::string* err) {
    const std::optional<std::string> text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot read file");
        return std::nullopt;
    }
    return parse_last_drive(*text, err);
}

std::vector<SpawnChoice> build_spawn_choices(const SpawnPresetFile* presets, const std::optional<LastDrive>& last,
                                             int zone, double e0, double n0) {
    std::vector<SpawnChoice> out;
    std::string presets_problem;
    if (presets != nullptr) presets_problem = spawn_presets_match_world(*presets, zone, e0, n0);
    if (presets != nullptr && presets_problem.empty()) {
        for (const SpawnPreset& p : presets->presets) {
            SpawnChoice c;
            c.kind = SpawnChoiceKind::Preset;
            c.id = p.id;
            c.label = p.name;
            c.detail = p.description;
            c.world = WorldKind::RealWorld;
            c.has_position = true;
            c.x = p.x;
            c.y = p.y;
            c.yaw_deg = p.yaw_deg;
            out.push_back(std::move(c));
        }
    } else {
        SpawnChoice c;
        c.kind = SpawnChoiceKind::Preset;
        c.id = "default_spawn";
        c.label = "Default spawn";
        c.detail = presets == nullptr ? "The world's own start point (the preset list is not available)."
                                      : "The world's own start point (the preset list does not match this world: " +
                                            presets_problem + ").";
        c.world = WorldKind::RealWorld;
        c.has_position = false;
        out.push_back(std::move(c));
    }

    SpawnChoice lp;
    lp.kind = SpawnChoiceKind::LastPosition;
    lp.id = "last_position";
    lp.label = "Last position";
    lp.world = WorldKind::RealWorld;
    if (!last.has_value()) {
        lp.available = false;
        lp.unavailable_reason = "No saved drive yet";
        lp.detail = lp.unavailable_reason;
    } else if (last->zone != 0 && (last->zone != zone || std::fabs(last->e0 - e0) > 0.5 || std::fabs(last->n0 - n0) > 0.5)) {
        lp.available = false;
        lp.unavailable_reason = "Saved in a different map origin";
        lp.detail = lp.unavailable_reason;
    } else {
        lp.has_position = true;
        lp.x = last->x;
        lp.y = last->y;
        lp.yaw_deg = last->has_yaw ? last->yaw_deg : 0.0;
        lp.detail = "Where you last drove" + (last->saved_at.empty() ? std::string() : " (" + last->saved_at + ")") +
                    (last->has_yaw ? std::string() : std::string("; this save has no heading, so the car faces east"));
    }
    out.push_back(std::move(lp));

    SpawnChoice ad;
    ad.kind = SpawnChoiceKind::Address;
    ad.id = "address";
    ad.label = "Search for an address";
    ad.detail = "Starts at the default spawn, then opens the address search (F10) to jump there.";
    ad.world = WorldKind::RealWorld;
    out.push_back(std::move(ad));

    SpawnChoice flat;
    flat.kind = SpawnChoiceKind::Flat;
    flat.id = "flat";
    flat.label = "Flat test world";
    flat.detail = "An endless flat plain. Loads instantly, no map data needed.";
    flat.world = WorldKind::Flat;
    out.push_back(std::move(flat));
    return out;
}

} // namespace rg
