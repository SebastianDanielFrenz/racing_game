// rg/world_config.cpp — see world_config.h for the public contract.
//
// Deliberately exception-free throughout (unlike physics_sim's own io
// loaders, e.g. core/src/io/vehicle_io.cpp's "catch (const
// nlohmann::json::parse_error& e) { ...e.what()... }" pattern): vault
// TOOL-019 documents that under the Windows clang-cl "asan" preset, a
// function that catches an exception by reference/value and then touches it
// (e.g. e.what()) reads garbage off the ASan-instrumented stack and crashes.
// A unit test that deliberately feeds this loader malformed JSON (one of the
// required "each error path" tests) would hit exactly that crash if this
// file used the same catch-and-.what() pattern. Instead: nlohmann::json is
// parsed with allow_exceptions=false + is_discarded(), and every field is
// checked with .contains()/.is_*() before any .get<T>() call - no throw, no
// catch, anywhere in this file.
#include "rg/world_config.h"

#include "g2m/core/geo/utm.h"

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>

namespace rg {

namespace {

namespace fs = std::filesystem;
using json = nlohmann::json;

// RG_G2M_HOME's default, when unset (R2.1 coordinator update, 2026-09-26 -
// supersedes the earlier fixed S:\claude_code\geo2map_cache\home-r1
// default): a repo-relative, gitignored copy of the source store at
// "<repo root>/cache/g2m/home-r1" (the external geo2map_cache directory
// stays untouched and read-only; see .gitignore's "cache/" entry and
// racing_game/CLAUDE.md's layout section for how that copy is populated).
// "Repo root" is the same nearest-ancestor-with-CMakeLists.txt search
// find_repo_root() below already does for surface_map/palette resolution -
// computed once per load_world_config() call and threaded through to
// expand_placeholders(). Only RG_G2M_HOME gets a default; every other
// "${NAME}" placeholder is an error if the environment variable is not set.
constexpr const char* kDefaultRgG2mHomeRelative = "cache/g2m/home-r1";

// RG_G2M_DERIVED (checked directly, not a "${NAME}" placeholder): when set,
// overrides derived_store.dir entirely (used as-is, no further expansion)
// after the normal "${RG_G2M_HOME}/derived" expansion below - a cheap escape
// hatch for a test/CI run wanting the derived store somewhere other than
// alongside RG_G2M_HOME, independent of the default-relocation above.
constexpr const char* kDerivedStoreOverrideEnvVar = "RG_G2M_DERIVED";

bool fail(std::string* err, const std::string& path, const std::string& message) {
    if (err != nullptr) {
        *err = path + ": " + message;
    }
    return false;
}

// Portable environment-variable read. Avoids std::getenv directly: the
// Windows UCRT headers mark it deprecated in favour of _dupenv_s, and
// rg_warnings builds with /WX (-Werror), which turns that deprecation
// warning into a hard build failure under both clang-cl and cl. _dupenv_s
// heap-allocates its own copy, which is freed immediately after copying it
// into the returned std::string.
std::optional<std::string> safe_getenv(const std::string& name) {
#ifdef _WIN32
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name.c_str()) != 0 || buf == nullptr) {
        return std::nullopt;
    }
    std::string value(buf);
    std::free(buf);
    return value;
#else
    const char* value = std::getenv(name.c_str());
    return value != nullptr ? std::optional<std::string>(value) : std::nullopt;
#endif
}

// Resolves `path` to an absolute path without throwing (weakly_canonical's
// error_code overload never throws; on any error this just falls back to
// the plain absolute path, uncanonicalised).
fs::path to_absolute(const std::string& path) {
    std::error_code ec;
    fs::path p(path);
    if (!p.is_absolute()) {
        fs::path cwd = fs::current_path(ec);
        if (!ec) {
            p = cwd / p;
        }
    }
    ec.clear();
    fs::path canon = fs::weakly_canonical(p, ec);
    return ec ? p : canon;
}

// Nearest ancestor of `dir` containing a CMakeLists.txt file, or nullopt if
// none is found before reaching the filesystem root (see world_config.h's
// doc comment for why this, and not e.g. ".git", is the chosen marker: a
// config file always lives inside a CMake-based repo checkout on this
// project, and both racing_game and geo2map_engine (and physics_sim) each
// have exactly one top-level CMakeLists.txt).
std::optional<fs::path> find_repo_root(fs::path dir) {
    std::error_code ec;
    while (true) {
        std::error_code exists_ec;
        if (fs::exists(dir / "CMakeLists.txt", exists_ec) && !exists_ec) {
            return dir;
        }
        fs::path parent = dir.parent_path();
        if (parent.empty() || parent == dir) {
            return std::nullopt;
        }
        dir = parent;
    }
}

// Expands every "${NAME}" placeholder in `in`. Returns nullopt (with *err
// set) on an unterminated placeholder, an unknown/unset variable other than
// RG_G2M_HOME, or (RG_G2M_HOME specifically, env var unset) no repo_root to
// build its default from.
std::optional<std::string> expand_placeholders(const std::string& in, const std::string& path,
                                               const std::optional<fs::path>& repo_root, std::string* err) {
    std::string out;
    out.reserve(in.size());
    std::size_t i = 0;
    while (i < in.size()) {
        if (in[i] == '$' && i + 1 < in.size() && in[i + 1] == '{') {
            const std::size_t close = in.find('}', i + 2);
            if (close == std::string::npos) {
                fail(err, path, "unterminated \"${\" placeholder in \"" + in + "\"");
                return std::nullopt;
            }
            const std::string name = in.substr(i + 2, close - (i + 2));
            if (name.empty()) {
                fail(err, path, "empty \"${}\" placeholder in \"" + in + "\"");
                return std::nullopt;
            }
            std::string value;
            if (name == "RG_G2M_HOME") {
                std::optional<std::string> env = safe_getenv(name);
                if (env.has_value()) {
                    value = *env;
                } else if (repo_root.has_value()) {
                    value = (*repo_root / fs::path(kDefaultRgG2mHomeRelative)).lexically_normal().string();
                } else {
                    fail(err, path,
                         "RG_G2M_HOME is not set and no repo root (nearest ancestor with a CMakeLists.txt) "
                         "was found to build its default (\"" +
                             std::string(kDefaultRgG2mHomeRelative) + "\") from");
                    return std::nullopt;
                }
            } else {
                std::optional<std::string> env = safe_getenv(name);
                if (!env.has_value()) {
                    fail(err, path, "unknown placeholder \"${" + name + "}\" (environment variable not set)");
                    return std::nullopt;
                }
                value = *env;
            }
            out += value;
            i = close + 1;
        } else {
            out += in[i];
            ++i;
        }
    }
    return out;
}

// --- Small typed-field helpers. Every one fails (returns false, sets *err)
// rather than throwing if the key is missing or the wrong JSON type.

bool require_object(const json& j, const std::string& path, const std::string& what, std::string* err) {
    if (!j.is_object()) {
        fail(err, path, what + " must be a JSON object");
        return false;
    }
    return true;
}

bool get_string(const json& obj, const char* key, const std::string& path, const std::string& what,
                std::string* out, std::string* err) {
    if (!obj.contains(key)) {
        fail(err, path, what + ": missing required field \"" + key + "\"");
        return false;
    }
    const json& v = obj.at(key);
    if (!v.is_string()) {
        fail(err, path, what + ": field \"" + key + "\" must be a string");
        return false;
    }
    *out = v.get<std::string>();
    return true;
}

bool get_bool(const json& obj, const char* key, const std::string& path, const std::string& what,
              bool* out, std::string* err) {
    if (!obj.contains(key)) {
        fail(err, path, what + ": missing required field \"" + key + "\"");
        return false;
    }
    const json& v = obj.at(key);
    if (!v.is_boolean()) {
        fail(err, path, what + ": field \"" + key + "\" must be a bool");
        return false;
    }
    *out = v.get<bool>();
    return true;
}

bool get_number(const json& obj, const char* key, const std::string& path, const std::string& what,
                double* out, std::string* err) {
    if (!obj.contains(key)) {
        fail(err, path, what + ": missing required field \"" + key + "\"");
        return false;
    }
    const json& v = obj.at(key);
    if (!v.is_number()) {
        fail(err, path, what + ": field \"" + key + "\" must be a number");
        return false;
    }
    *out = v.get<double>();
    return true;
}

bool get_int(const json& obj, const char* key, const std::string& path, const std::string& what,
             int* out, std::string* err) {
    if (!obj.contains(key)) {
        fail(err, path, what + ": missing required field \"" + key + "\"");
        return false;
    }
    const json& v = obj.at(key);
    if (!v.is_number_integer()) {
        fail(err, path, what + ": field \"" + key + "\" must be an integer");
        return false;
    }
    *out = v.get<int>();
    return true;
}

} // namespace

std::optional<WorldConfig> load_world_config(const std::string& path, std::string* err) {
    if (err != nullptr) {
        err->clear();
    }

    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        fail(err, path, "cannot open file");
        return std::nullopt;
    }
    std::ostringstream contents_stream;
    contents_stream << file.rdbuf();
    const std::string text = contents_stream.str();

    json root = json::parse(text, /*cb=*/nullptr, /*allow_exceptions=*/false);
    if (root.is_discarded()) {
        fail(err, path, "malformed JSON");
        return std::nullopt;
    }
    if (!require_object(root, path, "top level", err)) {
        return std::nullopt;
    }

    std::string format;
    if (!get_string(root, "format", path, "top level", &format, err)) {
        return std::nullopt;
    }
    if (format != "rg.world/1") {
        fail(err, path, "unsupported \"format\" (expected \"rg.world/1\", got \"" + format + "\")");
        return std::nullopt;
    }

    WorldConfig cfg;
    cfg.format = format;

    // Repo root (nearest ancestor of the CONFIG FILE with a CMakeLists.txt):
    // computed once, used both for RG_G2M_HOME's default (below) and for
    // surface_map/palette resolution (bottom of this function). A config
    // file with no repo root above it can still load successfully as long as
    // it never actually needs one - RG_G2M_HOME set explicitly and both
    // surface_map/palette already absolute - so this is looked up eagerly
    // but only turned into an error at the point something actually needs it
    // (expand_placeholders / the surface_map+palette block).
    const fs::path config_file_abs = to_absolute(path);
    const std::optional<fs::path> repo_root = find_repo_root(config_file_abs.parent_path());

    if (!get_string(root, "region", path, "top level", &cfg.region, err)) {
        return std::nullopt;
    }
    if (cfg.region.empty()) {
        fail(err, path, "\"region\" must not be empty");
        return std::nullopt;
    }

    // --- session_origin_utm ---
    if (!root.contains("session_origin_utm")) {
        fail(err, path, "missing required field \"session_origin_utm\"");
        return std::nullopt;
    }
    const json& origin = root.at("session_origin_utm");
    if (!require_object(origin, path, "\"session_origin_utm\"", err)) {
        return std::nullopt;
    }
    if (!get_int(origin, "zone", path, "\"session_origin_utm\"", &cfg.session_origin_utm.zone, err)) {
        return std::nullopt;
    }
    // Real use of geo2map_engine, not just a link-smoke reference: the same
    // 1..60 range g2m::geo::UtmZone::valid() enforces (hemisphere is not
    // part of this schema - session_origin_utm is always northern-hemisphere
    // on this project's home region, PLAN.md 15.1).
    if (!g2m::geo::UtmZone{cfg.session_origin_utm.zone}.valid()) {
        fail(err, path, "\"session_origin_utm.zone\" out of range (must be 1..60, got " +
                            std::to_string(cfg.session_origin_utm.zone) + ")");
        return std::nullopt;
    }
    if (!get_number(origin, "e0", path, "\"session_origin_utm\"", &cfg.session_origin_utm.e0, err)) {
        return std::nullopt;
    }
    if (!get_number(origin, "n0", path, "\"session_origin_utm\"", &cfg.session_origin_utm.n0, err)) {
        return std::nullopt;
    }

    // --- source_store ---
    if (!root.contains("source_store")) {
        fail(err, path, "missing required field \"source_store\"");
        return std::nullopt;
    }
    const json& source_store = root.at("source_store");
    if (!require_object(source_store, path, "\"source_store\"", err)) {
        return std::nullopt;
    }
    std::string source_dir_raw;
    if (!get_string(source_store, "dir", path, "\"source_store\"", &source_dir_raw, err)) {
        return std::nullopt;
    }
    auto source_dir_expanded = expand_placeholders(source_dir_raw, path, repo_root, err);
    if (!source_dir_expanded.has_value()) {
        return std::nullopt;
    }
    cfg.source_store.dir = *source_dir_expanded;
    if (!get_string(source_store, "scope", path, "\"source_store\"", &cfg.source_store.scope, err)) {
        return std::nullopt;
    }
    if (!get_bool(source_store, "read_only", path, "\"source_store\"", &cfg.source_store.read_only, err)) {
        return std::nullopt;
    }

    // --- derived_store ---
    if (!root.contains("derived_store")) {
        fail(err, path, "missing required field \"derived_store\"");
        return std::nullopt;
    }
    const json& derived_store = root.at("derived_store");
    if (!require_object(derived_store, path, "\"derived_store\"", err)) {
        return std::nullopt;
    }
    std::string derived_dir_raw;
    if (!get_string(derived_store, "dir", path, "\"derived_store\"", &derived_dir_raw, err)) {
        return std::nullopt;
    }
    auto derived_dir_expanded = expand_placeholders(derived_dir_raw, path, repo_root, err);
    if (!derived_dir_expanded.has_value()) {
        return std::nullopt;
    }
    cfg.derived_store.dir = *derived_dir_expanded;
    // RG_G2M_DERIVED: a cheap, direct (non-"${NAME}") override of the whole
    // derived_store.dir value, used as-is (no further expansion) - see
    // kDerivedStoreOverrideEnvVar's doc comment above.
    if (std::optional<std::string> derived_override = safe_getenv(kDerivedStoreOverrideEnvVar);
        derived_override.has_value() && !derived_override->empty()) {
        cfg.derived_store.dir = *derived_override;
    }
    if (!get_string(derived_store, "name", path, "\"derived_store\"", &cfg.derived_store.name, err)) {
        return std::nullopt;
    }

    // --- spawn ---
    if (!root.contains("spawn")) {
        fail(err, path, "missing required field \"spawn\"");
        return std::nullopt;
    }
    const json& spawn = root.at("spawn");
    if (!require_object(spawn, path, "\"spawn\"", err)) {
        return std::nullopt;
    }
    if (!get_number(spawn, "e", path, "\"spawn\"", &cfg.spawn.e, err)) {
        return std::nullopt;
    }
    if (!get_number(spawn, "n", path, "\"spawn\"", &cfg.spawn.n, err)) {
        return std::nullopt;
    }
    if (!get_number(spawn, "yaw_deg", path, "\"spawn\"", &cfg.spawn.yaw_deg, err)) {
        return std::nullopt;
    }

    // --- lod (optional; absent -> WorldConfig::Lod's own default) ---
    if (root.contains("lod")) {
        const json& lod = root.at("lod");
        if (!require_object(lod, path, "\"lod\"", err)) {
            return std::nullopt;
        }
        if (lod.contains("max_distance_m")) {
            if (!get_number(lod, "max_distance_m", path, "\"lod\"", &cfg.lod.max_distance_m, err)) {
                return std::nullopt;
            }
            if (cfg.lod.max_distance_m <= 0.0) {
                fail(err, path, "\"lod.max_distance_m\" must be > 0 (got " +
                                    std::to_string(cfg.lod.max_distance_m) + ")");
                return std::nullopt;
            }
        }
    }

    // --- surface_map / palette: repo-root-relative (see world_config.h) ---
    std::string surface_map_raw;
    if (!get_string(root, "surface_map", path, "top level", &surface_map_raw, err)) {
        return std::nullopt;
    }
    std::string palette_raw;
    if (!get_string(root, "palette", path, "top level", &palette_raw, err)) {
        return std::nullopt;
    }

    fs::path resolved_surface_map(surface_map_raw);
    fs::path resolved_palette(palette_raw);
    if (!resolved_surface_map.is_absolute() || !resolved_palette.is_absolute()) {
        if (!repo_root.has_value()) {
            fail(err, path,
                 "cannot resolve repo-root-relative paths (\"surface_map\"/\"palette\"): "
                 "no CMakeLists.txt found in any ancestor of \"" +
                     config_file_abs.parent_path().string() + "\"");
            return std::nullopt;
        }
        if (!resolved_surface_map.is_absolute()) {
            resolved_surface_map = *repo_root / resolved_surface_map;
        }
        if (!resolved_palette.is_absolute()) {
            resolved_palette = *repo_root / resolved_palette;
        }
    }
    cfg.surface_map = resolved_surface_map.lexically_normal().string();
    cfg.palette = resolved_palette.lexically_normal().string();

    return cfg;
}

} // namespace rg
