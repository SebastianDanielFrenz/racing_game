// test_world_config.cpp — rg::load_world_config coverage (PLAN.md R2.0):
// happy path, ${VAR} expansion, the RG_G2M_HOME default-when-unset case, and
// every documented validation error path. Builds fixture JSON with
// nlohmann::json directly (same precedent as physics_sim's own
// test_terrain_golden.cpp, which links nlohmann_json into ps_unit_tests for
// exactly this reason - see tests/unit/CMakeLists.txt) rather than
// hand-editing raw JSON strings, so each error case only touches the one
// field it means to break.
//
// TOOL-019 note (vault, physics_sim): none of these cases use
// `catch (const X&) { ...e.what()... }` - load_world_config itself never
// throws (see world_config.cpp's own top comment), and every assertion below
// checks CHECK(!result.has_value()) plus the returned error STRING, never a
// caught exception's .what().

#include "rg/world_config.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace {

namespace fs = std::filesystem;
using json = nlohmann::json;

// Sets (or, one-argument form, ensures unset) an environment variable for
// the lifetime of this object, restoring whatever was there before on
// destruction - so one test's environment tampering never leaks into
// another. Portable: _putenv_s on Windows (an empty value string removes
// the variable, per its own documented contract), setenv/unsetenv on POSIX.
class ScopedEnvVar {
public:
    ScopedEnvVar(const char* name, const char* value) : name_(name) {
        capture_previous();
        set(value);
    }
    explicit ScopedEnvVar(const char* name) : name_(name) {
        capture_previous();
        clear();
    }
    ScopedEnvVar(const ScopedEnvVar&) = delete;
    ScopedEnvVar& operator=(const ScopedEnvVar&) = delete;
    ~ScopedEnvVar() {
        if (had_prev_) {
            set(prev_value_.c_str());
        } else {
            clear();
        }
    }

private:
    // Portable environment-variable read - see rg::world_config.cpp's own
    // safe_getenv comment: std::getenv is deprecated by the Windows UCRT
    // headers, and rg_warnings builds with /WX, which turns that
    // deprecation warning into a hard build failure under both clang-cl and
    // cl. _dupenv_s heap-allocates its own copy, freed immediately.
    static std::optional<std::string> safe_getenv(const std::string& name) {
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

    void capture_previous() {
        std::optional<std::string> prev = safe_getenv(name_);
        had_prev_ = prev.has_value();
        if (had_prev_) prev_value_ = *prev;
    }
    void set(const char* value) {
#ifdef _WIN32
        _putenv_s(name_.c_str(), value);
#else
        setenv(name_.c_str(), value, 1);
#endif
    }
    void clear() {
#ifdef _WIN32
        _putenv_s(name_.c_str(), "");
#else
        unsetenv(name_.c_str());
#endif
    }
    std::string name_;
    std::string prev_value_;
    bool had_prev_ = false;
};

// Writes text to a fresh temp file and returns its path; the file is
// removed when the returned guard is destroyed. Two distinctly-named
// constructing free functions (not two overloaded constructors) so an
// nlohmann::json argument can never accidentally resolve against a
// std::string parameter (or vice versa) via json's own implicit
// conversions.
class TempFile {
public:
    static TempFile from_json(const json& j) { return TempFile(j.dump(2)); }
    static TempFile from_raw_text(const std::string& raw_text) { return TempFile(raw_text); }

    ~TempFile() {
        std::error_code ec;
        fs::remove(path_, ec);
    }
    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;
    TempFile(TempFile&&) = default;

    [[nodiscard]] std::string path() const { return path_.string(); }

private:
    explicit TempFile(const std::string& text) {
        // Deliberately NOT the OS temp directory (std::filesystem::
        // temp_directory_path()): load_world_config resolves "surface_map"/
        // "palette" against the nearest ancestor CMakeLists.txt
        // (find_repo_root(), world_config.cpp) whenever they are relative,
        // which every fixture built by valid_world_config_json() is - a
        // fixture written outside any repo checkout (the OS temp dir) can
        // therefore never load successfully, no matter what it tests,
        // exactly the bug that made "${RG_G2M_HOME} is expanded"/"defaults
        // when unset" fail with cfg.has_value() == false even once the
        // separate cross-process filename collision below was fixed. Using
        // "<repo root>/out/test_tmp" instead (out/ is .gitignore'd, never
        // committed) means find_repo_root() succeeds immediately at
        // RG_SOURCE_DIR itself, matching how the real committed
        // data/world/world_config.json resolves.
        //
        // ctest_discover_tests registers each TEST_CASE as its own ctest
        // test, invoked as a fresh "rg_unit_tests.exe <test name>" PROCESS -
        // so a per-process atomic counter alone is not unique under
        // `ctest --parallel`: two test processes both starting their own
        // counter at 0 raced on the identical filename
        // "rg_world_config_test_0.json", corrupting each other's JSON
        // mid-read (found via 4 tests flaking under --parallel 8 - only the
        // ones asserting exact parse success/content, not just "failed with
        // a non-empty error string", exposed it). The process id makes the
        // path unique across processes; the counter still disambiguates
        // multiple TempFiles within one process/test.
        static std::atomic<int> counter{0};
#ifdef _WIN32
        const unsigned long pid = ::GetCurrentProcessId();
#else
        const long pid = static_cast<long>(::getpid());
#endif
        const fs::path dir = fs::path(RG_SOURCE_DIR) / "out" / "test_tmp";
        std::error_code mkdir_ec;
        fs::create_directories(dir, mkdir_ec);
        path_ = dir /
                ("rg_world_config_test_" + std::to_string(pid) + "_" +
                 std::to_string(counter.fetch_add(1)) + ".json");
        std::ofstream f(path_, std::ios::binary);
        f << text;
    }
    fs::path path_;
};

// The exact shape of data/world/world_config.json (kept in sync by hand -
// small enough that a drift would be caught by the happy-path test below
// failing against the real committed file).
json valid_world_config_json() {
    json j;
    j["format"] = "rg.world/1";
    j["region"] = "home";
    j["session_origin_utm"] = {{"zone", 32}, {"e0", 464000}, {"n0", 5559000}};
    j["source_store"] = {{"dir", "${RG_G2M_HOME}"}, {"scope", "home-2026-09r1"}, {"read_only", true}};
    j["derived_store"] = {{"dir", "${RG_G2M_HOME}"}, {"name", "derived"}};
    j["spawn"] = {{"e", 462500.0}, {"n", 5559500.0}, {"yaw_deg", 0}};
    j["surface_map"] = "data/world/landclass_surfaces.json";
    j["palette"] = "data/world/landclass_palette.json";
    return j;
}

} // namespace

TEST_CASE("load_world_config: the real committed data/world/world_config.json parses", "[world_config]") {
    ScopedEnvVar env("RG_G2M_HOME", "S:/some/test/g2m/home");
    const std::string path = std::string(RG_SOURCE_DIR) + "/data/world/world_config.json";
    std::string err;
    auto cfg = rg::load_world_config(path, &err);
    REQUIRE(cfg.has_value());
    CHECK(err.empty());
    CHECK(cfg->format == "rg.world/1");
    CHECK(cfg->region == "home");
    CHECK(cfg->session_origin_utm.zone == 32);
    CHECK(cfg->session_origin_utm.e0 == 464000.0);
    CHECK(cfg->session_origin_utm.n0 == 5559000.0);
    CHECK(cfg->source_store.scope == "home-2026-09r1");
    CHECK(cfg->source_store.read_only == true);
    CHECK(cfg->derived_store.name == "derived");
    CHECK(cfg->spawn.e == 462500.0);
    CHECK(cfg->spawn.n == 5559500.0);
    // R2.1 LOD-distance measurement sweep's chosen default (see repo
    // CLAUDE.md's measurement table) - the goal's own "visible terrain to
    // 16-20 km" is best met by 20000, and the measured warm build time at
    // that distance (~33 ms) is barely worse than at 6000 (~23 ms).
    CHECK(cfg->lod.max_distance_m == 20000.0);
}

TEST_CASE("load_world_config: ${RG_G2M_HOME} is expanded in dir fields", "[world_config]") {
    ScopedEnvVar env("RG_G2M_HOME", "S:/some/test/g2m/home");
    TempFile file = TempFile::from_json(valid_world_config_json());
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    REQUIRE(cfg.has_value());
    CHECK(cfg->source_store.dir == "S:/some/test/g2m/home");
    CHECK(cfg->derived_store.dir == "S:/some/test/g2m/home");
}

TEST_CASE("load_world_config: RG_G2M_HOME defaults to <repo root>/cache/g2m/home-r1 when unset", "[world_config]") {
    // R2.1 coordinator update (2026-09-26): the default moved from a fixed
    // S:\claude_code\geo2map_cache\home-r1 path to a repo-relative,
    // gitignored copy. TempFile writes under "<RG_SOURCE_DIR>/out/test_tmp",
    // so find_repo_root() resolves to RG_SOURCE_DIR itself here.
    ScopedEnvVar env("RG_G2M_HOME"); // ensure unset for this test
    ScopedEnvVar derived_env("RG_G2M_DERIVED"); // ensure unset: isolate this test from the override below
    TempFile file = TempFile::from_json(valid_world_config_json());
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    REQUIRE(cfg.has_value());
    const fs::path expected = (fs::path(std::string(RG_SOURCE_DIR)) / "cache" / "g2m" / "home-r1").lexically_normal();
    CHECK(fs::path(cfg->source_store.dir).lexically_normal() == expected);
    CHECK(fs::path(cfg->derived_store.dir).lexically_normal() == expected);
}

TEST_CASE("load_world_config: RG_G2M_DERIVED overrides derived_store.dir entirely", "[world_config]") {
    ScopedEnvVar env("RG_G2M_HOME", "S:/some/test/g2m/home");
    ScopedEnvVar derived_env("RG_G2M_DERIVED", "S:/some/other/scratch/derived");
    TempFile file = TempFile::from_json(valid_world_config_json());
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    REQUIRE(cfg.has_value());
    CHECK(cfg->source_store.dir == "S:/some/test/g2m/home"); // unaffected
    CHECK(cfg->derived_store.dir == "S:/some/other/scratch/derived");
}

TEST_CASE("load_world_config: empty RG_G2M_DERIVED does not override", "[world_config]") {
    ScopedEnvVar env("RG_G2M_HOME", "S:/some/test/g2m/home");
    ScopedEnvVar derived_env("RG_G2M_DERIVED", ""); // set but empty
    TempFile file = TempFile::from_json(valid_world_config_json());
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    REQUIRE(cfg.has_value());
    CHECK(cfg->derived_store.dir == "S:/some/test/g2m/home"); // the normal ${RG_G2M_HOME} expansion stands
}

TEST_CASE("load_world_config: surface_map/palette resolve against the config file's own repo root", "[world_config]") {
    ScopedEnvVar env("RG_G2M_HOME", "irrelevant-for-this-test");
    const std::string path = std::string(RG_SOURCE_DIR) + "/data/world/world_config.json";
    std::string err;
    auto cfg = rg::load_world_config(path, &err);
    REQUIRE(cfg.has_value());

    const fs::path expected_surface_map =
        fs::path(std::string(RG_SOURCE_DIR)) / "data" / "world" / "landclass_surfaces.json";
    const fs::path expected_palette =
        fs::path(std::string(RG_SOURCE_DIR)) / "data" / "world" / "landclass_palette.json";
    CHECK(fs::path(cfg->surface_map).lexically_normal() == expected_surface_map.lexically_normal());
    CHECK(fs::path(cfg->palette).lexically_normal() == expected_palette.lexically_normal());
    CHECK(fs::path(cfg->surface_map).is_absolute());
}

TEST_CASE("load_world_config: file not found", "[world_config]") {
    std::string err;
    auto cfg = rg::load_world_config(std::string(RG_SOURCE_DIR) + "/data/world/does_not_exist.json", &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: malformed JSON", "[world_config]") {
    TempFile file = TempFile::from_raw_text(std::string("{ this is not valid json"));
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: top level is not an object", "[world_config]") {
    TempFile file = TempFile::from_json(json::array({1, 2, 3}));
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: missing format", "[world_config]") {
    json j = valid_world_config_json();
    j.erase("format");
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: wrong format value", "[world_config]") {
    json j = valid_world_config_json();
    j["format"] = "rg.world/2";
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK(err.find("format") != std::string::npos);
}

TEST_CASE("load_world_config: missing region", "[world_config]") {
    json j = valid_world_config_json();
    j.erase("region");
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: empty region", "[world_config]") {
    json j = valid_world_config_json();
    j["region"] = "";
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: missing session_origin_utm", "[world_config]") {
    json j = valid_world_config_json();
    j.erase("session_origin_utm");
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: session_origin_utm is not an object", "[world_config]") {
    json j = valid_world_config_json();
    j["session_origin_utm"] = 32;
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: zone out of range (too high)", "[world_config]") {
    json j = valid_world_config_json();
    j["session_origin_utm"]["zone"] = 61;
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK(err.find("zone") != std::string::npos);
}

TEST_CASE("load_world_config: zone out of range (zero)", "[world_config]") {
    json j = valid_world_config_json();
    j["session_origin_utm"]["zone"] = 0;
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK(err.find("zone") != std::string::npos);
}

TEST_CASE("load_world_config: zone wrong type", "[world_config]") {
    json j = valid_world_config_json();
    j["session_origin_utm"]["zone"] = "32";
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: missing e0", "[world_config]") {
    json j = valid_world_config_json();
    j["session_origin_utm"].erase("e0");
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: missing source_store", "[world_config]") {
    json j = valid_world_config_json();
    j.erase("source_store");
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: source_store missing dir", "[world_config]") {
    json j = valid_world_config_json();
    j["source_store"].erase("dir");
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: source_store.read_only wrong type", "[world_config]") {
    json j = valid_world_config_json();
    j["source_store"]["read_only"] = "yes";
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: missing derived_store", "[world_config]") {
    json j = valid_world_config_json();
    j.erase("derived_store");
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: missing spawn", "[world_config]") {
    json j = valid_world_config_json();
    j.erase("spawn");
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: spawn.e wrong type", "[world_config]") {
    json j = valid_world_config_json();
    j["spawn"]["e"] = "not a number";
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: missing surface_map", "[world_config]") {
    json j = valid_world_config_json();
    j.erase("surface_map");
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: missing palette", "[world_config]") {
    json j = valid_world_config_json();
    j.erase("palette");
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: unknown placeholder is an error", "[world_config]") {
    json j = valid_world_config_json();
    j["source_store"]["dir"] = "${RG_DEFINITELY_NOT_A_REAL_ENV_VAR_XYZ}";
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK(err.find("RG_DEFINITELY_NOT_A_REAL_ENV_VAR_XYZ") != std::string::npos);
}

TEST_CASE("load_world_config: unterminated placeholder is an error", "[world_config]") {
    json j = valid_world_config_json();
    j["source_store"]["dir"] = "${RG_G2M_HOME";
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

// --- "lod" (PLAN.md R2.1: max_distance_m is a config value, not a hard-coded
// constant) ---

TEST_CASE("load_world_config: lod absent defaults to 6000.0", "[world_config]") {
    ScopedEnvVar env("RG_G2M_HOME", "S:/some/test/g2m/home");
    json j = valid_world_config_json();
    REQUIRE_FALSE(j.contains("lod"));
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    REQUIRE(cfg.has_value());
    CHECK(err.empty());
    CHECK(cfg->lod.max_distance_m == 6000.0);
}

TEST_CASE("load_world_config: lod.max_distance_m overrides the default", "[world_config]") {
    ScopedEnvVar env("RG_G2M_HOME", "S:/some/test/g2m/home");
    json j = valid_world_config_json();
    j["lod"] = {{"max_distance_m", 12000.0}};
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    REQUIRE(cfg.has_value());
    CHECK(err.empty());
    CHECK(cfg->lod.max_distance_m == 12000.0);
}

TEST_CASE("load_world_config: lod.max_distance_m must be > 0", "[world_config]") {
    ScopedEnvVar env("RG_G2M_HOME", "S:/some/test/g2m/home");
    json j = valid_world_config_json();
    j["lod"] = {{"max_distance_m", 0.0}};
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: lod.max_distance_m wrong type", "[world_config]") {
    ScopedEnvVar env("RG_G2M_HOME", "S:/some/test/g2m/home");
    json j = valid_world_config_json();
    j["lod"] = {{"max_distance_m", "far"}};
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}

TEST_CASE("load_world_config: lod is not an object", "[world_config]") {
    ScopedEnvVar env("RG_G2M_HOME", "S:/some/test/g2m/home");
    json j = valid_world_config_json();
    j["lod"] = 6000.0;
    TempFile file = TempFile::from_json(j);
    std::string err;
    auto cfg = rg::load_world_config(file.path(), &err);
    CHECK_FALSE(cfg.has_value());
    CHECK_FALSE(err.empty());
}
