// test_settings.cpp - rg::Settings (R5): schema, validation, versioned JSON
// load/save, migration, unknown-key preservation.
#include "rg/settings.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <string>

namespace {

using Catch::Approx;

std::string temp_path(const std::string& name) {
    return (std::filesystem::temp_directory_path() / ("rg_settings_test_" + name)).string();
}

bool contains(const std::vector<std::string>& v, const std::string& s) {
    return std::find(v.begin(), v.end(), s) != v.end();
}

} // namespace

TEST_CASE("settings: schema is consistent", "[settings]") {
    std::set<std::string> keys;
    for (const rg::SettingDef& d : rg::settings_schema()) {
        CAPTURE(d.key);
        CHECK(keys.insert(d.key).second);
        CHECK(d.key.rfind(d.section + ".", 0) == 0);
        CHECK_FALSE(d.label.empty());
        CHECK_FALSE(d.help.empty());
        // The default must itself pass validation.
        rg::Settings s;
        std::string err;
        bool clamped = true;
        CHECK(s.set(d.key, d.default_value, &err, &clamped));
        CHECK_FALSE(clamped);
        if (d.type == rg::SettingType::Float || d.type == rg::SettingType::Int) CHECK(d.min < d.max);
        if (d.type == rg::SettingType::Choice) {
            CHECK_FALSE(d.choices.empty());
            CHECK(std::get<std::string>(d.default_value).size() > 0);
        }
    }
    const auto sections = rg::settings_sections();
    CHECK(sections == std::vector<std::string>{"graphics", "camera", "audio", "map_data"});
    for (const auto& s : sections) CHECK(rg::settings_section_title(s) != s);
    CHECK(rg::find_setting("nope") == nullptr);
    REQUIRE(rg::find_setting("map_data.store_dir") != nullptr);
    CHECK_FALSE(rg::find_setting("map_data.store_dir")->applies_live);
}

TEST_CASE("settings: defaults and typed reads", "[settings]") {
    rg::Settings s;
    CHECK_FALSE(s.dirty());
    CHECK(s.get_string("graphics.window_mode") == "windowed");
    CHECK(s.get_bool("graphics.vsync"));
    CHECK(s.get_int("graphics.max_fps") == 0);
    CHECK(s.get_float("camera.fov_deg") == Approx(70.0));
    CHECK(s.get_float("audio.master_volume") == Approx(1.0));
    CHECK(s.get_string("map_data.store_dir").empty());
    // Unknown key: neutral values.
    CHECK_FALSE(s.get_bool("x.y"));
    CHECK(s.get_int("x.y") == 0);
    CHECK(s.get_string("x.y").empty());
    CHECK(s.display_value("graphics.max_fps") == "unlimited");
    CHECK(s.display_value("graphics.vsync") == "on");
    CHECK(s.display_value("audio.master_volume") == "1.00");
    CHECK(s.display_value("camera.fov_deg") == "70");
    CHECK(s.display_value("map_data.store_dir") == "(default)");
}

TEST_CASE("settings: set validates", "[settings]") {
    rg::Settings s;
    std::string err;
    bool clamped = false;
    CHECK_FALSE(s.set("nope", true, &err));
    CHECK(err.find("unknown") != std::string::npos);
    CHECK_FALSE(s.set("graphics.vsync", std::int64_t{1}, &err));
    CHECK_FALSE(s.set("graphics.window_mode", std::string("tiny"), &err));
    CHECK_FALSE(s.set("graphics.window_mode", true, &err));
    CHECK_FALSE(s.set("graphics.max_fps", 59.5, &err)); // not whole
    CHECK_FALSE(s.set("audio.master_volume", std::string("loud"), &err));
    CHECK_FALSE(s.set("camera.fov_deg", std::numeric_limits<double>::quiet_NaN(), &err));
    CHECK_FALSE(s.set("map_data.store_dir", std::string("a\nb"), &err));
    CHECK_FALSE(s.dirty());

    CHECK(s.set("graphics.max_fps", 120.0, &err, &clamped)); // integral double accepted
    CHECK(s.get_int("graphics.max_fps") == 120);
    CHECK_FALSE(clamped);
    CHECK(s.dirty());
    CHECK(s.set("graphics.max_fps", std::int64_t{9999}, &err, &clamped));
    CHECK(clamped);
    CHECK(s.get_int("graphics.max_fps") == 360);
    CHECK(s.set("audio.master_volume", -3.0, &err, &clamped));
    CHECK(clamped);
    CHECK(s.get_float("audio.master_volume") == Approx(0.0));
    CHECK(s.set("audio.engine_volume", std::int64_t{1}, &err)); // int for a float setting
    CHECK(s.get_float("audio.engine_volume") == Approx(1.0));
    CHECK(s.set("graphics.window_mode", std::string("fullscreen"), &err));
    CHECK(s.get_string("graphics.window_mode") == "fullscreen");

    s.reset("graphics.window_mode");
    CHECK(s.get_string("graphics.window_mode") == "windowed");
    s.reset_all();
    CHECK(s.get_int("graphics.max_fps") == 0);
}

TEST_CASE("settings: save and reload keeps values", "[settings]") {
    const std::string path = temp_path("roundtrip.json");
    std::filesystem::remove(path);
    rg::Settings a;
    REQUIRE(a.set("graphics.window_mode", std::string("fullscreen")));
    REQUIRE(a.set("graphics.vsync", false));
    REQUIRE(a.set("graphics.max_fps", std::int64_t{144}));
    REQUIRE(a.set("camera.fov_deg", 85.0));
    REQUIRE(a.set("audio.tyre_volume", 0.25));
    REQUIRE(a.set("map_data.store_dir", std::string("D:/maps/store")));
    std::string err;
    REQUIRE(a.save_file(path, &err));
    CHECK_FALSE(a.dirty());
    CHECK_FALSE(std::filesystem::exists(path + ".tmp"));

    rg::Settings b;
    const rg::SettingsLoadReport r = b.load_file(path);
    CHECK_FALSE(r.file_missing);
    CHECK_FALSE(r.parse_error);
    CHECK(r.version_found == 1);
    CHECK_FALSE(r.migrated);
    CHECK_FALSE(r.from_newer);
    CHECK(r.unknown_keys.empty());
    CHECK(r.invalid_keys.empty());
    CHECK(r.clamped_keys.empty());
    CHECK_FALSE(b.dirty());
    CHECK(b.get_string("graphics.window_mode") == "fullscreen");
    CHECK_FALSE(b.get_bool("graphics.vsync"));
    CHECK(b.get_int("graphics.max_fps") == 144);
    CHECK(b.get_float("camera.fov_deg") == Approx(85.0));
    CHECK(b.get_float("audio.tyre_volume") == Approx(0.25));
    CHECK(b.get_string("map_data.store_dir") == "D:/maps/store");
    CHECK(b.to_json() == a.to_json());
    std::filesystem::remove(path);
}

TEST_CASE("settings: missing file and broken documents give defaults", "[settings]") {
    rg::Settings s;
    REQUIRE(s.set("graphics.max_fps", std::int64_t{60}));
    const auto missing = s.load_file(temp_path("definitely_missing.json"));
    CHECK(missing.file_missing);
    CHECK(s.get_int("graphics.max_fps") == 0);

    CHECK(s.load_text("{ not json").parse_error);
    CHECK(s.load_text("[1,2]").parse_error);
    CHECK(s.get_int("graphics.max_fps") == 0);

    // No/invalid format string: read leniently, version 0.
    const auto noformat = s.load_text(R"({"graphics":{"max_fps":75}})");
    CHECK(noformat.version_found == 0);
    CHECK_FALSE(noformat.parse_error);
    CHECK(s.get_int("graphics.max_fps") == 75);
}

TEST_CASE("settings: invalid and clamped values are reported", "[settings]") {
    rg::Settings s;
    const auto r = s.load_text(R"({"format":"rg.settings/1",
        "graphics":{"window_mode":"tiny","vsync":"yes","max_fps":9000},
        "camera":{"fov_deg":10},
        "audio":{"master_volume":0.5}})");
    CHECK(contains(r.invalid_keys, "graphics.window_mode"));
    CHECK(contains(r.invalid_keys, "graphics.vsync"));
    CHECK(contains(r.clamped_keys, "graphics.max_fps"));
    CHECK(contains(r.clamped_keys, "camera.fov_deg"));
    CHECK(s.get_string("graphics.window_mode") == "windowed");
    CHECK(s.get_bool("graphics.vsync"));
    CHECK(s.get_int("graphics.max_fps") == 360);
    CHECK(s.get_float("camera.fov_deg") == Approx(50.0));
    CHECK(s.get_float("audio.master_volume") == Approx(0.5));
    CHECK(s.dirty()); // the file differs from what was applied
}

TEST_CASE("settings: unknown keys survive a round trip", "[settings]") {
    rg::Settings s;
    const auto r = s.load_text(R"({"format":"rg.settings/1",
        "graphics":{"vsync":false,"future_option":"x","nested":{"a":1}},
        "network":{"endpoint":"https://example.invalid"},
        "toplevel_extra":[1,2,3]})");
    CHECK(contains(r.unknown_keys, "graphics.future_option"));
    CHECK(contains(r.unknown_keys, "network.endpoint"));
    CHECK(contains(r.unknown_keys, "graphics.nested"));
    CHECK(contains(r.unknown_keys, "toplevel_extra"));
    CHECK_FALSE(s.get_bool("graphics.vsync"));
    const std::string out = s.to_json();
    rg::Settings t;
    const auto r2 = t.load_text(out);
    CHECK(r2.unknown_keys == r.unknown_keys);
    CHECK(t.to_json() == out);
    CHECK(out.find("future_option") != std::string::npos);
    CHECK(out.find("https://example.invalid") != std::string::npos);
    CHECK(out.find("toplevel_extra") != std::string::npos);
}

TEST_CASE("settings: a newer format is read but not destroyed", "[settings]") {
    rg::Settings s;
    const auto r = s.load_text(R"({"format":"rg.settings/7","graphics":{"max_fps":90},"extra":{"k":true}})");
    CHECK(r.from_newer);
    CHECK(r.version_found == 7);
    CHECK_FALSE(r.migrated);
    CHECK(s.get_int("graphics.max_fps") == 90);
    const std::string out = s.to_json();
    // Written back as this build's version, the unknown section intact.
    CHECK(out.find("rg.settings/1") != std::string::npos);
    CHECK(out.find("\"extra\"") != std::string::npos);
}

TEST_CASE("settings: migrations run on an older document", "[settings]") {
    // kSettingsFormatVersion is 1, so the only "older" document is one with
    // no readable format string (version 0). A test-only chain renames a key
    // and converts a unit, exercising the same machinery a future v2 will use.
    std::vector<rg::SettingsMigration> chain;
    chain.push_back({0, [](rg::SettingsDoc& d) {
                         const auto it = d.find("graphics.framerate_limit");
                         if (it != d.end()) {
                             d["graphics.max_fps"] = it->second;
                             d.erase(it);
                         }
                         const auto fov = d.find("camera.fov_rad");
                         if (fov != d.end()) {
                             d["camera.fov_deg"] = std::get<double>(fov->second) * 180.0 / 3.14159265358979;
                             d.erase(fov);
                         }
                     }});
    rg::Settings s;
    const auto r = s.load_text(R"({"graphics":{"framerate_limit":144},"camera":{"fov_rad":1.5707963267949}})", chain);
    CHECK(r.migrated);
    CHECK(r.version_found == 0);
    CHECK(s.get_int("graphics.max_fps") == 144);
    CHECK(s.get_float("camera.fov_deg") == Approx(90.0).margin(1e-6));
    CHECK(r.unknown_keys.empty());
    CHECK(s.dirty()); // the migrated form should be saved
    // Without the chain the old keys are just unknown and kept.
    rg::Settings t;
    const auto r2 = t.load_text(R"({"graphics":{"framerate_limit":144}})");
    CHECK_FALSE(r2.migrated);
    CHECK(contains(r2.unknown_keys, "graphics.framerate_limit"));
    CHECK(t.get_int("graphics.max_fps") == 0);
    CHECK(rg::default_settings_migrations().empty());
}
