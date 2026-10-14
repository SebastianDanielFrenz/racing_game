// test_lod_settings.cpp — rg/lod_settings.h: the in-game LOD settings
// (data/settings/lod_settings.json), their schema rows and the geo2map
// parameters they turn into. Every default must reproduce the behaviour
// before the settings existed.
#include "rg/lod_settings.h"
#include "rg/settings.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <string>

using Catch::Approx;

namespace {

const std::string kRoot = RG_SOURCE_DIR;

rg::LodSettingsSpec shipped_spec() {
    std::string err;
    auto spec = rg::load_lod_settings_spec(kRoot + "/data/settings/lod_settings.json", &err);
    INFO(err);
    REQUIRE(spec.has_value());
    return *spec;
}

std::string replace(std::string text, const std::string& from, const std::string& to) {
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    return text.replace(at, from.size(), to);
}

const char* kMinimal = R"({"format": "rg.lod_settings/1", "section": {"id": "lod", "title": "LOD"},
  "settings": [
    {"key": "lod.view_distance_m", "type": "float", "label": "a", "default": 20000, "min": 4000, "max": 40000, "step": 1000},
    {"key": "lod.terrain_detail", "type": "float", "label": "b", "default": 1, "min": 0.5, "max": 2, "step": 0.1},
    {"key": "lod.road_detail", "type": "choice", "label": "c", "default": "standard", "choices": ["standard"]},
    {"key": "lod.map_detail", "type": "float", "label": "d", "default": 1, "min": 0.25, "max": 4, "step": 0.25}],
  "road_presets": {"standard": [{"min_class": "all"}]}})";

} // namespace

TEST_CASE("lod settings: the shipped file loads and its defaults are today's behaviour", "[lod_settings]") {
    const rg::LodSettingsSpec spec = shipped_spec();
    REQUIRE(spec.defs.size() == 4);
    // 'standard' is exactly geo2map's compiled render road policy, and the default choice.
    REQUIRE(spec.road_preset("standard") != nullptr);
    CHECK(*spec.road_preset("standard") == g2m::default_render_road_lod());
    for (int level = 0; level <= 8; ++level) {
        CHECK(g2m::render_road_lod_key(*spec.road_preset("standard"), level) ==
              g2m::render_road_lod_key(g2m::default_render_road_lod(), level));
    }
    const auto def_of = [&](const char* key) {
        const auto it = std::find_if(spec.defs.begin(), spec.defs.end(), [&](const rg::SettingDef& d) { return d.key == key; });
        REQUIRE(it != spec.defs.end());
        return *it;
    };
    CHECK(std::get<double>(def_of(rg::kLodViewDistanceKey).default_value) == Approx(20000.0));
    CHECK(std::get<double>(def_of(rg::kLodTerrainDetailKey).default_value) == Approx(1.0));
    CHECK(std::get<std::string>(def_of(rg::kLodRoadDetailKey).default_value) == "standard");
    CHECK(std::get<double>(def_of(rg::kLodMapDetailKey).default_value) == Approx(1.0));
    // The other presets really differ (otherwise the setting would be a placebo).
    for (const char* name : {"low", "high"}) {
        REQUIRE(spec.road_preset(name) != nullptr);
        CHECK_FALSE(rg::changed_road_lod_levels(spec.road_preset(name), nullptr, 6).empty());
    }
}

TEST_CASE("lod settings: defaults map to the unchanged geo2map parameters", "[lod_settings]") {
    const rg::LodSettingsSpec spec = shipped_spec();
    rg::LodChoice c;
    c.view_distance_m = 20000.0;
    c.terrain_detail = 1.0;
    c.road_detail = "standard";
    c.map_detail = 1.0;
    g2m::mesh::LodParams world;
    world.max_distance_m = 20000.0; // the shipped world configs
    const g2m::mesh::LodParams p = rg::terrain_lod_params(world, c);
    CHECK(p.range0_m == world.range0_m);
    CHECK(p.max_distance_m == world.max_distance_m);
    CHECK(p.max_level == world.max_level);
    CHECK(rg::changed_road_lod_levels(rg::road_lod_for(&spec, "standard").get(), nullptr, 10).empty());
    CHECK(rg::road_lod_for(&spec, "nope") == nullptr);
    CHECK(rg::road_lod_for(nullptr, "standard") == nullptr);
    const g2m::route::MapConfig mc = rg::map_config_for(c);
    for (std::size_t lod = 0; lod < g2m::route::kMapLodCount; ++lod) {
        const auto l = static_cast<g2m::route::MapLod>(lod);
        CHECK(g2m::route::map_lod_key(mc, l) == g2m::route::map_lod_key(g2m::route::default_map_config(), l));
    }
    // A neutral choice (no settings installed) changes nothing either.
    const g2m::mesh::LodParams q = rg::terrain_lod_params(world, rg::LodChoice{});
    CHECK(q.range0_m == world.range0_m);
    CHECK(q.max_distance_m == world.max_distance_m);
}

TEST_CASE("lod settings: non-default values reach the parameters", "[lod_settings]") {
    const rg::LodSettingsSpec spec = shipped_spec();
    rg::LodChoice c;
    c.view_distance_m = 8000.0;
    c.terrain_detail = 2.0;
    c.map_detail = 4.0;
    g2m::mesh::LodParams world;
    const g2m::mesh::LodParams p = rg::terrain_lod_params(world, c);
    CHECK(p.range0_m == Approx(500.0));
    CHECK(p.max_distance_m == Approx(8000.0));
    const g2m::route::MapConfig mc = rg::map_config_for(c);
    CHECK(mc.detail_scale == Approx(4.0));
    CHECK(g2m::route::map_lod_key(mc, g2m::route::MapLod::Mid) !=
          g2m::route::map_lod_key(g2m::route::default_map_config(), g2m::route::MapLod::Mid));
    // 'low' drops road classes at L1..L3 (L0 keeps every road), 'high' adds them at L4.
    const auto low = rg::changed_road_lod_levels(spec.road_preset("low"), nullptr, 6);
    CHECK(low == std::vector<int>{1, 2, 3});
    const auto high = rg::changed_road_lod_levels(spec.road_preset("high"), nullptr, 6);
    CHECK(std::find(high.begin(), high.end(), 0) == high.end());
    CHECK(std::find(high.begin(), high.end(), 4) != high.end());
    CHECK_FALSE(g2m::raster_params_for_render_level(4, g2m::default_render_road_lod()).has_value());
    CHECK(g2m::raster_params_for_render_level(4, *spec.road_preset("high")).has_value());
}

TEST_CASE("lod settings: installed rows join the settings schema and round-trip", "[lod_settings][settings]") {
    const rg::LodSettingsSpec spec = shipped_spec();
    rg::install_lod_settings(spec);
    rg::install_lod_settings(spec); // idempotent: replaced by key, never duplicated
    REQUIRE(rg::active_lod_settings() != nullptr);
    int count = 0;
    for (const rg::SettingDef& d : rg::settings_schema()) count += d.section == "lod" ? 1 : 0;
    CHECK(count == 4);
    const auto sections = rg::settings_sections();
    CHECK(std::find(sections.begin(), sections.end(), "lod") != sections.end());
    CHECK(rg::settings_section_title("lod") == "Level of detail");

    rg::Settings s;
    rg::LodChoice c = rg::lod_choice_from(s);
    CHECK(c.view_distance_m == Approx(20000.0));
    CHECK(c.road_detail == "standard");
    CHECK(s.display_value(rg::kLodViewDistanceKey) == "20000");
    CHECK(s.display_value(rg::kLodTerrainDetailKey) == "1.0");
    CHECK(s.set(rg::kLodRoadDetailKey, std::string("high")));
    CHECK_FALSE(s.set(rg::kLodRoadDetailKey, std::string("insane")));
    bool clamped = false;
    CHECK(s.set(rg::kLodViewDistanceKey, 99999.0, nullptr, &clamped));
    CHECK(clamped);
    CHECK(s.set(rg::kLodMapDetailKey, 2.0));
    rg::Settings back;
    const auto report = back.load_text(s.to_json());
    CHECK(report.invalid_keys.empty());
    c = rg::lod_choice_from(back);
    CHECK(c.road_detail == "high");
    CHECK(c.view_distance_m == Approx(40000.0));
    CHECK(c.map_detail == Approx(2.0));
}

TEST_CASE("lod settings: the loader refuses bad files", "[lod_settings]") {
    std::string err;
    CHECK(rg::parse_lod_settings_spec(kMinimal, "t", &err).has_value());
    CHECK_FALSE(rg::parse_lod_settings_spec("{}", "t", &err).has_value());
    CHECK_FALSE(rg::parse_lod_settings_spec(replace(kMinimal, "rg.lod_settings/1", "rg.lod_settings/2"), "t", &err));
    // A choice without a preset.
    CHECK_FALSE(rg::parse_lod_settings_spec(replace(kMinimal, R"(["standard"])", R"(["standard", "x"])"), "t", &err));
    CHECK(err.find("\"x\"") != std::string::npos);
    // An unknown road class.
    CHECK_FALSE(rg::parse_lod_settings_spec(replace(kMinimal, R"("min_class": "all")", R"("min_class": "runway")"), "t", &err));
    // Default outside the range.
    CHECK_FALSE(rg::parse_lod_settings_spec(replace(kMinimal, R"("default": 20000)", R"("default": 2000)"), "t", &err));
    // Map detail beyond geo2map's 0.05..20.
    CHECK_FALSE(rg::parse_lod_settings_spec(replace(kMinimal, R"("max": 4,)", R"("max": 40,)"), "t", &err));
    // A required key missing.
    CHECK_FALSE(rg::parse_lod_settings_spec(replace(kMinimal, "lod.map_detail", "lod.other"), "t", &err));
    // A key outside the section.
    CHECK_FALSE(rg::parse_lod_settings_spec(replace(kMinimal, "lod.view_distance_m", "graphics.view"), "t", &err));
}
