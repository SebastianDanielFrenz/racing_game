// rg/lod_settings.h — in-game level-of-detail settings (owner rule: LOD
// aggressiveness, distance and detail are in-game settings; the game passes
// them to geo2map on every call, geo2map docs/routing_design.md 6.1).
//
// Engine-neutral. data/settings/lod_settings.json ("rg.lod_settings/1") holds
// the four settings (key, label, range, default) and the road-detail presets;
// install_lod_settings() adds them to rg::Settings' schema, so the settings
// screen lists them like every other row. The helpers below turn the stored
// values into the geo2map parameters:
//   lod.view_distance_m  -> g2m::mesh::LodParams::max_distance_m
//   lod.terrain_detail   -> LodParams::range0_m = the world's range0_m * value
//   lod.road_detail      -> a g2m::RenderRoadLod preset (ClassLookup::lod)
//   lod.map_detail       -> g2m::route::MapConfig::detail_scale
// Every default reproduces the behaviour before these settings existed.
#pragma once

#include "rg/settings.h"

#include "g2m/layer/osm_roads.h"
#include "g2m/mesh/lod_select.h"
#include "g2m/route/route_map.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rg {

inline constexpr const char* kLodViewDistanceKey = "lod.view_distance_m";
inline constexpr const char* kLodTerrainDetailKey = "lod.terrain_detail";
inline constexpr const char* kLodRoadDetailKey = "lod.road_detail";
inline constexpr const char* kLodMapDetailKey = "lod.map_detail";

struct LodSettingsSpec {
    std::string section = "lod";
    std::string section_title;
    std::vector<SettingDef> defs; // the four keys above, in display order
    // Road-detail presets by name (every lod.road_detail choice has one).
    std::vector<std::pair<std::string, g2m::RenderRoadLod>> road_presets;
    [[nodiscard]] const g2m::RenderRoadLod* road_preset(const std::string& name) const;
};

// Strict, exception-free. Presets name road classes ("all" = rank 0, else a
// highway value of g2m::RoadStyle::default_style()); null = no road classes at
// that chunk level; levels past the list have none. nullopt + *err on any error.
std::optional<LodSettingsSpec> parse_lod_settings_spec(const std::string& json_text, const std::string& origin,
                                                       std::string* err);
std::optional<LodSettingsSpec> load_lod_settings_spec(const std::string& path, std::string* err);

// Adds the spec's settings to the schema (extend_settings_schema) and makes it
// the process' active spec. Call before the Settings is loaded.
void install_lod_settings(const LodSettingsSpec& spec);
// The installed spec, or null (no LOD settings: every consumer keeps its defaults).
std::shared_ptr<const LodSettingsSpec> active_lod_settings();

// The values as the consumers take them. Defaults = no change.
struct LodChoice {
    double view_distance_m = 0.0; // 0 = keep the world's own max_distance_m
    double terrain_detail = 1.0;
    std::string road_detail;      // preset name; empty or unknown = g2m::default_render_road_lod()
    double map_detail = 1.0;
    friend bool operator==(const LodChoice&, const LodChoice&) = default;
};
// Reads the four keys; a key missing from the schema keeps its LodChoice default.
LodChoice lod_choice_from(const Settings& settings);

// The world's LodParams with the choice applied (range0_m scaled, max_distance_m replaced when > 0).
g2m::mesh::LodParams terrain_lod_params(const g2m::mesh::LodParams& world, const LodChoice& choice);
// The road policy of the choice (a copy of the preset), or null = the default policy.
std::shared_ptr<const g2m::RenderRoadLod> road_lod_for(const LodSettingsSpec* spec, const std::string& preset);
// g2m::route::default_map_config() with detail_scale = choice.map_detail (clamped to geo2map's 0.05..20).
g2m::route::MapConfig map_config_for(const LodChoice& choice);
// Chunk levels 0..max_level whose road-class content differs between two policies (null = default).
std::vector<int> changed_road_lod_levels(const g2m::RenderRoadLod* a, const g2m::RenderRoadLod* b, int max_level);

} // namespace rg
