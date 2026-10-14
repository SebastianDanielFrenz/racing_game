// rg/crops.h - deterministic field looks: which crop grows on an OSM farmland
// polygon in which growth stage on a given date. Engine-neutral; the renderer
// (Godot now, Unreal after the port) only maps the returned FieldLook (crop,
// stage, height, colour, ground cover, row spacing and direction) onto its
// materials / foliage. Data: data/world/crops.json (rg.crops/1).
//
// Rule: a farmland field follows a rotation picked by hash(way id, world seed)
// (weighted); its crop for harvest year Y is sequence[(hash2 + Y) mod n]. The
// harvest year changes on the day the current crop's season ends (harvest and
// cultivation done), so autumn-sown crops show up in autumn of the year before
// their harvest. A per-field date jitter spreads sowing/harvest over days.
// Meadow, orchard, vineyard etc. follow one fixed calendar each. An OSM
// crop=* tag naming a known crop (or alias) overrides the rotation.
#pragma once

#include "rg/sky.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rg {

struct CropStage {
    std::string id;
    int start_doy = 1;          // day of year this stage starts (non-leap numbering)
    double height_m = 0;        // plant height
    double ground_cover = 0;    // 0 bare soil .. 1 closed canopy
    std::array<double, 3> rgb{0.4, 0.3, 0.2}; // dominant colour seen from a distance
};

struct CropDef {
    std::string id, label;
    std::vector<std::string> aliases; // OSM crop=* values
    double row_spacing_m = 0.125;
    int season_end_doy = 240;         // field free for the next crop from this day
    std::vector<CropStage> stages;    // ascending start_doy, wraps over New Year
};

struct Rotation {
    double weight = 1;
    std::vector<std::string> sequence; // crop ids
};

struct CropConfig {
    std::uint64_t world_seed = 0;
    int date_jitter_days = 0;
    std::vector<CropDef> crops;
    std::vector<Rotation> rotations;                              // for landuse=farmland
    std::vector<std::pair<std::string, std::string>> fixed_landuse; // OSM landuse/natural value -> crop id
    const CropDef* find(const std::string& id_or_alias) const;
};
std::optional<CropConfig> load_crop_config(const std::string& path, std::string* error);

struct FieldLook {
    std::string crop_id, crop_label, stage_id;
    int harvest_year = 0;
    double height_m = 0, ground_cover = 0, row_spacing_m = 0;
    std::array<double, 3> rgb{0, 0, 0};
    double row_angle_deg = 0; // left 0 by field_look; the caller sets it from field_row_angle_deg(polygon)
};

// The field's look on a local date. `landuse` is the OSM landuse (or natural)
// value ("farmland", "meadow", ...); `crop_tag` the crop=* value or "".
// nullopt: no rule for that landuse.
std::optional<FieldLook> field_look(const CropConfig& config, std::int64_t way_id, const std::string& landuse,
                                    const std::string& crop_tag, const CivilDate& date);

// Row direction of a field polygon (session x/y metres): the direction of its
// longest edge, degrees in [0, 180). 0 for fewer than two points.
double field_row_angle_deg(const std::vector<std::array<double, 2>>& polygon);

// The crop a farmland field grows in a given harvest year (no tag override).
const CropDef* rotation_crop(const CropConfig& config, std::int64_t way_id, int harvest_year);

} // namespace rg
