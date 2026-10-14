// rg/crops.h: deterministic crop choice, rotation by harvest year, growth
// stage by date, OSM crop=* override, fixed calendars, row direction, and
// data/world/crops.json.
#include "rg/crops.h"

#include <catch2/catch_test_macros.hpp>

#include <map>
#include <string>

namespace {
rg::CropConfig config() {
    std::string err;
    auto c = rg::load_crop_config(std::string(RG_SOURCE_DIR) + "/data/world/crops.json", &err);
    INFO(err);
    REQUIRE(c);
    return *c;
}
std::int64_t field_with(const rg::CropConfig& c, const std::string& crop, int year) {
    for (std::int64_t id = 1; id < 100000; ++id)
        if (rg::rotation_crop(c, id, year)->id == crop) return id;
    FAIL("no field found");
    return 0;
}
} // namespace

TEST_CASE("crops: data/world/crops.json loads", "[crops]") {
    const auto c = config();
    CHECK(c.crops.size() >= 10);
    CHECK(c.find("canola") == c.find("winter_rapeseed")); // alias
    std::string err;
    CHECK_FALSE(rg::load_crop_config(std::string(RG_SOURCE_DIR) + "/data/world/missing.json", &err));
    CHECK_FALSE(err.empty());
}

TEST_CASE("crops: same field, seed and date always give the same look", "[crops]") {
    const auto c = config();
    for (std::int64_t id : {1, 42, 123456789}) {
        const auto a = rg::field_look(c, id, "farmland", "", {2027, 6, 1});
        const auto b = rg::field_look(c, id, "farmland", "", {2027, 6, 1});
        REQUIRE(a);
        CHECK(a->crop_id == b->crop_id);
        CHECK(a->stage_id == b->stage_id);
    }
    // Another seed changes the layout.
    auto other = c;
    other.world_seed += 1;
    int differ = 0;
    for (std::int64_t id = 1; id <= 200; ++id)
        differ += rg::rotation_crop(c, id, 2027)->id != rg::rotation_crop(other, id, 2027)->id;
    CHECK(differ > 50);
}

TEST_CASE("crops: a field rotates by harvest year", "[crops]") {
    const auto c = config();
    const auto id = field_with(c, "winter_rapeseed", 2027);
    // The 3-crop rotation rapeseed -> wheat -> barley is the only one with rapeseed.
    CHECK(rg::rotation_crop(c, id, 2028)->id == "winter_wheat");
    CHECK(rg::rotation_crop(c, id, 2029)->id == "winter_barley");
    CHECK(rg::rotation_crop(c, id, 2030)->id == "winter_rapeseed");
}

TEST_CASE("crops: winter wheat through its season, sown before the harvest year", "[crops]") {
    const auto c = config();
    const auto id = field_with(c, "winter_wheat", 2027);
    struct Row {
        rg::CivilDate date;
        const char* stage;
    };
    for (const Row& r : {Row{{2026, 11, 15}, "emerged"}, Row{{2027, 3, 20}, "tillering"}, Row{{2027, 6, 10}, "heading"},
                         Row{{2027, 7, 25}, "ripe"}, Row{{2027, 8, 12}, "stubble"}}) {
        const auto look = rg::field_look(c, id, "farmland", "", r.date);
        REQUIRE(look);
        INFO(r.date.year << "-" << r.date.month << "-" << r.date.day);
        CHECK(look->crop_id == "winter_wheat");
        CHECK(look->harvest_year == 2027);
        CHECK(look->stage_id == r.stage);
    }
    const auto heading = rg::field_look(c, id, "farmland", "", {2027, 6, 10});
    CHECK(heading->height_m > 0.5);
    // After the season end the field belongs to the next crop of its rotation.
    const auto autumn = rg::field_look(c, id, "farmland", "", {2027, 10, 15});
    CHECK(autumn->harvest_year == 2028);
    CHECK(autumn->crop_id == rg::rotation_crop(c, id, 2028)->id);
}

TEST_CASE("crops: crop=* tag, fixed calendars, unknown landuse", "[crops]") {
    const auto c = config();
    const auto maize = rg::field_look(c, 7, "farmland", "maize", {2027, 8, 1});
    REQUIRE(maize);
    CHECK(maize->crop_id == "maize");
    CHECK(maize->stage_id == "tall");
    CHECK(maize->height_m > 2.0);
    CHECK(maize->row_spacing_m == 0.75);
    const auto meadow = rg::field_look(c, 7, "meadow", "", {2027, 5, 25});
    REQUIRE(meadow);
    CHECK(meadow->crop_id == "meadow");
    CHECK(meadow->stage_id == "tall");
    CHECK(rg::field_look(c, 7, "vineyard", "", {2027, 10, 5})->stage_id == "harvest");
    CHECK_FALSE(rg::field_look(c, 7, "industrial", "", {2027, 5, 25}));
    // An unknown crop tag falls back to the rotation.
    CHECK(rg::field_look(c, 7, "farmland", "quinoa", {2027, 6, 1})->crop_id == rg::field_look(c, 7, "farmland", "", {2027, 6, 1})->crop_id);
}

TEST_CASE("crops: crop shares follow the rotation weights", "[crops]") {
    const auto c = config();
    std::map<std::string, int> count;
    const int n = 20000;
    for (std::int64_t id = 1; id <= n; ++id) ++count[rg::rotation_crop(c, id, 2027)->id];
    const double wheat = double(count["winter_wheat"]) / n;
    CHECK(wheat > 0.25);
    CHECK(wheat < 0.45);
    CHECK(count["maize"] > 0);
    CHECK(count["sugar_beet"] > 0);
}

TEST_CASE("crops: row direction is the longest edge", "[crops]") {
    CHECK(rg::field_row_angle_deg({{0, 0}, {100, 0}, {100, 20}, {0, 20}}) == 0.0);
    const double a = rg::field_row_angle_deg({{0, 0}, {10, 10}, {5, 60}, {-5, 50}});
    CHECK(a > 90.0);  // edge (10,10)->(5,60) points up-left: ~95.7 deg
    CHECK(a < 100.0);
    CHECK(rg::field_row_angle_deg({{0, 0}}) == 0.0);
}
