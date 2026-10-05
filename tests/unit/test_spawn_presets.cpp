// test_spawn_presets.cpp - rg::SpawnPresets (R5): loader strictness, the
// world-origin check, last_drive parsing, the picker's choice list, and the
// shipped data/world/spawn_presets.json against data/world/world_config.json.
// The hidden [.][realdata] case re-measures every preset's slope on the real
// store with the same code tools/route_check runs.
#include "rg/spawn_presets.h"

#include "rg/route_check.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>
#include <optional>
#include <string>

namespace {

const std::string kRoot = RG_SOURCE_DIR;

std::string preset_json(const std::string& extra = "") {
    return R"({"id":"a_place","name":"A place","description":"d","source":"s","x":10.5,"y":-20.25,"yaw_deg":90,)"
           R"("measured_max_grade_pct":3.5)" +
           extra + "}";
}

std::string file_json(const std::string& presets, const std::string& fmt = "rg.spawn_presets/1",
                      const std::string& origin = R"({"zone":32,"e0":464000,"n0":5559000})") {
    return R"({"format":")" + fmt + R"(","session_origin_utm":)" + origin + R"(,"presets":[)" + presets + "]}";
}

bool rejects(const std::string& text, const std::string& needle) {
    std::string err;
    const auto f = rg::parse_spawn_presets(text, "t", &err);
    return !f.has_value() && err.find(needle) != std::string::npos;
}

} // namespace

TEST_CASE("spawn presets: a valid file loads", "[spawn]") {
    std::string err;
    const auto f = rg::parse_spawn_presets(file_json(preset_json()), "t", &err);
    INFO(err);
    REQUIRE(f.has_value());
    CHECK(f->zone == 32);
    CHECK(f->e0 == 464000.0);
    REQUIRE(f->presets.size() == 1);
    CHECK(f->presets[0].id == "a_place");
    CHECK(f->presets[0].x == 10.5);
    CHECK(f->presets[0].y == -20.25);
    CHECK(f->presets[0].yaw_deg == 90.0);
    CHECK(f->presets[0].measured_max_grade_pct == 3.5);
}

TEST_CASE("spawn presets: loader rejections", "[spawn]") {
    CHECK(rejects("nope", "not a JSON object"));
    CHECK(rejects(file_json(preset_json(), "rg.spawn_presets/2"), "format"));
    CHECK(rejects(R"({"format":"rg.spawn_presets/1","presets":[]})", "session_origin_utm"));
    CHECK(rejects(file_json(preset_json(), "rg.spawn_presets/1", R"({"zone":99,"e0":1,"n0":2})"), "zone"));
    CHECK(rejects(file_json(""), "non-empty"));
    CHECK(rejects(file_json("5"), "not an object"));
    CHECK(rejects(file_json(preset_json() + "," + preset_json()), "duplicate id"));
    // Field checks: replace a field by an invalid one.
    const auto with = [](const std::string& from, const std::string& to) {
        std::string p = preset_json();
        const auto pos = p.find(from);
        REQUIRE(pos != std::string::npos);
        p.replace(pos, from.size(), to);
        return file_json(p);
    };
    CHECK(rejects(with(R"("id":"a_place")", R"("id":"A Place")"), "lower_snake"));
    CHECK(rejects(with(R"("name":"A place",)", ""), "required strings"));
    CHECK(rejects(with(R"("source":"s",)", ""), "required strings"));
    CHECK(rejects(with(R"("x":10.5)", R"("x":"ten")"), "finite"));
    CHECK(rejects(with(R"("x":10.5)", R"("x":900000)"), "200 km"));
    CHECK(rejects(with(R"("yaw_deg":90)", R"("yaw_deg":720)"), "yaw_deg"));
    CHECK(rejects(with(R"("measured_max_grade_pct":3.5)", R"("measured_max_grade_pct":-1)"), "measured_max_grade_pct"));
    // The slope criterion is enforced at load.
    CHECK(rejects(with(R"("measured_max_grade_pct":3.5)", R"("measured_max_grade_pct":12.5)"), "exceeds"));
    std::string err;
    CHECK_FALSE(rg::load_spawn_presets(kRoot + "/data/world/none.json", &err).has_value());
    CHECK(err.find("cannot read") != std::string::npos);
}

TEST_CASE("spawn presets: the session origin must match the world config", "[spawn]") {
    std::string err;
    const auto f = rg::parse_spawn_presets(file_json(preset_json()), "t", &err);
    REQUIRE(f.has_value());
    CHECK(rg::spawn_presets_match_world(*f, 32, 464000.0, 5559000.0).empty());
    CHECK_FALSE(rg::spawn_presets_match_world(*f, 33, 464000.0, 5559000.0).empty());
    CHECK_FALSE(rg::spawn_presets_match_world(*f, 32, 465000.0, 5559000.0).empty());
    CHECK(rg::spawn_presets_match_world(*f, 32, 465000.0, 5559000.0).find("e0") != std::string::npos);
}

TEST_CASE("spawn presets: last_drive parsing", "[spawn]") {
    std::string err;
    // The file main.gd writes today (no yaw).
    const std::string legacy =
        R"({"saved_at":"2026-10-05T10:00:00","session_m":[2767.8,-7577.5,200.1],"world":"real_world",)"
        R"("utm_zone":32,"utm_m":[466767.8,5551422.5,200.1]})";
    const auto a = rg::parse_last_drive(legacy, &err);
    INFO(err);
    REQUIRE(a.has_value());
    CHECK(a->x == 2767.8);
    CHECK(a->y == -7577.5);
    CHECK_FALSE(a->has_yaw);
    CHECK(a->zone == 32);
    CHECK(std::fabs(a->e0 - 464000.0) < 1e-6);
    CHECK(std::fabs(a->n0 - 5559000.0) < 1e-6);
    CHECK(a->saved_at == "2026-10-05T10:00:00");
    // With a heading.
    const auto b = rg::parse_last_drive(R"({"session_m":[1,2,3],"world":"real_world","yaw_deg":-146.3})", &err);
    REQUIRE(b.has_value());
    CHECK(b->has_yaw);
    CHECK(b->yaw_deg == -146.3);
    CHECK(b->zone == 0); // no UTM data
    // Rejections.
    CHECK_FALSE(rg::parse_last_drive("garbage", &err).has_value());
    CHECK_FALSE(rg::parse_last_drive(R"({"world":"real_world"})", &err).has_value());
    CHECK_FALSE(rg::parse_last_drive(R"({"session_m":[1,2,3],"world":"flat"})", &err).has_value());
    CHECK(err.find("flat") != std::string::npos);
    CHECK_FALSE(rg::parse_last_drive(R"({"session_m":["a",2,3]})", &err).has_value());
    CHECK_FALSE(rg::parse_last_drive(R"({"session_m":[9e9,2,3]})", &err).has_value());
    CHECK_FALSE(rg::load_last_drive(kRoot + "/data/none.json", &err).has_value());
}

TEST_CASE("spawn presets: the picker's choice list", "[spawn]") {
    std::string err;
    const auto presets = rg::parse_spawn_presets(file_json(preset_json()), "t", &err);
    REQUIRE(presets.has_value());
    rg::LastDrive last;
    last.x = 5;
    last.y = 6;
    last.yaw_deg = 45;
    last.has_yaw = true;
    last.zone = 32;
    last.e0 = 464000;
    last.n0 = 5559000;

    auto choices = rg::build_spawn_choices(&*presets, last, 32, 464000.0, 5559000.0);
    REQUIRE(choices.size() == 4); // 1 preset, last position, address, flat
    CHECK(choices[0].kind == rg::SpawnChoiceKind::Preset);
    CHECK(choices[0].id == "a_place");
    CHECK(choices[0].has_position);
    CHECK(choices[0].world == rg::WorldKind::RealWorld);
    CHECK(choices[1].kind == rg::SpawnChoiceKind::LastPosition);
    CHECK(choices[1].available);
    CHECK(choices[1].x == 5.0);
    CHECK(choices[1].yaw_deg == 45.0);
    CHECK(choices[2].kind == rg::SpawnChoiceKind::Address);
    CHECK(choices[2].available);
    CHECK_FALSE(choices[2].has_position);
    CHECK(choices[3].kind == rg::SpawnChoiceKind::Flat);
    CHECK(choices[3].world == rg::WorldKind::Flat);
    CHECK(choices[3].available);
    for (const auto& c : choices) CHECK_FALSE(c.label.empty());

    // No saved drive: listed, unavailable, with the reason.
    choices = rg::build_spawn_choices(&*presets, std::nullopt, 32, 464000.0, 5559000.0);
    CHECK_FALSE(choices[1].available);
    CHECK_FALSE(choices[1].unavailable_reason.empty());
    CHECK_FALSE(choices[1].has_position);

    // A save from another origin is not silently shifted.
    last.e0 = 400000;
    choices = rg::build_spawn_choices(&*presets, last, 32, 464000.0, 5559000.0);
    CHECK_FALSE(choices[1].available);
    CHECK(choices[1].unavailable_reason.find("origin") != std::string::npos);

    // An older save without a heading faces east and says so.
    rg::LastDrive legacy;
    legacy.x = 1;
    legacy.y = 2;
    choices = rg::build_spawn_choices(&*presets, legacy, 32, 464000.0, 5559000.0);
    CHECK(choices[1].available);
    CHECK(choices[1].yaw_deg == 0.0);
    CHECK(choices[1].detail.find("east") != std::string::npos);

    // Presets for another world origin or no preset file: only a default spawn.
    choices = rg::build_spawn_choices(&*presets, std::nullopt, 32, 1.0, 2.0);
    CHECK(choices[0].id == "default_spawn");
    CHECK_FALSE(choices[0].has_position);
    CHECK(choices[0].detail.find("does not match") != std::string::npos);
    choices = rg::build_spawn_choices(nullptr, std::nullopt, 32, 464000.0, 5559000.0);
    CHECK(choices[0].id == "default_spawn");
    CHECK(choices.size() == 4);
}

TEST_CASE("spawn presets: the shipped file matches the shipped world config", "[spawn]") {
    std::string err;
    const auto world = rg::load_world_config(kRoot + "/data/world/world_config.json", &err);
    INFO(err);
    REQUIRE(world.has_value());
    const auto presets = rg::load_spawn_presets(kRoot + "/data/world/spawn_presets.json", &err);
    INFO(err);
    REQUIRE(presets.has_value());
    const auto& o = world->session_origin_utm;
    CHECK(rg::spawn_presets_match_world(*presets, o.zone, o.e0, o.n0).empty());
    CHECK(presets->presets.size() >= 4);
    // The first preset is the world config's own spawn.
    const double sx = world->spawn.e - o.e0;
    const double sy = world->spawn.n - o.n0;
    CHECK(std::fabs(presets->presets[0].x - sx) < 0.05);
    CHECK(std::fabs(presets->presets[0].y - sy) < 0.05);
    CHECK(std::fabs(presets->presets[0].yaw_deg - world->spawn.yaw_deg) < 0.05);
    for (const auto& p : presets->presets) {
        CAPTURE(p.id);
        CHECK(p.measured_max_grade_pct <= rg::kMaxSpawnGradePct);
    }
}

// Real data (hidden): every preset, driven 60 m ahead along its own heading,
// must pass route_check's grade limit on the real store, and the recorded
// number must match the measurement. SKIPs when the store cannot be opened.
TEST_CASE("spawn presets: every preset is drivable on the real store", "[.][realdata][spawn]") {
    std::string err;
    const auto world = rg::load_world_config(kRoot + "/data/world/world_config.json", &err);
    REQUIRE(world.has_value());
    const auto presets = rg::load_spawn_presets(kRoot + "/data/world/spawn_presets.json", &err);
    REQUIRE(presets.has_value());
    std::unique_ptr<rg::WorldTerrain> terrain = rg::WorldTerrain::open(*world, &err);
    if (terrain == nullptr) {
        SKIP("real map data store not available: " + err);
    }
    for (const auto& p : presets->presets) {
        CAPTURE(p.id);
        const double yaw = p.yaw_deg * 3.14159265358979323846 / 180.0;
        std::vector<rg::RoutePoint> pts;
        for (double s = 0.0; s <= 60.0 + 1e-9; s += 10.0) pts.push_back({p.x + std::cos(yaw) * s, p.y + std::sin(yaw) * s});
        rg::RouteCheckParams params;
        params.min_length_m = 50.0;
        params.min_seam_crossings = 0;
        params.min_corner_radius_m = 0.0;
        params.max_grade = rg::kMaxSpawnGradePct / 100.0;
        rg::Route route;
        route.format = "rg.route/1";
        route.name = p.id;
        route.zone = world->session_origin_utm.zone;
        route.e0 = world->session_origin_utm.e0;
        route.n0 = world->session_origin_utm.n0;
        route.spawn = {p.x, p.y};
        route.spawn_yaw_deg = p.yaw_deg;
        route.waypoints = pts;
        rg::WorldConfig w = *world;
        w.spawn.e = world->session_origin_utm.e0 + p.x;
        w.spawn.n = world->session_origin_utm.n0 + p.y;
        w.spawn.yaw_deg = p.yaw_deg;
        const rg::RouteCheckReport r = rg::check_route_on_world(route, w, *terrain, params);
        INFO(rg::format_route_report(r));
        CHECK(r.ok());
        CHECK(r.nodata_samples == 0);
        UNSCOPED_INFO("preset " << p.id << " measured max grade " << r.max_grade * 100.0 << " % (file " << p.measured_max_grade_pct << ")");
        CHECK(std::fabs(r.max_grade * 100.0 - p.measured_max_grade_pct) < 0.1);
    }
}
