// rg/wet_look.h: data/world/wet_look.json and the physics weather -> wet look mapping (visuals).
#include "rg/wet_look.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

namespace {
rg::WetLookConfig load() {
    std::string err;
    auto c = rg::load_wet_look_config(std::string(RG_SOURCE_DIR) + "/data/world/wet_look.json", &err);
    INFO(err);
    REQUIRE(c);
    return *c;
}
} // namespace

TEST_CASE("wet look: shipped config loads", "[wet_look]") {
    const auto c = load();
    CHECK(c.wet_full_mm > 0.0);
    CHECK(c.puddle_full_mm > c.puddle_start_mm);
    CHECK(c.rain_full_mm_h > c.rain_min_mm_h);
}

TEST_CASE("wet look: loader refuses bad files", "[wet_look]") {
    const std::string dir = std::string(RG_SOURCE_DIR) + "/out/test_tmp";
    std::filesystem::create_directories(dir);
    const std::string path = dir + "/wet_look_bad.json";
    std::string err;
    CHECK_FALSE(rg::load_wet_look_config(path + ".missing", &err));
    auto write = [&](const std::string& body) {
        std::ofstream(path, std::ios::binary) << body;
        err.clear();
        return rg::load_wet_look_config(path, &err);
    };
    const std::string ok_tail = R"("rain_full_mm_h": 8, "rain_min_mm_h": 0.05, "snow_below_c": 1, "response_s": 2})";
    CHECK(write(R"({"format": "rg.wet_look/1", "wet_full_mm": 0.1, "puddle_start_mm": 0.3, "puddle_full_mm": 1.5, )" + ok_tail));
    CHECK_FALSE(write(R"({"format": "rg.wet_look/2", "wet_full_mm": 0.1, "puddle_start_mm": 0.3, "puddle_full_mm": 1.5, )" + ok_tail));
    CHECK_FALSE(write(R"({"format": "rg.wet_look/1", "puddle_start_mm": 0.3, "puddle_full_mm": 1.5, )" + ok_tail));
    CHECK(err.find("wet_full_mm") != std::string::npos);
    CHECK_FALSE(write(R"({"format": "rg.wet_look/1", "wet_full_mm": 0.1, "puddle_start_mm": 2, "puddle_full_mm": 1.5, )" + ok_tail));
    CHECK_FALSE(write(R"({"format": "rg.wet_look/1", "wet_full_mm": -1, "puddle_start_mm": 0.3, "puddle_full_mm": 1.5, )" + ok_tail));
}

TEST_CASE("wet look: dry, damp, wet with puddles, rain and snow", "[wet_look]") {
    const auto c = load();
    const auto dry = rg::wet_look_target(0.0, 0.0, 15.0, c);
    CHECK(dry.wetness == 0.0);
    CHECK(dry.puddles == 0.0);
    CHECK(dry.drops == 0.0);
    CHECK(dry.flakes == 0.0);
    const auto damp = rg::wet_look_target(c.wet_full_mm * 0.5, 0.0, 15.0, c);
    CHECK(damp.wetness == Catch::Approx(0.5));
    CHECK(damp.puddles == 0.0);
    const auto soaked = rg::wet_look_target(c.puddle_full_mm, 4.0, 15.0, c);
    CHECK(soaked.wetness == 1.0);
    CHECK(soaked.puddles == Catch::Approx(1.0));
    CHECK(soaked.drops == Catch::Approx(4.0 / c.rain_full_mm_h));
    CHECK(soaked.flakes == 0.0);
    const auto drizzle = rg::wet_look_target(0.0, c.rain_min_mm_h * 0.5, 15.0, c);
    CHECK(drizzle.drops == 0.0);
    const auto snow = rg::wet_look_target(0.0, 2.0, c.snow_below_c - 3.0, c);
    CHECK(snow.drops == 0.0);
    CHECK(snow.flakes > 0.0);
    const auto bad = rg::wet_look_target(NAN, -3.0, NAN, c);
    CHECK(bad.wetness == 0.0);
    CHECK(bad.drops == 0.0);
}

TEST_CASE("wet look: filter jumps first, then follows with the time constant", "[wet_look]") {
    auto c = load();
    c.response_s = 2.0;
    rg::WetLookFilter f(c);
    CHECK(f.update(0.0, 0.0, 15.0, 0.016).wetness == 0.0);
    // One time constant towards a fully wet target: 1 - 1/e.
    double w = 0;
    for (int i = 0; i < 200; ++i) w = f.update(1.0, 0.0, 15.0, 0.01).wetness;
    CHECK(w == Catch::Approx(1.0 - std::exp(-1.0)).epsilon(1e-6));
    f.reset();
    CHECK(f.update(1.0, 0.0, 15.0, 0.01).wetness == 1.0);
}
