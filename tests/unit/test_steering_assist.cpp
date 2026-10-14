// test_steering_assist.cpp - rg/steering_assist.h (adaptive steering input translation, moved from
// game/scripts/adaptive_steering.gd). The golden sequence below was produced by running the original GDScript under
// Godot 4.7.2 headless with the shipped data/controls/steering.json and data/vehicles/car_sedan.json (2026-10-14).
#include "rg/steering_assist.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using Catch::Approx;

namespace {

const std::string kRoot = RG_SOURCE_DIR;

rg::SteeringAssist shipped() {
    std::string err;
    const auto cfg = rg::load_steering_assist_config(kRoot + "/data/controls/steering.json", &err);
    REQUIRE(cfg.has_value());
    rg::SteeringAssist s(*cfg);
    const auto geom = rg::steering_geometry_from_vehicle_json(kRoot + "/data/vehicles/car_sedan.json");
    REQUIRE(geom.has_value());
    s.set_geometry(*geom);
    return s;
}

rg::SteeringMotion motion(double fwd, double left, double yaw, long long reloc) {
    rg::SteeringMotion m;
    m.forward_mps = fwd;
    m.left_mps = left;
    m.yaw_rad_s = yaw;
    m.upright = 1.0;
    m.relocations = reloc;
    return m;
}

} // namespace

TEST_CASE("steering assist: shipped config and car geometry load", "[steering_assist]") {
    rg::SteeringAssist s = shipped();
    CHECK(s.config().enabled);
    CHECK(s.config().input_profile == "auto");
    CHECK(s.geometry().wheelbase_m == Approx(2.7));
    CHECK(s.geometry().max_wheel_angle_rad == Approx(0.5236));
}

TEST_CASE("steering assist: matches the former GDScript output", "[steering_assist]") {
    rg::SteeringAssist s = shipped();
    std::vector<double> got;
    for (int i = 0; i < 10; ++i) got.push_back(s.translate(1.0, true, false, motion(30, 0, 0, 0), 0.016));
    for (int i = 0; i < 5; ++i) got.push_back(s.translate(0.0, true, false, motion(30, 0, 0, 0), 0.016));
    for (int i = 0; i < 10; ++i) got.push_back(s.translate(0.8, true, false, motion(20, 5, -0.3, 0), 0.02));
    for (int i = 0; i < 3; ++i) got.push_back(s.translate(0.5, false, false, motion(40, -0.5, 0.1, 1), 0.016));
    got.push_back(s.translate(0.37, false, true, motion(40, 0, 0, 1), 0.016));
    const std::vector<double> gold = {
        0.000993692447, 0.003460238078, 0.007179104042, 0.012049248833, 0.018005201941, 0.024999092272,
        0.032993505491, 0.041957921436, 0.051866686719, 0.062697754768, 0.041957921436, 0.024999092272,
        0.012049248833, 0.003460238078, 0.000000000000, 0.019079298208, 0.142120836784, 0.454755388344,
        0.595145572264, 0.595145572264, 0.595145572264, 0.595145572264, 0.595145572264, 0.595145572264,
        0.595145572264, 0.040204442425, 0.040204442425, 0.040204442425, 0.370000000000};
    REQUIRE(got.size() == gold.size());
    for (std::size_t i = 0; i < gold.size(); ++i) {
        INFO("step " << i);
        CHECK(got[i] == Approx(gold[i]).margin(1e-9));
    }
}

TEST_CASE("steering assist: passthrough, sign symmetry, relocation reset", "[steering_assist]") {
    rg::SteeringAssist s = shipped();
    // A wheel or a disabled assist passes the raw level through (clamped).
    CHECK(s.translate(1.7, false, true, motion(30, 0, 0, 0), 0.016) == Approx(1.0));
    rg::SteeringAssistConfig off = s.config();
    off.enabled = false;
    rg::SteeringAssist d(off);
    CHECK(d.translate(-0.25, true, false, motion(30, 0, 0, 0), 0.016) == Approx(-0.25));
    // Mirror image: right steer while sliding right gives the negated value.
    rg::SteeringAssist l = shipped(), r = shipped();
    for (int i = 0; i < 6; ++i) {
        const double a = l.translate(0.8, true, false, motion(20, 5, -0.3, 0), 0.02);
        const double b = r.translate(-0.8, true, false, motion(20, -5, 0.3, 0), 0.02);
        CHECK(a == Approx(-b).margin(1e-12));
    }
    // A relocation (counter change) drops the keyboard ramp.
    CHECK(l.keyboard_level() > 0.0);
    l.translate(0.0, true, false, motion(20, 0, 0, 1), 0.0);
    CHECK(l.keyboard_level() == 0.0);
    // Non-finite input never reaches the channel.
    CHECK(s.translate(std::nan(""), true, false, motion(30, 0, 0, 0), 0.016) == 0.0);
}

TEST_CASE("steering assist: config loader refuses a missing key", "[steering_assist]") {
    const auto dir = std::filesystem::path(kRoot) / "out" / "test_tmp";
    std::filesystem::create_directories(dir);
    const auto path = (dir / "steering_missing.json").string();
    {
        std::ofstream(path) << R"({"enabled": true, "input_profile": "auto", "input_exponent": 1.8})";
    }
    std::string err;
    CHECK_FALSE(rg::load_steering_assist_config(path, &err).has_value());
    CHECK(err.find("normal_min_fraction") != std::string::npos);
    CHECK_FALSE(rg::load_steering_assist_config(path + ".absent", &err).has_value());
}
