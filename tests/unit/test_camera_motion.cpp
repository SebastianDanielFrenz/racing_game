// test_camera_motion.cpp - the driver's virtual body (rg/camera_motion.h) and
// the model-based bumper camera (rg/camera_math.h bumper_eye_from_model).
#include "rg/camera_math.h"
#include "rg/camera_motion.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <utility>
#include <string>

namespace {

using Catch::Matchers::WithinAbs;

const std::string kRepo = RG_SOURCE_DIR;

rg::CameraMotionConfig shipped() {
    std::string err;
    const auto c = rg::load_camera_motion_config(kRepo + "/data/controls/camera_motion.json", &err);
    INFO(err);
    REQUIRE(c);
    return *c;
}

// The car moves along +x with the given speed profile; 240 Hz ticks, 60 Hz frames.
struct Drive {
    rg::CameraMotion motion;
    double t = 0.0;
    double v = 0.0;
    explicit Drive(rg::CameraMotionConfig c) : motion(std::move(c)) {}
    void frames(int n, double accel, double bump_amp = 0.0) {
        for (int f = 0; f < n; ++f) {
            for (int k = 0; k < 4; ++k) {
                t += 1.0 / 240.0;
                v += accel / 240.0;
            }
            rg::CarMotionSample s;
            s.sim_time = t;
            s.velocity = ps::Vec3{v, 0.0, 0.0};
            const double c = bump_amp * std::sin(2.0 * 3.14159265358979 * 7.0 * t);
            s.wheel_compression = {c, -c, c, -c};
            motion.update(s, 1.0 / 60.0);
        }
    }
};

} // namespace

TEST_CASE("camera motion: shipped config loads, VR gains are zero", "[camera_motion]") {
    const auto c = shipped();
    REQUIRE(c.views.count("cockpit") == 1);
    CHECK(c.views.at("cockpit").head_translation == 1.0); // full strength for the first VR test
    CHECK(c.views.at("xr_cockpit").head_rotation == 0.0);
    CHECK(c.views.at("xr_cockpit").shake == 0.0);
    CHECK(c.views.at("xr_cockpit").fov == 0.0);
    // owner rule: head motion only in the cockpit views
    for (const char* v : {"chase", "bumper"}) {
        CHECK(c.views.at(v).head_translation == 0.0);
        CHECK(c.views.at(v).head_rotation == 0.0);
    }
}

TEST_CASE("camera motion: loader rejects bad files", "[camera_motion]") {
    std::ifstream in(kRepo + "/data/controls/camera_motion.json");
    nlohmann::json j;
    in >> j;
    const std::string dir = kRepo + "/out/test_tmp";
    std::string mk = "mkdir -p '" + dir + "'";
    REQUIRE(std::system(mk.c_str()) == 0);
    auto try_load = [&](const nlohmann::json& doc) {
        const std::string p = dir + "/camera_motion_bad.json";
        std::ofstream(p) << doc.dump();
        std::string err;
        const auto c = rg::load_camera_motion_config(p, &err);
        CHECK_FALSE(err.empty());
        return c.has_value();
    };
    auto a = j; a["views"]["xr_cockpit"]["head_rotation"] = 0.5;
    CHECK_FALSE(try_load(a));
    auto b = j; b["views"]["xr_cockpit"]["shake"] = 1.0;
    CHECK_FALSE(try_load(b));
    auto c = j; c["head"]["frequency_hz"] = {1.0, 2.0};
    CHECK_FALSE(try_load(c));
    auto d = j; d["format"] = "rg.camera_motion/2";
    CHECK_FALSE(try_load(d));
    auto e = j; e["views"].erase("cockpit");
    CHECK_FALSE(try_load(e));
    std::string err;
    CHECK_FALSE(rg::load_camera_motion_config(dir + "/missing.json", &err));
}

TEST_CASE("camera motion: braking moves the eye forward, then it settles", "[camera_motion]") {
    const auto cfg = shipped();
    Drive d(cfg);
    d.v = 30.0;
    d.frames(120, 0.0);
    CHECK_THAT(d.motion.state().offset.x, WithinAbs(0.0, 1e-9));
    d.frames(180, -9.0); // 3 s at ~0.9 g
    const double expect = cfg.limit_m[0] * std::tanh(cfg.deflection_m_per_mps2[0] * 9.0 / cfg.limit_m[0]);
    CHECK(d.motion.state().offset.x > 0.0);
    CHECK_THAT(d.motion.state().offset.x, WithinAbs(expect, 0.003));
    const auto cockpit = d.motion.view("cockpit", 1.0);
    CHECK(cockpit.pitch_down_rad > 0.0); // nods toward the wheel
    CHECK(d.motion.view("chase", 1.0).pitch_down_rad == 0.0);
    // stopped braking: back to the seat
    d.frames(300, 0.0);
    CHECK_THAT(d.motion.state().offset.x, WithinAbs(0.0, 0.001));
}

TEST_CASE("camera motion: constant velocity has no drift", "[camera_motion]") {
    Drive d(shipped());
    d.v = 80.0;
    d.frames(60 * 60, 0.0); // one minute
    const auto& s = d.motion.state();
    CHECK(s.offset.length() < 1e-9);
    CHECK(s.pitch_down_rad == 0.0);
    CHECK(s.roll_right_rad == 0.0);
    // the speed effects are on, but carry no head offset
    const auto bumper = d.motion.view("bumper", 1.0);
    CHECK(bumper.fov_extra_deg > 0.0);
    CHECK(std::fabs(bumper.offset.x) == 0.0);
}

TEST_CASE("camera motion: deterministic, strength scales the head, VR never rotates", "[camera_motion]") {
    Drive a(shipped()), b(shipped());
    for (Drive* d : {&a, &b}) {
        d->v = 40.0;
        d->frames(90, -7.0, 0.01);
        d->frames(90, 3.0, 0.02);
    }
    CHECK(a.motion.state().offset == b.motion.state().offset);
    const auto va = a.motion.view("cockpit", 1.0), vb = b.motion.view("cockpit", 1.0);
    CHECK(va.offset == vb.offset);
    CHECK(va.pitch_down_rad == vb.pitch_down_rad);
    const auto half = a.motion.view("cockpit", 0.5);
    CHECK_THAT(half.pitch_down_rad, WithinAbs(0.5 * va.pitch_down_rad, 1e-12));
    CHECK(a.motion.view("cockpit", 0.0).pitch_down_rad == 0.0);
    const auto xr = a.motion.view("xr_cockpit", 1.0);
    CHECK(xr.pitch_down_rad == 0.0);
    CHECK(xr.roll_right_rad == 0.0);
    CHECK(xr.fov_extra_deg == 0.0);
    CHECK(xr.offset == a.motion.state().offset); // translation only, no shake
    CHECK(a.motion.state().roughness_mps > 0.0);
}

TEST_CASE("camera motion: a relocation is not a force", "[camera_motion]") {
    Drive d(shipped());
    d.v = 50.0;
    d.frames(60, 0.0);
    d.v = 0.0; // teleport to rest in one tick
    d.frames(1, 0.0);
    CHECK(d.motion.state().offset.length() < 1e-9);
}

TEST_CASE("bumper camera: outside the body of every catalog model", "[camera_motion][bumper]") {
    std::ifstream in(kRepo + "/data/vehicles/catalog.json");
    nlohmann::json cat;
    in >> cat;
    int models = 0;
    for (const auto& e : cat["vehicles"]) {
        if (!e.contains("model")) continue; // presets inherit their base's model
        const std::string model = e["model"].get<std::string>();
        const std::string rig = kRepo + "/" + model.substr(0, model.size() - 4) + ".rig.json";
        std::string err;
        const auto b = rg::load_model_bounds(rig, &err);
        INFO(e["id"].get<std::string>() << " " << err);
        REQUIRE(b);
        const ps::Vec3 offset{0.1, 0.0, -0.4};
        const ps::Vec3 eye = rg::bumper_eye_from_model(*b, offset) - offset; // model frame
        CHECK(eye.x > b->max.x);
        CHECK(eye.z > b->min.z);
        CHECK(eye.z < b->max.z);
        CHECK(std::fabs(eye.y) < 0.01);
        ++models;
    }
    CHECK(models >= 5);
}

TEST_CASE("bumper camera: a rig socket overrides the computed eye", "[camera_motion][bumper]") {
    rg::ModelBounds b;
    b.min = {-2.0, -1.0, 0.0};
    b.max = {2.0, 1.0, 1.2};
    CHECK_THAT(rg::bumper_eye_from_model(b, {}).x, WithinAbs(2.05, 1e-12));
    CHECK_THAT(rg::bumper_eye_from_model(b, {}).z, WithinAbs(0.55, 1e-12));
    b.max.z = 0.8; // a low car: half its height
    CHECK_THAT(rg::bumper_eye_from_model(b, {}).z, WithinAbs(0.4, 1e-12));
    b.bumper_socket = ps::Vec3{1.5, 0.0, 0.9};
    const ps::Vec3 e = rg::bumper_eye_from_model(b, {0.0, 0.0, -0.5});
    CHECK_THAT(e.x, WithinAbs(1.5, 1e-12));
    CHECK_THAT(e.z, WithinAbs(0.4, 1e-12));
}
