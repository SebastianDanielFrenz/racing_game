// rg/exposure.h + the illuminance/exposure of rg::TimeWeather (lights/exposure, 2026-10-15):
// adaptation, the day/night range against data/world/sky.json, and that a lamp of constant
// physical intensity is negligible by day and dominant at night.
#include "rg/exposure.h"
#include "rg/time_weather.h"
#include "rg/world_config.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "rg/driver_actions.h"
#include <fstream>
#include <string>

namespace {
constexpr double kPi = 3.14159265358979323846;
rg::TimeWeather frankfurt(const std::string& date_month_day = "06-21") {
    std::string err;
    auto config = rg::load_sky_config(std::string(RG_SOURCE_DIR) + "/data/world/sky.json", &err);
    INFO(err);
    REQUIRE(config);
    int m = 6, d = 21;
    std::sscanf(date_month_day.c_str(), "%d-%d", &m, &d);
    config->start_date = rg::CivilDate{2026, m, d};
    auto world = rg::load_world_config(std::string(RG_SOURCE_DIR) + "/data/world/world_config.json", &err);
    REQUIRE(world);
    return rg::TimeWeather(*config, rg::location_from_world_config(*world));
}
} // namespace

TEST_CASE("exposure: formula, clamping, combine and validation", "[exposure]") {
    rg::ExposureConfig c;
    c.key = 1.5; c.min_adapted_lux = 50; c.max_adapted_lux = 150000; c.adapt_tau_s = 1.5;
    CHECK(rg::validate_exposure_config(c).empty());
    // A white surface under the adapted illuminance E has E/pi nits and shows at `key`.
    CHECK(1e5 / kPi * rg::exposure_for_lux(c, 1e5) == Catch::Approx(1.5));
    CHECK(rg::exposure_for_lux(c, 1.0) == Catch::Approx(rg::night_exposure(c)));
    CHECK(rg::exposure_for_lux(c, 1e9) == Catch::Approx(rg::exposure_for_lux(c, 150000)));
    CHECK(rg::exposure_for_lux(c, std::nan("")) == Catch::Approx(rg::night_exposure(c)));
    CHECK(rg::combine_exposure(0.01, 0.04, 0.5) == Catch::Approx(0.02));
    CHECK(rg::combine_exposure(0.01, 0.04, 0) == Catch::Approx(0.01));
    CHECK(rg::combine_exposure(0.01, -1, 0.5) == Catch::Approx(0.01));
    auto bad = c; bad.min_adapted_lux = 0; CHECK_FALSE(rg::validate_exposure_config(bad).empty());
    bad = c; bad.max_adapted_lux = 10; CHECK_FALSE(rg::validate_exposure_config(bad).empty());
    bad = c; bad.key = 0; CHECK_FALSE(rg::validate_exposure_config(bad).empty());
    bad = c; bad.adapt_tau_s = -1; CHECK_FALSE(rg::validate_exposure_config(bad).empty());
}

TEST_CASE("exposure: the controller adapts in log space and the first update snaps", "[exposure]") {
    rg::ExposureConfig c;
    c.adapt_tau_s = 2.0;
    rg::ExposureController x(c);
    CHECK_FALSE(x.valid());
    x.update(100, 0.1);
    CHECK(x.adapted_lux() == Catch::Approx(100));
    // One time constant towards 10000 lux: e^-1 of the way left in log10 (2 -> 4).
    x.update(10000, 2.0);
    CHECK(std::log10(x.adapted_lux()) == Catch::Approx(4 - 2 * std::exp(-1.0)).margin(1e-9));
    x.update(10000, 100.0);
    CHECK(x.adapted_lux() == Catch::Approx(10000).epsilon(1e-6));
    x.update(1, 0); // dt 0: no change
    CHECK(x.adapted_lux() == Catch::Approx(10000).epsilon(1e-6));
    x.reset(1); // below the floor
    CHECK(x.adapted_lux() == Catch::Approx(c.min_adapted_lux));
}

TEST_CASE("exposure: sky illuminance and exposure across the day, lamps vs daylight", "[exposure][sky]") {
    auto tw = frankfurt();
    REQUIRE(tw.choose("time:noon"));
    const auto noon = tw.state();
    REQUIRE(tw.choose("time:night"));
    const auto night = tw.state();
    tw.clock().set_local_time_h(22.0); // 2026-06-21 22:00 CEST: sun a few degrees below the horizon (dusk)
    const auto dusk = tw.state();
    REQUIRE(tw.choose("time:noon"));
    REQUIRE(tw.choose("weather:overcast"));
    const auto overcast = tw.state();
    std::printf("illuminance: noon %.0f lux (sun %.1f deg), overcast noon %.0f, dusk %.2f (sun %.1f deg), night %.4f (sun %.1f deg)\n",
                noon.lighting.illuminance_lux, noon.sun.elevation_deg, overcast.lighting.illuminance_lux,
                dusk.lighting.illuminance_lux, dusk.sun.elevation_deg, night.lighting.illuminance_lux, night.sun.elevation_deg);
    std::printf("exposure: noon %.3g, overcast %.3g, dusk %.3g, night %.3g (night/noon %.0f)\n", noon.exposure, overcast.exposure,
                dusk.exposure, night.exposure, night.exposure / noon.exposure);
    CHECK(noon.lighting.illuminance_lux > 80000);
    CHECK(overcast.lighting.illuminance_lux < 0.4 * noon.lighting.illuminance_lux);
    CHECK(overcast.lighting.illuminance_lux > 7000); // overcast midsummer noon: still above the auto-lamp off threshold
    CHECK(dusk.lighting.illuminance_lux < 1000);
    CHECK(night.lighting.illuminance_lux < 1);
    CHECK(night.exposure == Catch::Approx(rg::night_exposure(tw.config().exposure)));
    CHECK(night.exposure / noon.exposure > 1000);

    // A dipped beam (headlights.json, candela) on the road 25 m ahead against the sunlit road:
    // < 1 % of the daylight at noon, many times the ambient at night. Brake lens vs night.
    std::ifstream in(std::string(RG_SOURCE_DIR) + "/data/world/headlights.json");
    const auto h = nlohmann::json::parse(in);
    const double beam_lux = h.at("dipped").at("intensity_cd").get<double>() / (25.0 * 25.0);
    CHECK(beam_lux / noon.lighting.illuminance_lux < 0.01);
    CHECK(beam_lux * night.exposure / kPi > 0.2); // white surface display value at night
    CHECK(beam_lux * noon.exposure / kPi < 0.005);
    const double brake = h.at("lens_luminance_nits").at("brake").get<double>();
    CHECK(brake * night.exposure > 1.0); // saturated red at night
    CHECK(brake * noon.exposure < 0.5);  // well below a sunlit white surface (~1.5) by day
    // The pure-GDScript fallback exposure equals the night exposure.
    CHECK(h.at("fallback_exposure").get<double>() == Catch::Approx(rg::night_exposure(tw.config().exposure)).epsilon(1e-5));
}

TEST_CASE("exposure: sample() adapts over real time and snaps on a clock jump", "[exposure][sky]") {
    auto tw = frankfurt();
    REQUIRE(tw.choose("time:noon"));
    const double noon = tw.sample().exposure;
    CHECK(noon == Catch::Approx(tw.state().exposure));
    REQUIRE(tw.choose("time:night")); // menu jump: snaps
    CHECK(tw.sample().exposure == Catch::Approx(rg::night_exposure(tw.config().exposure)));
    // Fast clock across sunset: the exposure lags the instant value, then catches up.
    tw.clock().set_local_time_h(20.5);
    REQUIRE(tw.choose("rate:4")); // x600
    const double before = tw.sample().exposure;
    tw.advance(6.0); // one game hour in 6 real seconds, all at once
    const double instant = tw.state().exposure;
    const double lagged = tw.sample().exposure;
    INFO("before " << before << " lagged " << lagged << " instant " << instant);
    CHECK(instant > before * 2);
    CHECK(lagged > before);
    CHECK(lagged <= instant);
    REQUIRE(tw.choose("weather:overcast")); // a weather change snaps
    CHECK(tw.sample().exposure == Catch::Approx(tw.state().exposure));
}

// Hidden: writes the sky state the review shots use (game/scripts/lights_exposure_shots.gd),
// computed by rg_core for Frankfurt 2026-06-21 (noon, dusk, night) plus overcast noon, and the
// automatic headlamp decision for each (rg::DriverActions, data/world/headlights.json "auto").
//   RG_EXPOSURE_SHOTS_JSON=<file> rg_unit_tests "[exposure_shots]"
TEST_CASE("exposure: write the review-shot sky states", "[.][diag][exposure_shots]") {
    const char* path = std::getenv("RG_EXPOSURE_SHOTS_JSON");
    REQUIRE(path != nullptr);
    std::ifstream in(std::string(RG_SOURCE_DIR) + "/data/world/headlights.json");
    const auto h = nlohmann::json::parse(in).at("auto");
    std::ifstream cin_(std::string(RG_SOURCE_DIR) + "/data/controls/cabin.json");
    const auto cj = nlohmann::json::parse(cin_);
    nlohmann::json out = nlohmann::json::array();
    struct Shot { const char* name; double hour; const char* weather; };
    for (const Shot shot : {Shot{"noon", 13.0, "clear"}, Shot{"overcast", 13.0, "overcast"}, Shot{"sunset", 21.4, "clear"},
                            Shot{"dusk", 22.0, "clear"}, Shot{"night", 23.5, "clear"}}) {
        auto tw = frankfurt();
        REQUIRE(tw.choose(std::string("weather:") + shot.weather));
        tw.clock().set_local_time_h(shot.hour);
        const auto s = tw.sample();
        rg::CabinConfig cfg;
        cfg.crank_timeout_s = cj.at("crank_timeout_s"); cfg.stop_max_speed_mps = cj.at("stop_max_speed_mps");
        cfg.blink_hz = cj.at("blink_hz"); cfg.indicator_arm_deg = cj.at("indicator_arm_deg");
        cfg.indicator_cancel_deg = cj.at("indicator_cancel_deg"); cfg.brake_lamp_threshold = cj.at("brake_lamp_threshold");
        cfg.auto_lights = h.at("enabled"); cfg.auto_lights_on_lux = h.at("on_lux"); cfg.auto_lights_off_lux = h.at("off_lux");
        cfg.auto_lights_on_delay_s = h.at("on_delay_s"); cfg.auto_lights_off_delay_s = h.at("off_delay_s");
        rg::DriverActions actions;
        actions.configure_cabin(cfg);
        struct Sink final : rg::ChannelSink {
            double lights = 0;
            double get(std::string_view) const override { return 0; }
            void set(std::string_view c, double v) override { if (c == "cabin.lights") lights = v; }
        } sink;
        rg::DriverSnapshot snap;
        snap.ambient_lux = s.lighting.illuminance_lux;
        actions.advance({snap, true}, sink, 0.1);
        auto rgb = [](const rg::Rgb& c) { return nlohmann::json::array({c[0], c[1], c[2]}); };
        out.push_back({{"name", shot.name}, {"clock", s.clock_text}, {"sun_elevation_deg", s.sun.elevation_deg},
                       {"sun_energy", s.lighting.sun_energy}, {"sun_rgb", rgb(s.lighting.sun_rgb)},
                       {"moon_energy", s.lighting.moon_energy}, {"ambient_energy", s.lighting.ambient_energy},
                       {"sky_top_rgb", rgb(s.lighting.sky_top_rgb)}, {"sky_horizon_rgb", rgb(s.lighting.sky_horizon_rgb)},
                       {"illuminance_lux", s.lighting.illuminance_lux}, {"exposure", s.exposure}, {"auto_lights", sink.lights}});
    }
    std::ofstream(path) << out.dump(2) << "\n";
    std::printf("%s\n", out.dump(2).c_str());
}
