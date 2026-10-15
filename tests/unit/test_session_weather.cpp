// V8: the physics weather environment installed in rg::Session (flat mode):
// rain wets the tagged ground and lowers grip (longer braking distance),
// determinism across worker counts, the weather sync from TimeWeather and the
// menu entries enabled once a climate is set.
#include "rg/climate.h"
#include "rg/session.h"
#include "rg/time_weather.h"
#include "rg/weather_sync.h"
#include "rg/world_config.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>
#include <string>

namespace {
constexpr double kLat = 50.1109, kLon = 8.6821;
const rg::ClimateConfig& climate() {
    static const rg::ClimateConfig c = [] {
        std::string err;
        auto r = rg::load_climate_config(std::string(RG_SOURCE_DIR) + "/data/world/climate.json", &err);
        if (!r) FAIL(err);
        return *r;
    }();
    return c;
}
rg::SessionConfig weather_config(const std::string& weather_id, unsigned workers = 0, double speed = 0.0) {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/data/surfaces/surfaces.json";
    config.job_workers = workers;
    config.chassis_initial_velocity = ps::Vec3{speed, 0.0, 0.0};
    std::string err;
    // 14 Oct, 12:00 UT: about 15 degC, above freezing so rain stays water.
    auto w = rg::make_session_weather(climate(), kLat, kLon, 0.0, {2026, 10, 14}, 12.0, 1.0, weather_id, &err);
    if (!w) FAIL(err);
    config.weather = std::make_shared<const rg::SessionWeather>(std::move(*w));
    return config;
}
// One hour of the configured weather at once (a stepped forward jump of the condition grid), then settle.
void soak_one_hour(rg::Session& s) {
    const auto p = rg::env_clock_point({2026, 10, 14}, 13.0, kLon);
    s.set_environment_clock(p.env_s, p.day_index, 1.0);
    for (int i = 0; i < 5; ++i) s.step();
}
double braking_distance(const std::string& weather_id) {
    // The car starts at 25 m/s; the hour of weather is applied in the first ticks, then full brake.
    rg::Session s(weather_config(weather_id, 0, 25.0));
    soak_one_hour(s);
    const double x0 = s.world().get_pose(s.chassis_body()).position.x;
    s.world().set_control("brake", 1.0);
    for (int i = 0; i < 240 * 10; ++i) {
        s.step();
        if (s.world().get_motion(s.chassis_body()).linear.length() < 0.5) break;
    }
    return s.world().get_pose(s.chassis_body()).position.x - x0;
}
} // namespace

TEST_CASE("weather: no SessionConfig::weather installs nothing", "[weather][v8]") {
    rg::SessionConfig config;
    config.vehicle_json_path = std::string(RG_SOURCE_DIR) + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = std::string(RG_SOURCE_DIR) + "/data/surfaces/surfaces.json";
    rg::Session s(config);
    s.step();
    CHECK_FALSE(s.weather_installed());
    CHECK(s.world().environment() == nullptr);
    CHECK_FALSE(s.weather_state().installed);
}

TEST_CASE("weather: rain wets the tagged flat ground and lowers grip", "[weather][v8]") {
    rg::Session dry(weather_config("clear"));
    rg::Session wet(weather_config("rain"));
    REQUIRE(dry.weather_installed());
    soak_one_hour(dry);
    soak_one_hour(wet);
    for (int i = 0; i < 120; ++i) { dry.step(); wet.step(); }
    const auto d = dry.weather_state();
    const auto w = wet.weather_state();
    REQUIRE(w.installed);
    CHECK(w.precip_mm_h > 0.0);
    CHECK(d.precip_mm_h == 0.0);
    CHECK(w.wheel_surface == "asphalt");
    CHECK(w.wheel_water_mm > d.wheel_water_mm);
    CHECK(w.wheel_condition_factor < 0.95);
    CHECK(d.wheel_condition_factor > 0.99);
    // Pinned temperature: the physics air at the car equals the climate formula at the car's altitude.
    const double expect = rg::climate_temperature_c(climate(), {2026, 10, 14}, 12.0, kLon, climate().station_elevation_m, 0.95);
    CHECK(std::fabs(w.air_temp_c - expect) < 0.3);

    const double dry_m = braking_distance("clear");
    const double wet_m = braking_distance("rain");
    INFO("braking 25 m/s: dry " << dry_m << " m, wet " << wet_m << " m");
    CHECK(dry_m > 10.0);
    CHECK(wet_m > dry_m * 1.1);
}

TEST_CASE("weather: deterministic across worker counts", "[weather][v8][determinism]") {
    auto run = [](unsigned workers) {
        rg::Session s(weather_config("thunder", workers, 15.0));
        s.world().set_control("throttle", 0.3);
        for (int i = 0; i < 600; ++i) s.step();
        return s.world().state_hash();
    };
    CHECK(run(1) == run(4));
}

TEST_CASE("weather: TimeWeather menu enabled and WeatherSync feeds the session", "[weather][v8][sky]") {
    std::string err;
    auto sky = rg::load_sky_config(std::string(RG_SOURCE_DIR) + "/data/world/sky.json", &err);
    REQUIRE(sky);
    sky->start_date = rg::CivilDate{2026, 10, 14};
    rg::TimeWeather tw(*sky, rg::GeoLocation{kLat, kLon, 0.0});
    for (const auto& item : tw.menu())
        if (item.id == "weather:rain" || item.id == "weather:thunder") CHECK_FALSE(item.enabled); // no climate: greyed
    CHECK(std::isnan(tw.state().air_temp_c));
    tw.set_climate(std::make_shared<const rg::ClimateConfig>(climate()));
    for (const auto& item : tw.menu()) CHECK(item.enabled);
    CHECK(std::isfinite(tw.state().air_temp_c));
    REQUIRE(tw.choose("weather:rain"));

    auto w = rg::session_weather_for(tw, &err);
    INFO(err);
    REQUIRE(w);
    rg::SessionConfig config = weather_config("clear");
    config.weather = std::make_shared<const rg::SessionWeather>(std::move(*w));
    rg::Session s(config);
    rg::WeatherSync sync;
    CHECK(sync.sync(tw, s.weather_state(), s) == (rg::WeatherSync::Clock | rg::WeatherSync::Weather));
    s.step();
    CHECK(sync.sync(tw, s.weather_state(), s) == rg::WeatherSync::None);
    REQUIRE(tw.choose("weather:clear"));
    CHECK(sync.sync(tw, s.weather_state(), s) == rg::WeatherSync::Weather);
    REQUIRE(tw.choose("time:night"));
    CHECK((sync.sync(tw, s.weather_state(), s) & rg::WeatherSync::Clock) != 0);
    s.step();
    const auto st = s.weather_state();
    const auto expect = rg::env_clock_point({2026, 10, 14}, 23.5 - tw.clock().utc_offset_h(), kLon);
    CHECK(std::fabs(st.env_time_s - expect.env_s) < 5.0);
}
