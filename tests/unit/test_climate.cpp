// rg/climate.h: data/world/climate.json, the derived air temperature (monthly
// mean + daily amplitude, mid-afternoon peak, lapse rate, cloud damping) and
// the physics climate/preset/environment built from the same file (V8).
#include "rg/climate.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>

namespace {
constexpr double kLat = 50.1109, kLon = 8.6821;
rg::ClimateConfig load() {
    std::string err;
    auto r = rg::load_climate_config(std::string(RG_SOURCE_DIR) + "/data/world/climate.json", &err);
    INFO(err);
    REQUIRE(r);
    return *r;
}
// UT hour at which local mean solar time is `solar_h` at kLon.
double ut_for_solar(double solar_h) { return solar_h - kLon / 15.0; }
} // namespace

TEST_CASE("climate: data/world/climate.json loads", "[climate]") {
    const auto c = load();
    CHECK(c.station_elevation_m == Catch::Approx(112.0));
    CHECK(c.lapse_k_per_m == Catch::Approx(0.0065));
    CHECK(c.ground_surface == "asphalt");
    for (const char* id : {"clear", "partly_cloudy", "overcast", "rain", "snow", "hail", "thunder"}) CHECK(c.weather.count(id) == 1);
    CHECK(c.weather.at("rain").precip_mm_h > 0.0);
    std::string err;
    CHECK_FALSE(rg::load_climate_config(std::string(RG_SOURCE_DIR) + "/data/world/sky.json", &err));
    CHECK(err.find("rg.climate/1") != std::string::npos);
}

TEST_CASE("climate: monthly interpolation hits the midpoints and is cyclic", "[climate]") {
    const auto c = load();
    CHECK(rg::climate_daily_mean_c(c, 16.0) == Catch::Approx(c.temp_mean_c[0]));     // 16 Jan = Jan midpoint
    CHECK(rg::climate_daily_mean_c(c, 197.0) == Catch::Approx(c.temp_mean_c[6]));    // 16 Jul
    const double dec_jan = rg::climate_daily_mean_c(c, 365.0);
    CHECK(dec_jan > std::min(c.temp_mean_c[0], c.temp_mean_c[11]) - 1e-9);
    CHECK(dec_jan < std::max(c.temp_mean_c[0], c.temp_mean_c[11]) + 1e-9);
    CHECK(rg::climate_daily_mean_c(c, 1.0) == Catch::Approx(rg::climate_daily_mean_c(c, 366.0)));
    CHECK(rg::climate_daily_range_k(c, 197.0) == Catch::Approx(c.temp_range_k[6]));
}

TEST_CASE("climate: afternoon peak, night minimum, lapse and cloud damping", "[climate]") {
    const auto c = load();
    const rg::CivilDate jul{2026, 7, 16};
    const double peak = rg::climate_station_temperature_c(c, jul, ut_for_solar(14.5), kLon, 0.0);
    const double night = rg::climate_station_temperature_c(c, jul, ut_for_solar(2.5), kLon, 0.0);
    const double morning = rg::climate_station_temperature_c(c, jul, ut_for_solar(8.0), kLon, 0.0);
    CHECK(peak == Catch::Approx(c.temp_mean_c[6] + 0.5 * c.temp_range_k[6]).margin(0.05));
    CHECK(night == Catch::Approx(c.temp_mean_c[6] - 0.5 * c.temp_range_k[6]).margin(0.05));
    CHECK(peak > morning);
    CHECK(morning > night);
    // Summer afternoon warmer than winter afternoon.
    CHECK(peak > rg::climate_station_temperature_c(c, {2026, 1, 16}, ut_for_solar(14.5), kLon, 0.0) + 10.0);
    // Lapse: 1000 m higher is 6.5 K colder.
    const double high = rg::climate_temperature_c(c, jul, ut_for_solar(14.5), kLon, c.station_elevation_m + 1000.0, 0.0);
    CHECK(peak - high == Catch::Approx(6.5).margin(1e-9));
    // Overcast damps the daily swing by cloud_range_reduction.
    const double peak_ov = rg::climate_station_temperature_c(c, jul, ut_for_solar(14.5), kLon, 1.0);
    CHECK(peak_ov - c.temp_mean_c[6] == Catch::Approx((peak - c.temp_mean_c[6]) * (1.0 - c.cloud_range_reduction)).margin(0.02));
}

TEST_CASE("climate: env clock points and the physics environment", "[climate]") {
    const auto c = load();
    const auto p = rg::env_clock_point({2026, 1, 1}, 0.0, kLon);
    CHECK(p.day_index == 0);
    CHECK(p.solar_tod_s == Catch::Approx(kLon / 15.0 * 3600.0));
    CHECK(rg::env_clock_point({2026, 12, 31}, 23.5, kLon).day_index == 0); // past local midnight: next year's 1 Jan
    CHECK(rg::env_clock_point({2028, 2, 29}, 12.0, 0.0).day_index == 58);
    CHECK(rg::env_clock_point({2028, 3, 1}, 12.0, 0.0).day_index == 59);

    std::string err;
    auto climate = rg::make_physics_climate(c, kLat, kLon, &err);
    INFO(err);
    REQUIRE(climate);
    CHECK(climate->seasons.size() == 12);
    auto rain = rg::physics_weather_preset(c, "rain", 7.5);
    REQUIRE(rain);
    CHECK(rain->temp_is_override);
    CHECK(rain->temp_override_c == 7.5);
    CHECK_FALSE(rg::physics_weather_preset(c, "no_such", 7.5));
    auto w = rg::make_session_weather(c, kLat, kLon, 0.5, {2026, 10, 14}, 12.0, 1.0, "thunder", &err);
    INFO(err);
    REQUIRE(w);
    CHECK(w->config.fixed_day_of_year == rg::env_clock_point({2026, 10, 14}, 12.0, kLon).day_index);
    CHECK(w->config.weather_preset.lightning_per_min > 0.0);
    CHECK(w->config.surface_conditions.has_value());
}
