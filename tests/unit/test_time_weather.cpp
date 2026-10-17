// rg/sky.h + rg/time_weather.h: sun/moon position against reference values,
// civil time (EU summer time), the time controller, the weather icon table,
// the lighting curve, the menu view-model and data/world/sky.json.
#include "rg/sky.h"
#include "rg/time_weather.h"
#include "rg/world_config.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>

namespace {
constexpr double kLat = 50.1109, kLon = 8.6821; // Frankfurt am Main
double az_diff(double a, double b) {
    double d = std::fmod(std::abs(a - b), 360.0);
    return d > 180 ? 360 - d : d;
}
struct Ref {
    rg::CivilDate date;
    double ut_h;
    double sun_el, sun_az, moon_el, moon_az, moon_phase;
};
// Reference: PyEphem 4.2.1 (pressure 0 = no refraction, sea level), Frankfurt
// 50.1109 N 8.6821 E. Computed once, values copied here.
const Ref kRefs[] = {
    {{2026, 6, 21}, 10.0, 58.388, 139.523, -10.398, 77.587, 0.450},
    {{2026, 12, 21}, 11.0, 16.268, 174.430, -11.303, 31.946, 0.900},
    {{2026, 3, 20}, 6.5, 9.010, 101.158, 6.480, 84.830, 0.020},
    {{2026, 10, 14}, 15.0, 13.477, 238.781, 11.740, 191.466, 0.153},
    {{2026, 10, 14}, 22.0, -45.749, 334.324, -36.156, 269.047, 0.174},
    {{2027, 1, 10}, 4.0, -31.405, 87.356, -44.123, 59.417, 0.047},
    {{2026, 12, 24}, 20.0, -42.314, 284.573, 35.386, 91.998, 0.990},
    {{2026, 7, 29}, 22.0, -18.390, 337.085, 15.672, 153.480, 0.999},
};
} // namespace

TEST_CASE("sky: sun position matches the reference within 0.5 deg", "[sky]") {
    for (const auto& r : kRefs) {
        const auto sun = rg::sun_position(rg::julian_day(r.date, r.ut_h), kLat, kLon);
        INFO(r.date.year << "-" << r.date.month << "-" << r.date.day << " " << r.ut_h << " UT");
        CHECK(std::abs(sun.elevation_deg - r.sun_el) < 0.5);
        CHECK(az_diff(sun.azimuth_deg, r.sun_az) < 0.5);
        CHECK(std::hypot(std::hypot(sun.east, sun.north), sun.up) == Catch::Approx(1.0));
    }
}

TEST_CASE("sky: moon position within 1 deg, phase within 0.03", "[sky]") {
    for (const auto& r : kRefs) {
        const auto moon = rg::moon_position(rg::julian_day(r.date, r.ut_h), kLat, kLon);
        INFO(r.date.year << "-" << r.date.month << "-" << r.date.day << " " << r.ut_h << " UT");
        CHECK(std::abs(moon.elevation_deg - r.moon_el) < 1.0);
        CHECK(az_diff(moon.azimuth_deg, r.moon_az) < 1.0 / std::max(0.2, std::cos(moon.elevation_deg * 3.14159265 / 180)));
        CHECK(std::abs(moon.illuminated_fraction - r.moon_phase) < 0.03);
    }
}

TEST_CASE("sky: calendar helpers and EU summer time", "[sky]") {
    CHECK(rg::julian_day({2000, 1, 1}, 12.0) == Catch::Approx(2451545.0));
    CHECK(rg::day_of_year({2026, 12, 31}) == 365);
    CHECK(rg::day_of_year({2028, 12, 31}) == 366);
    const auto d = rg::add_days({2026, 12, 31}, 1);
    CHECK((d.year == 2027 && d.month == 1 && d.day == 1));
    const auto b = rg::add_days({2028, 3, 1}, -1);
    CHECK((b.month == 2 && b.day == 29));
    const rg::TimeZoneRule cet{1.0, true};
    // 2026: summer time from 29 March 01:00 UT to 25 October 01:00 UT.
    CHECK(rg::utc_offset_hours(cet, {2026, 3, 29}, 0.99) == 1.0);
    CHECK(rg::utc_offset_hours(cet, {2026, 3, 29}, 1.0) == 2.0);
    CHECK(rg::utc_offset_hours(cet, {2026, 10, 25}, 0.99) == 2.0);
    CHECK(rg::utc_offset_hours(cet, {2026, 10, 25}, 1.0) == 1.0);
    CHECK(rg::utc_offset_hours({1.0, false}, {2026, 7, 1}, 12) == 1.0);
}

TEST_CASE("time controller: progressive, user override, day rollover, local -> UT", "[sky][time_of_day]") {
    rg::TimeController clock({2026, 6, 21}, 23.5, 60.0, {1.0, true});
    CHECK(clock.utc_offset_h() == 2.0);
    // 12:00 CEST = 10:00 UT.
    clock.set_local_time_h(12.0);
    CHECK(clock.julian_day_ut() == Catch::Approx(rg::julian_day({2026, 6, 21}, 10.0)));
    clock.set_local_time_h(23.5);
    clock.advance(60.0); // 60 s x60 = 1 h
    CHECK(clock.local_date().day == 22);
    CHECK(clock.local_time_h() == Catch::Approx(0.5));
    clock.set_rate(0);
    clock.advance(1000);
    CHECK(clock.local_time_h() == Catch::Approx(0.5));
    clock.set_rate(-3); // ignored
    CHECK(clock.rate() == 0);
    clock.set_local_time_h(-1);
    CHECK(clock.local_time_h() == Catch::Approx(23.0));
    clock.advance(std::nan(""));
    CHECK(clock.local_time_h() == Catch::Approx(23.0));
}

TEST_CASE("weather icon: selection table", "[sky][weather_icon]") {
    const rg::IconThresholds t{0.2, 0.7, -3.0};
    using P = rg::Precipitation;
    using I = rg::WeatherIcon;
    struct Row {
        double cover;
        P precip;
        bool thunder;
        double sun_el;
        I icon;
    };
    const Row rows[] = {
        {0.0, P::None, false, 30, I::Sun},       {0.19, P::None, false, 30, I::Sun},
        {0.2, P::None, false, 30, I::SunCloud},  {0.69, P::None, false, -2.9, I::SunCloud},
        {0.7, P::None, false, 30, I::Cloud},     {0.9, P::None, false, -40, I::Cloud},
        {0.0, P::None, false, -3.1, I::Moon},    {0.4, P::None, false, -20, I::MoonCloud},
        {0.9, P::Rain, false, 30, I::Rain},      {0.9, P::Rain, false, -30, I::Rain},
        {0.9, P::Snow, false, 30, I::Snow},      {0.9, P::Hail, false, 30, I::Hail},
        {1.0, P::Rain, true, 30, I::Thunder},    {0.0, P::None, true, -30, I::Thunder},
    };
    for (const auto& r : rows) {
        INFO("cover " << r.cover << " sun " << r.sun_el);
        CHECK(rg::weather_icon({r.cover, r.precip, r.thunder}, r.sun_el, t) == r.icon);
    }
    CHECK(std::string(rg::weather_icon_name(I::MoonCloud)) == "moon_cloud");
}

TEST_CASE("time/weather: data/world/sky.json, lighting and menu", "[sky][time_of_day]") {
    std::string err;
    auto config = rg::load_sky_config(std::string(RG_SOURCE_DIR) + "/data/world/sky.json", &err);
    INFO(err);
    REQUIRE(config);
    config->start_date = rg::CivilDate{2026, 6, 21};
    config->start_local_time_h = 13.0;

    // Location from the shipped world config's UTM session origin.
    auto world = rg::load_world_config(std::string(RG_SOURCE_DIR) + "/data/world/world_config.json", &err);
    REQUIRE(world);
    const auto loc = rg::location_from_world_config(*world);
    CHECK(loc.latitude_deg == Catch::Approx(50.18).margin(0.2));
    CHECK(loc.longitude_deg == Catch::Approx(8.5).margin(0.2));
    CHECK(std::abs(loc.grid_convergence_deg) < 1.0);

    rg::TimeWeather tw(*config, loc);
    auto s = tw.state();
    CHECK(s.clock_text == "13:00");
    CHECK(s.date_text == "2026-06-21");
    CHECK(s.sun.elevation_deg > 55);
    CHECK(s.icon == rg::WeatherIcon::Sun);
    CHECK(s.lighting.sun_energy > 1.0);
    CHECK(s.lighting.moon_energy == 0);

    // Night: no sun light, the icon is the moon, ambient is low.
    REQUIRE(tw.choose("time:night"));
    s = tw.state();
    CHECK(s.sun.elevation_deg < -10);
    CHECK(s.icon == rg::WeatherIcon::Moon);
    CHECK(s.lighting.sun_energy == 0);
    CHECK(s.lighting.ambient_energy < 0.15);

    // Menu: precipitation greyed out with the reason, clouds applied.
    bool saw_rain = false;
    for (const auto& item : tw.menu())
        if (item.id == "weather:rain") {
            saw_rain = true;
            CHECK_FALSE(item.enabled);
            CHECK_FALSE(item.reason.empty());
        }
    CHECK(saw_rain);
    CHECK_FALSE(tw.choose("weather:rain"));
    CHECK_FALSE(tw.choose("weather:nope"));
    REQUIRE(tw.choose("weather:partly_cloudy"));
    CHECK(tw.state().icon == rg::WeatherIcon::MoonCloud);
    REQUIRE(tw.choose("time:noon"));
    const double clear_sun = rg::TimeWeather(*config, loc).state().lighting.sun_energy;
    REQUIRE(tw.choose("weather:overcast"));
    s = tw.state();
    CHECK(s.icon == rg::WeatherIcon::Cloud);
    CHECK(s.lighting.sun_energy < clear_sun * 0.6);

    // Rate options.
    REQUIRE(tw.choose("rate:3"));
    CHECK(tw.clock().rate() == 60.0);
    tw.advance(60);
    CHECK(tw.state().clock_text == "14:00");

    CHECK(rg::format_temperature_c(288.15) == "15 °C");
    CHECK(rg::format_temperature_c(273.0) == "0 °C");
}

TEST_CASE("time/weather: grid direction applies the meridian convergence", "[sky]") {
    rg::SkyPosition south;
    south.azimuth_deg = 180;
    south.up = 0;
    const auto d0 = rg::grid_direction(south, 0);
    CHECK(d0[1] == Catch::Approx(-1.0));
    const auto d1 = rg::grid_direction(south, 1.0); // grid north 1 deg east of true north
    CHECK(d1[0] > 0);                               // grid azimuth 179 deg: slightly east of grid south
    CHECK(d1[0] == Catch::Approx(std::sin(179.0 * 3.14159265358979 / 180)).margin(1e-9));
}
