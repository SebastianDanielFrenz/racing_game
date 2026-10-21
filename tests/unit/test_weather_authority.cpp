// Changing weather (owner 2026-10-21): rg::WeatherAuthority (the server role) runs a seeded chain over the sky.json
// presets from data/world/weather_schedule.json; clients only apply its rg.weather_broadcast/1 messages. Checks the
// loader, determinism, rewind, the season/temperature gate, the broadcast round trip (a remote client fed only the
// JSON matches the local in-process authority), the menu (Changing vs a picked preset, requests on a remote client)
// and the physics side (a scheduled change ramps over the weather now instead of resetting it).
#include "rg/climate.h"
#include "rg/session.h"
#include "rg/time_weather.h"
#include "rg/weather_authority.h"
#include "rg/weather_sync.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {
constexpr double kLat = 50.1109, kLon = 8.6821;
const std::string kRoot = RG_SOURCE_DIR;

const rg::SkyConfig& sky() {
    static const rg::SkyConfig c = [] {
        std::string err;
        auto r = rg::load_sky_config(kRoot + "/data/world/sky.json", &err);
        if (!r) FAIL(err);
        return *r;
    }();
    return c;
}
std::shared_ptr<const rg::ClimateConfig> climate() {
    static const auto c = [] {
        std::string err;
        auto r = rg::load_climate_config(kRoot + "/data/world/climate.json", &err);
        if (!r) FAIL(err);
        return std::make_shared<const rg::ClimateConfig>(*r);
    }();
    return c;
}
std::vector<std::string> preset_ids() {
    std::vector<std::string> ids;
    for (const auto& p : sky().weather_presets) ids.push_back(p.id);
    return ids;
}
const rg::WeatherScheduleConfig& schedule() {
    static const rg::WeatherScheduleConfig c = [] {
        std::string err;
        auto r = rg::load_weather_schedule(kRoot + "/data/world/weather_schedule.json", preset_ids(), &err);
        if (!r) FAIL(err);
        return *r;
    }();
    return c;
}
double game_s_at(rg::CivilDate d, double ut_h) { return (rg::julian_day(d, ut_h) - 2451545.0) * 86400.0; }

rg::WeatherAuthority authority(std::uint64_t seed, double epoch, const std::string& initial = "clear") {
    rg::WeatherScheduleConfig s = schedule();
    s.seed = seed;
    return rg::WeatherAuthority(s, sky().weather_presets, climate(), rg::GeoLocation{kLat, kLon, 0.0}, epoch, initial);
}

// The weather id every game hour over `days`.
std::vector<std::string> hourly(rg::WeatherAuthority& a, double epoch, int days) {
    std::vector<std::string> out;
    for (int h = 0; h <= days * 24; ++h) {
        a.update(epoch + h * 3600.0);
        out.push_back(a.current().weather_id);
    }
    return out;
}
} // namespace

TEST_CASE("weather schedule: shipped file loads and every state is a wired preset", "[weather][weather_schedule]") {
    const auto& s = schedule();
    CHECK(s.default_dynamic);
    CHECK(s.states.size() == 7);
    CHECK(s.seasons.size() == 3);
    CHECK(s.transition_game_s < s.min_duration_game_s);
    rg::TimeWeather tw(sky(), rg::GeoLocation{kLat, kLon, 0.0});
    tw.set_climate(climate());
    for (const auto& st : s.states) {
        const rg::WeatherPreset* p = nullptr;
        for (const auto& q : sky().weather_presets)
            if (q.id == st.id) p = &q;
        REQUIRE(p != nullptr);
        CHECK(tw.weather_wired(*p));
    }
}

TEST_CASE("weather schedule: loader rejections", "[weather][weather_schedule]") {
    std::string err;
    CHECK_FALSE(rg::load_weather_schedule(kRoot + "/data/world/no_such_file.json", preset_ids(), &err));
    CHECK_THAT(err, Catch::Matchers::ContainsSubstring("cannot read"));
    // A preset list without "hail": the shipped file names a state the sky does not know.
    std::vector<std::string> ids = preset_ids();
    ids.erase(std::remove(ids.begin(), ids.end(), "hail"), ids.end());
    CHECK_FALSE(rg::load_weather_schedule(kRoot + "/data/world/weather_schedule.json", ids, &err));
    CHECK_THAT(err, Catch::Matchers::ContainsSubstring("hail"));
}

TEST_CASE("weather authority: same seed same weather, another seed other weather", "[weather][weather_schedule]") {
    const double epoch = game_s_at({2026, 4, 1}, 10.0);
    auto a = authority(7, epoch), b = authority(7, epoch), c = authority(8, epoch);
    const auto ha = hourly(a, epoch, 30), hb = hourly(b, epoch, 30), hc = hourly(c, epoch, 30);
    CHECK(ha == hb);
    CHECK(ha != hc);
    int changes = 0;
    for (std::size_t i = 1; i < ha.size(); ++i) changes += ha[i] != ha[i - 1];
    INFO("changes in 30 days: " << changes);
    CHECK(changes > 30);  // about one change every 5-6 game hours on average
    CHECK(changes < 200);
}

TEST_CASE("weather authority: a clock jump back replays the chain", "[weather][weather_schedule]") {
    const double epoch = game_s_at({2026, 7, 1}, 0.0);
    auto a = authority(3, epoch);
    const auto fresh = hourly(a, epoch, 5);
    a.update(epoch + 40 * 86400.0); // far ahead
    a.update(epoch - 3600.0);       // before the epoch: the start weather
    CHECK(a.current().weather_id == "clear");
    const auto again = hourly(a, epoch, 5);
    CHECK(fresh == again);
}

TEST_CASE("weather authority: segments, transitions and the temperature gate over a year", "[weather][weather_schedule]") {
    const double epoch = game_s_at({2026, 1, 1}, 0.0);
    auto a = authority(20261021, epoch);
    const auto& s = schedule();
    std::map<std::string, int> segments;
    std::map<int, int> snow_by_month, thunder_by_month;
    double last_change = epoch;
    std::uint64_t last_rev = a.current().revision;
    for (double t = epoch; t < epoch + 365 * 86400.0; t += 600.0) {
        if (!a.update(t)) continue;
        const rg::WeatherBroadcast& b = a.current();
        CHECK(b.revision > last_rev);
        last_rev = b.revision;
        CHECK(b.dynamic);
        CHECK(b.transition_s == s.transition_game_s);
        CHECK(b.next_change_game_s - b.change_game_s >= s.min_duration_game_s - 1e-6);
        CHECK(b.change_game_s - last_change >= s.min_duration_game_s - 1e-6);
        last_change = b.change_game_s;
        ++segments[b.weather_id];
        rg::CivilDate d;
        const double jd = 2451545.0 + (b.change_game_s + 0.5 * s.transition_game_s) / 86400.0;
        // Month of the change (UT); the gate's temperature: the climate formula at that instant with the preset's cover.
        const double days = std::floor(jd + 0.5) - 2440588;
        d = rg::add_days({1970, 1, 1}, static_cast<int>(days));
        const double ut = (jd + 0.5 - std::floor(jd + 0.5)) * 24.0;
        double cover = 0;
        for (const auto& p : sky().weather_presets)
            if (p.id == b.weather_id) cover = p.state.cloud_cover;
        const double temp = rg::climate_station_temperature_c(*climate(), d, ut, kLon, cover);
        if (b.weather_id == "snow") { CHECK(temp <= 1.5); ++snow_by_month[d.month]; }
        if (b.weather_id == "rain") CHECK(temp >= 0.5);
        if (b.weather_id == "thunder") { CHECK(temp >= 12.0); ++thunder_by_month[d.month]; }
        if (b.weather_id == "hail") CHECK(temp >= 8.0);
    }
    std::string summary;
    for (const auto& [id, n] : segments) summary += id + " " + std::to_string(n) + ", ";
    INFO("segments in a year: " << summary);
    for (const char* id : {"clear", "partly_cloudy", "overcast", "rain", "snow", "thunder"}) CHECK(segments[id] > 0);
    for (int m : {6, 7, 8}) CHECK(snow_by_month[m] == 0);
    int summer_thunder = thunder_by_month[6] + thunder_by_month[7] + thunder_by_month[8];
    int winter_thunder = thunder_by_month[12] + thunder_by_month[1] + thunder_by_month[2];
    CHECK(summer_thunder > winter_thunder);
}

TEST_CASE("weather broadcast: JSON round trip and rejections", "[weather][weather_schedule]") {
    rg::WeatherBroadcast b;
    b.revision = 42;
    b.dynamic = true;
    b.weather_id = "rain";
    b.from_id = "overcast";
    b.change_game_s = 846000000.25;
    b.transition_s = 1800;
    b.next_change_game_s = 846010000.5;
    std::string err;
    const auto r = rg::parse_weather_broadcast(rg::to_json(b), &err);
    INFO(err);
    REQUIRE(r);
    CHECK(r->revision == 42);
    CHECK(r->dynamic);
    CHECK(r->weather_id == "rain");
    CHECK(r->from_id == "overcast");
    CHECK(r->change_game_s == b.change_game_s);
    CHECK(r->transition_s == 1800);
    CHECK(r->next_change_game_s == b.next_change_game_s);
    b.dynamic = false;
    b.next_change_game_s = std::nan("");
    const auto f = rg::parse_weather_broadcast(rg::to_json(b), &err);
    REQUIRE(f);
    CHECK_FALSE(f->dynamic);
    CHECK(std::isnan(f->next_change_game_s));
    CHECK_FALSE(rg::parse_weather_broadcast("{\"format\":\"rg.weather_broadcast/1\",\"revision\":-1}", &err));
    CHECK_FALSE(rg::parse_weather_broadcast("not json", &err));
}

TEST_CASE("weather: a remote client fed only broadcasts matches the local authority", "[weather][weather_schedule][sky]") {
    rg::SkyConfig cfg = sky();
    cfg.start_date = rg::CivilDate{2026, 6, 10};
    rg::TimeWeather local(cfg, rg::GeoLocation{kLat, kLon, 0.0});
    local.set_climate(climate());
    local.set_weather_schedule(schedule());
    local.clock().set_rate(600);

    // The server: an authority with the same inputs, next to its own clock. The client never runs the chain.
    rg::TimeWeather server_clock(cfg, rg::GeoLocation{kLat, kLon, 0.0});
    server_clock.clock().set_rate(600);
    rg::WeatherAuthority server(schedule(), cfg.weather_presets, climate(), rg::GeoLocation{kLat, kLon, 0.0},
                                rg::game_seconds(server_clock.clock()), cfg.default_weather);
    rg::TimeWeather client(cfg, rg::GeoLocation{kLat, kLon, 0.0});
    client.set_climate(climate());
    client.use_remote_weather();
    client.clock().set_rate(600);
    std::string err;
    auto send = [&] {
        const auto msg = rg::parse_weather_broadcast(rg::to_json(server.current()), &err);
        REQUIRE(msg);
        client.apply_broadcast(*msg);
    };
    send();
    int changes = 0;
    for (int frame = 0; frame < 2 * 24 * 6 * 10; ++frame) { // two game days at x600, 10 frames per real second
        local.advance(0.1);
        server_clock.advance(0.1);
        client.advance(0.1);
        if (server.update(rg::game_seconds(server_clock.clock()))) { send(); ++changes; }
        const auto a = local.state(), b = client.state();
        REQUIRE(a.weather_id == b.weather_id);
        REQUIRE(std::fabs(a.weather.cloud_cover - b.weather.cloud_cover) < 1e-9);
        REQUIRE(a.weather.precipitation == b.weather.precipitation);
        if (a.weather_blend > 0 && a.weather_blend < 1) {
            CHECK(std::isfinite(a.weather_ramp_s));
        }
    }
    CHECK(changes > 2);
}

TEST_CASE("weather: menu Changing vs a picked preset, requests on a remote client", "[weather][weather_schedule][sky]") {
    rg::SkyConfig cfg = sky();
    cfg.start_date = rg::CivilDate{2026, 6, 10};
    rg::TimeWeather tw(cfg, rg::GeoLocation{kLat, kLon, 0.0});
    tw.set_climate(climate());
    auto selected = [&](const std::string& id) {
        for (const auto& item : tw.menu())
            if (item.id == id) return item.selected;
        return false;
    };
    bool has_auto = false;
    for (const auto& item : tw.menu()) has_auto |= item.id == "weather:auto";
    CHECK_FALSE(has_auto); // nothing runs a schedule yet
    tw.set_weather_schedule(schedule());
    CHECK(selected("weather:auto"));
    CHECK(tw.state().weather_dynamic);

    REQUIRE(tw.choose("weather:rain"));
    CHECK(selected("weather:rain"));
    CHECK_FALSE(selected("weather:auto"));
    tw.clock().set_rate(600);
    for (int i = 0; i < 2000; ++i) tw.advance(0.1); // ~2 game days: a picked preset stays
    CHECK(tw.state().weather_id == "rain");
    CHECK_FALSE(tw.state().weather_dynamic);

    REQUIRE(tw.choose("weather:auto")); // changing again, starting from the rain
    CHECK(tw.state().weather_dynamic);
    CHECK(tw.state().weather_id == "rain");
    std::map<std::string, int> seen;
    for (int i = 0; i < 4000; ++i) { tw.advance(0.1); ++seen[tw.state().weather_id]; }
    CHECK(seen.size() > 1);

    rg::TimeWeather client(cfg, rg::GeoLocation{kLat, kLon, 0.0});
    client.set_climate(climate());
    client.use_remote_weather();
    REQUIRE(client.choose("weather:snow"));
    CHECK(client.state().weather_id == cfg.default_weather); // the server has not answered
    const auto req = client.take_weather_requests();
    REQUIRE(req.size() == 1);
    CHECK(req[0] == "weather:snow");
    CHECK(client.take_weather_requests().empty());
}

TEST_CASE("weather: a scheduled change ramps over the physics weather now", "[weather][weather_schedule][v8]") {
    rg::SessionConfig config;
    config.vehicle_json_path = kRoot + "/external/physics_sim/data/vehicles/car_sedan.json";
    config.surface_table_path = kRoot + "/data/surfaces/surfaces.json";
    std::string err;
    auto w = rg::make_session_weather(*climate(), kLat, kLon, 0.0, {2026, 10, 14}, 12.0, 1.0, "clear", &err);
    REQUIRE(w);
    config.weather = std::make_shared<const rg::SessionWeather>(std::move(*w));
    rg::Session s(config);
    const auto at = [&](double ut_h) { return rg::env_clock_point({2026, 10, 14}, ut_h, kLon); };
    const auto rain = rg::physics_weather_preset(*climate(), "rain", 15.0);
    const auto overcast = rg::physics_weather_preset(*climate(), "overcast", 15.0);
    REQUIRE(rain);
    REQUIRE(overcast);
    s.set_weather(*rain, 0.0);
    s.step();
    s.set_environment_clock(at(13.0).env_s, at(13.0).day_index, 1.0); // an hour of rain
    for (int i = 0; i < 5; ++i) s.step();
    const double raining = s.weather_state().precip_mm_h;
    REQUIRE(raining > 0.0);
    // Overcast ramps in over 600 env s on top of the rain; halfway it still rains about half as hard. Resetting to
    // the base preset first (the old behaviour) would show a dry clear-to-overcast blend here.
    s.set_weather(*overcast, 600.0);
    s.step();
    s.set_environment_clock(at(13.0).env_s + 300.0, at(13.0).day_index, 1.0);
    for (int i = 0; i < 5; ++i) s.step();
    const double halfway = s.weather_state().precip_mm_h;
    INFO("rain " << raining << " mm/h, halfway to overcast " << halfway << " mm/h");
    CHECK(halfway > 0.2 * raining);
    CHECK(halfway < raining);
    // A ramp of 0 replaces at once.
    s.set_weather(*overcast, 0.0);
    s.step();
    s.set_environment_clock(at(13.0).env_s + 3900.0, at(13.0).day_index, 1.0);
    for (int i = 0; i < 5; ++i) s.step();
    CHECK(s.weather_state().precip_mm_h == 0.0);
}
