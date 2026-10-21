// test_wind_noise.cpp - rg::WindNoise and the tyre rolling-noise law (owner 2026-10-19: "the wind noise is more or less a binary
// switch that turns on somewhere under 50 km/h and then stays at the exact same state regardless of increasing speed", then
// "that was the tyre noise": it neither got louder nor changed pitch with wheel speed).
//
// What is pinned: both layers grow continuously and monotonically with speed over 0-450 km/h, with the CPX-style slope for
// the tyre layer (30-40 dB per decade) and the interior slope (40 dB per decade) for the wind layer, with no audible step between adjacent km/h,
// pitch (recording rate, hiss centre, tread tone, wind centre) rising with speed, the data files' rejections, and the mix
// (tyre dominates in town, wind on the motorway). Recorded sabotages (see the notes at the end of each section):
//   S1 tyre_roll_gain clamps v at 50 km/h             -> T2/T3/T4/T6/W9 fail (5 cases, 413 assertions)
//   S2 tyre rate_exponent forced to 0 (frozen pitch)  -> T5 fails (1 case, 90 assertions)
//   S3 wind_noise_amplitude clamps U at 50 km/h       -> W2/W3/W6/W8/W9 fail (5 cases, 13 assertions)
// (each measured 2026-10-19 by applying the one-line change, rebuilding and running "[wind_noise],[tyre_sound]"; reverted after)
#include "rg/tyre_sound.h"
#include "rg/wind_noise.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <complex>
#include <cstring>
#include <fstream>
#include <iterator>
#include <numeric>
#include <string>
#include <vector>

namespace {
std::string read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    REQUIRE(f);
    return std::string((std::istreambuf_iterator<char>(f)), {});
}
rg::TyreRollConfig shipped_tyre() {
    rg::TyreRollConfig c;
    std::string err;
    REQUIRE(rg::parse_tyre_roll_config(read_file(std::string(RG_SOURCE_DIR) + "/data/controls/tyre_sound.json"), c, err));
    return c;
}
rg::WindNoiseConfig shipped_wind() {
    rg::WindNoiseConfig c;
    std::string err;
    REQUIRE(rg::parse_wind_noise_config(read_file(std::string(RG_SOURCE_DIR) + "/data/controls/wind_noise.json"), c, err));
    return c;
}
double kmh(double v) { return v / 3.6; }
double db(double a, double b) { return 20.0 * std::log10(a / b); }
double rms(const std::vector<float>& v) {
    double s = 0.0;
    for (float x : v) s += double(x) * x;
    return v.empty() ? 0.0 : std::sqrt(s / double(v.size()));
}
std::vector<std::int16_t> shipped_recording_pcm(int& channels) {
    const std::string raw = read_file(std::string(RG_SOURCE_DIR) + "/game/audio/tyres/asphalt_roll_provisional.wav");
    std::size_t at = 12;
    while (at + 8 <= raw.size() && std::memcmp(raw.data() + at, "data", 4) != 0) {
        std::uint32_t len;
        std::memcpy(&len, raw.data() + at + 4, 4);
        at += 8 + len + (len & 1u);
    }
    REQUIRE(at + 8 <= raw.size());
    std::uint16_t ch;
    std::memcpy(&ch, raw.data() + 22, 2);
    channels = ch;
    at += 8;
    std::vector<std::int16_t> pcm((raw.size() - at) / 2);
    std::memcpy(pcm.data(), raw.data() + at, pcm.size() * 2);
    return pcm;
}
// Rendered RMS of the bank (channel 0 = the car-level layer; sealed wheels' procedural roll is muted) at a steady speed.
double tyre_layer_rms(const rg::TyreRollConfig& cfg, std::shared_ptr<const std::vector<float>> loop, double speed_mps, double surface_db = 0.0) {
    rg::TyreSoundBank bank(4, 1234, nullptr, loop, cfg);
    rg::TyreSoundInput in;
    in.speed = speed_mps; in.tread_speed = speed_mps; in.load_n = 4000.0; in.surface_db = surface_db;
    for (int w = 0; w < 4; ++w) bank.update(w, in);
    bank.render(48000, 1.0f); // let the 50 ms speed / 20 ms gain smoothing settle
    std::vector<float> ch0;
    const auto out = bank.render(96000, 1.0f);
    for (std::size_t i = 0; i < out.size(); i += 4) ch0.push_back(out[i]);
    return rms(ch0);
}
double wind_rms(const rg::WindNoiseConfig& cfg, double airspeed_mps, int frames = 192000) {
    rg::WindNoise w(cfg, 5);
    w.set_airspeed(airspeed_mps);
    w.render(24000, 1.0);
    return rms(w.render(frames, 1.0));
}
std::shared_ptr<const std::vector<float>> shipped_loop(const rg::TyreRollConfig& cfg) {
    int ch = 2;
    const auto pcm = shipped_recording_pcm(ch);
    return rg::make_roll_loop(pcm.data(), pcm.size(), ch, cfg.loop_crossfade_ms);
}
} // namespace

// ---- tyre law -----------------------------------------------------------------------------------------------------------

TEST_CASE("T1 tyre config: the shipped file parses and states the CPX law", "[wind_noise][tyre_sound]") {
    const auto c = shipped_tyre();
    CHECK(c.level.level_db_per_decade >= 30.0);
    CHECK(c.level.level_db_per_decade <= 40.0);
    CHECK(c.level.max_speed_mps >= kmh(450.0)); // above the hypercar
    CHECK(c.rate_exponent > 0.1);
    REQUIRE(c.find_surface("asphalt"));
    CHECK(c.find_surface("asphalt")->sealed);
    CHECK(c.find_surface("cobblestone")->db > c.find_surface("asphalt")->db);
    CHECK_FALSE(c.find_surface("grass")->sealed);
}

TEST_CASE("T2 tyre level grows monotonically from standstill to 450 km/h, no plateau", "[wind_noise][tyre_sound]") {
    const auto c = shipped_tyre();
    CHECK(rg::tyre_roll_gain(c.level, 0.0) == 0.0);
    double prev = 0.0;
    for (double v = 1.0; v <= 450.0; v += 1.0) {
        const double g = rg::tyre_roll_gain(c.level, kmh(v));
        INFO("km/h " << v);
        REQUIRE(g > prev);
        prev = g;
    }
    // The pre-fix law was flat above 200 km/h and 6 dB per doubling below: at least +8 dB from 200 to 400.
    CHECK(db(rg::tyre_roll_gain(c.level, kmh(400.0)), rg::tyre_roll_gain(c.level, kmh(200.0))) > 8.0);
}

TEST_CASE("T3 tyre slope is 30-40 dB per decade at every doubling from 20 to 480 km/h", "[wind_noise][tyre_sound]") {
    const auto c = shipped_tyre();
    for (double v = 20.0; v <= 240.0; v *= 2.0) {
        const double per_doubling = db(rg::tyre_roll_gain(c.level, kmh(2.0 * v)), rg::tyre_roll_gain(c.level, kmh(v)));
        INFO("from " << v << " km/h: " << per_doubling << " dB per doubling");
        CHECK(per_doubling >= 30.0 * std::log10(2.0) - 0.05); // 9.03 dB
        CHECK(per_doubling <= 40.0 * std::log10(2.0) + 0.05); // 12.04 dB
    }
}

TEST_CASE("T4 tyre level has no step between adjacent km/h and nothing audible at walking pace", "[wind_noise][tyre_sound]") {
    const auto c = shipped_tyre();
    const double ref = rg::tyre_roll_gain(c.level, kmh(100.0));
    CHECK(db(rg::tyre_roll_gain(c.level, kmh(5.0)), ref) < -45.0); // no audible switch-on: already far below at 5 km/h
    for (double v = 5.0; v < 450.0; v += 1.0) {
        const double step = db(rg::tyre_roll_gain(c.level, kmh(v + 1.0)), rg::tyre_roll_gain(c.level, kmh(v)));
        INFO("km/h " << v);
        CHECK(step > 0.0);
        CHECK(step <= (v < 30.0 ? 3.1 : 1.3)); // a power law's own step: 35*log10((v+1)/v)
    }
}

TEST_CASE("T5 tyre pitch follows the speed: playback rate, hiss centre and tread tone all rise, tone ~ wheel speed", "[wind_noise][tyre_sound]") {
    const auto c = shipped_tyre();
    double rate = 0.0, hiss = 0.0;
    for (double v = 10.0; v <= 450.0; v += 5.0) {
        const double r = rg::tyre_roll_rate(c, kmh(v)), h = rg::tyre_hiss_centre_hz(c, kmh(v));
        INFO("km/h " << v);
        CHECK(r > rate);
        CHECK(h > hiss);
        rate = r; hiss = h;
    }
    // Not frozen above 100 km/h any more (the old clamp(v / 50, 0.5, 2.0)).
    CHECK(rg::tyre_roll_rate(c, kmh(300.0)) > 1.3 * rg::tyre_roll_rate(c, kmh(100.0)) * 0.99);
    CHECK(rg::tyre_roll_rate(c, kmh(40.0)) > rg::tyre_roll_rate(c, kmh(20.0)) * 1.1); // nor below 25 km/h
    // Hiss peak about 1 kHz at 100 km/h, moving up slowly.
    CHECK(rg::tyre_hiss_centre_hz(c, kmh(100.0)) == Catch::Approx(1000.0).margin(150.0));
    CHECK(rg::tyre_hiss_centre_hz(c, kmh(300.0)) < 2.0 * rg::tyre_hiss_centre_hz(c, kmh(100.0)));
    // Tread-block tone = blocks per revolution x revolutions per second: exactly proportional to the tread speed.
    CHECK(rg::tyre_tread_tone_hz(c, kmh(120.0)) == Catch::Approx(2.0 * rg::tyre_tread_tone_hz(c, kmh(60.0))).epsilon(1e-9));
    CHECK(rg::tyre_tread_tone_hz(c, kmh(240.0)) == Catch::Approx(4.0 * rg::tyre_tread_tone_hz(c, kmh(60.0))).epsilon(1e-9));
    CHECK(rg::tyre_tread_tone_hz(c, kmh(100.0)) == Catch::Approx(kmh(100.0) / c.tread_pitch_m).epsilon(1e-9));
}

TEST_CASE("T6 rendered tyre layer: RMS follows the law at 30-150 km/h and keeps rising to 300", "[wind_noise][tyre_sound]") {
    const auto c = shipped_tyre();
    const auto loop = shipped_loop(c);
    REQUIRE(loop);
    double prev = 0.0;
    for (double v : {10.0, 20.0, 30.0, 50.0, 70.0, 100.0, 150.0, 200.0, 250.0, 300.0}) {
        const double r = tyre_layer_rms(c, loop, kmh(v));
        INFO("km/h " << v << " rms " << r);
        CHECK(r > prev);
        prev = r;
    }
    // Slope inside the band where the soft limiter is transparent: 30 -> 60 -> 120 km/h within the 30-40 dB/decade band.
    const double r30 = tyre_layer_rms(c, loop, kmh(30.0)), r60 = tyre_layer_rms(c, loop, kmh(60.0)), r120 = tyre_layer_rms(c, loop, kmh(120.0));
    for (double per_doubling : {db(r60, r30), db(r120, r60)}) {
        CHECK(per_doubling >= 8.5);
        CHECK(per_doubling <= 12.5);
    }
    // 300 km/h is still +8 dB above 200 km/h even through the limiter (the old layer was exactly flat there).
    CHECK(db(tyre_layer_rms(c, loop, kmh(300.0)), tyre_layer_rms(c, loop, kmh(200.0))) > 4.0);
}

TEST_CASE("T7 tyre pitch in the rendered signal: the recording plays faster with speed, the tread tone sits at v / pitch", "[wind_noise][tyre_sound]") {
    // A quiet 300 Hz recording; the hiss is off, the tone is made to dominate (levels are relative to the recording's RMS).
    std::vector<std::int16_t> pcm(48000 * 3 * 2);
    for (std::size_t i = 0; i < pcm.size() / 2; ++i) {
        const auto s = static_cast<std::int16_t>(60.0 * std::sin(6.2831853 * 300.0 * double(i) / 48000.0));
        pcm[2 * i] = s; pcm[2 * i + 1] = s;
    }
    const auto loop = rg::make_roll_loop(pcm.data(), pcm.size(), 2, 75.0);
    REQUIRE(loop);
    auto cfg = shipped_tyre();
    cfg.hiss_db = -60.0; cfg.tone_db = 40.0; cfg.tone_sine_share = 1.0;
    auto rendered = [&](double v) {
        rg::TyreSoundBank bank(4, 1, nullptr, loop, cfg);
        rg::TyreSoundInput in; in.speed = kmh(v); in.tread_speed = kmh(v); in.load_n = 4000.0;
        for (int w = 0; w < 4; ++w) bank.update(w, in);
        bank.render(48000, 1.0f);
        std::vector<float> ch0;
        const auto out = bank.render(48000, 1.0f);
        for (std::size_t i = 0; i < out.size(); i += 4) ch0.push_back(out[i]);
        return ch0;
    };
    // Goertzel-style DFT magnitude at one frequency (the layer runs at 24 kHz held to 48 kHz).
    auto magnitude = [](const std::vector<float>& x, double hz) {
        std::complex<double> s = 0.0;
        for (std::size_t i = 0; i < std::min<std::size_t>(x.size(), 12000); ++i) s += double(x[i]) * std::exp(std::complex<double>(0.0, -6.283185307179586 * hz * double(i) / 48000.0));
        return std::abs(s);
    };
    for (double v : {60.0, 100.0, 160.0, 240.0}) {
        const auto x = rendered(v);
        const double f_tone = kmh(v) / cfg.tread_pitch_m;
        INFO("km/h " << v << " tone " << f_tone << " Hz");
        // The tone is the strongest line within +-6 % of the expected frequency and 10x above the same offset off the line.
        double best = 0.0, best_hz = 0.0;
        for (double hz = 0.8 * f_tone; hz <= 1.2 * f_tone; hz += 2.0) {
            const double m = magnitude(x, hz);
            if (m > best) { best = m; best_hz = hz; }
        }
        CHECK(best_hz == Catch::Approx(f_tone).epsilon(0.02));
        CHECK(best > 5.0 * magnitude(x, 0.7 * f_tone));
    }
}

TEST_CASE("T8 tyre surface offsets scale the layer, airborne wheels are silent", "[wind_noise][tyre_sound]") {
    const auto c = shipped_tyre();
    const auto loop = shipped_loop(c);
    const double asphalt = tyre_layer_rms(c, loop, kmh(80.0), 0.0);
    const double cobble = tyre_layer_rms(c, loop, kmh(80.0), c.find_surface("cobblestone")->db);
    CHECK(db(cobble, asphalt) == Catch::Approx(c.find_surface("cobblestone")->db).margin(0.3));
    rg::TyreSoundBank bank(4, 1, nullptr, loop, c);
    rg::TyreSoundInput air; air.speed = kmh(200.0); air.tread_speed = kmh(200.0); air.load_n = 0.0;
    for (int w = 0; w < 4; ++w) bank.update(w, air);
    bank.render(48000, 1.0f);
    CHECK(rms(bank.render(24000, 1.0f)) == 0.0);
}

TEST_CASE("T9 tyre config rejections", "[wind_noise][tyre_sound]") {
    auto reject = [](const std::string& body, const std::string& needle) {
        rg::TyreRollConfig c;
        std::string err;
        INFO(body);
        CHECK_FALSE(rg::parse_tyre_roll_config("{\"roll\":{" + body + "}}", c, err));
        CHECK(err.find(needle) != std::string::npos);
    };
    rg::TyreRollConfig c;
    std::string err;
    CHECK_FALSE(rg::parse_tyre_roll_config("not json", c, err));
    CHECK_FALSE(rg::parse_tyre_roll_config("{}", c, err));
    reject("\"bogus\": 1", "unknown key");
    reject("\"max_speed_kmh\": 50", "max_speed_kmh"); // the clamp that made the old layer flat
    reject("\"max_speed_kmh\": 250", "max_speed_kmh");
    reject("\"level_db_per_decade\": 5", "level_db_per_decade");
    reject("\"level_db_per_decade\": 120", "level_db_per_decade");
    reject("\"rate_exponent\": 0", "rate_exponent"); // frozen pitch
    reject("\"gain_at_reference\": -1", "gain_at_reference");
    reject("\"gain_at_reference\": \"loud\"", "must be a finite number");
    reject("\"tread_pitch_m\": 0", "tread_pitch_m");
    reject("\"roll_source\": \"mp3\"", "roll_source");
    reject("\"full_above_kmh\": 0", "full_above_kmh");
    reject("\"min_rate\": 0.9", "min_rate");
    reject("\"surfaces\": {\"asphalt\": {\"sealed\": true, \"db\": 99}}", "db");
    reject("\"surfaces\": {\"asphalt\": {\"sealed\": 1}}", "bad or unknown key");
    CHECK(rg::parse_tyre_roll_config("{\"roll\":{}}", c, err)); // defaults are valid
}

// ---- wind law -------------------------------------------------------------------------------------------------------------

TEST_CASE("W1 wind config: the shipped file parses and states the interior slope", "[wind_noise]") {
    const auto c = shipped_wind();
    CHECK(c.level_db_per_decade >= 38.0); // interior: steeper than the tyre layer (35), below the exterior 50-60
    CHECK(c.level_db_per_decade <= 45.0);
    CHECK(c.max_speed_mps >= kmh(450.0));
    CHECK(c.centre_exponent > 0.5);
}

TEST_CASE("W2 wind level grows monotonically with the airspeed up to 450 km/h", "[wind_noise]") {
    const auto c = shipped_wind();
    CHECK(rg::wind_noise_amplitude(c, 0.0) == 0.0);
    double prev = 0.0;
    for (double v = 1.0; v <= 450.0; v += 1.0) {
        const double a = rg::wind_noise_amplitude(c, kmh(v));
        INFO("km/h " << v);
        REQUIRE(a > prev);
        prev = a;
    }
    CHECK(db(rg::wind_noise_amplitude(c, kmh(400.0)), rg::wind_noise_amplitude(c, kmh(200.0))) > 11.0);
}

TEST_CASE("W3 wind slope is 38-45 dB per decade at every doubling, no step between adjacent km/h, nothing at walking pace", "[wind_noise]") {
    const auto c = shipped_wind();
    for (double v = 20.0; v <= 240.0; v *= 2.0) {
        const double per_doubling = db(rg::wind_noise_amplitude(c, kmh(2.0 * v)), rg::wind_noise_amplitude(c, kmh(v)));
        INFO("from " << v << " km/h: " << per_doubling << " dB per doubling");
        CHECK(per_doubling >= 38.0 * std::log10(2.0) - 0.05); // 11.44 dB
        CHECK(per_doubling <= 45.0 * std::log10(2.0) + 0.05); // 13.55 dB
    }
    CHECK(db(rg::wind_noise_amplitude(c, kmh(5.0)), rg::wind_noise_amplitude(c, kmh(100.0))) < -50.0);
    for (double v = 10.0; v < 450.0; v += 1.0) {
        const double step = db(rg::wind_noise_amplitude(c, kmh(v + 1.0)), rg::wind_noise_amplitude(c, kmh(v)));
        INFO("km/h " << v);
        CHECK(step <= (v < 30.0 ? 6.0 : 2.0));
    }
}

TEST_CASE("W4 wind centre frequency rises with the airspeed (Strouhal-like f ~ U)", "[wind_noise]") {
    const auto c = shipped_wind();
    double prev = 0.0;
    for (double v = 20.0; v <= 450.0; v += 5.0) {
        const double f = rg::wind_noise_centre_hz(c, kmh(v));
        INFO("km/h " << v);
        CHECK(f > prev);
        prev = f;
    }
    CHECK(rg::wind_noise_centre_hz(c, kmh(200.0)) == Catch::Approx(2.0 * rg::wind_noise_centre_hz(c, kmh(100.0))).epsilon(1e-9));
}

TEST_CASE("W5 wind voice: silent at rest, bounded, deterministic, block-size independent, mixes into every channel", "[wind_noise]") {
    const auto c = shipped_wind();
    rg::WindNoise rest(c, 1);
    CHECK(rms(rest.render(48000, 1.0)) == 0.0);
    rg::WindNoise a(c, 3), b(c, 3);
    a.set_airspeed(kmh(250.0)); b.set_airspeed(kmh(250.0));
    const auto whole = a.render(8192, 1.0);
    auto split = b.render(4096, 1.0);
    const auto tail = b.render(4096, 1.0);
    split.insert(split.end(), tail.begin(), tail.end());
    CHECK(whole == split);
    for (float s : whole) REQUIRE(std::isfinite(s));
    CHECK(rms(a.render(4096, 0.0)) == 0.0);
    CHECK(a.render(-1, 1.0).empty());
    CHECK(a.render(10000000, 1.0).size() == 16384u);
    a.set_airspeed(std::nan(""));
    for (float s : a.render(512, 1.0)) REQUIRE(std::isfinite(s));
    rg::WindNoise m(c, 3);
    m.set_airspeed(kmh(250.0));
    std::vector<float> buf(4 * 1024, 0.5f);
    m.mix_into(buf.data(), 4, 1024, 1.0);
    for (int f = 0; f < 1024; ++f)
        for (int ch = 1; ch < 4; ++ch) REQUIRE(buf[std::size_t(f) * 4 + ch] == buf[std::size_t(f) * 4]);
    bool changed = false;
    for (float s : buf) changed |= (s != 0.5f);
    CHECK(changed);
}

TEST_CASE("W6 rendered wind: RMS follows 40 dB per decade, spectrum moves up, no flat region", "[wind_noise]") {
    const auto c = shipped_wind();
    double prev = 0.0;
    for (double v : {20.0, 30.0, 50.0, 70.0, 100.0, 150.0, 200.0, 250.0, 300.0, 400.0}) {
        const double r = wind_rms(c, kmh(v));
        INFO("km/h " << v << " rms " << r);
        CHECK(r > prev);
        prev = r;
    }
    const double r50 = wind_rms(c, kmh(50.0)), r100 = wind_rms(c, kmh(100.0)), r200 = wind_rms(c, kmh(200.0)), r400 = wind_rms(c, kmh(400.0));
    for (double per_doubling : {db(r100, r50), db(r200, r100), db(r400, r200)}) {
        CHECK(per_doubling >= 11.0); // 12.04 nominal; the band shift and the gust modulation move the rendered value a little
        CHECK(per_doubling <= 15.5);
    }
    // The spectrum's weight moves up: the share of the energy below ~400 Hz (one-pole low-pass) falls with every doubling.
    auto low_share = [&](double v) {
        rg::WindNoise w(c, 5);
        w.set_airspeed(kmh(v));
        w.render(24000, 1.0);
        const auto x = w.render(96000, 1.0);
        double lp = 0.0, lo = 0.0, all = 0.0;
        const double a = 1.0 - std::exp(-6.283185307179586 * 400.0 / 48000.0);
        for (float s : x) { lp += a * (double(s) - lp); lo += lp * lp; all += double(s) * s; }
        return lo / all;
    };
    CHECK(low_share(50.0) > 1.15 * low_share(100.0));
    CHECK(low_share(100.0) > 1.15 * low_share(200.0));
    CHECK(low_share(200.0) > 1.15 * low_share(400.0));
}

TEST_CASE("W7 wind config rejections", "[wind_noise]") {
    auto reject = [](const std::string& body, const std::string& needle) {
        rg::WindNoiseConfig c;
        std::string err;
        INFO(body);
        CHECK_FALSE(rg::parse_wind_noise_config("{\"wind\":{" + body + "}}", c, err));
        CHECK(err.find(needle) != std::string::npos);
    };
    rg::WindNoiseConfig c;
    std::string err;
    CHECK_FALSE(rg::parse_wind_noise_config("[1]", c, err));
    CHECK_FALSE(rg::parse_wind_noise_config("{\"tyre\":{}}", c, err));
    reject("\"gain\": 1", "unknown key");
    reject("\"max_speed_kmh\": 50", "max_speed_kmh");
    reject("\"max_speed_kmh\": 300", "max_speed_kmh");
    reject("\"level_db_per_decade\": 10", "level_db_per_decade");
    reject("\"level_db_per_decade\": 200", "level_db_per_decade");
    reject("\"rms_at_reference\": 0", "rms_at_reference");
    reject("\"rms_at_reference\": 3", "rms_at_reference");
    reject("\"q\": \"wide\"", "must be a number");
    reject("\"centre_exponent\": -1", "centre_exponent");
    reject("\"gust_depth\": 0.9", "gust_depth");
    reject("\"full_above_kmh\": 40", "full_above_kmh");
    CHECK(rg::parse_wind_noise_config("{\"wind\":{}}", c, err));
}

TEST_CASE("W8 airspeed is relative to the air: a headwind makes the same ground speed louder", "[wind_noise]") {
    const auto c = shipped_wind();
    // The sim's aero airspeed = |ground velocity - wind|; the voice only sees the number.
    const double ground = kmh(100.0);
    const double headwind_air = ground + 10.0, tailwind_air = ground - 10.0;
    CHECK(wind_rms(c, headwind_air) > 1.5 * wind_rms(c, ground));
    CHECK(wind_rms(c, tailwind_air) < 0.7 * wind_rms(c, ground));
    CHECK(wind_rms(c, 10.0) > 0.0); // standing in a 10 m/s wind is not silent
}

// ---- mix ------------------------------------------------------------------------------------------------------------------

TEST_CASE("W9 balance: tyre noise dominates in town and on the motorway, wind above about 200 km/h", "[wind_noise][tyre_sound]") {
    const auto tc = shipped_tyre();
    const auto wc = shipped_wind();
    const auto loop = shipped_loop(tc);
    REQUIRE(loop);
    auto margin = [&](double v) { return db(tyre_layer_rms(tc, loop, kmh(v)), wind_rms(wc, kmh(v))); }; // + = tyre louder
    // 2026-10-21 (owner: the wind scaled too aggressively): wind at 40 dB per decade against the tyre's 35 with the 100 km/h
    // level kept moves the crossover from ~130 to ~200 km/h; measured margins 4.2 / 3.3 / 2.6 dB at 30 / 50 / 70 km/h,
    // -0.9 dB at 300 km/h (was 9.6 / 6.2 / 4.0 / -5.4 at 50 dB per decade).
    CHECK(margin(30.0) > 3.5);
    CHECK(margin(50.0) > 2.5);
    CHECK(margin(70.0) > 2.0);
    CHECK(margin(300.0) < 0.0);
    CHECK(margin(400.0) < -1.0);
    double crossover = 0.0;
    for (double v = 40.0; v <= 300.0; v += 2.0)
        if (margin(v) < 0.0) { crossover = v; break; }
    INFO("crossover km/h " << crossover);
    CHECK(crossover >= 170.0);
    CHECK(crossover <= 230.0);
    // Total level at 300 km/h stays below full scale (no clipping of the sum at the loudest speed).
    CHECK(tyre_layer_rms(tc, loop, kmh(300.0)) + wind_rms(wc, kmh(300.0)) < 0.5);
}

// ---- report: speed sweep, CSV of the sound-source parameters and a WAV ----------------------------------------------------------
#include <algorithm>
#include <cstdio>
#include <filesystem>
TEST_CASE("sweep report: 0 -> 300 km/h, parameters per 0.1 s and a WAV in out/audio_sweep", "[.][diag][wind_noise_sweep]") {
    const auto tc = shipped_tyre();
    const auto wc = shipped_wind();
    const auto loop = shipped_loop(tc);
    REQUIRE(loop);
    const std::filesystem::path dir = std::filesystem::path(RG_SOURCE_DIR) / "out" / "audio_sweep";
    std::filesystem::create_directories(dir);
    rg::TyreSoundBank bank(4, 1234, nullptr, loop, tc);
    rg::WindNoise wind(wc, 5);
    constexpr double seconds = 30.0, top_kmh = 300.0;
    constexpr int block = 480; // 10 ms
    std::vector<float> mix, tyre_only, wind_only;
    std::ofstream csv(dir / "sweep_parameters.csv");
    REQUIRE(csv);
    csv << "t_s,kmh,tyre_gain,tyre_rate,tyre_hiss_hz,tyre_tone_hz,tyre_rms,wind_amp,wind_centre_hz,wind_rms\n";
    for (int b = 0; b < int(seconds * 48000.0 / block); ++b) {
        const double t = b * block / 48000.0;
        const double v = kmh(top_kmh * t / seconds);
        rg::TyreSoundInput in; in.speed = v; in.tread_speed = v; in.load_n = 4000.0;
        for (int w = 0; w < 4; ++w) bank.update(w, in);
        wind.set_airspeed(v);
        const auto tyre = bank.render(block, 1.0f);
        const auto wnd = wind.render(block, 1.0);
        double st = 0.0, sw = 0.0;
        for (int i = 0; i < block; ++i) {
            const float ty = tyre[std::size_t(i) * 4];
            tyre_only.push_back(ty); wind_only.push_back(wnd[std::size_t(i)]); mix.push_back(ty + wnd[std::size_t(i)]);
            st += double(ty) * ty; sw += double(wnd[std::size_t(i)]) * wnd[std::size_t(i)];
        }
        if (b % 10 == 0) {
            const auto s = bank.roll_state();
            char line[256];
            std::snprintf(line, sizeof line, "%.2f,%.1f,%.5f,%.3f,%.1f,%.1f,%.5f,%.5f,%.1f,%.5f\n", t, v * 3.6, s.gain, s.rate, s.hiss_hz, s.tone_hz,
                          std::sqrt(st / block), wind.current_amplitude(), wind.current_centre_hz(), std::sqrt(sw / block));
            csv << line;
        }
    }
    csv.close();
    auto write_wav = [&](const char* name, const std::vector<float>& x) {
        std::ofstream f(dir / name, std::ios::binary);
        REQUIRE(f);
        const std::uint32_t bytes = std::uint32_t(x.size() * 2), rate = 48000, byte_rate = rate * 2, fmt = 16, riff = 36 + bytes;
        const std::uint16_t pcm = 1, ch = 1, align = 2, bits = 16;
        auto w = [&](const void* p, std::size_t n) { f.write(static_cast<const char*>(p), static_cast<std::streamsize>(n)); };
        w("RIFF", 4); w(&riff, 4); w("WAVEfmt ", 8); w(&fmt, 4); w(&pcm, 2); w(&ch, 2); w(&rate, 4); w(&byte_rate, 4); w(&align, 2); w(&bits, 2);
        w("data", 4); w(&bytes, 4);
        for (float sample : x) {
            const auto v = static_cast<std::int16_t>(std::lround(std::clamp(double(sample), -1.0, 1.0) * 32767.0));
            w(&v, 2);
        }
    };
    // The absolute level is quiet at low speed by design (35 / 50 dB per decade); the files are the raw layers, no normalisation.
    write_wav("sweep_tyre_and_wind_0_300kmh.wav", mix);
    write_wav("sweep_tyre_only_0_300kmh.wav", tyre_only);
    write_wav("sweep_wind_only_0_300kmh.wav", wind_only);
    SUCCEED();
}
