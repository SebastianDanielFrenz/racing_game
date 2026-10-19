// test_tyre_sound.cpp - rg::TyreSoundBank (port of the GDScript tyre voice): silence without ground contact, level and
// squeal when sliding, determinism, frame count.
#include "rg/tyre_sound.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace {
double rms(const std::vector<float>& v) {
    double s = 0.0;
    for (float x : v) s += double(x) * x;
    return v.empty() ? 0.0 : std::sqrt(s / v.size());
}
std::shared_ptr<const std::vector<std::int16_t>> squeal() {
    auto p = std::make_shared<std::vector<std::int16_t>>(48000);
    for (std::size_t i = 0; i < p->size(); ++i) (*p)[i] = static_cast<std::int16_t>(12000.0 * std::sin(0.3 * double(i)));
    return p;
}
} // namespace

TEST_CASE("tyre sound: no contact is silent, rolling hisses, sliding squeals", "[tyre_sound]") {
    rg::TyreSoundBank bank(4, 1234, squeal());
    rg::TyreSoundInput air;
    air.speed = 30.0;
    air.load_n = 0.0;
    for (int w = 0; w < 4; ++w) bank.update(w, air);
    auto silent = bank.render(2400, 1.0f);
    REQUIRE(silent.size() == 2400u * 4u);
    CHECK(rms(silent) == 0.0);

    rg::TyreSoundInput roll;
    roll.speed = 30.0;
    roll.tread_speed = 30.0;
    roll.load_n = 4000.0;
    for (int w = 0; w < 4; ++w) bank.update(w, roll);
    const double rolling = rms(bank.render(4800, 1.0f));
    CHECK(rolling > 0.0005);

    rg::TyreSoundInput slide = roll;
    slide.slip_angle = 0.3;
    slide.fy = 8000.0;
    for (int w = 0; w < 4; ++w) bank.update(w, slide);
    bank.render(4800, 1.0f); // let the squeal level rise
    const double sliding = rms(bank.render(4800, 1.0f));
    CHECK(sliding > 3.0 * rolling);
    CHECK(rms(bank.render(4800, 0.0f)) == 0.0);
}

TEST_CASE("tyre sound: same seed, same samples", "[tyre_sound][determinism]") {
    rg::TyreSoundBank a(2, 7, squeal()), b(2, 7, squeal());
    rg::TyreSoundInput in;
    in.speed = 20.0;
    in.load_n = 3000.0;
    in.slip_angle = 0.2;
    in.fy = 5000.0;
    for (int w = 0; w < 2; ++w) {
        a.update(w, in);
        b.update(w, in);
    }
    CHECK(a.render(1000, 1.0f) == b.render(1000, 1.0f));
}

// ---- owner 2026-10-18: recorded rolling layer ------------------------------------------------------------------------
namespace {
// A 3 s "recording": a 300 Hz tone with a slow amplitude drift, stereo, so the seam would click without the crossfade.
std::vector<std::int16_t> fake_recording() {
    std::vector<std::int16_t> pcm(48000 * 3 * 2);
    for (std::size_t i = 0; i < pcm.size() / 2; ++i) {
        const double t = double(i) / 48000.0;
        const double v = 9000.0 * std::sin(6.2831853 * 300.0 * t) * (0.6 + 0.4 * std::sin(6.2831853 * 0.31 * t));
        pcm[2 * i] = static_cast<std::int16_t>(v);
        pcm[2 * i + 1] = static_cast<std::int16_t>(v);
    }
    return pcm;
}
double roll_rms(rg::TyreSoundBank& bank, double speed, bool sealed = true) {
    rg::TyreSoundInput in;
    in.speed = speed; in.tread_speed = speed; in.load_n = 4000.0; in.sealed = sealed;
    for (int w = 0; w < bank.wheels(); ++w) bank.update(w, in);
    bank.render(24000, 1.0f);
    return rms(bank.render(96000, 1.0f));
}
}

TEST_CASE("tyre roll recording: loop seam is continuous", "[tyre_sound]") {
    const auto pcm = fake_recording();
    const auto loop = rg::make_roll_loop(pcm.data(), pcm.size(), 2, 75.0);
    REQUIRE(loop);
    const auto& l = *loop;
    CHECK(l.size() == pcm.size() / 2 - 3600);
    // The step across the seam (last -> first) must be no larger than the largest step inside the loop.
    double max_inner = 0.0;
    for (std::size_t i = 1; i < l.size(); ++i) max_inner = std::max(max_inner, double(std::abs(l[i] - l[i - 1])));
    CHECK(std::abs(l[0] - l.back()) <= 1.5 * max_inner);
    CHECK_FALSE(rg::make_roll_loop(pcm.data(), 100, 2, 75.0)); // too short
    CHECK_FALSE(rg::make_roll_loop(nullptr, 0, 2, 75.0));
}

TEST_CASE("tyre roll recording: silent at standstill, level grows with speed, wheels differ", "[tyre_sound]") {
    const auto pcm = fake_recording();
    const auto loop = rg::make_roll_loop(pcm.data(), pcm.size(), 2, 75.0);
    rg::TyreSoundBank bank(4, 1234, nullptr, loop, {});
    CHECK(roll_rms(bank, 0.0) == 0.0);
    double prev = 0.0;
    for (double kmh : {10.0, 20.0, 40.0, 70.0, 100.0}) {
        const double r = roll_rms(bank, kmh / 3.6);
        INFO("km/h " << kmh);
        CHECK(r > prev);
        prev = r;
    }
    // One car-level layer: with all wheels sealed, every wheel channel carries the same recorded signal.
    rg::TyreSoundInput in; in.speed = 15.0; in.tread_speed = 15.0; in.load_n = 4000.0;
    for (int w = 0; w < 4; ++w) bank.update(w, in);
    bank.render(24000, 1.0f);
    const auto out = bank.render(4800, 1.0f);
    int same = 0;
    for (std::size_t f = 0; f < 4800; ++f) if (out[f * 4] == out[f * 4 + 1]) ++same;
    CHECK(same == 4800);
}

TEST_CASE("tyre roll recording: fallback to the procedural hiss without the file or off sealed surfaces", "[tyre_sound]") {
    const auto pcm = fake_recording();
    const auto loop = rg::make_roll_loop(pcm.data(), pcm.size(), 2, 75.0);
    rg::TyreRollConfig off; off.use_recording = false; // roll_source: "procedural"
    const double v = 25.0;
    auto level = [&](std::shared_ptr<const std::vector<float>> l, rg::TyreRollConfig c, bool sealed) {
        rg::TyreSoundBank b(4, 1234, nullptr, std::move(l), c); // fresh bank: identical noise sequence
        return roll_rms(b, v, sealed);
    };
    const double procedural = level(nullptr, {}, true);
    CHECK(procedural > 0.0005);
    CHECK(level(loop, off, true) == procedural);      // config switch
    CHECK(level(loop, {}, false) == procedural);      // dirt/grass keep the hiss
    CHECK(level(loop, {}, true) != procedural);
}

// Level report with the shipped recording (data/controls/tyre_sound.json defaults) next to the motor total.
#include <cstdio>
#include <fstream>
TEST_CASE("tyre roll level report: bank RMS at 20/40/100 km/h with the shipped recording", "[.][diag][tyre_sound]") {
    std::ifstream f(std::string(RG_SOURCE_DIR) + "/game/audio/tyres/asphalt_roll_provisional.wav", std::ios::binary);
    REQUIRE(f);
    std::vector<char> raw((std::istreambuf_iterator<char>(f)), {});
    std::size_t data_at = 12; // RIFF header, then walk the chunks to "data"
    while (data_at + 8 <= raw.size() && std::memcmp(raw.data() + data_at, "data", 4) != 0) {
        std::uint32_t len; std::memcpy(&len, raw.data() + data_at + 4, 4);
        data_at += 8 + len + (len & 1u);
    }
    REQUIRE(data_at + 8 <= raw.size());
    data_at += 8;
    std::vector<std::int16_t> pcm((raw.size() - data_at) / 2);
    std::memcpy(pcm.data(), raw.data() + data_at, pcm.size() * 2);
    const auto loop = rg::make_roll_loop(pcm.data(), pcm.size(), 2, 75.0);
    REQUIRE(loop);
    rg::TyreSoundBank bank(4, 1234, nullptr, loop, {});
    rg::TyreSoundBank old_bank(4, 1234, nullptr);
    for (double kmh : {20.0, 40.0, 100.0})
        std::printf("%5.0f km/h tyres(4 wheels) recorded %.5f | procedural %.5f\n", kmh, roll_rms(bank, kmh / 3.6), roll_rms(old_bank, kmh / 3.6));
}
