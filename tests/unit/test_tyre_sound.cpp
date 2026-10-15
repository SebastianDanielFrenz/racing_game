// test_tyre_sound.cpp - rg::TyreSoundBank (port of the GDScript tyre voice): silence without ground contact, level and
// squeal when sliding, determinism, frame count.
#include "rg/tyre_sound.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
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
