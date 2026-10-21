#include <catch2/catch_test_macros.hpp>

#include "rg/regen_info.h"

TEST_CASE("regen_hud: only cars with electric motors show it", "[regen_info][hybrid_selector]") {
    ps::drivetrain::PowertrainSnapshot p;
    p.regen_power_w = 50000; p.regen_brake_share = 0.6; p.pad_brake_share = 0.4;
    CHECK_FALSE(rg::regen_hud(p).fitted);
    CHECK(rg::regen_label(rg::regen_hud(p)).empty());
    p.electric_fitted = true; p.electric_motor_count = 1;
    const auto h = rg::regen_hud(p);
    CHECK(h.fitted); CHECK(h.active);
    CHECK(h.power_kw == 50.0); CHECK(h.regen_share == 0.6); CHECK(h.pad_share == 0.4);
    CHECK(rg::regen_label(h) == "REGEN 50 kW / 60%");
}

TEST_CASE("regen_hud: idle and clamping", "[regen_info]") {
    ps::drivetrain::PowertrainSnapshot p;
    p.electric_fitted = true; p.electric_motor_count = 2;
    p.regen_power_w = -5; p.regen_brake_share = 1.5;
    const auto h = rg::regen_hud(p);
    CHECK_FALSE(h.active); CHECK(h.power_kw == 0.0); CHECK(h.regen_share == 1.0);
    CHECK(rg::regen_label(h) == "REGEN 0 kW");
}
