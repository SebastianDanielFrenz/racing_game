// test_control_capture.cpp - rg::BindingCapture / rg::AxisCalibrator (PLAN.md R5b): the
// press-the-input flow of the controls screen. Each test names its sabotage ("sabotage:").
#include "rg/control_capture.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

namespace {

using Catch::Approx;
using rg::BindingCapture;
using rg::BindingType;
using rg::CaptureState;
using rg::CaptureTarget;
using rg::DeviceClass;
using rg::InputSnapshot;

InputSnapshot keys(std::vector<std::string> k) {
    InputSnapshot s;
    s.keys = std::move(k);
    return s;
}

InputSnapshot with_pad(std::vector<int> buttons, std::vector<float> axes) {
    InputSnapshot s;
    rg::PadSnapshot p;
    p.slot = 0;
    p.buttons = std::move(buttons);
    p.axes = std::move(axes);
    s.pads.push_back(std::move(p));
    return s;
}

CaptureTarget button_target(DeviceClass d) {
    CaptureTarget t;
    t.device = d;
    t.button_action = true;
    return t;
}

CaptureTarget axis_target(DeviceClass d, rg::AxisRange r, int sign = 0) {
    CaptureTarget t;
    t.device = d;
    t.button_action = false;
    t.range = r;
    t.sign = sign;
    return t;
}

} // namespace

TEST_CASE("capture: the first newly pressed key binds, keys held at the start do not", "[controls][capture]") {
    BindingCapture c;
    c.begin(button_target(DeviceClass::Keyboard), keys({"Enter"})); // Enter opened the capture and is still down
    CHECK(c.feed(keys({"Enter"}), 0.016) == CaptureState::Waiting);
    CHECK(c.feed(keys({}), 0.016) == CaptureState::Waiting);
    CHECK(c.feed(keys({"X"}), 0.016) == CaptureState::Captured);
    CHECK(c.result().type == BindingType::Key);
    CHECK(c.result().key == "X");
    // Once finished it stays finished.
    CHECK(c.feed(keys({"Y"}), 0.016) == CaptureState::Captured);
    CHECK(c.result().key == "X");
    // sabotage: not seeding the previous key set in begin() binds Enter at once.
}

TEST_CASE("capture: Esc cancels on every device", "[controls][capture]") {
    for (const DeviceClass d : {DeviceClass::Keyboard, DeviceClass::Mouse, DeviceClass::Gamepad}) {
        BindingCapture c;
        c.begin(button_target(d), with_pad({}, {0, 0, 0, 0, 0, 0}));
        InputSnapshot esc = with_pad({}, {0, 0, 0, 0, 0, 0});
        esc.keys = {"Escape"};
        CHECK(c.feed(esc, 0.016) == CaptureState::Cancelled);
    }
}

TEST_CASE("capture: a signed-axis row gives the key the row's side", "[controls][capture]") {
    BindingCapture c;
    c.begin(axis_target(DeviceClass::Keyboard, rg::AxisRange::Signed, -1), keys({}));
    REQUIRE(c.feed(keys({"J"}), 0.016) == CaptureState::Captured);
    CHECK(c.result().sign == Approx(-1.0F));
}

TEST_CASE("capture: pad buttons bind on press, B cancels short and binds on a hold", "[controls][capture]") {
    {
        BindingCapture c;
        c.begin(button_target(DeviceClass::Gamepad), with_pad({}, {0, 0}));
        CHECK(c.feed(with_pad({3}, {0, 0}), 0.016) == CaptureState::Captured);
        CHECK(c.result().type == BindingType::JoyButton);
        CHECK(c.result().index == 3);
    }
    {
        BindingCapture c; // a short B press cancels
        c.begin(button_target(DeviceClass::Gamepad), with_pad({}, {0, 0}));
        CHECK(c.feed(with_pad({1}, {0, 0}), 0.1) == CaptureState::Waiting);
        CHECK(c.b_held() == Approx(0.1));
        CHECK(c.feed(with_pad({}, {0, 0}), 0.1) == CaptureState::Cancelled);
    }
    {
        BindingCapture c; // a held B binds
        c.begin(button_target(DeviceClass::Gamepad), with_pad({}, {0, 0}));
        CaptureState s = CaptureState::Waiting;
        for (int i = 0; i < 60 && s == CaptureState::Waiting; ++i) s = c.feed(with_pad({1}, {0, 0}), 0.016);
        CHECK(s == CaptureState::Captured);
        CHECK(c.result().index == 1);
    }
    {
        BindingCapture c; // B still down from the click that opened the capture: released first, no cancel
        c.begin(button_target(DeviceClass::Gamepad), with_pad({1}, {0, 0}));
        CHECK(c.feed(with_pad({1}, {0, 0}), 0.016) == CaptureState::Waiting);
        CHECK(c.feed(with_pad({}, {0, 0}), 0.016) == CaptureState::Waiting);
    }
    // sabotage: binding B on the first press (no hold) makes the cancel case Captured.
}

TEST_CASE("capture: an axis needs a real move, direction picks the half, noise is ignored", "[controls][capture]") {
    BindingCapture c;
    c.begin(axis_target(DeviceClass::Gamepad, rg::AxisRange::Unit), with_pad({}, {0.0F, 0.02F, 0.0F, 0.0F, 0.0F, 0.0F}));
    CHECK(c.feed(with_pad({}, {0.1F, 0.05F, 0.0F, 0.0F, 0.0F, 0.0F}), 0.016) == CaptureState::Waiting); // drift
    CHECK(c.feed(with_pad({}, {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.5F}), 0.016) == CaptureState::Waiting);  // below the threshold
    REQUIRE(c.feed(with_pad({}, {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.8F}), 0.016) == CaptureState::Captured);
    CHECK(c.result().type == BindingType::JoyAxis);
    CHECK(c.result().index == 5);
    CHECK(c.result().span == rg::AxisSpan::Positive);
    // Pushed the other way: the negative half.
    BindingCapture n;
    n.begin(axis_target(DeviceClass::Gamepad, rg::AxisRange::Unit), with_pad({}, {0.0F, 0.0F}));
    REQUIRE(n.feed(with_pad({}, {0.0F, -0.9F}), 0.016) == CaptureState::Captured);
    CHECK(n.result().index == 1);
    CHECK(n.result().span == rg::AxisSpan::Negative);
    // The largest deviation wins when two axes move.
    BindingCapture m;
    m.begin(axis_target(DeviceClass::Gamepad, rg::AxisRange::Unit), with_pad({}, {0.0F, 0.0F}));
    REQUIRE(m.feed(with_pad({}, {0.7F, 0.95F}), 0.016) == CaptureState::Captured);
    CHECK(m.result().index == 1);
    // sabotage: a threshold of 0.05 binds the drifting axis 0 first.
}

TEST_CASE("capture: a pedal resting at +1 is bound calibrated over its travel", "[controls][capture]") {
    BindingCapture c;
    c.begin(axis_target(DeviceClass::Gamepad, rg::AxisRange::Unit), with_pad({}, {0.0F, 1.0F}));
    REQUIRE(c.feed(with_pad({}, {0.0F, 0.1F}), 0.016) == CaptureState::Captured);
    CHECK(c.result().index == 1);
    CHECK(c.result().tuning.calibrated);
    CHECK(c.result().tuning.cal_min == Approx(1.0F));
    CHECK(c.result().tuning.cal_max == Approx(-1.0F));
    CHECK(c.result().span == rg::AxisSpan::Full);
    // And it reads 0 at rest, 1 at the end of the travel.
    CHECK(rg::evaluate_axis_binding(c.result(), 1.0F, rg::AxisRange::Unit) == Approx(0.0F));
    CHECK(rg::evaluate_axis_binding(c.result(), -1.0F, rg::AxisRange::Unit) == Approx(1.0F));
}

TEST_CASE("capture: the full-axis row takes axes only, a half-row takes the side's sign", "[controls][capture]") {
    BindingCapture c;
    c.begin(axis_target(DeviceClass::Gamepad, rg::AxisRange::Signed, 0), with_pad({}, {0.0F, 0.0F}));
    CHECK(c.feed(with_pad({4}, {0.0F, 0.0F}), 0.016) == CaptureState::Waiting); // a button does not bind an axis row
    REQUIRE(c.feed(with_pad({}, {-0.9F, 0.0F}), 0.016) == CaptureState::Captured);
    CHECK(c.result().span == rg::AxisSpan::Full);
    BindingCapture r;
    r.begin(axis_target(DeviceClass::Gamepad, rg::AxisRange::Signed, 1), with_pad({}, {0.0F, 0.0F}));
    REQUIRE(r.feed(with_pad({}, {0.9F, 0.0F}), 0.016) == CaptureState::Captured);
    CHECK(r.result().span == rg::AxisSpan::Positive);
    CHECK(r.result().sign == Approx(1.0F));
    BindingCapture b; // a button on a half row is a digital binding of that side
    b.begin(axis_target(DeviceClass::Gamepad, rg::AxisRange::Signed, -1), with_pad({}, {0.0F, 0.0F}));
    REQUIRE(b.feed(with_pad({9}, {0.0F, 0.0F}), 0.016) == CaptureState::Captured);
    CHECK(b.result().type == BindingType::JoyButton);
    CHECK(b.result().sign == Approx(-1.0F));
}

TEST_CASE("capture: the mouse binds buttons and the wheel, motion only for mouse look", "[controls][capture]") {
    {
        BindingCapture c;
        InputSnapshot start;
        start.mouse_buttons = {1}; // the click that opened the capture
        c.begin(button_target(DeviceClass::Mouse), start);
        CHECK(c.feed(start, 0.016) == CaptureState::Waiting);
        InputSnapshot right;
        right.mouse_buttons = {2};
        REQUIRE(c.feed(right, 0.016) == CaptureState::Captured);
        CHECK(c.result().type == BindingType::MouseButton);
        CHECK(c.result().index == 2);
    }
    {
        BindingCapture c;
        c.begin(button_target(DeviceClass::Mouse), InputSnapshot{});
        InputSnapshot w;
        w.wheel_down = 1.0F;
        REQUIRE(c.feed(w, 0.016) == CaptureState::Captured);
        CHECK(c.result().index == 5);
    }
    {
        BindingCapture c;
        c.begin(axis_target(DeviceClass::Mouse, rg::AxisRange::Delta), InputSnapshot{});
        InputSnapshot mv;
        mv.mouse_dx = 5.0F;
        mv.mouse_dy = 40.0F;
        CHECK(c.feed(mv, 0.016) == CaptureState::Waiting);
        mv.mouse_dy = 40.0F;
        REQUIRE(c.feed(mv, 0.016) == CaptureState::Captured);
        CHECK(c.result().type == BindingType::MouseMotion);
        CHECK(c.result().index == 1);
        BindingCapture d; // a button does not bind a mouse-look axis
        d.begin(axis_target(DeviceClass::Mouse, rg::AxisRange::Delta), InputSnapshot{});
        InputSnapshot click;
        click.mouse_buttons = {2};
        CHECK(d.feed(click, 0.016) == CaptureState::Waiting);
    }
}

TEST_CASE("calibrator: learns a pedal's travel from its rest end", "[controls][capture]") {
    rg::AxisCalibrator cal;
    cal.begin(1.0F);
    CHECK_FALSE(cal.valid());
    cal.feed(0.95F);
    CHECK_FALSE(cal.valid()); // moved too little to mean anything
    for (const float v : {0.5F, 0.0F, -0.5F, -0.8F, -0.6F}) cal.feed(v);
    CHECK(cal.valid());
    rg::AxisTuning t;
    REQUIRE(cal.apply(t, true));
    CHECK(t.calibrated);
    CHECK(t.cal_min == Approx(1.0F));
    CHECK(t.cal_max == Approx(-0.8F));
    // A centred wheel: lowest / highest seen.
    rg::AxisCalibrator wheel;
    wheel.begin(0.0F);
    for (const float v : {-0.9F, 0.0F, 0.85F}) wheel.feed(v);
    rg::AxisTuning w;
    REQUIRE(wheel.apply(w, false));
    CHECK(w.cal_min == Approx(-0.9F));
    CHECK(w.cal_max == Approx(0.85F));
    // Too little travel: refused, tuning untouched.
    rg::AxisCalibrator idle;
    idle.begin(0.0F);
    idle.feed(0.05F);
    rg::AxisTuning untouched;
    CHECK_FALSE(idle.apply(untouched, false));
    CHECK_FALSE(untouched.calibrated);
    // sabotage: taking cal_min = lowest (not the rest end) for a pedal flips it so it reads 1.0 at rest.
}
