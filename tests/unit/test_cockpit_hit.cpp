// rg::CockpitHit: a synthetic ray per hyper cockpit control must produce the same DriverActions writes as the
// equivalent keyboard/Set input; plus a miss, duplicate-edge, availability, moving-wheel and disabled cases.
#include "rg/cockpit_hit.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace {

struct Sink final : rg::ChannelSink {
    std::map<std::string, double, std::less<>> values;
    double get(std::string_view id) const override {
        const auto it = values.find(id);
        return it == values.end() ? 0 : it->second;
    }
    void set(std::string_view id, double value) override { values[std::string(id)] = value; }
};

rg::CabinConfig cabin_config() {
    std::ifstream input(std::string(RG_SOURCE_DIR) + "/data/controls/cabin.json");
    const auto j = nlohmann::json::parse(input);
    return {j.at("crank_timeout_s"), j.at("stop_max_speed_mps"), j.at("blink_hz"), j.at("indicator_arm_deg"), j.at("indicator_cancel_deg"), j.at("brake_lamp_threshold")};
}

rg::DriverSnapshot hyper_snapshot() {
    rg::DriverSnapshot s;
    s.fitted_bits = rg::kAssistBitTractionControl | rg::kAssistBitAeroControl | rg::kAssistBitLaunchControl;
    s.traction_modes = 31;
    s.drive_modes = 0xF;
    s.launch = rg::LaunchLamp::Off;
    return s;
}

constexpr double kFrame = 1.0 / 60.0;

struct Rig {
    rg::CockpitHit hit;
    Rig() {
        std::string error;
        const std::string root = RG_SOURCE_DIR;
        REQUIRE(hit.load(root + "/data/cockpits/car_hyper_common.json", root + "/external/physics_sim/data/models/car_hyper/car_hyper.rig.json", &error));
    }
    // A ray from 0.3 m in front of the part (along its normal) onto the centre of its hit box.
    std::pair<ps::Vec3, ps::Vec3> ray_at(std::size_t index) const {
        ps::Vec3 origin;
        ps::Mat3 rot;
        hit.part_pose(index, origin, rot);
        const ps::Vec3 normal = rot * ps::Vec3{0, 0, 1};
        return {hit.hit_centre_model(index) + normal * 0.3, normal * -1.0};
    }
};

// One side of the comparison: a DriverActions + sink fed either by mouse events or by direct keyboard-style submissions.
struct Side {
    rg::DriverActions actions;
    Sink sink;
    rg::DriverSnapshot snapshot = hyper_snapshot();
    int shift = 0; // net shift edges (+1 up / -1 down)
    Side() { actions.configure_cabin(cabin_config()); }
    void frame(std::vector<rg::ActionEvent> held = {}) {
        for (auto& e : held) actions.submit(e);
        const rg::VehicleContext ctx{snapshot, true};
        actions.apply(ctx, sink);
        actions.advance(ctx, sink, kFrame);
    }
    void submit(rg::ActionEvent e) { actions.submit(std::move(e)); }
};

void feed(Side& side, rg::CockpitHit& hit, const std::vector<rg::CockpitEvent>& events) {
    for (const auto& e : events) {
        if (e.shift != 0) side.shift += e.shift;
        else REQUIRE(e.action.source == rg::ActionSource::CockpitMouse);
        if (e.shift == 0) side.submit(e.action);
    }
    (void)hit;
}

// Press the control with its primary (or secondary) button for `down_frames` frames, then let it return.
std::vector<rg::CockpitEvent> click(Rig& rig, Side& side, std::size_t index, rg::CockpitButton button = rg::CockpitButton::Primary, int down_frames = 2) {
    const auto [o, d] = rig.ray_at(index);
    std::vector<rg::CockpitEvent> all;
    rig.hit.update(0, side.snapshot, side.sink); // availability is the previous frame's
    REQUIRE(rig.hit.pointer_down(o, d, button) == static_cast<int>(index));
    for (int f = 0; f < 20; ++f) {
        if (f == down_frames) rig.hit.pointer_up(button);
        rig.hit.update(kFrame, side.snapshot, side.sink);
        auto events = rig.hit.take_events();
        feed(side, rig.hit, events);
        for (auto& e : events) all.push_back(std::move(e));
        side.frame(rig.hit.held_actions());
    }
    return all;
}

struct Expect {
    const char* control;
    rg::CockpitButton button;
    const char* action; // "" for a shift edge
    rg::DriverActionKind kind;
    double value;
    int shift;
};

const Expect kExpected[] = {
    {"tc", rg::CockpitButton::Primary, "traction_control", rg::DriverActionKind::Press, 1, 0},
    {"tc", rg::CockpitButton::Secondary, "traction_control", rg::DriverActionKind::Set, 0, 0},
    {"lc", rg::CockpitButton::Primary, "toggle_launch_control", rg::DriverActionKind::Press, 1, 0},
    {"aero", rg::CockpitButton::Primary, "toggle_aero", rg::DriverActionKind::Press, 1, 0},
    {"am", rg::CockpitButton::Primary, "auto_shift", rg::DriverActionKind::Press, 1, 0},
    {"eco", rg::CockpitButton::Primary, "drive_mode", rg::DriverActionKind::Set, 0, 0},
    {"normal", rg::CockpitButton::Primary, "drive_mode", rg::DriverActionKind::Set, 1, 0},
    {"sport", rg::CockpitButton::Primary, "drive_mode", rg::DriverActionKind::Set, 2, 0},
    {"racing", rg::CockpitButton::Primary, "drive_mode", rg::DriverActionKind::Set, 3, 0},
    {"start", rg::CockpitButton::Primary, "start_stop", rg::DriverActionKind::Press, 1, 0},
    {"hazard", rg::CockpitButton::Primary, "hazard", rg::DriverActionKind::Press, 1, 0},
    {"lights", rg::CockpitButton::Primary, "lights", rg::DriverActionKind::Press, 1, 0},
    {"lights", rg::CockpitButton::Secondary, "lights", rg::DriverActionKind::Set, 2, 0},
    {"stalk_L", rg::CockpitButton::Primary, "indicator_left", rg::DriverActionKind::Press, 1, 0},
    {"stalk_L", rg::CockpitButton::Secondary, "indicator_right", rg::DriverActionKind::Press, 1, 0},
    {"stalk_R", rg::CockpitButton::Primary, "indicator_right", rg::DriverActionKind::Press, 1, 0},
    {"stalk_R", rg::CockpitButton::Secondary, "indicator_left", rg::DriverActionKind::Press, 1, 0},
    {"paddle_L", rg::CockpitButton::Primary, "", rg::DriverActionKind::Press, 0, -1},
    {"paddle_R", rg::CockpitButton::Primary, "", rg::DriverActionKind::Press, 0, 1},
};

std::map<std::string, double, std::less<>> state_of(const Side& s) {
    auto v = s.sink.values;
    const auto& c = s.actions.cabin();
    v["cab.hazard"] = c.hazard;
    v["cab.indicator"] = c.indicator;
    v["cab.lights"] = c.lights;
    v["cab.light_mode"] = static_cast<double>(c.light_mode);
    v["cab.auto_shift"] = c.auto_shift;
    v["cab.cranking"] = c.auto_cranking;
    v["cab.shift"] = s.shift;
    return v;
}

} // namespace

TEST_CASE("cockpit hit: layout loads the 17 hyper controls from the generated rig", "[cockpit][hit]") {
    Rig rig;
    CHECK(rig.hit.control_count() == 17);
    CHECK(rig.hit.find("spare") < 0); // the blank cap carries no control
    CHECK(rig.hit.find("horn") >= 0);
    CHECK(rig.hit.control(static_cast<std::size_t>(rig.hit.find("horn"))).wheel_mounted);
    // Thresholds come from the parts rig: the paddle fires at click_at_rad 0.09 of 0.14 rad, the buttons at the default fraction.
    const auto& paddle = rig.hit.control(static_cast<std::size_t>(rig.hit.find("paddle_L")));
    CHECK(paddle.press_at == Catch::Approx(0.09 / 0.14));
    const auto& tc = rig.hit.control(static_cast<std::size_t>(rig.hit.find("tc")));
    CHECK(tc.press_at == Catch::Approx(0.6));
    CHECK(rig.hit.debounce_s() == Catch::Approx(0.15));
}

TEST_CASE("cockpit hit: a click on each control equals the matching keyboard/Set input", "[cockpit][hit]") {
    for (const Expect& ex : kExpected) {
        INFO(ex.control << " button " << static_cast<int>(ex.button));
        Rig rig;
        const int index = rig.hit.find(ex.control);
        REQUIRE(index >= 0);
        Side mouse, keyboard;
        const auto events = click(rig, mouse, static_cast<std::size_t>(index), ex.button);
        REQUIRE(events.size() == 1); // exactly one event per click
        CHECK(events[0].control_id == ex.control);
        CHECK(events[0].shift == ex.shift);
        CHECK(events[0].action.id == ex.action);
        if (ex.shift == 0) {
            CHECK(events[0].action.kind == ex.kind);
            CHECK(events[0].action.value == ex.value);
        }
        if (ex.shift != 0) keyboard.shift += ex.shift;
        else keyboard.submit({ex.action, ex.kind, ex.value, rg::ActionSource::Keyboard});
        for (int f = 0; f < 20; ++f) keyboard.frame();
        CHECK(state_of(mouse) == state_of(keyboard));
    }
}

TEST_CASE("cockpit hit: the horn is a held level from press crossing to release crossing", "[cockpit][hit]") {
    Rig rig;
    Side mouse;
    const std::size_t horn = static_cast<std::size_t>(rig.hit.find("horn"));
    const auto [o, d] = rig.ray_at(horn);
    rig.hit.update(0, mouse.snapshot, mouse.sink);
    REQUIRE(rig.hit.pointer_down(o, d) == static_cast<int>(horn));
    int held_frames = 0;
    for (int f = 0; f < 30; ++f) {
        rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
        feed(mouse, rig.hit, rig.hit.take_events());
        mouse.frame(rig.hit.held_actions());
        if (mouse.actions.cabin().horn) ++held_frames;
    }
    CHECK(held_frames >= 25); // asserted every frame while the button stays down
    rig.hit.pointer_up();
    for (int f = 0; f < 20; ++f) {
        rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
        feed(mouse, rig.hit, rig.hit.take_events());
        mouse.frame(rig.hit.held_actions());
    }
    CHECK_FALSE(mouse.actions.cabin().horn);
    CHECK(rig.hit.held_actions().empty());
}

TEST_CASE("cockpit hit: a deliberate miss hits nothing and submits nothing", "[cockpit][hit]") {
    Rig rig;
    Side mouse;
    // Straight up from above the tunnel, but 1 m to the side of every control.
    const ps::Vec3 origin{0.2, 2.0, 0.9}, direction{0, 0, -1};
    CHECK(rig.hit.pick(origin, direction) < 0);
    CHECK(rig.hit.pointer_down(origin, direction) < 0);
    for (int f = 0; f < 10; ++f) rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
    CHECK(rig.hit.take_events().empty());
    // A ray that just clears a button box (beyond the pick margin) also misses.
    const std::size_t tc = static_cast<std::size_t>(rig.hit.find("tc"));
    const auto [o, d] = rig.ray_at(tc);
    const ps::Vec3 shifted = o + ps::Vec3{0, rig.hit.mouse_config().pick_margin_m + 0.05, 0};
    CHECK(rig.hit.pick(shifted, d) != static_cast<int>(tc));
}

TEST_CASE("cockpit hit: duplicate edges give one event", "[cockpit][hit]") {
    Rig rig;
    Side mouse;
    const std::size_t sport = static_cast<std::size_t>(rig.hit.find("sport"));
    const auto [o, d] = rig.ray_at(sport);
    // Two downs without an up, then a long hold: one event.
    rig.hit.update(0, mouse.snapshot, mouse.sink);
    REQUIRE(rig.hit.pointer_down(o, d) == static_cast<int>(sport));
    CHECK(rig.hit.pointer_down(o, d) == static_cast<int>(sport));
    std::size_t events = 0;
    for (int f = 0; f < 60; ++f) {
        rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
        events += rig.hit.take_events().size();
    }
    CHECK(events == 1);
    // A separate fast click-release-click inside the debounce window gives one event; one after it fires again.
    rig.hit.pointer_up();
    for (int f = 0; f < 20; ++f) rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
    rig.hit.take_events();
    events = 0;
    rig.hit.pointer_down(o, d);
    rig.hit.pointer_up();
    int guard = 0;
    while ((events == 0 || rig.hit.pressed(sport)) && guard++ < 20) {
        rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
        events += rig.hit.take_events().size();
    }
    REQUIRE(events == 1);
    rig.hit.pointer_down(o, d); // re-armed already, but only ~0.08 s after the first fire
    rig.hit.pointer_up();
    for (int f = 0; f < 8; ++f) {
        rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
        events += rig.hit.take_events().size();
    }
    CHECK(events == 1);
    CHECK(rig.hit.debounced_count() >= 1);
    for (int f = 0; f < 30; ++f) rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
    rig.hit.pointer_down(o, d);
    rig.hit.pointer_up();
    for (int f = 0; f < 10; ++f) {
        rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
        events += rig.hit.take_events().size();
    }
    CHECK(events == 2);
}

TEST_CASE("cockpit hit: unavailable controls are hit but inert", "[cockpit][hit]") {
    Rig rig;
    Side mouse;
    mouse.snapshot.drive_modes = 0x3; // only eco and normal offered
    mouse.snapshot.traction_modes = 0;
    rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
    CHECK(rig.hit.available(static_cast<std::size_t>(rig.hit.find("normal"))));
    CHECK_FALSE(rig.hit.available(static_cast<std::size_t>(rig.hit.find("sport"))));
    CHECK_FALSE(rig.hit.available(static_cast<std::size_t>(rig.hit.find("tc"))));
    CHECK_FALSE(rig.hit.available(static_cast<std::size_t>(rig.hit.find("epb")))); // no physics EPB: stays inert
    for (const char* id : {"sport", "racing", "tc", "epb"}) {
        const auto index = static_cast<std::size_t>(rig.hit.find(id));
        const auto [o, d] = rig.ray_at(index);
        CHECK(rig.hit.pointer_down(o, d) == static_cast<int>(index));
        rig.hit.pointer_up();
        for (int f = 0; f < 20; ++f) rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
        CHECK(rig.hit.take_events().empty());
    }
}

TEST_CASE("cockpit hit: wheel-mounted controls follow the steering wheel angle", "[cockpit][hit]") {
    Rig rig;
    const std::size_t horn = static_cast<std::size_t>(rig.hit.find("horn"));
    const std::size_t tc = static_cast<std::size_t>(rig.hit.find("tc"));
    const ps::Vec3 centre0 = rig.hit.hit_centre_model(horn);
    const ps::Vec3 tc0 = rig.hit.hit_centre_model(tc);
    const auto [o, d] = rig.ray_at(horn);
    CHECK(rig.hit.pick(o, d) == static_cast<int>(horn));
    rig.hit.set_wheel_angle(1.0);
    CHECK((rig.hit.hit_centre_model(horn) - centre0).length() < 2e-3); // the horn pad sits (almost) on the column axis...
    // ...but the paddles move on their circle, so the old ray no longer finds the left paddle.
    const std::size_t paddle = static_cast<std::size_t>(rig.hit.find("paddle_L"));
    rig.hit.set_wheel_angle(0);
    const auto [po, pd] = rig.ray_at(paddle);
    CHECK(rig.hit.pick(po, pd) == static_cast<int>(paddle));
    const ps::Vec3 paddle0 = rig.hit.hit_centre_model(paddle);
    rig.hit.set_wheel_angle(0.8);
    CHECK((rig.hit.hit_centre_model(paddle) - paddle0).length() > 0.02);
    CHECK(rig.hit.pick(po, pd) != static_cast<int>(paddle));
    const auto [po2, pd2] = rig.ray_at(paddle);
    CHECK(rig.hit.pick(po2, pd2) == static_cast<int>(paddle));
    CHECK((rig.hit.hit_centre_model(tc) - tc0).length() < 1e-12); // car-fixed controls do not move
}

TEST_CASE("cockpit hit: disabling releases a pressed control and ignores pointer input", "[cockpit][hit]") {
    Rig rig;
    Side mouse;
    const std::size_t horn = static_cast<std::size_t>(rig.hit.find("horn"));
    const auto [o, d] = rig.ray_at(horn);
    rig.hit.update(0, mouse.snapshot, mouse.sink);
    REQUIRE(rig.hit.pointer_down(o, d) == static_cast<int>(horn));
    for (int f = 0; f < 15; ++f) rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
    CHECK_FALSE(rig.hit.held_actions().empty());
    rig.hit.set_enabled(false);
    CHECK(rig.hit.held_actions().empty());
    rig.hit.update(kFrame, mouse.snapshot, mouse.sink);
    auto events = rig.hit.take_events();
    REQUIRE_FALSE(events.empty());
    CHECK(events.back().action.value == 0); // the horn level is let go
    CHECK(rig.hit.pointer_down(o, d) < 0);
    CHECK(rig.hit.hover() < 0);
}
