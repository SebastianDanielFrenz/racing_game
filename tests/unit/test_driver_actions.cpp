#include "rg/driver_actions.h"
#include <catch2/catch_test_macros.hpp>
#include <array>
#include <limits>
#include <map>
#include <utility>
#include <fstream>
#include <nlohmann/json.hpp>

namespace {
rg::CabinConfig cabin_config() {
    std::ifstream input(std::string(RG_SOURCE_DIR)+"/data/controls/cabin.json");
    const auto j=nlohmann::json::parse(input);
    return {j.at("crank_timeout_s"),j.at("stop_max_speed_mps"),j.at("blink_hz"),j.at("indicator_arm_deg"),j.at("indicator_cancel_deg"),j.at("brake_lamp_threshold")};
}
using Writes = std::vector<std::pair<std::string, double>>;
struct Sink final : rg::ChannelSink {
    std::map<std::string, double, std::less<>> values;
    Writes writes;
    double get(std::string_view id) const override {
        const auto it = values.find(id);
        return it == values.end() ? 0 : it->second;
    }
    void set(std::string_view id, double value) override {
        values[std::string(id)] = value;
        writes.emplace_back(id, value);
    }
};
rg::DriverSnapshot fitted() {
    rg::DriverSnapshot s;
    s.fitted_bits = rg::kAssistBitTractionControl | rg::kAssistBitAeroControl | rg::kAssistBitLaunchControl;
    s.traction_modes = 31;
    s.drive_modes = 7;
    s.nitrous_fitted = true;
    return s;
}
// Recorded frame script follows the old main.gd groups, not the new implementation.
struct Frame { int tc, aero, lc, dm, n2o, automatic; };
const std::array<Frame, 5> recorded{{{1,1,1,1,1,1}, {3,2,2,2,2,0}, {0,0,0,0,0,1}, {7,1,3,4,1,2}, {1,0,0,1,0,0}}};
Writes legacy() {
    Sink sink;
    bool automatic = false;
    for (const auto& f : recorded) {
        for (int i=0; i<f.automatic; ++i) automatic = !automatic;
        sink.set("assist.auto_shift", automatic ? 1 : 0);
        for (int i=0; i<f.tc; ++i) sink.set("assist.traction_control",rg::next_traction_mode(31,static_cast<int>(sink.get("assist.traction_control"))));
        for (int i=0; i<f.aero; ++i) sink.set("assist.aero_control",rg::toggled_aero_control(fitted().fitted_bits,sink.get("assist.aero_control")));
        for (int i=0; i<f.lc; ++i) sink.set("assist.launch_control",rg::toggled_launch_control(fitted().fitted_bits,sink.get("assist.launch_control")));
        for (int i=0; i<f.dm; ++i) sink.set("assist.drive_mode",rg::next_drive_mode(7,static_cast<int>(sink.get("assist.drive_mode"))));
        for (int i=0; i<f.n2o; ++i) sink.set("nitrous_arm",1-sink.get("nitrous_arm"));
    }
    return sink.writes;
}
Writes replay(rg::ActionSource source) {
    rg::DriverActions actions;
    Sink sink;
    for (const auto& f : recorded) {
        // Input polling order differed from the forwarding order before C0.
        const std::array<std::pair<const char*,int>,6> events{{{"auto_shift",f.automatic},{"traction_control",f.tc},{"toggle_aero",f.aero},{"drive_mode",f.dm},{"toggle_launch_control",f.lc},{"toggle_nitrous",f.n2o}}};
        for (const auto& [id,count] : events) for (int i=0;i<count;++i) {
            REQUIRE(actions.submit({id,rg::DriverActionKind::Hold,1,source}));
            REQUIRE(actions.submit({id,rg::DriverActionKind::Hold,0,source}));
        }
        actions.apply({fitted(),true},sink);
        CHECK(actions.pending_count()==0);
    }
    return sink.writes;
}
}

TEST_CASE("driver actions: recorded key script preserves every legacy channel write for all sources", "[driver_actions]") {
    const auto expected=legacy();
    for (int i=0;i<8;++i) CHECK(replay(static_cast<rg::ActionSource>(i))==expected);
}
TEST_CASE("driver actions: read the current physics-written value and chain presses", "[driver_actions]") {
    rg::DriverActions actions; Sink sink;
    sink.values["assist.traction_control"]=3;
    sink.values["assist.launch_control"]=1;
    actions.submit({"traction_control"}); actions.submit({"traction_control"});
    actions.submit({"toggle_launch_control"});
    actions.apply({fitted(),true},sink);
    CHECK(sink.get("assist.traction_control")==0);
    CHECK(sink.get("assist.launch_control")==0);
}
TEST_CASE("driver actions: blocked frames discard presses but preserve the auto-shift preference", "[driver_actions]") {
    rg::DriverActions actions; Sink sink;
    actions.submit({"traction_control",rg::DriverActionKind::Hold,1});
    actions.submit({"auto_shift",rg::DriverActionKind::Hold,1});
    actions.apply({fitted(),false},sink);
    CHECK(sink.writes.empty());
    CHECK(actions.cabin().auto_shift);
    actions.submit({"traction_control",rg::DriverActionKind::Hold,1});
    actions.apply({fitted(),true},sink);
    CHECK(sink.get("assist.traction_control")==0);
    CHECK(sink.get("assist.auto_shift")==1);
}
TEST_CASE("driver actions: no modes are invented and absent assists preserve legacy inert writes", "[driver_actions]") {
    rg::DriverActions actions; Sink sink;
    actions.submit({"traction_control"}); actions.submit({"drive_mode"}); actions.submit({"toggle_aero"});
    actions.apply({{},true},sink);
    CHECK(sink.writes==Writes{{"assist.auto_shift",0},{"assist.aero_control",0}});
    auto s=fitted(); s.drive_modes=0b1011;
    sink.values["assist.drive_mode"]=1;
    actions.submit({"drive_mode"}); actions.apply({s,true},sink);
    CHECK(sink.get("assist.drive_mode")==3);
    actions.submit({"drive_mode",rg::DriverActionKind::Set,2}); actions.apply({s,true},sink);
    CHECK(sink.get("assist.drive_mode")==3);
}
TEST_CASE("driver actions: one control-state projection reports effective state, fitting and lamps", "[driver_actions]") {
    Sink sink; auto s=fitted();
    s.traction_effective=3; s.traction_intervening=true;
    auto tc=rg::DriverActions::state("traction_control",s,sink);
    CHECK(tc.fitted); CHECK(tc.enabled); CHECK(tc.value==3); CHECK(tc.label=="Sport"); CHECK(tc.lamp_segments==8); CHECK(tc.blink);
    s.launch=rg::LaunchLamp::Armed;
    auto lc=rg::DriverActions::state("toggle_launch_control",s,sink);
    CHECK(lc.label=="LC ARMED"); CHECK(lc.blink); CHECK(lc.colour_role=="warning");
    s.drive_mode_effective=2;
    auto dm=rg::DriverActions::state("drive_mode",s,sink);
    CHECK(dm.value==2); CHECK(dm.label=="SPORT"); CHECK(dm.colour_role=="sport"); CHECK(dm.lamp_segments==4);
    s.aero_active=true; s.aero_target_deg=17;
    auto aero=rg::DriverActions::state("toggle_aero",s,sink);
    CHECK(aero.enabled); CHECK(aero.value==1); CHECK(aero.aero_target_deg==17);
    CHECK_FALSE(rg::DriverActions::state("traction_control",{},sink).fitted);
    CHECK_FALSE(rg::DriverActions::state("toggle_launch_control",{},sink).fitted);
    CHECK_FALSE(rg::DriverActions::state("drive_mode",{},sink).fitted);
    CHECK_FALSE(rg::DriverActions::state("toggle_aero",{},sink).fitted);
}

TEST_CASE("driver actions: park requests and physics EPB state", "[driver_actions]") {
    rg::DriverActions actions;
    Sink sink;
    auto s = fitted();
    CHECK(actions.submit({"park"}));
    actions.apply({s,true},sink);
    CHECK(sink.values.count("park_lock") == 0);
    s.park_fitted = true;
    CHECK(actions.submit({"park"}));
    actions.apply({s,true},sink);
    CHECK(sink.get("park_lock") == 1);
    CHECK_FALSE(rg::DriverActions::state("park",s,sink).active);
    s.park_engaged = true;
    CHECK(rg::DriverActions::state("park",s,sink).label == "P");
    CHECK(actions.submit({"epb"}));
    actions.apply({s,true},sink);
    CHECK(sink.values.count("epb") == 0);
    s.epb_fitted = true;
    CHECK(actions.submit({"epb"}));
    actions.apply({s,true},sink);
    CHECK(sink.get("epb") == 1);
    s.epb_refused = true;
    auto state = rg::DriverActions::state("epb",s,sink);
    CHECK(state.blink);
    CHECK_FALSE(state.active);
    CHECK(state.label == "EPB REFUSED");
    actions.apply({s,true},sink); // refusal never retries automatically
    CHECK(sink.get("epb") == 1);
    CHECK(actions.submit({"epb",rg::DriverActionKind::Set,0}));
    actions.apply({s,true},sink);
    CHECK(sink.get("epb") == 0);
    s.epb_refused = false; s.epb_state = 1;
    CHECK(rg::DriverActions::state("epb",s,sink).active);
}

TEST_CASE("driver cabin: start stops at running or configurable timeout and manual ignition wins", "[driver_actions][cabin]") {
    rg::DriverActions a; a.configure_cabin(cabin_config()); Sink s; auto snap=fitted();
    a.submit({"start_stop"}); a.apply({snap,true},s);
    CHECK(s.get("ignition")==1);
    a.advance({snap,true},s,0.5); CHECK(s.get("starter")==1);
    snap.engine_running=true;
    a.advance({snap,true},s,0.01); CHECK(s.get("starter")==0);
    CHECK_FALSE(a.cabin().auto_cranking);
    snap.speed_mps=5; a.submit({"start_stop"}); a.apply({snap,true},s);
    CHECK(s.get("ignition")==1); // moving stop refused
    snap.speed_mps=0; a.submit({"start_stop"}); a.apply({snap,true},s);
    CHECK(s.get("ignition")==0);
    snap.engine_running=false; a.submit({"start_stop"}); a.apply({snap,true},s);
    a.advance({snap,true},s,1.49); CHECK(s.get("starter")==1);
    a.advance({snap,true},s,0.01); CHECK(s.get("starter")==0);
    a.submit({"start_stop"}); a.apply({snap,true},s); s.set("ignition",0);
    a.advance({snap,true},s,0.1); CHECK_FALSE(a.cabin().auto_cranking);
    auto cfg=cabin_config(); cfg.crank_timeout_s=0.25; a.configure_cabin(cfg);
    a.submit({"start_stop"}); a.apply({snap,true},s);
    a.advance({snap,true},s,0.25); CHECK(s.get("starter")==0);
}
TEST_CASE("driver cabin: indicators self cancel only after turning their direction", "[driver_actions][cabin]") {
    rg::DriverActions a; a.configure_cabin(cabin_config()); Sink s; auto snap=fitted();
    a.submit({"indicator_left"}); a.apply({snap,true},s);
    snap.steering_wheel_deg=-100; a.advance({snap,true},s,0);
    CHECK(a.cabin().indicator==1); CHECK_FALSE(a.cabin().indicator_turned);
    snap.steering_wheel_deg=100; a.advance({snap,true},s,0);
    CHECK(a.cabin().indicator_turned);
    snap.steering_wheel_deg=20; a.advance({snap,true},s,0); CHECK(a.cabin().indicator==1);
    snap.steering_wheel_deg=10; a.advance({snap,true},s,0); CHECK(a.cabin().indicator==0);
    a.submit({"indicator_right"}); a.apply({snap,true},s);
    a.submit({"indicator_right"}); a.apply({snap,true},s); CHECK(a.cabin().indicator==0);
}
TEST_CASE("driver cabin: hazards override both sides and blink at configured frequency", "[driver_actions][cabin]") {
    rg::DriverActions a; a.configure_cabin(cabin_config()); Sink s; auto snap=fitted();
    a.submit({"indicator_left"}); a.submit({"hazard"}); a.apply({snap,true},s);
    a.advance({snap,true},s,0); CHECK(s.get("cabin.indicator_left")==1); CHECK(s.get("cabin.indicator_right")==1);
    a.advance({snap,true},s,1.0/3); a.advance({snap,true},s,0);
    CHECK(s.get("cabin.indicator_left")==0); CHECK(s.get("cabin.indicator_right")==0);
    a.advance({snap,true},s,1.0/3); a.advance({snap,true},s,0);
    CHECK(s.get("cabin.indicator_left")==1); CHECK(s.get("cabin.indicator_right")==1);
    a.submit({"hazard"}); a.apply({snap,true},s); a.advance({snap,true},s,0);
    CHECK(s.get("cabin.indicator_left")==1); CHECK(s.get("cabin.indicator_right")==0);
}
TEST_CASE("driver cabin: horn follows held level and blocked driving silences it", "[driver_actions][cabin]") {
    rg::DriverActions a; a.configure_cabin(cabin_config()); Sink s; auto snap=fitted();
    for (int source=0;source<8;++source) {
        a.submit({"horn",rg::DriverActionKind::Hold,1,static_cast<rg::ActionSource>(source)});
        a.apply({snap,true},s); a.advance({snap,true},s,0);
        CHECK(s.get("cabin.horn")==1);
        a.submit({"horn",rg::DriverActionKind::Hold,0,static_cast<rg::ActionSource>(source)});
        a.apply({snap,true},s); a.advance({snap,true},s,0);
        CHECK(s.get("cabin.horn")==0);
    }
    a.submit({"horn"}); a.apply({snap,true},s); a.advance({snap,false},s,0);
    CHECK(s.get("cabin.horn")==0);
    s.set("brake",0.06); a.advance({snap,true},s,0);
    CHECK(rg::DriverActions::state("brake_lamp",snap,s).active);
    snap.gear=-1; CHECK(rg::DriverActions::state("reverse_lamp",snap,s).active);
}
TEST_CASE("driver cabin: invalid timing settings are rejected", "[driver_actions][cabin]") {
    rg::DriverActions a; auto cfg=cabin_config(); cfg.blink_hz=0;
    CHECK_THROWS(a.configure_cabin(cfg));
    cfg=cabin_config();cfg.indicator_cancel_deg=cfg.indicator_arm_deg;
    CHECK_THROWS(a.configure_cabin(cfg));
}
TEST_CASE("driver cabin: lights toggle dipped beam, Set picks main beam, projection follows", "[driver_actions][cabin][lights]") {
    rg::DriverActions a; a.configure_cabin(cabin_config()); Sink s; auto snap=fitted();
    a.advance({snap,true},s,0); CHECK(s.get("cabin.lights")==0);
    CHECK_FALSE(rg::DriverActions::state("head_lamp",snap,s).active);
    a.submit({"lights",rg::DriverActionKind::Hold,1}); a.apply({snap,true},s); a.advance({snap,true},s,0);
    CHECK(s.get("cabin.lights")==1);
    a.submit({"lights",rg::DriverActionKind::Hold,1}); a.apply({snap,true},s); a.advance({snap,true},s,0);
    CHECK(s.get("cabin.lights")==1); // held: no second edge
    CHECK(rg::DriverActions::state("head_lamp",snap,s).active);
    CHECK(rg::DriverActions::state("tail_lamp",snap,s).active);
    CHECK_FALSE(rg::DriverActions::state("main_beam",snap,s).active);
    a.submit({"lights",rg::DriverActionKind::Set,2}); a.apply({snap,true},s); a.advance({snap,true},s,0);
    CHECK(s.get("cabin.lights")==2);
    CHECK(rg::DriverActions::state("main_beam",snap,s).active);
    a.submit({"lights",rg::DriverActionKind::Hold,0}); a.submit({"lights",rg::DriverActionKind::Hold,1}); a.apply({snap,true},s); a.advance({snap,true},s,0);
    CHECK(s.get("cabin.lights")==0);
}

TEST_CASE("driver actions: ABS and ESC lamps follow the fitted state and interventions", "[driver_actions][abs][esc]") {
    Sink sink;
    rg::DriverSnapshot s;
    auto abs = rg::DriverActions::state("abs", s, sink);
    auto esc = rg::DriverActions::state("esc", s, sink);
    CHECK_FALSE(abs.fitted);
    CHECK_FALSE(esc.fitted);
    CHECK(abs.label.empty());
    CHECK(esc.label.empty());
    s.abs_fitted = s.esc_fitted = true;
    s.abs_on = true;
    s.esc_effective = 1;
    abs = rg::DriverActions::state("abs", s, sink);
    esc = rg::DriverActions::state("esc", s, sink);
    CHECK(abs.label == "ABS");
    CHECK(abs.colour_role == "ok");
    CHECK_FALSE(abs.blink);
    CHECK(esc.label == "ESC ON");
    s.abs_intervening = s.esc_intervening = true;
    abs = rg::DriverActions::state("abs", s, sink);
    esc = rg::DriverActions::state("esc", s, sink);
    CHECK(abs.blink);
    CHECK(abs.label == "ABS ACTIVE");
    CHECK(esc.blink);
    CHECK(esc.label == "ESC ON ACTIVE");
    CHECK(esc.colour_role == "warning");
    s.abs_on = false;
    s.esc_effective = 0;
    abs = rg::DriverActions::state("abs", s, sink);
    esc = rg::DriverActions::state("esc", s, sink);
    CHECK(abs.label == "ABS OFF");
    CHECK_FALSE(abs.blink);
    CHECK(esc.label == "ESC OFF");
    CHECK_FALSE(esc.blink);
}

namespace {
rg::CabinConfig auto_lights_config() {
    auto cfg = cabin_config();
    std::ifstream input(std::string(RG_SOURCE_DIR)+"/data/world/headlights.json");
    const auto j = nlohmann::json::parse(input).at("auto");
    cfg.auto_lights = j.at("enabled");
    cfg.auto_lights_on_lux = j.at("on_lux");
    cfg.auto_lights_off_lux = j.at("off_lux");
    cfg.auto_lights_on_delay_s = j.at("on_delay_s");
    cfg.auto_lights_off_delay_s = j.at("off_delay_s");
    return cfg;
}
void step_lux(rg::DriverActions& a, Sink& s, rg::DriverSnapshot snap, double lux, double dt) {
    snap.ambient_lux = lux;
    a.advance({snap,true},s,dt);
}
}

TEST_CASE("driver cabin: automatic headlamps follow the light sensor with hysteresis and delays", "[driver_actions][cabin][lights][auto_lights]") {
    const auto cfg = auto_lights_config();
    REQUIRE(cfg.auto_lights);
    REQUIRE(cfg.auto_lights_on_lux < cfg.auto_lights_off_lux);
    rg::DriverActions a; a.configure_cabin(cfg); Sink s; const auto snap=fitted();
    // No reading yet: nothing changes.
    a.advance({snap,true},s,0.1); CHECK(s.get("cabin.lights")==0);
    // The first reading decides at once: a night spawn starts with dipped beam.
    step_lux(a,s,snap,0.01,0.1); CHECK(s.get("cabin.lights")==1);
    CHECK(rg::DriverActions::state("head_lamp",snap,s).active);
    CHECK(rg::DriverActions::state("tail_lamp",snap,s).active);
    // Between the thresholds nothing changes (hysteresis).
    step_lux(a,s,snap,0.5*(cfg.auto_lights_on_lux+cfg.auto_lights_off_lux),60); CHECK(s.get("cabin.lights")==1);
    // Bright, but shorter than the off delay: still on; a dark blip resets the timer.
    step_lux(a,s,snap,cfg.auto_lights_off_lux*2,cfg.auto_lights_off_delay_s*0.6); CHECK(s.get("cabin.lights")==1);
    step_lux(a,s,snap,cfg.auto_lights_on_lux*0.5,0.1);
    step_lux(a,s,snap,cfg.auto_lights_off_lux*2,cfg.auto_lights_off_delay_s*0.6); CHECK(s.get("cabin.lights")==1);
    step_lux(a,s,snap,cfg.auto_lights_off_lux*2,cfg.auto_lights_off_delay_s*0.6); CHECK(s.get("cabin.lights")==0);
    // Dark again: on after the on delay.
    step_lux(a,s,snap,cfg.auto_lights_on_lux*0.5,cfg.auto_lights_on_delay_s*0.5); CHECK(s.get("cabin.lights")==0);
    step_lux(a,s,snap,cfg.auto_lights_on_lux*0.5,cfg.auto_lights_on_delay_s*0.6); CHECK(s.get("cabin.lights")==1);
    // A sensor dropout (NaN) holds the state.
    step_lux(a,s,snap,std::numeric_limits<double>::quiet_NaN(),100); CHECK(s.get("cabin.lights")==1);
}

TEST_CASE("driver cabin: the lights key cycles the switch auto -> on -> off -> auto", "[driver_actions][cabin][lights][auto_lights]") {
    const auto cfg = auto_lights_config();
    rg::DriverActions a; a.configure_cabin(cfg); Sink s; const auto snap=fitted();
    const double day = cfg.auto_lights_off_lux*10, night = cfg.auto_lights_on_lux*0.001;
    auto press = [&](double lux) { a.submit({"lights"}); a.apply({snap,true},s); step_lux(a,s,snap,lux,0.1); };
    auto mode = [&] { return rg::DriverActions::state("light_mode",snap,s); };
    // Default: auto. By day the lamps are off and the projection says AUTO.
    CHECK(a.light_mode() == rg::LightMode::Auto);
    step_lux(a,s,snap,day,0.1);
    CHECK(s.get("cabin.lights")==0);
    CHECK(s.get("cabin.light_mode")==0);
    CHECK(mode().label=="AUTO"); CHECK(mode().active);
    CHECK(rg::DriverActions::state("lights",snap,s).label=="AUTO");
    // On: dipped beam in daylight, and it stays on however long it stays bright.
    press(day); CHECK(a.light_mode()==rg::LightMode::On); CHECK(s.get("cabin.lights")==1);
    CHECK(mode().label=="ON"); CHECK_FALSE(mode().active);
    CHECK(rg::DriverActions::state("lights",snap,s).label=="LIGHTS");
    step_lux(a,s,snap,day,600); CHECK(s.get("cabin.lights")==1);
    // Off: dark, and night does not switch them on.
    press(day); CHECK(a.light_mode()==rg::LightMode::Off); CHECK(s.get("cabin.lights")==0);
    CHECK(mode().label=="OFF");
    step_lux(a,s,snap,night,600); CHECK(s.get("cabin.lights")==0);
    CHECK(rg::DriverActions::state("lights",snap,s).label.empty());
    // Back to auto at night: on at once (the sensor already reads dark).
    press(night); CHECK(a.light_mode()==rg::LightMode::Auto); CHECK(s.get("cabin.lights")==1);
    CHECK(rg::DriverActions::state("lights",snap,s).label=="AUTO LIGHTS");
    // Main beam in auto at night is kept while it stays dark; dawn switches everything off.
    a.submit({"lights",rg::DriverActionKind::Set,2}); a.apply({snap,true},s); step_lux(a,s,snap,night,1);
    CHECK(s.get("cabin.lights")==2); CHECK(a.light_mode()==rg::LightMode::On); // Set 2 = on with main beam
    a.submit({"lights",rg::DriverActionKind::Set,-1}); a.apply({snap,true},s); step_lux(a,s,snap,night,1);
    CHECK(a.light_mode()==rg::LightMode::Auto); CHECK(s.get("cabin.lights")==2);
    step_lux(a,s,snap,day,cfg.auto_lights_off_delay_s+1); CHECK(s.get("cabin.lights")==0);
    // Set 0 / 1 pick off / on, as the cockpit switch and scripts use them.
    a.submit({"lights",rg::DriverActionKind::Set,0}); a.apply({snap,true},s); step_lux(a,s,snap,night,10);
    CHECK(a.light_mode()==rg::LightMode::Off); CHECK(s.get("cabin.lights")==0);
    a.submit({"lights",rg::DriverActionKind::Set,1}); a.apply({snap,true},s); step_lux(a,s,snap,day,10);
    CHECK(a.light_mode()==rg::LightMode::On); CHECK(s.get("cabin.lights")==1);
}

TEST_CASE("driver cabin: the main_beam action toggles high beam only while the headlamps are on", "[driver_actions][cabin][lights][auto_lights]") {
    const auto cfg = auto_lights_config();
    rg::DriverActions a; a.configure_cabin(cfg); Sink s; const auto snap=fitted();
    const double day = cfg.auto_lights_off_lux*10, night = cfg.auto_lights_on_lux*0.001;
    auto main_beam = [&](double lux) { a.submit({"main_beam"}); a.apply({snap,true},s); step_lux(a,s,snap,lux,0.1); };
    // Auto by day: lamps off, so the request does nothing and is not remembered for the night.
    step_lux(a,s,snap,day,0.1); main_beam(day);
    CHECK(s.get("cabin.lights")==0); CHECK_FALSE(a.cabin().main_beam);
    step_lux(a,s,snap,night,cfg.auto_lights_on_delay_s+1); CHECK(s.get("cabin.lights")==1);
    // Auto at night: toggles main beam on and off; the dipped beam stays.
    main_beam(night); CHECK(s.get("cabin.lights")==2); CHECK(rg::DriverActions::state("main_beam",snap,s).active);
    CHECK(rg::DriverActions::state("lights",snap,s).label=="AUTO MAIN BEAM");
    main_beam(night); CHECK(s.get("cabin.lights")==1); CHECK_FALSE(rg::DriverActions::state("main_beam",snap,s).active);
    // A held key fires once per press (edge), Set picks a state.
    a.submit({"main_beam",rg::DriverActionKind::Hold,1}); a.apply({snap,true},s); step_lux(a,s,snap,night,0.1);
    a.submit({"main_beam",rg::DriverActionKind::Hold,1}); a.apply({snap,true},s); step_lux(a,s,snap,night,0.1);
    CHECK(s.get("cabin.lights")==2);
    a.submit({"main_beam",rg::DriverActionKind::Set,1}); a.apply({snap,true},s); step_lux(a,s,snap,night,0.1);
    CHECK(s.get("cabin.lights")==2);
    // Dawn switches the auto lamps off and drops main beam; the next night comes back on dipped.
    step_lux(a,s,snap,day,cfg.auto_lights_off_delay_s+1); CHECK(s.get("cabin.lights")==0); CHECK_FALSE(a.cabin().main_beam);
    step_lux(a,s,snap,night,cfg.auto_lights_on_delay_s+1); CHECK(s.get("cabin.lights")==1);
    // Switch on by day: main beam works; switching off (lights key) drops it.
    a.submit({"lights",rg::DriverActionKind::Set,1}); a.apply({snap,true},s); step_lux(a,s,snap,day,0.1);
    main_beam(day); CHECK(s.get("cabin.lights")==2);
    a.submit({"lights"}); a.apply({snap,true},s); step_lux(a,s,snap,day,0.1); // On -> Off
    CHECK(s.get("cabin.lights")==0); CHECK_FALSE(a.cabin().main_beam);
    a.submit({"lights",rg::DriverActionKind::Set,1}); a.apply({snap,true},s); step_lux(a,s,snap,day,0.1);
    CHECK(s.get("cabin.lights")==1);
}

TEST_CASE("driver cabin: the headlamp switch survives a vehicle change and restores from its saved name", "[driver_actions][cabin][lights][auto_lights]") {
    rg::DriverActions a; a.configure_cabin(auto_lights_config()); Sink s; const auto snap=fitted();
    a.submit({"lights"}); a.submit({"lights",rg::DriverActionKind::Release,0}); a.apply({snap,true},s);
    a.submit({"lights"}); a.apply({snap,true},s);
    CHECK(a.light_mode()==rg::LightMode::Off);
    a.configure_auto_shift(true); // a new car: the cabin resets, the switch does not
    CHECK(a.light_mode()==rg::LightMode::Off);
    CHECK(a.cabin().auto_shift);
    rg::LightMode m = rg::LightMode::Auto;
    for (const char* name : {"auto", "on", "off"}) {
        REQUIRE(rg::parse_light_mode(name, m));
        CHECK(std::string(rg::light_mode_name(m)) == name);
        a.set_light_mode(m);
        CHECK(a.light_mode() == m);
    }
    CHECK_FALSE(rg::parse_light_mode("dipped", m));
    // A restored "on" lights the lamps at the next tick without a key press.
    a.set_light_mode(rg::LightMode::On); step_lux(a,s,snap,1e5,0.1); CHECK(s.get("cabin.lights")==1);
    // Without the auto config (cabin.json alone) auto holds whatever the lamps were.
    rg::DriverActions manual; manual.configure_cabin(cabin_config()); Sink m2;
    step_lux(manual,m2,snap,0.001,10); CHECK(m2.get("cabin.lights")==0);
}

TEST_CASE("driver cabin: automatic headlamp thresholds are universal (one config, no car or weather input)", "[driver_actions][cabin][auto_lights]") {
    // The sensor decision depends on the reading only: two different cars (snapshots) under the same
    // illuminance switch identically; weather enters only through the illuminance itself.
    const auto cfg = auto_lights_config();
    for (double lux : {cfg.auto_lights_on_lux*0.99, cfg.auto_lights_on_lux*1.01, cfg.auto_lights_off_lux*1.01}) {
        rg::DriverActions a, b; a.configure_cabin(cfg); b.configure_cabin(cfg); Sink sa, sb;
        auto snap_a = fitted(); auto snap_b = rg::DriverSnapshot{}; snap_b.speed_mps = 40; snap_b.gear = 5;
        step_lux(a,sa,snap_a,lux,0.1); step_lux(b,sb,snap_b,lux,0.1);
        CHECK(sa.get("cabin.lights") == sb.get("cabin.lights"));
        CHECK(sa.get("cabin.lights") == (lux < cfg.auto_lights_on_lux ? 1 : 0));
    }
}

TEST_CASE("driver cabin: invalid automatic headlamp settings are rejected", "[driver_actions][cabin][auto_lights]") {
    rg::DriverActions a;
    auto cfg = auto_lights_config(); cfg.auto_lights_off_lux = cfg.auto_lights_on_lux*0.5;
    CHECK_THROWS(a.configure_cabin(cfg));
    cfg = auto_lights_config(); cfg.auto_lights_on_lux = 0;
    CHECK_THROWS(a.configure_cabin(cfg));
    cfg = auto_lights_config(); cfg.auto_lights_on_delay_s = -1;
    CHECK_THROWS(a.configure_cabin(cfg));
    cfg = auto_lights_config(); cfg.auto_lights = false; cfg.auto_lights_on_lux = -1; // ignored while off
    CHECK_NOTHROW(a.configure_cabin(cfg));
}

TEST_CASE("EV start stop toggles READY without cranking or changing combustion ignition", "[driver_actions][ev]") {
    rg::DriverActions actions;actions.configure_cabin(cabin_config());
    Sink sink;sink.set("electric.ready",1);sink.set("ignition",0);
    auto snapshot=fitted();snapshot.transmission_kind=4;snapshot.electric_ready=true;
    CHECK(actions.submit({"start_stop"}));actions.apply({snapshot,true},sink);
    CHECK(sink.get("electric.ready")==0);CHECK(sink.get("starter")==0);CHECK(sink.get("ignition")==0);
    snapshot.electric_ready=false;
    CHECK(actions.submit({"start_stop"}));actions.apply({snapshot,true},sink);
    CHECK(sink.get("electric.ready")==1);CHECK(sink.get("starter")==0);
    snapshot.electric_ready=true;
    CHECK(rg::DriverActions::state("start_stop",snapshot,sink).label=="READY");
    snapshot.speed_mps=10;
    CHECK(actions.submit({"start_stop"}));actions.apply({snapshot,true},sink);
    CHECK(sink.get("electric.ready")==1);
}

TEST_CASE("driver actions: toggle_abs flips assist.abs and cycle_esc steps the offered ESC modes", "[driver_actions][abs][esc]") {
    rg::DriverSnapshot s;
    s.fitted_bits = rg::kAssistBitAbs | rg::kAssistBitEsc;
    s.abs_fitted = s.esc_fitted = true;
    s.esc_modes = 0b111; // off, on, sport
    rg::DriverActions actions; Sink sink;
    sink.values["assist.abs"] = 1; // the unit's default
    sink.values["assist.esc"] = 1;
    actions.submit({"toggle_abs"});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.abs") == 0);
    actions.submit({"toggle_abs"});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.abs") == 1);
    // ESC: on -> sport -> off -> on; presses in one frame chain through the value written last
    actions.submit({"cycle_esc"});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.esc") == 2);
    actions.submit({"cycle_esc"}); actions.submit({"cycle_esc"});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.esc") == 1);
    // a unit that offers only off and on never reaches sport
    s.esc_modes = 0b011;
    sink.values["assist.esc"] = 1;
    actions.submit({"cycle_esc"});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.esc") == 0);
    // the displays follow the state the keys changed
    s.abs_on = false;
    CHECK(rg::DriverActions::state("abs", s, sink).label == "ABS OFF");
    s.abs_on = true;
    CHECK(rg::DriverActions::state("abs", s, sink).label == "ABS");
    // an explicit Set (a script) works; a Set to an unoffered mode is refused
    actions.submit({"cycle_esc", rg::DriverActionKind::Set, 2});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.esc") == 0);
    actions.submit({"toggle_abs", rg::DriverActionKind::Set, 0});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.abs") == 0);
}

TEST_CASE("driver actions: the ABS and ESC keys do nothing on a car without the blocks", "[driver_actions][abs][esc]") {
    rg::DriverActions actions; Sink sink;
    rg::DriverSnapshot s = fitted(); // TC, aero, LC only
    s.esc_modes = 0b111;
    actions.submit({"toggle_abs"}); actions.submit({"cycle_esc"});
    actions.apply({s, true}, sink);
    CHECK(sink.values.count("assist.abs") == 0);
    CHECK(sink.values.count("assist.esc") == 0);
    // ABS only (the 1990s cars): the ESC key stays silent
    s.fitted_bits |= rg::kAssistBitAbs;
    actions.submit({"toggle_abs"}); actions.submit({"cycle_esc"});
    actions.apply({s, true}, sink);
    CHECK(sink.get("assist.abs") == 1);
    CHECK(sink.values.count("assist.esc") == 0);
}
