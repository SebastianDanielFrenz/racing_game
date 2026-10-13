#include "rg/driver_actions.h"
#include <catch2/catch_test_macros.hpp>
#include <array>
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
