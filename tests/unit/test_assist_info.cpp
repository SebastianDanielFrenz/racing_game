// test_assist_info.cpp - the driver-assist state the game reads from physics_sim's assist unit (rg/assist_info.h).
//
// - The F4 cycle goes through the traction-control modes the car OFFERS only (PowertrainSnapshot::
//   traction_modes_available_bits: Street 0b00011, Sport 0b01111, Racing 0b11111, a car without a unit 0b00011): a
//   mode the car does not offer is never selectable (owner rule).
// - The H toggle flips assist.aero_control only on a car that has the automation (assist_fitted_bits bit 6).
// - The game's car_hyper.json and car_hyper_n2o.json carry the Racing assist unit (as physics_sim's own hyper files do),
//   every other game car has none; the files are loaded validate-only (no engine-map generation).
#include "rg/vehicle_load.h"
#include "rg/assist_info.h"
#include "rg/session.h"

#include "ps/io/vehicle_io.h"
#include "ps/vehicle/assist_unit_desc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <string>
#include <vector>

namespace {

const std::string kRepo = RG_SOURCE_DIR;

constexpr std::uint32_t kStreet = 0b00011;
constexpr std::uint32_t kSport = 0b01111;
constexpr std::uint32_t kRacing = 0b11111;

// The sequence of codes F4 visits from `start`, n presses.
std::vector<int> presses(std::uint32_t bits, int start, int n) {
    std::vector<int> out;
    int mode = start;
    for (int i = 0; i < n; ++i) {
        mode = rg::next_traction_mode(bits, mode);
        out.push_back(mode);
    }
    return out;
}

ps::vehicle::VehicleDesc load_game_car(const std::string& file) {
    ps::io::EngineMapOptions options;
    options.generate = false; // validate-only: every loader check runs, no maps are generated
    return rg::load_vehicle(kRepo + "/data/vehicles/" + file, options);
}

} // namespace

TEST_CASE("assist cycle: a Racing unit steps through all five modes and wraps", "[assist][controls]") {
    CHECK(presses(kRacing, 0, 6) == std::vector<int>{1, 2, 3, 4, 0, 1});
    CHECK(presses(kRacing, 4, 2) == std::vector<int>{0, 1});
}

TEST_CASE("assist cycle: a Sport unit never reaches Drift, a Street unit only Off and On", "[assist][controls]") {
    CHECK(presses(kSport, 0, 5) == std::vector<int>{1, 2, 3, 0, 1});
    CHECK(presses(kStreet, 0, 4) == std::vector<int>{1, 0, 1, 0});
    for (int start = 0; start < rg::kTractionModeCodeCount; ++start) {
        for (int code : presses(kStreet, start, 8)) CHECK((code == 0 || code == 1));
        for (int code : presses(kSport, start, 8)) CHECK(code != 4);
    }
}

TEST_CASE("assist cycle: an unavailable starting mode is only a starting point", "[assist][controls]") {
    // A Street car whose requested code was 3 (physics maps it to On): the next press goes to an offered mode.
    CHECK(rg::next_traction_mode(kStreet, 3) == 0);
    CHECK(rg::next_traction_mode(kStreet, 4) == 0);
    CHECK(rg::next_traction_mode(kStreet, 2) == 0);
    // Out-of-range input is treated as "before Off".
    CHECK(rg::next_traction_mode(kRacing, -7) == 0);
    CHECK(rg::next_traction_mode(kRacing, 99) == 0);
}

TEST_CASE("assist cycle: no offered mode returns -1, a single offered mode is returned every time", "[assist][controls]") {
    CHECK(rg::next_traction_mode(0, 0) == -1);
    CHECK(rg::next_traction_mode(0, 3) == -1);
    CHECK(rg::next_traction_mode(0b00100, 0) == 2);
    CHECK(rg::next_traction_mode(0b00100, 2) == 2);
    // Bits above the five codes are ignored.
    CHECK(rg::next_traction_mode(0b100000, 0) == -1);
    // A unit without Off (hypothetical data): the cycle never lands on Off.
    CHECK(presses(0b11110, 4, 3) == std::vector<int>{1, 2, 3});
}

TEST_CASE("assist cycle: mode names and offered bits", "[assist][controls]") {
    CHECK(std::string(rg::traction_mode_name(0)) == "Off");
    CHECK(std::string(rg::traction_mode_name(1)) == "On");
    CHECK(std::string(rg::traction_mode_name(2)) == "Relaxed");
    CHECK(std::string(rg::traction_mode_name(3)) == "Sport");
    CHECK(std::string(rg::traction_mode_name(4)) == "Drift");
    CHECK(std::string(rg::traction_mode_name(5)) == "?");
    CHECK(rg::traction_mode_offered(kSport, 3));
    CHECK_FALSE(rg::traction_mode_offered(kSport, 4));
    CHECK_FALSE(rg::traction_mode_offered(kSport, -1));
}

TEST_CASE("assist aero toggle: flips only on a car with the automation", "[assist][controls][aero]") {
    const std::uint32_t with_aero = rg::kAssistBitAeroControl | rg::kAssistBitTractionControl;
    CHECK(rg::toggled_aero_control(with_aero, 1.0) == 0.0);
    CHECK(rg::toggled_aero_control(with_aero, 0.0) == 1.0);
    CHECK(rg::toggled_aero_control(with_aero, 0.49) == 1.0);
    // TC fitted but no wing automation: the press does nothing.
    CHECK(rg::toggled_aero_control(rg::kAssistBitTractionControl, 0.0) == 0.0);
    CHECK(rg::toggled_aero_control(rg::kAssistBitTractionControl, 1.0) == 1.0);
    CHECK(rg::toggled_aero_control(0, 0.0) == 0.0);
}

TEST_CASE("assist hud: the snapshot fields map to the HUD state", "[assist][controls]") {
    ps::drivetrain::PowertrainSnapshot p;
    p.assist_fitted_bits = rg::kAssistBitTractionControl | rg::kAssistBitAeroControl;
    p.traction_mode_requested = 4;
    p.traction_mode_effective = 3;
    p.traction_modes_available_bits = static_cast<std::uint8_t>(kSport);
    p.traction_control_intervening = true;
    p.aero_control_active = true;
    p.aero_control_target_deg = 4.0f;
    const rg::AssistHud h = rg::assist_hud(p);
    CHECK(h.tc_fitted);
    CHECK(h.tc_requested == 4);
    CHECK(h.tc_effective == 3);
    CHECK(h.tc_available_bits == kSport);
    CHECK(h.tc_intervening);
    CHECK(h.aero_fitted);
    CHECK(h.aero_active);
    CHECK(h.aero_target_deg == 4.0);

    const rg::AssistHud none = rg::assist_hud(ps::drivetrain::PowertrainSnapshot{});
    CHECK_FALSE(none.tc_fitted);
    CHECK_FALSE(none.aero_fitted);
    CHECK_FALSE(none.aero_active);
}

TEST_CASE("game cars: both hypercars carry the Racing assist unit, no other car has one", "[assist][vehicle_data]") {
    for (const char* file : {"car_hyper.json", "car_hyper_n2o.json"}) {
        INFO(file);
        const ps::vehicle::VehicleDesc desc = load_game_car(file);
        REQUIRE(desc.assist_unit.has_value());
        CHECK(desc.assist_unit->grade == ps::vehicle::AssistGrade::Racing);
        CHECK(desc.assist_unit->has_traction_control);
        CHECK(desc.traction_control.fitted);
        // The five modes, so F4 can reach Drift.
        CHECK(ps::vehicle::traction_modes_available_bits(desc.assist_unit, desc.traction_control.fitted) == kRacing);
        // TC starts On (the unit's default mode), not by a game-side switch.
        CHECK(ps::vehicle::traction_default_channel_value(desc.traction_control, desc.assist_unit) == 1.0);
        // The unit's aero automation finds the car's rear wing, so assist.aero_control exists and defaults on.
        REQUIRE(desc.assist_unit->aero_control.has_value());
        CHECK(desc.assist_unit->aero_control->default_enabled);
        const auto wing = ps::vehicle::find_aero_control_surface(desc.aero, desc.assist_unit);
        REQUIRE(wing.has_value());
        CHECK(desc.aero.surfaces[*wing].name == "rear_wing");
    }
    for (const char* file : {"car_sedan.json", "car_sedan_rwd.json", "car_sedan_awd.json", "car_sedan_gen_n2o.json"}) {
        INFO(file);
        const ps::vehicle::VehicleDesc desc = load_game_car(file);
        CHECK_FALSE(desc.assist_unit.has_value());
        // No unit: Off and On only, and the game no longer forces TC on (the car file's own `enabled`, 0).
        CHECK(ps::vehicle::traction_modes_available_bits(desc.assist_unit, desc.traction_control.fitted) == kStreet);
        CHECK(ps::vehicle::traction_default_channel_value(desc.traction_control, desc.assist_unit) == 0.0);
    }
}

// A real Session on the game's car_hyper.json (engine maps from the cache or generated, so [long]): the channels start
// at the unit's defaults without any game-side switch, the snapshot reports the five modes and the fitted/active wing
// automation, and the game no longer commands the wing itself (physics owns it: the World's own command stays 0).
TEST_CASE("assist: a hypercar Session starts with TC On and the wing automation on from the unit's defaults", "[assist][session][long]") {
    rg::SessionConfig config;
    config.vehicle_json_path = kRepo + "/data/vehicles/car_hyper.json";
    config.surface_table_path = kRepo + "/data/surfaces/surfaces.json";
    rg::Session session(config);
    CHECK(session.get_control("assist.traction_control") == 1.0);
    CHECK(session.get_control("assist.aero_control") == 1.0);
    for (int i = 0; i < 240; ++i) session.step();
    const auto pt = session.world().powertrain_state(session.vehicle_id());
    CHECK(pt.traction_modes_available_bits == kRacing);
    CHECK(pt.traction_mode_effective == 1);
    CHECK((pt.assist_fitted_bits & rg::kAssistBitAeroControl) != 0);
    CHECK(pt.aero_control_active);
    // Switching the automation off through the channel (what the H key does) parks it.
    session.world().set_control("assist.aero_control", 0.0);
    for (int i = 0; i < 12; ++i) session.step();
    CHECK_FALSE(session.world().powertrain_state(session.vehicle_id()).aero_control_active);
}
