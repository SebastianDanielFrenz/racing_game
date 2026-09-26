// test_vehicle_data.cpp — racing_game-owned vehicle/engine data (carvis
// brief 2026-09-27, "engine30": the 3.0 L N52B30-based sedan variant,
// data/vehicles/car_sedan.json + data/engines/n52b30_3l_na.json). Owner
// ruling for this pass, verbatim: "Don't overthink: test ONLY the clutch...
// There is no launch, stall, top-speed or other test; the owner finds
// anything else by driving." and "Wheelspin is allowed. Do not tune it
// away, and do not gate or test on it." — so this is deliberately the ONLY
// test for the new data files: the clutch's static torque capacity must
// clear the engine's own peak WOT torque by a safety margin, both read back
// from the loaded ps::vehicle::VehicleDesc (nothing hand-copied from the
// JSON's own "source" commentary, which is documentation, not a checked
// value).

#include "ps/io/vehicle_io.h"
#include "ps/drivetrain/powertrain_desc.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <string>

TEST_CASE("racing_game's 3.0 L sedan: clutch static capacity clears the engine's peak WOT torque",
          "[vehicle_data]") {
    const std::string path = std::string(RG_SOURCE_DIR) + "/data/vehicles/car_sedan.json";
    const ps::vehicle::VehicleDesc desc = ps::io::load_vehicle_json(path);

    const ps::drivetrain::TorqueMapEngineDesc* engine = nullptr;
    const ps::drivetrain::ClutchDesc* clutch = nullptr;
    for (const auto& component : desc.powertrain.components) {
        if (const auto* e = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&component.params)) {
            REQUIRE(engine == nullptr); // exactly one engine expected
            engine = e;
        } else if (const auto* c = std::get_if<ps::drivetrain::ClutchDesc>(&component.params)) {
            REQUIRE(clutch == nullptr); // exactly one clutch expected
            clutch = c;
        }
    }
    REQUIRE(engine != nullptr);
    REQUIRE(clutch != nullptr);

    REQUIRE_FALSE(engine->wot_torque_nm_vs_rpm.y.empty());
    const ps::real peak_wot_torque_nm =
        *std::max_element(engine->wot_torque_nm_vs_rpm.y.begin(), engine->wot_torque_nm_vs_rpm.y.end());
    REQUIRE(peak_wot_torque_nm > 0.0);

    const ps::real clutch_static_capacity_nm = clutch->capacity_nm * clutch->static_factor;

    // Owner ruling (this brief): "the clutch's static torque capacity is at
    // least 1.2x the engine's peak WOT torque" — the ONLY numeric floor this
    // data file is held to.
    REQUIRE(clutch_static_capacity_nm >= 1.2 * peak_wot_torque_nm);
}
