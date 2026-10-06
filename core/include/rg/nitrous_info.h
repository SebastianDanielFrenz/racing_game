// rg/nitrous_info.h - what the HUD needs to know about a car's nitrous kit (physics_sim N2O plan N1/N2).
//
// Engine-neutral and header-only: the Godot binding only forwards these values. A car has a kit iff its powertrain
// declares an N2oBottleDesc component; the arm switch is the plain 0/1 control channel "nitrous_arm" (gamepad Y in
// the controls, rg::Session::kControlChannelNames); the solenoid state, bottle content and flow come from the
// engine's EngineSoundState (n2o_* fields, charge_temperature_k).
#pragma once

#include "ps/drivetrain/nitrous_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/vehicle/vehicle_desc.h"

#include <string_view>
#include <variant>

namespace rg {

struct NitrousInfo {
    bool present = false;
    double bottle_capacity_kg = 0.0;
    double bottle_initial_kg = 0.0;
    double kit_rating_kw = 0.0; // metadata only (the engine never reads it)
};

[[nodiscard]] inline NitrousInfo nitrous_info(const ps::vehicle::VehicleDesc& desc) {
    NitrousInfo info;
    for (const auto& component : desc.powertrain.components) {
        if (const auto* bottle = std::get_if<ps::drivetrain::N2oBottleDesc>(&component.params)) {
            info.present = true;
            info.bottle_capacity_kg += static_cast<double>(bottle->capacity_kg);
            info.bottle_initial_kg += static_cast<double>(bottle->initial_kg);
        } else if (const auto* kit = std::get_if<ps::drivetrain::NitrousKitDesc>(&component.params)) {
            info.present = true;
            info.kit_rating_kw += static_cast<double>(kit->rating.power_kw);
        }
    }
    return info;
}

// NitrousSafetyCut as the engine reports it (EngineSoundState::n2o_safety_cut).
[[nodiscard]] constexpr std::string_view nitrous_cut_name(int cut) {
    switch (cut) {
        case 1: return "bottle pressure";
        case 2: return "fuel starve";
        case 3: return "duty time";
        default: return "";
    }
}

// The one-word state the HUD shows. A safety cut wins over everything (the kit is off whatever the switch says),
// then an open solenoid, then the arm switch.
[[nodiscard]] constexpr std::string_view nitrous_state_label(bool present, bool armed, bool spraying, int safety_cut,
                                                             bool purging = false) {
    if (!present) return "none";
    if (safety_cut != 0) return "CUT";
    if (spraying) return "SPRAYING";
    if (purging) return "PURGE";
    return armed ? "ARMED" : "off";
}

// Everything the HUD line shows for one frame (snapshot values, no physics).
struct NitrousHud {
    bool present = false;
    bool armed = false;    // the nitrous_arm switch
    bool spraying = false; // solenoid open and N2O flowing
    bool solenoid_open = false; // controller requests spraying and no cut holds (true with an empty bottle, no flow)
    bool purging = false;       // the purge vents the line (N4)
    double flow_fraction = 0.0; // progressive duty 0..1 (N4)
    int safety_cut = 0;
    double bottle_kg = 0.0, bottle_capacity_kg = 0.0, bottle_bar = 0.0;
    double flow_g_s = 0.0, kit_fuel_g_s = 0.0, retard_deg = 0.0;
    double charge_temperature_k = 0.0, lambda_combined = 0.0;
};

[[nodiscard]] inline NitrousHud nitrous_hud(const NitrousInfo& info, const ps::drivetrain::PowertrainSnapshot& p,
                                            double arm_channel) {
    NitrousHud h;
    h.present = info.present;
    if (!info.present) return h;
    h.armed = arm_channel >= 0.5;
    h.bottle_kg = static_cast<double>(p.n2o_bottle_kg);
    h.bottle_capacity_kg = info.bottle_capacity_kg;
    if (!p.engines.empty()) {
        const auto& e = p.engines[0];
        h.spraying = e.n2o_active != 0;
        h.solenoid_open = e.n2o_solenoid_open != 0;
        h.purging = e.n2o_purge_active != 0;
        h.flow_fraction = static_cast<double>(e.n2o_flow_fraction);
        h.safety_cut = static_cast<int>(e.n2o_safety_cut);
        h.bottle_bar = static_cast<double>(e.n2o_bottle_bar);
        h.flow_g_s = static_cast<double>(e.n2o_flow_g_s);
        h.kit_fuel_g_s = static_cast<double>(e.n2o_kit_fuel_g_s);
        h.retard_deg = static_cast<double>(e.n2o_retard_deg);
        h.charge_temperature_k = static_cast<double>(e.charge_temperature_k);
        h.lambda_combined = static_cast<double>(e.n2o_lambda_combined);
    }
    return h;
}

} // namespace rg
