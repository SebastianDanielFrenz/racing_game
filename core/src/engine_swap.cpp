// rg/engine_swap.cpp - see engine_swap.h.
#include "rg/engine_swap.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace rg {

namespace {

std::string num0(double v) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.0f", v);
    return buf;
}

double round_to(double v, double step) { return std::round(v / step) * step; }

} // namespace

std::string fuel_label(const std::string& stem) {
    std::string out;
    std::size_t pos = 0;
    bool first = true;
    while (pos <= stem.size()) {
        const std::size_t us = stem.find('_', pos);
        std::string word = stem.substr(pos, us == std::string::npos ? std::string::npos : us - pos);
        bool grade = word.size() >= 2 && word[0] == 'e';
        for (std::size_t i = 1; i < word.size() && grade; ++i) grade = word[i] >= '0' && word[i] <= '9';
        if (grade || (first && !word.empty())) word[0] = static_cast<char>(word[0] >= 'a' && word[0] <= 'z' ? word[0] - 'a' + 'A' : word[0]);
        if (!first) out += ' ';
        out += word;
        first = false;
        if (us == std::string::npos) break;
        pos = us + 1;
    }
    return out;
}

SwapPackage derive_swap_package(const SwapCar& car, const SwapEngine& engine, const SwapChoices& choices, bool engine_swapped) {
    SwapPackage p;
    p.engine_swapped = engine_swapped;

    // ---- fuel (D1) ----
    if (car.has_tank) {
        if (engine.fuels.empty()) {
            p.ok = false;
            p.error = "the engine lists no fuel";
            return p;
        }
        const auto lists = [&](const std::string& f) { return std::find(engine.fuels.begin(), engine.fuels.end(), f) != engine.fuels.end(); };
        std::string wanted = car.tank_fuel;
        if (!choices.fuel.empty()) {
            if (!lists(choices.fuel)) {
                p.ok = false;
                p.error = "fuel \"" + choices.fuel + "\" is not one of this engine's fuels (" + [&] {
                    std::string s;
                    for (const auto& f : engine.fuels) s += (s.empty() ? "" : ", ") + f;
                    return s;
                }() + ")";
                return p;
            }
            wanted = choices.fuel;
        } else if (!lists(wanted)) {
            wanted = engine.fuels.front();
            p.lines.push_back("Fuel changed to " + fuel_label(wanted) + ": this engine does not list " + fuel_label(car.tank_fuel));
        } else if (engine_swapped) {
            p.lines.push_back("Fuel kept: " + fuel_label(wanted));
        }
        p.fuel = wanted;
        p.fuel_changed = wanted != car.tank_fuel;
        p.set_fuel = p.fuel_changed;
        if (!choices.fuel.empty() && p.fuel_changed) p.lines.push_back("Fuel: " + fuel_label(wanted));
    }

    if (!engine_swapped) return p;

    // ---- clutch ----
    if (car.clutch_kinetic_nm > 0.0 && engine.peak_torque_nm > 0.0) {
        const double wanted = std::ceil(1.5 * engine.peak_torque_nm / 10.0) * 10.0;
        if (car.clutch_kinetic_nm < wanted) {
            p.set_clutch = true;
            p.clutch_kinetic_nm = wanted;
            p.lines.push_back("Clutch capacity raised from " + num0(car.clutch_kinetic_nm) + " to " + num0(wanted) +
                              " N*m (1.5 x the engine's " + num0(engine.peak_torque_nm) + " N*m)");
        } else {
            p.lines.push_back("Clutch kept at " + num0(car.clutch_kinetic_nm) + " N*m (already above 1.5 x the engine's " +
                              num0(engine.peak_torque_nm) + " N*m)");
        }
    }

    // ---- TCU ----
    if (engine.idle_rpm > 0.0 && engine.limiter_rpm > engine.idle_rpm) {
        p.set_tcu = true;
        SwapAutoClutch& c = p.auto_clutch;
        c.bite_rpm = engine.idle_rpm + 300.0;
        c.anti_stall_rpm = engine.idle_rpm - 100.0;
        c.rolling_reengage_rpm = engine.idle_rpm + 500.0;
        c.full_rpm_min = c.bite_rpm + 500.0;
        c.full_rpm_max = std::max(c.full_rpm_min, round_to(0.45 * engine.limiter_rpm, 50.0));
        SwapAutoShift& s = p.auto_shift;
        s.up_full = round_to(engine.limiter_rpm - 300.0, 50.0);
        s.up_zero = round_to(0.45 * engine.limiter_rpm, 50.0);
        s.down_zero = std::max(engine.idle_rpm + 300.0, round_to(0.19 * engine.limiter_rpm, 50.0));
        s.down_full = std::max(s.down_zero + 500.0, round_to(0.41 * engine.limiter_rpm, 50.0));
        p.zero_blip = engine.mechanical_throttle;
        p.lines.push_back("Clutch assist: bites at " + num0(c.bite_rpm) + " rpm, anti-stall at " + num0(c.anti_stall_rpm) + " rpm");
        p.lines.push_back("Auto shift: up at " + num0(s.up_full) + " rpm at full throttle (limiter " + num0(engine.limiter_rpm) + " rpm)");
    }

    // ---- parts that belong to the old engine ----
    p.drop_slot_overrides = car.has_slot_overrides;
    if (car.has_slot_overrides) p.lines.push_back("The old engine's turbo and exhaust choices were removed with it");
    if (car.has_cooling_system && !engine.has_thermal) {
        p.drop_cooling_system = true;
        p.lines.push_back("Engine temperature gauge removed: this engine has no thermal data");
    }
    if (car.has_nitrous && !engine.simulated) {
        p.drop_nitrous = true;
        p.lines.push_back("Nitrous kit removed: this engine has no air path model for it");
    }

    // ---- mass ----
    if (engine.mass_kg > 0.0 && car.stock_engine_mass_kg > 0.0) {
        p.mass_delta_kg = engine.mass_kg - car.stock_engine_mass_kg;
        if (std::abs(p.mass_delta_kg) >= 0.5) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%+.0f kg", p.mass_delta_kg);
            p.lines.push_back(std::string("Weight ") + buf + " (engine " + num0(engine.mass_kg) + " kg instead of " +
                              num0(car.stock_engine_mass_kg) + " kg)");
        }
    }
    return p;
}

FitResult check_fit(const CatalogEntry& car, const EngineEntry& engine) {
    // Reserved: car engine_bay_envelope_mm vs engine envelope_mm, mount pattern, driveline. Every engine fits today.
    (void)car;
    (void)engine;
    return FitResult{};
}

} // namespace rg
