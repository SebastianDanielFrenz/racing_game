// dashboard_info.cpp - rg/dashboard_info.h: battery readout and tachometer layout.
#include "rg/dashboard_info.h"

#include "json_util.h"

#include "ps/drivetrain/powertrain_desc.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <variant>

namespace rg {

namespace {

namespace pd = ps::drivetrain;
constexpr double kPi = 3.14159265358979323846;

bool has_key(const detail::json& o, const char* key) { return o.find(key) != o.end(); }

}  // namespace

std::optional<DashboardConfig> parse_dashboard_config(const std::string& text, std::string* err) {
    const auto doc = detail::parse_json(text);
    if (!doc || !doc->is_object()) {
        if (err) *err = "dashboard config: not a JSON object";
        return std::nullopt;
    }
    DashboardConfig c;
    struct Field { const char* key; double* out; };
    const Field fields[] = {{"battery_low_soc", &c.battery_low_soc},
                            {"battery_critical_soc", &c.battery_critical_soc},
                            {"motor_dial_tick_krpm", &c.motor_dial_tick_krpm},
                            {"motor_dial_max_krpm", &c.motor_dial_max_krpm}};
    for (const auto& f : fields) {
        if (!has_key(*doc, f.key)) continue;
        if (!detail::get_number(*doc, f.key, *f.out) || !std::isfinite(*f.out)) {
            if (err) *err = std::string("dashboard config: '") + f.key + "' must be a number";
            return std::nullopt;
        }
    }
    if (c.battery_low_soc < 0 || c.battery_low_soc > 1 || c.battery_critical_soc < 0 ||
        c.battery_critical_soc > c.battery_low_soc) {
        if (err) *err = "dashboard config: need 0 <= battery_critical_soc <= battery_low_soc <= 1";
        return std::nullopt;
    }
    if (c.motor_dial_tick_krpm <= 0 || c.motor_dial_max_krpm < 0) {
        if (err) *err = "dashboard config: motor_dial_tick_krpm must be > 0 and motor_dial_max_krpm >= 0";
        return std::nullopt;
    }
    return c;
}

BatteryHud battery_hud(const pd::PowertrainSnapshot& p, const DashboardConfig& cfg) {
    BatteryHud h;
    h.fitted = p.battery_capacity_kwh > 0.0;
    if (!h.fitted) return h;
    h.soc = std::clamp(static_cast<double>(p.battery_soc), 0.0, 1.0);
    h.percent = static_cast<int>(std::lround(100.0 * h.soc));
    h.level = h.soc < cfg.battery_critical_soc ? 2 : (h.soc < cfg.battery_low_soc ? 1 : 0);
    char buf[24];
    std::snprintf(buf, sizeof buf, "SoC %d%%", h.percent);
    h.label = buf;
    return h;
}

TachScales tach_scales(const ps::vehicle::VehicleDesc& desc, const DashboardConfig& cfg) {
    TachScales s;
    bool has_engine = false;
    const pd::ElectricMotorDesc* traction = nullptr;
    std::string traction_id;
    for (const auto& c : desc.powertrain.components) {
        if (const auto* e = std::get_if<pd::TorqueMapEngineDesc>(&c.params)) {
            has_engine = true;
            s.ice_limiter_rpm = e->limiter.rpm;
        } else if (const auto* e = std::get_if<pd::SimulatedEngineDesc>(&c.params)) {
            has_engine = true;
            s.ice_limiter_rpm = e->limiter.rpm;
        } else if (const auto* m = std::get_if<pd::ElectricMotorDesc>(&c.params)) {
            // The crank machine (generator/starter) follows the engine; the traction motor is the other one.
            if (!m->crank_assist && (traction == nullptr || m->max_rpm > traction->max_rpm)) {
                traction = m;
                traction_id = c.id;
            }
        }
    }
    s.kind = traction == nullptr ? TachKind::Ice : (has_engine ? TachKind::Hybrid : TachKind::Ev);
    if (traction == nullptr) return s;

    s.motor_limit_rpm = traction->max_rpm;
    s.motor_dial_tick_rpm = 1000.0 * cfg.motor_dial_tick_krpm;
    const double wanted = cfg.motor_dial_max_krpm > 0 ? 1000.0 * cfg.motor_dial_max_krpm : traction->max_rpm;
    s.motor_dial_max_rpm = std::ceil(wanted / s.motor_dial_tick_rpm - 1e-9) * s.motor_dial_tick_rpm;

    // Ratio to the gearbox output: the motor's shaft joins a gear pair's input port (motor rpm = out rpm x ratio).
    if (s.kind == TachKind::Hybrid) {
        for (const auto& shaft : desc.powertrain.shafts) {
            bool has_motor = false;
            for (const auto& port : shaft.ports) has_motor = has_motor || (port.component == traction_id && !port.is_wheel);
            if (!has_motor) continue;
            for (const auto& port : shaft.ports) {
                if (port.port != "in") continue;
                for (const auto& c : desc.powertrain.components) {
                    if (c.id != port.component) continue;
                    if (const auto* g = std::get_if<pd::GearPairDesc>(&c.params)) s.traction_ratio = g->ratio;
                }
            }
        }
    }
    return s;
}

double traction_motor_rpm(const pd::PowertrainSnapshot& p, const TachScales& scales) {
    if (scales.traction_ratio > 0.0)
        return std::abs(static_cast<double>(p.driveline.gearbox_out_omega)) * scales.traction_ratio * 30.0 / kPi;
    return static_cast<double>(p.motor_rpm);
}

}  // namespace rg
