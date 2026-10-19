// rg/limit_advisor.cpp - see limit_advisor.h.
#include "rg/limit_advisor.h"

#include "rg/vehicle_load.h"

#include "json_util.h"

#include "ps/drivetrain/electric_drive_desc.h"
#include "ps/drivetrain/powertrain_dyno.h"
#include "ps/io/limit_policy_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <set>
#include <stdexcept>
#include <string_view>
#include <utility>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace rg {

namespace pd = ps::drivetrain;

namespace {

using detail::json;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr const char* kSquared = "\xC2\xB2"; // UTF-8 superscript two

// ---- numbers ----------------------------------------------------------------------------------------------------------

std::string fixed(double v, int decimals) {
    if (!std::isfinite(v)) return "?";
    decimals = std::clamp(decimals, 0, 6);
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", decimals, v);
    std::string s(buf);
    // "-0" / "-0.0": a value that rounds to zero carries no sign.
    bool zero = true;
    for (const char c : s) {
        if (c >= '1' && c <= '9') zero = false;
    }
    if (zero && !s.empty() && s[0] == '-') s.erase(0, 1);
    return s;
}

// The built-in default decimals per quantity (the config may override every one).
int default_decimals(const std::string& q) {
    static const std::map<std::string, int> d = {{"bar", 1}, {"rpm", 0}, {"nm", 0},   {"kw", 0},       {"percent", 0}, {"ratio", 2}, {"mps", 1},
                                                 {"mm", 2},  {"mm2", 0}, {"kg", 1},   {"seconds", 1}, {"degrees", 1}, {"kpa", 0},   {"plain", 2}, {"flow", 3}};
    const auto it = d.find(q);
    return it == d.end() ? 1 : it->second;
}

int dec(const AdvisorConfig& c, const char* q) { return c.decimals_for(q, default_decimals(q)); }

std::string fmt_rpm(double rpm, const AdvisorConfig& c) { return std::isfinite(rpm) ? fixed(rpm, dec(c, "rpm")) + " RPM" : "?"; }

std::string fmt_bar(double pa, const AdvisorConfig& c) { return std::isfinite(pa) ? fixed(pa / 1.0e5, dec(c, "bar")) + " bar" : "?"; }
std::string fmt_kpa(double pa, const AdvisorConfig& c) { return std::isfinite(pa) ? fixed(pa / 1.0e3, dec(c, "kpa")) + " kPa" : "?"; }
std::string fmt_pct(double fraction, const AdvisorConfig& c) { return std::isfinite(fraction) ? fixed(fraction * 100.0, dec(c, "percent")) + " %" : "?"; }

// One value in the unit physics reports it in. Pascal is boost in bar (gauge) unless `pascal_kpa` (a manifold axis).
std::string fmt_quantity(double v, pd::LimitUnit unit, const AdvisorConfig& c, bool pascal_kpa = false) {
    if (!std::isfinite(v)) return "?";
    switch (unit) {
        case pd::LimitUnit::Dimensionless: return fixed(v, dec(c, "plain"));
        case pd::LimitUnit::Fraction: return fmt_pct(v, c);
        case pd::LimitUnit::Pascal: return pascal_kpa ? fmt_kpa(v, c) : fmt_bar(v, c);
        case pd::LimitUnit::Kelvin: return fixed(v, 0) + " K";
        case pd::LimitUnit::NewtonMetre: return fixed(v, dec(c, "nm")) + " N*m";
        case pd::LimitUnit::MetrePerSecond: return fixed(v, dec(c, "mps")) + " m/s";
        case pd::LimitUnit::RevPerMinute: return fmt_rpm(v, c);
        case pd::LimitUnit::SquareMetre: return fixed(v * 1.0e6, dec(c, "mm2")) + " mm" + kSquared;
        case pd::LimitUnit::Millimetre: return fixed(v, dec(c, "mm")) + " mm";
        case pd::LimitUnit::Kilogram: return fixed(v, dec(c, "kg")) + " kg";
        case pd::LimitUnit::Second: return fixed(v, dec(c, "seconds")) + " s";
        case pd::LimitUnit::CrankDegree: return fixed(v, dec(c, "degrees")) + " deg";
    }
    return fixed(v, 2);
}

// ---- config parsing ---------------------------------------------------------------------------------------------------

bool fail(std::string* err, const std::string& origin, const std::string& message) {
    if (err != nullptr) *err = origin + ": " + message;
    return false;
}

std::optional<pd::LimitKind> kind_from_name(const std::string& name) {
    for (int v = 0; v <= 18; ++v) {
        const auto k = static_cast<pd::LimitKind>(v);
        if (name == pd::to_string(k)) return k;
    }
    return std::nullopt;
}

std::optional<pd::LimitSeverity> severity_from_name(const std::string& n, bool* ok) {
    *ok = true;
    if (n == "near") return pd::LimitSeverity::Near;
    if (n == "limiting") return pd::LimitSeverity::Limiting;
    if (n == "exceeded") return pd::LimitSeverity::Exceeded;
    if (n == "never") return std::nullopt;
    *ok = false;
    return std::nullopt;
}

// Checks every {name} of a template against the facts the builder produces.
bool check_template(const std::string& tpl, const std::string& where, const std::string& origin, std::string* err) {
    const auto& known = known_advice_fact_names();
    for (std::size_t i = 0; i < tpl.size(); ++i) {
        if (tpl[i] == '}') return fail(err, origin, where + ": a '}' without '{' in \"" + tpl + "\"");
        if (tpl[i] != '{') continue;
        const std::size_t end = tpl.find('}', i);
        if (end == std::string::npos) return fail(err, origin, where + ": an unclosed '{' in \"" + tpl + "\"");
        const std::string name = tpl.substr(i + 1, end - i - 1);
        if (std::find(known.begin(), known.end(), name) == known.end()) {
            return fail(err, origin, where + ": unknown placeholder {" + name + "}");
        }
        i = end;
    }
    return true;
}

bool read_text(const json& o, const char* key, std::string& out, bool required, const std::string& where, const std::string& origin,
               std::string* err) {
    const auto it = o.find(key);
    if (it == o.end()) {
        if (required) return fail(err, origin, where + ": \"" + key + "\" is missing");
        return true;
    }
    if (!it->is_string()) return fail(err, origin, where + ": \"" + key + "\" must be text");
    out = it->get<std::string>();
    return check_template(out, where + "." + key, origin, err);
}

bool read_advice_text(const json& o, bool base, AdviceText& out, const std::string& where, const std::string& origin, std::string* err) {
    if (!read_text(o, "title", out.title, base, where, origin, err)) return false;
    if (!read_text(o, "suggestion", out.suggestion, false, where, origin, err)) return false;
    if (!read_text(o, "remedy", out.remedy, false, where, origin, err)) return false;
    if (!read_text(o, "remedy_failed", out.remedy_failed, false, where, origin, err)) return false;
    detail::get_string(o, "option", out.option);
    std::string cap;
    if (detail::get_string(o, "max_severity", cap)) {
        bool ok = false;
        out.max_severity = severity_from_name(cap, &ok);
        if (!ok || !out.max_severity) return fail(err, origin, where + ": \"max_severity\" must be near / limiting / exceeded");
    }
    const auto lines = o.find("lines");
    if (lines == o.end()) {
        if (base) return fail(err, origin, where + ": \"lines\" is missing");
        return true;
    }
    if (!lines->is_array() || (base && lines->empty())) return fail(err, origin, where + ": \"lines\" must be a" + (base ? " non-empty" : "n") + " array of text");
    for (const json& l : *lines) {
        if (!l.is_string()) return fail(err, origin, where + ": \"lines\" must hold text");
        out.lines.push_back(l.get<std::string>());
        if (!check_template(out.lines.back(), where + ".lines", origin, err)) return false;
    }
    return true;
}

bool read_kind_body(const json& o, const std::string& where, const std::string& origin, AdviceKindConfig& k, std::string* err) {
        // Optional: absent = the limit policy alone decides what is shown; "never" hides the kind; a severity is a
        // kind-specific floor above the policy's.
        if (o.contains("min_severity")) {
            std::string sev;
            bool ok = false;
            if (!detail::get_string(o, "min_severity", sev)) {
                fail(err, origin, where + ": \"min_severity\" must be near / limiting / exceeded / never");
                return false;
            }
            k.min_severity = severity_from_name(sev, &ok);
            if (!ok) {
                fail(err, origin, where + ": \"min_severity\" must be near / limiting / exceeded / never");
                return false;
            }
            if (sev == "never") k.hidden = true;
        }
        detail::get_string(o, "option", k.option_id);
        if (!read_advice_text(o, true, k.text, where, origin, err)) return false;
        const auto variants = o.find("variants");
        if (variants != o.end()) {
            if (!variants->is_object()) {
                fail(err, origin, where + ": \"variants\" must be an object");
                return false;
            }
            for (auto vt = variants->begin(); vt != variants->end(); ++vt) {
                if (!vt->is_object()) {
                    fail(err, origin, where + ".variants." + vt.key() + " must be an object");
                    return false;
                }
                AdviceText v;
                if (!read_advice_text(*vt, false, v, where + ".variants." + vt.key(), origin, err)) return false;
                k.variants.emplace(vt.key(), std::move(v));
            }
        }
    return true;
}

} // namespace

const AdviceKindConfig* AdvisorConfig::find(pd::LimitKind kind) const {
    for (const AdviceKindConfig& k : kinds) {
        if (k.kind == kind) return &k;
    }
    return nullptr;
}

const AdviceKindConfig* AdvisorConfig::find_rule(const std::string& rule_name) const {
    for (const AdviceKindConfig& k : build_rules) {
        if (k.rule_name == rule_name) return &k;
    }
    return nullptr;
}

int AdvisorConfig::decimals_for(const std::string& quantity, int fallback) const {
    const auto it = decimals.find(quantity);
    return it == decimals.end() ? fallback : it->second;
}

namespace {
std::string lower_first(std::string s) {
    if (!s.empty() && s[0] >= 'A' && s[0] <= 'Z') s[0] = static_cast<char>(s[0] - 'A' + 'a');
    return s;
}
// "the titanium connecting rods" style name of an installed part: grade + kind ("Stock (cast iron)" -> "stock (cast iron) crankshaft").
std::string internal_part_name(const InternalPartBuild& p) { return lower_first(p.grade_label) + " " + lower_first(p.kind_label); }
} // namespace

void set_part_limits(AdviceInput& input, const CompiledSetup& compiled) {
    if (compiled.internals) {
        const EngineInternalsBuild& in = *compiled.internals;
        if (const InternalPartBuild* w = in.weakest_piston_speed()) {
            // Rod load and piston speed meet in one currency: the rpm where the part reaches its rating, shown as the
            // equivalent mean piston speed.
            input.piston_speed_limit_m_s = w->limit_rpm * 2.0 * in.stroke_m / 60.0;
            input.piston_speed_part = internal_part_name(*w);
            input.piston_speed_kind = w->kind;
            input.piston_speed_kind_label = lower_first(w->kind_label);
            input.piston_speed_option = w->option_id;
        }
        if (const InternalPartBuild* v = in.valve_springs()) {
            input.valve_float_rpm = v->limit;
            input.valve_part = internal_part_name(*v);
            input.valve_option = v->option_id;
        }
        if (in.bore) {
            input.cylinder_wall_mm = in.bore->wall_mm;
            input.cylinder_wall_safe_mm = in.bore->safe_wall_mm;
            input.bore_label = lower_first(in.bore->label);
            input.bore_option = in.bore_option_id;
        }
    }
    if (!compiled.turbo_kit) return;
    const TurboKitBuild& kit = *compiled.turbo_kit;
    input.piping_rated_bar = kit.piping.rated_bar;
    input.piping_burst_bar = kit.piping.burst_bar;
    input.piping_label = kit.piping.label + " charge piping";
    input.turbo_max_shaft_rpm = kit.sizing.max_shaft_rpm;
}

void set_thermal_limits(AdviceInput& input, const ps::vehicle::VehicleDesc& desc, const CoolingCheckConfig& check, const CompiledSetup& compiled) {
    if (compiled.cooling) {
        for (const CoolingPartBuild& p : compiled.cooling->parts) {
            if (p.kind == "radiator") {
                input.radiator_part = lower_first(p.grade_label);
                input.radiator_option = p.option_id;
            } else if (p.kind == "oil_cooler") {
                input.oil_cooler_part = lower_first(p.grade_label);
                input.oil_cooler_option = p.option_id;
            }
        }
    }
    if (!desc.powertrain.thermal.has_value()) return;
    const pd::EngineThermalConfig& thermal = *desc.powertrain.thermal;
    double lhv = 0.0, limiter_rpm = 0.0;
    for (const pd::ComponentDesc& c : desc.powertrain.components) {
        if (const auto* s = std::get_if<pd::SimulatedEngineDesc>(&c.params)) {
            if (!s->fuels.empty()) lhv = s->fuels.front().lhv_j_kg;
            limiter_rpm = s->limiter.rpm;
            break;
        }
    }
    if (!(lhv > 0.0)) return;
    input.thermal_ambient_c = check.ambient_c;
    input.thermal_air_speed_m_s = check.ram_speed_m_s;
    input.coolant_limit_c = thermal.cooling.warnings.overheat_k - pd::kCelsiusToKelvin;
    input.oil_limit_c = check.oil_limit_c;
    double hot_coolant = -1.0e9, hot_oil = -1.0e9, coolant_air = check.ram_speed_m_s, oil_air = check.ram_speed_m_s;
    // Only the points up to max_rpm_fraction_of_limiter x the limiter (a car's top gear tops it out below the limiter: the
    // hypercar's full-throttle run settles at 7384 rpm) and within check.power_band of the peak power: a car holds full power for minutes only near its power peak
    // (gearing and drag put it there at top speed); the over-rev tail of the sweep is passed through in seconds.
    double peak_power_kw = 0.0;
    const double rpm_cap = limiter_rpm > 0.0 && check.max_rpm_fraction_of_limiter > 0.0 ? check.max_rpm_fraction_of_limiter * limiter_rpm : 1.0e9;
    for (const pd::DynoPoint& p : input.points) {
        if (p.rpm <= rpm_cap) peak_power_kw = std::max(peak_power_kw, static_cast<double>(p.power_kw));
    }
    for (const pd::DynoPoint& p : input.points) {
        if (!std::isfinite(p.fuel_g_s) || !(p.fuel_g_s > 0.0)) continue;
        if (p.rpm > rpm_cap || static_cast<double>(p.power_kw) < check.power_band * peak_power_kw) continue;
        const double fuel_w = p.fuel_g_s * 1.0e-3 * lhv; // the released fuel energy per second
        const double wall_w = thermal.engine.wall_heat_fraction * fuel_w;
        const double friction_w = check.friction_power_fraction * std::max(0.0, static_cast<double>(p.power_kw)) * 1000.0;
        const double power_kw = std::max(0.0, static_cast<double>(p.power_kw));
        const double air = check.top_speed_m_s_per_cbrt_kw > 0.0 ? check.top_speed_m_s_per_cbrt_kw * std::cbrt(power_kw) : check.ram_speed_m_s;
        const SteadyThermal t = steady_thermal(thermal, p.rpm, wall_w, friction_w, check.ambient_c, air);
        const ThermalTransient tr = transient_thermal(thermal, p.rpm, wall_w, friction_w, check.ambient_c, air, input.coolant_limit_c,
                                                      input.oil_limit_c, check.transient_horizon_s, check.transient_step_s);
        // The point that reaches the limit soonest; while none does, the hottest steady state (shown only for the record).
        const auto sooner = [](double t_new, double t_old) {
            if (!std::isfinite(t_new)) return false;
            return !std::isfinite(t_old) || t_new < t_old;
        };
        if (sooner(tr.coolant_limit_s, input.coolant_limit_time_s) || (!std::isfinite(input.coolant_limit_time_s) && !std::isfinite(tr.coolant_limit_s) && t.coolant_c > hot_coolant)) {
            hot_coolant = t.coolant_c;
            input.coolant_limit_time_s = tr.coolant_limit_s;
            input.coolant_peak_rpm = p.rpm;
            coolant_air = air;
        }
        if (sooner(tr.oil_limit_s, input.oil_limit_time_s) || (!std::isfinite(input.oil_limit_time_s) && !std::isfinite(tr.oil_limit_s) && t.oil_c > hot_oil)) {
            hot_oil = t.oil_c;
            input.oil_limit_time_s = tr.oil_limit_s;
            input.oil_peak_rpm = p.rpm;
            oil_air = air;
        }
    }
    if (hot_coolant > -1.0e8) {
        input.coolant_peak_c = hot_coolant;
        input.oil_peak_c = hot_oil;
        input.coolant_air_speed_m_s = coolant_air;
        input.oil_air_speed_m_s = oil_air;
    } else {
        input.coolant_limit_c = input.oil_limit_c = std::numeric_limits<double>::quiet_NaN();
    }
}


const std::vector<std::string>& known_advice_fact_names() {
    static const std::vector<std::string> names = {
        "target", "value", "limit", "value_pct", "limit_pct", "value_kpa", "limit_kpa", "rpm", "rpm_from", "rpm_to", "range",
        "peak", "peak_rpm", "low", "low_rpm", "reach_rpm", "drop_rpm", "current", "required_min", "required", "ratio_min", "ratio", "message", "coverage", "uncovered", "choke_flow", "demand_flow", "units", "burst", "part", "part_kind", "limit_rpm", "ambient", "air_speed", "time"};
    return names;
}

std::string render_advice_template(const std::string& tpl, const std::map<std::string, std::string>& facts, std::vector<std::string>* missing) {
    std::string out;
    for (std::size_t i = 0; i < tpl.size(); ++i) {
        if (tpl[i] != '{') {
            out += tpl[i];
            continue;
        }
        const std::size_t end = tpl.find('}', i);
        if (end == std::string::npos) {
            out += tpl.substr(i);
            break;
        }
        const std::string name = tpl.substr(i + 1, end - i - 1);
        const auto it = facts.find(name);
        if (it == facts.end()) {
            out += '?';
            if (missing != nullptr) missing->push_back(name);
        } else {
            out += it->second;
        }
        i = end;
    }
    return out;
}

std::optional<AdvisorConfig> parse_advisor_config(const std::string& json_text, const std::string& origin, std::string* err) {
    const std::optional<json> parsed = detail::parse_json(json_text);
    if (!parsed || !parsed->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    const json& root = *parsed;
    std::string format;
    if (!detail::get_string(root, "format", format) || format != "rg.garage_advisor/1") {
        fail(err, origin, "\"format\" must be \"rg.garage_advisor/1\"");
        return std::nullopt;
    }
    static const std::set<std::string> allowed = {"format", "source", "debounce_ms", "sweep", "merge_gap_rpm", "decimals", "remedy", "limit_policy", "kinds", "build_rules", "loader_warnings", "part_limits", "thermal_time"};
    for (auto it = root.begin(); it != root.end(); ++it) {
        const std::string& key = it.key();
        const bool help = key.size() > 5 && key.compare(key.size() - 5, 5, "_help") == 0;
        if (!help && allowed.find(key) == allowed.end()) {
            fail(err, origin, "unknown key \"" + key + "\"");
            return std::nullopt;
        }
    }
    AdvisorConfig c;
    if (const auto policy = root.find("limit_policy"); policy != root.end()) {
        if (!policy->is_string() || policy->get<std::string>().empty()) {
            fail(err, origin, "\"limit_policy\" must be the path of a physics_sim.limit_policy/1 file");
            return std::nullopt;
        }
        c.policy_file = policy->get<std::string>();
    }
    double d = 0.0;
    if (!detail::get_number(root, "debounce_ms", d) || d < 0.0 || d > 10000.0) {
        fail(err, origin, "\"debounce_ms\" must be a number in 0..10000");
        return std::nullopt;
    }
    c.debounce_ms = static_cast<int>(d);
    if (!detail::get_number(root, "merge_gap_rpm", c.merge_gap_rpm) || !(c.merge_gap_rpm >= 0.0)) {
        fail(err, origin, "\"merge_gap_rpm\" must be a number >= 0");
        return std::nullopt;
    }
    const auto sweep = root.find("sweep");
    if (sweep == root.end() || !sweep->is_object()) {
        fail(err, origin, "\"sweep\" must be an object");
        return std::nullopt;
    }
    if (!detail::get_string(*sweep, "quality", c.sweep.quality) || (c.sweep.quality != "sketch" && c.sweep.quality != "preview" && c.sweep.quality != "full")) {
        fail(err, origin, "sweep.quality must be \"sketch\", \"preview\" or \"full\"");
        return std::nullopt;
    }
    if (!detail::get_number(*sweep, "rpm_step", c.sweep.rpm_step) || !(c.sweep.rpm_step >= 10.0) || !detail::get_number(*sweep, "rpm_min", c.sweep.rpm_min) ||
        !(c.sweep.rpm_min > 0.0) || !detail::get_number(*sweep, "limiter_margin_rpm", c.sweep.limiter_margin_rpm) || !(c.sweep.limiter_margin_rpm >= 0.0) ||
        !detail::get_bool(*sweep, "nitrous_with_kit", c.sweep.nitrous_with_kit)) {
        fail(err, origin, "sweep needs rpm_step (>= 10), rpm_min (> 0), limiter_margin_rpm (>= 0) and nitrous_with_kit (true/false)");
        return std::nullopt;
    }
    const auto decimals = root.find("decimals");
    if (decimals != root.end()) {
        if (!decimals->is_object()) {
            fail(err, origin, "\"decimals\" must be an object");
            return std::nullopt;
        }
        for (auto it = decimals->begin(); it != decimals->end(); ++it) {
            if (!it->is_number_integer() || it->get<int>() < 0 || it->get<int>() > 4) {
                fail(err, origin, "decimals." + it.key() + " must be an integer in 0..4");
                return std::nullopt;
            }
            c.decimals[it.key()] = it->get<int>();
        }
    }
    const auto remedy = root.find("remedy");
    if (remedy != root.end()) {
        const auto points = remedy->is_object() ? remedy->find("solve_points") : remedy->end();
        if (!remedy->is_object() || points == remedy->end() || !points->is_array() || points->empty()) {
            fail(err, origin, "remedy.solve_points must be a non-empty array of \"worst\" / \"last\"");
            return std::nullopt;
        }
        c.solve_points.clear();
        for (const json& p : *points) {
            if (!p.is_string() || (p.get<std::string>() != "worst" && p.get<std::string>() != "last")) {
                fail(err, origin, "remedy.solve_points must hold \"worst\" / \"last\"");
                return std::nullopt;
            }
            c.solve_points.push_back(p.get<std::string>());
        }
    }
    const auto kinds = root.find("kinds");
    if (kinds == root.end() || !kinds->is_object() || kinds->empty()) {
        fail(err, origin, "\"kinds\" must be a non-empty object");
        return std::nullopt;
    }
    for (auto it = kinds->begin(); it != kinds->end(); ++it) {
        const std::string where = "kinds." + it.key();
        const auto kind = kind_from_name(it.key());
        if (!kind || !it->is_object()) {
            fail(err, origin, where + ": not a LimitKind name or not an object");
            return std::nullopt;
        }
        AdviceKindConfig k;
        k.kind = *kind;
        if (!read_kind_body(*it, where, origin, k, err)) return std::nullopt;
        c.kinds.push_back(std::move(k));
    }
    if (const auto rules = root.find("build_rules"); rules != root.end()) {
        if (!rules->is_object()) {
            fail(err, origin, "\"build_rules\" must be an object");
            return std::nullopt;
        }
        for (auto it = rules->begin(); it != rules->end(); ++it) {
            const std::string where = "build_rules." + it.key();
            static const std::set<std::string> rule_names = {"TurboStageMapCoverage", "ChargePipingPressure", "TurboShaftSpeed", "MeanPistonSpeed", "ValveFloat", "CylinderWall", "CoolantTemperature", "OilTemperature"};
            if (rule_names.count(it.key()) == 0 || !it->is_object()) {
                fail(err, origin, where + ": not a supported rule name (TurboStageMapCoverage, ChargePipingPressure, TurboShaftSpeed, MeanPistonSpeed, ValveFloat, CylinderWall, CoolantTemperature, OilTemperature) or not an object");
                return std::nullopt;
            }
            AdviceKindConfig k;
            k.rule_name = it.key();
            if (!read_kind_body(*it, where, origin, k, err)) return std::nullopt;
            c.build_rules.push_back(std::move(k));
        }
    }
    if (const auto limits = root.find("part_limits"); limits != root.end()) {
        if (!limits->is_object()) {
            fail(err, origin, "\"part_limits\" must be an object");
            return std::nullopt;
        }
        for (auto it = limits->begin(); it != limits->end(); ++it) {
            const std::string& key = it.key();
            const bool help = key == "source" || (key.size() > 5 && key.compare(key.size() - 5, 5, "_help") == 0);
            if (!help && key != "near_fraction" && key != "max_mean_piston_speed_m_s") {
                fail(err, origin, "part_limits: unknown key \"" + key + "\"");
                return std::nullopt;
            }
        }
        if (!detail::get_number(*limits, "near_fraction", c.part_near_fraction) || !(c.part_near_fraction > 0.0 && c.part_near_fraction <= 1.0) ||
            !detail::get_number(*limits, "max_mean_piston_speed_m_s", c.max_mean_piston_speed_m_s) ||
            !(c.max_mean_piston_speed_m_s > 5.0 && c.max_mean_piston_speed_m_s < 60.0)) {
            fail(err, origin, "part_limits needs near_fraction in (0, 1] and max_mean_piston_speed_m_s in (5, 60)");
            return std::nullopt;
        }
    }
    if (const auto tt = root.find("thermal_time"); tt != root.end()) {
        if (!tt->is_object()) {
            fail(err, origin, "\"thermal_time\" must be an object");
            return std::nullopt;
        }
        for (auto it = tt->begin(); it != tt->end(); ++it) {
            const std::string& key = it.key();
            const bool help = key == "source" || (key.size() > 5 && key.compare(key.size() - 5, 5, "_help") == 0);
            if (!help && key != "critical_within_s") {
                fail(err, origin, "thermal_time: unknown key \"" + key + "\"");
                return std::nullopt;
            }
        }
        if (!detail::get_number(*tt, "critical_within_s", c.thermal_critical_within_s) ||
            !(c.thermal_critical_within_s >= 0.0 && c.thermal_critical_within_s <= 3600.0)) {
            fail(err, origin, "thermal_time needs critical_within_s in [0, 3600]");
            return std::nullopt;
        }
    }
    if (const auto notice = root.find("loader_warnings"); notice != root.end()) {
        if (!notice->is_object()) {
            fail(err, origin, "\"loader_warnings\" must be an object");
            return std::nullopt;
        }
        AdviceKindConfig k;
        k.rule_name = "LoaderNotice";
        if (!read_kind_body(*notice, "loader_warnings", origin, k, err)) return std::nullopt;
        if (const auto ign = notice->find("ignore_containing"); ign != notice->end()) {
            if (!ign->is_array()) {
                fail(err, origin, "loader_warnings.ignore_containing must be an array of text");
                return std::nullopt;
            }
            for (const json& t : *ign) {
                if (!t.is_string()) {
                    fail(err, origin, "loader_warnings.ignore_containing must hold text");
                    return std::nullopt;
                }
                c.loader_ignore.push_back(t.get<std::string>());
            }
        }
        c.loader_notice = std::move(k);
    }
    return c;
}

std::optional<AdvisorConfig> load_advisor_config(const std::string& path, std::string* err, const std::string& user_policy_path) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot read the file");
        return std::nullopt;
    }
    std::optional<AdvisorConfig> config = parse_advisor_config(*text, path, err);
    if (!config) return std::nullopt;
    if (config->policy_file.empty() && user_policy_path.empty()) return config; // physics' compiled default
    try {
        if (config->policy_file.empty()) {
            // No policy file named: the user layer alone (omitted entries keep the compiled defaults).
            config->policy = ps::io::load_limit_policy_json(user_policy_path);
        } else {
            namespace fs = std::filesystem;
            const fs::path file = fs::path(config->policy_file).is_absolute() ? fs::path(config->policy_file) : fs::path(path).parent_path() / config->policy_file;
            config->policy = ps::io::load_limit_policy_json(file.generic_string(), user_policy_path);
        }
    } catch (const std::exception& e) {
        fail(err, path, std::string("limit policy: ") + e.what());
        return std::nullopt;
    }
    return config;
}

std::string user_limit_policy_path(const std::string& user_dir) {
    if (user_dir.empty()) return {};
    std::error_code ec;
    const std::filesystem::path file = std::filesystem::path(user_dir) / "limit_policy.json";
    return std::filesystem::is_regular_file(file, ec) ? file.generic_string() : std::string();
}

// ---- advice items -----------------------------------------------------------------------------------------------------

const char* to_string(AdviceSeverity s) {
    switch (s) {
        case AdviceSeverity::Advice: return "advice";
        case AdviceSeverity::Warning: return "warning";
        case AdviceSeverity::Critical: return "critical";
    }
    return "advice";
}

AdviceSeverity advice_severity(pd::LimitSeverity s) {
    switch (s) {
        case pd::LimitSeverity::Near: return AdviceSeverity::Advice;
        case pd::LimitSeverity::Limiting: return AdviceSeverity::Warning;
        case pd::LimitSeverity::Exceeded: return AdviceSeverity::Critical;
    }
    return AdviceSeverity::Advice;
}

const char* to_string(RemedyState s) {
    switch (s) {
        case RemedyState::None: return "none";
        case RemedyState::Closed: return "closed";
        case RemedyState::Available: return "available";
        case RemedyState::Running: return "running";
        case RemedyState::Solved: return "solved";
        case RemedyState::Failed: return "failed";
    }
    return "none";
}

bool advice_kind_is_solvable(pd::LimitKind kind) {
    return kind == pd::LimitKind::WastegateSaturated || kind == pd::LimitKind::BoostAboveTarget || kind == pd::LimitKind::TurboRotorCap;
}

std::string format_rpm_ranges(const std::vector<AdviceRange>& ranges, const AdvisorConfig& config) {
    std::string out;
    const int d = dec(config, "rpm");
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        if (i > 0) out += ", ";
        const AdviceRange& r = ranges[i];
        if (fixed(r.from_rpm, d) == fixed(r.to_rpm, d)) out += fixed(r.from_rpm, d);
        else out += fixed(r.from_rpm, d) + "-" + fixed(r.to_rpm, d);
    }
    return out.empty() ? out : out + " RPM";
}

namespace {

// Deviation from the limit, the tie-break between findings of one severity.
double deviation(const pd::LimitFinding& f) { return std::abs(f.value - f.limit); }

bool worse(const pd::LimitFinding& a, const pd::LimitFinding& b) {
    if (a.severity != b.severity) return a.severity > b.severity;
    return deviation(a) > deviation(b);
}

const pd::LimitFinding& worst_of(const std::vector<pd::LimitFinding>& v) {
    const pd::LimitFinding* w = &v.front();
    for (const pd::LimitFinding& f : v) {
        if (worse(f, *w)) w = &f;
    }
    return *w;
}

// The text of a variant: fields it leaves empty fall back to the kind's base text.
AdviceText resolve_text(const AdviceKindConfig& k, const std::string& variant) {
    AdviceText t = k.text;
    const auto it = k.variants.find(variant);
    if (it == k.variants.end()) return t;
    const AdviceText& v = it->second;
    if (!v.title.empty()) t.title = v.title;
    if (!v.lines.empty()) t.lines = v.lines;
    if (!v.suggestion.empty()) t.suggestion = v.suggestion;
    if (!v.remedy.empty()) t.remedy = v.remedy;
    if (!v.remedy_failed.empty()) t.remedy_failed = v.remedy_failed;
    if (!v.option.empty()) t.option = v.option;
    if (v.max_severity) t.max_severity = v.max_severity;
    return t;
}

void add_remedy_facts(std::map<std::string, std::string>& facts, const pd::Remedy& r, const AdvisorConfig& c) {
    facts["current"] = fmt_quantity(r.current, r.unit, c);
    facts["required_min"] = fmt_quantity(r.required_min, r.unit, c);
    facts["required"] = fmt_quantity(r.required_margin, r.unit, c);
    facts["ratio_min"] = fixed(r.required_min / r.current, dec(c, "ratio"));
    facts["ratio"] = fixed(r.required_margin / r.current, dec(c, "ratio"));
}

bool remedy_has_number(const pd::Remedy& r) { return std::isfinite(r.required_min) && std::isfinite(r.required_margin) && std::isfinite(r.current) && r.current != 0.0; }

void drop_empty(std::vector<std::string>& lines) {
    lines.erase(std::remove_if(lines.begin(), lines.end(), [](const std::string& s) { return s.empty(); }), lines.end());
}

} // namespace

std::vector<AdviceItem> build_advice(const AdviceInput& input, const AdvisorConfig& config) {
    // Group per (kind, part field). What is dropped: a kind the config does not list or hides, a severity the limit
    // policy does not warn about (config.policy.warns: the generic Near / Limiting / Exceeded filter, physics'), and
    // below the game's own per-kind floor (min_severity, only where the game genuinely adds one).
    std::map<std::pair<int, std::string>, std::vector<pd::LimitFinding>> groups;
    for (const pd::LimitFinding& f : input.findings) {
        // With the engine internals known, the game's MeanPistonSpeed build rule rates the speed against the weakest
        // installed part (a titanium rod set raises it); physics' one table figure would contradict it, so it is dropped.
        if (f.kind == pd::LimitKind::MeanPistonSpeed && std::isfinite(input.piston_speed_limit_m_s)) continue;
        const AdviceKindConfig* k = config.find(f.kind);
        if (k == nullptr || k->hidden || !config.policy.warns(f.severity) || (k->min_severity && f.severity < *k->min_severity)) continue;
        groups[{static_cast<int>(f.kind), f.part.field}].push_back(f);
    }
    // Sweep extent for the variants and "reach"/"drop" rpm.
    double first_rpm = kNaN, last_rpm = kNaN;
    for (const pd::DynoPoint& p : input.points) {
        if (!std::isfinite(first_rpm) || p.rpm < first_rpm) first_rpm = p.rpm;
        if (!std::isfinite(last_rpm) || p.rpm > last_rpm) last_rpm = p.rpm;
    }
    constexpr double kEps = 1.0e-6;

    std::vector<AdviceItem> items;
    for (auto& entry : groups) {
        std::vector<pd::LimitFinding>& fs = entry.second;
        std::stable_sort(fs.begin(), fs.end(), [](const pd::LimitFinding& a, const pd::LimitFinding& b) {
            const bool an = !std::isfinite(a.rpm), bn = !std::isfinite(b.rpm);
            if (an != bn) return an;
            return a.rpm < b.rpm;
        });
        const AdviceKindConfig& kc = *config.find(fs.front().kind);
        const pd::LimitFinding worst = worst_of(fs);

        AdviceItem item;
        item.kind = fs.front().kind;
        item.kind_name = pd::to_string(item.kind);
        item.id = item.kind_name + (fs.front().part.field.empty() ? std::string() : "@" + fs.front().part.field);
        item.option_id = kc.option_id;
        item.findings = fs;
        pd::LimitSeverity top = pd::LimitSeverity::Near;
        for (const pd::LimitFinding& f : fs) top = std::max(top, f.severity);
        item.severity = advice_severity(top);

        // Merge contiguous rpm ranges.
        for (const pd::LimitFinding& f : fs) {
            if (!std::isfinite(f.rpm)) continue;
            if (!item.ranges.empty() && f.rpm - item.ranges.back().to_rpm <= config.merge_gap_rpm + kEps) item.ranges.back().to_rpm = f.rpm;
            else item.ranges.push_back({f.rpm, f.rpm});
        }
        item.rpm_from = item.ranges.empty() ? kNaN : item.ranges.front().from_rpm;
        item.rpm_to = item.ranges.empty() ? kNaN : item.ranges.back().to_rpm;

        // The variant: where in the sweep the shortfall sits, or which axis a clamp was on.
        const double a = item.rpm_from, b = item.rpm_to;
        if (item.kind == pd::LimitKind::BoostBelowTarget && item.ranges.size() == 1 && std::isfinite(first_rpm)) {
            if (a <= first_rpm + kEps && b < last_rpm - kEps) item.variant = "spool";
            else if (a > first_rpm + kEps && b >= last_rpm - kEps) item.variant = "falloff";
        } else if (item.kind == pd::LimitKind::ClutchCapacity && fs.front().part.field.rfind("/shift_clutch", 0) == 0) {
            item.variant = "shift_clutch"; // the gearbox's own clutches (no manual clutch on this car)
        } else if (item.kind == pd::LimitKind::MapGridEdge && fs.front().part.field.find("p_im") == std::string::npos) {
            item.variant = "rpm";
        }

        // The facts.
        std::map<std::string, std::string> facts;
        const bool kpa = item.kind == pd::LimitKind::MapGridEdge && item.variant.empty();
        facts["value"] = fmt_quantity(worst.value, worst.unit, config, kpa);
        facts["limit"] = fmt_quantity(worst.limit, worst.unit, config, kpa);
        facts["value_pct"] = fmt_pct(worst.value, config);
        facts["limit_pct"] = fmt_pct(worst.limit, config);
        facts["value_kpa"] = worst.unit == pd::LimitUnit::Pascal ? fmt_kpa(worst.value, config) : "?";
        facts["limit_kpa"] = worst.unit == pd::LimitUnit::Pascal ? fmt_kpa(worst.limit, config) : "?";
        facts["rpm"] = fmt_rpm(worst.rpm, config);
        facts["rpm_from"] = fmt_rpm(a, config);
        facts["rpm_to"] = fmt_rpm(b, config);
        facts["range"] = format_rpm_ranges(item.ranges, config);
        if (std::isfinite(input.target_boost_bar)) facts["target"] = fixed(input.target_boost_bar, dec(config, "bar")) + " bar";
        else if (item.kind == pd::LimitKind::BoostBelowTarget || item.kind == pd::LimitKind::BoostAboveTarget) facts["target"] = fmt_bar(worst.limit, config);
        else facts["target"] = "?";
        // The best boost the setup achieved where the target was missed (the points inside the flagged ranges).
        double peak_bar = kNaN, peak_rpm = kNaN;
        for (const pd::DynoPoint& p : input.points) {
            const bool inside = item.ranges.empty() || std::any_of(item.ranges.begin(), item.ranges.end(), [&](const AdviceRange& r) {
                                    return p.rpm >= r.from_rpm - kEps && p.rpm <= r.to_rpm + kEps;
                                });
            if (inside && (!std::isfinite(peak_bar) || p.charge_boost_bar > peak_bar)) {
                peak_bar = p.charge_boost_bar;
                peak_rpm = p.rpm;
            }
        }
        facts["peak"] = std::isfinite(peak_bar) ? fixed(peak_bar, dec(config, "bar")) + " bar" : "?";
        facts["peak_rpm"] = fmt_rpm(peak_rpm, config);
        facts["low"] = fmt_bar(worst.value, config);
        facts["low_rpm"] = fmt_rpm(worst.rpm, config);
        double reach = kNaN, drop = kNaN;
        for (const pd::DynoPoint& p : input.points) {
            if (p.rpm > b + kEps && !(std::isfinite(reach) && p.rpm >= reach)) reach = p.rpm;
            if (p.rpm < a - kEps && !(std::isfinite(drop) && p.rpm <= drop)) drop = p.rpm;
        }
        facts["reach_rpm"] = fmt_rpm(reach, config);
        facts["drop_rpm"] = fmt_rpm(drop, config);

        // The remedy: a Closed number at once (the largest need of the range), a Simulated one waits for "Find fix".
        const pd::Remedy* closed = nullptr;
        bool simulated = false;
        for (const pd::LimitFinding& f : fs) {
            if (!f.remedy) continue;
            if (f.remedy->cost == pd::RemedyCost::Closed && remedy_has_number(*f.remedy)) {
                if (closed == nullptr || f.remedy->required_margin > closed->required_margin) closed = &*f.remedy;
            } else if (f.remedy->cost == pd::RemedyCost::Simulated && !f.remedy->solved) {
                simulated = true;
            }
        }
        const AdviceText text = resolve_text(kc, item.variant);
        if (!text.option.empty()) item.option_id = text.option;
        if (closed != nullptr) {
            add_remedy_facts(facts, *closed, config);
            item.remedy_state = RemedyState::Closed;
            if (!text.remedy.empty()) item.remedy_text = render_advice_template(text.remedy, facts);
        } else if (simulated && advice_kind_is_solvable(item.kind)) {
            item.remedy_state = RemedyState::Available;
            item.can_solve = true;
        }

        // A variant may cap how loud the item gets (turbo lag at low rpm is information, not a fault).
        if (text.max_severity) item.severity = std::min(item.severity, advice_severity(*text.max_severity));
        item.title = text.title;
        for (const std::string& l : text.lines) item.lines.push_back(render_advice_template(l, facts));
        drop_empty(item.lines);
        item.suggestion = render_advice_template(text.suggestion, facts);
        items.push_back(std::move(item));
    }

    // Static build-rule issues (stage map coverage): the physics severity, the numbers of the issue.
    for (const pd::BuildIssue& issue : input.build_issues) {
        const std::string name = pd::to_string(issue.rule);
        const AdviceKindConfig* kc = config.find_rule(name);
        if (kc == nullptr || kc->hidden || !config.policy.warns(issue.severity) || (kc->min_severity && issue.severity < *kc->min_severity)) continue;
        AdviceItem item;
        item.kind_name = name;
        item.rule_name = name;
        item.id = name;
        item.option_id = kc->option_id;
        item.severity = advice_severity(issue.severity);
        item.variant = issue.severity == pd::LimitSeverity::Limiting ? "beyond_map" : "";
        std::map<std::string, std::string> facts;
        if (input.turbo_choke && input.turbo_choke->chokes && kc->variants.count("choke") != 0) {
            // The map was extended around the demand and the rest is the compressor's own limit: say that.
            item.variant = "choke";
            facts["choke_flow"] = fixed(input.turbo_choke->choke_flow_kg_s, dec(config, "flow")) + " kg/s";
            facts["demand_flow"] = fixed(input.turbo_choke->demand_flow_kg_s, dec(config, "flow")) + " kg/s";
            facts["units"] = std::to_string(input.turbo_choke->units);
        }
        facts["message"] = issue.message;
        facts["coverage"] = fixed(issue.value * 100.0, dec(config, "coverage"));
        facts["uncovered"] = fixed((1.0 - issue.value) * 100.0, dec(config, "coverage"));
        facts["coverage"] += " %";
        facts["uncovered"] += " %";
        facts["target"] = std::isfinite(input.target_boost_bar) ? fixed(input.target_boost_bar, dec(config, "bar")) + " bar" : "?";
        const AdviceText text = resolve_text(*kc, item.variant);
        if (text.max_severity) item.severity = std::min(item.severity, advice_severity(*text.max_severity));
        item.title = text.title;
        for (const std::string& l : text.lines) item.lines.push_back(render_advice_template(l, facts));
        drop_empty(item.lines);
        item.suggestion = render_advice_template(text.suggestion, facts);
        items.push_back(std::move(item));
    }
    // Part limits (damage-model data): the fitted parts' ratings against what the sweep reached. Advice only.
    {
        double peak_charge = std::isfinite(input.target_boost_bar) ? input.target_boost_bar : 0.0;
        double peak_turbo = 0.0, turbo_at = 0.0, top_rpm = 0.0;
        for (const pd::DynoPoint& p : input.points) {
            peak_charge = std::max(peak_charge, static_cast<double>(p.charge_boost_bar));
            if (p.turbo_rpm > peak_turbo) {
                peak_turbo = p.turbo_rpm;
                turbo_at = p.rpm;
            }
            top_rpm = std::max(top_rpm, static_cast<double>(p.rpm));
        }
        const auto grade = [&](double value, double rated, double burst) -> std::optional<pd::LimitSeverity> {
            if (!(rated > 0.0) || !std::isfinite(value)) return std::nullopt;
            if (std::isfinite(burst) && value >= burst) return pd::LimitSeverity::Exceeded;
            if (value > rated) return pd::LimitSeverity::Limiting;
            if (value >= config.part_near_fraction * rated) return pd::LimitSeverity::Near;
            return std::nullopt;
        };
        const auto emit = [&](const std::string& name, std::optional<pd::LimitSeverity> sev, std::map<std::string, std::string> facts,
                              double rpm, const std::string& damage_part, const std::string& option_override) {
            if (!sev) return;
            const AdviceKindConfig* kc = config.find_rule(name);
            if (kc == nullptr || kc->hidden || !config.policy.warns(*sev) || (kc->min_severity && *sev < *kc->min_severity)) return;
            AdviceItem item;
            item.kind_name = item.rule_name = item.id = name;
            item.option_id = option_override.empty() ? kc->option_id : option_override;
            item.damage_part = damage_part;
            item.severity = advice_severity(*sev);
            item.variant = *sev == pd::LimitSeverity::Exceeded ? "burst" : (*sev == pd::LimitSeverity::Near ? "near" : "");
            if (kc->variants.count(item.variant) == 0) item.variant.clear();
            if (rpm > 0.0) {
                item.rpm_from = item.rpm_to = rpm;
                facts["rpm"] = fixed(rpm, 0) + " RPM";
            }
            const AdviceText text = resolve_text(*kc, item.variant);
            if (text.max_severity) item.severity = std::min(item.severity, advice_severity(*text.max_severity));
            item.title = text.title;
            for (const std::string& l : text.lines) item.lines.push_back(render_advice_template(l, facts));
            drop_empty(item.lines);
            item.suggestion = render_advice_template(text.suggestion, facts);
            items.push_back(std::move(item));
        };
        const int bar = dec(config, "bar");
        if (std::isfinite(input.piping_rated_bar)) {
            emit("ChargePipingPressure", grade(peak_charge, input.piping_rated_bar, input.piping_burst_bar),
                 {{"value", fixed(peak_charge, bar) + " bar"}, {"limit", fixed(input.piping_rated_bar, bar) + " bar"},
                  {"burst", fixed(input.piping_burst_bar, bar) + " bar"}, {"part", input.piping_label},
                  {"target", std::isfinite(input.target_boost_bar) ? fixed(input.target_boost_bar, bar) + " bar" : "?"}},
                 0.0, "charge_piping", "");
        }
        if (std::isfinite(input.turbo_max_shaft_rpm) && peak_turbo > 0.0) {
            emit("TurboShaftSpeed", grade(peak_turbo, input.turbo_max_shaft_rpm, std::numeric_limits<double>::quiet_NaN()),
                 {{"value", fixed(peak_turbo, 0) + " rpm"}, {"limit", fixed(input.turbo_max_shaft_rpm, 0) + " rpm"}}, turbo_at,
                 "turbo_unit", "");
        }
        if (input.stroke_m > 0.0 && top_rpm > 0.0) {
            // The weakest installed part's limit (rg/engine_internals.h), else the engine-wide figure.
            const double speed = 2.0 * input.stroke_m * top_rpm / 60.0;
            const bool parts = std::isfinite(input.piston_speed_limit_m_s);
            const double limit = parts ? input.piston_speed_limit_m_s : config.max_mean_piston_speed_m_s;
            const double safe_rpm = limit * 60.0 / (2.0 * input.stroke_m);
            emit("MeanPistonSpeed", grade(speed, limit, std::numeric_limits<double>::quiet_NaN()),
                 {{"value", fixed(speed, 1) + " m/s"}, {"limit", fixed(limit, 1) + " m/s"},
                  {"part", parts ? input.piston_speed_part : std::string("engine internals")},
                  {"part_kind", parts ? input.piston_speed_kind_label : std::string("engine internals")}, {"limit_rpm", fixed(safe_rpm, 0) + " RPM"}},
                 top_rpm, parts ? input.piston_speed_kind : std::string("engine_internals"), parts ? input.piston_speed_option : std::string());
        }
        if (std::isfinite(input.valve_float_rpm) && top_rpm > 0.0) {
            emit("ValveFloat", grade(top_rpm, input.valve_float_rpm, std::numeric_limits<double>::quiet_NaN()),
                 {{"value", fixed(top_rpm, 0) + " RPM"}, {"limit", fixed(input.valve_float_rpm, 0) + " RPM"}, {"part", input.valve_part}},
                 top_rpm, "valve_springs", input.valve_option);
        }
        // Coolant and oil temperature of the cooling check (rg::transient_thermal): the time from the warm spawn state to the
        // limit at constant full power. Never = clean, later than thermal_critical_within_s = a warning, sooner = critical.
        const auto temp_grade = [&](double time_s, double limit) -> std::optional<pd::LimitSeverity> {
            if (!std::isfinite(time_s) || !std::isfinite(limit)) return std::nullopt;
            return time_s <= config.thermal_critical_within_s ? pd::LimitSeverity::Exceeded : pd::LimitSeverity::Limiting;
        };
        const auto duration = [](double s) {
            if (!std::isfinite(s)) return std::string();
            if (s < 100.0) return fixed(std::max(1.0, std::round(s / 5.0) * 5.0), 0) + " s";
            if (s < 600.0) return fixed(std::round(s / 10.0) * 10.0, 0) + " s";
            return fixed(std::round(s / 60.0), 0) + " min";
        };
        const std::string ambient = fixed(input.thermal_ambient_c, 0) + " C";
        const std::string air = fixed(input.coolant_air_speed_m_s * 3.6, 0) + " km/h";
        const std::string oil_air = fixed(input.oil_air_speed_m_s * 3.6, 0) + " km/h";
        emit("CoolantTemperature", temp_grade(input.coolant_limit_time_s, input.coolant_limit_c),
             {{"value", fixed(input.coolant_peak_c, 0) + " C"}, {"time", duration(input.coolant_limit_time_s)}, {"limit", fixed(input.coolant_limit_c, 0) + " C"},
              {"part", input.radiator_part.empty() ? std::string("radiator") : input.radiator_part}, {"ambient", ambient}, {"air_speed", air}},
             input.coolant_peak_rpm, "head_gasket", input.radiator_option);
        emit("OilTemperature", temp_grade(input.oil_limit_time_s, input.oil_limit_c),
             {{"value", fixed(input.oil_peak_c, 0) + " C"}, {"time", duration(input.oil_limit_time_s)}, {"limit", fixed(input.oil_limit_c, 0) + " C"},
              {"part", input.oil_cooler_part.empty() ? std::string("oil cooling") : input.oil_cooler_part}, {"ambient", ambient}, {"air_speed", oil_air}},
             input.oil_peak_rpm, "crankshaft", input.oil_cooler_option);
        if (std::isfinite(input.cylinder_wall_mm) && std::isfinite(input.cylinder_wall_safe_mm) &&
            input.cylinder_wall_mm < input.cylinder_wall_safe_mm) {
            // A geometry figure, not a sweep value: thinner than safe is a warning at any rpm.
            emit("CylinderWall", pd::LimitSeverity::Limiting,
                 {{"value", fixed(input.cylinder_wall_mm, 1) + " mm"}, {"limit", fixed(input.cylinder_wall_safe_mm, 1) + " mm"},
                  {"part", input.bore_label}},
                 0.0, "cylinder_block", input.bore_option);
        }
    }
    // Engine-map loader warnings, one line each, in one item.
    if (config.loader_notice && !config.loader_notice->hidden) {
        const AdviceKindConfig& kc = *config.loader_notice;
        AdviceItem item;
        item.kind_name = item.rule_name = item.id = "LoaderNotice";
        item.option_id = kc.option_id;
        item.severity = AdviceSeverity::Advice;
        for (const std::string& w : input.loader_warnings) {
            const bool ignored = std::any_of(config.loader_ignore.begin(), config.loader_ignore.end(),
                                             [&](const std::string& t) { return w.find(t) != std::string::npos; });
            if (ignored) continue;
            std::map<std::string, std::string> facts{{"message", w}};
            for (const std::string& l : kc.text.lines) item.lines.push_back(render_advice_template(l, facts));
        }
        drop_empty(item.lines);
        if (!item.lines.empty()) {
            item.title = kc.text.title;
            item.suggestion = render_advice_template(kc.text.suggestion, {});
            items.push_back(std::move(item));
        }
    }

    const auto kind_order = [&](const AdviceItem& it) {
        if (!it.rule_name.empty()) return config.kinds.size() + (it.rule_name == "LoaderNotice" ? 1u : 0u);
        for (std::size_t i = 0; i < config.kinds.size(); ++i) {
            if (config.kinds[i].kind == it.kind) return i;
        }
        return config.kinds.size();
    };
    std::stable_sort(items.begin(), items.end(), [&](const AdviceItem& x, const AdviceItem& y) {
        if (x.severity != y.severity) return x.severity > y.severity;
        const std::size_t kx = kind_order(x), ky = kind_order(y);
        if (kx != ky) return kx < ky;
        return x.rpm_from < y.rpm_from;
    });
    return items;
}

void apply_solved_remedy(AdviceItem& item, const std::vector<pd::Remedy>& solved, const AdvisorConfig& config) {
    const AdviceKindConfig* kc = config.find(item.kind);
    const AdviceText text = kc != nullptr ? resolve_text(*kc, item.variant) : AdviceText{};
    bool all_solved = !solved.empty();
    const pd::Remedy* need = nullptr;
    for (const pd::Remedy& r : solved) {
        if (!r.solved || !remedy_has_number(r)) {
            all_solved = false;
            continue;
        }
        if (need == nullptr || r.required_margin > need->required_margin) need = &r;
    }
    std::map<std::string, std::string> facts;
    if (!all_solved || need == nullptr) {
        item.remedy_state = RemedyState::Failed;
        item.remedy_text = text.remedy_failed.empty() ? std::string("No fix was found.") : render_advice_template(text.remedy_failed, facts);
        return;
    }
    add_remedy_facts(facts, *need, config);
    item.remedy_state = RemedyState::Solved;
    item.remedy_text = text.remedy.empty() ? std::string() : render_advice_template(text.remedy, facts);
}

// ---- the physics part -------------------------------------------------------------------------------------------------

std::vector<pd::LimitFinding> check_shift_clutch_limits(const std::vector<pd::DynoPoint>& points, const pd::PowertrainDesc& powertrain,
                                                         const std::string& gearbox_file, const pd::LimitPolicy* policy) {
    std::vector<pd::LimitFinding> out;
    const pd::DynoPoint* peak = nullptr;
    for (const pd::DynoPoint& p : points) {
        if (std::isfinite(p.torque_nm) && (peak == nullptr || p.torque_nm > peak->torque_nm)) peak = &p;
    }
    if (peak == nullptr || !(peak->torque_nm > 0.0)) return out;
    const pd::LimitPolicy& pol = pd::resolve_limit_policy(policy);
    for (const pd::ComponentDesc& c : powertrain.components) {
        const auto* gb = std::get_if<pd::GearboxDesc>(&c.params);
        if (gb == nullptr || !(gb->shift_clutch.capacity_nm > 0.0)) continue;
        const double capacity = gb->shift_clutch.capacity_nm;
        const double static_capacity = capacity * gb->shift_clutch.static_factor;
        pd::LimitSeverity s = pd::LimitSeverity::Near;
        if (peak->torque_nm >= static_capacity) s = pd::LimitSeverity::Limiting;
        else if (peak->torque_nm * pol.driveline_torque_margin >= static_capacity) s = pd::LimitSeverity::Near;
        else continue;
        pd::LimitFinding f;
        f.kind = pd::LimitKind::ClutchCapacity;
        f.severity = s;
        f.rpm = peak->rpm;
        f.value = peak->torque_nm;
        f.limit = static_capacity;
        f.unit = pd::LimitUnit::NewtonMetre;
        f.part = pd::PartRef{"gearbox", gearbox_file, "/shift_clutch/capacity_nm"};
        pd::Remedy r;
        r.part = f.part;
        r.current = capacity;
        r.required_min = peak->torque_nm / gb->shift_clutch.static_factor;
        r.required_margin = pol.driveline_torque_margin * peak->torque_nm / gb->shift_clutch.static_factor;
        r.unit = pd::LimitUnit::NewtonMetre;
        r.cost = pd::RemedyCost::Closed;
        r.solved = true;
        f.remedy = std::move(r);
        out.push_back(std::move(f));
    }
    return out;
}

namespace {

// The electric motors whose shaft reaches the gearbox INPUT through shafts, clutches (taken as locked) and gear pairs - the
// motors whose torque the clutch and the gearbox input carry - each with its index among ALL motors (component order) and
// its speed per unit input speed (its torque reflected to the input = motor torque x that factor). Diffs and the gearbox
// itself are not crossed.
struct FeedingMotor {
    int index = 0;
    double speed_per_input = 1.0;
};
std::vector<FeedingMotor> motors_feeding_gearbox_input(const pd::PowertrainDesc& pt, const std::string& gearbox_id) {
    // Edges between "component.port" nodes: b's speed = a's speed x factor.
    std::multimap<std::string, std::pair<std::string, double>> edges;
    const auto link = [&](const std::string& a, const std::string& b, double factor) {
        edges.emplace(a, std::make_pair(b, factor));
        edges.emplace(b, std::make_pair(a, 1.0 / factor));
    };
    for (const pd::ShaftDesc& shaft : pt.shafts)
        for (std::size_t i = 1; i < shaft.ports.size(); ++i)
            link(shaft.ports[0].component + "." + shaft.ports[0].port, shaft.ports[i].component + "." + shaft.ports[i].port, 1.0);
    for (const pd::ComponentDesc& c : pt.components) {
        if (std::holds_alternative<pd::ClutchDesc>(c.params)) link(c.id + ".in", c.id + ".out", 1.0);
        if (const auto* g = std::get_if<pd::GearPairDesc>(&c.params))
            if (g->ratio != 0.0) link(c.id + ".in", c.id + ".out", 1.0 / g->ratio); // omega_in = ratio x omega_out
    }
    std::map<std::string, double> speed{{gearbox_id + ".in", 1.0}};
    std::vector<std::string> open{gearbox_id + ".in"};
    while (!open.empty()) {
        const std::string node = open.back();
        open.pop_back();
        const auto [lo, hi] = edges.equal_range(node);
        for (auto it = lo; it != hi; ++it)
            if (speed.emplace(it->second.first, speed[node] * it->second.second).second) open.push_back(it->second.first);
    }
    std::vector<FeedingMotor> out;
    int index = 0;
    for (const pd::ComponentDesc& c : pt.components) {
        if (!std::holds_alternative<pd::ElectricMotorDesc>(c.params)) continue;
        if (const auto it = speed.find(c.id + ".shaft"); it != speed.end()) out.push_back({index, std::abs(it->second)});
        ++index;
    }
    return out;
}

// A car without a combustion engine: the clutch / gearbox-input checks (physics check_driveline_limits) against the peak
// torque the traction motors put into the gearbox input, measured by physics run_powertrain_dyno (the car's own powertrain
// at full pedal in 1st gear, the gearbox input swept up to the first such motor's top speed). The input torque is the motors'
// shaft torque reflected through any reduction to the input (its losses ignored: an upper bound), so a slipping clutch does
// not distort it. Every check that does not apply
// is named in `skipped` (no thresholds of our own: a check physics has no limit kind for is not made).
void electric_driveline_checks(const ps::vehicle::VehicleDesc& desc, const AdvisorConfig& config, ps::jobs::JobSystem* pool,
                               const std::atomic<bool>* cancel, const std::string& clutch_file, const std::string& gearbox_file,
                               AdviceSweep& out) {
    out.skipped.push_back("Engine checks (rpm limits, boost, fuel, nitrous, cooling): the car has no combustion engine");
    out.skipped.push_back("Motor and battery checks: physics has no limit kind for electric machines or packs");
    const pd::ComponentDesc* gearbox = nullptr;
    bool clutch = false;
    for (const pd::ComponentDesc& c : desc.powertrain.components) {
        if (std::holds_alternative<pd::GearboxDesc>(c.params)) gearbox = &c;
        if (std::holds_alternative<pd::ClutchDesc>(c.params)) clutch = true;
    }
    if (gearbox == nullptr) {
        out.skipped.push_back(clutch ? "Clutch check: a clutch without a gearbox has no dyno axis (run_powertrain_dyno needs a gearbox)"
                                     : "Clutch and gearbox checks: the car has neither");
        return;
    }
    const auto& box = std::get<pd::GearboxDesc>(gearbox->params);
    if (box.type != pd::GearboxType::Manual && box.type != pd::GearboxType::Sequential) {
        out.skipped.push_back("Clutch and gearbox checks: run_powertrain_dyno does not support automatic, dual-clutch or CVT gearboxes yet");
        return;
    }
    const std::vector<FeedingMotor> feeding = motors_feeding_gearbox_input(desc.powertrain, gearbox->id);
    if (feeding.empty() || box.forward_ratios.empty()) {
        out.skipped.push_back("Clutch and gearbox checks: no traction motor drives the gearbox input");
        return;
    }
    const pd::ElectricMotorDesc* axis_motor = nullptr;
    int index = 0;
    for (const pd::ComponentDesc& c : desc.powertrain.components)
        if (const auto* m = std::get_if<pd::ElectricMotorDesc>(&c.params)) {
            if (index == feeding.front().index) axis_motor = m;
            ++index;
        }
    pd::PowertrainDynoSpec spec;
    // The axis is the gearbox input itself (the dyno's EngineRpm axis: the input shaft, engine or not); a motor ahead of a
    // clutch is not on the kinematic chain the dyno probes, so the MotorRpm axis cannot be used here.
    spec.axis = pd::PowertrainDynoAxis::EngineRpm;
    spec.gear = 1;
    const double step = config.sweep.rpm_step;
    const double top_input_rpm = (axis_motor->max_rpm - config.sweep.limiter_margin_rpm) / feeding.front().speed_per_input;
    for (double rpm = step; rpm < top_input_rpm; rpm += step) spec.points.push_back(rpm);
    if (spec.points.empty()) return;
    const auto t0 = std::chrono::steady_clock::now();
    const std::vector<pd::PowertrainDynoPoint> points = pd::run_powertrain_dyno(desc, nullptr, spec, pool, cancel).points;
    pd::DynoResult input; // the gearbox input as check_driveline_limits reads it: rpm and torque per point
    for (const pd::PowertrainDynoPoint& p : points) {
        const double input_rpm = p.axis_value;
        if (!(input_rpm > 1.0)) continue;
        double torque_nm = 0.0;
        for (const FeedingMotor& m : feeding) torque_nm += p.motors.at(static_cast<std::size_t>(m.index)).torque_nm * m.speed_per_input;
        pd::DynoPoint d;
        d.rpm = input_rpm;
        d.torque_nm = torque_nm;
        d.power_kw = torque_nm * input_rpm * 3.14159265358979323846 / 30.0 / 1000.0;
        input.points.push_back(d);
    }
    out.input_torque_points = input.points;
    for (pd::LimitFinding& f : pd::check_driveline_limits(input, desc.powertrain, clutch_file, gearbox_file, &config.policy))
        out.input.findings.push_back(std::move(f));
    out.sweep_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

} // namespace

AdviceSweep run_advice_sweep(const ps::vehicle::VehicleDesc& desc, const AdvisorConfig& config, ps::jobs::JobSystem* pool,
                             const std::atomic<bool>* cancel, const std::string& clutch_file, const std::string& gearbox_file) {
    const pd::ComponentParams* engine = nullptr;
    const pd::SimulatedEngineDesc* simulated = nullptr;
    double idle = 0.0, limiter = 0.0;
    for (const pd::ComponentDesc& c : desc.powertrain.components) {
        if (const auto* s = std::get_if<pd::SimulatedEngineDesc>(&c.params)) {
            engine = &c.params;
            simulated = s;
            idle = s->idle_rpm;
            limiter = s->limiter.rpm;
            break;
        }
        if (const auto* m = std::get_if<pd::TorqueMapEngineDesc>(&c.params)) {
            if (engine == nullptr) {
                engine = &c.params;
                idle = m->idle_rpm;
                limiter = m->limiter.rpm;
            }
        }
    }
    AdviceSweep out;
    if (engine == nullptr) { // electric and fuel-cell cars: no engine sweep; the driveline checks from the powertrain dyno
        electric_driveline_checks(desc, config, pool, cancel, clutch_file, gearbox_file, out);
        return out;
    }
    out.has_engine = true;

    pd::DynoSweepSpec spec;
    const double step = config.sweep.rpm_step;
    const double end = limiter - config.sweep.limiter_margin_rpm;
    double start = std::ceil(std::max(config.sweep.rpm_min, idle) / step) * step;
    if (!(start < end)) start = std::ceil(idle / step) * step;
    for (double rpm = start; rpm < end - 1.0e-6; rpm += step) spec.rpm.push_back(rpm);
    if (end > idle) spec.rpm.push_back(end);
    if (spec.rpm.empty()) spec.rpm.push_back(std::max(idle, 1.0));
    spec.policy = &config.policy; // the findings' grades and remedy margins; also kept by remedy_request.spec
    spec.turbo_count = 1; // wastegate remedies are reported as ratios, the per-turbo count does not change them
    if (simulated != nullptr) {
        if (simulated->turbo_pair) out.input.target_boost_bar = simulated->turbo_pair->target_boost_pa / 1.0e5;
        if (config.sweep.nitrous_with_kit) spec.nitrous = pd::nitrous_from_powertrain(desc.powertrain);
        out.input.build_issues = pd::check_turbo_stage_coverage(*simulated, &config.policy); // stage seam S0
        if (simulated->cycle) out.input.stroke_m = simulated->cycle->stroke_m;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const pd::DynoResult dyno = pd::run_dyno_sweep(*engine, nullptr, spec, pool, cancel);
    out.input.findings = dyno.limits;
    // Clutch capacity and (a gearbox file with rated_input_torque_nm) the gearbox input torque, both against the sweep's peak.
    for (pd::LimitFinding& f : pd::check_driveline_limits(dyno, desc.powertrain, clutch_file, gearbox_file, &config.policy)) {
        out.input.findings.push_back(std::move(f));
    }
    for (pd::LimitFinding& f : check_shift_clutch_limits(dyno.points, desc.powertrain, gearbox_file, &config.policy)) {
        out.input.findings.push_back(std::move(f));
    }
    out.input.points = dyno.points;
    out.remedy_request = pd::RemedyRequest{*engine, nullptr, spec};
    out.sweep_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return out;
}

// ---- the background advisor -------------------------------------------------------------------------------------------

namespace {

int background_workers() {
    // Leave four logical processors for rendering, audio and other work (like GarageDyno); RG_ENGINE_MAP_WORKERS overrides.
    int workers = static_cast<int>(std::max(1u, std::thread::hardware_concurrency() > 4 ? std::thread::hardware_concurrency() - 4 : 1u));
    if (const char* value = std::getenv("RG_ENGINE_MAP_WORKERS")) {
        const std::string_view text(value);
        int configured = 0;
        const auto parsed = std::from_chars(text.data(), text.data() + text.size(), configured);
        if (parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size() && configured >= 1 && configured <= 64) workers = configured;
    }
    return workers;
}

// The part file a powertrain component of the materialised vehicle file names ("clutch", "gearbox"): absolute, "" when the
// component is inline, missing or the file cannot be read. Only fills PartRef::file of the findings.
std::string component_ref_file(const std::string& vehicle_path, const std::string& component) {
    const auto text = detail::read_text_file(vehicle_path);
    if (!text) return {};
    const std::optional<json> vehicle = detail::parse_json(*text);
    if (!vehicle || !vehicle->is_object()) return {};
    const auto powertrain = vehicle->find("powertrain");
    if (powertrain == vehicle->end() || !powertrain->is_object()) return {};
    const auto components = powertrain->find("components");
    if (components == powertrain->end() || !components->is_array()) return {};
    for (const json& c : *components) {
        std::string id, type, ref;
        detail::get_string(c, "id", id);
        detail::get_string(c, "type", type);
        if ((id != component && type != component) || !detail::get_string(c, "ref", ref) || ref.empty()) continue;
        namespace fs = std::filesystem;
        std::error_code ec;
        const fs::path joined = fs::path(ref).is_absolute() ? fs::path(ref) : fs::path(vehicle_path).parent_path() / ref;
        const fs::path canonical = fs::weakly_canonical(joined, ec);
        return (ec ? joined : canonical).generic_string();
    }
    return {};
}

pd::MapQuality quality_from(const std::string& q) {
    if (q == "full") return pd::MapQuality::Full;
    if (q == "preview") return pd::MapQuality::Preview;
    return pd::MapQuality::Sketch;
}

} // namespace

LimitAdvisor::LimitAdvisor(AdvisorConfig config) : config_(std::move(config)), thread_([this] { run(); }) {}

LimitAdvisor::~LimitAdvisor() {
    {
        std::lock_guard lock(mutex_);
        stopping_ = true;
        if (cancel_) cancel_->store(true);
    }
    wake_.notify_all();
    thread_.join();
}

void LimitAdvisor::request(AdvisorRequest request) {
    std::lock_guard lock(mutex_);
    if (requested_key_ == request.key && request.entry.id == result_.vehicle_id) return;
    if (cancel_) cancel_->store(true);
    if (result_.vehicle_id != request.entry.id) result_ = AdvisorResult{};
    requested_key_ = request.key;
    ++generation_;
    pending_solves_.clear();
    result_.vehicle_id = request.entry.id;
    result_.busy = true;
    result_.error.clear();
    result_.progress = 0;
    result_.stage = "Waiting";
    result_.solving = false;
    pending_ = std::move(request);
    wake_.notify_all();
}

void LimitAdvisor::cancel() {
    std::lock_guard lock(mutex_);
    ++generation_;
    requested_key_.clear();
    pending_.reset();
    pending_solves_.clear();
    if (cancel_) cancel_->store(true);
    result_.busy = false;
    result_.solving = false;
    result_.stage.clear();
    // A cancelled Find fix returns to its button.
    for (AdviceItem& i : result_.items) {
        if (i.remedy_state == RemedyState::Running) i.remedy_state = RemedyState::Available;
    }
    wake_.notify_all();
}

void LimitAdvisor::solve(const std::string& item_id) {
    std::lock_guard lock(mutex_);
    for (AdviceItem& i : result_.items) {
        if (i.id == item_id && i.remedy_state == RemedyState::Available && !result_.busy) {
            i.remedy_state = RemedyState::Running; // shown at once; the worker picks it up
            result_.solving = true;
            result_.solve_progress = 0;
            pending_solves_.push_back(item_id);
            wake_.notify_all();
            return;
        }
    }
}

AdvisorResult LimitAdvisor::result() const {
    std::lock_guard lock(mutex_);
    return result_;
}

void LimitAdvisor::run() {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
    ps::jobs::JobSystemOptions pool_options;
    pool_options.background_priority = true;
    ps::jobs::JobSystem pool(background_workers(), pool_options);
    std::optional<AdviceSweep> last_sweep; // the sweep behind result_.items (worker thread only)
    std::uint64_t last_revision = 0;

    for (;;) {
        AdvisorRequest request;
        std::vector<std::string> solves;
        std::shared_ptr<std::atomic<bool>> cancel;
        std::uint64_t generation = 0;
        {
            std::unique_lock lock(mutex_);
            wake_.wait(lock, [this] { return stopping_ || pending_.has_value() || !pending_solves_.empty(); });
            if (stopping_) return;
            if (pending_) {
                // Wait for slider edits to settle before building maps again.
                auto observed = generation_;
                while (wake_.wait_for(lock, std::chrono::milliseconds(config_.debounce_ms), [&] { return stopping_ || generation_ != observed; })) {
                    if (stopping_) return;
                    observed = generation_;
                }
                if (!pending_) continue;
                request = std::move(*pending_);
                pending_.reset();
                solves.clear();
            } else {
                solves = std::move(pending_solves_);
                pending_solves_.clear();
            }
            generation = generation_;
            cancel = std::make_shared<std::atomic<bool>>(false);
            cancel_ = cancel;
        }

        if (!solves.empty()) {
            // ---- Find fix ----
            for (const std::string& id : solves) {
                AdviceItem item;
                std::uint64_t revision = 0;
                {
                    std::lock_guard lock(mutex_);
                    if (generation != generation_ || !last_sweep || last_revision != result_.revision) break;
                    const auto it = std::find_if(result_.items.begin(), result_.items.end(), [&](const AdviceItem& i) { return i.id == id; });
                    if (it == result_.items.end()) continue;
                    item = *it;
                    revision = result_.revision;
                    result_.solving = true;
                    result_.stage = "Solving";
                    result_.solve_progress = 0;
                }
                // The findings to solve: the worst and/or the last one of the item (config "solve_points").
                std::vector<pd::LimitFinding> targets;
                for (const std::string& p : config_.solve_points) {
                    const pd::LimitFinding& f = p == "last" ? item.findings.back() : worst_of(item.findings);
                    const bool dup = std::any_of(targets.begin(), targets.end(), [&](const pd::LimitFinding& t) { return t.rpm == f.rpm; });
                    if (!dup) targets.push_back(f);
                }
                std::vector<pd::Remedy> solved;
                bool cancelled = false;
                const auto t0 = std::chrono::steady_clock::now();
                for (std::size_t n = 0; n < targets.size() && !cancelled; ++n) {
                    pd::RemedyOptions options;
                    options.pool = &pool;
                    options.cancel = cancel.get();
                    options.on_progress = [&](std::size_t done, std::size_t total) {
                        std::lock_guard lock(mutex_);
                        if (generation == generation_) result_.solve_progress = static_cast<int>(100 * (n * total + done) / std::max<std::size_t>(1, targets.size() * total));
                    };
                    try {
                        solved.push_back(pd::solve_remedy(last_sweep->remedy_request, targets[n], options));
                    } catch (const ps::io::GenerationCancelled&) {
                        cancelled = true;
                    } catch (const std::exception&) {
                        pd::Remedy none;
                        none.solved = false;
                        solved.push_back(none);
                    }
                }
                const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                std::lock_guard lock(mutex_);
                result_.solving = false;
                result_.stage.clear();
                if (generation != generation_ || result_.revision != revision) continue; // the setup changed: the items are gone
                const auto it = std::find_if(result_.items.begin(), result_.items.end(), [&](const AdviceItem& i) { return i.id == id; });
                if (it == result_.items.end()) continue;
                if (cancelled) {
                    it->remedy_state = RemedyState::Available;
                } else {
                    apply_solved_remedy(*it, solved, config_);
                    result_.last_solve_seconds = seconds;
                }
            }
            continue;
        }

        // ---- a new setup ----
        const auto progress = [&](int value, const char* stage) {
            std::lock_guard lock(mutex_);
            if (generation == generation_) {
                result_.progress = value;
                result_.stage = stage;
            }
        };
        AdviceSweep sweep;
        std::vector<AdviceItem> items;
        std::string error;
        double maps_seconds = 0.0;
        try {
            progress(2, "Building engine maps");
            const auto t0 = std::chrono::steady_clock::now();
            ScopedWorkDir work(request.context.work_root + "_advisor", "advice");
            const auto material = materialise_setup(request.entry, request.table, request.context, request.setup, work.path());
            if (!material.ok) throw std::runtime_error(material.error);
            ps::io::EngineMapOptions maps;
            maps.quality = quality_from(config_.sweep.quality);
            maps.cache_dir = request.context.engine_map_cache_dir;
            maps.cancel = cancel.get();
            maps.pool = &pool;
            std::vector<std::string> loader_warnings;
            maps.warnings_out = &loader_warnings;
            maps.on_progress = [&](std::size_t done, std::size_t total) {
                progress(2 + static_cast<int>(78 * done / std::max<std::size_t>(1, total)), "Building engine maps");
            };
            auto desc = rg::load_vehicle(material.vehicle_path, maps);
            std::optional<TurboChoke> turbo_choke;
            if (request.context.turbo_map_extension) {
                // The stage map grows (once, then cached) when the engine's air demand runs past it. We are on the advisor's thread.
                progress(80, "Extending the turbo map");
                const TurboExtensionResult ext = extend_turbo_maps(desc, material.vehicle_path, request.context.engine_map_cache_dir,
                                                                   *request.context.turbo_map_extension, &pool, cancel.get());
                if (cancel->load()) continue;
                if (ext.status != TurboExtensionResult::Status::NotApplicable) turbo_choke = ext.choke;
                if (ext.status == TurboExtensionResult::Status::Failed) loader_warnings.push_back("turbo map extension: " + ext.message);
            }
            maps_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            progress(82, "Sweeping the engine");
            sweep = run_advice_sweep(desc, config_, &pool, cancel.get(), component_ref_file(material.vehicle_path, "clutch"),
                                     component_ref_file(material.vehicle_path, "gearbox"));
            sweep.input.loader_warnings = std::move(loader_warnings);
            sweep.input.turbo_choke = turbo_choke;
            set_part_limits(sweep.input, material.compiled);
            if (request.context.cooling_upgrades) set_thermal_limits(sweep.input, desc, request.context.cooling_upgrades->check, material.compiled);
            items = build_advice(sweep.input, config_);
        } catch (const ps::io::GenerationCancelled&) {
            continue; // a newer request (or cancel()) took over
        } catch (const std::exception& e) {
            error = e.what();
        }
        std::lock_guard lock(mutex_);
        if (generation != generation_ || cancel->load()) continue;
        result_.busy = false;
        result_.stage.clear();
        result_.error = error;
        if (error.empty()) {
            result_.items = std::move(items);
            result_.key = request.key;
            result_.progress = 100;
            result_.have_result = true;
            result_.maps_seconds = maps_seconds;
            result_.sweep_seconds = sweep.sweep_seconds;
            result_.solving = false;
            ++result_.revision;
            last_revision = result_.revision;
            last_sweep = std::move(sweep);
        }
    }
}

} // namespace rg
