// rg/cooling_upgrades.h implementation.
#include "rg/cooling_upgrades.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <set>

namespace rg {

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

bool fail(std::string* err, const std::string& origin, const std::string& msg) {
    if (err != nullptr) *err = origin + ": " + msg;
    return false;
}

bool is_snake(const std::string& s) {
    return !s.empty() && std::all_of(s.begin(), s.end(), [](char c) { return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'; });
}

std::string fmt(const char* f, double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), f, v);
    return buf;
}

// The keys a grade of each kind may carry (everything else, but id/label/source/fixed_kg/per_litre_kg, is refused).
const std::map<std::string, std::set<std::string>>& allowed_keys() {
    static const std::map<std::string, std::set<std::string>> keys = {
        {"radiator", {"area_factor", "core_factor", "ram_factor", "added_kg_per_m2"}},
        {"fan", {"fan_flow_factor", "fan_power_factor", "fan_on_offset_k"}},
        {"oil_cooler", {"oil_cooler_w_k_per_m_s_per_l"}},
        {"block_cooling", {"pump_factor", "extra_coolant_l_per_l", "thermostat_offset_k"}},
    };
    return keys;
}

const std::map<std::string, std::string>& default_damage_part() {
    static const std::map<std::string, std::string> parts = {
        {"radiator", "radiator"}, {"fan", "radiator_fan"}, {"oil_cooler", "oil_cooler"}, {"block_cooling", "engine_block"}};
    return parts;
}

// Reads the grade's numbers; each key range-checked.
bool read_grade(const json& go, const std::string& kind, const std::string& where, const std::string& origin, CoolingGrade& g, std::string* err) {
    if (!go.is_object() || !detail::get_string(go, "id", g.id) || !is_snake(g.id)) return fail(err, origin, where + ": \"id\" must be a lower_snake name");
    if (!detail::get_string(go, "label", g.label) || g.label.empty() || !detail::get_string(go, "source", g.source) || g.source.empty()) {
        return fail(err, origin, where + ": \"label\" and \"source\" are required");
    }
    const auto& allowed = allowed_keys().at(kind);
    struct Field {
        const char* key;
        double* target;
        double lo, hi;
    };
    const Field fields[] = {
        {"area_factor", &g.area_factor, 1.0, 4.0},
        {"core_factor", &g.core_factor, 0.5, 3.0},
        {"ram_factor", &g.ram_factor, 0.5, 3.0},
        {"added_kg_per_m2", &g.added_kg_per_m2, 0.0, 60.0},
        {"fan_flow_factor", &g.fan_flow_factor, 1.0, 5.0},
        {"fan_power_factor", &g.fan_power_factor, 0.5, 20.0},
        {"fan_on_offset_k", &g.fan_on_offset_k, -30.0, 0.0},
        {"oil_cooler_w_k_per_m_s_per_l", &g.oil_cooler_w_k_per_m_s_per_l, 0.0, 5.0},
        {"pump_factor", &g.pump_factor, 1.0, 4.0},
        {"extra_coolant_l_per_l", &g.extra_coolant_l_per_l, 0.0, 2.0},
        {"thermostat_offset_k", &g.thermostat_offset_k, -30.0, 0.0},
    };
    for (auto it = go.begin(); it != go.end(); ++it) {
        const std::string& key = it.key();
        if (key == "id" || key == "label" || key == "source" || key == "fixed_kg" || key == "per_litre_kg") continue;
        const bool known = std::any_of(std::begin(fields), std::end(fields), [&](const Field& f) { return key == f.key; });
        if (!known) return fail(err, origin, where + ": unknown key \"" + key + "\"");
        if (allowed.count(key) == 0) return fail(err, origin, where + ": \"" + key + "\" does not belong to a " + kind + " grade");
    }
    for (const Field& f : fields) {
        if (go.find(f.key) == go.end()) continue;
        if (!detail::get_number(go, f.key, *f.target) || !(*f.target >= f.lo && *f.target <= f.hi) || !std::isfinite(*f.target)) {
            return fail(err, origin, where + ": \"" + f.key + "\" must be a number in [" + fmt("%g", f.lo) + ", " + fmt("%g", f.hi) + "]");
        }
    }
    if (go.find("fixed_kg") != go.end() && (!detail::get_number(go, "fixed_kg", g.fixed_kg) || !(g.fixed_kg >= 0.0 && g.fixed_kg <= 50.0))) {
        return fail(err, origin, where + ": \"fixed_kg\" must be in [0, 50]");
    }
    if (go.find("per_litre_kg") != go.end() && (!detail::get_number(go, "per_litre_kg", g.per_litre_kg) || !(g.per_litre_kg >= 0.0 && g.per_litre_kg <= 10.0))) {
        return fail(err, origin, where + ": \"per_litre_kg\" must be in [0, 10]");
    }
    return true;
}

double num(const json& doc, const char* a, const char* b, double fallback = 0.0) {
    const auto it = doc.find(a);
    if (it == doc.end() || !it->is_object()) return fallback;
    double v = fallback;
    detail::get_number(*it, b, v);
    return v;
}

void set_in(json& patch, const char* a, const char* b, double v) { patch[a][b] = v; }

// Displacement of the engine: the thermal file's, else the hint.
double displacement_of(const json& thermal, double hint) {
    double d = 0.0;
    detail::get_number(thermal, "displacement_l", d);
    return d > 0.0 ? d : hint;
}

// Applies one grade: fills `patch` (when non-null) and the build record. Returns false + *err when the file lacks what it needs.
bool apply_part(const CoolingPart& part, const CoolingGrade& g, const json& cool, const json& therm, double displ, json* cool_patch,
                json* therm_patch, CoolingPartBuild& out, std::string* err) {
    out.kind = part.kind;
    out.kind_label = part.label;
    out.grade = g.id;
    out.grade_label = g.label;
    out.damage_part = part.damage_part;
    out.extra_kg = g.fixed_kg + g.per_litre_kg * displ;
    if (g.id == "stock") {
        out.extra_kg = 0.0;
        return true;
    }
    if (part.kind == "radiator") {
        const double area = num(cool, "radiator", "frontal_area_m2");
        const double ua = num(cool, "radiator", "ua_ref_w_k");
        const double flow = num(cool, "radiator", "air_mass_flow_ref_kg_s");
        const double ram = num(cool, "radiator", "ram_coefficient");
        if (!(area > 0.0 && ua > 0.0 && flow > 0.0)) {
            if (err != nullptr) *err = "the cooling system file has no radiator to enlarge";
            return false;
        }
        const double new_area = area * g.area_factor;
        const double new_ram = std::min(1.0, ram * g.ram_factor);
        if (cool_patch != nullptr) {
            set_in(*cool_patch, "radiator", "frontal_area_m2", new_area);
            set_in(*cool_patch, "radiator", "ua_ref_w_k", ua * g.area_factor * g.core_factor);
            set_in(*cool_patch, "radiator", "air_mass_flow_ref_kg_s", flow * g.area_factor);
            set_in(*cool_patch, "radiator", "ram_coefficient", new_ram);
        }
        out.extra_kg += g.added_kg_per_m2 * (new_area - area);
        out.detail = fmt("%.2f", area) + " -> " + fmt("%.2f", new_area) + " m2 core · heat exchange x" + fmt("%.2f", g.area_factor * g.core_factor) +
                     (g.ram_factor != 1.0 ? " · ram air x" + fmt("%.2f", new_ram / std::max(ram, 1e-9)) : std::string());
    } else if (part.kind == "fan") {
        const double v = num(cool, "fan", "face_velocity_m_s");
        const double power = num(cool, "fan", "power_w");
        double on_c = 0.0, off_c = 0.0;
        if (const auto f = cool.find("fan"); f != cool.end() && f->is_object()) {
            detail::get_number(*f, "on_c", on_c);
            detail::get_number(*f, "off_c", off_c);
        }
        if (!(v > 0.0)) {
            if (err != nullptr) *err = "the cooling system file has no fan to replace";
            return false;
        }
        if (cool_patch != nullptr) {
            set_in(*cool_patch, "fan", "face_velocity_m_s", v * g.fan_flow_factor);
            set_in(*cool_patch, "fan", "power_w", power * g.fan_power_factor);
            set_in(*cool_patch, "fan", "on_c", on_c + g.fan_on_offset_k);
            set_in(*cool_patch, "fan", "off_c", off_c + g.fan_on_offset_k);
        }
        out.detail = fmt("%.1f", v) + " -> " + fmt("%.1f", v * g.fan_flow_factor) + " m/s through the core · on at " + fmt("%.0f", on_c + g.fan_on_offset_k) + " C";
    } else if (part.kind == "oil_cooler") {
        const double stock = num(cool, "oil_cooler", "air_w_k_per_m_s");
        const double add = g.oil_cooler_w_k_per_m_s_per_l * displ;
        if (!(displ > 0.0)) {
            if (err != nullptr) *err = "the engine's displacement is unknown, so the oil cooler cannot be sized";
            return false;
        }
        if (cool_patch != nullptr) set_in(*cool_patch, "oil_cooler", "air_w_k_per_m_s", stock + add);
        out.detail = "oil-to-air cooler: +" + fmt("%.2f", add) + " W/K per m/s of air (" + fmt("%.2f", stock) + " -> " + fmt("%.2f", stock + add) + ")";
    } else if (part.kind == "block_cooling") {
        if (!(displ > 0.0)) {
            if (err != nullptr) *err = "the engine's displacement is unknown, so the block cooling cannot be sized";
            return false;
        }
        std::string text;
        const bool per_l_pump = therm.contains("pump") && therm["pump"].contains("l_min_per_krpm_per_l");
        const char* pump_key = per_l_pump ? "l_min_per_krpm_per_l" : "l_min_per_krpm";
        const double pump = num(therm, "pump", pump_key);
        if (!(pump > 0.0)) {
            if (err != nullptr) *err = "the engine thermal file has no coolant pump";
            return false;
        }
        if (g.pump_factor != 1.0) {
            if (therm_patch != nullptr) set_in(*therm_patch, "pump", pump_key, pump * g.pump_factor);
            text += "coolant pump x" + fmt("%.2f", g.pump_factor);
        }
        if (g.extra_coolant_l_per_l > 0.0) {
            const bool per_l_cap = therm.contains("capacity_per_l");
            const char* cap_obj = per_l_cap ? "capacity_per_l" : "capacity";
            const double cap = num(therm, cap_obj, "engine_j_k");
            const double rho = num(therm, "coolant", "density_kg_m3");
            const double cp = num(therm, "coolant", "cp_j_kg_k");
            if (!(cap > 0.0 && rho > 0.0 && cp > 0.0)) {
                if (err != nullptr) *err = "the engine thermal file has no engine heat capacity";
                return false;
            }
            const double joule_per_k = g.extra_coolant_l_per_l * (per_l_cap ? 1.0 : displ) * rho * cp / 1000.0;
            if (therm_patch != nullptr) set_in(*therm_patch, cap_obj, "engine_j_k", cap + joule_per_k);
            text += std::string(text.empty() ? "" : " · ") + "+" + fmt("%.1f", g.extra_coolant_l_per_l * displ) + " L coolant in the block";
            out.extra_kg += 0.0; // the coolant's mass is in the grade's per_litre_kg
        }
        if (g.thermostat_offset_k != 0.0) {
            const double open = num(therm, "thermostat", "open_c");
            const double full = num(therm, "thermostat", "full_open_c");
            if (!(open > 0.0 && full > open)) {
                if (err != nullptr) *err = "the engine thermal file has no thermostat";
                return false;
            }
            if (therm_patch != nullptr) {
                set_in(*therm_patch, "thermostat", "open_c", open + g.thermostat_offset_k);
                set_in(*therm_patch, "thermostat", "full_open_c", full + g.thermostat_offset_k);
            }
            text += std::string(text.empty() ? "" : " · ") + "thermostat opens at " + fmt("%.0f", open + g.thermostat_offset_k) + " C";
        }
        out.detail = text;
    }
    out.detail += " · +" + fmt("%.1f", out.extra_kg) + " kg";
    return true;
}

} // namespace

const CoolingGrade* CoolingPart::find(const std::string& id) const {
    for (const CoolingGrade& g : grades) if (g.id == id) return &g;
    return nullptr;
}

const CoolingPart* CoolingUpgradeConfig::find(const std::string& kind) const {
    for (const CoolingPart& p : parts) if (p.kind == kind) return &p;
    return nullptr;
}

double CoolingBuild::mass_delta_kg() const {
    double kg = 0.0;
    for (const CoolingPartBuild& p : parts) kg += p.extra_kg;
    return kg;
}

std::optional<CoolingUpgradeConfig> parse_cooling_upgrades(const std::string& json_text, const std::string& origin, std::string* err) {
    const auto root = detail::parse_json(json_text);
    if (!root || !root->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    std::string format;
    if (!detail::get_string(*root, "format", format) || format != "rg.cooling_upgrades/1") {
        fail(err, origin, "\"format\" must be \"rg.cooling_upgrades/1\"");
        return std::nullopt;
    }
    CoolingUpgradeConfig c;
    if (const auto check = root->find("check"); check != root->end()) {
        if (!check->is_object()) {
            fail(err, origin, "\"check\" must be an object");
            return std::nullopt;
        }
        struct Field { const char* key; double* target; double lo, hi; };
        const Field fields[] = {
            {"ambient_c", &c.check.ambient_c, -30.0, 60.0},
            {"ram_speed_m_s", &c.check.ram_speed_m_s, 0.0, 150.0},
            {"top_speed_m_s_per_cbrt_kw", &c.check.top_speed_m_s_per_cbrt_kw, 0.0, 30.0},
            {"power_band", &c.check.power_band, 0.0, 1.0},
            {"max_rpm_fraction_of_limiter", &c.check.max_rpm_fraction_of_limiter, 0.0, 1.0},
            {"friction_power_fraction", &c.check.friction_power_fraction, 0.0, 0.5},
            {"transient_horizon_s", &c.check.transient_horizon_s, 10.0, 7200.0},
            {"transient_step_s", &c.check.transient_step_s, 0.01, 2.0},
            {"oil_limit_c", &c.check.oil_limit_c, 80.0, 200.0},
        };
        for (const Field& f : fields) {
            if (!detail::get_number(*check, f.key, *f.target) || !(*f.target >= f.lo && *f.target <= f.hi)) {
                fail(err, origin, std::string("check.") + f.key + " must be a number in [" + fmt("%g", f.lo) + ", " + fmt("%g", f.hi) + "]");
                return std::nullopt;
            }
        }
        if (!detail::get_string(*check, "source", c.check.source) || c.check.source.empty()) {
            fail(err, origin, "check.source is required");
            return std::nullopt;
        }
    }
    const auto parts = root->find("parts");
    if (parts == root->end() || !parts->is_array() || parts->empty()) {
        fail(err, origin, "\"parts\" must be a non-empty array");
        return std::nullopt;
    }
    std::set<std::string> kinds;
    for (size_t i = 0; i < parts->size(); ++i) {
        const json& o = (*parts)[i];
        const std::string where = "parts[" + std::to_string(i) + "]";
        CoolingPart p;
        if (!o.is_object() || !detail::get_string(o, "kind", p.kind) || allowed_keys().count(p.kind) == 0 || !kinds.insert(p.kind).second) {
            fail(err, origin, where + ": \"kind\" must be one of radiator, fan, oil_cooler, block_cooling, each once");
            return std::nullopt;
        }
        if (!detail::get_string(o, "label", p.label) || p.label.empty()) {
            fail(err, origin, where + ": \"label\" is required");
            return std::nullopt;
        }
        p.damage_part = default_damage_part().at(p.kind);
        detail::get_string(o, "damage_part", p.damage_part);
        const auto grades = o.find("grades");
        if (grades == o.end() || !grades->is_array() || grades->empty()) {
            fail(err, origin, where + ": \"grades\" must be a non-empty array");
            return std::nullopt;
        }
        std::set<std::string> ids;
        for (size_t g = 0; g < grades->size(); ++g) {
            const std::string gw = where + ".grades[" + std::to_string(g) + "]";
            CoolingGrade gr;
            if (!read_grade((*grades)[g], p.kind, gw, origin, gr, err)) return std::nullopt;
            if (!ids.insert(gr.id).second) {
                fail(err, origin, gw + ": duplicate id \"" + gr.id + "\"");
                return std::nullopt;
            }
            if ((g == 0) != (gr.id == "stock")) {
                fail(err, origin, gw + ": \"stock\" must be the first grade, and only the first");
                return std::nullopt;
            }
            if (g == 0 && (gr.fixed_kg != 0.0 || gr.per_litre_kg != 0.0)) {
                fail(err, origin, gw + ": the stock grade adds no mass");
                return std::nullopt;
            }
            if (g > 0 && !(gr.fixed_kg > 0.0 || gr.per_litre_kg > 0.0 || gr.added_kg_per_m2 > 0.0)) {
                fail(err, origin, gw + ": every upgrade part has a mass (fixed_kg, per_litre_kg or added_kg_per_m2)");
                return std::nullopt;
            }
            p.grades.push_back(std::move(gr));
        }
        c.parts.push_back(std::move(p));
    }
    return c;
}

std::optional<CoolingUpgradeConfig> load_cooling_upgrades(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot read the file");
        return std::nullopt;
    }
    return parse_cooling_upgrades(*text, path, err);
}

namespace {
bool effective_json(const fs::path& path, int depth, json& out, std::string* err) {
    if (depth > 8) return fail(err, path.generic_string(), "extends chain deeper than 8 (or a cycle)");
    const auto text = detail::read_text_file(path.string());
    if (!text) return fail(err, path.generic_string(), "cannot open");
    auto doc = detail::parse_json(*text);
    if (!doc || !doc->is_object()) return fail(err, path.generic_string(), "not a JSON object");
    std::string parent;
    if (detail::get_string(*doc, "extends", parent) && !parent.empty()) {
        json base;
        const fs::path pp = fs::path(parent).is_absolute() ? fs::path(parent) : path.parent_path() / parent;
        if (!effective_json(pp, depth + 1, base, err)) return false;
        doc->erase("extends");
        base.merge_patch(*doc);
        out = std::move(base);
        return true;
    }
    out = std::move(*doc);
    return true;
}
} // namespace

std::optional<json> load_effective_json(const std::string& path, std::string* err) {
    json out;
    if (!effective_json(fs::path(path), 0, out, err)) return std::nullopt;
    return out;
}

std::optional<CoolingBuild> build_cooling(const CoolingUpgradeConfig& config, const std::map<std::string, std::string>& grades,
                                          const std::map<std::string, std::string>& options, const json& cooling_effective,
                                          const json& thermal_effective, double displacement_hint_l, CoolingEdit* edit, std::string* err) {
    CoolingBuild build;
    build.displacement_l = displacement_of(thermal_effective, displacement_hint_l);
    for (const CoolingPart& part : config.parts) {
        const auto chosen = grades.find(part.kind);
        const std::string id = chosen == grades.end() ? std::string("stock") : chosen->second;
        const CoolingGrade* g = part.find(id);
        if (g == nullptr) {
            fail(err, part.kind, "\"" + id + "\" is not a grade of the " + part.label);
            return std::nullopt;
        }
        CoolingPartBuild b;
        std::string why;
        if (!apply_part(part, *g, cooling_effective, thermal_effective, build.displacement_l, edit != nullptr ? &edit->cooling : nullptr,
                        edit != nullptr ? &edit->thermal : nullptr, b, &why)) {
            fail(err, part.kind, "the " + part.label + " cannot be changed: " + why);
            return std::nullopt;
        }
        if (const auto o = options.find(part.kind); o != options.end()) b.option_id = o->second;
        build.parts.push_back(std::move(b));
    }
    return build;
}

std::string cooling_grade_detail(const CoolingUpgradeConfig& config, const std::string& kind, const std::string& grade, const json& cooling_effective,
                                 const json& thermal_effective, double displacement_hint_l) {
    const CoolingPart* part = config.find(kind);
    const CoolingGrade* g = part != nullptr ? part->find(grade) : nullptr;
    if (g == nullptr) return {};
    CoolingPartBuild b;
    std::string why;
    if (!apply_part(*part, *g, cooling_effective, thermal_effective, displacement_of(thermal_effective, displacement_hint_l), nullptr, nullptr, b, &why)) {
        return {};
    }
    return b.detail;
}

std::vector<PartAttachment> cooling_parts(const CoolingBuild& build) {
    std::vector<PartAttachment> out;
    for (const CoolingPartBuild& p : build.parts) {
        if (p.grade == "stock") continue;
        PartAttachment a;
        a.parent_id = "engine.cooling";
        a.node.id = p.kind;
        a.node.label = p.grade_label;
        a.node.own_kg = p.extra_kg;
        a.node.source = "cooling upgrade " + p.grade + " (mass over the stock part it replaces)";
        out.push_back(std::move(a));
    }
    return out;
}

// ---- steady state -----------------------------------------------------------------------------------------------------

namespace {
constexpr double kCpAir = 1006.0;           // J/(kg K), as engine_thermal.cpp
constexpr double kGasConstantAir = 287.05;  // J/(kg K)
constexpr double kPressurePa = 101325.0;

// Cross-flow, both fluids unmixed (Incropera & DeWitt Table 11.3), as engine_thermal.cpp radiator_effectiveness.
double effectiveness(double ntu, double cr) {
    if (!(ntu > 0.0)) return 0.0;
    if (cr < 1e-9) return 1.0 - std::exp(-ntu);
    return 1.0 - std::exp((1.0 / cr) * std::pow(ntu, 0.22) * (std::exp(-cr * std::pow(ntu, 0.78)) - 1.0));
}

// engine_thermal.cpp radiator_conductance: eps x C_min of the radiator at these mass flows.
double radiator_g(const ps::drivetrain::CoolingSystemDesc& c, const ps::drivetrain::EngineThermalDesc& e, double m_air, double m_coolant) {
    if (!(m_air > 0.0) || !(m_coolant > 0.0)) return 0.0;
    const double c_a = m_air * kCpAir;
    const double c_c = m_coolant * e.coolant_cp_j_kg_k;
    const double ua = c.radiator.ua_ref_w_k * std::pow(m_air / c.radiator.air_mass_flow_ref_kg_s, c.radiator.ua_exponent);
    const double c_min = std::min(c_a, c_c), c_max = std::max(c_a, c_c);
    return effectiveness(ua / c_min, c_min / c_max) * c_min;
}
} // namespace

SteadyThermal steady_thermal(const ps::drivetrain::EngineThermalConfig& cfg, double rpm, double wall_w, double fric_w, double ambient_c,
                             double ram_mps) {
    namespace pd = ps::drivetrain;
    const pd::EngineThermalDesc& e = cfg.engine;
    const pd::CoolingSystemDesc& c = cfg.cooling;
    const double ta = ambient_c + pd::kCelsiusToKelvin;
    const double ram = std::max(0.0, ram_mps);
    const double q_e = wall_w + (1.0 - e.friction_to_oil_fraction) * fric_w;
    const double q_o = e.friction_to_oil_fraction * fric_w;
    const double g_ba = e.block_air_w_k + e.block_air_w_k_per_m_s * ram;
    const double g_oa = e.oil_air_w_k + (e.oil_air_w_k_per_m_s + c.oil_cooler_air_w_k_per_m_s) * ram;
    const double g_oc = e.oil_capacity_j_k / e.oil_coolant_time_constant_s;
    const double rho_air = kPressurePa / (kGasConstantAir * ta);
    const double pump_l_min = e.pump_l_min_per_krpm * (std::max(0.0, rpm) / 1000.0);

    struct State {
        double te, to, g_rad, x;
    };
    const auto state_at = [&](double te, bool fan) {
        const double x = std::clamp((te - e.thermostat.open_k) / (e.thermostat.full_open_k - e.thermostat.open_k), 0.0, 1.0);
        const double v_face = std::max(c.radiator.ram_coefficient * ram, fan ? c.fan.face_velocity_m_s : 0.0);
        const double m_air = rho_air * c.radiator.frontal_area_m2 * v_face;
        const double m_coolant = x * pump_l_min / 60000.0 * e.coolant_density_kg_m3;
        const double g_rad = radiator_g(c, e, m_air, m_coolant);
        const double to = (q_o + g_oc * te + g_oa * ta) / (g_oc + g_oa);
        return State{te, to, g_rad, x};
    };
    const auto solve = [&](bool fan) {
        // net(te) = heat in - heat out is decreasing in te; bisect its zero.
        double lo = ta, hi = ta + 600.0;
        for (int i = 0; i < 80; ++i) {
            const double mid = 0.5 * (lo + hi);
            const State s = state_at(mid, fan);
            const double net = q_e + g_oc * (s.to - s.te) - (s.g_rad + g_ba) * (s.te - ta);
            (net > 0.0 ? lo : hi) = mid;
        }
        return state_at(0.5 * (lo + hi), fan);
    };
    State s = solve(false);
    bool fan_on = false;
    if (s.te >= c.fan.on_k) {
        s = solve(true);
        fan_on = true;
    }
    SteadyThermal out;
    out.coolant_c = s.te - pd::kCelsiusToKelvin;
    out.oil_c = s.to - pd::kCelsiusToKelvin;
    out.radiator_w = s.g_rad * (s.te - ta);
    out.fan_on = fan_on;
    out.thermostat_opening = s.x;
    return out;
}

ThermalTransient transient_thermal(const ps::drivetrain::EngineThermalConfig& cfg, double rpm, double wall_w, double fric_w, double ambient_c,
                                   double ram_mps, double coolant_limit_c, double oil_limit_c, double horizon_s, double step_s) {
    namespace pd = ps::drivetrain;
    const pd::EngineThermalDesc& e = cfg.engine;
    const pd::CoolingSystemDesc& c = cfg.cooling;
    ThermalTransient out;
    const double ta = ambient_c + pd::kCelsiusToKelvin;
    const double ram = std::max(0.0, ram_mps);
    const double dt = step_s;
    // Heat per step, split as engine_thermal.cpp step() does.
    const double q_e = (wall_w + (1.0 - e.friction_to_oil_fraction) * fric_w) * dt;
    const double q_o = e.friction_to_oil_fraction * fric_w * dt;
    const double g_ba = e.block_air_w_k + e.block_air_w_k_per_m_s * ram;
    const double g_oa = e.oil_air_w_k + (e.oil_air_w_k_per_m_s + c.oil_cooler_air_w_k_per_m_s) * ram;
    const double g_oc = e.oil_capacity_j_k / e.oil_coolant_time_constant_s;
    const double rho_air = kPressurePa / (kGasConstantAir * ta);
    const double pump_l_min = e.pump_l_min_per_krpm * (std::max(0.0, rpm) / 1000.0);
    const double coolant_limit_k = coolant_limit_c + pd::kCelsiusToKelvin;
    const double oil_limit_k = oil_limit_c + pd::kCelsiusToKelvin;

    // The warm spawn state (engine_thermal.cpp's constructor): coolant a quarter into the thermostat's opening band, oil at
    // its reference temperature, wax and play at the coolant, fan off.
    double te = e.thermostat.open_k + 0.25 * (e.thermostat.full_open_k - e.thermostat.open_k);
    double to = e.oil.reference_temperature_k;
    double wax = te, play = te;
    bool fan = false;
    const long steps = static_cast<long>(std::ceil(horizon_s / dt));
    int quiet = 0;
    for (long i = 1; i <= steps; ++i) {
        // 1. Thermostat (wax lag, play, opening) and 2. fan hysteresis, both from the old engine temperature.
        const double a_wax = dt / e.thermostat.time_constant_s;
        wax = (wax + a_wax * te) / (1.0 + a_wax);
        const double half = 0.5 * e.thermostat.hysteresis_k;
        play = std::clamp(play, wax - half, wax + half);
        const double x = std::clamp((play - e.thermostat.open_k) / (e.thermostat.full_open_k - e.thermostat.open_k), 0.0, 1.0);
        fan = fan ? !(te <= c.fan.off_k) : te >= c.fan.on_k;
        // 3. Conductances.
        const double v_face = std::max(c.radiator.ram_coefficient * ram, fan ? c.fan.face_velocity_m_s : 0.0);
        const double g_rad = radiator_g(c, e, rho_air * c.radiator.frontal_area_m2 * v_face, x * pump_l_min / 60000.0 * e.coolant_density_kg_m3);
        // 4. Backward Euler of the two lumps.
        const double a11 = e.engine_capacity_j_k + dt * (g_rad + g_ba + g_oc);
        const double a12 = -dt * g_oc;
        const double a22 = e.oil_capacity_j_k + dt * (g_oa + g_oc);
        const double b1 = e.engine_capacity_j_k * te + q_e + dt * (g_rad + g_ba) * ta;
        const double b2 = e.oil_capacity_j_k * to + q_o + dt * g_oa * ta;
        const double det = a11 * a22 - a12 * a12;
        const double te_new = (b1 * a22 - a12 * b2) / det;
        const double to_new = (a11 * b2 - a12 * b1) / det;
        const double moved = std::max(std::abs(te_new - te), std::abs(to_new - to));
        te = te_new;
        to = to_new;
        const double t = static_cast<double>(i) * dt;
        if (!std::isfinite(out.coolant_limit_s) && te >= coolant_limit_k) out.coolant_limit_s = t;
        if (!std::isfinite(out.oil_limit_s) && to >= oil_limit_k) out.oil_limit_s = t;
        if (std::isfinite(out.coolant_limit_s) && std::isfinite(out.oil_limit_s)) break;
        // Settled (both lumps move less than 1 mK per minute for a minute): neither limit will be reached.
        quiet = moved < 1.0e-3 * dt / 60.0 ? quiet + 1 : 0;
        if (static_cast<double>(quiet) * dt > 60.0) break;
    }
    out.end_coolant_c = te - pd::kCelsiusToKelvin;
    out.end_oil_c = to - pd::kCelsiusToKelvin;
    return out;
}

} // namespace rg
