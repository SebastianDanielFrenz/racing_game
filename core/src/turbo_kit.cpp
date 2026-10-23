// rg/turbo_kit.cpp - see turbo_kit.h.
#include "rg/turbo_kit.h"

#include "json_util.h"

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/drivetrain/engine_rules.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <set>
#include <system_error>

namespace rg {

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

constexpr double kGasConstantAir = 287.05; // J/(kg K)
constexpr double kGamma = 1.4;
constexpr double kPi = 3.14159265358979323846;

bool fail(std::string* err, const std::string& path, const std::string& msg) {
    if (err != nullptr) *err = path + ": " + msg;
    return false;
}

bool read_num(const json& o, const char* key, double& out, double lo, double hi, const std::string& where, std::string* err) {
    if (!detail::get_number(o, key, out) || !std::isfinite(out) || out < lo || out > hi) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "\"%s\" must be a number in [%g, %g]", key, lo, hi);
        return fail(err, where, buf);
    }
    return true;
}

bool lower_snake(const std::string& s) {
    if (s.empty()) return false;
    for (char c : s) if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
    return true;
}

std::uint64_t fnv1a(const std::string& s) {
    std::uint64_t h = 1469598103934665603ull;
    for (unsigned char c : s) {
        h ^= c;
        h *= 1099511628211ull;
    }
    return h;
}

std::string hex16(std::uint64_t v) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(v));
    return buf;
}

double round_sig(double v, double step) { return std::round(v / step) * step; }

std::uint64_t tmp_salt() {
    return static_cast<std::uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())) ^
           static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
}

// Writes `text` to `path` unless a file with exactly that text is there (atomic: temp file + rename).
bool write_once(const std::string& path, const std::string& text, std::string* err) {
    if (const auto have = detail::read_text_file(path); have && *have == text) return true;
    const std::string tmp = path + ".tmp" + hex16(fnv1a(path + text) ^ tmp_salt());
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) return fail(err, tmp, "cannot write");
        out << text;
        out.flush();
        if (!out) return fail(err, tmp, "write failed");
    }
    std::error_code ec;
    fs::rename(fs::path(tmp), fs::path(path), ec);
    if (ec) {
        fs::remove(fs::path(tmp), ec);
        // Another thread may have written the same content meanwhile.
        if (const auto have = detail::read_text_file(path); have && *have == text) return true;
        return fail(err, path, "cannot place the file");
    }
    return true;
}

} // namespace

const TurboKitSpec* TurboKitConfig::find(const std::string& id) const {
    for (const TurboKitSpec& k : kits) if (k.id == id) return &k;
    return nullptr;
}

const ChargePipingSpec* TurboKitConfig::find_piping(const std::string& id) const {
    for (const ChargePipingSpec& p : piping) if (p.id == id) return &p;
    return nullptr;
}

double TurboKitBuild::total_mass_kg() const {
    double m = 0.0;
    for (const TurboKitPart& p : parts) m += p.mass_kg * p.count;
    return m;
}

std::optional<TurboKitConfig> load_turbo_kit_config(const std::string& path, const std::string& repo_root, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot read the file");
        return std::nullopt;
    }
    const auto root = detail::parse_json(*text);
    if (!root || !root->is_object()) {
        fail(err, path, "not a JSON object");
        return std::nullopt;
    }
    std::string format;
    if (!detail::get_string(*root, "format", format) || format != "rg.turbo_kits/1") {
        fail(err, path, "\"format\" must be \"rg.turbo_kits/1\"");
        return std::nullopt;
    }
    TurboKitConfig c;
    std::string anchor;
    if (!detail::get_string(*root, "anchor_hardware", anchor) || anchor.empty()) {
        fail(err, path, "\"anchor_hardware\" (a physics_sim.turbo/1 file) is required");
        return std::nullopt;
    }
    const fs::path ap(anchor);
    c.anchor_hardware = (ap.is_absolute() ? ap : fs::path(repo_root) / ap).lexically_normal().generic_string();
    if (!detail::read_text_file(c.anchor_hardware)) {
        fail(err, path, "anchor_hardware " + c.anchor_hardware + " cannot be read");
        return std::nullopt;
    }
    if (!read_num(*root, "anchor_exducer_mm", c.anchor_exducer_mm, 10.0, 300.0, path, err)) return std::nullopt;
    if (!detail::get_string(*root, "cache_subdir", c.cache_subdir) || c.cache_subdir.empty() ||
        c.cache_subdir.find_first_of("/\\") != std::string::npos) {
        fail(err, path, "\"cache_subdir\" must be a plain directory name");
        return std::nullopt;
    }
    const auto sizing = root->find("sizing");
    if (sizing == root->end() || !sizing->is_object()) {
        fail(err, path, "\"sizing\" object is missing");
        return std::nullopt;
    }
    const std::string sw = path + " sizing";
    if (!read_num(*sizing, "volumetric_efficiency", c.volumetric_efficiency, 0.3, 1.5, sw, err) ||
        !read_num(*sizing, "flow_margin", c.flow_margin, 1.0, 3.0, sw, err) ||
        !read_num(*sizing, "ambient_pressure_kpa", c.ambient_pressure_kpa, 50.0, 120.0, sw, err) ||
        !read_num(*sizing, "ambient_temperature_k", c.ambient_temperature_k, 230.0, 330.0, sw, err) ||
        !read_num(*sizing, "efficiency_log_slope", c.efficiency_log_slope, 0.0, 1.0, sw, err) ||
        !read_num(*sizing, "efficiency_min", c.efficiency_min, 0.4, 0.9, sw, err) ||
        !read_num(*sizing, "wastegate_to_throat", c.wastegate_to_throat, 0.5, 10.0, sw, err) ||
        !read_num(*sizing, "min_scale", c.min_scale, 0.1, 1.0, sw, err) ||
        !read_num(*sizing, "max_scale", c.max_scale, 1.0, 4.0, sw, err)) {
        return std::nullopt;
    }
    const auto conv = root->find("conversion");
    if (conv == root->end() || !conv->is_object()) {
        fail(err, path, "\"conversion\" object is missing");
        return std::nullopt;
    }
    const std::string cw = path + " conversion";
    if (!read_num(*conv, "max_compression_ratio", c.max_compression_ratio, 5.0, 14.0, cw, err) ||
        !read_num(*conv, "intercooler_effectiveness", c.intercooler_effectiveness, 0.0, 1.0, cw, err)) {
        return std::nullopt;
    }
    if (!detail::get_bool(*conv, "compressible_exhaust", c.compressible_exhaust)) {
        fail(err, cw, "\"compressible_exhaust\" must be true or false");
        return std::nullopt;
    }
    const auto bc = conv->find("boost_control");
    if (bc == conv->end() || !bc->is_object()) {
        fail(err, cw, "\"boost_control\" object is missing");
        return std::nullopt;
    }
    const std::string bw = cw + " boost_control";
    if (!read_num(*bc, "kp", c.kp, 0.0, 100.0, bw, err) || !read_num(*bc, "ki_per_s", c.ki_per_s, 0.0, 100.0, bw, err) ||
        !read_num(*bc, "kd_s", c.kd_s, 0.0, 10.0, bw, err) ||
        !read_num(*bc, "actuator_tau_s", c.actuator_tau_s, 0.0, 10.0, bw, err)) {
        return std::nullopt;
    }
    const auto kits = root->find("kits");
    if (kits == root->end() || !kits->is_array() || kits->empty()) {
        fail(err, path, "\"kits\" must be a non-empty array");
        return std::nullopt;
    }
    std::set<std::string> ids;
    for (const json& k : *kits) {
        if (!k.is_object()) {
            fail(err, path, "every kit must be an object");
            return std::nullopt;
        }
        TurboKitSpec s;
        if (!detail::get_string(k, "id", s.id) || !lower_snake(s.id) || !ids.insert(s.id).second) {
            fail(err, path, "every kit needs a unique lower_snake \"id\"");
            return std::nullopt;
        }
        const std::string kw = path + " kit \"" + s.id + "\"";
        if (!detail::get_string(k, "label", s.label) || s.label.empty()) {
            fail(err, kw, "\"label\" is required");
            return std::nullopt;
        }
        double count = 0;
        if (!read_num(k, "count", count, 1.0, 4.0, kw, err) || !read_num(k, "target_boost_bar", s.target_boost_bar, 0.1, 3.0, kw, err)) {
            return std::nullopt;
        }
        if (count != std::floor(count)) {
            fail(err, kw, "\"count\" must be a whole number");
            return std::nullopt;
        }
        s.count = static_cast<int>(count);
        if (k.contains("exducer_mm") && !read_num(k,"exducer_mm",s.exducer_mm,10,300,kw,err)) return std::nullopt;
        if (k.contains("configuration")) {
            std::string ref;
            if (!detail::get_string(k,"configuration",ref) || ref.empty() || s.exducer_mm<=0) {
                fail(err,kw,"configuration needs a path and exducer_mm");return std::nullopt;
            }
            const fs::path cp(ref);
            s.configuration=(cp.is_absolute()?cp:fs::path(repo_root)/cp).lexically_normal().generic_string();
            const auto contents=detail::read_text_file(s.configuration);
            const auto doc=contents?detail::parse_json(*contents):std::nullopt;
            std::string format,hardware;
            double units=0;
            if(!doc || !detail::get_string(*doc,"format",format) || format!="physics_sim.turbo_configuration/1" ||
               !detail::get_number(*doc,"count",units) || units!=s.count || !detail::get_string(*doc,"turbo",hardware) || hardware.empty()) {
                fail(err,kw,"configuration needs turbo_configuration/1, matching count and a turbo reference");return std::nullopt;
            }
            if(!detail::read_text_file((fs::path(s.configuration).parent_path()/hardware).lexically_normal().generic_string())) {
                fail(err,kw,"configuration turbo hardware cannot be read");return std::nullopt;
            }
        }
        c.kits.push_back(std::move(s));
    }
    const auto geo = root->find("envelope");
    const auto mass = root->find("mass");
    const auto lim = root->find("limits");
    if (geo == root->end() || !geo->is_object() || mass == root->end() || !mass->is_object() || lim == root->end() || !lim->is_object()) {
        fail(err, path, "\"envelope\", \"mass\" and \"limits\" objects are required");
        return std::nullopt;
    }
    const std::string gw = path + " envelope", mw = path + " mass", lw = path + " limits";
    if (!read_num(*geo, "unit_length_per_exducer", c.unit_length_per_exducer, 1.0, 6.0, gw, err) ||
        !read_num(*geo, "unit_length_add_mm", c.unit_length_add_mm, 0.0, 300.0, gw, err) ||
        !read_num(*geo, "unit_diameter_per_exducer", c.unit_diameter_per_exducer, 1.0, 5.0, gw, err) ||
        !read_num(*geo, "unit_gap_mm", c.unit_gap_mm, 0.0, 200.0, gw, err) ||
        !read_num(*geo, "pipe_velocity_m_s", c.pipe_velocity_m_s, 10.0, 200.0, gw, err) ||
        !read_num(*geo, "unit_side_protrusion", c.unit_side_protrusion, 0.0, 1.0, gw, err) ||
        !read_num(*mass, "unit_mass_fixed_kg", c.unit_mass_fixed_kg, 0.0, 50.0, mw, err) ||
        !read_num(*mass, "unit_mass_scaled_kg", c.unit_mass_scaled_kg, 0.0, 100.0, mw, err) ||
        !read_num(*mass, "hot_side_mass_per_unit_kg", c.hot_side_mass_per_unit_kg, 0.0, 50.0, mw, err) ||
        !read_num(*mass, "intercooler_area_m2_per_kg_s", c.intercooler_area_m2_per_kg_s, 0.01, 5.0, mw, err) ||
        !read_num(*mass, "intercooler_depth_m", c.intercooler_depth_m, 0.01, 0.3, mw, err) ||
        !read_num(*mass, "intercooler_density_kg_m3", c.intercooler_density_kg_m3, 50.0, 3000.0, mw, err) ||
        !read_num(*mass, "piping_length_base_m", c.piping_length_base_m, 0.1, 10.0, mw, err) ||
        !read_num(*mass, "piping_length_per_extra_unit_m", c.piping_length_per_extra_unit_m, 0.0, 5.0, mw, err) ||
        !read_num(*lim, "max_shaft_speed_factor", c.max_shaft_speed_factor, 1.0, 2.0, lw, err)) {
        return std::nullopt;
    }
    const auto piping = root->find("piping");
    if (piping == root->end() || !piping->is_array() || piping->empty()) {
        fail(err, path, "\"piping\" must be a non-empty array");
        return std::nullopt;
    }
    std::set<std::string> pids;
    for (const json& p : *piping) {
        ChargePipingSpec ps;
        if (!p.is_object() || !detail::get_string(p, "id", ps.id) || !lower_snake(ps.id) || !pids.insert(ps.id).second) {
            fail(err, path, "every piping entry needs a unique lower_snake \"id\"");
            return std::nullopt;
        }
        const std::string pw = path + " piping \"" + ps.id + "\"";
        if (!detail::get_string(p, "label", ps.label) || ps.label.empty()) {
            fail(err, pw, "\"label\" is required");
            return std::nullopt;
        }
        detail::get_string(p, "source", ps.source);
        if (!read_num(p, "rated_bar", ps.rated_bar, 0.1, 100.0, pw, err) || !read_num(p, "burst_bar", ps.burst_bar, 0.1, 200.0, pw, err) ||
            !read_num(p, "wall_mm", ps.wall_mm, 0.3, 20.0, pw, err) || !read_num(p, "density_kg_m3", ps.density_kg_m3, 100.0, 20000.0, pw, err)) {
            return std::nullopt;
        }
        if (!(ps.burst_bar > ps.rated_bar)) {
            fail(err, pw, "\"burst_bar\" must be above \"rated_bar\"");
            return std::nullopt;
        }
        c.piping.push_back(std::move(ps));
    }
    if (!detail::get_string(*root, "default_piping", c.default_piping) || c.find_piping(c.default_piping) == nullptr) {
        fail(err, path, "\"default_piping\" must name a piping entry");
        return std::nullopt;
    }
    return c;
}

std::string turbo_kit_refusal(const EngineFacts& f, const std::string& name) {
    const std::string who = name.empty() ? std::string("This engine") : "The " + name;
    if (!f.simulated) {
        return who + " is a torque-curve engine: its torque comes from a fixed curve with no air path for a turbo to "
                     "feed. Swap in a simulated engine to fit a turbo kit.";
    }
    if (!f.has_cycle) {
        return who + " has authored maps and no cycle model, so the boosted maps a turbo needs cannot be generated.";
    }
    if (f.lift_family) {
        return who + " has variable intake lift (Valvetronic-style); the physics model does not combine it with forced induction.";
    }
    if (f.rotary) return who + " is a rotary engine; the turbo kit rule sizes piston engines only.";
    if (!(f.displacement_l > 0.0) || !f.cycle) return who + " has no swept volume to size a turbo from.";
    return {};
}

TurboKitSizing size_turbo_kit(const TurboKitConfig& config, const TurboKitSpec& spec, const EngineFacts& facts, double limiter_rpm) {
    TurboKitSizing z;
    const std::string refusal = turbo_kit_refusal(facts, {});
    if (!refusal.empty()) {
        z.error = refusal;
        return z;
    }
    if (!(limiter_rpm > 0.0)) {
        z.error = "the engine has no limiter rpm to size the turbo at";
        return z;
    }
    const auto text = detail::read_text_file(config.anchor_hardware);
    const auto anchor = text ? detail::parse_json(*text) : std::nullopt;
    double a_flow = 0, a_rpm = 0, a_inertia = 0, a_turbine = 0, a_shaft = 0, a_ceff = 0, a_teff = 0, a_head = 0;
    if (!anchor || !detail::get_number(*anchor, "max_mass_flow_kg_s", a_flow) || !detail::get_number(*anchor, "reference_rpm", a_rpm) ||
        !detail::get_number(*anchor, "rotor_inertia_kgm2", a_inertia) || !detail::get_number(*anchor, "turbine_area_m2", a_turbine) ||
        !detail::get_number(*anchor, "shaft_loss_w_at_reference", a_shaft) ||
        !detail::get_number(*anchor, "compressor_efficiency", a_ceff) || !detail::get_number(*anchor, "turbine_efficiency", a_teff) ||
        !detail::get_number(*anchor, "reference_head_bar", a_head) || !(a_flow > 0.0)) {
        z.error = config.anchor_hardware + ": not a usable turbo hardware file";
        return z;
    }
    // Charge density at the target: isentropic compression at the anchor's efficiency, then the intercooler.
    const double p1 = config.ambient_pressure_kpa * 1000.0;
    const double t1 = config.ambient_temperature_k;
    const double pr = 1.0 + spec.target_boost_bar * 1.0e5 / p1;
    const double t2 = t1 * (1.0 + (std::pow(pr, (kGamma - 1.0) / kGamma) - 1.0) / a_ceff);
    const double tc = t2 - config.intercooler_effectiveness * (t2 - t1);
    const double rho = p1 * pr / (kGasConstantAir * tc);
    const double vd = facts.displacement_l / 1000.0;
    z.demand_kg_s = rho * vd * limiter_rpm / 120.0 * config.volumetric_efficiency;
    z.flow_per_turbo_kg_s = z.demand_kg_s / spec.count * config.flow_margin;
    z.scale = spec.exducer_mm > 0 ? spec.exducer_mm / config.anchor_exducer_mm : std::clamp(std::sqrt(z.flow_per_turbo_kg_s / a_flow), config.min_scale, config.max_scale);
    const double s = z.scale, s2 = s * s;
    z.exducer_mm = config.anchor_exducer_mm * s;
    z.reference_rpm = round_sig(a_rpm / s, 1000.0);
    z.max_mass_flow_kg_s = round_sig(a_flow * s2, 0.001);
    z.rotor_inertia_kgm2 = a_inertia * std::pow(s, 5.0);
    z.turbine_area_m2 = a_turbine * s2;
    z.wastegate_area_m2 = config.wastegate_to_throat * z.turbine_area_m2;
    z.shaft_loss_w = round_sig(a_shaft * s2, 1.0);
    z.compressor_efficiency = std::clamp(a_ceff + config.efficiency_log_slope * std::log(s), config.efficiency_min, a_ceff);
    z.turbine_efficiency = std::clamp(a_teff + config.efficiency_log_slope * std::log(s), config.efficiency_min, a_teff);
    z.reference_head_bar = a_head;
    if(!spec.configuration.empty()) {
        const auto config_text=detail::read_text_file(spec.configuration);
        const auto fixed=config_text?detail::parse_json(*config_text):std::nullopt;
        std::string ref;
        if(!fixed || !detail::get_string(*fixed,"turbo",ref)) {z.error="Cannot read fixed turbo configuration";return z;}
        const auto hardware_text=detail::read_text_file((fs::path(spec.configuration).parent_path()/ref).lexically_normal().generic_string());
        const auto hardware=hardware_text?detail::parse_json(*hardware_text):std::nullopt;
        if(!hardware) {z.error="Cannot read fixed turbo hardware";return z;}
        const auto number=[&](const char* key,double& value){
            return read_num(*hardware,key,value,0.0,1e12,spec.id,&z.error);
        };
        if(!number("reference_rpm",z.reference_rpm) || !number("max_mass_flow_kg_s",z.max_mass_flow_kg_s) ||
           !number("rotor_inertia_kgm2",z.rotor_inertia_kgm2) || !number("turbine_area_m2",z.turbine_area_m2) ||
           !number("wastegate_area_m2",z.wastegate_area_m2) || !number("shaft_loss_w_at_reference",z.shaft_loss_w) ||
           !number("compressor_efficiency",z.compressor_efficiency) || !number("turbine_efficiency",z.turbine_efficiency) ||
           !number("reference_head_bar",z.reference_head_bar)) return z;
    }
    z.compression_ratio = facts.has_turbo ? facts.compression_ratio : std::min(facts.compression_ratio, config.max_compression_ratio);
    z.torque_factor = pr;
    z.max_shaft_rpm = z.reference_rpm * config.max_shaft_speed_factor;
    z.unit_length_mm = config.unit_length_per_exducer * z.exducer_mm + config.unit_length_add_mm;
    z.unit_diameter_mm = config.unit_diameter_per_exducer * z.exducer_mm;
    // charge pipe per unit: the unit's share of the demand at charge density and pipe_velocity_m_s
    const double area = (z.demand_kg_s / spec.count) / (rho * config.pipe_velocity_m_s);
    z.pipe_diameter_mm = 1000.0 * std::sqrt(4.0 * area / kPi);
    z.unit_mass_kg = config.unit_mass_fixed_kg + config.unit_mass_scaled_kg * s * s2;
    z.hot_side_mass_kg = config.hot_side_mass_per_unit_kg * spec.count;
    z.intercooler_area_m2 = config.intercooler_area_m2_per_kg_s * z.demand_kg_s;
    z.intercooler_mass_kg = z.intercooler_area_m2 * config.intercooler_depth_m * config.intercooler_density_kg_m3;
    z.ok = true;
    return z;
}

std::array<double, 3> turbo_kit_envelope_mm(const TurboKitConfig& config, int count, const TurboKitSizing& z,
                                            const std::array<double, 3>& engine_mm, bool v_layout) {
    const int sides = v_layout && count > 1 ? 2 : 1;
    const int per_row = (count + sides - 1) / sides;
    const double row_len = per_row * z.unit_length_mm + (per_row - 1) * config.unit_gap_mm;
    std::array<double, 3> out = engine_mm;
    out[0] = std::max(engine_mm[0], row_len);
    out[1] = engine_mm[1] + sides * config.unit_side_protrusion * z.unit_diameter_mm;
    out[2] = std::max(engine_mm[2], z.unit_diameter_mm);
    return out;
}

TurboKitBuild describe_turbo_kit(const TurboKitConfig& config, const TurboKitSpec& spec, const TurboKitSizing& z,
                                 const ChargePipingSpec& piping) {
    TurboKitBuild b;
    b.spec = spec;
    b.sizing = z;
    b.piping = piping;
    b.piping_length_m = config.piping_length_base_m + config.piping_length_per_extra_unit_m * (spec.count - 1);
    char buf[300];
    TurboKitPart unit;
    unit.id = "turbo_unit";
    unit.label = "Turbocharger";
    unit.count = spec.count;
    unit.mass_kg = z.unit_mass_kg;
    std::snprintf(buf, sizeof(buf), "[AS] %.1f kg fixed (housings, bearing) + %.1f kg x s^3 (s = %.3f, wheel-diameter ratio to the anchor)",
                  config.unit_mass_fixed_kg, config.unit_mass_scaled_kg, z.scale);
    unit.mass_source = buf;
    unit.limits["max_shaft_rpm"] = z.max_shaft_rpm;
    b.parts.push_back(unit);
    TurboKitPart hot;
    hot.id = "hot_side";
    hot.label = "Exhaust manifold and downpipe";
    hot.count = spec.count;
    hot.mass_kg = config.hot_side_mass_per_unit_kg;
    hot.mass_source = "[AS] cast/tubular manifold and downpipe per turbo";
    b.parts.push_back(hot);
    TurboKitPart ic;
    ic.id = "intercooler";
    ic.label = "Intercooler";
    ic.mass_kg = z.intercooler_mass_kg;
    std::snprintf(buf, sizeof(buf), "[AS] front-mount core %.3f m2 (%.2f m2 per kg/s of %.3f kg/s) x %.0f mm x %.0f kg/m3",
                  z.intercooler_area_m2, config.intercooler_area_m2_per_kg_s, z.demand_kg_s, config.intercooler_depth_m * 1000.0,
                  config.intercooler_density_kg_m3);
    ic.mass_source = buf;
    b.parts.push_back(ic);
    TurboKitPart pipe;
    pipe.id = "charge_piping";
    pipe.label = piping.label + " charge piping";
    const double d = z.pipe_diameter_mm / 1000.0, wall = piping.wall_mm / 1000.0;
    pipe.mass_kg = kPi * (d + wall) * wall * piping.density_kg_m3 * b.piping_length_m;
    std::snprintf(buf, sizeof(buf), "[AS] %.2f m of %.0f mm pipe, %.1f mm wall at %.0f kg/m3 (%s)", b.piping_length_m, z.pipe_diameter_mm,
                  piping.wall_mm, piping.density_kg_m3, piping.id.c_str());
    pipe.mass_source = buf;
    pipe.limits["rated_bar"] = piping.rated_bar;
    pipe.limits["burst_bar"] = piping.burst_bar;
    b.parts.push_back(pipe);
    return b;
}

std::string turbo_kit_detail(const TurboKitConfig& config, const TurboKitSpec& spec, const TurboKitSizing& z) {
    (void)config;
    char buf[200];
    std::snprintf(buf, sizeof(buf), "%d x %.0f/%.0f mm compressor; %.2f kg/s per unit; %s",
                  spec.count, z.exducer_mm * 66.0/82.0, z.exducer_mm, z.max_mass_flow_kg_s,
                  spec.configuration.empty()?"sized for this engine":"fixed hardware");
    return buf;
}

std::string write_turbo_kit(const TurboKitConfig& config, const TurboKitSpec& spec, const TurboKitSizing& z,
                            const std::string& engine_name, const std::string& cache_dir, std::string* err) {
    if (!z.ok) {
        fail(err, spec.id, z.error);
        return {};
    }
    const fs::path dir = fs::path(cache_dir.empty() ? fs::temp_directory_path().generic_string() : cache_dir) / config.cache_subdir;
    std::error_code ec;
    fs::create_directories(dir, ec);
    char src[600];
    std::snprintf(src, sizeof(src),
                  "[D] Turbo kit hardware generated by rg::size_turbo_kit (racing_game core/src/turbo_kit.cpp) for %s, kit %s: "
                  "the anchor %s scaled at constant tip speed by s = %.4f (wheel-diameter ratio; flow, throat, gate and shaft "
                  "loss x s^2, reference rpm / s, inertia x s^5, efficiencies by the log rule). Engine demand %.4f kg/s at the "
                  "limiter, x %.2f margin over %d unit(s). Not a measured map.",
                  engine_name.c_str(), spec.id.c_str(), fs::path(config.anchor_hardware).filename().string().c_str(), z.scale,
                  z.demand_kg_s, config.flow_margin, spec.count);
    json hw = json::object();
    hw["format"] = "physics_sim.turbo/1";
    hw["name"] = "kit_" + spec.id;
    hw["source"] = src;
    hw["rotor_inertia_kgm2"] = z.rotor_inertia_kgm2;
    hw["reference_rpm"] = z.reference_rpm;
    hw["max_mass_flow_kg_s"] = z.max_mass_flow_kg_s;
    hw["compressor_efficiency"] = z.compressor_efficiency;
    hw["turbine_efficiency"] = z.turbine_efficiency;
    hw["turbine_area_m2"] = z.turbine_area_m2;
    hw["wastegate_area_m2"] = z.wastegate_area_m2;
    hw["shaft_loss_w_at_reference"] = z.shaft_loss_w;
    hw["reference_head_bar"] = z.reference_head_bar;
    const std::string hw_text = hw.dump(2) + "\n";
    const std::string hw_path = (dir / ("kit_turbo_" + hex16(fnv1a(hw_text)) + ".json")).generic_string();
    if (spec.configuration.empty() && !write_once(hw_path, hw_text, err)) return {};

    json cfg = json::object();
    cfg["format"] = "physics_sim.turbo_configuration/1";
    cfg["name"] = "kit_" + spec.id;
    cfg["turbo"] = hw_path;
    cfg["count"] = spec.count;
    cfg["target_boost_bar"] = spec.target_boost_bar;
    cfg["intercooler_effectiveness"] = config.intercooler_effectiveness;
    cfg["compressible_exhaust"] = config.compressible_exhaust;
    json bc = json::object();
    bc["source"] = "[A] the hyper twin's PID (gains per unit of target_boost_bar), as physics_sim's turbo templates use it";
    bc["kp"] = config.kp;
    bc["ki_per_s"] = config.ki_per_s;
    bc["kd_s"] = config.kd_s;
    bc["actuator_tau_s"] = config.actuator_tau_s;
    cfg["boost_control"] = bc;
    cfg["source"] = std::string("Garage turbo kit \"") + spec.label + "\" for " + engine_name +
                    " (data/engines/turbo_kits.json, rg::size_turbo_kit).";
    if (!spec.configuration.empty()) {
        const auto text=detail::read_text_file(spec.configuration);
        const auto original=text?detail::parse_json(*text):std::nullopt;
        if(!original || original->value("format",std::string{})!="physics_sim.turbo_configuration/1" || original->value("count",0)!=spec.count) {
            fail(err,spec.id,"invalid hardware configuration or count");return {};
        }
        cfg=*original;
        cfg["turbo"]=(fs::path(spec.configuration).parent_path()/cfg.at("turbo").get<std::string>()).lexically_normal().generic_string();
        cfg["target_boost_bar"]=spec.target_boost_bar;
    }
    const std::string cfg_text = cfg.dump(2) + "\n";
    const std::string cfg_path = (dir / ("kit_" + spec.id + "_" + hex16(fnv1a(cfg_text)) + ".json")).generic_string();
    if (!write_once(cfg_path, cfg_text, err)) return {};
    return cfg_path;
}

std::optional<std::vector<double>> turbo_kit_grid_kpa(const EngineFacts& facts, double target_boost_bar, std::string* err) {
    if (!facts.cycle) {
        if (err != nullptr) *err = "the engine has no cycle block";
        return std::nullopt;
    }
    try {
        const std::vector<ps::real> pa = ps::drivetrain::boost_grid_p_im(*facts.cycle, static_cast<ps::real>(target_boost_bar));
        std::vector<double> kpa;
        kpa.reserve(pa.size());
        for (ps::real p : pa) kpa.push_back(std::round(static_cast<double>(p) / 1000.0 * 1.0e6) / 1.0e6);
        return kpa;
    } catch (const std::exception& e) {
        if (err != nullptr) *err = e.what();
    } catch (...) {
        if (err != nullptr) *err = "boost_grid_p_im failed";
    }
    return std::nullopt;
}

PartAttachment turbo_kit_parts(const TurboKitBuild& build) {
    PartAttachment a;
    a.parent_id = "engine";
    a.node.id = "turbo_kit";
    a.node.key = "turbo_kit";
    a.node.label = build.spec.label;
    a.node.source = "turbo_kits.json kit " + build.spec.id + " (sized for this engine)";
    const std::string base = a.parent_id + "." + a.node.id;
    for (const TurboKitPart& part : build.parts) {
        for (int i = 1; i <= part.count; ++i) {
            const std::string n = part.count > 1 ? std::to_string(i) : std::string();
            PartNode leaf;
            leaf.id = base + "." + part.id + (n.empty() ? "" : "_" + n);
            leaf.key = "turbo_kit";
            leaf.label = part.label + (n.empty() ? "" : " " + n);
            leaf.own_kg = part.mass_kg;
            leaf.source = part.mass_source;
            a.node.children.push_back(std::move(leaf));
        }
    }
    return a;
}

} // namespace rg
