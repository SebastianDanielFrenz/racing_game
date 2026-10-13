// rg/tyre_spec.cpp - see tyre_spec.h.
#include "rg/tyre_spec.h"

#include "json_util.h"

#include "ps/vehicle/tyre_model.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <mutex>
#include <stdexcept>

namespace rg {

namespace {

namespace fs = std::filesystem;
using detail::json;

std::string num0(double v) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.0f", v);
    return buf;
}

int round_half_up(double x) { return static_cast<int>(std::floor(x + 0.5)); }

bool all_digits(const std::string& s, std::size_t max_len = 4) {
    return !s.empty() && s.size() <= max_len && std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; });
}

// A number from an object, `fallback` when absent or not a number (the families file is validated at load, so a
// fallback only ever applies to a key the base tyre does not author).
double num(const json& g, const char* key, double fallback) {
    const auto it = g.find(key);
    return it != g.end() && it->is_number() ? it->get<double>() : fallback;
}

// Every leaf below `j` is a number (the blocks of the base tyre).
bool all_numbers(const json& j) {
    if (j.is_number()) return true;
    if (!j.is_object()) return false;
    for (auto it = j.begin(); it != j.end(); ++it) {
        if (!all_numbers(it.value())) return false;
    }
    return true;
}

} // namespace

std::string tyre_id(const TyreSpec& s) {
    return s.compound + "_" + std::to_string(s.width_mm) + "_" + std::to_string(s.aspect) + "r" + std::to_string(s.rim_in);
}

std::optional<TyreSpec> parse_tyre_id(const std::string& id) {
    // compound_width_aspectrRim; the compound is lower-case letters/digits without an underscore.
    const std::size_t a = id.find('_');
    if (a == std::string::npos || a == 0) return std::nullopt;
    const std::size_t b = id.find('_', a + 1);
    if (b == std::string::npos) return std::nullopt;
    const std::size_t r = id.find('r', b + 1);
    if (r == std::string::npos) return std::nullopt;
    TyreSpec s;
    s.compound = id.substr(0, a);
    for (char c : s.compound) {
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9'))) return std::nullopt;
    }
    const std::string width = id.substr(a + 1, b - a - 1);
    const std::string aspect = id.substr(b + 1, r - b - 1);
    const std::string rim = id.substr(r + 1);
    if (!all_digits(width) || !all_digits(aspect, 3) || !all_digits(rim, 2)) return std::nullopt;
    s.width_mm = std::stoi(width);
    s.aspect = std::stoi(aspect);
    s.rim_in = std::stoi(rim);
    if (s.width_mm <= 0 || s.aspect <= 0 || s.rim_in <= 0) return std::nullopt;
    return s;
}

std::string tyre_size_text(const TyreSpec& s) {
    return std::to_string(s.width_mm) + "/" + std::to_string(s.aspect) + " R" + std::to_string(s.rim_in);
}

double tyre_diameter_mm(const TyreSpec& s) { return s.rim_in * 25.4 + 2.0 * s.aspect / 100.0 * s.width_mm; }

int tyre_aspect_for(int rim_in, int width_mm, double diameter_mm) {
    return round_half_up((diameter_mm - rim_in * 25.4) / 2.0 / width_mm * 100.0);
}

std::string tyre_fit_error(const TyreSpec& s, const TyreAxleFitment& f, double wheel_diameter_mm) {
    if (!f.rims_in.empty() && std::find(f.rims_in.begin(), f.rims_in.end(), s.rim_in) == f.rims_in.end()) {
        std::string wheels;
        for (std::size_t i = 0; i < f.rims_in.size(); ++i) {
            wheels += (i == 0 ? "R" : (i + 1 == f.rims_in.size() ? " or R" : ", R")) + std::to_string(f.rims_in[i]);
        }
        return "rim R" + std::to_string(s.rim_in) + " does not fit this axle's " + wheels + " wheels";
    }
    if (s.width_mm < f.min_width_mm) {
        return std::to_string(s.width_mm) + " mm is narrower than this axle's " + std::to_string(f.min_width_mm) + " mm minimum";
    }
    if (s.width_mm > f.max_width_mm) {
        return std::to_string(s.width_mm) + " mm is wider than this axle's " + std::to_string(f.max_width_mm) + " mm maximum";
    }
    if (f.width_step_mm > 0 && (s.width_mm - f.min_width_mm) % f.width_step_mm != 0) {
        return std::to_string(s.width_mm) + " mm is not a standard width for this axle (steps of " + std::to_string(f.width_step_mm) +
               " mm from " + std::to_string(f.min_width_mm) + " mm)";
    }
    if (wheel_diameter_mm > 0.0) {
        const double d = tyre_diameter_mm(s);
        if (std::abs(d - wheel_diameter_mm) > f.diameter_tolerance_mm + 1e-9) {
            return "overall diameter " + num0(d) + " mm differs from this axle's " + num0(wheel_diameter_mm) + " mm by more than " +
                   num0(f.diameter_tolerance_mm) + " mm";
        }
    }
    return {};
}

std::vector<TyreSpec> tyre_sizes_for(const TyreAxleFitment& f, double wheel_diameter_mm) {
    std::vector<TyreSpec> out;
    std::vector<int> rims = f.rims_in;
    std::sort(rims.begin(), rims.end());
    const int step = std::max(1, f.width_step_mm);
    for (int rim : rims) {
        for (int w = f.min_width_mm; w <= f.max_width_mm; w += step) {
            TyreSpec s;
            s.rim_in = rim;
            s.width_mm = w;
            s.aspect = tyre_aspect_for(rim, w, wheel_diameter_mm);
            if (s.aspect < 1) continue;
            if (tyre_fit_error(s, f, wheel_diameter_mm).empty()) out.push_back(std::move(s));
        }
    }
    return out;
}

// --- families -----------------------------------------------------------------------------------------------

struct TyreFamilies::Impl {
    json base;                 // base_tyre
    double ref_width_mm = 225.0;
    double lambda_ref = 1.0;
    double crr_ref = 0.01;
    double grip_exp = 0.35;
    double rr_exp = 0.65;
    double fz0_per_m3 = 85000.0;
    double vstiff_per_m3 = 4.4e6;
    int cap_max_rim_in = 0;
    double cap_kmh = 0.0;
    std::map<int, int> legacy_diameters;
};

const TyreCompound* TyreFamilies::find(const std::string& id) const {
    for (const TyreCompound& c : compounds_) {
        if (c.id == id) return &c;
    }
    return nullptr;
}

std::shared_ptr<const TyreFamilies> TyreFamilies::load(const std::string& path, std::string* err) {
    static std::mutex mutex;
    static std::map<std::string, std::shared_ptr<const TyreFamilies>> cache;

    const auto fail = [&](const std::string& what) -> std::shared_ptr<const TyreFamilies> {
        if (err != nullptr) *err = path + ": " + what;
        return nullptr;
    };

    std::error_code ec;
    const auto time = fs::last_write_time(path, ec);
    const std::string key = fs::path(path).lexically_normal().generic_string() + "|" + (ec ? std::string("-") : std::to_string(time.time_since_epoch().count()));
    {
        const std::lock_guard<std::mutex> lock(mutex);
        const auto it = cache.find(key);
        if (it != cache.end()) return it->second;
    }

    const auto text = detail::read_text_file(path);
    if (!text) return fail("cannot read the tyre families file");
    const auto parsed = detail::parse_json(*text);
    std::string format;
    if (!parsed || !parsed->is_object() || !detail::get_string(*parsed, "format", format) || format != "racing_game.tyre_families/4") {
        return fail("not a racing_game.tyre_families/4 file");
    }

    auto impl = std::make_shared<Impl>();
    const auto rule = parsed->find("rule");
    const auto base = parsed->find("base_tyre");
    const auto families = parsed->find("families");
    if (rule == parsed->end() || !rule->is_object()) return fail("\"rule\" must be an object");
    if (base == parsed->end() || !base->is_object() || !base->contains("rolling_resistance")) return fail("\"base_tyre\" must be an object with a rolling_resistance block");
    if (families == parsed->end() || !families->is_array() || families->empty()) return fail("\"families\" must be a non-empty array");

    const auto need = [&](const char* k, double& out) { return detail::get_number(*rule, k, out); };
    if (!need("reference_width_mm", impl->ref_width_mm) || !need("lambda_mu_ref", impl->lambda_ref) || !need("crr_base_ref", impl->crr_ref) ||
        !need("grip_width_exponent", impl->grip_exp) || !need("rolling_resistance_width_exponent", impl->rr_exp) ||
        !need("fz0_n_per_m3", impl->fz0_per_m3) || !need("vertical_stiffness_n_per_m_per_m3", impl->vstiff_per_m3) || !(impl->ref_width_mm > 0.0)) {
        return fail("\"rule\" lacks one of reference_width_mm, lambda_mu_ref, crr_base_ref, grip_width_exponent, "
                    "rolling_resistance_width_exponent, fz0_n_per_m3, vertical_stiffness_n_per_m_per_m3");
    }
    const auto cap = rule->find("rated_speed_cap_kmh_up_to_rim_in");
    if (cap != rule->end()) {
        double rim = 0.0;
        if (!cap->is_object() || !detail::get_number(*cap, "max_rim_in", rim) || !detail::get_number(*cap, "cap_kmh", impl->cap_kmh)) {
            return fail("rule.rated_speed_cap_kmh_up_to_rim_in needs max_rim_in and cap_kmh");
        }
        impl->cap_max_rim_in = static_cast<int>(rim);
    }
    for (const char* block : {"longitudinal", "lateral", "combined", "aligning", "overturning"}) {
        const auto it = base->find(block);
        if (it != base->end() && !all_numbers(*it)) return fail(std::string("base_tyre.") + block + " must hold only numbers");
    }
    for (const char* k : {"v0", "qsy2", "relaxation_length_x", "relaxation_length_y", "low_speed_damping_ratio", "low_speed_damping_vx"}) {
        const auto it = base->find(k);
        if (it != base->end() && !it->is_number()) return fail(std::string("base_tyre.") + k + " must be a number");
    }
    // The built tyres go through physics_sim's validate_tyre_desc (VehicleLoadOptions::tyres): fail here, with this
    // file's path, instead of at every vehicle load. (The TyreDesc defaults 1.5 / 2.0 apply when a key is absent.)
    const double damping_ratio = num(*base, "low_speed_damping_ratio", 1.5);
    if (damping_ratio < 0.0 || damping_ratio > 2.0) return fail("base_tyre.low_speed_damping_ratio must be in [0, 2]");
    if (!(num(*base, "low_speed_damping_vx", 2.0) > 0.0)) return fail("base_tyre.low_speed_damping_vx must be > 0");
    const json& rr = base->at("rolling_resistance");
    const auto rise = rr.is_object() ? rr.find("rise_at_fraction") : rr.end();
    if (!rr.is_object() || rise == rr.end() || !rise->is_object() || !rise->contains("fraction") || !rise->contains("factor") ||
        !rr.contains("factor_at_rated") || !all_numbers(*rise) || !rr.at("factor_at_rated").is_number()) {
        return fail("base_tyre.rolling_resistance needs rise_at_fraction{fraction, factor} and factor_at_rated");
    }
    impl->base = *base;

    const auto legacy = parsed->find("legacy_reference_diameters_mm");
    if (legacy != parsed->end() && legacy->is_object()) {
        for (auto it = legacy->begin(); it != legacy->end(); ++it) {
            if (!all_digits(it.key(), 2) || !it.value().is_number()) return fail("legacy_reference_diameters_mm maps a rim (inches) to a diameter in mm");
            impl->legacy_diameters[std::stoi(it.key())] = static_cast<int>(std::lround(it.value().get<double>()));
        }
    }

    auto out = std::make_shared<TyreFamilies>();
    for (const json& f : *families) {
        TyreCompound c;
        if (!f.is_object() || !detail::get_string(f, "id", c.id) || c.id.empty() || !detail::get_string(f, "label", c.label) ||
            !detail::get_number(f, "longitudinal_grip", c.longitudinal_grip) || !detail::get_number(f, "lateral_grip", c.lateral_grip) ||
            !detail::get_number(f, "rolling_resistance", c.rolling_resistance) || !detail::get_number(f, "rated_speed_kmh", c.rated_speed_kmh) ||
            !(c.longitudinal_grip > 0.0) || !(c.lateral_grip > 0.0) || !(c.rolling_resistance > 0.0) || !(c.rated_speed_kmh > 0.0)) {
            return fail("a family lacks id/label/longitudinal_grip/lateral_grip/rolling_resistance/rated_speed_kmh (positive numbers)");
        }
        for (char ch : c.id) {
            if (!((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9'))) return fail("family id \"" + c.id + "\" must be lower-case letters/digits (it is the first part of a tyre id)");
        }
        if (out->find(c.id) != nullptr) return fail("duplicate family \"" + c.id + "\"");
        detail::get_number(f, "stiffness_scale", c.stiffness_scale);
        detail::get_string(f, "image", c.image);
        detail::get_string(f, "description", c.description);
        out->compounds_.push_back(std::move(c));
    }
    out->impl_ = impl;

    std::shared_ptr<const TyreFamilies> result = out;
    const std::lock_guard<std::mutex> lock(mutex);
    cache[key] = result;
    return result;
}

double TyreFamilies::rated_speed_kmh(const TyreSpec& spec) const {
    const TyreCompound* c = find(spec.compound);
    if (c == nullptr) throw std::invalid_argument("unknown tyre compound \"" + spec.compound + "\"");
    double rated = c->rated_speed_kmh;
    if (impl_->cap_max_rim_in > 0 && spec.rim_in <= impl_->cap_max_rim_in) rated = std::min(rated, impl_->cap_kmh);
    return rated;
}

int TyreFamilies::legacy_diameter_mm(int rim_in) const {
    const auto it = impl_->legacy_diameters.find(rim_in);
    return it == impl_->legacy_diameters.end() ? 0 : it->second;
}

ps::vehicle::TyreDesc TyreFamilies::make(const TyreSpec& spec) const {
    const TyreCompound* c = find(spec.compound);
    if (c == nullptr) throw std::invalid_argument("unknown tyre compound \"" + spec.compound + "\"");
    if (spec.width_mm <= 0 || spec.aspect <= 0 || spec.rim_in <= 0) throw std::invalid_argument("tyre " + tyre_id(spec) + ": width, aspect and rim must be positive");
    const Impl& r = *impl_;
    const json& b = r.base;

    const double width_m = spec.width_mm / 1000.0;
    const double rim_d_m = spec.rim_in * 0.0254;
    const double diameter_m = rim_d_m + 2.0 * spec.aspect / 100.0 * width_m;
    const double volume = width_m * (diameter_m * diameter_m - rim_d_m * rim_d_m);
    const double ratio = spec.width_mm / r.ref_width_mm;

    ps::vehicle::TyreDesc d;
    d.name = tyre_id(spec);
    d.source = "Authored racing_game tyre: compound " + spec.compound + ", size " + tyre_size_text(spec) +
               ". Not measured tyre data. Built in memory by one rule from data/tyres/families.json (rg::TyreFamilies::make); "
               "the rated speed is an authored intended speed regime, not a failure model.";
    d.unloaded_radius = diameter_m / 2.0;
    d.width = width_m;
    d.rim_radius = rim_d_m / 2.0;
    d.fz0 = r.fz0_per_m3 * volume;
    d.vertical_stiffness = r.vstiff_per_m3 * volume;
    d.v0 = num(b, "v0", d.v0);
    d.qsy2 = num(b, "qsy2", d.qsy2);

    const json& rr = b.at("rolling_resistance");
    ps::vehicle::RollingResistanceAnchors anchors;
    anchors.rated_speed_kmh = rated_speed_kmh(spec);
    anchors.crr_base = r.crr_ref * c->rolling_resistance * std::pow(ratio, r.rr_exp);
    anchors.factor_at_rated = rr.at("factor_at_rated").get<double>();
    anchors.fraction = rr.at("rise_at_fraction").at("fraction").get<double>();
    anchors.factor = rr.at("rise_at_fraction").at("factor").get<double>();
    const auto rolling = ps::vehicle::derive_rolling_resistance(anchors, d.v0);
    d.qsy1 = rolling.qsy1;
    d.qsy3 = rolling.qsy3;
    d.qsy4 = rolling.qsy4;
    d.rolling_resistance_anchors = anchors;

    d.relaxation_length_x = num(b, "relaxation_length_x", d.relaxation_length_x);
    d.relaxation_length_y = num(b, "relaxation_length_y", d.relaxation_length_y);
    d.lambda_mux = r.lambda_ref * c->longitudinal_grip * std::pow(ratio, r.grip_exp);
    d.lambda_muy = r.lambda_ref * c->lateral_grip * std::pow(ratio, r.grip_exp);
    d.low_speed_damping_ratio = num(b, "low_speed_damping_ratio", d.low_speed_damping_ratio);
    d.low_speed_damping_vx = num(b, "low_speed_damping_vx", d.low_speed_damping_vx);

#define RG_TYRE_FIELD(block, member) d.member = num(block, #member, d.member)
    if (const auto it = b.find("longitudinal"); it != b.end()) {
        const json& g = *it;
        RG_TYRE_FIELD(g, pcx1);
        RG_TYRE_FIELD(g, pdx1); RG_TYRE_FIELD(g, pdx2); RG_TYRE_FIELD(g, pdx3);
        RG_TYRE_FIELD(g, pex1); RG_TYRE_FIELD(g, pex2); RG_TYRE_FIELD(g, pex3); RG_TYRE_FIELD(g, pex4);
        RG_TYRE_FIELD(g, pkx1); RG_TYRE_FIELD(g, pkx2); RG_TYRE_FIELD(g, pkx3);
        RG_TYRE_FIELD(g, phx1); RG_TYRE_FIELD(g, phx2);
        RG_TYRE_FIELD(g, pvx1); RG_TYRE_FIELD(g, pvx2);
    }
    if (const auto it = b.find("lateral"); it != b.end()) {
        const json& g = *it;
        RG_TYRE_FIELD(g, pcy1);
        RG_TYRE_FIELD(g, pdy1); RG_TYRE_FIELD(g, pdy2); RG_TYRE_FIELD(g, pdy3);
        RG_TYRE_FIELD(g, pey1); RG_TYRE_FIELD(g, pey2); RG_TYRE_FIELD(g, pey3); RG_TYRE_FIELD(g, pey4);
        RG_TYRE_FIELD(g, pky1); RG_TYRE_FIELD(g, pky2);
        RG_TYRE_FIELD(g, phy1); RG_TYRE_FIELD(g, phy2); RG_TYRE_FIELD(g, phy3);
        RG_TYRE_FIELD(g, pvy1); RG_TYRE_FIELD(g, pvy2); RG_TYRE_FIELD(g, pvy3); RG_TYRE_FIELD(g, pvy4);
    }
    if (const auto it = b.find("combined"); it != b.end()) {
        const json& g = *it;
        RG_TYRE_FIELD(g, rbx1); RG_TYRE_FIELD(g, rbx2); RG_TYRE_FIELD(g, rcx1); RG_TYRE_FIELD(g, rhx1);
        RG_TYRE_FIELD(g, rby1); RG_TYRE_FIELD(g, rby2); RG_TYRE_FIELD(g, rby3); RG_TYRE_FIELD(g, rcy1);
        RG_TYRE_FIELD(g, rhy1); RG_TYRE_FIELD(g, rhy2);
        RG_TYRE_FIELD(g, rvy1); RG_TYRE_FIELD(g, rvy2); RG_TYRE_FIELD(g, rvy3);
        RG_TYRE_FIELD(g, rvy4); RG_TYRE_FIELD(g, rvy5); RG_TYRE_FIELD(g, rvy6);
    }
    if (const auto it = b.find("aligning"); it != b.end()) {
        const json& g = *it;
        RG_TYRE_FIELD(g, qbz1); RG_TYRE_FIELD(g, qbz2); RG_TYRE_FIELD(g, qbz3);
        RG_TYRE_FIELD(g, qcz1);
        RG_TYRE_FIELD(g, qdz1); RG_TYRE_FIELD(g, qdz2);
        RG_TYRE_FIELD(g, qez1); RG_TYRE_FIELD(g, qez2);
        RG_TYRE_FIELD(g, qhz1); RG_TYRE_FIELD(g, qhz2); RG_TYRE_FIELD(g, qhz3);
    }
    if (const auto it = b.find("overturning"); it != b.end()) {
        const json& g = *it;
        RG_TYRE_FIELD(g, qsx1); RG_TYRE_FIELD(g, qsx2); RG_TYRE_FIELD(g, qsx3);
    }
#undef RG_TYRE_FIELD
    // The compound's stiffness scales the two cornering/braking stiffness factors, nothing else.
    d.pkx1 *= c->stiffness_scale;
    d.pky1 *= c->stiffness_scale;
    return d;
}

// --- migration ----------------------------------------------------------------------------------------------

std::string migrate_tyre_id(const TyreFamilies& families, const std::string& name) {
    std::string stem = name;
    const std::size_t slash = stem.find_last_of("/\\");
    if (slash != std::string::npos) stem = stem.substr(slash + 1);
    if (stem.size() > 5 && stem.compare(stem.size() - 5, 5, ".json") == 0) stem.resize(stem.size() - 5);

    // Hand copies and physics_sim stock files: the size is part of the name.
    static const std::map<std::string, std::string> kExact = {
        {"hyper_front_265_35r20", "road_265_35r20"},
        {"hyper_rear_325_30r21", "road_325_30r21"},
        {"passenger_car_225_45r17", "road_225_45r17"},
        {"hyper_front_track_265_35r20", "track_265_35r20"},
        {"hyper_rear_track_325_30r21", "track_325_30r21"},
        {"passenger_sport_225_45r17", "sport_225_45r17"},
        {"passenger_comfort_225_45r17", "economy_225_45r17"},
    };
    const auto exact = kExact.find(stem);
    if (exact != kExact.end()) return families.find(parse_tyre_id(exact->second)->compound) != nullptr ? exact->second : std::string();

    // Generated stems: <prefix>_<family>_w<mm>_r<rim>. The aspect they were sized with follows from the rim.
    for (const char* prefix : {"hyper_front_", "hyper_rear_", "passenger_"}) {
        const std::string p = prefix;
        if (stem.compare(0, p.size(), p) != 0) continue;
        const std::string rest = stem.substr(p.size());
        const std::size_t w = rest.rfind("_w");
        const std::size_t r = rest.rfind("_r");
        if (w == std::string::npos || r == std::string::npos || r < w) continue;
        const std::string family = rest.substr(0, w);
        const std::string width = rest.substr(w + 2, r - (w + 2));
        const std::string rim = rest.substr(r + 2);
        if (family.empty() || !all_digits(width) || !all_digits(rim, 2)) continue;
        if (families.find(family) == nullptr) return {};
        TyreSpec s;
        s.compound = family;
        s.width_mm = std::stoi(width);
        s.rim_in = std::stoi(rim);
        const int diameter = families.legacy_diameter_mm(s.rim_in);
        if (diameter <= 0 || s.width_mm <= 0) return {};
        s.aspect = tyre_aspect_for(s.rim_in, s.width_mm, diameter);
        return s.aspect > 0 ? tyre_id(s) : std::string();
    }
    return {};
}

} // namespace rg
