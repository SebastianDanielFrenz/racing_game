// rg/engine_internals.h implementation.
#include "rg/engine_internals.h"

#include "json_util.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>
#include <set>

namespace rg {

using json = nlohmann::json;

namespace {

bool fail(std::string* err, const std::string& origin, const std::string& msg) {
    if (err != nullptr) *err = origin + ": " + msg;
    return false;
}

bool is_snake(const std::string& s) {
    if (s.empty()) return false;
    return std::all_of(s.begin(), s.end(), [](char c) { return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'; });
}

double mean_piston_speed(double stroke_m, double rpm) { return 2.0 * stroke_m * rpm / 60.0; }

// The factory reference of a limit kind on this engine: the mean piston speed at the factory limiter, or 1 (a factor).
double factory_reference(InternalLimitKind k, const EngineFacts& f) {
    return k == InternalLimitKind::MeanPistonSpeed ? mean_piston_speed(f.stroke_m, f.limiter_rpm) : 1.0;
}

// The stock rule: a stock grade rates at least stock_margin x the factory value; an upgrade never below stock. The result
// in the kind's own unit (m/s, or a factor of the factory limiter).
double effective_limit(const EngineInternalsConfig& c, const InternalPartKind& kind, const InternalGrade& grade, const EngineFacts& f) {
    const double table_stock = kind.grades.front().limit;
    const double stock = std::max(table_stock, c.stock_margin * factory_reference(kind.limit_kind, f));
    if (grade.id == "stock") return stock;
    // A downgrade (table figure below the stock table figure) keeps its own figure, so it may sit below what the factory
    // limiter needs and then warns; an upgrade never rates below this engine's stock limit.
    return grade.limit < table_stock ? grade.limit : std::max(grade.limit, stock);
}

// Peak inertial tension of one connecting rod at TDC (exhaust stroke), N: m_recip x (S/2) x w^2 x (1 + lambda).
double rod_force_n(double m_recip_kg, double stroke_m, double lambda, double rpm) {
    const double w = rpm * 2.0 * 3.14159265358979323846 / 60.0;
    return m_recip_kg * (stroke_m / 2.0) * w * w * (1.0 + lambda);
}
// The engine speed at which that tension reaches `rated_n`.
double rod_limit_rpm(double rated_n, double m_recip_kg, double stroke_m, double lambda) {
    const double w = std::sqrt(rated_n / (m_recip_kg * (stroke_m / 2.0) * (1.0 + lambda)));
    return w * 60.0 / (2.0 * 3.14159265358979323846);
}

const InternalPartKind* piston_part(const EngineInternalsConfig& c) { return c.find("pistons"); }

// The factory baseline (the engine as its file has it); hand-made facts without factory_* fall back to the current values.
double factory_displacement(const EngineFacts& f) { return f.factory_displacement_l > 0.0 ? f.factory_displacement_l : f.displacement_l; }
double factory_bore(const EngineFacts& f) { return f.factory_bore_m > 0.0 ? f.factory_bore_m : f.bore_m; }
double factory_cr(const EngineFacts& f) { return f.factory_compression_ratio > 0.0 ? f.factory_compression_ratio : f.compression_ratio; }

// Piston mass of this bore relative to the factory piston: (bore / factory bore)^(2 + thickness exponent).
double piston_scale(const EngineInternalsConfig& c, const EngineFacts& f) {
    const double fb = factory_bore(f);
    if (!(fb > 0.0) || !(f.bore_m > 0.0)) return 1.0;
    const double k = c.bore ? c.bore->piston_thickness_exponent : 1.0;
    return std::pow(f.bore_m / fb, 2.0 + k);
}

// Reciprocating mass per cylinder: the piston set (grade `piston`, at `scale` x the factory piston's mass) plus the
// small-end share of the rod set (grade `rod`). Masses from the FACTORY displacement (a bore changes only the pistons).
double recip_mass_kg(const EngineInternalsConfig& c, const EngineFacts& f, const InternalPartKind& rods, const InternalGrade& rod,
                     const InternalGrade& piston, double scale) {
    const InternalPartKind* pk = piston_part(c);
    const double n = std::max(1, f.cylinders);
    const double d = factory_displacement(f);
    const double piston_kg = (pk != nullptr ? pk->stock_mass_per_litre_kg : 0.0) * d * piston.mass_factor * scale / n;
    const double rod_kg = rods.stock_mass_per_litre_kg * d * rod.mass_factor / n;
    return piston_kg + c.rod_small_end_share * rod_kg;
}

// A rod grade's rated tension in N per rod on this engine. Stock: at least what keeps stock_margin x the factory limiter
// with the stock pistons; an upgrade never below that; a downgrade (table figure below stock's) keeps its own figure.
// Rated on the FACTORY engine (factory displacement, factory-size pistons): heavier bored pistons load the same rod more.
double rod_rating_n(const EngineInternalsConfig& c, const InternalPartKind& kind, const InternalGrade& grade, const EngineFacts& f) {
    const double cyl_l = factory_displacement(f) / std::max(1, f.cylinders);
    const InternalGrade& stock_g = kind.grades.front();
    const InternalGrade* stock_piston = piston_part(c) != nullptr ? &piston_part(c)->grades.front() : nullptr;
    double needed = 0.0;
    if (stock_piston != nullptr) {
        needed = rod_force_n(recip_mass_kg(c, f, kind, stock_g, *stock_piston, 1.0), f.stroke_m, c.rod_crank_ratio, c.stock_margin * f.limiter_rpm);
    }
    const double stock = std::max(stock_g.limit * cyl_l, needed);
    if (grade.id == "stock") return stock;
    const double own = grade.limit * cyl_l;
    return grade.limit < stock_g.limit ? own : std::max(own, stock);
}

std::string fmt(const char* f, double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), f, v);
    return buf;
}

std::string thousands(double v) {
    const long long n = std::llround(v);
    std::string s = std::to_string(n < 0 ? -n : n);
    for (int i = static_cast<int>(s.size()) - 3; i > 0; i -= 3) s.insert(static_cast<size_t>(i), ",");
    return (n < 0 ? "-" : "") + s;
}

} // namespace

const BoreStep* BoreConfig::find(const std::string& id) const {
    for (const BoreStep& b : steps) if (b.id == id) return &b;
    return nullptr;
}

const InternalGrade* InternalPartKind::find(const std::string& id) const {
    for (const InternalGrade& g : grades) if (g.id == id) return &g;
    return nullptr;
}

const InternalPartKind* EngineInternalsConfig::find(const std::string& kind) const {
    for (const InternalPartKind& p : parts) if (p.kind == kind) return &p;
    return nullptr;
}

std::optional<EngineInternalsConfig> parse_engine_internals(const std::string& json_text, const std::string& origin, std::string* err) {
    const auto root = detail::parse_json(json_text);
    if (!root || !root->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    std::string format;
    if (!detail::get_string(*root, "format", format) || format != "rg.engine_internals/1") {
        fail(err, origin, "\"format\" must be \"rg.engine_internals/1\"");
        return std::nullopt;
    }
    EngineInternalsConfig c;
    if (!detail::get_number(*root, "stock_margin", c.stock_margin) || !(c.stock_margin >= 1.0 && c.stock_margin <= 2.0)) {
        fail(err, origin, "\"stock_margin\" must be a number in [1, 2]");
        return std::nullopt;
    }
    if (root->contains("rod_crank_ratio") && (!detail::get_number(*root, "rod_crank_ratio", c.rod_crank_ratio) || !(c.rod_crank_ratio > 0.1 && c.rod_crank_ratio < 0.6))) {
        fail(err, origin, "\"rod_crank_ratio\" must be a number in (0.1, 0.6)");
        return std::nullopt;
    }
    if (root->contains("rod_small_end_share") && (!detail::get_number(*root, "rod_small_end_share", c.rod_small_end_share) || !(c.rod_small_end_share >= 0.0 && c.rod_small_end_share <= 1.0))) {
        fail(err, origin, "\"rod_small_end_share\" must be a number in [0, 1]");
        return std::nullopt;
    }
    if (const auto bo = root->find("bore"); bo != root->end()) {
        if (!bo->is_object()) {
            fail(err, origin, "\"bore\" must be an object");
            return std::nullopt;
        }
        BoreConfig b;
        struct Num { const char* key; double* out; double lo, hi; };
        const Num nums[] = {
            {"pitch_factor", &b.pitch_factor, 1.01, 1.5},
            {"min_wall_mm", &b.min_wall_mm, 1.0, 30.0},
            {"safe_wall_mm", &b.safe_wall_mm, 1.0, 30.0},
            {"piston_thickness_exponent", &b.piston_thickness_exponent, 0.0, 2.0},
            {"liner_length_per_stroke", &b.liner_length_per_stroke, 0.5, 3.0},
            {"liner_density_kg_m3", &b.liner_density_kg_m3, 1000.0, 20000.0},
        };
        for (const Num& n : nums) {
            if (!detail::get_number(*bo, n.key, *n.out) || !(*n.out >= n.lo && *n.out <= n.hi)) {
                fail(err, origin, std::string("bore.") + n.key + " must be a number in [" + fmt("%g", n.lo) + ", " + fmt("%g", n.hi) + "]");
                return std::nullopt;
            }
        }
        if (b.safe_wall_mm < b.min_wall_mm) {
            fail(err, origin, "bore.safe_wall_mm must not be below bore.min_wall_mm");
            return std::nullopt;
        }
        std::string rule;
        if (!detail::get_string(*bo, "compression_rule", rule) || rule != "head_volume_kept") {
            fail(err, origin, "bore.compression_rule must be \"head_volume_kept\"");
            return std::nullopt;
        }
        const auto steps = bo->find("steps");
        if (steps == bo->end() || !steps->is_array() || steps->empty()) {
            fail(err, origin, "bore.steps must be a non-empty array");
            return std::nullopt;
        }
        std::set<std::string> ids;
        double last = 0.0;
        for (size_t i = 0; i < steps->size(); ++i) {
            const json& so = (*steps)[i];
            const std::string where = "bore.steps[" + std::to_string(i) + "]";
            BoreStep st;
            if (!so.is_object() || !detail::get_string(so, "id", st.id) || !is_snake(st.id) || st.id == "stock" || !ids.insert(st.id).second) {
                fail(err, origin, where + ": \"id\" must be a unique lower_snake name other than \"stock\"");
                return std::nullopt;
            }
            if (!detail::get_string(so, "label", st.label) || st.label.empty() || !detail::get_string(so, "source", st.source) || st.source.empty()) {
                fail(err, origin, where + ": \"label\" and \"source\" are required");
                return std::nullopt;
            }
            if (!detail::get_number(so, "oversize_mm", st.oversize_mm) || !(st.oversize_mm > last && st.oversize_mm <= 20.0)) {
                fail(err, origin, where + ": \"oversize_mm\" must be in (0, 20] and grow from step to step");
                return std::nullopt;
            }
            last = st.oversize_mm;
            b.steps.push_back(std::move(st));
        }
        c.bore = std::move(b);
    }
    const auto parts = root->find("parts");
    if (parts == root->end() || !parts->is_array() || parts->empty()) {
        fail(err, origin, "\"parts\" must be a non-empty array");
        return std::nullopt;
    }
    std::set<std::string> kinds;
    int limiter_parts = 0;
    int rod_parts = 0;
    for (size_t i = 0; i < parts->size(); ++i) {
        const json& o = (*parts)[i];
        const std::string where = "parts[" + std::to_string(i) + "]";
        if (!o.is_object()) {
            fail(err, origin, where + ": not an object");
            return std::nullopt;
        }
        InternalPartKind p;
        std::string lk;
        if (!detail::get_string(o, "kind", p.kind) || !is_snake(p.kind) || !kinds.insert(p.kind).second) {
            fail(err, origin, where + ": \"kind\" must be a unique lower_snake name");
            return std::nullopt;
        }
        if (!detail::get_string(o, "label", p.label) || p.label.empty()) {
            fail(err, origin, where + ": \"label\" is required");
            return std::nullopt;
        }
        if (!detail::get_string(o, "limit_kind", lk) || (lk != "mean_piston_speed" && lk != "limiter_factor" && lk != "rod_load")) {
            fail(err, origin, where + ": \"limit_kind\" must be \"mean_piston_speed\", \"limiter_factor\" or \"rod_load\"");
            return std::nullopt;
        }
        p.limit_kind = lk == "mean_piston_speed" ? InternalLimitKind::MeanPistonSpeed
                       : lk == "rod_load"        ? InternalLimitKind::RodLoad
                                                 : InternalLimitKind::LimiterFactor;
        if (p.limit_kind == InternalLimitKind::LimiterFactor && ++limiter_parts > 1) {
            fail(err, origin, where + ": only one part may have limit_kind \"limiter_factor\"");
            return std::nullopt;
        }
        if (!detail::get_number(o, "stock_mass_per_litre_kg", p.stock_mass_per_litre_kg) ||
            !(p.stock_mass_per_litre_kg > 0.0 && p.stock_mass_per_litre_kg < 50.0)) {
            fail(err, origin, where + ": \"stock_mass_per_litre_kg\" must be in (0, 50)");
            return std::nullopt;
        }
        if (!detail::get_string(o, "mass_source", p.mass_source) || p.mass_source.empty()) {
            fail(err, origin, where + ": \"mass_source\" is required");
            return std::nullopt;
        }
        const auto grades = o.find("grades");
        if (grades == o.end() || !grades->is_array() || grades->empty()) {
            fail(err, origin, where + ": \"grades\" must be a non-empty array");
            return std::nullopt;
        }
        std::set<std::string> ids;
        for (size_t g = 0; g < grades->size(); ++g) {
            const json& go = (*grades)[g];
            const std::string gw = where + ".grades[" + std::to_string(g) + "]";
            InternalGrade gr;
            if (!go.is_object() || !detail::get_string(go, "id", gr.id) || !is_snake(gr.id) || !ids.insert(gr.id).second) {
                fail(err, origin, gw + ": \"id\" must be a unique lower_snake name");
                return std::nullopt;
            }
            if (g == 0 && gr.id != "stock") {
                fail(err, origin, gw + ": the first grade must be \"stock\"");
                return std::nullopt;
            }
            if (!detail::get_string(go, "label", gr.label) || gr.label.empty() || !detail::get_string(go, "source", gr.source) || gr.source.empty()) {
                fail(err, origin, gw + ": \"label\" and \"source\" are required");
                return std::nullopt;
            }
            const double lo = p.limit_kind == InternalLimitKind::MeanPistonSpeed ? 5.0 : p.limit_kind == InternalLimitKind::RodLoad ? 1000.0 : 0.5;
            const double hi = p.limit_kind == InternalLimitKind::MeanPistonSpeed ? 60.0 : p.limit_kind == InternalLimitKind::RodLoad ? 1.0e6 : 3.0;
            if (!detail::get_number(go, "limit", gr.limit) || !(gr.limit >= lo && gr.limit <= hi)) {
                fail(err, origin, gw + ": \"limit\" must be in [" + fmt("%g", lo) + ", " + fmt("%g", hi) + "]");
                return std::nullopt;
            }
            if (!detail::get_number(go, "mass_factor", gr.mass_factor) || !(gr.mass_factor > 0.1 && gr.mass_factor < 3.0)) {
                fail(err, origin, gw + ": \"mass_factor\" must be in (0.1, 3)");
                return std::nullopt;
            }
            if (g == 0 && gr.mass_factor != 1.0) {
                fail(err, origin, gw + ": the stock grade's mass_factor is 1");
                return std::nullopt;
            }
            p.grades.push_back(std::move(gr));
        }
        if (p.limit_kind == InternalLimitKind::RodLoad) ++rod_parts;
        c.parts.push_back(std::move(p));
    }
    if (rod_parts > 1 || (rod_parts == 1 && c.find("pistons") == nullptr)) {
        fail(err, origin, "at most one \"rod_load\" part, and it needs a \"pistons\" part (the piston mass loads the rod)");
        return std::nullopt;
    }
    return c;
}

std::optional<EngineInternalsConfig> load_engine_internals(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot read the file");
        return std::nullopt;
    }
    return parse_engine_internals(*text, path, err);
}

std::string engine_internals_refusal(const EngineFacts& f, const std::string& name) {
    const std::string who = name.empty() ? std::string("This engine") : "The " + name;
    if (!f.simulated) {
        return who + " is a torque-curve engine: it has no modelled pistons, rods or valves to swap. Swap in a simulated engine "
                     "to change its internals.";
    }
    if (f.rotary) return who + " is a rotary engine: it has no pistons, connecting rods, crankshaft or valve springs.";
    if (!f.has_cycle || !(f.stroke_m > 0.0) || !(f.displacement_l > 0.0)) {
        return who + " has authored maps and no cycle model, so its stroke and swept volume are unknown.";
    }
    if (!(f.limiter_rpm > 0.0)) return who + " has no rev limiter to rate the parts against.";
    return {};
}

std::optional<BoreGeometry> bore_geometry(const EngineInternalsConfig& config, const EngineFacts& facts, const std::string& step, std::string* err) {
    const std::string refusal = engine_internals_refusal(facts, {});
    if (!refusal.empty()) {
        if (err != nullptr) *err = refusal;
        return std::nullopt;
    }
    if (!config.bore) {
        if (err != nullptr) *err = "engine_internals.json has no \"bore\" block";
        return std::nullopt;
    }
    const BoreConfig& bc = *config.bore;
    const BoreStep* st = step == "stock" ? nullptr : bc.find(step);
    if (step != "stock" && st == nullptr) {
        if (err != nullptr) *err = "\"" + step + "\" is not a bore step";
        return std::nullopt;
    }
    const double fb = factory_bore(facts);
    if (!(fb > 0.0)) {
        if (err != nullptr) *err = "the engine has no bore";
        return std::nullopt;
    }
    BoreGeometry g;
    g.step = st != nullptr ? st->id : "stock";
    g.label = st != nullptr ? st->label : "Stock bore";
    g.oversize_mm = st != nullptr ? st->oversize_mm : 0.0;
    g.factory_bore_m = fb;
    g.bore_m = fb + g.oversize_mm / 1000.0;
    g.spacing_mm = bc.pitch_factor * fb * 1000.0;
    g.wall_mm = g.spacing_mm - g.bore_m * 1000.0;
    g.min_wall_mm = bc.min_wall_mm;
    g.safe_wall_mm = bc.safe_wall_mm;
    g.max_oversize_mm = std::max(0.0, g.spacing_mm - fb * 1000.0 - bc.min_wall_mm);
    const double ratio2 = (g.bore_m / fb) * (g.bore_m / fb);
    g.factory_displacement_l = factory_displacement(facts);
    g.displacement_l = g.factory_displacement_l * ratio2;
    g.factory_compression_ratio = factory_cr(facts);
    // Head volume kept: Vc = Vs / (CR - 1) stays, Vs grows with bore^2. Rounded to 0.001 so the engine file stays tidy.
    g.compression_ratio = g.factory_compression_ratio > 1.0 ? std::round((1.0 + (g.factory_compression_ratio - 1.0) * ratio2) * 1000.0) / 1000.0
                                                            : g.factory_compression_ratio;
    const double liner_m = bc.liner_length_per_stroke * facts.stroke_m;
    const double area = 3.14159265358979323846 / 4.0 * (g.bore_m * g.bore_m - fb * fb);
    g.block_mass_delta_kg = -std::max(1, facts.cylinders) * area * liner_m * bc.liner_density_kg_m3;
    // Compare in 1/1000 mm so a step exactly at the minimum fits regardless of the last float bit.
    g.fits = std::llround(g.wall_mm * 1000.0) >= std::llround(bc.min_wall_mm * 1000.0);
    g.warns = g.fits && std::llround(g.wall_mm * 1000.0) < std::llround(bc.safe_wall_mm * 1000.0);
    if (!g.fits) {
        g.reason = "Boring " + fmt("%.1f", fb * 1000.0) + " mm out to " + fmt("%.1f", g.bore_m * 1000.0) + " mm would leave a " +
                   fmt("%.1f", g.wall_mm) + " mm wall between two cylinders (bore spacing about " + fmt("%.1f", g.spacing_mm) +
                   " mm); the block needs at least " + fmt("%.1f", bc.min_wall_mm) + " mm, so it takes at most +" +
                   fmt("%.1f", g.max_oversize_mm) + " mm.";
    }
    return g;
}

std::string bore_refusal(const EngineInternalsConfig& config, const EngineFacts& facts, const std::string& step, const std::string& name) {
    const std::string r = engine_internals_refusal(facts, name);
    if (!r.empty()) return r;
    std::string err;
    const auto g = bore_geometry(config, facts, step, &err);
    if (!g) return err;
    return g->fits ? std::string() : g->reason;
}

EngineFacts bored_engine_facts(const EngineFacts& facts, const BoreGeometry& g) {
    EngineFacts f = facts;
    if (!(f.factory_bore_m > 0.0)) f.factory_bore_m = g.factory_bore_m;
    if (!(f.factory_displacement_l > 0.0)) f.factory_displacement_l = g.factory_displacement_l;
    if (!(f.factory_compression_ratio > 0.0)) f.factory_compression_ratio = g.factory_compression_ratio;
    f.bore_m = g.bore_m;
    f.displacement_l = g.displacement_l;
    f.compression_ratio = g.compression_ratio;
    return f;
}

std::string bore_step_detail(const BoreGeometry& g) {
    std::string out = fmt("%.1f", g.bore_m * 1000.0) + " mm bore · " + fmt("%.2f", g.displacement_l) + " L";
    if (g.compression_ratio > 0.0) out += " · CR " + fmt("%.1f", g.compression_ratio);
    out += " · wall " + fmt("%.1f", g.wall_mm) + " mm";
    if (g.warns) out += " (thin: the setup check warns)";
    if (g.block_mass_delta_kg < -0.05) out += " · " + fmt("%.1f", g.block_mass_delta_kg) + " kg block";
    return out;
}

const InternalPartBuild* EngineInternalsBuild::weakest_piston_speed() const {
    const InternalPartBuild* w = nullptr;
    for (const InternalPartBuild& p : parts) {
        if (p.limit_kind == InternalLimitKind::LimiterFactor) continue;
        if (w == nullptr || p.limit_rpm < w->limit_rpm) w = &p; // ties: the first in config order
    }
    return w;
}

const InternalPartBuild* EngineInternalsBuild::valve_springs() const {
    for (const InternalPartBuild& p : parts) if (p.limit_kind == InternalLimitKind::LimiterFactor) return &p;
    return nullptr;
}

double EngineInternalsBuild::mass_delta_kg() const {
    double d = bore ? bore->block_mass_delta_kg : 0.0;
    for (const InternalPartBuild& p : parts) d += p.mass_delta_kg;
    return d;
}

std::optional<EngineInternalsBuild> describe_engine_internals(const EngineInternalsConfig& config, const EngineFacts& facts,
                                                             const std::map<std::string, std::string>& grades,
                                                             const std::map<std::string, std::string>& options, std::string* err) {
    const std::string refusal = engine_internals_refusal(facts, {});
    if (!refusal.empty()) {
        if (err != nullptr) *err = refusal;
        return std::nullopt;
    }
    EngineInternalsBuild b;
    EngineFacts f = facts;
    if (const auto bi = grades.find(kCylinderBoreKind); bi != grades.end() && bi->second != "stock") {
        std::string berr;
        auto g = bore_geometry(config, facts, bi->second, &berr);
        if (!g || !g->fits) {
            if (err != nullptr) *err = g ? g->reason : berr;
            return std::nullopt;
        }
        f = bored_engine_facts(facts, *g);
        b.bore = std::move(g);
        if (const auto o = options.find(kCylinderBoreKind); o != options.end()) b.bore_option_id = o->second;
    } else if (factory_bore(facts) > 0.0) {
        f.bore_m = factory_bore(facts); // "stock" bore: whatever the facts say, the factory pistons
        f.displacement_l = factory_displacement(facts);
        f.compression_ratio = factory_cr(facts);
    }
    for (const auto& [kind, grade] : grades) {
        if (kind == kCylinderBoreKind) continue;
        const InternalPartKind* k = config.find(kind);
        if (k == nullptr || k->find(grade) == nullptr) {
            if (err != nullptr) *err = "\"" + grade + "\" is not a grade of the engine part \"" + kind + "\"";
            return std::nullopt;
        }
    }
    b.stroke_m = f.stroke_m;
    b.factory_limiter_rpm = f.limiter_rpm;
    b.factory_piston_speed_m_s = mean_piston_speed(f.stroke_m, f.limiter_rpm);
    const double scale = piston_scale(config, f);
    const auto grade_of = [&](const InternalPartKind& k) -> const InternalGrade& {
        const auto it = grades.find(k.kind);
        return *k.find(it == grades.end() ? std::string("stock") : it->second);
    };
    for (const InternalPartKind& k : config.parts) {
        const InternalGrade& g = grade_of(k);
        InternalPartBuild p;
        p.kind = k.kind;
        p.kind_label = k.label;
        p.grade = g.id;
        p.grade_label = g.label;
        p.limit_kind = k.limit_kind;
        if (k.limit_kind == InternalLimitKind::RodLoad) {
            p.limit = rod_rating_n(config, k, g, f);
            const InternalPartKind* pk = piston_part(config);
            p.recip_mass_kg = recip_mass_kg(config, f, k, g, grade_of(*pk), scale);
            p.limit_rpm = rod_limit_rpm(p.limit, p.recip_mass_kg, f.stroke_m, config.rod_crank_ratio);
        } else {
            const double lim = effective_limit(config, k, g, f);
            p.limit = k.limit_kind == InternalLimitKind::MeanPistonSpeed ? lim : lim * f.limiter_rpm;
            p.limit_rpm = k.limit_kind == InternalLimitKind::MeanPistonSpeed ? lim * 60.0 / (2.0 * f.stroke_m) : p.limit;
        }
        // Masses on the factory displacement; only the pistons follow the bore (the delta is to the FACTORY stock set).
        const bool is_piston = &k == piston_part(config);
        const double stock_kg = k.stock_mass_per_litre_kg * factory_displacement(f);
        p.mass_kg = stock_kg * g.mass_factor * (is_piston ? scale : 1.0);
        p.mass_delta_kg = p.mass_kg - stock_kg;
        p.mass_source = k.mass_source + " Grade: " + g.source;
        if (is_piston && b.bore) {
            p.grade_label += ", " + fmt("%.1f", b.bore->bore_m * 1000.0) + " mm";
            p.mass_source += " Sized to the " + fmt("%.1f", b.bore->bore_m * 1000.0) + " mm bore: x" + fmt("%.3f", scale) +
                             " = (bore / factory bore)^(2 + piston_thickness_exponent).";
        }
        if (const auto o = options.find(k.kind); o != options.end()) p.option_id = o->second;
        b.parts.push_back(std::move(p));
    }
    return b;
}

double internal_grade_limit(const EngineInternalsConfig& config, const EngineFacts& facts, const std::string& kind, const std::string& grade) {
    const InternalPartKind* k = config.find(kind);
    const InternalGrade* g = k != nullptr ? k->find(grade) : nullptr;
    if (g == nullptr) return std::numeric_limits<double>::quiet_NaN();
    if (k->limit_kind == InternalLimitKind::RodLoad) return rod_rating_n(config, *k, *g, facts); // N per rod
    const double lim = effective_limit(config, *k, *g, facts);
    return k->limit_kind == InternalLimitKind::MeanPistonSpeed ? lim : lim * facts.limiter_rpm;
}

std::string internal_grade_detail(const EngineInternalsConfig& config, const EngineFacts& facts, const std::string& kind, const std::string& grade) {
    const InternalPartKind* k = config.find(kind);
    const InternalGrade* g = k != nullptr ? k->find(grade) : nullptr;
    if (g == nullptr) return {};
    const double lim = internal_grade_limit(config, facts, kind, grade);
    std::string out;
    if (k->limit_kind == InternalLimitKind::RodLoad) {
        // With the stock pistons; a lighter or heavier piston moves it (the setup check uses the installed ones).
        const double m = recip_mass_kg(config, facts, *k, *g, piston_part(config)->grades.front(), piston_scale(config, facts));
        out = "rated " + thousands(lim) + " N · " + thousands(rod_limit_rpm(lim, m, facts.stroke_m, config.rod_crank_ratio)) + " rpm with stock pistons";
        if (piston_scale(config, facts) != 1.0) out += " (" + fmt("%.1f", facts.bore_m * 1000.0) + " mm)";
    } else if (k->limit_kind == InternalLimitKind::MeanPistonSpeed) {
        out = "holds " + fmt("%.1f", lim) + " m/s mean piston speed";
        if (facts.stroke_m > 0.0) out += " (" + thousands(lim * 60.0 / (2.0 * facts.stroke_m)) + " rpm)";
    } else {
        out = "valves float above " + thousands(lim) + " rpm";
    }
    if (g->limit < k->grades.front().limit) {
        bool warns = false;
        if (k->limit_kind == InternalLimitKind::MeanPistonSpeed) warns = lim < factory_reference(k->limit_kind, facts);
        else if (k->limit_kind == InternalLimitKind::LimiterFactor) warns = lim < facts.limiter_rpm;
        else warns = rod_limit_rpm(lim, recip_mass_kg(config, facts, *k, *g, piston_part(config)->grades.front(), piston_scale(config, facts)), facts.stroke_m, config.rod_crank_ratio) < facts.limiter_rpm;
        out += warns ? " · below stock: warns at this engine's rev limit" : " · below stock";
    }
    const bool is_piston = k == piston_part(config);
    const double scale = is_piston ? piston_scale(config, facts) : 1.0;
    const double stock_kg = k->stock_mass_per_litre_kg * factory_displacement(facts);
    const double delta = stock_kg * (g->mass_factor * scale - 1.0);
    if (is_piston && scale != 1.0) out += " · " + fmt("%.1f", facts.bore_m * 1000.0) + " mm bore";
    out += " · " + fmt("%.1f", stock_kg * g->mass_factor * scale) + " kg";
    if (std::abs(delta) >= 0.05) out += " (" + std::string(delta > 0.0 ? "+" : "") + fmt("%.1f", delta) + " kg)";
    return out;
}

std::optional<PartAttachment> engine_internals_parts(const EngineInternalsBuild& build) {
    PartAttachment a;
    a.parent_id = "engine";
    a.node.id = "internals";
    a.node.key = "engine_internals";
    a.node.label = "Engine internals (difference to stock)";
    a.node.source = "engine_internals.json: each changed part's mass minus the stock part's; the engine's own mass holds the stock parts";
    for (const InternalPartBuild& p : build.parts) {
        if (p.grade == "stock" && std::abs(p.mass_delta_kg) < 1e-9) continue;
        PartNode leaf;
        leaf.id = "engine.internals." + p.kind;
        leaf.key = "engine_internals";
        std::string kind_lower = p.kind_label;
        for (char& ch : kind_lower) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        leaf.label = p.grade_label + " " + kind_lower; // "Titanium connecting rods"
        leaf.own_kg = p.mass_delta_kg;
        leaf.source = p.mass_source;
        a.node.children.push_back(std::move(leaf));
    }
    if (build.bore) {
        PartNode leaf;
        leaf.id = "engine.internals.cylinder_block";
        leaf.key = "engine_internals";
        leaf.label = "Cylinder block bored to " + fmt("%.1f", build.bore->bore_m * 1000.0) + " mm (material removed)";
        leaf.own_kg = build.bore->block_mass_delta_kg;
        leaf.source = "engine_internals.json bore: n x pi/4 x (bore^2 - factory bore^2) x liner_length_per_stroke x stroke x "
                      "liner_density_kg_m3 [AS]";
        a.node.children.push_back(std::move(leaf));
    }
    if (a.node.children.empty()) return std::nullopt;
    return a;
}

} // namespace rg
