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

// Reciprocating mass per cylinder: the piston set (grade `piston`) plus the small-end share of the rod set (grade `rod`).
double recip_mass_kg(const EngineInternalsConfig& c, const EngineFacts& f, const InternalPartKind& rods, const InternalGrade& rod,
                     const InternalGrade& piston) {
    const InternalPartKind* pk = piston_part(c);
    const double n = std::max(1, f.cylinders);
    const double piston_kg = (pk != nullptr ? pk->stock_mass_per_litre_kg : 0.0) * f.displacement_l * piston.mass_factor / n;
    const double rod_kg = rods.stock_mass_per_litre_kg * f.displacement_l * rod.mass_factor / n;
    return piston_kg + c.rod_small_end_share * rod_kg;
}

// A rod grade's rated tension in N per rod on this engine. Stock: at least what keeps stock_margin x the factory limiter
// with the stock pistons; an upgrade never below that; a downgrade (table figure below stock's) keeps its own figure.
double rod_rating_n(const EngineInternalsConfig& c, const InternalPartKind& kind, const InternalGrade& grade, const EngineFacts& f) {
    const double cyl_l = f.displacement_l / std::max(1, f.cylinders);
    const InternalGrade& stock_g = kind.grades.front();
    const InternalGrade* stock_piston = piston_part(c) != nullptr ? &piston_part(c)->grades.front() : nullptr;
    double needed = 0.0;
    if (stock_piston != nullptr) {
        needed = rod_force_n(recip_mass_kg(c, f, kind, stock_g, *stock_piston), f.stroke_m, c.rod_crank_ratio, c.stock_margin * f.limiter_rpm);
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
    double d = 0.0;
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
    for (const auto& [kind, grade] : grades) {
        const InternalPartKind* k = config.find(kind);
        if (k == nullptr || k->find(grade) == nullptr) {
            if (err != nullptr) *err = "\"" + grade + "\" is not a grade of the engine part \"" + kind + "\"";
            return std::nullopt;
        }
    }
    EngineInternalsBuild b;
    b.stroke_m = facts.stroke_m;
    b.factory_limiter_rpm = facts.limiter_rpm;
    b.factory_piston_speed_m_s = mean_piston_speed(facts.stroke_m, facts.limiter_rpm);
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
            p.limit = rod_rating_n(config, k, g, facts);
            const InternalPartKind* pk = piston_part(config);
            p.recip_mass_kg = recip_mass_kg(config, facts, k, g, grade_of(*pk));
            p.limit_rpm = rod_limit_rpm(p.limit, p.recip_mass_kg, facts.stroke_m, config.rod_crank_ratio);
        } else {
            const double lim = effective_limit(config, k, g, facts);
            p.limit = k.limit_kind == InternalLimitKind::MeanPistonSpeed ? lim : lim * facts.limiter_rpm;
            p.limit_rpm = k.limit_kind == InternalLimitKind::MeanPistonSpeed ? lim * 60.0 / (2.0 * facts.stroke_m) : p.limit;
        }
        const double stock_kg = k.stock_mass_per_litre_kg * facts.displacement_l;
        p.mass_kg = stock_kg * g.mass_factor;
        p.mass_delta_kg = p.mass_kg - stock_kg;
        p.mass_source = k.mass_source + " Grade: " + g.source;
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
        const double m = recip_mass_kg(config, facts, *k, *g, piston_part(config)->grades.front());
        out = "rated " + thousands(lim) + " N · " + thousands(rod_limit_rpm(lim, m, facts.stroke_m, config.rod_crank_ratio)) + " rpm with stock pistons";
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
        else warns = rod_limit_rpm(lim, recip_mass_kg(config, facts, *k, *g, piston_part(config)->grades.front()), facts.stroke_m, config.rod_crank_ratio) < facts.limiter_rpm;
        out += warns ? " · below stock: warns at this engine's rev limit" : " · below stock";
    }
    const double stock_kg = k->stock_mass_per_litre_kg * facts.displacement_l;
    const double delta = stock_kg * (g->mass_factor - 1.0);
    out += " · " + fmt("%.1f", stock_kg * g->mass_factor) + " kg";
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
        if (p.grade == "stock") continue;
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
    if (a.node.children.empty()) return std::nullopt;
    return a;
}

} // namespace rg
