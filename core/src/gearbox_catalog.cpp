// rg/gearbox_catalog.cpp - see gearbox_catalog.h.
#include "rg/gearbox_catalog.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <set>
#include <system_error>

namespace rg {

namespace {

namespace fs = std::filesystem;
using detail::json;

std::string norm(const std::string& p) {
    std::error_code ec;
    fs::path abs = fs::absolute(fs::path(p), ec);
    if (ec) abs = fs::path(p);
    return abs.lexically_normal().generic_string();
}

std::string resolve_from(const std::string& dir, const std::string& rel) {
    const fs::path p(rel);
    return norm(p.is_absolute() ? rel : (fs::path(dir) / p).generic_string());
}

bool is_lower_snake(const std::string& s) {
    if (s.empty()) return false;
    for (const char c : s) {
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
    }
    return true;
}

bool one_of(const std::string& s, std::initializer_list<const char*> values) {
    for (const char* v : values) if (s == v) return true;
    return false;
}

std::string num0(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.0f", v);
    return buf;
}

std::string capitalised(std::string s) {
    if (!s.empty() && s[0] >= 'a' && s[0] <= 'z') s[0] = static_cast<char>(s[0] - 'a' + 'A');
    return s;
}

} // namespace

const char* to_string(GearboxKind k) {
    switch (k) {
        case GearboxKind::Manual: return "manual";
        case GearboxKind::Sequential: return "sequential";
        case GearboxKind::Automatic: return "automatic";
        case GearboxKind::DualClutch: return "dual_clutch";
        case GearboxKind::Cvt: return "cvt";
    }
    return "manual";
}

bool is_clutch_to_clutch(GearboxKind k) {
    return k == GearboxKind::Automatic || k == GearboxKind::DualClutch || k == GearboxKind::Cvt;
}

std::unique_ptr<GearboxCatalog> GearboxCatalog::open(const std::string& catalog_path, const std::string& repo_root, std::string* err) {
    const auto fail = [&](const std::string& m) -> std::unique_ptr<GearboxCatalog> {
        if (err != nullptr) *err = catalog_path + ": " + m;
        return nullptr;
    };
    const auto text = detail::read_text_file(catalog_path);
    if (!text) return fail("cannot open");
    const auto root = detail::parse_json(*text);
    if (!root || !root->is_object()) return fail("not a JSON object");
    std::string format;
    if (!detail::get_string(*root, "format", format) || format != "rg.gearbox_catalog/1") return fail("format must be rg.gearbox_catalog/1");
    const auto rows = root->find("gearboxes");
    if (rows == root->end() || !rows->is_array()) return fail("\"gearboxes\" must be an array");
    std::unique_ptr<GearboxCatalog> cat(new GearboxCatalog());
    std::set<std::string> seen;
    for (const json& row : *rows) {
        if (!row.is_object()) return fail("a gearbox row is not an object");
        GearboxEntry e;
        if (!detail::get_string(row, "id", e.id) || !is_lower_snake(e.id)) return fail("a gearbox needs a lower_snake \"id\"");
        const std::string where = "gearbox \"" + e.id + "\": ";
        if (!seen.insert(e.id).second) return fail("duplicate " + where);
        if (!detail::get_string(row, "name", e.name) || e.name.empty()) return fail(where + "\"name\" missing");
        std::string kind;
        detail::get_string(row, "kind", kind);
        if (kind == "manual") e.kind = GearboxKind::Manual;
        else if (kind == "sequential") e.kind = GearboxKind::Sequential;
        else if (kind == "automatic") e.kind = GearboxKind::Automatic;
        else if (kind == "dual_clutch") e.kind = GearboxKind::DualClutch;
        else if (kind == "cvt") e.kind = GearboxKind::Cvt;
        else return fail(where + "\"kind\" must be manual, sequential, automatic, dual_clutch or cvt");
        detail::get_string(row, "focus", e.focus);
        if (!one_of(e.focus, {"economy", "touring", "performance", "race"})) return fail(where + "\"focus\" must be economy, touring, performance or race");
        detail::get_string(row, "gear_type", e.gear_type);
        if (!one_of(e.gear_type, {"helical", "straight_cut"})) return fail(where + "\"gear_type\" must be helical or straight_cut");
        detail::get_string(row, "synchro", e.synchro);
        if (!one_of(e.synchro, {"cone", "dog", "none"})) return fail(where + "\"synchro\" must be cone, dog or none");
        if ((e.synchro == "none") != is_clutch_to_clutch(e.kind)) {
            return fail(where + "\"synchro\" is none exactly for the automatic, dual_clutch and cvt kinds");
        }
        detail::get_string(row, "description", e.description);
        if (!detail::get_number(row, "rating_nm", e.rating_nm) || !(e.rating_nm > 0.0)) return fail(where + "\"rating_nm\" must be a number > 0");
        std::string file;
        if (!detail::get_string(row, "file", file) || file.empty()) return fail(where + "\"file\" missing");
        e.file = resolve_from(repo_root, file);
        // The gearbox file itself: format, selector type and the ratios the card shows.
        const auto box_text = detail::read_text_file(e.file);
        if (!box_text) return fail(where + e.file + ": cannot open");
        const auto box = detail::parse_json(*box_text);
        if (!box || !box->is_object()) return fail(where + e.file + ": not a JSON object");
        std::string box_format, box_type = "manual";
        detail::get_string(*box, "format", box_format);
        if (box_format != "physics_sim.gearbox/1") return fail(where + e.file + ": not a physics_sim.gearbox/1 file");
        detail::get_string(*box, "type", box_type);
        if (box_type != to_string(e.kind)) {
            return fail(where + "kind " + to_string(e.kind) + " but the file's type is " + box_type);
        }
        e.converter = box->contains("converter");
        if (e.kind == GearboxKind::Cvt) {
            // The variator: its range and virtual steps (the cvt file has no forward_ratios).
            const auto cvt = box->find("cvt");
            if (cvt == box->end() || !cvt->is_object()) return fail(where + e.file + ": a cvt file needs its \"cvt\" block");
            const auto steps = cvt->find("steps");
            if (steps == cvt->end() || !steps->is_array() || steps->empty()) return fail(where + e.file + ": no cvt steps");
            e.gears = static_cast<int>(steps->size());
            if (!detail::get_number(*cvt, "ratio_low", e.first_ratio) || !detail::get_number(*cvt, "ratio_high", e.top_ratio)) {
                return fail(where + e.file + ": no cvt ratio_low / ratio_high");
            }
        } else {
            const auto ratios = box->find("forward_ratios");
            if (ratios == box->end() || !ratios->is_array() || ratios->empty()) return fail(where + e.file + ": no forward_ratios");
            e.gears = static_cast<int>(ratios->size());
            e.first_ratio = ratios->front().is_number() ? ratios->front().get<double>() : 0.0;
            e.top_ratio = ratios->back().is_number() ? ratios->back().get<double>() : 0.0;
        }
        const auto tcu = row.find("tcu");
        if (is_clutch_to_clutch(e.kind)) {
            if (tcu == row.end() || !tcu->is_object()) return fail(where + "an automatic, dual_clutch or cvt box needs a \"tcu\" object");
            for (auto it = tcu->begin(); it != tcu->end(); ++it) {
                if (!one_of(it.key(), {"min_time_in_gear_s", "kickdown_throttle", "manual_timeout_s", "manual_up_rpm",
                                       "upshift_torque_fraction", "creep_torque_nm", "lockup", "cvt"})) {
                    return fail(where + "\"tcu\": unknown or derived key \"" + it.key() + "\"");
                }
            }
            if (tcu->contains("lockup") && !e.converter) return fail(where + "\"tcu\": lockup needs a converter in the gearbox file");
            if (tcu->contains("cvt") != (e.kind == GearboxKind::Cvt)) {
                return fail(where + (e.kind == GearboxKind::Cvt ? "\"tcu\": a cvt box needs \"cvt\": {\"stepped_sport\"}"
                                                                 : "\"tcu\": \"cvt\" is for a cvt box only"));
            }
            if (tcu->contains("cvt")) {
                const json& c = (*tcu)["cvt"];
                if (!c.is_object()) return fail(where + "\"tcu\": cvt must be an object");
                for (auto it = c.begin(); it != c.end(); ++it) {
                    if (it.key() != "stepped_sport" || !it.value().is_boolean()) {
                        return fail(where + "\"tcu\": cvt: only \"stepped_sport\" (true/false); the rpm curves are derived");
                    }
                }
            }
            if (tcu->contains("lockup")) {
                const json& lk = (*tcu)["lockup"];
                if (!lk.is_object()) return fail(where + "\"tcu\": lockup must be an object");
                for (auto it = lk.begin(); it != lk.end(); ++it) {
                    if (!one_of(it.key(), {"enabled", "min_gear", "max_throttle"})) {
                        return fail(where + "\"tcu\": lockup: unknown or derived key \"" + it.key() + "\"");
                    }
                }
            }
            if (e.kind == GearboxKind::Automatic && !box->contains("converter")) return fail(where + e.file + ": an automatic box needs a converter");
            e.tcu_json = tcu->dump();
        } else if (tcu != row.end()) {
            return fail(where + "\"tcu\" is only for the automatic, dual_clutch and cvt kinds");
        }
        cat->entries_.push_back(std::move(e));
    }
    return cat;
}

AutomaticTcuPoints derive_automatic_tcu_points(double idle, double limiter) {
    AutomaticTcuPoints p;
    if (!(idle > 0.0) || !(limiter > idle + 1000.0)) return p;
    const double band = limiter - idle;
    const auto r = [](double v) { return std::round(v / 50.0) * 50.0; };
    p.up_zero = r(idle + 0.20 * band);
    p.up_full = r(limiter - 400.0);
    p.down_zero = std::max(r(idle + 250.0), r(idle + 0.05 * band));
    p.down_full = r(idle + 0.40 * band);
    p.sport_up_zero = r(idle + 0.33 * band);
    p.sport_up_full = r(limiter - 250.0);
    p.sport_down_zero = r(idle + 0.17 * band);
    p.sport_down_full = r(idle + 0.53 * band);
    p.kickdown_max_rpm = r(limiter - 600.0);
    p.lockup_engage_rpm = std::max(r(idle + 400.0), r(idle + 0.10 * band));
    p.lockup_release_rpm = std::min(p.lockup_engage_rpm - 100.0, std::max(r(idle + 250.0), r(idle + 0.05 * band)));
    p.launch_bite_rpm = r(idle + 200.0);
    p.launch_full_rpm_min = std::max(p.launch_bite_rpm + 400.0, r(idle + 0.12 * band));
    p.launch_full_rpm_max = std::max(p.launch_full_rpm_min, r(idle + 0.45 * band));
    p.launch_anti_stall_rpm = r(0.75 * idle);
    p.launch_lock_slip_rpm = 100.0;
    const double target_f[4] = {0.09, 0.16, 0.32, 0.60};
    for (int i = 0; i < 4; ++i) p.cvt_target_rpm[i] = r(idle + target_f[i] * band);
    p.cvt_target_rpm[4] = r(limiter - 0.15 * band);
    const double min_f[3] = {0.07, 0.12, 0.25};
    for (int i = 0; i < 3; ++i) p.cvt_min_rpm[i] = r(idle + min_f[i] * band);
    p.ok = p.down_zero < p.up_zero && p.down_full < p.up_full && p.sport_down_zero < p.sport_up_zero &&
           p.sport_down_full < p.sport_up_full && p.launch_anti_stall_rpm > 0.0;
    return p;
}

const GearboxEntry* GearboxCatalog::find(const std::string& id) const {
    for (const GearboxEntry& e : entries_) if (e.id == id) return &e;
    return nullptr;
}

const GearboxEntry* GearboxCatalog::find_by_file(const std::string& path) const {
    const std::string want = norm(path);
    for (const GearboxEntry& e : entries_) if (e.file == want) return &e;
    return nullptr;
}

std::string gearbox_section(const GearboxEntry& e) {
    switch (e.kind) {
        case GearboxKind::Manual:
        case GearboxKind::Sequential: return "Manual";
        case GearboxKind::Automatic: return "Automatic";
        case GearboxKind::DualClutch: return "Dual-clutch";
        case GearboxKind::Cvt: return "CVT";
    }
    return "Manual";
}

std::vector<GearboxSection> GearboxCatalog::sections() const {
    std::vector<GearboxSection> out = {{"Manual", false}, {"Automatic", false}, {"Dual-clutch", false}, {"CVT", false}};
    for (const GearboxEntry& e : entries_) {
        const std::string label = gearbox_section(e);
        for (GearboxSection& s : out) if (s.label == label) s.has_entries = true;
    }
    return out;
}

std::string check_gearbox_rating(const GearboxEntry& box, double engine_peak_torque_nm) {
    if (!(engine_peak_torque_nm > 0.0)) return {};
    if (box.rating_nm + 1e-9 >= engine_peak_torque_nm) return {};
    return "gearbox: rated " + num0(box.rating_nm) + " N*m, engine makes " + num0(engine_peak_torque_nm) + " N*m (" + box.name + ")";
}

std::string gearbox_summary_line(const GearboxEntry& e) {
    std::string s = e.kind == GearboxKind::Sequential  ? "Sequential"
                  : e.kind == GearboxKind::DualClutch ? "Dual-clutch"
                  : e.kind == GearboxKind::Cvt        ? (e.converter ? "CVT, torque converter" : "CVT, starting clutch")
                                                      : capitalised(to_string(e.kind));
    s += " · " + capitalised(e.focus);
    if (e.gear_type == "straight_cut") s += " · straight-cut";
    return s;
}

std::string gearbox_detail_line(const GearboxEntry& e) {
    char buf[128];
    if (e.kind == GearboxKind::Cvt) {
        std::snprintf(buf, sizeof(buf), "stepless %.2f to %.2f · %d steps in S · rated %.0f N*m", e.first_ratio, e.top_ratio, e.gears,
                      e.rating_nm);
        return buf;
    }
    std::snprintf(buf, sizeof(buf), "%d gears · %.2f to %.2f · rated %.0f N*m", e.gears, e.first_ratio, e.top_ratio, e.rating_nm);
    return buf;
}

} // namespace rg
