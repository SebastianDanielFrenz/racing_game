// rg/gearbox_catalog.cpp - see gearbox_catalog.h.
#include "rg/gearbox_catalog.h"

#include "json_util.h"

#include <algorithm>
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
        case GearboxKind::Cvt: return "cvt";
    }
    return "manual";
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
        else if (kind == "cvt") e.kind = GearboxKind::Cvt;
        else return fail(where + "\"kind\" must be manual, sequential, automatic or cvt");
        detail::get_string(row, "focus", e.focus);
        if (!one_of(e.focus, {"economy", "touring", "performance", "race"})) return fail(where + "\"focus\" must be economy, touring, performance or race");
        detail::get_string(row, "gear_type", e.gear_type);
        if (!one_of(e.gear_type, {"helical", "straight_cut"})) return fail(where + "\"gear_type\" must be helical or straight_cut");
        detail::get_string(row, "synchro", e.synchro);
        if (!one_of(e.synchro, {"cone", "dog"})) return fail(where + "\"synchro\" must be cone or dog");
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
        if ((e.kind == GearboxKind::Manual || e.kind == GearboxKind::Sequential) && box_type != to_string(e.kind)) {
            return fail(where + "kind " + to_string(e.kind) + " but the file's type is " + box_type);
        }
        const auto ratios = box->find("forward_ratios");
        if (ratios == box->end() || !ratios->is_array() || ratios->empty()) return fail(where + e.file + ": no forward_ratios");
        e.gears = static_cast<int>(ratios->size());
        e.first_ratio = ratios->front().is_number() ? ratios->front().get<double>() : 0.0;
        e.top_ratio = ratios->back().is_number() ? ratios->back().get<double>() : 0.0;
        cat->entries_.push_back(std::move(e));
    }
    return cat;
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
        case GearboxKind::Cvt: return "CVT";
    }
    return "Manual";
}

std::vector<GearboxSection> GearboxCatalog::sections() const {
    std::vector<GearboxSection> out = {{"Manual", false}, {"Automatic", false}, {"CVT", false}};
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
    std::string s = e.kind == GearboxKind::Sequential ? "Sequential" : capitalised(to_string(e.kind));
    s += " · " + capitalised(e.focus);
    if (e.gear_type == "straight_cut") s += " · straight-cut";
    return s;
}

std::string gearbox_detail_line(const GearboxEntry& e) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%d gears · %.2f to %.2f · rated %.0f N*m", e.gears, e.first_ratio, e.top_ratio, e.rating_nm);
    return buf;
}

} // namespace rg
