// rg/vehicle_setup.cpp - see vehicle_setup.h.
#include "rg/vehicle_setup.h"

#include "json_util.h"

#include "ps/io/vehicle_io.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <system_error>

namespace rg {

namespace {

using detail::json;
namespace fs = std::filesystem;

// ---- small helpers ------------------------------------------------------------

std::string norm_path(const std::string& p) {
    std::error_code ec;
    fs::path abs = fs::absolute(fs::path(p), ec);
    if (ec) abs = fs::path(p);
    return abs.lexically_normal().generic_string();
}

std::string fmt_num(double v) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.4g", v);
    return buf;
}

std::string fmt2(double v) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.2f", v);
    return buf;
}

bool is_index(const std::string& s) {
    if (s.empty() || s.size() > 6) return false;
    for (const char c : s) {
        if (c < '0' || c > '9') return false;
    }
    return true;
}

std::string escape_pointer_key(const std::string& k) {
    std::string out;
    for (const char c : k) {
        if (c == '~') out += "~0";
        else if (c == '/') out += "~1";
        else out += c;
    }
    return out;
}

// ---- pointer patterns -----------------------------------------------------------
// "/a/b/*/c" and "/wheels/[is_front=true]/tyre": a segment is a key or array
// index, `*` (every array element) or `[key=value]` (the array elements whose
// field `key` equals `value`).

struct Segment {
    enum class Kind { Key, Wildcard, Filter } kind = Kind::Key;
    std::string key;
    std::string value;
};

bool parse_pattern(const std::string& pattern, std::vector<Segment>& out, std::string* err) {
    out.clear();
    if (pattern.empty() || pattern[0] != '/') {
        if (err != nullptr) *err = "pointer \"" + pattern + "\" must start with '/'";
        return false;
    }
    std::size_t pos = 1;
    while (pos <= pattern.size()) {
        const std::size_t end = pattern.find('/', pos);
        const std::string seg = pattern.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
        if (seg.empty()) {
            if (err != nullptr) *err = "pointer \"" + pattern + "\" has an empty segment";
            return false;
        }
        Segment s;
        if (seg == "*") {
            s.kind = Segment::Kind::Wildcard;
        } else if (seg.front() == '[') {
            const std::size_t eq = seg.find('=');
            if (seg.back() != ']' || eq == std::string::npos || eq < 2 || eq + 2 > seg.size() - 1 + 1) {
                if (err != nullptr) *err = "pointer \"" + pattern + "\": a filter must look like [key=value]";
                return false;
            }
            s.kind = Segment::Kind::Filter;
            s.key = seg.substr(1, eq - 1);
            s.value = seg.substr(eq + 1, seg.size() - eq - 2);
            if (s.key.empty() || s.value.empty()) {
                if (err != nullptr) *err = "pointer \"" + pattern + "\": a filter must look like [key=value]";
                return false;
            }
        } else {
            s.kind = Segment::Kind::Key;
            s.key = seg;
        }
        out.push_back(std::move(s));
        if (end == std::string::npos) break;
        pos = end + 1;
    }
    return !out.empty();
}

bool field_matches(const json& field, const std::string& value) {
    if (field.is_boolean()) return value == (field.get<bool>() ? "true" : "false");
    if (field.is_string()) return field.get<std::string>() == value;
    if (field.is_number()) {
        char* end = nullptr;
        const double v = std::strtod(value.c_str(), &end);
        return end != value.c_str() && *end == '\0' && field.get<double>() == v;
    }
    return false;
}

void expand_from(const json& node, const std::vector<Segment>& segs, std::size_t i, const std::string& prefix,
                 std::vector<std::string>& out) {
    if (i == segs.size()) {
        out.push_back(prefix);
        return;
    }
    const Segment& s = segs[i];
    switch (s.kind) {
        case Segment::Kind::Key:
            if (node.is_object()) {
                const auto it = node.find(s.key);
                if (it != node.end()) expand_from(*it, segs, i + 1, prefix + "/" + escape_pointer_key(s.key), out);
            } else if (node.is_array() && is_index(s.key)) {
                const std::size_t idx = static_cast<std::size_t>(std::stoul(s.key));
                if (idx < node.size()) expand_from(node[idx], segs, i + 1, prefix + "/" + s.key, out);
            }
            break;
        case Segment::Kind::Wildcard:
            if (node.is_array()) {
                for (std::size_t k = 0; k < node.size(); ++k) {
                    expand_from(node[k], segs, i + 1, prefix + "/" + std::to_string(k), out);
                }
            }
            break;
        case Segment::Kind::Filter:
            if (node.is_array()) {
                for (std::size_t k = 0; k < node.size(); ++k) {
                    const json& el = node[k];
                    if (!el.is_object()) continue;
                    const auto f = el.find(s.key);
                    if (f == el.end() || !field_matches(*f, s.value)) continue;
                    expand_from(el, segs, i + 1, prefix + "/" + std::to_string(k), out);
                }
            }
            break;
    }
}

// Concrete JSON pointers of every leaf the patterns reach in `doc`.
std::vector<std::string> expand_patterns(const json& doc, const std::vector<std::string>& patterns) {
    std::vector<std::string> out;
    for (const std::string& p : patterns) {
        std::vector<Segment> segs;
        if (!parse_pattern(p, segs, nullptr)) continue;
        expand_from(doc, segs, 0, std::string(), out);
    }
    return out;
}

const json* at_pointer(const json& doc, const std::string& ptr) {
    try {
        return &doc.at(json::json_pointer(ptr));
    } catch (...) {
        return nullptr;
    }
}

// ---- merge patch (RFC 7396) -----------------------------------------------------

json make_merge_patch(const json& base, const json& modified) {
    if (!base.is_object() || !modified.is_object()) return modified;
    json patch = json::object();
    for (auto it = base.begin(); it != base.end(); ++it) {
        const auto m = modified.find(it.key());
        if (m == modified.end()) {
            patch[it.key()] = nullptr;
        } else if (it.value() != *m) {
            patch[it.key()] = make_merge_patch(it.value(), *m);
        }
    }
    for (auto it = modified.begin(); it != modified.end(); ++it) {
        if (base.find(it.key()) == base.end()) patch[it.key()] = it.value();
    }
    return patch;
}

void collect_diffs(const json& a, const json& b, const std::string& prefix, std::vector<std::string>& out) {
    if (a.is_object() && b.is_object()) {
        for (auto it = a.begin(); it != a.end(); ++it) {
            const auto m = b.find(it.key());
            const std::string p = prefix + "/" + escape_pointer_key(it.key());
            if (m == b.end()) out.push_back(p);
            else collect_diffs(it.value(), *m, p, out);
        }
        for (auto it = b.begin(); it != b.end(); ++it) {
            if (a.find(it.key()) == a.end()) out.push_back(prefix + "/" + escape_pointer_key(it.key()));
        }
        return;
    }
    if (a.is_array() && b.is_array() && a.size() == b.size()) {
        for (std::size_t i = 0; i < a.size(); ++i) collect_diffs(a[i], b[i], prefix + "/" + std::to_string(i), out);
        return;
    }
    if (a != b) out.push_back(prefix.empty() ? "/" : prefix);
}

// ---- the files of a vehicle -----------------------------------------------------

struct FileDoc {
    std::string path; // absolute, normalised
    json base;
    json work;
};

struct Files {
    FileDoc vehicle;
    std::optional<FileDoc> gearbox;
    std::optional<FileDoc> engine;
    std::string original_engine_path;
    std::map<std::string, FileDoc> tyres; // by path
};

bool read_doc(const std::string& path, json& out, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        if (err != nullptr) *err = path + ": cannot open";
        return false;
    }
    auto parsed = detail::parse_json(*text);
    if (!parsed || !parsed->is_object()) {
        if (err != nullptr) *err = path + ": not a JSON object";
        return false;
    }
    out = std::move(*parsed);
    return true;
}

std::string dir_of(const std::string& path) { return fs::path(path).parent_path().generic_string(); }

std::string resolve_ref(const std::string& dir, const std::string& ref) {
    const fs::path p(ref);
    return norm_path(p.is_absolute() ? ref : (fs::path(dir) / p).generic_string());
}

// Index of the powertrain component of `type` in the vehicle doc, or -1.
int component_index(const json& vehicle, const char* type) {
    const auto pt = vehicle.find("powertrain");
    if (pt == vehicle.end() || !pt->is_object()) return -1;
    const auto comps = pt->find("components");
    if (comps == pt->end() || !comps->is_array()) return -1;
    for (std::size_t i = 0; i < comps->size(); ++i) {
        const json& c = (*comps)[i];
        std::string t;
        if (c.is_object() && detail::get_string(c, "type", t) && t == type && c.contains("ref")) return static_cast<int>(i);
    }
    return -1;
}

bool load_aux(const json& vehicle, const std::string& vehicle_dir, const char* type, std::optional<FileDoc>& out,
              std::string* err) {
    const int idx = component_index(vehicle, type);
    if (idx < 0) return true;
    std::string ref;
    detail::get_string(vehicle["powertrain"]["components"][static_cast<std::size_t>(idx)], "ref", ref);
    FileDoc f;
    f.path = resolve_ref(vehicle_dir, ref);
    if (!read_doc(f.path, f.base, err)) return false;
    f.work = f.base;
    out = std::move(f);
    return true;
}

// Re-reads the tyre files the (work) vehicle's wheels reference.
bool load_tyres(const FileDoc& vehicle, std::map<std::string, FileDoc>& out, std::string* err) {
    out.clear();
    const auto wheels = vehicle.work.find("wheels");
    if (wheels == vehicle.work.end() || !wheels->is_array()) return true;
    const std::string dir = dir_of(vehicle.path);
    for (const json& w : *wheels) {
        std::string tyre;
        if (!w.is_object() || !detail::get_string(w, "tyre", tyre)) continue;
        const std::string path = resolve_ref(dir, tyre);
        if (out.count(path) != 0) continue;
        FileDoc f;
        f.path = path;
        if (!read_doc(path, f.base, err)) return false;
        f.work = f.base;
        out[path] = std::move(f);
    }
    return true;
}

bool load_files(const CatalogEntry& entry, Files& files, std::string* err) {
    files.vehicle.path = norm_path(entry.vehicle_path);
    if (!read_doc(files.vehicle.path, files.vehicle.base, err)) return false;
    files.vehicle.work = files.vehicle.base;
    const std::string dir = dir_of(files.vehicle.path);
    if (!load_aux(files.vehicle.base, dir, "gearbox", files.gearbox, err)) return false;
    if (!load_aux(files.vehicle.base, dir, "engine", files.engine, err)) return false;
    if (files.engine) files.original_engine_path = files.engine->path;
    return load_tyres(files.vehicle, files.tyres, err);
}

// ---- options: the entry's view of the table ----------------------------------------

bool entry_offers(const CatalogEntry& entry, const std::string& id) {
    if (entry.setup_options.empty()) return true;
    return std::find(entry.setup_options.begin(), entry.setup_options.end(), id) != entry.setup_options.end();
}

std::vector<std::string> pointers_for(const CatalogEntry& entry, const SetupOptionDef& def) {
    const auto it = entry.setup_pointers.find(def.id);
    return it != entry.setup_pointers.end() ? it->second : def.pointers;
}

std::pair<double, double> range_for(const CatalogEntry& entry, const SetupOptionDef& def) {
    const auto it = entry.setup_ranges.find(def.id);
    if (it != entry.setup_ranges.end()) return it->second;
    return {def.min, def.max};
}

struct Target {
    FileDoc* file = nullptr;
    std::string pointer; // concrete JSON pointer in file->work
    std::string kind;    // "vehicle" | "gearbox" | "engine" | "tyre"
};

std::vector<Target> targets_of(Files& files, const SetupOptionDef& def, const std::vector<std::string>& patterns) {
    std::vector<Target> out;
    auto add = [&](FileDoc* f, const char* kind) {
        for (const std::string& p : expand_patterns(f->work, patterns)) out.push_back(Target{f, p, kind});
    };
    if (def.file == "vehicle") {
        add(&files.vehicle, "vehicle");
    } else if (def.file == "gearbox") {
        if (files.gearbox) add(&*files.gearbox, "gearbox");
    } else if (def.file == "engine") {
        if (files.engine) add(&*files.engine, "engine");
    } else if (def.file == "tyre") {
        for (auto& kv : files.tyres) add(&kv.second, "tyre");
    }
    return out;
}

std::string stem_of(const std::string& path) { return fs::path(path).stem().string(); }

// File choices are never a filesystem picker: only declared compatible IDs can resolve a path.
bool file_choice_fits(Files& files, const SetupOptionDef& def, const CatalogEntry& entry) {
    if (!entry_offers(entry, def.id)) return false;
    const std::string engine = stem_of(files.original_engine_path);
    if (std::find(def.compatible_engines.begin(), def.compatible_engines.end(), engine) == def.compatible_engines.end()) return false;
    const int idx = component_index(files.vehicle.base, "engine");
    if (def.no_turbo_override && idx >= 0 && files.vehicle.base["powertrain"]["components"][idx].contains("turbo_configuration")) return false;
    const json* base = def.file == "vehicle" ? &files.vehicle.base : (files.engine ? &files.engine->base : nullptr);
    return base && !expand_patterns(*base, pointers_for(entry, def)).empty();
}

std::string install_file_part(Files& files, const SetupOptionDef& def, const CatalogEntry& entry, const SetupValue& value) {
    if (!std::holds_alternative<std::string>(value)) return def.id + ": needs a component ID";
    if (!file_choice_fits(files, def, entry)) return def.id + ": not compatible with this vehicle";
    const auto& id = std::get<std::string>(value);
    if (id == "stock") return {};
    const auto part = std::find_if(def.parts.begin(), def.parts.end(), [&](const SetupPart& p) { return p.id == id; });
    if (part == def.parts.end()) return def.id + ": component ID is not in the fitment list";
    json doc;
    std::string err, format;
    if (!read_doc(part->path, doc, &err)) return err;
    if (part->naturally_aspirated) {
        if (def.part_format != "physics_sim.turbo_configuration/1" ||
            !detail::get_string(doc, "format", format) || format != "rg.induction_removal/1")
            return def.id + ": invalid naturally aspirated conversion";
        // Remove both sources: a vehicle override must not fall back to the engine default.
        if (files.engine) {
            files.engine->work.erase("default_turbo_configuration");
            // Boosted map nodes are unreachable without a compressor. Use the authored NA sampling grid.
            files.engine->work["cycle"]["grid_p_im_kpa"] = doc.at("grid_p_im_kpa");
        }
        const int engine_idx = component_index(files.vehicle.work, "engine");
        if (engine_idx >= 0) files.vehicle.work["powertrain"]["components"][engine_idx].erase("turbo_configuration");
        return {};
    }
    if (!detail::get_string(doc, "format", format) || format != def.part_format) return def.id + ": wrong component file format";
    for (auto& target : targets_of(files, def, pointers_for(entry, def))) {
        const json* leaf = at_pointer(target.file->work, target.pointer);
        if (!leaf || !leaf->is_string()) return def.id + ": reference is not a string";
        target.file->work[json::json_pointer(target.pointer)] = part->path;
    }
    // Reload the installed engine BEFORE calibrating it. Its own relative dependencies keep their directory.
    if (def.part_format == "physics_sim.engine/1") {
        if (!load_aux(files.vehicle.work, dir_of(files.vehicle.path), "engine", files.engine, &err)) return err;
    }
    return {};
}

// Wheels of an axle: {name, index, radius, width}.
struct WheelInfo {
    std::string name;
    std::size_t index = 0;
    double radius = 0.0;
    double width = 0.0;
    bool is_front = false;
    std::string tyre;
};

std::vector<WheelInfo> wheels_of(const json& vehicle, bool front) {
    std::vector<WheelInfo> out;
    const auto wheels = vehicle.find("wheels");
    if (wheels == vehicle.end() || !wheels->is_array()) return out;
    for (std::size_t i = 0; i < wheels->size(); ++i) {
        const json& w = (*wheels)[i];
        if (!w.is_object()) continue;
        WheelInfo info;
        info.index = i;
        info.is_front = false;
        detail::get_bool(w, "is_front", info.is_front);
        if (info.is_front != front) continue;
        detail::get_string(w, "name", info.name);
        detail::get_number(w, "wheel_radius", info.radius);
        detail::get_number(w, "wheel_width", info.width);
        detail::get_string(w, "tyre", info.tyre);
        out.push_back(info);
    }
    return out;
}

// Tyre files (by stem -> absolute path) whose size matches every wheel of `axle`.
std::map<std::string, std::string> tyre_candidates(const SetupContext& ctx, const std::vector<WheelInfo>& axle,
                                                    const std::string& stock_path,
                                                    const std::pair<double, double>* width_limits = nullptr) {
    std::map<std::string, std::string> out;
    if (axle.empty()) return out;
    constexpr double kTolM = 0.006;
    json stock_doc;
    double stock_rim = 0.0;
    if (read_doc(stock_path, stock_doc, nullptr)) detail::get_number(stock_doc, "rim_radius", stock_rim);
    std::error_code ec;
    for (const std::string& dir : ctx.tyre_dirs) {
        if (!fs::is_directory(fs::path(dir), ec)) continue;
        std::vector<fs::path> files;
        for (const auto& de : fs::directory_iterator(fs::path(dir), ec)) {
            if (de.is_regular_file() && de.path().extension() == ".json") files.push_back(de.path());
        }
        std::sort(files.begin(), files.end());
        for (const fs::path& f : files) {
            json doc;
            if (!read_doc(f.generic_string(), doc, nullptr)) continue;
            std::string format;
            if (!detail::get_string(doc, "format", format) || format.rfind("physics_sim.tyre", 0) != 0) continue;
            double radius = 0.0, width = 0.0;
            if (!detail::get_number(doc, "unloaded_radius", radius) || !detail::get_number(doc, "width", width)) continue;
            double rim = 0.0;
            detail::get_number(doc, "rim_radius", rim);
            bool fits = stock_rim <= 0.0 || std::abs(rim - stock_rim) < 0.001;
            for (const WheelInfo& w : axle) {
                if (std::abs(w.radius - radius) > kTolM) fits = false;
                if (width_limits != nullptr) {
                    const double mm = width * 1000.0;
                    if (mm < width_limits->first - 1e-6 || mm > width_limits->second + 1e-6) fits = false;
                } else if (std::abs(w.width - width) > kTolM) fits = false;
            }
            const std::string path = norm_path(f.generic_string());
            if (!fits && path != stock_path) continue;
            out.emplace(f.stem().string(), path); // first directory wins on a repeated stem
        }
    }
    return out;
}

// ---- option views ------------------------------------------------------------------

double front_share(const Files& files, const std::vector<std::string>& patterns, std::vector<Target>* targets_out,
                   Files* mutable_files) {
    (void)files;
    (void)patterns;
    (void)targets_out;
    (void)mutable_files;
    return 0.0;
}

// Brake torque of the targeted corner (leaf pointer ends in .../max_torque_nm).
struct BrakeCorner {
    Target target;
    double torque = 0.0;
    bool front = false;
};

bool brake_corners(Files& files, const std::vector<Target>& targets, std::vector<BrakeCorner>& out) {
    out.clear();
    for (const Target& t : targets) {
        const json* leaf = at_pointer(t.file->work, t.pointer);
        if (leaf == nullptr || !leaf->is_number()) return false;
        const std::size_t slash = t.pointer.rfind('/');
        const json* parent = at_pointer(t.file->work, t.pointer.substr(0, slash));
        std::string wheel;
        if (parent == nullptr || !parent->is_object() || !detail::get_string(*parent, "wheel", wheel)) return false;
        bool front = false;
        bool found = false;
        const auto wheels = files.vehicle.work.find("wheels");
        if (wheels != files.vehicle.work.end() && wheels->is_array()) {
            for (const json& w : *wheels) {
                std::string name;
                if (w.is_object() && detail::get_string(w, "name", name) && name == wheel) {
                    detail::get_bool(w, "is_front", front);
                    found = true;
                }
            }
        }
        if (!found) return false;
        BrakeCorner c;
        c.target = t;
        c.torque = leaf->get<double>();
        c.front = front;
        out.push_back(c);
    }
    return !out.empty();
}

constexpr double kEps = 1e-9;

} // namespace

// ---- public: small things ------------------------------------------------------------

const char* to_string(OptionKind k) {
    switch (k) {
        case OptionKind::Scale: return "scale";
        case OptionKind::ScaleList: return "scale_list";
        case OptionKind::BrakeBias: return "brake_bias";
        case OptionKind::FileChoice: return "file_choice";
        case OptionKind::TyreChoice: return "tyre_choice";
        case OptionKind::Bool: return "bool";
        case OptionKind::Colour: return "colour";
    }
    return "scale";
}

bool is_hex_colour(const std::string& s) {
    if (s.size() != 7 || s[0] != '#') return false;
    for (std::size_t i = 1; i < s.size(); ++i) {
        const char c = s[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) return false;
    }
    return true;
}

const SetupOptionDef* SetupOptionTable::find(const std::string& id) const {
    for (const SetupOptionDef& o : options) {
        if (o.id == id) return &o;
    }
    return nullptr;
}

std::optional<SetupOptionTable> parse_setup_options(const std::string& json_text, const std::string& origin,
                                                    std::string* err) {
    auto bad = [&](const std::string& message) -> std::optional<SetupOptionTable> {
        if (err != nullptr) *err = origin + ": " + message;
        return std::nullopt;
    };
    const std::optional<json> parsed = detail::parse_json(json_text);
    if (!parsed || !parsed->is_object()) return bad("not a JSON object");
    SetupOptionTable table;
    if (!detail::get_string(*parsed, "format", table.format) || table.format != "racing_game.vehicle_setup_options/1") {
        return bad("\"format\" must be \"racing_game.vehicle_setup_options/1\"");
    }
    const auto options = parsed->find("options");
    if (options == parsed->end() || !options->is_array() || options->empty()) return bad("\"options\" must be a non-empty array");
    std::set<std::string> seen;
    for (const json& o : *options) {
        if (!o.is_object()) return bad("an option is not an object");
        SetupOptionDef d;
        if (!detail::get_string(o, "id", d.id) || d.id.empty()) return bad("an option has no \"id\"");
        const std::string where = "option \"" + d.id + "\"";
        if (!seen.insert(d.id).second) return bad("duplicate " + where);
        detail::get_string(o, "label", d.label);
        if (d.label.empty()) d.label = d.id;
        detail::get_string(o, "group", d.group);
        detail::get_string(o, "area", d.area);
        detail::get_string(o, "help", d.help);
        detail::get_string(o, "unit", d.unit);
        std::string kind;
        if (!detail::get_string(o, "kind", kind)) return bad(where + ": \"kind\" missing");
        if (kind == "scale") d.kind = OptionKind::Scale;
        else if (kind == "scale_list") d.kind = OptionKind::ScaleList;
        else if (kind == "brake_bias") d.kind = OptionKind::BrakeBias;
        else if (kind == "file_choice") d.kind = OptionKind::FileChoice;
        else if (kind == "tyre_choice") d.kind = OptionKind::TyreChoice;
        else if (kind == "bool") d.kind = OptionKind::Bool;
        else if (kind == "colour") d.kind = OptionKind::Colour;
        else return bad(where + ": unknown kind \"" + kind + "\"");
        detail::get_number(o, "min", d.min);
        detail::get_number(o, "max", d.max);
        detail::get_number(o, "step", d.step);
        detail::get_number(o, "max_shift", d.max_shift);
        detail::get_string(o, "file", d.file);
        if (d.file != "vehicle" && d.file != "gearbox" && d.file != "engine" && d.file != "tyre") {
            return bad(where + ": \"file\" must be vehicle, gearbox, engine or tyre");
        }
        detail::get_string(o, "monotonic", d.monotonic);
        if (!d.monotonic.empty() && d.monotonic != "decreasing") return bad(where + ": \"monotonic\" must be \"decreasing\"");
        detail::get_string(o, "axle", d.axle);
        if (d.kind == OptionKind::TyreChoice && d.axle != "front" && d.axle != "rear") {
            return bad(where + ": a tyre choice needs \"axle\": front or rear");
        }
        const auto pointers = o.find("pointers");
        if (pointers != o.end()) {
            if (!pointers->is_array()) return bad(where + ": \"pointers\" must be an array");
            for (const json& p : *pointers) {
                if (!p.is_string()) return bad(where + ": \"pointers\" must hold strings");
                std::vector<Segment> segs;
                std::string perr;
                if (!parse_pattern(p.get<std::string>(), segs, &perr)) return bad(where + ": " + perr);
                d.pointers.push_back(p.get<std::string>());
            }
        }
        if (d.kind != OptionKind::Colour && d.pointers.empty()) return bad(where + ": \"pointers\" (the whitelist) is empty");
        if (d.kind == OptionKind::Colour && !d.pointers.empty()) return bad(where + ": a colour edits no file, so it has no pointers");
        if ((d.kind == OptionKind::Scale || d.kind == OptionKind::ScaleList) &&
            !(d.min > 0.0 && d.min <= 1.0 && d.max >= 1.0 && d.max < 10.0 && d.step > 0.0)) {
            return bad(where + ": a scale needs 0 < min <= 1 <= max < 10 and step > 0");
        }
        if (d.kind == OptionKind::BrakeBias && !(d.max_shift > 0.0 && d.max_shift < 0.5)) {
            return bad(where + ": max_shift must be in (0, 0.5)");
        }
        if (d.kind == OptionKind::FileChoice) {
            if (d.file != "vehicle" && d.file != "engine") return bad(where + ": file choice must edit vehicle or engine");
            if (!detail::get_string(o, "part_format", d.part_format) || d.part_format.empty()) return bad(where + ": part_format missing");
            detail::get_bool(o, "no_turbo_override", d.no_turbo_override);
            if (!o.contains("compatible_engines") || !o["compatible_engines"].is_array() || o["compatible_engines"].empty())
                return bad(where + ": explicit compatible_engines required");
            for (const auto& name : o["compatible_engines"]) {
                if (!name.is_string() || name.get<std::string>().empty()) return bad(where + ": invalid engine fitment");
                d.compatible_engines.push_back(name.get<std::string>());
            }
            if (!o.contains("parts") || !o["parts"].is_array() || o["parts"].empty()) return bad(where + ": parts missing");
            std::set<std::string> part_ids;
            for (const auto& part : o["parts"]) {
                SetupPart p;
                if (!part.is_object() || !detail::get_string(part, "id", p.id) || p.id.empty() || p.id == "stock" ||
                    !part_ids.insert(p.id).second || !detail::get_string(part, "path", p.path) || p.path.empty())
                    return bad(where + ": invalid or duplicate part");
                detail::get_string(part, "label", p.label);
                detail::get_string(part, "image", p.image);
                detail::get_string(part, "detail", p.detail);
                detail::get_bool(part, "naturally_aspirated", p.naturally_aspirated);
                p.path = resolve_ref(dir_of(origin), p.path);
                d.parts.push_back(std::move(p));
            }
        }
        table.options.push_back(std::move(d));
    }
    return table;
}

std::optional<SetupOptionTable> load_setup_options(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        if (err != nullptr) *err = path + ": cannot open";
        return std::nullopt;
    }
    return parse_setup_options(*text, path, err);
}

// ---- setup documents -----------------------------------------------------------------------

std::string setup_to_json(const VehicleSetup& setup) {
    json root = json::object();
    root["format"] = "racing_game.vehicle_setup/1";
    root["vehicle"] = setup.vehicle_id;
    json values = json::object();
    for (const auto& kv : setup.values) {
        if (std::holds_alternative<double>(kv.second)) values[kv.first] = std::get<double>(kv.second);
        else if (std::holds_alternative<bool>(kv.second)) values[kv.first] = std::get<bool>(kv.second);
        else if (std::holds_alternative<std::string>(kv.second)) values[kv.first] = std::get<std::string>(kv.second);
        else values[kv.first] = std::get<std::vector<double>>(kv.second);
    }
    root["options"] = std::move(values);
    return root.dump(2) + "\n";
}

std::optional<VehicleSetup> parse_setup(const std::string& json_text, const SetupOptionTable& table,
                                        const std::string& origin, std::string* err) {
    auto bad = [&](const std::string& message) -> std::optional<VehicleSetup> {
        if (err != nullptr) *err = origin + ": " + message;
        return std::nullopt;
    };
    const std::optional<json> parsed = detail::parse_json(json_text);
    if (!parsed || !parsed->is_object()) return bad("not a JSON object");
    std::string format;
    if (!detail::get_string(*parsed, "format", format) || format != "racing_game.vehicle_setup/1") {
        return bad("\"format\" must be \"racing_game.vehicle_setup/1\"");
    }
    VehicleSetup out;
    if (!detail::get_string(*parsed, "vehicle", out.vehicle_id) || out.vehicle_id.empty()) return bad("\"vehicle\" missing");
    const auto options = parsed->find("options");
    if (options == parsed->end() || !options->is_object()) return bad("\"options\" must be an object");
    for (auto it = options->begin(); it != options->end(); ++it) {
        const SetupOptionDef* def = table.find(it.key());
        if (def == nullptr) return bad("unknown option \"" + it.key() + "\"");
        const json& v = it.value();
        switch (def->kind) {
            case OptionKind::Scale:
            case OptionKind::BrakeBias:
                if (!v.is_number() || !std::isfinite(v.get<double>())) return bad("option \"" + it.key() + "\" must be a number");
                out.values[it.key()] = v.get<double>();
                break;
            case OptionKind::ScaleList: {
                if (!v.is_array() || v.empty()) return bad("option \"" + it.key() + "\" must be an array of numbers");
                std::vector<double> list;
                for (const json& e : v) {
                    if (!e.is_number() || !std::isfinite(e.get<double>())) return bad("option \"" + it.key() + "\" must be an array of numbers");
                    list.push_back(e.get<double>());
                }
                out.values[it.key()] = std::move(list);
                break;
            }
            case OptionKind::Bool:
                if (!v.is_boolean()) return bad("option \"" + it.key() + "\" must be true or false");
                out.values[it.key()] = v.get<bool>();
                break;
            case OptionKind::FileChoice:
            case OptionKind::TyreChoice:
            case OptionKind::Colour:
                if (!v.is_string()) return bad("option \"" + it.key() + "\" must be a string");
                out.values[it.key()] = v.get<std::string>();
                break;
        }
    }
    return out;
}

// ---- merge patch / whitelist (exposed) --------------------------------------------------------

std::string apply_merge_patch(const std::string& base_json, const std::string& patch_json) {
    auto base = detail::parse_json(base_json);
    auto patch = detail::parse_json(patch_json);
    if (!base || !patch) return {};
    base->merge_patch(*patch);
    return base->dump();
}

std::string whitelist_violation(const std::string& base_json, const std::string& patch_json,
                                const std::vector<std::string>& pointer_patterns) {
    auto base = detail::parse_json(base_json);
    auto patch = detail::parse_json(patch_json);
    if (!base || !patch) return "(unparsable)";
    json merged = *base;
    merged.merge_patch(*patch);
    std::vector<std::string> diffs;
    collect_diffs(*base, merged, std::string(), diffs);
    const std::vector<std::string> allowed_list = expand_patterns(*base, pointer_patterns);
    const std::set<std::string> allowed(allowed_list.begin(), allowed_list.end());
    for (const std::string& d : diffs) {
        if (allowed.count(d) == 0) return d;
    }
    return {};
}

// ---- the model (option views) ------------------------------------------------------------------

SetupModel build_setup_model(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx, const VehicleSetup* installed) {
    SetupModel model;
    Files files;
    if (!load_files(entry, files, &model.error)) return model;
    if (installed) {
        for (const auto& def : table.options) {
            if (def.kind != OptionKind::FileChoice) continue;
            const auto it = installed->values.find(def.id);
            if (it == installed->values.end()) continue;
            model.error = install_file_part(files, def, entry, it->second);
            if (!model.error.empty()) return model;
        }
    }
    for (const SetupOptionDef& def : table.options) {
        OptionView v;
        v.def = def;
        v.def.pointers = pointers_for(entry, def);
        const auto range = range_for(entry, def);
        v.min = range.first;
        v.max = range.second;
        v.available = entry_offers(entry, def.id);
        switch (def.kind) {
            case OptionKind::Scale: {
                v.stock = 1.0;
                const auto targets = targets_of(files, def, v.def.pointers);
                for (const auto& target : targets) {
                    const auto* leaf = at_pointer(target.file->base, target.pointer);
                    if (leaf && leaf->is_number()) v.stock_numbers.push_back(leaf->get<double>());
                }
                if (v.available && targets.empty()) v.available = false;
                break;
            }
            case OptionKind::ScaleList: {
                std::vector<Target> t = targets_of(files, def, v.def.pointers);
                v.list_size = static_cast<int>(t.size());
                v.stock = std::vector<double>(t.size(), 1.0);
                for (const Target& tg : t) {
                    const json* leaf = at_pointer(tg.file->work, tg.pointer);
                    v.stock_numbers.push_back(leaf != nullptr && leaf->is_number() ? leaf->get<double>() : 0.0);
                }
                if (t.empty()) v.available = false;
                break;
            }
            case OptionKind::BrakeBias: {
                std::vector<BrakeCorner> corners;
                const std::vector<Target> t = targets_of(files, def, v.def.pointers);
                double front = 0.0, total = 0.0;
                if (brake_corners(files, t, corners)) {
                    for (const BrakeCorner& c : corners) {
                        total += c.torque;
                        if (c.front) front += c.torque;
                    }
                }
                const double share = total > 0.0 ? front / total : 0.5;
                v.stock = share;
                if (entry.setup_ranges.count(def.id) == 0) {
                    v.min = std::max(0.2, share - def.max_shift);
                    v.max = std::min(0.85, share + def.max_shift);
                }
                if (total <= 0.0) v.available = false;
                break;
            }
            case OptionKind::FileChoice: {
                v.stock = std::string("stock");
                v.available = file_choice_fits(files, def, entry);
                SetupPart stock;
                stock.id = "stock";
                stock.label = "Original component";
                std::string original;
                const FileDoc* original_file = def.file == "vehicle" ? &files.vehicle : (files.engine ? &*files.engine : nullptr);
                if (original_file) {
                    const auto pointers = expand_patterns(original_file->base, v.def.pointers);
                    if (!pointers.empty()) {
                        const auto* leaf = at_pointer(original_file->base, pointers.front());
                        if (leaf && leaf->is_string()) original = resolve_ref(dir_of(original_file->path), leaf->get<std::string>());
                    }
                }
                for (const auto& part : def.parts) if (part.path == original) { stock = part; stock.id = "stock"; }
                v.parts.push_back(stock);
                v.choices.push_back("stock");
                for (const auto& part : def.parts) {
                    if (part.path == original) continue;
                    v.parts.push_back(part);
                    v.choices.push_back(part.id);
                }
                if (v.choices.size() < 2) v.available = false;
                break;
            }
            case OptionKind::TyreChoice: {
                const std::vector<WheelInfo> axle = wheels_of(files.vehicle.work, def.axle == "front");
                std::string stock_path;
                if (!axle.empty() && !axle.front().tyre.empty()) {
                    stock_path = resolve_ref(dir_of(files.vehicle.path), axle.front().tyre);
                }
                v.stock = stem_of(stock_path);
                const auto limits = entry.setup_ranges.find(def.id);
                for (const auto& kv : tyre_candidates(ctx, axle, stock_path, limits == entry.setup_ranges.end() ? nullptr : &limits->second)) v.choices.push_back(kv.first);
                if (axle.empty() || stock_path.empty() || v.choices.size() < 2) v.available = false;
                break;
            }
            case OptionKind::Bool: {
                const std::vector<Target> t = targets_of(files, def, v.def.pointers);
                bool value = false;
                if (!t.empty()) {
                    const json* leaf = at_pointer(t.front().file->work, t.front().pointer);
                    if (leaf != nullptr && leaf->is_boolean()) value = leaf->get<bool>();
                }
                v.stock = value;
                if (t.empty()) v.available = false;
                break;
            }
            case OptionKind::Colour:
                v.stock = def.id == "rim" ? entry.default_rim : entry.default_paint;
                break;
        }
        model.options.push_back(std::move(v));
    }
    return model;
}

// ---- compile ------------------------------------------------------------------------------------

namespace {

bool values_equal(const SetupValue& a, const SetupValue& b) {
    if (a.index() != b.index()) return false;
    if (std::holds_alternative<double>(a)) return std::abs(std::get<double>(a) - std::get<double>(b)) < kEps;
    if (std::holds_alternative<bool>(a)) return std::get<bool>(a) == std::get<bool>(b);
    if (std::holds_alternative<std::string>(a)) return std::get<std::string>(a) == std::get<std::string>(b);
    const auto& x = std::get<std::vector<double>>(a);
    const auto& y = std::get<std::vector<double>>(b);
    if (x.size() != y.size()) return false;
    for (std::size_t i = 0; i < x.size(); ++i) {
        if (std::abs(x[i] - y[i]) >= kEps) return false;
    }
    return true;
}

struct CompileState {
    Files files;
    CompiledSetup out;
    std::set<std::string> allowed_vehicle, allowed_gearbox, allowed_engine;
    std::map<std::string, std::set<std::string>> allowed_tyre; // by path
};

bool set_number(Target& t, double value) {
    try {
        t.file->work[json::json_pointer(t.pointer)] = value;
    } catch (...) {
        return false;
    }
    return true;
}

} // namespace

bool setup_values_equal(const SetupValue& a, const SetupValue& b) { return values_equal(a, b); }

CompiledSetup compile_setup_internal(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx,
                                     const VehicleSetup& setup, Files& files);

namespace {

// Applies one option's value to `files`; "" when fine, else the broken rule.
std::string apply_option(const CatalogEntry& entry, const SetupOptionDef& def, const SetupValue& value,
                         const SetupContext& ctx, Files& files, CompiledSetup& out,
                         std::vector<std::pair<const FileDoc*, std::string>>& touched) {
    const std::string id = def.id;
    if (!entry_offers(entry, id)) return "option \"" + id + "\" is not offered for " + entry.id;
    const std::vector<std::string> patterns = pointers_for(entry, def);
    const auto range = range_for(entry, def);
    const auto type_error = [&](const char* expected) { return "option \"" + id + "\" needs " + expected; };

    switch (def.kind) {
        case OptionKind::FileChoice: return install_file_part(files, def, entry, value);
        case OptionKind::Scale: {
            if (!std::holds_alternative<double>(value)) return type_error("a number");
            const double s = std::get<double>(value);
            if (!std::isfinite(s) || s < range.first - kEps || s > range.second + kEps) {
                return id + ": " + fmt2(s) + " is outside the allowed range " + fmt2(range.first) + " to " + fmt2(range.second);
            }
            std::vector<Target> targets = targets_of(files, def, patterns);
            if (targets.empty()) return "option \"" + id + "\" matches nothing in " + entry.id + "'s files";
            for (Target& t : targets) {
                const json* leaf = at_pointer(t.file->work, t.pointer);
                if (leaf == nullptr || !leaf->is_number()) return "option \"" + id + "\": " + t.pointer + " is not a number";
                if (!set_number(t, leaf->get<double>() * s)) return "option \"" + id + "\": cannot edit " + t.pointer;
                touched.emplace_back(t.file, t.pointer);
            }
            return {};
        }
        case OptionKind::ScaleList: {
            if (!std::holds_alternative<std::vector<double>>(value)) return type_error("a list of numbers");
            const auto& list = std::get<std::vector<double>>(value);
            std::vector<Target> targets = targets_of(files, def, patterns);
            if (targets.empty()) return "option \"" + id + "\" matches nothing in " + entry.id + "'s files";
            if (list.size() != targets.size()) {
                return id + ": needs " + std::to_string(targets.size()) + " values, got " + std::to_string(list.size());
            }
            std::vector<double> result;
            for (std::size_t i = 0; i < targets.size(); ++i) {
                const double s = list[i];
                if (!std::isfinite(s) || s < range.first - kEps || s > range.second + kEps) {
                    return id + " (#" + std::to_string(i + 1) + "): " + fmt2(s) + " is outside the allowed range " +
                           fmt2(range.first) + " to " + fmt2(range.second);
                }
                const json* leaf = at_pointer(targets[i].file->work, targets[i].pointer);
                if (leaf == nullptr || !leaf->is_number()) return "option \"" + id + "\": " + targets[i].pointer + " is not a number";
                result.push_back(leaf->get<double>() * s);
            }
            if (def.monotonic == "decreasing") {
                for (std::size_t i = 1; i < result.size(); ++i) {
                    if (!(result[i] < result[i - 1])) {
                        return id + ": the ratios must keep getting lower with every gear (gear " + std::to_string(i + 1) +
                               " would be " + fmt2(result[i]) + ", gear " + std::to_string(i) + " is " + fmt2(result[i - 1]) + ")";
                    }
                }
            }
            for (std::size_t i = 0; i < targets.size(); ++i) {
                if (!set_number(targets[i], result[i])) return "option \"" + id + "\": cannot edit " + targets[i].pointer;
                touched.emplace_back(targets[i].file, targets[i].pointer);
            }
            return {};
        }
        case OptionKind::BrakeBias: {
            if (!std::holds_alternative<double>(value)) return type_error("a number");
            const double b = std::get<double>(value);
            std::vector<Target> targets = targets_of(files, def, patterns);
            std::vector<BrakeCorner> corners;
            if (targets.empty() || !brake_corners(files, targets, corners)) {
                return "option \"" + id + "\" needs brake corners that name their wheel";
            }
            double front = 0.0, rear = 0.0;
            for (const BrakeCorner& c : corners) (c.front ? front : rear) += c.torque;
            if (front <= 0.0 || rear <= 0.0) return "option \"" + id + "\" needs front and rear brake corners";
            const double total = front + rear;
            const double stock = front / total;
            double lo = range.first, hi = range.second;
            if (entry.setup_ranges.count(def.id) == 0) {
                lo = std::max(0.2, stock - def.max_shift);
                hi = std::min(0.85, stock + def.max_shift);
            }
            if (!std::isfinite(b) || b < lo - kEps || b > hi + kEps) {
                return id + ": a front share of " + fmt2(b * 100.0) + " % is outside the allowed range " + fmt2(lo * 100.0) +
                       " % to " + fmt2(hi * 100.0) + " %";
            }
            const double kf = b * total / front;
            const double kr = (1.0 - b) * total / rear;
            for (BrakeCorner& c : corners) {
                if (!set_number(c.target, c.torque * (c.front ? kf : kr))) return "option \"" + id + "\": cannot edit " + c.target.pointer;
                touched.emplace_back(c.target.file, c.target.pointer);
            }
            return {};
        }
        case OptionKind::TyreChoice: {
            if (!std::holds_alternative<std::string>(value)) return type_error("a tyre name");
            const std::string& stem = std::get<std::string>(value);
            const std::vector<WheelInfo> axle = wheels_of(files.vehicle.work, def.axle == "front");
            std::string stock_path;
            if (!axle.empty() && !axle.front().tyre.empty()) stock_path = resolve_ref(dir_of(files.vehicle.path), axle.front().tyre);
            const auto limits = entry.setup_ranges.find(def.id);
            const auto candidates = tyre_candidates(ctx, axle, stock_path, limits == entry.setup_ranges.end() ? nullptr : &limits->second);
            const auto it = candidates.find(stem);
            if (it == candidates.end()) {
                return id + ": \"" + stem + "\" is not a tyre that fits the " + def.axle + " wheels";
            }
            std::vector<std::string> ref_patterns;
            for (const auto& pattern : patterns) if (pattern.size() >= 5 && pattern.substr(pattern.size()-5) == "/tyre") ref_patterns.push_back(pattern);
            std::vector<Target> targets = targets_of(files, def, ref_patterns);
            if (targets.empty()) return "option \"" + id + "\" matches nothing in " + entry.id + "'s files";
            for (Target& t : targets) {
                try {
                    t.file->work[json::json_pointer(t.pointer)] = it->second;
                } catch (...) {
                    return "option \"" + id + "\": cannot edit " + t.pointer;
                }
                touched.emplace_back(t.file, t.pointer);
            }
            json selected_doc;
            double selected_width = 0.0;
            if (!read_doc(it->second, selected_doc, nullptr) || !detail::get_number(selected_doc, "width", selected_width)) return id + ": tyre width unavailable";
            for (const auto& wheel : axle) {
                auto& target = files.vehicle.work["wheels"][wheel.index];
                if (wheel.width <= 0.0) return id + ": invalid original wheel width";
                target["wheel_width"] = selected_width;
                if (target.contains("wheel_inertia") && target["wheel_inertia"].is_number())
                    target["wheel_inertia"] = target["wheel_inertia"].get<double>() * selected_width / wheel.width;
            }
            return {};
        }
        case OptionKind::Bool: {
            if (!std::holds_alternative<bool>(value)) return type_error("true or false");
            std::vector<Target> targets = targets_of(files, def, patterns);
            if (targets.empty()) return "option \"" + id + "\" matches nothing in " + entry.id + "'s files";
            for (Target& t : targets) {
                const json* leaf = at_pointer(t.file->work, t.pointer);
                if (leaf == nullptr || !leaf->is_boolean()) return "option \"" + id + "\": " + t.pointer + " is not true/false";
                try {
                    t.file->work[json::json_pointer(t.pointer)] = std::get<bool>(value);
                } catch (...) {
                    return "option \"" + id + "\": cannot edit " + t.pointer;
                }
                touched.emplace_back(t.file, t.pointer);
            }
            return {};
        }
        case OptionKind::Colour: {
            if (!std::holds_alternative<std::string>(value) || !is_hex_colour(std::get<std::string>(value))) {
                return id + ": a colour must look like #rrggbb";
            }
            (id == "rim" ? out.rim : out.paint) = std::get<std::string>(value);
            return {};
        }
    }
    return {};
}

std::string patch_text(const json& patch) { return patch.dump(); }

std::string basename_key(const std::string& kind, const FileDoc& f) { return kind + ":" + stem_of(f.path); }

} // namespace

CompiledSetup compile_setup(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx,
                            const VehicleSetup& setup) {
    Files files;
    return compile_setup_internal(entry, table, ctx, setup, files);
}

CompiledSetup compile_setup_internal(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx,
                                     const VehicleSetup& setup, Files& files) {
    CompiledSetup out;
    out.vehicle_patch = "{}";
    if (setup.vehicle_id != entry.id) {
        out.error = "the setup is for \"" + setup.vehicle_id + "\", not " + entry.id;
        return out;
    }
    std::string err;
    if (!load_files(entry, files, &err)) {
        out.error = err;
        return out;
    }
    // Unknown option ids are an error: a typo must not silently do nothing.
    for (const auto& kv : setup.values) {
        if (table.find(kv.first) == nullptr) {
            out.error = "unknown option \"" + kv.first + "\"";
            return out;
        }
    }
    // The stock values, to skip options set back to what they already are.
    const SetupModel model = build_setup_model(entry, table, ctx);
    if (!model.error.empty()) {
        out.error = model.error;
        return out;
    }
    auto is_stock = [&](const SetupOptionDef& def, const SetupValue& value) {
        for (const OptionView& v : model.options) {
            if (v.def.id == def.id) return v.available && values_equal(v.stock, value);
        }
        return false;
    };

    std::vector<std::pair<const FileDoc*, std::string>> touched; // (file, pointer) per edit
    // Pass 1: tyre choices change which tyre files the wheels use.
    for (const SetupOptionDef& def : table.options) {
        if (def.kind != OptionKind::TyreChoice && def.kind != OptionKind::FileChoice) continue;
        const auto it = setup.values.find(def.id);
        if (it == setup.values.end() || is_stock(def, it->second)) continue;
        const std::string e = apply_option(entry, def, it->second, ctx, files, out, touched);
        if (!e.empty()) {
            out.error = e;
            return out;
        }
    }
    if (!load_tyres(files.vehicle, files.tyres, &err)) {
        out.error = err;
        return out;
    }
    // Pass 2: everything else, in table order.
    for (const SetupOptionDef& def : table.options) {
        if (def.kind == OptionKind::TyreChoice || def.kind == OptionKind::FileChoice) continue;
        const auto it = setup.values.find(def.id);
        if (it == setup.values.end()) continue;
        if (def.kind != OptionKind::Colour && is_stock(def, it->second)) continue;
        if (def.kind == OptionKind::Colour && !std::holds_alternative<std::string>(it->second)) {
            out.error = def.id + ": a colour must look like #rrggbb";
            return out;
        }
        const std::string e = apply_option(entry, def, it->second, ctx, files, out, touched);
        if (!e.empty()) {
            out.error = e;
            return out;
        }
    }

    // Merge patches (the mechanism) and the whitelist check on the result.
    auto finish_file = [&](const char* kind, const FileDoc& f, bool is_vehicle) -> bool {
        const json patch = make_merge_patch(f.base, f.work);
        json merged = f.base;
        merged.merge_patch(patch);
        if (merged != f.work) {
            out.error = std::string("internal: the merge patch for the ") + kind + " file does not reproduce the edit";
            return false;
        }
        if (patch.is_object() && patch.empty()) return true;
        // Whitelist: every changed leaf must be one an enabled option may touch.
        std::vector<std::string> patterns;
        for (const SetupOptionDef& def : table.options) {
            if (!entry_offers(entry, def.id) || def.kind == OptionKind::Colour) continue;
            const std::string want = is_vehicle ? "vehicle" : kind;
            if (def.file != want) continue;
            for (const std::string& p : pointers_for(entry, def)) patterns.push_back(p);
        }
        if (std::string(kind) == "engine") {
            for (const auto& def : table.options) {
                const auto selected = setup.values.find(def.id);
                if (!entry_offers(entry, def.id) || selected == setup.values.end() || !std::holds_alternative<std::string>(selected->second)) continue;
                for (const auto& part : def.parts) if (part.naturally_aspirated && part.id == std::get<std::string>(selected->second)) {
                    patterns.push_back("/default_turbo_configuration");
                    patterns.push_back("/cycle/grid_p_im_kpa");
                }
            }
        }
        const std::string violation = whitelist_violation(f.base.dump(), patch_text(patch), patterns);
        if (!violation.empty()) {
            out.error = "the setup would change " + violation + " in the " + kind + " file, which no option may touch";
            return false;
        }
        if (is_vehicle) out.vehicle_patch = patch_text(patch);
        else out.patches[basename_key(kind, f)] = patch_text(patch);
        return true;
    };
    if (!finish_file("vehicle", files.vehicle, true)) return out;
    if (files.gearbox && !finish_file("gearbox", *files.gearbox, false)) return out;
    if (files.engine && !finish_file("engine", *files.engine, false)) return out;
    for (const auto& kv : files.tyres) {
        if (!finish_file("tyre", kv.second, false)) return out;
    }
    out.ok = true;
    return out;
}

// ---- materialise ---------------------------------------------------------------------------------

namespace {

bool looks_like_path(const std::string& s) {
    if (s.empty() || s.size() > 400) return false;
    if (s.find('/') != std::string::npos || s.find('\\') != std::string::npos) return true;
    const auto ends = [&](const char* suffix) {
        const std::string t(suffix);
        return s.size() > t.size() && s.compare(s.size() - t.size(), t.size(), t) == 0;
    };
    return ends(".json") || ends(".tir");
}

// Every string value that names an existing file (relative to `dir`) becomes
// that file's absolute path. "source" texts are prose, never references.
void absolutise_refs(json& node, const std::string& dir) {
    if (node.is_object()) {
        for (auto it = node.begin(); it != node.end(); ++it) {
            if (it.key() == "source") continue;
            absolutise_refs(it.value(), dir);
        }
    } else if (node.is_array()) {
        for (json& e : node) absolutise_refs(e, dir);
    } else if (node.is_string()) {
        const std::string s = node.get<std::string>();
        if (!looks_like_path(s)) return;
        const fs::path p(s);
        const fs::path full = p.is_absolute() ? p : fs::path(dir) / p;
        std::error_code ec;
        if (fs::is_regular_file(full, ec)) node = norm_path(full.generic_string());
    }
}

void replace_strings(json& node, const std::map<std::string, std::string>& map) {
    if (node.is_object()) {
        for (auto it = node.begin(); it != node.end(); ++it) replace_strings(it.value(), map);
    } else if (node.is_array()) {
        for (json& e : node) replace_strings(e, map);
    } else if (node.is_string()) {
        const auto it = map.find(node.get<std::string>());
        if (it != map.end()) node = it->second;
    }
}

bool write_text(const std::string& path, const std::string& text, std::string* err) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        if (err != nullptr) *err = path + ": cannot write";
        return false;
    }
    out << text;
    out.flush();
    if (!out) {
        if (err != nullptr) *err = path + ": write failed";
        return false;
    }
    return true;
}

} // namespace

MaterialisedSetup materialise_setup(const CatalogEntry& entry, const SetupOptionTable& table,
                                    const SetupContext& ctx, const VehicleSetup& setup, const std::string& out_dir) {
    MaterialisedSetup out;
    Files files;
    out.compiled = compile_setup_internal(entry, table, ctx, setup, files);
    if (!out.compiled.ok) {
        out.error = out.compiled.error;
        return out;
    }
    out.dir = out_dir;
    // Nothing to patch: the original file is the vehicle (no copy, no temp files).
    if (out.compiled.vehicle_patch == "{}" && out.compiled.patches.empty()) {
        out.ok = true;
        out.vehicle_path = files.vehicle.path;
        return out;
    }
    std::error_code ec;
    fs::create_directories(fs::path(out_dir), ec);
    if (ec) {
        out.error = out_dir + ": cannot create (" + ec.message() + ")";
        return out;
    }
    std::map<std::string, std::string> moved; // original absolute path -> materialised path
    auto write_aux = [&](const char* kind, const FileDoc& f) -> bool {
        const json patch = make_merge_patch(f.base, f.work);
        if (patch.is_object() && patch.empty()) return true;
        json doc = f.base;
        doc.merge_patch(patch);
        absolutise_refs(doc, dir_of(f.path));
        const std::string path = norm_path((fs::path(out_dir) / (std::string(kind) + "_" + stem_of(f.path) + ".json")).generic_string());
        if (!write_text(path, doc.dump(2) + "\n", &out.error)) return false;
        out.files.push_back(path);
        moved[f.path] = path;
        return true;
    };
    if (files.gearbox && !write_aux("gearbox", *files.gearbox)) return out;
    if (files.engine && !write_aux("engine", *files.engine)) return out;
    for (const auto& kv : files.tyres) {
        if (!write_aux("tyre", kv.second)) return out;
    }
    json vehicle = files.vehicle.base;
    vehicle.merge_patch(*detail::parse_json(out.compiled.vehicle_patch));
    absolutise_refs(vehicle, dir_of(files.vehicle.path));
    replace_strings(vehicle, moved);
    const std::string vehicle_path =
        norm_path((fs::path(out_dir) / fs::path(files.vehicle.path).filename()).generic_string());
    if (!write_text(vehicle_path, vehicle.dump(2) + "\n", &out.error)) return out;
    out.files.push_back(vehicle_path);
    out.vehicle_path = vehicle_path;
    out.ok = true;
    return out;
}

// ---- validate ---------------------------------------------------------------------------------------

namespace {
std::atomic<unsigned> g_work_counter{0};
}

ScopedWorkDir::ScopedWorkDir(const std::string& root, const std::string& prefix) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const unsigned n = g_work_counter.fetch_add(1);
    const fs::path p = fs::path(root) / (prefix + "_" + std::to_string(static_cast<unsigned long long>(stamp) % 100000000ULL) + "_" + std::to_string(n));
    std::error_code ec;
    fs::create_directories(p, ec);
    path_ = p.generic_string();
}

ScopedWorkDir::~ScopedWorkDir() {
    std::error_code ec;
    if (!path_.empty()) fs::remove_all(fs::path(path_), ec);
}

std::size_t ScopedWorkDir::file_count(const std::string& dir) {
    std::error_code ec;
    if (!fs::is_directory(fs::path(dir), ec)) return 0;
    std::size_t n = 0;
    for (auto it = fs::recursive_directory_iterator(fs::path(dir), ec); !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
        if (it->is_regular_file(ec)) ++n;
    }
    return n;
}

ValidationResult validate_setup(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx,
                                const VehicleSetup& setup) {
    ValidationResult result;
    ScopedWorkDir scratch(ctx.work_root, "validate_" + entry.id);
    const MaterialisedSetup m = materialise_setup(entry, table, ctx, setup, scratch.path());
    if (!m.ok) {
        result.message = m.error;
        return result;
    }
    try {
        ps::io::EngineMapOptions options;
        options.cache_dir = ctx.engine_map_cache_dir;
        (void)ps::io::load_vehicle_json(m.vehicle_path, options);
    } catch (const std::exception& e) {
        result.message = e.what();
        return result;
    } catch (...) {
        result.message = "the vehicle loader failed with an unknown error";
        return result;
    }
    result.ok = true;
    return result;
}

} // namespace rg
