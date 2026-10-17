// rg/vehicle_setup.cpp - see vehicle_setup.h.
#include "rg/vehicle_setup.h"

#include "rg/engine_swap.h"
#include "rg/tyre_spec.h"
#include "rg/vehicle_load.h"

#include "driveline_swap_json.h"
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

std::string fmt_fixed(double v, int decimals) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
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
    std::optional<FileDoc> turbo;
    std::string original_engine_path;
    // Engine garage G1. The engine whose options (turbo, nitrous) apply is the INSTALLED one: engine_stem is the
    // catalog stem of the default engine it is (a user engine: its base), the car's own at load.
    std::string engine_stem;
    std::optional<EngineEntry> swapped; // set when engine_install put another engine in
    std::string fuel_choice;            // set when the fuel option picked a fuel
    // Vehicle-file subtrees the swap package (trusted code, not a player option) edited; exempt from the whitelist.
    std::vector<std::string> derived_subtrees;
    // Electronics parts the swap package re-targeted (a launch rpm outside the new engine's idle..limiter band).
    std::vector<FileDoc> electronics;
    // A garage turbo kit fitted to an engine without a turbo (catalog "turbo_kits"): its label and torque factor.
    std::string kit_label;
    double kit_torque_factor = 1.0;
    std::optional<TurboKitSpec> kit_spec; // the fitted kit (count, and the target it is sized for now)
    std::string kit_piping;               // charge piping choice ("" = the kit's default)
    // Engine internals (catalog "engine_internals"): part kind -> grade id; a kind not in the map is stock.
    std::map<std::string, std::string> internals;
    // Parts a setup feature adds to the part-mass tree beyond what the vehicle file names (rg/part_mass.h).
    std::vector<PartAttachment> part_extras;
    // Cooling system upgrades (catalog "cooling_upgrades"): part kind -> grade id; a kind not in the map is stock. The
    // cooling_system file and the engine thermal file (refresh_cooling) are loaded as docs whose `work` the grades edit; the
    // EFFECTIVE json (extends resolved) is what the grades read their numbers from.
    std::map<std::string, std::string> cooling_grades;
    std::optional<FileDoc> cooling;
    std::optional<FileDoc> thermal;
    nlohmann::json cooling_effective, thermal_effective;
    std::string cooling_blocker;             // "" = the car has a cooling model the options can work on
    std::optional<CoolingBuild> cooling_build;
    // Drivetrain swap: the driveline right after a drive layout choice (before an axle differential choice); unset =
    // the car's own. The differential options show the axle's kind here as their stock.
    std::optional<DrivelineInfo> layout_info;
    // The removable / swappable parts as fitted (catalog "interior", then the part choices); a removed part is absent.
    std::vector<FittedPart> trim;
    // Automatic gearbox swap: what installing a clutch-to-clutch box changed on the car (for the player).
    std::vector<std::string> gearbox_lines;
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
std::string stem_of_path(const std::string& path) { return fs::path(path).stem().string(); }

std::string resolve_ref(const std::string& dir, const std::string& ref) {
    const fs::path p(ref);
    return norm_path(p.is_absolute() ? ref : (fs::path(dir) / p).generic_string());
}

// Index of the powertrain component of `type` in the vehicle doc, or -1.
int controller_of_type(const json& vehicle, const char* type);

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

bool load_turbo(Files& files, std::string* err) {
    files.turbo.reset();
    if (!files.engine) return true;
    std::string ref, dir = dir_of(files.engine->path);
    detail::get_string(files.engine->work, "default_forced_induction", ref);
    const int idx = component_index(files.vehicle.work, "engine");
    if (idx >= 0 && files.vehicle.work["powertrain"]["components"][idx].contains("forced_induction")) {
        detail::get_string(files.vehicle.work["powertrain"]["components"][idx], "forced_induction", ref);
        dir = dir_of(files.vehicle.path);
    }
    if (ref.empty()) return true;
    FileDoc f; f.path = resolve_ref(dir, ref);
    if (!read_doc(f.path, f.base, err)) return false;
    f.work = f.base; files.turbo = std::move(f);
    return true;
}

bool load_files(const CatalogEntry& entry, Files& files, std::string* err) {
    files.vehicle.path = norm_path(entry.vehicle_path);
    if (!read_doc(files.vehicle.path, files.vehicle.base, err)) return false;
    files.vehicle.work = files.vehicle.base;
    files.trim.clear();
    for (const auto& [slot, key] : entry.interior) files.trim.push_back(FittedPart{slot, key, 1.0, ""});
    const std::string dir = dir_of(files.vehicle.path);
    if (!load_aux(files.vehicle.base, dir, "gearbox", files.gearbox, err)) return false;
    if (!load_aux(files.vehicle.base, dir, "engine", files.engine, err)) return false;
    if (files.engine) {
        files.original_engine_path = files.engine->path;
        files.engine_stem = stem_of_path(files.engine->path);
    }
    return load_turbo(files, err);
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

// The allowed factor range of a Scale/ScaleList option. A catalog entry's own range wins; a slider option derives it from
// its absolute bounds and `stock_file` (the first target leaf's stock value; <= 0 or NaN = unknown: no range then).
std::pair<double, double> range_for(const CatalogEntry& entry, const SetupOptionDef& def, double stock_file = 0.0) {
    const auto it = entry.setup_ranges.find(def.id);
    if (it != entry.setup_ranges.end()) return it->second;
    if (def.slider) {
        if (!(stock_file > 0.0) || !std::isfinite(stock_file)) return {1.0, 1.0};
        const auto bounds = slider_bounds(*def.slider, stock_file);
        return {bounds.first / stock_file, bounds.second / stock_file};
    }
    return {def.min, def.max};
}

struct Target {
    FileDoc* file = nullptr;
    std::string pointer; // concrete JSON pointer in file->work
    std::string kind;    // "vehicle" | "gearbox" | "engine" | "turbo"
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
    } else if (def.file == "turbo") {
        if (files.turbo) add(&*files.turbo, "turbo");
    }
    return out;
}

std::string stem_of(const std::string& path) { return fs::path(path).stem().string(); }

// The gearbox file the car ships with (the vehicle file as loaded, before any swap), absolute; "" without a gearbox.
std::string original_gearbox_path(const Files& files) {
    const int idx = component_index(files.vehicle.base, "gearbox");
    if (idx < 0) return {};
    std::string ref;
    detail::get_string(files.vehicle.base["powertrain"]["components"][static_cast<std::size_t>(idx)], "ref", ref);
    return ref.empty() ? std::string() : resolve_ref(dir_of(files.vehicle.path), ref);
}

// The peak torque (N*m) of the engine that is installed now (a swapped-in engine, else the car's own), from the
// engine list: a curve engine's WOT curve, a simulated engine's dyno-measured catalogue figure (a user engine
// inherits its base's). 0 = unknown, and then no gearbox is turned away. The N2O shot and a bigger turbo setting
// are not counted: the rating is checked against the engine as the list describes it.
double installed_engine_peak_torque(const Files& files, const SetupContext& ctx) {
    if (!ctx.engines || !files.engine) return 0.0;
    const EngineEntry* installed = files.swapped ? &*files.swapped : ctx.engines->find_by_file(files.engine->path);
    if (installed != nullptr) {
        const EnginePeaks p = ctx.engines->peaks(*installed);
        return p.known ? p.torque_nm : 0.0;
    }
    const auto facts = read_engine_facts(files.engine->path, nullptr);
    return facts && facts->curve_peaks.known ? facts->curve_peaks.torque_nm : 0.0;
}

// Why no garage turbo kit can go on the installed engine ("" = one can).
std::string turbo_kit_blocker(const Files& files, const SetupContext& ctx) {
    if (!ctx.turbo_kits) return "No turbo kits are available.";
    if (!files.engine) return "This car has no engine.";
    std::string err;
    const auto facts = read_engine_facts(files.engine->path, &err);
    if (!facts) return err;
    std::string name;
    if (files.swapped) {
        name = files.swapped->name;
    } else if (ctx.engines) {
        if (const EngineEntry* e = ctx.engines->find_by_file(files.engine->path)) name = e->name;
    }
    std::string why = turbo_kit_refusal(*facts, name);
    if (!why.empty()) return why;
    // The car's own vehicle file names a turbo for its engine (the slot override wins over the engine's default). An
    // engine swap drops that override with the old engine (apply_swap), so it only counts on the car's own engine.
    const int idx = component_index(files.vehicle.work, "engine");
    if (idx >= 0 && !files.swapped) {
        std::string fi;
        if (detail::get_string(files.vehicle.work["powertrain"]["components"][static_cast<std::size_t>(idx)], "forced_induction", fi) &&
            !fi.empty() && fi != "none") {
            return "This car's engine is turbocharged already: choose its turbo hardware under Turbo.";
        }
    }
    return {};
}

bool is_driveline_catalog(const SetupOptionDef& def) { return def.catalog == "drive_layouts" || def.catalog == "differentials"; }

// Drivetrain swap: why the option cannot apply to the car as it stands now (its working vehicle file), or "".
std::string driveline_fit_error(const Files& files, const SetupOptionDef& def, const SetupContext& ctx) {
    if (!ctx.drivelines) return "the drivetrain catalogue is unavailable";
    const DrivelineInfo info = detail::describe_driveline_json(files.vehicle.work);
    if (!info.swappable) return "this car's driveline cannot be swapped: " + info.reason;
    if (def.catalog == "differentials" && (def.axle == "front" ? info.front_kind : info.rear_kind).empty()) {
        return std::string("the ") + def.axle + " axle is not driven";
    }
    return {};
}

// "" when the installed engine's internals can be swapped, else the reason in the player's words.
std::string internals_blocker(const Files& files, const SetupContext& ctx) {
    if (!ctx.engine_internals) return "No engine internals are available.";
    if (!files.engine) return "This car has no engine.";
    std::string err;
    const auto facts = read_engine_facts(files.engine->path, &err);
    if (!facts) return err;
    std::string name;
    if (files.swapped) {
        name = files.swapped->name;
    } else if (ctx.engines) {
        if (const EngineEntry* e = ctx.engines->find_by_file(files.engine->path)) name = e->name;
    }
    return engine_internals_refusal(*facts, name);
}

// The installed engine's facts with the garage bore applied (files.internals[kCylinderBoreKind]): bore, displacement and
// compression ratio of the bored engine, factory_* the engine file's. What the turbo kit sizes from.
std::optional<EngineFacts> installed_engine_facts(const Files& files, const SetupContext& ctx, std::string* err) {
    if (!files.engine) {
        if (err != nullptr) *err = "This car has no engine.";
        return std::nullopt;
    }
    auto facts = read_engine_facts(files.engine->path, err);
    if (!facts) return facts;
    const auto it = files.internals.find(kCylinderBoreKind);
    if (it != files.internals.end() && it->second != "stock" && ctx.engine_internals) {
        if (const auto g = bore_geometry(*ctx.engine_internals, *facts, it->second, nullptr); g && g->fits) return bored_engine_facts(*facts, *g);
    }
    return facts;
}

// Why the bore step `id` cannot go on the installed engine ("" = it can; "stock" always can).
std::string bore_blocker(const Files& files, const SetupContext& ctx, const std::string& id) {
    const std::string why = internals_blocker(files, ctx);
    if (!why.empty() || id == "stock") return why;
    if (!ctx.engine_internals->bore) return "No bore upgrades are available.";
    const auto facts = read_engine_facts(files.engine->path, nullptr);
    if (!facts) return "The engine file cannot be read.";
    std::string name;
    if (files.swapped) {
        name = files.swapped->name;
    } else if (ctx.engines) {
        if (const EngineEntry* e = ctx.engines->find_by_file(files.engine->path)) name = e->name;
    }
    return bore_refusal(*ctx.engine_internals, *facts, id, name);
}

// File choices are never a filesystem picker: only declared compatible IDs can resolve a path.
// The engine thermal file the installed engine uses: the vehicle's engine component "thermal" override, else the engine file's
// default_thermal (followed through its extends chain; a relative path is relative to the file that names it). "" when none.
std::string engine_thermal_path(const Files& files) {
    const int idx = component_index(files.vehicle.work, "engine");
    if (idx >= 0) {
        std::string ref;
        if (detail::get_string(files.vehicle.work["powertrain"]["components"][static_cast<std::size_t>(idx)], "thermal", ref) && !ref.empty()) {
            return resolve_ref(dir_of(files.vehicle.path), ref);
        }
    }
    if (!files.engine) return {};
    std::string path = files.engine->path;
    for (int depth = 0; depth < 8 && !path.empty(); ++depth) {
        json doc;
        if (!read_doc(path, doc, nullptr)) return {};
        std::string ref;
        if (detail::get_string(doc, "default_thermal", ref) && !ref.empty()) return resolve_ref(dir_of(path), ref);
        std::string parent;
        if (!detail::get_string(doc, "extends", parent) || parent.empty()) return {};
        path = resolve_ref(dir_of(path), parent);
    }
    return {};
}

std::string cooling_system_path(const Files& files) {
    std::string ref;
    if (!detail::get_string(files.vehicle.work, "cooling_system", ref) || ref.empty()) return {};
    return resolve_ref(dir_of(files.vehicle.path), ref);
}

// "" when the car has a cooling model the cooling options can work on, else the reason in the player's words.
std::string cooling_blocker_of(const Files& files, const SetupContext& ctx) {
    if (!ctx.cooling_upgrades) return "No cooling upgrades are available.";
    if (cooling_system_path(files).empty()) {
        return files.swapped ? "The installed engine has no thermal data, so the car has no cooling model to upgrade."
                             : "This car has no cooling system model (its vehicle file names no cooling_system), so there is nothing to upgrade.";
    }
    if (engine_thermal_path(files).empty()) return "The installed engine has no thermal data, so the car has no cooling model to upgrade.";
    return {};
}

// Loads the cooling_system and engine thermal files of the car as it stands (after any swap), applies the chosen grades to the
// docs' `work`, rebuilds the cooling parts of the part-mass tree. `options`: part kind -> the option that swaps it.
// Returns "" or the reason a chosen grade cannot be applied; files.cooling_blocker says whether the options are offered at all.
std::string refresh_cooling(Files& files, const SetupContext& ctx, const std::map<std::string, std::string>& options = {}) {
    files.part_extras.erase(std::remove_if(files.part_extras.begin(), files.part_extras.end(),
                                           [](const PartAttachment& a) { return a.parent_id == "engine.cooling"; }),
                            files.part_extras.end());
    files.cooling.reset();
    files.thermal.reset();
    files.cooling_build.reset();
    files.cooling_blocker = cooling_blocker_of(files, ctx);
    if (!files.cooling_blocker.empty()) return files.cooling_grades.empty() ? std::string() : files.cooling_blocker;
    std::string err;
    FileDoc cool, therm;
    cool.path = cooling_system_path(files);
    therm.path = engine_thermal_path(files);
    const auto cool_eff = load_effective_json(cool.path, &err);
    const auto therm_eff = cool_eff ? load_effective_json(therm.path, &err) : std::nullopt;
    if (!cool_eff || !therm_eff || !read_doc(cool.path, cool.base, &err) || !read_doc(therm.path, therm.base, &err)) {
        files.cooling_blocker = "The cooling files cannot be read: " + err;
        return files.cooling_grades.empty() ? std::string() : files.cooling_blocker;
    }
    cool.work = cool.base;
    therm.work = therm.base;
    double hint = 0.0;
    if (files.engine) {
        if (const auto facts = read_engine_facts(files.engine->path, nullptr)) hint = facts->displacement_l;
    }
    CoolingEdit edit;
    auto build = build_cooling(*ctx.cooling_upgrades, files.cooling_grades, options, *cool_eff, *therm_eff, hint, &edit, &err);
    if (!build) {
        files.cooling_blocker = err;
        return files.cooling_grades.empty() ? std::string() : err;
    }
    files.cooling_effective = *cool_eff;
    files.thermal_effective = *therm_eff;
    if (!edit.cooling.empty()) cool.work.merge_patch(edit.cooling);
    if (!edit.thermal.empty()) therm.work.merge_patch(edit.thermal);
    files.cooling = std::move(cool);
    files.thermal = std::move(therm);
    for (PartAttachment& a : cooling_parts(*build)) files.part_extras.push_back(std::move(a));
    files.cooling_build = std::move(*build);
    return {};
}

bool file_choice_fits(Files& files, const SetupOptionDef& def, const CatalogEntry& entry, const SetupContext& ctx) {
    if (!entry_offers(entry, def.id)) return false;
    if (is_driveline_catalog(def)) return driveline_fit_error(files, def, ctx).empty();
    if (def.catalog.empty()) {
        // "*" in compatible_engines: the part does not depend on the engine (e.g. the assist electronics).
        const std::string& engine = files.engine_stem;
        const bool any_engine = std::find(def.compatible_engines.begin(), def.compatible_engines.end(), "*") != def.compatible_engines.end();
        if (!any_engine && std::find(def.compatible_engines.begin(), def.compatible_engines.end(), engine) == def.compatible_engines.end()) return false;
    } else if (def.catalog == "turbo_kits") {
        return turbo_kit_blocker(files, ctx).empty();
    } else if (def.catalog == "engine_internals") {
        if (def.internal == kCylinderBoreKind) return internals_blocker(files, ctx).empty() && ctx.engine_internals->bore.has_value();
        return internals_blocker(files, ctx).empty() && ctx.engine_internals->find(def.internal) != nullptr;
    } else if (def.catalog == "cooling_upgrades") {
        return cooling_blocker_of(files, ctx).empty() && ctx.cooling_upgrades->find(def.cooling_part) != nullptr;
    } else if (def.catalog == "charge_piping") {
        // Offered wherever a kit can go (the kit is chosen in the same visit); without a kit it changes nothing.
        return ctx.turbo_kits != nullptr && (files.kit_spec.has_value() || turbo_kit_blocker(files, ctx).empty());
    } else if (def.catalog == "gearboxes" ? !ctx.gearboxes : !ctx.engines) {
        return false;
    }
    const int idx = component_index(files.vehicle.base, "engine");
    if (def.no_turbo_override && idx >= 0 && files.vehicle.base["powertrain"]["components"][idx].contains("forced_induction")) return false;
    const json* base = def.file == "vehicle" ? &files.vehicle.base : (files.engine ? &files.engine->base : nullptr);
    if (!base) return false;
    return !expand_patterns(*base, pointers_for(entry, def)).empty() || def.insert_if_absent.has_value();
}

std::string install_clutch_to_clutch(Files& files, const GearboxEntry& g);

// The installed engine's catalogue entry (a swapped-in one, else the car's own); nullptr when not in the list.
const EngineEntry* installed_engine_entry(const Files& files, const SetupContext& ctx) {
    if (files.swapped) return &*files.swapped;
    if (!ctx.engines || !files.engine) return nullptr;
    return ctx.engines->find_by_file(files.engine->path);
}

// The fitted kit as a TurboKitBuild (sizing, piping, parts with masses and limits) and, below the engine of the part-mass
// tree, a "turbo_kit" node with one child per part (each turbo its own); nullopt and no node without a kit.
std::optional<TurboKitBuild> refresh_kit_parts(Files& files, const SetupContext& ctx) {
    files.part_extras.erase(std::remove_if(files.part_extras.begin(), files.part_extras.end(),
                                           [](const PartAttachment& a) { return a.parent_id == "engine" && a.node.id == "turbo_kit"; }),
                            files.part_extras.end());
    if (!files.kit_spec || !ctx.turbo_kits || !files.engine) return std::nullopt;
    const auto facts = installed_engine_facts(files, ctx, nullptr);
    if (!facts) return std::nullopt;
    const TurboKitSpec& spec = *files.kit_spec;
    const TurboKitSizing sizing = size_turbo_kit(*ctx.turbo_kits, spec, *facts, facts->limiter_rpm);
    if (!sizing.ok) return std::nullopt;
    const ChargePipingSpec* piping = ctx.turbo_kits->find_piping(files.kit_piping.empty() ? ctx.turbo_kits->default_piping : files.kit_piping);
    if (piping == nullptr) return std::nullopt;
    TurboKitBuild build = describe_turbo_kit(*ctx.turbo_kits, spec, sizing, *piping);
    if (const EngineEntry* eng = installed_engine_entry(files, ctx); eng != nullptr && eng->envelope_mm) {
        build.envelope_mm = turbo_kit_envelope_mm(*ctx.turbo_kits, spec.count, sizing, *eng->envelope_mm, eng->envelope_layout == "v");
    }
    files.part_extras.push_back(turbo_kit_parts(build));
    return build;
}

// The installed engine internals (every part of the table, stock where not chosen) with their limits for the INSTALLED
// engine, and the part-mass tree's "internals" node (the difference to stock); nullopt and no node when the engine's
// internals cannot be swapped. `options`: part kind -> the option that swaps it (the advisor names it).
std::optional<EngineInternalsBuild> refresh_internals_parts(Files& files, const SetupContext& ctx,
                                                            const std::map<std::string, std::string>& options = {}) {
    files.part_extras.erase(std::remove_if(files.part_extras.begin(), files.part_extras.end(),
                                           [](const PartAttachment& a) { return a.parent_id == "engine" && a.node.id == "internals"; }),
                            files.part_extras.end());
    if (!ctx.engine_internals || !files.engine) return std::nullopt;
    const auto facts = read_engine_facts(files.engine->path, nullptr);
    if (!facts) return std::nullopt;
    auto build = describe_engine_internals(*ctx.engine_internals, *facts, files.internals, options, nullptr);
    if (!build) return std::nullopt;
    if (auto node = engine_internals_parts(*build)) files.part_extras.push_back(std::move(*node));
    return build;
}

// Sizes the kit for the installed engine at spec.target_boost_bar, checks that engine + kit fit the bay, writes the
// generated files and converts the engine (default_forced_induction, boost grid, compression ratio). "" when fine.
std::string fit_turbo_kit(Files& files, const CatalogEntry& entry, const SetupContext& ctx, const TurboKitSpec& spec) {
    std::string err;
    const auto facts = installed_engine_facts(files, ctx, &err); // a bored engine: the kit sizes for its displacement
    if (!facts) return err;
    const TurboKitSizing sizing = size_turbo_kit(*ctx.turbo_kits, spec, *facts, facts->limiter_rpm);
    if (!sizing.ok) return sizing.error;
    if (const EngineEntry* eng = installed_engine_entry(files, ctx); eng != nullptr && eng->envelope_mm) {
        const auto box = turbo_kit_envelope_mm(*ctx.turbo_kits, spec.count, sizing, *eng->envelope_mm, eng->envelope_layout == "v");
        const FitResult fit = check_fit_size(entry, box);
        if (!fit.allowed) {
            std::string why;
            for (const std::string& r : fit.reasons) why += (why.empty() ? "" : "; ") + r;
            return spec.label + " (" + std::to_string(spec.count) + " turbo" + (spec.count > 1 ? "s" : "") + ") on the " + eng->name +
                   " does not fit " + entry.title + " (" + why + ")";
        }
    }
    const std::string cache = !ctx.engine_map_cache_dir.empty() ? ctx.engine_map_cache_dir : ctx.work_root;
    const std::string config_path = write_turbo_kit(*ctx.turbo_kits, spec, sizing, stem_of_path(files.engine->path), cache, &err);
    if (config_path.empty()) return err;
    const auto grid = turbo_kit_grid_kpa(*facts, spec.target_boost_bar, &err);
    if (!grid) return err;
    json& work = files.engine->work;
    work["default_forced_induction"] = norm_path(config_path);
    work["cycle"]["grid_p_im_kpa"] = *grid;
    if (sizing.compression_ratio < facts->compression_ratio) work["cycle"]["compression_ratio"] = sizing.compression_ratio;
    files.kit_label = spec.label;
    // Against the engine's catalogue peak (the unbored engine): a bored engine makes displacement-ratio more before boost.
    const double bore_ratio = facts->factory_displacement_l > 0.0 ? facts->displacement_l / facts->factory_displacement_l : 1.0;
    files.kit_torque_factor = sizing.torque_factor * bore_ratio;
    files.kit_spec = spec;
    refresh_kit_parts(files, ctx);
    return {};
}

// Engine garage G1: a choice from the engine store ("engines") or the installed engine's fuels ("fuels").
std::string install_catalog_part(Files& files, const SetupOptionDef& def, const CatalogEntry& entry, const SetupContext& ctx,
                                 const std::string& id) {
    if (def.catalog == "drive_layouts") {
        const DriveLayoutEntry* l = ctx.drivelines->find_layout(id);
        if (l == nullptr) return def.id + ": \"" + id + "\" is not a drive layout in the catalogue";
        const std::string e = detail::apply_layout_json(files.vehicle.work, *ctx.drivelines, *l);
        if (!e.empty()) return def.id + ": " + e;
        // The differentials and their shafts are rebuilt by trusted code: exempt from the whitelist like a swap package.
        files.derived_subtrees.push_back("/powertrain/components");
        files.derived_subtrees.push_back("/powertrain/shafts");
        files.layout_info = detail::describe_driveline_json(files.vehicle.work);
        return {};
    }
    if (def.catalog == "differentials") {
        const DiffUnit* u = ctx.drivelines->find_differential(id);
        if (u == nullptr) return def.id + ": \"" + id + "\" is not a differential in the catalogue";
        int index = -1;
        const std::string e = detail::apply_axle_diff_json(files.vehicle.work, def.axle == "front", *u, &index);
        if (!e.empty()) return def.id + ": " + e;
        files.derived_subtrees.push_back("/powertrain/components/" + std::to_string(index));
        return {};
    }
    if (def.catalog == "engines") {
        const EngineEntry* e = ctx.engines->find(id);
        if (e == nullptr) return def.id + ": \"" + id + "\" is not an engine in the garage list";
        const FitResult fit = check_fit(entry, *e);
        if (!fit.allowed) {
            std::string why;
            for (const std::string& r : fit.reasons) why += (why.empty() ? "" : "; ") + r;
            return def.id + ": " + e->name + " does not fit " + entry.title + (why.empty() ? std::string() : " (" + why + ")");
        }
        if (norm_path(e->file) == files.original_engine_path) {
            // The car's own engine, chosen explicitly: nothing changes (also when it was swapped before in this session).
            return {};
        }
        for (auto& target : targets_of(files, def, pointers_for(entry, def))) {
            const json* leaf = at_pointer(target.file->work, target.pointer);
            if (!leaf || !leaf->is_string()) return def.id + ": reference is not a string";
            target.file->work[json::json_pointer(target.pointer)] = norm_path(e->file);
        }
        std::string err;
        if (!load_aux(files.vehicle.work, dir_of(files.vehicle.path), "engine", files.engine, &err)) return err;
        files.engine_stem = e->base_stem;
        files.swapped = *e;
        return {};
    }
    if (def.catalog == "gearboxes") {
        const GearboxEntry* g = ctx.gearboxes->find(id);
        if (g == nullptr) return def.id + ": \"" + id + "\" is not a gearbox in the catalogue";
        if (norm_path(g->file) == original_gearbox_path(files)) {
            // The car's own gearbox, chosen explicitly: nothing changes (also when it was swapped before in this session).
            return {};
        }
        const std::string rating = check_gearbox_rating(*g, installed_engine_peak_torque(files, ctx));
        if (!rating.empty()) return rating;
        for (auto& target : targets_of(files, def, pointers_for(entry, def))) {
            const json* leaf = at_pointer(target.file->work, target.pointer);
            if (!leaf || !leaf->is_string()) return def.id + ": reference is not a string";
            target.file->work[json::json_pointer(target.pointer)] = norm_path(g->file);
        }
        std::string err;
        if (!load_aux(files.vehicle.work, dir_of(files.vehicle.path), "gearbox", files.gearbox, &err)) return err;
        const bool now_auto = controller_of_type(files.vehicle.work, "automatic_tcu") >= 0;
        if (is_clutch_to_clutch(g->kind)) {
            const std::string e = install_clutch_to_clutch(files, *g);
            if (!e.empty()) return def.id + ": " + e;
        } else if (now_auto) {
            return def.id + ": " + g->name + " needs a clutch pedal, and this car has none (it was built with a clutch-to-clutch gearbox)";
        }
        return {};
    }
    if (def.catalog == "turbo_kits") {
        const TurboKitSpec* spec = ctx.turbo_kits->find(id);
        if (spec == nullptr) return def.id + ": \"" + id + "\" is not a turbo kit";
        const std::string why = fit_turbo_kit(files, entry, ctx, *spec);
        return why.empty() ? std::string() : def.id + ": " + why;
    }
    if (def.catalog == "engine_internals" && def.internal == kCylinderBoreKind) {
        const std::string why = bore_blocker(files, ctx, id);
        if (!why.empty()) return def.id + ": " + why;
        const auto facts = read_engine_facts(files.engine->path, nullptr);
        std::string err;
        const auto g = facts ? bore_geometry(*ctx.engine_internals, *facts, id, &err) : std::nullopt;
        if (!g) return def.id + ": " + (err.empty() ? std::string("the engine file cannot be read") : err);
        // The engine file's own bore (displacement and the generated maps follow; bore_m is in the engine-map cache
        // key) and the compression ratio of the bigger swept volume over the same head (head volume kept).
        json& work = files.engine->work;
        work["bore_mm"] = std::round(g->bore_m * 1.0e6) / 1000.0;
        work["cycle"]["compression_ratio"] = g->compression_ratio;
        files.internals[kCylinderBoreKind] = id;
        refresh_internals_parts(files, ctx);
        return {};
    }
    if (def.catalog == "engine_internals") {
        const InternalPartKind* kind = ctx.engine_internals->find(def.internal);
        if (kind == nullptr || kind->find(id) == nullptr) return def.id + ": \"" + id + "\" is not a grade of " + def.internal;
        files.internals[def.internal] = id;
        refresh_internals_parts(files, ctx);
        return {};
    }
    if (def.catalog == "cooling_upgrades") {
        const CoolingPart* part = ctx.cooling_upgrades->find(def.cooling_part);
        if (part == nullptr || part->find(id) == nullptr) return def.id + ": \"" + id + "\" is not a grade of the " + def.cooling_part;
        files.cooling_grades[def.cooling_part] = id;
        return {};
    }
    if (def.catalog == "charge_piping") {
        if (ctx.turbo_kits->find_piping(id) == nullptr) return def.id + ": \"" + id + "\" is not a charge piping material";
        files.kit_piping = id;
        refresh_kit_parts(files, ctx);
        return {};
    }
    if (def.catalog == "fuels") {
        if (!files.engine) return def.id + ": the car has no engine";
        std::string err;
        const auto facts = read_engine_facts(files.engine->path, &err);
        if (!facts) return err;
        const bool listed = std::any_of(facts->fuels.begin(), facts->fuels.end(), [&](const FuelFact& f) { return f.name == id; });
        if (!listed) return def.id + ": \"" + id + "\" is not one of this engine's fuels";
        files.fuel_choice = id;
        return {};
    }
    return def.id + ": unknown catalog \"" + def.catalog + "\"";
}

std::string install_file_part(Files& files, const SetupOptionDef& def, const CatalogEntry& entry, const SetupContext& ctx,
                              const SetupValue& value) {
    if (!std::holds_alternative<std::string>(value)) return def.id + ": needs a component ID";
    if ((def.catalog == "turbo_kits" || def.catalog == "charge_piping" || def.catalog == "engine_internals" || def.catalog == "cooling_upgrades") &&
        std::get<std::string>(value) == "stock") return {}; // nothing to check
    if (is_driveline_catalog(def) && entry_offers(entry, def.id)) {
        if (std::get<std::string>(value) == "stock") return {}; // keep the driveline as it is
        const std::string why = driveline_fit_error(files, def, ctx);
        if (!why.empty()) return def.id + ": " + why;
    }
    if (!file_choice_fits(files, def, entry, ctx)) {
        if (def.catalog == "turbo_kits" && entry_offers(entry, def.id)) return def.id + ": " + turbo_kit_blocker(files, ctx);
        if (def.catalog == "engine_internals" && entry_offers(entry, def.id)) return def.id + ": " + internals_blocker(files, ctx);
        if (def.catalog == "cooling_upgrades" && entry_offers(entry, def.id)) return def.id + ": " + cooling_blocker_of(files, ctx);
        return def.id + ": not compatible with this vehicle";
    }
    const auto& id = std::get<std::string>(value);
    if (id == "stock") return {};
    if (!def.catalog.empty()) return install_catalog_part(files, def, entry, ctx, id);
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
            files.engine->work.erase("default_forced_induction");
            // Boosted map nodes are unreachable without a compressor. Use the authored NA sampling grid.
            files.engine->work["cycle"]["grid_p_im_kpa"] = doc.at("grid_p_im_kpa");
        }
        const int engine_idx = component_index(files.vehicle.work, "engine");
        if (engine_idx >= 0) files.vehicle.work["powertrain"]["components"][engine_idx].erase("forced_induction");
        return {};
    }
    if (!detail::get_string(doc, "format", format) || format != def.part_format) return def.id + ": wrong component file format";
    if (def.insert_if_absent && targets_of(files, def, pointers_for(entry, def)).empty()) {
        // The car has no such entry in its stock file: add it (see SetupOptionDef::insert_if_absent).
        const auto entry_doc = detail::parse_json(def.insert_if_absent->entry_json);
        if (!entry_doc || !entry_doc->is_object()) return def.id + ": invalid insert_if_absent entry";
        json item = *entry_doc;
        item[def.insert_if_absent->ref_key] = part->path;
        const json::json_pointer arr(def.insert_if_absent->array);
        json& target_array = files.vehicle.work[arr];
        if (!target_array.is_array()) target_array = json::array();
        target_array.push_back(std::move(item));
        return {};
    }
    for (auto& target : targets_of(files, def, pointers_for(entry, def))) {
        const json* leaf = at_pointer(target.file->work, target.pointer);
        if (!leaf || !leaf->is_string()) return def.id + ": reference is not a string";
        target.file->work[json::json_pointer(target.pointer)] = part->path;
    }
    // Reload the installed engine BEFORE calibrating it. Its own relative dependencies keep their directory.
    if (def.part_format == "physics_sim.engine/1") {
        if (!load_aux(files.vehicle.work, dir_of(files.vehicle.path), "engine", files.engine, &err)) return err;
        files.engine_stem = stem_of_path(files.engine->path);
    }
    return {};
}


// ---- the swap package (engine garage G1) ---------------------------------------------------------------

int component_of_type(const json& vehicle, const char* type) {
    const auto pt = vehicle.find("powertrain");
    if (pt == vehicle.end() || !pt->is_object()) return -1;
    const auto comps = pt->find("components");
    if (comps == pt->end() || !comps->is_array()) return -1;
    for (std::size_t i = 0; i < comps->size(); ++i) {
        std::string ty;
        if ((*comps)[i].is_object() && detail::get_string((*comps)[i], "type", ty) && ty == type) return static_cast<int>(i);
    }
    return -1;
}

int controller_of_type(const json& vehicle, const char* type) {
    const auto cs = vehicle.find("controllers");
    if (cs == vehicle.end() || !cs->is_array()) return -1;
    for (std::size_t i = 0; i < cs->size(); ++i) {
        std::string ty;
        if ((*cs)[i].is_object() && detail::get_string((*cs)[i], "type", ty) && ty == type) return static_cast<int>(i);
    }
    return -1;
}

// Automatic gearbox swap (2026-10-16): the car gets a clutch-to-clutch gearbox (automatic, dual_clutch or cvt, whose ref
// install_catalog_part has already repointed). Trusted code, like a drive layout: the clutch component goes (its two
// shafts become one, engine to gearbox), the manual_tcu becomes an automatic_tcu (same id, so the clutch assist
// options no longer find their pointers and drop out of the setup), drive-mode and assist-fitment electronics that
// need a manual_tcu are removed, and a park lock is fitted on the gearbox output when the car has none. The TCU's rpm
// points follow the INSTALLED engine (engine_install runs before gearbox_install). Rows of an already clutch-to-clutch
// car are rebuilt the same way (another automatic or dual-clutch box: the TCU template of the new box).
std::string install_clutch_to_clutch(Files& files, const GearboxEntry& g) {
    json& veh = files.vehicle.work;
    if (!files.engine) return "the car has no engine for the gearbox controller to read";
    std::string err;
    const auto facts = read_engine_facts(files.engine->path, &err);
    if (!facts) return err;
    const AutomaticTcuPoints pts = derive_automatic_tcu_points(facts->idle_rpm, facts->limiter_rpm);
    if (!pts.ok) return "the engine's idle and limiter rpm leave no room for a shift schedule";
    json& pt = veh["powertrain"];
    json& comps = pt["components"];
    json& shafts = pt["shafts"];
    if (!comps.is_array() || !shafts.is_array()) return "the vehicle has no powertrain components or shafts";
    const int gi = component_of_type(veh, "gearbox");
    const int ei = component_of_type(veh, "engine");
    if (gi < 0 || ei < 0) return "the vehicle has no gearbox or engine component";
    std::string gearbox_id, engine_id;
    detail::get_string(comps[gi], "id", gearbox_id);
    detail::get_string(comps[ei], "id", engine_id);

    // The clutch: merge its input and output shafts, then remove it.
    const int ci = component_of_type(veh, "clutch");
    if (ci >= 0) {
        std::string clutch_id;
        detail::get_string(comps[ci], "id", clutch_id);
        const std::string in = clutch_id + ".in", out = clutch_id + ".out";
        int si = -1, so = -1;
        for (std::size_t i = 0; i < shafts.size(); ++i) {
            const json& ports = shafts[i]["ports"];
            if (!ports.is_array()) continue;
            for (const json& p : ports) {
                if (p == in) si = static_cast<int>(i);
                if (p == out) so = static_cast<int>(i);
            }
        }
        if (si < 0 || so < 0 || si == so) return "the clutch is not between two shafts";
        json merged = json::array();
        for (const int idx : {si, so}) {
            for (const json& p : shafts[idx]["ports"]) if (p != in && p != out) merged.push_back(p);
        }
        json kept = json::array();
        for (std::size_t i = 0; i < shafts.size(); ++i) {
            if (static_cast<int>(i) == si) {
                json sh = shafts[i];
                sh["ports"] = merged;
                // A shaft compliance of either side is dropped with the clutch (the two shafts are one now).
                sh.erase("stiffness_nm_per_rad");
                sh.erase("damping_nm_s_per_rad");
                kept.push_back(std::move(sh));
            } else if (static_cast<int>(i) != so) {
                kept.push_back(shafts[i]);
            }
        }
        shafts = std::move(kept);
        comps.erase(comps.begin() + ci);
        files.gearbox_lines.push_back("Clutch removed: the gearbox shifts on its own clutches, no clutch pedal");
    }

    // The park lock on the gearbox output, unless the car has one.
    if (component_of_type(veh, "park_lock") < 0) {
        std::string pid = "park";
        for (int n = 2; ; ++n) {
            bool taken = false;
            for (const json& c : comps) if (c.value("id", std::string()) == pid) taken = true;
            if (!taken) break;
            pid = "park" + std::to_string(n);
        }
        const std::string gout = gearbox_id + ".out";
        bool placed = false;
        for (json& sh : shafts) {
            if (!sh["ports"].is_array()) continue;
            for (const json& p : sh["ports"]) placed = placed || p == gout;
            if (placed) { sh["ports"].push_back(pid + ".shaft"); break; }
        }
        if (!placed) return "the gearbox output is on no shaft";
        // [AS] as physics_sim's automatic fixtures: 1500 N*m at the gearbox output (x the final drive at the wheels),
        // engages below 1 rad/s.
        comps.push_back(json{{"id", pid}, {"type", "park_lock"}, {"capacity_nm", 1500.0}, {"static_factor", 1.0},
                             {"max_engage_speed_rad_s", 1.0}, {"channel", "park_lock"}});
        files.gearbox_lines.push_back("Park lock fitted on the gearbox output (P on the selector)");
    }

    // The controller.
    auto tpl = detail::parse_json(g.tcu_json);
    if (!tpl || !tpl->is_object()) return "the catalogue has no controller data for " + g.name;
    json tcu = *tpl;
    std::string tcu_id = "tcu";
    json& ctrls = veh["controllers"];
    if (!ctrls.is_array()) ctrls = json::array();
    int at = controller_of_type(veh, "manual_tcu");
    if (at < 0) at = controller_of_type(veh, "automatic_tcu");
    if (at >= 0) detail::get_string(ctrls[static_cast<std::size_t>(at)], "id", tcu_id);
    tcu["id"] = tcu_id;
    tcu["type"] = "automatic_tcu";
    tcu["gearbox"] = gearbox_id;
    tcu["engine"] = engine_id;
    tcu["schedule"] = json{{"up_rpm_at_zero_throttle", pts.up_zero}, {"up_rpm_at_full_throttle", pts.up_full},
                           {"down_rpm_at_zero_throttle", pts.down_zero}, {"down_rpm_at_full_throttle", pts.down_full}};
    tcu["sport_schedule"] = json{{"up_rpm_at_zero_throttle", pts.sport_up_zero}, {"up_rpm_at_full_throttle", pts.sport_up_full},
                                 {"down_rpm_at_zero_throttle", pts.sport_down_zero}, {"down_rpm_at_full_throttle", pts.sport_down_full}};
    tcu["kickdown_max_rpm"] = pts.kickdown_max_rpm;
    if (g.converter) { // a converter automatic or a converter CVT: lock-up
        json lk = tcu.contains("lockup") ? tcu["lockup"] : json::object();
        lk["engage_rpm"] = pts.lockup_engage_rpm;
        lk["release_rpm"] = pts.lockup_release_rpm;
        tcu["lockup"] = std::move(lk);
    } else { // a dual-clutch box or a starting-clutch CVT: the launch clutch
        tcu["launch"] = json{{"bite_rpm", pts.launch_bite_rpm}, {"full_rpm_min", pts.launch_full_rpm_min},
                             {"full_rpm_max", pts.launch_full_rpm_max}, {"lock_slip_rpm", pts.launch_lock_slip_rpm},
                             {"anti_stall_rpm", pts.launch_anti_stall_rpm}};
    }
    if (g.kind == GearboxKind::Cvt) {
        // CVT ratio law (physics automatic_tcu "cvt"): the engine rpm the variator aims at, from the installed engine.
        json c = tcu.contains("cvt") ? tcu["cvt"] : json::object();
        json tx = json::array(), ty = json::array(), mx = json::array(), my = json::array();
        for (int i = 0; i < 5; ++i) { tx.push_back(kCvtTargetThrottle[i]); ty.push_back(pts.cvt_target_rpm[i]); }
        for (int i = 0; i < 3; ++i) { mx.push_back(kCvtMinOutputRpm[i]); my.push_back(pts.cvt_min_rpm[i]); }
        c["target_rpm_vs_throttle"] = json{{"x", tx}, {"y", ty}};
        c["min_rpm_vs_output_rpm"] = json{{"x", mx}, {"y", my}};
        tcu["cvt"] = std::move(c);
    }
    const bool stepped = g.kind == GearboxKind::Cvt && tcu["cvt"].value("stepped_sport", true);
    if (at >= 0) ctrls[static_cast<std::size_t>(at)] = std::move(tcu);
    else ctrls.insert(ctrls.begin(), std::move(tcu));
    char buf[220];
    if (g.kind == GearboxKind::Cvt) {
        std::snprintf(buf, sizeof(buf), "CVT: holds %.0f rpm at full throttle%s, P R N D on the selector, the shift keys select the steps",
                      pts.cvt_target_rpm[4], stepped ? ", S steps through fixed ratios" : "");
    } else {
        std::snprintf(buf, sizeof(buf), "%s: shifts up at %.0f rpm at full throttle (sport %.0f), P R N D on the selector, the shift keys select manual mode",
                      g.kind == GearboxKind::Automatic ? "Automatic" : "Dual-clutch", pts.up_full, pts.sport_up_full);
    }
    files.gearbox_lines.push_back(buf);

    // Keep driving modes: the automatic TCU accepts their shift/close overrides too.
    // Only fitment of the removed manual-clutch/shift assists is incompatible.
    json kept = json::array();
    for (const json& c : ctrls) {
        std::string ty, ref, kind;
        if (c.is_object() && detail::get_string(c, "type", ty) && ty == "electronics" && detail::get_string(c, "ref", ref)) {
            json doc;
            if (read_doc(resolve_ref(dir_of(files.vehicle.path), ref), doc, nullptr)) detail::get_string(doc, "kind", kind);
            if (kind == "assist_fitment" && doc.dump().find("auto_") != std::string::npos) {
                files.gearbox_lines.push_back("Clutch and shift assists removed with the manual gearbox");
                continue;
            }
        }
        kept.push_back(c);
    }
    ctrls = std::move(kept);
    files.derived_subtrees.push_back("/powertrain/components");
    files.derived_subtrees.push_back("/powertrain/shafts");
    files.derived_subtrees.push_back("/controllers");
    return {};
}

const char* const kEngineSlotKeys[] = {"ecu", "cam", "forced_induction", "installation", "thermal"};

// Derives the swap package for the installed engine / chosen fuel and applies it to the working vehicle file.
// "" when fine (also when nothing was swapped or chosen: `pkg` stays empty), else the broken rule.
std::string apply_swap(Files& files, const CatalogEntry& entry, const SetupContext& ctx, SwapPackage& pkg) {
    pkg = SwapPackage{};
    if (!files.swapped && files.fuel_choice.empty() && files.kit_label.empty()) return {};
    if (!ctx.engines || !files.engine) return entry.id + ": no engine list to derive a swap from";
    json& veh = files.vehicle.work;
    if (!veh.contains("powertrain") || !veh["powertrain"].contains("components") || !veh["powertrain"]["components"].is_array()) {
        return entry.id + ": the vehicle has no powertrain components";
    }
    json& comps = veh["powertrain"]["components"];
    const int tank = component_of_type(veh, "fuel_tank");
    const int clutch = component_of_type(veh, "clutch");
    const int engine = component_of_type(veh, "engine");
    SwapCar car;
    if (tank >= 0) {
        std::string fuel;
        if (detail::get_string(comps[tank], "fuel", fuel)) {
            car.has_tank = true;
            car.tank_fuel = stem_of_path(fuel);
        }
    }
    if (clutch >= 0) detail::get_number(comps[clutch], "capacity_nm", car.clutch_kinetic_nm);
    car.has_nitrous = component_of_type(veh, "n2o_bottle") >= 0 || component_of_type(veh, "nitrous_kit") >= 0;
    if (engine >= 0) {
        for (const char* key : kEngineSlotKeys) car.has_slot_overrides = car.has_slot_overrides || comps[engine].contains(key);
    }
    car.has_cooling_system = veh.contains("cooling_system");
    if (const EngineEntry* stock = ctx.engines->find_by_file(files.original_engine_path)) car.stock_engine_mass_kg = stock->mass_kg;

    std::string err;
    const auto facts = read_engine_facts(files.engine->path, &err);
    if (!facts) return err;
    SwapEngine eng;
    const EngineEntry* installed = files.swapped ? &*files.swapped : ctx.engines->find_by_file(files.engine->path);
    eng.name = installed != nullptr ? installed->name : stem_of_path(files.engine->path);
    eng.mass_kg = installed != nullptr ? installed->mass_kg : facts->mass_kg;
    eng.peak_torque_nm = installed != nullptr ? ctx.engines->peaks(*installed).torque_nm : facts->curve_peaks.torque_nm;
    eng.idle_rpm = facts->idle_rpm;
    eng.limiter_rpm = facts->limiter_rpm;
    eng.simulated = facts->simulated;
    eng.mechanical_throttle = facts->mechanical_throttle;
    eng.has_thermal = facts->has_thermal;
    for (const FuelFact& f : facts->fuels) eng.fuels.push_back(f.name);

    SwapChoices choices;
    choices.fuel = files.fuel_choice;
    choices.turbo_kit = files.kit_label;
    choices.boost_torque_factor = files.kit_torque_factor;
    pkg = derive_swap_package(car, eng, choices, files.swapped.has_value());
    if (!pkg.ok) return entry.id + ": " + pkg.error;
    if (controller_of_type(veh, "manual_tcu") < 0) {
        // Automatic gearbox plan A1: a clutch-to-clutch box has no manual_tcu, so there are no clutch-assist or
        // auto-shift points to move (install_clutch_to_clutch derived the automatic_tcu's own from the engine).
        pkg.set_tcu = false;
        std::vector<std::string> kept;
        for (std::string& l : pkg.lines) {
            if (l.rfind("Clutch assist:", 0) == 0 || l.rfind("Auto shift:", 0) == 0) continue;
            kept.push_back(std::move(l));
        }
        pkg.lines = std::move(kept);
    }
    // A launch rpm is a second rev limit of the old engine: one outside the new engine's idle..limiter band is moved
    // to 60 % of the new limiter (at least idle + 300 rpm), else the loader rejects the car.
    files.electronics.clear();
    if (files.swapped && eng.limiter_rpm > eng.idle_rpm && veh.contains("controllers") && veh["controllers"].is_array()) {
        const double launch = std::min(std::round(std::max(eng.idle_rpm + 300.0, 0.6 * eng.limiter_rpm) / 50.0) * 50.0,
                                       eng.limiter_rpm - 100.0);
        for (const json& c : veh["controllers"]) {
            std::string ty, ref;
            if (!c.is_object() || !detail::get_string(c, "type", ty) || ty != "electronics" || !detail::get_string(c, "ref", ref)) continue;
            FileDoc doc;
            doc.path = resolve_ref(dir_of(files.vehicle.path), ref);
            if (!read_doc(doc.path, doc.base, &err)) return err;
            doc.work = doc.base;
            std::string kind;
            detail::get_string(doc.base, "kind", kind);
            json* holder = nullptr;
            const char* key = nullptr;
            if (kind == "launch_control") {
                holder = &doc.work;
                key = "rpm";
            } else if (kind == "assist_unit" && doc.work.contains("launch_control") && doc.work["launch_control"].is_object()) {
                holder = &doc.work["launch_control"];
                key = "launch_rpm";
            }
            double rpm = 0.0;
            if (holder == nullptr || !detail::get_number(*holder, key, rpm)) continue;
            if (rpm > eng.idle_rpm && rpm < eng.limiter_rpm) continue;
            (*holder)[key] = launch;
            pkg.lines.push_back("Launch control: launch rpm moved from " + std::to_string(static_cast<int>(rpm)) + " to " +
                                std::to_string(static_cast<int>(launch)) + " rpm (limiter " + std::to_string(static_cast<int>(eng.limiter_rpm)) + " rpm)");
            files.electronics.push_back(std::move(doc));
        }
    }
    // The car's current gearbox is not turned away by an engine swap, but the player is told when it is too weak.
    // A turbo kit counts with its estimated torque factor.
    const double boosted_peak = eng.peak_torque_nm * (files.kit_label.empty() ? 1.0 : std::max(1.0, files.kit_torque_factor));
    if ((files.swapped || !files.kit_label.empty()) && ctx.gearboxes && files.gearbox && boosted_peak > 0.0) {
        const GearboxEntry* g = ctx.gearboxes->find_by_file(files.gearbox->path);
        if (g != nullptr && !check_gearbox_rating(*g, boosted_peak).empty()) {
            char buf[200];
            std::snprintf(buf, sizeof(buf), "Gearbox: %s is rated %.0f N*m but this engine makes %s%.0f N*m - install a stronger one", g->name.c_str(), g->rating_nm,
                          files.kit_label.empty() ? "" : "about ", boosted_peak);
            pkg.lines.push_back(buf);
        }
    }

    const std::string comp_path = "/powertrain/components/";
    if (pkg.set_fuel && tank >= 0) {
        comps[tank]["fuel"] = ctx.engines->fuel_file(pkg.fuel);
        files.derived_subtrees.push_back(comp_path + std::to_string(tank) + "/fuel");
    }
    if (pkg.set_clutch && clutch >= 0) {
        comps[clutch]["capacity_nm"] = pkg.clutch_kinetic_nm;
        files.derived_subtrees.push_back(comp_path + std::to_string(clutch) + "/capacity_nm");
    }
    if (pkg.set_tcu) {
        const int tcu = controller_of_type(veh, "manual_tcu");
        if (tcu >= 0) {
            json& c = veh["controllers"][static_cast<std::size_t>(tcu)];
            const std::string base = "/controllers/" + std::to_string(tcu);
            json& ac = c["auto_clutch"];
            ac["bite_rpm"] = pkg.auto_clutch.bite_rpm;
            ac["full_rpm_min"] = pkg.auto_clutch.full_rpm_min;
            ac["full_rpm_max"] = pkg.auto_clutch.full_rpm_max;
            ac["anti_stall_rpm"] = pkg.auto_clutch.anti_stall_rpm;
            ac["rolling_reengage_rpm"] = pkg.auto_clutch.rolling_reengage_rpm;
            json& as = c["auto_shift"];
            as["up_rpm_at_zero_throttle"] = pkg.auto_shift.up_zero;
            as["up_rpm_at_full_throttle"] = pkg.auto_shift.up_full;
            as["down_rpm_at_zero_throttle"] = pkg.auto_shift.down_zero;
            as["down_rpm_at_full_throttle"] = pkg.auto_shift.down_full;
            files.derived_subtrees.push_back(base + "/auto_clutch");
            files.derived_subtrees.push_back(base + "/auto_shift");
            if (pkg.zero_blip) {
                json& ab = c["auto_blip"];
                ab["kp_per_rpm"] = 0.0;
                ab["max_throttle"] = 0.0;
                ab["upshift_torque_ramp_s"] = 0.0;
                files.derived_subtrees.push_back(base + "/auto_blip");
            }
        }
    }
    if (pkg.drop_slot_overrides && engine >= 0) {
        for (const char* key : kEngineSlotKeys) {
            if (comps[engine].contains(key)) {
                comps[engine].erase(key);
                files.derived_subtrees.push_back(comp_path + std::to_string(engine) + "/" + key);
            }
        }
    }
    if (pkg.drop_cooling_system) {
        // The new engine has no thermal data and the loader rejects a cooling_system without it: the car runs without
        // the thermal model (engine_thermal_start needs a cooling_system too).
        for (const char* key : {"cooling_system", "engine_thermal_start"}) {
            if (veh.contains(key)) {
                veh.erase(key);
                files.derived_subtrees.push_back(std::string("/") + key);
            }
        }
    }
    if (pkg.drop_nitrous) {
        std::set<std::string> gone;
        json kept = json::array();
        for (const json& c : comps) {
            std::string ty, id;
            detail::get_string(c, "type", ty);
            detail::get_string(c, "id", id);
            if (ty == "n2o_bottle" || ty == "nitrous_kit") gone.insert(id);
            else kept.push_back(c);
        }
        comps = std::move(kept);
        auto& pt = veh["powertrain"];
        if (pt.contains("fluid") && pt["fluid"].is_array()) {
            json fluid = json::array();
            for (const json& link : pt["fluid"]) {
                bool drop = false;
                if (link.is_array()) {
                    for (const json& port : link) {
                        if (!port.is_string()) continue;
                        const std::string s = port.get<std::string>();
                        const std::string owner = s.substr(0, s.find('.'));
                        if (gone.count(owner) != 0) drop = true;
                    }
                }
                if (!drop) fluid.push_back(link);
            }
            pt["fluid"] = std::move(fluid);
        }
        files.derived_subtrees.push_back("/powertrain/components");
        files.derived_subtrees.push_back("/powertrain/fluid");
    }
    return {};
}

// Wheels of an axle: {name, index, radius, width}.
// The part-mass tree of the files as they stand (rg/part_mass.h).
PartTree part_tree_of(const CatalogEntry& entry, const SetupContext& ctx, const Files& files) {
    PartTree t;
    if (!ctx.part_masses) {
        t.error = "no part mass table";
        return t;
    }
    if (!entry.chassis.body_mass_kg) {
        t.error = entry.id + ": the catalog gives no chassis.body_mass_kg";
        return t;
    }
    PartTreeInput in;
    in.body_kg = *entry.chassis.body_mass_kg;
    in.vehicle_json = files.vehicle.work.dump();
    in.vehicle_dir = dir_of(files.vehicle.path);
    in.engine_bay = entry.engine_bay;
    if (files.engine) {
        const EngineEntry* e = files.swapped ? &*files.swapped : (ctx.engines ? ctx.engines->find_by_file(files.engine->path) : nullptr);
        if (e != nullptr && e->mass_kg > 0.0) {
            in.engine = PartRef{e->name, e->mass_kg, "engine catalogue"};
        } else {
            double kg = 0.0;
            detail::get_number(files.engine->work, "mass_kg", kg);
            in.engine = PartRef{e != nullptr ? e->name : stem_of_path(files.engine->path), kg, "engine file mass_kg"};
        }
    }
    if (files.gearbox) {
        const GearboxEntry* g = ctx.gearboxes ? ctx.gearboxes->find_by_file(files.gearbox->path) : nullptr;
        if (g != nullptr) in.gearbox = PartRef{g->name, g->mass_kg, "gearbox catalogue"};
        else in.gearbox = PartRef{stem_of_path(files.gearbox->path), 0.0, ""};
    }
    in.extras = files.part_extras;
    // The installed engine's factory forced induction as its own parts below the (NA base) engine, sized from the
    // turbo configuration and its hardware file. A turbo kit a setup feature adds (an extra "engine.turbo_kit")
    // carries its own mass, so the configuration is not counted twice.
    const bool kit_extra = std::any_of(in.extras.begin(), in.extras.end(), [](const PartAttachment& a) {
        return a.parent_id == "engine" && a.node.id == "turbo_kit";
    });
    if (files.turbo && !kit_extra) {
        double count = 0.0;
        std::string hw_ref;
        detail::get_number(files.turbo->work, "count", count);
        detail::get_string(files.turbo->work, "turbo", hw_ref);
        json hw;
        if (count >= 1.0 && !hw_ref.empty() && read_doc(resolve_ref(dir_of(files.turbo->path), hw_ref), hw, nullptr)) {
            double inertia = 0.0, flow = 0.0;
            detail::get_number(hw, "rotor_inertia_kgm2", inertia);
            detail::get_number(hw, "max_mass_flow_kg_s", flow);
            TurboSystem t;
            t.count = static_cast<int>(count);
            t.label = t.count == 1 ? "Turbo system (single)" : t.count == 2 ? "Turbo system (twin)" : "Turbo system (" + std::to_string(t.count) + " units)";
            // Rotor size s relative to the turbo-kit anchor unit (rotor 2.3e-4 kg m2, the hyper's GTX3582R class).
            t.rotor_scale = inertia > 0.0 ? std::pow(inertia / 2.3e-4, 0.2) : 1.0;
            t.flow_kg_s = flow * t.count;
            t.source = "turbo configuration " + stem_of_path(files.turbo->path);
            in.turbo = t;
        }
    }
    in.trim = files.trim;
    in.stock_vehicle_json = files.vehicle.base.dump();
    return build_part_tree(*ctx.part_masses, in);
}

struct WheelInfo {
    std::string name;
    std::size_t index = 0;
    double radius = 0.0;
    double width = 0.0;
    bool is_front = false;
    std::optional<TyreSpec> tyre; // the wheel's "tyre_spec"
};

// The wheel's "tyre_spec" object, if it is a well-formed one.
std::optional<TyreSpec> tyre_spec_of(const json& wheel) {
    const auto it = wheel.find("tyre_spec");
    if (it == wheel.end() || !it->is_object()) return std::nullopt;
    TyreSpec s;
    double w = 0.0, a = 0.0, r = 0.0;
    if (!detail::get_string(*it, "compound", s.compound) || !detail::get_number(*it, "width_mm", w) || !detail::get_number(*it, "aspect", a) ||
        !detail::get_number(*it, "rim_in", r)) {
        return std::nullopt;
    }
    s.width_mm = static_cast<int>(w);
    s.aspect = static_cast<int>(a);
    s.rim_in = static_cast<int>(r);
    return s;
}

json tyre_spec_json(const TyreSpec& s) {
    json j = json::object();
    j["compound"] = s.compound;
    j["width_mm"] = s.width_mm;
    j["aspect"] = s.aspect;
    j["rim_in"] = s.rim_in;
    return j;
}

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
        info.tyre = tyre_spec_of(w);
        out.push_back(info);
    }
    return out;
}

// The fitment of an axle: the catalog entry's declaration, else only the stock tyre's own size.
TyreAxleFitment axle_fitment(const CatalogEntry& entry, const std::string& axle_name, const std::vector<WheelInfo>& axle) {
    const auto declared = entry.tyre_fitment.find(axle_name);
    if (declared != entry.tyre_fitment.end()) return declared->second;
    TyreAxleFitment f;
    if (!axle.empty() && axle.front().tyre) {
        f.rims_in = {axle.front().tyre->rim_in};
        f.min_width_mm = f.max_width_mm = axle.front().tyre->width_mm;
    }
    return f;
}

// The reason `tyre` does not fit a wheel of `axle`, or "".
std::string axle_fit_error(const TyreSpec& tyre, const TyreAxleFitment& fitment, const std::vector<WheelInfo>& axle) {
    for (const WheelInfo& w : axle) {
        const std::string e = tyre_fit_error(tyre, fitment, w.radius * 2000.0);
        if (!e.empty()) return e;
    }
    return {};
}

std::shared_ptr<const TyreFamilies> families_of(const SetupContext& ctx) {
    if (ctx.tyre_families_path.empty()) return nullptr;
    std::string err;
    return TyreFamilies::load(ctx.tyre_families_path, &err);
}

// Every tyre id the axle accepts: each compound at each size the fitment allows, plus the car's own tyre whatever it is.
std::vector<std::string> tyre_candidates(const SetupContext& ctx, const CatalogEntry& entry, const std::string& axle_name,
                                         const std::vector<WheelInfo>& axle) {
    std::vector<std::string> out;
    const auto families = families_of(ctx);
    if (axle.empty() || !families) return out;
    const TyreAxleFitment fitment = axle_fitment(entry, axle_name, axle);
    for (const TyreSpec& size : tyre_sizes_for(fitment, axle.front().radius * 2000.0)) {
        if (!axle_fit_error(size, fitment, axle).empty()) continue;
        for (const TyreCompound& c : families->compounds()) {
            TyreSpec s = size;
            s.compound = c.id;
            out.push_back(tyre_id(s));
        }
    }
    if (axle.front().tyre) {
        const std::string own = tyre_id(*axle.front().tyre);
        if (std::find(out.begin(), out.end(), own) == out.end()) out.push_back(own);
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
        case OptionKind::PartChoice: return "part_choice";
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

// ---- numbers in the player's units (garage numeric inputs) ---------------------------------------------------

NumberEntry number_entry(const OptionView& view, int index) {
    NumberEntry e;
    switch (view.def.kind) {
        case OptionKind::Scale: {
            e.unit = view.def.unit.empty() ? "x" : view.def.unit;
            e.lo = view.min;
            e.hi = view.max;
            if (view.def.slider) {
                const double stock = view.stock_numbers.empty() ? 0.0 : view.stock_numbers.front();
                e.multiplier = (stock > 0.0 ? stock : 1.0) * view.def.slider->factor;
                e.unit = view.def.slider->unit;
                e.decimals = view.def.slider->decimals;
                e.positional = true;
                e.mapping = view.def.slider->mapping;
                e.lo = view.min * e.multiplier;
                e.hi = view.max * e.multiplier;
            }
            break;
        }
        case OptionKind::ScaleList: {
            const double stock = index >= 0 && static_cast<std::size_t>(index) < view.stock_numbers.size() ? view.stock_numbers[index] : 0.0;
            e.multiplier = stock > 0.0 ? stock : 1.0;
            e.lo = view.min * e.multiplier;
            e.hi = view.max * e.multiplier;
            break;
        }
        case OptionKind::BrakeBias:
            e.multiplier = 100.0;
            e.unit = "%";
            e.decimals = 1;
            e.lo = view.min * 100.0;
            e.hi = view.max * 100.0;
            break;
        default: break;
    }
    return e;
}

double entry_to_displayed(const NumberEntry& entry, double stored) { return stored * entry.multiplier; }

std::string format_entry(const NumberEntry& entry, double stored) {
    if (!entry.positional && entry.unit == "x") return "x" + fmt_fixed(stored, entry.decimals);
    std::string text = fmt_fixed(stored * entry.multiplier, entry.decimals);
    if (!entry.unit.empty()) text += " " + entry.unit;
    return text;
}

double entry_slider_position(const NumberEntry& entry, double stored) {
    return slider_position(entry.mapping, entry.lo, entry.hi, stored * entry.multiplier);
}

double entry_value_at_position(const NumberEntry& entry, double position) {
    double displayed = slider_value(entry.mapping, entry.lo, entry.hi, position);
    displayed = round_to_decimals(displayed, entry.decimals);
    displayed = std::clamp(displayed, entry.lo, entry.hi);
    return displayed / entry.multiplier;
}

EntryConversion entry_from_displayed(const NumberEntry& entry, double displayed) {
    EntryConversion r;
    if (!std::isfinite(displayed)) {
        r.message = "that is not a number";
        return r;
    }
    const double eps = 1e-9 * std::max(1.0, std::abs(entry.hi));
    if (displayed < entry.lo - eps || displayed > entry.hi + eps) {
        r.message = fmt_fixed(displayed, entry.decimals) + (entry.unit.empty() ? "" : " " + entry.unit) +
                    " is outside the allowed range " + fmt_fixed(entry.lo, entry.decimals) + " to " + fmt_fixed(entry.hi, entry.decimals) +
                    (entry.unit.empty() ? "" : " " + entry.unit);
        return r;
    }
    r.ok = true;
    r.value = std::clamp(displayed, entry.lo, entry.hi) / entry.multiplier;
    return r;
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
        else if (kind == "part_choice") d.kind = OptionKind::PartChoice;
        else return bad(where + ": unknown kind \"" + kind + "\"");
        detail::get_number(o, "min", d.min);
        detail::get_number(o, "max", d.max);
        detail::get_number(o, "step", d.step);
        if (const auto slider = o.find("slider"); slider != o.end()) {
            if (d.kind != OptionKind::Scale) return bad(where + ": a \"slider\" block belongs to a scale option only");
            if (o.contains("min") || o.contains("max")) {
                return bad(where + ": a slider option takes its range from the slider block, not from \"min\"/\"max\"");
            }
            if (!slider->is_object()) return bad(where + ": \"slider\" must be an object");
            SliderSpec spec;
            std::string mapping;
            if (!detail::get_string(*slider, "mapping", mapping) || (mapping != "log" && mapping != "linear")) {
                return bad(where + ": slider \"mapping\" must be \"log\" or \"linear\"");
            }
            spec.mapping = mapping == "log" ? SliderMapping::Log : SliderMapping::Linear;
            if (!detail::get_number(*slider, "abs_min", spec.abs_min) || !detail::get_number(*slider, "abs_max", spec.abs_max)) {
                return bad(where + ": slider needs numbers abs_min and abs_max");
            }
            detail::get_number(*slider, "factor", spec.factor);
            double decimals = spec.decimals;
            if (detail::get_number(*slider, "decimals", decimals)) spec.decimals = static_cast<int>(decimals);
            if (static_cast<double>(spec.decimals) != decimals) return bad(where + ": slider decimals must be a whole number");
            if (!detail::get_string(*slider, "unit", spec.unit)) spec.unit = d.unit;
            const std::string spec_error = validate_slider_spec(spec);
            if (!spec_error.empty()) return bad(where + ": slider " + spec_error);
            d.slider = std::move(spec);
        }
        if (d.kind == OptionKind::BrakeBias) {
            // The brake bias limits are data (setup_options.json), never code: every one is required.
            if (!detail::get_number(o, "max_shift", d.max_shift)) return bad(where + ": brake_bias needs a number \"max_shift\"");
            if (!detail::get_number(o, "share_min", d.share_min)) return bad(where + ": brake_bias needs a number \"share_min\"");
            if (!detail::get_number(o, "share_max", d.share_max)) return bad(where + ": brake_bias needs a number \"share_max\"");
            if (!detail::get_number(o, "max_shift_bound", d.max_shift_bound)) {
                return bad(where + ": brake_bias needs a number \"max_shift_bound\"");
            }
        }
        detail::get_string(o, "file", d.file);
        detail::get_string(o, "model_option", d.model_option);
        if (d.file != "vehicle" && d.file != "gearbox" && d.file != "engine" && d.file != "turbo") {
            return bad(where + ": \"file\" must be vehicle, gearbox, engine or turbo");
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
        if (!d.model_option.empty() && (d.kind != OptionKind::Bool || !d.pointers.empty())) {
            return bad(where + ": a model_option is a bool that edits no file, so it has no pointers");
        }
        if (d.kind == OptionKind::PartChoice) {
            // A part choice edits no file: it swaps or removes one part of the part-mass tree.
            if (!d.pointers.empty()) return bad(where + ": a part choice edits no file, so it has no pointers");
            if (!detail::get_string(o, "slot", d.slot) || d.slot.empty()) return bad(where + ": a part choice needs a \"slot\"");
            if (!o.contains("parts") || !o["parts"].is_array() || o["parts"].empty()) return bad(where + ": parts missing");
            std::set<std::string> part_ids;
            for (const auto& part : o["parts"]) {
                SetupPart p;
                if (!part.is_object() || !detail::get_string(part, "id", p.id) || p.id.empty() || p.id == "stock" ||
                    !part_ids.insert(p.id).second || !detail::get_string(part, "label", p.label) || p.label.empty())
                    return bad(where + ": invalid or duplicate part (each needs an id other than \"stock\" and a label)");
                detail::get_string(part, "image", p.image);
                detail::get_string(part, "detail", p.detail);
                detail::get_bool(part, "removed", p.removed);
                detail::get_string(part, "mass_key", p.mass_key);
                if (part.contains("mass_factor") && (!detail::get_number(part, "mass_factor", p.mass_factor) ||
                                                     !(p.mass_factor > 0.0 && p.mass_factor <= 1.0)))
                    return bad(where + ": part \"" + p.id + "\": mass_factor must be in (0, 1]");
                const int ways = (p.removed ? 1 : 0) + (p.mass_key.empty() ? 0 : 1) + (part.contains("mass_factor") ? 1 : 0);
                if (ways != 1)
                    return bad(where + ": part \"" + p.id + "\" needs exactly one of mass_key, mass_factor or removed");
                d.parts.push_back(std::move(p));
            }
            table.options.push_back(std::move(d));
            continue;
        }
        if (d.kind != OptionKind::Colour && d.model_option.empty() && d.pointers.empty()) return bad(where + ": \"pointers\" (the whitelist) is empty");
        if (d.kind == OptionKind::Colour && !d.pointers.empty()) return bad(where + ": a colour edits no file, so it has no pointers");
        // A slider option's factor range comes from its absolute bounds and the car's stock value, not from min/max.
        if ((d.kind == OptionKind::Scale || d.kind == OptionKind::ScaleList) && !d.slider &&
            !(d.min > 0.0 && d.min <= 1.0 && d.max >= 1.0 && d.max < 10.0 && d.step > 0.0)) {
            return bad(where + ": a scale needs 0 < min <= 1 <= max < 10 and step > 0");
        }
        if (d.kind == OptionKind::BrakeBias) {
            if (!(d.max_shift_bound > 0.0 && d.max_shift_bound <= 0.5)) return bad(where + ": max_shift_bound must be in (0, 0.5]");
            if (!(d.max_shift > 0.0 && d.max_shift < d.max_shift_bound)) return bad(where + ": max_shift must be in (0, max_shift_bound)");
            if (!(d.share_min >= 0.0 && d.share_min < d.share_max && d.share_max <= 1.0)) {
                return bad(where + ": share_min and share_max must satisfy 0 <= share_min < share_max <= 1");
            }
        }
        if (d.kind == OptionKind::FileChoice) {
            if (d.file != "vehicle" && d.file != "engine") return bad(where + ": file choice must edit vehicle or engine");
            if (!detail::get_string(o, "part_format", d.part_format) || d.part_format.empty()) return bad(where + ": part_format missing");
            detail::get_bool(o, "no_turbo_override", d.no_turbo_override);
            if (o.contains("insert_if_absent")) {
                const json& ins = o["insert_if_absent"];
                SetupOptionDef::InsertIfAbsent v;
                if (!ins.is_object() || !ins.contains("entry") || !ins["entry"].is_object() ||
                    !detail::get_string(ins, "array", v.array) || v.array.empty() || v.array.front() != '/' ||
                    !detail::get_string(ins, "stock_label", v.stock_label) || v.stock_label.empty())
                    return bad(where + ": insert_if_absent needs array (a JSON pointer), entry (an object) and stock_label");
                detail::get_string(ins, "ref_key", v.ref_key);
                v.entry_json = ins["entry"].dump();
                d.insert_if_absent = std::move(v);
            }
            detail::get_string(o, "catalog", d.catalog);
            if (!d.catalog.empty() && d.catalog != "engines" && d.catalog != "fuels" && d.catalog != "gearboxes" &&
                d.catalog != "drive_layouts" && d.catalog != "differentials" && d.catalog != "turbo_kits" && d.catalog != "charge_piping" &&
                d.catalog != "engine_internals" && d.catalog != "cooling_upgrades") {
                return bad(where + ": \"catalog\" must be \"engines\", \"fuels\", \"gearboxes\", \"drive_layouts\", \"differentials\", \"turbo_kits\", \"charge_piping\", \"engine_internals\" or \"cooling_upgrades\"");
            }
            detail::get_string(o, "cooling_part", d.cooling_part);
            if ((d.catalog == "cooling_upgrades") != !d.cooling_part.empty()) {
                return bad(where + ": \"cooling_part\" (the cooling part kind) goes with the cooling_upgrades catalog, and only there");
            }
            if (d.catalog == "cooling_upgrades" && (d.file != "vehicle" || d.part_format != "rg.cooling_upgrades/1")) {
                return bad(where + ": a cooling option names the vehicle file with rg.cooling_upgrades/1 parts");
            }
            detail::get_string(o, "internal", d.internal);
            if ((d.catalog == "engine_internals") != !d.internal.empty()) {
                return bad(where + ": \"internal\" (the engine part kind) goes with the engine_internals catalog, and only there");
            }
            if (d.catalog == "engine_internals" && (d.file != "engine" || d.part_format != "rg.engine_internals/1")) {
                return bad(where + ": an engine internals option names the engine file with rg.engine_internals/1 parts");
            }
            if ((d.catalog == "turbo_kits" || d.catalog == "charge_piping") && (d.file != "engine" || d.part_format != "physics_sim.turbo_configuration/1")) {
                return bad(where + ": a turbo kit option edits the engine file with physics_sim.turbo_configuration/1 parts");
            }
            if ((d.catalog == "drive_layouts" || d.catalog == "differentials") && d.file != "vehicle") {
                return bad(where + ": a drivetrain option edits the vehicle file");
            }
            if (d.catalog == "differentials" && d.axle != "front" && d.axle != "rear") {
                return bad(where + ": a differential choice needs \"axle\": front or rear");
            }
            if (!d.catalog.empty()) {
                if (o.contains("parts") || o.contains("compatible_engines")) {
                    return bad(where + ": a catalog option takes its parts from the store, not from \"parts\"/\"compatible_engines\"");
                }
                table.options.push_back(std::move(d));
                continue;
            }
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

void migrate_setup_tyres(VehicleSetup& setup, const SetupOptionTable& table, const SetupContext& ctx) {
    const auto families = families_of(ctx);
    if (!families) return;
    for (auto& kv : setup.values) {
        const SetupOptionDef* def = table.find(kv.first);
        if (def == nullptr || def->kind != OptionKind::TyreChoice || !std::holds_alternative<std::string>(kv.second)) continue;
        const std::string mapped = migrate_tyre_id(*families, std::get<std::string>(kv.second));
        if (!mapped.empty()) kv.second = mapped;
    }
}

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
            case OptionKind::PartChoice:
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

namespace {

bool under_subtree(const std::string& path, const std::vector<std::string>& subtrees) {
    for (const std::string& s : subtrees) {
        if (path == s || (path.size() > s.size() && path.compare(0, s.size(), s) == 0 && path[s.size()] == '/')) return true;
    }
    return false;
}

} // namespace

std::string whitelist_violation_in(const std::string& base_json, const std::string& patch_json,
                                   const std::vector<std::string>& pointer_patterns, const std::vector<std::string>& derived_subtrees) {
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
        if (allowed.count(d) == 0 && !under_subtree(d, derived_subtrees)) return d;
    }
    return {};
}

std::string whitelist_violation(const std::string& base_json, const std::string& patch_json,
                                const std::vector<std::string>& pointer_patterns) {
    return whitelist_violation_in(base_json, patch_json, pointer_patterns, {});
}

// ---- the model (option views) ------------------------------------------------------------------

namespace {

std::string power_text(const EnginePeaks& p) {
    if (!p.known) return {};
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%.0f kW at %.0f rpm · %.0f N*m", p.power_kw, p.power_rpm, p.torque_nm);
    return buf;
}

// The approved parts of a file choice: the option table's own list, or - a catalog option - the store's.
std::vector<SetupPart> parts_of(const SetupOptionDef& def, const SetupContext& ctx, const Files& files,
                                 const CatalogEntry* entry_for_fit = nullptr) {
    if (def.catalog.empty()) return def.parts;
    std::vector<SetupPart> out;
    if (def.catalog == "drive_layouts") {
        if (!ctx.drivelines) return out;
        for (const DriveLayoutEntry& l : ctx.drivelines->layouts()) {
            SetupPart p;
            p.id = l.id;
            p.label = l.name;
            p.section = l.layout;
            p.subtitle = l.centre ? l.layout + " · " + diff_kind_label(l.centre->kind) + " centre" : l.layout;
            p.detail = l.description;
            out.push_back(std::move(p));
        }
        return out;
    }
    if (def.catalog == "differentials") {
        if (!ctx.drivelines) return out;
        for (const DiffUnit& u : ctx.drivelines->differentials()) {
            SetupPart p;
            p.id = u.id;
            p.label = u.name;
            p.subtitle = diff_kind_label(u.kind);
            p.detail = u.description;
            out.push_back(std::move(p));
        }
        return out;
    }
    if (def.catalog == "gearboxes") {
        if (!ctx.gearboxes) return out;
        const double peak = installed_engine_peak_torque(files, ctx);
        const std::string original = original_gearbox_path(files);
        for (const GearboxEntry& g : ctx.gearboxes->list()) {
            // The car's own box is always listed (it becomes "stock"); any other must take the engine's torque.
            if (norm_path(g.file) != original && !check_gearbox_rating(g, peak).empty()) continue;
            SetupPart p;
            p.id = g.id;
            p.label = g.name;
            p.path = g.file;
            p.image = "res://assets/components/drivetrain/gearbox.svg";
            p.section = gearbox_section(g);
            p.subtitle = gearbox_summary_line(g);
            p.detail = gearbox_detail_line(g);
            out.push_back(std::move(p));
        }
        return out;
    }
    if (def.catalog == "turbo_kits") {
        if (!ctx.turbo_kits || !files.engine) return out;
        const auto facts = installed_engine_facts(files, ctx, nullptr);
        if (!facts || !turbo_kit_refusal(*facts, {}).empty()) return out;
        const EngineEntry* eng = installed_engine_entry(files, ctx);
        for (const TurboKitSpec& k : ctx.turbo_kits->kits) {
            const TurboKitSizing z = size_turbo_kit(*ctx.turbo_kits, k, *facts, facts->limiter_rpm);
            if (!z.ok) continue;
            SetupPart p;
            p.id = k.id;
            p.label = k.label;
            p.path = "turbo_kit:" + k.id; // generated on install, never a stock file
            p.image = k.count > 1 ? "res://assets/components/engines/turbo_twin.png" : "res://assets/components/engines/turbo_single.png";
            p.detail = turbo_kit_detail(*ctx.turbo_kits, k, z);
            const TurboKitBuild b = describe_turbo_kit(*ctx.turbo_kits, k, z, *ctx.turbo_kits->find_piping(ctx.turbo_kits->default_piping));
            char mass[64];
            std::snprintf(mass, sizeof(mass), " · %.0f kg", b.total_mass_kg());
            p.detail += mass;
            if (eng != nullptr && eng->envelope_mm && entry_for_fit != nullptr) {
                const FitResult fit = check_fit_size(*entry_for_fit, turbo_kit_envelope_mm(*ctx.turbo_kits, k.count, z, *eng->envelope_mm, eng->envelope_layout == "v"));
                if (!fit.allowed) {
                    p.detail += " · does not fit the engine bay (" + fit.reasons.front() + ")";
                    p.fits = false;
                    p.fit_reason = k.label + " does not fit " + entry_for_fit->title + " (" + fit.reasons.front() + ")";
                }
            }
            out.push_back(std::move(p));
        }
        return out;
    }
    if (def.catalog == "engine_internals" && def.internal == kCylinderBoreKind) {
        if (!ctx.engine_internals || !ctx.engine_internals->bore || !files.engine) return out;
        const auto facts = read_engine_facts(files.engine->path, nullptr);
        if (!facts || !engine_internals_refusal(*facts, {}).empty()) return out;
        for (const BoreStep& st : ctx.engine_internals->bore->steps) {
            const auto g = bore_geometry(*ctx.engine_internals, *facts, st.id, nullptr);
            if (!g) continue;
            SetupPart p;
            p.id = st.id;
            p.label = st.label;
            p.path = "engine_bore:" + st.id; // edits the engine file's bore_mm and compression ratio
            p.detail = bore_step_detail(*g) + " · new pistons of this size";
            if (!g->fits) {
                p.fits = false;
                p.fit_reason = g->reason;
                p.detail += " · does not fit this block";
            }
            out.push_back(std::move(p));
        }
        return out;
    }
    if (def.catalog == "engine_internals") {
        if (!ctx.engine_internals || !files.engine) return out;
        const InternalPartKind* kind = ctx.engine_internals->find(def.internal);
        const auto facts = installed_engine_facts(files, ctx, nullptr); // a bored block: pistons of the new diameter
        if (kind == nullptr || !facts || !engine_internals_refusal(*facts, {}).empty()) return out;
        for (const InternalGrade& g : kind->grades) {
            if (g.id == "stock") continue; // the stock card (build_setup_model)
            SetupPart p;
            p.id = g.id;
            p.label = g.label;
            p.path = "engine_internals:" + kind->kind + ":" + g.id; // nothing is written: limits and masses only
            p.detail = internal_grade_detail(*ctx.engine_internals, *facts, kind->kind, g.id);
            out.push_back(std::move(p));
        }
        return out;
    }
    if (def.catalog == "cooling_upgrades") {
        if (!ctx.cooling_upgrades || !files.cooling_build) return out;
        const CoolingPart* part = ctx.cooling_upgrades->find(def.cooling_part);
        if (part == nullptr) return out;
        double hint = 0.0;
        if (files.engine) {
            if (const auto facts = read_engine_facts(files.engine->path, nullptr)) hint = facts->displacement_l;
        }
        for (const CoolingGrade& g : part->grades) {
            if (g.id == "stock") continue; // the stock card (build_setup_model)
            SetupPart p;
            p.id = g.id;
            p.label = g.label;
            p.path = "cooling_upgrades:" + part->kind + ":" + g.id; // the cooling files are patched on install
            p.detail = cooling_grade_detail(*ctx.cooling_upgrades, part->kind, g.id, files.cooling_effective, files.thermal_effective, hint);
            out.push_back(std::move(p));
        }
        return out;
    }
    if (def.catalog == "charge_piping") {
        if (!ctx.turbo_kits) return out;
        for (const ChargePipingSpec& m : ctx.turbo_kits->piping) {
            SetupPart p;
            p.id = m.id == ctx.turbo_kits->default_piping ? std::string("stock") : m.id;
            p.label = m.label + " charge piping";
            p.path = "charge_piping:" + m.id;
            char buf[160];
            std::snprintf(buf, sizeof(buf), "rated %.1f bar · bursts near %.0f bar · %.1f mm wall", m.rated_bar, m.burst_bar, m.wall_mm);
            p.detail = buf;
            if (m.id == ctx.turbo_kits->default_piping) continue; // the kit's own piping is the "stock" card
            out.push_back(std::move(p));
        }
        return out;
    }
    if (!ctx.engines) return out;
    if (def.catalog == "engines") {
        for (const EngineEntry& e : ctx.engines->list()) {
            SetupPart p;
            p.id = e.key;
            p.label = e.name;
            p.path = e.file;
            p.image = e.image;
            std::string detail = e.family;
            const std::string power = power_text(ctx.engines->peaks(e));
            if (!power.empty()) detail += (detail.empty() ? "" : " · ") + power;
            if (e.user) detail += std::string(detail.empty() ? "" : " · ") + "custom, based on " + e.base_name;
            p.detail = detail;
            if (entry_for_fit != nullptr) {
                const FitResult fit = check_fit(*entry_for_fit, e);
                if (!fit.allowed) {
                    std::string why;
                    for (const std::string& r : fit.reasons) why += (why.empty() ? "" : "; ") + r;
                    p.fits = false;
                    p.fit_reason = e.name + " does not fit " + entry_for_fit->title + (why.empty() ? std::string() : " (" + why + ")");
                }
            }
            out.push_back(std::move(p));
        }
    } else if (def.catalog == "fuels" && files.engine) {
        if (const auto facts = read_engine_facts(files.engine->path, nullptr)) {
            bool first = true;
            for (const FuelFact& f : facts->fuels) {
                SetupPart p;
                p.id = f.name;
                p.label = fuel_label(f.name) + (first ? " (reference fuel)" : "");
                p.path = ctx.engines->fuel_file(f.name);
                char buf[96];
                std::snprintf(buf, sizeof(buf), "%.1f MJ/kg · %.0f kg/m3", f.lhv_mj_kg, f.density_kg_m3);
                p.detail = buf;
                out.push_back(std::move(p));
                first = false;
            }
        }
    }
    return out;
}

// Drivetrain swap: the stock part and the choices of a layout or differential option. Stock = what the car has now
// (the layout as delivered; a differential: the axle's unit as the chosen layout left it), never offered again.
void driveline_choices(const SetupOptionDef& def, const SetupContext& ctx, const Files& files, OptionView& v) {
    const std::vector<SetupPart> all = parts_of(def, ctx, files);
    SetupPart stock;
    stock.id = "stock";
    std::string stock_part;
    if (def.catalog == "drive_layouts") {
        const DrivelineInfo base = detail::describe_driveline_json(files.vehicle.base);
        stock.label = "Original: " + driveline_summary(base);
        stock.section = base.layout;
        if (ctx.drivelines && base.swappable) {
            const DriveLayoutEntry* l = nullptr;
            if (base.layout == "AWD") {
                l = ctx.drivelines->awd_layout_of_centre(base.centre_kind);
            } else {
                for (const DriveLayoutEntry& e : ctx.drivelines->layouts()) if (e.layout == base.layout && l == nullptr) l = &e;
            }
            if (l != nullptr) stock_part = l->id;
        }
    } else {
        const DrivelineInfo now = files.layout_info ? *files.layout_info : detail::describe_driveline_json(files.vehicle.base);
        const std::string& kind = def.axle == "front" ? now.front_kind : now.rear_kind;
        stock.label = kind.empty() ? std::string("Not driven") : "Original: " + diff_kind_label(kind);
        if (ctx.drivelines && !kind.empty()) {
            if (const DiffUnit* u = ctx.drivelines->differential_of_kind(kind)) stock_part = u->id;
        }
    }
    for (const SetupPart& p : all) {
        if (p.id != stock_part) continue;
        stock = p; // the catalogue's card for what is fitted (the car keeps its own parameters of that kind)
        stock.id = "stock";
        stock.subtitle = p.subtitle.empty() ? std::string("fitted") : p.subtitle + " · fitted";
    }
    v.parts.push_back(stock);
    v.choices.push_back("stock");
    for (const SetupPart& p : all) {
        if (p.id == stock_part) continue;
        v.parts.push_back(p);
        v.choices.push_back(p.id);
    }
}

// A part choice (weight reduction): swaps, lightens or removes the fitted part of the option's slot.
std::string apply_part_choice(Files& files, const SetupOptionDef& def, const SetupValue& value) {
    if (!std::holds_alternative<std::string>(value)) return "option \"" + def.id + "\" needs a part id";
    const std::string& id = std::get<std::string>(value);
    if (id == "stock") return {};
    const auto fitted = std::find_if(files.trim.begin(), files.trim.end(), [&](const FittedPart& f) { return f.slot == def.slot; });
    if (fitted == files.trim.end()) return def.label + ": this car has no " + def.slot + " to change";
    const auto part = std::find_if(def.parts.begin(), def.parts.end(), [&](const SetupPart& p) { return p.id == id; });
    if (part == def.parts.end()) return def.label + ": unknown part \"" + id + "\"";
    if (part->removed) {
        files.trim.erase(fitted);
    } else if (!part->mass_key.empty()) {
        fitted->key = part->mass_key;
        fitted->factor = 1.0;
        fitted->label = part->label;
    } else {
        fitted->factor = part->mass_factor;
        fitted->label = part->label;
    }
    return {};
}

} // namespace

SetupModel build_setup_model(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx, const VehicleSetup* installed) {
    SetupModel model;
    Files files;
    if (!load_files(entry, files, &model.error)) return model;
    if (installed) {
        for (const auto& def : table.options) {
            if (def.kind != OptionKind::FileChoice && def.kind != OptionKind::PartChoice) continue;
            const auto it = installed->values.find(def.id);
            if (it == installed->values.end()) continue;
            if (def.kind == OptionKind::PartChoice) {
                if (entry_offers(entry, def.id) && entry.interior.count(def.slot) != 0) {
                    model.error = apply_part_choice(files, def, it->second);
                    if (!model.error.empty()) return model;
                }
                continue;
            }
            model.error = install_file_part(files, def, entry, ctx, it->second);
            if (!model.error.empty()) return model;
        }
    }
    {
        SwapPackage pkg;
        model.error = apply_swap(files, entry, ctx, pkg);
        if (!model.error.empty()) return model;
        model.swap_lines = pkg.lines;
        model.swap_lines.insert(model.swap_lines.end(), files.gearbox_lines.begin(), files.gearbox_lines.end());
        model.chassis_mass_delta_kg = pkg.mass_delta_kg;
        if (files.swapped) model.engine_title = files.swapped->name;
        if (files.swapped) {
            model.engine_key = files.swapped->key;
        } else if (ctx.engines) {
            if (const EngineEntry* own = ctx.engines->find_by_file(files.original_engine_path)) model.engine_key = own->key;
        }
    }
    if (!load_turbo(files, &model.error)) return model;
    refresh_internals_parts(files, ctx); // after the swap: the limits and masses of the INSTALLED engine
    {
        std::map<std::string, std::string> cooling_options;
        for (const SetupOptionDef& def : table.options) {
            if (def.catalog == "cooling_upgrades" && entry_offers(entry, def.id)) cooling_options.emplace(def.cooling_part, def.id);
        }
        refresh_cooling(files, ctx, cooling_options); // the installed engine's thermal file and the car's cooling_system
    }
    model.parts = part_tree_of(entry, ctx, files);
    model.driveline = detail::describe_driveline_json(files.vehicle.work);
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
                    // The working file: after a swap the derived values (a stronger clutch) are the ones scaled.
                    const auto* leaf = at_pointer(target.file->work, target.pointer);
                    if (leaf && leaf->is_number()) v.stock_numbers.push_back(leaf->get<double>());
                }
                if (def.slider) {
                    const auto absolute = range_for(entry, def, v.stock_numbers.empty() ? 0.0 : v.stock_numbers.front());
                    v.min = absolute.first;
                    v.max = absolute.second;
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
                    v.min = std::max(def.share_min, share - def.max_shift);
                    v.max = std::min(def.share_max, share + def.max_shift);
                }
                if (total <= 0.0) v.available = false;
                break;
            }
            case OptionKind::FileChoice: {
                v.stock = std::string("stock");
                v.available = file_choice_fits(files, def, entry, ctx);
                if (is_driveline_catalog(def)) {
                    driveline_choices(def, ctx, files, v);
                    if (v.choices.size() < 2) v.available = false;
                    break;
                }
                SetupPart stock;
                stock.id = "stock";
                stock.label = "Original component";
                std::string original;
                const FileDoc* original_file = def.file == "vehicle" ? &files.vehicle : (files.engine ? &*files.engine : nullptr);
                if (def.insert_if_absent && original_file && expand_patterns(original_file->base, v.def.pointers).empty()) {
                    stock.label = def.insert_if_absent->stock_label; // nothing fitted as delivered
                } else if (original_file) {
                    const auto pointers = expand_patterns(original_file->base, v.def.pointers);
                    if (!pointers.empty()) {
                        const auto* leaf = at_pointer(original_file->base, pointers.front());
                        if (leaf && leaf->is_string()) original = resolve_ref(dir_of(original_file->path), leaf->get<std::string>());
                    }
                }
                const std::vector<SetupPart> candidates = parts_of(def, ctx, files, &entry);
                for (const auto& part : candidates) if (norm_path(part.path) == original) { stock = part; stock.id = "stock"; }
                v.parts.push_back(stock);
                v.choices.push_back("stock");
                for (const auto& part : candidates) {
                    if (norm_path(part.path) == original) continue;
                    v.parts.push_back(part);
                    v.choices.push_back(part.id);
                    // Physical fit is the one thing that turns an engine away: it stays listed and says why.
                    const EngineEntry* engine = def.catalog == "engines" && ctx.engines ? ctx.engines->find(part.id) : nullptr;
                    if (engine == nullptr) continue;
                    const FitResult fit = check_fit(entry, *engine);
                    std::string why;
                    for (const std::string& r : fit.reasons) why += (why.empty() ? "" : "; ") + r;
                    if (!why.empty()) v.parts.back().detail += (v.parts.back().detail.empty() ? "" : " · ") + std::string("Does not fit: ") + why;
                }
                if (v.choices.size() < 2) v.available = false;
                if (def.catalog == "turbo_kits") {
                    v.parts.front().label = "No turbo (as built)";
                    if (!v.available && entry_offers(entry, def.id)) v.unavailable_reason = turbo_kit_blocker(files, ctx);
                }
                if (def.catalog == "engine_internals" && def.internal == kCylinderBoreKind) {
                    const auto facts = files.engine ? read_engine_facts(files.engine->path, nullptr) : std::nullopt;
                    v.parts.front().label = "Stock bore";
                    v.parts.front().path = "engine_bore:stock";
                    if (facts && ctx.engine_internals) {
                        if (const auto g = bore_geometry(*ctx.engine_internals, *facts, "stock", nullptr)) {
                            char buf[64];
                            std::snprintf(buf, sizeof(buf), " · takes at most +%.1f mm", g->max_oversize_mm);
                            v.parts.front().detail = bore_step_detail(*g) + buf;
                        }
                    }
                    if (!v.available && entry_offers(entry, def.id)) {
                        v.unavailable_reason = internals_blocker(files, ctx);
                        if (v.unavailable_reason.empty()) v.unavailable_reason = "No bore step fits this engine.";
                    }
                } else if (def.catalog == "engine_internals") {
                    const InternalPartKind* kind = ctx.engine_internals ? ctx.engine_internals->find(def.internal) : nullptr;
                    const auto facts = files.engine ? installed_engine_facts(files, ctx, nullptr) : std::nullopt;
                    if (kind != nullptr) {
                        v.parts.front().label = kind->grades.front().label;
                        v.parts.front().path = "engine_internals:" + kind->kind + ":stock";
                        if (facts && engine_internals_refusal(*facts, {}).empty()) {
                            v.parts.front().detail = internal_grade_detail(*ctx.engine_internals, *facts, kind->kind, "stock");
                        }
                    }
                    if (!v.available && entry_offers(entry, def.id)) v.unavailable_reason = internals_blocker(files, ctx);
                }
                if (def.catalog == "cooling_upgrades") {
                    const CoolingPart* part = ctx.cooling_upgrades ? ctx.cooling_upgrades->find(def.cooling_part) : nullptr;
                    if (part != nullptr) {
                        v.parts.front().label = part->grades.front().label;
                        v.parts.front().path = "cooling_upgrades:" + part->kind + ":stock";
                        v.parts.front().detail = part->grades.front().source;
                    }
                    if (!v.available && entry_offers(entry, def.id)) {
                        v.unavailable_reason = files.cooling_blocker.empty() ? std::string("No cooling upgrades are available.") : files.cooling_blocker;
                    }
                }
                if (def.catalog == "charge_piping" && ctx.turbo_kits) {
                    const ChargePipingSpec* m = ctx.turbo_kits->find_piping(ctx.turbo_kits->default_piping);
                    v.parts.front().label = m->label + " charge piping (comes with the kit)";
                    char buf[160];
                    std::snprintf(buf, sizeof(buf), "rated %.1f bar · bursts near %.0f bar · %.1f mm wall", m->rated_bar, m->burst_bar, m->wall_mm);
                    v.parts.front().detail = buf;
                    v.parts.front().path = "charge_piping:" + m->id;
                    if (!v.available && entry_offers(entry, def.id)) v.unavailable_reason = "Charge piping comes with a turbo kit: " + turbo_kit_blocker(files, ctx);
                }
                if (def.catalog == "gearboxes" && ctx.gearboxes) {
                    for (const GearboxSection& sec : ctx.gearboxes->sections()) if (!sec.has_entries) v.coming_soon.push_back(sec.label);
                }
                break;
            }
            case OptionKind::TyreChoice: {
                const std::vector<WheelInfo> axle = wheels_of(files.vehicle.work, def.axle == "front");
                if (!axle.empty() && axle.front().tyre) v.stock = tyre_id(*axle.front().tyre);
                v.choices = tyre_candidates(ctx, entry, def.axle, axle);
                if (axle.empty() || !axle.front().tyre || v.choices.size() < 2) v.available = false;
                break;
            }
            case OptionKind::Bool: {
                if (!def.model_option.empty()) { // a visual part: off as delivered, offered where the model has it
                    v.stock = false;
                    if (std::find(entry.model_options.begin(), entry.model_options.end(), def.model_option) == entry.model_options.end()) {
                        v.available = false;
                    }
                    break;
                }
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
            case OptionKind::PartChoice: {
                v.stock = std::string("stock");
                const auto slot = entry.interior.find(def.slot);
                const PartMassDef* own = (slot != entry.interior.end() && ctx.part_masses) ? ctx.part_masses->find(slot->second) : nullptr;
                if (own == nullptr) {
                    v.available = false;
                    break;
                }
                const auto kg = [](double m) { return fmt_fixed(m, 1) + " kg"; };
                SetupPart stock;
                stock.id = "stock";
                stock.label = own->label;
                stock.detail = kg(own->mass_kg);
                stock.mass_key = own->key;
                v.parts.push_back(stock);
                v.choices.push_back("stock");
                for (const SetupPart& p : def.parts) {
                    if (!p.mass_key.empty() && p.mass_key == own->key) continue; // the car's own part
                    SetupPart q = p;
                    double m = 0.0;
                    if (p.removed) {
                        m = 0.0;
                    } else if (!p.mass_key.empty()) {
                        const PartMassDef* d = ctx.part_masses->find(p.mass_key);
                        if (d == nullptr) continue; // no figure: not offered (the tree would refuse it)
                        m = d->mass_kg;
                    } else {
                        m = own->mass_kg * p.mass_factor;
                    }
                    const double saved = own->mass_kg - m;
                    q.detail = (p.removed ? std::string("removed") : kg(m)) + (saved >= 0.0 ? ", saves " + kg(saved) : ", adds " + kg(-saved));
                    v.parts.push_back(q);
                    v.choices.push_back(q.id);
                }
                if (v.choices.size() < 2) v.available = false;
                break;
            }
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

PartTree stock_part_tree(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx) {
    SetupModel m = build_setup_model(entry, table, ctx);
    if (!m.error.empty()) {
        PartTree t;
        t.error = m.error;
        return t;
    }
    return std::move(m.parts);
}

CompiledSetup compile_setup_internal(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx,
                                     const VehicleSetup& setup, Files& files);

namespace {

// Applies one option's value to `files`; "" when fine, else the broken rule.
std::string apply_option(const CatalogEntry& entry, const SetupOptionDef& def, const SetupValue& value,
                         const SetupContext& ctx, Files& files, CompiledSetup& out,
                         std::vector<std::pair<const FileDoc*, std::string>>& touched) {
    const std::string id = def.id;
    if (!entry_offers(entry, id)) return "option \"" + id + "\" is not offered for " + entry.id;
    // Retain boost calibration as dormant when the player swaps to NA.
    if (def.file == "turbo" && !files.turbo) return {};
    const std::vector<std::string> patterns = pointers_for(entry, def);
    const auto range = range_for(entry, def);
    const auto type_error = [&](const char* expected) { return "option \"" + id + "\" needs " + expected; };

    switch (def.kind) {
        case OptionKind::FileChoice: return install_file_part(files, def, entry, ctx, value);
        case OptionKind::PartChoice: return apply_part_choice(files, def, value);
        case OptionKind::Scale: {
            if (!std::holds_alternative<double>(value)) return type_error("a number");
            const double s = std::get<double>(value);
            std::vector<Target> targets = targets_of(files, def, patterns);
            if (targets.empty()) return "option \"" + id + "\" matches nothing in " + entry.id + "'s files";
            double lo = range.first, hi = range.second;
            double stock_file = 0.0;
            if (def.slider) {
                const json* first = at_pointer(targets.front().file->work, targets.front().pointer);
                if (first != nullptr && first->is_number()) stock_file = first->get<double>();
                const auto absolute = range_for(entry, def, stock_file);
                lo = absolute.first;
                hi = absolute.second;
            }
            if (!std::isfinite(s) || s < lo - kEps || s > hi + kEps) {
                if (def.slider && entry.setup_ranges.count(id) == 0 && stock_file > 0.0 && std::isfinite(s)) {
                    const double m = stock_file * def.slider->factor;
                    const int dec = def.slider->decimals;
                    return id + ": " + fmt_fixed(s * m, dec) + " " + def.slider->unit + " is outside the allowed range " +
                           fmt_fixed(lo * m, dec) + " to " + fmt_fixed(hi * m, dec) + " " + def.slider->unit;
                }
                return id + ": " + fmt2(s) + " is outside the allowed range " + fmt2(lo) + " to " + fmt2(hi);
            }
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
                lo = std::max(def.share_min, stock - def.max_shift);
                hi = std::min(def.share_max, stock + def.max_shift);
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
            const std::string& name = std::get<std::string>(value);
            const std::vector<WheelInfo> axle = wheels_of(files.vehicle.work, def.axle == "front");
            if (axle.empty() || !axle.front().tyre) return "option \"" + id + "\" matches nothing in " + entry.id + "'s files";
            const auto families = families_of(ctx);
            if (!families) return id + ": the tyre families file is unavailable";
            const std::optional<TyreSpec> spec = parse_tyre_id(name);
            if (!spec) return id + ": unknown tyre \"" + name + "\" (a tyre is named like road_265_35r20: compound, width, aspect and rim)";
            if (families->find(spec->compound) == nullptr) return id + ": unknown tyre compound \"" + spec->compound + "\"";
            // The car's own tyre is always allowed; anything else must fit the axle by its dimensions.
            if (!(*spec == *axle.front().tyre)) {
                const std::string reason = axle_fit_error(*spec, axle_fitment(entry, def.axle, axle), axle);
                if (!reason.empty()) return id + ": " + reason;
            }
            const double selected_width = spec->width_mm / 1000.0;
            for (const auto& wheel : axle) {
                auto& target = files.vehicle.work["wheels"][wheel.index];
                if (wheel.width <= 0.0) return id + ": invalid original wheel width";
                target["tyre_spec"] = tyre_spec_json(*spec);
                target["wheel_width"] = selected_width;
                if (target.contains("wheel_inertia") && target["wheel_inertia"].is_number())
                    target["wheel_inertia"] = target["wheel_inertia"].get<double>() * selected_width / wheel.width;
            }
            return {};
        }
        case OptionKind::Bool: {
            if (!std::holds_alternative<bool>(value)) return type_error("true or false");
            if (!def.model_option.empty()) {
                if (std::find(entry.model_options.begin(), entry.model_options.end(), def.model_option) == entry.model_options.end()) {
                    return id + ": " + entry.id + "'s model has no " + def.model_option;
                }
                if (std::get<bool>(value)) out.model_options.push_back(def.model_option);
                return {};
            }
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
                                     const VehicleSetup& setup_in, Files& files) {
    CompiledSetup out;
    VehicleSetup setup = setup_in;
    migrate_setup_tyres(setup, table, ctx); // pre-catalogue tyre names -> catalogue ids
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
    // Pass 1: tyre and part choices first (they decide which files the other options edit).
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
    // The swap package (engine garage G1): fuel, clutch, TCU, parts and mass follow an engine/fuel choice.
    {
        SwapPackage pkg;
        err = apply_swap(files, entry, ctx, pkg);
        if (!err.empty()) {
            out.error = err;
            return out;
        }
        out.swap_lines = pkg.lines;
        out.swap_lines.insert(out.swap_lines.end(), files.gearbox_lines.begin(), files.gearbox_lines.end());
        out.chassis_mass_delta_kg = pkg.mass_delta_kg;
        if (files.swapped) out.engine_title = files.swapped->name;
    }
    if (!load_turbo(files, &err)) { out.error = err; return out; }
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

    // A garage turbo kit: the boost slider moved the target away from the kit's starting one, so the hardware, the boost
    // grid and the fit are redone for the target actually set (the regenerated configuration carries that target).
    if (files.kit_spec && ctx.turbo_kits && files.engine) {
        TurboKitSpec spec = *files.kit_spec;
        if (files.turbo) {
            const double target = files.turbo->work.value("target_boost_bar", spec.target_boost_bar);
            if (std::abs(target - spec.target_boost_bar) > 1.0e-9) {
                spec.target_boost_bar = target;
                const std::string why = fit_turbo_kit(files, entry, ctx, spec);
                if (!why.empty()) {
                    out.error = "turbo_boost_target: " + why;
                    return out;
                }
                if (!load_turbo(files, &err)) {
                    out.error = err;
                    return out;
                }
            }
        }
        out.turbo_kit = refresh_kit_parts(files, ctx);
    }

    // Merge patches (the mechanism) and the whitelist check on the result.
    // Engine-affecting garage edits request interpolation-driven maps. Bounds
    // follow this engine's grid and the selected turbo target, in absolute kPa.
    bool adaptive_pressure = false;
    if (files.engine && files.engine->work.contains("cycle")) {
        for (const auto& def : table.options) {
            if (setup.values.count(def.id) && def.catalog != "engine_internals" && (def.file == "engine" || def.file == "turbo" || def.part_format == "physics_sim.turbo_configuration/1" || (def.part_format == "physics_sim.engine/1" && def.catalog.empty()))) adaptive_pressure = true;
        }
        if (adaptive_pressure) {
            auto& cycle = files.engine->work["cycle"];
            const auto& grid = cycle.at("grid_p_im_kpa");
            json sampling = cycle.value("pressure_sampling", json::object());
            double maximum = std::max(grid.back().get<double>(), sampling.value("max_kpa", 0.0));
            if (files.turbo) maximum = std::max(maximum, 101.325 + files.turbo->work.at("target_boost_bar").get<double>() * 100.0);
            sampling["enabled"] = true;
            if (!sampling.contains("min_kpa")) sampling["min_kpa"] = grid.front();
            sampling["max_kpa"] = maximum;
            if (!sampling.contains("relative_error")) sampling["relative_error"] = 0.05;
            if (!sampling.contains("max_points")) sampling["max_points"] = 24;
            if (!sampling.contains("max_passes")) sampling["max_passes"] = 8;
            sampling["allow_budget_limit"] = true;
            cycle["pressure_sampling"] = std::move(sampling);
        }
    }
    std::map<std::string, std::string> cooling_options;
    for (const SetupOptionDef& def : table.options) {
        if (def.catalog == "cooling_upgrades" && entry_offers(entry, def.id)) cooling_options.emplace(def.cooling_part, def.id);
    }
    if (const std::string why = refresh_cooling(files, ctx, cooling_options); !why.empty()) {
        out.error = "cooling: " + why;
        return out;
    }
    if (files.thermal && make_merge_patch(files.thermal->base, files.thermal->work) != json::object()) {
        // An engine thermal edit goes through the vehicle's engine component "thermal" override (physics_sim: path relative to
        // the vehicle file, wins over the engine file's default_thermal, which an extending user engine does not even carry).
        const int idx = component_index(files.vehicle.work, "engine");
        if (idx < 0) {
            out.error = "cooling: the car has no engine component to carry the thermal file";
            return out;
        }
        files.vehicle.work["powertrain"]["components"][static_cast<std::size_t>(idx)]["thermal"] = files.thermal->path;
        files.derived_subtrees.push_back("/powertrain/components");
    }
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
            if (def.insert_if_absent) patterns.push_back(def.insert_if_absent->array);
        }
        if (std::string(kind) == "engine") {
            if (adaptive_pressure) patterns.push_back("/cycle/pressure_sampling");
            for (const auto& def : table.options) {
                const auto selected = setup.values.find(def.id);
                if (!entry_offers(entry, def.id) || selected == setup.values.end() || !std::holds_alternative<std::string>(selected->second)) continue;
                for (const auto& part : def.parts) if (part.naturally_aspirated && part.id == std::get<std::string>(selected->second)) {
                    patterns.push_back("/default_forced_induction");
                    patterns.push_back("/cycle/grid_p_im_kpa");
                }
            }
        }
        json whitelist_base = f.base;
        // This generator policy is constructed above, never supplied by a
        // player option. Check every remaining edit against the normal list.
        if (adaptive_pressure && std::string(kind) == "engine") whitelist_base["cycle"]["pressure_sampling"] = f.work["cycle"]["pressure_sampling"];
        // The turbo kit conversion (install_catalog_part, trusted code driven by the turbo_kits option) sets these.
        if (!files.kit_label.empty() && std::string(kind) == "engine") {
            whitelist_base["default_forced_induction"] = f.work["default_forced_induction"];
            whitelist_base["cycle"]["grid_p_im_kpa"] = f.work["cycle"]["grid_p_im_kpa"];
            if (f.work["cycle"].contains("compression_ratio")) whitelist_base["cycle"]["compression_ratio"] = f.work["cycle"]["compression_ratio"];
        }
        const std::string violation = whitelist_violation_in(whitelist_base.dump(), patch_text(patch), patterns,
                                                              is_vehicle ? files.derived_subtrees : std::vector<std::string>{});
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
    if (files.turbo && !finish_file("turbo", *files.turbo, false)) return out;
    // Trusted swap-package edits (no player option touches an electronics part, so no whitelist).
    for (const FileDoc& f : files.electronics) {
        const json patch = make_merge_patch(f.base, f.work);
        if (!(patch.is_object() && patch.empty())) out.patches[basename_key("electronics", f)] = patch_text(patch);
    }
    // Cooling system / engine thermal edits of the cooling_upgrades options (trusted code: the grades are the whitelist).
    if (files.cooling) {
        const json patch = make_merge_patch(files.cooling->base, files.cooling->work);
        if (!(patch.is_object() && patch.empty())) out.patches[basename_key("cooling", *files.cooling)] = patch_text(patch);
    }
    if (files.thermal) {
        const json patch = make_merge_patch(files.thermal->base, files.thermal->work);
        if (!(patch.is_object() && patch.empty())) out.patches[basename_key("thermal", *files.thermal)] = patch_text(patch);
    }
    out.cooling = files.cooling_build;
    {
        // Engine internals for the installed engine (after any swap), each part naming the option that swaps it.
        std::map<std::string, std::string> internal_options;
        for (const SetupOptionDef& def : table.options) {
            if (def.catalog == "engine_internals" && entry_offers(entry, def.id)) internal_options.emplace(def.internal, def.id);
        }
        out.internals = refresh_internals_parts(files, ctx, internal_options);
    }
    out.parts = part_tree_of(entry, ctx, files);
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
        json doc = f.base;
        doc.merge_patch(patch);
        absolutise_refs(doc, dir_of(f.path));
        const json before_refs = doc;
        replace_strings(doc, moved);
        if (patch.is_object() && patch.empty() && doc == before_refs) return true;
        const std::string path = norm_path((fs::path(out_dir) / (std::string(kind) + "_" + stem_of(f.path) + ".json")).generic_string());
        if (!write_text(path, doc.dump(2) + "\n", &out.error)) return false;
        out.files.push_back(path);
        moved[f.path] = path;
        return true;
    };
    // The cooling_system and engine thermal copies come first: the engine and vehicle files name them (moved map).
    if (files.cooling && !write_aux("cooling", *files.cooling)) return out;
    if (files.thermal && !write_aux("thermal", *files.thermal)) return out;
    if (files.turbo && !write_aux("turbo", *files.turbo)) return out;
    if (files.gearbox && !write_aux("gearbox", *files.gearbox)) return out;
    if (files.engine && !write_aux("engine", *files.engine)) return out;
    for (const FileDoc& f : files.electronics) {
        if (!write_aux("electronics", f)) return out;
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
                                const VehicleSetup& setup, bool generate_maps) {
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
        options.generate = generate_maps;
        (void)rg::load_vehicle(m.vehicle_path, options);
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

namespace rg {

std::vector<SetupPart> visible_parts(const OptionView& view, bool show_all) {
    std::vector<SetupPart> out;
    for (const SetupPart& p : view.parts) {
        if (show_all || p.fits || p.id == "stock") out.push_back(p);
    }
    return out;
}

std::string hidden_parts_reason(const OptionView& view) {
    std::string first;
    bool any = false;
    for (const SetupPart& p : view.parts) {
        if (p.id == "stock") continue;
        any = true;
        if (p.fits) return {};
        if (first.empty()) first = p.fit_reason;
    }
    return any ? first : std::string();
}

} // namespace rg
