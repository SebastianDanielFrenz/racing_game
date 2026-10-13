// rg/vehicle_setup.cpp - see vehicle_setup.h.
#include "rg/vehicle_setup.h"

#include "rg/engine_swap.h"
#include "rg/tyre_spec.h"
#include "rg/vehicle_load.h"

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
    std::optional<FileDoc> turbo;
    std::string original_engine_path;
    // Engine garage G1. The engine whose options (turbo, nitrous) apply is the INSTALLED one: engine_stem is the
    // catalog stem of the default engine it is (a user engine: its base), the car's own at load.
    std::string engine_stem;
    std::optional<EngineEntry> swapped; // set when engine_install put another engine in
    std::string fuel_choice;            // set when the fuel option picked a fuel
    // Vehicle-file subtrees the swap package (trusted code, not a player option) edited; exempt from the whitelist.
    std::vector<std::string> derived_subtrees;
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

std::pair<double, double> range_for(const CatalogEntry& entry, const SetupOptionDef& def) {
    const auto it = entry.setup_ranges.find(def.id);
    if (it != entry.setup_ranges.end()) return it->second;
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

// File choices are never a filesystem picker: only declared compatible IDs can resolve a path.
bool file_choice_fits(Files& files, const SetupOptionDef& def, const CatalogEntry& entry, const SetupContext& ctx) {
    if (!entry_offers(entry, def.id)) return false;
    if (def.catalog.empty()) {
        // "*" in compatible_engines: the part does not depend on the engine (e.g. the assist electronics).
        const std::string& engine = files.engine_stem;
        const bool any_engine = std::find(def.compatible_engines.begin(), def.compatible_engines.end(), "*") != def.compatible_engines.end();
        if (!any_engine && std::find(def.compatible_engines.begin(), def.compatible_engines.end(), engine) == def.compatible_engines.end()) return false;
    } else if (def.catalog == "gearboxes" ? !ctx.gearboxes : !ctx.engines) {
        return false;
    }
    const int idx = component_index(files.vehicle.base, "engine");
    if (def.no_turbo_override && idx >= 0 && files.vehicle.base["powertrain"]["components"][idx].contains("forced_induction")) return false;
    const json* base = def.file == "vehicle" ? &files.vehicle.base : (files.engine ? &files.engine->base : nullptr);
    return base && !expand_patterns(*base, pointers_for(entry, def)).empty();
}

// Engine garage G1: a choice from the engine store ("engines") or the installed engine's fuels ("fuels").
std::string install_catalog_part(Files& files, const SetupOptionDef& def, const CatalogEntry& entry, const SetupContext& ctx,
                                 const std::string& id) {
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
        if (g->kind != GearboxKind::Manual && g->kind != GearboxKind::Sequential) {
            return def.id + ": " + g->name + " cannot be simulated yet";
        }
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
    if (!file_choice_fits(files, def, entry, ctx)) return def.id + ": not compatible with this vehicle";
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

const char* const kEngineSlotKeys[] = {"ecu", "cam", "forced_induction", "installation", "thermal"};

// Derives the swap package for the installed engine / chosen fuel and applies it to the working vehicle file.
// "" when fine (also when nothing was swapped or chosen: `pkg` stays empty), else the broken rule.
std::string apply_swap(Files& files, const CatalogEntry& entry, const SetupContext& ctx, SwapPackage& pkg) {
    pkg = SwapPackage{};
    if (!files.swapped && files.fuel_choice.empty()) return {};
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

    pkg = derive_swap_package(car, eng, SwapChoices{files.fuel_choice}, files.swapped.has_value());
    if (!pkg.ok) return entry.id + ": " + pkg.error;
    // The car's current gearbox is not turned away by an engine swap, but the player is told when it is too weak.
    if (files.swapped && ctx.gearboxes && files.gearbox && eng.peak_torque_nm > 0.0) {
        const GearboxEntry* g = ctx.gearboxes->find_by_file(files.gearbox->path);
        if (g != nullptr && !check_gearbox_rating(*g, eng.peak_torque_nm).empty()) {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "Gearbox: %s is rated %.0f N*m but this engine makes %.0f N*m - install a stronger one", g->name.c_str(), g->rating_nm, eng.peak_torque_nm);
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
        if (d.kind != OptionKind::Colour && d.pointers.empty()) return bad(where + ": \"pointers\" (the whitelist) is empty");
        if (d.kind == OptionKind::Colour && !d.pointers.empty()) return bad(where + ": a colour edits no file, so it has no pointers");
        const double scale_max_limit = d.id == "turbo_boost_target" ? 20.0 : 10.0;
        if ((d.kind == OptionKind::Scale || d.kind == OptionKind::ScaleList) &&
            !(d.min > 0.0 && d.min <= 1.0 && d.max >= 1.0 && d.max < scale_max_limit && d.step > 0.0)) {
            return bad(where + ": a scale needs 0 < min <= 1 <= max < 10 and step > 0");
        }
        if (d.kind == OptionKind::BrakeBias && !(d.max_shift > 0.0 && d.max_shift < 0.5)) {
            return bad(where + ": max_shift must be in (0, 0.5)");
        }
        if (d.kind == OptionKind::FileChoice) {
            if (d.file != "vehicle" && d.file != "engine") return bad(where + ": file choice must edit vehicle or engine");
            if (!detail::get_string(o, "part_format", d.part_format) || d.part_format.empty()) return bad(where + ": part_format missing");
            detail::get_bool(o, "no_turbo_override", d.no_turbo_override);
            detail::get_string(o, "catalog", d.catalog);
            if (!d.catalog.empty() && d.catalog != "engines" && d.catalog != "fuels" && d.catalog != "gearboxes") {
                return bad(where + ": \"catalog\" must be \"engines\", \"fuels\" or \"gearboxes\"");
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
std::vector<SetupPart> parts_of(const SetupOptionDef& def, const SetupContext& ctx, const Files& files) {
    if (def.catalog.empty()) return def.parts;
    std::vector<SetupPart> out;
    if (def.catalog == "gearboxes") {
        if (!ctx.gearboxes) return out;
        const double peak = installed_engine_peak_torque(files, ctx);
        const std::string original = original_gearbox_path(files);
        for (const GearboxEntry& g : ctx.gearboxes->list()) {
            // The car's own box is always listed (it becomes "stock"); any other must take the engine's torque.
            if (norm_path(g.file) != original && !check_gearbox_rating(g, peak).empty()) continue;
            if (g.kind != GearboxKind::Manual && g.kind != GearboxKind::Sequential) continue;
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

} // namespace

SetupModel build_setup_model(const CatalogEntry& entry, const SetupOptionTable& table, const SetupContext& ctx, const VehicleSetup* installed) {
    SetupModel model;
    Files files;
    if (!load_files(entry, files, &model.error)) return model;
    if (installed) {
        for (const auto& def : table.options) {
            if (def.kind != OptionKind::FileChoice) continue;
            const auto it = installed->values.find(def.id);
            if (it == installed->values.end()) continue;
            model.error = install_file_part(files, def, entry, ctx, it->second);
            if (!model.error.empty()) return model;
        }
    }
    {
        SwapPackage pkg;
        model.error = apply_swap(files, entry, ctx, pkg);
        if (!model.error.empty()) return model;
        model.swap_lines = pkg.lines;
        model.chassis_mass_delta_kg = pkg.mass_delta_kg;
        if (files.swapped) model.engine_title = files.swapped->name;
    }
    if (!load_turbo(files, &model.error)) return model;
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
                v.available = file_choice_fits(files, def, entry, ctx);
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
                const std::vector<SetupPart> candidates = parts_of(def, ctx, files);
                for (const auto& part : candidates) if (norm_path(part.path) == original) { stock = part; stock.id = "stock"; }
                v.parts.push_back(stock);
                v.choices.push_back("stock");
                for (const auto& part : candidates) {
                    if (norm_path(part.path) == original) continue;
                    v.parts.push_back(part);
                    v.choices.push_back(part.id);
                }
                if (v.choices.size() < 2) v.available = false;
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
    // Retain boost calibration as dormant when the player swaps to NA.
    if (def.file == "turbo" && !files.turbo) return {};
    const std::vector<std::string> patterns = pointers_for(entry, def);
    const auto range = range_for(entry, def);
    const auto type_error = [&](const char* expected) { return "option \"" + id + "\" needs " + expected; };

    switch (def.kind) {
        case OptionKind::FileChoice: return install_file_part(files, def, entry, ctx, value);
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

    // Merge patches (the mechanism) and the whitelist check on the result.
    // Engine-affecting garage edits request interpolation-driven maps. Bounds
    // follow this engine's grid and the selected turbo target, in absolute kPa.
    bool adaptive_pressure = false;
    if (files.engine && files.engine->work.contains("cycle")) {
        for (const auto& def : table.options) {
            if (setup.values.count(def.id) && (def.file == "engine" || def.file == "turbo" || def.part_format == "physics_sim.turbo_configuration/1" || (def.part_format == "physics_sim.engine/1" && def.catalog.empty()))) adaptive_pressure = true;
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
    if (files.turbo && !write_aux("turbo", *files.turbo)) return out;
    if (files.gearbox && !write_aux("gearbox", *files.gearbox)) return out;
    if (files.engine && !write_aux("engine", *files.engine)) return out;
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
