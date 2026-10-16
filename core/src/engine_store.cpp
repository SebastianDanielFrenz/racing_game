// rg/engine_store.cpp - see engine_store.h.
#include "rg/engine_store.h"

#include "json_util.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/engine_thermal_io.h"
#include "ps/io/vehicle_io.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <set>
#include <system_error>
#include <variant>

namespace rg {

namespace {

namespace fs = std::filesystem;
using detail::json;

constexpr double kPi = 3.14159265358979323846;

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

bool write_text(const std::string& path, const std::string& text, std::string* err) {
    std::error_code ec;
    fs::create_directories(fs::path(path).parent_path(), ec);
    const std::string tmp = path + ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) {
            if (err != nullptr) *err = tmp + ": cannot write";
            return false;
        }
        out << text;
        out.flush();
        if (!out) {
            if (err != nullptr) *err = tmp + ": write failed";
            return false;
        }
    }
    fs::rename(fs::path(tmp), fs::path(path), ec);
    if (ec) {
        fs::remove(fs::path(tmp), ec);
        if (err != nullptr) *err = path + ": cannot replace (" + ec.message() + ")";
        return false;
    }
    return true;
}

std::string now_text() {
    const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
#ifdef _WIN32
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return buf;
}

EngineFacts facts_of(const ps::drivetrain::ComponentParams& params, bool* ok) {
    EngineFacts f;
    *ok = true;
    if (const auto* tm = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&params)) {
        f.simulated = false;
        f.mechanical_throttle = tm->throttle_input == ps::drivetrain::ThrottleInput::Mechanical;
        f.spark = tm->ignition == ps::drivetrain::EngineIgnition::Spark;
        f.idle_rpm = tm->idle_rpm;
        f.limiter_rpm = tm->limiter.rpm;
        f.mass_kg = tm->mass_kg;
        for (const auto& fuel : tm->fuels) f.fuels.push_back({fuel.name, fuel.lhv_j_kg / 1.0e6, fuel.density_kg_m3});
        const auto& c = tm->wot_torque_nm_vs_rpm;
        EnginePeaks p;
        for (std::size_t i = 0; i < c.x.size() && i < c.y.size(); ++i) {
            const double power = c.y[i] * c.x[i] * kPi / 30000.0;
            if (!p.known || c.y[i] > p.torque_nm) {
                p.torque_nm = c.y[i];
                p.torque_rpm = c.x[i];
            }
            if (!p.known || power > p.power_kw) {
                p.power_kw = power;
                p.power_rpm = c.x[i];
            }
            p.known = true;
        }
        p.source = "wot_curve";
        f.curve_peaks = p;
    } else if (const auto* sim = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&params)) {
        f.simulated = true;
        f.mechanical_throttle = sim->throttle_input == ps::drivetrain::ThrottleInput::Mechanical;
        f.spark = sim->ignition == ps::drivetrain::EngineIgnition::Spark;
        f.idle_rpm = sim->idle_rpm;
        f.limiter_rpm = sim->limiter.rpm;
        f.mass_kg = sim->mass_kg;
        for (const auto& fuel : sim->fuels) f.fuels.push_back({fuel.name, fuel.lhv_j_kg / 1.0e6, fuel.density_kg_m3});
        f.has_turbo = sim->turbo_pair.has_value();
        f.rotary = sim->architecture == ps::drivetrain::EngineArchitecture::Rotary;
        f.lift_family = sim->has_lift_family() || (sim->cycle && !sim->cycle->lift_family.empty());
        if (sim->cycle) {
            const ps::drivetrain::CycleModelDesc& c = *sim->cycle;
            f.has_cycle = true;
            f.cylinders = c.cylinders;
            f.compression_ratio = c.compression_ratio;
            f.stroke_m = c.stroke_m;
            f.displacement_l = kPi / 4.0 * c.bore_m * c.bore_m * c.stroke_m * c.cylinders * 1000.0;
            f.cycle = std::make_shared<const ps::drivetrain::CycleModelDesc>(c);
        }
    } else {
        *ok = false;
    }
    return f;
}

struct CacheKey {
    std::string path;
    std::uintmax_t size = 0;
    std::int64_t mtime = 0;
    bool operator==(const CacheKey& o) const { return path == o.path && size == o.size && mtime == o.mtime; }
};

struct CacheValue {
    CacheKey key;
    EngineFacts facts;
};

std::mutex g_facts_mutex;
std::map<std::string, CacheValue> g_facts_cache;

} // namespace

const char* to_string(EngineClass c) {
    switch (c) {
        case EngineClass::Curve: return "curve";
        case EngineClass::Mapped: return "mapped";
        case EngineClass::Cycle: return "cycle";
    }
    return "curve";
}

std::optional<EngineFacts> read_engine_facts(const std::string& engine_file, std::string* err) {
    const std::string path = norm(engine_file);
    CacheKey key;
    key.path = path;
    {
        std::error_code ec;
        key.size = fs::file_size(fs::path(path), ec);
        const auto t = fs::last_write_time(fs::path(path), ec);
        key.mtime = static_cast<std::int64_t>(t.time_since_epoch().count());
    }
    {
        std::lock_guard<std::mutex> lock(g_facts_mutex);
        const auto it = g_facts_cache.find(path);
        if (it != g_facts_cache.end() && it->second.key == key) return it->second.facts;
    }
    try {
        ps::io::EngineMapOptions options;
        options.generate = false;
        const ps::drivetrain::ComponentParams params = ps::io::load_engine_json(path, options);
        bool ok = true;
        EngineFacts f = facts_of(params, &ok);
        if (ok) {
            // The engine thermal model needs the engine file to name a default_thermal (extends chain resolved).
            try {
                f.has_thermal = !ps::io::engine_default_thermal_path(path).empty();
            } catch (...) {
                f.has_thermal = false;
            }
        }
        if (!ok) {
            if (err != nullptr) *err = path + ": not an engine";
            return std::nullopt;
        }
        std::lock_guard<std::mutex> lock(g_facts_mutex);
        g_facts_cache[path] = CacheValue{key, f};
        return f;
    } catch (const std::exception& e) {
        if (err != nullptr) *err = e.what();
    } catch (...) {
        if (err != nullptr) *err = path + ": the engine loader failed with an unknown error";
    }
    return std::nullopt;
}

// ---- store ---------------------------------------------------------------------------------------

std::string EngineStore::user_root() const {
    if (user_dir_.empty()) return {};
    return user_dir_ + "/garage/engines";
}

std::string EngineStore::fuel_file(const std::string& fuel_name) const { return fuel_dir_ + "/" + fuel_name + ".json"; }

std::unique_ptr<EngineStore> EngineStore::open(const std::string& catalog_path, const std::string& repo_root,
                                               const std::string& user_dir, std::string* err) {
    const auto fail = [&](const std::string& m) -> std::unique_ptr<EngineStore> {
        if (err != nullptr) *err = catalog_path + ": " + m;
        return nullptr;
    };
    const auto text = detail::read_text_file(catalog_path);
    if (!text) return fail("cannot open");
    const auto doc = detail::parse_json(*text);
    if (!doc || !doc->is_object()) return fail("not a JSON object");
    std::string format;
    if (!detail::get_string(*doc, "format", format) || format != "rg.engine_catalog/1") return fail("format must be rg.engine_catalog/1");
    std::unique_ptr<EngineStore> s(new EngineStore());
    s->repo_root_ = norm(repo_root);
    s->user_dir_ = user_dir;
    std::string fuel_dir;
    if (!detail::get_string(*doc, "fuel_dir", fuel_dir) || fuel_dir.empty()) return fail("\"fuel_dir\" is required");
    s->fuel_dir_ = resolve_from(s->repo_root_, fuel_dir);
    const auto engines = doc->find("engines");
    if (engines == doc->end() || !engines->is_array() || engines->empty()) return fail("\"engines\" must be a non-empty array");
    std::set<std::string> ids, aliases;
    for (const json& row : *engines) {
        if (!row.is_object()) return fail("every engine must be an object");
        EngineEntry e;
        std::string cls, file;
        if (!detail::get_string(row, "id", e.id) || !is_lower_snake(e.id)) return fail("every engine needs a lower_snake \"id\"");
        const std::string where = "engine \"" + e.id + "\": ";
        if (!ids.insert(e.id).second) return fail(where + "duplicate id");
        if (!detail::get_string(row, "name", e.name) || e.name.empty()) return fail(where + "\"name\" is required");
        if (!detail::get_string(row, "file", file) || file.empty()) return fail(where + "\"file\" is required");
        if (!detail::get_string(row, "class", cls)) return fail(where + "\"class\" is required");
        if (cls == "curve") e.cls = EngineClass::Curve;
        else if (cls == "mapped") e.cls = EngineClass::Mapped;
        else if (cls == "cycle") e.cls = EngineClass::Cycle;
        else return fail(where + "\"class\" must be curve, mapped or cycle");
        detail::get_string(row, "family", e.family);
        detail::get_string(row, "image", e.image);
        detail::get_string(row, "detail", e.detail);
        detail::get_string(row, "mass_source", e.mass_source);
        if (!detail::get_number(row, "mass_kg", e.mass_kg) || !(e.mass_kg > 0.0)) return fail(where + "\"mass_kg\" must be a number > 0");
        if (e.mass_source.empty()) return fail(where + "\"mass_source\" is required");
        e.file = resolve_from(s->repo_root_, file);
        std::error_code ec;
        if (!fs::is_regular_file(fs::path(e.file), ec)) return fail(where + e.file + " does not exist");
        e.base_stem = fs::path(e.file).stem().string();
        if (const auto it = row.find("crank_axis_local"); it != row.end()) {
            if (!it->is_array() || it->size() != 3) return fail(where + "\"crank_axis_local\" must be [x, y, z]");
            for (std::size_t i = 0; i < 3; ++i) {
                if (!(*it)[i].is_number()) return fail(where + "\"crank_axis_local\" must be numbers");
                e.crank_axis_local[i] = (*it)[i].get<double>();
            }
        }
        if (const auto it = row.find("envelope"); it != row.end()) {
            if (!it->is_string() || it->get<std::string>().empty()) return fail(where + "\"envelope\" must be a path");
            e.envelope_path = resolve_from(s->repo_root_, it->get<std::string>());
            const auto text = detail::read_text_file(e.envelope_path);
            const auto env = text ? detail::parse_json(*text) : std::nullopt;
            std::string format;
            const auto size = env && env->is_object() ? env->find("size_mm") : json::const_iterator{};
            if (!env || !env->is_object() || !detail::get_string(*env, "format", format) || format != "rg.engine_envelope/1" ||
                size == env->end() || !size->is_array() || size->size() != 3) {
                return fail(where + e.envelope_path + ": not an rg.engine_envelope/1 file with size_mm [x, y, z]");
            }
            std::array<double, 3> mm{};
            for (std::size_t i = 0; i < 3; ++i) {
                if (!(*size)[i].is_number() || !((*size)[i].get<double>() > 0.0)) return fail(where + e.envelope_path + ": size_mm must be > 0");
                mm[i] = (*size)[i].get<double>();
            }
            e.envelope_mm = mm;
            detail::get_string(*env, "layout", e.envelope_layout);
            const fs::path glb = fs::path(e.envelope_path).parent_path() / (e.id + ".glb");
            std::error_code gec;
            if (fs::is_regular_file(glb, gec)) e.model_path = glb.generic_string();
        }
        if (const auto it = row.find("peaks"); it != row.end()) {
            if (!it->is_object()) return fail(where + "\"peaks\" must be an object");
            EnginePeaks p;
            if (!detail::get_number(*it, "torque_nm", p.torque_nm) || !detail::get_number(*it, "torque_rpm", p.torque_rpm) ||
                !detail::get_number(*it, "power_kw", p.power_kw) || !detail::get_number(*it, "power_rpm", p.power_rpm)) {
                return fail(where + "\"peaks\" needs torque_nm, torque_rpm, power_kw, power_rpm");
            }
            p.known = true;
            p.source = "catalog";
            e.catalog_peaks = p;
        } else if (e.cls != EngineClass::Curve) {
            return fail(where + "a " + cls + " engine has no WOT curve: \"peaks\" (measured with the engine dyno) is required");
        }
        if (const auto it = row.find("aliases"); it != row.end()) {
            if (!it->is_array()) return fail(where + "\"aliases\" must be an array of ids");
            for (const json& a : *it) {
                if (!a.is_string() || !is_lower_snake(a.get<std::string>())) return fail(where + "an alias must be a lower_snake id");
                const std::string alias = a.get<std::string>();
                if (!aliases.insert(alias).second || ids.count(alias) != 0) return fail(where + "alias \"" + alias + "\" is not unique");
                e.aliases.push_back(alias);
            }
        }
        e.key = "default:" + e.id;
        s->defaults_.push_back(std::move(e));
    }
    for (const std::string& a : aliases) {
        if (ids.count(a) != 0) return fail("alias \"" + a + "\" equals an engine id");
    }
    s->reload_user_engines();
    return s;
}

void EngineStore::rebuild() {
    entries_.clear();
    for (const EngineEntry& d : defaults_) {
        entries_.push_back(d);
        for (const EngineEntry& u : users_) {
            if (u.base_key == d.key) entries_.push_back(u);
        }
    }
}

void EngineStore::reload_user_engines() {
    users_.clear();
    const std::string root = user_root();
    std::error_code ec;
    if (!root.empty() && fs::is_directory(fs::path(root), ec)) {
        std::vector<std::string> uids;
        for (const auto& de : fs::directory_iterator(fs::path(root), ec)) {
            if (de.is_directory()) uids.push_back(de.path().filename().string());
        }
        std::sort(uids.begin(), uids.end());
        for (const std::string& uid : uids) {
            const std::string dir = root + "/" + uid;
            const auto text = detail::read_text_file(dir + "/meta.json");
            if (!text) continue;
            const auto meta = detail::parse_json(*text);
            std::string format, base, name;
            if (!meta || !meta->is_object() || !detail::get_string(*meta, "format", format) || format != "rg.user_engine/1" ||
                !detail::get_string(*meta, "base", base) || !detail::get_string(*meta, "name", name)) {
                continue;
            }
            const EngineEntry* b = nullptr;
            for (const EngineEntry& d : defaults_) {
                if (d.key == base) b = &d;
            }
            if (b == nullptr || !fs::is_regular_file(fs::path(dir + "/engine.json"), ec)) continue; // base removed from the game: ignored
            EngineEntry e = *b;
            e.key = "user:" + uid;
            e.id = uid;
            e.user = true;
            e.name = name;
            e.file = norm(dir + "/engine.json");
            e.base_key = b->key;
            e.base_name = b->name;
            e.aliases.clear();
            detail::get_string(*meta, "created", e.created);
            detail::get_string(*meta, "modified", e.modified);
            users_.push_back(std::move(e));
        }
    }
    rebuild();
}

const EngineEntry* EngineStore::find(const std::string& key) const {
    for (const EngineEntry& e : entries_) {
        if (e.key == key) return &e;
    }
    for (const EngineEntry& e : entries_) {
        if (e.user) continue;
        if (e.id == key) return &e;
        if (std::find(e.aliases.begin(), e.aliases.end(), key) != e.aliases.end()) return &e;
    }
    return nullptr;
}

const EngineEntry* EngineStore::find_by_file(const std::string& path) const {
    const std::string n = norm(path);
    for (const EngineEntry& e : entries_) {
        if (e.file == n) return &e;
    }
    return nullptr;
}

EnginePeaks EngineStore::peaks(const EngineEntry& e, std::string* err) const {
    if (e.cls == EngineClass::Curve) {
        if (const auto facts = read_engine_facts(e.file, err)) return facts->curve_peaks;
        return {};
    }
    return e.catalog_peaks;
}

// ---- user engines -----------------------------------------------------------------------------------

namespace {

// The text of a user engine file: {"extends": <abs base>, ...patch}. nullopt + *err on a bad patch.
std::optional<std::string> user_engine_text(const std::string& base_file, const std::string& patch_json, std::string* err) {
    auto patch = detail::parse_json(patch_json);
    if (!patch || !patch->is_object()) {
        if (err != nullptr) *err = "the engine change is not a JSON object";
        return std::nullopt;
    }
    json doc = json::object();
    doc["extends"] = base_file;
    for (auto it = patch->begin(); it != patch->end(); ++it) {
        if (it.key() == "extends") {
            if (err != nullptr) *err = "a user engine cannot change what it extends";
            return std::nullopt;
        }
        doc[it.key()] = it.value();
    }
    return doc.dump(2) + "\n";
}

bool validate_engine_text(const std::string& text, const std::string& base_dir, std::string* err) {
    try {
        ps::io::EngineMapOptions options;
        options.generate = false;
        (void)ps::io::load_engine_json_from_string(text, fs::path(base_dir), options);
    } catch (const std::exception& e) {
        if (err != nullptr) *err = e.what();
        return false;
    } catch (...) {
        if (err != nullptr) *err = "the engine loader failed with an unknown error";
        return false;
    }
    return true;
}

} // namespace

std::optional<std::string> EngineStore::create_user_engine(const std::string& base_key, const std::string& name,
                                                           const std::string& patch_json, std::string* err) {
    const std::string root = user_root();
    if (root.empty()) {
        if (err != nullptr) *err = "there is no user directory: user engines cannot be saved";
        return std::nullopt;
    }
    const EngineEntry* base = find(base_key);
    if (base == nullptr) {
        if (err != nullptr) *err = "unknown engine \"" + base_key + "\"";
        return std::nullopt;
    }
    // Editing a user engine's copy extends the DEFAULT it came from, never the user file itself.
    const EngineEntry* def = base->user ? find(base->base_key) : base;
    if (def == nullptr) {
        if (err != nullptr) *err = "the base engine of \"" + base_key + "\" is gone";
        return std::nullopt;
    }
    const auto text = user_engine_text(def->file, patch_json, err);
    if (!text) return std::nullopt;
    // A unique uid: e0001, e0002, ... after the highest in use (a removed engine's uid is not reused in a session).
    int highest = 0;
    std::error_code ec;
    if (fs::is_directory(fs::path(root), ec)) {
        for (const auto& de : fs::directory_iterator(fs::path(root), ec)) {
            const std::string n = de.path().filename().string();
            if (n.size() == 5 && n[0] == 'e') highest = std::max(highest, std::atoi(n.c_str() + 1));
        }
    }
    char uid_buf[16];
    std::snprintf(uid_buf, sizeof(uid_buf), "e%04d", highest + 1);
    const std::string uid = uid_buf;
    const std::string dir = root + "/" + uid;
    // The directory exists before the validation: the loader rebases the base engine's relative paths against it, and
    // a POSIX open() resolves "<dir>/../.." through the real directory (Windows normalises it lexically).
    fs::create_directories(fs::path(dir), ec);
    if (!validate_engine_text(*text, dir, err)) {
        fs::remove_all(fs::path(dir), ec);
        return std::nullopt;
    }
    int copies = 1;
    for (const EngineEntry& u : users_) {
        if (u.base_key == def->key) ++copies;
    }
    const std::string display = name.empty() ? def->name + " (custom " + std::to_string(copies) + ")" : name;
    json meta = json::object();
    meta["format"] = "rg.user_engine/1";
    meta["name"] = display;
    meta["base"] = def->key;
    meta["created"] = now_text();
    meta["modified"] = meta["created"];
    if (!write_text(dir + "/engine.json", *text, err) || !write_text(dir + "/meta.json", meta.dump(2) + "\n", err)) {
        fs::remove_all(fs::path(dir), ec);
        return std::nullopt;
    }
    reload_user_engines();
    return "user:" + uid;
}

bool EngineStore::update_user_engine(const std::string& key, const std::string& patch_json, std::string* err) {
    const EngineEntry* e = find(key);
    if (e == nullptr || !e->user) {
        if (err != nullptr) *err = "\"" + key + "\" is not a user engine";
        return false;
    }
    const EngineEntry* def = find(e->base_key);
    if (def == nullptr) {
        if (err != nullptr) *err = "the base engine of \"" + key + "\" is gone";
        return false;
    }
    const auto text = user_engine_text(def->file, patch_json, err);
    if (!text) return false;
    const std::string dir = user_root() + "/" + e->id;
    if (!validate_engine_text(*text, dir, err)) return false;
    if (!write_text(dir + "/engine.json", *text, err)) return false;
    if (const auto m = detail::read_text_file(dir + "/meta.json")) {
        if (auto meta = detail::parse_json(*m); meta && meta->is_object()) {
            (*meta)["modified"] = now_text();
            write_text(dir + "/meta.json", meta->dump(2) + "\n", nullptr);
        }
    }
    reload_user_engines();
    return true;
}

bool EngineStore::rename_user_engine(const std::string& key, const std::string& name, std::string* err) {
    const EngineEntry* e = find(key);
    if (e == nullptr || !e->user || name.empty()) {
        if (err != nullptr) *err = name.empty() ? "an engine needs a name" : "\"" + key + "\" is not a user engine";
        return false;
    }
    const std::string path = user_root() + "/" + e->id + "/meta.json";
    const auto m = detail::read_text_file(path);
    auto meta = m ? detail::parse_json(*m) : std::nullopt;
    if (!meta || !meta->is_object()) {
        if (err != nullptr) *err = path + ": unreadable";
        return false;
    }
    (*meta)["name"] = name;
    (*meta)["modified"] = now_text();
    if (!write_text(path, meta->dump(2) + "\n", err)) return false;
    reload_user_engines();
    return true;
}

bool EngineStore::remove_user_engine(const std::string& key, std::string* err) {
    const EngineEntry* e = find(key);
    if (e == nullptr || !e->user) {
        if (err != nullptr) *err = "\"" + key + "\" is not a user engine";
        return false;
    }
    std::error_code ec;
    fs::remove_all(fs::path(user_root() + "/" + e->id), ec);
    if (ec) {
        if (err != nullptr) *err = "cannot remove: " + ec.message();
        return false;
    }
    reload_user_engines();
    return true;
}

std::string EngineStore::user_patch(const std::string& key) const {
    const EngineEntry* e = find(key);
    if (e == nullptr || !e->user) return {};
    const auto text = detail::read_text_file(e->file);
    auto doc = text ? detail::parse_json(*text) : std::nullopt;
    if (!doc || !doc->is_object()) return {};
    doc->erase("extends");
    return doc->dump();
}

} // namespace rg
