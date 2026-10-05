// rg/vehicle_catalog.cpp - see vehicle_catalog.h.
#include "rg/vehicle_catalog.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <set>

namespace rg {

namespace {

using detail::json;
namespace fs = std::filesystem;

constexpr double kPi = 3.14159265358979323846;

bool fail(std::string* err, const std::string& origin, const std::string& message) {
    if (err != nullptr) *err = origin + ": " + message;
    return false;
}

bool valid_id(const std::string& s) {
    if (s.empty() || s.size() > 48) return false;
    for (const char c : s) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
        if (!ok) return false;
    }
    return true;
}

bool valid_hex_colour(const std::string& s) {
    if (s.size() != 7 || s[0] != '#') return false;
    for (std::size_t i = 1; i < s.size(); ++i) {
        const char c = s[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) return false;
    }
    return true;
}

std::string resolve(const std::string& root, const std::string& relative) {
    const fs::path p(relative);
    if (p.is_absolute()) return p.lexically_normal().generic_string();
    return (fs::path(root) / p).lexically_normal().generic_string();
}

std::optional<json> read_json_file(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        if (err != nullptr) *err = path + ": cannot open";
        return std::nullopt;
    }
    auto parsed = detail::parse_json(*text);
    if (!parsed || !parsed->is_object()) {
        if (err != nullptr) *err = path + ": not a JSON object";
        return std::nullopt;
    }
    return parsed;
}

bool read_triple(const json& o, const char* key, std::array<double, 3>& out) {
    const auto it = o.find(key);
    if (it == o.end() || !it->is_array() || it->size() != 3) return false;
    for (std::size_t i = 0; i < 3; ++i) {
        if (!(*it)[i].is_number()) return false;
        out[i] = (*it)[i].get<double>();
        if (!std::isfinite(out[i])) return false;
    }
    return true;
}

// The referenced file of the powertrain component of `type` ("engine",
// "gearbox"), resolved against the vehicle file's directory; "" when absent.
std::string component_ref(const json& vehicle, const std::string& vehicle_path, const char* type) {
    const auto pt = vehicle.find("powertrain");
    if (pt == vehicle.end() || !pt->is_object()) return {};
    const auto comps = pt->find("components");
    if (comps == pt->end() || !comps->is_array()) return {};
    for (const json& c : *comps) {
        if (!c.is_object()) continue;
        std::string t;
        if (!detail::get_string(c, "type", t) || t != type) continue;
        std::string ref;
        if (!detail::get_string(c, "ref", ref)) return {};
        return resolve(fs::path(vehicle_path).parent_path().generic_string(), ref);
    }
    return {};
}

} // namespace

const CatalogEntry* VehicleCatalog::find(const std::string& id) const {
    for (const CatalogEntry& e : entries) {
        if (e.id == id) return &e;
    }
    return nullptr;
}

std::optional<VehicleCatalog> parse_vehicle_catalog(const std::string& json_text, const std::string& repo_root,
                                                    const std::string& origin, std::string* err) {
    const std::optional<json> parsed = detail::parse_json(json_text);
    if (!parsed || !parsed->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    std::string format;
    if (!detail::get_string(*parsed, "format", format) || format != "rg.vehicle_catalog/1") {
        fail(err, origin, "\"format\" must be \"rg.vehicle_catalog/1\"");
        return std::nullopt;
    }
    VehicleCatalog out;
    const auto vehicles = parsed->find("vehicles");
    if (vehicles == parsed->end() || !vehicles->is_array() || vehicles->empty()) {
        fail(err, origin, "\"vehicles\" must be a non-empty array");
        return std::nullopt;
    }
    std::set<std::string> seen;
    for (const json& v : *vehicles) {
        if (!v.is_object()) {
            fail(err, origin, "a vehicles entry is not an object");
            return std::nullopt;
        }
        CatalogEntry e;
        if (!detail::get_string(v, "id", e.id) || !valid_id(e.id)) {
            fail(err, origin, "a vehicle has no valid \"id\" (lower_snake, <= 48 chars)");
            return std::nullopt;
        }
        const std::string where = origin + " (" + e.id + ")";
        if (!seen.insert(e.id).second) {
            fail(err, origin, "duplicate vehicle id \"" + e.id + "\"");
            return std::nullopt;
        }
        std::string vehicle_rel, model_rel;
        if (!detail::get_string(v, "title", e.title) || e.title.empty()) {
            fail(err, where, "\"title\" missing");
            return std::nullopt;
        }
        detail::get_string(v, "subtitle", e.subtitle);
        detail::get_string(v, "description", e.description);
        if (!detail::get_string(v, "vehicle", vehicle_rel) || !detail::get_string(v, "model", model_rel)) {
            fail(err, where, "\"vehicle\" and \"model\" paths are required");
            return std::nullopt;
        }
        e.vehicle_path = resolve(repo_root, vehicle_rel);
        e.model_path = resolve(repo_root, model_rel);
        std::string file_err;
        const std::optional<json> vehicle_doc = read_json_file(e.vehicle_path, &file_err);
        if (!vehicle_doc) {
            fail(err, where, file_err);
            return std::nullopt;
        }
        if (!detail::get_string(*vehicle_doc, "name", e.sim_name) || e.sim_name.empty()) {
            fail(err, where, e.vehicle_path + ": the vehicle file has no \"name\"");
            return std::nullopt;
        }
        std::error_code ec;
        if (!fs::is_regular_file(fs::path(e.model_path), ec)) {
            fail(err, where, "model not found: " + e.model_path);
            return std::nullopt;
        }
        const auto chassis = v.find("chassis");
        if (chassis == v.end() || !chassis->is_object()) {
            fail(err, where, "\"chassis\" block missing (mass_kg, half_extents, spawn_z_m)");
            return std::nullopt;
        }
        if (!detail::get_number(*chassis, "mass_kg", e.chassis.mass_kg) || !(e.chassis.mass_kg > 100.0) ||
            !(e.chassis.mass_kg < 100000.0)) {
            fail(err, where, "chassis.mass_kg must be a number in 100..100000");
            return std::nullopt;
        }
        if (!read_triple(*chassis, "half_extents", e.chassis.half_extents) || !(e.chassis.half_extents[0] > 0.1) ||
            !(e.chassis.half_extents[1] > 0.1) || !(e.chassis.half_extents[2] > 0.01)) {
            fail(err, where, "chassis.half_extents must be three positive numbers");
            return std::nullopt;
        }
        if (!detail::get_number(*chassis, "spawn_z_m", e.chassis.spawn_z_m) || !(e.chassis.spawn_z_m > 0.0) ||
            !(e.chassis.spawn_z_m < 5.0)) {
            fail(err, where, "chassis.spawn_z_m must be a number in 0..5");
            return std::nullopt;
        }
        detail::get_string(*chassis, "source", e.chassis.source);
        if (detail::get_string(v, "engine_bay", e.engine_bay) &&
            e.engine_bay != "front" && e.engine_bay != "mid" && e.engine_bay != "rear") {
            fail(err, where, "\"engine_bay\" must be front, mid or rear");
            return std::nullopt;
        }
        const auto declared = v.find("declared_engine");
        if (declared != v.end()) {
            DeclaredEngineStats d;
            if (!declared->is_object() || !detail::get_number(*declared, "peak_torque_nm", d.peak_torque_nm) ||
                !detail::get_number(*declared, "peak_torque_rpm", d.peak_torque_rpm) ||
                !detail::get_number(*declared, "peak_power_kw", d.peak_power_kw) ||
                !detail::get_number(*declared, "peak_power_rpm", d.peak_power_rpm) ||
                !detail::get_string(*declared, "source", d.source) || d.source.empty() || !(d.peak_torque_nm > 0.0) ||
                !(d.peak_power_kw > 0.0)) {
                fail(err, where, "\"declared_engine\" needs peak_torque_nm/_rpm, peak_power_kw/_rpm (> 0) and a source");
                return std::nullopt;
            }
            e.declared_engine = d;
        }
        const auto visual = v.find("visual");
        if (visual != v.end()) {
            if (!visual->is_object()) {
                fail(err, where, "\"visual\" must be an object");
                return std::nullopt;
            }
            const auto paint = visual->find("paint");
            if (paint != visual->end()) {
                if (!paint->is_string() || !valid_hex_colour(paint->get<std::string>())) {
                    fail(err, where, "visual.paint must be a #rrggbb colour");
                    return std::nullopt;
                }
                e.default_paint = paint->get<std::string>();
            }
            const auto rim = visual->find("rim");
            if (rim != visual->end()) {
                if (!rim->is_string() || !valid_hex_colour(rim->get<std::string>())) {
                    fail(err, where, "visual.rim must be a #rrggbb colour");
                    return std::nullopt;
                }
                e.default_rim = rim->get<std::string>();
            }
        }
        const auto setup = v.find("setup");
        if (setup != v.end()) {
            if (!setup->is_object()) {
                fail(err, where, "\"setup\" must be an object");
                return std::nullopt;
            }
            const auto options = setup->find("options");
            if (options != setup->end()) {
                if (!options->is_array()) {
                    fail(err, where, "setup.options must be an array of option ids");
                    return std::nullopt;
                }
                for (const json& o : *options) {
                    if (!o.is_string()) {
                        fail(err, where, "setup.options must be an array of option ids");
                        return std::nullopt;
                    }
                    e.setup_options.push_back(o.get<std::string>());
                }
            }
            const auto pointers = setup->find("pointers");
            if (pointers != setup->end()) {
                if (!pointers->is_object()) {
                    fail(err, where, "setup.pointers must be an object");
                    return std::nullopt;
                }
                for (auto it = pointers->begin(); it != pointers->end(); ++it) {
                    if (!it.value().is_array() || it.value().empty()) {
                        fail(err, where, "setup.pointers." + it.key() + " must be a non-empty array of strings");
                        return std::nullopt;
                    }
                    std::vector<std::string> list;
                    for (const json& p : it.value()) {
                        if (!p.is_string()) {
                            fail(err, where, "setup.pointers." + it.key() + " must be an array of strings");
                            return std::nullopt;
                        }
                        list.push_back(p.get<std::string>());
                    }
                    e.setup_pointers[it.key()] = std::move(list);
                }
            }
            const auto ranges = setup->find("ranges");
            if (ranges != setup->end()) {
                if (!ranges->is_object()) {
                    fail(err, where, "setup.ranges must be an object");
                    return std::nullopt;
                }
                for (auto it = ranges->begin(); it != ranges->end(); ++it) {
                    double lo = 0.0, hi = 0.0;
                    if (!it.value().is_object() || !detail::get_number(it.value(), "min", lo) ||
                        !detail::get_number(it.value(), "max", hi) || !(lo <= hi)) {
                        fail(err, where, "setup.ranges." + it.key() + " needs min <= max");
                        return std::nullopt;
                    }
                    e.setup_ranges[it.key()] = {lo, hi};
                }
            }
        }
        out.entries.push_back(std::move(e));
    }
    if (!detail::get_string(*parsed, "default", out.default_id)) out.default_id = out.entries.front().id;
    if (out.find(out.default_id) == nullptr) {
        fail(err, origin, "\"default\" names an unknown vehicle \"" + out.default_id + "\"");
        return std::nullopt;
    }
    return out;
}

std::optional<VehicleCatalog> load_vehicle_catalog(const std::string& path, const std::string& repo_root,
                                                   std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot open");
        return std::nullopt;
    }
    return parse_vehicle_catalog(*text, repo_root, path, err);
}

std::string default_engine_cache_dir(const std::string& vehicle_path) {
    std::error_code ec;
    const fs::path abs = fs::absolute(fs::path(vehicle_path), ec);
    return (abs.parent_path().parent_path().parent_path() / "out" / "godot_engine_cache").generic_string();
}

std::optional<VehicleStats> compute_vehicle_stats(const CatalogEntry& entry, std::string* err) {
    const std::optional<json> vehicle = read_json_file(entry.vehicle_path, err);
    if (!vehicle) return std::nullopt;
    VehicleStats stats;
    stats.mass_kg = entry.chassis.mass_kg;

    // Layout: the wheels that appear as "wheel:<name>" ports of any shaft are the
    // driven ones (the powertrain builder drives exactly those).
    std::set<std::string> driven;
    const auto pt = vehicle->find("powertrain");
    if (pt != vehicle->end() && pt->is_object()) {
        const auto shafts = pt->find("shafts");
        if (shafts != pt->end() && shafts->is_array()) {
            for (const json& shaft : *shafts) {
                const auto ports = shaft.is_object() ? shaft.find("ports") : shaft.end();
                if (!shaft.is_object() || ports == shaft.end() || !ports->is_array()) continue;
                for (const json& p : *ports) {
                    if (!p.is_string()) continue;
                    const std::string port = p.get<std::string>();
                    if (port.rfind("wheel:", 0) == 0) driven.insert(port.substr(6));
                }
            }
        }
    }
    int front_wheels = 0, rear_wheels = 0, front_driven = 0, rear_driven = 0;
    const auto wheels = vehicle->find("wheels");
    if (wheels != vehicle->end() && wheels->is_array()) {
        for (const json& w : *wheels) {
            if (!w.is_object()) continue;
            ++stats.wheel_count;
            std::string name;
            detail::get_string(w, "name", name);
            bool is_front = false;
            detail::get_bool(w, "is_front", is_front);
            (is_front ? front_wheels : rear_wheels) += 1;
            if (driven.count(name) != 0) {
                ++stats.driven_wheels;
                (is_front ? front_driven : rear_driven) += 1;
            }
        }
    }
    if (stats.driven_wheels == 0) {
        stats.layout = "other";
    } else if (front_driven == front_wheels && rear_driven == rear_wheels) {
        stats.layout = "AWD";
    } else if (front_driven > 0 && rear_driven == 0) {
        stats.layout = "FWD";
    } else if (rear_driven > 0 && front_driven == 0) {
        stats.layout = "RWD";
    } else {
        stats.layout = "other";
    }

    // Gearbox: forward gears.
    const std::string gearbox_path = component_ref(*vehicle, entry.vehicle_path, "gearbox");
    if (!gearbox_path.empty()) {
        const auto gearbox = read_json_file(gearbox_path, err);
        if (!gearbox) return std::nullopt;
        const auto ratios = gearbox->find("forward_ratios");
        if (ratios != gearbox->end() && ratios->is_array()) stats.gear_count = static_cast<int>(ratios->size());
    }

    // Engine: the WOT curve of a torque-map engine, else the declared figures.
    const std::string engine_path = component_ref(*vehicle, entry.vehicle_path, "engine");
    if (!engine_path.empty()) {
        const auto engine = read_json_file(engine_path, err);
        if (!engine) return std::nullopt;
        detail::get_string(*engine, "name", stats.engine_name);
        std::string kind = "torque_map";
        detail::get_string(*engine, "kind", kind);
        stats.engine_kind = kind;
        const auto curve = engine->find("wot_torque_nm_vs_rpm");
        if (curve != engine->end() && curve->is_object()) {
            const auto xs = curve->find("x");
            const auto ys = curve->find("y");
            if (xs != curve->end() && ys != curve->end() && xs->is_array() && ys->is_array() &&
                xs->size() == ys->size() && !xs->empty()) {
                for (std::size_t i = 0; i < xs->size(); ++i) {
                    if (!(*xs)[i].is_number() || !(*ys)[i].is_number()) continue;
                    const double rpm = (*xs)[i].get<double>();
                    const double nm = (*ys)[i].get<double>();
                    if (nm > stats.peak_torque_nm) {
                        stats.peak_torque_nm = nm;
                        stats.peak_torque_rpm = rpm;
                    }
                    const double kw = nm * rpm * kPi / 30.0 / 1000.0;
                    if (kw > stats.peak_power_kw) {
                        stats.peak_power_kw = kw;
                        stats.peak_power_rpm = rpm;
                    }
                }
            }
        }
    }
    if (stats.peak_torque_nm <= 0.0 && entry.declared_engine) {
        stats.peak_torque_nm = entry.declared_engine->peak_torque_nm;
        stats.peak_torque_rpm = entry.declared_engine->peak_torque_rpm;
        stats.peak_power_kw = entry.declared_engine->peak_power_kw;
        stats.peak_power_rpm = entry.declared_engine->peak_power_rpm;
        stats.engine_figures_declared = true;
    }
    return stats;
}

} // namespace rg
