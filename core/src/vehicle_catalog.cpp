// rg/vehicle_catalog.cpp - see vehicle_catalog.h.
#include "rg/vehicle_catalog.h"

#include "driveline_swap_json.h"
#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
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

std::optional<double> displacement_from_json(const json& engine) {
    double bore = 0.0, stroke = 0.0, cylinders = 0.0;
    if (!detail::get_number(engine, "bore_mm", bore) || !detail::get_number(engine, "stroke_mm", stroke) ||
        !detail::get_number(engine, "cylinders", cylinders)) {
        return std::nullopt;
    }
    if (!(bore > 0.0) || !(stroke > 0.0) || !(cylinders >= 1.0) || !std::isfinite(bore) || !std::isfinite(stroke)) {
        return std::nullopt;
    }
    return cylinders * kPi / 4.0 * bore * bore * stroke / 1.0e6; // mm^3 -> litres
}

} // namespace

std::optional<double> displacement_from_engine_json(const std::string& engine_json_text) {
    const std::optional<json> doc = detail::parse_json(engine_json_text);
    if (!doc || !doc->is_object()) return std::nullopt;
    return displacement_from_json(*doc);
}

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
        if (!detail::get_string(v, "title", e.title) || e.title.empty()) {
            fail(err, where, "\"title\" missing");
            return std::nullopt;
        }
        std::string preset_of;
        const bool is_preset = v.contains("preset_of");
        if (is_preset) {
            if (!detail::get_string(v, "preset_of", preset_of) || preset_of.empty()) {
                fail(err, where, "\"preset_of\" must be the id of a base vehicle");
                return std::nullopt;
            }
            const CatalogEntry* base = nullptr;
            for (const CatalogEntry& prior : out.entries) {
                if (prior.id == preset_of && prior.preset_of.empty()) base = &prior;
            }
            if (base == nullptr) {
                fail(err, where, "\"preset_of\" \"" + preset_of + "\" names no base vehicle listed before this entry");
                return std::nullopt;
            }
            for (const char* inherited : {"vehicle", "model", "chassis", "engine_bay", "engine_bay_envelope", "model_options", "declared_engine", "setup"}) {
                if (v.contains(inherited)) {
                    fail(err, where, std::string("a preset takes \"") + inherited + "\" from its base (\"" + preset_of + "\")");
                    return std::nullopt;
                }
            }
            const std::string id = e.id;
            const std::string title = e.title;
            e = *base;
            e.id = id;
            e.title = title;
            e.preset_of = preset_of;
            e.preset_setup.clear();
            const auto setup_values = v.find("preset_setup");
            if (setup_values == v.end() || !setup_values->is_object() || setup_values->empty()) {
                fail(err, where, "a preset needs a non-empty \"preset_setup\" object (option id -> value)");
                return std::nullopt;
            }
            for (auto it = setup_values->begin(); it != setup_values->end(); ++it) {
                SetupValue value;
                const json& jv = it.value();
                if (jv.is_boolean()) {
                    value = jv.get<bool>();
                } else if (jv.is_number()) {
                    value = jv.get<double>();
                } else if (jv.is_string()) {
                    value = jv.get<std::string>();
                } else if (jv.is_array() && !jv.empty()) {
                    std::vector<double> list;
                    for (const json& x : jv) {
                        if (!x.is_number()) {
                            fail(err, where, "preset_setup." + it.key() + ": a list holds numbers only");
                            return std::nullopt;
                        }
                        list.push_back(x.get<double>());
                    }
                    value = std::move(list);
                } else {
                    fail(err, where, "preset_setup." + it.key() + ": expected a number, a switch, a string or a list of numbers");
                    return std::nullopt;
                }
                e.preset_setup[it.key()] = std::move(value);
            }
        } else {
            std::string vehicle_rel, model_rel;
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
            if (const auto bay = v.find("engine_bay_envelope"); bay != v.end()) {
                EngineBayEnvelope b;
                std::string axis;
                const auto size = bay->is_object() ? bay->find("size_mm") : bay->end();
                bool ok = bay->is_object() && size != bay->end() && size->is_array() && size->size() == 3 &&
                          detail::get_string(*bay, "crank_axis", axis) && (axis == "x" || axis == "y") &&
                          detail::get_string(*bay, "source", b.source) && !b.source.empty();
                for (std::size_t i = 0; ok && i < 3; ++i) {
                    ok = (*size)[i].is_number() && (*size)[i].get<double>() > 0.0;
                    if (ok) b.size_mm[i] = (*size)[i].get<double>();
                }
                if (!ok) {
                    fail(err, where, "\"engine_bay_envelope\" needs size_mm [x, y, z] > 0, crank_axis \"x\" or \"y\" and a source");
                    return std::nullopt;
                }
                b.crank_axis = axis[0];
                e.engine_bay_envelope = b;
            }
            if (const auto mo = v.find("model_options"); mo != v.end()) {
                bool ok = mo->is_array();
                for (const json& o : ok ? *mo : json::array()) {
                    ok = ok && o.is_string() && !o.get<std::string>().empty();
                    if (ok) e.model_options.push_back(o.get<std::string>());
                }
                if (!ok) {
                    fail(err, where, "\"model_options\" must be a list of model option names");
                    return std::nullopt;
                }
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
        }
        detail::get_string(v, "subtitle", e.subtitle);
        detail::get_string(v, "description", e.description);
        if (v.contains("body_type")) {
            if (!detail::get_string(v, "body_type", e.body_type) || !valid_id(e.body_type)) {
                fail(err, where, "\"body_type\" must be a lower_snake word (sedan, hatchback, estate, pickup, suv, sports, hypercar, race, utility, ...)");
                return std::nullopt;
            }
        }
        if (v.contains("manufacturer")) {
            if (!detail::get_string(v, "manufacturer", e.manufacturer) || e.manufacturer.size() > 64) {
                fail(err, where, "\"manufacturer\" must be a short string");
                return std::nullopt;
            }
        }
        if (v.contains("displacement_l")) {
            double d = 0.0;
            std::string source;
            if (!detail::get_number(v, "displacement_l", d) || !(d > 0.0) || !(d < 30.0) ||
                !detail::get_string(v, "displacement_source", source) || source.empty()) {
                fail(err, where, "\"displacement_l\" needs a number in 0..30 litres and a \"displacement_source\"");
                return std::nullopt;
            }
            e.displacement_l = d;
            e.displacement_source = source;
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
            const auto fitment = setup->find("tyre_fitment");
            if (fitment != setup->end()) {
                if (!fitment->is_object()) {
                    fail(err, where, "setup.tyre_fitment must be an object");
                    return std::nullopt;
                }
                for (auto it = fitment->begin(); it != fitment->end(); ++it) {
                    TyreAxleFitment f;
                    double min_w = 0.0, max_w = 0.0, step = static_cast<double>(f.width_step_mm);
                    bool ok = (it.key() == "front" || it.key() == "rear") && it.value().is_object() &&
                              detail::get_number(it.value(), "min_width_mm", min_w) && detail::get_number(it.value(), "max_width_mm", max_w);
                    const bool tol_given = ok && detail::get_number(it.value(), "diameter_tolerance_mm", f.diameter_tolerance_mm);
                    const bool step_given = ok && detail::get_number(it.value(), "width_step_mm", step);
                    if (ok) {
                        const auto rims = it.value().find("rims_in");
                        ok = rims != it.value().end() && rims->is_array() && !rims->empty();
                        if (ok) {
                            for (const auto& r : *rims) {
                                if (!r.is_number() || r.get<double>() < 1.0 || r.get<double>() != std::floor(r.get<double>())) ok = false;
                                else f.rims_in.push_back(static_cast<int>(r.get<double>()));
                            }
                        }
                    }
                    ok = ok && min_w >= 1.0 && min_w <= max_w && min_w == std::floor(min_w) && max_w == std::floor(max_w) && step >= 1.0 &&
                         step == std::floor(step) && (!tol_given || f.diameter_tolerance_mm > 0.0);
                    (void)step_given;
                    if (!ok) {
                        fail(err, where, "setup.tyre_fitment." + it.key() + " needs front/rear with rims_in (non-empty list of whole inches), whole 1 <= min_width_mm <= max_width_mm, an optional whole width_step_mm >= 1 and an optional diameter_tolerance_mm > 0");
                        return std::nullopt;
                    }
                    f.min_width_mm = static_cast<int>(min_w);
                    f.max_width_mm = static_cast<int>(max_w);
                    f.width_step_mm = static_cast<int>(step);
                    e.tyre_fitment[it.key()] = f;
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

    // Layout: the wheels that appear as "wheel:<name>" ports of any shaft are the driven ones (rg/driveline_swap.h).
    const DrivelineInfo line = detail::describe_driveline_json(*vehicle);
    stats.layout = line.layout;
    stats.driven_wheels = line.driven_wheels;
    stats.wheel_count = line.wheel_count;
    stats.driveline = driveline_summary(line);

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
    stats.engine_path = engine_path;
    if (!engine_path.empty()) {
        const auto engine = read_json_file(engine_path, err);
        if (!engine) return std::nullopt;
        detail::get_string(*engine, "name", stats.engine_name);
        if (const auto liters = displacement_from_json(*engine)) {
            double bore = 0.0, stroke = 0.0, cylinders = 0.0;
            detail::get_number(*engine, "bore_mm", bore);
            detail::get_number(*engine, "stroke_mm", stroke);
            detail::get_number(*engine, "cylinders", cylinders);
            stats.displacement_known = true;
            stats.displacement_l = *liters;
            char text[96];
            std::snprintf(text, sizeof text, "engine file: %g cylinders x bore %g x stroke %g mm", cylinders, bore, stroke);
            stats.displacement_source = text;
        }
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
    if (!stats.displacement_known && entry.displacement_l) {
        stats.displacement_known = true;
        stats.displacement_l = *entry.displacement_l;
        stats.displacement_source = entry.displacement_source;
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
