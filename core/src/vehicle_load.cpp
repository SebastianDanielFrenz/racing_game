// rg/vehicle_load.cpp - see vehicle_load.h.
#include "rg/vehicle_load.h"

#include "json_util.h"

#include <filesystem>
#include <map>
#include <stdexcept>

namespace rg {

namespace {

namespace fs = std::filesystem;
using detail::json;

std::string resolve(const std::string& base_dir, const std::string& ref) {
    const fs::path p(ref);
    return (p.is_absolute() ? p : fs::path(base_dir) / p).lexically_normal().generic_string();
}

// The tyres a vehicle file's wheels declare, built in memory and keyed by WHEEL NAME (physics_sim's
// VehicleLoadOptions::tyres). Empty = the file declares none (every wheel then needs its own "tyre", or physics_sim
// rejects the file with its "wheel '<name>': no tyre" message).
std::map<std::string, ps::vehicle::TyreDesc> build_tyres(const std::string& path) {
    std::map<std::string, ps::vehicle::TyreDesc> out;
    const VehicleTyreSpecs specs = read_vehicle_tyre_specs(path);
    if (!specs.error.empty()) throw std::runtime_error(path + ": " + specs.error);
    if (specs.families_path.empty()) return out;
    std::string err;
    const auto families = TyreFamilies::load(specs.families_path, &err);
    if (!families) throw std::runtime_error(err);
    if (specs.wheels.size() != specs.names.size()) throw std::runtime_error(path + ": internal tyre reader mismatch (wheel names)");
    std::map<std::string, TyreSpec> seen;
    for (std::size_t i = 0; i < specs.wheels.size(); ++i) {
        const TyreSpec& s = specs.wheels[i];
        if (s.compound.empty()) continue;
        const std::string& name = specs.names[i];
        // Two wheels with one name share one injected tyre (physics_sim keys by name): they must agree.
        const auto dup = seen.find(name);
        if (dup != seen.end()) {
            if (!(dup->second == s)) {
                throw std::runtime_error(path + ": wheel " + std::to_string(i) + ": the name \"" + name + "\" is used by two wheels with different tyre_spec (" +
                                         tyre_id(dup->second) + " and " + tyre_id(s) + "); give every wheel a unique \"name\"");
            }
            continue;
        }
        seen.emplace(name, s);
        try {
            out.emplace(name, families->make(s));
        } catch (const std::invalid_argument& e) {
            throw std::runtime_error(path + ": wheel '" + name + "': " + e.what());
        }
    }
    return out;
}

// The caller's own injected tyres (if any) win over the file's specs.
ps::io::VehicleLoadOptions with_tyres(const std::string& path, ps::io::VehicleLoadOptions options) {
    for (auto& kv : build_tyres(path)) options.tyres.emplace(kv.first, std::move(kv.second));
    return options;
}
} // namespace

VehicleTyreSpecs read_vehicle_tyre_specs(const std::string& vehicle_path) {
    VehicleTyreSpecs out;
    const auto text = detail::read_text_file(vehicle_path);
    const auto parsed = text ? detail::parse_json(*text) : std::nullopt;
    if (!parsed || !parsed->is_object()) return out; // the physics loader reports unreadable files
    const auto wheels = parsed->find("wheels");
    if (wheels == parsed->end() || !wheels->is_array()) return out;

    bool any = false;
    for (const json& w : *wheels) {
        TyreSpec s;
        if (w.is_object() && w.contains("tyre_spec")) {
            const json& t = w.at("tyre_spec");
            double width = 0.0, aspect = 0.0, rim = 0.0;
            if (w.contains("tyre")) {
                out.error = "a wheel has both \"tyre\" and \"tyre_spec\"";
                return out;
            }
            if (!t.is_object() || !detail::get_string(t, "compound", s.compound) || s.compound.empty() ||
                !detail::get_number(t, "width_mm", width) || !detail::get_number(t, "aspect", aspect) || !detail::get_number(t, "rim_in", rim) ||
                width <= 0.0 || aspect <= 0.0 || rim <= 0.0) {
                out.error = "\"tyre_spec\" needs compound (text) and positive width_mm, aspect and rim_in";
                return out;
            }
            s.width_mm = static_cast<int>(width);
            s.aspect = static_cast<int>(aspect);
            s.rim_in = static_cast<int>(rim);
            any = true;
        }
        out.wheels.push_back(std::move(s));
        out.names.push_back(w.is_object() ? w.value("name", std::string("wheel")) : std::string("wheel"));
    }
    if (!any) return out;

    std::string families;
    if (!detail::get_string(*parsed, "tyre_families", families) || families.empty()) {
        out.error = "wheels carry \"tyre_spec\" but the file has no \"tyre_families\" path";
        return out;
    }
    out.families_path = resolve(fs::path(vehicle_path).parent_path().generic_string(), families);
    return out;
}

ps::vehicle::VehicleDesc load_vehicle(const std::string& path) {
    return ps::io::load_vehicle_json(path, with_tyres(path, ps::io::VehicleLoadOptions{}));
}

ps::vehicle::VehicleDesc load_vehicle(const std::string& path, const ps::io::EngineMapOptions& options) {
    ps::io::VehicleLoadOptions load_options;
    load_options.maps = options;
    return ps::io::load_vehicle_json(path, with_tyres(path, std::move(load_options)));
}

ps::vehicle::VehicleDesc load_vehicle(const std::string& path, const ps::io::VehicleLoadOptions& options) {
    return ps::io::load_vehicle_json(path, with_tyres(path, options));
}
} // namespace rg
