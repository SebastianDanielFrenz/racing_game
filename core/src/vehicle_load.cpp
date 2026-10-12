// rg/vehicle_load.cpp - see vehicle_load.h.
#include "rg/vehicle_load.h"

#include "json_util.h"

#include <filesystem>
#include <stdexcept>

namespace rg {

namespace {

namespace fs = std::filesystem;
using detail::json;

std::string resolve(const std::string& base_dir, const std::string& ref) {
    const fs::path p(ref);
    return (p.is_absolute() ? p : fs::path(base_dir) / p).lexically_normal().generic_string();
}

void inject(ps::vehicle::VehicleDesc& desc, const std::string& path) {
    const VehicleTyreSpecs specs = read_vehicle_tyre_specs(path);
    if (!specs.error.empty()) throw std::runtime_error(path + ": " + specs.error);
    if (specs.families_path.empty()) return;
    std::string err;
    const auto families = TyreFamilies::load(specs.families_path, &err);
    if (!families) throw std::runtime_error(err);
    if (specs.wheels.size() != desc.wheels.size()) {
        throw std::runtime_error(path + ": the wheel list changed between the tyre reader and the physics loader");
    }
    for (std::size_t i = 0; i < specs.wheels.size(); ++i) {
        const TyreSpec& s = specs.wheels[i];
        if (s.compound.empty()) continue;
        try {
            desc.wheels[i].tyre = families->make(s);
        } catch (const std::invalid_argument& e) {
            throw std::runtime_error(path + ": wheel " + std::to_string(i) + ": " + e.what());
        }
    }
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
    ps::vehicle::VehicleDesc desc = ps::io::load_vehicle_json(path);
    inject(desc, path);
    return desc;
}

ps::vehicle::VehicleDesc load_vehicle(const std::string& path, const ps::io::EngineMapOptions& options) {
    ps::vehicle::VehicleDesc desc = ps::io::load_vehicle_json(path, options);
    inject(desc, path);
    return desc;
}

ps::vehicle::VehicleDesc load_vehicle(const std::string& path, const ps::io::VehicleLoadOptions& options) {
    ps::vehicle::VehicleDesc desc = ps::io::load_vehicle_json(path, options);
    inject(desc, path);
    return desc;
}

} // namespace rg
