// rg/garage.cpp - see garage.h.
#include "rg/garage.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <system_error>

namespace rg {

namespace {

namespace fs = std::filesystem;
using detail::json;

std::uint64_t fnv1a(const std::string& s) {
    std::uint64_t h = 1469598103934665603ULL;
    for (const unsigned char c : s) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    return h;
}

std::string hex16(std::uint64_t v) {
    static const char* digits = "0123456789abcdef";
    std::string out(16, '0');
    for (int i = 15; i >= 0; --i) {
        out[static_cast<std::size_t>(i)] = digits[v & 0xF];
        v >>= 4;
    }
    return out;
}

bool write_file_atomic(const std::string& path, const std::string& text, std::string* err) {
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

bool option_value_type_ok(OptionKind kind, const SetupValue& v) {
    switch (kind) {
        case OptionKind::Scale:
        case OptionKind::BrakeBias: return std::holds_alternative<double>(v);
        case OptionKind::ScaleList: return std::holds_alternative<std::vector<double>>(v);
        case OptionKind::Bool: return std::holds_alternative<bool>(v);
        case OptionKind::TyreChoice:
        case OptionKind::Colour: return std::holds_alternative<std::string>(v);
    }
    return false;
}

} // namespace

std::string Garage::garage_dir() const { return config_.user_dir + "/garage"; }

std::string Garage::setup_path(const std::string& id) const { return garage_dir() + "/setups/" + id + ".json"; }

std::unique_ptr<Garage> Garage::open(const GarageConfig& config, std::string* err) {
    std::unique_ptr<Garage> g(new Garage());
    g->config_ = config;
    std::string e;
    auto catalog = load_vehicle_catalog(config.catalog_path, config.repo_root, &e);
    if (!catalog) {
        if (err != nullptr) *err = e;
        return nullptr;
    }
    g->catalog_ = std::move(*catalog);
    auto options = load_setup_options(config.options_path, &e);
    if (!options) {
        if (err != nullptr) *err = e;
        return nullptr;
    }
    g->options_ = std::move(*options);
    auto set = load_garage_set(config.set_path, &e);
    if (!set) {
        if (err != nullptr) *err = e;
        return nullptr;
    }
    g->set_ = std::move(*set);
    g->ctx_.tyre_dirs = config.tyre_dirs;
    g->ctx_.work_root = config.work_root;
    g->ctx_.engine_map_cache_dir = config.engine_map_cache_dir;

    // A crashed run can leave its drive materialisation behind: remove it.
    std::error_code ec;
    if (fs::is_directory(fs::path(config.work_root), ec)) {
        for (const auto& de : fs::directory_iterator(fs::path(config.work_root), ec)) {
            const std::string name = de.path().filename().string();
            if (de.is_directory() && (name.rfind("drive_", 0) == 0 || name.rfind("validate_", 0) == 0)) {
                fs::remove_all(de.path(), ec);
            }
        }
    }

    g->selected_ = g->catalog_.default_id;
    if (const auto text = detail::read_text_file(g->garage_dir() + "/selected.json")) {
        if (const auto doc = detail::parse_json(*text); doc && doc->is_object()) {
            std::string id;
            if (detail::get_string(*doc, "selected", id) && g->catalog_.find(id) != nullptr) g->selected_ = id;
        }
    }
    return g;
}

Garage::~Garage() { cleanup(); }

bool Garage::write_selection(std::string* err) const {
    json doc = json::object();
    doc["format"] = "racing_game.garage/1";
    doc["selected"] = selected_;
    return write_file_atomic(garage_dir() + "/selected.json", doc.dump(2) + "\n", err);
}

bool Garage::select(const std::string& id, std::string* err) {
    if (catalog_.find(id) == nullptr) {
        if (err != nullptr) *err = "unknown vehicle \"" + id + "\"";
        return false;
    }
    if (selected_ == id) return true;
    selected_ = id;
    return write_selection(err);
}

std::optional<VehicleStats> Garage::stats(const std::string& id, std::string* err) const {
    const CatalogEntry* e = catalog_.find(id);
    if (e == nullptr) {
        if (err != nullptr) *err = "unknown vehicle \"" + id + "\"";
        return std::nullopt;
    }
    return compute_vehicle_stats(*e, err);
}

std::optional<GarageSubject> Garage::subject(const std::string& id, const GVec3* model_min, const GVec3* model_max,
                                             std::string* err) const {
    const CatalogEntry* e = catalog_.find(id);
    if (e == nullptr) {
        if (err != nullptr) *err = "unknown vehicle \"" + id + "\"";
        return std::nullopt;
    }
    const auto text = detail::read_text_file(e->vehicle_path);
    const std::optional<json> doc = text ? detail::parse_json(*text) : std::nullopt;
    if (!doc || !doc->is_object()) {
        if (err != nullptr) *err = e->vehicle_path + ": cannot read";
        return std::nullopt;
    }
    GarageSubject s;
    s.engine_bay = e->engine_bay;
    double front = -1e9, rear = 1e9, half_track = 0.0, radius = 0.0;
    const auto wheels = doc->find("wheels");
    if (wheels != doc->end() && wheels->is_array()) {
        for (const json& w : *wheels) {
            const auto att = w.find("attachment_local");
            if (!w.is_object() || att == w.end() || !att->is_array() || att->size() != 3 || !(*att)[0].is_number() ||
                !(*att)[1].is_number()) {
                continue;
            }
            front = std::max(front, (*att)[0].get<double>());
            rear = std::min(rear, (*att)[0].get<double>());
            half_track = std::max(half_track, std::abs((*att)[1].get<double>()));
            double r = 0.0;
            if (detail::get_number(w, "wheel_radius", r)) radius = std::max(radius, r);
        }
    }
    if (front > rear) {
        s.front_axle_x = front;
        s.rear_axle_x = rear;
    }
    if (half_track > 0.0) s.half_track = half_track;
    if (radius > 0.0) s.wheel_radius = radius;
    if (model_min != nullptr && model_max != nullptr) {
        s.bounds_min = *model_min;
        s.bounds_max = *model_max;
    } else {
        s.bounds_min = {s.rear_axle_x - 0.95, -(s.half_track + 0.15), 0.0};
        s.bounds_max = {s.front_axle_x + 0.95, s.half_track + 0.15, std::max(1.2, 2.0 * s.wheel_radius + 0.45)};
    }
    return s;
}

VehicleSetup Garage::saved_setup(const std::string& id) const {
    VehicleSetup s;
    s.vehicle_id = id;
    const auto text = detail::read_text_file(setup_path(id));
    if (!text) return s;
    std::string err;
    auto parsed = parse_setup(*text, options_, setup_path(id), &err);
    if (!parsed || parsed->vehicle_id != id) return s; // a corrupt/foreign file is ignored, never fatal
    return *parsed;
}

bool Garage::has_saved_setup(const std::string& id) const { return !saved_setup(id).values.empty(); }

bool Garage::begin_edit(const std::string& id, std::string* err) {
    const CatalogEntry* e = catalog_.find(id);
    if (e == nullptr) {
        if (err != nullptr) *err = "unknown vehicle \"" + id + "\"";
        return false;
    }
    model_ = build_setup_model(*e, options_, ctx_);
    if (!model_.error.empty()) {
        if (err != nullptr) *err = model_.error;
        return false;
    }
    edit_id_ = id;
    working_ = saved_setup(id);
    saved_at_begin_ = working_;
    revalidate();
    return true;
}

SetupValue Garage::current_value(const std::string& option_id) const {
    const auto it = working_.values.find(option_id);
    if (it != working_.values.end()) return it->second;
    for (const OptionView& v : model_.options) {
        if (v.def.id == option_id) return v.stock;
    }
    return 0.0;
}

void Garage::revalidate() {
    validation_ = ValidationResult{};
    const CatalogEntry* e = catalog_.find(edit_id_);
    if (e == nullptr) {
        validation_.message = "no vehicle is being edited";
        return;
    }
    if (working_.values.empty()) {
        validation_.ok = true;
        return;
    }
    validation_ = validate_setup(*e, options_, ctx_, working_);
}

EditResult Garage::set_option(const std::string& option_id, const SetupValue& value) {
    EditResult r;
    if (!editing()) {
        r.message = "no vehicle is being edited";
        r.validation = validation_;
        return r;
    }
    const OptionView* view = nullptr;
    for (const OptionView& v : model_.options) {
        if (v.def.id == option_id) view = &v;
    }
    if (view == nullptr) {
        r.message = "unknown option \"" + option_id + "\"";
        r.validation = validation_;
        return r;
    }
    if (!view->available) {
        r.message = "option \"" + option_id + "\" is not available for this car";
        r.validation = validation_;
        return r;
    }
    if (!option_value_type_ok(view->def.kind, value)) {
        r.message = "option \"" + option_id + "\" got a value of the wrong type";
        r.validation = validation_;
        return r;
    }
    if (setup_values_equal(value, view->stock)) {
        working_.values.erase(option_id); // back to stock: not part of the overlay
    } else {
        working_.values[option_id] = value;
    }
    r.accepted = true;
    revalidate();
    r.validation = validation_;
    return r;
}

void Garage::reset_option(const std::string& option_id) {
    if (!editing()) return;
    working_.values.erase(option_id);
    revalidate();
}

void Garage::reset_all() {
    if (!editing()) return;
    working_.values.clear();
    revalidate();
}

bool Garage::dirty() const {
    if (!editing()) return false;
    return setup_to_json(working_) != setup_to_json(saved_at_begin_);
}

std::string Garage::working_paint() const {
    const CatalogEntry* e = catalog_.find(edit_id_);
    const auto it = working_.values.find("paint");
    if (it != working_.values.end() && std::holds_alternative<std::string>(it->second) &&
        is_hex_colour(std::get<std::string>(it->second))) {
        return std::get<std::string>(it->second);
    }
    return e != nullptr ? e->default_paint : std::string("#8a8f98");
}

std::string Garage::working_rim() const {
    const CatalogEntry* e = catalog_.find(edit_id_);
    const auto it = working_.values.find("rim");
    if (it != working_.values.end() && std::holds_alternative<std::string>(it->second) &&
        is_hex_colour(std::get<std::string>(it->second))) {
        return std::get<std::string>(it->second);
    }
    return e != nullptr ? e->default_rim : std::string("#c4c8cf");
}

bool Garage::save(std::string* err) {
    if (!editing()) {
        if (err != nullptr) *err = "no vehicle is being edited";
        return false;
    }
    if (!validation_.ok) {
        if (err != nullptr) *err = validation_.message.empty() ? "the setup is not valid" : validation_.message;
        return false;
    }
    const std::string path = setup_path(edit_id_);
    std::error_code ec;
    if (working_.values.empty()) {
        fs::remove(fs::path(path), ec);
    } else {
        working_.vehicle_id = edit_id_;
        if (!write_file_atomic(path, setup_to_json(working_), err)) return false;
    }
    saved_at_begin_ = working_;
    return true;
}

void Garage::discard() {
    edit_id_.clear();
    working_ = VehicleSetup{};
    saved_at_begin_ = VehicleSetup{};
    model_ = SetupModel{};
    validation_ = ValidationResult{};
}

void Garage::remove_drive_dir() {
    if (drive_dir_.empty()) return;
    std::error_code ec;
    fs::remove_all(fs::path(drive_dir_), ec);
    drive_dir_.clear();
}

void Garage::cleanup() {
    remove_drive_dir();
    discard();
}

DriveSelection Garage::prepare_drive(const std::string& id) {
    DriveSelection d;
    const CatalogEntry* e = catalog_.find(id);
    if (e == nullptr) {
        d.error = "unknown vehicle \"" + id + "\"";
        return d;
    }
    d.vehicle_id = e->id;
    d.sim_name = e->sim_name;
    d.model_path = e->model_path;
    d.chassis = e->chassis;
    d.paint = e->default_paint;
    d.rim = e->default_rim;
    d.vehicle_path = e->vehicle_path;

    remove_drive_dir();
    const VehicleSetup setup = saved_setup(id);
    if (!setup.values.empty()) {
        const std::string dir = config_.work_root + "/drive_" + id + "_" + hex16(fnv1a(setup_to_json(setup)));
        const MaterialisedSetup m = materialise_setup(*e, options_, ctx_, setup, dir);
        if (!m.ok) {
            // The saved setup no longer fits the data (a file changed): drive the stock car and say so.
            d.warning = "The saved setup for " + e->title + " was not applied: " + m.error;
            std::error_code ec;
            fs::remove_all(fs::path(dir), ec);
        } else {
            if (m.compiled.vehicle_patch != "{}" || !m.compiled.patches.empty()) {
                d.vehicle_path = m.vehicle_path;
                drive_dir_ = dir;
                d.modified = true;
            } else if (!m.files.empty()) {
                d.modified = true;
            }
            if (!m.compiled.paint.empty()) {
                d.paint = m.compiled.paint;
                d.modified = true;
            }
            if (!m.compiled.rim.empty()) {
                d.rim = m.compiled.rim;
                d.modified = true;
            }
        }
    }

    // Assist defaults of the vehicle file that will be driven (the setup may have changed them).
    d.manual_gearbox = false;
    if (const auto text = detail::read_text_file(d.vehicle_path)) {
        if (const auto doc = detail::parse_json(*text); doc && doc->is_object()) {
            const auto controllers = doc->find("controllers");
            if (controllers != doc->end() && controllers->is_array()) {
                for (const json& c : *controllers) {
                    std::string type;
                    if (!c.is_object() || !detail::get_string(c, "type", type) || type != "manual_tcu") continue;
                    d.manual_gearbox = true;
                    auto flag = [&](const char* block, bool fallback) {
                        const auto b = c.find(block);
                        bool v = fallback;
                        if (b != c.end() && b->is_object()) detail::get_bool(*b, "enabled", v);
                        return v;
                    };
                    d.assist_auto_clutch = flag("auto_clutch", true);
                    d.assist_auto_blip = flag("auto_blip", true);
                    d.assist_auto_shift = flag("auto_shift", false);
                    break;
                }
            }
        }
    }
    if (!d.manual_gearbox) d.assist_auto_shift = true; // an automatic car shifts itself
    d.ok = true;
    return d;
}

} // namespace rg
