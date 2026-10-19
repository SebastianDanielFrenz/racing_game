// rg/garage.cpp - see garage.h.
#include "rg/garage.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
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

std::string fmt_kg(double kg) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f kg", kg);
    return buf;
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
        case OptionKind::FileChoice:
        case OptionKind::TyreChoice:
        case OptionKind::Colour:
        case OptionKind::PartChoice: return std::holds_alternative<std::string>(v);
    }
    return false;
}

// Two overlays hold the same options with equal values.
bool setups_equal(const std::map<std::string, SetupValue>& a, const std::map<std::string, SetupValue>& b) {
    if (a.size() != b.size()) return false;
    for (const auto& [id, value] : a) {
        const auto it = b.find(id);
        if (it == b.end() || !setup_values_equal(value, it->second)) return false;
    }
    return true;
}

} // namespace

SetupContext Garage::context_for(const CatalogEntry& entry) const {
    SetupContext c = ctx_;
    if (c.engine_map_cache_dir.empty()) c.engine_map_cache_dir = default_engine_cache_dir(entry.vehicle_path);
    return c;
}

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
    // A preset's overlay must name known options with values of the right type (the value ranges and
    // the vehicle loader's verdict are checked when the preset is applied, see validate_setup).
    for (const CatalogEntry& e : g->catalog_.entries) {
        for (const auto& [option_id, value] : e.preset_setup) {
            const SetupOptionDef* def = g->options_.find(option_id);
            if (def == nullptr) {
                if (err != nullptr) *err = config.catalog_path + " (" + e.id + "): preset option \"" + option_id + "\" does not exist";
                return nullptr;
            }
            if (!option_value_type_ok(def->kind, value)) {
                if (err != nullptr) *err = config.catalog_path + " (" + e.id + "): preset option \"" + option_id + "\" has the wrong type of value";
                return nullptr;
            }
        }
    }
    g->ctx_.tyre_families_path = config.tyre_families_path;
    g->ctx_.work_root = config.work_root;
    g->ctx_.engine_map_cache_dir = config.engine_map_cache_dir;
    {
        const std::string cat = config.engine_catalog_path.empty() ? config.repo_root + "/data/engines/engine_catalog.json"
                                                                    : config.engine_catalog_path;
        auto store = EngineStore::open(cat, config.repo_root, config.user_dir, &e);
        if (!store) {
            if (err != nullptr) *err = e;
            return nullptr;
        }
        g->store_ = std::move(store);
        g->ctx_.engines = std::make_shared<const EngineStore>(*g->store_);
    }
    {
        // Turbo stage map extension policy: optional file, strict when present.
        std::string path = config.turbo_map_extension_path;
        if (path.empty()) {
            const std::string dflt = config.repo_root + "/data/engines/turbo_map_extension.json";
            std::error_code ec;
            if (fs::is_regular_file(fs::path(dflt), ec)) path = dflt;
        }
        if (!path.empty()) {
            auto cfg = load_turbo_map_extension_config(path, &e);
            if (!cfg) {
                if (err != nullptr) *err = e;
                return nullptr;
            }
            g->ctx_.turbo_map_extension = std::make_shared<const TurboMapExtensionConfig>(*cfg);
            g->turbo_map_extension_path_ = path;
        }
    }
    {
        // Garage turbo kits (rg/turbo_kit.h): optional file, strict when present.
        std::string path = config.turbo_kits_path;
        if (path.empty()) {
            const std::string dflt = config.repo_root + "/data/engines/turbo_kits.json";
            std::error_code ec;
            if (fs::is_regular_file(fs::path(dflt), ec)) path = dflt;
        }
        if (!path.empty()) {
            auto cfg = load_turbo_kit_config(path, config.repo_root, &e);
            if (!cfg) {
                if (err != nullptr) *err = e;
                return nullptr;
            }
            g->ctx_.turbo_kits = std::make_shared<const TurboKitConfig>(*cfg);
        }
    }
    {
        // Swappable engine internals (rg/engine_internals.h): optional file, strict when present.
        std::string path = config.engine_internals_path;
        if (path.empty()) {
            const std::string dflt = config.repo_root + "/data/engines/engine_internals.json";
            std::error_code ec;
            if (fs::is_regular_file(fs::path(dflt), ec)) path = dflt;
        }
        if (!path.empty()) {
            auto cfg = load_engine_internals(path, &e);
            if (!cfg) {
                if (err != nullptr) *err = e;
                return nullptr;
            }
            g->ctx_.engine_internals = std::make_shared<const EngineInternalsConfig>(*cfg);
        }
    }
    {
        // Cooling system upgrades (rg/cooling_upgrades.h): optional file, strict when present.
        std::string path = config.cooling_upgrades_path;
        if (path.empty()) {
            const std::string dflt = config.repo_root + "/data/cooling_systems/cooling_upgrades.json";
            std::error_code ec;
            if (fs::is_regular_file(fs::path(dflt), ec)) path = dflt;
        }
        if (!path.empty()) {
            auto cfg = load_cooling_upgrades(path, &e);
            if (!cfg) {
                if (err != nullptr) *err = e;
                return nullptr;
            }
            g->ctx_.cooling_upgrades = std::make_shared<const CoolingUpgradeConfig>(*cfg);
        }
    }
    {
        // Damage model (rg/damage.h): optional file, strict when present.
        std::string path = config.damage_config_path;
        if (path.empty()) {
            const std::string dflt = config.repo_root + "/data/vehicles/damage.json";
            std::error_code ec;
            if (fs::is_regular_file(fs::path(dflt), ec)) path = dflt;
        }
        if (!path.empty()) {
            auto cfg = load_damage_config(path, &e);
            if (!cfg) {
                if (err != nullptr) *err = e;
                return nullptr;
            }
            g->damage_ = std::move(*cfg);
            g->damage_config_path_ = path;
        }
    }
    {
        // Base part masses (rg/part_mass.h): optional file, strict when present.
        std::string path = config.part_masses_path;
        if (path.empty()) {
            const std::string dflt = config.repo_root + "/data/vehicles/part_masses.json";
            std::error_code ec;
            if (fs::is_regular_file(fs::path(dflt), ec)) path = dflt;
        }
        if (!path.empty()) {
            auto table = load_part_mass_table(path, &e);
            if (!table) {
                if (err != nullptr) *err = e;
                return nullptr;
            }
            g->ctx_.part_masses = std::make_shared<const PartMassTable>(std::move(*table));
        }
    }
    {
        // Weight-reduction choices -> model nodes to hide (rg/strip_visual.h): optional file, strict when present.
        std::string path = config.strip_visual_path;
        if (path.empty()) {
            const std::string dflt = config.repo_root + "/data/vehicles/strip_visual.json";
            std::error_code ec;
            if (fs::is_regular_file(fs::path(dflt), ec)) path = dflt;
        }
        if (!path.empty()) {
            auto table = load_strip_visual(path, &e);
            if (!table) {
                if (err != nullptr) *err = e;
                return nullptr;
            }
            g->strip_visual_ = std::make_shared<const StripVisualTable>(std::move(*table));
        }
    }
    {
        const std::string cat = config.gearbox_catalog_path.empty() ? config.repo_root + "/data/gearboxes/gearbox_catalog.json"
                                                                     : config.gearbox_catalog_path;
        auto boxes = GearboxCatalog::open(cat, config.repo_root, &e);
        if (!boxes) {
            if (err != nullptr) *err = e;
            return nullptr;
        }
        g->ctx_.gearboxes = std::shared_ptr<const GearboxCatalog>(std::move(boxes));
    }
    {
        const std::string cat = config.driveline_catalog_path.empty() ? config.repo_root + "/data/drivetrains/driveline_catalog.json"
                                                                       : config.driveline_catalog_path;
        auto lines = DrivelineCatalog::open(cat, &e);
        if (!lines) {
            if (err != nullptr) *err = e;
            return nullptr;
        }
        g->ctx_.drivelines = std::shared_ptr<const DrivelineCatalog>(std::move(lines));
    }

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
    g->refresh_browser();
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

const EngineEntry* Garage::bay_engine(const std::string& id) const {
    const VehicleSetup selected_setup = editing() && edit_id_ == id ? working_ : effective_setup(id);
    if (const auto it = selected_setup.values.find("powertrain_install"); it != selected_setup.values.end()) {
        if (const auto* kind = std::get_if<std::string>(&it->second); kind && (*kind == "battery_ev" || *kind == "fuel_cell_ev")) return nullptr;
    }
    if (editing() && edit_id_ == id && !model_.engine_key.empty()) return store_->find(model_.engine_key);
    // Not editing: the engine the saved setup (or the preset) installs, else the car's own.
    if (const CatalogEntry* e = catalog_.find(id)) {
        const VehicleSetup setup = effective_setup(id);
        if (!setup.values.empty()) {
            const SetupModel m = build_setup_model(*e, options_, context_for(*e), &setup);
            if (m.error.empty() && !m.engine_key.empty()) return store_->find(m.engine_key);
        }
    }
    std::string err;
    const auto st = stats(id, &err);
    if (!st || st->engine_path.empty()) return nullptr;
    return store_->find_by_file(st->engine_path);
}

std::optional<VehicleStats> Garage::stats(const std::string& id, std::string* err) const {
    const CatalogEntry* e = catalog_.find(id);
    if (e == nullptr) {
        if (err != nullptr) *err = "unknown vehicle \"" + id + "\"";
        return std::nullopt;
    }
    // Presets share their base's vehicle file: one computation per file (and declared displacement).
    const std::string key = e->vehicle_path + "#" + (e->displacement_l ? std::to_string(*e->displacement_l) : std::string());
    std::optional<VehicleStats> computed;
    if (const auto it = stats_cache_.find(key); it != stats_cache_.end()) {
        computed = it->second;
    } else {
        computed = compute_vehicle_stats(*e, err);
        if (!computed) return computed; // a failure is not cached: its message is wanted every time
        stats_cache_.emplace(key, *computed);
    }
    // A drivetrain swap: the layout the setup builds (only a setup that chooses a layout or a differential).
    const VehicleSetup setup = editing() && edit_id_ == id ? working_ : effective_setup(id);
    if (setup.values.count("powertrain_install") != 0 || setup.values.count("drive_layout") != 0 || setup.values.count("diff_front") != 0 || setup.values.count("diff_rear") != 0) {
        const SetupModel m = build_setup_model(*e, options_, context_for(*e), &setup);
        if (m.error.empty() && m.driveline.driven_wheels > 0) {
            computed->layout = m.driveline.layout;
            computed->driven_wheels = m.driveline.driven_wheels;
            computed->driveline = driveline_summary(m.driveline);
        }
    }
    // Owner on PR #22: a car's mass (a preset's too) is the sum of its parts with its setup applied.
    const std::string mass_key = id + "#" + setup_to_json(setup);
    if (const auto it = mass_cache_.find(mass_key); it != mass_cache_.end()) {
        computed->mass_kg = it->second;
    } else {
        PartTree t;
        if (setup.values.empty()) {
            t = stock_part_tree(*e, options_, context_for(*e));
        } else {
            CompiledSetup c = compile_setup(*e, options_, context_for(*e), setup);
            if (c.ok) t = std::move(c.parts);
        }
        if (t.ok) {
            computed->mass_kg = t.total_kg;
            mass_cache_.emplace(mass_key, t.total_kg);
        }
    }
    return computed;
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
    migrate_setup_tyres(*parsed, options_, ctx_); // pre-catalogue tyre names (saved before the universal tyres)
    return *parsed;
}

bool Garage::has_saved_setup(const std::string& id) const { return !saved_setup(id).values.empty(); }

VehicleSetup Garage::preset_setup(const std::string& id) const {
    VehicleSetup s;
    s.vehicle_id = id;
    if (const CatalogEntry* e = catalog_.find(id)) s.values = e->preset_setup;
    return s;
}

void Garage::normalise_engine_values(VehicleSetup& setup, std::vector<std::string>* notices) const {
    const auto it = setup.values.find("engine_install");
    if (it == setup.values.end() || !std::holds_alternative<std::string>(it->second)) return;
    const std::string v = std::get<std::string>(it->second);
    if (v == "stock") return;
    const EngineEntry* e = store_ ? store_->find(v) : nullptr; // a legacy id (sedan_i6) resolves here
    if (e == nullptr) {
        if (notices != nullptr) notices->push_back("The engine \"" + v + "\" is no longer in the garage list: the car keeps its own engine.");
        setup.values.erase("engine_install");
        setup.values.erase("fuel"); // a fuel picked for that engine
        return;
    }
    it->second = e->key;
}

VehicleSetup Garage::effective_setup_with_notices(const std::string& id, std::vector<std::string>* notices) const {
    VehicleSetup s = preset_setup(id);
    for (const auto& [option_id, value] : saved_setup(id).values) s.values[option_id] = value; // the player's own values win
    if (const CatalogEntry* e = catalog_.find(id)) { // a weight-reduction choice for a part the car lacks is dropped
        for (const SetupOptionDef& def : options_.options) {
            if (def.kind == OptionKind::PartChoice && !part_choice_offered(*e, def)) s.values.erase(def.id);
        }
    }
    normalise_engine_values(s, notices);
    return s;
}

VehicleSetup Garage::effective_setup(const std::string& id) const { return effective_setup_with_notices(id, nullptr); }

void Garage::engines_changed() {
    ctx_.engines = std::make_shared<const EngineStore>(*store_);
    mass_cache_.clear(); // a user engine's mass may have changed
    if (editing()) {
        std::vector<std::string> notices;
        normalise_engine_values(working_, &notices);
        revalidate();
    }
    refresh_browser();
}

std::optional<std::string> Garage::create_user_engine(const std::string& base_key, const std::string& name,
                                                      const std::string& patch_json, std::string* err) {
    auto key = store_->create_user_engine(base_key, name, patch_json, err);
    if (key) engines_changed();
    return key;
}

bool Garage::update_user_engine(const std::string& key, const std::string& patch_json, std::string* err) {
    const bool ok = store_->update_user_engine(key, patch_json, err);
    if (ok) engines_changed();
    return ok;
}

bool Garage::rename_user_engine(const std::string& key, const std::string& name, std::string* err) {
    const bool ok = store_->rename_user_engine(key, name, err);
    if (ok) engines_changed();
    return ok;
}

bool Garage::remove_user_engine(const std::string& key, std::string* err) {
    const bool ok = store_->remove_user_engine(key, err);
    if (ok) engines_changed();
    return ok;
}

bool Garage::begin_edit(const std::string& id, std::string* err) {
    const CatalogEntry* e = catalog_.find(id);
    if (e == nullptr) {
        if (err != nullptr) *err = "unknown vehicle \"" + id + "\"";
        return false;
    }
    model_ = build_setup_model(*e, options_, context_for(*e));
    if (!model_.error.empty()) {
        if (err != nullptr) *err = model_.error;
        return false;
    }
    edit_id_ = id;
    working_ = effective_setup(id);
    saved_at_begin_ = working_;
    revalidate();
    return true;
}

std::vector<SetupPart> Garage::powertrain_engine_choices() const {
    const auto* entry=catalog_.find(edit_id_);if(!entry)return {};
    VehicleSetup probe;probe.vehicle_id=edit_id_;probe.values["powertrain_install"]=std::string("ice");
    const auto model=build_setup_model(*entry,options_,context_for(*entry),&probe);
    if(!model.error.empty())return {};
    for(const auto& option:model.options)if(option.def.id=="engine_install")return option.parts;
    return {};
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
    SetupModel installed_model = build_setup_model(*e, options_, context_for(*e), &working_);
    if (installed_model.error.empty()) model_ = std::move(installed_model);
    parts_ = model_.parts;
    if (probe_only_) { // candidate_setup: the model follows the edits, nothing is loaded
        validation_.ok = true;
        return;
    }
    if (working_.values.empty()) {
        validation_.ok = true;
        return;
    }
    if (ctx_.part_masses) {
        // The whole working setup (a part choice the model does not install, a feature that adds parts).
        CompiledSetup c = compile_setup(*e, options_, context_for(*e), working_);
        if (c.ok && c.parts.ok) parts_ = std::move(c.parts);
    }
    validation_ = validate_setup(*e, options_, context_for(*e), working_, false);
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
    const CatalogEntry* entry = catalog_.find(edit_id_);
    const bool preset_has = entry != nullptr && entry->preset_setup.count(option_id) != 0;
    SetupValue stored = value;
    if (option_id == "engine_install") { // a legacy id (sedan_i6) is stored as the engine's key
        VehicleSetup one;
        one.values[option_id] = value;
        normalise_engine_values(one, nullptr);
        if (const auto it = one.values.find(option_id); it != one.values.end()) stored = it->second;
    }
    if (setup_values_equal(stored, view->stock) && !preset_has) {
        working_.values.erase(option_id); // back to stock: not part of the overlay
    } else {
        working_.values[option_id] = stored;
    }
    if(option_id=="powertrain_install") {
        for(const auto& def:options_.options) {
            if(def.id==option_id)continue;
            if(def.file=="engine"||def.file=="turbo"||def.file=="gearbox"||def.catalog=="engines"||def.catalog=="fuels"||def.catalog=="gearboxes"||def.id=="clutch_capacity")working_.values.erase(def.id);
        }
        r.notice="Powertrain component upgrades were reset for the conversion.";
    }
    r.accepted = true;
    if (option_id == "gearbox_install" && working_.values.erase("gear_ratios") != 0) {
        // The gear factors belong to the old box (another gear count, other ratios): back to the new box's own ratios.
        r.notice = "The gear ratios were reset to the new gearbox's own.";
    }
    revalidate();
    if (option_id == "engine_install" || option_id=="powertrain_install") {const auto dropped=drop_unavailable_values();if(!dropped.empty())r.notice+=dropped;}
    if (option_id == "powertrain_install") { // a drive layout / differential chosen before the conversion stays when the converted car still takes it
        const std::string kept = drop_driveline_values_after_conversion();
        if (!kept.empty()) r.notice += (r.notice.empty() ? "" : " ") + kept;
    }
    if (option_id == "gearbox_install") {
        const std::string dropped = drop_unavailable_values(true);
        if (!dropped.empty()) r.notice = r.notice.empty() ? dropped : r.notice + " " + dropped;
    }
    if (option_id == "drive_layout") r.notice = drop_driveline_values();
    r.validation = validation_;
    return r;
}

// After an engine change: values of options the installed engine does not offer (a turbo setting on an engine
// without a turbo, a fuel it does not list) are no longer meaningful; they return to stock and the player is told.
// What the new engine offers is read from a probe model with only the engine installed: the working
// model cannot be built while a now-incompatible value is still in the overlay.
std::string Garage::drop_unavailable_values(bool gearbox_change) {
    const CatalogEntry* entry = catalog_.find(edit_id_);
    if (entry == nullptr) return {};
    VehicleSetup probe;
    probe.vehicle_id = edit_id_;
    if(const auto it=working_.values.find("powertrain_install");it!=working_.values.end())probe.values["powertrain_install"]=it->second;
    if (const auto it = working_.values.find("engine_install"); it != working_.values.end()) probe.values["engine_install"] = it->second;
    if (gearbox_change) {
        if (const auto it = working_.values.find("gearbox_install"); it != working_.values.end()) probe.values["gearbox_install"] = it->second;
    }
    const SetupModel pm = build_setup_model(*entry, options_, context_for(*entry), &probe);
    if (!pm.error.empty()) return {}; // the engine itself does not install: revalidate() reports it
    std::string notice;
    std::vector<std::string> drop;
    for (const auto& [id, value] : working_.values) {
        if (id == "engine_install" || id=="powertrain_install" || (gearbox_change && id == "gearbox_install")) continue;
        if (id == "drive_layout" || id == "diff_front" || id == "diff_rear") continue; // judged on their own (drop_driveline_values*)
        const OptionView* view = nullptr;
        for (const OptionView& v : pm.options) if (v.def.id == id) view = &v;
        if (view == nullptr) continue;
        bool gone = !view->available;
        if (!gone && view->def.kind == OptionKind::FileChoice && std::holds_alternative<std::string>(value)) {
            const std::string& s = std::get<std::string>(value);
            gone = s != "stock" && std::find(view->choices.begin(), view->choices.end(), s) == view->choices.end();
            // Listed but refused on the new engine (a bore step its block's wall cannot take): back to stock too.
            for (const SetupPart& p : view->parts) if (!gone && s != "stock" && p.id == s && !p.fits) gone = true;
        }
        if (gone) drop.push_back(id);
    }
    // Gear factors belong to the gearbox they were set on: a gearbox that goes back to stock takes them along.
    if (std::find(drop.begin(), drop.end(), "gearbox_install") != drop.end() && working_.values.count("gear_ratios") != 0) {
        drop.push_back("gear_ratios");
    }
    for (const std::string& id : drop) {
        working_.values.erase(id);
        if (const auto it = entry->preset_setup.find(id); it != entry->preset_setup.end()) working_.values[id] = it->second;
        std::string label = id;
        for (const OptionView& v : pm.options) if (v.def.id == id && !v.def.label.empty()) label = v.def.label;
        notice += (notice.empty() ? "" : ", ") + label;
    }
    if (!drop.empty()) revalidate();
    if (notice.empty()) return notice;
    return (gearbox_change ? "The gearbox does not offer: " : "The engine does not offer: ") + notice + " (back to stock).";
}

std::string Garage::drop_driveline_values() {
    const CatalogEntry* entry = catalog_.find(edit_id_);
    if (entry == nullptr) return {};
    VehicleSetup probe;
    probe.vehicle_id = edit_id_;
    if (const auto it = working_.values.find("powertrain_install"); it != working_.values.end()) probe.values["powertrain_install"] = it->second;
    if (const auto it = working_.values.find("drive_layout"); it != working_.values.end()) probe.values["drive_layout"] = it->second;
    const SetupModel pm = build_setup_model(*entry, options_, context_for(*entry), &probe);
    if (!pm.error.empty()) return {}; // the layout itself does not install: revalidate() reports it
    std::string notice;
    bool changed = false;
    for (const char* id : {"diff_front", "diff_rear"}) {
        const auto it = working_.values.find(id);
        if (it == working_.values.end() || !std::holds_alternative<std::string>(it->second)) continue;
        const OptionView* view = nullptr;
        for (const OptionView& v : pm.options) if (v.def.id == id) view = &v;
        if (view == nullptr) continue;
        const std::string& s = std::get<std::string>(it->second);
        if (!view->available) { // that axle is no longer driven
            working_.values.erase(it);
            changed = true;
            notice += (notice.empty() ? "" : ", ") + (view->def.label.empty() ? std::string(id) : view->def.label);
        } else if (s != "stock" && std::find(view->choices.begin(), view->choices.end(), s) == view->choices.end()) {
            working_.values.erase(it); // the new layout already fits that kind on this axle: the choice is now stock
            changed = true;
        }
    }
    if (changed) revalidate();
    return notice.empty() ? notice : "The new layout does not drive that axle: " + notice + " (back to stock).";
}

// After a powertrain conversion: the drive layout and differentials chosen earlier stay if the converted car can still take
// them (a hybrid keeps its driveline editable), else they go back to stock and the player is told why.
std::string Garage::drop_driveline_values_after_conversion() {
    const CatalogEntry* entry = catalog_.find(edit_id_);
    if (entry == nullptr) return {};
    VehicleSetup probe;
    probe.vehicle_id = edit_id_;
    if (const auto it = working_.values.find("powertrain_install"); it != working_.values.end()) probe.values["powertrain_install"] = it->second;
    const SetupModel pm = build_setup_model(*entry, options_, context_for(*entry), &probe);
    if (!pm.error.empty()) return {};
    std::string notice;
    const auto it = working_.values.find("drive_layout");
    if (it != working_.values.end() && std::holds_alternative<std::string>(it->second)) {
        const OptionView* view = nullptr;
        for (const OptionView& v : pm.options) if (v.def.id == "drive_layout") view = &v;
        const std::string s = std::get<std::string>(it->second);
        const bool takes = view != nullptr && view->available && std::find(view->choices.begin(), view->choices.end(), s) != view->choices.end();
        if (!takes) { // unavailable: the converted car's driveline cannot be swapped; not listed: it already has that layout
            if (view != nullptr && !view->available && s != "stock") {
                notice = "The converted car cannot take the chosen drive layout" +
                         (view->unavailable_reason.empty() ? std::string() : " (" + view->unavailable_reason + ")") + " (back to stock).";
            }
            working_.values.erase(it);
        }
    }
    const std::string diffs = drop_driveline_values(); // differentials of an axle the kept layout does not drive
    if (!diffs.empty()) notice += (notice.empty() ? "" : " ") + diffs;
    revalidate();
    return notice;
}

std::optional<VehicleSetup> Garage::candidate_setup(const std::vector<std::pair<std::string, SetupValue>>& edits, std::string* err) {
    if (!editing()) {
        if (err != nullptr) *err = "no vehicle is being edited";
        return std::nullopt;
    }
    // Everything an edit touches: the working copy, the model it is judged against, the validation and the part tree.
    VehicleSetup saved_working = working_;
    SetupModel saved_model = model_;
    ValidationResult saved_validation = validation_;
    PartTree saved_parts = parts_;
    probe_only_ = true;
    std::optional<VehicleSetup> result;
    std::string failure;
    bool all_accepted = true;
    for (const auto& [option_id, value] : edits) {
        const EditResult r = set_option(option_id, value);
        if (!r.accepted) {
            all_accepted = false;
            failure = r.message;
            break;
        }
    }
    if (all_accepted) result = working_;
    probe_only_ = false;
    working_ = std::move(saved_working);
    model_ = std::move(saved_model);
    validation_ = std::move(saved_validation);
    parts_ = std::move(saved_parts);
    if (!result && err != nullptr) *err = failure;
    return result;
}

void Garage::reset_option(const std::string& option_id) {
    if (!editing()) return;
    working_.values.erase(option_id);
    const CatalogEntry* entry = catalog_.find(edit_id_);
    if (entry != nullptr) { // a preset's own value is the car's reset value, not the base car's stock one
        if (const auto it = entry->preset_setup.find(option_id); it != entry->preset_setup.end()) {
            working_.values[option_id] = it->second;
        }
    }
    revalidate();
}

void Garage::reset_all() {
    if (!editing()) return;
    working_ = preset_setup(edit_id_);
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

std::vector<std::string> Garage::working_model_options() const {
    std::vector<std::string> out;
    for (const SetupOptionDef& def : options_.options) {
        if (def.model_option.empty()) continue;
        const auto it = working_.values.find(def.id);
        if (it != working_.values.end() && std::holds_alternative<bool>(it->second) && std::get<bool>(it->second)) {
            out.push_back(def.model_option);
        }
    }
    return out;
}

std::vector<std::string> Garage::hidden_nodes_for(const CatalogEntry& e, const VehicleSetup& setup) const {
    if (!strip_visual_) return {};
    std::map<std::string, std::string> fitted; // slot -> part id of every weight-reduction choice other than stock
    for (const SetupOptionDef& def : options_.options) {
        if (def.kind != OptionKind::PartChoice || e.interior.count(def.slot) == 0) continue;
        const auto it = setup.values.find(def.id);
        if (it != setup.values.end() && std::holds_alternative<std::string>(it->second) && std::get<std::string>(it->second) != "stock") {
            fitted[def.slot] = std::get<std::string>(it->second);
        }
    }
    return strip_visual_->hidden_nodes(model_stem(e.model_path), fitted);
}

std::vector<std::string> Garage::working_hidden_nodes() const {
    const CatalogEntry* e = editing() ? catalog_.find(edit_id_) : nullptr;
    return e != nullptr ? hidden_nodes_for(*e, working_) : std::vector<std::string>{};
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
    if (setups_equal(working_.values, preset_setup(edit_id_).values)) {
        fs::remove(fs::path(path), ec); // nothing beyond the car's own preset: no player setup file
    } else {
        working_.vehicle_id = edit_id_;
        if (!write_file_atomic(path, setup_to_json(working_), err)) return false;
    }
    saved_at_begin_ = working_;
    refresh_browser();
    return true;
}

void Garage::discard() {
    edit_id_.clear();
    working_ = VehicleSetup{};
    saved_at_begin_ = VehicleSetup{};
    model_ = SetupModel{};
    validation_ = ValidationResult{};
    parts_ = PartTree{};
}

void Garage::remove_drive_dir() {
    if (drive_dir_.empty()) return;
    std::error_code ec;
    fs::remove_all(fs::path(drive_dir_), ec);
    drive_dir_.clear();
}

PartTree Garage::part_tree(const std::string& id) const {
    const CatalogEntry* e = catalog_.find(id);
    if (e == nullptr) {
        PartTree t;
        t.error = "unknown vehicle \"" + id + "\"";
        return t;
    }
    if (editing() && edit_id_ == id) return parts_;
    const VehicleSetup setup = effective_setup(id);
    if (setup.values.empty()) return stock_part_tree(*e, options_, context_for(*e));
    CompiledSetup c = compile_setup(*e, options_, context_for(*e), setup);
    if (!c.ok) {
        PartTree t;
        t.error = c.error;
        return t;
    }
    return std::move(c.parts);
}

std::string Garage::damage_state_path(const std::string& id) const {
    if (!damage_ || config_.user_dir.empty()) return {};
    return rg::damage_state_path(config_.user_dir, id);
}

DamageState Garage::damage_state(const std::string& id, std::string* err) const {
    const std::string path = damage_state_path(id);
    if (path.empty()) return {};
    std::string e;
    auto st = load_damage_state(path, &e);
    if (!st) {
        if (err != nullptr) *err = e;
        return {};
    }
    return *st;
}

bool Garage::repair_damage(const std::string& id, std::string* err) {
    if (catalog_.find(id) == nullptr) {
        if (err != nullptr) *err = "unknown vehicle \"" + id + "\"";
        return false;
    }
    const std::string path = damage_state_path(id);
    if (path.empty()) return true; // no damage model: nothing to repair
    return save_damage_state(path, DamageState{}, err);
}

PartTree Garage::stock_part_tree_of(const std::string& id) const {
    const CatalogEntry* e = catalog_.find(id);
    if (e == nullptr) {
        PartTree t;
        t.error = "unknown vehicle \"" + id + "\"";
        return t;
    }
    return stock_part_tree(*e, options_, context_for(*e));
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
    d.engine_map_cache_dir = context_for(*e).engine_map_cache_dir;
    d.turbo_map_extension_path = turbo_map_extension_path_;
    if (damage_) {
        d.damage_config_path = damage_config_path_;
        d.damage_state_path = damage_state_path(e->id);
    }

    remove_drive_dir();
    std::vector<std::string> notices;
    const VehicleSetup setup = effective_setup_with_notices(id, &notices);
    for (const std::string& n : notices) d.warning += (d.warning.empty() ? "" : " ") + n;
    bool setup_applied = false;
    if (!setup.values.empty()) {
        const std::string dir = config_.work_root + "/drive_" + id + "_" + hex16(fnv1a(setup_to_json(setup)));
        const MaterialisedSetup m = materialise_setup(*e, options_, context_for(*e), setup, dir);
        if (!m.ok) {
            // The saved setup no longer fits the data (a file changed): drive the stock car and say so.
            d.warning += std::string(d.warning.empty() ? "" : " ") + (e->preset_of.empty() ? "The saved setup for " : "The setup of ") + e->title + " was not applied: " + m.error;
            std::error_code ec;
            fs::remove_all(fs::path(dir), ec);
        } else {
            d.engine_title = m.compiled.engine_title;
            d.swap_lines = m.compiled.swap_lines;
            d.parts = m.compiled.parts;
            setup_applied = true;
            if (!d.parts.ok) d.chassis.mass_kg += m.compiled.chassis_mass_delta_kg; // no part-mass tree: the engine delta only
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
            if (!m.compiled.model_options.empty()) {
                d.model_options = m.compiled.model_options;
                d.modified = true;
            }
            d.hidden_nodes = hidden_nodes_for(*e, setup);
            if (!d.hidden_nodes.empty()) d.modified = true;
        }
    }

    // The chassis body mass is the part-mass tree's (rg/part_mass.h): the stock car's sums to the catalog figure.
    if (!setup_applied) d.parts = stock_part_tree(*e, options_, context_for(*e));
    if (d.parts.ok) {
        d.chassis.mass_kg = d.parts.chassis_kg;
        d.chassis.source = "part-mass tree: " + fmt_kg(d.parts.total_kg) + " total";
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
    if(const auto text=detail::read_text_file(d.vehicle_path))if(const auto doc=detail::parse_json(*text);doc&&doc->contains("powertrain")) {
        for(const auto& c:(*doc)["powertrain"]["components"])if(c.value("type",std::string{})=="hybrid_control") {d.manual_gearbox=false;d.assist_auto_shift=true;}
    }
    if (!d.manual_gearbox) d.assist_auto_shift = true; // an automatic car shifts itself
    d.ok = true;
    return d;
}

} // namespace rg

namespace rg {

std::vector<BrowserCar> Garage::browser_cars() const {
    std::vector<BrowserCar> out;
    out.reserve(catalog_.entries.size());
    for (const CatalogEntry& e : catalog_.entries) {
        BrowserCar c;
        c.id = e.id;
        c.title = e.title;
        c.subtitle = e.subtitle;
        c.body_type = e.body_type;
        c.manufacturer = e.manufacturer;
        c.model_path = e.model_path;
        c.preset = !e.preset_of.empty();
        c.paint = e.default_paint;
        c.rim = e.default_rim;
        const VehicleSetup setup = effective_setup(e.id);
        const auto colour = [&](const char* key, std::string& into) {
            const auto it = setup.values.find(key);
            if (it != setup.values.end() && std::holds_alternative<std::string>(it->second) &&
                is_hex_colour(std::get<std::string>(it->second))) {
                into = std::get<std::string>(it->second);
            }
        };
        colour("paint", c.paint);
        colour("rim", c.rim);
        if (const std::optional<VehicleStats> st = stats(e.id, nullptr)) {
            c.layout = st->layout.empty() ? "other" : st->layout;
            c.power_kw = st->peak_power_kw;
            c.power_rpm = st->peak_power_rpm;
            c.torque_nm = st->peak_torque_nm;
            c.torque_rpm = st->peak_torque_rpm;
            c.mass_kg = st->mass_kg;
            c.displacement_known = st->displacement_known;
            c.displacement_l = st->displacement_l;
        }
        out.push_back(std::move(c));
    }
    return out;
}

void Garage::refresh_browser() { browser_.set_cars(browser_cars()); }

} // namespace rg
