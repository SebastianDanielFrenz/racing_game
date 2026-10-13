#include "rg_garage.h"

#include "rg/engine_swap.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

using godot::D_METHOD;
using godot::String;

namespace rg_godot {

namespace {

std::string to_std(const String& s) { return std::string(s.utf8().get_data()); }
String from_std(const std::string& s) { return String::utf8(s.c_str()); }

// ISO 8855 (x forward, y left, z up) -> Godot (x right, y up, z back).
godot::Vector3 iso_to_godot(const rg::GVec3& v) {
    return godot::Vector3(static_cast<float>(-v.y), static_cast<float>(v.z), static_cast<float>(-v.x));
}

const char* kind_name(rg::OptionKind k) { return rg::to_string(k); }

godot::Variant value_to_variant(const rg::SetupValue& v) {
    if (std::holds_alternative<double>(v)) return godot::Variant(std::get<double>(v));
    if (std::holds_alternative<bool>(v)) return godot::Variant(std::get<bool>(v));
    if (std::holds_alternative<std::string>(v)) return godot::Variant(from_std(std::get<std::string>(v)));
    godot::PackedFloat64Array a;
    for (const double d : std::get<std::vector<double>>(v)) a.push_back(d);
    return godot::Variant(a);
}

bool variant_to_value(const godot::Variant& in, rg::SetupValue* out, std::string* err) {
    switch (in.get_type()) {
        case godot::Variant::BOOL: *out = static_cast<bool>(in); return true;
        case godot::Variant::INT: *out = static_cast<double>(static_cast<int64_t>(in)); return true;
        case godot::Variant::FLOAT: *out = static_cast<double>(in); return true;
        case godot::Variant::STRING:
        case godot::Variant::STRING_NAME: *out = to_std(String(in)); return true;
        case godot::Variant::PACKED_FLOAT64_ARRAY:
        case godot::Variant::PACKED_FLOAT32_ARRAY:
        case godot::Variant::ARRAY: {
            const godot::Array a = in;
            std::vector<double> v;
            for (int64_t i = 0; i < a.size(); ++i) v.push_back(static_cast<double>(a[i]));
            *out = std::move(v);
            return true;
        }
        default:
            if (err != nullptr) *err = "unsupported value type";
            return false;
    }
}

godot::Dictionary stats_to_dict(const rg::VehicleStats& s) {
    godot::Dictionary d;
    d["ok"] = true;
    d["engine_name"] = from_std(s.engine_name);
    d["engine_kind"] = from_std(s.engine_kind);
    d["peak_torque_nm"] = s.peak_torque_nm;
    d["peak_torque_rpm"] = s.peak_torque_rpm;
    d["peak_power_kw"] = s.peak_power_kw;
    d["peak_power_rpm"] = s.peak_power_rpm;
    d["figures_declared"] = s.engine_figures_declared;
    d["mass_kg"] = s.mass_kg;
    d["layout"] = from_std(s.layout);
    d["driven_wheels"] = static_cast<int64_t>(s.driven_wheels);
    d["wheel_count"] = static_cast<int64_t>(s.wheel_count);
    d["gear_count"] = static_cast<int64_t>(s.gear_count);
    d["displacement_known"] = s.displacement_known;
    d["displacement_l"] = s.displacement_l;
    d["displacement_source"] = from_std(s.displacement_source);
    return d;
}

godot::Dictionary ok_reply() {
    godot::Dictionary d;
    d["ok"] = true;
    d["error"] = String();
    return d;
}

godot::Dictionary fail_reply(const std::string& err) {
    godot::Dictionary d;
    d["ok"] = false;
    d["error"] = from_std(err);
    return d;
}

} // namespace

RgGarage::~RgGarage() = default;

godot::Dictionary RgGarage::initialize(const String& repo_root, const String& user_dir, const String& work_root,
                                      const String& catalog_path) {
    dyno_.cancel();
    advisor_.reset();
    advisor_error_.clear();
    garage_.reset();
    camera_.reset();
    rg::GarageConfig c;
    c.repo_root = to_std(repo_root);
    c.catalog_path = catalog_path.is_empty() ? c.repo_root + "/data/vehicles/catalog.json" : to_std(catalog_path);
    c.options_path = c.repo_root + "/data/vehicles/setup_options.json";
    c.set_path = c.repo_root + "/data/garage/garage_set.json";
    c.tyre_families_path = c.repo_root + "/data/tyres/families.json";
    c.user_dir = to_std(user_dir);
    c.work_root = to_std(work_root);
    work_root_ = c.work_root;
    std::string err;
    garage_ = rg::Garage::open(c, &err);
    if (garage_ == nullptr) return fail_reply(err);
    // The advisor is optional: a missing/bad data/garage/advisor.json only switches the advice off (reported by get_advice).
    std::string advisor_err;
    // The limit policy is data/garage/limit_policy.json (named by advisor.json); the player's optional layer is <user dir>/limit_policy.json.
    if (auto advisor_config = rg::load_advisor_config(c.repo_root + "/data/garage/advisor.json", &advisor_err, rg::user_limit_policy_path(c.user_dir)))
        advisor_ = std::make_unique<rg::LimitAdvisor>(std::move(*advisor_config));
    else
        advisor_error_ = advisor_err;
    return ok_reply();
}

godot::Array RgGarage::get_vehicles() {
    godot::Array out;
    if (garage_ == nullptr) return out;
    for (const rg::CatalogEntry& e : garage_->catalog().entries) out.push_back(get_vehicle(from_std(e.id)));
    return out;
}

godot::Dictionary RgGarage::get_vehicle(const String& id) {
    godot::Dictionary d;
    if (garage_ == nullptr) return d;
    const rg::CatalogEntry* e = garage_->catalog().find(to_std(id));
    if (e == nullptr) return d;
    d["id"] = from_std(e->id);
    d["title"] = from_std(e->title);
    d["subtitle"] = from_std(e->subtitle);
    d["description"] = from_std(e->description);
    d["default"] = e->id == garage_->catalog().default_id;
    d["selected"] = e->id == garage_->selected_id();
    d["has_setup"] = garage_->has_saved_setup(e->id);
    d["model_path"] = from_std(e->model_path);
    d["sim_name"] = from_std(e->sim_name);
    d["engine_bay"] = from_std(e->engine_bay);
    // The paint a drive of this car shows: the saved setup on top of the preset's overlay (rg::Garage::browser_cars).
    std::string paint = e->default_paint;
    std::string rim = e->default_rim;
    if (const rg::BrowserCar* bc = garage_->browser().find(e->id)) {
        paint = bc->paint;
        rim = bc->rim;
    }
    d["paint"] = from_std(paint);
    d["rim"] = from_std(rim);
    d["body_type"] = from_std(e->body_type);
    d["manufacturer"] = from_std(e->manufacturer);
    d["preset_of"] = from_std(e->preset_of);
    std::string err;
    if (const auto s = garage_->stats(e->id, &err)) {
        d["stats"] = stats_to_dict(*s);
    } else {
        godot::Dictionary sd;
        sd["ok"] = false;
        sd["error"] = from_std(err);
        d["stats"] = sd;
    }
    return d;
}

String RgGarage::get_selected_id() const { return garage_ ? from_std(garage_->selected_id()) : String(); }

godot::Dictionary RgGarage::select(const String& id) {
    if (garage_ == nullptr) return fail_reply("garage not initialised");
    std::string err;
    if (!garage_->select(to_std(id), &err)) return fail_reply(err);
    return ok_reply();
}

godot::Dictionary RgGarage::get_set() const {
    godot::Dictionary out;
    if (garage_ == nullptr) return out;
    const rg::GarageSetDesc& s = garage_->set();
    godot::Dictionary room;
    room["radius_m"] = s.room.radius_m;
    room["height_m"] = s.room.height_m;
    room["floor_colour"] = from_std(s.room.floor_colour);
    room["wall_colour"] = from_std(s.room.wall_colour);
    room["accent_colour"] = from_std(s.room.accent_colour);
    out["room"] = room;
    godot::Dictionary t;
    t["radius_m"] = s.turntable.radius_m;
    t["height_m"] = s.turntable.height_m;
    t["spin_deg_s"] = s.turntable.spin_deg_s;
    t["colour"] = from_std(s.turntable.colour);
    t["rim_colour"] = from_std(s.turntable.rim_colour);
    out["turntable"] = t;
    godot::Dictionary env;
    env["exposure"] = s.environment.exposure;
    env["ambient_energy"] = s.environment.ambient_energy;
    env["ambient_colour"] = from_std(s.environment.ambient_colour);
    env["reflection_energy"] = s.environment.reflection_energy;
    env["floor_reflectivity"] = s.environment.floor_reflectivity;
    out["environment"] = env;
    godot::Array lights;
    for (const rg::GarageLight& l : s.lights) {
        godot::Dictionary d;
        d["id"] = from_std(l.id);
        d["kind"] = from_std(l.kind);
        d["position"] = iso_to_godot(l.position);
        d["target"] = iso_to_godot(l.target);
        d["colour"] = from_std(l.colour);
        d["energy"] = l.energy;
        d["range_m"] = l.range_m;
        d["angle_deg"] = l.angle_deg;
        d["shadow"] = l.shadow;
        lights.push_back(d);
    }
    out["lights"] = lights;
    godot::Array boxes;
    for (const rg::GarageSoftbox& b : s.softboxes) {
        godot::Dictionary d;
        d["id"] = from_std(b.id);
        d["position"] = iso_to_godot(b.position);
        d["target"] = iso_to_godot(b.target);
        d["width_m"] = b.width_m;
        d["height_m"] = b.height_m;
        d["colour"] = from_std(b.colour);
        d["energy"] = b.energy;
        boxes.push_back(d);
    }
    out["softboxes"] = boxes;
    out["transition_s"] = s.transition_s;
    godot::Array areas;
    for (const rg::GarageAreaDef& a : s.areas) {
        godot::Dictionary d;
        d["id"] = from_std(a.id);
        d["label"] = from_std(a.label);
        d["spin"] = a.spin;
        areas.push_back(d);
    }
    out["areas"] = areas;
    return out;
}

// ---- configurator ------------------------------------------------------------

godot::Dictionary RgGarage::begin_edit(const String& id) {
    if (garage_ == nullptr) return fail_reply("garage not initialised");
    std::string err;
    if (!garage_->begin_edit(to_std(id), &err)) return fail_reply(err);
    refresh_dyno();
    return ok_reply();
}

String RgGarage::get_edit_id() const { return garage_ ? from_std(garage_->edit_id()) : String(); }

godot::Array RgGarage::get_options() const {
    godot::Array out;
    if (garage_ == nullptr || !garage_->editing()) return out;
    for (const rg::OptionView& v : garage_->model().options) {
        godot::Dictionary d;
        d["id"] = from_std(v.def.id);
        d["label"] = from_std(v.def.label);
        d["group"] = from_std(v.def.group);
        d["area"] = from_std(v.def.area);
        d["help"] = from_std(v.def.help);
        d["unit"] = from_std(v.def.unit);
        d["kind"] = String(kind_name(v.def.kind));
        d["min"] = v.min;
        d["max"] = v.max;
        d["step"] = v.def.step;
        d["available"] = v.available;
        d["stock"] = value_to_variant(v.stock);
        const rg::SetupValue cur = garage_->current_value(v.def.id);
        d["value"] = value_to_variant(cur);
        d["modified"] = !rg::setup_values_equal(cur, v.stock);
        godot::PackedStringArray choices;
        for (const std::string& c : v.choices) choices.push_back(from_std(c));
        d["choices"] = choices;
        godot::Array parts;
        for (const auto& p : v.parts) {
            godot::Dictionary part;
            part["id"] = from_std(p.id);
            part["label"] = from_std(p.label);
            part["image"] = from_std(p.image);
            part["size"] = from_std(p.detail);
            part["section"] = from_std(p.section);
            part["subtitle"] = from_std(p.subtitle);
            parts.push_back(part);
        }
        d["parts"] = parts;
        godot::PackedStringArray coming_soon;
        for (const std::string& c : v.coming_soon) coming_soon.push_back(from_std(c));
        d["coming_soon"] = coming_soon;
        d["list_size"] = static_cast<int64_t>(v.list_size);
        godot::PackedFloat64Array nums;
        for (const double n : v.stock_numbers) nums.push_back(n);
        d["stock_numbers"] = nums;
        out.push_back(d);
    }
    return out;
}

godot::Dictionary RgGarage::edit_reply(const rg::EditResult& r) const {
    godot::Dictionary d;
    d["accepted"] = r.accepted;
    d["message"] = from_std(r.message);
    d["notice"] = from_std(r.notice);
    d["ok"] = r.validation.ok;
    d["validation_message"] = from_std(r.validation.message);
    d["dirty"] = garage_ != nullptr && garage_->dirty();
    return d;
}

godot::Dictionary RgGarage::set_option(const String& option_id, const godot::Variant& value) {
    if (garage_ == nullptr || !garage_->editing()) {
        godot::Dictionary d;
        d["accepted"] = false;
        d["message"] = String("no edit session");
        d["ok"] = false;
        d["validation_message"] = String();
        d["dirty"] = false;
        return d;
    }
    rg::SetupValue v;
    std::string err;
    if (!variant_to_value(value, &v, &err)) {
        rg::EditResult r;
        r.accepted = false;
        r.message = err;
        r.validation = garage_->validation();
        return edit_reply(r);
    }
    godot::Dictionary d = edit_reply(garage_->set_option(to_std(option_id), v));
    refresh_dyno();
    d["value"] = value_to_variant(garage_->current_value(to_std(option_id)));
    return d;
}

godot::Dictionary RgGarage::reset_option(const String& option_id) {
    godot::Dictionary d;
    if (garage_ == nullptr || !garage_->editing()) return d;
    garage_->reset_option(to_std(option_id));
    refresh_dyno();
    rg::EditResult r;
    r.accepted = true;
    r.validation = garage_->validation();
    d = edit_reply(r);
    d["value"] = value_to_variant(garage_->current_value(to_std(option_id)));
    return d;
}

godot::Dictionary RgGarage::reset_all() {
    godot::Dictionary d;
    if (garage_ == nullptr || !garage_->editing()) return d;
    garage_->reset_all();
    refresh_dyno();
    rg::EditResult r;
    r.accepted = true;
    r.validation = garage_->validation();
    return edit_reply(r);
}

bool RgGarage::is_dirty() const { return garage_ != nullptr && garage_->editing() && garage_->dirty(); }

godot::Dictionary RgGarage::get_validation() const {
    godot::Dictionary d;
    const bool editing = garage_ != nullptr && garage_->editing();
    d["ok"] = editing ? garage_->validation().ok : true;
    d["message"] = editing ? from_std(garage_->validation().message) : String();
    return d;
}

godot::Dictionary RgGarage::save() {
    if (garage_ == nullptr || !garage_->editing()) return fail_reply("no edit session");
    std::string err;
    if (!garage_->save(&err)) return fail_reply(err);
    return ok_reply();
}

void RgGarage::discard() {
    dyno_.cancel();
    if (advisor_) advisor_->cancel();
    if (garage_ != nullptr) garage_->discard();
}

godot::Dictionary RgGarage::get_working_colours() const {
    godot::Dictionary d;
    if (garage_ == nullptr || !garage_->editing()) return d;
    d["paint"] = from_std(garage_->working_paint());
    d["rim"] = from_std(garage_->working_rim());
    return d;
}

// ---- drive hand-over ---------------------------------------------------------

godot::Dictionary RgGarage::prepare_drive(const String& id) {
    if (garage_ == nullptr) return fail_reply("garage not initialised");
    const rg::DriveSelection s = garage_->prepare_drive(to_std(id));
    godot::Dictionary d;
    d["ok"] = s.ok;
    d["error"] = from_std(s.error);
    d["warning"] = from_std(s.warning);
    d["vehicle_id"] = from_std(s.vehicle_id);
    d["sim_name"] = from_std(s.sim_name);
    d["vehicle_path"] = from_std(s.vehicle_path);
    d["model_path"] = from_std(s.model_path);
    d["paint"] = from_std(s.paint);
    d["rim"] = from_std(s.rim);
    d["engine_map_cache_dir"] = from_std(s.engine_map_cache_dir);
    d["modified"] = s.modified;
    godot::Dictionary chassis;
    chassis["mass_kg"] = s.chassis.mass_kg;
    chassis["half_extents"] = godot::Vector3(static_cast<float>(s.chassis.half_extents[0]),
                                             static_cast<float>(s.chassis.half_extents[1]),
                                             static_cast<float>(s.chassis.half_extents[2]));
    chassis["z_m"] = s.chassis.spawn_z_m;
    d["chassis"] = chassis;
    d["assist_auto_clutch"] = s.assist_auto_clutch;
    d["assist_auto_blip"] = s.assist_auto_blip;
    d["assist_auto_shift"] = s.assist_auto_shift;
    d["manual_gearbox"] = s.manual_gearbox;
    d["engine_title"] = from_std(s.engine_title);
    godot::PackedStringArray lines;
    for (const std::string& l : s.swap_lines) lines.push_back(from_std(l));
    d["swap_lines"] = lines;
    return d;
}

godot::Dictionary RgGarage::get_swap_info() const {
    godot::Dictionary d;
    godot::PackedStringArray lines;
    d["mass_delta_kg"] = 0.0;
    d["engine_title"] = String();
    if (garage_ != nullptr && garage_->editing()) {
        for (const std::string& l : garage_->swap_lines()) lines.push_back(from_std(l));
        d["mass_delta_kg"] = garage_->chassis_mass_delta_kg();
        d["engine_title"] = from_std(garage_->model().engine_title);
    }
    d["lines"] = lines;
    return d;
}

godot::Dictionary RgGarage::get_bay_engine(const String& id) const {
    godot::Dictionary d;
    if (garage_ == nullptr) return d;
    const rg::CatalogEntry* car = garage_->catalog().find(to_std(id));
    const rg::EngineEntry* e = car != nullptr ? garage_->bay_engine(car->id) : nullptr;
    if (e == nullptr) return d;
    d["name"] = from_std(e->name);
    d["model_path"] = from_std(e->model_path);
    d["crank_axis"] = car->engine_bay_envelope ? String(car->engine_bay_envelope->crank_axis == 'y' ? "y" : "x") : String();
    const std::array<double, 3> mm = e->envelope_mm.value_or(std::array<double, 3>{0.0, 0.0, 0.0});
    d["envelope_mm"] = godot::Vector3(static_cast<float>(mm[0]), static_cast<float>(mm[1]), static_cast<float>(mm[2]));
    const rg::FitResult fit = rg::check_fit(*car, *e);
    d["fits"] = fit.allowed;
    godot::PackedStringArray reasons;
    for (const std::string& r : fit.reasons) reasons.push_back(from_std(r));
    d["fit_reasons"] = reasons;
    return d;
}

godot::Array RgGarage::get_engines() const {
    godot::Array out;
    if (garage_ == nullptr) return out;
    const rg::EngineStore& store = garage_->engines();
    for (const rg::EngineEntry& e : store.list()) {
        godot::Dictionary d;
        d["key"] = from_std(e.key);
        d["name"] = from_std(e.name);
        d["family"] = from_std(e.family);
        d["class"] = String(rg::to_string(e.cls));
        d["image"] = from_std(e.image);
        d["detail"] = from_std(e.detail);
        d["user"] = e.user;
        d["base_key"] = from_std(e.base_key);
        d["base_name"] = from_std(e.base_name);
        d["mass_kg"] = e.mass_kg;
        std::string err;
        godot::PackedStringArray fuels;
        if (const auto facts = rg::read_engine_facts(e.file, &err)) {
            d["idle_rpm"] = facts->idle_rpm;
            d["limiter_rpm"] = facts->limiter_rpm;
            for (const auto& f : facts->fuels) fuels.push_back(from_std(f.name));
        }
        d["fuels"] = fuels;
        const rg::EnginePeaks p = store.peaks(e);
        d["torque_nm"] = p.torque_nm;
        d["torque_rpm"] = p.torque_rpm;
        d["power_kw"] = p.power_kw;
        d["power_rpm"] = p.power_rpm;
        out.push_back(d);
    }
    return out;
}

godot::Dictionary RgGarage::create_user_engine(const String& base_key, const String& name, const String& patch_json) {
    if (garage_ == nullptr) return fail_reply("garage not initialised");
    std::string err;
    const auto key = garage_->create_user_engine(to_std(base_key), to_std(name), to_std(patch_json), &err);
    if (!key) return fail_reply(err);
    godot::Dictionary d = ok_reply();
    d["key"] = from_std(*key);
    return d;
}

godot::Dictionary RgGarage::update_user_engine(const String& key, const String& patch_json) {
    if (garage_ == nullptr) return fail_reply("garage not initialised");
    std::string err;
    if (!garage_->update_user_engine(to_std(key), to_std(patch_json), &err)) return fail_reply(err);
    return ok_reply();
}

godot::Dictionary RgGarage::rename_user_engine(const String& key, const String& name) {
    if (garage_ == nullptr) return fail_reply("garage not initialised");
    std::string err;
    if (!garage_->rename_user_engine(to_std(key), to_std(name), &err)) return fail_reply(err);
    return ok_reply();
}

godot::Dictionary RgGarage::remove_user_engine(const String& key) {
    if (garage_ == nullptr) return fail_reply("garage not initialised");
    std::string err;
    if (!garage_->remove_user_engine(to_std(key), &err)) return fail_reply(err);
    return ok_reply();
}

void RgGarage::cleanup() {
    dyno_.cancel();
    if (advisor_) advisor_->cancel();
    if (garage_ != nullptr) garage_->cleanup();
}

int RgGarage::get_work_file_count() const {
    if (work_root_.empty()) return 0;
    return static_cast<int>(rg::ScopedWorkDir::file_count(work_root_));
}

// ---- camera ------------------------------------------------------------------

godot::Dictionary RgGarage::setup_camera(const String& id, const godot::Vector3& model_min_iso,
                                         const godot::Vector3& model_max_iso, bool has_model_bounds) {
    if (garage_ == nullptr) return fail_reply("garage not initialised");
    rg::GVec3 lo{model_min_iso.x, model_min_iso.y, model_min_iso.z};
    rg::GVec3 hi{model_max_iso.x, model_max_iso.y, model_max_iso.z};
    std::string err;
    const auto subject = garage_->subject(to_std(id), has_model_bounds ? &lo : nullptr, has_model_bounds ? &hi : nullptr, &err);
    if (!subject) return fail_reply(err);
    if (camera_) {
        camera_->set_subject(*subject);
    } else {
        camera_.emplace(garage_->set(), *subject);
        camera_->go_to("overview", true);
    }
    return ok_reply();
}

bool RgGarage::camera_go_to(const String& area_id, bool instant) {
    return camera_ && camera_->go_to(to_std(area_id), instant);
}

void RgGarage::camera_update(double dt_s) {
    if (camera_) camera_->update(dt_s);
}

godot::Dictionary RgGarage::get_camera() const {
    godot::Dictionary d;
    if (!camera_) return d;
    const rg::GarageCameraPose p = camera_->pose();
    d["position"] = iso_to_godot(p.position);
    d["look_at"] = iso_to_godot(p.look_at);
    d["fov_deg"] = p.fov_deg;
    d["car_yaw_rad"] = camera_->car_yaw_deg() * 3.14159265358979323846 / 180.0;
    d["area"] = from_std(camera_->area());
    d["settled"] = camera_->settled();
    d["in_transition"] = camera_->in_transition();
    d["progress"] = camera_->progress();
    return d;
}

String RgGarage::get_area_for_option(const String& option_id) const {
    if (garage_ != nullptr)
        if (const rg::SetupOptionDef* def = garage_->options().find(to_std(option_id))) return from_std(def->area);
    return String("overview");
}

void RgGarage::refresh_dyno() {
    if(!garage_ || !garage_->editing() || !garage_->validation().ok) { dyno_.cancel(); if(advisor_) advisor_->cancel(); return; }
    const auto* entry=garage_->catalog().find(garage_->edit_id());
    if(!entry) return;
    if(advisor_) {
        // The advisor sees the WHOLE pending setup (colours and gearing matter as much as the engine): what a drive would load.
        const std::string advisor_key=garage_->edit_id()+"|"+rg::setup_to_json(garage_->working());
        advisor_->request({*entry,garage_->options(),garage_->context_for(*entry),garage_->working(),advisor_key});
    }
    rg::VehicleSetup engine_setup; engine_setup.vehicle_id=entry->id;
    for(const auto& option:garage_->options().options) {
        if(option.file!="engine" && option.file!="turbo" && option.part_format!="physics_sim.engine/1" && option.part_format!="physics_sim.turbo_configuration/1") continue;
        auto value=garage_->working().values.find(option.id);
        if(value!=garage_->working().values.end()) engine_setup.values.emplace(*value);
    }
    const std::string key=rg::setup_to_json(engine_setup);
    dyno_.request({*entry,garage_->options(),garage_->context_for(*entry),engine_setup,key});
}
godot::Dictionary RgGarage::get_advice() const {
    godot::Dictionary out;
    out["available"]=advisor_!=nullptr;
    if(!advisor_) { out["error"]=from_std(advisor_error_); return out; }
    const auto result=advisor_->result();
    out["busy"]=result.busy; out["progress"]=result.progress; out["stage"]=from_std(result.stage);
    out["error"]=from_std(result.error); out["revision"]=int64_t(result.revision);
    out["have_result"]=result.have_result; out["solving"]=result.solving; out["solve_progress"]=result.solve_progress;
    out["maps_seconds"]=result.maps_seconds; out["sweep_seconds"]=result.sweep_seconds;
    out["vehicle_id"]=from_std(result.vehicle_id);
    godot::Array items;
    for(const auto& item:result.items) {
        godot::Dictionary d;
        d["id"]=from_std(item.id); d["kind"]=from_std(item.kind_name);
        d["severity"]=int(item.severity); d["severity_name"]=from_std(rg::to_string(item.severity));
        d["title"]=from_std(item.title);
        godot::Array lines; std::string text=item.title;
        for(const auto& line:item.lines) { lines.push_back(from_std(line)); text+="\n"+line; }
        if(!item.suggestion.empty()) text+="\n"+item.suggestion;
        d["lines"]=lines; d["text"]=from_std(text); d["suggestion"]=from_std(item.suggestion);
        d["option_id"]=from_std(item.option_id);
        d["rpm_from"]=item.rpm_from; d["rpm_to"]=item.rpm_to;
        d["remedy_state"]=int(item.remedy_state); d["remedy_text"]=from_std(item.remedy_text);
        d["can_solve"]=item.can_solve;
        items.push_back(d);
    }
    out["items"]=items;
    return out;
}
void RgGarage::solve_advice(const String& item_id) {
    if(advisor_) advisor_->solve(to_std(item_id));
}
godot::Dictionary RgGarage::get_dyno() const {
    const auto result=dyno_.result();
    godot::Dictionary out;
    out["busy"]=result.busy; out["progress"]=result.progress;
    out["revision"]=int64_t(result.revision);
    out["vehicle_id"]=from_std(result.vehicle_id); out["error"]=from_std(result.error);
    godot::Array points;
    for(const auto& point:result.points) {
        godot::Dictionary p; p["rpm"]=point.rpm; p["torque_nm"]=point.torque_nm; p["power_kw"]=point.power_kw;
        p["turbo_simulation"]=point.turbo_simulation;
        p["compressor_sim_fraction"]=point.compressor_sim_fraction;
        p["turbine_sim_fraction"]=point.turbine_sim_fraction;
        p["turbo_fallback_fraction"]=point.turbo_fallback_fraction;
        points.push_back(p);
    }
    out["points"]=points;
    godot::Array provisional;
    for(const auto& point:result.provisional_points) {
        godot::Dictionary p;p["rpm"]=point.rpm;p["torque_nm"]=point.torque_nm;p["power_kw"]=point.power_kw;
        p["turbo_simulation"]=point.turbo_simulation;
        p["compressor_sim_fraction"]=point.compressor_sim_fraction;
        p["turbine_sim_fraction"]=point.turbine_sim_fraction;
        p["turbo_fallback_fraction"]=point.turbo_fallback_fraction;
        provisional.push_back(p);
    }
    out["provisional_points"]=provisional;
    return out;
}

void RgGarage::_bind_methods() {
    using godot::ClassDB;
    ClassDB::bind_method(D_METHOD("initialize", "repo_root", "user_dir", "work_root", "catalog_path"), &RgGarage::initialize, DEFVAL(String()));
    bind_browser_methods();
    ClassDB::bind_method(D_METHOD("get_dyno"), &RgGarage::get_dyno);
    ClassDB::bind_method(D_METHOD("get_advice"), &RgGarage::get_advice);
    ClassDB::bind_method(D_METHOD("solve_advice", "item_id"), &RgGarage::solve_advice);
    ClassDB::bind_method(D_METHOD("is_ready"), &RgGarage::is_ready);
    ClassDB::bind_method(D_METHOD("get_vehicles"), &RgGarage::get_vehicles);
    ClassDB::bind_method(D_METHOD("get_vehicle", "id"), &RgGarage::get_vehicle);
    ClassDB::bind_method(D_METHOD("get_selected_id"), &RgGarage::get_selected_id);
    ClassDB::bind_method(D_METHOD("select", "id"), &RgGarage::select);
    ClassDB::bind_method(D_METHOD("get_set"), &RgGarage::get_set);
    ClassDB::bind_method(D_METHOD("begin_edit", "id"), &RgGarage::begin_edit);
    ClassDB::bind_method(D_METHOD("get_edit_id"), &RgGarage::get_edit_id);
    ClassDB::bind_method(D_METHOD("get_options"), &RgGarage::get_options);
    ClassDB::bind_method(D_METHOD("set_option", "option_id", "value"), &RgGarage::set_option);
    ClassDB::bind_method(D_METHOD("reset_option", "option_id"), &RgGarage::reset_option);
    ClassDB::bind_method(D_METHOD("reset_all"), &RgGarage::reset_all);
    ClassDB::bind_method(D_METHOD("is_dirty"), &RgGarage::is_dirty);
    ClassDB::bind_method(D_METHOD("get_validation"), &RgGarage::get_validation);
    ClassDB::bind_method(D_METHOD("get_swap_info"), &RgGarage::get_swap_info);
    ClassDB::bind_method(D_METHOD("get_engines"), &RgGarage::get_engines);
    ClassDB::bind_method(D_METHOD("get_bay_engine", "id"), &RgGarage::get_bay_engine);
    ClassDB::bind_method(D_METHOD("create_user_engine", "base_key", "name", "patch_json"), &RgGarage::create_user_engine);
    ClassDB::bind_method(D_METHOD("update_user_engine", "key", "patch_json"), &RgGarage::update_user_engine);
    ClassDB::bind_method(D_METHOD("rename_user_engine", "key", "name"), &RgGarage::rename_user_engine);
    ClassDB::bind_method(D_METHOD("remove_user_engine", "key"), &RgGarage::remove_user_engine);
    ClassDB::bind_method(D_METHOD("save"), &RgGarage::save);
    ClassDB::bind_method(D_METHOD("discard"), &RgGarage::discard);
    ClassDB::bind_method(D_METHOD("get_working_colours"), &RgGarage::get_working_colours);
    ClassDB::bind_method(D_METHOD("prepare_drive", "id"), &RgGarage::prepare_drive);
    ClassDB::bind_method(D_METHOD("cleanup"), &RgGarage::cleanup);
    ClassDB::bind_method(D_METHOD("get_work_file_count"), &RgGarage::get_work_file_count);
    ClassDB::bind_method(D_METHOD("setup_camera", "id", "model_min_iso", "model_max_iso", "has_model_bounds"), &RgGarage::setup_camera);
    ClassDB::bind_method(D_METHOD("camera_go_to", "area_id", "instant"), &RgGarage::camera_go_to);
    ClassDB::bind_method(D_METHOD("camera_update", "dt_s"), &RgGarage::camera_update);
    ClassDB::bind_method(D_METHOD("get_camera"), &RgGarage::get_camera);
    ClassDB::bind_method(D_METHOD("get_area_for_option", "option_id"), &RgGarage::get_area_for_option);
}

} // namespace rg_godot
