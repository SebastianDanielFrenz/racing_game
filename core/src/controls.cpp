// rg/controls.cpp - see controls.h.
#include "rg/controls.h"

#include "control_json.h"
#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <utility>

namespace rg {

namespace {

using detail::json;

std::string joy_key(const std::string& guid, int ordinal) {
    return "joy:" + (guid.empty() ? std::string("noguid") : guid) + "#" + std::to_string(ordinal);
}

bool is_pad_class(DeviceClass c) { return c == DeviceClass::Gamepad || c == DeviceClass::Wheel || c == DeviceClass::Generic; }

bool has_key(const InputSnapshot& in, const std::string& name) {
    return std::find(in.keys.begin(), in.keys.end(), name) != in.keys.end();
}

bool has_int(const std::vector<int>& v, int x) { return std::find(v.begin(), v.end(), x) != v.end(); }

bool binding_in_row(const Binding& b, const ActionDef& a, int sign) {
    if (a.kind == ActionKind::Axis && a.range == AxisRange::Signed) {
        if (sign == 0) return b.is_analogue() && b.span == AxisSpan::Full;
        return !(b.is_analogue() && b.span == AxisSpan::Full) && static_cast<int>(b.sign) == sign;
    }
    return true;
}

AxisRange target_range(const ActionDef& a) { return a.kind == ActionKind::Axis ? a.range : AxisRange::Unit; }

bool contains_ci(const std::string& hay, const char* needle) {
    std::string h = hay;
    std::transform(h.begin(), h.end(), h.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return h.find(needle) != std::string::npos;
}

bool read_file_to(const std::string& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

} // namespace

DeviceClass classify_joypad(const std::string& name, bool standard_mapping) {
    if (!standard_mapping) return DeviceClass::Generic;
    static const char* const kWheelNames[] = {"wheel", "g29", "g920", "g923", "g27", "g25", "t300", "t150", "t248", "fanatec", "moza", "simucube", "thrustmaster", "driving force"};
    for (const char* w : kWheelNames) {
        if (contains_ci(name, w)) return DeviceClass::Wheel;
    }
    return DeviceClass::Gamepad;
}

Controls::Controls() { reset_records(); }

void Controls::reset_records() {
    records_.clear();
    Record kb;
    kb.key = "keyboard";
    kb.cls = DeviceClass::Keyboard;
    kb.name = "Keyboard";
    kb.connected = true;
    Record mouse;
    mouse.key = "mouse";
    mouse.cls = DeviceClass::Mouse;
    mouse.name = "Mouse";
    mouse.connected = true;
    records_.push_back(std::move(kb));
    records_.push_back(std::move(mouse));
}

void Controls::changed() {
    ++revision_;
    dirty_ = true;
}

// ---- data ---------------------------------------------------------------------------------

bool Controls::load_data_text(const std::string& actions_json, const std::map<std::string, std::string>& default_profiles_json,
                              std::string* err) {
    std::optional<ActionSchema> schema = ActionSchema::parse(actions_json, "actions.json", err);
    if (!schema) return false;
    std::map<DeviceClass, std::map<std::string, std::vector<Binding>>> defaults;
    for (const auto& [name, text] : default_profiles_json) {
        const std::string origin = "defaults/" + name + ".json";
        const std::optional<json> parsed = detail::parse_json(text);
        if (!parsed || !parsed->is_object()) {
            if (err != nullptr) *err = origin + ": not a JSON object";
            return false;
        }
        std::string format;
        std::string cls_name;
        if (!detail::get_string(*parsed, "format", format) || format != "rg.control_profile/1" ||
            !detail::get_string(*parsed, "class", cls_name)) {
            if (err != nullptr) *err = origin + ": \"format\" must be \"rg.control_profile/1\" and \"class\" is required";
            return false;
        }
        const std::optional<DeviceClass> cls = device_class_from_name(cls_name);
        if (!cls) {
            if (err != nullptr) *err = origin + ": unknown class \"" + cls_name + "\"";
            return false;
        }
        const auto bindings = parsed->find("bindings");
        if (bindings == parsed->end() || !bindings->is_object()) {
            if (err != nullptr) *err = origin + ": \"bindings\" must be an object";
            return false;
        }
        std::map<std::string, std::vector<Binding>> map;
        for (const auto& [action, list] : bindings->items()) {
            const ActionDef* def = schema->find(action);
            if (def == nullptr) {
                if (err != nullptr) *err = origin + ": unknown action \"" + action + "\"";
                return false;
            }
            if (!list.is_array()) {
                if (err != nullptr) *err = origin + ": \"" + action + "\" must be an array";
                return false;
            }
            std::vector<Binding> v;
            for (const json& bj : list) {
                Binding b;
                std::string why;
                if (!detail::binding_from_json(bj, b, &why)) {
                    if (err != nullptr) *err = origin + ": " + action + ": " + why;
                    return false;
                }
                v.push_back(std::move(b));
            }
            map[action] = std::move(v);
        }
        defaults[*cls] = std::move(map);
    }
    schema_ = std::move(*schema);
    defaults_ = std::move(defaults);
    // Validate the defaults against the device rules once, here, so a bad data file is found at start-up.
    for (const auto& [cls, map] : defaults_) {
        Record probe;
        probe.cls = cls;
        probe.key = to_string(cls);
        for (const auto& [action, list] : map) {
            const ActionDef* def = schema_.find(action);
            for (const Binding& b : list) {
                std::string why;
                if (!binding_suits(probe, *def, b, &why)) {
                    if (err != nullptr) *err = std::string("defaults/") + to_string(cls) + ".json: " + action + ": " + why;
                    return false;
                }
            }
        }
    }
    ++revision_;
    return true;
}

bool Controls::load_data(const std::string& data_dir, std::string* err) {
    const std::string base = data_dir + "/controls";
    std::string actions;
    if (!read_file_to(base + "/actions.json", actions)) {
        if (err != nullptr) *err = base + "/actions.json: cannot read file";
        return false;
    }
    std::map<std::string, std::string> profiles;
    for (const char* name : {"keyboard", "mouse", "gamepad"}) {
        std::string text;
        if (!read_file_to(base + "/defaults/" + name + ".json", text)) {
            if (err != nullptr) *err = base + "/defaults/" + name + ".json: cannot read file";
            return false;
        }
        profiles[name] = std::move(text);
    }
    return load_data_text(actions, profiles, err);
}

// ---- lookup helpers -------------------------------------------------------------------------------

Controls::Record* Controls::find(const std::string& key) {
    for (Record& r : records_) {
        if (r.key == key) return &r;
    }
    return nullptr;
}

const Controls::Record* Controls::find(const std::string& key) const {
    for (const Record& r : records_) {
        if (r.key == key) return &r;
    }
    return nullptr;
}

const std::map<std::string, std::vector<Binding>>* Controls::base_overrides(const Record& r) const {
    if (r.customised) return &r.overrides;
    if (is_pad_class(r.cls) && r.ordinal > 1) {
        const Record* first = find(joy_key(r.guid, 1));
        if (first != nullptr && first->customised) return &first->overrides;
    }
    return nullptr;
}

const std::vector<Binding>* Controls::default_list(DeviceClass cls, const std::string& action) const {
    if (cls == DeviceClass::Wheel) cls = DeviceClass::Gamepad;
    const auto d = defaults_.find(cls);
    if (d == defaults_.end()) return nullptr;
    const auto it = d->second.find(action);
    return it == d->second.end() ? nullptr : &it->second;
}

std::vector<Binding> Controls::effective(const Record& r, const std::string& action) const {
    if (const auto* base = base_overrides(r)) {
        const auto it = base->find(action);
        if (it != base->end()) return it->second;
    }
    if (const auto* d = default_list(r.cls, action)) return *d;
    return {};
}

const std::vector<std::vector<Binding>>& Controls::effective_all(const Record& r) const {
    if (eff_cache_revision_ != revision_) {
        eff_cache_.clear();
        eff_cache_revision_ = revision_;
    }
    auto it = eff_cache_.find(r.key);
    if (it == eff_cache_.end()) {
        std::vector<std::vector<Binding>> all;
        all.reserve(schema_.size());
        for (const ActionDef& a : schema_.actions()) all.push_back(effective(r, a.id));
        it = eff_cache_.emplace(r.key, std::move(all)).first;
    }
    return it->second;
}

bool Controls::binding_suits(const Record& r, const ActionDef& a, const Binding& b, std::string* err) const {
    const auto fail = [&](const char* why) {
        if (err != nullptr) *err = why;
        return false;
    };
    if (r.cls == DeviceClass::Keyboard) {
        if (b.type != BindingType::Key) return fail("the keyboard only binds keys");
    } else if (r.cls == DeviceClass::Mouse) {
        if (b.type != BindingType::MouseButton && b.type != BindingType::MouseMotion) return fail("the mouse only binds its buttons and motion");
        if (a.group == ControlGroup::Menu) return fail("the mouse cannot be bound to menu actions");
    } else {
        if (b.type != BindingType::JoyButton && b.type != BindingType::JoyAxis) return fail("a pad only binds its buttons and axes");
    }
    const bool delta = a.kind == ActionKind::Axis && a.range == AxisRange::Delta;
    if (delta && b.type != BindingType::MouseMotion) return fail("this action takes mouse motion only");
    if (!delta && b.type == BindingType::MouseMotion) return fail("mouse motion only drives the mouse-look actions");
    if (b.type == BindingType::Key && b.key == "Escape" && a.id != "menu_back") return fail("Esc is reserved (it always goes back / pauses)");
    return true;
}

void Controls::make_custom(Record& r) {
    if (r.customised) return;
    if (const auto* base = base_overrides(r)) r.overrides = *base; // a second identical pad starts from the first one's setup
    r.customised = true;
}

// ---- persistence -------------------------------------------------------------------------------------

ControlsLoadReport Controls::load_document(const std::string& text) {
    ControlsLoadReport rep;
    reset_records();
    const auto bad = [&](const std::string& message) {
        rep.parse_error = true;
        rep.message = "controls file: " + message + " - using the built-in defaults";
        reset_records();
        ++revision_;
        return rep;
    };
    const std::optional<json> parsed = detail::parse_json(text);
    if (!parsed) return bad("syntax error");
    if (!parsed->is_object()) return bad("not a JSON object");
    std::string format;
    if (!detail::get_string(*parsed, "format", format) || format.rfind("rg.controls/", 0) != 0) return bad("\"format\" is not rg.controls/N");
    int version = 0;
    try {
        version = std::stoi(format.substr(12));
    } catch (...) { // NOLINT: std::stoi throws only on a malformed number; the format string is the whole input
        return bad("\"format\" has no version number");
    }
    rep.version_found = version;
    if (version != kControlsFormatVersion) {
        rep.unknown_version = true;
        rep.message = "controls file: format \"" + format + "\" is not supported by this build - using the built-in defaults";
        reset_records();
        ++revision_;
        return rep;
    }
    const auto devices = parsed->find("devices");
    if (devices == parsed->end() || !devices->is_array()) return bad("\"devices\" must be an array");
    std::set<std::string> seen;
    std::size_t index = 0;
    for (const json& d : *devices) {
        const std::string where = "devices[" + std::to_string(index++) + "]";
        if (!d.is_object()) {
            rep.dropped.push_back(where + ": not an object");
            continue;
        }
        Record rec;
        if (!detail::get_string(d, "key", rec.key) || rec.key.empty()) {
            rep.dropped.push_back(where + ": \"key\" missing");
            continue;
        }
        if (!seen.insert(rec.key).second) {
            rep.dropped.push_back(where + ": duplicate device \"" + rec.key + "\"");
            continue;
        }
        std::string cls_name;
        detail::get_string(d, "class", cls_name);
        const std::optional<DeviceClass> cls = device_class_from_name(cls_name);
        if (rec.key == "keyboard" || rec.key == "mouse") {
            rec.cls = rec.key == "keyboard" ? DeviceClass::Keyboard : DeviceClass::Mouse;
            rec.name = rec.key == "keyboard" ? "Keyboard" : "Mouse";
            rec.connected = true;
        } else if (rec.key.rfind("joy:", 0) == 0 && rec.key.find('#') != std::string::npos) {
            const std::size_t hash = rec.key.rfind('#');
            rec.guid = rec.key.substr(4, hash - 4);
            int ordinal = 0;
            try {
                ordinal = std::stoi(rec.key.substr(hash + 1));
            } catch (...) { // NOLINT
                ordinal = 0;
            }
            if (ordinal < 1 || ordinal > 64) {
                rep.dropped.push_back(where + ": bad ordinal in \"" + rec.key + "\"");
                continue;
            }
            rec.ordinal = ordinal;
            rec.cls = (cls && is_pad_class(*cls)) ? *cls : DeviceClass::Generic;
            detail::get_string(d, "name", rec.name);
            double v = 0;
            if (detail::get_number(d, "vendor", v)) rec.vendor = static_cast<int>(v);
            if (detail::get_number(d, "product", v)) rec.product = static_cast<int>(v);
        } else {
            rep.dropped.push_back(where + ": unknown device key \"" + rec.key + "\"");
            continue;
        }
        const auto ov = d.find("overrides");
        if (ov != d.end()) {
            if (!ov->is_object()) {
                rep.dropped.push_back(where + ": \"overrides\" must be an object");
            } else {
                rec.customised = true;
                for (const auto& [action, list] : ov->items()) {
                    const std::string at = where + " " + action;
                    const ActionDef* def = schema_.find(action);
                    if (def == nullptr) {
                        rep.dropped.push_back(at + ": unknown action");
                        continue;
                    }
                    if (!list.is_array() || list.size() > kMaxBindingsPerAction) {
                        rep.dropped.push_back(at + ": not a list of at most " + std::to_string(kMaxBindingsPerAction) + " bindings");
                        continue;
                    }
                    std::vector<Binding> v;
                    bool ok = true;
                    for (const json& bj : list) {
                        Binding b;
                        std::string why;
                        if (!detail::binding_from_json(bj, b, &why) || !binding_suits(rec, *def, b, &why)) {
                            rep.dropped.push_back(at + ": " + why);
                            ok = false;
                            break;
                        }
                        v.push_back(std::move(b));
                    }
                    if (ok) rec.overrides[action] = std::move(v);
                }
            }
        }
        if (rec.key == "keyboard") records_[0] = std::move(rec);
        else if (rec.key == "mouse") records_[1] = std::move(rec);
        else records_.push_back(std::move(rec));
    }
    if (!rep.dropped.empty()) rep.message = "controls file: " + std::to_string(rep.dropped.size()) + " entr" + (rep.dropped.size() == 1 ? "y" : "ies") + " could not be read and were dropped";
    ++revision_;
    return rep;
}

ControlsLoadReport Controls::load_text(const std::string& json_text) {
    ControlsLoadReport rep = load_document(json_text);
    dirty_ = false;
    return rep;
}

ControlsLoadReport Controls::load_file(const std::string& path) {
    namespace fs = std::filesystem;
    std::string text;
    if (!read_file_to(path, text)) {
        reset_records();
        ++revision_;
        dirty_ = false;
        ControlsLoadReport rep;
        rep.file_missing = true;
        return rep;
    }
    ControlsLoadReport rep = load_text(text);
    if (!rep.ok()) {
        std::error_code ec;
        const std::string backup = path + ".bak";
        fs::copy_file(path, backup, fs::copy_options::overwrite_existing, ec);
        if (!ec) {
            rep.backup_written = true;
            rep.backup_path = backup;
            rep.message += " (the file was kept as " + backup + ")";
        }
    }
    return rep;
}

std::string Controls::to_json() const {
    json doc = json::object();
    doc["format"] = "rg.controls/" + std::to_string(kControlsFormatVersion);
    json devices = json::array();
    std::vector<const Record*> order;
    for (const Record& r : records_) order.push_back(&r);
    std::sort(order.begin(), order.end(), [](const Record* a, const Record* b) {
        const auto rank = [](const Record* r) { return r->key == "keyboard" ? 0 : (r->key == "mouse" ? 1 : 2); };
        if (rank(a) != rank(b)) return rank(a) < rank(b);
        return a->key < b->key;
    });
    for (const Record* r : order) {
        const bool pad = is_pad_class(r->cls);
        if (!pad && !r->customised) continue; // keyboard / mouse are written only once the player changed something
        json d = json::object();
        d["key"] = r->key;
        d["class"] = to_string(r->cls);
        if (pad) {
            d["name"] = r->name;
            d["guid"] = r->guid;
            d["vendor"] = r->vendor;
            d["product"] = r->product;
            d["ordinal"] = r->ordinal;
        }
        if (r->customised) {
            json ov = json::object();
            for (const ActionDef& a : schema_.actions()) { // schema order: byte-stable output
                const auto it = r->overrides.find(a.id);
                if (it == r->overrides.end()) continue;
                json list = json::array();
                for (const Binding& b : it->second) list.push_back(detail::binding_to_json(b));
                ov[a.id] = std::move(list);
            }
            d["overrides"] = std::move(ov);
        }
        devices.push_back(std::move(d));
    }
    doc["devices"] = std::move(devices);
    return doc.dump(2) + "\n";
}

bool Controls::save_file(const std::string& path, std::string* err) {
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path target(path);
    if (target.has_parent_path()) fs::create_directories(target.parent_path(), ec);
    const fs::path tmp = fs::path(path + ".tmp");
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) {
            if (err != nullptr) *err = path + ": cannot write";
            return false;
        }
        out << to_json();
        out.flush();
        if (!out) {
            if (err != nullptr) *err = path + ": write failed";
            return false;
        }
    }
    ec.clear();
    fs::rename(tmp, target, ec);
    if (ec) {
        if (err != nullptr) *err = path + ": rename failed: " + ec.message();
        fs::remove(tmp, ec);
        return false;
    }
    dirty_ = false;
    return true;
}

// ---- devices ---------------------------------------------------------------------------------------------

std::string Controls::joypad_connected(int slot, const std::string& guid, const std::string& name, int vendor, int product,
                                       bool standard_mapping) {
    joypad_disconnected(slot); // a re-announced slot first gives its ordinal back
    std::set<int> taken;
    for (const Record& r : records_) {
        if (is_pad_class(r.cls) && r.connected && r.guid == guid) taken.insert(r.ordinal);
    }
    int ordinal = 1;
    while (taken.count(ordinal) != 0) ++ordinal;
    const std::string key = joy_key(guid, ordinal);
    Record* rec = find(key);
    const DeviceClass cls = classify_joypad(name, standard_mapping);
    bool is_new = false;
    if (rec == nullptr) {
        records_.emplace_back();
        rec = &records_.back();
        rec->key = key;
        is_new = true;
    }
    const bool meta_changed = is_new || rec->name != name || rec->vendor != vendor || rec->product != product || rec->cls != cls;
    rec->cls = cls;
    rec->name = name.empty() ? "Controller" : name;
    rec->guid = guid;
    rec->vendor = vendor;
    rec->product = product;
    rec->ordinal = ordinal;
    rec->connected = true;
    rec->slot = slot;
    ++revision_;
    if (meta_changed) dirty_ = true;
    return key;
}

void Controls::joypad_disconnected(int slot) {
    for (Record& r : records_) {
        if (is_pad_class(r.cls) && r.connected && r.slot == slot) {
            r.connected = false;
            r.slot = -1;
            ++revision_;
        }
    }
}

std::vector<DeviceInfo> Controls::devices() const {
    std::vector<const Record*> connected;
    std::vector<const Record*> remembered;
    for (std::size_t i = 2; i < records_.size(); ++i) (records_[i].connected ? connected : remembered).push_back(&records_[i]);
    std::sort(connected.begin(), connected.end(), [](const Record* a, const Record* b) { return a->slot < b->slot; });
    std::sort(remembered.begin(), remembered.end(), [](const Record* a, const Record* b) { return a->key < b->key; });
    std::vector<const Record*> all = {&records_[0], &records_[1]};
    all.insert(all.end(), connected.begin(), connected.end());
    all.insert(all.end(), remembered.begin(), remembered.end());
    std::vector<DeviceInfo> out;
    for (const Record* r : all) {
        DeviceInfo d;
        d.key = r->key;
        d.cls = r->cls;
        d.name = r->name;
        d.label = is_pad_class(r->cls) ? r->name + " #" + std::to_string(r->ordinal) : r->name;
        d.guid = r->guid;
        d.vendor = r->vendor;
        d.product = r->product;
        d.ordinal = r->ordinal;
        d.connected = r->connected;
        d.slot = r->slot;
        d.customised = r->customised;
        d.inherits = !r->customised && base_overrides(*r) != nullptr;
        out.push_back(std::move(d));
    }
    return out;
}

std::optional<DeviceInfo> Controls::device(const std::string& key) const {
    for (DeviceInfo& d : devices()) {
        if (d.key == key) return std::move(d);
    }
    return std::nullopt;
}

std::string Controls::key_for_slot(int slot) const {
    for (const Record& r : records_) {
        if (is_pad_class(r.cls) && r.connected && r.slot == slot) return r.key;
    }
    return {};
}

// ---- bindings --------------------------------------------------------------------------------------------

std::vector<std::string> Controls::applicable_actions(const std::string& device_key) const {
    std::vector<std::string> out;
    const Record* r = find(device_key);
    if (r == nullptr) return out;
    for (const ActionDef& a : schema_.actions()) {
        const bool delta = a.kind == ActionKind::Axis && a.range == AxisRange::Delta;
        bool ok = true;
        if (r->cls == DeviceClass::Keyboard || is_pad_class(r->cls)) ok = !delta;
        else if (r->cls == DeviceClass::Mouse) ok = a.group != ControlGroup::Menu;
        if (ok) out.push_back(a.id);
    }
    return out;
}

std::vector<Binding> Controls::effective_bindings(const std::string& device_key, const std::string& action) const {
    const Record* r = find(device_key);
    return r == nullptr ? std::vector<Binding>{} : effective(*r, action);
}

std::vector<Binding> Controls::default_bindings(const std::string& device_key, const std::string& action) const {
    const Record* r = find(device_key);
    if (r == nullptr) return {};
    const auto* d = default_list(r->cls, action);
    return d == nullptr ? std::vector<Binding>{} : *d;
}

bool Controls::is_overridden(const std::string& device_key, const std::string& action) const {
    const Record* r = find(device_key);
    if (r == nullptr) return false;
    const auto* base = base_overrides(*r);
    return base != nullptr && base->count(action) != 0;
}

std::vector<RowInfo> Controls::rows(const std::string& device_key) const {
    std::vector<RowInfo> out;
    const Record* rec = find(device_key);
    if (rec == nullptr) return out;
    const std::vector<Conflict> conflicts_all = conflicts(device_key);
    for (const std::string& id : applicable_actions(device_key)) {
        const ActionDef& a = *schema_.find(id);
        const std::vector<Binding> eff = effective(*rec, id);
        const bool signed_axis = a.kind == ActionKind::Axis && a.range == AxisRange::Signed;
        std::vector<int> signs;
        if (!signed_axis) signs = {0};
        else if (is_pad_class(rec->cls)) signs = {0, -1, 1};
        else signs = {-1, 1};
        for (const int sign : signs) {
            RowInfo row;
            row.action = id;
            row.sign = sign;
            row.group = a.group;
            row.overridden = is_overridden(device_key, id);
            if (!signed_axis) row.label = a.label;
            else if (sign == 0) row.label = a.label + " (axis)";
            else row.label = a.label + ": " + (sign < 0 ? a.negative_label : a.positive_label);
            for (std::size_t i = 0; i < eff.size(); ++i) {
                if (!binding_in_row(eff[i], a, sign)) continue;
                row.bindings.push_back(static_cast<int>(i));
                for (const Conflict& c : conflicts_all) {
                    if ((c.action_a == id && c.binding_a == eff[i]) || (c.action_b == id && c.binding_b == eff[i])) row.conflict = true;
                }
            }
            out.push_back(std::move(row));
        }
    }
    return out;
}

bool Controls::set_bindings(const std::string& device_key, const std::string& action, const std::vector<Binding>& bindings,
                            std::string* err) {
    Record* r = find(device_key);
    const ActionDef* a = schema_.find(action);
    if (r == nullptr || a == nullptr) {
        if (err != nullptr) *err = r == nullptr ? "unknown device" : "unknown action";
        return false;
    }
    if (bindings.size() > kMaxBindingsPerAction) {
        if (err != nullptr) *err = "too many bindings for one action";
        return false;
    }
    std::vector<Binding> clean = bindings;
    for (Binding& b : clean) {
        if (!binding_suits(*r, *a, b, err)) return false;
        if (b.is_analogue()) b.tuning = sanitize_tuning(b.tuning);
    }
    make_custom(*r);
    const auto* def = default_list(r->cls, action);
    const std::vector<Binding> empty;
    if (clean == (def != nullptr ? *def : empty)) r->overrides.erase(action);
    else r->overrides[action] = std::move(clean);
    changed();
    return true;
}

bool Controls::bind(const std::string& device_key, const std::string& action, int sign, Binding binding, bool replace,
                    std::string* err) {
    const Record* r = find(device_key);
    const ActionDef* a = schema_.find(action);
    if (r == nullptr || a == nullptr) {
        if (err != nullptr) *err = r == nullptr ? "unknown device" : "unknown action";
        return false;
    }
    const bool signed_axis = a->kind == ActionKind::Axis && a->range == AxisRange::Signed;
    if (signed_axis && sign != 0 && !binding.is_analogue()) binding.sign = static_cast<float>(sign);
    if (signed_axis && sign != 0 && binding.is_analogue() && binding.span != AxisSpan::Full) binding.sign = static_cast<float>(sign);
    std::vector<Binding> list = effective(*r, action);
    if (replace) {
        list.erase(std::remove_if(list.begin(), list.end(), [&](const Binding& b) { return binding_in_row(b, *a, sign); }), list.end());
    }
    // The same input twice on one action is one binding.
    list.erase(std::remove_if(list.begin(), list.end(), [&](const Binding& b) { return same_input(b, binding) && b.type == binding.type && b.span == binding.span; }), list.end());
    list.push_back(std::move(binding));
    return set_bindings(device_key, action, list, err);
}

bool Controls::remove_binding(const std::string& device_key, const std::string& action, int index, std::string* err) {
    std::vector<Binding> list = effective_bindings(device_key, action);
    if (index < 0 || static_cast<std::size_t>(index) >= list.size()) {
        if (err != nullptr) *err = "no such binding";
        return false;
    }
    list.erase(list.begin() + index);
    return set_bindings(device_key, action, list, err);
}

bool Controls::replace_binding(const std::string& device_key, const std::string& action, int index, Binding binding,
                               std::string* err) {
    std::vector<Binding> list = effective_bindings(device_key, action);
    if (index < 0 || static_cast<std::size_t>(index) >= list.size()) {
        if (err != nullptr) *err = "no such binding";
        return false;
    }
    list[static_cast<std::size_t>(index)] = std::move(binding);
    return set_bindings(device_key, action, list, err);
}

bool Controls::clear_action(const std::string& device_key, const std::string& action, std::string* err) {
    return set_bindings(device_key, action, {}, err);
}

bool Controls::reset_action(const std::string& device_key, const std::string& action) {
    Record* r = find(device_key);
    if (r == nullptr || schema_.find(action) == nullptr) return false;
    make_custom(*r);
    r->overrides.erase(action);
    changed();
    return true;
}

bool Controls::reset_device(const std::string& device_key) {
    Record* r = find(device_key);
    if (r == nullptr) return false;
    r->overrides.clear();
    // A second identical pad gets an explicit empty profile: it stops inheriting the first pad's setup.
    r->customised = is_pad_class(r->cls) && r->ordinal > 1;
    changed();
    return true;
}

bool Controls::bind_combined_pedals(const std::string& device_key, int axis, AxisSpan throttle_half, const AxisTuning& tuning,
                                    std::string* err) {
    if (throttle_half == AxisSpan::Full) {
        if (err != nullptr) *err = "combined pedals need a half for the throttle";
        return false;
    }
    Binding t;
    t.type = BindingType::JoyAxis;
    t.index = axis;
    t.span = throttle_half;
    t.tuning = tuning;
    Binding b = t;
    b.span = throttle_half == AxisSpan::Positive ? AxisSpan::Negative : AxisSpan::Positive;
    if (!set_bindings(device_key, "throttle", {t}, err)) return false;
    return set_bindings(device_key, "brake", {b}, err);
}

std::vector<Conflict> Controls::conflicts_for(const std::string& device_key, const std::string& action) const {
    std::vector<Conflict> out;
    for (Conflict& c : conflicts(device_key)) {
        if (c.action_a == action || c.action_b == action) out.push_back(std::move(c));
    }
    return out;
}

std::vector<Conflict> Controls::conflicts(const std::string& device_key) const {
    std::vector<Conflict> out;
    const Record* r = find(device_key);
    if (r == nullptr) return out;
    const std::vector<std::string> ids = applicable_actions(device_key);
    std::vector<std::vector<Binding>> eff;
    eff.reserve(ids.size());
    for (const std::string& id : ids) eff.push_back(effective(*r, id));
    for (std::size_t i = 0; i < ids.size(); ++i) {
        for (std::size_t j = i + 1; j < ids.size(); ++j) {
            const ActionDef& a = *schema_.find(ids[i]);
            const ActionDef& b = *schema_.find(ids[j]);
            if ((a.modes & b.modes) == 0) continue;
            for (const Binding& ba : eff[i]) {
                for (const Binding& bb : eff[j]) {
                    if (same_input(ba, bb)) out.push_back({ids[i], ids[j], ba, bb});
                }
            }
        }
    }
    return out;
}

// ---- evaluation ------------------------------------------------------------------------------------------

ActionState Controls::empty_state() const {
    ActionState s;
    const std::size_t n = schema_.size();
    s.value.assign(n, 0.0F);
    s.raw.assign(n, 0.0F);
    s.pulses.assign(n, 0.0F);
    s.keyboard.assign(n, 0.0F);
    s.pad.assign(n, 0.0F);
    s.source.assign(n, -1);
    return s;
}

void Controls::evaluate_record(const Record& r, const PadSnapshot* pad, const InputSnapshot& in, ActionState& out) const {
    const std::vector<std::vector<Binding>>& all = effective_all(r);
    for (std::size_t i = 0; i < all.size(); ++i) {
        const ActionDef& a = schema_.actions()[i];
        const AxisRange range = target_range(a);
        float best = 0.0F;
        float best_raw = 0.0F;
        float dsum = 0.0F; // digital sources on an axis add up (A and D together = 0, like Input.get_axis)
        float dsum_raw = 0.0F;
        float pulses = 0.0F;
        const bool axis_action = a.kind == ActionKind::Axis;
        for (const Binding& b : all[i]) {
            float c = 0.0F;
            float rw = 0.0F;
            bool digital = true;
            switch (b.type) {
                case BindingType::Key:
                    if (has_key(in, b.key)) c = rw = digital_contribution(b, range);
                    break;
                case BindingType::MouseButton:
                    if (b.index == 4) pulses += in.wheel_up;
                    else if (b.index == 5) pulses += in.wheel_down;
                    else if (b.index == 6) pulses += in.wheel_left;
                    else if (b.index == 7) pulses += in.wheel_right;
                    else if (has_int(in.mouse_buttons, b.index)) c = rw = digital_contribution(b, range);
                    break;
                case BindingType::MouseMotion:
                    digital = false;
                    if (a.range == AxisRange::Delta) c = evaluate_axis_binding(b, b.index == 0 ? in.mouse_dx : in.mouse_dy, AxisRange::Delta, &rw);
                    break;
                case BindingType::JoyButton:
                    if (pad != nullptr && has_int(pad->buttons, b.index)) c = rw = digital_contribution(b, range);
                    break;
                case BindingType::JoyAxis:
                    digital = false;
                    if (pad != nullptr && b.index >= 0 && static_cast<std::size_t>(b.index) < pad->axes.size()) {
                        c = evaluate_axis_binding(b, pad->axes[static_cast<std::size_t>(b.index)], range, &rw);
                    }
                    break;
            }
            if (!axis_action) {
                if (std::fabs(c) > 0.5F) best = best_raw = 1.0F;
            } else if (digital) {
                dsum += c;
                dsum_raw += rw;
            } else {
                if (std::fabs(c) > std::fabs(best)) best = c;
                if (std::fabs(rw) > std::fabs(best_raw)) best_raw = rw;
            }
        }
        if (axis_action) {
            dsum = std::clamp(dsum, -1.0F, 1.0F);
            dsum_raw = std::clamp(dsum_raw, -1.0F, 1.0F);
            if (std::fabs(dsum) > std::fabs(best)) best = dsum;
            if (std::fabs(dsum_raw) > std::fabs(best_raw)) best_raw = dsum_raw;
        }
        out.value[i] = best;
        out.raw[i] = best_raw;
        out.pulses[i] = pulses;
        if (best != 0.0F || pulses != 0.0F) out.source[i] = static_cast<int>(r.cls);
    }
}

namespace {

void merge_into(ActionState& total, const ActionState& one, DeviceClass cls) {
    for (std::size_t i = 0; i < total.value.size(); ++i) {
        if (std::fabs(one.value[i]) > std::fabs(total.value[i])) {
            total.value[i] = one.value[i];
            total.source[i] = static_cast<int>(cls);
        }
        if (std::fabs(one.raw[i]) > std::fabs(total.raw[i])) total.raw[i] = one.raw[i];
        total.pulses[i] += one.pulses[i];
        if (cls == DeviceClass::Keyboard) {
            total.keyboard[i] = one.value[i];
        } else if (is_pad_class(cls)) {
            if (std::fabs(one.value[i]) > std::fabs(total.pad[i])) total.pad[i] = one.value[i];
        }
    }
}

} // namespace

ActionState Controls::evaluate(const InputSnapshot& in) const {
    ActionState total = empty_state();
    for (const Record& r : records_) {
        if (!r.connected) continue;
        const PadSnapshot* pad = nullptr;
        if (is_pad_class(r.cls)) {
            for (const PadSnapshot& p : in.pads) {
                if (p.slot == r.slot) pad = &p;
            }
            if (pad == nullptr) continue;
        }
        ActionState one = empty_state();
        evaluate_record(r, pad, in, one);
        merge_into(total, one, r.cls);
    }
    return total;
}

ActionState Controls::evaluate_device(const std::string& device_key, const InputSnapshot& in) const {
    ActionState out = empty_state();
    const Record* r = find(device_key);
    if (r == nullptr) return out;
    const PadSnapshot* pad = nullptr;
    if (is_pad_class(r->cls)) {
        for (const PadSnapshot& p : in.pads) {
            if (p.slot == r->slot) pad = &p;
        }
        if (pad == nullptr && !in.pads.empty()) pad = &in.pads.front(); // the monitor feeds exactly the selected pad
    }
    evaluate_record(*r, pad, in, out);
    return out;
}

std::vector<std::string> Controls::polled_keys() const {
    std::set<std::string> keys;
    for (const std::vector<Binding>& list : effective_all(records_[0])) {
        for (const Binding& b : list) {
            if (b.type == BindingType::Key) keys.insert(b.key);
        }
    }
    return {keys.begin(), keys.end()};
}

bool Controls::mouse_button_bound(const std::string& action, int button) const {
    for (const Binding& b : effective(records_[1], action)) {
        if (b.type == BindingType::MouseButton && b.index == button) return true;
    }
    return false;
}

std::vector<Binding> Controls::menu_bindings(const std::string& action) const {
    std::vector<Binding> out;
    const ActionDef* a = schema_.find(action);
    if (a == nullptr) return out;
    const auto add = [&](const Binding& b) {
        if (b.type != BindingType::Key && b.type != BindingType::JoyButton) return;
        for (const Binding& have : out) {
            if (have.type == b.type && have.key == b.key && have.index == b.index) return;
        }
        out.push_back(b);
    };
    for (const Binding& b : a->always) add(b);
    for (const Record& r : records_) {
        if (!r.connected && is_pad_class(r.cls)) continue;
        if (r.cls == DeviceClass::Mouse) continue;
        for (const Binding& b : effective(r, action)) add(b);
    }
    return out;
}

} // namespace rg
