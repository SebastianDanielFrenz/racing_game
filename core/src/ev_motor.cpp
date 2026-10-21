// rg/ev_motor.cpp - see ev_motor.h.
#include "rg/ev_motor.h"

#include "ev_motor_json.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <map>
#include <set>

namespace rg {

namespace {

using detail::json;

bool is_lower_snake(const std::string& s) {
    if (s.empty()) return false;
    for (const char c : s) {
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
    }
    return true;
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool parse_drive(const std::string& s, EvAxleDrive& out) {
    if (s == "none") out = EvAxleDrive::None;
    else if (s == "axle") out = EvAxleDrive::Axle;
    else if (s == "wheel") out = EvAxleDrive::Wheel;
    else return false;
    return true;
}

int drive_motors(EvAxleDrive d) { return d == EvAxleDrive::Axle ? 1 : d == EvAxleDrive::Wheel ? 2 : 0; }

struct WheelInfo {
    std::string name;
    bool front = false;
    bool left = false;
};

// One drive unit of the powertrain graph.
struct Unit {
    std::string motor;                  // component id
    std::string reduction;              // differential (axle) or gear_pair (wheel) id
    bool axle = false;
    bool front = false;                 // the axle its wheels belong to
    std::vector<std::string> wheels;    // wheel names (2 for an axle unit, 1 for a wheel unit)
    bool left = false;                  // wheel unit: the wheel is on the left
    std::vector<std::size_t> shafts;    // shafts that belong to the unit (removed by a layout swap)
};

struct Topology {
    bool ok = false;
    std::string reason;
    std::vector<WheelInfo> wheels;
    std::vector<Unit> units;
    EvAxleDrive front = EvAxleDrive::None;
    EvAxleDrive rear = EvAxleDrive::None;
};

const WheelInfo* wheel_named(const Topology& t, const std::string& name) {
    for (const WheelInfo& w : t.wheels) if (w.name == name) return &w;
    return nullptr;
}

std::string owner_of(const std::string& port) { return port.substr(0, port.find('.')); }
std::string port_of(const std::string& port) {
    const auto dot = port.find('.');
    return dot == std::string::npos ? std::string() : port.substr(dot + 1);
}

std::vector<std::string> shaft_ports(const json& shaft) {
    std::vector<std::string> out;
    const auto ps = shaft.is_object() ? shaft.find("ports") : shaft.end();
    if (ps != shaft.end() && ps->is_array()) {
        for (const json& p : *ps) if (p.is_string()) out.push_back(p.get<std::string>());
    }
    return out;
}

Topology read_topology(const json& vehicle) {
    Topology t;
    auto fail = [&](const std::string& m) {
        t.ok = false;
        t.reason = m;
        return t;
    };
    const auto wheels = vehicle.find("wheels");
    if (wheels == vehicle.end() || !wheels->is_array()) return fail("the vehicle has no wheels");
    for (const json& w : *wheels) {
        if (!w.is_object()) continue;
        WheelInfo info;
        detail::get_string(w, "name", info.name);
        detail::get_bool(w, "is_front", info.front);
        const auto at = w.find("attachment_local");
        if (at == w.end() || !at->is_array() || at->size() != 3 || !(*at)[1].is_number()) return fail("wheel " + info.name + " has no attachment_local");
        const double y = (*at)[1].get<double>();
        if (y == 0.0) return fail("wheel " + info.name + " sits on the centre line (no left/right side)");
        info.left = y > 0.0;
        t.wheels.push_back(info);
    }
    const auto pt = vehicle.find("powertrain");
    if (pt == vehicle.end() || !pt->is_object()) return fail("the vehicle has no powertrain");
    const auto comps = pt->find("components");
    const auto shafts = pt->find("shafts");
    if (comps == pt->end() || !comps->is_array() || shafts == pt->end() || !shafts->is_array()) return fail("the powertrain has no components or shafts");
    std::map<std::string, std::string> type_of;
    std::vector<std::string> motors;
    for (const json& c : *comps) {
        std::string id, ty;
        if (!c.is_object() || !detail::get_string(c, "id", id) || !detail::get_string(c, "type", ty)) continue;
        type_of[id] = ty;
        if (ty == "engine") return fail("it has a combustion engine");
        if (ty == "fuel_cell") return fail("it has a fuel cell");
        if (ty == "electric_motor") motors.push_back(id);
        else if (ty != "battery" && ty != "differential" && ty != "gear_pair") return fail("it has a " + ty + " (" + id + ")");
    }
    if (motors.empty()) return fail("it has no electric motor");
    // Which shafts hold a port.
    auto shafts_with = [&](const std::string& port) {
        std::vector<std::size_t> out;
        for (std::size_t i = 0; i < shafts->size(); ++i) {
            const auto ports = shaft_ports((*shafts)[i]);
            if (std::find(ports.begin(), ports.end(), port) != ports.end()) out.push_back(i);
        }
        return out;
    };
    // The one other end of a two-port shaft holding `port`; false when it is anything else.
    auto other_end = [&](const std::string& port, std::string& other, Unit& u) {
        const auto idx = shafts_with(port);
        if (idx.size() != 1) return false;
        const auto ports = shaft_ports((*shafts)[idx.front()]);
        if (ports.size() != 2) return false;
        other = ports[0] == port ? ports[1] : ports[0];
        u.shafts.push_back(idx.front());
        return true;
    };
    std::set<std::string> claimed_wheels, claimed_reductions;
    for (const std::string& motor : motors) {
        Unit u;
        u.motor = motor;
        std::string peer;
        if (!other_end(motor + ".shaft", peer, u)) return fail("motor " + motor + " does not sit alone on a shaft with one reduction");
        const std::string red = owner_of(peer), pport = port_of(peer);
        const auto ty = type_of.find(red);
        if (ty == type_of.end() || !claimed_reductions.insert(red).second) return fail("motor " + motor + " is not fed into its own reduction");
        u.reduction = red;
        if (ty->second == "differential" && pport == "carrier") {
            u.axle = true;
            std::string l, r;
            if (!other_end(red + ".left", l, u) || !other_end(red + ".right", r, u)) return fail("differential " + red + " is not connected left and right");
            if (l.rfind("wheel:", 0) != 0 || r.rfind("wheel:", 0) != 0) return fail("differential " + red + " does not drive two wheels directly");
            const WheelInfo* wl = wheel_named(t, l.substr(6));
            const WheelInfo* wr = wheel_named(t, r.substr(6));
            if (wl == nullptr || wr == nullptr || wl == wr || wl->front != wr->front || wl->left == wr->left) {
                return fail("differential " + red + " does not drive the left and right wheel of one axle");
            }
            u.front = wl->front;
            u.wheels = {wl->name, wr->name};
        } else if (ty->second == "gear_pair" && pport == "in") {
            std::string w;
            if (!other_end(red + ".out", w, u)) return fail("gear pair " + red + " is not connected to one wheel");
            if (w.rfind("wheel:", 0) != 0) return fail("gear pair " + red + " does not drive a wheel directly");
            const WheelInfo* wi = wheel_named(t, w.substr(6));
            if (wi == nullptr) return fail("gear pair " + red + " drives an unknown wheel");
            u.front = wi->front;
            u.left = wi->left;
            u.wheels = {wi->name};
        } else {
            return fail("motor " + motor + " is not fed into a differential carrier or a gear pair input");
        }
        for (const std::string& w : u.wheels) {
            if (!claimed_wheels.insert(w).second) return fail("wheel " + w + " is driven twice");
        }
        t.units.push_back(std::move(u));
    }
    for (const auto& [id, ty] : type_of) {
        if ((ty == "differential" || ty == "gear_pair") && claimed_reductions.count(id) == 0) return fail("it has another " + ty + " (" + id + ")");
    }
    // Every shaft that holds a wheel must belong to a unit.
    std::set<std::size_t> unit_shafts;
    for (const Unit& u : t.units) unit_shafts.insert(u.shafts.begin(), u.shafts.end());
    for (std::size_t i = 0; i < shafts->size(); ++i) {
        if (unit_shafts.count(i) != 0) continue;
        for (const std::string& p : shaft_ports((*shafts)[i])) {
            if (p.rfind("wheel:", 0) == 0) return fail("wheel " + p.substr(6) + " is on a shaft the swap does not understand");
            const std::string owner = owner_of(p);
            const auto ty = type_of.find(owner);
            if (ty != type_of.end() && (ty->second == "differential" || ty->second == "gear_pair" || ty->second == "electric_motor")) {
                return fail("a shaft joins " + p + " to something the swap does not understand");
            }
        }
    }
    for (const bool front : {true, false}) {
        int axle_units = 0, left_units = 0, right_units = 0;
        for (const Unit& u : t.units) {
            if (u.front != front) continue;
            if (u.axle) ++axle_units;
            else (u.left ? left_units : right_units) += 1;
        }
        EvAxleDrive d = EvAxleDrive::None;
        if (axle_units == 1 && left_units == 0 && right_units == 0) d = EvAxleDrive::Axle;
        else if (axle_units == 0 && left_units == 1 && right_units == 1) d = EvAxleDrive::Wheel;
        else if (axle_units != 0 || left_units != 0 || right_units != 0) {
            return fail(std::string("the ") + (front ? "front" : "rear") + " axle is driven in a way the swap does not understand");
        }
        (front ? t.front : t.rear) = d;
    }
    t.ok = true;
    return t;
}

int component_index(const json& vehicle, const std::string& id) {
    const json& comps = vehicle["powertrain"]["components"];
    for (std::size_t i = 0; i < comps.size(); ++i) {
        std::string cid;
        if (comps[i].is_object() && detail::get_string(comps[i], "id", cid) && cid == id) return static_cast<int>(i);
    }
    return -1;
}

json shaft_of(const std::string& a, const std::string& b) {
    json s = json::object();
    s["ports"] = json::array({a, b});
    return s;
}

// The reduction of a unit as numbers, whichever kind of component holds it.
struct Reduction {
    double ratio = 1.0;
    double efficiency = 1.0;
    double inertia = 0.0;
    json axis = json::array({0, 1, 0});
    json source;       // the differential/gear_pair component itself
    bool is_axle = false;
};

Reduction reduction_of(const json& comp) {
    Reduction r;
    r.source = comp;
    std::string type;
    detail::get_string(comp, "type", type);
    r.is_axle = type == "differential";
    detail::get_number(comp, "ratio", r.ratio);
    detail::get_number(comp, "efficiency", r.efficiency);
    detail::get_number(comp, r.is_axle ? "carrier_inertia_kgm2" : "inertia_in_kgm2", r.inertia);
    const auto axis = comp.find(r.is_axle ? "carrier_axis_local" : "in_axis_local");
    if (axis != comp.end() && axis->is_array() && axis->size() == 3) r.axis = *axis;
    return r;
}

json make_axle_diff(const Reduction& r, const std::string& id) {
    if (r.is_axle) {
        json d = r.source;
        d["id"] = id;
        return d;
    }
    json d = json::object();
    d["id"] = id;
    d["type"] = "differential";
    d["kind"] = "open";
    d["ratio"] = r.ratio;
    d["efficiency"] = r.efficiency;
    d["carrier_inertia_kgm2"] = r.inertia;
    d["carrier_axis_local"] = r.axis;
    return d;
}

json make_wheel_gear(const Reduction& r, const std::string& id) {
    if (!r.is_axle) {
        json g = r.source;
        g["id"] = id;
        return g;
    }
    json g = json::object();
    g["id"] = id;
    g["type"] = "gear_pair";
    g["ratio"] = r.ratio;
    g["efficiency"] = r.efficiency;
    g["inertia_in_kgm2"] = r.inertia;
    g["in_axis_local"] = r.axis;
    return g;
}

// A hybrid's traction motors: a combustion engine plus electric motors that are no crank generator (crank_assist).
struct HybridUnit {
    std::string motor;
    std::string reduction;     // the gear pair / differential the motor feeds alone on its shaft, "" = none
    double downstream = 1.0;   // the final drive (differential carrier / gear pair input ratio) after that reduction
};

struct Hybrid {
    bool ok = false;
    std::string reason;
    std::vector<HybridUnit> units;
};

Hybrid read_hybrid(const json& vehicle) {
    Hybrid h;
    const auto pt = vehicle.find("powertrain");
    if (pt == vehicle.end() || !pt->is_object()) { h.reason = "the vehicle has no powertrain"; return h; }
    const auto comps = pt->find("components");
    const auto shafts = pt->find("shafts");
    if (comps == pt->end() || !comps->is_array() || shafts == pt->end() || !shafts->is_array()) {
        h.reason = "the powertrain has no components or shafts";
        return h;
    }
    std::map<std::string, const json*> by_id;
    std::map<std::string, std::string> type_of;
    bool engine = false;
    std::vector<std::string> motors;
    for (const json& c : *comps) {
        std::string id, ty;
        if (!c.is_object() || !detail::get_string(c, "id", id) || !detail::get_string(c, "type", ty)) continue;
        by_id[id] = &c;
        type_of[id] = ty;
        if (ty == "engine") engine = true;
        if (ty == "electric_motor") {
            bool assist = false;
            detail::get_bool(c, "crank_assist", assist);
            if (!assist) motors.push_back(id);
        }
    }
    if (!engine) { h.reason = "it has no combustion engine"; return h; }
    if (motors.empty()) { h.reason = "it has no traction motor"; return h; }
    auto ports_holding = [&](const std::string& port) {
        std::vector<std::string> out;
        for (const json& s : *shafts) {
            const auto ps = shaft_ports(s);
            if (std::find(ps.begin(), ps.end(), port) != ps.end()) out = ps;
        }
        return out;
    };
    for (const std::string& m : motors) {
        HybridUnit u;
        u.motor = m;
        const auto ps = ports_holding(m + ".shaft");
        if (ps.size() == 2) {
            const std::string peer = ps[0] == m + ".shaft" ? ps[1] : ps[0];
            const auto ty = type_of.find(owner_of(peer));
            if (ty != type_of.end() && ty->second == "gear_pair" && port_of(peer) == "in") {
                u.reduction = owner_of(peer);
                for (const std::string& p : ports_holding(u.reduction + ".out")) {
                    const auto t2 = type_of.find(owner_of(p));
                    if (t2 == type_of.end() || owner_of(p) == u.reduction) continue;
                    const bool diff = t2->second == "differential" && port_of(p) == "carrier";
                    const bool gear = t2->second == "gear_pair" && port_of(p) == "in";
                    if (!diff && !gear) continue;
                    double r = 0.0;
                    detail::get_number(*by_id[owner_of(p)], "ratio", r);
                    if (r > 0.0) { u.downstream = r; break; }
                }
            }
        }
        h.units.push_back(std::move(u));
    }
    h.ok = true;
    return h;
}

} // namespace

int EvLayoutEntry::motor_count() const { return drive_motors(front) + drive_motors(rear); }

// ---- catalogue ------------------------------------------------------------------------------------

std::unique_ptr<EvMotorCatalog> EvMotorCatalog::open(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        if (err != nullptr) *err = path + ": cannot open";
        return nullptr;
    }
    return parse(*text, path, err);
}

std::unique_ptr<EvMotorCatalog> EvMotorCatalog::parse(const std::string& json_text, const std::string& origin, std::string* err) {
    auto bad = [&](const std::string& m) {
        if (err != nullptr) *err = origin + ": " + m;
        return std::unique_ptr<EvMotorCatalog>{};
    };
    const auto doc = detail::parse_json(json_text);
    if (!doc || !doc->is_object()) return bad("not a JSON object");
    std::string format;
    if (!detail::get_string(*doc, "format", format) || format != "rg.ev_motor_catalog/1") return bad("\"format\" must be \"rg.ev_motor_catalog/1\"");
    auto cat = std::make_unique<EvMotorCatalog>();
    const auto layouts = doc->find("layouts");
    if (layouts == doc->end() || !layouts->is_array() || layouts->empty()) return bad("\"layouts\" must be a non-empty array");
    std::set<std::string> ids;
    std::set<std::pair<int, int>> shapes;
    for (std::size_t i = 0; i < layouts->size(); ++i) {
        const json& l = (*layouts)[i];
        const std::string where = "layouts[" + std::to_string(i) + "]";
        EvLayoutEntry e;
        std::string f, r;
        if (!l.is_object() || !detail::get_string(l, "id", e.id) || !is_lower_snake(e.id) || !ids.insert(e.id).second) {
            return bad(where + ": needs a unique lower_snake \"id\"");
        }
        if (!detail::get_string(l, "name", e.name) || e.name.empty()) return bad(where + ": needs a \"name\"");
        if (!detail::get_string(l, "front", f) || !parse_drive(f, e.front) || !detail::get_string(l, "rear", r) || !parse_drive(r, e.rear)) {
            return bad(where + ": \"front\" and \"rear\" must each be none, axle or wheel");
        }
        if (e.front == EvAxleDrive::None && e.rear == EvAxleDrive::None) return bad(where + ": a layout drives at least one axle");
        if (!shapes.insert({static_cast<int>(e.front), static_cast<int>(e.rear)}).second) return bad(where + ": another layout drives the axles the same way");
        detail::get_string(l, "description", e.description);
        cat->layouts_.push_back(std::move(e));
    }
    const auto units = doc->find("units");
    if (units == doc->end() || !units->is_array() || units->empty()) return bad("\"units\" must be a non-empty array");
    ids.clear();
    for (std::size_t i = 0; i < units->size(); ++i) {
        const json& u = (*units)[i];
        const std::string where = "units[" + std::to_string(i) + "]";
        EvMotorUnit m;
        if (!u.is_object() || !detail::get_string(u, "id", m.id) || !is_lower_snake(m.id) || !ids.insert(m.id).second) {
            return bad(where + ": needs a unique lower_snake \"id\"");
        }
        if (!detail::get_string(u, "label", m.label) || m.label.empty()) return bad(where + ": needs a \"label\"");
        if (!detail::get_string(u, "inspiration", m.inspiration) || m.inspiration.empty()) return bad(where + ": needs an \"inspiration\"");
        if (!detail::get_string(u, "source", m.source) || m.source.empty()) return bad(where + ": needs a \"source\"");
        const struct { const char* key; double* v; } required[] = {
            {"peak_torque_nm", &m.peak_torque_nm}, {"max_power_w", &m.max_power_w}, {"max_rpm", &m.max_rpm},
            {"inertia_kgm2", &m.inertia_kgm2},     {"efficiency", &m.efficiency},   {"regen_torque_nm", &m.regen_torque_nm},
            {"mass_kg", &m.mass_kg},               {"reduction_ratio", &m.reduction_ratio},
        };
        for (const auto& f : required) {
            if (!detail::get_number(u, f.key, *f.v) || !(*f.v > 0.0)) return bad(where + ": \"" + f.key + "\" must be a number > 0");
        }
        if (m.efficiency > 1.0) return bad(where + ": \"efficiency\" must be in (0, 1]");
        if (u.contains("reduction_efficiency") &&
            (!detail::get_number(u, "reduction_efficiency", m.reduction_efficiency) || !(m.reduction_efficiency > 0.0 && m.reduction_efficiency <= 1.0))) {
            return bad(where + ": \"reduction_efficiency\" must be in (0, 1]");
        }
        cat->units_.push_back(std::move(m));
    }
    return cat;
}

const EvMotorUnit* EvMotorCatalog::find_unit(const std::string& id) const {
    for (const auto& u : units_) if (u.id == id) return &u;
    return nullptr;
}
const EvLayoutEntry* EvMotorCatalog::find_layout(const std::string& id) const {
    for (const auto& l : layouts_) if (l.id == id) return &l;
    return nullptr;
}
const EvLayoutEntry* EvMotorCatalog::layout_of(EvAxleDrive front, EvAxleDrive rear) const {
    for (const auto& l : layouts_) if (l.front == front && l.rear == rear) return &l;
    return nullptr;
}

// ---- the transform -------------------------------------------------------------------------------

namespace detail {

EvMotorInfo describe_ev_motors_json(const json& vehicle, const EvMotorCatalog* catalog) {
    EvMotorInfo info;
    const Topology t = read_topology(vehicle);
    info.swappable = t.ok;
    info.reason = t.reason;
    if (!t.ok) {
        const Hybrid h = read_hybrid(vehicle);
        if (h.ok) {
            info.hybrid = true;
            info.traction_motors = static_cast<int>(h.units.size());
            info.reason = "it is a hybrid: only the drive unit of its traction motor changes, not the motor layout";
        }
        return info;
    }
    info.front = t.front;
    info.rear = t.rear;
    info.motor_count = static_cast<int>(t.units.size());
    for (const Unit& u : t.units) info.driven_wheels += static_cast<int>(u.wheels.size());
    if (catalog != nullptr) {
        if (const EvLayoutEntry* l = catalog->layout_of(t.front, t.rear)) info.layout_id = l->id;
    }
    return info;
}

std::string apply_ev_layout_json(json& vehicle, const EvLayoutEntry& layout) {
    const Topology t = read_topology(vehicle);
    if (!t.ok) return "this car's motors cannot be rearranged: " + t.reason;
    if (t.front == layout.front && t.rear == layout.rear) return {};
    const json& comps = vehicle["powertrain"]["components"];
    auto comp = [&](const std::string& id) -> const json& { return comps[static_cast<std::size_t>(component_index(vehicle, id))]; };
    // Templates: the car's own unit of that axle, else the other axle's.
    auto unit_of_axle = [&](bool front) -> const Unit* {
        for (const Unit& u : t.units) if (u.front == front) return &u;
        return nullptr;
    };
    auto template_unit = [&](bool front) -> const Unit* {
        const Unit* own = unit_of_axle(front);
        return own != nullptr ? own : unit_of_axle(!front);
    };
    std::vector<json> fresh;
    std::vector<json> fresh_shafts;
    for (const bool front : {true, false}) {
        const EvAxleDrive drive = front ? layout.front : layout.rear;
        if (drive == EvAxleDrive::None) continue;
        const Unit* tu = template_unit(front);
        if (tu == nullptr) return "this car has no motor to copy";
        const json motor_tpl = comp(tu->motor);
        const Reduction red = reduction_of(comp(tu->reduction));
        const std::string axle_name = front ? "front" : "rear";
        if (drive == EvAxleDrive::Axle) {
            json m = motor_tpl;
            m["id"] = axle_name + "_motor";
            fresh.push_back(std::move(m));
            fresh.push_back(make_axle_diff(red, axle_name + "_diff"));
            std::string l, r;
            for (const WheelInfo& w : t.wheels) {
                if (w.front != front) continue;
                (w.left ? l : r) = "wheel:" + w.name;
            }
            fresh_shafts.push_back(shaft_of(axle_name + "_motor.shaft", axle_name + "_diff.carrier"));
            fresh_shafts.push_back(shaft_of(axle_name + "_diff.left", l));
            fresh_shafts.push_back(shaft_of(axle_name + "_diff.right", r));
        } else {
            for (const bool left : {true, false}) {
                for (const WheelInfo& w : t.wheels) {
                    if (w.front != front || w.left != left) continue;
                    const std::string n = lower(w.name);
                    json m = motor_tpl;
                    m["id"] = "motor_" + n;
                    fresh.push_back(std::move(m));
                    fresh.push_back(make_wheel_gear(red, "reduction_" + n));
                    fresh_shafts.push_back(shaft_of("motor_" + n + ".shaft", "reduction_" + n + ".in"));
                    fresh_shafts.push_back(shaft_of("reduction_" + n + ".out", "wheel:" + w.name));
                }
            }
        }
    }
    std::set<std::string> old_ids;
    std::set<std::size_t> old_shafts;
    for (const Unit& u : t.units) {
        old_ids.insert(u.motor);
        old_ids.insert(u.reduction);
        old_shafts.insert(u.shafts.begin(), u.shafts.end());
    }
    // A new id must not clash with a component that stays.
    for (const json& c : comps) {
        std::string id;
        if (!c.is_object() || !detail::get_string(c, "id", id) || old_ids.count(id) != 0) continue;
        for (const json& f : fresh) {
            if (f["id"].get<std::string>() == id) return "this car's motors cannot be rearranged: a component is already called \"" + id + "\"";
        }
    }
    json new_comps = json::array();
    bool placed = false;
    for (const json& c : comps) {
        std::string id;
        if (c.is_object() && detail::get_string(c, "id", id) && old_ids.count(id) != 0) {
            if (!placed) {
                for (const json& f : fresh) new_comps.push_back(f);
                placed = true;
            }
            continue;
        }
        new_comps.push_back(c);
    }
    const json& shafts = vehicle["powertrain"]["shafts"];
    json new_shafts = json::array();
    placed = false;
    for (std::size_t i = 0; i < shafts.size(); ++i) {
        if (old_shafts.count(i) != 0) {
            if (!placed) {
                for (const json& s : fresh_shafts) new_shafts.push_back(s);
                placed = true;
            }
            continue;
        }
        new_shafts.push_back(shafts[i]);
    }
    if (!placed) for (const json& s : fresh_shafts) new_shafts.push_back(s);
    vehicle["powertrain"]["components"] = std::move(new_comps);
    vehicle["powertrain"]["shafts"] = std::move(new_shafts);
    return {};
}

std::optional<EvMotorUnit> ev_axle_unit_json(const json& vehicle, bool front) {
    const Topology t = read_topology(vehicle);
    if (!t.ok) return std::nullopt;
    const Unit* pick = nullptr;
    for (const Unit& u : t.units) if (u.front == front && pick == nullptr) pick = &u;
    for (const Unit& u : t.units) if (pick == nullptr) pick = &u;
    if (pick == nullptr) return std::nullopt;
    const json& comps = vehicle["powertrain"]["components"];
    const json& m = comps[static_cast<std::size_t>(component_index(vehicle, pick->motor))];
    const json& r = comps[static_cast<std::size_t>(component_index(vehicle, pick->reduction))];
    EvMotorUnit out;
    get_number(m, "peak_torque_nm", out.peak_torque_nm);
    get_number(m, "max_power_w", out.max_power_w);
    get_number(m, "max_rpm", out.max_rpm);
    get_number(m, "inertia_kgm2", out.inertia_kgm2);
    get_number(m, "efficiency", out.efficiency);
    get_number(m, "regen_torque_nm", out.regen_torque_nm);
    get_number(m, "mass_kg", out.mass_kg);
    get_number(r, "ratio", out.reduction_ratio);
    get_number(r, "efficiency", out.reduction_efficiency);
    return out;
}

std::string apply_ev_unit_json(json& vehicle, bool front, const EvMotorUnit& unit, std::vector<int>* touched) {
    const Topology t = read_topology(vehicle);
    if (!t.ok) return "this car's motors cannot be changed: " + t.reason;
    bool any = false;
    for (const Unit& u : t.units) {
        if (u.front != front) continue;
        any = true;
        const int mi = component_index(vehicle, u.motor);
        const int ri = component_index(vehicle, u.reduction);
        json& motor = vehicle["powertrain"]["components"][static_cast<std::size_t>(mi)];
        motor["peak_torque_nm"] = unit.peak_torque_nm;
        motor["max_power_w"] = unit.max_power_w;
        motor["max_rpm"] = unit.max_rpm;
        motor["inertia_kgm2"] = unit.inertia_kgm2;
        motor["efficiency"] = unit.efficiency;
        motor["regen_torque_nm"] = unit.regen_torque_nm;
        motor["mass_kg"] = unit.mass_kg;
        json& red = vehicle["powertrain"]["components"][static_cast<std::size_t>(ri)];
        red["ratio"] = unit.reduction_ratio;
        red["efficiency"] = unit.reduction_efficiency;
        if (touched != nullptr) {
            touched->push_back(mi);
            touched->push_back(ri);
        }
    }
    if (!any) return std::string("the ") + (front ? "front" : "rear") + " axle is not driven";
    return {};
}

std::string apply_ev_hybrid_unit_json(json& vehicle, const EvMotorUnit& unit, std::vector<int>* touched) {
    const Hybrid h = read_hybrid(vehicle);
    if (!h.ok) return "this car's traction motor cannot be changed: " + h.reason;
    double added_power_w = 0.0;  // the traction power the unit adds over the car's own motors
    for (const HybridUnit& u : h.units) {
        const int mi = component_index(vehicle, u.motor);
        json& motor = vehicle["powertrain"]["components"][static_cast<std::size_t>(mi)];
        double old_power = 0.0;
        get_number(motor, "max_power_w", old_power);
        added_power_w += std::max(0.0, unit.max_power_w - old_power);
        motor["peak_torque_nm"] = unit.peak_torque_nm;
        motor["max_power_w"] = unit.max_power_w;
        motor["max_rpm"] = unit.max_rpm;
        motor["inertia_kgm2"] = unit.inertia_kgm2;
        motor["efficiency"] = unit.efficiency;
        motor["regen_torque_nm"] = unit.regen_torque_nm;
        motor["mass_kg"] = unit.mass_kg;
        if (touched != nullptr) touched->push_back(mi);
        if (u.reduction.empty()) continue;
        const int ri = component_index(vehicle, u.reduction);
        json& red = vehicle["powertrain"]["components"][static_cast<std::size_t>(ri)];
        red["ratio"] = unit.reduction_ratio / u.downstream;
        red["efficiency"] = unit.reduction_efficiency;
        if (touched != nullptr) touched->push_back(ri);
    }
    // The hybrid supervisor asks the combustion engine for the part of the full-pedal request (requested_power_w, an authored
    // rating) the motors cannot supply. A stronger motor would reach the old rating and switch the engine off at full pedal, so
    // the rating grows with the motor power the unit adds (the car's own headroom for the engine stays).
    if (added_power_w > 0.0) {
        json& comps = vehicle["powertrain"]["components"];
        for (std::size_t i = 0; i < comps.size(); ++i) {
            std::string ty;
            if (!comps[i].is_object() || !detail::get_string(comps[i], "type", ty) || ty != "hybrid_control") continue;
            double requested = 0.0;
            if (!get_number(comps[i], "requested_power_w", requested)) continue;
            comps[i]["requested_power_w"] = requested + added_power_w;
            if (touched != nullptr) touched->push_back(static_cast<int>(i));
        }
    }
    return {};
}

std::optional<EvMotorUnit> ev_hybrid_unit_json(const json& vehicle) {
    const Hybrid h = read_hybrid(vehicle);
    if (!h.ok || h.units.empty()) return std::nullopt;
    const HybridUnit& u = h.units.front();
    const json& comps = vehicle["powertrain"]["components"];
    const json& m = comps[static_cast<std::size_t>(component_index(vehicle, u.motor))];
    EvMotorUnit out;
    get_number(m, "peak_torque_nm", out.peak_torque_nm);
    get_number(m, "max_power_w", out.max_power_w);
    get_number(m, "max_rpm", out.max_rpm);
    get_number(m, "inertia_kgm2", out.inertia_kgm2);
    get_number(m, "efficiency", out.efficiency);
    get_number(m, "regen_torque_nm", out.regen_torque_nm);
    get_number(m, "mass_kg", out.mass_kg);
    if (!u.reduction.empty()) {
        const json& r = comps[static_cast<std::size_t>(component_index(vehicle, u.reduction))];
        double ratio = 0.0;
        get_number(r, "ratio", ratio);
        out.reduction_ratio = ratio * u.downstream;
        get_number(r, "efficiency", out.reduction_efficiency);
    }
    return out;
}

} // namespace detail

EvMotorInfo describe_ev_motors(const std::string& vehicle_json, const EvMotorCatalog* catalog) {
    const auto doc = detail::parse_json(vehicle_json);
    if (!doc || !doc->is_object()) {
        EvMotorInfo info;
        info.reason = "not a JSON object";
        return info;
    }
    return detail::describe_ev_motors_json(*doc, catalog);
}

std::string apply_ev_motors(const std::string& vehicle_json, const EvMotorCatalog& catalog, const EvMotorChoice& choice,
                            std::string* out_json) {
    auto doc = detail::parse_json(vehicle_json);
    if (!doc || !doc->is_object()) return "the vehicle file is not a JSON object";
    if (!choice.layout_id.empty()) {
        const EvLayoutEntry* l = catalog.find_layout(choice.layout_id);
        if (l == nullptr) return "\"" + choice.layout_id + "\" is not a motor layout";
        const std::string e = detail::apply_ev_layout_json(*doc, *l);
        if (!e.empty()) return e;
    }
    for (const bool front : {true, false}) {
        const std::string& id = front ? choice.front_unit_id : choice.rear_unit_id;
        if (id.empty()) continue;
        const EvMotorUnit* u = catalog.find_unit(id);
        if (u == nullptr) return "\"" + id + "\" is not a motor unit";
        const std::string e = detail::apply_ev_unit_json(*doc, front, *u);
        if (!e.empty()) return e;
    }
    if (out_json != nullptr) *out_json = doc->dump();
    return {};
}

std::string ev_unit_figures(const EvMotorUnit& unit) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%.0f kW \xc2\xb7 %.0f N*m \xc2\xb7 %.0f rpm \xc2\xb7 %.0f kg", unit.max_power_w / 1000.0, unit.peak_torque_nm,
                  unit.max_rpm, unit.mass_kg);
    return buf;
}

} // namespace rg
