// rg/driveline_swap.cpp - see driveline_swap.h.
#include "rg/driveline_swap.h"

#include "driveline_swap_json.h"

#include <algorithm>
#include <map>
#include <set>

namespace rg {

namespace {

using detail::json;

// Every vehicle-file key that belongs to a differential's kind (physics_sim's loader accepts each only on its own kind).
const char* const kKindKeys[] = {"kind", "preload_nm", "ramp", "static_factor", "torque_bias_ratio", "viscous_nm_s_rad"};

bool is_lower_snake(const std::string& s) {
    if (s.empty()) return false;
    for (const char c : s) {
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
    }
    return true;
}

bool known_kind(const std::string& k) {
    return k == "open" || k == "clutch_lsd" || k == "torsen" || k == "viscous" || k == "locked";
}

// Validates one unit's kind parameters (`o` holds "kind" and the parameters) and returns {"kind":..., params}.
std::optional<json> unit_params(const json& o, const std::string& where, std::string* err) {
    auto fail = [&](const std::string& m) {
        if (err != nullptr) *err = where + ": " + m;
        return std::optional<json>{};
    };
    std::string kind;
    if (!detail::get_string(o, "kind", kind) || !known_kind(kind)) {
        return fail("\"kind\" must be one of open, clutch_lsd, torsen, viscous, locked");
    }
    json p = json::object();
    p["kind"] = kind;
    for (const char* key : kKindKeys) {
        const std::string k(key);
        if (k == "kind" || !o.contains(k)) continue;
        const bool allowed = (kind == "clutch_lsd" && (k == "preload_nm" || k == "ramp" || k == "static_factor")) ||
                             (kind == "torsen" && k == "torque_bias_ratio") || (kind == "viscous" && k == "viscous_nm_s_rad");
        if (!allowed) return fail("\"" + k + "\" does not belong to kind \"" + kind + "\"");
        p[k] = o[k];
    }
    double v = 0.0;
    if (kind == "viscous" && !(detail::get_number(o, "viscous_nm_s_rad", v) && v > 0.0)) return fail("viscous needs viscous_nm_s_rad > 0");
    if (kind == "torsen" && !(detail::get_number(o, "torque_bias_ratio", v) && v > 1.0)) return fail("torsen needs torque_bias_ratio > 1");
    if (kind == "clutch_lsd") {
        double d = -1.0, c = -1.0;
        const auto ramp = o.find("ramp");
        if (!(detail::get_number(o, "preload_nm", v) && v >= 0.0) || ramp == o.end() || !ramp->is_object() ||
            !detail::get_number(*ramp, "drive_factor", d) || !detail::get_number(*ramp, "coast_factor", c) || d < 0.0 || d >= 1.0 ||
            c < 0.0 || c >= 1.0) {
            return fail("clutch_lsd needs preload_nm >= 0 and ramp {drive_factor, coast_factor} in [0, 1)");
        }
    }
    return p;
}

std::string kind_of(const json& diff) {
    std::string k = "open";
    detail::get_string(diff, "kind", k);
    return k;
}

void set_kind(json& diff, const json& params) {
    for (const char* key : kKindKeys) diff.erase(key);
    for (auto it = params.begin(); it != params.end(); ++it) diff[it.key()] = it.value();
}

struct WheelInfo {
    std::string name;
    bool front = false;
    bool left = false;
};

struct Topology {
    bool ok = false;
    std::string reason;
    std::vector<WheelInfo> wheels;
    std::string gearbox_out;            // "<gearbox id>.out"
    std::size_t out_shaft = 0;          // index of the shaft holding gearbox_out
    std::string root;                   // the differential on the gearbox output
    std::string centre, front, rear;    // AWD: centre + both axles; single axle: front or rear == root
    std::set<std::size_t> drive_shafts; // shafts between the differentials and the wheels (removed by a layout swap)
};

const WheelInfo* wheel_named(const Topology& t, const std::string& name) {
    for (const WheelInfo& w : t.wheels) if (w.name == name) return &w;
    return nullptr;
}

std::string owner_of(const std::string& port) { return port.substr(0, port.find('.')); }

bool ends_with(const std::string& s, const char* suffix) {
    const std::string t(suffix);
    return s.size() > t.size() && s.compare(s.size() - t.size(), t.size(), t) == 0;
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
    for (const bool front : {true, false}) {
        int left = 0, right = 0;
        for (const WheelInfo& w : t.wheels) if (w.front == front) (w.left ? left : right) += 1;
        if (left != 1 || right != 1) return fail(std::string("the ") + (front ? "front" : "rear") + " axle does not have exactly one left and one right wheel");
    }
    const auto pt = vehicle.find("powertrain");
    if (pt == vehicle.end() || !pt->is_object()) return fail("the vehicle has no powertrain");
    const auto comps = pt->find("components");
    const auto shafts = pt->find("shafts");
    if (comps == pt->end() || !comps->is_array() || shafts == pt->end() || !shafts->is_array()) return fail("the powertrain has no components or shafts");
    std::map<std::string, std::string> type_of;
    std::string gearbox;
    for (const json& c : *comps) {
        std::string id, ty;
        if (!c.is_object() || !detail::get_string(c, "id", id) || !detail::get_string(c, "type", ty)) continue;
        type_of[id] = ty;
        if (ty == "transfer_case" || ty == "gear_pair") return fail("it has a " + ty + " (" + id + ")");
        if (ty == "gearbox") {
            if (!gearbox.empty()) return fail("it has two gearboxes");
            gearbox = id;
        }
    }
    if (gearbox.empty()) return fail("it has no gearbox");
    t.gearbox_out = gearbox + ".out";
    auto ports_of = [&](std::size_t i) {
        std::vector<std::string> out;
        const json& s = (*shafts)[i];
        const auto ps = s.is_object() ? s.find("ports") : s.end();
        if (s.is_object() && ps != s.end() && ps->is_array()) {
            for (const json& p : *ps) if (p.is_string()) out.push_back(p.get<std::string>());
        }
        return out;
    };
    bool found = false;
    for (std::size_t i = 0; i < shafts->size(); ++i) {
        const auto ports = ports_of(i);
        if (std::find(ports.begin(), ports.end(), t.gearbox_out) == ports.end()) continue;
        if (found) return fail("the gearbox output is on two shafts");
        found = true;
        t.out_shaft = i;
        for (const std::string& p : ports) {
            if (p == t.gearbox_out) continue;
            const std::string owner = owner_of(p);
            const auto ty = type_of.find(owner);
            if (ty != type_of.end() && ty->second == "differential") {
                if (!t.root.empty() || !ends_with(p, ".carrier")) return fail("the gearbox output feeds more than one differential");
                t.root = owner;
            }
        }
    }
    if (!found) return fail("the gearbox output is not connected");
    if (t.root.empty()) return fail("the gearbox output does not feed a differential");
    // The one other end of a two-port shaft holding `port`.
    auto other_end = [&](const std::string& port, std::string& other) {
        int hits = 0;
        for (std::size_t i = 0; i < shafts->size(); ++i) {
            const auto ports = ports_of(i);
            if (std::find(ports.begin(), ports.end(), port) == ports.end()) continue;
            ++hits;
            if (ports.size() != 2) return false;
            other = ports[0] == port ? ports[1] : ports[0];
            t.drive_shafts.insert(i);
        }
        return hits == 1;
    };
    // The axle a differential drives: both ends must be wheels of one axle (left wheel on .left).
    auto axle_of = [&](const std::string& diff, bool& front) {
        std::string l, r;
        if (!other_end(diff + ".left", l) || !other_end(diff + ".right", r)) return false;
        if (l.rfind("wheel:", 0) != 0 || r.rfind("wheel:", 0) != 0) return false;
        const WheelInfo* wl = wheel_named(t, l.substr(6));
        const WheelInfo* wr = wheel_named(t, r.substr(6));
        if (wl == nullptr || wr == nullptr || wl->front != wr->front || wl == wr) return false;
        front = wl->front;
        return true;
    };
    std::string l, r;
    if (!other_end(t.root + ".left", l) || !other_end(t.root + ".right", r)) return fail("differential " + t.root + " is not connected left and right");
    if (l.rfind("wheel:", 0) == 0) {
        bool front = false;
        t.drive_shafts.clear();
        if (!axle_of(t.root, front)) return fail("differential " + t.root + " does not drive the two wheels of one axle");
        (front ? t.front : t.rear) = t.root;
    } else {
        const std::string a = owner_of(l), b = owner_of(r);
        if (!ends_with(l, ".carrier") || !ends_with(r, ".carrier") || type_of[a] != "differential" || type_of[b] != "differential" || a == b) {
            return fail("differential " + t.root + " feeds something other than two axle differentials");
        }
        bool fa = false, fb = false;
        if (!axle_of(a, fa) || !axle_of(b, fb) || fa == fb) return fail("the two axle differentials do not drive one axle each");
        t.centre = t.root;
        t.front = fa ? a : b;
        t.rear = fa ? b : a;
    }
    for (const auto& [id, ty] : type_of) {
        if (ty == "differential" && id != t.centre && id != t.front && id != t.rear) return fail("it has another differential (" + id + ")");
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

std::string layout_of(const Topology& t) {
    if (!t.centre.empty()) return "AWD";
    return t.front.empty() ? "RWD" : "FWD";
}

json shaft(const std::string& a, const std::string& b) {
    json s = json::object();
    s["ports"] = json::array({a, b});
    return s;
}

} // namespace

// ---- catalogue ------------------------------------------------------------------------------------

std::unique_ptr<DrivelineCatalog> DrivelineCatalog::open(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        if (err != nullptr) *err = path + ": cannot open";
        return nullptr;
    }
    return parse(*text, path, err);
}

std::unique_ptr<DrivelineCatalog> DrivelineCatalog::parse(const std::string& json_text, const std::string& origin, std::string* err) {
    auto bad = [&](const std::string& m) {
        if (err != nullptr) *err = origin + ": " + m;
        return std::unique_ptr<DrivelineCatalog>{};
    };
    const auto doc = detail::parse_json(json_text);
    if (!doc || !doc->is_object()) return bad("not a JSON object");
    std::string format;
    if (!detail::get_string(*doc, "format", format) || format != "rg.driveline_catalog/1") return bad("\"format\" must be \"rg.driveline_catalog/1\"");
    auto cat = std::make_unique<DrivelineCatalog>();
    const auto unit = doc->find("centre_unit");
    double ratio = 0.0, eff = 0.0, inertia = -1.0;
    if (unit == doc->end() || !unit->is_object() || !detail::get_number(*unit, "ratio", ratio) || ratio <= 0.0 ||
        !detail::get_number(*unit, "efficiency", eff) || eff <= 0.0 || eff > 1.0 ||
        !detail::get_number(*unit, "carrier_inertia_kgm2", inertia) || inertia < 0.0 || !unit->contains("carrier_axis_local") ||
        !(*unit)["carrier_axis_local"].is_array() || (*unit)["carrier_axis_local"].size() != 3) {
        return bad("\"centre_unit\" needs ratio > 0, efficiency in (0, 1], carrier_inertia_kgm2 >= 0 and carrier_axis_local [x, y, z]");
    }
    json cu = json::object();
    for (const char* k : {"ratio", "efficiency", "carrier_inertia_kgm2", "carrier_axis_local"}) cu[k] = (*unit)[k];
    cat->centre_unit_json_ = cu.dump();

    const auto diffs = doc->find("differentials");
    if (diffs == doc->end() || !diffs->is_array() || diffs->empty()) return bad("\"differentials\" must be a non-empty array");
    std::set<std::string> ids;
    for (std::size_t i = 0; i < diffs->size(); ++i) {
        const json& d = (*diffs)[i];
        const std::string where = "differentials[" + std::to_string(i) + "]";
        DiffUnit u;
        if (!d.is_object() || !detail::get_string(d, "id", u.id) || !is_lower_snake(u.id) || !ids.insert(u.id).second) {
            return bad(where + ": needs a unique lower_snake \"id\"");
        }
        if (!detail::get_string(d, "name", u.name) || u.name.empty()) return bad(where + ": needs a \"name\"");
        detail::get_string(d, "description", u.description);
        std::string e;
        const auto p = unit_params(d, origin + ": " + where, &e);
        if (!p) {
            if (err != nullptr) *err = e;
            return nullptr;
        }
        u.kind = (*p)["kind"].get<std::string>();
        u.params_json = p->dump();
        cat->diffs_.push_back(std::move(u));
    }
    const auto layouts = doc->find("layouts");
    if (layouts == doc->end() || !layouts->is_array() || layouts->empty()) return bad("\"layouts\" must be a non-empty array");
    ids.clear();
    for (std::size_t i = 0; i < layouts->size(); ++i) {
        const json& l = (*layouts)[i];
        const std::string where = "layouts[" + std::to_string(i) + "]";
        DriveLayoutEntry e;
        if (!l.is_object() || !detail::get_string(l, "id", e.id) || !is_lower_snake(e.id) || !ids.insert(e.id).second) {
            return bad(where + ": needs a unique lower_snake \"id\"");
        }
        if (!detail::get_string(l, "name", e.name) || e.name.empty()) return bad(where + ": needs a \"name\"");
        if (!detail::get_string(l, "layout", e.layout) || (e.layout != "FWD" && e.layout != "RWD" && e.layout != "AWD")) {
            return bad(where + ": \"layout\" must be FWD, RWD or AWD");
        }
        detail::get_string(l, "description", e.description);
        const auto c = l.find("centre");
        if ((e.layout == "AWD") != (c != l.end())) return bad(where + ": an AWD layout needs a \"centre\", FWD and RWD take none");
        if (c != l.end()) {
            if (!c->is_object()) return bad(where + ": \"centre\" must be an object");
            std::string er;
            const auto p = unit_params(*c, origin + ": " + where + ".centre", &er);
            if (!p) {
                if (err != nullptr) *err = er;
                return nullptr;
            }
            const std::string ck = (*p)["kind"].get<std::string>();
            if (ck == "clutch_lsd" || ck == "torsen") {
                return bad(where + ".centre: a centre differential must be open, viscous or locked (physics_sim builds clutch LSD and "
                           "Torsen only as axle differentials)");
            }
            DiffUnit u;
            u.id = e.id + "_centre";
            u.name = e.name;
            u.kind = (*p)["kind"].get<std::string>();
            u.params_json = p->dump();
            e.centre = std::move(u);
        }
        cat->layouts_.push_back(std::move(e));
    }
    return cat;
}

const DriveLayoutEntry* DrivelineCatalog::find_layout(const std::string& id) const {
    for (const auto& l : layouts_) if (l.id == id) return &l;
    return nullptr;
}
const DiffUnit* DrivelineCatalog::find_differential(const std::string& id) const {
    for (const auto& d : diffs_) if (d.id == id) return &d;
    return nullptr;
}
const DiffUnit* DrivelineCatalog::differential_of_kind(const std::string& kind) const {
    for (const auto& d : diffs_) if (d.kind == kind) return &d;
    return nullptr;
}
const DriveLayoutEntry* DrivelineCatalog::awd_layout_of_centre(const std::string& kind) const {
    for (const auto& l : layouts_) if (l.centre && l.centre->kind == kind) return &l;
    return nullptr;
}

// ---- the transform -------------------------------------------------------------------------------

namespace detail {

DrivelineInfo describe_driveline_json(const json& vehicle) {
    DrivelineInfo info;
    // Layout from the driven wheels: the wheels that appear as "wheel:<name>" ports of any shaft (the powertrain
    // builder drives exactly those) - any topology, swappable or not.
    std::set<std::string> driven;
    const auto pt = vehicle.find("powertrain");
    if (pt != vehicle.end() && pt->is_object()) {
        const auto shafts = pt->find("shafts");
        if (shafts != pt->end() && shafts->is_array()) {
            for (const json& s : *shafts) {
                const auto ports = s.is_object() ? s.find("ports") : s.end();
                if (!s.is_object() || ports == s.end() || !ports->is_array()) continue;
                for (const json& p : *ports) {
                    if (!p.is_string()) continue;
                    const std::string port = p.get<std::string>();
                    if (port.rfind("wheel:", 0) == 0) driven.insert(port.substr(6));
                }
            }
        }
    }
    int front_wheels = 0, rear_wheels = 0, front_driven = 0, rear_driven = 0;
    const auto wheels = vehicle.find("wheels");
    if (wheels != vehicle.end() && wheels->is_array()) {
        for (const json& w : *wheels) {
            if (!w.is_object()) continue;
            ++info.wheel_count;
            std::string name;
            get_string(w, "name", name);
            bool is_front = false;
            get_bool(w, "is_front", is_front);
            (is_front ? front_wheels : rear_wheels) += 1;
            if (driven.count(name) != 0) {
                ++info.driven_wheels;
                (is_front ? front_driven : rear_driven) += 1;
            }
        }
    }
    if (info.driven_wheels == 0) info.layout = "other";
    else if (front_driven == front_wheels && rear_driven == rear_wheels) info.layout = "AWD";
    else if (front_driven > 0 && rear_driven == 0) info.layout = "FWD";
    else if (rear_driven > 0 && front_driven == 0) info.layout = "RWD";
    else info.layout = "other";

    const Topology t = read_topology(vehicle);
    info.swappable = t.ok;
    info.reason = t.reason;
    if (t.ok) {
        const json& comps = vehicle["powertrain"]["components"];
        auto kind = [&](const std::string& id) { return id.empty() ? std::string() : kind_of(comps[static_cast<std::size_t>(component_index(vehicle, id))]); };
        info.front_kind = kind(t.front);
        info.rear_kind = kind(t.rear);
        info.centre_kind = kind(t.centre);
    }
    return info;
}

std::string apply_layout_json(json& vehicle, const DrivelineCatalog& catalog, const DriveLayoutEntry& layout) {
    const Topology t = read_topology(vehicle);
    if (!t.ok) return "this car's driveline cannot be swapped: " + t.reason;
    const json& comps = vehicle["powertrain"]["components"];
    auto comp = [&](const std::string& id) -> const json& { return comps[static_cast<std::size_t>(component_index(vehicle, id))]; };
    const std::string current = layout_of(t);
    if (current == layout.layout) {
        if (layout.layout != "AWD") return {};
        if (kind_of(comp(t.centre)) == layout.centre->kind) return {};
    }
    // Templates: the car's own unit of that axle, else the other axle's.
    const json front_tpl = comp(t.front.empty() ? t.rear : t.front);
    const json rear_tpl = comp(t.rear.empty() ? t.front : t.rear);
    std::vector<json> fresh;
    std::vector<json> fresh_shafts;
    auto wheel_port = [&](bool front, bool left) {
        for (const WheelInfo& w : t.wheels) if (w.front == front && w.left == left) return "wheel:" + w.name;
        return std::string();
    };
    auto axle_unit = [&](const json& tpl, const std::string& id) {
        json d = tpl;
        d["id"] = id;
        d["type"] = "differential";
        return d;
    };
    std::string root;
    if (layout.layout == "AWD") {
        json centre;
        if (!t.centre.empty()) {
            centre = comp(t.centre);
        } else {
            centre = *detail::parse_json(catalog.centre_unit_json());
        }
        centre["id"] = "centre";
        centre["type"] = "differential";
        set_kind(centre, *detail::parse_json(layout.centre->params_json));
        // physics_sim refuses a lossy centre that is not open (a locked or viscous centre must be lossless).
        if (layout.centre->kind != "open") centre["efficiency"] = 1.0;
        fresh.push_back(std::move(centre));
        fresh.push_back(axle_unit(front_tpl, "front"));
        fresh.push_back(axle_unit(rear_tpl, "rear"));
        root = "centre";
        fresh_shafts.push_back(shaft("centre.left", "front.carrier"));
        fresh_shafts.push_back(shaft("centre.right", "rear.carrier"));
        fresh_shafts.push_back(shaft("front.left", wheel_port(true, true)));
        fresh_shafts.push_back(shaft("front.right", wheel_port(true, false)));
        fresh_shafts.push_back(shaft("rear.left", wheel_port(false, true)));
        fresh_shafts.push_back(shaft("rear.right", wheel_port(false, false)));
    } else {
        const bool front = layout.layout == "FWD";
        fresh.push_back(axle_unit(front ? front_tpl : rear_tpl, "diff"));
        root = "diff";
        fresh_shafts.push_back(shaft("diff.left", wheel_port(front, true)));
        fresh_shafts.push_back(shaft("diff.right", wheel_port(front, false)));
    }
    const std::set<std::string> old_ids = {t.centre, t.front, t.rear};
    // A new id must not clash with a component that stays.
    for (const json& c : comps) {
        std::string id;
        if (!c.is_object() || !detail::get_string(c, "id", id) || old_ids.count(id) != 0) continue;
        for (const json& f : fresh) {
            if (f["id"].get<std::string>() == id) return "this car's driveline cannot be swapped: a component is already called \"" + id + "\"";
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
        if (t.drive_shafts.count(i) != 0) {
            if (!placed) {
                for (const json& s : fresh_shafts) new_shafts.push_back(s);
                placed = true;
            }
            continue;
        }
        json s = shafts[i];
        if (i == t.out_shaft) {
            for (json& p : s["ports"]) {
                if (p.is_string() && p.get<std::string>() == t.root + ".carrier") p = root + ".carrier";
            }
        }
        new_shafts.push_back(std::move(s));
    }
    if (!placed) for (const json& s : fresh_shafts) new_shafts.push_back(s);
    vehicle["powertrain"]["components"] = std::move(new_comps);
    vehicle["powertrain"]["shafts"] = std::move(new_shafts);
    return {};
}

std::string apply_axle_diff_json(json& vehicle, bool front, const DiffUnit& unit, int* index) {
    const Topology t = read_topology(vehicle);
    if (!t.ok) return "this car's driveline cannot be swapped: " + t.reason;
    const std::string& id = front ? t.front : t.rear;
    if (id.empty()) return std::string("the ") + (front ? "front" : "rear") + " axle is not driven";
    const int i = component_index(vehicle, id);
    if (index != nullptr) *index = i;
    json& d = vehicle["powertrain"]["components"][static_cast<std::size_t>(i)];
    set_kind(d, *detail::parse_json(unit.params_json));
    return {};
}

} // namespace detail

DrivelineInfo describe_driveline(const std::string& vehicle_json) {
    const auto doc = detail::parse_json(vehicle_json);
    if (!doc || !doc->is_object()) {
        DrivelineInfo info;
        info.reason = "not a JSON object";
        return info;
    }
    return detail::describe_driveline_json(*doc);
}

std::string apply_driveline(const std::string& vehicle_json, const DrivelineCatalog& catalog, const DrivelineChoice& choice,
                            std::string* out_json) {
    auto doc = detail::parse_json(vehicle_json);
    if (!doc || !doc->is_object()) return "the vehicle file is not a JSON object";
    if (!choice.layout_id.empty()) {
        const DriveLayoutEntry* l = catalog.find_layout(choice.layout_id);
        if (l == nullptr) return "\"" + choice.layout_id + "\" is not a drive layout";
        const std::string e = detail::apply_layout_json(*doc, catalog, *l);
        if (!e.empty()) return e;
    }
    for (const bool front : {true, false}) {
        const std::string& id = front ? choice.front_diff_id : choice.rear_diff_id;
        if (id.empty()) continue;
        const DiffUnit* u = catalog.find_differential(id);
        if (u == nullptr) return "\"" + id + "\" is not a differential";
        const std::string e = detail::apply_axle_diff_json(*doc, front, *u);
        if (!e.empty()) return e;
    }
    if (out_json != nullptr) *out_json = doc->dump();
    return {};
}

std::string diff_kind_label(const std::string& kind) {
    if (kind == "open") return "Open";
    if (kind == "clutch_lsd") return "Clutch LSD";
    if (kind == "torsen") return "Torsen";
    if (kind == "viscous") return "Viscous";
    if (kind == "locked") return "Locked";
    return kind;
}

std::string driveline_summary(const DrivelineInfo& info) {
    std::string s = info.layout;
    auto add = [&](const std::string& kind, const char* where) {
        if (!kind.empty()) s += " · " + diff_kind_label(kind) + " " + where;
    };
    add(info.centre_kind, "centre");
    add(info.front_kind, "front");
    add(info.rear_kind, "rear");
    return s;
}

} // namespace rg
