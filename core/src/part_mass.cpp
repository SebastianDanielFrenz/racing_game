// part_mass.cpp - rg/part_mass.h: the per-part mass tree (owner on PR #19).
#include "rg/part_mass.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <set>

namespace rg {

namespace {

using json = nlohmann::json;
namespace fs = std::filesystem;

// Every key the tree builder looks up unconditionally; a table without one of them is refused at load.
constexpr const char* kRequiredParts[] = {
    "clutch",           "propshaft",        "halfshaft",       "steering",        "brake_front",
    "brake_rear",       "suspension_front", "suspension_rear", "wheel_default",   "cooling_system",
    "park_lock",        "fuel_tank",        "nitrous_kit",     "n2o_bottle",      "electronics_other",
    "engine_unknown",   "gearbox_unknown",  "diff_axle_open",  "diff_centre_open", "layout_conversion",
    "conversion_driveline", "conversion_transfer",
};

// Keys a node can carry besides the table's own (physics_carries may name them).
constexpr const char* kTreeKeys[] = {"wheel", "engine", "gearbox", "body"};

bool fail(std::string* err, const std::string& msg) {
    if (err != nullptr) *err = msg;
    return false;
}

std::string resolve(const std::string& dir, const std::string& ref) {
    const fs::path p(ref);
    if (p.is_absolute() || dir.empty()) return p.lexically_normal().generic_string();
    return (fs::path(dir) / p).lexically_normal().generic_string();
}

std::optional<json> read_json_file(const std::string& path) {
    const auto text = detail::read_text_file(path);
    if (!text) return std::nullopt;
    auto doc = detail::parse_json(*text);
    if (!doc || !doc->is_object()) return std::nullopt;
    return doc;
}

std::string lower_name(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

// The builder's working state.
struct Builder {
    const PartMassTable& table;
    const PartTreeInput& in;
    std::string error;

    PartNode leaf(const std::string& id, const std::string& key, double capacity_kg = 0.0,
                  const std::string& label_override = std::string()) {
        PartNode n;
        n.id = id;
        n.key = key;
        const PartMassDef* def = table.find(key);
        if (def == nullptr) {
            if (error.empty()) error = "part_masses.json has no mass for \"" + key + "\"";
            return n;
        }
        n.label = label_override.empty() ? def->label : label_override;
        n.own_kg = def->mass_kg + def->per_capacity_kg * std::max(0.0, capacity_kg);
        n.source = "part_masses.json " + key;
        n.carried_by_physics = table.carried_by_physics(key);
        return n;
    }
};

PartNode group(const std::string& id, const std::string& label) {
    PartNode n;
    n.id = id;
    n.label = label;
    n.source = "group";
    return n;
}

PartNode* find_mut(PartNode& n, const std::string& id) {
    if (n.id == id) return &n;
    for (PartNode& c : n.children) {
        if (PartNode* f = find_mut(c, id)) return f;
    }
    return nullptr;
}

void collect_ids(const PartNode& n, std::set<std::string>& ids, std::string* dup) {
    if (!ids.insert(n.id).second && dup != nullptr && dup->empty()) *dup = n.id;
    for (const PartNode& c : n.children) collect_ids(c, ids, dup);
}

// "diff.left" -> {"diff", "left"}.
std::pair<std::string, std::string> split_port(const std::string& port) {
    const auto dot = port.find('.');
    if (dot == std::string::npos) return {port, std::string()};
    return {port.substr(0, dot), port.substr(dot + 1)};
}

struct WheelFact {
    std::string name;
    bool front = false;
    double mass_kg = 0.0;
    bool has_mass = false;
};

std::string corner_label(const std::string& wheel) {
    const std::string n = lower_name(wheel);
    if (n == "fl") return "Front left";
    if (n == "fr") return "Front right";
    if (n == "rl") return "Rear left";
    if (n == "rr") return "Rear right";
    return "Wheel " + wheel;
}

} // namespace

const PartMassDef* PartMassTable::find(const std::string& key) const {
    for (const PartMassDef& p : parts) {
        if (p.key == key) return &p;
    }
    return nullptr;
}

bool PartMassTable::carried_by_physics(const std::string& key) const {
    return std::find(physics_carries.begin(), physics_carries.end(), key) != physics_carries.end();
}

std::optional<PartMassTable> parse_part_mass_table(const std::string& json_text, const std::string& origin, std::string* err) {
    const auto doc = detail::parse_json(json_text);
    if (!doc || !doc->is_object()) {
        fail(err, origin + ": not a JSON object");
        return std::nullopt;
    }
    PartMassTable t;
    if (!detail::get_string(*doc, "format", t.format) || t.format != "rg.part_masses/1") {
        fail(err, origin + ": format must be rg.part_masses/1");
        return std::nullopt;
    }
    const auto parts = doc->find("parts");
    if (parts == doc->end() || !parts->is_object() || parts->empty()) {
        fail(err, origin + ": \"parts\" must be a non-empty object");
        return std::nullopt;
    }
    for (auto it = parts->begin(); it != parts->end(); ++it) {
        const std::string where = origin + ": parts." + it.key() + ": ";
        if (!it.value().is_object()) {
            fail(err, where + "must be an object");
            return std::nullopt;
        }
        PartMassDef d;
        d.key = it.key();
        if (!detail::get_string(it.value(), "label", d.label) || d.label.empty()) {
            fail(err, where + "\"label\" missing");
            return std::nullopt;
        }
        if (!detail::get_number(it.value(), "mass_kg", d.mass_kg) || !(d.mass_kg >= 0.0) || !(d.mass_kg < 5000.0)) {
            fail(err, where + "\"mass_kg\" must be a number in [0, 5000)");
            return std::nullopt;
        }
        if (it.value().contains("per_capacity_kg") &&
            (!detail::get_number(it.value(), "per_capacity_kg", d.per_capacity_kg) || !(d.per_capacity_kg >= 0.0) ||
             !(d.per_capacity_kg < 100.0))) {
            fail(err, where + "\"per_capacity_kg\" must be a number in [0, 100)");
            return std::nullopt;
        }
        if (!detail::get_string(it.value(), "source", d.source) || d.source.empty()) {
            fail(err, where + "\"source\" missing (every figure names where it comes from)");
            return std::nullopt;
        }
        t.parts.push_back(std::move(d));
    }
    for (const char* key : kRequiredParts) {
        if (t.find(key) == nullptr) {
            fail(err, origin + ": parts." + key + " missing");
            return std::nullopt;
        }
    }
    const auto carries = doc->find("physics_carries");
    if (carries == doc->end() || !carries->is_array()) {
        fail(err, origin + ": \"physics_carries\" must be an array (empty when physics carries no part mass itself)");
        return std::nullopt;
    }
    for (const json& k : *carries) {
        if (!k.is_string()) {
            fail(err, origin + ": physics_carries entries must be part keys");
            return std::nullopt;
        }
        const std::string key = k.get<std::string>();
        const bool tree_key = std::find(std::begin(kTreeKeys), std::end(kTreeKeys), key) != std::end(kTreeKeys);
        if (key == "body" || (!tree_key && t.find(key) == nullptr)) {
            fail(err, origin + ": physics_carries names \"" + key + "\", which is not a part key");
            return std::nullopt;
        }
        t.physics_carries.push_back(key);
    }
    return t;
}

std::optional<PartMassTable> load_part_mass_table(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        fail(err, path + ": cannot open");
        return std::nullopt;
    }
    return parse_part_mass_table(*text, path, err);
}

double PartNode::total_kg() const {
    double t = own_kg;
    for (const PartNode& c : children) t += c.total_kg();
    return t;
}

double PartNode::carried_kg() const {
    double t = carried_by_physics ? own_kg : 0.0;
    for (const PartNode& c : children) t += c.carried_kg();
    return t;
}

const PartNode* PartNode::find(const std::string& node_id) const {
    if (id == node_id) return this;
    for (const PartNode& c : children) {
        if (const PartNode* f = c.find(node_id)) return f;
    }
    return nullptr;
}

PartTree build_part_tree(const PartMassTable& table, const PartTreeInput& in) {
    PartTree tree;
    Builder b{table, in, {}};
    const auto doc = detail::parse_json(in.vehicle_json);
    if (!doc || !doc->is_object()) {
        tree.error = "the vehicle file is not a JSON object";
        return tree;
    }
    const json& veh = *doc;
    if (!(in.body_kg > 0.0)) {
        tree.error = "the car has no body mass (catalog chassis.body_mass_kg)";
        return tree;
    }

    PartNode& root = tree.root;
    root.id = "body";
    root.key = "body";
    root.label = "Body";
    root.own_kg = in.body_kg;
    root.source = "catalog chassis.body_mass_kg";
    root.carried_by_physics = false;

    // ---- the vehicle file's parts ----
    std::vector<WheelFact> wheels;
    if (const auto w = veh.find("wheels"); w != veh.end() && w->is_array()) {
        for (const json& wj : *w) {
            if (!wj.is_object()) continue;
            WheelFact f;
            detail::get_string(wj, "name", f.name);
            detail::get_bool(wj, "is_front", f.front);
            f.has_mass = detail::get_number(wj, "wheel_mass", f.mass_kg);
            wheels.push_back(f);
        }
    }
    auto wheel_is_front = [&](const std::string& name) {
        for (const WheelFact& f : wheels) {
            if (f.name == name) return f.front;
        }
        return false;
    };

    const json empty_array = json::array();
    const json* comps = &empty_array;
    const json* shafts = &empty_array;
    if (const auto pt = veh.find("powertrain"); pt != veh.end() && pt->is_object()) {
        if (const auto c = pt->find("components"); c != pt->end() && c->is_array()) comps = &*c;
        if (const auto s = pt->find("shafts"); s != pt->end() && s->is_array()) shafts = &*s;
    }
    // Shaft connections: port -> the ports it shares a shaft with.
    std::multimap<std::string, std::string> linked;
    for (const json& s : *shafts) {
        const auto ports = s.find("ports");
        if (!s.is_object() || ports == s.end() || !ports->is_array()) continue;
        std::vector<std::string> ps;
        for (const json& p : *ports) {
            if (p.is_string()) ps.push_back(p.get<std::string>());
        }
        for (const std::string& a : ps) {
            for (const std::string& c : ps) {
                if (a != c) linked.emplace(a, c);
            }
        }
    }

    PartNode engine = group("engine", in.engine.label.empty() ? std::string("Engine") : in.engine.label);
    engine.key = "engine";
    if (in.engine.mass_kg > 0.0) {
        engine.own_kg = in.engine.mass_kg;
        engine.source = in.engine.source.empty() ? std::string("engine catalogue") : in.engine.source;
    } else {
        const PartNode fb = b.leaf("engine", "engine_unknown");
        engine.own_kg = fb.own_kg;
        engine.source = "part_masses.json engine_unknown (the engine has no catalogue mass)";
    }
    engine.carried_by_physics = table.carried_by_physics("engine");
    if (veh.contains("cooling_system")) engine.children.push_back(b.leaf("engine.cooling", "cooling_system"));

    PartNode drivetrain = group("drivetrain", "Drivetrain");
    PartNode running = group("running_gear", "Wheels, brakes and suspension");
    PartNode fuel = group("fuel_system", "Fuel system");
    PartNode electronics = group("electronics", "Electronics");

    std::optional<PartNode> gearbox;
    std::vector<PartNode> park_locks;
    std::optional<PartNode> nitrous_kit;
    std::vector<PartNode> bottles;
    std::set<std::string> used_ids;
    auto unique_id = [&](const std::string& base) {
        std::string id = base;
        for (int n = 2; used_ids.count(id) != 0; ++n) id = base + "_" + std::to_string(n);
        used_ids.insert(id);
        return id;
    };
    bool need_front_prop = false, need_rear_prop = false;
    bool front_driven = false, rear_driven = false;
    const bool engine_front = in.engine_bay == "front";

    for (const json& c : *comps) {
        if (!c.is_object()) continue;
        std::string type, cid, kind;
        detail::get_string(c, "type", type);
        detail::get_string(c, "id", cid);
        detail::get_string(c, "kind", kind);
        if (type == "engine") {
            continue; // the engine node above (the installed engine's catalogue figure)
        }
        if (type == "clutch") {
            drivetrain.children.push_back(b.leaf(unique_id("drivetrain.clutch"), "clutch"));
        } else if (type == "gearbox") {
            PartNode g = group(unique_id("drivetrain.gearbox"), in.gearbox.label.empty() ? std::string("Gearbox") : in.gearbox.label);
            g.key = "gearbox";
            if (in.gearbox.mass_kg > 0.0) {
                g.own_kg = in.gearbox.mass_kg;
                g.source = in.gearbox.source.empty() ? std::string("gearbox catalogue") : in.gearbox.source;
            } else {
                g.own_kg = b.leaf("x", "gearbox_unknown").own_kg;
                g.source = "part_masses.json gearbox_unknown (the gearbox is not in the gearbox catalogue)";
            }
            g.carried_by_physics = table.carried_by_physics("gearbox");
            gearbox = std::move(g);
        } else if (type == "park_lock") {
            park_locks.push_back(b.leaf("park_lock", "park_lock"));
        } else if (type == "differential") {
            if (kind.empty()) kind = "open";
            // Which wheels / differentials its two outputs drive.
            std::vector<std::string> driven;
            bool feeds_diff = false;
            for (const char* side : {"left", "right"}) {
                const auto range = linked.equal_range(cid + "." + side);
                for (auto it = range.first; it != range.second; ++it) {
                    if (it->second.rfind("wheel:", 0) == 0) {
                        driven.push_back(it->second.substr(6));
                    } else if (split_port(it->second).second == "carrier") {
                        feeds_diff = true;
                    }
                }
            }
            if (feeds_diff && driven.empty()) {
                PartNode tc = b.leaf(unique_id("drivetrain.transfer_case"), "diff_centre_" + kind);
                drivetrain.children.push_back(std::move(tc));
                // A centre differential feeds both axles: the one at the far end needs a propshaft.
                if (engine_front) need_rear_prop = true;
                else need_front_prop = true;
                continue;
            }
            const bool front = !driven.empty() && wheel_is_front(driven.front());
            if (!driven.empty()) (front ? front_driven : rear_driven) = true;
            PartNode axle = b.leaf(unique_id(front ? "drivetrain.front_axle" : "drivetrain.rear_axle"), "diff_axle_" + kind,
                                   0.0, front ? "Front axle" : "Rear axle");
            for (const std::string& w : driven) {
                PartNode h = b.leaf(axle.id + ".halfshaft_" + lower_name(w), "halfshaft");
                h.label = "Half shaft " + w;
                axle.children.push_back(std::move(h));
            }
            // A single axle differential at the far end from the engine is driven through a propshaft.
            const bool far = front ? in.engine_bay != "front" : engine_front;
            if (far) (front ? need_front_prop : need_rear_prop) = true;
            drivetrain.children.push_back(std::move(axle));
        } else if (type == "fuel_tank") {
            double cap = 0.0;
            detail::get_number(c, "capacity_kg", cap);
            fuel.children.push_back(b.leaf(unique_id("fuel_system.tank"), "fuel_tank", cap));
        } else if (type == "nitrous_kit") {
            nitrous_kit = b.leaf("engine.nitrous_kit", "nitrous_kit");
        } else if (type == "n2o_bottle") {
            double cap = 0.0;
            if (!detail::get_number(c, "capacity_kg", cap)) {
                std::string ref;
                if (detail::get_string(c, "ref", ref)) {
                    if (const auto bottle = read_json_file(resolve(in.vehicle_dir, ref))) detail::get_number(*bottle, "capacity_kg", cap);
                }
            }
            bottles.push_back(b.leaf("bottle", "n2o_bottle", cap));
        } else {
            tree.error = "part_masses: no mass rule for the powertrain component type \"" + type + "\" (" + cid + ")";
            return tree;
        }
    }
    // A propshaft only where the drive actually reaches an axle at the far end.
    if (need_front_prop || need_rear_prop) {
        PartNode p = b.leaf(unique_id("drivetrain.propshaft"), "propshaft");
        p.label = need_rear_prop ? "Propshaft (rear)" : "Propshaft (front)";
        drivetrain.children.push_back(std::move(p));
    }
    tree.layout = front_driven && rear_driven ? "AWD" : front_driven ? "FWD" : rear_driven ? "RWD" : "";
    // Owner on PR #19 (2026-10-15): every layout swap is allowed, but a layout other than the car's stock one carries
    // a weight penalty "to reflect that the drivetrain needs to run across the entire car, like in an AWD setup".
    // The new axle's differential and half shafts (and an AWD transfer case) come from the file like any other part;
    // on top of that the conversion adds its mounting hardware and, where the layout rule above gives the car no more
    // propshafts than it had stock, a driveline along the car.
    if (!in.stock_vehicle_json.empty()) {
        PartTreeInput stock_in = in;
        stock_in.vehicle_json = in.stock_vehicle_json;
        stock_in.stock_vehicle_json.clear();
        stock_in.extras.clear();
        const PartTree stock = build_part_tree(table, stock_in);
        tree.stock_layout = stock.layout;
        if (stock.ok && !stock.layout.empty() && !tree.layout.empty() && stock.layout != tree.layout) {
            auto props = [](const PartNode& dt) {
                int n = 0;
                for (const PartNode& c : dt.children) n += c.key == "propshaft" ? 1 : 0;
                return n;
            };
            const PartNode* stock_dt = stock.root.find("drivetrain");
            const int stock_props = stock_dt != nullptr ? props(*stock_dt) : 0;
            PartNode conv = b.leaf(unique_id("drivetrain.layout_conversion"), "layout_conversion", 0.0,
                                   "Layout conversion (" + stock.layout + " to " + tree.layout + ")");
            // A 2WD layout has no transfer case of its own to take the drive to the other end of the car.
            if (tree.layout != "AWD") conv.children.push_back(b.leaf(conv.id + ".transfer_drive", "conversion_transfer"));
            if (props(drivetrain) <= stock_props) conv.children.push_back(b.leaf(conv.id + ".driveline", "conversion_driveline"));
            drivetrain.children.push_back(std::move(conv));
        }
    }
    if (gearbox) {
        for (PartNode& pl : park_locks) {
            pl.id = gearbox->id + "." + pl.id;
            gearbox->children.push_back(std::move(pl));
        }
        park_locks.clear();
        // Gearbox right after the clutch: the order a player reads the driveline in.
        auto pos = std::find_if(drivetrain.children.begin(), drivetrain.children.end(),
                                [](const PartNode& n) { return n.key == "clutch"; });
        drivetrain.children.insert(pos == drivetrain.children.end() ? drivetrain.children.begin() : pos + 1, std::move(*gearbox));
    }
    for (PartNode& pl : park_locks) {
        pl.id = unique_id("drivetrain.park_lock");
        drivetrain.children.push_back(std::move(pl));
    }
    if (nitrous_kit || !bottles.empty()) {
        PartNode kit = nitrous_kit ? std::move(*nitrous_kit) : group("engine.nitrous_kit", "Nitrous system");
        for (std::size_t i = 0; i < bottles.size(); ++i) {
            bottles[i].id = kit.id + ".bottle" + (i == 0 ? std::string() : "_" + std::to_string(i + 1));
            kit.children.push_back(std::move(bottles[i]));
        }
        engine.children.push_back(std::move(kit));
    }

    // ---- running gear ----
    running.children.push_back(b.leaf("running_gear.steering", "steering"));
    std::set<std::string> braked;
    if (const auto bs = veh.find("brake_system"); bs != veh.end() && bs->is_object()) {
        if (const auto corners = bs->find("corners"); corners != bs->end() && corners->is_array()) {
            for (const json& cj : *corners) {
                std::string w;
                if (cj.is_object() && detail::get_string(cj, "wheel", w)) braked.insert(w);
            }
        }
    }
    for (const WheelFact& w : wheels) {
        PartNode corner = group(unique_id("running_gear." + lower_name(w.name)), corner_label(w.name));
        PartNode wheel;
        wheel.id = corner.id + ".wheel";
        wheel.key = "wheel";
        wheel.label = "Wheel and tyre";
        if (w.has_mass && w.mass_kg > 0.0) {
            wheel.own_kg = w.mass_kg;
            wheel.source = "vehicle file wheel_mass";
        } else {
            wheel.own_kg = b.leaf("x", "wheel_default").own_kg;
            wheel.source = "part_masses.json wheel_default";
        }
        wheel.carried_by_physics = table.carried_by_physics("wheel");
        corner.children.push_back(std::move(wheel));
        if (braked.count(w.name) != 0) corner.children.push_back(b.leaf(corner.id + ".brake", w.front ? "brake_front" : "brake_rear"));
        corner.children.push_back(b.leaf(corner.id + ".suspension", w.front ? "suspension_front" : "suspension_rear"));
        running.children.push_back(std::move(corner));
    }

    // ---- electronics parts the vehicle file names ----
    if (const auto ctrls = veh.find("controllers"); ctrls != veh.end() && ctrls->is_array()) {
        for (const json& c : *ctrls) {
            std::string type, cid, ref, kind;
            if (!c.is_object() || !detail::get_string(c, "type", type) || type != "electronics") continue;
            detail::get_string(c, "id", cid);
            if (detail::get_string(c, "ref", ref)) {
                if (const auto part = read_json_file(resolve(in.vehicle_dir, ref))) detail::get_string(*part, "kind", kind);
            }
            const std::string key = table.find("electronics_" + kind) != nullptr ? "electronics_" + kind : "electronics_other";
            electronics.children.push_back(b.leaf(unique_id("electronics." + (cid.empty() ? kind : cid)), key));
        }
    }

    root.children.push_back(std::move(engine));
    root.children.push_back(std::move(drivetrain));
    root.children.push_back(std::move(running));
    if (!fuel.children.empty()) root.children.push_back(std::move(fuel));
    if (!electronics.children.empty()) root.children.push_back(std::move(electronics));

    // ---- parts a setup feature adds (a turbo kit) ----
    for (const PartAttachment& a : in.extras) {
        PartNode* parent = find_mut(root, a.parent_id);
        if (parent == nullptr) {
            tree.error = "part \"" + a.node.id + "\" names the parent \"" + a.parent_id + "\", which is not in the tree";
            return tree;
        }
        PartNode n = a.node;
        n.id = a.parent_id + "." + n.id;
        n.carried_by_physics = n.carried_by_physics || (!n.key.empty() && table.carried_by_physics(n.key));
        parent->children.push_back(std::move(n));
    }

    if (!b.error.empty()) {
        tree.error = b.error;
        return tree;
    }
    std::set<std::string> ids;
    std::string dup;
    collect_ids(root, ids, &dup);
    if (!dup.empty()) {
        tree.error = "two parts share the id \"" + dup + "\"";
        return tree;
    }
    tree.total_kg = root.total_kg();
    tree.physics_carried_kg = root.carried_kg();
    // To the gram: the per-capacity figures leave float noise that would otherwise show as 1499.99999 kg.
    tree.chassis_kg = std::round((tree.total_kg - tree.physics_carried_kg) * 1000.0) / 1000.0;
    tree.ok = std::isfinite(tree.chassis_kg) && tree.chassis_kg > 0.0;
    if (!tree.ok) tree.error = "the part masses do not leave a positive chassis mass";
    return tree;
}

std::vector<MassGroup> mass_groups(const PartTree& tree) {
    std::vector<MassGroup> out;
    if (!tree.ok) return out;
    out.push_back({tree.root.id, tree.root.label, tree.root.own_kg});
    for (const PartNode& g : tree.root.children) out.push_back({g.id, g.label, g.total_kg()});
    return out;
}

} // namespace rg
