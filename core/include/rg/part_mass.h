// rg/part_mass.h - the per-part mass tree of a car (owner on PR #19, 2026-10-15: "a general mass per part system
// where cumulative mass is calculated instead makes more sense. Each parent can have a mass itself and then inherits
// the additional mass of its child components."). Engine-neutral, exception-free, no Godot type.
//
// A car is a tree of parts. Every node has its OWN mass; its cumulative mass is its own plus that of all its
// children. The root is the body (shell, interior, glass, wiring ... everything not modelled as a part of its own);
// its children are the groups the garage shows:
//   engine        the installed engine (engine catalogue figure) -> cooling system, nitrous kit (-> bottle), and any
//                 part a setup feature attaches to it (a turbo kit, PartTreeInput::extras)
//   drivetrain    clutch, gearbox (gearbox catalogue figure; -> park lock), transfer case (a centre differential),
//                 front / rear axle (the axle differential -> one half shaft per driven wheel), propshaft,
//                 layout conversion (a layout other than the stock one: mounting hardware, driveline along the car)
//   running_gear  steering, one corner per wheel (-> wheel and tyre, brake, suspension)
//   fuel_system   the fuel tank (empty)
//   electronics   the electronics parts the vehicle file names (assist unit, drive modes, EPB ...)
// The tree is built from DATA only: the vehicle file as the setup leaves it (its components, shafts, wheels, brake
// corners, controllers), the engine and gearbox catalogues, the per-car body mass (catalog `chassis.body_mass_kg`)
// and the base part masses of data/vehicles/part_masses.json (rg.part_masses/1). So a setup or swap changes the tree
// by changing the files: an engine swap the engine node, a gearbox swap the gearbox node, an AWD conversion adds a
// transfer case, a front axle and a propshaft, an assist unit adds an electronics part.
//
// The physics chassis body mass is the tree's total minus the parts physics_sim already simulates as masses of their
// own (PartMassTable::physics_carries). Today that list is empty: ps::vehicle::WheelDesc::wheel_mass is spin inertia
// only (unsprung mass is not a separate body), and the fuel/N2O contents are not added to the body either - see
// part_masses.json's "source".
#pragma once

#include <optional>
#include <string>
#include <vector>

namespace rg {

struct PartMassDef {
    std::string key;            // "clutch", "diff_axle_open", "electronics_assist_unit" ...
    std::string label;          // "Clutch"
    double mass_kg = 0.0;       // the part's own mass (for a sized part: the fixed share)
    double per_capacity_kg = 0.0; // sized parts (fuel tank, N2O bottle): + this x the capacity in kg
    std::string source;
};

struct PartMassTable {
    std::string format;
    std::vector<PartMassDef> parts;
    std::vector<std::string> physics_carries; // part keys physics simulates as their own mass (excluded from the body)
    [[nodiscard]] const PartMassDef* find(const std::string& key) const;
    [[nodiscard]] bool carried_by_physics(const std::string& key) const;
};

// Strict, exception-free. nullopt + *err on a bad file (unknown format, a missing required part, a negative mass).
std::optional<PartMassTable> load_part_mass_table(const std::string& path, std::string* err);
std::optional<PartMassTable> parse_part_mass_table(const std::string& json_text, const std::string& origin, std::string* err);

struct PartNode {
    std::string id;       // unique in the tree, dotted path ("drivetrain.front_axle.halfshaft_fl")
    std::string key;      // the table key the figure comes from ("" for a group or a catalogue part)
    std::string label;    // "Front axle differential"
    double own_kg = 0.0;
    std::string source;   // where own_kg comes from ("engine catalogue", "vehicle file wheel_mass", the table key)
    bool carried_by_physics = false; // simulated by physics as a mass of its own: not in the chassis body
    std::vector<PartNode> children;

    [[nodiscard]] double total_kg() const;         // own + every descendant
    [[nodiscard]] double carried_kg() const;       // the part of total_kg physics carries itself
    [[nodiscard]] const PartNode* find(const std::string& id) const; // this node or a descendant
};

// A part a setup feature adds below an existing node (a turbo kit below "engine"). `node.id` is the LAST segment;
// the tree makes it unique under the parent ("<parent>.<id>").
struct PartAttachment {
    std::string parent_id;
    PartNode node;
};

struct PartRef {
    std::string label;    // "Hyper V8 twin turbo"
    double mass_kg = 0.0; // 0 = unknown: the table's fallback is used (and named in the source)
    std::string source;
};

struct PartTreeInput {
    double body_kg = 0.0;           // the car's own body (catalog chassis.body_mass_kg)
    std::string vehicle_json;       // the vehicle file as the setup leaves it
    std::string vehicle_dir;        // resolves its relative component/controller refs
    std::string engine_bay = "front"; // "front" | "mid" | "rear" (catalog): where a propshaft is needed
    PartRef engine;                 // the installed engine
    PartRef gearbox;                // the installed gearbox
    std::vector<PartAttachment> extras;
    // The car's STOCK vehicle file ("" = no comparison). When the driven axles differ from it (a FWD/RWD/AWD swap),
    // the drivetrain gets a "layout_conversion" node (mounting hardware; plus a "conversion_driveline" along the car
    // where the new layout has no more propshafts than the stock one) - the owner's weight penalty for a layout swap.
    std::string stock_vehicle_json;
};

struct PartTree {
    bool ok = false;
    std::string error;
    PartNode root;               // the body; its children are the groups
    double total_kg = 0.0;       // root.total_kg()
    double physics_carried_kg = 0.0;
    double chassis_kg = 0.0;     // total - physics_carried: the physics chassis body mass
    std::string layout;          // "FWD" | "RWD" | "AWD" ("" = no driven axle found)
    std::string stock_layout;    // the stock file's layout ("" without PartTreeInput::stock_vehicle_json)
};

PartTree build_part_tree(const PartMassTable& table, const PartTreeInput& input);

// The garage's breakdown: the body (the root's own mass) followed by one row per group (a child of the root) with
// its cumulative mass.
struct MassGroup {
    std::string id;
    std::string label;
    double kg = 0.0;
};
std::vector<MassGroup> mass_groups(const PartTree& tree);

} // namespace rg
