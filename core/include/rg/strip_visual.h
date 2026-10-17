// rg/strip_visual.h - which model nodes a car's weight-reduction choices take off the visible car (owner on PR #22,
// 2026-10-16: "visually remove the elements that were removed for weight reduction"). Engine-neutral, exception-free,
// no Godot type: GDScript only switches the visibility of the nodes named here.
//
// Data: data/vehicles/strip_visual.json (rg.strip_visual/1). Per car model (the .glb file's stem) a list of rules
// {slot, part, hide}: when the setup fits part `part` in the weight-reduction slot `slot` (the part_choice option's
// slot and part ids, e.g. audio / removed), the nodes in `hide` are hidden. Several rules can name the same node.
// A slot with no rule for the fitted part changes nothing on screen (a lighter bonnet looks like the steel one), and
// a model that has no node for an element says so under `not_separate` (the element is part of the body mesh).
// `after_regeneration` rules name nodes that tools/cargen produces only in a regenerated model; a node the loaded
// model does not have is ignored at runtime.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace rg {

struct StripRule {
    std::string slot;               // the part_choice option's slot ("audio", "seat_passenger" ...)
    std::string part;               // the part id fitted ("removed", "stripped" ...)
    std::vector<std::string> hide;  // node names of the model
    bool after_regeneration = false; // the nodes exist only in a model regenerated with the current tools/cargen
    std::string note;
};

struct StripModel {
    std::vector<StripRule> rules;
    std::map<std::string, std::string> not_separate; // slot -> why nothing can be hidden (no node in the model)
};

struct StripVisualTable {
    std::string format;
    std::map<std::string, StripModel> models; // by model stem ("pico", "car_sedan" ...)

    [[nodiscard]] const StripModel* find(const std::string& model_stem) const;
    // The nodes to hide for the fitted parts: `fitted` maps slot -> part id (a slot at "stock" or absent hides
    // nothing). Sorted, unique. Empty for a model with no entry.
    [[nodiscard]] std::vector<std::string> hidden_nodes(const std::string& model_stem,
                                                        const std::map<std::string, std::string>& fitted) const;
    // Every node any rule of the model may hide (the visual resets these before applying a new setup).
    [[nodiscard]] std::vector<std::string> controlled_nodes(const std::string& model_stem) const;
};

// "…/data/models/pico/pico.glb" -> "pico" (the file's stem).
std::string model_stem(const std::string& model_path);

// Strict, exception-free. nullopt + *err on a bad file.
std::optional<StripVisualTable> parse_strip_visual(const std::string& json_text, const std::string& origin, std::string* err);
std::optional<StripVisualTable> load_strip_visual(const std::string& path, std::string* err);

} // namespace rg
