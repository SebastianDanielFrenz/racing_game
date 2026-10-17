// strip_visual.cpp - rg/strip_visual.h: weight-reduction choices -> model nodes to hide.
#include "rg/strip_visual.h"

#include "json_util.h"

#include <algorithm>
#include <filesystem>
#include <set>

namespace rg {

namespace {

using json = nlohmann::json;

bool fail(std::string* err, const std::string& msg) {
    if (err != nullptr) *err = msg;
    return false;
}

bool read_name_list(const json& v, std::vector<std::string>& out) {
    if (!v.is_array()) return false;
    for (const json& e : v) {
        if (!e.is_string() || e.get<std::string>().empty()) return false;
        out.push_back(e.get<std::string>());
    }
    return true;
}

} // namespace

std::string model_stem(const std::string& model_path) {
    return std::filesystem::path(model_path).stem().string();
}

const StripModel* StripVisualTable::find(const std::string& model_stem_in) const {
    const auto it = models.find(model_stem_in);
    return it == models.end() ? nullptr : &it->second;
}

std::vector<std::string> StripVisualTable::hidden_nodes(const std::string& model_stem_in,
                                                        const std::map<std::string, std::string>& fitted) const {
    std::set<std::string> out;
    if (const StripModel* m = find(model_stem_in)) {
        for (const StripRule& r : m->rules) {
            const auto it = fitted.find(r.slot);
            if (it != fitted.end() && it->second == r.part) out.insert(r.hide.begin(), r.hide.end());
        }
    }
    return {out.begin(), out.end()};
}

std::vector<std::string> StripVisualTable::controlled_nodes(const std::string& model_stem_in) const {
    std::set<std::string> out;
    if (const StripModel* m = find(model_stem_in)) {
        for (const StripRule& r : m->rules) out.insert(r.hide.begin(), r.hide.end());
    }
    return {out.begin(), out.end()};
}

std::optional<StripVisualTable> parse_strip_visual(const std::string& json_text, const std::string& origin, std::string* err) {
    const auto doc = detail::parse_json(json_text);
    if (!doc || !doc->is_object()) {
        fail(err, origin + ": not a JSON object");
        return std::nullopt;
    }
    StripVisualTable t;
    if (!detail::get_string(*doc, "format", t.format) || t.format != "rg.strip_visual/1") {
        fail(err, origin + ": format must be rg.strip_visual/1");
        return std::nullopt;
    }
    const auto models = doc->find("models");
    if (models == doc->end() || !models->is_object()) {
        fail(err, origin + ": \"models\" must be an object");
        return std::nullopt;
    }
    for (auto it = models->begin(); it != models->end(); ++it) {
        const std::string where = origin + ": model \"" + it.key() + "\"";
        if (!it->is_object()) {
            fail(err, where + " must be an object");
            return std::nullopt;
        }
        StripModel m;
        if (const auto rules = it->find("rules"); rules != it->end()) {
            if (!rules->is_array()) {
                fail(err, where + ": \"rules\" must be a list");
                return std::nullopt;
            }
            for (const json& r : *rules) {
                StripRule rule;
                if (!r.is_object() || !detail::get_string(r, "slot", rule.slot) || rule.slot.empty() ||
                    !detail::get_string(r, "part", rule.part) || rule.part.empty()) {
                    fail(err, where + ": a rule needs a \"slot\" and a \"part\"");
                    return std::nullopt;
                }
                const auto hide = r.find("hide");
                if (hide == r.end() || !read_name_list(*hide, rule.hide) || rule.hide.empty()) {
                    fail(err, where + ": rule " + rule.slot + "/" + rule.part + " needs a non-empty \"hide\" list of node names");
                    return std::nullopt;
                }
                detail::get_bool(r, "after_regeneration", rule.after_regeneration);
                detail::get_string(r, "note", rule.note);
                m.rules.push_back(std::move(rule));
            }
        }
        if (const auto ns = it->find("not_separate"); ns != it->end()) {
            if (!ns->is_object()) {
                fail(err, where + ": \"not_separate\" must be an object");
                return std::nullopt;
            }
            for (auto n = ns->begin(); n != ns->end(); ++n) {
                if (!n->is_string()) {
                    fail(err, where + ": not_separate." + n.key() + " must be a text");
                    return std::nullopt;
                }
                m.not_separate[n.key()] = n->get<std::string>();
            }
        }
        for (const StripRule& r : m.rules) {
            if (m.not_separate.count(r.slot) != 0 && !r.after_regeneration) {
                fail(err, where + ": slot " + r.slot + " has a rule and is also listed under not_separate");
                return std::nullopt;
            }
        }
        t.models[it.key()] = std::move(m);
    }
    return t;
}

std::optional<StripVisualTable> load_strip_visual(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        fail(err, path + ": cannot open");
        return std::nullopt;
    }
    return parse_strip_visual(*text, path, err);
}

} // namespace rg
