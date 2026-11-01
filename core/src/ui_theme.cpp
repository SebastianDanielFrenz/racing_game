// rg/ui_theme.cpp — see ui_theme.h.
#include "rg/ui_theme.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>

namespace rg {

namespace {

using detail::json;

bool fail(std::string* err, const std::string& origin, const std::string& message) {
    if (err != nullptr) *err = origin + ": " + message;
    return false;
}

bool check_format(const json& root, const char* format, const std::string& origin, std::string* err) {
    std::string found;
    if (!detail::get_string(root, "format", found) || found != format) {
        return fail(err, origin, std::string("\"format\" must be \"") + format + "\"");
    }
    return true;
}

int hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

double to_linear(double channel) {
    return channel <= 0.04045 ? channel / 12.92 : std::pow((channel + 0.055) / 1.055, 2.4);
}

double to_srgb(double linear) {
    const double l = std::clamp(linear, 0.0, 1.0);
    return l <= 0.0031308 ? l * 12.92 : 1.055 * std::pow(l, 1.0 / 2.4) - 0.055;
}

// D65 linear sRGB to Lab (the same formulas as ui_lightness).
std::array<double, 3> to_lab(const UiColour& c) {
    const double r = to_linear(c.r);
    const double g = to_linear(c.g);
    const double b = to_linear(c.b);
    const double x = (0.4124564 * r + 0.3575761 * g + 0.1804375 * b) / 0.95047;
    const double y = 0.2126729 * r + 0.7151522 * g + 0.0721750 * b;
    const double z = (0.0193339 * r + 0.1191920 * g + 0.9503041 * b) / 1.08883;
    const auto f = [](double t) { return t > 216.0 / 24389.0 ? std::cbrt(t) : (24389.0 / 27.0 * t + 16.0) / 116.0; };
    return {116.0 * f(y) - 16.0, 500.0 * (f(x) - f(y)), 200.0 * (f(y) - f(z))};
}

std::string number_text(double value) {
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%.2f", value);
    return buffer;
}

bool read_pairs(const json& j, std::vector<std::array<std::string, 2>>& out, const std::string& origin, std::string* err) {
    if (!j.is_array()) return fail(err, origin, "\"pairs\" must be an array");
    for (const json& p : j) {
        if (!p.is_array() || p.size() != 2 || !p[0].is_string() || !p[1].is_string()) {
            return fail(err, origin, "a pair must be two role names");
        }
        out.push_back({p[0].get<std::string>(), p[1].get<std::string>()});
    }
    return true;
}

bool read_groups(const json& root, UiRoleContract& c, const std::string& origin, std::string* err) {
    const auto groups = root.find("groups");
    if (groups == root.end() || !groups->is_array() || groups->empty()) {
        return fail(err, origin, "needs a non-empty \"groups\" array");
    }
    std::set<std::string> seen;
    for (const json& g : *groups) {
        UiRoleGroup group;
        if (!g.is_object() || !detail::get_string(g, "id", group.id) || !detail::get_string(g, "title", group.title)) {
            return fail(err, origin, "a group needs \"id\" and \"title\" strings");
        }
        const auto roles = g.find("roles");
        if (roles == g.end() || !roles->is_array() || roles->empty()) {
            return fail(err, origin, group.id + ": a group needs a non-empty \"roles\" array");
        }
        for (const json& r : *roles) {
            UiRoleDef def;
            def.group = group.id;
            if (!r.is_object() || !detail::get_string(r, "id", def.id) || !detail::get_string(r, "meaning", def.meaning)) {
                return fail(err, origin, group.id + ": a role needs \"id\" and \"meaning\" strings");
            }
            if (!seen.insert(def.id).second) return fail(err, origin, "role \"" + def.id + "\" is listed twice");
            c.roles.push_back(std::move(def));
        }
        c.groups.push_back(std::move(group));
    }
    return true;
}

bool read_fixed(const json& root, UiRoleContract& c, const std::string& origin, std::string* err) {
    const auto fixed = root.find("fixed");
    if (fixed == root.end() || !fixed->is_object()) return fail(err, origin, "needs a \"fixed\" object");
    for (auto it = fixed->begin(); it != fixed->end(); ++it) {
        if (c.has_role(it.key())) return fail(err, origin, "fixed colour \"" + it.key() + "\" has the name of a role");
        const std::optional<UiColour> colour = it->is_string() ? parse_ui_colour(it->get<std::string>()) : std::nullopt;
        if (!colour) return fail(err, origin, "fixed colour \"" + it.key() + "\" is not #rrggbb or #rrggbbaa");
        c.fixed[it.key()] = *colour;
    }
    const auto over = root.find("default_over");
    if (over == root.end() || !over->is_array() || over->empty()) {
        return fail(err, origin, "needs a non-empty \"default_over\" array");
    }
    for (const json& name : *over) {
        if (!name.is_string() || c.fixed.count(name.get<std::string>()) == 0) {
            return fail(err, origin, "\"default_over\" must name fixed colours");
        }
        const UiColour& scene = c.fixed[name.get<std::string>()];
        if (scene.a != 1.0) return fail(err, origin, "a scene in \"default_over\" must be opaque");
        c.default_over.push_back(name.get<std::string>());
    }
    const auto sits_on = root.find("sits_on");
    if (sits_on == root.end()) return true;
    if (!sits_on->is_object()) return fail(err, origin, "\"sits_on\" must be an object");
    for (auto it = sits_on->begin(); it != sits_on->end(); ++it) {
        if (!it->is_string() || !c.has_role(it.key()) || !c.has_role(it->get<std::string>())) {
            return fail(err, origin, "\"sits_on\" must map a role to a role: " + it.key());
        }
        c.sits_on[it.key()] = it->get<std::string>();
    }
    for (const auto& [start, base] : c.sits_on) {
        std::string at = base;
        for (std::size_t steps = 0; steps <= c.sits_on.size(); ++steps) {
            if (at == start) return fail(err, origin, "\"sits_on\" goes in a circle at " + start);
            const auto next = c.sits_on.find(at);
            if (next == c.sits_on.end()) break;
            at = next->second;
        }
    }
    return true;
}

bool read_contrast(const json& root, UiRoleContract& c, const std::string& origin, std::string* err) {
    const auto classes = root.find("contrast_classes");
    if (classes == root.end() || !classes->is_object() || classes->empty()) {
        return fail(err, origin, "needs a \"contrast_classes\" object");
    }
    std::map<std::string, double> ratios;
    for (auto it = classes->begin(); it != classes->end(); ++it) {
        if (!it->is_number() || it->get<double>() < 1.0 || it->get<double>() > 21.0) {
            return fail(err, origin, "contrast class \"" + it.key() + "\" must be a ratio of 1..21");
        }
        ratios[it.key()] = it->get<double>();
    }
    const auto contrast = root.find("contrast");
    if (contrast == root.end() || !contrast->is_array() || contrast->empty()) {
        return fail(err, origin, "needs a non-empty \"contrast\" array");
    }
    for (const json& entry : *contrast) {
        std::string fg;
        std::string cls;
        if (!entry.is_object() || !detail::get_string(entry, "fg", fg) || !detail::get_string(entry, "class", cls)) {
            return fail(err, origin, "a contrast entry needs \"fg\" and \"class\" strings");
        }
        if (!c.has_role(fg) && c.fixed.count(fg) == 0) return fail(err, origin, "contrast names an unknown colour \"" + fg + "\"");
        const auto ratio = ratios.find(cls);
        if (ratio == ratios.end()) return fail(err, origin, "contrast names an unknown class \"" + cls + "\"");
        const auto on = entry.find("on");
        if (on == entry.end() || !on->is_array() || on->empty()) {
            return fail(err, origin, fg + ": a contrast entry needs a non-empty \"on\" array");
        }
        for (const json& surface : *on) {
            if (!surface.is_string() || !c.has_role(surface.get<std::string>())) {
                return fail(err, origin, fg + ": \"on\" must name roles");
            }
            c.contrast.push_back({fg, surface.get<std::string>(), cls, ratio->second});
        }
    }
    return true;
}

bool read_distinct(const json& root, UiRoleContract& c, const std::string& origin, std::string* err) {
    const auto lightness = root.find("lightness_distinct");
    if (lightness == root.end() || !lightness->is_object() ||
        !detail::get_number(*lightness, "min_lightness_difference", c.min_lightness_difference)) {
        return fail(err, origin, "needs \"lightness_distinct\" with \"min_lightness_difference\"");
    }
    const auto pairs = lightness->find("pairs");
    if (pairs == lightness->end() || !read_pairs(*pairs, c.lightness_pairs, origin, err)) {
        return pairs == lightness->end() ? fail(err, origin, "\"lightness_distinct\" needs \"pairs\"") : false;
    }
    const auto vision = root.find("simulated_vision");
    if (vision == root.end() || !vision->is_object() || !detail::get_string(*vision, "claim", c.vision_claim) ||
        !detail::get_number(*vision, "min_delta_e", c.min_vision_delta_e)) {
        return fail(err, origin, "needs \"simulated_vision\" with \"claim\" and \"min_delta_e\"");
    }
    const auto vision_pairs = vision->find("pairs");
    if (vision_pairs == vision->end() || !read_pairs(*vision_pairs, c.vision_pairs, origin, err)) {
        return vision_pairs == vision->end() ? fail(err, origin, "\"simulated_vision\" needs \"pairs\"") : false;
    }
    const auto matrices = vision->find("matrices");
    if (matrices == vision->end() || !matrices->is_object() || matrices->empty()) {
        return fail(err, origin, "\"simulated_vision\" needs \"matrices\"");
    }
    for (auto it = matrices->begin(); it != matrices->end(); ++it) {
        UiVisionMatrix m{};
        if (!it->is_array() || it->size() != 3) return fail(err, origin, it.key() + ": a matrix is three rows");
        for (std::size_t row = 0; row < 3; ++row) {
            const json& values = (*it)[row];
            if (!values.is_array() || values.size() != 3) return fail(err, origin, it.key() + ": a matrix row is three numbers");
            for (std::size_t col = 0; col < 3; ++col) {
                if (!values[col].is_number()) return fail(err, origin, it.key() + ": a matrix row is three numbers");
                m[row][col] = values[col].get<double>();
            }
        }
        c.vision_matrices.emplace_back(it.key(), m);
    }
    for (const auto* list : {&c.lightness_pairs, &c.vision_pairs}) {
        for (const auto& pair : *list) {
            for (const std::string& role : pair) {
                if (!c.has_role(role)) return fail(err, origin, "a distinct pair names an unknown role \"" + role + "\"");
            }
        }
    }
    return true;
}

void flatten_tokens(const json& j, const std::string& prefix, std::map<std::string, double>& out, std::vector<std::string>& bad) {
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string name = prefix.empty() ? it.key() : prefix + "." + it.key();
        if (it->is_object()) flatten_tokens(*it, name, out, bad);
        else if (it->is_number()) out[name] = it->get<double>();
        else bad.push_back(name);
    }
}

} // namespace

std::optional<UiColour> parse_ui_colour(const std::string& text) {
    if ((text.size() != 7 && text.size() != 9) || text[0] != '#') return std::nullopt;
    double channels[4] = {0.0, 0.0, 0.0, 1.0};
    for (std::size_t i = 0; i + 1 < text.size(); i += 2) {
        const int high = hex_digit(text[i + 1]);
        const int low = hex_digit(text[i + 2]);
        if (high < 0 || low < 0) return std::nullopt;
        channels[i / 2] = (high * 16 + low) / 255.0;
    }
    return UiColour{channels[0], channels[1], channels[2], channels[3]};
}

std::string ui_colour_hex(const UiColour& c) {
    const auto byte = [](double v) { return static_cast<int>(std::lround(std::clamp(v, 0.0, 1.0) * 255.0)); };
    char buffer[12];
    std::snprintf(buffer, sizeof buffer, "#%02x%02x%02x%02x", byte(c.r), byte(c.g), byte(c.b), byte(c.a));
    return buffer;
}

UiColour ui_composite(const UiColour& top, const UiColour& bottom) {
    const auto over = [&](double t, double b) { return t * top.a + b * (1.0 - top.a); };
    return UiColour{over(top.r, bottom.r), over(top.g, bottom.g), over(top.b, bottom.b), 1.0};
}

double ui_luminance(const UiColour& c) {
    return 0.2126 * to_linear(c.r) + 0.7152 * to_linear(c.g) + 0.0722 * to_linear(c.b);
}

double ui_contrast_ratio(const UiColour& a, const UiColour& b) {
    const double la = ui_luminance(a);
    const double lb = ui_luminance(b);
    return (std::max(la, lb) + 0.05) / (std::min(la, lb) + 0.05);
}

double ui_lightness(const UiColour& colour) {
    return to_lab(colour)[0];
}

double ui_delta_e(const UiColour& a, const UiColour& b) {
    const std::array<double, 3> la = to_lab(a);
    const std::array<double, 3> lb = to_lab(b);
    return std::hypot(la[0] - lb[0], la[1] - lb[1], la[2] - lb[2]);
}

UiColour ui_simulate_vision(const UiColour& c, const UiVisionMatrix& m) {
    const double r = to_linear(c.r);
    const double g = to_linear(c.g);
    const double b = to_linear(c.b);
    const auto row = [&](int i) { return m[i][0] * r + m[i][1] * g + m[i][2] * b; };
    return UiColour{to_srgb(row(0)), to_srgb(row(1)), to_srgb(row(2)), c.a};
}

bool UiRoleContract::has_role(const std::string& id) const {
    return std::any_of(roles.begin(), roles.end(), [&](const UiRoleDef& r) { return r.id == id; });
}

const UiColour* UiPalette::role(const std::string& id) const {
    const auto it = colours.find(id);
    return it == colours.end() ? nullptr : &it->second;
}

const double* UiTokens::find(const std::string& name) const {
    const auto it = values.find(name);
    return it == values.end() ? nullptr : &it->second;
}

double UiTokens::px(const std::string& name, double viewport_height) const {
    const double* value = find(name);
    if (value == nullptr) return 0.0;
    return *value * std::max(viewport_height / reference_height_px, min_scale);
}

std::optional<UiRoleContract> parse_ui_role_contract(const std::string& json_text, const std::string& origin, std::string* err) {
    const std::optional<json> root = detail::parse_json(json_text);
    if (!root || !root->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    if (!check_format(*root, "rg.ui_roles/1", origin, err)) return std::nullopt;
    UiRoleContract c;
    if (!read_groups(*root, c, origin, err) || !read_fixed(*root, c, origin, err) || !read_contrast(*root, c, origin, err) ||
        !read_distinct(*root, c, origin, err)) {
        return std::nullopt;
    }
    return c;
}

std::optional<UiPalette> parse_ui_palette(const std::string& json_text, const std::string& origin, std::string* err) {
    const std::optional<json> root = detail::parse_json(json_text);
    if (!root || !root->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    if (!check_format(*root, "rg.ui_palette/1", origin, err)) return std::nullopt;
    UiPalette p;
    if (!detail::get_string(*root, "id", p.id) || p.id.empty() || !detail::get_string(*root, "label", p.label) ||
        !detail::get_string(*root, "source", p.source)) {
        fail(err, origin, "a palette needs \"id\", \"label\" and \"source\" strings");
        return std::nullopt;
    }
    const auto claims = root->find("claims");
    if (claims != root->end()) {
        if (!claims->is_array()) {
            fail(err, origin, "\"claims\" must be an array of strings");
            return std::nullopt;
        }
        for (const json& claim : *claims) {
            if (!claim.is_string()) {
                fail(err, origin, "\"claims\" must be an array of strings");
                return std::nullopt;
            }
            p.claims.push_back(claim.get<std::string>());
        }
    }
    const auto colours = root->find("colours");
    if (colours == root->end() || !colours->is_object()) {
        fail(err, origin, "needs a \"colours\" object");
        return std::nullopt;
    }
    for (auto it = colours->begin(); it != colours->end(); ++it) {
        const std::optional<UiColour> colour = it->is_string() ? parse_ui_colour(it->get<std::string>()) : std::nullopt;
        if (!colour) {
            fail(err, origin, "role \"" + it.key() + "\" is not #rrggbb or #rrggbbaa");
            return std::nullopt;
        }
        p.colours[it.key()] = *colour;
    }
    return p;
}

std::optional<UiTokens> parse_ui_tokens(const std::string& json_text, const std::string& origin, std::string* err) {
    const std::optional<json> root = detail::parse_json(json_text);
    if (!root || !root->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    if (!check_format(*root, "rg.ui_tokens/1", origin, err)) return std::nullopt;
    UiTokens t;
    if (!detail::get_number(*root, "reference_height_px", t.reference_height_px) || !(t.reference_height_px > 0.0) ||
        !detail::get_number(*root, "min_scale", t.min_scale) || !(t.min_scale > 0.0)) {
        fail(err, origin, "needs \"reference_height_px\" and \"min_scale\" above 0");
        return std::nullopt;
    }
    const auto tokens = root->find("tokens");
    if (tokens == root->end() || !tokens->is_object() || tokens->empty()) {
        fail(err, origin, "needs a non-empty \"tokens\" object");
        return std::nullopt;
    }
    std::vector<std::string> bad;
    flatten_tokens(*tokens, "", t.values, bad);
    if (!bad.empty()) {
        fail(err, origin, "token \"" + bad.front() + "\" is not a number");
        return std::nullopt;
    }
    return t;
}

std::optional<UiGameConfig> parse_ui_game_config(const std::string& json_text, const std::string& origin, std::string* err) {
    const std::optional<json> root = detail::parse_json(json_text);
    if (!root || !root->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    if (!check_format(*root, "rg.ui_game/1", origin, err)) return std::nullopt;
    UiGameConfig g;
    const auto section = root->find("section");
    if (section == root->end() || !section->is_object() || !detail::get_string(*section, "id", g.section) ||
        !detail::get_string(*section, "title", g.section_title) || g.section.empty()) {
        fail(err, origin, "needs a \"section\" with \"id\" and \"title\"");
        return std::nullopt;
    }
    const auto setting = root->find("palette_setting");
    if (setting == root->end() || !setting->is_object() || !detail::get_string(*setting, "key", g.palette_setting.key) ||
        !detail::get_string(*setting, "label", g.palette_setting.label)) {
        fail(err, origin, "needs a \"palette_setting\" with \"key\" and \"label\"");
        return std::nullopt;
    }
    detail::get_string(*setting, "help", g.palette_setting.help);
    if (g.palette_setting.key.rfind(g.section + ".", 0) != 0) {
        fail(err, origin, "the palette setting's key must start with \"" + g.section + ".\"");
        return std::nullopt;
    }
    const auto palettes = root->find("palettes");
    if (palettes == root->end() || !palettes->is_array() || palettes->empty()) {
        fail(err, origin, "needs a non-empty \"palettes\" array");
        return std::nullopt;
    }
    for (const json& id : *palettes) {
        if (!id.is_string()) {
            fail(err, origin, "\"palettes\" must list palette ids");
            return std::nullopt;
        }
        g.palettes.push_back(id.get<std::string>());
    }
    if (!detail::get_string(*root, "default_palette", g.default_palette) ||
        std::find(g.palettes.begin(), g.palettes.end(), g.default_palette) == g.palettes.end()) {
        fail(err, origin, "\"default_palette\" must be one of \"palettes\"");
        return std::nullopt;
    }
    return g;
}

namespace {
// A surface as it reaches the eye: the surfaces it sits on, on the scene.
UiColour surface_over_scene(const UiRoleContract& c, const UiPalette& p, const std::string& id, const UiColour& scene) {
    const auto base = c.sits_on.find(id);
    const UiColour below = base == c.sits_on.end() ? scene : surface_over_scene(c, p, base->second, scene);
    const UiColour* own = p.role(id);
    return own == nullptr ? below : ui_composite(*own, below);
}
} // namespace

std::vector<std::string> validate_ui_palette(const UiRoleContract& contract, const UiPalette& palette) {
    std::vector<std::string> problems;
    for (const UiRoleDef& role : contract.roles) {
        if (palette.role(role.id) == nullptr) problems.push_back("role \"" + role.id + "\" has no colour (" + role.meaning + ")");
    }
    for (const auto& [id, colour] : palette.colours) {
        if (!contract.has_role(id)) problems.push_back("role \"" + id + "\" is not in the role list");
    }
    // Rules that read a role skip it when the completeness check above already said it is missing.
    const auto colour_of = [&](const std::string& id) -> const UiColour* {
        const UiColour* c = palette.role(id);
        if (c != nullptr) return c;
        const auto fixed = contract.fixed.find(id);
        return fixed == contract.fixed.end() ? nullptr : &fixed->second;
    };
    for (const UiContrastPair& pair : contract.contrast) {
        const UiColour* fg = colour_of(pair.fg);
        const UiColour* on = palette.role(pair.on);
        if (fg == nullptr || on == nullptr) continue;
        double worst = 21.0;
        std::string worst_scene;
        for (const std::string& scene_name : contract.default_over) {
            const UiColour& scene = contract.fixed.at(scene_name);
            const UiColour surface = surface_over_scene(contract, palette, pair.on, scene);
            const double ratio = ui_contrast_ratio(ui_composite(*fg, surface), surface);
            if (ratio < worst) {
                worst = ratio;
                worst_scene = scene_name;
            }
        }
        if (worst < pair.min_ratio) {
            problems.push_back("\"" + pair.fg + "\" on \"" + pair.on + "\" over " + worst_scene + " has contrast " + number_text(worst) +
                               ", " + pair.contrast_class + " needs " + number_text(pair.min_ratio));
        }
    }
    for (const auto& pair : contract.lightness_pairs) {
        const UiColour* a = palette.role(pair[0]);
        const UiColour* b = palette.role(pair[1]);
        if (a == nullptr || b == nullptr) continue;
        const double difference = std::abs(ui_lightness(*a) - ui_lightness(*b));
        if (difference < contract.min_lightness_difference) {
            problems.push_back("\"" + pair[0] + "\" and \"" + pair[1] + "\" differ by only " + number_text(difference) +
                               " in lightness, they need " + number_text(contract.min_lightness_difference) +
                               " so that they do not rely on hue alone");
        }
    }
    if (std::find(palette.claims.begin(), palette.claims.end(), contract.vision_claim) != palette.claims.end()) {
        for (const auto& pair : contract.vision_pairs) {
            const UiColour* a = palette.role(pair[0]);
            const UiColour* b = palette.role(pair[1]);
            if (a == nullptr || b == nullptr) continue;
            for (const auto& [name, matrix] : contract.vision_matrices) {
                const double difference = ui_delta_e(ui_simulate_vision(*a, matrix), ui_simulate_vision(*b, matrix));
                if (difference < contract.min_vision_delta_e) {
                    problems.push_back("\"" + pair[0] + "\" and \"" + pair[1] + "\" differ by only " + number_text(difference) +
                                       " under simulated " + name + " vision, " + contract.vision_claim + " needs " +
                                       number_text(contract.min_vision_delta_e));
                }
            }
        }
    }
    return problems;
}

const UiPalette* UiTheme::palette(const std::string& id) const {
    for (const UiPalette& p : palettes) {
        if (p.id == id) return &p;
    }
    return nullptr;
}

std::optional<UiTheme> load_ui_theme(const std::string& data_dir, std::string* err) {
    const std::string dir = data_dir + "/ui/";
    const auto read = [&](const std::string& path) -> std::optional<std::string> {
        std::optional<std::string> text = detail::read_text_file(path);
        if (!text) fail(err, path, "cannot read");
        return text;
    };
    const std::optional<std::string> roles_text = read(dir + "roles.json");
    const std::optional<std::string> tokens_text = roles_text ? read(dir + "tokens.json") : std::nullopt;
    const std::optional<std::string> game_text = tokens_text ? read(dir + "game.json") : std::nullopt;
    if (!game_text) return std::nullopt;
    std::optional<UiRoleContract> contract = parse_ui_role_contract(*roles_text, dir + "roles.json", err);
    if (!contract) return std::nullopt;
    std::optional<UiTokens> tokens = parse_ui_tokens(*tokens_text, dir + "tokens.json", err);
    if (!tokens) return std::nullopt;
    std::optional<UiGameConfig> game = parse_ui_game_config(*game_text, dir + "game.json", err);
    if (!game) return std::nullopt;

    UiTheme theme;
    theme.contract = std::move(*contract);
    theme.tokens = std::move(*tokens);
    theme.game = std::move(*game);
    for (const std::string& id : theme.game.palettes) {
        const std::string path = dir + "palettes/" + id + ".json";
        std::string palette_err;
        const std::optional<std::string> text = detail::read_text_file(path);
        std::optional<UiPalette> palette = text ? parse_ui_palette(*text, path, &palette_err) : std::nullopt;
        if (!text) palette_err = path + ": cannot read";
        if (!palette) {
            theme.refused.push_back({id, {palette_err}});
            continue;
        }
        if (palette->id != id) {
            theme.refused.push_back({id, {path + ": \"id\" is \"" + palette->id + "\", game.json lists \"" + id + "\""}});
            continue;
        }
        std::vector<std::string> problems = validate_ui_palette(theme.contract, *palette);
        if (!problems.empty()) {
            theme.refused.push_back({id, std::move(problems)});
            continue;
        }
        theme.palettes.push_back(std::move(*palette));
    }
    if (theme.palettes.empty()) {
        fail(err, dir, "no palette passed the role contract, the first refusal: " +
                           (theme.refused.empty() || theme.refused.front().problems.empty()
                                ? std::string("none listed") : theme.refused.front().id + ": " + theme.refused.front().problems.front()));
        return std::nullopt;
    }
    theme.default_palette = theme.palette(theme.game.default_palette) != nullptr ? theme.game.default_palette : theme.palettes.front().id;
    return theme;
}

void install_ui_theme_settings(const UiTheme& theme) {
    SettingDef def = theme.game.palette_setting;
    def.section = theme.game.section;
    def.type = SettingType::Choice;
    def.default_value = theme.default_palette;
    for (const UiPalette& palette : theme.palettes) def.choices.push_back(palette.id);
    extend_settings_schema({def}, theme.game.section, theme.game.section_title);
}

} // namespace rg
