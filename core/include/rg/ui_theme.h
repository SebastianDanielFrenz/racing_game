// rg/ui_theme.h — the design language's rules (docs/design_language.md): UI code
// names roles, a palette gives every role a colour, and "any palette works on
// any screen" is checked here, not hoped for.
//
// Engine-neutral. Data:
//   data/ui/roles.json        "rg.ui_roles/1"   the role list and the rules: contrast
//                                               pairs with their classes, the scenes
//                                               behind translucent surfaces, the
//                                               distinctness rules
//   data/ui/tokens.json       "rg.ui_tokens/1"  the palette-independent sizes
//   data/ui/palettes/*.json   "rg.ui_palette/1" one colour per role
//   data/ui/game.json         "rg.ui_game/1"    the palettes this game offers and its default
// A palette that fails is refused with words and never reaches a screen; the
// player setting offers only the accepted ones. Godot's side is thin
// (godot_ext/src/rg_ui_theme.*, game/scripts/ui/ui_theme.gd); a UE5 port reuses
// the data and these rules unchanged.
#pragma once

#include "rg/settings.h"

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace rg {

// sRGB, each channel 0..1, straight alpha.
struct UiColour {
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    double a = 1.0;
    friend bool operator==(const UiColour&, const UiColour&) = default;
};

// "#rrggbb" or "#rrggbbaa"; nullopt on anything else.
std::optional<UiColour> parse_ui_colour(const std::string& text);
// Lower-case "#rrggbbaa".
std::string ui_colour_hex(const UiColour& colour);
// Straight-alpha "over": `top` over an opaque `bottom`; the result is opaque.
UiColour ui_composite(const UiColour& top, const UiColour& bottom);
// WCAG relative luminance of an opaque colour (alpha ignored).
double ui_luminance(const UiColour& colour);
// WCAG contrast ratio of two opaque colours, 1..21.
double ui_contrast_ratio(const UiColour& a, const UiColour& b);
// CIE L* of an opaque colour, 0..100.
double ui_lightness(const UiColour& colour);
// CIE76 Delta E of two opaque colours.
double ui_delta_e(const UiColour& a, const UiColour& b);
// An opaque colour as a viewer with that vision sees it: `matrix` acts on linear sRGB.
using UiVisionMatrix = std::array<std::array<double, 3>, 3>;
UiColour ui_simulate_vision(const UiColour& colour, const UiVisionMatrix& matrix);

struct UiRoleDef {
    std::string id;
    std::string group;
    std::string meaning;
};

struct UiRoleGroup {
    std::string id;
    std::string title;
};

// `fg` (a role or a fixed colour) must keep `min_ratio` against `on` (a role),
// `on` is stacked on what it sits on and on every scene when it is translucent.
struct UiContrastPair {
    std::string fg;
    std::string on;
    std::string contrast_class;
    double min_ratio = 1.0;
};

struct UiRoleContract {
    std::vector<UiRoleGroup> groups;
    std::vector<UiRoleDef> roles; // display order
    std::map<std::string, UiColour> fixed; // scenes and colours no palette sets
    std::vector<std::string> default_over;  // keys of `fixed`
    std::map<std::string, std::string> sits_on; // surface -> the surface always under it, else it lies on the scene
    std::vector<UiContrastPair> contrast;
    double min_lightness_difference = 0.0;
    std::vector<std::array<std::string, 2>> lightness_pairs;
    std::string vision_claim;
    double min_vision_delta_e = 0.0;
    std::vector<std::array<std::string, 2>> vision_pairs;
    std::vector<std::pair<std::string, UiVisionMatrix>> vision_matrices;
    [[nodiscard]] bool has_role(const std::string& id) const;
};

struct UiPalette {
    std::string id;
    std::string label;
    std::string source;
    std::vector<std::string> claims;
    std::map<std::string, UiColour> colours;
    // The role's colour, or nullptr for a role the palette does not give.
    [[nodiscard]] const UiColour* role(const std::string& id) const;
};

// Dotted names ("type.body", "gauge.sweep_start_deg") to design pixels or plain numbers.
struct UiTokens {
    double reference_height_px = 1080.0;
    double min_scale = 0.4;
    std::map<std::string, double> values;
    // The value in design pixels, or nullptr for an unknown token.
    [[nodiscard]] const double* find(const std::string& name) const;
    // value * max(viewport_height / reference_height_px, min_scale); the sizes scale with the screen.
    [[nodiscard]] double px(const std::string& name, double viewport_height) const;
};

struct UiGameConfig {
    std::string section = "interface";
    std::string section_title = "Interface";
    SettingDef palette_setting; // key, label, help; the choices come from the accepted palettes
    std::vector<std::string> palettes;
    std::string default_palette;
};

// Strict, exception-free: nullopt + *err (origin + words) on any error.
std::optional<UiRoleContract> parse_ui_role_contract(const std::string& json_text, const std::string& origin, std::string* err);
std::optional<UiPalette> parse_ui_palette(const std::string& json_text, const std::string& origin, std::string* err);
std::optional<UiTokens> parse_ui_tokens(const std::string& json_text, const std::string& origin, std::string* err);
std::optional<UiGameConfig> parse_ui_game_config(const std::string& json_text, const std::string& origin, std::string* err);

// Every way the palette breaks the contract, one sentence each; empty = accepted.
// Names the missing or unknown role, or the pair with the ratio found and the one required.
std::vector<std::string> validate_ui_palette(const UiRoleContract& contract, const UiPalette& palette);

// A refused palette and why.
struct UiRefusedPalette {
    std::string id;
    std::vector<std::string> problems;
};

struct UiTheme {
    UiRoleContract contract;
    UiTokens tokens;
    UiGameConfig game;
    std::vector<UiPalette> palettes;       // the accepted ones, in game.json's order
    std::vector<UiRefusedPalette> refused;
    std::string default_palette;           // game.json's, or the first accepted one when that one was refused
    [[nodiscard]] const UiPalette* palette(const std::string& id) const;
};

// Loads <data_dir>/ui. Refuses each failing palette (UiTheme::refused) and carries on with
// the rest; nullopt + *err only when the contract, tokens or game file fail or no palette
// survives.
std::optional<UiTheme> load_ui_theme(const std::string& data_dir, std::string* err);

// Adds the palette setting to the schema, offering the accepted palettes, default the
// game's. Call before the Settings is loaded (rg/settings.h extend_settings_schema).
void install_ui_theme_settings(const UiTheme& theme);

} // namespace rg
