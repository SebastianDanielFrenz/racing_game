// test_ui_theme.cpp — rg/ui_theme.h: the design language's rules. Every shipped
// palette is complete and passes contrast, a broken one is refused with words,
// roles and tokens answer, and a UI script cannot gain a hard-coded colour.
//
// Sabotages recorded (2026-11-01, each run once, undone after, [ui_theme] green again: 122 assertions in 17 cases):
//   S1 ui_theme.cpp validate_ui_palette: `if (worst < pair.min_ratio)` -> `if (false)`: 4 cases fail: "A palette with
//      bad contrast", "Translucent surfaces", "Translucent text", "A surface is judged on what it sits on".
//   S2 ui_theme.cpp surface_over_scene: `return ... : *own` with no scene behind it: "Translucent surfaces" and "A surface
//      is judged on what it sits on" fail.
//   S3 data/ui/palettes/farm.json: "text_muted" alpha down to #fbf7ea30: shipped palettes, the game-default case and "Loading
//      refuses" fail: `"text_muted" on "surface_screen" over bright_field has contrast 1.79, text needs 4.50`.
//   S4 data/ui/palettes/racing.json: "gauge_redline" removed: shipped palettes, the game-default case and "Role and token
//      lookup" fail: `role "gauge_redline" has no colour (The red zone of a dial.)`.
//   S5 game/scripts/hud.gd: `var sabotage_colour := Color(1, 0, 0)` added: "No hard-coded colour" fails.
//   S6 data/ui/colour_allowlist.json: an entry for hud.gd, which holds no colour: "No hard-coded colour" fails on stale.empty().
//   S7 ui_theme.cpp ui_simulate_vision: returns `c` unchanged: "colour_blind_safe is checked" fails.
//   S8 ui_theme.cpp: the `sits_on` lookup replaced by `scene`: 5 cases fail, racing is refused, so the default falls to
//      high_contrast ("high_contrast" == "racing").
//   S9 ui_theme.cpp: the missing-role report off: "A palette with a missing role" and "Loading refuses" fail.
//   S10 ui_theme.cpp: the unknown-role report off: "A palette with an unknown role" fails.
//   S11 ui_theme.cpp: `if (difference < contract.min_lightness_difference)` -> `if (false)`: "Status colours" fails.
//   S12 ui_theme.cpp: text alpha ignored in the ratio: "Translucent text" fails.
//   S13 ui_theme.cpp ui_luminance: green weight 0.7152 -> 0.0722: 7 cases fail, "Colour arithmetic" among them.
//   S14 ui_theme.cpp parse_ui_palette: the format check removed: "Malformed files" fails.
//   S15 data/ui/palettes/high_contrast.json: `"claims": []`: "The high-contrast palette claims" fails.
#include "rg/settings.h"
#include "rg/ui_theme.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace {

const std::string kRoot = RG_SOURCE_DIR;

std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

rg::UiTheme shipped_theme() {
    std::string err;
    auto theme = rg::load_ui_theme(kRoot + "/data", &err);
    INFO(err);
    REQUIRE(theme.has_value());
    return *theme;
}

std::string joined(const std::vector<std::string>& lines) {
    std::string out;
    for (const std::string& line : lines) out += line + "\n";
    return out;
}

const char* kContract = R"({"format": "rg.ui_roles/1",
  "contrast_classes": {"text": 4.5},
  "fixed": {"bright": "#ffffff", "dark": "#000000"},
  "default_over": ["bright", "dark"],
  "lightness_distinct": {"min_lightness_difference": 18, "pairs": [["good", "danger"]]},
  "simulated_vision": {"claim": "colour_blind_safe", "min_delta_e": 14, "pairs": [["good", "danger"]],
    "matrices": {"deutan": [[0.367322, 0.860646, -0.227968], [0.280085, 0.672501, 0.047413], [-0.011820, 0.042940, 0.968881]]}},
  "groups": [{"id": "g", "title": "G", "roles": [
    {"id": "text", "meaning": "Text."}, {"id": "surface", "meaning": "Behind it."},
    {"id": "good", "meaning": "Good."}, {"id": "danger", "meaning": "Danger."}]}],
  "contrast": [{"fg": "text", "class": "text", "on": ["surface"]}]})";

rg::UiRoleContract small_contract() {
    std::string err;
    auto contract = rg::parse_ui_role_contract(kContract, "test", &err);
    INFO(err);
    REQUIRE(contract.has_value());
    return *contract;
}

rg::UiPalette small_palette() {
    rg::UiPalette p;
    p.id = "small";
    p.colours["text"] = *rg::parse_ui_colour("#ffffff");
    p.colours["surface"] = *rg::parse_ui_colour("#000000");
    p.colours["good"] = *rg::parse_ui_colour("#40e060");
    p.colours["danger"] = *rg::parse_ui_colour("#a02010");
    return p;
}

// One line without its comment. A '#' inside a string is not a comment (hex colours in strings stay visible to the scan).
std::string code_part(const std::string& line) {
    bool in_string = false;
    char quote = '"';
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (in_string) {
            if (c == '\\') ++i;
            else if (c == quote) in_string = false;
        } else if (c == '"' || c == '\'') {
            in_string = true;
            quote = c;
        } else if (c == '#') {
            return line.substr(0, i);
        }
    }
    return line;
}

bool is_test_script(const std::string& name) {
    const auto ends = [&](const char* suffix) {
        const std::string s = suffix;
        return name.size() >= s.size() && name.compare(name.size() - s.size(), s.size(), s) == 0;
    };
    return ends("_test.gd") || ends("_shots.gd") || ends("_shot.gd") || ends("_bench.gd") ||
           name.find("_test_") != std::string::npos;
}

} // namespace

TEST_CASE("Report every palette's refusals", "[.][diag][ui_theme]") {
    std::string err;
    auto contract = rg::parse_ui_role_contract(read_file(kRoot + "/data/ui/roles.json"), "roles.json", &err);
    REQUIRE(contract.has_value());
    for (const char* id : {"racing", "farm", "high_contrast"}) {
        const std::string path = kRoot + "/data/ui/palettes/" + id + ".json";
        auto palette = rg::parse_ui_palette(read_file(path), path, &err);
        REQUIRE(palette.has_value());
        for (const std::string& problem : rg::validate_ui_palette(*contract, *palette))
            std::printf("%s: %s\n", id, problem.c_str());
    }
}

TEST_CASE("A surface is judged on what it sits on, not on the scene", "[ui_theme]") {
    std::string err;
    std::string text = kContract;
    const std::string on_scene = R"("default_over": ["bright", "dark"],)";
    const std::size_t at = text.find(on_scene);
    REQUIRE(at != std::string::npos);
    const std::string stacked = text.substr(0, at) + on_scene + R"( "sits_on": {"surface": "good"},)" + text.substr(at + on_scene.size());
    auto contract = rg::parse_ui_role_contract(stacked, "test", &err);
    INFO(err);
    REQUIRE(contract.has_value());
    rg::UiPalette p = small_palette();
    p.colours["text"] = *rg::parse_ui_colour("#ffffff");
    p.colours["surface"] = *rg::parse_ui_colour("#00000040"); // nearly clear: alone it loses to the bright scene
    p.colours["good"] = *rg::parse_ui_colour("#000000");      // the opaque surface it sits on saves it
    p.colours["danger"] = *rg::parse_ui_colour("#808080");
    CHECK(rg::validate_ui_palette(*contract, p).empty());
    CHECK_FALSE(rg::validate_ui_palette(small_contract(), p).empty());

    const std::string circle = text.substr(0, at) + on_scene + R"( "sits_on": {"surface": "good", "good": "surface"},)" + text.substr(at + on_scene.size());
    std::string words;
    CHECK_FALSE(rg::parse_ui_role_contract(circle, "test", &words).has_value());
    CHECK_THAT(words, ContainsSubstring("circle"));
    const std::string unknown = text.substr(0, at) + on_scene + R"( "sits_on": {"surface": "nothing"},)" + text.substr(at + on_scene.size());
    CHECK_FALSE(rg::parse_ui_role_contract(unknown, "test", &words).has_value());
    CHECK_THAT(words, ContainsSubstring("role to a role"));
}

TEST_CASE("Every shipped palette is complete and passes the role contract", "[ui_theme]") {
    const rg::UiTheme theme = shipped_theme();
    REQUIRE(theme.game.palettes.size() >= 3);
    for (const std::string& id : theme.game.palettes) {
        std::string err;
        const std::string path = kRoot + "/data/ui/palettes/" + id + ".json";
        auto palette = rg::parse_ui_palette(read_file(path), path, &err);
        INFO(err);
        REQUIRE(palette.has_value());
        const std::vector<std::string> problems = rg::validate_ui_palette(theme.contract, *palette);
        INFO(id << ":\n" << joined(problems));
        CHECK(problems.empty());
    }
    INFO(theme.refused.size() << " refused");
    CHECK(theme.refused.empty());
    CHECK(theme.palette("racing") != nullptr);
    CHECK(theme.palette("farm") != nullptr);
    CHECK(theme.palette("high_contrast") != nullptr);
}

TEST_CASE("The high-contrast palette claims colour-blind safety and the others do not need to", "[ui_theme]") {
    const rg::UiTheme theme = shipped_theme();
    const rg::UiPalette* high = theme.palette("high_contrast");
    REQUIRE(high != nullptr);
    CHECK(std::find(high->claims.begin(), high->claims.end(), theme.contract.vision_claim) != high->claims.end());
}

TEST_CASE("The game picks its default palette in data and the setting offers the accepted ones", "[ui_theme]") {
    const rg::UiTheme theme = shipped_theme();
    CHECK(theme.default_palette == "racing");
    rg::install_ui_theme_settings(theme);
    const rg::SettingDef* def = rg::find_setting(theme.game.palette_setting.key);
    REQUIRE(def != nullptr);
    CHECK(def->type == rg::SettingType::Choice);
    CHECK(def->choices == std::vector<std::string>{"racing", "farm", "high_contrast"});
    CHECK(std::get<std::string>(def->default_value) == "racing");
    CHECK(def->applies_live);
}

TEST_CASE("Role and token lookup", "[ui_theme]") {
    const rg::UiTheme theme = shipped_theme();
    const rg::UiPalette* racing = theme.palette("racing");
    REQUIRE(racing != nullptr);
    const rg::UiColour* accent = racing->role("accent");
    REQUIRE(accent != nullptr);
    CHECK(rg::ui_colour_hex(*accent) == "#e8732bff");
    CHECK(racing->role("no_such_role") == nullptr);
    REQUIRE(theme.tokens.find("type.body") != nullptr);
    CHECK(*theme.tokens.find("type.body") == Approx(16.0));
    CHECK(theme.tokens.find("type.nothing") == nullptr);
    CHECK(theme.tokens.px("type.body", 1080.0) == Approx(16.0));
    CHECK(theme.tokens.px("type.body", 2160.0) == Approx(32.0));
    // A small window stops shrinking at min_scale.
    CHECK(theme.tokens.px("type.body", 216.0) == Approx(16.0 * theme.tokens.min_scale));
}

TEST_CASE("Colour arithmetic", "[ui_theme]") {
    CHECK_FALSE(rg::parse_ui_colour("red").has_value());
    CHECK_FALSE(rg::parse_ui_colour("#12345").has_value());
    CHECK_FALSE(rg::parse_ui_colour("#12345g").has_value());
    const rg::UiColour half = *rg::parse_ui_colour("#ffffff80");
    CHECK(half.a == Approx(128.0 / 255.0));
    CHECK(rg::ui_colour_hex(half) == "#ffffff80");
    CHECK(rg::ui_contrast_ratio({1, 1, 1, 1}, {0, 0, 0, 1}) == Approx(21.0));
    CHECK(rg::ui_contrast_ratio({0, 0, 0, 1}, {1, 1, 1, 1}) == Approx(21.0));
    // Cited: #767676 is the lightest grey that passes 4.5:1 on white.
    CHECK(rg::ui_contrast_ratio(*rg::parse_ui_colour("#767676"), {1, 1, 1, 1}) == Approx(4.54).margin(0.01));
    const rg::UiColour over = rg::ui_composite({1, 1, 1, 0.5}, {0, 0, 0, 1});
    CHECK(over.r == Approx(0.5));
    CHECK(over.a == Approx(1.0));
    CHECK(rg::ui_lightness({1, 1, 1, 1}) == Approx(100.0).margin(0.01));
    CHECK(rg::ui_lightness({0, 0, 0, 1}) == Approx(0.0).margin(0.01));
}

TEST_CASE("A palette with a missing role is refused with words", "[ui_theme]") {
    const rg::UiRoleContract contract = small_contract();
    rg::UiPalette p = small_palette();
    p.colours.erase("text");
    const std::vector<std::string> problems = rg::validate_ui_palette(contract, p);
    REQUIRE(problems.size() == 1);
    CHECK_THAT(problems[0], ContainsSubstring("\"text\" has no colour"));
    CHECK_THAT(problems[0], ContainsSubstring("Text."));
}

TEST_CASE("A palette with an unknown role is refused with words", "[ui_theme]") {
    const rg::UiRoleContract contract = small_contract();
    rg::UiPalette p = small_palette();
    p.colours["txet"] = rg::UiColour{};
    const std::vector<std::string> problems = rg::validate_ui_palette(contract, p);
    REQUIRE(problems.size() == 1);
    CHECK_THAT(problems[0], ContainsSubstring("\"txet\" is not in the role list"));
}

TEST_CASE("A palette with bad contrast is refused with the pair and the numbers", "[ui_theme]") {
    const rg::UiRoleContract contract = small_contract();
    rg::UiPalette p = small_palette();
    p.colours["text"] = *rg::parse_ui_colour("#303030");
    const std::vector<std::string> problems = rg::validate_ui_palette(contract, p);
    REQUIRE(problems.size() == 1);
    CHECK_THAT(problems[0], ContainsSubstring("\"text\" on \"surface\""));
    CHECK_THAT(problems[0], ContainsSubstring("has contrast 1.5"));
    CHECK_THAT(problems[0], ContainsSubstring("text needs 4.50"));
}

TEST_CASE("Translucent surfaces are judged over the scene, the worst one counts", "[ui_theme]") {
    const rg::UiRoleContract contract = small_contract();
    rg::UiPalette p = small_palette();
    CHECK(rg::validate_ui_palette(contract, p).empty());
    // White text on a nearly clear black surface reads at night and vanishes over the bright scene.
    p.colours["surface"] = *rg::parse_ui_colour("#00000020");
    const std::vector<std::string> problems = rg::validate_ui_palette(contract, p);
    REQUIRE(problems.size() == 1);
    CHECK_THAT(problems[0], ContainsSubstring("over bright"));
}

TEST_CASE("Translucent text is judged at its real strength", "[ui_theme]") {
    const rg::UiRoleContract contract = small_contract();
    rg::UiPalette p = small_palette();
    p.colours["text"] = *rg::parse_ui_colour("#ffffff30");
    const std::vector<std::string> problems = rg::validate_ui_palette(contract, p);
    REQUIRE(problems.size() == 1);
    CHECK_THAT(problems[0], ContainsSubstring("has contrast"));
}

TEST_CASE("Status colours may not rely on hue alone", "[ui_theme]") {
    const rg::UiRoleContract contract = small_contract();
    rg::UiPalette p = small_palette();
    // Same lightness, opposite hue: the classic red against green.
    p.colours["good"] = *rg::parse_ui_colour("#2f9e44");
    p.colours["danger"] = *rg::parse_ui_colour("#e8590c");
    const std::vector<std::string> problems = rg::validate_ui_palette(contract, p);
    REQUIRE(problems.size() == 1);
    CHECK_THAT(problems[0], ContainsSubstring("do not rely on hue alone"));
}

TEST_CASE("colour_blind_safe is checked under simulated vision, only when claimed", "[ui_theme]") {
    rg::UiRoleContract contract = small_contract();
    contract.lightness_pairs.clear(); // only the claim's rule speaks here
    rg::UiPalette p = small_palette();
    // A green and a red of one lightness: plainly apart for most eyes, nearly the same under deutan vision.
    p.colours["good"] = *rg::parse_ui_colour("#6f8f2a");
    p.colours["danger"] = *rg::parse_ui_colour("#b5651d");
    REQUIRE(rg::ui_delta_e(p.colours["good"], p.colours["danger"]) > contract.min_vision_delta_e);
    CHECK(rg::validate_ui_palette(contract, p).empty());
    p.claims = {"colour_blind_safe"};
    const std::vector<std::string> problems = rg::validate_ui_palette(contract, p);
    REQUIRE(problems.size() == 1);
    CHECK_THAT(problems[0], ContainsSubstring("under simulated deutan vision"));
}

TEST_CASE("Loading refuses a broken palette, keeps the others and says why", "[ui_theme]") {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "rg_ui_theme_test";
    fs::remove_all(dir);
    fs::create_directories(dir / "ui" / "palettes");
    fs::copy_file(kRoot + "/data/ui/roles.json", dir / "ui" / "roles.json");
    fs::copy_file(kRoot + "/data/ui/tokens.json", dir / "ui" / "tokens.json");
    fs::copy_file(kRoot + "/data/ui/game.json", dir / "ui" / "game.json");
    fs::copy_file(kRoot + "/data/ui/palettes/farm.json", dir / "ui" / "palettes" / "farm.json");
    fs::copy_file(kRoot + "/data/ui/palettes/high_contrast.json", dir / "ui" / "palettes" / "high_contrast.json");
    // racing.json, the game's default, loses a role.
    nlohmann::json racing = nlohmann::json::parse(read_file(kRoot + "/data/ui/palettes/racing.json"));
    racing["colours"].erase("danger");
    std::ofstream(dir / "ui" / "palettes" / "racing.json") << racing.dump();

    std::string err;
    auto theme = rg::load_ui_theme(dir.string(), &err);
    INFO(err);
    REQUIRE(theme.has_value());
    CHECK(theme->palette("racing") == nullptr);
    CHECK(theme->palettes.size() == 2);
    REQUIRE(theme->refused.size() == 1);
    CHECK(theme->refused[0].id == "racing");
    CHECK_THAT(joined(theme->refused[0].problems), ContainsSubstring("\"danger\" has no colour"));
    CHECK(theme->default_palette == "farm"); // the first accepted one stands in

    // No palette left: nothing to run on, refused with words.
    fs::remove(dir / "ui" / "palettes" / "farm.json");
    fs::remove(dir / "ui" / "palettes" / "high_contrast.json");
    err.clear();
    CHECK_FALSE(rg::load_ui_theme(dir.string(), &err).has_value());
    CHECK_THAT(err, ContainsSubstring("no palette passed"));
    fs::remove_all(dir);
}

TEST_CASE("Malformed files are refused with words", "[ui_theme]") {
    std::string err;
    CHECK_FALSE(rg::parse_ui_palette("{", "p", &err).has_value());
    CHECK_THAT(err, ContainsSubstring("not a JSON object"));
    CHECK_FALSE(rg::parse_ui_palette(R"({"format": "rg.ui_palette/2"})", "p", &err).has_value());
    CHECK_THAT(err, ContainsSubstring("rg.ui_palette/1"));
    CHECK_FALSE(rg::parse_ui_palette(R"({"format": "rg.ui_palette/1", "id": "a", "label": "A", "source": "s",
        "colours": {"text": "white"}})", "p", &err).has_value());
    CHECK_THAT(err, ContainsSubstring("role \"text\" is not #rrggbb"));
    CHECK_FALSE(rg::parse_ui_role_contract(R"({"format": "rg.ui_roles/1"})", "r", &err).has_value());
    CHECK_THAT(err, ContainsSubstring("groups"));
    CHECK_FALSE(rg::parse_ui_tokens(R"({"format": "rg.ui_tokens/1", "reference_height_px": 0, "min_scale": 1})", "t", &err).has_value());
    CHECK_THAT(err, ContainsSubstring("reference_height_px"));
    CHECK_FALSE(rg::parse_ui_game_config(R"({"format": "rg.ui_game/1", "section": {"id": "ui", "title": "UI"},
        "palette_setting": {"key": "ui.palette", "label": "P"}, "palettes": ["a"], "default_palette": "b"})", "g", &err).has_value());
    CHECK_THAT(err, ContainsSubstring("default_palette"));
}

TEST_CASE("No hard-coded colour in a UI script", "[ui_theme]") {
    namespace fs = std::filesystem;
    const nlohmann::json allow = nlohmann::json::parse(read_file(kRoot + "/data/ui/colour_allowlist.json"));
    std::set<std::string> allowed_files;
    for (const auto& entry : allow.at("files")) {
        REQUIRE_FALSE(entry.at("why").get<std::string>().empty());
        allowed_files.insert(entry.at("path").get<std::string>());
    }
    struct AllowedLine {
        std::string path;
        std::string contains;
        bool used = false;
    };
    std::vector<AllowedLine> allowed_lines;
    for (const auto& entry : allow.at("lines")) {
        REQUIRE_FALSE(entry.at("why").get<std::string>().empty());
        allowed_lines.push_back({entry.at("path").get<std::string>(), entry.at("contains").get<std::string>()});
    }

    const std::regex colour_literal(R"(\bColor\s*\(|\bColor\.[A-Za-z_]+|\bColor8\s*\(|\[color=|\[bgcolor=)");
    std::set<std::string> files_with_colours;
    std::vector<std::string> found;
    int scanned = 0;
    for (const auto& entry : fs::recursive_directory_iterator(kRoot + "/game/scripts")) {
        if (!entry.is_regular_file() || entry.path().extension() != ".gd") continue;
        const std::string name = entry.path().filename().string();
        if (is_test_script(name)) continue;
        const std::string relative = fs::relative(entry.path(), kRoot + "/game/scripts").generic_string();
        ++scanned;
        std::ifstream in(entry.path());
        std::string line;
        int number = 0;
        while (std::getline(in, line)) {
            ++number;
            const std::string code = code_part(line);
            for (auto it = std::sregex_iterator(code.begin(), code.end(), colour_literal); it != std::sregex_iterator(); ++it) {
                if (it->str() == "Color.TRANSPARENT") continue; // no colour at all
                files_with_colours.insert(relative);
                if (allowed_files.count(relative) != 0) continue;
                bool allowed = false;
                for (AllowedLine& a : allowed_lines) {
                    if (a.path == relative && code.find(a.contains) != std::string::npos) {
                        a.used = true;
                        allowed = true;
                    }
                }
                if (!allowed) found.push_back(relative + ":" + std::to_string(number) + ": " + line);
                break;
            }
        }
    }
    REQUIRE(scanned > 50); // the scan really walked the scripts
    INFO("A UI colour is a role of data/ui/roles.json: UiTheme.role(&\"...\"). World colours go on the allow-list data/ui/colour_allowlist.json.\n"
         << joined(found));
    CHECK(found.empty());

    std::vector<std::string> stale;
    for (const std::string& path : allowed_files) {
        if (files_with_colours.count(path) == 0) stale.push_back(path + " (no colour left in it)");
    }
    for (const AllowedLine& a : allowed_lines) {
        if (!a.used) stale.push_back(a.path + ": \"" + a.contains + "\" (no such line)");
    }
    INFO("Stale allow-list entries:\n" << joined(stale));
    CHECK(stale.empty());
}
