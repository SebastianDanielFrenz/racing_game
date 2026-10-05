// test_credits.cpp - rg::Credits (R5): strict loader, the shipped
// data/credits.json, the attribution line, the section view-model and the
// coverage check against geo2map's pipeline data-sources table.
#include "rg/credits.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

const std::string kRoot = RG_SOURCE_DIR;

std::string valid_entry(const std::string& id, const std::string& extra = "") {
    return R"({"id":")" + id + R"(","kind":"data","name":"N","provider":"P","licence":"L",)" +
           R"("attribution_required":false)" + extra + "}";
}

std::string doc(const std::string& entries, const std::string& fmt = "rg.credits/1") {
    return R"({"format":")" + fmt + R"(","entries":[)" + entries + "]}";
}

bool rejects(const std::string& text, const std::string& needle) {
    std::string err;
    const auto c = rg::parse_credits(text, "t", &err);
    return !c.has_value() && err.find(needle) != std::string::npos;
}

std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

TEST_CASE("credits: a minimal valid document loads", "[credits]") {
    std::string err;
    const auto c = rg::parse_credits(doc(valid_entry("a")), "t", &err);
    REQUIRE(c.has_value());
    CHECK(c->entries.size() == 1);
    CHECK(c->entries[0].in_game);
    CHECK(rg::attribution_line(*c).empty());
}

TEST_CASE("credits: loader rejections", "[credits]") {
    CHECK(rejects("not json", "not a JSON object"));
    CHECK(rejects("[]", "not a JSON object"));
    CHECK(rejects(doc(valid_entry("a"), "rg.credits/2"), "format"));
    CHECK(rejects(R"({"format":"rg.credits/1"})", "entries"));
    CHECK(rejects(doc("3"), "not an object"));
    CHECK(rejects(doc(valid_entry("a") + "," + valid_entry("a")), "duplicate id"));
    CHECK(rejects(doc(R"({"kind":"data","name":"N","provider":"P","licence":"L","attribution_required":false})"),
                  "\"id\" missing"));
    CHECK(rejects(doc(R"({"id":"a","kind":"music","name":"N","provider":"P","licence":"L","attribution_required":false})"),
                  "kind"));
    CHECK(rejects(doc(R"({"id":"a","kind":"data","name":"N","provider":"P","attribution_required":false})"),
                  "required strings"));
    CHECK(rejects(doc(R"({"id":"a","kind":"data","name":"N","provider":"P","licence":"L"})"), "attribution_required"));
    // Required attribution with no text.
    CHECK(rejects(doc(R"({"id":"a","kind":"data","name":"N","provider":"P","licence":"L","attribution_required":true})"),
                  "attribution_text is empty"));
    // On the attribution line without a text.
    CHECK(rejects(doc(valid_entry("a", R"(,"on_attribution_line":true)")), "on_attribution_line"));
    CHECK(rejects(doc(valid_entry("a", R"(,"geo2map_datasets":"x")")), "geo2map_datasets"));
    CHECK(rejects(doc(valid_entry("a", R"(,"geo2map_datasets":[1])")), "geo2map_datasets"));
    std::string err;
    CHECK_FALSE(rg::load_credits(kRoot + "/data/does_not_exist.json", &err).has_value());
    CHECK(err.find("cannot read") != std::string::npos);
}

TEST_CASE("credits: attribution line and sections", "[credits]") {
    const std::string text = doc(
        valid_entry("a", R"(,"attribution_text":"AAA","on_attribution_line":true)") + "," +
        valid_entry("b", R"(,"attribution_text":"BBB","on_attribution_line":true,"in_game":false)") + "," +
        valid_entry("c", R"(,"attribution_text":"CCC","on_attribution_line":true)") + "," +
        R"({"id":"s","kind":"software","name":"S","provider":"P","licence":"L","attribution_required":false})");
    std::string err;
    const auto c = rg::parse_credits(text, "t", &err);
    REQUIRE(c.has_value());
    // not-in-game entries stay off the line
    CHECK(rg::attribution_line(*c) == "AAA | CCC");
    const auto sections = rg::credits_sections(*c);
    REQUIRE(sections.size() == 3);
    CHECK(sections[0].title == "Map and terrain data");
    CHECK(sections[0].entries.size() == 2);
    CHECK(sections[1].title == "Software");
    CHECK(sections[2].entries.size() == 1);
    CHECK(sections[2].entries[0]->id == "b");
}

TEST_CASE("credits: shipped data/credits.json", "[credits]") {
    std::string err;
    const auto c = rg::load_credits(kRoot + "/data/credits.json", &err);
    INFO(err);
    REQUIRE(c.has_value());
    const auto find = [&](const std::string& id) -> const rg::CreditEntry* {
        for (const auto& e : c->entries) {
            if (e.id == id) return &e;
        }
        return nullptr;
    };
    // The three datasets the game renders today, with their exact required text.
    const auto* osm = find("osm");
    REQUIRE(osm != nullptr);
    CHECK(osm->attribution_text == "\xC2\xA9 OpenStreetMap contributors");
    CHECK(osm->licence.find("ODbL") != std::string::npos);
    const auto* dgm = find("dgm1");
    REQUIRE(dgm != nullptr);
    CHECK(dgm->attribution_text.find("Hessische Verwaltung f\xC3\xBCr Bodenmanagement und Geoinformation") !=
          std::string::npos);
    const auto* wc = find("worldcover");
    REQUIRE(wc != nullptr);
    CHECK(wc->attribution_text.find("ESA WorldCover project 2021") != std::string::npos);
    // Every required-attribution entry that the game uses is on the line.
    const std::string line = rg::attribution_line(*c);
    CHECK(line.find(osm->attribution_text) != std::string::npos);
    CHECK(line.find(dgm->attribution_text) != std::string::npos);
    CHECK(line.find(wc->attribution_text) != std::string::npos);
    // Software the game links.
    for (const char* id : {"godot", "godot_cpp", "jolt", "nlohmann_json"}) {
        CAPTURE(id);
        CHECK(find(id) != nullptr);
    }
    CHECK(find("photon") != nullptr);
    // Unused-so-far datasets must not claim to be in the game.
    for (const char* id : {"hvbg_lod2", "frankfurt_baumkataster", "thuenen_tree_species"}) {
        const auto* e = find(id);
        REQUIRE(e != nullptr);
        CHECK_FALSE(e->in_game);
        CHECK_FALSE(e->on_attribution_line);
    }
}

TEST_CASE("credits: README table parser", "[credits]") {
    const std::string md =
        "intro\n\n| Dataset | Provider |\n|---|---|\n| First, set | A |\n| **Second** | B |\n\nafter | not | table\n";
    const auto names = rg::parse_geo2map_dataset_names(md);
    REQUIRE(names.size() == 2);
    CHECK(names[0] == "First, set");
    CHECK(names[1] == "**Second**");
    CHECK(rg::parse_geo2map_dataset_names("no table here").empty());

    std::string err;
    const auto c = rg::parse_credits(doc(valid_entry("a", R"(,"geo2map_datasets":["First, set"])")), "t", &err);
    REQUIRE(c.has_value());
    const auto missing = rg::uncovered_geo2map_datasets(*c, names);
    REQUIRE(missing.size() == 1);
    CHECK(missing[0] == "**Second**");
}

// Every source in geo2map's pipeline data-sources table must appear in the
// credits file (R5 acceptance). The README is not part of the pinned
// submodule yet, so the check reads the first copy that exists: the
// submodule's own, else the sibling working copy S:/claude_code/geo2map_engine
// (read-only). SKIPs when neither is there.
TEST_CASE("credits: every geo2map pipeline data source is covered", "[credits]") {
    namespace fs = std::filesystem;
    const std::vector<std::string> candidates = {
        kRoot + "/external/geo2map_engine/docs/pipeline/README.md",
        kRoot + "/../geo2map_engine/docs/pipeline/README.md",
    };
    std::string md;
    std::string used;
    for (const std::string& p : candidates) {
        if (!fs::exists(p)) continue;
        const std::string t = read_file(p);
        if (!rg::parse_geo2map_dataset_names(t).empty()) {
            md = t;
            used = p;
            break;
        }
    }
    if (md.empty()) {
        SKIP("geo2map pipeline README with a data-sources table not found");
    }
    INFO("README: " << used);
    std::string err;
    const auto c = rg::load_credits(kRoot + "/data/credits.json", &err);
    REQUIRE(c.has_value());
    const auto datasets = rg::parse_geo2map_dataset_names(md);
    CHECK(datasets.size() >= 12);
    const auto missing = rg::uncovered_geo2map_datasets(*c, datasets);
    for (const auto& m : missing) UNSCOPED_INFO("not in credits.json: " << m);
    CHECK(missing.empty());
}
