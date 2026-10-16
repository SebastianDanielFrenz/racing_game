// test_strip_visual.cpp - rg/strip_visual.h: the data that maps weight-reduction choices to model nodes to hide
// (data/vehicles/strip_visual.json), its strict loader, and the garage / drive hand-over.
#include "rg/garage.h"
#include "rg/strip_visual.h"
#include "rg/vehicle_catalog.h"
#include "rg/vehicle_setup.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <set>
#include <iterator>
#include <optional>
#include <string>

namespace {
namespace fs = std::filesystem;
using Catch::Matchers::ContainsSubstring;
const std::string kRoot = RG_SOURCE_DIR;

bool contains(const std::vector<std::string>& v, const std::string& s) { return std::find(v.begin(), v.end(), s) != v.end(); }

const char* kSmall = R"({
  "format": "rg.strip_visual/1",
  "models": {
    "m": {
      "rules": [
        {"slot": "audio", "part": "removed", "hide": ["head_unit", "knob"]},
        {"slot": "door_trim", "part": "removed", "hide": ["card_L", "knob"]},
        {"slot": "seat_passenger", "part": "removed", "hide": ["seat_p"], "after_regeneration": true}
      ],
      "not_separate": {"dashboard": "body mesh"}
    },
    "empty": {}
  }
})";

struct TempDirs {
    std::string base;
    TempDirs() {
        base = (fs::temp_directory_path() /
                ("rg_strip_visual_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count() % 1000000000)))
                   .generic_string();
        fs::create_directories(base + "/user");
        fs::create_directories(base + "/work");
    }
    ~TempDirs() {
        std::error_code ec;
        fs::remove_all(fs::path(base), ec);
    }
};

rg::GarageConfig config_for(const TempDirs& t) {
    rg::GarageConfig c;
    c.repo_root = kRoot;
    c.catalog_path = kRoot + "/data/vehicles/catalog.json";
    c.options_path = kRoot + "/data/vehicles/setup_options.json";
    c.set_path = kRoot + "/data/garage/garage_set.json";
    c.tyre_families_path = kRoot + "/data/tyres/families.json";
    c.user_dir = t.base + "/user";
    c.work_root = t.base + "/work";
    c.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
    return c;
}

rg::StripVisualTable shipped() {
    std::string err;
    auto t = rg::load_strip_visual(kRoot + "/data/vehicles/strip_visual.json", &err);
    INFO(err);
    REQUIRE(t.has_value());
    return *t;
}

std::optional<std::string> read_text(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return std::nullopt;
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

std::set<std::string> rig_nodes(const std::string& glb_path) {
    std::set<std::string> out;
    const std::string rig = glb_path.substr(0, glb_path.size() - 4) + ".rig.json";
    const auto text = read_text(rig);
    if (!text) return out;
    const auto doc = nlohmann::json::parse(*text, nullptr, /*allow_exceptions=*/false);
    if (doc.is_discarded() || !doc.is_object() || !doc.contains("nodes")) return out;
    for (const auto& n : doc["nodes"]) {
        if (n.is_object() && n.contains("name") && n["name"].is_string()) out.insert(n["name"].get<std::string>());
    }
    return out;
}
} // namespace

TEST_CASE("strip visual: a fitted part hides exactly its rule's nodes", "[strip_visual]") {
    std::string err;
    const auto t = rg::parse_strip_visual(kSmall, "inline", &err);
    INFO(err);
    REQUIRE(t.has_value());
    CHECK(rg::model_stem("/a/b/data/models/pico/pico.glb") == "pico");
    CHECK(t->hidden_nodes("m", {}).empty());
    CHECK(t->hidden_nodes("m", {{"audio", "stock"}}).empty());
    CHECK(t->hidden_nodes("m", {{"audio", "lightweight"}}).empty()); // another part id of the slot: no rule
    CHECK(t->hidden_nodes("m", {{"audio", "removed"}}) == std::vector<std::string>{"head_unit", "knob"});
    // two rules naming one node: once, sorted
    CHECK(t->hidden_nodes("m", {{"door_trim", "removed"}, {"audio", "removed"}}) ==
          std::vector<std::string>{"card_L", "head_unit", "knob"});
    // taking one choice back un-hides only what no other fitted choice hides
    CHECK(t->hidden_nodes("m", {{"door_trim", "removed"}}) == std::vector<std::string>{"card_L", "knob"});
    CHECK(t->hidden_nodes("m", {{"seat_passenger", "removed"}}) == std::vector<std::string>{"seat_p"});
    // an unknown model, an empty model: nothing
    CHECK(t->hidden_nodes("other", {{"audio", "removed"}}).empty());
    CHECK(t->hidden_nodes("empty", {{"audio", "removed"}}).empty());
    CHECK(t->controlled_nodes("m") == std::vector<std::string>{"card_L", "head_unit", "knob", "seat_p"});
    REQUIRE(t->find("m") != nullptr);
    CHECK(t->find("m")->not_separate.at("dashboard") == "body mesh");
}

TEST_CASE("strip visual: the loader refuses a malformed file", "[strip_visual]") {
    auto bad = [](const std::string& text, const char* needle) {
        std::string err;
        const auto t = rg::parse_strip_visual(text, "inline", &err);
        INFO(text);
        CHECK(!t.has_value());
        CHECK_THAT(err, ContainsSubstring(needle));
    };
    bad("not json", "not a JSON object");
    bad(R"({"format":"x","models":{}})", "rg.strip_visual/1");
    bad(R"({"format":"rg.strip_visual/1"})", "\"models\"");
    bad(R"({"format":"rg.strip_visual/1","models":{"m":1}})", "must be an object");
    bad(R"({"format":"rg.strip_visual/1","models":{"m":{"rules":[{"part":"removed","hide":["a"]}]}}})", "slot");
    bad(R"({"format":"rg.strip_visual/1","models":{"m":{"rules":[{"slot":"audio","part":"removed","hide":[]}]}}})", "hide");
    bad(R"({"format":"rg.strip_visual/1","models":{"m":{"rules":[{"slot":"audio","part":"removed","hide":[1]}]}}})", "hide");
    bad(R"({"format":"rg.strip_visual/1","models":{"m":{"not_separate":{"audio":3}}}})", "must be a text");
    // a slot with a rule cannot also be declared "not separate" (unless the rule waits for a regenerated model)
    bad(R"({"format":"rg.strip_visual/1","models":{"m":{"rules":[{"slot":"audio","part":"removed","hide":["a"]}],"not_separate":{"audio":"x"}}}})",
        "also listed under not_separate");
    std::string err;
    CHECK(!rg::load_strip_visual(kRoot + "/data/vehicles/does_not_exist.json", &err).has_value());
    CHECK_THAT(err, ContainsSubstring("cannot open"));
}

TEST_CASE("strip visual: the shipped data matches the options, the catalog and the models", "[strip_visual][data]") {
    const rg::StripVisualTable table = shipped();
    std::string err;
    const auto options = rg::load_setup_options(kRoot + "/data/vehicles/setup_options.json", &err);
    INFO(err);
    REQUIRE(options.has_value());
    // slot -> the part ids the part_choice option of that slot offers
    std::map<std::string, std::set<std::string>> offered;
    for (const rg::SetupOptionDef& d : options->options) {
        if (d.kind != rg::OptionKind::PartChoice) continue;
        for (const rg::SetupPart& p : d.parts) offered[d.slot].insert(p.id);
    }
    REQUIRE(offered.size() == 11);
    const auto catalog = rg::load_vehicle_catalog(kRoot + "/data/vehicles/catalog.json", kRoot, &err);
    INFO(err);
    REQUIRE(catalog.has_value());

    std::set<std::string> stems;
    for (const rg::CatalogEntry& e : catalog->entries) {
        if (e.interior.empty() || e.model_path.empty()) continue;
        const std::string stem = rg::model_stem(e.model_path);
        stems.insert(stem);
        INFO(e.id << " / model " << stem);
        const rg::StripModel* m = table.find(stem);
        REQUIRE(m != nullptr);
        // every weight-reduction slot of the car is either hidden by a rule or explained
        for (const auto& slot : e.interior) {
            const bool ruled = std::any_of(m->rules.begin(), m->rules.end(), [&](const rg::StripRule& r) { return r.slot == slot.first; });
            INFO("slot " << slot.first);
            CHECK((ruled || m->not_separate.count(slot.first) != 0));
        }
        // nodes of a rule that is live today exist in the model's rig file
        const std::set<std::string> nodes = rig_nodes(e.model_path);
        for (const rg::StripRule& r : m->rules) {
            const bool known = offered.count(r.slot) != 0 && offered[r.slot].count(r.part) != 0;
            INFO("rule " << r.slot << "/" << r.part);
            CHECK(known);
            if (r.after_regeneration) continue;
            REQUIRE(!nodes.empty());
            for (const std::string& n : r.hide) {
                INFO("node " << n);
                CHECK(nodes.count(n) == 1);
            }
        }
    }
    for (const auto& model : table.models) {
        INFO("model " << model.first);
        CHECK(stems.count(model.first) == 1); // no entry for a model the catalog does not use
        for (const rg::StripRule& r : model.second.rules) CHECK(offered.count(r.slot) == 1);
        for (const auto& ns : model.second.not_separate) CHECK(offered.count(ns.first) == 1);
    }
}

TEST_CASE("strip visual: the generated cars' sources name the nodes the data waits for", "[strip_visual][data]") {
    const rg::StripVisualTable table = shipped();
    for (const auto& [stem, model] : table.models) {
        for (const rg::StripRule& r : model.rules) {
            if (!r.after_regeneration) continue;
            const auto src = read_text(kRoot + "/tools/cargen/cars/" + stem + ".py");
            INFO(stem << " " << r.slot);
            REQUIRE(src.has_value()); // only generated cars may wait for a regenerated model
            for (const std::string& n : r.hide) {
                // the node is created by name ("seat_driver") or by prefix ("door_card_" + ...)
                const std::string prefix = n.substr(0, n.find_last_of('_') + 1);
                INFO("node " << n);
                CHECK((src->find("\"" + n + "\"") != std::string::npos || src->find("\"" + prefix + "\"") != std::string::npos));
            }
        }
    }
}

TEST_CASE("strip visual: the garage hands the hidden nodes to the visual", "[strip_visual][garage]") {
    TempDirs t;
    std::string err;
    auto g = rg::Garage::open(config_for(t), &err);
    INFO(err);
    REQUIRE(g != nullptr);
    REQUIRE(g->begin_edit("car_city", &err));
    CHECK(g->working_hidden_nodes().empty()); // stock
    // lighter parts of the same shape change nothing on screen
    REQUIRE(g->set_option("trim_hood", std::string("carbon")).accepted);
    REQUIRE(g->set_option("trim_fasteners", std::string("titanium")).accepted);
    REQUIRE(g->set_option("trim_driver_seat", std::string("carbon_bucket")).accepted);
    CHECK(g->working_hidden_nodes().empty());
    // taking the audio system out hides the head unit's controls
    REQUIRE(g->set_option("trim_audio", std::string("removed")).accepted);
    auto hidden = g->working_hidden_nodes();
    CHECK(contains(hidden, "screen_center"));
    CHECK(contains(hidden, "btn_wheel_audio_source"));
    CHECK(!contains(hidden, "btn_wheel_phone"));
    CHECK(!contains(hidden, "seat_passenger"));
    // the seats and door cards (nodes of a regenerated model)
    REQUIRE(g->set_option("trim_passenger_seat", std::string("removed")).accepted);
    REQUIRE(g->set_option("trim_rear_seats", std::string("removed")).accepted);
    REQUIRE(g->set_option("trim_door_trim", std::string("removed")).accepted);
    hidden = g->working_hidden_nodes();
    for (const char* n : {"seat_passenger", "seat_rear", "door_card_FL", "door_card_RR"}) CHECK(contains(hidden, n));
    CHECK(!contains(hidden, "seat_driver")); // the driver's seat stays
    // lightweight door cards are thin cards, still there
    REQUIRE(g->set_option("trim_door_trim", std::string("lightweight")).accepted);
    CHECK(!contains(g->working_hidden_nodes(), "door_card_FL"));
    // back to stock for the audio system: its controls come back
    REQUIRE(g->set_option("trim_audio", std::string("stock")).accepted);
    CHECK(!contains(g->working_hidden_nodes(), "screen_center"));
    // a saved setup reaches the drive
    REQUIRE(g->set_option("trim_audio", std::string("removed")).accepted);
    REQUIRE(g->save(&err));
    const rg::DriveSelection d = g->prepare_drive("car_city");
    INFO(d.error);
    REQUIRE(d.ok);
    CHECK(contains(d.hidden_nodes, "screen_center"));
    CHECK(contains(d.hidden_nodes, "seat_passenger"));
    CHECK(!contains(d.hidden_nodes, "door_card_FL"));
    g->cleanup();
    // another car: its model has no node for the audio system, so the choice hides nothing
    REQUIRE(g->begin_edit("car_sedan", &err));
    REQUIRE(g->set_option("trim_audio", std::string("removed")).accepted);
    CHECK(g->working_hidden_nodes().empty());
    g->discard();
}
