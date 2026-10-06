// test_surface_kinds.cpp - S1 / owner O-2: the game's own surface table has a named entry for every geo2map
// surface kind, with the names of physics_sim's surface_conditions classes (wet grip applies to each).

#include "rg/surface_kinds.h"

#include "ps/io/surface_table.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <fstream>
#include <set>
#include <string>

namespace {

const std::string kRepo = RG_SOURCE_DIR;

nlohmann::json read_json(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    REQUIRE(in.good());
    return nlohmann::json::parse(in);
}

} // namespace

TEST_CASE("surface table: the game's surfaces.json loads with every O-2 entry", "[surfaces][s1]") {
    ps::io::SurfaceTable table(kRepo + "/data/surfaces/surfaces.json");
    for (const char* name : {"asphalt", "kerb", "grass", "dirt", "low_mu", "field", "concrete", "paving_stones", "sett",
                             "cobblestone", "gravel", "compacted", "sand"}) {
        INFO("surface " << name);
        CHECK(table.id_for(name) != ps::kInvalidSurfaceId);
    }
    CHECK(table.size() == 13);
    // Ids are alphabetical: names() is sorted.
    const auto& names = table.names();
    CHECK(std::is_sorted(names.begin(), names.end()));
}

TEST_CASE("surface table: every new entry has plausible dry grip and rolling resistance", "[surfaces][s1]") {
    const auto surfaces = read_json(kRepo + "/data/surfaces/surfaces.json").at("surfaces");
    for (const char* name : {"concrete", "paving_stones", "sett", "cobblestone", "gravel", "compacted", "sand"}) {
        INFO("surface " << name);
        const auto& s = surfaces.at(name);
        const double mu = s.at("lambda_mu").get<double>();
        const double crr = s.at("crr").get<double>();
        CHECK(mu > 0.2);
        CHECK(mu <= 1.0); // dry asphalt is the reference 1.0
        CHECK(crr >= 0.010);
        CHECK(crr <= 0.20);
        CHECK(!s.at("source").get<std::string>().empty());
        CHECK(!s.at("display_name").get<std::string>().empty());
    }
    // Ordering the owner's tuning starts from: hard sealed > stone > loose.
    const auto mu = [&](const char* n) { return surfaces.at(n).at("lambda_mu").get<double>(); };
    CHECK(mu("concrete") >= mu("paving_stones"));
    CHECK(mu("paving_stones") > mu("sett"));
    CHECK(mu("sett") > mu("cobblestone"));
    CHECK(mu("cobblestone") > mu("gravel"));
    CHECK(mu("gravel") > mu("sand"));
}

TEST_CASE("surface kinds: every geo2map SurfaceKind maps to a name in the table and in physics_sim's condition classes",
          "[surfaces][s1]") {
    ps::io::SurfaceTable table(kRepo + "/data/surfaces/surfaces.json");
    const auto conditions = read_json(kRepo + "/external/physics_sim/data/surface_conditions/default.json").at("surfaces");

    std::set<std::string> used;
    for (const auto kind : rg::kAllSurfaceKinds) {
        const std::string name(rg::surface_name_for_kind(kind));
        INFO("kind " << static_cast<int>(kind) << " -> " << name);
        CHECK(table.id_for(name) != ps::kInvalidSurfaceId);
        CHECK(conditions.contains(name)); // wet grip works for it
        used.insert(name);
    }
    // The seven O-2 names are all reached by a kind of the same meaning.
    CHECK(rg::surface_name_for_kind(g2m::SurfaceKind::Concrete) == "concrete");
    CHECK(rg::surface_name_for_kind(g2m::SurfaceKind::PavingStones) == "paving_stones");
    CHECK(rg::surface_name_for_kind(g2m::SurfaceKind::Sett) == "sett");
    CHECK(rg::surface_name_for_kind(g2m::SurfaceKind::Cobblestone) == "cobblestone");
    CHECK(rg::surface_name_for_kind(g2m::SurfaceKind::Gravel) == "gravel");
    CHECK(rg::surface_name_for_kind(g2m::SurfaceKind::Compacted) == "compacted");
    CHECK(rg::surface_name_for_kind(g2m::SurfaceKind::Sand) == "sand");
    // An out-of-range value reads as Unknown.
    CHECK(rg::surface_name_for_kind(static_cast<g2m::SurfaceKind>(200)) == "asphalt");
    // The kind list is the whole enum (Unknown = 0 .. Water = 11).
    CHECK(rg::kAllSurfaceKinds.size() == static_cast<std::size_t>(g2m::SurfaceKind::Water) + 1);
}

TEST_CASE("surface table: every physics_sim condition class has a game surface entry", "[surfaces][s1]") {
    ps::io::SurfaceTable table(kRepo + "/data/surfaces/surfaces.json");
    const auto conditions = read_json(kRepo + "/external/physics_sim/data/surface_conditions/default.json").at("surfaces");
    for (auto it = conditions.begin(); it != conditions.end(); ++it) {
        INFO("condition class " << it.key());
        CHECK(table.id_for(it.key()) != ps::kInvalidSurfaceId);
    }
}
