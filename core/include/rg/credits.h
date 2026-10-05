// rg/credits.h — rg::Credits: the data-driven attribution list (PLAN.md R5,
// 11.2 "Credits"). Loaded from data/credits.json (format "rg.credits/1"), so
// a new data source or library is a data edit plus the unit test that every
// source of geo2map's pipeline data-sources table is covered
// (tests/unit/test_credits.cpp). Engine-neutral: the Godot layer only
// draws what attribution_line() and credits_sections() return.
//
// An entry's `attribution_text` is the exact text its licence requires, copied
// from geo2map_engine's docs/pipeline/README.md data-sources table and
// geo2map_data/SOURCES.md (read-only references). "(c)" in those files is
// written with the real copyright sign here; nothing else is reworded.
#pragma once

#include <optional>
#include <string>
#include <vector>

namespace rg {

enum class CreditKind { Data, Service, Software, Assets };

struct CreditEntry {
    std::string id;           // unique, lower_snake
    CreditKind kind = CreditKind::Data;
    std::string name;         // what the player sees as the entry title
    std::string provider;
    std::string licence;      // licence name
    std::string licence_url;  // may be empty
    // The exact text the licence requires (empty only when attribution is
    // optional AND none is kept). Shown in the credits screen.
    std::string attribution_text;
    bool attribution_required = false;
    // Data entries: true when the game shows or derives from the data today.
    // false = downloaded/prepared for a planned feature, listed in its own
    // credits section so the list never claims more than the game does.
    bool in_game = true;
    // Exact "Dataset" cell values of geo2map's docs/pipeline/README.md
    // data-sources table this entry covers (empty for non-geo2map entries).
    std::vector<std::string> geo2map_datasets;
    // Included in the one-line attribution shown on the boot splash, the
    // loading screen and under the minimap.
    bool on_attribution_line = false;
    std::string note; // free text, optional
};

struct Credits {
    std::string format; // "rg.credits/1"
    std::vector<CreditEntry> entries;
};

struct CreditsSection {
    std::string title;
    std::vector<const CreditEntry*> entries; // pointers into the Credits they came from
};

// Strict loader. nullopt + *err ("<path>: <problem>") on a missing file, bad
// JSON, wrong format string, a missing/mistyped field, an unknown kind, a
// duplicate id, or `attribution_required` with an empty text.
std::optional<Credits> load_credits(const std::string& path, std::string* err);
std::optional<Credits> parse_credits(const std::string& json_text, const std::string& origin, std::string* err);

// The attribution line: the attribution_text of every on_attribution_line &&
// in_game entry in file order, joined by " | ".
std::string attribution_line(const Credits& credits);

// The credits screen view-model, in display order: "Map and terrain data"
// (in-game Data), "Services" (in-game Service), "Software", "Assets", then
// "Prepared for upcoming features (not used in the game yet)" (every
// !in_game entry). Empty sections are omitted.
std::vector<CreditsSection> credits_sections(const Credits& credits);

const char* credit_kind_name(CreditKind kind);

// Extracts the "Dataset" column of the data-sources table of a geo2map
// pipeline README (the markdown table whose header row starts with
// "| Dataset |"). Used by the coverage unit test; returns an empty vector
// when no such table is found.
std::vector<std::string> parse_geo2map_dataset_names(const std::string& readme_markdown);

// Names of geo2map datasets (README table "Dataset" cells) that no entry's
// geo2map_datasets list contains.
std::vector<std::string> uncovered_geo2map_datasets(const Credits& credits, const std::vector<std::string>& datasets);

} // namespace rg
