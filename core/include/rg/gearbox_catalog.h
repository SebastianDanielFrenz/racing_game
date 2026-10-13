// rg/gearbox_catalog.h - the gearboxes the garage offers (gearbox swap, 2026-10-13). Engine-neutral.
//
// data/gearboxes/gearbox_catalog.json ("rg.gearbox_catalog/1") lists one row per physics_sim gearbox file the
// garage may install: display name, kind (manual | sequential | automatic | cvt), focus (economy | touring |
// performance | race), gear type (helical | straight_cut - the later gear-whine audio keys on it), synchro
// (cone | dog), the torque RATING and a description. Everything else (gears, ratios) is read from the gearbox
// file itself, never typed here.
//
// The rating is the maximum engine torque (N*m at the gearbox input) the box is rated for. physics_sim's
// GearboxDesc has no rating yet, so it lives in this catalogue; it moves into the physics file when the physics
// session adds one (the catalogue field then goes away without touching the gearbox files' other content).
// The garage offers a box only for an engine whose peak torque does not exceed it (check_gearbox_rating).
//
// Sections of the garage list: Manual (kind manual and sequential, the latter labelled "Sequential"),
// Automatic and CVT. Automatic and CVT have no entries until physics_sim models a torque converter and a
// variator; the garage shows them as "coming soon".
#pragma once

#include <memory>
#include <string>
#include <vector>

namespace rg {

enum class GearboxKind { Manual, Sequential, Automatic, Cvt };
const char* to_string(GearboxKind k);

struct GearboxEntry {
    std::string id;           // lower_snake, unique
    std::string name;         // display name
    GearboxKind kind = GearboxKind::Manual;
    std::string focus;        // economy | touring | performance | race
    std::string gear_type;    // helical | straight_cut
    std::string synchro;      // cone | dog
    std::string description;
    double rating_nm = 0.0;   // maximum engine torque the box takes
    std::string file;         // absolute, normalised path of the physics_sim gearbox file
    // Read from the gearbox file:
    int gears = 0;            // forward gears
    double first_ratio = 0.0;
    double top_ratio = 0.0;
};

// One section of the garage list.
struct GearboxSection {
    std::string label;        // "Manual", "Automatic", "CVT"
    bool has_entries = false; // false: coming soon
};

class GearboxCatalog {
public:
    // `catalog_path`: data/gearboxes/gearbox_catalog.json, `repo_root` resolves its relative paths.
    // nullptr + *err on a bad catalogue or a gearbox file that does not read.
    static std::unique_ptr<GearboxCatalog> open(const std::string& catalog_path, const std::string& repo_root, std::string* err);

    [[nodiscard]] const std::vector<GearboxEntry>& list() const { return entries_; }
    [[nodiscard]] const GearboxEntry* find(const std::string& id) const;
    // The entry whose gearbox file is `path` (any spelling), or nullptr.
    [[nodiscard]] const GearboxEntry* find_by_file(const std::string& path) const;
    // Manual, Automatic, CVT in that order; has_entries from the catalogue rows.
    [[nodiscard]] std::vector<GearboxSection> sections() const;

private:
    std::vector<GearboxEntry> entries_;
};

// The section label an entry is listed under ("Manual" for manual and sequential).
std::string gearbox_section(const GearboxEntry& e);

// "" when the box takes `engine_peak_torque_nm` (or the torque is unknown, <= 0), else the reason, e.g.
// "gearbox: rated 450 N*m, engine makes 1371 N*m (Close-ratio 6-speed manual)".
std::string check_gearbox_rating(const GearboxEntry& box, double engine_peak_torque_nm);

// One-line card text: "Sequential · Race · straight-cut" / "6 gears · 2.95 to 0.76 · rated 1800 N*m".
std::string gearbox_summary_line(const GearboxEntry& e);
std::string gearbox_detail_line(const GearboxEntry& e);

} // namespace rg
