// rg/gearbox_catalog.h - the gearboxes the garage offers (gearbox swap, 2026-10-13). Engine-neutral.
//
// data/gearboxes/gearbox_catalog.json ("rg.gearbox_catalog/1") lists one row per physics_sim gearbox file the
// garage may install: display name, kind (manual | sequential | automatic | dual_clutch | cvt), focus (economy |
// touring | performance | race), gear type (helical | straight_cut - the later gear-whine audio keys on it),
// synchro (cone | dog | none - "none" for the clutch-to-clutch kinds, which shift on wet clutches), the torque
// RATING and a description. Everything else (gears, ratios) is read from the gearbox
// file itself, never typed here.
//
// The rating is the maximum engine torque (N*m at the gearbox input) the box is rated for. physics_sim's
// GearboxDesc has no rating yet, so it lives in this catalogue; it moves into the physics file when the physics
// session adds one (the catalogue field then goes away without touching the gearbox files' other content).
// The garage offers a box only for an engine whose peak torque does not exceed it (check_gearbox_rating).
//
// Sections of the garage list: Manual (kind manual and sequential, the latter labelled "Sequential"),
// Automatic (torque converter), Dual-clutch and CVT. physics_sim models all three (gearbox types "automatic" /
// "dual_clutch" / "cvt", controller automatic_tcu; a cvt launches through a torque converter or its forward
// clutch, chosen by the file). A section without rows shows as "coming soon". A box's kind must match its file's
// type. A cvt file has no forward_ratios: its virtual steps (cvt.steps, used in S and with the paddles) count as
// the gears and first/top ratio are the variator's ratio_low/ratio_high.
#pragma once

#include <memory>
#include <string>
#include <vector>

namespace rg {

enum class GearboxKind { Manual, Sequential, Automatic, DualClutch, Cvt };
// True for the kinds that shift clutch-to-clutch under an automatic_tcu (no clutch pedal).
bool is_clutch_to_clutch(GearboxKind k);
const char* to_string(GearboxKind k);

struct GearboxEntry {
    std::string id;           // lower_snake, unique
    std::string name;         // display name
    GearboxKind kind = GearboxKind::Manual;
    std::string focus;        // economy | touring | performance | race
    std::string gear_type;    // helical | straight_cut
    std::string synchro;      // cone | dog | none
    std::string description;
    double rating_nm = 0.0;   // maximum engine torque the box takes
    std::string file;         // absolute, normalised path of the physics_sim gearbox file
    // Read from the gearbox file:
    int gears = 0;            // forward gears (cvt: the virtual steps)
    double first_ratio = 0.0; // cvt: ratio_low
    double top_ratio = 0.0;   // cvt: ratio_high
    bool converter = false;   // the file has a torque converter (automatic always, cvt optional)
    // Automatic / dual-clutch / cvt only: the row's "tcu" object (JSON text) - the automatic_tcu fields that belong to
    // the box (min_time_in_gear_s, kickdown_throttle, manual_timeout_s, upshift_torque_fraction, creep_torque_nm, the
    // lockup enabled/min_gear/max_throttle of a converter box, and for a cvt "cvt": {"stepped_sport"}). The
    // engine-dependent rpm points are derived on install (derive_automatic_tcu_points); ids are filled in by the
    // installer.
    std::string tcu_json;
};

// The engine-dependent rpm points of an automatic_tcu, derived from the installed engine's idle and limiter rpm
// (band = limiter - idle; every factor [AS], rounded to 50 rpm):
//   schedule        up 0.20 band above idle at zero throttle, limiter - 400 at full throttle;
//                   down idle + 0.05 band (at least idle + 250) at zero, idle + 0.40 band at full throttle;
//   sport_schedule  up idle + 0.33 band / limiter - 250, down idle + 0.17 band / idle + 0.53 band;
//   kickdown_max_rpm limiter - 600; lock-up engage idle + 0.10 band (at least idle + 400), release idle + 0.05 band
//                   (at least idle + 250, below engage);
//   dual-clutch launch (also a cvt without converter): bite idle + 200, full engagement from idle + 0.12 band (at
//                   least bite + 400) to idle + 0.45 band, anti-stall 0.75 x idle, lock slip 100 rpm;
//   cvt ratio law:  target engine rpm vs throttle 0 / 0.2 / 0.5 / 0.8 / 1 at idle + 0.09 / 0.16 / 0.32 / 0.60 band
//                   and limiter - 0.15 band; minimum rpm vs gearbox output rpm 0 / 2000 / 6000 at idle + 0.07 /
//                   0.12 / 0.25 band (an idle 800 / limiter 6500 engine gets physics_sim's sedan CVT fixture values).
// ok = false (and every value 0) when limiter <= idle + 1000.
struct AutomaticTcuPoints {
    bool ok = false;
    double up_zero = 0.0, up_full = 0.0, down_zero = 0.0, down_full = 0.0;
    double sport_up_zero = 0.0, sport_up_full = 0.0, sport_down_zero = 0.0, sport_down_full = 0.0;
    double kickdown_max_rpm = 0.0;
    double lockup_engage_rpm = 0.0, lockup_release_rpm = 0.0;
    double launch_bite_rpm = 0.0, launch_full_rpm_min = 0.0, launch_full_rpm_max = 0.0, launch_anti_stall_rpm = 0.0;
    double launch_lock_slip_rpm = 0.0;
    double cvt_target_rpm[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // at throttle kCvtTargetThrottle[i]
    double cvt_min_rpm[3] = {0.0, 0.0, 0.0};              // at output rpm kCvtMinOutputRpm[i]
};
inline constexpr double kCvtTargetThrottle[5] = {0.0, 0.2, 0.5, 0.8, 1.0};
inline constexpr double kCvtMinOutputRpm[3] = {0.0, 2000.0, 6000.0};
AutomaticTcuPoints derive_automatic_tcu_points(double idle_rpm, double limiter_rpm);

// One section of the garage list.
struct GearboxSection {
    std::string label;        // "Manual", "Automatic", "Dual-clutch", "CVT"
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
    // Manual, Automatic, Dual-clutch, CVT in that order; has_entries from the catalogue rows.
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
