// rg/driveline_swap.h - the drivetrain swap (owner on PR #5, 2026-10-15). Engine-neutral.
//
// data/drivetrains/driveline_catalog.json ("rg.driveline_catalog/1") lists the drive layouts the garage offers
// (FWD, RWD and AWD with an open / viscous / locked centre) and the axle differential units (open, clutch LSD,
// Torsen, viscous, locked). Every kind is one physics_sim simulates (ps::drivetrain::DifferentialKind); physics_sim
// builds clutch LSD and Torsen only as axle (leaf) differentials, so the catalogue refuses them as a centre, and a
// non-open centre is made lossless (efficiency 1.0), which physics_sim also requires.
//
// A swap is a generic transform of a vehicle file's driveline, never a per-car file:
//   * the driveline is read from the powertrain graph: the shaft holding "gearbox.out" feeds one differential's
//     carrier; that differential either drives the two wheels of one axle (FWD/RWD) or is a centre differential
//     whose left/right feed one axle differential each (AWD, left = front). Anything else (a transfer case, a gear
//     pair, a third axle, an axle without exactly one left and one right wheel) is not swappable - the garage then
//     does not offer the options and says why;
//   * a layout swap removes the differentials and their shafts and builds the new ones: the axle differential(s)
//     are copies of the car's own axle unit (the one of the same axle when there is one, else the other axle's) -
//     ratio, efficiency, carrier inertia, axis and kind - so overall gearing is unchanged; the AWD centre is the
//     catalogue's centre unit (or the car's own centre when it has one) with the chosen kind. Ids follow the
//     existing files: "diff" for a single axle, "centre"/"front"/"rear" for AWD (the final_drive option names
//     those). The gearbox output shaft keeps every other port on it (a park pawl). Left/right wheels by the sign
//     of attachment_local.y (ISO: +y is left);
//   * an axle differential choice replaces only that unit's kind and its kind parameters.
// Nothing here checks engine position (a transverse FWD box vs a longitudinal one): every swappable car can take
// every layout. Brakes, the handbrake and the controllers are untouched - traction control and launch control
// find the driven wheels from the powertrain graph in physics_sim.
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rg {

// A differential unit: a kind and its kind parameters (JSON object text holding only the keys physics_sim
// accepts for that kind, e.g. {"kind":"torsen","torque_bias_ratio":2.5}).
struct DiffUnit {
    std::string id;           // lower_snake, unique among the catalogue's differentials
    std::string name;
    std::string kind;         // open | clutch_lsd | torsen | viscous | locked
    std::string description;
    std::string params_json;  // the vehicle-file keys of this kind, "kind" included
};

struct DriveLayoutEntry {
    std::string id;           // lower_snake, unique
    std::string name;
    std::string layout;       // "FWD" | "RWD" | "AWD"
    std::string description;
    std::optional<DiffUnit> centre; // AWD only (required there): the centre kind and its parameters
};

class DrivelineCatalog {
public:
    // nullptr + *err on a bad file (unknown format, kind, missing field, a kind parameter of another kind).
    static std::unique_ptr<DrivelineCatalog> open(const std::string& path, std::string* err);
    static std::unique_ptr<DrivelineCatalog> parse(const std::string& json_text, const std::string& origin, std::string* err);

    [[nodiscard]] const std::vector<DriveLayoutEntry>& layouts() const { return layouts_; }
    [[nodiscard]] const std::vector<DiffUnit>& differentials() const { return diffs_; }
    [[nodiscard]] const DriveLayoutEntry* find_layout(const std::string& id) const;
    [[nodiscard]] const DiffUnit* find_differential(const std::string& id) const;
    // The catalogue differential of `kind` ("open", "torsen" ...), nullptr if none.
    [[nodiscard]] const DiffUnit* differential_of_kind(const std::string& kind) const;
    // The AWD layout whose centre is of `kind`, nullptr if none.
    [[nodiscard]] const DriveLayoutEntry* awd_layout_of_centre(const std::string& kind) const;
    // The centre unit's ratio/efficiency/inertia/axis (JSON object text), used when a car has no centre yet.
    [[nodiscard]] const std::string& centre_unit_json() const { return centre_unit_json_; }

private:
    std::vector<DriveLayoutEntry> layouts_;
    std::vector<DiffUnit> diffs_;
    std::string centre_unit_json_;
};

// What a vehicle file's driveline is.
struct DrivelineInfo {
    std::string layout = "other"; // "FWD" | "RWD" | "AWD" | "other" (from the driven wheels, any topology)
    int driven_wheels = 0;
    int wheel_count = 0;
    bool swappable = false;       // the topology is one the swap understands (see the file comment)
    std::string reason;           // why not, when !swappable
    std::string front_kind, rear_kind, centre_kind; // the differential kinds; "" when that unit does not exist
};

// Reads the driveline of a vehicle file's JSON text (never throws; unparsable text -> layout "other").
DrivelineInfo describe_driveline(const std::string& vehicle_json);

// The choices a setup makes; "" = keep.
struct DrivelineChoice {
    std::string layout_id;      // a DriveLayoutEntry id
    std::string front_diff_id;  // a DiffUnit id, for the front axle
    std::string rear_diff_id;   // a DiffUnit id, for the rear axle
};

// Applies the choice to the vehicle JSON text: "" and *out_json on success, else the reason in plain words
// ("the front axle is not driven", "this car's driveline cannot be swapped: ..."). An empty choice returns the
// text unchanged.
std::string apply_driveline(const std::string& vehicle_json, const DrivelineCatalog& catalog, const DrivelineChoice& choice,
                            std::string* out_json);

// "Open", "Clutch LSD", "Torsen", "Viscous", "Locked"; the kind itself when unknown.
std::string diff_kind_label(const std::string& kind);
// "RWD · Torsen rear" / "AWD · viscous centre · open front · open rear".
std::string driveline_summary(const DrivelineInfo& info);

} // namespace rg
