// rg/ev_motor.h - electric motor hardware and motor layouts for battery EVs (owner 2026-10-21). Engine-neutral.
//
// data/electric_motors/motor_catalog.json ("rg.ev_motor_catalog/1") lists the drive units the garage offers (a motor,
// its inverter and its own fixed reduction gear, every figure [AS]) and the motor layouts (single, dual, tri, quad).
//
// A layout is a generic transform of a vehicle file's powertrain, never a per-car file, in the style of
// rg/driveline_swap.h:
//   * the powertrain is read from the graph: every electric_motor sits alone on a shaft with the input of ONE reduction -
//     an axle unit (motor -> open differential carrier -> the left and right hub of one axle) or a per-wheel unit
//     (motor -> gear_pair in, gear_pair out -> one hub). Anything else (a combustion engine, a fuel cell, a gearbox, a
//     transfer case, a motor on a shared shaft, a differential that feeds a differential, one axle driven by a mix of
//     an axle unit and a wheel motor) is not swappable: the garage then does not offer the options and says why;
//   * a layout swap removes the drive units and their shafts and builds the new ones. Every new motor and reduction is a
//     copy of the car's own unit of the same axle (the other axle's when that axle had none), so a layout alone keeps the
//     hardware; a per-wheel reduction is a gear_pair with the ratio, efficiency and inertia of the axle differential it
//     replaces, so overall gearing does not change. Ids: "front_motor"/"front_diff", "rear_motor"/"rear_diff" for an axle
//     unit; "motor_<wheel>"/"reduction_<wheel>" (lower case wheel name) for a per-wheel unit. The battery, the brakes and
//     the controllers are untouched - traction control and launch control find the driven wheels from the powertrain graph
//     in physics_sim. Left/right wheels by the sign of attachment_local.y (ISO: +y is left);
//   * a unit choice sets the catalogue figures on every motor of one axle and the unit's reduction on every reduction of
//     that axle.
// Every motor gets the same torque command: there is no torque vectoring.
#pragma once

#include <memory>
#include <string>
#include <vector>

namespace rg {

// One drive unit: motor + inverter + reduction.
struct EvMotorUnit {
    std::string id;           // lower_snake, unique
    std::string label;
    std::string inspiration;  // a class name only, never a claim
    std::string source;       // [AS] note
    double peak_torque_nm = 0.0;
    double max_power_w = 0.0;
    double max_rpm = 0.0;
    double inertia_kgm2 = 0.0;
    double efficiency = 0.0;
    double regen_torque_nm = 0.0;
    double mass_kg = 0.0;     // motor + inverter + reduction
    double reduction_ratio = 0.0;
    double reduction_efficiency = 0.98;
};

// How one axle is driven.
enum class EvAxleDrive { None, Axle, Wheel };

struct EvLayoutEntry {
    std::string id;           // lower_snake, unique
    std::string name;
    std::string description;
    EvAxleDrive front = EvAxleDrive::None;
    EvAxleDrive rear = EvAxleDrive::None;
    [[nodiscard]] int motor_count() const;
};

class EvMotorCatalog {
public:
    // nullptr + *err on a bad file (unknown format, missing field, a non-positive figure, a duplicate id).
    static std::unique_ptr<EvMotorCatalog> open(const std::string& path, std::string* err);
    static std::unique_ptr<EvMotorCatalog> parse(const std::string& json_text, const std::string& origin, std::string* err);

    [[nodiscard]] const std::vector<EvMotorUnit>& units() const { return units_; }
    [[nodiscard]] const std::vector<EvLayoutEntry>& layouts() const { return layouts_; }
    [[nodiscard]] const EvMotorUnit* find_unit(const std::string& id) const;
    [[nodiscard]] const EvLayoutEntry* find_layout(const std::string& id) const;
    // The layout whose axle drives are `front`/`rear`, nullptr if the catalogue has none.
    [[nodiscard]] const EvLayoutEntry* layout_of(EvAxleDrive front, EvAxleDrive rear) const;

private:
    std::vector<EvMotorUnit> units_;
    std::vector<EvLayoutEntry> layouts_;
};

// What a vehicle file's motors are.
struct EvMotorInfo {
    bool swappable = false;   // a battery EV whose drive units the swap understands (see the file comment)
    std::string reason;       // why not, when !swappable
    // A car with a combustion engine and traction motors (a hybrid): not swappable (the layout stays battery-EV only), but the
    // drive unit of its traction motor(s) can change. The crank generator (crank_assist) is no traction motor.
    bool hybrid = false;
    int traction_motors = 0;  // hybrid only
    EvAxleDrive front = EvAxleDrive::None;
    EvAxleDrive rear = EvAxleDrive::None;
    int motor_count = 0;
    int driven_wheels = 0;
    std::string layout_id;    // the catalogue layout these drives make ("" = none, or no catalogue)
};

// Reads the motors of a vehicle file's JSON text (never throws). `catalog` (optional) fills layout_id.
EvMotorInfo describe_ev_motors(const std::string& vehicle_json, const EvMotorCatalog* catalog = nullptr);

// Applies a layout (and/or unit choices) to the vehicle JSON text: "" and *out_json on success, else the reason in plain
// words. Empty ids keep. An empty choice returns the text unchanged.
struct EvMotorChoice {
    std::string layout_id;
    std::string front_unit_id;
    std::string rear_unit_id;
};
std::string apply_ev_motors(const std::string& vehicle_json, const EvMotorCatalog& catalog, const EvMotorChoice& choice,
                            std::string* out_json);

// "Single", "Dual" ... for a layout; "3 motors"-style text for a card.
std::string ev_unit_figures(const EvMotorUnit& unit); // "380 kW · 640 N*m · 20000 rpm · 75 kg"

} // namespace rg
