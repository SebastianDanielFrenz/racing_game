#pragma once
#include "rg/vehicle_setup.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/vehicle_desc.h"
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
namespace rg {
// The reference conditions of the electrified (EV / fuel-cell / hybrid) garage dyno (data/garage/dyno_reference.json,
// rg.garage_dyno_reference/1). The chart shows what the car delivers at full pedal under exactly these conditions; the
// battery discharge limit itself is the pack's own (physics BatteryDesc::max_discharge_w, the limit the drive applies).
// Combustion-only cars ignore this file: their dyno is the crank sweep it always was.
struct DynoReference {
    double battery_soc = 0.8;          // every pack, held for the whole point (battery EVs and hybrids)
    // Fuel-cell cars: the pack is held at the stack's own target_soc minus this, where physics' stack law
    // (fuel_cell_request_j: full power at target - 0.05, none at target) supplies its full rating - battery_soc 0.8 is the
    // shipped stacks' target, where they supply nothing (owner 2026-10-20: the stack's share must be visible).
    double fuel_cell_soc_below_target = 0.05;
    double hydrogen_fraction = 1.0;    // of every fuel-cell tank
    std::string drive_mode = "normal"; // eco | normal | sport | racing (the hybrid supervisor's channel assist.drive_mode)
    int drive_mode_index = 1;          // 0..3, from drive_mode
    double pedal = 1.0;
    double reference_gear_ratio = 1.0; // hybrids: the forward gear whose ratio is nearest this (ties: the lower gear)
    int points = 33;                   // 2^k + 1
    int coarse_points = 9;             // 2^j + 1 <= points: the provisional curve
    double motor_rpm_min = 250.0;      // motor-rpm axis: first point
    double motor_rpm_top_fraction = 0.99; // motor-rpm axis: last point as a fraction of the axis motor's max_rpm
    // Hybrid engine-rpm axis: first point = the auto-clutch's lock-up speed (ManualTcu rolling_reengage_rpm) + this margin.
    double hybrid_clutch_margin_rpm = 25.0;
    double ambient_pressure_pa = 101325.0;
    double ambient_temperature_k = 288.15;
    // Setup options that change an electrified car's dyno beyond the engine ones (gearing moves the motor's speed).
    std::vector<std::string> electrified_setup_options;
    std::string source;
};
// nullopt + *err (path and reason) on an unreadable file, a wrong format, a value out of range or an unknown key.
std::optional<DynoReference> load_dyno_reference(const std::string& path, std::string* err);

enum class DynoTopology : std::uint8_t { Combustion = 0, Electric = 1, FuelCellElectric = 2, Hybrid = 3 };
enum class DynoAxis : std::uint8_t { EngineRpm = 0, MotorRpm = 1 };
const char* to_string(DynoTopology t);

// The setup options a dyno request needs: the engine and turbo options (as before), the powertrain conversion, and, for
// an electrified car (`reference` given), reference->electrified_setup_options. Electrified = the conversion choice is
// not "ice", or no conversion is chosen and the catalogue car's own vehicle file has an electric motor.
VehicleSetup dyno_setup(const SetupOptionTable& table, const CatalogEntry& entry, const VehicleSetup& working,
                        const DynoReference* reference);

struct DynoRequest {
    CatalogEntry entry;
    SetupOptionTable table;
    SetupContext context;
    VehicleSetup setup;
    std::string key;
    // Needed by electrified cars only (nullptr there = an error result); combustion cars never read it.
    std::shared_ptr<const DynoReference> reference;
};
struct DynoMotorShare {
    double rpm = 0, torque_nm = 0, power_kw = 0; // the motor's own shaft
    bool battery_limited = false; // this motor's limit was the pack (BatteryDischarge / Empty)
};
struct DynoPoint {
    // rpm = the axis value (crank rpm for combustion cars and hybrids, the axis motor's rpm for EVs). torque_nm / power_kw:
    // combustion = net crank output; electrified = the SYSTEM output absorbed at the hubs, torque referred to the axis shaft.
    double rpm=0, torque_nm=0, power_kw=0;
    bool turbo_simulation=false;
    double compressor_sim_fraction=0, turbine_sim_fraction=0, turbo_fallback_fraction=0;
    // Electrified only (0 / empty for combustion cars). Shares: engine at the crank, electric = the traction motors'
    // mechanical power; torques referred to the axis shaft (power / axis omega); fuel cell / battery = electrical power.
    double engine_rpm=0; // the crank's own speed (differs from rpm while the hybrid's auto-clutch slips)
    double engine_torque_nm=0, engine_power_kw=0;
    double electric_torque_nm=0, electric_power_kw=0;
    double fuel_cell_power_kw=0, battery_power_kw=0;
    bool battery_limited=false, engine_running=false, converged=true;
    int gear=0;
    std::vector<DynoMotorShare> motors; // traction motors, in DynoResult::motor_labels order
};
struct DynoResult {
    std::string vehicle_id, key, error;
    std::uint64_t revision=0;
    bool busy=false;
    int progress=0;
    std::vector<DynoPoint> points;
    std::vector<DynoPoint> provisional_points;
    // What the points mean (set with points; Combustion with an empty caption = the classic crank chart).
    DynoTopology topology = DynoTopology::Combustion;
    DynoAxis axis = DynoAxis::EngineRpm;
    int reference_gear = 0;                // hybrids: the held gear (1-based), else 0
    std::vector<std::string> motor_labels; // traction motor component ids
    std::string caption;                   // the reference conditions in the player's words ("" = combustion default)
    // Non-empty: this setup has no dyno yet (no curve is drawn); the reason in the player's words.
    std::string unsupported_reason;
};
// Immutable snapshots only; no Garage, Godot or live-world state on the worker.
class GarageDyno {
public:
    GarageDyno();
    ~GarageDyno();
    void request(DynoRequest request);
    void cancel();
    DynoResult result() const;
private:
    void run();
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::optional<DynoRequest> pending_;
    std::shared_ptr<std::atomic<bool>> cancel_;
    DynoResult result_;
    std::string requested_key_;
    std::uint64_t revision_=0;
    bool stopping_=false;
    std::thread thread_;
};

// The electrified chart's descriptive fields for a loaded vehicle (topology, axis, held gear, motor labels, caption,
// unsupported reason); nullopt = no traction motor (the combustion crank sweep applies).
struct ElectrifiedDynoPlan {
    DynoTopology topology = DynoTopology::Electric;
    DynoAxis axis = DynoAxis::MotorRpm;
    int gear = 0;                    // held gear for the EngineRpm axis (1-based), else 0
    int axis_motor = 0;              // MotorRpm axis: index among ALL the vehicle's electric motors (physics' motor_index)
    std::vector<int> traction;       // indices (among all motors) of the traction motors
    std::vector<std::string> labels; // their component ids
    std::vector<double> axis_points; // in rpm order
    double battery_soc = 0.8;        // the pack's held state of charge (DynoReference::battery_soc, or the fuel-cell rule)
    std::string caption;
    std::string unsupported_reason;  // non-empty: no sweep
};
std::optional<ElectrifiedDynoPlan> plan_electrified_dyno(const ps::vehicle::VehicleDesc& vehicle, const DynoReference& reference);
// Runs the plan's sweep through ps::drivetrain::run_powertrain_dyno: the coarse points first (publish(sorted points, done)),
// then the rest. Points come back in rpm order. Throws what the physics throws (std::invalid_argument, GenerationCancelled).
std::vector<DynoPoint> measure_electrified_dyno(const ps::vehicle::VehicleDesc& vehicle, const DynoReference& reference,
    const ElectrifiedDynoPlan& plan, ps::jobs::JobSystem* pool, const std::atomic<bool>* cancel,
    const std::function<void(const std::vector<DynoPoint>&, int)>& publish);
} // namespace rg
