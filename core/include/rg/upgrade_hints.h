// rg/upgrade_hints.h - the numbers behind the garage's upgrade hints (owner 2026-10-19). Engine-neutral.
//
// A hint is the change of one figure of the WHOLE CAR: the car with the candidate part swapped in minus the car as it is
// configured now (rg/upgrade_badges.h). This file computes those figures for one complete setup:
//
//   weight               the part-mass tree total (compile only)
//   rpm_tolerance        the weakest installed internal's limit rpm (compile only)
//   grip, rolling        the car's tyres at its static corner loads, from physics_sim's public tyre model (the vehicle loaded
//                        without engine maps)
//   battery_kwh          the vehicle file's batteries
//   power, torque, temperatures
//                        physics_sim's engine dyno (run_dyno_sweep) on the setup's engine; the temperatures are the engine
//                        thermal model's steady state at the peak-power point
//
// and caches them per (whole setup, nitrous armed) so that hovering over the tiles of a page costs one baseline and one sweep
// per tile, once. The expensive ones run on a background thread (UpgradeHints): the answer to a request is what is cached now
// plus the set of figures still pending, newest request wins, a change of the current configuration cancels the running work.
// Every reference (surface, speeds, ambient, map quality, rpm grid) is data/garage/upgrade_badges.json.
#pragma once

#include "rg/upgrade_badges.h"
#include "rg/vehicle_setup.h"

#include "ps/jobs/job_system.h"
#include "ps/vehicle/vehicle_desc.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace rg {

// Standard gravity: the "g" the grip figures are expressed in.
inline constexpr double kHintGravity = 9.80665;

// ---- the tyres ---------------------------------------------------------------------------------------------------------

// The reference surface and speed of the tyre figures, resolved from UpgradeReferences::tyres.
struct TyreReference {
    double lambda_mu = 1.0;     // the surface table's friction scale of the reference surface
    double crr = 0.0;           // its rolling-resistance coefficient
    double speed_m_s = 27.78;   // the rolling-resistance reference speed
};
// Loads the surface table named by the references; nullopt + *err when it cannot be read or lacks the surface.
std::optional<TyreReference> load_tyre_reference(const UpgradeReferences& refs, std::string* err);

// The static corner loads (N) of wheels at the given chassis-frame (x forward, y left) positions carrying `weight_n` with
// the centre of mass at the origin: the smallest-norm loads that sum to the weight and have no moment about either axis (a car
// without cross-weight). Fewer than three wheels, or wheels on one line, cannot carry a load and return an equal share.
std::vector<double> static_corner_loads(const std::vector<std::pair<double, double>>& wheel_xy, double weight_n);

struct TyreFigures {
    double weight_n = 0.0;
    std::vector<double> corner_loads_n;
    double peak_fx_n = 0.0;     // sum over the wheels of the peak |Fx| over the slip ratio (pure longitudinal slip)
    double peak_fy_n = 0.0;     // sum over the wheels of the peak |Fy| over the slip angle (pure lateral slip)
    double rolling_n = 0.0;     // sum over the wheels of the rolling resistance force when cruising at the reference speed
    // The three as a share of the weight: what the hints show (grip in g, rolling resistance as a relative change).
    double grip_longitudinal_g = 0.0;
    double grip_lateral_g = 0.0;
    double rolling_g = 0.0;
};
// physics_sim's public Pacejka model (create_pacejka_tyre_model) of every wheel of `desc`, at the static corner loads of a car
// of `mass_kg`, with the wheel's static camber. Throws std::runtime_error on a vehicle without wheels or a non-positive mass.
TyreFigures tyre_figures(const ps::vehicle::VehicleDesc& desc, double mass_kg, const TyreReference& reference);

// ---- the engine --------------------------------------------------------------------------------------------------------

struct HintEnginePeaks {
    bool has_engine = false;         // a simulated or torque-map engine was swept
    double peak_power_kw = 0.0;
    double peak_torque_nm = 0.0;
    double peak_power_rpm = 0.0;
    double coolant_c = std::numeric_limits<double>::quiet_NaN(); // steady state at the peak-power point; NaN without the model
    double oil_c = std::numeric_limits<double>::quiet_NaN();
    double electric_kw = 0.0;        // electric machines' rated power, capped by the batteries' discharge limit (added to power)
    double battery_kwh = 0.0;
    double sweep_seconds = 0.0;
};

// Everything figure computation needs besides the setup: the references, the tyre reference and the machinery.
struct FigureEnv {
    UpgradeReferences references;
    TyreReference tyre;
    bool tyre_ok = false;
    std::string tyre_error;
    ps::jobs::JobSystem* pool = nullptr;           // map generation and the dyno's per-rpm jobs (null = the calling thread)
    const std::atomic<bool>* cancel = nullptr;     // throws ps::io::GenerationCancelled when set
};
FigureEnv make_figure_env(const UpgradeReferences& refs);

// The immutable snapshot of one car the figures are computed for.
struct HintBase {
    CatalogEntry entry;
    SetupOptionTable table;
    SetupContext context;
};

struct SetupFigures {
    PartFigures figures;                    // what exists for this setup (a quantity the car has no number for is absent)
    std::set<UpgradeQuantity> computed;     // every quantity that was asked for (also those without a figure)
    std::string error;                      // non-empty: the setup could not be built (the figures are empty)
    double seconds = 0.0;
};
// Computes `wanted` for one complete setup, on the calling thread. `nitrous_armed`: power and the temperatures are measured with
// the car's N2O kit armed (a car without a kit is unchanged). Throws ps::io::GenerationCancelled when env.cancel is set.
SetupFigures compute_setup_figures(const HintBase& base, const VehicleSetup& setup, const std::set<UpgradeQuantity>& wanted,
                                   bool nitrous_armed, const FigureEnv& env);

// ---- the cache and the background thread ------------------------------------------------------------------------------

struct HintAnswer {
    PartFigures candidate, installed;
    std::set<UpgradeQuantity> pending; // wanted quantities whose candidate or installed figure is still being computed
    std::string error;                 // the candidate or the current configuration could not be built
};

struct HintStats {
    int jobs_started = 0, jobs_finished = 0, jobs_cancelled = 0;
    int figure_sets_computed = 0;      // compute_setup_figures calls that completed
    int cache_hits = 0;                // requests answered completely from the cache
    double last_job_seconds = 0.0, total_seconds = 0.0;
};

class UpgradeHints {
public:
    // `pool_workers` 0 = the background rule of the limit advisor (all but four logical processors, RG_ENGINE_MAP_WORKERS).
    explicit UpgradeHints(UpgradeBadgeConfig config, int pool_workers = 0);
    ~UpgradeHints();
    UpgradeHints(const UpgradeHints&) = delete;
    UpgradeHints& operator=(const UpgradeHints&) = delete;

    [[nodiscard]] const UpgradeBadgeConfig& config() const { return config_; }

    // The car and its CURRENT whole setup. The cache is kept while the car (entry id + engine list) stays the same; running
    // work for another current setup is cancelled. Cheap to call on every request.
    void set_base(HintBase base, VehicleSetup current);

    // What is known about `metrics` of `candidate` against the current setup; unknown figures are queued (newest request wins,
    // debounced) and listed as pending. `nitrous_armed` measures power and temperatures with the car's N2O kit armed.
    HintAnswer request(const VehicleSetup& candidate, const std::vector<UpgradeQuantity>& metrics, bool nitrous_armed);

    // Drops queued work and cancels the running job (the cache is kept).
    void cancel();
    // Blocks until nothing is queued or running; false on timeout. For tests and screenshot runs.
    bool wait_idle(int timeout_ms);
    [[nodiscard]] HintStats stats() const;

private:
    struct Entry {
        PartFigures figures;
        std::set<UpgradeQuantity> done;
        std::string error;
    };
    struct Job {
        std::string cand_key;
        VehicleSetup candidate;
        std::set<UpgradeQuantity> wanted;
        bool nitrous = false;
    };
    void run();
    static std::string key_of(const VehicleSetup& setup, bool nitrous);

    UpgradeBadgeConfig config_;
    FigureEnv env_;
    int pool_workers_ = 0;

    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::condition_variable idle_;
    HintBase base_;
    bool have_base_ = false;
    VehicleSetup current_;
    std::uint64_t base_generation_ = 0; // bumps when the current setup or the car changes
    std::uint64_t request_generation_ = 0; // bumps on every queued request / cancel
    std::map<std::string, Entry> cache_;
    std::optional<Job> pending_;
    bool running_ = false;
    bool running_candidate_phase_ = false;
    std::string running_cand_key_;
    std::shared_ptr<std::atomic<bool>> cancel_;
    HintStats stats_;
    bool stopping_ = false;
    std::thread thread_;
};

} // namespace rg
