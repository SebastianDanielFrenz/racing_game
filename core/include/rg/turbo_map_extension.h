// rg/turbo_map_extension.h - turbo stage maps that grow when the engine's air demand runs past them.
//
// Owner rule: "turbo maps should recompute and cache when out of range". A turbo hardware file may name a stage map
// ("performance_map", physics_sim.turbo_stage_maps/1). When ps::drivetrain::turbo_stage_demand finds steps of the
// engine's air demand outside that map's certified cells, extend_turbo_maps() widens the sampling around the demand
// (ps::drivetrain::meanline::extend_sampling_for_demand, policy = data/engines/turbo_map_extension.json), generates the
// map with the physics_sim meanline solver and caches it as <engine map cache dir>/<cache_subdir>/turbo_stage_<key>.json
// (key = meanline::cache_key of geometry + extended axes; written atomically). The same demand later hits the cache.
// The loaded VehicleDesc then carries the extended maps, so a drive and the garage setup check use the same map.
//
// Threading: extend_turbo_maps() is synchronous and runs the solver on the CALLING thread (or on `pool`), so callers call
// it from a worker (the garage advisor thread, Session's start-up worker). TurboMapWorker runs it on its own thread for
// callers that must not wait. Engine-neutral, no Godot type.
//
// A demand beyond the compressor's own choke line stays uncovered whatever the sampling (the solver finds no valid point).
// That is accepted: TurboExtensionResult::choke says at which per-turbo flow the compressor chokes at this boost, and the
// garage setup check reports exactly that.
#pragma once

#include "ps/drivetrain/turbo_meanline.h"
#include "ps/vehicle/vehicle_desc.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace ps::jobs { class JobSystem; }

namespace rg {

struct TurboMapExtensionConfig {
    bool enabled = true;
    ps::drivetrain::meanline::ExtensionPolicy policy;
    std::string cache_subdir = "turbo_stage_maps"; // under the engine map cache dir
    // Choke search (the flow above which no valid compressor point reaches the target pressure ratio).
    double choke_rpm_step = 500.0;     // corrected rpm step of the scan
    int choke_bisect_iterations = 40;
    double choke_flow_scan_max_factor = 1.5; // upper flow bound = largest demanded flow x this
};

// data/engines/turbo_map_extension.json (rg.turbo_map_extension/1), strict, exception-free. nullopt + *err on error.
std::optional<TurboMapExtensionConfig> load_turbo_map_extension_config(const std::string& path, std::string* err);

// The compressor's choke at the target boost, per turbo.
struct TurboChoke {
    bool chokes = false;             // some demanded step is still outside the map after the extension
    double choke_flow_kg_s = 0.0;    // per turbo: the largest flow at which the compressor reaches the target pressure ratio
    double demand_flow_kg_s = 0.0;   // per turbo: the engine's largest demanded flow at the target boost
    double pressure_ratio = 0.0;     // the target (charge + intercooler drop) / ambient
    int units = 1;
};

struct TurboExtensionResult {
    enum class Status {
        NotApplicable, // no simulated engine with a stage-map turbo, disabled, or the map file could not be located
        Covered,       // the map already covers the demand: nothing generated
        CacheHit,      // the extended map was in the cache: read, not generated
        Generated,     // generated now and written to the cache
        Failed,        // generation or the cache failed: the vehicle keeps its original map (message says why)
    };
    Status status = Status::NotApplicable;
    std::string message;
    std::string source_map_path;   // the performance_map file the extension started from
    std::string cache_file;        // the extended map file (CacheHit/Generated)
    std::uint64_t cache_key = 0;
    double seconds = 0.0;          // wall time of this call
    TurboChoke choke;              // filled whenever the engine has a stage-map turbo (even Covered)
    int uncovered_steps_before = 0;
    int uncovered_steps_after = 0;
};
const char* to_string(TurboExtensionResult::Status s);

struct TurboExtensionHooks {
    // Test hook: called on the generating thread right before the solver starts (a cache miss only).
    std::function<void()> before_generate;
};

// Extends the stage map of the (first) simulated engine's turbo in `desc` when its demand is not covered, as described
// above. `vehicle_path` is the vehicle file desc was loaded from (the turbo hardware file is found from it);
// `cache_dir` is the engine map cache dir ("" = nothing is cached or reused, the map is generated every time).
// Never throws except ps::io::GenerationCancelled-free: errors become Status::Failed. `cancel` is read between steps.
TurboExtensionResult extend_turbo_maps(ps::vehicle::VehicleDesc& desc, const std::string& vehicle_path, const std::string& cache_dir,
                                       const TurboMapExtensionConfig& config, ps::jobs::JobSystem* pool = nullptr,
                                       const std::atomic<bool>* cancel = nullptr, const TurboExtensionHooks* hooks = nullptr);

// The performance_map file the vehicle's engine turbo names ("" = none found). Follows vehicle component
// "forced_induction" or the engine file's "default_forced_induction" -> configuration "turbo" -> hardware "performance_map",
// each through "extends" parents.
std::string find_turbo_stage_map_path(const std::string& vehicle_path);

// extend_turbo_maps on its own thread. submit() returns at once; the job is polled. The destructor cancels and joins.
class TurboMapWorker {
public:
    struct Job {
        std::atomic<bool> done{false};
        std::atomic<bool> cancel{false};
        std::thread::id thread;            // the worker thread (differs from the submitter's)
        ps::vehicle::VehicleDesc desc;     // the extended copy, valid once done
        TurboExtensionResult result;       // valid once done
    };
    TurboMapWorker() = default;
    ~TurboMapWorker();
    TurboMapWorker(const TurboMapWorker&) = delete;
    TurboMapWorker& operator=(const TurboMapWorker&) = delete;

    std::shared_ptr<Job> submit(ps::vehicle::VehicleDesc desc, std::string vehicle_path, std::string cache_dir,
                                TurboMapExtensionConfig config, TurboExtensionHooks hooks = {});
    void cancel_all();

private:
    std::mutex mutex_;
    std::vector<std::pair<std::shared_ptr<Job>, std::thread>> jobs_;
};

} // namespace rg
