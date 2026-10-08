#pragma once
#include "rg/vehicle_setup.h"
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
namespace rg {
struct DynoRequest {
    CatalogEntry entry;
    SetupOptionTable table;
    SetupContext context;
    VehicleSetup setup;
    std::string key;
};
struct DynoPoint { double rpm=0, torque_nm=0, power_kw=0; };
struct DynoResult {
    std::string vehicle_id, key, error;
    std::uint64_t revision=0;
    bool busy=false;
    int progress=0;
    std::vector<DynoPoint> points;
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
} // namespace rg
