#pragma once
#include "rg/time_weather.h"
#include "g2m/world/terrain_service.h"
#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace rg {
// Shared by all consumers: an empty endpoint retains the offline single-player path.
std::string world_server_url();
struct RemoteEnvironment {
    std::string instance;
    std::int64_t server_us = 0;
    CivilDate date;
    double local_time_h = 0, game_s = 0, rate = 1, half_rtt_s = 0;
    WeatherBroadcast weather;
    GeoLocation location;
    std::chrono::steady_clock::time_point received;
};
// HTTP polling owns a worker; sample/status never perform network I/O.
class WorldServiceClient {
public:
    explicit WorldServiceClient(std::string url, std::string sky_path = {}, std::string climate_path = {});
    ~WorldServiceClient();
    WorldServiceClient(const WorldServiceClient&) = delete;
    WorldServiceClient& operator=(const WorldServiceClient&) = delete;
    std::optional<RemoteEnvironment> sample() const;
    std::string status() const;
    // Called by the presentation owner. Initial/restarted servers install a fresh
    // authority; subsequent samples slew forward or hold, never rewind the clock.
    void advance(TimeWeather& environment, double real_dt_s);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
// Called on the existing world-init worker, never on a render/simulation tick.
// Discovery must name a pinned release with all requested layers. No local-store fallback.
bool configure_remote_terrain(g2m::world::TerrainServiceConfig& config, const std::string& server_url,
                              const std::string& cache_dir, std::string* error);
} // namespace rg
