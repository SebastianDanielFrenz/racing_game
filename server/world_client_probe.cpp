#include "rg/world_service_client.h"
#include "rg/weather_authority.h"
#include "rg/climate.h"
#include "g2m/layer/terrain_layers.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
    if (argc != 4) { std::cerr << "usage: rg_world_client_probe URL ROOT CACHE_DIR\n"; return 2; }
    std::string error;
    const std::string root = argv[2];
    auto sky = rg::load_sky_config(root + "/data/world/sky.json", &error);
    auto climate = rg::load_climate_config(root + "/data/world/climate.json", &error);
    if (!sky || !climate) { std::cerr << error; return 1; }
    rg::TimeWeather tw(*sky, {50.1, 8.6, 0});
    tw.set_climate(std::make_shared<const rg::ClimateConfig>(*climate)); tw.use_remote_weather();
    rg::WorldServiceClient client(argv[1], root + "/data/world/sky.json", root + "/data/world/climate.json");
    const auto start = std::chrono::steady_clock::now();
    while (!client.sample()) {
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) { std::cerr << client.status(); return 1; }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    client.advance(tw, 0.02);
    const double first = rg::game_seconds(tw.clock());
    for (int i = 0; i < 20; ++i) { std::this_thread::sleep_for(std::chrono::milliseconds(20)); client.advance(tw, 0.02); }
    const auto e = *client.sample();
    if (rg::game_seconds(tw.clock()) < first || std::abs(rg::game_seconds(tw.clock()) - e.game_s) > 2 ||
        tw.weather_broadcast().weather_id != e.weather.weather_id) { std::cerr << "clock/weather mismatch"; return 1; }
    g2m::world::TerrainServiceConfig cfg;
    cfg.derive_workers = 2;
    cfg.stores.source_dir = "THIS_STORE_MUST_NOT_BE_OPENED";
    cfg.layers = {true, true, false, false, false};
    if (!rg::configure_remote_terrain(cfg, argv[1], argv[3], &error)) { std::cerr << error; return 1; }
    auto invalid = cfg;
    invalid.remote_transport.reset();
    if (g2m::world::TerrainService::open(invalid)) { std::cerr << "partial remote configuration accepted"; return 1; }
    invalid = cfg; invalid.layers.classes = true;
    if (g2m::world::TerrainService::open(invalid)) { std::cerr << "missing class layer accepted"; return 1; }
    auto service = g2m::world::TerrainService::open(cfg);
    if (!service) { std::cerr << service.error().message; return 1; }
    const g2m::TileKey key{g2m::geo::UtmZone{32}, 0, 1816, 21688};
    auto height = (*service)->height(key, g2m::world::Priority::Gate);
    if (!height.wait_for(std::chrono::seconds(25)) || !height.value()) { std::cerr << "remote height missing"; return 1; }
    const g2m::Request request = g2m::TileRequest{cfg.remote_manifest->rid, "g2m.terrain.height", key, std::nullopt};
    auto tile = cfg.remote_transport->send(request);
    const auto original = std::get<g2m::TileResponse>(tile).container;
    if (original.empty()) return 1;
    std::cout << "RG_WORLD_CLIENT_READY release=" << (*service)->release_id_hex() << std::endl;
    std::string line; std::getline(std::cin, line); // harness closes its own server before resuming
    // Bypass decoded cache to exercise the persistent validated-container cache.
    tile = cfg.remote_transport->send(request);
    if (std::get<g2m::TileResponse>(tile).container != original) { std::cerr << "cached tile changed"; return 1; }
    std::this_thread::sleep_for(std::chrono::milliseconds(3300));
    client.advance(tw, 0.02);
    const double held = rg::game_seconds(tw.clock());
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); client.advance(tw, 0.1);
    if (rg::game_seconds(tw.clock()) != held) { std::cerr << "stale authority kept advancing"; return 1; }
    std::cout << "RG_WORLD_CLIENT_PASS clock weather remote-stream cache outage-hold\n";
    return 0;
}
