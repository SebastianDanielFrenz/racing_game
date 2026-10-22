// Headless world services. Vehicle replication belongs to the later UDP service.
#include "rg/time_weather.h"
#include "rg/weather_authority.h"
#include "rg/climate.h"
#include "g2m/client/in_process_transport.h"
#include "g2m/http/http_server.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/tile_store.h"
#include "g2m/server/upstream.h"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <random>
#include <stdexcept>
#include <thread>
#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#endif

namespace {
using Json = nlohmann::json;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t interrupted = 0;
void signal_handler(int) { interrupted = 1; }
Json read_json(const fs::path& p) {
    std::ifstream f(p);
    if (!f) throw std::runtime_error("cannot read " + p.string());
    return Json::parse(f);
}
void keys(const Json& j, std::initializer_list<const char*> allowed) {
    if (!j.is_object()) throw std::runtime_error("expected an object");
    for (auto i = j.begin(); i != j.end(); ++i) {
        bool found = false;
        for (const auto* k : allowed) if (i.key() == k) found = true;
        if (!found) throw std::runtime_error("unknown config key: " + i.key());
    }
}
int integer(const Json& j, const char* key, int lo, int hi) {
    if (!j.at(key).is_number_integer()) throw std::runtime_error(std::string(key) + " must be an integer");
    const auto n = j.at(key).get<std::int64_t>();
    if (n < lo || n > hi) throw std::runtime_error(std::string(key) + " out of range");
    return static_cast<int>(n);
}
double number(const Json& j, const char* key, double lo, double hi) {
    const double n = j.at(key).get<double>();
    if (!std::isfinite(n) || n < lo || n > hi) throw std::runtime_error(std::string(key) + " out of range");
    return n;
}
fs::path path(const fs::path& root, const Json& j, const char* key) {
    const auto s = j.at(key).get<std::string>();
    if (s.empty()) throw std::runtime_error(std::string(key) + " must not be empty");
    return fs::absolute(root / fs::path(s)).lexically_normal();
}
// An OS lock releases even after a crash; two hosts must never overwrite one world.
struct StateLock {
#ifdef _WIN32
    HANDLE handle = INVALID_HANDLE_VALUE;
    explicit StateLock(const fs::path& p) {
        handle = CreateFileW(p.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (handle == INVALID_HANDLE_VALUE) throw std::runtime_error("world state is locked: " + p.string());
    }
    ~StateLock() { CloseHandle(handle); }
#else
    int handle = -1;
    explicit StateLock(const fs::path& p) {
        handle = ::open(p.c_str(), O_CREAT | O_RDWR, 0600);
        if (handle < 0) throw std::runtime_error("cannot open world lock");
        if (flock(handle, LOCK_EX | LOCK_NB) != 0) { ::close(handle); throw std::runtime_error("world state is locked"); }
    }
    ~StateLock() { ::close(handle); }
#endif
    StateLock(const StateLock&) = delete;
    StateLock& operator=(const StateLock&) = delete;
};
void save_json(const fs::path& p, const Json& j) {
    auto tmp = p; tmp += ".tmp";
    { std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
      if (!f || !(f << j.dump(2) << '\n') || !(f.flush())) throw std::runtime_error("checkpoint write failed"); }
#ifdef _WIN32
    if (!MoveFileExW(tmp.c_str(), p.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        throw std::runtime_error("checkpoint replacement failed");
#else
    fs::rename(tmp, p);
#endif
}
// Keep the tile service independent: its own port, workers, release and failures.
struct Maps {
    std::unique_ptr<g2m::TileStore> source, derived;
    g2m::BuiltinTerrainDerivers derivers;
    std::unique_ptr<g2m::LocalSourceUpstream> upstream;
    std::unique_ptr<g2m::Server> server;
    std::unique_ptr<g2m::InProcessTransport> transport;
    std::unique_ptr<g2m::http::HttpServer> http;
    Json discovery = {{"enabled", false}};
    Maps(const fs::path& root, const Json& cfg) {
        keys(cfg, {"enabled", "source", "source_name", "scope", "derived", "bind", "port", "threads", "public_url",
                   "roads", "geometry", "carved", "classes", "rail", "world_seed"});
        if (!cfg.at("enabled").get<bool>()) return;
        g2m::TileStoreConfig sc;
        sc.dir = path(root, cfg, "source").string();
        sc.name = cfg.value("source_name", std::string("tiles")); sc.read_only = true;
        auto s = g2m::TileStore::open(sc);
        if (!s) throw std::runtime_error(s.error().message);
        source = std::move(*s);
        sc.dir = path(root, cfg, "derived").string(); sc.name = "tiles"; sc.read_only = false;
        if (fs::weakly_canonical(source_dir(cfg, root)) == fs::weakly_canonical(sc.dir))
            throw std::runtime_error("source and derived stores must differ");
        auto d = g2m::TileStore::open(sc);
        if (!d) throw std::runtime_error(d.error().message);
        derived = std::move(*d);
        auto params = g2m::builtin_local_release_params(cfg.at("scope").get<std::string>(), derivers, {},
            cfg.value("roads", true), cfg.value("geometry", true), cfg.value("carved", true),
            cfg.value("classes", false), cfg.value("rail", false));
        if (cfg.contains("world_seed") && !cfg.at("world_seed").is_number_unsigned())
            throw std::runtime_error("world_seed must be unsigned");
        params.world_seed = cfg.value("world_seed", std::uint64_t{0});
        auto release = g2m::make_local_release(*source, params);
        if (!release) throw std::runtime_error(release.error().message);
        const auto rid = g2m::to_hex(release->rid);
        upstream = std::make_unique<g2m::LocalSourceUpstream>(*source);
        g2m::ServerConfig sv;
        sv.offline = true; sv.derived_store = derived.get(); sv.derivers = params.derivers; sv.upstreams = {upstream.get()};
        server = std::make_unique<g2m::Server>(sv);
        auto added = server->add_release(std::move(*release), cfg.at("scope").get<std::string>());
        if (!added) throw std::runtime_error(added.error().message);
        transport = std::make_unique<g2m::InProcessTransport>(*server);
        g2m::http::HttpServerConfig hc;
        hc.bind_address = cfg.at("bind").get<std::string>(); hc.port = integer(cfg, "port", 0, 65535);
        hc.threads = static_cast<std::size_t>(integer(cfg, "threads", 1, 64));
        auto h = g2m::http::HttpServer::start(*transport, hc);
        if (!h) throw std::runtime_error(h.error().message);
        http = std::move(*h);
        auto url = cfg.value("public_url", std::string{});
        if (url.empty()) url = http->base_url();
        discovery = {{"enabled", true}, {"base_url", url}, {"api_path", "/g2m/v1"}, {"release_id", rid}, {"port", http->port()}};
    }
    static std::string source_dir(const Json& j, const fs::path& root) { return path(root, j, "source").string(); }
};
int run(const fs::path& root, const fs::path& config, double run_for) {
    const auto cfg = read_json(config);
    keys(cfg, {"format", "bind", "port", "threads", "state_file", "checkpoint_interval_s", "sky", "climate", "schedule",
               "latitude_deg", "longitude_deg", "map"});
    if (cfg.at("format") != "rg.server/1") throw std::runtime_error("expected rg.server/1");
    const int port = integer(cfg, "port", 0, 65535), threads = integer(cfg, "threads", 1, 64);
    const double interval = number(cfg, "checkpoint_interval_s", 0.1, 3600);
    const rg::GeoLocation loc{number(cfg, "latitude_deg", -90, 90), number(cfg, "longitude_deg", -180, 180), 0};
    const auto state_file = path(root, cfg, "state_file");
    fs::create_directories(state_file.parent_path());
    StateLock lock(state_file.string() + ".lock");
    const auto sky_path = path(root, cfg, "sky"), climate_path = path(root, cfg, "climate"), schedule_path = path(root, cfg, "schedule");
    const Json identity = {{"sky", read_json(sky_path)}, {"climate", read_json(climate_path)}, {"schedule", read_json(schedule_path)},
        {"latitude_deg", loc.latitude_deg}, {"longitude_deg", loc.longitude_deg}};
    std::string error;
    auto sky = rg::load_sky_config(sky_path.string(), &error);
    if (!sky) throw std::runtime_error(error);
    auto climate = rg::load_climate_config(climate_path.string(), &error);
    if (!climate) throw std::runtime_error(error);
    std::vector<std::string> ids;
    for (const auto& p : sky->weather_presets) ids.push_back(p.id);
    auto schedule = rg::load_weather_schedule(schedule_path.string(), ids, &error);
    if (!schedule) throw std::runtime_error(error);
    rg::TimeWeather environment(*sky, loc);
    auto climate_ptr = std::make_shared<const rg::ClimateConfig>(*climate);
    environment.set_climate(climate_ptr); environment.use_remote_weather();
    double epoch = rg::game_seconds(environment.clock());
    if (fs::exists(state_file)) {
        const auto saved = read_json(state_file);
        if (saved.at("format") != "rg.server_state/1" || saved.at("content") != identity)
            throw std::runtime_error("checkpoint content mismatch; use a new state_file for a different world");
        const auto& date = saved.at("date");
        rg::CivilDate dt{integer(date, "year", 1, 9999), integer(date, "month", 1, 12), integer(date, "day", 1, 31)};
        if (dt.day > rg::days_in_month(dt.year, dt.month)) throw std::runtime_error("invalid checkpoint date");
        environment.clock().set_date(dt);
        environment.clock().set_local_time_h(number(saved, "local_time_h", 0, 24));
        environment.clock().set_rate(number(saved, "rate", 0, 100000));
        epoch = number(saved, "weather_epoch_game_s", -1e12, 1e12);
        if (epoch > rg::game_seconds(environment.clock())) throw std::runtime_error("checkpoint weather epoch is in the future");
    }
    rg::WeatherAuthority weather(*schedule, sky->weather_presets, climate_ptr, environment.location(), epoch, sky->default_weather, 1);
    weather.update(rg::game_seconds(environment.clock()));
    environment.apply_broadcast(weather.current());
    const auto checkpoint = [&] {
        const auto& date = environment.clock().local_date();
        save_json(state_file, {{"format", "rg.server_state/1"}, {"content", identity},
            {"date", {{"year", date.year}, {"month", date.month}, {"day", date.day}}},
            {"local_time_h", environment.clock().local_time_h()}, {"rate", environment.clock().rate()}, {"weather_epoch_game_s", epoch},
            {"weather", Json::parse(rg::to_json(weather.current()))}});
    };
    Maps maps(root, cfg.at("map"));
    checkpoint(); // fail before announcing readiness if persistence is unwritable
    std::random_device rng;
    const std::string instance = std::to_string(rng()) + "-" + std::to_string(rng());
    Json published;
    std::mutex mutex;
    const auto start = Clock::now();
    const auto publish = [&] {
        const auto state = environment.state();
        const auto now = Clock::now();
        const auto& date = environment.clock().local_date();
        Json j = {{"format", "rg.environment/1"}, {"instance_id", instance},
            {"server_us", std::chrono::duration_cast<std::chrono::microseconds>(now - start).count()},
            {"game_seconds_j2000", rg::game_seconds(environment.clock())}, {"rate", environment.clock().rate()},
            {"date", {{"year", date.year}, {"month", date.month}, {"day", date.day}}},
            {"local_time_h", environment.clock().local_time_h()}, {"utc_offset_h", environment.clock().utc_offset_h()},
            {"weather", Json::parse(rg::to_json(weather.current()))}, {"air_temperature_c", state.air_temp_c}};
        std::lock_guard guard(mutex); published = std::move(j);
    };
    publish();
    httplib::Server http;
    http.new_task_queue = [threads] { return new httplib::ThreadPool(static_cast<std::size_t>(threads), 0, 128); };
    http.set_read_timeout(5, 0); http.set_write_timeout(5, 0); http.set_payload_max_length(4096);
    http.set_keep_alive_max_count(100);
    const auto reply = [](httplib::Response& res, const Json& j) {
        res.set_header("Cache-Control", "no-store"); res.set_content(j.dump(), "application/json");
    };
    http.Get("/rg/v1/health", [&](const httplib::Request&, httplib::Response& res) {
        reply(res, {{"format", "rg.health/1"}, {"status", "ready"}, {"instance_id", instance}});
    });
    http.Get("/rg/v1/server", [&](const httplib::Request&, httplib::Response& res) {
        reply(res, {{"format", "rg.server_info/1"}, {"instance_id", instance}, {"environment", "/rg/v1/environment"},
            {"map", maps.discovery}, {"environment_content", identity}, {"player_replication", false}, {"shared_traffic", false}});
    });
    http.Get("/rg/v1/environment", [&](const httplib::Request&, httplib::Response& res) {
        std::lock_guard guard(mutex); reply(res, published);
    });
    const auto bind = cfg.at("bind").get<std::string>();
    const int bound = port == 0 ? http.bind_to_any_port(bind) : (http.bind_to_port(bind, port) ? port : -1);
    if (bound < 0) throw std::runtime_error("cannot bind environment HTTP port");
    std::atomic<bool> listen_ok{true}, listen_done{false};
    std::thread listener([&] { listen_ok = http.listen_after_bind(); listen_done = true; });
    while (!http.is_running() && !listen_done) std::this_thread::yield();
    std::signal(SIGINT, signal_handler); std::signal(SIGTERM, signal_handler);
    std::cout << "RG_SERVER_READY port=" << bound << " instance=" << instance << " map=" << maps.discovery.dump() << std::endl;
    auto previous = start, last_save = start;
    try {
        while (!interrupted && !listen_done &&
               (run_for == 0 || std::chrono::duration<double>(Clock::now() - start).count() < run_for)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            const auto now = Clock::now();
            environment.advance(std::chrono::duration<double>(now - previous).count()); previous = now;
            weather.update(rg::game_seconds(environment.clock())); environment.apply_broadcast(weather.current()); publish();
            if (std::chrono::duration<double>(now - last_save).count() >= interval) { checkpoint(); last_save = now; }
        }
        checkpoint();
    } catch (...) { http.stop(); listener.join(); throw; }
    http.stop(); listener.join();
    if (!listen_ok) throw std::runtime_error("HTTP listener failed");
    std::cout << "RG_SERVER_STOP saved=" << state_file.string() << std::endl;
    return 0;
}
} // namespace
int main(int argc, char** argv) {
    try {
        fs::path root = fs::current_path(), config = "data/server/server.json";
        double run_for = 0;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help") { std::cout << "rg_server [--root DIR] [--config FILE] [--run-for SECONDS]\n"; return 0; }
            if (i + 1 >= argc) throw std::runtime_error("missing option value");
            const std::string value = argv[++i];
            if (arg == "--root") root = fs::absolute(value);
            else if (arg == "--config") config = value;
            else if (arg == "--run-for") {
                std::size_t consumed = 0; run_for = std::stod(value, &consumed);
                if (consumed != value.size() || !std::isfinite(run_for) || run_for <= 0) throw std::runtime_error("invalid --run-for");
            } else throw std::runtime_error("unknown option " + arg);
        }
        return run(root, root / config, run_for);
    } catch (const std::exception& e) { std::cerr << "RG_SERVER_ERROR " << e.what() << '\n'; return 1; }
}
