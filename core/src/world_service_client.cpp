#include "rg/world_service_client.h"
#include "rg/weather_authority.h"
#include "g2m/http/http_transport.h"
#include "g2m/client/cache.h"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <mutex>
#include <thread>

namespace rg {
namespace {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
Json get_json(const std::string& base, const char* route) {
    // HttpTransport's parser also rejects credentials, queries, fragments and HTTPS.
    g2m::http::HttpTransportConfig check; check.base_url = base;
    auto valid = g2m::http::HttpTransport::create(check);
    if (!valid) throw std::runtime_error(valid.error().message);
    const auto& parsed = (*valid)->url();
    httplib::Client client(parsed.host, parsed.port);
    client.set_connection_timeout(2, 0); client.set_read_timeout(2, 0); client.set_write_timeout(2, 0);
    client.set_payload_max_length(1024 * 1024);
    auto response = client.Get(parsed.base_path + route);
    if (!response || response->status != 200) throw std::runtime_error("world service unavailable: " + base + route);
    return Json::parse(response->body);
}
double finite_number(const Json& j, const char* key, double lo, double hi) {
    if (!j.at(key).is_number()) throw std::runtime_error(std::string("non-number ") + key);
    const double v = j.at(key).get<double>();
    if (!std::isfinite(v) || v < lo || v > hi) throw std::runtime_error(std::string("invalid ") + key);
    return v;
}
int date_integer(const Json& j, const char* key, int lo, int hi) {
    if (!j.at(key).is_number_integer()) throw std::runtime_error("invalid date integer");
    return static_cast<int>(finite_number(j, key, lo, hi));
}
RemoteEnvironment decode(const Json& j) {
    if (j.at("format") != "rg.environment/1") throw std::runtime_error("unsupported environment protocol");
    RemoteEnvironment e;
    e.instance = j.at("instance_id").get<std::string>();
    if (e.instance.empty() || e.instance.size() > 128) throw std::runtime_error("invalid server instance");
    if (!j.at("server_us").is_number_integer()) throw std::runtime_error("invalid server time");
    e.server_us = static_cast<std::int64_t>(finite_number(j, "server_us", 0, 9e15));
    e.game_s = finite_number(j, "game_seconds_j2000", -1e12, 1e12);
    e.local_time_h = finite_number(j, "local_time_h", 0, 24);
    e.rate = finite_number(j, "rate", 0, 100000);
    const auto& d = j.at("date");
    e.date = {date_integer(d, "year", 1, 9999), date_integer(d, "month", 1, 12), date_integer(d, "day", 1, 31)};
    if (e.date.day > days_in_month(e.date.year, e.date.month)) throw std::runtime_error("invalid server date");
    const auto offset = finite_number(j, "utc_offset_h", -24, 24);
    const double declared = (julian_day(e.date, e.local_time_h - offset) - 2451545.0) * 86400;
    if (std::abs(declared - e.game_s) > 0.01) throw std::runtime_error("inconsistent server clocks");
    std::string error;
    auto b = parse_weather_broadcast(j.at("weather").dump(), &error);
    if (!b) throw std::runtime_error(error);
    e.weather = *b;
    return e;
}
class CachedTransport final : public g2m::ITransport {
public:
    std::unique_ptr<g2m::http::HttpTransport> remote;
    std::unique_ptr<g2m::ClientCache> cache;
    g2m::Response send(const g2m::Request& request) override {
        const auto* tile = std::get_if<g2m::TileRequest>(&request);
        if (tile) {
            auto hit = cache->get(tile->rid, tile->layer, tile->key);
            if (hit && *hit) {
                g2m::TileResponse response;
                response.container = std::move(**hit);
                auto header = g2m::decode_header(response.container);
                if (header) response.meta.etag = header->body_hash;
                if (tile->if_none_match && response.meta.etag == tile->if_none_match) {
                    response.meta.status = g2m::Status::NotModified; response.container.clear();
                }
                return response;
            }
        }
        auto response = remote->send(request);
        if (auto* t = std::get_if<g2m::TileResponse>(&response); t && t->meta.status == g2m::Status::Ok) {
            auto saved = cache->put(t->container);
            // Cache failure must not alter the validated live tile response.
            (void)saved;
        }
        return response;
    }
};
} // namespace
std::string world_server_url() {
    const char* s = std::getenv("RG_SERVER_URL");
    std::string url = s ? s : "";
    while (!url.empty() && url.back() == '/') url.pop_back();
    return url;
}
struct WorldServiceClient::Impl {
    mutable std::mutex mutex;
    std::condition_variable wake;
    bool stop = false;
    std::optional<RemoteEnvironment> latest;
    std::string state = "connecting";
    std::thread worker;
    std::string applied_instance;
    explicit Impl(std::string url, std::string sky_path, std::string climate_path)
        : worker([this, url = std::move(url), sky_path = std::move(sky_path), climate_path = std::move(climate_path)] {
        double min_rtt = std::numeric_limits<double>::infinity();
        std::string validated_instance;
        GeoLocation location;
        while (true) {
            try {
                const auto before = Clock::now();
                auto e = decode(get_json(url, "/rg/v1/environment"));
                if (validated_instance != e.instance) {
                    const auto info = get_json(url, "/rg/v1/server");
                    if (info.at("format") != "rg.server_info/1" || info.at("instance_id") != e.instance)
                        throw std::runtime_error("server restarted during handshake; retrying");
                    const auto& content = info.at("environment_content");
                    if (!sky_path.empty()) {
                        std::ifstream sky(sky_path), climate(climate_path);
                        if (!sky || !climate || Json::parse(sky) != content.at("sky") || Json::parse(climate) != content.at("climate"))
                            throw std::runtime_error("server sky/climate content differs from this client");
                    }
                    location.latitude_deg = finite_number(content, "latitude_deg", -90, 90);
                    location.longitude_deg = finite_number(content, "longitude_deg", -180, 180);
                    validated_instance = e.instance;
                }
                e.location = location;
                e.received = Clock::now();
                const double rtt = std::chrono::duration<double>(e.received - before).count();
                std::lock_guard guard(mutex);
                if (latest && latest->instance == e.instance && e.server_us < latest->server_us)
                    throw std::runtime_error("server clock moved backwards");
                if (!latest || latest->instance != e.instance) min_rtt = rtt;
                min_rtt = std::min(min_rtt, rtt);
                // A cached 20 Hz snapshot can be up to 50 ms old; never infer precision below that.
                e.half_rtt_s = min_rtt * 0.5;
                latest = std::move(e); state = "connected";
            } catch (const std::exception& e) { std::lock_guard guard(mutex); state = e.what(); }
            std::unique_lock lock(mutex);
            if (wake.wait_for(lock, std::chrono::milliseconds(500), [&] { return stop; })) break;
        }
    }) {}
    ~Impl() { { std::lock_guard guard(mutex); stop = true; } wake.notify_all(); worker.join(); }
};
WorldServiceClient::WorldServiceClient(std::string url, std::string sky, std::string climate)
    : impl_(std::make_unique<Impl>(std::move(url), std::move(sky), std::move(climate))) {}
WorldServiceClient::~WorldServiceClient() = default;
std::optional<RemoteEnvironment> WorldServiceClient::sample() const { std::lock_guard guard(impl_->mutex); return impl_->latest; }
std::string WorldServiceClient::status() const {
    std::lock_guard guard(impl_->mutex);
    if (impl_->latest && std::chrono::duration<double>(Clock::now() - impl_->latest->received).count() > 3)
        return "World server disconnected; time/weather held";
    return impl_->state;
}
void WorldServiceClient::advance(TimeWeather& tw, double dt) {
    const auto e = sample();
    if (!e) return; // Connecting must not run a separate local weather authority.
    const double age = std::chrono::duration<double>(Clock::now() - e->received).count();
    if (age > 3) return; // Freeze stale authority, instead of inventing weather through an outage.
    const auto known = [&](const std::string& id) {
        for (const auto& p : tw.config().weather_presets) if (p.id == id) return true;
        return false;
    };
    if (!known(e->weather.weather_id) || !known(e->weather.from_id)) {
        std::lock_guard guard(impl_->mutex); impl_->state = "Unsupported server weather preset"; return;
    }
    if (impl_->applied_instance != e->instance) {
        const auto climate = tw.climate();
        auto location = e->location; location.grid_convergence_deg = tw.location().grid_convergence_deg;
        TimeWeather fresh(tw.config(), location);
        fresh.set_climate(climate); fresh.use_remote_weather();
        fresh.clock().set_date(e->date); fresh.clock().set_local_time_h(e->local_time_h); fresh.clock().set_rate(e->rate);
        fresh.advance(age + e->half_rtt_s);
        tw = std::move(fresh);
        impl_->applied_instance = e->instance;
    } else {
        if (tw.clock().rate() != e->rate) tw.clock().set_rate(e->rate);
        const double target = e->game_s + (age + e->half_rtt_s) * e->rate;
        const double gap = target - game_seconds(tw.clock());
        if (e->rate > 0 && gap > 0) {
            const double step = gap > 5 * std::max(1.0, e->rate) ? gap : std::min(gap, std::max(0.0, dt) * e->rate * 1.1);
            tw.advance(step / e->rate);
        }
    }
    tw.apply_broadcast(e->weather);
}
bool configure_remote_terrain(g2m::world::TerrainServiceConfig& cfg, const std::string& url,
                              const std::string& cache_dir, std::string* error) {
    try {
        auto discovery = get_json(url, "/rg/v1/server");
        if (discovery.at("format") != "rg.server_info/1") throw std::runtime_error("unsupported server discovery");
        const auto& map = discovery.at("map");
        if (!map.at("enabled").get<bool>()) throw std::runtime_error("server has terrain delivery disabled");
        auto rid = g2m::content_hash_from_hex(map.at("release_id").get<std::string>());
        if (!rid) throw std::runtime_error("invalid pinned terrain release id");
        g2m::http::HttpTransportConfig hc; hc.base_url = map.at("base_url").get<std::string>();
        hc.connect_timeout_ms = 2000; hc.read_timeout_ms = 20000; hc.max_body_bytes = 128 * 1024 * 1024;
        auto transport = g2m::http::HttpTransport::create(hc);
        if (!transport) throw std::runtime_error(transport.error().message);
        auto response = (*transport)->send(g2m::ReleaseManifestRequest{*rid});
        const auto* manifest = std::get_if<g2m::ReleaseManifestResponse>(&response);
        if (!manifest || manifest->meta.status != g2m::Status::Ok || manifest->manifest.rid != *rid)
            throw std::runtime_error("cannot fetch pinned terrain release manifest");
        auto cache = g2m::ClientCache::open({cache_dir});
        if (!cache) throw std::runtime_error(cache.error().message);
        auto cached = std::make_shared<CachedTransport>();
        cached->remote = std::move(*transport); cached->cache = std::move(*cache);
        cfg.remote_transport = std::move(cached); cfg.remote_manifest = manifest->manifest;
        return true;
    } catch (const std::exception& e) { if (error) *error = e.what(); return false; }
}
} // namespace rg
