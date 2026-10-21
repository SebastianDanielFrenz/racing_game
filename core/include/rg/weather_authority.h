// rg/weather_authority.h - who decides the weather (owner 2026-10-21: "progressing weather ... it needs to come from a
// server later when playing on a server"). Engine-neutral, no physics World.
//
// WeatherAuthority is the server role: it owns the weather mode (dynamic = a seeded semi-Markov chain over the
// sky.json weather presets, data/world/weather_schedule.json; fixed = one preset the player or host picked) and emits
// a WeatherBroadcast whenever the weather changes. Clients never run the chain: rg::TimeWeather::apply_broadcast()
// takes a broadcast and blends the sky, the widget and (through rg::WeatherSync) the physics weather towards it. In
// single-player TimeWeather owns a local authority (the in-process "server"); on a network the server runs the
// authority next to the game clock, sends to_json(broadcast) to every client (also the latest one to a late joiner)
// and receives the clients' menu choices as requests (TimeWeather::take_weather_requests()).
//
// Game time = seconds since J2000.0 UT, game_seconds(TimeController). The chain is a pure function of
// (schedule, seed, epoch, initial preset, game time): segment k's preset and duration come from a counter hash of
// (seed, k), so the same authority state gives the same weather on any machine, and a clock jump backwards replays
// the chain from the epoch.
#pragma once

#include "rg/time_weather.h"

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rg {

struct ClimateConfig;

// data/world/weather_schedule.json (rg.weather_schedule/1).
struct WeatherScheduleState {
    std::string id;           // a sky.json weather preset id
    double mean_hours = 4;    // mean segment length in game hours
    // Temperature gate at the segment's mid-transition (needs a climate): above max -> too_warm, below min -> too_cold.
    double max_air_temp_c = std::numeric_limits<double>::infinity();
    std::string too_warm;
    double min_air_temp_c = -std::numeric_limits<double>::infinity();
    std::string too_cold;
};
struct WeatherScheduleTransition {
    std::string to;
    double weight = 0;
};
struct WeatherScheduleSeason {
    std::string id;
    std::vector<int> months;                                // 1..12 (the UT month at the segment's start)
    std::vector<std::vector<WeatherScheduleTransition>> next; // indexed like WeatherScheduleConfig::states
};
struct WeatherScheduleConfig {
    bool default_dynamic = true;
    std::uint64_t seed = 1;
    double transition_game_s = 1800;   // sky/physics blend from one preset to the next, game seconds
    double min_duration_game_s = 2700; // > transition_game_s
    double duration_sigma = 0.5;       // log-normal spread of the segment length
    std::vector<WeatherScheduleState> states;
    std::vector<WeatherScheduleSeason> seasons; // every month in exactly one season
    int state_index(const std::string& id) const;
    const WeatherScheduleSeason& season_for_month(int month) const;
};
// preset_ids: the sky.json weather presets the states and transitions must name.
std::optional<WeatherScheduleConfig> load_weather_schedule(const std::string& path, const std::vector<std::string>& preset_ids,
                                                           std::string* error);

// WeatherBroadcast (what a server sends) lives in rg/time_weather.h, the client side.
std::string to_json(const WeatherBroadcast& b);
std::optional<WeatherBroadcast> parse_weather_broadcast(const std::string& text, std::string* error);

// Seconds since J2000.0 UT of the clock's instant.
double game_seconds(const TimeController& clock);

class WeatherAuthority {
public:
    // presets: the sky.json weather presets (cloud covers for the temperature gate); climate optional (no gate
    // without); now_game_s: the epoch; initial_id: the weather it starts in.
    WeatherAuthority(WeatherScheduleConfig schedule, std::vector<WeatherPreset> presets,
                     std::shared_ptr<const ClimateConfig> climate, GeoLocation location, double now_game_s,
                     const std::string& initial_id, std::uint64_t last_revision = 0);
    // Advances (or rewinds) the chain to game time t; true when current() changed.
    bool update(double now_game_s);
    // Requests (a menu choice, or a client's request on a server). False for an unknown id.
    bool set_fixed(const std::string& weather_id, double now_game_s);
    void set_dynamic(double now_game_s);
    // "weather:<id>" or "weather:auto"; false otherwise.
    bool handle_request(const std::string& menu_id, double now_game_s);
    const WeatherBroadcast& current() const { return current_; }
    const WeatherScheduleConfig& schedule() const { return schedule_; }

private:
    struct Segment {
        std::int64_t index = 0;
        int state = 0, from = 0;
        double start = 0, end = 0;
    };
    Segment first_segment() const;
    Segment next_segment(const Segment& s) const;
    double duration(std::int64_t index, int state) const;
    int gate(int state, double at_game_s) const;
    void publish(int state, int from, double change, double transition, double next_change, bool dynamic);

    WeatherScheduleConfig schedule_;
    std::vector<WeatherPreset> presets_;
    std::shared_ptr<const ClimateConfig> climate_;
    GeoLocation location_;
    bool dynamic_ = false;
    double epoch_ = 0;
    int initial_ = 0;
    Segment seg_;
    WeatherBroadcast current_;
};

} // namespace rg
