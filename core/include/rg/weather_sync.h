// rg/weather_sync.h - feeds the game clock and the chosen weather of an
// rg::TimeWeather (with a climate) into a Session's physics environment (V8).
// Engine-neutral; the Godot adapter calls sync() from the sky controller's tick.
#pragma once

#include "rg/climate.h"
#include "rg/time_weather.h"

#include <cstdint>
#include <optional>
#include <string>

namespace rg {

class Session;
struct WeatherSnapshot;

// SessionConfig::weather for a new Session from the current clock and weather
// (nullopt + error without a climate or on a physics validation failure).
std::optional<SessionWeather> session_weather_for(const TimeWeather& tw, std::string* error);

class WeatherSync {
public:
    enum Sent : int { None = 0, Clock = 1, Weather = 2 };
    // Call once per new Session (a world switch) so the first sync sends everything.
    void reset() { *this = WeatherSync{}; }
    // Sends the clock when the game clock jumped (TimeController::revision), changed day or drifted from the
    // environment's published time (`published`, FrameSnapshot::weather) by more than the climate's resync window;
    // the weather when the preset changed or its pinned temperature moved by resync_temperature_k. Returns Sent bits.
    int sync(const TimeWeather& tw, const WeatherSnapshot& published, Session& session);

private:
    bool first_ = true;
    std::uint64_t clock_revision_ = 0;
    int day_index_ = -1;
    std::string weather_id_;
    double temp_c_ = 0;
};

} // namespace rg
