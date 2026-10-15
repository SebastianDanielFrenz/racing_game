// weather_sync.cpp - see rg/weather_sync.h.
#include "rg/weather_sync.h"

#include "rg/session.h"

#include <cmath>

namespace rg {

std::optional<SessionWeather> session_weather_for(const TimeWeather& tw, std::string* error) {
    if (!tw.climate()) {
        if (error) *error = "weather: no climate";
        return std::nullopt;
    }
    CivilDate d;
    double ut = 0;
    tw.clock().ut_instant(d, ut);
    const TimeWeatherState s = tw.state();
    const GeoLocation& loc = tw.location();
    return make_session_weather(*tw.climate(), loc.latitude_deg, loc.longitude_deg, loc.grid_convergence_deg, d, ut,
                                tw.clock().rate(), s.weather_id, error);
}

int WeatherSync::sync(const TimeWeather& tw, const WeatherSnapshot& published, Session& session) {
    if (!tw.climate() || !session.weather_installed()) return None;
    const ClimateConfig& c = *tw.climate();
    CivilDate d;
    double ut = 0;
    tw.clock().ut_instant(d, ut);
    const EnvClockPoint p = env_clock_point(d, ut, tw.location().longitude_deg);
    const double rate = tw.clock().rate();
    int sent = None;
    const bool drift = published.installed &&
                       std::fabs(published.env_time_s - p.env_s) > std::fmax(c.resync_clock_s, c.resync_clock_real_s * rate);
    if (first_ || tw.clock().revision() != clock_revision_ || p.day_index != day_index_ || drift) {
        session.set_environment_clock(p.env_s, p.day_index, rate);
        clock_revision_ = tw.clock().revision();
        day_index_ = p.day_index;
        sent |= Clock;
    }
    const TimeWeatherState s = tw.state();
    if (std::isfinite(s.air_temp_c) &&
        (first_ || s.weather_id != weather_id_ || std::fabs(s.air_temp_c - temp_c_) >= c.resync_temperature_k)) {
        if (const auto preset = physics_weather_preset(c, s.weather_id, s.air_temp_c)) {
            // A temperature refresh ramps in over one second; a new preset over the climate's weather ramp.
            const bool new_weather = s.weather_id != weather_id_ && !first_;
            session.set_weather(*preset, new_weather ? c.weather_ramp_s : 1.0);
            sent |= Weather;
        }
        weather_id_ = s.weather_id;
        temp_c_ = s.air_temp_c;
    }
    first_ = false;
    return sent;
}

} // namespace rg
