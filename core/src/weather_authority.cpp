// weather_authority.cpp - see rg/weather_authority.h.
#include "rg/weather_authority.h"

#include "json_util.h"

#include "rg/climate.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace rg {
namespace {

using detail::json;
constexpr double kJ2000 = 2451545.0;
constexpr std::int64_t kMaxWalk = 200000; // segments per update (a ~100-year clock jump at 4 h mean)

bool fail(std::string* error, const std::string& msg) {
    if (error) *error = "weather_schedule: " + msg;
    return false;
}

std::uint64_t splitmix64(std::uint64_t x) {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}
// Uniform [0, 1) from (seed, segment index, salt): a counter hash, no sequential RNG state.
double u01(std::uint64_t seed, std::int64_t index, std::uint64_t salt) {
    const std::uint64_t h = splitmix64(seed ^ splitmix64(static_cast<std::uint64_t>(index) * 8u + salt));
    return static_cast<double>(h >> 11) * (1.0 / 9007199254740992.0);
}

// UT civil date of a game time (seconds since J2000.0 UT).
void civil_of(double game_s, CivilDate& date, double& ut_h) {
    const double jd = kJ2000 + game_s / 86400.0;
    const double z = std::floor(jd + 0.5);
    const double frac = jd + 0.5 - z;
    // days since 1970-01-01 (JD 2440587.5), then Howard Hinnant's civil_from_days.
    const std::int64_t days = static_cast<std::int64_t>(z) - 2440588;
    const std::int64_t zz = days + 719468;
    const std::int64_t era = (zz >= 0 ? zz : zz - 146096) / 146097;
    const std::int64_t doe = zz - era * 146097;
    const std::int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const std::int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const std::int64_t mp = (5 * doy + 2) / 153;
    const int d = static_cast<int>(doy - (153 * mp + 2) / 5 + 1);
    const int m = static_cast<int>(mp < 10 ? mp + 3 : mp - 9);
    const int y = static_cast<int>(yoe + era * 400 + (m <= 2 ? 1 : 0));
    date = {y, m, d};
    ut_h = frac * 24.0;
}

} // namespace

int WeatherScheduleConfig::state_index(const std::string& id) const {
    for (std::size_t i = 0; i < states.size(); ++i)
        if (states[i].id == id) return static_cast<int>(i);
    return -1;
}

const WeatherScheduleSeason& WeatherScheduleConfig::season_for_month(int month) const {
    for (const auto& s : seasons)
        if (std::find(s.months.begin(), s.months.end(), month) != s.months.end()) return s;
    return seasons.front(); // the loader guarantees every month is covered
}

std::optional<WeatherScheduleConfig> load_weather_schedule(const std::string& path, const std::vector<std::string>& preset_ids,
                                                           std::string* error) {
    const auto text = detail::read_text_file(path);
    if (!text) { fail(error, "cannot read " + path); return std::nullopt; }
    const auto root = detail::parse_json(*text);
    if (!root || !root->is_object()) { fail(error, path + " is not a JSON object"); return std::nullopt; }
    const json& j = *root;
    std::string format;
    if (!detail::get_string(j, "format", format) || format != "rg.weather_schedule/1") {
        fail(error, "format must be rg.weather_schedule/1");
        return std::nullopt;
    }
    auto known = [&](const std::string& id) { return std::find(preset_ids.begin(), preset_ids.end(), id) != preset_ids.end(); };
    WeatherScheduleConfig c;
    std::string mode;
    if (!detail::get_string(j, "default_mode", mode) || (mode != "dynamic" && mode != "fixed")) {
        fail(error, "default_mode must be \"dynamic\" or \"fixed\"");
        return std::nullopt;
    }
    c.default_dynamic = mode == "dynamic";
    double seed = 0;
    if (!detail::get_number(j, "seed", seed) || seed < 0 || seed != std::floor(seed) || seed > 9007199254740992.0) {
        fail(error, "seed must be a non-negative integer");
        return std::nullopt;
    }
    c.seed = static_cast<std::uint64_t>(seed);
    if (!detail::get_number(j, "transition_game_s", c.transition_game_s) || !(c.transition_game_s > 0) ||
        !detail::get_number(j, "min_duration_game_s", c.min_duration_game_s) || !(c.min_duration_game_s > c.transition_game_s) ||
        !detail::get_number(j, "duration_sigma", c.duration_sigma) || !(c.duration_sigma >= 0) || c.duration_sigma > 3) {
        fail(error, "need transition_game_s > 0, min_duration_game_s > transition_game_s, duration_sigma in [0, 3]");
        return std::nullopt;
    }
    const auto st = j.find("states");
    if (st == j.end() || !st->is_object() || st->empty()) { fail(error, "states must be a non-empty object"); return std::nullopt; }
    for (auto it = st->begin(); it != st->end(); ++it) {
        WeatherScheduleState s;
        s.id = it.key();
        if (!known(s.id)) { fail(error, "states." + s.id + " is not a sky.json weather preset"); return std::nullopt; }
        const json& o = it.value();
        if (!o.is_object() || !detail::get_number(o, "mean_hours", s.mean_hours) || !(s.mean_hours > 0) || s.mean_hours > 1000) {
            fail(error, "states." + s.id + ".mean_hours must be in (0, 1000]");
            return std::nullopt;
        }
        const bool has_max = o.contains("max_air_temp_c"), has_min = o.contains("min_air_temp_c");
        if (has_max && (!detail::get_number(o, "max_air_temp_c", s.max_air_temp_c) || !detail::get_string(o, "too_warm", s.too_warm))) {
            fail(error, "states." + s.id + ": max_air_temp_c needs a number and too_warm");
            return std::nullopt;
        }
        if (has_min && (!detail::get_number(o, "min_air_temp_c", s.min_air_temp_c) || !detail::get_string(o, "too_cold", s.too_cold))) {
            fail(error, "states." + s.id + ": min_air_temp_c needs a number and too_cold");
            return std::nullopt;
        }
        if (!has_max && o.contains("too_warm")) { fail(error, "states." + s.id + ": too_warm without max_air_temp_c"); return std::nullopt; }
        if (!has_min && o.contains("too_cold")) { fail(error, "states." + s.id + ": too_cold without min_air_temp_c"); return std::nullopt; }
        c.states.push_back(s);
    }
    // nlohmann's object is key-sorted, so the state order (and with it every hash pick) is the same on every build.
    for (const auto& s : c.states) {
        if (!s.too_warm.empty() && c.state_index(s.too_warm) < 0) { fail(error, "states." + s.id + ".too_warm names no state"); return std::nullopt; }
        if (!s.too_cold.empty() && c.state_index(s.too_cold) < 0) { fail(error, "states." + s.id + ".too_cold names no state"); return std::nullopt; }
    }
    const auto se = j.find("seasons");
    if (se == j.end() || !se->is_array() || se->empty()) { fail(error, "seasons must be a non-empty array"); return std::nullopt; }
    std::set<int> months_seen;
    for (const json& o : *se) {
        WeatherScheduleSeason season;
        if (!o.is_object() || !detail::get_string(o, "id", season.id)) { fail(error, "each season needs an id"); return std::nullopt; }
        const auto m = o.find("months");
        if (m == o.end() || !m->is_array() || m->empty()) { fail(error, "season " + season.id + ": months must be a non-empty array"); return std::nullopt; }
        for (const json& v : *m) {
            if (!v.is_number_integer() || v.get<int>() < 1 || v.get<int>() > 12 || !months_seen.insert(v.get<int>()).second) {
                fail(error, "season " + season.id + ": months must be distinct integers 1..12 used by one season only");
                return std::nullopt;
            }
            season.months.push_back(v.get<int>());
        }
        const auto nx = o.find("next");
        if (nx == o.end() || !nx->is_object()) { fail(error, "season " + season.id + ": next must be an object"); return std::nullopt; }
        season.next.resize(c.states.size());
        for (std::size_t i = 0; i < c.states.size(); ++i) {
            const auto row = nx->find(c.states[i].id);
            if (row == nx->end() || !row->is_object() || row->empty()) {
                fail(error, "season " + season.id + ": next." + c.states[i].id + " must be a non-empty object");
                return std::nullopt;
            }
            for (auto t = row->begin(); t != row->end(); ++t) {
                if (c.state_index(t.key()) < 0 || !t.value().is_number() || !(t.value().get<double>() > 0)) {
                    fail(error, "season " + season.id + ": next." + c.states[i].id + "." + t.key() + " needs a state and a weight > 0");
                    return std::nullopt;
                }
                season.next[i].push_back({t.key(), t.value().get<double>()});
            }
        }
        for (auto t = nx->begin(); t != nx->end(); ++t)
            if (c.state_index(t.key()) < 0) { fail(error, "season " + season.id + ": next." + t.key() + " names no state"); return std::nullopt; }
        c.seasons.push_back(std::move(season));
    }
    if (months_seen.size() != 12) { fail(error, "seasons must cover all 12 months"); return std::nullopt; }
    return c;
}

std::string to_json(const WeatherBroadcast& b) {
    json j;
    j["format"] = "rg.weather_broadcast/1";
    j["revision"] = b.revision;
    j["mode"] = b.dynamic ? "dynamic" : "fixed";
    j["weather"] = b.weather_id;
    j["from"] = b.from_id;
    j["change_game_s"] = b.change_game_s;
    j["transition_s"] = b.transition_s;
    if (std::isfinite(b.next_change_game_s)) j["next_change_game_s"] = b.next_change_game_s;
    return j.dump();
}

std::optional<WeatherBroadcast> parse_weather_broadcast(const std::string& text, std::string* error) {
    auto bad = [&](const char* msg) {
        if (error) *error = std::string("weather_broadcast: ") + msg;
        return std::optional<WeatherBroadcast>{};
    };
    const auto root = detail::parse_json(text);
    if (!root || !root->is_object()) return bad("not a JSON object");
    const json& j = *root;
    std::string format, mode;
    if (!detail::get_string(j, "format", format) || format != "rg.weather_broadcast/1") return bad("format must be rg.weather_broadcast/1");
    WeatherBroadcast b;
    const auto rev = j.find("revision");
    if (rev == j.end() || !rev->is_number_unsigned()) return bad("revision must be an unsigned integer");
    b.revision = rev->get<std::uint64_t>();
    if (!detail::get_string(j, "mode", mode) || (mode != "dynamic" && mode != "fixed")) return bad("mode must be dynamic or fixed");
    b.dynamic = mode == "dynamic";
    if (!detail::get_string(j, "weather", b.weather_id) || b.weather_id.empty() || !detail::get_string(j, "from", b.from_id))
        return bad("weather and from must be strings");
    if (!detail::get_number(j, "change_game_s", b.change_game_s) || !std::isfinite(b.change_game_s) ||
        !detail::get_number(j, "transition_s", b.transition_s) || !(b.transition_s >= 0) || !std::isfinite(b.transition_s))
        return bad("change_game_s must be finite and transition_s >= 0");
    if (j.contains("next_change_game_s") &&
        (!detail::get_number(j, "next_change_game_s", b.next_change_game_s) || !std::isfinite(b.next_change_game_s)))
        return bad("next_change_game_s must be a number");
    return b;
}

double game_seconds(const TimeController& clock) { return (clock.julian_day_ut() - kJ2000) * 86400.0; }

WeatherAuthority::WeatherAuthority(WeatherScheduleConfig schedule, std::vector<WeatherPreset> presets,
                                   std::shared_ptr<const ClimateConfig> climate, GeoLocation location, double now_game_s,
                                   const std::string& initial_id, std::uint64_t last_revision)
    : schedule_(std::move(schedule)), presets_(std::move(presets)), climate_(std::move(climate)), location_(location) {
    current_.revision = last_revision;
    current_.weather_id = current_.from_id = initial_id;
    current_.change_game_s = now_game_s;
    if (schedule_.default_dynamic) set_dynamic(now_game_s);
    else set_fixed(initial_id, now_game_s);
}

double WeatherAuthority::duration(std::int64_t index, int state) const {
    // Log-normal with the state's mean: z from an Irwin-Hall n = 4 sum (variance 1/3).
    double sum = 0;
    for (std::uint64_t k = 0; k < 4; ++k) sum += u01(schedule_.seed, index, 1 + k);
    const double z = (sum - 2.0) * std::sqrt(3.0);
    const double sigma = schedule_.duration_sigma;
    const double mean_s = schedule_.states[static_cast<std::size_t>(state)].mean_hours * 3600.0;
    return std::max(schedule_.min_duration_game_s, mean_s * std::exp(sigma * z - 0.5 * sigma * sigma));
}

int WeatherAuthority::gate(int state, double at_game_s) const {
    if (!climate_) return state;
    for (int pass = 0; pass < 2; ++pass) {
        const WeatherScheduleState& s = schedule_.states[static_cast<std::size_t>(state)];
        double cover = 0;
        for (const auto& p : presets_)
            if (p.id == s.id) cover = p.state.cloud_cover;
        CivilDate d;
        double ut = 0;
        civil_of(at_game_s, d, ut);
        const double t = climate_station_temperature_c(*climate_, d, ut, location_.longitude_deg, cover);
        if (t > s.max_air_temp_c) state = schedule_.state_index(s.too_warm);
        else if (t < s.min_air_temp_c) state = schedule_.state_index(s.too_cold);
        else break;
    }
    return state;
}

WeatherAuthority::Segment WeatherAuthority::first_segment() const {
    Segment s;
    s.index = 0;
    s.state = s.from = initial_;
    s.start = epoch_;
    s.end = epoch_ + duration(0, initial_);
    return s;
}

WeatherAuthority::Segment WeatherAuthority::next_segment(const Segment& prev) const {
    Segment s;
    s.index = prev.index + 1;
    s.from = prev.state;
    s.start = prev.end;
    CivilDate d;
    double ut = 0;
    civil_of(s.start, d, ut);
    const auto& row = schedule_.season_for_month(d.month).next[static_cast<std::size_t>(prev.state)];
    double total = 0;
    for (const auto& t : row) total += t.weight;
    double pick = u01(schedule_.seed, s.index, 0) * total;
    s.state = schedule_.state_index(row.back().to);
    for (const auto& t : row) {
        if (pick < t.weight) { s.state = schedule_.state_index(t.to); break; }
        pick -= t.weight;
    }
    s.state = gate(s.state, s.start + 0.5 * schedule_.transition_game_s);
    s.end = s.start + duration(s.index, s.state);
    return s;
}

void WeatherAuthority::publish(int state, int from, double change, double transition, double next_change, bool dynamic) {
    const auto id = [&](int i) { return schedule_.states[static_cast<std::size_t>(i)].id; };
    current_.revision += 1;
    current_.dynamic = dynamic;
    current_.from_id = from == state ? id(state) : id(from);
    current_.weather_id = id(state);
    current_.change_game_s = change;
    current_.transition_s = transition;
    current_.next_change_game_s = next_change;
}

bool WeatherAuthority::update(double now) {
    if (!dynamic_ || !std::isfinite(now)) return false;
    const std::int64_t before = seg_.index;
    const double before_start = seg_.start;
    if (now < seg_.start) seg_ = first_segment(); // a clock jump backwards replays the chain from the epoch
    for (std::int64_t n = 0; now >= seg_.end && n < kMaxWalk; ++n) seg_ = next_segment(seg_);
    if (seg_.index == before && seg_.start == before_start) return false;
    // The segment's blend from the previous preset; segment 0 starts settled.
    publish(seg_.state, seg_.from, seg_.start, seg_.index == 0 ? 0.0 : schedule_.transition_game_s, seg_.end, true);
    return true;
}

bool WeatherAuthority::set_fixed(const std::string& weather_id, double now) {
    if (std::none_of(presets_.begin(), presets_.end(), [&](const WeatherPreset& p) { return p.id == weather_id; })) return false;
    dynamic_ = false;
    current_.revision += 1;
    current_.dynamic = false;
    current_.from_id = current_.weather_id;
    current_.weather_id = weather_id;
    current_.change_game_s = now;
    current_.transition_s = 0;
    current_.next_change_game_s = std::numeric_limits<double>::quiet_NaN();
    return true;
}

void WeatherAuthority::set_dynamic(double now) {
    // The chain starts in the current weather (or the first state when the schedule does not know it).
    const int i = schedule_.state_index(current_.weather_id);
    initial_ = i >= 0 ? i : 0;
    epoch_ = now;
    dynamic_ = true;
    seg_ = first_segment();
    current_.revision += 1;
    current_.dynamic = true;
    current_.from_id = current_.weather_id;
    current_.weather_id = schedule_.states[static_cast<std::size_t>(initial_)].id;
    current_.change_game_s = now;
    current_.transition_s = 0;
    current_.next_change_game_s = seg_.end;
}

bool WeatherAuthority::handle_request(const std::string& menu_id, double now) {
    static const std::string kPrefix = "weather:";
    if (menu_id.compare(0, kPrefix.size(), kPrefix) != 0) return false;
    const std::string key = menu_id.substr(kPrefix.size());
    if (key == "auto") {
        if (!dynamic_) set_dynamic(now);
        return true;
    }
    return set_fixed(key, now);
}

} // namespace rg
