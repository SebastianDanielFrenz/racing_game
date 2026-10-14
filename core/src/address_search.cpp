// address_search.cpp - see rg/address_search.h. Ported from game/scripts/address_teleport.gd; the hand-written
// transverse Mercator series there is replaced by geo2map's g2m::geo::Utm (Karney series, WGS84).
#include "rg/address_search.h"

#include "json_util.h"

#include "g2m/core/geo/ellipsoid.h"
#include "g2m/core/geo/utm.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace rg {

namespace {

std::string_view trim(std::string_view s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
    return s;
}

std::optional<double> parse_number(std::string_view text) {
    const std::string s(trim(text));
    if (s.empty()) return std::nullopt;
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end != s.c_str() + s.size() || !std::isfinite(v)) return std::nullopt;
    return v;
}

// A property value as text: strings as they are, numbers without a trailing ".0", anything else empty.
std::string text_of(const detail::json& v) {
    if (v.is_string()) return v.get<std::string>();
    if (v.is_number_integer()) return std::to_string(v.get<long long>());
    if (v.is_number()) {
        const double d = v.get<double>();
        char buf[64];
        if (std::floor(d) == d && std::fabs(d) < 1e15) std::snprintf(buf, sizeof buf, "%.0f", d);
        else std::snprintf(buf, sizeof buf, "%g", d);
        return buf;
    }
    return {};
}

std::string property(const detail::json& props, const char* key) {
    const auto it = props.find(key);
    return it == props.end() ? std::string{} : text_of(*it);
}

std::string format_coordinate(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.7f", v);
    return buf;
}

} // namespace

std::optional<GeocodingConfig> load_geocoding_config(const std::string& path, std::string* err) {
    const auto fail = [err](const std::string& m) -> std::optional<GeocodingConfig> {
        if (err) *err = m;
        return std::nullopt;
    };
    const auto text = detail::read_text_file(path);
    if (!text) return fail("geocoding config: cannot read " + path);
    const auto j = detail::parse_json(*text);
    if (!j || !j->is_object()) return fail("geocoding config: " + path + " is not a JSON object");
    GeocodingConfig c;
    double limit = 0.0;
    if (!detail::get_string(*j, "endpoint", c.endpoint) || c.endpoint.empty())
        return fail("geocoding config: 'endpoint' must be a non-empty string");
    if (!detail::get_number(*j, "maximum_distance_from_origin_m", c.maximum_distance_from_origin_m) ||
        !(c.maximum_distance_from_origin_m > 0.0))
        return fail("geocoding config: 'maximum_distance_from_origin_m' must be a positive number");
    if (!detail::get_number(*j, "result_limit", limit) || limit < 1.0 || limit > 50.0 || std::floor(limit) != limit)
        return fail("geocoding config: 'result_limit' must be an integer 1..50");
    c.result_limit = static_cast<int>(limit);
    if (!detail::get_string(*j, "language", c.language)) return fail("geocoding config: 'language' must be a string");
    if (!detail::get_number(*j, "min_request_interval_s", c.min_request_interval_s) || !(c.min_request_interval_s >= 0.0))
        return fail("geocoding config: 'min_request_interval_s' must be a number >= 0");
    return c;
}

std::optional<AddressMatch> parse_coordinate_query(std::string_view query) {
    const std::string_view q = trim(query);
    const auto comma = q.find(',');
    if (comma == std::string_view::npos || q.find(',', comma + 1) != std::string_view::npos) return std::nullopt;
    const auto lat = parse_number(q.substr(0, comma));
    const auto lon = parse_number(q.substr(comma + 1));
    if (!lat || !lon) return std::nullopt;
    return AddressMatch{std::string(q), *lat, *lon};
}

std::string address_cache_key(std::string_view query) {
    std::string key(trim(query));
    for (char& ch : key) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    return key;
}

std::string uri_encode(std::string_view text) {
    static constexpr char kHex[] = "0123456789ABCDEF";
    std::string out;
    out.reserve(text.size() * 3);
    for (const char ch : text) {
        const auto c = static_cast<unsigned char>(ch);
        if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            out.push_back(static_cast<char>(c));
        } else {
            out.push_back('%');
            out.push_back(kHex[c >> 4]);
            out.push_back(kHex[c & 15]);
        }
    }
    return out;
}

std::string geocoding_url(const GeocodingConfig& config, std::string_view query, const std::optional<GeoPoint>& bias) {
    std::string url = config.endpoint;
    url += config.endpoint.find('?') != std::string::npos ? '&' : '?';
    url += "q=" + uri_encode(trim(query)) + "&limit=" + std::to_string(config.result_limit);
    if (!config.language.empty()) url += "&lang=" + uri_encode(config.language);
    if (bias && std::isfinite(bias->lat) && std::isfinite(bias->lon))
        url += "&lat=" + format_coordinate(bias->lat) + "&lon=" + format_coordinate(bias->lon);
    return url;
}

std::optional<std::vector<AddressMatch>> parse_geocoding_response(std::string_view body) {
    const auto j = detail::parse_json(std::string(body));
    if (!j || !j->is_object()) return std::nullopt;
    const auto features = j->find("features");
    if (features == j->end() || !features->is_array()) return std::nullopt;
    std::vector<AddressMatch> matches;
    for (const auto& feature : *features) {
        if (!feature.is_object()) continue;
        const auto geometry = feature.find("geometry");
        if (geometry == feature.end() || !geometry->is_object()) continue;
        const auto coords = geometry->find("coordinates");
        if (coords == geometry->end() || !coords->is_array() || coords->size() < 2) continue;
        if (!(*coords)[0].is_number() || !(*coords)[1].is_number()) continue;
        const auto props = feature.find("properties");
        if (props == feature.end() || !props->is_object()) continue;
        std::vector<std::string> parts;
        std::string street = props->contains("street") ? property(*props, "street") : property(*props, "name");
        const std::string number = property(*props, "housenumber");
        if (!street.empty()) parts.emplace_back(std::string(trim(street + " " + number)));
        for (const char* key : {"postcode", "city", "state", "country"}) {
            std::string part = property(*props, key);
            if (!part.empty()) parts.push_back(std::move(part));
        }
        std::string label;
        for (std::size_t i = 0; i < parts.size(); ++i) label += (i ? ", " : "") + parts[i];
        matches.push_back({label, (*coords)[1].get<double>(), (*coords)[0].get<double>()});
    }
    return matches;
}

Destination check_destination(double lat, double lon, int utm_zone, double e0, double n0, double maximum_distance_m) {
    Destination d;
    if (!std::isfinite(lat) || !std::isfinite(lon) || lat < 0.0 || lat >= g2m::geo::kUtmMaxLat || lon < -180.0 ||
        lon > 180.0 || utm_zone < 1 || utm_zone > 60) {
        d.status = DestinationStatus::OutsideUtmRange;
        return d;
    }
    static const g2m::geo::Utm utm(g2m::geo::kWGS84);
    const g2m::geo::UtmForward f = utm.forward(g2m::geo::UtmZone{utm_zone, g2m::geo::Hemisphere::North}, lat, lon);
    d.session_x = f.easting - e0;
    d.session_y = f.northing - n0;
    d.status = std::hypot(d.session_x, d.session_y) > maximum_distance_m ? DestinationStatus::TooFar
                                                                          : DestinationStatus::Ok;
    return d;
}

} // namespace rg
