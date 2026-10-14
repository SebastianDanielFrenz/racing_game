// rg/address_search.h - the address teleport's engine-neutral half (F10 panel, data/controls/geocoding.json), moved out
// of game/scripts/address_teleport.gd (Unreal port prep, 2026-10-14). The front end owns only the text box, the result
// list and the HTTP request; this file decides what a query means, which URL to fetch, how a Photon (GeoJSON) answer
// becomes a list of matches and whether a chosen point is a valid destination in the session frame.
//
// Coordinates: WGS84 latitude / longitude in degrees; the destination is projected with g2m::geo::Utm (WGS84 ellipsoid,
// the world config's northern UTM zone) and the session origin (e0, n0) subtracted, i.e. session-local metres (ISO x =
// east, y = north).
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rg {

struct GeocodingConfig {
    std::string endpoint = "https://photon.komoot.io/api/";
    double maximum_distance_from_origin_m = 30000.0;
    int result_limit = 5;
    std::string language = "de";
    double min_request_interval_s = 1.0; // the public service's fair-use rate (one request per second)
};

// Loads data/controls/geocoding.json; every field is required. std::nullopt + *err on any problem.
std::optional<GeocodingConfig> load_geocoding_config(const std::string& path, std::string* err);

struct AddressMatch {
    std::string label;
    double lat = 0.0;
    double lon = 0.0;
};

// "50.1, 8.6" (two numbers separated by one comma, blanks allowed) -> that point as a single match labelled with the
// query; std::nullopt for anything else (the query is then an address to search).
std::optional<AddressMatch> parse_coordinate_query(std::string_view query);

// The lower-cased, trimmed query: the key of the front end's search cache.
std::string address_cache_key(std::string_view query);

// RFC 3986 percent-encoding of everything but the unreserved characters (A-Z a-z 0-9 - _ . ~), UTF-8 bytes as is.
std::string uri_encode(std::string_view text);

// The search URL: endpoint + q, limit, lang, plus lat/lon of `bias` (the world's spawn) when given.
struct GeoPoint {
    double lat = 0.0;
    double lon = 0.0;
};
std::string geocoding_url(const GeocodingConfig& config, std::string_view query, const std::optional<GeoPoint>& bias);

// A Photon answer ({"features": [{"geometry": {"coordinates": [lon, lat]}, "properties": {...}}]}) -> matches, label =
// "street housenumber, postcode, city, state, country" (empty parts left out; "name" when there is no street).
// Features without numeric coordinates are skipped. std::nullopt when the body is not such an object.
std::optional<std::vector<AddressMatch>> parse_geocoding_response(std::string_view body);

enum class DestinationStatus {
    Ok,
    OutsideUtmRange, // not finite, south of the equator or beyond 84 N, or |lon| > 180
    TooFar,          // farther than maximum_distance_from_origin_m from the session origin
};

struct Destination {
    DestinationStatus status = DestinationStatus::OutsideUtmRange;
    double session_x = 0.0; // valid for Ok and TooFar
    double session_y = 0.0;
};

// Projects (lat, lon) into the session frame of a world whose origin is (zone N, e0, n0) and checks it.
Destination check_destination(double lat, double lon, int utm_zone, double e0, double n0, double maximum_distance_m);

} // namespace rg
