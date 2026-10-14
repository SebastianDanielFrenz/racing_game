#include "rg_address_search.h"

#include "rg/address_search.h"

#include <godot_cpp/core/class_db.hpp>

#include <string>

namespace rg_godot {

using godot::Dictionary;
using godot::String;

namespace {
std::string to_std(const String& s) { return std::string(s.utf8().get_data()); }
String from_std(const std::string& s) { return String::utf8(s.c_str()); }

Dictionary match_dict(const rg::AddressMatch& m) {
    Dictionary d;
    d["label"] = from_std(m.label);
    d["lat"] = m.lat;
    d["lon"] = m.lon;
    return d;
}
} // namespace

Dictionary RgAddressSearch::load_config(const String& path) {
    Dictionary d;
    std::string err;
    const auto c = rg::load_geocoding_config(to_std(path), &err);
    d["ok"] = c.has_value();
    d["message"] = from_std(err);
    if (!c) return d;
    d["endpoint"] = from_std(c->endpoint);
    d["maximum_distance_from_origin_m"] = c->maximum_distance_from_origin_m;
    d["result_limit"] = c->result_limit;
    d["language"] = from_std(c->language);
    d["min_request_interval_s"] = c->min_request_interval_s;
    return d;
}

Dictionary RgAddressSearch::parse_coordinates(const String& query) {
    const auto m = rg::parse_coordinate_query(to_std(query));
    return m ? match_dict(*m) : Dictionary();
}

String RgAddressSearch::cache_key(const String& query) { return from_std(rg::address_cache_key(to_std(query))); }

String RgAddressSearch::search_url(const Dictionary& config, const String& query, const Dictionary& bias) {
    rg::GeocodingConfig c;
    c.endpoint = to_std(String(config.get("endpoint", from_std(c.endpoint))));
    c.result_limit = static_cast<int>(config.get("result_limit", c.result_limit));
    c.language = to_std(String(config.get("language", from_std(c.language))));
    std::optional<rg::GeoPoint> b;
    if (bias.has("latitude") && bias.has("longitude"))
        b = rg::GeoPoint{static_cast<double>(bias["latitude"]), static_cast<double>(bias["longitude"])};
    return from_std(rg::geocoding_url(c, to_std(query), b));
}

Dictionary RgAddressSearch::parse_response(const godot::PackedByteArray& body) {
    Dictionary d;
    const std::string text(reinterpret_cast<const char*>(body.ptr()), static_cast<std::size_t>(body.size()));
    const auto matches = rg::parse_geocoding_response(text);
    d["ok"] = matches.has_value();
    godot::Array list;
    if (matches)
        for (const auto& m : *matches) list.push_back(match_dict(m));
    d["matches"] = list;
    return d;
}

Dictionary RgAddressSearch::check_destination(double lat, double lon, const Dictionary& origin, double maximum_distance_m) {
    const rg::Destination r = rg::check_destination(lat, lon, static_cast<int>(origin.get("zone", 32)),
                                                    static_cast<double>(origin.get("e0", 0.0)),
                                                    static_cast<double>(origin.get("n0", 0.0)), maximum_distance_m);
    Dictionary d;
    d["status"] = String(r.status == rg::DestinationStatus::Ok        ? "ok"
                         : r.status == rg::DestinationStatus::TooFar ? "too_far"
                                                                      : "outside_utm");
    d["x"] = r.session_x;
    d["y"] = r.session_y;
    return d;
}

void RgAddressSearch::_bind_methods() {
    using godot::D_METHOD;
    const char* cls = "RgAddressSearch";
    godot::ClassDB::bind_static_method(cls, D_METHOD("load_config", "path"), &RgAddressSearch::load_config);
    godot::ClassDB::bind_static_method(cls, D_METHOD("parse_coordinates", "query"), &RgAddressSearch::parse_coordinates);
    godot::ClassDB::bind_static_method(cls, D_METHOD("cache_key", "query"), &RgAddressSearch::cache_key);
    godot::ClassDB::bind_static_method(cls, D_METHOD("search_url", "config", "query", "bias"), &RgAddressSearch::search_url);
    godot::ClassDB::bind_static_method(cls, D_METHOD("parse_response", "body"), &RgAddressSearch::parse_response);
    godot::ClassDB::bind_static_method(cls, D_METHOD("check_destination", "lat", "lon", "origin", "maximum_distance_m"),
                                       &RgAddressSearch::check_destination);
}

} // namespace rg_godot
