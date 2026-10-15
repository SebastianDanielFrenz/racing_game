// godot_ext/src/rg_address_search.h - RgAddressSearch: static bindings of rg/address_search.h (the address teleport's
// query parsing, search URL, Photon response parsing and destination check). game/scripts/address_teleport.gd keeps
// only the panel, the HTTPRequest and the search cache file.
#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/string.hpp>

namespace rg_godot {

class RgAddressSearch : public godot::RefCounted {
    GDCLASS(RgAddressSearch, godot::RefCounted)

public:
    // {ok, message, endpoint, maximum_distance_from_origin_m, result_limit, language, min_request_interval_s}
    static godot::Dictionary load_config(const godot::String& path);
    // {label, lat, lon} for a "lat, lon" query, {} otherwise.
    static godot::Dictionary parse_coordinates(const godot::String& query);
    static godot::String cache_key(const godot::String& query);
    // config: load_config()'s dictionary; bias: {latitude, longitude} (the world's spawn) or {}.
    static godot::String search_url(const godot::Dictionary& config, const godot::String& query, const godot::Dictionary& bias);
    // {ok, matches: [{label, lat, lon}]}; ok false = not a Photon answer.
    static godot::Dictionary parse_response(const godot::PackedByteArray& body);
    // origin: the world config's session_origin_utm {zone, e0, n0}. {status: "ok"|"outside_utm"|"too_far", x, y}.
    static godot::Dictionary check_destination(double lat, double lon, const godot::Dictionary& origin, double maximum_distance_m);
    // UTM zone (north) + easting/northing in metres (false easting included) -> {lat, lon} in degrees; {} when invalid.
    static godot::Dictionary utm_to_latlon(int zone, double easting, double northing);

protected:
    static void _bind_methods();
};

} // namespace rg_godot
