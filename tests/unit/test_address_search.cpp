// test_address_search.cpp - rg/address_search.h (address teleport query/URL/response/destination logic, moved from
// game/scripts/address_teleport.gd).
#include "rg/address_search.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>

using Catch::Approx;

TEST_CASE("address search: shipped config loads", "[address_search]") {
    std::string err;
    const auto c = rg::load_geocoding_config(std::string(RG_SOURCE_DIR) + "/data/controls/geocoding.json", &err);
    REQUIRE(c.has_value());
    CHECK(c->endpoint == "https://photon.komoot.io/api/");
    CHECK(c->maximum_distance_from_origin_m == Approx(30000.0));
    CHECK(c->result_limit == 5);
    CHECK(c->min_request_interval_s == Approx(1.0));
}

TEST_CASE("address search: coordinate queries", "[address_search]") {
    const auto p = rg::parse_coordinate_query("  50.1109, 8.6821 ");
    REQUIRE(p.has_value());
    CHECK(p->lat == Approx(50.1109));
    CHECK(p->lon == Approx(8.6821));
    CHECK(p->label == "50.1109, 8.6821");
    CHECK_FALSE(rg::parse_coordinate_query("Hauptstr. 1, Frankfurt").has_value());
    CHECK_FALSE(rg::parse_coordinate_query("50.1").has_value());
    CHECK_FALSE(rg::parse_coordinate_query("1,2,3").has_value());
    CHECK(rg::address_cache_key("  Römer FRANKFURT ") == "römer frankfurt");
}

TEST_CASE("address search: URL", "[address_search]") {
    rg::GeocodingConfig c;
    CHECK(rg::uri_encode("a b/ä~") == "a%20b%2F%C3%A4~");
    CHECK(rg::geocoding_url(c, " Zeil 1 ", std::nullopt) == "https://photon.komoot.io/api/?q=Zeil%201&limit=5&lang=de");
    c.endpoint = "https://example.test/api?x=1";
    CHECK(rg::geocoding_url(c, "a", rg::GeoPoint{50.5, 8.25}) ==
          "https://example.test/api?x=1&q=a&limit=5&lang=de&lat=50.5000000&lon=8.2500000");
}

TEST_CASE("address search: Photon response", "[address_search]") {
    const std::string body = R"({"features": [
      {"geometry": {"coordinates": [8.6821, 50.1109]},
       "properties": {"street": "Zeil", "housenumber": "1", "postcode": "60313", "city": "Frankfurt", "country": "Deutschland"}},
      {"geometry": {"coordinates": [8.5, 50.0]}, "properties": {"name": "Flughafen", "postcode": 60549}},
      {"geometry": {"coordinates": ["x", 50.0]}, "properties": {}},
      {"geometry": {"coordinates": [8.0, 50.0]}}
    ]})";
    const auto m = rg::parse_geocoding_response(body);
    REQUIRE(m.has_value());
    REQUIRE(m->size() == 2);
    CHECK((*m)[0].label == "Zeil 1, 60313, Frankfurt, Deutschland");
    CHECK((*m)[0].lat == Approx(50.1109));
    CHECK((*m)[0].lon == Approx(8.6821));
    CHECK((*m)[1].label == "Flughafen, 60549");
    CHECK_FALSE(rg::parse_geocoding_response("not json").has_value());
    CHECK_FALSE(rg::parse_geocoding_response(R"({"type": "x"})").has_value());
    CHECK(rg::parse_geocoding_response(R"({"features": []})")->empty());
}

TEST_CASE("address search: destination in the session frame", "[address_search]") {
    // Reference: the former GDScript series (WGS84, zone 32) gives E 477269.508, N 5551009.575 for this point.
    const auto d = rg::check_destination(50.1109, 8.6821, 32, 464000.0, 5559000.0, 30000.0);
    CHECK(d.status == rg::DestinationStatus::Ok);
    CHECK(d.session_x == Approx(477269.508 - 464000.0).margin(0.01));
    CHECK(d.session_y == Approx(5551009.575 - 5559000.0).margin(0.01));
    CHECK(rg::check_destination(48.137, 11.575, 32, 464000.0, 5559000.0, 30000.0).status ==
          rg::DestinationStatus::TooFar);
    CHECK(rg::check_destination(-1.0, 8.0, 32, 0, 0, 1e9).status == rg::DestinationStatus::OutsideUtmRange);
    CHECK(rg::check_destination(84.0, 8.0, 32, 0, 0, 1e9).status == rg::DestinationStatus::OutsideUtmRange);
    CHECK(rg::check_destination(50.0, 181.0, 32, 0, 0, 1e9).status == rg::DestinationStatus::OutsideUtmRange);
}
