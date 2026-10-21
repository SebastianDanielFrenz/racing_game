// rg/number_format.h: the one formatter of every upgrade badge. Decimals follow the magnitude of the ROUNDED value,
// capped by max_decimals; trailing zeros are dropped.
#include "rg/number_format.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

using namespace rg;

TEST_CASE("number format: decimals follow the magnitude, boundary by boundary", "[number_format]") {
    // |x| >= 100 -> 0, >= 10 -> 1, >= 1 -> 2, >= 0.1 -> 3 (cap 6 so the magnitude decides).
    CHECK(display_decimals(1000.0, 6) == 0);
    CHECK(display_decimals(100.0, 6) == 0);
    CHECK(display_decimals(99.9, 6) == 1);
    CHECK(display_decimals(10.0, 6) == 1);
    CHECK(display_decimals(9.99, 6) == 2);
    CHECK(display_decimals(1.0, 6) == 2);
    CHECK(display_decimals(0.999, 6) == 3);
    CHECK(display_decimals(0.1, 6) == 3);
    CHECK(display_decimals(0.099, 6) == 4);
    CHECK(display_decimals(0.01, 6) == 4);
    CHECK(display_decimals(0.0099, 6) == 5);
    CHECK(display_decimals(0.001, 6) == 5);
    CHECK(display_decimals(0.0009, 6) == 6);
    CHECK(display_decimals(-250.0, 6) == 0); // the sign does not matter
    CHECK(display_decimals(-9.5, 6) == 2);
}

TEST_CASE("number format: the cap wins over the magnitude", "[number_format]") {
    CHECK(display_decimals(1.5, 0) == 0);
    CHECK(display_decimals(1.5, 1) == 1);
    CHECK(display_decimals(0.05, 3) == 3);
    CHECK(display_decimals(0.05, 8) == 4);
    CHECK(display_decimals(0.05, 99) == 4); // a silly cap is clamped
    CHECK(display_decimals(0.05, -3) == 0);
}

TEST_CASE("number format: rounding carries into the next magnitude", "[number_format]") {
    CHECK(format_number(99.95, 1) == "100");     // not "100.0"
    CHECK(format_number(99.94, 1) == "99.9");
    CHECK(format_number(9.9951, 2) == "10");     // rounds to 10.00, which is judged as 10 -> one decimal -> "10"
    CHECK(format_number(9.95, 2) == "9.95");
    CHECK(format_number(9.996, 2) == "10");
    CHECK(format_number(0.9996, 3) == "1");
    CHECK(format_number(0.9994, 3) == "0.999");
    CHECK(format_number(-99.95, 1) == "-100");
    CHECK(format_number(999.6, 1) == "1000");
}

TEST_CASE("number format: trailing zeros are dropped, a zero has no sign", "[number_format]") {
    CHECK(format_number(5.0, 1) == "5");
    CHECK(format_number(5.50, 2) == "5.5");
    CHECK(format_number(-6.5, 1) == "-6.5");
    CHECK(format_number(0.0, 3) == "0");
    CHECK(format_number(-0.0, 3) == "0");
    CHECK(format_number(0.0004, 3) == "0");      // rounds to nothing at the cap
    CHECK(format_number(-0.0004, 3) == "0");
    CHECK(format_number(0.05, 3) == "0.05");
    CHECK(format_number(0.0500001, 3) == "0.05");
    CHECK(format_number(123.456, 3) == "123");
    CHECK(format_number(12.345, 3) == "12.3");
    CHECK(format_number(1.2345, 3) == "1.23");
    CHECK(format_number(0.12345, 3) == "0.123");
}

TEST_CASE("number format: the signed form adds + for a positive value only", "[number_format]") {
    CHECK(format_signed_number(5.5, 1) == "+5.5");
    CHECK(format_signed_number(-6.5, 1) == "-6.5");
    CHECK(format_signed_number(0.0, 1) == "0");
    CHECK(format_signed_number(0.00001, 1) == "0");
    CHECK(format_signed_number(5.0, 1) == "+5");
    CHECK(format_signed_number(99.95, 1) == "+100");
}

TEST_CASE("number format: non-finite values print as words", "[number_format]") {
    CHECK(format_number(std::numeric_limits<double>::quiet_NaN(), 2) == "nan");
    CHECK(format_number(std::numeric_limits<double>::infinity(), 2) == "inf");
    CHECK(format_number(-std::numeric_limits<double>::infinity(), 2) == "-inf");
    CHECK(display_decimals(std::numeric_limits<double>::infinity(), 2) == 0);
}
