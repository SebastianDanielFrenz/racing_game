// rg/number_format.h - the one number formatter of every upgrade badge (owner 2026-10-19).
//
// The digits after the point follow the MAGNITUDE of the shown value (about three significant digits), capped by the
// quantity's `max_decimals` from data/garage/upgrade_badges.json:
//     |x| >= 100 -> 0 decimals, >= 10 -> at most 1, >= 1 -> at most 2, >= 0.1 -> at most 3, >= 0.01 -> at most 4 ...
// The magnitude is judged on the ROUNDED value (99.95 prints "100", not "100.0"), and trailing zeros after the point
// are dropped ("+5 kg", not "+5.0 kg"). Engine-neutral, no locale: the decimal separator is always '.'.
#pragma once

#include <string>

namespace rg {

// Decimals the magnitude of `value` allows, at most `max_decimals` (0..8). The value is judged after rounding.
int display_decimals(double value, int max_decimals);

// "-6.5", "100", "0.05"; no sign for a positive value, "0" for a value that rounds to zero. NaN/inf: "nan"/"inf".
std::string format_number(double value, int max_decimals);

// The same with an explicit "+" for a positive value: "+5.5", "-6.5", "0".
std::string format_signed_number(double value, int max_decimals);

} // namespace rg
