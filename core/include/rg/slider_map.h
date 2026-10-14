// rg/slider_map.h - the data-driven slider mapping of a numeric setup option (garage numeric inputs, 2026-10-13).
//
// An option of kind "scale" stores a FACTOR (value in the file = stock leaf x factor). Its slider block in
// data/vehicles/setup_options.json says how the player sees and moves it in ABSOLUTE units:
//
//   "slider": {"mapping": "log", "abs_min": 2000, "abs_max": 20000, "unit": "rpm", "decimals": 0}
//
// abs_min / abs_max are in the unit of the file leaf the option edits (rpm for the limiter, bar for the boost
// target, seconds for the throttle actuator); "factor" (default 1) turns that unit into the displayed one
// (seconds -> ms = 1000). The stored value stays the factor (absolute / stock), so every saved setup keeps its
// meaning. The stock value is always allowed (the shown range is widened to contain it), so the stock setup of a
// car whose stock lies outside the authored range still validates.
//
// Everything here is pure maths with no Godot or physics types: the garage UI reads the results through RgGarage.
#pragma once

#include <string>
#include <utility>

namespace rg {

enum class SliderMapping { Linear, Log };

const char* to_string(SliderMapping m);

// As authored in the option table (unit of the file leaf).
struct SliderSpec {
    SliderMapping mapping = SliderMapping::Linear;
    double abs_min = 0.0;
    double abs_max = 0.0;
    double factor = 1.0;   // displayed = file value x factor
    int decimals = 2;      // digits after the point when the value is shown
    std::string unit;      // shown after the displayed value
};

// "" when the spec is usable, else the reason: finite bounds with abs_min < abs_max, abs_min > 0 for a log mapping,
// factor > 0, decimals in [0, 6].
std::string validate_slider_spec(const SliderSpec& spec);

// The allowed file-unit range for a leaf whose stock value is `stock_file`: [abs_min, abs_max] widened so the stock
// value itself lies inside it.
std::pair<double, double> slider_bounds(const SliderSpec& spec, double stock_file);

// Position in [0, 1] of `value` between lo and hi (both > 0 for Log); values outside are clamped.
double slider_position(SliderMapping mapping, double lo, double hi, double value);
// The inverse: pos clamped to [0, 1]; the result lies in [lo, hi].
double slider_value(SliderMapping mapping, double lo, double hi, double pos);

// `v` rounded to `decimals` digits.
double round_to_decimals(double v, int decimals);

} // namespace rg
