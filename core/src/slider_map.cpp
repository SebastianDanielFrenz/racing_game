// rg/slider_map.cpp - see slider_map.h.
#include "rg/slider_map.h"

#include <algorithm>
#include <cmath>

namespace rg {

const char* to_string(SliderMapping m) { return m == SliderMapping::Log ? "log" : "linear"; }

std::string validate_slider_spec(const SliderSpec& spec) {
    if (!std::isfinite(spec.abs_min) || !std::isfinite(spec.abs_max)) return "abs_min and abs_max must be finite numbers";
    if (!(spec.abs_min < spec.abs_max)) return "abs_min must be below abs_max";
    if (spec.mapping == SliderMapping::Log && !(spec.abs_min > 0.0)) return "a log slider needs abs_min > 0";
    if (spec.mapping == SliderMapping::Linear && !(spec.abs_min >= 0.0)) return "abs_min must not be negative";
    if (!std::isfinite(spec.factor) || !(spec.factor > 0.0)) return "factor must be a positive number";
    if (spec.decimals < 0 || spec.decimals > 6) return "decimals must be between 0 and 6";
    return {};
}

std::pair<double, double> slider_bounds(const SliderSpec& spec, double stock_file) {
    double lo = spec.abs_min, hi = spec.abs_max;
    if (std::isfinite(stock_file) && stock_file > 0.0) {
        lo = std::min(lo, stock_file);
        hi = std::max(hi, stock_file);
    }
    return {lo, hi};
}

double slider_position(SliderMapping mapping, double lo, double hi, double value) {
    if (!(hi > lo)) return 0.0;
    double p;
    if (mapping == SliderMapping::Log && lo > 0.0) {
        if (!(value > 0.0)) return 0.0;
        p = std::log(value / lo) / std::log(hi / lo);
    } else {
        p = (value - lo) / (hi - lo);
    }
    return std::clamp(p, 0.0, 1.0);
}

double slider_value(SliderMapping mapping, double lo, double hi, double pos) {
    const double p = std::clamp(pos, 0.0, 1.0);
    if (mapping == SliderMapping::Log && lo > 0.0 && hi > lo) return lo * std::pow(hi / lo, p);
    return lo + (hi - lo) * p;
}

double round_to_decimals(double v, int decimals) {
    const double scale = std::pow(10.0, static_cast<double>(std::clamp(decimals, 0, 6)));
    return std::round(v * scale) / scale;
}

} // namespace rg
