#include "rg/number_format.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace rg {

namespace {

// 100 -> 0 decimals ... 1e-6 -> 8 decimals. Literals, not pow(): every boundary is the correctly rounded double.
constexpr double kThreshold[] = {100.0, 10.0, 1.0, 0.1, 0.01, 0.001, 0.0001, 1.0e-5, 1.0e-6};
constexpr int kMaxTable = 8;

int decimals_by_magnitude(double a, int cap) {
    int d = 0;
    while (d < cap && d < kMaxTable && a < kThreshold[d]) ++d;
    return d;
}

std::string fixed(double a, int d) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", d, a);
    return buf;
}

} // namespace

int display_decimals(double value, int max_decimals) {
    const int cap = std::clamp(max_decimals, 0, kMaxTable);
    const double a = std::fabs(value);
    if (!std::isfinite(a) || a == 0.0) return 0;
    int d = decimals_by_magnitude(a, cap);
    // Rounding can carry into the next magnitude (99.95 -> "100.0", 9.996 -> "10.00"): judge the rounded value.
    const double rounded = std::strtod(fixed(a, d).c_str(), nullptr);
    d = std::min(d, decimals_by_magnitude(rounded, cap));
    return d;
}

std::string format_number(double value, int max_decimals) {
    if (std::isnan(value)) return "nan";
    if (std::isinf(value)) return value < 0.0 ? "-inf" : "inf";
    const int d = display_decimals(value, max_decimals);
    std::string s = fixed(std::fabs(value), d);
    if (s.find('.') != std::string::npos) {
        while (!s.empty() && s.back() == '0') s.pop_back();
        if (!s.empty() && s.back() == '.') s.pop_back();
    }
    if (s == "0" || s.empty()) return "0";
    return value < 0.0 ? "-" + s : s;
}

std::string format_signed_number(double value, int max_decimals) {
    std::string s = format_number(value, max_decimals);
    if (!s.empty() && s[0] != '-' && s != "0" && s != "nan") s.insert(s.begin(), '+');
    return s;
}

} // namespace rg
