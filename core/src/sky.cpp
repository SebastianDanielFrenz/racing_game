#include "rg/sky.h"

#include <cmath>

namespace rg {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
double wrap360(double a) {
    a = std::fmod(a, 360.0);
    return a < 0 ? a + 360.0 : a;
}
double sind(double a) { return std::sin(a * kDeg); }
double cosd(double a) { return std::cos(a * kDeg); }

// Horizontal position from right ascension/declination (deg) and the local
// hour angle (deg).
SkyPosition horizontal(double hour_angle_deg, double decl_deg, double lat_deg) {
    const double h = hour_angle_deg * kDeg, d = decl_deg * kDeg, f = lat_deg * kDeg;
    const double sin_el = std::sin(f) * std::sin(d) + std::cos(f) * std::cos(d) * std::cos(h);
    SkyPosition p;
    const double el = std::asin(sin_el < -1 ? -1 : sin_el > 1 ? 1 : sin_el);
    const double az = std::atan2(-std::sin(h) * std::cos(d), std::sin(d) * std::cos(f) - std::cos(d) * std::sin(f) * std::cos(h));
    p.elevation_deg = el / kDeg;
    p.azimuth_deg = wrap360(az / kDeg);
    p.east = std::cos(el) * std::sin(az);
    p.north = std::cos(el) * std::cos(az);
    p.up = std::sin(el);
    return p;
}

int last_sunday(int year, int month) {
    const int last = days_in_month(year, month);
    // Weekday from the Julian day: (JD + 1.5) mod 7, 0 = Sunday.
    const int weekday = static_cast<int>(std::fmod(std::floor(julian_day({year, month, last}, 0) + 1.5), 7.0));
    return last - weekday;
}

// Apparent solar ecliptic longitude (deg) and the NOAA intermediate values.
struct SunTerms {
    double app_long, decl, eq_time_min;
};
SunTerms sun_terms(double jd) {
    const double jc = (jd - 2451545.0) / 36525.0;
    const double l0 = wrap360(280.46646 + jc * (36000.76983 + jc * 0.0003032));
    const double m = 357.52911 + jc * (35999.05029 - 0.0001537 * jc);
    const double e = 0.016708634 - jc * (0.000042037 + 0.0000001267 * jc);
    const double c = sind(m) * (1.914602 - jc * (0.004817 + 0.000014 * jc)) + sind(2 * m) * (0.019993 - 0.000101 * jc) + sind(3 * m) * 0.000289;
    const double omega = 125.04 - 1934.136 * jc;
    const double app_long = l0 + c - 0.00569 - 0.00478 * sind(omega);
    const double mean_obliq = 23.0 + (26.0 + (21.448 - jc * (46.815 + jc * (0.00059 - jc * 0.001813))) / 60.0) / 60.0;
    const double obliq = mean_obliq + 0.00256 * cosd(omega);
    const double decl = std::asin(sind(obliq) * sind(app_long)) / kDeg;
    const double y = std::pow(std::tan(obliq * kDeg / 2), 2);
    const double eq = 4.0 / kDeg * (y * sind(2 * l0) - 2 * e * sind(m) + 4 * e * y * sind(m) * cosd(2 * l0) - 0.5 * y * y * sind(4 * l0) - 1.25 * e * e * sind(2 * m));
    return {app_long, decl, eq};
}
} // namespace

int days_in_month(int year, int month) {
    static const int days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month == 2 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)) return 29;
    return days[(month - 1 + 12) % 12];
}

double julian_day(const CivilDate& date, double ut_hours) {
    int y = date.year, m = date.month;
    if (m <= 2) { y -= 1; m += 12; }
    const int a = y / 100;
    const int b = 2 - a + a / 4;
    return std::floor(365.25 * (y + 4716)) + std::floor(30.6001 * (m + 1)) + date.day + b - 1524.5 + ut_hours / 24.0;
}

CivilDate add_days(CivilDate d, int days) {
    while (days > 0) {
        if (++d.day > days_in_month(d.year, d.month)) { d.day = 1; if (++d.month > 12) { d.month = 1; ++d.year; } }
        --days;
    }
    while (days < 0) {
        if (--d.day < 1) { if (--d.month < 1) { d.month = 12; --d.year; } d.day = days_in_month(d.year, d.month); }
        ++days;
    }
    return d;
}

int day_of_year(const CivilDate& d) {
    return static_cast<int>(julian_day(d, 0) - julian_day({d.year, 1, 1}, 0)) + 1;
}

SkyPosition sun_position(double jd, double lat, double lon) {
    const SunTerms s = sun_terms(jd);
    const double ut_min = std::fmod(jd + 0.5 - std::floor(jd + 0.5), 1.0) * 1440.0;
    const double true_solar_min = ut_min + s.eq_time_min + 4.0 * lon;
    const double hour_angle = true_solar_min / 4.0 - 180.0;
    return horizontal(hour_angle, s.decl, lat);
}

MoonPosition moon_position(double jd, double lat, double lon) {
    const double d = jd - 2451543.5;
    const double n = 125.1228 - 0.0529538083 * d, inc = 5.1454, w = 318.0634 + 0.1643573223 * d;
    const double a = 60.2666, e = 0.0549, m = wrap360(115.3654 + 13.0649929509 * d);
    double ecc = m + e / kDeg * sind(m) * (1 + e * cosd(m));
    for (int i = 0; i < 5; ++i) ecc -= (ecc - e / kDeg * sind(ecc) - m) / (1 - e * cosd(ecc));
    const double xv = a * (cosd(ecc) - e), yv = a * std::sqrt(1 - e * e) * sind(ecc);
    const double v = std::atan2(yv, xv) / kDeg;
    double r = std::hypot(xv, yv);
    const double xh = r * (cosd(n) * cosd(v + w) - sind(n) * sind(v + w) * cosd(inc));
    const double yh = r * (sind(n) * cosd(v + w) + cosd(n) * sind(v + w) * cosd(inc));
    const double zh = r * sind(v + w) * sind(inc);
    double lon_ecl = std::atan2(yh, xh) / kDeg, lat_ecl = std::atan2(zh, std::hypot(xh, yh)) / kDeg;
    // Perturbations (Schlyter): sun mean anomaly/longitude, elongation, latitude argument.
    const double ms = wrap360(356.0470 + 0.9856002585 * d), ws = 282.9404 + 4.70935e-5 * d;
    const double ls = ms + ws, lm = n + w + m, dd = lm - ls, f = lm - n;
    lon_ecl += -1.274 * sind(m - 2 * dd) + 0.658 * sind(2 * dd) - 0.186 * sind(ms) - 0.059 * sind(2 * m - 2 * dd) - 0.057 * sind(m - 2 * dd + ms) +
               0.053 * sind(m + 2 * dd) + 0.046 * sind(2 * dd - ms) + 0.041 * sind(m - ms) - 0.035 * sind(dd) - 0.031 * sind(m + ms) -
               0.015 * sind(2 * f - 2 * dd) + 0.011 * sind(m - 4 * dd);
    lat_ecl += -0.173 * sind(f - 2 * dd) - 0.055 * sind(m - f - 2 * dd) - 0.046 * sind(m + f - 2 * dd) + 0.033 * sind(f + 2 * dd) + 0.017 * sind(2 * m + f);
    r += -0.58 * cosd(m - 2 * dd) - 0.46 * cosd(2 * dd);
    const double ecl = 23.4393 - 3.563e-7 * d;
    const double xe = cosd(lon_ecl) * cosd(lat_ecl);
    const double ye = sind(lon_ecl) * cosd(lat_ecl) * cosd(ecl) - sind(lat_ecl) * sind(ecl);
    const double ze = sind(lon_ecl) * cosd(lat_ecl) * sind(ecl) + sind(lat_ecl) * cosd(ecl);
    const double ra = std::atan2(ye, xe) / kDeg, decl = std::atan2(ze, std::hypot(xe, ye)) / kDeg;
    const double ut_h = std::fmod(jd + 0.5 - std::floor(jd + 0.5), 1.0) * 24.0;
    const double sidereal = ls + 180.0 + ut_h * 15.0 + lon;
    MoonPosition out;
    static_cast<SkyPosition&>(out) = horizontal(sidereal - ra, decl, lat);
    // Topocentric parallax (the moon is ~60 Earth radii away: up to ~0.95 deg).
    const double par = std::asin(1.0 / r) / kDeg;
    const double el = out.elevation_deg - par * cosd(out.elevation_deg);
    const double horiz = cosd(el);
    out.elevation_deg = el;
    out.east = horiz * sind(out.azimuth_deg);
    out.north = horiz * cosd(out.azimuth_deg);
    out.up = sind(el);
    const double sun_long = sun_terms(jd).app_long;
    const double elong_cos = cosd(lat_ecl) * cosd(lon_ecl - sun_long);
    out.illuminated_fraction = (1 - elong_cos) / 2;
    out.waxing = wrap360(lon_ecl - sun_long) < 180.0;
    return out;
}

double utc_offset_hours(const TimeZoneRule& rule, const CivilDate& date, double ut_hours) {
    double offset = rule.standard_offset_h;
    if (!rule.eu_summer_time) return offset;
    const double now = julian_day(date, ut_hours);
    const double start = julian_day({date.year, 3, last_sunday(date.year, 3)}, 1.0);
    const double end = julian_day({date.year, 10, last_sunday(date.year, 10)}, 1.0);
    if (now >= start && now < end) offset += 1.0;
    return offset;
}

} // namespace rg
