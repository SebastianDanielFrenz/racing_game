// rg/sky.h - sun and moon position for a real date, time and place, plus the
// civil-time rule (EU summer time) the time-of-day controller uses.
// Engine-neutral, no Godot type. Accuracy: sun (NOAA spreadsheet algorithm)
// ~0.01 deg; moon (P. Schlyter's low-precision series incl. topocentric
// parallax) ~0.3 deg. Geometric elevations (no refraction).
#pragma once

namespace rg {

struct CivilDate {
    int year = 2026;
    int month = 1;
    int day = 1;
};

// Julian day (UT) of a Gregorian date plus hours since that date's 00:00 UT.
double julian_day(const CivilDate& date, double ut_hours);
int days_in_month(int year, int month);
// date + whole days (may be negative), Gregorian.
CivilDate add_days(CivilDate date, int days);
// ISO day of year, 1..366.
int day_of_year(const CivilDate& date);

struct SkyPosition {
    double elevation_deg = 0; // above the horizon positive
    double azimuth_deg = 0;   // from true north, clockwise, [0, 360)
    // Unit vector towards the body, local east/north/up.
    double east = 0, north = 0, up = 0;
};

struct MoonPosition : SkyPosition {
    double illuminated_fraction = 0; // 0 new .. 1 full
    bool waxing = true;
};

SkyPosition sun_position(double jd_ut, double lat_deg, double lon_deg);
MoonPosition moon_position(double jd_ut, double lat_deg, double lon_deg);

// Civil time zone: standard offset plus optional EU summer time (last Sunday
// of March 01:00 UT to last Sunday of October 01:00 UT, +1 h).
struct TimeZoneRule {
    double standard_offset_h = 1.0;
    bool eu_summer_time = true;
};
// Offset (hours, local = UT + offset) in force at a given UT instant.
double utc_offset_hours(const TimeZoneRule& rule, const CivilDate& date_ut, double ut_hours);

} // namespace rg
