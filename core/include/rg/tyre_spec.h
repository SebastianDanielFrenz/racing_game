// rg/tyre_spec.h - universal tyres: a compound plus a size, built on the fly.
//
// A tyre is a COMPOUND (family: cord ... drag, data/tyres/families.json) plus a SIZE (width / aspect R rim). It has
// no car and no axle in its identity, and there is no tyre file per size or compound: the only authored data is
// families.json (the per-compound parameters, the base Magic Formula tyre and the scaling rule), and
// TyreFamilies::make builds the ps::vehicle::TyreDesc in memory, the same spec always giving a bit-identical
// result. Vehicle files carry a tyre SPEC per wheel (`"tyre_spec": {"compound", "width_mm", "aspect", "rim_in"}`)
// instead of a path; rg::load_vehicle (rg/vehicle_load.h) injects the built tyre after the physics loader ran.
//
// Cars declare per-axle FITMENT by dimensions only (data/vehicles/catalog.json, `setup.tyre_fitment`): rim
// diameter(s), a width range in standard steps and an overall-diameter tolerance around the axle's own wheel. The
// garage offers every compound at every size the fitment allows.
//
// Id format (the value of a tyre_choice option, also used in saved setups): `<compound>_<width>_<aspect>r<rim>`,
// e.g. `track_325_30r21`.
#pragma once

#include "ps/vehicle/tyre_desc.h"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rg {

struct TyreSpec {
    std::string compound; // family id from families.json
    int width_mm = 0;
    int aspect = 0;       // sidewall height in percent of the width
    int rim_in = 0;
    bool operator==(const TyreSpec& o) const {
        return compound == o.compound && width_mm == o.width_mm && aspect == o.aspect && rim_in == o.rim_in;
    }
};

// `track_325_30r21`.
std::string tyre_id(const TyreSpec& spec);
// Syntax only (compound text, widths, aspect, rim): nullopt for anything else, including legacy names.
std::optional<TyreSpec> parse_tyre_id(const std::string& id);
// "325/30 R21".
std::string tyre_size_text(const TyreSpec& spec);
// Unloaded overall diameter: rim diameter + two sidewalls.
double tyre_diameter_mm(const TyreSpec& spec);
// The whole-percent aspect that brings the overall diameter of a tyre of this rim and width closest to
// `diameter_mm`.
int tyre_aspect_for(int rim_in, int width_mm, double diameter_mm);

// What one axle of a car accepts.
struct TyreAxleFitment {
    std::vector<int> rims_in;            // accepted rim diameters, inches
    int min_width_mm = 0;
    int max_width_mm = 0;
    int width_step_mm = 10;              // standard width steps counted from min_width_mm
    double diameter_tolerance_mm = 12.0; // |tyre overall diameter - the axle's wheel diameter| must not exceed this
};

// "" when `spec` fits an axle whose wheels have `wheel_diameter_mm`, else the reason in plain words (no option
// prefix): rim mismatch, too narrow, too wide, not a standard width step, or overall diameter off by more than the
// tolerance. The compound is not part of fitment.
std::string tyre_fit_error(const TyreSpec& spec, const TyreAxleFitment& fitment, double wheel_diameter_mm);

// Every compound-independent size the fitment allows, ascending by rim then width: one entry per (rim, width) with
// the aspect that keeps the overall diameter closest to the wheel's (entries outside the tolerance are dropped).
// `compound` is left empty.
std::vector<TyreSpec> tyre_sizes_for(const TyreAxleFitment& fitment, double wheel_diameter_mm);

struct TyreCompound {
    std::string id, label, description, image;
    double longitudinal_grip = 1.0;
    double lateral_grip = 1.0;
    double rolling_resistance = 1.0;
    double rated_speed_kmh = 0.0;
    double stiffness_scale = 1.0;
};

class TyreFamilies {
public:
    // Reads families.json (racing_game.tyre_families/4); cached per path and file time. nullptr + *err on failure.
    static std::shared_ptr<const TyreFamilies> load(const std::string& path, std::string* err);

    [[nodiscard]] const std::vector<TyreCompound>& compounds() const { return compounds_; }
    [[nodiscard]] const TyreCompound* find(const std::string& id) const;

    // The tyre, built in memory by the one rule of families.json. Throws std::invalid_argument for an unknown
    // compound or a non-positive size. The same spec always gives a bit-identical TyreDesc.
    [[nodiscard]] ps::vehicle::TyreDesc make(const TyreSpec& spec) const;
    // The compound's rated speed for this size (capped for small rims), km/h.
    [[nodiscard]] double rated_speed_kmh(const TyreSpec& spec) const;
    // Overall diameter the pre-universal generated tyres of a rim were sized for, mm; 0 = none (used by migration).
    [[nodiscard]] int legacy_diameter_mm(int rim_in) const;

private:
    struct Impl;
    std::shared_ptr<const Impl> impl_;
    std::vector<TyreCompound> compounds_;
};

// Maps a pre-universal tyre name to a tyre id: the per-car/per-axle generated stems (`hyper_rear_track_w255_r21`,
// `passenger_sport_w225_r17`), the hand copies (`passenger_sport_225_45r17`, `hyper_front_track_265_35r20`, ...) and
// the physics_sim stock files (`hyper_front_265_35r20`, `passenger_car_225_45r17`), given as a stem or a path.
// "" when `name` is no legacy name (a current id included) or names a compound/rim the families do not know.
std::string migrate_tyre_id(const TyreFamilies& families, const std::string& name);

} // namespace rg
