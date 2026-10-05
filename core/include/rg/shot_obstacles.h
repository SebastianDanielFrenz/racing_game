// rg/shot_obstacles.h - what the cinematic director must not put a camera in
// or film through (camera_math.h). Engine-neutral: an interface over session
// coordinates (x east, y north, z up, metres; z is the absolute height the
// terrain and buildings use). The real implementation is
// rg::BuildingObstacles (building_footprints.h), fed from the native building
// records; unit tests use small synthetic ones.
#pragma once

#include <optional>

namespace rg {

class ShotObstacles {
public:
    virtual ~ShotObstacles() = default;

    // Ground height (session z) at (x, y); nullopt = unknown (the director then
    // treats the ground as z = 0 and skips terrain occlusion there).
    [[nodiscard]] virtual std::optional<double> ground_height(double x, double y) const = 0;

    // True when (x, y, z) is inside, or within margin_m horizontally of, a
    // building whose vertical span covers z.
    [[nodiscard]] virtual bool building_covers(double x, double y, double z, double margin_m) const = 0;

    // True when the straight segment a -> b passes through a building volume.
    [[nodiscard]] virtual bool building_blocks(double ax, double ay, double az, double bx, double by,
                                               double bz) const = 0;

    // False while the data around (x, y) is still loading: the director then
    // neither trusts nor starts a roadside shot there (it frames from behind
    // the car instead and tries again). Static data is always ready.
    [[nodiscard]] virtual bool ready(double /*x*/, double /*y*/) const { return true; }
};

} // namespace rg
