// rg/road_ahead.h - follows the road under and ahead of the car through the
// client's resident road segments (g2m::RoadSegment, the same data the speed
// limit match and the road render use). Feeds the cinematic camera
// (camera_math.h): roadside shots are placed along the polyline this returns.
// Engine-neutral, allocation only in the caller's output vector.
//
// Works in grid metres around the car; the points come back offset by
// (anchor_x, anchor_y) - the car's session position in rg::Session, 0 for
// car-relative points - so the function needs no frame object. Reads segments only; a segment span is only
// valid for the duration of the call (rg::Session passes the resident tile
// lists it already reads for the speed-limit match, on the stepping thread).
#pragma once

#include "g2m/layer/osm_roads.h"

#include <cstddef>
#include <span>
#include <vector>

namespace rg {

// A point of a road polyline: x east, y north in metres of the frame the
// caller anchored the trace in (the session frame in rg::Session), plus the
// road's half width there.
struct RoadPoint {
    double x = 0.0;
    double y = 0.0;
    double half_width_m = 0.0;
};

struct RoadAheadParams {
    double max_length_m = 160.0;     // stop after this much road
    double max_snap_m = 12.0;        // the car must be this close to a segment to be "on" it
    double min_heading_cos = 0.5;    // |cos| between the segment and the car heading to use it as the start
    double max_turn_cos = 0.2;       // at a junction only continue onto a segment whose direction dots >= this
    double joint_tolerance_m = 0.1;  // two segment ends closer than this are the same node
    int max_hops = 400;
};

// Appends nothing and returns false when the car is not within max_snap_m of
// a segment roughly along its heading. Otherwise `out` is replaced by the
// polyline: first the car's projection onto its segment, then each node
// onwards in the direction of travel (the heading picks the direction on a
// two-way road; at a junction the straightest continuation wins).
// (car_e, car_n) are grid metres (easting/northing); (hx, hy) is the car's
// horizontal heading (need not be unit length; zero length returns false).
bool trace_road_ahead(const std::vector<std::span<const g2m::RoadSegment>>& tiles, double car_e, double car_n, double hx,
                      double hy, double anchor_x, double anchor_y, const RoadAheadParams& params,
                      std::vector<RoadPoint>& out);

} // namespace rg
