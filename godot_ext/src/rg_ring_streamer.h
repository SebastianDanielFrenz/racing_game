// godot_ext/src/rg_ring_streamer.h - RgRingStreamer: the thin Godot binding of g2m::world::RingStreamer (terrain
// migration phase 3b), the level-2 tile policy of the road ribbon and building streams (road_stream.gd,
// building_stream.gd): which tiles to build next, which to show, downgrade and evict. GDScript keeps only its worker
// thread (next_job + the tile build) and the render nodes. All decisions are geo2map's.
#pragma once

#include "g2m/world/ring_streamer.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>

#include <memory>

namespace rg_godot {

class RgRingStreamer : public godot::RefCounted {
    GDCLASS(RgRingStreamer, godot::RefCounted)

public:
    // {full_m, far_m (default = full_m), show_margin_m, evict_margin_m, behind_m, behind_penalty_m, min_speed_mps,
    // empty_keeps_resident}; missing keys keep RingStreamerConfig's defaults. Resets the streamer.
    void configure(const godot::Dictionary& config);
    // Focus in absolute UTM metres and its velocity; true = the job list was rebuilt (wake the worker).
    bool update(double easting, double northing, double vx, double vy);
    // {x, y, full} of the nearest queued job (removed from the queue), or {} when none. Any thread.
    godot::Dictionary next_job();
    int pending_jobs() const;
    // A built tile: 0 rejected (ignore it), 1 empty (free a resident node of the key unless empty_keeps_resident),
    // 2 added (free a resident node of the key, then show the new one).
    int accept(int x, int y, bool full, bool has_content);
    // [{x, y, near, far, downgraded, evicted}, ...] for the focus (absolute UTM).
    godot::Array frame(double easting, double northing);
    // Known tiles (built at some tier, possibly empty) as a flat (x, y, x, y, ...) array.
    godot::PackedInt32Array known_tiles() const;
    int resident_count() const;
    double oldest_wanted_age_s() const;

protected:
    static void _bind_methods();

private:
    std::unique_ptr<g2m::world::RingStreamer> ring_ = std::make_unique<g2m::world::RingStreamer>(g2m::world::RingStreamerConfig{});
};

} // namespace rg_godot
