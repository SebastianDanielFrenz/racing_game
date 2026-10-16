#include "rg_ring_streamer.h"

#include <godot_cpp/core/class_db.hpp>

#include <vector>

namespace rg_godot {

using godot::Dictionary;

void RgRingStreamer::configure(const Dictionary& config) {
    g2m::world::RingStreamerConfig c;
    const auto num = [&config](const char* key, double fallback) {
        return config.has(key) ? static_cast<double>(config[key]) : fallback;
    };
    c.full_m = num("full_m", c.full_m);
    c.far_m = num("far_m", c.full_m);
    c.show_margin_m = num("show_margin_m", c.show_margin_m);
    c.evict_margin_m = num("evict_margin_m", c.evict_margin_m);
    c.behind_m = num("behind_m", c.behind_m);
    c.behind_penalty_m = num("behind_penalty_m", c.behind_penalty_m);
    c.min_speed_mps = num("min_speed_mps", c.min_speed_mps);
    if (config.has("empty_keeps_resident")) c.empty_keeps_resident = static_cast<bool>(config["empty_keeps_resident"]);
    ring_ = std::make_unique<g2m::world::RingStreamer>(c);
}

bool RgRingStreamer::update(double easting, double northing, double vx, double vy) {
    return ring_->update(easting, northing, vx, vy);
}

Dictionary RgRingStreamer::next_job() {
    Dictionary d;
    const auto job = ring_->next_job();
    if (!job) return d;
    d["x"] = job->x;
    d["y"] = job->y;
    d["full"] = job->full;
    return d;
}

int RgRingStreamer::pending_jobs() const { return static_cast<int>(ring_->pending_jobs()); }

int RgRingStreamer::accept(int x, int y, bool full, bool has_content) {
    return static_cast<int>(ring_->accept(x, y, full, has_content));
}

godot::Array RgRingStreamer::frame(double easting, double northing) {
    std::vector<g2m::world::RingChange> changes;
    ring_->frame(easting, northing, changes);
    godot::Array out;
    for (const auto& c : changes) {
        Dictionary d;
        d["x"] = c.x;
        d["y"] = c.y;
        d["near"] = c.near_visible;
        d["far"] = c.far_visible;
        d["downgraded"] = c.downgraded;
        d["evicted"] = c.evicted;
        out.push_back(d);
    }
    return out;
}

godot::PackedInt32Array RgRingStreamer::known_tiles() const {
    godot::PackedInt32Array out;
    for (const auto& k : ring_->known()) {
        out.push_back(k.x);
        out.push_back(k.y);
    }
    return out;
}

int RgRingStreamer::resident_count() const { return static_cast<int>(ring_->resident_count()); }

double RgRingStreamer::oldest_wanted_age_s() const { return ring_->oldest_wanted_age_s(); }

void RgRingStreamer::_bind_methods() {
    using godot::ClassDB;
    using godot::D_METHOD;
    ClassDB::bind_method(D_METHOD("configure", "config"), &RgRingStreamer::configure);
    ClassDB::bind_method(D_METHOD("update", "easting", "northing", "vx", "vy"), &RgRingStreamer::update);
    ClassDB::bind_method(D_METHOD("next_job"), &RgRingStreamer::next_job);
    ClassDB::bind_method(D_METHOD("pending_jobs"), &RgRingStreamer::pending_jobs);
    ClassDB::bind_method(D_METHOD("accept", "x", "y", "full", "has_content"), &RgRingStreamer::accept);
    ClassDB::bind_method(D_METHOD("frame", "easting", "northing"), &RgRingStreamer::frame);
    ClassDB::bind_method(D_METHOD("known_tiles"), &RgRingStreamer::known_tiles);
    ClassDB::bind_method(D_METHOD("resident_count"), &RgRingStreamer::resident_count);
    ClassDB::bind_method(D_METHOD("oldest_wanted_age_s"), &RgRingStreamer::oldest_wanted_age_s);
}

} // namespace rg_godot
