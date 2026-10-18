#pragma once
// Post-trigger window bookkeeping for owner drive marks (engine-neutral, no Godot types).
//
// A drive mark stores the ~5 s of samples BEFORE the trigger; the owner also asks what happened AFTER it
// (2026-10-18). MarkTail tracks only the timing: begin() opens a window of window_s simulation seconds, the
// caller feeds every drive sample's time to advance(), which returns the marks whose window is now full, and
// flush() returns every still-open mark (session end, car reset) with the shorter window it got. The caller owns
// the sample payloads, keyed by the returned ids.
#include <cstdint>
#include <vector>

namespace rg {

// Default length of the recorded window after a mark; the adapter reads it from here (one config number).
inline constexpr double kDefaultPostMarkWindowS = 10.0;

class MarkTail {
public:
    struct Done {
        std::int64_t id = 0;
        double covered_s = 0.0; // how much of the window was actually recorded (== window_s when full)
        bool complete = false;  // true: the window filled; false: flushed early
    };

    // Open a window starting at sim time t_s. Returns the mark id.
    std::int64_t begin(double t_s, double window_s) {
        open_.push_back({next_id_, t_s, window_s < 0.0 ? 0.0 : window_s, t_s});
        return next_id_++;
    }

    // A sample at sim time t_s was recorded: marks it belongs to are the open ones (the caller appends it to each
    // open id, see open_ids()). Returns the marks whose window is now full. A sample at or past the window end is
    // still part of that mark (inclusive), so call open_ids() BEFORE advance() to append, then advance().
    std::vector<Done> advance(double t_s) {
        std::vector<Done> done;
        std::vector<Open> keep;
        for (Open& o : open_) {
            if (t_s > o.last_s) o.last_s = t_s;
            if (t_s - o.start_s >= o.window_s) done.push_back({o.id, o.window_s, true});
            else keep.push_back(o);
        }
        open_.swap(keep);
        return done;
    }

    // Close every open window early (session end, car reset).
    std::vector<Done> flush() {
        std::vector<Done> done;
        for (const Open& o : open_) done.push_back({o.id, o.last_s - o.start_s, false});
        open_.clear();
        return done;
    }

    std::vector<std::int64_t> open_ids() const {
        std::vector<std::int64_t> ids;
        for (const Open& o : open_) ids.push_back(o.id);
        return ids;
    }

    bool empty() const { return open_.empty(); }

private:
    struct Open {
        std::int64_t id;
        double start_s;
        double window_s;
        double last_s;
    };
    std::vector<Open> open_;
    std::int64_t next_id_ = 1;
};

} // namespace rg
