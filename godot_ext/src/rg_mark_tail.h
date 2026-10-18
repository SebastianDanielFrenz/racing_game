// godot_ext/src/rg_mark_tail.h - RgMarkTail: thin binding of rg/mark_tail.h (the post-trigger window of owner drive
// marks). The timing lives in rg_core; main.gd only keeps the sample payloads keyed by the ids returned here.
#pragma once

#include "rg/mark_tail.h"

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>

namespace rg_godot {

class RgMarkTail : public godot::RefCounted {
    GDCLASS(RgMarkTail, godot::RefCounted)

public:
    static double default_window_s() { return rg::kDefaultPostMarkWindowS; }
    // Open a window; returns the mark id.
    int64_t begin(double t_s, double window_s) { return tail_.begin(t_s, window_s); }
    // Ids of the marks that should receive the sample about to be recorded.
    godot::Array open_ids() const;
    // [{id, covered_s, complete}] for the windows this sample time filled.
    godot::Array advance(double t_s);
    // Same shape, for every still-open window (session end, car reset).
    godot::Array flush();

protected:
    static void _bind_methods();

private:
    rg::MarkTail tail_;
};

} // namespace rg_godot
