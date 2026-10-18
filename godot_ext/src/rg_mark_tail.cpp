#include "rg_mark_tail.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>

namespace rg_godot {

namespace {
godot::Array to_array(const std::vector<rg::MarkTail::Done>& done) {
    godot::Array out;
    for (const auto& d : done) {
        godot::Dictionary e;
        e["id"] = static_cast<int64_t>(d.id);
        e["covered_s"] = d.covered_s;
        e["complete"] = d.complete;
        out.push_back(e);
    }
    return out;
}
} // namespace

godot::Array RgMarkTail::open_ids() const {
    godot::Array out;
    for (std::int64_t id : tail_.open_ids()) out.push_back(static_cast<int64_t>(id));
    return out;
}

godot::Array RgMarkTail::advance(double t_s) { return to_array(tail_.advance(t_s)); }
godot::Array RgMarkTail::flush() { return to_array(tail_.flush()); }

void RgMarkTail::_bind_methods() {
    using godot::D_METHOD;
    const char* cls = "RgMarkTail";
    godot::ClassDB::bind_static_method(cls, D_METHOD("default_window_s"), &RgMarkTail::default_window_s);
    godot::ClassDB::bind_method(D_METHOD("begin", "t_s", "window_s"), &RgMarkTail::begin);
    godot::ClassDB::bind_method(D_METHOD("open_ids"), &RgMarkTail::open_ids);
    godot::ClassDB::bind_method(D_METHOD("advance", "t_s"), &RgMarkTail::advance);
    godot::ClassDB::bind_method(D_METHOD("flush"), &RgMarkTail::flush);
}

} // namespace rg_godot
