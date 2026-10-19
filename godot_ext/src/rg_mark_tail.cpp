#include "rg_mark_tail.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
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

double RgMarkTail::load_window_s(const godot::String& path) {
    std::string error;
    const double v = rg::load_post_mark_window_s(path.utf8().get_data(), &error);
    if (!error.empty()) godot::UtilityFunctions::push_warning(("drive_marks.json: " + error).c_str());
    return v;
}

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
    godot::ClassDB::bind_static_method(cls, D_METHOD("load_window_s", "path"), &RgMarkTail::load_window_s);
    godot::ClassDB::bind_method(D_METHOD("begin", "t_s", "window_s"), &RgMarkTail::begin);
    godot::ClassDB::bind_method(D_METHOD("open_ids"), &RgMarkTail::open_ids);
    godot::ClassDB::bind_method(D_METHOD("advance", "t_s"), &RgMarkTail::advance);
    godot::ClassDB::bind_method(D_METHOD("flush"), &RgMarkTail::flush);
}

} // namespace rg_godot
