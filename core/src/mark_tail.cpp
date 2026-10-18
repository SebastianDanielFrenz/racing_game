// rg/mark_tail.h loader.
#include "rg/mark_tail.h"

#include "json_util.h"

#include <cmath>

namespace rg {

double load_post_mark_window_s(const std::string& path, std::string* error) {
    auto fail = [&](const std::string& message) {
        if (error) *error = message + " (using the default " + std::to_string(kDefaultPostMarkWindowS) + " s)";
        return kDefaultPostMarkWindowS;
    };
    if (error) error->clear();
    const auto text = detail::read_text_file(path);
    if (!text) return fail("cannot read " + path);
    const auto doc = detail::parse_json(*text);
    if (!doc || !doc->is_object()) return fail("not a JSON object: " + path);
    const auto it = doc->find("post_window_s");
    if (it == doc->end() || !it->is_number()) return fail("post_window_s missing or not a number");
    const double v = it->get<double>();
    if (!std::isfinite(v) || v < 0.0) return fail("post_window_s must be finite and >= 0");
    return v;
}

} // namespace rg
