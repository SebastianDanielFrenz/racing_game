// json_util.h — internal helpers for the rg_core data loaders (credits,
// settings, spawn presets). Exception-free by design, same reason as
// world_config.cpp's header comment (vault TOOL-019: catching a nlohmann
// exception and touching it crashes under the clang-cl asan preset): files
// are parsed with allow_exceptions=false and every field is type-checked
// before it is read.
#pragma once

#include <nlohmann/json.hpp>

#include <fstream>
#include <optional>
#include <sstream>
#include <string>

namespace rg::detail {

using json = nlohmann::json;

inline std::optional<std::string> read_text_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return std::nullopt;
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Parses text; std::nullopt on a syntax error.
inline std::optional<json> parse_json(const std::string& text) {
    json j = json::parse(text, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return std::nullopt;
    return j;
}

inline bool get_string(const json& o, const char* key, std::string& out) {
    const auto it = o.find(key);
    if (it == o.end() || !it->is_string()) return false;
    out = it->get<std::string>();
    return true;
}

inline bool get_bool(const json& o, const char* key, bool& out) {
    const auto it = o.find(key);
    if (it == o.end() || !it->is_boolean()) return false;
    out = it->get<bool>();
    return true;
}

inline bool get_number(const json& o, const char* key, double& out) {
    const auto it = o.find(key);
    if (it == o.end() || !it->is_number()) return false;
    out = it->get<double>();
    return true;
}

} // namespace rg::detail
