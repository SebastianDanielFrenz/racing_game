#include "rg/wet_look.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>

namespace rg {

namespace {
double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }
bool fail(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}
} // namespace

std::optional<WetLookConfig> load_wet_look_config(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) { fail(err, path + ": cannot read"); return std::nullopt; }
    const auto doc = detail::parse_json(*text);
    if (!doc || !doc->is_object()) { fail(err, path + ": not a JSON object"); return std::nullopt; }
    std::string format;
    if (!detail::get_string(*doc, "format", format) || format != "rg.wet_look/1") {
        fail(err, path + ": format must be \"rg.wet_look/1\"");
        return std::nullopt;
    }
    WetLookConfig c;
    struct Field { const char* key; double* out; };
    const Field fields[] = {{"wet_full_mm", &c.wet_full_mm},       {"puddle_start_mm", &c.puddle_start_mm},
                            {"puddle_full_mm", &c.puddle_full_mm}, {"rain_full_mm_h", &c.rain_full_mm_h},
                            {"rain_min_mm_h", &c.rain_min_mm_h},   {"snow_below_c", &c.snow_below_c},
                            {"response_s", &c.response_s}};
    for (const auto& f : fields) {
        if (!detail::get_number(*doc, f.key, *f.out) || !std::isfinite(*f.out)) {
            fail(err, path + ": \"" + f.key + "\" (number) is required");
            return std::nullopt;
        }
        if (*f.out < 0.0 && std::string(f.key) != "snow_below_c") {
            fail(err, path + ": \"" + f.key + "\" must be >= 0");
            return std::nullopt;
        }
    }
    if (c.wet_full_mm <= 0.0) { fail(err, path + ": wet_full_mm must be > 0"); return std::nullopt; }
    if (c.puddle_full_mm <= c.puddle_start_mm) {
        fail(err, path + ": puddle_full_mm must be > puddle_start_mm");
        return std::nullopt;
    }
    if (c.rain_full_mm_h <= c.rain_min_mm_h) {
        fail(err, path + ": rain_full_mm_h must be > rain_min_mm_h");
        return std::nullopt;
    }
    return c;
}

WetLook wet_look_target(double water_mm, double precip_mm_h, double air_temp_c, const WetLookConfig& cfg) {
    WetLook w;
    const double water = std::isfinite(water_mm) ? std::max(water_mm, 0.0) : 0.0;
    const double precip = std::isfinite(precip_mm_h) ? std::max(precip_mm_h, 0.0) : 0.0;
    w.wetness = clamp01(water / cfg.wet_full_mm);
    w.puddles = clamp01((water - cfg.puddle_start_mm) / (cfg.puddle_full_mm - cfg.puddle_start_mm));
    const double density =
        precip < cfg.rain_min_mm_h ? 0.0 : clamp01(precip / cfg.rain_full_mm_h);
    const bool snow = std::isfinite(air_temp_c) && air_temp_c < cfg.snow_below_c;
    w.drops = snow ? 0.0 : density;
    w.flakes = snow ? density : 0.0;
    return w;
}

const WetLook& WetLookFilter::update(double water_mm, double precip_mm_h, double air_temp_c, double dt_s) {
    const WetLook t = wet_look_target(water_mm, precip_mm_h, air_temp_c, cfg_);
    if (!primed_ || cfg_.response_s <= 0.0) {
        value_ = t;
        primed_ = true;
        return value_;
    }
    const double dt = std::isfinite(dt_s) ? std::max(dt_s, 0.0) : 0.0;
    const double a = 1.0 - std::exp(-dt / cfg_.response_s);
    value_.wetness += (t.wetness - value_.wetness) * a;
    value_.puddles += (t.puddles - value_.puddles) * a;
    value_.drops += (t.drops - value_.drops) * a;
    value_.flakes += (t.flakes - value_.flakes) * a;
    return value_;
}

} // namespace rg
