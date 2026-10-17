#pragma once
// Wet look (visuals): maps the physics weather (water film under the car, rain rate, air temperature) to what the
// renderer shows - road wetness, puddles, falling drops or flakes. Engine-neutral; the numbers come from
// data/world/wet_look.json (rg.wet_look/1). The shading itself (how dark, how glossy) is the renderer's
// (data/textures/surface_textures.json "wet").
#include <optional>
#include <string>

namespace rg {

struct WetLookConfig {
    double wet_full_mm = 0.1;        // water film at which a surface reads fully wet
    double puddle_start_mm = 0.3;    // puddles start forming above this film
    double puddle_full_mm = 1.5;     // full puddle cover
    double rain_full_mm_h = 8.0;     // drop density 1 at this rate
    double rain_min_mm_h = 0.05;     // no drops below this rate
    double snow_below_c = 1.0;       // precipitation falls as snow below this air temperature
    double response_s = 2.0;         // smoothing time constant (wheel contact changes, surface changes)
};

// Strict loader: every field above is required, all >= 0, puddle_full_mm > puddle_start_mm, wet_full_mm > 0,
// rain_full_mm_h > rain_min_mm_h. std::nullopt + *err on failure.
std::optional<WetLookConfig> load_wet_look_config(const std::string& path, std::string* err);

struct WetLook {
    double wetness = 0;      // 0 dry .. 1 fully wet
    double puddles = 0;      // 0 none .. 1 full puddle cover
    double drops = 0;        // falling rain density 0..1
    double flakes = 0;       // falling snow density 0..1
};

// The instantaneous target from one weather sample (no smoothing).
WetLook wet_look_target(double water_mm, double precip_mm_h, double air_temp_c, const WetLookConfig& cfg);

// First-order smoothing toward the target (time constant response_s); the first update jumps to the target.
class WetLookFilter {
public:
    explicit WetLookFilter(WetLookConfig cfg = {}) : cfg_(cfg) {}
    const WetLook& update(double water_mm, double precip_mm_h, double air_temp_c, double dt_s);
    const WetLook& value() const { return value_; }
    void reset() { primed_ = false; value_ = {}; }
    const WetLookConfig& config() const { return cfg_; }

private:
    WetLookConfig cfg_;
    WetLook value_;
    bool primed_ = false;
};

} // namespace rg
