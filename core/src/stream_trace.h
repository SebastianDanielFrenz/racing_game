#pragma once
// stream_trace.h - opt-in (env RG_STREAM_TRACE=1) timing accumulators for the terrain streaming path: where the
// time of a physics height tile goes (raw DEM, smoothing, road surface, carve apply; road surface split into
// road.geom, profile completion and patch build). Thread-safe, allocation-free on the hot path, zero cost
// beyond one relaxed atomic load when off. Printed by Session as RG_STREAM lines (cumulative; diff two lines
// for a window).
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace rg::stream_trace {

inline bool enabled() {
    static const bool on = [] {
#ifdef _WIN32
        char* buf = nullptr;
        std::size_t len = 0;
        const bool set = _dupenv_s(&buf, &len, "RG_STREAM_TRACE") == 0 && buf != nullptr && buf[0] != '\0';
        std::free(buf);
        return set;
#else
        const char* v = std::getenv("RG_STREAM_TRACE");
        return v != nullptr && v[0] != '\0';
#endif
    }();
    return on;
}

// Experiment knob: a numeric environment variable, `fallback` when unset/unparsable (diagnostics only; the shipped
// values live in world_config.json).
inline double env_double(const char* name, double fallback) {
    std::string text;
#ifdef _WIN32
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name) == 0 && buf != nullptr) text = buf;
    std::free(buf);
#else
    if (const char* v = std::getenv(name)) text = v;
#endif
    if (text.empty()) return fallback;
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    return end != text.c_str() ? value : fallback;
}

enum Phase : int {
    RawHeight = 0, // ctx.height(key): raw DEM tile (store read or derive, incl. waiting on a joined job)
    Smooth,        // terrain smoothing (reads neighbour tiles through ctx.height)
    RoadSurface,   // join_road_surface: the whole nested road-surface product (incl. waiting)
    Apply,         // copy + RoadSurfacePatch::apply
    RoadGeom,      // produce_road_surface: ctx.road_geom (store read or derive)
    RoadComplete,  // produce_road_surface: DEM dependencies + profile completion (+ retries)
    RoadPatch,     // produce_road_surface: RoadSurfacePatch construction
    PhaseCount
};

struct Slot {
    std::atomic<std::uint64_t> n{0}, ns{0}, max_ns{0};
    void add(std::uint64_t d) {
        n.fetch_add(1, std::memory_order_relaxed);
        ns.fetch_add(d, std::memory_order_relaxed);
        std::uint64_t m = max_ns.load(std::memory_order_relaxed);
        while (d > m && !max_ns.compare_exchange_weak(m, d, std::memory_order_relaxed)) {}
    }
};

// [phase][0 = level 0 (physics), 1 = coarse levels (render)]
inline Slot (&slots())[PhaseCount][2] {
    static Slot s[PhaseCount][2];
    return s;
}

inline const char* phase_name(int p) {
    static const char* names[PhaseCount] = {"raw_height", "smooth", "road_surface", "apply", "road_geom", "road_complete", "road_patch"};
    return names[p];
}

class Scope {
public:
    Scope(Phase p, int level) : on_(enabled()), slot_(on_ ? &slots()[p][level > 0 ? 1 : 0] : nullptr) {
        if (on_) t0_ = std::chrono::steady_clock::now();
    }
    ~Scope() {
        if (on_) slot_->add(static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0_).count()));
    }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;

private:
    bool on_;
    Slot* slot_;
    std::chrono::steady_clock::time_point t0_{};
};

// "raw_height:L0:n=12:ms=345.6:max=80.1 ..." (cumulative)
inline std::string summary() {
    std::string out;
    char buf[160];
    for (int p = 0; p < PhaseCount; ++p) {
        for (int lv = 0; lv < 2; ++lv) {
            const Slot& s = slots()[p][lv];
            const auto n = s.n.load(std::memory_order_relaxed);
            if (n == 0) continue;
            std::snprintf(buf, sizeof buf, "%s:%s:n=%llu:ms=%.1f:max=%.1f ", phase_name(p), lv ? "Lc" : "L0",
                          static_cast<unsigned long long>(n), static_cast<double>(s.ns.load(std::memory_order_relaxed)) / 1e6,
                          static_cast<double>(s.max_ns.load(std::memory_order_relaxed)) / 1e6);
            out += buf;
        }
    }
    return out;
}

} // namespace rg::stream_trace
