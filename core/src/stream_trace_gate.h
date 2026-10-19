#pragma once
// stream_trace_gate.h - RG_STREAM_TRACE=1 diagnostics for the physics terrain gate (internal to session.cpp).
// Times every physics height-tile fetch (TracingFetch) and follows each wanted tile from "first in the keep set
// and not resident" to "installed" (GateTrace::on_update), so a gate freeze can be explained: which tiles were
// missing (distance/heading relative to the car), how long they had been wanted, how long the fetch itself took
// and how long they queued. Output on stdout: RG_GATE (one line per freeze episode while driving) and
// RG_STREAM / RG_STREAM_TILES (every 5 s: the stream_trace.h phase table plus the tile windows).
#include "stream_trace.h"

#include "g2m/core/tile_key.h"
#include "g2m/phys/height_tile_loader.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/physics_streamer.h"
#include "g2m/phys/resident_heights.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <span>
#include <unordered_map>
#include <vector>

namespace rg {

class TracingFetch final : public g2m::phys::IHeightTileFetch {
public:
    explicit TracingFetch(std::shared_ptr<g2m::phys::IHeightTileFetch> inner) : inner_(std::move(inner)) {}
    g2m::phys::FetchResult fetch(const g2m::TileKey& key) override {
        const auto t0 = std::chrono::steady_clock::now();
        g2m::phys::FetchResult r = inner_->fetch(key);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::lock_guard<std::mutex> lock(mutex_);
        last_ms_[key.packed()] = ms;
        ++count_;
        total_ms_ += ms;
        max_ms_ = std::max(max_ms_, ms);
        return r;
    }
    bool provides_roads() const override { return inner_->provides_roads(); }
    bool provides_classes() const override { return inner_->provides_classes(); }
    double last_ms(std::uint64_t packed) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = last_ms_.find(packed);
        return it == last_ms_.end() ? -1.0 : it->second;
    }
    void totals(std::uint64_t& n, double& total_ms, double& max_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        n = count_;
        total_ms = total_ms_;
        max_ms = max_ms_;
    }

private:
    std::shared_ptr<g2m::phys::IHeightTileFetch> inner_;
    std::mutex mutex_;
    std::unordered_map<std::uint64_t, double> last_ms_;
    std::uint64_t count_ = 0;
    double total_ms_ = 0.0, max_ms_ = 0.0;
};

class GateTrace {
public:
    explicit GateTrace(std::shared_ptr<TracingFetch> f) : fetch_(std::move(f)), t0_(Clock::now()) {}

    // Sim thread, right after PhysicsTerrainStreamer::update.
    void on_update(const g2m::phys::PhysicsTerrainStreamer& streamer, const g2m::phys::ResidentHeightSet& resident,
                   const g2m::phys::PhysicsTileGrid& grid, std::span<const g2m::phys::InterestPoint> points,
                   const g2m::phys::GateStatus& status) {
        const double now = now_ms();
        gs_ = status;
        const auto packed_less = [](const g2m::TileKey& a, const g2m::TileKey& b) { return a.packed() < b.packed(); };
        const auto& keep = streamer.keep_keys();
        for (const g2m::TileKey& k : keep) {
            if (!resident.contains(k)) wanted_.emplace(k.packed(), Want{k, now});
        }
        for (auto it = wanted_.begin(); it != wanted_.end();) {
            if (resident.contains(it->second.key)) {
                const double latency = now - it->second.first_ms;
                const double f = fetch_->last_ms(it->first);
                lat_.push_back(latency);
                fet_.push_back(f);
                que_.push_back(std::max(0.0, latency - std::max(0.0, f)));
                it = wanted_.erase(it);
            } else if (!std::binary_search(keep.begin(), keep.end(), it->second.key, packed_less)) {
                it = wanted_.erase(it);
            } else {
                ++it;
            }
        }
        // Which gate tiles are missing, relative to the first interest point (the car) and its heading.
        near_dist_ = -1;
        miss_ahead_ = 0;
        near_along_ = near_lat_ = near_age_ = far_along_ = 0;
        if (!status.ready && !points.empty()) {
            const g2m::phys::InterestPoint& p = points.front();
            const double sp = std::sqrt(p.vx * p.vx + p.vy * p.vy);
            const double hx = sp > 1.0 ? p.vx / sp : 1.0, hy = sp > 1.0 ? p.vy / sp : 0.0;
            const double e0 = static_cast<double>(grid.frame().e0_m()), n0 = static_cast<double>(grid.frame().n0_m());
            const double half = static_cast<double>(g2m::TileKey::kLevel0SizeM) / 2.0;
            for (const g2m::TileKey& k : streamer.gate_keys()) {
                if (resident.contains(k)) continue;
                const double rx = static_cast<double>(k.min_easting()) + half - e0 - p.x;
                const double ry = static_cast<double>(k.min_northing()) + half - n0 - p.y;
                const double d = std::sqrt(rx * rx + ry * ry);
                const double along = rx * hx + ry * hy, lat = -rx * hy + ry * hx;
                if (along > 0) ++miss_ahead_;
                far_along_ = std::max(far_along_, along);
                if (near_dist_ < 0 || d < near_dist_) {
                    near_dist_ = d;
                    near_along_ = along;
                    near_lat_ = lat;
                    const auto w = wanted_.find(k.packed());
                    near_age_ = w == wanted_.end() ? 0.0 : now - w->second.first_ms;
                }
            }
        }
        if (now - last_print_ms_ >= 5000.0) {
            last_print_ms_ = now;
            print_summary(now);
        }
    }

    // One call per tick attempt (stepping thread), after the gate verdict.
    void on_attempt(bool tiles_ok, bool decks_ok, bool have_vehicle, double speed) {
        const bool blocked = !(tiles_ok && decks_ok);
        const double now = now_ms();
        if (blocked && have_vehicle) {
            if (!in_freeze_) {
                in_freeze_ = true;
                fz_start_ms_ = now;
                fz_attempts_ = 0;
                fz_missing_first_ = gs_.missing_required;
                fz_missing_peak_ = 0;
                fz_tiles_ = fz_decks_ = false;
                fz_speed_ = speed;
                fz_near_dist_ = near_dist_;
                fz_near_along_ = near_along_;
                fz_near_lat_ = near_lat_;
                fz_near_age_ = near_age_;
                fz_far_along_ = far_along_;
                fz_miss_ahead_ = miss_ahead_;
                fz_inflight_ = gs_.inflight;
            }
            ++fz_attempts_;
            fz_missing_peak_ = std::max(fz_missing_peak_, gs_.missing_required);
            if (!tiles_ok) {
                fz_tiles_ = true;
            } else if (!decks_ok) {
                fz_decks_ = true;
            }
        } else if (in_freeze_) {
            in_freeze_ = false;
            std::printf("RG_GATE episode t_s=%.2f dur_ms=%.0f attempts=%llu speed_mps=%.1f cause=%s missing_first=%u missing_peak=%u "
                        "ahead_missing=%u near_dist_m=%.0f near_along_m=%.0f near_lat_m=%.0f near_wanted_age_ms=%.0f far_along_m=%.0f inflight=%u\n",
                        fz_start_ms_ / 1000.0, now - fz_start_ms_, static_cast<unsigned long long>(fz_attempts_), fz_speed_,
                        fz_tiles_ && fz_decks_ ? "tiles+decks" : fz_tiles_ ? "tiles" : "decks", fz_missing_first_, fz_missing_peak_,
                        fz_miss_ahead_, fz_near_dist_, fz_near_along_, fz_near_lat_, fz_near_age_, fz_far_along_, fz_inflight_);
            std::fflush(stdout);
        }
    }

private:
    using Clock = std::chrono::steady_clock;
    struct Want {
        g2m::TileKey key;
        double first_ms;
    };

    double now_ms() const { return std::chrono::duration<double, std::milli>(Clock::now() - t0_).count(); }

    static double pct(std::vector<double> v, double q) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        const auto idx = static_cast<std::size_t>(std::ceil(q * static_cast<double>(v.size())));
        return v[std::min(v.size() - 1, idx == 0 ? 0 : idx - 1)];
    }

    void print_summary(double now) {
        std::uint64_t n = 0;
        double tot = 0, mx = 0;
        fetch_->totals(n, tot, mx);
        std::printf("RG_STREAM t_s=%.1f fetch_n=%llu fetch_total_ms=%.0f fetch_max_ms=%.0f wanted=%zu %s\n", now / 1000.0,
                    static_cast<unsigned long long>(n), tot, mx, wanted_.size(), stream_trace::summary().c_str());
        std::printf("RG_STREAM_TILES t_s=%.1f win_n=%zu latency_ms[p50=%.0f p95=%.0f max=%.0f] fetch_ms[p50=%.0f p95=%.0f max=%.0f] queue_ms[p50=%.0f p95=%.0f max=%.0f]\n",
                    now / 1000.0, lat_.size(), pct(lat_, 0.5), pct(lat_, 0.95), pct(lat_, 1.0), pct(fet_, 0.5), pct(fet_, 0.95),
                    pct(fet_, 1.0), pct(que_, 0.5), pct(que_, 0.95), pct(que_, 1.0));
        std::fflush(stdout);
        lat_.clear();
        fet_.clear();
        que_.clear();
    }

    std::shared_ptr<TracingFetch> fetch_;
    Clock::time_point t0_;
    std::map<std::uint64_t, Want> wanted_;
    std::vector<double> lat_, fet_, que_;
    double last_print_ms_ = 0.0;

    g2m::phys::GateStatus gs_;
    unsigned miss_ahead_ = 0;
    double near_dist_ = -1, near_along_ = 0, near_lat_ = 0, near_age_ = 0, far_along_ = 0;

    bool in_freeze_ = false;
    double fz_start_ms_ = 0;
    std::uint64_t fz_attempts_ = 0;
    unsigned fz_missing_first_ = 0, fz_missing_peak_ = 0;
    bool fz_tiles_ = false, fz_decks_ = false;
    double fz_speed_ = 0;
    double fz_near_dist_ = -1, fz_near_along_ = 0, fz_near_lat_ = 0, fz_near_age_ = 0, fz_far_along_ = 0;
    unsigned fz_miss_ahead_ = 0, fz_inflight_ = 0;
};

} // namespace rg
