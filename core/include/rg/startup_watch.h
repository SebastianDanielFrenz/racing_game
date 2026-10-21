// rg/startup_watch.h - the start-up timeout rules, engine-neutral and clock-free (the caller passes the time).
//
// A cold derive (first start after a geo2map pin bump: every tile derives) can legitimately take minutes, so a
// wall-clock limit fails a healthy start. The rule is: fail only when NOTHING moved for `no_progress_s` (the
// progress signature - resident / missing / in-flight / failed counts - stayed identical), or when the whole wait
// exceeds the generous absolute cap `max_s`. Both come from physics.startup_timeout_s / physics.startup_max_s.
#pragma once

#include <cstdint>

namespace rg {

struct StartupWatchConfig {
    double no_progress_s = 60.0; // no change of the signature for this long = stuck
    double max_s = 900.0;        // absolute cap on the whole wait (<= 0 = no cap)
};

struct StartupSignature {
    std::uint32_t missing_required = 0;
    std::uint32_t inflight = 0;
    std::uint32_t resident = 0;
    std::uint32_t failed = 0;
    bool operator==(const StartupSignature&) const = default;
};

enum class StartupVerdict { Waiting, NoProgress, CapExceeded };

class StartupWatch {
public:
    explicit StartupWatch(StartupWatchConfig config = {}) : config_(config) {}

    // `now_s`: seconds since the wait began (monotonic). Call once per poll.
    StartupVerdict observe(double now_s, const StartupSignature& sig) {
        if (!have_last_ || !(sig == last_)) {
            last_ = sig;
            have_last_ = true;
            last_change_s_ = now_s;
        }
        if (config_.max_s > 0.0 && now_s > config_.max_s) return StartupVerdict::CapExceeded;
        if (now_s - last_change_s_ > config_.no_progress_s) return StartupVerdict::NoProgress;
        return StartupVerdict::Waiting;
    }
    [[nodiscard]] double last_change_s() const { return last_change_s_; }

private:
    StartupWatchConfig config_;
    StartupSignature last_{};
    bool have_last_ = false;
    double last_change_s_ = 0.0;
};

} // namespace rg
