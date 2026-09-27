// rg/terrain_view_streamer.cpp — see rg/terrain_view_streamer.h for the API
// and the adapter's ordering contract.
//
// Why one diff at a time (no new selection while a diff is Ready/Applying),
// coalesced to the latest focus:
//   - A diff is "new selection minus resident set". If a second selection
//     started while the first diff was only partly applied, it would have to
//     be diffed against a set that is neither the old nor the new selection
//     (old ∪ some of the adds), or against the first diff's target, which is
//     not resident yet. Either way the second diff could remove a chunk the
//     first diff has not finished replacing, or add a chunk the first diff is
//     still uploading - the "removals only after all adds" rule would then
//     have to span two diffs. One diff at a time keeps the rule local: every
//     diff is exactly target(n) vs target(n-1).
//   - Rejected: queueing every due selection. Moving fast while a diff is
//     being uploaded would queue stale intermediate diffs whose adds are
//     uploaded only to be removed by the next one.
//   - Rejected: cancel-and-restart the in-flight job on every due move. At
//     speed the job might never finish (starvation), and the partial build is
//     wasted.
//   - Cost of the choice: the follow-up selection starts one diff-apply later
//     (at 0.8 ms/frame and a few dozen adds, well under a second) - small
//     against the 128 m reselect distance (~4 s at 30 m/s). commit() starts
//     that follow-up immediately, not on the next update_focus().
//
// Determinism: the selection is g2m::mesh::select_chunks (pure), the key
// sets are sorted std::vectors diffed with std::set_difference (no unordered
// container anywhere), and build_render_chunks writes each chunk by index, so
// a diff is byte-identical for any build_threads.
//
// Exception-free (vault TOOL-019): nothing here throws or catches.
#include "rg/terrain_view_streamer.h"

#include <algorithm>
#include <chrono>
#include <iterator>
#include <utility>

namespace rg {

namespace {

unsigned auto_initial_threads() {
    const unsigned hw = std::thread::hardware_concurrency();
    return hw == 0 ? 4u : std::min(hw, 8u);
}

} // namespace

TerrainViewStreamer::TerrainViewStreamer(TerrainViewSource source, Options options)
    : source_(source), options_(options) {
    worker_ = std::thread([this]() { worker_main(); });
}

TerrainViewStreamer::~TerrainViewStreamer() {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        stop_ = true;
    }
    cancel_.store(true, std::memory_order_relaxed);
    cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool TerrainViewStreamer::build_initial(double x, double y, std::vector<RenderChunk>& out) {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (phase_ != Phase::Idle) {
            return false;
        }
    }
    // Idle and every public method runs on the one adapter thread, so the
    // worker cannot start a job while this builds - no lock needed around it.
    std::vector<g2m::mesh::ChunkKey> keys;
    select_view_keys(x, y, source_.params, source_.e0, source_.n0, keys);
    const unsigned threads =
        options_.initial_build_threads == 0 ? auto_initial_threads() : options_.initial_build_threads;
    build_render_chunks(keys, source_.lookup, source_.ctx, source_.e0, source_.n0, threads, out, /*cancel=*/nullptr,
                        source_.class_lookup, source_.surfaces);

    std::sort(keys.begin(), keys.end());
    std::lock_guard<std::mutex> lk(mutex_);
    resident_ = std::move(keys);
    has_selection_ = true;
    selection_x_ = x;
    selection_y_ = y;
    stats_.resident_count = resident_.size();
    return true;
}

void TerrainViewStreamer::update_focus(double x, double y) {
    std::lock_guard<std::mutex> lk(mutex_);
    latest_x_ = x;
    latest_y_ = y;
    has_latest_ = true;
    maybe_start_locked();
}

void TerrainViewStreamer::maybe_start_locked() {
    if (phase_ != Phase::Idle || !has_latest_ || stop_) {
        return;
    }
    if (has_selection_) {
        const double dx = latest_x_ - selection_x_;
        const double dy = latest_y_ - selection_y_;
        const double r = options_.reselect_distance_m;
        if (dx * dx + dy * dy <= r * r) {
            return;
        }
    }
    has_selection_ = true;
    selection_x_ = latest_x_;
    selection_y_ = latest_y_;
    job_x_ = latest_x_;
    job_y_ = latest_y_;
    job_serial_ = next_serial_++;
    job_pending_ = true;
    phase_ = Phase::Building;
    ++stats_.selections_started;
    cv_.notify_one();
}

bool TerrainViewStreamer::poll(TerrainViewDiff& out) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (phase_ != Phase::Ready) {
        return false;
    }
    out = std::move(ready_);
    ready_ = TerrainViewDiff{};
    applying_serial_ = out.serial;
    phase_ = Phase::Applying;
    return true;
}

bool TerrainViewStreamer::commit(std::uint64_t serial) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (phase_ != Phase::Applying || serial != applying_serial_) {
        return false;
    }
    resident_ = std::move(target_);
    target_.clear();
    phase_ = Phase::Idle;
    ++stats_.diffs_committed;
    stats_.resident_count = resident_.size();
    maybe_start_locked();
    return true;
}

TerrainViewStreamer::Phase TerrainViewStreamer::phase() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return phase_;
}

TerrainViewStreamer::Stats TerrainViewStreamer::stats() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return stats_;
}

std::vector<g2m::mesh::ChunkKey> TerrainViewStreamer::resident_keys() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return resident_;
}

void TerrainViewStreamer::worker_main() {
    for (;;) {
        std::vector<g2m::mesh::ChunkKey> resident;
        double x = 0.0;
        double y = 0.0;
        std::uint64_t serial = 0;
        {
            std::unique_lock<std::mutex> lk(mutex_);
            cv_.wait(lk, [this]() { return stop_ || job_pending_; });
            if (stop_) {
                return;
            }
            job_pending_ = false;
            // resident_ only changes in build_initial (Idle) and commit
            // (Applying), never while Building - a copy keeps the build
            // below entirely lock-free anyway.
            resident = resident_;
            x = job_x_;
            y = job_y_;
            serial = job_serial_;
        }

        const auto start = std::chrono::steady_clock::now();

        std::vector<g2m::mesh::ChunkKey> selected;
        select_view_keys(x, y, source_.params, source_.e0, source_.n0, selected);
        std::sort(selected.begin(), selected.end());
        selected.erase(std::unique(selected.begin(), selected.end()), selected.end());

        std::vector<g2m::mesh::ChunkKey> to_build;
        TerrainViewDiff diff;
        diff.serial = serial;
        diff.focus_x = x;
        diff.focus_y = y;
        std::set_difference(selected.begin(), selected.end(), resident.begin(), resident.end(),
                            std::back_inserter(to_build));
        std::set_difference(resident.begin(), resident.end(), selected.begin(), selected.end(),
                            std::back_inserter(diff.removed));

        if (!build_render_chunks(to_build, source_.lookup, source_.ctx, source_.e0, source_.n0,
                                 std::max(1u, options_.build_threads), diff.added, &cancel_,
                                 source_.class_lookup, source_.surfaces)) {
            return; // cancelled: only the destructor raises cancel_
        }
        const double ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

        std::lock_guard<std::mutex> lk(mutex_);
        if (stop_) {
            return;
        }
        stats_.chunks_built += to_build.size();
        stats_.chunks_reused += selected.size() - to_build.size();
        stats_.last_added = diff.added.size();
        stats_.last_removed = diff.removed.size();
        stats_.last_build_ms = ms;
        ready_ = std::move(diff);
        target_ = std::move(selected);
        phase_ = Phase::Ready;
    }
}

} // namespace rg
