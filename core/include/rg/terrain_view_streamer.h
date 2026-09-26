// rg/terrain_view_streamer.h — rg::TerrainViewStreamer: render LOD that
// follows a moving focus point (R2.2 plan section 3, commit R8).
//
// Engine-neutral (no Godot type, repo CLAUDE.md / user memory
// "engine-neutral logic"): a Godot adapter (godot_ext's RgTerrainView) and a
// future UE5 adapter drive it through exactly the same calls. It never
// touches a renderer - it only decides WHICH chunks should be resident, builds
// the new ones on a background thread, and hands the adapter a diff.
//
// What it does: once the focus point (the car, in session-local metres) has
// moved more than Options::reselect_distance_m since the focus of the last
// selection, one background worker re-runs g2m::mesh::select_chunks for the
// new focus, diffs the resulting key set against the resident key set, builds
// ONLY the added chunks (a chunk whose key is already resident is reused as
// it is - the adapter keeps it uploaded, nothing is rebuilt) and publishes a
// TerrainViewDiff{added (built chunks), removed (keys)}, both sorted by
// ChunkKey ascending.
//
// ---------------------------------------------------------------------------
// Adapter ordering contract (the whole point of this class - no holes):
//
//   every frame:
//     1. streamer.update_focus(x, y)             never blocks; may start a job
//     2. if no diff is in progress:
//          streamer.poll(diff)                   never blocks; true = a diff is
//                                                yours now (phase Applying)
//     3. upload diff.added within this frame's time/count budget (any order,
//        any number per frame - chunks are independent)
//     4. ONLY in the frame the LAST add of this diff has been uploaded (or
//        later): free every chunk in diff.removed, then call
//        streamer.commit(diff.serial)
//
//   Removals after adds means the view only ever holds old ∪ (part of new)
//   and then new, never less than one complete selection: an old coarse
//   chunk stays until all its finer replacements are up, and an old set of
//   fine chunks stays until their coarser replacement is up. Old and new
//   chunks overlap for the frames in between (acceptable - both are the same
//   terrain surface at different LOD); a hole never appears.
//
//   While a diff is Ready or Applying, the streamer starts NO new selection:
//   update_focus() only records the latest focus. commit() then immediately
//   starts one selection for the LATEST focus if it is due (so any number of
//   moves while busy coalesce into exactly one follow-up diff, to where the
//   focus is now - never a chain of stale intermediate diffs). Rationale in
//   the .cpp's top comment.
//
//   A diff that turns out empty (the focus moved far enough to reselect, but
//   the selection did not change) is still delivered; the adapter just
//   commits it.
// ---------------------------------------------------------------------------
//
// Threading: every public method is called from ONE adapter thread (the
// render/main thread); the class owns one worker std::thread internally,
// which runs the selection + build (the build itself fans out to
// Options::build_threads short-lived threads via build_render_chunks). The
// source's lookup must be thread-safe (see TerrainViewSource).
//
// Shutdown: the destructor raises a cancel flag the build checks before every
// chunk and joins the worker - it waits at most for the chunks already being
// built (one blocking tile fetch each, worst case), never a whole selection.
#pragma once

#include "rg/terrain_render.h"
#include "rg/world_terrain.h"

#include "g2m/mesh/terrain_chunk.h"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace rg {

struct TerrainViewDiff {
    std::uint64_t serial = 0;           // pass to commit(); 1, 2, 3, ... per streamer
    double focus_x = 0.0;               // the session-local focus this diff selects for
    double focus_y = 0.0;
    std::vector<RenderChunk> added;     // new-set minus resident-set, built, sorted by key ascending
    std::vector<g2m::mesh::ChunkKey> removed; // resident-set minus new-set, sorted ascending
};

class TerrainViewStreamer {
public:
    struct Options {
        // A new selection is due once the focus is MORE than this far (2D,
        // metres) from the focus of the last selection.
        double reselect_distance_m = 128.0;
        // Threads the background build fans out to (>= 1). Kept small by
        // default: the build runs next to the sim thread and the renderer,
        // and a 128 m step only adds a few dozen chunks.
        unsigned build_threads = 2;
        // Threads build_initial() uses on the caller's thread (0 = auto,
        // min(hardware_concurrency, 8) - WorldTerrain::build_static_view's
        // own choice).
        unsigned initial_build_threads = 0;
    };

    enum class Phase {
        Idle,     // nothing in flight; the next due update_focus() starts a selection
        Building, // the worker is selecting/building
        Ready,    // a diff waits for poll()
        Applying, // the adapter holds the diff; waiting for commit()
    };

    struct Stats {
        std::uint64_t selections_started = 0;
        std::uint64_t diffs_committed = 0;
        std::uint64_t chunks_built = 0;   // by the worker (not build_initial)
        std::uint64_t chunks_reused = 0;  // new-set keys that were already resident, summed over selections
        std::size_t last_added = 0;
        std::size_t last_removed = 0;
        double last_build_ms = 0.0;       // worker wall time of the last selection + build
        std::size_t resident_count = 0;   // keys committed as resident
    };

    TerrainViewStreamer(TerrainViewSource source, Options options);
    explicit TerrainViewStreamer(TerrainViewSource source) : TerrainViewStreamer(source, Options{}) {}
    ~TerrainViewStreamer();
    TerrainViewStreamer(const TerrainViewStreamer&) = delete;
    TerrainViewStreamer& operator=(const TerrainViewStreamer&) = delete;

    // Synchronous first selection around (x, y) on the calling thread: `out`
    // gets every selected chunk (select_chunks order - identical to
    // build_static_view_from_lookup's output for the same point) and the
    // streamer records those keys as resident and (x, y) as the last
    // selection focus. Only valid while Idle (returns false and leaves `out`
    // untouched otherwise). Optional: without it, the first update_focus()
    // starts a background selection against an empty resident set (a diff
    // with every chunk added, nothing removed).
    bool build_initial(double x, double y, std::vector<RenderChunk>& out);

    // Records the latest focus; starts a background selection for it iff the
    // streamer is Idle and (no selection has happened yet, or the focus is
    // more than reselect_distance_m from the last selection's focus). Never
    // blocks on the worker.
    void update_focus(double x, double y);

    // Non-blocking. If a diff is Ready, moves it into `out`, switches to
    // Applying and returns true; otherwise returns false and leaves `out`
    // untouched.
    bool poll(TerrainViewDiff& out);

    // The adapter has uploaded every add and freed every removal of diff
    // `serial` (contract step 4). Makes that diff's selection the resident
    // set, returns to Idle, then starts a selection for the latest focus if
    // one is already due. Returns false (and changes nothing) if `serial` is
    // not the diff currently being applied.
    bool commit(std::uint64_t serial);

    [[nodiscard]] Phase phase() const;
    [[nodiscard]] Stats stats() const;
    // Committed resident keys, sorted ascending (a copy - thread-safe).
    [[nodiscard]] std::vector<g2m::mesh::ChunkKey> resident_keys() const;

private:
    void worker_main();
    void maybe_start_locked();

    const TerrainViewSource source_;
    const Options options_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    Phase phase_ = Phase::Idle;
    bool stop_ = false;
    bool job_pending_ = false;        // set with phase_ = Building; the worker clears it when it takes the job

    bool has_selection_ = false;      // a selection was started (build_initial or a job) at least once
    double selection_x_ = 0.0;        // focus of the last STARTED selection
    double selection_y_ = 0.0;
    double latest_x_ = 0.0;           // latest update_focus() point
    double latest_y_ = 0.0;
    bool has_latest_ = false;

    std::uint64_t next_serial_ = 1;
    std::uint64_t job_serial_ = 0;
    double job_x_ = 0.0;
    double job_y_ = 0.0;

    std::vector<g2m::mesh::ChunkKey> resident_;   // sorted; changed only by build_initial/commit (worker idle)
    std::vector<g2m::mesh::ChunkKey> target_;     // the Ready/Applying diff's full new key set (sorted)
    TerrainViewDiff ready_;                       // valid while phase_ == Ready
    std::uint64_t applying_serial_ = 0;
    Stats stats_;

    std::atomic<bool> cancel_{false};
    std::thread worker_; // last member: started in the constructor after everything above is initialised
};

} // namespace rg
