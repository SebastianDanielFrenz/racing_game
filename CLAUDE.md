# racing_game — architecture map

Pure map of what exists and how to build/test/run it. Rationale and design
decisions live in chat/task reports, not here (this repo does not use the
knowledge vault workflow other repos on this machine use - see the R0 task
report for the full decision/pitfall list). Design/phase plan: `PLAN.md`.

## Layout

```
racing_game/
  CMakeLists.txt, CMakePresets.json   top-level build config
  README.md                           setup pointer
  CLAUDE.md                           this file
  PLAN.md                             design and phase plan
  .gitmodules                         external/physics_sim + external/geo2map_engine submodule pins
  external/
    physics_sim/                      git submodule, READ-ONLY from this repo (another session owns it), pinned by commit - see "physics_sim submodule" below
    geo2map_engine/                    git submodule, READ-ONLY from this repo (another session owns it), pinned by commit - see "geo2map_engine submodule" below
  core/
    include/rg/
      session.h                       rg::Session, SessionConfig, FrameSnapshot, WheelSnapshot, kControlChannelNames[]/kControlChannelCount
      world_config.h                  rg::WorldConfig, rg::load_world_config() - see "World config" below
      terrain_view_streamer.h         rg::TerrainViewStreamer (R2.2 R8): render LOD follows a focus point - background reselect + build, key diff {added, removed}, adapter ordering contract in the header; see "Render LOD streaming (R8)" below
      route_check.h                   rg::Route + load_route() ("rg.route/1", exception-free, optional "criteria" object -> RouteCriteria), apply_route_criteria() (RouteCheckParams defaults < route file < CLI, field by field), phys_tile_index()/count_seam_crossings() (255 m physics grid, origin 0.5), sample_l0_height() (bilinear on the L0 cell-centre lattice through any tile lookup), check_route() (length, 10 m-window max/p99 grade plus a steep_stretches list above grade_report_threshold, seam crossings, elevation range, NoData, corner radius = circle through the points +-corner_window_m (10 m) along the route at every 1 m sample (waypoint-density independent) plus a tight_corners list below corner_report_radius_m, start offset vs RouteCheckParams' R5 criteria), route_matches_world(), check_route_on_world() (the same over WorldTerrain::height_tile_shared) - used by tools/route_check and the [realdata] test
    src/
      session.cpp                     Session implementation - builds a ps::World by hand (ground + chassis + one vehicle), never parses a scenario JSON
      world_config.cpp                load_world_config() implementation - strict, exception-free JSON validation (see "World config" below)
      terrain_view_streamer.cpp       TerrainViewStreamer implementation (one worker thread, one diff in flight, coalescing)
      route_check.cpp                 route_check.h implementation
    CMakeLists.txt                    rg_core STATIC target
  godot_ext/
    src/
      register_types.h/.cpp           GDExtension entry point (rg_godot_library_init), registers RgSimulation + RgTerrainView
      rg_simulation.h/.cpp            RgSimulation : godot::Node - the one GDScript-facing class; owns one rg::Session
      rg_terrain_view.h/.cpp          RgTerrainView : godot::Node3D (PLAN.md R2.1) - LOD terrain preview seam, streamed around a focus since R8; see "Terrain preview (R2.1)" and "Render LOD streaming (R8)" below
      frame_convert.h                 rg_godot-namespaced wrapper around physics_sim's frame_convert_core.h (Vec3f/basis/pose -> godot::Vector3/Basis/Transform3D)
    CMakeLists.txt                    rg_godot SHARED target (the GDExtension DLL)
  game/                                Godot project (res:// root)
    project.godot                     rg_* input actions, gl_compatibility renderer
    rg_godot.gdextension              points at bin/librg_godot.dll/.so
    bin/                              rg_godot build output (librg_godot.dll + .pdb)
    scenes/
      main.tscn                       one-node stub (Node3D + main.gd) - the scene is built procedurally, see main.gd's own comment
    scripts/
      main.gd                         builds the whole R0 scene in _ready(); per-frame input -> RgSimulation.set_control() wiring; a `--terrain-preview` cmdline user-arg (after `--`) branches into the R2.1 static-terrain-plus-fly-camera scene instead - see "Terrain preview (R2.1)" below; in that scene the fly camera drives `RgTerrainView.update_focus` every frame (R8) except under `--screenshots`/`--stream-test`
      terrain_stream_test.gd          `--terrain-preview --stream-test` (R8 headless check): after the initial upload, moves the LOD focus through 5 fixed steps from spawn, waits for each streamed diff to be fully applied, prints one line per step + `terrain stream test done: ...`, quits (180 s wall-clock timeout)
      chase_cam.gd                    reused near-verbatim from physics_sim's demo (same RgSimulation method names)
      fly_cam.gd                      free-fly camera script for `--terrain-preview` (PLAN.md R2.1): WASD + Space/E up + Ctrl/Q down, Shift x6 speed, right-mouse-button capture + look, Esc releases capture; no RgSimulation dependency (plain Camera3D script)
      gauge_logic.gd                  copied verbatim from physics_sim's demo (engine-neutral static math, no Godot Control dependency)
      tach_gauge.gd                   reused near-verbatim from physics_sim's demo (round tach/speed/gear/lamp gauge)
      hud.gd                          trimmed port of physics_sim's demo hud.gd (debug text HUD; no terrain-tile/haptics lines - R0 has neither)
      input_map.gd                    new plain-GDScript input node (not a C++ GDExtension class like physics_sim's PsInputMap) - keyboard+gamepad polling, larger-magnitude-wins merge
    shaders/
      terrain.gdshader                hypsometric terrain shader (PLAN.md R2.1): `ALBEDO = COLOR.rgb` (reads RgTerrainView's per-vertex RGBA8 colours); no world-space coordinates anywhere (object-space VERTEX/NORMAL and Godot's own per-fragment builtins only), so it survives the floating-origin rebase unmodified
  data/
    world/
      world_config.json               rg.world/1 (PLAN.md R2.0) instance - see "World config" below; also what `RgTerrainView::initialize`/`tools/lod_measure` open against
      regions.json                    g2m.regions/1 (geo2map_engine G1c I7): region "home" (Main-Taunus-Kreis,
                                       Hochtaunuskreis, Frankfurt-Höchst), halo_m 2000 - consumed by
                                       `g2m_tiler import ...regions.json#home ...` (S:\claude_code\geo2map_engine)
    routes/
      home_r1_drive.json              rg.route/1 (R2.2 R5): the scripted-drive route, waypoints in the session frame + the spawn it starts from; checked by tools/route_check. Spawn (Engelsruhe, Frankfurt-Unterliederbach) -> B 8 -> the Koenigstein city-limit sign, 9.85 km, 2103 OSM-centreline waypoints densified to <= 5 m; its "criteria" (max_grade_pct 31, min_corner_radius_m 7) and every measured number are in its own "source" field
  cache/                               gitignored, LOCAL ONLY - never committed, never read by CI. `cache/g2m/home-r1/` is this machine's copy of the geo2map_engine source/derived store that `data/world/world_config.json`'s `${RG_G2M_HOME}` placeholder (default `S:\claude_code\geo2map_cache\home-r1`, `world_config.cpp`'s `kDefaultRgG2mHome`) resolves against - populated by pointing at (or copying from) an existing geo2map_engine store; nothing in this repo bakes it (`g2m_tiler.exe bake` run from here was denied, see the R2.1 task report). A checkout with no such store cannot open `rg::WorldTerrain` yet. `tools/lod_measure`'s own `RG_G2M_DERIVED` (default `out/g2m_derived/home-r1`, also gitignored) is a SEPARATE on-demand derived-tile cache this repo's own tools populate themselves and is unrelated to `cache/`.
  tests/
    unit/
      catch_main.cpp                  custom Catch2 v3 entry point (installs headless CRT handlers via physics_sim's always-built ps_headless_env)
      test_session.cpp                rg::Session tests: step stability, control-channel round-trip, snapshot/wheel-state sanity
      test_terrain_view_streamer.cpp  rg::TerrainViewStreamer tests (R8) over a synthetic, optionally gated tile store: exact key diff, hole-free adds-then-removals at every step (plus a wrong-order negative control), no work while stationary, 1-vs-8 build-thread identical diffs, coalescing while busy, cancel+join on destruction
      test_route_check.cpp            rg::route_check tests on synthetic terrain (seam counting incl. negative indices/corners, grade window max/p99, NoData, corner radius, start/length criteria, sample_l0_height across tile borders, load_route errors, steep-stretch and tight-corner lists, corner radius on arcs/kinks and its density independence, "criteria" loading + apply_route_criteria precedence); one hidden `[.][realdata]` case runs the committed route on the real store under its own "criteria", SKIP unless RG_G2M_HOME is set
      test_world_terrain.cpp          rg::WorldTerrain / build_static_view_from_lookup tests (PLAN.md R2.1) over a synthetic in-memory TileKey->HeightTile map - no TileStore/Server needed; chunk selection, session-local origin math, 1-vs-N-thread byte-identical output; fetch_height_tile_cached concurrency; decode_height_tile_container on synthetic containers (height_offset of both signs with NoData kept, int32 overflow / NoData-collision rejection, layer/key mismatch, truncation)
      CMakeLists.txt                  rg_test_catch_main + rg_unit_tests targets, CTest registration
  tools/
    common.ps1                        shared PowerShell helpers (VS dev-shell entry, Godot exe lookup, cmake wrappers, submodule update) - dot-sourced by run.ps1/smoke_test.ps1/ci.ps1
    setup_dev_env.ps1                 -CheckOnly only (installs nothing - see its own header)
    run.ps1, run.cmd                  incremental build + launch Godot on game/ - pass `-- --terrain-preview` (see run.ps1's own pass-through-args comment) to launch the R2.1 terrain preview instead of the R0 drivable scene
    smoke_test.ps1                    headless Godot smoke test (build, ctest, headless run, assert no errors + sim thread ticking); `-TerrainPreview` runs the R2.1 headless check instead (asserts >= 150 chunks selected and a completed upload, see its own header comment); `-TerrainStream` (implies -TerrainPreview, R8) adds `--stream-test` and asserts `steps=5 diffs=5 ... missing_removals=0` - with the 0-ERROR-lines check this is the RID-leak check after streamed add/remove diffs
    ci.ps1                            Windows CI: debug + release legs (configure, build, ctest) + smoke_test - this repo's ci.ps1 has NO Linux leg (unlike physics_sim's tools/ci.ps1)
    lod_measure/
      main.cpp, CMakeLists.txt        lod_measure executable (PLAN.md R2.1): measures rg::WorldTerrain::build_static_view cold/warm wall time + chunk/vertex counts against the real data/world/world_config.json at max_distance_m in {6000, 12000, 20000} - see "Terrain preview (R2.1)" below for the measured table and chosen default
    hash_check/
      main.cpp, CMakeLists.txt        hash_check executable - the R0 acceptance check, see "Hash comparison acceptance check" below
    route_check/
      main.cpp, CMakeLists.txt        route_check executable (R2.2 R5, links rg_core only): `route_check [--world-config PATH] [--route PATH] [--min-length-m M] [--max-grade-pct P] [--min-seams N] [--min-corner-radius-m R]` (defaults data/world/world_config.json, data/routes/home_r1_drive.json; the four limits override the route file's "criteria", which override RouteCheckParams' defaults) - prints the effective criteria, then rg::check_route_on_world's report; exit 0 pass, 1 criterion failed, 2 load/usage error
```

## physics_sim submodule

`external/physics_sim` is a git submodule of `S:\claude_code\physics_sim`,
added as a local absolute `file://`-style path (`.gitmodules`'s `url`) per
decision D9. **Every command that clones or updates it must pass
`-c protocol.file.allow=always`**, or git >= 2.38 refuses it with
`fatal: transport 'file' not allowed` (confirmed empirically on git
2.54.0.windows.1) - `tools/common.ps1`'s `Update-PhysicsSimSubmodule`
always does this; a manual clone must do the same by hand:
```
git clone <racing_game url>
cd racing_game
git -c protocol.file.allow=always submodule update --init --recursive
```

`S:\claude_code\physics_sim` (the submodule's source repo) is READ-ONLY
from this repo and its tooling: nothing here ever writes into
`external/physics_sim`, runs physics_sim's own `tools/ci.ps1` or
`adapters/godot/smoke_test.ps1`, or builds inside physics_sim's own `out/`
directory. All builds that touch the submodule's CMake targets build under
`racing_game/out/build/<preset>` instead (`add_subdirectory(external/
physics_sim EXCLUDE_FROM_ALL)` in the top-level `CMakeLists.txt` - the
submodule contributes targets to racing_game's own build graph without
being racing_game's default build set).

## geo2map_engine submodule

`external/geo2map_engine` is a git submodule of `S:\claude_code\geo2map_engine`,
added the same way as `external/physics_sim` above (local absolute
`file://`-style URL, same `-c protocol.file.allow=always` requirement,
`tools/common.ps1`'s `Update-Submodules` covers both). Same READ-ONLY rule:
nothing here ever writes into `external/geo2map_engine` or runs its own
tooling/CI; `add_subdirectory(external/geo2map_engine EXCLUDE_FROM_ALL)` in
the top-level `CMakeLists.txt`, right after the physics_sim block.

Force-set before that `add_subdirectory` (matching geo2map_engine's own
`consumer-check` preset combination exactly): `G2M_BUILD_TESTS=OFF`,
`G2M_BUILD_APPS=OFF`, `G2M_BUILD_IMPORT=OFF` (skips the whole OSM/GeoTIFF
decoder stack and its FetchContent deps - libdeflate/zlib-ng/protozero/
libosmium), `G2M_BUILD_FUZZERS=OFF`. Also forces `CMAKE_C_COMPILER` to match
`CMAKE_CXX_COMPILER` on the clang-cl presets (geo2map_engine's own
`cmake/deps_core.cmake` calls `enable_language(C)` unconditionally for
zstd/SQLite; only its `deps_decoders.cmake`, skipped here via
`G2M_BUILD_IMPORT=OFF`, has its own such guard).

Even with every `G2M_BUILD_*` option OFF, `deps_core.cmake` still
`FetchContent_Declare`s `g2m_zstd`/`g2m_sqlite`/`g2m_json` (needed by
`g2m_layers`/`g2m_server`/`g2m_client` themselves). `RG_G2M_DEPS_DIR` (cache
`PATH`, default `S:/claude_code/geo2map_engine/out/build/release/_deps` on
Windows, `$ENV{HOME}/build/geo2map_engine/linux-release/_deps` on Linux)
points `FETCHCONTENT_SOURCE_DIR_<NAME>` at that directory's own
`<name>-src` subdirectories when they exist, so configuring here never
reaches the network for these - it reuses geo2map_engine's own already-built
release/`linux-release` preset's populated FetchContent sources. Override
with `-DRG_G2M_DEPS_DIR=...` if that path differs on another machine; a
`message(WARNING ...)` fires if the directory does not exist.

## World config

`data/world/world_config.json` (schema `rg.world/1`, PLAN.md R2.0) names one
drivable world region: `format` (must be exactly `"rg.world/1"`), `region`,
`session_origin_utm` (`{zone, e0, n0}` - zone `1..60`, `g2m::geo::UtmZone`'s
own range), `source_store`/`derived_store` (`{dir, scope|name, read_only}` -
geo2map_engine store locations), `spawn` (`{e, n, yaw_deg}` - yaw 0 = east, counter-clockwise, the vehicle's +x axis rotated about +Z), `surface_map`/
`palette` (paths).

`rg::WorldConfig` + `rg::load_world_config(path, err)`
(`core/include/rg/world_config.h` + `core/src/world_config.cpp`) parse and
strictly validate this file - non-throwing (`nlohmann::json::parse(...,
allow_exceptions=false)`, TOOL-019 exception-free contract), returns
`std::optional<WorldConfig>`, `*err` set to a human-readable message on
`nullopt`.

Two distinct path-resolution rules, per field:
- `source_store.dir`/`derived_store.dir`: `${VAR}` placeholder expansion
  only, then used as-is (relative-to-CWD if not already absolute) - these
  name a location outside the repo (a geo2map_engine cache/store), never
  resolved against a repo root.
- `surface_map`/`palette`: resolved against the REPO ROOT, defined as the
  nearest ancestor of the CONFIG FILE'S OWN directory that contains a
  top-level `CMakeLists.txt` (`find_repo_root`) - not the process's CWD, and
  not necessarily racing_game's own root if the config file lives in a
  different checkout. Neither file is required to exist, only the path is
  resolved. A config file with no `CMakeLists.txt` anywhere above it (e.g.
  one written to a bare OS temp directory) fails this resolution - test
  fixtures needing a `surface_map`/`palette` pass live under
  `out/test_tmp/` inside this repo, not the OS temp dir, for exactly this
  reason.

`${VAR}` placeholder expansion applies only to the two `dir` fields above.
`${RG_G2M_HOME}` is special-cased: if the env var is unset it defaults to
`S:\claude_code\geo2map_cache\home-r1` (`kDefaultRgG2mHome`,
`world_config.cpp`) instead of erroring; every other unset/unknown
`${NAME}` is a validation error.

## Terrain preview (R2.1)

First visible, static-LOD terrain plus a fly camera - no vehicle, no
`ps::World` (PLAN.md R2.1).

`rg::WorldTerrain` (`core/include/rg/world_terrain.h` + `core/src/
world_terrain.cpp`): the engine-neutral seam onto geo2map_engine's offline
in-process server/mesh stack. `WorldTerrain::open(config, err)` opens
`config.source_store`/`derived_store`, builds one local geo2map_engine
release (`g2m::builtin_local_release_params` + `g2m::make_local_release` -
the same wiring `g2m_tiler bake` and geo2map_engine's own golden tests use)
and adds it to an offline `g2m::Server` (`offline = true`: its only
`IUpstream` is a `g2m::LocalSourceUpstream`, so it never touches the
network). `build_static_view(cam_x, cam_y, out)` (session-local metres)
runs `g2m::mesh::select_chunks` against `lod_params()` and builds every
selected chunk (`gather_window` + `build_chunk`) into `out` as
`rg::RenderChunk`s, threaded (`build_static_view_from_lookup`'s
`thread_count` param, each worker writing its own assigned index) so the
result is byte-identical for any thread count. `height_tile()` is a
mutex-guarded, cached blocking fetch+decode (per-tile cache, decode work
outside the lock) - `fetch_stats()` reports `cache_hits`/`server_ok`/
`server_miss` since `open()`. The decode is the free function
`decode_height_tile_container(container, expect_layer, expect_key)`
(mirrors geo2map's `TransportHeightTileFetch`: `parse_container`, header
layer/key must match, heap `HeightTile`, the header's `height_offset` added
to every non-NoData sample, `Status::Internal` if a sum would reach NoData
or overflow int32). `RenderChunk` (`rg/terrain_render.h`):
`mesh` (geo2map_engine's own built `TerrainChunkMesh` - positions/normals/
indices, Z-up, local to `mesh.origin`), `origin_session[3]` (`mesh.origin -
(E0, N0, 0)`, still session-local metres, double precision), `rgba` (one
packed RGBA8 colour per vertex - a hypsometric height/slope ramp;
`terrain.class`/a real LandClass palette isn't in the geo2map_engine pin
yet, so every chunk currently shades by height/slope, see
`world_terrain.cpp`'s `chunk_vertex_color()`).

`RgTerrainView : godot::Node3D` (`godot_ext/src/rg_terrain_view.h/.cpp`) -
the Godot-facing seam, mirroring `RgSimulation`'s "all engine-neutral logic
stays in rg_core, this file only converts" shape: `initialize(world_config_
absolute_path)` opens a `WorldTerrain` and caches its spawn point
session-local (`get_spawn_x()`/`get_spawn_y()`, `WorldConfig::spawn.e/n`
minus `session_origin_utm.e0/n0`, so a GDScript caller never has to
re-parse `world_config.json` itself); `load_preview(spawn_x, spawn_y)` runs
one `build_static_view` and queues every chunk for per-frame upload
(`_process`, budgeted `upload_budget_per_frame_` chunks/frame so hundreds
of meshes never stall one frame); `set_render_origin(session_origin)`
re-transforms every already-uploaded instance for the floating-origin
rebase without rebuilding meshes; `set_material(material_rid)` attaches a
`ShaderMaterial`'s RID (`game/shaders/terrain.gdshader`) to every uploaded
chunk's mesh surface - required because Godot's default material does not
read a mesh's own vertex COLOR array as albedo, so without this call the
per-vertex hypsometric colours are built but never actually visible.
Uploads via the low-level `RenderingServer` API directly (`mesh_create`/
`mesh_add_surface_from_arrays`/`instance_create2`/`instance_set_transform`),
one mesh + one instance RID per chunk, tracked and freed in
`free_all_uploaded()` (destructor and start of `load_preview()`, so a
second preview load or a process exit leaks zero RIDs).
`upload_one_chunk()` skips building/uploading the surface array entirely
whenever a chunk's vertex or index count is zero (real data at the
imported region's coverage edge can have vertices but empty `indices` -
assigning an empty `PackedInt32Array` to `ARRAY_INDEX` still sets Godot's
own `ARRAY_FORMAT_INDEX` bit, which then fails `RenderingServer`'s surface
validation) while STILL always creating the mesh/instance RID pair, so the
1:1 `chunks_[i]`/`instance_rids_[i]` indexing invariant `set_render_origin`
relies on is never desynced by a coverage-edge chunk.

`game/scripts/main.gd`'s `--terrain-preview` cmdline user-arg (`OS.
get_cmdline_user_args()`, everything after Godot's own `--`) branches into
`_build_terrain_preview_scene()` instead of the R0 drivable scene: builds
an `RgTerrainView`, initializes it against `data/world/world_config.json`,
builds the `terrain.gdshader` `ShaderMaterial`, loads the preview at the
config's own spawn point, sets the render origin, adds a
`DirectionalLight3D` + `WorldEnvironment` (procedural sky) and a `Camera3D`
named "FlyCam" running `fly_cam.gd`. Prints `"terrain preview selected:
chunks=N vertices=N build_ms=F"` once chunk selection finishes and
`"terrain preview loaded: chunks=N vertices=N upload_ms=F"` once every
chunk has uploaded - `tools/smoke_test.ps1 -TerrainPreview` greps both.

`tools/lod_measure` (executable, links `rg_core` only): sweeps
`max_distance_m` in `{6000, 12000, 20000}` (`max_level` fixed at 6) against
the real committed `data/world/world_config.json`, running
`build_static_view` twice per distance (cold: this process's first touch
of each tile; warm: same `WorldTerrain` instance, tiles already cached).
Measured at the spawn in `data/world/world_config.json` (Engelsruhe, session
(2767.79, -7577.48); re-measured 2026-09-26 after the spawn moved there from
(-1500, 500)): Windows, `release`, one run against an empty `RG_G2M_DERIVED`
(a fresh directory, so every tile is a first touch), against the real
home-r1 store. (The `RG_G2M_DERIVED = ...` line lod_measure prints is always
the default path, even when the env var overrides it.)

| max_distance_m | chunks | total_verts | cold_ms | warm_ms |
|---|---|---|---|---|
| 6000  | 377 | 1,690,845 | 85,233 | 24.07 |
| 12000 | 416 | 1,865,760 | 61,016 | 27.64 |
| 20000 | 474 | 2,125,890 | 80,063 | 32.44 |

At the previous spawn (-1500, 500), three clean runs gave
386/433/488 chunks, 1.73M/1.94M/2.19M vertices, warm 23.5/28.7/32.7 ms.

Chosen default: **20000 m** (`data/world/world_config.json`'s own
`lod.max_distance_m`) - the R2.1 goal is "visible terrain to 16-20 km";
warm cost at 20000 m (~33 ms) is barely above 6000 m's (~23 ms) and both
are comfortably under PLAN.md R2.1's 3000 ms target, so the choice is
driven by coverage instead - 20000 m's 474 chunks/2.13M vertices stay
close to the "~400 chunk" naive-worst-case budget rather than blowing it
up. The COLD numbers above are NOT representative of a baked-store first
load - `cold` means "this process's on-demand Cache->Derive->Upstream path
touching each tile for the first time", 61-116 s regardless of distance;
`g2m_tiler bake` into `cache/g2m/home-r1/derived` was denied by the
session's own permission classifier and was not retried, so only the warm
number is representative of a real pre-baked/pre-derived run.

## Render LOD streaming (R8)

`RenderChunk::key` (`rg/terrain_render.h`) is the chunk's g2m `ChunkKey`
(always equal to `mesh.key`). `build_static_view_from_lookup` is now
`select_view_keys` (selection only) + `build_render_chunks` (build a given
key list, index-parallel, optional cancel flag), both public in
`rg/world_terrain.h`; `TerrainViewSource` bundles LOD params, session origin
and a thread-safe tile lookup (`WorldTerrain::view_source()` returns one over
itself - destroy the streamer before the `WorldTerrain`).

`rg::TerrainViewStreamer` (`rg/terrain_view_streamer.h/.cpp`, Godot-free):
`build_initial(x, y, out)` (synchronous first selection, optional),
`update_focus(x, y)` (non-blocking; starts one background selection once the
focus is more than `Options::reselect_distance_m` = 128 m from the last
selection's focus), `poll(diff)` (non-blocking), `commit(serial)`.
`TerrainViewDiff{serial, focus, added (built RenderChunks), removed (keys)}`,
both sorted by key; only keys not already resident are built. Adapter
contract: upload every add (any per-frame budget), THEN free the removals and
`commit()` - never the other way round, so the view never drops below one
complete selection. One diff in flight: no new selection while a diff is
Ready/Applying; `commit()` starts one follow-up for the latest focus (moves
while busy coalesce). Phases `Idle/Building/Ready/Applying`; `stats()`.
Destructor cancels (checked before each chunk) and joins the worker.

Godot side (`RgTerrainView`): `load_preview` seeds a streamer via
`build_initial` (same chunk set/order as `build_static_view`) and uploads it
with the 8-chunks/frame count budget (a loading phase). After that,
`update_focus(session_x, session_y)` forwards to the streamer and `_process`
runs `apply_diff`: poll -> append adds to the upload queue -> upload under
`upload_budget_ms` (default 0.8 ms, steady clock, at least one operation per
frame) -> once every add is up, free removed chunks' RIDs under the same
budget (possibly over several frames) -> `commit`. `chunks_`/`mesh_rids_`/
`instance_rids_` stay index-aligned (append; swap-remove all three) with a
`std::map<ChunkKey, size_t>` key->index map. `is_fully_uploaded()` is also
false while a diff is half-applied; `is_stream_idle()`; `godot_to_session()`
(render-frame position -> session XY for the focus); `get_stream_stats()`
(pending_adds/removals, last_diff_added/removed, upload_ms_this_frame,
last_remove_ms, diffs_applied, diff_errors, resident_chunks,
selections_started, last_build_ms, streamer_phase). Not wired into drive
mode (R9). `tools/smoke_test.ps1 -TerrainStream` (debug DLL, real home-r1
store): 5 steps, 708 added / 708 removed, 76-226 adds per diff, background
build 108-293 ms, streaming frames 1.6-2.0 ms max (one op may overshoot the
0.8 ms budget), removals 4-9 ms per diff spread over frames.

## Targets

- `rg_core` (STATIC, `core/`): `rg::Session` - owns one `ps::World` (one
  static ground box, one dynamic chassis body, one vehicle built from
  `ps::io::load_vehicle_json`), a `ps_godot::SimThread` (240 Hz fixed-rate
  sim thread, reused BY PATH from `external/physics_sim/adapters/godot/
  src/sim_thread.h` - Godot-free), a `ps_godot::TripleBuffer<FrameSnapshot>`
  (reused by path, `triple_buffer.h`) for race-free snapshot publication,
  and a `ps_godot::OriginRebase` (reused by path, `origin_rebase.h`/
  `frame_convert_core.cpp`) for floating-origin support. Also `rg::
  load_world_config` (`world_config.h`/`.cpp` - see "World config" below).
  Links `ps_core` PUBLIC, plus (PLAN.md R2.0) `g2m_server`/`g2m_client`/
  `g2m_layers` PUBLIC and `nlohmann_json::nlohmann_json` PRIVATE (used only
  inside `world_config.cpp`'s own implementation). `g2m_mesh` does not exist
  yet on geo2map_engine's current pin - not linked; add it once it lands. No
  Godot type anywhere (engine-neutral, MEMORY.md's engine-neutral-logic rule
  - a future UE5 port reuses this target unchanged). `g2m_mesh` (PLAN.md
  R2.1, landed on the submodule pin with G2.3) adds `rg::WorldTerrain`
  (`world_terrain.h`/`.cpp`) + `rg::RenderChunk`/`build_static_view_from_
  lookup` (`terrain_render.h`) - see "Terrain preview (R2.1)" above.
- `rg_godot` (SHARED, `godot_ext/`): the GDExtension DLL
  (`game/bin/librg_godot.dll`). Two classes registered: `RgSimulation :
  godot::Node` owns one `rg::Session` and exposes it to GDScript (body
  transforms, control channels, wheel/gauge/powertrain telemetry,
  origin-rebase seam); `RgTerrainView : godot::Node3D` (PLAN.md R2.1) owns
  one `rg::WorldTerrain` and uploads its `RenderChunk`s via the low-level
  `RenderingServer` API - see "Terrain preview (R2.1)" above. Links
  `rg_core` + `godot-cpp`. This is the ONE place `ps::`/`rg::`/`g2m::` types
  cross into `godot::` types (mirrors physics_sim's own adapter's "all
  conversion happens in exactly one place").
- `lod_measure` (executable, `tools/lod_measure/`): links `rg_core` only
  (no Godot) - the PLAN.md R2.1 LOD-distance measurement sweep, see
  "Terrain preview (R2.1)" above for the tool and its measured table.
- `hash_check` (executable, `tools/hash_check/`): builds an `rg::Session`
  matching `external/physics_sim/data/scenarios/vehicle_step_steer.json`
  by hand (ground/chassis/vehicle construction + the same control events),
  steps it, and prints its `state_hash` for comparison against physics_sim's
  own `ps_run` on the same scenario file - see "Hash comparison acceptance
  check" below.
- `rg_test_catch_main` (STATIC, `tests/unit/`): custom Catch2 v3 entry
  point (own fetch, v3.16.0 - NOT physics_sim's `ps_test_catch_main`, which
  only exists when `PS_BUILD_TESTS=ON`, and this repo sets it `OFF`).
  Links `Catch2::Catch2` + physics_sim's always-built `ps_headless_env`.
- `rg_unit_tests` (executable, `tests/unit/`): `rg_core`'s own tests
  (`test_session.cpp`), `rg::load_world_config` coverage
  (`test_world_config.cpp` - happy path, `${VAR}` expansion, the
  `RG_G2M_HOME` default, every validation error path, plus PLAN.md R2.1's
  `lod`/`max_distance_m` cases), a geo2map_engine link-smoke test
  (`test_g2m_link_smoke.cpp` - constructs a `g2m::TileKey` and round-trips
  `packed()`/`unpack()`, both defined out of line in `g2m_core`, proving
  `rg_core` actually LINKS a geo2map_engine symbol, not just compiles
  against its headers) and `rg::WorldTerrain`/`build_static_view_from_lookup`
  coverage (`test_world_terrain.cpp` - PLAN.md R2.1, synthetic in-memory
  `TileKey`->`HeightTile` map, see "Terrain preview (R2.1)" above). Catch2
  v3 via `rg_test_catch_main`, plus `nlohmann_json::nlohmann_json` PRIVATE
  (test fixture JSON is built with `nlohmann::json` directly). Registered
  with CTest.

## CMake options

- `RG_BUILD_TESTS` (default `ON`): build `rg_unit_tests`, fetch Catch2.
- `RG_BUILD_GODOT_EXTENSION` (default `ON`): build `rg_godot`, fetch
  godot-cpp (branch `4.5`). Godot RUNTIME is 4.7.2 (`compatibility_minimum
  = "4.5"`, `compatibility_maximum = "4.7"` in `game/rg_godot.gdextension`).

physics_sim's own build options are force-set before its
`add_subdirectory`: `PS_BUILD_TESTS=OFF` (its own Catch2/`ps_run`/`sweep`/
bench targets are not needed here - `ps_run` can still be built explicitly
by target name for the hash-check acceptance evidence, see below),
`PS_BUILD_GODOT_ADAPTER=OFF` (its own Godot adapter/demo are irrelevant to
this repo).

geo2map_engine's own build options (`G2M_BUILD_TESTS`/`G2M_BUILD_APPS`/
`G2M_BUILD_IMPORT`/`G2M_BUILD_FUZZERS`, all forced `OFF`) and
`RG_G2M_DEPS_DIR` are documented under "geo2map_engine submodule" above.

## Dependencies (FetchContent, pinned)

| Dependency | Pin | Notes |
|---|---|---|
| physics_sim | submodule commit (see `.gitmodules`/`git submodule status`) | brings JoltPhysics v5.6.0 + nlohmann/json v3.12.0 transitively, `PS_BUILD_TESTS=OFF`/`PS_BUILD_GODOT_ADAPTER=OFF` |
| geo2map_engine | submodule commit (see `.gitmodules`/`git submodule status`) | brings zstd v1.5.7 + SQLite 3.53.4 + its own nlohmann/json v3.12.0 pin transitively (`g2m_dep_*`, PRIVATE to geo2map_engine's own targets); `G2M_BUILD_TESTS=OFF`/`G2M_BUILD_APPS=OFF`/`G2M_BUILD_IMPORT=OFF`/`G2M_BUILD_FUZZERS=OFF` |
| godot-cpp | branch `4.5` | only when `RG_BUILD_GODOT_EXTENSION=ON` |
| Catch2 | `v3.16.0` | only when `RG_BUILD_TESTS=ON`; `/EHsc` applied to its targets under MSVC-family compilers |

## CMake presets

Windows (Ninja; run from a VS 2022 Build Tools developer environment -
`tools/run.ps1`/`smoke_test.ps1`/`ci.ps1` enter it automatically):

| Preset | Compiler | Build type |
|---|---|---|
| `debug` | clang-cl | Debug |
| `release` | clang-cl | Release |
| `relwithdebinfo` | clang-cl | RelWithDebInfo |
| `asan` | clang-cl | RelWithDebInfo + AddressSanitizer |
| `msvc-release` | cl | Release |

`asan`'s ASan runtime `/WHOLEARCHIVE` linking (top-level `CMakeLists.txt`)
applies to `CMAKE_EXE_LINKER_FLAGS`, `CMAKE_SHARED_LINKER_FLAGS` and
`CMAKE_MODULE_LINKER_FLAGS` alike - needed because `rg_godot`
(`godot_ext/`) is a `SHARED` target, not an executable, and lld-link never
implicitly links the ASan runtime for clang-cl regardless of target type.

Linux (`linux-release`, Ninja + clang + lld, binary dir
`~/build/racing_game/<preset>`): `RG_BUILD_GODOT_EXTENSION` forced `OFF`
(godot-cpp is Windows-only in this repo's current scope).

`EXCLUDE_FROM_ALL` note: the submodule's own targets (`ps_run`, `sweep`,
etc.) are NOT in the default build set. Build one explicitly by name if
needed, e.g. `cmake --build out\build\debug --target ps_run` (used for the
hash-check acceptance evidence).

## Hash comparison acceptance check

`hash_check.exe [data_dir]` (default `external/physics_sim/data`) builds
an `rg::Session` with the SAME ground/chassis/vehicle parameters and
control-event sequence as `external/physics_sim/data/scenarios/
vehicle_step_steer.json`, steps it 960 ticks at 240 Hz, and prints
`state_hash=0x...`. Comparing this against physics_sim's own `ps_run
external/physics_sim/data/scenarios/vehicle_step_steer.json` run from
inside this repo's own build tree is a genuine regression test of
`rg::Session`'s hand-written world-construction code (`build_world_
contents` in `core/src/session.cpp`) against physics_sim's own scenario
loader - not a tautological reload of the same JSON. A mismatch means
`Session`'s construction diverges from the scenario's own semantics
somewhere (wrong default, wrong body order, missed control event, etc.).

## Building, testing, running

```powershell
tools\setup_dev_env.ps1 -CheckOnly   # verify toolchain (installs nothing)

# from a VS 2022 developer shell (tools\run.ps1/ci.ps1/smoke_test.ps1 enter one automatically)
git -c protocol.file.allow=always submodule update --init --recursive
cmake --preset debug -DRG_BUILD_GODOT_EXTENSION=ON
cmake --build --preset debug
ctest --preset debug --output-on-failure

tools\run.ps1              # incremental build + launch Godot on game/
tools\run.cmd -- --terrain-preview   # ... or launch the R2.1 static-terrain fly-camera preview instead
tools\smoke_test.ps1        # headless build + ctest + headless Godot run
tools\smoke_test.ps1 -TerrainPreview # ... or the R2.1 terrain-preview headless check
tools\smoke_test.ps1 -TerrainStream  # ... or the R8 streamed-LOD headless check (focus moved in 5 steps)
tools\ci.ps1                 # debug + release legs + smoke_test
```
