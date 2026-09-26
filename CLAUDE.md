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
    src/
      session.cpp                     Session implementation - builds a ps::World by hand (ground + chassis + one vehicle), never parses a scenario JSON
      world_config.cpp                load_world_config() implementation - strict, exception-free JSON validation (see "World config" below)
    CMakeLists.txt                    rg_core STATIC target
  godot_ext/
    src/
      register_types.h/.cpp           GDExtension entry point (rg_godot_library_init), registers RgSimulation
      rg_simulation.h/.cpp            RgSimulation : godot::Node - the one GDScript-facing class; owns one rg::Session
      frame_convert.h                 rg_godot-namespaced wrapper around physics_sim's frame_convert_core.h (Vec3f/basis/pose -> godot::Vector3/Basis/Transform3D)
    CMakeLists.txt                    rg_godot SHARED target (the GDExtension DLL)
  game/                                Godot project (res:// root)
    project.godot                     rg_* input actions, gl_compatibility renderer
    rg_godot.gdextension              points at bin/librg_godot.dll/.so
    bin/                              rg_godot build output (librg_godot.dll + .pdb)
    scenes/
      main.tscn                       one-node stub (Node3D + main.gd) - the scene is built procedurally, see main.gd's own comment
    scripts/
      main.gd                         builds the whole R0 scene in _ready(); per-frame input -> RgSimulation.set_control() wiring
      chase_cam.gd                    reused near-verbatim from physics_sim's demo (same RgSimulation method names)
      gauge_logic.gd                  copied verbatim from physics_sim's demo (engine-neutral static math, no Godot Control dependency)
      tach_gauge.gd                   reused near-verbatim from physics_sim's demo (round tach/speed/gear/lamp gauge)
      hud.gd                          trimmed port of physics_sim's demo hud.gd (debug text HUD; no terrain-tile/haptics lines - R0 has neither)
      input_map.gd                    new plain-GDScript input node (not a C++ GDExtension class like physics_sim's PsInputMap) - keyboard+gamepad polling, larger-magnitude-wins merge
  data/
    world/
      regions.json                    g2m.regions/1 (geo2map_engine G1c I7): region "home" (Main-Taunus-Kreis,
                                       Hochtaunuskreis, Frankfurt-Höchst), halo_m 2000 - consumed by
                                       `g2m_tiler import ...regions.json#home ...` (S:\claude_code\geo2map_engine)
  tests/
    unit/
      catch_main.cpp                  custom Catch2 v3 entry point (installs headless CRT handlers via physics_sim's always-built ps_headless_env)
      test_session.cpp                rg::Session tests: step stability, control-channel round-trip, snapshot/wheel-state sanity
      CMakeLists.txt                  rg_test_catch_main + rg_unit_tests targets, CTest registration
  tools/
    common.ps1                        shared PowerShell helpers (VS dev-shell entry, Godot exe lookup, cmake wrappers, submodule update) - dot-sourced by run.ps1/smoke_test.ps1/ci.ps1
    setup_dev_env.ps1                 -CheckOnly only (installs nothing - see its own header)
    run.ps1, run.cmd                  incremental build + launch Godot on game/
    smoke_test.ps1                    headless Godot smoke test (build, ctest, headless run, assert no errors + sim thread ticking)
    ci.ps1                            Windows CI: debug + release legs (configure, build, ctest) + smoke_test
    hash_check/
      main.cpp, CMakeLists.txt        hash_check executable - the R0 acceptance check, see "Hash comparison acceptance check" below
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
geo2map_engine store locations), `spawn` (`{e, n, yaw_deg}`), `surface_map`/
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
  - a future UE5 port reuses this target unchanged).
- `rg_godot` (SHARED, `godot_ext/`): the GDExtension DLL
  (`game/bin/librg_godot.dll`). `RgSimulation : godot::Node` is the only
  class registered; it owns one `rg::Session` and exposes it to GDScript
  (body transforms, control channels, wheel/gauge/powertrain telemetry,
  origin-rebase seam). Links `rg_core` + `godot-cpp`. This is the ONE place
  `ps::`/`rg::` types cross into `godot::` types (mirrors physics_sim's own
  adapter's "all conversion happens in exactly one place").
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
  `RG_G2M_HOME` default, every validation error path) and a geo2map_engine
  link-smoke test (`test_g2m_link_smoke.cpp` - constructs a `g2m::TileKey`
  and round-trips `packed()`/`unpack()`, both defined out of line in
  `g2m_core`, proving `rg_core` actually LINKS a geo2map_engine symbol, not
  just compiles against its headers). Catch2 v3 via `rg_test_catch_main`,
  plus `nlohmann_json::nlohmann_json` PRIVATE (test fixture JSON is built
  with `nlohmann::json` directly). Registered with CTest.

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
tools\smoke_test.ps1        # headless build + ctest + headless Godot run
tools\ci.ps1                 # debug + release legs + smoke_test
```
