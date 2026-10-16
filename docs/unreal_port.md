# Unreal Engine 5 port plan

Owner request (PR #5, 2026-10-14): "Prepare for a port to Unreal within the next few days."

The repo's rule already points at this port: all game logic lives in the engine-neutral C++ library `rg_core`,
`godot_ext/` is a thin binding layer and `game/scripts/` is presentation. This document measures how true that is
today, lists what still has to move, and lays out the UE5 project, the engine services the port replaces and the
milestones. Questions for the owner are in section 14, each with a recommendation.

## 1. Audit: what the Godot side holds today

Method: every `game/scripts/*.gd` and `godot_ext/src/*` file read or skimmed; lines (raw, incl. comments) assigned to
one of four classes. Mixed files were split by function. The numbers are estimates at function granularity, measured
on `claude/plan-work-q2a5oy` at fb590ec (before the moves in section 2).

- **(a) presentation**: drawing, widgets, scene building, camera rigs' node handling, debug text. Rewritten per
  engine.
- **(b) leaked logic**: game rules, math or data interpretation that an Unreal front end would have to re-implement
  identically. Must move to `rg_core` before or during the port.
- **(c) engine service**: rendering upload, audio output, device input, GLTF loading, floating origin, threads,
  XR, HTTP, DPI. The port replaces these with the UE equivalent.
- **(t) test / tool harness**: `--*-test`, `--*-shots`, smoke and tour scripts. Re-created as UE automation tests
  only where they guard a contract.

### 1.1 Totals

| | lines | (a) presentation | (b) leaked logic | (c) engine service | (t) tests/tools |
|---|---|---|---|---|---|
| `game/scripts/*.gd` (73 files) | 17,489 | 10,301 (59 %) | 1,166 (7 %) | 1,199 (7 %) | 4,823 (28 %) |
| `godot_ext/src/*` (24 files) | 6,316 | 5,379 thin conversion (85 %) | 212 (3 %) | 725 (11 %) | 0 |

Read: the binding layer is clean (3 % leaks). GDScript holds about 1,200 lines of logic that would otherwise be
written twice; the rest is presentation and tests, which a port rewrites anyway.

### 1.2 Per file (GDScript, non-test files with any (b) or (c) share, largest first)

| file | lines | (a) | (b) | (c) | (t) |
|---|---|---|---|---|---|
| main.gd | 1932 | 1523 | 227 | 90 | 92 (`--bindings-test`) |
| shell_ui.gd | 1739 | 1712 | 27 | 0 | 0 |
| garage_scene.gd | 826 | 766 | 0 | 60 (GLB load) | 0 |
| cockpit_controls.gd | 635 | 335 | 200 | 100 (ray pick, mouse) | 0 |
| input_map.gd | 581 | 331 | 70 | 180 (devices, hot-plug, capture) | 0 |
| vehicle_visual.gd | 368 | 215 | 29 | 124 (GLTFDocument, node binding) | 0 |
| vehicle_audio.gd | 284 | 42 | 12 | 230 (spatial backend, players) | 0 |
| address_teleport.gd | 263 | 178 | 70 | 15 (HTTPRequest) | 0 |
| car_thumbnails.gd | 262 | 202 | 0 | 60 (SubViewport render) | 0 |
| building_stream.gd | 244 | 109 | 95 | 40 (Thread/Mutex) | 0 |
| xr_menu.gd | 206 | 166 | 0 | 40 (XR) | 0 |
| road_stream.gd | 169 | 89 | 45 | 35 (Thread/Mutex) | 0 |
| walker_rig.gd | 151 | 131 | 20 | 0 | 0 |
| camera_director.gd | 147 | 107 | 0 | 40 (floating origin) | 0 |
| cockpit_common.gd | 145 | 65 | 20 | 60 (GLB extraction) | 0 |
| drone_rig.gd | 132 | 97 | 35 | 0 | 0 |
| chase_rig.gd | 109 | 84 | 25 | 0 | 0 |
| seat_adjustment.gd | 100 | 85 | 10 | 5 | 0 |
| gauge_logic.gd | 87 | 0 | 87 | 0 | 0 |
| fly_cam.gd | 82 | 62 | 0 | 20 | 0 |
| orbit_rig.gd / free_rig.gd | 157 | 137 | 20 | 0 | 0 |
| tyre_sound_model.gd | 79 | 0 | 79 | 0 | 0 |
| adaptive_steering.gd | 76 | 0 | 76 | 0 | 0 |
| xr_rig.gd / ui_scale.gd | 90 | 0 | 0 | 90 | 0 |
| model_options.gd / hud.gd / npc_traffic.gd | 513 | 484 | 19 | 10 | 0 |

Pure presentation (no (b)/(c)): controls_ui, car_browser_ui, tach_gauge, body_visuals, digital_cluster,
loading_overlay, the remaining rigs, the HUD helpers (drive_mode_theme, thermal_readout, driver_state,
exterior_lamps, slider_box, hyper_live_cluster, frame_probe), npc_truck, walker_visual, garage_dyno_chart.
Pure test/tool: the 25 `*_test.gd`, `*_shots.gd`, `*_smoke.gd`, `*_tour.gd` files.

godot_ext: (c) = `rg_terrain_view.cpp` RenderingServer upload/diff/free/rebase (~360 lines incl. `heal_tick`),
`rg_simulation.cpp` init worker thread + cancel/reap (~170), `rg_engine_audio.cpp`, `frame_convert.h`,
`register_types.cpp`; (b) = the items marked "binding" in 1.3.

### 1.3 Leaks (b), with location

Ordered by port value (how much an Unreal front end would otherwise duplicate). "Moved" = done in this package.

| # | where | what | status |
|---|---|---|---|
| 1 | `adaptive_steering.gd` (whole file) | steer input shaping: keyboard ramp, speed-dependent lock limit, counter-steer authority | **moved** -> `rg/steering_assist.h` |
| 2 | `address_teleport.gd:140-262` | coordinate-query parsing, Photon URL, GeoJSON parsing, hand-written UTM series, destination range check; rate limit / result count / language hard-coded | **moved** -> `rg/address_search.h` (UTM now via `g2m::geo::Utm`); limits now in `data/controls/geocoding.json` |
| 3 | `main.gd:1123-1204` `_forward_driving` | control merge: cockpit override vs device (max / override-wins), panel-open brake hold, shift-count accumulation, park release on shift, drive-mode message | open: `rg::DriverControlMerge` (frame in: device levels + cockpit overrides + flags; out: channel writes) |
| 4 | `main.gd:1267-1330` `_process_body` | in-world action routing: which edge press is valid in which screen/mode/world state (truck, view cycle, get out/in, jump, drone target, reset, flip) | open: `rg::FrameCommands` next to `ShellFlow`/`PlayerModeMachine` |
| 5 | `cockpit_controls.gd:270-592` | `rg.cockpit/1` interpretation: per-kind pose mapping (needle/knob/stalk/toggle/slider/button/gear gate), click/step/drag actions, ignition detents, cosmetic switch state, invented oil-pressure/battery readings | open: `rg::CockpitModel` (pose values + action results; GDScript keeps picking and node transforms) |
| 6 | `building_stream.gd:87-209` | building tile scheduling, wall extrusion, roof triangulation incl. even/odd courtyard slabs (uses Godot `Geometry2D.triangulate_polygon`) | open: `rg::tessellate_buildings` (needs a C++ triangulator, e.g. earcut.hpp) |
| 7 | `road_stream.gd:85-106`, `building_stream.gd:87-110` | duplicated 1 km tile wanted-set + "behind a fast car goes last" ordering | open: one `rg::tile_schedule(focus, velocity, radius)` |
| 8 | `tyre_sound_model.gd` (whole file) | tyre audio DSP (roll noise, scrub, squeal crossfade) | open: `rg::TyreSound` (PCM out; front end only outputs) |
| 9 | `input_map.gd:146-164`, `:332-413`, `:567-576` | auto-shift default from the vehicle file's `manual_tcu`, edge counting, ignition toggle state, steering-wheel angle from steer ratio | open: fold into `rg::Controls` / `rg::DriverActions` |
| 10 | `chase_rig.gd:62-107`, `drone_rig.gd:64-130`, `walker_rig.gd:81-102`, `free_rig.gd`, `orbit_rig.gd` | orbit-yaw target from stick/keys, zoom law, lag filters, terrain-clearance probes | partly done (2026-10-15): the drone part is `rg::drone_*` in `rg/camera_math.h` (`drone_follow_step`, `drone_zoom_step`, `drone_target_orbit_yaw`, `drone_clear_terrain`; golden test against the former GDScript), `drone_rig.gd` and the VR `xr_drone` rig only draw; chase_rig/walker_rig/free_rig/orbit_rig remain open |
| 11 | `main.gd:1031-1046` `_save_drive_location`, `main.gd:1888-1932` `_log_owner_mark` | last_drive.json / drive_marks.jsonl records (session -> UTM, compass heading) | open: writer next to the reader in `rg/spawn_presets.h` |
| 12 | `shell_ui.gd:1338` `_tyre_presentation` | parses tyre ids and applies the rated-speed cap - duplicates `rg/tyre_spec.h` | open: `RgGarage` returns the parsed spec |
| 13 | `gauge_logic.gd` (whole file) | tach scale (max rpm from limiter, ticks, redline, gear label, lamp state) | open (small; UMG would re-implement it) |
| 14 | `seat_adjustment.gd:26-28` | seat offset limits hard-coded; per-car offsets in a Godot ConfigFile | open: limits to data, store via `rg::Settings` |
| 15 | `model_options.gd:24-36` | hood variant choice by engine height | open: next to `rg::check_fit` |
| 16 | `vehicle_visual.gd:258-286` | model-to-physics wheel alignment (mean offset + per-wheel bias) | open (engine-neutral math on rig.json + VehicleDesc) |
| 17 | binding `rg_simulation.cpp:785-818` | `get_vehicle_gauge_info` walks VehicleDesc components | open: `rg::gauge_info(VehicleDesc)` |
| 18 | binding `rg_simulation.cpp:820-850` | park-lock fitted scan, drive power formula, assist bit decode (1/2/4/8) | open: `rg::assist_hud` already exists - extend |
| 19 | binding `rg_simulation.cpp:659-672` | steering kinematics computed through a Godot basis round trip | open: `rg::chassis_local_motion(FrameSnapshot)` |
| 20 | binding `rg_simulation.cpp:1000-1015` | drone-target labels ("NPC truck", "NPC lorry #n") | done (2026-10-15): `rg::drone_target_label` in `rg/player_mode.h`, the binding passes the lorry flag |
| 21 | binding `rg_terrain_view.cpp:268-383` | ground-check / bed-check probe loops | open: `rg/terrain_stream.h` (owned by the heal agent area today) |

## 2. Moved in this package

- `core/include/rg/steering_assist.h` + `core/src/steering_assist.cpp`: `rg::SteeringAssist`,
  `load_steering_assist_config` (strict, every key required), `steering_geometry_from_vehicle_json`.
  Binding `godot_ext/src/rg_steering_assist.*` (`RgSteeringAssist`, RefCounted). `adaptive_steering.gd` keeps its
  method names and forwards; `main.gd` is unchanged. Test `tests/unit/test_steering_assist.cpp` [steering_assist]
  replays a golden sequence produced by the original GDScript under Godot 4.7.2 (29 outputs, 1e-9).
- `core/include/rg/address_search.h` + `core/src/address_search.cpp`: config loader, coordinate query, cache key,
  URL, Photon response parsing, destination check through `g2m::geo::Utm` (WGS84). Binding
  `godot_ext/src/rg_address_search.*` (`RgAddressSearch`, static). `address_teleport.gd` keeps the panel, the
  HTTPRequest and the cache file. `data/controls/geocoding.json` gained `result_limit`, `language`,
  `min_request_interval_s` (were literals). Test `tests/unit/test_address_search.cpp` [address_search]; the
  projection agrees with the former GDScript series to < 1 cm at Frankfurt.

## 3. UE5 project and module layout

```
RacingGameUE/                              (new repo or racing_game/ue/; see question Q1)
  RacingGame.uproject
  Plugins/RacingGameCore/
    RacingGameCore.uplugin
    Source/ThirdParty/RgLibs/              produced by tools/build_ue_libs.ps1, not edited by hand
      RgLibs.Build.cs                      ModuleType.External: PublicIncludePaths + PublicAdditionalLibraries
      include/                             rg/, ps/, ps_audio/, g2m/, generated g2m layer headers, nlohmann/
      lib/Win64/{rg_core,ps_core,ps_audio,Jolt,g2m_*,zstd,sqlite3}.lib
      BUILD_STAMP.json                     source sha, submodule shas, preset, compiler
    Source/RgRuntime/                      the binding layer (successor of godot_ext)
      RgSimulationSubsystem                UWorldSubsystem: owns rg::Session + PlayerModeMachine, init worker
      RgTerrainComponent                   chunk upload (RealtimeMesh or ProceduralMesh), diff apply, rebase
      RgRoadComponent / RgBuildingComponent  road ribbons, decks, building tiles
      RgVehicleActor                       car visual from GLB + rig.json, wheel poses from FrameSnapshot
      RgAudioComponent                     engine voice + tyre PCM out
      RgInputComponent                     raw device snapshot -> rg::Controls
      RgShellSubsystem                     UGameInstanceSubsystem: ShellFlow, Settings, Credits, Garage, CarBrowser
      RgFrame.h                            ISO <-> UE conversions (section 4), the only place they happen
  Source/RacingGame/                       game module: GameMode, PlayerController, camera rigs, UMG widgets
  Content/                                 materials, UMG assets, input assets (no gameplay data)
```

Rules carried over: `RgRuntime` converts and forwards (same as godot_ext today); no `rg::` type in Blueprint
signatures; game data stays in `data/` and is staged as non-asset files (`DirectoriesToAlwaysStageAsNonUFS`).

**How CMake feeds UBT.** UBT does not run CMake. A new preset `ue-win64` (clang-cl or cl, see Q2) builds the static
libraries with the UE-compatible flags below, and `tools/build_ue_libs.ps1` runs `cmake --build` +
`cmake --install --prefix <plugin>/Source/ThirdParty/RgLibs` and writes `BUILD_STAMP.json`. `RgLibs.Build.cs` lists
the `.lib` files and fails with a clear message when the stamp's source sha differs from `git rev-parse HEAD` of the
game repo (stale libraries). CI runs the script before `RunUAT BuildCookRun`. Compiling the ~600 sources in UBT
directly was rejected: Jolt's ISA defines, geo2map's generated headers and the FetchContent dependencies are already
solved in CMake.

Flags the `ue-win64` preset must set (ABI with UE's own modules):
- `CMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL` (/MD) for every config: UE links the release CRT even in
  DebugGame/Development; `_ITERATOR_DEBUG_LEVEL=0`.
- `/EHsc` and RTTI ON inside the libraries (section 12).
- No LTO across the boundary (`/GL` off), same Windows SDK and C++20.
- Jolt stays private to ps_core (physics_sim keeps Jolt types out of its public headers), so UE translation units
  never see `JPH_*` defines; a test in the preset checks that no public header includes `Jolt/`.
- Symbol clashes: g2m links its own SQLite and zstd. Keep UE's `SQLiteCore` plugin disabled, or build g2m's SQLite
  with `SQLITE_API` renamed; check zstd against the engine's ThirdParty list for the UE version chosen.

**Macro hygiene.** UE defines function-like macros `check`, `verify`, `ensure`, `TEXT`. `ps/environment/weather.h:282`
declares a member named `ensure(...)`, which breaks any UE translation unit that includes it after `CoreMinimal.h`.
Include engine-neutral headers only through `RgThirdParty.h`:
```cpp
THIRD_PARTY_INCLUDES_START
#pragma push_macro("check") / ("verify") / ("ensure")
#undef check
#undef verify
#undef ensure
#include "rg/session.h"   // and friends
#pragma pop_macro(...)
THIRD_PARTY_INCLUDES_END
```

## 4. Coordinate frames

| frame | axes | unit | handedness |
|---|---|---|---|
| ISO 8855 / session (rg, ps, g2m) | x forward (east in the session frame), y left (north), z up | m | right |
| Godot (today) | `godot = (-iso.y, iso.z, -iso.x)` (`frame_convert_core.h`) | m | right |
| Unreal | X forward, Y right, Z up | cm | left |

Mapping (the only conversion code, `RgFrame.h`):
- position: `ue = (iso.x, -iso.y, iso.z) * 100`. Session x = east -> UE +X, session y = north -> UE -Y.
- rotation: the y mirror conjugates rotations: matrix `R_ue = M R_iso M` with `M = diag(1, -1, 1)`; quaternion
  `(w, x, y, z)_iso -> (w, -x, y, -z)_ue`. Yaw changes sign (ISO + = counter-clockwise from above, UE + = clockwise):
  `FRotator.Yaw = -yaw_iso_deg`, `Roll = -roll_iso_deg`, `Pitch = pitch_iso_deg` with UE's nose-up-positive pitch
  checked against ISO's nose-down-positive pitch in the conversion test (expected sign flip; pin it by test, not by
  hand).
- velocities and directions: same as positions (x100 for linear; angular: `(-wx, wy, -wz)` for the pseudo-vector).
- Unit tests in the plugin (UE automation): round trip of a pose, a turning car's yaw rate sign, a wheel steer
  angle sign (left steer must turn the front wheels to the left on screen), and the ground plane normal.

Only `RgFrame.h` multiplies by 100. Every rg/ps/g2m call stays in metres, ISO.

## 5. Large world coordinates vs the floating origin

Today: physics runs in the session frame (doubles where it matters), the Godot render frame is single precision, so
`ps_godot::OriginRebase` + `camera_director.gd` shift every rig root and `RgTerrainView::set_render_origin` moves
every chunk instance when the camera leaves a radius.

UE5 renders with large world coordinates (double transforms, tile-relative GPU math). The session frame spans about
±30 km (3e6 cm), well inside LWC's precise range. Plan:
- Put the session origin at the UE world origin and do **not** rebase. `RgFrame` keeps an `origin` parameter (zero)
  so a rebase can come back if a measurement shows jitter.
- Each terrain/road/building section keeps its own local origin (`RenderChunk::origin_session`) as the component
  transform, so vertex positions stay small floats.
- The PHYS-008 camera contract (one owner of the origin, all rigs shifted in the same frame) disappears with the
  rebase; the camera-switch test keeps the "no jump" part.
- World Partition / level streaming are not used: the world is procedural; `TerrainViewStreamer` stays the
  streaming authority.

## 6. Terrain chunk upload

`rg::TerrainViewStreamer` already produces diffs `{added RenderChunks, removed keys}` on its worker, sorted by urgency,
with the rule "upload every add, then free removals, then commit". The UE adapter mirrors `RgTerrainView::apply_diff`:
- Component: **RealtimeMeshComponent** (RMC, MIT, Fab/GitHub) - section updates without PMC's per-call
  `TArray<FVector>` conversion and without collision cooking; mesh data can be filled off the game thread and
  committed on it. Fallback: `UProceduralMeshComponent` with `bCreateCollision = false` and positions converted with
  one pass over the chunk. See Q4.
- One component per chunk (~400-500 resident), transform = chunk origin (section 5). Colour = the per-vertex RGBA8
  the core already computes; normals from the chunk; no UE collision (physics is ps/Jolt).
- Budget: reuse `rg::adaptive_upload_budget_ms` (0.8 ms rising to ~3 ms with backlog) on the game thread; build the
  RMC stream sets on a task (`UE::Tasks`) and only commit on the game thread.
- Heal path (`RenderChunk::carve`, `rg::stale_fallback_chunks`, `replace_chunk_at`): same order, upload the
  replacement first, free the old section after.
- The probes (`get_ground_check`, `get_bed_check`) should move to core first (leak #21).

## 7. Road and building meshes, decks

- Road ribbons: `RgTerrainView::get_road_visual_tile` already returns core-built arrays (vertices, normals, uv, uv2,
  colours, indices) -> one RMC section per 1 km tile. The scheduling in `road_stream.gd` moves to core (leak #7).
- Road decks / bridge slabs: `Session::sync_road_decks` keeps physics; the render side is the deck mesh list ->
  RMC sections.
- Buildings: today the extrusion and roof triangulation are GDScript (leak #6) - move to core before the port, then
  the UE side is again "arrays -> section" with near/tall batches. Instanced or HISM meshes for later asset-based
  buildings (docs/buildings.md).
- Ribbon depth pull (road.gdshader `depth_pull_*`): rebuild as a material World Position Offset toward the camera or
  as a decal/translucency sort; see section 11.

## 8. Car visual

Today `vehicle_visual.gd` loads `data/models/<car>/<car>.glb` at runtime (GLTFDocument), binds `susp_*`/`steer_*`/
`wheel_*` by the names in `<car>.rig.json`, aligns the model to the physics wheel attachments and poses the nodes from
`FrameSnapshot::wheels`. `cockpit_common.gd` extracts common part trees from `cockpit_parts.glb`.

Options:
- **glTFRuntime** (MIT plugin): loads the same GLB at runtime with its node hierarchy and names - a direct port of
  today's flow, no import step, cargen output usable immediately.
- **Interchange** (editor import): cooked StaticMesh/Blueprint assets, better LODs/materials, but the cargen GLB ->
  asset step becomes part of the pipeline and node names must survive import.
Recommendation: glTFRuntime for the first playable (parity), Interchange later for shipping art (Q5). The alignment
math (leak #16) and the cockpit model (leak #5) move to core first so both paths share them.

## 9. Sim thread, game thread, triple buffer

Unchanged in principle. `rg::Session` runs its own 240 Hz `FixedRateLoop` thread; it publishes one `FrameSnapshot`
per tick through `ps_godot::TripleBuffer` (Godot-free header reused by path). The UE side:
- `URgSimulationSubsystem::Tick` (game thread, `TG_PrePhysics`) reads the latest snapshot once per frame and every
  component reads that copy - never a live `World` accessor (same rule as `RgSimulation` today).
- Controls go the other way through `Session::set_control` (atomics), as now.
- Session construction stays on a worker (`run_terrain_init_worker` equivalent) with `StartupProgress` polled by the
  loading screen; cancel-then-join on teardown; never detach. Use `FRunnable` or `std::thread` - rg_core already
  uses `std::thread`, so the adapter only needs the join discipline.
- UE's Chaos physics is not used for the car, terrain or traffic; actors that only display sim state have physics
  and collision off.

## 10. Audio

Today: engine voice = physics_sim `vehicle_voice.cpp` (Godot-free, own render thread) + tyre PCM from
`tyre_sound_model.gd`, sent to the Windows spatial backend (`ps_audio` native output, `PsSpatialAudio` wrapper),
bypassing Godot's buses; Godot `AudioStreamPlayer3D` fallback.

UE plan:
- Keep the producers engine-neutral: `vehicle_voice` as is; move the tyre DSP to core (leak #8).
- Output path A (parity): call `ps_audio` native output from `RgAudioComponent` exactly as `PsSpatialAudio` does
  (positions relative to the active camera each frame).
- Output path B (UE-native): one `USynthComponent` per source (engine, four tyres), `OnGenerateAudio` pulls PCM from
  the producers; spatialisation and attenuation by UE (MetaSounds later), Microsoft Spatial Sound plugin for Windows
  Sonic/Atmos.
Recommendation: B, keeping A behind a setting for A/B listening (Q6). The horn's generated WAV becomes a
`USoundWave` or a synth source.

## 11. Shaders and materials

The owner dropped all shader work until after the port (2026-10-14): no quality-tier file exists. In UE the materials
are **rebuilt natively**; the Godot shaders are only a reference for what they compute:
- **Landscape / terrain material** (`game/shaders/terrain.gdshader`): albedo = the chunk's per-vertex colour
  (hypsometric ramp + paved/unpaved classes from `chunk_vertex_colors()`); object-space only. UE: an opaque material
  reading Vertex Color, later layered with textures by land class. Not UE Landscape (heights come from geo2map
  chunks, not a heightmap actor).
- **Road material** (`game/shaders/road.gdshader`): ribbon albedo/markings from uv/uv2/colour, plus the depth pull
  toward the camera so ribbons win against coarse ground (`depth_pull_per_m/near_m/max_m`, computed in view space).
  UE: World Position Offset along the camera vector, or a depth bias; to be measured against L1-L3 chunks.
- **Car paint**: today Godot StandardMaterial3D recolouring from garage paint/rim colours. UE: a car-paint master
  material (clear coat) with paint and rim colour parameters set from `RgGarage.get_working_colours` equivalents.
- **Scalability**: map graphics settings to UE scalability groups (view distance, shadows, post, foliage later) and
  `r.ScreenPercentage`; the terrain LOD distance (`world_config.lod.max_distance_m`) and the upload budget become a
  game-specific scalability CVar. No tier file is needed for the port.

## 12. Exceptions and RTTI

- rg_core throws in about 59 places (`session.cpp`, `world_terrain.cpp`, `vehicle_load.cpp`, `tyre_spec.cpp`,
  `limit_advisor.cpp`, ...), physics_sim and geo2map also throw. UE modules compile with exceptions **off** by
  default, and on MSVC that changes `std::exception`'s type (`_HAS_EXCEPTIONS=0` names it `stdext::exception`) - the
  same trap that crashed the Godot binding (R9, CLAUDE.md "Godot bindings for Session terrain").
- Containment rule (already the Godot rule): **no exception crosses the library boundary**. Every entry point the
  adapter calls that can throw has a non-throwing wrapper in rg_core that catches inside rg_core:
  `rg::make_session` exists; the rest are listed in the day-1 checklist (section 13.2).
- The libraries themselves build with `/EHsc`. The `RgRuntime` module sets `bEnableExceptions = true` as a second
  line of defence (it costs little and makes an unexpected throw a logged error instead of `std::terminate`); the
  game module keeps UE's default.
- RTTI: `core/src/world_terrain.cpp:457` uses `typeid(RoadSurface)` for the product registry, and `std::any` /
  type-erased joins in g2m use it too. The libraries keep RTTI on (`/GR`); UE modules default to `bUseRTTI = false`,
  which is fine as long as no RTTI-dependent code is instantiated in a UE translation unit (header-only templates that
  use `typeid`/`dynamic_cast` must not be called from `RgRuntime`; if one is, set `bUseRTTI = true` on `RgRuntime`
  only).

## 13. Milestones and day-1 checklist

### 13.1 Milestones

1. **M0 libs**: `ue-win64` preset + `tools/build_ue_libs.ps1`; an empty UE 5 project whose plugin links the libs and
   creates a flat `rg::Session`, steps it, logs `state_hash` equal to `hash_check`. (Proves CRT/exceptions/ABI.)
2. **M1 flat drive**: chassis box + wheels from the snapshot, chase camera, keyboard through `rg::Controls`,
   `RgFrame` conversion tests.
3. **M2 car art**: glTFRuntime car, wheel poses, steering; engine voice + tyre audio (path B).
4. **M3 real world**: terrain init worker + loading screen numbers, `TerrainViewStreamer` -> RMC sections with
   budget, road ribbons, decks, buildings; no rebase; drive the home-r1 route.
5. **M4 shell**: UMG screens from `RgShellSubsystem` view-models: boot, main menu, spawn picker, loading, pause,
   settings, credits; then controls screen, garage, car browser.
6. **M5 modes and cameras**: free cam, drone, on foot, bumper/cockpit/orbit/cinematic, cockpit clicking
   (`rg::CockpitModel`), traffic and truck visuals.
7. **M6 materials and scalability**, VR (OpenXR) last.
Each milestone ends with a UE automation test that mirrors the matching smoke test (`-Drive`, `-Shell`, `-Garage`,
`-Cameras`).

### 13.2 What must be true in rg_core before day 1

- [ ] Leaks #3 (control merge) and #4 (frame command routing) moved: otherwise M1 re-implements `main.gd` rules.
- [ ] Leak #19 (`chassis_local_motion`) moved, so steering assist input is computed without an engine basis.
- [ ] Leaks #6/#7 (building tessellation, tile schedule) moved, with a C++ triangulator, before M3.
- [ ] Leak #8 (tyre DSP) moved before M2.
- [ ] Leak #5 (`rg::CockpitModel`) moved before M5.
- [ ] Leaks #17/#18/#20 (gauge info, assist bits, drone labels) in core so `RgRuntime` stays conversion-only.
- [ ] Every throwing entry point the adapter uses has a non-throwing wrapper (make_session pattern): vehicle load
      (`rg::load_vehicle`), `WorldTerrain::open`, garage `prepare_drive`, `limit_advisor`, `garage_dyno`, truck/traffic
      planners (already caught inside Session), environment load.
- [ ] No public header used by the adapter needs `JPH_*` defines or includes Jolt; macro clashes (`ensure` in
      `ps/environment/weather.h`) handled by the include wrapper or renamed upstream (physics_sim request).
- [ ] A `ue-win64` CMake preset (/MD, no debug CRT, /EHsc, RTTI, C++20) and `cmake --install` rules for rg_core,
      ps_core, ps_audio, g2m_* and their public headers (today no target has install rules).
- [ ] File paths: every loader takes absolute paths (true today); the UE side resolves `data/` from
      `FPaths::ProjectDir()` and stages it as non-UFS.
- [ ] User data (`settings.json`, `controls.json`, garage, `last_drive.json`) written through rg_core with a user
      dir the adapter passes in (true for Settings/Controls/Garage; seat offsets and address cache still in Godot
      ConfigFiles - leaks #14 and #2's cache).

## 14. Open decisions for the owner

| # | question | recommendation |
|---|---|---|
| Q1 | Where does the UE project live: `racing_game/ue/` in this repo, or a new repo with racing_game as a submodule? | **Same repo, `ue/` folder**: one sha for data, core and both front ends; Godot stays buildable until UE reaches parity. |
| Q2 | Compiler for the UE libs: MSVC `cl` (UE's default toolchain) or clang-cl (this repo's presets)? | **MSVC cl** for the libraries UE links (`msvc-release` preset exists); keep clang-cl for the Godot/test builds. Fewer ABI surprises. |
| Q3 | Which UE 5 minor version? | **The latest 5.x release at port start**, pinned; RealtimeMesh and glTFRuntime must have a build for it. |
| Q4 | Terrain component: RealtimeMeshComponent (third-party, faster) or ProceduralMeshComponent (built in)? | **RealtimeMeshComponent**; PMC as fallback if the plugin lags behind the engine version. |
| Q5 | Car art: glTFRuntime at runtime or Interchange import? | **glTFRuntime** first (parity with today's runtime GLB loading, cargen output usable as is); Interchange later for shipping art. |
| Q6 | Audio output: keep the Windows spatial backend (`ps_audio`) or UE audio (USynthComponent + MetaSounds)? | **UE audio**, with `ps_audio` output kept behind a setting for A/B listening. |
| Q7 | Floating origin in UE? | **No rebase**; rely on LWC with per-section local origins; keep the `origin` parameter in `RgFrame` for a fallback. |
| Q8 | Input: keep `rg::Controls` as the binding authority (UE only provides raw device states) or move bindings to Enhanced Input mapping contexts generated from `data/controls`? | **Keep `rg::Controls`** (controls screen, saved `controls.json`, calibration and device memory stay identical); Enhanced Input only as the raw key/axis source. |
| Q9 | Does Godot keep running in parallel until UE parity, and do new features land in both? | **Yes until M4**; new gameplay goes to rg_core only, Godot presentation is frozen except fixes. |
| Q10 | VR in the first UE milestone set? | **No, last** (M6); OpenXR in UE is a separate risk. |
