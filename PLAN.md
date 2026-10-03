# Open-world racing game on real map data: implementation plan

- **Owner:** Sebastian.
- **Written:** 2026-09-25 by the planning session (Opus 5.5).
- **How:** read-only research against:
  - `S:\claude_code\physics_sim` at `be36545`. The other session's uncommitted drivetrain work was not read.
  - the knowledge vault: PHYS-003, -008, -009, -011, -021, -025, -026, -032, -035, -042 and TOOL-023.
  - the web: three research agents; their sources are listed under References.
- **Nothing was built, installed, downloaded or created.** Only this file was written.
- **Markers:**
  - **[unverified]**: not confirmed against code or a primary source.
  - **[estimate]**: my own arithmetic, to be replaced by a measurement at the milestone named next to it.

Three repositories:

| Repo | Role | Depends on |
|---|---|---|
| `S:\claude_code\physics_sim` (exists) | vehicle and rigid-body physics (`ps_core`) | Jolt, nlohmann/json |
| `S:\claude_code\geo2map_engine` (new) | geography to game map: library + map server. C++ namespace `g2m`, target prefix `g2m_` | third-party libs only; never physics_sim, never a game |
| `S:\claude_code\racing_game` (new) | the game: Godot project + engine-neutral game logic | pins both repos above |

A future farming simulator is a fourth repo that pins the same two libraries.

---

> **Split note.** This file holds the game-side and cross-repo parts (sections 0, 1, 9-15) of the joint plan written on 2026-09-25, including the milestone schedule for BOTH repos. The library and server sections (2-8) live in `S:\claude_code\geo2map_engine\PLAN.md`. Section numbers are kept identical in both files, so a reference such as "(3.3)" or "(6.2)" points there.

## 0. Key decisions (details in the sections named)

1. **geo2map_engine is a standalone engine-neutral C++20 library plus a map server built from the same code (sections 2 and 3).**
   - It has no dependency on physics_sim or on any game.
   - It exposes generic typed map data plus extension points; games add their meaning through those.
2. **Map data is served by a map server, with one protocol and three deployments (section 3):**
   - the owner's hosted default server, which the game ships pointing at;
   - a custom endpoint set in config;
   - an integrated server: the same server code in-process, or as a local sidecar, for offline play and development.
   - The protocol is HTTP GET on immutable, release-pinned tile URLs, with a manifest endpoint.
3. **Served data: baked (derived) generic layers by default. Raw source layers are also on the protocol, and every layer can be derived on the client as a fallback (section 3.3; owner decision D2).**
   - Baked tiles cost about the same bandwidth as raw data.
   - They save client CPU, are identical for every player by construction, and cache across players on a CDN.
   - Render meshes, vegetation instances and physics tiles are never served. The client builds them cheaply from the served layers.
4. **Geodata acquisition (section 5.5):**
   - Bulk downloads converted by our own offline tiler: HVBG DGM1, Geofabrik OSM extracts.
   - Range-read cloud-optimised files from public open-data buckets: Copernicus GLO-30, ESA WorldCover.
   - Never live queries against OSM services.
5. **Grid and frame: UTM (section 4).**
   - Tiles are keyed by (UTM zone, level, easting index, northing index).
   - The session frame is the UTM grid minus an integer-kilometre origin; z is orthometric height.
   - Hessen DGM1 is published natively on integer UTM32 metres, so the home region needs no resampling.
6. **Physics gets terrain and roads through the heightfield (section 8).**
   - Roads are carved into a 1 m height raster with per-cell surface ids.
   - Only bridges (later also tunnels and buildings) are streamed static mesh bodies.
   - The glue is `g2m_ps_bridge`, an opt-in target in the geo2map_engine repo. It is built only when the consumer already provides `ps_core`.
7. **physics_sim needs a few additive changes (section 9).**
   - Blocker R1: pool heightfields clamp heights to about ±200 m, and the home region spans roughly 90–880 m [unverified: typical Taunus/Main elevations].
8. **The game builds one GDExtension DLL (`rg_godot`, section 10).**
   - It contains `ps_core`, `g2m`, the bridge, and the game's own engine-neutral `rg_core`.
   - Both libraries are pinned as git submodules at explicit commits. Pinning a sibling path is not an option: it would build against another session's uncommitted work.
9. **Milestones interleave library (G) and game (R) steps (section 12).**
   - Everything up to real roads uses the integrated in-process server only.
   - The HTTP transport and sidecar follow once the layer set is stable. The hosted default server comes after that.

---

## 1. What exists in physics_sim, and what it means here (verified in code)

Paths below are relative to `S:\claude_code\physics_sim`.

- **`ps::terrain::ITerrainSource`** (`core/include/ps/terrain/terrain_source.h`) is a pure-virtual public header. It can be implemented outside `ps_core`.
  - `World::set_terrain_source` takes a `shared_ptr<const ITerrainSource>`. It may be called only once per `World` (`world.h`).
  - Contract:
    - fixed `tile_size_m()` and `samples_per_tile()`;
    - opaque 64-bit `TileKey`;
    - `key_for_world_xy`, `key_offset`, `tile_origin_xy`;
    - `fill_tile` is deterministic, thread-safe across keys and allocation-free: it writes into pre-sized vectors.
  - `samples_per_tile` must be **even** (asserted in `tile_manager.cpp`), so a 1 m grid gives 256 samples over a 255 m tile.
- **Height range is clamped (requires R1).**
  - `TileManager` poses each pool body at `TileSample::tile_origin_xy`, including z.
  - But every pool heightfield is built from a ±200 m placeholder (`kPlaceholderHeightRangeM`, `tile_manager.cpp`).
  - Jolt v5.6.0's `HeightFieldShape::SetHeights` clamps values outside the construction-time range (its header says so; see also vault TOOL-023).
  - The `TileSample` comment calls the origin's z "not meaningful".
  - `adapters/godot/src/fall_detector.h` reads `heights[]` as absolute.
  - Result: real terrain above 200 m is clamped today.
- **PHYS-021 terrain/static-feature suppression cannot help on real terrain.** It only probes vertices at height exactly 0.0 (`compute_suppression_holes`).
  - The design below does not need it: roads live in the heightfield, and bridges sit above it.
  - So the bridge sets `TerrainConfig::suppress_terrain_under_static_geometry = false`.
- **Static body churn.**
  - Creating or destroying a static body clears the whole suppression cache (`TileManager::on_static_geometry_changed`). With suppression off, that is harmless.
  - Jolt shape construction (`jolt_backend.cpp` `make_shape`) runs on the calling thread, so a large `MeshShape` built on the sim thread costs tick time. Fix: R2.
- **Tile-seam contact spike (requires R3).** PLAN.md P8 carries an open single-tick spike of about 159 m/s² when a body crosses a terrain tile seam (vault PHYS-021).
- **Not implemented, although PLAN.md section 6 names them:**
  - `IStaticStructureSource`, `IWorldShape` (only a comment in `core/src/articulation/gravity.h`), and a scenario georeference;
  - terrain render meshes in the Godot demo. It renders the test-feature `.glb` plus a grid shader; the procedural heightfield is collision-only (`terrain_visual.gd`).
- **Godot adapter.**
  - `adapters/godot/CMakeLists.txt` builds `libps_godot.dll` into `adapters/godot/demo/bin`, inside the physics repo. A game must not build that target.
  - These files are Godot-free (namespace `ps_godot`) and reusable: `sim_thread.h`, `triple_buffer.h`, `render_interp.h`, `origin_rebase.h`, `frame_convert_core.*`, `input_map_core.*`, `haptics_core.*`.
  - Worth reusing from the demo:
    - `tach_gauge.gd` + `gauge_logic.gd` (the math is already engine-neutral);
    - `chase_cam.gd`, `free_cam.gd`, `hud.gd`, `vehicle_visual.gd`;
    - the PHYS-008 ordering rule: cameras at `process_priority = -1000`, and only the current camera rebases.
- **Vehicle files.**
  - `data/vehicles/car_sedan.json` (`physics_sim.vehicle/2`) refers to `engines/`, `gearboxes/` and `tyres/` files by path relative to the vehicle file (`vehicle_io.cpp`). A `ref` component may contain only `id`/`type`/`ref`.
  - Chassis mass, box and inertia are **not** in the vehicle file. `main.gd` hard-codes `CHASSIS_MASS` and `CHASSIS_HALF_EXTENTS`.
- **`data/models/car_hyper`** has a `.glb` and a `rig.json`, but no physics data. `data/models` and `tools/assetgen` belong to another session.
  - Geometry: wheelbase 2.70 m; track 1.70/1.65 m.
  - Tyres: 265/35 R20 front (radius 0.347 m), 325/30 R21 rear (0.364 m).
  - Suspension travel −0.06..+0.05 m; max steer 0.5585 rad.
  - Nodes: wing and flaps, `socket_driver_eye`, steering wheel, seats.
- **`SurfaceTable`** assigns ids by alphabetical name. Consumers must resolve surfaces by name, never by hard-coded id.
- **Build constraints.**
  - Static CRT, set before `project()`.
  - `/arch:AVX2` on every TU near Jolt headers (vault PHYS-003, the `DVec3Arg` ABI).
  - `CMAKE_SOURCE_DIR` is used only under `tests/`, so `add_subdirectory(... EXCLUDE_FROM_ALL)` with `PS_BUILD_TESTS=OFF` should work [unverified until R0 builds it].
  - The repo has **no git remote**.
  - godot-cpp is pinned to branch 4.5 and loads in the installed Godot 4.7.2 (vault PHYS-011).

---

## 9. Requests to physics_sim (additive; for the physics session via owner-approved briefs)

| # | Change | Why | Needed by |
|---|---|---|---|
| R1 | Per-tile vertical datum: heights relative to `TileSample::tile_origin_xy.z`, documented. Height consumers (`fall_detector`, any snapshot user) add it. Test with a tile at +800 m | Jolt clamps to the ±200 m placeholder range (TOOL-023) | R2 milestone (real terrain), **blocking** |
| R2 | Prebuilt static shapes: thread-safe `IRigidBackend::create_shape(ShapeDesc) -> ShapeHandle`, and `create_body` from a handle | no mesh BVH build inside the tick | bridges (R3), buildings (R7) |
| R3 | Root-cause the terrain tile-seam contact spike (~159 m/s², PLAN.md P8) | a car crosses a seam every 2.5–10 s | R2 acceptance |
| R4 | Package the Godot-free adapter cores as an engine-neutral static library target (e.g. `ps_frontend`), buildable without godot-cpp | game and UE5 reuse without compiling adapter internals | R0 can compile the files by path meanwhile |
| R5 | Chassis mass/inertia/CoM and collision box in the vehicle file | configurator and hypercar need them as data | R6/R8 (interim: the game catalog carries them) |
| R6 | Optional: `load_vehicle_json_from_string(json, base_dir)` | configurator without temp files | R6 (interim: materialise into the cache, 11.6) |
| R7 | Hypercar data set + validation (11.7) | new vehicle | R8 |
| R8 | Later: `IWorldShape` curved WGS84 | real horizon on long drives | optional |

The game never edits physics_sim.

---

## 10. racing_game repository and build

### 10.1 Layout (`S:\claude_code\racing_game`)

```
racing_game/
  CMakeLists.txt, CMakePresets.json   debug, release, relwithdebinfo, asan (clang-cl), msvc-release; linux-release (rg_core + tests only)
  CLAUDE.md, README.md, PLAN.md       PLAN.md = this plan, adopted and maintained
  external/physics_sim/               git submodule, pinned commit
  external/geo2map_engine/            git submodule, pinned commit
  core/                               rg_core: session orchestration (World + g2m client/server + bridge), game state machine,
                                      vehicle catalog + configurator, navigation guidance, camera rig math, HUD/minimap
                                      view-models, settings (incl. map server endpoint), saves
  godot_ext/                          rg_godot GDExtension (thin): session node, g2m upload nodes, input, haptics, audio hooks
  game/                               Godot project: project.godot, scenes/, scripts/, ui/, shaders/, materials/, textures/, bin/
  data/
    vehicles/catalog.json, setups/    catalog entries + setup overlays
    surfaces/                         racing surfaces + g2m class mapping
    world/                            regions.json (home region, spawn presets), g2m client/server configs, style tables
  tests/                              unit (rg_core), integration (headless: physics + g2m integrated server + bridge), golden routes
  tools/setup_dev_env.ps1             verifies the toolchain; fetches approved dev assets/data on request (-FetchData)
  tools/ci.ps1                        -Affected / default / -Full, like physics_sim
  tools/run.ps1, run.cmd              incremental extension build + launch Godot (like drive.ps1)
  tools/smoke_test.ps1                headless boot + autopilot drive on the fixture region + streaming check
```

### 10.2 Pinning and build

- **Submodules** with local file URLs (`S:/claude_code/physics_sim`, `S:/claude_code/geo2map_engine`).
  - git ≥ 2.38 needs `-c protocol.file.allow=always` for local-path submodules [unverified for git 2.54 on Windows; check in R0].
  - Switch the URL when a remote exists.
- **Bumping a pin** is an explicit commit ("bump physics_sim to <sha>: <why>") followed by game CI.
- **Rejected alternatives:**
  - FetchContent: it works, but every preset clones again and the pin hides in CMake.
  - Sibling-path `add_subdirectory`: unpinned, and it sees the other session's uncommitted work.
- **CMake:**
  - `CMAKE_MSVC_RUNTIME_LIBRARY` exactly as physics_sim, before `project()`.
  - `PS_BUILD_TESTS=OFF`, `PS_BUILD_GODOT_ADAPTER=OFF`.
  - `add_subdirectory(external/physics_sim EXCLUDE_FROM_ALL)`, then `add_subdirectory(external/geo2map_engine EXCLUDE_FROM_ALL)`. The latter sees `ps_core` and builds `g2m_ps_bridge`.
  - The game fetches godot-cpp branch 4.5 itself (same pin as physics_sim).
  - physics_sim's `tools/ps_run` and `sweep` are always built by its CMake; with `EXCLUDE_FROM_ALL` they are skipped unless requested [verify in R0].
- **One GDExtension:** `game/rg_godot.gdextension` points at `res://bin/rg_godot.dll`, with `compatibility_minimum = "4.5"`.
  - The DLL links `ps_core`, the `g2m` libraries, `g2m_ps_bridge`, `g2m_godot` and `rg_core`.
  - Reasons: one `ps::World` shared by physics, bridge and rendering, and one copy of static state (Jolt registration, allocator hooks).
  - The build writes only into the game repo.
  - PHYS-011's first-import quirk applies: a fresh checkout needs one `--editor --headless --quit-after 1` run before headless tests.

### 10.3 CI

**Game (`racing_game/tools/ci.ps1`):**
- `-Affected`: release leg, affected topics.
- **Default (< 5 min):**
  - clang-cl `release` + `debug` and an `msvc-release` build;
  - `rg_core` unit tests and integration tests;
  - the headless Godot smoke test (fixture region via the integrated server, 60 s autopilot, the fall detector must not fire);
  - WSL `linux-release` for `rg_core` + integration.
- **`-Full`:** adds `asan`, a cross-platform map-determinism check, and the streaming benchmark.
- **Map determinism:**
  - Derive a fixed set of fixture tiles and compare with committed hashes on every leg.
  - A 60 s headless drive with a scripted input must give identical physics + tile hashes at 1, 4 and 10 workers, and on Windows vs Linux.
- **Grep gates:** no Godot include in `core/`, no platform transcendentals in `core/`.

**geo2map_engine (`tools/ci.ps1`, same structure as physics_sim's minus Godot):**
- unit tests;
- golden tiles per layer (Windows clang-cl, cl, Linux clang, compared);
- protocol conformance (in-process vs HTTP transport must return byte-identical tiles; ETag/immutability headers; 404/503/410 semantics);
- fuzzers (short in the default run, long in `-Full`);
- benches (derivation CPU per km², bytes per km²);
- the bridge built against a pinned physics_sim commit.

---

## 11. Game front end

### 11.1 Engine-neutral vs Godot

- **`rg_core` (C++):**
  - session orchestration: World, g2m client, integrated server, bridge;
  - game state machine; vehicle catalog and setups;
  - navigation: route requests, guidance (next manoeuvre, distance, lane hints);
  - camera rig math;
  - settings (incl. `map_server` endpoint, cache size, offline toggle) and saves;
  - HUD and minimap view-models.
- **Godot:** rendering, UI layout, input devices, audio output.
- **GDScript** only binds view-models to controls.

### 11.2 Flow

```
Boot: splash + data attribution, load settings, open caches, contact map server (manifest; offline if unreachable)
  -> Main menu: Free roam | (later: Events) | Garage | Settings (map server, cache, pin region, graphics, input) | Credits | Quit
     Garage -> Vehicle select (turntable of the .glb, stats from data) -> Configurator
     Free roam -> Spawn picker (home-region presets, pick on map) -> Loading (per-layer progress + attribution)
  -> Drive: HUD, minimap, cameras
     Pause: Resume | Map & route | Garage (respawn) | Settings | Main menu
     Map: pan/zoom, set destination (snapped to nearest routable edge), route + ETA, dev teleport
```

### 11.3 Cameras

- **Chase:** existing `chase_cam.gd`.
- **Bumper/hood:** offset from the rig's bounds.
- **Cockpit:** `socket_driver_eye` (car_sedan and car_hyper both have it; car_hyper has a steering-wheel node).
- **Orbit.**
- **Free:** existing `free_cam.gd`.
- **Cinematic:** roadside cameras placed along the current road edge ahead, using the road graph.
- All follow PHYS-008: `process_priority = -1000`, and only the active camera rebases.

### 11.4 HUD, minimap, full map

- **HUD:** `tach_gauge.gd` + `gauge_logic.gd` bottom right, speed, gear, assist lamps. Next-manoeuvre arrow + distance. Attribution line under the minimap.
- **Minimap:**
  - a `SubViewport` drawing client-side map geometry;
  - 0.5–2 km radius with speed-dependent zoom; heading-up;
  - road width by class, route highlight, player arrow;
  - rendered at a fixed resolution in a round mask.
- **Full map:** the same geometry at map zoom levels, drawn as GPU vectors (crisp at 4K).
- **Routing:**
  - A* over the graph with a travel-time cost (maxspeed, class defaults, turn penalties, restrictions);
  - heuristic = straight-line distance / max network speed, which is admissible;
  - bidirectional search and the coarse hierarchy for long routes; contraction hierarchies later if needed.
- **Traffic (later) needs from the graph:** lane connectivity (`turn:lanes`), signals and stop points, speed limits, restrictions, junction topology. All of these are already in `roads.graph`.

### 11.5 Rendering and dev textures

- **Terrain:** a splat shader with 4 layers from land-class weights, triplanar on steep slopes.
- **Roads:** asphalt/sett/concrete/gravel materials, plus a marking decal mesh (procedural dash patterns).
- **Buildings:** a few tiling facade and roof materials, UVs in metres, window grid by levels.
- **Vegetation:** instanced low-poly trees and bushes via `MultiMeshInstance3D`, with an impostor band.
- **Atmosphere:** sky, sun, fog.
- **Our own chunk streaming, not Terrain3D.** Terrain3D is limited to 65.5 km, has its own region model, and double precision is experimental (research).
- **Upscaling:** FSR for PLAN.md's 4K/240 target.
- **Dev textures:** CC0 only, from ambientCG and Poly Haven (both CC0 1.0).
  - Proposed sets: asphalt, worn asphalt, sett/cobblestone, concrete, gravel, dirt, grass, forest floor, rock, farmland soil, plaster facade, brick facade, roof tiles.
  - 13 sets at 2K.
  - Fetched by the setup script from a pinned, approved list (decision D14).

### 11.6 Vehicle selector and configurator

- **`data/vehicles/catalog.json`**, per entry:
  - the physics vehicle file (inside the pinned physics_sim tree);
  - the model (`external/physics_sim/data/models/<id>`, read-only);
  - display data;
  - chassis mass/box/inertia (until R5);
  - allowed setup ranges.
- **Setup overlays** (`racing_game.vehicle_setup/1`): a whitelist of JSON-pointer paths with ranges, applied as a JSON merge patch to the vehicle file and its referenced engine, gearbox and tyre files. v1 options:
  - final drive ±20 %; gear ratios ±20 % (kept monotonic);
  - brake bias (same total) and brake force;
  - springs, dampers and ARBs ±30 %;
  - tyre choice from tyre files matching the wheel size;
  - assists via the existing `assist.*` channels (auto clutch, auto blip, auto shift); ABS/TC/ESC once physics has them;
  - paint and rim colour (visual only).
- **Validation:**
  - The merged files are materialised into the cache, with refs rewritten to absolute paths (`vehicle_io.cpp` resolves `ref` relative to the vehicle file).
  - They are then loaded with `ps::io::load_vehicle_json`.
  - A loader error is shown, and the setup cannot be saved.
  - R6 later removes the temp files.

### 11.7 Hypercar physics data (decision D5)

**Recommendation:** author it in physics_sim (`data/vehicles/car_hyper.json` + `engines/`, `gearboxes/`, `tyres/`) via a brief to the physics session.
- The validation tests belong there.
- PLAN.md section 8 makes physics_sim the home of vehicle data.
- The game adds only a catalog entry.

**Data plan** (PLAN.md section 8: public data only, a source per field; the model is not a real car, so figures are labelled class-typical):
- **Geometry:** from `car_hyper.rig.json`.
- **Mass:** ~1,400 kg, 42/58 front/rear.
- **Engine:** twin-turbo V8, ~600 kW / ~800 N·m; curve shape from published dyno curves of comparable engines (±10 % check, PHYS-032).
- **Gearbox:** 7-speed sequential manual with auto clutch until physics I10 brings DCTs.
- **Differential:** RWD open diff until an LSD exists.
- **Tyres:** MF5.2-subset JSON fitted to published generic high-performance data; never a proprietary `.tir`.
- **Aero:** CdA ~0.75 m². **No downforce until physics P5.** The wing and flaps are visual only until then, and the game says so.
- **Acceptance (in physics_sim):**
  - loads; static corner weights;
  - top speed and 0–100 km/h within the P4 test tolerances against a point-mass reference;
  - no chassis-box/wheel-cast overlap (the `build_vehicle_rig` constraint).

---

## 12. Milestones

Rules:
- Each milestone ends with its acceptance tests green in its repo's CI, the CLAUDE.md map updated, and vault tickets written.
- Physics prerequisites are in brackets.
- **Model:** **Opus** = open design, numerics, determinism, concurrency. **Sonnet** = implementation from a precise brief. The main session always diagnoses, briefs and verifies.
- **Order:** G0 ∥ R0 → G1 → G2 → R2 → G3 → R3 → G4 → R4 → R5 → R6 → G5/R7 → R8 → G6 → R9 → G7 → G8/R10 → traffic → radio.
- Everything up to R8 runs on the integrated in-process server. The protocol exists from G1 as in-process request/response types; G6 gives it its wire form. The hosted server (G7) comes once the game is playable offline.

**G0: library skeleton, determinism, geodesy.** Model: Opus for math and geodesy; Sonnet for skeleton and CI.
- Scope: repo, presets, CI (Windows ×3 + WSL), deterministic math port, TM/UTM, tile keys, raster types, hash and golden infrastructure, grep gate.
- Acceptance:
  - TM forward/inverse vs GeographicLib reference values at 10,000 points (incl. extended zone ±9°): < 1 µm;
  - round trip ≤ 5 nm (ruling 2026-09-25);
  - identical hashes Windows/Linux;
  - libm grep clean.

**R0: game skeleton.** Model: Sonnet. Prerequisite: none (R4 optional).
- Scope: repo, submodules, single `rg_godot` DLL with `ps_core`, drive car_sedan on a flat box in Godot, chase cam, tach HUD, `run.ps1`, smoke test.
- Acceptance:
  - a clean checkout builds via `setup_dev_env.ps1 -CheckOnly` + preset;
  - headless smoke test passes;
  - identical physics hash to physics_sim's `ps_run` for the same scenario.

**G1: server core, raw layers, tiler, cache.** Model: Opus for resolver/release/protocol-types design; Sonnet for the TIFF subset reader, PBF import, SQLite stores.
- Scope:
  - `g2m::Server` with router and resolver, source store, release manifest (in-process);
  - `g2m.src.elev.dgm1`, `g2m.src.osm`, `g2m.src.landcover`;
  - `g2m_tiler` (LZW/Deflate GeoTIFF subset, COG range reads, libosmium PBF);
  - client cache;
  - offline mode;
  - the home-region import (5.5).
- Acceptance:
  - tiler output on the fixture is byte-identical Windows/Linux;
  - raw tiles round-trip the source values exactly (quantisation 1/256 m);
  - LRU/pinning/corruption tests pass;
  - offline returns cached or `degraded`, never a network call.

**G2: base terrain + bridge.** Model: Opus. Prerequisite: [R1].
- Scope: `g2m.elev.base`, `g2m.terrain.height` without roads, `terrain.class` from land cover, client streamer, terrain render chunks, `g2m_ps_bridge` terrain source.
- Acceptance:
  - heights equal DGM1 within 1/256 m at every fixture sample;
  - chunk borders are bit-identical between neighbours;
  - `fill_tile` is allocation-free (debug_alloc) and worker-count invariant;
  - cost per 256² fill is measured.

**R2: drive real terrain.** Model: Sonnet (Opus for seam diagnosis). Prerequisites: [R1, R3].
- Scope: home region via the integrated server (DGM1 imported); terrain rendering with LOD and floating origin; streaming overlay; the clock freezes on a missing tile.
- Acceptance:
  - a 30 km scripted drive across MTK/HTK: no fall report;
  - no seam spike above the R3 bound;
  - main-thread upload ≤ 1 ms/frame p99;
  - sim holds 240 Hz.

**G3: roads.** Model: Opus.
- Scope: `roads.graph`, `roads.geom`, carving, bridges, surface kinds.
- Acceptance:
  - on the fixture, road-surface cross-slope within spec ±0.2 %;
  - max grade within class limits;
  - junction height continuity < 1 cm between all incoming edges;
  - no step > 5 mm between carved road and blend at any sample;
  - bridge deck/approach step ≤ 2 cm;
  - contact/`Fz` spike where heightfield meets a static bridge or structure mesh stays within the R3 seam bound. physics_sim reported an open 162 m/s² heightfield-vs-mesh spike on test_ground (x = −500), not covered by R3 at 5c648a5. Report measurements back to the physics session;
  - golden hashes Windows = Linux;
  - bytes/km² and CPU-s/km² measured (replacing 3.8's estimates).

G3 status (2026-10-02): roads.graph and deterministic horizontal/vertical
construction are implemented. Junctions use shared sloping DEM planes;
degree-two bridge/tag boundaries use bounded approach-based deck estimates.
The marked A66 bridge is accepted by the profile builder; this is estimated
geometry, and real bridge collision acceptance remains pending. The previous
grade-policy question came from treating an underpass DEM drop as a deck slope
and is withdrawn. G3-C remains in progress: complete profile acceptance.
roads.geom codec, staged dependencies and opt-in server/bake serving are
implemented. Numeric incline supplies a bounded soft slope target while shared
boundaries stay fixed. G3-D/E carving, junction surfaces,
ribbons, markings and bridge bodies, and G3-F/R3 route acceptance remain open.
Physics fixture fix bd41ac4 is integrated.

OSM dashboard extension: directional explicit speed limits are carried on
resident road metadata and exposed in car snapshots and Godot. Numeric,
unrestricted and unknown values remain distinct; conditional values are marked.
Ground matching excludes bridge decks until 3D road matching exists. Main car
is the supplied physics_sim car_hyper, including its authored hypercar powertrain, tyres and suspension.

**R3: roads in game.** Model: Sonnet. Prerequisite: [R2 for bridges].
- Scope: road ribbons + markings, surfaces into physics by name, bridge bodies.
- Acceptance:
  - drive a 20-junction scripted route: wheel `Fz` spikes within the R3 bound;
  - the surface id under each wheel matches `SurfaceKind`;
  - bridges driveable both ways.

R3 status (2026-10-01): terrain road colours and per-cell surface grip are
implemented (G2.5a-grip R-b/R-c). The shipped world maps paved/unpaved/off-road
to asphalt/dirt/grass. Real home-r1 proof recorded 2103/2103 route samples
asphalt, 84/84 forest-control samples grass, plus Session wheel checks at 7
route and 5 off-road positions. Drive smoke checks asphalt at spawn. See
CLAUDE.md and tests/unit/test_route_grip.cpp. Road ribbons/markings, bridge
bodies and bidirectional bridge driving, and the 20-junction wheel-force
acceptance remain outstanding; R3 is not complete.

**G4: routing and map geometry.** Model: Opus for graph/routing design; Sonnet for map geometry.
- Scope: A* + hierarchy, restrictions, guidance data, client map geometry.
- Acceptance:
  - routes on the fixture equal a brute-force Dijkstra;
  - restrictions honoured in unit cases;
  - route requests under 10 ms within 30 km [target].

**R4: minimap, full map, navigation.** Model: Sonnet.
- Acceptance: minimap cost < 0.3 ms per frame; route guidance through 10 manoeuvres on a scripted drive; attribution always visible.

**R5: shell.** Model: Sonnet.
- Scope: menus, spawn picker, cameras (all six), loading screens, settings (incl. map server endpoint, cache, pin region), credits with the full attribution list.
- Acceptance: headless UI flow test; camera switch with no rebase jump (PHYS-008 test).

**R6: vehicle select and configurator.** Model: Sonnet. Prerequisite: [R5 or the catalog interim].
- Acceptance:
  - every whitelisted change round-trips through `load_vehicle_json`;
  - an out-of-range or invalid overlay is rejected with the loader message;
  - the setup persists.

**G5 + R7: land cover, buildings, water, vegetation, building collision.** Model: Sonnet (Opus for building-collision streaming). Prerequisite: [R2].
- Acceptance:
  - building bases sit on terrain (max gap < 5 cm);
  - vegetation is deterministic per seed;
  - building collision add/remove is deterministic across worker counts;
  - frame time within the R9 budget in Frankfurt-Höchst.

**R8: hypercar.** Model: Sonnet for the catalog; the physics session does the data. Prerequisite: [R7].
- Acceptance: selectable and drivable; the physics validation tests pass in physics_sim.

**G6: wire protocol + sidecar.** Model: Opus for the protocol spec; Sonnet for HTTP and conformance.
- Scope: `g2m_http` (server + libcurl fetcher), `g2m_server` binary, Godot `IFetcher`, protocol v1 docs, conformance suite, packs, `/hashes`; the game setting for a custom endpoint.
- Acceptance:
  - in-process vs HTTP give byte-identical tiles;
  - conformance suite passes against the sidecar;
  - fuzzed responses never crash the client;
  - the game runs against a sidecar with the same physics hash as in-process.

**R9: performance.** Model: Opus for analysis; Sonnet for fixes.
- Scope: 4K/240 target via FSR on the 5900X, PLAN.md adapter budget (< 1 ms main-thread own work), streaming at hypercar speed.
- Acceptance:
  - measured frame-time and tick-rate report;
  - no hitch > 1 frame from streaming at 100 m/s.

**G7: hosted default server.** Model: Opus for ops design; Sonnet for scripts. Prerequisites: owner decisions D3/D4/D7/D15.
- Scope:
  - deployment in the owner's estate, CDN in front;
  - pre-baked Hessen (then Germany) packs; the on-demand derivation queue;
  - API keys and rate limits; release pipeline and retention;
  - ODbL publication of the packs; privacy notice.
- Acceptance:
  - load test (N simulated drivers) meets the latency target;
  - CDN hit rate ≥ 95 % in baked regions;
  - measured egress per player-hour;
  - the game ships pointing at it; offline fallback verified.

**G8/R10: worldwide.** Model: Opus.
- Scope: GLO-30 (+ Mapterhorn mirror, decision D13) with DSM correction, zone crossing, other German states' DGM1, optionally Overture.
- Acceptance:
  - driving across the UTM 32/33 boundary without a seam;
  - a sample of international regions derives and is drivable.

**Later: traffic.** Model: Opus.
- Lane-level graph (lane connectivity from `turn:lanes`), signals, AI drivers on physics vehicles.
- Needs PLAN.md P9's 30/300-vehicle budgets.

**Later: in-car radio (owner request 2026-09-26; do not start before traffic).** Model: Opus for the mood classifier and source abstraction; Sonnet for the players and UI.
- Several radio stations. Each is either a live internet radio stream or a music-service API source. The owner's own is music.ksfhost.de; players bring their own compatible service (endpoint + credentials in settings). Nothing is bundled or hosted for players.
- The music follows driving style and game state. Examples of distinct moods: a slow night drive; 300 km/h on the Autobahn; drifting corner after corner; city streets at normal speed; city streets at insane speed. The inputs are telemetry the game already has (speed, slip angles, yaw rate, road class from G3, urban/rural from G5, time of day), classified engine-neutrally in `rg_core`. The Godot adapter only plays audio.
- Open for design: how a mood maps to a station, track or playlist query per source type; whether live streams can follow a mood at all (e.g. per-station mood tags) or only API sources; crossfade and hysteresis so short events don't flip the music; licensing and attribution of third-party streams.

---

## 13. Downloads and installs for owner approval

Nothing below has been downloaded or installed. The setup scripts will install or fetch only items on an approved list. Sizes are approximate; [unverified] where no primary figure was found.

**Already installed (verified this session): no action.** VS 2022 Build Tools (clang-cl, cl), CMake, Ninja, Git 2.54, Python (`py`), `curl.exe`, Godot 4.7.2 (winget), WSL with clang/lld.

**Source dependencies (FetchContent at configure time; pinned tags):**

| Item | Licence | Approx. size | Used by |
|---|---|---|---|
| godot-cpp branch 4.5 | MIT | ~30 MB source [unverified] | game extension (physics_sim already fetches it) |
| SQLite amalgamation 3.x | public domain | ~3 MB zip [unverified] | client cache, server stores |
| zstd | BSD (dual BSD/GPLv2; use BSD) | ~2.5 MB [unverified] | tile compression |
| libdeflate | MIT | ~0.5 MB [unverified] | DEFLATE in TIFF/COG |
| Clipper2 | BSL-1.0 | ~1 MB [unverified] | road offsets, junctions |
| earcut.hpp | ISC | < 0.1 MB | polygon triangulation |
| cpp-httplib | MIT | ~0.5 MB, one header | HTTP server (sidecar/hosted) |
| libcurl (Schannel on Windows, OpenSSL on Linux) | curl (MIT-like) | ~4 MB source [unverified] | tools, server upstream fetches (not in the game) |
| libosmium + protozero | BSL-1.0 / BSD-2 | ~2 MB [unverified] | `g2m_tiler` only |
| zlib-ng | zlib | ~1 MB | libosmium PBF (tiler only) |
| GeographicLib (test data generation only) | MIT | ~2 MB [unverified] | G0 reference values |
| later, only if needed: libwebp (decode) | BSD-3 | ~4 MB [unverified] | Mapterhorn WebP (G8) |
| later: vtzero + protozero | BSD-2 | < 1 MB | MVT/PMTiles basemaps, if ever |

**Datasets (dev machine, integrated server; 5.4/5.5):**

| Item | Licence | Approx. size | How |
|---|---|---|---|
| Hessen DGM1: Main-Taunus-Kreis, Hochtaunuskreis, Frankfurt am Main | dl-de/zero-2-0 | 1.1–2.6 GB [estimate] | manual download from gds.hessen.de (JS portal, no automatable URL found) |
| Geofabrik `hessen-latest.osm.pbf` | ODbL 1.0 | 329 MB | setup script, once (Geofabrik fair use) |
| Copernicus GLO-30 N50E008 (+ N49E008 if needed) | Copernicus DEM licence | 25–40 MB each [unverified] | range reads (or one download) from AWS open data |
| ESA WorldCover v200 N48E006 map | CC BY 4.0 | ~47 MB average [estimate] | range reads (or one download) from AWS open data |
| CC0 dev textures (13 sets, 2K) from ambientCG / Poly Haven | CC0 1.0 | 150–400 MB [estimate] | setup script, listed per asset |
| Hessen LoD2 for the region (optional, G5+) | dl-de/zero-2-0 | unknown [unverified] | manual |

**Later (hosted server, G7/G8; each approved separately at that time):**
- Germany DGM1, all states: 0.4–1 TB as delivered [estimate].
- Geofabrik Germany extract (~4 GB [unverified]) or the OSM planet (~80–90 GB [unverified]).
- WorldCover global: 124 GB.
- Mapterhorn planet PMTiles z0–12: ~120 GB.
- GLO-30 global mirror (size [unverified]).
- Accounts/services: object storage + CDN, compute VM(s), domain + TLS. Choice per D3.

---

## 14. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Physics blockers R1 (height clamp) and R3 (seam spike) not scheduled by the physics session | R2 milestone blocked | brief them now; G0–G2 proceed in parallel; the bridge tests with an offset-free fixture until R1 lands |
| Cross-compiler determinism of derivation (client vs server) | multiplayer desyncs; wasted CDN cache | 2.4 rules, quantised outputs, golden hashes on every compiler, multiplayer uses baked/verified tiles only (D12) |
| OSM data quality (missing lanes/width, bow-tie junctions, bad layer tags) | ugly or undrivable junctions | consolidation, tagged defaults flagged "inferred", visual QA tool in `g2m_cli`, per-edge overrides as a data layer later |
| GLO-30 is a DSM (forest plateaus ~13 m median error) | worldwide terrain wrong under forests | canopy inpainting with WorldCover; road profiles robust-fitted; a national DTM wherever one exists; FABDEM only if its licence allows |
| ODbL obligations misunderstood | legal exposure for the hosted server | publish packs under ODbL + reproducible method; get a legal check before G7 |
| Volunteer/third-party services (Mapterhorn, Geofabrik) change or rate-limit | pipeline breaks | mirror what we use on the server; clients never contact them |
| HVBG portal is manual | first-region setup friction | one-off documented manual step; other states (NRW) automatable |
| Streaming cost at 240 Hz (256² `SetHeights`, uploads) unmeasured | hitches | measure in G2; tile fills budgeted per tick; 511 m tiles as a fallback option |
| Godot single precision and main-thread upload | jitter, hitches | floating origin (exists), upload budget, no Terrain3D |
| Scope ("worldwide from day one") | slow first playable | worldwide architecture, but home-region data first; G8 later |
| Hosted server cost or abuse | bills, outages | CDN-static packs, keys + quotas for derivation, load shedding to client-side derivation |
| GDPR for server logs | compliance | privacy notice, 7-day retention, no tracking without consent |
| Physics repo churn (another session, uncommitted work) | broken builds on pin bumps | explicit pins; bump only green physics commits; the bridge compiled in g2m CI |
| Jolt internal allocation (`QuadTree::UpdatePrepare`) and D8 gaps | hitches with many streamed bodies | tracked in physics P9; the game limits bodies per tick |
| godot-cpp 4.5 vs Godot 4.7 runtime | API mismatch later | pin both; move together deliberately |

---

## 15. Open decisions for the owner (each with a recommendation)

| # | Question | Recommendation |
|---|---|---|
| D1 | P11 reconciliation | P11 becomes "physics_sim consumes geo2map_engine via `g2m_ps_bridge`"; physics keeps `ITerrainSource`, R1, R2 and `IWorldShape`; the offline bbox bake survives only as `g2m_tiler` feeding the server (2.7) |
| D2 | Baked vs raw served data | baked generic layers by default; raw layers also served; every built-in layer derivable locally; game layers derived client-side, optionally as server plugins (3.3). Cheaper alternative: baked-only for remote servers |
| D3 | Hosting in "the owner's estate": what exists? | object storage + CDN (Cloudflare R2 for zero egress) for packs, plus one small VM for on-demand derivation behind a reverse proxy; start with Hessen pre-baked |
| D4 | Auth for the public default server | anonymous CDN reads; per-build revocable API key + quotas only for on-demand derivation |
| D5 | Where the hypercar data lives | physics_sim `data/` via a brief to the physics session (11.7) |
| D6 | Vegetation collision | none in v1 (trees are decoration); revisit with a "forest = rough surface" class |
| D7 | ODbL compliance form; open-source geo2map_engine? | publish OSM-derived packs per release under ODbL; open-sourcing the library (e.g. MIT) is the simplest "method" disclosure and helps modders run custom endpoints; legal check before G7 |
| D8 | Frame: UTM vs a local ENU or ECEF frame | UTM (section 4): native to German data, stable keys, flat physics; curved shape later via R8 |
| D9 | Pinning via submodules | yes, local file URLs; add a remote when one exists |
| D10 | Game repo name and Godot version policy | `racing_game` as working name; godot-cpp 4.5 API on Godot 4.7.x runtime as physics_sim does |
| D11 | Release cadence of the hosted dataset | quarterly OSM snapshot releases; keep 3 live |
| D12 | Multiplayer tile policy | baked or hash-verified tiles only |
| D13 | Worldwide DEM source for the server | mirror a Mapterhorn extract (already a merged global mosaic incl. all German DGM1) plus GLO-30 as the base; never hotlink |
| D14 | Dev textures in git (LFS) or fetched | fetched by the setup script from a pinned, approved list; the game repo stays small |
| D15 | Server privacy/telemetry | IP logs 7 days, no analytics without consent; privacy notice before G7 |
| D16 | Which physics requests to brief now | R1 and R3 immediately (they block R2); R2 and R4 soon; R5–R7 before R6/R8 |
| D17 | Custom endpoint trust | allowed via settings/CLI with a one-time warning; all served bytes treated as untrusted (fuzzed decoders, size limits) |


### 15.1 Owner rulings (2026-09-25)

| # | Ruling | Consequence for this plan |
|---|---|---|
| D2 | Priority/fallback order: finished (baked) layers, then raw layers, then self-fetched from sources | As recommended. All three paths share the derivation code and must give identical tile hashes (2.4). |
| D3 | Self-hosted on the owner's estate. During development the server runs on this PC. Private use (owner and friends); not published or sold | G7 shrinks to one machine on the estate: no CDN, no Cloudflare R2, no quota system. D4 becomes a single shared key or LAN/VPN-only access. The protocol stays CDN-cacheable so this can change later. G7's load-test and CDN-hit-rate acceptance items are dropped; egress per player-hour is still measured. |
| D5 | The hypercar lives only in racing_game (`racing_game/data/vehicles`), not in physics_sim. It can be copied over later | R7 is no longer a physics request. R8 becomes game-side: data plus validation tests in racing_game's own test suite, run against the pinned `ps_core` (same public `load_vehicle_json` API). |
| D7 | geo2map_engine may be open source: MIT | The repo is created with an MIT LICENSE. ODbL attribution stays wherever OSM-derived data is shown. Share-alike for a publicly served database matters only if the server is ever opened beyond private use; get a legal check before then. |
| downloads | §13's source dependencies and dev datasets approved | Downloads go to `S:\claude_code\geo2map_data` (outside every repo; `G2M_DATA_DIR`). The later hosted-server items in §13 still need their own approval. |
| D16 | R1 (per-tile vertical datum) and R3 (tile-seam spike) briefed to the physics session on 2026-09-25, with owner approval; R2 and R4-R6 to be briefed when needed; R7 dropped (D5) | R2 (drive real terrain) waits for both to land in physics_sim, then for a submodule pin bump. |

---

## References

**physics_sim (read-only):**
- `PLAN.md`: sections 1, 3 (D1–D10), 6 seams, P2 front-end targets, M1, P4 HUD, P8 seam spike, P9 budgets, P11, section 8.
- `core/include/ps/terrain/terrain_source.h`, `terrain_config.h`
- `core/src/terrain/tile_manager.{h,cpp}`
- `core/src/backend/jolt/jolt_backend.cpp`
- `core/include/ps/backend/shape_desc.h`
- `core/include/ps/world/world.h`
- `core/src/io/vehicle_io.cpp`
- `adapters/godot/{CMakeLists.txt, src/*, demo/*}`
- `data/models/car_hyper/car_hyper.rig.json`, `data/vehicles/car_sedan.json`, `data/surfaces/surfaces.json`
- Vault: PHYS-003, -008, -009, -011, -021, -025, -026, -032, -035, -042; TOOL-023.

**OSM and vector data:**
- https://wiki.openstreetmap.org/wiki/Overpass_API
- https://operations.osmfoundation.org/policies/tiles/
- https://operations.osmfoundation.org/policies/nominatim/
- https://opendatacommons.org/licenses/odbl/1-0/
- https://osmfoundation.org/wiki/Licence/Community_Guidelines/Produced_Work_-_Guideline
- https://osmfoundation.org/wiki/Licence/Attribution_Guidelines
- https://download.geofabrik.de/europe/germany/hessen.html
- https://download.geofabrik.de/technical.html
- https://docs.overturemaps.org/guides/transportation/
- https://docs.overturemaps.org/schema/reference/transportation/segment/
- https://docs.overturemaps.org/attribution/
- https://docs.protomaps.com/basemaps/downloads
- https://docs.protomaps.com/basemaps/layers
- https://github.com/mapbox/vector-tile-spec
- https://openmaptiles.org/schema/
- https://valhalla.github.io/valhalla/concepts/tiles/
- https://developers.cloudflare.com/r2/pricing/

**Elevation and land cover:**
- https://hvbg.hessen.de/landesvermessung/geotopographie/3d-daten/digitale-gelaendemodelle
- https://hvbg.hessen.de/geoinformation/open-data
- https://metaver.de/trefferanzeige?docuuid=dbf48a95-b44d-48b3-a5b4-981e4c1bd8e6
- https://hvbg.hessen.de/landesvermessung/geotopographie/3d-daten/3d-gebaeudemodelle
- https://gds.hessen.de/INTERSHOP/web/WFS/HLBG-Geodaten-Site/de_DE/-/EUR/ViewDownloadcenter-Start
- https://www.opengeodata.nrw.de/produkte/geobasis/hm/dgm1_tiff/dgm1_tiff/
- https://geodaten.bayern.de/opengeodata/OpenDataDetail.html?pn=dgm1
- https://registry.opendata.aws/copernicus-dem/
- https://copernicus-dem-30m.s3.amazonaws.com/readme.html
- https://dataspace.copernicus.eu/explore-data/data-collections/copernicus-contributing-missions/collections-description/COP-DEM
- https://registry.opendata.aws/terrain-tiles/
- https://github.com/tilezen/joerd/blob/master/docs/formats.md
- https://mapterhorn.com/data-access/
- https://download.mapterhorn.com/attribution.json
- https://protomaps.com/blog/mapterhorn-terrain/
- https://registry.opendata.aws/esa-worldcover-vito/
- https://esa-worldcover.s3.eu-central-1.amazonaws.com/readme.html
- https://land.copernicus.eu/en/products/products-that-are-no-longer-disseminated-on-the-clms-website
- https://meetingorganizer.copernicus.org/EGU22/EGU22-8994.html

**Libraries:**
- https://geographiclib.sourceforge.io/C++/doc/classGeographicLib_1_1TransverseMercator.html (Karney, J. Geodesy 85(8), 2011)
- https://proj.org/en/stable/install.html
- https://github.com/AngusJohnson/Clipper2
- https://github.com/mapbox/earcut.hpp
- https://github.com/mapbox/vtzero
- https://github.com/mapbox/protozero
- https://github.com/ebiggers/libdeflate
- https://github.com/randy408/libspng
- https://github.com/webmproject/libwebp
- https://gdal.org/en/stable/drivers/raster/cog.html
- https://github.com/yhirose/cpp-httplib
- https://github.com/curl/curl/blob/master/docs/INSTALL-CMAKE.md
- https://github.com/facebook/zstd

**Road geometry:**
- https://a-b-street.github.io/docs/tech/map/geometry/index.html
- https://documentation.beamng.com/world_editor/tools/road_architect/terraforming/
- https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/latest/specification/10_roads/10_05_elevation.html
- https://github.com/ebertolazzi/Clothoids
- https://arxiv.org/pdf/1209.0910
- https://github.com/tordanik/OSM2World/pull/176
- https://carla.readthedocs.io/en/0.9.11/tuto_G_openstreetmap/

**Godot and textures:**
- https://docs.godotengine.org/en/4.4/tutorials/physics/large_world_coordinates.html
- https://docs.godotengine.org/en/latest/tutorials/performance/thread_safe_apis.html
- https://github.com/godotengine/godot/issues/90461
- https://docs.godotengine.org/en/stable/tutorials/performance/pipeline_compilations.html
- https://github.com/TokisanGames/Terrain3D
- https://docs.godotengine.org/en/4.4/classes/class_httpclient.html
- https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/HTTP/FHttpModule
- https://docs.ambientcg.com/license/
- https://polyhaven.com/license

VR/audio integration (2026-10-03): physics_sim master pin ba47951 (fetched
from S:/claude_code/physics_sim) retains the terrain boundary fix and adds
the reusable Windows spatial backend. The supplied car_hyper remains the
default. tools/run.ps1 -VR launches OpenXR Vulkan Mobile; tracked cockpit
and FreeCam rigs share camera-director rebasing. Tyre and eligible live-engine
sources use listener-relative Windows objects with Godot 3D fallback.
Setup, hardware limits and verification: docs/vr_audio.md. The committed
hypercar torque-map engine has no live cycle voice; owner turbo/audio-bank
work remains pending. This presentation work does not change G3-C progress.

Latest physics upgrade (2026-10-03): master pin 2f4599c from
S:/claude_code/physics_sim includes 40923cf's supplied simulated twin-turbo
car_hyper and live engine voice, plus the embedded CMake-path correction.
The default car is unchanged by name and now loads these new library data.
Gauge metadata accepts SimulatedEngineDesc as well as torque-map engines.
Earlier notes about the committed hypercar lacking a live voice are superseded.
G3-C road work is unchanged.


2026-10-03 B8 integration: level-0 terrain now shares road-profile surfacing
between physics and rendering. Separate bridge slabs and lower tunnel floors
have native mesh collision; lower-layer OSM tunnel crossings also supply
highway roofs when the upper ways lack bridge tags. All seven owner-reported
crossings have explicit structure-ID and collision checks. Game-side bounded
reconstruction handles selected source declines and split bridge ways; it
does not complete G3-C's original source-class acceptance. General terrain
smoothing is configurable (enabled/radius_m/strength/passes), halo-correct,
and preserves NoData. The complete B8 height comparison and three-depression
hypercar contact drive pass. See docs/road_surfaces.md for settings, policies
and remaining G3-D/E and G3-F/R3 work.


2026-10-03 building presentation: native OSM footprints, multipolygon courtyards,
recorded heights/level estimates and replaceable batched extrusion presentation
are integrated. Ordinary buildings stream nearby; >=50 m skyline buildings remain
visible up to a configurable 40 km, subject to imported coverage and occlusion.
Worker decoding/tessellation and bounded uploads avoid blocking world startup.
Building collision, architectural roof reconstruction, premade asset matching and
imagery-derived appearance remain later work. Both appearance strategies may be
combined (docs/buildings.md). This feature does not complete the outstanding
G3-C source acceptance or G3-D/E and G3-F/R3 road work.
Physics is pinned to requested e3e1e886b23b7db1c06a9cee4eb469c68452483b from S:.
Reset/start coordinates are configurable and currently target the old spawn point.

2026-10-03 road definition: profile-aligned asphalt overlay, lane/edge paint,
explicit tagged turn arrows and a configurable gentle shared-heightfield verge
are integrated. Background geometry generation and bounded tile uploads preserve
startup streaming. This is a presentation increment; surveyed junction marking
layouts and the remaining G3 source/road acceptance steps are still outstanding.

2026-10-03 owner B8 rollover: identified bare-earth terrain protruding through an inferred tunnel roof near 50.1361,8.4860. Upper-deck terrain clearance, C1 approach grades and shared roof/marking heights are fixed; focused synthetic and exact-location geometry regressions pass. Driving acceptance remains with the owner.


Aerodynamics integration (2026-10-03): physics_sim upgraded from S: to
8b04f0e5a8aa288c0297ed144906cf2918cece27. Reusable signed lift/drag, body moments,
wind, finite drafting wakes, local ground effect (including inverted operation),
rate-limited active wings and energy-budgeted suction fans are implemented.
Game integration supplies configurable atmosphere/wind and hypercar cruise/airbrake
policy; actual wing pitch/lift and aero telemetry use frame snapshots and bookmarks.
See docs/aerodynamics.md and data/world/environment.json. Stock hypercar has no fans.
Aircraft controls and traffic actors remain future gameplay work; this is a
coefficient model, not CFD. Native owner reports 58 focused tests / 1372 assertions.


2026-10-03 aero performance regression: reproduced 105.6 ms ticks from tiny
altitude changes rebuilding native engine steady tables; ground probes cost
0.001 ms. Physics upgraded from S: to 5ec8b7d0a39953b0bfe8f57e1abc3808b5b37ede to remove the
hot-path rebuild while preserving continuous atmosphere. Local real-terrain
hypercar regression is `[aero_perf]`; see docs/aerodynamics.md.

2026-10-03 hypercar engine calibration: physics pin bc6fdc41db4675e3bb56c9f1cb8b418cbc9724fd from S:.
Authored breathing/turbo/cooling/ECU calibration now produces net 1000.2 kW
at 7500 RPM and 1371.0 Nm at 6000 RPM; fixed bore/stroke/compression/boost/E5/limiter
and 1500 kg game mass preserved. See docs/hypercar_engine.md for same-rig before/after
figures and limitations. Native focused dyno validation; road acceptance with owner.

2026-10-03 NPC drafting truck prototype: T places/replaces one moving truck
45 m ahead, Shift+T removes. Background OSM profile routing, conservative
rightmost-through-lane policy, one-way direction and speed/curvature/end braking;
kinematic collision body and native truck WakeSource drive actual hypercar aero.
Frame-latched visual/status/bookmark data. See docs/npc_truck.md. General traffic
AI, full truck drivetrain/articulation and imagery-derived lane refinement remain future work.

2026-10-03 future NPC traffic requirements: player-centered population radius,
minimum spawn distance, unobstructed rightmost-lane placement, live density,
strict applicable speed limits, 80 km/h truck and randomized 80-250 km/h car
fallbacks on roads without limits, physics/weather/surface-aware corner speed,
and public-road highway whitelist plus access restrictions. Planned, not yet
implemented. Specification and sequence: docs/npc_traffic.md.

2026-10-04 adjustable cockpit seat: default eye 20 cm forward, F6 live fore/aft,
height and lateral UI, saved per vehicle. Shared eye anchor updates desktop/XR
cockpit rigs; brake held and look disabled during adjustment. See docs/seat_adjustment.md.

2026-10-04 low-RPM WOT clutch flare: physics pin 60d60fa09f48e38c5fc382b0a082a1407d94ff6c
from S: carries live boosted engine torque into auto-clutch rolling reengagement.
Native matching reproduction re-locks within 2 seconds; 14,544 focused assertions
passed upstream. Game DLL rebuilt; exact owner road replay remains owner acceptance.
Evidence: external/physics_sim/HYPER_CLUTCH_FLARE.md.

2026-10-04 NPC traffic first implementation: automatic real-world population of
lightweight cars/trucks, background connected-profile routing to random reachable
residential-building/public-parking destinations, native collision and drafting
wakes, rightmost usable lanes, public-access/one-way/speed rules, obstruction
checks and spacing. F7 exposes live saved density/radius/exclusion/grip settings.
Arrival currently stops at the nearest ordinary road point then despawns. Out-of-view
retirement discards identity permanently. Defaults: 4/lane-km, 600m radius, 100m
exclusion, 24 vehicles. See docs/npc_traffic.md for current limits and next stages:
shared weather/physical surface grip, junction conflict handling, precise lane
turn rules, parking maneuvers and car-type target distributions.

2026-10-04 NPC lifecycle correction: F no longer clears traffic; actor motion is
frozen during relocation priming and resumes on the same trip. R still clears.
Runtime planning now uses nonblocking published geometry only, indexed destination
lookup and cancellation throughout CPU work, preventing the old distant-derivation
wait from NPC shutdown/reset. Scan timing/population telemetry added. Focused
flip/cancel/traffic/drafting checks: 40 assertions, 5 cases; owner road retest pending.

2026-10-04 traffic settings apparent freeze: owner log continued at ~240 Hz with
frozen=no and player stationary. F7 inadvertently reused the seat panel's modal
full-brake/input-suppression group. Traffic settings now have a separate mouse/UI
guard: driving remains active, camera look is suppressed while using the panel.
Godot script checks pass. Owner log preserved in out/traffic_settings_owner.log.

2026-10-04 NPC population scale: removed adapter cap 24. Defaults are now 2048
maximum, 120/lane-km and 1200m radius, with F7 cap adjustable to 4096 and density
up to 1000/lane-km. Saved settings migrate once. Staged 512-trip batches, four
spawns/tick, spatial neighbor checks/staggered 20 Hz probes, nearby-only wakes and
instanced 22-part rendering replace the main quadratic/spawn/render bottlenecks.
47 focused assertions pass; headless renderer instantiated 2048 cars + 256 trucks.
Full-game performance at high populations remains owner acceptance.
