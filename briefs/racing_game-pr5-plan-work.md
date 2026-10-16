# racing_game #5: Plan work (ready-now packages A1, A4, A7, A8, B1, B2 and owner questions) - the big integration branch

This PR started as "work through the ready-now packages of docs/next_steps_brief.md" and became the integration branch for almost everything done in the cloud between 2026-10-14 and 2026-10-16. It adds garage features (assist unit, turbo kits for any engine, engine bays, drivetrain swaps, automatic/DCT/CVT gearboxes, cooling upgrades, engine internals), cockpit and camera work (hazard/horn clicks, driver body motion, drone and FPV drone, per-car bumper camera), physics features (weather, RACING mode, EPB, ABS/ESC, TC fix), visuals (time of day, headlights with exposure, Mobile renderer default, imported car models), the terrain migration phases 3a-6, underpass decks, routing/minimap, LOD settings and render-stutter fixes. Owner wish (2026-10-14 19:26): "I want to actually merge this branch at some point." Sub-PRs were merged INTO this branch, not main. **The PR body on GitHub is stale (it describes only the first slice, 2026-10-14); the comments are the real log.** The Windows machine has never run this branch.

## State at hand-off

| Item | Value |
|---|---|
| Repo | SebastianDanielFrenz/racing_game |
| PR | https://github.com/SebastianDanielFrenz/racing_game/pull/5 |
| Head branch | `claude/plan-work-q2a5oy` |
| Base | `main` (not stacked). Many other PR branches target THIS branch (see sub-PR list) |
| Head sha | `07581be` ("Expose JSON dependency used by public cooling upgrade headers"), verified with `git rev-parse origin/claude/plan-work-q2a5oy` |
| Draft/ready | DRAFT |
| Mergeable | `clean` (GitHub), base sha at PR read = main `81e9f21` |
| Size (GitHub, may lag the newest merges) | 422 files, +43,946 / -4,037, 165 commits, 101 comments |
| external/physics_sim pin | `f38075e` (physics_sim master: contents mass #10, TC fix #11, automatic/DCT gearboxes #12). It is on the default branch (`git branch -r --contains` lists origin/master). |
| external/geo2map_engine pin | `2571f43`, on geo2map `main`. (Earlier today the branch was at `106d5ba`; PR #26 states geo2map follows the base at 2571f43.) |
| Not-yet-merged sub-PRs | #22 part-mass (ready), #26 automatic/DCT gearboxes (draft), #33 damage model (draft). #33 pins physics_sim PR #15 commit `26a193d` (branch `claude/cooling-capacity`, NOT on master); #26 pins physics `ad401bc` (master, includes CVT #13) per its body. Pins will conflict when these merge; re-pin to physics master after physics_sim #15. |

## What is done (all merged into the branch unless marked)

First slice (PR body): A1 setup check (turbo stage-map coverage + loader warnings in `data/garage/advisor.json`, envelope-cost measured, box grounds checked); A4 sedan assist unit (`insert_if_absent` rule; No unit/Street/Sport/Racing); A7 N2O gain measured (`rg_unit_tests "[nitrous][gain]"` hidden; hyper_n2o +75 % dv in 2 s, max boost 2.55 bar vs 1.8 target; sedan gen_n2o +6 %); A8 brake-bias limits into `setup_options.json`; B1 cockpit hazard/indicator/horn clicks through DriverActions (horn held while mouse held; `cockpit_test.gd` 186 checks); B2 one steering-lock field `steering_wheel_lock_deg` (Pico 540, Ventura 612, Runabout 225); cargen models round 1+2 (Pico, Ventura GT, Runabout T: chord-tolerance tessellation, 3 LODs each, lamps/grilles/discs/tread; rig/socket names unchanged).

Done in comments / sub-PRs (one line each, see list below for numbers): runtime turbo-map extension in C++ (`data/engines/turbo_map_extension.json`, `[turbo_map]` tests, commit 94da60e); LOD settings, route planner, minimap, `docs/navigation.md` (8991024); time of day + sun/moon + weather/time widget + headlights key O + crop rules (`docs/visual_world_plan.md`, 99deb8d); physics weather environment in the Session (rain wets road, derived temperature from `data/world/climate.json`); stutter fixes + `RG_FRAME_STATS`/`RG_FRAME_SPIKE` diagnostics (fb590ec); cameras: driver virtual-body motion (setting `camera.body_motion`, default 1.0, `data/controls/camera_motion.json`, toned down in 177e7ea), per-car bumper camera from model bounds, orbit/drone lag fixed (0 m error), speed shake + FOV widening (flat screen only); gamepad defaults (paddles 16-19, touchpad, MISC1, D-pad right, L3); 8 imported fictional car models via `tools/carimport/carimport.py` (GLBs via LFS, a2a5b84) with credits; terrain migration phases 3a-6 (geo2map #3-#7 + racing_game #8, #9, #11, #12, all merged 2026-10-15); overpass-spike/underpass work (#6, #16 with geo2map #13); EPB/RACING mode/ABS-ESC in game (#10); TC fix (#25 with physics_sim #11/#8); debug-log key copies coordinates (#15).
Unreal port docs kept (`docs/unreal_port.md`, `docs/car_art_shortlist.md`) but UE work is paused by owner.

## Sub-PRs merged into this branch (base claude/plan-work-q2a5oy)

| PR | One line |
|---|---|
| #6 | Keep grade-separated roads apart in terrain carving (overpass spikes at utm32n/2/457/5422) |
| #7 | Visuals: imported car models, shaders, textures |
| #10 | RACING drive mode, electric parking brake, ABS/ESC in the game |
| #13 | Move terrain, road and building mesh packing off the main thread |
| #14 | Perf: gauges redraw on change, renderer/render-thread switches, measured engine assessment |
| #15 | Debug log key (Delete) copies car coordinates to clipboard + shows them 6 s |
| #16 | Underpass decks from geo2map #13 (deck only over the crossing span, max 80 m) |
| #17 | Automatic headlamps (on below 1000 lux, off above 7000) and sky-driven exposure for lamps |
| #18 | Mobile renderer as default, Compatibility as fallback (stacked on #14) |
| #19 | Garage drivetrain swap: FWD/RWD/AWD and axle differentials (open, LSD, Torsen, viscous, locked) |
| #20 | Drone camera: rig maths in rg_core, drone in camera-switch test, VR drone rig |
| #21 | Garage turbo kits (Street 0.5 bar, Sport 1.0, Twin) for any engine without a turbo; also engine bore upgrades |
| #23 | Pilotable FPV drone: kinematic `rg::FpvDrone`, mode `fpv_drone`, rotor sound |
| #24 | Engine bays: any engine into any car if it physically fits (measured bays, reason shown when not) |
| #25 | TC fix: physics_sim d11626d (closed-loop LC hand-over, Sport spark retard) + game-car regression test |
| #27 | Opt-in: render class windows from served `g2m.terrain.class` layer |
| #28 | Cooling upgrades (radiator, fan, oil cooler, block cooling) + coolant/oil check in advisor |
| #30 | CVT gearboxes in garage (into #26's branch, so reaches plan-work only when #26 merges) |
| #31 | Pin physics_sim to master f38075e |
| #32 | Chase/free/walker/orbit rig maths into rg_core; walker rig in camera-switch matrix (Godot run pending) |

Also merged per comments, with other bases: #4 road crossings, #8/#9/#11/#12 terrain migration 3a/3b/4/5, #1.. earlier items. (#29 not read; #22/#26/#33 open, below.) Note: #30 shows merged but its base is `claude/automatic-gearbox` (#26), so check #26's merge to land it.

Open sub-PRs: **#22** part-mass tree (ready, clean; 39 files); **#26** automatic + dual-clutch gearboxes (draft, clean; owner said "resolve conflicts" 14:46 on 10-16; head 68e3da4); **#33** damage model piece 1 (draft, clean; 22 files).

## Plan package status (docs/next_steps_brief.md + PLAN.md; inferred from comments/PRs, not re-verified in code)

Lane A (physics follow-ups on game side)
- A1 advisor/hitch/box: DONE. A2 pin bump after BOV r3 + G1: PARTLY (BOV r3 merged in physics, pin now at master f38075e; G1 limiter range status unknown; the `test_rev_limit_extremes` workaround removal and `[long]` advisor rerun are NOT confirmed done).
- A3 RACING + EPB: DONE (#10). A4 assist unit: DONE. A5 ABS UI: PARTLY (#10 wires ABS/ESC, but comment 10-15 04:27 says no shipped car fits ABS/ESC yet, so nothing changes in play). A6 clutch bite: DROPPED by owner ("not now"). A7 N2O gain: DONE. A8 brake-bias to data: DONE.
Lane B (cockpit/controls/VR)
- B1 DONE. B2 DONE. B3 cursor: owner agreed it stays captured, no change. B4 M1 cockpit furniture/sockets: NOT STARTED (needs car-model update in physics_sim data/models; hold until car-art decision). B5/B6 (P2 variants, P3 clicking): NOT STARTED beyond B1. B7: gamepad bindings DONE (41020a9); Dashboard v2 NOT STARTED (agent asked what it should contain, no answer on record); VR menu check NOT STARTED.
Lane C (terrain migration; geo2map first)
- C1 phase 3 (decks, ribbons, streamers, buildings): DONE (geo2map #3/#4, racing_game #8/#9). C2 phase 4 (tunnel floors, bridge anchors): DONE (#5/#11, release id bump). C3 phase 5 (200 m abutment walk): DONE (#6/#12, release id bump). C4 phase 6 carve readiness: DONE (geo2map #7). C5 phase 7 carve switch (delete game carve, no fallback): BLOCKED waiting for owner's real-store probe (GAP 1 = 0). C6 phase 8 cleanup: NOT STARTED (needs C5). C7 (terrain.class, underpass dips): PARTLY (#27 opt-in served class, #13/#16 underpass decks, #6 spikes). C8 R3 completion (markings P3, 20-junction acceptance): NOT STARTED/unknown. C9 render streaming leftovers: PARTLY (#13 off-thread packing; far road layer not). C10 fast first load: NOT STARTED.
Lane D
- D1 wider stage map: owner chose to SKIP static map and instead do the C++ port + runtime generation: DONE (physics_sim PR #2 merged, game side 94da60e). The N2O hyper's remaining step is genuine compressor choke (0.519 kg/s passes vs 0.534 wanted), shown as advice. D2 engine design GUI: NOT STARTED. D3 supercharged Demon 170: NOT STARTED.
Lane E
- E1 G4/R4 routing + minimap: MOSTLY (geo2map #2 A* routing; game minimap, route destination via F10, manoeuvre list, "Off route"). Missing: turn restrictions, auto re-route, destinations beyond 10 km square, full map, square minimap only. E2 G5/R7 land cover/buildings/water/vegetation: NOT STARTED (C1 prerequisite now done). E3 R9 performance: PARTLY (stutter fixes, #13, #14; owner target 240 fps at medium; 60 fps on 120 Hz seen in his log). E4 G6 protocol/server: geo2map #8 (G6 HTTP) merged per comment. E5-E9: NOT STARTED.
Other: Unreal port prep PAUSED (docs kept). Automatic/DCT/CVT: DONE in physics, game side pending #26. Damage model: first piece #33 pending. Part mass: #22 pending. Shader quality tiers: dropped until UE port. Textures: network blocked in the cloud, plan names sets to fetch locally.

## What is unfinished / next steps (ordered)

1. On Windows: fetch physics_sim/geo2map submodules, build, run `tools\ci.ps1` and the smoke flags below. Nothing in this branch was ever built on Windows.
2. Decide merge order for #22, #26 (resolve conflicts the owner asked for), #33 (re-pin after physics_sim #15), then retire stacked pins (physics `ad401bc` / `26a193d` vs branch pin `f38075e`).
3. Terrain migration phase 7: bake `g2m.roads.geom,g2m.terrain.carved` into corridor and Hessen stores (`g2m_tiler bake --layers ...`), run roadcmp probe (`RG_ROADCMP=1`), require GAP 1 = 0, drive a bridge/tunnel stretch, report numbers on the PR. Only then phase 7; the agent will not touch the carve code before.
4. Decide car art (docs/car_art_shortlist.md) and brand-name models (see questions); then hypercar/sedan model refresh in physics_sim and B4 M1 furniture.
5. Renderer: check the Mobile renderer on Windows (see risks), decide questions on #18.
6. Stutter: owner supplied logs; send a fresh run with `RG_FRAME_SPIKE_MS=20` on this branch.
7. Remaining owner-requested items not started: Dashboard v2 spec, more physics (G1 confirmation), E2 land cover/buildings, visible rain/snow, real textures.

## Open questions for Sebastian (unanswered)

From the PR body (all three still without a recorded answer):
1. A1: setup check shows 75.0 % (sketch maps) vs 68.8 % (full maps); recommendation: keep sketch (moot now that runtime map generation exists).
2. A1: flat-world ground from 2000 m static box to a tessellated mesh (box kept only for hash_check); recommendation: yes (box had 3 phantom casts in 200,000).
3. A7: car_hyper_n2o never sprays in Normal (mode file allows nitrous only in Sport). Recommendation: HUD hint "N2O: Sport mode only".
From comments:
4. Brand-name LFS models (Ford GT/Model T, Porsche 911, 3 Lamborghinis, Pagani, McLaren, Mercedes, Honda): recommendation keep out of playable cars (reference/private only). Also: are look-alikes of real cars acceptable? No answer on record.
5. Car art source for later: licensed models (recommended) vs generator vs commissioned. Shortlist in `docs/car_art_shortlist.md`; owner answer unclear, imports of the fictional set happened.
6. Dashboard v2 content (B7).
7. Unreal port decisions (10 questions in `docs/unreal_port.md`); moot while paused.
8. Driver camera: speed shake and FOV widening currently on chase/bumper/flat cockpit; asked whether cockpit-only.
9. Reality check on Frankfurt monthly climate figures (approximations of DWD normals, flagged in the file); rain intensity 4 mm/h choice.
10. Per-PR questions: #17 (3 questions), #18 (Forward+ as option? bring back brighter look? auto-fallback to Compatibility after Vulkan crash?), #19 (4 q), #20 (FPV done, answered), #21 (4 q), #22 (spawn fill, bare shell masses, strippable slots), #26 (pad button for manual mode; real gearbox data to be done locally by owner), #33 (4 q: wear speed, repair button location, damage_part name, should ruined parts stop the engine), geo2map #13 (steep walls decided; deck cap 80 m kept).

## Decisions already answered (do not re-ask)

- B3 cursor stays captured (already works); A6 clutch bite not now; B1 release-button hook, held click holds horn.
- D1: skip static wider map; port generator to C++ and generate at runtime with a safety margin, cached ("Port to C++ and then generate at runtime"). "Build everything in physics_sim yourself, too."
- Downloads of textures/sounds/models allowed, obey licences (cloud network blocks ambientcg, polyhaven, sketchfab, fab, kenney, opengameart).
- Start time: real date 14:00 real-time clock; headlights follow the car's switch and O (agent first built auto-on at owner's complaint that they are not automatic at night, then #17 added auto headlamps + exposure; automatic on by default, switch/O override); Q3 install physics weather: yes; Q4 derive temperature.
- Car-model refinement yes; shaders dropped until after UE port; UE port stopped, docs kept.
- Mobile renderer now (#18), Mobile default, Compatibility fallback. Target platforms: Steam Frame (ARM64 Linux), Windows (NVIDIA Pascal+, AMD RDNA2+), Linux PCs (NVIDIA/AMD). Intel GPU later maybe; Android/iOS OUT.
- Camera: virtual-body head motion applies in the cockpit view only (VR included) at full strength by default for his first VR test; tone-down: sideways -70 %, front-back -25 % (up-down felt about right). Bumper camera per car from model bounds.
- Long tunnels follow straight lines between portals "for now"; underpass: deck only over crossing span, terrain keeps steep cutting walls, keep parameters.
- Automatic gearbox UX: shift up/down binds step P-R-N-D, manual mode by a key (Y); CVT follow-up yes; double clutch included. Any engine into any car if it fits; drivetrain swaps (FWD/RWD/AWD + differentials) accepted; mass per part tree (parent includes children). Penalties for layout swaps asked.
- Damage model rules: persists until repaired; repairs free; repair requires explicit action (garage, or box/finish in races later); damage changes driving.
- Debug log key copies coordinates to clipboard + on screen.
- Create a manager PR on geo2map (geo2map_engine #12, `docs/manager.md`) - post geo2map requests there.
- Drone camera PR requested; FPV drone built on owner's answer. Stage-map question settled. C1 terrain migration: owner answers in geo2map `docs/terrain_migration_plan.md` section 5 (bridge walk 200 m, three release-id bumps, geo2map serves tunnel roofs, curved/sloped tunnels must work).
- Performance target: 240 fps at medium settings after shaders; HDR support desirable later.
- "Resolve conflicts" on #26 (10-16 14:46).

## Owner rules beyond this PR

- All work on branches and draft PRs; nothing merged to main without owner; no coordinator approval needed.
- Metric units, plain reports, "Don't overthink it", "Don't run tests unless really necessary" (scoped checks).
- Engine-neutral logic in `core/`; `godot_ext`/GDScript thin; numbers in data files. Limits are config.
- One agent and one CI owner per repo; submodules `external/*` read-only from racing_game (only pins move).
- Pin bumps: `git add external/<sub>` before CI (ci.ps1 resets submodules). Git LFS .glb only pushable from the owner's machine (the cloud could not upload LFS except one session pushed 8 GLBs in a2a5b84; verify).
- Questions go in the PR, before/after images welcome (PR-images orphan branch).
- Usage cap of original account noted in brief (stops at 94 % weekly).

## Known issues / risks

- Everything was verified on Linux clang only; Godot extension linked/built on Linux only for some slices; GDScript changes in #22, #26, #33 only parse-checked. Owner has not run any of it on Windows.
- Mobile renderer (#18): on the cloud's software Vulkan driver the soft sun-shadow filter left the car unlit in sun (suspected driver bug). Check on Windows; brighter look differs from Compatibility.
- Pins differ between sub-PR branches (#26 physics `ad401bc`, #33 `26a193d` branch commit, branch `f38075e`); merges will conflict in `CLAUDE.md`, `vehicle_setup.cpp`, `tests/unit/CMakeLists.txt`, `gearbox_catalog.json`.
- Phase 7 carve switch deletes the game's carve with no fallback; do not proceed without the zero-gap probe on real stores. Phases 4 and 5 bumped release ids: re-bake/warm stores before test drives (first load otherwise derives roads.geom, ~13 s release, the debug DLL times out).
- Physics caveats: N2O boost overshoot (2.55 bar vs 1.8 target); compressor choke on n2o hyper; snow does not reduce grip (no physics model); road wets on a 60 s env step; rain/snow have no visuals; ABS/ESC not fitted to any shipped car.
- All new masses/ratios/thresholds are [AS] estimates; Frankfurt climate numbers are approximations; the Pico flares widen it 5 cm visually.
- Intermittent unnamed failing test once in full runs (not reproduced).
- Gamepad paddles/touchpad/MISC1 untested on hardware; D-pad right overlap with drone target.
- Stutter: heal_tick can still rebuild 2 chunks/frame on main thread; logs show ~60 fps on 120 Hz; audio probe 2-3 ms/frame and road/building streaming peaks 8-32 ms were noted.
- Imported cars: `carimport.py` needs UTF-8 (fixed 7852d69); five imported cars have no catalog entry (#22 note). Brand-name models must stay out of playable cars pending decision.
- `game/scripts/traction_control_test.gd.uid` should stay untracked per brief.

## Local build, run and verify on Windows

```powershell
git -C S:\claude_code\physics_sim fetch origin
git -C S:\claude_code\geo2map_engine fetch origin
cd S:\claude_code\racing_game
git fetch origin; git checkout claude/plan-work-q2a5oy; git pull
git lfs pull
git -c protocol.file.allow=always submodule update --init --recursive
. .\tools\common.ps1; Enter-VsDevShell
cmake --preset release -DRG_BUILD_GODOT_EXTENSION=ON
cmake --build --preset release
tools\ci.ps1 -Only release -SkipSmoke       # scoped core tests
build\...\rg_unit_tests "[garage]"          # also: [gearbox_swap] [automatic] [cvt] [turbo_kit] [turbo_map] [part_mass] [damage] [engine_swap] [limit_advisor] [driver_actions] [session] [nitrous][gain] [thermal]
```
Smoke: `tools\smoke_test.ps1 -Garage` (garage_test.gd; run it after #26 - expects no coming-soon gearbox sections), `-Shell`, `-Cameras` (includes drone and walker sections, Godot run of walker still pending), `-Controls`, `-CarBrowser`, `-TerrainStream`. Needs map store (RG_G2M_HOME): `-Drive`, `-DriveDelayMs 200`, `--road-shots`, underpass/overpass check at UTM 32N E 468936 N 5552913 (session 4936.064, -6086.422; tile utm32n/2/457/5422), route/grip `[.][realdata]` tests, phase-7 roadcmp probe. Needs LFS models: garage, car browser, drive with imported cars (`python tools\carimport\carimport.py` regenerates them; GLBs committed via LFS in a2a5b84).
Game: `tools\run.ps1 -Drive` (real world), `-Flat`; for stutter: `$env:RG_FRAME_SPIKE_MS="20"; tools\run.ps1 -Drive 2>&1 | Tee-Object out\stutter_log.txt`, then Select-String for `RG_FRAME_STATS|RG_FRAME_SPIKE|RG_TICK_SPIKE|RG_TERRAIN_FALLBACK|RG_TERRAIN_STREAM`. Headless check: `godot --headless --path game --quit-after 200000 -- --garage-test --shell-user-dir <fresh dir>`; `cockpit_test.gd` (186 checks).
Screenshots worth taking: Mobile vs Compatibility (noon, dusk, night with headlamps), garage showroom shadows, cockpit with body motion, new cars (Pico, Ventura, Runabout, imported), minimap/route HUD, weather widget, wet road. Hash expectation: `hash_check` vehicle_step_steer `state_hash=0xace4d38ea725e065` (per #26/#33 text).

## What was NOT verified in the cloud

Windows build; Godot rendering and every shader/visual (Mobile renderer, exposure, headlights, models); real map store (minimap/routing, underpass fix on real tile, terrain phases, phase-7 probe); LFS-dependent garage on a fresh checkout; VR (body motion, drone rig); gamepad hardware; the Godot garage UI for #22/#26/#33; stutter root cause (render CPU+GPU measured about 3 ms, rest unattributed); manual-mode entry in #26 (read from code, not run). Core tests passed on Linux (642-759 cases depending on date, e.g. 751/751 on #33, 2,350 core assertions on visuals slice).

## Files to read first

1. `docs/next_steps_brief.md` (package list; stale on A/B status)
2. `PLAN.md` section 12 and its dated status lines (R5 drone/walker follow-up, S1, nitrous)
3. `CLAUDE.md` (architecture map; sub-PR sections appended)
4. `docs/visual_world_plan.md`, `docs/navigation.md`, `docs/unreal_port.md`, `docs/car_art_shortlist.md`, `docs/car_model_import.md`
5. `docs/garage_ui.md`, `docs/damage_model.md` (on #33), `docs/terrain_migration_plan.md` in geo2map
6. `core/include/rg/part_mass.h` (on #22), `gearbox_catalog.h`, `driver_actions.h`, `camera_math.h`
7. `data/controls/camera_motion.json`, `data/controls/cabin.json`, `data/garage/advisor.json`, `data/engines/turbo_map_extension.json`, `data/world/climate.json`
8. `tools/carimport/carimport.py`, `tools/cargen/`
