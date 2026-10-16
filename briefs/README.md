# Cluster brief: racing_game + physics_sim + geo2map_engine

Hand-off from the cloud sessions to local agents on Sebastian's Windows machine (WSL2, Git LFS car models, geo2map map stores).
Written 2026-10-16 evening, when Sebastian ended cloud development on all three repos. Each open PR has its own brief in this folder;
read this file first, then the brief for the PR you continue.

Owner: Sebastian (GitHub `Gogolinolett-AI`, repo owner account `SebastianDanielFrenz`). He writes German or English; answer plainly, metric units.

## 0. Heads documented in these briefs

If a branch has moved past these, the brief is older than the branch: read the new commits first.

| Branch | Head |
|---|---|
| racing_game `main` | `5e3ca18` |
| racing_game `claude/plan-work-q2a5oy` (#5) | `07581be` |
| racing_game `claude/part-mass` (#22) | `88896b3` |
| racing_game `claude/automatic-gearbox` (#26) | `68e3da4` |
| racing_game `claude/strip-visual` (#29) | `b522ab3` |
| racing_game `claude/cooling-damage` (#33) | `caf2453` |
| physics_sim `master` | `c08b00c` |
| physics_sim `claude/block-cooling` (#14) | `97e4a85` |
| physics_sim `claude/cooling-capacity` (#15) | `4e81619` |
| geo2map_engine `main` | `cd76bcb` |
| geo2map_engine `claude/manager` (#12) | `9c3a830` |

## 1. The three repos

| Repo | Role | Default branch | Head at hand-off |
|---|---|---|---|
| [racing_game](https://github.com/SebastianDanielFrenz/racing_game) | The game: Godot 4.7.2 project (`game/`), engine-neutral C++ core `rg_core` (`core/`), GDExtension `rg_godot` (`godot_ext/`). Open world on real map data of Main-Taunus-Kreis, Hochtaunuskreis and Frankfurt-Höchst. | `main` | `5e3ca18` |
| [physics_sim](https://github.com/SebastianDanielFrenz/physics_sim) | Vehicle and rigid-body physics on Jolt (`ps_core`): engines, turbos, N2O, gearboxes, tyres, thermal, weather. Submodule `external/physics_sim`. | `master` | `c08b00c` |
| [geo2map_engine](https://github.com/SebastianDanielFrenz/geo2map_engine) | Geography to game map: tile layers (DEM, OSM roads, roads.geom, terrain.carved/class), offline tiler, in-process and HTTP map server, routing (G4), `g2m_ps_bridge`. Submodule `external/geo2map_engine`. Never depends on physics_sim or a game. | `main` | `cd76bcb` |

racing_game pins both libraries as submodules. Library changes reach the game only through a pin bump in racing_game.
`racing_game/CLAUDE.md` is the architecture map (very detailed); `PLAN.md` holds the design and milestone plan; the library repos have their own `CLAUDE.md` and `PLAN.md`.

## 2. The most important fact: racing_game work is on the plan branch, not main

- `main` stops at road crossings (PR #4) plus the ARM64/SteamOS installer and LFS bootstrap commits (`c4493e5`, `5e3ca18`).
- Roughly 25 PRs from 2026-10-14..16 were merged into **`claude/plan-work-q2a5oy`** (head `07581be`), which is **draft PR #5** against main. Sebastian wants to merge it into main eventually ("I want to actually merge this branch at some point").
- main has 2 commits the plan branch lacks (installer + LFS bootstrap). Merge main into the plan branch before or while landing #5.
- Base new racing_game work on the plan branch unless Sebastian merges #5 first.
- Open PRs stacked on it: #22 (part masses), #26 (automatic/DCT/CVT gearboxes; already contains merged #30), #29 (strip visual, stacked on #22), #33 (damage model, needs physics_sim #15).

## 3. Open PRs and their briefs

| Repo | PR | Branch -> base | State | Brief |
|---|---|---|---|---|
| racing_game | [#5](https://github.com/SebastianDanielFrenz/racing_game/pull/5) Plan work (integration branch) | `claude/plan-work-q2a5oy` -> `main` | draft | `racing_game-pr5-plan-work.md` |
| racing_game | [#22](https://github.com/SebastianDanielFrenz/racing_game/pull/22) Per-part mass tree | `claude/part-mass` -> plan branch | ready | `racing_game-pr22-part-mass.md` |
| racing_game | [#26](https://github.com/SebastianDanielFrenz/racing_game/pull/26) Automatic, DCT (and CVT via #30) gearboxes | `claude/automatic-gearbox` -> plan branch | draft | `racing_game-pr26-automatic-gearbox.md` |
| racing_game | [#29](https://github.com/SebastianDanielFrenz/racing_game/pull/29) Strip visual | `claude/strip-visual` -> `claude/part-mass` | draft | `racing_game-pr29-strip-visual.md` |
| racing_game | [#33](https://github.com/SebastianDanielFrenz/racing_game/pull/33) Damage model piece 1 (cooling) | `claude/cooling-damage` -> plan branch | draft | `racing_game-pr33-cooling-damage.md` |
| physics_sim | [#14](https://github.com/SebastianDanielFrenz/physics_sim/pull/14) Engine water jacket, pump upgrade, steady-state solver | `claude/block-cooling` -> `master` | draft | `physics_sim-pr14-block-cooling.md` |
| physics_sim | [#15](https://github.com/SebastianDanielFrenz/physics_sim/pull/15) Runtime radiator/fan capacity API | `claude/cooling-capacity` -> `master` | draft | `physics_sim-pr15-cooling-capacity.md` |
| geo2map_engine | [#12](https://github.com/SebastianDanielFrenz/geo2map_engine/pull/12) Manager tracker (docs only) | `claude/manager` -> `main` | draft | `geo2map_engine-pr12-manager.md` |

### Suggested merge order (inferred from the stacking; Sebastian merges, agents do not)

1. physics_sim #14 and #15 into master (independent of each other).
2. racing_game #22, then #29 (re-target #29 to the plan branch after #22 lands; it has one known conflict in `data/vehicles/catalog.json`).
3. racing_game #26 (carries #30's CVT work).
4. racing_game #33 after re-pinning `external/physics_sim` from its PR-branch commit to physics_sim master (which then contains #15).
5. One pin bump PR on the plan branch to physics_sim master (with #14) to adopt the water jacket (see the physics_sim #14 brief).
6. Merge main into the plan branch, then #5 into main.

Sub-PR pins differ (plan branch physics `f38075e`; #26 `ad401bc`; #33 physics `26a193d` = first commit of physics_sim #15's branch, NOT on master, and geo2map `16a8ec6`). Expect conflicts in `external/physics_sim`, `CLAUDE.md`, `core/src/vehicle_setup.cpp`, `tests/unit/CMakeLists.txt`, `data/gearboxes/gearbox_catalog.json`. Resolve by taking physics_sim master and re-running the affected tests.

## 4. Submodule pins

| racing_game ref | physics_sim | geo2map_engine |
|---|---|---|
| `main` (5e3ca18) | `1e07c0e` | `3092d66` |
| plan branch (07581be) | `f38075e` | `2571f43` |

- All four are on the submodules' default branches; both submodules have moved on since (physics master `c08b00c` has CVT/DCT launch control #13 and a CMake fix #16; geo2map main `cd76bcb` has turn restrictions #14 and lane hints #16).
- `geo2map_engine/bridges/physics_sim/physics_sim.pin` is `b90a531`, which is not on physics_sim master and matches neither racing_game pin. racing_game's CLAUDE.md says the two must be equal; only geo2map's own `bridge-release` CI leg reads it. Bump it to the physics_sim commit racing_game pins when next touching geo2map.
- On Sebastian's machine `.git/config` points the submodules at local paths `S:\claude_code\<repo>`. Never run `git submodule sync` there; use `git -c protocol.file.allow=always submodule update --init --recursive`. Stage the pointer (`git add external/<sub>`) before running `tools\ci.ps1`, because ci.ps1 resets submodules.
- racing_game never edits files inside `external/`. Change the library repo on its own branch/PR, then bump the pin.

## 5. Build, test, run (Windows, VS 2022 dev shell; scripts enter it themselves)

racing_game:
```powershell
setup.cmd                                   # toolchain via winget, submodules, map-data report (-CheckOnly installs nothing)
git lfs install --local; git lfs pull       # 22 downloaded + imported car GLBs; verify: git lfs ls-files --size, git lfs fsck
git -c protocol.file.allow=always submodule update --init --recursive
cmake --preset release -DRG_BUILD_GODOT_EXTENSION=ON; cmake --build --preset release
ctest --preset release --output-on-failure   # or rg_unit_tests "[tag]" for one area (Catch2 v3 tags)
tools\ci.ps1 -Only release -SkipSmoke       # the per-change check named in docs/next_steps_brief.md; tools\ci.ps1 = full
tools\smoke_test.ps1 -Garage | -Shell | -Cameras | -Controls | -CarBrowser | -TerrainStream | -Drive | -DriveDelayMs 200
tools\run.ps1 [-Drive | -Flat | -VR | -Preset debug]   # build + launch; default opens the menu shell
```
- Presets: debug, release, relwithdebinfo (run.ps1 default), asan (clang-cl), msvc-release; `linux-release` (rg_core + tests only); `linux-arm64-release` (cross build of `rg_godot` for Steam Frame).
- Use release/relwithdebinfo for real-world drives: a debug DLL times out deriving roads.geom at the start-up gate.
- PowerShell 5.1 smoke tests fail on any stderr output; tools print to stdout.
- Godot may be running on Sebastian's PC and lock `game/bin/librg_godot.dll`.
- Leave `game/scripts/traction_control_test.gd.uid` untracked.
- `hash_check` (vehicle_step_steer) must stay unchanged unless a physics pin bump moves it on purpose; the PR texts name the expected hash.

physics_sim: `cmake --preset release; cmake --build --preset release; ctest --preset release --output-on-failure`; `tools\ci.ps1` (fast by default; `-Affected`/`-Topic a,b`, `-Sanitizers`, `-Full`; 3 WSL Linux legs, `-SkipWsl` to skip; cross-platform hash gate Windows == Linux). `[long]` tests are excluded from `-Affected`; run turbo/N2O ones by hand. Read its `AGENTS.md` and `RUNTIME_LOCKS.md` before touching the Godot adapter.

geo2map_engine: `cmake --preset release; cmake --build --preset release; ctest --preset release --output-on-failure`; `tools\ci.ps1`; `--fresh` on a stale cache. Tools: `g2m_tiler` (import/bake), `g2m_road_probe`, `g2m_server` (HTTP sidecar), `tools/junction_view`.

WSL2 is used for physics_sim's Linux CI legs, the Frame cross toolchain (`setup_dev_env.ps1 -Frame`) and Linux presets.

## 6. Map data

- Not in Git. `${RG_G2M_HOME}` defaults to `<racing_game>/cache/g2m/home-r1`, `${RG_G2M_CACHE}` to `<racing_game>/../geo2map_cache` (`core/src/world_config.cpp`). Sebastian's machine historically used `S:\claude_code\geo2map_cache\home-r1`.
- Worlds in `data/world/`: `world_config.json` (home), `world_config_rhein_main_corridor.json` (store `rhein-main-corridor-r2`; VR default, A5 probe route), `world_config_rhein_main_hessen.json` (`rhein-main-hessen-r1`). The flat world needs no store (`tools\run.cmd -Flat`).
- `setup.cmd` only reports which stores exist; it downloads nothing.
- **Re-derive/bake needed:** geo2map PRs merged 10-14..16 (road crossings, terrain migration phases 4 and 5, underpasses) bumped release ids and the roads.geom config hash. Re-bake or warm the stores before test drives; otherwise the first load derives roads.geom on the fly (about 13 s per area in release).
- `[.][realdata]` tests and `-Drive` smoke tests SKIP without a store. No cloud session ever had a real store, so every real-data claim in the PRs is unverified.

## 7. What the cloud could and could not verify

- Cloud sessions ran Linux x86-64 with clang. They ran rg_core/physics/geo2map unit tests, a headless Linux build of the GDExtension (`--flat`, `--bindings-test`), and software renders of generated models.
- **Never verified:**
  - anything on Windows;
  - Godot rendering on real GPUs (the Mobile renderer, which became the default on the plan branch, showed an unlit car in sun on the cloud's software Vulkan);
  - real map stores;
  - VR, gamepad hardware;
  - the Godot garage/HUD UI of #22, #26, #29, #33 (GDScript only parse-checked).
- First job on any PR: Windows build plus its smoke flag, then screenshots for the PR.

## 8. Platforms

- Targets: Steam Frame (ARM64 Linux, standalone VR), Windows PCs (NVIDIA Pascal+, AMD RDNA2+), Linux PCs (NVIDIA/AMD). Intel GPUs maybe later. **No Android/iOS.**
- The Mobile renderer is the default on the plan branch (Steam Frame needs it); Compatibility is the fallback (`graphics.renderer` setting, `play_compat.cmd`). `main` still uses gl_compatibility.
- The ARM64 build:
  - `linux-arm64-release` cross-builds `librg_godot.arm64.so`; docs are in `docs/linux_arm64_install.md`.
  - The symbol/dlopen checks and the bindings test passed under qemu.
  - The native installer is `tools/install_linux.sh`.
  - **None of this has been tested on a device.**
- A desktop Linux x86-64 preset with the extension does not exist yet.

## 9. Open work beyond the open PRs

racing_game (`docs/next_steps_brief.md` lanes A-E; status per package in the #5 brief):
- **C5, terrain migration phase 7 (carve switch): blocked on Sebastian's real-store probe.**
  - Bake `g2m.roads.geom,g2m.terrain.carved` into the corridor and Hessen stores.
  - Run the roadcmp probe; it needs GAP 1 = 0.
  - Drive bridges and tunnels.
  - Phase 7 deletes the game's carve with no fallback, so do not start it before the probe passes. Phase 8 cleanup follows.
- Cockpit (`docs/cockpit_remaining_work_brief.md`): B4 permanent cockpit furniture and sockets, B5 fitted-state variants, B6 click interaction, Dashboard v2 (content never specified), VR menu check, VR hands.
- E2: land cover, buildings, water and vegetation (G5/R7).
- E3: performance; the target is 240 fps at medium settings. Stutter logs showed about 60 fps on a 120 Hz screen.
- E1 routing leftovers: auto re-route, a full map, destinations beyond 10 km.
- Damage model beyond cooling: knock, piston speed, charge-pipe boost, turbo overspeed.
- Real gearbox data, which Sebastian supplies locally.
- Brand-name car models: decision pending.
- Road crossings (merged in geo2map #1 and racing_game #4) have unanswered owner points. Ask them in a PR, not chat:
  - hide `service=parking_aisle`/`driveway` ways (example way 494430548, Claude recommended hiding);
  - Sebastian's remark about edge lines "going into the middle of the 4-lane road" from the A66 into Unterliederbach, never located;
  - his last comment (geo2map #1, 2026-10-14 14:39Z) that the whole B8-to-A66 ramp junction (broken lines, gore) sits "too far back".
  - His owner spots (50.174871,8.462884; 50.177647,8.460454; 50.145426,8.459762; rest stop 49.974365,8.604079) and the A5 seams were never checked on the real store.

physics_sim (`docs/next_steps_brief.md`):
- R0 barrel rolls (needs a game repro);
- W7 aquaplaning, W8 snow/ice, W10 tyre temperature and wear, W11 environment sync;
- a supercharger model;
- P9 multi-vehicle budgets for NPC traffic;
- P1b DFCO, which moves hashes;
- N2O boost overshoot: 2.55 bar against the 1.8 bar target;
- an ARM64 Steam Runtime SDK build.

geo2map_engine (tracker `docs/manager.md` on #12):
- the home-store underpass census and the phase-7 probe, both waiting on Sebastian;
- the Königstein roads.geom tile the deriver rejects (`utm32n/2/452/5428`);
- long tunnels over several OSM ways;
- G3-E/F, P3 markings, G5 land cover, G7 hosted server (the HTTP v1 binding and the G7 list were accepted by Sebastian on geo2map #8), G8 worldwide;
- open question: should racing_game talk to `g2m_server` over HTTP (opt-in sidecar)?

## 10. Working rules Sebastian set (carry these forward)

- **Branches and PRs:**
  - Work on branches and draft PRs. Branches and PRs are allowed in any of the three repos, whoever "owns" it.
  - Sebastian reviews and merges; agents never merge into `main`/`master`.
- **Questions go in the PR**, not chat. Give a recommendation with each question.
- **PR descriptions:**
  - Open with "Before:" and "After:" paragraphs.
  - Add before/after images. Upload them to racing_game's orphan branch `PR-images`, one folder per topic, and link them as `https://github.com/SebastianDanielFrenz/racing_game/blob/PR-images/<path>?raw=true`.
  - To work on the orphan branch, use a separate worktree.
- **Code layout:** engine-neutral logic lives in `core/` (rg_core). `godot_ext` and GDScript only convert and draw. Limits and tuning numbers go in data files, not code.
- **Physical modelling:**
  - Mass comes from parts: a part tree, a bare chassis with its own mass, fuel counted live, factory turbos as their own parts.
  - Damage persists, repair is free but explicit, damage changes driving, and wear grows with how far a value is past its limit.
- **Roads:** must match satellite imagery and OSM, NPCs must be able to navigate them, and sign and light positions must make sense under German StVO.
- **Testing:** "Don't run tests unless really necessary". Run the scoped tests for what you touched.
- **Reports:** plain, short, metric units.
- **Subagent model tiers:** Haiku for clearly defined tasks, Sonnet for debugging, Opus for hard or sticky issues.
- **Git LFS:** car `.glb` models in `data/models/downloaded/` and `data/models/imported/` are LFS. Cloud sessions could not upload LFS objects; local commits can. Generated models from `tools/cargen` and `tools/carimport` are deterministic; regenerate them locally and commit them.

## 11. Where the history lives

- PR comments (#5 has 101) hold Sebastian's answers. The PR bodies are partly stale.
- The cloud project threads, which local agents cannot read; this folder summarises them:
  - "Work through the plan" (all of the plan branch);
  - "Improve road crossings" (geo2map #1, racing_game #4);
  - "Design three new cars" (#2).
- Images: racing_game branch `PR-images` (`plan-work/`, `road-crossings/`, `road-overpass/`, `underpass-mask/`, `visuals/`, `mobile-renderer/`, `lights-exposure/`).
