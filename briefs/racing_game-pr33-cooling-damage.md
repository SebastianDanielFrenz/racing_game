# racing_game #33: Damage model, first piece: radiator, fan and head gasket wear when the engine overheats

Each car keeps a health figure (0-100 %) for its radiator, radiator fan and head gasket. A part wears when the coolant stays well past the car's own overheat warning; worn parts cool less (radiator/gasket: less heat to the air, fan: less air), so the engine runs hotter and wears more. Damage persists per car (saved under the user dir, survives world reload / Change car / restart), a HUD line shows it (`DAMAGE: Radiator 62 % - repair in the garage`), and the garage's Engine > Cooling page has a free **Repair** button. This is the first piece of a larger damage model; Sebastian's review comment asks for much more (see sections 4 and 6).

## 1. State at hand-off

| Item | Value |
|---|---|
| Repo / PR | racing_game, https://github.com/SebastianDanielFrenz/racing_game/pull/33 |
| Head branch / sha | `claude/cooling-damage` @ `caf2453` (full caf24539423a20997b4b81aa7f615f94135907b6; verified at the end, see bottom) |
| Base branch | `claude/plan-work-q2a5oy` (a long-lived integration branch, not main). PR base sha 1345de5; the base head is now `07581be`, 5 commits ahead of this branch's merge-base `b4bb731`. GitHub says mergeable_state "clean", but rebase/merge the base in before continuing |
| State | DRAFT, 1 commit, 22 files, +1388 / -1 |
| physics_sim pin | `26a193d` = the FIRST commit of physics_sim PR #15 (branch `claude/cooling-capacity`). **Unmerged PR commit, not on physics master.** The base branch is on `f38075e` (on physics master). |
| geo2map_engine pin | `16a8ec6`, on geo2map main (the base branch is on `2571f43`, also on main; the difference is the base moving on, not this PR) |

**Merge order: physics_sim #15 first (into master), then re-pin `external/physics_sim` here to the merged master commit, then merge #33.** Do not merge #33 with the pin on 26a193d.

**Cloud work was stopped mid-way. What landed and what did not** (checked via `git log` of both branches and the PR bodies):
- racing_game #33 has exactly ONE commit (caf2453, 2026-10-16 14:29 UTC). Nothing was pushed after it. These items from the cloud session's "IN PROGRESS" list did **NOT land** in #33: harsher wear far past limits, oil-driven engine failure, repair menu item, Difficulty settings block / damage toggle. None of them exist on the branch (verified by file list: no settings schema, shell_flow or pause-menu changes; `damage.json` has only the three first-piece rules).
- physics_sim #15 DID get a later push (`4e81619`, 19:10 UTC) that adds `World::set_engine_wear` and the fan-power change. **#33 does not use it** (still pinned to 26a193d, and no oil/wear code on the racing_game side). See the physics_sim #15 brief.

## 2. What is done (all in caf2453)

- `core/include/rg/damage.h`, `core/src/damage.cpp` (engine-neutral): `DamageConfig` from `data/vehicles/damage.json` (`rg.damage/1`, strict loader), `DamageState` (part id -> health, stored in `<user dir>/garage/damage/<vehicle id>.json`, atomic write, healthy car = no file, unknown part ids preserved), `DamageModel` (one tick per physics tick), `cooling_scales`, `damage_view`, `damage_warning_line`.
- `core/src/session.cpp` / `session.h`: `SessionConfig::damage`; `setup_damage/update_damage/apply_cooling_scales` run on the stepping thread right after `World::step`, reading the engine thermal snapshot; factors go to physics via `World::set_engine_cooling_capacity`; `Session::damage_report()`, `request_damage_repair()`. A car brought in damaged drives damaged from tick 1.
- Garage: `Garage::damage_config/damage_state/damage_state_path/repair_damage`, `DriveSelection::damage_config_path/damage_state_path`.
- Godot bindings (conversion only, `godot_ext/src/rg_damage_convert.h`): `RgGarage.get_damage(id)`, `RgGarage.repair_damage(id)`, `RgSimulation.get_damage_state()`, `RgSimulation.repair_damage(state_path)`, `set_vehicle_overrides` keys `damage_config_path`/`damage_state_path`. `RgSimulation` saves a changed state on teardown, exit and every 5 s.
- UI: `hud.gd` one warning line; `shell_ui.gd` Damage panel + "Repair (free)" button on Engine > Cooling; `main.gd` wiring. Pause-menu garage repair fixes the running car at once.
- Rules in `data/vehicles/damage.json` (all [AS]): radiator (UA x0.25 at 0 %; worn by coolant > warning+4 K for 15 s), radiator_fan (fan flow x0; above warning, fan on, 30 s), head_gasket (UA x0.6; > warning+12 K for 5 s). Factor = `min_scale + (1-min_scale) * health`.
- Docs: `docs/damage_model.md`; one CLAUDE.md line.
- Tests (`[damage]`, `tests/unit/test_damage.cpp`, 9 cases): config load/reject, state round trip, wear (dwell, hotter = faster, no self-heal, clamp), repair; driving effect (radiator+fan at 10 %: coolant 122.7 C vs 106.5 C healthy after 20 s part throttle; identical at 1 and 4 workers); forced overheat in a Session; repair in running Session; garage paths. No-op guarantee: healthy car has the same state hash as no damage model; hash_check vehicle_step_steer stays `0xace4d38ea725e065`. PR reports full rg_unit_tests 751/751 on Linux.

## 3. Not done: Sebastian's requested next steps (ordered)

1. Re-pin physics_sim after #15 merges (and decide whether to take `set_engine_wear` from the same PR, section 7).
2. **Difficulty settings block** with a "mechanical damage" toggle (rg::Settings schema in `core/include/rg/settings.h`, shell_ui settings screen, applied in `main.gd _apply_setting`). Must be SEPARATE from a later collision-damage toggle (add only the mechanical one now, name keys so a `difficulty.collision_damage` can sit beside it). Off = model not run / nothing wears (and presumably existing damage has no effect; decide and document).
3. **Repair as a menu item** as well as the garage button (pause menu entry and/or garage first page; ShellFlow action + view-model, see `core/include/rg/shell_flow.h`). Repair stays free and an explicit player action.
4. **Wear scales with how far past the safe limit**: rate must rise steeply (not just linear per K); retune `damage.json` rates/`rate_per_s_per_k`.
5. **Oil-driven engine failure**: an engine pushed far beyond limits must break before oil reaches ~170 C. Reference from Sebastian: 2 MW on the hyper with 11 bar boost reached about 190 C oil. Acceptance: that scenario cannot get oil to 170 C within 3 minutes because the engine fails first. Needs a health-to-function translation for engine internals (oil temperature signal, an engine-wear part driving `World::set_engine_wear` friction, and a cascade that can stop the engine, see section 6).
6. More parts/sources later (section 6 list).
7. Cleanup: the setup-check coolant item still names `engine_block` as `damage_part`; question 3 below.

## 4. Open questions for Sebastian

The PR's four questions were ALL answered in his comment (one comment on #33, 2026-10-16 14:42 UTC, and one on physics_sim #15). Mapping, per his numbering (his reply has items 1, 2, 4; **no answer was given to the PR's question 3 (head_gasket vs engine_block naming in the setup check)** and the repair-button-on-first-page part of Q2 is only partly answered):

Answered (decisions, do not re-ask):
- Q1 wear speed: "pushing far in excess of safe values should run a part quicker; getting oil up to 170 C within 3 minutes should not be possible because the engine breaks first (2 MW on the hyper car with 11 bar boost got around 190 C)."
- Q2 repair placement: "Repair is a menu item and a garage feature. We also need a block 'difficulty', which contains a toggle for the damage model. This is SEPARATE from collision damage features that might exist later."
- Q4 ruined part: "Depends on the part. There needs to be a translation from health to functionality; an engine without a real working cooling solution should keep running, but then other parts will start failing and that might cause the engine to shut off."

Still unanswered:
- Q3: should the setup check's coolant item name `head_gasket` instead of `engine_block`? Recommendation (from the PR, implied): yes, since head_gasket is the part that wears. Low risk; do it unless he objects in the PR.
- Where exactly the Repair menu item goes (pause menu? main menu garage? both?). Recommendation: pause menu "Repair car" plus the garage button.
Post new questions in the PR, not in chat.

## 5. (see section 6)

## 6. Owner decisions and rules that apply beyond this PR

- Damage persists until repaired. Repairs are free. A repair needs an explicit player action (garage / menu item; later in races a long pit stop). Damage changes how the car drives (never just a cosmetic).
- Wear grows with how far past the safe limit the signal is.
- An engine pushed far beyond limits breaks before oil reaches ~170 C.
- Each part's health maps to its function. An engine with no working cooling keeps running, but other parts then fail, and that cascade can shut the engine off.
- A "Difficulty" settings block holds a mechanical-damage toggle; collision damage (possibly later) gets its own separate toggle.
- Planned future damage sources: knock, mean piston speed, charge-pipe boost, turbo overspeed, overheating parts (also in `docs/damage_model.md` "Adding a source later").
- Physics changes are allowed ("The owner allowed this physics change"); physics_sim is changed through its own PRs.
- PR conventions: questions in the PR, before/after images on the orphan branch `PR-images` (https://github.com/SebastianDanielFrenz/racing_game/blob/PR-images/<folder>/<file>?raw=true).

## 7. Known issues / risks

- Branch pinned to an unmerged physics commit (merge order above). The pin 26a193d is the old first commit; physics PR head is now 4e81619 (3 commits incl. a merge of master). When re-pinning, expect `hash_check` and all hashes to stay unchanged (the new physics only changes hashes when factors differ from 1); verify.
- Base branch moved 5 commits since this branch was cut; merge `claude/plan-work-q2a5oy` into this branch first. The geo2map pin differs between the two (16a8ec6 here vs 2571f43 on base); take the base's.
- All wear thresholds/rates/min_scale are invented [AS] values.
- The head gasket only costs cooling (no power loss; engine model lacks a seam). `set_engine_wear` (physics #15 head) is a possible seam for engine-internals wear.
- A normal drive rarely overheats (hypercar cooling was upsized in #28); only unit tests forced it. Real overheat never seen in a drive.
- Physics_sim has a separate in-progress branch `claude/block-cooling` touching the same thermal files (physics #15 says whichever lands second may need a rebase).

## 8. Local build, run, verify (Windows)

```
git fetch origin && git checkout claude/cooling-damage && git merge origin/claude/plan-work-q2a5oy
git -c protocol.file.allow=always submodule update --init --recursive
cmake --preset debug -DRG_BUILD_GODOT_EXTENSION=ON
cmake --build --preset debug
out\build\debug\tests\unit\rg_unit_tests.exe "[damage]"      # (path per your build tree) or: ctest --preset debug -R damage --output-on-failure
ctest --preset debug --output-on-failure                        # full suite, expect all green
tools\ci.ps1                                                    # debug+release+smoke legs
tools\smoke_test.ps1 -Garage                                    # garage flow incl. the Cooling page / Repair button
tools\smoke_test.ps1 -Drive                                     # needs the geo2map store (RG_G2M_HOME)
tools\run.ps1 -Flat                                             # then overheat the car (hold full throttle at standstill / low gear, small radiator setup) and watch the HUD DAMAGE line
```
Screenshots to take (before/after, upload to `PR-images/cooling-damage/`): garage Engine > Cooling Damage panel with a worn car, the HUD line while driving, panel after Repair. The tags that cover this: `[damage]`, `[thermal]`, `[engine_swap]`, `[vehicle_data]`. In physics_sim: `ps_unit_tests "[thermal]"` and `"[cooling_capacity]"`. Steps 3 and 5 of section 3 need a hypercar run with boost (no map store needed; flat world is enough).

## 9. Not verified in the cloud

- The GDExtension was NOT built or linked (Linux cloud has no Windows/godot-cpp build); changed `godot_ext` files were only `clang -fsyntax-only` checked against a Linux godot-cpp build (that caught one namespace error).
- Nothing was run in a Godot window; `hud.gd`, `shell_ui.gd`, `main.gd` got only a headless parse check.
- No real overheat observed in a drive; only unit tests force it.
- No Windows CI, no map store, no LFS models. Linux unit tests only (751/751 per the PR).

## 10. Files to read first

1. `docs/damage_model.md`
2. `data/vehicles/damage.json`
3. `core/include/rg/damage.h`, `core/src/damage.cpp`
4. `core/src/session.cpp` (grep `damage`)
5. `tests/unit/test_damage.cpp`
6. `core/include/rg/settings.h` (for the Difficulty block)
7. `core/include/rg/shell_flow.h` (for the repair menu item)
8. `game/scripts/shell_ui.gd` (Damage panel), `game/scripts/hud.gd`
9. physics_sim `core/include/ps/world/world.h` (`set_engine_cooling_capacity`, `set_engine_wear`)
