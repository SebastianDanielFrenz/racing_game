# racing_game #26: Automatic and dual-clutch gearboxes (garage swap, selector, HUD)

Adds automatic (torque converter), dual-clutch (DCT) and, since #30 merged into this branch, CVT gearboxes to the garage's gearbox swap. A player picks one in the Garage under "Automatic", "Dual-clutch" or "CVT"; the car loses its clutch pedal, gets an automatic TCU and a park lock. While driving, shift up / shift down (E / Q, pad B / X) step the selector P-R-N-D one position per press; Y toggles manual mode (tiptronic gears, cluster shows M<gear>); the cluster and tach show P R N D S or M + gear. Manual cars are unchanged. It answers "Still no automatic transmissions?" and "Double clutch, too, pls" on #5. Physics side is physics_sim #12 and #13 (both merged).

## State at hand-off

| Item | Value |
|---|---|
| Repo | SebastianDanielFrenz/racing_game |
| PR | https://github.com/SebastianDanielFrenz/racing_game/pull/26 |
| Head branch | claude/automatic-gearbox |
| Base branch | claude/plan-work-q2a5oy (the integration branch, not master) |
| Head sha | 68e3da4 |
| Status | DRAFT; mergeable_state "clean" (conflicts were resolved in the last merge, owner had asked "Resolve conflicts") |
| Size | 32 files, +1424 / -68, 12 commits |
| physics_sim pin | ad401bc: on physics_sim origin/master (verified in the cloud clone). Base branch pins f38075e, so this PR bumps it |
| geo2map_engine pin | 2571f43: on geo2map_engine origin/main; same as the base branch |

## What is done

- Pin bump to physics_sim master ad401bc (automatic/DCT gearboxes: torque converter, clutch-to-clutch shifts, `automatic_tcu`, CVT type, DCT launch control, contents mass, TC fix).
- Catalogue `data/gearboxes/gearbox_catalog.json` (`core/include/rg/gearbox_catalog.h`, `core/src/gearbox_catalog.cpp`): `auto_8at` (400 N*m) and `heavy_8at` (750, new `heavy_8at.json`) in Automatic; `dct_7` (350) and `hyper_7dct` (1600, new `hyper_7dct.json`) in a new Dual-clutch section; CVT rows `cvt_converter`, `cvt_clutch` (250) in CVT (from #30, which has an engine-derived ratio law). All six new rows carry `mass_kg`/`mass_source` ([AS]) because the part-mass tree requires them.
- Swap behaviour (`core/src/vehicle_setup.cpp`, `garage.cpp`): removes the clutch component, converts `manual_tcu` to `automatic_tcu` (schedule normal/sport, kickdown, manual-mode upshift rpm from the engine's idle/limiter via `derive_automatic_tcu_points`), fits a park lock if missing, drops `drive_modes`. Swap lines read "Clutch removed" / "Park lock fitted". Clutch-capacity and shift-assist options are dropped with "The gearbox does not offer: ... (back to stock)" and no longer offered. `gearbox_install` uses the same rating filter as manual boxes.
- Selector logic (`core/include/rg/driver_actions.h`, `core/src/driver_actions.cpp`): `rg::selector_shift`; P/R refused while rolling faster than 1 m/s (HUD message, no change); D<->N allowed at speed; shift up in D does nothing. `manual_mode` action (Y), mode shown from the physics snapshot, not a game-side latch. Actions `select_drive/reverse/neutral` removed; `transmission_mode` (D/S) kept with no default key. P (`park`) keeps its key on all cars (= select P on an automatic). Controls schema now 50 actions (`data/controls/actions.json`, `defaults/keyboard.json`, `docs/controls.md`).
- Godot side (not compiled, see below): `godot_ext/src/rg_driver_actions.*`, `rg_simulation.cpp`, `game/scripts/main.gd`, `input_map.gd`, `driver_state.gd`, `digital_cluster.gd`, `tach_gauge.gd`, `shell_ui.gd`, `garage_test.gd`; docs `docs/garage_ui.md`.
- Tests: `[automatic][driver_actions]` (selector sequence, refusals while rolling, car without park lock, manual toggle, removed actions), `tests/unit/test_automatic_gearbox_swap.cpp`, `test_gearbox_swap.cpp`, `test_controls.cpp` (expects 50 actions, Y = manual_mode).

## What is unfinished / next steps

1. On Windows, build with the Godot extension and run the smoke tests below; fix compile/GDScript errors (none of the Godot side has been compiled).
2. Verify in a real drive: selector stepping with E/Q, Y manual mode entry/exit, cluster/tach letters, P/R refusal message while rolling.
3. Verify the physics assumption that manual mode is entered from one up plus one down count in the same tick (read from physics code only).
4. Choose a pad button for manual mode (currently none; see questions).
5. Replace [AS] gearbox data with the owner's real boxes (to be done locally by Sebastian/agents, web access was limited in cloud).
6. PR body still describes the pin as done; the last cloud comment said the body would be updated after tests/hash check pass (the body now reads ad401bc, so that appears done; inferred).
7. Mark ready for review once the Windows checks pass; the PR is a draft.

## Open questions for Sebastian (unanswered)

1. Data: ratios, ratings, masses, converter K-factors and shift-clutch numbers are all [AS]. Do you have real boxes to model instead? (He said in a comment he does, to be done locally; no concrete data given yet.)
2. Pad: which pad button should toggle manual mode? Recommendation: none chosen in the PR; pick an unused button (inferred, not proposed in the PR).

### Already answered (decisions, do not re-ask)

- CVT as a follow-up PR: "yes" -> built as #30 and merged into this branch (he later asked to verify; it was confirmed).
- Real gearboxes: he has some in mind, will be done locally, cloud web access too limited.
- Selector design: "Rather upshift/downshift binds only. P->R->N->D. Manual shifting possible using a key bind (you proposed Y)."
- "Resolve conflicts" (last comment, 2026-10-16 14:46): done by the 68e3da4 merge per the PR body.

## Owner decisions and rules that apply beyond this PR

- Automatic/DCT cars use the existing shift up/down actions for the P-R-N-D lever; no separate P/R/N/D binds; manual mode by a dedicated key.
- Logic belongs in rg_core (`selector_shift`, `DriverActions`); GDScript only forwards (repo engine-neutral rule).
- Physics behaviours the HUD shows come from the physics snapshot, not game-side latches.

## Known issues / risks

- Godot/GDExtension code and all six GDScript files are unverified.
- No pad binding for manual mode.
- Manual-mode entry timing relies on physics behaviour not run.
- All gearbox numbers are assumptions with unverified reasoning in each file's `source`.
- Merge order: base is claude/plan-work-q2a5oy; the branch bumps physics_sim to ad401bc, so anything else touching that pin on the base will conflict. #30 (CVT) is already inside this branch; do not merge it separately.
- Cloud test counts: full `rg_unit_tests` 759 cases / 151621 assertions pass on linux-release (pin ad401bc + plan-work merge 68e3da4); `hash_check` unchanged `state_hash=0xace4d38ea725e065`.

## Local build, run and verify on Windows

```
git checkout claude/automatic-gearbox
git -c protocol.file.allow=always submodule update --init --recursive   # physics_sim ad401bc
cmake --preset debug -DRG_BUILD_GODOT_EXTENSION=ON
cmake --build --preset debug
ctest --preset debug --output-on-failure
out\build\debug\rg_unit_tests "[automatic]"      # also "[driver_actions]" "[gearbox_swap]" "[controls]"
tools\smoke_test.ps1 -Garage        # garage flow, gearbox swap UI (no map store needed)
tools\smoke_test.ps1 -Shell
tools\smoke_test.ps1 -Drive         # needs RG_G2M_HOME map store
tools\run.ps1 -Flat                 # flat world: pick auto_8at / dct_7 / CVT in Garage, drive
tools\ci.ps1                        # full debug+release+smoke
```
Manual check: in the Garage, swap in `auto_8at`, `dct_7`, `cvt_converter`; confirm the swap notices; drive: E/Q steps P-R-N-D, try P/R while rolling (refused + message), Y in D, E/Q tiptronic, Y back. Take before/after screenshots of the cluster in P/R/N/D/S/M3 and the garage Gearbox picker with the new sections; upload to the PR-images branch (folder `automatic-gearbox`). The car models need LFS; the real world needs the map store.

## What was NOT verified in the cloud

Everything Godot: godot-cpp not built on Linux, `godot --check-only` only complains about missing extension classes. Garage smoke, Windows build, rendering, real map data, LFS models. Per the PR, only core tests on Linux plus a flat-ground drive test (car_sedan: `auto_8at` reaches 6th / 158 km/h, `dct_7` 5th / 157 km/h, CVT rows ~123-126 km/h; creeps at idle) were run.

## Files to read first

1. `core/src/driver_actions.cpp` and `core/include/rg/driver_actions.h` (selector_shift, manual_mode)
2. `core/src/vehicle_setup.cpp` (automatic swap package)
3. `core/src/gearbox_catalog.cpp` / `gearbox_catalog.h`
4. `data/gearboxes/gearbox_catalog.json`
5. `tests/unit/test_automatic_gearbox_swap.cpp`
6. `game/scripts/main.gd`, `input_map.gd`, `digital_cluster.gd`
7. `godot_ext/src/rg_driver_actions.cpp`
8. `docs/garage_ui.md`, `docs/controls.md`
