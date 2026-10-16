# racing_game #22: Garage: per-part mass tree (cumulative masses, layout-swap penalty)

A car's mass is now the sum of its parts instead of one flat number. A car is a tree: a bare-shell body at the root, with engine, drivetrain, running gear, fuel system, electronics and interior/panels groups beneath it. Each part has its own mass, and the parent's cumulative mass includes its children. Fuel and N2O count in the mass live, so the car gets lighter as they are used. The garage has a new Chassis -> Weight and a Weight reduction tile (11 options: lighter seats, bonnet, trim, fasteners, or removing seats, audio, carpets, spare wheel and so on). The car browser and preset stats show the mass of the parts with the preset's setup applied. A layout swap (FWD/RWD/AWD) stays allowed but adds a weight penalty. Stock totals changed: sedan 1500 -> 1365 kg, hyper 1500 -> 1265 kg. All new masses are [AS] (estimated, not cited).

## State at hand-off

| Item | Value |
|---|---|
| Repo | racing_game |
| PR | https://github.com/SebastianDanielFrenz/racing_game/pull/22 |
| Head | `claude/part-mass`, sha `88896b3` |
| Base | `claude/plan-work-q2a5oy` (not stacked on another PR branch) |
| Stacked on this | #29 (base `claude/part-mass`), hides weight-reduced parts visually |
| Draft / ready | ready (not draft), open |
| Mergeable | `clean` per GitHub at last read (owner commented "Resolve merge conflicts" four times; the latest head `88896b3` is a merge of plan-work, so the conflicts were resolved by it; re-check on your side) |
| Size | 39 files, +934 / -90, 9 commits |
| physics_sim pin | `f38075e`, on physics_sim master (merged; contains #10 contents mass, #11 TC fix, #12 automatic/DCT gearboxes) |
| geo2map_engine pin | `2571f43`, on geo2map main; same as the base branch |

The base branch has the same two pins, so this PR does not change the submodules relative to its base.

## What is done

- `core/include/rg/part_mass.h` and `core/src/part_mass.cpp` (engine-neutral): the part tree, cumulative mass, layout-swap penalty. Built from data only:
  - the vehicle file as the setup leaves it
  - engine and gearbox catalogue `mass_kg`
  - `data/vehicles/part_masses.json` (every figure [AS]/[D] with reasoning)
  - catalog per-car `chassis.body_mass_kg` (+ `body_mass_source`, itemised) and `interior` map
- Physics chassis mass = tree total minus what physics carries itself (tank and bottle contents).
- Live fuel/N2O: every game car file turns on physics_sim `contents_mass`. The body updates every 0.1 kg. Spawn fill is a full tank (tank `initial_kg`, default capacity) and the bottle's `initial_kg`, else a full bottle.
- Factory turbos are separate nodes under `engine.turbo_system` (each turbo, hot side, intercooler, piping). Hyper V8 is 175 kg NA in `engine_catalog.json`, twin-turbo system 54.2 kg; NA conversion drops those 54 kg. A `turbo_kit` extra from #21 replaces the group (no double count).
- Body mass is assigned per car, not back-calculated (owner's rule, see below).
- Weight reduction: new option kind `part_choice`, 11 options in `data/vehicles/setup_options.json` (group "Weight reduction"). Mass only. UI in `game/scripts/shell_ui.gd`; core in `core/src/vehicle_setup.cpp`.
- `Garage::stats` (car browser and catalog stats) uses the tree with the effective setup. Example: `car_sedan_rwd_track` uses a sport bucket seat and drops rear seats, audio, sound deadening, spare wheel.
- Layout-swap penalty: 15 kg mounting hardware for any layout change, +18 kg transfer drive when the result is 2WD, +14 kg driveline along the car where needed. FWD->RWD +45, RWD->FWD +35, FWD->AWD +87 kg.
- Bindings and UI: `RgGarage.get_mass_breakdown(id)` (`godot_ext/src/rg_garage.cpp`), Chassis -> Weight tile.
- Merge fallout already fixed: `car_city_bigbay` keeps the 425 kg shell (888.6 kg dry chassis) with the `interior` map and trim_* options; `car_runabout` carries both trim_* and cooling_* options; engine-swap mass-delta test follows the `sedan_i4` alias (105 - 175 kg).
- Totals (old -> new, full tank): hyper 1500 -> 1265, hyper_n2o 1500 -> 1284, sedan/toyama_xr 1500 -> 1365, sedan_rwd 1303, sedan_awd 1357, sedan_gen_n2o 1326, generic_suv 1511, city/city_bigbay 852 -> 909, gt70/phoenix_455 1282 -> 1387, runabout 644 -> 732. Bare shells: hyper 600, sedans 680, suv 820, city 425, gt70 665, runabout 215.
- Tests: `tests/unit/test_part_mass.cpp` [part_mass] (per-class plausible total range, body 25-55 % of total, leaves sum to total, chassis = total - carried, catalog mass equals dry chassis, fuel contents node, contents/turbo/interior cases, hyper turbos and NA conversion, weight-reduction choices, preset stats mass, garage view). Updated expectations in `test_engine_garage`, `test_vehicle_catalog`, `test_garage` (sedan dry chassis 1329.4 kg), `test_vehicle_setup`, `test_turbo_kit`. Docs: `docs/garage_ui.md`.

## What is unfinished / next steps

1. Build on Windows (`tools\ci.ps1` or the presets below) and fix any compile fallout in `godot_ext` (not compiled in the cloud).
2. Run `tools\smoke_test.ps1 -Garage` and look at the Chassis -> Weight and Weight reduction screens (not seen rendered).
3. Re-check mergeability against `claude/plan-work-q2a5oy`; the owner asked to resolve conflicts repeatedly, so confirm the base has not moved again.
4. Cars with `contents_mass` on but no catalog entry (so no `interior` map): `car_ace_11`, `car_fairheaven_lt80`, `car_kiri_86`, `car_lct_3000`, `car_shvan_92`. Add catalog entries or interior maps if they become selectable.
5. Visuals of removed parts are #29's job (separate stacked PR). Merge order: #22 first, then #29.
6. Possibly add more strippable slots if the owner answers question 3 below (glass, A/C, wiring).

## Open questions for Sebastian

From the PR body (still no answer on the PR in the comments I read):
1. Spawn fill: is a full tank right, or a partial tank (e.g. half)? Recommendation: full tank (current default; the owner replied "count fuel" but did not pick a fill level).
2. Bare-shell masses plausible? They move stock totals (sedan 1500 -> 1365, hyper 1500 -> 1265). Recommendation: accept as [AS] for now; owner said values may change later.
3. Other strippable slots (glass, A/C, wiring)? Today they are inside the bare shell. Recommendation: keep as is until asked.

Answered by the owner (decisions, do not re-ask), reply to the earlier numbered questions:
- Count fuel in the mass, live or semi-live.
- Engine mass is always a base plus turbo mass if the car has factory turbos.
- Layout-penalty sizes: "Fine for now, values might change later."
- Every car's mass is the sum of its parts, presets included.
- Weight reduction means removing interior parts (seats, audio, door panels, front panels) or swapping in lighter variants (screws, hood and so on).

## Owner decisions and rules from review comments

- "A general mass per part system where cumulative mass is calculated... Each parent can have a mass itself and then inherits the additional mass of its child components." (on #19)
- "Stop calculating backwards from a body mass. Instead, assign a value to the mass of the chassis without anything added (interior maximally ripped out, no engine assembly, no drivetrain, no wheels)." Applies to any future car: give it an independent bare-shell mass with itemised source.
- Presets get their mass from their parts, not from a stored figure.
- "Create a new PR that visually removes elements that were removed for weight reduction": done as draft #29.
- Layout swaps stay allowed, with a penalty (earlier decision on #19).

## Known issues / risks

- Every new mass is [AS], not cited.
- A removed part still shows on the car in this PR (visuals are in #29).
- Pico touchscreen and wheel buttons / Ventura radio hiding live in #29; seats and door cards there need a regenerated glb (cargen change, no glb committed; LFS objects can only be pushed from Sebastian's checkout).
- Cloud result: full `rg_unit_tests` (Linux release) passed 751 cases / 144931 assertions after the latest merge; one unnamed case failed once in an earlier run of the same binary and passed on re-run, not reproduced. Watch for flakiness.
- `hash_check` unchanged: `state_hash=0xace4d38ea725e065`.
- Changing the chassis mass of every car changes handling and lap behaviour everywhere; check hash-pinned tests that use physics_sim's own car files (they should be unaffected, inferred from the PR saying hash_check is unchanged).

## Local build, run and verify on Windows

```
git -c protocol.file.allow=always submodule update --init --recursive
cmake --preset debug -DRG_BUILD_GODOT_EXTENSION=ON
cmake --build --preset debug
ctest --preset debug --output-on-failure
out\build\debug\...\rg_unit_tests "[part_mass]"
rg_unit_tests "[engine_garage]"   # also: test_vehicle_catalog, test_garage, test_vehicle_setup, test_turbo_kit
tools\smoke_test.ps1 -Garage
tools\ci.ps1
tools\run.ps1                      # open the Garage, Chassis -> Weight and Weight reduction
```
(Test binary path and exact tag names for the secondary files are inferred; check the test source for the tags.)
- Screenshots worth taking: Chassis -> Weight breakdown and Weight reduction tile for car_sedan and car_hyper; car browser stats for `car_sedan_rwd_track` before/after. Upload to the `PR-images` orphan branch, e.g. `PR-images/part-mass/<name>.png`, link as `https://github.com/SebastianDanielFrenz/racing_game/blob/PR-images/part-mass/<name>.png?raw=true`.
- Drive check: spawn with a full tank, drive a while, confirm the chassis mass drops (HUD or `RgSimulation` mass readouts) in 0.1 kg steps.
- No step needs the map store or LFS models, except real-world driving (`RG_G2M_HOME`).

## What was NOT verified in the cloud

- No Windows build, `godot_ext` was not compiled, no Godot rendering.
- `smoke_test -Garage` and the Weight / Weight reduction screens were not run. `shell_ui.gd` only passed a headless `--check-only` parse.
- No real map store, no LFS models.
- Live fuel behaviour in a real drive not observed (unit tests only).

## Files to read first

1. `core/include/rg/part_mass.h`
2. `core/src/part_mass.cpp`
3. `data/vehicles/part_masses.json`
4. `data/vehicles/catalog.json` (`chassis.body_mass_kg`, `body_mass_source`, `interior`)
5. `data/vehicles/setup_options.json` (the `part_choice` weight-reduction options)
6. `core/src/vehicle_setup.cpp`
7. `core/src/garage.cpp`
8. `tests/unit/test_part_mass.cpp`
9. `game/scripts/shell_ui.gd` (Weight tiles)
10. `docs/garage_ui.md`
