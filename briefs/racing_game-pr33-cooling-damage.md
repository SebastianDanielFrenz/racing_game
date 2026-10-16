# racing_game #33: Damage model: cooling parts and engine internals wear, a worn-out engine seizes; repair entries; difficulty toggle

Each car keeps a health figure (0-100 %) for five parts:
- cooling: radiator, radiator fan, head gasket;
- engine internals: crank bearings ("crankshaft") and pistons/rings.

How the model behaves:
- A part wears when its signal runs past its safe value. The wear rate grows steeply with the excess.
- Each part maps health to function in its own way. Cooling parts lose radiator conductance or fan air. Internals raise engine friction, which means less power and more oil heat.
- Below 20 % health the internals seize. The engine shuts off and does not restart until the car is repaired.
- The cascade Sebastian asked for works: bad cooling, then hot coolant, then hot oil, then bearing/piston wear, then more friction, then seizure.
- Repair is free and always an explicit player action. It is offered in three places: pause menu "Repair car", a Repair button on the car browser, and the Engine > Cooling panel.
- Settings > Difficulty > Mechanical damage turns the whole model on or off.

## 1. State at hand-off

| Item | Value |
|---|---|
| Repo / PR | racing_game, https://github.com/SebastianDanielFrenz/racing_game/pull/33 |
| Head branch / sha | `claude/cooling-damage` @ `a80c395` (full a80c3957bfca2e5d11192ad6e0f89e0b2b5d0a27). Three commits on top of the base: caf2453 (first piece), 096b625 (merge of the base `claude/plan-work-q2a5oy` at 07581be), a80c395 (everything else) |
| Base branch | `claude/plan-work-q2a5oy` @ `07581be`, already merged into this branch. GitHub reports mergeable "clean" |
| State | DRAFT, 34 files, +2344 / -13. Core and tests are complete. The Godot side is written but was never built or run |
| physics_sim pin | `4e81619` = HEAD of physics_sim PR #15 (branch `claude/cooling-capacity`). **This is an unmerged PR commit, not on physics master.** It adds `World::set_engine_wear` and makes fan power scale with fan flow cubed |
| geo2map_engine pin | `16a8ec6`, on geo2map main. The base branch has `2571f43`, also on main. Take the base's pin when re-pinning |

**Merge order:**
1. Merge physics_sim #15 into master. See the #14/#15 conflict note in those briefs.
2. Re-pin `external/physics_sim` here to the merged master commit.
3. Re-run the tests.
4. Merge #33.

Do not merge #33 while the pin is on 4e81619.

## 2. What is done

### Core (engine-neutral)

- **`core/include/rg/damage.h`, `core/src/damage.cpp`:**
  - `DamageConfig` comes from `data/vehicles/damage.json` (schema `rg.damage/2`). Each wear rule is a curve from excess to health lost per second, extrapolated along its last slope.
  - `DamageState` is stored in `<user dir>/garage/damage/<vehicle id>.json`. A healthy car has no file.
  - Also here: `DamageModel`, `wear_rate_per_s`, `damage_scales` (radiator UA, fan flow, friction), `seized_part`.
  - Rule flags `only_while_fan_on` and `only_while_running`: a stopped engine does not wear its bearings.
- **`core/src/session.cpp`:**
  - Worn cooling goes to physics `World::set_engine_cooling_capacity`; worn internals go to `World::set_engine_wear`. Each seam is called only when its own factors change.
  - While a part is seized, `ignition` and `starter` are held at 0 before every step. The channel mirror is told, so the player's ignition setting survives a repair.
  - `set_damage_enabled` applies the toggle live.
  - `DamageReport` has `enabled`, `seized` and `seized_part`.
  - A car brought in damaged drives damaged from tick 1.
- **`rg::ShellFlow`:** the pause item `repair_car`, and the event `RepairChosen` on VehicleSelect / ChangeCar, both lead to the action `RepairCar{vehicle}`.
- **`rg::Settings`:** section Difficulty, key `difficulty.mechanical_damage` (default on).
  - Off: nothing wears and stored damage has no effect, but it stays on disk.
  - It is separate from any later collision damage. Name a future key `difficulty.collision_damage`.
- **Setup check naming:**
  - The coolant limit's damage part is now `head_gasket`. This was PR question 3; Sebastian did not answer it, and it was done as the PR recommended.
  - The oil item says `crankshaft`, which is now a real part.
  - `cooling_upgrades.json` keeps `engine_block` for block_cooling, because there it names that upgrade's own hardware.

### Godot (conversion and drawing only)

- **Bindings:**
  - `RgShell.repair_chosen(id)` and the `repair_car` action.
  - `RgSimulation.set_damage_enabled` / `is_damage_enabled`.
  - `RgSimulation.repair_damage("")` repairs the running car.
  - `get_damage_state` adds `enabled`, `seized` and `seized_part`.
  - `rg_damage_convert.h` adds `friction_scale`, `seized` and `seized_label`.
- **GDScript:**
  - main.gd: `_repair_car` and the setting handler.
  - car_browser_ui.gd: damage lines and a Repair button.
  - shell_ui.gd: signal forwarding.
  - shell_flow_test.gd: the pause list.
  - hud.gd lines: `DAMAGE: Radiator 62 % ... - repair: pause menu or garage` and `ENGINE SEIZED (Crank bearings) - repair the car: pause menu or garage`.

### Parts

All values are [AS] (assumed, not sourced):

| Part | Effect | At 0 % | Seizes below | Safe value |
|---|---|---|---|---|
| radiator | radiator UA | x0.25 | - | overheat warning + 4 K, 15 s dwell |
| radiator_fan | fan flow | x0 | - | overheat warning with the fan on, 30 s |
| head_gasket | radiator UA | x0.6 | - | overheat warning + 12 K, 5 s |
| crankshaft (crank bearings) | friction | x3 | 20 % | oil 135 C, engine running |
| pistons (pistons and rings) | friction | x2 | 20 % | oil 145 C, engine running |

### Docs

`docs/damage_model.md` was rewritten, and the CLAUDE.md `damage.h` line updated.

### Tests on a80c395 (Linux)

- Full `rg_unit_tests`: 759/759 pass (147,552 assertions).
- `hash_check` vehicle_step_steer: `0xace4d38ea725e065`, unchanged.
- New `tests/unit/test_damage_engine.cpp`:
  - Bearings at 50 % double the friction: 4882 vs 2801 rpm.
  - Sedan cascade (dead radiator and fan, part throttle): seizes at about 297 s, peak oil about 153 C. With working cooling the bearings stay above 95 %.
  - Hash, health and seize time are identical at 1 and 4 workers.
  - A seized engine does not restart; after a repair it starts.
  - Toggle off gives the same hash as no damage model at all.
  - Oil ramp.
- Updated: `test_damage.cpp`, `test_shell_flow.cpp`, `test_settings.cpp`, `test_cooling_upgrades.cpp`.

## 3. Calibration and the owner's 190 C case

Owner rule: an engine pushed far past its limits must break before the oil reaches 170 C. His reference: 2 MW on the hypercar with 11 bar boost reached about 190 C oil.

**The 190 C case was NOT reproduced on this build.**
- **Setup of the hidden `[.][damage][long]` runs** in `test_damage_engine.cpp`: 30 C ambient, full throttle from standstill on a 40 km strip, `turbo_boost_target` 11 / 1.8.
- **What happened:**
  - The turbos choke at 4.8-5.7 bar, so 11 bar is never reached.
  - With #28's upsized `hyper.json` the oil settles near 122 C.
- **Runs with no wear:**
  - car_hyper, twin_large: 1886 kW;
  - car_hyper, single_120mm: 2290 kW;
  - car_hyper_n2o with nitrous: 2266 kW.

**So the oil curves were calibrated on a prescribed oil ramp from 110 C instead.** The bearings seize at:
- 161 C at 0.44 K/s (the owner's rate);
- 166 C at 1 K/s;
- 169 C at 3 K/s.

These figures come before any friction feedback; in a drive, that feedback makes the seizure come sooner.

**Dead radiator and fan on the hypercar:**
- 30 s: the head gasket is gone.
- 90 s (oil 136.6 C): the bearings start wearing.
- 141.8 s (oil 164.3 C): the engine seizes.

**Suggested local follow-up:** try to reproduce Sebastian's 190 C setup in the real game and confirm the engine seizes first. If the exact setup or turbo is unclear, ask him in the PR.

## 4. Known gaps (from the PR)

- **A seized engine is not mechanically locked.** Ignition and starter are held off, but a car rolling in gear still turns the crank. A real lock needs a physics_sim seam (its own PR there).
- **Coolant temperatures are not physical after total cooling loss.** The coolant climbs past about 300 C, because physics has no boiling model.
- **The sedan's oil passes 135 C even with healthy cooling** when it is free-revved in neutral for minutes. That is why the cascade test runs at part throttle. Check whether this is plausible or a thermal-model issue.
- **Leftover calibration overrides.** `test_damage_engine.cpp` still reads `RG_CASCADE_*`, `RG_WEAR_THR` and `RG_DAMAGE_*`. They are harmless and can be removed.
- **Every threshold, curve and factor is an invented [AS] value.**
- **It depends on physics_sim #15 (unmerged),** which conflicts with physics_sim #14 on the shared thermal files.

## 5. Next steps (ordered)

1. Build and run the Godot side on Windows (section 7). Fix anything that does not compile or parse.
2. After physics_sim #15 merges:
   - re-pin `external/physics_sim` to master;
   - re-run `[damage]`, `[thermal]` and `[cooling_upgrades]`;
   - run `hash_check` and expect `0xace4d38ea725e065`.
3. Take before/after screenshots into `PR-images/cooling-damage/` and link them in the PR:
   - the HUD damage line;
   - the ENGINE SEIZED line;
   - the car browser's Repair button;
   - the pause menu's Repair car;
   - Settings > Difficulty.
4. Try the owner's 190 C scenario in the real game (section 3).
5. Later damage sources Sebastian named: knock, mean piston speed, charge-pipe boost, turbo overspeed, other overheating parts. See `docs/damage_model.md`, "Adding a source later".

## 6. Owner decisions (do not re-ask)

All four PR questions are answered or settled. Sebastian's comment of 2026-10-16 14:42 UTC decided:
- Wear grows with how far past the safe value the part runs.
- Oil cannot reach 170 C within 3 minutes, because the engine fails first.
- Repair is a menu item and a garage feature.
- A Difficulty block holds the mechanical-damage toggle, separate from collision damage.
- Health translates to function per part. A cooling failure cascades into other parts, and that can stop the engine.

Q3 (head_gasket naming) got no answer and was done as recommended.

Other rules that apply:
- Damage persists until repaired.
- Repairs are free and explicit.
- Damage must change how the car drives.
- Physics changes are allowed through physics_sim PRs.
- Questions go in the PR.

## 7. Local build, run, verify (Windows)

```
git fetch origin && git checkout claude/cooling-damage
git -c protocol.file.allow=always submodule update --init --recursive
cmake --preset debug -DRG_BUILD_GODOT_EXTENSION=ON
cmake --build --preset debug
ctest --preset debug -R damage --output-on-failure
ctest --preset debug --output-on-failure            # expect 759 green
tools\ci.ps1
tools\smoke_test.ps1 -Shell                          # pause menu now lists Repair car
tools\smoke_test.ps1 -Garage
tools\smoke_test.ps1 -CarBrowser
tools\smoke_test.ps1 -Drive                          # needs RG_G2M_HOME
tools\run.ps1 -Flat                                  # overheat: pull radiator/fan via setup or hold WOT; watch HUD; repair from pause menu
```
Hidden calibration runs: `rg_unit_tests "[.][damage][long]"` (slow).

## 8. Not verified in the cloud

- **The GDExtension was not built.**
  - `rg_simulation.cpp`, `rg_simulation_shell.cpp` and `rg_shell.cpp` were only checked with `clang -fsyntax-only` against a Linux godot-cpp build.
  - `rg_garage.cpp` was not checked that way; this PR does not change it.
- **The GDScript changes were not parsed or run in Godot:** main.gd, car_browser_ui.gd, shell_ui.gd, hud.gd, shell_flow_test.gd. The headless shell, car-browser and garage tests were not run.
- **Nothing else ran either:** no Windows CI, no map store, no LFS models, no screenshots.

## 9. Files to read first

1. `docs/damage_model.md`
2. `data/vehicles/damage.json`
3. `core/include/rg/damage.h`, `core/src/damage.cpp`
4. `core/src/session.cpp` (grep `damage`, `seized`)
5. `tests/unit/test_damage_engine.cpp`, `tests/unit/test_damage.cpp`
6. `core/src/shell_flow.cpp` (RepairCar), `core/src/settings.cpp` (Difficulty)
7. `game/scripts/main.gd` (`_repair_car`), `car_browser_ui.gd`, `hud.gd`
8. physics_sim `core/include/ps/world/world.h` (`set_engine_cooling_capacity`, `set_engine_wear`)
