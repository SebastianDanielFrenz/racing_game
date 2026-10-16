# physics_sim #14: Engine thermal: water jacket, pump upgrade and a steady-state solver

The engine thermal model used to be one lump (block+head+coolant at one temperature) plus an oil lump. This PR adds an optional `water_jacket` block to `physics_sim.engine_thermal/1`: the engine splits into a metal lump and a coolant lump joined by a conductance G = A * max(h_min, h_ref*(Q/Q_ref)^n). Heat sources (wall heat, friction share, block-to-air, oil exchange) act on the metal; radiator/thermostat/fan/gauge/warnings act on the coolant. The metal runs ~5 K above coolant at cruise and ~10 K at full load. A bigger wetted area, better film coefficient or faster pump lowers metal and oil temperatures, so a "block cooling" upgrade now has something to change. A cooling system can carry `pump.flow_factor` (pump upgrade). New `ps::drivetrain::solve_steady_thermal` returns settled temperatures without a time history. Metal temperature is published (snapshot, CSV, Godot dictionary, demo HUD line). Gauges/warnings still read coolant. Every shipped engine gets a jacket via the class file.

## State at hand-off

| | |
|---|---|
| Repo | SebastianDanielFrenz/physics_sim |
| PR | https://github.com/SebastianDanielFrenz/physics_sim/pull/14 |
| Head branch | claude/block-cooling (sha 97e4a85) |
| Base | master (c08b00c = merge of #16). Not stacked. Merge commit 894721c already merged origin/master incl. #16 |
| Draft/ready | Draft |
| Mergeable | clean (GitHub mergeable_state) |
| Size | 21 files, +1162 / -67, 3 commits |
| Pins in racing_game | There is no racing_game branch for this PR. racing_game HEAD (5e3ca18) pins physics_sim 1e07c0e (is an ancestor of physics_sim master, i.e. on master; master has 43 commits beyond it, inferred from `git log 1e07c0e..origin/master`) and geo2map_engine 3092d66 (not checked against geo2map main) |

## What is done
- Core: `core/include/ps/drivetrain/engine_thermal_desc.h` (jacket + pump fields), `engine_thermal_steady.h` (new `solve_steady_thermal`: rpm, wall heat, friction heat, ambient, ram speed, thermostat offset -> coolant/metal/oil temps, thermostat opening, fan, conductances, heat flows; works without a jacket), `core/src/drivetrain/engine_thermal.cpp/.h` (three-lump backward Euler; metal in the state hash only when a jacket exists), `core/src/io/engine_thermal_io.cpp` (loader; rejects a jacket leaving metal < 10 % of the old engine capacity; `pump.flow_factor` in (0,4]), `core/src/world/world.cpp`, `world.h`.
- Jacket keys: `wetted_area_m2`(`_per_l`), `coolant_volume_l`(`_per_l`), `reference_flow_l_min`(`_per_l`), `htc_ref_w_m2_k`, `htc_min_w_m2_k`, `flow_exponent` (default 0.8). `"water_jacket": null` or absent = old two-lump behaviour, bit-identical.
- Published: snapshot `metal_c`, `jacket_heat_w`, `jacket_conductance_w_k`, `coolant_flow_l_min`; Godot `get_engine_thermal` gains `metal_c`, `jacket_heat_w`, `coolant_flow_l_min` (`adapters/godot/src/ps_simulation.*`); CSV `vehicle<i>.thermal.metal_c/.jacket_heat_w/.coolant_flow_l_min` only for jacketed engines (jacket-less header unchanged); demo `hud.gd` line shows metal temp.
- Data: `data/engine_thermal/class_si_passenger.json` carries the jacket (0.2 m2/L, 3.0 L/L coolant, 5000 W/m2K at 30 l/min per litre, exponent 0.8, floor 500; all [AS]); `i4_2l_na.json` and `toyota_a25a_fks.json` extend it. Fixture `tests/support/fixtures/thermal/i4_2l_na_no_jacket.json`.
- Docs: `docs/data_formats/engine_thermal.md` ("Water jacket" section, measured tables), `cooling_system.md` (pump row), CLAUDE.md.
- Tests: `tests/unit/test_engine_thermal_jacket.cpp` tag `[thermal]` (J1 loader, J2 h(Q)/G monotonicity, J3 energy balance 1e-9, J4 solver vs 4 h transient within 0.02 K, J5 neutral keys bit-identical, J6 World hash 1 vs 10 workers + no allocations, J7 every shipped engine has a jacket and CSV channels; hidden `[.][diag]` measurement case; three recorded sabotages). `test_engine_thermal.cpp` U3 now turns the jacket off explicitly.
- Hashes: `vehicle_thermal` 0xfb37e62eaf06cabb -> **0x79bdd9ac60402967** (A25A now jacketed), recorded in `tests/determinism/test_worker_count_hash.cpp`. `vehicle_step_steer` stays 0xe77fd6569d82786c; all other 21 scenarios unchanged.
- Measured (PR body): 2.0 L I4, 3000 rpm/12 kW/3 kW friction/25 m/s: no jacket 88.55/88.55/85.44 (coolant/metal/oil), jacket 88.54/93.67/89.24, area x2 metal 91.15, pump x1.5 metal 92.11. 5000 rpm/35 kW: metal 101.63 vs 89.82 without jacket. A25A 6000 rpm/45 kW: metal 99.05.

## What is unfinished / next steps
1. PR is draft; Sebastian to review and mark ready/merge (no open review threads or reviews; none requested).
2. Run the Windows legs and Godot adapter build locally (see section 8) - never run in the cloud.
3. Merge to physics_sim master. Then the racing_game adoption (below).

### Adopting in racing_game after merge (inferred from reading racing_game; the PR does not describe it)
- Pin bump: `external/physics_sim` -> the PR's merge commit on master (use `git -c protocol.file.allow=always submodule update` per CLAUDE.md). Pin must also be consistent with geo2map `bridges/physics_sim/physics_sim.pin` (CLAUDE.md says the two pins must match; check whether geo2map needs a bump too). The current pin 1e07c0e is 43 commits behind master, so the bump brings in more than this PR - re-run the full suite for those changes too.
- Data: all five racing_game thermal files (`data/engine_thermal/{n52b30,hyper_v8_one1,pico_i3_1l0_na,runabout_i4_2l9_sv,ventura_v8_5l0_na}.json`) `extends` physics_sim's `class_si_passenger.json`, so every game engine gets the jacket automatically, no file change required. Their `capacity_per_l.engine_j_k` (26600..42000) leaves the metal lump > 10 % (coolant part is 3.0 L/L*1065*3400 = ~10.9 kJ/K per L, inferred arithmetic), so the loader check should pass; confirm with `test_game_car_thermal` `[thermal][vehicle_data]`. Game cars use the cooling systems `data/cooling_systems/hyper.json` and physics_sim `sedan.json`; neither needs `pump.flow_factor` unless a pump upgrade is wanted.
- Behaviour change: metal/oil temperatures rise a few K to ~10 K at load; the hypercar thermal measurements in `data/cooling_systems/hyper.json` `source` and `hyper_v8_one1.json` (wall_heat_fraction pinned 0.20) and the `[.][diag][thermal_run]` case (`RG_DIAG_CAR/MODE/...`) in `tests/unit/test_game_car_thermal.cpp` should be re-measured; any recorded temperature thresholds may shift. Inferred; not tested.
- Per-class jacket override per engine (e.g. hyper V8 bigger area) is optional: add a `water_jacket` block in the racing_game engine thermal file.
- Garage option for "block cooling"/pump upgrade (the pending racing_game plan step "Adopt the water jacket in racing_game once #14 merges"): add setup_options.json entries (JSON-pointer paths into the thermal file `/water_jacket/wetted_area_m2_per_l`, `/htc_ref_w_m2_k` and the cooling system `/pump/flow_factor`, range (0,4]) in `data/vehicles/setup_options.json`; the thermal file must be reachable by the merge-patch materialiser (`core/include/rg/vehicle_setup.h` patches the vehicle/engine/gearbox/tyre files - check whether engine_thermal/cooling_system files are in its set). Not examined in detail.
- Telemetry/HUD: `RgSimulation` get_engine_thermal passthrough (`godot_ext/src/rg_simulation.cpp`) and `game/scripts/hud.gd` could show `metal_c`; the owner ruled gauges/warnings stay on coolant.
- Update CLAUDE.md pin line and any racing_game hash pins that depend on thermal state (tests that hash physics_sim scenarios still point at physics_sim's own files; `vehicle_step_steer` unchanged). Check `hash_check` still prints the same hash.

## Open questions for Sebastian
None unanswered in the PR. (Inferred open point worth asking in the racing_game PR, not here: whether to expose a garage upgrade for jacket area / pump flow.)

Already answered (decisions, do not re-ask), comment 2026-10-16 by Sebastian:
1. "Should a shipped engine get a jacket?" -> "yes, every engine gets one." (implemented in the class file, test J7)
2. "Should there be a metal-temperature warning?" -> "publish the temperature, but do not modify gauges." (published; gauge, overheat/cold warnings, thermostat and fan unchanged)

## Owner decisions and rules that apply beyond this PR
- Every engine thermal model carries a water jacket by default (class-level).
- New physical channels are published to snapshot/CSV/Godot/HUD text but gauges and warnings are not changed unless asked.
- Jacket-less user files must keep behaving exactly as before (bit-identical hashes).

## Known issues / risks
- All jacket numbers are [AS] assumptions (0.8 exponent = Dittus-Boelter); one class value for both shipped engines; nothing calibrated.
- Thermostat (2 K play, 20 s wax lag) hunts in a limit cycle at small openings (pre-existing); with a jacket `solve_steady_thermal` gives the centre of the hunting, not a time average. J4 uses a proportional thermostat for the partial-open case.
- `World::engine_thermal_state` still returns coolant and oil only; `set_engine_thermal_state` sets metal = coolant. Read metal from the snapshot.
- 8 pre-existing `[!mayfail]` failures in `ps_unit_tests "~[long]"` (1124/1132 pass).
- `vehicle_thermal` hash changed on purpose (see above); any consumer pinning it must update.
- No shipped `data/vehicles` car installs a cooling system in physics_sim itself, so the effect only shows in `*_thermal` fixtures and racing_game cars.

## Local build, run and verify on Windows
physics_sim (from `S:\claude_code\physics_sim`, check its CLAUDE.md/`tools\ci.ps1` for the exact presets; commands below are inferred from the PR text and racing_game conventions):
```
git fetch origin; git checkout claude/block-cooling
tools\ci.ps1                      # Windows legs incl. determinism
# or targeted:
ps_unit_tests "[thermal]"         # expect 42 cases
ps_determinism_tests              # expect 21 cases, vehicle_thermal 0x79bdd9ac60402967 at 1/4/10 workers
ps_run data\scenarios\vehicle_step_steer.json   # 0xe77fd6569d82786c
ps_unit_tests "[.][diag]"  # prints the jacket comparison tables
```
Godot adapter: build `adapters/godot` and run its demo; check the HUD text line shows metal next to coolant/oil (needs a vehicle with a cooling system, e.g. a `*_thermal` fixture) and `get_engine_thermal` has `metal_c`.
racing_game after the pin bump: `cmake --preset debug`, `cmake --build --preset debug`, `rg_unit_tests "[thermal]"`, `rg_unit_tests "[vehicle_data]"`, `rg_unit_tests "[engine_swap]"`, `hash_check`, then `tools\ci.ps1`; `tools\smoke_test.ps1 -Garage` (garage loads vehicle thermal files). The hidden `[.][diag][thermal_run]` case (RG_DIAG_CAR=car_hyper, RG_DIAG_MODE) gives before/after hypercar temperatures. No map store (RG_G2M_HOME) or LFS models needed for these.

## What was NOT verified in the cloud
- Windows legs, `tools/ci.ps1`, the Godot adapter build and demo HUD were not run (stated in the PR).
- Only linux-release was run: `[thermal]` 42 pass; determinism 21 pass; `~[long]` 1124/1132 (8 existing mayfail); all `ps_run` scenarios match recorded hashes.
- Real-engine calibration of jacket values: none.
- racing_game adoption: not attempted; my adoption notes are inferred from reading racing_game files.

## Files to read first
1. `docs/data_formats/engine_thermal.md` (Water jacket section) in physics_sim
2. `core/include/ps/drivetrain/engine_thermal_desc.h`
3. `core/include/ps/drivetrain/engine_thermal_steady.h`
4. `core/src/drivetrain/engine_thermal.cpp`
5. `core/src/io/engine_thermal_io.cpp`
6. `data/engine_thermal/class_si_passenger.json`
7. `tests/unit/test_engine_thermal_jacket.cpp`
8. racing_game: `data/engine_thermal/*.json`, `data/cooling_systems/hyper.json`, `tests/unit/test_game_car_thermal.cpp`, `core/include/rg/vehicle_setup.h`
