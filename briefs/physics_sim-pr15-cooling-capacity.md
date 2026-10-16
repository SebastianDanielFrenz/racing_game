# physics_sim #15: World::set_engine_cooling_capacity: runtime radiator / fan factors

Physics seam for racing_game's damage model. Before, a car's cooling system was fixed once loaded; a game could not make a worn radiator or broken fan cool less while driving. `World::set_engine_cooling_capacity(vehicle, CoolingCapacity{radiator_ua_scale, fan_flow_scale})` scales the radiator UA and the fan face velocity at runtime. The branch has since also gained `World::set_engine_wear` (worn engine internals raise friction/heat) and a fan-power change (see section 3).

## 1. State at hand-off

| Item | Value |
|---|---|
| Repo / PR | physics_sim, https://github.com/SebastianDanielFrenz/physics_sim/pull/15 |
| Head branch / sha | `claude/cooling-capacity` @ `4e81619` (full 4e816198916dec76b0afac3f20ec834e96749156; re-verified at the end, see bottom) |
| Base | `master` (c08b00c at time of reading; branch already contains it via merge commit bab3ddf) |
| State | DRAFT, mergeable_state reported "unknown" by GitHub, but a local `git merge-tree` of master with the branch gave no conflict (inferred clean) |
| Size | 3 commits, 9 files, +217 / -3 (GitHub PR numbers; the PR body text predates commit 3) |
| Commits | `26a193d` set_engine_cooling_capacity; `13e0452` (CMake <3.30 configure fix, came in from master) merged by `bab3ddf`; `4e81619` set_engine_wear + fan power |
| racing_game consumer | racing_game #33 pins `26a193d` (commit 1 only) |

**Merge order: this PR first, then racing_game #33 re-pinned to the merged master commit.** The pin in #33 is an unmerged commit of THIS branch (26a193d) and must not be merged as is.

**What landed vs. the "IN PROGRESS" item "fan power drops with a damaged fan":** it DID land, in `4e81619` (pushed 19:10 UTC, after Sebastian's answer): `fan_power_w` now scales with the cube of `fan_flow_scale` (fan affinity laws), exactly the loaded power at scale 1. Test: the `[cooling_capacity]` case was extended. Not yet updated: the PR description (still says the question is open and lists only the first commit's content).

## 2. What is done

- `ps::drivetrain::CoolingCapacity{radiator_ua_scale, fan_flow_scale}` in `core/include/ps/drivetrain/engine_thermal_desc.h`, both default 1.0.
- `World::set_engine_cooling_capacity(VehicleId, CoolingCapacity)`: queued for the next tick boundary (like `set_engine_thermal_state`), stays until the next call, `reset_vehicle` does not touch it. `EngineThermal::radiator_conductance` multiplies UA by the scale; the fan term multiplies the fan face velocity; ram air is NOT scaled (a dead fan still lets the airstream through).
- Snapshot publishes `radiator_ua_scale`, `fan_flow_scale` in `EngineThermalSnapshot`.
- Hash: factors are folded into `fold_state_hash` only while they differ from (1,1); an unused or explicit (1,1) call is bit-identical.
- Throws on unknown vehicle, vehicle without `cooling_system`, or a factor not finite or outside [0, 1e3].
- Commit 3: `World::set_engine_wear(VehicleId, EngineWear{friction_scale})` multiplies the plant's oil-viscosity friction scale k(T_oil) (more friction torque and heat into the oil); ECU estimate untouched; queued like the cooling call; 1.0 bit-identical and unhashed, other values hashed and published as `EngineThermalSnapshot::wear_friction_scale`; same rejections. Intended for racing_game's worn bearings/pistons (the oil-failure requirement).
- Files: `core/include/ps/world/world.h`, `core/include/ps/drivetrain/engine_thermal_desc.h`, `core/src/drivetrain/engine_thermal.{h,cpp}`, `powertrain.{h,cpp}`, `core/src/vehicle/vehicle_instance.h`, `core/src/world/world.cpp`, `tests/unit/test_engine_thermal_powertrain.cpp`.
- Tests: `[cooling_capacity]` ((1,1) same hash and coolant as never calling; (0.3,0.3) hotter: 107.4 C vs 98.8 C after 10 s from 104 C at part throttle; 1 vs 4 workers same hash; rejections; fan power cubic) and `[engine_wear]` (bit-identical at 1, worn = more friction heat, hotter oil, lower rpm, 1 vs 4 workers, rejections). PR reports `ps_unit_tests "[thermal]"` 35/35 on linux-release for the first commit; the numbers for commit 3 are not in the PR text (the commit message only lists the tests).

## 3. Unfinished / next steps

1. Run the whole physics suite (only `[thermal]` was run) and Windows CI; confirm `[engine_wear]` and the fan-power test pass there (the cloud result for commit 3 is not documented in the PR).
2. Update the PR description to cover commit 3 (set_engine_wear, fan power) and mark ready when green.
3. Merge to master (rebase/merge may conflict with `claude/block-cooling`, an in-progress branch touching the same thermal files; whoever lands second rebases).
4. Then tell racing_game #33 to re-pin to the merged master commit.
5. Possibly further seams Sebastian's damage plan needs (inferred): an engine-stop/failure path for a cascade (racing_game can already cut ignition via controls; check), no request made yet.

## 4. Open questions for Sebastian

- Answered (decision): "should a damaged fan also report less `fan_power_w`?" Sebastian: "Yes, report less fan power." Implemented in 4e81619 (cubic with flow scale; the cube law is the implementer's choice, not his, so he may want to look at it).
- No unanswered questions in the PR.

## 5. Owner decisions and rules that apply beyond this PR

- The owner allowed this physics change for the damage model; physics_sim is changed through its own branches/PRs (clones allow branches + PRs in all three repos).
- Damage model rules (from racing_game #33 comments): damage persists until repaired, repair free and explicit, damage must change how the car drives, wear grows with distance past limits, each part's health maps to function and a cooling cascade can stop the engine, an engine pushed far past its limits fails before oil reaches ~170 C (2 MW / 11 bar hyper reached ~190 C), "Difficulty" toggle for mechanical damage separate from collision damage. Future damage sources: knock, piston speed, charge-pipe boost, turbo overspeed, overheating parts. Physics seams for these are likely requested of this repo.

## 6. Known issues / risks

- PR text is stale relative to the head (commit 3 missing in the description).
- On CMake 3.28 the cloud configure failed at `ps_environment_standalone` (WHOLE_ARCHIVE); fixed on master by #16 (13e0452), already merged into this branch.
- Overlaps with `claude/block-cooling` (same files).
- Determinism: factors hashed only when != 1, so existing scenario hashes should be unchanged; racing_game reports hash_check vehicle_step_steer `0xace4d38ea725e065` unchanged with 26a193d (not re-reported for 4e81619).

## 7. Local build, run, verify (Windows)

Check physics_sim's `CLAUDE.md`/README and `tools\ci.ps1` for the exact presets; typical:
```
git fetch origin && git checkout claude/cooling-capacity
cmake --preset <debug|release> ; cmake --build --preset <...>
ps_unit_tests "[cooling_capacity]"
ps_unit_tests "[engine_wear]"
ps_unit_tests "[thermal]"
tools\ci.ps1        # physics_sim's own full CI (run it in the physics_sim checkout)
```
No map store or LFS models needed. Then in racing_game: set the submodule `external/physics_sim` to this head, `cmake --build --preset debug`, `rg_unit_tests "[damage]"`, run `hash_check` and compare `vehicle_step_steer` to `0xace4d38ea725e065`.

## 8. Not verified in the cloud

Only `[thermal]` (35 cases) for the first commit, Linux clang release; no full suite, no Windows CI; commit 3 results not documented; no behavior check beyond unit tests.

## 9. Files to read first

1. `core/include/ps/world/world.h` (the two new public calls)
2. `core/include/ps/drivetrain/engine_thermal_desc.h`
3. `core/src/drivetrain/engine_thermal.cpp`
4. `core/src/drivetrain/powertrain.cpp`
5. `tests/unit/test_engine_thermal_powertrain.cpp` (`[cooling_capacity]`, `[engine_wear]`)
6. racing_game `docs/damage_model.md` (consumer, branch `claude/cooling-damage`)
