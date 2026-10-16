# Damage model (first piece, 2026-10-16)

The owner answered "yes" on PR #28 to "should the radiator or fan also appear as a damage part that fails?". This is the
first piece: a per-car store of part health and three cooling parts that wear when the engine overheats.

## Owner rules (binding)

1. Damage persists until it is repaired.
2. Repairs are free.
3. A repair needs an explicit player action: the garage's **Repair (free)** button. Later, in races, a long pit stop.
4. Damage changes how the car drives.

## What exists

| Piece | Where | What |
|---|---|---|
| Config | `data/vehicles/damage.json` (`rg.damage/1`) | parts (id, label, description, effect, `min_scale`), accumulation rules, `report_below`, `save_interval_s`; strict loader |
| Model | `core/include/rg/damage.h`, `core/src/damage.cpp` | `DamageConfig`, `DamageState` (part id -> health 0..1), `DamageModel` (one tick), `cooling_scales`, `damage_view`, `damage_warning_line` |
| Session | `core/src/session.cpp` (`setup_damage`, `update_damage`, `apply_cooling_scales`) | `SessionConfig::damage`; runs on the stepping thread right after `World::step`; `damage_report()`, `request_damage_repair()` |
| Physics seam | physics_sim `World::set_engine_cooling_capacity` (`CoolingCapacity{radiator_ua_scale, fan_flow_scale}`) | scales the radiator's UA and the fan's face velocity at runtime; (1, 1) is bit-identical to never calling it |
| Persistence | `<user dir>/garage/damage/<vehicle id>.json` (`rg.damage_state/1`) | written atomically; a healthy car has no file |
| Garage | `rg::Garage::damage_config / damage_state / damage_state_path / repair_damage`; `DriveSelection::damage_config_path / damage_state_path` | |
| Godot | `RgGarage.get_damage(id)`, `RgGarage.repair_damage(id)`; `RgSimulation.get_damage_state()`, `RgSimulation.repair_damage(state_path)`; `set_vehicle_overrides` keys `damage_config_path`, `damage_state_path` | conversion only (`godot_ext/src/rg_damage_convert.h`) |
| UI | `game/scripts/hud.gd` (one warning line), `game/scripts/shell_ui.gd` (Damage panel + Repair button on the Engine > Cooling page) | |

## Parts and rules (all values [AS])

| Part | Effect | At health 0 | Worn by |
|---|---|---|---|
| `radiator` | radiator UA | x0.25 | coolant above the car's overheat warning + 4 K for 15 s: 0.15 %/s + 0.03 %/s per K above that |
| `radiator_fan` | fan face velocity | x0.0 | coolant above the overheat warning with the fan on for 30 s: 0.08 %/s |
| `head_gasket` | radiator UA (gas pockets) | x0.6 | coolant above the overheat warning + 12 K for 5 s: 0.4 %/s + 0.08 %/s per K |

A part's factor is `min_scale + (1 - min_scale) x health`; parts on the same effect multiply. The overheat warning is the
car's own (`cooling_system` `warnings.overheat_c`, read through `rg::engine_thermal_info`). A dwell timer resets as soon as
the signal drops below the threshold; health never comes back on its own.

## Lifecycle

- **Drive start.** `Garage::prepare_drive` names the config and the car's state file; `main.gd` passes both through
  `set_vehicle_overrides`; the init worker loads them into `SessionConfig::damage` (a broken state file drives healthy and
  is left alone, with an `RG_DAMAGE` line on stderr). A damaged car drives damaged from its first tick.
- **While driving.** `RgSimulation.get_damage_state()` (polled by the HUD) saves a changed state at most every
  `save_interval_s` (5 s).
- **World reload, Change car, main menu, exit.** `RgSimulation::teardown_current` and the destructor save a changed
  state before the Session is destroyed, so the next load of the same car (same catalog id) reads it back.
- **Repair.** The garage button calls `RgGarage.repair_damage(id)` (removes the file) and, if that car is the one running
  (pause -> garage), `RgSimulation.repair_damage(state_path)` -> `Session::request_damage_repair()`: healthy in the report at
  once, the physics back to (1, 1) at the next tick.

## Determinism and the no-op guarantee

The model is plain double arithmetic on the stepping thread in file order, so a run is worker-count invariant (test: 1 vs
4 workers). A healthy car never calls `set_engine_cooling_capacity`, and the physics only folds the factors into the state
hash once they differ from 1, so every pinned hash is unchanged (hash_check vehicle_step_steer 0xace4d38ea725e065). A car
without the thermal model keeps its state as loaded: nothing wears and nothing is applied.

## Adding a source later

Every limit of the garage setup check names the part it would damage (`AdviceItem::damage_part`). A new source (knock,
mean piston speed, charge-pipe boost, turbo overspeed, oil temperature, other overheating parts) is a new `DamageSignal`
read from the snapshot, plus a part whose `DamageEffect` the Session hands to a physics seam. The store is keyed by part
id and keeps ids it does not know, so a newer build's parts survive an older build's save.

## Tests

`tests/unit/test_damage.cpp` (`[damage]`): config loads and rejects bad files, state round trip (unknown ids kept, a healthy
state removes the file, a broken file is an error), scales, accumulation (dwell, excess, no self-healing, fan-on rule,
clamp at 0, repair), Session no-op hash (thermal car and the hash_check car), driving effect (worn radiator + fan run the
coolant hotter) at 1 and 4 workers, forced overheat wears parts deterministically, repair in a running Session, garage
paths and repair. physics_sim: `test_engine_thermal_powertrain.cpp` `[cooling_capacity]`.
