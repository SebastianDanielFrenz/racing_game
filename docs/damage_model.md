# Damage model (2026-10-16)

The owner answered "yes" on PR #28 to "should the radiator or fan also appear as a damage part that fails?". On PR #33
he added: "pushing far in excess of safe values should ruin a part quicker; getting oil up to 170 C within 3 minutes
should not be possible because the engine breaks first", "Repair is a menu item and a garage feature. We also need a
block 'difficulty', which contains a toggle for the damage model. This is SEPARATE from collision damage", and "an
engine without a real working cooling solution should keep running, but then other parts will start failing and that
might cause the engine to shut off".

## Owner rules (binding)

1. Damage persists until it is repaired.
2. Repairs are free.
3. A repair needs an explicit player action: the pause menu's **Repair car**, the garage's **Repair (free)** buttons
   (car browser = the garage's first page, and Engine > Cooling). Later, in races, a long pit stop.
4. Damage changes how the car drives.
5. The whole model is switched by **Settings > Difficulty > Mechanical damage** (`difficulty.mechanical_damage`, default
   on). Off: nothing wears and stored damage has no effect; the state stays on disk. Collision damage is not part of it.

## What exists

| Piece | Where | What |
|---|---|---|
| Config | `data/vehicles/damage.json` (`rg.damage/2`) | parts (id, label, description, effect, `scale_at_zero`, optional `seize_below`), wear rules with curves, `report_below`, `save_interval_s`; strict loader |
| Model | `core/include/rg/damage.h`, `core/src/damage.cpp` | `DamageConfig`, `DamageState`, `DamageModel` (one tick), `wear_rate_per_s`, `damage_scales`, `seized_part`, `damage_view`, `damage_warning_line` |
| Session | `core/src/session.cpp` (`setup_damage`, `update_damage`, `apply_damage_scales`, `apply_damage_controls`) | `SessionConfig::damage` (with `enabled`); runs on the stepping thread right after `World::step`; `damage_report()`, `request_damage_repair()`, `set_damage_enabled()` |
| Physics seams | physics_sim `World::set_engine_cooling_capacity` (radiator UA, fan flow; fan power scales with flow cubed) and `World::set_engine_wear` (`EngineWear{friction_scale}`: the engine plant's friction) | 1 is bit-identical to never calling them |
| Seizure | the Session holds the `ignition` and `starter` channels at 0 before every step while a part is seized | no physics seam: the engine stops and cannot be cranked; the crank is not locked (a moving car still turns it in gear) |
| Persistence | `<user dir>/garage/damage/<vehicle id>.json` (`rg.damage_state/1`) | written atomically; a healthy car has no file |
| Shell | `rg::ShellFlow`: pause item `repair_car`, event `RepairChosen` on VehicleSelect / ChangeCar, action `RepairCar{vehicle}` | `RgShell.repair_chosen(id)`; main.gd `_repair_car` |
| Setting | `rg::Settings` `difficulty.mechanical_damage` (section Difficulty) | main.gd `_apply_setting` -> `RgSimulation.set_damage_enabled` |
| Godot | `RgGarage.get_damage/repair_damage`; `RgSimulation.get_damage_state()` (adds enabled, seized, seized_part), `repair_damage(state_path)` ("" = the running car), `set_damage_enabled` | conversion only (`godot_ext/src/rg_damage_convert.h`) |
| UI | `hud.gd` (one line; "ENGINE SEIZED (...)" when seized; none while the setting is off), `car_browser_ui.gd` (damage lines + Repair on the focused car), `shell_ui.gd` (Engine > Cooling panel) | |

## Parts and rules (all values [AS])

Health to function: `factor = scale_at_zero + (1 - scale_at_zero) x health`, parts on the same effect multiply.

| Part | Effect | At health 0 | Seizes below | Worn by (safe value; curve = health lost per second over the excess) |
|---|---|---|---|---|
| `radiator` | radiator UA | x0.25 | - | coolant > overheat warning + 4 K for 15 s; 0.15 %/s at the limit, 2 %/s at +30 K |
| `radiator_fan` | fan flow | x0.0 | - | coolant > overheat warning with the fan on, 30 s; 0.08 %/s rising to 0.4 %/s |
| `head_gasket` | radiator UA | x0.6 | - | coolant > overheat warning + 12 K for 5 s; 0.4 %/s rising to 25 %/s at +50 K |
| `crankshaft` (Crank bearings) | friction | x3.0 | 0.2 | oil > 135 C while the engine runs: [0, 0.01 %/s], [5 K, 0.05], [10, 0.2], [20, 1.5], [30, 10], [35 K, 100 %/s] |
| `pistons` (Pistons and rings) | friction | x2.0 | 0.2 | oil > 145 C while the engine runs: [0, 0.05 %/s], [10, 0.4], [20, 3], [25 K, 60 %/s] |

Curves are linear between points and continue along the last segment, so the wear grows faster than the excess. 135 C is
the setup check's oil limit (`cooling_upgrades.json` `oil_limit_c`): a car the check passes never wears its internals. The
overheat warning is the car's own (`warnings.overheat_c`, 118 C for the class defaults and the hypercar).

The cascade follows from the physics: worn cooling runs the coolant hotter, the coolant heats the oil, hot oil wears the
bearings and pistons, their extra friction heats the oil further and costs power, and they seize.

## Calibration against the owner's report

Owner: car_hyper at about 2 MW and 11 bar reached about 190 C oil within 3 minutes. Measured (hidden
`tests/unit/test_damage_engine.cpp` `[.][damage][long]`, 30 C, full throttle from standstill on a 40 km strip,
turbo_boost_target 11 / 1.8):

| Setup | Peak crank power | Peak boost | Oil after 180 s, model off | With the model |
|---|---|---|---|---|
| hyper_twin_large | 1886 kW | 4.81 bar | 122.0 C | no wear |
| hyper_single_120mm | 2290 kW | 5.71 bar | 121.3 C | no wear |
| car_hyper_n2o, twin_large_n2o, Sport + nitrous | 2266 kW | 4.97 bar | 121.7 C | no wear |
| hyper_twin_large, radiator and fan dead | 1887 kW | 4.81 bar | 151.5 C | seized (crank bearings) at 141.8 s with oil 164.3 C |

The garage cannot reproduce 11 bar (the turbos choke near 4-6 bar) and the oil stays near 122 C with the upsized
`hyper.json` cooling, so the owner's 190 C is not reproduced with this build. The calibration is therefore also checked on
a prescribed oil ramp (fast case "oil climbing toward 190 C"): from 110 C at 0.44 K/s (the owner's rate) the bearings
seize at 161 C, at 1 K/s at 166 C, at 3 K/s at 169 C, before any friction feedback is counted.

Dead-radiator run timeline (live model): 20 s coolant 150 C, head gasket starts wearing; 30 s head gasket 0; 90 s oil
136.6 C, bearings start; 120 s oil 150 C, friction x1.18; 135 s oil 158 C, x1.86; 140 s oil 162 C, x2.71, bearings 0.35;
141.8 s bearings 0.20: seized, ignition off. The coolant reaches about 315 C before the stop: the physics model has no
boiling, so that number is not physical.

## Lifecycle

- **Drive start.** `Garage::prepare_drive` names the config and the car's state file; `main.gd` passes both through
  `set_vehicle_overrides`; the init worker loads them into `SessionConfig::damage` with `enabled` from the setting.
- **While driving.** `RgSimulation.get_damage_state()` (polled by the HUD) saves a changed state at most every 5 s.
- **World reload, Change car, main menu, exit.** The state is saved before the Session is destroyed.
- **Repair.** Pause menu "Repair car" repairs the running car (`RgSimulation.repair_damage("")`) and its file; the car
  browser's Repair repairs the focused car's file (and the running car if it is that one). The engine unseizes at the next
  tick; the player then starts it as usual.
- **Setting.** `set_damage_enabled(false)`: the next tick gives the physics healthy factors and releases a seized engine;
  `true` applies the stored state again.

## Determinism and the no-op guarantee

Plain double arithmetic on the stepping thread in file order: worker-count invariant (tests at 1 vs 4 workers). A healthy
car never calls the physics seams, so every pinned hash is unchanged (hash_check vehicle_step_steer 0xace4d38ea725e065).

## Tests

`tests/unit/test_damage.cpp` (`[damage]`): config and loader errors (curves, ranges), state file, scales and seizure, wear
growing faster than the excess, accumulation, Session no-op hash, driving effect at 1 and 4 workers, repair, garage.
`tests/unit/test_damage_engine.cpp` (`[damage][engine]`): worn bearings cost rpm through `set_engine_wear`; the cascade on
the sedan (dead radiator and fan, part throttle: seized at about 297 s, healthy cooling keeps the bearings above 95 %;
1 vs 4 workers identical); a seized engine does not restart until repaired and seizes while running; the setting off =
no wear, no effect, state kept, toggling live; the oil ramp; hidden `[.][damage][long]` owner scenario above.
`tests/unit/test_shell_flow.cpp` (Repair car), `test_settings.cpp` (Difficulty). physics_sim:
`test_engine_thermal_powertrain.cpp` `[cooling_capacity]`, `[engine_wear]`.
