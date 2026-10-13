# Three new cars: design plan

Status: PLAN SETTLED, modelling in progress (2026-10-13). Owner request: three
cars with clearly different design languages, exterior and interior planned,
interactive cockpit parts (turnable steering wheel, gear shifter, physical
buttons, screens whose contents come later), ready for CFD aero evaluation.
The owner answered the questions on PR #2 on 2026-10-13; section 8 records the
decisions and the sections below are updated to match.

| id (working) | title (working) | brief | era / language |
|---|---|---|---|
| `car_runabout` | **Runabout T** | something akin to a Ford Model T | 1910s-20s coachbuilt: upright, separate parts, black enamel and brass |
| `car_gt70` | **Ventura GT** | sleek 1970s car, long hood, room for a big swapped engine | 1970s grand tourer: long-hood/short-deck, crease lines, chrome |
| `car_city` | **Pico** | cheap economy car like a Citroen C1 2018 / VW up! 2022 | 2010s-20s A-segment: tall, rounded box, big glass, plastic and LED |

All three are generic and class-typical, like every model in physics_sim's
`data/models` (README: "none reproduces a specific production vehicle"): no
real make, badge, grille shape or trademark is copied. The references set
proportions and mood only.

## 1. What already exists and what this plan reuses

- **Model convention** (physics_sim `data/models/README.md`): `<id>.glb`
  (glTF, `gltf(x,y,z) = iso(y,z,x)`) + `<id>.rig.json` (ISO 8855, metres,
  origin on the ground midway between the axles, tyres touching z = 0). Nodes
  `susp_XX -> steer_XX -> wheel_XX`, `socket_*` empties (`socket_driver_eye`
  drives the cockpit camera and the seat adjustment), light functions mapped to
  lens materials, joints in node extras. The new cars follow it unchanged, so
  `vehicle_visual.gd`, `body_visuals.gd`, the cockpit rig, the garage and the
  car browser load them without special cases.
- **Generator**: physics_sim `tools/assetgen` (stdlib-only Python, byte
  deterministic): `CarBody` section lofts, `parts.corner/steering_wheel/seat/
  mirror`, `mesh` primitives, `scene.Model` writes GLB + rig.
- **CFD tooling** (`docs/aero_cfd_plan.md`, `tools/aero_bake`): GLB -> ISO OBJ
  export and topology audit, solid preparation, wheel region partition,
  OpenFOAM case generation. The hypercar showed what a visual-first model costs
  later: 3300 open edges, a voxel "core fill" that put 10 % of the surface more
  than 24 mm off the real shape, intakes that are backed facades, overlapping
  multi-part assemblies. Section 5 is designed so the new cars skip that repair
  stage.
- **Garage**: catalog entries (`data/vehicles/catalog.json`), engine swaps
  (`EngineStore`, `derive_swap_package`), gearbox swaps (`GearboxCatalog`),
  universal tyres (`TyreFamilies`: `cord`, `economy`, `touring` ... already
  exist), and the reserved fit seam `rg::check_fit` (`engine_swap.h`: "Reserved:
  car engine_bay_envelope_mm, engine envelope_mm, mount, driveline").
- **Gap found**: the hypercar rig has a `steering_wheel` joint, but no game
  script drives it today (nothing in `game/scripts` references it). There is no
  shifter, pedal, button or needle convention yet. Section 4 adds one.

## 2. Where the generators live (decision D1, Q1: default taken)

`data/models` and `tools/assetgen` belong to the physics_sim session and are
read-only from this repo. Recommendation: a racing_game-owned generator
`tools/cargen/` (stdlib Python, deterministic like assetgen) that **imports
physics_sim's `tools/assetgen/assetgen` package read-only** (`sys.path` to the
submodule) for `mesh`, `scene`, `parts`, `carbody`, `vec`, and writes the new
cars to **`data/models/<id>/`** in this repo. Anything assetgen lacks (CFD
skin export, cockpit sidecar, new primitives) is written in `tools/cargen`, not
patched into the submodule. If a pin bump changes an assetgen function the
determinism check (`cargen/validate.py --determinism`) flags it.

Alternative: brief the physics_sim session to add the three generators there.
That keeps one asset pipeline but puts the game's car design on another
session's queue.

## 3. The three cars

Dimensions are targets for the generator; final numbers go into each
generator's header like assetgen's modules. Mass and chassis box go into the
catalog entry (interim, as for every car today).

### 3.1 Runabout T (Model T-like, ~1915-1925)

**Design language: "coachbuilt".** Every function is its own visible part:
tall flat radiator with a brass shell, short bonnet with louvres, separate
cycle-style front and rear wings (fenders) joined by running boards, upright
flat two-piece windscreen, carriage lamps on the scuttle, large drum
headlamps on a bar, black gloss enamel, wooden artillery wheels with thin
tyres, exposed leaf springs and axles. Verticals and circles, no surface flows
into the next.

| | target |
|---|---|
| length / width / height | 3.40 m / 1.70 m / 1.95 m (closed body) |
| wheelbase / track | 2.54 m / 1.42 m |
| tyres | 30 x 3.5 in clincher -> universal tyre `cord`, about 90/95-21 equivalent, r ~0.36 m |
| ground clearance | ~0.25 m |
| mass (catalog) | ~650 kg |
| engine bay | front, short (I4 2.9 L class) |

**Body style (decided, Q2):** an open two-door touring body with a
**convertible top that is a garage upgrade option**: "no top" (open car) and
"top up" (fabric top on bows, open sides). The top is its own node and its own
model part; the garage option toggles it on the model and switches the CFD
pose (two CFD exports: open, top up). A folding/animated top is not planned
now.

**Interior:** wooden floorboards, one buttoned leather bench, plain painted
dash board with a coil box and a key switch, one optional accessory
speedometer (cable driven, round), ammeter, choke pull-rod through the
radiator apron (visible from the seat as a knob on the dash in our version).
Large four-spoke wooden steering wheel with the spark-advance and hand
throttle levers under the rim (cosmetic: the spark lever moves, the hand
throttle feeds the throttle channel like a pedal). **Normal manual driving
(decided, Q3):** clutch, brake and throttle pedals in today's order and a tall
period floor gear lever in an open three-speed gate, plus a tall handbrake
lever at the side. Door pulls on straps, rear oval window in the top.

**Interactive parts:** steering wheel, hand throttle lever, spark lever, three
pedals, gear lever, handbrake lever, ignition key, choke knob, horn bulb,
doors, windscreen upper pane (tilts open), speedometer and ammeter needles. No
screens.

**Powertrain:** a plain manual (a game-owned three-speed `physics_sim.gearbox/1`
file) and a period 2.9 L side-valve I4 (~15 kW at 1600 rpm, ~110 N m) as a
simulated cycle engine with pipes, so it has a live voice like the other two
(the owner asked for sound on Q6/Q9; giving the third car one is the default
here). Cord tyres on 21 in rims in the axle fitment.

### 3.2 Ventura GT (1970s long-hood GT)

**Design language: "chrome and crease".** Front-engined two-seat (2+2 optional)
fastback coupé, proportions of the era's long-hood GTs: hood about 45 % of the
length, cabin set back over the rear axle, short tail. A sharp shoulder
crease running from headlamp to tail lamp with a "Coke-bottle" kick over the
rear wheel, a flat hood with a raised centre power bulge, recessed full-width
black grille, slim chrome bumpers with rubber over-riders, a Kamm tail with
round twin tail lamps, louvred rear quarter windows, 70s paint (burnt orange,
olive, bronze metallic), polished five-spoke alloys with chrome lips.

| | target |
|---|---|
| length / width / height | 4.45 m / 1.80 m / 1.22 m |
| wheelbase / track | 2.55 m / 1.48 m |
| tyres | 215/60 R15 front, 225/60 R15 rear (`touring`/`sport` family) |
| mass (catalog) | ~1350 kg with the stock engine |
| engine bay | front-mid: engine behind the front axle line |

**Engine bay sized for swaps (decided "big", Q7).** The bay is the defining
constraint. Envelope, measured from the radiator core back to the firewall and
above the crossmember: **1000 mm long x 900 mm wide x 720 mm high** under the
stock hood (V12 class). That fits a long I6 (3.0 L ~ 750 mm), a 60/90 deg V8 including the
hypercar's twin-turbo V8 with its turbos at the sides (~800 x 850 x 700) and
an NA V12 (~950 mm); the four-rotor rotary is small. Taller engines get the
**hood variants** as separate parts: flat (stock), raised power bulge
(+60 mm), shaker/scoop (+120 mm, through-hole visible), so the swap shows on
the outside. This is the first real user of `check_fit`: the car gets a bay
envelope in its catalog entry and every engine gets a real envelope from its
generated engine mesh (section 6); an engine that does not fit is turned away
in the garage with the reason ("engine is 1080 mm long, the bay takes 1000 mm").

**Headlamps (decided, Q4): pop-ups.** Twin round lamps per side in two
pop-up pods hinged at their rear edge (`popup_L`/`popup_R`, revolute, 0 =
closed flush with the hood, + = up). They rise with the headlamp switch, and
the CFD exports carry both poses (lamps down, lamps up). **Roof (Q5, not
answered, default kept):** fastback with a flush glass hatch.

**Interior:** low bucket seats with corduroy/houndstooth centres, deep-dish
three-spoke steering wheel with a wooden rim and a centre horn button, a
cliff-face dash with **six round analogue gauges in individual cowls**
(speedometer, tachometer big; oil pressure, water temperature, fuel,
voltmeter small), a centre stack with a row of **chrome toggle switches**
(lights, wipers, fan, hazard, fog), two heater slide levers, a push-button
radio (later content), a ball-top **gear lever in a leather gaiter on a tall
tunnel console**, a pull-up handbrake, three floor pedals (organ throttle),
door cards with window cranks and chrome pulls, a clock.

**Interactive parts:** steering wheel, gear lever (two-axis: gate position
from the selected gear), handbrake, three pedals, six needles + clock hands,
toggles, heater levers, radio buttons and dial, indicator and wiper stalks,
ignition key, window cranks, doors, hood, hatch, pop-up lamps. "Screens": none; the radio
dial and the gauge faces are textures with needles as joints.

**Stock engine (decided, Q6): a V8 with sound.** A new game-owned
naturally aspirated 90 deg cross-plane V8 (~5.0 L, period two-valve pushrod
character, ~220 kW, ~430 N m) written as a `physics_sim.engine/1` simulated
engine with a cycle block and intake/exhaust pipe blocks plus an installation
file, which is what gives it a live engine voice (physics_sim's
`VehicleVoice::try_create` needs exactly that). All numbers labelled [AS]
like the other game engines. Gearbox: the catalogue's `heavy_duty_6mt` or
`close_ratio_6mt` (rated for the V8's torque), so gearbox swaps work out of
the box.

### 3.3 Pico (A-segment city car, 2018-2022)

**Design language: "friendly box".** Short overhangs, wheels pushed to the
corners, a tall glass house, a one-piece frameless glass tailgate with black
surround, a smiling lower intake under a slim upper grille, big swept
headlamps with an LED daytime line, black plastic arch and sill cladding,
skinny 14 in steel wheels with covers (alloys as a variant), contrast roof
option. Everything is rounded-rectangle: lamps, windows, screen, buttons,
vents all share one corner radius family.

| | target |
|---|---|
| length / width / height | 3.55 m / 1.64 m / 1.48 m |
| wheelbase / track | 2.42 m / 1.42 m |
| tyres | 165/65 R14 (`economy` family) |
| mass (catalog) | ~880 kg |
| engine bay | front, transverse, small (I3 1.0 L) |

**Doors (decided, Q8):** five-door (two rear doors with pop-out rear windows
like the class).

**Interior:** body-colour dash strip across the whole dash (the class
signature), a small instrument cluster (analogue speedometer + small mono LCD
for rev bar, fuel, trip), a **7 in central touchscreen** standing on the dash
(contents later: radio, phone, trip, camera), two round vents at the ends and
two in the middle, three rotary climate knobs, a hazard button, window switches
on the doors (front only, rear pop-out), a five-speed H-gate lever, a pull
handbrake, three pedals, steering wheel with audio/phone buttons, stalks, hard
plastic door cards with bottle holders, simple bench-like rear seat, roof
grab handles, exposed body colour on the door tops.

**Interactive parts:** steering wheel (+ its buttons), gear lever, handbrake,
pedals, stalks, climate knobs, hazard button, window switches, cluster
needle + LCD, touchscreen, doors, tailgate, rear pop-out windows.

**Engine (decided, Q9):** a new game-owned 1.0 L naturally aspirated I3
(~50 kW, ~95 N m) with sound: a simulated cycle engine with pipe blocks and an
installation file like the V8 above. Gearbox: existing `econ_5mt`.

## 4. Interactive cockpit convention (new, game-owned)

The rig keeps physics_sim's format unchanged. Cockpit parts are ordinary rig
nodes with joints (so Blender/viewer posing keeps working); what drives them is
a new game-owned sidecar **`<id>.cockpit.json`** (`rg.cockpit/1`), read by a new
`cockpit_controls.gd` under `vehicle_visual.gd`:

```json
{
  "format": "rg.cockpit/1",
  "controls": [
    {"node": "steering_wheel", "source": "wheel_steer_angle", "map": {"lock_to_lock_turns": 2.8}},
    {"node": "pedal_throttle", "source": "control:throttle", "map": {"range": [0, 1]}},
    {"node": "shifter", "source": "gear", "map": {"gate": {"1": [-1, 1], "2": [-1, -1], "N": [0, 0]}}},
    {"node": "needle_tach", "source": "engine_rpm", "map": {"in": [0, 8000], "out_deg": [0, 270]}},
    {"node": "btn_hazard", "source": "control:hazard", "map": {"press_m": 0.004}}
  ],
  "screens": [
    {"node": "screen_center", "material": "screen_center", "px": [1024, 600], "uv_rect": [0, 0, 1, 1], "role": "infotainment"}
  ]
}
```

- **Naming**: `ctl_*`/`pedal_*`/`btn_*`/`sw_*`/`knob_*`/`lever_*`,
  `needle_*`, `screen_*`. Each moving part is its own node with its own
  pivot, so a press, flick or turn is one joint value.
- **Sources** are what the frame snapshot already has (steer angle, pedals as
  applied controls, gear, rpm, speed, fuel, temperatures) or cosmetic local
  state (hazard, wipers, lights). The steering wheel turns from the physics
  road-wheel angle times a per-car ratio (lock-to-lock in the sidecar, since
  physics_sim's steering block has no ratio).
- **Screens** are flat quads with their own material and UV 0..1; at runtime
  the material's albedo/emission is replaced by a `SubViewport` texture, the
  way `hyper_live_cluster.gd` mounts the hypercar's digital cluster. Contents
  are a later task; the model ships with a dark "off" material.
- **Interaction (decided, Q11: clickable)**: parts first follow the car's real
  inputs; then each control gets a hit box in the sidecar so the mouse (a ray
  from the cockpit camera) or a VR hand can press, flick, turn or drag it, and
  the action goes out through the same control channels the keyboard uses.
- The hypercar can adopt the same sidecar later (its steering wheel would then
  finally turn), after the hypercar model work in progress elsewhere lands.

## 5. Built for CFD from the start (decided, Q14: derive from the visual model)

The CFD body is DERIVED from the visual GLB by the existing `tools/aero_bake`
chain (`geometry.py` export and audit -> `prepare_solid.py` /
`prepare_regions.py` -> OpenFOAM case). No separate hand-built CFD skin. So
the visual model itself has to be clean enough that the derivation needs no
voxel "core fill" guesswork. Rules every generator follows:

1. **Every exterior part is a closed, manifold, consistently wound solid** in
   the visual GLB: body shell with stitched inner returns (the hypercar's
   repaired pattern, built in from the start), glazing as closed slabs, lamps
   and grilles backed, tyres and rims closed. Each part is checked per node
   and combined (0 open / nonmanifold / degenerate / duplicate / mis-wound
   edges at 1 micrometre weld).
2. **No coincident surfaces between parts**: adjoining exterior parts keep a
   1 mm clearance or share no faces; overlaps are explicit and listed in the
   model notes (as `GEOMETRY.md` does for the hypercar).
3. **Exterior selection sidecar** `<id>.flow.json`
   (`physics_sim.visual_flow_selection/1`, pinned to the GLB's sha256) marks
   the triangle ranges that form the flow boundary: outer body/glazing skins,
   lamps, grille faces, mirrors, wheels, underbody. Interior, engine bay,
   inner skins are excluded. The generator writes it from its own part
   bookkeeping, so the ranges are exact.
4. **Wheels** are separate nodes (`wheel_XX` with the tyre and rim inside),
   so `prepare_regions.py` gets the rotating-wall partitions directly, and
   >= 10 mm clearance to the arch liners over full steer and travel (checked
   by the generator). Arches are open cavities with closed liners.
5. **Cooling**: grilles are open in front of a closed radiator face (a
   recorded "sealed at the core" assumption for the first sweeps); no
   painted-on facades.
6. **Underbody** modelled to the level each car has (Pico: floor with tunnel
   and tank bulge; GT: floor with exhaust tunnel and rear valance; Runabout:
   ladder frame, axles, springs and running boards as closed solids - its
   bluff underside is a large part of its drag).
7. **Small parts** (< 6 mm: badges, wipers, antenna, stitching) stay out of
   the flow selection and are listed as omitted.
8. **Pose variants** come from the rig joints, which `geometry.py` already
   poses before export: GT pop-ups up/down, Runabout top on/off and
   windscreen open, closed doors and straight wheels for the base case.
9. **Gates** in `tools/cargen/validate.py`: the topology audit above,
   determinism (same bytes twice), selection hash and range checks, joint
   limits, clearances, and the `geometry.py` export + audit run on every car.

Cd targets for sanity, not tuning: Runabout ~0.75-0.9 at ~2.3 m2, Ventura GT
~0.38-0.42 at ~1.75 m2, Pico ~0.32-0.34 at ~2.05 m2. Until CFD maps are
validated the cars use authored analytic aero like every car today.

## 6. Garage, catalog and physics data

- Catalog entries per car (`body_type` vintage / gt / city), `engine_bay`
  (front for all three), `engine_bay_envelope_mm` for the GT (and the others,
  so `check_fit` can turn a V8 away from the Pico once engines carry sizes),
  tyre fitment per axle, chassis mass/box, paint and rim colours, presets
  (e.g. Pico "base / sport", GT "stock / big-block").
- Vehicle files `data/vehicles/car_runabout.json`, `car_gt70.json`,
  `car_city.json` (game-owned, `physics_sim.vehicle/2`, refs into physics_sim
  data like `car_sedan.json`) with wheel attachment points taken from the
  rig, plus engine thermal and cooling files like every game car.
- **Engine meshes from physics data (decided, Q10)**: every engine gets its
  own generated mesh built from its physical description - architecture,
  cylinder count, bore, stroke, rod length, bank layout and angle, valves per
  cylinder, intake runners/plenum, exhaust primaries/collectors, turbo count
  and size, and (for the rotary) rotor count. A game-owned generator
  (`tools/cargen/engine_mesh.py`) derives block length from bore pitch,
  deck height from stroke and rod, heads from valve count, and so on, and
  writes `<engine>.glb` plus an envelope (`rg.engine_envelope/1`: bounding box
  and mount points). Engines with only a torque map and no geometry get an
  envelope from a class rule (displacement and cylinder count) and say so.
  The car's catalog entry carries its bay envelope and engine mount point;
  `rg::check_fit` compares the two, so an engine that does not physically
  fit is turned away with the reason. The garage engine-bay camera shows the
  installed engine's mesh at the mount.

## 7. Work plan

1. `tools/cargen` skeleton: imports physics_sim's assetgen read-only, own
   validate (topology, determinism, selection, clearances), a numpy/PIL
   preview renderer (no Blender or Godot in the cloud container), tests.
2. Engines: the 1.0 I3 and the V8 (and the Runabout's 2.9 I4) as simulated
   cycle engines with pipes and installations, checked by loading them with
   physics_sim's loader and generating maps on a Linux build here; engine
   catalog entries.
3. Engine mesh generator from physics data, envelopes for every catalog
   engine, `rg::check_fit` using them, garage bay view.
4. Cars one at a time, each: exterior -> topology/flow selection ->
   interior and cockpit parts -> `rg.cockpit/1` sidecar -> catalog entry and
   vehicle file -> preview renders on the PR. Order (Q13 not answered,
   default kept): **Pico**, **Ventura GT**, **Runabout T**.
5. Cockpit parts driven by the car (`cockpit_controls.gd`), then clickable
   (Q11: required, last): mouse ray / VR hand ray against per-control hit
   boxes from the sidecar, press/flick/turn animation, and the action sent
   through the existing control channels.
6. Triangle budgets: Pico 110k, Ventura GT 140k, Runabout T 130k (the
   hypercar is 179k).

Things that cannot be checked from the cloud container and are left to the
owner: running the cars in Godot (no Godot here), the Windows build, the live
engine voice by ear, and the OpenFOAM pilot runs.

## 8. Decisions (owner answers on PR #2, 2026-10-13)

| # | question | decision |
|---|---|---|
| Q1 | Where do the generators live? | not answered: default `tools/cargen` in racing_game, importing physics_sim's assetgen read-only |
| Q2 | Runabout body | open touring; the convertible top is a garage upgrade option (top / no top) |
| Q3 | Runabout controls | normal manual |
| Q4 | GT headlamps | pop-ups |
| Q5 | GT roof | not answered: default fastback |
| Q6 | GT stock engine | a V8, with sound |
| Q7 | GT bay size | big (V12 class) |
| Q8 | Pico doors | five-door |
| Q9 | Pico engine | a new 1.0 L NA I3, with sound |
| Q10 | Engine meshes | every engine gets its own mesh from its physical properties; the engine must physically fit the car |
| Q11 | Clickable cockpit parts | required; may come last |
| Q12 | Names | not answered: default Runabout T / Ventura GT / Pico |
| Q13 | Order | not answered: default Pico, GT, Runabout |
| Q14 | CFD body | derived from the visual model |

The hypercar model is being changed elsewhere (physical dials, buttons and
indicators). This work does not touch it; adopting the cockpit sidecar for
the hypercar waits until that lands on main.

Nothing here needs a physics_sim change: the engines, gearboxes and vehicle
files are game-owned data in the formats physics_sim already loads.
