# Three new cars: design plan

Status: PLAN, nothing modelled yet (2026-10-13). Owner request: three cars with
clearly different design languages, exterior and interior planned, interactive
cockpit parts (turnable steering wheel, gear shifter, physical buttons, screens
whose contents come later), ready for CFD aero evaluation. Open questions are in
section 8 and on the pull request; modelling starts once they are answered.

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

## 2. Where the generators live (decision D1, question Q1)

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

**Body style (Q2):** recommended a closed two-door, one-bench "doctor's
coupé": interior is fully visible, CFD gets one closed outer surface, rain and
VR work. Alternatives: open 5-seat touring with the soft top up (fabric top as
a fixed shell, open sides: much harder for CFD), or a two-seat open runabout.

**Interior:** wooden floorboards, one buttoned leather bench, plain painted
dash board with a coil box and a key switch, one optional accessory
speedometer (cable driven, round), ammeter, choke pull-rod through the
radiator apron (visible from the seat as a knob on the dash in our version).
Large four-spoke wooden steering wheel with the **spark-advance and throttle
levers under the rim** (Model T style hand throttle). Three floor pedals
(clutch/low, reverse, brake) and a tall hand lever at the left (high-gear
release + parking brake). Door pulls on straps, roll-down windows with leather
straps, rear oval window.

**Interactive parts:** steering wheel, hand throttle lever, spark lever, three
pedals, hand lever, ignition key, choke knob, doors, windscreen upper pane
(tilts open), speedometer and ammeter needles. No screens.

**Physics questions (Q3):** a Model T drives with a two-speed planetary box
whose gears are selected by pedals (left pedal: down = low, middle = neutral,
up = high with the hand lever forward), a reverse pedal and a hand throttle.
physics_sim's gearbox kinds are `manual` and `sequential` only. Options:
(a) request a `planetary` gearbox + pedal-band controls from physics_sim and
drive the Model T as it was (recommended, it is the car's whole character);
(b) model the Model T controls visually but map them to a plain two-speed
manual and standard pedals until physics has it. Also needed: a period
2.9 L side-valve I4 (~15 kW at 1600 rpm, ~110 N m) engine file, cord tyre
sizes on 21 in rims in the axle fitment, and a magneto/coil ignition is
cosmetic only.

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

**Engine bay sized for swaps (D3, Q7).** The bay is the defining constraint.
Recommended envelope, measured behind the front axle centre and above the
crossmember: **1000 mm long x 900 mm wide x 720 mm high** under the stock
hood. That fits a long I6 (3.0 L ~ 750 mm), a 60/90 deg V8 including the
hypercar's twin-turbo V8 with its turbos at the sides (~800 x 850 x 700) and
an NA V12 (~950 mm); the four-rotor rotary is small. Taller engines get the
**hood variants** as separate parts: flat (stock), raised power bulge
(+60 mm), shaker/scoop (+120 mm, through-hole visible), so the swap shows on
the outside. This is the first real user of `check_fit`: the car gets
`engine_bay_envelope_mm` in the catalog, engines get `envelope_mm`; until
engines carry dimensions everything still fits (today's behaviour).

**Headlamps (Q4):** recommended fixed quad round lamps in recessed chrome
cups behind the grille line: one CFD pose, cleaner nose. Alternative: pop-up
lamps (a joint per lamp, very 70s, but adds a second CFD pose and a drag
penalty when up). **Roof (Q5):** recommended fastback with a flush rear
hatch; alternatives notchback or shooting brake.

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
ignition key, window cranks, doors, hood, hatch. "Screens": none; the radio
dial and the gauge faces are textures with needles as joints.

**Stock engine (Q6):** recommended the existing N52-class 3.0 I6
(`n52b30_3l_na`) as the placeholder stock engine; a period-correct 5.0 L
carburetted V8 or 4.4 L V12 would be a new engine file (physics_sim engine
work). Gearbox: the existing `econ_5mt`/`close_ratio_6mt` catalogue boxes, so
gearbox swaps work out of the box.

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

**Doors (Q8):** recommended five-door (two rear doors with pop-out rear
windows like the class). Alternative: three-door.

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

**Engine (Q9):** a 1.0 L NA I3 (~50 kW, ~95 N m) is the class engine and does
not exist yet. Recommended: use `i4_2l_na` as the placeholder stock engine and
ask physics_sim for an I3. Gearbox: existing `econ_5mt`.

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
- **Interaction (Q11)**: first version is display-only (parts follow the car's
  real inputs). Clicking/touching buttons with the mouse or VR hands later uses
  the same nodes plus a collision box per button in the sidecar.
- The hypercar can adopt the same sidecar later (its steering wheel would then
  finally turn); not part of this work unless asked.

## 5. Built for CFD from the start (D5, Q14)

Rules every generator follows, so `tools/aero_bake` runs without a voxel core
fill:

1. **Two exports from one parametric source.** The visual GLB, and a
   separate **CFD skin** `<id>.cfd.glb` built from the same section curves:
   one closed, manifold, consistently wound outer body shell (glazing flush and
   part of the shell, panel gaps closed, no inner skins, no trim), plus four
   closed wheel solids (tyre + rim face disc) and named separate solids only
   where flow really separates (mirrors, Runabout lamps/wings/running boards,
   GT bumpers if detached). No union by voxels; the generator emits closed
   surfaces directly. Byte-deterministic like the visual GLB.
2. **Labelled regions** in the CFD skin as separate nodes: `body`, `wheel_FL..RR`
   (for rotatingWallVelocity), `underbody`, `inlet_*`/`outlet_*` faces and
   optional `porous_*` closed volumes (radiator) - the same partitions
   `prepare_regions.py` currently reconstructs from triangle ranges.
3. **Clearances**: >= 10 mm between any two CFD solids at rest and over the
   full steering and suspension travel range (checked by the generator), so
   wheels never merge with arches; wheel arches are open cavities with closed
   liners.
4. **Cooling**: grilles are open through-ducts ending at a closed radiator
   face (porous zone volume exported separately), never a backed facade. A
   first sweep may seal the inlet face; the assumption is recorded per case.
5. **Underbody**: modelled to the level each car has (Pico: flat floor with
   tunnel and fuel tank bulge; GT: floor with exhaust tunnel and rear valance;
   Runabout: open ladder frame, axles and springs as closed simple solids -
   the bluff underside is a large part of its drag and must be in the model).
6. **Small parts** (< 6 mm thick: badges, wipers, antenna, door handles,
   stitching) are visual-only and listed in the CFD manifest as omitted.
7. **Manifest** `<id>.cfd.json`: source hashes, frontal area, reference
   point = chassis CoM after the wheel-derived alignment `vehicle_visual.gd`
   applies (aero_cfd_plan warns not to mix ground origin and CoM), ride height,
   pose (doors closed, wheels straight, windscreen closed, lamps down, hood
   variant), omitted parts, open/sealed openings.
8. **Gates** (in `tools/cargen/validate.py` and the existing
   `tools/aero_bake/geometry.py` audit): 0 open / nonmanifold / degenerate /
   duplicate / inconsistently wound edges per solid and combined, no
   intersections between solids, volume and bounds match the visual model
   within 5 mm. Plus a `.flow.json` exterior selection for the visual GLB in
   `physics_sim.visual_flow_selection/1` so existing consumers work.
9. **Pose variants** are explicit exports: GT hood variants (and pop-ups if
   chosen), Runabout windscreen open.

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
- Engines in the engine bay (Q10): no engine has a visual model today; the
  garage engine-bay camera would look at an empty bay. Recommended: a simple
  generic engine mesh per family (I3/I4/I6/V8/V12/rotary) generated from the
  engine catalogue's numbers, mounted at an `socket_engine` in the bay.

## 7. Work plan once the questions are answered

1. `tools/cargen` skeleton, validate (structure, determinism, CFD gates),
   `rg.cockpit/1` + `cockpit_controls.gd`, CFD skin writer; tests.
2. Cars one at a time, each: exterior -> CFD skin + audit -> interior and
   cockpit parts -> catalog/vehicle file -> screenshots (garage, chase,
   cockpit) on the PR. Recommended order (Q13): **Pico** (simplest, proves
   the pipeline), **Ventura GT** (engine bay + swap), **Runabout T** (waits
   longest on physics answers).
3. Triangle budgets: Pico 110k, Ventura GT 140k, Runabout T 130k (the
   hypercar is 179k), CFD skins separate and not in the runtime budget.

## 8. Open questions (also on the PR)

| # | question | recommended |
|---|---|---|
| Q1 | Generators in racing_game `tools/cargen` (importing assetgen read-only), or in physics_sim's assetgen? | racing_game |
| Q2 | Runabout body: closed coupé, open touring with top, or two-seat runabout? | closed coupé |
| Q3 | Runabout controls: real Model T pedal planetary (physics_sim request), or Model T looks on a normal manual for now? | real (request) |
| Q4 | GT headlamps: fixed quad or pop-ups? | fixed |
| Q5 | GT roof: fastback, notchback or shooting brake? | fastback |
| Q6 | GT stock engine: existing 3.0 I6, or a new period V8/V12 file? | existing I6 |
| Q7 | GT bay size: fit up to a V12 (1000 mm) or V8 only (850 mm)? | V12 |
| Q8 | Pico: five-door or three-door? | five-door |
| Q9 | Pico engine: placeholder 2.0 I4 now + request a 1.0 I3, or wait for the I3? | placeholder |
| Q10 | Generic engine meshes in the bay (game-owned)? | yes |
| Q11 | Cockpit parts display-only first, clickable later? | yes |
| Q12 | Names: Runabout T / Ventura GT / Pico ok? | ok |
| Q13 | Order: Pico, GT, Runabout? | yes |
| Q14 | CFD skin as a separate generated mesh from the same sections? | yes |

Requests that would go to physics_sim (only after the owner agrees): a
planetary two-speed gearbox with pedal bands (Q3), a period side-valve I4, a
1.0 I3 (Q9), optionally a period V8/V12 (Q6), a steering ratio field in the
steering block, and engine `envelope_mm` in engine files (Q7).
