# Garage UI and tyre customization

## Current state (2026-10-08)

Upgrades has vehicle information on the left, the showroom car in the middle and component cards on the right. Tuning adjusts installed parts. Existing validation, reset, save and Drive remain authoritative in the native garage model.

Category tiles replace the old camera-area tabs and component dropdown. B/Esc
returns one level at a time; the root opens the save-and-exit prompt. Upgrades:

- Engine → Turbo → upgrades for the installed single/dual setup; Camshaft is unavailable.
- Swaps → Engine → compatible engine choices.
- Swaps → Forced induction → NA/Single turbo/Twin turbo, installed directly.
  Exact turbo hardware is chosen separately in Engine → Turbo. NA removes both
  engine-default and vehicle-override turbo references; it does not disable nitrous.
- Swaps → Drivetrain → AWD/RWD/FWD (unavailable until native conversion support).
- Chassis → Weight reduction (unavailable), Aero → Front diffuser/Rear wing
  (unavailable), or Suspension & brakes → tuning installed hardware.
- Wheels → All/Front/Rear → Compounds/Dimensions → installable image cards.
- Drivetrain and Appearance & assists link to installed-part tuning.
- Upgrades → Drivetrain → Gearbox (2026-10-13): swap the car's gearbox. Boxes are grouped Manual (manual
  and sequential, the latter labelled "Sequential"), Automatic and CVT; the last two are empty until
  physics_sim models a torque converter and a variator and show "Coming soon". Every car is offered the
  whole catalogue (data/gearboxes/gearbox_catalog.json) filtered by the INSTALLED engine's peak torque:
  a box whose rating is below it is not listed, and a setup that forces one anyway is rejected with
  "gearbox: rated 450 N*m, engine makes 1371 N*m". The car's own box is the "stock" card. Changing
  the box clears the per-gear ratio factors (with a notice); an engine swap that leaves a too-weak box
  drops it back to stock with a notice. The card shows kind, focus, gear type, gear count, first/last
  ratio and rating. The N2O shot and extra boost are not counted against the rating (the list describes
  the engine as catalogued).
- Upgrades → Swaps → Drivetrain (2026-10-15): change the drive layout and the axle differentials. Layout cards
  are grouped FWD / RWD / AWD (AWD with an open, viscous or locked centre; physics_sim has no Torsen/LSD centre yet); below them one card row per
  DRIVEN axle offers open, clutch LSD, Torsen, viscous or locked. The first card of each list is what the car has now
  (the layout as delivered; for a differential: the axle's unit as the chosen layout left it). Catalogue:
  data/drivetrains/driveline_catalog.json (rg.driveline_catalog/1); the transform is generic code
  (rg/driveline_swap.h): the new axle units copy the car's own (ratio, efficiency, inertia, kind), so the gearing does
  not change and the final drive slider scales the new units. No engine-position rule: every car whose driveline is
  one gearbox output into one or two axle differentials takes every layout; a transfer case or anything else greys
  the tile out. A layout change that stops driving an axle returns that axle's differential choice to stock with a
  notice. Every layout stays offered, FWD <-> RWD included (owner on PR #19). A swap that runs the driveline the length
  of the car (an AWD conversion, FWD <-> RWD) carries a weight penalty: the part-mass tree (see "Part masses" below)
  adds the new axle hardware and a layout-conversion node. The stats (vehicle select, car browser) follow the setup's layout.

Compound installs preserve each axle's width. Dimension installs preserve each
axle's compound. All applies the chosen property to both axles, offering only
choices compatible with both fitments. Different compounds can remain on front
and rear after a combined width change. No unsupported physical upgrades are
offered as functional parts. Tuning also uses category tiles.

The installed part receives keyboard/controller focus on opening an install menu;
the orange focus outline moves with navigation. INSTALLED remains a separate
caption. Combined wheels with differing configurations have no installed marker
and start focus on the first valid choice. Grid direction neighbors keep focus
within the part grid. Category tiles use text; installable components retain images.

Tyres are universal (2026-10-13): a tyre is a compound plus a size (`<compound>_<width>_<aspect>r<rim>`, e.g. `track_325_30r21`) and belongs to no car or axle. Each car declares per-axle fitment by dimensions only, in data/vehicles/catalog.json under `setup.tyre_fitment.tyre_front` / `tyre_rear`: accepted rim diameters, a width range in standard steps and an overall-diameter tolerance around the axle's own wheel (the aspect that keeps the overall diameter is derived per width). The garage offers every compound at every size the fitment allows (hyper front 235-305 mm, hyper rear 275-365 mm, sedan axles 195-255 mm, steps of 10 mm; these are conservative game limits, not measured fender clearance). A size that does not fit is rejected with the reason, for example `tyre_rear: 255 mm is narrower than this axle's 275 mm minimum`. Names of the earlier per-car tyre files in saved setups are migrated automatically (rg::migrate_tyre_id).

Installation updates the axle's tyre spec (`tyre_spec` of each wheel), actual wheel width and wheel rotational inertia. Showroom and driving wheel meshes scale along their axle to match the installed width. Existing tyre images represent compound/tread style; widths share a style image and have explicit dimensional captions. The car meshes retain their original tread geometry for now.

## Garage exit flow (2026-10-08)

The configurator footer has Reset all and Save. There are no Back or Drive buttons.
B/Esc first returns through the nested category hierarchy. At the top level
it opens a modal with exactly two actions: **Save and drive** and **Save and go to
selection**. B/Esc again cancels that prompt and resumes editing, preserving focus.
The prompt confines keyboard/controller focus to its choices. Both actions save
before navigating, including clean/stock setups. Invalid setups disable both
actions; a save failure leaves the prompt open and displays the error. Existing
main-menu spawn picking and paused-world respawn semantics still apply to driving.
The vehicle selection browser retains its existing back navigation.

## Authored physical fits

There are no tyre files per size or compound. data/tyres/families.json holds the only authored data (per-compound parameters, the base Magic Formula tyre and the scaling rule); rg::TyreFamilies::make (core/src/tyre_spec.cpp) builds the ps::vehicle::TyreDesc in memory and rg::load_vehicle (core/src/vehicle_load.cpp) injects it into the loaded VehicleDesc, because vehicle files carry a `tyre_spec` per wheel instead of a path. The physics library source is not edited. The garage thumbnails/labels (game/assets/components/tyres/catalog.json) come from tools/generate_tyre_presentation.py.

For width ratio r = selected width / baseline width:

- Longitudinal/lateral friction scales by r^0.35, with separate family factors. Grip increases with diminishing returns.
- Rolling resistance base scales by r^0.65, with a family factor.
- Wheel inertia scales by r at unchanged unloaded radius.

Compound factors (longitudinal grip, lateral grip, rolling resistance) are listed per family in data/tyres/families.json (road 1/1/1, track 1.10/1.10/1.10, slick 1.20/1.20/1.15, drag 1.25/0.80/1.35 for the first four; the ordinary road families are in docs/tyre_families.md). They apply to one base coefficient set for all sizes (no anchoring to any old stock tyre). The drag fit favors straight-line traction and sacrifices lateral grip.

Rated speeds are per compound (road/track 500 km/h, slick 450, drag 350 ...), capped at 270 km/h for rims up to 17 in. Ratings participate in the physics library's existing speed-dependent rolling resistance anchors. They are displayed in the UI, but are not a blowout, temperature or comprehensive high-speed grip/failure model. These fictional fits require driving calibration; no measured tyre claim is made.

## Remaining plan

Step 1 layout is implemented. Native performance benchmarking/cache is still needed for acceleration milestones, top speed with gear/RPM and maximum longitudinal/lateral acceleration. Those values remain explicitly unmeasured. Declared engine specifications are displayed separately.

Step 2 now implements compound/tread and width installation, fitment validation, physical width response and width visuals. Further tyre work includes mesh tread geometry, measured/calibrated fits, pressure/temperature and weather-sensitive behavior.

Step 3 now supports engine and turbo installation cards plus installed-engine calibration.
The conventional sedan family can swap the library 2.0 L inline-four and game 3.0 L
inline-six torque-map engines. This is authored compatibility for fictional chassis;
chassis mass and the engine-bay mesh stay unchanged, not a measured real-world swap.
Hypercars retain their V8 and offer turbo configurations: original twin at 1.8 bar,
touring twin at 1.0 bar, balanced twin at 1.4 bar, and (non-nitrous only) single at
1.4 bar. Boost values are charge-pressure targets, not guaranteed manifold pressure.
The nitrous version has its own choices retaining its larger-wastegate hardware.
Installation uses whitelisted IDs resolved to approved files. Unknown file paths,
cross-family installs and incompatible overrides are rejected. Engine references
are loaded before tuning, then Save/Drive use normal native loader validation.
Changes apply on Drive, not live in an already-running session.

Transparent images distinguish inline-four / inline-six and single / twin hardware.
They are illustrative component renders, not exact scans. Calibration variants of
the same hardware share an image. Rev limiter and throttle-response labels show
RPM and milliseconds from the installed engine's baseline, rather than only factors.
Stock engine specifications are labeled as stock: tuned dyno/benchmarks remain pending.
ECU map parts, further engine internals, engine geometry design and engine-bay mesh
replacement remain future work.

## Verification

Native setup suite: 193 assertions across 13 existing cases passed after the fitment change. New width tests passed 111 assertions across two cases and check grip, rolling resistance, inertia, unchanged radius/other axle, width caps, catalog-configured expansion and all families' available widths. The garage smoke passed 121 checks, including the two-stage flow and 305 mm installation/restoration, existing tuning rejection/save/reload/Drive and node cleanup. Screenshots of type selection and the width submenu were reviewed at 1920x1080. Logs: out/tyre_custom_final.log and out/tyre_width_final_unit.log; images: out/tyre_custom_shots.
Engine customization now starts with installed-engine calibration in Tuning / Engine bay: rev limit (75–100% of stock) and electronic throttle response time (50–200% of stock). These whitelist actual engine-file parameters, retain bore/stroke and torque maps, and use the existing materialization and physics-loader validation. Engines lacking the corresponding field do not offer that control. Higher rev limits require a future compatible internals upgrade. Performance figures remain stock references until the benchmark model supplies setup-specific results.

2026-10-08 engine-parts verification: 81 assertions in four focused native cases
cover engine swap plus limiter ordering, loader validation, incompatible/path-like
IDs, Save/restart/Drive, turbo count/flow/inertia, boost targets and the nitrous
wastegate. Existing setup regression: 463 assertions in 16 cases. Expanded garage
flow: 150 checks before the final highlight check, then 151 checks including it.
Component-card screenshots reviewed at 1920x1080 for hyper and sedan. Evidence:
out/engine_parts_unit3.log, out/engine_parts_setup_regression.log,
out/engine_parts_garage_final.log, out/engine_parts_shots and
out/engine_parts_sedan_shots. Dedicated logs/user directories kept normal saves
and the owner's drive log intact.

2026-10-08 garage-exit verification: 160 garage smoke checks passed, including
B cancellation, controller A confirmation, tyre-submenu back navigation, invalid
setup blocking, both save-first destinations, respawn and node cleanup. No engine
script errors. Prompt and footer screenshots reviewed at 1920x1080. Evidence:
out/garage_exit_accept.log, out/garage_exit_accept_engine.log and
out/garage_exit_shots (garage_exit.png and upgrade_engine.png).

2026-10-08 tile-hierarchy verification: 188 garage checks passed, including both-axle
compound and dimension installs, split compound preservation, fitment intersection,
single/dual filtering, installed focus, submenu back, invalid setups, persistence,
drive/respawn and cleanup. Evidence: out/garage_tiles_focus.log. Screenshots in
out/garage_tiles_final_shots reviewed at 1920x1080; root categories and tyre compound cards.

Direct induction types (2026-10-08): Swaps installs NA, Single turbo or Twin turbo
without opening hardware submenus. Re-selecting the current type preserves its
exact hardware. Engine upgrades show only the installed turbo topology. N2O single
uses the existing per-turbo N2O wastegate hardware. NA changes cycle-map sampling
to authored reachable pressures of 25/50/75/90 kPa instead of boosted nodes;
this is an NA conversion approximation and retains the original engine internals.
Verification: 195 garage checks and 80 native assertions across four engine-part
cases passed. Logs: out/garage_induction_final.log, out/garage_induction_polish_unit.log.
N2O swap screen captured in out/garage_induction_ready_shots/induction_types.png.

## Turbo kits for engines without a turbo (2026-10-15)

Owner on PR #5: "Turbos cannot be fitted yet to many engines." The option `turbo_kit` (setup_options.json, catalog
`turbo_kits`, group "Turbo system") fits a turbo system to the INSTALLED engine when it is a simulated engine with a cycle
block and no turbo of its own (pico, ventura, runabout, the demo pipe I4, the A25A; also after an engine swap into any
car). The generated hardware and configuration files are content-addressed under `<engine map cache>/turbo_kits/`.

Installing a kit sets the engine's `default_forced_induction` to the generated configuration, its
`cycle.grid_p_im_kpa` to `ps::drivetrain::boost_grid_p_im(cycle, target)` and lowers a compression ratio above 10.0 to
10.0 (physics_sim engine.md "Adding or removing forced induction"; knock, lambda and spark stay the physics' own
models). The swap package treats a kit like a torque increase: the clutch rule (1.5 x peak) uses peak x the charge
pressure ratio, and the gearbox notice counts the boosted torque. Torque-curve engines, already-turbocharged engines,
rotaries and lift-family engines are refused: `OptionView::unavailable_reason` (RgGarage `unavailable_reason`) says why,
and Engine -> Turbo / Swaps -> Forced induction show that text instead of hiding the tile. Changing to an engine that
cannot take a kit drops it (`Garage::drop_unavailable_values`).

### Sizing logic (owner on PR #21: count and boost are free choices)

The kits in `data/engines/turbo_kits.json` (rg.turbo_kits/1) are shortcuts for a turbo COUNT and a STARTING boost:
Street (1, 0.5 bar), Sport (1, 1.0 bar), Twin-turbo (2, 1.0 bar), Quad-turbo (4, 1.0 bar). Any count goes on any eligible
engine (no cylinder rule); the boost slider (`turbo_boost_target`) then moves the target freely. One rule, applied to the
final target (compile re-sizes the kit after the slider: `CompiledSetup::turbo_kit`):

1. Air demand of the whole engine at the limiter: charge density (ambient compressed to the target at the anchor's
   efficiency, then the intercooler at 0.75 effectiveness) x swept volume x limiter rpm / 120 x VE 0.9.
2. Split over N turbos: each compressor must flow demand / N x 1.4 (flow margin: no choke at the limiter, no late spool).
3. Each unit = physics_sim's anchor `hyper_turbo_approx.json` (GTX3582R class, 82 mm exducer) scaled at constant tip
   speed, s = sqrt(flow / anchor flow), clamped to [0.3, 1.6]: flow, throat, wastegate, shaft loss x s^2, reference rpm / s,
   rotor inertia x s^5, efficiencies anchor + 0.127 ln s. More turbos = smaller, faster, quicker-spooling, less efficient
   units; a quad on a 1.0 L three reaches the 0.3 clamp (four 25 mm units, slightly oversized).
4. Envelope per unit from its exducer D: 2.6 D + 60 mm long, 2.2 D across. The units sit in a row along the crank on the
   exhaust side (a V engine with more than one: split over both banks); the row sets the length when longer than the
   engine; width grows by 0.5 x 2.2 D per side (the turbo takes the NA exhaust manifold's place, which the engine envelope
   already holds). `rg::check_fit_size` holds engine + kit against the car's `engine_bay_envelope`; a kit that does not
   fit is refused on install ("... does not fit Pico (556 mm long, the bay takes 500 mm)") and its card says so first.
   The charge pipe and the front-mounted intercooler are not in the box.
5. Masses ([AS], `mass` block): turbo unit 2.0 kg + 7.5 kg x s^3; exhaust manifold and downpipe 3 kg per turbo;
   intercooler core 0.45 m^2 per kg/s of demand x 76 mm x 650 kg/m^3; charge piping 1.8 m (+0.6 m per extra turbo) of a
   pipe sized for 60 m/s at charge density, wall and density from the material. `rg::turbo_kit_parts` puts them under the
   engine of the part-mass tree (`engine.turbo_kit.*`), so the drive's chassis mass follows the kit.

Measured (Sketch maps, engine dyno at the same rpm): Street kit on every kit engine x1.30 to x1.61 power at 0.43-0.49 bar;
the Quad on the pico at 1.0 bar 88 kW at 5650 rpm against 54 kW NA (x1.63), rotor ~212 000 rpm (rated 460 000).

### Charge piping

Option `charge_piping` (catalog `charge_piping`, offered wherever a kit can go): Plastic (comes with the kit, rated 1.5 bar,
bursts near 3 bar), Aluminium (3 / 8 bar), Stainless steel (5 / 15 bar), each with wall, density and an [AS] source. The
setup check reports boost above the rating; the setup still saves.

### Damage model data

Limits are recorded as data now and only WARNED about by the setup check (limit advisor, game-side build rules in
`data/garage/advisor.json`), nothing fails in the simulation yet:

| Rule | Value | Limit (data) | Severity |
|---|---|---|---|
| `ChargePipingPressure` | max(boost target, sweep charge pressure) | piping `rated_bar` / `burst_bar` (turbo_kits.json `piping`) | Near from 90 %, warning above rated, critical (variant `burst`) at burst |
| `TurboShaftSpeed` | sweep rotor speed (one rotor) | kit `max_shaft_rpm` = reference rpm x `limits.max_shaft_speed_factor` 1.15 | Near from 90 %, warning above |
| `MeanPistonSpeed` | 2 x stroke x top sweep rpm | the WEAKEST installed rod / piston / crank grade (engine_internals.json); without internals data advisor.json `part_limits.max_mean_piston_speed_m_s` 25 | Near from 90 %, warning above |
| `ValveFloat` | top sweep rpm | valve spring grade x the engine's factory limiter (engine_internals.json `limiter_factor`) | Near from 90 %, warning above |
| `CylinderWall` | the wall left between two bores of a bored block | engine_internals.json `bore.safe_wall_mm` 5 mm [AS] | warning below (a geometry figure, any rpm); damage part `cylinder_block`, option `engine_bore` |

Every part-limit item carries `AdviceItem::damage_part` (Godot: `damage_part` in `RgGarage.get_advice`): the part a
damage model would wear or break - `charge_piping`, `turbo_unit`, `connecting_rods`, `pistons`, `crankshaft`,
`valve_springs`, `cylinder_block` (a thin wall after a bore), or `engine_internals` for the engine-wide fallback. Its `option_id` is the garage option that swaps that
part (the weak rod set: `engine_rods`), so "fix" opens the part, not only the rev limit.

How a future damage model reads them: `CompiledSetup::turbo_kit` (a `TurboKitBuild`) lists each fitted part
(`TurboKitPart`: id, count, mass, `limits` map - `max_shaft_rpm` on `turbo_unit`, `rated_bar` / `burst_bar` on
`charge_piping`), and the advisor's `AdviceInput` carries the same figures (`set_part_limits`). A damage model would run
on the stepping thread against the live `PowertrainState` (turbo_rpm, boost) instead of the dyno sweep, use the same
numbers as thresholds (rated = wear starts, burst = the part fails: boost leak / turbo failure), and its outcome would be
the engine and turbo telemetry the HUD already shows. Knock is the physics' own model already (ECU and knock limit);
overheating has the engine thermal model's temperatures but no limits yet.

**Engine internals** (2026-10-16, owner on PR #21: the safe piston speed depends on the material, a titanium rod as in
F1 holds more; `rg/engine_internals.h`, `data/engines/engine_internals.json` rg.engine_internals/1): four swappable
parts, garage Upgrades > Engine > Internals (options `engine_rods`, `engine_pistons`, `engine_crankshaft`,
`engine_valve_springs`, catalog `engine_internals`):

| Part | Grades ([AS] rating, mass vs stock) |
|---|---|
| Connecting rods (tensile load, kN per litre of cylinder volume) | cast steel (economy) 26 x1.10 (below stock); stock powder-forged 40; forged steel H-beam 56 x1.05; titanium 70 x0.65 |
| Pistons (mean piston speed) | low-silicon cast 18 m/s x1.05 (below stock); stock cast 21 m/s; hypereutectic 23 m/s x0.97; forged 2618 26 m/s x0.92 |
| Crankshaft (mean piston speed) | lightweight cast 18.5 m/s x0.85 (below stock); stock cast iron 21 m/s; forged steel 25 m/s x1.0; billet steel 27 m/s x0.9 |
| Valve springs (x factory limiter) | weak single 0.85 x x0.85 (below stock); stock 1.0 x; uprated dual 1.25 x x1.15; race beehive 1.45 x x1.0 |

**Connecting-rod load** (owner on PR #21: the load on the rod scales with the piston's weight). The rod is rated by the
peak inertial tension at TDC on the exhaust stroke, not by piston speed:

    F = m_recip x (S / 2) x w^2 x (1 + lambda)

`m_recip` = the piston set (pin and rings included) of ONE cylinder + one third of the rod's own mass
(`rod_small_end_share`), both from the installed grades' mass factors; `S` = stroke; `w` = 2 pi rpm / 60;
`lambda` = crank radius / rod length (`rod_crank_ratio` 0.3 [AS], one ratio for every engine because the engine data
carries no rod length yet; use the engine's own once it does). A rod grade's rating is `limit` x (displacement / cylinders)
newtons. The rpm where the rod reaches it is `w = sqrt(rating / (m_recip x S/2 x (1 + lambda)))`, so a lighter piston
grade (forged 0.92) raises it, a heavier one (low-silicon cast 1.05) lowers it, and lighter rods lower the load too. A
stock rod on stock pistons holds at least `stock_margin` x the factory limiter (the margin is on the engine speed).

**Weakest part.** Every bottom-end part is reduced to the engine speed at which it reaches its rating (piston speed ->
rpm = v x 60 / (2 S); rod load as above): the lowest one is the engine's safe rpm and the part the setup check names. The
rod's rating is displayed there as the equivalent mean piston speed (`limit`) with its rpm (`limit_rpm`).

**Downgrades.** The first grade is always stock; a grade whose table figure is below stock's keeps its own figure instead of
being lifted to the engine's stock rating, so it may sit below what the factory limiter needs and then the setup check warns
(`MeanPistonSpeed` / `ValveFloat`, naming the part) at the stock rev limit; the card says "below stock". Stock itself never
warns. Upgrades still never rate below stock.

**Upgrade lists show what fits.** By default the garage upgrade lists (turbo kits, engine swaps and the other catalog option
cards) hide a choice the car cannot fit; each `SetupPart` carries `fits` / `fit_reason` (core, `visible_parts`,
`hidden_parts_reason`). A tile whose choices are all hidden shows one line with the reason. A "Show all" toggle in the
upgrade panel (hidden setting `garage.show_all_upgrades`, default off) lists them greyed out with their reason.

- A stock part rates at least `stock_margin` (1.15) x the factory figure (piston speed at the engine's own limiter, or
  the limiter), so no car as built warns; an upgrade never rates below stock. The engine's limit is its
  weakest rod / piston / crank (titanium rods on a cast crank still break the crank).
- Masses: stock set = kg per litre x displacement, a grade = stock x `mass_factor`; the part-mass tree gets the
  DIFFERENCE under `engine.internals.<kind>` (negative for lighter parts), so the drive's chassis mass follows. The
  physics engine inertia is not changed (no physics input for rotating mass yet).
- The choice edits no engine file (limits and masses only). Simulated engines with a cycle block only; torque-curve and
  rotary engines show the reason. A choice is kept across an engine swap and re-rated for the new engine (a swap to a torque-curve engine then refuses the setup with the reason, as for a turbo kit).
- While the internals are known, physics' own `MeanPistonSpeed` dyno finding (one table figure, 25 m/s) is dropped: the
  part rule replaces it.

**Cylinder bore** (2026-10-16, owner on PR #21: "I want upgrades that bore out the cylinders; this logically implies new
pistons (due to diameter difference), among other things."). Garage option `engine_bore` (Upgrades > Engine > Internals,
catalog `engine_internals`, `internal` "cylinder_bore", placed after `engine_install` and before `turbo_kit` in
setup_options.json so a kit sizes for the bored engine). Unlike the other internals it EDITS the engine file: `bore_mm`
and `cycle/compression_ratio` through the merge patch (the option's pointers are the whitelist), so the swept volume and
the generated maps follow (`bore_m` is part of the physics engine-map cache key, so a bored engine generates its own maps
once, then hits the cache). Steps and figures in engine_internals.json `"bore"`, every one [AS] with its reasoning:

| Step | Oversize | Note |
|---|---|---|
| `overbore_0_5` | +0.5 mm | first rebuild oversize, every block takes it |
| `overbore_1_0` | +1.0 mm | largest rebore without sleeves |
| `big_bore` | +3.0 mm | re-sleeved big-bore kit, limited by the wall |

- **Per-engine limit.** No engine file carries its bore spacing, so spacing = `pitch_factor` 1.09 x factory bore [AS]
  (K20 1.08, N52 1.07, LS3 1.08, B18 1.11). Wall = spacing - new bore. Below `min_wall_mm` 4.0 the step is refused
  (`SetupPart::fits` false with the reason, hidden unless "Show all"; a saved one is refused by compile, and an engine
  swap to a block that cannot take it drops it back to stock with a notice); below `safe_wall_mm` 5.0 it fits but the
  setup check warns (`CylinderWall`, damage part `cylinder_block`).

  | Engine | Bore | Spacing [AS] | +3 mm wall | Big bore |
  |---|---|---|---|---|
  | 1.0 L I3 (Pico) | 71.0 | 77.4 | 3.4 mm | refused (at most +2.4 mm) |
  | 2.5 L I4 (A25A) | 87.5 | 95.4 | 4.9 mm | fits, warns |
  | demo I4 (pipes) | 86.0 | 93.7 | 4.7 mm | fits, warns |
  | twin-turbo V8 (hyper) | 92.0 | 100.3 | 5.3 mm | fits |
  | 2.9 L side-valve I4 | 95.25 | 103.8 | 5.6 mm | fits |
  | 5.0 L V8 (Ventura) | 101.6 | 110.7 | 6.1 mm | fits |

- **Compression ratio** is computed, not kept: the head's clearance volume stays, the swept volume grows with bore^2, so
  CR' = 1 + (CR - 1) x (bore' / bore)^2 (rounded to 0.001; the Ventura's 9.0 becomes 9.48 with the big bore). The gasket
  ring and the crown above the deck are ignored (CR comes out slightly high). A turbo kit fitted afterwards still lowers
  it to the kit's ceiling.
- **Pistons of the new size.** A bored block takes pistons of the new diameter only: every piston grade is then offered
  at that bore (cards and labels name it, e.g. "104.6 mm bore"); mass = factory piston x (bore' / bore)^(2 + k),
  `piston_thickness_exponent` k = 1 [AS] (crown area x bore^2, crown thickness x bore for the same bending stress). The
  heavier piston loads the SAME rods more (rods keep their factory rating and mass), so the rod-limited rpm drops:
  Ventura big bore pistons 3.46 -> 3.78 kg, rod-limited 8,182 -> 7,955 rpm. Rods, crank and springs keep their masses.
- **Part-mass tree.** `engine.internals.pistons` carries the piston difference to the factory set even at the stock
  grade, `engine.internals.cylinder_block` the liner ring bored away: n x pi/4 x (b'^2 - b^2) x 1.5 stroke [AS] x 7200
  kg/m3 cast iron [AS] (Ventura big bore -3.2 kg).
- **Turbo kit** sizes from the bored displacement (the facts it reads are the bored engine's): Ventura + street single
  air demand 0.412 -> 0.436 kg/s, wheel scale 0.980 -> 1.009. The kit's torque factor for clutch sizing is multiplied by
  the displacement ratio. A bore without a kit or swap does not re-size the clutch (no swap package runs; +6 % torque on
  the big bore against the clutches' >= 1.2 x margin). Bay fit is unchanged (same block).
- **Measured** (engine dyno, Sketch maps, `[bore][dyno][long]`):

  | Engine, step | Displacement | 2,200-2,300 rpm | 3,450-3,600 | 4,750-4,900 | 5,650-5,850 |
  |---|---|---|---|---|---|
  | Ventura 5.0 V8, big bore (+3.0 mm) | +6.0 % | 433 -> 461 N*m (+6.5 %) | 457 -> 488 (+6.8 %) | 456 -> 470 (+3.3 %) | 384 -> 386 (+0.5 %) |
  | A25A 2.5 I4, +1.0 mm | +2.3 % | 240 -> 247 N*m (+3.0 %) | 285 -> 293 (+2.7 %) | 254 -> 257 (+1.2 %) | 213 -> 214 (+0.3 %) |

  Low and mid rpm gain the displacement plus a little from the higher compression ratio; at the top the gain fades
  because valves, ports, cams and the intake are the stock engine's (breathing-limited), which is what a real overbore
  does without head work.
- **Not changed** (physics gaps, stated): the engine_thermal file's own `displacement_l` and heat capacities stay stock;
  valve diameters (the `ValveDiameterBand` build rule compares them to the bore, so a big bore can move it), ports, cams
  and the engine's rotating inertia are unchanged; the knock-limited ECU sees the higher CR through the cycle model.
  No stroker crank: changing the stroke moves every stroke-based limit, the rod ratio (the A25A already sits at the
  3.0 `RodRatio` build-rule edge) and the deck height, so it needs new rods and pistons with another pin height - left
  for a later step.

Damage-model hook (owner on PR #5: damage persists until repaired, repairs are an explicit action - garage, later a pit
stop or finishing with it - probably free, and damage affects driving). Not built yet; the data is shaped for it:
`CompiledSetup::internals` / `turbo_kit` list every installed part with its limit, and every warning names its
`damage_part`. A damage model would keep a per-part wear/broken state in the saved setup (next to the part's grade),
add wear while the live state is above the rating (piston speed from the engine rpm, boost, rotor speed), break the part
at its failure figure, apply the effect in the drive (a broken rod: the engine stops; floated valves: misfire / power
loss; a burst pipe: boost leak), and clear the state on the garage's repair action.

Open questions for the owner: whether the charge pipe and intercooler should count against the bay (needs a bay model of
the gaps), whether metal piping should cost anything besides mass, and whether a burst should later be a part the player
replaces.

Tests: tests/unit/test_turbo_kit.cpp `[turbo_kit]`, tests/unit/test_engine_internals.cpp `[engine_internals]`, tests/unit/test_engine_bore.cpp `[bore]` (dyno: `[bore][dyno][long]`); the dyno comparisons (kit vs NA at the same rpm, the quad on the
pico) are `[turbo_kit][dyno][long]`.

## Cooling system upgrades (2026-10-16)

Owner on PR #21: "More combustion should result in more heat for the engine components which should need to be cooled. We need
cooling system upgrades (water cooling elements in the engine block, bigger radiator, ...)."

**Does the heat follow the combustion?** Yes, in the physics already (physics_sim engine thermal model): the heat into the engine
lump is `wall_heat_fraction` x the RELEASED fuel energy of each substep, plus friction. Measured here (car_gt70, 35 C, full
throttle 25 s, `[.][cooling][long]`): peak wall heat 135 kW without a turbo kit, 281 kW with the sport single kit (x2.08), coolant
after 25 s 95.1 vs 110.4 C. No physics change is needed for that. What the schema cannot do: it has NO water-jacket geometry,
passage area or wall-to-coolant heat transfer coefficient, so "water cooling elements in the block" cannot be a separate heat
path; they are modelled with the levers that exist (a stronger pump, more coolant in the block, a cooler thermostat).

`rg/cooling_upgrades.h`, `data/cooling_systems/cooling_upgrades.json` (rg.cooling_upgrades/1), four setup options in the
**Cooling** group (`cooling_radiator`, `cooling_fan`, `cooling_oil_cooler`, `cooling_block_cooling`, catalog `cooling_upgrades`).
A grade is a set of factors applied to the car's EFFECTIVE cooling_system / engine thermal values (the `extends` chain resolved),
so one table fits every car and engine; `materialise_setup` writes patched copies of the cooling_system file (radiator, fan, oil
cooler) and the vehicle's engine thermal override (block cooling).

| Part | Grades (all figures [AS], reasoning in each grade's `source`) | Lever in the physics |
|---|---|---|
| Radiator | stock; aluminium wide core (area x1.35, UA per area x1.15); race ducted (area x1.8, x1.3, ram x1.25) | `radiator.frontal_area_m2`, `ua_ref_w_k`, `air_mass_flow_ref_kg_s`, `ram_coefficient` |
| Fan | stock; high-flow (flow x1.5, on/off 4 K earlier); race twin (x2.2, 8 K earlier) | `fan.face_velocity_m_s`, `power_w`, `on_c/off_c`; only helps below the speed where the ram air is faster than the fan |
| Oil cooler | stock; street (+0.15 W/K per m/s per litre); race (+0.35) | `oil_cooler.air_w_k_per_m_s` |
| Block cooling | stock; high-flow pump (x1.5); enlarged jackets (pump x1.5, +0.35 L coolant per litre, thermostat 6 K lower) | `pump.l_min_per_krpm`, engine lump capacity, `thermostat.open_c/full_open_c` |

* **Mass.** Each grade is a child of the part-mass tree's `engine.cooling` node with the mass the NEW part adds over the stock
  part (fixed kg + kg per litre + kg per extra m2 of core); the chassis mass moves by the same amount (test: children sum, tree
  total moves by `CoolingBuild::mass_delta_kg`).
* **Availability.** The options are offered on every car with a cooling_system; a car without one gets the option unavailable with
  the reason. Each `SetupPart` carries `fits` / `fit_reason` (a grade the engine's file cannot work on, e.g. a per-litre grade
  without a displacement); the garage hides a non-fitting tile unless "show all" is on.
* **Setup check.** `CoolantTemperature` / `OilTemperature` rules (advisor.json build_rules). Owner answer on PR #28
  (option C): the check reports the TIME to the limit at constant full power, not only the steady temperature.
  `rg::transient_thermal` runs the physics model's own two-lump time history (the engine thermal file's heat capacities,
  backward Euler, thermostat wax lag and play, fan hysteresis, eps-NTU radiator) from the warm spawn state, for the sweep
  points up to 0.9 x the rev limiter and within 5 % of the peak power of those (a car tops out in its top gear below the
  limiter), on a 30 C day with the ram air that power holds (11 m/s x cbrt(kW)); step 0.25 s, horizon 1800 s
  (cooling_upgrades.json check). The soonest point counts: never reached = clean, reached after more than
  `thermal_time.critical_within_s` (60 s, advisor.json) = a warning, sooner = critical. The text names the time, the rpm, the
  air speed and the steady temperature it would settle at (`rg::steady_thermal`). The limit is the car's own
  `warnings.overheat_c` (coolant) or 135 C (oil). Items name `damage_part` (`engine_block` for the coolant, `crankshaft` for
  the oil film) and the option that fixes it (`cooling_radiator`, `cooling_oil_cooler`). Nothing is clamped.
* **Stock cars.** Every catalog car is clean except the nitrous hypercar. data/cooling_systems/hyper.json was upsized
  for this check (owner on PR #28: 1.4 m2 of radiator core, UA_ref 7200 W/K, ram 0.22, fan 7 m/s, oil cooler 1.0 W/K per
  m/s): stock car_hyper settles at 99.6 C coolant / 120.4 C oil and reaches neither limit (the first sizing reached 118 C
  after 16 s and settled at 134.5 C: critical). car_hyper_n2o shares the cooling system and the dyno sweep sprays without
  end: it settles at 119.6 C and reaches 118 C after 10 s (critical; the thermostat lag overshoots the steady figure), the oil
  135 C after 107 s; a real shot lasts 35 s.
* **Measured.** car_hyper, 35 C, full throttle 170 s to 413 km/h: stock coolant 96.2 C / oil 110.8 C at the end (first
  sizing 117.1 / 123.2 C), race radiator + race oil cooler 93.9 C / 89.3 C. `steady_thermal` at the end of the run predicts
  96.2 C (stock) and 93.5 C (race). ISA 15 C: peak 99.2 C during the pull, plateau 89-95 C.

Tests: tests/unit/test_cooling_upgrades.cpp `[cooling]` (table, options on every car, materialised files, mass sum, steady check,
advisor items); `[.][cooling][long]` runs the measurements above and the check on every stock car.

## Live engine dyno (2026-10-08)

The dyno chart is anchored at the bottom of the left performance panel, outside
its statistics scroll area. Orange is net crank torque (Nm), blue is power (kW),
with separate vertical scales and RPM on the horizontal axis. These are native
engine outputs, not the catalog's declared peak figures.

RgGarage snapshots engine-affecting configuration into a single GarageDyno worker.
Engine swaps, induction/hardware changes and engine-file calibration options are
included; tyre, brake, gearbox, assist and paint settings do not change the dyno
fingerprint. Identical requests are deduplicated. New requests cancel running map
work and supersede queued snapshots. Only the latest successful complete result
is published. The previous curve remains visible while updating or on failure;
changing vehicles clears it. No Godot or live simulation state is read on workers.

The harness reuses the physics library's own SimulatedEngine pre_solve/post_solve,
holding 33 RPM points from idle to just below the configured limiter at full pedal.
Each point settles for 15 simulated seconds and averages the final second. Net
brake torque is tau0 minus kinetic friction capacity; power is torque*omega/1000.
Conditions: ISA, warm reference friction, N2O disarmed. Torque-map engines use their
native net WOT curve. This is crank output, not wheel output or an acceleration
benchmark, and it excludes a nitrous-on curve for now.

Full engine maps use the library cache. Two background-priority generation workers
and cancellation/progress reuse PHYS-065 (physics_sim engine-garage P1). Editing
uses generate=false validation, leaving generation-dependent checks to the dyno
worker and the normal full validation/drive loader. Native dyno errors are shown
without throwing away the last successful graph.

Native evidence: out/garage_dyno_unit.log (99 assertions: WOT magnitude, Nm/kW/RPM
conversion, NA reduction, latest request wins, deduplication, cancellation and
failure retention). out/garage_dyno_regression.log (590 garage/setup assertions).
Chart retention/swap check: out/dyno_chart_check.log. The updated DLL is installed;
out/adaptive_garage_smoke.log records 199 live headless garage checks.

## Setup check: the limit advisor (2026-10-13)

Owner decision: "Allow stupid turbo setups, show recommendations/warnings." The garage never
clamps a setup value (`turbo_boost_target` still reaches x20). Instead the SETUP CHECK panel under
the options lists simulation-derived notes for the pending setup.

- Core `rg/limit_advisor.h` / `core/src/limit_advisor.cpp` (engine-neutral, no Godot): the pending
  setup (the whole working setup) is materialised and loaded exactly like a drive
  (`materialise_setup` + `load_vehicle`, Sketch-quality engine maps, cached under the engine-map cache
  dir), the engine is swept on physics_sim's dyno (`run_dyno_sweep`, 250 rpm steps from 2000 rpm to 50 rpm
  under the limiter, the nitrous kit included when the car has one), `check_driveline_limits` adds the clutch
  and the gearbox (`GearboxInputTorque`, only for a gearbox file that names `rated_input_torque_nm`), and the
  LimitFindings become advice items: grouped per kind and part field, contiguous rpm ranges merged,
  worst severity first. Near = advice (neutral), Limiting = warning (amber), Exceeded = critical (red).
- Everything the game decides lives in `data/garage/advisor.json` (`rg.garage_advisor/1`): debounce
  (300 ms), sweep grid, merge gap, the text templates with `{placeholders}` (validated at load),
  per-variant text (BoostBelowTarget "spool" / "falloff", MapGridEdge "rpm"), `max_severity` caps (turbo lag
  at low rpm never louder than advice) and the game-side kind filters (next bullet). What physics grades a
  finding by, and which severities are worth telling, is physics' limit policy (below).
- **Who filters what (L0, owner rule: refusal limits and advice thresholds are config values).**
  `data/garage/advisor.json` names `limit_policy.json` (next to it), a `physics_sim.limit_policy/1` file that
  `extends` physics' `data/limit_policies/default.json` and overrides nothing yet. The advisor loads it with
  `ps::io::load_limit_policy_json(path, user_path)`; the optional user layer is `<user dir>/limit_policy.json`
  (`rg::user_limit_policy_path`, merged last; a broken file switches the advice off with the loader's message
  naming the file and key, like a broken `advisor.json`). The policy reaches `DynoSweepSpec::policy`
  (so also `solve_remedy`) and `check_driveline_limits`. Findings are never dropped by physics; the generic
  "is this worth telling" filter is `policy.warns(severity)` (default `warn_at` near = everything) and is the ONLY
  severity filter for most kinds. The game adds exactly two things in `advisor.json`: `"min_severity": "never"`
  hides a kind (RevLimiter, FuelCut: the sweep stops 50 rpm under the limiter, so they would only restate the
  rev limit setting) and `"min_severity": "exceeded"` on MapGridEdge (Near the edge of the engine's data is a
  modelling detail, not advice). A kind without `min_severity` is told whenever the policy warns. Refusal
  (`policy.refuses` / `has_refusing_finding`): the garage has NO build refusal driven by physics findings -
  extreme setups are accepted by design and the setup check only advises - so nothing uses it; the only refusals are
  the setup compile errors and the game's own gearbox swap rating check (catalogue `rating_nm`, see
  `rg::check_gearbox_rating`).
- Item format: bold title, the configured value, what the setup achieved and where, an optional suggestion:
  `Excessive turbo boost target` (title, coloured by severity) / `Boost target is set to 15.5 bar.` / `The current setup achieved` /
  `13.8 bar @ 3250 RPM.`. Quantities: bar gauge, RPM, N*m, kW, m/s (piston speed), per-cent.
- Hidden on purpose: RevLimiter and FuelCut (the sweep stops 50 rpm under the limiter, so they would only
  restate the rev limit setting), MapGridEdge below Exceeded (a modelling detail). Not used: any km/h item (no
  finding carries a speed). Skipped templates are listed under `kinds` with `"min_severity": "never"`.
- Gearbox (physics T0): `GearboxInputTorque` reads `Gearbox overloaded` / `Gearbox is rated for 1300 N*m.` /
  `The engine makes 1371 N*m @ 6000 RPM.` / `A stronger gearbox.`, with the Closed remedy line (1.1 x the peak,
  physics' `gearbox_rating_remedy_margin`). It is produced only for a gearbox whose physics file has
  `rated_input_torque_nm` (the peak includes the N2O kit; graded 0-5 % over Near, 5-15 % Limiting, more Exceeded,
  all policy entries). No shipped gearbox file carries a rating yet - the game's `rating_nm` lives in
  `data/gearboxes/gearbox_catalog.json` - so the item does not appear on any shipped car.
- Remedies: a Closed remedy (clutch capacity, gearbox rating, kit jet, bottle) shows its line at once. The wastegate kinds
  (WastegateSaturated, BoostAboveTarget, TurboRotorCap) get a "Find fix" button: `solve_remedy` runs on the
  worker (about 0.6 s on the hyper, Sketch maps) and the item shows the area ratio ("x the stock area"). Knock
  has no solvable remedy, so no button.
- Stage-map coverage: a turbo with an offline stage map gets "Turbo map covers only part of the flow" (Near =
  advice) or "Boost target is beyond the turbo map" (Limiting = warning) with the share of the engine's air flow
  the map covers at the target boost ("covers 75.0 % of the engine's air flow"; physics grades it in 1/16 steps
  and the number depends on the map quality the advisor sweeps with). Text and filters: `build_rules` in
  data/garage/advisor.json. Other engine loader warnings appear as lines of one "Engine data notice" item.
- Threading: `LimitAdvisor` mirrors `GarageDyno` (own thread, debounce, newest request wins, cancellation,
  progress, immutable snapshots only). `RgGarage.get_advice()` / `solve_advice(id)` are thin; `shell_ui.gd`
  polls every 0.2 s, shows "SETUP CHECK - checking... n%" (items of the previous result dimmed) and redraws
  only when the result changes.
- Tests: tests/unit/test_limit_advisor.cpp ([limit_advisor]: config, templates, grouping/merging, variants,
  remedies, thread, the limit policy file / user layer / warn_at filter, the gearbox template and a sweep
  that carries a gearbox rating through `check_driveline_limits`; the [long] case runs the real advisor on
  car_hyper_n2o with the owner setup and stock),
  `garage_test.gd` (advisor available, extreme boost accepted and flagged, panel draws the items).

## Adaptive engine pressure maps (2026-10-08)

Engine-affecting garage edits enable physics_sim's `cycle.pressure_sampling`.
The pressure domain uses the engine's authored lower bound and extends its upper
bound to the requested absolute pressure (ambient plus gauge boost). The game
requests a 2% interpolation tolerance, at most 512 pressure probes and eight
refinement passes. Authored pressure grids provide seeds, not mandatory rows in
every generated table. The native sampler validates each interval at quarter,
half and three-quarter pressures and refines according to actual cycle results
across RPM, including the efficiency map's fresh-air-mass coordinate.

Every computed operating point remains available during generation and is reused
across passes. Pressure-dependent tables choose their retained axes independently;
efficiency retains all its air-mass knots. Adaptive settings and generator version
are included in the map cache identity. Reduction updates the content hash before
writing the cache, and a disk-cache round trip is covered by the native tests.
Budget exhaustion reports a convergence error and the dyno keeps its previous
successful chart. This sampling error control does not extend the ECU's authored
ignition/fueling calibration or prove the cycle model's physical accuracy at
extreme pressures.

Native implementation is in the S: physics_sim checkout, imported into the game
at d2d43f6. Evidence: out/adaptive_pressure_tests.log (65 native assertions),
out/adaptive_pressure_regression.log (1288 existing control/cache assertions),
out/adaptive_game_hardware_tests.log (57 hardware/domain assertions) and
out/adaptive_setup_tests.log (structural checks through 30 bar requested boost).

## Part masses (2026-10-15)

Owner on PR #19: each part has its own mass, a parent carries the cumulative mass of its children, and swaps change the
car's mass through the parts. `rg/part_mass.h` builds the tree from data only (the vehicle file as the setup leaves it,
the engine and gearbox catalogues' `mass_kg`, `data/vehicles/part_masses.json`, the catalog's per-car
`chassis.body_mass_kg`); the drive's chassis mass is the tree's total minus what physics carries itself (wheel_mass is
spin inertia only, so today that is the tank and bottle contents).

Body masses (owner on PR #22) are no longer back-calculated from an old total: each car's `body_mass_kg` is an
independent [AS] bare-shell figure (interior ripped out, no engine assembly, no drivetrain, no wheels) with an itemised
`body_mass_source`. The catalog's `chassis.mass_kg` is the stock tree's dry chassis mass. Stock totals with a full tank:

| car | old chassis | new total | body |
|---|---|---|---|
| car_hyper | 1500 | 1265 | 600 |
| car_hyper_n2o | 1500 | 1284 | 600 |
| car_sedan, car_toyama_xr | 1500 | 1365 | 680 |
| car_sedan_rwd | 1500 | 1303 | 680 |
| car_sedan_awd | 1500 | 1357 | 680 |
| car_sedan_gen_n2o | 1500 | 1326 | 680 |
| car_generic_suv | 1500 | 1511 | 820 |
| car_city | 852 | 909 | 425 |
| car_gt70, car_phoenix_455 | 1282 | 1387 | 665 |
| car_runabout | 644 | 732 | 215 |

Contents mass: every game vehicle file sets physics_sim's `contents_mass` (threshold 0.1 kg), so the chassis body
carries the fuel tank and N2O bottle contents and gets lighter as they are used. The tree shows them as
`fuel_system.tank.contents` and `engine.nitrous_kit.bottle.contents` at the spawn fill (tank `initial_kg`, default the
capacity: a full tank; bottle `initial_kg` of the component or bottle file, else full).

Turbos: a factory forced-induction setup is its own `engine.turbo_system` group (one node per turbo, hot side,
intercooler, charge piping; sized from rotor inertia and air flow, all [AS]). The engine catalogue's mass is the NA base
(the hypercar's V8 175 kg, its twin-turbo system 54 kg), so a turbo conversion or removal changes the mass. A
`turbo_kit` extra under the engine (PR #21) replaces this group.

Weight reduction: `part_choice` options in setup_options.json (group "Weight reduction") swap, lighten (a mass factor)
or remove one interior part per slot; the catalog's per-car `interior` map names the stock parts (masses in
part_masses.json). The model shows the removal where it has a node for the part ("Visual removal" below). A preset's stats mass is its tree with its
`preset_setup` applied.

A drivetrain layout other than the stock one carries a penalty (owner: "the drivetrain needs to run across the entire
car, like in an AWD setup"): the new axle's differential and half shafts and an AWD transfer case come from the file;
the `layout_conversion` node adds 15 kg of mounting hardware, an 18 kg transfer drive for a 2WD result, and a 14 kg
driveline along the car where the new layout has no more propshafts than the stock one. Front-engine examples on the
synthetic car: FWD -> RWD +45 kg, RWD -> FWD +35 kg, FWD -> AWD +87 kg (with a viscous centre / Torsen rear). All
figures are [AS].

The garage shows it under Chassis -> Weight (`RgGarage.get_mass_breakdown(id)`: total, change against stock, body and
groups, the drivetrain's parts).

Visual removal (owner on PR #22: "visually remove elements that were removed for weight reduction"): what the model
shows follows the weight-reduction choices. `data/vehicles/strip_visual.json` (rg.strip_visual/1, `rg/strip_visual.h`,
Godot-free) maps, per model (the .glb file's stem), a fitted part of a slot (`audio` / `removed`) to the model nodes to
hide. `Garage::working_hidden_nodes()` (the open setup) and `DriveSelection::hidden_nodes` (the drive) give the list
(`RgGarage.get_working_colours()` and `prepare_drive()` key `hidden_nodes`); GDScript only toggles visibility:
`model_options.gd apply(root, rig, options, hidden)` in `vehicle_visual.gd` (drive, cockpit included) and
`garage_scene.gd` (car and mirror twin), and shows again what the previous call hid. `cockpit_controls.gd pick` skips a
hidden control, so a removed radio cannot be clicked. A lighter part of the same shape (carbon bonnet, titanium
fasteners, bucket seats) changes nothing on screen; the weave or a paint tint is an open question.
`car_mesh_merge.gd` treats rig nodes with a `strip` extra like option parts (kept as their own group), so a node that
hides is never baked into the body surface.

Per model, `not_separate` in the data says why a slot shows no change: the element is part of the single body mesh
(no node to hide), or the choice is a lighter part of the same shape. What can be hidden today (committed .glb files):
the Pico's touchscreen and steering-wheel audio buttons, the Ventura GT's radio buttons and knobs. The generated cars
need a regenerated model for the rest: tools/cargen now splits out, as static nodes tagged `strip` in the rig,
`seat_driver` / `seat_passenger` / `seat_rear` (Pico), `seat_driver` / `seat_passenger` (Ventura GT), `seat_rear`
(Runabout T), the door cards `door_card_*` (Pico, Ventura GT, as children of the doors), `carpets` (all three: floor
carpet, the Ventura's luggage-deck carpet, the Runabout's floor mat), `sound_deadening` (Pico, Ventura GT: floor pad
under the carpet and rear bulkhead pad, made in `cargen/common.py cabin_shell`), `dash_console` / `dash_glovebox`
(Pico, Ventura GT; the Runabout only `dash_glovebox`, a tool tray) and `spare_wheel` (Ventura GT lying on the luggage
deck, Runabout T upright on the tail; the Pico's space-saver sits under the boot floor and is not modelled). A stripped
dashboard keeps the dash body, gauges, switches, climate knobs and gear lever. The data's `after_regeneration` rules
name them and a model without them ignores the rule. Regenerating (`python3 tools/cargen/build.py`) needs the owner's
checkout with the LFS files; the glb, rig, cockpit and flow files are committed together afterwards. The test
`[strip_visual][data]` checks every catalog car's slots against the data and every live node against the rig file;
`cockpit_test.gd` checks the hiding and the clicks. Thumbnails (`car_thumbnails.gd`) show the stock car.

A weight-reduction option is offered only for a part the car has (owner on PR #29: "the passenger seat should not be
removeable if a passenger seat does not exist"). The rule is the catalog's `interior` map (`rg::part_choice_offered`):
the garage lists the option for a slot only when the car has it, and a saved setup holding a choice for a missing part
is ignored (`Garage::effective_setup` drops it, `compile_setup` skips it). The Runabout T's front bench is one part
(`seat_vintage_bench`, 18 kg, the slot `seat_driver`) with no passenger-seat option; the Ventura GT and the two-seat
hypercars have no `rear_seats` slot (the Ventura's 14 kg of rear-deck trim moved into its body mass, so its total is
unchanged). Tests `[strip_visual][seats]`.
