# Dated status notes (history)

Dated status notes moved out of PLAN.md on 2026-10-18; history, not current instructions.

## Notes that followed the References section (verbatim)

VR/audio integration (2026-10-03): physics_sim master pin ba47951 (fetched
from S:/claude_code/physics_sim) retains the terrain boundary fix and adds
the reusable Windows spatial backend. The supplied car_hyper remains the
default. tools/run.ps1 -VR launches OpenXR Vulkan Mobile; tracked cockpit
and FreeCam rigs share camera-director rebasing. Tyre and eligible live-engine
sources use listener-relative Windows objects with Godot 3D fallback.
Setup, hardware limits and verification: docs/vr_audio.md. The committed
hypercar torque-map engine has no live cycle voice; owner turbo/audio-bank
work remains pending. This presentation work does not change G3-C progress.

Latest physics upgrade (2026-10-03): master pin 2f4599c from
S:/claude_code/physics_sim includes 40923cf's supplied simulated twin-turbo
car_hyper and live engine voice, plus the embedded CMake-path correction.
The default car is unchanged by name and now loads these new library data.
Gauge metadata accepts SimulatedEngineDesc as well as torque-map engines.
Earlier notes about the committed hypercar lacking a live voice are superseded.
G3-C road work is unchanged.


2026-10-03 B8 integration: level-0 terrain now shares road-profile surfacing
between physics and rendering. Separate bridge slabs and lower tunnel floors
have native mesh collision; lower-layer OSM tunnel crossings also supply
highway roofs when the upper ways lack bridge tags. All seven owner-reported
crossings have explicit structure-ID and collision checks. Game-side bounded
reconstruction handles selected source declines and split bridge ways; it
does not complete G3-C's original source-class acceptance. General terrain
smoothing is configurable (enabled/radius_m/strength/passes), halo-correct,
and preserves NoData. The complete B8 height comparison and three-depression
hypercar contact drive pass. See docs/road_surfaces.md for settings, policies
and remaining G3-D/E and G3-F/R3 work.


2026-10-03 building presentation: native OSM footprints, multipolygon courtyards,
recorded heights/level estimates and replaceable batched extrusion presentation
are integrated. Ordinary buildings stream nearby; >=50 m skyline buildings remain
visible up to a configurable 40 km, subject to imported coverage and occlusion.
Worker decoding/tessellation and bounded uploads avoid blocking world startup.
Building collision, architectural roof reconstruction, premade asset matching and
imagery-derived appearance remain later work. Both appearance strategies may be
combined (docs/buildings.md). This feature does not complete the outstanding
G3-C source acceptance or G3-D/E and G3-F/R3 road work.
Physics is pinned to requested e3e1e886b23b7db1c06a9cee4eb469c68452483b from S:.
Reset/start coordinates are configurable and currently target the B 8 north-west of Unterliederbach.

2026-10-03 road definition: profile-aligned asphalt overlay, lane/edge paint,
explicit tagged turn arrows and a configurable gentle shared-heightfield verge
are integrated. Background geometry generation and bounded tile uploads preserve
startup streaming. This is a presentation increment; surveyed junction marking
layouts and the remaining G3 source/road acceptance steps are still outstanding.

2026-10-03 owner B8 rollover: identified bare-earth terrain protruding through an inferred tunnel roof near 50.1361,8.4860. Upper-deck terrain clearance, C1 approach grades and shared roof/marking heights are fixed; focused synthetic and exact-location geometry regressions pass. Driving acceptance remains with the owner.


Aerodynamics integration (2026-10-03): physics_sim upgraded from S: to
8b04f0e5a8aa288c0297ed144906cf2918cece27. Reusable signed lift/drag, body moments,
wind, finite drafting wakes, local ground effect (including inverted operation),
rate-limited active wings and energy-budgeted suction fans are implemented.
Game integration supplies configurable atmosphere/wind and hypercar cruise/airbrake
policy; actual wing pitch/lift and aero telemetry use frame snapshots and bookmarks.
See docs/aerodynamics.md and data/world/environment.json. Stock hypercar has no fans.
Aircraft controls and traffic actors remain future gameplay work; this is a
coefficient model, not CFD. Native owner reports 58 focused tests / 1372 assertions.


2026-10-03 aero performance regression: reproduced 105.6 ms ticks from tiny
altitude changes rebuilding native engine steady tables; ground probes cost
0.001 ms. Physics upgraded from S: to 5ec8b7d0a39953b0bfe8f57e1abc3808b5b37ede to remove the
hot-path rebuild while preserving continuous atmosphere. Local real-terrain
hypercar regression is `[aero_perf]`; see docs/aerodynamics.md.

2026-10-03 hypercar engine calibration: physics pin bc6fdc41db4675e3bb56c9f1cb8b418cbc9724fd from S:.
Authored breathing/turbo/cooling/ECU calibration now produces net 1000.2 kW
at 7500 RPM and 1371.0 Nm at 6000 RPM; fixed bore/stroke/compression/boost/E5/limiter
and 1500 kg game mass preserved. See docs/hypercar_engine.md for same-rig before/after
figures and limitations. Native focused dyno validation; road acceptance with owner.

2026-10-03 NPC drafting truck prototype: T places/replaces one moving truck
45 m ahead, Shift+T removes. Background OSM profile routing, conservative
rightmost-through-lane policy, one-way direction and speed/curvature/end braking;
kinematic collision body and native truck WakeSource drive actual hypercar aero.
Frame-latched visual/status/bookmark data. See docs/npc_truck.md. General traffic
AI, full truck drivetrain/articulation and imagery-derived lane refinement remain future work.

2026-10-03 future NPC traffic requirements: player-centered population radius,
minimum spawn distance, unobstructed rightmost-lane placement, live density,
strict applicable speed limits, 80 km/h truck and randomized 80-250 km/h car
fallbacks on roads without limits, physics/weather/surface-aware corner speed,
and public-road highway whitelist plus access restrictions. Planned, not yet
implemented. Specification and sequence: docs/npc_traffic.md.

2026-10-04 adjustable cockpit seat: default eye 20 cm forward, F6 live fore/aft,
height and lateral UI, saved per vehicle. Shared eye anchor updates desktop/XR
cockpit rigs; brake held and look disabled during adjustment. See docs/seat_adjustment.md.

2026-10-04 low-RPM WOT clutch flare: physics pin 60d60fa09f48e38c5fc382b0a082a1407d94ff6c
from S: carries live boosted engine torque into auto-clutch rolling reengagement.
Native matching reproduction re-locks within 2 seconds; 14,544 focused assertions
passed upstream. Game DLL rebuilt; exact owner road replay remains owner acceptance.
Evidence: external/physics_sim/HYPER_CLUTCH_FLARE.md.

2026-10-04 NPC traffic first implementation: automatic real-world population of
lightweight cars/trucks, background connected-profile routing to random reachable
residential-building/public-parking destinations, native collision and drafting
wakes, rightmost usable lanes, public-access/one-way/speed rules, obstruction
checks and spacing. F7 exposes live saved density/radius/exclusion/grip settings.
Arrival currently stops at the nearest ordinary road point then despawns. Out-of-view
retirement discards identity permanently. Defaults: 4/lane-km, 600m radius, 100m
exclusion, 24 vehicles. See docs/npc_traffic.md for current limits and next stages:
shared weather/physical surface grip, junction conflict handling, precise lane
turn rules, parking maneuvers and car-type target distributions.

2026-10-04 NPC lifecycle correction: F no longer clears traffic; actor motion is
frozen during relocation priming and resumes on the same trip. R still clears.
Runtime planning now uses nonblocking published geometry only, indexed destination
lookup and cancellation throughout CPU work, preventing the old distant-derivation
wait from NPC shutdown/reset. Scan timing/population telemetry added. Focused
flip/cancel/traffic/drafting checks: 40 assertions, 5 cases; owner road retest pending.

2026-10-04 traffic settings apparent freeze: owner log continued at ~240 Hz with
frozen=no and player stationary. F7 inadvertently reused the seat panel's modal
full-brake/input-suppression group. Traffic settings now have a separate mouse/UI
guard: driving remains active, camera look is suppressed while using the panel.
Godot script checks pass. Owner log preserved in out/traffic_settings_owner.log.

2026-10-04 NPC population scale: removed adapter cap 24. Defaults are now 2048
maximum, 120/lane-km and 1200m radius, with F7 cap adjustable to 4096 and density
up to 1000/lane-km. Saved settings migrate once. Staged 512-trip batches, four
spawns/tick, spatial neighbor checks/staggered 20 Hz probes, nearby-only wakes and
instanced 22-part rendering replace the main quadratic/spawn/render bottlenecks.
47 focused assertions pass; headless renderer instantiated 2048 cars + 256 trucks.
Full-game performance at high populations remains owner acceptance.

2026-10-07 offline CFD aero-map plan: runtime interpolates offline body-axis force/moment coefficients, with explicit model replacement and provenance. Authored hypercar coefficients remain unchanged until validated results are available. Implementation stages and acceptance: docs/aero_cfd_plan.md; bake configuration: data/aero/hypercar_bake.json.

2026-10-08 hypercar model preparation: integrated source visual closure repairs (physics_sim 87b0f41); independent audit has zero topology defects. Prepared separate 6 mm CFD exterior with source hashes, quality gates and review render; 17 Python tests and Godot asset import pass. 195 cases now bind the prepared STL and provenance. Solver/domain/rotating-wheel/wing sweeps and CFD convergence remain next; no coefficients changed. See docs/aero_cfd_plan.md.

2026-10-08 configurable rolling resistance: physics_sim accf416 integrated; physics.rolling_resistance_model defaults to quadratic, with fourth_power restoring the original MF law. Rated-speed calibration, cached car switching, parser validation and Godot startup checks pass; installed S: DLL. See docs/rolling_resistance.md.

2026-10-08 CFD stage3 pilot: OpenFOAM14 installed in WSL; closed-surface audit, independently validated fluid mesh and first real pressure/viscous loads. Added reproducible pilot-only case/report tools and joint-limited wing-pose export. Refinement sensitivity, rotating wheels, boundary layers and convergence remain; authored runtime coefficients unchanged. See docs/aero_cfd_plan.md.

2026-10-08 active-wing CFD readiness: integrated physics_sim45d5ef5 optional wing_lift_m axis, exported and solid-prepared 45deg/0.28m airbrake pose. Full-passive game maps require lift-height samples for lifting wings; cached rolling selection and activation gate tests pass. Two mesh pilots completed (175103/875527 cells); loads are not converged, so CFD coefficients remain disabled.


## Moved from PLAN.md section 12 (verbatim)

### G3 status entries of 2026-10-02 and 2026-10-05

G3 status (2026-10-02): roads.graph and deterministic horizontal/vertical
construction are implemented. Junctions use shared sloping DEM planes;
degree-two bridge/tag boundaries use bounded approach-based deck estimates.
The marked A66 bridge is accepted by the profile builder; this is estimated
geometry, and real bridge collision acceptance remains pending. The previous
grade-policy question came from treating an underpass DEM drop as a deck slope
and is withdrawn. G3-C remains in progress: complete profile acceptance.
roads.geom codec, staged dependencies and opt-in server/bake serving are
implemented. Numeric incline supplies a bounded soft slope target while shared
boundaries stay fixed. G3-D/E carving, junction surfaces,
ribbons, markings and bridge bodies, and G3-F/R3 route acceptance remain open.
Physics fixture fix bd41ac4 is integrated.

G3 status (2026-10-05, supersedes the open list above): the owner restarted G3/R3
("Start work on G3/R3") and it continues from geo2map street fidelity
(geo2map docs/street_fidelity/plan.md). Done in geo2map: P1 lane-aware
cross-sections, roads.geom encoding 6, carve footprint/lane API, carved traffic
calming (vault G2M-016); P2 = G3-D design (p2_junction_design.md) and slices 1-7 -
rules, the canonical road height, junction rings/trims/clusters, junction heights
and raised tables, profile completion tier (relaxed fits served flagged
"completed", owner 2026-10-05), crossings/stop lines/signal posts (vault G2M-017).
Owner decisions: endpoint heights win over class grade (over-steep = grade
exception, never clamped); carving moves to the server with G3-D and the game then
deletes its own carve step with no fallback; no zero-gap gate for that switch.
Open: P2 slice 8 (roads.geom encoding 7) in progress, slices 9-12 (g2m.terrain.carved,
terrain.class, wiring, census), G3-C profile acceptance, the racing_game carve
switch, G3-E ribbons from roads.geom + P3 markings + bridge bodies, G3-F/R3 route
acceptance.

### S1 open item, N2O measurement (2026-10-14)

S1 open item DONE (2026-10-14, measured on Linux, release build; `tests/unit/test_nitrous_gain.cpp`, hidden `[.][nitrous][gain]`): armed vs unarmed, WOT, rolling start in the held gear at 2500 rpm on the flat world, auto-shift off, gear held via gear_request, window 2.0 s.

| car / gear | dv 2 s unarmed | dv 2 s armed | rpm 2500 -> 6000 unarmed | armed | max boost unarmed | armed | first spray |
|---|---|---|---|---|---|---|---|
| car_hyper_n2o, 3rd (Sport mode) | 6.89 m/s | 12.03 m/s (+74.7 %) | 3.68 s | 2.95 s | 1.92 bar | 2.55 bar (1.78 at 2 s) | 0.94 s (rpm reaches the 3000 floor) |
| car_sedan_gen_n2o, 2nd | 8.01 m/s | 8.50 m/s (+6.2 %) | 3.56 s | 2.98 s | no turbo | no turbo | 0.38 s |

Findings: (1) the hypercar only sprays in the Sport drive mode (`drive_modes_car_hyper_n2o.json`: `nitrous_arm_allowed` false in Eco and in the default Normal): armed in Normal the HUD shows ARMED but the solenoid never opens (measured: identical to unarmed, 0 s spray) - confirm the owner wants a silent block in the default mode. (2) The boost overshoot caveat still holds: 2.55 bar max with spray against the 1.8 bar target. (3) The sedan's 37 kW shot is small: +6 % dv over 2 s, but 0.6 s faster to 6000 rpm. The hyper's 2 s dv includes 0.94 s of unsprayed turbo lag from 2500 rpm (spray needs 3000 rpm).
Known physics caveat (kept): with spray the hyper boost can overshoot 1.8 bar and the turbo can sit on the rotor speed cap at 6000-7000 rpm. Earlier rough numbers on the pre-606c428 hyper: +25.9 % dv in 0.6 s.

### R6b garage slice (2026-10-08)

- R6b garage slice (2026-10-08): implemented compatible sedan I4/I6 engine reference selection, standard/nitrous hyper turbo configuration cards, illustrative component images, installed-engine limiter/throttle tuning with RPM/ms readouts, and Save/restart/Drive materialization. Unknown/cross-family choices are rejected; nitrous turbo hardware retains its larger wastegate. Native checks: 81 focused engine assertions, 463 existing setup assertions; garage flow: 151 checks. Changes apply on Drive. Live World turbo replacement, ECU map parts, engine design/internals and mesh/mass changes remain pending. The separate Step 1 performance benchmark/cache remains pending. See docs/garage_ui.md.
