# Offline CFD aerodynamic maps

Owner decision 2026-10-07: lower drag must be earned by a redesigned shape.
No arbitrary reduction of the hypercar coefficients; keep the authored analytic
model until a verified map is enabled. CFD runs offline, never inside a frame.

## Implementation stages

1. Runtime foundation: physics_sim owns validated coefficient maps and dense
interpolation. Game owns selection/provenance and tooling. Body-axis forces
Cx/Cy/Cz and roll/pitch/yaw moments Cl/Cm/Cn use ISO +x forward,+y left,+z up;
q=0.5*rho*V², force=q*A*C, moment=q*A*L*C about declared reference point.
Interpolate signed yaw, pitch and speed; optional ride height/actual wing angle.
Full yaw coverage includes backward/sideways slides. Wind and wakes alter the
local air-relative velocity BEFORE lookup. Zero speed gives zero q.

2. Offline preparation: identify/hash geometry and export a solver-ready solid.
The current visual GLB is not proof of a watertight CFD body. Close openings,
model floor/cooling paths/wheels and record what is omitted. Moving ground and
rotating wheels for road cases; mesh/domain independence and convergence checks.
Choose an available external solver (e.g. OpenFOAM); do not present generated
sweep files or analytic samples as CFD results.

3. Baseline sweep: coarse free-air yaw/pitch/speed cases first. Refine forward
small yaw and pitch where gradients are steep; refine extreme-angle regions
for sliding/backwards/inverted cases. Separately ground/wing sweeps near normal
attitude. Use dense grids per validated scenario, never silently fill missing
CFD cases. Record density, viscosity/Reynolds, boundary conditions, reference
area/length/point, actual wing geometry and solver/model/version.

4. Shape redesign and comparison: run baseline and proposed body under identical
conditions; compare drag, cooling massflow, downforce and pitch/axle balance.
Reject improvement claims supported only by coefficient edits. Select a map
only after acceptable force/moment behavior and geometry provenance checks.

5. Runtime acceptance: interpolation/periodic seam/reversed flow/zero speed,
invalid-data rejection, wake-relative sampling, ground misses, pure moments,
replacement ownership and reset/state hash. Focused tests first; full traffic
FPS and owner driving afterward. Map selection must survive flat/real-world
reloads without regenerating maps or rereading mutable physics state.

## Model ownership

A body-only CFD model replaces analytic body loads and preserves separately
modelled surfaces. A full-car passive map replaces body AND wing/underfloor/
splitter loads, while keeping actuators and powered fans; otherwise downforce
and drag are counted twice. Never multiply tabulated ground effect by the old
analytic ground factor again. Grounded maps require a matching nearby surface;
missing/incompatible support uses explicit free-air data or analytic fallback.
Close multi-vehicle interference remains a later CFD scenario family; existing
wake deficits feed the single-vehicle maps initially and are an approximation.

## Deliverables and current limits

Configuration: data/aero/hypercar_bake.json (disabled, no coefficients supplied).
Offline tool: tools/aero_bake, solver case CSV and checked force normalization.
Native map schema/loading/runtime is supplied by Physics Worker main and
integrated via a physics_sim pin. No CFD solver run, validated new geometry,
manufacturer coefficients or earned drag reduction is claimed by foundation
implementation. Ground/wing maps need appropriate samples before activation.

Game selection: data/aero/hypercar_map.json, strictly disabled by default.
Enabling requires the matching .provenance.json sidecar with kind=cfd,
validated=true, exact map SHA256, and unchanged geometry file SHA256 hashes.
The importer emits this sidecar; --validated is an explicit operator review
attestation, not automatic CFD verification. Session validates on startup and
attaches to a cloned VehicleDesc, including warm reloads. Runtime telemetry
exposes map activation, missing-ground fallback, clamp mask, actual inputs and
coefficients through get_aero_state()/owner pings. Full-passive replacement of
an adjustable wing requires wing-angle samples; a fixed-wing baseline alone
cannot silently replace the hypercar's airbrake behavior.

## Foundation status (2026-10-07)

Runtime and game integration use physics_sim a949346b3d4e0d63a775951a2bde8edbbcffb9ba, fetched from S:/claude_code/physics_sim. Game selection checks passed (8 assertions); Python bake checks passed (8 cases), including exact exported-byte hashing on Windows. The native worker reports 6492 assertions across 23 aero cases. A 195-case baseline sweep manifest is prepared in out/aero_hypercar_cases; no solver has run. Stage 1 is complete in source; stage 2 still requires a CFD-ready solid and an actual solver setup. Default selection is disabled.

## Supplied hypercar baseline preparation (2026-10-07)

The owner selected the existing supplied car_hyper shape as the first baseline.
Added tools/aero_bake/geometry.py: deterministic rest-pose GLB-to-OBJ export in
ISO axes, configurable topology audit and part-by-part report. Current output:
out/aero_hypercar_geometry/hypercar_rest_iso.obj and audit.json (source/art unchanged).
113,960 triangles; dimensions approximately 4.702 x 2.140 x 1.155 m. Combined
1-micrometre audit: 3300 boundary edges, 244 nonmanifold edges, 224 degenerate
triangles; no duplicate faces or inconsistent winding edges. These are mesh
preparation findings, not CFD results. Repair body panels/closed seams while
retaining intended intake and wheel flow; distinguish interior trim from the
flow boundary. Review intersections and cooling paths separately after repairs.

The exporter deliberately retains the rig ground origin and rest actuator pose.
The game aligns the art using wheel attachment differences (vehicle_visual.gd);
solver preparation must explicitly reproduce that body/COM relationship and
chosen ride height before importing moments. Do not silently interpret ground
origin moments as chassis COM moments. Full passive runtime maps still require
actual wing-angle sweep/binding before activation.

13 Python tests pass: accessor stride and hierarchy, coordinate permutation,
closed/open topology, reversed winding, duplicate/degenerate faces, bake/load
conversion and exact hashes. The available WSL Ubuntu has no foamRun,
simpleFoam or surfaceCheck; no Blender executable was found in the normal
Windows installation path or PATH. Stage 2 has export/audit complete; solid
repair and CFD solver installation/configuration remain outstanding. There are
still no measured coefficients and the selection remains disabled.
## Model preparation status (2026-10-08)

Integrated physics_sim87b0f41be9a872161ad938b84468de552ec506ac from S:.
Visual source now has stitched inner skins/returns, backed lenses/grilles,
closed tyre/rim profiles and an exterior-selection sidecar. Independent game
GLB audit:179324 triangles, zero boundary/nonmanifold/degenerate/duplicate/winding
errors per assembled node and combined. Godot import finds both doors, frunk,
engine cover, wing, four wheels and driver eye. Native deterministic rebuild
and Blender closed/open-pose checks pass. Art shape/joint dimensions preserved.

Game export applies measured wheel-derived chassis alignment [.216,0,-.535]m.
Hash-bound car_hyper.flow.json selects94204 exterior candidate triangles, omitting
hidden cabin/engine geometry. Game-owned prepare_solid.py produces a separate
filled CFD exterior: central body/cabin core filled from body-only floor/roof
columns within0.55m of the centerline, seam closing, enclosed-volume flood fill,
Gaussian/Taubin smoothing. Wheel/aero heights cannot fill that core. Backed
cooling/no duct-flow assumptions and all source hashes/tool versions are explicit.
Whole-car surface subdivision was replaced with bounded batches to limit memory.
Watertight but fragmented/thin-shell candidates fail volume/component gates.

Final prepared output: out/aero_hypercar_cfd_6mm/hypercar_cfd_rest.stl (about216MB,
4317598 triangles), manifest.json and preview.png. Six-millimetre candidate:
watertight/consistent winding,5 retained components,3.88222m3 solid volume,
5.34mm maximum overall-bounds drift. Eight-millimetre comparison:3 components,
3.96505m3,6.58mm drift; volume differs2.13% and nearby parts merge, so6mm is the
preparation baseline. These are geometry checks, not mesh/force convergence.
STL remains a derivative approximation: inspect small features and improve
resolution as the solver requires. cfd_validated=false is deliberate.

17 focused Python tests pass, including open/closed volume filling, small-seam
closure, preserved open volume with closing disabled, synthetic solid topology,
exact STL/cache hashes, source-bound selection, mirrored transforms and chassis
alignment. Default bake config now binds STL+manifest+visualGLB+selection+vehicle;
195 baseline cases were prepared again using those exact inputs. No solver run,
CFD coefficients or drag reduction. Stage2 visual repair and preliminary solid
preparation are complete; stage3 begins with fluid-domain/moving-ground/wheel/
wing-pose setup and solver/refinement/convergence work. Engine runtime is unchanged.
## First solver pilot (2026-10-08)

Installed the official OpenFOAM Foundation14 package (20260724) in Ubuntu26.04
WSL. The game continues to use its existing authored aerodynamic coefficients.
Added `openfoam_case.py` and `report_openfoam.py`: source-hash-checked case
preparation from the installed steady motorBike tutorial, ISO-frame moving
ground/inlet, kOmegaSST, chassis-origin pressure+viscous load extraction, strict
independent mesh-validation gate and pilot-only reports. Actual cases execute
on WSL ext4; compact evidence is copied back under `out/aero_openfoam_pilot`.

OpenFOAM surfaceCheck independently confirms a closed surface/five components.
First175103-cell fluid mesh had two boundary faces above checkMesh's skewness
limit, despite passing the mesher's looser default. Stopped that solve and
limited boundary skewness to4; replacement mesh passes checkMesh: max skewness
3.88822, nonorthogonality below65degrees. Completed300 initial steady iterations
and a restarted300-iteration load-sampling segment. Last50 samples have about
3.9N drag standard deviation, but lift/side force still fluctuate substantially.
This demonstrates meshing/load extraction, not force convergence or certified Cd.
Finer refinement4 case is a separate sensitivity experiment.

Geometry export now supports joint-limited wing pitch/lift using the same axes
as vehicle_visual.gd, records the actuator pose and leaves body geometry intact.
Exported the raised45degree/0.28m airbrake pose for separate solid preparation.
Tests cover rotation direction, body isolation, finite values and joint limits.

Stage3 has an executable first meshing/solver workflow. Remaining: semantic
rotating-tyre patch regions, boundary-layer/y+ assessment, physical ground-contact
and ride-height treatment, domain/mesh/time convergence, cooling assumptions,
wing-pose/ride-height/speed sweeps and the final geometry review. The pilots use
static wheels, fixed wing pose, a2mm numerical ground gap and no prism layers.
Every pilot report has validated=false/runtime_map_eligible=false; no runtime
coefficient map is produced or enabled. See tools/aero_bake/README.md for commands.
Refinement4 completed:875527 cells, checkMesh passes (max skewness3.974),
final flow reconstructed after a six-worker continuation. Final50-sample mean
drag magnitude rose from1281N to1502N (about17%); downforce rose from108N
to770N. Different transient averaging segments and missing boundary layers
preclude a convergence claim. Archive:out/aero_openfoam_pilot_r4. Raised wing
45deg/lift0.28m solid passes watertight/winding/volume/bounds gates:
3.88278m3,five components,5.34mm bounds drift; it has not been flow-solved.

Found independent wing-height state missing from the original five-axis map.
Offline sweeps now accept wing_lift_m; full-passive selection requires height
samples for lifting wings. Integrated physics_sim45d5ef5a10744da725b0c60de6a707f04b7a4b08: actual wing-height
interpolation is an optional sixth axis; legacy table behavior/hash are preserved.
Native27 aero tests pass. Game activation and rolling-selection tests pass
(39 assertions); Godot startup checks pass and the S: DLL is installed.

Aero preparation now has22 focused Python tests (bake, geometry, solid and
force parsing). The raised-wing solid and refinement studies remain separate
from the rest baseline provenance and disabled game map. Production high-speed
cases need compressibility assessment in addition to the listed checks.
