# Aero bake tools

Python3 standard library only. From S:/claude_code/racing_game:

```powershell
python tools/aero_bake/bake.py prepare data/aero/hypercar_bake.json out/aero_hypercar_cases
python tools/aero_bake/bake.py convert data/aero/hypercar_bake.json out/solver_loads.csv data/aero/hypercar_coefficients.json
python -m unittest discover -s tools/aero_bake -p test_bake.py
```

Prepare writes ordered cases.csv and a SHA256 geometry manifest. It does NOT
export a watertight mesh or run CFD. Vectors in cases.csv are vehicle velocity
relative to air in body axes, not the opposite solver inlet velocity.

Solver loads CSV requires the configured axis columns, density_kg_m3,
fx_n,fy_n,fz_n,mx_nm,my_nm,mz_nm. Forces use ISO body +x forward,+y left,+z up;
moments are about reference_point_local_m in the same axes. Convert normalizes
by q*A and q*A*L, produces strict physics_sim.aero_coefficients/1 plus a separate
.provenance.json file, and rejects incomplete, duplicate or nonfinite results.
Last axis varies fastest. Reversed travel normally produces positive Cx.

Yaw -180/+180 and all yaw labels at pitch +/-90 describe identical physical
flow directions: export the same converged physical result for those labels.
Boundary rows must match exactly; the tool never silently smooths/averages CFD
results. Speeds in offline sweeps are positive; runtime handles zero speed.

Record the actual solver/version in the config before conversion. After mesh,
domain, convergence and force/moment review, --validated explicitly records the
operator's attestation in the sidecar; it does not compute that review. Default
map selection requires reviewed data. Keep enabled=false until correct solver
results, geometry and replacement ownership are supplied. The starter sweep
has no ride-height/wing axes and is free-air only. A full-car fixed-wing map
must not masquerade as variable-wing data; dynamic wing configurations require
a wing_offset_deg axis and a wing_surface binding. Use replace_body only for
a body-only CFD geometry with separately authored surface loads.

## Supplied hypercar geometry

```powershell
python tools/aero_bake/geometry.py external/physics_sim/data/models/car_hyper/car_hyper.glb out/aero_hypercar_geometry
python -m unittest discover -s tools/aero_bake -p 'test_*.py'
```

Exports hypercar_rest_iso.obj with named primitives and audit.json. Node hierarchy,
TRS/matrix transforms and interleaved accessors follow the Khronos glTF 2.0
specification: https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html.
Axes follow this asset's rig permutation. Coordinates remain at the rig's ground
origin, NOT chassis COM; keep that distinction in solver moments. Before baking,
apply/review the same wheel-derived body alignment as vehicle_visual.gd and
record suspension compression/ride height and actual aero actuator pose.
Export retains all visual geometry, including interior details; it is input for
solid preparation, not an automatic selection of exposed flow surfaces.

Audit uses configurable --weld-tolerance-m (default 1 micrometre) for edge
identity only and never changes exported vertices. Reports open/nonmanifold
edges, inconsistent winding, duplicate faces and degeneracies per primitive and
combined. Closed edge topology alone does not establish CFD readiness: check
self intersections, internal components, cooling flow, solid volume and mesh
quality independently. Do not fill all holes blindly: wheel arches and intakes
can be intentional. Unsupported compressed/skinned/morphed geometry fails.
Pass --vehicle external/physics_sim/data/vehicles/car_hyper.json to export in
chassis body coordinates. The tool derives the body shift from the mean of wheel
attachment minus suspension rest-pivot positions, just as vehicle_visual.gd does;
steer/wheel nodes also receive their individual residual suspension bias. The
supplied vehicle gives [0.216,0,-0.535] m. audit.json records the source vehicle
hash, offset and origin convention. This is a rest assembly, not a claim of a
particular settled suspension compression/ground clearance or deployed wing.
The audit also groups primitives by node, so material seams are not mistaken for
openings in an assembled part. boundary_components records edge counts, loop
regularity and exact ISO bounds for locating remaining holes. It does not guess
which loops should be capped. Negative-scale transforms preserve outward winding.
## CFD exterior preparation

The supplied asset now includes car_hyper.flow.json, a GLB-hash-bound selection
of final node/material triangle ranges. --flow checks the hash, range bounds,
non-overlap and referenced parts before extracting the exterior candidates.
Hidden cabin/engine geometry and reversed inner skins are omitted.

```powershell
python -m venv out/aero_tools_venv
out/aero_tools_venv/Scripts/python.exe -m pip install -r tools/aero_bake/requirements-solid.txt
python tools/aero_bake/geometry.py external/physics_sim/data/models/car_hyper/car_hyper.glb out/aero_hypercar_flow_final --vehicle external/physics_sim/data/vehicles/car_hyper.json --flow external/physics_sim/data/models/car_hyper/car_hyper.flow.json
out/aero_tools_venv/Scripts/python.exe tools/aero_bake/prepare_solid.py out/aero_hypercar_flow_final out/aero_hypercar_cfd_6mm --voxel-m .006
out/aero_tools_venv/Scripts/python.exe -m unittest discover -s tools/aero_bake -p 'test_*.py'
python tools/aero_bake/bake.py prepare data/aero/hypercar_bake.json out/aero_hypercar_cases
```

prepare_solid.py writes binary hypercar_cfd_rest.stl and a provenance/quality
manifest. All geometry stays in ISO chassis coordinates. Surface subdivision
uses bounded batches (whole-car subdivision consumed excessive memory); raster
caches are keyed by exact source OBJ bytes. It seals small panel seams with
--seam-closing-voxels (0..3, default1) and fills enclosed volume. The supplied
hypercar's central body core is filled between its own floor/roof within0.55m
of the centerline, safely inboard of wheel wells; wing/wheel/splitter height is
never used for that fill. This is an explicitly sealed-cabin/cooling baseline,
not a generic reconstruction of arbitrary cars or cooling ducts.

Half-voxel Gaussian and16 Taubin smoothing passes suppress artificial voxel
roughness. Tiny components below10 cubic centimetres are dropped in this CFD
derivative, not the visual model. Quality gates reject non-watertight/wrong-
winding output, implausibly small solid volume, more than32 retained components,
or excessive bounds/smoothing displacement. Watertight shells alone previously
produced hundreds of fragments and inadequate body volume: they are rejected.

--voxel-m (2..20mm, default6mm) controls the approximation. Review the manifest,
shape and resolution sensitivity before solver use; cfd_validated stays false.
Current6mm STL is about216MB/4.32M triangles and remains in ignored out/.
The8mm comparison merges nearby parts, so use6mm for preparation; this is NOT a
claim of CFD/force convergence. Pose is fixed rest wing/straight wheels/closed
panels. Moving ground, rotating wheels, variable-wing poses and solver domain
remain separate setup. Default bake hashes STL, solid manifest, GLB, selection
and vehicle definition, and requires these prepared outputs to exist.

Optional Blender review render (installed at S:/programs/Blender5.2):

```powershell
& 'S:/programs/Blender 5.2/blender.exe' --background --factory-startup --threads 8 --python-exit-code 1 --python tools/aero_bake/render_solid.py -- out/aero_hypercar_cfd_6mm out/aero_hypercar_cfd_6mm/preview.png
```

Use spaces in Blender flags: --threads 8 and --python-exit-code 1.
The render checks STL hash before import. Visual model is preserved separately.
Algorithm references: https://trimesh.org/trimesh.voxel.creation.html,
https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.binary_fill_holes.html,
https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.binary_closing.html.
## OpenFOAM 14 exploratory mesh/solver cases

The official Foundation OpenFOAM14 package is installed in the existing Ubuntu
WSL distribution. Source `/opt/openfoam14/etc/bashrc` explicitly for each run;
no shell startup files are changed. Installation source:
https://openfoam.org/download/14-ubuntu/ .

Use the installed `incompressibleFluid/motorBikeSteady` tutorial as a template:

```bash
source /opt/openfoam14/etc/bashrc
python3 /mnt/s/claude_code/racing_game/tools/aero_bake/openfoam_case.py \
  --solid /mnt/s/claude_code/racing_game/out/aero_hypercar_cfd_6mm \
  --output /home/sebas/rg-cfd/hypercar-pilot-new --refinement 3
bash /home/sebas/rg-cfd/hypercar-pilot-new/Allrun
python3 /mnt/s/claude_code/racing_game/tools/aero_bake/report_openfoam.py \
  /home/sebas/rg-cfd/hypercar-pilot-new \
  /mnt/s/claude_code/racing_game/out/aero_openfoam_pilot/new_results.json
```

Output directory must be new. Cases run on WSL's native filesystem, preserving
S: source assets and archiving compact reports back to S:. ISO-frame flow and
ground velocity are both (-speed,0,0); inlet is the +X domain boundary. Ground
is 2mm below the prepared surface's minimum Z. Turbulence model is kOmegaSST,
1% inlet turbulence intensity and 0.2m turbulence length scale. Loads are
pressure plus viscous forces/moments about chassis origin, with rho=1.225kg/m3.
The 37x20x10.54m domain, refinement and tolerances are initial pilot values.
`Allrun` requires independent `checkMesh` to report `Mesh OK` before solving.

The pilot deliberately has static wheels, a rest wing and no prism layers.
It tests meshing and signed load extraction; it cannot certify coefficients.
Both manifests/reports explicitly set `runtime_map_eligible=false` and
`validated=false`. No runtime map is written. `report_openfoam.py` sums the
four OpenFOAM pressure/viscous force/moment vectors, rejects malformed samples,
and reports two-window drift and standard deviation. Iteration counts in the
steady solve are pseudo-time, not physical seconds. Convergence requires
stable loads, residual/continuity assessment, mesh/domain studies, boundary
layers, rotating tyre regions, wing poses and geometry review.
The geometry exporter also accepts `--wing-offset-deg` and `--wing-lift-m`.
These move `wing_flap` and `wing_lift` in the same local axes as the visual
adapter, preserve body geometry, enforce authored joint limits and record
actuator pose in `audit.json`. For example, export pitch45deg/lift0.28m to a new
directory, then prepare its own solid. Geometry filenames remain fixed within
each pose directory; provenance carries the pose. Never reuse a rest-wing
STL or hash for another pose. A raised airbrake candidate is in
`out/aero_hypercar_wing45`; its derivative solid is separate from the baseline.
CFD solver worker count is configurable with --workers (1,2,4,6,8); default6.
Meshing remains serial. Parallel flow fields can be reconstructed using
reconstructPar -latestTime. Reports use the newest uninterrupted force segment
after a restart, recording its start and exact force/log/dictionary hashes.
The bake CSV accepts independent wing_offset_deg and wing_lift_m axes; the
height axis varies fastest. Production full-passive maps for lifting wings
need both pose controls sampled and a wing_surface binding. High-speed cases
above roughly Mach0.3 also require a compressibility assessment; these initial
incompressible50m/s pilots do not certify the hypercar's maximum-speed aero.
