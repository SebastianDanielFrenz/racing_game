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
