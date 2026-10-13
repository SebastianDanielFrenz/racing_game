# Cockpit parts generator (P1)

Run from the racing_game repository in PowerShell:

```powershell
$env:PYTHONDONTWRITEBYTECODE = '1'
python tools/cockpitgen/build.py
python tools/cockpitgen/validate.py
& 'S:/programs/Blender 5.2/blender.exe' --background --factory-startup --threads 8 --python-exit-code 1 --python tools/cockpitgen/render_sheet.py -- out/cockpit_parts_sheet.png
& 'S:/programs/Blender 5.2/blender.exe' --background --factory-startup --threads 8 --python-exit-code 1 --python tools/cockpitgen/render_sheet.py -- out/cockpit_parts_posed.png posed
```

`build.py` imports the assetgen library from the pinned, read-only
`external/physics_sim/tools/assetgen`. Generation uses only Python's standard
library; Blender is an independent development-time importer and renderer.

Inputs: `data/cockpits/parts_spec.json`. Outputs:
`data/models/cockpit_parts/cockpit_parts.glb` and `cockpit_parts.rig.json`.
The rig follows `physics_sim.rig/1`, with an additive `cockpit_parts` catalog.
Geometry is in metres. All authoring/rig axes are ISO (+X forward, +Y left,
+Z up); GLB joint axes are converted by assetgen to glTF (+Y up).

Each `part_<id>` tree has its mounting origin at zero; all parts overlap in the
library. Instantiate a selected tree, not the whole GLB. Apply the receiving
socket's normal/up frame to the root. The wheel-mounted parts travel with the
wheel socket. No car model, CFD sidecar or physics code is changed.

The catalog contains TC with five separate LEDs, LC, aero, A/M, four separate
drive-mode buttons, hazard, start/stop, EPB rocker, guarded N2O toggle, sequential,
six-speed H-pattern and automatic selector levers, left/right paddles, left/right
stalks, light dial, horn pad and a blank cap. The selector has independent P and
unlock buttons; the sequential knob has P and A/M buttons. The N2O cover and
switch have separate pivots and hit volumes.

`ctl_*` joints provide limits, detents and spring-return metadata. `hit_*` empty
nodes provide local box volumes, poke/grab points and press axes. Interaction
vectors in extras explicitly use ISO coordinates; they are not implicitly glTF
vectors. Defaults, travel thresholds, debounce and haptic timings come from the
spec. This metadata declares intended interaction; no action handling is added
by this increment. Action names and fitted-state wiring are resolved in C0/P2.

`lamp_*` materials are unlit in the GLB, with on colour/strength in rig `lights`.
Legends use extruded stroke geometry, so there are no font or texture dependencies
at runtime. Preview typography outside the parts is Blender text and is not
included in the shipped GLB. The posed sheet exercises joint movement without
changing the rest-pose asset.

`validate.py` checks structure, identity rest rotations, mounting origins, axis
conversion, interactions, lamp materials, finite/nondegenerate geometry, unit
normals, the configured triangle budget, and byte-identical regeneration of both
outputs. The Blender renderer independently checks all rig nodes survived import.

This is the reusable asset increment. Car sockets/furniture (M1), game-side
placement/fitted blanks (P2), clicking (P3), and controller/hand interaction (V1/V2)
are separate work. No in-car integration or reach audit is claimed here.
