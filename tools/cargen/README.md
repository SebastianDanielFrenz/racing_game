# cargen: racing_game's own car models

Generates the Pico, Ventura GT and Runabout T (plan: `docs/new_cars_plan.md`) on
top of physics_sim's asset generator (`external/physics_sim/tools/assetgen`,
imported read-only). Python 3 + numpy for building (normals); PIL for previews.

```
python3 tools/cargen/build.py [pico ventura_gt runabout_t]   # writes data/models/<id>/
python3 tools/cargen/validate.py [ids] [--determinism]        # assetgen checks + closed shells + flow + cockpit
python3 tools/cargen/preview.py data/models/pico/pico.glb out/cargen_preview/pico   # software renders
xvfb-run -s "-screen 0 1600x900x24" godot --path game --resolution 1600x900 \
    --script $PWD/tools/cargen/godot_shots.gd -- $PWD/data/models/pico/pico.glb out/cargen_godot pico_
                                                              # garage-style renders through Godot itself
```

Per model: `<id>.glb` (LOD0), `<id>_lod1.glb`, `<id>_lod2.glb`, `<id>.rig.json` (physics_sim.rig/1,
with a `lods` list and suggested `quality_tiers`), `<id>.flow.json`
(exterior surface for CFD, pinned to the glb's sha256) and `<id>.cockpit.json`
(rg.cockpit/1: every movable cockpit control with its signal, joint mapping and
click box, every screen with its material and pixel size).

Every node is a set of closed, consistently wound shells (validate.py checks it,
same criterion as the hypercar). That is visual closure, not a CFD-ready solid.

Tessellation and LODs (`cargen/quality.py`): one parameter set per LOD (chord tolerance 0.35 / 1.2 / 6 mm,
body station step and points around, detail level). Every revolved or round part gets its segment count from
its radius and the tolerance (assetgen's lathe/tube/cylinder/circle/rrect/rounded_box/ellipsoid are routed
through it), so nothing is faceted at garage distance and lower LODs come from the same code. `Q.coarse(k)`
loosens the tolerance for the cabin seen through glass. LOD1 drops fine detail (tread blocks, lettering,
plate characters, disc slots); LOD2 also drops the cabin furniture and cockpit-control geometry (nodes kept).
All LODs share node names, pivots, joints and material names. Normals are recomputed by angle (48 deg) after
the build (`cargen/normals.py`), so sharp profile steps shade crisp and lathe seams do not show.

Detail parts (`cargen/detail.py`): lamps with housing, reflector bowls / projectors around the emissive
elements and a domed lens (`lens_clear`/`lens_red`/`lens_amber`), plates with recess, blue band and raised
characters, egg-crate grille bars in front of a dark mouth, rubber/chrome window seals, handle recesses,
plan-view offset paths for bumpers. Tyres have directional tread blocks and moulded sidewall lettering;
ventilated discs carry slots.
