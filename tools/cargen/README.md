# cargen: racing_game's own car models

Generates the Pico, Ventura GT and Runabout T (plan: `docs/new_cars_plan.md`) on
top of physics_sim's asset generator (`external/physics_sim/tools/assetgen`,
imported read-only). Python 3 stdlib for building; numpy/PIL for checks and previews.

```
python3 tools/cargen/build.py [pico ventura_gt runabout_t]   # writes data/models/<id>/
python3 tools/cargen/validate.py [ids] [--determinism]        # assetgen checks + closed shells + flow + cockpit
python3 tools/cargen/preview.py data/models/pico/pico.glb out/cargen_preview/pico   # software renders
```

Per model: `<id>.glb`, `<id>.rig.json` (physics_sim.rig/1), `<id>.flow.json`
(exterior surface for CFD, pinned to the glb's sha256) and `<id>.cockpit.json`
(rg.cockpit/1: every movable cockpit control with its signal, joint mapping and
click box, every screen with its material and pixel size).

Every node is a set of closed, consistently wound shells (validate.py checks it,
same criterion as the hypercar). That is visual closure, not a CFD-ready solid.
