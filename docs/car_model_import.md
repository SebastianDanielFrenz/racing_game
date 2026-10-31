# Importing the downloaded car models

The 22 CC BY 4.0 models in `data/models/downloaded/` (see `ATTRIBUTIONS.md` there and `docs/model_lfs.md`) are
raw Sketchfab exports. `tools/carimport/` surveys them and turns a chosen one into a rigged game model.

## Tools

- `python3 tools/carimport/survey.py [--json out.json] [files]`: the table below (one row per model).
- `python3 tools/carimport/carimport.py [ids]`: imports the cars listed in `tools/carimport/cars.json` into
  `data/models/imported/<id>/<id>.glb` + `<id>.rig.json` (physics_sim.rig/1) and writes the game vehicle file of
  each; `--check` only re-runs the checks on the written files. Needs Python 3 with numpy (Pillow for the survey).
- `analyze.py` (geometry, axes, wheel detection), `glb.py` (minimal GLB reader/writer).

What the importer does:
- axes and units: every node matrix applied (Sketchfab files carry their unit scale in the root node), source axes
  from `cars.json` (`forward`, `up`) or guessed (up +Y, forward = longer horizontal axis, sign from part names);
  output in the rig frame (ISO x fwd, y left, z up; the glb in glTF axes gltf(x,y,z) = iso(y,z,x)), origin on the
  ground below the wheels, x = 0 midway between the axles;
- wheels: nodes named wheel/tire/tyre/rim (corner from the position), else geometric clustering (connected
  components that are round in side view, touch the ground and are narrower than tall, one per corner, plus every
  component inside it; such meshes are split into wheel and body meshes); each wheel goes under
  `susp_<C>` / `steer_<C>` (front) / `wheel_<C>` with all three pivots at the wheel's bbox centre; brake parts
  (`brake_parts` regex, calipers) under `steer_<C>` / `susp_<C>` so they steer but do not spin;
- baked shadow / ground planes are dropped; materials, textures and vertex data are reused byte for byte;
- `asset.copyright` / `asset.extras` keep the original credit plus the source file's SHA-256 and the modification
  note; `attributions.json` gets a `derived` record per import (CC BY: indicate changes);
- vehicle file: a copy of the base vehicle file (`cars.json` `vehicle.base`) with the wheel positions of the model
  (wheelbase scaled keeping the base car's CoG fraction, track from the model) and a tyre whose radius matches the
  modelled wheel. The physics follows the art here because only four numbers change; the art is untouched.
- `vehicle_visual.gd` / `garage_scene.gd` scale a wheel of any rig by physics tyre width / the rig wheel's
  `tyre_width` (the stock tyre), or its `width` where the rig has no `tyre_width`, so the stock tyre shows the art
  unscaled. The rig key `wheel_width_scale: "rig"` is still written but no longer read.

Checks (exit 1 on failure): four wheels, every pivot within 1 cm of its wheel's centre measured in the written
file, round wheels, wheel bottoms on the ground plane (1 cm), attribution, licence and source hash in the asset,
tyre radius within 1.5 cm of the modelled wheel, physics track within 1 cm of the model's. The geometric path was
also run (not committed) on the Ford GT and McLaren F1 LM files: four wheels, split meshes, checks pass.

## Survey (2026-10-14)

Triangles after dropping baked ground planes; axes = source world axes for ISO forward/up; wheelbase / track /
wheel radius from the detected wheel centres.

| file | triangles | materials | textures (largest) | mesh nodes | wheels | interior | doors | glass | axes (source) | L x W x H m | wheelbase / track / wheel r m | notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `ace_11_-_low_poly_model.glb` | 18,432 | 8 | 12 (2048x2048, 1024x1024, 512x512) | 28 | node names | yes | no | named | fwd +x, up +y | 3.84 x 1.83 x 1.45 | 2.40 / 1.41 / 0.30 |  |
| `fairheaven_lt_80_-_low_poly_model.glb` | 19,705 | 8 | 12 (2048x2048, 1024x1024, 512x512) | 32 | node names | yes | yes | named | fwd +x, up +y | 5.00 x 1.83 x 1.35 | 2.65 / 1.46 / 0.30 |  |
| `ford_gt_2006_-_low_poly.glb` | 12,829 | 5 | 19 (2048x2048) | 5 | geometric clustering | no | no | named | fwd +z, up +y | 4.73 x 2.08 x 1.17 | 2.85 / 1.58 / 0.38 |  |
| `ford_model_t_roadster.glb` | 29,459 | 13 | 0 | 35 | geometric clustering | no | yes | named | fwd +z, up +y | 3.43 x 1.71 x 1.97 | 2.57 / 1.49 / 0.41 |  |
| `free_1975_porsche_911_930_turbo.glb` | 241,260 | 14 | 27 (4096x4096, 2048x2048, 1024x1024) | 75 | geometric clustering | no | no | 6 blend material(s) | fwd +z, up +y | 6.23 x 3.12 x 1.87 | 2.58 / 1.74 / 0.36 |  |
| `free__formula_one_lp-830_sdc.glb` | 31,176 | 13 | 3 (4096x4096, 2048x2048, 256x256) | 19 | geometric clustering | no | no | no | fwd +z, up +y | 4.85 x 1.86 x 1.09 | 3.32 / 1.42 / 0.35 |  |
| `future_car.glb` | 130,624 | 4 | 6 (4096x4096, 2048x2048, 1024x1024) | 5 | not found (geometric: 0 tyre-like parts at the corners) | no | no | 1 blend material(s) | fwd +x, up +y | 6.40 x 3.35 x 2.03 | - | baked ground/shadow plane dropped |
| `generic_passenger_car_pack.glb` | 69,312 | 41 | 39 (2048x2048, 1024x1024) | 75 | node names | no | no | named | fwd +x, up +y | 19.27 x 19.27 x 1.76 | 8.43 / 8.99 / 2.00 |  |
| `generic_toyama_xr_sedan.glb` | 24,918 | 11 | 5 (4096x4096, 2048x2048) | 33 | node names | no | no | named | fwd +z, up +y | 4.44 x 1.95 x 1.45 | 2.52 / 1.52 / 0.33 | baked ground/shadow plane dropped |
| `honda_s800.glb` | 184,570 | 38 | 34 (1024x1024, 512x256, 512x512) | 38 | geometric clustering | yes | no | 6 blend material(s) | fwd +z, up +y | 3.35 x 1.54 x 1.20 | 2.02 / 1.18 / 0.29 |  |
| `kiri_86_-_low_poly_model.glb` | 20,314 | 8 | 13 (2048x2048, 1024x1024, 512x512) | 30 | node names | yes | yes | named | fwd +x, up +y | 4.31 x 1.76 x 1.32 | 2.46 / 1.42 / 0.29 |  |
| `lamborghini_centenario_lp-770_interior_sdc.glb` | 325,733 | 44 | 7 (4096x4096, 1024x64, 256x256) | 81 | geometric clustering | yes | yes | 3 blend material(s) | fwd +z, up +y | 4.98 x 2.27 x 1.16 | 2.69 / 1.76 / 0.36 | skins 0, anims 1 |
| `lamborghini_murcielago__lp-640_-_low_poly.glb` | 11,044 | 9 | 22 (2048x2048, 256x256) | 18 | node names | no | no | named | fwd +z, up +y | 17.24 x 8.24 x 4.26 | 9.99 / 6.43 / 1.28 |  |
| `lamborghini_revuelto.glb` | 547,716 | 51 | 30 (2048x2048, 1024x1024, 512x512) | 560 | node names | yes | yes | named | fwd +z, up +y | 4.95 x 2.27 x 1.13 | 2.62 / 1.57 / 0.39 |  |
| `lct_3000_95_-_low_poly_model.glb` | 18,939 | 8 | 13 (2048x2048, 1024x1024, 512x512) | 27 | node names | yes | yes | named | fwd +x, up +y | 6.02 x 2.61 x 2.85 | 3.35 / 1.82 / 0.36 |  |
| `lowpoly_generic_suv.glb` | 8,022 | 5 | 5 (4096x4096, 2048x2048) | 9 | node names | yes | no | named | fwd +z, up +y | 4.75 x 2.09 x 1.65 | 2.86 / 1.55 / 0.38 | baked ground/shadow plane dropped |
| `mclaren_f1lm_-_low_poly.glb` | 12,758 | 6 | 23 (2048x2048) | 6 | geometric clustering | no | no | 1 blend material(s) | fwd +z, up +y | 4.74 x 2.20 x 1.26 | 3.01 / 1.67 / 0.37 |  |
| `mercedes_benz_sl-65_amg_-_low_poly.glb` | 16,302 | 17 | 16 (4096x4096, 1024x1024) | 17 | geometric clustering | no | no | 2 blend material(s) | fwd +z, up +y | 5.09 x 2.29 x 1.42 | 2.81 / 1.85 / 0.39 | skins 0, anims 1 |
| `muscle_california_speed.glb` | 130,672 | 7 | 24 (4096x4096, 2048x2048, 1024x1024) | 280 | not found (geometric: 0 tyre-like parts at the corners) | yes | yes | named | fwd +z, up +y | 4.65 x 1.97 x 4.09 | - | skins 1, anims 4 |
| `pagani_zonda_cinque.glb` | 15,994 | 5 | 18 (4096x4096, 2048x2048) | 5 | geometric clustering | no | no | 2 blend material(s) | fwd +z, up +y | 4.62 x 2.44 x 1.26 | 2.87 / 1.88 / 0.38 |  |
| `phoenix_455_71_-_low_poly_model.glb` | 25,943 | 8 | 13 (2048x2048, 1024x1024, 512x512) | 30 | node names | yes | yes | named | fwd +x, up +y | 4.89 x 2.09 x 1.33 | 2.74 / 1.71 / 0.32 |  |
| `shvan_92_-_low_poly_model.glb` | 23,045 | 8 | 13 (2048x2048, 1024x1024, 512x512) | 24 | node names | yes | no | named | fwd +x, up +y | 4.90 x 2.35 x 1.93 | 3.13 / 1.66 / 0.35 |  |

Findings:
- Mostly game-ready low-poly cars (8k-30k triangles) with 2k-4k textures. The Zhabotinsky family (Ace, Fairheaven,
  Kiri, LCT, Phoenix, Shvan) and both mk2design models have one node per wheel and per caliper, named by corner.
- `future_car.glb` and `muscle_california_speed.glb`: wheels not found. Future Car is five merged meshes
  (130k triangles) with no separable round wheel component; Muscle is a skinned, animated rig (bbox 4.09 m tall: a
  second object stands above the car). Both need manual work.
- `generic_passenger_car_pack.glb` holds several cars side by side (19 m bbox); it would have to be split first.
- Scale problems: `lamborghini_murcielago...glb` is about 3.7x too large (17 m long); the Porsche 911 bbox
  (6.2 x 3.1 m) includes extra objects.
- Interior: Ace, Fairheaven, Kiri, LCT, Phoenix, Shvan, Honda, Centenario, Revuelto, SUV, Muscle. Doors as separate
  nodes: Fairheaven, Kiri, LCT, Phoenix, Model T, Centenario, Revuelto, Muscle. None has a cockpit rig, lights
  metadata or engine bay; the imported rigs therefore have no sockets and no `lights` block.

## Playable cars

| Catalog id | Model | Base physics | Wheelbase / track (m) | Tyre | Notes |
|---|---|---|---|---|---|
| `car_toyama_xr` | `imported/toyama_xr` (Generic Toyama XR Sedan, mk2design) | `car_sedan.json` (3.0 I6, FWD) | 2.524 / 1.518 | road 225/50 R17 | no interior |
| `car_generic_suv` | `imported/generic_suv` (Lowpoly Generic SUV, mk2design) | `car_sedan_awd.json` (2.0 I4, AWD) | 2.860 / 1.555 | road 235/65 R18 | spawn 7 cm higher |
| `car_phoenix_455` | `imported/phoenix_455` (Phoenix 455 '71, Daniel Zhabotinsky) | `car_gt70.json` (5.0 V8, RWD LSD) | 2.741 / 1.712 | belted 215/225/60 R15 | interior, opening parts not rigged |
| `car_ace_11` | `imported/ace_11` (Ace '11, Daniel Zhabotinsky) | `car_city.json` (Pico 1.0 I3, FWD) | 2.405 / 1.410 | economy 185/60 R15 | interior |
| `car_fairheaven_lt80` | `imported/fairheaven_lt80` (Fairheaven LT '80, Daniel Zhabotinsky) | `car_sedan_rwd.json` (2.0 I4, RWD) | 2.652 / 1.464 | belted 205/60 R15 | interior |
| `car_kiri_86` | `imported/kiri_86` (Kiri '86, Daniel Zhabotinsky) | `car_sedan_rwd.json` (2.0 I4, RWD) | 2.459 / 1.422 | touring 185/60 R14 | interior |
| `car_lct_3000` | `imported/lct_3000` (LCT 3000 '95, Daniel Zhabotinsky) | `car_sedan_awd.json` (2.0 I4, AWD) | 3.345 / 1.888 | road 225/70 R16 | interior |
| `car_shvan_92` | `imported/shvan_92` (Shvan '92, Daniel Zhabotinsky) | `car_sedan_rwd.json` (2.0 I4, RWD) | 3.135 / 1.658 | road 215/70 R16 | interior |

Their vehicle files are `data/vehicles/car_<id>.json` (written by the importer). Chassis mass and box are the base
car's. Since 2026-10-15 cars.json also names, per car, the paint/rim materials (renamed `paint`/`rim`, or
`paint_tinted` with an atlas texture), the lamp lenses (`lights`: node/material regex plus an optional ISO box per kind;
copied onto `light_<kind>` materials that the rig's `lights` block names, emission = the base texture) and the driver
eye (`at` or the steering wheel node plus an offset -> `socket_driver_eye`). Fairheaven has no indicator lenses; the
Generic SUV has head and brake lenses only. **The GLBs are Git LFS and were not pushed from the cloud session: run
`python3 tools/carimport/carimport.py` on a machine with LFS to regenerate all eight** (until then the old GLBs show
no lamp emission, and the paint colour only applies where the material was already named paint). The five new
cars' catalog entries wait in `tools/carimport/catalog_pending.json` (the catalog refuses a missing model file);
insert them into `data/vehicles/catalog.json` after regenerating.

## Owner decision needed: brand models

These carry real manufacturers' names and designs. The CC BY licence covers the creator's copyright in the mesh,
not trademarks or design rights, so they are **not** made playable:

- `ford_gt_2006_-_low_poly.glb` (Ford GT), `ford_model_t_roadster.glb` (Ford Model T)
- `free_1975_porsche_911_930_turbo.glb` (Porsche 911)
- `lamborghini_centenario_lp-770_interior_sdc.glb`, `lamborghini_murcielago__lp-640_-_low_poly.glb`,
  `lamborghini_revuelto.glb` (Lamborghini)
- `free__formula_one_lp-830_sdc.glb` (a Formula One car with a Lamborghini-style "LP" name; brand status unclear)
- `pagani_zonda_cinque.glb` (Pagani)
- `mclaren_f1lm_-_low_poly.glb` (McLaren)
- `mercedes_benz_sl-65_amg_-_low_poly.glb` (Mercedes-Benz)
- `honda_s800.glb` (Honda)

Options: leave them out, or use them only after a trademark review (renamed, badges removed).

Also for the owner: the "fictional" models carry made-up names but are recognisable real designs. The Toyama XR is
a Subaru Impreza WRX STI (bug-eye front), the Generic SUV a BMW X4 (kidney grille), the Phoenix 455 a Pontiac
Firebird Trans Am; the Zhabotinsky family follows the same pattern. They are playable as asked; whether the
recognisable shapes and grilles are acceptable is the owner's call.
