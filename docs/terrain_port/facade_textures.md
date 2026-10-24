# Building facade texture assets and performance

The near building pass uses shared photographed window atlases and real material
maps, not flat colour alone. Residential window frames/glazing and office glazing
come from selected original images in Scouser's CC0 urban texture collection;
industrial window frames, glass dirt and wear use Poly Haven's Modular Factory
Facade maps. Wood doors, plaster, masonry, timber, cladding and sills use shared
material textures. Source authors and licences are in `data/credits.json` and
`data/textures/buildings/assets.json`.

The source photographs contain baked lighting/reflection. They are appearance
fallbacks, not reconstructions of the mapped building or simulated interiors.
There is no transparent window geometry, per-window light or parallax room shader.

## Reproduce the assets

Run `python tools/fetch_building_assets.py` from a fresh checkout; use `--verify`
for a network-free hash check. Direct image files are pinned by URL, byte length
and SHA256. Two selected facade photographs also pin their source ZIP and member
hashes. The ZIP is downloaded to a temporary folder, only listed members are
installed, and the temporary source is deleted. The 205 MB source ZIP is not a
runtime asset or part of the release. Selected installed textures are copied
into local snapshots by the existing release tool. Source images stay outside
Git under the existing asset-fetch policy.

## Rendering and LOD constraints

- Material IDs 0..11 remain stable. Door 12, OfficeWindow 13 and IndustrialWindow 14
  append shared batches. Atlas regions are configured in `materials.json`;
  native window/door quads have upright unit UVs. Wall/roof UVs remain metric.
- Window quads keep six vertices each. Door inserts use six vertices instead of
  36. There is no object or unique material per building/window. At most three
  extra material batches can appear per near tile (door and two window types).
- Existing building ring distances, skyline thresholds and detail-free far
  geometry are unchanged. Distant meshes emit no window, door or facade frame
  details. Atlas texturing does not extend their visibility or generation range.
- Materials and images are shared across tiles. Repeated image/channel paths
  reuse one GPU texture; frame/sill concrete therefore reuses the flat-roof maps.
  Mipmaps, compressed textures and anisotropic filtering handle distant detail.
- The shared building texture set measures 39.00 MiB, versus 24.67 MiB previously,
  on the tested desktop renderer. No texture streaming or per-building image
  allocation is introduced. The material set remains below a 64 MiB review budget.

## Comparison procedure

Run the same `building_texture_benchmark.gd` script against the baseline snapshot
and candidate project, passing a separate absolute `RG_TEXTURE_BENCH_OUT` directory
to each. Use a real GPU renderer, `--rendering-method gl_compatibility`, and
`--resolution 1280x900` (pass option and value separately). The script uses the same
Unterliederbach source tile and close camera, warms rendering for one second,
samples three seconds, and records counts/memory/frame intervals. Five warm
native geometry calls provide a median independent of first-use data derivation.
Run comparisons sequentially after builds/tests have stopped. Reuse both output
folders for warm-cache comparison; first-use derived-cache time is not a texture
rendering cost. JSON results accompany this note.

This is a small static-camera comparison on one Radeon RX 7900 XT. It is not a
full-scene GPU timer, a streaming-drive benchmark or a hardware VR/ARM guarantee.
Use the real-world drive smoke as an additional lifecycle check.

[Reviewed appearance gallery](textured_facades.png).

Observed local comparison (both derived caches warm):

| Metric | Before | Textured |
|---|---:|---:|
| Tile vertices | 327168 | 324288 |
| Close-camera draw calls | 7 | 9 |
| Warm generation median, ms | 27.692 | 27.507 |
| Frame interval median, ms | 0.146 | 0.15 |
| Frame interval p95, ms | 0.217 | 0.22 |
