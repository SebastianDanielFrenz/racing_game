# OSM buildings and skyline

The real-world view reads building ways and multipolygon relations from the local,
read-only `g2m.src.osm` dataset. Complete footprints are projected into the session
UTM zone. A footprint has one owner tile even when source halos overlap. Concave
outlines and inner courtyards are retained. Missing relation members are skipped;
missing DEM support does not invent a building elevation.

Height preference: `height` (metres or feet), otherwise `building:levels` times
`storey_height_m`, plus recorded roof height/levels; otherwise `fallback_height_m`.
`min_height` / `building:min_level` leave elevated parts open underneath. Walls
extend to the lowest sampled ground, with a flat base/top reference at the highest
sampled footprint ground. Recorded `building:colour` hexadecimal colours are used;
otherwise deterministic muted colours. Roofs are currently flat caps, not inferred
roof architecture. These buildings are visual geometry; vehicle collision is not
implemented for buildings yet.

`data/world/world_config.json` has a `buildings` section (reload world/restart):

| Option | Default | Meaning |
| --- | ---: | --- |
| enabled | true | Enable building rendering |
| ordinary_distance_m | 2200 | Nearby ordinary-building batches |
| skyline_min_height_m | 50 | Minimum total height retained in the skyline |
| skyline_distance_m | 40000 | Skyline visibility/streaming radius |
| fallback_height_m | 8 | Estimated height when OSM has no usable height/levels |
| storey_height_m | 3 | Estimated floor height for level counts |

Distances are bounded to 256–10,000 m for ordinary buildings and up to 80,000 m
for the skyline (at least the ordinary distance). Floor estimates are bounded to
1–10 m and fallback heights to 1–100 m to keep accidental settings manageable.

Only the imported dataset can supply landmarks; increasing the range does not
fetch extra coverage. Camera far planes extend beyond the skyline range. Terrain
and other buildings can occlude towers naturally. Heights without tags cannot be
identified as skyscrapers from this dataset alone.

OSM decoding and mesh tessellation run on a dedicated worker. Nearby tiles are
prioritised; four completed batches at most wait for upload, and one tile is
uploaded per frame. Draw calls are grouped by 1 km owner tile into ordinary and
tall batches, rather than one node per building. Distant ordinary geometry is
evicted; skyline meshes retain the actual footprint and height. No expensive
textures or shadows are generated. Shutdown joins this worker before releasing
the shared world terrain. Driving does not wait for the skyline scan.

## Later appearance strategy

Placement and appearance are separate. The native building record retains OSM
identity (way/relation), footprint rings, measured/estimated height provenance,
minimum height and colour. The current presentation is extrusion. Later options
can coexist: choose premade assets procedurally from these records and other
sources, or reconstruct individual facades/roofs from licensed satellite and
street imagery. Reconstructed landmarks can override procedural assets while
unmatched buildings retain the extrusion fallback. Preserve coordinate alignment,
source attribution and height provenance through all replacements. More detailed
assets can introduce shape LODs or ordinary-building clusters; keep landmark
silhouettes at long range. Imagery inference is future work, not part of this build.

## Camera obstacles (cinematic view, 2026-10-05)

The cinematic director must not film from inside a building or through one. `rg::BuildingObstacles`
(`core/include/rg/building_footprints.h`, engine-neutral) turns the native `rg::Building` records into footprint prisms
in session coordinates (one prism per outer ring; the vertical span is `[bottom, base + height]` in absolute metres, the
same z the session uses; inner courtyards are ignored - a camera in one is enclosed anyway) on a 32 m grid, loaded per
1 km UTM tile (`WorldTerrain::buildings_tile`, min_height 0, fallback 8 m, storey 3 m, the values the ordinary
`building_stream.gd` tiles use) by a worker thread for the 3 x 3 tiles around the car; far tiles are evicted.
`ready(x, y)` is false until the tile has arrived and the director then frames from behind the car (`chase_fallback`)
instead of trusting missing data. The director rejects a candidate camera that lies inside or within 1 m of a footprint
whose span covers the camera height, or whose sight line to the car and to the road a third and two thirds of the way
(buildings and sampled terrain, 0.3 m clearance) is blocked; it tries the other road side, then other lead distances
(x0.7, 1.35, 0.5, 1.7), then the chase-style fallback, retrying a roadside shot every second. Per-frame work is bounded
and allocation-free (`RgSimulation::sync_cinematic_obstacles`, `update_cinematic`). Counters: `CinematicDirector::stats()`.
Tests: `tests/unit/test_cinematic_obstacles.cpp`. The flat world has no buildings and no obstacles.
