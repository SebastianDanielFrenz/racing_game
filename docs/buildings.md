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
