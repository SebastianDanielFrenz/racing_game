# Road surfaces, bridges and terrain smoothing

The S: game now uses road profiles for its one-metre driving terrain. The
same cached heights feed physics and the visible nearby terrain. Bridges have
separate mesh collision and rendering, rather than raising the ground beneath
them. Their slabs have a 0.5 m thickness. Tunnel floors remain on the lower
road; an OSM tunnel crossing a higher-layer highway supplies an inferred roof
even when the highway has no `bridge` tag.

## Configuring terrain smoothing

Edit `data/world/world_config.json`:

```json
"terrain_smoothing": {
  "enabled": true,
  "radius_m": 2.0,
  "strength": 0.35,
  "passes": 1
}
```

| Parameter | Meaning | Valid range |
|---|---|---|
| `enabled` | Apply terrain smoothing before road surfacing | Boolean |
| `radius_m` | Gaussian neighbourhood radius in metres | 0–8 |
| `strength` | Blend from original terrain to the filtered terrain | 0–1 |
| `passes` | Number of Gaussian passes | Integer 1–4 |

The modest defaults reduce small bumps while preserving slopes. A zero
strength or radius disables the filtering. The radius rounds down to whole
cells at each terrain LOD; coarser cells larger than the radius are unchanged.
Larger radii or more passes increase loading work. Restart the game or reload
the real world with F8 twice after changing these settings.

Filtering uses a halo large enough for every pass, so tile boundaries do not
create artificial edges. NoData centres remain NoData. Raw imported elevation
tiles are immutable, and road/bridge fitting reads those original elevations.
Road profiles and structural decks take precedence over general smoothing.

## Reported B8 crossings

| Latitude | Longitude | Structures represented |
|---|---|---|
| 50.121806 | 8.524495 | Ramp bridges, including OSM ways 8099864 and 8099866 |
| 50.123512 | 8.515598 | Track bridge 5558250 |
| 50.126719 | 8.502332 | B8 bridges 1096866567 and 1096866569 |
| 50.130483 | 8.491895 | L3014 bridge 14799333 over B8 |
| 50.135940 | 8.486279 | Tunnel 23384258; roofs on both B8 carriageways |
| 50.141900 | 8.479882 | Tunnel 23091108; roofs on both B8 carriageways |
| 50.149487 | 8.466567 | Track bridge 32275368 |

The runtime can reconstruct some profiles explicitly rejected by the original
library fit. Ground-road fallbacks retain shared endpoint heights, use up to
12% longitudinal grade and a vertical radius of at least 500 m on major roads;
noisy endpoint gradient estimates are bounded at 10%. Bridge fallbacks follow
supported approaches, including spans split across OSM ways; their radius
policy can relax to 250 m. If estimated short-span gradients disagree, the
measured abutment rise supplies a common secant gradient. Tunnel-floor fits
allow 30% grade and 20 m radius. These are game reconstruction policies for
imperfect bare-earth data, not surveyed bridge dimensions or proof that the
source library's original class constraints passed. Unsupported/missing-data
profiles remain explicit declines. No external library source was changed.

Detailed surfacing starts only on level-0 terrain, preventing the distant
render LOD from triggering expensive complete-way profile derivation. Nearby
structural collision installs behind the simulation gate, with one mesh per
attempt. Rendering uploads two structural meshes per frame. Teleports still
prime the terrain before placing the car; those relocation pauses are distinct
from steady driving performance.

## Verification and diagnostics

The final regression suite passed all 158 tests. The two local B8 real-data
checks passed 4,277 assertions on the final code.

`tests/unit/test_road_surface.cpp` checks all listed OSM structure IDs and
native collision casts. Its local real-data comparison covers the full 9.85 km
B8 route; the measured height-gradient roughness fell by about 65%. The three
former depressions' maximum sampled grades fell from approximately 39%, 32%
and 32% to 3.4%, 6.1% and 10.9% respectively.

`game/scripts/b8_contact_smoke.gd` drives the supplied hypercar across those
three depressions. The verified run recorded zero falls or terrain misses,
239.9–240.1 Hz during driving, and at most 0.092 seconds with all four wheels
unloaded. `b8_structure_shots.gd` captures the seven reported crossings.

The live HUD/window and `RG_DRIVE ready` report the native build's source
revision and optimisation mode. The historical Godot project/user-data name
is retained. Once per second and on window close, `user://last_drive.json`
stores the current session position, UTM coordinates, source revision and
terrain/timing status. This makes a future report from inside a hole locatable.

Plan position: game-side G3-D/E surfacing and structural collision are now
integrated. Full source-profile acceptance (G3-C), general junction surfaces,
road markings and the complete G3-F/R3 route acceptance remain open.

## Engine map disk cache

`physics.engine_map_cache_enabled` defaults to `true`, including when omitted.
It enables the physics library's persistent `.psmaps` cache in
`external/physics_sim/out/godot_engine_cache` for the supplied vehicles. Set it
to `false` to regenerate maps on each fresh vehicle-definition load. The
in-memory vehicle definition is still reused on world reloads; restart the
game to measure a changed disk-cache setting. Library cache keys invalidate
entries when generation inputs or generator versions change.
`RG_ENGINE_MAP_CACHE` reports the directory and number of generated engines
on each fresh definition load (`generated=0` for cache hits or engines that
do not require generated maps). Runtime verification is left to the owner.

## Incomplete road-profile coverage

A complete-way `roads.geom` fit can require DEM tiles outside the imported
region even when the requested one-metre terrain is available. When the
deriver explicitly reports `required dependency absent`, the game retains
its valid smoothed terrain and OSM grip instead of failing that terrain tile
and freezing permanently. No deck or fitted road is fabricated for that
geometry tile. `RG_ROAD_SURFACE unavailable` records this fallback once per
geometry tile and world load; other fetch/decode errors remain failures.

## High-speed merge surface correction

The owner's 2026-10-03 drive reached 321.5 km/h before losing wheel contact
around simulation time 95 s, near UTM 32N E468766 N5552822. It recorded no
dropped ticks or terrain misses. Fine surface samples along the logged
trajectory revealed alternating changes up to 9 cm per half-metre sample
where ground profiles 3995697 and 4823280 overlapped. Nearest-profile ownership
was switching abruptly across their boundary. Ground surfacing now blends
the nearest sample from each profile across an overlap band (normalised
width 0.35), restricted to equal OSM layers and tunnel status. The largest
sampled change in the onset area fell to approximately 2.3 cm. Single-profile
surfaces retain their original grades; bridge decks remain separate.
The one-second drive log lacks wheel/attitude history, so causation of every
reported rollover and the final driving behaviour require owner confirmation.