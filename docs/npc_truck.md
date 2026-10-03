# Drafting truck prototype

Press **T** in a running world to place or replace one truck about 45 m ahead
along the nearest usable road in your driving direction. **Shift+T** removes it.
Resetting or relocating the player removes the truck; a world switch also clears it.
The truck starts moving at the permitted local speed, targeting 70 km/h; native
`request_npc_truck(enabled, speed_kph)` accepts targets from 10 to 90 km/h.
Route preparation runs on a background thread. Placement uses your current
position when that route arrives; stale/off-route requests report a retry message.

The procedural cab, cargo body, wheels and lights follow a frame-latched native
pose with the same floating-origin conversion as the player. The truck has a
13 x 2.5 x 3.6 m moving box collider, with wheels rendered separately. This first
NPC uses a kinematic traffic controller, not a simulated engine/tyre drivetrain
or an articulated tractor/trailer. It collides with the player but cannot be
pushed aside. Its lane/profile controller supplies motion and nearby terrain
heights; it does not integrate truck suspension.

## Lanes and route

Uses accepted smooth OSM road profiles, actual OSM shared-node connectivity,
width, lanes, one-way and direction-specific tags. The truck keeps the rightmost
usable lane; valid turn-lane tags move it out of turn-only lanes when a through
lane is available. A missing/implausible lane count falls back conservatively
to width. Roads narrower than 2.8 m are rejected; unsupported profiles are not
invented. Single narrow lanes use the center. Bridge/tunnel deck heights come
from the game's same reconstructed geometry.

The controller continues through sufficiently straight connected profiles,
preferring the same way. It does not connect roads just because they cross in
map coordinates. It brakes for bends, usable speed limits and route ends, and
for the player directly ahead. Missing or unsafe continuations cause a stop.
The route is bounded to 2.5 km/32 stretches. The truck stops pulling away beyond
about 200 m separation to stay near the existing player terrain/collision area.
This is one drafting target, not a general traffic system with signals,
right-of-way, lane changing, overtaking or intersection turn restrictions.

## Real aerodynamic wake

Each physics tick publishes one `ps::aero::WakeSource` using the truck's actual
world pose and velocity. The native physics engine samples that source for the
hypercar during its aero substeps; no artificial acceleration or grip bonus is
applied. The initial authored truck wake has length 60 m, radius 2 m, expansion
0.15, deficit 0.45 and turbulence 0.20, following the library's truck example.
These coefficients are uncalibrated. Wake response rotates with motion, handles
wind and disappears when the truck is removed. The HUD shows truck status/speed
and body dynamic pressure relative to undisturbed airflow. Middle-click owner
bookmarks include truck status as well as existing aero readings.

Focused checks cover lane policy, actual moving collision/snapshot publication,
native wake reduction and removal. Local real-data route checks cover the spawn
and the owner B8 point at 50.141900, 8.479882. Driving acceptance is with the owner.

## Imagery refinement

Street-level imagery can later refine this same shared lane model offline.
Mapillary supplies detected map features including lane markings and signs:
[map features](https://help.mapillary.com/hc/en-us/articles/115002332165-Map-features).
Imagery has [CC-BY-SA conditions](https://help.mapillary.com/hc/en-us/articles/115001770409-CC-BY-SA-license-for-open-data).
No imagery is imported by this feature. Proposed corrections should carry source,
date and confidence, and retain OSM/width fallback where observations are weak.
