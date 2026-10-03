# Future NPC traffic system

Owner requirements recorded 2026-10-03. Planned work; the current single drafting
truck prototype does not implement this complete system.

## Population and spawning

- Each player creates a configurable traffic population radius around them.
  Spawn candidates lie on eligible roads within that radius.
- Never spawn within a configurable minimum distance of any player.
- Spawn in the rightmost lane for the selected direction of travel only when
  the vehicle footprint and safe clearance are unobstructed. If blocked, defer
  spawning rather than place the NPC in another lane or overlap a vehicle.
- Resolve lane direction and one-way restrictions from OSM. Use conservative
  width/profile fallback when lane data is unreliable; reject unusable lanes.
- NPC density is adjustable during play. Density changes reconcile population
  gradually; increasing density must still obey exclusion and occupancy checks.
  Radius, minimum spawn distance and population limits need configurable values.

## Speed policy

- NPCs never exceed the applicable road/lane speed limit. Resolve directional,
  lane-specific and vehicle-specific limits where available.
- NPCs attempt to drive at the applicable limit when conditions permit, subject
  to vehicle capability, traffic, stopping distance and corner grip.
- Where there is no speed limit, trucks target at most 80 km/h. Cars draw a
  persistent maximum target speed on spawn between 80 and 250 km/h. Vehicle type
  may influence that distribution without completely determining the result.
- The 80 km/h truck fallback and random car target apply to roads without a
  limit; they do not authorize exceeding an explicit limit.
- A nonnumeric or conditional OSM limit must not silently be interpreted as
  unlimited. Resolve its meaning/conditions or use an explicit conservative
  fallback policy. An explicit unlimited designation is distinct from missing
  data.

## Cornering and grip

- Adapt target speed to the vehicle's available lateral grip and upcoming
  curvature. Tight bends, lower-grip surfaces and rain, snow or ice reduce it.
- Use the same surface/weather grip information as physics. Account for vehicle
  differences and reserve grip for braking; do not assume one fixed lateral
  acceleration for all NPCs.
- Look ahead and brake before the bend or lower speed limit, rather than wait
  until entering it. The minimum of legal, corner, traffic and capability caps
  determines the current target speed.

## Eligible public roads

Only these OSM highway classes are candidates:

`motorway`, `trunk`, `primary`, `secondary`, `tertiary`, `unclassified`,
`residential`, `motorway_link`, `trunk_link`, `primary_link`, `secondary_link`,
`tertiary_link`, `living_street`, `road`.

The class whitelist is necessary but not sufficient: honor applicable public
motor-vehicle access restrictions, including direction and vehicle type.
Private/nonpublic roads cannot spawn NPCs even when their highway class is in
this list. Missing access tags on an otherwise eligible class are not by
 themselves a prohibition; explicit restrictions and resolved OSM defaults apply.

## Implementation sequence

1. Shared road eligibility, access, lane occupancy and speed-limit resolution.
2. Player-centered population manager with spawn exclusion and live density.
3. Per-NPC persistent desired speeds, route following and look-ahead grip caps.
4. Traffic spacing, collision avoidance and junction behavior; preserve native
   aerodynamic wakes for drafting behind all eligible vehicles.

Numeric population defaults, speed distribution and public-access resolution
policy remain implementation choices. Record their selected values alongside
configuration when implemented. General traffic does not yet exist in the game.
## First traffic implementation (2026-10-04)

In the real world, traffic is enabled by default at 4 vehicles per lane-km,
with a 600 m population radius, a 100 m horizontal player exclusion distance,
and a hard cap of 24 active vehicles. Flat mode has no OSM destinations and
therefore does not invent traffic trips. F7 opens live density, radius, minimum
spawn-distance and grip controls, saved in user://traffic.cfg. Zero density
retires vehicles once they leave view; the panel holds the player's brake.

Destination selection is random among locally reachable buildings whose
representative point lies inside tagged residential landuse, or public
amenity=parking nodes/areas. Source OSM tags are read directly, independent of
building rendering. Complete closed multipolygon outer members are supported;
fragmented members are deferred. Residential inner rings are excluded.
No arbitrary road endpoint is substituted when destination data is absent.

A background planner scans the nearby source/derived tiles, builds directed
accepted-profile edges, and finds connected-node paths. It honors the highway
whitelist, public motor-vehicle access, one-way direction and node-via turn
restrictions. Unresolved conditional access is excluded; unsupported via-way
restrictions conservatively prevent changing from the affected way. There is
no connection merely because coordinates cross. Rightmost usable lanes share
the drafting truck's width/one-way/through-lane fallback. Planned starts and
live native footprint-height sweeps reject occupied spawns.

The destination attaches within 40 m to an ordinary public road, excluding
motorway/trunk/link and tagged bridge/tunnel stretches. On arrival the NPC
brakes to a stop at that road point and despawns. It does not drive through a
building or turn into a parking bay yet. Cars receive a persistent random
80-250 km/h fallback target; trucks use 80 km/h where no numeric limit exists.
Numeric speed limits are targets subject to authored capability caps (250 km/h
car, 120 km/h truck), cornering and traffic. Lane limits use their conservative
minimum until per-lane resolution is complete. Unresolved limits use a cautious
fallback rather than unlimited speed.

Moving cars/trucks have native kinematic collision bodies and real aerodynamic
wake sources, combined with the manually placed T truck. Their engines,
suspension and tyres are not individually simulated. Generic car/truck meshes
follow the same frame snapshot and floating-origin conversion as the player.
They slow for the player, other NPCs and ray-detected obstructions. This remains
a first traffic controller, not finished junction right-of-way/signal AI.

Vehicles outside radius + 50 m retire after three seconds outside the camera
frustum; density reduction uses the same visibility guard. Arrival despawns
immediately. IDs are monotonically assigned during a session; retired vehicles
are deleted and never restored. Replacement traffic consists of new random
trips. World changes/reset clear the population. This is a single-player
population manager; camera-frustum checks conservatively retain occluded actors
rather than treating renderer occlusion as authoritative.

Corner targets use a conservative authored lateral-acceleration budget,
curvature and an initial gravel/unpaved penalty, with upstream braking. The live
grip factor further scales speed targets; it is an explicit provisional weather
control, not automatic rain/snow/ice coupling or a full physical tyre-grip model.
That shared weather/surface integration, stronger intersection conflict handling,
lane-specific turn selection, parking maneuvers and car-type speed distributions
remain the next stages of the owner specification.

Focused policy/destination tests and the existing drafting-truck integration
checks pass. A hidden real-cache route check validates reachable destinations,
arrival stopping points and spawn exclusion. Driving acceptance remains with
the owner.

## Flip/reset and shutdown correction (2026-10-04)

F preserves nearby actors and freezes their motion during relocation priming,
then resumes their existing trips. R/explicit relocation still clears traffic.
The runtime planner now consumes only already-published road geometry, using
a nonblocking cache lookup; busy/missing profiles defer population work. It
does not derive distant geometry during NPC planning. A spatial road-sample
index replaces repeated all-roads searches for each destination. Cancellation
is checked during indexing, destination search, graph traversal and route sampling.
Busy caches do not reset the existing population target to zero. Scan duration,
trip/destination counts and cancellation are logged as RG_TRAFFIC_SCAN; RG_DRIVE
now includes population count and loading state. The owner hang log was preserved
at out/traffic_hang_owner.log. Focused flip-preservation and cancellation checks pass.
