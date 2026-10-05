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

In the real world, traffic is enabled by default at 120 vehicles per lane-km,
with a 1200 m population radius, a 100 m horizontal player exclusion distance,
and a default limit of 2048 active vehicles (adjustable to 4096). Flat mode has no OSM destinations and
therefore does not invent traffic trips. F7 opens live density, radius, minimum
spawn-distance, grip and maximum-population controls, saved in user://traffic.cfg. Zero density
retires vehicles once they leave view. Driving stays active while the traffic
panel is open; it captures the mouse and disables camera look, without applying
the seat panel's brake/input suppression.

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


## High population support (2026-10-04)

The former 24-vehicle adapter cap is removed. Native and F7 configuration support
0-4096 actors, density up to 1000/lane-km and radius up to 3000 m. Default maximum
is 2048, density 120/lane-km and radius 1200 m. Existing saved settings receive a
one-time migration to at least those density/radius defaults and the 2048 limit;
subsequent user changes persist normally. F7 shows active, target, cap and queued
counts. A numeric target is not a promise that occupied or missing roads can fit
that population: road occupancy, public access and reachable destinations still
apply. No actor is forced into obstructed space to meet the count.

Planning returns at most 512 candidate trips per batch, with bounded attempts
and spatial buckets for candidate occupancy. The physics thread checks at most
eight candidates and creates at most four native bodies per tick. A staged queue
feeds later ticks rather than creating thousands at once; settings/reset cancel
obsolete candidates. Full populations do not regenerate unused trip batches.
Controller neighbor samples and staggered obstacle probes operate at 20 Hz using
32 m spatial buckets, with cached following/obstacle caps. Pose integration and
legal/corner caps remain on physics ticks. Retirement uses constant-time actor
removal. Only nearby wake sources within 200 m of the player are submitted;
more distant finite wakes cannot reach the player with the authored coefficients.

Cars and trucks render as shared MultiMesh body/window/wheel/lamp batches rather
than thousands of individual scene nodes. Buffers grow by powers of two and
visibility feedback is sent at 10 Hz. Native collision bodies remain present for
all active NPCs; this is not yet a distant traffic physics LOD system.

Focused checks: 47 assertions in 6 cases, including high-capacity sanitization,
existing flip/cancellation and real drafting. A headless render construction
check instantiates 2048 cars and 256 trucks in 22 mesh batches. This validates
render construction, not full-game FPS or 240 Hz physics at those populations;
the owner's driving/performance acceptance remains outstanding.

### Native render uploads (2026-10-04)

The renderer retains the existing six car parts and sixteen truck parts.
Population-sized pose/color conversion and buffer packing now run in C++,
with one complete MultiMesh upload per nonempty part instead of per-instance
GDScript setters. Camera-frustum sphere checks exclude off-screen geometry
before packing, including conservative whole-vehicle margins. Visibility
feedback remains at 10 Hz; collision, routing, population and nearby wakes
are unchanged. The main thread still submits these buffers; this change removes
script/binding overhead rather than relocating the renderer to another thread.
Every five seconds RG_TRAFFIC_RENDER records active/rendered counts and the
maximum native packing/upload submission time. This excludes GPU execution and
other main-thread work; full-drive FPS remains an owner acceptance check.

## Stuck NPCs: detector, causes, fixes (2026-10-05)

Owner report: "NPCs are getting stuck a lot".

**Detector** (`core/include/rg/traffic_stuck.h`, `core/src/traffic_stuck.cpp`, engine-neutral, driven by `Session::update_traffic`):
an actor is *stuck* when it is not arriving and its speed stays below 0.5 m/s for more than 5 s. The cause recorded is
the cap that binds its target speed (follow rule, obstacle probe, route/legal/corner cap) and, for a follow/probe cause,
the blocking actor and its relation (oncoming / crossing / same direction). A wait chain is walked to its root so a
deadlock cycle (the root) is told from a queue member behind a moving or stopped head. `RG_TRAFFIC_STUCK` lines are
logged per event and `RG_TRAFFIC_STUCK_SUMMARY` carries the counters (`events`, `active`, `recovered`, `chain_*`,
`root_*`, `rootrel_*`, `rel_*`, `overrides`). Seams: `Session::traffic_stuck_stats()`, `traffic_actor_count()`,
`traffic_scan_in_flight()`, `add_test_traffic_actor(pose, speed, route_length_m, stop_at_end)`. Tests
`tests/unit/test_traffic_stuck.cpp`.

**Harness** `tools/traffic_probe` (deterministic, real home world, default population 2048 cap, 60 s of simulated
time, waits for the route scans): `out/traffic_stuck/before*.out.txt` vs `after_final.out.txt`.

| (same seed, harness) | before | after |
|---|---|---|
| stuck events in 60 s | 1700 | 783 |
| still stuck at t = 60 | 1700 | 417 |
| recovered | 0 | 366 |
| events / actor-hour | 75.9 | 35.1 |
| wait chains ending in a cycle (deadlock) | 1651 | 6 |
| distinct deadlock roots | 166 | 2 |
| root relation oncoming / crossing / same dir | 153 / 7 / 6 | 0 / 0 / 2 |

**Root causes found, fixed at the root:**
1. *Two-way single-lane roads put both directions on the centre line* (`npc_truck.cpp` `truck_lane`: the offset for one lane was
   0 for both directions): head-on mutual yield (153 of the 166 deadlock roots were oncoming pairs). A two-way road with one lane
   in total now puts each direction in the centre of its own half (`-width*.25*dir`); a one-way road stays centred (test in `test_npc_truck.cpp`).
2. *The follow rule and the obstacle probe obeyed oncoming and crossing NPCs.* An oncoming NPC is now ignored by the
   follow rule (the lateral separation of the lanes is the avoidance), the probe no longer hits NPC bodies (NPCs are
   handled by the neighbour rule only).
3. *Side-by-side overlap:* two NPCs whose lateral distance is under 2.7 m and gap a few cm were each "ahead" of the other.
   Deterministic tie-break by id (the lower id yields).
4. *Crossing / mixed wait cycles:* the smallest deterministic right-of-way rule: for a crossing neighbour the lower id has
   priority; a car waiting longer than `kCrossingPatienceS` (8 s) ignores a crossing blocker (`overrides` counts these,
   3569 in the 60 s run; each is a cycle broken). Not a real right-of-way.
5. *Arrival despawn margin 0.5 m* was smaller than the distance from the route end to the speed-0 point (up to one route
   point spacing): the actor parked just short of the end for ever. The margin is now the last route spacing + 0.1 m,
   clamped to [0.5, 4] m.
6. *Corner speed pass* (`npc_traffic.cpp`) divided by a 0.1 m floor at junction joins (curvature spike capping the speed
   to a crawl): windowed 4 m pass.

**What is left (honest):** the remaining events are queues behind a moving head at default density (`chain_moving` 744 of
783; a car behind a slow or briefly stopped leader exceeds 5 s at < 0.5 m/s when a head stops for an arrival or a
junction), not deadlocks; `root_cycle` 2 (a 3-cycle and a 4+-cycle at junctions with crossing flows) remain. The backstop
despawn is **not** used (`backstop=0`). A full junction right-of-way needs: per-junction reservation or priority from the
road data (priority roads, give way/stop signs, signals, right-hand rule fallback), turning-conflict zones instead of the
point-neighbour test, and a yield state that survives the 20 Hz decisions. Not attempted here.
