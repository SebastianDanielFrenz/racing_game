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
