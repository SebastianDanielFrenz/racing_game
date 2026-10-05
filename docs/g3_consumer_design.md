# G3/R3 consumer design: carve switch, ribbons, bridges, route acceptance

Status: design only, nothing built (2026-10-05). Author: Opus design session.

This document covers the racing_game side of finishing G3 (roads) and R3
(roads in game). geo2map's server side is designed in
`geo2map_engine/docs/street_fidelity/p2_junction_design.md` (P2, slices 1-12)
and `docs/street_fidelity/plan.md` (P3 markings). Nothing here changes
geo2map or physics_sim. Requests to both are collected in section 9.

Owner rulings this design builds on:

- Carving moves to the server with G3-D. At the switch the game deletes its
  own carve step, profile completion and related code. There is no fallback
  (P2 design section 10).
- There is no zero-gap gate. A work-in-progress state is acceptable.
- Relaxed profiles are served flagged "completed".
- Kerbs are visual only, 12 cm. Physics stays flush.
- Tunnels are not drivable in v1.

Notation: `file:line` anchors refer to racing_game at 9aca2d3. geo2map
anchors refer to c5ea27c plus the uncommitted D32/D35 work that was in its
tree when this was written.

---

## 1. Starting state

### 1.1 Pins

| Repo | Game pin | Head | Notes |
|---|---|---|---|
| geo2map_engine | 54d7083 | c5ea27c (+ uncommitted D32/D35) | 24 commits apart. Public headers changed additively only. Bridge/phys/server sources changed only in 2 CMakeLists lines. |
| physics_sim | 7d5316f | 7d5316f | The geo2map bridge's own `physics_sim.pin` is b90a531 at both geo2map SHAs. This mismatch already existed and is not new. |

The uncommitted geo2map work adds `discovery_stages() = 2` to
`RoadGeomDeriver` (D35: the far ends of bridges incident to a tile's nodes)
and `bridge-far-end=1` to the roads.geom config text. That changes the
release id again. **Only a committed geo2map SHA may be pinned.**

### 1.2 What geo2map serves (roads.geom encoding 7, P2 slices 1-8)

- Section 1, graph. Section 2, entries and profiles, with completed profiles
  served (retry 20 m / 16384 iterations, abutment ground path, ground ramp,
  strict anchor at bridge nodes D30).
- Section 3, cross-sections (`CrossSectionBand`: kind, direction, change,
  counted, paved, marked_right, width_mm, turn bits).
- Section 4, features (not junction-adjusted;
  `adjust_features_for_junctions` does that client-side).
- Section 5, junctions. Stored in the owner tile only. Rings are road bed,
  paved and outer.
- Section 6, furniture: stop lines, crossings and signals. Flag bit 3 is
  reserved; a consumer derives "inside junction" from the owner junction's
  trims.
- Section 7, provenance per entry: Strict, Retry, BridgeCompleted or
  GroundCompleted.
- Consumer API: `carriageway_edges`, `carve_footprint` (paved, roadbed,
  carriageway and outer edges, crown pivot/kind, kerb lines,
  `kRoadKerbHeightM = 0.12`), `driving_lanes_at`, `envelope_road_segments`,
  `road_surface_height_m` (z = p + C − C(0) + V + F), `feature_height_m`,
  `make_junction_surface`/`junction_height_m`, `build_road_furniture`,
  `furniture_quad`, `furniture_lane_groups`, `signal_post`, and
  `RoadCrossSection::boundaries_at`.

Still to come from geo2map:

- **Slice 9:** `g2m.terrain.carved`, L0-L6, cell-registered, SrcElev codec.
- **Slice 10:** `g2m.terrain.class`, node-registered 257×257 at integer
  metres, with planes land_class, surface (SurfaceKind 0-11), role and rank.
  - Carver inputs: 3×3 elev.base plus up to four roads.geom L2 tiles within
    192 m.
  - Deck clearance clamp: z = min(z, deck_top − 0.55·S(e/4)) over the bridge
    footprint (paved + 1 m).
  - Tunnels are never carved.
- **Slice 11:**
  - bake opt-in and builtin release;
  - `fill_physics_surfaces` class mode;
  - a render ClassWindow;
  - the `g2m_ps_bridge` layer option.
- **Slice 12:** census.
- **P3:** final marking descriptors (D-SF3, hybrid markings).

### 1.3 What the game does today

The game carves and smooths terrain itself and builds its own decks:

- `WorldTerrain::fetch_and_decode` (`core/src/world_terrain.cpp:354-367`)
  smooths L0 (`smooth_terrain`, :356). It then applies a `RoadSurfacePatch`
  built in `road_surface_patch()` (:369-439).
- `road_surface_patch()` fetches roads.geom. It completes declined profiles
  itself: it fetches DEM dependencies, calls `complete_road_profiles`, and
  retries at 20 m / 16384 iterations (:397-429). It then publishes decks
  (:432-437).
- `road_structures.cpp` builds decks (`deck()` :61-120). It infers tunnel
  roofs for the three B8 underpass dips that OSM does not tag as bridges
  (:241-272, vault G2M-011).
- `Session::sync_road_decks` (`core/src/session.cpp:796-826`) installs each
  deck synchronously on the stepping thread with `world_->create_body`
  (MeshShape), one per attempt, while the clock is frozen.
- Render road colour comes from the game rasterising src.osm segments into
  chunk classes (`road_classes.cpp:73-107`, `world_terrain.cpp:117-141`).
- Road visuals are `build_road_visual` (`road_visual.cpp:12-98`). It parses
  lanes, oneway and turn:lanes per tag, drapes on bilinear `sample_l0_height`,
  and is uploaded by `game/scripts/road_stream.gd` as one 1 km `ArrayMesh`
  per frame with no ms budget.
- Physics grip uses the bridge's road mode (paved/unpaved/off_road →
  asphalt/dirt/grass, `session.cpp:288-307`).

---

## 2. geo2map pin bump strategy

**Decision: bump early, to the first committed geo2map SHA that contains the
D32/D35 work. In the same commit, delete only the game's profile completion
and retry. Keep the game's carving, smoothing, decks and inferred tunnel
roofs until the carve switch (section 3).**

| Option | Pros | Cons |
|---|---|---|
| A. Pure bump, change nothing | Smallest diff | Two completion tiers stack: the game re-completes and retries profiles the server already completed. Results differ from what the server will carve later. The game's 20 m retry overwrites served `Retry` profiles with its own. |
| **B. Bump + delete game completion/retry (recommended)** | The game consumes exactly what the server serves, so its carve uses the same profiles the server will carve. That turns the later switch into a terrain-source change, not a profile change. It removes about 150 lines. The DEM-dependency fetch leaves the game process (start-up cost drops). It also removes the game's use of `road_dem_dependencies`/`RoadDemSampler`, whose headers D32 is changing. | One more pin bump before the switch. The B8 profiles may change visibly (the served completion differs from the game's). |
| C. No bump until slice 11 (big bang) | One bump | 24+ commits and four layers land in one change. Ribbon, route and bridge work could not use encoding 7 (junctions, furniture, provenance) until then. A regression would be hard to bisect. |

B wins because every later slice (ribbons, junction surfaces, furniture,
bridge decks from served profiles, the surface reference in the harness)
needs encoding 7. It also isolates the profile-source change from the
terrain-source change.

**Keep across the bump:**

- `RoadSurfacePatch` and the smoothing.
- The "required dependency absent → empty patch" fallback
  (`world_terrain.cpp:381-387`). The server can still decline at the region
  edge.
- Decks and tunnel-roof inference.

**Delete in the bump commit:**

- `world_terrain.cpp:397-429` (builder, dependencies, `complete_road_profiles`
  call, 20 m retry loop).
- `complete_road_profiles` in `road_structures.cpp:129-232` and its
  declaration in `road_structures.h`.
- The tests that exercise only it.

**Later bumps** (always a committed SHA):

- after geo2map slice 11, for the switch (S12);
- after P3 markings;
- otherwise only when a slice needs something.

Each bump checks:

1. the bridge's `physics_sim.pin` against the game's physics_sim pin; record
   any difference in the commit message;
2. the release id; a new id re-derives the server cache, so measure the first
   start;
3. `tools\ci.ps1 -Only release`, `tools\smoke_test.ps1 -Drive`, and
   `b8_contact_smoke.gd` at stations 2800, 4390 and 5200.

**Risk.** Provenance-flagged profiles must be consumed as ordinary profiles.
`RoadSurfacePatch` must not filter on `decline` alone when a completed
profile is present. A unit test pins this (S1).

---

## 3. Carve switch (G3-D consumer)

### 3.1 What the game consumes after the switch

- **Physics heights:** `g2m.terrain.carved` L0, through the bridge's layer
  option. `fill_tile` still fills 255 m node-registered tiles. Nothing is
  smoothed or patched client-side.
- **Physics surfaces:** `fill_physics_surfaces` in class mode, from
  `g2m.terrain.class` (surface plane where role marks a road, land_class
  elsewhere). The game supplies full tables:
  - SurfaceKind (12 values) → game surface name;
  - LandClass (14 values) → game surface name.

  Both tables live in world_config `physics.surface_kinds` and
  `physics.land_classes`, resolved through the game-owned
  `data/surfaces/surfaces.json`. A missing entry is a load error, never a
  silent default.
- **Render heights:** the same carved L0. Render chunks keep the Jolt
  diagonal (`terrain_chunk.h`), so the terrain mesh equals the physics
  surface.
- **Render colour:** the ClassWindow from slice 11 (surface and land_class
  planes) replaces the game's src.osm rasterisation.
- **roads.geom:** still fetched, now without a patch, for:
  - ribbons, junctions and furniture (section 4);
  - bridge decks (section 5);
  - NPC traffic (`road_geometry_at`, used by `npc_traffic.cpp:90` and
    `npc_truck.cpp:54`);
  - the harness's surface reference (section 6).
- **src.osm RoadSegments:** kept for metadata only (speed limit and
  road_ahead, `session.cpp:1120-1150`). See decision 3.3.

### 3.2 Delete map (P2 section 10 list, with anchors)

Deleted in S12. No fallback path survives.

| What | Where |
|---|---|
| Smoothing call | `core/src/world_terrain.cpp:356` |
| Patch carve in fetch | `world_terrain.cpp:357-365` |
| `road_surface_patch()` | `world_terrain.cpp:369-439`. Replaced by a plain roads.geom fetch+cache (`road_geometry_shared(key)`), keeping the "outside coverage" and "dependency absent" handling as an empty tile. |
| Chunk road-class raster | `world_terrain.cpp:117-141` (`rasterize_chunk_road_classes`, `world_terrain_road_class_lookup`), `:605`, `:615` (`ClassLookup` wiring) |
| Road-class raster code | `core/src/road_classes.cpp:73-107` (`rasterize_chunk_road_classes`), `road_class_lattice`, `src_osm_tiles_for_chunk`, if no metadata user remains. `test_road_classes.cpp` goes with it. |
| `RoadSurfacePatch` | `core/include/rg/road_surface.h`, `core/src/road_surface.cpp` (whole files), `tests/unit/test_road_surface.cpp`. Its B8 deck/structure-id cases move to the S11 deck-builder tests. |
| Tunnel-roof inference | `core/src/road_structures.cpp:241-272` (inside `build_road_decks`) |
| Old deck builder | `road_structures.cpp:28-120` (`abutment`, `deck`) and `road_deck_height` :122-127, replaced in S11 |
| Terrain smoothing | `core/src/terrain_smoothing.cpp`, `core/include/rg/terrain_smoothing.h`, `tests/unit/test_terrain_smoothing.cpp` |
| Config | `world_config.h:106-107`, `world_config.cpp:450-469` (`terrain_smoothing`, `road_visuals.verge_drop_m`), the `test_world_config.cpp` smoothing case (:980-987), and the config wiring at `world_terrain.cpp:237-239, 274, 289` |
| Road-mode grip | `session.cpp:288-307` (paved/unpaved/off_road) and the road-mode `G2mTerrainSource` construction (:69, :86), replaced by class mode. `physics.road_surfaces` stays as a deprecated key that is rejected with a pointer to the new tables. |
| Bilinear visual ground | `world_terrain.cpp:538-542` `road_visual_tile`. Deleted by S10 (replaced by the ribbon view), not by S12. |

**Kept:**

- `road_decks()` (`world_terrain.cpp:441-447`), fed by the S11 builder.
- `cached_road_geometry_at` and `road_geometry_at` (:450-464), now backed by
  the plain cache.
- `road_segments_shared`/`extract_road_segments` (:621-727) and the
  `HeightTileSharedFetch` src.osm attachment (`terrain_mode.cpp`), for
  metadata only.

### 3.3 Choices

**Speed limit and road_ahead source**

| Option | Pros | Cons |
|---|---|---|
| **Keep src.osm RoadSegments as metadata (recommended for v1)** | No behaviour change for HUD and NPCs. Already streamed per L2 ancestor. | Two road sources stay alive (src.osm for metadata, roads.geom for geometry). |
| Migrate to roads.geom now (`envelope_road_segments` + entry tags) | One source | Not needed for G3/R3. Raises the risk of the switch commit. Speed tags may not be carried in roads.geom entries (would need checking). |

Recommendation: keep src.osm metadata. Migrate later as its own slice. Ask
geo2map to keep optional RoadSegment metadata in the class-mode
`FetchResult` (request G-2).

**Mapping table location**

| Option | Pros | Cons |
|---|---|---|
| **world_config tables, names resolved against the game surfaces.json (recommended)** | Data, not code. The owner tunes grip in the same file as grass. Engine-neutral. | Two tables to keep complete |
| Hard-coded switch in Session | Simple | Grip tuning needs a rebuild. Not data-driven. |

**SurfaceKind grip values (owner O-2: own surfaces).** Every SurfaceKind has
its own entry in the world_config `physics.surface_kinds` table; there is no
folding onto asphalt or dirt. surfaces.json gains named surfaces for the kinds
that have none today, each with a cited `lambda_mu`/`crr` that the owner tunes by
driving (grass precedent, docs/grass_grip.md):

| SurfaceKind | Surface |
|---|---|
| Asphalt | asphalt |
| Concrete, PavingStones, Sett, Cobblestone | `concrete`, `paving_stones`, `sett`, `cobblestone` (new, cited) |
| Gravel, Compacted, Sand | `gravel`, `compacted`, `sand` (new, cited) |
| Dirt | dirt |
| Grass | grass |
| Water | grass (not drivable in practice) |
| Unknown | the land_class mapping |

**Gate for the switch.** Owner: no zero-gap gate. The switch is still
measured, not blind. The S12 commit records:

1. `b8_contact_smoke.gd` at 2800/4390/5200;
2. the three B8 underpass dips at s ≈ 2852, 4453 and 5261 (G2M-011
   acceptance: no dip deeper than about 0.3 m), measured as profile minus a
   straight chord over ±60 m;
3. a route harness run (S5).

**Known regression risk.** The server carves ground over lower-layer tunnels
from the DEM-fitted profile and has no roof inference. The B8 dips can come
back. Recommendation: switch anyway (WIP is acceptable, and the owner ruled
"no fallback"). Report the measured depths to geo2map with request G-6
(census and DEM masking of ground over lower tunnels). Do not re-add roof
inference in the game.

---

## 4. G3-E road ribbons, junctions, furniture and markings

### 4.1 Rendering approach

| Option | Pros | Cons |
|---|---|---|
| A. Keep `road_visual.cpp` per-tag parsing | Exists | Lane counts are guessed from tags and disagree with the served cross-sections. No junction surfaces. Bilinear drape floats over or sinks into the Jolt-diagonal terrain. |
| B. Texture-space roads: terrain shader from the class plane plus a marking texture | No separate mesh, no z-fighting | The class plane is 1 m. Lane lines are 10-15 cm wide, so markings would need their own streamed high-resolution texture. Lane-accurate arrows are hard. |
| **C. Mesh ribbons from roads.geom cross-sections, draped on the physics triangle surface (recommended)** | Lanes, widths and junction trims come from the same data the server carves with. Visual equals physics by construction. Markings are vertex attributes, so they are exact at band boundaries. | New builder. Needs care at junction stitching and an upload budget. |

Recommendation: C near the car (default radius 1 km, configurable). Beyond
it, use the ClassWindow colour from slice 11; before the switch, the existing
chunk class raster. A second mesh LOD is not built in v1: at more than 1 km
the class colour is visually enough, and it halves the work. Revisit only if
the owner sees popping.

### 4.2 Ribbon geometry (core, engine-neutral: `core/include/rg/road_ribbon.h`)

- **Input:** one `g2m::RoadGeomTile` (owner-tile junctions, cross-sections,
  furniture) plus a `TriangleSurface`. The `TriangleSurface` is a height
  query that reproduces the physics heightfield exactly: 1 m node grid, Jolt
  diagonal, same tile origin. It is fed from the same decoded L0 the physics
  fill uses (game-carved before S12, server-carved after). It must not use
  bilinear `sample_l0_height`.
- **Columns:** at every band boundary from
  `RoadCrossSection::boundaries_at(s, out)`, plus extra columns so no column
  gap exceeds 1 m.
- **Stations:** every 1 m along s. Stations are also inserted at every
  cross-section change point (lane add/drop tapers) and at each junction
  trim s.
- **Vertex z:** `TriangleSurface(x, y) + surface_lift` (0.018 m, the existing
  value). Measured bound: a planar quad over a piecewise-linear 1 m
  triangulation can sit below the terrain by about
  Δslope × spacing / 4. For a 2.5 % crown and 1 m spacing that is about
  1.3 cm, which is under the lift. The S7 test enforces this and does not
  rely on the estimate.

  Alternative: clip ribbons exactly against the heightfield triangle grid.
  That is exact but gives 2-4× the triangles. Keep it as the fallback if the
  drape test fails on real data.
- **Kerbs:** visual only, using `carve_footprint` kerb lines. A 12 cm raised
  strip with a vertical face toward the carriageway, outside the physics
  (physics is flush, owner ruling).
- **Junctions** (from section 5 rings):
  - ear-clip the paved ring (road bed and outer rings as separate material
    bands);
  - subdivide edges to at most 1 m;
  - split each trim segment at the incoming ribbon's column positions, so
    every ribbon end edge shares vertices with the junction mesh;
  - give interior vertices a 1 m Steiner grid so the drape follows the
    terrain.

  Result: no T-junctions and no cracks. The z source is the same
  `TriangleSurface`, not `junction_height_m`. Visual equals physics; the
  server already carved the junction surface into the terrain.
- **Bridges:** ribbons over a bridge entry drape on the deck top (section 5)
  instead of the terrain, using the same lift. Tunnel entries are not drawn
  in v1.
- **Output:** per 256 m render chunk (aligned to terrain chunks), positions
  relative to the chunk origin, as `RoadVisualVertex` extended with
  marking-line attributes (4.3). Deterministic order: entries by
  (way_id, stretch), then junction id.

### 4.3 Interim markings until geo2map P3

| Option | Pros | Cons |
|---|---|---|
| A. No markings until P3 | Zero throw-away | Roads read as grey bands for weeks |
| **B. Derive line types from band data now, in the shape P3's descriptors will take (recommended)** | Visible lanes now. The vertex/shader contract survives P3: only the source of the line types changes. | Some rules (dashed vs. solid at no-overtaking zones) are guesses until P3 |
| C. Keep the old per-tag shader parameters | Exists | Disagrees with the served lanes |

How option B works:

- **Line type per band boundary**, from adjacent `CrossSectionBand`s:
  - edge line where paved meets unpaved or verge;
  - centre line where `direction` changes; solid if either band has
    `change` forbidden, else dashed;
  - lane line between same-direction counted lanes (dashed);
  - no line where `marked_right` is false.
- **Encoding:** each boundary column is duplicated, so the line sits exactly
  on a quad edge. The line type and the distance to the boundary are passed
  in UV2/COLOR. `road.gdshader` draws the line from these instead of from
  uniforms.
- **Arrows:** from the band turn bits, placed 5 m and 25 m before a stop line
  or junction trim.
- **Stop lines and crossings:** from section 6 via `furniture_quad(tile, f)`,
  as separate quads in the same chunk mesh.
- **Signals:** `signal_post` positions as instanced posts (simple mesh).
- **Inside a junction:** furniture inside a junction (derived from the owner
  junction's trims, since flag bit 3 is reserved) is skipped for lines.

### 4.4 Upload, budget and floating origin (`godot_ext`)

| Option | Pros | Cons |
|---|---|---|
| A. Keep `road_stream.gd` (one 1 km ArrayMesh per frame, GDScript worker) | Exists | No ms budget (one tile can cost several ms). Per-frame node repositioning in GDScript. Logic in `game/scripts`, which must stay thin. |
| **B. New `RgRoadView` in godot_ext mirroring `RgTerrainView` (recommended)** | The same proven pattern: worker build in core, a main-thread upload queue under a budget, and `set_render_origin` re-transforming chunk instances on an origin shift only | New class |

- **Budget:** one budget shared between terrain and roads, 0.8 ms per frame
  total (`RgTerrainView::set_upload_budget_ms` default, `rg_terrain_view.cpp:220`).
  Separate budgets would sum to 1.6 ms and break R2's 1 ms p99. Terrain
  chunks under the car go first. Roads take the remainder, plus a
  starvation guard: at least one road chunk every N frames.
- **Upload granularity:** a 256 m chunk (about 20-60k vertices near
  junctions). A chunk is uploaded whole or deferred.
- **Floating origin:** chunk vertices are stored relative to the chunk
  origin (double in session coordinates). The node transform
  = chunk_origin − render_origin is set only when the render origin moves,
  never per frame.
- **Removed:** `road_stream.gd`; `road_visual.cpp`, `road_visual.h` and
  `road_visual_tile` (`world_terrain.cpp:538-542`); the
  `get_road_visual_tile` binding (`rg_terrain_view.cpp:153-160`).

---

## 5. Bridges

### 5.1 Deck geometry from served profiles

The new builder replaces `deck()`/`abutment()`:

- Inputs are the bridge entries (layer/bridge flag; provenance Strict, Retry
  or BridgeCompleted) and their cross-sections.
- **Top surface:** `road_surface_height_m(tile, entry, s, t, params)` at
  columns from `carve_footprint` (paved and roadbed edges plus 1 m columns)
  and 1 m stations. So crown, cross-slope and features match the road
  exactly. The old 5-vertex crown and `attributes.width_mm` are gone.
- **Slab:** 0.5 m slab, with side faces. End faces are inset below the
  surface, decided in S13.
- **Triangle edges:** at most 20 m. The constraint is PHYS-047 (Jolt's
  large-triangle CastShape artefact). The 1 m stations satisfy it already.

### 5.2 Installation into physics

| Option | Pros | Cons |
|---|---|---|
| A. Current path: `create_body(MeshShape)` on the stepping thread, clock frozen until the required decks are in | Works at 7d5316f. Deterministic if the required set is defined deterministically. | Jolt's mesh BVH build runs between ticks on the sim thread. Long decks cost real time (to be measured in S11). |
| **B. Physics_sim R2: thread-safe `create_shape` off-tick + cheap `create_body(handle)` (recommended target)** | BVH build on a worker. Install cost is O(1). | Does not exist at 7d5316f. Requested from physics_sim with owner approval (O-1). |
| C. Decks as chains of oriented boxes | No BVH, available now | Box joints create steps and gaps on curved or vertically curved decks, which is exactly what the 2 cm acceptance forbids. Active box edges create ghost contacts for the chassis. |
| D. Decks as extra heightfield bodies | Heightfield spares exist (dd4799b) | A deck sits over terrain. A heightfield cannot represent the slab or overhangs, and it collides with the terrain tile's own heightfield at the abutment. |

Recommendation: A now, B when R2 lands (the owner agreed, O-1; the coordinator has asked physics_sim for R2, and a later slice after it lands moves the deck build off the sim thread). A is made deterministic and bounded:

- **Required set:** at each tick boundary, the decks whose footprint
  intersects (physics radius + 255 m) around each interest point. This is a
  pure function of the interest-point positions.
- **Order and pacing:**
  - install in (way_id, start_station) order;
  - at most K decks per tick boundary (K = 1 today; measure the cost);
  - freeze the clock (existing gate, `session.cpp:715`) while any required
    deck is not installed.
- Build the `ps::MeshShape` desc on the worker that decodes roads.geom.
  Only `create_body` stays on the sim thread.

Determinism test: shuffle the worker preparation order; BodyIds and
`state_hash` must be equal.

### 5.3 Abutment joint (Opus, after the switch)

What happens where the deck meets the carved approach:

- The server's clamp keeps terrain at least 0.55 m below the deck over the
  bridge footprint. It ramps through S(e/4).
- At the bridge node, D30's strict anchor makes the deck profile and the
  approach profile agree.
- The wheel `shape_cast` returns the maximum of terrain and deck under the
  wheel cylinder. So the joint is a step only if the deck end and the carved
  terrain disagree at the joint line.

S13 must decide and prove three things:

1. Where the deck mesh ends relative to the node (at the node, or overlapping
   the approach by 1-2 m with its top equal to the carved surface).
2. How the slab end face is shaped: vertical end face, or chamfered below the
   surface so the chassis cannot catch it.
3. How width mismatches are handled (deck paved width vs. carved approach
   footprint).

Acceptance is in 6.4. Tunnels stay non-drivable: no tunnel bodies, no tunnel
ribbons.

---

## 6. G3-F / R3 acceptance harness

### 6.1 Route and follower

- **Follower** (`core/include/rg/route_follower.h`, engine-neutral): pure
  pursuit on the route polyline plus a speed PI. Installed through
  `DriveScript::set_controller`.
  - Look-ahead = clamp(0.8 v, 4, 25) m.
  - Target speed = min(route speed cap, sqrt(a_lat · R)) with a_lat = 4 m/s².
  - The follower reads the published snapshot only, so it is deterministic.
- **Route:** `data/routes/home_g3_junctions.json` (rg.route/1). Produced by a
  tool that walks roads.geom junction records from the home spawn and picks
  a path through at least 20 distinct junction records. It includes B8
  bridge #4 (s ≈ 2852) and the L3014 bridge, each crossed in both directions
  (an out-and-back leg). The junction ids and the bridge entries are stored
  in the route's `criteria` so the test can recount them.

  Alternative: hand-pick the route in the editor. Rejected: a generated
  route is reproducible and the junction count is checkable.

### 6.2 The R3 bound: proposal

PLAN.md has no numeric R3 route bound. The R3 seam work (PHYS-045) bounds
spikes relative to a seamless reference: wheel Fz peak ≤ 1.05 × seamless,
chassis |az| ≤ 2 × seamless + 0.5 m/s². A real route has no seamless
reference. The proposal has two levels.

**Fixture level, exactly the R3 form.** Synthetic worlds:

1. A tile seam vs. the same surface as one heightfield.
2. A heightfield-to-deck joint with step h ∈ {0, 1, 2, 3} cm vs. the same
   surface as one mesh.

Each is driven at 10, 20, 30 and 40 m/s, both directions.

- h = 0 joints must meet the PHYS-045 relative bounds. Any miss is a
  physics_sim finding and goes into a repro bundle.
- The h = 2 cm run calibrates the route bound.

**Route level, an isolated-impulse metric.**

- For each wheel: r(t) = Fz(t) − median(Fz over t−4 … t+4 ticks), at 240 Hz.
- A real road feature at the 1 m heightfield resolution lasts at least
  1 m / v. At 40 m/s that is 6 ticks. A step shows up as a narrow residual.
- The bound is B(v) = 1.1 × max r measured on the 2 cm fixture, linearly
  interpolated over v. It is stored with its calibration provenance in
  `data/acceptance/r3_bound.json`.
- A route passes when:
  - no sample has r > B(v);
  - every Fz ≤ 3 × static corner load;
  - there is no chassis contact event with terrain or a deck.

So "within the R3 bound" means "no worse than a 2 cm step", which is the G3
deck/approach tolerance. The owner accepted this definition (O-3).

Alternatives considered:

- **A fixed absolute Fz cap.** It cannot separate steps from real road
  curvature at speed.
- **Chassis az only.** It hides single-wheel events.
- **Comparing against a smoothed re-drive.** There is no smoothed world to
  drive.

Check before calibrating: whether `WheelState::load` is the last substep or
a tick average. The metric uses whatever the snapshot carries; calibration
and route must match.

### 6.3 Surface check

- **Reference:** at each tick, for each wheel in contact, evaluate roads.geom
  `carve_footprint` at the contact point. This is independent of the class
  raster the physics uses.
  - Inside the paved band: the entry's SurfaceKind through the mapping
    table.
  - On a deck: the deck surface.
  - Otherwise: not checked against roads; land class is checked only in the
    forest/field control samples that the existing grip test already uses.
- **Exclusions:**
  - points within 1.5 m of any band edge or junction ring edge (one cell
    diagonal plus rounding: the physics surface is per 1 m cell);
  - wheels without contact.
- **Pass:** zero mismatches, and at least 50 % of wheel samples checked (so
  the exclusions cannot hollow out the check).

### 6.4 Bridges both ways

For every bridge on the route, in each direction:

- **Geometric step:** cast the real wheel shape along both wheel tracks
  across each joint at 1 cm spacing over ±1 m. The maximum height jump must
  be ≤ 2 cm.
- **Wheel force evidence:** every wheel's r(t) within ±0.5 s of the joint
  must be ≤ B(v), at route speed and at 10 m/s.
- **Safety:** no chassis contact.
- **Deck coverage:** the car is on the deck, i.e. the wheel surface id
  equals the deck surface, for the deck's whole length.

### 6.5 Repro capture for the physics_sim coordinator

On any exceedance, write `out/r3_repro/<route>_<tick>/`:

- **Pins:** racing_game, geo2map and physics_sim SHAs, server release id.
- **Terrain:** the 3×3 physics tiles' `TileSample` (heights and surface ids)
  around the spike, plus any deck meshes there (vertices and indices).
- **Vehicle:** the vehicle file, pose, twist, gear and wheel omegas 1 s
  before the spike.
- **Controls:** the control log for that 1 s plus 0.5 s after.
- **Report:** a short `report.json` (wheel, r, B(v), position).

`tools/rg_r3_replay` builds a plain `ps::World` with a dumped-tile
`ITerrainSource` and the static meshes. It places the chassis, calls
`World::reset_vehicle` (gear, hubs rolling with the chassis, clutch locked)
and replays the controls. It confirms the spike reproduces (r ≥ 0.5 × the
recorded value, same wheel, ±0.1 s) and prints the bundle path. The bundle
depends on neither geo2map nor the game, so physics_sim can run it in its
own tests.

---

## 7. Ordering against geo2map

| Work | Can start | Waits for |
|---|---|---|
| S1 pin bump + completion deletion | when geo2map commits D32/D35 | a committed SHA |
| S2 follower, S3 route, S4 bound calibration, S5 harness, S6 repro | now (S3 and S5's surface reference need S1) | - |
| S7-S10 ribbons, junctions, markings, RgRoadView | after S1 (encoding 7). They drape on whatever heightfield physics uses, so they are valid before and after the switch. | - |
| S11 deck builder from served profiles | after S1 | - |
| S12 carve switch | geo2map slice 11 (carved + class + class-mode fill + ClassWindow + bridge layer option + builtin release) and a pin bump | slices 9-11 |
| S13 abutment geometry + contact measurement | after S12 (the approach must be the server-carved one) | S12 |
| S14 R3 acceptance run | after S13 | S13 |
| Final markings | geo2map P3 | P3 |
| Off-tick deck install | physics_sim R2 | R2 requested (O-1 answered yes) |

Slice 12 (census) is informational for S12/S14. It does not block.

Running the harness (S5) before the switch gives a baseline on the
game-carved world. S12 and S13 are then judged against measured numbers,
not memory.

---

## 8. Slice plan

One commit per slice. "CI" is `tools\ci.ps1 -Only release` unless noted.
Smokes:

- `tools\smoke_test.ps1 -Drive`;
- `b8_contact_smoke.gd` (stations 2800/4390/5200);
- the route smoke `tools\smoke_test.ps1 -Route`, added in S5.

Real-data tests are hidden `[.][realdata]` cases that need `RG_G2M_HOME`.
Every slice records one sabotage that makes its new test fail, in the commit
message.

| # | Slice | Size | Model | Needs |
|---|---|---|---|---|
| S1 | geo2map pin bump + delete game completion/retry | M | Sonnet | committed D32/D35 SHA |
| S2 | Route follower (pure pursuit + speed PI) | M | Sonnet | - |
| S3 | 20-junction home route generator + data | M | Sonnet | S1 |
| S4 | Fz residual metric + fixture calibration of B(v) | L | **Opus** (numeric contact analysis) | - |
| S5 | Route acceptance harness + `-Route` smoke + baseline | L | Sonnet | S2-S4 |
| S6 | Repro bundle + `rg_r3_replay` | M | Sonnet | S4, S5 |
| S7 | Ribbon builder (columns, stations, triangle drape, kerbs) | L | Sonnet | S1 |
| S8 | Junction surfaces + seam stitching to ribbons | M | **Opus** (bridge/junction geometry) | S7 |
| S9 | Interim markings + furniture quads + shader | M | Sonnet | S7, S8 |
| S10 | `RgRoadView` (shared budget, floating origin); delete road_visual / road_stream.gd | M | Sonnet | S9 |
| S11 | Deck builder from served profiles + deterministic bounded install | M | Sonnet | S1 |
| S12 | Carve switch (consume carved + class, delete list 3.2, surface tables) | L | Sonnet | geo2map slice 11 + bump |
| S13 | Abutment joint geometry + bridge contact measurement both ways | L | **Opus** (bridge geometry, numeric contact) | S12, S4 |
| S14 | R3 acceptance run, PLAN/CLAUDE status, physics report | S | Sonnet | S13 |

### S1: pin bump and completion deletion

- **Change:**
  - bump `external/geo2map_engine` to the committed D32/D35 SHA;
  - delete `world_terrain.cpp:397-429` and `complete_road_profiles`
    (`road_structures.cpp:129-232` plus its declaration);
  - delete the tests that cover only those.
- **Tests:**
  - a unit test: a `RoadGeomTile` whose entry carries a profile with
    provenance BridgeCompleted/GroundCompleted/Retry gives a patch that uses
    that profile;
  - realdata: B8 accepted count ≥ before, the 7 B8 crossing structure ids
    unchanged, `test_route_grip` still 2103/2103 asphalt.
- **Sabotage:** skip entries with any provenance other than Strict → the
  unit test fails.
- **CI:** release; `-Drive`; `b8_contact_smoke`. Record the first-start
  re-derive time and the bridge's physics_sim.pin.

### S2: route follower

- **Change:** `rg::RouteFollower` (core) and a `DriveScript` controller
  adapter.
- **Tests:** synthetic flat world:
  - a 50 m-radius circle at 15 m/s with lateral error ≤ 0.5 m in steady
    state;
  - an S-bend within 1 m;
  - speed within ±1 m/s of target after 5 s;
  - identical inputs at 1 and 10 workers.
- **Sabotage:** flip the steering sign → circle test fails.
- **As built (S2):** `core/include/rg/route_follower.h`, `core/src/route_follower.cpp`,
  `tests/unit/test_route_follower.cpp` (tag `[route_follower]`). Deviations and
  refinements, all kept inside the design's intent:
  - The controller reads the chassis pose and motion from the `World` on the
    stepping thread right before `World::step()`, not from a published
    `FrameSnapshot` (the snapshot is only published by the real-time loop and
    a synchronous `Session::step()` publishes nothing). It is the same
    state, so it is just as deterministic; `FollowerState` is a plain struct,
    so a harness can also feed it from a snapshot.
  - Target speed is `min(cap, sqrt(a_lat / kappa))` as designed, plus a
    backward braking pass at 3.5 m/s² and 0 at the route end: a pure
    `sqrt(a_lat R)` is only valid at the corner, the car must slow before it.
    Curvature is route_check's definition (circle through the points ±10 m).
  - The projection onto the route searches only a window (5 m back, 12 m
    ahead of the previous projection; the first fix is limited to the first
    60 m, `seek(s)` starts mid-route). A global nearest-point search jumps onto
    the return leg of an out-and-back (S3's route crosses two bridges both
    ways) or onto a later lap; it is only the fallback when the car is more
    than 25 m from the route.
  - Reference point = rear axle, wheelbase / rear-axle x / steering limit come
    from the loaded `VehicleDesc` (`follower_params_for_vehicle`).
  - The adapter sets `ignition`, `assist.auto_clutch`, `assist.auto_shift` once
    and writes `steer`/`throttle`/`brake` every tick.
  - Measured on the flat world with car_sedan.json: 50 m circle at 15 m/s,
    steady state max |r - R| 0.43 m (rear axle 0.43 m outside the route);
    S-bend (R 60 m, 50 deg each way) max route distance 0.33 m; speed within
    0.30 m/s of the plan after 5 s on a straight.

### S3: junction route

- **Change:**
  - a tool `tools/rg_route_gen` (C++, links the g2m client), or a hidden
    realdata test that writes the file;
  - `data/routes/home_g3_junctions.json` with criteria listing the junction
    ids and bridge entries.
- **Tests:** a realdata test recounts junction rings crossed (≥ 20) and both
  directions over B8 #4 and the L3014 bridge.
- **Sabotage:** delete waypoints across one junction → count fails.
- **As built (S3):** `core/include/rg/road_network.h` + `core/src/road_network.cpp`
  (`RoadNetwork`, `plan_path`, `smooth_polyline`), the `rg.route/1` extensions in
  `core/include/rg/route_check.h`, `WorldTerrain::road_graph_tile`, `tools/rg_route_gen`,
  `data/routes/home_g3_junctions.gen.json` (the generator config) and the generated
  `data/routes/home_g3_junctions.json`. Tests: `tests/unit/test_road_network.cpp` (synthetic
  networks, planner, smoothing), `tests/unit/test_route_g3.cpp` (the committed file recounted
  from its waypoints, loader, grade exemption; a hidden `[.][realdata]` case checks the file
  against the real roads.graph store). Deviations from the text above:
  - **Junctions are derived from `roads.graph`, not from roads.geom junction records.** At the
    pinned geo2map (54d7083) there are no junction records (they arrive with encoding 7, S1).
    A junction here is a cluster (nodes within 30 m, `kJunctionClusterM`) of OSM nodes with >= 3
    distinct drivable neighbours and >= 2 ways; drivable = motorway..residential and their
    `_link`s, access not private/no (`kDrivableClassesVersion` 1, recorded in the file). The
    generator and the route do not depend on the pin bump.
  - **Junction and bridge lists are top-level arrays (`junctions`, `bridges`, `generator`), not
    inside `criteria`.** `criteria` stays a set of numeric limits and gained `min_junctions`; the
    lists are data, not limits. The loader is strict about all of them.
  - **The generator is a deterministic standalone tool plus the committed output**, not a hidden
    test that writes the file (the test side only recounts). Same config + same data pin gives the
    same bytes (checked by regenerating and comparing the SHA-256).
  - **`bridges` lists every bridge crossing on the path**, not only the two required ones (a
    marked traversal of the L3014 way is followed by an unavoidable third crossing, see the
    stats). B8 bridge #4 is a dual carriageway (oneway ways 1096866569 north-west and
    1096866567 south-east, 13.3 m each): "both directions" is each carriageway in its own
    direction (`group` "B8 bridge #4"); the L3014 bridge (way 14799333, 47.3 m) is two-way and is
    crossed with direction -1, +1, and -1 again.
  - **`check_route_on_world` exempts grade windows touching a listed bridge** from
    `max_grade_pct` (`RouteCheckParams::grade_exempt_s`, counted in
    `grade_window_exempt_count`). The L0 heights carry no deck: the L3014 bridge crosses a cutting
    and read 65 % without it. The criterion is unchanged everywhere else. The route also sets
    `min_corner_radius_m` 4 (junction turns; `home_r1_drive` sets 7 and 31 % grade for the same
    reason). `tools/route_check` now also reports `junctions_crossed` / `bridges_crossed` and fails
    on a missed `min_junctions` or an uncrossed listed bridge.
  - **Corners are rounded** (`smooth_polyline`: a quadratic Bezier corner of tangent length
    min(10 m * tan(turn/2), 0.45 * the shorter segment), resampled to <= 5 m): OSM junction turns
    are 90-degree kinks, which a driver cannot follow and `route_check` reads as radius 0. The
    route therefore deviates up to ~3.5 m from the way centre-line at a 90-degree turn.
  - Route (home world, pin 54d7083): 1819 waypoints, 9080 m; 25 distinct junction clusters
    (criterion 20); five bridge crossings; 46 physics-tile seam crossings; elevation 109-185 m;
    max grade 4.75 % outside the 210 exempt windows; tightest corner radius 4.9 m. It runs spawn
    (Engelsruhe, as `home_r1_drive`) -> the 12-node junction at OSM node 94147307 -> B8 over the
    north-west carriageway bridge -> the L3014 bridge (both directions) -> the B8 south-east
    carriageway bridge. `tools/route_check` fails only on the pre-existing `world:` spawn
    mismatch, which `home_r1_drive` shows too (the world config's spawn moved to the old spawn).
  - **Re-resolving the junctions after the S1 pin bump.** The identity of a junction is its set of
    OSM node ids (`junctions[].node_ids`; OSM ids survive a data pin, positions and geo2map record
    ids may not). With P2 records, for each stored junction find the record whose node set
    contains a stored node id (records carry `owner_node_id` and node ids), fall back to the
    nearest record centre within 30 m, and check the route passes through the record's ring
    (replace the recount radius `radius_m` by the ring). The route itself stays valid: it was
    planned on way ids and node ids, and the generator can re-run on the new pin (the config
    names ways and nodes, not coordinates). The `generator.geo2map_pin` field records which pin
    the file was generated against.

### S4: Fz metric and calibration (Opus)

- **Change:**
  - `core/include/rg/contact_metrics.h` (median residual, B(v)
    interpolation, envelope);
  - synthetic fixtures (seam vs. single heightfield; joint with
    h = 0/1/2/3 cm vs. single mesh; 10-40 m/s; both directions);
  - `data/acceptance/r3_bound.json`.
- **Tests:**
  - the B(v) derivation is pinned;
  - h = 0 meets the PHYS-045 relative bounds (otherwise a repro and a
    physics report, documented and not tuned away);
  - h = 3 cm exceeds B(v) at every speed, so the metric discriminates.
- **Sabotage:** calibrate from h = 0 → the h = 2 cm fixture is flagged.
- **Also:** state whether `WheelState::load` is per-tick or last-substep.

### S5: route harness

- **Change:**
  - a headless route runner (Session + follower over the route), collecting
    per-wheel Fz, surface, contact and chassis contacts;
  - evaluation of 6.2-6.4;
  - a JSON report;
  - `tools\smoke_test.ps1 -Route`.
- **Baseline:** run on the current game-carved world and commit the measured
  numbers in the report section of the commit message. Failures are
  expected and allowed (WIP).
- **Tests:** synthetic mini route with an injected 3 cm step → harness
  fails; without it → passes.
- **Sabotage:** surface reference ignores decks → bridge leg fails.

### S6: repro capture

- **Change:** the bundle writer (6.5) and `tools/rg_r3_replay`.
- **Tests:** S4's 3 cm fixture spike → bundle → replay reproduces it.
- **Sabotage:** omit deck meshes from the bundle → replay finds no spike and
  the test fails.

### S7: ribbons

- **Change:**
  - `rg::TriangleSurface` (Jolt-diagonal query over decoded L0, shared with
    the physics fill code path);
  - `rg::build_road_ribbons(tile, surface, params)` (4.2) with kerbs.
- **Tests:**
  - every vertex z = triangle surface + lift (bit-exact);
  - 10⁴ random points on the ribbon vs. physics `ray_cast` on a crowned
    synthetic road with a diagonal ridge: the ribbon is in [0, 4 cm] above
    the terrain;
  - column positions match `boundaries_at` to 1 mm;
  - adjacent chunks share border vertices bit-identically.
- **Sabotage:** bilinear height → ray-cast test fails.

### S8: junctions (Opus)

- **Change:** ring triangulation, Steiner grid, trim splitting at ribbon
  columns (4.2).
- **Tests:**
  - every ribbon end edge is matched by junction edges with identical
    vertices (no T-junction, no crack) on synthetic 3-way and 4-way
    junctions and on one realdata B8 junction;
  - drape bound as S7;
  - deterministic vertex order.
- **Sabotage:** skip trim splitting → T-junction test fails.

### S9: markings and furniture

- **Change:** line types per boundary (4.3); arrows from turn bits; stop
  lines and crossings via `furniture_quad`; signal posts; `road.gdshader`
  reads per-vertex line data.
- **Tests:** core descriptors for a one-way 2-lane, a two-way with a turn
  lane, and a no-overtaking stretch.
- **Sabotage:** swap direction handling → centre-line type test fails.
- **Smoke:** `b8_structure_shots.gd` screenshots for owner review.

### S10: RgRoadView

- **Change:**
  - godot_ext `RgRoadView` (worker build, upload queue, shared 0.8 ms
    budget object with `RgTerrainView`, origin-shift transforms);
  - `main.gd` wiring;
  - delete `road_stream.gd`, `road_visual.*`, `road_visual_tile` and the
    dictionary binding;
  - update `test_route_road_render_path.cpp`.
- **Tests:** core upload-queue accounting (priority by distance, starvation
  guard, budget never exceeded by more than one chunk's measured cost).
- **Smoke:** `-Drive` records main-thread upload p99 ≤ 1 ms.
- **Sabotage:** ignore the budget → p99 check fails.

### S11: decks

- **Change:** deck builder (5.1) replacing `deck()`/`abutment()`/
  `road_deck_height`; deterministic required set and K-per-boundary install
  (5.2) in `Session::sync_road_decks`. Tunnel-roof inference stays until
  S12. The install stays between ticks with the clock frozen (O-1: R2 is
  requested, not landed); moving the deck build to R2 is a later slice.
- **Tests:**
  - deck top equals `road_surface_height_m` at every station/column to 1 mm;
  - triangle edges ≤ 20 m;
  - a shuffled prepare order gives the same BodyIds and `state_hash`;
  - install cost per deck is measured and printed.
- **Sabotage:** drop the crown term → height test fails.

### S12: carve switch

- **Change:**
  - bump geo2map to the slice-11 SHA;
  - consume carved L0 and class (3.1); delete list 3.2;
  - world_config `physics.surface_kinds`/`physics.land_classes` tables, with
    `physics.road_surfaces` rejected;
  - owner O-2: the new named surfaces (`concrete`, `paving_stones`, `sett`,
    `cobblestone`, `gravel`, `compacted`, `sand`) in `data/surfaces/surfaces.json`
    with cited `lambda_mu`/`crr`, every SurfaceKind pointing at its own entry
    (no asphalt/dirt folding); the alphabetical SurfaceId renumbering re-records
    the hashes that pin ids (CLAUDE.md lists them);
  - ClassWindow render colour.
- **Tests:**
  - every SurfaceKind and LandClass is mapped (load error otherwise), and no
    SurfaceKind maps to a surface shared with another kind except Water/Unknown
    (O-2);
  - `test_route_grip` realdata (2103/2103 asphalt, 84/84 grass controls);
  - S5 harness rerun.
- **Recorded:** `b8_contact_smoke`; B8 dips (3.3); harness deltas vs. the S5
  baseline.
- **Sabotage:** remove the Concrete row → load error test fails.

### S13: abutments (Opus)

- **Change:** deck-end geometry (5.3) and any slab end-face change.
- **Measurement:** 6.4 for every route bridge, both directions, at route
  speed and 10 m/s. Report heightfield-vs-mesh numbers to the physics
  session through the coordinator (this is G3's open "162 m/s²
  heightfield-vs-mesh" item).
- **Tests:** a synthetic abutment fixture with the server clamp shape gives
  a step ≤ 2 cm and r ≤ B(v).
- **Sabotage:** end the deck 0.3 m short → step test fails.

### S14: acceptance

- **Change:** run the S5 harness on the switched world; write the results
  into PLAN.md's G3/R3 status and CLAUDE.md; attach repro bundles for any
  remaining spike.
- **CI:** release, `-Drive`, `-Route`.

---

## 9. Requests

### 9.1 To geo2map slices 10-12, beyond their current design

- **G-1.** Carved and class tiles must still be served when one of their up
  to four roads.geom inputs is "required dependency absent" at the region
  edge. Either carve without that road or treat it as absent. An error
  would freeze the game's gate, and the game has no fallback after the
  switch.
- **G-2.** Class-mode `FetchResult` (bridge) should keep the optional
  RoadSegment metadata the road mode carries today. The game uses it for
  speed limit and road_ahead.
- **G-3.** Class-mode `fill_physics_surfaces` should take full lookup tables
  (SurfaceKind 0-11 → SurfaceId and LandClass 0-13 → SurfaceId), not the
  three-way paved/unpaved/off_road choice.
- **G-4.** Export a Jolt-diagonal triangle-height helper for a decoded L0
  tile (the exact physics surface). Game ribbons and decks then drape on the
  same function the bridge fills from, instead of a second copy.
- **G-5.** Flags in `BuiltinTerrainDerivers::ordered()` and
  `builtin_local_release_params` to include the carved and class layers, so
  the game's in-process server can enable them without hand-wiring. Also
  prefetch support for carved tiles in the bridge (the physics_sim prefetch
  ring).
- **G-6.** In the slice 12 census: count ground stretches that pass over
  lower-layer roads not tagged as bridges or tunnels (the B8 underpass case,
  vault G2M-011). Report the profile dip vs. a straight chord. If the dips
  exceed about 0.3 m, consider DEM masking under such crossings. The game
  deletes its tunnel-roof inference at the switch.
- **G-7.** Before the switch: measured start-up and derivation cost of
  carved and class tiles for the home region (cold cache), so the game can
  size its gate and start-up progress.

### 9.2 To physics_sim

- **R2** (PLAN.md section 9, already listed): thread-safe
  `IRigidBackend::create_shape(ShapeDesc) -> ShapeHandle` and
  `create_body(handle)`. Not present at 7d5316f; only heightfield spares
  exist (dd4799b). Needed to move deck BVH builds off the sim thread. Per
  D16 it is briefed with owner approval (given, O-1; requested 2026-10-05).

---

## 10. Owner decisions

Answered by the owner 2026-10-05.

- **O-1. Prebuilt shapes: "Yes, ask now".** The coordinator has asked
  physics_sim for R2. S11 keeps the between-ticks install with the clock
  frozen (5.2 option A) until R2 lands; a later slice, after R2, moves the deck
  build to a worker and installs through `create_body(handle)` (5.2 option B).
- **O-2. Road grip: "Own surfaces".** Concrete, PavingStones, Sett,
  Cobblestone, Gravel, Compacted and Sand each become a named surface in
  `data/surfaces/surfaces.json`, with cited grip values (`lambda_mu`, `crr`)
  that the owner then tunes by driving, as was done for grass. There is no
  mapping onto asphalt or dirt. **Chosen: part of S12's SurfaceKind ->
  SurfaceId table (G-3), not its own slice.** Reason: before the switch these
  kinds do not reach the physics at all (the bridge's road mode only knows
  paved/unpaved/off_road), and surfaces.json assigns SurfaceIds alphabetically,
  so adding eight names earlier would renumber every id and move recorded
  hashes for nothing. S12 already owns the table, its completeness check and the
  hash re-record.
- **O-3. R3 route bound: "Accept".** The bound is: no wheel Fz spike worse than
  a 2 cm step at the same speed (B(v), calibrated in S4 and stored in
  `data/acceptance/r3_bound.json`), plus a cap of 3x the static corner load, plus
  no chassis contact with terrain or a deck. Section 6.2 stands as written.
