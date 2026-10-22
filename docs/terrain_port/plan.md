# Terrain feature port execution ledger

Branch: `codex/terrain-feature-port` in racing_game and geo2map_engine.
Game baseline: c816c3a (main f5ddce3 plus the tested world-service client/server).
Geo2map baseline: 5cdd643. Reference: BeamNG.Earth
606ba0fb966529659d07a3120c31c01c0fc3df92, MIT, copyright 2026 TecnaGamer.
Never execute its supplied native binaries. This is a feature/algorithm port,
not a promise of byte-identical output or a transplant of its companion server.

## Binding choices and why

The owner requested comprehensive adoption on 2026-10-22. Existing owner choices
remain binding unless explicitly superseded; research recommendations are not
owner rulings. Sources: G2M-031; racing_game PLAN sections 0 and 15.1;
geo2map PLAN 2.4; street_fidelity/p1_cross_section_design owner rulings;
the owner's October surface-alignment reports and instructions.

| Choice | Retain / discard / adapt | Reason and authority |
|---|---|---|
| Engine-neutral geo2map, also usable by farming games | Retain C++ facts/generators, thin renderer bindings | Owner: reuse outside racing_game; no Godot/BeamNG/physics types in generic geometry |
| Baked -> raw -> self-acquired source fallback | Retain one canonical derivation path | D2 owner ruling: all paths must agree on input bytes and output hashes |
| UTM tiles, orthometric heights, floating origin | Retain; adapt provider projections at import | D8/native DGM1 grid; replacing keys would discard working storage, profiles and physics |
| Determinism and immutable release identity | Retain pinned source hashes, sorted iteration, det math | Multiplayer hash verification D12 and cross-platform derivation |
| Worldwide elevation sources | Mapterhorn and Copernicus GLO-30 approved | Owner 2026-10-22: use both; retain source provenance and evaluate terrain results |
| Vertical datum policy | Missing verification is not an availability blocker | Owner explicitly rescinded the earlier restriction on 2026-10-22: prioritize resulting terrain; use known conversions where available, validate joins and road/physics alignment, record inferred adjustments |
| Source quality | Keep measured DGM1/DTM; correct only identified DSM fallback | Good ground must not be degraded by a global lower-resolution surface model |
| Road ownership | Retain served cross-sections, profiles, junctions and shared surface sampler | Owner: visible and logical asphalt/path surfaces must agree; stop terrain paint masquerading as paths |
| Heights / grades | Endpoints and mapped heights win; report exceptions | Existing owner ruling: do not clamp endpoint heights to a class-grade default |
| Kerbs / calming | Preserve 12 cm visual kerbs, flush physics for now; representable vertical calming | Binding Q1 option B / Q2 option C, minimum 1 m ramps; do not silently add kerb collision |
| Sources / network | Bounded extracts and pinned public COGs; no live OSM or Mapterhorn hotlink default | D13 and source-service limits; new request does not select a different source policy |
| Deployment | Integrated/local and estate server remain supported | D3: development on this PC, owner/friends; no imposed hosted dependency |
| Assets | Original or compatible licensed material sets, provenance recorded | Public repos; BeamNG stock assets are proprietary and outside the mod's MIT licence |
| Building appearance | Actual textured 3D models, not footprint extrusion as the result | Owner 2026-10-22 follow-up; silhouettes, facades and material detail are part of acceptance |
| Buildings | Mapped facts before inferred regional defaults; keep courtyards and foundations | P4 official LoD2 priority in Hessen; procedural facades fill missing appearance, not replace measured geometry |
| Vegetation | Generic scatter/season descriptors; no tree collision in v1 | D6 and future farming use; keep mapped trees before inferred scatter |
| Renderer | Geometry/material semantics in native library; thin Godot preview, UE-compatible data | Existing renderer-migration choice; do not introduce a new custom Godot shader dependency |
| Data distribution | Preserve all input licences and attribution separately from code | G2M-031 correction: local derivation is not a blanket ODbL exemption |

## Complete feature disposition and execution order

Each row is part of the port scope. A row is not complete until its integration
and acceptance evidence are recorded below. Dependencies establish order, not
permission to drop less visible features.

| Slice | Upstream features | Destination / adaptation | Acceptance |
|---|---|---|---|
| P0 | Source/config/provenance | Branches, inventory, decision ledger, MIT notices, fixture licence manifest | Review every upstream server module; no unclassified terrain subsystem |
| P1 | Buildings: floor bands, window bays, gable/hip/flat roofs, overhangs, regional looks | g2m world mesh generator, OSM metadata retained, original procedural facade palette; renderer only uploads | Concave/courtyard/raised/rotated fixtures; mapped total heights; bounded geometry; skyline LOD; actual game binding and captures |
| P2 | Markings: line ribbons, dimensions, dashes, arrows, crossing/stop trimming | Served cross-section client geometry; German rules in home region, country profiles elsewhere | Stable station dash phase across tiles; no paint under/over wrong deck; junction/crossing fixtures; no change to physics surface |
| P3 | Bridges: parapets, piers, arches, trusses, stays/suspension | Provenance-preserving OSM/Wikidata/NBI enrichment + render-only members on existing decks | Exact IDs first, ambiguous/replacement/parallel cases generic; clearances; no deck-height/collider changes; long-span limits explicit |
| P4 | Sidewalks, plazas, parking, gutters/drains, street furniture | Extend authoritative route surfaces/feature descriptors; avoid independent draped overlaps | Shared footprint/material ownership; existing kerb choice; bounded batches; junction/slope/underpass scenes |
| P5 | Source fetch, range reads, cache, retries, circuit breakers, budgets | Generic acquisition API separate from tile transport; pinned source manifests | Fake-clock/provider tests, cancellation, byte limits, conditional/range validation, offline/429/503/404 distinctions |
| P6 | DEM mosaic/fallback, COG sampling | Source-aware canonical imports with grid registration and explicit vertical datum | Mixed latitude widths, antimeridian/UTM boundaries, NoData, source priority, negative real ground, changed bytes => changed release |
| P7 | DSM -> ground morphology | Deterministic masked opening/slope/relief/deep-cut stages before upsampling | Whole raster vs tiled incl corners; composed halo; held-out DGM1 quality/time/memory; DTM unchanged |
| P8 | Grading, retaining walls, abutments, rock faces | Retain current ground profiles; adopt useful protections and visual cut/fill treatment | Parallel elevations, hairpins, bridge approaches and seam fixtures; measured improvement before algorithm replacement |
| P9 | Ocean/seacoast/islands, rivers/lakes | Generic water masks, bank/bed geometry and material descriptors | Preserve below-sea-level land, water datum, connected banks and tile ownership; no invented fallback land |
| P10 | Satellite/regional imagery, deshadow, colour matching, texture slots | Versioned source imagery and colour/material pipeline, original assets | Datum/alignment; border colour continuity, source attribution; memory/disk budgets; no proprietary textures |
| P11 | Vegetation, canopy, seasonal palettes | Deterministic instances from landcover/mapped trees/canopy, crop-aware exclusion | Stable IDs/seed, no road/building scatter, season/crop coexistence, bounded LOD and upload costs |
| P12 | Airports/runways, airport grading/marks, parking | Canonical surface/marking generators from mapped aeroway facts | Shared grade/runway contact, intersections, country markings, explicit unmapped-data fallback |
| P13 | Far field, LOD cells, prioritisation, prewarm, memory pacing | Adapt algorithms to current ring/terrain streamers and job scheduler, not BeamNG TerrainBlock/DAE | Cold/warm load, teleport, high-speed streaming, cancellation, origin rebase and VR; no nested-worker deadlocks |
| P14 | Roads widths/junctions/navigation, place selection/geocoding | Compare existing cross-section/routing/address features; adopt missing facts/fixtures | No width guessing where served lane evidence exists; no runtime OSM queries; existing routing invariants |
| P15 | Weather/seasons/gravity | Reuse existing authoritative time/weather and physics gravity; port only missing presentation inputs | No second authority or live-weather dependency; server outage/rejoin; UE-neutral descriptors |
| P16 | Packaging / credits / updates | Reproducible source and asset packages; credits from manifest; explicit region expansion | Clean-machine build, network-denied installed region, Windows/Linux/ARM64 tests, pinned dependency commits |

Upstream integration-only modules (earthd, Lua protocol, BeamNG TerrainBlock,
DAE writer, engine material slots, bc7 packaging, CPU pinning and mod updater)
are replaced by our server/protocol/mesh/upload/install equivalents. Their goals
(bounded work, upload pacing, cache invalidation, credits) remain in P0/P13/P16.
No discarded component is allowed to hide a missing user-visible feature.

## Fundamental incompatibility policy

Ask the owner with a concrete comparison only when no prior decision resolves it:
mapped LoD2 vs conflicting OSM
geometry; ambiguous current/replacement bridge identity; unavailable permissible
imagery/materials where appearance cannot be reproduced; a feature requiring a
change to the physical ground or the existing source-hosting policy.
Keep unaffected slices moving. Missing data is reported as missing; do not invent
measurements or treat an agent recommendation as owner approval. Public-provider
fallback must never silently change physics under an existing release ID.

## Execution evidence

- P0: dedicated sibling worktrees created from the tested baselines; other
  sessions' working trees left untouched. Plan records owner rationale separately
  from implementation choices.
- P1 partial: native roofs clipped to concave footprints/courtyards, metric UVs,
  eight material semantics, window frames/sills, residential entrances and chimneys.
  Godot uploads native geometry with pinned CC0 PBR assets. Official LoD2 ingestion,
  roof direction support and broader regional archetypes remain pending.
- P2 partial: bounded explicit paint clipped to draped road geometry, preserving
  configurable line width and three-argument binding compatibility.
- P3 partial: tagged bridge visual members on authoritative physical decks;
  inventory enrichment, supports/clearances and long-span acceptance remain pending.
- P7 partial: deterministic masked DSM correction with composed tiled halos;
  not yet connected to worldwide ingestion.
- Windows world suite: 1,341,164 assertions in 115 cases passed at geo2map 18cc542.
  Linux port fixtures: 115,251 assertions in 22 cases passed, including matching
  geometry/DSM golden hashes. Actual game probe: 533 buildings, 72 pitched roofs,
  48 entrances, 36 chimneys, 18,074 windows; 459,318 vertices, ~71 ms native
  building generation on this machine. Materials: 13.33 MiB; 12 pinned asset files
  verified. Windows game/native build and Vulkan probe passed. These timings
  are one tile, not a general streaming performance guarantee.
- P5/P6 next: source manifests/acquisition and worldwide elevation inputs, with
  source-aware joins and result checks. Both providers approved; no hard datum
  exclusion. Remaining P4-P16 acceptance is pending; this is not full parity.


### Geographic source slice (2026-10-22)

- geo2map 468cb4b adds bounded explicit GLO-30 acquisition with a SHA256 source
  lock, cached EPSG:4326 COG mosaic sampling, UTM reprojection, a native importer
  and explicit elevation-layer selection in TerrainService. Source metadata is
  retained through the height pyramid; established DGM1 defaults keep their config
  hash. Multiple imported elevation products can coexist in a scope.
- racing_game exposes `source_store.elevation_layer` (default DGM1) and forwards
  it to the shared terrain service. No existing playable region is replaced.
- Real N49/E008 GLO-30 import: 1,048,576 valid samples, no NoData, ~0.62-0.66 s
  sampling/encoding; byte-identical Windows/Linux containers. Native decoder/import
  suite: 2,351,806 assertions passed (154 cases, 12 external-data cases skipped).
  Windows world suite: 1,341,235 assertions in 116 cases. Linux service subset:
  69,284 assertions in 22 cases. Game world/config/surface regressions: 971
  assertions in 85 cases. Five acquisition tests passed with a fake network.
- P5/P6 remain partial: this is explicit offline acquisition/import and selectable
  serving, not automatic worldwide fallback. Mapterhorn WebP/PMTiles decoding,
  source blending, DSM filtering integration and complete regional inputs remain
  pending. Full native layer regression passed: 18,512,011 assertions in 345 cases.
  Real-data game mesh probe passed with 163 selected chunks / 731,055 vertices.
  Reproduce with `RG_GLO30_SOURCE_STORE` pointing to the documented probe store
  and Godot `--script res://scripts/geographic_terrain_test.gd` (headless supported).
- See `external/geo2map_engine/docs/pipeline/geographic_elevation.md` for commands,
  exact hashes, bounds and source-specific limitations.

### Prepared fallback and ground joins (2026-10-22)

- geo2map 8df9057 adds local PMTiles v3 extraction, pinned WebP-to-RGBA
  preparation, native Terrarium mosaic sampling and the combined
  `g2m_elevation_prepare` command. Explicit offline preparation supports ordered
  measured-primary -> Mapterhorn -> GLO-30 source priority. DSM correction runs
  at a globally aligned coarse source scale before canonical 1 m upsampling;
  known DTM and unknown source kinds pass through without DSM filtering.
- Blending preserves every valid primary integer sample. Only fallback cells
  receive bounded, distance-decaying overlap corrections. Composed halos and
  stable tie-breaking make tile boundaries deterministic. Input roles, hashes,
  missing primary dependencies and options are pinned in provenance; a changed
  source/provenance requires a new scope. Writer claims prevent concurrent
  preparation into the same destination.
- Mixed/unknown-reference composites use explicit `VerticalDatum::Unspecified`
  (container value 3), supported by road, rail and carving consumers. Unknown
  documentation is permitted per the owner's revised ruling. Known conflicting
  DHHN2016/EGM2008 dependencies still fail. Existing source containers are unchanged;
  older binaries reject the new value and must be upgraded with these sources.
- Windows native unit suite: 2,351,836 assertions / 161 cases passed, 12
  external-data cases skipped. World suite: 1,374,379 assertions / 125 cases.
  Affected container/road/rail/carve subset: 13,009 assertions / 30 cases passed
  on Windows and Linux. Source samplers: 342 assertions / 11 cases on both;
  ground/blend/DSM: 34,193 assertions / 12 cases on both. Native command tests:
  four passed on each platform. Offline Python tools: nine preparation and eleven
  extraction tests; Windows symlink cases skipped, Linux symlink cases passed;
  real WebP decoding passed using Pillow 12.3.0 on Windows (Linux codec case skipped).
- Real Hessen comparison: raw/prepared RMS errors (metres) 3.147/1.837,
  1.604/1.339, 2.067/1.490 and 17.547/17.438 at 16,384 points per tile.
  All 1,048,576 primary cells per tile were preserved exactly. The large fourth
  residual remains unresolved; these mixed-source checks do not establish global
  accuracy. Updated Windows/Linux preparation of tile 455/5421 produced identical
  canonical container SHA256
  `0fc3522a724232747c65a0e320fd27087d0b0a8141d9779f4321ff89ea3d76f8`.
- P5/P6/P7 advance through explicit prepared-source ingestion, not automatic
  worldwide acquisition. A bounded live Mapterhorn XYZ probe returned HTTP 403;
  current provider pixels remain unvalidated. The codec/PMTiles tests use generated
  fixtures, while GLO-30 tests also use pinned real COGs. No existing region is
  replaced, and full playable-region inputs and broader P0-P16 acceptance remain.
- Reproduction and limits:
  `external/geo2map_engine/docs/pipeline/elevation_preparation.md`,
  `mapterhorn_preparation.md` and `mapterhorn_pmtiles.md`.

- Game integration: rebuilt `rg_godot` and `rg_unit_tests` against 8df9057.
  Prepared Unspecified-reference composite loaded through the native service and
  Godot mesh binding: 181 chunks / 811,785 vertices, no script errors. Game
  terrain/config/surface/credits suite: 7,632 assertions / 98 cases passed; one
  optional local B8-data case skipped. This headless probe validates source and
  mesh plumbing, not driving, visual quality, VR or ARM64 hardware acceptance.
