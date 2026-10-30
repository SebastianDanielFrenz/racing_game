# In-game LOD settings, minimap and route planner (R4map, 2026-10-14)

## LOD settings

Four settings on the Settings screen, section "Level of detail". Ranges, defaults and the road presets live in
`data/settings/lod_settings.json` (`rg.lod_settings/1`); `rg::install_lod_settings` adds them to the settings schema at
`RgShell.initialize`. Every default reproduces the behaviour before the settings existed.

| key | default | range | effect |
|---|---|---|---|
| `lod.view_distance_m` | 20000 | 4000..40000 m | `LodParams::max_distance_m` of the render terrain |
| `lod.terrain_detail` | 1.0 | 0.5..3.0 | multiplies `LodParams::range0_m` (250 m) |
| `lod.road_detail` | standard | low / standard / high | `g2m::RenderRoadLod` per chunk level (`standard` == `default_render_road_lod()`) |
| `lod.map_detail` | 1.0 | 0.25..4.0 | `g2m::route::MapConfig::detail_scale` of the minimap |

Path of a change: `main.gd _apply_setting` -> `RgTerrainView.set_view_lod(choice)` and `RgNavigation.set_view_lod(choice)`.
Terrain: `WorldTerrain::set_render_road_lod` + `TerrainViewStreamer::set_source` (new LodParams/road LOD); the streamer
reselects at once and rebuilds the RESIDENT chunks only on the levels whose `render_road_lod_key` changed
(`changed_road_lod_levels`). Those come back in `TerrainViewDiff::replaced` and are swapped in place with
`replace_chunk_at` (upload first, old RIDs freed after), before the diff's removals, so the view never has a hole.
Map: `MapView::set_config`; `MapTileCache` drops only LODs whose key changed.

## Route planner and minimap

`rg::RoutePlanner` (`core/include/rg/route_planner.h`) builds one routing graph for the loaded region at world load, on
its own worker: the level-2 `g2m.roads.graph` tiles in a square of `region_radius_m` around the spawn
(`WorldTerrain::road_graph_tile`, Warm priority), `g2m::route::assemble_region_graph` + `build_region_routing` (ALT
landmarks). Route requests are coalesced (the newest wins): heading-aware snap of the car, snap of the destination,
A* + ALT, `route_polyline`, `build_guidance`. `route_progress` gives the distance along, off-route flag and the next
`maneuvers_ahead` manoeuvres for the HUD. Configuration: `data/world/route_planner.json` (`rg.route_planner/1`).

`rg::MapView` (`map_view.h`) serves the minimap geometry: `select_map_lod` for the radius, `g2m::route::MapTileCache`
tiles built from the same level-2 roads.graph tiles, flattened to session-metre polylines, on a worker.

Godot: `RgNavigation` (binding), `game/scripts/navigation_hud.gd` (attaches only while a real world runs) and
`minimap.gd` (heading-up drawing, around the drone in FPV drone mode). Destination: F10 address search, button "Set as route destination".

## Limits

- No turn restrictions: roads.graph does not carry them yet (geo2map G4 slice 3). The graph is built without them.
- No automatic re-route when off route; the HUD shows "off route" and a new request re-plans.
- The routing region is fixed at world load (10 km square half-size); destinations outside it are unreachable.
- Real-data routing has not been run in this increment (no store on the build machine): only the synthetic grid tests
  (`rg_unit_tests "[route_planner],[map_view]"`) cover it.
- The minimap is square, not round.
