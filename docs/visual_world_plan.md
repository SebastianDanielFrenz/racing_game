# Visual world plan (grass, fields, paths, textures, lights, time of day, weather)

Owner request 2026-10-14: low and tall grass, farmland with crops by season
(deterministic), dirt paths, real asphalt and surface textures, headlights that
light the road, time of day (progressive and set by the player) and a top-right
weather/time menu with condition icons and temperature.

Slices V1 (time of day + widget) and V2 (headlights) are built; see the end.

## Owner questions (each with a recommendation)

1. **Renderer.** Everything below is planned for `gl_compatibility` (today's
   setting, also used for VR via OpenXR). Forward+ would give cheaper many-light
   scenes, decals, better shadows and SSAO, but changes the look and the VR path.
   *Recommendation:* stay on Compatibility, build every slice to work there,
   and re-run the Forward+/Mobile A/B only after the stutter work is closed.
2. **Game clock start.** Today: the real date (UTC) at 14:00 local, progressing
   at x1 (real time). *Recommendation:* keep that; the menu offers stopped/x1/
   x10/x60/x600 and preset times. Alternative: always start at a fixed date.
3. **Weather owner.** physics_sim already has a full environment
   (`ps::environment`: calendar, sun, weather timeline with rain/snow/hail,
   water film and wet grip, ambient temperature); racing_game installs none.
   *Recommendation:* install it in `Session` in slice V8 so the menu's rain
   really makes the road wet; the game keeps its own civil clock (real
   longitude, EU summer time; the physics sun ignores longitude) and feeds
   the date/time into it.
4. **Headlights default.** Today: off until the player presses O.
   *Recommendation:* switch on automatically when the sun is below -2 deg
   unless the player switched them off this drive (rule in rg_core, threshold
   in data).
5. **Crop season source.** *Recommendation:* the in-game date from the time
   controller, a fixed seed per world in data (so every player sees the same
   crop on the same field on the same date), rotation by year.
6. **Temperature.** The widget shows the air temperature at the car from
   `data/world/environment.json` (ISA: 15 deg C at sea level minus 6.5 K/km);
   it does not follow time of day or season. *Recommendation:* take it from
   the physics climate once V8 installs it.

Assets: the owner allowed downloading textures, sounds and models within their
licence terms (2026-10-14). Use CC0 sets (ambientCG, Poly Haven); record each
set's source URL and licence in `data/credits.json` (provenance, even where CC0
asks for nothing). Size policy: 1K maps for ground/grass/crops, 2K only for
asphalt (the surface seen closest and longest); JPG/PNG sources, Godot imports
them VRAM-compressed (BC7/BC1 desktop, ETC2 on Quest/Steam Frame). Estimate:
1K set (albedo, normal, roughness) ~2-4 MB on disk, ~4 MB in VRAM with mips;
2K set ~8-14 MB on disk, ~16 MB VRAM. About 14 sets (12 x 1K + 2 x 2K) = ~60 MB
in git, ~80 MB VRAM. If the repo grows past that, move `game/assets/textures`
to git LFS (owner decision at that point).

## What exists today

- **Renderer.** `game/project.godot`: `gl_compatibility` for desktop and mobile.
  Compatibility is a multi-pass forward renderer: the directional light(s) go
  in the base pass, every omni/spot light touching an object costs one more
  pass of that object; limits `rendering/limits/opengl/max_lights_per_object`
  (default 8) and `max_renderable_lights` (default 32). Spot/omni shadows exist
  but each shadowed light renders its own shadow map every frame. No SSAO, SSR,
  SDFGI, volumetric fog or decals. MultiMesh works and is the main tool for
  vegetation; GPU particles are available for rain/snow.
- **Sky.** Before V1, `main.gd` had one fixed `DirectionalLight3D` (-55/-35 deg,
  energy 1.1, shadows to 300 m) and a default `ProceduralSkyMaterial`.
- **Terrain shading.** `game/shaders/terrain.gdshader`: `ALBEDO = COLOR.rgb`
  from `RgTerrainView`'s per-vertex RGBA8 (`rg::chunk_vertex_colors`:
  hypsometric height/slope ramp, roads painted grey/brown from the road-class
  lattice). Own `light()` with an `ambient_floor`. No textures, no world-space
  UVs (the shader avoids world coordinates because of the floating origin).
- **Roads.** `road.gdshader` ribbons (G3-E) with a view-space depth pull; plain
  colours. Road classes are paved/unpaved only; `highway=track` is the one
  unpaved class (3 m), `path`/`footway`/`bridleway`/`cycleway` are not rendered
  (geo2map `osm_roads.cpp` style table).
- **Surfaces.** `data/surfaces/surfaces.json`: 13 physics surfaces (asphalt,
  kerb, grass, dirt, low_mu, field, concrete, paving_stones, sett, cobblestone,
  gravel, compacted, sand); `rg/surface_kinds.h` maps `g2m::SurfaceKind` (from
  OSM `surface=*`) to them. Render colour uses only asphalt/dirt.
- **Land cover.** geo2map has `g2m.src.landcover` (ESA WorldCover classes, 10 m)
  and `g2m::LandClass` (TreeCover, Shrub, Grassland, Cropland, Bare, BuiltUp,
  Wetland, Water, ...). These do NOT reach the render chunks: the chunk
  `ClassWindow` only carries rasterised road classes. `g2m.src.osm` tiles keep
  the full OSM data (ways/relations with tags), so `landuse=*` polygons are
  available to a worker-side rasteriser.
- **Weather/environment.** `data/world/environment.json` (`rg::EnvironmentConfig`):
  ISA pressure/temperature with altitude, wind, gusts - feeds aero and engine
  air. physics_sim's `ps::environment` (W-series: rain/water film, wet grip,
  calendar, sun) is not installed by the game.
- **Lamps.** `exterior_lamps.gd` switched rig lamp materials' emission
  (indicators, brake, reverse); no light was cast. Rigs have `lights.head`
  materials but no lamp sockets. `CabinState::lights` existed without an action.

## Features

Per-frame cost rule for every slice: the main thread only uploads finished
buffers under a time budget (like the terrain's 0.8 ms); generation runs on
workers in rg_core; drawing is instanced (MultiMesh) or done in shaders.

| Feature | Approach | Data | Main thread / GPU | rg_core vs Godot | Assets |
|---|---|---|---|---|---|
| Time of day | Real sun/moon position (NOAA / Schlyter) from date, local time, lat/lon of the world origin; light/sky/ambient by sun elevation from a key table | `data/world/sky.json` | one native call at 10 Hz; sky radiance re-rendered only on colour change | astronomy, clock, lighting curve: rg_core; lights/env: `sky_controller.gd` | none |
| Weather widget + menu | Icon from (cloud cover, precipitation, thunder, night); menu items with enabled/reason from a view-model | same | 4 Hz label update | icon table, menu VM: rg_core; drawing: `weather_icon.gd` (procedural) | none |
| Headlights | SpotLight3D per headlamp cluster from the rig's head lamp geometry, dipped/main beam | `data/world/headlights.json` | +1 pass per lit object per spot (2 spots); no shadows | on/off state: DriverActions; lights: `headlights.gd` | none |
| Real textures for surfaces | Texture2DArray per class (asphalt, concrete, sett, cobble, paving, gravel, compacted, dirt, sand, grass, forest floor, field soil); road ribbons sample by surface id, terrain by land class; UV = (chunk origin mod period) + object-space position, passed as an instance uniform so the floating origin stays exact | surfaces + land classes | GPU: 2-3 texture samples per pixel, triplanar only on steep slopes | surface id per ribbon/vertex: rg_core; shaders: Godot | CC0 1K sets, 2K asphalt |
| Land classes to render chunks | Rasterise OSM `landuse`/`natural` polygons (+ WorldCover where OSM is empty) into a per-L2-tile class raster (2 m) on a worker; vertex class id in the chunk (COLOR.a or CUSTOM0) | `g2m.src.osm`, `g2m.src.landcover` | none (worker) | rg_core (like the road raster) | none |
| Low grass | Clumps (alpha-scissor cards, 3-4 per clump) scattered by hash(cell, seed) on Grassland/meadow/verges, density from the class raster, MultiMesh per 32 m cell within ~80 m, wind sway in the vertex shader, distance fade | class raster | upload <= 0.3 ms/frame; target GPU < 1.5 ms at 1080p | positions/transform buffers: rg_core workers; MultiMesh + shader: Godot | 1-2 CC0 grass atlases |
| Tall grass | Same scatter, taller cards, lower density, only meadow/`natural=grassland`/field edges; mown state by season | class raster + date | as above | as above | CC0 tall-grass atlas |
| Fields and crops | Per `landuse=farmland` polygon: crop = rotation[(hash(way id, world seed) + year) mod n], growth stage from day of year (crop calendar in data); far: shader rows along the polygon's main axis with stage colour; near (< 150 m) for tall crops (maize, rapeseed, cereals after heading): MultiMesh cards | OSM polygons, `data/world/crops.json`, game date | worker build; GPU like grass | crop choice, stage, row axis: rg_core; shader/MultiMesh: Godot | CC0 soil/stubble/crop textures |
| Dirt paths | Add `path`/`footway`/`bridleway`/`cycleway` (unpaved/paved by `surface=*`) to the ribbon style; `track` with `tracktype` grade3-5 gets a two-rut texture with a grass strip | OSM | ribbons as today | geo2map style table (request), ribbon width/texture id: rg_core | CC0 dirt/gravel |
| Weather wired | Install `ps::environment` in Session (preset or dynamic weather), wet grip, rain/snow GPU particles around the camera, wet road shader (darker, lower roughness, puddle mask), snow cover blend, lightning flash + delayed thunder | `ps::environment` climate | particles GPU only | weather state: physics + rg_core; visuals: Godot | CC0 rain/thunder sounds |

### Crops for the Main-Taunus region

OSM tags available: `landuse=farmland` (most fields; `crop=*` is rarely set
in Germany, so the crop is generated), `landuse=meadow`, `landuse=orchard`
(Streuobstwiesen, common on the Taunus slopes), `landuse=vineyard` (Hochheim am
Main is in the Main-Taunus-Kreis), `landuse=allotments`,
`landuse=greenhouse_horticulture`, `landuse=plant_nursery`, `landuse=grass`,
`natural=grassland`, `natural=scrub`, `natural=heath`. Where `crop=*` is set
it wins.

Crop list with assumed weights [AS] (Hessen arable mix, to be checked against
the Hessian statistics office): winter wheat 30, winter barley 12, winter
rapeseed 10, silage/grain maize 12, sugar beet 5, spring barley 6, rye/triticale
6, potatoes 2, field grass/clover 7, fallow/flower strip 4, other 6. Rotation
example: rapeseed -> wheat -> barley -> (maize or beet) -> wheat. Stages per
crop (day-of-year ranges in data): bare/ploughed, sown, emerged, tillering,
stem extension, heading/flowering (rapeseed yellow late April-May), ripening,
harvested stubble, cultivated. Same field, seed and date always give the same
look.

## Slices (in order) and acceptance

- **V1 time of day + widget (built).** rg_core `sky.h` (sun/moon), `time_weather.h`
  (clock, lighting, icon, menu VM), `RgSky`, `sky_controller.gd`,
  `weather_widget.gd`, `weather_icon.gd`. Accept: unit tests (sun within 0.5 deg
  and moon within 1 deg of PyEphem for Frankfurt at 8 instants, EU summer time
  switch, controller rollover, icon table, menu greying); pure-GDScript widget
  test; owner screenshots at noon, dusk, night.
- **V2 headlights (built).** `lights` action (O, rebindable), `cabin.lights`
  channel, head/tail lamp emission, `headlights.gd`. Accept: unit test of the
  action; GDScript test of cluster placement and aim; owner night drive shows
  the road lit to ~60 m dipped / ~160 m main beam; GPU frame time with lights
  on vs off measured on the owner's machine (target < +0.5 ms).
- **V3 surface textures.** Download CC0 sets, Texture2DArray, road ribbons by
  surface id, terrain grass/dirt by the existing ramp. Accept: screenshots of
  each surface; VRAM < 100 MB for all ground textures; frame time +-0.3 ms.
- **V4 land classes in render chunks.** Worker-side OSM polygon raster. Accept:
  synthetic-polygon unit tests; real store: share of chunk vertices per class
  vs polygon area within 2 %.
- **V5 low grass.** Accept: deterministic scatter test (same cell -> same
  buffer, any thread count); main-thread upload <= 0.3 ms/frame at 150 km/h;
  GPU < 1.5 ms at 1080p; no pop inside 60 m.
- **V6 fields and crops + tall grass.** `data/world/crops.json`. Accept: unit
  tests of crop choice/rotation/stage; screenshots in April, June, August,
  October.
- **V7 dirt paths.** Geo2map style request for path classes; rut texture.
  Accept: route samples on tracks classify unpaved; screenshots.
- **V8 weather wired.** `ps::environment` in Session, menu precipitation enabled,
  wet grip, particles, wet shader, temperature from the climate. Accept: grip
  drop on wet asphalt matches physics_sim's own test values; particle cost
  < 0.5 ms GPU.
- **V9 (optional) renderer A/B** for headlight shadows and more lights.

## Built in V1/V2 (2026-10-14)

- `core/include/rg/sky.h`, `core/src/sky.cpp`: Julian day, Gregorian date
  helpers, sun (NOAA) and moon (Schlyter + parallax, illuminated fraction)
  positions, EU summer time.
- `core/include/rg/time_weather.h`, `core/src/time_weather.cpp`: `SkyConfig`
  (`data/world/sky.json`, `rg.sky/1`), `TimeController`, `sky_lighting`,
  `weather_icon`, `TimeWeather` (state + menu view-model), location from the
  world config's UTM origin (with meridian convergence).
- `godot_ext/src/rg_sky.*`: `RgSky` binding. `get_aero_state()` gained
  `temperature_k`/`pressure_pa`.
- `game/scripts/sky_controller.gd`, `weather_widget.gd`, `weather_icon.gd`,
  `headlights.gd`; `exterior_lamps.gd` also drives head/tail lamp emission.
- Weather menu: Clear, Partly cloudy, Overcast are applied (sky colour and light
  level only); Rain, Snow, Hail, Thunderstorm are greyed out (no precipitation
  visuals, no wet-grip environment in Session).
- Known limits: the clock is not saved between runs; the mouse must be free
  (not captured by the cockpit view) to click the widget; UTM grid north vs true
  north is corrected, refraction is not modelled.
