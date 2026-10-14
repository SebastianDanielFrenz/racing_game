# Visual world plan (grass, fields, paths, textures, lights, time of day, weather)

Owner request 2026-10-14: low and tall grass, farmland with crops by season
(deterministic), dirt paths, real asphalt and surface textures, headlights that
light the road, time of day (progressive and set by the player) and a top-right
weather/time menu with condition icons and temperature.

Scope (owner, 2026-10-14): the game moves to Unreal Engine within days. No new
Godot shaders. Everything that decides WHAT is shown (clock, sun/moon, weather
state, icon, crops and seasons, beam parameters) lives in rg_core with tests,
its numbers in `data/`; the renderer only maps the results onto its materials,
foliage and lights. Godot keeps a thin preview (sky light, widget, headlights).

Built: V1 time of day + widget, V2 headlights, V6a crop/season logic (no
rendering). See "Built" at the end.

## Owner questions (each with a recommendation)

1. **Godot preview.** The Godot side stays a thin preview (sky light, widget,
   headlights on `gl_compatibility`, where every spot light costs one extra pass
   per lit object). *Recommendation:* no Godot work beyond that; grass, crops and
   textures are rendered first in Unreal.
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
licence terms (2026-10-14). Use CC0 sets (ambientCG, Poly Haven); keep them
under `data/textures/<surface>/` (engine-neutral, for the UE import) and record
each set's source URL and licence in `data/credits.json` (provenance, even where
CC0 asks for nothing). Not done yet: this container's egress proxy refuses
ambientcg.com, polyhaven.com/dl.polyhaven.org and cdn.jsdelivr.net (HTTP 403 on
CONNECT), so the download has to run on the owner's machine (or the hosts be
allowed). Size policy: 1K maps for ground/grass/crops, 2K only for
asphalt (the surface seen closest and longest); JPG/PNG sources, the engine imports
them block-compressed (BC7/BC5/BC1 desktop, ASTC/ETC2 on Quest/Steam Frame). Estimate:
1K set (albedo, normal, roughness) ~2-4 MB on disk, ~4 MB in VRAM with mips;
2K set ~8-14 MB on disk, ~16 MB VRAM. About 14 sets (12 x 1K + 2 x 2K) = ~60 MB
in git, ~80 MB VRAM. If the repo grows past that, move `data/textures`
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

## Features (engine-neutral core, Unreal mapping)

Per-frame cost rule for every slice: generation runs on workers in rg_core and
is deterministic; the engine instances (UE: PCG / instanced static meshes /
foliage, Nanite where available) and does the shading in materials.

| Feature | rg_core (engine-neutral, tested) | Data | Unreal mapping | Godot now |
|---|---|---|---|---|
| Time of day | `rg::TimeWeather`: civil clock (rate, user override, EU summer time), sun/moon position from date + lat/lon, light level and sky colours by sun elevation and cloud cover | `data/world/sky.json` | Directional Light (sun, Atmosphere Sun Light index 0) + second directional (moon, index 1) rotated from `sun`/`moon` directions; Sky Atmosphere + Sky Light (real-time capture) + Exponential Height Fog; Volumetric Clouds coverage from `cloud_cover`; the lighting key table becomes optional (UE's atmosphere does the colours) - keep it for exposure/ambient targets | `sky_controller.gd` |
| Weather widget + menu | icon table (`weather_icon`), menu view-model with enabled/reason, temperature text | same | UMG widget top right reading the same view-model; icons as SVG/texture | `weather_widget.gd`, `weather_icon.gd` (procedural) |
| Weather wired | V8: `ps::environment` in Session (weather timeline, water film, wet grip, temperature); the view-model then enables rain/snow/hail/thunder | physics_sim climate | Niagara rain/snow/hail around the camera, wet material parameter (darken + roughness + puddle mask), lightning flash on the moon/sun light + delayed thunder | - |
| Headlights | `lights` action, `cabin.lights` (0 off, 1 dipped, 2 main), head/tail lamp projection | `data/world/headlights.json` (range, cone, energy, pitch) | two Spot Lights per car at the lamp geometry (or a socket added in the car asset), IES profile for a real low-beam cut-off, Lumen picks up bounce; shadows on for the player car only | `headlights.gd` SpotLight3D |
| Surface textures | surface id per road ribbon / ground cell (exists: `surface_kinds.h`, 13 surfaces) | `data/surfaces/surfaces.json` + texture sets under `data/textures/` | Landscape material / road material with a texture array or layer per surface id (asphalt, concrete, sett, cobble, paving, gravel, compacted, dirt, sand, grass, field soil, forest floor) | none (kept per-vertex colours) |
| Land classes | V4: rasterise OSM `landuse`/`natural` polygons (+ WorldCover where OSM is empty) into a per-tile class raster (2 m) on a worker | `g2m.src.osm`, `g2m.src.landcover` | Landscape layer weights / Runtime Virtual Texture from the class raster; PCG graphs keyed on class | - |
| Low / tall grass | V5: deterministic scatter per 32 m cell (hash of cell + seed), density and height by class and season (meadow mown/tall from `crops.json` meadow calendar) | class raster, `crops.json` | PCG grass/foliage instances (or Landscape Grass Types) from the core's density/height per cell; wind via UE's foliage wind | - |
| Fields and crops | V6a built: `rg::field_look` (crop by rotation + seed + harvest year, stage by date, height, ground cover, colour, row spacing; row direction from the polygon) | `data/world/crops.json` | per field: a material instance with stage colour, row direction/spacing and cover; tall crops (maize, rapeseed, cereals after heading) as PCG instanced meshes near the camera, impostors/material far | - |
| Dirt paths | V7: add `path`/`footway`/`bridleway`/`cycleway` to the ribbon style; `track` + `tracktype` grade3-5 = two ruts with a grass strip | OSM | road ribbons as spline meshes / procedural mesh with the rut material | - |

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
- **V3 surface textures.** Download CC0 sets into `data/textures/` with credits
  entries (on the owner's machine: hosts blocked here); UE landscape/road
  materials by surface id. Accept: one set per physics surface; screenshots;
  VRAM < 100 MB for all ground textures.
- **V4 land classes.** Worker-side OSM polygon raster in rg_core (class per
  2 m cell, field way id per cell for `field_look`). Accept:
  synthetic-polygon unit tests; real store: share of chunk vertices per class
  vs polygon area within 2 %.
- **V5 low grass.** rg_core scatter (density/height per cell). Accept:
  deterministic scatter test (same cell -> same instances, any thread count);
  in UE: game-thread cost of instance updates <= 0.3 ms/frame at 150 km/h,
  GPU < 1.5 ms at 1080p, no pop inside 60 m.
- **V6a crop/season logic (built).** `rg::field_look`, `data/world/crops.json`,
  unit tests. **V6b** UE field materials and tall-crop instances: screenshots
  in April, June, August, October.
- **V7 dirt paths.** Geo2map style request for path classes; rut texture.
  Accept: route samples on tracks classify unpaved; screenshots.
- **V8 weather wired.** `ps::environment` in Session, menu precipitation enabled,
  wet grip, particles, wet shader, temperature from the climate. Accept: grip
  drop on wet asphalt matches physics_sim's own test values; particle cost
  < 0.5 ms GPU.
- **V9 (UE)** headlight IES profiles and per-car shadowed spots.

## Built (2026-10-14)

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
- `core/include/rg/crops.h`, `core/src/crops.cpp`, `data/world/crops.json`
  (`rg.crops/1`): 13 crops/calendars (winter wheat, winter barley, winter
  rapeseed, maize, sugar beet, spring barley, rye/triticale, potatoes, field
  grass, fallow, meadow, orchard, vineyard), 6 weighted rotations, fixed
  calendars for meadow/grass/grassland/orchard/vineyard, `field_look`,
  `rotation_crop`, `field_row_angle_deg`. Nothing renders it yet.
- Weather menu: Clear, Partly cloudy, Overcast are applied (sky colour and light
  level only); Rain, Snow, Hail, Thunderstorm are greyed out (no precipitation
  visuals, no wet-grip environment in Session).
- Known limits: the clock is not saved between runs; the mouse must be free
  (not captured by the cockpit view) to click the widget; UTM grid north vs true
  north is corrected, refraction is not modelled.
