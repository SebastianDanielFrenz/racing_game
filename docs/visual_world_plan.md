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
- Weather menu: every preset is enabled. Cloud cover drives the sky; since V8
  (below) every preset also drives the physics weather.
- **V8 physics part (built 2026-10-14, owner answers Q1-Q4).** Clock start kept
  (real date, 14:00, x1). Headlights follow the car's light switch (Ventura
  `sw_lights` submits the `lights` action; the hypercar's rotary switch is posed
  from `cabin.lights`) and the O key; automatic headlamps since 2026-10-15
  (below). `data/world/climate.json`
  (`rg.climate/1`, `core/include/rg/climate.h`) is the one source of the air
  temperature: monthly mean + daily range (interpolated between month
  midpoints), cosine with a 14:30 local-mean-solar peak, damped by cloud cover,
  minus 6.5 K/km above the station. `rg::Session` installs a
  `ps::environment::Environment` when `SessionConfig::weather` is set (flat and
  terrain; Preset weather, surface conditions, wet grip; the flat ground becomes
  a mesh tagged asphalt), pinned to that temperature; `rg::WeatherSync`
  (`weather_sync.h`) feeds the game clock (jumps, day change, drift) and the
  chosen preset into it. Measured (test_session_weather): one hour of 4 mm/h
  rain -> 0.4 mm film, condition factor 0.85, braking from 25 m/s 33.4 m dry vs
  49.4 m wet. Limits: snow adds no water film (physics W8 not yet), so snow
  does not lower grip yet; wetting follows the 60 s environment grid, so at x1
  a dry road takes minutes of rain to get wet; no precipitation visuals or wet
  shader (no new Godot shaders, owner rule; UE work paused).
- Known limits: the clock is not saved between runs; the mouse must be free
  (not captured by the cockpit view) to click the widget; UTM grid north vs true
  north is corrected, refraction is not modelled.

## Automatic headlamps and exposure (2026-10-15, owner comment on PR #5)

Owner: headlights should come on by themselves at night; brake lamps should not
glow in daylight but show clearly at night; headlights should be visible by day
without lighting the road. Done with constant physical lamp intensities and a
scene exposure, not with day/night switches on the lamps.

- **Illuminance.** `rg::SkyLighting::illuminance_lux` (horizontal lux): per
  lighting key in `data/world/sky.json` (0.002 starlit night, 3.4 at -6 deg,
  600 sun on the horizon, 15k/55k/100k at 10/30/60 deg; textbook-order values,
  recalled, unverified), log-interpolated, x (1 - 0.8 cover^1.5) for clouds
  (~0.3 overcast), plus moonlight (0.25 lux full moon at the zenith).
- **Exposure** (`core/include/rg/exposure.h`): display = luminance (cd/m2) x
  key x pi / A, A = adapted illuminance clamped to [50, 150000] lux and
  adapted in log space (tau 1.5 s; a clock jump or weather choice snaps).
  Night exposure 0.094, noon ~4.7e-5 (about 2000x apart). `TimeWeather::sample()`
  carries it; `RgSky.get_state()` reports `illuminance_lux`, `exposure`,
  `adapted_lux`, `night_exposure`. `combine_exposure()` is the seam for a
  renderer-measured exposure (geometric blend).
- **Lamps** (`data/world/headlights.json`, `game/scripts/light_exposure.gd`):
  spots in candela (dipped 15 kcd, main 60 kcd, tail 12 cd, brake 150 cd,
  reverse 300 cd), lenses in cd/m2 (head 40k, indicators 25k, reverse 10k,
  brake 8k, tail 800), all x exposure. At night the beams light the road and
  the red lenses saturate (clipped per colour peak so they stay red); at noon a
  beam adds <1 % to the daylight on the road, the brake lens shows a dim red
  (display ~0.38, a sunlit white surface ~1.5) and the tail lens is dark;
  head/indicator lenses stay lit. Rear spots fall off as inverse square.
  Godot spots have no beam pattern: the beams keep a flatter falloff
  (`attenuation` 0.9/0.8) anchored so the illuminance at `reference_m` is exact.
- **Renderer.** Measured with a probe scene (2026-10-15): in gl_compatibility
  `Environment.tonemap_exposure` scales light energies correctly but sky and
  emission values far above 1 clip before the exposure (a 1000x sky at exposure
  0.001 renders white); the Mobile renderer clipped the whole scene at large
  values. So the sky keeps its tuned look at environment exposure 1 and only the
  lamps are multiplied by the exposure - identical on screen for the linear
  tonemap. For an HDR path (Forward+ float buffers, later HDR output): scale the
  sun/sky/ambient by 1/exposure and set the camera exposure to `exposure`
  instead of scaling the lamps; on Mobile a measured auto exposure
  (`CameraAttributesPractical.auto_exposure_*`) can replace or blend with the
  sky value through `combine_exposure`. Cockpit/dash lamps and unshaded
  materials are not exposure-scaled yet.
- **Automatic headlamps** (`rg::DriverActions`, `CabinConfig::auto_lights_*`
  from headlights.json `auto`): dipped beam on below 1000 lux for 2 s, off above
  7000 lux for 10 s (UN R48 6.2.7.6 thresholds, recalled, unverified); the first
  reading decides at once. A manual choice (O key, cockpit switch) holds until
  the sensor's decision next changes; main beam is kept while it stays dark.
- **Tests.** `test_exposure.cpp` ([exposure]), `test_driver_actions.cpp`
  ([auto_lights]), `visual_world_test.gd` (lamp energies follow the exposure).
  Shots: `game/scripts/lights_exposure_shots.gd` with the states that
  `rg_unit_tests "[exposure_shots]"` writes.
