# Mobile renderer (2026-10-15)

The game's default renderer is Godot's **Mobile** renderer (Vulkan). This is the renderer the Steam Frame
(standalone VR, ARM64 Linux) needs. The **Compatibility** renderer (OpenGL) is kept as a fallback.

## Choosing the renderer

- `game/project.godot`: `rendering/renderer/rendering_method = "mobile"`, and
  `application/config/project_settings_override = "user://renderer_override.cfg"`.
- Setting `graphics.renderer` (`rg::Settings`, choices `mobile` | `compatibility`, default `mobile`,
  `SettingDef::applies_on_restart`): the Settings screen marks it "applies when the game is started again".
  `game/scripts/renderer_choice.gd` turns it into the override file. `compatibility` writes
  `[rendering] renderer/rendering_method="gl_compatibility"` to `user://renderer_override.cfg`, and `mobile`
  removes the file. Godot reads that file at the next start. `main.gd` prints one
  `RG_RENDERER setting=.. running=.. driver=.. adapter=..` line, with the reason when the setting and the running
  renderer differ.
- A `--rendering-method` on the command line wins over the setting:
  - `tools/run.ps1 -Renderer compat|mobile|forward_plus`;
  - the release's `play_compat.cmd`;
  - `play_vr.cmd`, which passes `--rendering-method mobile --rendering-driver vulkan`.
- If Vulkan fails to start, Godot falls back to Compatibility on its own
  (`rendering/rendering_device/fallback_to_opengl3`, the engine default).
- Test: `godot --headless --path game --script res://scripts/renderer_choice_test.gd`. It checks the project keys, the
  override file written and removed, and the report line. Core: `rg_unit_tests "[settings][renderer]"`.

## Audit: what differs on Mobile

All numbers below come from lavapipe (Mesa's software Vulkan) under xvfb on Linux, compared with the Compatibility
renderer on the same machine. Nothing here has been checked on a real GPU yet.

| Feature | Finding | Change |
|---|---|---|
| Overall brightness | Compatibility draws a shadowed sun in an extra additive pass and blends it in sRGB space. Sun plus ambient therefore came out brighter than their linear sum. The same scene without sun shadows is identical in both renderers (probe: lit box top 0.7255 in both; with shadows, Compatibility 1.0 and Mobile 0.7255). Mobile's result is the correct one. The difference shows where sun and ambient overlap: lit car paint and lit ground are 10-25 % darker than under Compatibility with shadows on. | None. Brightness is set by the exposure/lights work (see below). |
| `terrain.gdshader` far colour | `layer_mean` (the texture mean used beyond `detail_far_m`, the macro variation and the puddle threshold) is an sRGB mean. Compatibility hands `source_color` samples to the shader in that encoding, but Mobile hands them over linear. Ground beyond `detail_far_m` came out about twice as bright as the textured ground in front of it (0.40 vs 0.18). | `mean_colour()` linearises the mean outside Compatibility (`#if CURRENT_RENDERER == RENDERER_COMPATIBILITY`). Far and near now match (0.213 / 0.211). Compatibility output is unchanged. |
| `terrain.gdshader` `light()` | The diffuse and specular terms give the same result in both renderers. | None |
| Terrain as a shadow caster | Compatibility ignored a terrain caster 2 km above the receiver (the receiver stayed sunlit). Mobile shadows it. On the real map, hills may cast shadows on Mobile that Compatibility never showed. | None. Worth a look on the real map. |
| `road.gdshader` depth pull | Computed in view space through `MODELVIEW_MATRIX`, so it does not depend on the renderer. | None. Not seen on screen: the flat world has no road ribbons. |
| Buildings, roads (StandardMaterial3D, vertex colours) | Nothing specific to the renderer. | None |
| Garage (`garage_scene.gd`) | The mirrored car twin and the softbox sky work. The floor glare and the walls read brighter, and the car is a little darker (the same blending effect as above). | None |
| `car_mesh_merge.gd` | Depth-pre-pass glass joins the shadow proxy only under Compatibility (checked there). On Mobile such instances keep their own shadow, which is correct but costs a little more. | None. Follow-up: check it on Mobile. |
| Headlights (`headlights.gd`) | On Mobile, lights are clustered in one pass (at most 8 spot + 8 omni per mesh), so each spot no longer adds a pass per lit object. The Compatibility cost note applies to the fallback only. | Documentation only (the file belongs to the lights work) |
| Sun soft shadows | On lavapipe, Mobile's soft directional shadow filter (`soft_shadow_filter_quality` 2, the default) left StandardMaterial surfaces fully unlit. Quality 0 (hard) works. This looks like a lavapipe defect, but it has not been checked on a GPU. The screenshots use quality 0 through a temporary override file. | None. **Needs a check on Windows.** |
| `RgTerrainView` (RenderingServer upload) | Plain arrays, no compression flags, one material per mesh: nothing specific to the renderer. | None |
| Glow, tonemap | The game's Environment uses no glow outside the garage. The garage's ACES and glow render on Mobile. | None |

What Mobile allows for the exposure work (a separate branch): auto-exposure through `CameraAttributesPractical`
(`auto_exposure_enabled`), an HDR 3D buffer (Compatibility clamps to LDR), working glow on emissive lamps, and spot
lights that no longer cost a pass each.

## Follow-ups

These are notes, not work in this change.

- Reflection probes in the garage. Mobile supports them; they could replace or complement the mirrored twin.
- MSAA 2x/4x or FXAA as a graphics setting.
- Glow on emissive lamps at night.
- Forward+ as an optional desktop quality renderer (owner question).
- Linux and ARM64:
  - the `linux-release` preset still forces `RG_BUILD_GODOT_EXTENSION` off;
  - `linux-arm64-release` (Steam Frame, `librg_godot.arm64.so`) builds it;
  - `game/rg_godot.gdextension` already names `librg_godot.so` for linux x86_64; a desktop Linux x86-64 preset with the extension on, and a CI leg for it,
    are still missing for the Linux PC target (NVIDIA and AMD Vulkan drivers);
  - `tools/frame_install.ps1` already starts with `--rendering-method mobile --rendering-driver vulkan`.
- `RG_BENCH_SYNTHETIC_TERRAIN` (main.gd, bench only) does not keep its drop. `camera_director.set_terrain_view`
  resets the pushed render origin, and the next frame puts the sim's origin back. The synthetic hills then sit above
  the flat ground box, so on Mobile they shadow the whole box.
