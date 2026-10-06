# racing_game — architecture map

Pure map of what exists and how to build/test/run it. Rationale and design
decisions live in chat/task reports, not here (this repo does not use the
knowledge vault workflow other repos on this machine use - see the R0 task
report for the full decision/pitfall list). Design/phase plan: `PLAN.md`.

## Layout

```
racing_game/
  CMakeLists.txt, CMakePresets.json   top-level build config
  README.md                           setup pointer
  CLAUDE.md                           this file
  PLAN.md                             design and phase plan
  .gitmodules                         external/physics_sim + external/geo2map_engine submodule pins
  external/
    physics_sim/                      git submodule, READ-ONLY from this repo (another session owns it), pinned by commit - see "physics_sim submodule" below
    geo2map_engine/                    git submodule, READ-ONLY from this repo (another session owns it), pinned by commit - see "geo2map_engine submodule" below
  core/
    include/rg/
      session.h                       rg::Session, SessionConfig (optional `terrain`, optional `startup`), FrameSnapshot, WheelSnapshot, StreamingStatus, kControlChannelNames[]/kControlChannelCount; R9: StartupProgress/SessionCancelled, set_vehicle_control, request_relocate/request_reset_to_spawn, make_session(); R9b: set_followed_vehicle/followed_vehicle/followed_loss (drone-follow second physics interest point), kNpcTruckVehicleId, StreamingStatus followed_id/followed_lost/followed_lost_id/interest_points - see "Session terrain mode (R4)" and "Player modes and world switch (R9)" below
      player_mode.h                   rg::PlayerMode/ModeRules/rules_for(mode, drone_target)/PlayerModeMachine (incl. set_drone_target)/DroneCandidate/next_drone_target/unattended_controls (R9/R9b, engine-neutral mode state machine) - see "Player modes and world switch (R9)" below
      walker.h                        (R9c, engine-neutral, no Jolt/Godot type) rg::WalkerController (kinematic capsule body in the ps::World, own collision with collide_shape_into/ray_cast: walk 1.4 / run 5 m/s, gravity, jump, 15 cm step-up, 45 deg slope limit, snap-down 0.3 m, hold when no ground is loaded), WalkerConfig/WalkerInput/WalkerState, OrientedRect + vehicle_footprint() (the own car's wheel/chassis footprint the walker is blocked by - the thin physical chassis box is ignored), within_enter_range(), select_spawn_spot() (driver side, passenger side, rear, front, above the roof) - see "On foot (R9c)" below
      terrain_mode.h                  rg::TerrainModeConfig (SessionConfig::terrain), rg::HeightTileSharedFetch (g2m::phys::IHeightTileFetch over a HeightTileFetchFn - WorldTerrain::height_tile_shared), make_terrain_mode(WorldConfig, WorldTerrain) + a pure (WorldConfig, SessionFrame, fetch) overload
      drive_script.h                  rg::DriveScript (header-only, Godot-free): tick-indexed sample-and-hold control events in DRIVE ticks (since spawn) plus an optional per-tick controller hook (the R5 autopilot's extension point); Session::set_drive_script
      fixed_rate_loop.h               rg::FixedRateLoop: fixed-rate wall-clock loop around a bool try_step() (frozen tick resyncs the deadline - no catch-up burst), LoopStats; runs Session::start()'s ticks
      world_config.h                  rg::WorldConfig, rg::load_world_config() - see "World config" below
      terrain_view_streamer.h         rg::TerrainViewStreamer (R2.2 R8): render LOD follows a focus point - background reselect + build, key diff {added, removed}, adapter ordering contract in the header; see "Render LOD streaming (R8)" below
      road_classes.h                  rg::ClassLookup/RoadSegmentsFn, road_raster_params_for_level() (roads_plan.md R-2 level policy), road_class_lattice(), src_osm_tiles_for_chunk(), rasterize_chunk_road_classes() - OSM road classes (paved/unpaved) rasterised onto a render chunk's L0 cell-centre vertex lattice; render path only, no cell_surface_ids/grip - see "Visible roads (R-2)" below
      route_check.h                   rg::Route + load_route() ("rg.route/1", exception-free, optional "criteria" object -> RouteCriteria), apply_route_criteria() (RouteCheckParams defaults < route file < CLI, field by field), phys_tile_index()/count_seam_crossings() (255 m physics grid, origin 0.5), sample_l0_height() (bilinear on the L0 cell-centre lattice through any tile lookup), check_route() (length, 10 m-window max/p99 grade plus a steep_stretches list above grade_report_threshold, seam crossings, elevation range, NoData, corner radius = circle through the points +-corner_window_m (10 m) along the route at every 1 m sample (waypoint-density independent) plus a tight_corners list below corner_report_radius_m, start offset vs RouteCheckParams' R5 criteria), route_matches_world(), check_route_on_world() (the same over WorldTerrain::height_tile_shared) - used by tools/route_check and the [realdata] test
      credits.h                       (R5) rg::Credits: data/credits.json (rg.credits/1) loaded exception-free, entries (name, licence, required attribution text, where shown), `attribution_line()` (the one line shown on the boot splash, the loading screen and the credits screen), sections for the credits screen; test_credits checks every data source of the geo2map README table is covered (reads the sibling geo2map_engine README when the submodule pin predates it, SKIP when absent)
      settings.h                      (R5) rg::Settings: `settings_schema()` (graphics.window_mode/vsync/max_fps, camera.fov_deg, audio.master/engine/tyre_volume, map_data.store_dir; each with type, range, default, applies_live), versioned `rg.settings/1` JSON file (settings.json in the user dir), load report (missing/parse error/migrated/unknown + invalid keys), set/get/reset with validation + clamping, atomic save; only settings that really apply are in the schema
      spawn_presets.h                 (R5) rg::SpawnPresets (data/world/spawn_presets.json, rg.spawn_presets/1: id, label, session-frame x/y, yaw) + `build_spawn_choices()` (presets, "default_spawn", "last_position" from user_dir/last_drive.json, "address", "flat"; each with available/unavailable_reason; a preset outside the loaded store is greyed out)
      shell_flow.h                    (R5) rg::ShellFlow: the screen state machine (Boot, MainMenu, SpawnPicker, Loading, Drive, Pause, Settings, Credits, Quit) - events in (boot_finished, menu_item, spawn_picked, back, load_ready/failed/cancelled, pause_toggle, direct_start), a Transition out with ACTIONS (show_screen always last; load_world, unload_world, set_paused, reset_car, save_settings, quit); GDScript only carries the actions out
      vehicle_catalog.h               (R6) rg::VehicleCatalog: data/vehicles/catalog.json (rg.vehicle_catalog/1) - drivable cars (id, title, model, sim name, default paint/rim, chassis mass/box interim until physics_sim carries it), stats read from the vehicle/engine/gearbox files (power/torque from the engine curves or cycle block, mass, layout, driven wheels, gear count)
      vehicle_setup.h                 (R6) rg::SetupModel/VehicleSetup/materialise_setup: data/vehicles/setup_options.json (rg.vehicle_setup_options/1, 17 whitelisted options with ranges), RFC 7396 merge patch onto the vehicle file and the referenced engine/gearbox/tyre files, materialisation into a work dir and validation with ps::io::load_vehicle_json (loader message returned, an invalid setup cannot be saved); ScopedWorkDir
      garage_set.h                    (R6) rg::GarageSet (data/garage/garage_set.json, rg.garage_set/1: room, turntable, lights, camera areas) + the garage camera (smooth moves between overview/wheels/engine bay/rear, framing from the car's bounds), all Godot-free
      garage.h                        (R6) rg::Garage: selection + saved setup per vehicle (<user dir>/garage/), the edit session (working copy, validation on every change, save only while valid), prepare_drive() (materialises the saved setup into <work_root>/drive_<id>_<hash>, names everything a world load needs), cleanup()
      car_browser.h                   (R6c) rg::CarBrowser: the car browser model, Godot-free - BrowserCar (id, title, subtitle, body type, manufacturer, power/torque/mass/displacement, drive layout, paint/rim, model), GroupBy (body_type, drive_layout, power_band, manufacturer, none; computed category headers, power bands <100/100-199/200-399/400-699/700+ kW), SortKey (name, power, torque, mass, power_to_weight, displacement; ascending/descending; unknown values last, ties by title then id), BrowserFilter (drive layout / body type / power band, ORed within a facet, ANDed across), the grid (2-4 rows, column-major per category, sticky focus row, wrap, category jumps), scroll state and the virtualised `view(visible_columns, overscan)`, `thumbnail_key(model, paint, rim, version)`, group/sort/filter persisted through six hidden `browser.*` settings (SettingDef::shown = false)
      camera_math.h                   (R5) rg::DriveView (chase, bumper, cockpit, orbit, cinematic) with next_drive_view/toggle_cockpit_view, orbit state + `orbit_step`, bumper mount, `CinematicDirector` (roadside camera shots along `road_ahead`, falling back to the car's predicted path in the flat world and off-road): all camera math, no Godot type ; 2026-10-05: `chase_follow_offset` (the chase rig smooths the car-relative OFFSET, not the absolute position: zero lag at any speed), the cinematic director searches candidates against `ShotObstacles` (`shot_obstacles.h`; `building_footprints.h` = `BuildingFootprints`/`BuildingObstacles`, per-1 km-tile footprint prisms on a worker thread; see docs/buildings.md "Camera obstacles"), `ShotSource::ChaseFallback`
      road_ahead.h                    (R5) rg::trace_road_ahead(): the road polyline ahead of the car from the client road data (Session's road tiles), used by the cinematic director; `FrameSnapshot::road_ahead` is filled only while `Session::set_road_ahead_wanted(true)`
    src/
      session.cpp                     Session implementation - builds a ps::World by hand (flat: ground box + chassis + one vehicle; terrain: streamed G2mTerrainSource + chassis + one vehicle), never parses a scenario JSON; the terrain gate, start-up, priming, spawn rays, StreamingStatus atomics, relocation, make_session
      player_mode.cpp                 player_mode.h implementation
      walker.cpp                      walker.h implementation (uses ps::math::sincos/atan2 only)
      terrain_mode.cpp                terrain_mode.h implementation
      fixed_rate_loop.cpp             FixedRateLoop implementation
      world_config.cpp                load_world_config() implementation - strict, exception-free JSON validation (see "World config" below)
      terrain_view_streamer.cpp       TerrainViewStreamer implementation (one worker thread, one diff in flight, coalescing)
      road_classes.cpp                road_classes.h implementation
      route_check.cpp                 route_check.h implementation
      credits.cpp, settings.cpp, spawn_presets.cpp, shell_flow.cpp, camera_math.cpp, road_ahead.cpp, json_util.h   (R5) implementations of the six headers above (json_util.h: shared exception-free JSON helpers)
    CMakeLists.txt                    rg_core STATIC target
  godot_ext/
    src/
      register_types.h/.cpp           GDExtension entry point (rg_godot_library_init), registers RgSimulation + RgTerrainView
      rg_simulation.h/.cpp            RgSimulation : godot::Node - the one GDScript-facing class; owns one rg::Session and one rg::PlayerModeMachine; initialize_terrain() (R7) builds a terrain-mode Session on a worker thread, polled via get_init_status() - see "Godot bindings for Session terrain (R7)" and "Player modes and world switch (R9)" below; get_wheel_attachment_local/_steered/_is_front (static, from Session::vehicle_desc())/_compression/_spin_angle/_steer_angle (per-frame, from FrameSnapshot::wheels[i].state) feed vehicle_visual.gd - see "Vehicle visual (carvis)" below
      rg_terrain_view.h/.cpp          RgTerrainView : godot::Node3D (PLAN.md R2.1) - LOD terrain preview seam, streamed around a focus since R8; see "Terrain preview (R2.1)", "Render LOD streaming (R8)" and "Godot bindings for Session terrain (R7)" below
      rg_shell.h/.cpp                 (R5) RgShell : godot::Node - rg::ShellFlow + rg::Settings + rg::Credits + spawn choices as Dictionaries/Arrays for GDScript (initialize(data_dir, user_dir, world_config_path), boot_finished/direct_start/menu_item/spawn_picked/back/load_ready/load_failed/load_cancelled/pause_toggle -> Transition dictionary, get_*_menu_items, get_spawn_choices, get_settings_schema, get_setting/set_setting/save_settings, get_attribution_line, get_credits_sections)
      rg_garage.h/.cpp                (R6) RgGarage : godot::Node - rg::Garage as Dictionaries/Arrays (get_vehicles/get_vehicle with stats, get_options, set_option, save, prepare_drive, setup_camera/camera_go_to/camera_update/get_camera, get_work_file_count); RgShell gained vehicle_chosen/garage_drive and the open_garage/close_garage actions; RgSimulation::set_vehicle_overrides (chassis mass/box/z, engine map cache dir)
      rg_garage_browser.cpp           (R6c) the RgGarage.browser_* bindings (state load/get, group/sort choices, toggle/clear filter, filter options with counts, rows/layout/view, move_focus/set_focus, scroll, get_car with thumbnail_key, maxima, refresh); RgGarage.initialize takes an optional catalog path (--catalog)
      rg_camera_math.h/.cpp           (R5) RgCameraMath : godot::Object - static bindings of rg/camera_math.h (drive_view_names, next_drive_view, toggle_cockpit_view, orbit_default_state, orbit_step)
      rg_simulation_shell.cpp         (R5) second translation unit of RgSimulation: unload() (stop sim thread, streaming, modes, cinematic), set_paused/is_paused, set_spawn_override/clear_spawn_override, set_store_dir_override, set_road_ahead_wanted, get_chassis_session_pose, session_to_godot, get_bumper_camera, update_cinematic/reset_cinematic
      frame_convert.h                 rg_godot-namespaced wrapper around physics_sim's frame_convert_core.h (Vec3f/basis/pose -> godot::Vector3/Basis/Transform3D)
    CMakeLists.txt                    rg_godot SHARED target (the GDExtension DLL)
  game/                                Godot project (res:// root)
    project.godot                     rg_* input actions, gl_compatibility renderer
    rg_godot.gdextension              points at bin/librg_godot.dll/.so
    bin/                              rg_godot build output (librg_godot.dll + .pdb)
    scenes/
      main.tscn                       one-node stub (Node3D + main.gd) - the scene is built procedurally, see main.gd's own comment
    scripts/
      main.gd                         the one game scene (R9), built in _ready(): flat or real world, Drive or FreeCam, runtime mode/world switching, the loading flow, forwarding input to RgSimulation (process priority -2000); flags in its own header and in "Player modes and world switch (R9)" below. A `--terrain-preview` cmdline user-arg (after `--`) branches into the R2.1 static-terrain-plus-fly-camera scene instead - see "Terrain preview (R2.1)" below; in that scene the fly camera drives `RgTerrainView.update_focus` every frame (R8) except under `--screenshots`/`--stream-test`; `--bindings-test` (R7) runs a flat-mode-only RgSimulation/RgTerrainView bindings smoke check instead - see "Godot bindings for Session terrain (R7)" below. `_rg_data_path(relative)` (engine30, 2026-09-27) resolves against this repo's OWN `data/` (sibling of `external/`, same "one level up from res://" pattern as `_data_path`'s `external/physics_sim/data`) - both bindings-test and `_load_world`'s `vehicle_json` now point at `_rg_data_path("vehicles/car_sedan.json")` (racing_game's own file, see "data/vehicles, data/engines (engine30)" below), `surface_table_json` is racing_game's own `data/surfaces/surfaces.json` (via `_rg_data_path`-style `res://../data/`, own file since RACE-002, grass `lambda_mu` 0.63 since 2026-10-05, see docs/grass_grip.md), not physics_sim's shared table. `get_body_visuals()` exposes the `body_visuals.gd` node (drive_tour.gd's steering-proof poses.txt columns read `get_wheel_visual_steer_angle_rad` through it)
      terrain_stream_test.gd          `--terrain-preview --stream-test` (R8 headless check): after the initial upload, moves the LOD focus through 5 fixed steps from spawn, waits for each streamed diff to be fully applied, prints one line per step + `terrain stream test done: ...`, quits (180 s wall-clock timeout)
      camera_director.gd              (R9) owns the camera rigs; floating-origin rebase by the ACTIVE rig only (moves rig ROOTS), set_render_origin in the same frame, update_focus from the active camera (priority -1000)
      chase_rig.gd                    (R9) Drive rig: root + Camera3D, lagged chase of the chassis, look-around from the camera input group; replaces R0's chase_cam.gd. side_offset_m (carvis, 2026-09-27, default 0.0 - no behaviour change for any normal chase view): adds a lateral offset (chassis basis.y, ISO left) to the "behind" position, still looking at the centreline - a directly-behind camera can never show a front wheel (the body occludes it face-on); drive_tour.gd's steering_close shot is the only caller that sets it nonzero, for a 3/4-rear angle
      walker_rig.gd                   (R9c) OnFoot rig (`walker`): root + Camera3D, third person (orbit, wheel/PgUp/PgDn zoom, terrain clearance) or first person (Tab); owns the look yaw/pitch, `get_look_forward()` is what main.gd forwards with the walking input; reads RgSimulation.get_walker_state(). walker_visual.gd: placeholder capsule + facing nose (data/models untouched)
      free_rig.gd                     (R9) FreeCam rig: root (position, yaw) + Camera3D (pitch), driven only by the camera input group; place(pos, yaw, pitch)
      body_visuals.gd                 (R9; carvis) ChassisRoot (Node3D, transform = get_body_transform("chassis") every frame, priority 0) carries two children: ChassisBox (the red placeholder, CHASSIS_HALF_EXTENTS, unswapped) and VehicleVisual (vehicle_visual.gd, the real car_sedan.glb); the box is hidden once the model reports loaded, shown as fallback otherwise (vehicle_model_ok(), read by hud.gd). Also the flat world's ground box - its BoxMesh size/local position are authored in ISO order (x fwd, y left, z up, matching CHASSIS_HALF_EXTENTS), NOT native Godot order (bug found/fixed 2026-09-27: `_ground_anchor.transform` is `get_body_transform("ground")`'s ISO->Godot basis - an ISO-native mesh authored Y-up instead rendered as a ~2000-unit vertical wall, invisible edge-on from most angles and a "green plane sideways through the middle of the car" from others). on_session_ready() (called by main.gd right after a Session starts/re-starts) rebuilds VehicleVisual's wheel bindings for the new Session. get_wheel_visual_steer_angle_rad(i) (carvis steering-proof fix, 2026-09-27) forwards to VehicleVisual's own accessor of the same name
      vehicle_visual.gd               (carvis, 2026-09-26) near-copy port of physics_sim's adapters/godot/demo/scripts/vehicle_visual.gd (read-only reference): loads external/physics_sim/data/models/car_sedan/car_sedan.glb at runtime via GLTFDocument (data/models is owned by another session - read, never copied into this repo), binds susp_*/steer_*/wheel_* nodes by name from car_sedan.rig.json, aligns the model to the physics wheel attachment points, and drives suspension travel/steer angle/spin angle from RgSimulation's per-wheel accessors every frame - no client-side spin integration (unlike the reference) since rg::vehicle::WheelState already carries an integrated spin_angle. Keyed by a model-name string (vehicle_name) so a future car_hyper reuses it. rebuild() re-binds wheel nodes without reloading the .glb - called by body_visuals.gd's on_session_ready() after a runtime world switch (R7/R9) rebuilds the Session; `_process()` also retries `_bind_wheel_nodes()` on its own while `_wheel_nodes` is still empty (bug found/fixed 2026-09-27: the flat world's `on_session_ready()` call lands the same frame as `Session::start()`, before the sim thread's first tick has created the vehicle, so `get_vehicle_wheel_count()` read 0 at that instant and the binding silently stayed empty forever - poses.txt's `wheel_steer_visual_rad` stuck at 0.0 every frame was the symptom; the real world's own step-count-gated `on_session_ready()` call was already correct). get_wheel_visual_steer_angle_rad(wheel_index) (carvis steering-proof fix, 2026-09-27) reads back the steer_<corner> node's own last-applied local Y rotation (0.0 if unsteered/missing) - read by drive_tour.gd for poses.txt so a sign/axis mismatch between the physics steer angle and what actually got applied to the visual shows up as two numbers, not just a screenshot
      loading_overlay.gd              (R9) CanvasLayer shown while a real-world load runs (get_init_status() numbers) or after it failed. A roads.geom tile the deriver itself rejects ("deriver g2m.roads.geom: roads.geom: ...", deterministic) falls back to smoothed terrain like "required dependency absent" (RG_ROAD_SURFACE unavailable reason=deriver_rejected, world_terrain.cpp) instead of being retried until the 30 s start-up timeout (Koenigstein preset, tile utm32n/2/452/5428, 2026-10-06; geo2map fix queued)
      shell_ui.gd                     (R5) CanvasLayer (layer 30): boot splash, main menu, spawn picker, pause menu, settings (rows built from the schema: CheckButton/HSlider/OptionButton/LineEdit; Settings has a "Controls..." button, the screen itself is controls_ui.gd) and credits, all built from RgShell view-models; signals menu_item_chosen/spawn_chosen/back_requested/setting_changed/boot_finished; test hooks get_button(id)/button_ids()/get_setting_control(key)
      bumper_rig.gd, orbit_rig.gd, cinematic_rig.gd   (R5) the three new drive-view rigs (see "Shell (R5)"); all follow the PHYS-008 rig contract
      garage_scene.gd                 (R6) CanvasLayer (layer 20): the showroom in its OWN World3D (SubViewport): procedural room, turntable, softbox sky for reflections + a mirrored car twin on the glossy floor, studio lights, camera from RgGarage.get_camera (shifted so the car sits in the screen part the side panel leaves free); model loading (GLB) and paint/rim recolouring - shown through a TextureRect whose SubViewport is sized to physical pixels (rect size x content_scale_factor), so UI scaling does not blur the car
      ui_scale.gd                     root content_scale_factor = the OS display scale (Windows: monitor DPI / 96, e.g. 1.5 at 150 %), applied first thing in main.gd _ready; scales menus, garage, HUD and overlays, 3D stays at full resolution (stretch mode disabled); override --ui-scale=F or env RG_UI_SCALE (2026-10-06)
      garage_shots.gd, garage_test.gd (R6) `--garage-shots <dir>` (real window: the acceptance screenshots) and `--garage-test` (headless acceptance flow, tools/smoke_test.ps1 -Garage)
      car_browser_ui.gd, car_thumbnails.gd (R6c) the car browser screen (stats of the focused car on the left - metric only; horizontally scrolling grid of 2-4 rows with sticky category headers, tiles instantiated only for the window RgGarage.browser_get_view returns; filter/sort panel on F / gamepad Y / the on-screen button; arrows, D-pad, left stick, mouse hover/click/wheel) and the lazy preview renderer (own SubViewport + World3D studio, one picture per frame, PNG cache user://car_thumbs/<key>.png, none in a headless run); both are created only while a browser screen is up
      car_browser_test.gd, car_browser_shots.gd (R6c) `--car-browser-test [--car-browser-big]` (headless acceptance flow, tools/smoke_test.ps1 -CarBrowser / -CarBrowserBig) and `--car-browser-shots <dir> [--car-browser-big]` (real window; tools/car_browser_shots.ps1 runs both sets); `--catalog <path>` loads another vehicle catalog (tools/make_synthetic_catalog.ps1 writes the 200-car synthetic one)
      shell_flow_test.gd              (R5) `--shell-test`: the headless UI flow test (tools/smoke_test.ps1 -Shell), presses the real buttons
      camera_switch_test.gd           (R5) `--camera-test`: the PHYS-008 camera-switch test (tools/smoke_test.ps1 -Cameras)
      drive_smoke.gd                  (R9) `--drive --drive-smoke`: scripted drive, relocation (with --g2m-fetch-delay-ms), mode round trip, world round trip incl. a cancelled load; prints RG_DRIVE lines for tools/smoke_test.ps1 -Drive
      drive_tour.gd                   (R9; carvis) `--screenshots <dir>`, with or without `--drive` (a flat-world run skips the loading-overlay shot and runs the same tour unattended): loading overlay, spawn chase, a steering-lock close-up (front wheels turned, proves per-wheel steer_angle), driving and free-cam proof shots plus poses.txt. `02_steering_close.png` (steering-proof fix, 2026-09-27) frames a FRONT 3/4 angle (`CLOSE_DISTANCE_M` negative + `chase_rig.gd`'s `side_offset_m`, see that file's own comment) so both turned front wheels are visible - a rear-3/4 angle's near-side wheel is the rear-left one, hidden from the front wheel by the cabin. poses.txt's `_numbers()` also logs `wheel_steer_phys_rad`/`wheel_steer_visual_rad` per wheel (physics `get_wheel_steer_angle` vs. the visual node's own applied rotation, via `main.get_body_visuals()`)
      road_shots.gd                   (roads_plan.md R-4, owner request 2026-09-27) `--drive --road-shots <dir>`: relocates (`RgSimulation.relocate_vehicle`, session-local coords) to four `data/routes/home_r1_drive.json` arc-length points (`_point_at_s`, same Euclidean-arc-length convention as `rg::route_check`/drive_smoke.gd's `_relocate_target`) and screenshots each once `RgTerrainView.is_stream_idle()` (coordinator review fix, 2026-09-27 - was `is_fully_uploaded()`, which reads true even mid-build; see "Visible roads (R-2)" below) - `05_road_b8_junction.png` (s~150), `06_road_bridge.png` (s~2870, the route's one OSM-tagged bridge), `07_road_forest.png` (s~6000), `08_road_konigstein.png` (s~9800) - plus poses.txt (session position/speed/surface/chunk count/`resident_l0`/streamer stats per shot). See "Visible roads (R-2)" below for what the four shots actually show.
      fly_cam.gd                      free-fly camera script for `--terrain-preview` (PLAN.md R2.1): WASD + Space/E up + Ctrl/Q down, Shift x6 speed, right-mouse-button capture + look, Esc releases capture; no RgSimulation dependency (plain Camera3D script)
      gauge_logic.gd                  copied verbatim from physics_sim's demo (engine-neutral static math, no Godot Control dependency)
      tach_gauge.gd                   reused near-verbatim from physics_sim's demo (round tach/speed/gear/lamp gauge)
      hud.gd                          trimmed port of physics_sim's demo hud.gd (debug text HUD); R9: mode/world line, terrain stats line, "STREAMING TERRAIN... (n)" while the gate is frozen, one-line key help; carvis: a WARNING line while body_visuals.vehicle_model_ok() is false (console already gets vehicle_visual.gd's own push_warning either way). Steering-proof fix (2026-09-27): the driving-inputs-live control lines (steer/throttle/brake/handbrake/clutch) read back `RgSimulation.get_control(channel)` - the value actually applied to the sim this frame (main.gd's own `set_control` target, whether sourced from `_input_map` or `scripted_controls`) - instead of `_input_map.get_steer()` etc., which read 0 during a scripted-control run (drive_tour/drive_smoke) even while a nonzero value was being driven into the sim; `_input_map.get_ignition()`/`get_auto_shift()` are unaffected (no sim-side control channel for either)
      input_map.gd                    new plain-GDScript input node (not a C++ GDExtension class like physics_sim's PsInputMap) - keyboard+gamepad polling, larger-magnitude-wins merge; R9: separate driving and camera input groups plus mode/world/reset edge actions (keys in "Player modes and world switch (R9)" below)
      controls_ui.gd                  (R5b) the Controls screen (device list, action rows, capture/calibration overlay, live monitor, fixed keys); see "Controls (R5b)"
      controls_test.gd, controls_shots.gd  (R5b) `--controls-test`/`--controls-verify` headless checks (62 + 14) and `--controls-shots <dir>` screenshots
    shaders/
      terrain.gdshader                hypsometric terrain shader (PLAN.md R2.1): `ALBEDO = COLOR.rgb` (reads RgTerrainView's per-vertex RGBA8 colours); no world-space coordinates anywhere (object-space VERTEX/NORMAL and Godot's own per-fragment builtins only), so it survives the floating-origin rebase unmodified
  data/
    world/
      world_config.json               rg.world/1 (PLAN.md R2.0) instance - see "World config" below; also what `RgTerrainView::initialize`/`tools/lod_measure` open against
      regions.json                    g2m.regions/1 (geo2map_engine G1c I7): region "home" (Main-Taunus-Kreis,
                                       Hochtaunuskreis, Frankfurt-Höchst), halo_m 2000 - consumed by
                                       `g2m_tiler import ...regions.json#home ...` (S:\claude_code\geo2map_engine)
      spawn_presets.json              rg.spawn_presets/1 (R5): the spawn picker's presets (world_spawn, engelsruhe, b8_trunk, hornau, b8_north, koenigstein) in the session frame
    credits.json                      rg.credits/1 (R5): every data source/library with licence and required attribution text; shown on the boot splash, loading screen and credits screen
    controls/
      fixed_keys.json                 rg.fixed_keys/1 (R5b): the keys the controls screen lists but cannot rebind (Esc/P, F6, F7, F10, F9, middle mouse, Shift); described by hand, keep in step with the scripts
      actions.json                    rg.control_actions/1 (R5b): the ONE action schema (35 actions: id, label, group, kind, mode held/edge, range, modes); strictly validated
      defaults/{keyboard,mouse,gamepad}.json  rg.control_profile/1 (R5b): built-in per-class default bindings (equal to the pre-R5b ones)
    routes/
      home_r1_drive.json              rg.route/1 (R2.2 R5): the scripted-drive route, waypoints in the session frame + the spawn it starts from; checked by tools/route_check. Spawn (Engelsruhe, Frankfurt-Unterliederbach) -> B 8 -> the Koenigstein city-limit sign, 9.85 km, 2103 OSM-centreline waypoints densified to <= 5 m; its "criteria" (max_grade_pct 31, min_corner_radius_m 7) and every measured number are in its own "source" field
    vehicles/
      car_sedan.json                  physics_sim.vehicle/2 (engine30, 2026-09-27): racing_game-OWNED variant of external/physics_sim's data/vehicles/car_sedan.json (physics_sim is read-only from this repo, so this is a full copy, not an override/patch) - byte-identical except the engine component (now engines/n52b30_3l_na.json below, was physics_sim's i4_2l_na.json), the clutch capacity (300.0 -> 450.0 N*m kinetic, static_factor unchanged 1.15) and the wheel tyre / gearbox component refs (now `../../external/physics_sim/data/tyres/...`/`.../gearboxes/...` - refs resolve relative to THIS file's own directory, two levels shallower than physics_sim's original). This is the game's drivable car (main.gd's `_rg_data_path("vehicles/car_sedan.json")`, both flat and real worlds) - see this file's own "source" field for the full derivation and tests/unit/test_vehicle_data.cpp for the one number it is held to. Every test that pins a state hash (hash_check, test_session*, test_player_mode) still points explicitly at physics_sim's OWN car_sedan.json (external/physics_sim/data/vehicles/car_sedan.json, unaffected by this file's existence) - unchanged hashes
    engines/
      n52b30_3l_na.json                physics_sim.engine/1 (engine30, 2026-09-27): racing_game-owned NA 3.0 L I6 (BMW N52B30, 190 kW@6600rpm/300 N*m@2500-4000rpm variant) torque-map engine, referenced only by data/vehicles/car_sedan.json above - see this file's own "source" field for the full per-point citation (published vs. derived vs. assumed)
  cache/                               gitignored, LOCAL ONLY - never committed, never read by CI. `cache/g2m/home-r1/` is this machine's copy of the geo2map_engine source/derived store that `data/world/world_config.json`'s `${RG_G2M_HOME}` placeholder (default `S:\claude_code\geo2map_cache\home-r1`, `world_config.cpp`'s `kDefaultRgG2mHome`) resolves against - populated by pointing at (or copying from) an existing geo2map_engine store; nothing in this repo bakes it (`g2m_tiler.exe bake` run from here was denied, see the R2.1 task report). A checkout with no such store cannot open `rg::WorldTerrain` yet. `tools/lod_measure`'s own `RG_G2M_DERIVED` (default `out/g2m_derived/home-r1`, also gitignored) is a SEPARATE on-demand derived-tile cache this repo's own tools populate themselves and is unrelated to `cache/`.
  tests/
    unit/
      catch_main.cpp                  custom Catch2 v3 entry point (installs headless CRT handlers via physics_sim's always-built ps_headless_env)
      test_session.cpp                rg::Session tests: step stability, control-channel round-trip, snapshot/wheel-state sanity
      test_player_mode.cpp            rg::PlayerModeMachine/rules_for/unattended_controls (R9), Session::set_vehicle_control on a flat Session, reset to spawn
      test_walker.cpp                 (R9c, tags [on_foot], [determinism]) walker helpers (rect/footprint/enter range/spawn-spot order) and WalkerController on a plain ps::World: walk/run/diagonal, fall/land/jump, 12 cm kerb stepped / 25 cm blocked, wall stop + slide, 30 deg ramp climbed / 60 deg refused, footprint block, no-ground hold, ledge fall, 1 vs 4 workers state_hash
      test_session_walker.cpp         (R9c, tag [on_foot]) Session walker: spawn beside the driver door / passenger-side / roof fallback, the car blocks the walker and is undisturbed, get-in range + counters, despawn, input/run/jump, 1 vs 4 workers hash, relocation, real-time loop snapshot
      test_session_terrain.cpp        rg::Session terrain mode (R4, tag [session_terrain]); R9c adds the on-foot streaming cases (walker = point 0, parked car = point 1, never below terrain; worker-count determinism); R9 adds StartupProgress/cancel incl. make_session, relocation with a forced freeze, and a hidden `[.][realdata]` real-store cancel-at-several-points case; on a synthetic in-memory IHeightTileFetch (sine hills in 1/256 m, a NoData patch, 404 outside +-3 km, a switchable 503 storm): spawn ride height vs flat mode, 30 s scripted drive (0 falls/fill misses, state_hash identical at 1 vs 4 workers, fetch delay 0 vs 20 ms, and with a forced mid-drive freeze on Failed (503) keys lifted by retry_failed_tiles), spawn over NoData throws, coverage edge never freezes, 503 storm freezes the real-time loop with no step and resumes without a burst
      test_terrain_mode.cpp           rg/terrain_mode.h (R4, tag [terrain_mode]): physics heights (HeightTileSharedFetch -> ResidentHeightSet -> G2mTerrainSource::fill_tile) == render heights (same cache -> build_render_chunks L0 meshes) == (raw + height_offset)/256 for containers with non-zero offsets, one decode per tile; racing_game's container decode == geo2map's TransportHeightTileFetch; status pass-through; make_terrain_mode conversion
      test_fixed_rate_loop.cpp        rg::FixedRateLoop tests: no catch-up burst after a freeze, prompt stop(), stats
      test_terrain_view_streamer.cpp  rg::TerrainViewStreamer tests (R8) over a synthetic, optionally gated tile store: exact key diff, hole-free adds-then-removals at every step (plus a wrong-order negative control), no work while stationary, 1-vs-8 build-thread identical diffs, coalescing while busy, cancel+join on destruction
      test_route_check.cpp            rg::route_check tests on synthetic terrain (seam counting incl. negative indices/corners, grade window max/p99, NoData, corner radius, start/length criteria, sample_l0_height across tile borders, load_route errors, steep-stretch and tight-corner lists, corner radius on arcs/kinks and its density independence, "criteria" loading + apply_route_criteria precedence); one hidden `[.][realdata]` case runs the committed route on the real store under its own "criteria", SKIP unless RG_G2M_HOME is set
      test_credits.cpp, test_settings.cpp, test_spawn_presets.cpp, test_shell_flow.cpp, test_camera_math.cpp, test_road_ahead.cpp   (R5) one per new core header; test_player_mode.cpp gained the unload_world case, test_session.cpp the pause/road_ahead cases, test_world_config.cpp the store_dir override
      test_world_terrain.cpp          rg::WorldTerrain / build_static_view_from_lookup tests (PLAN.md R2.1) over a synthetic in-memory TileKey->HeightTile map - no TileStore/Server needed; chunk selection, session-local origin math, 1-vs-N-thread byte-identical output; fetch_height_tile_cached concurrency; decode_height_tile_container on synthetic containers (height_offset of both signs with NoData kept, int32 overflow / NoData-collision rejection, layer/key mismatch, truncation)
      test_road_classes.cpp           rg::road_classes tests (roads_plan.md R-2) over synthetic RoadSegment lists - no TileStore/Server/OSM-decode needed: the level policy's min_rank/min_half_width_mm against RoadStyle::default_style()'s own tertiary/primary ranks, road_class_lattice()/src_osm_tiles_for_chunk() against terrain_chunk.h's vertex-placement formula, a road segment appearing at L0 and being filtered out level-by-level (residential/tertiary/primary ranks), a null ClassLookup leaving the ClassWindow all-Unknown
      test_route_road_coverage.cpp    (roads_plan.md R-3) hidden `[.][realdata]` case, SKIP unless RG_G2M_HOME is set: point_is_road() classifies one absolute UTM point via WorldTerrain::road_segments + a 1x1 g2m::Lattice fed to rasterize_road_segments at the L0 level policy (every road, min_rank=0/min_half_width_mm=0); measured on the real home-r1 store against the committed data/routes/home_r1_drive.json (2103 waypoints): 100.00% road coverage (2103/2103, spawn included), forest-stretch control (84 points 30 m off-route at s=[5800,6200], the s~6000 m stretch roads_plan.md names) 0.00% road - see "Visible roads (R-2)" below
      test_vehicle_data.cpp           (engine30, 2026-09-27, tag [vehicle_data]) loads data/vehicles/car_sedan.json via ps::io::load_vehicle_json and asserts the ONLY number that data file is held to: the clutch's static torque capacity (capacity_nm * static_factor) >= 1.2x the engine's peak WOT torque (max of TorqueMapEngineDesc::wot_torque_nm_vs_rpm.y) - both read back from the loaded VehicleDesc's PowertrainDesc::components, nothing hand-copied from the JSON's own "source" commentary. No launch/stall/top-speed/wheelspin test (owner ruling: wheelspin is allowed and not to be gated or tested on)
      CMakeLists.txt                  rg_test_catch_main + rg_unit_tests targets, CTest registration
  tools/
    common.ps1                        shared PowerShell helpers (VS dev-shell entry, Godot exe lookup, cmake wrappers, submodule update) - dot-sourced by run.ps1/smoke_test.ps1/ci.ps1
    setup_dev_env.ps1                 -CheckOnly only (installs nothing - see its own header)
    run.ps1, run.cmd                  incremental build + launch Godot on game/ - opens the game SHELL (boot splash, main menu) by default (R5); `-Drive` skips it and starts the real world in Drive mode (`--drive`), `-Flat` the flat scene (`--flat`), `-VR` starts in the world too; pass `-- --terrain-preview` (see run.ps1's own pass-through-args comment) to launch the R2.1 terrain preview instead
    smoke_test.ps1                    headless Godot smoke test (build, ctest, headless run, assert no errors + sim thread ticking); `-TerrainPreview` runs the R2.1 headless check instead (asserts >= 150 chunks selected and a completed upload, see its own header comment); `-TerrainStream` (implies -TerrainPreview, R8) adds `--stream-test` and asserts `steps=5 diffs=5 ... missing_removals=0` - with the 0-ERROR-lines check this is the RID-leak check after streamed add/remove diffs; `-Drive` (R9) runs `--drive --drive-smoke` and asserts `RG_DRIVE ready`, falls=0 misses=0 on every RG_DRIVE numbers line, ticks > 0 and result=ok; `-DriveDelayMs N` adds `--g2m-fetch-delay-ms N` and asserts relocate_freezes >= 1 and advanced_after_relocate > 0; both SKIP (exit 0) without a geo2map store; R5: the plain run passes `--flat`, `-Shell` runs the headless UI flow test, `-Cameras` the PHYS-008 camera-switch test; R6: `-Garage` runs the headless garage acceptance flow (`garage_test.gd`)
    make_synthetic_catalog.ps1        (R6c) writes a synthetic vehicle catalog (the real base cars + generated presets, default 200 entries) for the car browser tests and screenshots; `car_browser_shots.ps1` takes the browser screenshots into out/car_browser_screens/ (needs a real window)
    ci.ps1                            Windows CI: debug + release legs (configure, build, ctest) + smoke_test - this repo's ci.ps1 has NO Linux leg (unlike physics_sim's tools/ci.ps1); R5 adds the smoke_test -Shell and -Cameras legs, R6 the -Garage leg
    lod_measure/
      main.cpp, CMakeLists.txt        lod_measure executable (PLAN.md R2.1): measures rg::WorldTerrain::build_static_view cold/warm wall time + chunk/vertex counts against the real data/world/world_config.json at max_distance_m in {6000, 12000, 20000} - see "Terrain preview (R2.1)" below for the measured table and chosen default
    hash_check/
      main.cpp, CMakeLists.txt        hash_check executable - the R0 acceptance check, see "Hash comparison acceptance check" below
    route_check/
      main.cpp, CMakeLists.txt        route_check executable (R2.2 R5, links rg_core only): `route_check [--world-config PATH] [--route PATH] [--min-length-m M] [--max-grade-pct P] [--min-seams N] [--min-corner-radius-m R]` (defaults data/world/world_config.json, data/routes/home_r1_drive.json; the four limits override the route file's "criteria", which override RouteCheckParams' defaults) - prints the effective criteria, then rg::check_route_on_world's report; exit 0 pass, 1 criterion failed, 2 load/usage error
```

## physics_sim submodule

`external/physics_sim` is a git submodule of `S:\claude_code\physics_sim`,
added as a local absolute `file://`-style path (`.gitmodules`'s `url`) per
decision D9. **Every command that clones or updates it must pass
`-c protocol.file.allow=always`**, or git >= 2.38 refuses it with
`fatal: transport 'file' not allowed` (confirmed empirically on git
2.54.0.windows.1) - `tools/common.ps1`'s `Update-PhysicsSimSubmodule`
always does this; a manual clone must do the same by hand:
```
git clone <racing_game url>
cd racing_game
git -c protocol.file.allow=always submodule update --init --recursive
```

`S:\claude_code\physics_sim` (the submodule's source repo) is READ-ONLY
from this repo and its tooling: nothing here ever writes into
`external/physics_sim`, runs physics_sim's own `tools/ci.ps1` or
`adapters/godot/smoke_test.ps1`, or builds inside physics_sim's own `out/`
directory. All builds that touch the submodule's CMake targets build under
`racing_game/out/build/<preset>` instead (`add_subdirectory(external/
physics_sim EXCLUDE_FROM_ALL)` in the top-level `CMakeLists.txt` - the
submodule contributes targets to racing_game's own build graph without
being racing_game's default build set).

## geo2map_engine submodule

`external/geo2map_engine` is a git submodule of `S:\claude_code\geo2map_engine`,
added the same way as `external/physics_sim` above (local absolute
`file://`-style URL, same `-c protocol.file.allow=always` requirement,
`tools/common.ps1`'s `Update-Submodules` covers both). Same READ-ONLY rule:
nothing here ever writes into `external/geo2map_engine` or runs its own
tooling/CI; `add_subdirectory(external/geo2map_engine EXCLUDE_FROM_ALL)` in
the top-level `CMakeLists.txt`, right after the physics_sim block.

Force-set before that `add_subdirectory` (matching geo2map_engine's own
`consumer-check` preset combination exactly): `G2M_BUILD_TESTS=OFF`,
`G2M_BUILD_APPS=OFF`, `G2M_BUILD_IMPORT=OFF` (skips the whole OSM/GeoTIFF
decoder stack and its FetchContent deps - libdeflate/zlib-ng/protozero/
libosmium), `G2M_BUILD_FUZZERS=OFF`. Also forces `CMAKE_C_COMPILER` to match
`CMAKE_CXX_COMPILER` on the clang-cl presets (geo2map_engine's own
`cmake/deps_core.cmake` calls `enable_language(C)` unconditionally for
zstd/SQLite; only its `deps_decoders.cmake`, skipped here via
`G2M_BUILD_IMPORT=OFF`, has its own such guard).

R2.2 G1-G9 added `g2m_phys` (built unconditionally, no `G2M_BUILD_*` gate)
and `g2m_ps_bridge` (`bridges/physics_sim/`, gated by its own
`if(TARGET ps_core)` check in geo2map_engine's top-level `CMakeLists.txt` -
satisfied here because `external/physics_sim`'s `add_subdirectory` above
runs first). `bridges/physics_sim/physics_sim.pin` records the exact
physics_sim commit the bridge is built/tested against on geo2map_engine's
own CI (its `bridge-release` leg); it must equal this repo's own
`external/physics_sim` submodule sha (`git submodule status`) - both are
`5c648a5` as of the R2.2 R1 submodule bump. A mismatch is a coordination
error between the two submodule pins, not something CMake itself checks.

Even with every `G2M_BUILD_*` option OFF, `deps_core.cmake` still
`FetchContent_Declare`s `g2m_zstd`/`g2m_sqlite`/`g2m_json` (needed by
`g2m_layers`/`g2m_server`/`g2m_client` themselves). `RG_G2M_DEPS_DIR` (cache
`PATH`, default `S:/claude_code/geo2map_engine/out/build/release/_deps` on
Windows, `$ENV{HOME}/build/geo2map_engine/linux-release/_deps` on Linux)
points `FETCHCONTENT_SOURCE_DIR_<NAME>` at that directory's own
`<name>-src` subdirectories when they exist, so configuring here never
reaches the network for these - it reuses geo2map_engine's own already-built
release/`linux-release` preset's populated FetchContent sources. Override
with `-DRG_G2M_DEPS_DIR=...` if that path differs on another machine; a
`message(WARNING ...)` fires if the directory does not exist.

Bumped to `1e18588` (G2.5a-grip G-a/G-b/G-c, 2026-09-27) to bring in road
paint + the owner-rule raster (`g2m::layer::osm_roads`'s `RoadValueLut`,
`paint_road_segments`/`paint_road_blocks`/`rasterize_road_blocks`,
`raster_params_for_render_level`), roads riding on height residency plus
`fill_physics_surfaces` (`g2m::phys`), and the bridge's road-mode
`G2mTerrainSource` ctor - consumed starting with R-b below. `g2m::phys::
FetchResult` gained a third `roads` member; every 2-arg brace-init in this
repo (`core/src/terrain_mode.cpp`, `tests/unit/test_session_reinit.cpp`,
`tests/unit/test_session_terrain.cpp`) now passes an explicit `nullptr` for
it - no behaviour change, `HeightTileSharedFetch::fetch` still never
produces roads (R-c wires that up).

## World config

`data/world/world_config.json` (schema `rg.world/1`, PLAN.md R2.0) names one
drivable world region: `format` (must be exactly `"rg.world/1"`), `region`,
`session_origin_utm` (`{zone, e0, n0}` - zone `1..60`, `g2m::geo::UtmZone`'s
own range), `source_store`/`derived_store` (`{dir, scope|name, read_only}` -
geo2map_engine store locations), `spawn` (`{e, n, yaw_deg}` - yaw 0 = east, counter-clockwise, the vehicle's +x axis rotated about +Z), `surface_map`/
`palette` (paths).

`rg::WorldConfig` + `rg::load_world_config(path, err)`
(`core/include/rg/world_config.h` + `core/src/world_config.cpp`) parse and
strictly validate this file - non-throwing (`nlohmann::json::parse(...,
allow_exceptions=false)`, TOOL-019 exception-free contract), returns
`std::optional<WorldConfig>`, `*err` set to a human-readable message on
`nullopt`.

Two distinct path-resolution rules, per field:
- `source_store.dir`/`derived_store.dir`: `${VAR}` placeholder expansion
  only, then used as-is (relative-to-CWD if not already absolute) - these
  name a location outside the repo (a geo2map_engine cache/store), never
  resolved against a repo root.
- `surface_map`/`palette`: resolved against the REPO ROOT, defined as the
  nearest ancestor of the CONFIG FILE'S OWN directory that contains a
  top-level `CMakeLists.txt` (`find_repo_root`) - not the process's CWD, and
  not necessarily racing_game's own root if the config file lives in a
  different checkout. Neither file is required to exist, only the path is
  resolved. A config file with no `CMakeLists.txt` anywhere above it (e.g.
  one written to a bare OS temp directory) fails this resolution - test
  fixtures needing a `surface_map`/`palette` pass live under
  `out/test_tmp/` inside this repo, not the OS temp dir, for exactly this
  reason.

`${VAR}` placeholder expansion applies only to the two `dir` fields above.
`${RG_G2M_HOME}` is special-cased: if the env var is unset it defaults to
`S:\claude_code\geo2map_cache\home-r1` (`kDefaultRgG2mHome`,
`world_config.cpp`) instead of erroring; every other unset/unknown
`${NAME}` is a validation error.

## Terrain preview (R2.1)

First visible, static-LOD terrain plus a fly camera - no vehicle, no
`ps::World` (PLAN.md R2.1).

`rg::WorldTerrain` (`core/include/rg/world_terrain.h` + `core/src/
world_terrain.cpp`): the engine-neutral seam onto geo2map_engine's offline
in-process server/mesh stack. `WorldTerrain::open(config, err)` opens
`config.source_store`/`derived_store`, builds one local geo2map_engine
release (`g2m::builtin_local_release_params` + `g2m::make_local_release` -
the same wiring `g2m_tiler bake` and geo2map_engine's own golden tests use)
and adds it to an offline `g2m::Server` (`offline = true`: its only
`IUpstream` is a `g2m::LocalSourceUpstream`, so it never touches the
network). `build_static_view(cam_x, cam_y, out)` (session-local metres)
runs `g2m::mesh::select_chunks` against `lod_params()` and builds every
selected chunk (`gather_window` + `build_chunk`) into `out` as
`rg::RenderChunk`s, threaded (`build_static_view_from_lookup`'s
`thread_count` param, each worker writing its own assigned index) so the
result is byte-identical for any thread count. `height_tile()` is a
mutex-guarded, cached blocking fetch+decode (per-tile cache, decode work
outside the lock) - `fetch_stats()` reports `cache_hits`/`server_ok`/
`server_miss` since `open()`. The decode is the free function
`decode_height_tile_container(container, expect_layer, expect_key)`
(mirrors geo2map's `TransportHeightTileFetch`: `parse_container`, header
layer/key must match, heap `HeightTile`, the header's `height_offset` added
to every non-NoData sample, `Status::Internal` if a sum would reach NoData
or overflow int32). `RenderChunk` (`rg/terrain_render.h`):
`mesh` (geo2map_engine's own built `TerrainChunkMesh` - positions/normals/
indices, Z-up, local to `mesh.origin`), `origin_session[3]` (`mesh.origin -
(E0, N0, 0)`, still session-local metres, double precision), `rgba` (one
packed RGBA8 colour per vertex - a hypsometric height/slope ramp, with
`LandClass::PavedRoad`/`UnpavedRoad` painted over it where `mesh.land_class`
says so - see "Visible roads (R-2)" below - every other class still falls
through to the height/slope ramp, see `world_terrain.cpp`'s
`chunk_vertex_colors()`).

`RgTerrainView : godot::Node3D` (`godot_ext/src/rg_terrain_view.h/.cpp`) -
the Godot-facing seam, mirroring `RgSimulation`'s "all engine-neutral logic
stays in rg_core, this file only converts" shape: `initialize(world_config_
absolute_path)` opens a `WorldTerrain` and caches its spawn point
session-local (`get_spawn_x()`/`get_spawn_y()`, `WorldConfig::spawn.e/n`
minus `session_origin_utm.e0/n0`, so a GDScript caller never has to
re-parse `world_config.json` itself); `load_preview(spawn_x, spawn_y)` runs
one `build_static_view` and queues every chunk for per-frame upload
(`_process`, budgeted `upload_budget_per_frame_` chunks/frame so hundreds
of meshes never stall one frame); `set_render_origin(session_origin)`
re-transforms every already-uploaded instance for the floating-origin
rebase without rebuilding meshes; `set_material(material_rid)` attaches a
`ShaderMaterial`'s RID (`game/shaders/terrain.gdshader`) to every uploaded
chunk's mesh surface - required because Godot's default material does not
read a mesh's own vertex COLOR array as albedo, so without this call the
per-vertex hypsometric colours are built but never actually visible.
Uploads via the low-level `RenderingServer` API directly (`mesh_create`/
`mesh_add_surface_from_arrays`/`instance_create2`/`instance_set_transform`),
one mesh + one instance RID per chunk, tracked and freed in
`free_all_uploaded()` (destructor and start of `load_preview()`, so a
second preview load or a process exit leaks zero RIDs).
`upload_one_chunk()` skips building/uploading the surface array entirely
whenever a chunk's vertex or index count is zero (real data at the
imported region's coverage edge can have vertices but empty `indices` -
assigning an empty `PackedInt32Array` to `ARRAY_INDEX` still sets Godot's
own `ARRAY_FORMAT_INDEX` bit, which then fails `RenderingServer`'s surface
validation) while STILL always creating the mesh/instance RID pair, so the
1:1 `chunks_[i]`/`instance_rids_[i]` indexing invariant `set_render_origin`
relies on is never desynced by a coverage-edge chunk.

`game/scripts/main.gd`'s `--terrain-preview` cmdline user-arg (`OS.
get_cmdline_user_args()`, everything after Godot's own `--`) branches into
`_build_terrain_preview_scene()` instead of the R0 drivable scene: builds
an `RgTerrainView`, initializes it against `data/world/world_config.json`,
builds the `terrain.gdshader` `ShaderMaterial`, loads the preview at the
config's own spawn point, sets the render origin, adds a
`DirectionalLight3D` + `WorldEnvironment` (procedural sky) and a `Camera3D`
named "FlyCam" running `fly_cam.gd`. Prints `"terrain preview selected:
chunks=N vertices=N build_ms=F"` once chunk selection finishes and
`"terrain preview loaded: chunks=N vertices=N upload_ms=F"` once every
chunk has uploaded - `tools/smoke_test.ps1 -TerrainPreview` greps both.

`tools/lod_measure` (executable, links `rg_core` only): sweeps
`max_distance_m` in `{6000, 12000, 20000}` (`max_level` fixed at 6) against
the real committed `data/world/world_config.json`, running
`build_static_view` twice per distance (cold: this process's first touch
of each tile; warm: same `WorldTerrain` instance, tiles already cached).
Measured at the spawn in `data/world/world_config.json` (Engelsruhe, session
(2767.79, -7577.48); re-measured 2026-09-26 after the spawn moved there from
(-1500, 500)): Windows, `release`, one run against an empty `RG_G2M_DERIVED`
(a fresh directory, so every tile is a first touch), against the real
home-r1 store. (The `RG_G2M_DERIVED = ...` line lod_measure prints is always
the default path, even when the env var overrides it.)

| max_distance_m | chunks | total_verts | cold_ms | warm_ms |
|---|---|---|---|---|
| 6000  | 377 | 1,690,845 | 85,233 | 24.07 |
| 12000 | 416 | 1,865,760 | 61,016 | 27.64 |
| 20000 | 474 | 2,125,890 | 80,063 | 32.44 |

At the previous spawn (-1500, 500), three clean runs gave
386/433/488 chunks, 1.73M/1.94M/2.19M vertices, warm 23.5/28.7/32.7 ms.

Re-measured 2026-09-27 after roads_plan.md R-2's road-class rasterisation
(`rasterize_chunk_road_classes`, wired into `build_render_chunks` - see
"Visible roads (R-2)" below): warm_ms at the default 20000 m rose from the
32.44 ms above to **36.89 ms** (+13.7%, one run, same empty-`RG_G2M_DERIVED`/
real-home-r1-store method) - within R-3's own +50% budget (48.66 ms). Full
sweep: 6000 m 29.38 ms, 12000 m 32.49 ms, 20000 m 36.89 ms.

Chosen default: **20000 m** (`data/world/world_config.json`'s own
`lod.max_distance_m`) - the R2.1 goal is "visible terrain to 16-20 km";
warm cost at 20000 m (~33 ms) is barely above 6000 m's (~23 ms) and both
are comfortably under PLAN.md R2.1's 3000 ms target, so the choice is
driven by coverage instead - 20000 m's 474 chunks/2.13M vertices stay
close to the "~400 chunk" naive-worst-case budget rather than blowing it
up. The COLD numbers above are NOT representative of a baked-store first
load - `cold` means "this process's on-demand Cache->Derive->Upstream path
touching each tile for the first time", 61-116 s regardless of distance;
`g2m_tiler bake` into `cache/g2m/home-r1/derived` was denied by the
session's own permission classifier and was not retried, so only the warm
number is representative of a real pre-baked/pre-derived run.

## Render LOD streaming (R8)

`RenderChunk::key` (`rg/terrain_render.h`) is the chunk's g2m `ChunkKey`
(always equal to `mesh.key`). `build_static_view_from_lookup` is now
`select_view_keys` (selection only) + `build_render_chunks` (build a given
key list, index-parallel, optional cancel flag), both public in
`rg/world_terrain.h`; `TerrainViewSource` bundles LOD params, session origin
and a thread-safe tile lookup (`WorldTerrain::view_source()` returns one over
itself - destroy the streamer before the `WorldTerrain`).

`rg::TerrainViewStreamer` (`rg/terrain_view_streamer.h/.cpp`, Godot-free):
`build_initial(x, y, out)` (synchronous first selection, optional),
`update_focus(x, y)` (non-blocking; starts one background selection once the
focus is more than `Options::reselect_distance_m` = 128 m from the last
selection's focus), `poll(diff)` (non-blocking), `commit(serial)`.
`TerrainViewDiff{serial, focus, added (built RenderChunks), removed (keys)}`,
both sorted by key; only keys not already resident are built. Adapter
contract: upload every add (any per-frame budget), THEN free the removals and
`commit()` - never the other way round, so the view never drops below one
complete selection. One diff in flight: no new selection while a diff is
Ready/Applying; `commit()` starts one follow-up for the latest focus (moves
while busy coalesce). Phases `Idle/Building/Ready/Applying`; `stats()`.
Destructor cancels (checked before each chunk) and joins the worker.

Godot side (`RgTerrainView`): `load_preview` seeds a streamer via
`build_initial` (same chunk set/order as `build_static_view`) and uploads it
with the 8-chunks/frame count budget (a loading phase). After that,
`update_focus(session_x, session_y)` forwards to the streamer and `_process`
runs `apply_diff`: poll -> append adds to the upload queue -> upload under
`upload_budget_ms` (default 0.8 ms, steady clock, at least one operation per
frame) -> once every add is up, free removed chunks' RIDs under the same
budget (possibly over several frames) -> `commit`. `chunks_`/`mesh_rids_`/
`instance_rids_` stay index-aligned (append; swap-remove all three) with a
`std::map<ChunkKey, size_t>` key->index map. `is_fully_uploaded()` is also
false while a diff is half-applied; `is_stream_idle()`; `godot_to_session()`
(render-frame position -> session XY for the focus); `get_stream_stats()`
(pending_adds/removals, last_diff_added/removed, upload_ms_this_frame,
last_remove_ms, diffs_applied, diff_errors, resident_chunks,
selections_started, last_build_ms, streamer_phase). Not wired into drive
mode (R9). `tools/smoke_test.ps1 -TerrainStream` (debug DLL, real home-r1
store): 5 steps, 708 added / 708 removed, 76-226 adds per diff, background
build 108-293 ms, streaming frames 1.6-2.0 ms max (one op may overshoot the
0.8 ms budget), removals 4-9 ms per diff spread over frames.

## Visible roads (R-2)

`rg/road_classes.h/.cpp` (roads_plan.md R-2, owner request 2026-09-27):
OSM road classes (paved/unpaved) painted onto each render chunk's own L0
cell-centre vertex lattice, using geo2map_engine's `g2m::extract_road_segments`
and shared owner-block rasterisation. Road grip is now wired in Session; see the
G2.5a-grip update below.

`ClassLookup{fn, ctx}` (mirrors `g2m::mesh::TileLookup`'s bare
function-pointer-plus-ctx shape) is a field of `TerrainViewSource` and a
trailing parameter of `build_render_chunks`/`build_static_view_from_lookup`;
a default-constructed (null `fn`) one leaves every call byte-identical to
before this field existed - `WorldTerrain::view_source()`/`build_static_view()`
populate a real one (`&world_terrain_road_class_lookup`, an anonymous-
namespace free function in `world_terrain.cpp` mirroring `world_terrain_lookup`)
so both the streaming (drive) and static (preview) render paths pick up roads;
`terrain_view_streamer.cpp`'s two `build_render_chunks` call sites forward
`source_.class_lookup`.

`WorldTerrain::road_segments(TileKey)` (level-2 `g2m.src.osm` tiles, 1024 m +
128 m halo): blocking fetch (same `manifest_rid_`/`transport_` the height-tile
path uses) + `g2m::parse_container`/`decode_body`/`decode_src_osm` +
`g2m::extract_road_segments` against `g2m::RoadStyle::default_style()`,
cached (`osm_cache_mutex_`/`osm_cache_`, same double-checked-insert scheme as
`height_cache_` - a failure is reported to stderr and NOT cached, so it can be
retried; "no roads" is a valid, non-fatal result). `OsmFetchStats{ok, fail,
cache_hits}` (`osm_fetch_stats()`, coordinator review follow-up, 2026-09-27 -
mirrors the pre-existing `FetchStats`/`fetch_stats()` height-tile diagnostic)
counts every call outcome across every calling thread (relaxed atomics,
counters only) - added to directly measure `road_segments()`'s own
fetch-failure rate under the real multi-threaded render path instead of
guessing at it; see the R-4 finding below for the numbers this produced (a
clean `fail=0` in every run - the concurrency/fetch-failure hypothesis first
suspected for the missing-road bug was tested directly and NOT confirmed).

rasterize_chunk_road_classes uses g2m::rasterize_road_blocks on the chunk
lattice, painting each aligned L2 block from only its owner tile.
road_raster_params_for_level forwards to the shared geo2map policy. Level policy
(ranks looked up from `RoadStyle::default_style()` itself, never hard-coded):
L0-L1 every road (`min_rank=0`), L2 tertiary and above, L3 primary and above
plus a `0.75 * spacing` half-width floor, L>=4 `std::nullopt` (no road
classes at all - both "disabled" and "no roads found" leave the chunk's
`ClassWindow` all-`Unknown`, which `build_chunk` already treats as "no class
data").

Render colours resolve road classes through the configured RoadSurfaceMap names:
asphalt is grey (0.30, 0.30, 0.32), dirt is brown (0.36, 0.28, 0.16). Other
names and off-road classes use the height/slope ramp.

**R-3 acceptance measurement** (`test_route_road_coverage.cpp`, real home-r1
store, 2026-09-27): the committed `data/routes/home_r1_drive.json` (2103
waypoints) classifies **100.00% road at L0** (2103/2103, spawn included); an
84-point control sampled 30 m off-route across the forest stretch at
s=[5800, 6200] classifies **0.00% road** - both comfortably inside
roads_plan.md's >= 99% / < 10% bounds. `tools/lod_measure`'s warm_ms at the
default 20000 m rose from 32.44 ms to 36.89 ms (+13.7%) with this rasterisation
now running on every chunk build - see "Terrain preview (R2.1)" above for the
full sweep and the +50% budget check.

`game/scripts/road_shots.gd` (`--drive --road-shots <dir>`) gates every
shot on `RgTerrainView.is_stream_idle()` (upload queue drained AND the
streamer idle, i.e. the post-relocate diff applied), not
`is_fully_uploaded()` alone, and logs `get_stream_stats()` into
`poses.txt`. `tests/unit/test_route_road_render_path.cpp`
(`[.][realdata][terrain]`) drives the real `select_view_keys` +
`build_render_chunks` path through road_shots.gd's four relocate targets.
`WorldTerrain::osm_fetch_stats()` (`OsmFetchStats{ok, fail, cache_hits}`)
counts `g2m.src.osm` fetches. Findings and rationale: vault G2M-009.

## Session terrain mode (R4)

`SessionConfig::terrain` (`std::optional<TerrainModeConfig>`) switches
`rg::Session` from the flat ground box to streamed geo2map terrain; unset,
Session builds exactly what it built before (flat-mode `step()` is still
one plain `World::step()`, so `hash_check` is unchanged).

Per session (`session.cpp`'s `Session::Terrain`): `g2m::phys::
PhysicsTileGrid` (the config's `SessionFrame`, 256 samples), a shared
`ResidentHeightSet`, a `HeightTileLoader` (`physics.loader_workers`) over
`TerrainModeConfig::fetch`, a `PhysicsTerrainStreamer` (default
`StreamerConfig`: gate r_tm+1, prefetch r_tm+2, 3 s look-ahead) and a
`g2m::ps_bridge::G2mTerrainSource` installed via `World::
set_terrain_source(source, make_terrain_config(physics.radius_m,
physics.max_tile_fills_per_tick))` (pool 49 at r = 400 m, one interest
point).

Tick attempt (`step_once`, used by `try_step()`, `step()` and the loop):
`physics_interest_points()` (a list; the chassis pose/velocity as id 0 and,
R9b, only while a drone-follow target is set and found, that vehicle's
pose/velocity as id 1 - the TileManager pool is doubled in `setup_terrain`
for it, 98 tiles at r = 400 m) -> `streamer.update(points)` +
`World::set_terrain_interest_point` per point -> gate not ready: return
false with the World untouched (StreamingStatus frozen/frozen_attempts/
freeze_count) -> else drive script (or, from the loop, the set_control
atomics) -> `World::step()` -> fall detector + StreamingStatus. `step()`
blocks (1 ms retries, `startup_timeout_s` hard error) until the tick runs.
`retry_failed_tiles()` asks the next gate check to call
`streamer.retry_failed()`.

Construction (terrain): validate (fetch, radius, fills, timeout, surface
name) -> block until the gate around the spawn is ready (Failed keys
retried; `startup_timeout_s` -> `std::runtime_error`) -> K priming ticks
with no vehicle, K = ceil(side^2/F)+1, side = 2*ceil(r/255)+1 (26 at the
defaults; `TerrainModeConfig::prime_ticks` overrides) -> assert resident
tiles >= side^2, 0 starved, 0 fill misses -> five downward rays (centre +
the yaw-rotated chassis corners, from z = 3000 over 6000 m; a miss ->
`std::runtime_error` "spawn over NoData") -> chassis at max hit +
`chassis_z_m` + `physics.spawn_clearance_m`, yaw about +Z -> vehicle.
`spawn_tick()` = K; `drive_tick()` = ticks since.

Real game wiring: `make_terrain_mode(world_config, world_terrain)` -
spawn converted into the session frame, fetch = `HeightTileSharedFetch`
over `WorldTerrain::height_tile_shared` (the render path's own decoded-tile
cache; the physics and render paths share one `HeightTile` object per key).

`Session::world_terrain()` (R2.2 R7) returns the `shared_ptr<WorldTerrain>`
a terrain-mode Session was built from (null in flat mode, or when the
`TerrainModeConfig` was hand-assembled rather than built via
`make_terrain_mode(world_config, world_terrain)`) - `RgTerrainView::
initialize_shared` (below) reuses it instead of opening/decoding a second
copy.

## Road grip update (G2.5a-grip R-b/R-c, 2026-10-01)

The shipped physics.road_surfaces block enables asphalt for paved roads, dirt
for unpaved roads and grass elsewhere. RoadSurfaceMap shares the configured
names between render and physics. Render colours recognise asphalt (grey) and
dirt (brown); other names and off-road classes use the height/slope ramp.

rasterize_chunk_road_classes now calls g2m::rasterize_road_blocks: each aligned
L2 OSM block uses only its owner tile's segments, avoiding neighbour-order
dependence at equal-rank overlaps. road_raster_params_for_level forwards to the
shared g2m::raster_params_for_render_level policy.

In R4 terrain mode, make_terrain_mode supplies both height_tile_shared and
road_segments_shared through HeightTileSharedFetch. Each L0 height key fetches
roads from its L2 ancestor. The loader requires roads in this mode: a failed
road input is retried/failed with the residency entry and freezes the gate. A
successful empty road list is valid. Session::setup_terrain resolves the
configured names through its SurfaceTable into a RoadValueLut and uses the
road-mode G2mTerrainSource constructor. Missing road-layer/provider support,
unknown names and ids >= 255 are startup errors. Disabling road_surfaces retains
the uniform terrain_surface path and its existing synthetic R4 hash.

StreamingStatus and RgSimulation.get_streaming_status expose road_surfaces,
osm_ok and osm_fail. HUD shows grip: roads (with the OSM counters) or grip:
uniform <name>. RG_DRIVE status includes wheel-zero surface=; spawn_check uses
surface0=. The -Drive smoke requires asphalt at the road spawn.

The real-data test_route_grip.cpp ([.][realdata][grip], requires RG_G2M_HOME)
recorded 2103/2103 route samples asphalt and 84/84 forest-control samples grass.
Session checks recorded all four wheels asphalt at 7/7 route positions and at
least three wheels grass at 5/5 off-road positions. Rough terrain can leave one
wheel without contact. These results do not establish bridge or 20-junction
wheel-force acceptance.

## Godot bindings for Session terrain (R7)

Binding layer only (owner rule: no game logic in `godot_ext` - a future
UE5 port reuses `rg_core` unchanged); the mode framework, `--drive`, HUD and
the flat<->terrain world switch UI came with R9 (see "Player modes and world
switch (R9)" below). R7 itself added only a `--bindings-test` smoke path to
`main.gd` (below).

`RgSimulation` (`godot_ext/src/rg_simulation.h/.cpp`) additions:
- `initialize_terrain(world_config_path, vehicle_json_path,
  surface_table_path) -> bool` - builds a terrain-mode `rg::Session` via
  `make_terrain_mode(...)`. NON-BLOCKING: the whole `Session` construction
  (which can block up to `startup_timeout_s`, 30 s, on real data - see
  "Session terrain mode (R4)" above) runs on a worker thread; returns
  `true` once the thread has been STARTED, not once the Session is ready.
- `get_init_status() -> Dictionary` - `{state: "idle"|"loading"|"ready"|
  "error", message: String, stage: "opening"|"waiting_for_gate"|"priming"|
  "spawning"|"done", resident_l0, missing_required, inflight, failed,
  prime_done, prime_total}`, polled every frame by GDScript for the loading
  overlay. Since R9 the figures are live while `"loading"`: they read the
  in-flight start-up's `rg::StartupProgress` atomics (`SessionConfig::
  startup`); once `"ready"` they read `session_->streaming_status()`.
- `start()` is only valid once `get_init_status().state == "ready"` (it is
  a no-op - `session_` still null - before that).
- `is_terrain_mode() -> bool`, `get_streaming_status() -> Dictionary`
  (every `rg::StreamingStatus` field, snake_case keys),
  `get_render_origin_session() -> Vector3`, `retry_failed_tiles()`.
- Both `initialize(vehicle_json_path, surface_table_path)` (flat) and
  `initialize_terrain(...)` (terrain) are RE-ENTRANT: calling either again
  on an `RgSimulation` that already holds a Session (loading or running)
  tears the old one down first (`teardown_current()`) and builds the new
  one - the runtime flat<->terrain world switch. No restart, no leak.

Threading contract (`initialize_terrain`'s worker thread,
`run_terrain_init_worker`): an atomic `InitPhase{Idle,Loading,Ready,Error}`
state machine, release-stored by the worker after it builds the `Session`
via `rg::make_session` (nullptr + message on any failure) into a staged
`pending_session_`,
acquire-loaded by the main thread before adopting it into `session_`.
`reap_init_thread(bool wait)` is the one join point:
  - `wait=false` (`get_init_status()`, `start()`, every per-frame poll
    path): peeks the atomic first and returns WITHOUT joining while still
    `Loading` - never blocks the render thread.
  - `wait=true` (`stop()`, `teardown_current()`, the destructor): always
    `std::thread::join()`s. The thread is NEVER detached.
Cancel-then-join (R9, replaces R7's wait): a re-init (`teardown_current()`)
or the destructor that lands while an `initialize_terrain` is still loading
first sets that start-up's `StartupProgress::cancel` (`cancel_init()`), then
joins. `Session`'s constructor polls the flag in its gate and priming loops
and throws `SessionCancelled`, which `rg::make_session` turns into nullptr.
The flag is also checked by the worker between its steps (after the config
load and after `WorldTerrain::open`) and by `setup_terrain` around the
TileManager pool construction (slow in a debug build); only those steps
themselves are uninterruptible. `stop()` still waits without cancelling.

No exception crosses into, or is caught in, `godot_ext`: godot-cpp's default
`GODOTCPP_DISABLE_EXCEPTIONS` puts `-D_HAS_EXCEPTIONS=0` on `rg_godot`, which
makes MSVC STL's `std::exception` name `stdext::exception` in that TU - a
different type from what `rg_core` throws, so a `catch (const
std::exception&)` there never matches and the exception ends in
`std::terminate` (exit 0xC0000409 - the R9 cancel-during-load crash). Every
`Session` construction therefore goes through `rg::make_session`
(`session.h`), which catches inside `rg_core`.

`RgTerrainView` (`godot_ext/src/rg_terrain_view.h/.cpp`) additions:
- `initialize_shared(sim: RgSimulation) -> bool` - reuses the Session's
  `WorldTerrain` via a friend-class accessor (`RgSimulation::
  shared_world_terrain()`, private - not Variant-bound) instead of opening/
  decoding a second copy; `terrain_` is now a `shared_ptr<WorldTerrain>` (was
  `unique_ptr`) so it can hold either a privately-owned copy (`initialize`)
  or a shared one (`initialize_shared`).
- `release()` - frees every uploaded chunk and drops the shared terrain (for
  a world switch); `initialize`/`initialize_shared` may be called again
  after it.
- `update_focus(x, y)` is unchanged - GDScript passes the ACTIVE camera's
  position (not necessarily the car's, per the R9 mode framework), no C++
  change needed for that.

`--bindings-test` (`main.gd`, wired into `tools/smoke_test.ps1 -BindingsTest`
and a second `tools/ci.ps1` smoke leg): a flat-mode-only smoke check (no
`RG_G2M_HOME`/geo2map cache, so it runs in CI) proving the new methods exist
and a flat `initialize` -> `initialize` re-init cycle runs without a crash.
`tests/unit/test_session_reinit.cpp` is the rg_core-only (no Godot)
counterpart: builds a terrain Session on a synthetic fetch and destroys it,
a flat Session, a terrain Session again, all in one process, then checks a
flat-mode `state_hash()` (the `hash_check`-style scenario) is unchanged
across that whole sequence.

## Player modes and world switch (R9)

State machine in `rg_core` (`rg/player_mode.h`, engine-neutral), bound by
`RgSimulation` (`set_player_mode(name)`, `cycle_player_mode()`,
`get_player_mode()`, `get_mode_state()` -> `{mode, implemented,
vehicle_control, driving_inputs_live, camera_inputs_live, camera_rig,
world_kind, world_phase, other_world, revision}`); GDScript holds only the
rigs, the input mapping and the HUD.

| mode | implemented | car | driving inputs | camera inputs | rig |
|---|---|---|---|---|---|
| `drive` | yes | player | live | live (look-around) | `chase` |
| `free_cam` | yes | unattended | - | live | `free` |
| `drone_follow`, own car (default) | yes | player | live | live | `drone` |
| `drone_follow`, NPC target | yes | unattended | - | live | `drone` |
| `cockpit` | reserved | - | - | - | `seat` |
| `on_foot` (R9c) | yes | unattended | - (walking inputs live) | live (look) | `walker` |

- `request_mode` refuses an unimplemented mode; `cycle_mode` skips them.
  Cycle order: drive -> free_cam -> drone_follow -> on_foot -> drive. Getting
  out is gated on the car speed (`kGetOutMaxSpeedMps` 2.0): the speed-aware
  `request_mode(mode, speed)` answers `Result::Refused` (counted in
  `get_out_refusals()`), `cycle_mode(speed)` skips on_foot; the binding passes
  the chassis speed.
- Drone follow (R9b): the camera trails a TARGET vehicle from above/behind
  (`drone_rig.gd`, pure display). `PlayerModeMachine::set_drone_target(id)`
  holds the target (empty = the player's own car, the default on entering the
  mode; reset on any mode change); `rules_for(DroneFollow, target)` gives
  Player + live driving inputs for the own car, Unattended + no driving
  inputs for an NPC target. `next_drone_target(...)` cycles own car -> NPC
  vehicles nearest first (ties by id, relative to the PLAYER's car, not the
  camera - a camera-relative order would ping-pong between the two nearest
  vehicles) -> own car, range 1500 m. Bound as `set_drone_target(id)`,
  `cycle_drone_target()` (returns the new id, -1 = own car),
  `get_drone_target_transform()` (Variant, null when the target is gone) and
  `get_mode_state()` `drone_target_id`/`drone_target_own`/`drone_target_label`.
  `Session::set_followed_vehicle(id)` makes the followed actor a SECOND
  physics interest point (id 1; the player's stays id 0), exempts it from the
  traffic despawn rules and the truck's 200 m stop rule, and when it
  disappears anyway (route end, no such id) clears the follow and counts it
  in `StreamingStatus::followed_lost`/`followed_lost_id`; the binding then
  returns the mode to the own car. The NPC truck's vehicle id is
  `Session::kNpcTruckVehicleId` = 1<<62: traffic ids count up from 1 so they
  can never reach it, and it stays a positive int64 for Godot. VR keeps its
  `xr_free` rig for the drone view (no XR drone rig yet).
- On foot (R9c): see "On foot (R9c)" below.
- World switch: `begin_world_load(kind)` -> Loading (supersedes a load in
  flight) -> `finish_world_load(serial, ok)` -> Ready/Failed. The mode is
  kept across a switch; while the world is not Ready, `effective_rules()`
  masks the driving inputs. No automatic fallback on failure.
- Unattended car (`unattended_controls(speed)`, applied by `Session` itself
  to its real-time loop's control copy): steer/throttle/starter 0, clutch 1,
  brake 0.6 above 2 m/s, then brake 1 + handbrake 1.
- Render streaming follows the ACTIVE camera (`camera_director.gd` ->
  `RgTerrainView.update_focus`); physics interest follows the vehicle.
- `RgSimulation.initialize()`/`initialize_terrain()` call
  `begin_world_load`/`finish_world_load` themselves; the binding pushes
  `effective_rules().vehicle_control` into the Session after every change.
- Relocation (`Session::request_relocate`/`request_reset_to_spawn`, bound as
  `relocate_vehicle(x, y, yaw_deg)`/`reset_vehicle_to_spawn()`; counted in
  `StreamingStatus::relocations`/`relocate_failures`): the "reset car" key
  and the smoke test's forced gate freeze. `set_fetch_delay_ms(ms)` wraps the
  next `initialize_terrain`'s fetch in `g2m::phys::DelayedFetch`.

Command line (`main.gd`, user args after `--`): none = flat scene + Drive;
`--drive` = real world + Drive; `--free-cam` = start in FreeCam;
`--g2m-fetch-delay-ms N`; `--drive-smoke`; `--screenshots <dir>` (with or
without `--drive`: `drive_tour.gd`, see "Vehicle visual (carvis)" below);
`--road-shots <dir>` (with `--drive`: `road_shots.gd`, roads_plan.md R-4, see
"Visible roads (R-2)" below); `--terrain-preview` and `--bindings-test` unchanged. Flags pick only the
start; mode and world change at runtime.

Keys (`input_map.gd`; the HUD shows a one-line summary):

| action | keyboard | gamepad |
|---|---|---|
| cycle mode (drive / free cam / drone follow / on foot) | V | Back |
| get out of the car (below 2 m/s) / get into it at the door | G | X (get in only: X is shift-down while driving) |
| on foot: move / run / jump | WASD / Shift / Space | left stick / L3 / A |
| on foot: first / third person | Tab | right-stick click |
| drone follow: next target (own car -> NPC vehicles) | N | D-pad right |
| drone follow: zoom | mouse wheel, PageUp / PageDown | - |
| switch world (flat / real; also cancels a load) | F8 | - |
| reset car to spawn | R | Y |
| drive: steer / throttle / brake | A, D / W / S | left stick / RT / LT |
| drive: shift up / down | E / Q | B / X |
| drive: handbrake / clutch | Space / C | A / LB |
| ignition toggle (starts on) / starter (hold) | I / K | D-pad up / - (no pad default) |
| auto-shift toggle (starts on) | F5 | D-pad left |
| free cam: move / up / down / fast | WASD / E, Space / Q, Ctrl / Shift | left stick / RB, RT / LB, LT / L3 |
| look (both rigs) | arrows; right mouse button captures the mouse, Esc releases | right stick |

## On foot (R9c)

The player walks beside the parked car, then gets back in. Engine-neutral
logic in `rg_core`; GDScript only reads state and forwards input.

- `rg/walker.h`/`walker.cpp`: `WalkerController` owns ONE kinematic Jolt
  capsule (radius 0.3, height 1.75, moved like the NPC traffic bodies:
  `set_motion((target - pos)/dt)`, so the body pose after `World::step` equals
  the controller's state and it is hashed through pose + velocity). Collision is
  the controller's own: `collide_shape_into` for walls/ceilings, `ray_cast`/
  `ray_cast_excluding` for ground (never `shape_cast`, which cannot exclude the
  walker's own body). Ground move = raise by the step height, move, resolve
  walls (contacts at or below the step height are not walls), drop ray; a rise
  over 15 cm or a slope over 45 deg is refused and slid along. A drop deeper
  than 0.3 m goes airborne. No ground at all (terrain tile not loaded):
  `hold` keeps the height instead of falling into the void.
- The own car's physical chassis box is thin (wheel casts need it), so it is
  IGNORED by the walker; instead `vehicle_footprint()` builds an oriented
  rectangle from the wheel attachments + chassis half extents (+5 cm margin,
  z prism chassis -1.0..+0.85 m) that blocks the walker and defines the get-in
  range (<= 1.5 m from the outline, |dz| <= 2.5 m). NPC cars and the truck are
  kinematic boxes and block through overlap queries. NPC TAKEOVER IS NOT
  SUPPORTED: traffic actors have no drivetrain, so getting in only ever means
  the own car.
- `Session` (session.h "walker" block): requests are atomics processed in
  `step_once` AFTER the terrain gate and any relocation (`process_walker_request`,
  1 spawn, 2 enter, 3 despawn), so a spawn only happens with the car's terrain
  resident; answers are counters (`walker_counters()`: spawned, entered,
  enter_refused, despawned, spawn_failed) the binding polls. Input:
  `set_walker_input` (mutex) + `request_walker_jump` (atomic counter); a
  synchronous `step()` applies it too. `update_walker()` runs in the vehicle
  block of `step_once` and refreshes the car footprint each tick.
  `spawn_walker`: `select_spawn_spot` tries the driver side (+local y, door at
  x 0.3, offsets 0/0.7/1.4 m), the passenger side, rear, front, then above the
  roof, facing away from the car.
- Interest points: the walker is point 0 (`kPlayerInterestId`), the parked car
  (or the relocation target) point 1 (`kParkedCarInterestId` = `kFollowedInterestId`);
  a followed vehicle is ignored while a walker is active. The TileManager pool
  was already sized for two points. NPC traffic spawns/despawns/wakes relative
  to `traffic_anchor()` (the walker while on foot) and the truck's 200 m rule
  uses the nearer of car and walker.
- Binding (`rg_simulation.cpp`): `apply_mode_to_session()` sends a spawn/despawn
  request only when "mode == OnFoot" changes (`walker_wanted_`, reset per
  Session, so a world switch re-spawns the walker in the new Session);
  `poll_walker()` (from `get_mode_state()`) turns `entered`/`spawn_failed` into
  `request_mode(Drive)`; `set_walker_input(move_right, move_forward,
  look_forward (Godot), run)`, `request_walker_jump()`, `request_walker_enter()`
  (only while on foot), `get_walker_state()` ({} without a walker; position/
  velocity/facing in the Godot frame, origin-relative, grounded, hold, blocked,
  can_enter, enter_distance_m, car_position); `get_mode_state()` adds
  `walking_inputs_live`, `get_out_refusals`, `walker_enter_refused`;
  `set_player_mode` answers `"refused"`.
- GDScript: `walker_rig.gd`/`walker_visual.gd` (above); `input_map.gd` walking
  group (`get_walk_move/get_walk_run`, edge counts `consume_jump_count`,
  `consume_interact_count`, `consume_get_out_key_count`); `main.gd`
  `_forward_walking`, `_notify_mode_events` (HUD messages for a refused get-out
  / get-in), `scripted_walk` (drive_smoke), the G / Space handling in
  `_process`; `hud.gd` `show_message`, the bottom prompt ("G / X: get into the
  car") and the on-foot line. VR has no walker rig (`xr_free` is kept).
- Tests: `[on_foot]` (see the layout entries above) and `tools/smoke_test.ps1
  -Drive` (drive_smoke.gd: get-out refused at speed, spawn, run away, get-in
  refused out of range, run back, get in, drive).

## Shell (R5, 2026-10-05)

Menus, spawn picker, six cameras, loading screen, settings and credits. Logic in `rg_core`, GDScript only builds controls from view-models and forwards input (engine-neutral rule).

- **Start.** A plain launch = boot splash (`shell_ui.gd`, 1.5 s or any key; attribution line at the bottom) -> main menu (Free roam | Settings | Credits | Quit). These flags start a world directly and skip the menu (`rg::ShellFlow::direct_start`): `--drive`, `--flat`, `--free-cam`, `--drive-smoke`, `--screenshots`, `--road-shots`, `--g2m-fetch-delay-ms`, `--camera-test` (`main.gd` `_build_scene`, `_direct_start`). `--shell-test` boots into the shell; `--shell-user-dir <dir>` redirects settings.json / last_drive.json (tests). `tools/run.ps1` opens the shell by default, `-Drive` forwards `--drive`, `-Flat` forwards `--flat`.
- **Flow.** `rg::ShellFlow` (core/include/rg/shell_flow.h) decides, `main.gd` `_apply_transition` (~1111) carries out the actions: `_show_screen`, `_begin_world_load` (spawn override + `_load_world`), `_unload_world` (`_release_world_nodes` + `RgSimulation.unload()`), `_set_paused`, `save_settings`, `_quit_game`. Esc / P pause (`_input` ~1062, `_on_escape` ~1192; an open F6/F7/F10 panel or a captured mouse takes the Esc first), Esc backs out of spawn picker, credits, settings and the loading screen (cancel = back to the main menu + unload). Pause menu: Resume | Reset car | Settings | Main menu. No Garage / Events / Map & route (not built yet, no placebo entries).
- **World lifecycle.** Free roam -> spawn picker (presets from `data/world/spawn_presets.json`, "last position", "address search", "flat world") -> Loading -> Drive. A real-world pick sets `RgSimulation.set_spawn_override(x, y, yaw)`; the flat world clears it. `_world_running()` (~692) tells the flow one frame later (`call_deferred("_post_load_ready")`) so the transition that started the load finished its own actions. Main menu from Pause tears the session down (`_release_world_nodes` ~622, `RgSimulation.unload`: sim thread stopped, terrain streaming stopped, modes/cinematic reset) so Free roam works repeatedly in one process (shell_flow_test: node count identical after each round).
- **Loading screen** (`loading_overlay.gd`): the real `get_init_status()` numbers as a 4-stage checklist, a ProgressBar while streaming the spawn area (gate_total - missing_required) / gate_total (gate_total = the largest missing count seen, StartupProgress::gate_total, 2026-10-06) and during priming (prime_done / prime_total), the place name and the attribution line; a load failure started from the menu returns to the main menu with the message (no script error).
- **Settings** (`rg::Settings`, `<user dir>/settings.json`, saved when the settings screen is left and on window close): applied in `main.gd` `_apply_setting` (~1216): window mode, vsync (skipped under VR), max fps, field of view (`camera_director.set_base_fov`; rigs add their own offsets: bumper +10, cockpit +5), audio master/engine/tyre volume (gain multipliers in `vehicle_audio.gd`, which bypasses Godot buses), map store dir (applies at the next world load). traffic.cfg, seat_positions.cfg and address_search_cache.cfg are untouched.
- **Cameras** (`rg::DriveView`, `main.gd` `_drive_view`, `set_drive_view` ~1205): B / D-pad right cycles chase -> bumper -> cockpit -> orbit -> cinematic (Drive mode only); Tab / right-stick click keeps toggling chase <-> cockpit (`RgCameraMath.toggle_cockpit_view`); V mode cycle unchanged. Rigs: `chase_rig.gd`, `bumper_rig.gd` (mount from `RgSimulation.get_bumper_camera`), `cockpit_rig.gd`, `orbit_rig.gd` (state in `RgCameraMath.orbit_step`, mouse/stick look, zoom), `cinematic_rig.gd` (`RgSimulation.update_cinematic`: roadside cameras along the client road data ahead of the car, the car's predicted path in the flat world and where no road is known; `set_road_ahead_wanted(true)` only while this view is active), `free_rig.gd`; all follow the PHYS-008 rig contract (director priority -1000, only the active rig rebases, every rig root shifted on an origin change).
- **Credits** screen and attribution line come from `data/credits.json` via `rg::Credits`.
- **Tests.** Core: test_credits/settings/spawn_presets/shell_flow/camera_math/road_ahead (+ player_mode unload, session pause). Headless: `tools/smoke_test.ps1 -Shell` (74 checks, 92 with the real-world round `--shell-real` that smoke_test adds when a geo2map store exists - cancel a load with Esc, then load and drive the real world, unload: boot -> main menu -> settings save/reload -> credits -> Free roam flat -> pause/resume/reset -> main menu -> Free roam again -> quit), `-Cameras` (PHYS-008: all 20 ordered pairs of the drive views plus the free cam, plain and with a floating-origin rebase in the switch frame, sim paused; tolerance 0.05 m / 0.5 deg, measured worst 0.0014 m / 0.0 deg; the sabotage "director no longer syncs the origin" fails it). Both in `tools/ci.ps1`.
- **Deferred (PLAN R5/11.2/11.3):** pick-on-map spawn (needs R4's map), Garage / Events / Map & route menu entries, map server endpoint / pin region / cache size settings, resolution scale / FSR, input rebinding (done in R5b), gamepad pause button (done 2026-10-06: pad Menu/Start = Esc, the starter has no pad default), drone/walker rigs in the camera-switch matrix (they need a drone target / walker; same contract).

## Controls (R5b, 2026-10-06)

Control configuration menu with per-device memory. Full description: `docs/controls.md`. Logic in `rg_core` (`control_types.h`, `control_schema.h`, `control_binding.h`, `control_capture.h`, `controls.h` + `core/src/control*.cpp`), `RgControls` (`godot_ext/src/rg_controls.*`) converts Variants, `input_map.gd` gathers a raw snapshot (keys, mouse, pads) and calls `RgControls.evaluate`, `controls_ui.gd` draws.

- **Model.** One action schema (`data/controls/actions.json`); profiles hold overrides over class defaults (`data/controls/defaults/*.json`); the player's file is `<user dir>/controls.json` (`rg.controls/1`; broken file -> defaults + `.bak`). Device key `keyboard` / `mouse` / `joy:<guid>#<n>` (SDL GUID + ordinal among same-GUID devices; fallback own -> GUID#1 -> class default); hot-plug via `Input.joy_connection_changed`; several devices at once, larger magnitude wins. `project.godot` has no `[input]` section any more; menu actions `ui_accept`/`ui_cancel` are rebuilt from the bindings (`input_map.sync_menu_actions`).
- **Screen.** `shell_ui.gd` screen `controls` (Settings -> "Controls...", pause menu "controls"), `controls_ui.gd`; `main.gd` keeps `get_controls()`/`get_input_map()`, suspends `input_map` while the screen is up and saves on leaving/close. Test hooks: `get_hook(id)` (`bind:<action>:<sign>`, `add:`, `clear:`, `reset:`, `tune:`, `invert:`, `calibrate:`, `combine:`, `reset_device`, `calib_apply`, `overlay_cancel`), `inject_pad`, `begin_capture`, `select_device`, `status_text`; `input_map.set_test_snapshot` for simulated keys/pads.
- **Tests.** Core `test_controls.cpp` (41 cases), `test_control_capture.cpp`, `test_shell_flow.cpp`. Headless `tools\smoke_test.ps1 -Controls` (two Godot processes: `--controls-test` 62 checks writes, `--controls-verify` 14 checks reads back after a restart; in `tools\ci.ps1`), `-Shell` covers pause -> Controls -> Esc. Screenshots: `tools\controls_shots.ps1` (`--controls-shots <dir>`) into `out/controls_screens/`.
- **Deferred.** Real hardware only simulated (gamepad focus navigation, wheel/pedals, identical pads, hot-plug); steer rows of a pad show side rows unbound when the full axis is bound; the fixed keys are not rebindable; no per-vehicle profiles.

## Garage (R6, 2026-10-05)

Vehicle select, configurator and the garage set. Logic in `rg_core` (`vehicle_catalog.h`, `vehicle_setup.h`, `garage_set.h`, `garage.h`), `RgGarage` binds it, `garage_scene.gd` / `shell_ui.gd` only draw and forward.

- **Flow.** Main menu Garage (after Free roam) -> vehicle select (cars + stats from the data, turntable shows the highlighted car) -> Configure -> configurator (tabs overview / wheels & brakes / engine bay / rear: the camera flies to the area, the panel lists its options) -> Save / Drive. Drive saves a valid unsaved change first, then opens the spawn picker like Free roam. Pause menu "Garage (respawn)" (after Reset car) -> the same screens -> Drive respawns the chosen car (real world: at the old chassis pose via the spawn override; the flat world has one spawn). `rg::ShellFlow` screens VehicleSelect/Configurator, events vehicle_chosen/garage_drive, actions open_garage/close_garage, `garage_return()`.
- **Catalog** `data/vehicles/catalog.json`: car_hyper (default), car_sedan (the game's own `data/vehicles/car_sedan.json`), car_sedan_rwd/_awd (physics_sim's). Chassis mass/box (interim, PLAN physics request R5) lives in the catalog. The saved choice is `<user dir>/garage/selected.json`; direct-start flags (`--drive`, `--flat`, ...) drive `--vehicle <id>` or the catalog default, never the saved choice, so smokes stay deterministic.
- **Setups** `racing_game.vehicle_setup/1` (`<user dir>/garage/setups/<id>.json`): option id -> value; `data/vehicles/setup_options.json` whitelists JSON-pointer paths with ranges (final drive +-20 %, gear ratios +-20 % per gear and monotonic, brake force and bias, springs/dampers/ARBs +-30 %, tyre choice from `data/tyres/*.json`, assists = defaults of the assist.* channels, paint and rim colour visual only). The setup is a merge patch (RFC 7396) per file onto the vehicle file and the referenced engine/gearbox/tyre files, materialised into `<work_root>/drive_<id>_<hash>` and loaded with `ps::io::load_vehicle_json`; a loader error is shown verbatim and Save/Drive stay disabled. The drive uses the materialised copy (`main.gd` `_prepare_vehicle` -> `RgSimulation.set_vehicle_overrides`); work files are removed on `_unload_world` (`RgGarage.cleanup`) and on exit.
- **Garage set** (`data/garage/garage_set.json`, own procedural geometry, no external assets; Forza Horizon only as mood): dark room with ribbed walls, turntable with an orange ring and tick marks (slow spin), studio lights + softboxes. The renderer is gl_compatibility (no SSR/probes): reflections come from a sky shader that paints the softboxes plus a mirrored car twin (render layer 2, mirrored lights) seen through the glossy floor. Camera areas overview / wheels / engine bay (top) / rear; ISO to Godot: godot = (-iso.y, iso.z, -iso.x).
- **Teardown.** Leaving the garage (Back, Drive) removes the garage scene and every node it created; the car's visual (`body_visuals.gd clear_vehicle`) is released with the world, so node counts return to their pre-drive value (shell_flow_test and garage_test count them).
- **Tests.** Core: test_vehicle_catalog, test_vehicle_setup, test_garage, test_shell_flow. Headless: `tools/smoke_test.ps1 -Garage` (`garage_test.gd`, 106 checks: stock drive reads the wheel compression, Garage -> select -> front spring slider to x1.3 -> non-monotonic gear set rejected with the loader message, Save/Drive disabled, out-of-range final drive rejected -> Save -> a second RgGarage reads it -> Drive: the physics shows the front compression at 0.769 x stock and the rear unchanged -> pause -> Garage (respawn) -> sedan drives -> main menu: no world, no work file, node count back), in `tools/ci.ps1`. Screens: `--garage-shots <dir>` (needs a real window) writes vehicle_select / overview / turntable_rotated / edit_wheels / edit_engine / edit_rear PNGs (`out/r6_screens/`).
- **Deferred.** 787b/pipes/gen/tractor variants are not listed; ABS/traction control/ESC options (not in the vehicle file); seat offsets are keyed by vehicle name at ready. Physics requests: chassis mass/box/inertia in the vehicle file, `load_vehicle_json_from_string` (no work files), real tyre compound data (`data/tyres/*.json` here are game-owned approximations).

## Car browser and in-world car switch (R6c, 2026-10-06)

The Garage screen (main menu, pause "Garage (respawn)") and the new pause entry "Change car" are one browser. Logic in `rg_core` (`car_browser.h`, presets in `vehicle_catalog.h`/`garage.h`), `RgGarage.browser_*` binds it, `car_browser_ui.gd` / `car_thumbnails.gd` only draw and forward (full file list in the Layout above).

- **Presets are catalog entries.** `preset_of` (a base entry listed BEFORE it) + `preset_setup` (the same option ids and values as a saved setup, validated against setup_options.json at load) + own id/title/subtitle/body type/look. A preset inherits files, chassis and model from its base; the player's saved setup is per entry and applied on top. Shipped: car_sedan_comfort/sport, car_sedan_rwd_drift/track, car_sedan_awd_rally/winter, car_hyper_track/stealth. `body_type` (open list), optional `manufacturer`, optional `displacement_l` + `displacement_source`; displacement is computed from the engine file (bore x stroke x cylinders), else the catalog field, else "-". Stats of a preset are those of its base vehicle file (the overlay is not applied to the stats shown).
- **Flows.** Main menu Garage: browser -> Configure -> configurator -> Drive (unchanged after the browser). Pause "Change car": browser -> select -> the world reloads with the new car at the old position and heading (flat world: `RgSimulation::initialize` applies the spawn override as `request_relocate`; real world: the existing spawn override), zero velocity, no road reset, the loading overlay is shown. `ps::World` has no vehicle removal, so the swap is a Session rebuild, not an in-place swap. Esc closes an open filter panel first, then leaves.
- **Persistence.** group_by / sort_key / sort_desc / filter_layouts / filter_body_types / filter_power_bands are hidden settings (`browser.*`, not on the Settings screen), saved on every change by `main.gd` `_on_browser_state_changed`.
- **Thumbnails.** `car_thumbnails.gd` renders one picture per frame in its own SubViewport (never the garage or driving world), key = `rg::thumbnail_key(model, paint, rim, version)`; bump `kThumbnailRenderVersion` (car_browser.h) with `RENDER_VERSION`-relevant camera/light/shader changes. Headless runs have no renderer: the tiles keep their placeholder.
- **Cost.** Nodes exist only for the visible columns plus 2 overscan (the 200-car synthetic catalog keeps < 70 tiles); the garage 3D viewport is switched off while a browser is up (`garage_scene.gd set_rendering`).
- **Tests.** Core: test_car_browser.cpp / test_car_browser_data.cpp (categories per group mode, filter, sort, grid navigation, displacement, preset overlay, a 200-entry synthetic catalog; each with a recorded sabotage), test_shell_flow (Change car). Headless: `tools/smoke_test.ps1 -CarBrowser` (open, keys, grouping, AWD filter vs the stats, settings persistence, F/Esc, Free roam -> relocate -> Change car -> same position within 0.5 m read from the physics, at rest, wheel load = m*g, node count back) and `-CarBrowserBig` (the synthetic 200-car catalog, tile window), both in `tools/ci.ps1`. Screens: `tools/car_browser_shots.ps1` -> `out/car_browser_screens/`.
- **Deferred.** Stats of a preset do not include its overlay; no search box; no manufacturer logos; thumbnails are rendered at 384x216 from one fixed three-quarter camera; the lift on a respawn is whatever `request_relocate` does (the owner should check the drop at the swap).

## Vehicle visual (carvis, 2026-09-26)

The drivable car uses physics_sim's own art (`external/physics_sim/data/
models/car_sedan/car_sedan.glb` + `car_sedan.rig.json`) instead of the red
placeholder box, ported from `external/physics_sim/adapters/godot/demo/
scripts/vehicle_visual.gd` (read-only reference - `data/models` is owned by
another session; read it, never copy/edit it into this repo).

- `game/scripts/vehicle_visual.gd` loads the `.glb` at runtime via
  `GLTFDocument` (not through the editor's `res://` import pipeline), binds
  `susp_<corner>`/`steer_<corner>`/`wheel_<corner>` nodes by name
  (`car_sedan.rig.json`'s own names, corner in `{FL,FR,RL,RR}`), aligns the
  model root to the physics wheel attachment points (mean per-axis offset,
  per-wheel residual bias), and every frame sets suspension travel/steer
  angle/spin angle from `RgSimulation`'s per-wheel accessors. Keyed by a
  model-name string (`vehicle_name`) so a future `car_hyper` reuses it
  unchanged. No `-90 deg X` rotation and no client-side spin integration
  (see the file's own header comment for both, and physics_sim's reference
  file for the full model-space derivation) - `rg::vehicle::WheelState`
  already carries an integrated `spin_angle`.
- `game/scripts/body_visuals.gd` parents `VehicleVisual` (and the red
  `ChassisBox` fallback) under `ChassisRoot`, the one node whose transform
  is set from `get_body_transform("chassis")` each frame - a floating-origin
  rebase moves both together. The box is hidden once the model reports
  loaded (`VehicleVisual.model_loaded()`); it stays visible as a fallback if
  the `.glb` fails to load, and `vehicle_model_ok()` lets `hud.gd` show a
  matching HUD warning (`vehicle_visual.gd` also `push_warning()`s to the
  console either way).
- `RgSimulation` additions (`godot_ext/src/rg_simulation.h/.cpp`):
  `get_wheel_attachment_local`/`get_wheel_steered`/`get_wheel_is_front`
  (static per-vehicle geometry, read from `rg::Session::vehicle_desc()`,
  cached at spawn - mirrors physics_sim's own `ps_simulation.cpp::
  wheel_desc()` pattern) and `get_wheel_compression`/`get_wheel_spin_angle`/
  `get_wheel_steer_angle` (per-frame, read from the race-free
  `Session::snapshot().wheels[i].state`, i.e. `ps::vehicle::WheelState` -
  NEVER a live `World` accessor called from the Godot thread; the sim runs
  on its own thread, see "Session terrain mode (R4)" above).
- World-switch survival (R7/R9): `body_visuals.gd.on_session_ready()`
  (called by `main.gd` right after a Session starts/re-starts, both the
  flat and the real-world path) calls `VehicleVisual.rebuild()`, which
  re-runs wheel-node binding and model alignment against whatever `Session`
  `RgSimulation` now holds - cheap (no `.glb` reload) - since a world switch
  rebuilds `rg::Session` from under the same `RgSimulation` node, and the
  wheel count/attachment points `VehicleVisual` cached at its own `_ready()`
  may belong to a destroyed Session by then.
- `drive_tour.gd` (`--screenshots <dir>`, with or without `--drive` - a
  flat-world run has no loading phase, so the tour just skips that one shot
  and runs the same sequence unattended) captures: `01_spawn_chase.png`
  (chase cam at spawn), `02_steering_close.png` (car held stationary on
  brake+handbrake, full steer lock, chase rig zoomed in AND swung to a 3/4-
  rear angle via `chase_rig.gd`'s `side_offset_m` - proves the front wheels
  turn under a real per-wheel `steer_angle`; a dead-on rear view, tried
  first, cannot show a front wheel at all - the body occludes it face-on
  whatever the distance/height), `03_drive_chase.png`, `04_free_cam_above.png`,
  and (real-world only, if the load is slow enough) `05_loading_overlay.png`.
  `poses.txt`'s `wheel_ride_height_m` (carvis, 2026-09-27:
  `chassis_session_position().z + attachment_local.z + get_wheel_compression()
  - wheel_radius`, per wheel) is the ride-height check the carvis brief asked
  for - exactly 0 at the flat world's settled spawn (`[0.0000, 0.0000, 0.0000,
  0.0000]`), a few mm to ~3 cm while driving/braking (load transfer, not a
  bug). Only meaningful as "metres above ground" in the flat world, where
  session z is literally height above the z=0 plane - real-world session z is
  an absolute terrain-relative elevation (no ground-height-at-XY query is
  exposed), so the same field there is a large near-constant number, not a
  clearance; the real-world ride-height claim rests on the screenshots'
  visible ground contact shadow instead.

## Targets

- `rg_core` (STATIC, `core/`): `rg::Session` - owns one `ps::World` (a
  static ground box in flat mode or streamed terrain in terrain mode, one
  dynamic chassis body, one vehicle built from `ps::io::load_vehicle_json`),
  an `rg::FixedRateLoop` (240 Hz fixed-rate loop thread; its sleep is
  `ps_godot::detail::precise_sleep_until`, reused BY PATH from `external/
  physics_sim/adapters/godot/src/sim_thread.h` - Godot-free), a
  `ps_godot::FallDetector` (terrain mode, reused by path, `fall_detector.h`),
  a `ps_godot::TripleBuffer<FrameSnapshot>`
  (reused by path, `triple_buffer.h`) for race-free snapshot publication,
  and a `ps_godot::OriginRebase` (reused by path, `origin_rebase.h`/
  `frame_convert_core.cpp`) for floating-origin support. Also `rg::
  load_world_config` (`world_config.h`/`.cpp` - see "World config" below).
  Links `ps_core` PUBLIC, plus (PLAN.md R2.0) `g2m_server`/`g2m_client`/
  `g2m_layers` PUBLIC and `nlohmann_json::nlohmann_json` PRIVATE (used only
  inside `world_config.cpp`'s own implementation). `g2m_mesh` does not exist
  yet on geo2map_engine's current pin - not linked; add it once it lands. No
  Godot type anywhere (engine-neutral, MEMORY.md's engine-neutral-logic rule
  - a future UE5 port reuses this target unchanged). `g2m_mesh` (PLAN.md
  R2.1, landed on the submodule pin with G2.3) adds `rg::WorldTerrain`
  (`world_terrain.h`/`.cpp`) + `rg::RenderChunk`/`build_static_view_from_
  lookup` (`terrain_render.h`) - see "Terrain preview (R2.1)" above.
  `g2m_phys`/`g2m_ps_bridge` (R2.2 R1, landed on the submodule pin with G9)
  are also linked PUBLIC: `g2m_phys` is geo2map_engine's own ps-free
  physics-tile-grid library (`PhysicsTileGrid`/`ResidentHeightSet`/
  `fill_physics_heights`/`HeightTileLoader`/`PhysicsTerrainStreamer`);
  `g2m_ps_bridge` (`bridges/physics_sim/` on the geo2map_engine submodule,
  built only via its own `if(TARGET ps_core)` guard - satisfied because this
  repo's own top-level `CMakeLists.txt` `add_subdirectory`s `external/
  physics_sim` before `external/geo2map_engine`) adds `g2m::ps_bridge::
  G2mTerrainSource : ps::terrain::ITerrainSource` +
  `make_terrain_config(interest_radius_m, fills_per_tick)`. PUBLIC, not
  PRIVATE, because R2.2 R4 puts a `G2mTerrainSource` inside `rg::Session`,
  so it will be named in `session.h` (a public header) the same way
  `ps::World` already is - see `core/CMakeLists.txt`'s own comment.
  `test_g2m_ps_bridge_link_smoke.cpp` (below) proves the link; `Session`'s
  terrain mode uses both (see "Session terrain mode (R4)").
- `rg_godot` (SHARED, `godot_ext/`): the GDExtension DLL
  (`game/bin/librg_godot.dll`). Two classes registered: `RgSimulation :
  godot::Node` owns one `rg::Session` and exposes it to GDScript (body
  transforms, control channels, wheel/gauge/powertrain telemetry,
  origin-rebase seam); `RgTerrainView : godot::Node3D` (PLAN.md R2.1) owns
  one `rg::WorldTerrain` and uploads its `RenderChunk`s via the low-level
  `RenderingServer` API - see "Terrain preview (R2.1)" above. Links
  `rg_core` + `godot-cpp`. This is the ONE place `ps::`/`rg::`/`g2m::` types
  cross into `godot::` types (mirrors physics_sim's own adapter's "all
  conversion happens in exactly one place").
- `lod_measure` (executable, `tools/lod_measure/`): links `rg_core` only
  (no Godot) - the PLAN.md R2.1 LOD-distance measurement sweep, see
  "Terrain preview (R2.1)" above for the tool and its measured table.
- `hash_check` (executable, `tools/hash_check/`): builds an `rg::Session`
  matching `external/physics_sim/data/scenarios/vehicle_step_steer.json`
  by hand (ground/chassis/vehicle construction + the same control events),
  steps it, and prints its `state_hash` for comparison against physics_sim's
  own `ps_run` on the same scenario file - see "Hash comparison acceptance
  check" below.
- `rg_test_catch_main` (STATIC, `tests/unit/`): custom Catch2 v3 entry
  point (own fetch, v3.16.0 - NOT physics_sim's `ps_test_catch_main`, which
  only exists when `PS_BUILD_TESTS=ON`, and this repo sets it `OFF`).
  Links `Catch2::Catch2` + physics_sim's always-built `ps_headless_env`.
- `rg_unit_tests` (executable, `tests/unit/`): `rg_core`'s own tests
  (`test_session.cpp`), `rg::load_world_config` coverage
  (`test_world_config.cpp` - happy path, `${VAR}` expansion, the
  `RG_G2M_HOME` default, every validation error path, plus PLAN.md R2.1's
  `lod`/`max_distance_m` cases), a geo2map_engine link-smoke test
  (`test_g2m_link_smoke.cpp` - constructs a `g2m::TileKey` and round-trips
  `packed()`/`unpack()`, both defined out of line in `g2m_core`, proving
  `rg_core` actually LINKS a geo2map_engine symbol, not just compiles
  against its headers), a physics-bridge link-smoke test (R2.2 R1,
  `test_g2m_ps_bridge_link_smoke.cpp` - `g2m::ps_bridge::make_terrain_config
  (400.0, 1)` gives `max_resident_tiles == 49`/`max_tile_fills_per_tick == 1`
  (the pool-49/F-stays-1 decision geo2map_engine PLAN.md 8.5 records "as
  built"), plus a bare `g2m::phys::PhysicsTileGrid` `PhysTileIndex`
  pack/unpack round trip - no real-data access, proves `rg_core` LINKS
  `g2m_phys`/`g2m_ps_bridge`, not just compiles against their headers) and
  `rg::WorldTerrain`/`build_static_view_from_lookup`
  coverage (`test_world_terrain.cpp` - PLAN.md R2.1, synthetic in-memory
  `TileKey`->`HeightTile` map, see "Terrain preview (R2.1)" above), and
  `rg::road_classes` coverage (`test_road_classes.cpp` - roads_plan.md R-2,
  synthetic `RoadSegment` lists, see "Visible roads (R-2)" below), and the
  real-store route/road-coverage acceptance check (`test_route_road_coverage.cpp`
  - roads_plan.md R-3, hidden `[.][realdata]`, see its own layout-entry line
  above and "Visible roads (R-2)" below). Catch2
  v3 via `rg_test_catch_main`, plus `nlohmann_json::nlohmann_json` PRIVATE
  (test fixture JSON is built with `nlohmann::json` directly). Registered
  with CTest.

## CMake options

- `RG_BUILD_TESTS` (default `ON`): build `rg_unit_tests`, fetch Catch2.
- `RG_BUILD_GODOT_EXTENSION` (default `ON`): build `rg_godot`, fetch
  godot-cpp (branch `4.5`). Godot RUNTIME is 4.7.2 (`compatibility_minimum
  = "4.5"`, `compatibility_maximum = "4.7"` in `game/rg_godot.gdextension`).

physics_sim's own build options are force-set before its
`add_subdirectory`: `PS_BUILD_TESTS=OFF` (its own Catch2/`ps_run`/`sweep`/
bench targets are not needed here - `ps_run` can still be built explicitly
by target name for the hash-check acceptance evidence, see below),
`PS_BUILD_GODOT_ADAPTER=OFF` (its own Godot adapter/demo are irrelevant to
this repo).

geo2map_engine's own build options (`G2M_BUILD_TESTS`/`G2M_BUILD_APPS`/
`G2M_BUILD_IMPORT`/`G2M_BUILD_FUZZERS`, all forced `OFF`) and
`RG_G2M_DEPS_DIR` are documented under "geo2map_engine submodule" above.

## Dependencies (FetchContent, pinned)

| Dependency | Pin | Notes |
|---|---|---|
| physics_sim | submodule commit (see `.gitmodules`/`git submodule status`) | brings JoltPhysics v5.6.0 + nlohmann/json v3.12.0 transitively, `PS_BUILD_TESTS=OFF`/`PS_BUILD_GODOT_ADAPTER=OFF` |
| geo2map_engine | submodule commit (see `.gitmodules`/`git submodule status`) | brings zstd v1.5.7 + SQLite 3.53.4 + its own nlohmann/json v3.12.0 pin transitively (`g2m_dep_*`, PRIVATE to geo2map_engine's own targets); `G2M_BUILD_TESTS=OFF`/`G2M_BUILD_APPS=OFF`/`G2M_BUILD_IMPORT=OFF`/`G2M_BUILD_FUZZERS=OFF` |
| godot-cpp | branch `4.5` | only when `RG_BUILD_GODOT_EXTENSION=ON` |
| Catch2 | `v3.16.0` | only when `RG_BUILD_TESTS=ON`; `/EHsc` applied to its targets under MSVC-family compilers |

## CMake presets

Windows (Ninja; run from a VS 2022 Build Tools developer environment -
`tools/run.ps1`/`smoke_test.ps1`/`ci.ps1` enter it automatically):

| Preset | Compiler | Build type |
|---|---|---|
| `debug` | clang-cl | Debug |
| `release` | clang-cl | Release |
| `relwithdebinfo` | clang-cl | RelWithDebInfo |
| `asan` | clang-cl | RelWithDebInfo + AddressSanitizer |
| `msvc-release` | cl | Release |

`asan`'s ASan runtime `/WHOLEARCHIVE` linking (top-level `CMakeLists.txt`)
applies to `CMAKE_EXE_LINKER_FLAGS`, `CMAKE_SHARED_LINKER_FLAGS` and
`CMAKE_MODULE_LINKER_FLAGS` alike - needed because `rg_godot`
(`godot_ext/`) is a `SHARED` target, not an executable, and lld-link never
implicitly links the ASan runtime for clang-cl regardless of target type.

Linux (`linux-release`, Ninja + clang + lld, binary dir
`~/build/racing_game/<preset>`): `RG_BUILD_GODOT_EXTENSION` forced `OFF`
(godot-cpp is Windows-only in this repo's current scope).

`EXCLUDE_FROM_ALL` note: the submodule's own targets (`ps_run`, `sweep`,
etc.) are NOT in the default build set. Build one explicitly by name if
needed, e.g. `cmake --build out\build\debug --target ps_run` (used for the
hash-check acceptance evidence).

## Hash comparison acceptance check

`hash_check.exe [data_dir]` (default `external/physics_sim/data`) builds
an `rg::Session` with the SAME ground/chassis/vehicle parameters and
control-event sequence as `external/physics_sim/data/scenarios/
vehicle_step_steer.json`, steps it 960 ticks at 240 Hz, and prints
`state_hash=0x...`. Comparing this against physics_sim's own `ps_run
external/physics_sim/data/scenarios/vehicle_step_steer.json` run from
inside this repo's own build tree is a genuine regression test of
`rg::Session`'s hand-written world-construction code (`build_world_
contents` in `core/src/session.cpp`) against physics_sim's own scenario
loader - not a tautological reload of the same JSON. A mismatch means
`Session`'s construction diverges from the scenario's own semantics
somewhere (wrong default, wrong body order, missed control event, etc.).

## Recovery and stall diagnostics

F / gamepad D-pad down requests flip upright while running in Drive mode.
RgSimulation.flip_vehicle_upright forwards to Session::request_flip_upright. The
stepping thread reads the current pose and reuses relocation at the current XY
with a pure-yaw orientation. Heading uses the projected forward axis, falling
back to the left axis when the forward projection is shorter than 0.2. Five-ray
placement clears the terrain, motion is zeroed and the gear becomes neutral; a
running engine stays running. An explicit same-tick relocation takes precedence.
R still resets to spawn. Unit tests cover roof/side recovery, the nose-down
heading fallback and relocation precedence.

The real-time Session loop queues attempts over 8 ms or with a start-to-start
gap over 20 ms (capacity 256). drain_tick_spikes returns and clears records and
the overflow count; synchronous stepping does not collect them. main.gd prints
RG_TICK_SPIKE during its one-second running-world reports, up to 40 lines plus a
suppressed count. Records include tick/wall time, stepped/frozen/relocation
kind, gap and previous duration, total duration,
request/gate/control/World-step/post-step timings, position, speed and resident
tile count. RG_FRAME_SPIKE reports main-thread frames over 50 ms with the step
count. These thresholds exceed the 4.17 ms budget at 240 Hz.

Set RG_WORLD_CSV to an output path before launching to record ps::World per-tick
telemetry, including stage.<name>.ms, for attribution inside step_ms (for
example terrain_tiles). The path announcement goes to stdout to avoid the
PowerShell 5.1 smoke-test stderr failure rule.

tools/run.ps1 defaults to relwithdebinfo; select -Preset debug explicitly when
needed. All presets write game/bin/librg_godot.dll. Run/smoke helpers compare
game/bin/librg_godot.preset with the requested preset and remove a stale DLL
before building to force relinking. After a successful build they record the
preset. -SkipBuild warns on a mismatch and launches the existing DLL.
RgSimulation.get_build_info includes build_type=, and RG_DRIVE ready includes
build= so logs identify the binary actually loaded.

## Terrain prefetch snapshots

The geo2map bridge publishes immutable resident height/road snapshots between
World steps. Background terrain jobs retain those snapshots; replacement and
eviction use copy-on-write, and retired snapshots are reclaimed on the owner
thread. A ticket becomes stale when its footprint's tile state or height/road
identity changes. Unrelated resident changes do not invalidate it.

World physics config accepts prefetch_margin_tiles and prefetch_max_tiles,
defaulting to 0/0 (off). The shipped world enables margin 1 and cap 24. Margin
must fit the streamer's one-tile gate; enabled caps range from 1 to 1024.
Session status, Godot streaming status and tick-spike records expose all nine
physics prefetch counters. The HUD reports installs, synchronous misses,
late waits, maximum wait duration and stale inputs. Prefetch changes preparation
timing; per-tick hashes remain equal to synchronous terrain across worker counts.
## Building, testing, running

```powershell
tools\setup_dev_env.ps1 -CheckOnly   # verify toolchain (installs nothing)

# from a VS 2022 developer shell (tools\run.ps1/ci.ps1/smoke_test.ps1 enter one automatically)
git -c protocol.file.allow=always submodule update --init --recursive
cmake --preset debug -DRG_BUILD_GODOT_EXTENSION=ON
cmake --build --preset debug
ctest --preset debug --output-on-failure

tools\run.ps1              # incremental build + launch Godot on game/: real world, Drive mode
tools\run.cmd              # the same from cmd.exe
tools\run.cmd -Flat        # ... start in the flat test scene instead
tools\run.cmd -- --terrain-preview   # ... or launch the R2.1 static-terrain fly-camera preview instead
tools\smoke_test.ps1        # headless build + ctest + headless Godot run
tools\smoke_test.ps1 -TerrainPreview # ... or the R2.1 terrain-preview headless check
tools\smoke_test.ps1 -TerrainStream  # ... or the R8 streamed-LOD headless check (focus moved in 5 steps)
tools\smoke_test.ps1 -Drive          # ... or the R9 real-world drive + mode/world round trip (SKIP without a store)
tools\smoke_test.ps1 -DriveDelayMs 200  # ... the same with delayed fetches, a relocation and a forced gate freeze
tools\ci.ps1                 # debug + release legs + smoke_test
```

## G3 dependency checkpoint (2026-10-02)

physics_sim pin bd41ac4 adds cross-body edge welding for flush heightfield /
static-mesh transitions. The owner fixture reproduces the old 162 m/s² kick;
fixed bidirectional crossings match continuous surfaces at 1/4/24 workers.
Real bridge approaches remain an acceptance task. geo2map_engine pin 54d7083
adds opt-in roads.graph, complete reference fitting with explicit curvature
exceptions, DEM dependency discovery, sloping junction/bridge approach profiles,
numeric incline targets and opt-in roads.geom codec/server/bake serving.
Existing terrain layers remain the default. See PLAN.md G3 for current gaps.


## OSM speed-limit dashboard (2026-10-02)

FrameSnapshot::road_speed_limit carries already-resident ground-road metadata;
no tile fetch is performed while publishing the car snapshot. The Godot method
get_vehicle_speed_limit(vehicle_name) returns kind (numeric/unrestricted/unknown),
kph, conditional, osm_way_id and road_found. Forward/backward tags follow OSM
way direction; missing/symbolic values stay unknown. Conditional/lane/variable
tags are flagged but not evaluated. Bridge decks are excluded from this ground
matcher until 3D road geometry matching exists. tach_gauge.gd draws the readout.
Main scene selects the supplied physics_sim data/vehicles/car_hyper.json and art, with its authored 1 MW V8, seven-speed gearbox, tyres and suspension. Both flat and terrain initialization use the prescribed 0.50 m chassis height and {2.0, 0.4, 0.12} m collision half extents.

Physics update (2026-10-02): pin ed15a6f0c5b63a031d40cf419279fce3567378d3 combines committed master bf2c7cdc with the existing bd41ac4 terrain boundary fix. Default car uses the supplied submodule hypercar definition; the provisional game-owned hypercar copy is removed.

VR/audio integration (2026-10-03): physics_sim master pin ba47951 (fetched
from S:/claude_code/physics_sim) retains the terrain boundary fix and adds
the reusable Windows spatial backend. The supplied car_hyper remains the
default. tools/run.ps1 -VR launches OpenXR Vulkan Mobile; tracked cockpit
and FreeCam rigs share camera-director rebasing. Tyre and eligible live-engine
sources use listener-relative Windows objects with Godot 3D fallback.
Setup, hardware limits and verification: docs/vr_audio.md. The committed
hypercar torque-map engine has no live cycle voice; owner turbo/audio-bank
work remains pending. This presentation work does not change G3-C progress.

Latest physics upgrade (2026-10-03): master pin 2f4599c from
S:/claude_code/physics_sim includes 40923cf's supplied simulated twin-turbo
car_hyper and live engine voice, plus the embedded CMake-path correction.
The default car is unchanged by name and now loads these new library data.
Gauge metadata accepts SimulatedEngineDesc as well as torque-map engines.
Earlier notes about the committed hypercar lacking a live voice are superseded.
G3-C road work is unchanged.

Vehicle definitions are retained per path in RgSimulation after successful load, passed to SessionConfig::vehicle_definition for world rebuilds. Init-worker cache access is serialized by join/Ready acquire; world runtime state remains independent. Restart after editing definition dependencies. Verified warm reload 0.56 s, cancelled load 255 ms; initial V8 map generation ~34 s.


## Road reconstruction and terrain smoothing (2026-10-03)

`core/include/rg/road_surface.h`, `road_structures.h`, `terrain_smoothing.h`
and matching .cpp files implement game-side profile surfacing, separate bridge
slabs/tunnel floors/OSM-derived highway roofs, and configurable Gaussian terrain
smoothing. `WorldTerrain` retains an immutable raw-height cache and publishes
the processed level-0 heights to both physics and rendering. Geometry derives
from immutable elev.base; structural publication has a separate short lock.
`Session::sync_road_decks` installs nearby mesh colliders behind the gate through `rg::DeckInstaller` (S1: `deck_installer.h/.cpp`, shapes built on two own threads via physics_sim R2 `World::create_shape`, installed with `create_body(desc, handle)` in fixed (way_id, start, end) order, decks 600 m beyond the required radius prefetched, `kDeckInstallBudget` 16 per attempt, `SessionConfig::legacy_deck_install` = the old sim-thread build kept as the reference/A-B; `Session::deck_install_stats()`; tests `test_deck_installer.cpp` [deck_installer], hidden `test_deck_install_perf.cpp` [.][realdata][perf][s1]);
`RgTerrainView::sync_road_decks` owns independently rebased structural RIDs.
Settings and reconstruction bounds are in `docs/road_surfaces.md`.
**S1 (2026-10-06):** pins physics_sim 606c428 (surface classes, N2O N2-N7, R2 prebuilt shapes, car_hyper_n2o) and geo2map_engine 072a321 (slice 9 carved layer; behaviour change: served bridge profiles end exactly at the stretch end, so `road_structures.cpp` `deck()` keeps the last station exactly - a float ulp had dropped way 32275368's deck). `data/surfaces/surfaces.json` has 13 surfaces: concrete, paving_stones, sett, cobblestone, gravel, compacted, sand added with the names of physics_sim's surface_conditions classes (wet grip applies); `core/include/rg/surface_kinds.h` maps every `g2m::SurfaceKind` to a name (Unknown -> asphalt, Water -> low_mu placeholder); values are dry [AS] numbers whose citations are UNVERIFIED (notes in each entry's source), test `test_surface_kinds.cpp` [surfaces][s1]. Nitrous: catalog entries `car_sedan_gen_n2o` and `car_hyper_n2o` (kW/Nm/kg only), `core/include/rg/nitrous_info.h` (`nitrous_info`, `nitrous_hud`: state off/ARMED/SPRAYING/PURGE/CUT, bottle kg/bar, flow g/s, kit fuel), `RgSimulation.get_vehicle_nitrous`, HUD line in `hud.gd`, action `toggle_nitrous` (pad Y, key N); spawn/HUD test `test_nitrous_spawn.cpp` [nitrous][s1] (both catalog cars spawn, HUD off -> ARMED); the armed-vs-unarmed gain measurement is postponed (PLAN.md). Known physics caveat (physics_sim, not fixed here): with spray the hyper's boost can overshoot 1.8 bar to 2.0-2.4 bar and its turbo can sit on the rotor speed cap at 6000-7000 rpm.

`game/scripts/b8_contact_smoke.gd` is a SceneTree test launched with
`--script res://scripts/b8_contact_smoke.gd -- --drive --audio=godot`;
`b8_structure_shots.gd` captures seven owner-reported crossings. Unit regressions
cover smoothing halos/NoData/config bounds and local real-data road/collision
acceptance. `last_drive.json` in the existing Godot user-data directory records
session/UTM positions, build and status; the HUD/native build info reports the
source revision. Source libraries remain unchanged.

Current physics pin: 7d5316f (external/physics_sim submodule, bumped 2026-10-05 from 9e94f0f, which was bumped from 976e8d7; supersedes upgrade notes above). 9e94f0f -> 7d5316f brings weather W6 (water film, wet grip as a factor on SurfaceTable lambda_mu, tyre profiles): a no-op while no physics_sim environment with surface conditions is installed, and racing_game installs none; no source change needed, hash_check vehicle_step_steer state_hash 0xbab69b300eba41a3 unchanged. OSM building presentation and future mixed asset/imagery approach: docs/buildings.md.

Aerodynamics: docs/aerodynamics.md; configuration data/world/environment.json. Native surfaces, drafting wakes, ground effect and fan energy are owned by physics_sim; game Session supplies weather and active-wing policy, with latched HUD/art/bookmark telemetry.

Hypercar engine physical calibration: docs/hypercar_engine.md and external/physics_sim/data/engines/HYPER_ONE1.md. Fixed geometry/boost/E5; generated net 1 MW at 7500 RPM, game mass remains 1500 kg.

NPC truck: core/npc_truck plus Session stepping-thread collider/wake; background profile-route worker; game/scripts/npc_truck.gd procedural visuals. Controls and limits: docs/npc_truck.md.


NPC traffic: core/include/rg/npc_traffic.h and core/src/npc_traffic.cpp own public
road/speed/destination policies and background graph planning. Session owns the
physics-thread actor lifecycle, collision bodies and combined wakes. Adapter
get_traffic_state publishes frame-latched actors; game/scripts/npc_traffic.gd owns
presentation and F7 configuration. WorldTerrain::source_osm_tile is blocking,
background-only source access. Specification/current scope: docs/npc_traffic.md.
Stuck-NPC detector and fixes (2026-10-05): core/include/rg/traffic_stuck.h + core/src/traffic_stuck.cpp (stuck = speed < 0.5 m/s for > 5 s, cause = binding cap, wait-chain roots; `RG_TRAFFIC_STUCK` / `RG_TRAFFIC_STUCK_SUMMARY` logs), headless harness tools/traffic_probe (output in out/traffic_stuck/), results and remaining limits in docs/npc_traffic.md "Stuck NPCs".

Game server (plan only, nothing built): docs/game_server.md - authority model, protocol, interest management, deployment on the estate, increments S1-S9 and the required physics_sim seams; section 16.1 holds the owner's binding answers (30 Hz snapshots, environment time continues after a restart, remote cars silent apart from tyre audio, all other questions at the recommended defaults). Vault RACE-009.

G3/R3 consumer design (plan only, nothing built): docs/g3_consumer_design.md - geo2map pin bump strategy (bump early, delete only the game's profile completion), the carve switch delete map with file/line anchors, G3-E ribbons/junctions/interim markings draped on the Jolt-diagonal physics surface via an RgRoadView under the shared 0.8 ms upload budget, bridge decks from served profiles with deterministic bounded install (physics_sim R2 for off-tick shapes), the G3-F/R3 route harness (20-junction route, Fz residual bound calibrated on a 2 cm step, surface check against carve_footprint, repro bundles), slices S1-S14 and the requests to geo2map/physics_sim.
