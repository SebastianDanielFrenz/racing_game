# racing_game — architecture map

Pure map of what exists and how to build/test/run it. Rationale and design
decisions live in chat/task reports, not here (this repo does not use the
knowledge vault workflow other repos on this machine use - see the R0 task
report for the full decision/pitfall list). Design/phase plan: `PLAN.md`.

## Layout

```
racing_game/
  CMakeLists.txt, CMakePresets.json   top-level build config
  README.md                           new-machine setup (clone, setup.cmd, map data)
  setup.cmd                           one-step machine setup: wraps tools/setup_dev_env.ps1 (winget toolchain install, submodules, map-data report; -CheckOnly/-WithSiblings/-SkipPython)
  CLAUDE.md                           this file
  PLAN.md                             design and phase plan
  docs/unreal_port.md                 (2026-10-14) Unreal Engine 5 port plan: Godot-side audit (presentation / leaked logic / engine services, per file), leak list with file:line, UE module layout, ISO->UE frame mapping, LWC vs floating origin, upload, audio, input, UI, materials, exceptions/RTTI, milestones, day-1 checklist, owner questions
  .gitmodules                         external/physics_sim + external/geo2map_engine submodule pins; URLs are the GitHub repos (this machine's .git/config keeps the local S:\claude_code\<repo> URLs - never `git submodule sync` here)
  external/
    physics_sim/                      git submodule, READ-ONLY from this repo (another session owns it), pinned by commit - see "physics_sim submodule" below
    geo2map_engine/                    git submodule, READ-ONLY from this repo (another session owns it), pinned by commit - see "geo2map_engine submodule" below
  core/
    include/rg/
      session.h                       rg::Session, SessionConfig (optional `terrain`, optional `startup`), FrameSnapshot, WheelSnapshot, StreamingStatus, kControlChannelNames[]/kControlChannelCount; R9: StartupProgress/SessionCancelled, set_vehicle_control, request_relocate/request_reset_to_spawn, make_session(); R9b: set_followed_vehicle/followed_vehicle/followed_loss (drone-follow second physics interest point), kNpcTruckVehicleId, StreamingStatus followed_id/followed_lost/followed_lost_id/interest_points - see "Session terrain mode (R4)" and "Player modes and world switch (R9)" below
      player_mode.h                   rg::PlayerMode/ModeRules/rules_for(mode, drone_target)/PlayerModeMachine (incl. set_drone_target)/DroneCandidate/next_drone_target/unattended_controls (R9/R9b, engine-neutral mode state machine) - see "Player modes and world switch (R9)" below
      fpv_drone.h                     (FPV drone, engine-neutral, no Jolt body) rg::FpvDrone: kinematic flyer - six-axis input in the yaw frame (right/up/forward), yaw and pitch rates, max speed, accel/decel limits, ground clamp through an `FpvGroundProbe` (Session casts a World ray down; no ground loaded = height held), no roll/wind/stabilisation; FpvDroneConfig loaded from data/controls/fpv_drone.json (`flight` block, rg.fpv_drone/1, strict) - see "FPV drone" below
      fpv_drone_sound.h               rg::FpvRotorVoice/FpvAudioConfig: procedural quadcopter rotor buzz (4 detuned rotors, blade-pass harmonics + air noise; idle hum, pitch and level rise with FpvDroneState::activity), `audio` block of the same json; bound as RgFpvSound (godot_ext/src/rg_fpv_sound.*), fed by game/scripts/fpv_audio.gd
      walker.h                        (R9c, engine-neutral, no Jolt/Godot type) rg::WalkerController (kinematic capsule body in the ps::World, own collision with collide_shape_into/ray_cast: walk 1.4 / run 5 m/s, gravity, jump, 15 cm step-up, 45 deg slope limit, snap-down 0.3 m, hold when no ground is loaded), WalkerConfig/WalkerInput/WalkerState, OrientedRect + vehicle_footprint() (the own car's wheel/chassis footprint the walker is blocked by - the thin physical chassis box is ignored), within_enter_range(), select_spawn_spot() (driver side, passenger side, rear, front, above the roof) - see "On foot (R9c)" below
      terrain_mode.h                  rg::TerrainModeConfig (SessionConfig::terrain), rg::HeightTileSharedFetch (g2m::phys::IHeightTileFetch over a HeightTileFetchFn - WorldTerrain::height_tile_shared), make_terrain_mode(WorldConfig, WorldTerrain) + a pure (WorldConfig, SessionFrame, fetch) overload
      drive_script.h                  rg::DriveScript (header-only, Godot-free): tick-indexed sample-and-hold control events in DRIVE ticks (since spawn) plus an optional per-tick controller hook (the R5 autopilot's extension point); Session::set_drive_script
      fixed_rate_loop.h               rg::FixedRateLoop: fixed-rate wall-clock loop around a bool try_step() (frozen tick resyncs the deadline - no catch-up burst), LoopStats; runs Session::start()'s ticks
      world_config.h                  rg::WorldConfig, rg::load_world_config() - see "World config" below
      render_focus.h                  (2026-10-13, engine-neutral) rg::RenderFocusTracker/RenderFocusParams/rig_leads_focus: velocity-led render focus p + v_hat*min(|v|*1 s, 150 m), smoothed (tau 0.35 s); applied only while the active camera follows the car (chase/bumper/cockpit/orbit/xr_cockpit)
      terrain_stream.h                (2026-10-13) upload-throughput helpers: adaptive_upload_budget_ms (0.8 ms up to ~3 ms with the backlog), chunk_time_to_reach_s/sort_adds_by_urgency (finest level first, then time to reach), lod_coverage_ahead/grid_coverage_ahead (l0..l3_ahead_m instrumentation), chunk_surface_height/SampleWindow (visible-vs-physics ground check), reselect_distance_for_speed (reselect 64 m at >= 100 m/s)
      synthetic_terrain.h             (C9, 2026-10-15) rg::SyntheticTerrain: rolling-hill L0-L6 height tiles built in memory (thread-safe, lazy) + a TerrainViewSource over them (game LOD defaults, chunk colour hook) - the no-map-store scene of RgTerrainView.load_synthetic_preview / terrain_upload_bench.gd
      terrain_view_streamer.h         rg::TerrainViewStreamer / TerrainViewDiff (R2.2 R8): aliases of geo2map g2m::world::ViewStreamer / ViewDiff since terrain migration phase 3b (render LOD follows a focus point - background reselect + build, key diff {added, removed, replaced}, adapter ordering contract in g2m/world/view_streamer.h); see "Render LOD streaming (R8)" below
      terrain_colours.h               (phase 3b) the game's terrain chunk colours: TerrainColourTable by class id (make_terrain_colour_table from RoadSurfaceMap: asphalt grey, dirt brown), chunk_vertex_colors (+ the height/slope ramp for every other class), apply_debug_tint (RG_TERRAIN_TINT), chunk_colour_hook = the ViewSource colour hook; test_terrain_colours.cpp [terrain_colours] pins the bytes recorded at 5a3baed
      road_classes.h                  rg::RoadSurfaceMap only (physics.road_surfaces names -> render colours; leaves in phase 7). The rest (ClassLookup/RoadSegmentsFn, road_raster_params_for_level, road_class_lattice, src_osm_tiles_for_chunk, rasterize_chunk_road_classes) lives in geo2map g2m::world (chunk_road_classes.h) since terrain migration phase 1 - see "Visible roads (R-2)" below
      route_check.h                   rg::Route + load_route() ("rg.route/1", exception-free, optional "criteria" object -> RouteCriteria), apply_route_criteria() (RouteCheckParams defaults < route file < CLI, field by field), phys_tile_index()/count_seam_crossings() (255 m physics grid, origin 0.5), sample_l0_height() (bilinear on the L0 cell-centre lattice through any tile lookup), check_route() (length, 10 m-window max/p99 grade plus a steep_stretches list above grade_report_threshold, seam crossings, elevation range, NoData, corner radius = circle through the points +-corner_window_m (10 m) along the route at every 1 m sample (waypoint-density independent) plus a tight_corners list below corner_report_radius_m, start offset vs RouteCheckParams' R5 criteria), route_matches_world(), check_route_on_world() (the same over WorldTerrain::height_tile_shared) - used by tools/route_check and the [realdata] test
      tyre_sound.h                    (perf, 2026-10-15) rg::TyreVoice/TyreSoundBank: procedural tyre roll hiss + skid squeal, interleaved per-wheel render (port of the former tyre_sound_model.gd + its GDScript mixing loop, was 2-3 ms/frame on the main thread); bound as RgTyreSound (godot_ext/src/rg_tyre_sound.*: setup/update_wheel/render, static channel_to_stereo/scaled for the Godot fallback audio path); test_tyre_sound.cpp [tyre_sound]
      biquad.h, wind_noise.h (core/src/wind_noise.cpp)  (wind noise, 2026-10-19) rg::BandPass (RBJ band-pass, unit-power norm) and rg::WindNoise: aero wind layer from the AIRSPEED (RgSimulation::get_airspeed_mps = aero airspeed, else |v - env wind|), A = rms_at_reference*(U/ref)^(50/20), centre ~ U (Strouhal), gust modulation; data/controls/wind_noise.json (racing_game.wind_noise/1, strict parse, max_speed >= 400 km/h); bound as RgWindNoise (godot_ext/src/rg_wind_noise.*), mixed into the tyre PCM by vehicle_audio.gd (rides on the tyre volume)
      tyre_sound.h (rewrite 2026-10-19)  the owner heard the tyre layer as a switch: old law gain 0.25*(v/ref) capped at 1.0 (flat above 200 km/h) and playback rate clamped to [0.5,2] (frozen below 25 / above 100 km/h). Now: CPX-style gain 35 dB/decade up to 500 km/h, rate (v/50)^0.35, plus procedural hiss (centre ~ v^0.3) and a tread tone at exactly tread_speed/pitch (wheel angular speed x blocks per rev [AS]); per-surface dB table in data/controls/tyre_sound.json; RgTyreSound::setup takes the config JSON text and returns an error string, get_roll_state(); tests test_wind_noise.cpp T1-T9 W1-W9 [wind_noise][tyre_sound] (3 recorded sabotages in its header), hidden diag [wind_noise_sweep] writes out/audio_sweep/*.wav + sweep_parameters.csv; env RG_AUDIO_PARAM_LOG=<csv> logs per-frame audio parameters from vehicle_audio.gd
      credits.h                       (R5) rg::Credits: data/credits.json (rg.credits/1) loaded exception-free, entries (name, licence, required attribution text, where shown), `attribution_line()` (the one line shown on the boot splash, the loading screen and the credits screen), sections for the credits screen; test_credits checks every data source of the geo2map README table is covered (reads the sibling geo2map_engine README when the submodule pin predates it, SKIP when absent)
      settings.h                      (R5) rg::Settings: `settings_schema()` (graphics.window_mode/vsync/max_fps, camera.fov_deg, audio.master/engine/tyre_volume, map_data.store_dir; each with type, range, default, applies_live), versioned `rg.settings/1` JSON file (settings.json in the user dir), load report (missing/parse error/migrated/unknown + invalid keys), set/get/reset with validation + clamping, atomic save; only settings that really apply are in the schema
      lod_settings.h                  (R4map, 2026-10-14) in-game LOD settings: data/settings/lod_settings.json (rg.lod_settings/1: lod.view_distance_m, lod.terrain_detail, lod.road_detail (presets low/standard/high = g2m::RenderRoadLod by road class name), lod.map_detail; ranges, defaults, decimals) installed into the settings schema via `extend_settings_schema` (settings.h); `lod_choice_from`, `terrain_lod_params` (range0_m x detail, max_distance_m), `road_lod_for`, `map_config_for` (MapConfig::detail_scale), `changed_road_lod_levels`; every default = the behaviour before (test_lod_settings.cpp [lod_settings])
      route_planner.h                 (R4map) rg::RoutePlanner: region routing graph from level-2 roads.graph tiles (g2m_route assemble_region_graph + build_region_routing, ALT landmarks) on its own worker, coalesced route requests (heading-aware snap, A*+ALT, route_polyline, build_guidance) -> PlannedRoute (session metres, manoeuvres); `route_progress` (along/off-route/next manoeuvres for the HUD); `tiles_in_square`; config data/world/route_planner.json (rg.route_planner/1). No turn restrictions yet (roads.graph has none, g2m slice 3)
      map_view.h                      (R4map) rg::MapView: minimap geometry through g2m::route::MapTileCache on a worker (select_map_lod, map tiles assembled from level-2 roads.graph tiles, flattened to session-metre polylines), `set_config` = the map-detail setting; tests test_route_planner.cpp [route_planner][map_view] on a synthetic grid tiled with select_road_graph_tile
      spawn_presets.h                 (R5) rg::SpawnPresets (data/world/spawn_presets.json, rg.spawn_presets/1: id, label, session-frame x/y, yaw) + `build_spawn_choices()` (presets, "default_spawn", "last_position" from user_dir/last_drive.json, "address", "flat"; each with available/unavailable_reason; a preset outside the loaded store is greyed out)
      shell_flow.h                    (R5) rg::ShellFlow: the screen state machine (Boot, MainMenu, SpawnPicker, Loading, Drive, Pause, Settings, Credits, Quit) - events in (boot_finished, menu_item, spawn_picked, back, load_ready/failed/cancelled, pause_toggle, direct_start), a Transition out with ACTIONS (show_screen always last; load_world, unload_world, set_paused, reset_car, save_settings, quit); GDScript only carries the actions out
      vehicle_catalog.h               (R6) rg::VehicleCatalog: data/vehicles/catalog.json (rg.vehicle_catalog/1) - drivable cars (id, title, model, sim name, default paint/rim, chassis mass/box interim until physics_sim carries it), stats read from the vehicle/engine/gearbox files (power/torque from the engine curves or cycle block, mass, layout, driven wheels, gear count)
      tyre_spec.h                     (2026-10-13) rg::TyreSpec/TyreFamilies/TyreAxleFitment: universal tyres (compound + size), `TyreFamilies::make` builds the ps TyreDesc in memory from data/tyres/families.json, fitment checks + size lists, `migrate_tyre_id` for pre-universal names (see "Universal tyres" in the Setups notes)
      vehicle_load.h                  (2026-10-13) rg::load_vehicle: ps::io::load_vehicle_json with VehicleLoadOptions::tyres built from a vehicle file declares as per-wheel `tyre_spec` (use it for every game vehicle file)
      vehicle_setup.h                 (R6) rg::SetupModel/VehicleSetup/materialise_setup: data/vehicles/setup_options.json (rg.vehicle_setup_options/1, 22 whitelisted options with ranges and compatible engine/turbo file choices), RFC 7396 merge patch onto the vehicle file and the referenced engine/gearbox/tyre files, materialisation into a work dir and validation with ps::io::load_vehicle_json (loader message returned, an invalid setup cannot be saved); ScopedWorkDir; electric drive (PR #55): ev_motor_scale (every electric_motor torque/power/regen/mass x0.5-2) and ev_battery_pack, kind scale_preset (each preset scales the leaves of pointer pattern k by factors[k]; cards with computed figures from the optional "display" list; five [AS] packs on capacity/discharge/charge/mass) - tests test_ev_part_upgrades.cpp [ev]
      slider_map.h                    (2026-10-13) rg::SliderSpec/slider_position/slider_value: the data-driven slider mapping (log|linear, abs_min/abs_max in a unit) of a scale option in setup_options.json ("slider" block; rev limiter 2000-20000 rpm log, boost 0.1-30 bar log); vehicle_setup.h NumberEntry/number_entry/entry_from_displayed = the value as the player reads it, used by the garage value boxes (shell_ui.gd, game/scripts/slider_box.gd for the other sliders); tests test_slider_map, test_rev_limit_extremes [long]
      garage_set.h                    (R6) rg::GarageSet (data/garage/garage_set.json, rg.garage_set/1: room, turntable, lights, camera areas) + the garage camera (smooth moves between overview/wheels/engine bay/rear, framing from the car's bounds), all Godot-free
      garage.h                        (R6) rg::Garage: selection + saved setup per vehicle (<user dir>/garage/), the edit session (working copy, validation on every change, save only while valid), prepare_drive() (materialises the saved setup into <work_root>/drive_<id>_<hash>, names everything a world load needs), cleanup()
      engine_store.h                  (engine garage G1) rg::EngineStore: data/engines/engine_catalog.json (rg.engine_catalog/1: six default engines with class curve/mapped/cycle, family, image, mass + source, aliases sedan_i4/sedan_i6, measured peaks of the cycle engines) plus the player's user engines (<user dir>/garage/engines/<uid>/{meta.json, engine.json = {"extends": <default>, ...merge patch}}, validated with the physics loader before writing) in ONE list (user engine directly after its default); read_engine_facts() (idle, limiter, fuels, curve peaks via load_engine_json generate=false, cached). Impl core/src/engine_store.cpp
      engine_swap.h                   (engine garage G1) rg::derive_swap_package (fuel kept or changed D1, clutch = 1.5 x peak torque if lower, auto-clutch and auto-shift points from idle/limiter, blip zeroed on a mechanical throttle, slot overrides and a nitrous kit dropped where they no longer fit, the cooling_system dropped when the new engine has no thermal data (EngineFacts::has_thermal, SwapPackage::drop_cooling_system), chassis-mass delta) and rg::check_fit (engine envelope vs the catalog entry's engine_bay_envelope: the ONLY thing that refuses an engine swap); applied by vehicle_setup.cpp's apply_swap. Impl core/src/engine_swap.cpp
      limit_advisor.h                 (garage setup check, 2026-10-13) rg::LimitAdvisor + build_advice + AdvisorConfig (data/garage/advisor.json, rg.garage_advisor/1: debounce, sweep grid, per-kind min severity, merge gap, text templates, variants, max_severity caps): the pending setup is materialised/loaded like a drive (Sketch maps), swept with ps::drivetrain::run_dyno_sweep + check_driveline_limits, findings grouped per kind/part field with merged rpm ranges, Near/Limiting/Exceeded = advice/warning/critical; wastegate kinds get an on-demand solve_remedy ("Find fix"), Closed remedies show at once; never clamps a setup value. Stage-map coverage: `AdviceInput::build_issues` (physics `check_turbo_stage_coverage` with the policy, severity from physics) renders the `build_rules.TurboStageMapCoverage` text of advisor.json ({coverage} = share of the engine's air flow the stage map covers; variant `beyond_map` for Limiting), `loader_warnings` passes the engine loader's `EngineMapOptions::warnings_out` lines through as one "LoaderNotice" item (`ignore_containing` drops the stage-map advisory, already a build rule). RgGarage.get_advice/solve_advice, shell_ui.gd SETUP CHECK panel. Impl core/src/limit_advisor.cpp, tests tests/unit/test_limit_advisor.cpp, docs docs/garage_ui.md
      engine_internals.h              (2026-10-16, PR #21) rg::EngineInternalsConfig/load_engine_internals (data/engines/engine_internals.json, rg.engine_internals/1: connecting rods, pistons, crankshaft, valve springs with material grades, [AS] limits and mass factors), describe_engine_internals (limits for the installed engine: stock >= stock_margin x factory, upgrades >= stock, downgrade grades (cast-steel rods, low-silicon pistons, lightweight cast crank, weak single springs) may rate below stock and then warn at the stock limiter; rods are rated by tensile load F = m_recip x S/2 x w^2 x (1+lambda) with the installed piston's mass, every bottom-end part reduced to its limit_rpm, weakest wins; valve-float rpm); SetupPart fits/fit_reason + visible_parts/hidden_parts_reason (upgrade lists hide what the car cannot fit, hidden setting garage.show_all_upgrades), engine_internals_parts (mass difference to stock under engine.internals); garage options engine_rods/engine_pistons/engine_crankshaft/engine_valve_springs (catalog "engine_internals", no engine file edit), advisor MeanPistonSpeed (weakest part) / ValveFloat, AdviceItem::damage_part. Cylinder bore (BoreConfig, engine_internals.json "bore"): option engine_bore (internal "cylinder_bore", kCylinderBoreKind) with steps +0.5/+1.0/big bore +3.0 mm, wall = pitch_factor 1.09 x factory bore - new bore [AS] (refused below 4 mm, fits=false; CylinderWall warns below 5 mm, damage_part cylinder_block), bore_geometry/bore_refusal/bored_engine_facts (EngineFacts::factory_* keep the unbored figures); writes the engine file's bore_mm + computed compression ratio (head volume kept) through the merge patch, pistons of the new size (mass x (b/b0)^3, so the rod-limited rpm drops), engine.internals.cylinder_block mass node, a turbo kit sizes for the bored displacement. Impl core/src/engine_internals.cpp, tests tests/unit/test_engine_internals.cpp [engine_internals], tests/unit/test_engine_bore.cpp [bore], docs docs/garage_ui.md "Damage model data"
      cooling_upgrades.h              (2026-10-16, PR #21 comment) rg::CoolingUpgradeConfig/load_cooling_upgrades (data/cooling_systems/cooling_upgrades.json, rg.cooling_upgrades/1: radiator, fan, oil_cooler, block_cooling grades as factors over the car's effective cooling_system / engine thermal values, mass per grade as children of the part-mass node engine.cooling, `check` block of the setup check), build_cooling (merge patches for the cooling_system and engine thermal files, written by materialise_setup), steady_thermal (the physics model's steady state, validated against a full-throttle run: 117.3 vs 117.1 C), transient_thermal (the same model's time history from the warm spawn state: the setup check grades the time to the limit, PR #28 owner answer option C; advisor.json thermal_time.critical_within_s 60 s = warning vs critical), setup options cooling_radiator/_fan/_oil_cooler/_block_cooling (catalog `cooling_upgrades`), advisor rules CoolantTemperature/OilTemperature (set_thermal_limits). Tests tests/unit/test_cooling_upgrades.cpp [cooling]; docs/garage_ui.md "Cooling system upgrades"
      damage.h                        (2026-10-16, owner on PR #28 / #33) rg::DamageConfig/load_damage_config (data/vehicles/damage.json, rg.damage/2: parts radiator / radiator_fan / head_gasket / crankshaft / pistons, each with a health-to-function effect radiator_ua | fan_flow | friction and scale_at_zero, optional seize_below; rules = coolant/oil past an absolute or overheat-relative safe value (optional dwell, only_while_fan_on, only_while_running) wearing at a rate read off a curve over the excess, wear_rate_per_s), DamageState (part id -> health 0..1, <user dir>/garage/damage/<vehicle id>.json rg.damage_state/1, healthy = no file, unknown ids kept), DamageModel (one tick on the Session stepping thread after World::step: SessionConfig::damage incl. enabled = difficulty.mechanical_damage, Session::damage_report/request_damage_repair/set_damage_enabled; damage_scales -> physics_sim World::set_engine_cooling_capacity / set_engine_wear, never called while healthy, so hashes are unchanged; seized_part -> the Session holds ignition and starter at 0 until a repair), damage_view/damage_warning_line (HUD). Garage::damage_state/repair_damage, DriveSelection::damage_config_path/damage_state_path; ShellFlow pause item repair_car + RepairChosen on VehicleSelect/ChangeCar -> action RepairCar; RgGarage.get_damage/repair_damage, RgSimulation.get_damage_state/repair_damage("" = running car)/set_damage_enabled; hud.gd line, car_browser_ui.gd Repair, shell_ui.gd Engine > Cooling panel. Tests test_damage.cpp, test_damage_engine.cpp [damage] (hidden [.][damage][long] owner scenario); docs/damage_model.md
      turbo_map_extension.h           (2026-10-14) rg::extend_turbo_maps / TurboMapWorker / TurboMapExtensionConfig (data/engines/turbo_map_extension.json, rg.turbo_map_extension/1: physics ExtensionPolicy numbers, cache subdir, choke search): when ps::drivetrain::turbo_stage_demand finds the engine's air demand outside the turbo's stage map, the map (found from the vehicle file: forced_induction/default_forced_induction -> configuration "turbo" -> hardware "performance_map") is regenerated around the demand (meanline::extend_sampling_for_demand + generate) and cached as <engine map cache dir>/turbo_stage_maps/turbo_stage_<meanline cache_key>.json (atomic write; demand is read against the map FILE so the key is stable and a second load, or a second pass, is a cache hit); the VehicleDesc gets the extended maps. Runs on the caller's thread, so only from workers: the garage advisor/dyno threads (SetupContext::turbo_map_extension, loaded by Garage::open) and Session start-up (SessionConfig::turbo_map_extension, set by RgSimulation from the vehicle override turbo_map_extension_path = DriveSelection::turbo_map_extension_path); TurboMapWorker runs one on its own thread. A demand beyond the compressor's choke stays uncovered: TurboExtensionResult::choke gives the per-turbo choke flow (AdviceInput::turbo_choke -> advisor.json build_rules.TurboStageMapCoverage variant "choke": "Compressor chokes at this boost", "passes at most X kg/s, the engine wants Y kg/s"; accepted for the N2O hypercar at 1.8 bar: 0.5185 of 0.5336 kg/s per turbo). Impl core/src/turbo_map_extension.cpp, tests tests/unit/test_turbo_map_extension.cpp [turbo_map]
      turbo_kit.h                     (2026-10-15) rg::TurboKitConfig/load_turbo_kit_config (data/engines/turbo_kits.json, rg.turbo_kits/1: anchor turbo, sizing, conversion, envelope, mass, limits, charge piping plastic/aluminium/steel; kits street_single/sport_single/sport_twin/quad = count 1-4 + starting boost on ANY eligible engine), turbo_kit_refusal (torque curve, turbocharged already, no cycle, lift family, rotary), size_turbo_kit (air demand at the limiter / N turbos -> anchor scaled at constant tip speed; unit envelope and masses), turbo_kit_envelope_mm (engine + kit box for rg::check_fit_size, engine_swap.h), describe_turbo_kit -> TurboKitBuild (parts with [AS] masses and damage-model limits), turbo_kit_parts (part-mass tree under the engine), write_turbo_kit, turbo_kit_grid_kpa. Setup options `turbo_kit` (re-sized for the final boost slider target; bay fit refused with the dimension) and `charge_piping`; CompiledSetup::turbo_kit; advisor build rules ChargePipingPressure/TurboShaftSpeed/MeanPistonSpeed (warnings only). Impl core/src/turbo_kit.cpp, tests tests/unit/test_turbo_kit.cpp [turbo_kit], docs/garage_ui.md "Turbo kits" (sizing logic, Damage model data)
      number_format.h                 (2026-10-19, upgrade hints) rg::display_decimals/format_number/format_signed_number: the ONE number formatter of every badge (|x| >= 100 -> 0 decimals, >= 10 -> at most 1, >= 1 -> at most 2, >= 0.1 -> at most 3, and so on, capped by the quantity's max_decimals; trailing zeros dropped, "+5 kg"); tests tests/unit/test_number_format.cpp (every boundary). Impl core/src/number_format.cpp
      upgrade_badges.h                (2026-10-19, rg.upgrade_badges/2, data/garage/upgrade_badges.json) rg::UpgradeBadgeConfig/load_upgrade_badge_config (rejects /1, the old `decimals` key, a quantity without higher_is_better, unknown keys), upgrade_badges(cfg, category, candidate, installed, pending*) = the badges of a tile (candidate whole-car figure minus the CURRENT one, direction decides green/red, min_change skips, max_badges 3, relative quantities as percent, a pending quantity as the "..." placeholder), upgrade_category_for (option id, else its group), option_part_badges (weight of a part-choice option from its own part masses). Every reference condition (tyre surface/speed, dyno map quality/rpm grid/debounce, thermal ambient/air speed) lives in the file. Impl core/src/upgrade_badges.cpp
      upgrade_hints.h                 (2026-10-19, upgrade hints; owner: a hint is the whole car with the candidate swapped in minus the whole car as configured NOW) rg::compute_setup_figures (weight = part-mass tree total, rpm tolerance = weakest internal's limit_rpm, grip/rolling resistance from physics_sim's Pacejka model at the static corner loads (static_corner_loads: smallest-norm loads with the weight and no moment) on the reference surface, battery kWh, peak power/torque from run_dyno_sweep on Sketch maps and steady coolant/oil temperature from steady_thermal at the peak-power point; N2O power measured with the kit armed) and rg::UpgradeHints (one background thread, background-priority JobSystem, cache per (whole setup, nitrous flag), cheap weight/rpm figures at once and the sweeps after the debounce, newest request wins, set_base with another current setup or cancel() cancels the running work, a placeholder until ready). Garage::candidate_setup replays a tile's edits on a copy of the working setup (set_option's rules, nothing loaded). Godot: RgGarage.get_hints/wait_for_hints/cancel_hints/get_hint_stats; shell_ui.gd draws the chips (pending = grey "...", polled every 0.25 s). Tests tests/unit/test_upgrade_hints.cpp [upgrade_hints] (6 recorded sabotages in its header); garage_shots.gd --hint-shots-only. Impl core/src/upgrade_hints.cpp
      gearbox_catalog.h               (gearbox swap, 2026-10-13) rg::GearboxCatalog: data/gearboxes/gearbox_catalog.json (rg.gearbox_catalog/1: id, name, kind manual|sequential|automatic|dual_clutch|cvt, focus economy|touring|performance|race, gear_type helical|straight_cut, synchro cone|dog, description, rating_nm = the maximum engine torque the box takes, file); gears/first/top ratio are READ from the physics_sim.gearbox/1 file, kind must match the file's type. check_gearbox_rating -> "gearbox: rated 450 N*m, engine makes 1371 N*m (Name)"; sections() = Manual (manual + sequential) / Automatic / Dual-clutch / CVT, CVT (2026-10-16): cvt_converter / cvt_clutch rows (rated 250 N*m; gears = the S-mode steps, first/top = ratio_low/ratio_high), install_clutch_to_clutch adds the automatic_tcu "cvt" ratio law (derive_automatic_tcu_points cvt_target_rpm / cvt_min_rpm), tests [cvt] in test_automatic_gearbox_swap.cpp. Automatic + dual-clutch (2026-10-16): auto_8at, heavy_8at, dct_7, hyper_7dct rows; installing one (vehicle_setup.cpp install_clutch_to_clutch) removes the clutch, turns the manual_tcu into an automatic_tcu with derive_automatic_tcu_points (schedule from idle/limiter), fits a park lock, drops drive_modes; Garage::drop_unavailable_values(true) drops clutch/shift-assist values; DriverActions selector_shift/selector_label + actions select_drive/select_reverse/select_neutral/transmission_mode (channel transmission.mode), RgDriverActions.selector_shift; tests tests/unit/test_automatic_gearbox_swap.cpp [automatic] (creep + drive from rest through the gears). The setup option gearbox_install (catalog "gearboxes", group Drivetrain, applied after engine_install) repoints the vehicle's gearbox component ref; the offered list = every manual/sequential box whose rating covers the INSTALLED engine's peak torque (EngineStore::peaks, N2O shot and boost upgrades not counted) plus the car's own box under the id "stock"; an engine swap that leaves a catalogued box too weak adds a notice (the stock box is never rejected); a new box clears gear_ratios (the factors are per gear of the old box). mass_kg = the box's own node in the part-mass tree (rg/part_mass.h). Impl core/src/gearbox_catalog.cpp, tests tests/unit/test_gearbox_swap.cpp
      strip_visual.h                  (PR #22 follow-up) rg::StripVisualTable/StripRule/StripModel + load_strip_visual/parse_strip_visual/model_stem: data/vehicles/strip_visual.json (rg.strip_visual/1) maps a fitted weight-reduction part (slot + part id) to the model nodes a car hides, per model (glb stem); `hidden_nodes(stem, {slot: part})`; `not_separate` explains slots with no node; `after_regeneration` rules name nodes tools/cargen produces but the committed glb lacks. Garage::working_hidden_nodes / DriveSelection::hidden_nodes / RgGarage `hidden_nodes`; model_options.gd apply(..., hidden) toggles visibility (drive + cockpit + garage); impl core/src/strip_visual.cpp, test tests/unit/test_strip_visual.cpp [strip_visual], game/scripts/cockpit_test.gd, docs/garage_ui.md "Visual removal"
      part_mass.h                     (part-mass tree, 2026-10-15, owner on PR #19) rg::PartMassTable (data/vehicles/part_masses.json, rg.part_masses/1: base part masses [AS] by component type / diff kind / electronics kind, sized parts per kg of capacity, physics_carries = parts physics simulates as own masses, empty today) + build_part_tree: body (catalog chassis.body_mass_kg) -> engine (engine catalogue mass; cooling, nitrous kit/bottle, PartAttachment extras), drivetrain (clutch, gearbox from the gearbox catalogue's mass_kg, transfer case, front/rear axle + half shafts, propshaft where the driven axle is at the far end from the engine bay, layout_conversion when the driven axles differ from the STOCK file: mounting hardware + transfer drive (2WD) + driveline along the car where the new layout has no more propshafts than stock - the owner's layout-swap penalty), running_gear (steering, corners: vehicle-file wheel_mass, brake, suspension), fuel_system, electronics; cumulative = own + children; chassis_kg = total - physics-carried (to the gram) is the drive's chassis mass (Garage::prepare_drive); PR #22: body_mass_kg is an independent [AS] bare shell per car (catalog chassis.mass_kg = the stock tree's dry chassis), contents nodes fuel_system.tank.contents / engine.nitrous_kit.bottle.contents (spawn fill, carried by physics_sim contents_mass, set in every car_*.json), engine.turbo_system (factory turbos, NA engine mass in the engine catalogue; skipped when a turbo_kit extra exists), trim group from the catalog `interior` map + part_choice options (Weight reduction); Garage::stats mass_kg = the tree with the effective setup (presets included); tests test_part_mass.cpp [part_mass] (class bounds + sum invariant). mass_groups() = the garage breakdown; RgGarage.get_mass_breakdown(id), shell_ui.gd Chassis -> Weight. Impl core/src/part_mass.cpp
      driveline_swap.h                (drivetrain swap, 2026-10-15) rg::DrivelineCatalog (data/drivetrains/driveline_catalog.json, rg.driveline_catalog/1: centre_unit, layouts fwd/rwd/awd_open/awd_viscous/awd_locked (no Torsen/LSD centre: physics_sim builds those only as axle diffs; a non-open centre is made lossless), differentials open/clutch_lsd/torsen/viscous/locked with their physics kind keys), describe_driveline (layout/driven wheels/kinds/swappable - also the source of VehicleStats::layout/driven_wheels/driveline), apply_driveline (generic transform: gearbox output -> one axle diff "diff" or centre + "front"/"rear"; new axle units copy the car's own, the park pawl and other ports on the gearbox output stay; transfer case / gear pair / extra diffs = not swappable, except the battery-hybrid conversion's "conversion_reduction" gear_pair: the traction motor rides on the gearbox output shaft, so every layout and differential swap keeps it on whatever the gearbox drives; unavailable swap options carry OptionView::unavailable_reason, shown under the greyed Swaps tile; Garage::drop_driveline_values_after_conversion keeps a layout/differential chosen before a powertrain conversion when the converted car still takes it). Setup options drive_layout, diff_front, diff_rear (catalogs "drive_layouts"/"differentials", applied after gearbox_install, the transformed subtrees exempt from the whitelist like the swap package); Garage::drop_driveline_values after a layout change; Garage::stats follows the setup's layout; shell_ui.gd Swaps > Drivetrain. Internal core/src/driveline_swap_json.h. Impl core/src/driveline_swap.cpp, tests tests/unit/test_drivetrain_swap.cpp ([drivetrain_swap], the flat-road drive per layout and the World build of every car x layout are [drive])
      crt_guard.h                     rg::install_crt_guard(): debug-CRT assert/error/warn and abort() go to stderr (never a modal dialog) plus a symbolised stack trace of a CRT assertion; called by the test entry point and by the Godot extension's init (debug builds, or any run with --headless). Impl core/src/crt_guard.cpp (Windows, dbghelp); test tests/unit/test_crt_guard.cpp
      car_browser.h                   (R6c) rg::CarBrowser: the car browser model, Godot-free - BrowserCar (id, title, subtitle, body type, manufacturer, power/torque/mass/displacement, drive layout, paint/rim, model), GroupBy (body_type, drive_layout, power_band, manufacturer, none; computed category headers, power bands <100/100-199/200-399/400-699/700+ kW), SortKey (name, power, torque, mass, power_to_weight, displacement; ascending/descending; unknown values last, ties by title then id), BrowserFilter (drive layout / body type / power band, ORed within a facet, ANDed across), the grid (2-4 rows, column-major per category, sticky focus row, wrap, category jumps), scroll state and the virtualised `view(visible_columns, overscan)`, `thumbnail_key(model, paint, rim, version)`, group/sort/filter persisted through six hidden `browser.*` settings (SettingDef::shown = false)
      camera_math.h                   (R5) rg::DriveView (chase, bumper, cockpit, orbit, cinematic) with next_drive_view/toggle_cockpit_view, orbit state + `orbit_step`, bumper mount, `CinematicDirector` (roadside camera shots along `road_ahead`, falling back to the car's predicted path in the flat world and off-road): all camera math, no Godot type ; 2026-10-05: `chase_follow_offset` (the chase rig smooths the car-relative OFFSET, not the absolute position: zero lag at any speed), the cinematic director searches candidates against `ShotObstacles` (`shot_obstacles.h`; `building_footprints.h` = `BuildingFootprints`/`BuildingObstacles`, per-1 km-tile footprint prisms on a worker thread; see docs/buildings.md "Camera obstacles"), `ShotSource::ChaseFallback`; 2026-10-15: `DroneParams/DroneState/drone_follow_step/drone_zoom_step/drone_target_orbit_yaw/drone_probe_offsets/drone_clear_terrain` (DroneFollow camera, bound as `RgCameraMath.drone_*`); 2026-10-16 (leak #10): the other rigs' maths too - `ChaseParams/ChaseState/chase_follow_step`, `FreeParams/FreeState/free_activate/free_place/free_step`, `WalkerCamParams/WalkerCamState/walker_cam_step/walker_cam_aim/walker_cam_look_forward`, `OrbitRigState/orbit_rig_step` (around `orbit_step`), shared `ground_probe_offsets/clear_terrain_height/look_valid/rig_orbit_yaw`; Z-up frame like the drone, bound as `RgCameraMath.chase_*/free_*/walker_cam_*/orbit_rig_*`, the rig scripts keep only a state Dictionary; goldens against the former GDScript in test_camera_math.cpp [chase][free][walker_cam][orbit_rig]
      camera_motion.h                 (2026-10-14) rg::CameraMotion: the driver's virtual body (per-axis damped spring-mass head driven by the chassis acceleration in the car frame -> eye offset + pitch/roll) and the speed effects (shake rising with speed and suspension roughness, FOV widening); data/controls/camera_motion.json (rg.camera_motion/1, per-view gains: head motion only in cockpit/xr_cockpit, the loader refuses a VR rotation/shake/FOV gain); the setting camera.body_motion (default 1 = full) scales the head. camera_math.h also has `load_model_bounds`/`bumper_eye_from_model`: the bumper eye just ahead of the model's rig.json overall_bounds nose (or its `socket_bumper_camera`), so every model works without per-car data. Tests test_camera_motion.cpp [camera_motion][bumper]
      road_ahead.h                    (R5) rg::trace_road_ahead(): the road polyline ahead of the car from the client road data (Session's road tiles), used by the cinematic director; `FrameSnapshot::road_ahead` is filled only while `Session::set_road_ahead_wanted(true)`
      steering_assist.h               (2026-10-14, moved from adaptive_steering.gd) rg::SteeringAssist: steer input shaping (keyboard ramp, speed-dependent lock limit, counter-steer authority) from data/controls/steering.json (strict loader) and the car's wheelbase / full-lock angle (steering_geometry_from_vehicle_json); bound as RgSteeringAssist, adaptive_steering.gd forwards
      address_search.h                (2026-10-14, moved from address_teleport.gd) F10 address teleport logic: data/controls/geocoding.json loader (endpoint, max distance, result_limit, language, min_request_interval_s), "lat, lon" query, cache key, Photon URL, GeoJSON response -> matches, destination check via g2m::geo::Utm (WGS84) into the session frame, utm_to_latlon (the inverse, used by the owner-mark clipboard text: the log_marker action copies "lat, lon" and the UTM coordinates); bound as static RgAddressSearch
      sky.h                           (V1, 2026-10-14) sun (NOAA) and moon (Schlyter + parallax, illuminated fraction) position for a date/UT/lat/lon, Gregorian date helpers, EU summer time (TimeZoneRule)
      time_weather.h                  (V1) rg::TimeWeather: data/world/sky.json (rg.sky/1), TimeController (local civil clock, rate, user override), sky_lighting (sun/moon energy, sky colours, ambient by sun elevation + cloud cover), weather_icon table, the weather/time menu view-model (every preset enabled once a climate is set: set_climate), air_temp_c from rg/climate.h, TimeController::revision/ut_instant, location_from_world_config (UTM origin -> lat/lon + convergence) - see docs/visual_world_plan.md
      exposure.h                      (2026-10-15) rg::ExposureConfig/ExposureController/exposure_for_lux/night_exposure/combine_exposure: camera exposure from the sky illuminance (display = cd/m2 x key x pi / adapted lux), sky.json "exposure"; tests test_exposure.cpp [exposure]
      climate.h                       (V8, 2026-10-14) rg::ClimateConfig (data/world/climate.json, rg.climate/1): the ONE air temperature (climate_temperature_c: monthly mean + range, 14:30 solar peak, cloud damping, lapse), make_physics_climate (12 month seasons), physics_weather_preset (temperature pinned), env_clock_point, make_session_weather -> SessionConfig::weather
      weather_sync.h                  (V8) rg::WeatherSync: game clock (revision, day, drift > resync window) and weather choice/temperature drift -> Session::set_environment_clock/set_weather; session_weather_for(TimeWeather)
      crops.h                         (V6a, 2026-10-14) rg::field_look: deterministic crop per OSM farmland way (weighted rotation by hash(way id, world seed), crop by harvest year, harvest year switches at the crop's season_end_doy), growth stage by date (height, ground cover, colour, row spacing), crop=* override, fixed calendars (meadow, orchard, vineyard), field_row_angle_deg; data/world/crops.json (rg.crops/1); nothing renders it yet (Unreal port)
    src/
      session.cpp                     Session implementation - builds a ps::World by hand (flat: ground box + chassis + one vehicle; terrain: streamed G2mTerrainSource + chassis + one vehicle), never parses a scenario JSON; the terrain gate, start-up, priming, spawn rays, StreamingStatus atomics, relocation, make_session
      player_mode.cpp                 player_mode.h implementation
      walker.cpp                      walker.h implementation (uses ps::math::sincos/atan2 only)
      terrain_mode.cpp                terrain_mode.h implementation
      fixed_rate_loop.cpp             FixedRateLoop implementation
      world_config.cpp                load_world_config() implementation - strict, exception-free JSON validation (see "World config" below)
      terrain_colours.cpp             terrain_colours.h implementation (moved from world_terrain.cpp in phase 3b; terrain_view_streamer.cpp and buildings.cpp are deleted, the code is geo2map's)
      route_check.cpp                 route_check.h implementation
      surface_layers.h/.cpp           (spawn z-fighting, 2026-10-19) rg::resolve_surface_layers(RoadRibbonTile&), engine-neutral mesh arithmetic called once per fresh tile at the end of WorldTerrain::road_visual_tile_shared (after append_route_surface_mesh / append_standalone_path_mesh): the draped road-visual layers never overlap. Rank verge 0 < walkway/path 1 < unpaved route strip 2 (flags 8192) < carriageway 3 < junction patch 4 (width > 500); a lower-ranked triangle is geometrically cut (polygon clipping, attributes interpolated barycentrically) against every higher-ranked one at (nearly) the same height (kSameSurfaceHeightM 0.25); two carriageway ribbons over each other (an on-ramp ending on the through road's axis) are ordered by lane count (more lanes wins, equal = untouched); decals (flags 16/32: paint, turn arrows, junction overlay) are lifted kDecalLiftM 0.025 because they cannot be cut. Replaces "lift the layers a few mm apart": the layers are draped by separate vertex sampling and differ 2-24 mm, more than their 3-7 mm lifts. Tests tests/unit/test_surface_layers.cpp SL1-SL11 [surface_layers][world] (SL8 hidden-style real tile, Engelsruhe spawn). docs/road_surfaces.md "Layering of road surfaces"
      credits.cpp, settings.cpp, spawn_presets.cpp, shell_flow.cpp, camera_math.cpp, road_ahead.cpp, json_util.h   (R5) implementations of the six headers above (json_util.h: shared exception-free JSON helpers)
    CMakeLists.txt                    rg_core STATIC target
  godot_ext/
    src/
      register_types.h/.cpp           GDExtension entry point (rg_godot_library_init), registers RgSimulation + RgTerrainView
      rg_simulation.h/.cpp            RgSimulation : godot::Node - the one GDScript-facing class; owns one rg::Session and one rg::PlayerModeMachine; initialize_terrain() (R7) builds a terrain-mode Session on a worker thread, polled via get_init_status() - see "Godot bindings for Session terrain (R7)" and "Player modes and world switch (R9)" below; get_wheel_attachment_local/_steered/_is_front (static, from Session::vehicle_desc())/_compression/_spin_angle/_steer_angle (per-frame, from FrameSnapshot::wheels[i].state) feed vehicle_visual.gd - see "Vehicle visual (carvis)" below
      rg_render_focus.h/.cpp          (2026-10-13) RgRenderFocus : godot::RefCounted - tracker wrapper for camera_director.gd, static rig_leads_focus and grid_coverage_ahead (road/building ribbon coverage)
      rg_ring_streamer.h/.cpp         (phase 3b) RgRingStreamer : godot::RefCounted - binding of g2m::world::RingStreamer (configure, update, next_job (any thread), accept, frame -> [{x, y, near, far, downgraded, evicted}], known_tiles, pending_jobs, oldest_wanted_age_s); road_stream.gd and building_stream.gd only run their worker thread (next_job -> build the tile) and the render nodes
      rg_steering_assist.h/.cpp, rg_address_search.h/.cpp   (2026-10-14) RgSteeringAssist (RefCounted) and RgAddressSearch (static) - conversion-only bindings of the two headers above
      rg_sky.h/.cpp                   (V1) RgSky : godot::RefCounted - rg::TimeWeather binding (initialize(sky_json, world_config), advance, get_state with Godot-frame sun/moon directions, get_menu, choose, set_local_time_h, format_temperature_c)
      rg_terrain_view.h/.cpp          RgTerrainView : godot::Node3D (PLAN.md R2.1) - LOD terrain preview seam, streamed around a focus since R8; see "Terrain preview (R2.1)", "Render LOD streaming (R8)" and "Godot bindings for Session terrain (R7)" below
      terrain_surface_pack.h/.cpp     (C9, 2026-10-15) pack_chunk_surface (a chunk's ARRAY_VERTEX/NORMAL/COLOR/INDEX via ptrw copies) + SurfacePackJob (one worker thread packs a diff's adds in order while the main thread uploads the packed ones) - see "Upload pipelining (C9)" below
      rg_navigation.h/.cpp            (R4map) RgNavigation : godot::Node - binding of RoutePlanner + MapView for one real world (initialize_shared(sim, config, x, y) via RgSimulation's friend shared_world_terrain, release, request_map/poll_map/get_map, plan_route/poll_route/get_route, get_progress, set_view_lod). RgTerrainView.set_view_lod(choice) applies the LOD settings (TerrainViewStreamer::set_source: reselect at once; resident chunks of the levels whose render_road_lod_key changed are rebuilt into TerrainViewDiff::replaced and swapped in place via replace_chunk_at); RgShell.get_lod_choice()
      rg_shell.h/.cpp                 (R5) RgShell : godot::Node - rg::ShellFlow + rg::Settings + rg::Credits + spawn choices as Dictionaries/Arrays for GDScript (initialize(data_dir, user_dir, world_config_path), boot_finished/direct_start/menu_item/spawn_picked/back/load_ready/load_failed/load_cancelled/pause_toggle -> Transition dictionary, get_*_menu_items, get_spawn_choices, get_settings_schema, get_setting/set_setting/save_settings, get_attribution_line, get_credits_sections)
      rg_garage.h/.cpp                (R6) RgGarage : godot::Node - rg::Garage as Dictionaries/Arrays (get_vehicles/get_vehicle with stats, get_options, set_option, save, prepare_drive, setup_camera/camera_go_to/camera_update/get_camera, get_work_file_count); RgShell gained vehicle_chosen/garage_drive and the open_garage/close_garage actions; RgSimulation::set_vehicle_overrides (chassis mass/box/z, engine map cache dir)
      rg_garage_browser.cpp           (R6c) the RgGarage.browser_* bindings (state load/get, group/sort choices, toggle/clear filter, filter options with counts, rows/layout/view, move_focus/set_focus, scroll, get_car with thumbnail_key, maxima, refresh); RgGarage.initialize takes an optional catalog path (--catalog)
      rg_camera_math.h/.cpp           (R5) RgCameraMath : godot::Object - static bindings of rg/camera_math.h (drive_view_names, next_drive_view, toggle_cockpit_view, orbit_default_state, orbit_step, drone_*, chase_*, free_*, walker_cam_*, orbit_rig_*, ground_probe_points/apply_clearance/look_valid)
      rg_camera_motion.h/.cpp         RgCameraMotion : godot::RefCounted - rg/camera_motion.h binding (initialize, update(RgSimulation.get_camera_motion_sample(), delta), get_view(view, strength), get_state); RgSimulation.get_bumper_camera(rig_json_path, model_offset)
      rg_simulation_shell.cpp         (R5) second translation unit of RgSimulation: unload() (stop sim thread, streaming, modes, cinematic), set_paused/is_paused, set_spawn_override/clear_spawn_override, set_store_dir_override, set_road_ahead_wanted, get_chassis_session_pose, session_to_godot, get_bumper_camera, update_cinematic/reset_cinematic
      frame_convert.h                 rg_godot-namespaced wrapper around physics_sim's frame_convert_core.h (Vec3f/basis/pose -> godot::Vector3/Basis/Transform3D)
    CMakeLists.txt                    rg_godot SHARED target (the GDExtension DLL)
  game/                                Godot project (res:// root)
    project.godot                     rg_* input actions, Mobile renderer (Vulkan; Compatibility = fallback, Forward+ = optional desktop quality, via graphics.renderer / user://renderer_override.cfg, see docs/mobile_renderer.md)
    rg_godot.gdextension              points at bin/librg_godot.dll/.so
    bin/                              rg_godot build output (librg_godot.dll + .pdb)
    scenes/
      main.tscn                       one-node stub (Node3D + main.gd) - the scene is built procedurally, see main.gd's own comment
    scripts/
      main.gd                         the one game scene (R9), built in _ready(): flat or real world, Drive or FreeCam, runtime mode/world switching, the loading flow, forwarding input to RgSimulation (process priority -2000); flags in its own header and in "Player modes and world switch (R9)" below. A `--terrain-preview` cmdline user-arg (after `--`) branches into the R2.1 static-terrain-plus-fly-camera scene instead - see "Terrain preview (R2.1)" below; in that scene the fly camera drives `RgTerrainView.update_focus` every frame (R8) except under `--screenshots`/`--stream-test`; `--bindings-test` (R7) runs a flat-mode-only RgSimulation/RgTerrainView bindings smoke check instead - see "Godot bindings for Session terrain (R7)" below. `_rg_data_path(relative)` (engine30, 2026-09-27) resolves against this repo's OWN `data/` (sibling of `external/`, same "one level up from res://" pattern as `_data_path`'s `external/physics_sim/data`) - both bindings-test and `_load_world`'s `vehicle_json` now point at `_rg_data_path("vehicles/car_sedan.json")` (racing_game's own file, see "data/vehicles, data/engines (engine30)" below), `surface_table_json` is racing_game's own `data/surfaces/surfaces.json` (via `_rg_data_path`-style `res://../data/`, own file since RACE-002, grass `lambda_mu` 0.6 since 2026-10-12 (0.63 from 2026-10-05), see docs/grass_grip.md), not physics_sim's shared table. `get_body_visuals()` exposes the `body_visuals.gd` node (drive_tour.gd's steering-proof poses.txt columns read `get_wheel_visual_steer_angle_rad` through it)
      terrain_stream_test.gd          `--terrain-preview --stream-test` (R8 headless check): after the initial upload, moves the LOD focus through 5 fixed steps from spawn, waits for each streamed diff to be fully applied, prints one line per step + `terrain stream test done: ...`, quits (180 s wall-clock timeout)
      terrain_upload_bench.gd         (C9) SceneTree bench, no map store: `godot --headless --path game --script res://scripts/terrain_upload_bench.gd -- --speed=100 --seconds=20 --fps=120` streams RgTerrainView.load_synthetic_preview with a moving focus and prints RG_UPLOAD_BENCH lines (per-frame adapter ms avg/p50/p99/max, per-chunk pack/surface/instance ms from get_frame_costs uploads_total/upload_*_ms_total)
      camera_director.gd              (R9) owns the camera rigs; floating-origin rebase by the ACTIVE rig only (moves rig ROOTS), set_render_origin in the same frame, update_focus from the active camera (priority -1000); owns the RgCameraMotion and hands each rig with a `motion` property its view (cockpit: head offset/pitch/roll; xr: head offset only; chase/bumper/cockpit: shake + FOV) before update_rig. camera_lag_test.gd (`godot --headless --path game --script res://scripts/camera_lag_test.gd`): rig position/angle error vs the ideal pose at 50/150/300 km/h, straight and turning; orbit and drone smooth the car-relative offset since 2026-10-14. bumper_shots.gd: one bumper-view screenshot per car (`-- --flat --vehicle ID --bumper-out F [--bumper-old]`)
      chase_rig.gd                    (R9) Drive rig: root + Camera3D, lagged chase of the chassis, look-around from the camera input group; replaces R0's chase_cam.gd. side_offset_m (carvis, 2026-09-27, default 0.0 - no behaviour change for any normal chase view): adds a lateral offset (chassis basis.y, ISO left) to the "behind" position, still looking at the centreline - a directly-behind camera can never show a front wheel (the body occludes it face-on); drive_tour.gd's steering_close shot is the only caller that sets it nonzero, for a 3/4-rear angle
      walker_rig.gd                   (R9c) OnFoot rig (`walker`): root + Camera3D, third person (orbit, wheel/PgUp/PgDn zoom, terrain clearance) or first person (Tab); owns the look yaw/pitch, `get_look_forward()` is what main.gd forwards with the walking input; reads RgSimulation.get_walker_state(). walker_visual.gd: placeholder capsule + facing nose (data/models untouched)
      free_rig.gd                     (R9) FreeCam rig: root (position, yaw) + Camera3D (pitch), driven only by the camera input group; place(pos, yaw, pitch)
      body_visuals.gd                 (R9; carvis) ChassisRoot (Node3D, transform = get_body_transform("chassis") every frame, priority 0) carries two children: ChassisBox (the red placeholder, CHASSIS_HALF_EXTENTS, unswapped) and VehicleVisual (vehicle_visual.gd, the real car_sedan.glb); the box is hidden once the model reports loaded, shown as fallback otherwise (vehicle_model_ok(), read by hud.gd). Also the flat world's ground box - its BoxMesh size/local position are authored in ISO order (x fwd, y left, z up, matching CHASSIS_HALF_EXTENTS), NOT native Godot order (bug found/fixed 2026-09-27: `_ground_anchor.transform` is `get_body_transform("ground")`'s ISO->Godot basis - an ISO-native mesh authored Y-up instead rendered as a ~2000-unit vertical wall, invisible edge-on from most angles and a "green plane sideways through the middle of the car" from others). on_session_ready() (called by main.gd right after a Session starts/re-starts) rebuilds VehicleVisual's wheel bindings for the new Session. get_wheel_visual_steer_angle_rad(i) (carvis steering-proof fix, 2026-09-27) forwards to VehicleVisual's own accessor of the same name
      vehicle_visual.gd               (carvis, 2026-09-26) near-copy port of physics_sim's adapters/godot/demo/scripts/vehicle_visual.gd (read-only reference): loads external/physics_sim/data/models/car_sedan/car_sedan.glb at runtime via GLTFDocument (data/models is owned by another session - read, never copied into this repo), binds susp_*/steer_*/wheel_* nodes by name from car_sedan.rig.json, aligns the model to the physics wheel attachment points, and drives suspension travel/steer angle/spin angle from RgSimulation's per-wheel accessors every frame - no client-side spin integration (unlike the reference) since rg::vehicle::WheelState already carries an integrated spin_angle. Keyed by a model-name string (vehicle_name) so a future car_hyper reuses it. rebuild() re-binds wheel nodes without reloading the .glb - called by body_visuals.gd's on_session_ready() after a runtime world switch (R7/R9) rebuilds the Session; `_process()` also retries `_bind_wheel_nodes()` on its own while `_wheel_nodes` is still empty (bug found/fixed 2026-09-27: the flat world's `on_session_ready()` call lands the same frame as `Session::start()`, before the sim thread's first tick has created the vehicle, so `get_vehicle_wheel_count()` read 0 at that instant and the binding silently stayed empty forever - poses.txt's `wheel_steer_visual_rad` stuck at 0.0 every frame was the symptom; the real world's own step-count-gated `on_session_ready()` call was already correct). get_wheel_visual_steer_angle_rad(wheel_index) (carvis steering-proof fix, 2026-09-27) reads back the steer_<corner> node's own last-applied local Y rotation (0.0 if unsteered/missing) - read by drive_tour.gd for poses.txt so a sign/axis mismatch between the physics steer angle and what actually got applied to the visual shows up as two numbers, not just a screenshot
      loading_overlay.gd              (R9) CanvasLayer shown while a real-world load runs (get_init_status() numbers) or after it failed. A roads.geom tile the deriver itself rejects ("deriver g2m.roads.geom: roads.geom: ...", deterministic) falls back to smoothed terrain like "required dependency absent" (RG_ROAD_SURFACE unavailable reason=deriver_rejected, world_terrain.cpp) instead of being retried until the 30 s start-up timeout (Koenigstein preset, tile utm32n/2/452/5428, 2026-10-06; geo2map fix queued)
      navigation_hud.gd, minimap.gd   (R4map) CanvasLayer created by main.gd after the F10 dialog: attaches an RgNavigation while a real world runs (released otherwise), forwards the car pose, draws the heading-up minimap (polylines from RgNavigation.get_map, the planned route) and the next manoeuvres; destination from address_teleport.gd's "Set as route destination" button (signal destination_chosen). No logic here
      light_exposure.gd, lights_exposure_shots.gd   (2026-10-15) shared scene exposure + light sensor (static vars, written by sky_controller.gd) and the lamp photometry -> Godot units (spot cd, lens cd/m2); review shots day/dusk/night (states from rg_unit_tests "[exposure_shots]")
      sky_controller.gd               (V1) Sun/Moon DirectionalLight3Ds + WorldEnvironment driven by RgSky at 10 Hz (sky colours only on change); fixed sun without the extension. weather_widget.gd (top-right icon/time/temperature, click = time/weather menu from the RgSky view-model), weather_icon.gd (procedural icons). headlights.gd (V2): SpotLight3D per headlamp cluster of the rig's lights.head materials, data/world/headlights.json, follows cabin.lights; attached by exterior_lamps.gd, which also lights head/tail lamp emission. Test: visual_world_test.gd (pure GDScript)
      shell_ui.gd                     (R5) CanvasLayer (layer 30): boot splash, main menu, spawn picker, pause menu, settings (rows built from the schema: CheckButton/HSlider/OptionButton/LineEdit; Settings has a "Controls..." button, the screen itself is controls_ui.gd) and credits, all built from RgShell view-models; signals menu_item_chosen/spawn_chosen/back_requested/setting_changed/boot_finished; test hooks get_button(id)/button_ids()/get_setting_control(key)
      bumper_rig.gd, orbit_rig.gd, cinematic_rig.gd   (R5) the three new drive-view rigs (see "Shell (R5)"); all follow the PHYS-008 rig contract
      garage_scene.gd                 (R6) CanvasLayer (layer 20): the showroom in its OWN World3D (SubViewport): procedural room, turntable, softbox sky for reflections + a mirrored car twin on the glossy floor, studio lights, camera from RgGarage.get_camera (shifted so the car sits in the screen part the side panel leaves free); model loading (GLB) and paint/rim recolouring - shown through a TextureRect whose SubViewport is sized to physical pixels (rect size x content_scale_factor), so UI scaling does not blur the car
      ui_scale.gd                     root content_scale_factor = the OS display scale (Windows: monitor DPI / 96, e.g. 1.5 at 150 %), applied first thing in main.gd _ready; scales menus, garage, HUD and overlays, 3D stays at full resolution (stretch mode disabled); override --ui-scale=F or env RG_UI_SCALE (2026-10-06)
      garage_shots.gd, garage_test.gd (R6) `--garage-shots <dir>` (real window: the acceptance screenshots) and `--garage-test` (headless acceptance flow, tools/smoke_test.ps1 -Garage)
      car_browser_ui.gd, car_thumbnails.gd (R6c) the car browser screen (stats of the focused car on the left - metric only; horizontally scrolling grid of 2-4 rows with sticky category headers, tiles instantiated only for the window RgGarage.browser_get_view returns; filter/sort panel on F / gamepad Y / the on-screen button; arrows, D-pad, left stick, mouse hover/click/wheel) and the lazy preview renderer (own SubViewport + World3D studio, one picture per frame, PNG cache user://car_thumbs/<key>.png, none in a headless run); both are created only while a browser screen is up
      car_browser_test.gd, car_browser_shots.gd (R6c) `--car-browser-test [--car-browser-big]` (headless acceptance flow, tools/smoke_test.ps1 -CarBrowser / -CarBrowserBig) and `--car-browser-shots <dir> [--car-browser-big]` (real window; tools/car_browser_shots.ps1 runs both sets); `--catalog <path>` loads another vehicle catalog (tools/make_synthetic_catalog.ps1 writes the 200-car synthetic one)
      shell_flow_test.gd              (R5) `--shell-test`: the headless UI flow test (tools/smoke_test.ps1 -Shell), presses the real buttons
      camera_switch_test.gd           (R5) `--camera-test`: the PHYS-008 camera-switch test (tools/smoke_test.ps1 -Cameras; drone, FPV drone and walker rigs are in the matrix)
      drive_smoke.gd                  (R9) `--drive --drive-smoke`: scripted drive, relocation (with --g2m-fetch-delay-ms), mode round trip, world round trip incl. a cancelled load; prints RG_DRIVE lines for tools/smoke_test.ps1 -Drive
      drive_tour.gd                   (R9; carvis) `--screenshots <dir>`, with or without `--drive` (a flat-world run skips the loading-overlay shot and runs the same tour unattended): loading overlay, spawn chase, a steering-lock close-up (front wheels turned, proves per-wheel steer_angle), driving and free-cam proof shots plus poses.txt. `02_steering_close.png` (steering-proof fix, 2026-09-27) frames a FRONT 3/4 angle (`CLOSE_DISTANCE_M` negative + `chase_rig.gd`'s `side_offset_m`, see that file's own comment) so both turned front wheels are visible - a rear-3/4 angle's near-side wheel is the rear-left one, hidden from the front wheel by the cabin. poses.txt's `_numbers()` also logs `wheel_steer_phys_rad`/`wheel_steer_visual_rad` per wheel (physics `get_wheel_steer_angle` vs. the visual node's own applied rotation, via `main.get_body_visuals()`)
      spawn_shots.gd                  `--drive --spawn-shots <dir>` (z-fighting diagnosis, 2026-10-19): once the streamed terrain settled it shoots the car at rest from the chase camera and from a low close camera, each with everything drawn / terrain only / ribbons only (01..06 png + poses.txt). Needs a window; SETTLE_S 12 s covers the late road-tile stream
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
      route_planner.json              rg.route_planner/1 (R4map): routing region radius, landmarks, snap distance/heading tolerance, HUD manoeuvre count, off-route distance, minimap radius, map tile cache limit
      regions.json                    g2m.regions/1 (geo2map_engine G1c I7): region "home" (Main-Taunus-Kreis,
                                       Hochtaunuskreis, Frankfurt-Höchst), halo_m 2000 - consumed by
                                       `g2m_tiler import ...regions.json#home ...` (S:\claude_code\geo2map_engine)
      spawn_presets.json              rg.spawn_presets/1 (R5): the spawn picker's presets (world_spawn, engelsruhe, b8_trunk, b8_north, koenigstein, a5_north) in the session frame
      climate.json                    rg.climate/1 (V8): station elevation, monthly temperature/range/rh/ground temperature, lapse/peak/cloud damping, resync limits, physics climate blocks, surface-condition and tyre-profile paths, physics preset per sky weather id
      sky.json                        rg.sky/1 (V1): time zone, start date/time, rate options, time and weather presets, icon thresholds, lighting keys by sun elevation
      headlights.json                 rg.headlights/1 (V2): dipped/main beam range, angle, energy, pitch, shadows off, lights_per_car
      crops.json                      rg.crops/1 (V6a): world seed, date jitter, 13 crop calendars, 6 weighted rotations, fixed landuse calendars
    settings/lod_settings.json        rg.lod_settings/1 (R4map): the four in-game LOD settings and the road-detail presets, see core lod_settings.h
    credits.json                      rg.credits/1 (R5): every data source/library with licence and required attribution text; shown on the boot splash, loading screen and credits screen
    controls/
      fixed_keys.json                 rg.fixed_keys/1 (R5b): the keys the controls screen lists but cannot rebind (Esc/P, F6, F7, F10, F9, middle mouse, Shift); described by hand, keep in step with the scripts
      actions.json                    rg.control_actions/1 (R5b): the ONE action schema (52 actions: id, label, group, kind, mode held/edge, range, modes); strictly validated
      defaults/{keyboard,mouse,gamepad}.json  rg.control_profile/1 (R5b): built-in per-class default bindings (the pre-R5b ones, plus the B7 pad bindings of driving actions: docs/controls.md "Gamepad coverage")
    routes/
      home_r1_drive.json              rg.route/1 (R2.2 R5): the scripted-drive route, waypoints in the session frame + the spawn it starts from; checked by tools/route_check. Spawn (Engelsruhe, Frankfurt-Unterliederbach) -> B 8 -> the Koenigstein city-limit sign, 9.85 km, 2103 OSM-centreline waypoints densified to <= 5 m; its "criteria" (max_grade_pct 31, min_corner_radius_m 7) and every measured number are in its own "source" field
    vehicles/
      car_sedan.json                  physics_sim.vehicle/2 (engine30, 2026-09-27): racing_game-OWNED variant of external/physics_sim's data/vehicles/car_sedan.json (physics_sim is read-only from this repo, so this is a full copy, not an override/patch) - byte-identical except the engine component (now engines/n52b30_3l_na.json below, was physics_sim's i4_2l_na.json), the clutch capacity (300.0 -> 450.0 N*m kinetic, static_factor unchanged 1.15) and the wheel tyre / gearbox component refs (now `../../external/physics_sim/data/tyres/...`/`.../gearboxes/...` - refs resolve relative to THIS file's own directory, two levels shallower than physics_sim's original). This is the game's drivable car (main.gd's `_rg_data_path("vehicles/car_sedan.json")`, both flat and real worlds) - see this file's own "source" field for the full derivation and tests/unit/test_vehicle_data.cpp for the one number it is held to. Every test that pins a state hash (hash_check, test_session*, test_player_mode) still points explicitly at physics_sim's OWN car_sedan.json (external/physics_sim/data/vehicles/car_sedan.json, unaffected by this file's existence) - unchanged hashes
      car_toyama_xr.json, car_generic_suv.json, car_phoenix_455.json, car_ace_11.json, car_fairheaven_lt80.json, car_kiri_86.json, car_lct_3000.json, car_shvan_92.json   (2026-10-14/15) written by tools/carimport from car_sedan / car_sedan_awd / car_gt70 with the imported models' wheel positions and tyres; catalog entries car_toyama_xr, car_generic_suv, car_phoenix_455 (models data/models/imported/<id>/, Git LFS; credits model_<id>). Rigs with "wheel_width_scale": "rig" give vehicle_visual.gd / garage_scene.gd the wheel's own reference width (tyre_width)
      car_sedan_rwd.json, car_sedan_awd.json, car_sedan_gen_n2o.json   (engine garage G1) racing_game-owned copies of the physics_sim variants (refs point back into external/physics_sim/data/) with a fuel_tank + engine.fuel link added so the garage fuel choice works on every catalog car (E5, the A25A car E10)
    engine_thermal/                    physics_sim.engine_thermal/1 (engine thermal model, every game car has one since 2026-10-12): n52b30.json (car_sedan, extends physics_sim class_si_passenger.json, 2.996 L cited, rest [AS]/[D]) and hyper_v8_one1.json (car_hyper, car_hyper_n2o: 5.066 L, wall_heat_fraction pinned to 0.20 = the turbo configurations default); engines name them via default_thermal
    strip_visual.json                 rg.strip_visual/1 (PR #22 follow-up): per model, which nodes the weight-reduction choices hide (see core strip_visual.h); sits beside part_masses.json and setup_options.json ; a weight-reduction option is offered only where the catalog `interior` has the slot (`rg::part_choice_offered`: no passenger-seat option on the Runabout bench, no rear seats on the Ventura GT/hypercars)
    cooling_systems/                  physics_sim.cooling_system/1: hyper.json (1.4 m2 of radiator core scaled from the class core by area x 1.2 core depth, ram 0.22, fan 7 m/s, oil cooler 1.0 W/K per m/s; upsized 2026-10-16 for the time-to-limit setup check; sizing arithmetic and measured temperatures in its source). The sedans name physics_sim cooling_systems/sedan.json. Vehicle files also carry engine_thermal_start "warm" (the one place to change the spawn temperature: "ambient" = cold soak)
    engines/
      n52b30_3l_na.json                physics_sim.engine/1 (engine30, 2026-09-27): racing_game-owned NA 3.0 L I6 (BMW N52B30, 190 kW@6600rpm/300 N*m@2500-4000rpm variant) torque-map engine, referenced only by data/vehicles/car_sedan.json above - see this file's own "source" field for the full per-point citation (published vs. derived vs. assumed)
      engine_catalog.json              rg.engine_catalog/1 (engine garage G1): the default engines of the garage (see core/include/rg/engine_store.h); fuel_dir external/physics_sim/data/fuels; masses [AS]; hyper/demo-pipes/A25A peaks measured with run_dyno_sweep (hidden [long] re-measure in tests/unit/test_engine_garage.cpp)
    gearboxes/                         gearbox swap (2026-10-13), racing_game-owned physics_sim.gearbox/1 files + the catalogue: econ_5mt.json (3.45-0.70, 320 N*m), econ_6mt_overdrive.json (3.38-0.58, 320), close_ratio_6mt.json (3.30-0.92, 550), heavy_duty_6mt.json (2.95-0.60, 700, stronger synchros), straight_cut_6sq.json (type sequential, 2.95-0.76, rated 1800, dog engagement: 90 N*m speed-match stand-in, 25 rad/s dog window, 0.025 s neutral, eta 0.985) - all ratios, efficiencies, inertias and timings [AS] with the reasoning in each file's `source`; gearbox_catalog.json lists these plus physics_sim's sedan_6mt (Touring, 400) and hyper_7mt (Hypercar, 1600). game/assets/components/drivetrain/gearbox.svg is the garage card art
    drivetrains/driveline_catalog.json  rg.driveline_catalog/1 (drivetrain swap, 2026-10-15): drive layouts and differential units; numbers and [AS] reasoning in its `source`
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
      test_render_focus.cpp           (2026-10-13, tags [render_focus][terrain_stream]) render focus lead/cap/smoothing/reset, adaptive upload budget, urgency sort, lod/grid coverage ahead, chunk_surface_height, SampleWindow, reselect_distance_for_speed
      test_steering_assist.cpp, test_address_search.cpp   (2026-10-14, tags [steering_assist], [address_search]) golden sequence from the former GDScript steering, loader errors, mirror symmetry, relocation reset; query/URL/response parsing, UTM destination vs the former GDScript series (< 1 cm)
      test_terrain_view_streamer.cpp  rg::TerrainViewStreamer tests (R8) over a synthetic, optionally gated tile store: exact key diff, hole-free adds-then-removals at every step (plus a wrong-order negative control), no work while stationary, 1-vs-8 build-thread identical diffs, coalescing while busy, cancel+join on destruction
      test_route_check.cpp            rg::route_check tests on synthetic terrain (seam counting incl. negative indices/corners, grade window max/p99, NoData, corner radius, start/length criteria, g2m::world::sample_height_bilinear across tile borders, load_route errors, steep-stretch and tight-corner lists, corner radius on arcs/kinks and its density independence, "criteria" loading + apply_route_criteria precedence); one hidden `[.][realdata]` case runs the committed route on the real store under its own "criteria", SKIP unless RG_G2M_HOME is set
      test_envelope_cost.cpp          (hidden [.][perf][envelope]) times the first flat Session of car_hyper/car_hyper_n2o (pays physics' full-load envelope build) against the second (memo hit)
      test_game_car_thermal.cpp       (tags [thermal][vehicle_data], [engine_swap]) every data/vehicles/car_*.json loads with the engine thermal model (validate-only load) and a car loses its cooling_system when a swapped-in engine has no thermal data; hidden [.][diag][thermal_run] measures hypercar coolant/oil over a full-throttle run or a 130 km/h cruise (RG_DIAG_CAR/MODE/AMBIENT_C/HOLD_S/NITROUS)
      test_lod_settings.cpp           (R4map, tag [lod_settings]) shipped lod_settings.json loads, defaults == today's LodParams/default_render_road_lod/MapConfig, strict loader errors, schema extension, changed_road_lod_levels; test_terrain_view_streamer.cpp gained set_source cases (reselect on view distance, only changed road-LOD levels rebuilt into `replaced`)
      test_route_planner.cpp          (R4map, tags [route_planner], [map_view]) synthetic 9x9 street grid tiled into roads.graph tiles: graph build, snap, route + manoeuvres, unreachable/no-start, progress/off-route, coalescing, map view tiles and detail change
      test_credits.cpp, test_settings.cpp, test_spawn_presets.cpp, test_shell_flow.cpp, test_camera_math.cpp, test_road_ahead.cpp   (R5) one per new core header; test_player_mode.cpp gained the unload_world case, test_session.cpp the pause/road_ahead cases, test_world_config.cpp the store_dir override
      test_climate.cpp, test_session_weather.cpp   (V8, tags [climate], [weather][v8]) climate.json, temperature (peak/night/lapse/cloud), physics climate/preset; no weather = nothing installed, rain wets the tagged flat ground (condition factor < 0.95, braking 25 m/s +10 % or more), 1 vs 4 workers hash, menu enabled + WeatherSync
      test_time_weather.cpp           (V1, tags [sky], [time_of_day], [weather_icon]) sun within 0.5 deg / moon within 1 deg of PyEphem for Frankfurt, EU summer time, clock rollover, icon table, sky.json + lighting + menu greying
      test_crops.cpp                  (V6a, tag [crops]) config load, determinism, rotation by harvest year, winter wheat stages Nov..Aug, crop tag/fixed calendars, rotation shares, row direction
      test_world_terrain.cpp          rg::WorldTerrain / build_static_view_from_lookup tests (PLAN.md R2.1) over a synthetic in-memory TileKey->HeightTile map - no TileStore/Server needed; chunk selection, session-local origin math, 1-vs-N-thread byte-identical output (the decode/cache cases moved to geo2map tests/world with the code, phase 1)
      test_surface_layers.cpp         resolve_surface_layers (surface_layers.h): SL1 verge under a walkway band, SL2 footway across the carriageway, SL3 junction patch over the ribbon, SL4 decal lift, SL5-SL7 over-reach guards (shared edges, bridge deck height, uv/height of cut pieces), SL8 the real Engelsruhe spawn tile (no overlapping layers, decals lifted, timing report), SL9 earth track across a dual carriageway (B8 site 2), SL10/SL11 on-ramp ribbon vs through road (B8 site 4) and its guards; each with a recorded sabotage
      test_route_road_coverage.cpp    (roads_plan.md R-3) hidden `[.][realdata]` case, SKIP unless RG_G2M_HOME is set: point_is_road() classifies one absolute UTM point via WorldTerrain::road_segments + a 1x1 g2m::Lattice fed to rasterize_road_segments at the L0 level policy (every road, min_rank=0/min_half_width_mm=0); measured on the real home-r1 store against the committed data/routes/home_r1_drive.json (2103 waypoints): 100.00% road coverage (2103/2103, spawn included), forest-stretch control (84 points 30 m off-route at s=[5800,6200], the s~6000 m stretch roads_plan.md names) 0.00% road - see "Visible roads (R-2)" below
      test_vehicle_data.cpp           (engine30, 2026-09-27, tag [vehicle_data]) loads data/vehicles/car_sedan.json via ps::io::load_vehicle_json and asserts the ONLY number that data file is held to: the clutch's static torque capacity (capacity_nm * static_factor) >= 1.2x the engine's peak WOT torque (max of TorqueMapEngineDesc::wot_torque_nm_vs_rpm.y) - both read back from the loaded VehicleDesc's PowertrainDesc::components, nothing hand-copied from the JSON's own "source" commentary. No launch/stall/top-speed/wheelspin test (owner ruling: wheelspin is allowed and not to be gated or tested on)
      CMakeLists.txt                  rg_test_catch_main + rg_unit_tests targets, CTest registration
  tools/
    common.ps1                        shared PowerShell helpers (VS dev-shell entry, Godot exe lookup, cmake wrappers, submodule update) - dot-sourced by run.ps1/smoke_test.ps1/ci.ps1
    setup_dev_env.ps1                 installs the toolchain via winget (VS 2022 Build Tools + clang-cl, CMake, Ninja, Godot 4.7.2, Python 3.13), checks out the submodules, optionally clones the sibling repos (-WithSiblings), reports which worlds have map data; -CheckOnly installs nothing
    run.ps1, run.cmd                  incremental build + launch Godot on game/ - opens the game SHELL (boot splash, main menu) by default (R5); `-Drive` skips it and starts the real world in Drive mode (`--drive`), `-Flat` the flat scene (`--flat`), `-VR` starts in the world too, by default at preset a5_north in the rhein_main_corridor world (sets RG_WORLD_CONFIG for the child only unless already set; main.gd `--spawn=ID` picks any data/world/spawn_presets.json preset on a `--drive` start); pass `-- --terrain-preview` (see run.ps1's own pass-through-args comment) to launch the R2.1 terrain preview instead
    make_release.ps1, make_release.cmd  playable release SNAPSHOT: builds rg_godot (relwithdebinfo, like run.ps1; `-SkipBuild`), refuses a dirty tracked tree (`-AllowDirty`), copies game/ (DLL+PDB+.godot), data/, external/physics_sim/data (+ tyre audio, warm engine-map caches), external/geo2map_engine/data and a stub CMakeLists.txt (world_config.cpp's repo-root marker) into S:\claude_code\racing_game_releases\<yyyyMMdd-HHmm>_<sha>\ with play.cmd / play_drive.cmd (RG_G2M_CACHE / RG_G2M_HOME default to the shared geo2map locations) and RELEASE.txt; maintains latest.cmd, keeps the newest `-Keep` (3) snapshots; the geo2map stores are shared, never copied
    smoke_test.ps1                    headless Godot smoke test (build, ctest, headless run, assert no errors + sim thread ticking); `-TerrainPreview` runs the R2.1 headless check instead (asserts >= 150 chunks selected and a completed upload, see its own header comment); `-TerrainStream` (implies -TerrainPreview, R8) adds `--stream-test` and asserts `steps=5 diffs=5 ... missing_removals=0` - with the 0-ERROR-lines check this is the RID-leak check after streamed add/remove diffs; `-Drive` (R9) runs `--drive --drive-smoke` and asserts `RG_DRIVE ready`, falls=0 misses=0 on every RG_DRIVE numbers line, ticks > 0 and result=ok; `-DriveDelayMs N` adds `--g2m-fetch-delay-ms N` and asserts relocate_freezes >= 1 and advanced_after_relocate > 0; both SKIP (exit 0) without a geo2map store; R5: the plain run passes `--flat`, `-Shell` runs the headless UI flow test, `-Cameras` the PHYS-008 camera-switch test; R6: `-Garage` runs the headless garage acceptance flow (`garage_test.gd`)
    carimport/                        (2026-10-14) Python: survey.py (table of data/models/downloaded/*.glb), carimport.py (cars.json -> data/models/imported/<id>/<id>.glb + .rig.json: axes/metres normalised, wheels by name or geometric clustering under susp_/steer_/wheel_ pivots at the wheel centres, calipers under steer_, attribution + source sha256 in asset, `derived` record in attributions.json, game vehicle file from a base file with the model's wheel positions and a matching tyre; lamp lenses split onto light_<kind> materials by node/material regex + ISO box (rig `lights` block, colours from cars.json light_look), paint/rim materials renamed to the roles the garage recolours (paint_tinted keeps an atlas texture), socket_driver_eye; self-checks), analyze.py, glb.py; docs/car_model_import.md (survey, playable cars, brand models awaiting the owner)
    make_synthetic_catalog.ps1        (R6c) writes a synthetic vehicle catalog (the real base cars + generated presets, default 200 entries) for the car browser tests and screenshots; `car_browser_shots.ps1` takes the browser screenshots into out/car_browser_screens/ (needs a real window)
    ci.ps1                            Windows CI, default = release leg, ctest -LE long (a test >= 5 s in release is tagged [long]; Catch2 tags are CTest labels), -Topic <tag>, -Smoke, -Full = debug + release, every test incl. [long], all smoke runs (smoke_test.ps1 -SkipTests so it does not re-run ctest) - NO Linux leg (unlike physics_sim's tools/ci.ps1); R5 adds the smoke_test -Shell and -Cameras legs, R6 the -Garage leg
    lod_measure/
      main.cpp, CMakeLists.txt        lod_measure executable (PLAN.md R2.1): measures rg::WorldTerrain::build_static_view cold/warm wall time + chunk/vertex counts against the real data/world/world_config.json at max_distance_m in {6000, 12000, 20000} - see "Terrain preview (R2.1)" below for the measured table and chosen default
    hash_check/
      main.cpp, CMakeLists.txt        hash_check executable - the R0 acceptance check, see "Hash comparison acceptance check" below
    route_check/
      main.cpp, CMakeLists.txt        route_check executable (R2.2 R5, links rg_core only): `route_check [--world-config PATH] [--route PATH] [--min-length-m M] [--max-grade-pct P] [--min-seams N] [--min-corner-radius-m R]` (defaults data/world/world_config.json, data/routes/home_r1_drive.json; the four limits override the route file's "criteria", which override RouteCheckParams' defaults) - prints the effective criteria, then rg::check_route_on_world's report; exit 0 pass, 1 criterion failed, 2 load/usage error
```

## physics_sim submodule

`external/physics_sim` is a git submodule. `.gitmodules` names the GitHub
repo (a fresh clone fetches from there); this machine's own `.git/config`
keeps `S:\claude_code\physics_sim` as a local absolute path (decision D9), so
a pin bump here needs no push first. **Every command that updates it on this
machine must pass `-c protocol.file.allow=always`**, or git >= 2.38 refuses it with
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

Bumped to `2571f43` (2026-10-16, geo2map PR #15 on top of the underpass decks PR #13): `g2m/world/chunk_terrain_classes.h` (`TerrainClassLookup{fn, ctx, lod}`, `terrain_class_window`), `ViewSource::terrain_classes`, `TerrainService::terrain_class/peek_terrain_class`. The bump also brings `roads.geom` encoding 9 (underpass decks, decisions D53-D61): the roads.geom config hash and so the release id change, so every derived store goes cold for roads.geom, terrain.carved and the default release's other roads-dependent layers (warm before a drive); geo2map's `road_structures` deck builder now also emits the served underpass decks. Opt-in use of the class layer: "Served render classes" below.

Bumped to `16a8ec6` (2026-10-15, geo2map main after PR #13 merged, same tree as the tested head 0198cd4; underpass decks D53-D61): served `roads.geom` encoding 9 carries `RoadGeomTile::underpasses` (span of a ground stretch over a lower road in a DEM cutting, profile = rim chord). `rg::build_road_decks` is `g2m::world::build_road_decks`, which emits one inferred deck per span (no tunnel roof under it), so `RoadSurfacePatch` skips the ground carve there and keeps terrain below the slab (as for any inferred deck), the ribbons draw the deck height and `DeckInstaller` installs it as a collider. Test `tests/unit/test_underpass_decks.cpp` [underpass]. Grip still comes from `g2m.src.osm` road segments (the served envelope's no-grip-over-decks rule is not used here). Release-id bump: warm the stores again. Steep walls (D59): `RoadSurfacePatch` gives each 1 m carve segment the end planes of the served underpass decks beside its piece (`Segment::deck_lo/deck_hi`, from `geometry.underpasses`); past a plane its weight (and its transition influence) fades to zero over `kUnderpassWallFadeM` = 1 m like the server's carve, so the capsule ends and blend no longer paint the road up to 11 m into the cutting. Bridge approach decks are unchanged. The game builds no tunnel roof of its own (roofs come only from `g2m::world::build_road_decks`). Tag rules (D61): parking-structure storeys (`service=parking_aisle`, `level=*`) never get a deck, and a road over tunnel-tagged lower ways only stays on the ground (no served deck; the inferred tunnel roof as before).

Bumped to `a6e9947` (2026-10-15, terrain migration phase 5, geo2map design D48): the served completion's abutment walk is 200 m (was 60 m) and the second roads.geom discovery stage lists 205 m DEM/graph squares around every node of the incident bridges (`road_bridge_dependencies` gained a `walk_margin_mm` argument, default `kRoadBridgeWalkMarginMm`; tests/unit/test_road_overpass.cpp uses the default, as the deriver does). Release-id bump #2: warm the stores again. The game's `complete_road_profiles`, 20 m retry, DEM-sampler use and deck filling from game profiles are still NOT deleted: the deletion gate is the real-store probe (zero game-only entries on both regions), which has not run.

Bumped to `ff39870` (2026-10-15, terrain migration phase 4, geo2map design D47): served `roads.geom` encoding 8 carries tunnel floors (provenance `TunnelFloor`, the game's DEM-floor rule) and the game's accepted-bridge anchors, so `core/src/road_structures.cpp` `complete_road_profiles` finds tunnels and those bridges/ground stretches already profiled and skips them (it completes only declined entries). It is NOT deleted yet: the owner rule asks for zero game-only differences on the real stores first (geo2map's roadcmp test shows 0 on both OSM fixtures; the real-store probe is pending). Release-id bump: the derived stores go cold for roads.geom and terrain.carved; warm them (`TerrainService::warm` or `g2m_tiler bake`) before a drive.

Bumped to `7426eb8` (2026-10-15, terrain migration phase 3b: `g2m::world` ViewStreamer, RingStreamer and BuildingSet, see "Streamers and buildings moved to geo2map" below).

Bumped to `2b6a6e5` (2026-10-15, terrain migration phase 3a: `g2m::world` road ribbons and structure decks, incl. the grade-separation approach decks of the overpass fix, see "Road ribbons and structure decks moved to geo2map" below).

Bumped to `162f0c5` (2026-10-14, geo2map main: junction crossings PR #1 + G4 routing slices 1/2/4/5 and the LOD-settings API PR #2): `g2m_route` is linked PUBLIC into `rg_core` (route_planner.h, map_view.h); `g2m::RenderRoadLod` / `ClassLookup::lod` carry the in-game road-detail setting.

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
`${RG_G2M_HOME}` and `${RG_G2M_CACHE}` are special-cased: unset, they
default to `<repo root>/cache/g2m/home-r1` and `<repo root>/../geo2map_cache`
(`kDefaultRgG2mHomeRelative`/`kDefaultRgG2mCacheRelative`, `world_config.cpp`)
instead of erroring; the rhein_main configs name their stores as
`${RG_G2M_CACHE}/<store>`. Every other unset/unknown `${NAME}` is a
validation error.

## Terrain preview (R2.1)

First visible, static-LOD terrain plus a fly camera - no vehicle, no
`ps::World` (PLAN.md R2.1).

`rg::WorldTerrain` (`core/include/rg/world_terrain.h` + `core/src/
world_terrain.cpp`): the engine-neutral seam onto geo2map_engine's offline
in-process server/mesh stack. `WorldTerrain::open(config, err)` opens a
`g2m::world::TerrainService` (terrain migration phase 2, geo2map `g2m_world`):
it owns `config.source_store`/`derived_store`, the local release (same layer
opt-ins as before, release id unchanged), an offline in-process server and ONE
worker pool; every fetch is a ticket with a priority (Gate > Physics >
Prefetch > Render > Warm) and per-tile dedup. The game's own work runs on that
pool as two caller-defined products (`register_product_kind`): `rg.height.carved`
(raw DEM -> `smooth_terrain` -> road patch, any level; carve only at L0) and
`rg.road.surface` (roads.geom + DEM dependencies -> `complete_road_profiles` ->
20 m retry -> `RoadSurfacePatch`, one per L2 tile, joined by the ~25 L0 tiles of
that L2 tile with `TerrainService::join_erased`, so the 16-20 s cold derivation
runs once and on a worker; the old `geometry_mutex_` is gone, only a brief
`geometry_cache_mutex_` guards the two published-geometry maps). Callers on
non-pool threads (physics loaders, render workers, background builders) wait on
tickets; `height_tile_shared(key, priority)` / `road_segments_shared(key,
priority)` take the priority, `set_physics_priority` is what the physics fetch
asks at (Session::setup_terrain: Gate until the start-up gate opens, then
Physics). `physics_fetch()` of g2m_world is NOT used: it serves the uncarved
`g2m.terrain.height`, the game still carves (until phase 7). `build_static_view(cam_x, cam_y, out)` (session-local metres)
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
through to the height/slope ramp, see `chunk_vertex_colors()` in
`terrain_colours.cpp`).

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
(`g2m::world::rasterize_chunk_road_classes`, wired into `build_render_chunks` - see
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

## Served render classes (2026-10-16, opt-in)

`data/world/world_config.json` key `"render_classes"`: `"osm"` (default, absent = osm) or `"served"` (`WorldConfig::render_classes_served`). `"osm"` is the path described under "Visible roads (R-2)" (the game's `ClassLookup` over `WorldTerrain::road_segments`), release id and existing stores unchanged. `"served"`: `WorldTerrain::open` sets `LayerOptIn::classes` (g2m.terrain.class joins the release, which also adds g2m.terrain.carved; `height()` still serves g2m.terrain.height, physics and the game's own carve are untouched) so the RELEASE ID CHANGES and the derived store must re-derive; `view_source()` and `build_static_view()` then set `ViewSource::terrain_classes` (`world_terrain_class_lookup` -> `WorldTerrain::terrain_class_tile`: `peek_terrain_class`, else a Render-priority ticket waited on the build worker; null on Absent/Failed = all-Unknown window, Failed logs `WorldTerrain::terrain_class_tile ... failed` once per call) with the in-game road-detail `RenderRoadLod` as `lod` (the streamer's `set_source` LOD rebuild works as before), and leaves `class_lookup` unset so no src.osm raster runs for render. Colours still come from `chunk_colour_hook(RoadSurfaceMap)`. `rg::build_render_chunks`/`build_static_view_from_lookup` gained a trailing `TerrainClassLookup` parameter (default empty = byte-identical). Physics grip is NOT switched (still the OSM raster through `HeightTileSharedFetch`), so render and grip can differ at road borders. Differences to the OSM raster: a render vertex reads the class node at its south-west corner (cell centre vs node, up to one row/column at a road edge), junction cores take the highest-rank approach (server ownership). Tests `tests/unit/test_served_terrain_class.cpp` [served_class] (synthetic road scene, both paths, default-off) and `test_world_config.cpp`. Not verified on a real store.

## Session terrain mode (R4)

`SessionConfig::terrain` (`std::optional<TerrainModeConfig>`) switches
`rg::Session` from the flat ground box to streamed geo2map terrain; unset,
Session builds exactly what it built before (flat-mode `step()` is still
one plain `World::step()`, so `hash_check` is unchanged).

Diagnostics: env `RG_STREAM_TRACE=1` prints `RG_GATE` (one line per gate-freeze episode: cause, missing keys, distance/heading,
wanted age), `RG_STREAM`/`RG_STREAM_TILES` (every 5 s: phase table of the height derive, tile latency split fetch/queue)
(core/src/stream_trace.h, stream_trace_gate.h); `hs_drive_test.gd --hs-fly` flies the FPV drone along a route (physics
interest point 0) with `RG_FPV_CONFIG=<fast drone json>`, `--headless` works; experiment env knobs RG_STREAM_LOADER_WORKERS,
RG_STREAM_LOOKAHEAD_S/_CAP_M, RG_VIEW_INITIAL_THREADS.

Per session (`session.cpp`'s `Session::Terrain`): `g2m::phys::
PhysicsTileGrid` (the config's `SessionFrame`, 256 samples), a shared
`ResidentHeightSet`, a `HeightTileLoader` (`physics.loader_workers`, 16 in the shipped
configs: workers block in the shared derive pool, so a cold gate wait scales with them, vault PHYS/RG stream ticket) over
`TerrainModeConfig::fetch`, a `PhysicsTerrainStreamer` (default
`StreamerConfig`: gate r_tm+1, prefetch r_tm+2, look-ahead `physics.lookahead_s`/
`lookahead_cap_m` = 3 s / 510 m) and a
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
- Flat `initialize()` uses the SAME worker machinery (`run_flat_init_worker`, InitPhase atomics, `pending_session_`, cancel-then-join): it returns true once the worker STARTED (a turbo car's first engine builds a full-load envelope, seconds), `get_init_status()` reports loading/ready/error for flat too, `start()` only after "ready" (main.gd `_poll_loading` -> `_finish_flat_load`; the loading overlay shows for flat too). The spawn override ("Change car") is applied as `request_relocate` when the Session is adopted (`pending_flat_spawn_`).
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
| `fpv_drone` | yes | unattended | - (flying inputs live) | - (yaw/pitch are flying inputs) | `fpv` |

- `request_mode` refuses an unimplemented mode; `cycle_mode` skips them.
  Cycle order: drive -> free_cam -> drone_follow -> on_foot -> fpv_drone -> drive. Getting
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
  can never reach it, and it stays a positive int64 for Godot. The drone
  maths is `rg::drone_*` in `rg/camera_math.h` (`DroneParams`/`DroneState`, `drone_follow_step`: target-relative
  lag + snap beyond 60 m + orbit yaw from keys/stick; `drone_zoom_step`; `drone_probe_offsets` +
  `drone_clear_terrain`), bound as `RgCameraMath.drone_*` (state is a Dictionary the rig hands back; Godot Y-up <->
  core Z-up `(x, -z, y)` done in the binding); `drone_rig.gd` only forwards input, probes the ground and places the
  camera. The HUD label is `rg::drone_target_label` (player_mode.h). VR has its own `xr_drone` rig (`xr_rig.gd`
  `drone_follow`): same maths, the origin trails the target, the player's view is never rotated for them (yaw set
  once per activation to face the target, no orbit input); not tested on a headset. Tests: test_camera_math.cpp
  `[drone]` (golden against the former GDScript), `-Cameras` section 5.
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

## FPV drone

A pilotable first-person quadcopter, deliberately simple (owner, PR #20): kinematic, no real physics model, no wind, no
roll, no stabilisation logic. Engine-neutral logic in `rg_core`; GDScript only forwards input and draws.

- `rg/fpv_drone.h`/`fpv_drone.cpp`: `FpvDrone::step(FpvDroneInput, dt, FpvGroundProbe)`. Input: move right/up/forward in the
  drone's YAW frame (forward is horizontal whatever the pitch), yaw (+ = left) and pitch (+ = nose up), each -1..1. Velocity
  moves toward the wanted velocity with `accel_mps2` (speeding up) / `decel_mps2` (slowing, reversing); `max_speed_mps`
  horizontal and `max_vertical_speed_mps`; yaw/pitch rates follow the input with an exponential response, pitch is limited
  (`pitch_limit_deg`), the yaw wraps. Ground: the probe returns the ground height below (x, y) looking down from the
  highest of old/new height + `ground_probe_up_m`; the centre never goes below ground + `ground_clearance_m` (vertical
  speed zeroed, `on_ground`); no ground loaded (probe = nullopt) holds the height (`hold`) but still moves sideways. No
  side collision with buildings or cars (only the ray below, which also lands on the car roof). `FpvDroneState::activity`
  (0..1: acceleration, deceleration, turning) drives the audio. Tuning: `data/controls/fpv_drone.json` (`flight` and
  `audio` blocks, strict loader `load_fpv_drone_config`).
- `Session`: `request_fpv_spawn()` (above the chassis by `spawn_height_above_car_m`, facing along it, after the terrain gate)
  / `request_fpv_despawn()`, `set_fpv_input()`, `fpv_counters()`, `fpv_active()`, `FrameSnapshot::fpv`. Stepped in
  `step_once` next to the walker (`update_fpv`, ray = `World::backend().ray_cast`). Like the walker, the drone is physics
  interest point `kPlayerInterestId` and the parked car `kParkedCarInterestId`; `traffic_anchor()` is the drone.
- Mode: `PlayerMode::FpvDrone` (`rules_for`: car Unattended, driving inputs off, `flying_inputs_live`, rig `fpv`), never
  speed-gated. Bound in `RgSimulation` (`set_fpv_input(right, up, forward, yaw, pitch)`, `get_fpv_state()`,
  `set_fpv_drone_config(path)`, `get_mode_state().flying_inputs_live`); `apply_mode_to_session` spawns/despawns on a mode
  change like the walker.
- Controls: no new action ids. The camera group's actions are live in `fpv_drone` too (`actions.json`, mode bit
  `kModeFpvDrone`): `cam_move_x/z/y` = right/forward/up (WASD + E,Space / Q,Ctrl; left stick + RB/LB or triggers),
  `cam_look_x/y` = yaw/pitch (arrows; right stick). `main.gd _forward_flying`.
- Presentation: `game/scripts/fpv_rig.gd` (camera at the drone, rotation = yaw about Y, pitch about X, never roll, FOV 100),
  HUD line (speed, height, rotor load), `game/scripts/fpv_audio.gd` + `RgFpvSound` (`rg/fpv_drone_sound.h`: four detuned
  rotors, a blade-pass hum with harmonics plus air noise; idle hum, pitch 150 -> 420 Hz and level rise with activity,
  slower fall-off). VR keeps `xr_free` for this mode (no XR flyer rig).
- Tests: `test_fpv_drone.cpp` [fpv] (max speed, accel/decel limits, yaw frame, pitch limit/no tilt of the thrust, yaw/pitch
  rates, ground clamp, hold without ground, no tunnelling, determinism, config, rotor sound), `test_session_fpv.cpp`
  (flat Session, worker-count determinism), `test_session_terrain.cpp` [fpv] (interest points, hills), `-Cameras`
  section 6 (switch matrix, flying, leaving removes the drone), `test_player_mode.cpp`.

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
- **Deferred (PLAN R5/11.2/11.3):** pick-on-map spawn (needs R4's map), Garage / Events / Map & route menu entries, map server endpoint / pin region / cache size settings, resolution scale / FSR, input rebinding (done in R5b), gamepad pause button (done 2026-10-06: pad Menu/Start = Esc, the starter has no pad default), the walker rig in the camera-switch matrix (it needs a stepping simulation and a spawned walker; same contract). The drone rig is in the matrix since 2026-10-15 (`camera_switch_test.gd` section 5: first-frame continuity, settled pose independent of the previous view and of a rebase, back to each drive view).

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
- **Setups** `racing_game.vehicle_setup/1` (`<user dir>/garage/setups/<id>.json`): option id -> value; `data/vehicles/setup_options.json` whitelists JSON-pointer paths with ranges (final drive +-20 %, gear ratios +-20 % per gear and monotonic, brake force and bias (its share limits `share_min`/`share_max`, `max_shift` and `max_shift_bound` are required fields of the brake_bias entry), springs/dampers/ARBs +-30 %, tyre choice = any compound x any size the axle fitment allows (universal tyres, see below), assists = defaults of the assist.* channels, `assist_unit` = Street/Sport/Racing electronics part (a file_choice with `insert_if_absent` {array, entry, ref_key, stock_label}: a car whose stock file lacks the `assists` controller, i.e. the sedans, offers stock = "No assist unit" and any grade appends the controller entry), paint and rim colour visual only). The setup is a merge patch (RFC 7396) per file onto the vehicle file and the referenced engine/gearbox/tyre files, materialised into `<work_root>/drive_<id>_<hash>` and loaded with `rg::load_vehicle` (`ps::io::load_vehicle_json` + tyre injection); a loader error is shown verbatim and Save/Drive stay disabled. The drive uses the materialised copy (`main.gd` `_prepare_vehicle` -> `RgSimulation.set_vehicle_overrides`); work files are removed on `_unload_world` (`RgGarage.cleanup`) and on exit.
- **Garage set** (`data/garage/garage_set.json`, own procedural geometry, no external assets; Forza Horizon only as mood): dark room with ribbed walls, turntable with an orange ring and tick marks (slow spin), studio lights + softboxes. The garage was built on gl_compatibility (no SSR/probes; Mobile, the default since 2026-10-15, renders it the same way): reflections come from a sky shader that paints the softboxes plus a mirrored car twin (render layer 2, mirrored lights) seen through the glossy floor. Camera areas overview / wheels / engine bay (top) / rear; ISO to Godot: godot = (-iso.y, iso.z, -iso.x).
- **Teardown.** Leaving the garage (Back, Drive) removes the garage scene and every node it created; the car's visual (`body_visuals.gd clear_vehicle`) is released with the world, so node counts return to their pre-drive value (shell_flow_test and garage_test count them).
- **Tests.** Core: test_vehicle_catalog, test_vehicle_setup, test_garage, test_shell_flow. Headless: `tools/smoke_test.ps1 -Garage` (`garage_test.gd`, 106 checks: stock drive reads the wheel compression, Garage -> select -> front spring slider to x1.3 -> non-monotonic gear set rejected with the loader message, Save/Drive disabled, out-of-range final drive rejected -> Save -> a second RgGarage reads it -> Drive: the physics shows the front compression at 0.769 x stock and the rear unchanged -> pause -> Garage (respawn) -> sedan drives -> main menu: no world, no work file, node count back), in `tools/ci.ps1`. Screens: `--garage-shots <dir>` (needs a real window) writes vehicle_select / overview / turntable_rotated / edit_wheels / edit_engine / edit_rear PNGs (`out/r6_screens/`).
- **Universal tyres** (2026-10-13, `core/include/rg/tyre_spec.h`, `vehicle_load.h`): a tyre = compound (9 families in `data/tyres/families.json` `racing_game.tyre_families/4`, the ONLY authored tyre data) + size, id `<compound>_<width>_<aspect>r<rim>` (`track_325_30r21`). No tyre file per size: `TyreFamilies::make(spec)` builds the `ps::vehicle::TyreDesc` in memory by one rule (grip x (w/225)^0.35, crr x (w/225)^0.65, pkx1/pky1 x stiffness, fz0/vertical stiffness from section volume, qsy1/3/4 via `derive_rolling_resistance`); same spec = bit-identical desc. Vehicle files (`car_*.json`) have per-wheel `"tyre_spec": {compound,width_mm,aspect,rim_in}` and NO `"tyre"` path, so a raw `ps::io::load_vehicle_json` of them THROWS "no tyre": ALWAYS load through `rg::load_vehicle(path[, options])`, which passes the built tyres as `ps::io::VehicleLoadOptions::tyres` (physics_sim 6c5bb87; `TyreFamilies::make` sets a non-empty `source`). Fitment per axle by dimensions only in `data/vehicles/catalog.json` `setup.tyre_fitment.tyre_front/tyre_rear` {rims_in, min/max_width_mm, width_step_mm, diameter_tolerance_mm}; the garage lists every compound at every fitting size; errors name the reason (`tyre_rear: 255 mm is narrower than this axle's 275 mm minimum`). Old setups: `migrate_tyre_id` / `migrate_setup_tyres` map old stems and physics paths (hyper_rear_track_w255_r21, passenger_sport_225_45r17, physics stock files) to ids when a setup is read. UI metadata: `tools/generate_tyre_presentation.py` -> `game/assets/components/tyres/catalog.json`; `shell_ui.gd` `_tyre_presentation` parses the id. Tests `tests/unit/test_tyre_spec.cpp` [tyre_spec].
- **Gearbox swap** (2026-10-13, `gearbox_catalog.h`, `data/gearboxes/`, docs/garage_ui.md): option `gearbox_install` (Drivetrain, catalog "gearboxes"); rating lives in the CATALOGUE (`rating_nm`), not the physics file - physics_sim's GearboxDesc has no rating yet, then the field moves there; engine peak torque = `EngineStore::peaks` (curve engines read their WOT curve, cycle engines the catalogue's dyno-measured figure, user engines inherit their base's; unknown = no gearbox is turned away; N2O shot and extra boost are NOT counted). Automatic, Dual-clutch and CVT sections all have rows since 2026-10-16. Auto-shift: the car's rpm points and every drive mode's overrides leave room for each box's largest gear step (test_gearbox_swap `[tcu]`: step < up/down at zero and full throttle); short WOT drives on auto-shift show no grind events (`[drive][long]`). The straight-cut sequential box has no explicit dog mode in physics: it is a sequential box with a 90 N*m speed-match capacity and a wide dog window as stand-ins (open point for the physics session). Tests: `[gearbox_swap]` (catalogue, every compatible car x box loads validate-only, offered list == compatible set, rating message, setup round trip, Garage resets/drops).
- **Deferred.** 787b/pipes/gen/tractor variants are not listed; ABS/traction control/ESC options (not in the vehicle file); seat offsets are keyed by vehicle name at ready. Physics requests: chassis mass/box/inertia in the vehicle file, `load_vehicle_json_from_string` (no work files), real tyre compound data (`data/tyres/*.json` here are game-owned approximations).

## New cars: Pico, Ventura GT, Runabout T (2026-10-13)

Plan, design decisions and owner answers: `docs/new_cars_plan.md` (PR #2). Three cars with their own art, engines and cockpits; nothing of the hypercar changed.

- **Models.** `tools/cargen/` (Python, see its README: `build.py [ids]`, `validate.py`, `preview.py`; `cars/pico.py`, `ventura_gt.py`, `runabout_t.py`; shared pieces in `cargen/common.py`) writes `data/models/<car>/<car>.glb` + `.rig.json` (physics_sim.rig/1: nodes, joints, sockets incl. `socket_engine_mount`/`socket_driver_eye`, `engine_bay` box, hood variants) + `.cockpit.json` (rg.cockpit/1: every control with node, source, map, hit box, action) + `.flow.json` (the exterior surface for CFD, pinned to the glb sha256). Closed shells are visual closure, not yet a CFD-ready solid. LODs (2026-10-14): `<car>.glb` is LOD0, plus `<car>_lod1.glb`/`<car>_lod2.glb` with the same nodes/joints/materials, listed in the rig's `lods` (+ suggested `quality_tiers`, not wired into the game yet); tessellation per LOD in `cargen/quality.py` (chord tolerance, body stations), normals by angle `cargen/normals.py`, shared lamps/plates/grille bars/seals in `cargen/detail.py`; `tools/cargen/godot_shots.gd` renders garage-style review shots through Godot (xvfb works).
- **Engines.** `data/engines/pico_i3_1l0_na.json` (1.0 L I3 NA, 102.8 N*m @ 4300, 54.1 kW @ 5800), `ventura_v8_5l0_na.json` (5.0 L V8 NA, 470 N*m @ 4250, 234 kW @ 5250), `runabout_i4_2l9_sv.json` (2.9 L side-valve I4, 135 N*m @ 750, 18.5 kW @ 1550, limiter 2200); all simulated (cycle model, live engine sound, knock-limited ECU on E5) with installation and `data/engine_thermal/` files; the measured numbers are in each file's source. Rows in `engine_catalog.json`.
- **Cars.** `data/vehicles/car_city.json` (FWD, econ_5mt, economy 165/65 R14), `car_gt70.json` (RWD clutch LSD, close_ratio_6mt, belted 215/225 R15), `car_runabout.json` (RWD open diff, `data/gearboxes/vintage_3mt.json` 3-speed sliding mesh rated 160 N*m, cord 89/105 R21, rear brakes only - so its catalog `setup.options` leaves out brake_bias). Catalog entries carry `engine_bay_envelope` and (Runabout) `model_options: ["roof"]`.
- **Engine meshes and fit.** `tools/cargen/engine_mesh.py` builds a mesh + `rg.engine_envelope/1` (length/width/height in mm) for EVERY catalog engine from its physical data into `data/models/engines/<id>/`; each engine catalog row names its `envelope`. `rg::check_fit` (engine_swap.h) compares it with the car's `engine_bay_envelope` (crank along x or y). Every catalog car has a measured bay (car_city_bigbay = car_city with a declared bigger front clip, same files and art; 2026-10-15, how each was measured in its `source`, [AS] = estimated allowance); a car without one would take every engine. A refused swap says which dimension does not fit; the engine list of every car shows every store engine, a non-fitting one with "Does not fit: <reason>" in its detail (vehicle_setup.cpp build_setup_model). Full engine x car matrix test: test_engine_garage.cpp `[fit][matrix]` (prints ENGINE_FIT_MATRIX).
- **Swap package additions.** A narrow rpm band (idle..limiter) gets its auto-shift points inside the band; a launch rpm (assist unit / two-step) outside the new engine's band moves to 60 % of its limiter (an electronics file copy in the drive dir).
- **Garage.** The installed engine's mesh sits on `socket_engine_mount` and the hood opens in the engine bay camera area (`garage_scene.gd` `_update_bay`, `RgGarage.get_bay_engine`); the Ventura shows the raised hood (bulge/scoop) a tall engine needs (`model_options.gd hood_variant`). Setup option `folding_top` (kind bool + `model_option`: no file, visual only - weight and drag are not in the physics) shows the Runabout's top in the garage and the drive.
- **Cockpit.** `game/scripts/cockpit_controls.gd`, attached by `vehicle_visual.gd` to any model with a `.cockpit.json`: needles, steering wheel, pedals, handbrake, gear lever follow the car; left click acts (switch, knob/stalk/slider step, button, gear lever = shift up), shift+click / wheel steps back (gear down), pedals/levers/wheel drag, the ignition key sets the ignition (a 4-position key's start detent cranks); only while the camera is near `socket_driver_eye` and the mouse is free. `main.gd _forward_driving` pulls overrides/shifts/ignition/starter. Hazard, indicator stalk, horn and (V8) light switches submit DriverActions events (source CockpitMouse) through `cockpit_controls.gd driver_actions` (set by main.gd to the shared instance) and are posed from the shared projection (state + cabin.* channels); without that object they fall back to local state. Other cosmetic channels (fog, wipers, radio, climate) are state only (DriverActions has no action for them); screens show nothing yet. Headless check: `godot --headless --path game --script res://scripts/cockpit_test.gd` (186 checks, needs no GDExtension). Steering lock (B2): ONE field `steering_wheel_lock_deg` (degrees from centre to one lock; cabin.json = default for cars without a cockpit file, `.cockpit.json` `steering.steering_wheel_lock_deg` = per-car: pico 540, ventura_gt 612, runabout_t 225), read by cockpit_controls.gd and vehicle_visual.gd.

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
(`linux-arm64-release` turns it ON for the Steam Frame's `librg_godot.arm64.so`; a desktop Linux x86-64 preset with the extension is still missing, see docs/mobile_renderer.md).

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

Frame-time diagnostics (2026-10-14, render stutter): main.gd prints `RG_FRAME_STATS` once per second (fps, frame ms
avg/p99/max, frames over 25/50 ms, per probe avg/max ms) and `RG_FRAME_SPIKE` per frame over `RG_FRAME_SPIKE_MS` (default
50) with process_ms, render_cpu/gpu_ms (viewport render timing, off with `RG_RENDER_TIMING=0`), the terrain upload/heal/decks
ms (`RgTerrainView.get_frame_costs`) and the nodes' own `_process` costs. Nodes time themselves with
`game/scripts/frame_probe.gd` (`FrameProbe.add(name, usec)`; the node's `_process` wraps `_process_body`). A spike whose
probes sum to a few ms was spent outside the scripts (render, GPU, driver, OS). The 1 Hz bed probe is thinned to 800
samples (`get_bed_check` max_samples) and `rendered_vs_bed` prefilters by chunk footprint; hidden benchmark
`rg_unit_tests "[.][perf][stutter]"`.

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
setup.cmd                            # new machine: install toolchain, submodules, report map data
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
tools\ci.ps1                 # DEFAULT: release leg, ctest -LE long (parallel)
tools\ci.ps1 -Topic dyno,garage  # ... only the tests with these Catch2 tags (-WithLong adds the [long] ones)
tools\ci.ps1 -Full           # debug + release, every test incl. [long], all smoke runs
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
and matching .cpp files (the deck meshes are geo2map's since phase 3a, see below) implement game-side profile surfacing, separate bridge
slabs/tunnel floors/OSM-derived highway roofs, and configurable Gaussian terrain
smoothing. `WorldTerrain` retains an immutable raw-height cache and publishes
the processed level-0 heights to both physics and rendering. Geometry derives
from immutable elev.base; structural publication has a separate short lock.
`Session::sync_road_decks` installs nearby mesh colliders behind the gate through `rg::DeckInstaller` (S1: `deck_installer.h/.cpp`, shapes built on two own threads via physics_sim R2 `World::create_shape`, installed with `create_body(desc, handle)` in fixed (way_id, start, end) order, decks 600 m beyond the required radius prefetched, `kDeckInstallBudget` 16 per attempt, `SessionConfig::legacy_deck_install` = the old sim-thread build kept as the reference/A-B; `Session::deck_install_stats()`; tests `test_deck_installer.cpp` [deck_installer], hidden `test_deck_install_perf.cpp` [.][realdata][perf][s1]);
`RgTerrainView::sync_road_decks` owns independently rebased structural RIDs.
Settings and reconstruction bounds are in `docs/road_surfaces.md`.
**Grade-separated crossings (2026-10-15):** `rg::RoadSeparation` (road_structures.h; world config block `road_separation`: clearance_m 2.5, margin_m 3.0, approach_reach_m 60) - `RoadSurfacePatch::apply` never lets a road lift (lower) terrain inside another road's footprint + margin when it is more than the clearance above (below) it, and `build_road_decks` adds inferred approach decks (`RoadDeck::start_ref` now matched too) where a ground approach of an OSM bridge still overlaps a lower road, so render ribbons and collision no longer carry the embankment onto the road underneath. Tests `tests/unit/test_road_overpass.cpp` [road_overpass] (synthetic motorway overpass, negative control, hidden `[.][realdata]` case at utm32n/2/457/5422).
**S1 (2026-10-06):** pins physics_sim 606c428 (surface classes, N2O N2-N7, R2 prebuilt shapes, car_hyper_n2o) and geo2map_engine 072a321 (slice 9 carved layer; behaviour change: served bridge profiles end exactly at the stretch end, so `road_structures.cpp` `deck()` keeps the last station exactly - a float ulp had dropped way 32275368's deck). `data/surfaces/surfaces.json` has 13 surfaces: concrete, paving_stones, sett, cobblestone, gravel, compacted, sand added with the names of physics_sim's surface_conditions classes (wet grip applies); `core/include/rg/surface_kinds.h` maps every `g2m::SurfaceKind` to a name (Unknown -> asphalt, Water -> low_mu placeholder); values are dry [AS] numbers whose citations are UNVERIFIED (notes in each entry's source), test `test_surface_kinds.cpp` [surfaces][s1]. Nitrous: catalog entries `car_sedan_gen_n2o` and `car_hyper_n2o` (kW/Nm/kg only), `core/include/rg/nitrous_info.h` (`nitrous_info`, `nitrous_hud`: state off/ARMED/SPRAYING/PURGE/CUT, bottle kg/bar, flow g/s, kit fuel), `RgSimulation.get_vehicle_nitrous`, HUD line in `hud.gd`, action `toggle_nitrous` (pad Y, key N); spawn/HUD test `test_nitrous_spawn.cpp` [nitrous][s1] (both catalog cars spawn, HUD off -> ARMED); the armed-vs-unarmed gain measurement is done 2026-10-14: hidden `tests/unit/test_nitrous_gain.cpp` [.][nitrous][gain], table in PLAN.md; the hyper sprays only in Sport drive mode. Known physics caveat (physics_sim, not fixed here): with spray the hyper's boost can overshoot 1.8 bar to 2.0-2.4 bar and its turbo can sit on the rotor speed cap at 6000-7000 rpm.

`game/scripts/b8_contact_smoke.gd` is a SceneTree test launched with
`--script res://scripts/b8_contact_smoke.gd -- --drive --audio=godot`;
`b8_structure_shots.gd` captures seven owner-reported crossings. Unit regressions
cover smoothing halos/NoData/config bounds and local real-data road/collision
acceptance. `last_drive.json` in the existing Godot user-data directory records
session/UTM positions, build and status; the HUD/native build info reports the
source revision. Source libraries remain unchanged.

Current physics pin: ad401bc (physics_sim master = merge of PR #13, CVT gearbox type and DCT launch control; on top of master f38075e: contents mass #10, traction-control fix #11, automatic/DCT gearboxes #12); hash_check vehicle_step_steer state_hash 0xace4d38ea725e065. Older pin notes below are history.

Current physics pin: d11626d (physics_sim claude/tc-fix, PR #5 TC fix: closed-loop launch-control hand-over, Racing unit TC Sport retards the spark, Sport/Relaxed gains up; regression tests/unit/test_tc_owner_repro.cpp [tc_regression], hidden [tc_repro] diagnostics; hash_check vehicle_step_steer state_hash 0xace4d38ea725e065 unchanged - its car has no assist unit). Before: 4fe9a1f (physics_sim master after PR #8, the TC drift fix). Earlier pin history: 7d5316f (2026-10-05, from 9e94f0f, from 976e8d7). 9e94f0f -> 7d5316f brings weather W6 (water film, wet grip as a factor on SurfaceTable lambda_mu, tyre profiles): a no-op while no physics_sim environment with surface conditions is installed, and racing_game installs none; no source change needed, hash_check vehicle_step_steer state_hash 0xbab69b300eba41a3 unchanged. OSM building presentation and future mixed asset/imagery approach: docs/buildings.md.

Aerodynamics: docs/aerodynamics.md; configuration data/world/environment.json. Native surfaces, drafting wakes, ground effect and fan energy are owned by physics_sim; game Session supplies weather and active-wing policy, with latched HUD/art/bookmark telemetry.

Hypercar engine physical calibration: docs/hypercar_engine.md and external/physics_sim/data/engines/HYPER_ONE1.md. Fixed geometry/boost/E5; generated net 1 MW at 7500 RPM, game mass remains 1500 kg.

NPC truck: core/npc_truck plus Session stepping-thread collider/wake; background profile-route worker; game/scripts/npc_truck.gd procedural visuals. Controls and limits: docs/npc_truck.md.


NPC traffic: core/include/rg/npc_traffic.h and core/src/npc_traffic.cpp own public
road/speed/destination policies and background graph planning. Session owns the
physics-thread actor lifecycle, collision bodies and combined wakes. Adapter
get_traffic_state publishes frame-latched actors; game/scripts/npc_traffic.gd owns
presentation and F7 configuration. WorldTerrain::source_osm_tile is blocking,
background-only source access. Specification/current scope: docs/npc_traffic.md.
Junction turns (2026-10-14): `join_route_legs`/`junction_turn_cuts` (npc_traffic.h) replace the node-to-node lane join with a
curve between the served junction trims (tests `[junction_turn]`, docs/npc_traffic.md "Turn paths through junctions").
Right of way: `TrafficTrip::junctions` (rank, served give-way/stop line, signal phase), `junction_yields`/`traffic_signal_green`,
held in Session::update_traffic before the turn path (tests `[right_of_way]`, docs/npc_traffic.md "Right of way").
Stuck-NPC detector and fixes (2026-10-05): core/include/rg/traffic_stuck.h + core/src/traffic_stuck.cpp (stuck = speed < 0.5 m/s for > 5 s, cause = binding cap, wait-chain roots; `RG_TRAFFIC_STUCK` / `RG_TRAFFIC_STUCK_SUMMARY` logs), headless harness tools/traffic_probe (output in out/traffic_stuck/), results and remaining limits in docs/npc_traffic.md "Stuck NPCs".

Game server (plan only, nothing built): docs/game_server.md - authority model, protocol, interest management, deployment on the estate, increments S1-S9 and the required physics_sim seams; section 16.1 holds the owner's binding answers (30 Hz snapshots, environment time continues after a restart, remote cars silent apart from tyre audio, all other questions at the recommended defaults). Vault RACE-009.

G3/R3 consumer design (plan only, nothing built): docs/g3_consumer_design.md - geo2map pin bump strategy (bump early, delete only the game's profile completion), the carve switch delete map with file/line anchors, G3-E ribbons/junctions/interim markings draped on the Jolt-diagonal physics surface via an RgRoadView under the shared 0.8 ms upload budget, bridge decks from served profiles with deterministic bounded install (physics_sim R2 for off-tick shapes), the G3-F/R3 route harness (20-junction route, Fz residual bound calibrated on a 2 cm step, surface check against carve_footprint, repro bundles), slices S1-S14 and the requests to geo2map/physics_sim.

**High-speed render streaming (2026-10-13, owner complaint: at 400-580 km/h the terrain resolves late, roads are not visible far ahead, "I drove through the visible ground").** Render side only. (1) `camera_director.gd` feeds `RgTerrainView.update_focus_moving(x, y, vx, vy)` with the velocity-led focus (`RgRenderFocus`, velocity = `RgSimulation.get_chassis_session_pose()` vx/vy/vz, session frame); `road_stream.gd`/`building_stream.gd` read the same led focus (`render_focus_session()`), tiles behind a fast car go last. Free cam, drone and on foot get no lead. (2) `RgTerrainView::apply_diff` uses an adaptive per-frame upload budget (`set_upload_adaptive`, `set_upload_budget_max_ms`, default 0.8 ms up to ~3 ms), adds are sorted by time to reach (finest level first) inside the worker; the no-holes contract (adds before removals) is unchanged. `TerrainViewStreamer::update_focus(x, y, vx, vy, ref_x, ref_y)` and the reselect distance shrink with speed. (3) `WorldTerrain::carve_max_level()` (default 2, `set_carve_max_level` 0..2, preview flag `--carve-max-level=N`): L1/L2 chunks are carved with the same L2 road surface so coarse ground no longer rises above the ribbons; L1/L2 fall back to the smoothed uncarved tile when the road surface is not ready, L0 still fails loudly. Cost per chunk build 1.9 ms (carve 0) -> 2.9 ms (carve 2). (4) `road.gdshader` `depth_pull_per_m`/`depth_pull_near_m`/`depth_pull_max_m`: pulls the ribbon toward the camera along the view ray so it wins the depth test against coarse ground; computed in VIEW space (VERTEX is model space; the first version used it directly and hid every ribbon). (5) Instrumentation: `RG_TERRAIN_STREAM` and `RG_ROAD_STREAM` lines at 1 Hz from `main.gd _report_streams()` (speed, l0/l1/l2_ahead_m, pending adds/removals, upload ms avg/max, chunk latency p95, budget, lead_m, ground_above_m = visible surface above physics surface at 0/5/20/60 m, far_ground_above_m at 150/300 m; road pending/oldest_age_s/ribbon_ahead_m). Windowed tests: `--terrain-preview --stream-fly` (`game/scripts/terrain_fly_test.gd`, constant-speed fly, RG_FLY lines) and `game/scripts/hs_drive_test.gd` (SceneTree script, real-route pure-pursuit drive with the shipped hypercar, options --hs-speed/--hs-start/--hs-seconds/--hs-baseline/--hs-carve=N/--hs-shots=<dir>, RG_HS_DONE line; needs `--drive`).

**Gray patches over the road (2026-10-13, owner: cockpit at 428 km/h, gray where the road should be, "I can drive through the terrain").** Latent defect found and fixed; the owner's exact trigger is NOT reproduced (see below). (1) Defect: an L1/L2 tile whose road surface could not be derived (`WorldTerrain::produce_height`) returned the smoothed UNCARVED tile as Ready and kept it in the service cache and `height_cache_` for good, and `TerrainViewStreamer` never rebuilds a resident key - so such a chunk draws ground above the physics road bed (L0 physics stays carved) and hides the ribbons. Forced on real data (`RG_TERRAIN_FORCE_FALLBACK=100000`, 100 m/s on the A 5): rendered ground up to 1.0 m above the bed, 132 fallback chunks resident, a magenta band in the `RG_TERRAIN_TINT=carve` shot. (2) The old `ground_above_m` probes could not see it (they compare an L0 chunk with the L0 carved tile and skip points without a physics tile, i.e. every coarse chunk): they read 0.000 in the forced run while the real excess was 1.0 m, exactly like the owner's log. New probe `RgTerrainView.get_bed_check(px, py, dx, dy, ahead_m)` = rendered ground (highest uploaded chunk, any level) minus the physics ROAD BED centre line (`RoadSurfacePatch::bed_points`, `WorldTerrain::road_bed_points`, `rg::rendered_vs_bed`); `RG_TERRAIN_STREAM` gained `fallback_chunks fallback_tiles bed_above_max_m bed_above_p95_m bed_samples bed_worst_level bed_worst_carve`. (3) Fix: `RenderChunk::carve` (NotCarved/Carved/Fallback = worst state of the chunk's height window, `rg::chunk_window_carve`), `WorldTerrain::tile_carve/fallback_tile_count/heal_fallback_tiles` (re-carves fallback tiles from the retained uncarved tile once the road surface can be derived, replaces the `height_cache_` entry, old tile kept alive in `retired_tiles_` because chunk builds hold raw pointers), `rg::stale_fallback_chunks` picks the resident chunks built as Fallback that are carved now; `RgTerrainView::heal_tick()` (every frame, background `std::async` re-carve every 2 s while fallback tiles exist, then at most 2 chunk rebuilds per frame swapped in place by `replace_chunk_at`: upload first, free the old RIDs after, no hole). Forced run: bed_above_max 1.016 -> 0.066 m, fallback chunks 132 -> 0. (4) Debug aids: env `RG_TERRAIN_TINT=level|carve|both` (chunk colours by LOD level / carve state), `RG_TERRAIN_FORCE_FALLBACK=N` (first N coarse tiles produced as Fallback), `RG_TERRAIN_NO_HEAL=1`, stderr `RG_TERRAIN_FALLBACK ...` per fallback tile, `main.debug_hide(terrain, ribbons)` + `RgTerrainView.set_terrain_visible`; `hs_drive_test.gd` gained `--hs-route=<json>` (routes are densified to 10 m), `--hs-view=cockpit`, `--hs-hide-terrain/--hs-hide-ribbons`, `--hs-shot-t0/--hs-shot-step`, bed-probe fields in RG_HS_DONE; `data/routes/a5_corridor_probe.json` (the owner's A 5 southbound path from his log, corridor world). Unit test `tests/unit/test_terrain_carve_state.cpp` ([terrain_stream]). Heal rebuilds run off the main thread: `rg::ChunkRebuildJob` (terrain_stream.h, one batch of up to 8 keys in flight via std::async, cancel+join in release/load_preview/dtor); `heal_tick()` only swaps finished chunks in (2 per frame, key looked up again, dropped if gone or no longer Fallback) and `heal_ms` is that swap time. Unfixed/unknown: with intact data no fallback occurs on the A 5 route (5.2 km at up to 114 m/s, Compatibility and Mobile renderers: bed_above_max 0.07 m, 0 fallback tiles), so what made a road surface fail in the owner's session is unknown - the stderr `RG_ROAD_SURFACE fetch_failed` / `RG_TERRAIN_FALLBACK` lines of his console would tell. L3+ chunks (beyond ~1.3 km at 4 m/8 m grid and up) are still never carved; only the ribbon depth pull covers them.

## Streamers and buildings moved to geo2map (terrain migration phase 3b, 2026-10-15)

Since the geo2map pin `7426eb8` the render chunk selection and build, the render LOD streamer, the ring policy of the
road ribbon and building streams and the building extraction are geo2map's `g2m_world`, ported statement for statement
(libm -> `g2m::det`) and pinned in geo2map's `tests/world/test_view_streamer.cpp`, `test_ring_streamer.cpp` and
`test_buildings.cpp` against this repo's own code and scripts at 5a3baed:
- `rg::RenderChunk` = `g2m::world::ViewChunk`, `rg::ChunkCarve` = `g2m::world::ChunkCarve`, `rg::TerrainViewSource` =
  `g2m::world::ViewSource` (no `surfaces` field any more: a `colour` hook), `rg::TerrainViewStreamer`/`TerrainViewDiff`,
  `rg::select_view_keys`, `chunk_window_carve`, `TileCarveFn`, `reselect_distance_for_speed`, `chunk_time_to_reach_s`,
  `sort_adds_by_urgency`, `changed_road_lod_levels` and `rg::Building`/`BuildingPoint`/`BuildingGround`/
  `extract_buildings` are aliases or using-declarations. `build_render_chunks` / `build_static_view_from_lookup`
  stay as thin wrappers (same arguments) over `g2m::world::build_view_chunks` / `build_static_view` with
  `rg::chunk_colour_hook(surfaces)`.
- Colours stay here: `rg/terrain_colours.h` (a table keyed by the per-vertex class id, the hypsometric ramp for the rest,
  the RG_TERRAIN_TINT debug tint), applied on the build thread through `ViewSource::colour`; `WorldTerrain::view_source()`
  sets it from the world's road surfaces. `test_terrain_colours.cpp` holds the colour bytes recorded before the move.
- `road_stream.gd` / `building_stream.gd` keep their worker thread and the render nodes; which tile to build next, and
  which to show, downgrade (buildings: near part dropped beyond ordinary + 2048 m) or evict (beyond the distance +
  2048 m) comes from `RgRingStreamer` (`godot_ext/src/rg_ring_streamer.*`). Roads: one tier, a tile built empty stays
  known; buildings: full within ordinary + 725 m, skyline-only within skyline + 725 m.
- Release id unchanged (no served layer changed).

## Road ribbons and structure decks moved to geo2map (terrain migration phase 3a, 2026-10-14)

The ribbon mesh (everything in "Junction surfaces" and "Junction markings" below) and the structure decks (bridge
decks, tunnel floors, inferred tunnel roofs) are built by geo2map's `g2m_world` since the geo2map pin `2b6a6e5`:
`g2m::world::build_road_ribbons` / `add_junction_surfaces` / `RoadRibbonTile` (`g2m/world/road_ribbons.h`) and
`g2m::world::build_road_decks` / `road_deck_height` / `RoadDeck` / `build_structure_set` (`g2m/world/road_structures.h`),
ported statement for statement (libm -> `g2m::det`) and pinned byte for byte in geo2map's
`tests/world/test_road_meshes.cpp` against this repo's own code at 0c027dc. `core/src/road_visual.cpp`,
`rg/road_visual.h` and the mesh half of `road_structures.cpp` are deleted; `rg/road_structures.h` keeps
`complete_road_profiles` (until phase 4) and aliases `rg::RoadDeck`, `rg::road_deck_height`, `rg::build_road_decks`.
`WorldTerrain::road_visual_tile` returns a `g2m::world::RoadRibbonTile`; colours and the Godot upload
(`RgTerrainView::get_road_visual_tile`, road.gdshader) stay here. `rg::traffic_road_rank` forwards to
`g2m::world::road_priority_rank` (one rank table for traffic and paint). The junction-surface tests moved with the code
(geo2map `[world][road_ribbons][junction]`). The grade-separation approach decks (overpass fix above) moved too:
`rg::RoadSeparation` is `g2m::world::RoadSeparation` (`{clearance_m, margin_m, approach_reach_m, enabled = true}`),
`build_road_decks(geometry, separation)` / `build_road_ribbons(..., lift, separation)`; the world config block still
fills it in `WorldTerrain::open`, and `RoadSurfacePatch`'s own separation rule stays here (phase 4). geo2map pins the
port against this repo's 945455a output with separation on and off. The old `RG_STRUCTURE tunnel_roof` stderr line is gone (the library does
not print; inferred decks carry `inferred = true`). File names below refer to the geo2map port now.

## Junction surfaces (2026-10-13)

geo2map `src/world/road_ribbons.cpp` `add_junction_surfaces` (formerly `core/src/road_visual.cpp`, called at the end of
`build_road_ribbons`) draws one asphalt patch per junction the roads.geom tile owns (geo2map encoding 7
`RoadGeomTile::junctions`, road bed ring; declined and `OnStructure` junctions skipped). Ear clipping of the ring, then
every triangle cut by the terrain lattice lines (x.5 m), vertices shared, heights from the ground sampler, 4 mm above
the ribbons so the square ribbon ends vanish; a ring with a NoData vertex is dropped whole. Ribbon lane markings end at
the served trim (2 m ramp) where this tile owns the junction, else the old 3-11 m ramp. Rings carry geo2map's kerb
returns (D44, geo2map PR #1). A store derived before encoding 7 has no junctions: ribbons only, as before. Test
geo2map `tests/world/test_road_meshes.cpp` ([world][road_ribbons][junction]).

**Junction markings (2026-10-14).** `build_road_ribbons` (geo2map) also lays out the paint at served junctions (owner tile only):
- Ribbons follow the served cross-section: asphalt = `carve_footprint().roadbed`, markings over the counted lanes
  (lanes/forward/width from the counted bands when a cross-section exists, else the tags), a 0.3 m gravel verge strip
  (vertex `verge`, adapter bit 256) each side. The last ribbon row sits exactly at the stretch length (it used to land
  one ulp past it and drop the last metre, leaving a gap at every stretch end).
- Through road: the pair of approaches within 35 deg with the highest rank (StVO 18(3) ranks of `rg::traffic_road_rank`)
  continues its lane and edge lines over the junction as a cubic overlay (flag 32); not at a signal, a roundabout or an
  equal-rank crossing. Its edge line is broken (flags 64/128) across a side road's mouth (motorway: 0.30 m 6/6, else
  1.5/1.5) and hidden (512/1024) where it leaves the ring.
  Signalled junctions keep the through overlay too. Where a two-way road splits into two one-way ways at the junction
  (a dual carriageway joining up), the straight pair covers one direction only, so the two-way road's other lanes get
  a second overlay into the opposite one-way way; each strip covers only its half of the two-way road.
- Lines (flag 16, 4 mm above the junction): OSM stop lines (solid 0.5 m), give-way lines (0.5 m dashes, 0.25 m gaps) at
  OSM give-way and roundabout entries and implied on a lower-ranked side road off a through road (not at a merge under
  25 deg), crossings (zebra 0.5/0.5 bars, ladder, Furt = two broken 0.12 m lines, also the default at a signalised
  crossing without a pattern). A stop line whose centre lies on a placed crossing (+0.5 m) is not drawn: the crossing's line already stops the traffic.
- Turn guide lines: an approach whose leftmost two or more lanes are tagged only "left" (turn:lanes) gets a broken
  overlay (short dashes, lane and edge lines) from those lanes into the leftmost lanes of the arm 15-150 deg to the
  left nearest a right angle (not the through pair).
- Gores (sign 298): an exit or on-ramp (one-way, under 45 deg, through road and ramp both motorway/trunk/primary/
  secondary or their links) leaving or joining a through overlay gets the ramp's inner edge line: a curve from where
  the ramp leaves the through road (the first overlay row at which the paved ring reaches 0.5 m beyond the through
  bed edge), on the through road's edge line, into the ramp's own edge line at its lead; drawn only where it has
  cleared that edge line (on the ramp), so it never crosses the through lanes. Diagonal bars between it and the through
  road's edge line where they are 0.6 m apart, bordered on the through side. Only where a nose exists: the point halfway between the ramp's inner edge at its lead and the
  through edge lies outside the ring and off the far approach's lanes; otherwise (a lane gain/drop on a paved box)
  nothing is painted.
- Turn arrows (turn:lanes) stand 15 m before the stretch end, or further back so their tip ends 3 m before the first
  marking the traffic meets there (junction trim, a stop/give-way line of that direction, a crossing).
- Side roads closed to the public (access no/private/permissive/permit/...) never open an edge line; on a motorway or a
  motorway/trunk link a service road opens it only when it is plain (no service=*) and meets it within 35 deg (a
  rest-area ramp). Ribbons draw OSM surface=paving_stones/sett/cobblestone as stones without painted lines (flag 2048)
  and concrete with 5 m slab joints (flag 4096).
- Ribbons end 0.1 m under a served junction's trim line (owner tile; a stretch whose trims overlap draws none), so a
  ribbon wider than the ring no longer sticks out beside it as a step; the gravel verge (0.3 m, mitred) follows the
  road bed ring instead, kerb returns included, open across each approach's mouth.
- Junction patches where arms of different surfaces meet are split per surface instead of filled with asphalt: the major
  road (through pair of one surface, else the top-ranked arm) keeps a spine of its bed width across the ring, the rest of
  each lattice cell goes to the nearer minor arm (equal-distance line, arm rays clamped at the ring centre), with u/v along
  that arm so the paving pattern follows it. Unpaved arms (tracks; the terrain draws them) take part with a dirt piece
  (flag 8192) and never carry the major road while a paved arm meets there; a through pair may change paving (each
  keeps its own up to the line across the middle between their cuts); a minor arm only claims corners on its own side
  of the major road.
- The vertex flags no longer fit 4 bits: the adapter packs them over COLOR.b (low 8) and COLOR.a (high 8).
- Review tooling used: a geo2map scratch test dumping the mesh of both OSM fixtures and a Python software rasteriser
  (not committed).

## Cockpit control assets (P1, 2026-10-13)

`tools/cockpitgen/` builds and validates the game-owned `data/models/cockpit_parts/cockpit_parts.glb` + `.rig.json` from `data/cockpits/parts_spec.json`, importing the pinned assetgen library read-only. The library contains 22 standalone part trees (29 joints, 26 hit boxes), including four drive-mode buttons, TC LEDs, guarded nitrous and three shifter variants. Every part root has its own zero mounting origin; instantiate one tree at a socket, never the whole overlapping library. Rig axes/extras are ISO; GLB joint axes are converted to glTF by assetgen. Lamps start unlit. No physics/core/car-model or runtime wiring change. Commands and extraction contract: `tools/cockpitgen/README.md`. Independent rest/posed Blender sheets: `out/cockpit_parts_sheet.png`, `out/cockpit_parts_posed.png`. Action/state work C0/C1, car sockets M1, placement P2, clicks P3 and VR interactions remain separate.

### Common cockpit visual integration (2026-10-13)

`game/scripts/cockpit_common.gd` extracts only the common part trees, called by both `vehicle_visual.gd` and `garage_scene.gd` for car_hyper.glb (including N2O/engine-swapped hyper configurations that share that body). `data/cockpits/car_hyper_common.json` provides 18 mounts: two five-slot tunnel rows, start, hazard, lights, stalks, paddles and horn; no shifter or N2O. Missing car sockets use configured ISO-local fallback mounts. Actual sockets take precedence, reading normal/up from the supplied car rig, with explicit scene metadata overriding it. Horn/paddles are children of the wheel. Supported lamps project existing powertrain snapshot fields; missing/new-function telemetry stays unlit. No driver actions or physics logic is added by the visual integration. The current hazard is below the screen; the between-vents position awaits M1 furniture/socket work. Tests: `game/scripts/cockpit_common_test.gd` exercises actual drive/garage loaders, reflection idempotency, parent transforms, socket precedence and lamp mapping. Add `-- --capture` on a rendered Godot run for `out/cockpit_common_in_car.png`. C0/C1 and full fitted-state/click/VR integration remain pending.

### Cockpit mouse clicks (P3, 2026-10-18)

`core/include/rg/cockpit_hit.h` + `core/src/cockpit_hit.cpp` (`rg::CockpitHit`, engine-neutral, ISO model frame) pick the 17 mounted hyper controls with a ray (hit boxes from the generated parts rig, `pick_margin_m`), run push travel/threshold/re-arm/debounce from the rig, and emit `CockpitEvent`s whose actions are the SAME `DriverActions` ids as keyboard/Set input with source `CockpitMouse`: tc (left press / right set 0), lc, aero, a/m, eco/normal/sport/racing (set 0..3, only offered modes active, blank cap otherwise), start, hazard, lights (click, right = set 2), stalk_L/R (indicators), paddles (shift edges through main.gd's shift counters, not DriverActions ids), horn (held level, Set 1 at the press crossing, release at the up crossing, `held_actions()` re-asserts each frame). EPB has no binding: inert and blank. Layout: `data/cockpits/car_hyper_common.json` gained a required "mouse" block (`max_eye_distance_m`, `pick_margin_m`, `travel_per_s`) and a per-mount "control" block (mode push|click|hold, state id, primary/secondary binding, `mode_bit`, `blank_when_unavailable`). Binding `godot_ext/src/rg_cockpit.*` (`RgCockpit`; additive `rg_godot::driver_snapshot_from`); adapter `game/scripts/cockpit_mouse.gd` (world ray -> model ISO, hover box, ctl_ joint travel poses, blank caps; active only while driving live, world input active, cursor free, no menu open, eye within `max_eye_distance_m`; left = primary, right = secondary), wired in `cockpit_common.gd` / `vehicle_visual.get_cockpit_mouse()` / `main.gd _forward_driving` (`flush_to(driver_actions)`, `consume_shift_up/down`). pico/ventura_gt/runabout_t keep `cockpit_controls.gd`. Tests: `rg_unit_tests "[cockpit][hit]"` (8 cases: ray per control == keyboard/Set writes, miss, duplicate edge/debounce, inert controls, wheel-angle following), `godot --headless --path game --script res://scripts/cockpit_mouse_test.gd` (add `-- --capture` windowed for `out/hyper_cockpit_clicks/*.png`). Remaining for M1: hazard between the vents, pedals, door handles, shifter variants, N2O guard, wipers on stalk_R, stalk/light-dial pose visuals, real car sockets instead of the fallback mounts.

## Visual world: time of day, weather widget, headlights (V1/V2, 2026-10-14)

Plan and owner questions for grass, crops, paths, textures and weather: `docs/visual_world_plan.md`. Built: rg::TimeWeather (rg/sky.h, rg/time_weather.h, data/world/sky.json) drives sun, moon, sky colours and ambient through `RgSky` + `sky_controller.gd`; the top-right `weather_widget.gd` shows the icon, time, date and the air temperature (`get_aero_state().temperature_k`) and opens the time/weather menu (every preset enabled: V8 below). Headlights: DriverActions action `lights` (default key O; Press toggles dipped, Set 2 = main beam) -> channel `cabin.lights` (appended to kControlChannelNames) -> `headlights.gd` SpotLight3Ds (no shadows; each spot adds a draw pass per lit object in the gl_compatibility fallback; clustered, max 8 spots per mesh, on Mobile) and head/tail lamp emission.

Rear lamps (2026-10-15): `headlights.gd` also makes one red SpotLight3D at the tail/brake lamp materials (dim for tail, `brake_energy` while braking) and one white at the reverse lamps, aimed back and down, states pushed by `exterior_lamps.gd` (`set_states`); numbers in `data/world/headlights.json` (`tail`, `reverse`). Review shots: `game/scripts/night_lights_shot.gd`.

**V8 weather (2026-10-14).** `SessionConfig::weather` (rg::SessionWeather from `rg/climate.h`) installs a `ps::environment::Environment` (Preset mode, calendar locked to the game day, surface conditions + summer tyre profile, ISA-lifted air) in flat and terrain Sessions; the flat ground becomes a mesh tagged `climate.json ground_surface`; set_ambient/set_aero_environment are skipped while it is installed (the air comes from `World::vehicle_air`). Unset (tests, hash_check) = no change. `Session::set_weather/set_environment_clock` (applied on the stepping thread), `FrameSnapshot::weather` / `weather_state()`. Godot: RgSky loads climate.json beside sky.json (`get_climate_error`, state `air_temp_c`); `RgSimulation.set_weather_source(sky)` (main.gd), `sync_weather()` (sky_controller.gd, 10 Hz), `get_weather_state()`. The hypercar's rotary light switch is posed from `cabin.lights` (`data/cockpits/car_hyper_common.json` "poses", cockpit_common.gd). Measured numbers and limits (no snow grip loss before physics W8, minutes-long wetting at x1): docs/visual_world_plan.md.

**Automatic headlamps and exposure (2026-10-15).** `rg/exposure.h` (`ExposureConfig`/`ExposureController`/`combine_exposure`; sky.json `exposure`): display = cd/m2 x key x pi / adapted illuminance (clamped 50..150000 lux), from `SkyLighting::illuminance_lux` (sky.json key `illuminance_lux`, clouds, moon); `TimeWeather::sample()` adapts it, `RgSky.get_state()` reports it, `sky_controller.gd` writes it to `light_exposure.gd` (static vars). Lamps are photometric (headlights.json: spots in cd, lenses in cd/m2) x exposure, so they vanish against daylight and dominate at night; Compatibility keeps the sky at exposure 1 and scales only the lamps (it clips large sky/emission values). `DriverActions` headlamp switch `rg::LightMode` Auto/On/Off (the `lights` key O and the cockpit switch cycle it, default Auto; channel `cabin.light_mode`, projection id `light_mode` / lights label "AUTO ...", HUD line, tach AUTO lamp; persisted as the hidden setting `cabin.light_mode`, kept across a vehicle change). Auto = light sensor (`CabinConfig::auto_lights_*` from headlights.json `auto`, `DriverSnapshot::ambient_lux`): on < 1000 lux / off > 7000 lux, universal (no per-car or per-weather threshold; weather only changes the illuminance read). Details: docs/visual_world_plan.md "Automatic headlamps and exposure".

## Driver actions and shared control state (C0, 2026-10-13)

`core/include/rg/driver_actions.h` + `core/src/driver_actions.cpp` own the six existing TC/aero/LC/drive-mode/nitrous/auto-shift actions. `ActionEvent{id, DriverActionKind, value, ActionSource}`, `DriverActions::submit/apply`, `CabinState`, `ChannelSink`, `DriverSnapshot` and the single `ControlState state(id,snapshot,channels)` projection are Godot-free. `godot_ext/src/rg_driver_actions.*` only converts Variants. `input_map.gd` submits levels; `main.gd::_forward_driving` applies returned channel writes once per frame and drops blocked presses. `main.get_driver_actions()` exposes the shared instance. Legacy auto-shift preference persistence and forwarding order are preserved. `driver_state.gd`, HUD, digital cluster and common cockpit lamps read the shared projection. Existing engine/physics source is unchanged. New C1 functions and P3 clicking are not included.

Checks: five recorded-input/state cases (685 assertions), eight source kinds, native binding/actual keyboard input tests, actual drive/garage cockpit loader tests, release CI 543/543, flat and real-world drive smoke. The game Session hash is unchanged by C0 (also linked/tested without the driver-actions object); the separate ps_run scenario parity check currently differs, recorded in RACE-020 and out/c0. See out/c0/c0_report.md for detailed results and the dev-test closures. C1/C2 and M1 furniture/sockets remain pending.

## C1 park/EPB interface increment (2026-10-13)
Added Session park_lock, gear_request (initial sentinel -100), transmission.selector (reserved), and epb channels. The core DriverActions park action requests the physics park pawl; shifting releases park_lock. P now requests park instead of pausing; Esc/Start retain pause. The cluster shows P from published park_lock_engaged, never merely from the request. The imported physics supports park but not EPB, so EPB remains unfitted/unbound; core display projection handles future latched/handbrake state and refusal. No game-side EPB latch, automatic retry or dynamic stop. New actions appear in the controls schema. Remaining C1 work: timed start/stop, indicators/hazards/horn, exterior lamps and cockpit absolute actions; C2 and permanent car furniture/sockets still pending.

## C1 timed cabin controls (2026-10-13)
Implemented native DriverActions start_stop, hazard, indicator_left/right and held horn. CabinConfig is required and loaded from data/controls/cabin.json: crank timeout, stop speed, blink frequency, indicator wheel-angle arming/return thresholds, brake-lamp threshold; the same file supplies steering ratio and generated horn settings. There are no embedded timing defaults. Automatic cranking stops at published Running, timeout or manual ignition-off; I/K remain supported. Main synchronizes returned ignition changes into the input preference so the next frame cannot undo them. Blocked/typing/scripted driving cancels auto-crank and silences the horn. Hazards override both indicator lamps; a latched direction returns when the wheel turns toward it then crosses the return threshold. Brake/reverse and indicator emission use the car rig's material names/colours/strengths via exterior_lamps.gd; no car/physics asset rebuild. Cockpit hazard/start lamps and cluster arrows consume the shared core state. Generated looping dual-tone WAV horn follows held input, master volume and pause.
Defaults: F6 start/stop, F7 hazard, Z/X left/right, held J horn (all rebindable); P park from the previous increment. EPB remains unfitted with the currently imported physics; its shared refusal/latched state and cluster/cockpit lamp projection are prepared, without any game-side latch or retry. Screenshot out/c1_hazards.png uses the real car and a controlled hazard state fixture. Tests include native timing/cancellation/level/source cases, real keyboard dispatch, actual rig emission materials, generated horn PCM and normal flat runtime.
C0 and C1 game functionality are implemented; physics EPB availability remains external. C2 camera/wheel/cursor work and permanent M1 furniture/sockets remain next, followed by full P2/P3 configuration and interaction.

## C2 cockpit view basics (2026-10-13)
Native camera_math CockpitLook/cockpit_look_step integrate mouse and continuous rate-based keyboard/stick look, clamp pitch from config, ignore disabled input and sanitize non-finite values. RgCameraMath only converts dictionaries. The flat cockpit rig follows the configured driver eye and applies these yaw/pitch values. data/controls/cabin.json now supplies pitch limits (-60/+10), initial pitch, mouse sensitivity, look rate/deadzone, steering wheel lock (360 degrees per direction) and steering ratio. vehicle_visual rotates the steering_wheel about its own rig joint axis while preserving its rest basis; horn/paddles follow as children. drive_tour poses log cockpit_wheel_rad (steer 0.6 -> 3.7699 rad, matching configured lock).
Entering/resuming the flat cockpit captures/hides the cursor; middle click releases/re-captures. Leaving it or opening a menu releases; menu clicks cannot re-capture. Coordinate bookmarks move from the hard-coded middle-button handler to the remappable log_marker action, Delete by default. Marks retain their old persistent file and diagnostics, with source log_marker. Mouse capture's default binding is middle button.
C1 correction: F6/F7 were already fixed seat/traffic panel shortcuts. Start/stop now defaults to F11, hazards F12; native schema and actual keyboard tests updated. Z/X indicators, held J horn, P park and I/K remain unchanged.
Validation: release CI 552/552 (230 seconds); real-window and headless cockpit_view_test, actual C1 keyboard test, common cockpit integration and editor passed; rendered down-looking screenshot out/c2_cockpit_down.png verified. Headless cannot honor OS capture mode, so the test checks adapter state there and checks actual capture mode in the real renderer. drive_tour ran with no script errors and wrote wheel-angle evidence to out/c2_drive_tour/poses.txt. Its throttle phase stayed in neutral/at rest, so this is pose evidence, not a driving-performance test. No physics source/pin changes.
C0/C1/C2 are implemented (EPB remains unavailable until physics provides it). Next: M1 permanent car furniture/sockets, then finish P2 variants/fitted-state controls and P3 clicking, followed by VR hands/interaction. The car model updater is this session, per the owner's correction.

## In-game LOD settings and navigation (R4map, 2026-10-14)

Four LOD settings (terrain view distance, terrain detail, road detail preset, map detail) from `data/settings/lod_settings.json`,
applied live through `RgTerrainView.set_view_lod` (reselect + in-place rebuild of only the chunk levels whose road-LOD key changed)
and `RgNavigation.set_view_lod`; defaults = the behaviour before. Route planner (region graph from roads.graph tiles, ALT, guidance)
and minimap (`MapTileCache`) in `rg_core`, thin HUD in `navigation_hud.gd`/`minimap.gd`, destination from the F10 dialog. No turn
restrictions yet, no auto re-route, real-data routing unverified. Details: `docs/navigation.md`.

## Upload pipelining (C9, 2026-10-15)

`RgTerrainView` packs the Packed arrays of a diff's adds (and of the bulk load) on a `SurfacePackJob` worker; the main thread only calls `mesh_add_surface_from_arrays` + `instance_create2` for chunks already packed (`upload_next_chunk` waits otherwise), and polls the next diff only once the pack job is idle. Decks, LOD-setting rebuilds and heal swaps still pack inline (few chunks). Same arrays as before: the synthetic-scene screenshot (xvfb, opengl3) is byte-identical old vs new. `road_stream.gd` / `building_stream.gd` build their `ArrayMesh` on the worker thread (the main thread only attaches it: 3-4 ms -> < 0.7 ms for a 50k-vertex road tile) and re-place their tiles only when the render origin changed (was every frame over every tile). Measured with terrain_upload_bench.gd (475 chunks, 2.13 M vertices, 100 m/s, 120 fps cap, headless): per-chunk main-thread packing 0.16 -> 0.001 ms, streaming adapter time per frame avg 0.23 -> 0.13 ms (p99 ~3 ms = the adaptive budget). What remains is Godot's own surface encode in `mesh_add_surface_from_arrays` (0.3 ms/chunk headless, 0.6 ms under llvmpipe GL), still on the main thread.
## Frame-time fixes (perf, 2026-10-15)
Owner log (60 fps, 120 Hz, vsync on): scripts ~4.5 ms + render ~3 ms leave ~10 ms unexplained per frame, and `RG_FRAME_SPIKE` shows `road_stream` 20-47 ms / `building_stream` 8-24 ms spikes. Changes: tyre/engine fallback audio synthesis moved to C++ (`rg/tyre_sound.h`); `road_stream.gd` / `building_stream.gd` build their `ArrayMesh` on the worker thread (main thread only attaches it), and `building_stream.gd` plans its 40 km tile list (`_plan`, ~6500 tiles + sort) on the worker with `_known` guarded by the mutex. New diagnostics: `RG_FRAME_STATS` probes `scene_span` (first to last `_process` of the frame, `frame_probe_end.gd`) and `outside` (frame delta minus scene_span = engine render / driver / vsync wait / OS). `Performance.TIME_PROCESS` (`process_ms`) is the LONGEST process+draw of the previous whole second, not a per-frame mean; trust `scene_span`/`outside`.

Second pass (2026-10-15, owner log 40-45 fps on an RX 7900 XT; his build was 435498f, before C9 and the C++ tyre audio): `frame_probe_end.gd` splits "outside" by the SceneTree `physics_frame`/`process_frame` and RenderingServer `frame_pre_draw`/`frame_post_draw` signals into the probes `ph_loop` (OS events, _input, max_fps sleep), `ph_physics`, `ph_pre`, `ph_post` (deferred calls, canvas redraw, RS sync of worker-thread commands), `ph_draw` (RS draw incl. buffer swap / vsync wait), `ph_frame` (real wall time; the process delta is clamped by Godot) and `render_cpu`/`render_gpu` (main viewport); none in a headless run (no draw signals). `_draw` of `tach_gauge.gd`/`digital_cluster.gd`/`minimap.gd` is probed as `tach_draw`/`cluster_draw`/`minimap_draw`. `RG_FRAME_STATS` also prints `draw_calls objects primitives vsync max_fps size`. A/B switch: env `RG_PERF_AB=<feature>` (+ `RG_PERF_AB_S`, default 5 s) toggles one feature off/on every period and tags the stats lines `ab=<feature>:on|off` (`main.gd _perf_ab_apply`: shadows, terrain_shadows (`RgTerrainView.set_cast_shadows`), 3d, terrain, roads, buildings, car, hud, audio, sky). `hyper_live_cluster.gd`: the 1024x400 cluster SubViewport renders once per cluster redraw (20 Hz) instead of every frame (UPDATE_ALWAYS before).

Third pass (2026-10-15, owner log `perf_baseline.log` on PR-images: 4K, vsync off, ~81 fps median, 12.4 ms/frame): scripts 1.5 ms, `ph_post` 6.4 ms of which `minimap_draw` 3.8 ms, `ph_draw` 3.0 ms, GPU 3.5 ms. `minimap.gd` now draws its lines once, in session coordinates, on a child Node2D (`_MapLayer`) that is redrawn only when the map, route or size changes; a car move only sets that layer's transform (rendered output pixel-identical to before). Two ~0.5 s hitches came from `vehicle_audio.gd _initialize_audio`; it now prints `open_ms` per spatial-audio open and `start_ms`/`tyres_ms` for the engine voice and tyre setup, to find which step is slow.

Fourth pass (2026-10-15, perf PR stacked on #13; owner ~115 fps at 4K after #13, main thread bound: scripts + canvas + GL submission run in series on it). Redraw only on change: `tach_gauge.gd` draws its dial (face, ticks, numbers, rings) on a child `_DialLayer` recorded once per car / window size / drive-mode colour, and records the needle/gear/lamps/texts only when a value it shows changes (`_dynamic_signature`, needle quantised to 0.05 deg; blink phases count only while something blinks; a moved needle skips the signature); `cockpit_common.gd` / `exterior_lamps.gd` / `headlights.gd` push lamp material or light parameters only on a change; `navigation_hud.gd` asks for the map and guidance at 10 Hz (car marker every frame); `main._panel_open` uses `get_first_node_in_group`. Settings `graphics.shadow_quality` (high = four cascades to 300 m, terrain casts; medium two cascades; low two cascades to 150 m, terrain does not cast; off) and `graphics.render_scale` (0.5-1, bilinear 3D scaling, HUD stays native), both default to the previous look (`main._apply_graphics_quality`). Diagnostics: `RG_PERF_AB=<a,b,...>|all` cycles several features (on, off, next); `RG_FRAME_STATS` names `renderer=<method>/<driver> render_thread=<safe|separate>`; with `--render-thread separate` (or `RG_RENDER_TIMING=0`) the render-time reads and the draw-phase probes are skipped (`frame_probe_end.gd draw_phases`; they would sync the render thread every frame). Bench hooks for a flat start: `RG_BENCH_SYNTHETIC_TERRAIN=<metres>` streams the synthetic 20 km terrain (`RgTerrainView.load_synthetic_preview`) that many metres below the car, `RG_BENCH_SURFACES=1` prints `RG_SURFACES` lines (mesh instances, surfaces, shadow-casting surfaces, triangles per scene node). `tools/perf_summary.py <log>` prints medians, probes and the per-feature A/B table. `tools/run.ps1 -Renderer compat|mobile|forward_plus [-Driver opengl3|vulkan|d3d12] [-RenderThread safe|separate]` forwards Godot's `--rendering-method` / `--rendering-driver` / `--render-thread`. Measured counts (flat + synthetic terrain, 2026-10-15): 771 draw calls / 2.41M primitives in Compatibility (sun shadows 257 / 1.0M, car 325 incl. shadows from 160 mesh surfaces, terrain 232 / 2.0M, 2D 116); the same scene in Mobile 460 draw calls, Forward+ 775 (depth prepass); both render darker than Compatibility (lighting and tonemapping differ), so a renderer switch needs a light / tonemap retune.
Fifth pass (2026-10-15, after #14). Diagnostics are opt-in: env `RG_PERF=1` (`tools/run.ps1 -Perf`; `-Renderer`/`-Driver`/`-RenderThread` set it too; `RG_PERF_AB` and `RG_BENCH_SURFACES` imply it) turns on `FrameProbe.add` (a no-op behind `FrameProbe.enabled` otherwise), the viewport render timers, `frame_probe_end.gd` and the 1 Hz `RG_FRAME_STATS` / `RG_TERRAIN_STREAM` / `RG_ROAD_STREAM` / `RG_TRAFFIC_RENDER` lines; without it only a short `RG_FRAME_SPIKE delta_ms=... ticks=...` line per frame over 50 ms remains. `minimap.gd` draws every map line into one vertex-coloured mesh (a port of Godot's own polyline strip, `_strip` / `_edge_offset`): each `draw_polyline` was one canvas draw call, ~1400 per frame on a real map (the owner's draw calls with the 3D world off). `game/scripts/canvas_draw_bench.gd` (`godot --path game --script res://scripts/canvas_draw_bench.gd -- --widget=minimap [--lines=N] [--png=F]`) prints a widget's canvas draw calls; `RG_BENCH_SURFACES` also prints `RG_VIEWPORTS` (draw calls per viewport, 3D on and off), `RG_CANVAS_LAYER` (canvas draw calls per CanvasLayer) and `RG_CANVAS_BIG` (canvas items covering a quarter of the window). Car: `car_mesh_merge.gd` (vehicle_visual.gd after colours/lamps/cockpit; garage_scene.gd with `all_joints`, no shadow step) splits the model into rigid groups (root + every jointed / option / wheel node, the cockpit parts' `ctl_*` joints, `.cockpit.json` nodes), merges surfaces sharing a material inside a group into a `MergedStatic` child (material objects kept, also as surface overrides for the colour code), builds per group and cull mode one SHADOWS_ONLY `ShadowProxy` from the shadow casters (opaque + depth pre-pass glass in Compatibility) and takes the sun shadow off cockpit parts under 15 cm; car_hyper 325 -> 147 draw calls (flat 539 -> 361, flat + synthetic terrain 771 -> 593), `RG_CAR_MERGE=0` = off. `car_lods.gd` + setting `graphics.car_lod` (full default = LOD0 only; high/medium/low = the rig's `quality_tiers` switch distances): LOD1/LOD2 meshes as children of each LOD0 mesh node with visibility ranges (cargen cars only; NPC traffic draws boxes). `game/scripts/car_lod_bench.gd -- --model=<glb> [--tier=] [--out=]` renders one car at 8 / 30 / 100 m and prints draw calls / primitives. `hud.gd` read-out: one child Label per line of `Readout`, text set only on change.

## Surfaces, wet look and car materials (visuals PR #7, 2026-10-15)

- **Textures.** `data/textures/<surface>/` eight ambientCG CC0 sets (credits `texture_*`), `surface_textures.json` (scales, tints, strengths, wet block). `game/scripts/surface_materials.gd` (shared instance via `shared()`) packs them into S3TC Texture2DArrays (~16 MB VRAM, ~340 ms load) and drives the global shader uniforms of `project.godot` [shader_globals] (`rg_surface_detail`, `rg_origin_mod`, `rg_wetness`, `rg_puddles`). `terrain.gdshader`: vertex alpha is a two-value kind (1 natural, 0 road; `world_terrain.cpp` kKindNatural/kKindRoad), paved vs unpaved from the paint colour (r - b > 0.12); natural layers by greenness/slope; roughness-gated specular. `road.gdshader` textured by vertex flags (sett 16384, `road_visual.h`). Setting `graphics.surface_detail` off/low/high (off = the old flat colours; forced off under RG_TERRAIN_TINT).
- **Wet look and rain.** `rg/wet_look.h` + `data/world/wet_look.json` (rg.wet_look/1): wetness/puddles/rain/snow density from the weather state, bound as `RgSimulation.load_wet_look/get_wet_look`; `weather_fx.gd` (created by main.gd) sets the wet uniforms and draws rain streaks / snow (GPUParticles3D around the camera); setting `graphics.rain_density`. Tests `[wet_look]`.
- **Review scene.** `game/scripts/surface_shots.gd` (`godot --path game --script res://scripts/surface_shots.gd -- --out DIR [--detail off|low|high]`, needs a window/xvfb): synthetic hills, terrain road strips and ribbon roads, dry and wet.
- **Car materials.** `game/scripts/car_materials.gd` + `data/garage/car_materials.json` (rg.car_materials/1): paint clear coat, rims, glass (transparent materials named glass/vetro/window); used by vehicle_visual.gd (drive, always applied), garage_scene.gd and car_thumbnails.gd (kThumbnailRenderVersion 2). `exterior_lamps.gd` keeps one material copy per surface and lights the strongest active kind (tail and brake share a lens). vehicle_visual.gd adds rig sockets the .glb lacks as plain nodes (older exports without socket_driver_eye). Catalog entries of the five 2026-10-15 imports (Ace 11, Fairheaven LT 80, Kiri 86, LCT 3000, Shvan 92) wait in `tools/carimport/catalog_pending.json` until their LFS GLBs are regenerated (docs/car_model_import.md).

## RACING drive mode, EPB, ABS/ESC (2026-10-15)
Pin: physics_sim 2556af9 (claude/abs-esc stacked on claude/q3-racing-epb: drive mode 3 RACING, `epb` part with per-mode `epb_mode`, assist-unit `abs`/`esc` blocks). `rg/assist_info.h`: `kDriveModeCodeCount` 4 (RACING offered only when `drive_modes_available_bits` has bit 3, the cycle skips unoffered), `kAssistBitEpb/Abs/Esc` (7/8/9), `esc_mode_name/next_esc_mode/toggled_abs`, `AssistHud` abs/esc/epb fields. EPB: `car_hyper`/`car_hyper_n2o` carry the physics `epb` part and `car_hyper` the `drive_modes` part (racing_game-owned vehicle files); `epb_fitted/epb_state/epb_refused` come from the assist bits and snapshot, the `epb` action still writes only the switch LEVEL (physics refuses at speed in park_only, handbrake in RACING; an unfitted car writes nothing). ABS/ESC: channels `assist.abs`/`assist.esc` (Session adopts the physics defaults), `DriverActions::state("abs"|"esc")` (lamp label ABS / ABS ACTIVE / ESC ON / ESC SPORT ACTIVE, blink while intervening) shown in hud.gd and digital_cluster.gd; no key action for them yet. Garage: `assist_unit` has three more parts (`assists_{street,sport,racing}_abs_esc` = `data/electronics/assist_unit_*_abs_esc.json`, the physics grade file plus its abs/esc blocks, so TC modes and the wing automation stay). `drive_mode_theme.gd` RACING = red.

## Mobile renderer (2026-10-15)
Default renderer is Mobile (Vulkan, the Steam Frame's); Compatibility is the fallback. Forward+ is an optional third choice for desktop quality (the Steam Frame stays on Mobile through its launcher's `--rendering-method mobile`). Setting `graphics.renderer` (mobile | compatibility | forward_plus, `SettingDef::applies_on_restart`) -> `game/scripts/renderer_choice.gd` writes/removes `user://renderer_override.cfg` (project.godot `application/config/project_settings_override`), read by Godot at the next start; `--rendering-method` (run.ps1 `-Renderer`, release `play_compat.cmd`) wins; Godot itself falls back to OpenGL when Vulkan fails, the only automatic switch (owner: fallback stays manual). Owner: the physically correct light balance of Mobile/Forward+ stays (no brighter sun), exposure is tuned by PR #17. `main.gd` prints `RG_RENDERER ...`. `terrain.gdshader` `mean_colour()` linearises the sRGB `layer_mean` outside Compatibility. Test `game/scripts/renderer_choice_test.gd`, `[settings][renderer]`. Per-feature audit, lavapipe caveats and Forward+ check and follow-ups (Linux/ARM64 extension builds, MSAA, probes): `docs/mobile_renderer.md`.

## Railways (RACE-028, 2026-10-20)

Rail bridges over roads, ballast and rails on the ground, from geo2map's served layer `g2m.rail.geom` (geo2map decision G2M-026, `docs/formats/rail_geom.md` there). Opt-in: world config `"rail": true` (optional `"rail_config"`, default `rail.json` beside the config) puts the layer in the release, so the release id changes and a store derived for the plain release is not reused; shipped configs stay without it, `data/world/world_config_rhein_main_hessen_rail.json` is the hessen config with it (derived folder `baked_rail`, never the owner's `baked`).

- **Geometry (core, engine-neutral).** `rg/rail_meshes.h/.cpp`, numbers in `data/world/rail.json` (rg.rail/1, all [AS] except the gauge): `build_rail_tile_mesh` (per level-2 tile: bridge piece = deck slab + ballast trough + two parapets + rails; ground piece = ballast trapezoid sunk `ballast_depth_m` + rails; flat shaded, vertex colours, CCW outward in the east/north/up frame, local to the tile corner) and `build_rail_deck_meshes` (collision, below). The profile height of a piece is the top of the ballast, taken from the line (never the DEM under a span). Not drawn: tunnels (no portals in v1), declined pieces, subway and tram (config `draw.classes`); a `railway=construction` piece is drawn with deck and ballast but without rails (`construction_rails`).
- **Collision.** Only bridge deck slabs, nothing else (rails, beds, parapets, tunnels have none). A deck is a closed prism over the spans whose underside is at least `collision.min_clearance_m` (5 m) above the ground there (probed with `surface_height_exact` at the centre and 0.5 m inside both edges); a span with less clearance, or with unknown ground, keeps its look but a vehicle drives through it: a rail deck never blocks the road beneath. `WorldTerrain::rail_decks_near` turns them into `RoadDeck`s (way id = NEGATED rail way id, stations piece and piece + 1, so a DeckKey never collides with a road deck) and `Session::sync_road_decks` installs them with the road decks; unknown ground retries once a second, up to 20 times.
- **Render.** `RgTerrainView.get_rail_tile(x, y)` (vertices in Godot axes, indices reversed to clockwise, `origin_session`) and `get_rail_info()`; `game/scripts/rail_stream.gd` streams tiles like `road_stream.gd` (RgRingStreamer, worker thread, vertex-colour StandardMaterial3D), created by `main.gd` only when the world config says `"rail": true`.
- **Tests.** `tests/unit/test_rail_meshes.cpp` [rail] (config loader and rejections, volumes and extents against the analytic prisms, winding, bridge vs ground, class/tunnel/declined/construction switches, tracks, determinism, clearance-aware deck collision, the world-config key), each with a recorded sabotage whose numbers are printed (reversed winding -> negative volume, no end caps -> 33 % volume missing, no clearance test -> a 1.75 m gap over the road instead of 5 m).
- **Screenshots / drive-through.** `game/scripts/rail_shots.gd` (`godot --path game --script res://scripts/rail_shots.gd -- --drive`, window needed, env `RG_WORLD_CONFIG` = the `_rail` config, `RG_G2M_CACHE`, `RG_G2M_DERIVED`): finds the crossing at the owner spot (UTM32 E467577.92 N5550232.49) from the meshes, shoots from the road through the opening, from the side, along the rail and from above into `out/rail_shots/`, then drives the car through the opening in both directions.
- **Left for later.** Earthwork (embankment/cutting geometry), carving the terrain under rail bridges, level crossings, tunnel portals, tram, a rail graph/topology layer, catenary, trains, collision for anything but decks.
- **Submodule pin (for testing).** `external/geo2map_engine` is bumped to 5d43dd7 (geo2map `claude/rail-integration` = `claude/rail-layer` 97a168e merged with PR #20 3e26db7), because the layer is not on geo2map main yet. Repoint to main once G2M-026 is merged; no other `external/*` pin is touched.
