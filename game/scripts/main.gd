extends Node3D
const SurfaceMaterials := preload("res://scripts/surface_materials.gd")
const DriverState := preload("res://scripts/driver_state.gd") # preloaded once: this is called several times per frame
# game/scripts/main.gd — the one game scene (R2.2 R9), built procedurally in
# _ready() rather than hand-authored as a .tscn (a script-built scene stays in
# sync with RgSimulation's evolving method surface; game/scenes/main.tscn is
# a one-node stub with this script attached).
#
# Command line (user args after `--`):
#   (none)                  the game shell: boot splash, then the main menu
#                           (shell_ui.gd; the screen flow is rg::ShellFlow)
#   --flat                  flat test scene, Drive mode, straight in (no menu)
#   --drive                 real world (RG_G2M_HOME store), Drive mode, straight in
#   --free-cam              start in FreeCam mode instead of Drive (straight in:
#                           the flat scene unless --drive is also given)
#   --shell-user-dir <dir>  keep settings.json / last_drive.json in <dir> instead
#                           of user:// (the headless shell test uses this)
#   --shell-test            the headless UI flow test (shell_flow_test.gd;
#                           tools/smoke_test.ps1 -Shell)
#   --controls-test         the headless control configuration test (controls_test.gd,
#                           tools/smoke_test.ps1 -Controls); --controls-verify is its second
#                           phase in a NEW process on the same --shell-user-dir;
#                           --controls-shots <dir> screenshots the controls screen (controls_shots.gd)
#   --camera-test           the PHYS-008 camera-switch test (camera_switch_test.gd;
#                           tools/smoke_test.ps1 -Cameras), flat scene
#   --g2m-fetch-delay-ms N  real world: every tile fetch is delayed N..2N ms
#                           (g2m::phys::DelayedFetch) - forces gate freezes
#   --drive-smoke           scripted drive + relocation, prints RG_DRIVE lines
#                           and quits (drive_smoke.gd; tools/smoke_test.ps1 -Drive)
#   --screenshots <dir>     with --drive: the drive screenshot tour
#                           (drive_tour.gd); with --terrain-preview: the R2.1
#                           static tour (screenshot_tour.gd)
#   --road-shots <dir>      with --drive: relocates to four points along
#                           data/routes/home_r1_drive.json and screenshots
#                           each (road_shots.gd, roads_plan.md R-4)
#   --terrain-preview       the R2.1 static terrain preview: RgTerrainView
#                           alone, no Session, no car (unchanged; also
#                           --stream-test)
#   --bindings-test         the R7 binding smoke check (unchanged)
# Every flag above except --shell-test/--shell-user-dir (and every scripted run:
# --drive-smoke, --screenshots, --road-shots, --g2m-fetch-delay-ms) starts a
# world directly (rg::ShellFlow's DirectStart) without showing the menu. The
# flags only pick the STARTING world and mode: both change at runtime.
#
# Who decides what: the player-mode state machine and its rules live in
# rg_core (rg::PlayerModeMachine, via RgSimulation.set_player_mode/
# cycle_player_mode/get_mode_state) - which camera rig is active, which input
# groups are live, whether the car is driven by the player or parked
# unattended (the Session applies that itself), which world is loaded. This
# script only forwards: input_map.gd (devices -> values), camera_director.gd
# + chase_rig.gd/free_rig.gd (camera rigs, floating origin, render focus),
# hud.gd/tach_gauge.gd/loading_overlay.gd (drawing), body_visuals.gd
# (placeholder meshes).
#
# Process priority -2000: input is polled, mode/world actions are applied and
# the driving controls are forwarded BEFORE camera_director.gd (-1000) moves
# the rigs and rebases, and before body_visuals.gd (0) places the meshes.
#
# Shell (R5): rg_core's rg::ShellFlow (via RgShell) decides which screen follows
# which and hands back actions (show a screen, load/unload a world, pause, reset
# the car, save settings, quit); _apply_transition() carries them out. The
# screens are shell_ui.gd (boot, main menu, spawn picker, pause, settings,
# credits) and loading_overlay.gd (loading); the Drive screen is the world
# itself with the HUD. Esc pauses, Esc backs out of every other screen.
#
# World load flow: both worlds build their Session on a worker thread (flat =
# initialize(), real = initialize_terrain()) -> poll
# get_init_status() each frame, the loading overlay shows its numbers ->
# "ready": initial controls, start() -> first tick: RgTerrainView.
# initialize_shared(sim), load_preview at the chassis -> "running". F8 at any
# point (also while loading - R9's cancel-then-join returns promptly) releases
# the terrain view and loads the other world; the player mode is kept.

# The simulation's name of the car being driven (the vehicle file's "name"); set by
# _apply_vehicle from the garage's drive selection before every world load. "car_hyper"
# until the first load (the --bindings-test path and the catalog default).
var VEHICLE_NAME := "car_hyper"
const DriveModeTheme := preload("res://scripts/drive_mode_theme.gd")
const WORLD_LABEL :={"flat": "flat test scene", "real_world": "real world"}

# --- unified scene state (read by drive_smoke.gd / drive_tour.gd) ---
var world_kind: String = ""      # world being loaded or loaded: "flat" / "real_world"
var world_state: String = "none" # "loading" -> "starting" -> "running"; or "failed"
var fetch_delay_ms: int = 0
# Non-empty: overrides the driving input group (channel -> value), used by the
# scripted smoke and screenshot tour instead of an autopilot.
var scripted_controls: Dictionary = {}
# On-foot counterpart of scripted_controls (drive_smoke.gd): while non-empty
# {move_right, move_forward, look (Godot-frame Vector3), run} replaces the
# player's walking input.
var scripted_walk: Dictionary = {}
var ready_sim_time: float = -1.0 # sim time when the current world became "running"

var _steering = preload("res://scripts/adaptive_steering.gd").new()
var _simulation: Node
var _input_map: Node
var _drive_view: String = "chase" # chase / bumper / cockpit / orbit / cinematic (rg::DriveView)
var _drone_target_seen: int = -1 # last drone_target_id printed (-1 = own car)
var _walker_rig: Node3D
var _fpv_audio: Node
var _walker_visual: Node3D
var _refusals_seen: int = 0 # get_mode_state()["get_out_refusals"] already reported to the HUD
var _enter_refused_seen: int = 0 # ... ["walker_enter_refused"]
var _mode_seen: String = "drive"
var _seat_ui: CanvasLayer
var _director: Node
var _hud: Node
var _sky: Node            # sky_controller.gd
var _weather_widget: CanvasLayer # weather_widget.gd (top right)
var _weather_fx: Node3D          # weather_fx.gd (wet look, rain / snow)
var _overlay: Node
var _visuals: Node
var _roads: Node3D
var _buildings: Node3D
var _world_view: Node # RgTerrainView shared with the real-world Session
var _load_started_ms: int = 0
var _last_report_ms: int = 0
const FRAME_SPIKE_S := 0.05
const FrameProbe := preload("res://scripts/frame_probe.gd")
var _fd_spike_s := FRAME_SPIKE_S # env RG_FRAME_SPIKE_MS overrides (set in _ready)
var _fd_frames := PackedFloat32Array()
var _fd_probe_sum: Dictionary = {}
var _fd_probe_max: Dictionary = {}
var _fd_process_sum := 0.0
var _fd_process_max := 0.0
var _fd_last_report_ms := 0
# Render A/B (frame-time diagnosis 2026-10-15): env RG_PERF_AB=<feature> switches one feature off and on again every
# RG_PERF_AB_S seconds (default 5) while a world runs, and RG_FRAME_STATS carries ab=<feature>:on|off, so one log
# compares the same scene with and without it. Features: shadows (sun shadow), terrain_shadows (terrain chunks drawn
# into the sun's shadow splits), 3d (the main viewport's whole 3D render), terrain, roads, buildings, car, hud, audio
# (vehicle_audio _process), sky (sky background -> clear colour), navigation (minimap + guidance layer), traffic (NPC
# traffic visuals). RG_PERF_AB may name several features separated by commas, or "all": they are measured one after
# the other (feature 1 on, off, feature 2 on, off, ...) and repeat; tools/perf_summary.py prints the on/off table.
var _ab_feature := ""
var _perf := false # RG_PERF=1 (or RG_PERF_AB / RG_BENCH_SURFACES): frame probes, render timers, 1 Hz perf lines
var _render_timing := true # viewport render times + draw-phase probes (off with RG_RENDER_TIMING=0 or a separate render thread)
var _ab_list := PackedStringArray()
var _ab_index := 0
const AB_ALL := ["3d", "shadows", "terrain_shadows", "terrain", "roads", "buildings", "car", "traffic", "hud", "navigation", "sky", "audio"]
var _traffic_visual: Node
var _ab_period_ms := 5000
var _ab_off := false
var _ab_switched_ms := 0
var _ab_saved_bg := -1
var _spawn_reported: bool = false
var _drive_diag_history: Array[Dictionary] = []
var _drive_diag_last_sample_s: float = -1.0e9
const DRIVE_DIAG_HISTORY_S := 5.2
const DRIVE_DIAG_SAMPLE_PERIOD_S := 0.1

# --- shell (R5) ---
var _shell: Node # RgShell: flow, settings, credits, spawn presets
var _shell_ui: CanvasLayer
var _controls: Node # RgControls (R5b): the player's control configuration, loaded before the input map exists
# --- garage (R6) ---
var _garage: Node                 # RgGarage: catalog, setups, edit session, drive hand-over
var _garage_scene: CanvasLayer    # garage_scene.gd while the garage is open, else null
var _thumbs: Node                 # car_thumbnails.gd while the car browser is up, else null (R6c)
var _catalog_override: String = "" # --catalog <path>: a different vehicle catalog (tests, the 200-car synthetic one)
var _drive_selection: Dictionary = {} # RgGarage.prepare_drive() of the car being driven
var _direct_vehicle: String = ""  # direct-start flags drive --vehicle <id> or the catalog default, never the saved choice
var _teleport_dialog: CanvasLayer
var _gauge: Control
var _screen: String = "boot" # mirrors rg::ShellFlow's screen
var _direct_start: bool = false # a start flag skipped the menu
var _world_request: Dictionary = {} # what the flow asked to load (place label, address search)
var _paused: bool = false
var _panels_open_prev: bool = false # a panel was open at the end of the last frame (Esc then belongs to it)
var _mouse_captured_prev: bool = false # likewise for the captured mouse (Esc then releases it)
var shell_log: PackedStringArray = PackedStringArray() # "from>to" of every accepted transition (shell_flow_test.gd)

# Held as a script member (NOT a local var) so the Resource stays alive for as
# long as Main does - a local ShaderMaterial would be freed (and its
# RenderingServer material RID with it) when the building function returns,
# leaving RgTerrainView holding a dangling RID. Used by both the preview and
# the real-world view.
var _terrain_material: ShaderMaterial
# --terrain-preview only.
var _terrain_view: Node
# R4 minimap + route guidance (navigation_hud.gd; follows the real world itself).
var _navigation: CanvasLayer
var _terrain_preview_reported: bool = false
# --terrain-preview without --screenshots/--stream-test: the fly camera whose
# position drives RgTerrainView.update_focus every frame (R2.2 R8).
var _terrain_focus_camera: Camera3D

func _data_path(relative: String) -> String:
	# game/ is its own Godot project root (res://); external/physics_sim/data
	# lives one level up, OUTSIDE res:// entirely, so build the absolute
	# filesystem path by hand.
	var project_root: String = ProjectSettings.globalize_path("res://")
	return (project_root.path_join("../external/physics_sim/data").path_join(relative)).simplify_path()

func _world_config_path() -> String:
	# Default to the largest region, including the corridor to Mannheim.
	# RG_WORLD_CONFIG (env) picks another world, e.g.
	# data/world/world_config_rhein_main_hessen.json (relative to the repo root).
	var project_root: String = ProjectSettings.globalize_path("res://")
	var override := OS.get_environment("RG_WORLD_CONFIG")
	if override != "":
		return override if override.is_absolute_path() else (project_root.path_join("..").path_join(override)).simplify_path()
	return (project_root.path_join("../data/world/world_config_rhein_main_corridor.json")).simplify_path()

# racing_game's OWN data/ (data/vehicles/, data/engines/ - carvis brief
# 2026-09-27, the racing_game-owned 3.0 L sedan variant: physics_sim is
# read-only, so a vehicle file that needs its OWN engine/clutch numbers lives
# here instead of in external/physics_sim/data/vehicles). Sibling of
# external/, same "one level up from res://" pattern as _data_path.
func _rg_data_path(relative: String) -> String:
	var project_root: String = ProjectSettings.globalize_path("res://")
	return (project_root.path_join("../data").path_join(relative)).simplify_path()

func _ready() -> void:
	# Window close is handled in _notification: the audio players are stopped and the
	# audio thread given time to release their generator playbacks before quitting.
	get_tree().set_auto_accept_quit(false)
	var ab_env := OS.get_environment("RG_PERF_AB").strip_edges().to_lower()
	for f in (",".join(AB_ALL) if ab_env == "all" else ab_env).split(",", false):
		_ab_list.append(f.strip_edges())
	_ab_feature = _ab_list[0] if not _ab_list.is_empty() else ""
	var ab_s := OS.get_environment("RG_PERF_AB_S")
	if ab_s.is_valid_float() and float(ab_s) >= 1.0:
		_ab_period_ms = int(float(ab_s) * 1000.0)
	# Performance diagnostics are opt-in (owner, 2026-10-15: no cost in a normal game): RG_PERF=1 turns on the frame
	# probes, the GPU/CPU render timers, frame_probe_end.gd and the 1 Hz RG_FRAME_STATS / RG_TERRAIN_STREAM /
	# RG_ROAD_STREAM / RG_TRAFFIC_RENDER lines; RG_PERF_AB and RG_BENCH_SURFACES imply it. Without it only RG_FRAME_SPIKE
	# stays (a delta comparison per frame; the line then has no probe or render-timer fields).
	_perf = OS.get_environment("RG_PERF") == "1" or not _ab_list.is_empty() or OS.get_environment("RG_BENCH_SURFACES") != ""
	FrameProbe.enabled = _perf
	var spike_env := OS.get_environment("RG_FRAME_SPIKE_MS")
	if spike_env.is_valid_float() and float(spike_env) > 0.0:
		_fd_spike_s = float(spike_env) / 1000.0
	# With --render-thread separate, reading the render times (and the draw signals' phase split) would make the main
	# thread wait for the render thread every frame, i.e. undo the overlap; there frame_ms is the measure.
	_render_timing = _perf and OS.get_environment("RG_RENDER_TIMING") != "0" and _render_thread_mode() != "separate"
	if _render_timing:
		RenderingServer.viewport_set_measure_render_time(get_viewport().get_viewport_rid(), true) # RG_FRAME_SPIKE render_cpu/gpu_ms
	preload("res://scripts/ui_scale.gd").apply(get_tree())
	if _perf:
		var probe_end := Node.new()
		probe_end.name = "FrameProbeEnd"
		probe_end.set_script(preload("res://scripts/frame_probe_end.gd"))
		probe_end.set("draw_phases", _render_timing)
		add_child(probe_end)
	var user_args := OS.get_cmdline_user_args()
	if "--bindings-test" in user_args:
		_run_bindings_test()
	elif "--terrain-preview" in user_args:
		_build_terrain_preview_scene()
	else:
		_build_scene(user_args)

# Flat initialize() builds its Session on a worker: wait for get_init_status() "ready".
func _bindings_wait_ready(sim: Node) -> bool:
	var began := Time.get_ticks_msec()
	while true:
		var st: Dictionary = sim.get_init_status()
		var state := str(st.get("state", ""))
		if state == "ready":
			return true
		if state == "error" or Time.get_ticks_msec() - began > 120000:
			push_error("bindings test: flat init did not become ready: %s" % str(st))
			get_tree().quit(1)
			return false
		await get_tree().process_frame
	return false

func _run_bindings_test() -> void:
	# R2.2 R7 smoke-only check (tools/smoke_test.ps1 -BindingsTest, task 3 of
	# that milestone's brief): proves the new RgSimulation/RgTerrainView
	# methods exist and that a flat initialize() -> initialize() re-init
	# cycle (the "runtime world switch", no restart) completes without a
	# crash. Deliberately never calls initialize_terrain()/
	# RgTerrainView.initialize_shared() with real data - that needs
	# RG_G2M_HOME plus a geo2map cache, unavailable in CI. Does not touch
	# _build_scene()/_build_terrain_preview_scene() or any of their nodes -
	# a fully separate path, same "leaves the other branches alone" pattern
	# _build_terrain_preview_scene() already follows.
	var sim: Node = ClassDB.instantiate("RgSimulation")
	sim.name = "BindingsTestSim"
	add_child(sim)

	var vehicle_json: String = _data_path("vehicles/%s.json" % VEHICLE_NAME)
	var surface_table_json: String = ProjectSettings.globalize_path("res://../data/surfaces/surfaces.json")

	var ok: bool = sim.initialize(vehicle_json, surface_table_json)
	if not ok:
		push_error("bindings test: first initialize() failed: %s" % sim.get_last_error())
		get_tree().quit(1)
		return
	if not await _bindings_wait_ready(sim):
		return
	sim.start()

	# Controls (owner 2026-10-05): the gamepad-Y nitrous toggle writes the pre-seeded
	# "nitrous_arm" channel; the render-interpolation diagnostics binding exists.
	sim.set_control("nitrous_arm", 1.0 - sim.get_control("nitrous_arm"))
	if sim.get_control("nitrous_arm") != 1.0:
		push_error("bindings test: the nitrous_arm channel did not take the toggle (is %s)" % sim.get_control("nitrous_arm"))
		get_tree().quit(1)
		return
	sim.set_control("nitrous_arm", 0.0)
	var render_diag: Dictionary = sim.get_render_diagnostics()
	if not render_diag.has("frames_late"):
		push_error("bindings test: get_render_diagnostics() lacks frames_late")
		get_tree().quit(1)
		return

	var limit: Dictionary = sim.get_vehicle_speed_limit(VEHICLE_NAME)
	if limit.get("kind", "") != "unknown" or bool(limit.get("road_found", true)):
		push_error("bindings test: flat-world speed limit should be unknown")
		get_tree().quit(1)
		return

	# New methods exist and read sanely in flat mode.
	if bool(sim.is_terrain_mode()):
		push_error("bindings test: is_terrain_mode() true right after a flat initialize()")
		get_tree().quit(1)
		return
	var init_status: Dictionary = sim.get_init_status()
	var streaming_status: Dictionary = sim.get_streaming_status()
	if bool(streaming_status.get("terrain_mode", true)):
		push_error("bindings test: get_streaming_status().terrain_mode true in flat mode")
		get_tree().quit(1)
		return
	var _origin: Vector3 = sim.get_render_origin_session() # must not crash
	sim.retry_failed_tiles() # must not crash (a documented no-op without a terrain Session)

	# Re-init cycle (Task 1's own requirement): initialize() again on an
	# object that already holds a running Session must stop/destroy the old
	# one cleanly and build the new one - no restart, no leak, no deadlock.
	ok = sim.initialize(vehicle_json, surface_table_json)
	if not ok:
		push_error("bindings test: second initialize() (re-init) failed: %s" % sim.get_last_error())
		get_tree().quit(1)
		return
	if not await _bindings_wait_ready(sim):
		return
	sim.start()
	if not bool(sim.is_running()):
		push_error("bindings test: sim not running after re-init")
		get_tree().quit(1)
		return
	sim.stop()

	# RgTerrainView: methods exist; a null-sim call fails cleanly (no
	# crash); release() is safe to call again on an already-empty view.
	var terrain_view: Node = ClassDB.instantiate("RgTerrainView")
	terrain_view.name = "BindingsTestTerrainView"
	add_child(terrain_view)
	var shared_ok: bool = terrain_view.initialize_shared(null)
	if shared_ok:
		push_error("bindings test: initialize_shared(null) unexpectedly returned true")
		get_tree().quit(1)
		return
	terrain_view.release()
	terrain_view.release() # idempotent - must not crash called twice

	print("bindings test: ok init_status_state=%s streaming_terrain_mode=%s" % [
		init_status.get("state", "?"), streaming_status.get("terrain_mode", true)
	])
	get_tree().quit(0)

func _build_terrain_preview_scene() -> void:
	# PLAN.md R2.1: "a --terrain-preview branch in main.gd ... Camera3D (near
	# 0.25, far 25000), DirectionalLight3D, simple sky/environment,
	# RgTerrainView, fly cam placed above spawn at a height that shows the
	# ridge; no Session/vehicle. Without the flag, behaviour unchanged." This
	# function touches NOTHING _build_scene()/its helpers use (no
	# RgSimulation, no chase_cam/input_map/hud/tach_gauge) - a clean
	# alternate scene, not a variant of the normal one.
	var terrain_view: Node = ClassDB.instantiate("RgTerrainView")
	terrain_view.name = "Terrain"
	add_child(terrain_view)
	_terrain_view = terrain_view

	var ok: bool = terrain_view.initialize(_world_config_path())
	if not ok:
		push_error("RgTerrainView.initialize failed: %s" % terrain_view.get_last_error())
		return

	# A/B measurement: --carve-max-level=N (0 = only L0 is carved with the road, the
	# pre-2026-10-13 behaviour; default 2).
	for a in OS.get_cmdline_user_args():
		if a.begins_with("--carve-max-level="):
			terrain_view.set_carve_max_level(int(a.get_slice("=", 1)))

	# game/shaders/terrain.gdshader: vertex colour (the hypsometric ramp
	# RgTerrainView uploads as ARRAY_COLOR) + Lambert - see set_material's
	# own doc comment for why this call is required for the colours to be
	# visible at all (Godot's default material ignores vertex COLOR).
	_terrain_material = ShaderMaterial.new()
	_terrain_material.shader = load("res://shaders/terrain.gdshader")
	SurfaceMaterials.shared().apply_terrain(_terrain_material)
	terrain_view.set_material(_terrain_material.get_rid())

	var spawn_x: float = terrain_view.get_spawn_x()
	var spawn_y: float = terrain_view.get_spawn_y()
	ok = terrain_view.load_preview(spawn_x, spawn_y)
	if not ok:
		push_error("RgTerrainView.load_preview failed: %s" % terrain_view.get_last_error())
		return

	# tools/smoke_test.ps1's --terrain-preview mode and this task's own
	# report both grep for this line (chunk selection/build_static_view
	# wall time alone - _process's own "terrain preview loaded" line below
	# reports the per-chunk RenderingServer upload time separately, once
	# every queued chunk has actually been drained through the upload
	# budget).
	print("terrain preview selected: chunks=%d vertices=%d build_ms=%.2f" % [
		terrain_view.get_chunk_count(), terrain_view.get_total_vertex_count(), terrain_view.get_last_build_time_ms()
	])

	# render_origin = spawn snapped to integer metres (PLAN.md R2.1's own
	# floating-origin convention, see set_render_origin's doc comment) - "up"
	# stays 0.0 (no terrain-height lookup here; the residual z spread across
	# this one home region, roughly 90-880 m ASL, is still far below float
	# precision concerns at these chunk sizes).
	terrain_view.set_render_origin(Vector3(roundf(spawn_x), roundf(spawn_y), 0.0))
	SurfaceMaterials.set_render_origin(Vector3(roundf(spawn_x), roundf(spawn_y), 0.0))

	# --- lighting + sky (same minimal setup as _build_scene's, standalone
	# here since this branch never calls _build_scene) ---
	var sun := DirectionalLight3D.new()
	sun.name = "Sun"
	sun.rotation_degrees = Vector3(-55.0, -35.0, 0.0)
	sun.light_energy = 1.1
	sun.shadow_enabled = true
	add_child(sun)
	var env_node := WorldEnvironment.new()
	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	env.sky = Sky.new()
	env.sky.sky_material = ProceduralSkyMaterial.new()
	env_node.environment = env
	add_child(env_node)

	# --- fly camera, placed above spawn at a height that shows the ridge
	# (this home region's terrain spans roughly 90-880 m ASL per the repo
	# CLAUDE.md's terrain data notes - 600 m above the spawn point's own
	# render-origin-relative (0, 0) clears the highest ridge with room to
	# look down at the valley too) ---
	var camera := Camera3D.new()
	camera.name = "FlyCam"
	camera.current = true
	camera.fov = 70.0
	camera.near = 0.25
	camera.far = 43000.0
	camera.position = Vector3(0.0, 600.0, 0.0)
	camera.rotation_degrees = Vector3(-25.0, 0.0, 0.0)
	camera.set_script(load("res://scripts/fly_cam.gd"))
	add_child(camera)

	# --screenshots <dir>: auto-capture a fixed pose list, then quit
	# (screenshot_tour.gd). Needs a real renderer, not --headless.
	var user_args := OS.get_cmdline_user_args()
	var shot_index := user_args.find("--screenshots")
	if shot_index >= 0 and shot_index + 1 < user_args.size():
		camera.set_process(false)
		camera.set_process_input(false)
		var tour := Node.new()
		tour.name = "ScreenshotTour"
		tour.set_script(load("res://scripts/screenshot_tour.gd"))
		tour.camera = camera
		tour.terrain_view = terrain_view
		tour.out_dir = user_args[shot_index + 1]
		add_child(tour)
	elif "--stream-test" in user_args:
		# R2.2 R8 headless check (tools/smoke_test.ps1 -TerrainStream):
		# terrain_stream_test.gd drives the LOD focus itself, in steps.
		var stream_test := Node.new()
		stream_test.name = "TerrainStreamTest"
		stream_test.set_script(load("res://scripts/terrain_stream_test.gd"))
		stream_test.terrain_view = terrain_view
		stream_test.spawn = Vector2(spawn_x, spawn_y)
		add_child(stream_test)
	elif "--stream-fly" in user_args:
		# Windowed high-speed stream measurement (terrain_fly_test.gd).
		var fly_test := Node.new()
		fly_test.name = "TerrainFlyTest"
		fly_test.set_script(load("res://scripts/terrain_fly_test.gd"))
		fly_test.terrain_view = terrain_view
		fly_test.spawn = Vector2(spawn_x, spawn_y)
		add_child(fly_test)
	else:
		# R2.2 R8: the fly camera drives the render LOD focus (_process). Not
		# in --screenshots mode, so the tour keeps its fixed spawn chunk set
		# and its 5 poses unchanged.
		_terrain_focus_camera = camera

# Sun, moon, sky and ambient follow the time of day (sky_controller.gd, RgSky,
# data/world/sky.json); the weather/time widget sits top right.
func _add_sun_and_sky() -> void:
	_sky = load("res://scripts/sky_controller.gd").new()
	_sky.name = "Sky"
	add_child(_sky)
	_sky.setup(ProjectSettings.globalize_path("res://../data/world/sky.json").simplify_path(), _world_config_path())
	# V8: the next world load installs this sky's clock, weather and climate as the physics environment.
	if _sky.sky != null and _simulation != null and _simulation.has_method("set_weather_source"):
		_simulation.set_weather_source(_sky.sky)
		_sky.simulation = _simulation
	_weather_widget = CanvasLayer.new()
	_weather_widget.name = "WeatherWidget"
	_weather_widget.set_script(load("res://scripts/weather_widget.gd"))
	_weather_widget.sky_controller = _sky
	_weather_widget.simulation = _simulation
	add_child(_weather_widget)
	# Wet road look + rain / snow particles (weather_fx.gd, rg/wet_look.h).
	if _simulation != null and _simulation.has_method("get_wet_look"):
		_weather_fx = load("res://scripts/weather_fx.gd").new()
		_weather_fx.name = "WeatherFx"
		_weather_fx.simulation = _simulation
		add_child(_weather_fx)

func get_sky_controller() -> Node:
	return _sky

func _build_scene(user_args: PackedStringArray) -> void:
	_teleport_dialog = CanvasLayer.new()
	_teleport_dialog.set_script(load("res://scripts/address_teleport.gd"))
	_teleport_dialog.main = self
	add_child(_teleport_dialog)
	_navigation = CanvasLayer.new()
	_navigation.set_script(load("res://scripts/navigation_hud.gd"))
	_navigation.main = self
	_navigation.config_path = ProjectSettings.globalize_path("res://../data/world/route_planner.json")
	add_child(_navigation)
	set_process_priority(-2000)
	var start_world := "real_world" if "--drive" in user_args else "flat"
	var start_mode := "free_cam" if "--free-cam" in user_args else "drive"
	# Anything that names a start skips the menu (R5): the flags the scripted
	# runs and the old run.ps1 use keep starting directly.
	for direct_flag in ["--drive", "--flat", "--free-cam", "--drive-smoke", "--screenshots", "--road-shots", "--g2m-fetch-delay-ms", "--camera-test"]:
		if direct_flag in user_args:
			_direct_start = true
	var delay_index := user_args.find("--g2m-fetch-delay-ms")
	if delay_index >= 0 and delay_index + 1 < user_args.size():
		fetch_delay_ms = int(user_args[delay_index + 1])

	# --- simulation node ---
	_simulation = ClassDB.instantiate("RgSimulation")
	var dashboard_config = JSON.parse_string(FileAccess.get_file_as_string(ProjectSettings.globalize_path("res://../data/controls/dashboard.json")))
	if dashboard_config is Dictionary:
		_simulation.configure_dashboard(dashboard_config)
	_simulation.name = "Simulation"
	var rolling_config = JSON.parse_string(FileAccess.get_file_as_string(_world_config_path()))
	if rolling_config is Dictionary:
		_simulation.set_rolling_resistance_model(str(rolling_config.get("physics", {}).get("rolling_resistance_model", "quadratic")))
	_simulation.set_aero_map_selection_path(ProjectSettings.globalize_path("res://../data/aero/hypercar_map.json"))
	var fpv_err: String = _simulation.set_fpv_drone_config(ProjectSettings.globalize_path("res://../data/controls/fpv_drone.json"))
	if fpv_err != "":
		push_warning("fpv drone config: " + fpv_err + " (built-in defaults)")
	add_child(_simulation)
	if fetch_delay_ms > 0:
		_simulation.set_fetch_delay_ms(fetch_delay_ms)
	_simulation.set_player_mode(start_mode)

	_add_sun_and_sky()

	# NodePaths below are literal "../<sibling>" and are set BEFORE
	# add_child(): add_child() runs the child's _ready() synchronously while
	# Main is inside the tree, so an export set afterwards would be read too
	# late (R0's smoke-test finding).

	# --- input (devices -> values only; the bindings are the player's, R5b) ---
	_build_controls(user_args)
	_input_map = load("res://scripts/input_map.gd").new()
	_input_map.name = "InputMap"
	_input_map.controls = _controls
	var truck_visual := preload("res://scripts/npc_truck.gd").new()
	truck_visual.simulation = _simulation
	add_child(truck_visual)
	add_child(_input_map)
	var traffic := preload("res://scripts/npc_traffic.gd").new()
	_traffic_visual = traffic
	traffic.simulation = _simulation
	add_child(traffic)

	# --- placeholder/real body meshes (VEHICLE_NAME doubles as the
	# physics_sim data/models/<id>/<id>.glb id - see body_visuals.gd/
	# vehicle_visual.gd header comments) ---
	_visuals = Node3D.new()
	_visuals.name = "BodyVisuals"
	_visuals.set_script(load("res://scripts/body_visuals.gd"))
	_visuals.simulation = _simulation
	_visuals.vehicle_name = VEHICLE_NAME
	_visuals.model_absolute_path = "" # the car comes with the first world load (_apply_vehicle)
	add_child(_visuals)

	_seat_ui = preload("res://scripts/seat_adjustment.gd").new()
	_seat_ui.body_visuals = _visuals
	_seat_ui.vehicle_name = VEHICLE_NAME
	add_child(_seat_ui)

	# --- camera rigs + director (floating origin, render focus) ---
	_director = Node.new()
	_director.name = "CameraDirector"
	_director.set_script(load("res://scripts/camera_director.gd"))
	_director.simulation = _simulation
	_director.input_map = _input_map
	add_child(_director)
	var chase := Node3D.new()
	chase.name = "ChaseRig"
	chase.set_script(load("res://scripts/chase_rig.gd"))
	add_child(chase)
	_director.add_rig("chase", chase)
	var cockpit := Node3D.new()
	cockpit.name = "CockpitRig"
	cockpit.set_script(load("res://scripts/cockpit_rig.gd"))
	cockpit.body_visuals = _visuals
	add_child(cockpit)
	_director.add_rig("cockpit", cockpit)
	var bumper := Node3D.new()
	bumper.name = "BumperRig"
	bumper.set_script(load("res://scripts/bumper_rig.gd"))
	bumper.body_visuals = _visuals
	add_child(bumper)
	_director.add_rig("bumper", bumper)
	var orbit := Node3D.new()
	orbit.name = "OrbitRig"
	orbit.set_script(load("res://scripts/orbit_rig.gd"))
	add_child(orbit)
	_director.add_rig("orbit", orbit)
	var cinematic := Node3D.new()
	cinematic.name = "CinematicRig"
	cinematic.set_script(load("res://scripts/cinematic_rig.gd"))
	add_child(cinematic)
	_director.add_rig("cinematic", cinematic)
	var free := Node3D.new()
	free.name = "FreeRig"
	free.set_script(load("res://scripts/free_rig.gd"))
	add_child(free)
	_director.add_rig("free", free)
	var fpv := Node3D.new()
	fpv.name = "FpvRig"
	fpv.set_script(load("res://scripts/fpv_rig.gd"))
	add_child(fpv)
	_director.add_rig("fpv", fpv)
	var drone := Node3D.new()
	drone.name = "DroneRig"
	drone.set_script(load("res://scripts/drone_rig.gd"))
	add_child(drone)
	_director.add_rig("drone", drone)
	_walker_rig = Node3D.new()
	_walker_rig.name = "WalkerRig"
	_walker_rig.set_script(load("res://scripts/walker_rig.gd"))
	add_child(_walker_rig)
	_director.add_rig("walker", _walker_rig)
	_walker_visual = Node3D.new()
	_walker_visual.set_script(load("res://scripts/walker_visual.gd"))
	_walker_visual.simulation = _simulation
	_walker_visual.director = _director
	add_child(_walker_visual)

	_try_start_vr()

	# --- real-world terrain view (initialize_shared once a Session runs) ---
	_world_view = ClassDB.instantiate("RgTerrainView")
	_world_view.name = "Terrain"
	add_child(_world_view)
	_terrain_material = ShaderMaterial.new()
	_terrain_material.shader = load("res://shaders/terrain.gdshader")
	SurfaceMaterials.shared().apply_terrain(_terrain_material)

	# --- HUD (debug text + streaming banner) ---
	_hud = CanvasLayer.new()
	_hud.name = "Hud"
	_hud.set_script(load("res://scripts/hud.gd"))
	var readout := Label.new()
	readout.name = "Readout"
	readout.position = Vector2(16, 16)
	readout.add_theme_font_size_override("font_size", 14)
	readout.add_theme_color_override("font_outline_color", Color.BLACK)
	readout.add_theme_constant_override("outline_size", 4)
	_hud.add_child(readout)
	var banner := Label.new()
	banner.name = "Streaming"
	banner.set_anchors_preset(Control.PRESET_CENTER_TOP)
	banner.grow_horizontal = Control.GROW_DIRECTION_BOTH
	banner.position.y = 90
	banner.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	banner.add_theme_font_size_override("font_size", 34)
	banner.add_theme_color_override("font_color", Color(1.0, 0.85, 0.2))
	banner.add_theme_color_override("font_outline_color", Color.BLACK)
	banner.add_theme_constant_override("outline_size", 8)
	banner.visible = false
	_hud.add_child(banner)
	_hud.simulation_path = NodePath("../Simulation")
	_hud.input_map_path = NodePath("../InputMap")
	_hud.director_path = NodePath("../CameraDirector")
	_hud.body_visuals_path = NodePath("../BodyVisuals")
	_hud.vehicle_name = VEHICLE_NAME
	add_child(_hud)

	# --- tach/speed gauge ---
	_gauge = Control.new()
	_gauge.name = "TachGauge"
	_gauge.set_script(load("res://scripts/tach_gauge.gd"))
	_gauge.simulation_path = NodePath("../Simulation")
	_gauge.input_map_path = NodePath("../InputMap")
	_gauge.vehicle_name = VEHICLE_NAME
	add_child(_gauge)

	# --- loading overlay ---
	_overlay = CanvasLayer.new()
	_overlay.name = "LoadingOverlay"
	_overlay.set_script(load("res://scripts/loading_overlay.gd"))
	add_child(_overlay)
	if _xr_menu != null: _xr_menu.attach(_overlay)

	_build_shell(user_args)
	_apply_mode_state()
	if _direct_start:
		_apply_transition(_shell.direct_start(_direct_start_world(start_world, user_args)))
	else:
		_set_world_ui_visible(false)
		_show_screen("boot")

	# --- scripted runs ---
	if "--drive-smoke" in user_args:
		var smoke := Node.new()
		smoke.name = "DriveSmoke"
		smoke.set_script(load("res://scripts/drive_smoke.gd"))
		smoke.main = self
		add_child(smoke)
	var shot_index := user_args.find("--screenshots")
	if shot_index >= 0 and shot_index + 1 < user_args.size():
		var tour := Node.new()
		tour.name = "DriveTour"
		tour.set_script(load("res://scripts/drive_tour.gd"))
		tour.main = self
		tour.out_dir = user_args[shot_index + 1]
		add_child(tour)
	if "--shell-test" in user_args:
		var shell_test := Node.new()
		shell_test.name = "ShellFlowTest"
		shell_test.set_script(load("res://scripts/shell_flow_test.gd"))
		shell_test.main = self
		add_child(shell_test)
	if "--controls-test" in user_args or "--controls-verify" in user_args:
		var controls_test := Node.new()
		controls_test.name = "ControlsTest"
		controls_test.set_script(load("res://scripts/controls_test.gd"))
		controls_test.main = self
		controls_test.verify_phase = "--controls-verify" in user_args
		add_child(controls_test)
	var controls_shots_index := user_args.find("--controls-shots")
	if controls_shots_index >= 0 and controls_shots_index + 1 < user_args.size():
		var controls_shots := Node.new()
		controls_shots.name = "ControlsShots"
		controls_shots.set_script(load("res://scripts/controls_shots.gd"))
		controls_shots.main = self
		controls_shots.out_dir = user_args[controls_shots_index + 1]
		add_child(controls_shots)
	var garage_shots_index := user_args.find("--garage-shots")
	if garage_shots_index >= 0 and garage_shots_index + 1 < user_args.size():
		var garage_shots := Node.new()
		garage_shots.name = "GarageShots"
		garage_shots.set_script(load("res://scripts/garage_shots.gd"))
		garage_shots.main = self
		garage_shots.out_dir = user_args[garage_shots_index + 1]
		var vehicle_arg := user_args.find("--garage-shots-vehicle")
		if vehicle_arg >= 0 and vehicle_arg + 1 < user_args.size():
			garage_shots.vehicle = user_args[vehicle_arg + 1]
		var prefix_arg := user_args.find("--garage-shots-prefix")
		if prefix_arg >= 0 and prefix_arg + 1 < user_args.size():
			garage_shots.prefix = user_args[prefix_arg + 1]
		add_child(garage_shots)
	if "--car-browser-test" in user_args:
		var browser_test := Node.new()
		browser_test.name = "CarBrowserTest"
		browser_test.set_script(load("res://scripts/car_browser_test.gd"))
		browser_test.main = self
		browser_test.big = "--car-browser-big" in user_args
		add_child(browser_test)
	var browser_shots_index := user_args.find("--car-browser-shots")
	if browser_shots_index >= 0 and browser_shots_index + 1 < user_args.size():
		var browser_shots := Node.new()
		browser_shots.name = "CarBrowserShots"
		browser_shots.set_script(load("res://scripts/car_browser_shots.gd"))
		browser_shots.main = self
		browser_shots.out_dir = user_args[browser_shots_index + 1]
		browser_shots.big = "--car-browser-big" in user_args
		add_child(browser_shots)
	if "--garage-test" in user_args:
		var garage_test := Node.new()
		garage_test.name = "GarageTest"
		garage_test.set_script(load("res://scripts/garage_test.gd"))
		garage_test.main = self
		add_child(garage_test)
	if "--camera-test" in user_args:
		var camera_test := Node.new()
		camera_test.name = "CameraSwitchTest"
		camera_test.set_script(load("res://scripts/camera_switch_test.gd"))
		camera_test.main = self
		add_child(camera_test)
	var road_shots_index := user_args.find("--road-shots")
	if road_shots_index >= 0 and road_shots_index + 1 < user_args.size():
		var road_shots := Node.new()
		road_shots.name = "RoadShots"
		road_shots.set_script(load("res://scripts/road_shots.gd"))
		road_shots.main = self
		road_shots.out_dir = user_args[road_shots_index + 1]
		add_child(road_shots)

# --- accessors for the scripted runs ---
func get_simulation() -> Node:
	return _simulation

func get_director() -> Node:
	return _director

func get_overlay() -> Node:
	return _overlay

func get_world_view() -> Node:
	return _world_view

func get_shell() -> Node:
	return _shell

func get_shell_ui() -> CanvasLayer:
	return _shell_ui

func shell_screen() -> String:
	return _screen

func drive_view() -> String:
	return _drive_view

func get_body_visuals() -> Node:
	return _visuals

func chassis_session_position() -> Vector3:
	return _world_view.godot_to_session(_simulation.get_body_transform("chassis").origin)

# --- world load / switch ---
func switch_world() -> void:
	var other: String = str(_simulation.get_mode_state().get("other_world", "flat"))
	print("RG_WORLD switch %s -> %s (was %s)" % [world_kind, other, world_state])
	_world_request = {}
	_load_world(other)

var _vehicle_audio: Node3D

# Frees everything that belongs to the running world and reads its Session (road
# and building streams, the vehicle audio, the terrain view): a world switch and
# the way back to the main menu both start with this.
const AUDIO_SETTLE_S := 0.1
var _quitting := false

func _release_world_nodes() -> void:
	if _roads != null:
		_roads.shutdown()
		_roads.queue_free()
		_roads = null
	if _buildings != null:
		_buildings.shutdown()
		_buildings.queue_free()
		_buildings = null
	_steering.reset()
	if _vehicle_audio != null:
		_vehicle_audio.shutdown()
		_vehicle_audio.queue_free()
		_vehicle_audio = null
	if _fpv_audio != null:
		_fpv_audio.queue_free()
		_fpv_audio = null
	# The view shares the old Session's WorldTerrain: release it first.
	_world_view.release()
	_director.set_terrain_view(null)

func _load_world(kind: String, keep_spawn_override: bool = false) -> void:
	_release_world_nodes()
	world_kind = kind
	world_state = "loading"
	ready_sim_time = -1.0
	_spawn_reported = false
	_load_started_ms = Time.get_ticks_msec()
	_hud.reset_tick_window()
	_visuals.show_ground = kind == "flat"
	var sel: Dictionary = _prepare_vehicle()
	if not bool(sel.get("ok", false)):
		_load_failed("vehicle: %s" % str(sel.get("error", "?")))
		return
	var vehicle_json: String = str(sel["vehicle_path"])
	_steering.configure_vehicle(vehicle_json)
	_input_map.configure_vehicle(vehicle_json, bool(sel["assist_auto_shift"]))
	var surface_table_json: String = ProjectSettings.globalize_path("res://../data/surfaces/surfaces.json")
	if kind == "flat":
		if not keep_spawn_override:
			_simulation.clear_spawn_override()
		# Non-blocking like the real world: the Session is built on the init
		# worker (a turbo car's first engine builds a full-load envelope);
		# _poll_loading() starts it once get_init_status() says "ready".
		if not _simulation.initialize(vehicle_json, surface_table_json):
			_load_failed(str(_simulation.get_last_error()))
			return
		_overlay.show_loading(WORLD_LABEL[kind], {}, 0.0, _loading_footer(), str(_world_request.get("label", "")))
		print("RG_WORLD loading world=flat")
	else:
		# The spawn the player picked belongs to a load started from the shell
		# (set_spawn_override in _begin_world_load); F8 starts at the world's own.
		_simulation.set_store_dir_override(str(_shell.get_setting("map_data.store_dir")))
		_simulation.initialize_terrain(_world_config_path(), vehicle_json, surface_table_json)
		_overlay.show_loading(WORLD_LABEL[kind], {}, 0.0, _loading_footer(), str(_world_request.get("label", "")))
		print("RG_WORLD loading world=real_world fetch_delay_ms=%d" % fetch_delay_ms)

func _loading_footer() -> String:
	return "Esc cancels and returns to the main menu   F8 switches to the other world"

func _load_failed(message: String) -> void:
	world_state = "failed"
	var label: String = WORLD_LABEL.get(world_kind, world_kind)
	if _screen == "loading" and not _direct_start:
		# Started from the menu: the flow returns to the main menu, which shows
		# the message. Expected (a missing map store), so no script error.
		print("RG_WORLD load failed: %s" % message)
		_apply_transition(_shell.load_failed(message))
		return
	_overlay.show_error(label, message, "Esc returns to the main menu   F8 switches to the other world")
	push_error("%s did not load: %s" % [label, message])

# The world just became drivable (flat: at once; real world: its first tick).
# The flow hears about it one frame later (_post_load_ready) so a transition that
# started the load has finished applying its own actions first.
func _world_running() -> void:
	_simulation.set_road_ahead_wanted(_drive_view == "cinematic")
	_apply_audio_settings()
	call_deferred("_post_load_ready")

func _post_load_ready() -> void:
	if _screen != "loading" or world_state != "running":
		return
	_apply_transition(_shell.load_ready())
	if bool(_world_request.get("open_address_search", false)) and world_kind == "real_world":
		_world_request["open_address_search"] = false
		_teleport_dialog.open_dialog()

# Controls a fresh Session needs before its first tick: every channel starts
# at 0 in rg::Session, so ignition must be on BEFORE start() or the car
# spawns with the engine off. auto_clutch/auto_blip are always on (no pedal
# work needed to pull away); ignition/auto_shift follow input_map.gd's
# toggles (ignition starts on; manual cars start with auto-shift off).
func _apply_start_controls() -> void:
	_simulation.set_control("ignition", 1.0 if _input_map.get_ignition() else 0.0)
	# the vehicle file's own assist defaults (after the saved setup), read by RgGarage.prepare_drive
	_simulation.set_control("assist.auto_clutch", 1.0 if bool(_drive_selection.get("assist_auto_clutch", true)) else 0.0)
	_simulation.set_control("assist.auto_blip", 1.0 if bool(_drive_selection.get("assist_auto_blip", true)) else 0.0)
	_simulation.set_control("assist.auto_shift", 1.0 if _input_map.get_auto_shift() else 0.0)

func _poll_loading() -> void:
	var st: Dictionary = _simulation.get_init_status()
	st["engine_title"] = str(_drive_selection.get("engine_title", ""))
	st["swap_lines"] = _drive_selection.get("swap_lines", PackedStringArray())
	var elapsed := (Time.get_ticks_msec() - _load_started_ms) / 1000.0
	match str(st.get("state", "")):
		"loading":
			_overlay.show_loading(WORLD_LABEL[world_kind], st, elapsed, _loading_footer(), str(_world_request.get("label", "")))
		"ready":
			_overlay.show_loading(WORLD_LABEL[world_kind], st, elapsed, _loading_footer(), str(_world_request.get("label", "")))
			_apply_start_controls()
			_simulation.start()
			if world_kind == "flat":
				_finish_flat_load()
			else:
				world_state = "starting"
		"error":
			_load_failed(str(st.get("message", "")))

# The flat world is drivable as soon as its Session is started.
func _finish_flat_load() -> void:
	world_state = "running"
	ready_sim_time = 0.0
	_overlay.hide_overlay()
	_visuals.on_session_ready()
	_bench_synthetic_terrain()
	print("RG_WORLD ready world=flat")
	_world_running()

# The --render-thread command-line choice (safe = rendering on the main thread, the default; separate = own thread).
func _render_thread_mode() -> String:
	var args := OS.get_cmdline_args()
	var i := args.find("--render-thread")
	return args[i + 1] if i >= 0 and i + 1 < args.size() else "safe"

# Bench only (perf, 2026-10-15): RG_BENCH_SYNTHETIC_TERRAIN=<metres> streams the synthetic rolling-hills terrain
# (RgTerrainView.load_synthetic_preview, the game's LOD defaults, 20 km) around the flat-world car, lowered by that many
# metres so it stays under the ground box. Gives a no-map-store scene with the real world's terrain draw calls and
# primitives plus the car and HUD, for RG_FRAME_STATS / RG_PERF_AB runs on any machine. No physics behind it.
func _bench_synthetic_terrain() -> void:
	if OS.get_environment("RG_BENCH_SURFACES") != "":
		get_tree().create_timer(3.0).timeout.connect(_print_surface_census)
		get_tree().create_timer(4.0).timeout.connect(_print_viewport_census)
	var drop := OS.get_environment("RG_BENCH_SYNTHETIC_TERRAIN")
	if drop.is_empty():
		return
	var pose: Dictionary = _simulation.get_chassis_session_pose()
	_world_view.set_material(_terrain_material.get_rid())
	_world_view.load_synthetic_preview(float(pose.get("x", 0.0)), float(pose.get("y", 0.0)), 20000.0)
	_world_view.set_render_origin(Vector3(0.0, 0.0, float(drop)))
	_director.set_terrain_view(_world_view)
	print("RG_BENCH synthetic terrain chunks=%d vertices=%d drop_m=%s" % [_world_view.get_chunk_count(), _world_view.get_total_vertex_count(), drop])

# RG_BENCH_SURFACES=1 (bench only): 3 s after a flat world is ready, prints one RG_SURFACES line per scene node that
# holds mesh surfaces (MeshInstance3D only; RenderingServer-direct terrain is counted by RgTerrainView): visible mesh
# instances, surfaces (= draw calls in the colour pass), surfaces whose instance casts sun shadows (drawn again per
# cascade they touch) and triangles. Names the draw-call budget per feature.
# RG_BENCH_SURFACES: draw calls per viewport (3D visible / shadow / canvas), and the same with the root viewport's 3D
# switched off (RG_PERF_AB "3d"), so the draw calls that are not the 3D world are attributed.
func _print_viewport_census() -> void:
	for pass_name in ["3d_on", "3d_off"]:
		get_viewport().disable_3d = pass_name == "3d_off"
		for i in 3:
			await get_tree().process_frame
		var ports: Array = [get_viewport()]
		ports.append_array(get_tree().root.find_children("*", "SubViewport", true, false))
		for vp in ports:
			var v := vp as Viewport
			var mode := -1 if v == get_viewport() else int((v as SubViewport).render_target_update_mode)
			print("RG_VIEWPORTS %s %s size=%s update=%d disable_3d=%s visible_dc=%d shadow_dc=%d canvas_dc=%d canvas_prims=%d" % [
				pass_name, str(v.get_path()), str(v.get_visible_rect().size), mode, str(v.disable_3d),
				v.get_render_info(Viewport.RENDER_INFO_TYPE_VISIBLE, Viewport.RENDER_INFO_DRAW_CALLS_IN_FRAME),
				v.get_render_info(Viewport.RENDER_INFO_TYPE_SHADOW, Viewport.RENDER_INFO_DRAW_CALLS_IN_FRAME),
				v.get_render_info(Viewport.RENDER_INFO_TYPE_CANVAS, Viewport.RENDER_INFO_DRAW_CALLS_IN_FRAME),
				v.get_render_info(Viewport.RENDER_INFO_TYPE_CANVAS, Viewport.RENDER_INFO_PRIMITIVES_IN_FRAME)])
		print("RG_VIEWPORTS %s total_dc=%d" % [pass_name, int(Performance.get_monitor(Performance.RENDER_TOTAL_DRAW_CALLS_IN_FRAME))])
	get_viewport().disable_3d = false
	# Canvas draw calls per CanvasLayer (hide it for a few frames, read the difference) and every visible canvas item that
	# covers at least a quarter of the window (full-screen fill cost at 4K).
	var root_vp := get_viewport()
	var base := root_vp.get_render_info(Viewport.RENDER_INFO_TYPE_CANVAS, Viewport.RENDER_INFO_DRAW_CALLS_IN_FRAME)
	for layer in get_tree().root.find_children("*", "CanvasLayer", true, false):
		var cl := layer as CanvasLayer
		if not cl.visible or cl.get_viewport() != root_vp:
			continue
		cl.visible = false
		for i in 3:
			await get_tree().process_frame
		var without := root_vp.get_render_info(Viewport.RENDER_INFO_TYPE_CANVAS, Viewport.RENDER_INFO_DRAW_CALLS_IN_FRAME)
		cl.visible = true
		for i in 3:
			await get_tree().process_frame
		print("RG_CANVAS_LAYER %s layer=%d canvas_dc=%d" % [str(cl.get_path()), cl.layer, base - without])
	var area := root_vp.get_visible_rect().get_area()
	for item in get_tree().root.find_children("*", "Control", true, false):
		var c := item as Control
		if c.is_visible_in_tree() and c.get_viewport() == root_vp and c.get_global_rect().get_area() >= area * 0.25:
			print("RG_CANVAS_BIG %s class=%s rect=%s material=%s" % [str(c.get_path()), c.get_class(), str(c.get_global_rect()), str(c.material)])

func _print_surface_census() -> void:
	var rows := {}
	var stack: Array = [[self, ""]]
	while not stack.is_empty():
		var item: Array = stack.pop_back()
		var node: Node = item[0]
		var group: String = item[1]
		if node is VisualInstance3D and not (node as VisualInstance3D).is_visible_in_tree():
			continue
		if node is CanvasItem and not (node as CanvasItem).is_visible_in_tree():
			continue
		if node is MeshInstance3D and (node as MeshInstance3D).mesh != null:
			var mi := node as MeshInstance3D
			var r: Array = rows.get(group, [0, 0, 0, 0])
			var n := mi.mesh.get_surface_count()
			r[0] += 1
			r[1] += n
			if mi.cast_shadow != GeometryInstance3D.SHADOW_CASTING_SETTING_OFF:
				r[2] += n
			for i in n:
				var arrays := mi.mesh.surface_get_arrays(i) if mi.mesh is ArrayMesh else []
				if arrays.size() > Mesh.ARRAY_INDEX and arrays[Mesh.ARRAY_INDEX] != null and (arrays[Mesh.ARRAY_INDEX] as PackedInt32Array).size() > 0:
					r[3] += (arrays[Mesh.ARRAY_INDEX] as PackedInt32Array).size() / 3
				elif arrays.size() > Mesh.ARRAY_VERTEX and arrays[Mesh.ARRAY_VERTEX] != null:
					r[3] += (arrays[Mesh.ARRAY_VERTEX] as PackedVector3Array).size() / 3
			rows[group] = r
		for c in node.get_children():
			var g: String = group
			if node == self:
				g = str(c.name)
			elif group.count("/") < 1 and node.get_parent() == self:
				g = group + "/" + str(c.name)
			stack.append([c, g])
	for g in rows:
		var r: Array = rows[g]
		print("RG_SURFACES %s instances=%d surfaces=%d shadow_surfaces=%d triangles=%d" % [g, r[0], r[1], r[2], r[3]])

# First tick after start(): the snapshot now holds the spawned chassis, so the
# terrain view can be built around it.
func _attach_world_view() -> void:
	if int(_simulation.get_step_count()) <= 0:
		return
	if not _world_view.initialize_shared(_simulation):
		_load_failed("RgTerrainView.initialize_shared: %s" % _world_view.get_last_error())
		return
	_world_view.set_material(_terrain_material.get_rid())
	_world_view.set_render_origin(_simulation.get_render_origin_session())
	SurfaceMaterials.set_render_origin(_simulation.get_render_origin_session())
	var chassis := chassis_session_position()
	if not _world_view.load_preview(chassis.x, chassis.y):
		_load_failed("RgTerrainView.load_preview: %s" % _world_view.get_last_error())
		return
	_director.set_terrain_view(_world_view)
	var world_data: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(_world_config_path()))
	var road_settings: Dictionary = world_data.get("road_visuals", {})
	if bool(road_settings.get("enabled", true)):
		_roads = preload("res://scripts/road_stream.gd").new()
		_roads.view = _world_view
		_roads.simulation = _simulation
		_roads.director = _director
		_roads.settings = road_settings
		_roads.utm_origin = Vector2(world_data.session_origin_utm.e0, world_data.session_origin_utm.n0)
		add_child(_roads)
		_world_view.tree_exiting.connect(_roads.shutdown, CONNECT_ONE_SHOT)
		_simulation.tree_exiting.connect(_roads.shutdown, CONNECT_ONE_SHOT)
	var building_settings: Dictionary = world_data.get("buildings", {})
	if bool(building_settings.get("enabled", true)):
		_buildings = preload("res://scripts/building_stream.gd").new()
		_buildings.view = _world_view
		_buildings.simulation = _simulation
		_buildings.director = _director
		_buildings.settings = building_settings
		_buildings.utm_origin = Vector2(world_data.session_origin_utm.e0, world_data.session_origin_utm.n0)
		add_child(_buildings)
		_world_view.tree_exiting.connect(_buildings.shutdown, CONNECT_ONE_SHOT)
		_simulation.tree_exiting.connect(_buildings.shutdown, CONNECT_ONE_SHOT)
	world_state = "running"
	ready_sim_time = float(_simulation.get_sim_time())
	_overlay.hide_overlay()
	_visuals.on_session_ready()
	_world_running()
	var ss: Dictionary = _simulation.get_streaming_status()
	DisplayServer.window_set_title("racing_game | " + _simulation.get_build_info())
	print("RG_DRIVE ready world=real_world load_s=%.2f startup_ms=%.0f resident_l0=%d prime_ticks=%d chassis_session=(%.2f, %.2f, %.2f) chunks=%d fetch_delay_ms=%d mode=%s build=%s" % [
		(Time.get_ticks_msec() - _load_started_ms) / 1000.0, float(ss.get("startup_ms", 0.0)),
		int(ss.get("resident_l0", 0)), int(ss.get("prime_ticks", 0)), chassis.x, chassis.y, chassis.z,
		int(_world_view.get_chunk_count()), fetch_delay_ms, _simulation.get_player_mode(), _simulation.get_build_info()])

# Once per second of wall time in the real world, plus one spawn check 2 s
# (sim time) after the world became drivable - its numbers are how a wrong
# spawn (airborne, sunk, sliding) shows up in a log.
func _report() -> void:
	if world_kind != "real_world":
		return
	var sim_t: float = _simulation.get_sim_time()
	if not _spawn_reported and sim_t - ready_sim_time >= 2.0:
		_spawn_reported = true
		var loads := PackedStringArray()
		var load_sum := 0.0
		for i in range(_simulation.get_vehicle_wheel_count(VEHICLE_NAME)):
			var l: float = _simulation.get_wheel_load_n(VEHICLE_NAME, i)
			load_sum += l
			loads.append("%s=%.0f" % [_simulation.get_wheel_name(VEHICLE_NAME, i), l])
		var pt: Dictionary = _simulation.get_vehicle_powertrain(VEHICLE_NAME)
		var p := chassis_session_position()
		print("RG_DRIVE spawn_check t=%.2f session=(%.2f, %.2f, %.2f) speed_mps=%.3f wheel_loads_n=[%s] load_sum_n=%.0f engine=%s gear=%d surface0=%s" % [
			sim_t - ready_sim_time, p.x, p.y, p.z, _simulation.get_body_speed_mps("chassis"), ", ".join(loads), load_sum,
			pt.get("engine_state", "?"), int(pt.get("gear", 0)), _simulation.get_wheel_surface_name(VEHICLE_NAME, 0)])
	var now := Time.get_ticks_msec()
	if now - _last_report_ms < 1000:
		return
	_last_report_ms = now
	print(status_line("RG_DRIVE t=%.1f" % (sim_t - ready_sim_time)))
	_save_drive_location()
	_report_tick_spikes()
	if _perf:
		_report_streams()

# Render-stream health at speed (owner complaint 2026-10-13: at 400-580 km/h the
# terrain does not resolve in time, the road is not visible far ahead). One line
# each per second:
#  RG_TERRAIN_STREAM  l0_ahead_m / l1_ahead_m = how far along the direction of
#     travel the UPLOADED ground is finest-level-covered (L0 = 1 m grid, L1 =
#     2 m); pending_adds/removals of the diff in progress; adds_s = chunks
#     uploaded in the last second; upload_ms avg/max per frame; chunk_lat_p95_ms
#     = selection -> upload of a chunk; budget_ms = the adaptive upload budget;
#     lead_m = the render-focus lead; ground_above_m = the highest visible
#     surface above the physics surface (carved L0 height tiles) at the car and
#     5/20/60 m ahead (max_ground_above_m since the start; far_ground_above_m =
#     the same 150/300 m ahead); fps.
#  RG_ROAD_STREAM  ribbon tiles pending, the oldest wait, ribbon_ahead_m.
func _report_streams() -> void:
	if _world_view == null or _simulation == null:
		return
	var pose: Dictionary = _simulation.get_chassis_session_pose()
	if pose.is_empty():
		return
	var px: float = pose.get("x", 0.0)
	var py: float = pose.get("y", 0.0)
	var vx: float = pose.get("vx", 0.0)
	var vy: float = pose.get("vy", 0.0)
	var speed := Vector2(vx, vy).length()
	var dir := Vector2(vx, vy) / speed if speed >= 2.0 else Vector2.from_angle(deg_to_rad(float(pose.get("yaw_deg", 0.0))))
	var cov: Dictionary = _world_view.get_lod_coverage(px, py, dir.x, dir.y)
	var st: Dictionary = _world_view.get_stream_stats()
	var rs: Dictionary = _world_view.take_report_stats()
	var gc: Dictionary = _world_view.get_ground_check(px, py, dir.x, dir.y)
	var bed: Dictionary = _world_view.get_bed_check(px, py, dir.x, dir.y, 600.0)
	var lead_m: float = _director.render_focus_lead_m() if _director != null else 0.0
	print("RG_TERRAIN_STREAM speed_mps=%.1f l0_ahead_m=%.0f l1_ahead_m=%.0f l2_ahead_m=%.0f pending_adds=%d pending_removals=%d adds_s=%d upload_ms_avg=%.2f upload_ms_max=%.2f chunk_lat_p95_ms=%.0f budget_ms=%.2f lead_m=%.0f ground_above_m=%.3f max_ground_above_m=%.3f ground_level=%d far_ground_above_m=%.3f fps=%d resident=%d fallback_chunks=%d fallback_tiles=%d bed_above_max_m=%.3f bed_above_p95_m=%.3f bed_samples=%d bed_worst_level=%d bed_worst_carve=%d" % [
		speed, float(cov.get("l0_ahead_m", 0.0)), float(cov.get("l1_ahead_m", 0.0)), float(cov.get("l2_ahead_m", 0.0)),
		int(st.get("pending_adds", 0)), int(st.get("pending_removals", 0)), int(rs.get("chunks_uploaded", 0)),
		float(rs.get("upload_ms_avg", 0.0)), float(rs.get("upload_ms_max", 0.0)), float(rs.get("chunk_latency_p95_ms", 0.0)),
		float(rs.get("budget_ms_avg", 0.0)), lead_m, float(gc.get("visible_above_m", -1000.0)),
		float(gc.get("max_since_reset_m", -1000.0)), int(gc.get("visible_level", -1)), float(gc.get("far_above_m", -1000.0)),
		int(Engine.get_frames_per_second()),
		int(st.get("resident_chunks", 0)), int(st.get("fallback_chunks", 0)), int(st.get("fallback_tiles", 0)),
		float(bed.get("above_max_m", -1000.0)), float(bed.get("above_p95_m", -1000.0)), int(bed.get("samples", 0)),
		int(bed.get("worst_level", -1)), int(bed.get("worst_carve", 0))])
	if _roads != null and _roads.has_method("get_stats"):
		var rd: Dictionary = _roads.get_stats(Vector2(px, py), dir)
		print("RG_ROAD_STREAM pending=%d oldest_age_s=%.1f ribbon_ahead_m=%.0f tiles=%d built_total=%d" % [
			int(rd.get("pending", 0)), float(rd.get("oldest_age_s", 0.0)), float(rd.get("ribbon_ahead_m", 0.0)),
			int(rd.get("tiles", 0)), int(rd.get("built_total", 0))])

# Debug aid (gray-patch diagnosis 2026-10-13): hide the terrain chunks and/or the road ribbons to see
# which one covers what. Tint chunks by level / carve state with env RG_TERRAIN_TINT=level|carve|both.
func debug_hide(terrain: bool, ribbons: bool) -> void:
	if _world_view != null:
		_world_view.set_terrain_visible(not terrain)
	if _roads != null:
		_roads.visible = not ribbons

# Tick-spike diagnostics (owner drive 2026-09-27, ~170 ms sim stalls): the
# sim thread's slow or late tick attempts, per phase (rg::Session's
# drain_tick_spikes), at most MAX_SPIKE_LINES per report so a long stall
# cannot flood the log; the rest is summarised as a count.
const MAX_SPIKE_LINES := 40
func _report_tick_spikes() -> void:
	var lines: PackedStringArray = _simulation.drain_tick_spikes()
	for i in range(mini(lines.size(), MAX_SPIKE_LINES)):
		print("RG_TICK_SPIKE " + lines[i])
	if lines.size() > MAX_SPIKE_LINES:
		print("RG_TICK_SPIKE suppressed=%d" % (lines.size() - MAX_SPIKE_LINES))

# The shared "numbers" part of the RG_DRIVE lines (drive_smoke.gd's final
# line too): "falls=%d misses=%d" stays one contiguous token pair, the smoke
# greps it.
func _notification(what: int) -> void:
	if what == NOTIFICATION_WM_CLOSE_REQUEST:
		if _quitting:
			return
		_quitting = true
		if world_kind == "real_world" and world_state == "running":
			_save_drive_location()
		if _shell != null:
			_shell.save_settings()
		if _controls != null:
			_controls.save()
		await _shutdown_audio_and_quit()

var _saved_origin_path := ""
var _saved_origin: Dictionary = {}

func _save_drive_location() -> void:
	var p := chassis_session_position()
	var config_path := _world_config_path()
	if config_path != _saved_origin_path: # read + parse the world config once, not on the main thread every second
		_saved_origin_path = config_path
		var config = JSON.parse_string(FileAccess.get_file_as_string(config_path))
		_saved_origin = config.get("session_origin_utm", {}) if config is Dictionary else {}
	var origin: Dictionary = _saved_origin
	var pose: Dictionary = _simulation.get_chassis_session_pose()
	var file := FileAccess.open(_user_dir().path_join("last_drive.json"), FileAccess.WRITE)
	if file:
		file.store_string(JSON.stringify({"saved_at": Time.get_datetime_string_from_system(),
			"session_m": [p.x, p.y, p.z], "yaw_deg": float(pose.get("yaw_deg", 0.0)), "world": world_kind,
			"utm_zone": origin.get("zone"), "utm_m": [float(origin.get("e0", 0))+p.x, float(origin.get("n0", 0))+p.y, p.z],
			"build": _simulation.get_build_info(), "status": status_line("last_drive")}, "	"))

func status_line(prefix: String) -> String:
	var ss: Dictionary = _simulation.get_streaming_status()
	var pt: Dictionary = _simulation.get_vehicle_powertrain(VEHICLE_NAME)
	var surface := ""
	if _simulation.get_vehicle_names().has(VEHICLE_NAME):
		surface = _simulation.get_wheel_surface_name(VEHICLE_NAME, 0)
	var ls: Dictionary = _simulation.get_loop_stats()
	var p := chassis_session_position()
	prefix += " session=(%.3f,%.3f,%.3f)" % [p.x, p.y, p.z]
	return "%s ticks=%d speed_kmh=%.1f gear=%d rpm=%.0f frozen=%s missing=%d inflight=%d freezes=%d frozen_ticks=%d falls=%d misses=%d starved=%d relocations=%d relocate_failures=%d mode=%s surface=%s dropped_ticks=%d step_max_ms=%.1f prefetch_installed=%d prefetch_late_sync=%d prefetch_late_wait=%d prefetch_wait_max_ms=%.3f prefetch_stale=%d" % [
		prefix, int(_simulation.get_step_count()), _simulation.get_body_speed_mps("chassis") * 3.6,
		int(pt.get("gear", 0)), float(pt.get("rpm", 0.0)), "yes" if bool(ss.get("frozen", false)) else "no",
		int(ss.get("missing_required", 0)), int(ss.get("inflight", 0)), int(ss.get("freeze_count", 0)),
		int(ss.get("frozen_attempts", 0)), int(ss.get("falls", 0)), int(ss.get("fill_misses", 0)),
		int(ss.get("starved_tiles", 0)), int(ss.get("relocations", 0)), int(ss.get("relocate_failures", 0)),
		_simulation.get_player_mode(), surface if surface != "" else "?",
		int(ls.get("dropped_ticks", 0)), float(ls.get("step_max_ms", 0.0)),
		int(ss.get("prefetch_installed", 0)), int(ss.get("prefetch_late_sync", 0)),
		int(ss.get("prefetch_late_wait", 0)), float(ss.get("prefetch_late_wait_ns_max", 0)) / 1000000.0,
		int(ss.get("prefetch_stale_inputs", 0))] + " npc=%d traffic_loading=%s" % [_simulation.get_traffic_state().get("actors", []).size(), str(_simulation.get_traffic_state().get("loading", false))] + " lc=%d clutch=%.3f tc=%.3f slip=%.3f throttle=%.3f brake=%.3f" % [int(pt.get("launch_phase",0)), float(pt.get("clutch_engagement",0.0)), float(pt.get("traction_torque_fraction",1.0)), float(pt.get("traction_positive_slip",0.0)), _simulation.get_control("throttle"), _simulation.get_control("brake")]

# rg_core decides the active rig and which input groups are live.
func _apply_mode_state() -> Dictionary:
	var ms: Dictionary = _simulation.get_mode_state()
	var rig_name := str(ms.get("camera_rig", "chase"))
	if rig_name == "chase":
		rig_name = _drive_view # chase / bumper / cockpit / orbit / cinematic
	if _vr_active:
		# VR has its own drone rig (xr_drone: trails the target, the player's view is never rotated for them).
		# (No XR walker rig yet: on foot the player keeps the tracked free flight.)
		if rig_name == "drone":
			rig_name = "xr_drone"
		else:
			rig_name = "xr_free" if rig_name == "free" or rig_name == "walker" or rig_name == "fpv" else "xr_cockpit"
	var drone_id := int(ms.get("drone_target_id", -1))
	if drone_id != _drone_target_seen:
		_drone_target_seen = drone_id
		print("RG_WORLD drone target -> %s (id %d)" % [ms.get("drone_target_label", "?"), drone_id])
	_director.set_active(rig_name)
	_notify_mode_events(ms)
	_director.camera_input_live = bool(ms.get("camera_inputs_live", true)) and _screen == "drive" and not _panel_open()
	return ms

# One of the F6 / F7 / F10 panels is open (they own the keyboard and the mouse).
func _panel_open() -> bool:
	# Called several times a frame: first-node lookups, no group arrays built (perf, 2026-10-15).
	var tree := get_tree()
	return tree.get_first_node_in_group("seat_adjustment_open") != null or tree.get_first_node_in_group("traffic_settings_open") != null or tree.get_first_node_in_group("address_teleport_open") != null

# Messages for what the mode framework refused, and a log line per mode change
# (a get-in changes the mode without any key press of the cycle).
func _notify_mode_events(ms: Dictionary) -> void:
	var mode_now := str(ms.get("mode", "drive"))
	if mode_now != _mode_seen:
		_mode_seen = mode_now
		print("RG_WORLD mode now %s" % mode_now)
	var refusals := int(ms.get("get_out_refusals", 0))
	if refusals > _refusals_seen and _hud != null:
		_hud.show_message("Stop the car first: getting out needs a speed below 2 m/s", 3.0)
	_refusals_seen = refusals
	var enter_refused := int(ms.get("walker_enter_refused", 0))
	if enter_refused > _enter_refused_seen and _hud != null:
		_hud.show_message("Too far from the car: walk up to its door (within 1.5 m)", 3.0)
	_enter_refused_seen = enter_refused

# Walking input (on foot): the camera group's move axes, run, plus the look
# heading the walker rig owns; jump and interact are edge presses handled in
# _process. The walker's physics lives in rg_core.
func _forward_walking(live: bool) -> void:
	if not bool(_simulation.is_running()):
		return
	var blocked: bool = _panel_open()
	if not live or blocked:
		_simulation.set_walker_input(0.0, 0.0, _walker_rig.get_look_forward(), false)
		return
	if not scripted_walk.is_empty():
		_simulation.set_walker_input(float(scripted_walk.get("move_right", 0.0)), float(scripted_walk.get("move_forward", 0.0)), scripted_walk.get("look", _walker_rig.get_look_forward()), bool(scripted_walk.get("run", false)))
		return
	var m: Vector2 = _input_map.get_walk_move()
	_simulation.set_walker_input(m.x, m.y, _walker_rig.get_look_forward(), _input_map.get_walk_run())

# FPV drone input (fpv_drone mode): the camera group's move axes (right, up, forward) and look axes (yaw, pitch). The
# flight model lives in rg_core (rg::FpvDrone); the rate axes are +left / +up there, the look input is +right / +up.
func _forward_flying(live: bool) -> void:
	if not bool(_simulation.is_running()):
		return
	if not live or _panel_open():
		_simulation.set_fpv_input(0.0, 0.0, 0.0, 0.0, 0.0)
		return
	var m: Vector3 = _input_map.get_camera_move()
	var look: Vector2 = _input_map.get_camera_look_rate()
	_simulation.set_fpv_input(m.x, m.y, m.z, -look.x, look.y)

func _forward_driving(live: bool, delta: float) -> void:
	# Edge counts are consumed every frame, so presses made while the car is
	# unattended never arrive later as a burst of shifts.
	var cabin_cockpit: Node3D = _visuals.get_cockpit() if _visuals != null and _visuals.has_method("get_cockpit") else null
	if cabin_cockpit != null:
		cabin_cockpit.set("driver_actions", _input_map.driver_actions) # cockpit switches submit through the shared instance
	var up: int = _input_map.consume_shift_up_count()
	var down: int = _input_map.consume_shift_down_count()
	if not live or not bool(_simulation.is_running()):
		_input_map.advance_cabin(_simulation,{},delta,false,VEHICLE_NAME)
		_steering.reset()
		return
	if get_tree().get_nodes_in_group("seat_adjustment_open").size() > 0 or get_tree().get_nodes_in_group("address_teleport_open").size() > 0:
		_input_map.driver_actions.discard_pending()
		_simulation.set_control("steer", 0.0)
		_simulation.set_control("throttle", 0.0)
		_simulation.set_control("brake", 1.0)
		_input_map.advance_cabin(_simulation,{},delta,false,VEHICLE_NAME)
		return
	if not scripted_controls.is_empty():
		_input_map.advance_cabin(_simulation,{},delta,false,VEHICLE_NAME)
		for channel in scripted_controls:
			_simulation.set_control(channel, float(scripted_controls[channel]))
		return
	# Clickable cockpit (cockpit_controls.gd): a dragged pedal/lever/wheel wins over the input device while held,
	# the gear lever adds shifts, the ignition key sets the ignition and holds the starter in its start detent.
	# These channels (pedals, steer, shifts, ignition, starter) are not owned by RgDriverActions, so they stay direct
	# writes; the cockpit's hazard/indicator/horn switches submit DriverActions events (cockpit_controls.gd) and
	# the light/wiper/climate/radio switches are still visual state only (get_cosmetic).
	var cockpit: Node3D = _visuals.get_cockpit() if _visuals != null and _visuals.has_method("get_cockpit") else null
	var cockpit_overrides: Dictionary = {}
	var cockpit_starter := false
	if cockpit != null:
		cockpit_overrides = cockpit.get_overrides()
		up += cockpit.consume_shift_up()
		down += cockpit.consume_shift_down()
		var ignition_request: int = cockpit.consume_ignition_request()
		if ignition_request >= 0:
			_input_map.set_ignition(ignition_request == 1)
		cockpit_starter = cockpit.starter_held()
	var steer: float = _steering.translate(_input_map.get_steer(), _input_map.steering_uses_keyboard(), _input_map.steering_uses_wheel(), _simulation.get_steering_kinematics(), delta)
	_simulation.set_control("steer", float(cockpit_overrides.get("steer", steer)))
	_simulation.set_control("throttle", maxf(_input_map.get_throttle(), float(cockpit_overrides.get("throttle", 0.0))))
	_simulation.set_control("brake", maxf(_input_map.get_brake(), float(cockpit_overrides.get("brake", 0.0))))
	_simulation.set_control("handbrake", maxf(_input_map.get_handbrake(), float(cockpit_overrides.get("handbrake", 0.0))))
	_simulation.set_control("clutch", maxf(_input_map.get_clutch(), float(cockpit_overrides.get("clutch", 0.0))))
	_simulation.set_control("ignition", 1.0 if _input_map.get_ignition() else 0.0)
	# The cockpit's ignition key holds the starter in its start detent (cockpit_controls.gd), ORed with the device starter.
	_simulation.set_control("starter", 1.0 if (_input_map.get_starter() or cockpit_starter) else 0.0)
	if cockpit != null:
		cockpit.submit_held()
	var pt: Dictionary = _simulation.get_vehicle_powertrain(VEHICLE_NAME)
	pt["speed_mps"] = _simulation.get_body_speed_mps("chassis")
	var channels: Dictionary = DriverState.channels(_simulation)
	var drive_mode_change := -1
	for write in _input_map.driver_actions.apply(pt,channels,true):
		_simulation.set_control(write["channel"],float(write["value"]))
		if write["channel"] == "ignition": _input_map.sync_ignition(float(write["value"]))
		if write["channel"] == "assist.drive_mode": drive_mode_change = roundi(write["value"])
	_input_map.advance_cabin(_simulation,pt,delta,true,VEHICLE_NAME)
	_save_light_mode_if_changed()
	if drive_mode_change >= 0 and _hud != null:
		var requested := pt.duplicate()
		requested["drive_mode_effective"] = drive_mode_change
		var state: Dictionary = RgDriverActions.state("drive_mode",requested,channels)
		_hud.show_message("Drive mode: " + str(state["label"]),2.0,DriveModeTheme.color_for_mode(drive_mode_change))
	# shift_up_count/shift_down_count are running edge counters on the sim
	# side (session.cpp's kControlChannelNames): add this frame's edges.
	# Automatic and dual-clutch gearboxes (physics automatic_tcu): the shift keys step the selector P-R-N-D
	# (rg::selector_shift decides); in D with manual mode on (Y) they change gears.
	# Every hybrid has the same selector whatever its gearbox (the hybrid supervisor owns the gearbox, a manual box included).
	if (up > 0 or down > 0) and (int(pt.get("transmission_kind",0)) != 0 or bool(pt.get("hybrid_fitted",false))):
		var sel: Dictionary = RgDriverActions.selector_shift(pt,up,down)
		if sel["change"]:
			_simulation.set_control("gear_request",float(sel["gear_request"]))
		if int(sel["park_request"]) >= 0:
			_simulation.set_control("park_lock",float(sel["park_request"]))
		if sel["refused"] and _hud != null:
			_hud.show_message("Stop the car first: P and R are refused while rolling",2.5)
		if not sel["forward_counts"]:
			up = 0
			down = 0
	elif up > 0 or down > 0:
		_simulation.set_control("park_lock",0.0)
	if up > 0:
		_simulation.set_control("shift_up_count", _simulation.get_control("shift_up_count") + up)
	if down > 0:
		_simulation.set_control("shift_down_count", _simulation.get_control("shift_down_count") + down)

# Frame-time diagnostics (render stutter investigation 2026-10-14). The delta of THIS frame is the cost of the previous
# one (all of its _process calls, the render and the vsync wait), so the probes taken now (FrameProbe, frame_probe.gd)
# belong to the frame that took `delta`.
#  RG_FRAME_SPIKE  one line per frame over RG_FRAME_SPIKE_MS (default 50): delta_ms, ticks, process_ms (Godot's own
#     TIME_PROCESS monitor: the LONGEST process+draw of the last whole second, refreshed once a second - not this
#     frame, and not an average; the ph_* probes below are the per-frame split), render_cpu_ms / render_gpu_ms (main viewport; gpu 0 when the driver gives no
#     timer), terrain_* (RgTerrainView per-frame costs), probes = the nodes' own _process costs, largest first. A spike
#     whose probes sum to a few ms while delta is large was spent OUTSIDE the scripts (render, GPU, driver, OS).
#  RG_FRAME_STATS  once per second: fps, frame ms avg/p99/max, frames over 25 / 50 ms, draw calls / objects /
#     primitives of the last frame, vsync mode / max_fps / window size, per probe avg/max ms. Probes besides the
#     nodes' own: scene_span, outside and the engine phases ph_loop / ph_physics / ph_pre / ph_post / ph_draw /
#     render_cpu / render_gpu (frame_probe_end.gd explains each).
func _frame_diag(delta: float) -> void:
	var probes: Dictionary = FrameProbe.take_frame()
	if world_state != "running" or _simulation == null:
		return
	var ms := delta * 1000.0
	if not _perf:
		if delta > _fd_spike_s:
			print("RG_FRAME_SPIKE delta_ms=%.1f ticks=%d (RG_PERF=1 adds the probes)" % [ms, int(_simulation.get_step_count())])
		return
	if probes.has("scene_span"):
		probes["outside"] = maxi(0, int(delta * 1000000.0) - int(probes["scene_span"])) # engine render / driver / vsync / OS
	var process_ms: float = Performance.get_monitor(Performance.TIME_PROCESS) * 1000.0
	_fd_frames.append(ms)
	_fd_process_sum += process_ms
	_fd_process_max = maxf(_fd_process_max, process_ms)
	for k in probes:
		var v := float(probes[k]) / 1000.0
		_fd_probe_sum[k] = float(_fd_probe_sum.get(k, 0.0)) + v
		_fd_probe_max[k] = maxf(float(_fd_probe_max.get(k, 0.0)), v)
	if delta > _fd_spike_s:
		var keys: Array = probes.keys()
		keys.sort_custom(func(a, b) -> bool: return probes[a] > probes[b])
		var parts := PackedStringArray()
		for i in range(mini(keys.size(), 8)):
			parts.append("%s=%.1f" % [keys[i], float(probes[keys[i]]) / 1000.0])
		var vp := get_viewport().get_viewport_rid()
		var tv := ""
		if _world_view != null and _world_view.has_method("get_frame_costs"):
			var c: Dictionary = _world_view.get_frame_costs()
			tv = " terrain_upload_ms=%.2f terrain_heal_ms=%.2f terrain_decks_ms=%.2f terrain_pending=%d" % [
				float(c.get("upload_ms", 0.0)), float(c.get("heal_ms", 0.0)), float(c.get("decks_ms", 0.0)), int(c.get("pending_work", 0))]
		print("RG_FRAME_SPIKE delta_ms=%.1f ticks=%d process_ms=%.1f render_cpu_ms=%.1f render_gpu_ms=%.1f%s probes=[%s]" % [
			ms, int(_simulation.get_step_count()), process_ms,
			RenderingServer.viewport_get_measured_render_time_cpu(vp) if _render_timing else -1.0,
			RenderingServer.viewport_get_measured_render_time_gpu(vp) if _render_timing else -1.0,
			tv, ", ".join(parts)])
	var now := Time.get_ticks_msec()
	if now - _fd_last_report_ms < 1000:
		return
	_fd_last_report_ms = now
	var n := _fd_frames.size()
	if n > 0:
		var sorted := _fd_frames.duplicate()
		sorted.sort()
		var sum := 0.0
		var over25 := 0
		var over50 := 0
		for f in _fd_frames:
			sum += f
			over25 += 1 if f > 25.0 else 0
			over50 += 1 if f > 50.0 else 0
		var pk := PackedStringArray()
		for k in _fd_probe_sum:
			pk.append("%s=%.2f/%.1f" % [k, float(_fd_probe_sum[k]) / n, float(_fd_probe_max[k])])
		pk.sort()
		# draw_calls/objects/primitives: the last drawn frame's totals over every viewport (Godot's own monitors);
		# vsync/max_fps/size: what the window actually runs with, so a log names its own settings.
		var win := get_window()
		var ab := "" if _ab_feature == "" else " ab=%s:%s" % [_ab_feature, "off" if _ab_off else "on"]
		print("RG_FRAME_STATS frames=%d fps=%d frame_ms_avg=%.1f p99=%.1f max=%.1f over25=%d over50=%d process_ms_avg=%.1f process_ms_max=%.1f draw_calls=%d objects=%d primitives=%d vsync=%d max_fps=%d size=%dx%d renderer=%s/%s render_thread=%s%s probes_avg/max_ms=[%s]" % [
			n, int(Engine.get_frames_per_second()), sum / n, sorted[mini(n - 1, int(ceil(0.99 * n)) - 1)], sorted[n - 1], over25, over50,
			_fd_process_sum / n, _fd_process_max,
			int(Performance.get_monitor(Performance.RENDER_TOTAL_DRAW_CALLS_IN_FRAME)),
			int(Performance.get_monitor(Performance.RENDER_TOTAL_OBJECTS_IN_FRAME)),
			int(Performance.get_monitor(Performance.RENDER_TOTAL_PRIMITIVES_IN_FRAME)),
			int(DisplayServer.window_get_vsync_mode()), Engine.max_fps, win.size.x, win.size.y,
			RenderingServer.get_current_rendering_method(), RenderingServer.get_current_rendering_driver_name(), _render_thread_mode(), ab, ", ".join(pk)])
	_fd_frames.clear()
	_fd_probe_sum.clear()
	_fd_probe_max.clear()
	_fd_process_sum = 0.0
	_fd_process_max = 0.0
	if _ab_feature != "" and now - _ab_switched_ms >= _ab_period_ms:
		_ab_switched_ms = now
		if _ab_off and _ab_list.size() > 1:
			_perf_ab_apply(_ab_feature, false) # back on, then the next feature's "on" period
			_ab_off = false
			_ab_index = (_ab_index + 1) % _ab_list.size()
			_ab_feature = _ab_list[_ab_index]
		else:
			_ab_off = not _ab_off
			_perf_ab_apply(_ab_feature, _ab_off)
		print("RG_PERF_AB %s -> %s" % [_ab_feature, "off" if _ab_off else "on"])

# Switches one render feature for the RG_PERF_AB comparison (diagnosis only; nothing else calls it).
func _perf_ab_apply(feature: String, off: bool) -> void:
	match feature:
		"shadows":
			if _sky != null and _sky.sun != null:
				_sky.sun.shadow_enabled = not off
		"3d":
			get_viewport().disable_3d = off
		"terrain":
			if _world_view != null:
				_world_view.set_terrain_visible(not off)
		"terrain_shadows":
			if _world_view != null:
				_world_view.set_cast_shadows(not off)
		"roads":
			if _roads != null:
				_roads.visible = not off
		"buildings":
			if _buildings != null:
				_buildings.visible = not off
		"car":
			if _visuals != null:
				_visuals.visible = not off
		"hud":
			if _hud != null:
				_hud.visible = not off
		"navigation":
			if _navigation != null:
				_navigation.visible = not off
		"traffic":
			if _traffic_visual != null:
				_traffic_visual.visible = not off
		"audio":
			if _vehicle_audio != null:
				_vehicle_audio.set_process(not off)
		"sky":
			if _sky != null and _sky.environment != null:
				if _ab_saved_bg < 0:
					_ab_saved_bg = _sky.environment.background_mode
				_sky.environment.background_mode = Environment.BG_COLOR if off else _ab_saved_bg
		_:
			push_warning("RG_PERF_AB: unknown feature '%s'" % feature)

func _process(delta: float) -> void:
	var process_t0 := Time.get_ticks_usec()
	FrameProbe.frame_start_us = process_t0
	_process_body(delta)
	FrameProbe.add("main", Time.get_ticks_usec() - process_t0)

func _process_body(delta: float) -> void:
	# Main-thread frame hitches, to line up with RG_TICK_SPIKE (sim thread).
	_frame_diag(delta)
	if _terrain_view != null and not _terrain_preview_reported and bool(_terrain_view.is_fully_uploaded()):
		_terrain_preview_reported = true
		print("terrain preview loaded: chunks=%d vertices=%d upload_ms=%.2f" % [
			_terrain_view.get_chunk_count(), _terrain_view.get_total_vertex_count(),
			_terrain_view.get_total_upload_time_ms()
		])
	if _terrain_view != null and _terrain_focus_camera != null:
		var focus: Vector3 = _terrain_view.godot_to_session(_terrain_focus_camera.global_position)
		_terrain_view.update_focus(focus.x, focus.y)

	if _simulation == null or _director == null:
		return

	_input_map.poll()
	# In-world actions only count on the Drive screen; the edge counts are consumed
	# every frame either way so a press in a menu never arrives later.
	var in_drive := _screen == "drive"
	if in_drive and world_state == "running":
		_record_drive_diag_sample()
	var truck_presses: int = _input_map.consume_npc_truck()
	if in_drive and world_state == "running" and get_tree().get_nodes_in_group("address_teleport_open").is_empty() and truck_presses > 0:
		_simulation.request_npc_truck(not Input.is_key_pressed(KEY_SHIFT), 70.0)
	var camera_presses: int = _input_map.consume_cycle_camera()
	var view_presses: int = _input_map.consume_cycle_view()
	if in_drive and not _vr_active:
		var tab_mode: String = _simulation.get_player_mode()
		if camera_presses % 2 == 1:
			if tab_mode == "drive":
				set_drive_view(RgCameraMath.toggle_cockpit_view(_drive_view))
			elif tab_mode == "on_foot":
				_walker_rig.toggle_first_person()
		if tab_mode == "drive":
			for _i in range(view_presses):
				set_drive_view(RgCameraMath.next_drive_view(_drive_view))
	var mode_presses: int = _input_map.consume_cycle_mode()
	for _i in range(mode_presses if in_drive else 0):
		print("RG_WORLD mode -> %s" % _simulation.cycle_player_mode())
	# On foot (R9c): G gets out of a (nearly stopped) car in drive mode and gets
	# back in when standing at its door; Space jumps. The counts are consumed
	# every frame so presses in the wrong mode never arrive later.
	var get_out_presses: int = _input_map.consume_get_out_key_count()
	var interact_presses: int = _input_map.consume_interact_count()
	var jump_presses: int = _input_map.consume_jump_count()
	if in_drive and world_state == "running":
		var foot_mode: String = _simulation.get_player_mode()
		if foot_mode == "drive" and get_out_presses > 0 and not _vr_active:
			print("RG_WORLD get out -> %s" % _simulation.set_player_mode("on_foot"))
		elif foot_mode == "on_foot":
			if interact_presses > 0:
				_simulation.request_walker_enter()
			for _i in range(jump_presses):
				_simulation.request_walker_jump()
	# Next drone-follow target (own car -> nearest NPC vehicles -> own car);
	# presses outside drone_follow are consumed and ignored.
	for _i in range(_input_map.consume_cycle_drone_target()):
		if in_drive and world_state == "running" and _simulation.get_player_mode() == "drone_follow":
			_simulation.cycle_drone_target()
	if _input_map.consume_switch_world() > 0 and (in_drive or _screen == "loading"):
		switch_world()
	for _i in range(_input_map.consume_log_marker()):
		if in_drive and world_state == "running": _log_owner_mark()
	if _input_map.consume_reset_car() > 0 and in_drive and world_state == "running":
		_simulation.reset_vehicle_to_spawn()
	if _input_map.consume_flip_upright() > 0 and in_drive and world_state == "running" and _simulation.get_player_mode() == "drive":
		_simulation.flip_vehicle_upright()

	match world_state:
		"loading":
			_poll_loading()
		"starting":
			_attach_world_view()
		"running":
			var report_t0 := Time.get_ticks_usec()
			_report()
			FrameProbe.add("report", Time.get_ticks_usec() - report_t0)

	if world_state == "running" and _vehicle_audio == null:
		_vehicle_audio = Node3D.new()
		_vehicle_audio.name = "VehicleAudio"
		_vehicle_audio.set_script(load("res://scripts/vehicle_audio.gd"))
		_vehicle_audio.simulation = _simulation
		_vehicle_audio.director = _director
		_vehicle_audio.vehicle_name = VEHICLE_NAME
		add_child(_vehicle_audio)
		_fpv_audio = Node.new()
		_fpv_audio.name = "FpvAudio"
		_fpv_audio.set_script(load("res://scripts/fpv_audio.gd"))
		_fpv_audio.simulation = _simulation
		add_child(_fpv_audio)
		_apply_audio_settings()
	if _vehicle_audio != null:
		_vehicle_audio.muted = _paused
	var ms := _apply_mode_state()
	var forward_t0 := Time.get_ticks_usec()
	_forward_driving(bool(ms.get("driving_inputs_live", false)) and in_drive, delta)
	FrameProbe.add("forward_driving", Time.get_ticks_usec() - forward_t0)
	_forward_walking(bool(ms.get("walking_inputs_live", false)) and in_drive)
	_forward_flying(bool(ms.get("flying_inputs_live", false)) and in_drive)
	if _fpv_audio != null:
		_fpv_audio.muted = _paused
	_panels_open_prev = _panel_open()
	_mouse_captured_prev = _input_map.is_mouse_captured()

var _vr_active := false
var _xr_menu: Node3D

func _try_start_vr() -> void:
	var xr := XRServer.find_interface("OpenXR")
	if xr == null or not xr.is_initialized():
		print("RG_VR inactive: OpenXR not initialized; using desktop cameras")
		return
	DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
	get_viewport().use_xr = true
	for rig_name in ["xr_cockpit", "xr_free", "xr_drone"]:
		var rig := XROrigin3D.new()
		rig.name = rig_name
		rig.set_script(load("res://scripts/xr_rig.gd"))
		rig.body_visuals = _visuals
		rig.free_flight = rig_name == "xr_free"
		rig.drone_follow = rig_name == "xr_drone"
		add_child(rig)
		_director.add_rig(rig_name, rig)
	_vr_active = true
	_xr_menu = load("res://scripts/xr_menu.gd").new()
	_xr_menu.main = self
	add_child(_xr_menu)
	print("RG_VR active: tracked cockpit, F9 recenter; FreeCam uses tracked free flight, drone follow trails the target (xr_drone)")

# Owner bookmarks survive normal log rotation in user://drive_marks.jsonl.
func _input(event: InputEvent) -> void:
	# A value box has the keyboard: a menu_back binding on a letter or digit is text there, only Esc leaves.
	var typing_in_box: bool = event is InputEventKey and event.keycode != KEY_ESCAPE and _typing()
	if _shell != null and (_screen == "configurator" or (_xr_menu != null and _xr_menu.active)) and event.is_action_pressed("ui_cancel") and not typing_in_box:
		_on_escape()
		get_viewport().set_input_as_handled()
		return
	if _shell != null and event is InputEventKey and event.pressed and not event.echo:
		if event.keycode == KEY_ESCAPE:
			_on_escape()
	# Pad Menu/Start opens and closes the pause menu like Esc (owner 2026-10-06).
	if _shell != null and event is InputEventJoypadButton and event.pressed and event.button_index == JOY_BUTTON_START:
		_on_escape()

# --- garage (R6) ------------------------------------------------------------------

# The catalog's default car (data/vehicles/catalog.json "default").
func _default_vehicle_id() -> String:
	for v in _garage.get_vehicles():
		if bool(v["default"]):
			return str(v["id"])
	var all: Array = _garage.get_vehicles()
	return str(all[0]["id"]) if not all.is_empty() else "car_hyper"

# Which car the next world load drives: a direct-start flag run the catalog default
# (or --vehicle), a menu start the player's saved choice.
func _vehicle_choice() -> String:
	return _direct_vehicle if _direct_start else str(_garage.get_selected_id())

# Materialises the chosen car's saved setup (RgGarage) and points every consumer of
# the car at it: simulation chassis/cache overrides, model + colours, HUD/gauge names.
func _prepare_vehicle() -> Dictionary:
	var sel: Dictionary = _garage.prepare_drive(_vehicle_choice())
	if bool(sel.get("ok", false)):
		_apply_vehicle(sel)
	return sel

func _apply_vehicle(sel: Dictionary) -> void:
	_drive_selection = sel
	VEHICLE_NAME = str(sel["sim_name"])
	var chassis: Dictionary = sel["chassis"]
	_simulation.set_vehicle_overrides({
		"mass_kg": float(chassis["mass_kg"]),
		"half_extents": chassis["half_extents"],
		"z_m": float(chassis["z_m"]),
		"engine_map_cache_dir": str(sel["engine_map_cache_dir"]),
		"turbo_map_extension_path": str(sel.get("turbo_map_extension_path", "")),
		"damage_config_path": str(sel.get("damage_config_path", "")),
		"damage_state_path": str(sel.get("damage_state_path", "")),
	})
	# optional model parts: the setup's (a folding top) plus the raised hood a tall swapped-in engine needs
	var parts: Array = Array(sel.get("model_options", []))
	var bay: Dictionary = _garage.get_bay_engine(str(sel.get("vehicle_id", ""))) if _garage.has_method("get_bay_engine") else {}
	if not bay.is_empty():
		var hood: String = preload("res://scripts/model_options.gd").hood_variant(str(sel["model_path"]).get_basename() + ".rig.json", float((bay.get("envelope_mm", Vector3.ZERO) as Vector3).z))
		if hood != "":
			parts.append(hood)
	_visuals.set_vehicle(VEHICLE_NAME, str(sel["model_path"]), str(sel["paint"]), str(sel["rim"]), parts, Array(sel.get("hidden_nodes", [])))
	_hud.vehicle_name = VEHICLE_NAME
	_gauge.set_vehicle_name(VEHICLE_NAME)
	_seat_ui.vehicle_name = VEHICLE_NAME
	if str(sel.get("warning", "")) != "":
		print("RG_GARAGE warning: %s" % sel["warning"])
	print("RG_GARAGE drive car=%s sim=%s modified=%s path=%s" % [sel["vehicle_id"], VEHICLE_NAME, sel["modified"], sel["vehicle_path"]])

func get_garage() -> Node:
	return _garage

func get_garage_scene() -> CanvasLayer:
	return _garage_scene

func get_drive_selection() -> Dictionary:
	return _drive_selection

func _open_garage() -> void:
	if _garage_scene != null:
		return
	_garage_scene = load("res://scripts/garage_scene.gd").new()
	_garage_scene.name = "GarageScene"
	_garage_scene.garage = _garage
	add_child(_garage_scene)
	if _xr_menu != null: _xr_menu.attach(_garage_scene)

# Removes the garage scene and every node it created (the acceptance test counts
# the tree), ends any edit session and removes the scratch files.
func _close_garage() -> void:
	_garage.discard()
	if _garage_scene != null:
		_garage_scene.get_parent().remove_child(_garage_scene)
		_garage_scene.queue_free()
		_garage_scene = null

# The car browser screens (R6c): vehicle_select (main menu / pause "Garage", leads on to the
# configurator) and change_car (pause, swaps the car in place). Both show the same browser; the
# garage 3D scene is not drawn behind it (the browser's backdrop is opaque), the thumbnail
# renderer exists only while a browser is up, and the saved group/sort/filter state is loaded.
func _enter_car_browser() -> void:
	_garage.discard() # coming back from the configurator ends the edit session
	if _garage_scene == null:
		_open_garage()
	_garage_scene.set_rendering(false)
	_garage.browser_load_state(_browser_state_from_settings())
	_garage.browser_refresh()
	_make_thumbs()
	_shell_ui.thumbs = _thumbs

func _make_thumbs() -> void:
	if _thumbs != null:
		return
	_thumbs = load("res://scripts/car_thumbnails.gd").new()
	_thumbs.name = "CarThumbnails"
	add_child(_thumbs)

func _free_thumbs() -> void:
	if _thumbs == null:
		return
	_shell_ui.thumbs = null
	remove_child(_thumbs)
	_thumbs.queue_free()
	_thumbs = null

func get_thumbnails() -> Node:
	return _thumbs

const BROWSER_KEYS := ["group_by", "sort_key", "sort_desc", "filter_layouts", "filter_body_types", "filter_power_bands"]

func _browser_state_from_settings() -> Dictionary:
	var state := {}
	for key in BROWSER_KEYS:
		state[key] = _shell.get_setting("browser." + key)
	return state

# The browser changed its group / sort / filter: keep it in the settings file.
func _on_browser_state_changed() -> void:
	var state: Dictionary = _garage.browser_get_state()
	for key in BROWSER_KEYS:
		_shell.set_setting("browser." + key, state[key])
	_shell.save_settings()

func _enter_configurator() -> void:
	var id := str(_shell.get_garage_vehicle())
	_shell_ui.garage_vehicle = id
	var begun: Dictionary = _garage.begin_edit(id)
	if not bool(begun.get("ok", false)):
		push_error("RG_GARAGE begin_edit failed: %s" % begun.get("error", "?"))
	if _garage_scene == null:
		_open_garage()
	_garage_scene.set_rendering(true)
	_garage_scene.show_vehicle(_garage.get_vehicle(id))
	var colours: Dictionary = _garage.get_working_colours()
	_garage_scene.set_colours(str(colours.get("paint", "")), str(colours.get("rim", "")))
	_garage_scene.set_model_options(Array(colours.get("model_options", [])), Array(colours.get("hidden_nodes", [])))
	_garage_scene.panel_side = "right"
	_garage_scene.panel_px = 528.0
	_garage_scene.go_to_area("overview", false)

func _on_vehicle_chosen(id: String) -> void:
	_garage.select(id)
	_direct_vehicle = id # a "Change car" in a direct-start session drives the new car too
	_apply_transition(_shell.vehicle_chosen(id))

func _on_garage_area_chosen(area_id: String) -> void:
	if _garage_scene != null:
		_garage_scene.go_to_area(area_id, false)

func _on_garage_option_changed(option_id: String) -> void:
	if _garage_scene == null:
		return
	if option_id == "paint" or option_id == "rim" or option_id == "":
		var colours: Dictionary = _garage.get_working_colours()
		_garage_scene.set_colours(str(colours.get("paint", "")), str(colours.get("rim", "")))
	# optional model parts (the Runabout's folding top): shown as soon as the option changes
	var working: Dictionary = _garage.get_working_colours()
	_garage_scene.set_model_options(Array(working.get("model_options", [])), Array(working.get("hidden_nodes", [])))

func _on_damage_repaired(state_path: String) -> void:
	if _simulation != null and _simulation.has_method("repair_damage"):
		_simulation.repair_damage(state_path)

# RgShell action repair_car (damage model, rg/damage.h; free): vehicle "" = the car being driven (pause menu "Repair car"),
# else a catalog id from the garage's first page. The running car is repaired at once when it is that car.
func _repair_car(vehicle_id: String) -> void:
	if vehicle_id == "":
		var ok := false
		if _simulation != null and _simulation.has_method("repair_damage") and world_state == "running":
			ok = _simulation.repair_damage("")
		print("RG_DAMAGE repair running car ok=%s" % ok)
		if ok and _hud != null and _hud.has_method("show_message"):
			_hud.show_message("Car repaired")
		return
	var damage: Dictionary = _garage.get_damage(vehicle_id)
	var reply: Dictionary = _garage.repair_damage(vehicle_id)
	print("RG_DAMAGE repair %s ok=%s %s" % [vehicle_id, reply.get("ok", false), reply.get("error", "")])
	if bool(reply.get("ok", false)):
		_on_damage_repaired(str(damage.get("state_path", "")))
	_shell_ui.refresh_browser()

func _on_garage_save() -> void:
	var saved: Dictionary = _garage.save()
	print("RG_GARAGE save ok=%s %s" % [saved.get("ok", false), saved.get("error", "")])
	_shell_ui.refresh_configurator()

# Drive: valid unsaved changes are saved first; an invalid working copy cannot be driven.
func _on_garage_exit(destination: String) -> void:
	if destination not in ["drive", "selection"]:
		return
	var saved: Dictionary = _garage.save()
	if not bool(saved.get("ok",false)):
		_shell_ui.garage_exit_failed(str(saved.get("error","Save failed")))
		return
	if destination == "drive":
		_on_garage_drive()
	else:
		_apply_transition(_shell.back())

func _on_garage_drive() -> void:
	if _garage.is_dirty():
		var saved: Dictionary = _garage.save()
		if not bool(saved.get("ok", false)):
			_shell_ui.refresh_configurator()
			return
	_garage.discard()
	_apply_transition(_shell.garage_drive())

# --- shell (R5) -------------------------------------------------------------------

func _user_dir() -> String:
	return _user_dir_override if _user_dir_override != "" else ProjectSettings.globalize_path("user://")

var _user_dir_override: String = ""
const RendererChoice := preload("res://scripts/renderer_choice.gd")
var _renderer_reported := false

func _apply_user_dir_flag(user_args: PackedStringArray) -> void:
	var dir_index := user_args.find("--shell-user-dir")
	if dir_index >= 0 and dir_index + 1 < user_args.size():
		_user_dir_override = user_args[dir_index + 1]
		DirAccess.make_dir_recursive_absolute(_user_dir_override)

# The control configuration (R5b): schema, default profiles and the player's
# user://controls.json, loaded before anything reads a device. A broken file
# never stops the game: defaults, the bad file kept as controls.json.bak.
func _build_controls(user_args: PackedStringArray) -> void:
	_apply_user_dir_flag(user_args)
	_controls = ClassDB.instantiate("RgControls")
	_controls.name = "Controls"
	add_child(_controls)
	var report: Dictionary = _controls.initialize(_rg_data_path(""), _user_dir())
	if not bool(report.get("ok", false)):
		push_error("RG_CONTROLS initialise failed: %s" % report.get("error", "?"))
		return
	print("RG_CONTROLS ready file=%s%s" % [report.get("path", ""), " (new)" if bool(report.get("file_missing", false)) else ""])
	if str(report.get("message", "")) != "":
		print("RG_CONTROLS warning: %s" % report["message"])
		for line in report.get("dropped", PackedStringArray()):
			print("RG_CONTROLS dropped: %s" % line)

func get_controls() -> Node:
	return _controls

func get_input_map() -> Node:
	return _input_map

# One shared dispatcher for future cockpit mouse, VR and HUD action sources.
func get_driver_actions() -> RgDriverActions:
	return _input_map.driver_actions

func _build_shell(user_args: PackedStringArray) -> void:
	_apply_user_dir_flag(user_args)
	_shell = ClassDB.instantiate("RgShell")
	_shell.name = "Shell"
	add_child(_shell)
	var report: Dictionary = _shell.initialize(_rg_data_path(""), _user_dir(), _world_config_path())
	for problem in report.get("problems", PackedStringArray()):
		print("RG_SHELL problem: %s" % problem)
	print("RG_SHELL ready credits=%d presets=%d settings=%s%s" % [
		int(report.get("credits_entries", 0)), int(report.get("preset_count", 0)), report.get("settings_path", ""),
		" (new)" if bool(report.get("settings_file_missing", false)) else ""])
	_shell_ui = CanvasLayer.new()
	_shell_ui.name = "ShellUi"
	_shell_ui.set_script(load("res://scripts/shell_ui.gd"))
	_shell_ui.shell = _shell
	_shell_ui.fixed_keys_path = _rg_data_path("controls/fixed_keys.json")
	_shell_ui.controls = _controls
	_shell_ui.input_map = _input_map
	_shell_ui.controls_changed.connect(func(): _input_map.sync_menu_actions())
	_shell_ui.menu_item_chosen.connect(func(id: String): _apply_transition(_shell.menu_item(id)))
	_shell_ui.spawn_chosen.connect(func(id: String): _apply_transition(_shell.spawn_picked(id)))
	_shell_ui.back_requested.connect(func(): _apply_transition(_shell.back()))
	_shell_ui.boot_finished.connect(func(): _apply_transition(_shell.boot_finished()))
	_shell_ui.setting_changed.connect(_apply_setting)
	_garage = ClassDB.instantiate("RgGarage")
	_garage.name = "Garage"
	add_child(_garage)
	var catalog_flag := user_args.find("--catalog")
	if catalog_flag >= 0 and catalog_flag + 1 < user_args.size():
		_catalog_override = user_args[catalog_flag + 1]
	var garage_report: Dictionary = _garage.initialize(ProjectSettings.globalize_path("res://").path_join("..").simplify_path(), _user_dir(), _user_dir().path_join("garage_work"), _catalog_override)
	if bool(garage_report.get("ok", false)):
		print("RG_GARAGE ready vehicles=%d selected=%s" % [_garage.get_vehicles().size(), _garage.get_selected_id()])
	else:
		push_error("RG_GARAGE initialise failed: %s" % garage_report.get("error", "?"))
	_direct_vehicle = _default_vehicle_id()
	var vehicle_flag := user_args.find("--vehicle")
	if vehicle_flag >= 0 and vehicle_flag + 1 < user_args.size():
		_direct_vehicle = user_args[vehicle_flag + 1]
	_shell_ui.garage = _garage
	_shell_ui.browser_state_changed.connect(_on_browser_state_changed)
	_shell_ui.vehicle_chosen.connect(_on_vehicle_chosen)
	_shell_ui.garage_area_chosen.connect(_on_garage_area_chosen)
	_shell_ui.garage_option_changed.connect(_on_garage_option_changed)
	_shell_ui.garage_save_requested.connect(_on_garage_save)
	# A garage repair of the car that is running (pause -> garage) applies at once, not only at the next world load.
	_shell_ui.damage_repaired.connect(_on_damage_repaired)
	_shell_ui.repair_requested.connect(func(id: String): _apply_transition(_shell.repair_chosen(id)))
	_shell_ui.garage_exit_requested.connect(_on_garage_exit)
	add_child(_shell_ui)
	if _xr_menu != null: _xr_menu.attach(_shell_ui)
	_overlay.attribution_line = str(_shell.get_attribution_line())
	for section in _shell.get_settings_schema():
		for def in section["settings"]:
			_apply_setting(str(def["key"]))

# Carries out what rg::ShellFlow decided. `show_screen` is always the last action
# of a transition, so a load or an unload has been started when the new screen
# appears.
func _apply_transition(transition: Dictionary) -> void:
	if not bool(transition.get("accepted", false)):
		return
	shell_log.append("%s>%s" % [transition["from"], transition["to"]])
	print("RG_SHELL %s -> %s" % [transition["from"], transition["to"]])
	for action in transition["actions"]:
		match str(action["kind"]):
			"show_screen":
				_show_screen(str(action["screen"]))
			"load_world":
				_begin_world_load(action["world"], bool(action["flag"]))
			"open_garage":
				_open_garage()
			"close_garage":
				_close_garage()
			"unload_world":
				_unload_world()
			"set_paused":
				_set_paused(bool(action["flag"]))
			"reset_car":
				if world_state == "running":
					_simulation.reset_vehicle_to_spawn()
			"repair_car":
				_repair_car(str(action.get("vehicle", "")))
			"save_settings":
				_shell.save_settings()
			"save_controls":
				_controls.save()
			"quit":
				_quit_game()

func _show_screen(screen_name: String) -> void:
	_screen = screen_name
	if screen_name == "vehicle_select" or screen_name == "change_car":
		_enter_car_browser()
	elif screen_name == "configurator":
		_free_thumbs()
		_enter_configurator()
	else:
		_free_thumbs()
	_input_map.set_world_input_active(screen_name == "drive")
	_input_map.suspended = screen_name == "controls" # the controls screen reads devices itself
	_shell_ui.show_screen(screen_name)
	if _xr_menu != null: _xr_menu.show_screen(screen_name)
	_set_world_ui_visible(screen_name == "drive" or screen_name == "pause")
	if screen_name != "loading":
		_overlay.hide_overlay()
	if screen_name != "drive" and screen_name != "loading":
		_input_map.release_mouse()

func _set_world_ui_visible(visible_now: bool) -> void:
	_hud.visible = visible_now
	_gauge.visible = visible_now
	if _weather_widget != null:
		_weather_widget.visible = visible_now

# A direct start into the real world may name a spawn preset with --spawn=ID
# (data/world/spawn_presets.json; tools/run.ps1 -VR passes --spawn=a5_north).
# An unknown id falls back to the world's own spawn with a warning.
func _direct_start_world(start_world: String, user_args: PackedStringArray) -> Dictionary:
	var world := {"kind": start_world, "has_spawn": false}
	if start_world != "real_world":
		return world
	for a in user_args:
		if not a.begins_with("--spawn="):
			continue
		var id := a.get_slice("=", 1)
		var path := ProjectSettings.globalize_path("res://").path_join("../data/world/spawn_presets.json").simplify_path()
		var data = JSON.parse_string(FileAccess.get_file_as_string(path))
		if data is Dictionary:
			for p in data.get("presets", []):
				if str(p.get("id", "")) == id:
					print("RG_SPAWN preset=%s x=%.1f y=%.1f yaw=%.1f" % [id, float(p["x"]), float(p["y"]), float(p["yaw_deg"])])
					return {"kind": start_world, "has_spawn": true, "x": float(p["x"]), "y": float(p["y"]),
						"yaw_deg": float(p["yaw_deg"]), "label": str(p.get("name", id))}
		push_warning("--spawn=%s: no such preset in %s; using the world's own spawn" % [id, path])
	return world

func _begin_world_load(world: Dictionary, respawn: bool = false) -> void:
	_world_request = world.duplicate()
	var kind := str(world.get("kind", "flat"))
	var old_pose: Dictionary = _simulation.get_chassis_session_pose() if respawn and world_state == "running" else {}
	var keep_override := false
	if respawn and not old_pose.is_empty():
		# the garage respawn / "Change car": the new car starts where the old one stood, same heading
		# (the flat world applies it as a relocation right after the session exists)
		_simulation.set_spawn_override(float(old_pose["x"]), float(old_pose["y"]), float(old_pose["yaw_deg"]))
		keep_override = true
	elif kind == "real_world" and bool(world.get("has_spawn", false)):
		_simulation.set_spawn_override(float(world["x"]), float(world["y"]), float(world["yaw_deg"]))
	else:
		_simulation.clear_spawn_override()
	_load_world(kind, keep_override)

# Back to "no world": stops the sim thread, the terrain streaming and the audio
# (RgSimulation.unload) after freeing the nodes that read them.
func _unload_world() -> void:
	_release_world_nodes()
	_simulation.unload()
	_visuals.clear_vehicle()
	_garage.cleanup() # the setup materialisation of the car just unloaded (R6)
	world_kind = ""
	world_state = "none"
	ready_sim_time = -1.0
	_world_request = {}
	_paused = false
	scripted_controls = {}
	_visuals.show_ground = false
	_overlay.hide_overlay()
	_hud.reset_tick_window()
	print("RG_WORLD unloaded")

func _set_paused(paused: bool) -> void:
	_paused = paused
	_simulation.set_paused(paused)
	if _vehicle_audio != null:
		_vehicle_audio.muted = paused
	if paused:
		_input_map.release_mouse()
	print("RG_WORLD paused=%s" % paused)

func _toggle_pause() -> void:
	if world_state == "running":
		_apply_transition(_shell.pause_toggle())

func _quit_game() -> void:
	_unload_world()
	await _shutdown_audio_and_quit()

# Stops every AudioStreamPlayer3D generator playback (vehicle_audio.gd) and waits
# ~0.1 s for Godot's audio thread to release them before the process quits -
# quitting straight away leaks the AudioStreamGeneratorPlayback objects
# ("ObjectDB instances were leaked at exit", physics_sim 804137e). A `--quit-after`
# exit never reaches here: vehicle_audio.gd's _exit_tree does the same wait.
func _shutdown_audio_and_quit() -> void:
	if _vehicle_audio != null:
		_vehicle_audio.shutdown()
		_vehicle_audio.queue_free()
		_vehicle_audio = null
	await get_tree().create_timer(AUDIO_SETTLE_S, true, false, true).timeout
	get_tree().quit()

func _typing() -> bool:
	var ui_viewport: Viewport = _xr_menu.viewport if _xr_menu != null and _xr_menu.active else get_viewport()
	return _panel_open() or ui_viewport.gui_get_focus_owner() is LineEdit

func _on_escape() -> void:
	match _screen:
		"vehicle_select", "change_car":
			# an open filter panel takes the Esc first
			if not _shell_ui.browser_consume_back():
				_apply_transition(_shell.back())
		"controls":
			# a running capture / calibration takes the Esc first
			if not _shell_ui.controls_consume_back():
				_apply_transition(_shell.back())
		"configurator":
			_shell_ui.garage_consume_back()
		"spawn_picker", "credits", "settings", "loading":
			_apply_transition(_shell.back())
		"pause":
			_apply_transition(_shell.pause_toggle())
		"drive":
			# An open panel or a captured mouse takes the Esc (closes / releases) first.
			if _panels_open_prev or _mouse_captured_prev:
				return
			_toggle_pause()

# The driving view (rg::DriveView): the director's rig for Drive mode.
func set_drive_view(view: String) -> void:
	if view == _drive_view:
		return
	_drive_view = view
	print("RG_WORLD view -> %s" % view)
	if _simulation != null:
		_simulation.set_road_ahead_wanted(view == "cinematic")

# The headlamp switch (auto/on/off, the lights key cycles it) persists like a setting: saved when it changes.
var _saved_light_mode := ""
func _save_light_mode_if_changed() -> void:
	if _shell == null or _input_map == null:
		return
	var mode: String = _input_map.get_light_mode()
	if mode != _saved_light_mode:
		_saved_light_mode = mode
		_shell.set_setting("cabin.light_mode", mode)
		_shell.save_settings()

# Sun shadows and 3D resolution scale (perf, 2026-10-15). The defaults (high, 1.0) are the look before these settings
# existed: four PSSM cascades to 300 m with terrain casting, native resolution. Also re-applied when a world's terrain
# view starts (the terrain's cast flag lives on its instances).
func _apply_graphics_quality() -> void:
	if _shell == null:
		return
	var q := str(_shell.get_setting("graphics.shadow_quality"))
	var sun: DirectionalLight3D = _sky.sun if _sky != null else null
	if sun != null:
		sun.shadow_enabled = q != "off"
		sun.directional_shadow_mode = DirectionalLight3D.SHADOW_PARALLEL_4_SPLITS if q == "high" else DirectionalLight3D.SHADOW_PARALLEL_2_SPLITS
		sun.directional_shadow_max_distance = 150.0 if q == "low" else 300.0
	if _world_view != null and _world_view.has_method("set_cast_shadows"):
		_world_view.set_cast_shadows(q == "high" or q == "medium")
	if not _vr_active:
		var vp := get_viewport()
		var scale := float(_shell.get_setting("graphics.render_scale"))
		vp.scaling_3d_mode = Viewport.SCALING_3D_MODE_BILINEAR
		vp.scaling_3d_scale = clampf(scale, 0.5, 1.0)

# One setting's effect, read back from RgShell (the value there is already
# validated and clamped by rg::Settings). Called at start-up for every setting
# and whenever the settings screen changes one.
func _apply_setting(key: String) -> void:
	var value: Variant = _shell.get_setting(key)
	match key:
		"cabin.light_mode":
			if _input_map != null:
				_input_map.set_light_mode(str(value))
			_saved_light_mode = str(value)
		"graphics.window_mode":
			var mode := DisplayServer.WINDOW_MODE_WINDOWED
			if str(value) == "fullscreen":
				mode = DisplayServer.WINDOW_MODE_FULLSCREEN
			elif str(value) == "exclusive_fullscreen":
				mode = DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN
			DisplayServer.window_set_mode(mode)
		"graphics.renderer":
			# Restart-only (renderer_choice.gd): the override file is what Godot reads at the next start. A test run with
			# --shell-user-dir keeps its settings elsewhere and must not touch the player's override.
			if _user_dir_override == "":
				var err := RendererChoice.store(str(value))
				if err != "":
					push_warning("renderer setting: " + err)
			if not _renderer_reported:
				_renderer_reported = true
				print(RendererChoice.report(str(value)))
		"graphics.vsync":
			if not _vr_active: # the headset's own refresh drives frame pacing
				DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_ENABLED if bool(value) else DisplayServer.VSYNC_DISABLED)
		"graphics.max_fps":
			Engine.max_fps = int(value)
		"graphics.surface_detail":
			# The debug chunk tint (RG_TERRAIN_TINT) is only visible as plain colours.
			SurfaceMaterials.set_detail("off" if OS.get_environment("RG_TERRAIN_TINT") != "" else str(value))
		"graphics.shadow_quality", "graphics.render_scale":
			_apply_graphics_quality()
		"graphics.car_lod":
			preload("res://scripts/car_lods.gd").tier = str(value) # used at the next car load (car_lods.gd)
		"graphics.rain_density":
			if _weather_fx != null:
				_weather_fx.density = float(value)
		"difficulty.mechanical_damage":
			# Live: the running car stops wearing (stored damage stops acting) or starts again; also every later load.
			if _simulation != null and _simulation.has_method("set_damage_enabled"):
				_simulation.set_damage_enabled(bool(value))
		"camera.fov_deg":
			_director.set_base_fov(float(value))
		"camera.body_motion":
			_director.set_body_motion_strength(float(value))
		"audio.master_volume", "audio.engine_volume", "audio.tyre_volume":
			_apply_audio_settings()
		"map_data.store_dir":
			_simulation.set_store_dir_override(str(value))
		"lod.view_distance_m", "lod.terrain_detail", "lod.road_detail", "lod.map_detail":
			var choice: Dictionary = _shell.get_lod_choice()
			_world_view.set_view_lod(choice)
			if _navigation != null:
				_navigation.set_view_lod(choice)

func _apply_audio_settings() -> void:
	if _vehicle_audio == null:
		return
	_vehicle_audio.master_volume = float(_shell.get_setting("audio.master_volume"))
	_vehicle_audio.engine_volume = float(_shell.get_setting("audio.engine_volume"))
	_vehicle_audio.tyre_volume = float(_shell.get_setting("audio.tyre_volume"))
	if _fpv_audio != null:
		_fpv_audio.master_volume = float(_shell.get_setting("audio.master_volume"))

func _log_owner_mark() -> void:
	var point := chassis_session_position()
	var config = JSON.parse_string(FileAccess.get_file_as_string(_world_config_path()))
	var origin: Dictionary = config.get("session_origin_utm", {}) if config is Dictionary else {}
	var wheels: Array = []
	for wheel in range(_simulation.get_vehicle_wheel_count(VEHICLE_NAME)):
		wheels.append({"load_n": _simulation.get_wheel_load_n(VEHICLE_NAME,wheel), "slip_deg": rad_to_deg(_simulation.get_wheel_slip_angle(VEHICLE_NAME,wheel)),
			"slip_ratio": _simulation.get_wheel_slip_ratio(VEHICLE_NAME,wheel), "omega_rad_s": _simulation.get_wheel_omega(VEHICLE_NAME,wheel),
			"steer_deg": rad_to_deg(_simulation.get_wheel_steer_angle(VEHICLE_NAME,wheel)), "fx_n": _simulation.get_wheel_fx(VEHICLE_NAME,wheel),
			"fy_n": _simulation.get_wheel_fy(VEHICLE_NAME,wheel), "surface": _simulation.get_wheel_surface_name(VEHICLE_NAME,wheel)})
	var mark: Dictionary = {"saved_at": Time.get_datetime_string_from_system(), "source":"log_marker", "world":world_kind, "sim_time_s":_simulation.get_sim_time(), "tick":_simulation.get_step_count(), "session_m":[point.x,point.y,point.z], "speed_kmh":_simulation.get_body_speed_mps("chassis")*3.6, "camera":_director.active_name, "wheels":wheels, "build":_simulation.get_build_info()}
	if world_kind == "real_world":
		mark["utm_zone"] = origin.get("zone")
		mark["utm_m"] = [float(origin.get("e0",0))+point.x,float(origin.get("n0",0))+point.y,point.z]
	# The camera pose too: in free cam / drone the owner marks what the camera sees, not the car.
	var cam: Camera3D = _director.active_camera()
	if cam != null:
		var cam_xform := cam.global_transform
		var cam_p: Vector3 = _world_view.godot_to_session(cam_xform.origin)
		var cam_f: Vector3 = _world_view.godot_to_session(cam_xform.origin - cam_xform.basis.z) - cam_p
		mark["camera_session_m"] = [cam_p.x, cam_p.y, cam_p.z]
		mark["camera_heading_deg"] = rad_to_deg(atan2(cam_f.x, cam_f.y)) # compass: 0 = +y (north), 90 = +x (east)
		mark["camera_pitch_deg"] = rad_to_deg(atan2(cam_f.z, Vector2(cam_f.x, cam_f.y).length()))
		if world_kind == "real_world":
			mark["camera_utm_m"] = [float(origin.get("e0",0))+cam_p.x,float(origin.get("n0",0))+cam_p.y,cam_p.z]
	mark["camera_input"] = _input_map.get_camera_diagnostics()
	mark["camera_input"]["enabled"] = _director.camera_input_live
	mark["camera_input"]["seat_settings_open"] = get_tree().get_nodes_in_group("seat_adjustment_open").size() > 0
	mark["camera_input"]["traffic_settings_open"] = get_tree().get_nodes_in_group("traffic_settings_open").size() > 0
	if _vehicle_audio != null:
		mark["audio"] = _vehicle_audio.get_audio_diagnostics()
	mark["powertrain"] = _simulation.get_vehicle_powertrain(VEHICLE_NAME)
	mark["nitrous"] = _simulation.get_vehicle_nitrous(VEHICLE_NAME)
	mark["chassis_session_pose"] = _simulation.get_chassis_session_pose()
	mark["controls"] = {"steer": _simulation.get_control("steer"), "throttle": _simulation.get_control("throttle"),
		"brake": _simulation.get_control("brake"), "handbrake": _simulation.get_control("handbrake"),
		"traction_control": _simulation.get_control("assist.traction_control"), "drive_mode": _simulation.get_control("assist.drive_mode"),
		"nitrous_arm": _simulation.get_control("nitrous_arm")}
	mark["aero"] = _simulation.get_aero_state()
	mark["pretrigger_drive_samples"] = _drive_diag_history.duplicate(true)
	mark["npc_truck"] = _simulation.get_npc_truck_state()
	var traffic: Dictionary = _simulation.get_traffic_state()
	var actors: Array = traffic.get("actors", [])
	# Keep owner pings bounded even with thousands of traffic actors.
	traffic.erase("actors")
	traffic["active"] = actors.size()
	mark["traffic"] = traffic
	# The coordinates for the owner: lat/lon (pasteable into a map) in the real world, session metres in the flat one.
	# Computed before the JSON is encoded, so the mark carries them; the same text goes to the clipboard below.
	var copy_text := "flat x %.1f y %.1f z %.1f" % [point.x, point.y, point.z]
	if world_kind == "real_world":
		var zone := int(origin.get("zone", 32))
		var easting := float(origin.get("e0", 0)) + point.x
		var northing := float(origin.get("n0", 0)) + point.y
		copy_text = "UTM %dN E %.1f N %.1f  z %.1f m" % [zone, easting, northing, point.z]
		if ClassDB.class_exists("RgAddressSearch"):
			var ll: Dictionary = RgAddressSearch.utm_to_latlon(zone, easting, northing)
			if not ll.is_empty():
				mark["latlon"] = [ll["lat"], ll["lon"]]
				copy_text = "%.6f, %.6f" % [ll["lat"], ll["lon"]] + "  " + copy_text
	var encoded := JSON.stringify(mark)
	print("RG_OWNER_MARK " + encoded)
	var mark_path := "user://drive_marks.jsonl"
	var file := FileAccess.open(mark_path,FileAccess.READ_WRITE if FileAccess.file_exists(mark_path) else FileAccess.WRITE)
	if file:
		file.seek_end()
		file.store_line(encoded)
		file.flush()
	DisplayServer.clipboard_set(copy_text)
	print("RG_OWNER_MARK_COPY " + copy_text)
	if _hud != null:
		_hud.show_message("Marked (copied): " + copy_text, 6.0)

# Keep a small rolling window so an owner bookmark captures the loss-of-control
# onset, not just the state after the car has already left the road. Sampling is
# simulation-time based and the retained window is bounded to about five seconds.
func _record_drive_diag_sample() -> void:
	var sim_t: float = _simulation.get_sim_time()
	if sim_t < _drive_diag_last_sample_s:
		_drive_diag_history.clear()
		_drive_diag_last_sample_s = -1.0e9
	if sim_t - _drive_diag_last_sample_s < DRIVE_DIAG_SAMPLE_PERIOD_S:
		return
	_drive_diag_last_sample_s = sim_t
	var wheels: Array = []
	for wheel in range(_simulation.get_vehicle_wheel_count(VEHICLE_NAME)):
		wheels.append({"load_n": _simulation.get_wheel_load_n(VEHICLE_NAME, wheel),
			"slip_deg": rad_to_deg(_simulation.get_wheel_slip_angle(VEHICLE_NAME, wheel)),
			"slip_ratio": _simulation.get_wheel_slip_ratio(VEHICLE_NAME, wheel),
			"omega_rad_s": _simulation.get_wheel_omega(VEHICLE_NAME, wheel),
			"steer_deg": rad_to_deg(_simulation.get_wheel_steer_angle(VEHICLE_NAME, wheel)),
			"fx_n": _simulation.get_wheel_fx(VEHICLE_NAME, wheel),
			"fy_n": _simulation.get_wheel_fy(VEHICLE_NAME, wheel),
			"surface": _simulation.get_wheel_surface_name(VEHICLE_NAME, wheel)})
	_drive_diag_history.append({"sim_time_s": sim_t, "tick": _simulation.get_step_count(),
		"chassis": _simulation.get_chassis_session_pose(),
		"speed_kmh": _simulation.get_body_speed_mps("chassis") * 3.6,
		"controls": {"steer": _simulation.get_control("steer"), "throttle": _simulation.get_control("throttle"),
			"brake": _simulation.get_control("brake"), "handbrake": _simulation.get_control("handbrake"),
			"traction_control": _simulation.get_control("assist.traction_control"),
			"drive_mode": _simulation.get_control("assist.drive_mode"), "nitrous_arm": _simulation.get_control("nitrous_arm")},
		"wheels": wheels, "powertrain": _simulation.get_vehicle_powertrain(VEHICLE_NAME),
		"nitrous": _simulation.get_vehicle_nitrous(VEHICLE_NAME)})
	while not _drive_diag_history.is_empty() and sim_t - float(_drive_diag_history[0].get("sim_time_s", sim_t)) > DRIVE_DIAG_HISTORY_S:
		_drive_diag_history.pop_front()
