extends Node3D
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
# itself with the HUD. Esc/P pause, Esc backs out of every other screen.
#
# World load flow: flat = RgSimulation.initialize() + start() at once. Real
# world = initialize_terrain() (start-up runs on a worker thread) -> poll
# get_init_status() each frame, the loading overlay shows its numbers ->
# "ready": initial controls, start() -> first tick: RgTerrainView.
# initialize_shared(sim), load_preview at the chassis -> "running". F8 at any
# point (also while loading - R9's cancel-then-join returns promptly) releases
# the terrain view and loads the other world; the player mode is kept.

# The simulation's name of the car being driven (the vehicle file's "name"); set by
# _apply_vehicle from the garage's drive selection before every world load. "car_hyper"
# until the first load (the --bindings-test path and the catalog default).
var VEHICLE_NAME := "car_hyper"
const WORLD_LABEL := {"flat": "flat test scene", "real_world": "real world"}

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
var _walker_visual: Node3D
var _refusals_seen: int = 0 # get_mode_state()["get_out_refusals"] already reported to the HUD
var _enter_refused_seen: int = 0 # ... ["walker_enter_refused"]
var _mode_seen: String = "drive"
var _seat_ui: CanvasLayer
var _director: Node
var _hud: Node
var _overlay: Node
var _visuals: Node
var _roads: Node3D
var _buildings: Node3D
var _world_view: Node # RgTerrainView shared with the real-world Session
var _load_started_ms: int = 0
var _last_report_ms: int = 0
const FRAME_SPIKE_S := 0.05
var _spawn_reported: bool = false

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
	# racing_game's OWN data/world/world_config.json (PLAN.md R2.0/R2.1).
	# RG_WORLD_CONFIG (env) picks another world, e.g.
	# data/world/world_config_rhein_main_hessen.json (relative to the repo root).
	var project_root: String = ProjectSettings.globalize_path("res://")
	var override := OS.get_environment("RG_WORLD_CONFIG")
	if override != "":
		return override if override.is_absolute_path() else (project_root.path_join("..").path_join(override)).simplify_path()
	return (project_root.path_join("../data/world/world_config.json")).simplify_path()

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
	preload("res://scripts/ui_scale.gd").apply(get_tree())
	var user_args := OS.get_cmdline_user_args()
	if "--bindings-test" in user_args:
		_run_bindings_test()
	elif "--terrain-preview" in user_args:
		_build_terrain_preview_scene()
	else:
		_build_scene(user_args)

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

	# game/shaders/terrain.gdshader: vertex colour (the hypsometric ramp
	# RgTerrainView uploads as ARRAY_COLOR) + Lambert - see set_material's
	# own doc comment for why this call is required for the colours to be
	# visible at all (Godot's default material ignores vertex COLOR).
	_terrain_material = ShaderMaterial.new()
	_terrain_material.shader = load("res://shaders/terrain.gdshader")
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
	else:
		# R2.2 R8: the fly camera drives the render LOD focus (_process). Not
		# in --screenshots mode, so the tour keeps its fixed spawn chunk set
		# and its 5 poses unchanged.
		_terrain_focus_camera = camera

func _add_sun_and_sky() -> void:
	var sun := DirectionalLight3D.new()
	sun.name = "Sun"
	sun.rotation_degrees = Vector3(-55.0, -35.0, 0.0)
	sun.light_energy = 1.1
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 300.0
	add_child(sun)
	var env_node := WorldEnvironment.new()
	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	env.sky = Sky.new()
	env.sky.sky_material = ProceduralSkyMaterial.new()
	env_node.environment = env
	add_child(env_node)

func _build_scene(user_args: PackedStringArray) -> void:
	_teleport_dialog = CanvasLayer.new()
	_teleport_dialog.set_script(load("res://scripts/address_teleport.gd"))
	_teleport_dialog.main = self
	add_child(_teleport_dialog)
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
	_simulation.name = "Simulation"
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
		if not _simulation.initialize(vehicle_json, surface_table_json):
			_load_failed(str(_simulation.get_last_error()))
			return
		_apply_start_controls()
		_simulation.start()
		world_state = "running"
		ready_sim_time = 0.0
		_overlay.hide_overlay()
		_visuals.on_session_ready()
		print("RG_WORLD ready world=flat")
		_world_running()
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
	var elapsed := (Time.get_ticks_msec() - _load_started_ms) / 1000.0
	match str(st.get("state", "")):
		"loading":
			_overlay.show_loading(WORLD_LABEL[world_kind], st, elapsed, _loading_footer(), str(_world_request.get("label", "")))
		"ready":
			_overlay.show_loading(WORLD_LABEL[world_kind], st, elapsed, _loading_footer(), str(_world_request.get("label", "")))
			_apply_start_controls()
			_simulation.start()
			world_state = "starting"
		"error":
			_load_failed(str(st.get("message", "")))

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

func _save_drive_location() -> void:
	var p := chassis_session_position()
	var config = JSON.parse_string(FileAccess.get_file_as_string(_world_config_path()))
	var origin: Dictionary = config.get("session_origin_utm", {}) if config is Dictionary else {}
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
		int(ss.get("prefetch_stale_inputs", 0))] + " npc=%d traffic_loading=%s" % [_simulation.get_traffic_state().get("actors", []).size(), str(_simulation.get_traffic_state().get("loading", false))]

# rg_core decides the active rig and which input groups are live.
func _apply_mode_state() -> Dictionary:
	var ms: Dictionary = _simulation.get_mode_state()
	var rig_name := str(ms.get("camera_rig", "chase"))
	if rig_name == "chase":
		rig_name = _drive_view # chase / bumper / cockpit / orbit / cinematic
	if _vr_active:
		# VR keeps its own free rig for the drone view too (no XR drone rig yet).
		# (No XR walker rig yet either: on foot the player keeps the tracked free flight.)
		rig_name = "xr_free" if rig_name == "free" or rig_name == "drone" or rig_name == "walker" else "xr_cockpit"
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
	return get_tree().get_nodes_in_group("seat_adjustment_open").size() > 0 or get_tree().get_nodes_in_group("traffic_settings_open").size() > 0 or get_tree().get_nodes_in_group("address_teleport_open").size() > 0

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

func _forward_driving(live: bool, delta: float) -> void:
	# Edge counts are consumed every frame, so presses made while the car is
	# unattended never arrive later as a burst of shifts.
	var up: int = _input_map.consume_shift_up_count()
	var down: int = _input_map.consume_shift_down_count()
	var nitrous_toggles: int = _input_map.consume_nitrous_toggle()
	if not live or not bool(_simulation.is_running()):
		_steering.reset()
		return
	if get_tree().get_nodes_in_group("seat_adjustment_open").size() > 0 or get_tree().get_nodes_in_group("address_teleport_open").size() > 0:
		_simulation.set_control("steer", 0.0)
		_simulation.set_control("throttle", 0.0)
		_simulation.set_control("brake", 1.0)
		_simulation.set_control("starter", 0.0)
		return
	if not scripted_controls.is_empty():
		for channel in scripted_controls:
			_simulation.set_control(channel, float(scripted_controls[channel]))
		return
	_simulation.set_control("steer", _steering.translate(_input_map.get_steer(), _input_map.steering_uses_keyboard(), _input_map.steering_uses_wheel(), _simulation.get_steering_kinematics(), delta))
	_simulation.set_control("throttle", _input_map.get_throttle())
	_simulation.set_control("brake", _input_map.get_brake())
	_simulation.set_control("handbrake", _input_map.get_handbrake())
	_simulation.set_control("clutch", _input_map.get_clutch())
	_simulation.set_control("ignition", 1.0 if _input_map.get_ignition() else 0.0)
	_simulation.set_control("starter", 1.0 if _input_map.get_starter() else 0.0)
	_simulation.set_control("assist.auto_shift", 1.0 if _input_map.get_auto_shift() else 0.0)
	# Nitrous arm switch (gamepad Y, owner 2026-10-05; physics_sim 804137e): each
	# rising edge flips the plain 0/1 "nitrous_arm" channel. Cars without a nitrous
	# kit never declare it, so for them it is inert.
	for _i in range(nitrous_toggles):
		_simulation.set_control("nitrous_arm", 1.0 - _simulation.get_control("nitrous_arm"))
	# shift_up_count/shift_down_count are running edge counters on the sim
	# side (session.cpp's kControlChannelNames): add this frame's edges.
	if up > 0:
		_simulation.set_control("shift_up_count", _simulation.get_control("shift_up_count") + up)
	if down > 0:
		_simulation.set_control("shift_down_count", _simulation.get_control("shift_down_count") + down)

func _process(delta: float) -> void:
	# Main-thread frame hitches, to line up with RG_TICK_SPIKE (sim thread).
	if world_state == "running" and delta > FRAME_SPIKE_S:
		print("RG_FRAME_SPIKE delta_ms=%.1f ticks=%d" % [delta * 1000.0, int(_simulation.get_step_count())])
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
			_report()

	if world_state == "running" and _vehicle_audio == null:
		_vehicle_audio = Node3D.new()
		_vehicle_audio.name = "VehicleAudio"
		_vehicle_audio.set_script(load("res://scripts/vehicle_audio.gd"))
		_vehicle_audio.simulation = _simulation
		_vehicle_audio.director = _director
		_vehicle_audio.vehicle_name = VEHICLE_NAME
		add_child(_vehicle_audio)
		_apply_audio_settings()
	if _vehicle_audio != null:
		_vehicle_audio.muted = _paused
	var ms := _apply_mode_state()
	_forward_driving(bool(ms.get("driving_inputs_live", false)) and in_drive, delta)
	_forward_walking(bool(ms.get("walking_inputs_live", false)) and in_drive)
	_panels_open_prev = _panel_open()
	_mouse_captured_prev = _input_map.is_mouse_captured()

var _vr_active := false

func _try_start_vr() -> void:
	var xr := XRServer.find_interface("OpenXR")
	if xr == null or not xr.is_initialized():
		print("RG_VR inactive: OpenXR not initialized; using desktop cameras")
		return
	DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
	get_viewport().use_xr = true
	for rig_name in ["xr_cockpit", "xr_free"]:
		var rig := XROrigin3D.new()
		rig.name = rig_name
		rig.set_script(load("res://scripts/xr_rig.gd"))
		rig.body_visuals = _visuals
		rig.free_flight = rig_name == "xr_free"
		add_child(rig)
		_director.add_rig(rig_name, rig)
	_vr_active = true
	print("RG_VR active: tracked cockpit, F9 recenter; FreeCam uses tracked free flight")

# Owner bookmarks survive normal log rotation in user://drive_marks.jsonl.
func _input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_MIDDLE and world_state == "running":
		_log_owner_mark()
	if _shell != null and event is InputEventKey and event.pressed and not event.echo:
		if event.keycode == KEY_ESCAPE:
			_on_escape()
		elif event.keycode == KEY_P and (_screen == "drive" or _screen == "pause") and not _typing():
			_toggle_pause()
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
	})
	_visuals.set_vehicle(VEHICLE_NAME, str(sel["model_path"]), str(sel["paint"]), str(sel["rim"]))
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

# Removes the garage scene and every node it created (the acceptance test counts
# the tree), ends any edit session and removes the scratch files.
func _close_garage() -> void:
	_garage.discard()
	if _garage_scene != null:
		remove_child(_garage_scene)
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

func _on_garage_save() -> void:
	var saved: Dictionary = _garage.save()
	print("RG_GARAGE save ok=%s %s" % [saved.get("ok", false), saved.get("error", "")])
	_shell_ui.refresh_configurator()

# Drive: valid unsaved changes are saved first; an invalid working copy cannot be driven.
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
	_shell_ui.garage_drive_requested.connect(_on_garage_drive)
	add_child(_shell_ui)
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
	_input_map.suspended = screen_name == "controls" # the controls screen reads devices itself
	_shell_ui.show_screen(screen_name)
	_set_world_ui_visible(screen_name == "drive" or screen_name == "pause")
	if screen_name != "loading":
		_overlay.hide_overlay()
	if screen_name != "drive" and screen_name != "loading":
		_input_map.release_mouse()

func _set_world_ui_visible(visible_now: bool) -> void:
	_hud.visible = visible_now
	_gauge.visible = visible_now

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
	return _panel_open() or get_viewport().gui_get_focus_owner() is LineEdit

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
		"spawn_picker", "credits", "settings", "loading", "configurator":
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

# One setting's effect, read back from RgShell (the value there is already
# validated and clamped by rg::Settings). Called at start-up for every setting
# and whenever the settings screen changes one.
func _apply_setting(key: String) -> void:
	var value: Variant = _shell.get_setting(key)
	match key:
		"graphics.window_mode":
			var mode := DisplayServer.WINDOW_MODE_WINDOWED
			if str(value) == "fullscreen":
				mode = DisplayServer.WINDOW_MODE_FULLSCREEN
			elif str(value) == "exclusive_fullscreen":
				mode = DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN
			DisplayServer.window_set_mode(mode)
		"graphics.vsync":
			if not _vr_active: # the headset's own refresh drives frame pacing
				DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_ENABLED if bool(value) else DisplayServer.VSYNC_DISABLED)
		"graphics.max_fps":
			Engine.max_fps = int(value)
		"camera.fov_deg":
			_director.set_base_fov(float(value))
		"audio.master_volume", "audio.engine_volume", "audio.tyre_volume":
			_apply_audio_settings()
		"map_data.store_dir":
			_simulation.set_store_dir_override(str(value))

func _apply_audio_settings() -> void:
	if _vehicle_audio == null:
		return
	_vehicle_audio.master_volume = float(_shell.get_setting("audio.master_volume"))
	_vehicle_audio.engine_volume = float(_shell.get_setting("audio.engine_volume"))
	_vehicle_audio.tyre_volume = float(_shell.get_setting("audio.tyre_volume"))

func _log_owner_mark() -> void:
	var point := chassis_session_position()
	var config = JSON.parse_string(FileAccess.get_file_as_string(_world_config_path()))
	var origin: Dictionary = config.get("session_origin_utm", {}) if config is Dictionary else {}
	var wheels: Array = []
	for wheel in range(_simulation.get_vehicle_wheel_count(VEHICLE_NAME)):
		wheels.append({"load_n": _simulation.get_wheel_load_n(VEHICLE_NAME,wheel), "slip_deg": rad_to_deg(_simulation.get_wheel_slip_angle(VEHICLE_NAME,wheel)), "surface": _simulation.get_wheel_surface_name(VEHICLE_NAME,wheel)})
	var mark: Dictionary = {"saved_at": Time.get_datetime_string_from_system(), "source":"middle_mouse", "world":world_kind, "sim_time_s":_simulation.get_sim_time(), "tick":_simulation.get_step_count(), "session_m":[point.x,point.y,point.z], "speed_kmh":_simulation.get_body_speed_mps("chassis")*3.6, "camera":_director.active_name, "wheels":wheels, "build":_simulation.get_build_info()}
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
	mark["aero"] = _simulation.get_aero_state()
	mark["npc_truck"] = _simulation.get_npc_truck_state()
	var traffic: Dictionary = _simulation.get_traffic_state()
	var actors: Array = traffic.get("actors", [])
	# Keep owner pings bounded even with thousands of traffic actors.
	traffic.erase("actors")
	traffic["active"] = actors.size()
	mark["traffic"] = traffic
	var encoded := JSON.stringify(mark)
	print("RG_OWNER_MARK " + encoded)
	var mark_path := "user://drive_marks.jsonl"
	var file := FileAccess.open(mark_path,FileAccess.READ_WRITE if FileAccess.file_exists(mark_path) else FileAccess.WRITE)
	if file:
		file.seek_end()
		file.store_line(encoded)
		file.flush()
