extends Node3D
# game/scripts/main.gd — the one game scene (R2.2 R9), built procedurally in
# _ready() rather than hand-authored as a .tscn (a script-built scene stays in
# sync with RgSimulation's evolving method surface; game/scenes/main.tscn is
# a one-node stub with this script attached).
#
# Command line (user args after `--`):
#   (none)                  flat test scene, Drive mode
#   --drive                 real world (RG_G2M_HOME store), Drive mode
#   --free-cam              start in FreeCam mode instead of Drive
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
# The flags only pick the STARTING world and mode: both change at runtime.
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
# World load flow: flat = RgSimulation.initialize() + start() at once. Real
# world = initialize_terrain() (start-up runs on a worker thread) -> poll
# get_init_status() each frame, the loading overlay shows its numbers ->
# "ready": initial controls, start() -> first tick: RgTerrainView.
# initialize_shared(sim), load_preview at the chassis -> "running". F8 at any
# point (also while loading - R9's cancel-then-join returns promptly) releases
# the terrain view and loads the other world; the player mode is kept.

const VEHICLE_NAME := "car_hyper"
const WORLD_LABEL := {"flat": "flat test scene", "real_world": "real world"}

# --- unified scene state (read by drive_smoke.gd / drive_tour.gd) ---
var world_kind: String = ""      # world being loaded or loaded: "flat" / "real_world"
var world_state: String = "none" # "loading" -> "starting" -> "running"; or "failed"
var fetch_delay_ms: int = 0
# Non-empty: overrides the driving input group (channel -> value), used by the
# scripted smoke and screenshot tour instead of an autopilot.
var scripted_controls: Dictionary = {}
var ready_sim_time: float = -1.0 # sim time when the current world became "running"

var _steering = preload("res://scripts/adaptive_steering.gd").new()
var _simulation: Node
var _input_map: Node
var _cockpit_view: bool = false
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
	var project_root: String = ProjectSettings.globalize_path("res://")
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
	var surface_table_json: String = _data_path("surfaces/surfaces.json")

	var ok: bool = sim.initialize(vehicle_json, surface_table_json)
	if not ok:
		push_error("bindings test: first initialize() failed: %s" % sim.get_last_error())
		get_tree().quit(1)
		return
	sim.start()

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
	set_process_priority(-2000)
	var start_world := "real_world" if "--drive" in user_args else "flat"
	var start_mode := "free_cam" if "--free-cam" in user_args else "drive"
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

	# --- input (devices -> values only) ---
	_input_map = load("res://scripts/input_map.gd").new()
	_input_map.name = "InputMap"
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
	_visuals.model_absolute_path = _data_path("models/%s/%s.glb" % [VEHICLE_NAME, VEHICLE_NAME])
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
	var free := Node3D.new()
	free.name = "FreeRig"
	free.set_script(load("res://scripts/free_rig.gd"))
	add_child(free)
	_director.add_rig("free", free)

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
	var gauge := Control.new()
	gauge.name = "TachGauge"
	gauge.set_script(load("res://scripts/tach_gauge.gd"))
	gauge.simulation_path = NodePath("../Simulation")
	gauge.input_map_path = NodePath("../InputMap")
	gauge.vehicle_name = VEHICLE_NAME
	add_child(gauge)

	# --- loading overlay ---
	_overlay = CanvasLayer.new()
	_overlay.name = "LoadingOverlay"
	_overlay.set_script(load("res://scripts/loading_overlay.gd"))
	add_child(_overlay)

	_apply_mode_state()
	_load_world(start_world)

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

func get_body_visuals() -> Node:
	return _visuals

func chassis_session_position() -> Vector3:
	return _world_view.godot_to_session(_simulation.get_body_transform("chassis").origin)

# --- world load / switch ---
func switch_world() -> void:
	var other: String = str(_simulation.get_mode_state().get("other_world", "flat"))
	print("RG_WORLD switch %s -> %s (was %s)" % [world_kind, other, world_state])
	_load_world(other)

var _vehicle_audio: Node3D

func _load_world(kind: String) -> void:
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
	world_kind = kind
	world_state = "loading"
	ready_sim_time = -1.0
	_spawn_reported = false
	_load_started_ms = Time.get_ticks_msec()
	_hud.reset_tick_window()
	_visuals.show_ground = kind == "flat"
	var vehicle_json: String = _data_path("vehicles/%s.json" % VEHICLE_NAME)
	_steering.configure_vehicle(vehicle_json)
	_input_map.configure_vehicle(vehicle_json)
	var surface_table_json: String = _data_path("surfaces/surfaces.json")
	if kind == "flat":
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
	else:
		_simulation.initialize_terrain(_world_config_path(), vehicle_json, surface_table_json)
		_overlay.show_loading(WORLD_LABEL[kind], {}, 0.0)
		print("RG_WORLD loading world=real_world fetch_delay_ms=%d" % fetch_delay_ms)

func _load_failed(message: String) -> void:
	world_state = "failed"
	_overlay.show_error(WORLD_LABEL.get(world_kind, world_kind), message)
	push_error("%s did not load: %s" % [WORLD_LABEL.get(world_kind, world_kind), message])

# Controls a fresh Session needs before its first tick: every channel starts
# at 0 in rg::Session, so ignition must be on BEFORE start() or the car
# spawns with the engine off. auto_clutch/auto_blip are always on (no pedal
# work needed to pull away); ignition/auto_shift follow input_map.gd's
# toggles (ignition starts on; manual cars start with auto-shift off).
func _apply_start_controls() -> void:
	_simulation.set_control("ignition", 1.0 if _input_map.get_ignition() else 0.0)
	_simulation.set_control("assist.auto_clutch", 1.0)
	_simulation.set_control("assist.auto_blip", 1.0)
	_simulation.set_control("assist.auto_shift", 1.0 if _input_map.get_auto_shift() else 0.0)

func _poll_loading() -> void:
	var st: Dictionary = _simulation.get_init_status()
	var elapsed := (Time.get_ticks_msec() - _load_started_ms) / 1000.0
	match str(st.get("state", "")):
		"loading":
			_overlay.show_loading(WORLD_LABEL[world_kind], st, elapsed)
		"ready":
			_overlay.show_loading(WORLD_LABEL[world_kind], st, elapsed)
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
	if what == NOTIFICATION_WM_CLOSE_REQUEST and world_kind == "real_world" and world_state == "running":
		_save_drive_location()

func _save_drive_location() -> void:
	var p := chassis_session_position()
	var config = JSON.parse_string(FileAccess.get_file_as_string(_world_config_path()))
	var origin: Dictionary = config.get("session_origin_utm", {}) if config is Dictionary else {}
	var file := FileAccess.open("user://last_drive.json", FileAccess.WRITE)
	if file:
		file.store_string(JSON.stringify({"saved_at": Time.get_datetime_string_from_system(),
			"session_m": [p.x, p.y, p.z], "world": world_kind,
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
	if rig_name == "chase" and _cockpit_view:
		rig_name = "cockpit"
	if _vr_active:
		rig_name = "xr_free" if rig_name == "free" else "xr_cockpit"
	_director.set_active(rig_name)
	_director.camera_input_live = bool(ms.get("camera_inputs_live", true)) and not (get_tree().get_nodes_in_group("seat_adjustment_open").size() > 0)
	return ms

func _forward_driving(live: bool, delta: float) -> void:
	# Edge counts are consumed every frame, so presses made while the car is
	# unattended never arrive later as a burst of shifts.
	var up: int = _input_map.consume_shift_up_count()
	var down: int = _input_map.consume_shift_down_count()
	if not live or not bool(_simulation.is_running()):
		_steering.reset()
		return
	if get_tree().get_nodes_in_group("seat_adjustment_open").size() > 0:
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
	if world_state == "running" and Input.is_action_just_pressed("rg_npc_truck"):
		_simulation.request_npc_truck(not Input.is_key_pressed(KEY_SHIFT), 70.0)
	var camera_presses: int = _input_map.consume_cycle_camera()
	if camera_presses % 2 == 1 and not _vr_active and _simulation.get_player_mode() == "drive":
		_cockpit_view = not _cockpit_view
	for _i in range(_input_map.consume_cycle_mode()):
		print("RG_WORLD mode -> %s" % _simulation.cycle_player_mode())
	if _input_map.consume_switch_world() > 0:
		switch_world()
	if _input_map.consume_reset_car() > 0 and world_state == "running":
		_simulation.reset_vehicle_to_spawn()
	if _input_map.consume_flip_upright() > 0 and world_state == "running" and _simulation.get_player_mode() == "drive":
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
	var ms := _apply_mode_state()
	_forward_driving(bool(ms.get("driving_inputs_live", false)), delta)

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
	mark["aero"] = _simulation.get_aero_state()
	mark["npc_truck"] = _simulation.get_npc_truck_state()
	mark["traffic"] = _simulation.get_traffic_state()
	var encoded := JSON.stringify(mark)
	print("RG_OWNER_MARK " + encoded)
	var mark_path := "user://drive_marks.jsonl"
	var file := FileAccess.open(mark_path,FileAccess.READ_WRITE if FileAccess.file_exists(mark_path) else FileAccess.WRITE)
	if file:
		file.seek_end()
		file.store_line(encoded)
		file.flush()
