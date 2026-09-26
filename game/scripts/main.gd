extends Node3D
# game/scripts/main.gd — R0's whole scene, built procedurally in _ready()
# rather than hand-authored as a .tscn (decision: a script-built scene is
# easier to keep in sync with RgSimulation's evolving method surface across
# R-milestones than a binary/text scene resource with exported NodePaths
# that silently go stale - physics_sim's own demo mixes both, R0 trims to
# just this). game/scenes/main.tscn is therefore a one-line stub (a single
# Node3D with this script attached) - see that file's own comment.
#
# Responsibilities: construct one RgSimulation node and initialize() it
# against the submodule's own car_sedan.json + surfaces.json, start() the
# sim thread, spawn placeholder ground/chassis meshes kept in sync from
# get_body_transform() every frame (no wheel meshes in R0 - deliberate scope
# trim, PLAN.md 12's R0 acceptance doesn't ask for them and RgSimulation
# exposes no per-wheel Transform3D yet, only WheelState scalars for the
# HUD), wire up chase_cam.gd/hud.gd/tach_gauge.gd/input_map.gd exactly the
# way physics_sim's own main.gd wires PsSimulation/PsInputMap to their
# equivalents, and forward input_map.gd's polled channels into
# RgSimulation.set_control() every frame using this repo's own
# core/src/session.cpp::kControlChannelNames list.

const CHASSIS_HALF_EXTENTS := Vector3(2.0, 0.4, 0.15) # matches SessionConfig's default (x fwd, y left, z up)
const GROUND_HALF_EXTENT_M := 1000.0 # matches SessionConfig::ground_half_extent_m's default
const VEHICLE_NAME := "car_sedan"

var _simulation: Node
var _input_map: Node
var _chassis_mesh: MeshInstance3D
var _origin_root: Node3D # everything positioned in the sim's floating-origin frame hangs off this

# --terrain-preview only. Held as a script member (NOT a local var in
# _build_terrain_preview_scene) so the Resource stays alive for as long as
# Main does - a local ShaderMaterial would be freed (and its RenderingServer
# material RID along with it, via Material's own destructor) the instant
# _build_terrain_preview_scene() returns, leaving RgTerrainView holding a
# dangling RID.
var _terrain_material: ShaderMaterial
var _terrain_view: Node
var _terrain_preview_reported: bool = false

func _data_path(relative: String) -> String:
	# game/ is its own Godot project root (res://); external/physics_sim/data
	# lives one level up, OUTSIDE res:// entirely (the submodule is a sibling
	# directory of game/, per this repo's top-level layout - see CLAUDE.md),
	# so ProjectSettings.globalize_path can't reach it via a res:// path -
	# build the absolute filesystem path by hand instead.
	var project_root: String = ProjectSettings.globalize_path("res://")
	return (project_root.path_join("../external/physics_sim/data").path_join(relative)).simplify_path()

func _world_config_path() -> String:
	# racing_game's OWN data/world/world_config.json (PLAN.md R2.0/R2.1) -
	# one level up from game/, same "submodule/sibling dir is outside res://"
	# reasoning as _data_path above, but this file lives directly in this
	# repo (not external/physics_sim).
	var project_root: String = ProjectSettings.globalize_path("res://")
	return (project_root.path_join("../data/world/world_config.json")).simplify_path()

func _ready() -> void:
	if "--terrain-preview" in OS.get_cmdline_user_args():
		_build_terrain_preview_scene()
	else:
		_build_scene()

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
	camera.far = 25000.0
	camera.position = Vector3(0.0, 600.0, 0.0)
	camera.rotation_degrees = Vector3(-25.0, 0.0, 0.0)
	camera.set_script(load("res://scripts/fly_cam.gd"))
	add_child(camera)

func _build_scene() -> void:
	# --- simulation node ---
	_simulation = ClassDB.instantiate("RgSimulation")
	_simulation.name = "Simulation"
	add_child(_simulation)

	var vehicle_json: String = _data_path("vehicles/car_sedan.json")
	var surface_table_json: String = _data_path("surfaces/surfaces.json")
	var ok: bool = _simulation.initialize(vehicle_json, surface_table_json)
	if not ok:
		push_error("RgSimulation.initialize failed: %s" % _simulation.get_last_error())
		return
	_simulation.start()

	# --- floating-origin root: ground + chassis meshes hang off this so a
	# rebase (see chase_cam.gd) moves them all consistently with the camera ---
	_origin_root = Node3D.new()
	_origin_root.name = "OriginRoot"
	add_child(_origin_root)

	# --- lighting (a single directional light is enough for R0) ---
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

	# --- ground mesh (visual only - collision lives entirely in ps_core /
	# rg_core, this box is just placed to match SessionConfig's own static
	# ground body so it LOOKS like what the car is driving on) ---
	var ground := MeshInstance3D.new()
	ground.name = "Ground"
	var ground_mesh := BoxMesh.new()
	ground_mesh.size = Vector3(GROUND_HALF_EXTENT_M * 2.0, 1.0, GROUND_HALF_EXTENT_M * 2.0)
	ground.mesh = ground_mesh
	ground.position = Vector3(0.0, -0.5, 0.0)
	var ground_mat := StandardMaterial3D.new()
	ground_mat.albedo_color = Color(0.25, 0.32, 0.22)
	ground.material_override = ground_mat
	_origin_root.add_child(ground)

	# --- chassis mesh (placeholder box, sized from SessionConfig's default
	# chassis_half_extents - godot_ext/src/frame_convert.h's iso_to_godot_
	# transform converts ps::Pose's ISO 8855 x-fwd/y-left/z-up to Godot's
	# x-right/y-up/-z-fwd, so the mesh's own local axes need no extra
	# rotation, only the BoxMesh's Godot-space size swapped accordingly:
	# ISO x (length) -> Godot Z depth, ISO y (width) -> Godot X width,
	# ISO z (height) -> Godot Y height) ---
	_chassis_mesh = MeshInstance3D.new()
	_chassis_mesh.name = "Chassis"
	var chassis_box := BoxMesh.new()
	chassis_box.size = Vector3(CHASSIS_HALF_EXTENTS.y * 2.0, CHASSIS_HALF_EXTENTS.z * 2.0, CHASSIS_HALF_EXTENTS.x * 2.0)
	_chassis_mesh.mesh = chassis_box
	var chassis_mat := StandardMaterial3D.new()
	chassis_mat.albedo_color = Color(0.75, 0.1, 0.1)
	_chassis_mesh.material_override = chassis_mat
	_origin_root.add_child(_chassis_mesh)

	# --- camera ---
	var camera := Camera3D.new()
	camera.name = "ChaseCam"
	camera.current = true
	camera.fov = 70.0
	# NodePaths below are literal "../<sibling>" rather than
	# self.get_path_to(_simulation): every one of these nodes ends up a
	# direct child of Main (this node), same as _simulation/_input_map, so
	# the relative path is known up front from the tree layout this function
	# itself builds. This also sidesteps a real ordering bug found during
	# R0's own smoke-test debugging: get_path_to() needs the CALLING node
	# already inside the tree (so it has to run after add_child()), but
	# add_child() calls the child's _ready() SYNCHRONOUSLY when the parent
	# is already inside the tree (true here - Main is the running scene
	# root) - so an export like simulation_path set AFTER add_child() is set
	# too late, after _ready() already read the not-yet-assigned (empty)
	# value. Setting a literal NodePath before add_child() avoids both
	# failure modes at once.
	var chase_cam_script := load("res://scripts/chase_cam.gd")
	camera.set_script(chase_cam_script)
	camera.chassis_body_name = "chassis"
	camera.simulation_path = NodePath("../Simulation")
	add_child(camera)

	# --- input ---
	_input_map = load("res://scripts/input_map.gd").new()
	_input_map.name = "InputMap"
	add_child(_input_map)

	# --- HUD (debug text) ---
	var hud := CanvasLayer.new()
	hud.name = "Hud"
	var hud_script := load("res://scripts/hud.gd")
	hud.set_script(hud_script)
	var readout := Label.new()
	readout.name = "Readout"
	readout.position = Vector2(16, 16)
	readout.add_theme_font_size_override("font_size", 14)
	hud.add_child(readout)
	hud.simulation_path = NodePath("../Simulation")
	hud.input_map_path = NodePath("../InputMap")
	hud.vehicle_name = VEHICLE_NAME
	add_child(hud)

	# --- tach/speed gauge ---
	var gauge := Control.new()
	gauge.name = "TachGauge"
	var gauge_script := load("res://scripts/tach_gauge.gd")
	gauge.set_script(gauge_script)
	gauge.simulation_path = NodePath("../Simulation")
	gauge.input_map_path = NodePath("../InputMap")
	gauge.vehicle_name = VEHICLE_NAME
	add_child(gauge)

func _process(_delta: float) -> void:
	if _terrain_view != null and not _terrain_preview_reported and bool(_terrain_view.is_fully_uploaded()):
		_terrain_preview_reported = true
		print("terrain preview loaded: chunks=%d vertices=%d upload_ms=%.2f" % [
			_terrain_view.get_chunk_count(), _terrain_view.get_total_vertex_count(),
			_terrain_view.get_total_upload_time_ms()
		])

	if _simulation == null or not bool(_simulation.is_running()):
		return

	_input_map.poll()
	_simulation.set_control("steer", _input_map.get_steer())
	_simulation.set_control("throttle", _input_map.get_throttle())
	_simulation.set_control("brake", _input_map.get_brake())
	_simulation.set_control("handbrake", _input_map.get_handbrake())
	_simulation.set_control("clutch", _input_map.get_clutch())
	_simulation.set_control("ignition", 1.0 if _input_map.get_ignition() else 0.0)
	_simulation.set_control("starter", 1.0 if _input_map.get_starter() else 0.0)
	_simulation.set_control("assist.auto_shift", 1.0 if _input_map.get_auto_shift() else 0.0)
	# shift_up_count/shift_down_count are edge counters (RgSimulation's
	# session.cpp doc comment) - read-and-reset via consume_*, forwarded as
	# an absolute running count added onto whatever World::set_control
	# already holds, since the control channel itself is just a plain
	# double, not itself an edge-counting primitive on the sim side.
	var up: int = _input_map.consume_shift_up_count()
	if up > 0:
		_simulation.set_control("shift_up_count", _simulation.get_control("shift_up_count") + up)
	var down: int = _input_map.consume_shift_down_count()
	if down > 0:
		_simulation.set_control("shift_down_count", _simulation.get_control("shift_down_count") + down)

	if _chassis_mesh != null:
		_chassis_mesh.transform = _simulation.get_body_transform("chassis")
