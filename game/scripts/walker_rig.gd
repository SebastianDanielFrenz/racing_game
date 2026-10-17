extends Node3D
# game/scripts/walker_rig.gd — the OnFoot mode camera rig (rg::CameraRig::Walker).
# Same rig contract as chase_rig.gd / free_rig.gd / drone_rig.gd: a ROOT Node3D
# (position + yaw) with a Camera3D child (pitch); camera_director.gd moves the
# ROOT for floating-origin rebases (shift_origin) and calls update_rig() only
# while this rig is active.
#
# Display + look only. The walking itself (speed, gravity, steps, collisions)
# lives in rg_core (rg/walker.h, rg::Session); this rig reads the walker's
# position from RgSimulation.get_walker_state() and owns the LOOK direction
# (yaw/pitch), which main.gd forwards with the walking input
# (get_look_forward()).
#
# Third person (default): the camera orbits the walker's head at
# `distance_m` (mouse wheel / PageUp,PageDown), clamped above the terrain.
# First person: the camera sits at eye height, toggled with Tab
# (main.gd -> toggle_first_person()). Look input comes from input_map.gd's
# CAMERA group: right stick / arrow keys (rate) and the captured mouse
# (right mouse button), exactly like free_rig.gd.
#
# Display only (docs/unreal_port.md leak #10): the look angles, the zoom law, the
# pivot smoothing, the third-person placement and the aim are rg::walker_cam_* in
# core/include/rg/camera_math.h (RgCameraMath.walker_cam_*). This script keeps the
# state Dictionary, forwards input and the walker state, probes the ground and
# places the Camera3D.

@export var look_rate_rad_s: float = 2.2
@export var mouse_sensitivity: float = 0.0025
@export var third_person_distance_m: float = 3.2
@export var min_distance_m: float = 1.2
@export var max_distance_m: float = 9.0
@export var head_height_m: float = 1.55   # third-person pivot above the feet
@export var eye_height_m: float = 1.62    # first-person camera above the feet
@export var terrain_clearance_m: float = 0.3
@export var follow_rate: float = 28.0     # exponential smoothing of the pivot (1/s)
@export var snap_m: float = 12.0          # a jump this large (relocation, spawn) snaps instead of smoothing
@export var third_pitch_min: float = -1.1
@export var third_pitch_max: float = 1.2
@export var first_pitch_limit: float = 1.5

var camera: Camera3D
var first_person: bool = false
var _state: Dictionary = {} # RgCameraMath.walker_cam_* state

func _params() -> Dictionary:
	return {
		"look_rate_rad_s": look_rate_rad_s,
		"mouse_sensitivity": mouse_sensitivity,
		"third_person_distance_m": third_person_distance_m,
		"min_distance_m": min_distance_m,
		"max_distance_m": max_distance_m,
		"head_height_m": head_height_m,
		"eye_height_m": eye_height_m,
		"terrain_clearance_m": terrain_clearance_m,
		"follow_rate": follow_rate,
		"snap_m": snap_m,
		"third_pitch_min": third_pitch_min,
		"third_pitch_max": third_pitch_max,
		"first_pitch_limit": first_pitch_limit,
	}

func _ready() -> void:
	_state = RgCameraMath.walker_cam_initial_state(_params())
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.fov = 70.0
	camera.near = 0.05
	camera.far = 43000.0
	add_child(camera)

# Called on every switch to this rig; the walker may not exist yet (it spawns a
# tick after the mode change), so the pose is taken from the first walker state.
func activate(from: Transform3D) -> void:
	position = from.origin
	_state = RgCameraMath.walker_cam_activate(_state, -from.basis.z, _params())
	_apply_rotation(_state["yaw_rad"], _state["pitch_rad"])

func shift_origin(delta: Vector3) -> void:
	position += delta
	_state = RgCameraMath.walker_cam_shift_origin(_state, delta)

func toggle_first_person() -> void:
	first_person = not first_person

# Horizontal forward direction of the look heading in Godot space, for the
# walking input (the sim turns it into an ISO yaw).
func get_look_forward() -> Vector3:
	return RgCameraMath.walker_cam_look_forward(_state)

func _apply_rotation(yaw: float, pitch: float) -> void:
	rotation = Vector3(0.0, yaw, 0.0)
	if camera != null:
		camera.rotation = Vector3(pitch, 0.0, 0.0)

func update_rig(delta: float, simulation: Node, input_map: Node, camera_input_live: bool) -> void:
	if simulation == null or input_map == null or int(simulation.get_step_count()) <= 0:
		_state = RgCameraMath.walker_cam_hold(_state)
		return
	# Always drain the mouse and the wheel so motion made while the camera
	# inputs were off (or another rig was active) does not arrive later.
	var ws: Dictionary = simulation.get_walker_state()
	var input := {
		"has_walker": not ws.is_empty(),
		"feet": Vector3.ZERO,
		"car_position": Vector3.ZERO,
		"mouse": input_map.consume_mouse_delta(),
		"wheel_steps": float(input_map.consume_zoom_steps()),
		"look": Vector2.ZERO,
		"zoom_key": 0.0,
		"live": camera_input_live,
		"first_person": first_person,
	}
	if not ws.is_empty():
		input["feet"] = ws["position"]
		input["car_position"] = ws["car_position"]
	if camera_input_live:
		input["look"] = input_map.get_camera_look_rate()
		input["zoom_key"] = float(input_map.get_camera_zoom_key())
	var frame: Dictionary = RgCameraMath.walker_cam_step(_state, input, delta, _params())
	_state = frame
	if not bool(frame["placed"]):
		# Not spawned yet (waiting for the terrain gate / the next tick) or
		# already despawned: hold still, keep the look responsive.
		_apply_rotation(frame["yaw"], frame["pitch"])
		return
	if not bool(frame["third_person"]):
		position = frame["position"]
		_apply_rotation(frame["yaw"], frame["pitch"])
		return
	# Third person: orbit the head along the look direction, clear of the terrain.
	var pos: Vector3 = frame["position"]
	var grounds: Array = []
	for probe in RgCameraMath.ground_probe_points(pos):
		grounds.append(simulation.get_camera_ground_height(probe))
	pos = RgCameraMath.apply_clearance(pos, grounds, terrain_clearance_m)
	position = pos
	# Aim at the pivot (the clearance push may have tilted the line of sight);
	# only the rotation changes, the stored look angles stay the player's.
	var aim: Dictionary = RgCameraMath.walker_cam_aim(frame["pivot"], pos, _params())
	if bool(aim["valid"]):
		_apply_rotation(aim["yaw"], aim["pitch"])
	else:
		_apply_rotation(frame["yaw"], frame["pitch"])
