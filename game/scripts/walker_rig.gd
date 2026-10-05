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
var _yaw: float = 0.0     # 0 = looking along -Z, + = turn left (free_rig.gd's convention)
var _pitch: float = -0.15 # + = up
var _distance_m: float = 3.2
var _smoothed_feet: Vector3 = Vector3.ZERO
var _have_walker: bool = false
var _drain_wheel: bool = false

func _ready() -> void:
	_distance_m = clampf(third_person_distance_m, min_distance_m, max_distance_m)
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
	_have_walker = false
	_drain_wheel = true
	var fwd: Vector3 = -from.basis.z
	_yaw = atan2(-fwd.x, -fwd.z)
	_pitch = -0.15
	_apply_rotation()

func shift_origin(delta: Vector3) -> void:
	position += delta
	_smoothed_feet += delta

func toggle_first_person() -> void:
	first_person = not first_person

# Horizontal forward direction of the look heading in Godot space, for the
# walking input (the sim turns it into an ISO yaw).
func get_look_forward() -> Vector3:
	return Basis(Vector3.UP, _yaw) * Vector3(0.0, 0.0, -1.0)

func _apply_rotation() -> void:
	rotation = Vector3(0.0, _yaw, 0.0)
	if camera != null:
		camera.rotation = Vector3(_pitch, 0.0, 0.0)

func _update_look(delta: float, input_map: Node, live: bool) -> void:
	# Always drain the mouse and the wheel so motion made while the camera
	# inputs were off (or another rig was active) does not arrive later.
	var mouse: Vector2 = input_map.consume_mouse_delta()
	var wheel: float = input_map.consume_zoom_steps()
	if _drain_wheel:
		_drain_wheel = false
		wheel = 0.0
	if not live:
		return
	var look: Vector2 = input_map.get_camera_look_rate()
	_yaw -= mouse.x * mouse_sensitivity + look.x * look_rate_rad_s * delta
	_pitch -= mouse.y * mouse_sensitivity
	_pitch += look.y * look_rate_rad_s * delta
	if first_person:
		_pitch = clampf(_pitch, -first_pitch_limit, first_pitch_limit)
	else:
		_pitch = clampf(_pitch, third_pitch_min, third_pitch_max)
		var key: float = input_map.get_camera_zoom_key()
		_distance_m *= pow(1.15, -wheel) * exp(-key * 1.2 * delta)
		_distance_m = clampf(_distance_m, min_distance_m, max_distance_m)

func update_rig(delta: float, simulation: Node, input_map: Node, camera_input_live: bool) -> void:
	if simulation == null or input_map == null or int(simulation.get_step_count()) <= 0:
		_have_walker = false
		return
	var ws: Dictionary = simulation.get_walker_state()
	if ws.is_empty():
		# Not spawned yet (waiting for the terrain gate / the next tick) or
		# already despawned: hold still, keep the look responsive.
		_update_look(delta, input_map, camera_input_live)
		_apply_rotation()
		return
	var feet: Vector3 = ws["position"]
	if not _have_walker:
		_have_walker = true
		_smoothed_feet = feet
		# First sight of the walker: look at the car it just got out of.
		var to_car: Vector3 = ws["car_position"] - feet
		to_car.y = 0.0
		if to_car.length() > 0.1:
			_yaw = atan2(-to_car.x, -to_car.z)
		_pitch = -0.15
	elif _smoothed_feet.distance_to(feet) > snap_m:
		_smoothed_feet = feet
	else:
		_smoothed_feet = _smoothed_feet.lerp(feet, 1.0 - exp(-follow_rate * delta))
	_update_look(delta, input_map, camera_input_live)

	if first_person:
		position = _smoothed_feet + Vector3.UP * eye_height_m
		_apply_rotation()
		return
	# Third person: orbit the head along the look direction.
	var pivot: Vector3 = _smoothed_feet + Vector3.UP * head_height_m
	var look_dir: Vector3 = Basis(Vector3.UP, _yaw) * (Basis(Vector3.RIGHT, _pitch) * Vector3(0.0, 0.0, -1.0))
	var pos: Vector3 = pivot - look_dir * _distance_m
	for probe in [Vector3.ZERO, Vector3(0.3, 0, 0), Vector3(-0.3, 0, 0), Vector3(0, 0, 0.3), Vector3(0, 0, -0.3)]:
		var ground = simulation.get_camera_ground_height(pos + probe)
		if ground != null:
			pos.y = maxf(pos.y, float(ground) + terrain_clearance_m)
	position = pos
	# Aim at the pivot (the clearance push may have tilted the line of sight);
	# only the rotation changes, the stored look angles stay the player's.
	var to_pivot: Vector3 = pivot - pos
	var flat: float = Vector2(to_pivot.x, to_pivot.z).length()
	if flat > 0.01:
		rotation = Vector3(0.0, atan2(-to_pivot.x, -to_pivot.z), 0.0)
		camera.rotation = Vector3(clampf(atan2(to_pivot.y, flat), third_pitch_min, third_pitch_max), 0.0, 0.0)
	else:
		_apply_rotation()
