extends Node3D
# game/scripts/orbit_rig.gd - the orbit view (rg::DriveView::Orbit, PLAN.md
# R5): the camera circles the car at a distance the player sets. A rig like
# chase_rig.gd (root Node3D + Camera3D child; camera_director.gd shifts the
# root on a floating-origin rebase and calls update_rig() only while active).
#
# The motion is rg_core's (RgCameraMath.orbit_step: azimuth/elevation/distance
# from look and zoom input, clamps, the idle turntable drift); this script
# keeps the state Dictionary between frames, follows the car with a short lag
# and places the camera. Input comes only from input_map.gd's camera group:
# arrow keys / right stick (or the captured mouse) turn, wheel / PageUp /
# PageDown zoom. The one absolute position held across frames (the smoothed look-at
# point) is shifted in shift_origin().

const SNAP_M := 60.0
const FOLLOW_LAG := 8.0 # higher = snappier
const CLEARANCE_M := 0.6
const MOUSE_LOOK_PER_PX := 0.05 # full deflection (1.0) per 20 px of captured-mouse motion in one frame

var camera: Camera3D

var _state: Dictionary = {}
var _target: Vector3 = Vector3.ZERO
var _initialized: bool = false

func _ready() -> void:
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.near = 0.1
	camera.far = 43000.0
	camera.fov = 70.0
	add_child(camera)

func set_base_fov(base_fov_deg: float) -> void:
	camera.fov = base_fov_deg

func activate(_from: Transform3D) -> void:
	_state = RgCameraMath.orbit_default_state()
	_initialized = false

func shift_origin(delta: Vector3) -> void:
	position += delta
	_target += delta

func update_rig(delta: float, simulation: Node, input_map: Node, input_live: bool) -> void:
	if simulation == null or int(simulation.get_step_count()) <= 0:
		_initialized = false
		return
	if _state.is_empty():
		_state = RgCameraMath.orbit_default_state()
	var chassis: Transform3D = simulation.get_body_transform("chassis")
	var focus: Vector3 = chassis.origin + Vector3.UP * 1.0
	if not _initialized or _target.distance_to(focus) > SNAP_M:
		_target = focus
		_initialized = true
	_target = _target.lerp(focus, 1.0 - exp(-FOLLOW_LAG * delta))

	var input := {"look_x": 0.0, "look_y": 0.0, "zoom_steps": 0.0, "zoom_key": 0.0, "live": input_live}
	if input_live and input_map != null:
		var look: Vector2 = input_map.get_camera_look_rate()
		var mouse: Vector2 = input_map.consume_mouse_delta()
		# Right/up on the stick or arrow keys moves the camera right/up; the
		# captured mouse drags the camera the same way (right = camera right,
		# mouse up = camera down, like looking up from below the car).
		input["look_x"] = clampf(look.x + mouse.x * MOUSE_LOOK_PER_PX, -1.0, 1.0)
		input["look_y"] = clampf(look.y + mouse.y * MOUSE_LOOK_PER_PX, -1.0, 1.0)
		input["zoom_steps"] = input_map.consume_zoom_steps()
		input["zoom_key"] = input_map.get_camera_zoom_key()
	_state = RgCameraMath.orbit_step(_state, input, delta)

	# offset: ISO heading frame (x forward, y left, z up) -> Godot.
	var o: Vector3 = _state["offset"]
	var forward := Vector3(chassis.basis.x.x, 0.0, chassis.basis.x.z)
	forward = forward.normalized() if forward.length() > 0.001 else Vector3(0.0, 0.0, -1.0)
	var left := Vector3.UP.cross(forward)
	position = _target + forward * o.x + left * o.y + Vector3.UP * o.z
	var ground = simulation.get_camera_ground_height(position)
	if ground != null:
		position.y = maxf(position.y, float(ground) + CLEARANCE_M)
	if position.distance_to(_target) > 0.01:
		look_at(_target, Vector3.UP)
