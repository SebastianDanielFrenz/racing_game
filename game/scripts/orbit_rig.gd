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
# PageDown zoom. The look-at point follows the car through a smoothed
# CAR-RELATIVE offset (zero lag at any constant speed; only changes of the
# offset, e.g. a jump, are smoothed), so nothing absolute needs shifting.
#
# Display only (docs/unreal_port.md leak #10): the look-at lag, the snap, the
# mouse scaling and the placement in the car's heading frame are
# rg::orbit_rig_* in core/include/rg/camera_math.h (RgCameraMath.orbit_rig_*).
# This script keeps the state Dictionary, forwards the chassis pose and input,
# probes the ground and places the Camera3D.

const CLEARANCE_M := 0.6 # terrain clearance (rg::OrbitRigParams::clearance_m)

var camera: Camera3D

var _state: Dictionary = {} # RgCameraMath.orbit_rig_* state

func _ready() -> void:
	_state = RgCameraMath.orbit_rig_initial_state()
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.near = 0.1
	camera.far = 43000.0
	camera.fov = 70.0
	add_child(camera)

func set_base_fov(base_fov_deg: float) -> void:
	camera.fov = base_fov_deg

func activate(_from: Transform3D) -> void:
	_state = RgCameraMath.orbit_rig_activate(_state)

func shift_origin(delta: Vector3) -> void:
	position += delta # the look-at point is chassis-relative: nothing absolute is kept

func update_rig(delta: float, simulation: Node, input_map: Node, input_live: bool) -> void:
	if simulation == null or int(simulation.get_step_count()) <= 0:
		_state = RgCameraMath.orbit_rig_reset(_state)
		return
	var chassis: Transform3D = simulation.get_body_transform("chassis")
	var input := {
		"chassis_position": chassis.origin,
		# basis.x is the car's FORWARD direction in Godot space (frame_convert.h)
		"chassis_forward": chassis.basis.x,
		"look": Vector2.ZERO,
		"mouse": Vector2.ZERO,
		"zoom_steps": 0.0,
		"zoom_key": 0.0,
		"live": input_live,
	}
	if input_live and input_map != null:
		# Right/up on the stick or arrow keys moves the camera right/up; the
		# captured mouse drags the camera the same way (right = camera right,
		# mouse up = camera down, like looking up from below the car).
		input["look"] = input_map.get_camera_look_rate()
		input["mouse"] = input_map.consume_mouse_delta()
		input["zoom_steps"] = input_map.consume_zoom_steps()
		input["zoom_key"] = input_map.get_camera_zoom_key()
	var frame: Dictionary = RgCameraMath.orbit_rig_step(_state, input, delta)
	_state = frame
	var pos: Vector3 = frame["position"]
	var ground = simulation.get_camera_ground_height(pos)
	position = RgCameraMath.apply_clearance(pos, [ground], CLEARANCE_M)
	if RgCameraMath.look_valid(position, frame["target"]):
		look_at(frame["target"], Vector3.UP)
