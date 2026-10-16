extends Node3D
# game/scripts/drone_rig.gd — the DroneFollow mode camera rig
# (rg::CameraRig::Drone). Same rig contract as chase_rig.gd / free_rig.gd: a
# ROOT Node3D plus a Camera3D child; camera_director.gd moves the ROOT for
# floating-origin rebases (shift_origin) and calls update_rig() only while
# this rig is active.
#
# Trails a TARGET vehicle from above and behind: the target's transform comes
# from RgSimulation.get_drone_target_transform() (the player's own chassis, a
# traffic car or the NPC truck - rg_core's mode machine decides which, this
# rig only displays; the call is cheap, never the whole traffic list).
#  - lag: exponential on the TARGET-RELATIVE offset (zero lag at any constant
#    speed, only offset changes - heading, zoom, the fly-in - are smoothed);
#    snaps after a jump of more than SNAP_M (a relocation, or a target switch
#    to a far-away vehicle);
#  - orbit: from the right-stick POSITION or the arrow keys, like chase_rig.gd;
#  - zoom: the trailing distance (the height scales with it) from the mouse
#    wheel and PageUp/PageDown, clamped to MIN_DISTANCE_M..MAX_DISTANCE_M;
#  - terrain clearance: simulation.get_camera_ground_height() probes.
# When the target is not available (the Session lost it) the rig holds still
# for the frame; rg_core returns the mode to the own car on its own.
#
# Display only (docs/unreal_port.md leak #10): the orbit-yaw target, the zoom law,
# the lag filter, the snap and the terrain clearance are rg::drone_* in
# core/include/rg/camera_math.h (RgCameraMath.drone_*). This script keeps the
# state Dictionary RgCameraMath hands back, forwards input, probes the ground
# (simulation.get_camera_ground_height) and places the Camera3D.

@export var follow_distance_m: float = 12.0 # horizontal distance behind the target (the default zoom)
@export var height_per_distance: float = 0.5 # follow_height = distance * this (12 m back -> 6 m up)
@export var look_height_m: float = 1.0
@export var terrain_clearance_m: float = 1.0
@export var position_lag: float = 4.0 # higher = snappier
@export var orbit_deadzone: float = 0.2
@export var orbit_lag: float = 10.0 # higher = snappier; <= 0 = no smoothing
@export var zoom_wheel_factor: float = 1.15 # distance factor per wheel tick
@export var zoom_key_rate: float = 1.2 # e-folds per second while PageUp/PageDown is held

var camera: Camera3D
var _state: Dictionary = {} # RgCameraMath.drone_* state

func _params() -> Dictionary:
	return {
		"follow_distance_m": follow_distance_m,
		"height_per_distance": height_per_distance,
		"look_height_m": look_height_m,
		"terrain_clearance_m": terrain_clearance_m,
		"position_lag": position_lag,
		"orbit_deadzone": orbit_deadzone,
		"orbit_lag": orbit_lag,
		"zoom_wheel_factor": zoom_wheel_factor,
		"zoom_key_rate": zoom_key_rate,
	}

func _ready() -> void:
	_state = RgCameraMath.drone_initial_state(_params())
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.fov = 70.0
	camera.near = 0.1
	camera.far = 43000.0
	add_child(camera)

# Starts from where the previous camera was and flies to the drone position
# (the exponential lag; a jump beyond the snap distance snaps instead).
func activate(from: Transform3D) -> void:
	_state = RgCameraMath.drone_activate(_state, from.origin)

func shift_origin(delta: Vector3) -> void:
	position += delta
	_state = RgCameraMath.drone_shift_origin(_state, delta)

func update_rig(delta: float, simulation: Node, input_map: Node, camera_input_live: bool) -> void:
	if simulation == null or int(simulation.get_step_count()) <= 0:
		_state = RgCameraMath.drone_hold(_state)
		return
	if input_map != null:
		# The wheel is always drained so ticks made while the camera inputs were
		# off do not arrive later as one big zoom.
		_state = RgCameraMath.drone_zoom_step(_state, float(input_map.consume_zoom_steps()),
			float(input_map.get_camera_zoom_key()), camera_input_live, delta, _params())
	var target: Variant = simulation.get_drone_target_transform()
	if target == null:
		return # the Session lost the vehicle this frame; the mode falls back to the own car
	var xform: Transform3D = target
	var input := {
		"target_position": xform.origin,
		# basis.x is the vehicle's FORWARD direction in Godot space (frame_convert.h)
		"target_forward": xform.basis.x,
		"key_look": Vector2.ZERO,
		"stick": Vector2.ZERO,
		"live": camera_input_live and input_map != null,
	}
	if input["live"]:
		input["key_look"] = input_map.get_camera_key_look()
		input["stick"] = input_map.get_camera_stick()
	var params := _params()
	var frame: Dictionary = RgCameraMath.drone_follow_step(_state, input, delta, params)
	_state = frame
	var pos: Vector3 = frame["position"]
	var grounds: Array = []
	for probe in RgCameraMath.drone_probe_points(pos):
		grounds.append(simulation.get_camera_ground_height(probe))
	position = RgCameraMath.drone_apply_clearance(pos, grounds, params)
	if bool(frame["has_look"]):
		look_at(frame["look_at"], Vector3.UP)
