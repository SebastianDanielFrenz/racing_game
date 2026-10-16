extends Node3D
# game/scripts/chase_rig.gd — the Drive mode camera rig (rg::CameraRig::Chase).
# A rig is a ROOT Node3D plus a Camera3D child (a future XR rig swaps the
# child for an XRCamera3D under an XROrigin3D root): camera_director.gd moves
# the ROOT for floating-origin rebases (shift_origin) and calls update_rig()
# only while this rig is active. The rig never rebases by itself.
#
# Follows the chassis's own Transform3D (RgSimulation.get_body_transform,
# already origin-relative). The camera sits at a SMOOTHED OFFSET from the car
# (RgCameraMath.chase_follow_offset, rg_core: 1 - exp(-k*dt), any frame rate): the
# car's translation carries it rigidly, so it never trails at speed or under
# acceleration; only the swing behind a turning/pitching car is eased. (Smoothing
# the absolute position, as this rig used to, trailed the car by speed/6 m: 5 m
# at 30 m/s.) The look-around orbit
# comes from the camera input group (right stick POSITION sets the angle -
# releasing it returns to the chase view - or the arrow keys, held = look
# 90 degrees left/right, up+down... = look back), only while camera inputs
# are live. Snaps instead of lagging after a jump of more than snap_m (a
# relocation / "reset car").
#
# Display only (docs/unreal_port.md leak #10): the follow offset, its lag, the
# orbit yaw, the snap and the terrain clearance are rg::chase_* in
# core/include/rg/camera_math.h (RgCameraMath.chase_*). This script keeps the
# state Dictionary RgCameraMath hands back, forwards the chassis axes and the
# look input, probes the ground and places the Camera3D.
# Ported from physics_sim's adapters/godot/demo/scripts/chase_cam.gd (read-
# only reference) via this repo's R0 chase_cam.gd, which it replaces.

@export var follow_distance_m: float = 7.0
@export var follow_height_m: float = 2.5
@export var look_height_m: float = 1.0
# Lateral offset (world metres, + = the car's left - basis.y, frame_convert.h's
# ISO y) added to the "behind" position, still looking at the centreline.
# Zero for every normal chase view; drive_tour.gd's steering_close shot is the
# only caller that sets this nonzero - a directly-behind camera can never see
# a front wheel at all (the body occludes it face-on), so a 3/4 view needs
# some sideways offset to reveal the near-side wheels' profile/steer angle.
@export var side_offset_m: float = 0.0
@export var terrain_clearance_m: float = 0.6
@export var position_lag: float = 6.0 # higher = snappier
@export var orbit_deadzone: float = 0.2
@export var orbit_lag: float = 14.0 # higher = snappier; <= 0 = no smoothing
@export var snap_m: float = 60.0 # an offset jump this large (a changed follow distance) snaps

var camera: Camera3D
var _state: Dictionary = {} # RgCameraMath.chase_* state

func _params() -> Dictionary:
	return {
		"follow_distance_m": follow_distance_m,
		"follow_height_m": follow_height_m,
		"look_height_m": look_height_m,
		"side_offset_m": side_offset_m,
		"terrain_clearance_m": terrain_clearance_m,
		"position_lag": position_lag,
		"orbit_deadzone": orbit_deadzone,
		"orbit_lag": orbit_lag,
		"snap_m": snap_m,
	}

func _ready() -> void:
	_state = RgCameraMath.chase_initial_state()
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.fov = 70.0
	camera.near = 0.1
	camera.far = 43000.0
	add_child(camera)

# Speed effects from camera_director.gd (RgCameraMotion view "chase"): a small
# shake (car frame, added after aiming) and FOV widening; no head motion here.
var motion: Dictionary = {}
var motion_view := "chase"
var motion_enabled := true
var _base_fov: float = 70.0

func set_base_fov(base_fov_deg: float) -> void:
	_base_fov = base_fov_deg
	camera.fov = base_fov_deg

func activate(_from: Transform3D) -> void:
	_state = RgCameraMath.chase_reset(_state) # re-seat behind the car

func shift_origin(delta: Vector3) -> void:
	position += delta # the offset is car-relative: a rebase does not touch it

func update_rig(delta: float, simulation: Node, input_map: Node, camera_input_live: bool) -> void:
	if simulation == null or int(simulation.get_step_count()) <= 0:
		_state = RgCameraMath.chase_reset(_state)
		return
	var chassis_xform: Transform3D = simulation.get_body_transform("chassis")
	# basis.x is the car's FORWARD direction in Godot space (frame_convert.h:
	# column i = the sim's local axis i, ISO 8855 x-forward), basis.y its left.
	var input := {
		"target_position": chassis_xform.origin,
		"forward": chassis_xform.basis.x,
		"left": chassis_xform.basis.y,
		"key_look": Vector2.ZERO,
		"stick": Vector2.ZERO,
		"live": camera_input_live and input_map != null,
	}
	if input["live"]:
		input["key_look"] = input_map.get_camera_key_look()
		input["stick"] = input_map.get_camera_stick()
	var frame: Dictionary = RgCameraMath.chase_follow_step(_state, input, delta, _params())
	_state = frame
	# Resolve clearance after following lag AND orbit, which can otherwise
	# put the camera below rising terrain. Probe its near-plane footprint.
	var pos: Vector3 = frame["position"]
	var grounds: Array = []
	for probe in RgCameraMath.ground_probe_points(pos):
		grounds.append(simulation.get_camera_ground_height(probe))
	position = RgCameraMath.apply_clearance(pos, grounds, terrain_clearance_m)
	if RgCameraMath.look_valid(position, frame["look_at"]):
		look_at(frame["look_at"], Vector3.UP)
	if motion_enabled and not motion.is_empty():
		position += chassis_xform.basis * (motion["offset"] as Vector3)
		camera.fov = _base_fov + float(motion["fov_extra_deg"])
