extends Node3D
# game/scripts/chase_rig.gd — the Drive mode camera rig (rg::CameraRig::Chase).
# A rig is a ROOT Node3D plus a Camera3D child (a future XR rig swaps the
# child for an XRCamera3D under an XROrigin3D root): camera_director.gd moves
# the ROOT for floating-origin rebases (shift_origin) and calls update_rig()
# only while this rig is active. The rig never rebases by itself.
#
# Follows the chassis's own Transform3D (RgSimulation.get_body_transform,
# already origin-relative) with an exponential lag; the look-around orbit
# comes from the camera input group (right stick POSITION sets the angle -
# releasing it returns to the chase view - or the arrow keys, held = look
# 90 degrees left/right, up+down... = look back), only while camera inputs
# are live. Snaps instead of lagging after a jump of more than SNAP_M (a
# relocation / "reset car").
# Ported from physics_sim's adapters/godot/demo/scripts/chase_cam.gd (read-
# only reference) via this repo's R0 chase_cam.gd, which it replaces.

const SNAP_M := 60.0

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

var camera: Camera3D
var _smoothed_pos: Vector3 = Vector3.ZERO
var _initialized: bool = false
var _orbit_yaw: float = 0.0 # radians, + = looking right of the car's heading

func _ready() -> void:
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.fov = 70.0
	camera.near = 0.1
	camera.far = 25000.0
	add_child(camera)

func activate(_from: Transform3D) -> void:
	_initialized = false # re-seat behind the car

func shift_origin(delta: Vector3) -> void:
	position += delta
	_smoothed_pos += delta

func _target_orbit_yaw(input_map: Node) -> float:
	var look: Vector2 = input_map.get_camera_key_look()
	if absf(look.x) > 0.5:
		return signf(look.x) * PI * 0.5
	if look.y < -0.5:
		return PI
	var stick: Vector2 = input_map.get_camera_stick()
	var mag: float = min(stick.length(), 1.0)
	if mag <= orbit_deadzone:
		return 0.0
	# Direction selects a full-circle orbit. lerp_angle below wraps the
	# front-view +/-PI seam; scaling the angle by magnitude would split it.
	return atan2(stick.x, stick.y)

func update_rig(delta: float, simulation: Node, input_map: Node, camera_input_live: bool) -> void:
	if simulation == null or int(simulation.get_step_count()) <= 0:
		_initialized = false
		return
	var chassis_xform: Transform3D = simulation.get_body_transform("chassis")
	var chassis_pos: Vector3 = chassis_xform.origin
	# basis.x is the car's FORWARD direction in Godot space (frame_convert.h:
	# column i = the sim's local axis i, ISO 8855 x-forward).
	var behind := -chassis_xform.basis.x.normalized() * follow_distance_m
	var side := chassis_xform.basis.y.normalized() * side_offset_m
	var desired_pos := chassis_pos + behind + side + Vector3.UP * follow_height_m
	var look_at_pos := chassis_pos + Vector3.UP * look_height_m

	if not _initialized or _smoothed_pos.distance_to(desired_pos) > SNAP_M:
		_smoothed_pos = desired_pos
		_initialized = true
	_smoothed_pos = _smoothed_pos.lerp(desired_pos, 1.0 - exp(-position_lag * delta))

	var target_yaw := 0.0
	if camera_input_live and input_map != null:
		target_yaw = _target_orbit_yaw(input_map)
	if orbit_lag <= 0.0:
		_orbit_yaw = target_yaw
	else:
		_orbit_yaw = lerp_angle(_orbit_yaw, target_yaw, 1.0 - exp(-orbit_lag * delta))
	var offset := _smoothed_pos - chassis_pos
	position = chassis_pos + offset.rotated(Vector3.UP, -_orbit_yaw)
	# Resolve clearance after following lag AND orbit, which can otherwise
	# put the camera below rising terrain. Probe its near-plane footprint.
	for probe in [Vector3.ZERO, Vector3(0.3,0,0), Vector3(-0.3,0,0), Vector3(0,0,0.3), Vector3(0,0,-0.3)]:
		var ground = simulation.get_camera_ground_height(position+probe)
		if ground != null:
			position.y = maxf(position.y,float(ground)+terrain_clearance_m)
	if position.distance_to(look_at_pos) > 0.01:
		look_at(look_at_pos, Vector3.UP)
