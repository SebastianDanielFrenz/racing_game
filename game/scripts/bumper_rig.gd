extends Node3D
# game/scripts/bumper_rig.gd - the bumper / hood view (rg::DriveView::Bumper,
# PLAN.md R5). A rig like chase_rig.gd: a ROOT Node3D plus a Camera3D child;
# camera_director.gd moves the root for floating-origin rebases (shift_origin)
# and calls update_rig() only while this rig is active (PHYS-008 rig contract).
#
# Where the camera sits is rg_core's call (RgSimulation.get_bumper_camera ->
# rg::bumper_eye_local: just ahead of the front axle, low, looking very
# slightly down the road, derived from the vehicle's own wheels); this script
# only mounts the camera rigidly on the chassis transform and lets the player
# glance left/right/back with the same camera-group look input the cockpit rig
# uses. There is no smoothing, so nothing here holds an absolute position that
# a rebase would have to correct - the root is re-placed from the chassis every
# frame, and shift_origin() moves it as the contract requires.

# The setting "camera.fov_deg" is the base field of view; the bumper view is
# wider than the chase view (a camera at the nose needs the room).
const FOV_OFFSET_DEG := 10.0

@export var look_deadzone: float = 0.2
@export var look_lag: float = 14.0

var camera: Camera3D
var _yaw: float = 0.0
var _mount: Dictionary = {}

func _ready() -> void:
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.near = 0.05
	camera.far = 43000.0
	camera.fov = 70.0 + FOV_OFFSET_DEG
	add_child(camera)

func set_base_fov(base_fov_deg: float) -> void:
	camera.fov = base_fov_deg + FOV_OFFSET_DEG

func activate(_from: Transform3D) -> void:
	_yaw = 0.0
	_mount = {} # re-read: a new Session may carry another vehicle

func shift_origin(delta: Vector3) -> void:
	position += delta

func update_rig(delta: float, simulation: Node, input_map: Node, input_live: bool) -> void:
	if simulation == null or int(simulation.get_step_count()) <= 0:
		return
	if _mount.is_empty():
		_mount = simulation.get_bumper_camera()
		if _mount.is_empty():
			return
	var chassis: Transform3D = simulation.get_body_transform("chassis")
	var target := 0.0
	if input_live and input_map != null:
		var keys: Vector2 = input_map.get_camera_key_look()
		var stick: Vector2 = input_map.get_camera_stick()
		if absf(keys.x) > 0.5:
			target = signf(keys.x) * PI * 0.5
		elif keys.y < -0.5:
			target = PI
		elif stick.length() > look_deadzone:
			target = atan2(stick.x, stick.y)
	_yaw = target if look_lag <= 0.0 else lerp_angle(_yaw, target, 1.0 - exp(-look_lag * delta))
	var up: Vector3 = chassis.basis.z.normalized()
	var side: Vector3 = chassis.basis.y.normalized().rotated(up, -_yaw)
	var direction: Vector3 = chassis.basis.x.normalized().rotated(up, -_yaw)
	# Pitch down about the car's left axis: positive rotation about +y (left)
	# tips the forward axis towards the ground (ISO x forward, y left, z up).
	direction = direction.rotated(side, deg_to_rad(float(_mount.get("pitch_down_deg", 1.5))))
	position = chassis * (_mount.get("eye_local", Vector3(2.0, 0.0, 0.2)) as Vector3)
	basis = Basis.looking_at(direction, up)
