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
#  - lag: exponential, snaps after a jump of more than SNAP_M (a relocation, or
#    a target switch to a far-away vehicle);
#  - orbit: from the right-stick POSITION or the arrow keys, like chase_rig.gd;
#  - zoom: the trailing distance (the height scales with it) from the mouse
#    wheel and PageUp/PageDown, clamped to MIN_DISTANCE_M..MAX_DISTANCE_M;
#  - terrain clearance: simulation.get_camera_ground_height() probes.
# When the target is not available (the Session lost it) the rig holds still
# for the frame; rg_core returns the mode to the own car on its own.

const SNAP_M := 60.0
const MIN_DISTANCE_M := 4.0
const MAX_DISTANCE_M := 60.0

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
var _distance_m: float = 12.0
var _smoothed_pos: Vector3 = Vector3.ZERO
var _initialized: bool = false
var _orbit_yaw: float = 0.0 # radians, + = looking right of the target's heading
var _heading: Vector3 = Vector3.FORWARD # last horizontal heading of the target (Godot space)
var _drain_wheel: bool = false

func _ready() -> void:
	_distance_m = clampf(follow_distance_m, MIN_DISTANCE_M, MAX_DISTANCE_M)
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.fov = 70.0
	camera.near = 0.1
	camera.far = 43000.0
	add_child(camera)

# Starts from where the previous camera was and flies to the drone position
# (the exponential lag; a jump beyond SNAP_M snaps instead).
func activate(from: Transform3D) -> void:
	_smoothed_pos = from.origin
	_initialized = true
	_orbit_yaw = 0.0
	_drain_wheel = true # wheel ticks made while another rig was active must not zoom this one

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
	return atan2(stick.x, stick.y)

func _update_zoom(delta: float, input_map: Node, camera_input_live: bool) -> void:
	if input_map == null:
		return
	# Always drain the wheel so ticks made while the camera inputs were off do
	# not arrive later as one big zoom.
	var wheel: float = input_map.consume_zoom_steps()
	if _drain_wheel:
		_drain_wheel = false
		return
	if not camera_input_live:
		return
	var key: float = input_map.get_camera_zoom_key()
	_distance_m *= pow(zoom_wheel_factor, -wheel) * exp(-key * zoom_key_rate * delta)
	_distance_m = clampf(_distance_m, MIN_DISTANCE_M, MAX_DISTANCE_M)

func update_rig(delta: float, simulation: Node, input_map: Node, camera_input_live: bool) -> void:
	if simulation == null or int(simulation.get_step_count()) <= 0:
		_initialized = false
		return
	_update_zoom(delta, input_map, camera_input_live)
	var target: Variant = simulation.get_drone_target_transform()
	if target == null:
		return # the Session lost the vehicle this frame; the mode falls back to the own car
	var xform: Transform3D = target
	var target_pos: Vector3 = xform.origin
	# basis.x is the vehicle's FORWARD direction in Godot space (frame_convert.h:
	# column i = the sim's local axis i, ISO 8855 x-forward); keep it horizontal
	# so a pitching car does not swing the drone up and down.
	var fwd := Vector3(xform.basis.x.x, 0.0, xform.basis.x.z)
	if fwd.length() > 0.2:
		_heading = fwd.normalized()
	var behind := -_heading * _distance_m
	var desired_pos := target_pos + behind + Vector3.UP * (_distance_m * height_per_distance)
	var look_at_pos := target_pos + Vector3.UP * look_height_m

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
	var offset := _smoothed_pos - target_pos
	position = target_pos + offset.rotated(Vector3.UP, -_orbit_yaw)
	# Terrain clearance after lag AND orbit (either can put the camera below
	# rising terrain); probe the near-plane footprint like chase_rig.gd.
	for probe in [Vector3.ZERO, Vector3(0.3, 0, 0), Vector3(-0.3, 0, 0), Vector3(0, 0, 0.3), Vector3(0, 0, -0.3)]:
		var ground = simulation.get_camera_ground_height(position + probe)
		if ground != null:
			position.y = maxf(position.y, float(ground) + terrain_clearance_m)
	if position.distance_to(look_at_pos) > 0.01:
		look_at(look_at_pos, Vector3.UP)
