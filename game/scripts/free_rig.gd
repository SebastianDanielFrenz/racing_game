extends Node3D
# game/scripts/free_rig.gd — the FreeCam mode camera rig (rg::CameraRig::Free):
# a ROOT Node3D (position + yaw) with a Camera3D child (pitch). A future XR
# rig keeps the same split (root = XROrigin3D, head look from the headset).
# camera_director.gd moves the ROOT for floating-origin rebases
# (shift_origin) and calls update_rig() only while this rig is active.
#
# Input comes only from input_map.gd's CAMERA group (never raw devices here):
# move = WASD / left stick (horizontal, relative to the heading - straight
# runs never dive), up/down = E,Space / Q,Ctrl / RB,LB / triggers, look = the
# right stick or the arrow keys (rate) and, optionally, the captured mouse
# (right mouse button captures, Esc releases), fast = Shift / left-stick
# click. The old --terrain-preview fly_cam.gd (raw keys, mouse-look only) is
# unchanged and still used by that static preview.

@export var move_speed_mps: float = 40.0
@export var fast_multiplier: float = 6.0
@export var look_rate_rad_s: float = 1.8
@export var mouse_sensitivity: float = 0.0025
@export var pitch_limit_rad: float = 1.5

var camera: Camera3D
var _yaw: float = 0.0
var _pitch: float = 0.0

func _ready() -> void:
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.fov = 70.0
	camera.near = 0.25
	camera.far = 25000.0
	add_child(camera)

# Starts where the previous camera was, looking the same way (no jump on a
# mode switch).
func activate(from: Transform3D) -> void:
	position = from.origin
	var fwd: Vector3 = -from.basis.z
	_yaw = atan2(-fwd.x, -fwd.z)
	_pitch = clampf(asin(clampf(fwd.y, -1.0, 1.0)), -pitch_limit_rad, pitch_limit_rad)
	_apply_rotation()

# Absolute placement (screenshot tour / tools): position, yaw (0 = -Z,
# + = turn left) and pitch (+ = up), radians.
func place(pos: Vector3, yaw: float, pitch: float) -> void:
	position = pos
	_yaw = yaw
	_pitch = clampf(pitch, -pitch_limit_rad, pitch_limit_rad)
	_apply_rotation()

func shift_origin(delta: Vector3) -> void:
	position += delta

func _apply_rotation() -> void:
	rotation = Vector3(0.0, _yaw, 0.0)
	if camera != null:
		camera.rotation = Vector3(_pitch, 0.0, 0.0)

func update_rig(delta: float, _simulation: Node, input_map: Node, camera_input_live: bool) -> void:
	if not camera_input_live or input_map == null:
		return
	var mouse: Vector2 = input_map.consume_mouse_delta()
	var look: Vector2 = input_map.get_camera_look_rate()
	_yaw -= mouse.x * mouse_sensitivity + look.x * look_rate_rad_s * delta
	_pitch -= mouse.y * mouse_sensitivity
	_pitch += look.y * look_rate_rad_s * delta
	_pitch = clampf(_pitch, -pitch_limit_rad, pitch_limit_rad)
	_apply_rotation()

	var m: Vector3 = input_map.get_camera_move()
	var heading := Basis(Vector3.UP, _yaw)
	var move: Vector3 = heading * Vector3(m.x, 0.0, -m.z) + Vector3.UP * m.y
	if move.length() > 1.0:
		move = move.normalized()
	var speed := move_speed_mps * (fast_multiplier if input_map.get_camera_fast() else 1.0)
	position += move * speed * delta
