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
#
# Display only (docs/unreal_port.md leak #10): yaw/pitch from the look input, the
# heading-relative movement and the pitch limit are rg::free_* in
# core/include/rg/camera_math.h (RgCameraMath.free_*). This script keeps the
# state Dictionary, forwards input and applies the rotation.

@export var move_speed_mps: float = 40.0
@export var fast_multiplier: float = 6.0
@export var look_rate_rad_s: float = 1.8
@export var mouse_sensitivity: float = 0.0025
@export var pitch_limit_rad: float = 1.5

var camera: Camera3D
var _state: Dictionary = {} # RgCameraMath.free_* state

func _params() -> Dictionary:
	return {
		"move_speed_mps": move_speed_mps,
		"fast_multiplier": fast_multiplier,
		"look_rate_rad_s": look_rate_rad_s,
		"mouse_sensitivity": mouse_sensitivity,
		"pitch_limit_rad": pitch_limit_rad,
	}

func _ready() -> void:
	_state = RgCameraMath.free_initial_state()
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.fov = 70.0
	camera.near = 0.25
	camera.far = 43000.0
	add_child(camera)

# Starts where the previous camera was, looking the same way (no jump on a
# mode switch).
func activate(from: Transform3D) -> void:
	_state = RgCameraMath.free_activate(_state, from.origin, -from.basis.z, _params())
	_apply()

# Absolute placement (screenshot tour / tools): position, yaw (0 = -Z,
# + = turn left) and pitch (+ = up), radians.
func place(pos: Vector3, yaw: float, pitch: float) -> void:
	_state = RgCameraMath.free_place(_state, pos, yaw, pitch, _params())
	_apply()

func shift_origin(delta: Vector3) -> void:
	_state = RgCameraMath.free_shift_origin(_state, delta)
	position += delta

func _apply() -> void:
	position = _state["position"]
	rotation = Vector3(0.0, float(_state["yaw_rad"]), 0.0)
	if camera != null:
		camera.rotation = Vector3(float(_state["pitch_rad"]), 0.0, 0.0)

func update_rig(delta: float, _simulation: Node, input_map: Node, camera_input_live: bool) -> void:
	if not camera_input_live or input_map == null:
		return
	var input := {
		"mouse": input_map.consume_mouse_delta(),
		"look": input_map.get_camera_look_rate(),
		"move": input_map.get_camera_move(),
		"fast": bool(input_map.get_camera_fast()),
		"live": true,
	}
	_state = RgCameraMath.free_step(_state, input, delta, _params())
	_apply()
