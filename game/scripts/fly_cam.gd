extends Camera3D
# game/scripts/fly_cam.gd — R2.1 terrain-preview-only free camera. Used
# ONLY by main.gd's --terrain-preview branch (no RgSimulation/vehicle exists
# in that mode at all) - deliberately independent of input_map.gd/
# project.godot's action map, so this feature adds zero InputMap actions:
# it reads raw keys/mouse motion straight off Input/_unhandled_input, the
# same minimal shape as any stand-alone fly-cam script.
#
# Controls (PLAN.md R2.1's own spec):
#   WASD              - move forward/back/left/right (camera-local, ignores
#                        pitch so straight running never dives/climbs)
#   Space or E        - move up (world +Y)
#   Ctrl or Q         - move down (world -Y)
#   Shift (held)      - fast_multiplier x move speed
#   right mouse button - capture the mouse and start mouse-look (captured
#                        mouse motion turns the camera; press again after
#                        Esc to recapture)
#   Esc               - release the mouse (MOUSE_MODE_VISIBLE)
#
# No floating-origin rebase here (unlike chase_cam.gd): R2.1's preview is a
# single fixed-origin fly-around of one home-region view, not a long-distance
# drive - RgTerrainView.set_render_origin is called once at load (main.gd),
# never per frame.

@export var move_speed_mps: float = 60.0
@export var fast_multiplier: float = 6.0
@export var mouse_sensitivity: float = 0.0025
@export var pitch_limit_rad: float = 1.5 # a hair under +-90 deg

var _yaw: float = 0.0
var _pitch: float = 0.0
var _mouse_captured: bool = false

func _ready() -> void:
	# Preserve whatever yaw/pitch the caller placed this camera at
	# (main.gd points it at the ridge before the first frame runs).
	_yaw = rotation.y
	_pitch = rotation.x
	Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
	_mouse_captured = false

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo and event.keycode == KEY_ESCAPE:
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
		_mouse_captured = false
		return

	if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_RIGHT:
		Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
		_mouse_captured = true
		return

	if event is InputEventMouseMotion and _mouse_captured:
		_yaw -= event.relative.x * mouse_sensitivity
		_pitch -= event.relative.y * mouse_sensitivity
		_pitch = clampf(_pitch, -pitch_limit_rad, pitch_limit_rad)
		rotation = Vector3(_pitch, _yaw, 0.0)

func _process(delta: float) -> void:
	var basis: Basis = transform.basis
	var move := Vector3.ZERO
	if Input.is_key_pressed(KEY_W):
		move -= basis.z
	if Input.is_key_pressed(KEY_S):
		move += basis.z
	if Input.is_key_pressed(KEY_A):
		move -= basis.x
	if Input.is_key_pressed(KEY_D):
		move += basis.x
	if Input.is_key_pressed(KEY_SPACE) or Input.is_key_pressed(KEY_E):
		move += Vector3.UP
	if Input.is_key_pressed(KEY_CTRL) or Input.is_key_pressed(KEY_Q):
		move -= Vector3.UP

	if move.length() > 0.0001:
		move = move.normalized()

	var speed := move_speed_mps
	if Input.is_key_pressed(KEY_SHIFT):
		speed *= fast_multiplier

	position += move * speed * delta
