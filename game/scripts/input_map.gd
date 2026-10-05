extends Node
# game/scripts/input_map.gd — the input layer (a plain GDScript node, not a
# C++ class like physics_sim's PsInputMap - see this repo's CLAUDE.md). It
# only READS devices and shapes values; it never decides what they mean for
# the game. main.gd forwards the groups the mode framework says are live
# (rg_core's rg::PlayerModeMachine::effective_rules via RgSimulation.
# get_mode_state): the DRIVING group into RgSimulation.set_control, the
# CAMERA group into the active camera rig. A key shared by both groups (W/S,
# E/Q) therefore means one thing at a time. A future XR input map replaces
# this node with the same getters.
#
# Driving group (gamepad + keyboard, larger magnitude wins - physics_sim's
# input_map.cpp merge convention): steer/throttle/brake/handbrake/clutch,
# shift up/down edge counts, ignition (TOGGLE, starts on), starter (held),
# assist.auto_shift (TOGGLE, starts off for manual cars).
# Camera group: move (x right, y up, z forward, each -1..1), look rate
# (yaw/pitch, -1..1: right stick or arrow keys), mouse look (captured mouse
# delta in pixels; the right mouse button captures, Esc releases), fast.
# Drone-follow zoom (camera group): mouse wheel ticks (consume_zoom_steps) and
# PageUp/PageDown held (get_camera_zoom_key); next drone target is a global
# action (N / D-pad right); the next driving view is B / RB (owner 2026-10-05,
# matching physics_sim; in free cam RB is also "up", harmless: the view cycle
# only acts in drive mode). Gamepad Y toggles the nitrous arm switch (the
# "nitrous_arm" control channel, a plain 0/1 level; only cars with a nitrous kit
# declare it). Resetting the car is keyboard-only (owner 2026-10-05: "Reset car
# should never be on the controller").
# Walking group (on foot, R9c): reuses the camera group's move axes (WASD /
# left stick: x right, z forward, relative to the look heading, which the walker
# rig owns) and "fast" as RUN (Shift / left-stick click); adds JUMP (Space /
# gamepad A, edge count) and INTERACT (G / gamepad X, edge count). G is also the
# keyboard GET-OUT key in drive mode (edge count of the KEY only: gamepad X is
# shift-down while driving, so it must never get the player out of the car).
# Global actions (edges, consumed once per frame by main.gd): cycle mode, cycle
# driving view, switch world, reset car, next drone target. Those plus the camera keys are added to InputMap
# at runtime (_ensure_action), so project.godot's [input] section only holds
# the R0 driving actions.
#
# shift_up/shift_down are per-frame-consumed EDGE COUNTS, mirroring the
# shift_up_count/shift_down_count channels' "edges since last consumed"
# contract (session.cpp's kControlChannelNames) - main.gd reads each once
# per frame.

const STEER_DEADZONE := 0.08
const TRIGGER_DEADZONE := 0.02
const STICK_DEADZONE := 0.15

@export var joy_device: int = 0
@export var camera_joy_device: int = -1 # -1: right-stick activity selects device
var _camera_device := -1
var _camera_devices_signature := ""

var _prev := {}
var _shift_up_count: int = 0
var _shift_down_count: int = 0
var _cycle_camera_count: int = 0
var _cycle_mode_count: int = 0
var _cycle_view_count: int = 0
var _switch_world_count: int = 0
var _reset_car_count: int = 0
var _nitrous_toggle_count: int = 0
var _flip_upright_count: int = 0
var _cycle_drone_count: int = 0
var _jump_count: int = 0
var _interact_count: int = 0
var _get_out_key_count: int = 0
var _zoom_steps: float = 0.0 # mouse-wheel ticks since last consumed, + = zoom in
var _cam_zoom_key: float = 0.0 # PageUp (+1, zoom in) / PageDown (-1, zoom out)

var _steer_keyboard: bool = false
var _steer_wheel: bool = false
var _steer: float = 0.0
var _throttle: float = 0.0
var _brake: float = 0.0
var _handbrake: float = 0.0
var _clutch: float = 0.0
var _ignition_on: bool = true # physics_sim's input_map_core default: spawn with the engine running
var _starter_held: bool = false
var _auto_shift_on: bool = false
var _configured_vehicle_path: String = ""

var _cam_move: Vector3 = Vector3.ZERO
var _cam_look_rate: Vector2 = Vector2.ZERO
var _cam_key_look: Vector2 = Vector2.ZERO
var _cam_stick: Vector2 = Vector2.ZERO # raw right-stick position (chase orbit), x right, y up
var _cam_fast: bool = false
var _mouse_delta: Vector2 = Vector2.ZERO
var _mouse_captured: bool = false

# Gives the mouse back (pause menu opened with the keyboard while it was
# captured for looking around).
func release_mouse() -> void:
	Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
	_mouse_captured = false
	_mouse_delta = Vector2.ZERO

func is_mouse_captured() -> bool:
	return _mouse_captured

# `auto_shift_default` (R6): the car's assist default after its saved setup; -1 = decide
# from the controller block (a manual gearbox starts with auto-shift off).
func configure_vehicle(path: String, auto_shift_default = null) -> void:
	# Keep the driver toggle across world/reset reloads of the same car.
	var key := "%s|%s" % [path, str(auto_shift_default)]
	if key == _configured_vehicle_path:
		return
	var vehicle = JSON.parse_string(FileAccess.get_file_as_string(path))
	if not vehicle is Dictionary:
		return
	var manual := false
	for controller in vehicle.get("controllers", []):
		if controller is Dictionary and str(controller.get("type", "")) == "manual_tcu":
			manual = true
			break
	_auto_shift_on = (not manual) if auto_shift_default == null else bool(auto_shift_default)
	_configured_vehicle_path = key

func _ready() -> void:
	_ensure_action("rg_cycle_camera", [KEY_TAB], [JOY_BUTTON_RIGHT_STICK])
	_ensure_action("rg_cycle_mode", [KEY_V], [JOY_BUTTON_BACK])
	# R5: next driving view (chase, bumper, cockpit, orbit, cinematic). Pad RB
	# (was D-pad right until 2026-10-05); RB is also "free cam up", the two only
	# act in different modes (main.gd consumes the view edge every frame and
	# applies it in drive mode only).
	_ensure_action("rg_cycle_view", [KEY_B], [JOY_BUTTON_RIGHT_SHOULDER])
	_ensure_action("rg_switch_world", [KEY_F8], [])
	_ensure_action("rg_reset_car", [KEY_R], []) # keyboard only: never on the controller (owner 2026-10-05)
	_ensure_action("rg_toggle_nitrous", [], [JOY_BUTTON_Y]) # nitrous arm switch (physics_sim 804137e); no keyboard key
	_ensure_action("rg_npc_truck", [KEY_T], [])
	_ensure_action("rg_flip_upright", [KEY_F], [JOY_BUTTON_DPAD_DOWN])
	# Drone follow: N (next target) / D-pad right - both unused by anything else.
	_ensure_action("rg_cycle_drone_target", [KEY_N], [JOY_BUTTON_DPAD_RIGHT])
	_ensure_action("rg_zoom_in", [KEY_PAGEUP], [])
	_ensure_action("rg_zoom_out", [KEY_PAGEDOWN], [])
	_ensure_action("rg_cam_forward", [KEY_W], [])
	_ensure_action("rg_cam_back", [KEY_S], [])
	_ensure_action("rg_cam_left", [KEY_A], [])
	_ensure_action("rg_cam_right", [KEY_D], [])
	_ensure_action("rg_cam_up", [KEY_E, KEY_SPACE], [JOY_BUTTON_RIGHT_SHOULDER])
	_ensure_action("rg_cam_down", [KEY_Q, KEY_CTRL], [JOY_BUTTON_LEFT_SHOULDER])
	_ensure_action("rg_cam_fast", [KEY_SHIFT], [JOY_BUTTON_LEFT_STICK])
	_ensure_action("rg_look_left", [KEY_LEFT], [])
	_ensure_action("rg_look_right", [KEY_RIGHT], [])
	_ensure_action("rg_look_up", [KEY_UP], [])
	_ensure_action("rg_look_down", [KEY_DOWN], [])

static func _ensure_action(action: String, keys: Array, joy_buttons: Array) -> void:
	if InputMap.has_action(action):
		return
	InputMap.add_action(action)
	for k in keys:
		var ev := InputEventKey.new()
		ev.physical_keycode = k
		InputMap.action_add_event(action, ev)
	for b in joy_buttons:
		var jev := InputEventJoypadButton.new()
		jev.button_index = b
		InputMap.action_add_event(action, jev)

func _input(event: InputEvent) -> void:
	if get_tree().get_nodes_in_group("seat_adjustment_open").size() > 0 or get_tree().get_nodes_in_group("traffic_settings_open").size() > 0:
		_mouse_captured = false
		_mouse_delta = Vector2.ZERO
		return
	if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_RIGHT:
		Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
		_mouse_captured = true
	elif event is InputEventKey and event.pressed and not event.echo and event.keycode == KEY_ESCAPE:
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
		_mouse_captured = false
	elif event is InputEventMouseMotion and _mouse_captured:
		_mouse_delta += event.relative
	elif event is InputEventMouseButton and event.pressed:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			_zoom_steps += event.factor if event.factor > 0.0 else 1.0
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_zoom_steps -= event.factor if event.factor > 0.0 else 1.0

static func _shape_bidirectional(raw: float, deadzone: float) -> float:
	raw = clampf(raw, -1.0, 1.0)
	var mag: float = absf(raw)
	if mag <= deadzone:
		return 0.0
	var shaped: float = clampf((mag - deadzone) / (1.0 - deadzone), 0.0, 1.0)
	return shaped if raw >= 0.0 else -shaped

static func _shape_unidirectional(raw: float, deadzone: float) -> float:
	raw = clampf(raw, 0.0, 1.0)
	if raw <= deadzone:
		return 0.0
	return clampf((raw - deadzone) / (1.0 - deadzone), 0.0, 1.0)

# Rising edge of `held` under `key`.
func _edge(key: String, held: bool) -> bool:
	var was: bool = _prev.get(key, false)
	_prev[key] = held
	return held and not was

func poll() -> void:
	if not get_tree().get_nodes_in_group("address_teleport_open").is_empty():
		_cam_stick = Vector2.ZERO
		_cam_look_rate = Vector2.ZERO
		return # Text entry must not trigger R/F/V or driving/ignition actions.
	var input := Input
	var pads := input.get_connected_joypads()
	var has_pad: bool = not pads.is_empty()
	var device: int = pads[0] if has_pad else joy_device
	var jb := func(b: int) -> bool: return has_pad and input.is_joy_button_pressed(device, b)
	var ja := func(a: int) -> float: return input.get_joy_axis(device, a) if has_pad else 0.0

	# --- driving group ---
	var joy_steer: float = ja.call(JOY_AXIS_LEFT_X)
	var joy_throttle: float = ja.call(JOY_AXIS_TRIGGER_RIGHT)
	var joy_brake: float = ja.call(JOY_AXIS_TRIGGER_LEFT)
	var key_steer: float = input.get_axis("rg_steer_left", "rg_steer_right")
	var key_throttle: float = input.get_action_strength("rg_throttle")
	var key_brake: float = input.get_action_strength("rg_brake")
	_steer_keyboard = absf(key_steer) > absf(joy_steer)
	_steer_wheel = false
	if has_pad:
		var device_name: String = input.get_joy_name(device).to_lower()
		for wheel_name in ["wheel", "g29", "g920", "g923", "t300", "t150", "t248", "fanatec", "moza", "simucube"]:
			if device_name.contains(wheel_name):
				_steer_wheel = true
	var raw_steer: float = key_steer if _steer_keyboard else joy_steer
	var raw_throttle: float = maxf(key_throttle, joy_throttle)
	var raw_brake: float = maxf(key_brake, joy_brake)
	# ISO 8855 sign flip (the "steer" channel: positive = LEFT).
	_steer = -_shape_bidirectional(raw_steer, STEER_DEADZONE)
	_throttle = _shape_unidirectional(raw_throttle, TRIGGER_DEADZONE)
	_brake = _shape_unidirectional(raw_brake, TRIGGER_DEADZONE)
	_handbrake = 1.0 if (jb.call(JOY_BUTTON_A) or input.is_action_pressed("rg_handbrake")) else 0.0
	_clutch = 1.0 if (jb.call(JOY_BUTTON_LEFT_SHOULDER) or input.is_action_pressed("rg_clutch")) else 0.0
	if _edge("shift_up", jb.call(JOY_BUTTON_B) or input.is_action_pressed("rg_shift_up")):
		_shift_up_count += 1
	if _edge("shift_down", jb.call(JOY_BUTTON_X) or input.is_action_pressed("rg_shift_down")):
		_shift_down_count += 1
	if _edge("ignition", jb.call(JOY_BUTTON_DPAD_UP) or input.is_action_pressed("rg_ignition")):
		_ignition_on = not _ignition_on
	_starter_held = jb.call(JOY_BUTTON_START) or input.is_action_pressed("rg_starter")
	if _edge("auto_shift", jb.call(JOY_BUTTON_DPAD_LEFT) or input.is_action_pressed("rg_auto_shift")):
		_auto_shift_on = not _auto_shift_on

	# --- camera group ---
	var move := Vector3(
		input.get_action_strength("rg_cam_right") - input.get_action_strength("rg_cam_left"),
		input.get_action_strength("rg_cam_up") - input.get_action_strength("rg_cam_down"),
		input.get_action_strength("rg_cam_forward") - input.get_action_strength("rg_cam_back"))
	var lx: float = _shape_bidirectional(ja.call(JOY_AXIS_LEFT_X), STICK_DEADZONE)
	var ly: float = _shape_bidirectional(ja.call(JOY_AXIS_LEFT_Y), STICK_DEADZONE)
	if absf(lx) > absf(move.x):
		move.x = lx
	if absf(ly) > absf(move.z):
		move.z = -ly # stick up (-Y) = forward
	var trig_up: float = _shape_unidirectional(ja.call(JOY_AXIS_TRIGGER_RIGHT), TRIGGER_DEADZONE)
	var trig_down: float = _shape_unidirectional(ja.call(JOY_AXIS_TRIGGER_LEFT), TRIGGER_DEADZONE)
	if absf(trig_up - trig_down) > absf(move.y):
		move.y = trig_up - trig_down
	_cam_move = move
	# Camera input can come from a separate pad while a wheel drives the car.
	# Keep the selected pad at rest; change only on actual right-stick activity.
	if not pads.has(_camera_device):
		_camera_device = device if has_pad else -1
	if camera_joy_device >= 0 and pads.has(camera_joy_device):
		_camera_device = camera_joy_device
	else:
		var strongest := 0.2
		for candidate in pads:
			var candidate_stick := Vector2(input.get_joy_axis(candidate, JOY_AXIS_RIGHT_X), -input.get_joy_axis(candidate, JOY_AXIS_RIGHT_Y))
			if candidate_stick.length() > strongest:
				strongest = candidate_stick.length()
				_camera_device = candidate
	_cam_stick = Vector2(input.get_joy_axis(_camera_device, JOY_AXIS_RIGHT_X), -input.get_joy_axis(_camera_device, JOY_AXIS_RIGHT_Y)) if _camera_device >= 0 else Vector2.ZERO
	var devices_signature := str(pads) + ":" + str(_camera_device)
	if devices_signature != _camera_devices_signature:
		_camera_devices_signature = devices_signature
		print("RG_INPUT_CAMERA devices=", pads, " selected=", _camera_device, " name=", input.get_joy_name(_camera_device) if _camera_device >= 0 else "none")
	var look := Vector2(
		input.get_action_strength("rg_look_right") - input.get_action_strength("rg_look_left"),
		input.get_action_strength("rg_look_up") - input.get_action_strength("rg_look_down"))
	_cam_key_look = look
	var rx: float = _shape_bidirectional(_cam_stick.x, STICK_DEADZONE)
	var ry: float = _shape_bidirectional(_cam_stick.y, STICK_DEADZONE)
	if absf(rx) > absf(look.x):
		look.x = rx
	if absf(ry) > absf(look.y):
		look.y = ry
	_cam_look_rate = look
	_cam_fast = input.is_action_pressed("rg_cam_fast")
	_cam_zoom_key = input.get_action_strength("rg_zoom_in") - input.get_action_strength("rg_zoom_out")

	# --- global actions ---
	if _edge("cycle_camera", input.is_action_pressed("rg_cycle_camera")):
		_cycle_camera_count += 1
	if _edge("cycle_mode", input.is_action_pressed("rg_cycle_mode")):
		_cycle_mode_count += 1
	if _edge("cycle_view", input.is_action_pressed("rg_cycle_view")):
		_cycle_view_count += 1
	if _edge("switch_world", input.is_action_pressed("rg_switch_world")):
		_switch_world_count += 1
	if _edge("reset_car", input.is_action_pressed("rg_reset_car")):
		_reset_car_count += 1
	if _edge("toggle_nitrous", input.is_action_pressed("rg_toggle_nitrous")):
		_nitrous_toggle_count += 1
	if _edge("flip_upright", input.is_action_pressed("rg_flip_upright")):
		_flip_upright_count += 1
	if _edge("cycle_drone_target", input.is_action_pressed("rg_cycle_drone_target")):
		_cycle_drone_count += 1
	# --- walking group edges (on foot) ---
	var g_key: bool = input.is_physical_key_pressed(KEY_G)
	if _edge("jump", jb.call(JOY_BUTTON_A) or input.is_physical_key_pressed(KEY_SPACE)):
		_jump_count += 1
	if _edge("interact", jb.call(JOY_BUTTON_X) or g_key):
		_interact_count += 1
	if _edge("get_out_key", g_key):
		_get_out_key_count += 1

# --- driving group getters ---
func get_steer() -> float:
	return _steer

func get_throttle() -> float:
	return _throttle

func get_brake() -> float:
	return _brake

func get_handbrake() -> float:
	return _handbrake

func get_clutch() -> float:
	return _clutch

func get_ignition() -> bool:
	return _ignition_on

func get_starter() -> bool:
	return _starter_held

func get_auto_shift() -> bool:
	return _auto_shift_on

func consume_shift_up_count() -> int:
	var n := _shift_up_count
	_shift_up_count = 0
	return n

func consume_shift_down_count() -> int:
	var n := _shift_down_count
	_shift_down_count = 0
	return n

# --- camera group getters ---
func get_camera_move() -> Vector3:
	return _cam_move

func get_camera_look_rate() -> Vector2:
	return _cam_look_rate

func get_camera_key_look() -> Vector2:
	return _cam_key_look

func get_camera_diagnostics() -> Dictionary:
	return {"device": _camera_device, "name": Input.get_joy_name(_camera_device) if _camera_device >= 0 else "none", "stick_x": _cam_stick.x, "stick_y": _cam_stick.y, "connected": Array(Input.get_connected_joypads())}

func get_camera_stick() -> Vector2:
	return _cam_stick

func get_camera_fast() -> bool:
	return _cam_fast

# PageUp (+1) / PageDown (-1) held: drone-follow zoom in / out.
func get_camera_zoom_key() -> float:
	return _cam_zoom_key

# Mouse-wheel ticks since the last call (+ = wheel up = zoom in).
func consume_zoom_steps() -> float:
	var n := _zoom_steps
	_zoom_steps = 0.0
	return n

# Captured-mouse motion since the last call, in pixels.
func consume_mouse_delta() -> Vector2:
	var d := _mouse_delta
	_mouse_delta = Vector2.ZERO
	return d

# --- walking group getters (on foot) ---
# x = right, y = forward, each -1..1 (the camera group's move, horizontal part).
func get_walk_move() -> Vector2:
	return Vector2(_cam_move.x, _cam_move.z)

func get_walk_run() -> bool:
	return _cam_fast

func consume_jump_count() -> int:
	var n := _jump_count
	_jump_count = 0
	return n

# G or gamepad X: get into the own car when on foot.
func consume_interact_count() -> int:
	var n := _interact_count
	_interact_count = 0
	return n

# The G key alone: get out of the car in drive mode.
func consume_get_out_key_count() -> int:
	var n := _get_out_key_count
	_get_out_key_count = 0
	return n

# --- global actions (edge counts, read-and-reset once per frame) ---
func consume_cycle_mode() -> int:
	var n := _cycle_mode_count
	_cycle_mode_count = 0
	return n

func consume_cycle_view() -> int:
	var n := _cycle_view_count
	_cycle_view_count = 0
	return n

func consume_cycle_drone_target() -> int:
	var n := _cycle_drone_count
	_cycle_drone_count = 0
	return n

func consume_switch_world() -> int:
	var n := _switch_world_count
	_switch_world_count = 0
	return n

func consume_reset_car() -> int:
	var n := _reset_car_count
	_reset_car_count = 0
	return n

# Rising edges of gamepad Y since last consumed (each one flips the nitrous arm switch).
func consume_nitrous_toggle() -> int:
	var n := _nitrous_toggle_count
	_nitrous_toggle_count = 0
	return n

func consume_flip_upright() -> int:
	var n := _flip_upright_count
	_flip_upright_count = 0
	return n

# R0 has no haptics (tach_gauge.gd's RUMBLE lamp reads this).
func get_rumble_enabled() -> bool:
	return false

func steering_uses_keyboard() -> bool:
	return _steer_keyboard

func steering_uses_wheel() -> bool:
	return _steer_wheel and not _steer_keyboard

func consume_cycle_camera() -> int:
	var count := _cycle_camera_count
	_cycle_camera_count = 0
	return count
