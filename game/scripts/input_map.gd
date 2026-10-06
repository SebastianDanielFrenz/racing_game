extends Node
# game/scripts/input_map.gd — the input layer (a plain GDScript node, not a
# C++ class like physics_sim's PsInputMap - see this repo's CLAUDE.md). It
# only READS devices; WHAT an input means is the player's configuration, owned
# by rg_core's rg::Controls (PLAN.md R5b, through the RgControls node main.gd
# creates and loads BEFORE this node exists): every frame poll() gathers a raw
# snapshot - the physical keys any binding uses, the mouse buttons, the wheel
# ticks and mouse motion collected by _input, every connected pad's buttons and
# axes - hands it to RgControls.evaluate() and keeps the resulting action
# values behind the getters below, which are the same getters the game had
# before the controls menu (main.gd, the camera rigs, hud and gauge read them
# unchanged). Devices are identified by the engine's joypad slot here and
# mapped to profiles (SDL GUID + ordinal) by rg::Controls; the hot-plug signal
# below tells it who comes and goes.
#
# main.gd forwards the groups the mode framework says are live
# (rg_core's rg::PlayerModeMachine::effective_rules via RgSimulation.
# get_mode_state): the DRIVING group into RgSimulation.set_control, the
# CAMERA group into the active camera rig. A key shared by both groups (W/S,
# E/Q) therefore means one thing at a time. A future XR input map replaces
# this node with the same getters.
#
# Driving group (every device at once, larger magnitude wins - physics_sim's
# input_map.cpp merge convention): steer/throttle/brake/handbrake/clutch,
# shift up/down edge counts, ignition (TOGGLE, starts on), starter (held),
# assist.auto_shift (TOGGLE, starts off for manual cars).
# Camera group: move (x right, y up, z forward, each -1..1), look rate
# (yaw/pitch, -1..1: right stick or arrow keys), mouse look (captured mouse
# delta in pixels; the "mouse_capture" binding - the right button by default -
# captures, Esc releases), fast. Drone-follow zoom (camera group): mouse wheel
# ticks (consume_zoom_steps) and the zoom bindings held (get_camera_zoom_key).
# Walking group (on foot, R9c): reuses the camera group's move axes and "fast" as
# RUN; adds JUMP and INTERACT edge counts. The get-out key (G by default) is the
# separate "get_out" action: edge count of that action only (the pad has none by
# default: its X is shift-down while driving).
# Global actions (edges, consumed once per frame by main.gd): cycle mode, cycle
# driving view, switch world, reset car, next drone target, truck ahead.
#
# shift_up/shift_down are per-frame-consumed EDGE COUNTS, mirroring the
# shift_up_count/shift_down_count channels' "edges since last consumed"
# contract (session.cpp's kControlChannelNames) - main.gd reads each once
# per frame.
#
# Menu navigation: the engine's ui_accept / ui_cancel are rebuilt from the
# controls' menu_confirm / menu_back actions (Enter, Esc, pad A / B always
# included) whenever the bindings change.

var controls: Node # RgControls, set by main.gd before add_child
@export var joy_device: int = 0

# True while a screen that reads devices itself (the controls screen) is up: poll()
# then drops every edge so a key pressed to bind it does not act later in the world.
var suspended: bool = false

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
var _npc_truck_count: int = 0
var _zoom_steps: float = 0.0 # mouse-wheel ticks since last consumed, + = zoom in
var _cam_zoom_key: float = 0.0 # zoom-in binding (+1) / zoom-out binding (-1) held

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
var _cam_stick: Vector2 = Vector2.ZERO # raw pad look stick (chase orbit), x right, y up
var _cam_fast: bool = false
var _mouse_delta: Vector2 = Vector2.ZERO
var _mouse_captured: bool = false

# Raw input collected between two polls by _input.
var _wheel_up: float = 0.0
var _wheel_down: float = 0.0
var _wheel_left: float = 0.0
var _wheel_right: float = 0.0
var _motion: Vector2 = Vector2.ZERO

var _key_codes: Dictionary = {} # key name -> physical keycode (cache, built from the polled names)
var _polled_names: PackedStringArray = PackedStringArray()
var _polled_codes: PackedInt32Array = PackedInt32Array()
var _polled_revision: int = -1
var _wheel_connected: bool = false
var _test_snapshot: Dictionary = {}
var _use_test_snapshot: bool = false

const POLLED_PAD_BUTTONS := 48
const POLLED_PAD_AXES := 10
const POLLED_MOUSE_BUTTONS := [1, 2, 3, 8, 9]

# Gives the mouse back (pause menu opened with the keyboard while it was
# captured for looking around).
func release_mouse() -> void:
	Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
	_mouse_captured = false
	_mouse_delta = Vector2.ZERO
	_motion = Vector2.ZERO

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
	if controls == null:
		push_error("input_map.gd: no RgControls (main.gd must set `controls` before add_child)")
		return
	Input.joy_connection_changed.connect(_on_joy_connection_changed)
	for device in Input.get_connected_joypads():
		_register_pad(device)
	sync_menu_actions()

# --- devices -------------------------------------------------------------------

func _register_pad(device: int) -> void:
	var info: Dictionary = Input.get_joy_info(device)
	var key: String = controls.joypad_connected(device, Input.get_joy_guid(device), Input.get_joy_name(device),
		int(info.get("vendor_id", 0)), int(info.get("product_id", 0)), Input.is_joy_known(device))
	print("RG_INPUT pad connected slot=%d name=%s guid=%s key=%s" % [device, Input.get_joy_name(device), Input.get_joy_guid(device), key])
	_refresh_device_flags()

func _on_joy_connection_changed(device: int, connected: bool) -> void:
	if controls == null:
		return
	if connected:
		_register_pad(device)
	else:
		controls.joypad_disconnected(device)
		print("RG_INPUT pad disconnected slot=%d" % device)
		_refresh_device_flags()

func _refresh_device_flags() -> void:
	_wheel_connected = false
	for d in controls.get_devices():
		if bool(d["connected"]) and str(d["class"]) == "wheel":
			_wheel_connected = true

# ui_accept / ui_cancel follow the menu_confirm / menu_back bindings (the built-in
# Enter / Esc / pad A / B aliases are always part of them).
func sync_menu_actions() -> void:
	_sync_ui_action("ui_accept", "menu_confirm")
	_sync_ui_action("ui_cancel", "menu_back")

func _sync_ui_action(ui_action: String, controls_action: String) -> void:
	if not InputMap.has_action(ui_action):
		return
	InputMap.action_erase_events(ui_action)
	for b in controls.get_menu_bindings(controls_action):
		if str(b["type"]) == "key":
			var code: int = OS.find_keycode_from_string(str(b["key"]))
			if code == 0:
				continue
			var ev := InputEventKey.new()
			ev.physical_keycode = code as Key
			InputMap.action_add_event(ui_action, ev)
		elif str(b["type"]) == "joy_button":
			var jev := InputEventJoypadButton.new()
			jev.button_index = int(b["index"]) as JoyButton
			InputMap.action_add_event(ui_action, jev)

func _input(event: InputEvent) -> void:
	if controls == null:
		return
	if get_tree().get_nodes_in_group("seat_adjustment_open").size() > 0 or get_tree().get_nodes_in_group("traffic_settings_open").size() > 0:
		_mouse_captured = false
		_motion = Vector2.ZERO
		return
	if event is InputEventMouseButton and event.pressed and controls.is_mouse_button_bound("mouse_capture", event.button_index):
		Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
		_mouse_captured = true
	elif (event is InputEventKey and event.pressed and not event.echo and event.keycode == KEY_ESCAPE) 			or (event is InputEventJoypadButton and event.pressed and event.button_index == JOY_BUTTON_START):
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
		_mouse_captured = false
	elif event is InputEventMouseMotion and _mouse_captured:
		_motion += event.relative
	elif event is InputEventMouseButton and event.pressed:
		var factor: float = event.factor if event.factor > 0.0 else 1.0
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			_wheel_up += factor
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_wheel_down += factor
		elif event.button_index == MOUSE_BUTTON_WHEEL_LEFT:
			_wheel_left += factor
		elif event.button_index == MOUSE_BUTTON_WHEEL_RIGHT:
			_wheel_right += factor

# Rising edge of `held` under `key`.
func _edge(key: String, held: bool) -> bool:
	var was: bool = _prev.get(key, false)
	_prev[key] = held
	return held and not was

func _refresh_polled_keys() -> void:
	var revision: int = controls.get_revision()
	if revision == _polled_revision:
		return
	_polled_revision = revision
	_polled_names = controls.get_polled_keys()
	_polled_codes = PackedInt32Array()
	for key_name in _polled_names:
		if not _key_codes.has(key_name):
			var code: int = OS.find_keycode_from_string(key_name)
			if code == 0:
				push_warning("input_map.gd: unknown key name '%s' in the controls" % key_name)
			_key_codes[key_name] = code
		_polled_codes.append(int(_key_codes[key_name]))
	sync_menu_actions()

# The raw snapshot rg::Controls evaluates (see RgControls for the format).
func _gather_snapshot() -> Dictionary:
	_refresh_polled_keys()
	var keys := PackedStringArray()
	for i in range(_polled_names.size()):
		var code: int = _polled_codes[i]
		if code != 0 and Input.is_physical_key_pressed(code as Key):
			keys.append(_polled_names[i])
	var buttons := PackedInt32Array()
	for b in POLLED_MOUSE_BUTTONS:
		if Input.is_mouse_button_pressed(b as MouseButton):
			buttons.append(b)
	var pads := []
	for device in Input.get_connected_joypads():
		var pressed := PackedInt32Array()
		for b in range(POLLED_PAD_BUTTONS):
			if Input.is_joy_button_pressed(device, b as JoyButton):
				pressed.append(b)
		var axes := PackedFloat32Array()
		for a in range(POLLED_PAD_AXES):
			axes.append(Input.get_joy_axis(device, a as JoyAxis))
		pads.append({"slot": device, "buttons": pressed, "axes": axes})
	var snapshot := {
		"keys": keys, "mouse_buttons": buttons, "pads": pads,
		"mouse_dx": _motion.x, "mouse_dy": _motion.y,
		"wheel_up": _wheel_up, "wheel_down": _wheel_down, "wheel_left": _wheel_left, "wheel_right": _wheel_right,
	}
	_motion = Vector2.ZERO
	_wheel_up = 0.0
	_wheel_down = 0.0
	_wheel_left = 0.0
	_wheel_right = 0.0
	return snapshot

# Test hook (tools/smoke_test.ps1 -Controls): while set, poll() evaluates this
# snapshot instead of the real devices ({} = the real devices again).
func set_test_snapshot(snapshot: Dictionary) -> void:
	_test_snapshot = snapshot
	_use_test_snapshot = not snapshot.is_empty()

func _clear_edges() -> void:
	_shift_up_count = 0
	_shift_down_count = 0
	_cycle_camera_count = 0
	_cycle_mode_count = 0
	_cycle_view_count = 0
	_switch_world_count = 0
	_reset_car_count = 0
	_nitrous_toggle_count = 0
	_flip_upright_count = 0
	_cycle_drone_count = 0
	_jump_count = 0
	_interact_count = 0
	_get_out_key_count = 0
	_npc_truck_count = 0
	_zoom_steps = 0.0
	_mouse_delta = Vector2.ZERO

func poll() -> void:
	if controls == null:
		return
	if suspended:
		_gather_snapshot() # drain the wheel / motion accumulators
		_clear_edges()
		return
	if not get_tree().get_nodes_in_group("address_teleport_open").is_empty():
		_cam_stick = Vector2.ZERO
		_cam_look_rate = Vector2.ZERO
		return # Text entry must not trigger R/F/V or driving/ignition actions.
	var snapshot: Dictionary = _test_snapshot if _use_test_snapshot else _gather_snapshot()
	controls.evaluate(snapshot)
	var v := func(action: String) -> float: return controls.get_value(action)
	var held := func(action: String) -> bool: return controls.get_value(action) > 0.5

	# --- driving group ---
	var steer_kb: float = controls.get_keyboard_value("steer")
	var steer_pad: float = controls.get_pad_value("steer")
	_steer_keyboard = absf(steer_kb) > absf(steer_pad)
	_steer_wheel = _wheel_connected or controls.get_source("steer") == 3 # DeviceClass::Wheel
	# ISO 8855 sign flip (the "steer" channel: positive = LEFT).
	_steer = -v.call("steer")
	_throttle = v.call("throttle")
	_brake = v.call("brake")
	_handbrake = v.call("handbrake")
	_clutch = v.call("clutch")
	if _edge("shift_up", held.call("shift_up")):
		_shift_up_count += 1
	if _edge("shift_down", held.call("shift_down")):
		_shift_down_count += 1
	if _edge("ignition", held.call("ignition")):
		_ignition_on = not _ignition_on
	_starter_held = held.call("starter")
	if _edge("auto_shift", held.call("auto_shift")):
		_auto_shift_on = not _auto_shift_on

	# --- camera group ---
	_cam_move = Vector3(v.call("cam_move_x"), v.call("cam_move_y"), v.call("cam_move_z"))
	_cam_key_look = Vector2(controls.get_keyboard_value("cam_look_x"), controls.get_keyboard_value("cam_look_y"))
	_cam_stick = Vector2(controls.get_pad_raw("cam_look_x"), controls.get_pad_raw("cam_look_y"))
	_cam_look_rate = Vector2(v.call("cam_look_x"), v.call("cam_look_y"))
	_cam_fast = held.call("cam_fast")
	_cam_zoom_key = v.call("cam_zoom_in") - v.call("cam_zoom_out")
	_zoom_steps += controls.get_pulses("cam_zoom_in") - controls.get_pulses("cam_zoom_out")
	_mouse_delta += Vector2(v.call("mouse_look_x"), v.call("mouse_look_y"))

	# --- global actions ---
	if _edge("cycle_camera", held.call("cycle_camera")):
		_cycle_camera_count += 1
	if _edge("cycle_mode", held.call("cycle_mode")):
		_cycle_mode_count += 1
	if _edge("cycle_view", held.call("cycle_view")):
		_cycle_view_count += 1
	if _edge("switch_world", held.call("switch_world")):
		_switch_world_count += 1
	if _edge("reset_car", held.call("reset_car")):
		_reset_car_count += 1
	if _edge("toggle_nitrous", held.call("toggle_nitrous")):
		_nitrous_toggle_count += 1
	if _edge("flip_upright", held.call("flip_upright")):
		_flip_upright_count += 1
	if _edge("cycle_drone_target", held.call("cycle_drone_target")):
		_cycle_drone_count += 1
	if _edge("npc_truck", held.call("npc_truck")):
		_npc_truck_count += 1
	# --- walking group edges (on foot) ---
	if _edge("jump", held.call("jump")):
		_jump_count += 1
	if _edge("interact", held.call("interact")):
		_interact_count += 1
	if _edge("get_out", held.call("get_out")):
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
	return {"device": -1, "name": "controls", "stick_x": _cam_stick.x, "stick_y": _cam_stick.y, "connected": Array(Input.get_connected_joypads())}

func get_camera_stick() -> Vector2:
	return _cam_stick

func get_camera_fast() -> bool:
	return _cam_fast

# Zoom-in (+1) / zoom-out (-1) bindings held (PageUp / PageDown by default): drone-follow zoom.
func get_camera_zoom_key() -> float:
	return _cam_zoom_key

# Mouse-wheel ticks since the last call (+ = zoom in, by default wheel up).
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

# The "interact" binding (G / gamepad X by default): get into the own car when on foot.
func consume_interact_count() -> int:
	var n := _interact_count
	_interact_count = 0
	return n

# The "get_out" binding (G by default): get out of the car in drive mode.
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

func consume_npc_truck() -> int:
	var n := _npc_truck_count
	_npc_truck_count = 0
	return n

# Rising edges of the nitrous binding (gamepad Y by default) since last consumed (each one flips the nitrous arm switch).
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
