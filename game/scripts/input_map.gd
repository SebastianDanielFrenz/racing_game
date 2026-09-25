extends Node
# game/scripts/input_map.gd — R0's input layer, a plain GDScript node (NOT a
# C++ GDExtension class like physics_sim's own PsInputMap/input_map_core.h -
# see this repo's CLAUDE.md for that decision and why). R0 has no haptics, no
# steering assist, no camera-orbit/reset/debug-toggle actions and no
# auto-clutch/auto-blip/auto-shift TOGGLE bindings (assist.auto_shift is
# driven by a single held action instead, rg_auto_shift in project.godot) -
# all trimmed relative to physics_sim's fuller demo, which this file's
# gather-raw-input/shape/merge pattern is otherwise a direct port of
# (input_map.cpp::gather_raw_input + input_map_core.cpp::shape_*_axis,
# read in full from physics_sim's adapters/godot/src/ to get the exact
# larger-magnitude-wins merge and deadzone/curve shaping right).
#
# Digital shift_up/shift_down are exposed as per-frame-consumed EDGE COUNTS
# (get_shift_up_count/get_shift_down_count reset to 0 the moment they are
# read), mirroring ControlChannels::shift_up_count/shift_down_count's own
# "count of edges since last consumed" contract (RgSimulation.set_control's
# receiving end, session.cpp's kControlChannelNames) - main.gd must read and
# forward each one exactly once per frame or edges are lost/double counted.

const STEER_DEADZONE := 0.08
const TRIGGER_DEADZONE := 0.02

@export var joy_device: int = 0

var _prev_shift_up_held: bool = false
var _prev_shift_down_held: bool = false
var _shift_up_count: int = 0
var _shift_down_count: int = 0

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

# Cached this frame's shaped channel values (gathered once in _process,
# read by main.gd afterwards - Godot node process order puts this node
# before main.gd's own _process only if the scene wires it that way, so
# main.gd's script instead calls poll() itself once per frame; see
# main.gd's own comment).
var _steer: float = 0.0
var _throttle: float = 0.0
var _brake: float = 0.0
var _handbrake: float = 0.0
var _clutch: float = 0.0
var _ignition_held: bool = false
var _starter_held: bool = false
var _auto_shift_held: bool = false

func poll() -> void:
	var input := Input
	var pads := input.get_connected_joypads()
	var has_pad: bool = not pads.is_empty()
	var device: int = pads[0] if has_pad else joy_device

	var joy_steer: float = input.get_joy_axis(device, JOY_AXIS_LEFT_X) if has_pad else 0.0
	var joy_throttle: float = input.get_joy_axis(device, JOY_AXIS_TRIGGER_RIGHT) if has_pad else 0.0
	var joy_brake: float = input.get_joy_axis(device, JOY_AXIS_TRIGGER_LEFT) if has_pad else 0.0
	var joy_handbrake: bool = has_pad and input.is_joy_button_pressed(device, JOY_BUTTON_A)
	var joy_clutch: bool = has_pad and input.is_joy_button_pressed(device, JOY_BUTTON_LEFT_SHOULDER)

	var key_steer: float = input.get_axis("rg_steer_left", "rg_steer_right")
	var key_throttle: float = input.get_action_strength("rg_throttle")
	var key_brake: float = input.get_action_strength("rg_brake")
	var key_handbrake: bool = input.is_action_pressed("rg_handbrake")
	var key_clutch: bool = input.is_action_pressed("rg_clutch")

	# Larger-magnitude source wins (same merge convention as physics_sim's
	# own input_map.cpp) - a plugged-in-but-idle pad's zeroed axes never
	# override an actively-held key.
	var raw_steer: float = key_steer if absf(key_steer) > absf(joy_steer) else joy_steer
	var raw_throttle: float = key_throttle if key_throttle > joy_throttle else joy_throttle
	var raw_brake: float = key_brake if key_brake > joy_brake else joy_brake

	# ISO 8855 sign flip (this repo's session.cpp/RgSimulation "steer"
	# channel, positive = steer LEFT) applied here, same place physics_sim's
	# input_map_core.cpp applies it - the one spot RawInput's hardware
	# convention (-1 left/+1 right) turns into the sim's own convention.
	_steer = -_shape_bidirectional(raw_steer, STEER_DEADZONE)
	_throttle = _shape_unidirectional(raw_throttle, TRIGGER_DEADZONE)
	_brake = _shape_unidirectional(raw_brake, TRIGGER_DEADZONE)
	_handbrake = 1.0 if (joy_handbrake or key_handbrake) else 0.0
	_clutch = 1.0 if (joy_clutch or key_clutch) else 0.0

	var shift_up_held: bool = (has_pad and input.is_joy_button_pressed(device, JOY_BUTTON_B)) \
		or input.is_action_pressed("rg_shift_up")
	var shift_down_held: bool = (has_pad and input.is_joy_button_pressed(device, JOY_BUTTON_X)) \
		or input.is_action_pressed("rg_shift_down")
	if shift_up_held and not _prev_shift_up_held:
		_shift_up_count += 1
	if shift_down_held and not _prev_shift_down_held:
		_shift_down_count += 1
	_prev_shift_up_held = shift_up_held
	_prev_shift_down_held = shift_down_held

	_ignition_held = (has_pad and input.is_joy_button_pressed(device, JOY_BUTTON_DPAD_UP)) \
		or input.is_action_pressed("rg_ignition")
	_starter_held = (has_pad and input.is_joy_button_pressed(device, JOY_BUTTON_START)) \
		or input.is_action_pressed("rg_starter")
	_auto_shift_held = input.is_action_pressed("rg_auto_shift")

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
	return _ignition_held

func get_starter() -> bool:
	return _starter_held

func get_auto_shift() -> bool:
	return _auto_shift_held

# Consumed (read-and-reset) exactly once per frame by main.gd.
func consume_shift_up_count() -> int:
	var n := _shift_up_count
	_shift_up_count = 0
	return n

func consume_shift_down_count() -> int:
	var n := _shift_down_count
	_shift_down_count = 0
	return n

# R0 has no haptics (tach_gauge.gd's RUMBLE lamp reads this and stays
# permanently off - see hud.gd's own port, game/scripts/hud.gd).
func get_rumble_enabled() -> bool:
	return false
