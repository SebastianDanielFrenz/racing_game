extends CanvasLayer
# game/scripts/hud.gd — trimmed debug/diagnostic text HUD, adapted from
# physics_sim's own adapters/godot/demo/scripts/hud.gd (read-only reference,
# not a submodule file). Dropped relative to that file, both because R0 has
# neither subsystem at all: terrain tile resident/starved counts (R0's
# ground is one static box, not TileManager-streamed) and the haptics
# per-source debug readout (RUMBLE stays permanently off, input_map.gd's
# get_rumble_enabled() stub). Gear/rpm/speed/assist-lamp status lives in the
# round gauge (tach_gauge.gd, reused verbatim), same split as the reference.
#
# Expects a child Label named "Readout" (created by main.gd when it builds
# the scene) - same contract as the reference file.

@export var simulation_path: NodePath
@export var input_map_path: NodePath
@export var vehicle_name: String = ""

var _simulation: Node
var _input_map: Node
var _label: Label

# Rolling one-second measurement window for the sim's own tick rate,
# independent of Godot's render fps - same technique as the reference file.
var _window_start_s: float = 0.0
var _window_start_ticks: int = 0
var _last_measured_hz: float = 0.0
var _printed_first_tick: bool = false

var _frame_time_sum_us: int = 0
var _frame_time_max_us: int = 0
var _frame_time_count: int = 0
var _last_avg_frame_time_us: float = 0.0
var _last_max_frame_time_us: int = 0

func _ready() -> void:
	_simulation = get_node_or_null(simulation_path)
	_input_map = get_node_or_null(input_map_path)
	_label = get_node("Readout")
	_window_start_s = Time.get_ticks_msec() / 1000.0

func _bar(value01: float, width: int = 20) -> String:
	var v: float = clampf(value01, 0.0, 1.0)
	var filled: int = int(round(v * width))
	return "[" + "#".repeat(filled) + "-".repeat(width - filled) + "]"

func _process(_delta: float) -> void:
	if _simulation == null:
		return

	var now_s := Time.get_ticks_msec() / 1000.0
	var ticks: int = _simulation.get_step_count()

	if not _printed_first_tick and ticks > 0:
		_printed_first_tick = true
		print("rg_godot HUD: sim thread running, first observed tick count: %d (target rate %.1f Hz)" % [ticks, _simulation.get_tick_rate_hz()])

	var frame_time_us: int = _simulation.consume_adapter_frame_time_us()
	_frame_time_sum_us += frame_time_us
	_frame_time_max_us = max(_frame_time_max_us, frame_time_us)
	_frame_time_count += 1

	if now_s - _window_start_s >= 1.0:
		_last_measured_hz = float(ticks - _window_start_ticks) / (now_s - _window_start_s)
		_window_start_s = now_s
		_window_start_ticks = ticks
		_last_avg_frame_time_us = float(_frame_time_sum_us) / max(_frame_time_count, 1)
		_last_max_frame_time_us = _frame_time_max_us
		_frame_time_sum_us = 0
		_frame_time_max_us = 0
		_frame_time_count = 0
		print("rg_godot HUD: measured sim tick rate over last window: %.2f Hz (target %.1f Hz), fps=%.1f, adapter_ms avg=%.3f max=%.3f" % [
			_last_measured_hz, _simulation.get_tick_rate_hz(), Engine.get_frames_per_second(),
			_last_avg_frame_time_us / 1000.0, _last_max_frame_time_us / 1000.0])

	var lines := PackedStringArray()
	lines.append("racing_game R0 - test drive")
	lines.append("sim tick rate: target %.1f Hz  measured %.2f Hz" % [_simulation.get_tick_rate_hz(), _last_measured_hz])
	lines.append("godot fps: %.1f    adapter main-thread: avg %.3f ms  max %.3f ms" % [
		Engine.get_frames_per_second(), _last_avg_frame_time_us / 1000.0, _last_max_frame_time_us / 1000.0])
	lines.append("sim time: %.2f s   ticks: %d   bodies: %d" % [_simulation.get_sim_time(), ticks, _simulation.get_body_count()])
	lines.append("")

	if vehicle_name != "" and _simulation.get_vehicle_names().has(vehicle_name):
		if _input_map != null:
			lines.append("steer     %+.2f %s" % [_input_map.get_steer(), _bar((_input_map.get_steer() + 1.0) * 0.5)])
			lines.append("throttle  %5.2f %s" % [_input_map.get_throttle(), _bar(_input_map.get_throttle())])
			lines.append("brake     %5.2f %s" % [_input_map.get_brake(), _bar(_input_map.get_brake())])
			lines.append("handbrake %5.2f  clutch %5.2f" % [_input_map.get_handbrake(), _input_map.get_clutch()])
		lines.append("")
		var wheel_count: int = _simulation.get_vehicle_wheel_count(vehicle_name)
		lines.append("wheel  load(N)  slip_ratio  slip_angle(deg)  surface")
		for i in range(wheel_count):
			var wname: String = _simulation.get_wheel_name(vehicle_name, i)
			var load_n: float = _simulation.get_wheel_load_n(vehicle_name, i)
			var slip_ratio: float = _simulation.get_wheel_slip_ratio(vehicle_name, i)
			var slip_angle_deg: float = rad_to_deg(_simulation.get_wheel_slip_angle(vehicle_name, i))
			var surface: String = _simulation.get_wheel_surface_name(vehicle_name, i)
			if surface == "":
				surface = "?"
			lines.append("%-5s  %7.0f     %+.3f          %+6.2f          %s" % [wname, load_n, slip_ratio, slip_angle_deg, surface])
	else:
		lines.append("(no vehicle spawned)")

	lines.append("")
	lines.append("controls: gamepad (steer/throttle/brake/handbrake) + WASD/space keyboard fallback")
	lines.append("E/Q or B/X(joy) shift up/down   C or LB(joy) clutch (hold)")
	lines.append("I or D-pad Up ignition (hold)   K or Start(joy) starter (hold)")
	lines.append("Tab hold assist.auto_shift")
	_label.text = "\n".join(lines)
