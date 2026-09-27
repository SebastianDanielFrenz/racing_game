extends CanvasLayer
# game/scripts/hud.gd — debug/diagnostic text HUD (drawing only), adapted from
# physics_sim's adapters/godot/demo/scripts/hud.gd (read-only reference).
# Gear/rpm/speed/assist lamps live in the round gauge (tach_gauge.gd), same
# split as the reference. It reads state through RgSimulation's getters
# (get_mode_state - rg_core's mode framework - and get_streaming_status) and
# the camera director; it decides nothing. A future world-space XR HUD
# replaces this CanvasLayer and reads the same getters.
#
# Expects child Labels "Readout" (top-left text) and "Streaming" (the big
# centred "STREAMING TERRAIN... (n)" line while the terrain gate holds the
# clock), created by main.gd.
#
# tools/smoke_test.ps1 greps the two print() lines below ("sim thread
# running, first observed tick count" and "measured sim tick rate over last
# window") - keep their wording.

const KEY_HELP := "V mode (drive/free cam)  F8 world (flat/real)  R reset car  WASD drive | fly  E/Q shift | up/down  Space handbrake  C clutch  I ignition  K starter  F5 auto-shift  arrows/right stick/RMB+mouse look"

@export var simulation_path: NodePath
@export var input_map_path: NodePath
@export var director_path: NodePath
@export var body_visuals_path: NodePath
@export var vehicle_name: String = ""

var _simulation: Node
var _input_map: Node
var _director: Node
var _body_visuals: Node
var _label: Label
var _streaming_label: Label

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
	_director = get_node_or_null(director_path)
	_body_visuals = get_node_or_null(body_visuals_path)
	_label = get_node("Readout")
	_streaming_label = get_node_or_null("Streaming")
	_window_start_s = Time.get_ticks_msec() / 1000.0

func _bar(value01: float, width: int = 20) -> String:
	var v: float = clampf(value01, 0.0, 1.0)
	var filled: int = int(round(v * width))
	return "[" + "#".repeat(filled) + "-".repeat(width - filled) + "]"

# A world switch builds a new Session whose tick count restarts at 0.
func reset_tick_window() -> void:
	_window_start_s = Time.get_ticks_msec() / 1000.0
	_window_start_ticks = 0

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

	var mode: Dictionary = _simulation.get_mode_state()
	var ss: Dictionary = _simulation.get_streaming_status()
	var terrain_mode: bool = bool(ss.get("terrain_mode", false))

	if _streaming_label != null:
		var frozen: bool = terrain_mode and bool(ss.get("frozen", false))
		_streaming_label.visible = frozen
		if frozen:
			_streaming_label.text = "STREAMING TERRAIN... (%d)" % int(ss.get("missing_required", 0))

	var lines := PackedStringArray()
	lines.append("racing_game R9 - mode: %s   world: %s (%s)   car: %s" % [
		mode.get("mode", "?"), mode.get("world_kind", "?"), mode.get("world_phase", "?"), mode.get("vehicle_control", "?")])
	lines.append("sim tick rate: target %.1f Hz  measured %.2f Hz" % [_simulation.get_tick_rate_hz(), _last_measured_hz])
	lines.append("godot fps: %.1f    adapter main-thread: avg %.3f ms  max %.3f ms" % [
		Engine.get_frames_per_second(), _last_avg_frame_time_us / 1000.0, _last_max_frame_time_us / 1000.0])
	lines.append("sim time: %.2f s   ticks: %d   speed: %.1f km/h" % [
		_simulation.get_sim_time(), ticks, _simulation.get_body_speed_mps("chassis") * 3.6])
	if _body_visuals != null and not bool(_body_visuals.vehicle_model_ok()):
		lines.append("WARNING: car_sedan.glb failed to load - showing placeholder box (see console)")
	if terrain_mode:
		var chunks: int = 0
		var rebases: int = 0
		if _director != null:
			rebases = int(_director.rebase_count)
			if _director.terrain_view != null:
				chunks = int(_director.terrain_view.get_chunk_count())
		lines.append("terrain: L0 %d  missing %d  inflight %d  failed %d  tiles %d  starved %d | freezes %d  frozen ticks %d | falls %d  misses %d | resets %d | chunks %d  rebases %d" % [
			int(ss.get("resident_l0", 0)), int(ss.get("missing_required", 0)), int(ss.get("inflight", 0)),
			int(ss.get("failed", 0)), int(ss.get("resident_tiles", 0)), int(ss.get("starved_tiles", 0)),
			int(ss.get("freeze_count", 0)), int(ss.get("frozen_attempts", 0)), int(ss.get("falls", 0)),
			int(ss.get("fill_misses", 0)), int(ss.get("relocations", 0)), chunks, rebases])
		# G2.5a-grip R-c: per-cell OSM road grip vs. one uniform terrain_surface.
		if bool(ss.get("road_surfaces", false)):
			lines.append("grip: roads (osm_ok=%d osm_fail=%d)" % [int(ss.get("osm_ok", 0)), int(ss.get("osm_fail", 0))])
		else:
			lines.append("grip: uniform %s" % _simulation.get_terrain_surface_name())
	lines.append("")

	if vehicle_name != "" and _simulation.get_vehicle_names().has(vehicle_name):
		if _input_map != null and bool(mode.get("driving_inputs_live", false)):
			# Read back the value actually sent to the sim THIS frame
			# (RgSimulation.get_control, main.gd's own set_control channel
			# names) rather than input_map.gd's raw device reading: while
			# main.scripted_controls drives the car (drive_smoke.gd/
			# drive_tour.gd), main.gd._forward_driving sends the scripted
			# values instead of input_map's, and input_map's own getters would
			# then read 0/stale here even though a different value is being
			# driven into the sim - see carvis brief 2026-09-27's steering
			# proof shot fix.
			var steer: float = _simulation.get_control("steer")
			var throttle: float = _simulation.get_control("throttle")
			var brake: float = _simulation.get_control("brake")
			var handbrake: float = _simulation.get_control("handbrake")
			var clutch: float = _simulation.get_control("clutch")
			lines.append("steer     %+.2f %s" % [steer, _bar((steer + 1.0) * 0.5)])
			lines.append("throttle  %5.2f %s" % [throttle, _bar(throttle)])
			lines.append("brake     %5.2f %s" % [brake, _bar(brake)])
			lines.append("handbrake %5.2f  clutch %5.2f   ignition %s  auto-shift %s" % [
				handbrake, clutch,
				"on" if _input_map.get_ignition() else "off", "on" if _input_map.get_auto_shift() else "off"])
		else:
			lines.append("car unattended: brakes held, clutch pressed")
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
	lines.append(KEY_HELP)
	_label.text = "\n".join(lines)
