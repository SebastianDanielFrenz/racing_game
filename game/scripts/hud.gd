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

const KEY_HELP := "F7 traffic  F6 seat adjustment  Middle-click log location  Tab/right-stick click chase/cockpit (first/third person on foot)  B/RB next view (chase, bumper, cockpit, orbit, cinematic)  Esc/P pause  V/Back mode (drive/free cam/drone follow/on foot)  G get out (car below 2 m/s) | get in at the door (on foot: WASD move, Shift run, Space jump, G or pad X get in)  N or D-pad right next drone target  wheel/PgUp/PgDn drone zoom  F8 world (flat/real)  R reset car (keyboard)  Y(pad) nitrous arm  F flip upright  WASD drive | fly  E/Q shift | up/down  Space handbrake  C clutch  I ignition  K starter  F5 auto-shift  arrows/right stick/RMB+mouse look"

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
var _prompt_label: Label
var _message_text: String = ""
var _message_until_ms: int = 0

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
	# Bottom-centre prompt line (on-foot get-in prompt and short refusals).
	_prompt_label = Label.new()
	_prompt_label.name = "Prompt"
	_prompt_label.set_anchors_preset(Control.PRESET_CENTER_BOTTOM)
	_prompt_label.grow_horizontal = Control.GROW_DIRECTION_BOTH
	_prompt_label.grow_vertical = Control.GROW_DIRECTION_BEGIN
	_prompt_label.position.y = -90
	_prompt_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_prompt_label.add_theme_font_size_override("font_size", 26)
	_prompt_label.add_theme_color_override("font_outline_color", Color.BLACK)
	_prompt_label.add_theme_constant_override("outline_size", 6)
	_prompt_label.visible = false
	add_child(_prompt_label)
	_window_start_s = Time.get_ticks_msec() / 1000.0

func _bar(value01: float, width: int = 20) -> String:
	var v: float = clampf(value01, 0.0, 1.0)
	var filled: int = int(round(v * width))
	return "[" + "#".repeat(filled) + "-".repeat(width - filled) + "]"

# A short message on the prompt line (e.g. a refused get-out), for `seconds`.
func show_message(text: String, seconds: float = 3.0) -> void:
	_message_text = text
	_message_until_ms = Time.get_ticks_msec() + int(seconds * 1000.0)

# The prompt line: a pending message wins, else the on-foot get-in prompt.
func _update_prompt(mode: Dictionary) -> void:
	if _prompt_label == null:
		return
	var text := ""
	if Time.get_ticks_msec() < _message_until_ms:
		text = _message_text
	elif str(mode.get("mode", "")) == "on_foot":
		var ws: Dictionary = _simulation.get_walker_state()
		if ws.is_empty():
			text = "Getting out..."
		elif bool(ws.get("can_enter", false)):
			text = "G / X: get into the car"
		else:
			text = "On foot - car %.0f m away (get within 1.5 m of it to get in)" % maxf(float(ws.get("enter_distance_m", 0.0)), 0.0)
	_prompt_label.text = text
	_prompt_label.visible = text != ""

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
		var rd: Dictionary = _simulation.get_render_diagnostics() # nominal-clock pose sampling counters (cumulative)
		print("rg_godot HUD: measured sim tick rate over last window: %.2f Hz (target %.1f Hz), fps=%.1f, adapter_ms avg=%.3f max=%.3f, render sampled=%d late=%d early=%d stepped=%d" % [
			_last_measured_hz, _simulation.get_tick_rate_hz(), Engine.get_frames_per_second(),
			_last_avg_frame_time_us / 1000.0, _last_max_frame_time_us / 1000.0,
			rd.get("frames_sampled", 0), rd.get("frames_late", 0), rd.get("frames_early", 0), rd.get("frames_stepped", 0)])

	var mode: Dictionary = _simulation.get_mode_state()
	var ss: Dictionary = _simulation.get_streaming_status()
	var terrain_mode: bool = bool(ss.get("terrain_mode", false))

	if _streaming_label != null:
		var frozen: bool = terrain_mode and bool(ss.get("frozen", false))
		_streaming_label.visible = frozen
		if frozen:
			_streaming_label.text = "STREAMING TERRAIN... (%d)" % int(ss.get("missing_required", 0))

	_update_prompt(mode)

	var lines := PackedStringArray()
	lines.append("racing_game R9 - mode: %s   world: %s (%s)   car: %s" % [
		mode.get("mode", "?"), mode.get("world_kind", "?"), mode.get("world_phase", "?"), mode.get("vehicle_control", "?")])
	if str(mode.get("mode", "")) == "on_foot":
		var foot: Dictionary = _simulation.get_walker_state()
		if not foot.is_empty():
			lines.append("on foot: speed %.1f m/s  %s%s  car %.1f m away" % [
				Vector2(foot["velocity"].x, foot["velocity"].z).length(),
				"grounded" if bool(foot.get("grounded", false)) else "airborne",
				"  (no ground loaded: holding)" if bool(foot.get("hold", false)) else "",
				float(foot.get("enter_distance_m", 0.0))])
	if str(mode.get("mode", "")) == "drone_follow":
		lines.append("drone target: %s   (N / D-pad right: next target, wheel / PgUp / PgDn: zoom)" % mode.get("drone_target_label", "own car"))
	lines.append(_simulation.get_build_info())
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
		var aero: Dictionary = _simulation.get_aero_state()
		if bool(aero.get("enabled",false)):
			lines.append("aero: air %.0f km/h | drag %.0f N | downforce %.0f N | front %.0f%% | wing %+.1f deg | fan %.1f kW" % [float(aero.airspeed_m_s)*3.6,float(aero.drag_n),float(aero.downforce_n),float(aero.front_balance)*100.0,float(aero.wing_pitch_offset_deg),float(aero.fan_power_w)/1000.0])
		# Nitrous (physics_sim N2O): only a car with a kit (car_sedan_gen_n2o) has this line.
		var n2o: Dictionary = _simulation.get_vehicle_nitrous(vehicle_name)
		if bool(n2o.get("present", false)):
			var n2o_state: String = str(n2o.get("state", "off"))
			var n2o_line := "nitrous %s (N key / pad Y: arm)   bottle %.2f/%.2f kg  %.0f bar   N2O %.0f g/s  kit fuel %.0f g/s" % [
				n2o_state, float(n2o.get("bottle_kg", 0.0)), float(n2o.get("capacity_kg", 0.0)), float(n2o.get("bottle_bar", 0.0)),
				float(n2o.get("flow_g_s", 0.0)), float(n2o.get("kit_fuel_g_s", 0.0))]
			if bool(n2o.get("spraying", false)):
				n2o_line += "   charge %.0f K  lambda %.2f  retard %.1f deg" % [
					float(n2o.get("charge_temperature_k", 0.0)), float(n2o.get("lambda_combined", 0.0)), float(n2o.get("retard_deg", 0.0))]
			if str(n2o.get("cut_reason", "")) != "":
				n2o_line += "   safety cut: %s" % str(n2o.get("cut_reason", ""))
			lines.append(n2o_line)
		var truck: Dictionary = _simulation.get_npc_truck_state()
		lines.append("T: truck ahead | Shift+T: remove | %s | %.0f km/h" % [str(truck.get("message", "")), float(truck.get("speed_kph", 0))])
		if bool(truck.get("active", false)):
			lines.append("Draft: body dynamic pressure %.0f%% of undisturbed airflow" % (float(aero.get("wake_factor", 1.0)) * 100.0))
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
