extends Control
# game/scripts/tach_gauge.gd — driver HUD tachometer gauge, reused
# near-verbatim from physics_sim's own adapters/godot/demo/scripts/
# tach_gauge.gd (read-only reference, not a submodule file). Depends only on
# RgSimulation.get_vehicle_names/get_vehicle_gauge_info/get_vehicle_
# powertrain/get_vehicle_ground_speed_mps, all implemented with the same
# names/Dictionary-key contract as physics_sim's own PsSimulation (see
# godot_ext/src/rg_simulation.cpp), so no logic change was needed beyond
# this header comment. Electronic-throttle vehicles expose the live TC lamp;
# other assist slots remain hidden until their controllers are fitted.
#
# All the non-rendering math (tick/label layout, needle angle, max-rpm
# rounding, lamp state, needle smoothing) lives in gauge_logic.gd (copied
# verbatim - see that file's own header) as plain static functions with no
# Godot Control/_draw() dependency, so it ports to UE5 UMG/C++ untouched
# (this repo's CLAUDE.md engine-neutral-logic rule); only this file is
# Godot-specific.

@export var simulation_path: NodePath
@export var input_map_path: NodePath
@export var vehicle_name: String = ""

# Explicit preload (not a bare "GaugeLogic" reference) - see the reference
# file's own comment: this project also builds its scene procedurally at
# runtime, before Godot has necessarily rescanned res:// for global
# class_name declarations.
const GaugeLogic := preload("res://scripts/gauge_logic.gd")

var _simulation: Node
# True speed over ground (|linear velocity|, world frame), drawn small above
# the indicated speed so the speedo's gearbox-output-derived value can be
# compared against it - same testing aid as the reference.
var _gps_kmh: float = 0.0
var _input_map: Node

var _gauge_info: Dictionary = {}
var _last_pt: Dictionary = {}
var _speed_limit: Dictionary = {}

var _display_rpm: float = 0.0
const NEEDLE_SMOOTH_RATE_PER_S := 10.0

const COLOR_BG := Color(0.05, 0.05, 0.07, 0.55)
const COLOR_TICK_MAJOR := Color(0.92, 0.92, 0.95, 0.95)
const COLOR_TICK_MINOR := Color(0.6, 0.6, 0.66, 0.7)
const COLOR_REDLINE := Color(0.85, 0.15, 0.1, 0.9)
const COLOR_NEEDLE := Color(1.0, 1.0, 1.0, 0.95)
const COLOR_NEEDLE_REDLINE := Color(1.0, 0.3, 0.15, 1.0)
const COLOR_GEAR_BG := Color(0.08, 0.08, 0.1, 0.85)
const COLOR_GEAR_TEXT := Color(0.95, 0.95, 1.0, 1.0)
const COLOR_GEAR_STALLED := Color(0.6, 0.15, 0.15, 1.0)
const COLOR_GEAR_GRIND_FLASH := Color(1.0, 0.7, 0.0, 1.0)
const COLOR_LAMP_OFF := Color(0.4, 0.4, 0.44, 0.85)
const COLOR_LAMP_ON := Color(0.25, 0.9, 0.35, 1.0)
const COLOR_LAMP_INTERVENING := Color(1.0, 0.75, 0.0, 1.0)
const COLOR_SPEED_TEXT := Color(0.95, 0.95, 1.0, 1.0)

const SYSTEM_LAMP_NAMES := ["ABS", "TC", "ESP", "LC", "HILL"]
const SYSTEM_LAMP_SLOTS_CLOCK_DEG := [250.0, 322.0, 34.0, 106.0, 178.0]

# R6: a different car (garage): the gauge info is read again for it.
func set_vehicle_name(name_in: String) -> void:
	vehicle_name = name_in
	_gauge_info = {}

func _ready() -> void:
	set_anchors_preset(Control.PRESET_FULL_RECT)
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_simulation = get_node_or_null(simulation_path)
	_input_map = get_node_or_null(input_map_path)

func _process(delta: float) -> void:
	if _simulation != null and vehicle_name != "" and _simulation.get_vehicle_names().has(vehicle_name):
		if _gauge_info.is_empty():
			_gauge_info = _simulation.get_vehicle_gauge_info(vehicle_name)
		_last_pt = _simulation.get_vehicle_powertrain(vehicle_name)
		_speed_limit = _simulation.get_vehicle_speed_limit(vehicle_name)
		_gps_kmh = _simulation.get_vehicle_ground_speed_mps(vehicle_name) * 3.6
		var target_rpm: float = _last_pt.get("rpm", 0.0)
		_display_rpm = GaugeLogic.smooth_towards(_display_rpm, target_rpm, NEEDLE_SMOOTH_RATE_PER_S, delta)
	else:
		_last_pt = {}
		_speed_limit = {}
	queue_redraw()

func _clock_rad(clock_deg: float) -> float:
	return deg_to_rad(clock_deg - 90.0)

func _polar(center: Vector2, radius: float, clock_deg: float) -> Vector2:
	var rad: float = _clock_rad(clock_deg)
	return center + radius * Vector2(cos(rad), sin(rad))

func _draw_centered_text(font: Font, pos: Vector2, text: String, font_size: int, color: Color) -> void:
	var text_size: Vector2 = font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size)
	draw_string(font, pos - Vector2(text_size.x * 0.5, -text_size.y * 0.35), text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, color)

func _draw_lamp(font: Font, pos: Vector2, label: String, state: int, dot_radius: float, font_size: int) -> void:
	var color: Color
	if state == GaugeLogic.LampState.ON:
		color = COLOR_LAMP_ON
	elif state == GaugeLogic.LampState.INTERVENING:
		var blink: bool = fmod(Time.get_ticks_msec() / 1000.0, 0.4) < 0.2
		color = COLOR_LAMP_INTERVENING if blink else COLOR_LAMP_OFF
	else:
		color = COLOR_LAMP_OFF
	draw_circle(pos, dot_radius, color)
	_draw_centered_text(font, pos + Vector2(0, dot_radius + font_size * 0.9), label, font_size, color)

# R0 has no haptics (input_map.gd.get_rumble_enabled() always returns
# false), so the RUMBLE lamp below is always off but still drawn - same
# "one table, lamp name -> field" pattern as the reference, minus its
# now-unused fitted-per-vehicle-assist plumbing (R0's gauge_info never
# reports any).
func _existing_assist_lamp_states() -> Array:
	var rumble_on: bool = _input_map != null and bool(_input_map.get_rumble_enabled())
	var table := [
		{"name": "CLUTCH", "active": bool(_last_pt.get("assist_auto_clutch", false))},
		{"name": "BLIP", "active": bool(_last_pt.get("assist_auto_blip", false))},
		{"name": "SHIFT", "active": bool(_last_pt.get("assist_auto_shift", false))},
		{"name": "RUMBLE", "active": rumble_on},
	]
	var out := []
	for row in table:
		out.append({"name": row["name"], "state": GaugeLogic.lamp_state(true, row["active"], false)})
	return out

func _draw() -> void:
	if _simulation == null or vehicle_name == "" or _gauge_info.is_empty() or _last_pt.is_empty():
		return

	var idle_rpm: float = _gauge_info.get("idle_rpm", 0.0)
	var limiter_rpm: float = _gauge_info.get("limiter_rpm", 0.0)
	var gear_count: int = int(_gauge_info.get("gear_count", 0))
	var max_rpm: float = GaugeLogic.max_rpm_for_limiter(limiter_rpm)

	var vp_size: Vector2 = get_viewport_rect().size
	var scale: float = max(vp_size.y / 1080.0, 0.4)
	var radius: float = 150.0 * scale
	var margin: float = 64.0 * scale
	var center := Vector2(vp_size.x - margin - radius, vp_size.y - margin - radius * 1.5)

	var font: Font = get_theme_default_font()
	var font_size_tick: int = int(round(15.0 * scale))
	var gear_font_scale: float = 1.0 if gear_count < 10 else 0.75
	var font_size_gear: int = int(round(46.0 * scale * gear_font_scale))
	var font_size_lamp: int = int(round(13.0 * scale))
	var font_size_speed: int = int(round(52.0 * scale))
	var font_size_speed_unit: int = int(round(16.0 * scale))

	draw_circle(center, radius * 1.08, COLOR_BG)

	var redline_start_rad: float = _clock_rad(GaugeLogic.redline_start_clock_deg(limiter_rpm, max_rpm))
	var redline_end_rad: float = _clock_rad(GaugeLogic.redline_end_clock_deg(max_rpm))
	if redline_end_rad > redline_start_rad + 0.001:
		draw_arc(center, radius * 1.02, redline_start_rad, redline_end_rad, 32, COLOR_REDLINE, radius * 0.05, true)

	for major_rpm in GaugeLogic.major_tick_rpms(max_rpm):
		var deg: float = GaugeLogic.clock_deg_for_rpm(major_rpm, max_rpm)
		draw_line(_polar(center, radius * 0.86, deg), _polar(center, radius, deg), COLOR_TICK_MAJOR, scale * 2.5, true)
		_draw_centered_text(font, _polar(center, radius * 0.70, deg), str(int(round(major_rpm / 1000.0))), font_size_tick, COLOR_TICK_MAJOR)
	for minor_rpm in GaugeLogic.minor_tick_rpms(max_rpm):
		var deg2: float = GaugeLogic.clock_deg_for_rpm(minor_rpm, max_rpm)
		draw_line(_polar(center, radius * 0.93, deg2), _polar(center, radius, deg2), COLOR_TICK_MINOR, scale * 1.5, true)

	if idle_rpm > 0.0:
		draw_circle(_polar(center, radius * 0.79, GaugeLogic.clock_deg_for_rpm(idle_rpm, max_rpm)), scale * 3.0, COLOR_TICK_MINOR)

	var needle_deg: float = GaugeLogic.clock_deg_for_rpm(_display_rpm, max_rpm)
	var in_redline: bool = limiter_rpm > 0.0 and _display_rpm >= limiter_rpm
	var needle_color: Color = COLOR_NEEDLE_REDLINE if in_redline else COLOR_NEEDLE
	draw_line(_polar(center, radius * 0.14, needle_deg + 180.0), _polar(center, radius * 0.88, needle_deg), needle_color, scale * 3.0, true)
	draw_circle(center, radius * 0.06, needle_color)

	var gear_radius: float = radius * 0.34
	var engine_state: String = String(_last_pt.get("engine_state", "off"))
	var grind_active: bool = bool(_last_pt.get("grind_active", false))
	var blink_on: bool = fmod(Time.get_ticks_msec() / 1000.0, 0.3) < 0.15
	var gear_text_color: Color = COLOR_GEAR_TEXT
	if grind_active and blink_on:
		gear_text_color = COLOR_GEAR_GRIND_FLASH
	elif engine_state == "stalled" or engine_state == "off":
		gear_text_color = COLOR_GEAR_STALLED
	draw_circle(center, gear_radius, COLOR_GEAR_BG)
	draw_arc(center, gear_radius, 0.0, TAU, 48, COLOR_TICK_MINOR, scale * 1.5, true)
	_draw_centered_text(font, center, GaugeLogic.gear_label(int(_last_pt.get("gear", 0))), font_size_gear, gear_text_color)

	var fitted_assists: PackedStringArray = _gauge_info.get("fitted_assists", PackedStringArray())
	for i in range(SYSTEM_LAMP_NAMES.size()):
		var lamp_name: String = SYSTEM_LAMP_NAMES[i]
		var state: int = GaugeLogic.lamp_state(fitted_assists.has(lamp_name), lamp_name == "TC" and bool(_last_pt.get("assist_traction_control", false)), lamp_name == "TC" and bool(_last_pt.get("traction_control_intervening", false)))
		if state == GaugeLogic.LampState.HIDDEN:
			continue
		_draw_lamp(font, _polar(center, gear_radius * 1.7, SYSTEM_LAMP_SLOTS_CLOCK_DEG[i]), lamp_name, state, 6.0 * scale, font_size_lamp)

	var extra_lamps: Array = _existing_assist_lamp_states()
	var row_y: float = center.y + radius * 1.28
	var row_spacing: float = 62.0 * scale
	var row_start_x: float = center.x - row_spacing * (extra_lamps.size() - 1) * 0.5
	for i in range(extra_lamps.size()):
		var entry: Dictionary = extra_lamps[i]
		_draw_lamp(font, Vector2(row_start_x + i * row_spacing, row_y), entry["name"], entry["state"], 6.0 * scale, font_size_lamp)

	var speed_kmh: float = float(_last_pt.get("speedo_kmh", 0.0))
	var speed_pos := Vector2(center.x - radius * 1.55, center.y)
	_draw_centered_text(font, speed_pos + Vector2(0, -font_size_speed * 0.35), "%d" % int(round(speed_kmh)), font_size_speed, COLOR_SPEED_TEXT)
	_draw_centered_text(font, speed_pos + Vector2(0, -font_size_speed * 1.15), "GPS %d" % int(round(_gps_kmh)), font_size_speed_unit, COLOR_TICK_MINOR)
	_draw_centered_text(font, speed_pos + Vector2(0, font_size_speed * 0.55), "km/h", font_size_speed_unit, COLOR_TICK_MINOR)

	var limit_pos := speed_pos + Vector2(0, font_size_speed * 1.7)
	var limit_radius: float = 24.0 * scale
	draw_circle(limit_pos, limit_radius, Color(0.95, 0.95, 0.95))
	draw_arc(limit_pos, limit_radius, 0, TAU, 48, COLOR_REDLINE, 4.0 * scale, true)
	var limit_text: String = "--"
	if _speed_limit.get("kind", "unknown") == "numeric":
		limit_text = "%d" % int(round(float(_speed_limit.get("kph", 0.0))))
	elif _speed_limit.get("kind", "unknown") == "unrestricted":
		limit_text = "///"
	_draw_centered_text(font, limit_pos, limit_text, int(round(19.0 * scale)), Color(0.1, 0.1, 0.1))
	if _speed_limit.get("conditional", false):
		_draw_centered_text(font, limit_pos + Vector2(limit_radius * 1.5, 0), "?", font_size_speed_unit, COLOR_SPEED_TEXT)

	var warning: String = ""
	if engine_state == "stalled":
		warning = "ENGINE STALLED - hold Start / K to restart"
	elif engine_state == "off":
		warning = "IGNITION OFF - hold I"
	elif grind_active:
		warning = "GRIND!"
	var slow_blink_on: bool = fmod(Time.get_ticks_msec() / 1000.0, 1.0) < 0.7
	if warning != "" and (engine_state == "off" or (slow_blink_on if engine_state == "stalled" else blink_on)):
		var banner_pos := Vector2(center.x - radius * 0.5, center.y - radius * 1.35)
		_draw_centered_text(font, banner_pos, warning, int(round(24.0 * scale)), COLOR_NEEDLE_REDLINE)
