extends RefCounted
# game/scripts/thermal_readout.gd - the engine temperature readout shared by every dashboard (the round HUD gauge,
# tach_gauge.gd, and the hypercar's cockpit cluster, digital_cluster.gd). Drawing only: the values, the thresholds and
# the level (cold / normal / hot / overheat) come from RgSimulation.get_vehicle_thermal, i.e. rg::engine_thermal_hud
# in rg_core (core/include/rg/engine_thermal_info.h). A car without a thermal model reports {present: false} and
# nothing is drawn - no fake value.

const UiTheme := preload("res://scripts/ui/ui_theme.gd")

# rg::ThermalLevel code -> colour (0 cold, 1 normal, 2 hot, 3 overheat).
static func level_color(level_code: int) -> Color:
	match level_code:
		0: return UiTheme.role(&"info")
		2: return UiTheme.role(&"warning")
		3: return UiTheme.role(&"danger")
		_: return UiTheme.role(&"good")

static func is_present(thermal: Dictionary) -> bool:
	return bool(thermal.get("present", false))

# "91 C" style value text (degrees Celsius, whole numbers).
static func temp_text(celsius: float) -> String:
	return "%d°C" % int(round(celsius))

# Draws the readout centred on `centre`: a COOLANT value line, a needle-position bar (fill = the car's own gauge
# curve) coloured by level, and an OIL line. `blink_on` flashes the overheat state. `unit` scales the sizes (1.0 =
# 1080p HUD scale).
static func draw_bar(canvas: CanvasItem, font: Font, centre: Vector2, unit: float, thermal: Dictionary, blink_on: bool) -> void:
	if not is_present(thermal):
		return
	var level: int = int(thermal.get("level_code", 1))
	var color: Color = level_color(level)
	if level == 3 and not blink_on:
		color = UiTheme.role(&"text")
	var width: float = 150.0 * unit
	var bar_h: float = 9.0 * unit
	var label_size: int = int(round(13.0 * unit))
	var value_size: int = int(round(22.0 * unit))
	var coolant_c: float = float(thermal.get("coolant_c", 0.0))
	var oil_c: float = float(thermal.get("oil_c", 0.0))
	var gauge: float = clampf(float(thermal.get("gauge", 0.0)), 0.0, 1.0)

	var left: float = centre.x - width * 0.5
	canvas.draw_string(font, Vector2(left, centre.y - value_size * 0.35), "COOLANT", HORIZONTAL_ALIGNMENT_LEFT, -1, label_size, UiTheme.role(&"text_muted"))
	var value: String = temp_text(coolant_c)
	var value_w: float = font.get_string_size(value, HORIZONTAL_ALIGNMENT_LEFT, -1, value_size).x
	canvas.draw_string(font, Vector2(left + width - value_w, centre.y - value_size * 0.35), value, HORIZONTAL_ALIGNMENT_LEFT, -1, value_size, color)
	var bar_y: float = centre.y + 2.0 * unit
	canvas.draw_rect(Rect2(left, bar_y, width, bar_h), UiTheme.role(&"gauge_track"))
	canvas.draw_rect(Rect2(left, bar_y, width * gauge, bar_h), color)
	var oil_text: String = "OIL %s" % temp_text(oil_c)
	canvas.draw_string(font, Vector2(left, bar_y + bar_h + label_size * 1.25), oil_text, HORIZONTAL_ALIGNMENT_LEFT, -1, label_size, UiTheme.role(&"text_muted"))
	if bool(thermal.get("fan_on", false)):
		var fan_w: float = font.get_string_size("FAN", HORIZONTAL_ALIGNMENT_LEFT, -1, label_size).x
		canvas.draw_string(font, Vector2(left + width - fan_w, bar_y + bar_h + label_size * 1.25), "FAN", HORIZONTAL_ALIGNMENT_LEFT, -1, label_size, UiTheme.role(&"good"))
