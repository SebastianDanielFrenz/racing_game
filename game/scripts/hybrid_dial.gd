extends RefCounted
# Dashboard pieces for cars with a traction battery (owner 2026-10-18). Every VALUE comes from core (rg/dashboard_info.h through
# RgSimulation.get_vehicle_gauge_info / get_vehicle_powertrain: tach_kind, motor_dial_max_rpm, motor_dial_tick_rpm,
# traction_motor_rpm, battery_percent, battery_level, battery_label); this file only draws them, so the HUD gauge and the
# digital cluster look the same.

const COLOR_OK := Color(0.2, 0.95, 0.75)
const COLOR_LOW := Color(1.0, 0.65, 0.15)
const COLOR_CRITICAL := Color(1.0, 0.25, 0.16)
const COLOR_TRACK := Color(0.15, 0.27, 0.34)
const COLOR_MOTOR_NEEDLE := Color(0.35, 0.8, 1.0)

static func is_hybrid(gauge_info: Dictionary) -> bool:
	return str(gauge_info.get("tach_kind", "ice")) == "hybrid"

# Labelled values of the inner scale: 0, tick, 2*tick ... up to and including max (rpm).
static func inner_ticks(max_rpm: float, tick_rpm: float) -> Array:
	var out := []
	if max_rpm <= 0.0 or tick_rpm <= 0.0:
		return out
	var v := 0.0
	while v <= max_rpm + 0.01:
		out.append(v)
		v += tick_rpm
	return out

static func battery_color(level: int, ok := COLOR_OK) -> Color:
	if level >= 2:
		return COLOR_CRITICAL
	if level == 1:
		return COLOR_LOW
	return ok

static func has_battery(pt: Dictionary) -> bool:
	return bool(pt.get("battery_fitted", false))

# "SoC 25%" and a small bar below it, both centred on `center_x`; `top_y` is the text baseline. Nothing for a car without a
# traction battery. Returns the height used.
static func draw_battery(canvas: CanvasItem, font: Font, center_x: float, top_y: float, pt: Dictionary, font_size: int, bar_size: Vector2, ok := COLOR_OK) -> float:
	if not has_battery(pt):
		return 0.0
	var color := battery_color(int(pt.get("battery_level", 0)), ok)
	var label := str(pt.get("battery_label", ""))
	var width := font.get_string_size(label, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size).x
	canvas.draw_string(font, Vector2(center_x - width * 0.5, top_y), label, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, color)
	var bar_top := top_y + font_size * 0.35
	var left := center_x - bar_size.x * 0.5
	var fraction := clampf(float(pt.get("battery_percent", 0)) / 100.0, 0.0, 1.0)
	canvas.draw_rect(Rect2(left, bar_top, bar_size.x, bar_size.y), COLOR_TRACK)
	canvas.draw_rect(Rect2(left, bar_top, bar_size.x * fraction, bar_size.y), color)
	return font_size * 0.35 + bar_size.y
