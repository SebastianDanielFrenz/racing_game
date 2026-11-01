extends Control
# game/scripts/ui/ui_gauge.gd - the design language's round dial (docs/design_language.md): a face, a sweep of numbered ticks,
# a red zone, a needle and a gear window in the hub. The anatomy is the gauge.* tokens, the colours are gauge roles; the dial
# scales with the Control's size, so it survives a small corner and a headset view.
#     const UiGauge := preload("res://scripts/ui/ui_gauge.gd")
#     var dial := UiGauge.new()
#     dial.custom_minimum_size = Vector2(240, 240)
#     dial.max_value = 2800.0; dial.major_step = 400.0; dial.figure_divisor = 100.0
#     dial.value = rpm; dial.gear = "4"; dial.readout = "18"; dial.caption = "km/h"
# The needle turns to gauge_needle_redline in the red zone. alarm paints the gear window with surface_inset_alert: a stall.

const UiTheme := preload("res://scripts/ui/ui_theme.gd")

@export var max_value := 100.0: set = _set_max_value
@export var major_step := 20.0: set = _set_major_step
@export var minor_per_major := 4: set = _set_minor_per_major
# Where the red zone starts; below 0 it is gauge.redline_fraction of max_value.
@export var redline_from := -1.0: set = _set_redline_from
# Figures on the scale are value / figure_divisor.
@export var figure_divisor := 1.0: set = _set_figure_divisor

var value := 0.0: set = _set_value
var gear := "": set = _set_gear
var readout := "": set = _set_readout
var caption := "": set = _set_caption
var alarm := false: set = _set_alarm

func _ready() -> void:
	UiTheme.follow(self)
	resized.connect(queue_redraw)

func _set_max_value(v: float) -> void:
	max_value = v
	queue_redraw()

func _set_major_step(v: float) -> void:
	major_step = v
	queue_redraw()

func _set_minor_per_major(v: int) -> void:
	minor_per_major = v
	queue_redraw()

func _set_redline_from(v: float) -> void:
	redline_from = v
	queue_redraw()

func _set_figure_divisor(v: float) -> void:
	figure_divisor = v
	queue_redraw()

func _set_value(v: float) -> void:
	value = v
	queue_redraw()

func _set_gear(v: String) -> void:
	gear = v
	queue_redraw()

func _set_readout(v: String) -> void:
	readout = v
	queue_redraw()

func _set_caption(v: String) -> void:
	caption = v
	queue_redraw()

func _set_alarm(v: bool) -> void:
	alarm = v
	queue_redraw()

func _angle(at: float) -> float:
	var start := deg_to_rad(UiTheme.token(&"gauge.sweep_start_deg"))
	var end := deg_to_rad(UiTheme.token(&"gauge.sweep_end_deg"))
	return lerpf(start, end, clampf(at / maxf(max_value, 1e-6), 0.0, 1.0))

func _polar(centre: Vector2, radius: float, angle: float) -> Vector2:
	return centre + radius * Vector2.from_angle(angle)

func _centred(font: Font, at: Vector2, line: String, font_size: int, colour: Color) -> void:
	var extent := font.get_string_size(line, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size)
	draw_string(font, at - Vector2(extent.x * 0.5, -extent.y * 0.35), line, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, colour)

func _draw() -> void:
	var radius := minf(size.x, size.y) * 0.5
	var k := radius / UiTheme.token(&"gauge.design_radius")
	var centre := size * 0.5
	var font := get_theme_default_font()
	draw_circle(centre, radius, UiTheme.role(&"surface_hud"))
	var ring := radius * UiTheme.token(&"gauge.tick_ring_fraction")
	var red_from := redline_from if redline_from >= 0.0 else max_value * UiTheme.token(&"gauge.redline_fraction")
	var arc := UiTheme.token(&"stroke.gauge_arc") * k
	draw_arc(centre, ring + arc * 0.5, _angle(red_from), _angle(max_value), 32, UiTheme.role(&"gauge_redline"), arc, true)
	var major := UiTheme.token(&"gauge.major_tick_length") * k
	var minor := UiTheme.token(&"gauge.minor_tick_length") * k
	var figures := radius * UiTheme.token(&"gauge.figure_ring_fraction")
	var figure_size := int(round(UiTheme.token(&"type.small") * k))
	var count := int(floor(max_value / maxf(major_step, 1e-6) + 1e-6))
	for i in range(count + 1):
		var at := i * major_step
		var angle := _angle(at)
		draw_line(_polar(centre, ring - major, angle), _polar(centre, ring, angle), UiTheme.role(&"gauge_tick_major"), UiTheme.token(&"stroke.edge") * k, true)
		_centred(font, _polar(centre, figures, angle), "%d" % int(round(at / figure_divisor)), figure_size, UiTheme.role(&"gauge_tick_major"))
		if i == count:
			continue
		for j in range(1, minor_per_major):
			var between := _angle(at + major_step * j / minor_per_major)
			draw_line(_polar(centre, ring - minor, between), _polar(centre, ring, between), UiTheme.role(&"gauge_tick_minor"), UiTheme.token(&"stroke.hair") * k, true)
	var needle := UiTheme.role(&"gauge_needle_redline") if value >= red_from else UiTheme.role(&"gauge_needle")
	var needle_angle := _angle(value)
	draw_line(_polar(centre, -radius * 0.13, needle_angle), _polar(centre, ring - minor, needle_angle), needle, UiTheme.token(&"stroke.gauge_needle") * k, true)
	var inset := radius * UiTheme.token(&"gauge.gear_inset_fraction")
	draw_circle(centre, UiTheme.token(&"gauge.needle_hub_radius") * k, needle)
	draw_circle(centre, inset, UiTheme.role(&"surface_inset_alert") if alarm else UiTheme.role(&"surface_inset"))
	draw_arc(centre, inset, 0.0, TAU, 48, needle, UiTheme.token(&"stroke.hair") * k, true)
	if gear != "":
		_centred(font, centre, gear, int(round(UiTheme.token(&"type.gauge_gear") * k)), UiTheme.role(&"text"))
	var below := centre + Vector2(0.0, radius * 0.62)
	if readout != "":
		_centred(font, below, readout, int(round(UiTheme.token(&"type.lead") * k)), UiTheme.role(&"text"))
	if caption != "":
		_centred(font, below + Vector2(0.0, UiTheme.token(&"type.lead") * k), caption, int(round(UiTheme.token(&"type.caption") * k)), UiTheme.role(&"text_muted"))
