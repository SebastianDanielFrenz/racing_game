extends CanvasLayer
# Top-right weather/time widget: icon for the current condition, time, date,
# air temperature at the car. Click it for the time/weather menu. Which icon,
# which menu items and which are greyed out (with the reason) come from
# rg::TimeWeather through RgSky (sky_controller.gd's `sky`); this script only
# draws and forwards. Temperature: the air at the car (RgSimulation
# get_weather_state().air_temp_c / get_aero_state().temperature_k: with the
# physics weather installed both are the climate.json temperature from date,
# time and location, lifted to the car's altitude), else RgSky's own
# air_temp_c (no world loaded).

const BG := Color(0.08, 0.09, 0.11, 1.0)
var sky_controller: Node
var simulation: Node

var _panel: PanelContainer
var _icon: Control
var _time_label: Label
var _info_label: Label
var _menu: PanelContainer
var _menu_box: VBoxContainer
var _slider: HSlider
var _slider_guard := false
var _accum := 1.0

func _ready() -> void:
	layer = 15
	_panel = PanelContainer.new()
	_panel.name = "WeatherPanel"
	_panel.add_theme_stylebox_override("panel", _style(BG))
	_top_right(_panel, 12)
	_panel.mouse_filter = Control.MOUSE_FILTER_STOP
	_panel.tooltip_text = "Time and weather (click)"
	_panel.gui_input.connect(_on_panel_input)
	add_child(_panel)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 8)
	row.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_panel.add_child(row)
	_icon = Control.new()
	_icon.set_script(load("res://scripts/weather_icon.gd"))
	_icon.custom_minimum_size = Vector2(44, 44)
	_icon.background = BG
	_icon.mouse_filter = Control.MOUSE_FILTER_IGNORE
	row.add_child(_icon)
	var texts := VBoxContainer.new()
	texts.mouse_filter = Control.MOUSE_FILTER_IGNORE
	texts.add_theme_constant_override("separation", 0)
	row.add_child(texts)
	_time_label = Label.new()
	_time_label.add_theme_font_size_override("font_size", 20)
	_time_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	texts.add_child(_time_label)
	_info_label = Label.new()
	_info_label.add_theme_font_size_override("font_size", 12)
	_info_label.add_theme_color_override("font_color", Color(0.75, 0.78, 0.82))
	_info_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	texts.add_child(_info_label)

	_menu = PanelContainer.new()
	_menu.name = "WeatherMenu"
	_menu.add_theme_stylebox_override("panel", _style(Color(BG, 0.95)))
	_top_right(_menu, 76)
	_menu.custom_minimum_size = Vector2(300, 0)
	_menu.visible = false
	add_child(_menu)
	_menu_box = VBoxContainer.new()
	_menu.add_child(_menu_box)

# Anchored to the top-right corner, growing left and down with its content.
func _top_right(control: Control, top: float) -> void:
	control.anchor_left = 1.0
	control.anchor_right = 1.0
	control.offset_left = -16.0
	control.offset_right = -16.0
	control.offset_top = top
	control.offset_bottom = top
	control.grow_horizontal = Control.GROW_DIRECTION_BEGIN

func _style(color: Color) -> StyleBoxFlat:
	var box := StyleBoxFlat.new()
	box.bg_color = color
	box.set_corner_radius_all(6)
	box.content_margin_left = 10
	box.content_margin_right = 10
	box.content_margin_top = 6
	box.content_margin_bottom = 6
	return box

func _sky():
	return sky_controller.sky if sky_controller != null else null

func _process(delta: float) -> void:
	_accum += delta
	if _accum < 0.25:
		return
	_accum = 0.0
	var sky = _sky()
	if sky == null:
		_panel.visible = false
		return
	_panel.visible = true
	var s: Dictionary = sky.get_state()
	_icon.icon = s["icon"]
	_time_label.text = s["clock_text"]
	var temperature := "-- °C"
	var aero: Dictionary = simulation.get_aero_state() if simulation != null and simulation.has_method("get_aero_state") else {}
	if aero.has("temperature_k"):
		temperature = sky.format_temperature_c(float(aero["temperature_k"]))
	elif s.has("air_temp_c") and not is_nan(float(s["air_temp_c"])):
		temperature = sky.format_temperature_c(float(s["air_temp_c"]) + 273.15)
	_info_label.text = "%s  %s  %s" % [temperature, s["weather_label"], s["date_text"]]
	if _menu.visible and not _slider_guard and _slider != null:
		_slider_guard = true
		_slider.value = float(s["local_time_h"])
		_slider_guard = false

func _on_panel_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		toggle_menu()
		get_viewport().set_input_as_handled()

func toggle_menu() -> void:
	_menu.visible = not _menu.visible
	if _menu.visible:
		_rebuild_menu()

func _rebuild_menu() -> void:
	for child in _menu_box.get_children():
		child.queue_free()
	var sky = _sky()
	if sky == null:
		return
	var s: Dictionary = sky.get_state()
	_menu_box.add_child(_heading("Time of day"))
	_slider = HSlider.new()
	_slider.min_value = 0.0
	_slider.max_value = 24.0
	_slider.step = 0.25
	_slider.value = float(s["local_time_h"])
	_slider.custom_minimum_size = Vector2(260, 20)
	_slider.value_changed.connect(_on_slider)
	_menu_box.add_child(_slider)
	var groups := {"time": null, "rate": null, "weather": null}
	for item in sky.get_menu():
		var group: String = item["group"]
		if groups[group] == null:
			if group == "rate":
				_menu_box.add_child(_heading("Time speed"))
			elif group == "weather":
				_menu_box.add_child(_heading("Weather"))
			var flow := HFlowContainer.new()
			_menu_box.add_child(flow)
			groups[group] = flow
		var button := Button.new()
		button.text = item["label"]
		button.toggle_mode = group != "time"
		button.button_pressed = item["selected"]
		button.disabled = not item["enabled"]
		button.tooltip_text = item["reason"]
		button.focus_mode = Control.FOCUS_NONE
		button.pressed.connect(_choose.bind(String(item["id"])))
		groups[group].add_child(button)
	var note := Label.new()
	note.text = "Greyed out: " + _first_reason(sky.get_menu())
	note.autowrap_mode = TextServer.AUTOWRAP_WORD
	note.custom_minimum_size = Vector2(260, 0)
	note.add_theme_font_size_override("font_size", 11)
	note.add_theme_color_override("font_color", Color(0.65, 0.68, 0.72))
	note.visible = note.text != "Greyed out: "
	_menu_box.add_child(note)

func _first_reason(items: Array) -> String:
	for item in items:
		if not item["enabled"] and item["reason"] != "":
			return item["reason"]
	return ""

func _heading(text: String) -> Label:
	var label := Label.new()
	label.text = text
	label.add_theme_font_size_override("font_size", 13)
	label.add_theme_color_override("font_color", Color(1.0, 0.85, 0.4))
	return label

func _on_slider(value: float) -> void:
	if _slider_guard or _sky() == null:
		return
	_sky().set_local_time_h(value)
	sky_controller.apply_now()

func _choose(id: String) -> void:
	var sky = _sky()
	if sky == null or not sky.choose(id):
		return
	sky_controller.apply_now()
	_accum = 1.0
	_rebuild_menu.call_deferred()

# Test hook: the ids the open menu shows, with enabled/selected.
func menu_items() -> Array:
	return _sky().get_menu() if _sky() != null else []
