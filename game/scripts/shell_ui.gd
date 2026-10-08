extends CanvasLayer
# game/scripts/shell_ui.gd - the game shell's screens (PLAN.md R5, 11.2): boot
# splash, main menu, spawn picker, pause menu, settings and credits. Drawing and
# input forwarding ONLY: every list on every screen is a view-model that RgShell
# (rg_core's rg::ShellFlow / rg::Settings / rg::Credits / rg::SpawnPresets)
# hands over - menu items, spawn choices (with their availability reasons), the
# settings schema (type, range, choices, whether it applies live) and the
# credits sections - and every user action leaves as a signal that main.gd turns
# into a flow event. This script decides nothing: it does not know which menu
# comes after which, what a setting's valid range is or what is credited.
#
# R6: vehicle select (R6c: the car browser, car_browser_ui.gd) and the configurator are drawn here too, over
# the garage scene (garage_scene.gd, one layer below). Their view-models come from
# RgGarage (stats and options as data, the loader's verdict on every change); the
# controls forward the player's values to RgGarage.set_option and show what it
# answers - this script knows no range, no whitelist and no vehicle.
#
# R5b: the controls screen (Settings -> Controls, pause menu -> Controls) is controls_ui.gd, built over
# RgControls (rg_core's rg::Controls); this script only instantiates it and relays its signals.
#
# The loading screen is loading_overlay.gd (it needs the per-frame start-up
# numbers); the drive screen is no screen at all (this layer hides itself).
#
# Test hooks (tools/smoke_test.ps1 -Shell drives the real controls through
# them): get_button(id) returns the Button of the current screen, get_setting_
# control(key) the control of a settings row; a headless test "clicks" with
# button.pressed.emit() / changes a control's value exactly like the player.

signal menu_item_chosen(id: String)
signal spawn_chosen(id: String)
signal back_requested
signal setting_changed(key: String)
signal boot_finished
# R6 garage
signal vehicle_chosen(id: String)
signal browser_state_changed # group, sort or filter of the car browser changed (main.gd persists it)
signal garage_area_chosen(area_id: String)
signal garage_option_changed(option_id: String)
signal garage_save_requested
signal garage_exit_requested(destination: String)
signal controls_changed # a binding / tuning changed on the controls screen

const BOOT_SECONDS := 1.5
const TITLE := "racing_game"

var shell: Node # RgShell
var fixed_keys_path: String = "" # data/controls/fixed_keys.json (keys the controls screen lists but cannot rebind)
var controls: Node # RgControls (R5b)
var input_map: Node # input_map.gd (kept for the controls screen's tests)
var screen: String = "" # the screen built now ("" before the first)
var garage: Node # RgGarage (vehicle select / configurator view-models)
var garage_vehicle: String = "" # the configurator's car (RgShell.get_garage_vehicle)

var _root: Control
var _boot_elapsed: float = 0.0
var _boot_done: bool = false
var _buttons: Dictionary = {} # id -> Button of the current screen
var _setting_controls: Dictionary = {} # key -> control of the settings screen
var _setting_displays: Dictionary = {} # key -> Label with the shown value
var _setting_notes: Dictionary = {} # key -> Label for a validation message
var thumbs: Node                                # car_thumbnails.gd while a browser screen is up (main.gd owns it)
var _browser: Control                          # car_browser_ui.gd on the vehicle_select / change_car screens
var _controls_ui: Control                      # controls_ui.gd on the controls screen
var _option_controls: Dictionary = {}          # configurator: option id -> control
var _option_values: Dictionary = {}            # configurator: option id -> Label with the shown value
var _option_notes: Dictionary = {}             # configurator: option id -> Label with a rejection
var _option_box: VBoxContainer                 # configurator: the rows of the active tab
var _area_buttons: Dictionary = {}             # configurator: area id -> Button
var _active_area: String = "overview"
var _status_label: Label                       # configurator: the loader's verdict / save state
const PAINT_PALETTE := ["#9aa0ac", "#e8e8ea", "#1b1d22", "#a8473f", "#c9262e", "#e8742a", "#e0b43a", "#3f7a4a", "#3f6b8f", "#2848a8", "#6b3fa0", "#7a8a99"]
const RIM_PALETTE := ["#2b2d31", "#c4c8cf", "#e8e8ea", "#e8742a", "#c9262e", "#e0b43a", "#3f6b8f", "#111111"]


var _garage_mode := "upgrades"
var _garage_left: PanelContainer
var _garage_right: PanelContainer
var _garage_performance: VBoxContainer
var _garage_dyno: Control
var _garage_dyno_status: Label
var _dyno_poll := 0.0
var _garage_path: Array[String] = []
var _garage_preferred: Button
var _component_textures: Dictionary = {}
var _component_catalog: Dictionary = {}
var _garage_exit: Control
var _garage_exit_note: Label
var _garage_exit_focus: Control

func garage_panel_widths() -> Vector2:
	if screen != "configurator" or _garage_left == null or _garage_right == null:
		return Vector2.ZERO
	return Vector2(_garage_left.size.x + 28.0, _garage_right.size.x + 28.0)

func _fit_garage_panels() -> void:
	if screen != "configurator" or _garage_left == null or _garage_right == null:
		return
	var width := _root.size.x
	_garage_left.custom_minimum_size.x = clampf(width * 0.25, 170.0, 360.0)
	_garage_right.custom_minimum_size.x = clampf(width * 0.29, 220.0, 440.0)

func _ready() -> void:
	layer = 30
	_root = Control.new()
	_root.name = "Root"
	_root.set_anchors_preset(Control.PRESET_FULL_RECT)
	add_child(_root)
	visible = false

# ---- public ----------------------------------------------------------------

func show_screen(screen_name: String) -> void:
	for child in _root.get_children():
		_root.remove_child(child)
		child.queue_free()
	_buttons.clear()
	_setting_controls.clear()
	_setting_displays.clear()
	_setting_notes.clear()
	_option_controls.clear()
	_option_values.clear()
	_option_notes.clear()
	_area_buttons.clear()
	_browser = null
	_controls_ui = null
	_option_box = null
	_status_label = null
	_garage_left = null
	_garage_right = null
	_garage_performance = null
	_garage_exit = null
	_garage_exit_note = null
	_garage_exit_focus = null
	screen = screen_name
	match screen_name:
		"boot":
			_boot_elapsed = 0.0
			_boot_done = false
			_build_boot()
		"main_menu":
			_build_main_menu()
		"spawn_picker":
			_build_spawn_picker()
		"pause":
			_build_pause()
		"settings":
			_build_settings()
		"credits":
			_build_credits()
		"vehicle_select", "change_car":
			_build_browser(screen_name)
		"configurator":
			_build_configurator()
		"controls":
			_build_controls()
		_:
			visible = false
			return
	visible = true

func get_button(id: String) -> Button:
	return _buttons.get(id, null)

func button_ids() -> PackedStringArray:
	return PackedStringArray(_buttons.keys())

func get_setting_control(key: String) -> Control:
	return _setting_controls.get(key, null)

# Configurator test hooks: the control of an option row (only the active tab's rows exist).
func get_option_control(option_id: String) -> Control:
	return _option_controls.get(option_id, null)

func option_control_ids() -> PackedStringArray:
	return PackedStringArray(_option_controls.keys())

func active_area() -> String:
	return _active_area

func status_text() -> String:
	return _status_label.text if _status_label != null else ""

func _process(delta: float) -> void:
	_fit_garage_panels()
	_dyno_poll -= delta
	if screen == "configurator" and is_instance_valid(_garage_dyno) and _dyno_poll <= 0.0:
		_dyno_poll = 0.15
		if not garage.has_method("get_dyno"):
			_garage_dyno_status.text = "Dyno build pending · restart after installation"
			return
		var data: Dictionary = garage.get_dyno()
		_garage_dyno.update_result(data)
		_garage_dyno_status.text = ("Updating · %d%%" % int(data.get("progress",0))) if bool(data.get("busy",false)) else ("Dyno failed · previous curve retained" if not str(data.get("error","")).is_empty() else "Crank · WOT · ISA · N₂O off")
		_garage_dyno_status.tooltip_text = str(data.get("error",""))
	if screen == "boot" and not _boot_done:
		_boot_elapsed += delta
		if _boot_elapsed >= BOOT_SECONDS:
			_finish_boot()

func _input(event: InputEvent) -> void:
	if screen != "boot" or _boot_done:
		return
	var pressed := false
	if event is InputEventKey:
		pressed = event.pressed and not event.echo
	elif event is InputEventMouseButton or event is InputEventJoypadButton:
		pressed = event.pressed
	if pressed:
		_finish_boot()

func _finish_boot() -> void:
	_boot_done = true
	boot_finished.emit()

# ---- building blocks ---------------------------------------------------------

func _backdrop(alpha: float) -> void:
	var rect := ColorRect.new()
	rect.color = Color(0.05, 0.07, 0.09, alpha)
	rect.set_anchors_preset(Control.PRESET_FULL_RECT)
	_root.add_child(rect)

func _label(text: String, font_size: int, align: int = HORIZONTAL_ALIGNMENT_CENTER, dim: bool = false) -> Label:
	var label := Label.new()
	label.text = text
	label.add_theme_font_size_override("font_size", font_size)
	label.horizontal_alignment = align
	label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	if dim:
		label.modulate = Color(1, 1, 1, 0.7)
	return label

func _centered_column(min_width: float) -> VBoxContainer:
	var center := CenterContainer.new()
	center.set_anchors_preset(Control.PRESET_FULL_RECT)
	_root.add_child(center)
	var column := VBoxContainer.new()
	column.custom_minimum_size = Vector2(min_width, 0)
	column.add_theme_constant_override("separation", 12)
	center.add_child(column)
	return column

# A full-screen frame: title on top, a scrolling body, a footer row. Returns
# {body, footer}.
func _page(title: String, subtitle: String = "") -> Dictionary:
	var margin := MarginContainer.new()
	margin.set_anchors_preset(Control.PRESET_FULL_RECT)
	for side in ["left", "right", "top", "bottom"]:
		margin.add_theme_constant_override("margin_" + side, 36)
	_root.add_child(margin)
	var frame := VBoxContainer.new()
	frame.add_theme_constant_override("separation", 10)
	margin.add_child(frame)
	frame.add_child(_label(title, 32))
	if subtitle != "":
		frame.add_child(_label(subtitle, 14, HORIZONTAL_ALIGNMENT_CENTER, true))
	var scroll := ScrollContainer.new()
	scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	frame.add_child(scroll)
	var body := VBoxContainer.new()
	body.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	body.add_theme_constant_override("separation", 8)
	scroll.add_child(body)
	var footer := HBoxContainer.new()
	footer.alignment = BoxContainer.ALIGNMENT_CENTER
	footer.add_theme_constant_override("separation", 16)
	frame.add_child(footer)
	return {"body": body, "footer": footer}

func _button(parent: Control, id: String, text: String, callback: Callable, width: float = 0.0) -> Button:
	var button := Button.new()
	button.text = text
	button.add_theme_font_size_override("font_size", 20)
	if width > 0.0:
		button.custom_minimum_size = Vector2(width, 0)
	button.pressed.connect(callback)
	parent.add_child(button)
	_buttons[id] = button
	return button

func _focus_first() -> void:
	if screen == "configurator" and is_instance_valid(_garage_preferred):
		_focus_garage.call_deferred()
		return
	for id in _buttons:
		var button: Button = _buttons[id]
		if not button.disabled:
			call_deferred("_grab_focus_if_alive", button)
			return

# The screen can be replaced before the deferred call runs (a quick key press).
func _grab_focus_if_alive(button: Button) -> void:
	if is_instance_valid(button) and button.is_inside_tree() and button.is_visible_in_tree():
		button.grab_focus()

func _attribution_footer(parent: Control) -> void:
	var line: String = str(shell.get_attribution_line()) if shell != null else ""
	if line != "":
		var label := _label(line, 12, HORIZONTAL_ALIGNMENT_CENTER, true)
		parent.add_child(label)

# ---- screens -----------------------------------------------------------------

func _build_boot() -> void:
	_backdrop(1.0)
	var column := _centered_column(760)
	column.add_child(_label(TITLE, 56))
	column.add_child(_label("Drive the real world", 22, HORIZONTAL_ALIGNMENT_CENTER, true))
	column.add_child(_label("", 20))
	_attribution_footer(column)
	column.add_child(_label("press any key", 16, HORIZONTAL_ALIGNMENT_CENTER, true))

func _build_main_menu() -> void:
	_backdrop(1.0)
	var column := _centered_column(420)
	column.add_child(_label(TITLE, 48))
	var error: String = str(shell.get_last_error())
	if error != "":
		var message := _label("The world did not load: " + error, 16)
		message.add_theme_color_override("font_color", Color(1.0, 0.7, 0.3))
		column.add_child(message)
		shell.clear_last_error()
	for item in shell.get_main_menu_items():
		var id: String = str(item["id"])
		_button(column, id, str(item["label"]), func(): menu_item_chosen.emit(id), 420)
	column.add_child(_label("", 10))
	_attribution_footer(column)
	_focus_first()

func _build_pause() -> void:
	_backdrop(0.72)
	var column := _centered_column(380)
	column.add_child(_label("Paused", 40))
	for item in shell.get_pause_menu_items():
		var id: String = str(item["id"])
		_button(column, id, str(item["label"]), func(): menu_item_chosen.emit(id), 380)
	_focus_first()

func _build_spawn_picker() -> void:
	_backdrop(1.0)
	var page := _page("Free roam", "Choose where the drive starts")
	var body: VBoxContainer = page["body"]
	for choice in shell.get_spawn_choices():
		var id: String = str(choice["id"])
		var holder := VBoxContainer.new()
		holder.add_theme_constant_override("separation", 2)
		body.add_child(holder)
		var button := _button(holder, "spawn:" + id, str(choice["label"]), func(): spawn_chosen.emit(id))
		button.alignment = HORIZONTAL_ALIGNMENT_LEFT
		button.disabled = not bool(choice["available"])
		var detail: String = str(choice["detail"])
		if not bool(choice["available"]):
			detail = str(choice["unavailable_reason"])
		if detail != "":
			var detail_label := _label(detail, 14, HORIZONTAL_ALIGNMENT_LEFT, true)
			detail_label.add_theme_constant_override("line_spacing", 0)
			holder.add_child(detail_label)
	_button(page["footer"], "back", "Back", func(): back_requested.emit(), 180)
	_focus_first()

func _build_credits() -> void:
	_backdrop(1.0)
	var page := _page("Credits", str(shell.get_attribution_line()))
	var body: VBoxContainer = page["body"]
	for section in shell.get_credits_sections():
		var heading := _label(str(section["title"]), 24, HORIZONTAL_ALIGNMENT_LEFT)
		body.add_child(heading)
		for entry in section["entries"]:
			var box := VBoxContainer.new()
			box.add_theme_constant_override("separation", 1)
			body.add_child(box)
			box.add_child(_label(str(entry["name"]), 18, HORIZONTAL_ALIGNMENT_LEFT))
			var meta := str(entry["provider"])
			if str(entry["licence"]) != "":
				meta += "  |  " + str(entry["licence"])
			if str(entry["licence_url"]) != "":
				meta += "  |  " + str(entry["licence_url"])
			box.add_child(_label(meta, 13, HORIZONTAL_ALIGNMENT_LEFT, true))
			if str(entry["attribution_text"]) != "":
				box.add_child(_label(str(entry["attribution_text"]), 14, HORIZONTAL_ALIGNMENT_LEFT))
			if str(entry["note"]) != "":
				box.add_child(_label(str(entry["note"]), 13, HORIZONTAL_ALIGNMENT_LEFT, true))
	_button(page["footer"], "back", "Back", func(): back_requested.emit(), 180)
	_focus_first()

# ---- settings (built from RgShell.get_settings_schema) ---------------------------

func _build_settings() -> void:
	_backdrop(1.0)
	var page := _page("Settings", "Changes apply at once where marked; they are saved when you leave this screen")
	var body: VBoxContainer = page["body"]
	for section in shell.get_settings_schema():
		body.add_child(_label(str(section["title"]), 24, HORIZONTAL_ALIGNMENT_LEFT))
		for def in section["settings"]:
			body.add_child(_setting_row(def))
	body.add_child(_label("Controls", 24, HORIZONTAL_ALIGNMENT_LEFT))
	var controls_row := HBoxContainer.new()
	controls_row.add_theme_constant_override("separation", 12)
	body.add_child(controls_row)
	_button(controls_row, "controls", "Controls...", func(): menu_item_chosen.emit("controls"), 240)
	controls_row.add_child(_label("Rebind keys, buttons, axes and pedals per device; the choices are remembered per device.", 13, HORIZONTAL_ALIGNMENT_LEFT, true))
	_button(page["footer"], "reset_settings", "Reset to defaults", func():
		shell.reset_all_settings()
		for key in _setting_controls.keys():
			setting_changed.emit(key)
		show_screen("settings"), 240)
	_button(page["footer"], "back", "Back", func(): back_requested.emit(), 180)
	_focus_first()

func _setting_row(def: Dictionary) -> Control:
	var key: String = str(def["key"])
	var holder := VBoxContainer.new()
	holder.add_theme_constant_override("separation", 0)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 12)
	holder.add_child(row)
	var name_label := _label(str(def["label"]), 18, HORIZONTAL_ALIGNMENT_LEFT)
	name_label.custom_minimum_size = Vector2(300, 0)
	row.add_child(name_label)
	var kind: String = str(def["type"])
	var control: Control
	var show_value := false
	match kind:
		"bool":
			var check := CheckButton.new()
			check.button_pressed = bool(shell.get_setting(key))
			check.toggled.connect(func(on: bool): _apply_setting(key, on))
			control = check
		"int", "float":
			var slider := HSlider.new()
			slider.min_value = float(def["min"])
			slider.max_value = float(def["max"])
			slider.step = float(def["step"])
			slider.value = float(shell.get_setting(key))
			slider.custom_minimum_size = Vector2(320, 24)
			slider.size_flags_vertical = Control.SIZE_SHRINK_CENTER
			var is_int := kind == "int"
			slider.value_changed.connect(func(v: float): _apply_setting(key, int(round(v)) if is_int else v))
			control = slider
			show_value = true
		"choice":
			var options := OptionButton.new()
			var choices: PackedStringArray = def["choices"]
			for choice in choices:
				options.add_item(str(choice).replace("_", " "))
			options.selected = maxi(choices.find(str(shell.get_setting(key))), 0)
			options.item_selected.connect(func(index: int): _apply_setting(key, choices[index]))
			control = options
		_:
			var edit := LineEdit.new()
			edit.text = str(shell.get_setting(key))
			edit.custom_minimum_size = Vector2(420, 0)
			edit.placeholder_text = "(default)"
			edit.text_submitted.connect(func(text: String): _apply_text_setting(key, text))
			edit.focus_exited.connect(func(): _apply_text_setting(key, edit.text))
			control = edit
	row.add_child(control)
	_setting_controls[key] = control
	if show_value:
		var value_label := _label(str(shell.get_setting_display(key)), 16, HORIZONTAL_ALIGNMENT_LEFT)
		value_label.custom_minimum_size = Vector2(110, 0)
		row.add_child(value_label)
		_setting_displays[key] = value_label
	var help: String = str(def["help"])
	if not bool(def["applies_live"]):
		help += " (applies when the next world loads)"
	holder.add_child(_label(help, 13, HORIZONTAL_ALIGNMENT_LEFT, true))
	var note := _label("", 13, HORIZONTAL_ALIGNMENT_LEFT)
	note.add_theme_color_override("font_color", Color(1.0, 0.7, 0.3))
	note.visible = false
	holder.add_child(note)
	_setting_notes[key] = note
	return holder

# A text field commits on Enter / focus loss, and only when it actually changed
# (removing the screen also drops focus).
func _apply_text_setting(key: String, text: String) -> void:
	if text != str(shell.get_setting(key)):
		_apply_setting(key, text)

func _apply_setting(key: String, value: Variant) -> void:
	var result: Dictionary = shell.set_setting(key, value)
	var control: Control = _setting_controls.get(key, null)
	var note: Label = _setting_notes.get(key, null)
	if note != null:
		note.visible = not bool(result["ok"]) or bool(result["clamped"])
		if not bool(result["ok"]):
			note.text = str(result["error"])
		elif bool(result["clamped"]):
			note.text = "Out of range: using " + str(shell.get_setting_display(key))
	if control is HSlider:
		(control as HSlider).set_value_no_signal(float(result["value"]))
	if control is LineEdit and not bool(result["ok"]):
		(control as LineEdit).text = str(result["value"])
	if _setting_displays.has(key):
		(_setting_displays[key] as Label).text = str(shell.get_setting_display(key))
	setting_changed.emit(key)

# ---- controls (R5b) -------------------------------------------------------------------------

func _build_controls() -> void:
	var ui := Control.new()
	ui.set_script(load("res://scripts/controls_ui.gd"))
	ui.name = "ControlsUi"
	ui.controls = controls
	ui.fixed_keys_path = fixed_keys_path
	ui.back_requested.connect(func(): back_requested.emit())
	ui.changed.connect(func(): controls_changed.emit())
	_root.add_child(ui)
	_controls_ui = ui
	_buttons["back"] = ui.get_hook("back")
	ui.focus_first_device()

# Esc cancels a running capture / calibration first; true when it did.
func controls_consume_back() -> bool:
	return _controls_ui != null and _controls_ui.consume_back()

func controls_screen() -> Control:
	return _controls_ui

# ---- garage: vehicle select ----------------------------------------------------

# A translucent side panel over the garage scene; returns the column inside it.
func _side_panel(on_left: bool, width: float) -> VBoxContainer:
	var margin := MarginContainer.new()
	margin.set_anchors_preset(Control.PRESET_FULL_RECT)
	margin.mouse_filter = Control.MOUSE_FILTER_IGNORE
	for side in ["left", "right", "top", "bottom"]:
		margin.add_theme_constant_override("margin_" + side, 28)
	_root.add_child(margin)
	var row := HBoxContainer.new()
	row.mouse_filter = Control.MOUSE_FILTER_IGNORE
	margin.add_child(row)
	var spacer := Control.new()
	spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	spacer.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var panel := PanelContainer.new()
	panel.custom_minimum_size = Vector2(width, 0)
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.03, 0.04, 0.055, 0.82)
	style.border_color = Color(0.91, 0.45, 0.17, 0.9)
	style.set_border_width_all(0)
	style.border_width_left = 0 if on_left else 3
	style.border_width_right = 3 if on_left else 0
	style.set_corner_radius_all(6)
	style.set_content_margin_all(18)
	panel.add_theme_stylebox_override("panel", style)
	if on_left:
		row.add_child(panel)
		row.add_child(spacer)
	else:
		row.add_child(spacer)
		row.add_child(panel)
	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation", 8)
	panel.add_child(column)
	return column

func _build_browser(screen_name: String) -> void:
	var ui = Control.new()
	ui.set_script(load("res://scripts/car_browser_ui.gd"))
	ui.name = "CarBrowser"
	ui.garage = garage
	ui.thumbs = thumbs
	ui.screen_title = "Garage" if screen_name == "vehicle_select" else "Change car"
	ui.choose_text = "Configure" if screen_name == "vehicle_select" else "Change car"
	ui.car_chosen.connect(func(id: String): vehicle_chosen.emit(id))
	ui.back_requested.connect(func(): back_requested.emit())
	ui.state_changed.connect(func(): browser_state_changed.emit())
	_root.add_child(ui)
	_browser = ui
	_buttons["choose"] = ui.get_control("choose")
	_buttons["back"] = ui.get_control("back")
	_buttons["filter"] = ui.get_control("filter_button")

# Esc closes the browser's filter panel first; true when it did.
func browser_consume_back() -> bool:
	return _browser != null and _browser.consume_back()

func browser() -> Control:
	return _browser

func highlighted_vehicle() -> String:
	return _browser.focused_id() if _browser != null else ""

# ---- garage: configurator ---------------------------------------------------------

func _build_configurator() -> void:
	var left := _side_panel(true, 300)
	_garage_left = left.get_parent()
	left.add_child(_label("VEHICLE PERFORMANCE", 22, HORIZONTAL_ALIGNMENT_LEFT))
	var perf_scroll := ScrollContainer.new()
	perf_scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	perf_scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	left.add_child(perf_scroll)
	_garage_performance = VBoxContainer.new()
	_garage_performance.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_garage_performance.add_theme_constant_override("separation", 14)
	perf_scroll.add_child(_garage_performance)
	_refresh_performance()
	left.add_child(HSeparator.new())
	left.add_child(_label("ENGINE DYNO",19,HORIZONTAL_ALIGNMENT_LEFT))
	_garage_dyno = preload("res://scripts/garage_dyno_chart.gd").new()
	_garage_dyno.name = "DynoChart"
	left.add_child(_garage_dyno)
	_garage_dyno_status = _label("Computing dyno…",12,HORIZONTAL_ALIGNMENT_LEFT,true)
	_garage_dyno_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	left.add_child(_garage_dyno_status)
	_dyno_poll = 0.0
	var column := _side_panel(false, 400)
	_garage_right = column.get_parent()
	var vehicle: Dictionary = garage.get_vehicle(garage_vehicle)
	column.add_child(_label(str(vehicle.get("title", garage_vehicle)), 20, HORIZONTAL_ALIGNMENT_LEFT))
	var modes := HBoxContainer.new()
	column.add_child(modes)
	var mode_group := ButtonGroup.new()
	for mode in ["upgrades", "tuning"]:
		var button := _button(modes,"garage:" + mode,mode.capitalize(),func(): _set_garage_mode(mode),0)
		button.toggle_mode = true
		button.button_group = mode_group
		button.add_theme_font_size_override("font_size",16)
		button.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		button.set_pressed_no_signal(mode == _garage_mode)
	var scroll := ScrollContainer.new()
	scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	column.add_child(scroll)
	_option_box = VBoxContainer.new()
	_option_box.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_option_box.add_theme_constant_override("separation",12)
	scroll.add_child(_option_box)
	_status_label = _label("",14,HORIZONTAL_ALIGNMENT_LEFT)
	_status_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	column.add_child(_status_label)
	var footer := GridContainer.new()
	footer.columns = 2
	column.add_child(footer)
	_button(footer,"reset_all","Reset all",func():
		garage.reset_all()
		_draw_garage_options()
		garage_option_changed.emit("")
		_refresh_status(),0)
	_button(footer,"save","Save",func(): garage_save_requested.emit(),0)
	for button in footer.get_children():
		button.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_garage_path.clear()
	_active_area = "overview"
	_draw_garage_options()
	_refresh_status()
	_fit_garage_panels()
	_focus_first()

# B/Esc backs out of nested width choices before offering the garage exit.
func garage_consume_back() -> void:
	if _garage_exit != null:
		_close_garage_exit()
	elif not _garage_path.is_empty():
		_garage_path.pop_back()
		_draw_garage_options()
	else:
		_open_garage_exit()

func _open_garage_exit() -> void:
	if _garage_exit != null:
		return
	_garage_exit_focus = get_viewport().gui_get_focus_owner()
	_garage_left.hide()
	_garage_right.hide()
	_garage_exit = Control.new()
	_garage_exit.set_anchors_preset(Control.PRESET_FULL_RECT)
	_garage_exit.mouse_filter = Control.MOUSE_FILTER_STOP
	_root.add_child(_garage_exit)
	var shade := ColorRect.new()
	shade.color = Color(0.02,0.025,0.035,0.78)
	shade.set_anchors_preset(Control.PRESET_FULL_RECT)
	_garage_exit.add_child(shade)
	var center := CenterContainer.new()
	center.set_anchors_preset(Control.PRESET_FULL_RECT)
	_garage_exit.add_child(center)
	var panel := PanelContainer.new()
	panel.custom_minimum_size.x = minf(440.0,_root.size.x-48.0)
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.03,0.04,0.055,0.98)
	style.border_color = Color(0.91,0.45,0.17)
	style.set_border_width_all(2)
	style.set_corner_radius_all(6)
	style.set_content_margin_all(24)
	panel.add_theme_stylebox_override("panel",style)
	center.add_child(panel)
	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation",16)
	panel.add_child(column)
	column.add_child(_label("Leave garage?",26))
	_garage_exit_note = _label("Save your setup and choose where to go.",16)
	column.add_child(_garage_exit_note)
	var drive := _button(column,"garage_exit:drive","Save and drive",func(): garage_exit_requested.emit("drive"))
	var selection := _button(column,"garage_exit:selection","Save and go to selection",func(): garage_exit_requested.emit("selection"))
	var valid := bool(garage.get_validation()["ok"])
	drive.disabled = not valid
	selection.disabled = not valid
	if not valid:
		_garage_exit_note.text = "Fix the setup before saving. B / Esc returns to editing."
	else:
		column.add_child(_label("B / Esc · Keep editing",14,HORIZONTAL_ALIGNMENT_CENTER,true))
	# Keep directional controller/keyboard focus within the two choices.
	for button in [drive,selection]:
		var other: Button = selection if button == drive else drive
		button.focus_neighbor_top = button.get_path_to(other)
		button.focus_neighbor_bottom = button.get_path_to(other)
		button.focus_neighbor_left = button.get_path_to(other)
		button.focus_neighbor_right = button.get_path_to(other)
		button.focus_next = button.get_path_to(other)
		button.focus_previous = button.get_path_to(other)
	if valid:
		drive.grab_focus()

func garage_exit_failed(message: String) -> void:
	if _garage_exit_note != null:
		_garage_exit_note.text = "Could not save: " + message

func _close_garage_exit() -> void:
	_root.remove_child(_garage_exit)
	_garage_exit.queue_free()
	_garage_exit = null
	_garage_exit_note = null
	_buttons.erase("garage_exit:drive")
	_buttons.erase("garage_exit:selection")
	_garage_left.show()
	_garage_right.show()
	if is_instance_valid(_garage_exit_focus) and _garage_exit_focus.is_visible_in_tree():
		_garage_exit_focus.grab_focus()
	else:
		_focus_first()
	_garage_exit_focus = null

func _set_garage_mode(mode: String) -> void:
	_garage_mode = mode
	for mode_id in ["upgrades", "tuning"]:
		var button: Button = _buttons.get("garage:" + mode_id, null)
		if button != null:
			button.set_pressed_no_signal(mode_id == mode)
	_garage_path.clear()
	_draw_garage_options()

func _refresh_performance() -> void:
	if _garage_performance == null:
		return
	for child in _garage_performance.get_children():
		_garage_performance.remove_child(child)
		child.queue_free()
	var stats: Dictionary = garage.get_vehicle(garage_vehicle).get("stats", {})
	_garage_performance.add_child(_label("Acceleration",20,HORIZONTAL_ALIGNMENT_LEFT))
	# No made-up times: measured results will be supplied by the performance model.
	var note := _label("No measured acceleration runs for this setup yet.",14,HORIZONTAL_ALIGNMENT_LEFT,true)
	note.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_garage_performance.add_child(note)
	for heading in ["Top speed", "Maximum lateral acceleration", "Maximum longitudinal acceleration"]:
		_garage_performance.add_child(_label(heading,17,HORIZONTAL_ALIGNMENT_LEFT))
		_garage_performance.add_child(_label("Not measured",14,HORIZONTAL_ALIGNMENT_LEFT,true))
	_garage_performance.add_child(HSeparator.new())
	_garage_performance.add_child(_label("Stock vehicle specification",19,HORIZONTAL_ALIGNMENT_LEFT))
	for text in ["Mass  %.0f kg" % float(stats.get("mass_kg",0)),"Power  %.1f kW" % float(stats.get("peak_power_kw",0)),"Torque  %.0f Nm" % float(stats.get("peak_torque_nm",0)),"%s · %d gears" % [str(stats.get("layout","")),int(stats.get("gear_count",0))]]:
		_garage_performance.add_child(_label(text,16,HORIZONTAL_ALIGNMENT_LEFT))
	if bool(stats.get("figures_declared",false)):
		_garage_performance.add_child(_label("Declared engine figures",13,HORIZONTAL_ALIGNMENT_LEFT,true))

func _draw_garage_options() -> void:
	if _option_box == null:
		return
	for child in _option_box.get_children():
		_option_box.remove_child(child)
		child.queue_free()
	_option_controls.clear()
	_option_values.clear()
	_option_notes.clear()
	_garage_preferred = null
	for key in _buttons.keys():
		if str(key).begins_with("tile:") or str(key).begins_with("part:") or str(key).begins_with("tyre:"):
			_buttons.erase(key)
	if _garage_mode == "upgrades":
		_draw_upgrades()
		return
	if _garage_path.is_empty():
		_draw_tiles(["Engine", "Wheels", "Chassis", "Drivetrain", "Appearance & assists"])
		return
	_option_box.add_child(_label(" / ".join(_garage_path),20,HORIZONTAL_ALIGNMENT_LEFT))
	var group := ""
	var count := 0
	for opt in garage.get_options():
		if str(opt["area"]) != _active_area or not bool(opt["available"]):
			continue
		if str(opt["kind"]) in ["tyre_choice", "file_choice"]:
			_option_box.add_child(_label("Installed " + str(opt["label"]).to_lower(),17,HORIZONTAL_ALIGNMENT_LEFT))
			var installed := _label(_part_name(opt) if str(opt["kind"]) == "file_choice" else _display_value("tyre_choice",opt["value"]),14,HORIZONTAL_ALIGNMENT_LEFT,true)
			installed.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
			_option_box.add_child(installed)
			count += 1
			continue
		if str(opt["group"]) != group:
			group = str(opt["group"])
			_option_box.add_child(_label(group,20,HORIZONTAL_ALIGNMENT_LEFT))
		_option_box.add_child(_option_row(opt))
		count += 1
	if count == 0:
		_option_box.add_child(_label("No adjustable installed parts in this area.",14,HORIZONTAL_ALIGNMENT_LEFT,true))

	_focus_tuning.call_deferred()

func _focus_tuning() -> void:
	for control in _option_controls.values():
		if is_instance_valid(control) and control.is_visible_in_tree() and control.focus_mode != Control.FOCUS_NONE:
			control.grab_focus()
			return

# Navigation owns focus; installed markers remain separate from the moving cursor.
func _draw_tiles(names: Array, unavailable: Array = []) -> void:
	var grid := GridContainer.new()
	grid.columns = 2
	grid.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_option_box.add_child(grid)
	for item in names:
		var name := str(item)
		var button := _button(grid,"tile:" + name,name + ("\nNot available yet" if unavailable.has(name) else ""),func(): _enter_garage_tile(name),0)
		button.custom_minimum_size = Vector2(0,112)
		button.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		button.add_theme_font_size_override("font_size",16)
		button.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		button.disabled = unavailable.has(name)
		if _garage_preferred == null and not button.disabled:
			_garage_preferred = button
	_focus_garage.call_deferred()

func _focus_garage() -> void:
	if is_instance_valid(_garage_preferred) and _garage_preferred.is_inside_tree():
		var parent := _garage_preferred.get_parent()
		if parent is GridContainer:
			var cards: Array[Button] = []
			for child in parent.get_children():
				if child is Button and not child.disabled: cards.append(child)
			for i in range(cards.size()):
				var card := cards[i]
				var columns: int = parent.columns
				card.focus_neighbor_left = card.get_path_to(cards[maxi(0,i-1)] if i % columns > 0 else card)
				card.focus_neighbor_right = card.get_path_to(cards[mini(cards.size()-1,i+1)] if i % columns < columns-1 else card)
				card.focus_neighbor_top = card.get_path_to(cards[i-columns] if i >= columns else card)
				card.focus_neighbor_bottom = card.get_path_to(cards[i+columns] if i+columns < cards.size() else card)
		_garage_preferred.grab_focus()

func _enter_garage_tile(name: String) -> void:
	_garage_path.append(name)
	var area := {"Engine":"engine", "Wheels":"wheels", "Chassis":"wheels", "Drivetrain":"rear", "Appearance & assists":"overview", "Swaps":"engine"}
	_active_area = str(area.get(_garage_path[0],"overview"))
	garage_area_chosen.emit(_active_area)
	_draw_garage_options()

func _garage_option(id: String) -> Dictionary:
	for opt in garage.get_options():
		if str(opt["id"]) == id and bool(opt["available"]):
			return opt
	return {}

func _draw_upgrades() -> void:
	_option_box.add_child(_label(" / ".join(_garage_path) if not _garage_path.is_empty() else "Upgrade categories",20,HORIZONTAL_ALIGNMENT_LEFT))
	if _garage_path.is_empty():
		_draw_tiles(["Engine", "Chassis", "Wheels", "Swaps", "Drivetrain", "Appearance & assists"])
		return
	var top := _garage_path[0]
	if top == "Wheels":
		if _garage_path.size() == 1:
			_draw_tiles(["All", "Front", "Rear"])
		elif _garage_path.size() == 2:
			_draw_tiles(["Compounds", "Dimensions"])
		else:
			_draw_wheel_parts()
		return
	if top in ["Engine", "Swaps"]:
		if _garage_path.size() == 1:
			var disabled: Array = ["Camshaft"]
			if _garage_option("engine_install").is_empty(): disabled.append("Engine")
			if _garage_option("turbo_install").is_empty() and _garage_option("turbo_install_n2o").is_empty(): disabled.append("Turbo")
			if top == "Swaps":
				_draw_tiles(["Engine", "Forced induction", "Drivetrain"], [] + (["Engine"] if disabled.has("Engine") else []) + (["Forced induction"] if disabled.has("Turbo") else []))
			else:
				_draw_tiles(["Turbo", "N2O", "Camshaft"],disabled + (["N2O"] if _garage_option("nitrous_install_hyper").is_empty() and _garage_option("nitrous_install_sedan").is_empty() else []))
			return
		if top == "Swaps" and _garage_path[1] == "Drivetrain":
			_draw_tiles(["AWD", "RWD", "FWD"],["AWD", "RWD", "FWD"])
			return
		var opt := _garage_option("engine_install") if _garage_path[1] == "Engine" else _garage_option("turbo_install")
		if opt.is_empty(): opt = _garage_option("turbo_install_n2o")
		if _garage_path[1] == "N2O":
			opt = _garage_option("nitrous_install_hyper")
			if opt.is_empty(): opt = _garage_option("nitrous_install_sedan")
		if opt.is_empty(): return
		if top == "Swaps" and _garage_path[1] == "Forced induction":
			_draw_induction_swaps(opt)
			return
		if _garage_path[1] == "Turbo" and _induction_type(str(opt["value"])) == "NA":
			_option_box.add_child(_label("Naturally aspirated engine. Install forced induction in Swaps to upgrade turbo hardware.",16,HORIZONTAL_ALIGNMENT_LEFT,true))
			return
		var grid := GridContainer.new()
		grid.columns = 2
		_option_box.add_child(grid)
		for part in opt["parts"]:
			if _garage_path[1] == "Turbo":
				if str(part["id"]) in ["hyper_touring", "hyper_balanced", "hyper_single", "hyper_touring_n2o", "hyper_balanced_n2o", "hyper_single_n2o"] and str(part["id"]) != str(opt["value"]): continue
				var single := false
				for installed_part in opt["parts"]:
					if str(installed_part["id"]) == str(opt["value"]): single = str(installed_part["id"]).contains("single")
				if str(part["id"]) == "hyper_na": continue
				if str(part["id"]).contains("single") != single: continue
			var installed := str(part["id"]) == str(opt["value"])
			var card := _make_component_card(part,"INSTALLED" if installed else "Install")
			grid.add_child(card)
			_buttons["part:" + str(opt["id"]) + ":" + str(part["id"])] = card
			card.pressed.connect(func(): _edit_option(str(opt["id"]),str(part["id"])); _draw_garage_options())
			if installed or _garage_preferred == null: _garage_preferred = card
		_focus_garage.call_deferred()
		return
	if top == "Chassis":
		if _garage_path.size() == 1:
			_draw_tiles(["Weight reduction", "Aero", "Suspension & brakes"],["Weight reduction"])
		elif _garage_path[1] == "Aero":
			_draw_tiles(["Front diffuser", "Rear wing"],["Front diffuser", "Rear wing"])
		else:
			_draw_tuning_link()
		return
	_draw_tuning_link()

func _induction_type(part_id: String) -> String:
	if part_id == "hyper_na": return "NA"
	return "Single turbo" if part_id.contains("single") else "Twin turbo"

func _draw_induction_swaps(opt: Dictionary) -> void:
	var grid := GridContainer.new()
	grid.columns = 3
	_option_box.add_child(grid)
	var current_type := _induction_type(str(opt["value"]))
	for type_name in ["NA", "Single turbo", "Twin turbo"]:
		var target: Dictionary = {}
		for part in opt["parts"]:
			if _induction_type(str(part["id"])) == type_name:
				if target.is_empty() or str(part["id"]) == str(opt["value"]): target = part
		var installed: bool = current_type == type_name
		var presentation := target.duplicate()
		presentation["label"] = type_name
		presentation["size"] = "Naturally aspirated" if type_name == "NA" else "Select hardware in Engine upgrades"
		var card := _make_component_card(presentation,"INSTALLED" if installed else "Install")
		card.disabled = target.is_empty()
		grid.add_child(card)
		_buttons["induction:" + type_name] = card
		card.pressed.connect(func():
			if not installed: _edit_option(str(opt["id"]),str(target["id"]))
			_draw_garage_options())
		if installed or _garage_preferred == null: _garage_preferred = card
	_focus_garage.call_deferred()

func _draw_tuning_link() -> void:
	_option_box.add_child(_label("Adjust installed parts in Tuning.",16,HORIZONTAL_ALIGNMENT_LEFT,true))
	var button := _button(_option_box,"tile:tune","Tune installed parts",func():
		var category := _garage_path[0]
		_set_garage_mode("tuning")
		_enter_garage_tile(category))
	_garage_preferred = button
	_focus_garage.call_deferred()

func _wheel_options() -> Array:
	var result: Array = []
	for axle in ["front", "rear"]:
		if _garage_path[1] == "All" or _garage_path[1].to_lower() == axle:
			var opt := _garage_option("tyre_" + axle)
			if not opt.is_empty(): result.append(opt)
	return result

func _wheel_choice(opt: Dictionary, family: String, width: int) -> String:
	if str(_tyre_presentation(str(opt["value"])).get("family")) == family and int(_tyre_presentation(str(opt["value"])).get("width_mm")) == width:
		return str(opt["value"])
	for choice in opt["choices"]:
		var info := _tyre_presentation(str(choice))
		if str(info.get("family")) == family and int(info.get("width_mm")) == width: return str(choice)
	return ""

func _draw_wheel_parts() -> void:
	var options := _wheel_options()
	if options.is_empty(): return
	var compounds := _garage_path[2] == "Compounds"
	var keys: Array = []
	if compounds:
		_tyre_presentation(str(options[0]["value"]))
		keys = _component_catalog.get("family_order",[])
	else:
		for choice in options[0]["choices"]:
			var width := int(_tyre_presentation(str(choice)).get("width_mm"))
			if not keys.has(width): keys.append(width)
		keys.sort()
	var grid := GridContainer.new()
	grid.columns = 3
	grid.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_option_box.add_child(grid)
	for key in keys:
		var installs: Dictionary = {}
		var installed := true
		var presentation: Dictionary = {}
		for opt in options:
			var current := _tyre_presentation(str(opt["value"]))
			var family := str(key) if compounds else str(current.get("family"))
			var width := int(current.get("width_mm")) if compounds else int(key)
			var choice := _wheel_choice(opt,family,width)
			if choice.is_empty(): break
			installs[str(opt["id"])] = choice
			installed = installed and choice == str(opt["value"])
			presentation = _tyre_presentation(choice).duplicate()
		if installs.size() != options.size(): continue
		if compounds: presentation.erase("size")
		else:
			presentation["label"] = "%d mm" % int(key)
			if options.size() == 2: presentation["size"] = "Both axles · preserves compounds"
		var card := _make_component_card(presentation,"INSTALLED" if installed else "Install")
		grid.add_child(card)
		_buttons["tyre:" + str(key)] = card
		card.pressed.connect(func():
			# Resolve both fitments before changing either axle.
			for id in installs: _edit_option(str(id),installs[id])
			_draw_garage_options())
		if installed or _garage_preferred == null: _garage_preferred = card
	_focus_garage.call_deferred()

func _make_component_card(presentation: Dictionary, action: String) -> Button:
	var card := Button.new()
	card.custom_minimum_size = Vector2(0,240)
	var focus_style := StyleBoxFlat.new()
	focus_style.bg_color = Color(0,0,0,0)
	focus_style.border_color = Color(1.0,0.55,0.2)
	focus_style.set_border_width_all(3)
	focus_style.set_corner_radius_all(4)
	card.add_theme_stylebox_override("focus",focus_style)
	var content := VBoxContainer.new()
	content.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	content.offset_left = 6
	content.offset_right = -6
	content.offset_top = 8
	content.offset_bottom = -8
	content.mouse_filter = Control.MOUSE_FILTER_IGNORE
	card.add_child(content)
	var picture := TextureRect.new()
	picture.texture = _component_texture(str(presentation.get("image", "")))
	picture.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	picture.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	picture.size_flags_vertical = Control.SIZE_EXPAND_FILL
	picture.custom_minimum_size.y = 110
	picture.mouse_filter = Control.MOUSE_FILTER_IGNORE
	content.add_child(picture)
	for text in [str(presentation.get("label", "Tyre")), str(presentation.get("size", "")), ("Rated %d km/h" % int(presentation["rated_speed_kmh"])) if presentation.has("rated_speed_kmh") else "", action]:
		if text.is_empty():
			continue
		var caption := _label(text, 12, HORIZONTAL_ALIGNMENT_CENTER)
		caption.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		caption.mouse_filter = Control.MOUSE_FILTER_IGNORE
		content.add_child(caption)
	card.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	return card

func _part_name(opt: Dictionary) -> String:
	for part in opt.get("parts", []):
		if str(part["id"]) == str(opt["value"]):
			return str(part["label"])
	return str(opt["value"])

# Presentation is explicitly mapped to a definition ID, never inferred from grip.
func _tyre_presentation(definition_id: String) -> Dictionary:
	if _component_catalog.is_empty():
		var parsed: Variant = JSON.parse_string(FileAccess.get_file_as_string("res://assets/components/tyres/catalog.json"))
		if parsed is Dictionary:
			_component_catalog = parsed
	return _component_catalog.get("parts", {}).get(definition_id, {"label":definition_id.replace("_", " ")})

func garage_wheel_widths() -> Vector2:
	var result := Vector2.ZERO
	if garage == null or screen != "configurator":
		return result
	for opt in garage.get_options():
		if str(opt["id"]) == "tyre_front":
			result.x = float(_tyre_presentation(str(opt["value"])).get("width_mm",0)) / 1000.0
		elif str(opt["id"]) == "tyre_rear":
			result.y = float(_tyre_presentation(str(opt["value"])).get("width_mm",0)) / 1000.0
	return result

func _component_texture(path: String) -> Texture2D:
	if path.is_empty():
		return null
	if not _component_textures.has(path):
		# Read the PNG directly so the first launch does not depend on editor imports.
		var source := Image.load_from_file(path)
		_component_textures[path] = ImageTexture.create_from_image(source) if source != null else null
	return _component_textures[path]

func _select_area(area_id: String, move_camera: bool) -> void:
	_active_area = area_id
	if _area_buttons.has(area_id):
		(_area_buttons[area_id] as Button).set_pressed_no_signal(true)
	_draw_garage_options()
	if move_camera:
		garage_area_chosen.emit(area_id)

func _display_value(kind: String, value: Variant) -> String:
	match kind:
		"scale":
			var f := float(value)
			return "x%.2f  (%+.0f %%)" % [f, (f - 1.0) * 100.0]
		"brake_bias":
			return "%.1f %% front" % (float(value) * 100.0)
		"bool":
			return "on" if bool(value) else "off"
		"tyre_choice":
			return str(value).replace("_", " ")
		"scale_list":
			var parts := PackedStringArray()
			for x in value:
				parts.append("%.2f" % float(x))
			return "x" + " / x".join(parts)
	return str(value)

func _option_display(opt: Dictionary, value: Variant) -> String:
	var stock: PackedFloat64Array = opt.get("stock_numbers", PackedFloat64Array())
	if not stock.is_empty():
		if str(opt["id"]) == "turbo_boost_target":
			return "%.2f bar" % (stock[0] * float(value))
		if str(opt["id"]) == "engine_rev_limit":
			return "%d rpm" % roundi(stock[0] * float(value))
		if str(opt["id"]) == "engine_throttle_response":
			return "%.0f ms" % (stock[0] * float(value) * 1000.0)
	return _display_value(str(opt["kind"]), value)

func _option_row(opt: Dictionary) -> Control:
	var id: String = str(opt["id"])
	var kind: String = str(opt["kind"])
	var holder := VBoxContainer.new()
	holder.add_theme_constant_override("separation", 1)
	holder.set_meta("kind", kind)
	holder.set_meta("option_definition", opt)
	var head := HBoxContainer.new()
	head.add_theme_constant_override("separation", 8)
	holder.add_child(head)
	var name_label := _label(str(opt["label"]), 16, HORIZONTAL_ALIGNMENT_LEFT)
	name_label.custom_minimum_size = Vector2(130, 0)
	head.add_child(name_label)
	var value_label := _label("" if kind == "scale_list" else _option_display(opt, opt["value"]), 14, HORIZONTAL_ALIGNMENT_LEFT, true)
	value_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	head.add_child(value_label)
	_option_values[id] = value_label
	match kind:
		"scale", "brake_bias":
			var slider := HSlider.new()
			var exponential_boost := str(opt["id"]) == "turbo_boost_target"
			if exponential_boost:
				slider.min_value = 0.0
				slider.max_value = 1.0
				slider.step = 0.001
				var current_bar := float(opt["stock_numbers"][0]) * float(opt["value"])
				slider.value = clampf(log(current_bar / 0.1) / log(30.0 / 0.1), 0.0, 1.0)
			else:
				slider.min_value = float(opt["min"])
				slider.max_value = float(opt["max"])
				slider.step = float(opt["step"])
				slider.value = float(opt["value"])
			slider.custom_minimum_size = Vector2(0, 22)
			slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			slider.value_changed.connect(func(x: float):
				if exponential_boost:
					var target_bar := 0.1 * pow(30.0 / 0.1, x)
					_edit_option(id, target_bar / float(opt["stock_numbers"][0]))
				else:
					_edit_option(id, x))
			holder.add_child(slider)
			_option_controls[id] = slider
		"scale_list":
			var grid := VBoxContainer.new()
			grid.add_theme_constant_override("separation", 0)
			holder.add_child(grid)
			var values: PackedFloat64Array = opt["value"]
			var stock: PackedFloat64Array = opt["stock_numbers"]
			var sliders: Array = []
			var labels: Array = []
			for i in range(int(opt["list_size"])):
				var line := HBoxContainer.new()
				line.add_theme_constant_override("separation", 6)
				grid.add_child(line)
				var gear_label := _label("Gear %d" % (i + 1), 13, HORIZONTAL_ALIGNMENT_LEFT, true)
				gear_label.custom_minimum_size = Vector2(56, 0)
				line.add_child(gear_label)
				var slider := HSlider.new()
				slider.min_value = float(opt["min"])
				slider.max_value = float(opt["max"])
				slider.step = float(opt["step"])
				slider.value = values[i]
				slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
				slider.size_flags_vertical = Control.SIZE_SHRINK_CENTER
				line.add_child(slider)
				var ratio_label := _label("", 13, HORIZONTAL_ALIGNMENT_RIGHT)
				ratio_label.custom_minimum_size = Vector2(70, 0)
				line.add_child(ratio_label)
				sliders.append(slider)
				labels.append(ratio_label)
				_option_controls["%s:%d" % [id, i]] = slider
			var push := func():
				var list := PackedFloat64Array()
				for sl in sliders:
					list.append((sl as HSlider).value)
				_edit_option(id, list)
			for sl in sliders:
				(sl as HSlider).value_changed.connect(func(_x: float): push.call())
			_option_controls[id] = grid
			holder.set_meta("gear_labels", labels)
			holder.set_meta("gear_sliders", sliders)
			holder.set_meta("gear_stock", stock)
			_refresh_gear_labels(labels, sliders, stock)
		"tyre_choice":
			var picker := OptionButton.new()
			var choices: PackedStringArray = opt["choices"]
			for c in choices:
				picker.add_item(str(c).replace("_", " "))
			picker.selected = maxi(choices.find(str(opt["value"])), 0)
			picker.item_selected.connect(func(index: int): _edit_option(id, choices[index]))
			holder.add_child(picker)
			_option_controls[id] = picker
		"bool":
			var check := CheckButton.new()
			check.button_pressed = bool(opt["value"])
			check.toggled.connect(func(on: bool): _edit_option(id, on))
			head.add_child(check)
			_option_controls[id] = check
		"colour":
			var palette: Array = PAINT_PALETTE if id == "paint" else RIM_PALETTE
			var flow := HFlowContainer.new()
			flow.add_theme_constant_override("h_separation", 6)
			flow.add_theme_constant_override("v_separation", 6)
			holder.add_child(flow)
			for hex in palette:
				var swatch := Button.new()
				swatch.custom_minimum_size = Vector2(34, 34)
				var sb := StyleBoxFlat.new()
				sb.bg_color = Color.html(str(hex))
				sb.set_corner_radius_all(4)
				sb.set_border_width_all(2)
				sb.border_color = Color(1, 1, 1, 0.25)
				swatch.add_theme_stylebox_override("normal", sb)
				var sbh := sb.duplicate() as StyleBoxFlat
				sbh.border_color = Color(1, 1, 1, 0.9)
				swatch.add_theme_stylebox_override("hover", sbh)
				swatch.add_theme_stylebox_override("pressed", sbh)
				swatch.tooltip_text = str(hex)
				swatch.pressed.connect(func(): _edit_option(id, str(hex)))
				flow.add_child(swatch)
				_buttons["%s:%s" % [id, str(hex)]] = swatch
			_option_controls[id] = flow
	var help := str(opt["help"])
	if help != "":
		holder.add_child(_label(help, 12, HORIZONTAL_ALIGNMENT_LEFT, true))
	var note := _label("", 13, HORIZONTAL_ALIGNMENT_LEFT)
	note.add_theme_color_override("font_color", Color(1.0, 0.55, 0.3))
	note.visible = false
	holder.add_child(note)
	_option_notes[id] = note
	return holder

func _refresh_gear_labels(labels: Array, sliders: Array, stock: PackedFloat64Array) -> void:
	for i in range(labels.size()):
		var factor := (sliders[i] as HSlider).value
		(labels[i] as Label).text = "%.2f" % (stock[i] * factor if i < stock.size() else factor)

# The player changed an option: RgGarage decides (type, range, the loader's verdict).
func _edit_option(option_id: String, value: Variant) -> void:
	var result: Dictionary = garage.set_option(option_id, value)
	var note: Label = _option_notes.get(option_id, null)
	if note != null:
		note.visible = not bool(result["accepted"])
		note.text = str(result["message"])
	var label: Label = _option_values.get(option_id, null)
	if label != null and result.has("value"):
		var row := label.get_parent().get_parent()
		if str(row.get_meta("kind", "")) != "scale_list":
			label.text = _option_display(row.get_meta("option_definition", {}), result["value"])
		if row.has_meta("gear_labels"):
			_refresh_gear_labels(row.get_meta("gear_labels"), row.get_meta("gear_sliders"), row.get_meta("gear_stock"))
	garage_option_changed.emit(option_id)
	_refresh_status()

# The loader's verdict on the working copy and what Save/Drive may do.
func _refresh_status() -> void:
	if _status_label == null:
		return
	var verdict: Dictionary = garage.get_validation()
	var ok := bool(verdict["ok"])
	var dirty: bool = garage.is_dirty()
	if not ok:
		_status_label.text = "Cannot save: " + str(verdict["message"])
		_status_label.add_theme_color_override("font_color", Color(1.0, 0.5, 0.3))
	elif dirty:
		_status_label.text = "Unsaved changes (valid)"
		_status_label.add_theme_color_override("font_color", Color(0.75, 0.85, 1.0))
	else:
		_status_label.text = "Saved setup" if _has_changes() else "Stock setup"
		_status_label.add_theme_color_override("font_color", Color(0.8, 0.8, 0.8))
	var save_button: Button = _buttons.get("save", null)
	if save_button != null:
		save_button.disabled = (not ok) or (not dirty)

func _has_changes() -> bool:
	for opt in garage.get_options():
		if bool(opt["modified"]):
			return true
	return false

# After Save (main.gd): refresh the status line and the buttons from RgGarage.
func refresh_configurator() -> void:
	_refresh_status()
