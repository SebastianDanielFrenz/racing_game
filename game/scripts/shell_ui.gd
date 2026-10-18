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
signal damage_repaired(state_path: String) # the garage repaired a car (main.gd repairs the running world's car too)
signal repair_requested(id: String) # the car browser's "Repair (free)" (main.gd hands it to RgShell.repair_chosen)

const SliderBox := preload("res://scripts/slider_box.gd")
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
var _option_entries: Dictionary = {}           # configurator: option id -> Array of LineEdit (typed value; one per gear of a list)
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
# Limit advisor (setup check): simulation-derived notes for the pending setup; the core decides everything.
var _advice_box: VBoxContainer
var _advice_status: Label
var _advice_list: VBoxContainer
var _advice_scroll: ScrollContainer
var _advice_poll := 0.0
var _advice_signature := ""
var _advice_data: Dictionary = {}
var _garage_path: Array[String] = []
var _garage_preferred: Button
var _engine_notice := ""                        # G1: options an engine change reset (shown under the engine cards)
var _gearbox_notice := ""                       # a gearbox change reset the gear ratios (shown under the gearbox cards)
var _driveline_notice := ""                     # a drive layout change put a differential choice back to stock
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
	_option_entries.clear()
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

# The typed-value box of an option (the gear `index` of a list), null when the row has none.
func get_option_entry(option_id: String, index: int = 0) -> LineEdit:
	var boxes: Array = _option_entries.get(option_id, [])
	return boxes[index] if index >= 0 and index < boxes.size() else null

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
		var label := "Provisional · refining" if not data.get("provisional_points",[]).is_empty() else "Updating"
		_garage_dyno_status.text = ("%s · %d%%" % [label,int(data.get("progress",0))]) if bool(data.get("busy",false)) else ("Dyno failed · previous curve retained" if not str(data.get("error","")).is_empty() else "Crank · WOT · ISA · N₂O off")
		if not bool(data.get("busy",false)) and str(data.get("error","")).is_empty():
			var simulated_turbo := false
			var mixed_turbo := false
			for point in data.get("points",[]):
				simulated_turbo = simulated_turbo or bool(point.get("turbo_simulation",false))
				mixed_turbo = mixed_turbo or float(point.get("turbo_fallback_fraction",0.0)) > 0.0
			if simulated_turbo:
				_garage_dyno_status.text += " · Turbo: sim + fallback" if mixed_turbo else " · Turbo: simulated"
		_garage_dyno_status.tooltip_text = str(data.get("error",""))
	_advice_poll -= delta
	if screen == "configurator" and _advice_box != null and is_instance_valid(_advice_box) and _advice_poll <= 0.0:
		_advice_poll = 0.2
		_refresh_advice()
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
	if control is HSlider:
		# the slider's number can be typed too (garage numeric inputs); the slider stays the one source of truth
		SliderBox.attach(row, control as HSlider, _step_decimals(float(def["step"])))
	if show_value:
		var value_label := _label(str(shell.get_setting_display(key)), 16, HORIZONTAL_ALIGNMENT_LEFT)
		value_label.custom_minimum_size = Vector2(110, 0)
		row.add_child(value_label)
		_setting_displays[key] = value_label
	var help: String = str(def["help"])
	if bool(def.get("applies_on_restart", false)):
		help += " (applies when the game is started again)"
	elif not bool(def["applies_live"]):
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
	ui.repair_requested.connect(func(id: String): repair_requested.emit(id))
	_root.add_child(ui)
	_browser = ui
	_buttons["choose"] = ui.get_control("choose")
	_buttons["back"] = ui.get_control("back")
	_buttons["filter"] = ui.get_control("filter_button")

# The car browser's stats again (after a repair).
func refresh_browser() -> void:
	if _browser != null and is_instance_valid(_browser) and _browser.has_method("refresh_stats"):
		_browser.refresh_stats()

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
	scroll.custom_minimum_size.y = 150.0 # the setup check below never squeezes the options away
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	column.add_child(scroll)
	_option_box = VBoxContainer.new()
	_option_box.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_option_box.add_theme_constant_override("separation",12)
	scroll.add_child(_option_box)
	_build_advice_panel(column)
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
	var focus := get_viewport().gui_get_focus_owner()
	if focus is LineEdit and focus.has_meta("entry"):
		# Esc in a value box takes the typing back (and leaves the box) before it leaves the page.
		(focus as LineEdit).text = str(focus.get_meta("shown", ""))
		focus.release_focus()
		return
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
	_option_entries.clear()
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
		if str(opt["id"]) == "clutch_capacity": continue
		if str(opt["area"]) != _active_area or not bool(opt["available"]):
			continue
		if str(opt["kind"]) in ["tyre_choice", "file_choice", "part_choice"]:
			_option_box.add_child(_label("Installed " + str(opt["label"]).to_lower(),17,HORIZONTAL_ALIGNMENT_LEFT))
			var installed := _label(_part_name(opt) if str(opt["kind"]) != "tyre_choice" else _display_value("tyre_choice",opt["value"]),14,HORIZONTAL_ALIGNMENT_LEFT,true)
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

# ---- garage: setup check (limit advisor) -------------------------------------------------------------------------

const ADVICE_COLOURS := [Color(0.62, 0.78, 0.95), Color(0.98, 0.72, 0.22), Color(0.95, 0.32, 0.28)]
const ADVICE_MAX_HEIGHT := 140.0

func _build_advice_panel(column: VBoxContainer) -> void:
	_advice_box = VBoxContainer.new()
	_advice_box.name = "SetupCheck"
	_advice_box.add_theme_constant_override("separation", 4)
	column.add_child(_advice_box)
	_advice_box.add_child(HSeparator.new())
	_advice_status = _label("SETUP CHECK", 14, HORIZONTAL_ALIGNMENT_LEFT, true)
	_advice_box.add_child(_advice_status)
	_advice_scroll = ScrollContainer.new()
	_advice_scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	_advice_scroll.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_advice_scroll.size_flags_vertical = Control.SIZE_SHRINK_BEGIN
	_advice_box.add_child(_advice_scroll)
	_advice_list = VBoxContainer.new()
	_advice_list.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_advice_list.add_theme_constant_override("separation", 6)
	_advice_scroll.add_child(_advice_list)
	_advice_signature = ""
	_advice_poll = 0.0

# The advisor's findings never block an edit (owner 2026-10-13: allow extreme setups, show warnings).
func _refresh_advice() -> void:
	if not garage.has_method("get_advice"):
		_advice_status.text = "Setup check pending · restart after installation"
		return
	var data: Dictionary = garage.get_advice()
	_advice_data = data
	if not bool(data.get("available", false)):
		_advice_status.text = "SETUP CHECK · unavailable"
		_advice_status.tooltip_text = str(data.get("error", ""))
		return
	var busy := bool(data.get("busy", false))
	var items: Array = data.get("items", [])
	var worst := -1
	for item in items:
		worst = maxi(worst, int(item.get("severity", 0)))
	var text := "SETUP CHECK"
	if busy:
		text += " · checking… %d%%" % int(data.get("progress", 0))
	elif not str(data.get("error", "")).is_empty():
		text += " · check failed"
	elif bool(data.get("have_result", false)):
		text += " · %s" % ("no issues found" if items.is_empty() else ("%d note%s" % [items.size(), "" if items.size() == 1 else "s"]))
	_advice_status.text = text
	_advice_status.tooltip_text = str(data.get("error", "")) if not busy else str(data.get("stage", ""))
	_advice_status.modulate = ADVICE_COLOURS[worst] if (worst >= 0 and not busy) else Color(1, 1, 1, 0.7)
	var signature := "%d|%s|%s|%d" % [int(data.get("revision", 0)), str(data.get("vehicle_id", "")), str(busy), 1 if bool(data.get("solving", false)) else 0]
	for item in items:
		signature += "|%s:%d" % [str(item.get("id", "")), int(item.get("remedy_state", 0))]
	if signature == _advice_signature:
		return
	_advice_signature = signature
	_draw_advice(items, busy)

func _draw_advice(items: Array, stale: bool) -> void:
	for key in _buttons.keys():
		if str(key).begins_with("advice_fix:"):
			_buttons.erase(key)
	for child in _advice_list.get_children():
		_advice_list.remove_child(child)
		child.queue_free()
	for item in items:
		_advice_list.add_child(_advice_item_control(item, stale))
	# A fixed band (items scroll inside it): the option list above never loses its room.
	_advice_scroll.custom_minimum_size.y = ADVICE_MAX_HEIGHT if not items.is_empty() else 0.0
	_advice_scroll.visible = not items.is_empty()

func _advice_item_control(item: Dictionary, stale: bool) -> Control:
	var colour: Color = ADVICE_COLOURS[clampi(int(item.get("severity", 0)), 0, 2)]
	var panel := PanelContainer.new()
	panel.set_meta("advice_id", str(item.get("id", "")))
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.03, 0.04, 0.055, 0.82)
	style.border_color = colour
	style.border_width_left = 4
	style.set_content_margin_all(8)
	style.content_margin_left = 12
	panel.add_theme_stylebox_override("panel", style)
	if stale:
		panel.modulate = Color(1, 1, 1, 0.55)
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 2)
	panel.add_child(box)
	var title := _label(str(item.get("title", "")), 15, HORIZONTAL_ALIGNMENT_LEFT)
	title.add_theme_color_override("font_color", colour)
	box.add_child(title)
	for line in item.get("lines", []):
		box.add_child(_label(str(line), 13, HORIZONTAL_ALIGNMENT_LEFT))
	var suggestion := str(item.get("suggestion", ""))
	if not suggestion.is_empty():
		box.add_child(_label(suggestion, 13, HORIZONTAL_ALIGNMENT_LEFT, true))
	var state := int(item.get("remedy_state", 0))
	var remedy_text := str(item.get("remedy_text", ""))
	if not remedy_text.is_empty() and state in [1, 4, 5]:
		var fix_label := _label(remedy_text, 13, HORIZONTAL_ALIGNMENT_LEFT)
		fix_label.add_theme_color_override("font_color", Color(0.55, 0.88, 0.6) if state != 5 else Color(0.98, 0.72, 0.22))
		box.add_child(fix_label)
	if bool(item.get("can_solve", false)) and state in [2, 3]:
		var id := str(item.get("id", ""))
		var button := Button.new()
		button.text = "Searching…" if state == 3 else "Find fix"
		button.disabled = state == 3
		button.add_theme_font_size_override("font_size", 14)
		button.size_flags_horizontal = Control.SIZE_SHRINK_BEGIN
		button.pressed.connect(func():
			garage.solve_advice(id)
			button.text = "Searching…"
			button.disabled = true)
		box.add_child(button)
		_buttons["advice_fix:" + id] = button
	return panel

# Test hooks: the advisor's last polled view-model.
func advice_data() -> Dictionary:
	return _advice_data

func advice_status_text() -> String:
	return _advice_status.text if _advice_status != null and is_instance_valid(_advice_status) else ""

func advice_item_count() -> int:
	return _advice_list.get_child_count() if _advice_list != null and is_instance_valid(_advice_list) else 0

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

# The turbo kit option (core turbo_kit.h) whether it is available or not: an unavailable one carries the reason.
func _turbo_kit_option() -> Dictionary:
	for opt in garage.get_options():
		if str(opt["id"]) == "turbo_kit" and (bool(opt["available"]) or not str(opt.get("unavailable_reason", "")).is_empty()):
			return opt
	return {}

# Turbo kits for an engine without a turbo of its own: generic cards, or the reason none can be fitted.
func _draw_turbo_kit(kit: Dictionary) -> void:
	if not bool(kit["available"]):
		var why := _label(str(kit.get("unavailable_reason", "")),16,HORIZONTAL_ALIGNMENT_LEFT,true)
		why.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		_option_box.add_child(why)
		return
	var note := _label("Turbo, intercooler and wastegate sized for the installed engine.",13,HORIZONTAL_ALIGNMENT_LEFT,true)
	note.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_option_box.add_child(note)
	var grid := GridContainer.new()
	grid.columns = 2
	_option_box.add_child(grid)
	_add_hidden_note(kit)
	for part in kit["parts"]:
		var installed := str(part["id"]) == str(kit["value"])
		if not _part_visible(part) and not installed: continue
		var presentation: Dictionary = part.duplicate()
		presentation["size"] = str(part.get("detail", ""))
		var card := _make_component_card(presentation,"INSTALLED" if installed else "Install")
		_grey_if_unfit(card, part)
		grid.add_child(card)
		_buttons["part:turbo_kit:" + str(part["id"])] = card
		card.pressed.connect(func(): _edit_option("turbo_kit",str(part["id"])); _draw_garage_options())
		if installed or _garage_preferred == null: _garage_preferred = card
	# Charge piping (plastic / aluminium / steel): its rating is what the setup check holds the boost against.
	var piping := _garage_option("charge_piping")
	if not piping.is_empty() and str(kit["value"]) != "stock":
		_option_box.add_child(_label("Charge piping",16,HORIZONTAL_ALIGNMENT_LEFT,true))
		var pgrid := GridContainer.new()
		pgrid.columns = 2
		_option_box.add_child(pgrid)
		for part in piping["parts"]:
			var fitted := str(part["id"]) == str(piping["value"])
			var shown: Dictionary = part.duplicate()
			shown["size"] = str(part.get("detail", ""))
			var pcard := _make_component_card(shown,"INSTALLED" if fitted else "Install")
			pgrid.add_child(pcard)
			_buttons["part:charge_piping:" + str(part["id"])] = pcard
			pcard.pressed.connect(func(): _edit_option("charge_piping",str(part["id"])); _draw_garage_options())
	_draw_swap_summary()
	_focus_garage.call_deferred()

const INTERNALS_OPTIONS := ["engine_bore", "engine_rods", "engine_pistons", "engine_crankshaft", "engine_valve_springs"]
const COOLING_OPTIONS := ["cooling_radiator", "cooling_fan", "cooling_oil_cooler", "cooling_block_cooling"]

# An option whether it is available or not (an unavailable one that the car offers carries its reason); {} when absent.
func _any_option(id: String) -> Dictionary:
	for opt in garage.get_options():
		if str(opt["id"]) == id and (bool(opt["available"]) or not str(opt.get("unavailable_reason", "")).is_empty()):
			return opt
	return {}

func _cooling_offered() -> bool:
	for id in COOLING_OPTIONS:
		if not _any_option(id).is_empty(): return true
	return false

func _internals_offered() -> bool:
	for id in INTERNALS_OPTIONS:
		if not _any_option(id).is_empty(): return true
	return false

# Engine internals (core engine_internals.h): one row of grade cards per part. The engine's piston-speed limit is its
# weakest part's; the setup check names the part that would fail. Nothing breaks yet (no damage model).
func _draw_internals() -> void:
	_draw_grade_rows(INTERNALS_OPTIONS, "A bigger bore adds displacement and takes new pistons of that size. The safe piston speed is that of the weakest part: rods, pistons and crankshaft. Valve springs set the rpm the valves follow the cam to.")

# Cooling (core cooling_upgrades.h): more fuel burned means more heat; radiator, fan, oil cooler and block cooling carry it off.
# Each grade card shows its numbers and the mass it adds; the setup check warns when the coolant or oil would run too hot.
func _draw_cooling() -> void:
	_draw_damage_panel()
	_draw_grade_rows(COOLING_OPTIONS, "More fuel burned means more heat for the engine. Bigger cooling parts carry it off and weigh more. The setup check warns when the coolant or oil would run too hot at full power.")

# Damage (core damage.h; owner rules: damage stays until repaired, repairs are free and need this button). Shown only while a
# part of the edited car is worn: one line per part with its health and what it does, and the Repair button.
func _draw_damage_panel() -> void:
	if not garage.has_method("get_damage"): return
	var id := str(garage.get_edit_id())
	var damage: Dictionary = garage.get_damage(id)
	if not bool(damage.get("present", false)) or not bool(damage.get("any_damaged", false)): return
	_option_box.add_child(_label("Damage",16,HORIZONTAL_ALIGNMENT_LEFT,true))
	for part in damage.get("parts", []):
		if not bool(part.get("damaged", false)): continue
		var line := _label("%s: %.0f %%  -  %s" % [str(part["label"]), float(part["health"]) * 100.0, str(part["description"])],14,HORIZONTAL_ALIGNMENT_LEFT,true)
		line.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		_option_box.add_child(line)
	_button(_option_box, "repair_damage", "Repair (free)", func():
		var reply: Dictionary = garage.repair_damage(id)
		if bool(reply.get("ok", false)):
			damage_repaired.emit(str(damage.get("state_path", "")))
		else:
			push_warning("RG_DAMAGE repair failed: %s" % str(reply.get("error", "")))
		_draw_garage_options())

# One row of grade cards per option id. A part the car cannot fit (fits = false) is hidden unless the player asks for all parts.
func _draw_grade_rows(ids: Array, note_text: String) -> void:
	var note := _label(note_text,13,HORIZONTAL_ALIGNMENT_LEFT,true)
	note.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_option_box.add_child(note)
	for id in ids:
		var opt := _any_option(id)
		if opt.is_empty(): continue
		_option_box.add_child(_label(str(opt["label"]),16,HORIZONTAL_ALIGNMENT_LEFT,true))
		if not bool(opt["available"]):
			var why := _label(str(opt.get("unavailable_reason", "")),14,HORIZONTAL_ALIGNMENT_LEFT,true)
			why.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
			_option_box.add_child(why)
			continue
		var grid := GridContainer.new()
		grid.columns = 2
		_option_box.add_child(grid)
		_add_hidden_note(opt)
		for part in opt["parts"]:
			var fitted := str(part["id"]) == str(opt["value"])
			if not _part_visible(part) and not fitted: continue
			var shown: Dictionary = part.duplicate()
			shown["size"] = str(part.get("detail", ""))
			var card := _make_component_card(shown,"INSTALLED" if fitted else "Install")
			_grey_if_unfit(card, part)
			grid.add_child(card)
			_buttons["part:" + id + ":" + str(part["id"])] = card
			var option_id: String = id
			card.pressed.connect(func(): _edit_option(option_id,str(part["id"])); _draw_garage_options())
			if fitted and _garage_preferred == null: _garage_preferred = card
	_draw_swap_summary()
	_focus_garage.call_deferred()

# "Show all" (hidden setting garage.show_all_upgrades, default off): by default an upgrade list holds only what the car can
# fit; core decides (part "fits"/"fit_reason", option "hidden_reason"), this only filters.
func _show_all_upgrades() -> bool:
	return shell != null and bool(shell.get_setting("garage.show_all_upgrades"))

func _part_visible(part: Dictionary) -> bool:
	return bool(part.get("fits", true)) or _show_all_upgrades()

# A card of a part the car cannot fit (shown only with "show all"): greyed out, the reason as its text, not installable.
func _grey_if_unfit(card: Control, part: Dictionary) -> void:
	if bool(part.get("fits", true)): return
	card.modulate = Color(1, 1, 1, 0.5)
	if card is BaseButton: card.disabled = true
	card.tooltip_text = str(part.get("fit_reason", ""))

# One line saying why a list is empty: every choice was hidden because the car cannot fit it.
func _add_hidden_note(opt: Dictionary) -> void:
	var why := str(opt.get("hidden_reason", ""))
	if why.is_empty() or _show_all_upgrades(): return
	var shown_any := false
	for part in opt["parts"]:
		if str(part["id"]) != "stock" and _part_visible(part): shown_any = true
	if shown_any: return
	var note := _label("%s: nothing fits. %s" % [str(opt["label"]), why],14,HORIZONTAL_ALIGNMENT_LEFT,true)
	note.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_option_box.add_child(note)

func _draw_show_all_toggle() -> void:
	var check := CheckButton.new()
	check.text = "Show all (including what does not fit)"
	check.button_pressed = _show_all_upgrades()
	check.toggled.connect(func(on: bool):
		shell.set_setting("garage.show_all_upgrades", on)
		shell.save_settings()
		_draw_garage_options())
	_option_box.add_child(check)
	_buttons["show_all_upgrades"] = check

func _draw_powertrain_swaps() -> void:
	var opt := _garage_option("powertrain_install")
	if opt.is_empty():
		return
	if _garage_path.size() == 2:
		_draw_tiles(["Internal combustion", "EV", "Hybrid"])
		return
	var category: String = _garage_path[2]
	if category == "Internal combustion" and _garage_path.size() == 3:
		_draw_tiles(["Gasoline", "Diesel", "Hydrogen"])
		return
	var parts: Array = opt["parts"]
	var option_id := "powertrain_install"
	var value: String = str(opt["value"])
	if category == "Internal combustion":
		# Probe the ICE conversion's engine list even when the installed car is electric.
		var engine_parts: Array = garage.get_powertrain_engine_choices()
		parts = engine_parts
		option_id = "engine_install"
		value = str(_garage_option("engine_install").get("value", "")) if str(opt["value"]) == "ice" else ""
	var grid := GridContainer.new()
	grid.columns = 2
	_option_box.add_child(grid)
	for part in parts:
		var section: String = str(part.get("section", ""))
		if category == "Internal combustion":
			if section != _garage_path[3]:
				continue
		elif section != category:
			continue
		var installed := str(part["id"]) == value
		if not _part_visible(part) and not installed:
			continue
		var card := _make_component_card(part, "INSTALLED" if installed else "Install")
		_grey_if_unfit(card, part)
		grid.add_child(card)
		_buttons["part:" + option_id + ":" + str(part["id"])] = card
		card.pressed.connect(func():
			if category == "Internal combustion":
				_edit_option("powertrain_install", "ice")
			_edit_option(option_id, str(part["id"]))
			_draw_garage_options())
		if installed or _garage_preferred == null:
			_garage_preferred = card
	_draw_swap_summary()
	_focus_garage.call_deferred()

func _draw_upgrades() -> void:
	_option_box.add_child(_label(" / ".join(_garage_path) if not _garage_path.is_empty() else "Upgrade categories",20,HORIZONTAL_ALIGNMENT_LEFT))
	_draw_show_all_toggle()
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
	if top == "Swaps" and _garage_path.size() >= 2 and _garage_path[1] == "Powertrain":
		_draw_powertrain_swaps()
		return
	if top in ["Engine", "Swaps"]:
		if _garage_path.size() == 1:
			var disabled: Array = ["Camshaft"]
			if _garage_option("engine_install").is_empty(): disabled.append("Engine")
			if _garage_option("turbo_install").is_empty() and _garage_option("turbo_install_n2o").is_empty() and _turbo_kit_option().is_empty(): disabled.append("Turbo")
			if top == "Swaps":
				var no_fuel: bool = _garage_option("fuel").is_empty()
				var no_drivetrain: bool = _garage_option("drive_layout").is_empty() and _garage_option("diff_front").is_empty() and _garage_option("diff_rear").is_empty()
				_draw_tiles(["Powertrain", "Engine", "Fuel", "Forced induction", "Drivetrain"], [] + (["Engine"] if disabled.has("Engine") else []) + (["Fuel"] if no_fuel else []) + (["Forced induction"] if disabled.has("Turbo") else []) + (["Drivetrain"] if no_drivetrain else []))
			else:
				_draw_tiles(["Turbo", "N2O", "Internals", "Cooling", "Camshaft"],disabled + (["N2O"] if _garage_option("nitrous_install_hyper").is_empty() and _garage_option("nitrous_install_sedan").is_empty() else []) + ([] if _internals_offered() else ["Internals"]) + ([] if _cooling_offered() else ["Cooling"]))
			return
		if top == "Engine" and _garage_path[1] == "Internals":
			_draw_internals()
			return
		if top == "Engine" and _garage_path[1] == "Cooling":
			_draw_cooling()
			return
		if top == "Swaps" and _garage_path[1] == "Drivetrain":
			_draw_drivetrain_swaps()
			return
		var opt := _garage_option("engine_install") if _garage_path[1] == "Engine" else _garage_option("turbo_install")
		if _garage_path[1] == "Fuel": opt = _garage_option("fuel")
		if opt.is_empty() and _garage_path[1] != "Fuel": opt = _garage_option("turbo_install_n2o")
		if _garage_path[1] == "N2O":
			opt = _garage_option("nitrous_install_hyper")
			if opt.is_empty(): opt = _garage_option("nitrous_install_sedan")
		if opt.is_empty() and _garage_path[1] in ["Turbo", "Forced induction"] and not _turbo_kit_option().is_empty():
			_draw_turbo_kit(_turbo_kit_option())
			return
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
			if not _part_visible(part) and not installed: continue
			var card := _make_component_card(part,"INSTALLED" if installed else "Install")
			_grey_if_unfit(card, part)
			grid.add_child(card)
			_buttons["part:" + str(opt["id"]) + ":" + str(part["id"])] = card
			card.pressed.connect(func(): _edit_option(str(opt["id"]),str(part["id"])); _draw_garage_options())
			if installed or _garage_preferred == null: _garage_preferred = card
		if top == "Swaps" and _garage_path[1] in ["Engine", "Fuel"]:
			_draw_swap_summary()
		_focus_garage.call_deferred()
		return
	if top == "Drivetrain":
		var clutch := _garage_option("clutch_capacity")
		var gearbox := _garage_option("gearbox_install")
		if _garage_path.size() == 1:
			_draw_tiles(["Clutch", "Gearbox", "Gearing"], (["Clutch"] if clutch.is_empty() else []) + (["Gearbox"] if gearbox.is_empty() else []))
		elif _garage_path[1] == "Clutch":
			_draw_clutch_upgrades(clutch)
		elif _garage_path[1] == "Gearbox":
			_draw_gearbox_choices(gearbox)
		else:
			_draw_tuning_link()
		return
	if top == "Chassis":
		if _garage_path.size() == 1:
			_draw_tiles(["Weight", "Weight reduction", "Aero", "Suspension & brakes"],["Weight reduction"] if _weight_reduction_options().is_empty() else [])
		elif _garage_path[1] == "Weight":
			_draw_mass_breakdown()
		elif _garage_path[1] == "Weight reduction":
			_draw_weight_reduction()
		elif _garage_path[1] == "Aero":
			_draw_tiles(["Front diffuser", "Rear wing"],["Front diffuser", "Rear wing"])
		else:
			_draw_tuning_link()
		return
	_draw_tuning_link()

# The part-mass tree of the car being edited (RgGarage.get_mass_breakdown): total, change against stock, the body and
# one row per group, then the drivetrain's parts (a layout conversion shows up there).
func _draw_mass_breakdown() -> void:
	var info: Dictionary = garage.get_mass_breakdown(garage.get_edit_id())
	var box := VBoxContainer.new()
	box.name = "MassBreakdown"
	_option_box.add_child(box)
	if not bool(info.get("ok", false)):
		box.add_child(_label("Weight not available: " + str(info.get("error", "")), 14, HORIZONTAL_ALIGNMENT_LEFT, true))
		return
	var total := float(info["total_kg"])
	var delta := total - float(info.get("stock_kg", total))
	var head := "Total %.0f kg" % total
	if absf(delta) >= 0.5:
		head += "  (%+.0f kg vs stock)" % delta
	box.add_child(_label(head, 16, HORIZONTAL_ALIGNMENT_LEFT))
	for group in info["groups"]:
		box.add_child(_label("%s  %.0f kg" % [str(group["label"]), float(group["kg"])], 14, HORIZONTAL_ALIGNMENT_LEFT))
	box.add_child(_label("Drivetrain parts", 16, HORIZONTAL_ALIGNMENT_LEFT))
	for part in info["parts"]:
		var id := str(part["id"])
		if not id.begins_with("drivetrain.") or int(part["depth"]) > 3: continue
		var indent := "    ".repeat(int(part["depth"]) - 2)
		box.add_child(_label("%s%s  %.0f kg" % [indent, str(part["label"]), float(part["total_kg"])], 14, HORIZONTAL_ALIGNMENT_LEFT, true))

# The part choices (setup_options.json kind part_choice, group "Weight reduction") this car offers: one row of cards per
# slot, the fitted part marked. The model shows the removal where it has a node for the part (data/vehicles/strip_visual.json).
func _weight_reduction_options() -> Array:
	var out: Array = []
	for opt in garage.get_options():
		if str(opt["kind"]) == "part_choice" and bool(opt["available"]):
			out.append(opt)
	return out

func _draw_weight_reduction() -> void:
	var info: Dictionary = garage.get_mass_breakdown(garage.get_edit_id())
	if bool(info.get("ok", false)):
		var total := float(info["total_kg"])
		var delta := total - float(info.get("stock_kg", total))
		_option_box.add_child(_label("Total %.0f kg%s" % [total, ("  (%+.0f kg vs stock)" % delta) if absf(delta) >= 0.5 else ""], 16, HORIZONTAL_ALIGNMENT_LEFT))
	for opt in _weight_reduction_options():
		_option_box.add_child(_label(str(opt["label"]), 17, HORIZONTAL_ALIGNMENT_LEFT))
		var grid := GridContainer.new()
		grid.columns = 3
		grid.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		_option_box.add_child(grid)
		for part in opt["parts"]:
			var installed := str(part["id"]) == str(opt["value"])
			var presentation: Dictionary = part.duplicate()
			presentation["badge_option"] = str(opt["id"])
			presentation["badge_part"] = str(part["id"])
			var card := _make_component_card(presentation, "INSTALLED" if installed else "Install")
			grid.add_child(card)
			var opt_id := str(opt["id"])
			var part_id := str(part["id"])
			_buttons["part:" + opt_id + ":" + part_id] = card
			card.pressed.connect(func(): _edit_option(opt_id, part_id); _draw_garage_options())
			if _garage_preferred == null: _garage_preferred = card
	_focus_garage.call_deferred()

# G1: what the engine/fuel choice changed (fuel, clutch, shift points, weight) and what an engine change reset.
func _draw_swap_summary() -> void:
	var info: Dictionary = garage.get_swap_info()
	var lines: Array = []
	for line in info.get("lines", []):
		lines.append(str(line))
	if not _engine_notice.is_empty():
		lines.append(_engine_notice)
	if lines.is_empty():
		return
	var box := VBoxContainer.new()
	box.name = "SwapSummary"
	_option_box.add_child(box)
	box.add_child(_label("This swap changes", 16, HORIZONTAL_ALIGNMENT_LEFT))
	for line in lines:
		box.add_child(_label("- " + str(line), 14, HORIZONTAL_ALIGNMENT_LEFT, true))

func _draw_clutch_upgrades(opt: Dictionary) -> void:
	if opt.is_empty(): return
	var numbers: Array = opt.get("stock_numbers", [])
	if numbers.is_empty(): return
	var grid := GridContainer.new()
	grid.columns = 2
	_option_box.add_child(grid)
	var levels := [{"label":"Stock", "factor":1.0}, {"label":"Reinforced", "factor":1.5}, {"label":"Competition", "factor":2.0}, {"label":"Heavy duty", "factor":3.0}, {"label":"Extreme", "factor":5.0}, {"label":"Drag competition", "factor":8.0}]
	for level in levels:
		var factor := float(level["factor"])
		if factor < float(opt["min"]) or factor > float(opt["max"]): continue
		var installed := is_equal_approx(float(opt["value"]),factor)
		var presentation := {"label":level["label"], "size":"Torque capacity · %.0f Nm" % (float(numbers[0])*factor), "image":"res://assets/components/drivetrain/clutch.svg"}
		var card := _make_component_card(presentation,"INSTALLED" if installed else "Install")
		grid.add_child(card)
		_buttons["clutch:" + str(factor)] = card
		card.pressed.connect(func(): _edit_option("clutch_capacity",factor); _draw_garage_options())
		if installed or _garage_preferred == null: _garage_preferred = card
	_focus_garage.call_deferred()

# The gearbox swap: the compatible boxes in sections (Manual incl. sequential, Automatic, Dual-clutch, CVT). A section with no entries
# yet says "Coming soon". The list is already filtered by the torque rating against the installed engine (RgGarage).
func _draw_gearbox_choices(opt: Dictionary) -> void:
	if opt.is_empty(): return
	var by_section: Dictionary = {}
	for part in opt["parts"]:
		var section := str(part.get("section", ""))
		if section.is_empty(): section = "Manual"
		if not by_section.has(section): by_section[section] = []
		by_section[section].append(part)
	var order: Array = ["Manual", "Automatic", "Dual-clutch", "CVT"]
	for section in by_section.keys():
		if not order.has(section): order.append(section)
	var note := _label("Only gearboxes rated for the installed engine's torque are listed.",13,HORIZONTAL_ALIGNMENT_LEFT,true)
	note.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_option_box.add_child(note)
	for section in order:
		_option_box.add_child(_label(str(section),18,HORIZONTAL_ALIGNMENT_LEFT))
		if not by_section.has(section):
			var soon := PackedStringArray(opt.get("coming_soon", [])).has(str(section))
			_option_box.add_child(_label("Coming soon" if soon else "No gearbox in this section is rated for the installed engine.",14,HORIZONTAL_ALIGNMENT_LEFT,true))
			continue
		var grid := GridContainer.new()
		grid.columns = 2
		grid.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		_option_box.add_child(grid)
		for part in by_section[section]:
			var installed := str(part["id"]) == str(opt["value"])
			var presentation: Dictionary = part.duplicate()
			presentation["size"] = (str(part.get("subtitle", "")) + "\n" + str(part.get("size", ""))).strip_edges()
			var card := _make_component_card(presentation,"INSTALLED" if installed else "Install")
			grid.add_child(card)
			_buttons["part:" + str(opt["id"]) + ":" + str(part["id"])] = card
			card.pressed.connect(func(): _edit_option(str(opt["id"]),str(part["id"])); _draw_garage_options())
			if installed or _garage_preferred == null: _garage_preferred = card
	for text in [_gearbox_notice, _engine_notice]:
		if not str(text).is_empty():
			var line := _label(str(text),14,HORIZONTAL_ALIGNMENT_LEFT,true)
			line.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
			_option_box.add_child(line)
	_focus_garage.call_deferred()

# The drivetrain swap: the drive layouts by section (FWD / RWD / AWD with its centre differential), then the axle
# differentials of the driven axles. RgGarage decides what fits; the first card of each list is what the car has now.
func _draw_drivetrain_swaps() -> void:
	var layout := _garage_option("drive_layout")
	if layout.is_empty():
		_option_box.add_child(_label("This car's driveline cannot be swapped.",14,HORIZONTAL_ALIGNMENT_LEFT,true))
	else:
		var by_section: Dictionary = {}
		for part in layout["parts"]:
			var section := str(part.get("section", ""))
			if not by_section.has(section): by_section[section] = []
			by_section[section].append(part)
		for section in ["FWD", "RWD", "AWD"]:
			if not by_section.has(section): continue
			_option_box.add_child(_label(str(section),18,HORIZONTAL_ALIGNMENT_LEFT))
			_draw_part_grid(layout, by_section[section])
	for id in ["diff_front", "diff_rear"]:
		var opt := _garage_option(id)
		if opt.is_empty(): continue
		_option_box.add_child(_label(str(opt.get("label", id)),18,HORIZONTAL_ALIGNMENT_LEFT))
		_draw_part_grid(opt, opt["parts"])
	if not _driveline_notice.is_empty():
		var line := _label(_driveline_notice,14,HORIZONTAL_ALIGNMENT_LEFT,true)
		line.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		_option_box.add_child(line)
	_focus_garage.call_deferred()

func _draw_part_grid(opt: Dictionary, parts: Array) -> void:
	var grid := GridContainer.new()
	grid.columns = 2
	grid.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_option_box.add_child(grid)
	for part in parts:
		var installed := str(part["id"]) == str(opt["value"])
		var presentation: Dictionary = part.duplicate()
		presentation["size"] = (str(part.get("subtitle", "")) + "\n" + str(part.get("detail", ""))).strip_edges()
		var card := _make_component_card(presentation,"INSTALLED" if installed else "Install")
		grid.add_child(card)
		_buttons["part:" + str(opt["id"]) + ":" + str(part["id"])] = card
		card.pressed.connect(func(): _edit_option(str(opt["id"]),str(part["id"])); _draw_garage_options())
		if installed or _garage_preferred == null: _garage_preferred = card

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

# One part tile (owner 2026-10-18): the picture, the part's name and an INSTALLED mark, all INSIDE the tile. The tile has a
# fixed height and the name lives in a band at its bottom, clipped, so no text can leave the tile or reach the next row.
# A highlighted tile (focus or hover) shows its numeric upgrade badges (rg/upgrade_badges.h, through RgGarage.get_part_badges)
# at its bottom middle: red when the part is worse for that number, green when better. The action text ("Install") is gone.
const TILE_HEIGHT := 150.0
const TILE_NAME_BAND := 40.0
const BADGE_WORSE := Color(0.78, 0.16, 0.16, 0.85)
const BADGE_BETTER := Color(0.16, 0.6, 0.25, 0.85)

func _make_component_card(presentation: Dictionary, action: String) -> Button:
	var card := Button.new()
	card.custom_minimum_size = Vector2(0, TILE_HEIGHT)
	card.clip_contents = true
	card.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var focus_style := StyleBoxFlat.new()
	focus_style.bg_color = Color(0,0,0,0)
	focus_style.border_color = Color(1.0,0.55,0.2)
	focus_style.set_border_width_all(3)
	focus_style.set_corner_radius_all(4)
	card.add_theme_stylebox_override("focus",focus_style)
	var picture := TextureRect.new()
	picture.texture = _component_texture(str(presentation.get("image", "")))
	picture.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	picture.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	picture.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	picture.offset_left = 6
	picture.offset_right = -6
	picture.offset_top = 6
	picture.offset_bottom = -TILE_NAME_BAND
	picture.mouse_filter = Control.MOUSE_FILTER_IGNORE
	card.add_child(picture)
	var name_label := _label(str(presentation.get("label", "Tyre")), 12, HORIZONTAL_ALIGNMENT_CENTER)
	name_label.name = "Name"
	name_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	name_label.clip_text = true
	name_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	name_label.set_anchors_and_offsets_preset(Control.PRESET_BOTTOM_WIDE)
	name_label.offset_top = -TILE_NAME_BAND
	name_label.offset_bottom = -2
	card.add_child(name_label)
	if action == "INSTALLED":
		var mark := _label("INSTALLED", 10)
		mark.name = "Installed"
		mark.autowrap_mode = TextServer.AUTOWRAP_OFF
		mark.mouse_filter = Control.MOUSE_FILTER_IGNORE
		mark.add_theme_stylebox_override("normal", _chip_style(Color(0,0,0,0.6)))
		mark.set_anchors_and_offsets_preset(Control.PRESET_TOP_LEFT)
		mark.offset_left = 4
		mark.offset_top = 4
		card.add_child(mark)
	var opt_id := str(presentation.get("badge_option", ""))
	if not opt_id.is_empty():
		card.set_meta("badge_option", opt_id)
		card.set_meta("badge_part", str(presentation.get("badge_part", "")))
		var row := HBoxContainer.new()
		row.name = "Badges"
		row.alignment = BoxContainer.ALIGNMENT_CENTER
		row.mouse_filter = Control.MOUSE_FILTER_IGNORE
		row.visible = false
		row.set_anchors_and_offsets_preset(Control.PRESET_BOTTOM_WIDE)
		row.offset_top = -TILE_NAME_BAND - 26
		row.offset_bottom = -TILE_NAME_BAND - 2
		card.add_child(row)
		var show_badges := func():
			_fill_badges(row, opt_id, str(presentation.get("badge_part", "")))
			row.visible = true
		var hide_badges := func(): row.visible = false
		card.focus_entered.connect(show_badges)
		card.mouse_entered.connect(show_badges)
		card.focus_exited.connect(hide_badges)
		card.mouse_exited.connect(hide_badges)
	return card

func _chip_style(color: Color) -> StyleBoxFlat:
	var style := StyleBoxFlat.new()
	style.bg_color = color
	style.set_corner_radius_all(4)
	style.set_content_margin_all(4)
	return style

# The numeric badges of one part against the installed one, at most two (the core decides what and how many).
func _fill_badges(row: HBoxContainer, option_id: String, part_id: String) -> void:
	for child in row.get_children():
		row.remove_child(child)
		child.free()
	if garage == null or not garage.has_method("get_part_badges"):
		return
	for badge in garage.get_part_badges(option_id, part_id):
		var label := _label(str(badge["text"]), 13)
		label.autowrap_mode = TextServer.AUTOWRAP_OFF
		label.mouse_filter = Control.MOUSE_FILTER_IGNORE
		label.add_theme_stylebox_override("normal", _chip_style(BADGE_WORSE if bool(badge["worse"]) else BADGE_BETTER))
		row.add_child(label)

func _part_name(opt: Dictionary) -> String:
	for part in opt.get("parts", []):
		if str(part["id"]) == str(opt["value"]):
			return str(part["label"])
	return str(opt["value"])

# A tyre id is compound_width_aspectrRim (e.g. road_265_35r20). The compound's label/image/rated speed come from the
# manifest generated from data/tyres/families.json; the size is read from the id itself. Never inferred from grip.
func _tyre_presentation(definition_id: String) -> Dictionary:
	if _component_catalog.is_empty():
		var parsed: Variant = JSON.parse_string(FileAccess.get_file_as_string("res://assets/components/tyres/catalog.json"))
		if parsed is Dictionary:
			_component_catalog = parsed
	var fallback := {"label":definition_id.replace("_", " ")}
	var parts := definition_id.split("_")
	if parts.size() != 3 or not parts[1].is_valid_int():
		return fallback
	var tail := parts[2].split("r")
	if tail.size() != 2 or not tail[0].is_valid_int() or not tail[1].is_valid_int():
		return fallback
	var family: Dictionary = _component_catalog.get("families", {}).get(parts[0], {})
	if family.is_empty():
		return fallback
	var rated := float(family.get("rated_speed_kmh", 0.0))
	var cap: Dictionary = _component_catalog.get("rated_speed_cap", {})
	if not cap.is_empty() and int(tail[1]) <= int(cap.get("max_rim_in", 0)):
		rated = minf(rated, float(cap.get("cap_kmh", rated)))
	return {
		"label": str(family.get("label", parts[0])),
		"family": parts[0],
		"width_mm": int(parts[1]),
		"size": "%d/%d R%d" % [int(parts[1]), int(tail[0]), int(tail[1])],
		"image": str(family.get("image", "")),
		"rated_speed_kmh": rated,
	}

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
		if path.ends_with(".svg"):
			_component_textures[path] = load(path) as Texture2D
			return _component_textures[path]
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

# An option with an absolute slider (rpm, bar, ms) is shown in its own unit by rg core; the rest as a factor.
func _option_display(opt: Dictionary, value: Variant) -> String:
	var entry: Dictionary = opt.get("entry", {})
	if bool(entry.get("positional", false)) and garage != null:
		return str(garage.option_entry_text(str(opt["id"]), 0, float(value)))
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
			var entry: Dictionary = opt.get("entry", {})
			var positional := bool(entry.get("positional", false))
			var line := HBoxContainer.new()
			line.add_theme_constant_override("separation", 8)
			holder.add_child(line)
			var slider := HSlider.new()
			if positional:
				# an absolute (rpm, bar ...) slider: position 0..1 along the mapping the option table names
				slider.min_value = 0.0
				slider.max_value = 1.0
				slider.step = 0.001
				slider.value = garage.option_entry_position(id, 0, float(opt["value"]))
			else:
				slider.min_value = float(opt["min"])
				slider.max_value = float(opt["max"])
				slider.step = float(opt["step"])
				slider.value = float(opt["value"])
			slider.custom_minimum_size = Vector2(0, 22)
			slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			slider.size_flags_vertical = Control.SIZE_SHRINK_CENTER
			slider.value_changed.connect(func(x: float):
				_edit_option(id, garage.option_entry_value(id, 0, x) if positional else x))
			line.add_child(slider)
			var box := _entry_box(id, 0, entry, float(opt["value"]))
			_enter_focuses_box(slider, box)
			line.add_child(box)
			var unit_label := _label(str(entry.get("unit", "")), 13, HORIZONTAL_ALIGNMENT_LEFT, true)
			unit_label.custom_minimum_size = Vector2(34, 0)
			line.add_child(unit_label)
			_option_entries[id] = [box]
			holder.set_meta("main_slider", slider)
			holder.set_meta("positional", positional)
			_option_controls[id] = slider
		"scale_list":
			var grid := VBoxContainer.new()
			grid.add_theme_constant_override("separation", 0)
			holder.add_child(grid)
			var values: PackedFloat64Array = opt["value"]
			var entries: Array = opt.get("entries", [])
			var sliders: Array = []
			var boxes: Array = []
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
				var box := _entry_box(id, i, entries[i] if i < entries.size() else {}, values[i])
				box.custom_minimum_size = Vector2(70, 0)
				_enter_focuses_box(slider, box)
				line.add_child(box)
				sliders.append(slider)
				boxes.append(box)
				_option_controls["%s:%d" % [id, i]] = slider
			var push := func():
				var list := PackedFloat64Array()
				for sl in sliders:
					list.append((sl as HSlider).value)
				_edit_option(id, list)
			for sl in sliders:
				(sl as HSlider).value_changed.connect(func(_x: float): push.call())
			_option_controls[id] = grid
			_option_entries[id] = boxes
			holder.set_meta("gear_sliders", sliders)
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

const ENTRY_BOX_WIDTH := 84.0

# Decimals that show a slider step exactly (1 -> 0, 0.05 -> 2, 0.001 -> 3).
func _step_decimals(step: float) -> int:
	var decimals := 0
	while decimals < 4 and absf(step * pow(10.0, decimals) - round(step * pow(10.0, decimals))) > 1e-6:
		decimals += 1
	return decimals

# A compact box next to a slider: the number in the option's absolute display unit (rpm, bar, ratio, % ...). Enter or
# leaving the box commits it through _edit_option; a refused value is taken back with the reason. Click-focus only, so
# the gamepad (and Tab) skip it; the keyboard reaches it with Enter on the slider.
func _entry_box(id: String, index: int, entry: Dictionary, stored: float) -> LineEdit:
	var box := LineEdit.new()
	box.name = "Entry_%s_%d" % [id, index]
	box.custom_minimum_size = Vector2(ENTRY_BOX_WIDTH, 0)
	box.alignment = HORIZONTAL_ALIGNMENT_RIGHT
	box.focus_mode = Control.FOCUS_CLICK
	box.select_all_on_focus = true
	box.context_menu_enabled = false
	box.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	box.add_theme_font_size_override("font_size", 14)
	var unit := str(entry.get("unit", ""))
	box.tooltip_text = "Type a value%s and press Enter. Allowed: %s to %s%s." % [
		" in " + unit if not unit.is_empty() else "",
		_entry_number(entry, float(entry.get("lo", 0.0)), true), _entry_number(entry, float(entry.get("hi", 0.0)), true),
		" " + unit if not unit.is_empty() else ""]
	box.set_meta("entry", {"id": id, "index": index, "unit": unit})
	_set_entry_text(box, entry, stored)
	box.text_submitted.connect(func(t: String): _commit_entry(box, t))
	box.focus_exited.connect(func(): _commit_entry(box, box.text))
	return box

# `value` is already in display units when `displayed`, else a stored value (multiplied by the entry's multiplier).
func _entry_number(entry: Dictionary, value: float, displayed: bool = false) -> String:
	var shown := value if displayed else value * float(entry.get("multiplier", 1.0))
	return ("%." + str(int(entry.get("decimals", 2))) + "f") % shown

func _set_entry_text(box: LineEdit, entry: Dictionary, stored: float) -> void:
	var text := _entry_number(entry, stored)
	box.text = text
	box.set_meta("shown", text)

# Keyboard users: Enter on a slider jumps to its value box (a pad's A button must not, it would trap the focus).
func _enter_focuses_box(slider: HSlider, box: LineEdit) -> void:
	slider.gui_input.connect(func(event: InputEvent):
		if event is InputEventKey and event.pressed and not event.echo and (event.keycode == KEY_ENTER or event.keycode == KEY_KP_ENTER):
			box.grab_focus()
			slider.accept_event())

# The number a player typed ("8000", "8 000", "52,5", "8000 rpm", "x1.05"); NAN when it is none.
func _parse_typed_number(text: String, unit: String) -> float:
	var t := text.strip_edges().replace(",", ".").replace(" ", "")
	if not unit.is_empty() and t.to_lower().ends_with(unit.to_lower()):
		t = t.substr(0, t.length() - unit.length())
	if t.begins_with("x") or t.begins_with("X"):
		t = t.substr(1)
	if t.is_empty() or not t.is_valid_float():
		return NAN
	return t.to_float()

func _commit_entry(box: LineEdit, text: String) -> void:
	if garage == null or not is_instance_valid(box) or not box.has_meta("entry"):
		return
	var typed := text.strip_edges()
	if typed == str(box.get_meta("shown", "")):
		return
	var meta: Dictionary = box.get_meta("entry")
	var id := str(meta["id"])
	var index := int(meta["index"])
	var number := _parse_typed_number(typed, str(meta["unit"]))
	if is_nan(number):
		_reject_entry(id, "\"%s\" is not a number." % typed)
		return
	var parsed: Dictionary = garage.option_entry_parse(id, index, number)
	if not bool(parsed["ok"]):
		_reject_entry(id, str(parsed["message"]))
		return
	var opt := _garage_option(id)
	if opt.is_empty():
		return
	var before: Variant = opt["value"]
	var value: Variant = float(parsed["value"])
	if str(opt["kind"]) == "scale_list":
		var list := PackedFloat64Array(before)
		list[index] = float(parsed["value"])
		value = list
	var was_valid := bool(garage.get_validation()["ok"])
	var result: Dictionary = _edit_option(id, value)
	if not bool(result.get("accepted", false)):
		_reject_entry(id, str(result.get("message", "not accepted")))
	elif was_valid and not bool(result["ok"]):
		# the physics loader (or a rule such as gears that must keep getting lower) refuses this combination
		var reason := str(result["validation_message"])
		_edit_option(id, before)
		_reject_entry(id, reason)

# Takes a refused typed value back (the boxes show the working value again) and says why.
func _reject_entry(id: String, reason: String) -> void:
	var opt := _garage_option(id)
	if not opt.is_empty():
		_sync_row(id, opt["value"])
	var note: Label = _option_notes.get(id, null)
	if note != null:
		note.text = reason
		note.visible = true

# The slider(s), value box(es) and summary of an option row follow `value` (no signals: nothing is edited again).
func _sync_row(id: String, value: Variant) -> void:
	var label: Label = _option_values.get(id, null)
	if label == null or not is_instance_valid(label):
		return
	var row := label.get_parent().get_parent()
	var opt: Dictionary = row.get_meta("option_definition", {})
	var boxes: Array = _option_entries.get(id, [])
	if str(row.get_meta("kind", "")) == "scale_list":
		var sliders: Array = row.get_meta("gear_sliders", [])
		var values := PackedFloat64Array(value)
		var entries: Array = opt.get("entries", [])
		for i in range(mini(sliders.size(), values.size())):
			(sliders[i] as HSlider).set_value_no_signal(values[i])
			if i < boxes.size():
				_set_entry_text(boxes[i], entries[i] if i < entries.size() else {}, values[i])
		return
	label.text = _option_display(opt, value)
	var slider: HSlider = row.get_meta("main_slider", null)
	if slider != null:
		slider.set_value_no_signal(garage.option_entry_position(id, 0, float(value)) if bool(row.get_meta("positional", false)) else float(value))
	if not boxes.is_empty():
		_set_entry_text(boxes[0], opt.get("entry", {}), float(value))

# The player changed an option: RgGarage decides (type, range, the loader's verdict).
func _edit_option(option_id: String, value: Variant) -> Dictionary:
	var result: Dictionary = garage.set_option(option_id, value)
	if option_id == "engine_install":
		_engine_notice = str(result.get("notice", ""))
	if option_id == "gearbox_install":
		_gearbox_notice = str(result.get("notice", ""))
	if option_id == "drive_layout":
		_driveline_notice = str(result.get("notice", ""))
	var note: Label = _option_notes.get(option_id, null)
	if note != null:
		note.visible = not bool(result["accepted"])
		note.text = str(result["message"])
	if result.has("value"):
		_sync_row(option_id, result["value"])
	garage_option_changed.emit(option_id)
	_refresh_status()
	return result

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
