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

const BOOT_SECONDS := 1.5
const TITLE := "racing_game"

var shell: Node # RgShell
var binding_labels_path: String = "" # data/controls/binding_labels.json
var screen: String = "" # the screen built now ("" before the first)

var _root: Control
var _boot_elapsed: float = 0.0
var _boot_done: bool = false
var _buttons: Dictionary = {} # id -> Button of the current screen
var _setting_controls: Dictionary = {} # key -> control of the settings screen
var _setting_displays: Dictionary = {} # key -> Label with the shown value
var _setting_notes: Dictionary = {} # key -> Label for a validation message

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

func _process(delta: float) -> void:
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
	_add_bindings_view(body)
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

# ---- bindings view (read-only; rebinding is deferred) --------------------------

func _event_text(event: InputEvent) -> String:
	if event is InputEventKey:
		return (event as InputEventKey).as_text_physical_keycode().replace(" (Physical)", "")
	if event is InputEventJoypadButton:
		var text := (event as InputEventJoypadButton).as_text()
		var open := text.find("(")
		if open >= 0:
			var inner := text.substr(open + 1)
			var cut := inner.find(",")
			if cut < 0:
				cut = inner.find(")")
			if cut > 0:
				return "Pad " + inner.substr(0, cut)
		return text
	return event.as_text()

func _add_bindings_view(body: VBoxContainer) -> void:
	body.add_child(_label("Controls", 24, HORIZONTAL_ALIGNMENT_LEFT))
	body.add_child(_label("Read-only: rebinding is not available yet.", 13, HORIZONTAL_ALIGNMENT_LEFT, true))
	var parsed = JSON.parse_string(FileAccess.get_file_as_string(binding_labels_path)) if binding_labels_path != "" else null
	if not parsed is Dictionary:
		body.add_child(_label("(the bindings list could not be read)", 14, HORIZONTAL_ALIGNMENT_LEFT, true))
		return
	for group in parsed.get("groups", []):
		body.add_child(_label(str(group.get("title", "")), 18, HORIZONTAL_ALIGNMENT_LEFT))
		for entry in group.get("actions", []):
			var action := str(entry.get("action", ""))
			var keys := PackedStringArray()
			if InputMap.has_action(action):
				for event in InputMap.action_get_events(action):
					keys.append(_event_text(event))
			_binding_row(body, str(entry.get("label", action)), ", ".join(keys) if not keys.is_empty() else "(unbound)")
		for entry in group.get("fixed", []):
			_binding_row(body, str(entry.get("label", "")), str(entry.get("keys", "")))
		for line in group.get("analog", []):
			body.add_child(_label(str(line), 14, HORIZONTAL_ALIGNMENT_LEFT, true))

func _binding_row(parent: VBoxContainer, label: String, keys: String) -> void:
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 12)
	var name_label := _label(label, 15, HORIZONTAL_ALIGNMENT_LEFT)
	name_label.custom_minimum_size = Vector2(520, 0)
	row.add_child(name_label)
	var keys_label := _label(keys, 15, HORIZONTAL_ALIGNMENT_LEFT)
	keys_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(keys_label)
	parent.add_child(row)
