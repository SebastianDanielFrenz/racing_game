extends Control
# game/scripts/controls_ui.gd - the controls screen (PLAN.md R5b): Settings -> Controls and the
# pause menu's Controls. Drawing and input forwarding ONLY: what an action is, which bindings a
# device has, what conflicts, how a captured input becomes a binding, how an axis is calibrated and
# what the game receives from a device are all decided by rg_core (rg::Controls / rg::BindingCapture /
# rg::AxisCalibrator, through the RgControls node main.gd creates). This script gathers the raw device
# state the capture needs (it sees every key / button / axis itself: input_map.gd is suspended while this
# screen is up), draws what RgControls hands back and forwards what the player did.
#
# Layout: devices on the left (connected first, then remembered but unplugged ones, "name #n"), the
# actions of the selected device grouped in the middle (one row per action or per side of a signed axis:
# [binding(s)] [+ add a second] [clear] [reset]), and on the right the selected action's bindings with
# their tuning (deadzone, saturation, response curve, sensitivity, invert, calibrate, combined pedals) over
# a live input monitor (the device's raw axes / keys / buttons, and what the game gets from them).
#
# Input: everything is reachable with the mouse, the keyboard (Tab / arrows / Enter) and a gamepad
# (D-pad / left stick move the focus, A presses, B goes back). While a binding is captured the keyboard,
# the pad and (for the mouse profile) the mouse belong to the capture: Esc cancels, on a pad a short press
# of B cancels and a hold binds it (rg::BindingCapture).
#
# Test hooks (tools/smoke_test.ps1 -Controls, controls_test.gd): get_hook(id), hook_ids(), select_device(),
# begin_capture(), is_capturing(), is_calibrating(), inject_pad() (a pad snapshot instead of the device),
# status_text(), refresh().

signal back_requested
signal changed # a binding / tuning changed (main.gd re-syncs the menu actions; the file is saved on leaving)

const ACCENT := Color(0.91, 0.45, 0.17)
const WARN := Color(1.0, 0.7, 0.3)
const DIM := Color(1, 1, 1, 0.7)
const POLLED_PAD_BUTTONS := 48
const POLLED_PAD_AXES := 10
const MOUSE_BUTTONS := [1, 2, 3, 8, 9]
const RAW_AXIS_ROWS := 8
const MAPPED_ROWS := 8
const CONFIRM_MS := 4000

var controls: Node           # RgControls
var fixed_keys_path: String = ""

var _device_key: String = "keyboard"
var _selected_action: String = ""
var _selected_sign: int = 0
var _capture: Dictionary = {}     # {} = none; {action, sign, replace, label}
var _calib: Dictionary = {}       # {} = none; {action, index, axis, label}
var _revision: int = -1
var _devices_sig: String = ""
var _held_keys: Dictionary = {}
var _held_mouse: Dictionary = {}
var _motion: Vector2 = Vector2.ZERO
var _wheel: Dictionary = {"up": 0.0, "down": 0.0, "left": 0.0, "right": 0.0}
var _pad_override: Dictionary = {}
var _hooks: Dictionary = {}       # id -> Control (test hooks)
var _rows_cache: Dictionary = {}  # "action:sign" -> row dict of the selected device
var _row_resets: Dictionary = {}  # "action:sign" -> Button
var _row_labels: Dictionary = {}  # "action:sign" -> Label
var _last_focus_hook: String = ""
var _reset_armed_until: int = 0
var _detail_reset: Button
var _snapshot_last: Dictionary = {}

var _device_box: VBoxContainer
var _rows_box: VBoxContainer
var _detail_box: VBoxContainer
var _status: Label
var _device_title: Label
var _reset_device_button: Button
var _overlay: Control
var _overlay_title: Label
var _overlay_hint: Label
var _overlay_extra: Label
var _overlay_progress: ProgressBar
var _overlay_buttons: HBoxContainer
var _overlay_apply: Button
var _overlay_cancel: Button
var _mon_header: Label
var _mon_raw_box: VBoxContainer
var _mon_raw: Array = []          # [{box, bar, value}]
var _mon_digital: Label
var _mon_mapped: Array = []       # [{box, name, bar, value}]

func _ready() -> void:
	set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_build()
	Input.joy_connection_changed.connect(func(_d: int, _c: bool): call_deferred("refresh"))
	refresh()

# ---- test hooks / queries --------------------------------------------------------------

func get_hook(id: String) -> Control:
	return _hooks.get(id, null)

func hook_ids() -> PackedStringArray:
	return PackedStringArray(_hooks.keys())

func selected_device() -> String:
	return _device_key

func is_capturing() -> bool:
	return not _capture.is_empty()

func is_calibrating() -> bool:
	return not _calib.is_empty()

func status_text() -> String:
	return _status.text if _status != null else ""

# A pad snapshot used instead of the real device for the selected pad's slot ({} = real device).
func inject_pad(slot: int, buttons: PackedInt32Array, axes: PackedFloat32Array) -> void:
	_pad_override = {"slot": slot, "buttons": buttons, "axes": axes}

func clear_pad_injection() -> void:
	_pad_override = {}

# Esc: a running capture / calibration takes it first (true when it did).
func consume_back() -> bool:
	if not _capture.is_empty():
		_cancel_capture("Cancelled")
		return true
	if not _calib.is_empty():
		_cancel_calibration()
		return true
	return false

func select_device(key: String) -> void:
	if key == _device_key and not _rows_cache.is_empty():
		return
	_cancel_capture("")
	_cancel_calibration()
	_device_key = key
	_selected_action = ""
	_selected_sign = 0
	_last_focus_hook = ""
	_rebuild_devices()
	_rebuild_rows()
	_rebuild_detail()

func begin_capture(action: String, sign: int, replace: bool) -> void:
	_begin_capture(action, sign, replace)

# Full rebuild from RgControls (devices, rows, detail); keeps the selection where it still exists.
func refresh() -> void:
	if controls == null or not controls.is_ready():
		return
	var known := false
	for d in controls.get_devices():
		if str(d["key"]) == _device_key:
			known = true
	if not known:
		_device_key = "keyboard"
		_selected_action = ""
	_revision = controls.get_revision()
	_devices_sig = _devices_signature()
	_rebuild_devices()
	_rebuild_rows()
	_rebuild_detail()

# ---- building -------------------------------------------------------------------------------

func _label(text: String, font_size: int, align: int = HORIZONTAL_ALIGNMENT_LEFT, dim: bool = false) -> Label:
	var label := Label.new()
	label.text = text
	label.add_theme_font_size_override("font_size", font_size)
	label.horizontal_alignment = align
	label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	if dim:
		label.modulate = DIM
	return label

func _button(text: String, hook_id: String, callback: Callable, font_size: int = 15) -> Button:
	var button := Button.new()
	button.text = text
	button.add_theme_font_size_override("font_size", font_size)
	button.pressed.connect(callback)
	if hook_id != "":
		_hooks[hook_id] = button
		button.focus_entered.connect(func(): _last_focus_hook = hook_id)
	return button

func _panel(min_width: float) -> VBoxContainer:
	var panel := PanelContainer.new()
	panel.custom_minimum_size = Vector2(min_width, 0)
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.03, 0.04, 0.055, 0.85)
	style.set_corner_radius_all(6)
	style.set_content_margin_all(10)
	panel.add_theme_stylebox_override("panel", style)
	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation", 6)
	panel.add_child(column)
	return column

func _scroll(parent: Control) -> VBoxContainer:
	var scroll := ScrollContainer.new()
	scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	scroll.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	parent.add_child(scroll)
	var box := VBoxContainer.new()
	box.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	box.add_theme_constant_override("separation", 4)
	scroll.add_child(box)
	return box

func _build() -> void:
	var backdrop := ColorRect.new()
	backdrop.color = Color(0.05, 0.07, 0.09, 1.0)
	backdrop.set_anchors_preset(Control.PRESET_FULL_RECT)
	add_child(backdrop)
	var margin := MarginContainer.new()
	margin.set_anchors_preset(Control.PRESET_FULL_RECT)
	for side in ["left", "right", "top", "bottom"]:
		margin.add_theme_constant_override("margin_" + side, 22)
	add_child(margin)
	var frame := VBoxContainer.new()
	frame.add_theme_constant_override("separation", 8)
	margin.add_child(frame)
	frame.add_child(_label("Controls", 30, HORIZONTAL_ALIGNMENT_CENTER))
	var body := HBoxContainer.new()
	body.size_flags_vertical = Control.SIZE_EXPAND_FILL
	body.add_theme_constant_override("separation", 10)
	frame.add_child(body)
	# devices
	var left := _panel(230)
	left.get_parent().size_flags_vertical = Control.SIZE_EXPAND_FILL
	body.add_child(left.get_parent())
	left.add_child(_label("Devices", 20))
	_device_box = _scroll(left)
	_reset_device_button = _button("Reset device to defaults", "reset_device", _on_reset_device)
	left.add_child(_reset_device_button)
	# actions
	var middle := _panel(0)
	middle.get_parent().size_flags_horizontal = Control.SIZE_EXPAND_FILL
	middle.get_parent().size_flags_vertical = Control.SIZE_EXPAND_FILL
	body.add_child(middle.get_parent())
	_device_title = _label("", 20)
	middle.add_child(_device_title)
	_rows_box = _scroll(middle)
	# detail + monitor
	var right := VBoxContainer.new()
	right.custom_minimum_size = Vector2(340, 0)
	right.add_theme_constant_override("separation", 8)
	body.add_child(right)
	var detail := _panel(0)
	detail.get_parent().size_flags_vertical = Control.SIZE_EXPAND_FILL
	right.add_child(detail.get_parent())
	_detail_box = _scroll(detail)
	var monitor := _panel(0)
	monitor.get_parent().size_flags_vertical = Control.SIZE_EXPAND_FILL
	right.add_child(monitor.get_parent())
	_build_monitor(monitor)
	# footer
	var footer := HBoxContainer.new()
	footer.add_theme_constant_override("separation", 14)
	frame.add_child(footer)
	var back := _button("Back", "back", func(): back_requested.emit(), 20)
	back.custom_minimum_size = Vector2(160, 0)
	footer.add_child(back)
	_status = _label("", 14)
	_status.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_status.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	footer.add_child(_status)
	_build_overlay()

func _build_monitor(column: VBoxContainer) -> void:
	_mon_header = _label("Live input", 18)
	column.add_child(_mon_header)
	column.add_child(_label("Raw device state on top, what the game receives below.", 12, HORIZONTAL_ALIGNMENT_LEFT, true))
	# a fixed-height scroll area: the tuning panel above keeps the room it needs on a small window
	var scroll := ScrollContainer.new()
	scroll.custom_minimum_size = Vector2(0, 150)
	scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	column.add_child(scroll)
	var inner := VBoxContainer.new()
	inner.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	inner.add_theme_constant_override("separation", 2)
	scroll.add_child(inner)
	_mon_raw_box = VBoxContainer.new()
	_mon_raw_box.add_theme_constant_override("separation", 1)
	inner.add_child(_mon_raw_box)
	for i in range(RAW_AXIS_ROWS):
		var box := HBoxContainer.new()
		box.add_theme_constant_override("separation", 6)
		var name_label := _label("Axis %d" % i, 12)
		name_label.custom_minimum_size = Vector2(52, 0)
		box.add_child(name_label)
		var bar := ProgressBar.new()
		bar.min_value = -1.0
		bar.max_value = 1.0
		bar.show_percentage = false
		bar.custom_minimum_size = Vector2(150, 12)
		bar.size_flags_vertical = Control.SIZE_SHRINK_CENTER
		bar.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		box.add_child(bar)
		var value := _label("0.00", 12, HORIZONTAL_ALIGNMENT_RIGHT)
		value.custom_minimum_size = Vector2(46, 0)
		box.add_child(value)
		_mon_raw_box.add_child(box)
		_mon_raw.append({"box": box, "bar": bar, "value": value})
	_mon_digital = _label("", 12)
	inner.add_child(_mon_digital)
	inner.add_child(_label("Game receives", 14))
	for i in range(MAPPED_ROWS):
		var box := HBoxContainer.new()
		box.add_theme_constant_override("separation", 6)
		var name_label := _label("", 12)
		name_label.custom_minimum_size = Vector2(120, 0)
		name_label.clip_text = true
		name_label.autowrap_mode = TextServer.AUTOWRAP_OFF
		box.add_child(name_label)
		var bar := ProgressBar.new()
		bar.min_value = -1.0
		bar.max_value = 1.0
		bar.show_percentage = false
		bar.custom_minimum_size = Vector2(90, 12)
		bar.size_flags_vertical = Control.SIZE_SHRINK_CENTER
		bar.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		box.add_child(bar)
		var value := _label("", 12, HORIZONTAL_ALIGNMENT_RIGHT)
		value.custom_minimum_size = Vector2(96, 0)
		box.add_child(value)
		box.visible = false
		inner.add_child(box)
		_mon_mapped.append({"box": box, "name": name_label, "bar": bar, "value": value})

func _build_overlay() -> void:
	_overlay = Control.new()
	_overlay.set_anchors_preset(Control.PRESET_FULL_RECT)
	_overlay.mouse_filter = Control.MOUSE_FILTER_STOP
	_overlay.visible = false
	add_child(_overlay)
	var dim := ColorRect.new()
	dim.color = Color(0, 0, 0, 0.72)
	dim.set_anchors_preset(Control.PRESET_FULL_RECT)
	dim.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_overlay.add_child(dim)
	var center := CenterContainer.new()
	center.set_anchors_preset(Control.PRESET_FULL_RECT)
	center.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_overlay.add_child(center)
	var column := _panel(520)
	center.add_child(column.get_parent())
	_overlay_title = _label("", 26, HORIZONTAL_ALIGNMENT_CENTER)
	column.add_child(_overlay_title)
	_overlay_hint = _label("", 20, HORIZONTAL_ALIGNMENT_CENTER)
	_overlay_hint.add_theme_color_override("font_color", ACCENT)
	column.add_child(_overlay_hint)
	_overlay_extra = _label("", 14, HORIZONTAL_ALIGNMENT_CENTER, true)
	column.add_child(_overlay_extra)
	_overlay_progress = ProgressBar.new()
	_overlay_progress.min_value = 0.0
	_overlay_progress.max_value = 1.0
	_overlay_progress.show_percentage = false
	_overlay_progress.custom_minimum_size = Vector2(0, 10)
	column.add_child(_overlay_progress)
	_overlay_buttons = HBoxContainer.new()
	_overlay_buttons.alignment = BoxContainer.ALIGNMENT_CENTER
	_overlay_buttons.add_theme_constant_override("separation", 14)
	column.add_child(_overlay_buttons)
	_overlay_apply = _button("Apply", "calib_apply", _on_calibration_apply, 18)
	_overlay_buttons.add_child(_overlay_apply)
	_overlay_cancel = _button("Cancel", "overlay_cancel", func(): consume_back(), 18)
	_overlay_buttons.add_child(_overlay_cancel)

# ---- devices ---------------------------------------------------------------------------------

func _devices_signature() -> String:
	var parts := PackedStringArray()
	for d in controls.get_devices():
		parts.append("%s|%s|%s|%s" % [d["key"], d["connected"], d["customised"], d["inherits"]])
	return ";".join(parts)

func _device() -> Dictionary:
	for d in controls.get_devices():
		if str(d["key"]) == _device_key:
			return d
	return {}

func _is_pad(d: Dictionary) -> bool:
	var cls := str(d.get("class", ""))
	return cls == "gamepad" or cls == "wheel" or cls == "generic"

func _rebuild_devices() -> void:
	for child in _device_box.get_children():
		_device_box.remove_child(child)
		child.queue_free()
	for id in _hooks.keys():
		if str(id).begins_with("device:"):
			_hooks.erase(id)
	var devices: Array = controls.get_devices()
	var ordered: Array = []
	for d in devices:
		if bool(d["connected"]):
			ordered.append(d)
	for d in devices:
		if not bool(d["connected"]):
			ordered.append(d)
	var group := ButtonGroup.new()
	for d in ordered:
		var key := str(d["key"])
		var status := "connected" if bool(d["connected"]) else "not connected"
		if bool(d["customised"]):
			status += " - customised"
		elif bool(d["inherits"]):
			status += " - uses setup #1"
		var button := Button.new()
		button.toggle_mode = true
		button.button_group = group
		button.alignment = HORIZONTAL_ALIGNMENT_LEFT
		button.text = "%s\n%s" % [str(d["label"]), status]
		button.add_theme_font_size_override("font_size", 14)
		button.set_pressed_no_signal(key == _device_key)
		if not bool(d["connected"]):
			button.modulate = Color(1, 1, 1, 0.65)
		button.pressed.connect(func(): select_device(key))
		button.focus_entered.connect(func(): _last_focus_hook = "device:" + key)
		_device_box.add_child(button)
		_hooks["device:" + key] = button
	var dev := _device()
	_reset_device_button.text = "Reset device to defaults"
	_reset_device_button.disabled = dev.is_empty() or not bool(dev.get("customised", false))
	_reset_armed_until = 0

# ---- action rows ------------------------------------------------------------------------------

func _row_key(action: String, sign: int) -> String:
	return "%s:%d" % [action, sign]

func _rebuild_rows() -> void:
	for child in _rows_box.get_children():
		_rows_box.remove_child(child)
		child.queue_free()
	for id in _hooks.keys():
		var s := str(id)
		if s.begins_with("bind:") or s.begins_with("add:") or s.begins_with("clear:") or s.begins_with("reset:"):
			_hooks.erase(id)
	_rows_cache.clear()
	_row_resets.clear()
	_row_labels.clear()
	var dev := _device()
	if dev.is_empty():
		return
	var connected := bool(dev["connected"])
	_device_title.text = "%s%s" % [str(dev["label"]), "" if connected else "  (not connected)"]
	if str(dev["class"]) == "generic":
		_rows_box.add_child(_label("This controller has no standard mapping (SDL does not know it): it does nothing until you bind its buttons and axes here.", 14, HORIZONTAL_ALIGNMENT_LEFT, true))
	if not connected:
		_rows_box.add_child(_label("Remembered device: its setup is kept and applies again when it is plugged in. Connect it to change a binding.", 14, HORIZONTAL_ALIGNMENT_LEFT, true))
	var others: Dictionary = {} # action -> Array of labels
	for c in controls.get_conflicts(_device_key):
		var a := str(c["action_a"])
		var b := str(c["action_b"])
		if not others.has(a):
			others[a] = []
		if not others.has(b):
			others[b] = []
		(others[a] as Array).append(str(c["label_b"]))
		(others[b] as Array).append(str(c["label_a"]))
	var group_title := ""
	for row in controls.get_rows(_device_key):
		var action := str(row["action"])
		var sign := int(row["sign"])
		var key := _row_key(action, sign)
		_rows_cache[key] = row
		if str(row["group_title"]) != group_title:
			group_title = str(row["group_title"])
			var heading := _label(group_title, 18)
			heading.add_theme_color_override("font_color", ACCENT)
			_rows_box.add_child(heading)
		_rows_box.add_child(_action_row(row, key, connected, others.get(action, [])))
	_add_fixed_keys()
	_restore_focus()

func _action_row(row: Dictionary, key: String, connected: bool, others: Array) -> Control:
	var action := str(row["action"])
	var sign := int(row["sign"])
	var holder := VBoxContainer.new()
	holder.add_theme_constant_override("separation", 0)
	var line := HBoxContainer.new()
	line.add_theme_constant_override("separation", 6)
	holder.add_child(line)
	var name_label := _label(str(row["label"]), 15)
	name_label.custom_minimum_size = Vector2(170, 0)
	name_label.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	line.add_child(name_label)
	_row_labels[key] = name_label
	var texts := PackedStringArray()
	for b in row["bindings"]:
		texts.append(_binding_text(b, sign))
	var main := _button(" / ".join(texts) if not texts.is_empty() else "(unbound)", "bind:" + key, func(): _begin_capture(action, sign, true))
	main.alignment = HORIZONTAL_ALIGNMENT_LEFT
	main.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	main.clip_text = true
	main.disabled = not connected
	main.tooltip_text = "Click, then press the input to bind (replaces this row)"
	main.focus_entered.connect(func(): _select_row(action, sign))
	main.pressed.connect(func(): _select_row(action, sign))
	line.add_child(main)
	var add := _button("+", "add:" + key, func(): _begin_capture(action, sign, false))
	add.tooltip_text = "Add a second binding"
	add.disabled = not connected or (row["bindings"] as Array).size() >= 8
	add.focus_entered.connect(func(): _select_row(action, sign))
	line.add_child(add)
	var clear := _button("Clear", "clear:" + key, func(): _on_clear_row(action, sign))
	clear.disabled = (row["bindings"] as Array).is_empty()
	clear.focus_entered.connect(func(): _select_row(action, sign))
	line.add_child(clear)
	var reset := _button("Reset", "reset:" + key, func(): _on_reset_action(action))
	reset.visible = bool(row["overridden"])
	reset.focus_entered.connect(func(): _select_row(action, sign))
	line.add_child(reset)
	_row_resets[key] = reset
	if not others.is_empty():
		var note := _label("Also used by: " + ", ".join(PackedStringArray(others)), 12)
		note.add_theme_color_override("font_color", WARN)
		holder.add_child(note)
	if action == _selected_action and sign == _selected_sign:
		name_label.add_theme_color_override("font_color", ACCENT)
	return holder

# A key bound to one side of a signed axis is shown without the "(-)" / "(+)" tag: the row already says which side.
func _binding_text(b: Dictionary, row_sign: int) -> String:
	var text := str(b["text"])
	if row_sign != 0 and str(b["type"]) == "key":
		for tag in [" (-)", " (+)"]:
			if text.ends_with(tag):
				return text.substr(0, text.length() - tag.length())
	return text

func _add_fixed_keys() -> void:
	var parsed = JSON.parse_string(FileAccess.get_file_as_string(fixed_keys_path)) if fixed_keys_path != "" else null
	if not parsed is Dictionary:
		return
	var heading := _label("Fixed keys (not rebindable)", 18)
	heading.add_theme_color_override("font_color", ACCENT)
	_rows_box.add_child(heading)
	for entry in parsed.get("entries", []):
		var line := HBoxContainer.new()
		line.add_theme_constant_override("separation", 12)
		var keys := _label(str(entry.get("keys", "")), 14)
		keys.custom_minimum_size = Vector2(170, 0)
		line.add_child(keys)
		var text := _label(str(entry.get("label", "")), 14, HORIZONTAL_ALIGNMENT_LEFT, true)
		text.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		line.add_child(text)
		_rows_box.add_child(line)

func _select_row(action: String, sign: int) -> void:
	if action == _selected_action and sign == _selected_sign:
		return
	var old := _row_key(_selected_action, _selected_sign)
	if _row_labels.has(old):
		(_row_labels[old] as Label).remove_theme_color_override("font_color")
	_selected_action = action
	_selected_sign = sign
	var key := _row_key(action, sign)
	if _row_labels.has(key):
		(_row_labels[key] as Label).add_theme_color_override("font_color", ACCENT)
	_rebuild_detail()

func _restore_focus() -> void:
	if _last_focus_hook != "":
		call_deferred("_grab_hook", _last_focus_hook)

func _grab_hook(id: String) -> void:
	var control: Control = _hooks.get(id, null)
	if control != null and is_instance_valid(control) and control.is_inside_tree() and control.is_visible_in_tree() \
			and not (control is Button and (control as Button).disabled):
		control.grab_focus()

func focus_first_device() -> void:
	call_deferred("_grab_hook", "device:" + _device_key)

# ---- detail panel ---------------------------------------------------------------------------------

func _selected_row() -> Dictionary:
	return _rows_cache.get(_row_key(_selected_action, _selected_sign), {})

func _rebuild_detail() -> void:
	for child in _detail_box.get_children():
		_detail_box.remove_child(child)
		child.queue_free()
	for id in _hooks.keys():
		var s := str(id)
		if s.begins_with("remove:") or s.begins_with("tune:") or s.begins_with("invert:") or s.begins_with("calibrate:") \
				or s.begins_with("combine:") or s == "reset_action":
			_hooks.erase(id)
	_detail_reset = null
	var row := _selected_row()
	if row.is_empty():
		_detail_box.add_child(_label("Select an action to see its bindings and tune its axes.", 14, HORIZONTAL_ALIGNMENT_LEFT, true))
		return
	var action := str(row["action"])
	var connected := bool(_device().get("connected", false))
	_detail_box.add_child(_label(str(row["label"]), 20))
	if str(row["help"]) != "":
		_detail_box.add_child(_label(str(row["help"]), 13, HORIZONTAL_ALIGNMENT_LEFT, true))
	var bindings: Array = row["bindings"]
	if bindings.is_empty():
		_detail_box.add_child(_label("Not bound on this device.", 14, HORIZONTAL_ALIGNMENT_LEFT, true))
	for b in bindings:
		_detail_box.add_child(_binding_detail(action, b, connected))
	_detail_reset = _button("Reset this action to its default", "reset_action", func(): _on_reset_action(action))
	_detail_reset.disabled = not bool(row["overridden"])
	_detail_box.add_child(_detail_reset)

func _binding_detail(action: String, b: Dictionary, connected: bool) -> Control:
	var index := int(b["list_index"])
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 2)
	var head := HBoxContainer.new()
	head.add_theme_constant_override("separation", 6)
	box.add_child(head)
	var title := _label(_binding_text(b, _selected_sign), 16)
	title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	head.add_child(title)
	head.add_child(_button("Remove", "remove:%d" % index, func(): _on_remove_binding(action, index)))
	if not bool(b["analogue"]):
		return box
	_tuning_slider(box, "deadzone", "Deadzone", action, index, float(b["deadzone"]), 0.0, 0.8, 0.01)
	_tuning_slider(box, "saturation", "Saturation", action, index, float(b["saturation"]), 0.2, 1.0, 0.01)
	_tuning_slider(box, "curve", "Response curve", action, index, float(b["curve"]), 0.5, 3.0, 0.05)
	_tuning_slider(box, "sensitivity", "Sensitivity", action, index, float(b["sensitivity"]), 0.25, 4.0, 0.05)
	var invert := CheckBox.new()
	invert.text = "Invert"
	invert.button_pressed = bool(b["invert"])
	invert.add_theme_font_size_override("font_size", 14)
	invert.toggled.connect(func(on: bool): _tune(action, index, "invert", on))
	box.add_child(invert)
	_hooks["invert:%d" % index] = invert
	if str(b["type"]) == "joy_axis":
		var cal_text := "Calibrated: %.2f to %.2f" % [float(b["cal_min"]), float(b["cal_max"])] if bool(b["calibrated"]) else "Not calibrated (factory range)"
		box.add_child(_label(cal_text, 12, HORIZONTAL_ALIGNMENT_LEFT, true))
		var buttons := HBoxContainer.new()
		buttons.add_theme_constant_override("separation", 6)
		box.add_child(buttons)
		var calibrate := _button("Calibrate", "calibrate:%d" % index, func(): _begin_calibration(action, index))
		calibrate.disabled = not connected
		buttons.add_child(calibrate)
		if (action == "throttle" or action == "brake") and not bool(b["calibrated"]) and str(b["span"]) != "full":
			var combine := _button("One axis for both pedals", "combine:%d" % index, func(): _on_combine(action, index))
			combine.tooltip_text = "Use this axis as throttle (one half) and brake (the other half), as on a combined pedal axis"
			buttons.add_child(combine)
	box.add_child(HSeparator.new())
	return box

func _tuning_slider(parent: Control, key: String, text: String, action: String, index: int, value: float, lo: float, hi: float, step: float) -> void:
	var line := HBoxContainer.new()
	line.add_theme_constant_override("separation", 6)
	var name_label := _label(text, 13)
	name_label.custom_minimum_size = Vector2(104, 0)
	line.add_child(name_label)
	var slider := HSlider.new()
	slider.min_value = lo
	slider.max_value = hi
	slider.step = step
	slider.value = clampf(value, lo, hi)
	slider.custom_minimum_size = Vector2(110, 20)
	slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	slider.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	line.add_child(slider)
	var value_label := _label("%.2f" % slider.value, 13, HORIZONTAL_ALIGNMENT_RIGHT)
	value_label.custom_minimum_size = Vector2(40, 0)
	line.add_child(value_label)
	slider.value_changed.connect(func(v: float):
		value_label.text = "%.2f" % v
		_tune(action, index, key, v))
	parent.add_child(line)
	_hooks["tune:%s:%d" % [key, index]] = slider

func _tune(action: String, index: int, key: String, value: Variant) -> void:
	var r: Dictionary = controls.set_binding_tuning(_device_key, action, index, {key: value})
	if not bool(r.get("ok", false)):
		_set_status(str(r.get("error", "could not change that")), true)
		return
	_revision = controls.get_revision()
	changed.emit()
	_update_override_marks()

# After an edit that keeps the row structure (a slider): only the Reset buttons follow.
func _update_override_marks() -> void:
	for row in controls.get_rows(_device_key):
		var key := _row_key(str(row["action"]), int(row["sign"]))
		_rows_cache[key] = row
		var reset: Button = _row_resets.get(key, null)
		if reset != null:
			reset.visible = bool(row["overridden"])
	if _detail_reset != null:
		_detail_reset.disabled = not bool(_selected_row().get("overridden", false))
	_devices_sig = _devices_signature()
	_update_device_status_text()

func _update_device_status_text() -> void:
	var dev := _device()
	var button: Button = _hooks.get("device:" + _device_key, null)
	if button != null and not dev.is_empty():
		var status := "connected" if bool(dev["connected"]) else "not connected"
		if bool(dev["customised"]):
			status += " - customised"
		elif bool(dev["inherits"]):
			status += " - uses setup #1"
		button.text = "%s\n%s" % [str(dev["label"]), status]
	_reset_device_button.disabled = dev.is_empty() or not bool(dev.get("customised", false))

# ---- edits -----------------------------------------------------------------------------------------

func _after_edit(message: String, warn: bool = false) -> void:
	_revision = controls.get_revision()
	_devices_sig = _devices_signature()
	changed.emit()
	_rebuild_devices()
	_rebuild_rows()
	_rebuild_detail()
	_set_status(message, warn)

func _on_clear_row(action: String, sign: int) -> void:
	if controls.clear_row(_device_key, action, sign):
		_after_edit("Cleared %s" % str(_rows_cache.get(_row_key(action, sign), {}).get("label", action)))

func _on_reset_action(action: String) -> void:
	if controls.reset_action(_device_key, action):
		_after_edit("Reset to the default")

func _on_remove_binding(action: String, index: int) -> void:
	if controls.remove_binding(_device_key, action, index):
		_after_edit("Binding removed")

func _on_combine(action: String, index: int) -> void:
	var r: Dictionary = controls.make_combined_pedals(_device_key, action, index)
	if bool(r.get("ok", false)):
		_after_edit("Throttle and brake now share one axis")
	else:
		_set_status(str(r.get("error", "could not combine")), true)

func _on_reset_device() -> void:
	var now := Time.get_ticks_msec()
	if now > _reset_armed_until:
		_reset_armed_until = now + CONFIRM_MS
		_reset_device_button.text = "Press again to confirm"
		return
	_reset_armed_until = 0
	if controls.reset_device(_device_key):
		_after_edit("%s is back to its defaults" % str(_device().get("label", "Device")))

func _set_status(text: String, warn: bool = false) -> void:
	_status.text = text
	if warn:
		_status.add_theme_color_override("font_color", WARN)
	else:
		_status.remove_theme_color_override("font_color")

# ---- capture ----------------------------------------------------------------------------------------------

func _begin_capture(action: String, sign: int, replace: bool) -> void:
	var dev := _device()
	if dev.is_empty() or not bool(dev["connected"]):
		_set_status("Connect the device to change its bindings.", true)
		return
	_cancel_calibration()
	var snap := _snapshot_now()
	var r: Dictionary = controls.capture_begin(_device_key, action, sign, snap)
	if not bool(r.get("ok", false)):
		_set_status(str(r.get("error", "cannot capture")), true)
		return
	var label := str(_rows_cache.get(_row_key(action, sign), {}).get("label", action))
	_capture = {"action": action, "sign": sign, "replace": replace, "label": label}
	_overlay_title.text = ("Bind: " if replace else "Add a binding: ") + label
	var hint := str(controls.get_capture_hint())
	_overlay_hint.text = hint if hint != "" else "Move the mouse"
	var cls := str(dev["class"])
	_overlay_extra.text = "Esc cancels" if not _is_pad(dev) else "Tap B to cancel - hold B to bind B itself (Esc cancels too)"
	_overlay_progress.visible = _is_pad(dev)
	_overlay_progress.value = 0.0
	_overlay_buttons.visible = cls != "mouse"
	_overlay_apply.visible = false
	_overlay_cancel.visible = true
	_overlay.visible = true
	_set_status("")

func _cancel_capture(message: String) -> void:
	if _capture.is_empty():
		return
	_capture = {}
	_overlay.visible = false
	_set_status(message)
	_restore_focus()

func _finish_capture() -> void:
	var action := str(_capture["action"])
	var sign := int(_capture["sign"])
	var replace := bool(_capture["replace"])
	var label := str(_capture["label"])
	var bound := str(controls.get_capture_result().get("text", ""))
	_capture = {}
	_overlay.visible = false
	var r: Dictionary = controls.bind_captured(_device_key, action, sign, replace)
	if not bool(r.get("ok", false)):
		_set_status(str(r.get("error", "could not bind that")), true)
		_restore_focus()
		return
	var conflicts: Array = r.get("conflicts", [])
	if conflicts.is_empty():
		_after_edit("%s: %s" % [label, bound])
	else:
		_after_edit("%s: %s - also used by %s (fine when they act in different modes)" % [label, bound, ", ".join(PackedStringArray(conflicts))], true)

# ---- calibration ----------------------------------------------------------------------------------------------

func _begin_calibration(action: String, index: int) -> void:
	var dev := _device()
	if dev.is_empty() or not bool(dev["connected"]):
		return
	var b: Dictionary = controls.get_binding(_device_key, action, index)
	if b.is_empty() or str(b["type"]) != "joy_axis":
		return
	_cancel_capture("")
	var axis := int(b["index"])
	var snap := _snapshot_now()
	var rest := 0.0
	var pads: Array = snap.get("pads", [])
	if not pads.is_empty():
		var axes: PackedFloat32Array = pads[0]["axes"]
		if axis < axes.size():
			rest = axes[axis]
	controls.calibration_begin(rest)
	_calib = {"action": action, "index": index, "axis": axis, "label": str(_selected_row().get("label", action))}
	_overlay_title.text = "Calibrate: " + str(_calib["label"])
	_overlay_hint.text = "Leave it at rest, then move it through its whole travel"
	_overlay_extra.text = "Pedals: press fully and release. Wheels and sticks: turn / push to both ends. Then press Apply."
	_overlay_progress.visible = false
	_overlay_buttons.visible = true
	_overlay_apply.visible = true
	_overlay_apply.disabled = true
	_overlay_cancel.visible = true
	_overlay.visible = true
	call_deferred("_grab_hook", "overlay_cancel")

func _cancel_calibration() -> void:
	if _calib.is_empty():
		return
	_calib = {}
	_overlay.visible = false
	_set_status("Calibration cancelled")
	_restore_focus()

func _on_calibration_apply() -> void:
	if _calib.is_empty():
		return
	var action := str(_calib["action"])
	var index := int(_calib["index"])
	_calib = {}
	_overlay.visible = false
	if controls.calibration_apply(_device_key, action, index):
		_after_edit("Calibration saved")
	else:
		_set_status("The axis did not move far enough to calibrate", true)
		_restore_focus()

# ---- raw input -----------------------------------------------------------------------------------------------------

func _key_name(event: InputEventKey) -> String:
	var code: int = event.physical_keycode if event.physical_keycode != 0 else event.keycode
	return OS.get_keycode_string(code as Key)

func _input(event: InputEvent) -> void:
	if not is_visible_in_tree():
		return
	var capturing := not _capture.is_empty()
	if event is InputEventKey:
		var k := event as InputEventKey
		var key_name := _key_name(k)
		if k.pressed and not k.echo:
			_held_keys[key_name] = true
			if k.keycode == KEY_ESCAPE and (capturing or not _calib.is_empty()):
				consume_back()
				get_viewport().set_input_as_handled()
				return
		elif not k.pressed:
			_held_keys.erase(key_name)
		if capturing:
			get_viewport().set_input_as_handled()
	elif event is InputEventMouseButton:
		var m := event as InputEventMouseButton
		var factor: float = m.factor if m.factor > 0.0 else 1.0
		if m.pressed:
			match m.button_index:
				MOUSE_BUTTON_WHEEL_UP: _wheel["up"] += factor
				MOUSE_BUTTON_WHEEL_DOWN: _wheel["down"] += factor
				MOUSE_BUTTON_WHEEL_LEFT: _wheel["left"] += factor
				MOUSE_BUTTON_WHEEL_RIGHT: _wheel["right"] += factor
				_: _held_mouse[int(m.button_index)] = true
		else:
			_held_mouse.erase(int(m.button_index))
		if capturing and str(_device().get("class", "")) == "mouse":
			get_viewport().set_input_as_handled()
	elif event is InputEventMouseMotion:
		_motion += (event as InputEventMouseMotion).relative
	elif event is InputEventJoypadButton:
		var j := event as InputEventJoypadButton
		if capturing:
			get_viewport().set_input_as_handled() # the capture polls the pad itself
		elif j.pressed and j.button_index == JOY_BUTTON_B:
			if not consume_back():
				back_requested.emit()
			get_viewport().set_input_as_handled()
	elif event is InputEventJoypadMotion and capturing:
		get_viewport().set_input_as_handled()

# The raw snapshot of the selected device (RgControls documents the format): keys / mouse from the events seen
# here, the selected pad polled (or injected by a test). The wheel and mouse motion are consumed.
func _snapshot_now() -> Dictionary:
	var pads := []
	var dev := _device()
	if not dev.is_empty() and _is_pad(dev):
		var slot := int(dev["slot"])
		if bool(dev["connected"]) or not _pad_override.is_empty():
			if not _pad_override.is_empty() and int(_pad_override["slot"]) == slot:
				pads.append(_pad_override.duplicate())
			elif bool(dev["connected"]) and slot >= 0:
				var pressed := PackedInt32Array()
				for b in range(POLLED_PAD_BUTTONS):
					if Input.is_joy_button_pressed(slot, b as JoyButton):
						pressed.append(b)
				var axes := PackedFloat32Array()
				for a in range(POLLED_PAD_AXES):
					axes.append(Input.get_joy_axis(slot, a as JoyAxis))
				pads.append({"slot": slot, "buttons": pressed, "axes": axes})
	var buttons := PackedInt32Array()
	for b in _held_mouse.keys():
		buttons.append(int(b))
	var snap := {
		"keys": PackedStringArray(_held_keys.keys()), "mouse_buttons": buttons, "pads": pads,
		"mouse_dx": _motion.x, "mouse_dy": _motion.y,
		"wheel_up": _wheel["up"], "wheel_down": _wheel["down"], "wheel_left": _wheel["left"], "wheel_right": _wheel["right"],
	}
	_motion = Vector2.ZERO
	_wheel = {"up": 0.0, "down": 0.0, "left": 0.0, "right": 0.0}
	return snap

# ---- per frame ---------------------------------------------------------------------------------------------------------

func _process(delta: float) -> void:
	if controls == null or not controls.is_ready() or not is_visible_in_tree():
		return
	var rev: int = controls.get_revision()
	if rev != _revision:
		# something outside this screen changed the controls (a pad plugged in, a reload)
		refresh()
	var snap := _snapshot_now()
	if not _capture.is_empty():
		var state := str(controls.capture_feed(snap, delta))
		_overlay_progress.value = float(controls.get_capture_hold_progress())
		if state == "captured":
			_finish_capture()
		elif state == "cancelled":
			_cancel_capture("Cancelled")
	elif not _calib.is_empty():
		_feed_calibration(snap)
	_update_monitor(snap)

func _feed_calibration(snap: Dictionary) -> void:
	var pads: Array = snap.get("pads", [])
	if pads.is_empty():
		return
	var axes: PackedFloat32Array = pads[0]["axes"]
	var axis := int(_calib["axis"])
	if axis >= axes.size():
		return
	controls.calibration_feed(axes[axis])
	var cal: Dictionary = controls.get_calibration()
	_overlay_extra.text = "Now %.2f   seen %.2f to %.2f   (rest %.2f)%s" % [axes[axis], float(cal["low"]), float(cal["high"]), float(cal["rest"]),
		"" if bool(cal["valid"]) else "   - move it further" ]
	_overlay_apply.disabled = not bool(cal["valid"])

func _update_monitor(snap: Dictionary) -> void:
	var dev := _device()
	if dev.is_empty():
		return
	var pads: Array = snap.get("pads", [])
	var show_axes := not pads.is_empty()
	_mon_raw_box.visible = show_axes
	if show_axes:
		var axes: PackedFloat32Array = pads[0]["axes"]
		for i in range(RAW_AXIS_ROWS):
			var entry: Dictionary = _mon_raw[i]
			var v := axes[i] if i < axes.size() else 0.0
			# the usual six axes are always listed, a seventh / eighth only while it moves
			(entry["box"] as Control).visible = i < 6 or absf(v) > 0.01
			(entry["bar"] as ProgressBar).value = v
			(entry["value"] as Label).text = "%.2f" % v
		var held := PackedStringArray()
		for b in pads[0]["buttons"]:
			held.append(str(b))
		_mon_digital.text = "Buttons: " + (", ".join(held) if not held.is_empty() else "-")
	elif str(dev["class"]) == "mouse":
		var held := PackedStringArray()
		for b in _held_mouse.keys():
			held.append(str(b))
		_mon_digital.text = "Mouse buttons: " + (", ".join(held) if not held.is_empty() else "-")
	else:
		var keys: PackedStringArray = snap["keys"]
		_mon_digital.text = "Keys: " + (", ".join(keys) if keys.size() > 0 else "-")
	var mapped: Array = controls.monitor_device(_device_key, snap)
	for i in range(MAPPED_ROWS):
		var entry: Dictionary = _mon_mapped[i]
		var box: Control = entry["box"]
		if i >= mapped.size():
			box.visible = false
			continue
		var m: Dictionary = mapped[i]
		box.visible = true
		(entry["name"] as Label).text = str(m["label"])
		var bar := entry["bar"] as ProgressBar
		bar.min_value = -1.0 if str(m["range"]) == "signed" else 0.0
		bar.value = float(m["value"])
		var text := "%.2f" % float(m["value"])
		if str(m["range"]) != "button" and absf(float(m["raw"]) - float(m["value"])) > 0.005:
			text += " (raw %.2f)" % float(m["raw"])
		elif str(m["range"]) == "button":
			text = "pressed" if float(m["value"]) != 0.0 else "pulse"
		(entry["value"] as Label).text = text
