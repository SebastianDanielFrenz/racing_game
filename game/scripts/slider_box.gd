extends RefCounted
# A typed-value box beside a plain HSlider (garage numeric inputs, 2026-10-13): the number the slider shows can also be
# typed. Enter or leaving the box applies it; a value outside the slider's range, or text that is no number, is taken back
# with the reason (tooltip and red text, and `on_reject` when given). The slider stays the one source of truth: the box only
# sets slider.value, so every existing value_changed handler runs exactly as for a drag. Click-focus only, so a gamepad (and
# Tab) skip the box; input_map.gd ignores game keys while a LineEdit has the focus.
#
# `scale` converts the slider's unit to the displayed one (metres to centimetres: 100), `unit` is a suffix typed or not.
# The garage option rows have their own boxes (shell_ui.gd, rg core conversions) because their mapping and limits come from
# the option data; this helper is for sliders whose limits are the slider's own.

const BOX_WIDTH := 78.0
const REJECT_COLOUR := Color(1.0, 0.45, 0.35)

static func attach(parent: Control, slider: HSlider, decimals: int = 2, scale: float = 1.0, unit: String = "", on_reject: Callable = Callable()) -> LineEdit:
	var box := LineEdit.new()
	box.name = "Entry_" + slider.name if slider.name != "" else "Entry"
	box.custom_minimum_size = Vector2(BOX_WIDTH, 0)
	box.alignment = HORIZONTAL_ALIGNMENT_RIGHT
	box.focus_mode = Control.FOCUS_CLICK
	box.select_all_on_focus = true
	box.context_menu_enabled = false
	box.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	var normal_tip := "Type a value%s and press Enter. Allowed: %s to %s%s." % [
		" in " + unit if unit != "" else "", _format(slider.min_value * scale, decimals), _format(slider.max_value * scale, decimals),
		" " + unit if unit != "" else ""]
	box.tooltip_text = normal_tip
	box.text = _format(slider.value * scale, decimals)
	box.set_meta("shown", box.text)
	slider.value_changed.connect(func(v: float):
		if box.has_focus():
			return
		box.text = _format(v * scale, decimals)
		box.set_meta("shown", box.text)
		box.remove_theme_color_override("font_color")
		box.tooltip_text = normal_tip)
	var commit := func(text: String):
		var typed := text.strip_edges()
		if typed == str(box.get_meta("shown", "")):
			return
		var number := parse_number(typed, unit)
		var reason := ""
		if is_nan(number):
			reason = "\"%s\" is not a number." % typed
		else:
			var value := number / scale
			var eps := 1e-9 * maxf(1.0, absf(slider.max_value))
			if value < slider.min_value - eps or value > slider.max_value + eps:
				reason = "%s is outside the allowed range %s to %s%s." % [_format(number, decimals), _format(slider.min_value * scale, decimals), _format(slider.max_value * scale, decimals), " " + unit if unit != "" else ""]
			else:
				slider.value = clampf(value, slider.min_value, slider.max_value) # snaps to the step, runs value_changed
		if reason != "":
			box.add_theme_color_override("font_color", REJECT_COLOUR)
			box.tooltip_text = reason
			if on_reject.is_valid():
				on_reject.call(reason)
		else:
			box.remove_theme_color_override("font_color")
			box.tooltip_text = normal_tip
		box.text = _format(slider.value * scale, decimals)
		box.set_meta("shown", box.text)
	box.text_submitted.connect(commit)
	box.focus_exited.connect(func(): commit.call(box.text))
	parent.add_child(box)
	return box

static func _format(value: float, decimals: int) -> String:
	return ("%." + str(maxi(decimals, 0)) + "f") % value

# "8000", "8 000", "52,5", "52.5 cm", "x1.05": NAN when the text is no number.
static func parse_number(text: String, unit: String = "") -> float:
	var t := text.strip_edges().replace(",", ".").replace(" ", "")
	if unit != "" and t.to_lower().ends_with(unit.to_lower()):
		t = t.substr(0, t.length() - unit.length())
	if t.begins_with("x") or t.begins_with("X"):
		t = t.substr(1)
	if t.is_empty() or not t.is_valid_float():
		return NAN
	return t.to_float()
