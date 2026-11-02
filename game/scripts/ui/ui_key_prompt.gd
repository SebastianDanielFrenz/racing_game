extends HBoxContainer
# game/scripts/ui/ui_key_prompt.gd - the design language's key prompt (docs/design_language.md): a key cap and what it does.
#     const UiKeyPrompt := preload("res://scripts/ui/ui_key_prompt.gd")
#     panel_column.add_child(UiKeyPrompt.new("L", "Turn on lights"))
# The cap takes a word as well as a letter, so a gamepad button or "Space" fits. A prompt that cannot be used right now stays
# in place, muted: set_available(false). The list of prompts belongs in a UiPanel, not bare on the scene.

const UiTheme := preload("res://scripts/ui/ui_theme.gd")

var _cap := Label.new()
var _action := Label.new()

func _init(key := "", action := "") -> void:
	add_theme_constant_override(&"separation", int(UiTheme.token(&"key.gap")))
	var box := UiTheme.style(&"surface_chip", &"border_strong")
	box.set_corner_radius_all(int(UiTheme.token(&"radius.chip")))
	box.set_border_width_all(int(UiTheme.token(&"stroke.hair")))
	box.content_margin_left = UiTheme.token(&"key.cap_padding")
	box.content_margin_right = UiTheme.token(&"key.cap_padding")
	_cap.add_theme_stylebox_override(&"normal", box)
	_cap.add_theme_font_size_override(&"font_size", int(UiTheme.token(&"type.small")))
	_cap.custom_minimum_size = Vector2(UiTheme.token(&"key.cap_min_width"), UiTheme.token(&"key.cap_height"))
	_cap.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_cap.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	_action.add_theme_font_size_override(&"font_size", int(UiTheme.token(&"type.body")))
	_action.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	add_child(_cap)
	add_child(_action)
	setup(key, action)
	set_available(true)

func setup(key: String, action: String) -> void:
	_cap.text = key
	_action.text = action

func set_available(available: bool) -> void:
	var role := &"text" if available else &"text_muted"
	UiTheme.text(_cap, role)
	UiTheme.text(_action, role)
