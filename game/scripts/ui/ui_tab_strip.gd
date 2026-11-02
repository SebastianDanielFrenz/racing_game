extends HBoxContainer
# game/scripts/ui/ui_tab_strip.gd - the design language's tab strip (docs/design_language.md): icon tabs in a row, the selected
# one filled with the accent and underlined, so it reads without hue.
#     const UiTabStrip := preload("res://scripts/ui/ui_tab_strip.gd")
#     var tabs := UiTabStrip.new()
#     tabs.add_tab(&"shop", shop_icon, "Shop")
#     tabs.tab_selected.connect(_show_page)
#     tabs.select(&"shop")
# Icons are white line drawings on transparent (docs/design_language.md), the strip tints them. The label is the tooltip, and
# the tab's text when there is no icon. Mouse, keyboard and gamepad reach it the same way: the tabs are Buttons, ui_left and
# ui_right move the focus ring, ui_accept selects.

const UiTheme := preload("res://scripts/ui/ui_theme.gd")

signal tab_selected(id: StringName)

const COLOUR_ITEMS := [
	&"font_color", &"font_hover_color", &"font_pressed_color", &"font_focus_color", &"font_hover_pressed_color",
	&"icon_normal_color", &"icon_hover_color", &"icon_pressed_color", &"icon_focus_color", &"icon_hover_pressed_color",
]
const BOX_ITEMS := [&"normal", &"hover", &"pressed", &"hover_pressed"]

var selected: StringName = &""
var _tabs: Dictionary = {} # id -> Button

func _init() -> void:
	add_theme_constant_override(&"separation", int(UiTheme.token(&"tab.gap")))

func add_tab(id: StringName, icon: Texture2D, label: String) -> Button:
	var tab := Button.new()
	var side := UiTheme.token(&"tab.height")
	tab.custom_minimum_size = Vector2(side, side)
	tab.tooltip_text = label
	tab.focus_mode = Control.FOCUS_ALL
	if icon != null:
		tab.icon = icon
		tab.expand_icon = true
		tab.icon_alignment = HORIZONTAL_ALIGNMENT_CENTER
	else:
		tab.text = label
	var ring := UiTheme.style(&"hover_tint", &"focus_ring")
	ring.draw_center = false
	ring.set_border_width_all(int(UiTheme.token(&"stroke.focus")))
	ring.set_corner_radius_all(int(UiTheme.token(&"radius.control")))
	tab.add_theme_stylebox_override(&"focus", ring)
	tab.pressed.connect(select.bind(id))
	_tabs[id] = tab
	add_child(tab)
	_paint(tab, false)
	return tab

func select(id: StringName) -> void:
	if not _tabs.has(id):
		push_error("UiTabStrip: no tab named %s" % id)
		return
	var changed := id != selected
	selected = id
	for key: StringName in _tabs:
		_paint(_tabs[key], key == id)
	if changed:
		tab_selected.emit(id)

func _box(fill: StringName, edge: StringName) -> StyleBoxFlat:
	var box := UiTheme.style(fill, edge)
	box.set_corner_radius_all(int(UiTheme.token(&"radius.control")))
	var margin := (UiTheme.token(&"tab.height") - UiTheme.token(&"tab.icon")) * 0.5
	box.set_content_margin_all(margin)
	return box

func _paint(tab: Button, on: bool) -> void:
	UiTheme.bind_font(tab, &"text_on_accent" if on else &"text_control", COLOUR_ITEMS)
	for item: StringName in BOX_ITEMS:
		var fill := &"surface_control"
		if on:
			fill = &"accent"
		elif item == &"hover":
			fill = &"surface_control_hover"
		elif item != &"normal":
			fill = &"surface_control_pressed"
		var box := _box(fill, &"border_selected" if on else &"")
		box.border_width_bottom = int(UiTheme.token(&"tab.selected_underline")) if on else 0
		tab.add_theme_stylebox_override(item, box)
